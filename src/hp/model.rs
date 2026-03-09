use std::f64::consts::PI;

use crate::hp::config::{
    CYLINDER_COUNT, DEFAULT_WIEBE_A, DEFAULT_WIEBE_M, EngineConfig, HeadlessConfig,
};
use crate::hp::math::{
    R_AIR, bidirectional_orifice_mass_flow, cp_air, crossed_angle, rpm_to_rad_s,
    runner_pressure_loss_pa, shaft_power_kw, wrap_deg,
};

const MIN_PRESSURE_PA: f64 = 5_000.0;
const MIN_TEMP_K: f64 = 180.0;
const MIN_MASS_KG: f64 = 1.0e-7;
const MAX_FACE_FLOW_KG_S: f64 = 5.0;

#[derive(Debug, Clone)]
pub struct TorqueCurveSample {
    pub rpm: f64,
    pub indicated_torque_nm: f64,
    pub brake_torque_nm: f64,
    pub indicated_power_kw: f64,
    pub brake_power_kw: f64,
    pub trapped_air_mg: f64,
    pub volumetric_efficiency: f64,
    pub bmep_bar: f64,
}

#[derive(Debug, Clone)]
pub struct PvRecord {
    pub cycle_deg: f64,
    pub cylinder_index: usize,
    pub volume_m3: f64,
    pub pressure_pa: f64,
    pub temperature_k: f64,
    pub mass_kg: f64,
    pub intake_lift_m: f64,
    pub exhaust_lift_m: f64,
    pub gas_torque_nm: f64,
}

#[derive(Debug, Clone)]
pub struct SweepResult {
    pub samples: Vec<TorqueCurveSample>,
    pub pv_records_by_rpm: Vec<(f64, Vec<PvRecord>)>,
}

#[derive(Debug, Clone)]
pub struct OperatingPointResult {
    pub sample: TorqueCurveSample,
    pub pv_records: Vec<PvRecord>,
}

#[derive(Debug, Clone)]
struct CylinderState {
    mass_kg: f64,
    temp_k: f64,
}

#[derive(Debug, Clone)]
struct RunnerLineState {
    cell_pressures_pa: Vec<f64>,
    face_mass_flows_kg_s: Vec<f64>,
}

#[derive(Debug, Clone)]
struct SystemState {
    plenum_pressure_pa: f64,
    collector_pressure_pa: f64,
    cylinders: Vec<CylinderState>,
    intake_lines: Vec<RunnerLineState>,
    exhaust_lines: Vec<RunnerLineState>,
}

#[derive(Debug, Clone)]
struct CylinderDerivative {
    d_mass_kg_s: f64,
    d_temp_k_s: f64,
}

#[derive(Debug, Clone)]
struct RunnerLineDerivative {
    d_cell_pressures_pa_s: Vec<f64>,
    d_face_mass_flows_kg_s2: Vec<f64>,
}

#[derive(Debug, Clone)]
struct SystemDerivative {
    d_plenum_pressure_pa_s: f64,
    d_collector_pressure_pa_s: f64,
    cylinders: Vec<CylinderDerivative>,
    intake_lines: Vec<RunnerLineDerivative>,
    exhaust_lines: Vec<RunnerLineDerivative>,
}

#[derive(Debug, Clone)]
struct CylinderTracker {
    phase_offset_deg: f64,
    fuel_mass_cycle_kg: f64,
}

#[derive(Debug, Clone)]
struct FlowSnapshot {
    throttle_flow_kg_s: f64,
    tailpipe_flow_kg_s: f64,
    intake_valve_flows_kg_s: Vec<f64>,
    exhaust_valve_flows_kg_s: Vec<f64>,
}

#[derive(Debug, Clone)]
struct OperatingPointSimulator<'a> {
    config: &'a HeadlessConfig,
    omega_rad_s: f64,
    throttle: f64,
    theta_deg: f64,
    state: SystemState,
    trackers: Vec<CylinderTracker>,
}

pub fn run_sweep(config: &HeadlessConfig) -> Result<SweepResult, String> {
    let mut samples = Vec::new();
    let mut pv_records_by_rpm = Vec::new();
    for &rpm in &config.sweep.rpm_points {
        let write_pv = config
            .sweep
            .pv_rpm_points
            .iter()
            .any(|candidate| (*candidate - rpm).abs() < 0.5);
        let (sample, pv_records) = run_operating_point(config, rpm, write_pv)?;
        samples.push(sample);
        if write_pv {
            pv_records_by_rpm.push((rpm, pv_records));
        }
    }
    Ok(SweepResult {
        samples,
        pv_records_by_rpm,
    })
}

pub fn run_point(
    config: &HeadlessConfig,
    rpm: f64,
    write_pv: bool,
) -> Result<OperatingPointResult, String> {
    let (sample, pv_records) = run_operating_point(config, rpm, write_pv)?;
    Ok(OperatingPointResult { sample, pv_records })
}

fn run_operating_point(
    config: &HeadlessConfig,
    rpm: f64,
    write_pv: bool,
) -> Result<(TorqueCurveSample, Vec<PvRecord>), String> {
    if rpm <= 0.0 {
        return Err("RPM points must be positive".to_string());
    }
    let mut sim = OperatingPointSimulator::new(config, rpm)?;
    let steps_per_cycle = (720.0 / config.numerics.dt_deg.max(1.0e-6)).round() as usize;
    let step_deg = 720.0 / steps_per_cycle.max(1) as f64;
    let sample_start_step = config
        .numerics
        .warmup_cycles
        .saturating_mul(steps_per_cycle);
    let total_steps = sample_start_step.saturating_add(
        config
            .numerics
            .sample_cycles
            .saturating_mul(steps_per_cycle),
    );
    let pv_start_step = total_steps.saturating_sub(steps_per_cycle);

    let mut torque_sum = 0.0;
    let mut torque_samples = 0usize;
    let mut trapped_air_sum = 0.0;
    let mut trapped_air_samples = 0usize;
    let mut pv_records = Vec::new();

    for step_index in 0..total_steps {
        let prev_theta_deg = sim.theta_deg;
        let prev_state = sim.state.clone();
        sim.step_rk4(step_deg);
        let next_theta_deg = sim.theta_deg;
        let next_state = sim.state.clone();

        if step_index >= sample_start_step {
            torque_sum +=
                sim.indicated_torque_step(&prev_state, &next_state, prev_theta_deg, step_deg);
            torque_samples = torque_samples.saturating_add(1);
        }

        for cylinder_index in 0..CYLINDER_COUNT {
            let local_prev_deg = sim.local_theta_deg(cylinder_index, prev_theta_deg);
            let local_next_deg = sim.local_theta_deg(cylinder_index, next_theta_deg);
            if step_index >= sample_start_step
                && crossed_angle(
                    local_prev_deg,
                    local_next_deg,
                    config.engine.intake_close_deg,
                )
            {
                trapped_air_sum += next_state.cylinders[cylinder_index].mass_kg;
                trapped_air_samples = trapped_air_samples.saturating_add(1);
            }

            if write_pv && step_index >= pv_start_step {
                pv_records.push(sim.pv_record_for_step(
                    cylinder_index,
                    &next_state,
                    next_theta_deg,
                ));
            }
        }
    }

    let omega = sim.omega_rad_s;
    let friction_torque = config.engine.friction_c0_nm
        + config.engine.friction_c1_nms * omega
        + config.engine.friction_c2_nms2 * omega.powi(2);
    let indicated_torque_nm = torque_sum / torque_samples.max(1) as f64;
    let brake_torque_nm = indicated_torque_nm - friction_torque;
    let trapped_air_kg = trapped_air_sum / trapped_air_samples.max(1) as f64;
    let displaced_per_cyl = sim.displacement_per_cylinder_m3();
    let rho_amb = config.engine.ambient_pressure_pa / (R_AIR * config.engine.intake_temp_k);
    let volumetric_efficiency = trapped_air_kg / (rho_amb * displaced_per_cyl).max(f64::EPSILON);
    let displacement_total = displaced_per_cyl * CYLINDER_COUNT as f64;
    let bmep_bar = if displacement_total <= 0.0 {
        0.0
    } else {
        4.0 * PI * brake_torque_nm / displacement_total * 1.0e-5
    };

    Ok((
        TorqueCurveSample {
            rpm,
            indicated_torque_nm,
            brake_torque_nm,
            indicated_power_kw: shaft_power_kw(rpm, indicated_torque_nm),
            brake_power_kw: shaft_power_kw(rpm, brake_torque_nm),
            trapped_air_mg: trapped_air_kg * 1.0e6,
            volumetric_efficiency,
            bmep_bar,
        },
        pv_records,
    ))
}

impl<'a> OperatingPointSimulator<'a> {
    fn new(config: &'a HeadlessConfig, rpm: f64) -> Result<Self, String> {
        let cylinders = CYLINDER_COUNT;
        let base_mass = config.engine.ambient_pressure_pa
            * Self::cylinder_volume_static(&config.engine, 180.0)
            / (R_AIR * config.engine.intake_temp_k);
        let intake_line = Self::new_runner_line(
            config.numerics.intake_runner_cells,
            config.engine.ambient_pressure_pa,
        )?;
        let exhaust_line = Self::new_runner_line(
            config.numerics.exhaust_runner_cells,
            config.engine.ambient_pressure_pa,
        )?;
        let displaced_per_cyl = Self::displacement_per_cylinder_static(&config.engine);
        let trapped_ambient = config.engine.ambient_pressure_pa * displaced_per_cyl
            / (R_AIR * config.engine.intake_temp_k);
        let fuel_mass_cycle_kg = trapped_ambient
            / (config.combustion.stoich_afr * config.combustion.lambda_target).max(f64::EPSILON);
        Ok(Self {
            config,
            omega_rad_s: rpm_to_rad_s(rpm),
            throttle: config.sweep.throttle.clamp(0.0, 1.0),
            theta_deg: 0.0,
            state: SystemState {
                plenum_pressure_pa: config.engine.ambient_pressure_pa,
                collector_pressure_pa: config.engine.ambient_pressure_pa,
                cylinders: (0..cylinders)
                    .map(|_| CylinderState {
                        mass_kg: base_mass.max(MIN_MASS_KG),
                        temp_k: config.engine.intake_temp_k,
                    })
                    .collect(),
                intake_lines: (0..cylinders).map(|_| intake_line.clone()).collect(),
                exhaust_lines: (0..cylinders).map(|_| exhaust_line.clone()).collect(),
            },
            trackers: (0..cylinders)
                .map(|index| CylinderTracker {
                    phase_offset_deg: index as f64 * 720.0 / cylinders as f64,
                    fuel_mass_cycle_kg,
                })
                .collect(),
        })
    }

    fn new_runner_line(cells: usize, pressure_pa: f64) -> Result<RunnerLineState, String> {
        if cells == 0 {
            return Err("runner cell count must be at least 1".to_string());
        }
        Ok(RunnerLineState {
            cell_pressures_pa: vec![pressure_pa; cells],
            face_mass_flows_kg_s: vec![0.0; cells],
        })
    }

    fn step_rk4(&mut self, delta_theta_deg: f64) {
        let dt_s = delta_theta_deg.to_radians() / self.omega_rad_s.max(f64::EPSILON);
        let theta0 = self.theta_deg;
        let k1 = self.derivatives(&self.state, theta0);
        let s2 = self.state.add_scaled(&k1, 0.5 * dt_s);
        let k2 = self.derivatives(&s2, theta0 + 0.5 * delta_theta_deg);
        let s3 = self.state.add_scaled(&k2, 0.5 * dt_s);
        let k3 = self.derivatives(&s3, theta0 + 0.5 * delta_theta_deg);
        let s4 = self.state.add_scaled(&k3, dt_s);
        let k4 = self.derivatives(&s4, theta0 + delta_theta_deg);
        self.state = self.state.rk4_combine(&k1, &k2, &k3, &k4, dt_s);
        self.theta_deg = wrap_deg(theta0 + delta_theta_deg);
        self.update_fueling_events(theta0, self.theta_deg);
    }

    fn update_fueling_events(&mut self, prev_theta_deg: f64, next_theta_deg: f64) {
        for cylinder_index in 0..CYLINDER_COUNT {
            let local_prev_deg = self.local_theta_deg(cylinder_index, prev_theta_deg);
            let local_next_deg = self.local_theta_deg(cylinder_index, next_theta_deg);
            if crossed_angle(
                local_prev_deg,
                local_next_deg,
                self.config.engine.intake_close_deg,
            ) {
                let trapped_air_kg = self.state.cylinders[cylinder_index]
                    .mass_kg
                    .max(MIN_MASS_KG);
                self.trackers[cylinder_index].fuel_mass_cycle_kg = trapped_air_kg
                    / (self.config.combustion.stoich_afr * self.config.combustion.lambda_target)
                        .max(f64::EPSILON);
            }
        }
    }

    fn derivatives(&self, state: &SystemState, theta_deg: f64) -> SystemDerivative {
        let cp = cp_air();
        let cv = cp - R_AIR;
        let flow = self.flow_snapshot(state, theta_deg);

        let cylinders = state
            .cylinders
            .iter()
            .enumerate()
            .map(|(cylinder_index, cyl)| {
                let local_theta_deg = self.local_theta_deg(cylinder_index, theta_deg);
                let volume = self.cylinder_volume(local_theta_deg);
                let dvolume_dt =
                    self.cylinder_volume_derivative_rad(local_theta_deg) * self.omega_rad_s;
                let pressure_pa = self.cylinder_pressure_pa(cyl, volume);
                let intake_temp = self.config.engine.intake_temp_k;
                let exhaust_temp = self.config.engine.exhaust_temp_k;
                let m_dot_iv = flow.intake_valve_flows_kg_s[cylinder_index];
                let m_dot_ev = flow.exhaust_valve_flows_kg_s[cylinder_index];
                let (mass_in, enthalpy_in) =
                    inflow_terms(m_dot_iv, intake_temp, m_dot_ev, exhaust_temp, cp);
                let (mass_out, enthalpy_out) = outflow_terms(m_dot_iv, m_dot_ev, cyl.temp_k, cp);
                let dm_dt = mass_in - mass_out;
                let q_comb_dot = self.heat_release_rate_w(cylinder_index, local_theta_deg);
                let q_wall_dot = self.config.combustion.wall_htc_w_m2k
                    * self.cylinder_surface_area(local_theta_deg)
                    * (cyl.temp_k - self.config.engine.wall_temp_k).max(0.0);
                let d_u_dt =
                    q_comb_dot - q_wall_dot - pressure_pa * dvolume_dt + enthalpy_in - enthalpy_out;
                let d_temp_dt =
                    (d_u_dt - cv * cyl.temp_k * dm_dt) / (cyl.mass_kg.max(MIN_MASS_KG) * cv);
                CylinderDerivative {
                    d_mass_kg_s: dm_dt,
                    d_temp_k_s: d_temp_dt,
                }
            })
            .collect();

        let intake_lines = self.intake_line_derivatives(state, &flow);
        let exhaust_lines = self.exhaust_line_derivatives(state, &flow);
        let plenum_outflow: f64 = state
            .intake_lines
            .iter()
            .map(|line| line.face_mass_flows_kg_s[0])
            .sum();
        let collector_inflow: f64 = state
            .exhaust_lines
            .iter()
            .map(|line| line.face_mass_flows_kg_s.last().copied().unwrap_or(0.0))
            .sum();

        SystemDerivative {
            d_plenum_pressure_pa_s: R_AIR * self.config.engine.intake_temp_k
                / self.config.engine.plenum_volume_m3.max(1.0e-8)
                * (flow.throttle_flow_kg_s - plenum_outflow),
            d_collector_pressure_pa_s: R_AIR * self.config.engine.exhaust_temp_k
                / self.config.engine.collector_volume_m3.max(1.0e-8)
                * (collector_inflow - flow.tailpipe_flow_kg_s),
            cylinders,
            intake_lines,
            exhaust_lines,
        }
    }

    fn intake_line_derivatives(
        &self,
        state: &SystemState,
        flow: &FlowSnapshot,
    ) -> Vec<RunnerLineDerivative> {
        let area = runner_area(self.config.engine.intake_runner_diameter_m);
        let seg_length = self.config.engine.intake_runner_length_m
            / self.config.numerics.intake_runner_cells.max(1) as f64;
        let seg_volume = area * seg_length;
        let temp_k = self.config.engine.intake_temp_k;

        state
            .intake_lines
            .iter()
            .enumerate()
            .map(|(cylinder_index, line)| {
                let mut d_faces = vec![0.0; line.face_mass_flows_kg_s.len()];
                let mut d_cells = vec![0.0; line.cell_pressures_pa.len()];
                for face_index in 0..line.face_mass_flows_kg_s.len() {
                    let p_left = if face_index == 0 {
                        state.plenum_pressure_pa
                    } else {
                        line.cell_pressures_pa[face_index - 1]
                    };
                    let p_right = line.cell_pressures_pa[face_index];
                    let density = 0.5 * (p_left + p_right) / (R_AIR * temp_k);
                    let loss = runner_pressure_loss_pa(
                        line.face_mass_flows_kg_s[face_index],
                        density.max(1.0e-6),
                        area.max(1.0e-9),
                        self.config.engine.intake_runner_loss_coeff
                            / line.face_mass_flows_kg_s.len().max(1) as f64,
                    );
                    d_faces[face_index] =
                        area / seg_length.max(1.0e-6) * ((p_left - p_right) - loss);
                }
                for cell_index in 0..line.cell_pressures_pa.len() {
                    let inflow = line.face_mass_flows_kg_s[cell_index];
                    let outflow = if cell_index + 1 < line.cell_pressures_pa.len() {
                        line.face_mass_flows_kg_s[cell_index + 1]
                    } else {
                        flow.intake_valve_flows_kg_s[cylinder_index]
                    };
                    d_cells[cell_index] =
                        R_AIR * temp_k / seg_volume.max(1.0e-9) * (inflow - outflow);
                }
                RunnerLineDerivative {
                    d_cell_pressures_pa_s: d_cells,
                    d_face_mass_flows_kg_s2: d_faces,
                }
            })
            .collect()
    }

    fn exhaust_line_derivatives(
        &self,
        state: &SystemState,
        flow: &FlowSnapshot,
    ) -> Vec<RunnerLineDerivative> {
        let area = runner_area(self.config.engine.exhaust_runner_diameter_m);
        let seg_length = self.config.engine.exhaust_runner_length_m
            / self.config.numerics.exhaust_runner_cells.max(1) as f64;
        let seg_volume = area * seg_length;
        let temp_k = self.config.engine.exhaust_temp_k;

        state
            .exhaust_lines
            .iter()
            .enumerate()
            .map(|(cylinder_index, line)| {
                let mut d_faces = vec![0.0; line.face_mass_flows_kg_s.len()];
                let mut d_cells = vec![0.0; line.cell_pressures_pa.len()];
                for face_index in 0..line.face_mass_flows_kg_s.len() {
                    let p_left = line.cell_pressures_pa[face_index];
                    let p_right = if face_index + 1 < line.cell_pressures_pa.len() {
                        line.cell_pressures_pa[face_index + 1]
                    } else {
                        state.collector_pressure_pa
                    };
                    let density = 0.5 * (p_left + p_right) / (R_AIR * temp_k);
                    let loss = runner_pressure_loss_pa(
                        line.face_mass_flows_kg_s[face_index],
                        density.max(1.0e-6),
                        area.max(1.0e-9),
                        self.config.engine.exhaust_runner_loss_coeff
                            / line.face_mass_flows_kg_s.len().max(1) as f64,
                    );
                    d_faces[face_index] =
                        area / seg_length.max(1.0e-6) * ((p_left - p_right) - loss);
                }
                for cell_index in 0..line.cell_pressures_pa.len() {
                    let inflow = if cell_index == 0 {
                        flow.exhaust_valve_flows_kg_s[cylinder_index]
                    } else {
                        line.face_mass_flows_kg_s[cell_index - 1]
                    };
                    let outflow = line.face_mass_flows_kg_s[cell_index];
                    d_cells[cell_index] =
                        R_AIR * temp_k / seg_volume.max(1.0e-9) * (inflow - outflow);
                }
                RunnerLineDerivative {
                    d_cell_pressures_pa_s: d_cells,
                    d_face_mass_flows_kg_s2: d_faces,
                }
            })
            .collect()
    }

    fn flow_snapshot(&self, state: &SystemState, theta_deg: f64) -> FlowSnapshot {
        let throttle_cda = self.config.engine.throttle_discharge_coeff
            * self.config.engine.throttle_area_max_m2
            * throttle_area_fraction(self.throttle);
        let tailpipe_cda =
            self.config.engine.tailpipe_discharge_coeff * self.config.engine.tailpipe_area_m2;
        let throttle_flow_kg_s = bidirectional_orifice_mass_flow(
            throttle_cda,
            self.config.engine.ambient_pressure_pa,
            self.config.engine.intake_temp_k,
            state.plenum_pressure_pa,
            self.config.engine.intake_temp_k,
        )
        .max(0.0);
        let tailpipe_flow_kg_s = bidirectional_orifice_mass_flow(
            tailpipe_cda,
            state.collector_pressure_pa,
            self.config.engine.exhaust_temp_k,
            self.config.engine.ambient_pressure_pa,
            self.config.engine.intake_temp_k,
        )
        .max(0.0);

        let mut intake_valve_flows_kg_s = Vec::with_capacity(state.cylinders.len());
        let mut exhaust_valve_flows_kg_s = Vec::with_capacity(state.cylinders.len());
        for cylinder_index in 0..state.cylinders.len() {
            let local_theta_deg = self.local_theta_deg(cylinder_index, theta_deg);
            let intake_cda = self.config.engine.intake_valve_cd
                * effective_valve_area_m2(
                    self.config.engine.intake_valve_diameter_m,
                    self.intake_valve_lift_m(local_theta_deg),
                );
            let exhaust_cda = self.config.engine.exhaust_valve_cd
                * effective_valve_area_m2(
                    self.config.engine.exhaust_valve_diameter_m,
                    self.exhaust_valve_lift_m(local_theta_deg),
                );
            let volume = self.cylinder_volume(local_theta_deg);
            let pressure_pa = self.cylinder_pressure_pa(&state.cylinders[cylinder_index], volume);
            let intake_port_pa = *state.intake_lines[cylinder_index]
                .cell_pressures_pa
                .last()
                .unwrap_or(&state.plenum_pressure_pa);
            let exhaust_port_pa = *state.exhaust_lines[cylinder_index]
                .cell_pressures_pa
                .first()
                .unwrap_or(&state.collector_pressure_pa);
            intake_valve_flows_kg_s.push(bidirectional_orifice_mass_flow(
                intake_cda,
                intake_port_pa,
                self.config.engine.intake_temp_k,
                pressure_pa,
                state.cylinders[cylinder_index].temp_k,
            ));
            exhaust_valve_flows_kg_s.push(bidirectional_orifice_mass_flow(
                exhaust_cda,
                pressure_pa,
                state.cylinders[cylinder_index].temp_k,
                exhaust_port_pa,
                self.config.engine.exhaust_temp_k,
            ));
        }

        FlowSnapshot {
            throttle_flow_kg_s,
            tailpipe_flow_kg_s,
            intake_valve_flows_kg_s,
            exhaust_valve_flows_kg_s,
        }
    }

    fn local_theta_deg(&self, cylinder_index: usize, global_theta_deg: f64) -> f64 {
        wrap_deg(global_theta_deg + self.trackers[cylinder_index].phase_offset_deg)
    }

    fn displacement_per_cylinder_m3(&self) -> f64 {
        Self::displacement_per_cylinder_static(&self.config.engine)
    }

    fn displacement_per_cylinder_static(engine: &EngineConfig) -> f64 {
        0.25 * PI * engine.bore_m.powi(2) * engine.stroke_m
    }

    fn cylinder_volume(&self, theta_deg: f64) -> f64 {
        Self::cylinder_volume_static(&self.config.engine, theta_deg)
    }

    fn cylinder_volume_static(engine: &EngineConfig, theta_deg: f64) -> f64 {
        let crank_radius = 0.5 * engine.stroke_m;
        let rod = engine.conrod_m;
        let theta = theta_deg.to_radians();
        let piston_area = 0.25 * PI * engine.bore_m.powi(2);
        let x = crank_radius * (1.0 - theta.cos()) + rod
            - (rod.powi(2) - crank_radius.powi(2) * theta.sin().powi(2))
                .max(1.0e-12)
                .sqrt();
        let displacement = Self::displacement_per_cylinder_static(engine);
        let clearance = displacement / (engine.compression_ratio - 1.0).max(1.0e-6);
        clearance + piston_area * x
    }

    fn cylinder_volume_derivative_rad(&self, theta_deg: f64) -> f64 {
        let crank_radius = 0.5 * self.config.engine.stroke_m;
        let rod = self.config.engine.conrod_m;
        let theta = theta_deg.to_radians();
        let piston_area = 0.25 * PI * self.config.engine.bore_m.powi(2);
        let root = (rod.powi(2) - crank_radius.powi(2) * theta.sin().powi(2))
            .max(1.0e-12)
            .sqrt();
        let dx_dtheta =
            crank_radius * theta.sin() + crank_radius.powi(2) * theta.sin() * theta.cos() / root;
        piston_area * dx_dtheta
    }

    fn cylinder_surface_area(&self, theta_deg: f64) -> f64 {
        let piston_area = 0.25 * PI * self.config.engine.bore_m.powi(2);
        let displacement = self.cylinder_volume(theta_deg) - self.clearance_volume_m3();
        let height = displacement / piston_area.max(1.0e-9);
        2.0 * piston_area + PI * self.config.engine.bore_m * height.max(0.0)
    }

    fn clearance_volume_m3(&self) -> f64 {
        let displacement = self.displacement_per_cylinder_m3();
        displacement / (self.config.engine.compression_ratio - 1.0).max(1.0e-6)
    }

    fn cylinder_pressure_pa(&self, cylinder: &CylinderState, volume_m3: f64) -> f64 {
        (cylinder.mass_kg.max(MIN_MASS_KG) * R_AIR * cylinder.temp_k.max(MIN_TEMP_K))
            / volume_m3.max(1.0e-9)
    }

    fn intake_valve_lift_m(&self, theta_deg: f64) -> f64 {
        smooth_lift(
            theta_deg,
            self.config.engine.intake_open_deg,
            self.config.engine.intake_close_deg,
            self.config.engine.intake_max_lift_m,
        )
    }

    fn exhaust_valve_lift_m(&self, theta_deg: f64) -> f64 {
        smooth_lift(
            theta_deg,
            self.config.engine.exhaust_open_deg,
            self.config.engine.exhaust_close_deg,
            self.config.engine.exhaust_max_lift_m,
        )
    }

    fn heat_release_rate_w(&self, cylinder_index: usize, local_theta_deg: f64) -> f64 {
        let combustion = &self.config.combustion;
        let start_deg = wrap_deg(720.0 - combustion.ignition_advance_deg);
        let duration_deg = combustion.burn_duration_deg.max(1.0e-6);
        let mut rel_deg = local_theta_deg - start_deg;
        if rel_deg < 0.0 {
            rel_deg += 720.0;
        }
        let x = rel_deg / duration_deg;
        if !(0.0..=1.0).contains(&x) {
            return 0.0;
        }
        let dx_ddeg = DEFAULT_WIEBE_A * (DEFAULT_WIEBE_M + 1.0) / duration_deg
            * x.powf(DEFAULT_WIEBE_M)
            * (-DEFAULT_WIEBE_A * x.powf(DEFAULT_WIEBE_M + 1.0)).exp();
        self.trackers[cylinder_index].fuel_mass_cycle_kg
            * combustion.fuel_lhv_j_per_kg
            * dx_ddeg
            * self.omega_rad_s.to_degrees()
    }

    fn indicated_torque_step(
        &self,
        prev_state: &SystemState,
        next_state: &SystemState,
        prev_theta_deg: f64,
        delta_theta_deg: f64,
    ) -> f64 {
        let delta_theta_rad = delta_theta_deg.to_radians().max(1.0e-9);
        let mut torque_sum = 0.0;
        for cylinder_index in 0..CYLINDER_COUNT {
            let prev_local_deg = self.local_theta_deg(cylinder_index, prev_theta_deg);
            let next_local_deg = wrap_deg(prev_local_deg + delta_theta_deg);
            let v_prev = self.cylinder_volume(prev_local_deg);
            let v_next = self.cylinder_volume(next_local_deg);
            let p_prev = self.cylinder_pressure_pa(&prev_state.cylinders[cylinder_index], v_prev);
            let p_next = self.cylinder_pressure_pa(&next_state.cylinders[cylinder_index], v_next);
            torque_sum += 0.5 * (p_prev + p_next) * (v_next - v_prev) / delta_theta_rad;
        }
        torque_sum
    }

    fn pv_record_for_step(
        &self,
        cylinder_index: usize,
        state: &SystemState,
        theta_deg: f64,
    ) -> PvRecord {
        let local_theta_deg = self.local_theta_deg(cylinder_index, theta_deg);
        let volume_m3 = self.cylinder_volume(local_theta_deg);
        let pressure_pa = self.cylinder_pressure_pa(&state.cylinders[cylinder_index], volume_m3);
        PvRecord {
            cycle_deg: local_theta_deg,
            cylinder_index: cylinder_index + 1,
            volume_m3,
            pressure_pa,
            temperature_k: state.cylinders[cylinder_index].temp_k,
            mass_kg: state.cylinders[cylinder_index].mass_kg,
            intake_lift_m: self.intake_valve_lift_m(local_theta_deg),
            exhaust_lift_m: self.exhaust_valve_lift_m(local_theta_deg),
            gas_torque_nm: pressure_pa * self.cylinder_volume_derivative_rad(local_theta_deg),
        }
    }
}

impl SystemState {
    fn add_scaled(&self, derivative: &SystemDerivative, scale: f64) -> Self {
        let mut next = self.clone();
        next.plenum_pressure_pa += derivative.d_plenum_pressure_pa_s * scale;
        next.collector_pressure_pa += derivative.d_collector_pressure_pa_s * scale;
        for (state, deriv) in next.cylinders.iter_mut().zip(&derivative.cylinders) {
            state.mass_kg += deriv.d_mass_kg_s * scale;
            state.temp_k += deriv.d_temp_k_s * scale;
        }
        for (line, deriv) in next.intake_lines.iter_mut().zip(&derivative.intake_lines) {
            for (pressure, delta) in line
                .cell_pressures_pa
                .iter_mut()
                .zip(&deriv.d_cell_pressures_pa_s)
            {
                *pressure += delta * scale;
            }
            for (flow, delta) in line
                .face_mass_flows_kg_s
                .iter_mut()
                .zip(&deriv.d_face_mass_flows_kg_s2)
            {
                *flow += delta * scale;
            }
        }
        for (line, deriv) in next.exhaust_lines.iter_mut().zip(&derivative.exhaust_lines) {
            for (pressure, delta) in line
                .cell_pressures_pa
                .iter_mut()
                .zip(&deriv.d_cell_pressures_pa_s)
            {
                *pressure += delta * scale;
            }
            for (flow, delta) in line
                .face_mass_flows_kg_s
                .iter_mut()
                .zip(&deriv.d_face_mass_flows_kg_s2)
            {
                *flow += delta * scale;
            }
        }
        next.sanitize();
        next
    }

    fn rk4_combine(
        &self,
        k1: &SystemDerivative,
        k2: &SystemDerivative,
        k3: &SystemDerivative,
        k4: &SystemDerivative,
        dt: f64,
    ) -> Self {
        let mut next = self.clone();
        next.plenum_pressure_pa += dt / 6.0
            * (k1.d_plenum_pressure_pa_s
                + 2.0 * k2.d_plenum_pressure_pa_s
                + 2.0 * k3.d_plenum_pressure_pa_s
                + k4.d_plenum_pressure_pa_s);
        next.collector_pressure_pa += dt / 6.0
            * (k1.d_collector_pressure_pa_s
                + 2.0 * k2.d_collector_pressure_pa_s
                + 2.0 * k3.d_collector_pressure_pa_s
                + k4.d_collector_pressure_pa_s);
        for index in 0..next.cylinders.len() {
            next.cylinders[index].mass_kg += dt / 6.0
                * (k1.cylinders[index].d_mass_kg_s
                    + 2.0 * k2.cylinders[index].d_mass_kg_s
                    + 2.0 * k3.cylinders[index].d_mass_kg_s
                    + k4.cylinders[index].d_mass_kg_s);
            next.cylinders[index].temp_k += dt / 6.0
                * (k1.cylinders[index].d_temp_k_s
                    + 2.0 * k2.cylinders[index].d_temp_k_s
                    + 2.0 * k3.cylinders[index].d_temp_k_s
                    + k4.cylinders[index].d_temp_k_s);
        }
        for index in 0..next.intake_lines.len() {
            for cell in 0..next.intake_lines[index].cell_pressures_pa.len() {
                next.intake_lines[index].cell_pressures_pa[cell] += dt / 6.0
                    * (k1.intake_lines[index].d_cell_pressures_pa_s[cell]
                        + 2.0 * k2.intake_lines[index].d_cell_pressures_pa_s[cell]
                        + 2.0 * k3.intake_lines[index].d_cell_pressures_pa_s[cell]
                        + k4.intake_lines[index].d_cell_pressures_pa_s[cell]);
            }
            for face in 0..next.intake_lines[index].face_mass_flows_kg_s.len() {
                next.intake_lines[index].face_mass_flows_kg_s[face] += dt / 6.0
                    * (k1.intake_lines[index].d_face_mass_flows_kg_s2[face]
                        + 2.0 * k2.intake_lines[index].d_face_mass_flows_kg_s2[face]
                        + 2.0 * k3.intake_lines[index].d_face_mass_flows_kg_s2[face]
                        + k4.intake_lines[index].d_face_mass_flows_kg_s2[face]);
            }
        }
        for index in 0..next.exhaust_lines.len() {
            for cell in 0..next.exhaust_lines[index].cell_pressures_pa.len() {
                next.exhaust_lines[index].cell_pressures_pa[cell] += dt / 6.0
                    * (k1.exhaust_lines[index].d_cell_pressures_pa_s[cell]
                        + 2.0 * k2.exhaust_lines[index].d_cell_pressures_pa_s[cell]
                        + 2.0 * k3.exhaust_lines[index].d_cell_pressures_pa_s[cell]
                        + k4.exhaust_lines[index].d_cell_pressures_pa_s[cell]);
            }
            for face in 0..next.exhaust_lines[index].face_mass_flows_kg_s.len() {
                next.exhaust_lines[index].face_mass_flows_kg_s[face] += dt / 6.0
                    * (k1.exhaust_lines[index].d_face_mass_flows_kg_s2[face]
                        + 2.0 * k2.exhaust_lines[index].d_face_mass_flows_kg_s2[face]
                        + 2.0 * k3.exhaust_lines[index].d_face_mass_flows_kg_s2[face]
                        + k4.exhaust_lines[index].d_face_mass_flows_kg_s2[face]);
            }
        }
        next.sanitize();
        next
    }

    fn sanitize(&mut self) {
        self.plenum_pressure_pa = self.plenum_pressure_pa.max(MIN_PRESSURE_PA);
        self.collector_pressure_pa = self.collector_pressure_pa.max(MIN_PRESSURE_PA);
        for cylinder in &mut self.cylinders {
            cylinder.mass_kg = cylinder.mass_kg.max(MIN_MASS_KG);
            cylinder.temp_k = cylinder.temp_k.max(MIN_TEMP_K);
        }
        for line in self
            .intake_lines
            .iter_mut()
            .chain(self.exhaust_lines.iter_mut())
        {
            for pressure in &mut line.cell_pressures_pa {
                *pressure = pressure.max(MIN_PRESSURE_PA);
            }
            for flow in &mut line.face_mass_flows_kg_s {
                *flow = flow.clamp(-MAX_FACE_FLOW_KG_S, MAX_FACE_FLOW_KG_S);
            }
        }
    }
}

fn inflow_terms(
    m_dot_iv: f64,
    intake_temp_k: f64,
    m_dot_ev: f64,
    exhaust_temp_k: f64,
    cp: f64,
) -> (f64, f64) {
    let mut mass_in = 0.0;
    let mut enthalpy_in = 0.0;
    if m_dot_iv > 0.0 {
        mass_in += m_dot_iv;
        enthalpy_in += m_dot_iv * cp * intake_temp_k;
    }
    if m_dot_ev < 0.0 {
        mass_in += -m_dot_ev;
        enthalpy_in += -m_dot_ev * cp * exhaust_temp_k;
    }
    (mass_in, enthalpy_in)
}

fn outflow_terms(m_dot_iv: f64, m_dot_ev: f64, cyl_temp_k: f64, cp: f64) -> (f64, f64) {
    let mut mass_out = 0.0;
    let mut enthalpy_out = 0.0;
    if m_dot_iv < 0.0 {
        mass_out += -m_dot_iv;
        enthalpy_out += -m_dot_iv * cp * cyl_temp_k;
    }
    if m_dot_ev > 0.0 {
        mass_out += m_dot_ev;
        enthalpy_out += m_dot_ev * cp * cyl_temp_k;
    }
    (mass_out, enthalpy_out)
}

fn runner_area(diameter_m: f64) -> f64 {
    0.25 * PI * diameter_m.max(1.0e-6).powi(2)
}

fn effective_valve_area_m2(valve_diameter_m: f64, lift_m: f64) -> f64 {
    let curtain = PI * valve_diameter_m.max(0.0) * lift_m.max(0.0);
    let seat = runner_area(valve_diameter_m);
    curtain.min(seat)
}

fn throttle_area_fraction(throttle: f64) -> f64 {
    (0.005 + 0.995 * throttle.clamp(0.0, 1.0).powf(1.7)).clamp(0.005, 1.0)
}

fn smooth_lift(theta_deg: f64, open_deg: f64, close_deg: f64, max_lift_m: f64) -> f64 {
    let theta = wrap_deg(theta_deg);
    let open = wrap_deg(open_deg);
    let close = wrap_deg(close_deg);
    let duration = if close >= open {
        close - open
    } else {
        close + 720.0 - open
    };
    let mut rel = theta - open;
    if rel < 0.0 {
        rel += 720.0;
    }
    if rel > duration {
        return 0.0;
    }
    0.5 * max_lift_m.max(0.0) * (1.0 - (2.0 * PI * rel / duration.max(1.0e-9)).cos())
}

#[cfg(test)]
mod tests {
    use super::{OperatingPointSimulator, run_sweep, smooth_lift};
    use crate::hp::config::{EngineConfig, HeadlessConfig, NumericsConfig, SweepConfig};
    use crate::hp::yaml::parse_document;

    fn reference_config() -> HeadlessConfig {
        HeadlessConfig {
            output_dir: "dist/hp".to_string(),
            engine: EngineConfig {
                bore_m: 0.0805,
                stroke_m: 0.0976,
                conrod_m: 0.154,
                compression_ratio: 13.0,
                ambient_pressure_pa: 101_325.0,
                intake_temp_k: 305.0,
                exhaust_temp_k: 930.0,
                wall_temp_k: 420.0,
                plenum_volume_m3: 0.0032,
                collector_volume_m3: 0.0050,
                throttle_area_max_m2: 0.0027,
                throttle_discharge_coeff: 0.82,
                tailpipe_area_m2: 0.0022,
                tailpipe_discharge_coeff: 0.87,
                intake_runner_length_m: 0.36,
                intake_runner_diameter_m: 0.034,
                intake_runner_loss_coeff: 1.6,
                exhaust_runner_length_m: 0.74,
                exhaust_runner_diameter_m: 0.036,
                exhaust_runner_loss_coeff: 1.9,
                friction_c0_nm: 5.1,
                friction_c1_nms: 0.0040,
                friction_c2_nms2: 0.000020,
                intake_open_deg: 340.0,
                intake_close_deg: 580.0,
                exhaust_open_deg: 140.0,
                exhaust_close_deg: 380.0,
                intake_valve_diameter_m: 0.032,
                exhaust_valve_diameter_m: 0.028,
                intake_valve_cd: 0.72,
                exhaust_valve_cd: 0.74,
                intake_max_lift_m: 0.0105,
                exhaust_max_lift_m: 0.0098,
            },
            combustion: crate::hp::config::CombustionConfig {
                stoich_afr: 14.7,
                lambda_target: 0.88,
                fuel_lhv_j_per_kg: 43.0e6,
                ignition_advance_deg: 20.0,
                burn_duration_deg: 42.0,
                wall_htc_w_m2k: 220.0,
            },
            sweep: SweepConfig {
                throttle: 1.0,
                rpm_points: vec![2_500.0, 4_500.0],
                pv_rpm_points: vec![2_500.0],
            },
            numerics: NumericsConfig {
                dt_deg: 0.5,
                intake_runner_cells: 4,
                exhaust_runner_cells: 4,
                warmup_cycles: 2,
                sample_cycles: 2,
            },
        }
    }

    fn preset_config(path: &str) -> HeadlessConfig {
        let text = match path {
            "config/presets/nissan_tiida_hr16de.yaml" => {
                include_str!("../../config/presets/nissan_tiida_hr16de.yaml")
            }
            "config/presets/honda_s2000_f20c.yaml" => {
                include_str!("../../config/presets/honda_s2000_f20c.yaml")
            }
            _ => panic!("unknown preset path {path}"),
        };
        let parsed = parse_document(text).expect("preset yaml should parse");
        HeadlessConfig::from_yaml(&parsed).expect("preset should validate")
    }

    fn sample_at_rpm(
        result: &crate::hp::model::SweepResult,
        rpm: f64,
    ) -> &crate::hp::model::TorqueCurveSample {
        result
            .samples
            .iter()
            .find(|sample| (sample.rpm - rpm).abs() < 0.5)
            .expect("rpm point should exist")
    }

    #[test]
    fn valve_lift_profile_is_zero_outside_window() {
        assert_eq!(smooth_lift(100.0, 340.0, 580.0, 0.01), 0.0);
        assert!(smooth_lift(450.0, 340.0, 580.0, 0.01) > 0.0);
    }

    #[test]
    fn equilibrium_state_has_finite_derivatives() {
        let cfg = reference_config();
        let sim = OperatingPointSimulator::new(&cfg, 2_000.0).expect("sim should build");
        let deriv = sim.derivatives(&sim.state, 90.0);
        assert!(deriv.d_plenum_pressure_pa_s.is_finite());
        assert!(deriv.d_collector_pressure_pa_s.is_finite());
        assert!(
            deriv
                .cylinders
                .iter()
                .all(|cyl| cyl.d_mass_kg_s.is_finite() && cyl.d_temp_k_s.is_finite())
        );
    }

    #[test]
    fn reference_sweep_returns_positive_brake_torque() {
        let cfg = reference_config();
        let result = run_sweep(&cfg).expect("sweep should run");
        assert_eq!(result.samples.len(), 2);
        assert!(
            result
                .samples
                .iter()
                .all(|sample| sample.brake_torque_nm.is_finite() && sample.brake_torque_nm > 20.0)
        );
    }

    #[test]
    fn honda_s2000_preset_tracks_official_high_rpm_band() {
        let cfg = preset_config("config/presets/honda_s2000_f20c.yaml");
        let result = run_sweep(&cfg).expect("sweep should run");
        let torque_7500 = sample_at_rpm(&result, 7_500.0).brake_torque_nm;
        let power_8300 = sample_at_rpm(&result, 8_300.0).brake_power_kw;
        assert!(
            (228.0..=245.0).contains(&torque_7500),
            "Honda preset should stay near the F20C high-rpm torque band, got {torque_7500:.1} Nm"
        );
        assert!(
            (185.0..=195.0).contains(&power_8300),
            "Honda preset should stay near the F20C high-rpm power band, got {power_8300:.1} kW"
        );
    }

    #[test]
    fn nissan_hr16de_preset_stays_in_current_tiida_approximation_band() {
        let cfg = preset_config("config/presets/nissan_tiida_hr16de.yaml");
        let result = run_sweep(&cfg).expect("sweep should run");
        let torque_4400 = sample_at_rpm(&result, 4_400.0).brake_torque_nm;
        let power_6000 = sample_at_rpm(&result, 6_000.0).brake_power_kw;
        assert!(
            (166.0..=171.0).contains(&torque_4400),
            "Nissan preset should stay in the current HR16DE / Tiida approximation band, got {torque_4400:.1} Nm"
        );
        assert!(
            (83.0..=86.0).contains(&power_6000),
            "Nissan preset should stay in the current HR16DE / Tiida approximation band, got {power_6000:.1} kW"
        );
    }
}
