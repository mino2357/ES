use std::collections::VecDeque;
use std::f64::consts::{PI, TAU};
use std::time::Instant;

use crate::config::{
    AppConfig, AutoControlConfig, BenchConfig, BenchMixtureMode, CamConfig, ControlDefaults,
    EnvironmentConfig, ExternalLoadConfig, ModelConfig, NumericsConfig, PlotConfig, PvModelConfig,
    VolumetricEfficiencyConfig,
};
use crate::constants::{
    FIXED_CYLINDER_COUNT, FUEL_LHV_J_PER_KG, GAMMA_AIR, R_AIR, W_PER_KW, W_PER_MECHANICAL_HP,
};

const DEFAULT_INTAKE_RUNNER_LENGTH_M: f64 = 0.36;
const DEFAULT_EXHAUST_RUNNER_LENGTH_M: f64 = 0.74;
const DEFAULT_INTAKE_RUNNER_DIAMETER_M: f64 = 0.034;
const DEFAULT_EXHAUST_RUNNER_DIAMETER_M: f64 = 0.036;

#[derive(Debug, Clone)]
pub(crate) struct EngineParams {
    pub(crate) displacement_m3: f64,
    pub(crate) compression_ratio: f64,
    pub(crate) bore_m: f64,
    pub(crate) stroke_m: f64,
    pub(crate) inertia_kgm2: f64,
    pub(crate) friction_c0_nm: f64,
    pub(crate) friction_c1_nms: f64,
    pub(crate) friction_c2_nms2: f64,
    pub(crate) intake_volume_m3: f64,
    pub(crate) intake_runner_volume_m3: f64,
    pub(crate) exhaust_volume_m3: f64,
    pub(crate) exhaust_runner_volume_m3: f64,
    pub(crate) throttle_area_max_m2: f64,
    pub(crate) tailpipe_area_m2: f64,
    pub(crate) idle_target_rpm: f64,
    pub(crate) max_rpm: f64,
}

#[derive(Debug, Clone)]
pub(crate) struct ControlInput {
    pub(crate) throttle_cmd: f64,
    pub(crate) load_cmd: f64,
    pub(crate) starter_cmd: bool,
    pub(crate) spark_cmd: bool,
    pub(crate) fuel_cmd: bool,
    pub(crate) ignition_timing_deg: f64,
    pub(crate) vvt_intake_deg: f64,
    pub(crate) vvt_exhaust_deg: f64,
}

impl Default for ControlInput {
    fn default() -> Self {
        let defaults = ControlDefaults::default();
        Self::from_defaults(&defaults)
    }
}

impl ControlInput {
    fn from_defaults(defaults: &ControlDefaults) -> Self {
        Self {
            throttle_cmd: defaults.throttle_cmd,
            load_cmd: defaults.load_cmd,
            starter_cmd: defaults.starter_cmd,
            spark_cmd: defaults.spark_cmd,
            fuel_cmd: defaults.fuel_cmd,
            ignition_timing_deg: defaults.ignition_timing_deg,
            vvt_intake_deg: defaults.vvt_intake_deg,
            vvt_exhaust_deg: defaults.vvt_exhaust_deg,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum WotSearchAxis {
    Ignition,
    IntakeVvt,
    ExhaustVvt,
}

impl WotSearchAxis {
    fn next(self) -> Self {
        match self {
            Self::Ignition => Self::IntakeVvt,
            Self::IntakeVvt => Self::ExhaustVvt,
            Self::ExhaustVvt => Self::Ignition,
        }
    }

    pub(crate) fn label(self) -> &'static str {
        match self {
            Self::Ignition => "Ignition",
            Self::IntakeVvt => "VVT intake",
            Self::ExhaustVvt => "VVT exhaust",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum WotSearchPhase {
    Idle,
    Cranking,
    Baseline,
    Candidate,
}

impl WotSearchPhase {
    pub(crate) fn label(self) -> &'static str {
        match self {
            Self::Idle => "Idle",
            Self::Cranking => "Cranking",
            Self::Baseline => "Baseline",
            Self::Candidate => "Candidate",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct WotSearchPoint {
    pub(crate) ignition_deg: f64,
    pub(crate) vvt_intake_deg: f64,
    pub(crate) vvt_exhaust_deg: f64,
}

impl WotSearchPoint {
    fn from_control(control: &ControlInput) -> Self {
        Self {
            ignition_deg: control.ignition_timing_deg,
            vvt_intake_deg: control.vvt_intake_deg,
            vvt_exhaust_deg: control.vvt_exhaust_deg,
        }
    }
}

#[derive(Debug, Clone)]
pub(crate) struct AutoController {
    pub(crate) enabled: bool,
    pub(crate) wot_efficiency_enabled: bool,
    pub(crate) integral: f64,
    pub(crate) wot_phase: WotSearchPhase,
    pub(crate) wot_axis: WotSearchAxis,
    pub(crate) wot_best_point: WotSearchPoint,
    pub(crate) wot_best_eta: f64,
    wot_baseline_point: WotSearchPoint,
    wot_candidate_point: WotSearchPoint,
    wot_baseline_eta: f64,
    wot_direction: f64,
    wot_eval_time_s: f64,
    wot_eval_sum: f64,
    wot_eval_samples: u32,
    wot_initialized: bool,
}

impl Default for AutoController {
    fn default() -> Self {
        Self {
            enabled: false,
            wot_efficiency_enabled: false,
            integral: 0.0,
            wot_phase: WotSearchPhase::Idle,
            wot_axis: WotSearchAxis::Ignition,
            wot_best_point: WotSearchPoint {
                ignition_deg: 0.0,
                vvt_intake_deg: 0.0,
                vvt_exhaust_deg: 0.0,
            },
            wot_best_eta: 0.0,
            wot_baseline_point: WotSearchPoint {
                ignition_deg: 0.0,
                vvt_intake_deg: 0.0,
                vvt_exhaust_deg: 0.0,
            },
            wot_candidate_point: WotSearchPoint {
                ignition_deg: 0.0,
                vvt_intake_deg: 0.0,
                vvt_exhaust_deg: 0.0,
            },
            wot_baseline_eta: 0.0,
            wot_direction: 1.0,
            wot_eval_time_s: 0.0,
            wot_eval_sum: 0.0,
            wot_eval_samples: 0,
            wot_initialized: false,
        }
    }
}

impl AutoController {
    pub(crate) fn reset_wot_search(&mut self, control: &ControlInput) {
        let point = WotSearchPoint::from_control(control);
        self.wot_phase = if self.wot_efficiency_enabled {
            WotSearchPhase::Cranking
        } else {
            WotSearchPhase::Idle
        };
        self.wot_axis = WotSearchAxis::Ignition;
        self.wot_best_point = point;
        self.wot_best_eta = 0.0;
        self.wot_baseline_point = point;
        self.wot_candidate_point = point;
        self.wot_baseline_eta = 0.0;
        self.wot_direction = 1.0;
        self.wot_eval_time_s = 0.0;
        self.wot_eval_sum = 0.0;
        self.wot_eval_samples = 0;
        self.wot_initialized = false;
    }

    fn reset_wot_window(&mut self) {
        self.wot_eval_time_s = 0.0;
        self.wot_eval_sum = 0.0;
        self.wot_eval_samples = 0;
    }

    fn current_wot_point(&self) -> WotSearchPoint {
        match self.wot_phase {
            WotSearchPhase::Candidate => self.wot_candidate_point,
            _ => self.wot_best_point,
        }
    }

    fn initialize_wot_search(
        &mut self,
        control: &ControlInput,
        rpm: f64,
        load: f64,
        tuning: &AutoControlConfig,
        model: &ModelConfig,
    ) {
        let mut point =
            clamp_wot_search_point(WotSearchPoint::from_control(control), tuning, model);
        point.ignition_deg = estimate_mbt_deg(model, rpm, load);
        self.wot_best_point = point;
        self.wot_baseline_point = point;
        self.wot_candidate_point = point;
        self.wot_best_eta = 0.0;
        self.wot_baseline_eta = 0.0;
        self.wot_axis = WotSearchAxis::Ignition;
        self.wot_direction = 1.0;
        self.wot_phase = WotSearchPhase::Baseline;
        self.wot_initialized = true;
        self.reset_wot_window();
    }

    fn seed_candidate_from_best(
        &mut self,
        tuning: &AutoControlConfig,
        model: &ModelConfig,
    ) -> bool {
        self.wot_candidate_point = match self.wot_axis {
            WotSearchAxis::Ignition => step_wot_search_point(
                self.wot_best_point,
                self.wot_axis,
                self.wot_direction,
                tuning,
                model,
            ),
            WotSearchAxis::IntakeVvt => {
                let mut point = self.wot_best_point;
                point.vvt_intake_deg = tuning.wot_search_vvt_min_deg;
                clamp_wot_search_point(point, tuning, model)
            }
            WotSearchAxis::ExhaustVvt => {
                let mut point = self.wot_best_point;
                point.vvt_exhaust_deg = tuning.wot_search_vvt_min_deg;
                clamp_wot_search_point(point, tuning, model)
            }
        };
        if self.wot_axis != WotSearchAxis::Ignition
            && self.wot_candidate_point == self.wot_best_point
        {
            return self.advance_vvt_scan_candidate(tuning, model);
        }
        self.wot_candidate_point != self.wot_best_point
    }

    fn advance_vvt_scan_candidate(
        &mut self,
        tuning: &AutoControlConfig,
        model: &ModelConfig,
    ) -> bool {
        let (current_value, step) = match self.wot_axis {
            WotSearchAxis::IntakeVvt => (
                self.wot_candidate_point.vvt_intake_deg,
                tuning.wot_search_vvt_intake_step_deg.abs(),
            ),
            WotSearchAxis::ExhaustVvt => (
                self.wot_candidate_point.vvt_exhaust_deg,
                tuning.wot_search_vvt_exhaust_step_deg.abs(),
            ),
            WotSearchAxis::Ignition => return false,
        };
        let next_value = current_value + step.max(f64::EPSILON);
        if next_value > tuning.wot_search_vvt_max_deg + 1.0e-9 {
            return false;
        }
        let mut point = self.wot_best_point;
        match self.wot_axis {
            WotSearchAxis::IntakeVvt => point.vvt_intake_deg = next_value,
            WotSearchAxis::ExhaustVvt => point.vvt_exhaust_deg = next_value,
            WotSearchAxis::Ignition => {}
        }
        self.wot_candidate_point = clamp_wot_search_point(point, tuning, model);
        true
    }

    fn advance_wot_axis(&mut self) {
        self.wot_axis = self.wot_axis.next();
        self.wot_direction = 1.0;
        self.wot_phase = WotSearchPhase::Baseline;
        self.reset_wot_window();
    }

    pub(crate) fn apply(
        &mut self,
        control: &mut ControlInput,
        rpm: f64,
        running: bool,
        idle_target_rpm: f64,
        dt: f64,
        tuning: &AutoControlConfig,
        model: &ModelConfig,
        load: f64,
    ) {
        if self.wot_efficiency_enabled {
            control.spark_cmd = true;
            control.fuel_cmd = true;
            control.throttle_cmd = tuning.wot_target_throttle.clamp(0.0, 1.0);

            let cranking = !running || rpm < tuning.cranking_rpm_threshold;
            control.starter_cmd = cranking && rpm < tuning.starter_cutoff_rpm;

            if !self.wot_initialized {
                self.initialize_wot_search(control, rpm, load, tuning, model);
            }
            if !running || rpm < tuning.wot_search_min_rpm {
                self.wot_phase = WotSearchPhase::Cranking;
                self.reset_wot_window();
            } else if self.wot_phase == WotSearchPhase::Cranking {
                self.wot_phase = WotSearchPhase::Baseline;
                self.reset_wot_window();
            }

            let point = clamp_wot_search_point(self.current_wot_point(), tuning, model);
            control.ignition_timing_deg = point.ignition_deg;
            control.vvt_intake_deg = point.vvt_intake_deg;
            control.vvt_exhaust_deg = point.vvt_exhaust_deg;
            return;
        }

        // Auto mode is deliberately simple: force spark/fuel on, then PI the throttle to idle.
        if !self.enabled {
            return;
        }

        control.spark_cmd = true;
        control.fuel_cmd = true;

        let cranking = !running || rpm < tuning.cranking_rpm_threshold;
        control.starter_cmd = cranking && rpm < tuning.starter_cutoff_rpm;

        let target_rpm = if control.starter_cmd {
            tuning.starter_target_rpm
        } else {
            idle_target_rpm
        };
        let error = target_rpm - rpm;

        let (kp, ki, base) = if control.starter_cmd {
            (
                tuning.starter_kp,
                tuning.starter_ki,
                tuning.starter_base_throttle,
            )
        } else {
            (tuning.run_kp, tuning.run_ki, tuning.run_base_throttle)
        };

        self.integral =
            (self.integral + error * dt).clamp(tuning.integral_min, tuning.integral_max);
        control.throttle_cmd = (base + kp * error + ki * self.integral)
            .clamp(tuning.throttle_min, tuning.throttle_max);

        if rpm > target_rpm + tuning.integral_relief_rpm_band {
            self.integral *= tuning.integral_relief_factor;
        }
    }

    pub(crate) fn observe_wot_efficiency(
        &mut self,
        observation: &Observation,
        dt: f64,
        tuning: &AutoControlConfig,
        model: &ModelConfig,
    ) {
        if !self.wot_efficiency_enabled {
            return;
        }
        if self.wot_phase == WotSearchPhase::Cranking
            || observation.rpm < tuning.wot_search_min_rpm
            || observation.indicated_work_cycle_j <= 0.0
            || !observation.eta_thermal_indicated_pv.is_finite()
        {
            self.reset_wot_window();
            return;
        }

        self.wot_eval_time_s += dt;
        self.wot_eval_sum += observation.eta_thermal_indicated_pv.max(0.0);
        self.wot_eval_samples = self.wot_eval_samples.saturating_add(1);
        if self.wot_eval_time_s < tuning.wot_search_eval_time_s || self.wot_eval_samples == 0 {
            return;
        }

        let avg_eta = self.wot_eval_sum / self.wot_eval_samples as f64;
        self.reset_wot_window();

        match self.wot_phase {
            WotSearchPhase::Baseline => {
                self.wot_baseline_point = self.wot_best_point;
                self.wot_baseline_eta = avg_eta;
                self.wot_best_eta = self.wot_best_eta.max(avg_eta);
                self.wot_direction = 1.0;
                if self.seed_candidate_from_best(tuning, model) {
                    self.wot_phase = WotSearchPhase::Candidate;
                } else {
                    self.advance_wot_axis();
                }
            }
            WotSearchPhase::Candidate => {
                if avg_eta > self.wot_best_eta {
                    self.wot_best_eta = avg_eta;
                }

                if self.wot_axis == WotSearchAxis::Ignition {
                    if avg_eta > self.wot_baseline_eta + tuning.wot_search_min_improvement {
                        self.wot_best_point = self.wot_candidate_point;
                        self.wot_baseline_point = self.wot_best_point;
                        self.wot_baseline_eta = avg_eta;
                        if !self.seed_candidate_from_best(tuning, model) {
                            self.advance_wot_axis();
                        }
                    } else if self.wot_direction > 0.0 {
                        self.wot_direction = -1.0;
                        if !self.seed_candidate_from_best(tuning, model) {
                            self.advance_wot_axis();
                        }
                    } else {
                        self.advance_wot_axis();
                    }
                } else {
                    if avg_eta > self.wot_baseline_eta + tuning.wot_search_min_improvement {
                        self.wot_best_point = self.wot_candidate_point;
                        self.wot_baseline_eta = avg_eta;
                    }
                    if !self.advance_vvt_scan_candidate(tuning, model) {
                        self.advance_wot_axis();
                    }
                }
            }
            WotSearchPhase::Idle | WotSearchPhase::Cranking => {}
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub(crate) struct EngineState {
    pub(crate) omega_rad_s: f64,
    pub(crate) theta_rad: f64,
    pub(crate) p_intake_pa: f64,
    pub(crate) p_intake_runner_pa: f64,
    pub(crate) p_exhaust_pa: f64,
    pub(crate) p_exhaust_runner_pa: f64,
    pub(crate) m_dot_intake_runner_kg_s: f64,
    pub(crate) m_dot_exhaust_runner_kg_s: f64,
    pub(crate) throttle_eff: f64,
    pub(crate) running: bool,
}

impl Default for EngineState {
    fn default() -> Self {
        let env = EnvironmentConfig::default();
        let model = ModelConfig::default();
        Self {
            omega_rad_s: 0.0,
            theta_rad: 0.0,
            p_intake_pa: model.initial_intake_pressure_pa,
            p_intake_runner_pa: model.initial_intake_pressure_pa,
            p_exhaust_pa: env.ambient_pressure_pa,
            p_exhaust_runner_pa: env.ambient_pressure_pa,
            m_dot_intake_runner_kg_s: 0.0,
            m_dot_exhaust_runner_kg_s: 0.0,
            throttle_eff: model.initial_throttle_eff,
            running: false,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub(crate) struct Derivatives {
    d_omega: f64,
    d_theta: f64,
    d_p_intake: f64,
    d_p_intake_runner: f64,
    d_p_exhaust: f64,
    d_p_exhaust_runner: f64,
    d_m_dot_intake_runner: f64,
    d_m_dot_exhaust_runner: f64,
    d_throttle: f64,
}

#[derive(Debug, Clone, Copy)]
pub(crate) struct EvalPoint {
    m_dot_throttle: f64,
    m_dot_cyl_mean: f64,
    m_dot_cyl: f64,
    m_dot_exhaust_in: f64,
    m_dot_tailpipe: f64,
    torque_combustion_nm: f64,
    torque_friction_nm: f64,
    torque_starter_nm: f64,
    torque_pumping_nm: f64,
    torque_load_nm: f64,
    p_intake_cyl_pa: f64,
    p_exhaust_cyl_pa: f64,
    trapped_air_kg: f64,
    fuel_mass_cycle_cyl_kg: f64,
    combustion_rate_norm: f64,
    combustion_enabled: bool,
    burn_start_deg: f64,
    burn_duration_deg: f64,
    phase_eff: f64,
    ve_effective: f64,
    intake_wave_current_pa: f64,
    exhaust_wave_current_pa: f64,
    intake_ram_multiplier: f64,
    exhaust_scavenge_multiplier: f64,
    ve_pulse_multiplier: f64,
    lambda_target: f64,
    intake_charge_temp_k: f64,
    heat_loss_cycle_j: f64,
    ignition_timing_deg: f64,
    exhaust_temp_k: f64,
}

#[derive(Debug, Clone, Copy)]
struct WaveActionState {
    intake_wave_current_pa: f64,
    exhaust_wave_current_pa: f64,
    intake_boundary_wave_pa: f64,
    exhaust_boundary_wave_pa: f64,
    intake_flow_multiplier: f64,
    exhaust_flow_multiplier: f64,
    intake_ram_multiplier: f64,
    exhaust_scavenge_multiplier: f64,
    ve_pulse_multiplier: f64,
    scavenging_head_norm: f64,
}

impl Default for WaveActionState {
    fn default() -> Self {
        Self {
            intake_wave_current_pa: 0.0,
            exhaust_wave_current_pa: 0.0,
            intake_boundary_wave_pa: 0.0,
            exhaust_boundary_wave_pa: 0.0,
            intake_flow_multiplier: 1.0,
            exhaust_flow_multiplier: 1.0,
            intake_ram_multiplier: 1.0,
            exhaust_scavenge_multiplier: 1.0,
            ve_pulse_multiplier: 1.0,
            scavenging_head_norm: 0.0,
        }
    }
}

#[derive(Debug, Clone, Copy)]
struct ChargeState {
    intake_charge_temp_k: f64,
    total_charge_mass_kg: f64,
    mixture_gamma: f64,
    burned_gas_cp_j_per_kgk: f64,
}

#[allow(dead_code)]
#[derive(Debug, Clone, Copy)]
pub(crate) struct LockedCycleAverage {
    pub(crate) torque_combustion_nm: f64,
    pub(crate) torque_friction_nm: f64,
    pub(crate) torque_pumping_nm: f64,
    pub(crate) torque_load_nm: f64,
    pub(crate) torque_net_nm: f64,
    pub(crate) air_consumption_gps: f64,
    pub(crate) trapped_air_mg: f64,
    pub(crate) exhaust_temp_k: f64,
}

#[derive(Debug, Clone, Copy)]
pub(crate) struct BenchSample {
    pub(crate) target_rpm: f64,
    pub(crate) rpm: f64,
    pub(crate) torque_brake_nm: f64,
    pub(crate) load_cmd: f64,
    pub(crate) load_torque_nm: f64,
    pub(crate) map_kpa: f64,
    pub(crate) eta_thermal_indicated_pv: f64,
    pub(crate) lambda_target: f64,
    pub(crate) intake_charge_temp_k: f64,
    pub(crate) mixture_mode: BenchMixtureMode,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum BenchPhase {
    Idle,
    Warmup,
    Sweep,
    Complete,
}

impl BenchPhase {
    pub(crate) fn label(self) -> &'static str {
        match self {
            Self::Idle => "Idle",
            Self::Warmup => "Warmup",
            Self::Sweep => "Sweep",
            Self::Complete => "Complete",
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub(crate) struct BenchStatus {
    pub(crate) mode: BenchMixtureMode,
    pub(crate) phase: BenchPhase,
    pub(crate) target_rpm: f64,
    pub(crate) live_rpm: f64,
    pub(crate) completed_points: usize,
    pub(crate) total_points: usize,
    pub(crate) phase_elapsed_s: f64,
    pub(crate) phase_total_s: f64,
    pub(crate) live_torque_brake_nm: f64,
    pub(crate) live_load_cmd: f64,
    pub(crate) live_load_torque_nm: f64,
    pub(crate) live_eta_thermal_indicated_pv: f64,
    pub(crate) live_map_kpa: f64,
    pub(crate) live_lambda_target: f64,
    pub(crate) live_intake_charge_temp_k: f64,
}

#[derive(Debug, Clone, Copy, Default)]
struct BenchAccumulator {
    rpm_sum: f64,
    torque_brake_sum_nm: f64,
    load_cmd_sum: f64,
    load_torque_sum_nm: f64,
    map_sum_kpa: f64,
    eta_sum: f64,
    lambda_sum: f64,
    intake_charge_temp_sum_k: f64,
    p_intake_sum_pa: f64,
    p_intake_runner_sum_pa: f64,
    p_exhaust_sum_pa: f64,
    p_exhaust_runner_sum_pa: f64,
    m_dot_intake_runner_sum_kg_s: f64,
    m_dot_exhaust_runner_sum_kg_s: f64,
    samples: usize,
}

impl BenchAccumulator {
    fn clear(&mut self) {
        *self = Self::default();
    }

    fn accumulate(&mut self, obs: &Observation, state: EngineState) {
        self.rpm_sum += obs.rpm;
        self.torque_brake_sum_nm += obs.torque_net_nm + obs.torque_load_nm;
        self.load_cmd_sum += obs.load_cmd;
        self.load_torque_sum_nm += obs.torque_load_nm;
        self.map_sum_kpa += obs.map_kpa;
        self.eta_sum += obs.eta_thermal_indicated_pv.max(0.0);
        self.lambda_sum += obs.lambda_target;
        self.intake_charge_temp_sum_k += obs.intake_charge_temp_k;
        self.p_intake_sum_pa += state.p_intake_pa;
        self.p_intake_runner_sum_pa += state.p_intake_runner_pa;
        self.p_exhaust_sum_pa += state.p_exhaust_pa;
        self.p_exhaust_runner_sum_pa += state.p_exhaust_runner_pa;
        self.m_dot_intake_runner_sum_kg_s += state.m_dot_intake_runner_kg_s;
        self.m_dot_exhaust_runner_sum_kg_s += state.m_dot_exhaust_runner_kg_s;
        self.samples = self.samples.saturating_add(1);
    }

    fn count_f64(&self) -> f64 {
        self.samples.max(1) as f64
    }

    fn mean_rpm(&self) -> f64 {
        self.rpm_sum / self.count_f64()
    }

    fn mean_torque_brake_nm(&self) -> f64 {
        self.torque_brake_sum_nm / self.count_f64()
    }

    fn mean_load_cmd(&self) -> f64 {
        self.load_cmd_sum / self.count_f64()
    }

    fn mean_load_torque_nm(&self) -> f64 {
        self.load_torque_sum_nm / self.count_f64()
    }

    fn mean_map_kpa(&self) -> f64 {
        self.map_sum_kpa / self.count_f64()
    }

    fn mean_eta(&self) -> f64 {
        self.eta_sum / self.count_f64()
    }

    fn mean_lambda_target(&self) -> f64 {
        self.lambda_sum / self.count_f64()
    }

    fn mean_intake_charge_temp_k(&self) -> f64 {
        self.intake_charge_temp_sum_k / self.count_f64()
    }

    #[cfg(test)]
    fn locked_state(&self, target_omega_rad_s: f64, throttle_eff: f64) -> EngineState {
        let count = self.count_f64();
        EngineState {
            omega_rad_s: target_omega_rad_s,
            theta_rad: 0.0,
            p_intake_pa: self.p_intake_sum_pa / count,
            p_intake_runner_pa: self.p_intake_runner_sum_pa / count,
            p_exhaust_pa: self.p_exhaust_sum_pa / count,
            p_exhaust_runner_pa: self.p_exhaust_runner_sum_pa / count,
            m_dot_intake_runner_kg_s: self.m_dot_intake_runner_sum_kg_s / count,
            m_dot_exhaust_runner_kg_s: self.m_dot_exhaust_runner_sum_kg_s / count,
            running: true,
            throttle_eff,
        }
    }
}

#[derive(Debug, Clone, Copy)]
struct BenchDynoController {
    integral: f64,
    load_cmd: f64,
}

impl BenchDynoController {
    fn new(initial_load_cmd: f64) -> Self {
        Self {
            integral: 0.0,
            load_cmd: initial_load_cmd.clamp(0.0, 1.0),
        }
    }

    fn update(&mut self, target_rpm: f64, actual_rpm: f64, dt: f64, bench: &BenchConfig) -> f64 {
        let dyno = &bench.dyno;
        let error_rpm = actual_rpm - target_rpm;
        let candidate_integral =
            (self.integral + error_rpm * dt).clamp(dyno.integral_min, dyno.integral_max);
        let load_cmd = (dyno.initial_load_cmd
            + dyno.speed_hold_kp * error_rpm
            + dyno.speed_hold_ki * candidate_integral)
            .clamp(0.0, 1.0);
        let saturated_high = load_cmd >= 1.0 - 1.0e-9 && error_rpm > 0.0;
        let saturated_low = load_cmd <= 1.0e-9 && error_rpm < 0.0;
        if !(saturated_high || saturated_low) {
            self.integral = candidate_integral;
        }
        self.load_cmd = load_cmd;
        load_cmd
    }
}

pub(crate) struct BenchSession {
    config: AppConfig,
    mode: BenchMixtureMode,
    control_seed: ControlInput,
    targets_rpm: Vec<f64>,
    current_idx: usize,
    phase: BenchPhase,
    phase_elapsed_s: f64,
    sim: Option<Simulator>,
    dyno: BenchDynoController,
    bins: Vec<BenchAccumulator>,
    results: Vec<BenchSample>,
    live_rpm: f64,
    live_torque_brake_nm: f64,
    live_load_cmd: f64,
    live_load_torque_nm: f64,
    live_eta_thermal_indicated_pv: f64,
    live_map_kpa: f64,
    live_lambda_target: f64,
    live_intake_charge_temp_k: f64,
    dt_next_s: f64,
}

#[derive(Debug, Clone)]
pub(crate) struct Observation {
    pub(crate) rpm: f64,
    pub(crate) load_cmd: f64,
    pub(crate) map_kpa: f64,
    pub(crate) intake_runner_kpa: f64,
    pub(crate) exhaust_kpa: f64,
    pub(crate) exhaust_runner_kpa: f64,
    pub(crate) intake_wave_kpa: f64,
    pub(crate) exhaust_wave_kpa: f64,
    pub(crate) exhaust_runner_flow_gps: f64,
    pub(crate) exhaust_temp_k: f64,
    pub(crate) intake_charge_temp_k: f64,
    pub(crate) torque_combustion_nm: f64,
    pub(crate) torque_combustion_cycle_nm: f64,
    pub(crate) torque_friction_nm: f64,
    pub(crate) torque_pumping_nm: f64,
    pub(crate) torque_starter_nm: f64,
    pub(crate) torque_load_nm: f64,
    pub(crate) torque_net_inst_nm: f64,
    pub(crate) torque_net_nm: f64,
    pub(crate) brake_bmep_bar: f64,
    pub(crate) air_flow_gps: f64,
    pub(crate) trapped_air_mg: f64,
    pub(crate) volumetric_efficiency: f64,
    pub(crate) intake_ram_multiplier: f64,
    pub(crate) exhaust_scavenge_multiplier: f64,
    pub(crate) ve_pulse_multiplier: f64,
    pub(crate) lambda_target: f64,
    pub(crate) heat_loss_cycle_j: f64,
    pub(crate) imep_bar: f64,
    pub(crate) eta_thermal_theoretical: f64,
    pub(crate) eta_thermal_indicated_pv: f64,
    pub(crate) indicated_work_cycle_j: f64,
    pub(crate) combustion_rate_norm: f64,
    pub(crate) ignition_timing_deg: f64,
    pub(crate) cycle_deg: f64,
    pub(crate) stable_idle: bool,
    pub(crate) pv_points: Vec<(f64, f64)>,
}

#[derive(Debug, Clone, Copy)]
pub(crate) struct PvSample {
    pub(crate) cycle: u64,
    pub(crate) cycle_deg: f64,
    pub(crate) volume: f64,
    pub(crate) pressure_pa: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct CycleHistorySample {
    pub(crate) cycle: u64,
    pub(crate) cycle_deg: f64,
    pub(crate) value: f64,
}

// Owns the continuous engine state plus the cycle-averaged quantities shown in the GUI.
pub(crate) struct Simulator {
    pub(crate) env: EnvironmentConfig,
    pub(crate) cam: CamConfig,
    pub(crate) plot: PlotConfig,
    pub(crate) auto_control: AutoControlConfig,
    pub(crate) model: ModelConfig,
    pub(crate) numerics: NumericsConfig,
    pub(crate) params: EngineParams,
    pub(crate) control: ControlInput,
    pub(crate) auto: AutoController,
    pub(crate) state: EngineState,
    pub(crate) history_rpm: VecDeque<CycleHistorySample>,
    pub(crate) history_torque_net_nm: VecDeque<CycleHistorySample>,
    pub(crate) history_torque_load_nm: VecDeque<CycleHistorySample>,
    pub(crate) history_trapped_air_mg: VecDeque<CycleHistorySample>,
    pub(crate) pv_history: VecDeque<PvSample>,
    intake_pulse_mean_single: f64,
    exhaust_pulse_mean_single: f64,
    cycle_index: u64,
    prev_theta_rad: f64,
    combustion_cycle_accum_nm: f64,
    combustion_cycle_samples: u32,
    combustion_cycle_mean_nm: f64,
    fuel_cycle_accum_kg: f64,
    fuel_cycle_samples: u32,
    fuel_cycle_recent_kg: VecDeque<f64>,
    indicated_work_recent_j: VecDeque<f64>,
    net_torque_cycle_accum_nm: f64,
    net_torque_cycle_samples: u32,
    net_torque_cycle_mean_nm: f64,
    net_torque_cycle_recent_nm: VecDeque<f64>,
    torque_net_filtered_nm: f64,
    step_count: u64,
    lambda_override: Option<f64>,
    charge_cooling_enabled: bool,
}

impl Simulator {
    pub(crate) fn new(cfg: &AppConfig) -> Self {
        let params = EngineParams {
            displacement_m3: cfg.engine.derived_displacement_m3(),
            compression_ratio: cfg.engine.compression_ratio,
            bore_m: cfg.engine.bore_m,
            stroke_m: cfg.engine.stroke_m,
            inertia_kgm2: cfg.engine.inertia_kgm2,
            friction_c0_nm: cfg.engine.friction_c0_nm,
            friction_c1_nms: cfg.engine.friction_c1_nms,
            friction_c2_nms2: cfg.engine.friction_c2_nms2,
            intake_volume_m3: cfg.engine.intake_volume_m3,
            intake_runner_volume_m3: cfg.engine.intake_runner_volume_m3,
            exhaust_volume_m3: cfg.engine.exhaust_volume_m3,
            exhaust_runner_volume_m3: cfg.engine.exhaust_runner_volume_m3,
            throttle_area_max_m2: cfg.engine.throttle_area_max_m2,
            tailpipe_area_m2: cfg.engine.tailpipe_area_m2,
            idle_target_rpm: cfg.engine.idle_target_rpm,
            max_rpm: cfg.engine.max_rpm.max(cfg.model.max_rpm_floor),
        };
        let mut state = EngineState::default();
        state.p_intake_pa = cfg.model.initial_intake_pressure_pa;
        state.p_intake_runner_pa = cfg.model.initial_intake_pressure_pa;
        state.throttle_eff = cfg.model.initial_throttle_eff;
        state.p_exhaust_pa = cfg.environment.ambient_pressure_pa;
        state.p_exhaust_runner_pa = cfg.environment.ambient_pressure_pa;
        let pv_cap = (cfg.plot.pv_recent_cycles.max(1)
            * cfg.plot.pv_subsamples_per_step.max(1)
            * cfg.model.pv_capacity_scale)
            .max(cfg.model.pv_capacity_min);
        let history_capacity = cfg
            .plot
            .rpm_history_capacity
            .max(cfg.model.history_capacity_floor);
        let intake_pulse_mean_single =
            valve_window_mean_single(cfg.cam.intake_duration_deg, &cfg.model);
        let exhaust_pulse_mean_single =
            valve_window_mean_single(cfg.cam.exhaust_duration_deg, &cfg.model);
        Self {
            env: cfg.environment.clone(),
            cam: cfg.cam.clone(),
            plot: cfg.plot.clone(),
            auto_control: cfg.auto_control.clone(),
            model: cfg.model.clone(),
            numerics: cfg.numerics.clone(),
            params,
            control: ControlInput::from_defaults(&cfg.control_defaults),
            auto: AutoController::default(),
            state,
            history_rpm: VecDeque::with_capacity(history_capacity),
            history_torque_net_nm: VecDeque::with_capacity(history_capacity),
            history_torque_load_nm: VecDeque::with_capacity(history_capacity),
            history_trapped_air_mg: VecDeque::with_capacity(history_capacity),
            pv_history: VecDeque::with_capacity(pv_cap),
            intake_pulse_mean_single,
            exhaust_pulse_mean_single,
            cycle_index: 0,
            prev_theta_rad: 0.0,
            combustion_cycle_accum_nm: 0.0,
            combustion_cycle_samples: 0,
            combustion_cycle_mean_nm: 0.0,
            fuel_cycle_accum_kg: 0.0,
            fuel_cycle_samples: 0,
            fuel_cycle_recent_kg: VecDeque::with_capacity(cfg.model.cycle_metric_history_capacity),
            indicated_work_recent_j: VecDeque::with_capacity(
                cfg.model.cycle_metric_history_capacity,
            ),
            net_torque_cycle_accum_nm: 0.0,
            net_torque_cycle_samples: 0,
            net_torque_cycle_mean_nm: 0.0,
            net_torque_cycle_recent_nm: VecDeque::with_capacity(
                cfg.model.cycle_metric_history_capacity,
            ),
            torque_net_filtered_nm: 0.0,
            step_count: 0,
            lambda_override: None,
            charge_cooling_enabled: false,
        }
    }

    pub(crate) fn set_idle_auto_enabled(&mut self, enabled: bool) {
        self.auto.enabled = enabled;
        self.auto.integral = 0.0;
        if enabled {
            self.auto.wot_efficiency_enabled = false;
        }
        self.auto.reset_wot_search(&self.control);
    }

    pub(crate) fn set_wot_efficiency_auto_enabled(&mut self, enabled: bool) {
        self.auto.wot_efficiency_enabled = enabled;
        self.auto.integral = 0.0;
        if enabled {
            self.auto.enabled = false;
        }
        self.auto.reset_wot_search(&self.control);
    }

    pub(crate) fn disable_auto_modes(&mut self) {
        self.auto.enabled = false;
        self.auto.wot_efficiency_enabled = false;
        self.auto.integral = 0.0;
        self.auto.reset_wot_search(&self.control);
    }

    fn theoretical_otto_efficiency(&self) -> f64 {
        let r = self
            .params
            .compression_ratio
            .max(self.model.theoretical_efficiency_compression_floor);
        (1.0 - 1.0 / r.powf(GAMMA_AIR - 1.0)).clamp(0.0, 1.0)
    }

    fn swept_volume_per_cylinder_m3(&self) -> f64 {
        self.params.displacement_m3 / FIXED_CYLINDER_COUNT as f64
    }

    fn clearance_volume_per_cylinder_m3(&self) -> f64 {
        self.swept_volume_per_cylinder_m3()
            / (self.params.compression_ratio - 1.0).max(self.model.compression_ratio_guard)
    }

    fn cylinder_surface_area_m2(&self) -> f64 {
        let bore_area = 0.25 * PI * self.params.bore_m.powi(2);
        let wall_area = PI * self.params.bore_m * self.params.stroke_m;
        self.model.heat_transfer.wall_area_scale * (2.0 * bore_area + wall_area)
    }

    fn mean_piston_speed_mps(&self, rpm: f64) -> f64 {
        2.0 * self.params.stroke_m * rpm.max(0.0) / 60.0
    }

    fn runner_area_m2(diameter_m: f64) -> f64 {
        0.25 * PI * diameter_m.max(1.0e-4).powi(2)
    }

    fn geometry_scaled_inertance(
        base_inertance_pa_s2_per_kg: f64,
        length_m: f64,
        diameter_m: f64,
        reference_length_m: f64,
        reference_diameter_m: f64,
    ) -> f64 {
        let length_scale = length_m.max(1.0e-4) / reference_length_m.max(1.0e-4);
        let area_scale = (reference_diameter_m.max(1.0e-4) / diameter_m.max(1.0e-4)).powi(2);
        (base_inertance_pa_s2_per_kg * length_scale * area_scale).max(1.0)
    }

    fn intake_runner_inertance_pa_s2_per_kg(&self) -> f64 {
        Self::geometry_scaled_inertance(
            self.model.gas_path.intake_runner_inertance_pa_s2_per_kg,
            self.model.gas_path.intake_runner_length_m,
            self.model.gas_path.intake_runner_diameter_m,
            DEFAULT_INTAKE_RUNNER_LENGTH_M,
            DEFAULT_INTAKE_RUNNER_DIAMETER_M,
        )
    }

    fn exhaust_runner_inertance_pa_s2_per_kg(&self) -> f64 {
        Self::geometry_scaled_inertance(
            self.model.gas_path.exhaust_runner_inertance_pa_s2_per_kg,
            self.model.gas_path.exhaust_runner_length_m,
            self.model.gas_path.exhaust_runner_diameter_m,
            DEFAULT_EXHAUST_RUNNER_LENGTH_M,
            DEFAULT_EXHAUST_RUNNER_DIAMETER_M,
        )
    }

    fn runner_pressure_loss_pa(
        m_dot_kg_s: f64,
        upstream_pa: f64,
        downstream_pa: f64,
        gas_temp_k: f64,
        length_m: f64,
        diameter_m: f64,
        parallel_paths: usize,
        friction_factor: f64,
        local_loss_coeff: f64,
    ) -> f64 {
        if m_dot_kg_s.abs() <= f64::EPSILON || diameter_m <= 0.0 || gas_temp_k <= 0.0 {
            return 0.0;
        }

        let area_m2 = Self::runner_area_m2(diameter_m) * parallel_paths.max(1) as f64;
        let density_kg_m3 =
            (0.5 * (upstream_pa + downstream_pa)).max(1.0) / (R_AIR * gas_temp_k.max(1.0));
        let k_total = friction_factor.max(0.0) * length_m.max(0.0) / diameter_m.max(1.0e-4)
            + local_loss_coeff.max(0.0);
        let dynamic_pressure_pa =
            m_dot_kg_s.powi(2) / (2.0 * density_kg_m3.max(1.0e-6) * area_m2.max(1.0e-8).powi(2));

        m_dot_kg_s.signum() * k_total * dynamic_pressure_pa
    }

    fn intake_runner_pressure_loss_pa(&self, state: EngineState) -> f64 {
        Self::runner_pressure_loss_pa(
            state.m_dot_intake_runner_kg_s,
            state.p_intake_pa,
            state.p_intake_runner_pa,
            self.env.intake_temp_k,
            self.model.gas_path.intake_runner_length_m,
            self.model.gas_path.intake_runner_diameter_m,
            FIXED_CYLINDER_COUNT,
            self.model.gas_path.intake_runner_friction_factor,
            self.model.gas_path.intake_runner_local_loss_coeff,
        )
    }

    fn exhaust_runner_pressure_loss_pa(&self, state: EngineState, exhaust_temp_k: f64) -> f64 {
        Self::runner_pressure_loss_pa(
            state.m_dot_exhaust_runner_kg_s,
            state.p_exhaust_runner_pa,
            state.p_exhaust_pa,
            exhaust_temp_k,
            self.model.gas_path.exhaust_runner_length_m,
            self.model.gas_path.exhaust_runner_diameter_m,
            FIXED_CYLINDER_COUNT,
            self.model.gas_path.exhaust_runner_friction_factor,
            self.model.gas_path.exhaust_runner_local_loss_coeff,
        )
    }

    fn helmholtz_frequency_hz(
        volume_m3: f64,
        length_m: f64,
        diameter_m: f64,
        gas_temp_k: f64,
        branch_count: usize,
    ) -> f64 {
        let area_total_m2 = Self::runner_area_m2(diameter_m) * branch_count.max(1) as f64;
        let effective_length_m = length_m.max(1.0e-4) + 0.6 * diameter_m.max(1.0e-4);
        let sound_speed = (GAMMA_AIR * R_AIR * gas_temp_k.max(1.0)).sqrt();
        sound_speed / (2.0 * PI)
            * (area_total_m2 / (volume_m3.max(1.0e-6) * effective_length_m)).sqrt()
    }

    fn resonance_gain(
        excitation_hz: f64,
        natural_hz: f64,
        damping_ratio: f64,
        blend: f64,
        gain_min: f64,
        gain_max: f64,
    ) -> f64 {
        if excitation_hz <= 0.0 || natural_hz <= 0.0 {
            return 1.0;
        }
        let zeta = damping_ratio.max(1.0e-3);
        let ratio = excitation_hz / natural_hz.max(f64::EPSILON);
        let response = 1.0 / ((1.0 - ratio.powi(2)).powi(2) + (2.0 * zeta * ratio).powi(2)).sqrt();
        (1.0 + blend.max(0.0) * (response - 1.0)).clamp(gain_min, gain_max)
    }

    fn intake_centerline_deg(&self) -> f64 {
        self.cam.intake_centerline_deg - self.control.vvt_intake_deg
    }

    fn exhaust_centerline_deg(&self) -> f64 {
        self.cam.exhaust_centerline_deg + self.control.vvt_exhaust_deg
    }

    fn intake_open_deg(&self) -> f64 {
        self.intake_centerline_deg() - 0.5 * self.cam.intake_duration_deg
    }

    fn intake_close_deg(&self) -> f64 {
        self.intake_centerline_deg() + 0.5 * self.cam.intake_duration_deg
    }

    fn exhaust_open_deg(&self) -> f64 {
        self.exhaust_centerline_deg() - 0.5 * self.cam.exhaust_duration_deg
    }

    fn firing_interval_deg(&self) -> f64 {
        720.0 / FIXED_CYLINDER_COUNT as f64
    }

    fn effective_group_count(&self, requested: usize) -> usize {
        let cylinders = FIXED_CYLINDER_COUNT;
        let requested = requested.max(1).min(cylinders);
        for group_count in (1..=requested).rev() {
            if cylinders % group_count == 0 {
                return group_count;
            }
        }
        1
    }

    fn valve_group_weights(
        &self,
        cycle_deg: f64,
        center_deg: f64,
        duration_deg: f64,
        group_count: usize,
    ) -> Vec<f64> {
        let group_count = group_count.max(1);
        let cylinders = FIXED_CYLINDER_COUNT;
        let firing_interval = self.firing_interval_deg();
        let mut weights = vec![0.0; group_count];
        for cylinder in 0..cylinders {
            let offset = cylinder as f64 * firing_interval;
            let theta = (cycle_deg - offset).rem_euclid(720.0);
            let lift = cam_lift_mm(theta, center_deg, duration_deg, 1.0, &self.model);
            weights[cylinder % group_count] += lift;
        }
        weights
    }

    fn weighted_group_value(values: &[f64], weights: &[f64]) -> f64 {
        if values.is_empty() {
            return 0.0;
        }
        let weight_sum: f64 = weights.iter().copied().sum();
        if weight_sum <= f64::EPSILON {
            return values.iter().copied().sum::<f64>() / values.len() as f64;
        }
        values
            .iter()
            .zip(weights.iter())
            .map(|(value, weight)| value * weight)
            .sum::<f64>()
            / weight_sum
    }

    fn group_wave_pressure_at(
        &self,
        current_cycle_deg: f64,
        rpm: f64,
        target_group: usize,
        group_count: usize,
        source_event_base_deg: f64,
        runner_length_m: f64,
        delay_scale: f64,
        decay_time_s: f64,
        gas_temp_k: f64,
        source_flow_abs_kg_s: f64,
        flow_reference_kg_s: f64,
        pressure_gain: f64,
        pressure_limit_pa: f64,
        reflection_sign: f64,
    ) -> f64 {
        if rpm <= 0.0
            || runner_length_m <= 0.0
            || decay_time_s <= 0.0
            || source_flow_abs_kg_s <= 0.0
            || pressure_gain.abs() <= f64::EPSILON
        {
            return 0.0;
        }

        let sound_speed = (GAMMA_AIR * R_AIR * gas_temp_k.max(f64::EPSILON)).sqrt();
        let quarter_wave_hz = sound_speed / (4.0 * runner_length_m);
        let delay_s = delay_scale * runner_length_m / sound_speed.max(f64::EPSILON);
        let deg_per_second = rpm * 6.0;
        let firing_interval = self.firing_interval_deg();
        let memory = self.model.wave_action.event_memory.max(1);
        let group_size = (FIXED_CYLINDER_COUNT / group_count.max(1)).max(1) as f64;
        let flow_scale = source_flow_abs_kg_s / flow_reference_kg_s.max(f64::EPSILON);
        let mut response = 0.0;

        for cylinder in 0..FIXED_CYLINDER_COUNT {
            if cylinder % group_count.max(1) != target_group {
                continue;
            }
            let event_deg =
                (source_event_base_deg + cylinder as f64 * firing_interval).rem_euclid(720.0);
            let base_elapsed_deg = (current_cycle_deg - event_deg).rem_euclid(720.0);
            for cycle_back in 0..memory {
                let elapsed_deg = base_elapsed_deg + 720.0 * cycle_back as f64;
                let elapsed_s = elapsed_deg / deg_per_second.max(f64::EPSILON);
                response += wave_kernel(elapsed_s, delay_s, decay_time_s, quarter_wave_hz);
            }
        }

        let pressure_pa =
            reflection_sign * pressure_gain * self.env.ambient_pressure_pa * flow_scale * response
                / group_size;
        pressure_pa.clamp(-pressure_limit_pa, pressure_limit_pa)
    }

    fn group_wave_pressures(
        &self,
        current_cycle_deg: f64,
        rpm: f64,
        requested_group_count: usize,
        source_event_base_deg: f64,
        runner_length_m: f64,
        delay_scale: f64,
        decay_time_s: f64,
        gas_temp_k: f64,
        source_flow_abs_kg_s: f64,
        flow_reference_kg_s: f64,
        pressure_gain: f64,
        pressure_limit_pa: f64,
        reflection_sign: f64,
    ) -> Vec<f64> {
        let group_count = self.effective_group_count(requested_group_count);
        (0..group_count)
            .map(|group| {
                self.group_wave_pressure_at(
                    current_cycle_deg,
                    rpm,
                    group,
                    group_count,
                    source_event_base_deg,
                    runner_length_m,
                    delay_scale,
                    decay_time_s,
                    gas_temp_k,
                    source_flow_abs_kg_s,
                    flow_reference_kg_s,
                    pressure_gain,
                    pressure_limit_pa,
                    reflection_sign,
                )
            })
            .collect()
    }

    fn mean_group_wave_at_event(
        &self,
        rpm: f64,
        requested_group_count: usize,
        source_event_base_deg: f64,
        target_event_base_deg: f64,
        runner_length_m: f64,
        delay_scale: f64,
        decay_time_s: f64,
        gas_temp_k: f64,
        source_flow_abs_kg_s: f64,
        flow_reference_kg_s: f64,
        pressure_gain: f64,
        pressure_limit_pa: f64,
        reflection_sign: f64,
    ) -> f64 {
        let group_count = self.effective_group_count(requested_group_count);
        let firing_interval = self.firing_interval_deg();
        let cylinders = FIXED_CYLINDER_COUNT;
        let mut sum = 0.0;
        for cylinder in 0..cylinders {
            let target_event_deg =
                (target_event_base_deg + cylinder as f64 * firing_interval).rem_euclid(720.0);
            sum += self.group_wave_pressure_at(
                target_event_deg,
                rpm,
                cylinder % group_count,
                group_count,
                source_event_base_deg,
                runner_length_m,
                delay_scale,
                decay_time_s,
                gas_temp_k,
                source_flow_abs_kg_s,
                flow_reference_kg_s,
                pressure_gain,
                pressure_limit_pa,
                reflection_sign,
            );
        }
        sum / cylinders as f64
    }

    fn wave_action_state(
        &self,
        state: EngineState,
        cycle_deg: f64,
        rpm: f64,
        ve_base: f64,
        lambda_target: f64,
        base_intake_boundary_pa: f64,
        exhaust_temp_k: f64,
    ) -> WaveActionState {
        if rpm <= 0.0 {
            return WaveActionState::default();
        }

        let wave = &self.model.wave_action;
        let intake_group_count = self.effective_group_count(wave.intake_group_count);
        let exhaust_group_count = self.effective_group_count(wave.exhaust_group_count);
        let intake_branch_count = (FIXED_CYLINDER_COUNT / intake_group_count.max(1)).max(1);
        let exhaust_branch_count = (FIXED_CYLINDER_COUNT / exhaust_group_count.max(1)).max(1);
        let afr = self.model.stoich_afr * lambda_target;
        let m_dot_cyl_mean_est = engine_air_consumption(
            self.params.displacement_m3,
            rpm,
            ve_base,
            base_intake_boundary_pa,
            self.env.intake_temp_k,
        );
        let m_dot_fuel_est = if self.control.fuel_cmd {
            m_dot_cyl_mean_est / afr.max(f64::EPSILON)
        } else {
            0.0
        };
        let intake_source_flow_abs_kg_s =
            state.m_dot_intake_runner_kg_s.abs().max(m_dot_cyl_mean_est);
        let exhaust_source_flow_abs_kg_s = state
            .m_dot_exhaust_runner_kg_s
            .abs()
            .max(m_dot_cyl_mean_est + m_dot_fuel_est);
        let intake_resonance_gain = Self::resonance_gain(
            rpm / 120.0 * intake_branch_count as f64,
            Self::helmholtz_frequency_hz(
                self.params.intake_volume_m3,
                self.model.gas_path.intake_runner_length_m,
                self.model.gas_path.intake_runner_diameter_m,
                self.env.intake_temp_k,
                intake_branch_count,
            ),
            wave.intake_resonance_damping_ratio,
            wave.intake_resonance_gain_blend,
            wave.resonance_gain_min,
            wave.resonance_gain_max,
        );
        let exhaust_resonance_gain = Self::resonance_gain(
            rpm / 120.0 * exhaust_branch_count as f64,
            Self::helmholtz_frequency_hz(
                self.params.exhaust_volume_m3 / exhaust_group_count.max(1) as f64,
                self.model.gas_path.exhaust_runner_length_m,
                self.model.gas_path.exhaust_runner_diameter_m,
                exhaust_temp_k,
                exhaust_branch_count,
            ),
            wave.exhaust_resonance_damping_ratio,
            wave.exhaust_resonance_gain_blend,
            wave.resonance_gain_min,
            wave.resonance_gain_max,
        );
        let intake_pressure_gain = wave.intake_pressure_gain * intake_resonance_gain;
        let exhaust_pressure_gain = wave.exhaust_pressure_gain * exhaust_resonance_gain;

        let intake_current_group_pressures = self.group_wave_pressures(
            cycle_deg,
            rpm,
            intake_group_count,
            self.intake_open_deg(),
            wave.intake_runner_length_m,
            wave.intake_delay_scale,
            wave.intake_decay_time_s,
            self.env.intake_temp_k,
            intake_source_flow_abs_kg_s,
            wave.intake_flow_reference_kg_s,
            intake_pressure_gain,
            wave.intake_pressure_limit_pa,
            1.0,
        );
        let exhaust_current_group_pressures = self.group_wave_pressures(
            cycle_deg,
            rpm,
            exhaust_group_count,
            self.exhaust_open_deg(),
            wave.exhaust_primary_length_m,
            wave.exhaust_delay_scale,
            wave.exhaust_decay_time_s,
            exhaust_temp_k,
            exhaust_source_flow_abs_kg_s,
            wave.exhaust_flow_reference_kg_s,
            exhaust_pressure_gain,
            wave.exhaust_pressure_limit_pa,
            -1.0,
        );
        let intake_current_wave_pa = Self::weighted_group_value(
            &intake_current_group_pressures,
            &self.valve_group_weights(
                cycle_deg,
                self.intake_centerline_deg(),
                self.cam.intake_duration_deg,
                intake_group_count,
            ),
        );
        let exhaust_current_wave_pa = Self::weighted_group_value(
            &exhaust_current_group_pressures,
            &self.valve_group_weights(
                cycle_deg,
                self.exhaust_centerline_deg(),
                self.cam.exhaust_duration_deg,
                exhaust_group_count,
            ),
        );
        let intake_boundary_wave_pa = self.mean_group_wave_at_event(
            rpm,
            intake_group_count,
            self.intake_open_deg(),
            self.intake_close_deg(),
            wave.intake_runner_length_m,
            wave.intake_delay_scale,
            wave.intake_decay_time_s,
            self.env.intake_temp_k,
            intake_source_flow_abs_kg_s,
            wave.intake_flow_reference_kg_s,
            intake_pressure_gain,
            wave.intake_pressure_limit_pa,
            1.0,
        );
        let intake_overlap_wave_pa = self.mean_group_wave_at_event(
            rpm,
            intake_group_count,
            self.intake_open_deg(),
            360.0,
            wave.intake_runner_length_m,
            wave.intake_delay_scale,
            wave.intake_decay_time_s,
            self.env.intake_temp_k,
            intake_source_flow_abs_kg_s,
            wave.intake_flow_reference_kg_s,
            intake_pressure_gain,
            wave.intake_pressure_limit_pa,
            1.0,
        );
        let exhaust_boundary_wave_pa = self.mean_group_wave_at_event(
            rpm,
            exhaust_group_count,
            self.exhaust_open_deg(),
            360.0,
            wave.exhaust_primary_length_m,
            wave.exhaust_delay_scale,
            wave.exhaust_decay_time_s,
            exhaust_temp_k,
            exhaust_source_flow_abs_kg_s,
            wave.exhaust_flow_reference_kg_s,
            exhaust_pressure_gain,
            wave.exhaust_pressure_limit_pa,
            -1.0,
        );
        let intake_ram_multiplier =
            1.0 + wave.intake_ram_gain * intake_boundary_wave_pa / self.env.ambient_pressure_pa;
        let scavenging_head =
            (intake_overlap_wave_pa - exhaust_boundary_wave_pa) / self.env.ambient_pressure_pa;
        let exhaust_scavenge_multiplier = 1.0 + wave.exhaust_scavenge_gain * scavenging_head;
        let ve_pulse_multiplier = (intake_ram_multiplier * exhaust_scavenge_multiplier)
            .clamp(wave.ve_pulse_min, wave.ve_pulse_max);
        let intake_flow_multiplier = (1.0
            + wave.intake_flow_wave_gain * intake_current_wave_pa / self.env.ambient_pressure_pa)
            .max(0.0);
        let exhaust_flow_multiplier = (1.0
            - wave.exhaust_flow_wave_gain * exhaust_current_wave_pa / self.env.ambient_pressure_pa)
            .max(0.0);

        WaveActionState {
            intake_wave_current_pa: intake_current_wave_pa,
            exhaust_wave_current_pa: exhaust_current_wave_pa,
            intake_boundary_wave_pa,
            exhaust_boundary_wave_pa,
            intake_flow_multiplier,
            exhaust_flow_multiplier,
            intake_ram_multiplier,
            exhaust_scavenge_multiplier,
            ve_pulse_multiplier,
            scavenging_head_norm: scavenging_head,
        }
    }

    fn valve_pulse_factors(&self, cycle_deg: f64) -> (f64, f64) {
        let cylinders = FIXED_CYLINDER_COUNT;
        let intake_lift_sum = aggregate_valve_lift_norm(
            cycle_deg,
            cylinders,
            self.intake_centerline_deg(),
            self.cam.intake_duration_deg,
            &self.model,
        );
        let exhaust_lift_sum = aggregate_valve_lift_norm(
            cycle_deg,
            cylinders,
            self.exhaust_centerline_deg(),
            self.cam.exhaust_duration_deg,
            &self.model,
        );
        let intake_mean = (self.intake_pulse_mean_single * cylinders as f64).max(f64::EPSILON);
        let exhaust_mean = (self.exhaust_pulse_mean_single * cylinders as f64).max(f64::EPSILON);
        let intake_raw = intake_lift_sum / intake_mean;
        let exhaust_raw = exhaust_lift_sum / exhaust_mean;
        (
            1.0 + self.model.gas_path.intake_pulse_blend * (intake_raw - 1.0),
            1.0 + self.model.gas_path.exhaust_pulse_blend * (exhaust_raw - 1.0),
        )
    }

    fn overlap_lift_fraction(&self) -> f64 {
        let intake_lift = cam_lift_mm(
            360.0,
            self.intake_centerline_deg(),
            self.cam.intake_duration_deg,
            1.0,
            &self.model,
        )
        .clamp(0.0, 1.0);
        let exhaust_lift = cam_lift_mm(
            360.0,
            self.exhaust_centerline_deg(),
            self.cam.exhaust_duration_deg,
            1.0,
            &self.model,
        )
        .clamp(0.0, 1.0);
        intake_lift.min(exhaust_lift)
    }

    fn overlap_ve_multiplier(&self, state: EngineState) -> f64 {
        let overlap = self.overlap_lift_fraction();
        let pressure_term =
            (state.p_intake_runner_pa - state.p_exhaust_runner_pa) / self.env.ambient_pressure_pa;
        let flow_term = state.m_dot_exhaust_runner_kg_s
            / self
                .model
                .gas_path
                .overlap_flow_reference_kg_s
                .max(f64::EPSILON);
        (1.0 + overlap
            * (self.model.gas_path.overlap_pressure_coeff * pressure_term
                + self.model.gas_path.overlap_flow_coeff * flow_term))
            .clamp(
                self.model.gas_path.overlap_effect_min,
                self.model.gas_path.overlap_effect_max,
            )
    }

    fn gas_gamma_from_cp_j_per_kgk(cp_j_per_kgk: f64) -> f64 {
        let cp = cp_j_per_kgk.max(R_AIR + 1.0);
        cp / (cp - R_AIR)
    }

    fn fresh_charge_cp_j_per_kgk(&self) -> f64 {
        let gas = &self.model.gas_thermo;
        gas.fresh_cp_j_per_kgk
            .clamp(gas.cp_min_j_per_kgk, gas.cp_max_j_per_kgk)
    }

    fn burned_gas_cp_j_per_kgk(&self, temp_k: f64, internal_egr_fraction: f64) -> f64 {
        let gas = &self.model.gas_thermo;
        let delta_t = temp_k - gas.burned_cp_reference_temp_k;
        (gas.burned_cp_ref_j_per_kgk
            + gas.burned_cp_temp_coeff_j_per_kgk2 * delta_t
            + gas.burned_cp_egr_coeff_j_per_kgk * internal_egr_fraction.max(0.0))
        .clamp(gas.cp_min_j_per_kgk, gas.cp_max_j_per_kgk)
    }

    fn base_intake_cylinder_boundary_pa(&self, state: EngineState) -> f64 {
        lerp(
            state.p_intake_pa,
            state.p_intake_runner_pa,
            self.model
                .gas_path
                .intake_boundary_runner_weight
                .clamp(0.0, 1.0),
        )
    }

    fn base_exhaust_cylinder_boundary_pa(&self, state: EngineState) -> f64 {
        lerp(
            state.p_exhaust_pa,
            state.p_exhaust_runner_pa,
            self.model
                .gas_path
                .exhaust_boundary_runner_weight
                .clamp(0.0, 1.0),
        )
    }

    fn intake_cylinder_boundary_pa(&self, state: EngineState, wave_boundary_pa: f64) -> f64 {
        (self.base_intake_cylinder_boundary_pa(state) + wave_boundary_pa).clamp(
            self.model.intake_pressure_min_pa,
            self.model.intake_pressure_max_pa,
        )
    }

    fn exhaust_cylinder_boundary_pa(&self, state: EngineState, wave_boundary_pa: f64) -> f64 {
        (self.base_exhaust_cylinder_boundary_pa(state) + wave_boundary_pa).clamp(
            self.env.ambient_pressure_pa * self.model.exhaust_pressure_min_ambient_ratio,
            self.env.ambient_pressure_pa + self.model.exhaust_pressure_max_over_ambient_pa,
        )
    }

    fn target_lambda(&self, throttle_eff: f64) -> f64 {
        self.lambda_override
            .unwrap_or_else(|| {
                (self.model.lambda_base - self.model.lambda_throttle_coeff * throttle_eff)
                    .clamp(self.model.lambda_min, self.model.lambda_max)
            })
            .clamp(self.model.lambda_min, self.model.lambda_max)
    }

    fn fresh_charge_temp_k(&self, trapped_air_guess_kg: f64, fuel_mass_guess_kg: f64) -> f64 {
        if !self.charge_cooling_enabled {
            return self.env.intake_temp_k;
        }
        let evap = &self.model.fuel_evaporation;
        let charge_mass_kg = (trapped_air_guess_kg + fuel_mass_guess_kg)
            .max(self.model.fuel_mass_presence_threshold_kg);
        let cp_air = self.fresh_charge_cp_j_per_kgk();
        let q_evap_j = fuel_mass_guess_kg.max(0.0)
            * evap.latent_heat_j_per_kg
            * evap.charge_cooling_effectiveness;
        let delta_t_k = q_evap_j / (charge_mass_kg * cp_air);
        (self.env.intake_temp_k - delta_t_k)
            .clamp(evap.intake_charge_temp_min_k, self.env.intake_temp_k)
    }

    fn internal_egr_fraction(&self, state: EngineState, wave_action: WaveActionState) -> f64 {
        let egr = &self.model.internal_egr;
        let overlap = self.overlap_lift_fraction();
        if overlap <= 0.0 {
            return 0.0;
        }

        let pressure_backflow = ((state.p_exhaust_runner_pa - state.p_intake_runner_pa)
            / self.env.ambient_pressure_pa)
            .max(0.0);
        let wave_backflow = (-wave_action.scavenging_head_norm).max(0.0);
        let reverse_flow = (-state.m_dot_exhaust_runner_kg_s
            / egr.reverse_flow_reference_kg_s.max(f64::EPSILON))
        .max(0.0);
        (overlap
            * (egr.overlap_base_fraction
                + egr.pressure_backflow_gain * pressure_backflow
                + egr.wave_backflow_gain * wave_backflow
                + egr.reverse_flow_gain * reverse_flow))
            .clamp(egr.fraction_min, egr.fraction_max)
    }

    fn charge_state(
        &self,
        trapped_air_kg: f64,
        fuel_mass_kg: f64,
        residual_temp_k: f64,
        internal_egr_fraction: f64,
    ) -> ChargeState {
        let fresh_charge_temp_k = self.fresh_charge_temp_k(trapped_air_kg, fuel_mass_kg);
        let fresh_cp = self.fresh_charge_cp_j_per_kgk();
        let gas_fraction = internal_egr_fraction.clamp(0.0, 0.95);
        let residual_mass_kg =
            trapped_air_kg.max(0.0) * gas_fraction / (1.0 - gas_fraction).max(1.0e-6);
        let burned_gas_cp_j_per_kgk =
            self.burned_gas_cp_j_per_kgk(residual_temp_k, internal_egr_fraction);
        let fresh_mass_kg = (trapped_air_kg + fuel_mass_kg.max(0.0))
            .max(self.model.fuel_mass_presence_threshold_kg);
        let total_charge_mass_kg = fresh_mass_kg + residual_mass_kg;
        let mixed_cp_numerator =
            fresh_mass_kg * fresh_cp + residual_mass_kg * burned_gas_cp_j_per_kgk;
        let mixture_cp_j_per_kgk = if total_charge_mass_kg > f64::EPSILON {
            mixed_cp_numerator / total_charge_mass_kg
        } else {
            fresh_cp
        };
        let energy_numerator = fresh_mass_kg * fresh_cp * fresh_charge_temp_k
            + residual_mass_kg * burned_gas_cp_j_per_kgk * residual_temp_k;
        let intake_charge_temp_k = if mixed_cp_numerator > f64::EPSILON {
            energy_numerator / mixed_cp_numerator
        } else {
            fresh_charge_temp_k
        }
        .clamp(
            fresh_charge_temp_k,
            residual_temp_k.max(fresh_charge_temp_k),
        );

        ChargeState {
            intake_charge_temp_k,
            total_charge_mass_kg,
            mixture_gamma: Self::gas_gamma_from_cp_j_per_kgk(mixture_cp_j_per_kgk),
            burned_gas_cp_j_per_kgk,
        }
    }

    fn configure_bench_mixture_mode(&mut self, mode: BenchMixtureMode, bench: &BenchConfig) {
        self.lambda_override = Some(match mode {
            BenchMixtureMode::RichChargeCooling => bench.rich_charge_cooling_lambda,
            BenchMixtureMode::LambdaOne => bench.lambda_one_target,
        });
        self.charge_cooling_enabled = true;
    }

    fn cylinder_heat_loss_cycle_j(
        &self,
        rpm: f64,
        cylinder_pressure_pa: f64,
        total_charge_mass_kg: f64,
        fuel_mass_cycle_cyl_kg: f64,
        intake_charge_temp_k: f64,
        mixture_gamma: f64,
        burned_gas_cp_j_per_kgk: f64,
        phase_eff: f64,
        burn_duration_deg: f64,
    ) -> f64 {
        if fuel_mass_cycle_cyl_kg <= self.model.fuel_mass_presence_threshold_kg || rpm <= 0.0 {
            return 0.0;
        }

        let ht = &self.model.heat_transfer;
        let charge_mass_kg = total_charge_mass_kg.max(self.model.fuel_mass_presence_threshold_kg);
        let gamma = mixture_gamma.max(1.01);
        let compression_temp_k =
            intake_charge_temp_k * self.params.compression_ratio.powf(gamma - 1.0);
        let delta_t_adiabatic =
            fuel_mass_cycle_cyl_kg * FUEL_LHV_J_PER_KG / (charge_mass_kg * burned_gas_cp_j_per_kgk);
        let gas_temp_k = (compression_temp_k
            + ht.gas_temp_rise_gain * phase_eff * delta_t_adiabatic)
            .clamp(self.env.intake_temp_k, ht.gas_temp_max_k);
        let pressure_ratio = (cylinder_pressure_pa.max(self.env.ambient_pressure_pa)
            / ht.reference_pressure_pa.max(f64::EPSILON))
        .max(f64::EPSILON);
        let temperature_ratio =
            (gas_temp_k / ht.reference_temp_k.max(f64::EPSILON)).max(f64::EPSILON);
        let piston_speed_ratio = (self.mean_piston_speed_mps(rpm)
            / ht.reference_piston_speed_mps.max(f64::EPSILON))
        .max(f64::EPSILON);
        let h_w_m2k = ht.base_h_w_m2k
            * pressure_ratio.powf(ht.pressure_exponent)
            * temperature_ratio.powf(ht.temperature_exponent)
            * piston_speed_ratio.powf(ht.piston_speed_exponent);
        let exposure_s = ht.duration_scale
            * (burn_duration_deg / 720.0)
            * (120.0 / rpm.max(self.model.combustion_enable_rpm_min));
        let q_loss = h_w_m2k
            * self.cylinder_surface_area_m2()
            * (gas_temp_k - ht.wall_temp_k).max(0.0)
            * exposure_s.max(0.0);
        let fuel_heat = fuel_mass_cycle_cyl_kg * FUEL_LHV_J_PER_KG;
        q_loss.clamp(0.0, fuel_heat * ht.heat_loss_fraction_max)
    }

    fn indicated_work_for_cycle_j(&self, cycle: u64) -> Option<f64> {
        let swept_volume = self.swept_volume_per_cylinder_m3();
        let mut first: Option<(f64, f64)> = None;
        let mut prev: Option<(f64, f64)> = None;
        let mut work_j = 0.0;
        let mut samples = 0usize;

        for sample in self.pv_history.iter().filter(|s| s.cycle == cycle) {
            let curr = (sample.volume * swept_volume, sample.pressure_pa.max(0.0));
            if let Some((v_prev, p_prev)) = prev {
                work_j += 0.5 * (p_prev + curr.1) * (curr.0 - v_prev);
            } else {
                first = Some(curr);
            }
            prev = Some(curr);
            samples = samples.saturating_add(1);
        }

        if samples < 4 {
            return None;
        }
        if let (Some((v_last, p_last)), Some((v_first, p_first))) = (prev, first) {
            work_j += 0.5 * (p_last + p_first) * (v_first - v_last);
        }
        Some(work_j)
    }

    fn mean_indicated_work_recent_cycles_j(&self, n_cycles: usize) -> Option<f64> {
        mean_recent(&self.indicated_work_recent_j, n_cycles)
    }

    fn mean_fuel_cycle_recent_kg(&self, n_cycles: usize) -> Option<f64> {
        mean_recent(&self.fuel_cycle_recent_kg, n_cycles)
    }

    fn mean_net_torque_cycle_recent_nm(&self, n_cycles: usize) -> Option<f64> {
        mean_recent(&self.net_torque_cycle_recent_nm, n_cycles)
    }

    fn build_pv_display_points(&self) -> Vec<(f64, f64)> {
        // Average recent p-V samples into crank-angle bins so the displayed loop stays smooth.
        let bins = self.model.pv_display_bins.max(1);
        let mut raw = Vec::with_capacity(self.pv_history.len());
        let mut sum_v = vec![0.0; bins];
        let mut sum_p = vec![0.0; bins];
        let mut count = vec![0u32; bins];
        for s in &self.pv_history {
            raw.push((s.volume, s.pressure_pa));
            let mut idx = ((s.cycle_deg / 720.0) * bins as f64) as usize;
            idx = idx.min(bins.saturating_sub(1));
            sum_v[idx] += s.volume;
            sum_p[idx] += s.pressure_pa;
            count[idx] = count[idx].saturating_add(1);
        }
        if raw.len() < self.model.pv_display_raw_min_points {
            return raw;
        }

        let mut averaged = Vec::with_capacity(bins + 1);
        for i in 0..bins {
            if count[i] == 0 {
                continue;
            }
            let c = count[i] as f64;
            averaged.push((sum_v[i] / c, sum_p[i] / c));
        }
        if averaged.len() < self.model.pv_display_min_populated_bins.min(bins) {
            return raw;
        }
        if let Some(first) = averaged.first().copied() {
            averaged.push(first);
        }
        averaged
    }

    fn displayed_pv_peak_pressure_pa(
        &self,
        p_intake_pa: f64,
        fuel_mass_cycle_cyl_kg: f64,
        heat_loss_cycle_j: f64,
        phase_eff: f64,
    ) -> f64 {
        let clearance_volume_cyl = self
            .clearance_volume_per_cylinder_m3()
            .max(self.model.clearance_volume_floor_m3);
        let compression_peak = (p_intake_pa
            * self
                .params
                .compression_ratio
                .powf(self.model.compression_peak_gamma))
        .clamp(p_intake_pa, self.model.compression_peak_max_pa);
        let fuel_heat_cycle_j = fuel_mass_cycle_cyl_kg * FUEL_LHV_J_PER_KG;
        let load = (p_intake_pa / self.env.ambient_pressure_pa)
            .clamp(self.model.load_min, self.model.load_max);
        let combustion_pressure_gain = self.model.combustion_pressure_gain
            * load.powf(self.model.combustion_pressure_load_exponent);
        let effective_fuel_heat_j = (fuel_heat_cycle_j - heat_loss_cycle_j).max(0.0);
        let delta_p_comb =
            combustion_pressure_gain * phase_eff * effective_fuel_heat_j / clearance_volume_cyl;
        (compression_peak + delta_p_comb).clamp(compression_peak, self.model.peak_pressure_max_pa)
    }

    pub(crate) fn build_ptheta_display_curves(&self, samples: usize) -> Vec<Vec<(f64, f64)>> {
        let rpm = rad_s_to_rpm(self.state.omega_rad_s.max(0.0));
        if !self.pv_display_active(rpm, self.state) {
            return Vec::new();
        }

        let count = samples.max(180);
        let firing_interval = self.firing_interval_deg();
        let mut global_curves = vec![Vec::with_capacity(count + 1); FIXED_CYLINDER_COUNT];

        for i in 0..=count {
            let global_theta_deg = 720.0 * i as f64 / count as f64;
            let sample_state = EngineState {
                theta_rad: global_theta_deg.to_radians(),
                ..self.state
            };
            let eval = self.eval(sample_state);
            let p_peak_pa = self.displayed_pv_peak_pressure_pa(
                eval.p_intake_cyl_pa,
                eval.fuel_mass_cycle_cyl_kg,
                eval.heat_loss_cycle_j,
                eval.phase_eff,
            );

            for cylinder_idx in 0..FIXED_CYLINDER_COUNT {
                let offset_deg = cylinder_idx as f64 * firing_interval;
                let local_theta_deg = (global_theta_deg - offset_deg).rem_euclid(720.0);
                let (_, pressure_pa) = instantaneous_pv_sample(
                    self.params.compression_ratio,
                    local_theta_deg.to_radians(),
                    eval.p_intake_cyl_pa,
                    eval.p_exhaust_cyl_pa,
                    p_peak_pa,
                    eval.combustion_enabled,
                    eval.burn_start_deg,
                    eval.burn_start_deg + eval.burn_duration_deg,
                    &self.model.pv_model,
                );
                global_curves[cylinder_idx].push((global_theta_deg, pressure_pa));
            }
        }

        global_curves
    }

    fn ignition_timing_deg(&self) -> f64 {
        self.control.ignition_timing_deg.clamp(
            self.model.ignition_timing_min_deg,
            self.model.ignition_timing_max_deg,
        )
    }

    fn ignition_phase_characteristics(
        &self,
        rpm: f64,
        load: f64,
        ignition_timing_deg: f64,
    ) -> (f64, f64, f64, bool, f64) {
        // MBT is represented as a reduced-order map plus asymmetric penalties for advance/retard.
        let mbt_deg = estimate_mbt_deg(&self.model, rpm, load);
        let spark_error_deg = ignition_timing_deg - mbt_deg;
        let phase_eff = if spark_error_deg >= 0.0 {
            (-(spark_error_deg / self.model.phase_sigma_advanced_deg).powi(2)).exp()
        } else {
            (-(spark_error_deg / self.model.phase_sigma_retarded_deg).powi(2)).exp()
        };
        let phase_stable = spark_error_deg <= self.model.phase_stable_advanced_limit_deg
            && spark_error_deg >= -self.model.phase_stable_retarded_limit_deg;
        let t_exh_scale = (1.0 + self.model.exhaust_temp_retard_gain * (-spark_error_deg).max(0.0)
            - self.model.exhaust_temp_advance_gain * spark_error_deg.max(0.0))
        .clamp(
            self.model.exhaust_temp_scale_min,
            self.model.exhaust_temp_scale_max,
        );
        let exhaust_temp_k = (self.env.exhaust_temp_k * t_exh_scale)
            .clamp(self.model.exhaust_temp_min_k, self.model.exhaust_temp_max_k);
        (
            mbt_deg,
            spark_error_deg,
            phase_eff,
            phase_stable,
            exhaust_temp_k,
        )
    }

    pub(crate) fn locked_cycle_average(
        &self,
        state: EngineState,
        samples: usize,
    ) -> LockedCycleAverage {
        // Sample a full 720 deg at fixed operating point for validation against catalog ranges.
        let count = samples.max(32);
        let mut torque_combustion_sum = 0.0;
        let mut torque_friction_sum = 0.0;
        let mut torque_pumping_sum = 0.0;
        let mut torque_load_sum = 0.0;
        let mut air_consumption_sum = 0.0;
        let mut trapped_air_sum = 0.0;
        let mut exhaust_temp_sum = 0.0;

        for i in 0..count {
            let cycle_deg = 720.0 * i as f64 / count as f64;
            let sample_state = EngineState {
                theta_rad: cycle_deg.to_radians(),
                ..state
            };
            let eval = self.eval(sample_state);
            torque_combustion_sum += eval.torque_combustion_nm;
            torque_friction_sum += eval.torque_friction_nm;
            torque_pumping_sum += eval.torque_pumping_nm;
            torque_load_sum += eval.torque_load_nm;
            air_consumption_sum += eval.m_dot_cyl * 1.0e3;
            trapped_air_sum += eval.trapped_air_kg * 1.0e6;
            exhaust_temp_sum += eval.exhaust_temp_k;
        }

        let inv_count = 1.0 / count as f64;
        let torque_combustion_nm = torque_combustion_sum * inv_count;
        let torque_friction_nm = torque_friction_sum * inv_count;
        let torque_pumping_nm = torque_pumping_sum * inv_count;
        let torque_load_nm = torque_load_sum * inv_count;
        LockedCycleAverage {
            torque_combustion_nm,
            torque_friction_nm,
            torque_pumping_nm,
            torque_load_nm,
            torque_net_nm: torque_combustion_nm
                - torque_friction_nm
                - torque_pumping_nm
                - torque_load_nm,
            air_consumption_gps: air_consumption_sum * inv_count,
            trapped_air_mg: trapped_air_sum * inv_count,
            exhaust_temp_k: exhaust_temp_sum * inv_count,
        }
    }

    fn eval(&self, state: EngineState) -> EvalPoint {
        // Evaluate all reduced-order closures at the current state before forming the ODE RHS.
        let rpm = rad_s_to_rpm(state.omega_rad_s.max(0.0));
        let throttle_area = self.params.throttle_area_max_m2
            * (self.model.throttle_area_offset
                + self.model.throttle_area_gain
                    * state.throttle_eff.powf(self.model.throttle_area_exponent))
            .clamp(self.model.throttle_area_min_scale, 1.0);
        let m_dot_throttle = orifice_mass_flow(
            throttle_area * self.model.throttle_discharge_coeff,
            self.env.ambient_pressure_pa,
            state.p_intake_pa,
            self.env.intake_temp_k,
        );

        let ve_base = volumetric_efficiency(
            rpm,
            self.control.vvt_intake_deg,
            self.control.vvt_exhaust_deg,
            state.throttle_eff,
            &self.model.volumetric_efficiency,
        );
        let lambda_target = self.target_lambda(state.throttle_eff);
        let base_intake_boundary_pa = self.base_intake_cylinder_boundary_pa(state);
        let load_wave_estimate = (base_intake_boundary_pa / self.env.ambient_pressure_pa)
            .clamp(self.model.load_min, self.model.load_max);
        let ignition_timing_deg = self.ignition_timing_deg();
        let (_, _, _, _, wave_exhaust_temp_k) =
            self.ignition_phase_characteristics(rpm, load_wave_estimate, ignition_timing_deg);
        let cycle_deg = state.theta_rad.to_degrees().rem_euclid(720.0);
        let wave_action = self.wave_action_state(
            state,
            cycle_deg,
            rpm,
            ve_base,
            lambda_target,
            base_intake_boundary_pa,
            wave_exhaust_temp_k,
        );
        let p_intake_cyl_pa =
            self.intake_cylinder_boundary_pa(state, wave_action.intake_boundary_wave_pa);
        let p_exhaust_cyl_pa =
            self.exhaust_cylinder_boundary_pa(state, wave_action.exhaust_boundary_wave_pa);
        let ve_effective =
            (ve_base * self.overlap_ve_multiplier(state) * wave_action.ve_pulse_multiplier).clamp(
                self.model.volumetric_efficiency.overall_min,
                self.model.volumetric_efficiency.overall_max,
            );
        let afr = self.model.stoich_afr * lambda_target;
        let trapped_air_uncooked = trapped_air_mass(
            self.params.displacement_m3,
            ve_effective,
            p_intake_cyl_pa,
            self.env.intake_temp_k,
            FIXED_CYLINDER_COUNT,
        );
        let fuel_mass_guess_kg = trapped_air_uncooked / afr;
        let load = (p_intake_cyl_pa / self.env.ambient_pressure_pa)
            .clamp(self.model.load_min, self.model.load_max);
        let (_mbt_deg, _spark_error_deg, phase_eff_base, phase_stable, base_exhaust_temp_k) =
            self.ignition_phase_characteristics(rpm, load, ignition_timing_deg);
        let internal_egr_fraction = self.internal_egr_fraction(state, wave_action);
        let charge_state_guess = self.charge_state(
            trapped_air_uncooked,
            fuel_mass_guess_kg,
            base_exhaust_temp_k,
            internal_egr_fraction,
        );
        let trapped_air = trapped_air_mass(
            self.params.displacement_m3,
            ve_effective,
            p_intake_cyl_pa,
            charge_state_guess.intake_charge_temp_k,
            FIXED_CYLINDER_COUNT,
        );
        let fuel_mass_per_cycle_cyl = trapped_air / afr;
        let charge_state = self.charge_state(
            trapped_air,
            fuel_mass_per_cycle_cyl,
            base_exhaust_temp_k,
            internal_egr_fraction,
        );
        let m_dot_cyl_mean = engine_air_consumption(
            self.params.displacement_m3,
            rpm,
            ve_effective,
            p_intake_cyl_pa,
            charge_state.intake_charge_temp_k,
        );
        let (intake_pulse_factor, exhaust_pulse_factor) = self.valve_pulse_factors(cycle_deg);
        let m_dot_cyl =
            m_dot_cyl_mean * intake_pulse_factor.max(0.0) * wave_action.intake_flow_multiplier;
        let phase_dilution = (1.0
            - self.model.internal_egr.phase_dilution_gain * internal_egr_fraction)
            .clamp(self.model.internal_egr.phase_dilution_min, 1.0);
        let phase_eff = phase_eff_base * phase_dilution;
        let burn_start = self.model.burn_start_base_deg - ignition_timing_deg
            + self.model.burn_start_vvt_intake_coeff * self.control.vvt_intake_deg;
        let burn_duration = (self.model.burn_duration_base_deg
            - self.model.burn_duration_throttle_coeff * state.throttle_eff)
            + self.model.internal_egr.burn_duration_gain_deg_per_fraction * internal_egr_fraction;
        let burn_duration = burn_duration.clamp(
            self.model.burn_duration_min_deg,
            self.model.burn_duration_max_deg,
        );
        let combustion_rate_norm = wiebe_combustion_rate(
            state.theta_rad.to_degrees(),
            FIXED_CYLINDER_COUNT,
            burn_start,
            burn_duration,
            self.model.wiebe_a,
            self.model.wiebe_m,
        );
        let fuel_heat_cycle_j = fuel_mass_per_cycle_cyl * FUEL_LHV_J_PER_KG;

        // Combustion torque is gated by start/run logic, then shaped over crank angle by Wiebe burn rate.
        let combustion_enabled = self.control.fuel_cmd
            && self.control.spark_cmd
            && rpm > self.model.combustion_enable_rpm_min
            && p_intake_cyl_pa > self.model.combustion_enable_intake_pressure_min_pa
            && phase_stable
            && (self.control.starter_cmd
                || state.running
                || rpm > self.model.combustion_enable_running_rpm_min);
        let cylinder_pressure_ref = (p_intake_cyl_pa
            * self
                .params
                .compression_ratio
                .powf(self.model.compression_peak_gamma))
        .clamp(p_intake_cyl_pa, self.model.compression_peak_max_pa);
        let heat_loss_cycle_j = if combustion_enabled {
            self.cylinder_heat_loss_cycle_j(
                rpm,
                cylinder_pressure_ref,
                charge_state.total_charge_mass_kg,
                fuel_mass_per_cycle_cyl,
                charge_state.intake_charge_temp_k,
                charge_state.mixture_gamma,
                charge_state.burned_gas_cp_j_per_kgk,
                phase_eff,
                burn_duration,
            )
        } else {
            0.0
        };
        let heat_loss_fraction = if fuel_heat_cycle_j > 0.0 {
            heat_loss_cycle_j / fuel_heat_cycle_j
        } else {
            0.0
        };
        let eta_thermal_base = (self.model.eta_base_offset + self.model.eta_load_coeff * load
            - self.model.eta_rpm_abs_coeff * (rpm - self.model.eta_rpm_reference).abs())
        .clamp(self.model.eta_base_min, self.model.eta_base_max);
        let eta_thermal = (eta_thermal_base * phase_eff - heat_loss_fraction)
            .clamp(self.model.eta_min, self.model.eta_max);
        let work_per_cycle_cyl = fuel_heat_cycle_j * eta_thermal;
        let mean_torque = work_per_cycle_cyl * FIXED_CYLINDER_COUNT as f64 / (4.0 * PI);

        let torque_combustion = if combustion_enabled {
            mean_torque * combustion_rate_norm.clamp(0.0, self.model.combustion_rate_max)
        } else {
            0.0
        };

        let torque_friction = self.params.friction_c0_nm
            + self.params.friction_c1_nms * state.omega_rad_s
            + self.params.friction_c2_nms2 * state.omega_rad_s.powi(2);
        let torque_pumping = ((p_exhaust_cyl_pa - p_intake_cyl_pa) * self.params.displacement_m3
            / (4.0 * PI))
            .clamp(
                self.model.pumping_torque_min_nm,
                self.model.pumping_torque_max_nm,
            );
        let torque_starter = if self.control.starter_cmd {
            starter_torque(rpm, &self.model)
        } else {
            0.0
        };
        let torque_load = external_load_torque_nm(
            self.control.load_cmd,
            state.omega_rad_s,
            &self.model.external_load,
        );

        let cycles_per_sec = rpm / 120.0;
        let fuel_mass_cycle_cyl = if combustion_enabled {
            fuel_mass_per_cycle_cyl
        } else {
            0.0
        };
        let m_dot_fuel = if combustion_enabled {
            fuel_mass_cycle_cyl * cycles_per_sec * FIXED_CYLINDER_COUNT as f64
        } else {
            0.0
        };
        let m_dot_exhaust_mean = m_dot_cyl_mean + m_dot_fuel;
        let m_dot_exhaust_in = m_dot_exhaust_mean
            * exhaust_pulse_factor.max(0.0)
            * wave_action.exhaust_flow_multiplier;
        let exhaust_temp_k = (base_exhaust_temp_k
            * (1.0 - self.model.heat_transfer.exhaust_temp_cooling_gain * heat_loss_fraction))
            .clamp(self.model.exhaust_temp_min_k, self.model.exhaust_temp_max_k);
        let m_dot_tailpipe = orifice_mass_flow(
            self.params.tailpipe_area_m2 * self.model.tailpipe_discharge_coeff,
            state.p_exhaust_pa,
            self.env.ambient_pressure_pa,
            exhaust_temp_k,
        );

        EvalPoint {
            m_dot_throttle,
            m_dot_cyl_mean,
            m_dot_cyl,
            m_dot_exhaust_in,
            m_dot_tailpipe,
            torque_combustion_nm: torque_combustion,
            torque_friction_nm: torque_friction,
            torque_starter_nm: torque_starter,
            torque_pumping_nm: torque_pumping,
            torque_load_nm: torque_load,
            p_intake_cyl_pa,
            p_exhaust_cyl_pa,
            trapped_air_kg: trapped_air,
            fuel_mass_cycle_cyl_kg: fuel_mass_cycle_cyl,
            combustion_rate_norm,
            combustion_enabled,
            burn_start_deg: burn_start,
            burn_duration_deg: burn_duration,
            phase_eff,
            ve_effective,
            intake_wave_current_pa: wave_action.intake_wave_current_pa,
            exhaust_wave_current_pa: wave_action.exhaust_wave_current_pa,
            intake_ram_multiplier: wave_action.intake_ram_multiplier,
            exhaust_scavenge_multiplier: wave_action.exhaust_scavenge_multiplier,
            ve_pulse_multiplier: wave_action.ve_pulse_multiplier,
            lambda_target,
            intake_charge_temp_k: charge_state.intake_charge_temp_k,
            heat_loss_cycle_j,
            ignition_timing_deg,
            exhaust_temp_k,
        }
    }

    fn derivatives(&self, state: EngineState, eval: EvalPoint) -> Derivatives {
        // The ODE itself stays compact because all nonlinear closures have already been evaluated.
        let net_torque = eval.torque_combustion_nm + eval.torque_starter_nm
            - eval.torque_friction_nm
            - eval.torque_pumping_nm
            - eval.torque_load_nm;
        let intake_runner_loss_pa = self.intake_runner_pressure_loss_pa(state);
        let exhaust_runner_loss_pa =
            self.exhaust_runner_pressure_loss_pa(state, eval.exhaust_temp_k);
        Derivatives {
            d_omega: net_torque / self.params.inertia_kgm2,
            d_theta: state.omega_rad_s,
            d_p_intake: (R_AIR * self.env.intake_temp_k / self.params.intake_volume_m3)
                * (eval.m_dot_throttle - state.m_dot_intake_runner_kg_s),
            d_p_intake_runner: (R_AIR * self.env.intake_temp_k
                / self.params.intake_runner_volume_m3)
                * (state.m_dot_intake_runner_kg_s - eval.m_dot_cyl),
            d_p_exhaust: (R_AIR * eval.exhaust_temp_k / self.params.exhaust_volume_m3)
                * (state.m_dot_exhaust_runner_kg_s - eval.m_dot_tailpipe),
            d_p_exhaust_runner: (R_AIR * eval.exhaust_temp_k
                / self.params.exhaust_runner_volume_m3)
                * (eval.m_dot_exhaust_in - state.m_dot_exhaust_runner_kg_s),
            d_m_dot_intake_runner: ((state.p_intake_pa - state.p_intake_runner_pa)
                - intake_runner_loss_pa)
                / self.intake_runner_inertance_pa_s2_per_kg()
                - self.model.gas_path.intake_runner_damping_per_s * state.m_dot_intake_runner_kg_s,
            d_m_dot_exhaust_runner: ((state.p_exhaust_runner_pa - state.p_exhaust_pa)
                - exhaust_runner_loss_pa)
                / self.exhaust_runner_inertance_pa_s2_per_kg()
                - self.model.gas_path.exhaust_runner_damping_per_s
                    * state.m_dot_exhaust_runner_kg_s,
            d_throttle: (self.control.throttle_cmd - state.throttle_eff)
                / self.model.throttle_time_constant_s,
        }
    }

    fn clamp_state(&self, mut state: EngineState) -> EngineState {
        state.omega_rad_s = state
            .omega_rad_s
            .max(0.0)
            .min(rpm_to_rad_s(self.params.max_rpm));
        state.theta_rad = wrap_cycle(state.theta_rad);
        state.p_intake_pa = state.p_intake_pa.clamp(
            self.model.intake_pressure_min_pa,
            self.model.intake_pressure_max_pa,
        );
        state.p_intake_runner_pa = state.p_intake_runner_pa.clamp(
            self.model.gas_path.runner_pressure_min_pa,
            self.model.gas_path.runner_pressure_max_pa,
        );
        state.p_exhaust_pa = state.p_exhaust_pa.clamp(
            self.env.ambient_pressure_pa * self.model.exhaust_pressure_min_ambient_ratio,
            self.env.ambient_pressure_pa + self.model.exhaust_pressure_max_over_ambient_pa,
        );
        state.p_exhaust_runner_pa = state.p_exhaust_runner_pa.clamp(
            self.model.gas_path.runner_pressure_min_pa,
            self.model.gas_path.runner_pressure_max_pa,
        );
        state.m_dot_intake_runner_kg_s = state.m_dot_intake_runner_kg_s.clamp(
            -self.model.gas_path.runner_flow_limit_kg_s,
            self.model.gas_path.runner_flow_limit_kg_s,
        );
        state.m_dot_exhaust_runner_kg_s = state.m_dot_exhaust_runner_kg_s.clamp(
            -self.model.gas_path.runner_flow_limit_kg_s,
            self.model.gas_path.runner_flow_limit_kg_s,
        );
        state.throttle_eff = state.throttle_eff.clamp(0.0, 1.0);
        state
    }

    fn update_running_flag(&self, state: &mut EngineState) {
        let rpm = rad_s_to_rpm(state.omega_rad_s);
        if self.control.spark_cmd && self.control.fuel_cmd && rpm > self.model.running_set_rpm {
            state.running = true;
        }
        if rpm < self.model.running_clear_rpm && !self.control.starter_cmd {
            state.running = false;
        }
    }

    fn advance_state_rk2_from_eval(
        &self,
        state: EngineState,
        eval_1: EvalPoint,
        dt: f64,
    ) -> EngineState {
        // Midpoint RK2 is the production integrator for the mean-value state vector.
        let k1 = self.derivatives(state, eval_1);
        let mid = integrate_state(state, k1, 0.5 * dt);
        let eval_2 = self.eval(mid);
        let k2 = self.derivatives(mid, eval_2);
        let mut next = integrate_state(state, k2, dt);
        next = self.clamp_state(next);
        self.update_running_flag(&mut next);
        next
    }

    pub(crate) fn advance_state_rk2(&self, state: EngineState, dt: f64) -> EngineState {
        let eval_1 = self.eval(state);
        self.advance_state_rk2_from_eval(state, eval_1, dt)
    }

    fn advance_state_rk3_from_eval(
        &self,
        state: EngineState,
        eval_1: EvalPoint,
        dt: f64,
    ) -> EngineState {
        // Classical Kutta RK3 keeps 3rd-order accuracy with one fewer stage than RK4.
        let k1 = self.derivatives(state, eval_1);
        let stage2 = integrate_state(state, k1, 0.5 * dt);
        let eval_2 = self.eval(stage2);
        let k2 = self.derivatives(stage2, eval_2);
        let k12 = weighted_derivatives2(k1, -1.0, k2, 2.0);
        let stage3 = integrate_state(state, k12, dt);
        let eval_3 = self.eval(stage3);
        let k3 = self.derivatives(stage3, eval_3);
        let mut next = integrate_state(
            state,
            weighted_derivatives3(k1, 1.0 / 6.0, k2, 4.0 / 6.0, k3, 1.0 / 6.0),
            dt,
        );
        next = self.clamp_state(next);
        self.update_running_flag(&mut next);
        next
    }

    pub(crate) fn advance_state_rk3(&self, state: EngineState, dt: f64) -> EngineState {
        let eval_1 = self.eval(state);
        self.advance_state_rk3_from_eval(state, eval_1, dt)
    }

    fn append_pv_history_samples(
        &mut self,
        eval_prev: EvalPoint,
        state_next: EngineState,
        eval_next: EvalPoint,
    ) -> bool {
        let theta_start = self.prev_theta_rad;
        let mut theta_end = state_next.theta_rad;
        let cycle_crossed = theta_end < theta_start;
        if cycle_crossed {
            theta_end += 4.0 * PI;
        }
        if theta_end - theta_start <= f64::EPSILON {
            return false;
        }

        let subsamples = self.plot.pv_subsamples_per_step.max(1);
        let combustion_active = eval_prev.combustion_enabled || eval_next.combustion_enabled;

        // Reconstruct a dense recent-cycle p-V loop by interpolating step endpoints instead of
        // re-running the full reduced-order model for every display-only subsample.
        for i in 1..=subsamples {
            let frac = i as f64 / subsamples as f64;
            let theta_unwrapped = theta_start + (theta_end - theta_start) * frac;
            let theta = theta_unwrapped.rem_euclid(4.0 * PI);
            let cycle = self.cycle_index + u64::from(theta_unwrapped >= 4.0 * PI);
            let p_intake = lerp(eval_prev.p_intake_cyl_pa, eval_next.p_intake_cyl_pa, frac);
            let p_exhaust = lerp(eval_prev.p_exhaust_cyl_pa, eval_next.p_exhaust_cyl_pa, frac);
            let phase_eff = lerp(eval_prev.phase_eff, eval_next.phase_eff, frac);
            let fuel_mass_cycle_cyl = lerp(
                eval_prev.fuel_mass_cycle_cyl_kg,
                eval_next.fuel_mass_cycle_cyl_kg,
                frac,
            )
            .max(0.0);
            let heat_loss_cycle_j = lerp(
                eval_prev.heat_loss_cycle_j,
                eval_next.heat_loss_cycle_j,
                frac,
            )
            .max(0.0);
            let burn_start_deg = lerp(eval_prev.burn_start_deg, eval_next.burn_start_deg, frac);
            let burn_duration_deg = lerp(
                eval_prev.burn_duration_deg,
                eval_next.burn_duration_deg,
                frac,
            )
            .max(1.0);
            let combustion_enabled = combustion_active
                && fuel_mass_cycle_cyl > self.model.fuel_mass_presence_threshold_kg;
            let p_peak = self.displayed_pv_peak_pressure_pa(
                p_intake,
                fuel_mass_cycle_cyl,
                heat_loss_cycle_j,
                phase_eff,
            );
            let (volume_rel, pressure_pa) = instantaneous_pv_sample(
                self.params.compression_ratio,
                theta,
                p_intake,
                p_exhaust,
                p_peak,
                combustion_enabled,
                burn_start_deg,
                burn_start_deg + burn_duration_deg,
                &self.model.pv_model,
            );
            push_bounded(
                &mut self.pv_history,
                PvSample {
                    cycle,
                    cycle_deg: theta.to_degrees().rem_euclid(720.0),
                    volume: volume_rel,
                    pressure_pa,
                },
            );
        }

        cycle_crossed
    }

    fn finalize_completed_cycle(&mut self) {
        if self.combustion_cycle_samples > 0 {
            self.combustion_cycle_mean_nm =
                self.combustion_cycle_accum_nm / self.combustion_cycle_samples as f64;
        }
        if let Some(work_j) = self.indicated_work_for_cycle_j(self.cycle_index) {
            push_bounded(&mut self.indicated_work_recent_j, work_j);
        }
        if self.fuel_cycle_samples > 0 {
            let mean_fuel = self.fuel_cycle_accum_kg / self.fuel_cycle_samples as f64;
            push_bounded(&mut self.fuel_cycle_recent_kg, mean_fuel.max(0.0));
        }
        if self.net_torque_cycle_samples > 0 {
            self.net_torque_cycle_mean_nm =
                self.net_torque_cycle_accum_nm / self.net_torque_cycle_samples as f64;
            push_bounded(
                &mut self.net_torque_cycle_recent_nm,
                self.net_torque_cycle_mean_nm,
            );
        }
        self.combustion_cycle_accum_nm = 0.0;
        self.combustion_cycle_samples = 0;
        self.fuel_cycle_accum_kg = 0.0;
        self.fuel_cycle_samples = 0;
        self.net_torque_cycle_accum_nm = 0.0;
        self.net_torque_cycle_samples = 0;
        self.cycle_index = self.cycle_index.wrapping_add(1);
    }

    fn trim_pv_history(&mut self) {
        let min_cycle = self
            .cycle_index
            .saturating_sub(self.plot.pv_recent_cycles.saturating_sub(1) as u64);
        while self
            .pv_history
            .front()
            .is_some_and(|sample| sample.cycle < min_cycle)
        {
            self.pv_history.pop_front();
        }
    }

    fn pv_display_active(&self, rpm: f64, state: EngineState) -> bool {
        self.control.starter_cmd || state.running || rpm > self.model.combustion_enable_rpm_min
    }

    pub(crate) fn step(&mut self, dt: f64) -> Observation {
        let state_prev = self.state;
        let rpm_now = rad_s_to_rpm(self.state.omega_rad_s.max(0.0));
        let load_now = (self.base_intake_cylinder_boundary_pa(self.state)
            / self.env.ambient_pressure_pa)
            .clamp(self.model.load_min, self.model.load_max);
        self.auto.apply(
            &mut self.control,
            rpm_now,
            self.state.running,
            self.params.idle_target_rpm,
            dt,
            &self.auto_control,
            &self.model,
            load_now,
        );
        let eval_prev = self.eval(state_prev);
        self.state = self.advance_state_rk3_from_eval(state_prev, eval_prev, dt);
        let rpm = rad_s_to_rpm(self.state.omega_rad_s);
        let cycle_crossed = self.state.theta_rad < self.prev_theta_rad;
        self.step_count = self.step_count.wrapping_add(1);

        let eval_final = self.eval(self.state);
        let torque_net_inst_nm = eval_final.torque_combustion_nm + eval_final.torque_starter_nm
            - eval_final.torque_friction_nm
            - eval_final.torque_pumping_nm
            - eval_final.torque_load_nm;
        let torque_target_nm = if self.cycle_index > 0 {
            self.mean_net_torque_cycle_recent_nm(self.model.net_torque_smoothing_cycles)
                .unwrap_or(self.net_torque_cycle_mean_nm)
        } else {
            torque_net_inst_nm
        };
        // Filter the displayed net torque so cycle-to-cycle pulsation remains readable in real time.
        let alpha_torque =
            (dt / (self.model.net_torque_filter_time_constant_s + dt)).clamp(0.0, 1.0);
        self.torque_net_filtered_nm +=
            alpha_torque * (torque_target_nm - self.torque_net_filtered_nm);
        self.combustion_cycle_accum_nm += eval_final.torque_combustion_nm;
        self.combustion_cycle_samples = self.combustion_cycle_samples.saturating_add(1);
        self.fuel_cycle_accum_kg += eval_final.fuel_mass_cycle_cyl_kg;
        self.fuel_cycle_samples = self.fuel_cycle_samples.saturating_add(1);
        self.net_torque_cycle_accum_nm += torque_net_inst_nm;
        self.net_torque_cycle_samples = self.net_torque_cycle_samples.saturating_add(1);
        let trapped_air_mg = eval_final.trapped_air_kg * 1e6;
        let pv_display_active = self.pv_display_active(rpm, self.state);
        if pv_display_active {
            let pv_cycle_crossed =
                self.append_pv_history_samples(eval_prev, self.state, eval_final);
            debug_assert_eq!(pv_cycle_crossed, cycle_crossed);
        } else {
            self.pv_history.clear();
        }

        if cycle_crossed {
            self.finalize_completed_cycle();
        }
        let cycle_deg = self.state.theta_rad.to_degrees().rem_euclid(720.0);
        let history_cycle = self.cycle_index;
        push_cycle_history_sample(&mut self.history_rpm, history_cycle, cycle_deg, rpm);
        push_cycle_history_sample(
            &mut self.history_torque_net_nm,
            history_cycle,
            cycle_deg,
            self.torque_net_filtered_nm,
        );
        push_cycle_history_sample(
            &mut self.history_torque_load_nm,
            history_cycle,
            cycle_deg,
            eval_final.torque_load_nm,
        );
        push_cycle_history_sample(
            &mut self.history_trapped_air_mg,
            history_cycle,
            cycle_deg,
            trapped_air_mg,
        );
        self.prev_theta_rad = self.state.theta_rad;
        if pv_display_active {
            self.trim_pv_history();
        }
        let pv_points = if pv_display_active {
            self.build_pv_display_points()
        } else {
            Vec::new()
        };
        let eta_thermal_theoretical = self.theoretical_otto_efficiency();
        let indicated_work_cycle_j = if pv_display_active {
            self.mean_indicated_work_recent_cycles_j(self.model.eta_indicated_average_cycles)
                .unwrap_or(0.0)
        } else {
            0.0
        };
        let swept_volume = self
            .swept_volume_per_cylinder_m3()
            .max(self.model.imep_swept_volume_floor_m3);
        let imep_cycle_pa = indicated_work_cycle_j / swept_volume;
        let fuel_cycle_kg = if pv_display_active {
            self.mean_fuel_cycle_recent_kg(self.model.eta_indicated_average_cycles)
                .unwrap_or(eval_final.fuel_mass_cycle_cyl_kg)
        } else {
            0.0
        };
        let fuel_heat_cycle_j = fuel_cycle_kg * FUEL_LHV_J_PER_KG;
        let eta_thermal_indicated_pv =
            if pv_display_active && fuel_cycle_kg > self.model.fuel_mass_presence_threshold_kg {
                (indicated_work_cycle_j / fuel_heat_cycle_j)
                    .clamp(self.model.eta_indicated_min, self.model.eta_indicated_max)
            } else {
                0.0
            };

        let observation = Observation {
            rpm,
            load_cmd: self.control.load_cmd,
            map_kpa: self.state.p_intake_pa * 1e-3,
            intake_runner_kpa: self.state.p_intake_runner_pa * 1e-3,
            exhaust_kpa: self.state.p_exhaust_pa * 1e-3,
            exhaust_runner_kpa: self.state.p_exhaust_runner_pa * 1e-3,
            intake_wave_kpa: eval_final.intake_wave_current_pa * 1.0e-3,
            exhaust_wave_kpa: eval_final.exhaust_wave_current_pa * 1.0e-3,
            exhaust_runner_flow_gps: self.state.m_dot_exhaust_runner_kg_s * 1.0e3,
            exhaust_temp_k: eval_final.exhaust_temp_k,
            intake_charge_temp_k: eval_final.intake_charge_temp_k,
            torque_combustion_nm: eval_final.torque_combustion_nm,
            torque_combustion_cycle_nm: self.combustion_cycle_mean_nm,
            torque_friction_nm: eval_final.torque_friction_nm,
            torque_pumping_nm: eval_final.torque_pumping_nm,
            torque_starter_nm: eval_final.torque_starter_nm,
            torque_load_nm: eval_final.torque_load_nm,
            torque_net_inst_nm,
            torque_net_nm: self.torque_net_filtered_nm,
            brake_bmep_bar: torque_to_bmep_bar(
                self.torque_net_filtered_nm,
                self.params.displacement_m3,
            ),
            air_flow_gps: eval_final.m_dot_cyl_mean * 1e3,
            trapped_air_mg,
            volumetric_efficiency: eval_final.ve_effective,
            intake_ram_multiplier: eval_final.intake_ram_multiplier,
            exhaust_scavenge_multiplier: eval_final.exhaust_scavenge_multiplier,
            ve_pulse_multiplier: eval_final.ve_pulse_multiplier,
            lambda_target: eval_final.lambda_target,
            heat_loss_cycle_j: eval_final.heat_loss_cycle_j,
            imep_bar: imep_cycle_pa * 1e-5,
            eta_thermal_theoretical,
            eta_thermal_indicated_pv,
            indicated_work_cycle_j,
            combustion_rate_norm: eval_final.combustion_rate_norm,
            ignition_timing_deg: eval_final.ignition_timing_deg,
            cycle_deg: self.state.theta_rad.to_degrees(),
            stable_idle: self.state.running
                && (rpm - self.params.idle_target_rpm).abs() < self.model.stable_idle_rpm_band
                && self.control.throttle_cmd < self.model.stable_idle_throttle_max,
            pv_points,
        };
        self.auto
            .observe_wot_efficiency(&observation, dt, &self.auto_control, &self.model);
        observation
    }
}

fn mean_recent(history: &VecDeque<f64>, n_cycles: usize) -> Option<f64> {
    if n_cycles == 0 || history.is_empty() {
        return None;
    }
    let count = n_cycles.min(history.len());
    let sum: f64 = history.iter().rev().take(count).copied().sum();
    Some(sum / count as f64)
}

fn push_cycle_history_sample(
    history: &mut VecDeque<CycleHistorySample>,
    cycle: u64,
    cycle_deg: f64,
    value: f64,
) {
    push_bounded(
        history,
        CycleHistorySample {
            cycle,
            cycle_deg: cycle_deg.rem_euclid(720.0),
            value,
        },
    );
}

fn push_bounded<T>(history: &mut VecDeque<T>, value: T) {
    if history.len() == history.capacity() {
        history.pop_front();
    }
    history.push_back(value);
}

fn lerp(a: f64, b: f64, t: f64) -> f64 {
    a + (b - a) * t
}

fn estimate_mbt_deg(model: &ModelConfig, rpm: f64, load: f64) -> f64 {
    (model.mbt_base_deg
        + model.mbt_rpm_slope_deg_per_rpm * (rpm - model.mbt_rpm_reference)
        + model.mbt_load_coeff * (1.0 - load))
        .clamp(model.mbt_min_deg, model.mbt_max_deg)
}

fn clamp_wot_search_point(
    point: WotSearchPoint,
    tuning: &AutoControlConfig,
    model: &ModelConfig,
) -> WotSearchPoint {
    WotSearchPoint {
        ignition_deg: point
            .ignition_deg
            .clamp(model.ignition_timing_min_deg, model.ignition_timing_max_deg),
        vvt_intake_deg: point
            .vvt_intake_deg
            .clamp(tuning.wot_search_vvt_min_deg, tuning.wot_search_vvt_max_deg),
        vvt_exhaust_deg: point
            .vvt_exhaust_deg
            .clamp(tuning.wot_search_vvt_min_deg, tuning.wot_search_vvt_max_deg),
    }
}

fn step_wot_search_point(
    point: WotSearchPoint,
    axis: WotSearchAxis,
    direction: f64,
    tuning: &AutoControlConfig,
    model: &ModelConfig,
) -> WotSearchPoint {
    let mut next = point;
    match axis {
        WotSearchAxis::Ignition => {
            next.ignition_deg += direction * tuning.wot_search_ignition_step_deg;
        }
        WotSearchAxis::IntakeVvt => {
            next.vvt_intake_deg += direction * tuning.wot_search_vvt_intake_step_deg;
        }
        WotSearchAxis::ExhaustVvt => {
            next.vvt_exhaust_deg += direction * tuning.wot_search_vvt_exhaust_step_deg;
        }
    }
    clamp_wot_search_point(next, tuning, model)
}

pub(crate) fn integrate_state(state: EngineState, k: Derivatives, dt: f64) -> EngineState {
    EngineState {
        omega_rad_s: state.omega_rad_s + k.d_omega * dt,
        theta_rad: state.theta_rad + k.d_theta * dt,
        p_intake_pa: state.p_intake_pa + k.d_p_intake * dt,
        p_intake_runner_pa: state.p_intake_runner_pa + k.d_p_intake_runner * dt,
        p_exhaust_pa: state.p_exhaust_pa + k.d_p_exhaust * dt,
        p_exhaust_runner_pa: state.p_exhaust_runner_pa + k.d_p_exhaust_runner * dt,
        m_dot_intake_runner_kg_s: state.m_dot_intake_runner_kg_s + k.d_m_dot_intake_runner * dt,
        m_dot_exhaust_runner_kg_s: state.m_dot_exhaust_runner_kg_s + k.d_m_dot_exhaust_runner * dt,
        throttle_eff: state.throttle_eff + k.d_throttle * dt,
        running: state.running,
    }
}

fn weighted_derivatives2(a: Derivatives, wa: f64, b: Derivatives, wb: f64) -> Derivatives {
    Derivatives {
        d_omega: a.d_omega * wa + b.d_omega * wb,
        d_theta: a.d_theta * wa + b.d_theta * wb,
        d_p_intake: a.d_p_intake * wa + b.d_p_intake * wb,
        d_p_intake_runner: a.d_p_intake_runner * wa + b.d_p_intake_runner * wb,
        d_p_exhaust: a.d_p_exhaust * wa + b.d_p_exhaust * wb,
        d_p_exhaust_runner: a.d_p_exhaust_runner * wa + b.d_p_exhaust_runner * wb,
        d_m_dot_intake_runner: a.d_m_dot_intake_runner * wa + b.d_m_dot_intake_runner * wb,
        d_m_dot_exhaust_runner: a.d_m_dot_exhaust_runner * wa + b.d_m_dot_exhaust_runner * wb,
        d_throttle: a.d_throttle * wa + b.d_throttle * wb,
    }
}

fn weighted_derivatives3(
    a: Derivatives,
    wa: f64,
    b: Derivatives,
    wb: f64,
    c: Derivatives,
    wc: f64,
) -> Derivatives {
    Derivatives {
        d_omega: a.d_omega * wa + b.d_omega * wb + c.d_omega * wc,
        d_theta: a.d_theta * wa + b.d_theta * wb + c.d_theta * wc,
        d_p_intake: a.d_p_intake * wa + b.d_p_intake * wb + c.d_p_intake * wc,
        d_p_intake_runner: a.d_p_intake_runner * wa
            + b.d_p_intake_runner * wb
            + c.d_p_intake_runner * wc,
        d_p_exhaust: a.d_p_exhaust * wa + b.d_p_exhaust * wb + c.d_p_exhaust * wc,
        d_p_exhaust_runner: a.d_p_exhaust_runner * wa
            + b.d_p_exhaust_runner * wb
            + c.d_p_exhaust_runner * wc,
        d_m_dot_intake_runner: a.d_m_dot_intake_runner * wa
            + b.d_m_dot_intake_runner * wb
            + c.d_m_dot_intake_runner * wc,
        d_m_dot_exhaust_runner: a.d_m_dot_exhaust_runner * wa
            + b.d_m_dot_exhaust_runner * wb
            + c.d_m_dot_exhaust_runner * wc,
        d_throttle: a.d_throttle * wa + b.d_throttle * wb + c.d_throttle * wc,
    }
}

pub(crate) fn wrap_cycle(theta_rad: f64) -> f64 {
    theta_rad.rem_euclid(2.0 * TAU)
}

pub(crate) fn rad_s_to_rpm(rad_s: f64) -> f64 {
    rad_s * 60.0 / TAU
}

pub(crate) fn rpm_linked_dt(
    base_dt: f64,
    rpm: f64,
    idle_target_rpm: f64,
    numerics: &NumericsConfig,
) -> f64 {
    // Keep crank-angle advance per step roughly constant so high RPM gets finer time resolution.
    let base = base_dt.max(numerics.rpm_link_base_dt_min_s);
    let idle = idle_target_rpm.max(numerics.rpm_link_idle_rpm_min);
    let target_deg_per_step = (base * idle * 6.0).clamp(
        numerics.rpm_link_deg_per_step_min,
        numerics.rpm_link_deg_per_step_max,
    );
    let rpm_eff = rpm.max(numerics.rpm_link_rpm_floor);
    let dt = target_deg_per_step / (rpm_eff * 6.0);
    let dt_min = (base * numerics.rpm_link_dt_min_factor).max(numerics.rpm_link_dt_min_floor_s);
    let dt_max = (base * numerics.rpm_link_dt_max_factor).min(numerics.rpm_link_dt_max_cap_s);
    dt.clamp(dt_min, dt_max)
}

fn realtime_fixed_dt_candidate(base_dt: f64, max_rpm: f64, numerics: &NumericsConfig) -> f64 {
    let rpm_ref = max_rpm.max(numerics.rpm_link_rpm_floor).max(f64::EPSILON);
    let max_deg = numerics
        .realtime_fixed_dt_max_deg_per_step
        .max(f64::EPSILON);
    base_dt.min(max_deg / (rpm_ref * 6.0))
}

#[cfg(test)]
fn bench_nominal_dt(bench: &BenchConfig, rpm: f64, numerics: &NumericsConfig) -> f64 {
    bench_nominal_dt_for_deg_per_step(bench, rpm, numerics, bench.integration_deg_per_step)
}

fn bench_nominal_dt_for_deg_per_step(
    bench: &BenchConfig,
    rpm: f64,
    numerics: &NumericsConfig,
    deg_per_step: f64,
) -> f64 {
    let rpm_eff = rpm.max(numerics.rpm_link_rpm_floor);
    let dt = deg_per_step.max(f64::EPSILON) / (rpm_eff * 6.0);
    dt.clamp(bench.integration_dt_min_s, bench.integration_dt_max_s)
}

fn bench_sweep_target_rpm(bench: &BenchConfig, elapsed_s: f64) -> f64 {
    let duration = bench.sweep_duration_s.max(f64::EPSILON);
    let progress = (elapsed_s / duration).clamp(0.0, 1.0);
    lerp(
        bench.rpm_start_rpm.max(0.0),
        bench.rpm_end_rpm.max(bench.rpm_start_rpm),
        progress,
    )
}

fn nearest_bench_target_idx(targets_rpm: &[f64], rpm: f64) -> usize {
    targets_rpm
        .iter()
        .enumerate()
        .min_by(|(_, a), (_, b)| ((*a - rpm).abs()).total_cmp(&((*b - rpm).abs())))
        .map(|(idx, _)| idx)
        .unwrap_or(0)
}

pub(crate) fn state_error_norm(
    coarse: EngineState,
    fine: EngineState,
    numerics: &NumericsConfig,
) -> f64 {
    state_error_norm_internal(coarse, fine, numerics, true)
}

#[cfg(test)]
pub(crate) fn running_state_error_norm(
    coarse: EngineState,
    fine: EngineState,
    numerics: &NumericsConfig,
) -> f64 {
    state_error_norm_internal(coarse, fine, numerics, false)
}

fn state_error_norm_internal(
    coarse: EngineState,
    fine: EngineState,
    numerics: &NumericsConfig,
    include_theta: bool,
) -> f64 {
    // Normalize each state component before taking the max so convergence checks stay well-scaled.
    let cycle_period = 2.0 * TAU;
    let mut dtheta = (coarse.theta_rad - fine.theta_rad).rem_euclid(cycle_period);
    if dtheta > TAU {
        dtheta -= cycle_period;
    }
    let e_omega = (coarse.omega_rad_s - fine.omega_rad_s).abs()
        / (numerics.state_error_omega_bias + fine.omega_rad_s.abs());
    let e_theta = dtheta.abs() / numerics.state_error_theta_scale_rad;
    let e_pim = (coarse.p_intake_pa - fine.p_intake_pa).abs()
        / (numerics.state_error_p_intake_bias_pa
            + numerics.state_error_p_intake_rel * fine.p_intake_pa);
    let e_pir = (coarse.p_intake_runner_pa - fine.p_intake_runner_pa).abs()
        / (numerics.state_error_p_intake_runner_bias_pa
            + numerics.state_error_p_intake_runner_rel * fine.p_intake_runner_pa);
    let e_pexh = (coarse.p_exhaust_pa - fine.p_exhaust_pa).abs()
        / (numerics.state_error_p_exhaust_bias_pa
            + numerics.state_error_p_exhaust_rel * fine.p_exhaust_pa);
    let e_per = (coarse.p_exhaust_runner_pa - fine.p_exhaust_runner_pa).abs()
        / (numerics.state_error_p_exhaust_runner_bias_pa
            + numerics.state_error_p_exhaust_runner_rel * fine.p_exhaust_runner_pa);
    let e_mi = (coarse.m_dot_intake_runner_kg_s - fine.m_dot_intake_runner_kg_s).abs()
        / numerics.state_error_m_dot_intake_runner_scale_kg_s;
    let e_me = (coarse.m_dot_exhaust_runner_kg_s - fine.m_dot_exhaust_runner_kg_s).abs()
        / numerics.state_error_m_dot_exhaust_runner_scale_kg_s;
    let e_th =
        (coarse.throttle_eff - fine.throttle_eff).abs() / numerics.state_error_throttle_scale;
    let mut err = e_omega
        .max(e_pim)
        .max(e_pir)
        .max(e_pexh)
        .max(e_per)
        .max(e_mi)
        .max(e_me)
        .max(e_th);
    if include_theta {
        err = err.max(e_theta);
    }
    err
}

fn bench_state_step_error(sim: &Simulator, dt: f64) -> f64 {
    let state = sim.state;
    let state_full = sim.advance_state_rk3(state, dt);
    let state_half = sim.advance_state_rk3(sim.advance_state_rk3(state, dt * 0.5), dt * 0.5);
    state_error_norm(state_full, state_half, &sim.numerics)
}

fn bench_commit_step(
    sim: &mut Simulator,
    config: &AppConfig,
    dyno: &mut BenchDynoController,
    dt: f64,
    target_rpm: f64,
) -> crate::simulator::Observation {
    let load_cmd = dyno.update(
        target_rpm,
        rad_s_to_rpm(sim.state.omega_rad_s),
        dt,
        &config.bench,
    );
    sim.control.load_cmd = load_cmd;
    sim.step(dt)
}

#[cfg(test)]
fn bench_adaptive_step(
    sim: &mut Simulator,
    config: &AppConfig,
    dyno: &mut BenchDynoController,
    target_rpm: f64,
    phase_remaining_s: f64,
    dt_next_s: &mut f64,
) -> (crate::simulator::Observation, f64) {
    bench_adaptive_step_with_deg_per_step(
        sim,
        config,
        dyno,
        target_rpm,
        phase_remaining_s,
        dt_next_s,
        config.bench.integration_deg_per_step,
    )
}

fn bench_adaptive_step_with_deg_per_step(
    sim: &mut Simulator,
    config: &AppConfig,
    dyno: &mut BenchDynoController,
    target_rpm: f64,
    phase_remaining_s: f64,
    dt_next_s: &mut f64,
    deg_per_step: f64,
) -> (crate::simulator::Observation, f64) {
    let dt_nominal = bench_nominal_dt_for_deg_per_step(
        &config.bench,
        target_rpm,
        &config.numerics,
        deg_per_step,
    );
    let min_dt = if phase_remaining_s <= config.bench.integration_dt_min_s {
        phase_remaining_s.max(f64::EPSILON)
    } else {
        config.bench.integration_dt_min_s
    };
    let max_dt = dt_nominal
        .min(config.bench.integration_dt_max_s)
        .min(phase_remaining_s.max(min_dt));
    let mut dt = if *dt_next_s > 0.0 {
        *dt_next_s
    } else {
        dt_nominal
    }
    .clamp(min_dt, max_dt);
    let mut error = bench_state_step_error(sim, dt);
    let tolerance = config.bench.integration_error_tolerance.max(f64::EPSILON);
    let mut refinements = 0usize;
    while error > tolerance
        && refinements < config.bench.integration_refine_limit
        && dt > min_dt * (1.0 + f64::EPSILON)
    {
        dt = (dt * 0.5).max(min_dt);
        error = bench_state_step_error(sim, dt);
        refinements = refinements.saturating_add(1);
    }

    let obs = bench_commit_step(sim, config, dyno, dt, target_rpm);
    let growth = if error < tolerance * 0.25 {
        config.bench.integration_dt_growth.max(1.0)
    } else {
        1.0
    };
    *dt_next_s = (dt * growth).clamp(config.bench.integration_dt_min_s, dt_nominal);
    (obs, dt)
}

#[derive(Debug, Clone, Copy)]
pub(crate) struct RealtimePerformanceEstimate {
    pub(crate) wall_per_step_s: f64,
    pub(crate) floor_dt_s: f64,
    pub(crate) fixed_dt_candidate_s: f64,
    pub(crate) fixed_dt_headroom_ratio: f64,
}

pub(crate) fn estimate_realtime_performance(
    config: &AppConfig,
    dt_base: f64,
) -> RealtimePerformanceEstimate {
    // Per-step cost is dominated by the ODE RHS and stays almost dt-independent for this model.
    let probe_dt = dt_base.max(config.numerics.realtime_probe_dt_min_s);
    let mut sim = Simulator::new(config);
    sim.auto.enabled = true;
    for _ in 0..config.numerics.realtime_warmup_steps {
        sim.step(probe_dt);
    }
    let samples = config.numerics.realtime_sample_steps.max(1);
    let t0 = Instant::now();
    for _ in 0..samples {
        sim.step(probe_dt);
    }
    let wall_per_step = t0.elapsed().as_secs_f64() / samples as f64;
    let floor_dt_s = (wall_per_step * config.numerics.realtime_margin_factor).clamp(
        config.numerics.realtime_floor_min_s,
        probe_dt * config.numerics.realtime_floor_probe_factor_max,
    );
    let fixed_dt_candidate_s =
        realtime_fixed_dt_candidate(dt_base, config.engine.max_rpm, &config.numerics);
    RealtimePerformanceEstimate {
        wall_per_step_s: wall_per_step,
        floor_dt_s,
        fixed_dt_candidate_s,
        fixed_dt_headroom_ratio: fixed_dt_candidate_s / floor_dt_s.max(f64::EPSILON),
    }
}

pub(crate) fn rpm_to_rad_s(rpm: f64) -> f64 {
    rpm * TAU / 60.0
}

pub(crate) fn shaft_power_w(rpm: f64, torque_nm: f64) -> f64 {
    torque_nm * rpm_to_rad_s(rpm)
}

pub(crate) fn shaft_power_kw(rpm: f64, torque_nm: f64) -> f64 {
    shaft_power_w(rpm, torque_nm) / W_PER_KW
}

pub(crate) fn shaft_power_hp(rpm: f64, torque_nm: f64) -> f64 {
    shaft_power_w(rpm, torque_nm) / W_PER_MECHANICAL_HP
}

pub(crate) fn torque_to_bmep_bar(torque_nm: f64, displacement_m3: f64) -> f64 {
    if displacement_m3 <= 0.0 {
        return 0.0;
    }
    4.0 * PI * torque_nm / displacement_m3 * 1.0e-5
}

pub(crate) fn external_load_torque_nm(
    load_cmd: f64,
    omega_rad_s: f64,
    load_model: &ExternalLoadConfig,
) -> f64 {
    let command = load_cmd.clamp(0.0, 1.0);
    if command <= 0.0 {
        return 0.0;
    }
    let exponent = load_model.command_exponent.max(f64::EPSILON);
    let command = command.powf(exponent);
    let omega = omega_rad_s.max(0.0);
    let reference_torque = load_model.base_torque_nm
        + load_model.speed_linear_nms * omega
        + load_model.speed_quadratic_nms2 * omega.powi(2);
    (command * reference_torque).clamp(load_model.torque_min_nm, load_model.torque_max_nm)
}

pub(crate) fn bench_rpm_targets(bench: &BenchConfig) -> Vec<f64> {
    let start = bench.rpm_start_rpm.max(0.0);
    let end = bench.rpm_end_rpm.max(start);
    let step = bench.rpm_step_rpm.max(1.0);
    let count = (((end - start) / step).floor() as usize).saturating_add(1);
    let mut targets = Vec::with_capacity(count.max(1));
    let mut rpm = start;
    while rpm <= end + 0.5 * step {
        targets.push(rpm.min(end));
        rpm += step;
        if targets
            .last()
            .is_some_and(|last| (*last - end).abs() < f64::EPSILON)
        {
            break;
        }
    }
    if targets.is_empty() {
        targets.push(start);
    }
    if targets.last().copied().unwrap_or(start) < end {
        targets.push(end);
    }
    targets
}

fn seed_bench_simulator(
    config: &AppConfig,
    control_seed: &ControlInput,
    mode: BenchMixtureMode,
    target_rpm: f64,
) -> (Simulator, f64) {
    let mut sim = Simulator::new(config);
    sim.configure_bench_mixture_mode(mode, &config.bench);
    sim.model.external_load = config.bench.dyno.absorber_model.clone();

    let mut control = control_seed.clone();
    let throttle_target = config.auto_control.wot_target_throttle.clamp(0.0, 1.0);
    control.throttle_cmd = throttle_target;
    control.load_cmd = config.bench.dyno.initial_load_cmd.clamp(0.0, 1.0);
    control.starter_cmd = false;
    control.spark_cmd = true;
    control.fuel_cmd = true;
    sim.control = control;

    let target_rpm = target_rpm.clamp(config.bench.rpm_start_rpm.max(0.0), sim.params.max_rpm);
    let target_omega_rad_s = rpm_to_rad_s(target_rpm);
    let initial_intake_pa =
        (config.environment.ambient_pressure_pa * config.bench.initial_map_ratio).clamp(
            sim.model.intake_pressure_min_pa,
            sim.model.intake_pressure_max_pa,
        );
    let initial_exhaust_pa = (config.environment.ambient_pressure_pa
        + config.bench.initial_exhaust_over_ambient_pa)
        .clamp(
            config.environment.ambient_pressure_pa * sim.model.exhaust_pressure_min_ambient_ratio,
            config.environment.ambient_pressure_pa + sim.model.exhaust_pressure_max_over_ambient_pa,
        );
    sim.state = EngineState {
        omega_rad_s: target_omega_rad_s,
        theta_rad: 410.0_f64.to_radians(),
        p_intake_pa: initial_intake_pa,
        p_exhaust_pa: initial_exhaust_pa,
        throttle_eff: throttle_target,
        running: true,
        p_intake_runner_pa: initial_intake_pa.clamp(
            sim.model.gas_path.runner_pressure_min_pa,
            sim.model.gas_path.runner_pressure_max_pa,
        ),
        p_exhaust_runner_pa: initial_exhaust_pa.clamp(
            sim.model.gas_path.runner_pressure_min_pa,
            sim.model.gas_path.runner_pressure_max_pa,
        ),
        m_dot_intake_runner_kg_s: 0.0,
        m_dot_exhaust_runner_kg_s: 0.0,
    };
    sim.prev_theta_rad = sim.state.theta_rad;
    (sim, target_rpm)
}

impl BenchSession {
    pub(crate) fn new(
        config: &AppConfig,
        control_seed: ControlInput,
        mode: BenchMixtureMode,
    ) -> Self {
        let mut session = Self {
            config: config.clone(),
            mode,
            control_seed,
            targets_rpm: bench_rpm_targets(&config.bench),
            current_idx: 0,
            phase: BenchPhase::Idle,
            phase_elapsed_s: 0.0,
            sim: None,
            dyno: BenchDynoController::new(config.bench.dyno.initial_load_cmd),
            bins: vec![BenchAccumulator::default(); bench_rpm_targets(&config.bench).len()],
            results: Vec::new(),
            live_rpm: 0.0,
            live_torque_brake_nm: 0.0,
            live_load_cmd: config.bench.dyno.initial_load_cmd.clamp(0.0, 1.0),
            live_load_torque_nm: 0.0,
            live_eta_thermal_indicated_pv: 0.0,
            live_map_kpa: 0.0,
            live_lambda_target: 0.0,
            live_intake_charge_temp_k: config.environment.intake_temp_k,
            dt_next_s: 0.0,
        };
        session.prepare_sweep();
        session
    }

    pub(crate) fn stop(&mut self) {
        self.phase = BenchPhase::Idle;
        self.sim = None;
        self.phase_elapsed_s = 0.0;
        self.dt_next_s = 0.0;
    }

    pub(crate) fn is_active(&self) -> bool {
        matches!(self.phase, BenchPhase::Warmup | BenchPhase::Sweep)
    }

    pub(crate) fn is_complete(&self) -> bool {
        self.phase == BenchPhase::Complete
    }

    pub(crate) fn results(&self) -> &[BenchSample] {
        &self.results
    }

    pub(crate) fn status(&self) -> BenchStatus {
        let total_points = self.targets_rpm.len();
        let target_rpm = self.current_target_rpm();
        let phase_total_s = match self.phase {
            BenchPhase::Warmup => self.config.bench.sweep_warmup_time_s,
            BenchPhase::Sweep => self.config.bench.sweep_duration_s,
            BenchPhase::Idle | BenchPhase::Complete => 0.0,
        };
        BenchStatus {
            mode: self.mode,
            phase: self.phase,
            target_rpm,
            completed_points: self.results.len(),
            total_points,
            phase_elapsed_s: self.phase_elapsed_s,
            phase_total_s,
            live_rpm: self.live_rpm,
            live_torque_brake_nm: self.live_torque_brake_nm,
            live_load_cmd: self.live_load_cmd,
            live_load_torque_nm: self.live_load_torque_nm,
            live_eta_thermal_indicated_pv: self.live_eta_thermal_indicated_pv,
            live_map_kpa: self.live_map_kpa,
            live_lambda_target: self.live_lambda_target,
            live_intake_charge_temp_k: self.live_intake_charge_temp_k,
        }
    }

    pub(crate) fn live_sample(&self) -> Option<BenchSample> {
        if !self.is_active() || self.targets_rpm.is_empty() {
            return None;
        }
        Some(BenchSample {
            target_rpm: self.current_target_rpm(),
            rpm: self.live_rpm,
            torque_brake_nm: self.live_torque_brake_nm,
            load_cmd: self.live_load_cmd,
            load_torque_nm: self.live_load_torque_nm,
            map_kpa: self.live_map_kpa,
            eta_thermal_indicated_pv: self.live_eta_thermal_indicated_pv,
            lambda_target: self.live_lambda_target,
            intake_charge_temp_k: self.live_intake_charge_temp_k,
            mixture_mode: self.mode,
        })
    }

    pub(crate) fn advance_steps(&mut self, max_steps: usize) {
        if max_steps == 0 || !self.is_active() {
            return;
        }
        for _ in 0..max_steps {
            if !self.is_active() {
                break;
            }
            self.advance_one_step();
        }
    }

    fn current_target_rpm(&self) -> f64 {
        if self.targets_rpm.is_empty() {
            return 0.0;
        }
        match self.phase {
            BenchPhase::Warmup => self.targets_rpm[0],
            BenchPhase::Sweep => bench_sweep_target_rpm(&self.config.bench, self.phase_elapsed_s),
            BenchPhase::Idle => self.targets_rpm[self.current_idx.min(self.targets_rpm.len() - 1)],
            BenchPhase::Complete => self.targets_rpm.last().copied().unwrap_or(0.0),
        }
    }

    fn prepare_sweep(&mut self) {
        if self.targets_rpm.is_empty() {
            self.phase = if self.targets_rpm.is_empty() {
                BenchPhase::Idle
            } else {
                BenchPhase::Complete
            };
            self.sim = None;
            self.phase_elapsed_s = 0.0;
            return;
        }

        let (sim, target_rpm) = seed_bench_simulator(
            &self.config,
            &self.control_seed,
            self.mode,
            self.targets_rpm[0],
        );

        self.phase = if self.config.bench.sweep_warmup_time_s > 0.0 {
            BenchPhase::Warmup
        } else {
            BenchPhase::Sweep
        };
        self.current_idx = 0;
        self.phase_elapsed_s = 0.0;
        for bin in &mut self.bins {
            bin.clear();
        }
        self.dyno = BenchDynoController::new(self.config.bench.dyno.initial_load_cmd);
        self.results.clear();
        self.live_rpm = target_rpm;
        self.live_torque_brake_nm = 0.0;
        self.live_load_cmd = self.config.bench.dyno.initial_load_cmd.clamp(0.0, 1.0);
        self.live_load_torque_nm = 0.0;
        self.live_eta_thermal_indicated_pv = 0.0;
        self.live_map_kpa = sim.state.p_intake_pa * 1.0e-3;
        self.live_lambda_target = sim
            .lambda_override
            .unwrap_or(self.config.bench.lambda_one_target);
        self.live_intake_charge_temp_k = self.config.environment.intake_temp_k;
        self.dt_next_s = bench_nominal_dt_for_deg_per_step(
            &self.config.bench,
            target_rpm,
            &self.config.numerics,
            self.config.bench.sweep_integration_deg_per_step,
        );
        self.sim = Some(sim);
    }

    fn advance_one_step(&mut self) {
        let target_rpm = self.current_target_rpm();
        self.current_idx = nearest_bench_target_idx(&self.targets_rpm, target_rpm);
        let phase_total_s = match self.phase {
            BenchPhase::Warmup => self.config.bench.sweep_warmup_time_s,
            BenchPhase::Sweep => self.config.bench.sweep_duration_s,
            BenchPhase::Idle | BenchPhase::Complete => 0.0,
        };
        let phase_remaining_s = (phase_total_s - self.phase_elapsed_s).max(
            self.config
                .bench
                .integration_dt_min_s
                .min(phase_total_s.max(0.0)),
        );

        let (obs, dt_used) = {
            let sim = self
                .sim
                .as_mut()
                .expect("bench session should own a simulator while active");
            bench_adaptive_step_with_deg_per_step(
                sim,
                &self.config,
                &mut self.dyno,
                target_rpm,
                phase_remaining_s,
                &mut self.dt_next_s,
                self.config.bench.sweep_integration_deg_per_step,
            )
        };

        self.live_rpm = obs.rpm;
        self.live_torque_brake_nm = obs.torque_net_nm + obs.torque_load_nm;
        self.live_load_cmd = obs.load_cmd;
        self.live_load_torque_nm = obs.torque_load_nm;
        self.live_map_kpa = obs.map_kpa;
        self.live_lambda_target = obs.lambda_target;
        self.live_intake_charge_temp_k = obs.intake_charge_temp_k;
        if self.phase == BenchPhase::Sweep {
            let sim_state = self
                .sim
                .as_ref()
                .expect("bench session should own a simulator while sweeping")
                .state;
            let bin_idx = nearest_bench_target_idx(&self.targets_rpm, target_rpm);
            self.bins[bin_idx].accumulate(&obs, sim_state);
            self.refresh_results_from_bins();
            let bin = &self.bins[bin_idx];
            self.live_rpm = bin.mean_rpm();
            self.live_torque_brake_nm = bin.mean_torque_brake_nm();
            self.live_load_cmd = bin.mean_load_cmd();
            self.live_load_torque_nm = bin.mean_load_torque_nm();
            self.live_map_kpa = bin.mean_map_kpa();
            self.live_eta_thermal_indicated_pv = bin.mean_eta();
            self.live_lambda_target = bin.mean_lambda_target();
            self.live_intake_charge_temp_k = bin.mean_intake_charge_temp_k();
        } else {
            self.live_eta_thermal_indicated_pv = 0.0;
        }

        self.phase_elapsed_s += dt_used;
        match self.phase {
            BenchPhase::Warmup if self.phase_elapsed_s >= self.config.bench.sweep_warmup_time_s => {
                self.phase = BenchPhase::Sweep;
                self.phase_elapsed_s = 0.0;
                self.live_eta_thermal_indicated_pv = 0.0;
                self.dt_next_s = bench_nominal_dt_for_deg_per_step(
                    &self.config.bench,
                    target_rpm,
                    &self.config.numerics,
                    self.config.bench.sweep_integration_deg_per_step,
                );
            }
            BenchPhase::Sweep if self.phase_elapsed_s >= self.config.bench.sweep_duration_s => {
                self.phase = BenchPhase::Complete;
                self.sim = None;
                self.phase_elapsed_s = 0.0;
            }
            BenchPhase::Idle | BenchPhase::Complete | BenchPhase::Warmup | BenchPhase::Sweep => {}
        }
    }

    fn refresh_results_from_bins(&mut self) {
        self.results.clear();
        for (rpm, bin) in self.targets_rpm.iter().copied().zip(self.bins.iter()) {
            if bin.samples == 0 {
                continue;
            }
            self.results.push(BenchSample {
                target_rpm: rpm,
                rpm: bin.mean_rpm(),
                torque_brake_nm: bin.mean_torque_brake_nm(),
                load_cmd: bin.mean_load_cmd(),
                load_torque_nm: bin.mean_load_torque_nm(),
                map_kpa: bin.mean_map_kpa(),
                eta_thermal_indicated_pv: bin.mean_eta(),
                lambda_target: bin.mean_lambda_target(),
                intake_charge_temp_k: bin.mean_intake_charge_temp_k(),
                mixture_mode: self.mode,
            });
        }
    }
}

#[cfg(test)]
pub(crate) fn measure_wot_bench_sample(
    config: &AppConfig,
    control_seed: &ControlInput,
    rpm: f64,
    mode: BenchMixtureMode,
) -> BenchSample {
    let (mut sim, target_rpm) = seed_bench_simulator(config, control_seed, mode, rpm);
    let mut dyno = BenchDynoController::new(config.bench.dyno.initial_load_cmd);

    let mut dt_next_s = bench_nominal_dt(&config.bench, target_rpm, &config.numerics);
    let mut settle_elapsed_s = 0.0;
    while settle_elapsed_s < config.bench.settle_time_s {
        let (_, dt_used) = bench_adaptive_step(
            &mut sim,
            config,
            &mut dyno,
            target_rpm,
            config.bench.settle_time_s - settle_elapsed_s,
            &mut dt_next_s,
        );
        settle_elapsed_s += dt_used;
    }

    let mut average = BenchAccumulator::default();
    let mut average_elapsed_s = 0.0;
    while average_elapsed_s < config.bench.average_time_s {
        let (obs, dt_used) = bench_adaptive_step(
            &mut sim,
            config,
            &mut dyno,
            target_rpm,
            config.bench.average_time_s - average_elapsed_s,
            &mut dt_next_s,
        );
        average.accumulate(&obs, sim.state);
        average_elapsed_s += dt_used;
    }

    let mean_omega = rpm_to_rad_s(average.mean_rpm());
    let state_locked = average.locked_state(mean_omega, sim.state.throttle_eff);
    let avg = sim.locked_cycle_average(state_locked, config.bench.locked_cycle_samples);

    BenchSample {
        target_rpm,
        rpm: average.mean_rpm(),
        torque_brake_nm: avg.torque_net_nm + avg.torque_load_nm,
        load_cmd: average.mean_load_cmd(),
        load_torque_nm: average.mean_load_torque_nm(),
        map_kpa: average.mean_map_kpa(),
        eta_thermal_indicated_pv: average.mean_eta(),
        lambda_target: average.mean_lambda_target(),
        intake_charge_temp_k: average.mean_intake_charge_temp_k(),
        mixture_mode: mode,
    }
}

pub(crate) fn quick_wot_bench_preview_curve(
    config: &AppConfig,
    control_seed: &ControlInput,
    mode: BenchMixtureMode,
) -> Vec<BenchSample> {
    // The preview is intentionally coarse: it should draw a plausible dyno curve
    // almost immediately, then the budgeted bench sweep refines the result in-place.
    let preview_samples = (config.bench.locked_cycle_samples / 4).clamp(24, 64);
    bench_rpm_targets(&config.bench)
        .into_iter()
        .map(|rpm| {
            let (mut sim, target_rpm) = seed_bench_simulator(config, control_seed, mode, rpm);
            sim.control.load_cmd = 0.0;
            let avg = sim.locked_cycle_average(sim.state, preview_samples);
            let eval = sim.eval(sim.state);
            BenchSample {
                target_rpm,
                rpm: target_rpm,
                torque_brake_nm: avg.torque_net_nm,
                load_cmd: 0.0,
                load_torque_nm: 0.0,
                map_kpa: sim.state.p_intake_pa * 1.0e-3,
                eta_thermal_indicated_pv: 0.0,
                lambda_target: eval.lambda_target,
                intake_charge_temp_k: eval.intake_charge_temp_k,
                mixture_mode: mode,
            }
        })
        .collect()
}

pub(crate) fn starter_torque(rpm: f64, model: &ModelConfig) -> f64 {
    if rpm < model.starter_torque.low_rpm_threshold {
        model.starter_torque.low_torque_nm
    } else if rpm < model.starter_torque.high_rpm_threshold {
        model.starter_torque.high_torque_nm
    } else {
        0.0
    }
}

pub(crate) fn orifice_mass_flow(cd_area: f64, p_up: f64, p_down: f64, t_k: f64) -> f64 {
    if p_up <= 0.0 || cd_area <= 0.0 {
        return 0.0;
    }

    let pr = (p_down / p_up).clamp(0.0, 1.0);
    let critical = (2.0 / (GAMMA_AIR + 1.0)).powf(GAMMA_AIR / (GAMMA_AIR - 1.0));
    let coeff_choked = (GAMMA_AIR / R_AIR).sqrt()
        * (2.0 / (GAMMA_AIR + 1.0)).powf((GAMMA_AIR + 1.0) / (2.0 * (GAMMA_AIR - 1.0)));

    let mass_flow = if pr <= critical {
        cd_area * p_up / t_k.sqrt() * coeff_choked
    } else {
        let term = (2.0 * GAMMA_AIR / (R_AIR * (GAMMA_AIR - 1.0))
            * (pr.powf(2.0 / GAMMA_AIR) - pr.powf((GAMMA_AIR + 1.0) / GAMMA_AIR)))
        .max(0.0);
        cd_area * p_up / t_k.sqrt() * term.sqrt()
    };

    mass_flow.max(0.0)
}

pub(crate) fn volumetric_efficiency(
    rpm: f64,
    vvt_i: f64,
    vvt_e: f64,
    throttle_eff: f64,
    ve_model: &VolumetricEfficiencyConfig,
) -> f64 {
    let rpm_term = (ve_model.rpm_base
        + ve_model.rpm_gain * (-(rpm - ve_model.rpm_center).powi(2) / ve_model.rpm_width).exp())
    .clamp(ve_model.rpm_min, ve_model.rpm_max);
    let vvt_term = 1.0 + ve_model.vvt_intake_coeff * vvt_i - ve_model.vvt_exhaust_coeff * vvt_e;
    let throttle_term = (ve_model.throttle_base + ve_model.throttle_gain * throttle_eff.sqrt())
        .clamp(ve_model.throttle_min, ve_model.throttle_max);
    (rpm_term * vvt_term * throttle_term).clamp(ve_model.overall_min, ve_model.overall_max)
}

pub(crate) fn engine_air_consumption(
    displacement_m3: f64,
    rpm: f64,
    ve: f64,
    map_pa: f64,
    temp_k: f64,
) -> f64 {
    let cycles_per_sec = rpm / 120.0;
    let mass_per_cycle = displacement_m3 * ve * map_pa / (R_AIR * temp_k);
    (mass_per_cycle * cycles_per_sec).max(0.0)
}

pub(crate) fn trapped_air_mass(
    displacement_m3: f64,
    ve: f64,
    map_pa: f64,
    temp_k: f64,
    cylinders: usize,
) -> f64 {
    let cyl_vol = displacement_m3 / cylinders as f64;
    cyl_vol * ve * map_pa / (R_AIR * temp_k)
}

pub(crate) fn wiebe_combustion_rate(
    cycle_angle_deg: f64,
    cylinders: usize,
    start_deg: f64,
    duration_deg: f64,
    a: f64,
    m: f64,
) -> f64 {
    let mut dxb_sum = 0.0;
    let firing_interval = 720.0 / cylinders as f64;

    for c in 0..cylinders {
        let offset = c as f64 * firing_interval;
        let theta = (cycle_angle_deg - offset).rem_euclid(720.0);
        let x = (theta - start_deg) / duration_deg;
        if (0.0..=1.0).contains(&x) {
            let dxb = a * (m + 1.0) / duration_deg * x.powf(m) * (-a * x.powf(m + 1.0)).exp();
            dxb_sum += dxb;
        }
    }

    let mean_rate = cylinders as f64 / 720.0;
    if mean_rate > 0.0 {
        dxb_sum / mean_rate
    } else {
        0.0
    }
}

pub(crate) fn instantaneous_pv_sample(
    compression_ratio: f64,
    theta_rad: f64,
    p_intake_pa: f64,
    p_exhaust_pa: f64,
    p_peak_pa: f64,
    combustion_enabled: bool,
    soc_deg: f64,
    eoc_deg: f64,
    pv_model: &PvModelConfig,
) -> (f64, f64) {
    // This is a display-model cylinder trace: phase-aligned with combustion torque, but not state-solved.
    let v_min = 1.0 / (compression_ratio - 1.0);
    let v_max = v_min + 1.0;
    let gamma_c = pv_model.gamma_compression;
    let gamma_e = pv_model.gamma_expansion;
    let burn = if combustion_enabled { 1.0 } else { 0.0 };
    let comp_end = p_intake_pa * (v_max / v_min).powf(gamma_c);
    let p_peak = p_peak_pa.max(comp_end);

    let cycle_deg = theta_rad.to_degrees().rem_euclid(720.0);
    let piston_phase = theta_rad.rem_euclid(TAU);
    let vol = v_min + 0.5 * (v_max - v_min) * (1.0 - piston_phase.cos());
    let volume_at_deg = |deg: f64| -> f64 {
        let phase = deg.to_radians().rem_euclid(TAU);
        v_min + 0.5 * (v_max - v_min) * (1.0 - phase.cos())
    };
    let motored_pressure = |deg: f64, volume: f64| -> f64 {
        if deg < 180.0 {
            let x = deg / 180.0;
            p_intake_pa * (1.0 - pv_model.intake_pulsation_amplitude * (PI * x).sin().powi(2))
        } else if deg < 360.0 {
            p_intake_pa * (v_max / volume).powf(gamma_c)
        } else {
            comp_end * (v_min / volume).powf(gamma_e)
        }
    };

    let soc_deg = soc_deg;
    let eoc_deg = eoc_deg.max(soc_deg + 1.0);
    let evo_deg: f64 = pv_model.evo_deg;
    let blowdown_end_deg: f64 = pv_model.blowdown_end_deg;

    let xb = smoothstep01((cycle_deg - soc_deg) / (eoc_deg - soc_deg));
    let delta_p_peak = burn * (p_peak - comp_end).max(0.0);
    let vol_at_eoc = volume_at_deg(eoc_deg);
    let vol_at_evo = volume_at_deg(evo_deg);
    let combustion_delta = if cycle_deg < soc_deg {
        0.0
    } else if cycle_deg < eoc_deg {
        delta_p_peak * xb
    } else if cycle_deg < evo_deg {
        delta_p_peak * (vol_at_eoc / vol).powf(gamma_e)
    } else {
        0.0
    };
    let p_power_at_evo = motored_pressure(evo_deg, vol_at_evo)
        + delta_p_peak * (vol_at_eoc / vol_at_evo).powf(gamma_e);

    let p = if cycle_deg < 180.0 {
        motored_pressure(cycle_deg, vol)
    } else if cycle_deg < 360.0 {
        motored_pressure(cycle_deg, vol)
    } else if cycle_deg < evo_deg {
        (motored_pressure(cycle_deg, vol) + combustion_delta)
            .max(p_exhaust_pa * pv_model.expansion_floor_exhaust_ratio)
    } else if cycle_deg < blowdown_end_deg {
        let x = (cycle_deg - evo_deg) / (blowdown_end_deg - evo_deg);
        p_power_at_evo + (p_exhaust_pa - p_power_at_evo) * smoothstep01(x)
    } else {
        let x = (cycle_deg - blowdown_end_deg) / (720.0 - blowdown_end_deg);
        p_exhaust_pa + (p_intake_pa - p_exhaust_pa) * smoothstep01(x)
    };

    (vol, p.max(pv_model.pressure_floor_pa))
}

fn smoothstep01(x: f64) -> f64 {
    let t = x.clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

fn wave_kernel(elapsed_s: f64, delay_s: f64, decay_time_s: f64, quarter_wave_hz: f64) -> f64 {
    if elapsed_s < delay_s || decay_time_s <= 0.0 || quarter_wave_hz <= 0.0 {
        return 0.0;
    }
    let t = elapsed_s - delay_s;
    (-t / decay_time_s).exp() * (TAU * quarter_wave_hz * t).cos()
}

fn valve_window_mean_single(duration_deg: f64, model: &ModelConfig) -> f64 {
    let samples = model.cam_profile_samples.max(180);
    let mut sum = 0.0;
    for i in 0..samples {
        let deg = 720.0 * i as f64 / samples as f64;
        sum += cam_lift_mm(deg, 360.0, duration_deg, 1.0, model);
    }
    (sum / samples as f64).max(f64::EPSILON)
}

fn aggregate_valve_lift_norm(
    cycle_deg: f64,
    cylinders: usize,
    center_deg: f64,
    duration_deg: f64,
    model: &ModelConfig,
) -> f64 {
    let firing_interval = 720.0 / cylinders.max(1) as f64;
    let mut sum = 0.0;
    for c in 0..cylinders.max(1) {
        let offset = c as f64 * firing_interval;
        let theta = (cycle_deg - offset).rem_euclid(720.0);
        sum += cam_lift_mm(theta, center_deg, duration_deg, 1.0, model);
    }
    sum
}

pub(crate) fn cam_lift_mm(
    cycle_deg: f64,
    center_deg: f64,
    duration_deg: f64,
    max_lift_mm: f64,
    model: &ModelConfig,
) -> f64 {
    let half = (duration_deg * 0.5).max(model.cam_half_duration_min_deg);
    let delta = (cycle_deg - center_deg + 360.0).rem_euclid(720.0) - 360.0;
    if delta.abs() >= half {
        return 0.0;
    }
    let x = (delta / half).clamp(-1.0, 1.0);
    let smooth = 0.5 * (1.0 + (PI * x).cos());
    max_lift_mm * smooth.powf(model.cam_shape_exponent)
}

pub(crate) fn cam_profile_points(
    control: &ControlInput,
    cam: &CamConfig,
    model: &ModelConfig,
) -> (Vec<[f64; 2]>, Vec<[f64; 2]>) {
    // Sample both lobes over the full 720 deg cycle so the GUI can overlay lift and crank cursor.
    let samples = model.cam_profile_samples.max(2);
    let intake_center = cam.intake_centerline_deg - control.vvt_intake_deg;
    let exhaust_center = cam.exhaust_centerline_deg + control.vvt_exhaust_deg;
    let mut intake = Vec::with_capacity(samples);
    let mut exhaust = Vec::with_capacity(samples);

    for i in 0..samples {
        let deg = i as f64;
        intake.push([
            deg,
            cam_lift_mm(
                deg,
                intake_center,
                cam.intake_duration_deg,
                cam.intake_max_lift_mm,
                model,
            ),
        ]);
        exhaust.push([
            deg,
            cam_lift_mm(
                deg,
                exhaust_center,
                cam.exhaust_duration_deg,
                cam.exhaust_max_lift_mm,
                model,
            ),
        ]);
    }

    (intake, exhaust)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::audio::{AudioParams, render_engine_audio};

    fn wrapped_angle_error(a: f64, b: f64) -> f64 {
        let period = 4.0 * PI;
        let mut d = (a - b).rem_euclid(period);
        if d > 0.5 * period {
            d -= period;
        }
        d.abs()
    }

    fn integrate_state_open_loop_rk2(
        sim: &Simulator,
        mut state: EngineState,
        dt: f64,
        steps: usize,
    ) -> EngineState {
        for _ in 0..steps {
            state = sim.advance_state_rk2(state, dt);
        }
        state
    }

    fn integrate_state_open_loop_rk3(
        sim: &Simulator,
        mut state: EngineState,
        dt: f64,
        steps: usize,
    ) -> EngineState {
        for _ in 0..steps {
            state = sim.advance_state_rk3(state, dt);
        }
        state
    }

    fn configure_high_rpm_operating_point(sim: &mut Simulator, rpm: f64, throttle: f64) {
        sim.control.starter_cmd = false;
        sim.control.spark_cmd = true;
        sim.control.fuel_cmd = true;
        sim.control.throttle_cmd = throttle;
        sim.control.vvt_intake_deg = 0.0;
        sim.control.vvt_exhaust_deg = 0.0;

        let map_pa = sim.env.ambient_pressure_pa * (0.45 + 0.50 * throttle).clamp(0.0, 1.0);
        let load =
            (map_pa / sim.env.ambient_pressure_pa).clamp(sim.model.load_min, sim.model.load_max);
        let (mbt_deg, _, _, _, _) = sim.ignition_phase_characteristics(rpm, load, 0.0);
        sim.control.ignition_timing_deg = mbt_deg;
        sim.state = EngineState {
            omega_rad_s: rpm_to_rad_s(rpm),
            theta_rad: 410.0_f64.to_radians(),
            p_intake_pa: map_pa,
            p_intake_runner_pa: map_pa,
            p_exhaust_pa: sim.env.ambient_pressure_pa + 7_000.0,
            p_exhaust_runner_pa: sim.env.ambient_pressure_pa + 7_000.0,
            m_dot_intake_runner_kg_s: 0.0,
            m_dot_exhaust_runner_kg_s: 0.0,
            throttle_eff: throttle,
            running: true,
        };
        sim.prev_theta_rad = sim.state.theta_rad;
    }

    fn eval_operating_point(cfg: &AppConfig, rpm: f64, throttle: f64) -> EvalPoint {
        let mut sim = Simulator::new(cfg);
        configure_high_rpm_operating_point(&mut sim, rpm, throttle);
        sim.eval(sim.state)
    }

    fn internal_egr_at_state(sim: &Simulator, state: EngineState) -> f64 {
        let rpm = rad_s_to_rpm(state.omega_rad_s.max(0.0));
        let ve_base = volumetric_efficiency(
            rpm,
            sim.control.vvt_intake_deg,
            sim.control.vvt_exhaust_deg,
            state.throttle_eff,
            &sim.model.volumetric_efficiency,
        );
        let lambda_target = sim.target_lambda(state.throttle_eff);
        let base_intake_boundary_pa = sim.base_intake_cylinder_boundary_pa(state);
        let load_wave_estimate = (base_intake_boundary_pa / sim.env.ambient_pressure_pa)
            .clamp(sim.model.load_min, sim.model.load_max);
        let ignition_timing_deg = sim.ignition_timing_deg();
        let (_, _, _, _, wave_exhaust_temp_k) =
            sim.ignition_phase_characteristics(rpm, load_wave_estimate, ignition_timing_deg);
        let cycle_deg = state.theta_rad.to_degrees().rem_euclid(720.0);
        let wave_action = sim.wave_action_state(
            state,
            cycle_deg,
            rpm,
            ve_base,
            lambda_target,
            base_intake_boundary_pa,
            wave_exhaust_temp_k,
        );
        sim.internal_egr_fraction(state, wave_action)
    }

    #[derive(Debug, Clone, Copy)]
    struct FixedControlSteadyCase {
        label: &'static str,
        throttle: f64,
        load_cmd: f64,
        settle_time_s: f64,
    }

    #[derive(Debug, Clone, Copy)]
    struct PhaseSectionCrossing {
        state: EngineState,
        elapsed_s: f64,
    }

    #[derive(Debug, Clone, Copy, Default)]
    struct ObservationWindowAverage {
        rpm_sum: f64,
        eta_sum: f64,
        samples: usize,
    }

    impl ObservationWindowAverage {
        fn accumulate(&mut self, obs: &Observation) {
            self.rpm_sum += obs.rpm;
            self.eta_sum += obs.eta_thermal_indicated_pv;
            self.samples = self.samples.saturating_add(1);
        }

        fn mean_rpm(&self) -> f64 {
            self.rpm_sum / self.samples.max(1) as f64
        }

        fn mean_eta(&self) -> f64 {
            self.eta_sum / self.samples.max(1) as f64
        }
    }

    #[derive(Debug, Clone, Copy)]
    struct PeriodicSteadyMetrics {
        rpm: f64,
        cycle_time_s: f64,
        section_error_norm: f64,
        delta_rpm: f64,
        max_pressure_delta_kpa: f64,
        max_runner_flow_delta_gps: f64,
        delta_throttle: f64,
    }

    #[derive(Debug, Clone, Copy)]
    struct ProductionEngineReference {
        name: &'static str,
        displacement_l: f64,
        bore_mm: f64,
        stroke_mm: f64,
        compression_ratio: f64,
        peak_torque_nm: f64,
        peak_torque_rpm: f64,
        peak_power_kw: f64,
        peak_power_rpm: f64,
    }

    #[derive(Debug, Clone, Copy)]
    struct BenchCurveMetrics {
        torque_peak: (f64, f64, f64),
        power_peak: (f64, f64, f64),
        peak_bmep_bar: f64,
        peak_specific_power_kw_per_l: f64,
    }

    const TOYOTA_DYNAMIC_FORCE_2L: ProductionEngineReference = ProductionEngineReference {
        name: "Toyota Corolla Hatchback 2.0 Dynamic Force",
        displacement_l: 1.987,
        bore_mm: 80.5,
        stroke_mm: 97.6,
        compression_ratio: 13.0,
        peak_torque_nm: 205.0,
        peak_torque_rpm: 4_800.0,
        peak_power_kw: 126.0,
        peak_power_rpm: 6_600.0,
    };

    const MAZDA_SKYACTIV_G_20: ProductionEngineReference = ProductionEngineReference {
        name: "Mazda SKYACTIV-G 2.0",
        displacement_l: 1.997,
        bore_mm: 83.5,
        stroke_mm: 91.2,
        compression_ratio: 12.0,
        peak_torque_nm: 194.0,
        peak_torque_rpm: 4_100.0,
        peak_power_kw: 113.0,
        peak_power_rpm: 6_000.0,
    };

    const HONDA_CIVIC_20: ProductionEngineReference = ProductionEngineReference {
        name: "Honda Civic 2.0 i-VTEC",
        displacement_l: 1.996,
        bore_mm: 86.0,
        stroke_mm: 85.9,
        compression_ratio: 10.8,
        peak_torque_nm: 187.1,
        peak_torque_rpm: 4_200.0,
        peak_power_kw: 117.8,
        peak_power_rpm: 6_500.0,
    };

    const FORD_ECOSPORT_20: ProductionEngineReference = ProductionEngineReference {
        name: "Ford EcoSport 2.0 Ti-VCT",
        displacement_l: 2.0,
        bore_mm: 87.5,
        stroke_mm: 83.1,
        compression_ratio: 12.0,
        peak_torque_nm: 202.0,
        peak_torque_rpm: 4_500.0,
        peak_power_kw: 123.8,
        peak_power_rpm: 6_500.0,
    };

    const TOYOTA_86_20: ProductionEngineReference = ProductionEngineReference {
        name: "Toyota 86 2.0",
        displacement_l: 1.998,
        bore_mm: 86.0,
        stroke_mm: 86.0,
        compression_ratio: 12.5,
        peak_torque_nm: 205.0,
        peak_torque_rpm: 6_600.0,
        peak_power_kw: 147.0,
        peak_power_rpm: 7_000.0,
    };

    const PROD_GEOMETRY_DISPLACEMENT_MARGIN_L: f64 = 0.01;
    const PROD_GEOMETRY_STROKE_BORE_MARGIN: f64 = 0.02;
    const PROD_PEAK_TORQUE_LOW_RPM_MARGIN_NM: f64 = 14.0;
    const PROD_PEAK_TORQUE_MID_RPM_MARGIN_NM: f64 = 3.0;
    const PROD_PEAK_POWER_MARGIN_KW: f64 = 6.0;
    const PROD_PEAK_POWER_RPM_MARGIN: f64 = 500.0;
    const PROD_PEAK_BMEP_MARGIN_BAR: f64 = 0.8;
    const PROD_SPECIFIC_POWER_MARGIN_KW_PER_L: f64 = 2.5;

    fn high_efficiency_2l_na_refs() -> [ProductionEngineReference; 2] {
        [TOYOTA_DYNAMIC_FORCE_2L, MAZDA_SKYACTIV_G_20]
    }

    fn broader_2l_na_refs() -> [ProductionEngineReference; 4] {
        [
            TOYOTA_DYNAMIC_FORCE_2L,
            MAZDA_SKYACTIV_G_20,
            HONDA_CIVIC_20,
            FORD_ECOSPORT_20,
        ]
    }

    fn summarize_bench_curve(curve: &[(f64, f64, f64)], displacement_m3: f64) -> BenchCurveMetrics {
        let torque_peak = curve
            .iter()
            .max_by(|a, b| a.1.total_cmp(&b.1))
            .copied()
            .expect("curve should not be empty");
        let power_peak = curve
            .iter()
            .max_by(|a, b| a.2.total_cmp(&b.2))
            .copied()
            .expect("curve should not be empty");
        BenchCurveMetrics {
            torque_peak,
            power_peak,
            peak_bmep_bar: torque_to_bmep_bar(torque_peak.1, displacement_m3),
            peak_specific_power_kw_per_l: power_peak.2
                / (displacement_m3 * 1_000.0).max(f64::EPSILON),
        }
    }

    fn interpolate_engine_state(a: EngineState, b: EngineState, frac: f64) -> EngineState {
        let t = frac.clamp(0.0, 1.0);
        EngineState {
            omega_rad_s: lerp(a.omega_rad_s, b.omega_rad_s, t),
            theta_rad: wrap_cycle(lerp(a.theta_rad, b.theta_rad, t)),
            p_intake_pa: lerp(a.p_intake_pa, b.p_intake_pa, t),
            p_intake_runner_pa: lerp(a.p_intake_runner_pa, b.p_intake_runner_pa, t),
            p_exhaust_pa: lerp(a.p_exhaust_pa, b.p_exhaust_pa, t),
            p_exhaust_runner_pa: lerp(a.p_exhaust_runner_pa, b.p_exhaust_runner_pa, t),
            m_dot_intake_runner_kg_s: lerp(
                a.m_dot_intake_runner_kg_s,
                b.m_dot_intake_runner_kg_s,
                t,
            ),
            m_dot_exhaust_runner_kg_s: lerp(
                a.m_dot_exhaust_runner_kg_s,
                b.m_dot_exhaust_runner_kg_s,
                t,
            ),
            throttle_eff: lerp(a.throttle_eff, b.throttle_eff, t),
            running: if t < 0.5 { a.running } else { b.running },
        }
    }

    fn settle_fixed_control_case(
        cfg: &AppConfig,
        throttle: f64,
        load_cmd: f64,
        settle_time_s: f64,
    ) -> (Simulator, Observation) {
        let dt = cfg.environment.dt;
        let mut sim = Simulator::new(cfg);
        sim.control.starter_cmd = true;
        sim.control.spark_cmd = true;
        sim.control.fuel_cmd = true;
        sim.control.throttle_cmd = throttle;
        sim.control.load_cmd = load_cmd;

        let mut obs = sim.step(dt);
        for i in 0..((settle_time_s / dt) as usize) {
            if i > (2.2 / dt) as usize {
                sim.control.starter_cmd = false;
            }
            obs = sim.step(dt);
        }
        (sim, obs)
    }

    fn average_observation_window(
        sim: &mut Simulator,
        dt: f64,
        duration_s: f64,
    ) -> (Observation, ObservationWindowAverage) {
        let steps = (duration_s / dt.max(f64::EPSILON)).ceil() as usize;
        let mut last_obs = sim.step(dt);
        let mut average = ObservationWindowAverage::default();
        average.accumulate(&last_obs);
        for _ in 1..steps.max(1) {
            last_obs = sim.step(dt);
            average.accumulate(&last_obs);
        }
        (last_obs, average)
    }

    fn sample_fixed_control_case(
        cfg: &AppConfig,
        case: FixedControlSteadyCase,
        average_window_s: f64,
    ) -> (Simulator, Observation, ObservationWindowAverage) {
        let (mut sim, _) =
            settle_fixed_control_case(cfg, case.throttle, case.load_cmd, case.settle_time_s);
        let (last_obs, average) =
            average_observation_window(&mut sim, cfg.environment.dt, average_window_s);
        (sim, last_obs, average)
    }

    fn next_phase_section_crossing(
        sim: &Simulator,
        start_state: EngineState,
        dt: f64,
        phase_deg: f64,
        max_time_s: f64,
    ) -> PhaseSectionCrossing {
        let phase_rad = phase_deg.to_radians().rem_euclid(4.0 * PI);
        let mut prev = start_state;
        let mut elapsed_s = 0.0;
        let max_steps = (max_time_s / dt.max(f64::EPSILON)).ceil() as usize;

        for _ in 0..max_steps.max(1) {
            let next = sim.advance_state_rk3(prev, dt);
            let theta_prev = prev.theta_rad;
            let mut theta_next = next.theta_rad;
            if theta_next < theta_prev {
                theta_next += 4.0 * PI;
            }
            let mut phase_unwrapped = phase_rad;
            while phase_unwrapped <= theta_prev + 1.0e-12 {
                phase_unwrapped += 4.0 * PI;
            }
            if phase_unwrapped <= theta_next + 1.0e-12 {
                let frac =
                    (phase_unwrapped - theta_prev) / (theta_next - theta_prev).max(f64::EPSILON);
                return PhaseSectionCrossing {
                    state: interpolate_engine_state(prev, next, frac),
                    elapsed_s: elapsed_s + frac * dt,
                };
            }
            prev = next;
            elapsed_s += dt;
        }

        panic!("failed to find phase-section crossing within {max_time_s:.3} s");
    }

    fn periodic_steady_metrics(
        cfg: &AppConfig,
        sim: &Simulator,
        start_state: EngineState,
        rpm_hint: f64,
        phase_deg: f64,
    ) -> PeriodicSteadyMetrics {
        let rpm = rpm_hint.max(rad_s_to_rpm(start_state.omega_rad_s)).max(1.0);
        let dt = rpm_linked_dt(
            cfg.environment.dt,
            rpm,
            cfg.engine.idle_target_rpm,
            &cfg.numerics,
        );
        let cycle_time_guess_s = 120.0 / rpm.max(f64::EPSILON);
        let max_time_s = (cycle_time_guess_s * 1.8).max(dt * 8.0);

        let first = next_phase_section_crossing(sim, start_state, dt, phase_deg, max_time_s);
        let second = next_phase_section_crossing(sim, first.state, dt, phase_deg, max_time_s);
        let third = next_phase_section_crossing(sim, second.state, dt, phase_deg, max_time_s);

        let delta_rpm =
            (rad_s_to_rpm(third.state.omega_rad_s) - rad_s_to_rpm(second.state.omega_rad_s)).abs();
        let max_pressure_delta_kpa = [
            third.state.p_intake_pa - second.state.p_intake_pa,
            third.state.p_intake_runner_pa - second.state.p_intake_runner_pa,
            third.state.p_exhaust_pa - second.state.p_exhaust_pa,
            third.state.p_exhaust_runner_pa - second.state.p_exhaust_runner_pa,
        ]
        .into_iter()
        .map(f64::abs)
        .fold(0.0, f64::max)
            / 1_000.0;
        let max_runner_flow_delta_gps = [
            third.state.m_dot_intake_runner_kg_s - second.state.m_dot_intake_runner_kg_s,
            third.state.m_dot_exhaust_runner_kg_s - second.state.m_dot_exhaust_runner_kg_s,
        ]
        .into_iter()
        .map(f64::abs)
        .fold(0.0, f64::max)
            * 1_000.0;

        PeriodicSteadyMetrics {
            rpm,
            cycle_time_s: 0.5 * (second.elapsed_s + third.elapsed_s),
            section_error_norm: running_state_error_norm(second.state, third.state, &cfg.numerics),
            delta_rpm,
            max_pressure_delta_kpa,
            max_runner_flow_delta_gps,
            delta_throttle: (third.state.throttle_eff - second.state.throttle_eff).abs(),
        }
    }

    fn run_wot_case(
        enable_search: bool,
        ignition_deg: f64,
        vvt_intake_deg: f64,
        vvt_exhaust_deg: f64,
    ) -> (Simulator, f64) {
        let cfg = AppConfig::default();
        let dt = cfg.environment.dt;
        let mut sim = Simulator::new(&cfg);
        sim.control.ignition_timing_deg = ignition_deg;
        sim.control.vvt_intake_deg = vvt_intake_deg;
        sim.control.vvt_exhaust_deg = vvt_exhaust_deg;
        if enable_search {
            sim.set_wot_efficiency_auto_enabled(true);
        } else {
            sim.control.starter_cmd = true;
            sim.control.spark_cmd = true;
            sim.control.fuel_cmd = true;
            sim.control.throttle_cmd = 1.0;
        }

        let total_steps = (14.0 / dt) as usize;
        let average_start = (12.0 / dt) as usize;
        let mut eta_sum = 0.0;
        let mut count = 0usize;
        for i in 0..total_steps {
            if !enable_search && i > (2.0 / dt) as usize {
                sim.control.starter_cmd = false;
            }
            let obs = sim.step(dt);
            if i >= average_start {
                eta_sum += obs.eta_thermal_indicated_pv;
                count = count.saturating_add(1);
            }
        }
        (sim, eta_sum / count.max(1) as f64)
    }

    fn bench_curve_samples(
        cfg: &AppConfig,
        mode: BenchMixtureMode,
        rpms: &[f64],
        use_mbt_seed: bool,
    ) -> Vec<(f64, f64, f64)> {
        rpms.iter()
            .copied()
            .map(|rpm| {
                let mut control = ControlInput::default();
                if use_mbt_seed {
                    let sim = Simulator::new(cfg);
                    let load = cfg
                        .bench
                        .initial_map_ratio
                        .clamp(sim.model.load_min, sim.model.load_max);
                    control.ignition_timing_deg = estimate_mbt_deg(&sim.model, rpm, load);
                }
                let sample = measure_wot_bench_sample(cfg, &control, rpm, mode);
                let power_kw = shaft_power_kw(sample.rpm, sample.torque_brake_nm);
                (sample.rpm, sample.torque_brake_nm, power_kw)
            })
            .collect()
    }

    fn fast_bench_test_config() -> AppConfig {
        let mut cfg = AppConfig::default();
        // Keep the locked-RPM bench physics checks active, but shorten settling/averaging windows.
        cfg.bench.settle_time_s = 0.18;
        cfg.bench.average_time_s = 0.10;
        cfg.bench.locked_cycle_samples = 96;
        cfg.bench.integration_deg_per_step = 1.0;
        cfg.bench.integration_error_tolerance = 0.04;
        cfg.bench.integration_refine_limit = 6;
        cfg
    }

    #[test]
    fn external_load_torque_increases_with_command_and_speed() {
        let cfg = AppConfig::default();
        let load_model = &cfg.model.external_load;

        let no_load = external_load_torque_nm(0.0, rpm_to_rad_s(3_000.0), load_model);
        let part_load = external_load_torque_nm(0.4, rpm_to_rad_s(2_000.0), load_model);
        let full_load = external_load_torque_nm(1.0, rpm_to_rad_s(4_500.0), load_model);

        assert_eq!(no_load, 0.0);
        assert!(part_load > 0.0);
        assert!(full_load > part_load);
    }

    #[test]
    fn external_load_reduces_net_torque_at_same_operating_point() {
        let cfg = AppConfig::default();
        let mut sim = Simulator::new(&cfg);
        configure_high_rpm_operating_point(&mut sim, 2_800.0, 0.62);
        let sample_state = sim.state;

        sim.control.load_cmd = 0.0;
        let unloaded = sim.eval(sample_state);
        sim.control.load_cmd = 1.0;
        let loaded = sim.eval(sample_state);

        let unloaded_net = unloaded.torque_combustion_nm + unloaded.torque_starter_nm
            - unloaded.torque_friction_nm
            - unloaded.torque_pumping_nm
            - unloaded.torque_load_nm;
        let loaded_net = loaded.torque_combustion_nm + loaded.torque_starter_nm
            - loaded.torque_friction_nm
            - loaded.torque_pumping_nm
            - loaded.torque_load_nm;

        assert!(loaded.torque_load_nm > 10.0);
        assert!(loaded_net + 10.0 < unloaded_net);
    }

    #[test]
    fn bench_seed_arms_dyno_speed_hold_load() {
        let cfg = AppConfig::default();
        let mut control = ControlInput::default();
        control.load_cmd = 1.0;

        let (sim, _) =
            seed_bench_simulator(&cfg, &control, BenchMixtureMode::RichChargeCooling, 2_500.0);

        assert!(
            (sim.control.load_cmd - cfg.bench.dyno.initial_load_cmd).abs() < 1.0e-12,
            "bench seed should use the dyno initial load command"
        );
        assert!(
            (sim.model.external_load.base_torque_nm - cfg.bench.dyno.absorber_model.base_torque_nm)
                .abs()
                < 1.0e-12,
            "bench seed should swap in the dyno absorber model"
        );
    }

    #[test]
    fn rich_wot_bench_curve_resembles_modern_2l_na_shape() {
        let cfg = fast_bench_test_config();
        let rpms = [1_500.0, 3_000.0, 4_500.0, 5_500.0, 6_500.0];
        let curve = bench_curve_samples(&cfg, BenchMixtureMode::RichChargeCooling, &rpms, true);
        let metrics = summarize_bench_curve(&curve, cfg.engine.displacement_m3);

        for (rpm, torque_nm, power_kw) in &curve {
            println!(
                "rich WOT curve: rpm={rpm:.0}, torque={torque_nm:.1} Nm, power={power_kw:.1} kW"
            );
        }

        // Reference anchors:
        // - Toyota Corolla Hatchback 2.0 Dynamic Force: 205 Nm @ 4800 rpm, 126 kW @ 6600 rpm
        // - Mazda SKYACTIV-G 2.0: 194 Nm @ 4100 rpm, 113 kW @ 6000 rpm
        // - Toyota 86 2.0: 205 Nm @ 6600 rpm, 147 kW @ 7000 rpm
        //
        // Common shape across these naturally aspirated gasoline engines:
        // torque builds from low rpm into a broad mid/high plateau, while peak power arrives later.
        assert!(
            metrics.torque_peak.1 > curve[0].1 * 1.05,
            "peak torque should still sit above the low-rpm torque band for a modern 2.0L NA curve"
        );
        assert!(
            metrics.torque_peak.0 >= 2_500.0 && metrics.torque_peak.0 <= 6_500.0,
            "peak torque rpm should stay in the midrange even under the shortened bench smoke config, got {:.0} rpm",
            metrics.torque_peak.0
        );
        assert!(
            metrics.power_peak.0 >= metrics.torque_peak.0,
            "peak power should arrive no earlier than peak torque, got torque peak {:.0} rpm and power peak {:.0} rpm",
            metrics.torque_peak.0,
            metrics.power_peak.0
        );
        assert!(
            metrics.power_peak.0 >= 5_500.0,
            "peak power should sit in the upper rpm band for a sporty 2.0L NA engine, got {:.0} rpm",
            metrics.power_peak.0
        );
        assert!(
            curve[3].2 > curve[1].2 * 1.35,
            "high-rpm power should rise well above low/mid-rpm power"
        );
        assert!(
            (150.0..=220.0).contains(&metrics.torque_peak.1),
            "peak torque should stay within a broad modern 2.0L NA reference band, got {:.1} Nm",
            metrics.torque_peak.1
        );
        assert!(
            (95.0..=155.0).contains(&metrics.power_peak.2),
            "peak power should stay within a broad modern 2.0L NA reference band, got {:.1} kW",
            metrics.power_peak.2
        );
        assert!(
            metrics.power_peak.2 <= TOYOTA_86_20.peak_power_kw + 5.0,
            "default calibration should stay below the sporty {} upper envelope, got {:.1} kW",
            TOYOTA_86_20.name,
            metrics.power_peak.2
        );
        assert!(
            metrics.power_peak.0 <= TOYOTA_86_20.peak_power_rpm,
            "default calibration should peak no later than the sporty {} envelope, got {:.0} rpm",
            TOYOTA_86_20.name,
            metrics.power_peak.0
        );
    }

    #[test]
    fn default_geometry_matches_modern_high_efficiency_2l_na_family() {
        let cfg = AppConfig::default();
        let refs = high_efficiency_2l_na_refs();
        let displacement_l = cfg.engine.displacement_m3 * 1_000.0;
        let stroke_to_bore = cfg.engine.stroke_m / cfg.engine.bore_m.max(f64::EPSILON);
        let compression_min = refs
            .iter()
            .map(|r| r.compression_ratio)
            .fold(f64::INFINITY, f64::min);
        let compression_max = refs
            .iter()
            .map(|r| r.compression_ratio)
            .fold(f64::NEG_INFINITY, f64::max);
        let displacement_min = refs
            .iter()
            .map(|r| r.displacement_l)
            .fold(f64::INFINITY, f64::min);
        let displacement_max = refs
            .iter()
            .map(|r| r.displacement_l)
            .fold(f64::NEG_INFINITY, f64::max);
        let stroke_to_bore_min = refs
            .iter()
            .map(|r| r.stroke_mm / r.bore_mm)
            .fold(f64::INFINITY, f64::min);
        let stroke_to_bore_max = refs
            .iter()
            .map(|r| r.stroke_mm / r.bore_mm)
            .fold(f64::NEG_INFINITY, f64::max);

        // Reference family:
        // - Toyota Dynamic Force 2.0 / Corolla Hatchback: 80.5 x 97.6 mm, CR 13.0
        // - Mazda SKYACTIV-G 2.0: 83.5 x 91.2 mm, CR 12.0
        assert!(
            (displacement_min - PROD_GEOMETRY_DISPLACEMENT_MARGIN_L
                ..=displacement_max + PROD_GEOMETRY_DISPLACEMENT_MARGIN_L)
                .contains(&displacement_l),
            "default displacement should stay in the 2.0L class, got {:.3} L",
            displacement_l
        );
        assert!(
            (compression_min..=compression_max + 0.1).contains(&cfg.engine.compression_ratio),
            "default compression ratio should stay in a modern high-efficiency NA band, got {:.2}",
            cfg.engine.compression_ratio
        );
        assert!(
            (stroke_to_bore_min - PROD_GEOMETRY_STROKE_BORE_MARGIN
                ..=stroke_to_bore_max + PROD_GEOMETRY_STROKE_BORE_MARGIN)
                .contains(&stroke_to_bore),
            "stroke/bore should stay in the modern long-stroke 2.0L NA family, got {:.3}",
            stroke_to_bore
        );
    }

    #[test]
    fn simulator_is_fixed_to_four_cylinder_firing_spacing() {
        let cfg = AppConfig::default();
        let sim = Simulator::new(&cfg);
        assert!((sim.firing_interval_deg() - 180.0).abs() < 1.0e-12);
        assert!(
            (sim.swept_volume_per_cylinder_m3() * FIXED_CYLINDER_COUNT as f64
                - sim.params.displacement_m3)
                .abs()
                < 1.0e-12
        );
    }

    #[test]
    fn rich_wot_bench_curve_tracks_high_efficiency_2l_na_production_targets() {
        let cfg = fast_bench_test_config();
        let refs = high_efficiency_2l_na_refs();
        let rpms = [2_500.0, 4_500.0, 6_500.0];
        let curve = bench_curve_samples(&cfg, BenchMixtureMode::RichChargeCooling, &rpms, true);
        let metrics = summarize_bench_curve(&curve, cfg.engine.displacement_m3);
        let peak_torque_min = refs
            .iter()
            .map(|r| r.peak_torque_nm)
            .fold(f64::INFINITY, f64::min);
        let peak_torque_max = refs
            .iter()
            .map(|r| r.peak_torque_nm)
            .fold(f64::NEG_INFINITY, f64::max);
        let peak_power_min = refs
            .iter()
            .map(|r| r.peak_power_kw)
            .fold(f64::INFINITY, f64::min);
        let peak_power_max = refs
            .iter()
            .map(|r| r.peak_power_kw)
            .fold(f64::NEG_INFINITY, f64::max);
        let peak_power_rpm_min = refs
            .iter()
            .map(|r| r.peak_power_rpm)
            .fold(f64::INFINITY, f64::min);
        let peak_power_rpm_max = refs
            .iter()
            .map(|r| r.peak_power_rpm)
            .fold(f64::NEG_INFINITY, f64::max);

        // Official production anchors:
        // - Toyota Corolla Hatchback 2.0 Dynamic Force: 169 hp @ 6600 rpm, 151 lb-ft (205 Nm) @ 4800 rpm
        // - Mazda SKYACTIV-G 2.0: 154 PS @ 6000 rpm, 194 Nm @ 4100 rpm
        //
        // The default simulator calibration is intended to sit in this efficient-production band,
        // not at the higher-output sports-engine end represented by the Toyota 86.
        assert!(
            (peak_torque_min - PROD_PEAK_TORQUE_LOW_RPM_MARGIN_NM..=peak_torque_max)
                .contains(&curve[0].1),
            "2500 rpm torque should stay in the Corolla/Mazda 2.0L production band, got {:.1} Nm",
            curve[0].1
        );
        assert!(
            (peak_torque_min..=peak_torque_max + PROD_PEAK_TORQUE_MID_RPM_MARGIN_NM)
                .contains(&curve[1].1),
            "4500 rpm torque should sit near the production-engine peak band, got {:.1} Nm",
            curve[1].1
        );
        assert!(
            (peak_power_min + 5.0..=peak_power_max + PROD_PEAK_POWER_MARGIN_KW)
                .contains(&curve[2].2),
            "6500 rpm power should stay near the Corolla/Mazda upper production band, got {:.1} kW",
            curve[2].2
        );
        assert!(
            metrics.torque_peak.0 + 250.0 < metrics.power_peak.0,
            "peak torque should occur well before peak power, torque peak {:.0} rpm power peak {:.0} rpm",
            metrics.torque_peak.0,
            metrics.power_peak.0
        );
        assert!(
            (peak_power_rpm_min..=peak_power_rpm_max + PROD_PEAK_POWER_RPM_MARGIN)
                .contains(&metrics.power_peak.0),
            "peak power rpm should stay in the Corolla/Mazda production band, got {:.0} rpm",
            metrics.power_peak.0
        );
        assert!(
            (12.0..=12.5 + PROD_PEAK_BMEP_MARGIN_BAR).contains(&metrics.peak_bmep_bar),
            "peak brake BMEP should stay in a realistic high-efficiency 2.0L NA band, got {:.2} bar",
            metrics.peak_bmep_bar
        );
        assert!(
            (peak_power_min / TOYOTA_DYNAMIC_FORCE_2L.displacement_l
                ..=peak_power_max / MAZDA_SKYACTIV_G_20.displacement_l
                    + PROD_SPECIFIC_POWER_MARGIN_KW_PER_L)
                .contains(&metrics.peak_specific_power_kw_per_l),
            "specific power should stay in a realistic Corolla/Mazda 2.0L NA band, got {:.2} kW/L",
            metrics.peak_specific_power_kw_per_l
        );
    }

    #[test]
    fn rich_wot_bench_curve_stays_inside_broader_modern_2l_na_envelope() {
        let cfg = fast_bench_test_config();
        let refs = broader_2l_na_refs();
        let rpms = [1_500.0, 3_000.0, 4_500.0, 6_500.0];
        let curve = bench_curve_samples(&cfg, BenchMixtureMode::RichChargeCooling, &rpms, true);
        let metrics = summarize_bench_curve(&curve, cfg.engine.displacement_m3);

        let torque_min = refs
            .iter()
            .map(|r| r.peak_torque_nm)
            .fold(f64::INFINITY, f64::min);
        let torque_max = refs
            .iter()
            .map(|r| r.peak_torque_nm)
            .fold(f64::NEG_INFINITY, f64::max);
        let power_min = refs
            .iter()
            .map(|r| r.peak_power_kw)
            .fold(f64::INFINITY, f64::min);
        let power_max = refs
            .iter()
            .map(|r| r.peak_power_kw)
            .fold(f64::NEG_INFINITY, f64::max);
        let torque_rpm_min = refs
            .iter()
            .map(|r| r.peak_torque_rpm)
            .fold(f64::INFINITY, f64::min);
        let torque_rpm_max = refs
            .iter()
            .map(|r| r.peak_torque_rpm)
            .fold(f64::NEG_INFINITY, f64::max);
        let power_rpm_min = refs
            .iter()
            .map(|r| r.peak_power_rpm)
            .fold(f64::INFINITY, f64::min);
        let power_rpm_max = refs
            .iter()
            .map(|r| r.peak_power_rpm)
            .fold(f64::NEG_INFINITY, f64::max);

        assert!(
            (torque_min - 8.0..=torque_max + 8.0).contains(&metrics.torque_peak.1),
            "default peak torque should stay inside the broader 2.0L NA production envelope, got {:.1} Nm",
            metrics.torque_peak.1
        );
        assert!(
            (power_min - 2.0..=power_max + 3.0).contains(&metrics.power_peak.2),
            "default peak power should stay inside the broader 2.0L NA production envelope, got {:.1} kW",
            metrics.power_peak.2
        );
        assert!(
            (torque_rpm_min - 1_200.0..=torque_rpm_max + 200.0).contains(&metrics.torque_peak.0),
            "default peak torque rpm should stay inside the broader 2.0L NA production envelope, got {:.0} rpm",
            metrics.torque_peak.0
        );
        assert!(
            (power_rpm_min - 200.0..=power_rpm_max + 200.0).contains(&metrics.power_peak.0),
            "default peak power rpm should stay inside the broader 2.0L NA production envelope, got {:.0} rpm",
            metrics.power_peak.0
        );
        assert!(
            curve.first().is_some_and(|p| p.1 > 175.0),
            "low-rpm torque should remain healthy for a long-stroke 2.0L NA baseline"
        );
    }

    #[test]
    fn mbt_seed_improves_high_rpm_bench_power_curve() {
        let cfg = fast_bench_test_config();
        let rpms = [4_500.0, 6_500.0];
        let fixed = bench_curve_samples(&cfg, BenchMixtureMode::RichChargeCooling, &rpms, false);
        let mbt = bench_curve_samples(&cfg, BenchMixtureMode::RichChargeCooling, &rpms, true);

        assert!(
            mbt[0].2 > fixed[0].2 + 15.0,
            "MBT-like calibration should materially improve mid/high-rpm power"
        );
        assert!(
            mbt[1].2 > fixed[1].2 + 20.0,
            "MBT-like calibration should materially improve redline-near power"
        );
    }

    #[test]
    fn bench_targets_cover_requested_range() {
        let cfg = AppConfig::default();
        let targets = bench_rpm_targets(&cfg.bench);
        assert_eq!(
            targets.first().copied().unwrap_or_default(),
            cfg.bench.rpm_start_rpm
        );
        assert_eq!(
            targets.last().copied().unwrap_or_default(),
            cfg.bench.rpm_end_rpm
        );
        assert!(targets.len() > 10);
        assert!(targets.windows(2).all(|w| w[1] > w[0]));
    }

    #[test]
    fn bench_nominal_dt_is_finer_than_realtime_rule_at_high_rpm() {
        let cfg = AppConfig::default();
        let rpm = 6_000.0;
        let bench_dt = bench_nominal_dt(&cfg.bench, rpm, &cfg.numerics);
        let realtime_dt = rpm_linked_dt(
            cfg.environment.dt,
            rpm,
            cfg.engine.idle_target_rpm,
            &cfg.numerics,
        );
        assert!(bench_dt < realtime_dt);
    }

    #[test]
    fn bench_session_reports_live_progress_until_completion() {
        let mut cfg = AppConfig::default();
        cfg.bench.rpm_start_rpm = 1500.0;
        cfg.bench.rpm_end_rpm = 2000.0;
        cfg.bench.rpm_step_rpm = 500.0;
        cfg.bench.sweep_warmup_time_s = 0.01;
        cfg.bench.sweep_duration_s = 0.04;
        cfg.bench.sweep_integration_deg_per_step = 2.0;

        let mut session = BenchSession::new(
            &cfg,
            ControlInput::default(),
            BenchMixtureMode::RichChargeCooling,
        );
        let status0 = session.status();
        assert_eq!(status0.phase, BenchPhase::Warmup);
        assert_eq!(status0.total_points, 2);
        assert!(session.live_sample().is_some());

        for _ in 0..2_000 {
            session.advance_steps(1);
            if session.is_complete() {
                break;
            }
        }

        let status = session.status();
        assert!(session.is_complete());
        assert_eq!(status.phase, BenchPhase::Complete);
        assert_eq!(session.results().len(), 2);
        assert!(
            session
                .results()
                .iter()
                .all(|s| s.torque_brake_nm.is_finite())
        );
        assert!(session.results()[0].rpm < session.results()[1].rpm);
    }

    #[test]
    fn wot_bench_sample_returns_physical_values() {
        let cfg = fast_bench_test_config();
        let sim = Simulator::new(&cfg);
        let sample = measure_wot_bench_sample(
            &cfg,
            &sim.control,
            3_500.0,
            BenchMixtureMode::RichChargeCooling,
        );
        assert!((3_400.0..=3_600.0).contains(&sample.rpm));
        assert!(sample.torque_brake_nm.is_finite());
        assert!(sample.map_kpa.is_finite());
        assert!(sample.eta_thermal_indicated_pv.is_finite());
        assert_eq!(sample.mixture_mode, BenchMixtureMode::RichChargeCooling);
        assert!(
            sample.torque_brake_nm > 40.0,
            "bench sample torque too small: rpm={:.1} torque={:.2} map={:.2} eta={:.4}",
            sample.rpm,
            sample.torque_brake_nm,
            sample.map_kpa,
            sample.eta_thermal_indicated_pv
        );
        assert!((50.0..=110.0).contains(&sample.map_kpa));
        assert!(sample.eta_thermal_indicated_pv > 0.10);
    }

    #[test]
    fn wot_bench_sample_uses_positive_dyno_load_to_hold_speed() {
        let cfg = fast_bench_test_config();
        let sim = Simulator::new(&cfg);
        let sample = measure_wot_bench_sample(
            &cfg,
            &sim.control,
            3_500.0,
            BenchMixtureMode::RichChargeCooling,
        );

        assert!(
            (sample.rpm - sample.target_rpm).abs() < 160.0,
            "bench dyno should hold speed near target, target {:.0} rpm measured {:.0} rpm",
            sample.target_rpm,
            sample.rpm
        );
        assert!(
            sample.load_cmd > 0.05,
            "bench dyno should apply positive absorber command, got {:.3}",
            sample.load_cmd
        );
        assert!(
            sample.load_torque_nm > 20.0,
            "bench dyno should apply positive absorber torque, got {:.1} Nm",
            sample.load_torque_nm
        );
    }

    #[test]
    fn wot_bench_sample_is_stable_under_bench_dt_refinement() {
        let mut coarse_cfg = fast_bench_test_config();
        coarse_cfg.bench.integration_deg_per_step = 1.2;
        coarse_cfg.bench.integration_error_tolerance = 0.05;
        coarse_cfg.bench.locked_cycle_samples = 120;

        let mut fine_cfg = coarse_cfg.clone();
        fine_cfg.bench.settle_time_s = 0.24;
        fine_cfg.bench.average_time_s = 0.12;
        fine_cfg.bench.integration_deg_per_step = 0.6;
        fine_cfg.bench.integration_error_tolerance = 0.02;
        fine_cfg.bench.locked_cycle_samples = 320;

        let seed = Simulator::new(&coarse_cfg).control;
        let coarse = measure_wot_bench_sample(
            &coarse_cfg,
            &seed,
            3_500.0,
            BenchMixtureMode::RichChargeCooling,
        );
        let fine = measure_wot_bench_sample(
            &fine_cfg,
            &seed,
            3_500.0,
            BenchMixtureMode::RichChargeCooling,
        );

        assert!((coarse.torque_brake_nm - fine.torque_brake_nm).abs() < 8.0);
        assert!((coarse.map_kpa - fine.map_kpa).abs() < 2.5);
        assert!((coarse.eta_thermal_indicated_pv - fine.eta_thermal_indicated_pv).abs() < 0.03);
    }

    #[test]
    fn lambda_one_bench_runs_at_stoichiometric_target() {
        let cfg = fast_bench_test_config();
        let sim = Simulator::new(&cfg);
        let sample =
            measure_wot_bench_sample(&cfg, &sim.control, 3_500.0, BenchMixtureMode::LambdaOne);
        assert_eq!(sample.mixture_mode, BenchMixtureMode::LambdaOne);
        assert!(
            (sample.lambda_target - cfg.bench.lambda_one_target).abs() < 1.0e-6,
            "lambda-one bench should hold stoich target, got {:.4}",
            sample.lambda_target
        );
        assert!(sample.torque_brake_nm.is_finite());
    }

    #[test]
    fn rich_charge_cooling_bench_outperforms_lambda_one_at_wot() {
        let cfg = fast_bench_test_config();
        let seed = ControlInput::default();
        let lambda_one =
            measure_wot_bench_sample(&cfg, &seed, 3_500.0, BenchMixtureMode::LambdaOne);
        let rich =
            measure_wot_bench_sample(&cfg, &seed, 3_500.0, BenchMixtureMode::RichChargeCooling);
        assert!(
            rich.torque_brake_nm > lambda_one.torque_brake_nm + 4.0,
            "rich charge-cooling bench should beat lambda=1 on torque, rich={:.1} lambda1={:.1}",
            rich.torque_brake_nm,
            lambda_one.torque_brake_nm
        );
        assert!(
            rich.intake_charge_temp_k < lambda_one.intake_charge_temp_k - 1.0,
            "rich charge-cooling bench should cool the charge more, rich={:.1}K lambda1={:.1}K",
            rich.intake_charge_temp_k,
            lambda_one.intake_charge_temp_k
        );
    }

    #[test]
    fn quick_bench_preview_curve_covers_requested_range() {
        let mut cfg = AppConfig::default();
        cfg.bench.rpm_start_rpm = 1_500.0;
        cfg.bench.rpm_end_rpm = 2_500.0;
        cfg.bench.rpm_step_rpm = 500.0;
        cfg.bench.locked_cycle_samples = 96;

        let seed = ControlInput::default();
        let curve = quick_wot_bench_preview_curve(&cfg, &seed, BenchMixtureMode::RichChargeCooling);

        assert_eq!(curve.len(), 3);
        assert!((curve.first().unwrap().rpm - 1_500.0).abs() < 1.0e-9);
        assert!((curve.last().unwrap().rpm - 2_500.0).abs() < 1.0e-9);
        assert!(
            curve
                .iter()
                .all(|sample| sample.torque_brake_nm.is_finite())
        );
        assert!(curve.iter().all(|sample| sample.map_kpa.is_finite()));
    }

    #[test]
    fn wot_vvt_search_starts_from_global_min_and_scans_forward() {
        let cfg = AppConfig::default();
        let tuning = &cfg.auto_control;
        let model = &cfg.model;
        let mut auto = AutoController::default();
        auto.wot_best_point = WotSearchPoint {
            ignition_deg: 18.0,
            vvt_intake_deg: 12.0,
            vvt_exhaust_deg: 6.0,
        };

        auto.wot_axis = WotSearchAxis::IntakeVvt;
        assert!(auto.seed_candidate_from_best(tuning, model));
        assert!(
            (auto.wot_candidate_point.vvt_intake_deg - tuning.wot_search_vvt_min_deg).abs()
                < 1.0e-9
        );
        assert!(
            (auto.wot_candidate_point.vvt_exhaust_deg - auto.wot_best_point.vvt_exhaust_deg).abs()
                < 1.0e-9
        );
        assert!(auto.advance_vvt_scan_candidate(tuning, model));
        assert!(
            (auto.wot_candidate_point.vvt_intake_deg
                - (tuning.wot_search_vvt_min_deg + tuning.wot_search_vvt_intake_step_deg.abs()))
            .abs()
                < 1.0e-9
        );

        auto.wot_axis = WotSearchAxis::ExhaustVvt;
        assert!(auto.seed_candidate_from_best(tuning, model));
        assert!(
            (auto.wot_candidate_point.vvt_exhaust_deg - tuning.wot_search_vvt_min_deg).abs()
                < 1.0e-9
        );
        assert!(
            (auto.wot_candidate_point.vvt_intake_deg - auto.wot_best_point.vvt_intake_deg).abs()
                < 1.0e-9
        );
        assert!(auto.advance_vvt_scan_candidate(tuning, model));
        assert!(
            (auto.wot_candidate_point.vvt_exhaust_deg
                - (tuning.wot_search_vvt_min_deg + tuning.wot_search_vvt_exhaust_step_deg.abs()))
            .abs()
                < 1.0e-9
        );
    }

    #[test]
    fn auto_mode_reaches_idle_from_stop() {
        let cfg = AppConfig::default();
        let dt = cfg.environment.dt;
        let mut sim = Simulator::new(&cfg);
        sim.auto.enabled = true;
        let mut obs = sim.step(dt);
        for _ in 0..((14.0 / dt) as usize) {
            obs = sim.step(dt);
        }

        assert!(
            sim.state.running,
            "engine should be running in auto mode, rpm={:.1}, map={:.1}, intake_runner={:.1}, exhaust_runner={:.1}",
            obs.rpm, obs.map_kpa, obs.intake_runner_kpa, obs.exhaust_runner_kpa
        );
        assert!(
            (700.0..=1050.0).contains(&obs.rpm),
            "unexpected final rpm: {:.1}",
            obs.rpm
        );
        assert!(
            (25.0..=85.0).contains(&obs.map_kpa),
            "map out of expected idle range: {:.1} kPa",
            obs.map_kpa
        );
    }

    #[test]
    fn wot_efficiency_auto_mode_improves_poor_wot_calibration() {
        let poor_ignition_deg = 0.0;
        let poor_vvt_intake_deg = -28.0;
        let poor_vvt_exhaust_deg = 28.0;
        let (_manual_sim, manual_eta) = run_wot_case(
            false,
            poor_ignition_deg,
            poor_vvt_intake_deg,
            poor_vvt_exhaust_deg,
        );
        let (auto_sim, auto_eta) = run_wot_case(
            true,
            poor_ignition_deg,
            poor_vvt_intake_deg,
            poor_vvt_exhaust_deg,
        );

        let rpm = rad_s_to_rpm(auto_sim.state.omega_rad_s.max(0.0));
        let load = (auto_sim.state.p_intake_pa / auto_sim.env.ambient_pressure_pa)
            .clamp(auto_sim.model.load_min, auto_sim.model.load_max);
        let mbt_deg = estimate_mbt_deg(&auto_sim.model, rpm, load);

        assert!(
            auto_eta > manual_eta + 0.03,
            "WOT search should improve indicated efficiency, manual={manual_eta:.3}, auto={auto_eta:.3}"
        );
        assert!(
            auto_sim.auto.wot_best_point.ignition_deg > poor_ignition_deg + 8.0,
            "best ignition should move materially away from the poor initial seed, best={:.1} deg mbt={mbt_deg:.1} deg",
            auto_sim.auto.wot_best_point.ignition_deg
        );
        assert!(
            (auto_sim.control.throttle_cmd - 1.0).abs() < 1.0e-12,
            "WOT search should force full throttle"
        );
        assert!(auto_sim.control.spark_cmd && auto_sim.control.fuel_cmd);
        assert!(auto_sim.auto.wot_best_eta > 0.0);
    }

    #[test]
    fn states_remain_physical_under_start_sequence() {
        let cfg = AppConfig::default();
        let dt = cfg.environment.dt;
        let mut sim = Simulator::new(&cfg);
        sim.control.starter_cmd = true;
        sim.control.spark_cmd = true;
        sim.control.fuel_cmd = true;
        sim.control.throttle_cmd = 0.22;

        for i in 0..((8.0 / dt) as usize) {
            let obs = sim.step(dt);
            if i > (2.0 / dt) as usize {
                sim.control.starter_cmd = false;
                sim.control.throttle_cmd = 0.10;
            }
            assert!(obs.rpm.is_finite());
            assert!(obs.map_kpa.is_finite());
            assert!(obs.exhaust_kpa.is_finite());
            assert!(sim.state.p_intake_pa > 15_000.0);
            assert!(sim.state.p_intake_runner_pa > 15_000.0);
            assert!(sim.state.p_exhaust_pa > 70_000.0);
            assert!(sim.state.p_exhaust_runner_pa > 30_000.0);
            assert!(sim.state.omega_rad_s >= 0.0);
            assert!(
                sim.state.m_dot_intake_runner_kg_s.abs()
                    <= sim.model.gas_path.runner_flow_limit_kg_s + 1.0e-9
            );
            assert!(
                sim.state.m_dot_exhaust_runner_kg_s.abs()
                    <= sim.model.gas_path.runner_flow_limit_kg_s + 1.0e-9
            );
        }
    }

    #[test]
    fn valve_pulse_factors_average_to_unity_over_cycle() {
        let cfg = AppConfig::default();
        let sim = Simulator::new(&cfg);
        let samples = 1440usize;
        let mut intake_sum = 0.0;
        let mut exhaust_sum = 0.0;
        for i in 0..samples {
            let cycle_deg = 720.0 * i as f64 / samples as f64;
            let (intake, exhaust) = sim.valve_pulse_factors(cycle_deg);
            intake_sum += intake;
            exhaust_sum += exhaust;
        }
        let intake_mean = intake_sum / samples as f64;
        let exhaust_mean = exhaust_sum / samples as f64;
        assert!(
            (intake_mean - 1.0).abs() < 0.02,
            "intake pulse factor mean should be ~1, got {intake_mean:.4}"
        );
        assert!(
            (exhaust_mean - 1.0).abs() < 0.02,
            "exhaust pulse factor mean should be ~1, got {exhaust_mean:.4}"
        );
    }

    #[test]
    fn overlap_backpressure_reduces_effective_volumetric_efficiency() {
        let cfg = AppConfig::default();
        let mut sim = Simulator::new(&cfg);
        sim.control.throttle_cmd = 1.0;
        sim.control.spark_cmd = true;
        sim.control.fuel_cmd = true;
        sim.control.vvt_intake_deg = 32.0;
        sim.control.vvt_exhaust_deg = 32.0;

        let scavenging_state = EngineState {
            omega_rad_s: rpm_to_rad_s(3_000.0),
            theta_rad: 380.0_f64.to_radians(),
            p_intake_pa: 96_000.0,
            p_intake_runner_pa: 97_000.0,
            p_exhaust_pa: 100_000.0,
            p_exhaust_runner_pa: 92_000.0,
            m_dot_intake_runner_kg_s: 0.02,
            m_dot_exhaust_runner_kg_s: 0.04,
            throttle_eff: 1.0,
            running: true,
        };
        let backflow_state = EngineState {
            p_exhaust_pa: 122_000.0,
            p_exhaust_runner_pa: 150_000.0,
            m_dot_exhaust_runner_kg_s: -0.05,
            ..scavenging_state
        };

        let scavenging_eval = sim.eval(scavenging_state);
        let backflow_eval = sim.eval(backflow_state);
        assert!(
            scavenging_eval.ve_effective > backflow_eval.ve_effective + 0.015,
            "backpressure during overlap should penalize VE, scavenging={:.3} backflow={:.3}",
            scavenging_eval.ve_effective,
            backflow_eval.ve_effective
        );
    }

    #[test]
    fn burned_gas_cp_rises_with_temperature_and_lowers_gamma() {
        let sim = Simulator::new(&AppConfig::default());
        let cool_cp = sim.burned_gas_cp_j_per_kgk(800.0, 0.02);
        let hot_cp = sim.burned_gas_cp_j_per_kgk(1_150.0, 0.14);
        let cool_gamma = Simulator::gas_gamma_from_cp_j_per_kgk(cool_cp);
        let hot_gamma = Simulator::gas_gamma_from_cp_j_per_kgk(hot_cp);

        assert!(
            hot_cp > cool_cp + 40.0,
            "burned-gas cp should rise with temperature / dilution, cool={cool_cp:.1} hot={hot_cp:.1}"
        );
        assert!(
            hot_gamma < cool_gamma - 0.01,
            "higher burned-gas cp should lower gamma, cool={cool_gamma:.4} hot={hot_gamma:.4}"
        );
    }

    #[test]
    fn overlap_backflow_increases_internal_egr_fraction() {
        let mut cfg = AppConfig::default();
        cfg.model.internal_egr.pressure_backflow_gain = 0.18;
        cfg.model.internal_egr.wave_backflow_gain = 0.20;
        cfg.model.internal_egr.reverse_flow_gain = 0.24;
        cfg.model.internal_egr.fraction_max = 0.24;

        let mut sim = Simulator::new(&cfg);
        sim.control.throttle_cmd = 1.0;
        sim.control.spark_cmd = true;
        sim.control.fuel_cmd = true;
        sim.control.vvt_intake_deg = 32.0;
        sim.control.vvt_exhaust_deg = 32.0;

        let scavenging_state = EngineState {
            omega_rad_s: rpm_to_rad_s(3_000.0),
            theta_rad: 380.0_f64.to_radians(),
            p_intake_pa: 96_000.0,
            p_intake_runner_pa: 97_000.0,
            p_exhaust_pa: 100_000.0,
            p_exhaust_runner_pa: 92_000.0,
            m_dot_intake_runner_kg_s: 0.02,
            m_dot_exhaust_runner_kg_s: 0.04,
            throttle_eff: 1.0,
            running: true,
        };
        let backflow_state = EngineState {
            p_exhaust_pa: 122_000.0,
            p_exhaust_runner_pa: 150_000.0,
            m_dot_exhaust_runner_kg_s: -0.05,
            ..scavenging_state
        };

        let scavenging_egr = internal_egr_at_state(&sim, scavenging_state);
        let backflow_egr = internal_egr_at_state(&sim, backflow_state);

        assert!(
            backflow_egr > scavenging_egr + 0.01,
            "overlap backflow should increase internal EGR, scavenging={scavenging_egr:.4} backflow={backflow_egr:.4}"
        );
    }

    #[test]
    fn internal_egr_heats_charge_and_stretches_burn() {
        let mut cfg = AppConfig::default();
        cfg.model.internal_egr.pressure_backflow_gain = 0.18;
        cfg.model.internal_egr.wave_backflow_gain = 0.22;
        cfg.model.internal_egr.reverse_flow_gain = 0.28;
        cfg.model.internal_egr.burn_duration_gain_deg_per_fraction = 40.0;
        cfg.model.internal_egr.phase_dilution_gain = 0.45;
        cfg.model.internal_egr.fraction_max = 0.26;

        let mut sim = Simulator::new(&cfg);
        sim.control.throttle_cmd = 1.0;
        sim.control.spark_cmd = true;
        sim.control.fuel_cmd = true;
        sim.control.vvt_intake_deg = 32.0;
        sim.control.vvt_exhaust_deg = 32.0;

        let scavenging_state = EngineState {
            omega_rad_s: rpm_to_rad_s(3_000.0),
            theta_rad: 380.0_f64.to_radians(),
            p_intake_pa: 96_000.0,
            p_intake_runner_pa: 97_000.0,
            p_exhaust_pa: 100_000.0,
            p_exhaust_runner_pa: 92_000.0,
            m_dot_intake_runner_kg_s: 0.02,
            m_dot_exhaust_runner_kg_s: 0.04,
            throttle_eff: 1.0,
            running: true,
        };
        let backflow_state = EngineState {
            p_exhaust_pa: 122_000.0,
            p_exhaust_runner_pa: 150_000.0,
            m_dot_exhaust_runner_kg_s: -0.05,
            ..scavenging_state
        };

        let scavenging_eval = sim.eval(scavenging_state);
        let backflow_eval = sim.eval(backflow_state);

        assert!(
            backflow_eval.intake_charge_temp_k > scavenging_eval.intake_charge_temp_k + 5.0,
            "internal EGR should heat the trapped charge, scavenging={:.1} K backflow={:.1} K",
            scavenging_eval.intake_charge_temp_k,
            backflow_eval.intake_charge_temp_k
        );
        assert!(
            backflow_eval.burn_duration_deg > scavenging_eval.burn_duration_deg + 0.5,
            "internal EGR should stretch burn duration, scavenging={:.2} deg backflow={:.2} deg",
            scavenging_eval.burn_duration_deg,
            backflow_eval.burn_duration_deg
        );
        assert!(
            backflow_eval.torque_combustion_nm < scavenging_eval.torque_combustion_nm,
            "dilution should soften combustion torque, scavenging={:.2} Nm backflow={:.2} Nm",
            scavenging_eval.torque_combustion_nm,
            backflow_eval.torque_combustion_nm
        );
    }

    #[test]
    fn geometry_scaled_runner_inertance_tracks_length_and_diameter() {
        let base = 18_000.0;
        let longer = Simulator::geometry_scaled_inertance(base, 0.54, 0.034, 0.36, 0.034);
        let narrower = Simulator::geometry_scaled_inertance(base, 0.36, 0.028, 0.36, 0.034);
        let wider = Simulator::geometry_scaled_inertance(base, 0.36, 0.040, 0.36, 0.034);

        assert!(longer > base, "longer runner should increase inertance");
        assert!(narrower > base, "narrower runner should increase inertance");
        assert!(wider < base, "wider runner should reduce inertance");
    }

    #[test]
    fn runner_pressure_loss_opposes_flow_and_grows_with_flow() {
        let low_forward = Simulator::runner_pressure_loss_pa(
            0.03,
            101_000.0,
            96_000.0,
            320.0,
            0.36,
            0.034,
            FIXED_CYLINDER_COUNT,
            0.028,
            1.6,
        );
        let high_forward = Simulator::runner_pressure_loss_pa(
            0.06,
            101_000.0,
            96_000.0,
            320.0,
            0.36,
            0.034,
            FIXED_CYLINDER_COUNT,
            0.028,
            1.6,
        );
        let reverse = Simulator::runner_pressure_loss_pa(
            -0.03,
            101_000.0,
            96_000.0,
            320.0,
            0.36,
            0.034,
            FIXED_CYLINDER_COUNT,
            0.028,
            1.6,
        );

        assert!(
            low_forward > 0.0,
            "forward flow loss should oppose positive flow"
        );
        assert!(
            reverse < 0.0,
            "reverse flow loss should oppose negative flow"
        );
        assert!(
            high_forward.abs() > low_forward.abs() * 3.0,
            "quadratic loss should grow strongly with flow, low={low_forward:.2}, high={high_forward:.2}"
        );
    }

    #[test]
    fn helmholtz_frequency_drops_with_longer_runner() {
        let short = Simulator::helmholtz_frequency_hz(0.0032, 0.30, 0.034, 305.0, 1);
        let long = Simulator::helmholtz_frequency_hz(0.0032, 0.60, 0.034, 305.0, 1);

        assert!(
            long < short,
            "longer runner should lower Helmholtz frequency, short={short:.1} Hz long={long:.1} Hz"
        );
    }

    #[test]
    fn wave_action_zero_gains_collapse_to_unity_multipliers() {
        let mut cfg = AppConfig::default();
        cfg.model.wave_action.intake_pressure_gain = 0.0;
        cfg.model.wave_action.exhaust_pressure_gain = 0.0;
        cfg.model.wave_action.intake_flow_wave_gain = 0.0;
        cfg.model.wave_action.exhaust_flow_wave_gain = 0.0;
        cfg.model.wave_action.intake_ram_gain = 0.0;
        cfg.model.wave_action.exhaust_scavenge_gain = 0.0;

        let eval = eval_operating_point(&cfg, 3_500.0, 1.0);
        assert!(
            eval.intake_wave_current_pa.abs() < 1.0e-9,
            "intake wave should vanish when wave gains are zero, got {:.6} Pa",
            eval.intake_wave_current_pa
        );
        assert!(
            eval.exhaust_wave_current_pa.abs() < 1.0e-9,
            "exhaust wave should vanish when wave gains are zero, got {:.6} Pa",
            eval.exhaust_wave_current_pa
        );
        assert!((eval.intake_ram_multiplier - 1.0).abs() < 1.0e-12);
        assert!((eval.exhaust_scavenge_multiplier - 1.0).abs() < 1.0e-12);
        assert!((eval.ve_pulse_multiplier - 1.0).abs() < 1.0e-12);
    }

    #[test]
    fn longer_intake_runner_shifts_ram_tuning_to_lower_rpm() {
        let mut short_cfg = AppConfig::default();
        short_cfg.model.wave_action.intake_runner_length_m = 0.34;
        short_cfg.model.wave_action.intake_pressure_gain = 0.34;
        short_cfg.model.wave_action.intake_ram_gain = 1.10;
        short_cfg.model.wave_action.exhaust_pressure_gain = 0.0;
        short_cfg.model.wave_action.exhaust_scavenge_gain = 0.0;
        short_cfg.model.wave_action.intake_group_count = 4;
        short_cfg.model.wave_action.event_memory = 4;
        let mut long_cfg = short_cfg.clone();
        long_cfg.model.wave_action.intake_runner_length_m = 0.62;

        let rpm_grid = [
            1_800.0, 2_600.0, 3_400.0, 4_200.0, 5_000.0, 5_800.0, 6_600.0,
        ];
        let short_peak = rpm_grid
            .into_iter()
            .map(|rpm| {
                (
                    rpm,
                    eval_operating_point(&short_cfg, rpm, 1.0).intake_ram_multiplier,
                )
            })
            .max_by(|a, b| a.1.total_cmp(&b.1))
            .unwrap();
        let long_peak = rpm_grid
            .into_iter()
            .map(|rpm| {
                (
                    rpm,
                    eval_operating_point(&long_cfg, rpm, 1.0).intake_ram_multiplier,
                )
            })
            .max_by(|a, b| a.1.total_cmp(&b.1))
            .unwrap();

        assert!(
            long_peak.0 + 400.0 <= short_peak.0,
            "long intake runner should shift ram tuning lower in rpm, short peak={:.0} rpm ({:.3}) long peak={:.0} rpm ({:.3})",
            short_peak.0,
            short_peak.1,
            long_peak.0,
            long_peak.1
        );
    }

    #[test]
    fn exhaust_grouping_changes_scavenging_response() {
        let mut paired_cfg = AppConfig::default();
        paired_cfg.model.wave_action.exhaust_group_count = 2;
        paired_cfg.model.wave_action.exhaust_primary_length_m = 0.82;
        paired_cfg.model.wave_action.exhaust_pressure_gain = 0.40;
        paired_cfg.model.wave_action.exhaust_scavenge_gain = 0.85;
        paired_cfg.model.wave_action.intake_pressure_gain = 0.12;
        paired_cfg.model.wave_action.intake_ram_gain = 0.25;
        paired_cfg.model.wave_action.event_memory = 4;
        let mut merged_cfg = paired_cfg.clone();
        merged_cfg.model.wave_action.exhaust_group_count = 1;

        let paired = eval_operating_point(&paired_cfg, 4_200.0, 1.0);
        let merged = eval_operating_point(&merged_cfg, 4_200.0, 1.0);

        assert!(
            (paired.exhaust_scavenge_multiplier - merged.exhaust_scavenge_multiplier).abs() > 0.01,
            "changing exhaust grouping should materially change scavenging, paired={:.3} merged={:.3}",
            paired.exhaust_scavenge_multiplier,
            merged.exhaust_scavenge_multiplier
        );
        assert!(
            (paired.ve_pulse_multiplier - merged.ve_pulse_multiplier).abs() > 0.01,
            "changing exhaust grouping should materially change VE pulse multiplier, paired={:.3} merged={:.3}",
            paired.ve_pulse_multiplier,
            merged.ve_pulse_multiplier
        );
    }

    #[test]
    fn cylinder_heat_loss_reduces_wot_efficiency_and_torque() {
        let mut cfg_cold = AppConfig::default();
        cfg_cold.model.heat_transfer.base_h_w_m2k = 0.0;
        let mut cfg_hot = cfg_cold.clone();
        cfg_hot.model.heat_transfer.base_h_w_m2k = 1_200.0;
        cfg_hot.model.heat_transfer.heat_loss_fraction_max = 0.20;

        let rpm = 3_500.0;
        let mut sim_cold = Simulator::new(&cfg_cold);
        configure_high_rpm_operating_point(&mut sim_cold, rpm, 1.0);
        let mut sim_hot = Simulator::new(&cfg_hot);
        configure_high_rpm_operating_point(&mut sim_hot, rpm, 1.0);

        let samples = 720usize;
        let mut cold_heat_loss_sum = 0.0;
        let mut hot_heat_loss_sum = 0.0;
        let mut cold_torque_sum = 0.0;
        let mut hot_torque_sum = 0.0;
        for i in 0..samples {
            let theta = (720.0 * i as f64 / samples as f64).to_radians();
            let cold = sim_cold.eval(EngineState {
                theta_rad: theta,
                ..sim_cold.state
            });
            let hot = sim_hot.eval(EngineState {
                theta_rad: theta,
                ..sim_hot.state
            });
            cold_heat_loss_sum += cold.heat_loss_cycle_j;
            hot_heat_loss_sum += hot.heat_loss_cycle_j;
            cold_torque_sum += cold.torque_combustion_nm;
            hot_torque_sum += hot.torque_combustion_nm;
        }
        let inv_samples = 1.0 / samples as f64;
        let cold_heat_loss = cold_heat_loss_sum * inv_samples;
        let hot_heat_loss = hot_heat_loss_sum * inv_samples;
        let cold_torque = cold_torque_sum * inv_samples;
        let hot_torque = hot_torque_sum * inv_samples;

        assert!(
            hot_heat_loss > cold_heat_loss + 10.0,
            "stronger wall heat transfer should increase per-cycle heat loss, cold={:.1} J hot={:.1} J",
            cold_heat_loss,
            hot_heat_loss
        );
        assert!(
            hot_torque + 5.0 < cold_torque,
            "heat loss should reduce combustion torque, cold={:.1} hot={:.1}",
            cold_torque,
            hot_torque
        );
    }

    #[test]
    fn runner_pulsation_states_remain_bounded_and_nontrivial_at_high_rpm() {
        let cfg = AppConfig::default();
        let dt = rpm_linked_dt(
            cfg.environment.dt,
            6_500.0,
            cfg.engine.idle_target_rpm,
            &cfg.numerics,
        );
        let mut sim = Simulator::new(&cfg);
        configure_high_rpm_operating_point(&mut sim, 6_500.0, 1.0);

        let mut peak_intake_dp = 0.0_f64;
        let mut peak_exhaust_dp = 0.0_f64;
        for _ in 0..2_400 {
            let obs = sim.step(dt);
            peak_intake_dp =
                peak_intake_dp.max((sim.state.p_intake_pa - sim.state.p_intake_runner_pa).abs());
            peak_exhaust_dp =
                peak_exhaust_dp.max((sim.state.p_exhaust_runner_pa - sim.state.p_exhaust_pa).abs());
            assert!(obs.rpm.is_finite());
            assert!(sim.state.p_intake_runner_pa.is_finite());
            assert!(sim.state.p_exhaust_runner_pa.is_finite());
            assert!(sim.state.m_dot_intake_runner_kg_s.is_finite());
            assert!(sim.state.m_dot_exhaust_runner_kg_s.is_finite());
            assert!(
                sim.state.m_dot_intake_runner_kg_s.abs()
                    <= sim.model.gas_path.runner_flow_limit_kg_s + 1.0e-9
            );
            assert!(
                sim.state.m_dot_exhaust_runner_kg_s.abs()
                    <= sim.model.gas_path.runner_flow_limit_kg_s + 1.0e-9
            );
        }

        assert!(
            peak_intake_dp > 200.0,
            "intake runner model should develop nontrivial plenum-runner pressure differential, got {peak_intake_dp:.1} Pa"
        );
        assert!(
            peak_exhaust_dp > 400.0,
            "exhaust runner model should develop nontrivial runner-collector pressure differential, got {peak_exhaust_dp:.1} Pa"
        );
    }

    #[test]
    fn rk2_matches_exact_throttle_lag_solution_at_high_rpm() {
        let mut cfg = AppConfig::default();
        cfg.engine.bore_m = 0.0;
        cfg.engine.stroke_m = 0.0;
        cfg.engine.friction_c0_nm = 0.0;
        cfg.engine.friction_c1_nms = 0.0;
        cfg.engine.friction_c2_nms2 = 0.0;
        let mut sim = Simulator::new(&cfg);
        sim.control.starter_cmd = false;
        sim.control.spark_cmd = false;
        sim.control.fuel_cmd = false;
        sim.control.throttle_cmd = 0.82;

        let rpm = 6_500.0;
        let dt = rpm_linked_dt(
            cfg.environment.dt,
            rpm,
            cfg.engine.idle_target_rpm,
            &cfg.numerics,
        );
        let steps = 160usize;
        let elapsed = dt * steps as f64;
        let tau = cfg.model.throttle_time_constant_s;
        let alpha0 = 0.07;
        let state0 = EngineState {
            omega_rad_s: rpm_to_rad_s(rpm),
            theta_rad: 1.3,
            p_intake_pa: cfg.environment.ambient_pressure_pa,
            p_intake_runner_pa: cfg.environment.ambient_pressure_pa,
            p_exhaust_pa: cfg.environment.ambient_pressure_pa,
            p_exhaust_runner_pa: cfg.environment.ambient_pressure_pa,
            m_dot_intake_runner_kg_s: 0.0,
            m_dot_exhaust_runner_kg_s: 0.0,
            throttle_eff: alpha0,
            running: true,
        };

        let exact_alpha =
            sim.control.throttle_cmd + (alpha0 - sim.control.throttle_cmd) * (-elapsed / tau).exp();
        let exact_theta = wrap_cycle(state0.theta_rad + state0.omega_rad_s * elapsed);
        let final_state = integrate_state_open_loop_rk2(&sim, state0, dt, steps);

        assert!(
            (final_state.throttle_eff - exact_alpha).abs() < 2.0e-5,
            "throttle RK2 error too large at high rpm, exact={exact_alpha:.6}, got={:.6}",
            final_state.throttle_eff
        );
        assert!(
            (final_state.omega_rad_s - state0.omega_rad_s).abs() < 1.0e-12,
            "omega should remain constant when all torques are zero"
        );
        assert!(
            wrapped_angle_error(final_state.theta_rad, exact_theta) < 1.0e-10,
            "theta integration should remain exact for constant omega"
        );
    }

    #[test]
    fn rk2_matches_exact_linear_spin_decay_at_high_rpm() {
        let mut cfg = AppConfig::default();
        cfg.engine.bore_m = 0.0;
        cfg.engine.stroke_m = 0.0;
        cfg.engine.friction_c0_nm = 0.0;
        cfg.engine.friction_c1_nms = 0.012;
        cfg.engine.friction_c2_nms2 = 0.0;
        let mut sim = Simulator::new(&cfg);
        sim.control.starter_cmd = false;
        sim.control.spark_cmd = false;
        sim.control.fuel_cmd = false;
        sim.control.throttle_cmd = 0.22;

        let rpm = 6_700.0;
        let dt = rpm_linked_dt(
            cfg.environment.dt,
            rpm,
            cfg.engine.idle_target_rpm,
            &cfg.numerics,
        );
        let steps = 96usize;
        let elapsed = dt * steps as f64;
        let omega0 = rpm_to_rad_s(rpm);
        let theta0 = 0.9;
        let a = cfg.engine.friction_c1_nms / cfg.engine.inertia_kgm2;
        let state0 = EngineState {
            omega_rad_s: omega0,
            theta_rad: theta0,
            p_intake_pa: cfg.environment.ambient_pressure_pa,
            p_intake_runner_pa: cfg.environment.ambient_pressure_pa,
            p_exhaust_pa: cfg.environment.ambient_pressure_pa,
            p_exhaust_runner_pa: cfg.environment.ambient_pressure_pa,
            m_dot_intake_runner_kg_s: 0.0,
            m_dot_exhaust_runner_kg_s: 0.0,
            throttle_eff: sim.control.throttle_cmd,
            running: true,
        };

        let exact_omega = omega0 * (-a * elapsed).exp();
        let exact_theta = wrap_cycle(theta0 + omega0 * (1.0 - (-a * elapsed).exp()) / a);
        let final_state = integrate_state_open_loop_rk2(&sim, state0, dt, steps);

        let rel_omega_err = ((final_state.omega_rad_s - exact_omega) / exact_omega).abs();
        assert!(
            rel_omega_err < 8.0e-6,
            "omega RK2 relative error too large at high rpm, exact={exact_omega:.6}, got={:.6}, rel={rel_omega_err:.3e}",
            final_state.omega_rad_s
        );
        assert!(
            wrapped_angle_error(final_state.theta_rad, exact_theta) < 8.0e-5,
            "theta RK2 absolute error too large, exact={exact_theta:.6}, got={:.6}",
            final_state.theta_rad
        );
    }

    #[test]
    fn rk3_matches_exact_throttle_lag_solution_at_high_rpm() {
        let mut cfg = AppConfig::default();
        cfg.engine.bore_m = 0.0;
        cfg.engine.stroke_m = 0.0;
        cfg.engine.friction_c0_nm = 0.0;
        cfg.engine.friction_c1_nms = 0.0;
        cfg.engine.friction_c2_nms2 = 0.0;
        let mut sim = Simulator::new(&cfg);
        sim.control.starter_cmd = false;
        sim.control.spark_cmd = false;
        sim.control.fuel_cmd = false;
        sim.control.throttle_cmd = 0.82;

        let rpm = 6_500.0;
        let dt = rpm_linked_dt(
            cfg.environment.dt,
            rpm,
            cfg.engine.idle_target_rpm,
            &cfg.numerics,
        );
        let steps = 160usize;
        let elapsed = dt * steps as f64;
        let tau = cfg.model.throttle_time_constant_s;
        let alpha0 = 0.07;
        let state0 = EngineState {
            omega_rad_s: rpm_to_rad_s(rpm),
            theta_rad: 1.3,
            p_intake_pa: cfg.environment.ambient_pressure_pa,
            p_intake_runner_pa: cfg.environment.ambient_pressure_pa,
            p_exhaust_pa: cfg.environment.ambient_pressure_pa,
            p_exhaust_runner_pa: cfg.environment.ambient_pressure_pa,
            m_dot_intake_runner_kg_s: 0.0,
            m_dot_exhaust_runner_kg_s: 0.0,
            throttle_eff: alpha0,
            running: true,
        };

        let exact_alpha =
            sim.control.throttle_cmd + (alpha0 - sim.control.throttle_cmd) * (-elapsed / tau).exp();
        let exact_theta = wrap_cycle(state0.theta_rad + state0.omega_rad_s * elapsed);
        let final_state = integrate_state_open_loop_rk3(&sim, state0, dt, steps);

        assert!(
            (final_state.throttle_eff - exact_alpha).abs() < 2.0e-7,
            "throttle RK3 error too large at high rpm, exact={exact_alpha:.6}, got={:.6}",
            final_state.throttle_eff
        );
        assert!(
            (final_state.omega_rad_s - state0.omega_rad_s).abs() < 1.0e-12,
            "omega should remain constant when all torques are zero"
        );
        assert!(
            wrapped_angle_error(final_state.theta_rad, exact_theta) < 1.0e-10,
            "theta integration should remain exact for constant omega"
        );
    }

    #[test]
    fn rk3_matches_exact_linear_spin_decay_at_high_rpm() {
        let mut cfg = AppConfig::default();
        cfg.engine.bore_m = 0.0;
        cfg.engine.stroke_m = 0.0;
        cfg.engine.friction_c0_nm = 0.0;
        cfg.engine.friction_c1_nms = 0.012;
        cfg.engine.friction_c2_nms2 = 0.0;
        let mut sim = Simulator::new(&cfg);
        sim.control.starter_cmd = false;
        sim.control.spark_cmd = false;
        sim.control.fuel_cmd = false;
        sim.control.throttle_cmd = 0.22;

        let rpm = 6_700.0;
        let dt = rpm_linked_dt(
            cfg.environment.dt,
            rpm,
            cfg.engine.idle_target_rpm,
            &cfg.numerics,
        );
        let steps = 96usize;
        let elapsed = dt * steps as f64;
        let omega0 = rpm_to_rad_s(rpm);
        let theta0 = 0.9;
        let a = cfg.engine.friction_c1_nms / cfg.engine.inertia_kgm2;
        let state0 = EngineState {
            omega_rad_s: omega0,
            theta_rad: theta0,
            p_intake_pa: cfg.environment.ambient_pressure_pa,
            p_intake_runner_pa: cfg.environment.ambient_pressure_pa,
            p_exhaust_pa: cfg.environment.ambient_pressure_pa,
            p_exhaust_runner_pa: cfg.environment.ambient_pressure_pa,
            m_dot_intake_runner_kg_s: 0.0,
            m_dot_exhaust_runner_kg_s: 0.0,
            throttle_eff: sim.control.throttle_cmd,
            running: true,
        };

        let exact_omega = omega0 * (-a * elapsed).exp();
        let exact_theta = wrap_cycle(theta0 + omega0 * (1.0 - (-a * elapsed).exp()) / a);
        let final_state = integrate_state_open_loop_rk3(&sim, state0, dt, steps);

        let rel_omega_err = ((final_state.omega_rad_s - exact_omega) / exact_omega).abs();
        assert!(
            rel_omega_err < 3.0e-8,
            "omega RK3 relative error too large at high rpm, exact={exact_omega:.6}, got={:.6}, rel={rel_omega_err:.3e}",
            final_state.omega_rad_s
        );
        assert!(
            wrapped_angle_error(final_state.theta_rad, exact_theta) < 1.0e-6,
            "theta RK3 absolute error too large, exact={exact_theta:.6}, got={:.6}",
            final_state.theta_rad
        );
    }

    #[test]
    fn realtime_performance_estimate_reports_positive_headroom() {
        let cfg = AppConfig::default();
        let perf = estimate_realtime_performance(&cfg, cfg.environment.dt);

        eprintln!(
            "realtime wall_per_step={:.3e}s floor={:.3e}s fixed_candidate={:.3e}s headroom={:.1}x",
            perf.wall_per_step_s,
            perf.floor_dt_s,
            perf.fixed_dt_candidate_s,
            perf.fixed_dt_headroom_ratio
        );
        assert!(perf.wall_per_step_s.is_finite() && perf.wall_per_step_s > 0.0);
        assert!(
            perf.floor_dt_s.is_finite() && perf.floor_dt_s >= cfg.numerics.realtime_floor_min_s
        );
        assert!(perf.fixed_dt_candidate_s.is_finite() && perf.fixed_dt_candidate_s > 0.0);
        assert!(perf.fixed_dt_headroom_ratio.is_finite() && perf.fixed_dt_headroom_ratio > 0.0);
    }

    #[test]
    fn audio_energy_increases_with_exhaust_pressure() {
        let low = render_engine_audio(
            AudioParams {
                exhaust_pressure_kpa: 102.0,
                exhaust_runner_pressure_kpa: 104.0,
                intake_runner_pressure_kpa: 96.0,
                exhaust_wave_kpa: 0.0,
                exhaust_runner_flow_gps: 12.0,
                engine_speed_rpm: 900.0,
                exhaust_temp_k: 880.0,
                cycle_deg: 0.0,
                output_gain: 1.0,
            },
            2.0,
            48_000,
        );
        let high = render_engine_audio(
            AudioParams {
                exhaust_pressure_kpa: 165.0,
                exhaust_runner_pressure_kpa: 188.0,
                intake_runner_pressure_kpa: 92.0,
                exhaust_wave_kpa: 0.0,
                exhaust_runner_flow_gps: 58.0,
                engine_speed_rpm: 3000.0,
                exhaust_temp_k: 980.0,
                cycle_deg: 0.0,
                output_gain: 1.0,
            },
            2.0,
            48_000,
        );
        let rms = |samples: &[f32]| -> f32 {
            let power = samples.iter().map(|v| v * v).sum::<f32>() / samples.len().max(1) as f32;
            power.sqrt()
        };
        assert!(rms(&high) > rms(&low) * 1.2);
    }

    #[test]
    fn audio_normalization_limits_extreme_loudness_ratio() {
        let low = render_engine_audio(
            AudioParams {
                exhaust_pressure_kpa: 102.0,
                exhaust_runner_pressure_kpa: 104.0,
                intake_runner_pressure_kpa: 96.0,
                exhaust_wave_kpa: 0.0,
                exhaust_runner_flow_gps: 10.0,
                engine_speed_rpm: 900.0,
                exhaust_temp_k: 860.0,
                cycle_deg: 0.0,
                output_gain: 1.0,
            },
            2.0,
            48_000,
        );
        let high = render_engine_audio(
            AudioParams {
                exhaust_pressure_kpa: 170.0,
                exhaust_runner_pressure_kpa: 198.0,
                intake_runner_pressure_kpa: 88.0,
                exhaust_wave_kpa: 0.0,
                exhaust_runner_flow_gps: 65.0,
                engine_speed_rpm: 4_200.0,
                exhaust_temp_k: 1_020.0,
                cycle_deg: 0.0,
                output_gain: 1.0,
            },
            2.0,
            48_000,
        );
        let rms = |samples: &[f32]| -> f32 {
            let power = samples.iter().map(|v| v * v).sum::<f32>() / samples.len().max(1) as f32;
            power.sqrt()
        };
        let ratio = rms(&high) / rms(&low).max(1.0e-6);
        assert!(
            ratio > 1.1,
            "normalization should not erase load/rpm loudness increase"
        );
        assert!(
            ratio < 4.0,
            "normalization should limit extreme loudness swing, ratio={ratio:.3}"
        );
    }

    #[test]
    fn audio_is_nearly_silent_at_zero_rpm() {
        let stopped = render_engine_audio(
            AudioParams {
                exhaust_pressure_kpa: 101.325,
                exhaust_runner_pressure_kpa: 101.325,
                intake_runner_pressure_kpa: 101.325,
                exhaust_wave_kpa: 0.0,
                exhaust_runner_flow_gps: 0.0,
                engine_speed_rpm: 0.0,
                exhaust_temp_k: 880.0,
                cycle_deg: 0.0,
                output_gain: 1.0,
            },
            1.0,
            48_000,
        );
        let idling = render_engine_audio(
            AudioParams {
                exhaust_pressure_kpa: 103.0,
                exhaust_runner_pressure_kpa: 106.0,
                intake_runner_pressure_kpa: 92.0,
                exhaust_wave_kpa: 0.0,
                exhaust_runner_flow_gps: 8.0,
                engine_speed_rpm: 850.0,
                exhaust_temp_k: 880.0,
                cycle_deg: 0.0,
                output_gain: 1.0,
            },
            1.0,
            48_000,
        );
        let rms = |samples: &[f32]| -> f32 {
            let power = samples.iter().map(|v| v * v).sum::<f32>() / samples.len().max(1) as f32;
            power.sqrt()
        };
        assert!(
            rms(&stopped) < 1.0e-4,
            "audio should be effectively silent at zero rpm, rms={:.6}",
            rms(&stopped)
        );
        assert!(
            rms(&idling) > rms(&stopped) * 100.0,
            "idling audio should be meaningfully above zero-rpm audio"
        );
    }

    #[test]
    fn audio_pitch_tracks_firing_frequency() {
        let sample_rate = 48_000;
        let low_rpm = 1_000.0_f32;
        let high_rpm = 3_000.0_f32;
        let low = render_engine_audio(
            AudioParams {
                exhaust_pressure_kpa: 120.0,
                exhaust_runner_pressure_kpa: 132.0,
                intake_runner_pressure_kpa: 94.0,
                exhaust_wave_kpa: 0.0,
                exhaust_runner_flow_gps: 18.0,
                engine_speed_rpm: low_rpm,
                exhaust_temp_k: 900.0,
                cycle_deg: 0.0,
                output_gain: 1.0,
            },
            1.5,
            sample_rate,
        );
        let high = render_engine_audio(
            AudioParams {
                exhaust_pressure_kpa: 120.0,
                exhaust_runner_pressure_kpa: 175.0,
                intake_runner_pressure_kpa: 88.0,
                exhaust_wave_kpa: 0.0,
                exhaust_runner_flow_gps: 48.0,
                engine_speed_rpm: high_rpm,
                exhaust_temp_k: 900.0,
                cycle_deg: 0.0,
                output_gain: 1.0,
            },
            1.5,
            sample_rate,
        );

        let estimate_periodic_frequency = |samples: &[f32], min_hz: f32, max_hz: f32| -> f32 {
            let start = samples.len() / 4;
            let window = &samples[start..];
            let rectified: Vec<f32> = window.iter().map(|v| v.abs()).collect();
            let mean = rectified.iter().copied().sum::<f32>() / rectified.len().max(1) as f32;
            let min_lag = (sample_rate as f32 / max_hz).floor() as usize;
            let max_lag = (sample_rate as f32 / min_hz).ceil() as usize;
            let mut best_lag = min_lag.max(1);
            let mut best_corr = f32::MIN;
            for lag in min_lag.max(1)..=max_lag.min(rectified.len().saturating_sub(2)) {
                let mut corr = 0.0_f32;
                for i in 0..(rectified.len() - lag) {
                    let a = rectified[i] - mean;
                    let b = rectified[i + lag] - mean;
                    corr += a * b;
                }
                if corr > best_corr {
                    best_corr = corr;
                    best_lag = lag;
                }
            }
            sample_rate as f32 / best_lag as f32
        };

        let low_est_hz = estimate_periodic_frequency(&low, 20.0, 90.0);
        let high_est_hz = estimate_periodic_frequency(&high, 70.0, 220.0);
        let low_expected_hz = low_rpm * FIXED_CYLINDER_COUNT as f32 / 120.0;
        let high_expected_hz = high_rpm * FIXED_CYLINDER_COUNT as f32 / 120.0;

        assert!(
            ((low_est_hz - low_expected_hz) / low_expected_hz).abs() < 0.15,
            "low-rpm audio periodicity should follow firing frequency, expected {:.1} Hz got {:.1} Hz",
            low_expected_hz,
            low_est_hz
        );
        assert!(
            ((high_est_hz - high_expected_hz) / high_expected_hz).abs() < 0.12,
            "high-rpm audio periodicity should follow firing frequency, expected {:.1} Hz got {:.1} Hz",
            high_expected_hz,
            high_est_hz
        );
        assert!(
            high_est_hz > low_est_hz * 2.4,
            "audio pitch should rise strongly with rpm, low {:.1} Hz high {:.1} Hz",
            low_est_hz,
            high_est_hz
        );
    }

    #[test]
    fn audio_higher_exhaust_flow_increases_broadband_content() {
        let sample_rate = 48_000;
        let low_flow = render_engine_audio(
            AudioParams {
                exhaust_pressure_kpa: 126.0,
                exhaust_runner_pressure_kpa: 136.0,
                intake_runner_pressure_kpa: 94.0,
                exhaust_wave_kpa: 1.5,
                exhaust_runner_flow_gps: 12.0,
                engine_speed_rpm: 2_400.0,
                exhaust_temp_k: 920.0,
                cycle_deg: 0.0,
                output_gain: 1.0,
            },
            1.5,
            sample_rate,
        );
        let high_flow = render_engine_audio(
            AudioParams {
                exhaust_pressure_kpa: 126.0,
                exhaust_runner_pressure_kpa: 148.0,
                intake_runner_pressure_kpa: 90.0,
                exhaust_wave_kpa: 4.5,
                exhaust_runner_flow_gps: 52.0,
                engine_speed_rpm: 2_400.0,
                exhaust_temp_k: 920.0,
                cycle_deg: 0.0,
                output_gain: 1.0,
            },
            1.5,
            sample_rate,
        );
        let hf_ratio = |samples: &[f32]| -> f32 {
            let start = samples.len() / 4;
            let window = &samples[start..];
            let rms =
                (window.iter().map(|v| v * v).sum::<f32>() / window.len().max(1) as f32).sqrt();
            let diff_rms = (window
                .windows(2)
                .map(|w| {
                    let dv = w[1] - w[0];
                    dv * dv
                })
                .sum::<f32>()
                / window.len().max(1) as f32)
                .sqrt();
            diff_rms / rms.max(1.0e-6)
        };
        let low_ratio = hf_ratio(&low_flow);
        let high_ratio = hf_ratio(&high_flow);
        assert!(
            high_ratio > low_ratio * 1.05,
            "higher exhaust flow should add some broadband edge content, low={low_ratio:.3} high={high_ratio:.3}"
        );
    }

    #[test]
    fn pv_history_contains_multiple_recent_cycles() {
        let cfg = AppConfig::default();
        let mut sim = Simulator::new(&cfg);
        sim.auto.enabled = true;
        for _ in 0..((6.0 / cfg.environment.dt) as usize) {
            sim.step(cfg.environment.dt);
        }
        let first_cycle = sim
            .pv_history
            .front()
            .map(|s| s.cycle)
            .expect("pv history should not be empty");
        let last_cycle = sim
            .pv_history
            .back()
            .map(|s| s.cycle)
            .expect("pv history should not be empty");
        assert!(last_cycle >= first_cycle);
        assert!(last_cycle - first_cycle <= cfg.plot.pv_recent_cycles as u64);
    }

    #[test]
    fn pv_plot_is_hidden_when_engine_is_stopped() {
        let cfg = AppConfig::default();
        let mut sim = Simulator::new(&cfg);
        sim.pv_history.push_back(PvSample {
            cycle: 0,
            cycle_deg: 0.0,
            volume: 0.2,
            pressure_pa: 180_000.0,
        });
        sim.state.omega_rad_s = 0.0;
        sim.state.running = false;
        sim.control.starter_cmd = false;
        sim.control.spark_cmd = false;
        sim.control.fuel_cmd = false;

        let obs = sim.step(cfg.environment.dt);

        assert!(
            sim.pv_history.is_empty(),
            "stopped engine should clear stale p-V history"
        );
        assert!(
            obs.pv_points.is_empty(),
            "stopped engine should not draw a p-V loop"
        );
        assert_eq!(obs.indicated_work_cycle_j, 0.0);
        assert_eq!(obs.eta_thermal_indicated_pv, 0.0);
    }

    #[test]
    fn compression_and_exhaust_flow_functions_are_well_behaved() {
        let t = EnvironmentConfig::default().intake_temp_k;
        let flow_low = orifice_mass_flow(3.0e-4, 101_325.0, 95_000.0, t);
        let flow_mid = orifice_mass_flow(3.0e-4, 101_325.0, 70_000.0, t);
        let flow_choked = orifice_mass_flow(3.0e-4, 101_325.0, 30_000.0, t);
        assert!(flow_low > 0.0);
        assert!(flow_mid > flow_low);
        assert!(flow_choked >= flow_mid);

        let rpm = rad_s_to_rpm(rpm_to_rad_s(1200.0));
        assert!((rpm - 1200.0).abs() < 1e-9);
    }

    #[test]
    fn pv_work_integral_matches_rectangular_cycle() {
        let cfg = AppConfig::default();
        let mut sim = Simulator::new(&cfg);
        let cycle = 3_u64;
        sim.pv_history.clear();
        sim.pv_history.push_back(PvSample {
            cycle,
            cycle_deg: 0.0,
            volume: 0.50,
            pressure_pa: 100_000.0,
        });
        sim.pv_history.push_back(PvSample {
            cycle,
            cycle_deg: 180.0,
            volume: 0.50,
            pressure_pa: 400_000.0,
        });
        sim.pv_history.push_back(PvSample {
            cycle,
            cycle_deg: 360.0,
            volume: 1.00,
            pressure_pa: 400_000.0,
        });
        sim.pv_history.push_back(PvSample {
            cycle,
            cycle_deg: 540.0,
            volume: 1.00,
            pressure_pa: 100_000.0,
        });

        let work = sim
            .indicated_work_for_cycle_j(cycle)
            .expect("should compute p-V work");
        let swept = sim.swept_volume_per_cylinder_m3();
        let expected = (400_000.0 - 100_000.0) * (1.00 - 0.50) * swept;
        let rel_err = ((work - expected) / expected).abs();
        assert!(
            rel_err < 1e-9,
            "work mismatch, got {work}, expected {expected}, rel_err={rel_err}"
        );
    }

    #[test]
    fn thermal_efficiency_observation_is_finite() {
        let cfg = AppConfig::default();
        let dt = cfg.environment.dt;
        let mut sim = Simulator::new(&cfg);
        sim.auto.enabled = true;
        let mut obs = sim.step(dt);
        for _ in 0..((8.0 / dt) as usize) {
            obs = sim.step(dt);
        }
        assert!((0.4..0.75).contains(&obs.eta_thermal_theoretical));
        assert!(obs.eta_thermal_indicated_pv.is_finite());
        assert!(obs.indicated_work_cycle_j.is_finite());
    }

    #[test]
    fn rpm_linked_dt_becomes_smaller_as_rpm_increases() {
        let base = 0.0015;
        let idle = 850.0;
        let numerics = NumericsConfig::default();
        let dt_idle = rpm_linked_dt(base, idle, idle, &numerics);
        let dt_high = rpm_linked_dt(base, 4000.0, idle, &numerics);
        let dt_low = rpm_linked_dt(base, 200.0, idle, &numerics);
        assert!(dt_high < dt_idle);
        assert!(dt_low > dt_idle);
    }

    #[test]
    fn state_error_norm_detects_difference() {
        let a = EngineState {
            omega_rad_s: 120.0,
            theta_rad: 1.0,
            p_intake_pa: 45_000.0,
            p_intake_runner_pa: 44_800.0,
            p_exhaust_pa: 105_000.0,
            p_exhaust_runner_pa: 106_000.0,
            m_dot_intake_runner_kg_s: 0.01,
            m_dot_exhaust_runner_kg_s: 0.02,
            throttle_eff: 0.12,
            running: true,
        };
        let b = EngineState {
            omega_rad_s: 121.5,
            theta_rad: 1.02,
            p_intake_pa: 45_300.0,
            p_intake_runner_pa: 45_050.0,
            p_exhaust_pa: 104_700.0,
            p_exhaust_runner_pa: 105_600.0,
            m_dot_intake_runner_kg_s: 0.013,
            m_dot_exhaust_runner_kg_s: 0.017,
            throttle_eff: 0.121,
            running: true,
        };
        let numerics = NumericsConfig::default();
        assert!(state_error_norm(a, a, &numerics) < 1.0e-12);
        assert!(state_error_norm(a, b, &numerics) > 0.0);
    }

    #[test]
    fn ignition_timing_command_changes_effective_advance() {
        let cfg = AppConfig::default();
        let mut sim = Simulator::new(&cfg);
        sim.control.ignition_timing_deg = 10.0;
        let base = sim.ignition_timing_deg();
        sim.control.ignition_timing_deg = 18.0;
        let advanced = sim.ignition_timing_deg();
        sim.control.ignition_timing_deg = 2.0;
        let retarded = sim.ignition_timing_deg();
        assert!(
            advanced > base,
            "BTDC larger should mean more advanced ignition"
        );
        assert!(
            retarded < base,
            "BTDC smaller should mean more retarded ignition"
        );
    }

    #[test]
    fn pv_combustion_phasing_tracks_burn_window() {
        let pv_model = PvModelConfig::default();
        let sample_pressure = |deg: f64, soc_deg: f64, eoc_deg: f64| -> f64 {
            instantaneous_pv_sample(
                10.5,
                deg.to_radians(),
                90_000.0,
                115_000.0,
                4_200_000.0,
                true,
                soc_deg,
                eoc_deg,
                &pv_model,
            )
            .1
        };

        let early_soc = 342.0;
        let early_eoc = 404.0;
        let late_soc = 366.0;
        let late_eoc = 428.0;

        let p_early_370 = sample_pressure(370.0, early_soc, early_eoc);
        let p_late_370 = sample_pressure(370.0, late_soc, late_eoc);
        let peak_angle = |soc_deg: f64, eoc_deg: f64| -> f64 {
            let mut best_deg = 360.0;
            let mut best_p = f64::MIN;
            for deg in 320..521 {
                let pressure = sample_pressure(deg as f64, soc_deg, eoc_deg);
                if pressure > best_p {
                    best_p = pressure;
                    best_deg = deg as f64;
                }
            }
            best_deg
        };

        let peak_early = peak_angle(early_soc, early_eoc);
        let peak_late = peak_angle(late_soc, late_eoc);

        assert!(
            p_early_370 > p_late_370,
            "earlier combustion phasing should raise pressure earlier in the power stroke"
        );
        assert!(
            peak_late > peak_early + 8.0,
            "retarded burn window should shift the pressure peak later, early peak={peak_early:.1} deg late peak={peak_late:.1} deg"
        );
    }

    #[test]
    fn ptheta_curves_show_staggered_cylinder_peaks() {
        let cfg = AppConfig::default();
        let mut sim = Simulator::new(&cfg);
        configure_high_rpm_operating_point(&mut sim, 4_500.0, 1.0);

        let global_curves = sim.build_ptheta_display_curves(360);
        assert_eq!(global_curves.len(), FIXED_CYLINDER_COUNT);

        let mut global_peaks: Vec<f64> = global_curves
            .iter()
            .map(|curve| {
                curve
                    .iter()
                    .max_by(|a, b| a.1.total_cmp(&b.1))
                    .map(|point| point.0)
                    .expect("global p-theta curve should not be empty")
            })
            .collect();
        global_peaks.sort_by(|a, b| a.total_cmp(b));
        let mut global_spacings: Vec<f64> = global_peaks
            .windows(2)
            .map(|pair| pair[1] - pair[0])
            .collect();
        global_spacings.push(global_peaks[0] + 720.0 - global_peaks[FIXED_CYLINDER_COUNT - 1]);
        assert!(
            global_spacings
                .iter()
                .all(|spacing| (*spacing - 180.0).abs() < 50.0),
            "global p-theta peaks should be staggered across the 720 deg cycle, peaks={global_peaks:?}"
        );
    }

    #[test]
    fn high_rpm_nonlinear_state_converges_under_dt_refinement() {
        let cfg = AppConfig::default();
        let rpm = 6_500.0;
        let throttle = 0.98;
        let dt_nom = rpm_linked_dt(
            cfg.environment.dt,
            rpm,
            cfg.engine.idle_target_rpm,
            &cfg.numerics,
        );
        let horizon = 0.020;
        let steps_coarse = (horizon / dt_nom).round() as usize;
        let steps_mid = steps_coarse * 2;
        let steps_fine = steps_coarse * 4;
        let dt_coarse = horizon / steps_coarse as f64;
        let dt_mid = horizon / steps_mid as f64;
        let dt_fine = horizon / steps_fine as f64;

        let mut sim_coarse = Simulator::new(&cfg);
        configure_high_rpm_operating_point(&mut sim_coarse, rpm, throttle);
        let state0 = sim_coarse.state;

        let mut sim_mid = Simulator::new(&cfg);
        configure_high_rpm_operating_point(&mut sim_mid, rpm, throttle);

        let mut sim_fine = Simulator::new(&cfg);
        configure_high_rpm_operating_point(&mut sim_fine, rpm, throttle);

        let state_coarse =
            integrate_state_open_loop_rk3(&sim_coarse, state0, dt_coarse, steps_coarse);
        let state_mid = integrate_state_open_loop_rk3(&sim_mid, state0, dt_mid, steps_mid);
        let state_fine = integrate_state_open_loop_rk3(&sim_fine, state0, dt_fine, steps_fine);

        let err_coarse = state_error_norm(state_coarse, state_fine, &cfg.numerics);
        let err_mid = state_error_norm(state_mid, state_fine, &cfg.numerics);

        assert!(
            err_coarse < 2.7,
            "coarse high-rpm state error too large versus refined reference: {err_coarse:.4}"
        );
        assert!(
            err_mid < err_coarse * 0.75,
            "dt refinement should substantially reduce high-rpm state error, coarse={err_coarse:.4}, mid={err_mid:.4}"
        );
    }

    #[test]
    fn high_rpm_fired_operation_remains_physical_near_redline() {
        let cfg = AppConfig::default();
        let rpm = 6_800.0;
        let throttle = 1.0;
        let dt = rpm_linked_dt(
            cfg.environment.dt,
            rpm,
            cfg.engine.idle_target_rpm,
            &cfg.numerics,
        );
        let mut sim = Simulator::new(&cfg);
        configure_high_rpm_operating_point(&mut sim, rpm, throttle);

        let steps = (0.25 / dt).ceil() as usize;
        for _ in 0..steps {
            let obs = sim.step(dt);
            assert!(obs.rpm.is_finite(), "rpm became non-finite near redline");
            assert!(
                obs.map_kpa.is_finite(),
                "MAP became non-finite near redline"
            );
            assert!(
                obs.exhaust_kpa.is_finite(),
                "exhaust pressure became non-finite near redline"
            );
            assert!(obs.rpm >= 0.0, "rpm went negative near redline");
            assert!(
                sim.state.p_intake_pa >= cfg.model.intake_pressure_min_pa
                    && sim.state.p_intake_pa <= cfg.model.intake_pressure_max_pa,
                "intake pressure left configured bounds near redline: {}",
                sim.state.p_intake_pa
            );
            assert!(
                sim.state.p_exhaust_pa
                    >= cfg.environment.ambient_pressure_pa
                        * cfg.model.exhaust_pressure_min_ambient_ratio
                    && sim.state.p_exhaust_pa
                        <= cfg.environment.ambient_pressure_pa
                            + cfg.model.exhaust_pressure_max_over_ambient_pa,
                "exhaust pressure left configured bounds near redline: {}",
                sim.state.p_exhaust_pa
            );
        }
    }

    #[test]
    fn retarded_ignition_raises_exhaust_temperature() {
        let run_case = |ignition_deg: f64| -> f64 {
            let cfg = AppConfig::default();
            let dt = cfg.environment.dt;
            let mut sim = Simulator::new(&cfg);
            sim.control.starter_cmd = true;
            sim.control.spark_cmd = true;
            sim.control.fuel_cmd = true;
            sim.control.throttle_cmd = 0.36;
            sim.control.ignition_timing_deg = ignition_deg;
            let _ = sim.step(dt);
            for i in 0..((10.0 / dt) as usize) {
                if i > (2.0 / dt) as usize {
                    sim.control.starter_cmd = false;
                }
                let _ = sim.step(dt);
            }

            let mut temp_sum = 0.0;
            let mut count = 0usize;
            for _ in 0..((1.5 / dt) as usize) {
                let obs = sim.step(dt);
                temp_sum += obs.exhaust_temp_k;
                count = count.saturating_add(1);
            }
            temp_sum / count as f64
        };

        let temp_nom = run_case(16.0);
        let temp_ret = run_case(0.0);
        assert!(
            temp_ret > temp_nom + 35.0,
            "retarded ignition should heat exhaust, nominal={temp_nom:.1} K, retarded={temp_ret:.1} K"
        );
    }

    #[test]
    fn overly_advanced_ignition_can_stall_engine() {
        let cfg = AppConfig::default();
        let dt = cfg.environment.dt;
        let mut sim = Simulator::new(&cfg);
        sim.control.starter_cmd = true;
        sim.control.spark_cmd = true;
        sim.control.fuel_cmd = true;
        sim.control.throttle_cmd = 0.30;
        sim.control.ignition_timing_deg = 45.0;

        let mut obs = sim.step(dt);
        for i in 0..((9.0 / dt) as usize) {
            if i > (2.0 / dt) as usize {
                sim.control.starter_cmd = false;
            }
            obs = sim.step(dt);
        }

        assert!(
            !sim.state.running,
            "engine should not remain running under extreme over-advanced ignition"
        );
        assert!(
            obs.rpm < 320.0,
            "engine speed should drop near stall under extreme over-advanced ignition, rpm={:.1}",
            obs.rpm
        );
    }

    #[test]
    fn higher_compression_ratio_improves_theoretical_efficiency() {
        let mut low_cfg = AppConfig::default();
        low_cfg.engine.compression_ratio = 8.0;
        let mut hi_cfg = AppConfig::default();
        hi_cfg.engine.compression_ratio = 12.0;
        let low = Simulator::new(&low_cfg).theoretical_otto_efficiency();
        let hi = Simulator::new(&hi_cfg).theoretical_otto_efficiency();
        assert!(hi > low);
    }

    #[test]
    fn no_fuel_means_near_zero_indicated_efficiency() {
        let cfg = AppConfig::default();
        let dt = cfg.environment.dt;
        let mut sim = Simulator::new(&cfg);
        sim.control.starter_cmd = true;
        sim.control.spark_cmd = true;
        sim.control.fuel_cmd = false;
        sim.control.throttle_cmd = 0.2;
        let mut obs = sim.step(dt);
        for _ in 0..((4.0 / dt) as usize) {
            obs = sim.step(dt);
        }
        assert!(obs.eta_thermal_indicated_pv <= 0.05);
        assert!(obs.torque_combustion_nm.abs() < 1.0e-6);
    }

    #[test]
    fn friction_slows_engine_without_starter_and_fuel() {
        let cfg = AppConfig::default();
        let dt = cfg.environment.dt;
        let mut sim = Simulator::new(&cfg);
        sim.state.omega_rad_s = rpm_to_rad_s(2000.0);
        sim.state.running = true;
        sim.control.starter_cmd = false;
        sim.control.spark_cmd = false;
        sim.control.fuel_cmd = false;
        sim.control.throttle_cmd = 0.0;
        let rpm_start = rad_s_to_rpm(sim.state.omega_rad_s);
        for _ in 0..((3.0 / dt) as usize) {
            sim.step(dt);
        }
        let rpm_end = rad_s_to_rpm(sim.state.omega_rad_s);
        assert!(rpm_end < rpm_start);
    }

    #[test]
    fn fired_pressure_does_not_collapse_immediately_after_combustion() {
        let cr = 10.5;
        let p_intake = 90_000.0;
        let p_exhaust = 120_000.0;
        let p_peak = 4_500_000.0;
        let sample = |deg: f64| -> f64 {
            let theta = deg.to_radians();
            let pv_model = PvModelConfig::default();
            let (_, p) = instantaneous_pv_sample(
                cr, theta, p_intake, p_exhaust, p_peak, true, 350.0, 430.0, &pv_model,
            );
            p
        };
        let p_380 = sample(380.0);
        let p_500 = sample(500.0);
        let p_570 = sample(570.0);
        assert!(p_380 > p_500);
        assert!(p_500 > p_exhaust * 1.2);
        assert!(p_570 > p_exhaust);
    }

    #[test]
    fn imep_is_not_alternating_zero_under_wot_fired_operation() {
        let cfg = AppConfig::default();
        let dt = cfg.environment.dt;
        let mut sim = Simulator::new(&cfg);
        sim.control.starter_cmd = true;
        sim.control.spark_cmd = true;
        sim.control.fuel_cmd = true;
        sim.control.throttle_cmd = 1.0;

        for i in 0..((6.0 / dt) as usize) {
            let _ = sim.step(dt);
            if i > (2.0 / dt) as usize {
                sim.control.starter_cmd = false;
            }
        }

        let mut nonzero = 0usize;
        let samples = 240usize;
        for _ in 0..samples {
            let obs = sim.step(dt);
            if obs.imep_bar.abs() > 0.05 {
                nonzero = nonzero.saturating_add(1);
            }
        }
        assert!(
            nonzero > samples * 9 / 10,
            "imep nonzero ratio too low: {nonzero}/{samples}"
        );
    }

    #[test]
    fn net_torque_is_consistent_with_components() {
        let cfg = AppConfig::default();
        let dt = cfg.environment.dt;
        let mut sim = Simulator::new(&cfg);
        sim.auto.enabled = true;
        let obs = sim.step(dt);
        let expected = obs.torque_combustion_nm + obs.torque_starter_nm
            - obs.torque_friction_nm
            - obs.torque_pumping_nm;
        assert!((obs.torque_net_inst_nm - expected).abs() < 1.0e-9);
        assert!(obs.torque_net_nm.is_finite());
    }

    #[test]
    fn smoothed_net_torque_has_less_ripple_than_instantaneous() {
        let cfg = AppConfig::default();
        let dt = cfg.environment.dt;
        let mut sim = Simulator::new(&cfg);
        sim.control.starter_cmd = true;
        sim.control.spark_cmd = true;
        sim.control.fuel_cmd = true;
        sim.control.throttle_cmd = 1.0;

        for i in 0..((6.0 / dt) as usize) {
            let _ = sim.step(dt);
            if i > (2.0 / dt) as usize {
                sim.control.starter_cmd = false;
            }
        }

        let mut inst = Vec::with_capacity(480);
        let mut smooth = Vec::with_capacity(480);
        for _ in 0..480 {
            let obs = sim.step(dt);
            inst.push(obs.torque_net_inst_nm);
            smooth.push(obs.torque_net_nm);
        }

        let stddev = |v: &[f64]| -> f64 {
            let mean = v.iter().copied().sum::<f64>() / v.len().max(1) as f64;
            let var = v
                .iter()
                .map(|x| {
                    let d = *x - mean;
                    d * d
                })
                .sum::<f64>()
                / v.len().max(1) as f64;
            var.sqrt()
        };

        let inst_std = stddev(&inst);
        let smooth_std = stddev(&smooth);
        assert!(smooth_std < inst_std * 0.70);
    }

    #[test]
    fn steady_state_efficiency_sweep_matches_gasoline_si_range() {
        let cfg = AppConfig::default();
        let throttles = [0.10, 0.16, 0.24, 0.34, 0.50, 0.70, 1.00];
        // Throttled SI engines collapse sharply at very light load, while modern
        // production gasoline engines peak around the high-30% to 40% efficiency band.
        let eta_min_by_throttle = [0.08, 0.08, 0.08, 0.10, 0.20, 0.30, 0.22];
        let eta_max_realistic = cfg.model.eta_indicated_max + 1.0e-9;
        let mut eta_by_throttle = Vec::with_capacity(throttles.len());
        for throttle in throttles.iter().copied() {
            let case = FixedControlSteadyCase {
                label: "efficiency_sweep",
                throttle,
                load_cmd: 0.80,
                settle_time_s: 14.0,
            };
            let (_, obs, average) = sample_fixed_control_case(&cfg, case, 2.0);
            let eta_avg = average.mean_eta();
            let rpm_avg = average.mean_rpm();
            eta_by_throttle.push((throttle, rpm_avg, eta_avg));
            assert!(obs.eta_thermal_indicated_pv.is_finite());
        }

        for (throttle, rpm, eta) in eta_by_throttle.iter().copied() {
            eprintln!("steady sweep: throttle={throttle:.2}, rpm={rpm:.0}, eta_ind_pv={eta:.3}");
        }

        for (idx, (throttle, rpm_avg, eta_avg)) in eta_by_throttle.iter().copied().enumerate() {
            assert!(
                (eta_min_by_throttle[idx]..=eta_max_realistic).contains(&eta_avg),
                "eta out of expected SI indicated-efficiency band: throttle={throttle:.2}, rpm={rpm_avg:.0}, eta={eta_avg:.3}"
            );
        }

        for window in eta_by_throttle.windows(2) {
            assert!(
                window[1].1 >= window[0].1,
                "steady-state RPM should rise with throttle in the loaded sweep"
            );
        }
    }

    #[test]
    fn representative_operating_points_are_periodic_steady_states() {
        let cfg = AppConfig::default();
        let cases = [
            FixedControlSteadyCase {
                label: "light_load",
                throttle: 0.10,
                load_cmd: 0.80,
                settle_time_s: 16.0,
            },
            FixedControlSteadyCase {
                label: "mid_load",
                throttle: 0.34,
                load_cmd: 0.80,
                settle_time_s: 16.0,
            },
            FixedControlSteadyCase {
                label: "full_load",
                throttle: 1.00,
                load_cmd: 0.80,
                settle_time_s: 16.0,
            },
        ];

        for case in cases {
            let (sim, obs) =
                settle_fixed_control_case(&cfg, case.throttle, case.load_cmd, case.settle_time_s);
            let metrics = periodic_steady_metrics(&cfg, &sim, sim.state, obs.rpm, 360.0);

            eprintln!(
                "periodic steady: case={}, rpm={:.0}, cycle_time={:.4}s, err={:.4e}, delta_rpm={:.3}, max_dp={:.3}kPa, max_dmdot={:.3}g/s, delta_alpha={:.4e}",
                case.label,
                metrics.rpm,
                metrics.cycle_time_s,
                metrics.section_error_norm,
                metrics.delta_rpm,
                metrics.max_pressure_delta_kpa,
                metrics.max_runner_flow_delta_gps,
                metrics.delta_throttle,
            );

            assert!(
                metrics.section_error_norm < 2.7,
                "same-phase reduced-state map should be nearly fixed at {} (err {:.4e})",
                case.label,
                metrics.section_error_norm
            );
            assert!(
                metrics.delta_rpm / metrics.rpm.max(1.0) < 1.0e-3,
                "cycle-to-cycle RPM drift should stay small relative to mean speed at {} ({:.3} rpm over {:.0} rpm)",
                case.label,
                metrics.delta_rpm,
                metrics.rpm
            );
            assert!(
                metrics.max_pressure_delta_kpa < 3.5,
                "cycle-to-cycle pressure drift should stay small at {} ({:.3} kPa)",
                case.label,
                metrics.max_pressure_delta_kpa
            );
            assert!(
                metrics.max_runner_flow_delta_gps < 10.0,
                "cycle-to-cycle runner-flow drift should stay small at {} ({:.3} g/s)",
                case.label,
                metrics.max_runner_flow_delta_gps
            );
            assert!(
                metrics.delta_throttle < 2.0e-3,
                "effective throttle should return to the same phase-section value at {} ({:.4e})",
                case.label,
                metrics.delta_throttle
            );
        }
    }

    #[test]
    fn locked_full_load_torque_matches_modern_2l_na_gasoline_band() {
        let cfg = AppConfig::default();
        let mut sim = Simulator::new(&cfg);
        sim.control.spark_cmd = true;
        sim.control.fuel_cmd = true;
        sim.control.starter_cmd = false;
        sim.control.throttle_cmd = 1.0;

        let rpm = 4_500.0;
        let map_pa = cfg.environment.ambient_pressure_pa * 0.96;
        let exhaust_pa = cfg.environment.ambient_pressure_pa + 5_000.0;
        let load = (map_pa / cfg.environment.ambient_pressure_pa)
            .clamp(sim.model.load_min, sim.model.load_max);
        let (mbt_deg, _, _, _, _) = sim.ignition_phase_characteristics(rpm, load, 0.0);
        sim.control.ignition_timing_deg = mbt_deg;
        let avg = sim.locked_cycle_average(
            EngineState {
                omega_rad_s: rpm_to_rad_s(rpm),
                theta_rad: 0.0,
                p_intake_pa: map_pa,
                p_intake_runner_pa: map_pa,
                p_exhaust_pa: exhaust_pa,
                p_exhaust_runner_pa: exhaust_pa,
                m_dot_intake_runner_kg_s: 0.0,
                m_dot_exhaust_runner_kg_s: 0.0,
                throttle_eff: 1.0,
                running: true,
            },
            1440,
        );
        let brake_bmep_bar = torque_to_bmep_bar(avg.torque_net_nm, sim.params.displacement_m3);

        // Toyota Dynamic Force 2.0, Mazda SKYACTIV-G 2.0, and Ford 2.0 GDI Ti-VCT
        // all sit near the 200 Nm class at full load, so the simulator should stay
        // in that neighborhood for a 2.0 L NA locked-speed operating point.
        assert!(
            (140.0..=220.0).contains(&avg.torque_net_nm),
            "locked full-load torque should stay in a realistic modern 2.0L NA gasoline window, got {:.1} Nm (bmep {:.1} bar)",
            avg.torque_net_nm,
            brake_bmep_bar
        );
        assert!(
            (8.5..=13.8).contains(&brake_bmep_bar),
            "locked full-load bmep should stay in a realistic 2.0L NA band, got {:.2} bar for {:.1} Nm",
            brake_bmep_bar,
            avg.torque_net_nm
        );
    }

    #[test]
    fn locked_full_load_air_consumption_stays_in_typical_2l_na_window() {
        let cfg = AppConfig::default();
        let mut sim = Simulator::new(&cfg);
        sim.control.spark_cmd = true;
        sim.control.fuel_cmd = true;
        sim.control.starter_cmd = false;
        sim.control.throttle_cmd = 1.0;

        let rpm = 4_500.0;
        let map_pa = cfg.environment.ambient_pressure_pa * 0.96;
        let load = (map_pa / cfg.environment.ambient_pressure_pa)
            .clamp(sim.model.load_min, sim.model.load_max);
        let (mbt_deg, _, _, _, _) = sim.ignition_phase_characteristics(rpm, load, 0.0);
        sim.control.ignition_timing_deg = mbt_deg;
        let avg = sim.locked_cycle_average(
            EngineState {
                omega_rad_s: rpm_to_rad_s(rpm),
                theta_rad: 0.0,
                p_intake_pa: map_pa,
                p_intake_runner_pa: map_pa,
                p_exhaust_pa: cfg.environment.ambient_pressure_pa + 5_000.0,
                p_exhaust_runner_pa: cfg.environment.ambient_pressure_pa + 5_000.0,
                m_dot_intake_runner_kg_s: 0.0,
                m_dot_exhaust_runner_kg_s: 0.0,
                throttle_eff: 1.0,
                running: true,
            },
            1440,
        );

        assert!(
            (55.0..=90.0).contains(&avg.air_consumption_gps),
            "locked full-load air consumption should stay in a realistic 2.0L NA band, got {:.1} g/s",
            avg.air_consumption_gps
        );
    }

    #[test]
    fn shaft_power_helpers_match_reference_conversion() {
        let rpm = 5_252.113_122;
        let torque_nm = 1.355_817_948_331_4;
        let power_kw = shaft_power_kw(rpm, torque_nm);
        let power_hp = shaft_power_hp(rpm, torque_nm);

        assert!((power_kw - 0.745_699_871_582_270_1).abs() < 1.0e-9);
        assert!((power_hp - 1.0).abs() < 1.0e-9);
    }
}
