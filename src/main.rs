use std::collections::VecDeque;
use std::f64::consts::PI;
use std::io::{Read, Write, stdin, stdout};
use std::process::Command;
use std::time::{Duration, Instant};

const R_AIR: f64 = 287.0;
const T_AIR: f64 = 300.0;
const P_AMBIENT: f64 = 101_325.0;
const P_EXHAUST: f64 = 101_325.0;
const IDLE_TARGET_RPM: f64 = 850.0;
const DT: f64 = 0.002;

#[derive(Debug, Clone)]
struct EngineParams {
    displacement_m3: f64,
    compression_ratio: f64,
    inertia: f64,
    friction_coef: f64,
    cylinders: usize,
}

#[derive(Debug, Clone)]
struct ControlInput {
    throttle: f64,
    starter_on: bool,
    spark_on: bool,
    fuel_on: bool,
    vvt_intake_deg: f64,
    vvt_exhaust_deg: f64,
}

impl Default for ControlInput {
    fn default() -> Self {
        Self {
            throttle: 0.06,
            starter_on: false,
            spark_on: false,
            fuel_on: false,
            vvt_intake_deg: 0.0,
            vvt_exhaust_deg: 0.0,
        }
    }
}

#[derive(Debug)]
struct IntakeManifold {
    volume_m3: f64,
    pressure_pa: f64,
}

#[derive(Debug)]
struct ExhaustSystem {
    pressure_pa: f64,
}

#[derive(Debug)]
struct EngineState {
    omega_rad_s: f64,
    crank_angle_rad: f64,
    running: bool,
    cycle_phase: f64,
}

#[derive(Debug)]
struct Observation {
    rpm: f64,
    map_kpa: f64,
    torque_combustion_nm: f64,
    torque_friction_nm: f64,
    torque_starter_nm: f64,
    air_flow_gps: f64,
    trapped_air_mg: f64,
    imep_bar: f64,
    stable_idle: bool,
    pv_points: Vec<(f64, f64)>,
}

struct Simulator {
    params: EngineParams,
    control: ControlInput,
    intake: IntakeManifold,
    exhaust: ExhaustSystem,
    engine: EngineState,
    history_rpm: VecDeque<f64>,
}

impl Simulator {
    fn new() -> Self {
        Self {
            params: EngineParams {
                displacement_m3: 0.0020,
                compression_ratio: 10.5,
                inertia: 0.18,
                friction_coef: 0.018,
                cylinders: 4,
            },
            control: ControlInput::default(),
            intake: IntakeManifold {
                volume_m3: 0.003,
                pressure_pa: 35_000.0,
            },
            exhaust: ExhaustSystem {
                pressure_pa: P_EXHAUST,
            },
            engine: EngineState {
                omega_rad_s: 0.0,
                crank_angle_rad: 0.0,
                running: false,
                cycle_phase: 0.0,
            },
            history_rpm: VecDeque::with_capacity(200),
        }
    }

    fn step(&mut self, dt: f64) -> Observation {
        let throttle_area = 4.0e-4 * (0.12 + 8.0 * self.control.throttle.powi(2));
        let m_dot_throttle = throttle_mass_flow(throttle_area, P_AMBIENT, self.intake.pressure_pa);

        let rpm = rad_s_to_rpm(self.engine.omega_rad_s.max(0.0));
        let ve = volumetric_efficiency(
            rpm,
            self.control.vvt_intake_deg,
            self.control.vvt_exhaust_deg,
            self.control.throttle,
        );
        let m_dot_engine = engine_air_consumption(
            self.params.displacement_m3,
            rpm,
            ve,
            self.intake.pressure_pa,
            T_AIR,
        );

        let dm = (m_dot_throttle - m_dot_engine) * dt;
        let current_mass = self.intake.pressure_pa * self.intake.volume_m3 / (R_AIR * T_AIR);
        let next_mass = (current_mass + dm).max(1e-6);
        self.intake.pressure_pa = next_mass * R_AIR * T_AIR / self.intake.volume_m3;

        let trapped_air = trapped_air_mass(
            self.params.displacement_m3,
            ve,
            self.intake.pressure_pa,
            T_AIR,
            self.params.cylinders,
        );

        let combustion_enabled = self.control.fuel_on
            && self.control.spark_on
            && self.control.starter_on
            && rpm > 120.0
            && self.intake.pressure_pa > 28_000.0;

        let torque_combustion = if combustion_enabled {
            self.engine.running = true;
            indicated_torque(trapped_air, self.control.throttle, rpm)
        } else if self.engine.running && self.control.fuel_on && self.control.spark_on {
            indicated_torque(trapped_air, self.control.throttle, rpm) * 0.65
        } else {
            self.engine.running = false;
            0.0
        };

        let torque_friction = self.params.friction_coef * self.engine.omega_rad_s
            + 5.0
            + 0.0008 * self.engine.omega_rad_s.powi(2);
        let torque_starter = if self.control.starter_on {
            starter_torque(rpm)
        } else {
            0.0
        };

        let net_torque = torque_combustion + torque_starter - torque_friction;
        self.engine.omega_rad_s =
            (self.engine.omega_rad_s + net_torque / self.params.inertia * dt).max(0.0);
        self.engine.crank_angle_rad =
            (self.engine.crank_angle_rad + self.engine.omega_rad_s * dt) % (4.0 * PI);
        self.engine.cycle_phase = self.engine.crank_angle_rad / (4.0 * PI);

        let imep_pa = if trapped_air > 0.0 {
            (torque_combustion * 4.0 * PI) / self.params.displacement_m3
        } else {
            0.0
        };

        if self.history_rpm.len() == self.history_rpm.capacity() {
            self.history_rpm.pop_front();
        }
        self.history_rpm.push_back(rpm);

        Observation {
            rpm,
            map_kpa: self.intake.pressure_pa * 1e-3,
            torque_combustion_nm: torque_combustion,
            torque_friction_nm: torque_friction,
            torque_starter_nm: torque_starter,
            air_flow_gps: m_dot_throttle * 1e3,
            trapped_air_mg: trapped_air * 1e6,
            imep_bar: imep_pa * 1e-5,
            stable_idle: self.engine.running
                && (rpm - IDLE_TARGET_RPM).abs() < 120.0
                && self.control.throttle < 0.18,
            pv_points: pv_diagram(
                self.params.compression_ratio,
                self.intake.pressure_pa,
                self.exhaust.pressure_pa,
            ),
        }
    }
}

fn throttle_mass_flow(area: f64, p_up: f64, p_down: f64) -> f64 {
    let cd = 0.8;
    let dp = (p_up - p_down).max(0.0);
    let rho = p_up / (R_AIR * T_AIR);
    cd * area * (2.0 * rho * dp).sqrt()
}

fn engine_air_consumption(displacement: f64, rpm: f64, ve: f64, map_pa: f64, t: f64) -> f64 {
    let cycles_per_sec = rpm / 120.0;
    let mass_per_cycle = displacement * ve * map_pa / (R_AIR * t);
    (mass_per_cycle * cycles_per_sec).max(0.0)
}

fn trapped_air_mass(displacement: f64, ve: f64, map_pa: f64, t: f64, cylinders: usize) -> f64 {
    let cyl_vol = displacement / cylinders as f64;
    cyl_vol * ve * map_pa / (R_AIR * t)
}

fn indicated_torque(trapped_air_kg: f64, throttle: f64, rpm: f64) -> f64 {
    let lambda_eff = (0.7 + 0.6 * throttle).clamp(0.65, 1.15);
    let lhv = 43e6;
    let afr = 14.7 / lambda_eff;
    let fuel_mass = trapped_air_kg / afr;
    let eta = (0.2 + 0.13 * throttle - 0.00002 * (rpm - 2500.0).abs()).clamp(0.12, 0.35);
    let work_per_cyl_cycle = fuel_mass * lhv * eta;
    let total_work = work_per_cyl_cycle * 4.0;
    total_work / (4.0 * PI)
}

fn starter_torque(rpm: f64) -> f64 {
    if rpm < 250.0 {
        52.0
    } else if rpm < 500.0 {
        25.0
    } else {
        0.0
    }
}

fn volumetric_efficiency(rpm: f64, vvt_i: f64, vvt_e: f64, throttle: f64) -> f64 {
    let rpm_term = (0.75 + 0.17 * (-(rpm - 2800.0).powi(2) / 2.2e6).exp()).clamp(0.45, 0.96);
    let vvt_gain = 1.0 + 0.0025 * vvt_i - 0.0018 * vvt_e;
    let throttle_gain = (0.55 + throttle * 0.9).clamp(0.45, 1.0);
    (rpm_term * vvt_gain * throttle_gain).clamp(0.35, 1.05)
}

fn pv_diagram(compression_ratio: f64, p_intake: f64, p_exhaust: f64) -> Vec<(f64, f64)> {
    let mut points = Vec::with_capacity(96);
    let gamma = 1.33;
    let v_min = 1.0 / (compression_ratio - 1.0);
    let v_max = v_min + 1.0;

    for i in 0..24 {
        let f = i as f64 / 23.0;
        let v = v_max - (v_max - v_min) * f;
        let p = p_intake * (v_max / v).powf(gamma);
        points.push((v, p));
    }
    let p_peak = points.last().map(|(_, p)| p * 3.8).unwrap_or(p_intake);
    for i in 0..24 {
        let f = i as f64 / 23.0;
        let v = v_min + (v_max - v_min) * f;
        let p = p_peak * (v_min / v).powf(gamma);
        points.push((v, p));
    }
    for i in 0..24 {
        let f = i as f64 / 23.0;
        let v = v_max - (v_max - v_min) * f;
        let p = p_exhaust + (p_intake - p_exhaust) * (1.0 - f);
        points.push((v, p));
    }
    for i in 0..24 {
        let f = i as f64 / 23.0;
        let v = v_min + (v_max - v_min) * f;
        let p = p_intake + 2500.0 * (0.5 - (f - 0.5).abs());
        points.push((v, p.max(20_000.0)));
    }

    points
}

fn rad_s_to_rpm(rad_s: f64) -> f64 {
    rad_s * 60.0 / (2.0 * PI)
}

fn draw_ui(sim: &Simulator, obs: &Observation) -> std::io::Result<()> {
    let mut out = stdout();
    write!(out, "\x1b[2J\x1b[H")?;
    writeln!(out, "4-stroke Engine Start/Idle Simulator (Rust)")?;
    writeln!(out, "================================================")?;
    writeln!(out, "Controls: [q] quit  [s] starter  [i] spark  [f] fuel")?;
    writeln!(
        out,
        "          [w/x] throttle +/-  [a/z] intake VVT +/-  [k/m] exhaust VVT +/-"
    )?;
    writeln!(out)?;

    writeln!(out, "Throttle: {:>5.1}%", sim.control.throttle * 100.0)?;
    writeln!(out, "Starter : {}", flag(sim.control.starter_on))?;
    writeln!(out, "Spark   : {}", flag(sim.control.spark_on))?;
    writeln!(out, "Fuel    : {}", flag(sim.control.fuel_on))?;
    writeln!(
        out,
        "VVT(I/E): {:+5.1} / {:+5.1} deg",
        sim.control.vvt_intake_deg, sim.control.vvt_exhaust_deg
    )?;
    writeln!(out)?;

    writeln!(out, "RPM              : {:>7.1}", obs.rpm)?;
    writeln!(out, "MAP              : {:>7.1} kPa", obs.map_kpa)?;
    writeln!(out, "Air flow         : {:>7.2} g/s", obs.air_flow_gps)?;
    writeln!(out, "Trapped air      : {:>7.2} mg/cyl", obs.trapped_air_mg)?;
    writeln!(
        out,
        "Comb torque      : {:>7.2} Nm",
        obs.torque_combustion_nm
    )?;
    writeln!(out, "Starter torque   : {:>7.2} Nm", obs.torque_starter_nm)?;
    writeln!(out, "Friction torque  : {:>7.2} Nm", obs.torque_friction_nm)?;
    writeln!(out, "IMEP             : {:>7.2} bar", obs.imep_bar)?;
    writeln!(
        out,
        "State            : {}",
        if obs.stable_idle {
            "IDLE STABLE"
        } else {
            "TRANSIENT"
        }
    )?;

    writeln!(out, "\nRPM trend:")?;
    draw_sparkline(&mut out, sim.history_rpm.iter().copied().collect(), 64, 8)?;

    writeln!(out, "\nApproximate p-V diagram:")?;
    draw_xy_plot(&mut out, &obs.pv_points, 48, 12)?;

    out.flush()?;
    Ok(())
}

fn draw_sparkline<W: Write>(
    out: &mut W,
    values: Vec<f64>,
    width: usize,
    height: usize,
) -> std::io::Result<()> {
    if values.is_empty() {
        return Ok(());
    }

    let max = values
        .iter()
        .copied()
        .fold(f64::NEG_INFINITY, f64::max)
        .max(1.0);
    let min = values
        .iter()
        .copied()
        .fold(f64::INFINITY, f64::min)
        .min(max - 1.0);

    let mut canvas = vec![vec![' '; width]; height];
    let step = (values.len().max(width) as f64 / width as f64).max(1.0);

    for x in 0..width {
        let idx = ((x as f64 * step) as usize).min(values.len() - 1);
        let v = values[idx];
        let y = ((v - min) / (max - min + 1e-6) * ((height - 1) as f64)).round() as usize;
        let row = height - 1 - y.min(height - 1);
        canvas[row][x] = '*';
    }

    for row in canvas {
        let line: String = row.into_iter().collect();
        writeln!(out, "{}", line)?;
    }
    writeln!(out, "min:{:.0} rpm  max:{:.0} rpm", min, max)?;
    Ok(())
}

fn draw_xy_plot<W: Write>(
    out: &mut W,
    points: &[(f64, f64)],
    width: usize,
    height: usize,
) -> std::io::Result<()> {
    if points.is_empty() {
        return Ok(());
    }

    let (v_min, v_max) = points
        .iter()
        .fold((f64::INFINITY, f64::NEG_INFINITY), |(mn, mx), (v, _)| {
            (mn.min(*v), mx.max(*v))
        });
    let (p_min, p_max) = points
        .iter()
        .fold((f64::INFINITY, f64::NEG_INFINITY), |(mn, mx), (_, p)| {
            (mn.min(*p), mx.max(*p))
        });

    let mut canvas = vec![vec![' '; width]; height];
    for (v, p) in points {
        let x = (((v - v_min) / (v_max - v_min + 1e-12)) * ((width - 1) as f64)).round() as usize;
        let y = (((p - p_min) / (p_max - p_min + 1e-12)) * ((height - 1) as f64)).round() as usize;
        let row = height - 1 - y.min(height - 1);
        let col = x.min(width - 1);
        canvas[row][col] = 'o';
    }

    for row in canvas {
        let line: String = row.into_iter().collect();
        writeln!(out, "{}", line)?;
    }
    writeln!(
        out,
        "Vn [{:.2}..{:.2}]  p [{:.0}..{:.0}] Pa",
        v_min, v_max, p_min, p_max
    )?;
    Ok(())
}

fn flag(on: bool) -> &'static str {
    if on { "ON" } else { "OFF" }
}

fn set_raw_mode(enable: bool) {
    let cmd = if enable {
        "stty -icanon -echo min 0 time 0"
    } else {
        "stty sane"
    };
    let _ = Command::new("sh").arg("-c").arg(cmd).status();
}

fn handle_key(sim: &mut Simulator, b: u8) -> bool {
    match b as char {
        'q' => return true,
        's' => sim.control.starter_on = !sim.control.starter_on,
        'i' => sim.control.spark_on = !sim.control.spark_on,
        'f' => sim.control.fuel_on = !sim.control.fuel_on,
        'w' => sim.control.throttle = (sim.control.throttle + 0.01).clamp(0.0, 1.0),
        'x' => sim.control.throttle = (sim.control.throttle - 0.01).clamp(0.0, 1.0),
        'a' => sim.control.vvt_intake_deg = (sim.control.vvt_intake_deg + 1.0).clamp(-40.0, 40.0),
        'z' => sim.control.vvt_intake_deg = (sim.control.vvt_intake_deg - 1.0).clamp(-40.0, 40.0),
        'k' => sim.control.vvt_exhaust_deg = (sim.control.vvt_exhaust_deg + 1.0).clamp(-40.0, 40.0),
        'm' => sim.control.vvt_exhaust_deg = (sim.control.vvt_exhaust_deg - 1.0).clamp(-40.0, 40.0),
        _ => {}
    }
    false
}

fn main() -> std::io::Result<()> {
    set_raw_mode(true);
    print!("\x1b[?1049h\x1b[?25l");
    stdout().flush()?;

    let mut sim = Simulator::new();
    let mut last_tick = Instant::now();
    let mut input = stdin();
    let mut buf = [0u8; 64];

    let run_result = 'mainloop: loop {
        match input.read(&mut buf) {
            Ok(n) if n > 0 => {
                for b in &buf[..n] {
                    if handle_key(&mut sim, *b) {
                        break 'mainloop Ok(());
                    }
                }
            }
            Ok(_) => {}
            Err(_) => {}
        }

        let now = Instant::now();
        let elapsed = now.duration_since(last_tick);
        if elapsed.as_secs_f64() >= DT {
            last_tick = now;
            let obs = sim.step(DT);
            draw_ui(&sim, &obs)?;
        } else {
            std::thread::sleep(Duration::from_millis(1));
        }
    };

    print!("\x1b[?25h\x1b[?1049l");
    stdout().flush()?;
    set_raw_mode(false);

    run_result
}
