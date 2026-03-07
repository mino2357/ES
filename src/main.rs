use std::collections::VecDeque;
use std::f32::consts::PI as PI_F32;
use std::f64::consts::PI;
use std::sync::{Arc, Mutex};
use std::time::Duration;
use std::time::Instant;

use eframe::egui;
use egui_plot::{Line, Plot, PlotPoints};
use rodio::{OutputStream, Sink, Source};

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

#[derive(Debug, Clone)]
struct Observation {
    rpm: f64,
    map_kpa: f64,
    torque_combustion_nm: f64,
    torque_friction_nm: f64,
    torque_starter_nm: f64,
    air_flow_gps: f64,
    trapped_air_mg: f64,
    imep_bar: f64,
    wiebe_phase_deg: f64,
    wiebe_burn_rate: f64,
    stable_idle: bool,
    exhaust_kpa: f64,
    exhaust_pulse_norm: f64,
    pv_points: Vec<(f64, f64)>,
}

#[derive(Debug, Clone, Copy)]
struct AudioParams {
    rpm: f32,
    throttle: f32,
    torque_norm: f32,
    starter_on: bool,
    running: bool,
    cylinders: usize,
    exhaust_pressure_norm: f32,
    exhaust_pulse_norm: f32,
}

impl Default for AudioParams {
    fn default() -> Self {
        Self {
            rpm: 0.0,
            throttle: 0.0,
            torque_norm: 0.0,
            starter_on: false,
            running: false,
            cylinders: 4,
            exhaust_pressure_norm: 0.0,
            exhaust_pulse_norm: 0.0,
        }
    }
}

struct ExhaustSynth {
    sample_rate: u32,
    phase: f32,
    note_phase: f32,
    pulse_env: f32,
    lp_state: f32,
    hp_state: f32,
    hp_prev_in: f32,
    noise_state: u32,
    frame_count: u32,
    cached: AudioParams,
    shared: Arc<Mutex<AudioParams>>,
}

impl ExhaustSynth {
    fn new(shared: Arc<Mutex<AudioParams>>, sample_rate: u32) -> Self {
        Self {
            sample_rate,
            phase: 0.0,
            note_phase: 0.0,
            pulse_env: 0.0,
            lp_state: 0.0,
            hp_state: 0.0,
            hp_prev_in: 0.0,
            noise_state: 0x1234_5678,
            frame_count: 0,
            cached: AudioParams::default(),
            shared,
        }
    }

    fn next_noise(&mut self) -> f32 {
        self.noise_state = self
            .noise_state
            .wrapping_mul(1_664_525)
            .wrapping_add(1_013_904_223);
        let x = ((self.noise_state >> 8) & 0xFFFF) as f32 / 65535.0;
        x * 2.0 - 1.0
    }
}

impl Iterator for ExhaustSynth {
    type Item = f32;

    fn next(&mut self) -> Option<Self::Item> {
        if self.frame_count % 256 == 0 {
            if let Ok(v) = self.shared.lock() {
                self.cached = *v;
            }
        }
        self.frame_count = self.frame_count.wrapping_add(1);

        let sr = self.sample_rate as f32;
        let rpm = self.cached.rpm.max(0.0);
        let cyl = self.cached.cylinders.max(1) as f32;

        let firing_hz = (rpm / 120.0) * cyl;
        let fundamental_hz = (rpm / 60.0) * (cyl * 0.5);

        self.phase += firing_hz / sr;
        if self.phase >= 1.0 {
            self.phase -= 1.0;
            let hit_strength = 0.12
                + self.cached.throttle * 0.45
                + self.cached.torque_norm * 0.35
                + self.cached.exhaust_pulse_norm * 0.55;
            self.pulse_env += hit_strength;
        }

        self.pulse_env *= 0.994;

        self.note_phase += 2.0 * PI_F32 * (fundamental_hz.max(20.0) / sr);
        if self.note_phase > 2.0 * PI_F32 {
            self.note_phase -= 2.0 * PI_F32;
        }

        let tone = self.note_phase.sin()
            + 0.45 * (2.0 * self.note_phase).sin()
            + 0.2 * (3.0 * self.note_phase).sin();
        let crackle = self.next_noise()
            * (0.15 + self.cached.throttle * 0.85)
            * (0.4 + self.cached.exhaust_pressure_norm * 0.8)
            * self.pulse_env.min(1.4);

        let mut raw = tone * self.pulse_env + crackle * 0.35;

        if self.cached.starter_on && rpm < 320.0 {
            let starter_phase = self.frame_count as f32 * 2.0 * PI_F32 * 42.0 / sr;
            raw += 0.12 * starter_phase.sin();
        }

        let lp_alpha = (0.14 + 0.18 * self.cached.exhaust_pressure_norm).clamp(0.08, 0.38);
        self.lp_state += lp_alpha * (raw - self.lp_state);
        let hp_alpha = 0.985;
        self.hp_state = hp_alpha * (self.hp_state + self.lp_state - self.hp_prev_in);
        self.hp_prev_in = self.lp_state;

        let mut out = self.hp_state;
        if !self.cached.running && !self.cached.starter_on {
            out *= 0.05;
        }

        let gain = 0.12
            + self.cached.throttle * 0.22
            + self.cached.torque_norm * 0.20
            + self.cached.exhaust_pressure_norm * 0.24
            + self.cached.exhaust_pulse_norm * 0.12;
        Some((out * gain).clamp(-0.95, 0.95))
    }
}

impl Source for ExhaustSynth {
    fn current_frame_len(&self) -> Option<usize> {
        None
    }

    fn channels(&self) -> u16 {
        1
    }

    fn sample_rate(&self) -> u32 {
        self.sample_rate
    }

    fn total_duration(&self) -> Option<Duration> {
        None
    }
}

struct AudioEngine {
    _stream: OutputStream,
    sink: Sink,
    shared: Arc<Mutex<AudioParams>>,
}

impl AudioEngine {
    fn new() -> Option<Self> {
        let (stream, handle) = OutputStream::try_default().ok()?;
        let sink = Sink::try_new(&handle).ok()?;
        let sample_rate = 48_000;
        let shared = Arc::new(Mutex::new(AudioParams::default()));
        sink.append(ExhaustSynth::new(shared.clone(), sample_rate));
        sink.play();
        Some(Self {
            _stream: stream,
            sink,
            shared,
        })
    }

    fn update(&self, params: AudioParams) {
        if let Ok(mut p) = self.shared.lock() {
            *p = params;
        }
    }
}

impl Drop for AudioEngine {
    fn drop(&mut self) {
        self.sink.stop();
    }
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
            history_rpm: VecDeque::with_capacity(500),
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

        let cycle_angle_deg = self.engine.crank_angle_rad.to_degrees();
        let wiebe_burn_rate = wiebe_combustion_rate(
            cycle_angle_deg,
            self.params.cylinders,
            365.0 + self.control.vvt_intake_deg * 0.2,
            70.0,
            5.0,
            2.0,
        );

        let torque_combustion = if combustion_enabled {
            self.engine.running = true;
            indicated_torque(
                trapped_air,
                self.control.throttle,
                rpm,
                self.params.cylinders,
                wiebe_burn_rate,
            )
        } else if self.engine.running && self.control.fuel_on && self.control.spark_on {
            indicated_torque(
                trapped_air,
                self.control.throttle,
                rpm,
                self.params.cylinders,
                wiebe_burn_rate,
            ) * 0.65
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

        let engine_load = (m_dot_engine / 0.11).clamp(0.0, 2.0);
        let combustion_pulse = wiebe_burn_rate.clamp(0.0, 6.0) / 6.0;
        let exhaust_target_pa = P_EXHAUST
            + 18_000.0 * engine_load
            + 22_000.0 * combustion_pulse
            + 10_000.0 * self.control.throttle;
        let relax = if self.engine.running { 0.18 } else { 0.05 };
        self.exhaust.pressure_pa += (exhaust_target_pa - self.exhaust.pressure_pa) * relax;
        self.exhaust.pressure_pa = self
            .exhaust
            .pressure_pa
            .clamp(P_EXHAUST * 0.90, P_EXHAUST + 60_000.0);

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
            wiebe_phase_deg: cycle_angle_deg,
            wiebe_burn_rate,
            stable_idle: self.engine.running
                && (rpm - IDLE_TARGET_RPM).abs() < 120.0
                && self.control.throttle < 0.18,
            exhaust_kpa: self.exhaust.pressure_pa * 1e-3,
            exhaust_pulse_norm: combustion_pulse,
            pv_points: pv_diagram(
                self.params.compression_ratio,
                self.intake.pressure_pa,
                self.exhaust.pressure_pa,
            ),
        }
    }
}

struct DashboardApp {
    sim: Simulator,
    latest: Observation,
    last_tick: Instant,
    audio: Option<AudioEngine>,
}

impl DashboardApp {
    fn new() -> Self {
        let mut sim = Simulator::new();
        let latest = sim.step(DT);
        Self {
            sim,
            latest,
            last_tick: Instant::now(),
            audio: AudioEngine::new(),
        }
    }

    fn apply_shortcuts(&mut self, ctx: &egui::Context) {
        ctx.input(|i| {
            if i.key_pressed(egui::Key::Q) {
                ctx.send_viewport_cmd(egui::ViewportCommand::Close);
            }
            if i.key_pressed(egui::Key::S) {
                self.sim.control.starter_on = !self.sim.control.starter_on;
            }
            if i.key_pressed(egui::Key::I) {
                self.sim.control.spark_on = !self.sim.control.spark_on;
            }
            if i.key_pressed(egui::Key::F) {
                self.sim.control.fuel_on = !self.sim.control.fuel_on;
            }
            if i.key_pressed(egui::Key::W) {
                self.sim.control.throttle = (self.sim.control.throttle + 0.01).clamp(0.0, 1.0);
            }
            if i.key_pressed(egui::Key::X) {
                self.sim.control.throttle = (self.sim.control.throttle - 0.01).clamp(0.0, 1.0);
            }
        });
    }

    fn advance_simulation(&mut self) {
        let now = Instant::now();
        let mut steps = ((now - self.last_tick).as_secs_f64() / DT).floor() as usize;
        steps = steps.clamp(1, 16);

        for _ in 0..steps {
            self.latest = self.sim.step(DT);
        }
        self.last_tick = now;

        if let Some(audio) = &self.audio {
            let torque_norm = (self.latest.torque_combustion_nm / 140.0).clamp(0.0, 1.0) as f32;
            audio.update(AudioParams {
                rpm: self.latest.rpm as f32,
                throttle: self.sim.control.throttle as f32,
                torque_norm,
                starter_on: self.sim.control.starter_on,
                running: self.sim.engine.running,
                cylinders: self.sim.params.cylinders,
                exhaust_pressure_norm: ((self.latest.exhaust_kpa - 101.325) / 60.0).clamp(0.0, 1.0)
                    as f32,
                exhaust_pulse_norm: self.latest.exhaust_pulse_norm as f32,
            });
        }
    }
}

impl eframe::App for DashboardApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        self.apply_shortcuts(ctx);
        self.advance_simulation();

        egui::TopBottomPanel::top("header").show(ctx, |ui| {
            ui.heading("4-stroke Engine Start / Idle Realtime Dashboard");
            ui.label("Shortcut: [Q] quit [S] starter [I] spark [F] fuel [W/X] throttle ±");
        });

        egui::SidePanel::left("controls").show(ctx, |ui| {
            ui.heading("Controls");
            ui.add(egui::Slider::new(&mut self.sim.control.throttle, 0.0..=1.0).text("Throttle"));
            ui.checkbox(&mut self.sim.control.starter_on, "Starter");
            ui.checkbox(&mut self.sim.control.spark_on, "Spark");
            ui.checkbox(&mut self.sim.control.fuel_on, "Fuel");
            ui.add(
                egui::Slider::new(&mut self.sim.control.vvt_intake_deg, -40.0..=40.0)
                    .text("VVT Intake [deg]"),
            );
            ui.add(
                egui::Slider::new(&mut self.sim.control.vvt_exhaust_deg, -40.0..=40.0)
                    .text("VVT Exhaust [deg]"),
            );
            ui.separator();
            ui.horizontal_wrapped(|ui| {
                for cyl in [2, 3, 4, 6, 8] {
                    ui.selectable_value(&mut self.sim.params.cylinders, cyl, format!("{cyl} cyl"));
                }
            });
            ui.separator();
            ui.label(if self.latest.stable_idle {
                "State: IDLE STABLE"
            } else {
                "State: TRANSIENT"
            });
            ui.label(if self.audio.is_some() {
                "Audio: ON (synthetic exhaust)"
            } else {
                "Audio: OFF (device unavailable)"
            });
        });

        egui::CentralPanel::default().show(ctx, |ui| {
            ui.columns(2, |columns| {
                columns[0].group(|ui| {
                    ui.label(format!("RPM: {:.1}", self.latest.rpm));
                    ui.label(format!("MAP: {:.1} kPa", self.latest.map_kpa));
                    ui.label(format!("Air flow: {:.2} g/s", self.latest.air_flow_gps));
                    ui.label(format!(
                        "Trapped air: {:.2} mg/cyl",
                        self.latest.trapped_air_mg
                    ));
                    ui.label(format!(
                        "Torque (comb/starter/friction): {:.2} / {:.2} / {:.2} Nm",
                        self.latest.torque_combustion_nm,
                        self.latest.torque_starter_nm,
                        self.latest.torque_friction_nm
                    ));
                    ui.label(format!("IMEP: {:.2} bar", self.latest.imep_bar));
                    ui.label(format!(
                        "Exhaust pressure: {:.1} kPa",
                        self.latest.exhaust_kpa
                    ));
                    ui.label(format!(
                        "Wiebe phase/rate: {:.1} deg / {:.2}",
                        self.latest.wiebe_phase_deg, self.latest.wiebe_burn_rate
                    ));
                });

                columns[1].group(|ui| {
                    let rpm_points: PlotPoints<'_> = self
                        .sim
                        .history_rpm
                        .iter()
                        .enumerate()
                        .map(|(i, rpm)| [i as f64, *rpm])
                        .collect();
                    let rpm_line = Line::new("RPM", rpm_points).color(egui::Color32::LIGHT_GREEN);
                    Plot::new("rpm_plot")
                        .height(200.0)
                        .allow_scroll(false)
                        .allow_drag(false)
                        .show(ui, |plot_ui| {
                            plot_ui.line(rpm_line);
                        });

                    let pv_points: PlotPoints<'_> = self
                        .latest
                        .pv_points
                        .iter()
                        .map(|(v, p)| [*v, *p * 1e-3])
                        .collect();
                    let pv_line = Line::new("p-V", pv_points)
                        .color(egui::Color32::from_rgb(255, 170, 40))
                        .width(2.0);
                    Plot::new("pv_plot")
                        .height(200.0)
                        .allow_scroll(false)
                        .allow_drag(false)
                        .show(ui, |plot_ui| {
                            plot_ui.line(pv_line);
                        });
                });
            });
        });

        ctx.request_repaint();
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

fn indicated_torque(
    trapped_air_kg: f64,
    throttle: f64,
    rpm: f64,
    cylinders: usize,
    wiebe_burn_rate: f64,
) -> f64 {
    let lambda_eff = (0.7 + 0.6 * throttle).clamp(0.65, 1.15);
    let lhv = 43e6;
    let afr = 14.7 / lambda_eff;
    let fuel_mass = trapped_air_kg / afr;
    let eta = (0.2 + 0.13 * throttle - 0.00002 * (rpm - 2500.0).abs()).clamp(0.12, 0.35);
    let work_per_cyl_cycle = fuel_mass * lhv * eta;
    let total_work = work_per_cyl_cycle * cylinders as f64;
    let mean_torque = total_work / (4.0 * PI);
    mean_torque * wiebe_burn_rate.clamp(0.0, 6.0)
}

fn wiebe_combustion_rate(
    cycle_angle_deg: f64,
    cylinders: usize,
    start_deg: f64,
    duration_deg: f64,
    a: f64,
    m: f64,
) -> f64 {
    let mut dxb_sum = 0.0;
    let firing_interval = 720.0 / cylinders as f64;

    for cyl in 0..cylinders {
        let phase_offset = cyl as f64 * firing_interval;
        let mut theta = cycle_angle_deg - phase_offset;
        while theta < 0.0 {
            theta += 720.0;
        }
        while theta >= 720.0 {
            theta -= 720.0;
        }

        let x = (theta - start_deg) / duration_deg;
        if (0.0..=1.0).contains(&x) {
            let dxb_dtheta =
                a * (m + 1.0) / duration_deg * x.powf(m) * (-a * x.powf(m + 1.0)).exp();
            dxb_sum += dxb_dtheta;
        }
    }

    let mean_rate = cylinders as f64 / 720.0;
    if mean_rate > 0.0 {
        dxb_sum / mean_rate
    } else {
        0.0
    }
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

fn main() -> eframe::Result {
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default().with_inner_size([1200.0, 760.0]),
        ..Default::default()
    };

    eframe::run_native(
        "ES Simulator Dashboard",
        options,
        Box::new(|_cc| Ok(Box::new(DashboardApp::new()))),
    )
}
