mod theme;
mod widgets;

use std::time::{Duration, Instant};

use eframe::egui;
use egui_plot::{Legend, Line, Plot, PlotBounds, PlotPoints, VLine};

use self::theme::DashboardTheme;
use self::widgets::{
    GaugeSpec, LinearMeterSpec, annunciator, digital_readout, gauge, linear_meter, metric_row,
    monitor_heading, section_label,
};
use crate::config::{AppConfig, ExternalLoadMode, UiConfig};
use crate::constants::FIXED_CYLINDER_COUNT;
use crate::simulator::{
    CycleHistorySample, Simulator, accuracy_priority_dt, cam_lift_mm, cam_profile_points,
    estimate_realtime_performance, external_load_available_torque_nm,
    external_load_command_for_torque_nm, external_load_power_limit_torque_nm,
    external_load_reflected_inertia_kgm2, external_load_speed_limit_active,
    external_load_vehicle_speed_kph, rad_s_to_rpm, rpm_linked_dt, shaft_power_hp, shaft_power_kw,
};

#[derive(Debug, Clone, Copy)]
struct GraphLayoutHeights {
    standard_plot_px: f32,
    pv_plot_px: f32,
    section_spacing_px: f32,
}

fn graph_layout_heights(ui_config: &UiConfig) -> GraphLayoutHeights {
    let scale = 0.84;
    let standard_plot_px = (ui_config.plot_height_px * scale).max(72.0);
    let pv_plot_px = (ui_config.pv_plot_height_px * scale).max(standard_plot_px * 1.55);

    GraphLayoutHeights {
        standard_plot_px,
        pv_plot_px,
        section_spacing_px: 4.0,
    }
}

fn responsive_card_columns(available_width: f32, min_card_width: f32, max_columns: usize) -> usize {
    let spacing = 8.0;
    let fitted = ((available_width + spacing) / (min_card_width + spacing)).floor() as usize;
    fitted.clamp(1, max_columns.max(1))
}

fn responsive_card_width(available_width: f32, columns: usize) -> f32 {
    let spacing = 8.0;
    let total_spacing = spacing * columns.saturating_sub(1) as f32;
    ((available_width - total_spacing) / columns.max(1) as f32).max(96.0)
}

fn recent_cycle_plot_lines(
    history: &std::collections::VecDeque<CycleHistorySample>,
    recent_cycles: usize,
) -> Vec<Vec<[f64; 2]>> {
    let Some(latest_cycle) = history.back().map(|sample| sample.cycle) else {
        return Vec::new();
    };
    // Keep the requested number of completed cycles plus the currently accumulating cycle so
    // the plot does not visually "reset" before the next 720 deg trace has been drawn.
    let min_cycle = latest_cycle.saturating_sub(recent_cycles.max(1) as u64);
    let mut lines = Vec::new();
    let mut current_cycle = None;
    let mut current_points: Vec<[f64; 2]> = Vec::new();

    for sample in history.iter().filter(|sample| sample.cycle >= min_cycle) {
        if current_cycle != Some(sample.cycle) {
            if !current_points.is_empty() {
                lines.push(std::mem::take(&mut current_points));
            }
            current_cycle = Some(sample.cycle);
        }
        current_points.push([sample.cycle_deg, sample.value]);
    }
    if !current_points.is_empty() {
        lines.push(current_points);
    }
    lines
}

fn recent_cycle_min_max(
    history: &std::collections::VecDeque<CycleHistorySample>,
    recent_cycles: usize,
) -> Option<(f64, f64)> {
    let latest_cycle = history.back()?.cycle;
    let min_cycle = latest_cycle.saturating_sub(recent_cycles.max(1) as u64);

    history
        .iter()
        .filter(|sample| sample.cycle >= min_cycle)
        .map(|sample| sample.value)
        .fold(None, |acc: Option<(f64, f64)>, value| match acc {
            Some((mn, mx)) => Some((mn.min(value), mx.max(value))),
            None => Some((value, value)),
        })
}

fn faded_cycle_color(base: egui::Color32, index: usize, total: usize) -> egui::Color32 {
    let fade = if total <= 1 {
        1.0
    } else {
        0.35 + 0.65 * ((index + 1) as f32 / total as f32)
    };
    egui::Color32::from_rgba_unmultiplied(
        base.r(),
        base.g(),
        base.b(),
        (255.0 * fade).round().clamp(0.0, 255.0) as u8,
    )
}

fn adaptive_plot_y_range(
    min_max: Option<(f64, f64)>,
    min_span: f64,
    margin_ratio: f64,
    clamp_min: Option<f64>,
) -> (f64, f64) {
    let min_span = min_span.max(f64::EPSILON);
    let (data_min, data_max) = min_max.unwrap_or((0.0, 0.0));
    let span = (data_max - data_min).max(min_span);
    let pad = span * margin_ratio.max(0.0);
    let center = 0.5 * (data_min + data_max);
    let half_range = 0.5 * span + pad;

    let mut y_min = center - half_range;
    let mut y_max = center + half_range;

    if let Some(lower_floor) = clamp_min {
        if y_min < lower_floor {
            let shift = lower_floor - y_min;
            y_min = lower_floor;
            y_max += shift;
        }
    }

    if y_max - y_min < min_span {
        let grow = 0.5 * (min_span - (y_max - y_min));
        y_min -= grow;
        y_max += grow;
        if let Some(lower_floor) = clamp_min {
            if y_min < lower_floor {
                let shift = lower_floor - y_min;
                y_min = lower_floor;
                y_max += shift;
            }
        }
    }

    (y_min, y_max.max(y_min + min_span))
}

fn combine_min_max(a: Option<(f64, f64)>, b: Option<(f64, f64)>) -> Option<(f64, f64)> {
    match (a, b) {
        (Some((a_min, a_max)), Some((b_min, b_max))) => Some((a_min.min(b_min), a_max.max(b_max))),
        (Some(bounds), None) | (None, Some(bounds)) => Some(bounds),
        (None, None) => None,
    }
}

fn lerp_color(a: egui::Color32, b: egui::Color32, t: f32) -> egui::Color32 {
    let t = t.clamp(0.0, 1.0);
    let lerp_channel = |lhs: u8, rhs: u8| -> u8 {
        (lhs as f32 + (rhs as f32 - lhs as f32) * t)
            .round()
            .clamp(0.0, 255.0) as u8
    };
    egui::Color32::from_rgba_unmultiplied(
        lerp_channel(a.r(), b.r()),
        lerp_channel(a.g(), b.g()),
        lerp_channel(a.b(), b.b()),
        lerp_channel(a.a(), b.a()),
    )
}

fn tent_roof_y(
    x: f32,
    bore_left: f32,
    bore_right: f32,
    chamber_apex: egui::Pos2,
    deck_y: f32,
) -> f32 {
    if x <= chamber_apex.x {
        let t = ((x - bore_left) / (chamber_apex.x - bore_left).max(f32::EPSILON)).clamp(0.0, 1.0);
        egui::lerp(deck_y..=chamber_apex.y, t)
    } else {
        let t = ((x - chamber_apex.x) / (bore_right - chamber_apex.x).max(f32::EPSILON))
            .clamp(0.0, 1.0);
        egui::lerp(chamber_apex.y..=deck_y, t)
    }
}

fn flame_front_points(
    spark_pos: egui::Pos2,
    radius_x: f32,
    radius_y: f32,
    bore_left: f32,
    bore_right: f32,
    chamber_apex: egui::Pos2,
    deck_y: f32,
    piston_top_y: f32,
) -> Vec<egui::Pos2> {
    let mut points = Vec::with_capacity(49);
    let x_min = bore_left + 6.0;
    let x_max = bore_right - 6.0;
    let y_bottom = (piston_top_y - 6.0).max(spark_pos.y + 8.0);
    for step in 0..=48 {
        let angle = std::f32::consts::TAU * step as f32 / 48.0;
        let mut x = spark_pos.x + radius_x * angle.cos();
        x = x.clamp(x_min, x_max);
        let roof_y = tent_roof_y(x, bore_left, bore_right, chamber_apex, deck_y) + 5.0;
        let local_y = radius_y * angle.sin();
        let stretch = if local_y < 0.0 { 0.64 } else { 1.08 };
        let mut y = spark_pos.y + local_y * stretch;
        y = y.clamp(roof_y, y_bottom);
        points.push(egui::pos2(x, y));
    }
    points
}

fn cylinder_trace_color(index: usize) -> egui::Color32 {
    const COLORS: [egui::Color32; 4] = [
        egui::Color32::from_rgb(255, 170, 40),
        egui::Color32::from_rgb(90, 180, 255),
        egui::Color32::from_rgb(110, 220, 150),
        egui::Color32::from_rgb(255, 105, 125),
    ];
    COLORS[index % COLORS.len()]
}

fn wrap_cycle_deg(value: f64) -> f64 {
    value.rem_euclid(720.0)
}

fn shortest_cycle_delta_deg(from: f64, to: f64) -> f64 {
    let wrapped = (to - from + 360.0).rem_euclid(720.0) - 360.0;
    if wrapped <= -360.0 {
        wrapped + 720.0
    } else {
        wrapped
    }
}

fn current_stroke_label(cycle_deg: f64) -> &'static str {
    match wrap_cycle_deg(cycle_deg) {
        deg if deg < 180.0 => "intake stroke",
        deg if deg < 360.0 => "compression stroke",
        deg if deg < 540.0 => "expansion stroke",
        _ => "exhaust stroke",
    }
}

fn speed_hold_desired_net_torque_nm(error_rpm: f64, integral_state: f64) -> f64 {
    const KP_NM_PER_RPM: f64 = 0.26;
    const KI_NM_PER_RPM_S: f64 = 0.06;
    KP_NM_PER_RPM * error_rpm + KI_NM_PER_RPM_S * integral_state
}

fn show_collapsible_module(
    ui: &mut egui::Ui,
    theme: DashboardTheme,
    id_source: &'static str,
    title: &str,
    subtitle: &str,
    accent: egui::Color32,
    default_open: bool,
    body: impl FnOnce(&mut egui::Ui),
) {
    theme.monitor_frame().show(ui, |ui| {
        egui::CollapsingHeader::new(
            egui::RichText::new(title)
                .color(theme.text_main)
                .strong()
                .size(23.0 / 1.5),
        )
        .id_salt(id_source)
        .default_open(default_open)
        .show(ui, |ui| {
            ui.label(
                egui::RichText::new(subtitle)
                    .color(accent)
                    .family(egui::FontFamily::Monospace)
                    .size(11.0),
            );
            ui.add_space(4.0);
            body(ui);
        });
    });
}

// The dashboard owns real-time pacing, user controls, and fixed-range plots.
struct DashboardApp {
    sim: Simulator,
    theme: DashboardTheme,
    ui_config: UiConfig,
    latest: crate::simulator::Observation,
    last_tick: Instant,
    dt_base: f64,
    dt_next: f64,
    dt_min_bound: f64,
    dt_max_bound: f64,
    realtime_fixed_dt_s: Option<f64>,
    load_target_rpm: f64,
    load_speed_integral: f64,
    required_brake_torque_nm: f64,
    schematic_cycle_deg: f64,
    bench_estop: bool,
    bench_dyno_enabled: bool,
    bench_cell_vent: bool,
    bench_coolant_conditioning: bool,
}

impl DashboardApp {
    fn new(config: AppConfig) -> Self {
        let theme = DashboardTheme::default();
        let ui_config = config.ui.clone();
        let dt = config.environment.dt.max(ui_config.min_base_dt_s);
        let (dt_min_bound, dt_max_bound, realtime_fixed_dt_s, dt_next) = if ui_config
            .sync_to_wall_clock
        {
            // Benchmark a safe realtime floor once at startup and keep fixed dt when headroom is ample.
            let realtime_perf = estimate_realtime_performance(&config, dt);
            let dt_min_bound = realtime_perf
                .floor_dt_s
                .max(dt * ui_config.realtime_dt_min_factor);
            let dt_max_bound = (dt * ui_config.realtime_dt_max_factor)
                .max(dt_min_bound * ui_config.realtime_dt_max_over_min_factor);
            let realtime_fixed_dt_s = (realtime_perf.fixed_dt_headroom_ratio
                >= config.numerics.realtime_fixed_dt_headroom_ratio)
                .then_some(
                    realtime_perf
                        .fixed_dt_candidate_s
                        .clamp(dt_min_bound, dt_max_bound),
                );
            let dt_next =
                realtime_fixed_dt_s.unwrap_or_else(|| dt.clamp(dt_min_bound, dt_max_bound));
            (dt_min_bound, dt_max_bound, realtime_fixed_dt_s, dt_next)
        } else {
            let dt_min_bound = config
                .numerics
                .rpm_link_dt_min_floor_s
                .max(ui_config.dt_epsilon_s);
            let dt_max_bound = config.numerics.accuracy_dt_max_s.max(dt_min_bound);
            let dt_next = accuracy_priority_dt(config.engine.default_target_rpm, &config.numerics)
                .clamp(dt_min_bound, dt_max_bound);
            (dt_min_bound, dt_max_bound, None, dt_next)
        };
        let mut sim = Simulator::new(&config);
        let latest = sim.step(dt_next);
        let required_brake_torque_nm = latest.torque_net_nm + latest.torque_load_nm;
        let schematic_cycle_deg = wrap_cycle_deg(latest.cycle_deg);
        Self {
            sim,
            theme,
            ui_config,
            latest,
            last_tick: Instant::now(),
            dt_base: dt,
            dt_next,
            dt_min_bound,
            dt_max_bound,
            realtime_fixed_dt_s,
            load_target_rpm: config.engine.default_target_rpm,
            load_speed_integral: 0.0,
            required_brake_torque_nm,
            schematic_cycle_deg,
            bench_estop: false,
            bench_dyno_enabled: true,
            bench_cell_vent: true,
            bench_coolant_conditioning: true,
        }
    }

    fn sync_required_brake_torque(&mut self) {
        let combustion_ready = self.sim.control.spark_cmd
            && self.sim.control.fuel_cmd
            && self.latest.rpm > self.sim.model.combustion_enable_rpm_min
            && self.latest.map_kpa * 1.0e3
                > self.sim.model.combustion_enable_intake_pressure_min_pa;
        let brake_output_nm = self.latest.torque_net_nm + self.latest.torque_load_nm;
        self.required_brake_torque_nm = if combustion_ready {
            brake_output_nm.max(0.0)
        } else {
            0.0
        };
    }

    fn rpm_error(&self) -> f64 {
        self.load_target_rpm - self.latest.rpm
    }

    fn shaft_torque_estimate_nm(&self) -> f64 {
        self.latest.torque_combustion_cycle_nm
            - self.latest.torque_friction_nm
            - self.latest.torque_pumping_nm
    }

    fn bench_interlock_ok(&self) -> bool {
        self.bench_dyno_enabled
            && self.bench_cell_vent
            && self.bench_coolant_conditioning
            && !self.bench_estop
    }

    fn bench_firing_permitted(&self) -> bool {
        self.bench_cell_vent && self.bench_coolant_conditioning && !self.bench_estop
    }

    fn dyno_available_torque_nm(&self) -> f64 {
        external_load_available_torque_nm(self.sim.state.omega_rad_s, &self.sim.model.external_load)
    }

    fn dyno_power_limit_torque_nm(&self) -> f64 {
        external_load_power_limit_torque_nm(
            self.sim.state.omega_rad_s,
            &self.sim.model.external_load,
        )
    }

    fn dyno_power_limit_active(&self) -> bool {
        let applied_power_kw = shaft_power_kw(self.latest.rpm, self.latest.torque_load_nm.abs());
        applied_power_kw >= self.sim.model.external_load.absorber_power_limit_kw * 0.98
    }

    fn apply_bench_console_interlocks(&mut self) {
        if self.bench_estop {
            self.sim.control.throttle_cmd = 0.0;
            self.sim.control.spark_cmd = false;
            self.sim.control.fuel_cmd = false;
            self.load_target_rpm = 0.0;
            self.load_speed_integral = 0.0;
        } else if !self.bench_firing_permitted() {
            self.sim.control.spark_cmd = false;
            self.sim.control.fuel_cmd = false;
        }
        if !self.bench_dyno_enabled {
            self.sim.control.load_cmd = 0.0;
            self.load_speed_integral = 0.0;
        }
    }

    fn update_schematic_phase(&mut self, wall_dt_s: f64) {
        let running = self.sim.state.running || self.latest.rpm > 120.0;
        let target_phase = wrap_cycle_deg(self.latest.cycle_deg);
        if !running {
            self.schematic_cycle_deg = target_phase;
            return;
        }

        let align_alpha = 1.0 - (-7.5 * wall_dt_s.max(1.0 / 240.0)).exp();
        let delta = shortest_cycle_delta_deg(self.schematic_cycle_deg, target_phase);
        self.schematic_cycle_deg = wrap_cycle_deg(self.schematic_cycle_deg + delta * align_alpha);
    }

    fn apply_load_input(&mut self, dt: f64) {
        if !self.bench_dyno_enabled || self.bench_estop {
            self.sim.control.load_cmd = 0.0;
            return;
        }
        let rpm = rad_s_to_rpm(self.sim.state.omega_rad_s.max(0.0));
        let error_rpm = self.load_target_rpm - rpm;
        self.load_speed_integral =
            (self.load_speed_integral + error_rpm * dt).clamp(-6_000.0, 6_000.0);
        let desired_net_torque_nm =
            speed_hold_desired_net_torque_nm(error_rpm, self.load_speed_integral);
        let target_machine_torque_nm = (self.shaft_torque_estimate_nm() - desired_net_torque_nm)
            .clamp(
                self.sim.model.external_load.torque_min_nm,
                self.sim.model.external_load.torque_max_nm,
            );
        self.sim.control.load_cmd = external_load_command_for_torque_nm(
            target_machine_torque_nm,
            self.sim.state.omega_rad_s,
            &self.sim.model.external_load,
        );
    }

    fn apply_shortcuts(&mut self, ctx: &egui::Context) {
        ctx.input(|i| {
            if i.key_pressed(egui::Key::Q) {
                ctx.send_viewport_cmd(egui::ViewportCommand::Close);
            }
            if i.key_pressed(egui::Key::I) {
                self.sim.control.spark_cmd = !self.sim.control.spark_cmd;
            }
            if i.key_pressed(egui::Key::F) {
                self.sim.control.fuel_cmd = !self.sim.control.fuel_cmd;
            }
            if i.key_pressed(egui::Key::E) {
                self.bench_estop = !self.bench_estop;
            }
            if i.key_pressed(egui::Key::W) {
                self.sim.control.throttle_cmd = (self.sim.control.throttle_cmd
                    + self.ui_config.throttle_key_step)
                    .clamp(0.0, 1.0);
            }
            if i.key_pressed(egui::Key::X) {
                self.sim.control.throttle_cmd = (self.sim.control.throttle_cmd
                    - self.ui_config.throttle_key_step)
                    .clamp(0.0, 1.0);
            }
        });
    }

    fn advance_simulation(&mut self) {
        let now = Instant::now();
        let wall_dt_s = (now - self.last_tick).as_secs_f64();
        self.apply_bench_console_interlocks();
        let target_time = if self.ui_config.sync_to_wall_clock {
            wall_dt_s.max(self.dt_base)
        } else {
            self.ui_config.simulated_time_per_frame_s.max(self.dt_base)
        };
        let mut simulated = 0.0;
        let mut steps = 0usize;
        let max_steps = self.ui_config.max_steps_per_frame.max(1);
        // In accuracy-first mode the solver advances a fixed slice of simulated time per frame
        // and does not try to stay phase-locked to wall clock.
        while simulated < target_time && steps < max_steps {
            let dt_target = if self.ui_config.sync_to_wall_clock {
                if let Some(dt_fixed) = self.realtime_fixed_dt_s {
                    self.dt_next = dt_fixed;
                    dt_fixed
                } else {
                    let dt_nom = rpm_linked_dt(
                        self.dt_base,
                        self.latest.rpm,
                        self.sim.params.default_target_rpm,
                        &self.sim.numerics,
                    );
                    let dt_target = dt_nom.clamp(self.dt_min_bound, self.dt_max_bound);
                    self.dt_next += self.ui_config.dt_smoothing_factor * (dt_target - self.dt_next);
                    self.dt_next = self.dt_next.clamp(self.dt_min_bound, self.dt_max_bound);
                    self.dt_next
                }
            } else {
                self.dt_next = accuracy_priority_dt(self.latest.rpm, &self.sim.numerics)
                    .clamp(self.dt_min_bound, self.dt_max_bound);
                self.dt_next
            };

            let remaining = target_time - simulated;
            let dt_step = dt_target.min(remaining).max(self.ui_config.dt_epsilon_s);
            self.apply_load_input(dt_step);
            self.latest = self.sim.step(dt_step);
            simulated += dt_step;
            steps = steps.saturating_add(1);
        }
        if simulated < target_time {
            // Render the newest computed state even if the requested simulated horizon was not
            // fully consumed in this frame.
            let dt_tail = (target_time - simulated)
                .max(self.ui_config.dt_epsilon_s)
                .min(self.dt_max_bound);
            self.apply_load_input(dt_tail);
            self.latest = self.sim.step(dt_tail);
            self.dt_next = self
                .realtime_fixed_dt_s
                .unwrap_or_else(|| dt_tail.clamp(self.dt_min_bound, self.dt_max_bound));
        }
        self.sync_required_brake_torque();
        self.update_schematic_phase(wall_dt_s);
        self.last_tick = now;
    }

    fn render_header_panel(&self, ctx: &egui::Context) {
        egui::TopBottomPanel::top("header")
            .frame(self.theme.header_frame())
            .show(ctx, |ui| {
                ui.horizontal(|ui| {
                    ui.vertical(|ui| {
                        ui.label(
                            egui::RichText::new("EDM TEST CELL // INLINE-4 DYNAMOMETER")
                                .color(self.theme.text_main)
                                .strong()
                                .size(26.0),
                        );
                        ui.label(
                            egui::RichText::new(if self.ui_config.sync_to_wall_clock {
                                "Wall-clock synchronized console with p-V / p-theta monitoring"
                            } else {
                                "Accuracy-first transient console with p-V / p-theta monitoring"
                            })
                            .color(self.theme.text_soft)
                            .size(12.0),
                        );
                    });
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        digital_readout(
                            ui,
                            self.theme,
                            self.theme.cyan,
                            "CELL STATUS",
                            if self.required_brake_torque_nm > 0.5 {
                                "FIRED"
                            } else {
                                "MOTOR"
                            },
                            "",
                            self.sim.model.external_load.mode.label(),
                            160.0,
                        );
                    });
                });
                ui.add_space(8.0);
                ui.horizontal_wrapped(|ui| {
                    annunciator(ui, self.theme, "E-STOP", self.bench_estop, self.theme.red);
                    annunciator(
                        ui,
                        self.theme,
                        "INTERLOCK",
                        self.bench_interlock_ok(),
                        self.theme.green,
                    );
                    annunciator(
                        ui,
                        self.theme,
                        "DYNO EN",
                        self.bench_dyno_enabled,
                        self.theme.cyan,
                    );
                    annunciator(
                        ui,
                        self.theme,
                        "VENT",
                        self.bench_cell_vent,
                        self.theme.cyan,
                    );
                    annunciator(
                        ui,
                        self.theme,
                        "COOLING",
                        self.bench_coolant_conditioning,
                        self.theme.cyan,
                    );
                    annunciator(
                        ui,
                        self.theme,
                        "RUN",
                        self.latest.rpm > 120.0,
                        self.theme.green,
                    );
                    annunciator(
                        ui,
                        self.theme,
                        "FUEL",
                        self.sim.control.fuel_cmd,
                        self.theme.amber,
                    );
                    annunciator(
                        ui,
                        self.theme,
                        "SPARK",
                        self.sim.control.spark_cmd,
                        self.theme.cyan,
                    );
                    annunciator(
                        ui,
                        self.theme,
                        "MOTOR",
                        self.sim.control.load_cmd < -0.02,
                        self.theme.red,
                    );
                    annunciator(
                        ui,
                        self.theme,
                        "FIRING",
                        self.required_brake_torque_nm > 0.5,
                        self.theme.green,
                    );
                    annunciator(
                        ui,
                        self.theme,
                        "SPD CTRL",
                        self.bench_dyno_enabled && !self.bench_estop,
                        self.theme.amber,
                    );
                    annunciator(
                        ui,
                        self.theme,
                        "PWR LIM",
                        self.dyno_power_limit_active(),
                        self.theme.red,
                    );
                    annunciator(
                        ui,
                        self.theme,
                        "ACCURACY",
                        !self.ui_config.sync_to_wall_clock,
                        self.theme.green,
                    );
                });
            });
    }

    fn render_control_rack(&mut self, ctx: &egui::Context) {
        egui::SidePanel::left("controls")
            .default_width(272.0)
            .resizable(true)
            .frame(self.theme.rack_frame())
            .show(ctx, |ui| {
                egui::ScrollArea::vertical()
                    .auto_shrink([false, false])
                    .show(ui, |ui| {
                        section_label(ui, self.theme, "OPERATOR RACK", self.theme.amber);

                        show_collapsible_module(
                            ui,
                            self.theme,
                            "rack_bench_console",
                            "Bench Console",
                            "interlocks, dyno coupling, and absorber limits",
                            self.theme.cyan,
                            true,
                            |ui| {
                                ui.horizontal_wrapped(|ui| {
                                    ui.toggle_value(&mut self.bench_estop, "E-STOP");
                                    ui.toggle_value(&mut self.bench_dyno_enabled, "Dyno enable");
                                    ui.toggle_value(&mut self.bench_cell_vent, "Cell vent");
                                    ui.toggle_value(
                                        &mut self.bench_coolant_conditioning,
                                        "Cooling cond",
                                    );
                                });
                                ui.add_space(6.0);
                                egui::Grid::new("bench_console_grid")
                                    .num_columns(2)
                                    .spacing(egui::vec2(16.0, 6.0))
                                    .show(ui, |ui| {
                                        metric_row(
                                            ui,
                                            self.theme,
                                            "Interlock",
                                            if self.bench_interlock_ok() {
                                                "closed".to_owned()
                                            } else {
                                                "open".to_owned()
                                            },
                                        );
                                        metric_row(
                                            ui,
                                            self.theme,
                                            "Control mode",
                                            if self.bench_dyno_enabled {
                                                "speed control".to_owned()
                                            } else {
                                                "dyno uncoupled".to_owned()
                                            },
                                        );
                                        metric_row(
                                            ui,
                                            self.theme,
                                            "Absorber torque act",
                                            format!("{:+.1} Nm", self.latest.torque_load_nm),
                                        );
                                        metric_row(
                                            ui,
                                            self.theme,
                                            "Absorber power",
                                            format!(
                                                "{:.1} / {:.0} kW",
                                                shaft_power_kw(
                                                    self.latest.rpm,
                                                    self.latest.torque_load_nm.abs()
                                                ),
                                                self.sim
                                                    .model
                                                    .external_load
                                                    .absorber_power_limit_kw
                                            ),
                                        );
                                        metric_row(
                                            ui,
                                            self.theme,
                                            "Available torque",
                                            format!("{:.1} Nm", self.dyno_available_torque_nm()),
                                        );
                                        metric_row(
                                            ui,
                                            self.theme,
                                            "Speed limit",
                                            format!(
                                                "{:.0} rpm{}",
                                                self.sim
                                                    .model
                                                    .external_load
                                                    .absorber_speed_limit_rpm,
                                                if external_load_speed_limit_active(
                                                    self.sim.state.omega_rad_s,
                                                    &self.sim.model.external_load
                                                ) {
                                                    " trip"
                                                } else {
                                                    ""
                                                }
                                            ),
                                        );
                                        metric_row(
                                            ui,
                                            self.theme,
                                            "Rotor inertia",
                                            format!(
                                                "{:.3} kg m^2",
                                                self.sim
                                                    .model
                                                    .external_load
                                                    .absorber_rotor_inertia_kgm2
                                            ),
                                        );
                                        metric_row(
                                            ui,
                                            self.theme,
                                            "Power-limit torque",
                                            format!("{:.1} Nm", self.dyno_power_limit_torque_nm()),
                                        );
                                    });
                            },
                        );

                        ui.add_space(8.0);
                        show_collapsible_module(
                            ui,
                            self.theme,
                            "rack_actuator",
                            "Actuator Deck",
                            "speed-control console with throttle, ignition, and VVT inputs",
                            self.theme.amber,
                            true,
                            |ui| {
                                let mut load_mode = self.sim.model.external_load.mode;

                                ui.add(
                                    egui::Slider::new(
                                        &mut self.sim.control.throttle_cmd,
                                        0.0..=1.0,
                                    )
                                    .text("Throttle cmd"),
                                );
                                let target_rpm_before = self.load_target_rpm;
                                ui.add(
                                    egui::Slider::new(
                                        &mut self.load_target_rpm,
                                        0.0..=self.sim.params.max_rpm,
                                    )
                                    .text("Target RPM"),
                                );
                                if (self.load_target_rpm - target_rpm_before).abs() > f64::EPSILON {
                                    self.load_speed_integral = 0.0;
                                }
                                ui.label(format!(
                                    "Required brake torque: {:.1} Nm",
                                    self.required_brake_torque_nm
                                ));
                                ui.label(format!(
                                    "Required brake power: {:.1} kW",
                                    shaft_power_kw(self.latest.rpm, self.required_brake_torque_nm)
                                ));
                                ui.label(format!(
                                    "RPM error: {:+.0} rpm / absorber torque {:.1} Nm",
                                    self.rpm_error(),
                                    self.latest.torque_load_nm
                                ));
                                egui::ComboBox::from_label("Load model")
                                    .selected_text(load_mode.label())
                                    .show_ui(ui, |ui| {
                                        ui.selectable_value(
                                            &mut load_mode,
                                            ExternalLoadMode::BrakeMap,
                                            ExternalLoadMode::BrakeMap.label(),
                                        );
                                        ui.selectable_value(
                                            &mut load_mode,
                                            ExternalLoadMode::VehicleEquivalent,
                                            ExternalLoadMode::VehicleEquivalent.label(),
                                        );
                                    });
                                if load_mode != self.sim.model.external_load.mode {
                                    self.sim.model.external_load.mode = load_mode;
                                }
                                ui.label(format!(
                                    "Absorber command: {:+.3}",
                                    self.sim.control.load_cmd
                                ));
                                ui.add_enabled_ui(self.bench_firing_permitted(), |ui| {
                                    ui.checkbox(&mut self.sim.control.spark_cmd, "Spark");
                                    ui.checkbox(&mut self.sim.control.fuel_cmd, "Fuel");
                                });
                                if !self.bench_firing_permitted() {
                                    ui.label(
                                        egui::RichText::new("Firing inhibited by bench interlock")
                                            .color(self.theme.red)
                                            .monospace(),
                                    );
                                }
                                ui.separator();
                                ui.add_enabled(
                                    true,
                                    egui::Slider::new(
                                        &mut self.sim.control.vvt_intake_deg,
                                        self.ui_config.vvt_slider_min_deg
                                            ..=self.ui_config.vvt_slider_max_deg,
                                    )
                                    .text("VVT Intake [deg]"),
                                );
                                ui.add_enabled(
                                    true,
                                    egui::Slider::new(
                                        &mut self.sim.control.vvt_exhaust_deg,
                                        self.ui_config.vvt_slider_min_deg
                                            ..=self.ui_config.vvt_slider_max_deg,
                                    )
                                    .text("VVT Exhaust [deg]"),
                                );
                                ui.add_enabled(
                                    true,
                                    egui::Slider::new(
                                        &mut self.sim.control.ignition_timing_deg,
                                        self.ui_config.ignition_slider_min_deg
                                            ..=self.ui_config.ignition_slider_max_deg,
                                    )
                                    .text("Ignition [deg BTDC]"),
                                );
                            },
                        );

                        ui.add_space(8.0);
                        show_collapsible_module(
                            ui,
                            self.theme,
                            "rack_status",
                            "Status Bus",
                            "cell and runtime telemetry",
                            self.theme.green,
                            true,
                            |ui| {
                                let load_model = &self.sim.model.external_load;
                                ui.label(format!(
                                    "Engine layout: fixed {}-cylinder",
                                    FIXED_CYLINDER_COUNT
                                ));
                                ui.label(format!(
                                    "Throttle eff: {:.3}",
                                    self.sim.state.throttle_eff
                                ));
                                ui.label("Operator input: Throttle cmd + Target RPM");
                                ui.label(format!("Target RPM: {:.0}", self.load_target_rpm));
                                ui.label(format!(
                                    "Required brake torque: {:.1} Nm",
                                    self.required_brake_torque_nm
                                ));
                                ui.label(format!(
                                    "Required brake power: {:.1} kW",
                                    shaft_power_kw(self.latest.rpm, self.required_brake_torque_nm)
                                ));
                                ui.label(format!("RPM error: {:+.0} rpm", self.rpm_error()));
                                ui.label(format!(
                                    "Absorber command: {:+.3}",
                                    self.sim.control.load_cmd
                                ));
                                ui.label(format!(
                                    "Absorber torque act / shaft est: {:+.1} / {:+.1} Nm",
                                    self.latest.torque_load_nm,
                                    self.shaft_torque_estimate_nm()
                                ));
                                ui.label(format!("Load model: {}", load_model.mode.label()));
                                ui.label(format!(
                                    "Bench interlock: {} / dyno {}",
                                    if self.bench_interlock_ok() {
                                        "closed"
                                    } else {
                                        "open"
                                    },
                                    if self.bench_dyno_enabled {
                                        "enabled"
                                    } else {
                                        "disabled"
                                    }
                                ));
                                ui.label(format!(
                                    "Absorber limit: {:.0} kW / {:.0} rpm",
                                    load_model.absorber_power_limit_kw,
                                    load_model.absorber_speed_limit_rpm
                                ));
                                if load_model.mode == ExternalLoadMode::VehicleEquivalent {
                                    ui.label(format!(
                                        "Vehicle eq: {:.1} km/h / Jref {:.3} kg m^2",
                                        external_load_vehicle_speed_kph(
                                            self.sim.state.omega_rad_s,
                                            load_model,
                                        ),
                                        external_load_reflected_inertia_kgm2(
                                            self.sim.control.load_cmd,
                                            load_model
                                        )
                                    ));
                                }
                                ui.label(if self.ui_config.sync_to_wall_clock {
                                    "Solver mode: wall-clock synchronized"
                                } else {
                                    "Solver mode: accuracy first"
                                });
                                ui.label(if self.required_brake_torque_nm > 0.5 {
                                    "State: FIRED"
                                } else {
                                    "State: MOTORING"
                                });
                                ui.label(if !self.bench_dyno_enabled {
                                    "Machine mode: DYNO DISABLED"
                                } else if self.sim.control.load_cmd < -0.02 {
                                    "Machine mode: SPEED HOLD / MOTOR"
                                } else {
                                    "Machine mode: SPEED HOLD / ABSORB"
                                });
                            },
                        );
                        ui.add_space(8.0);
                        self.render_state_bus(ui);
                    });
            });
    }

    fn render_console_overview(&self, ui: &mut egui::Ui) {
        let brake_torque_out_nm = self.required_brake_torque_nm;
        let brake_power_kw = shaft_power_kw(self.latest.rpm, brake_torque_out_nm);
        let absorber_torque_nm = self.latest.torque_load_nm.abs();
        let absorber_power_kw = shaft_power_kw(self.latest.rpm, absorber_torque_nm);
        let panel_width = ui.available_width().max(320.0);
        let readout_columns = responsive_card_columns(panel_width, 176.0, 6);
        let readout_width = responsive_card_width(panel_width, readout_columns).min(228.0);
        let gauge_columns = responsive_card_columns(panel_width, 136.0, 6);
        let gauge_width = responsive_card_width(panel_width, gauge_columns).min(172.0);
        let meter_columns = responsive_card_columns(panel_width, 188.0, 3);
        let meter_width = responsive_card_width(panel_width, meter_columns).min(252.0);
        let intake_state = format!(
            "MAP {:.1} kPa / runner {:.1} kPa",
            self.latest.map_kpa, self.latest.intake_runner_kpa
        );
        let thermal_state = format!(
            "eta {:.1}% / Wi {:.3} kJ",
            self.latest.eta_thermal_indicated_pv * 100.0,
            self.latest.indicated_work_cycle_j * 1.0e-3
        );

        self.theme.monitor_frame().show(ui, |ui| {
            monitor_heading(
                ui,
                self.theme,
                "Operator Display",
                "primary live instruments",
                self.theme.amber,
            );
            let mut readout_index = 0usize;
            while readout_index < 8 {
                ui.horizontal(|ui| {
                    ui.spacing_mut().item_spacing.x = 8.0;
                    for slot in 0..readout_columns {
                        let idx = readout_index + slot;
                        if idx >= 8 {
                            break;
                        }
                        match idx {
                            0 => digital_readout(
                                ui,
                                self.theme,
                                self.theme.amber,
                                "ENGINE SPEED",
                                &format!("{:.0}", self.latest.rpm),
                                "rpm",
                                if self.required_brake_torque_nm > 0.5 {
                                    "fired operating point"
                                } else {
                                    "motoring operating point"
                                },
                                readout_width,
                            ),
                            1 => digital_readout(
                                ui,
                                self.theme,
                                self.theme.red,
                                "BRAKE TORQUE",
                                &format!("{:.1}", brake_torque_out_nm),
                                "Nm",
                                &format!("net {:.1} Nm", self.latest.torque_net_nm),
                                readout_width,
                            ),
                            2 => digital_readout(
                                ui,
                                self.theme,
                                self.theme.cyan,
                                "BRAKE POWER FILT",
                                &format!("{:.1}", brake_power_kw),
                                "kW",
                                &format!(
                                    "{:.1} hp",
                                    shaft_power_hp(self.latest.rpm, brake_torque_out_nm)
                                ),
                                readout_width,
                            ),
                            3 => digital_readout(
                                ui,
                                self.theme,
                                self.theme.red,
                                "ABSORBER TORQUE",
                                &format!("{:+.1}", self.latest.torque_load_nm),
                                "Nm",
                                &format!("{:.1} kW", absorber_power_kw),
                                readout_width,
                            ),
                            4 => digital_readout(
                                ui,
                                self.theme,
                                self.theme.green,
                                "AIR CHARGE",
                                &format!("{:.2}", self.latest.trapped_air_mg),
                                "mg/cyl",
                                &format!("VE {:.1}%", self.latest.volumetric_efficiency * 100.0),
                                readout_width,
                            ),
                            5 => digital_readout(
                                ui,
                                self.theme,
                                self.theme.amber,
                                "INTAKE BUS",
                                &format!("{:.1}", self.latest.map_kpa),
                                "kPa",
                                &intake_state,
                                readout_width,
                            ),
                            6 => digital_readout(
                                ui,
                                self.theme,
                                self.theme.green,
                                "DYNO LIMIT",
                                &format!("{:.0}", self.dyno_available_torque_nm()),
                                "Nm",
                                &format!(
                                    "{:.0} kW / {:.0} rpm",
                                    self.sim.model.external_load.absorber_power_limit_kw,
                                    self.sim.model.external_load.absorber_speed_limit_rpm
                                ),
                                readout_width,
                            ),
                            7 => digital_readout(
                                ui,
                                self.theme,
                                self.theme.cyan,
                                "INDIC EFF",
                                &format!("{:.1}", self.latest.eta_thermal_indicated_pv * 100.0),
                                "%",
                                &thermal_state,
                                readout_width,
                            ),
                            _ => unreachable!(),
                        }
                    }
                });
                readout_index += readout_columns;
            }

            ui.add_space(6.0);
            let mut gauge_index = 0usize;
            while gauge_index < 7 {
                ui.horizontal(|ui| {
                    ui.spacing_mut().item_spacing.x = 8.0;
                    for slot in 0..gauge_columns {
                        let idx = gauge_index + slot;
                        if idx >= 7 {
                            break;
                        }
                        let spec = match idx {
                            0 => GaugeSpec {
                                label: "RPM",
                                value: self.latest.rpm,
                                min: 0.0,
                                max: self.sim.params.max_rpm,
                                unit: "rpm",
                                accent: self.theme.amber,
                                footer: "crankshaft speed",
                                width: gauge_width,
                                height: 112.0,
                            },
                            1 => GaugeSpec {
                                label: "MAP",
                                value: self.latest.map_kpa,
                                min: 20.0,
                                max: 120.0,
                                unit: "kPa",
                                accent: self.theme.cyan,
                                footer: "intake plenum",
                                width: gauge_width,
                                height: 112.0,
                            },
                            2 => GaugeSpec {
                                label: "LAMBDA",
                                value: self.latest.lambda_target,
                                min: 0.70,
                                max: 1.20,
                                unit: "-",
                                accent: self.theme.green,
                                footer: "target mixture",
                                width: gauge_width,
                                height: 112.0,
                            },
                            3 => GaugeSpec {
                                label: "BMEP",
                                value: self.latest.brake_bmep_bar,
                                min: 0.0,
                                max: 16.0,
                                unit: "bar",
                                accent: self.theme.red,
                                footer: "brake mean effective pressure",
                                width: gauge_width,
                                height: 112.0,
                            },
                            4 => GaugeSpec {
                                label: "EXH TEMP",
                                value: self.latest.exhaust_temp_k,
                                min: 300.0,
                                max: 1_200.0,
                                unit: "K",
                                accent: self.theme.amber,
                                footer: "collector model",
                                width: gauge_width,
                                height: 112.0,
                            },
                            5 => GaugeSpec {
                                label: "ABS TORQ",
                                value: absorber_torque_nm,
                                min: 0.0,
                                max: self.sim.model.external_load.torque_max_nm.max(1.0),
                                unit: "Nm",
                                accent: self.theme.red,
                                footer: "absorber applied torque",
                                width: gauge_width,
                                height: 112.0,
                            },
                            6 => GaugeSpec {
                                label: "INT EGR",
                                value: self.latest.internal_egr_fraction * 100.0,
                                min: 0.0,
                                max: 30.0,
                                unit: "%",
                                accent: self.theme.green,
                                footer: "internal residual fraction",
                                width: gauge_width,
                                height: 112.0,
                            },
                            _ => unreachable!(),
                        };
                        gauge(ui, self.theme, spec);
                    }
                });
                gauge_index += gauge_columns;
            }

            ui.add_space(6.0);
            let mut meter_index = 0usize;
            while meter_index < 7 {
                ui.horizontal(|ui| {
                    ui.spacing_mut().item_spacing.x = 8.0;
                    for slot in 0..meter_columns {
                        let idx = meter_index + slot;
                        if idx >= 7 {
                            break;
                        }
                        let spec = match idx {
                            0 => LinearMeterSpec {
                                label: "THROTTLE CMD",
                                value: self.sim.control.throttle_cmd,
                                min: 0.0,
                                max: 1.0,
                                accent: self.theme.amber,
                                value_text: format!("{:.3}", self.sim.control.throttle_cmd),
                                width: meter_width,
                            },
                            1 => LinearMeterSpec {
                                label: "THROTTLE EFF",
                                value: self.sim.state.throttle_eff,
                                min: 0.0,
                                max: 1.0,
                                accent: self.theme.cyan,
                                value_text: format!("{:.3}", self.sim.state.throttle_eff),
                                width: meter_width,
                            },
                            2 => LinearMeterSpec {
                                label: "TARGET RPM",
                                value: self.load_target_rpm,
                                min: 0.0,
                                max: self.sim.params.max_rpm,
                                accent: self.theme.red,
                                value_text: format!(
                                    "{:.0} rpm / err {:+.0}",
                                    self.load_target_rpm,
                                    self.rpm_error()
                                ),
                                width: meter_width,
                            },
                            3 => LinearMeterSpec {
                                label: "IGNITION",
                                value: self.sim.control.ignition_timing_deg,
                                min: self.ui_config.ignition_slider_min_deg,
                                max: self.ui_config.ignition_slider_max_deg,
                                accent: self.theme.green,
                                value_text: format!(
                                    "{:.1} deg BTDC",
                                    self.sim.control.ignition_timing_deg
                                ),
                                width: meter_width,
                            },
                            4 => LinearMeterSpec {
                                label: "VVT IN",
                                value: self.sim.control.vvt_intake_deg,
                                min: self.ui_config.vvt_slider_min_deg,
                                max: self.ui_config.vvt_slider_max_deg,
                                accent: self.theme.cyan,
                                value_text: format!("{:.1} deg", self.sim.control.vvt_intake_deg),
                                width: meter_width,
                            },
                            5 => LinearMeterSpec {
                                label: "VVT EX",
                                value: self.sim.control.vvt_exhaust_deg,
                                min: self.ui_config.vvt_slider_min_deg,
                                max: self.ui_config.vvt_slider_max_deg,
                                accent: self.theme.red,
                                value_text: format!("{:.1} deg", self.sim.control.vvt_exhaust_deg),
                                width: meter_width,
                            },
                            6 => LinearMeterSpec {
                                label: "BRAKE POWER",
                                value: brake_power_kw,
                                min: 0.0,
                                max: 160.0,
                                accent: self.theme.cyan,
                                value_text: format!("{:.1} kW", brake_power_kw),
                                width: meter_width,
                            },
                            _ => unreachable!(),
                        };
                        linear_meter(ui, self.theme, spec);
                    }
                });
                meter_index += meter_columns;
            }
        });
        ui.add_space(8.0);
    }

    fn render_state_bus(&self, ui: &mut egui::Ui) {
        show_collapsible_module(
            ui,
            self.theme,
            "rack_sensor_state",
            "Sensor / State Bus",
            "live reduced-order state and closures",
            self.theme.green,
            true,
            |ui| {
                egui::Grid::new("state_bus_grid")
                    .num_columns(2)
                    .spacing(egui::vec2(16.0, 6.0))
                    .show(ui, |ui| {
                        metric_row(
                            ui,
                            self.theme,
                            "Absorber torque act",
                            format!("{:+.1} Nm", self.latest.torque_load_nm),
                        );
                        metric_row(
                            ui,
                            self.theme,
                            "Absorber torque avail",
                            format!("{:.1} Nm", self.dyno_available_torque_nm()),
                        );
                        metric_row(
                            ui,
                            self.theme,
                            "Absorber power",
                            format!(
                                "{:.1} / {:.0} kW",
                                shaft_power_kw(self.latest.rpm, self.latest.torque_load_nm.abs()),
                                self.sim.model.external_load.absorber_power_limit_kw
                            ),
                        );
                        metric_row(
                            ui,
                            self.theme,
                            "Intake runner pressure",
                            format!("{:.1} kPa", self.latest.intake_runner_kpa),
                        );
                        metric_row(
                            ui,
                            self.theme,
                            "Exhaust pressure",
                            format!("{:.1} kPa", self.latest.exhaust_kpa),
                        );
                        metric_row(
                            ui,
                            self.theme,
                            "Exhaust runner pressure",
                            format!("{:.1} kPa", self.latest.exhaust_runner_kpa),
                        );
                        metric_row(
                            ui,
                            self.theme,
                            "Wave pressure I / E",
                            format!(
                                "{:.2} / {:.2} kPa",
                                self.latest.intake_wave_kpa, self.latest.exhaust_wave_kpa
                            ),
                        );
                        metric_row(
                            ui,
                            self.theme,
                            "Air flow",
                            format!("{:.2} g/s", self.latest.air_flow_gps),
                        );
                        metric_row(
                            ui,
                            self.theme,
                            "Charge temp",
                            format!("{:.1} K", self.latest.intake_charge_temp_k),
                        );
                        metric_row(
                            ui,
                            self.theme,
                            "Pulse VE / ram / scav",
                            format!(
                                "{:.3} / {:.3} / {:.3}",
                                self.latest.ve_pulse_multiplier,
                                self.latest.intake_ram_multiplier,
                                self.latest.exhaust_scavenge_multiplier
                            ),
                        );
                        metric_row(
                            ui,
                            self.theme,
                            "Internal EGR",
                            format!("{:.1} %", self.latest.internal_egr_fraction * 100.0),
                        );
                        metric_row(
                            ui,
                            self.theme,
                            "Ignition timing",
                            format!("{:.1} deg BTDC", self.latest.ignition_timing_deg),
                        );
                        metric_row(
                            ui,
                            self.theme,
                            "Heat loss",
                            format!("{:.3} kJ/cyl/cycle", self.latest.heat_loss_cycle_j * 1.0e-3),
                        );
                        metric_row(
                            ui,
                            self.theme,
                            "IMEP",
                            format!("{:.2} bar", self.latest.imep_bar),
                        );
                        metric_row(
                            ui,
                            self.theme,
                            "Theoretical eta",
                            format!("{:.1} %", self.latest.eta_thermal_theoretical * 100.0),
                        );
                        metric_row(
                            ui,
                            self.theme,
                            "Cycle angle / burn rate",
                            format!(
                                "{:.1} deg / {:.2}",
                                self.latest.cycle_deg, self.latest.combustion_rate_norm
                            ),
                        );
                        metric_row(
                            ui,
                            self.theme,
                            "Net torque inst / filt",
                            format!(
                                "{:+.2} / {:+.2} Nm",
                                self.latest.torque_net_inst_nm, self.latest.torque_net_nm
                            ),
                        );
                        metric_row(
                            ui,
                            self.theme,
                            "Torque comb inst / cycle mean",
                            format!(
                                "{:.2} / {:.2} Nm",
                                self.latest.torque_combustion_nm,
                                self.latest.torque_combustion_cycle_nm
                            ),
                        );
                        metric_row(
                            ui,
                            self.theme,
                            "Torque friction / pump",
                            format!(
                                "{:.2} / {:.2} Nm",
                                self.latest.torque_friction_nm, self.latest.torque_pumping_nm
                            ),
                        );
                        metric_row(
                            ui,
                            self.theme,
                            "Shaft est / machine torque",
                            format!(
                                "{:+.2} / {:+.2} Nm",
                                self.shaft_torque_estimate_nm(),
                                self.latest.torque_load_nm
                            ),
                        );
                        metric_row(
                            ui,
                            self.theme,
                            "Brake power / load power",
                            format!(
                                "{:.2} / {:.2} kW",
                                shaft_power_kw(self.latest.rpm, self.required_brake_torque_nm),
                                shaft_power_kw(self.latest.rpm, self.latest.torque_load_nm)
                            ),
                        );
                    });
            },
        );
    }

    fn render_pv_plot(&self, ui: &mut egui::Ui, graph_heights: GraphLayoutHeights) {
        let pv_points: PlotPoints<'_> = self
            .latest
            .pv_points
            .iter()
            .map(|(v, p)| [*v, *p * 1e-3])
            .collect();
        let pv_y_peak_kpa = self
            .latest
            .pv_points
            .iter()
            .map(|(_, p)| *p * 1e-3)
            .fold(self.sim.plot.pv_y_min_kpa, f64::max);
        let pv_y_max_plot = self.sim.plot.pv_y_max_kpa.max(
            (pv_y_peak_kpa * self.ui_config.pv_headroom_ratio)
                .max(self.sim.plot.pv_y_min_kpa + self.ui_config.pv_min_headroom_kpa),
        );
        let pv_line = Line::new(pv_points)
            .name("p-V")
            .color(egui::Color32::from_rgb(255, 170, 40))
            .width(self.ui_config.line_width_px);

        self.theme.monitor_frame().show(ui, |ui| {
            monitor_heading(
                ui,
                self.theme,
                "Cylinder p-V",
                "reconstructed display loop",
                self.theme.amber,
            );
            Plot::new("pv_plot")
                .height(graph_heights.pv_plot_px)
                .allow_scroll(false)
                .allow_drag(false)
                .allow_zoom(false)
                .legend(Legend::default())
                .x_axis_label("Normalized volume [-]")
                .y_axis_label("Pressure [kPa]")
                .show(ui, |plot_ui| {
                    plot_ui.set_auto_bounds(false);
                    plot_ui.set_plot_bounds(PlotBounds::from_min_max(
                        [self.sim.plot.pv_x_min, self.sim.plot.pv_y_min_kpa],
                        [self.sim.plot.pv_x_max, pv_y_max_plot],
                    ));
                    plot_ui.line(pv_line);
                });
        });
    }

    fn render_ptheta_plot(&self, ui: &mut egui::Ui, graph_heights: GraphLayoutHeights) {
        let sample_count = (self.sim.model.pv_display_bins / FIXED_CYLINDER_COUNT).max(360);
        let curves = self.sim.build_ptheta_display_curves(sample_count);
        let ptheta_y_peak_kpa = curves
            .iter()
            .flat_map(|curve| curve.iter().map(|(_, p)| *p * 1e-3))
            .fold(self.sim.plot.pv_y_min_kpa, f64::max);
        let ptheta_y_max_plot = self.sim.plot.pv_y_max_kpa.max(
            (ptheta_y_peak_kpa * self.ui_config.pv_headroom_ratio)
                .max(self.sim.plot.pv_y_min_kpa + self.ui_config.pv_min_headroom_kpa),
        );

        self.theme.monitor_frame().show(ui, |ui| {
            monitor_heading(
                ui,
                self.theme,
                "Cylinder p-theta",
                "4-cylinder overlay / 0..720 degCA",
                self.theme.cyan,
            );
            Plot::new("ptheta_plot")
                .height(graph_heights.pv_plot_px)
                .allow_scroll(false)
                .allow_drag(false)
                .allow_zoom(false)
                .legend(Legend::default())
                .x_axis_label("Crank angle [degCA]")
                .y_axis_label("Pressure [kPa]")
                .show(ui, |plot_ui| {
                    plot_ui.set_auto_bounds(false);
                    plot_ui.set_plot_bounds(PlotBounds::from_min_max(
                        [0.0, self.sim.plot.pv_y_min_kpa],
                        [720.0, ptheta_y_max_plot],
                    ));
                    for (cylinder_idx, curve) in curves.iter().enumerate() {
                        if curve.is_empty() {
                            continue;
                        }
                        let points: PlotPoints<'_> =
                            curve.iter().map(|(theta, p)| [*theta, *p * 1e-3]).collect();
                        plot_ui.line(
                            Line::new(points)
                                .name(format!("Cyl {}", cylinder_idx + 1))
                                .color(cylinder_trace_color(cylinder_idx))
                                .width(self.ui_config.line_width_px),
                        );
                    }
                });
        });
    }

    fn render_pressure_plots(&self, ui: &mut egui::Ui, graph_heights: GraphLayoutHeights) {
        ui.columns(2, |columns| {
            self.render_pv_plot(&mut columns[0], graph_heights);
            self.render_ptheta_plot(&mut columns[1], graph_heights);
        });
        ui.add_space(graph_heights.section_spacing_px);
    }

    fn render_cycle_monitors(&self, ui: &mut egui::Ui, graph_heights: GraphLayoutHeights) {
        show_collapsible_module(
            ui,
            self.theme,
            "cycle_monitors",
            "Cycle Monitors",
            "recent histories and valve event trace",
            self.theme.cyan,
            true,
            |ui| {
                let recent_cycles = self.sim.plot.history_recent_cycles.max(1);

                ui.columns(2, |columns| {
                    let rpm_lines = recent_cycle_plot_lines(&self.sim.history_rpm, recent_cycles);
                    let rpm_min_span = (self.sim.params.max_rpm * 0.04).max(150.0);
                    let (rpm_y_min, rpm_y_max) = adaptive_plot_y_range(
                        recent_cycle_min_max(&self.sim.history_rpm, recent_cycles),
                        rpm_min_span,
                        self.ui_config.torque_margin_ratio,
                        Some(0.0),
                    );
                    Plot::new("rpm_plot")
                        .height(graph_heights.standard_plot_px)
                        .allow_scroll(false)
                        .allow_drag(false)
                        .allow_zoom(false)
                        .legend(Legend::default())
                        .x_axis_label("Crank angle [degCA]")
                        .y_axis_label("RPM inst [rpm]")
                        .show(&mut columns[0], |plot_ui| {
                            plot_ui.set_auto_bounds(false);
                            plot_ui.set_plot_bounds(PlotBounds::from_min_max(
                                [0.0, rpm_y_min],
                                [720.0, rpm_y_max],
                            ));
                            let total = rpm_lines.len();
                            for (idx, points) in rpm_lines.into_iter().enumerate() {
                                let mut line = Line::new(points)
                                    .color(faded_cycle_color(
                                        egui::Color32::LIGHT_GREEN,
                                        idx,
                                        total,
                                    ))
                                    .width(self.ui_config.line_width_px);
                                if idx + 1 == total {
                                    line = line.name("RPM");
                                }
                                plot_ui.line(line);
                            }
                            plot_ui.vline(
                                VLine::new(self.latest.cycle_deg)
                                    .name("Crank cursor")
                                    .color(self.theme.chrome),
                            );
                        });

                    let torque_lines =
                        recent_cycle_plot_lines(&self.sim.history_torque_net_nm, recent_cycles);
                    let load_lines =
                        recent_cycle_plot_lines(&self.sim.history_torque_load_nm, recent_cycles);
                    let (torque_y_min, torque_y_max) = adaptive_plot_y_range(
                        combine_min_max(
                            recent_cycle_min_max(&self.sim.history_torque_net_nm, recent_cycles),
                            recent_cycle_min_max(&self.sim.history_torque_load_nm, recent_cycles),
                        ),
                        self.ui_config.torque_min_span_nm,
                        self.ui_config.torque_margin_ratio,
                        None,
                    );
                    Plot::new("torque_plot")
                        .height(graph_heights.standard_plot_px)
                        .allow_scroll(false)
                        .allow_drag(false)
                        .allow_zoom(false)
                        .legend(Legend::default())
                        .x_axis_label("Crank angle [degCA]")
                        .y_axis_label("Net torque filt / load [Nm]")
                        .show(&mut columns[1], |plot_ui| {
                            plot_ui.set_auto_bounds(false);
                            plot_ui.set_plot_bounds(PlotBounds::from_min_max(
                                [0.0, torque_y_min],
                                [720.0, torque_y_max],
                            ));
                            let total = torque_lines.len().max(load_lines.len());
                            let torque_total = torque_lines.len();
                            let load_total = load_lines.len();
                            for (idx, points) in torque_lines.into_iter().enumerate() {
                                let mut line = Line::new(points)
                                    .color(faded_cycle_color(
                                        egui::Color32::from_rgb(255, 120, 120),
                                        idx,
                                        total,
                                    ))
                                    .width(self.ui_config.line_width_px);
                                if idx + 1 == torque_total {
                                    line = line.name("Filtered net torque [Nm]");
                                }
                                plot_ui.line(line);
                            }
                            for (idx, points) in load_lines.into_iter().enumerate() {
                                let mut line = Line::new(points)
                                    .color(faded_cycle_color(
                                        egui::Color32::from_rgb(255, 185, 70),
                                        idx,
                                        total,
                                    ))
                                    .width(self.ui_config.line_width_px);
                                if idx + 1 == load_total {
                                    line = line.name("External load [Nm]");
                                }
                                plot_ui.line(line);
                            }
                            plot_ui.vline(
                                VLine::new(self.latest.cycle_deg)
                                    .name("Crank cursor")
                                    .color(self.theme.chrome),
                            );
                        });
                });

                ui.add_space(graph_heights.section_spacing_px);

                ui.columns(2, |columns| {
                    let trapped_air_lines =
                        recent_cycle_plot_lines(&self.sim.history_trapped_air_mg, recent_cycles);
                    let trapped_air_min_span =
                        (self.ui_config.trapped_air_min_y_max_mg * 0.25).max(20.0);
                    let (trapped_air_y_min, trapped_air_y_max) = adaptive_plot_y_range(
                        recent_cycle_min_max(&self.sim.history_trapped_air_mg, recent_cycles),
                        trapped_air_min_span,
                        self.ui_config.torque_margin_ratio,
                        Some(0.0),
                    );
                    Plot::new("air_mass_plot")
                        .height(graph_heights.standard_plot_px)
                        .allow_scroll(false)
                        .allow_drag(false)
                        .allow_zoom(false)
                        .legend(Legend::default())
                        .x_axis_label("Crank angle [degCA]")
                        .y_axis_label("Trapped air inst [mg/cyl]")
                        .show(&mut columns[0], |plot_ui| {
                            plot_ui.set_auto_bounds(false);
                            plot_ui.set_plot_bounds(PlotBounds::from_min_max(
                                [0.0, trapped_air_y_min],
                                [720.0, trapped_air_y_max],
                            ));
                            let total = trapped_air_lines.len();
                            for (idx, points) in trapped_air_lines.into_iter().enumerate() {
                                let mut line = Line::new(points)
                                    .color(faded_cycle_color(
                                        egui::Color32::from_rgb(120, 200, 255),
                                        idx,
                                        total,
                                    ))
                                    .width(self.ui_config.line_width_px);
                                if idx + 1 == total {
                                    line = line.name("Trapped air inst [mg/cyl]");
                                }
                                plot_ui.line(line);
                            }
                            plot_ui.vline(
                                VLine::new(self.latest.cycle_deg)
                                    .name("Crank cursor")
                                    .color(self.theme.chrome),
                            );
                        });

                    let (intake_cam, exhaust_cam) =
                        cam_profile_points(&self.sim.control, &self.sim.cam, &self.sim.model);
                    let intake_line = Line::new(intake_cam)
                        .name("Intake lift")
                        .color(egui::Color32::from_rgb(80, 190, 255))
                        .width(self.ui_config.line_width_px);
                    let exhaust_line = Line::new(exhaust_cam)
                        .name("Exhaust lift")
                        .color(egui::Color32::from_rgb(255, 110, 110))
                        .width(self.ui_config.line_width_px);
                    let crank_cursor = VLine::new(self.latest.cycle_deg)
                        .name("Crank")
                        .color(egui::Color32::from_rgb(170, 170, 170))
                        .width(self.ui_config.crank_line_width_px);
                    Plot::new("cam_plot")
                        .height(graph_heights.standard_plot_px)
                        .allow_scroll(false)
                        .allow_drag(false)
                        .allow_zoom(false)
                        .legend(Legend::default())
                        .x_axis_label("Crank angle [degCA]")
                        .y_axis_label("Valve lift [mm]")
                        .show(&mut columns[1], |plot_ui| {
                            plot_ui.set_auto_bounds(false);
                            plot_ui.set_plot_bounds(PlotBounds::from_min_max(
                                [0.0, 0.0],
                                [720.0, self.sim.cam.display_y_max_mm],
                            ));
                            plot_ui.line(intake_line);
                            plot_ui.line(exhaust_line);
                            plot_ui.vline(crank_cursor);
                        });
                });

                ui.add_space(graph_heights.section_spacing_px);
                self.render_engine_motion_schematic(ui, graph_heights);
            },
        );
    }

    fn render_engine_motion_schematic(&self, ui: &mut egui::Ui, graph_heights: GraphLayoutHeights) {
        self.theme.monitor_frame().show(ui, |ui| {
            monitor_heading(
                ui,
                self.theme,
                "Engine Motion Schematic",
                "single-cylinder cutaway showing piston, crank, poppet valves, tent-roof chamber, and flame travel from current bore, stroke, ignition, and VVT",
                self.theme.green,
            );

            let desired_height = (graph_heights.standard_plot_px * 3.4).clamp(360.0, 460.0);
            let desired_width = ui.available_width().min(460.0).max(340.0);
            let desired_size = egui::vec2(desired_width, desired_height);

            let local_cycle_deg = self.schematic_cycle_deg;
            let intake_center_deg =
                self.sim.cam.intake_centerline_deg - self.sim.control.vvt_intake_deg;
            let exhaust_center_deg =
                self.sim.cam.exhaust_centerline_deg + self.sim.control.vvt_exhaust_deg;
            let intake_lift_mm = cam_lift_mm(
                local_cycle_deg,
                intake_center_deg,
                self.sim.cam.intake_duration_deg,
                self.sim.cam.intake_max_lift_mm,
                &self.sim.model,
            )
            .max(0.0);
            let exhaust_lift_mm = cam_lift_mm(
                local_cycle_deg,
                exhaust_center_deg,
                self.sim.cam.exhaust_duration_deg,
                self.sim.cam.exhaust_max_lift_mm,
                &self.sim.model,
            )
            .max(0.0);
            let intake_open_deg =
                wrap_cycle_deg(intake_center_deg - 0.5 * self.sim.cam.intake_duration_deg);
            let intake_close_deg =
                wrap_cycle_deg(intake_center_deg + 0.5 * self.sim.cam.intake_duration_deg);
            let exhaust_open_deg =
                wrap_cycle_deg(exhaust_center_deg - 0.5 * self.sim.cam.exhaust_duration_deg);
            let exhaust_close_deg =
                wrap_cycle_deg(exhaust_center_deg + 0.5 * self.sim.cam.exhaust_duration_deg);

            let spark_event_deg = (360.0 - self.latest.ignition_timing_deg).rem_euclid(720.0);
            let spark_rel_deg = (local_cycle_deg - spark_event_deg).rem_euclid(720.0);
            let burn_duration_deg = self.latest.burn_duration_deg.max(1.0);
            let burn_rel_deg =
                (local_cycle_deg - self.latest.burn_start_deg.rem_euclid(720.0)).rem_euclid(720.0);
            let ignition_flash = self.sim.control.spark_cmd
                && self.sim.control.fuel_cmd
                && spark_rel_deg <= 12.0;
            let burn_active = self.sim.control.spark_cmd
                && self.sim.control.fuel_cmd
                && burn_rel_deg <= burn_duration_deg + 18.0;
            let crank_deg_per_s = (self.latest.rpm.max(1.0) * 6.0) as f32;
            let burn_elapsed_s = (burn_rel_deg as f32 / crank_deg_per_s).max(0.0);
            let burn_total_s = (burn_duration_deg as f32 / crank_deg_per_s).max(1.0e-4);
            let characteristic_travel_m = (0.48 * self.sim.params.bore_m.max(1.0e-4)) as f32;
            let combustion_rate_gain = (0.60
                + 0.40 * self.latest.combustion_rate_norm.clamp(0.0, 1.4) as f32)
                .clamp(0.55, 1.20);
            let flame_speed_mps =
                (characteristic_travel_m / burn_total_s).max(0.1) * combustion_rate_gain;
            let flame_radius_norm = if burn_active {
                (burn_elapsed_s * flame_speed_mps / characteristic_travel_m).clamp(0.0, 1.0)
            } else {
                0.0
            };
            let chamber_base = if self.sim.control.fuel_cmd {
                self.theme.cyan.gamma_multiply(0.14)
            } else {
                self.theme.panel_bg
            };
            let chamber_hot = self.theme.red.gamma_multiply(0.34);
            let chamber_fill = if burn_active || ignition_flash {
                lerp_color(chamber_base, chamber_hot, flame_radius_norm.max(0.22))
            } else {
                chamber_base
            };

            ui.vertical_centered(|ui| {
                let (rect, _) = ui.allocate_exact_size(desired_size, egui::Sense::hover());
                let painter = ui.painter_at(rect);
                painter.rect_filled(rect, 12.0, self.theme.panel_alt_bg);
                painter.rect_stroke(
                    rect,
                    12.0,
                    egui::Stroke::new(1.0, self.theme.bezel),
                    egui::epaint::StrokeKind::Inside,
                );

                let center_x = rect.center().x;
                let top_y = rect.top() + 26.0;
                let deck_y = rect.top() + 112.0;
                let max_geom = self.sim.params.stroke_m.max(self.sim.params.bore_m * 0.92);
                let bore_ratio = (self.sim.params.bore_m / max_geom.max(f64::EPSILON)) as f32;
                let stroke_ratio = (self.sim.params.stroke_m / max_geom.max(f64::EPSILON)) as f32;
                let bore_px = (136.0 + 38.0 * bore_ratio).clamp(126.0, 176.0);
                let stroke_px = (168.0 + 64.0 * stroke_ratio).clamp(170.0, 240.0);
                let bore_left = center_x - bore_px * 0.5;
                let bore_right = center_x + bore_px * 0.5;
                let cylinder_bottom = deck_y + stroke_px;
                let cylinder_rect = egui::Rect::from_min_max(
                    egui::pos2(bore_left, deck_y - 12.0),
                    egui::pos2(bore_right, cylinder_bottom),
                );
                let chamber_apex = egui::pos2(center_x, deck_y - 38.0);
                let intake_port_rect = egui::Rect::from_min_max(
                    egui::pos2(bore_left - 98.0, top_y + 8.0),
                    egui::pos2(center_x - 18.0, top_y + 44.0),
                );
                let exhaust_port_rect = egui::Rect::from_min_max(
                    egui::pos2(center_x + 18.0, top_y + 8.0),
                    egui::pos2(bore_right + 98.0, top_y + 44.0),
                );
                let intake_stem_x = bore_left + bore_px * 0.33;
                let exhaust_stem_x = bore_right - bore_px * 0.33;
                let seat_y = deck_y - 5.0;
                let valve_max_lift_mm = self
                    .sim
                    .cam
                    .intake_max_lift_mm
                    .max(self.sim.cam.exhaust_max_lift_mm)
                    .max(1.0);
                let lift_scale = 32.0 / valve_max_lift_mm as f32;
                let intake_lift_px = intake_lift_mm as f32 * lift_scale;
                let exhaust_lift_px = exhaust_lift_mm as f32 * lift_scale;
                let valve_closed_head_y = seat_y + 5.0;
                let intake_head_y = valve_closed_head_y + intake_lift_px;
                let exhaust_head_y = valve_closed_head_y + exhaust_lift_px;
                let piston_phase = local_cycle_deg.to_radians().rem_euclid(std::f64::consts::TAU);
                let piston_frac = 0.5 * (1.0 - piston_phase.cos());
                let piston_h = 26.0;
                let piston_y = deck_y + (stroke_px - piston_h - 8.0) * piston_frac as f32;
                let piston_rect = egui::Rect::from_min_max(
                    egui::pos2(bore_left + 6.0, piston_y),
                    egui::pos2(bore_right - 6.0, piston_y + piston_h),
                );
                let crank_center = egui::pos2(center_x, cylinder_bottom + 46.0);
                let crank_radius = 28.0;
                let crank_pin = egui::pos2(
                    center_x + crank_radius * (piston_phase.sin() as f32),
                    crank_center.y - crank_radius * (piston_phase.cos() as f32),
                );
                let rod_top = piston_rect.center_bottom();
                let chamber_poly = vec![
                    egui::pos2(bore_left, deck_y),
                    egui::pos2(bore_left + 20.0, deck_y - 10.0),
                    chamber_apex,
                    egui::pos2(bore_right - 20.0, deck_y - 10.0),
                    egui::pos2(bore_right, deck_y),
                ];

                painter.rect_filled(cylinder_rect, 10.0, chamber_fill);
                painter.add(egui::Shape::convex_polygon(
                    chamber_poly.clone(),
                    chamber_fill,
                    egui::Stroke::NONE,
                ));
                painter.rect_stroke(
                    cylinder_rect,
                    10.0,
                    egui::Stroke::new(1.4, self.theme.chrome),
                    egui::epaint::StrokeKind::Inside,
                );
                painter.add(egui::Shape::closed_line(
                    chamber_poly,
                    egui::Stroke::new(2.0, self.theme.bezel),
                ));

                painter.rect_filled(intake_port_rect, 8.0, self.theme.cyan.gamma_multiply(0.14));
                painter.rect_stroke(
                    intake_port_rect,
                    8.0,
                    egui::Stroke::new(1.2, self.theme.cyan),
                    egui::epaint::StrokeKind::Inside,
                );
                painter.rect_filled(exhaust_port_rect, 8.0, self.theme.red.gamma_multiply(0.14));
                painter.rect_stroke(
                    exhaust_port_rect,
                    8.0,
                    egui::Stroke::new(1.2, self.theme.red),
                    egui::epaint::StrokeKind::Inside,
                );
                painter.text(
                    intake_port_rect.center_top() + egui::vec2(0.0, 6.0),
                    egui::Align2::CENTER_TOP,
                    "INTAKE",
                    egui::FontId::monospace(10.0),
                    self.theme.cyan,
                );
                painter.text(
                    exhaust_port_rect.center_top() + egui::vec2(0.0, 6.0),
                    egui::Align2::CENTER_TOP,
                    "EXHAUST",
                    egui::FontId::monospace(10.0),
                    self.theme.red,
                );
                painter.line_segment(
                    [intake_port_rect.center_bottom(), egui::pos2(intake_stem_x - 12.0, seat_y - 20.0)],
                    egui::Stroke::new(1.6, self.theme.cyan),
                );
                painter.line_segment(
                    [exhaust_port_rect.center_bottom(), egui::pos2(exhaust_stem_x + 12.0, seat_y - 20.0)],
                    egui::Stroke::new(1.6, self.theme.red),
                );

                let guide_top_y = top_y + 18.0;
                let guide_bottom_y = seat_y - 18.0;
                painter.line_segment(
                    [
                        egui::pos2(intake_stem_x, guide_top_y),
                        egui::pos2(intake_stem_x, intake_head_y),
                    ],
                    egui::Stroke::new(2.2, self.theme.cyan),
                );
                painter.line_segment(
                    [
                        egui::pos2(exhaust_stem_x, guide_top_y),
                        egui::pos2(exhaust_stem_x, exhaust_head_y),
                    ],
                    egui::Stroke::new(2.2, self.theme.red),
                );
                painter.rect_filled(
                    egui::Rect::from_center_size(
                        egui::pos2(intake_stem_x, guide_top_y - 4.0),
                        egui::vec2(18.0, 8.0),
                    ),
                    2.0,
                    self.theme.cyan.gamma_multiply(0.25),
                );
                painter.rect_filled(
                    egui::Rect::from_center_size(
                        egui::pos2(exhaust_stem_x, guide_top_y - 4.0),
                        egui::vec2(18.0, 8.0),
                    ),
                    2.0,
                    self.theme.red.gamma_multiply(0.25),
                );
                painter.line_segment(
                    [egui::pos2(intake_stem_x, guide_top_y), egui::pos2(intake_stem_x, guide_bottom_y)],
                    egui::Stroke::new(1.0, self.theme.chrome.gamma_multiply(0.55)),
                );
                painter.line_segment(
                    [egui::pos2(exhaust_stem_x, guide_top_y), egui::pos2(exhaust_stem_x, guide_bottom_y)],
                    egui::Stroke::new(1.0, self.theme.chrome.gamma_multiply(0.55)),
                );

                painter.line_segment(
                    [egui::pos2(intake_stem_x - 15.0, seat_y), egui::pos2(intake_stem_x - 4.0, seat_y + 8.0)],
                    egui::Stroke::new(1.5, self.theme.chrome),
                );
                painter.line_segment(
                    [egui::pos2(intake_stem_x + 15.0, seat_y), egui::pos2(intake_stem_x + 4.0, seat_y + 8.0)],
                    egui::Stroke::new(1.5, self.theme.chrome),
                );
                painter.line_segment(
                    [egui::pos2(exhaust_stem_x - 15.0, seat_y), egui::pos2(exhaust_stem_x - 4.0, seat_y + 8.0)],
                    egui::Stroke::new(1.5, self.theme.chrome),
                );
                painter.line_segment(
                    [egui::pos2(exhaust_stem_x + 15.0, seat_y), egui::pos2(exhaust_stem_x + 4.0, seat_y + 8.0)],
                    egui::Stroke::new(1.5, self.theme.chrome),
                );
                painter.line_segment(
                    [egui::pos2(intake_stem_x - 12.0, intake_head_y), egui::pos2(intake_stem_x + 12.0, intake_head_y)],
                    egui::Stroke::new(4.0, self.theme.cyan),
                );
                painter.line_segment(
                    [egui::pos2(exhaust_stem_x - 12.0, exhaust_head_y), egui::pos2(exhaust_stem_x + 12.0, exhaust_head_y)],
                    egui::Stroke::new(4.0, self.theme.red),
                );

                painter.rect_filled(piston_rect, 5.0, self.theme.amber.gamma_multiply(0.90));
                painter.rect_stroke(
                    piston_rect,
                    5.0,
                    egui::Stroke::new(1.0, self.theme.amber),
                    egui::epaint::StrokeKind::Inside,
                );
                painter.line_segment([rod_top, crank_pin], egui::Stroke::new(2.6, self.theme.text_main));
                painter.circle_stroke(
                    crank_center,
                    crank_radius,
                    egui::Stroke::new(1.6, self.theme.chrome),
                );
                painter.line_segment([crank_center, crank_pin], egui::Stroke::new(2.2, self.theme.text_main));
                painter.circle_filled(crank_pin, 4.4, self.theme.text_main);
                painter.circle_filled(crank_center, 3.8, self.theme.chrome);
                painter.line_segment(
                    [
                        egui::pos2(center_x - 42.0, crank_center.y + 30.0),
                        egui::pos2(center_x + 42.0, crank_center.y + 30.0),
                    ],
                    egui::Stroke::new(1.2, self.theme.bezel),
                );

                let spark_plug_pos = egui::pos2(center_x, chamber_apex.y + 10.0);
                painter.line_segment(
                    [
                        egui::pos2(center_x, chamber_apex.y - 18.0),
                        egui::pos2(center_x, chamber_apex.y + 2.0),
                    ],
                    egui::Stroke::new(2.0, self.theme.text_soft),
                );
                painter.line_segment(
                    [
                        egui::pos2(center_x - 5.0, chamber_apex.y - 12.0),
                        egui::pos2(center_x + 5.0, chamber_apex.y - 12.0),
                    ],
                    egui::Stroke::new(1.2, self.theme.text_soft),
                );
                painter.circle_filled(spark_plug_pos, 3.0, self.theme.text_main);

                let chamber_clip = egui::Rect::from_min_max(
                    egui::pos2(bore_left + 2.0, chamber_apex.y - 4.0),
                    egui::pos2(bore_right - 2.0, piston_rect.top() - 2.0),
                );
                let chamber_painter = painter.with_clip_rect(chamber_clip);
                if ignition_flash {
                    for (angle_deg, length) in [
                        (-55.0_f32, 16.0_f32),
                        (-18.0_f32, 18.0_f32),
                        (20.0_f32, 18.0_f32),
                        (54.0_f32, 16.0_f32),
                    ] {
                        let angle = angle_deg.to_radians();
                        let tip = egui::pos2(
                            spark_plug_pos.x + length * angle.cos() as f32,
                            spark_plug_pos.y + length * angle.sin() as f32,
                        );
                        chamber_painter.line_segment(
                            [spark_plug_pos, tip],
                            egui::Stroke::new(1.8, self.theme.red),
                        );
                    }
                }
                if burn_active {
                    let max_radius_x = (bore_px * 0.44).max(18.0);
                    let max_radius_y =
                        ((piston_rect.top() - spark_plug_pos.y - 10.0).max(18.0)).min(stroke_px * 0.62);
                    let radius_x = egui::lerp(10.0..=max_radius_x, flame_radius_norm.powf(0.88));
                    let radius_y = egui::lerp(12.0..=max_radius_y, flame_radius_norm.powf(0.80));
                    let outer_front = flame_front_points(
                        spark_plug_pos,
                        radius_x,
                        radius_y,
                        bore_left,
                        bore_right,
                        chamber_apex,
                        deck_y,
                        piston_rect.top(),
                    );
                    let mid_front = flame_front_points(
                        spark_plug_pos,
                        radius_x * 0.68,
                        radius_y * 0.66,
                        bore_left,
                        bore_right,
                        chamber_apex,
                        deck_y,
                        piston_rect.top(),
                    );
                    let inner_front = flame_front_points(
                        spark_plug_pos,
                        radius_x * 0.36,
                        radius_y * 0.34,
                        bore_left,
                        bore_right,
                        chamber_apex,
                        deck_y,
                        piston_rect.top(),
                    );

                    chamber_painter.add(egui::Shape::convex_polygon(
                        outer_front.clone(),
                        self.theme.red.gamma_multiply(0.08 + 0.10 * (1.0 - flame_radius_norm)),
                        egui::Stroke::NONE,
                    ));
                    chamber_painter.add(egui::Shape::convex_polygon(
                        mid_front.clone(),
                        self.theme.amber.gamma_multiply(0.11 + 0.10 * (1.0 - flame_radius_norm)),
                        egui::Stroke::NONE,
                    ));
                    chamber_painter.add(egui::Shape::convex_polygon(
                        inner_front.clone(),
                        self.theme.red.gamma_multiply(0.18 + 0.10 * (1.0 - flame_radius_norm)),
                        egui::Stroke::NONE,
                    ));
                    chamber_painter.add(egui::Shape::closed_line(
                        outer_front,
                        egui::Stroke::new(2.0, self.theme.amber.gamma_multiply(0.95)),
                    ));
                    chamber_painter.add(egui::Shape::closed_line(
                        mid_front,
                        egui::Stroke::new(1.2, self.theme.red.gamma_multiply(0.78)),
                    ));
                    chamber_painter.circle_filled(
                        spark_plug_pos,
                        (8.0 + 8.0 * (1.0 - flame_radius_norm)).max(5.0),
                        self.theme.amber.gamma_multiply(0.82),
                    );
                }

                painter.text(
                    rect.left_top() + egui::vec2(18.0, 14.0),
                    egui::Align2::LEFT_TOP,
                    "CYL 1 CUTAWAY",
                    egui::FontId::monospace(10.5),
                    self.theme.amber,
                );
                painter.text(
                    egui::pos2(bore_left - 18.0, seat_y + 18.0),
                    egui::Align2::RIGHT_TOP,
                    "intake valve",
                    egui::FontId::monospace(10.0),
                    self.theme.cyan,
                );
                painter.text(
                    egui::pos2(bore_right + 18.0, seat_y + 18.0),
                    egui::Align2::LEFT_TOP,
                    "exhaust valve",
                    egui::FontId::monospace(10.0),
                    self.theme.red,
                );
                painter.text(
                    egui::pos2(center_x, piston_rect.bottom() + 12.0),
                    egui::Align2::CENTER_TOP,
                    "piston",
                    egui::FontId::monospace(10.0),
                    self.theme.amber,
                );
                painter.text(
                    egui::pos2(center_x, crank_center.y + 36.0),
                    egui::Align2::CENTER_TOP,
                    "crank",
                    egui::FontId::monospace(10.0),
                    self.theme.text_soft,
                );
            });

            ui.add_space(6.0);
            egui::Grid::new("engine_motion_metrics")
                .num_columns(2)
                .spacing(egui::vec2(16.0, 4.0))
                .show(ui, |ui| {
                    metric_row(
                        ui,
                        self.theme,
                        "Display phase",
                        format!("{:.0} degCA", local_cycle_deg),
                    );
                    metric_row(
                        ui,
                        self.theme,
                        "Current stroke",
                        current_stroke_label(local_cycle_deg).to_owned(),
                    );
                    metric_row(
                        ui,
                        self.theme,
                        "Bore / Stroke",
                        format!(
                            "{:.1} mm / {:.1} mm",
                            self.sim.params.bore_m * 1.0e3,
                            self.sim.params.stroke_m * 1.0e3
                        ),
                    );
                    metric_row(
                        ui,
                        self.theme,
                        "Intake / Exhaust lift",
                        format!("{:.1} mm / {:.1} mm", intake_lift_mm, exhaust_lift_mm),
                    );
                    metric_row(
                        ui,
                        self.theme,
                        "Intake / Exhaust VVT",
                        format!(
                            "{:.1} deg / {:.1} deg",
                            self.sim.control.vvt_intake_deg, self.sim.control.vvt_exhaust_deg
                        ),
                    );
                    metric_row(
                        ui,
                        self.theme,
                        "Spark / Burn start",
                        format!(
                            "{:.0} / {:.0} degCA",
                            spark_event_deg, self.latest.burn_start_deg
                        ),
                    );
                    metric_row(
                        ui,
                        self.theme,
                        "IVO / IVC",
                        format!("{:.0} / {:.0} degCA", intake_open_deg, intake_close_deg),
                    );
                    metric_row(
                        ui,
                        self.theme,
                        "EVO / EVC",
                        format!("{:.0} / {:.0} degCA", exhaust_open_deg, exhaust_close_deg),
                    );
                    metric_row(
                        ui,
                        self.theme,
                        "Actual / display phase",
                        format!("{:.0} / {:.0} degCA", self.latest.cycle_deg, local_cycle_deg),
                    );
                    metric_row(
                        ui,
                        self.theme,
                        "Combustion",
                        if burn_active {
                            format!(
                                "flame {:.0} % / {:.1} m/s",
                                flame_radius_norm * 100.0,
                                flame_speed_mps
                            )
                        } else if ignition_flash {
                            "spark discharge".to_owned()
                        } else if self.sim.control.fuel_cmd {
                            "fresh charge".to_owned()
                        } else {
                            "motoring".to_owned()
                        },
                    );
                });
        });
    }
}

#[cfg(test)]
mod tests {
    use std::collections::VecDeque;

    use super::{
        adaptive_plot_y_range, graph_layout_heights, recent_cycle_plot_lines,
        shortest_cycle_delta_deg, speed_hold_desired_net_torque_nm, wrap_cycle_deg,
    };
    use crate::config::AppConfig;
    use crate::simulator::CycleHistorySample;

    #[test]
    fn graph_layout_uses_positive_sizes() {
        let cfg = AppConfig::default();
        let layout = graph_layout_heights(&cfg.ui);

        assert!(layout.standard_plot_px >= 72.0);
        assert!(layout.pv_plot_px > layout.standard_plot_px);
        assert!(layout.section_spacing_px > 0.0);
    }

    #[test]
    fn adaptive_plot_y_range_tracks_positive_operating_window() {
        let (y_min, y_max) =
            adaptive_plot_y_range(Some((1_990.0, 2_010.0)), 150.0, 0.15, Some(0.0));

        assert!(
            y_min > 1_800.0,
            "lower bound should follow the operating point"
        );
        assert!(
            y_max < 2_200.0,
            "upper bound should stay near the operating point"
        );
    }

    #[test]
    fn adaptive_plot_y_range_keeps_signed_trace_centered() {
        let (y_min, y_max) = adaptive_plot_y_range(Some((145.0, 150.0)), 10.0, 0.15, None);

        assert!(
            y_min > 130.0,
            "signed plot should not be anchored near zero"
        );
        assert!(
            y_max < 170.0,
            "signed plot should keep a compact dynamic range"
        );
    }

    #[test]
    fn recent_cycle_plot_lines_keep_only_requested_cycles() {
        let history = VecDeque::from(vec![
            CycleHistorySample {
                cycle: 4,
                cycle_deg: 40.0,
                value: 10.0,
            },
            CycleHistorySample {
                cycle: 5,
                cycle_deg: 60.0,
                value: 20.0,
            },
            CycleHistorySample {
                cycle: 5,
                cycle_deg: 180.0,
                value: 22.0,
            },
            CycleHistorySample {
                cycle: 6,
                cycle_deg: 30.0,
                value: 30.0,
            },
            CycleHistorySample {
                cycle: 6,
                cycle_deg: 240.0,
                value: 32.0,
            },
        ]);

        let lines = recent_cycle_plot_lines(&history, 2);

        assert_eq!(lines.len(), 3);
        assert_eq!(lines[0], vec![[40.0, 10.0]]);
        assert_eq!(lines[1], vec![[60.0, 20.0], [180.0, 22.0]]);
        assert_eq!(lines[2], vec![[30.0, 30.0], [240.0, 32.0]]);
    }

    #[test]
    fn recent_cycle_plot_lines_default_to_latest_cycle() {
        let history = VecDeque::from(vec![
            CycleHistorySample {
                cycle: 1,
                cycle_deg: 10.0,
                value: 100.0,
            },
            CycleHistorySample {
                cycle: 2,
                cycle_deg: 20.0,
                value: 200.0,
            },
        ]);

        let lines = recent_cycle_plot_lines(&history, 1);

        assert_eq!(lines, vec![vec![[10.0, 100.0]], vec![[20.0, 200.0]]]);
    }

    #[test]
    fn cycle_wrap_helpers_follow_shortest_direction() {
        assert!((wrap_cycle_deg(725.0) - 5.0).abs() < 1.0e-12);
        assert!((shortest_cycle_delta_deg(710.0, 8.0) - 18.0).abs() < 1.0e-12);
        assert!((shortest_cycle_delta_deg(8.0, 710.0) + 18.0).abs() < 1.0e-12);
    }

    #[test]
    fn speed_hold_target_matches_shaft_torque_at_zero_error() {
        let desired = speed_hold_desired_net_torque_nm(0.0, 0.0);
        assert!(desired.abs() < 1.0e-12);
    }

    #[test]
    fn speed_hold_requests_positive_net_torque_below_target_speed() {
        let desired = speed_hold_desired_net_torque_nm(250.0, 120.0);
        assert!(desired > 0.0);
    }
}

impl eframe::App for DashboardApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        self.apply_shortcuts(ctx);
        self.advance_simulation();

        self.render_header_panel(ctx);
        self.render_control_rack(ctx);

        egui::CentralPanel::default().show(ctx, |ui| {
            egui::ScrollArea::both()
                .id_salt("dashboard_main_scroll")
                .auto_shrink([false, false])
                .show(ui, |ui| {
                    let graph_heights = graph_layout_heights(&self.ui_config);

                    self.render_console_overview(ui);
                    self.render_pressure_plots(ui, graph_heights);
                    self.render_cycle_monitors(ui, graph_heights);
                });
        });

        let frame_ms = 1000u64 / self.ui_config.repaint_hz.max(1) as u64;
        ctx.request_repaint_after(Duration::from_millis(frame_ms.max(1)));
    }
}

fn run_app_with_options(config: AppConfig, options: eframe::NativeOptions) -> eframe::Result {
    eframe::run_native(
        "ES Simulator Dashboard",
        options,
        Box::new(move |cc| {
            DashboardTheme::default().apply(&cc.egui_ctx);
            Ok(Box::new(DashboardApp::new(config.clone())))
        }),
    )
}

pub(crate) fn run_app(config: AppConfig) -> eframe::Result {
    // Window size comes from YAML so different machine setups can keep their preferred layout.
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([config.ui.window_width_px, config.ui.window_height_px]),
        ..Default::default()
    };

    run_app_with_options(config, options)
}
