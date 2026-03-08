use std::time::{Duration, Instant};

use eframe::egui;
use egui_plot::{Legend, Line, MarkerShape, Plot, PlotBounds, PlotPoints, Points, VLine};

use crate::audio::{AudioEngine, AudioParams};
use crate::config::{AppConfig, BenchMixtureMode, UiConfig};
use crate::constants::FIXED_CYLINDER_COUNT;
use crate::simulator::{
    BenchSample, BenchSession, BenchStatus, ControlInput, CycleHistorySample, Simulator,
    cam_profile_points, estimate_realtime_dt_floor, quick_wot_bench_preview_curve, rpm_linked_dt,
    shaft_power_hp, shaft_power_kw,
};

#[derive(Default)]
struct BenchRunner {
    session: Option<BenchSession>,
    preview_results: Vec<BenchSample>,
}

impl BenchRunner {
    fn start(&mut self, config: AppConfig, control_seed: ControlInput, mode: BenchMixtureMode) {
        self.preview_results = quick_wot_bench_preview_curve(&config, &control_seed, mode);
        self.session = Some(BenchSession::new(&config, control_seed, mode));
    }

    fn stop(&mut self) {
        if let Some(session) = &mut self.session {
            session.stop();
        }
        self.preview_results.clear();
    }

    fn status(&self) -> Option<BenchStatus> {
        self.session.as_ref().map(BenchSession::status)
    }

    fn is_active(&self) -> bool {
        self.session.as_ref().is_some_and(BenchSession::is_active)
    }

    fn is_complete(&self) -> bool {
        self.session.as_ref().is_some_and(BenchSession::is_complete)
    }

    fn results(&self) -> &[BenchSample] {
        self.session
            .as_ref()
            .map(BenchSession::results)
            .unwrap_or(&[])
    }

    fn preview_results(&self) -> &[BenchSample] {
        &self.preview_results
    }

    fn live_sample(&self) -> Option<BenchSample> {
        self.session.as_ref().and_then(BenchSession::live_sample)
    }

    fn advance_steps(&mut self, max_steps: usize) {
        if let Some(session) = &mut self.session {
            session.advance_steps(max_steps);
        }
    }

    fn advance_budgeted(&mut self, max_steps: usize, frame_time_budget_ms: f64) {
        if max_steps == 0 || frame_time_budget_ms <= 0.0 || !self.is_active() {
            return;
        }
        let t0 = Instant::now();
        let budget_s = frame_time_budget_ms * 1.0e-3;
        let mut steps = 0usize;
        while steps < max_steps && self.is_active() && t0.elapsed().as_secs_f64() < budget_s {
            self.advance_steps(1);
            steps = steps.saturating_add(1);
        }
    }
}

#[derive(Debug, Clone)]
struct BenchCurvePlotData {
    summary: String,
    label: String,
    curve_points: Vec<[f64; 2]>,
    preview_curve_points: Vec<[f64; 2]>,
    completed_points: Vec<[f64; 2]>,
    origin_anchor: Option<[f64; 2]>,
    peak_torque_point: Option<[f64; 2]>,
    live_point: Option<[f64; 2]>,
    power_curve_points: Vec<[f64; 2]>,
    preview_power_curve_points: Vec<[f64; 2]>,
    completed_power_points: Vec<[f64; 2]>,
    peak_power_point: Option<[f64; 2]>,
    live_power_point: Option<[f64; 2]>,
    x_min: f64,
    x_max: f64,
    y_min: f64,
    y_max: f64,
    power_y_min: f64,
    power_y_max: f64,
}

#[derive(Debug, Clone, Copy)]
struct GraphLayoutHeights {
    standard_plot_px: f32,
    pv_plot_px: f32,
    bench_torque_plot_px: f32,
    bench_power_plot_px: f32,
    section_spacing_px: f32,
}

fn graph_layout_heights(ui_config: &UiConfig, bench_visible: bool) -> GraphLayoutHeights {
    let scale = if bench_visible { 0.62 } else { 1.0 };
    let standard_plot_px = (ui_config.plot_height_px * scale).max(72.0);
    let pv_plot_px = (ui_config.pv_plot_height_px * scale).max(standard_plot_px * 1.75);
    let bench_torque_multiplier = if bench_visible { 1.4 } else { 2.0 };
    let bench_power_multiplier = if bench_visible { 1.05 } else { 1.35 };

    GraphLayoutHeights {
        standard_plot_px,
        pv_plot_px,
        bench_torque_plot_px: (ui_config.plot_height_px * bench_torque_multiplier * scale)
            .max(standard_plot_px * 1.25),
        bench_power_plot_px: (ui_config.plot_height_px * bench_power_multiplier * scale)
            .max(standard_plot_px),
        section_spacing_px: if bench_visible { 4.0 } else { 8.0 },
    }
}

fn bench_curve_plot_data(
    bench_results: &[BenchSample],
    preview_results: &[BenchSample],
    live_bench: Option<BenchSample>,
    bench_config: &crate::config::BenchConfig,
    ui_config: &UiConfig,
) -> BenchCurvePlotData {
    fn sort_points(mut points: Vec<[f64; 2]>) -> Vec<[f64; 2]> {
        points.sort_by(|a, b| a[0].total_cmp(&b[0]));
        points
    }

    let axis_x_min = bench_config
        .display_rpm_min
        .max(0.0)
        .min(bench_config.rpm_end_rpm.max(0.0));

    let completed_points: Vec<[f64; 2]> = sort_points(
        bench_results
            .iter()
            .map(|sample| [sample.rpm, sample.torque_net_nm])
            .collect(),
    );
    let preview_curve_points: Vec<[f64; 2]> = sort_points(
        preview_results
            .iter()
            .map(|sample| [sample.rpm, sample.torque_net_nm])
            .collect(),
    );
    let completed_power_points: Vec<[f64; 2]> = sort_points(
        bench_results
            .iter()
            .map(|sample| [sample.rpm, shaft_power_kw(sample.rpm, sample.torque_net_nm)])
            .collect(),
    );
    let preview_power_curve_points: Vec<[f64; 2]> = sort_points(
        preview_results
            .iter()
            .map(|sample| [sample.rpm, shaft_power_kw(sample.rpm, sample.torque_net_nm)])
            .collect(),
    );

    let live_point = live_bench.map(|sample| [sample.rpm, sample.torque_net_nm]);
    let live_power_point =
        live_bench.map(|sample| [sample.rpm, shaft_power_kw(sample.rpm, sample.torque_net_nm)]);
    let has_curve_data =
        !completed_points.is_empty() || !preview_curve_points.is_empty() || live_point.is_some();
    let origin_anchor =
        (bench_config.include_zero_rpm_anchor && axis_x_min <= f64::EPSILON && has_curve_data)
            .then_some([0.0, 0.0]);
    let mut curve_points = Vec::with_capacity(
        completed_points.len()
            + usize::from(live_point.is_some())
            + usize::from(origin_anchor.is_some()),
    );
    let mut power_curve_points = Vec::with_capacity(
        completed_power_points.len()
            + usize::from(live_power_point.is_some())
            + usize::from(origin_anchor.is_some()),
    );
    if let Some(anchor) = origin_anchor {
        curve_points.push(anchor);
        power_curve_points.push(anchor);
    }
    curve_points.extend(completed_points.iter().copied());
    power_curve_points.extend(completed_power_points.iter().copied());
    if let Some(point) = live_point {
        let duplicate_existing = curve_points.iter().any(|existing| {
            (existing[0] - point[0]).abs() <= f64::EPSILON
                && (existing[1] - point[1]).abs() <= f64::EPSILON
        });
        if !duplicate_existing {
            curve_points.push(point);
            curve_points.sort_by(|a, b| a[0].total_cmp(&b[0]));
        }
    }
    if let Some(point) = live_power_point {
        let duplicate_existing = power_curve_points.iter().any(|existing| {
            (existing[0] - point[0]).abs() <= f64::EPSILON
                && (existing[1] - point[1]).abs() <= f64::EPSILON
        });
        if !duplicate_existing {
            power_curve_points.push(point);
            power_curve_points.sort_by(|a, b| a[0].total_cmp(&b[0]));
        }
    }

    let peak_torque_sample = bench_results
        .iter()
        .max_by(|a, b| a.torque_net_nm.total_cmp(&b.torque_net_nm))
        .copied();
    let peak_power_sample = bench_results
        .iter()
        .max_by(|a, b| {
            shaft_power_kw(a.rpm, a.torque_net_nm)
                .total_cmp(&shaft_power_kw(b.rpm, b.torque_net_nm))
        })
        .copied();
    let preview_peak_torque_sample = preview_results
        .iter()
        .max_by(|a, b| a.torque_net_nm.total_cmp(&b.torque_net_nm))
        .copied();
    let preview_peak_power_sample = preview_results
        .iter()
        .max_by(|a, b| {
            shaft_power_kw(a.rpm, a.torque_net_nm)
                .total_cmp(&shaft_power_kw(b.rpm, b.torque_net_nm))
        })
        .copied();
    let peak_torque_point = peak_torque_sample
        .or(preview_peak_torque_sample)
        .map(|sample| [sample.rpm, sample.torque_net_nm]);
    let peak_power_point = peak_power_sample
        .or(preview_peak_power_sample)
        .map(|sample| [sample.rpm, shaft_power_kw(sample.rpm, sample.torque_net_nm)]);

    let summary = if let (Some(peak_torque), Some(peak_power)) =
        (peak_torque_sample, peak_power_sample)
    {
        format!(
            "Dyno curves: peak torque {:.1} Nm @ {:.0} rpm, peak power {:.1} kW ({:.1} hp) @ {:.0} rpm ({})",
            peak_torque.torque_net_nm,
            peak_torque.rpm,
            shaft_power_kw(peak_power.rpm, peak_power.torque_net_nm),
            shaft_power_hp(peak_power.rpm, peak_power.torque_net_nm),
            peak_power.rpm,
            peak_torque.mixture_mode.label()
        )
    } else if let (Some(peak_torque), Some(peak_power)) =
        (preview_peak_torque_sample, preview_peak_power_sample)
    {
        format!(
            "Dyno preview: peak torque {:.1} Nm @ {:.0} rpm, peak power {:.1} kW ({:.1} hp) @ {:.0} rpm ({})",
            peak_torque.torque_net_nm,
            peak_torque.rpm,
            shaft_power_kw(peak_power.rpm, peak_power.torque_net_nm),
            shaft_power_hp(peak_power.rpm, peak_power.torque_net_nm),
            peak_power.rpm,
            peak_torque.mixture_mode.label()
        )
    } else if let Some(sample) = live_bench {
        format!(
            "Dyno curves: measuring {:.0} rpm / {:.1} Nm / {:.1} kW ({})",
            sample.rpm,
            sample.torque_net_nm,
            shaft_power_kw(sample.rpm, sample.torque_net_nm),
            sample.mixture_mode.label()
        )
    } else {
        "Dyno curves: run bench to populate".to_owned()
    };

    let label = bench_results
        .first()
        .map(|sample| sample.mixture_mode.label())
        .or_else(|| {
            preview_results
                .first()
                .map(|sample| sample.mixture_mode.label())
        })
        .or_else(|| live_bench.map(|sample| sample.mixture_mode.label()))
        .unwrap_or("no data")
        .to_owned();

    let mut torque_min = 0.0_f64;
    let mut torque_max = 0.0_f64;
    for point in completed_points
        .iter()
        .copied()
        .chain(preview_curve_points.iter().copied())
        .chain(live_point)
    {
        torque_min = torque_min.min(point[1]);
        torque_max = torque_max.max(point[1]);
    }
    let torque_span = (torque_max - torque_min).max(ui_config.torque_min_span_nm);
    let y_min = if torque_min >= 0.0 {
        0.0
    } else {
        torque_min - ui_config.torque_margin_ratio * torque_span
    };
    let y_max = (torque_max + ui_config.torque_margin_ratio * torque_span)
        .max(ui_config.torque_floor_abs_nm);

    let x_min = axis_x_min;
    let x_max = bench_config.rpm_end_rpm.max(x_min + 1.0);
    let power_max = completed_power_points
        .iter()
        .chain(preview_power_curve_points.iter())
        .chain(live_power_point.iter())
        .map(|point| point[1])
        .fold(0.0_f64, f64::max);
    let power_y_max =
        (power_max * (1.0 + ui_config.torque_margin_ratio)).max(ui_config.torque_floor_abs_nm);

    BenchCurvePlotData {
        summary,
        label,
        curve_points,
        preview_curve_points,
        completed_points,
        origin_anchor,
        peak_torque_point,
        live_point,
        power_curve_points,
        preview_power_curve_points,
        completed_power_points,
        peak_power_point,
        live_power_point,
        x_min,
        x_max,
        y_min,
        y_max,
        power_y_min: 0.0,
        power_y_max,
    }
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

fn cylinder_trace_color(index: usize) -> egui::Color32 {
    const COLORS: [egui::Color32; 4] = [
        egui::Color32::from_rgb(255, 170, 40),
        egui::Color32::from_rgb(90, 180, 255),
        egui::Color32::from_rgb(110, 220, 150),
        egui::Color32::from_rgb(255, 105, 125),
    ];
    COLORS[index % COLORS.len()]
}

// The dashboard owns real-time pacing, user controls, and fixed-range plots.
struct DashboardApp {
    config: AppConfig,
    sim: Simulator,
    ui_config: UiConfig,
    latest: crate::simulator::Observation,
    last_tick: Instant,
    audio: Option<AudioEngine>,
    audio_output_gain: f32,
    dt_base: f64,
    dt_next: f64,
    dt_min_bound: f64,
    dt_max_bound: f64,
    bench: BenchRunner,
    bench_mode_selected: BenchMixtureMode,
}

impl DashboardApp {
    fn new(config: AppConfig) -> Self {
        let ui_config = config.ui.clone();
        let bench_mode_selected = config.bench.default_mode;
        let dt = config.environment.dt.max(ui_config.min_base_dt_s);
        // Benchmark a safe realtime floor once at startup so the adaptive dt stays interactive.
        let realtime_floor_dt = estimate_realtime_dt_floor(&config, dt);
        let dt_min_bound = realtime_floor_dt.max(dt * ui_config.realtime_dt_min_factor);
        let dt_max_bound = (dt * ui_config.realtime_dt_max_factor)
            .max(dt_min_bound * ui_config.realtime_dt_max_over_min_factor);
        let dt_next = dt.clamp(dt_min_bound, dt_max_bound);
        let audio_output_gain = config.audio.output_gain.max(ui_config.audio_gain_floor);
        let mut sim = Simulator::new(&config);
        let latest = sim.step(dt_next);
        let initial_audio_params = AudioParams {
            exhaust_pressure_kpa: (config.environment.ambient_pressure_pa * 1.0e-3) as f32,
            exhaust_runner_pressure_kpa: (config.environment.ambient_pressure_pa * 1.0e-3) as f32,
            intake_runner_pressure_kpa: (config.environment.ambient_pressure_pa * 1.0e-3) as f32,
            exhaust_runner_flow_gps: 0.0,
            engine_speed_rpm: latest.rpm as f32,
            exhaust_temp_k: config.environment.exhaust_temp_k as f32,
            output_gain: audio_output_gain,
        };
        let audio = AudioEngine::new(&config.audio, initial_audio_params);
        Self {
            config,
            sim,
            ui_config,
            latest,
            last_tick: Instant::now(),
            audio,
            audio_output_gain,
            dt_base: dt,
            dt_next,
            dt_min_bound,
            dt_max_bound,
            bench: BenchRunner::default(),
            bench_mode_selected,
        }
    }

    fn bench_control_seed(&self) -> ControlInput {
        let mut seed = self.sim.control.clone();
        if self.sim.auto.wot_best_eta > 0.0 {
            seed.ignition_timing_deg = self.sim.auto.wot_best_point.ignition_deg;
            seed.vvt_intake_deg = self.sim.auto.wot_best_point.vvt_intake_deg;
            seed.vvt_exhaust_deg = self.sim.auto.wot_best_point.vvt_exhaust_deg;
        }
        seed
    }

    fn start_bench(&mut self, mode: BenchMixtureMode) {
        let mut cfg = self.config.clone();
        cfg.engine.max_rpm = self.sim.params.max_rpm;
        self.bench.start(cfg, self.bench_control_seed(), mode);
    }

    fn apply_shortcuts(&mut self, ctx: &egui::Context) {
        ctx.input(|i| {
            if i.key_pressed(egui::Key::Q) {
                ctx.send_viewport_cmd(egui::ViewportCommand::Close);
            }
            if i.key_pressed(egui::Key::A) {
                let next = !self.sim.auto.enabled;
                self.sim.set_idle_auto_enabled(next);
            }
            if i.key_pressed(egui::Key::O) {
                let next = !self.sim.auto.wot_efficiency_enabled;
                self.sim.set_wot_efficiency_auto_enabled(next);
            }
            if i.key_pressed(egui::Key::B) {
                if self.bench.is_active() {
                    self.bench.stop();
                } else {
                    self.start_bench(self.bench_mode_selected);
                }
            }
            if i.key_pressed(egui::Key::L) {
                self.bench_mode_selected = BenchMixtureMode::LambdaOne;
                if self.bench.is_active() {
                    self.bench.stop();
                } else {
                    self.start_bench(BenchMixtureMode::LambdaOne);
                }
            }
            if i.key_pressed(egui::Key::S) {
                self.sim.disable_auto_modes();
                self.sim.control.starter_cmd = !self.sim.control.starter_cmd;
            }
            if i.key_pressed(egui::Key::I) {
                self.sim.disable_auto_modes();
                self.sim.control.spark_cmd = !self.sim.control.spark_cmd;
            }
            if i.key_pressed(egui::Key::F) {
                self.sim.disable_auto_modes();
                self.sim.control.fuel_cmd = !self.sim.control.fuel_cmd;
            }
            if i.key_pressed(egui::Key::W) {
                self.sim.disable_auto_modes();
                self.sim.control.throttle_cmd = (self.sim.control.throttle_cmd
                    + self.ui_config.throttle_key_step)
                    .clamp(0.0, 1.0);
            }
            if i.key_pressed(egui::Key::X) {
                self.sim.disable_auto_modes();
                self.sim.control.throttle_cmd = (self.sim.control.throttle_cmd
                    - self.ui_config.throttle_key_step)
                    .clamp(0.0, 1.0);
            }
        });
    }

    fn advance_simulation(&mut self) {
        let now = Instant::now();
        let elapsed = (now - self.last_tick).as_secs_f64();
        let target_time = elapsed.max(self.dt_base);
        let mut simulated = 0.0;
        let mut steps = 0usize;
        let max_steps = self.ui_config.max_steps_per_frame.max(1);
        // Consume elapsed wall time with multiple solver steps so GUI frame rate and ODE dt decouple.
        while simulated < target_time && steps < max_steps {
            let dt_nom = rpm_linked_dt(
                self.dt_base,
                self.latest.rpm,
                self.sim.params.idle_target_rpm,
                &self.sim.numerics,
            );
            let dt_target = dt_nom.clamp(self.dt_min_bound, self.dt_max_bound);
            self.dt_next += self.ui_config.dt_smoothing_factor * (dt_target - self.dt_next);
            self.dt_next = self.dt_next.clamp(self.dt_min_bound, self.dt_max_bound);

            let remaining = target_time - simulated;
            let dt_step = self.dt_next.min(remaining).max(self.ui_config.dt_epsilon_s);
            self.latest = self.sim.step(dt_step);
            simulated += dt_step;
            steps = steps.saturating_add(1);
        }
        if simulated < target_time {
            // If the frame budget was large, finish the leftover with one bounded catch-up step.
            let dt_tail = (target_time - simulated)
                .max(self.ui_config.dt_epsilon_s)
                .min(self.dt_max_bound);
            self.latest = self.sim.step(dt_tail);
            self.dt_next = dt_tail.clamp(self.dt_min_bound, self.dt_max_bound);
        }
        self.last_tick = now;

        if let Some(audio) = &self.audio {
            // Audio only needs the latest reduced state, not the full simulator internals.
            audio.update(AudioParams {
                exhaust_pressure_kpa: self.latest.exhaust_kpa as f32,
                exhaust_runner_pressure_kpa: self.latest.exhaust_runner_kpa as f32,
                intake_runner_pressure_kpa: self.latest.intake_runner_kpa as f32,
                exhaust_runner_flow_gps: self.latest.exhaust_runner_flow_gps as f32,
                engine_speed_rpm: self.latest.rpm as f32,
                exhaust_temp_k: self.latest.exhaust_temp_k as f32,
                output_gain: self.audio_output_gain,
            });
        }
    }

    fn advance_bench(&mut self) {
        if self.bench.is_active() {
            self.bench.advance_budgeted(
                self.config.bench.steps_per_frame.max(1),
                self.config.bench.frame_time_budget_ms,
            );
        }
    }

    fn bench_curve_visible(&self) -> bool {
        !self.bench.results().is_empty()
            || !self.bench.preview_results().is_empty()
            || self.bench.live_sample().is_some()
    }

    fn render_bench_curve(&self, ui: &mut egui::Ui, graph_heights: GraphLayoutHeights) {
        let bench_results = self.bench.results();
        let bench_preview = self.bench.preview_results();
        let live_bench = self.bench.live_sample();
        if bench_results.is_empty() && bench_preview.is_empty() && live_bench.is_none() {
            return;
        }
        let plot = bench_curve_plot_data(
            bench_results,
            bench_preview,
            live_bench,
            &self.config.bench,
            &self.ui_config,
        );

        ui.group(|ui| {
            ui.label(&plot.summary);

            Plot::new("bench_torque_curve_plot")
                .height(graph_heights.bench_torque_plot_px)
                .allow_scroll(false)
                .allow_drag(false)
                .allow_zoom(false)
                .include_x(plot.x_min)
                .include_y(0.0)
                .legend(Legend::default())
                .x_axis_label("Engine speed [rpm]")
                .y_axis_label("Net torque [Nm]")
                .show(ui, |plot_ui| {
                    plot_ui.set_auto_bounds(false);
                    plot_ui.set_plot_bounds(PlotBounds::from_min_max(
                        [plot.x_min, plot.y_min],
                        [plot.x_max, plot.y_max],
                    ));

                    if plot.preview_curve_points.len() >= 2 {
                        plot_ui.line(
                            Line::new(plot.preview_curve_points.clone())
                                .name(format!("Preview torque [{}]", plot.label))
                                .color(egui::Color32::from_gray(120))
                                .width(self.ui_config.line_width_px),
                        );
                    }

                    if plot.curve_points.len() >= 2 {
                        plot_ui.line(
                            Line::new(plot.curve_points.clone())
                                .name(format!("Torque curve [{}]", plot.label))
                                .color(egui::Color32::from_rgb(255, 200, 80))
                                .width(self.ui_config.line_width_px + 0.5),
                        );
                    }

                    if !plot.completed_points.is_empty() {
                        plot_ui.points(
                            Points::new(plot.completed_points.clone())
                                .name("Bench curve points")
                                .shape(MarkerShape::Circle)
                                .radius(3.5)
                                .color(egui::Color32::from_rgb(255, 200, 80)),
                        );
                    }

                    if let Some(anchor) = plot.origin_anchor {
                        plot_ui.points(
                            Points::new(vec![anchor])
                                .name("0 rpm anchor")
                                .shape(MarkerShape::Circle)
                                .radius(2.5)
                                .color(egui::Color32::from_gray(120)),
                        );
                    }

                    if let Some(peak_point) = plot.peak_torque_point {
                        let peak_point = Points::new(vec![peak_point])
                            .name("Peak torque")
                            .shape(MarkerShape::Diamond)
                            .radius(5.5)
                            .color(egui::Color32::from_rgb(255, 235, 140));
                        plot_ui.points(peak_point);
                    }

                    if let Some(live_point) = plot.live_point {
                        plot_ui.vline(
                            VLine::new(live_point[0])
                                .name("Current RPM")
                                .color(egui::Color32::from_rgb(255, 120, 40)),
                        );
                        let live_point = Points::new(vec![live_point])
                            .name("Current point")
                            .shape(MarkerShape::Circle)
                            .radius(5.0)
                            .color(egui::Color32::from_rgb(255, 120, 40));
                        plot_ui.points(live_point);
                    }
                });

            Plot::new("bench_power_curve_plot")
                .height(graph_heights.bench_power_plot_px)
                .allow_scroll(false)
                .allow_drag(false)
                .allow_zoom(false)
                .include_x(plot.x_min)
                .include_y(0.0)
                .legend(Legend::default())
                .x_axis_label("Engine speed [rpm]")
                .y_axis_label("Brake power [kW]")
                .show(ui, |plot_ui| {
                    plot_ui.set_auto_bounds(false);
                    plot_ui.set_plot_bounds(PlotBounds::from_min_max(
                        [plot.x_min, plot.power_y_min],
                        [plot.x_max, plot.power_y_max],
                    ));

                    if plot.preview_power_curve_points.len() >= 2 {
                        plot_ui.line(
                            Line::new(plot.preview_power_curve_points.clone())
                                .name(format!("Preview power [{}]", plot.label))
                                .color(egui::Color32::from_gray(120))
                                .width(self.ui_config.line_width_px),
                        );
                    }

                    if plot.power_curve_points.len() >= 2 {
                        plot_ui.line(
                            Line::new(plot.power_curve_points.clone())
                                .name(format!("Power curve [{}]", plot.label))
                                .color(egui::Color32::from_rgb(120, 210, 255))
                                .width(self.ui_config.line_width_px + 0.5),
                        );
                    }

                    if !plot.completed_power_points.is_empty() {
                        plot_ui.points(
                            Points::new(plot.completed_power_points.clone())
                                .name("Bench power points")
                                .shape(MarkerShape::Circle)
                                .radius(3.5)
                                .color(egui::Color32::from_rgb(120, 210, 255)),
                        );
                    }

                    if let Some(peak_power_point) = plot.peak_power_point {
                        plot_ui.points(
                            Points::new(vec![peak_power_point])
                                .name("Peak power")
                                .shape(MarkerShape::Diamond)
                                .radius(5.5)
                                .color(egui::Color32::from_rgb(170, 240, 255)),
                        );
                    }

                    if let Some(live_power_point) = plot.live_power_point {
                        plot_ui.vline(
                            VLine::new(live_power_point[0])
                                .name("Current RPM")
                                .color(egui::Color32::from_rgb(80, 170, 255)),
                        );
                        plot_ui.points(
                            Points::new(vec![live_power_point])
                                .name("Current power")
                                .shape(MarkerShape::Circle)
                                .radius(5.0)
                                .color(egui::Color32::from_rgb(80, 170, 255)),
                        );
                    }
                });
        });
        ui.add_space(graph_heights.section_spacing_px);
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

        ui.group(|ui| {
            Plot::new("pv_plot")
                .height(graph_heights.pv_plot_px)
                .allow_scroll(false)
                .allow_drag(false)
                .allow_zoom(false)
                .legend(Legend::default())
                .x_axis_label("Normalized volume [-]")
                .y_axis_label("Pressure [kPa]")
                .show(ui, |plot_ui| {
                    // Keep the configured p-V window visible, but allow extra headroom for peaks.
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

        ui.group(|ui| {
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
}

#[cfg(test)]
mod tests {
    use std::collections::VecDeque;

    use super::{
        adaptive_plot_y_range, bench_curve_plot_data, graph_layout_heights, recent_cycle_plot_lines,
    };
    use crate::config::{AppConfig, BenchMixtureMode};
    use crate::simulator::{BenchSample, BenchSession, ControlInput, CycleHistorySample};

    fn sample(rpm: f64, torque_net_nm: f64, mode: BenchMixtureMode) -> BenchSample {
        BenchSample {
            rpm,
            torque_net_nm,
            map_kpa: 95.0,
            eta_thermal_indicated_pv: 0.30,
            lambda_target: 1.0,
            intake_charge_temp_k: 300.0,
            mixture_mode: mode,
        }
    }

    #[test]
    fn bench_curve_plot_uses_live_point_before_first_completed_sample() {
        let cfg = AppConfig::default();
        let live = sample(3_500.0, 152.0, BenchMixtureMode::LambdaOne);

        let plot = bench_curve_plot_data(&[], &[], Some(live), &cfg.bench, &cfg.ui);

        assert_eq!(plot.live_point, Some([3_500.0, 152.0]));
        assert!(plot.completed_points.is_empty());
        assert_eq!(plot.origin_anchor, Some([0.0, 0.0]));
        assert_eq!(plot.curve_points.len(), 2);
        assert_eq!(plot.curve_points[0], [0.0, 0.0]);
        assert_eq!(plot.power_curve_points[0], [0.0, 0.0]);
        assert!(plot.y_max > 152.0);
        assert_eq!(plot.x_min, 0.0);
        assert_eq!(plot.x_max, cfg.bench.rpm_end_rpm);
    }

    #[test]
    fn bench_curve_plot_sorts_samples_and_marks_peak() {
        let cfg = AppConfig::default();
        let results = vec![
            sample(4_000.0, 188.0, BenchMixtureMode::RichChargeCooling),
            sample(2_000.0, 142.0, BenchMixtureMode::RichChargeCooling),
            sample(3_000.0, 171.0, BenchMixtureMode::RichChargeCooling),
        ];

        let plot = bench_curve_plot_data(&results, &[], None, &cfg.bench, &cfg.ui);
        let rpms: Vec<f64> = plot.curve_points.iter().map(|point| point[0]).collect();

        assert_eq!(rpms, vec![0.0, 2_000.0, 3_000.0, 4_000.0]);
        assert_eq!(plot.peak_torque_point, Some([4_000.0, 188.0]));
        assert!(plot.peak_power_point.is_some());
        assert_eq!(plot.completed_points.len(), 3);
        assert_eq!(plot.origin_anchor, Some([0.0, 0.0]));
    }

    #[test]
    fn bench_session_feeds_visible_curve_plot_data() {
        let mut cfg = AppConfig::default();
        cfg.bench.rpm_start_rpm = 1_500.0;
        cfg.bench.rpm_end_rpm = 2_000.0;
        cfg.bench.rpm_step_rpm = 500.0;
        cfg.bench.settle_time_s = 0.01;
        cfg.bench.average_time_s = 0.01;
        cfg.bench.locked_cycle_samples = 64;

        let mut session =
            BenchSession::new(&cfg, ControlInput::default(), BenchMixtureMode::LambdaOne);

        session.advance_steps(64);
        let plot = bench_curve_plot_data(
            session.results(),
            &[],
            session.live_sample(),
            &cfg.bench,
            &cfg.ui,
        );

        assert!(
            plot.live_point.is_some() || !plot.completed_points.is_empty(),
            "bench plot should have a live or completed point while session has started"
        );
        assert!(plot.y_max >= cfg.ui.torque_floor_abs_nm);
        assert!(plot.x_max > plot.x_min);
    }

    #[test]
    fn graph_layout_compacts_when_bench_is_visible() {
        let cfg = AppConfig::default();

        let normal = graph_layout_heights(&cfg.ui, false);
        let compact = graph_layout_heights(&cfg.ui, true);

        assert!(compact.standard_plot_px < normal.standard_plot_px);
        assert!(compact.pv_plot_px < normal.pv_plot_px);
        assert!(compact.bench_torque_plot_px < normal.bench_torque_plot_px);
        assert!(compact.bench_power_plot_px < normal.bench_power_plot_px);
        assert!(compact.section_spacing_px < normal.section_spacing_px);
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
}

impl eframe::App for DashboardApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        self.apply_shortcuts(ctx);
        self.advance_simulation();
        self.advance_bench();

        egui::TopBottomPanel::top("header").show(ctx, |ui| {
            ui.heading("4-stroke Engine Physics Simulator");
            ui.label(
                "Shortcut: [A] auto-start+idle [O] auto-WOT-efficiency [B] auto-bench(selected) [L] lambda=1 bench [S] starter [I] spark [F] fuel [W/X] throttle [Q] quit",
            );
        });

        egui::SidePanel::left("controls").show(ctx, |ui| {
            ui.heading("Controls");
            let mut idle_auto = self.sim.auto.enabled;
            if ui.checkbox(&mut idle_auto, "Auto Start + Idle").changed() {
                self.sim.set_idle_auto_enabled(idle_auto);
            }
            let mut wot_auto = self.sim.auto.wot_efficiency_enabled;
            if ui
                .checkbox(&mut wot_auto, "Auto WOT Efficiency Search")
                .changed()
            {
                self.sim.set_wot_efficiency_auto_enabled(wot_auto);
            }
            egui::ComboBox::from_label("Bench mode")
                .selected_text(self.bench_mode_selected.label())
                .show_ui(ui, |ui| {
                    ui.selectable_value(
                        &mut self.bench_mode_selected,
                        BenchMixtureMode::RichChargeCooling,
                        BenchMixtureMode::RichChargeCooling.label(),
                    );
                    ui.selectable_value(
                        &mut self.bench_mode_selected,
                        BenchMixtureMode::LambdaOne,
                        BenchMixtureMode::LambdaOne.label(),
                    );
                });
            ui.horizontal(|ui| {
                if ui.button("Run Bench").clicked() {
                    self.start_bench(self.bench_mode_selected);
                }
                if self.bench.is_active() && ui.button("Stop Bench").clicked() {
                    self.bench.stop();
                }
            });
            ui.separator();
            let auto_actuator_locked =
                self.sim.auto.enabled || self.sim.auto.wot_efficiency_enabled;
            let calibration_locked = self.sim.auto.wot_efficiency_enabled;

            ui.add_enabled(
                !auto_actuator_locked,
                egui::Slider::new(&mut self.sim.control.throttle_cmd, 0.0..=1.0)
                    .text("Throttle cmd"),
            );
            ui.add(
                egui::Slider::new(&mut self.sim.control.load_cmd, 0.0..=1.0)
                    .text("Load cmd"),
            );
            ui.add_enabled(
                !auto_actuator_locked,
                egui::Checkbox::new(&mut self.sim.control.starter_cmd, "Starter"),
            );
            ui.add_enabled(
                !auto_actuator_locked,
                egui::Checkbox::new(&mut self.sim.control.spark_cmd, "Spark"),
            );
            ui.add_enabled(
                !auto_actuator_locked,
                egui::Checkbox::new(&mut self.sim.control.fuel_cmd, "Fuel"),
            );

            ui.add_enabled(
                !calibration_locked,
                egui::Slider::new(
                    &mut self.sim.control.vvt_intake_deg,
                    self.ui_config.vvt_slider_min_deg..=self.ui_config.vvt_slider_max_deg,
                )
                .text("VVT Intake [deg]"),
            );
            ui.add_enabled(
                !calibration_locked,
                egui::Slider::new(
                    &mut self.sim.control.vvt_exhaust_deg,
                    self.ui_config.vvt_slider_min_deg..=self.ui_config.vvt_slider_max_deg,
                )
                .text("VVT Exhaust [deg]"),
            );
            ui.add_enabled(
                !calibration_locked,
                egui::Slider::new(
                    &mut self.sim.control.ignition_timing_deg,
                    self.ui_config.ignition_slider_min_deg..=self.ui_config.ignition_slider_max_deg,
                )
                .text("Ignition Timing [deg BTDC]"),
            );

            ui.separator();
            ui.label(format!("Engine layout: fixed {}-cylinder", FIXED_CYLINDER_COUNT));
            ui.label(format!("Throttle eff: {:.3}", self.sim.state.throttle_eff));
            ui.label(format!("Load cmd: {:.3}", self.sim.control.load_cmd));
            ui.label(if self.latest.stable_idle {
                "State: IDLE STABLE"
            } else {
                "State: TRANSIENT"
            });
            ui.label(if self.sim.auto.enabled {
                "Auto mode: IDLE"
            } else if self.sim.auto.wot_efficiency_enabled {
                "Auto mode: WOT efficiency"
            } else {
                "Auto mode: OFF"
            });
            let bench_status = self.bench.status();
            ui.label(match bench_status {
                Some(status) if self.bench.is_active() => {
                    format!(
                        "Bench: RUNNING [{}] ({}/{})",
                        status.mode.label(),
                        status.completed_points,
                        status.total_points
                    )
                }
                Some(status) if self.bench.is_complete() => {
                    format!("Bench: COMPLETE [{}] ({})", status.mode.label(), status.total_points)
                }
                _ => "Bench: OFF".to_owned(),
            });
            if let Some(status) = bench_status {
                if self.bench.is_active() || self.bench.is_complete() {
                    ui.label(format!(
                        "Bench phase: {} / {} @ {:.0} rpm",
                        status.mode.label(),
                        status.phase.label(),
                        status.target_rpm
                    ));
                }
                if status.total_points > 0 {
                    let overall_progress =
                        status.completed_points as f32 / status.total_points as f32;
                    ui.add(
                        egui::ProgressBar::new(overall_progress.clamp(0.0, 1.0))
                            .text(format!("Bench overall {:.0}%", overall_progress * 100.0)),
                    );
                }
                if self.bench.is_active() && status.phase_total_s > 0.0 {
                    let phase_progress =
                        (status.phase_elapsed_s / status.phase_total_s).clamp(0.0, 1.0) as f32;
                    ui.add(
                        egui::ProgressBar::new(phase_progress)
                            .text(format!("Phase {:.0}%", phase_progress * 100.0)),
                    );
                    ui.label(format!(
                        "Bench phase progress: {:.2}/{:.2} s",
                        status.phase_elapsed_s, status.phase_total_s
                    ));
                    ui.label(format!(
                        "Bench live: {:.1} Nm / MAP {:.1} kPa / eta {:.1} % / lambda {:.2} / Tcharge {:.1} K",
                        status.live_torque_nm,
                        status.live_map_kpa,
                        status.live_eta_thermal_indicated_pv * 100.0,
                        status.live_lambda_target,
                        status.live_intake_charge_temp_k
                    ));
                }
            }
            if self.sim.auto.wot_efficiency_enabled {
                ui.label(format!(
                    "WOT search: {} / {}",
                    self.sim.auto.wot_phase.label(),
                    self.sim.auto.wot_axis.label()
                ));
                ui.label(format!(
                    "WOT best eta: {:.1} % @ ign {:.1} / VVTi {:.1} / VVTe {:.1}",
                    self.sim.auto.wot_best_eta * 100.0,
                    self.sim.auto.wot_best_point.ignition_deg,
                    self.sim.auto.wot_best_point.vvt_intake_deg,
                    self.sim.auto.wot_best_point.vvt_exhaust_deg
                ));
            }
            if let Some(last) = self.bench.results().last() {
                let last_power_kw = shaft_power_kw(last.rpm, last.torque_net_nm);
                let last_power_hp = shaft_power_hp(last.rpm, last.torque_net_nm);
                ui.label(format!(
                    "Bench last: {} / {:.0} rpm / {:.1} Nm / {:.1} kW / {:.1} HP / MAP {:.1} kPa / eta {:.1} % / lambda {:.2} / Tcharge {:.1} K",
                    last.mixture_mode.label(),
                    last.rpm,
                    last.torque_net_nm,
                    last_power_kw,
                    last_power_hp,
                    last.map_kpa,
                    last.eta_thermal_indicated_pv * 100.0,
                    last.lambda_target,
                    last.intake_charge_temp_k
                ));
            }
            ui.label(if self.audio.is_some() {
                "Audio: ON (runner/collector pulse synth)"
            } else {
                "Audio: OFF (device unavailable)"
            });
        });

        egui::CentralPanel::default().show(ctx, |ui| {
            let graph_heights = graph_layout_heights(&self.ui_config, self.bench_curve_visible());

            self.render_pressure_plots(ui, graph_heights);
            self.render_bench_curve(ui, graph_heights);

            ui.columns(2, |columns| {
                columns[0].group(|ui| {
                    let combustion_power_kw =
                        shaft_power_kw(self.latest.rpm, self.latest.torque_combustion_cycle_nm);
                    let combustion_power_hp =
                        shaft_power_hp(self.latest.rpm, self.latest.torque_combustion_cycle_nm);
                    let brake_power_kw = shaft_power_kw(self.latest.rpm, self.latest.torque_net_nm);
                    let brake_power_hp = shaft_power_hp(self.latest.rpm, self.latest.torque_net_nm);
                    let load_power_kw = shaft_power_kw(self.latest.rpm, self.latest.torque_load_nm);
                    let load_power_hp = shaft_power_hp(self.latest.rpm, self.latest.torque_load_nm);
                    ui.label(format!("RPM: {:.1}", self.latest.rpm));
                    ui.label(format!("MAP: {:.1} kPa", self.latest.map_kpa));
                    ui.label(format!(
                        "Intake runner pressure: {:.1} kPa",
                        self.latest.intake_runner_kpa
                    ));
                    ui.label(format!(
                        "Exhaust pressure: {:.1} kPa",
                        self.latest.exhaust_kpa
                    ));
                    ui.label(format!(
                        "Exhaust runner pressure: {:.1} kPa",
                        self.latest.exhaust_runner_kpa
                    ));
                    ui.label(format!(
                        "Wave pressure (intake/exhaust): {:.2} / {:.2} kPa",
                        self.latest.intake_wave_kpa, self.latest.exhaust_wave_kpa
                    ));
                    ui.label(format!("Exhaust temp: {:.1} K", self.latest.exhaust_temp_k));
                    ui.label(format!("Air flow: {:.2} g/s", self.latest.air_flow_gps));
                    ui.label(format!(
                        "Trapped air: {:.2} mg/cyl",
                        self.latest.trapped_air_mg
                    ));
                    ui.label(format!(
                        "Lambda target / charge temp: {:.2} / {:.1} K",
                        self.latest.lambda_target, self.latest.intake_charge_temp_k
                    ));
                    ui.label(format!(
                        "Volumetric efficiency: {:.1} %",
                        self.latest.volumetric_efficiency * 100.0
                    ));
                    ui.label(format!(
                        "Pulse VE / ram / scav: {:.3} / {:.3} / {:.3}",
                        self.latest.ve_pulse_multiplier,
                        self.latest.intake_ram_multiplier,
                        self.latest.exhaust_scavenge_multiplier
                    ));
                    ui.label(format!(
                        "Cylinder heat loss: {:.3} kJ/cyl/cycle",
                        self.latest.heat_loss_cycle_j * 1.0e-3
                    ));
                    ui.label(format!(
                        "Ignition timing: {:.1} deg BTDC",
                        self.latest.ignition_timing_deg
                    ));
                    ui.label(format!(
                        "Torque (comb inst/comb mean/starter/friction/pump/load): {:.2} / {:.2} / {:.2} / {:.2} / {:.2} / {:.2} Nm",
                        self.latest.torque_combustion_nm,
                        self.latest.torque_combustion_cycle_nm,
                        self.latest.torque_starter_nm,
                        self.latest.torque_friction_nm,
                        self.latest.torque_pumping_nm,
                        self.latest.torque_load_nm
                    ));
                    ui.label(format!(
                        "Net torque (inst/smooth): {:.2} / {:.2} Nm",
                        self.latest.torque_net_inst_nm, self.latest.torque_net_nm
                    ));
                    ui.label(format!(
                        "External load power: {:.2} kW / {:.2} HP",
                        load_power_kw, load_power_hp
                    ));
                    ui.label(format!(
                        "Combustion gross power: {:.2} kW / {:.2} HP",
                        combustion_power_kw, combustion_power_hp
                    ));
                    ui.label(format!(
                        "Brake power (smooth): {:.2} kW / {:.2} HP",
                        brake_power_kw, brake_power_hp
                    ));
                    ui.label(format!("Brake BMEP (smooth): {:.2} bar", self.latest.brake_bmep_bar));
                    ui.label(format!("IMEP: {:.2} bar", self.latest.imep_bar));
                    ui.label(format!(
                        "Thermal eff. (Otto): {:.1} %",
                        self.latest.eta_thermal_theoretical * 100.0
                    ));
                    ui.label(format!(
                        "Indicated eff. from p-V: {:.1} % (Wi: {:.3} kJ/cyl/cycle)",
                        self.latest.eta_thermal_indicated_pv * 100.0,
                        self.latest.indicated_work_cycle_j * 1.0e-3
                    ));
                    ui.label(format!(
                        "Cycle angle / burn-rate: {:.1} deg / {:.2}",
                        self.latest.cycle_deg, self.latest.combustion_rate_norm
                    ));
                });

                columns[1].group(|ui| {
                    let recent_cycles = self.sim.plot.history_recent_cycles.max(1);
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
                        .y_axis_label("RPM [rpm]")
                        .show(ui, |plot_ui| {
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
                        .y_axis_label("Torque [Nm]")
                        .show(ui, |plot_ui| {
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
                                    line = line.name("Net torque [Nm]");
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
                        });

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
                        .y_axis_label("Trapped air [mg/cyl]")
                        .show(ui, |plot_ui| {
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
                                    line = line.name("Trapped air [mg/cyl]");
                                }
                                plot_ui.line(line);
                            }
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
                        .show(ui, |plot_ui| {
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
            });
        });

        let frame_ms = 1000u64 / self.ui_config.repaint_hz.max(1) as u64;
        ctx.request_repaint_after(Duration::from_millis(frame_ms.max(1)));
    }
}

pub(crate) fn run_app(config: AppConfig) -> eframe::Result {
    // Window size comes from YAML so different machine setups can keep their preferred layout.
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([config.ui.window_width_px, config.ui.window_height_px]),
        ..Default::default()
    };

    eframe::run_native(
        "ES Simulator Dashboard",
        options,
        Box::new(move |_cc| Ok(Box::new(DashboardApp::new(config.clone())))),
    )
}
