mod theme;
mod widgets;

use std::time::{Duration, Instant};
use std::{
    fs::{self, File},
    io::{BufWriter, Write},
    path::PathBuf,
};

use eframe::egui;
use egui_plot::{Legend, Line, MarkerShape, Plot, PlotBounds, PlotPoints, Points, VLine};

use self::theme::DashboardTheme;
use self::widgets::{
    GaugeSpec, LinearMeterSpec, annunciator, digital_readout, gauge, linear_meter, metric_row,
    monitor_heading, section_label,
};
use crate::config::{AppConfig, BenchMixtureMode, ExternalLoadMode, UiConfig};
use crate::constants::FIXED_CYLINDER_COUNT;
use crate::simulator::{
    BenchSample, BenchSession, BenchStatus, ControlInput, CycleHistorySample, Simulator,
    accuracy_priority_dt, cam_profile_points, estimate_realtime_performance,
    external_load_reflected_inertia_kgm2, external_load_vehicle_speed_kph,
    quick_wot_bench_preview_curve, rpm_linked_dt, shaft_power_hp, shaft_power_kw,
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

fn bench_mode_slug(mode: BenchMixtureMode) -> &'static str {
    match mode {
        BenchMixtureMode::RichChargeCooling => "rich_charge_cooling",
        BenchMixtureMode::LambdaOne => "lambda_one",
    }
}

fn export_bench_results_csv(
    results: &[BenchSample],
    mode: BenchMixtureMode,
) -> Result<PathBuf, String> {
    let output_dir = PathBuf::from("dist").join("bench");
    fs::create_dir_all(&output_dir)
        .map_err(|err| format!("failed to create '{}': {err}", output_dir.display()))?;
    let path = output_dir.join(format!("bench-{}-latest.csv", bench_mode_slug(mode)));
    let file = File::create(&path)
        .map_err(|err| format!("failed to create '{}': {err}", path.display()))?;
    let mut writer = BufWriter::new(file);
    writeln!(
        writer,
        "target_rpm,measured_rpm,brake_torque_nm,brake_power_kw,dyno_load_cmd,dyno_load_torque_nm,map_kpa,eta_thermal_indicated_pv,lambda_target,intake_charge_temp_k,mixture_mode"
    )
    .map_err(|err| format!("failed to write '{}': {err}", path.display()))?;
    for sample in results {
        writeln!(
            writer,
            "{:.1},{:.1},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{}",
            sample.target_rpm,
            sample.rpm,
            sample.torque_brake_nm,
            shaft_power_kw(sample.rpm, sample.torque_brake_nm),
            sample.load_cmd,
            sample.load_torque_nm,
            sample.map_kpa,
            sample.eta_thermal_indicated_pv,
            sample.lambda_target,
            sample.intake_charge_temp_k,
            bench_mode_slug(sample.mixture_mode),
        )
        .map_err(|err| format!("failed to write '{}': {err}", path.display()))?;
    }
    Ok(path)
}

#[derive(Debug, Clone)]
struct BenchCurvePlotData {
    summary: String,
    label: String,
    curve_line_points: Vec<[f64; 2]>,
    preview_curve_points: Vec<[f64; 2]>,
    completed_points: Vec<[f64; 2]>,
    origin_anchor: Option<[f64; 2]>,
    peak_torque_point: Option<[f64; 2]>,
    live_point: Option<[f64; 2]>,
    power_curve_line_points: Vec<[f64; 2]>,
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
    let scale = if bench_visible { 0.72 } else { 0.92 };
    let standard_plot_px = (ui_config.plot_height_px * scale).max(72.0);
    let pv_plot_px = (ui_config.pv_plot_height_px * scale).max(standard_plot_px * 1.55);
    let bench_torque_multiplier = if bench_visible { 1.2 } else { 1.55 };
    let bench_power_multiplier = if bench_visible { 1.2 } else { 1.25 };

    GraphLayoutHeights {
        standard_plot_px,
        pv_plot_px,
        bench_torque_plot_px: (ui_config.plot_height_px * bench_torque_multiplier * scale)
            .max(standard_plot_px * 1.15),
        bench_power_plot_px: (ui_config.plot_height_px * bench_power_multiplier * scale)
            .max(standard_plot_px),
        section_spacing_px: if bench_visible { 4.0 } else { 6.0 },
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
            .map(|sample| [sample.rpm, sample.torque_brake_nm])
            .collect(),
    );
    let preview_curve_points: Vec<[f64; 2]> = sort_points(
        preview_results
            .iter()
            .map(|sample| [sample.rpm, sample.torque_brake_nm])
            .collect(),
    );
    let completed_power_points: Vec<[f64; 2]> = sort_points(
        bench_results
            .iter()
            .map(|sample| {
                [
                    sample.rpm,
                    shaft_power_kw(sample.rpm, sample.torque_brake_nm),
                ]
            })
            .collect(),
    );
    let preview_power_curve_points: Vec<[f64; 2]> = sort_points(
        preview_results
            .iter()
            .map(|sample| {
                [
                    sample.rpm,
                    shaft_power_kw(sample.rpm, sample.torque_brake_nm),
                ]
            })
            .collect(),
    );

    let live_point = live_bench.map(|sample| [sample.rpm, sample.torque_brake_nm]);
    let live_power_point = live_bench.map(|sample| {
        [
            sample.rpm,
            shaft_power_kw(sample.rpm, sample.torque_brake_nm),
        ]
    });
    let has_curve_data =
        !completed_points.is_empty() || !preview_curve_points.is_empty() || live_point.is_some();
    let origin_anchor =
        (bench_config.include_zero_rpm_anchor && axis_x_min <= f64::EPSILON && has_curve_data)
            .then_some([0.0, 0.0]);
    // Keep anchors and live points as markers only so the chart never implies a measured slope
    // that the bench solver has not actually sampled yet.
    let mut curve_line_points = Vec::with_capacity(
        completed_points.len()
            + usize::from(origin_anchor.is_some() && !completed_points.is_empty()),
    );
    let mut power_curve_line_points = Vec::with_capacity(
        completed_power_points.len()
            + usize::from(origin_anchor.is_some() && !completed_power_points.is_empty()),
    );
    if let Some(anchor) = origin_anchor.filter(|_| !completed_points.is_empty()) {
        curve_line_points.push(anchor);
        power_curve_line_points.push(anchor);
    }
    curve_line_points.extend(completed_points.iter().copied());
    power_curve_line_points.extend(completed_power_points.iter().copied());

    let peak_torque_sample = bench_results
        .iter()
        .max_by(|a, b| a.torque_brake_nm.total_cmp(&b.torque_brake_nm))
        .copied();
    let peak_power_sample = bench_results
        .iter()
        .max_by(|a, b| {
            shaft_power_kw(a.rpm, a.torque_brake_nm)
                .total_cmp(&shaft_power_kw(b.rpm, b.torque_brake_nm))
        })
        .copied();
    let preview_peak_torque_sample = preview_results
        .iter()
        .max_by(|a, b| a.torque_brake_nm.total_cmp(&b.torque_brake_nm))
        .copied();
    let preview_peak_power_sample = preview_results
        .iter()
        .max_by(|a, b| {
            shaft_power_kw(a.rpm, a.torque_brake_nm)
                .total_cmp(&shaft_power_kw(b.rpm, b.torque_brake_nm))
        })
        .copied();
    let peak_torque_point = peak_torque_sample
        .or(preview_peak_torque_sample)
        .map(|sample| [sample.rpm, sample.torque_brake_nm]);
    let peak_power_point = peak_power_sample
        .or(preview_peak_power_sample)
        .map(|sample| {
            [
                sample.rpm,
                shaft_power_kw(sample.rpm, sample.torque_brake_nm),
            ]
        });

    let summary = if let (Some(peak_torque), Some(peak_power)) =
        (peak_torque_sample, peak_power_sample)
    {
        format!(
            "Dyno curves: peak torque {:.1} Nm @ {:.0} rpm, peak power {:.1} kW ({:.1} hp) @ {:.0} rpm ({})",
            peak_torque.torque_brake_nm,
            peak_torque.rpm,
            shaft_power_kw(peak_power.rpm, peak_power.torque_brake_nm),
            shaft_power_hp(peak_power.rpm, peak_power.torque_brake_nm),
            peak_power.rpm,
            peak_torque.mixture_mode.label()
        )
    } else if let (Some(peak_torque), Some(peak_power)) =
        (preview_peak_torque_sample, preview_peak_power_sample)
    {
        format!(
            "Dyno preview: peak torque {:.1} Nm @ {:.0} rpm, peak power {:.1} kW ({:.1} hp) @ {:.0} rpm ({})",
            peak_torque.torque_brake_nm,
            peak_torque.rpm,
            shaft_power_kw(peak_power.rpm, peak_power.torque_brake_nm),
            shaft_power_hp(peak_power.rpm, peak_power.torque_brake_nm),
            peak_power.rpm,
            peak_torque.mixture_mode.label()
        )
    } else if let Some(sample) = live_bench {
        format!(
            "Dyno curves: measuring {:.0} rpm (target {:.0}) / {:.1} Nm / {:.1} kW ({})",
            sample.rpm,
            sample.target_rpm,
            sample.torque_brake_nm,
            shaft_power_kw(sample.rpm, sample.torque_brake_nm),
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
        curve_line_points,
        preview_curve_points,
        completed_points,
        origin_anchor,
        peak_torque_point,
        live_point,
        power_curve_line_points,
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
    config: AppConfig,
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
    bench: BenchRunner,
    bench_mode_selected: BenchMixtureMode,
    bench_export_note: Option<String>,
    bench_exported_complete: bool,
}

impl DashboardApp {
    fn new(config: AppConfig) -> Self {
        let theme = DashboardTheme::default();
        let ui_config = config.ui.clone();
        let bench_mode_selected = config.bench.default_mode;
        let dt = config.environment.dt.max(ui_config.min_base_dt_s);
        let (dt_min_bound, dt_max_bound, realtime_fixed_dt_s, dt_next) =
            if ui_config.sync_to_wall_clock {
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
                let dt_next = accuracy_priority_dt(config.engine.idle_target_rpm, &config.numerics)
                    .clamp(dt_min_bound, dt_max_bound);
                (dt_min_bound, dt_max_bound, None, dt_next)
            };
        let mut sim = Simulator::new(&config);
        let latest = sim.step(dt_next);
        Self {
            config,
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
            bench: BenchRunner::default(),
            bench_mode_selected,
            bench_export_note: None,
            bench_exported_complete: false,
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
        cfg.model.external_load = self.sim.model.external_load.clone();
        self.bench.start(cfg, self.bench_control_seed(), mode);
        self.bench_export_note = None;
        self.bench_exported_complete = false;
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
        let target_time = if self.ui_config.sync_to_wall_clock {
            (now - self.last_tick).as_secs_f64().max(self.dt_base)
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
                        self.sim.params.idle_target_rpm,
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
            self.latest = self.sim.step(dt_tail);
            self.dt_next = self
                .realtime_fixed_dt_s
                .unwrap_or_else(|| dt_tail.clamp(self.dt_min_bound, self.dt_max_bound));
        }
        self.last_tick = now;
    }

    fn advance_bench(&mut self) {
        let was_complete = self.bench.is_complete();
        if self.bench.is_active() {
            self.bench.advance_budgeted(
                self.config.bench.steps_per_frame.max(1),
                self.config.bench.frame_time_budget_ms,
            );
        }
        if self.bench.is_complete() && !was_complete && !self.bench_exported_complete {
            self.bench_export_note = match self.bench.status() {
                Some(status) => export_bench_results_csv(self.bench.results(), status.mode)
                    .map(|path| format!("{}", path.display()))
                    .map_err(|err| err)
                    .ok(),
                None => None,
            };
            self.bench_exported_complete = true;
        }
    }

    fn bench_curve_visible(&self) -> bool {
        !self.bench.results().is_empty()
            || !self.bench.preview_results().is_empty()
            || self.bench.live_sample().is_some()
    }

    fn bench_status_summary(&self) -> String {
        match self.bench.status() {
            Some(status) if self.bench.is_active() => format!(
                "{} / {} / {:.0}->{:.0} rpm / {}/{}",
                status.mode.label(),
                status.phase.label(),
                status.live_rpm,
                status.target_rpm,
                status.completed_points,
                status.total_points
            ),
            Some(status) if self.bench.is_complete() => {
                format!(
                    "{} / complete / {} pts",
                    status.mode.label(),
                    status.total_points
                )
            }
            _ => "bench idle".to_owned(),
        }
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
                            egui::RichText::new(
                                if self.ui_config.sync_to_wall_clock {
                                    "Wall-clock synchronized console with dyno sweep and p-V / p-theta monitoring"
                                } else {
                                    "Accuracy-first transient console with dyno sweep and p-V / p-theta monitoring"
                                },
                            )
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
                            if self.latest.stable_idle { "STABLE" } else { "LIVE" },
                            "",
                            &self.bench_status_summary(),
                            160.0,
                        );
                    });
                });
                ui.add_space(8.0);
                ui.horizontal_wrapped(|ui| {
                    annunciator(
                        ui,
                        self.theme,
                        "RUN",
                        self.sim.state.running,
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
                        "STARTER",
                        self.sim.control.starter_cmd,
                        self.theme.red,
                    );
                    annunciator(
                        ui,
                        self.theme,
                        "AUTO IDLE",
                        self.sim.auto.enabled,
                        self.theme.green,
                    );
                    annunciator(
                        ui,
                        self.theme,
                        "WOT SEARCH",
                        self.sim.auto.wot_efficiency_enabled,
                        self.theme.amber,
                    );
                    annunciator(
                        ui,
                        self.theme,
                        "BENCH",
                        self.bench.is_active() || self.bench.is_complete(),
                        self.theme.cyan,
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
            .default_width(300.0)
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
                    "rack_automation",
                    "Automation",
                    "IDLE / WOT supervisor",
                    self.theme.green,
                    true,
                    |ui| {
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
                    if self.sim.auto.wot_efficiency_enabled {
                        ui.label(format!(
                            "WOT search: {} / {}",
                            self.sim.auto.wot_phase.label(),
                            self.sim.auto.wot_axis.label()
                        ));
                        ui.label(format!(
                            "Best eta: {:.1}% @ ign {:.1} / VVTi {:.1} / VVTe {:.1}",
                            self.sim.auto.wot_best_eta * 100.0,
                            self.sim.auto.wot_best_point.ignition_deg,
                            self.sim.auto.wot_best_point.vvt_intake_deg,
                            self.sim.auto.wot_best_point.vvt_exhaust_deg
                        ));
                    }
                    },
                );

                ui.add_space(8.0);
                show_collapsible_module(
                    ui,
                    self.theme,
                    "rack_bench",
                    "Bench Sequencer",
                    "preview + continuous sweep",
                    self.theme.cyan,
                    true,
                    |ui| {
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
                        if ui
                            .add_sized([126.0, 34.0], egui::Button::new("Run Bench"))
                            .clicked()
                        {
                            self.start_bench(self.bench_mode_selected);
                        }
                        if ui
                            .add_enabled(
                                self.bench.is_active(),
                                egui::Button::new("Stop Bench").min_size(egui::vec2(126.0, 34.0)),
                            )
                            .clicked()
                        {
                            self.bench.stop();
                        }
                    });

                    if let Some(status) = self.bench.status() {
                        if status.total_points > 0 {
                            let overall_progress =
                                status.completed_points as f32 / status.total_points as f32;
                            ui.add(
                                egui::ProgressBar::new(overall_progress.clamp(0.0, 1.0))
                                    .text(format!("overall {:.0}%", overall_progress * 100.0)),
                            );
                        }
                        if self.bench.is_active() && status.phase_total_s > 0.0 {
                            let phase_progress = (status.phase_elapsed_s / status.phase_total_s)
                                .clamp(0.0, 1.0)
                                as f32;
                            ui.add(
                                egui::ProgressBar::new(phase_progress)
                                    .text(format!("phase {:.0}%", phase_progress * 100.0)),
                            );
                            ui.label(format!(
                                "Live: {:.0}->{:.0} rpm / {:.1} Nm / load {:.3} ({:.1} Nm) / MAP {:.1} kPa / eta {:.1}% / lambda {:.2} / Tcharge {:.1} K",
                                status.live_rpm,
                                status.target_rpm,
                                status.live_torque_brake_nm,
                                status.live_load_cmd,
                                status.live_load_torque_nm,
                                status.live_map_kpa,
                                status.live_eta_thermal_indicated_pv * 100.0,
                                status.live_lambda_target,
                                status.live_intake_charge_temp_k
                            ));
                        }
                        ui.label(format!("Bench status: {}", self.bench_status_summary()));
                        if let Some(path) = &self.bench_export_note {
                            ui.label(format!("Bench CSV: {path}"));
                        }
                    }
                    },
                );

                ui.add_space(8.0);
                show_collapsible_module(
                    ui,
                    self.theme,
                    "rack_actuator",
                    "Actuator Deck",
                    "manual command surface",
                    self.theme.amber,
                    true,
                    |ui| {
                    let auto_actuator_locked =
                        self.sim.auto.enabled || self.sim.auto.wot_efficiency_enabled;
                    let calibration_locked = self.sim.auto.wot_efficiency_enabled;
                    let mut load_mode = self.sim.model.external_load.mode;

                    ui.add_enabled(
                        !auto_actuator_locked,
                        egui::Slider::new(&mut self.sim.control.throttle_cmd, 0.0..=1.0)
                            .text("Throttle cmd"),
                    );
                    ui.add(
                        egui::Slider::new(&mut self.sim.control.load_cmd, 0.0..=1.0)
                            .text("Load cmd"),
                    );
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
                    ui.separator();
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
                    false,
                    |ui| {
                    let load_model = &self.sim.model.external_load;
                    ui.label(format!(
                        "Engine layout: fixed {}-cylinder",
                        FIXED_CYLINDER_COUNT
                    ));
                    ui.label(format!("Throttle eff: {:.3}", self.sim.state.throttle_eff));
                    ui.label(format!("Load cmd: {:.3}", self.sim.control.load_cmd));
                    ui.label(format!("Load model: {}", load_model.mode.label()));
                    if load_model.mode == ExternalLoadMode::VehicleEquivalent {
                        ui.label(format!(
                            "Vehicle eq: {:.1} km/h / Jref {:.3} kg m^2",
                            external_load_vehicle_speed_kph(
                                self.sim.state.omega_rad_s,
                                load_model,
                            ),
                            external_load_reflected_inertia_kgm2(self.sim.control.load_cmd, load_model)
                        ));
                    }
                    ui.label(if self.ui_config.sync_to_wall_clock {
                        "Solver mode: wall-clock synchronized"
                    } else {
                        "Solver mode: accuracy first"
                    });
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
                    if let Some(last) = self.bench.results().last() {
                        ui.label(format!(
                            "Last bench: {} / {:.0} rpm (target {:.0}) / {:.1} Nm / load {:.3} / lambda {:.2}",
                            last.mixture_mode.label(),
                            last.rpm,
                            last.target_rpm,
                            last.torque_brake_nm,
                            last.load_cmd,
                            last.lambda_target
                        ));
                    }
                    },
                );
                ui.add_space(8.0);
                self.render_state_bus(ui);
                    });
            });
    }

    fn render_console_overview(&self, ui: &mut egui::Ui) {
        let combustion_power_kw =
            shaft_power_kw(self.latest.rpm, self.latest.torque_combustion_cycle_nm);
        let brake_power_kw = shaft_power_kw(self.latest.rpm, self.latest.torque_net_nm);
        let panel_width = ui.available_width().max(320.0);
        let readout_columns = responsive_card_columns(panel_width, 190.0, 6);
        let readout_width = responsive_card_width(panel_width, readout_columns).min(248.0);
        let gauge_columns = responsive_card_columns(panel_width, 148.0, 6);
        let gauge_width = responsive_card_width(panel_width, gauge_columns).min(188.0);
        let meter_columns = responsive_card_columns(panel_width, 210.0, 3);
        let meter_width = responsive_card_width(panel_width, meter_columns).min(282.0);
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
            while readout_index < 6 {
                ui.horizontal(|ui| {
                    ui.spacing_mut().item_spacing.x = 8.0;
                    for slot in 0..readout_columns {
                        let idx = readout_index + slot;
                        if idx >= 6 {
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
                                if self.latest.stable_idle {
                                    "idle stable"
                                } else {
                                    "dynamic state"
                                },
                                readout_width,
                            ),
                            1 => digital_readout(
                                ui,
                                self.theme,
                                self.theme.red,
                                "NET TORQUE FILT",
                                &format!("{:.1}", self.latest.torque_net_nm),
                                "Nm",
                                &format!("inst {:.1} Nm", self.latest.torque_net_inst_nm),
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
                                    shaft_power_hp(self.latest.rpm, self.latest.torque_net_nm)
                                ),
                                readout_width,
                            ),
                            3 => digital_readout(
                                ui,
                                self.theme,
                                self.theme.green,
                                "AIR CHARGE",
                                &format!("{:.2}", self.latest.trapped_air_mg),
                                "mg/cyl",
                                &format!("VE {:.1}%", self.latest.volumetric_efficiency * 100.0),
                                readout_width,
                            ),
                            4 => digital_readout(
                                ui,
                                self.theme,
                                self.theme.amber,
                                "INTAKE BUS",
                                &format!("{:.1}", self.latest.map_kpa),
                                "kPa",
                                &intake_state,
                                readout_width,
                            ),
                            5 => digital_readout(
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
            while gauge_index < 6 {
                ui.horizontal(|ui| {
                    ui.spacing_mut().item_spacing.x = 8.0;
                    for slot in 0..gauge_columns {
                        let idx = gauge_index + slot;
                        if idx >= 6 {
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
                                label: "COMB PWR",
                                value: combustion_power_kw,
                                min: 0.0,
                                max: 160.0,
                                unit: "kW",
                                accent: self.theme.cyan,
                                footer: "gross combustion",
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
            while meter_index < 6 {
                ui.horizontal(|ui| {
                    ui.spacing_mut().item_spacing.x = 8.0;
                    for slot in 0..meter_columns {
                        let idx = meter_index + slot;
                        if idx >= 6 {
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
                                label: "LOAD CMD",
                                value: self.sim.control.load_cmd,
                                min: 0.0,
                                max: 1.0,
                                accent: self.theme.red,
                                value_text: format!("{:.3}", self.sim.control.load_cmd),
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
            false,
            |ui| {
                egui::Grid::new("state_bus_grid")
                    .num_columns(2)
                    .spacing(egui::vec2(16.0, 6.0))
                    .show(ui, |ui| {
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
                            "Starter / load torque",
                            format!(
                                "{:.2} / {:.2} Nm",
                                self.latest.torque_starter_nm, self.latest.torque_load_nm
                            ),
                        );
                        metric_row(
                            ui,
                            self.theme,
                            "Brake power filt / load power",
                            format!(
                                "{:.2} / {:.2} kW",
                                shaft_power_kw(self.latest.rpm, self.latest.torque_net_nm),
                                shaft_power_kw(self.latest.rpm, self.latest.torque_load_nm)
                            ),
                        );
                    });
            },
        );
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

        self.theme.monitor_frame().show(ui, |ui| {
            monitor_heading(
                ui,
                self.theme,
                "Dyno Monitor",
                &plot.summary,
                self.theme.cyan,
            );
            ui.columns(2, |columns| {
                Plot::new("bench_torque_curve_plot")
                    .height(graph_heights.bench_torque_plot_px)
                    .allow_scroll(false)
                    .allow_drag(false)
                    .allow_zoom(false)
                    .include_x(plot.x_min)
                    .include_y(0.0)
                    .legend(Legend::default())
                    .x_axis_label("Engine speed [rpm]")
                    .y_axis_label("Brake torque [Nm]")
                    .show(&mut columns[0], |plot_ui| {
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

                        if plot.curve_line_points.len() >= 2 {
                            plot_ui.line(
                                Line::new(plot.curve_line_points.clone())
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
                    .show(&mut columns[1], |plot_ui| {
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

                        if plot.power_curve_line_points.len() >= 2 {
                            plot_ui.line(
                                Line::new(plot.power_curve_line_points.clone())
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
            false,
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
            },
        );
    }
}

#[cfg(test)]
mod tests {
    use std::collections::VecDeque;

    use super::{
        adaptive_plot_y_range, bench_curve_plot_data, export_bench_results_csv,
        graph_layout_heights, recent_cycle_plot_lines,
    };
    use crate::config::{AppConfig, BenchMixtureMode};
    use crate::simulator::{BenchSample, BenchSession, ControlInput, CycleHistorySample};

    fn sample(rpm: f64, torque_brake_nm: f64, mode: BenchMixtureMode) -> BenchSample {
        BenchSample {
            target_rpm: rpm,
            rpm,
            torque_brake_nm,
            load_cmd: 0.55,
            load_torque_nm: torque_brake_nm,
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
        assert!(plot.curve_line_points.is_empty());
        assert!(plot.power_curve_line_points.is_empty());
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
        let rpms: Vec<f64> = plot
            .curve_line_points
            .iter()
            .map(|point| point[0])
            .collect();

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
    fn bench_export_writes_csv_with_brake_torque_columns() {
        let results = vec![
            sample(2_000.0, 150.0, BenchMixtureMode::RichChargeCooling),
            sample(3_000.0, 180.0, BenchMixtureMode::RichChargeCooling),
        ];

        let path = export_bench_results_csv(&results, BenchMixtureMode::RichChargeCooling).unwrap();
        let text = std::fs::read_to_string(&path).unwrap();

        assert!(text.contains("brake_torque_nm"));
        assert!(text.contains("dyno_load_torque_nm"));
        assert!(text.contains("rich_charge_cooling"));
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

        self.render_header_panel(ctx);
        self.render_control_rack(ctx);

        egui::CentralPanel::default().show(ctx, |ui| {
            egui::ScrollArea::both()
                .id_salt("dashboard_main_scroll")
                .auto_shrink([false, false])
                .show(ui, |ui| {
                    let graph_heights =
                        graph_layout_heights(&self.ui_config, self.bench_curve_visible());

                    self.render_console_overview(ui);
                    self.render_pressure_plots(ui, graph_heights);
                    self.render_bench_curve(ui, graph_heights);
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

#[cfg(target_os = "android")]
pub(crate) fn run_android_app(
    config: AppConfig,
    android_app: winit::platform::android::activity::AndroidApp,
) -> eframe::Result {
    let mut options = eframe::NativeOptions::default();
    options.android_app = Some(android_app);
    run_app_with_options(config, options)
}
