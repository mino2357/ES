use std::f64::consts::PI;

use eframe::egui;
use egui_plot::{Legend, Line, MarkerShape, Plot, PlotBounds, PlotPoints, Points, VLine};

use super::super::app::{CycleSubview, OperatorTab, PressureSubview, VisualizationTab};
use super::super::build_info;
use super::super::simple_view_model::{
    OPERATING_POINT_BRAKE_LABELS, OPERATING_POINT_SPEED_LABELS, OperatingPointCellHeatViewModel,
    OperatingPointTableViewModel, SimpleDashboardViewModel, StatusRow,
};
use super::super::startup_fit::{StartupFitPhase, StartupFitStatus, StartupFitTorqueCurvePoint};
use super::super::state::{DashboardState, PostFitRuntimeMode};
use super::super::theme::DashboardTheme;
use super::super::widgets::{GaugeSpec, LinearMeterSpec, gauge, linear_meter, monitor_heading};
use crate::config::UiConfig;
use crate::constants::FIXED_CYLINDER_COUNT;
use crate::simulator::{cam_profile_points, shaft_power_kw};

#[derive(Debug, Clone)]
struct PlotSeries {
    name: Option<String>,
    color: egui::Color32,
    points: Vec<[f64; 2]>,
}

#[derive(Debug, Clone)]
struct PlotMarker {
    name: Option<String>,
    color: egui::Color32,
    point: [f64; 2],
    shape: MarkerShape,
    radius: f32,
}

pub(crate) fn render_simple_dashboard(
    ctx: &egui::Context,
    theme: DashboardTheme,
    ui_config: &UiConfig,
    state: &mut DashboardState,
    active_tab: &mut VisualizationTab,
    operator_tab: &mut OperatorTab,
    pressure_subview: &mut PressureSubview,
    cycle_subview: &mut CycleSubview,
    header_badge: &str,
) {
    let view = SimpleDashboardViewModel::from_state(state, ui_config);
    render_header(ctx, theme, &view, header_badge);
    render_controls_panel(
        ctx,
        theme,
        ui_config,
        state,
        &view,
        operator_tab,
        header_badge,
    );
    render_visualization_panel(
        ctx,
        theme,
        ui_config,
        state,
        &view,
        active_tab,
        pressure_subview,
        cycle_subview,
    );
}

fn render_header(
    ctx: &egui::Context,
    theme: DashboardTheme,
    view: &SimpleDashboardViewModel,
    header_badge: &str,
) {
    egui::TopBottomPanel::top("simple_header").show(ctx, |ui| {
        let panel_width = ui.available_width();
        theme.header_frame().show(ui, |ui| {
            ui.set_min_width(panel_width.max(320.0));
            ui.horizontal(|ui| {
                ui.vertical(|ui| {
                    ui.horizontal_wrapped(|ui| {
                        ui.heading(
                            egui::RichText::new("ES Simulator")
                                .color(theme.text_main)
                                .strong(),
                        );
                        ui.separator();
                        ui.label(
                            egui::RichText::new(
                                "Numerical startup fit from first fire, with live p-V / p-theta monitoring",
                            )
                            .color(theme.text_soft),
                        );
                    });

                    ui.horizontal_wrapped(|ui| {
                        ui.colored_label(
                            fit_status_color(theme, view.fit_phase),
                            egui::RichText::new(view.fit_status_value).strong(),
                        );
                        ui.separator();
                        ui.label(&view.header_fit_text);
                        ui.separator();
                        ui.label(&view.header_rpm_text);
                        ui.separator();
                        ui.label(&view.header_target_text);
                        ui.separator();
                        ui.label(&view.header_error_text);
                    });
                });

                ui.with_layout(egui::Layout::right_to_left(egui::Align::TOP), |ui| {
                    ui.label(
                        egui::RichText::new(header_badge)
                            .color(theme.amber)
                            .monospace()
                            .size(10.5),
                    );
                });
            });
        });
    });
}

fn render_controls_panel(
    ctx: &egui::Context,
    theme: DashboardTheme,
    ui_config: &UiConfig,
    state: &mut DashboardState,
    view: &SimpleDashboardViewModel,
    operator_tab: &mut OperatorTab,
    header_badge: &str,
) {
    egui::SidePanel::left("simple_controls_v2")
        .default_width(336.0)
        .min_width(320.0)
        .max_width(380.0)
        .resizable(true)
        .show(ctx, |ui| {
            theme.rack_frame().show(ui, |ui| {
                monitor_heading(
                    ui,
                    theme,
                    "Operator Deck",
                    "Keep fit and runtime controls inside the default window footprint",
                    theme.cyan,
                );
                render_operator_tabs(ui, theme, operator_tab);
                ui.add_space(8.0);

                match *operator_tab {
                    OperatorTab::Fit => render_fit_panel(ui, theme, view),
                    OperatorTab::Status => render_status_tab(ui, theme, view, header_badge),
                    OperatorTab::Runtime => render_manual_panel(ui, theme, ui_config, state, view),
                }
            });
        });
}

fn render_visualization_panel(
    ctx: &egui::Context,
    theme: DashboardTheme,
    ui_config: &UiConfig,
    state: &DashboardState,
    view: &SimpleDashboardViewModel,
    active_tab: &mut VisualizationTab,
    pressure_subview: &mut PressureSubview,
    cycle_subview: &mut CycleSubview,
) {
    let fit_status = state.startup_fit_status();
    egui::CentralPanel::default().show(ctx, |ui| {
        theme.monitor_frame().show(ui, |ui| {
            let fit_phase = fit_status.phase;
            monitor_heading(
                ui,
                theme,
                "Visualization Deck",
                "One page per view so the default window stays readable",
                tab_accent(*active_tab, theme),
            );
            if fit_phase == StartupFitPhase::Priming {
                render_visualization_arming_panel(ui, theme, state);
            } else {
                render_output_cards(ui, theme, state);
                ui.add_space(8.0);
                render_visualization_tabs(ui, theme, active_tab);
                ui.add_space(8.0);

                match *active_tab {
                    VisualizationTab::Overview => {
                        render_overview_tab(ui, theme, ui_config, state, view)
                    }
                    VisualizationTab::Map => render_map_tab(ui, theme, view),
                    VisualizationTab::Fit => {
                        render_fit_diagnostics_tab(ui, theme, ui_config, state, &fit_status)
                    }
                    VisualizationTab::Pressure => {
                        render_pressure_tab(ui, theme, ui_config, state, pressure_subview)
                    }
                    VisualizationTab::Cycle => {
                        render_cycle_tab(ui, theme, ui_config, state, cycle_subview)
                    }
                }
            }
        });
    });
}

fn render_status_tab(
    ui: &mut egui::Ui,
    theme: DashboardTheme,
    view: &SimpleDashboardViewModel,
    header_badge: &str,
) {
    monitor_heading(
        ui,
        theme,
        "Run Status",
        "Binary identification and reduced-order engine readback",
        theme.green,
    );
    ui.label(
        egui::RichText::new(header_badge)
            .color(theme.amber)
            .monospace()
            .size(10.5),
    );
    ui.label(
        egui::RichText::new(format!("UI layout {}", build_info::layout_revision()))
            .color(theme.text_soft)
            .monospace()
            .size(10.0),
    );
    ui.separator();
    ui.label(&view.live_status_message);
    ui.separator();
    egui::Grid::new("live_status_grid")
        .num_columns(2)
        .spacing(egui::vec2(12.0, 8.0))
        .show(ui, |ui| {
            for row in &view.live_rows {
                render_status_row(ui, row);
            }
        });
}

fn render_status_row(ui: &mut egui::Ui, row: &StatusRow) {
    ui.label(row.label);
    ui.vertical(|ui| {
        ui.label(&row.value);
    });
    ui.end_row();
}

fn render_operator_tabs(ui: &mut egui::Ui, theme: DashboardTheme, operator_tab: &mut OperatorTab) {
    ui.horizontal_wrapped(|ui| {
        for tab in OperatorTab::ALL {
            if deck_tab_button(ui, theme, tab.label(), *operator_tab == tab, theme.cyan).clicked() {
                *operator_tab = tab;
            }
        }
    });
}

fn render_fit_panel(ui: &mut egui::Ui, theme: DashboardTheme, view: &SimpleDashboardViewModel) {
    ui.label(&view.fit_detail);
    theme.instrument_frame(theme.cyan).show(ui, |ui| {
        ui.horizontal(|ui| {
            ui.colored_label(
                fit_status_color(theme, view.fit_phase),
                egui::RichText::new(view.fit_status_value).strong(),
            );
            ui.label(egui::RichText::new(&view.fit_progress_label).color(theme.text_soft));
        });
        ui.add(
            egui::ProgressBar::new(view.fit_progress_fraction)
                .desired_width(safe_available_width(ui, 220.0))
                .text(view.fit_progress_label.clone()),
        );
    });
    ui.add_space(6.0);
    egui::Grid::new("fit_grid")
        .num_columns(2)
        .spacing(egui::vec2(12.0, 6.0))
        .show(ui, |ui| {
            for row in &view.fit_rows {
                render_status_row(ui, row);
            }
        });
}

fn render_manual_panel(
    ui: &mut egui::Ui,
    theme: DashboardTheme,
    ui_config: &UiConfig,
    state: &mut DashboardState,
    view: &SimpleDashboardViewModel,
) {
    ui.heading("Runtime Control");
    if let Some(message) = &view.manual_control_hint {
        ui.label(egui::RichText::new(message).color(theme.red));
    } else {
        ui.label(
            egui::RichText::new("Standard runtime keeps realtime response, and Actuator lab exposes all actuator overrides in the same window.")
                .color(theme.text_soft),
        );
    }

    theme.instrument_frame(theme.amber).show(ui, |ui| {
        ui.add_enabled_ui(!view.fit_active, |ui| {
            let mut selected_mode = state.post_fit_mode;
            ui.horizontal_wrapped(|ui| {
                ui.selectable_value(
                    &mut selected_mode,
                    PostFitRuntimeMode::StandardRuntime,
                    "Standard runtime",
                );
                ui.selectable_value(
                    &mut selected_mode,
                    PostFitRuntimeMode::ActuatorLab,
                    "Actuator lab",
                );
            });
            state.set_post_fit_runtime_mode(selected_mode);
            ui.label(
                egui::RichText::new(state.post_fit_mode.detail())
                    .color(theme.text_soft)
                    .size(10.5),
            );
            ui.separator();
            ui.add(
                egui::Slider::new(&mut state.driver_demand, 0.0..=1.0).text("Driver demand"),
            );
            ui.label(format!(
                "Torque request: {:+.1} Nm",
                state.post_fit_baseline.requested_brake_torque_nm
            ));
            ui.label(format!(
                "Auto baseline: throttle {:.3} / ignition {:.1} deg / VVT {:+.1} / {:+.1} deg",
                state.post_fit_baseline.throttle_cmd,
                state.post_fit_baseline.ignition_timing_deg,
                state.post_fit_baseline.vvt_intake_deg,
                state.post_fit_baseline.vvt_exhaust_deg
            ));
            ui.add(
                egui::Slider::new(
                    &mut state.load_target_rpm,
                    0.0..=state.sim.params.max_rpm,
                )
                .text("Target RPM"),
            );
            state.refresh_post_fit_preview();
            ui.label(format!(
                "Required brake torque: {:.1} Nm / RPM err {:+.0}",
                state.latest.torque_load_nm,
                state.rpm_error()
            ));
            ui.separator();
            if state.post_fit_mode == PostFitRuntimeMode::ActuatorLab {
                ui.label(
                    egui::RichText::new(
                        "Actuator lab is live. Manual values are applied directly; the auto baseline stays visible for comparison.",
                    )
                    .color(theme.text_soft)
                    .size(10.5),
                );
                ui.add(
                    egui::Slider::new(&mut state.sim.control.throttle_cmd, 0.0..=1.0)
                        .text("Throttle override"),
                );
                ui.label(format!(
                    "Throttle delta vs auto: {:+.3}",
                    state.sim.control.throttle_cmd - state.post_fit_baseline.throttle_cmd
                ));
                ui.add(
                    egui::Slider::new(
                        &mut state.sim.control.ignition_timing_deg,
                        ui_config.ignition_slider_min_deg..=ui_config.ignition_slider_max_deg,
                    )
                    .text("Ignition override [deg BTDC]"),
                );
                ui.label(format!(
                    "Ignition delta vs auto: {:+.1} deg",
                    state.sim.control.ignition_timing_deg
                        - state.post_fit_baseline.ignition_timing_deg
                ));
                ui.add(
                    egui::Slider::new(
                        &mut state.sim.control.vvt_intake_deg,
                        ui_config.vvt_slider_min_deg..=ui_config.vvt_slider_max_deg,
                    )
                    .text("VVT intake override [deg]"),
                );
                ui.add(
                    egui::Slider::new(
                        &mut state.sim.control.vvt_exhaust_deg,
                        ui_config.vvt_slider_min_deg..=ui_config.vvt_slider_max_deg,
                    )
                    .text("VVT exhaust override [deg]"),
                );
                ui.label(format!(
                    "VVT delta vs auto: IN {:+.1} / EX {:+.1} deg",
                    state.sim.control.vvt_intake_deg - state.post_fit_baseline.vvt_intake_deg,
                    state.sim.control.vvt_exhaust_deg - state.post_fit_baseline.vvt_exhaust_deg
                ));
            } else {
                ui.label(
                    egui::RichText::new(
                        "Standard runtime is live. Driver demand is the primary input; throttle, ignition, and VVT follow the auto baseline.",
                    )
                    .color(theme.text_soft)
                    .size(10.5),
                );
            }
            ui.separator();
            ui.add(
                egui::Checkbox::new(&mut state.sim.control.spark_cmd, "Spark enable"),
            );
            ui.add(egui::Checkbox::new(&mut state.sim.control.fuel_cmd, "Fuel enable"));
        });
    });

    ui.add_space(6.0);
    ui.label(egui::RichText::new(&view.external_load_note).color(theme.text_soft));
}

fn render_visualization_tabs(
    ui: &mut egui::Ui,
    theme: DashboardTheme,
    active_tab: &mut VisualizationTab,
) {
    ui.horizontal_wrapped(|ui| {
        for tab in VisualizationTab::ALL {
            if deck_tab_button(
                ui,
                theme,
                tab.label(),
                *active_tab == tab,
                tab_accent(tab, theme),
            )
            .clicked()
            {
                *active_tab = tab;
            }
        }
    });
    ui.add_space(2.0);
    ui.label(
        egui::RichText::new((*active_tab).description())
            .color(theme.text_soft)
            .monospace()
            .size(10.0),
    );
}

fn render_pressure_subtabs(
    ui: &mut egui::Ui,
    theme: DashboardTheme,
    pressure_subview: &mut PressureSubview,
) {
    ui.horizontal_wrapped(|ui| {
        for tab in PressureSubview::ALL {
            if deck_tab_button(
                ui,
                theme,
                tab.label(),
                *pressure_subview == tab,
                theme.amber,
            )
            .clicked()
            {
                *pressure_subview = tab;
            }
        }
    });
}

fn render_cycle_subtabs(
    ui: &mut egui::Ui,
    theme: DashboardTheme,
    cycle_subview: &mut CycleSubview,
) {
    ui.horizontal_wrapped(|ui| {
        for tab in CycleSubview::ALL {
            if deck_tab_button(ui, theme, tab.label(), *cycle_subview == tab, theme.cyan).clicked()
            {
                *cycle_subview = tab;
            }
        }
    });
}

fn deck_tab_button(
    ui: &mut egui::Ui,
    theme: DashboardTheme,
    label: &str,
    selected: bool,
    accent: egui::Color32,
) -> egui::Response {
    let button = egui::Button::new(
        egui::RichText::new(label)
            .color(if selected { accent } else { theme.text_soft })
            .strong(),
    )
    .fill(if selected {
        theme.panel_alt_bg
    } else {
        theme.panel_bg
    })
    .stroke(egui::Stroke::new(
        if selected { 1.4 } else { 1.0 },
        if selected { accent } else { theme.bezel },
    ));
    ui.add(button)
}

fn render_output_cards(ui: &mut egui::Ui, theme: DashboardTheme, state: &DashboardState) {
    let indicated_torque_nm =
        state.latest.indicated_work_cycle_j * FIXED_CYLINDER_COUNT as f64 / (4.0 * PI);
    let net_power_kw = shaft_power_kw(state.latest.rpm, state.latest.torque_net_nm);
    let required_brake_torque_nm = state.latest.torque_load_nm;
    let cards = [
        (
            theme.cyan,
            "Indicated torque",
            format!("{:+.1}", indicated_torque_nm),
            "Nm",
            format!(
                "p-V Wi {:.3} kJ/cyl",
                state.latest.indicated_work_cycle_j * 1.0e-3
            ),
        ),
        (
            theme.red,
            "Required brake torque",
            format!("{:+.1}", required_brake_torque_nm),
            "Nm",
            format!("bench load cmd {:+.3}", state.sim.control.load_cmd),
        ),
        (
            theme.amber,
            "Net torque / power",
            format!("{:+.1}", state.latest.torque_net_nm),
            "Nm",
            format!("{:+.1} kW at {:.0} rpm", net_power_kw, state.latest.rpm),
        ),
        (
            theme.green,
            "IMEP / eta_i",
            format!("{:.2}", state.latest.imep_bar),
            "bar",
            format!(
                "{:.1}% from p-V loop",
                state.latest.eta_thermal_indicated_pv * 100.0
            ),
        ),
    ];

    if safe_available_width(ui, 720.0) < 760.0 {
        egui::Grid::new("simple_metric_cards_grid")
            .num_columns(2)
            .spacing(egui::vec2(8.0, 8.0))
            .show(ui, |ui| {
                for (index, (accent, label, value, unit, footer)) in cards.iter().enumerate() {
                    render_metric_card(
                        ui,
                        theme,
                        *accent,
                        label,
                        value.clone(),
                        unit,
                        footer.clone(),
                    );
                    if index % 2 == 1 {
                        ui.end_row();
                    }
                }
            });
    } else {
        ui.horizontal_wrapped(|ui| {
            for (accent, label, value, unit, footer) in cards.iter() {
                ui.allocate_ui_with_layout(
                    egui::vec2(166.0, 82.0),
                    egui::Layout::top_down(egui::Align::Min),
                    |ui| {
                        render_metric_card(
                            ui,
                            theme,
                            *accent,
                            label,
                            value.clone(),
                            unit,
                            footer.clone(),
                        );
                    },
                );
            }
        });
    }
}

fn render_overview_tab(
    ui: &mut egui::Ui,
    theme: DashboardTheme,
    ui_config: &UiConfig,
    state: &DashboardState,
    _view: &SimpleDashboardViewModel,
) {
    let left = |ui: &mut egui::Ui| {
        gauge(
            ui,
            theme,
            GaugeSpec {
                label: "ENGINE SPEED",
                value: state.latest.rpm,
                min: 0.0,
                max: state.sim.params.max_rpm,
                unit: "rpm",
                accent: theme.cyan,
                footer: &format!("target {:.0} rpm", state.load_target_rpm),
                width: 148.0,
                height: 104.0,
            },
        );
        gauge(
            ui,
            theme,
            GaugeSpec {
                label: "MANIFOLD PRESSURE",
                value: state.latest.map_kpa,
                min: 20.0,
                max: (state.sim.env.ambient_pressure_pa * 1.0e-3 + 40.0).max(90.0),
                unit: "kPa",
                accent: theme.green,
                footer: &format!("runner {:.1} kPa", state.latest.intake_runner_kpa),
                width: 148.0,
                height: 104.0,
            },
        );
        gauge(
            ui,
            theme,
            GaugeSpec {
                label: "INDICATED EFF",
                value: state.latest.eta_thermal_indicated_pv * 100.0,
                min: 0.0,
                max: 50.0,
                unit: "%",
                accent: theme.amber,
                footer: &format!("IMEP {:.2} bar", state.latest.imep_bar),
                width: 148.0,
                height: 104.0,
            },
        );
    };

    let right = |ui: &mut egui::Ui| {
        linear_meter(
            ui,
            theme,
            LinearMeterSpec {
                label: "NET TORQUE",
                value: state.latest.torque_net_nm,
                min: -80.0,
                max: 160.0,
                accent: theme.red,
                value_text: format!("{:+.1} Nm", state.latest.torque_net_nm),
                width: 228.0,
            },
        );
        linear_meter(
            ui,
            theme,
            LinearMeterSpec {
                label: "TORQUE REQUEST",
                value: state.post_fit_baseline.requested_brake_torque_nm,
                min: 0.0,
                max: 160.0,
                accent: theme.amber,
                value_text: format!(
                    "{:+.1} Nm / load {:+.1}",
                    state.post_fit_baseline.requested_brake_torque_nm, state.latest.torque_load_nm
                ),
                width: 228.0,
            },
        );
        linear_meter(
            ui,
            theme,
            LinearMeterSpec {
                label: "DRIVER DEMAND",
                value: state.driver_demand * 100.0,
                min: 0.0,
                max: 100.0,
                accent: theme.cyan,
                value_text: format!(
                    "{:.0}% / auto thr {:.1}%",
                    state.driver_demand * 100.0,
                    state.post_fit_baseline.throttle_cmd * 100.0
                ),
                width: 228.0,
            },
        );
        linear_meter(
            ui,
            theme,
            LinearMeterSpec {
                label: "THROTTLE ACT",
                value: state.sim.control.throttle_cmd * 100.0,
                min: 0.0,
                max: 100.0,
                accent: theme.amber,
                value_text: format!(
                    "{:.1}% / auto {:.1}%",
                    state.sim.control.throttle_cmd * 100.0,
                    state.post_fit_baseline.throttle_cmd * 100.0
                ),
                width: 228.0,
            },
        );
        linear_meter(
            ui,
            theme,
            LinearMeterSpec {
                label: "IGNITION",
                value: state.sim.control.ignition_timing_deg,
                min: ui_config.ignition_slider_min_deg,
                max: ui_config.ignition_slider_max_deg,
                accent: theme.green,
                value_text: format!(
                    "{:.1} deg / auto {:.1}",
                    state.sim.control.ignition_timing_deg,
                    state.post_fit_baseline.ignition_timing_deg
                ),
                width: 228.0,
            },
        );
    };

    ui.horizontal_wrapped(|ui| {
        ui.vertical(|ui| left(ui));
        ui.add_space(8.0);
        ui.vertical(|ui| right(ui));
    });
}

fn render_map_tab(ui: &mut egui::Ui, theme: DashboardTheme, view: &SimpleDashboardViewModel) {
    render_operating_point_table(ui, theme, &view.operating_point_table);
}

fn render_fit_diagnostics_tab(
    ui: &mut egui::Ui,
    theme: DashboardTheme,
    ui_config: &UiConfig,
    state: &DashboardState,
    fit: &StartupFitStatus,
) {
    let mut series = Vec::new();
    let coarse_series = build_fit_torque_curve_series(
        "Coarse MBT best",
        egui::Color32::from_rgb(120, 140, 160),
        &fit.coarse_torque_curve,
    );
    if !coarse_series.points.is_empty() {
        series.push(coarse_series);
    }
    let refine_series =
        build_fit_torque_curve_series("Refined MBT best", theme.green, &fit.refine_torque_curve);
    if !refine_series.points.is_empty() {
        series.push(refine_series);
    }

    let mut markers = Vec::new();
    if fit.release_required_brake_torque_nm.is_finite() && fit.release_throttle_cmd.is_finite() {
        markers.push(PlotMarker {
            name: Some(if fit.active {
                "Best so far".to_owned()
            } else {
                "Selected release".to_owned()
            }),
            color: theme.green,
            point: [
                fit.release_throttle_cmd * 100.0,
                fit.release_required_brake_torque_nm,
            ],
            shape: MarkerShape::Circle,
            radius: 6.0,
        });
    }
    if !fit.active && fit.required_brake_torque_nm.is_finite() && fit.throttle_cmd.is_finite() {
        markers.push(PlotMarker {
            name: Some("Verified result".to_owned()),
            color: theme.amber,
            point: [fit.throttle_cmd * 100.0, fit.required_brake_torque_nm],
            shape: MarkerShape::Diamond,
            radius: 6.0,
        });
    }
    if state.latest.torque_load_nm.is_finite() && state.sim.control.throttle_cmd.is_finite() {
        markers.push(PlotMarker {
            name: Some("Live point".to_owned()),
            color: theme.cyan,
            point: [
                state.sim.control.throttle_cmd * 100.0,
                state.latest.torque_load_nm,
            ],
            shape: MarkerShape::Square,
            radius: 5.5,
        });
    }

    ui.horizontal_wrapped(|ui| {
        ui.label(
            egui::RichText::new(
                "Each point is the per-throttle MBT result; y is the brake torque required to hold the target RPM.",
            )
            .color(theme.text_soft)
            .monospace()
            .size(10.0),
        );
    });
    ui.add_space(6.0);
    ui.horizontal_wrapped(|ui| {
        ui.colored_label(
            theme.green,
            format!(
                "SEL {:.1}% / spark {:.1} / {:+.1} Nm / {:.0} rpm",
                fit.release_throttle_cmd * 100.0,
                fit.release_ignition_timing_deg,
                fit.release_required_brake_torque_nm,
                fit.release_avg_rpm
            ),
        );
        if !fit.active {
            ui.separator();
            ui.colored_label(
                theme.amber,
                format!(
                    "RES {:.1}% / spark {:.1} / {:+.1} Nm / {:.0} rpm",
                    fit.throttle_cmd * 100.0,
                    fit.ignition_timing_deg,
                    fit.required_brake_torque_nm,
                    fit.avg_rpm
                ),
            );
        }
        ui.separator();
        ui.colored_label(
            theme.cyan,
            format!(
                "LIVE {:.1}% / spark {:.1} / {:+.1} Nm / {:.0} rpm",
                state.sim.control.throttle_cmd * 100.0,
                state.sim.control.ignition_timing_deg,
                state.latest.torque_load_nm,
                state.latest.rpm
            ),
        );
    });
    ui.add_space(6.0);

    let plot_height = bounded_plot_height(ui, ui_config.plot_height_px * 1.9, 260.0, 460.0, 170.0);
    render_line_plot(
        ui,
        theme,
        ui_config,
        "startup_fit_torque_curve_plot",
        "Startup-fit torque curve",
        "Per-throttle MBT brake-torque requirement through coarse and refined search",
        theme.green,
        "Throttle command [%]",
        "Required brake torque [Nm]",
        (0.0, 100.0),
        bounds_from_series(&series, 1, (0.0, 40.0), 10.0, 0.14, Some(0.0)),
        &series,
        &markers,
        None,
        plot_height,
    );
}

fn build_fit_torque_curve_series(
    name: &str,
    color: egui::Color32,
    curve: &[StartupFitTorqueCurvePoint],
) -> PlotSeries {
    PlotSeries {
        name: Some(name.to_owned()),
        color,
        points: curve
            .iter()
            .filter_map(|point| {
                let plot_point = [point.throttle_cmd * 100.0, point.required_brake_torque_nm];
                point_is_finite(plot_point).then_some(plot_point)
            })
            .collect(),
    }
}

fn render_operating_point_table(
    ui: &mut egui::Ui,
    theme: DashboardTheme,
    table: &OperatingPointTableViewModel,
) {
    monitor_heading(
        ui,
        theme,
        "Operating Point Map",
        "Speed x required-brake-torque table for fit release, post-fit result, and live point",
        theme.green,
    );
    theme.instrument_frame(theme.green).show(ui, |ui| {
        ui.horizontal_wrapped(|ui| {
            ui.label(
                egui::RichText::new(table.note)
                    .color(theme.text_soft)
                    .monospace()
                    .size(10.0),
            );
        });
        ui.add_space(6.0);
        ui.horizontal_wrapped(|ui| {
            ui.colored_label(theme.green, table.fit_summary.as_str());
            if let Some(result_summary) = &table.result_summary {
                ui.separator();
                ui.colored_label(theme.amber, result_summary.as_str());
            }
            ui.separator();
            ui.colored_label(theme.cyan, table.live_summary.as_str());
        });
        ui.add_space(8.0);

        let desired_height = safe_available_height(ui, 340.0);
        let (rect, _) = ui.allocate_exact_size(
            egui::vec2(safe_available_width(ui, 680.0), desired_height),
            egui::Sense::hover(),
        );
        paint_operating_point_table(ui, theme, rect, table);
    });
}

fn paint_operating_point_table(
    ui: &egui::Ui,
    theme: DashboardTheme,
    rect: egui::Rect,
    table: &OperatingPointTableViewModel,
) {
    let painter = ui.painter_at(rect);
    let outer = rect.shrink2(egui::vec2(4.0, 4.0));
    painter.rect_filled(outer, 10.0, theme.panel_bg);
    painter.rect_stroke(
        outer,
        10.0,
        egui::Stroke::new(1.0, theme.bezel),
        egui::StrokeKind::Inside,
    );

    let cols = OPERATING_POINT_SPEED_LABELS.len();
    let rows = OPERATING_POINT_BRAKE_LABELS.len();
    let inner = outer.shrink2(egui::vec2(10.0, 10.0));
    let top_header_h = 22.0;
    let left_header_w = 54.0;
    let bottom_label_h = 16.0;
    let axis_font = egui::FontId::monospace(9.0);
    let band_font = egui::FontId::monospace(8.0);
    let cell_font = egui::FontId::monospace(9.0);
    let cell_w = ((inner.width() - left_header_w) / cols as f32).max(1.0);
    let cell_h = ((inner.height() - top_header_h - bottom_label_h) / rows as f32).max(1.0);
    let grid_rect = egui::Rect::from_min_size(
        inner.min,
        egui::vec2(
            left_header_w + cell_w * cols as f32,
            top_header_h + cell_h * rows as f32,
        ),
    );

    painter.text(
        egui::pos2(grid_rect.left() + left_header_w, grid_rect.top()),
        egui::Align2::LEFT_TOP,
        "Engine speed [rpm]",
        axis_font.clone(),
        theme.text_soft,
    );

    for (col, label) in OPERATING_POINT_SPEED_LABELS.iter().enumerate() {
        let x = grid_rect.left() + left_header_w + col as f32 * cell_w + cell_w * 0.5;
        painter.text(
            egui::pos2(x, grid_rect.top() + top_header_h - 8.0),
            egui::Align2::CENTER_CENTER,
            *label,
            band_font.clone(),
            theme.text_soft,
        );
    }

    painter.text(
        egui::pos2(
            grid_rect.left(),
            grid_rect.top() + top_header_h + cell_h * rows as f32 + 6.0,
        ),
        egui::Align2::LEFT_TOP,
        "Required brake torque [Nm]",
        axis_font,
        theme.text_soft,
    );

    for draw_row in 0..rows {
        let row = rows - 1 - draw_row;
        let top = grid_rect.top() + top_header_h + draw_row as f32 * cell_h;
        painter.text(
            egui::pos2(grid_rect.left() + left_header_w - 8.0, top + cell_h * 0.5),
            egui::Align2::RIGHT_CENTER,
            OPERATING_POINT_BRAKE_LABELS[row],
            band_font.clone(),
            theme.text_soft,
        );

        for col in 0..cols {
            let heat_index = row * cols + col;
            let cell_heat = table.cell_heat.get(heat_index).copied().unwrap_or_default();
            let cell_rect = egui::Rect::from_min_size(
                egui::pos2(grid_rect.left() + left_header_w + col as f32 * cell_w, top),
                egui::vec2(cell_w - 4.0, cell_h - 4.0),
            );
            let fit_here = table.fit_marker.cell.row == row && table.fit_marker.cell.col == col;
            let result_here = table
                .result_marker
                .as_ref()
                .map(|marker| marker.cell.row == row && marker.cell.col == col)
                .unwrap_or(false);
            let live_here = table.live_marker.cell.row == row && table.live_marker.cell.col == col;
            let stroke = if fit_here || result_here || live_here {
                egui::Stroke::new(1.2, theme.chrome)
            } else {
                egui::Stroke::new(1.0, theme.bezel)
            };
            painter.rect_filled(
                cell_rect,
                6.0,
                operating_point_cell_fill(theme, cell_heat, table.max_heat_hits),
            );
            painter.rect_stroke(cell_rect, 6.0, stroke, egui::StrokeKind::Inside);
            paint_operating_point_count(&painter, cell_rect, band_font.clone(), theme, cell_heat);
            paint_operating_point_badges(
                &painter,
                cell_rect,
                cell_font.clone(),
                theme,
                fit_here,
                result_here,
                live_here,
            );
        }
    }
}

fn operating_point_cell_fill(
    theme: DashboardTheme,
    heat: OperatingPointCellHeatViewModel,
    max_hits: u16,
) -> egui::Color32 {
    let total_hits = heat.coarse_hits.saturating_add(heat.refine_hits);
    if total_hits == 0 || max_hits == 0 {
        return theme.panel_alt_bg;
    }

    let mut fill = theme.panel_alt_bg;
    if heat.coarse_hits > 0 {
        let coarse_ratio = (heat.coarse_hits as f32 / max_hits as f32).clamp(0.0, 1.0);
        fill = mix_color(
            fill,
            egui::Color32::from_rgb(70, 105, 138),
            0.18 + 0.32 * coarse_ratio,
        );
    }
    if heat.refine_hits > 0 {
        let refine_ratio = (heat.refine_hits as f32 / max_hits as f32).clamp(0.0, 1.0);
        fill = mix_color(fill, theme.green, 0.18 + 0.44 * refine_ratio);
    }
    fill
}

fn paint_operating_point_count(
    painter: &egui::Painter,
    cell_rect: egui::Rect,
    font: egui::FontId,
    theme: DashboardTheme,
    heat: OperatingPointCellHeatViewModel,
) {
    let total_hits = heat.coarse_hits.saturating_add(heat.refine_hits);
    if total_hits == 0 {
        return;
    }
    painter.text(
        cell_rect.center(),
        egui::Align2::CENTER_CENTER,
        total_hits.to_string(),
        font,
        egui::Color32::from_rgba_unmultiplied(
            theme.text_main.r(),
            theme.text_main.g(),
            theme.text_main.b(),
            150,
        ),
    );
}

fn paint_operating_point_badges(
    painter: &egui::Painter,
    cell_rect: egui::Rect,
    font: egui::FontId,
    theme: DashboardTheme,
    fit_here: bool,
    result_here: bool,
    live_here: bool,
) {
    let badge_w = (cell_rect.width() * 0.42).clamp(16.0, 28.0);
    let badge_h = (cell_rect.height() * 0.34).clamp(10.0, 18.0);
    let positions = [
        egui::Rect::from_min_size(
            cell_rect.min + egui::vec2(3.0, 3.0),
            egui::vec2(badge_w, badge_h),
        ),
        egui::Rect::from_min_size(
            egui::pos2(cell_rect.right() - badge_w - 3.0, cell_rect.top() + 3.0),
            egui::vec2(badge_w, badge_h),
        ),
        egui::Rect::from_min_size(
            egui::pos2(
                cell_rect.center().x - badge_w * 0.5,
                cell_rect.bottom() - badge_h - 3.0,
            ),
            egui::vec2(badge_w, badge_h),
        ),
    ];
    let badges = [
        (fit_here, "F", theme.green),
        (result_here, "R", theme.amber),
        (live_here, "L", theme.cyan),
    ];
    let mut slot = 0usize;
    for (active, label, accent) in badges {
        if !active {
            continue;
        }
        let rect = positions[slot.min(positions.len() - 1)];
        painter.rect_filled(
            rect,
            4.0,
            egui::Color32::from_rgba_unmultiplied(accent.r(), accent.g(), accent.b(), 36),
        );
        painter.rect_stroke(
            rect,
            4.0,
            egui::Stroke::new(1.0, accent),
            egui::StrokeKind::Inside,
        );
        painter.text(
            rect.center(),
            egui::Align2::CENTER_CENTER,
            label,
            font.clone(),
            accent,
        );
        slot += 1;
    }
}

fn mix_color(base: egui::Color32, accent: egui::Color32, amount: f32) -> egui::Color32 {
    let t = amount.clamp(0.0, 1.0);
    let blend = |a: u8, b: u8| -> u8 {
        ((a as f32) + ((b as f32) - (a as f32)) * t)
            .round()
            .clamp(0.0, 255.0) as u8
    };
    egui::Color32::from_rgba_unmultiplied(
        blend(base.r(), accent.r()),
        blend(base.g(), accent.g()),
        blend(base.b(), accent.b()),
        255,
    )
}

fn render_pressure_tab(
    ui: &mut egui::Ui,
    theme: DashboardTheme,
    ui_config: &UiConfig,
    state: &DashboardState,
    pressure_subview: &mut PressureSubview,
) {
    let pv_points: Vec<[f64; 2]> = state
        .latest
        .pv_points
        .iter()
        .filter_map(|(volume, pressure_pa)| {
            let point = [*volume, *pressure_pa * 1.0e-3];
            point_is_finite(point).then_some(point)
        })
        .collect();
    let pv_series = vec![PlotSeries {
        name: Some("p-V".to_owned()),
        color: egui::Color32::from_rgb(255, 170, 40),
        points: pv_points,
    }];
    let log_series = vec![PlotSeries {
        name: Some("log10 p-V".to_owned()),
        color: theme.cyan,
        points: build_log_log_pv_points(&state.latest.pv_points),
    }];
    let ptheta_curves = state
        .sim
        .build_ptheta_display_curves(ptheta_display_sample_count(
            ui.available_width().max(320.0),
            state.sim.model.pv_display_bins,
        ));
    let ptheta_series: Vec<PlotSeries> = ptheta_curves
        .iter()
        .enumerate()
        .map(|(idx, curve)| PlotSeries {
            name: Some(format!("Cyl {}", idx + 1)),
            color: cylinder_trace_color(idx),
            points: curve
                .iter()
                .filter_map(|(theta, pressure_pa)| {
                    let point = [*theta, *pressure_pa * 1.0e-3];
                    point_is_finite(point).then_some(point)
                })
                .collect(),
        })
        .collect();
    let pv_markers = pv_combustion_markers(state, theme);
    render_pressure_subtabs(ui, theme, pressure_subview);
    ui.add_space(6.0);
    let plot_height = bounded_plot_height(ui, ui_config.pv_plot_height_px, 220.0, 360.0, 210.0);

    match *pressure_subview {
        PressureSubview::Pv => render_line_plot(
            ui,
            theme,
            ui_config,
            "simple_pv_plot",
            "Cylinder p-V",
            "single-zone diagnostic loop",
            theme.amber,
            "Normalized volume [-]",
            "Pressure [kPa]",
            (state.sim.plot.pv_x_min, state.sim.plot.pv_x_max),
            bounds_from_series(
                &pv_series,
                1,
                (state.sim.plot.pv_y_min_kpa, state.sim.plot.pv_y_max_kpa),
                20.0,
                0.08,
                Some(state.sim.plot.pv_y_min_kpa),
            ),
            &pv_series,
            &pv_markers,
            None,
            plot_height,
        ),
        PressureSubview::LogPv => render_line_plot(
            ui,
            theme,
            ui_config,
            "simple_log_pv_plot",
            "Cylinder p-V (log-log)",
            "log10 volume vs log10 pressure",
            theme.cyan,
            "log10 Normalized volume [-]",
            "log10 Pressure [kPa]",
            bounds_from_series(&log_series, 0, (-3.0, 0.5), 0.1, 0.06, None),
            bounds_from_series(&log_series, 1, (0.0, 3.5), 0.1, 0.06, None),
            &log_series,
            &[],
            None,
            plot_height,
        ),
        PressureSubview::Ptheta => render_line_plot(
            ui,
            theme,
            ui_config,
            "simple_ptheta_plot",
            "Cylinder p-theta",
            "4-cylinder overlay / 0..720 degCA",
            theme.green,
            "Crank angle [degCA]",
            "Pressure [kPa]",
            (0.0, 720.0),
            bounds_from_series(
                &ptheta_series,
                1,
                (state.sim.plot.pv_y_min_kpa, state.sim.plot.pv_y_max_kpa),
                20.0,
                0.08,
                Some(state.sim.plot.pv_y_min_kpa),
            ),
            &ptheta_series,
            &[],
            None,
            plot_height,
        ),
    }
}

fn render_cycle_tab(
    ui: &mut egui::Ui,
    theme: DashboardTheme,
    ui_config: &UiConfig,
    state: &DashboardState,
    cycle_subview: &mut CycleSubview,
) {
    let rpm_series = vec![PlotSeries {
        name: Some("RPM".to_owned()),
        color: theme.green,
        points: latest_cycle_points(&state.sim.history_rpm),
    }];
    let torque_series = vec![
        PlotSeries {
            name: Some("Net torque".to_owned()),
            color: theme.red,
            points: latest_cycle_points(&state.sim.history_torque_net_nm),
        },
        PlotSeries {
            name: Some("Load torque".to_owned()),
            color: theme.amber,
            points: latest_cycle_points(&state.sim.history_torque_load_nm),
        },
    ];
    let air_series = vec![PlotSeries {
        name: Some("Trapped air".to_owned()),
        color: theme.cyan,
        points: latest_cycle_points(&state.sim.history_trapped_air_mg),
    }];
    let (intake_cam_points, exhaust_cam_points) =
        cam_profile_points(&state.sim.control, &state.sim.cam, &state.sim.model);
    let cam_series = vec![
        PlotSeries {
            name: Some("Intake lift".to_owned()),
            color: egui::Color32::from_rgb(80, 190, 255),
            points: intake_cam_points
                .into_iter()
                .filter_map(|point| {
                    let point = [point[0], point[1]];
                    point_is_finite(point).then_some(point)
                })
                .collect(),
        },
        PlotSeries {
            name: Some("Exhaust lift".to_owned()),
            color: egui::Color32::from_rgb(255, 110, 110),
            points: exhaust_cam_points
                .into_iter()
                .filter_map(|point| {
                    let point = [point[0], point[1]];
                    point_is_finite(point).then_some(point)
                })
                .collect(),
        },
    ];
    let cursor = Some((state.latest.cycle_deg, "Crank cursor", theme.chrome));
    render_cycle_subtabs(ui, theme, cycle_subview);
    ui.add_space(6.0);
    let plot_height = bounded_plot_height(ui, ui_config.plot_height_px * 1.55, 170.0, 320.0, 210.0);

    match *cycle_subview {
        CycleSubview::Rpm => render_line_plot(
            ui,
            theme,
            ui_config,
            "cycle_rpm_plot",
            "RPM history",
            "latest-cycle instantaneous speed",
            theme.green,
            "Crank angle [degCA]",
            "RPM inst [rpm]",
            (0.0, 720.0),
            bounds_from_series(
                &rpm_series,
                1,
                (0.0, state.sim.params.max_rpm),
                150.0,
                0.15,
                Some(0.0),
            ),
            &rpm_series,
            &[],
            cursor,
            plot_height,
        ),
        CycleSubview::Torque => render_line_plot(
            ui,
            theme,
            ui_config,
            "cycle_torque_plot",
            "Torque and load history",
            "latest-cycle torque overlay",
            theme.red,
            "Crank angle [degCA]",
            "Torque [Nm]",
            (0.0, 720.0),
            bounds_from_series(&torque_series, 1, (-20.0, 120.0), 10.0, 0.15, None),
            &torque_series,
            &[],
            cursor,
            plot_height,
        ),
        CycleSubview::Air => render_line_plot(
            ui,
            theme,
            ui_config,
            "cycle_air_plot",
            "Trapped air history",
            "latest-cycle charge trace",
            theme.cyan,
            "Crank angle [degCA]",
            "Trapped air inst [mg/cyl]",
            (0.0, 720.0),
            bounds_from_series(&air_series, 1, (0.0, 100.0), 20.0, 0.15, Some(0.0)),
            &air_series,
            &[],
            cursor,
            plot_height,
        ),
        CycleSubview::Valve => render_line_plot(
            ui,
            theme,
            ui_config,
            "cycle_cam_plot",
            "Valve lift map",
            "current intake and exhaust cam profile",
            theme.amber,
            "Crank angle [degCA]",
            "Valve lift [mm]",
            (0.0, 720.0),
            (
                0.0,
                state
                    .sim
                    .cam
                    .intake_max_lift_mm
                    .max(state.sim.cam.exhaust_max_lift_mm)
                    * 1.08,
            ),
            &cam_series,
            &[],
            cursor,
            plot_height,
        ),
    }
}

fn render_line_plot(
    ui: &mut egui::Ui,
    theme: DashboardTheme,
    ui_config: &UiConfig,
    plot_id: &str,
    title: &str,
    subtitle: &str,
    accent: egui::Color32,
    x_label: &str,
    y_label: &str,
    x_bounds: (f64, f64),
    y_bounds: (f64, f64),
    series: &[PlotSeries],
    markers: &[PlotMarker],
    cursor: Option<(f64, &str, egui::Color32)>,
    height: f32,
) {
    let sanitized_series: Vec<PlotSeries> = series
        .iter()
        .map(|line| PlotSeries {
            name: line.name.clone(),
            color: line.color,
            points: line
                .points
                .iter()
                .copied()
                .filter(|point| point_is_finite(*point))
                .collect(),
        })
        .collect();
    let sanitized_markers: Vec<PlotMarker> = markers
        .iter()
        .filter(|marker| point_is_finite(marker.point))
        .cloned()
        .collect();
    let has_any_points = sanitized_series.iter().any(|line| !line.points.is_empty());
    let bounds_are_finite = x_bounds.0.is_finite()
        && x_bounds.1.is_finite()
        && y_bounds.0.is_finite()
        && y_bounds.1.is_finite()
        && x_bounds.0 < x_bounds.1
        && y_bounds.0 < y_bounds.1;

    theme.monitor_frame().show(ui, |ui| {
        monitor_heading(ui, theme, title, subtitle, accent);
        if !has_any_points || !bounds_are_finite {
            ui.label(
                egui::RichText::new("Waiting for finite diagnostic data before drawing this plot")
                    .color(theme.text_soft)
                    .monospace()
                    .size(10.5),
            );
            ui.add_space(height.max(120.0) - 22.0);
            return;
        }
        Plot::new(plot_id)
            .height(height)
            .allow_scroll(false)
            .allow_drag(false)
            .allow_zoom(false)
            .legend(Legend::default())
            .x_axis_label(x_label)
            .y_axis_label(y_label)
            .show(ui, |plot_ui| {
                plot_ui.set_auto_bounds(false);
                plot_ui.set_plot_bounds(PlotBounds::from_min_max(
                    [x_bounds.0, y_bounds.0],
                    [x_bounds.1, y_bounds.1],
                ));
                for line_vm in &sanitized_series {
                    let points: PlotPoints<'_> = line_vm.points.iter().copied().collect();
                    let mut line = Line::new(points)
                        .color(line_vm.color)
                        .width(ui_config.line_width_px);
                    if let Some(name) = &line_vm.name {
                        line = line.name(name.clone());
                    }
                    plot_ui.line(line);
                }
                for marker in &sanitized_markers {
                    let mut points = Points::new(vec![marker.point])
                        .shape(marker.shape)
                        .color(marker.color)
                        .filled(false)
                        .radius(marker.radius);
                    if let Some(name) = &marker.name {
                        points = points.name(name.clone());
                    }
                    plot_ui.points(points);
                }
                if let Some((cursor_x, cursor_name, cursor_color)) = cursor {
                    plot_ui.vline(
                        VLine::new(cursor_x)
                            .name(cursor_name)
                            .color(cursor_color)
                            .width(ui_config.crank_line_width_px),
                    );
                }
            });
    });
}

fn render_metric_card(
    ui: &mut egui::Ui,
    theme: DashboardTheme,
    accent: egui::Color32,
    label: &str,
    value: String,
    unit: &str,
    footer: String,
) {
    theme.instrument_frame(accent).show(ui, |ui| {
        ui.set_min_height(78.0);
        ui.label(
            egui::RichText::new(label)
                .color(theme.text_soft)
                .monospace()
                .size(10.0),
        );
        ui.horizontal(|ui| {
            ui.label(
                egui::RichText::new(value)
                    .color(accent)
                    .monospace()
                    .strong()
                    .size(22.0),
            );
            ui.label(
                egui::RichText::new(unit)
                    .color(theme.text_main)
                    .monospace()
                    .size(10.0),
            );
        });
        ui.label(
            egui::RichText::new(footer)
                .color(theme.text_soft)
                .monospace()
                .size(9.0),
        );
    });
}

fn latest_cycle_points(
    history: &std::collections::VecDeque<crate::simulator::CycleHistorySample>,
) -> Vec<[f64; 2]> {
    let Some(latest_cycle) = history.back().map(|sample| sample.cycle) else {
        return Vec::new();
    };
    history
        .iter()
        .filter(|sample| sample.cycle == latest_cycle)
        .filter_map(|sample| {
            let point = [sample.cycle_deg, sample.value];
            point_is_finite(point).then_some(point)
        })
        .collect()
}

fn build_log_log_pv_points(pv_points: &[(f64, f64)]) -> Vec<[f64; 2]> {
    pv_points
        .iter()
        .filter_map(|(volume, pressure_pa)| {
            let pressure_kpa = *pressure_pa * 1.0e-3;
            (*volume > 0.0 && pressure_kpa > 0.0 && volume.is_finite() && pressure_kpa.is_finite())
                .then(|| [volume.log10(), pressure_kpa.log10()])
        })
        .filter(|point| point_is_finite(*point))
        .collect()
}

fn pv_combustion_markers(state: &DashboardState, theme: DashboardTheme) -> Vec<PlotMarker> {
    if !state.sim.control.spark_cmd || !state.sim.control.fuel_cmd {
        return Vec::new();
    }
    let Some(cycle_samples) = latest_complete_pv_cycle_samples(&state.sim.pv_history) else {
        return Vec::new();
    };

    let spark_deg = spark_event_cycle_deg(state.latest.ignition_timing_deg);
    let soc_deg = combustion_fraction_cycle_deg(
        state.latest.burn_start_deg,
        state.latest.burn_duration_deg,
        state.sim.model.wiebe_a,
        state.sim.model.wiebe_m,
        0.0,
    );
    let ca10_deg = combustion_fraction_cycle_deg(
        state.latest.burn_start_deg,
        state.latest.burn_duration_deg,
        state.sim.model.wiebe_a,
        state.sim.model.wiebe_m,
        0.10,
    );
    let ca50_deg = combustion_fraction_cycle_deg(
        state.latest.burn_start_deg,
        state.latest.burn_duration_deg,
        state.sim.model.wiebe_a,
        state.sim.model.wiebe_m,
        0.50,
    );
    let ca90_deg = combustion_fraction_cycle_deg(
        state.latest.burn_start_deg,
        state.latest.burn_duration_deg,
        state.sim.model.wiebe_a,
        state.sim.model.wiebe_m,
        0.90,
    );
    let eoc_deg = combustion_fraction_cycle_deg(
        state.latest.burn_start_deg,
        state.latest.burn_duration_deg,
        state.sim.model.wiebe_a,
        state.sim.model.wiebe_m,
        1.0,
    );

    let mut markers = Vec::new();
    if let Some(point) = interpolate_pv_cycle_point(&cycle_samples, spark_deg) {
        markers.push(PlotMarker {
            name: Some(format!("Ignition {:.1} degCA", spark_deg)),
            color: theme.red,
            point,
            shape: MarkerShape::Cross,
            radius: 8.0,
        });
    }
    if let Some(point) = interpolate_pv_cycle_point(&cycle_samples, soc_deg) {
        markers.push(PlotMarker {
            name: Some(format!("SOC {:.1} degCA", soc_deg)),
            color: egui::Color32::from_rgb(255, 140, 80),
            point,
            shape: MarkerShape::Circle,
            radius: 5.5,
        });
    }
    if let Some(point) = interpolate_pv_cycle_point(&cycle_samples, ca10_deg) {
        markers.push(PlotMarker {
            name: Some(format!("CA10 {:.1} degCA", ca10_deg)),
            color: egui::Color32::from_rgb(255, 210, 90),
            point,
            shape: MarkerShape::Cross,
            radius: 8.0,
        });
    }
    if let Some(point) = interpolate_pv_cycle_point(&cycle_samples, ca50_deg) {
        markers.push(PlotMarker {
            name: Some(format!("CA50 {:.1} degCA", ca50_deg)),
            color: theme.green,
            point,
            shape: MarkerShape::Cross,
            radius: 8.0,
        });
    }
    if let Some(point) = interpolate_pv_cycle_point(&cycle_samples, ca90_deg) {
        markers.push(PlotMarker {
            name: Some(format!("CA90 {:.1} degCA", ca90_deg)),
            color: theme.cyan,
            point,
            shape: MarkerShape::Cross,
            radius: 8.0,
        });
    }
    if let Some(point) = interpolate_pv_cycle_point(&cycle_samples, eoc_deg) {
        markers.push(PlotMarker {
            name: Some(format!("EOC {:.1} degCA", eoc_deg)),
            color: egui::Color32::from_rgb(216, 228, 236),
            point,
            shape: MarkerShape::Diamond,
            radius: 5.5,
        });
    }
    markers
}

fn spark_event_cycle_deg(ignition_timing_deg: f64) -> f64 {
    (360.0 - ignition_timing_deg).rem_euclid(720.0)
}

fn combustion_fraction_cycle_deg(
    burn_start_deg: f64,
    burn_duration_deg: f64,
    wiebe_a: f64,
    wiebe_m: f64,
    burned_fraction: f64,
) -> f64 {
    let xb = wiebe_fraction_location(burned_fraction, wiebe_a, wiebe_m);
    (burn_start_deg + burn_duration_deg.max(1.0) * xb).rem_euclid(720.0)
}

fn wiebe_fraction_location(target_fraction: f64, wiebe_a: f64, wiebe_m: f64) -> f64 {
    let a = wiebe_a.max(f64::EPSILON);
    let m = wiebe_m.max(0.0);
    let target = target_fraction.clamp(0.0, 1.0);
    if target <= 0.0 {
        return 0.0;
    }
    if target >= 1.0 {
        return 1.0;
    }

    let denom = (1.0 - (-a).exp()).max(f64::EPSILON);
    let residual = (1.0 - target * denom).clamp(f64::EPSILON, 1.0);
    (-residual.ln() / a).powf(1.0 / (m + 1.0)).clamp(0.0, 1.0)
}

fn latest_complete_pv_cycle_samples(
    history: &std::collections::VecDeque<crate::simulator::PvSample>,
) -> Option<Vec<crate::simulator::PvSample>> {
    let mut recent_cycles = Vec::new();
    for sample in history.iter().rev() {
        if !recent_cycles.contains(&sample.cycle) {
            recent_cycles.push(sample.cycle);
        }
    }

    for cycle in recent_cycles {
        let samples: Vec<_> = history
            .iter()
            .copied()
            .filter(|sample| sample.cycle == cycle)
            .collect();
        if pv_cycle_is_complete(&samples) {
            return Some(samples);
        }
    }

    None
}

fn pv_cycle_is_complete(samples: &[crate::simulator::PvSample]) -> bool {
    if samples.len() < 32 {
        return false;
    }
    let mut min_deg = f64::INFINITY;
    let mut max_deg = f64::NEG_INFINITY;
    for sample in samples {
        if !sample.cycle_deg.is_finite() {
            continue;
        }
        min_deg = min_deg.min(sample.cycle_deg);
        max_deg = max_deg.max(sample.cycle_deg);
    }
    min_deg <= 20.0 && max_deg >= 700.0
}

fn interpolate_pv_cycle_point(
    samples: &[crate::simulator::PvSample],
    target_deg: f64,
) -> Option<[f64; 2]> {
    if samples.len() < 2 {
        return None;
    }

    let mut ordered = samples.to_vec();
    ordered.sort_by(|a, b| a.cycle_deg.total_cmp(&b.cycle_deg));
    let first_deg = ordered.first()?.cycle_deg;
    let target = target_deg.rem_euclid(720.0);
    let target_unwrapped = if target < first_deg {
        target + 720.0
    } else {
        target
    };

    for idx in 0..ordered.len() {
        let start = ordered[idx];
        let end = if idx + 1 < ordered.len() {
            ordered[idx + 1]
        } else {
            crate::simulator::PvSample {
                cycle: start.cycle,
                cycle_deg: ordered[0].cycle_deg + 720.0,
                volume: ordered[0].volume,
                pressure_pa: ordered[0].pressure_pa,
            }
        };
        let start_deg = start.cycle_deg;
        let end_deg = end.cycle_deg;
        if !(start_deg.is_finite() && end_deg.is_finite()) || end_deg <= start_deg {
            continue;
        }
        if target_unwrapped < start_deg || target_unwrapped > end_deg {
            continue;
        }
        let t = ((target_unwrapped - start_deg) / (end_deg - start_deg)).clamp(0.0, 1.0);
        let point = [
            interpolate_scalar(start.volume, end.volume, t),
            interpolate_scalar(start.pressure_pa, end.pressure_pa, t) * 1.0e-3,
        ];
        if point_is_finite(point) {
            return Some(point);
        }
    }

    None
}

fn interpolate_scalar(a: f64, b: f64, t: f64) -> f64 {
    a + (b - a) * t
}

fn bounds_from_series(
    series: &[PlotSeries],
    axis: usize,
    fallback: (f64, f64),
    min_span: f64,
    pad_ratio: f64,
    floor: Option<f64>,
) -> (f64, f64) {
    let mut min_value = f64::INFINITY;
    let mut max_value = f64::NEG_INFINITY;
    for line in series {
        for point in &line.points {
            if !point[axis].is_finite() {
                continue;
            }
            min_value = min_value.min(point[axis]);
            max_value = max_value.max(point[axis]);
        }
    }
    let (mut low, mut high) = if min_value.is_finite() && max_value.is_finite() {
        (min_value, max_value)
    } else {
        fallback
    };
    let span = (high - low).max(min_span);
    let pad = span * pad_ratio;
    low -= pad;
    high += pad;
    if let Some(min_floor) = floor {
        if low < min_floor {
            high += min_floor - low;
            low = min_floor;
        }
    }
    (low, high.max(low + min_span))
}

fn point_is_finite(point: [f64; 2]) -> bool {
    point[0].is_finite() && point[1].is_finite()
}

fn safe_available_width(ui: &egui::Ui, fallback: f32) -> f32 {
    let width = ui.available_width();
    if width.is_finite() {
        width.max(0.0)
    } else {
        fallback
    }
}

fn safe_available_height(ui: &egui::Ui, fallback: f32) -> f32 {
    let height = ui.available_height();
    if height.is_finite() {
        height.max(0.0)
    } else {
        fallback
    }
}

fn bounded_plot_height(ui: &egui::Ui, fallback: f32, min: f32, max: f32, reserve: f32) -> f32 {
    let panel_height = ui.max_rect().height();
    let candidate = if panel_height.is_finite() {
        (panel_height - reserve).max(min)
    } else {
        fallback
    };
    candidate.clamp(min, max)
}

fn ptheta_display_sample_count(column_width_px: f32, pv_display_bins: usize) -> usize {
    let max_samples = (pv_display_bins / FIXED_CYLINDER_COUNT).max(180);
    let width_based = (column_width_px.max(240.0) * 0.72).round() as usize;
    width_based.clamp(180, max_samples)
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

fn tab_accent(tab: VisualizationTab, theme: DashboardTheme) -> egui::Color32 {
    match tab {
        VisualizationTab::Overview => theme.green,
        VisualizationTab::Map => theme.green,
        VisualizationTab::Fit => theme.green,
        VisualizationTab::Pressure => theme.amber,
        VisualizationTab::Cycle => theme.cyan,
    }
}

fn render_visualization_arming_panel(
    ui: &mut egui::Ui,
    theme: DashboardTheme,
    state: &DashboardState,
) {
    theme.instrument_frame(theme.amber).show(ui, |ui| {
        ui.label(
            egui::RichText::new("Visualization arming")
                .color(theme.amber)
                .strong(),
        );
        ui.label(
            egui::RichText::new(
                "Holding the deck in a static, non-interactive layout until the fired cycle produces stable diagnostic samples.",
            )
            .color(theme.text_soft)
            .monospace()
            .size(10.0),
        );
        ui.add_space(6.0);
        ui.label(
            egui::RichText::new(format!(
                "prime wall {:.1}s / rpm {:.0} / throttle {:.3} / spark {:.1} deg BTDC",
                state.startup_fit_status().wall_elapsed_s,
                state.latest.rpm,
                state.sim.control.throttle_cmd,
                state.sim.control.ignition_timing_deg,
            ))
            .color(theme.text_main)
            .monospace()
            .size(10.5),
        );
        ui.label(
            egui::RichText::new(
                "Tabs and plots unlock automatically after PRIME finishes so the default window stays stable.",
            )
            .color(theme.text_soft)
            .monospace()
            .size(10.0),
        );
        ui.add_space(120.0);
    });
}

#[cfg(test)]
mod tests {
    use super::{
        combustion_fraction_cycle_deg, interpolate_pv_cycle_point,
        latest_complete_pv_cycle_samples, spark_event_cycle_deg, wiebe_fraction_location,
    };
    use crate::simulator::PvSample;
    use std::collections::VecDeque;

    #[test]
    fn combustion_fraction_markers_stay_inside_burn_window() {
        let soc = combustion_fraction_cycle_deg(352.0, 60.0, 5.2, 2.0, 0.0);
        let ca10 = combustion_fraction_cycle_deg(352.0, 60.0, 5.2, 2.0, 0.10);
        let ca50 = combustion_fraction_cycle_deg(352.0, 60.0, 5.2, 2.0, 0.50);
        let ca90 = combustion_fraction_cycle_deg(352.0, 60.0, 5.2, 2.0, 0.90);
        let eoc = combustion_fraction_cycle_deg(352.0, 60.0, 5.2, 2.0, 1.0);

        assert!((soc - 352.0).abs() < 1.0e-12);
        assert!(ca10 > 352.0);
        assert!(ca10 < ca50);
        assert!(ca50 > 352.0);
        assert!(ca50 < 412.0);
        assert!(ca90 > ca50);
        assert!(ca90 < 412.0);
        assert!((eoc - 412.0).abs() < 1.0e-12);
        assert!(wiebe_fraction_location(0.5, 5.2, 2.0) > 0.0);
        assert!(wiebe_fraction_location(0.5, 5.2, 2.0) < 1.0);
    }

    #[test]
    fn spark_event_uses_btdc_convention() {
        assert!((spark_event_cycle_deg(15.0) - 345.0).abs() < 1.0e-12);
    }

    #[test]
    fn pv_marker_interpolates_with_cycle_wrap() {
        let samples = vec![
            PvSample {
                cycle: 7,
                cycle_deg: 700.0,
                volume: 0.40,
                pressure_pa: 400_000.0,
            },
            PvSample {
                cycle: 7,
                cycle_deg: 10.0,
                volume: 0.50,
                pressure_pa: 500_000.0,
            },
            PvSample {
                cycle: 7,
                cycle_deg: 30.0,
                volume: 0.60,
                pressure_pa: 600_000.0,
            },
        ];

        let point = interpolate_pv_cycle_point(&samples, 0.0).expect("wrapped marker point");

        assert!(point[0] > 0.40 && point[0] < 0.50);
        assert!(point[1] > 400.0 && point[1] < 500.0);
    }

    #[test]
    fn marker_source_ignores_incomplete_latest_cycle() {
        let mut history = VecDeque::new();
        for idx in 0..36 {
            let deg = idx as f64 * (719.0 / 35.0);
            history.push_back(PvSample {
                cycle: 10,
                cycle_deg: deg,
                volume: 0.08 + 0.92 * (deg / 719.0),
                pressure_pa: 100_000.0 + 3_900_000.0 * (1.0 - (deg / 719.0)),
            });
        }
        history.push_back(PvSample {
            cycle: 11,
            cycle_deg: 0.0,
            volume: 0.95,
            pressure_pa: 100_000.0,
        });
        history.push_back(PvSample {
            cycle: 11,
            cycle_deg: 120.0,
            volume: 0.90,
            pressure_pa: 120_000.0,
        });

        let selected = latest_complete_pv_cycle_samples(&history).expect("complete cycle");

        assert!(selected.iter().all(|sample| sample.cycle == 10));
    }
}

fn fit_status_color(theme: DashboardTheme, phase: StartupFitPhase) -> egui::Color32 {
    match phase {
        StartupFitPhase::Priming => theme.lamp_off,
        StartupFitPhase::Optimizing => theme.amber,
        StartupFitPhase::Verifying => theme.cyan,
        StartupFitPhase::Ready => theme.green,
    }
}
