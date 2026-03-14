use std::f64::consts::PI;

use eframe::egui;
use egui_plot::{Legend, Line, Plot, PlotBounds, PlotPoints};

use super::super::simple_view_model::{SimpleDashboardViewModel, StatusRow};
use super::super::startup_fit::StartupFitPhase;
use super::super::state::DashboardState;
use super::super::theme::DashboardTheme;
use crate::config::UiConfig;
use crate::constants::FIXED_CYLINDER_COUNT;
use crate::simulator::shaft_power_kw;

pub(crate) fn render_simple_dashboard(
    ctx: &egui::Context,
    theme: DashboardTheme,
    ui_config: &UiConfig,
    state: &mut DashboardState,
) {
    let view = SimpleDashboardViewModel::from_state(state, ui_config);
    render_header(ctx, theme, &view);
    render_controls_panel(ctx, theme, ui_config, state, &view);
    render_visualization_panel(ctx, theme, ui_config, state);
    render_live_status_panel(ctx, theme, &view);
}

fn render_header(ctx: &egui::Context, theme: DashboardTheme, view: &SimpleDashboardViewModel) {
    egui::TopBottomPanel::top("simple_header").show(ctx, |ui| {
        theme.header_frame().show(ui, |ui| {
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
    });
}

fn render_controls_panel(
    ctx: &egui::Context,
    theme: DashboardTheme,
    ui_config: &UiConfig,
    state: &mut DashboardState,
    view: &SimpleDashboardViewModel,
) {
    egui::SidePanel::left("simple_controls")
        .default_width(336.0)
        .resizable(true)
        .show(ctx, |ui| {
            theme.rack_frame().show(ui, |ui| {
                ui.heading("Startup Numerical Fit");
                ui.label(view.fit_detail);

                theme.instrument_frame(theme.cyan).show(ui, |ui| {
                    ui.horizontal(|ui| {
                        ui.colored_label(
                            fit_status_color(theme, view.fit_phase),
                            egui::RichText::new(view.fit_status_value).strong(),
                        );
                        ui.label(
                            egui::RichText::new(&view.fit_progress_label).color(theme.text_soft),
                        );
                    });
                    ui.add(
                        egui::ProgressBar::new(view.fit_progress_fraction)
                            .desired_width(f32::INFINITY)
                            .text(view.fit_progress_label.clone()),
                    );
                });

                egui::Grid::new("fit_grid")
                    .num_columns(2)
                    .spacing(egui::vec2(12.0, 6.0))
                    .show(ui, |ui| {
                        for row in &view.fit_rows {
                            render_status_row(ui, row);
                        }
                    });

                ui.separator();
                ui.heading("Manual Control");
                if let Some(message) = view.manual_control_hint {
                    ui.label(egui::RichText::new(message).color(theme.red));
                }

                theme.instrument_frame(theme.amber).show(ui, |ui| {
                    ui.add_enabled_ui(!view.fit_active, |ui| {
                        ui.add(
                            egui::Slider::new(&mut state.sim.control.throttle_cmd, 0.0..=1.0)
                                .text("Throttle"),
                        );
                        ui.add(
                            egui::Slider::new(
                                &mut state.sim.control.ignition_timing_deg,
                                ui_config.ignition_slider_min_deg
                                    ..=ui_config.ignition_slider_max_deg,
                            )
                            .text("Ignition [deg BTDC]"),
                        );
                        ui.add(
                            egui::Slider::new(
                                &mut state.sim.control.vvt_intake_deg,
                                ui_config.vvt_slider_min_deg..=ui_config.vvt_slider_max_deg,
                            )
                            .text("VVT intake [deg]"),
                        );
                        ui.add(
                            egui::Slider::new(
                                &mut state.sim.control.vvt_exhaust_deg,
                                ui_config.vvt_slider_min_deg..=ui_config.vvt_slider_max_deg,
                            )
                            .text("VVT exhaust [deg]"),
                        );
                        ui.checkbox(&mut state.sim.control.spark_cmd, "Spark");
                        ui.checkbox(&mut state.sim.control.fuel_cmd, "Fuel");
                    });
                });

                ui.separator();
                ui.label(egui::RichText::new(view.external_load_note).color(theme.text_soft));
            });
        });
}

fn render_visualization_panel(
    ctx: &egui::Context,
    theme: DashboardTheme,
    ui_config: &UiConfig,
    state: &DashboardState,
) {
    egui::TopBottomPanel::top("simple_visualization")
        .frame(theme.monitor_frame())
        .resizable(false)
        .default_height(408.0)
        .show(ctx, |ui| {
            render_output_cards(ui, theme, state);
            ui.add_space(8.0);
            render_pressure_plots(ui, theme, ui_config, state, 252.0);
        });
}

fn render_live_status_panel(
    ctx: &egui::Context,
    theme: DashboardTheme,
    view: &SimpleDashboardViewModel,
) {
    egui::CentralPanel::default().show(ctx, |ui| {
        egui::ScrollArea::vertical()
            .auto_shrink([false, false])
            .show(ui, |ui| {
                theme.monitor_frame().show(ui, |ui| {
                    ui.heading("Live Status");
                    ui.label(view.live_status_message);

                    ui.separator();
                    egui::Grid::new("live_status_grid")
                        .num_columns(2)
                        .spacing(egui::vec2(16.0, 8.0))
                        .show(ui, |ui| {
                            for row in &view.live_rows {
                                render_status_row(ui, row);
                            }
                        });
                });
            });
    });
}

fn render_output_cards(ui: &mut egui::Ui, theme: DashboardTheme, state: &DashboardState) {
    let indicated_torque_nm =
        state.latest.indicated_work_cycle_j * FIXED_CYLINDER_COUNT as f64 / (4.0 * PI);
    let net_power_kw = shaft_power_kw(state.latest.rpm, state.latest.torque_net_nm);
    let required_brake_torque_nm = state.latest.torque_load_nm;

    ui.columns(4, |columns| {
        render_metric_card(
            &mut columns[0],
            theme,
            theme.cyan,
            "Indicated torque",
            format!("{:+.1}", indicated_torque_nm),
            "Nm",
            format!(
                "p-V Wi {:.3} kJ/cyl",
                state.latest.indicated_work_cycle_j * 1.0e-3
            ),
        );
        render_metric_card(
            &mut columns[1],
            theme,
            theme.red,
            "Required brake torque",
            format!("{:+.1}", required_brake_torque_nm),
            "Nm",
            format!(
                "bench load cmd {:+.3}",
                state.sim.control.load_cmd
            ),
        );
        render_metric_card(
            &mut columns[2],
            theme,
            theme.amber,
            "Net torque / power",
            format!("{:+.1}", state.latest.torque_net_nm),
            "Nm",
            format!("{:+.1} kW at {:.0} rpm", net_power_kw, state.latest.rpm),
        );
        render_metric_card(
            &mut columns[3],
            theme,
            theme.green,
            "IMEP / eta_i",
            format!("{:.2}", state.latest.imep_bar),
            "bar",
            format!("{:.1}% from p-V loop", state.latest.eta_thermal_indicated_pv * 100.0),
        );
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

fn render_pressure_plots(
    ui: &mut egui::Ui,
    theme: DashboardTheme,
    ui_config: &UiConfig,
    state: &DashboardState,
    plot_height: f32,
) {
    let pv_points: Vec<[f64; 2]> = state
        .latest
        .pv_points
        .iter()
        .map(|(volume, pressure_pa)| [*volume, *pressure_pa * 1.0e-3])
        .collect();
    let pv_y_peak_kpa = pv_points
        .iter()
        .map(|point| point[1])
        .fold(state.sim.plot.pv_y_min_kpa, f64::max);
    let pv_y_max_plot = state.sim.plot.pv_y_max_kpa.max(
        (pv_y_peak_kpa * ui_config.pv_headroom_ratio)
            .max(state.sim.plot.pv_y_min_kpa + ui_config.pv_min_headroom_kpa),
    );

    let plot_width = ((ui.available_width() - 8.0).max(320.0)) * 0.5;
    let sample_count = ptheta_display_sample_count(plot_width, state.sim.model.pv_display_bins);
    let ptheta_curves = state.sim.build_ptheta_display_curves(sample_count);
    let ptheta_y_peak_kpa = ptheta_curves
        .iter()
        .flat_map(|curve| curve.iter().map(|(_, pressure_pa)| *pressure_pa * 1.0e-3))
        .fold(state.sim.plot.pv_y_min_kpa, f64::max);
    let ptheta_y_max_plot = state.sim.plot.pv_y_max_kpa.max(
        (ptheta_y_peak_kpa * ui_config.pv_headroom_ratio)
            .max(state.sim.plot.pv_y_min_kpa + ui_config.pv_min_headroom_kpa),
    );

    let render_pv = |ui: &mut egui::Ui| {
        theme.monitor_frame().show(ui, |ui| {
            ui.horizontal(|ui| {
                ui.label(
                    egui::RichText::new("Cylinder p-V")
                        .color(theme.text_main)
                        .strong(),
                );
                ui.separator();
                ui.label(
                    egui::RichText::new("reconstructed diagnostic loop")
                        .color(theme.amber)
                        .monospace()
                        .size(10.0),
                );
            });
            if pv_points.is_empty() {
                ui.label(
                    egui::RichText::new(
                        "waiting for cylinder pressure samples while the fired state spins up",
                    )
                    .color(theme.text_soft)
                    .monospace()
                    .size(10.0),
                );
            }
            Plot::new("simple_pv_plot")
                .height(plot_height)
                .allow_scroll(false)
                .allow_drag(false)
                .allow_zoom(false)
                .legend(Legend::default())
                .x_axis_label("Normalized volume [-]")
                .y_axis_label("Pressure [kPa]")
                .show(ui, |plot_ui| {
                    plot_ui.set_auto_bounds(false);
                    plot_ui.set_plot_bounds(PlotBounds::from_min_max(
                        [state.sim.plot.pv_x_min, state.sim.plot.pv_y_min_kpa],
                        [state.sim.plot.pv_x_max, pv_y_max_plot],
                    ));
                    let points: PlotPoints<'_> = pv_points.iter().copied().collect();
                    plot_ui.line(
                        Line::new(points)
                            .name("p-V")
                            .color(egui::Color32::from_rgb(255, 170, 40))
                            .width(ui_config.line_width_px),
                    );
                });
        });
    };

    let render_ptheta = |ui: &mut egui::Ui| {
        theme.monitor_frame().show(ui, |ui| {
            ui.horizontal(|ui| {
                ui.label(
                    egui::RichText::new("Cylinder p-theta")
                        .color(theme.text_main)
                        .strong(),
                );
                ui.separator();
                ui.label(
                    egui::RichText::new("4-cylinder overlay / 0..720 degCA")
                        .color(theme.cyan)
                        .monospace()
                        .size(10.0),
                );
            });
            if ptheta_curves.iter().all(|curve| curve.is_empty()) {
                ui.label(
                    egui::RichText::new(
                        "waiting for pressure reconstruction before drawing the 4-cylinder overlay",
                    )
                    .color(theme.text_soft)
                    .monospace()
                    .size(10.0),
                );
            }
            Plot::new("simple_ptheta_plot")
                .height(plot_height)
                .allow_scroll(false)
                .allow_drag(false)
                .allow_zoom(false)
                .legend(Legend::default())
                .x_axis_label("Crank angle [degCA]")
                .y_axis_label("Pressure [kPa]")
                .show(ui, |plot_ui| {
                    plot_ui.set_auto_bounds(false);
                    plot_ui.set_plot_bounds(PlotBounds::from_min_max(
                        [0.0, state.sim.plot.pv_y_min_kpa],
                        [720.0, ptheta_y_max_plot],
                    ));
                    for (idx, curve) in ptheta_curves.iter().enumerate() {
                        let points: PlotPoints<'_> = curve
                            .iter()
                            .map(|(theta, pressure_pa)| [*theta, *pressure_pa * 1.0e-3])
                            .collect();
                        plot_ui.line(
                            Line::new(points)
                                .name(format!("Cyl {}", idx + 1))
                                .color(cylinder_trace_color(idx))
                                .width(ui_config.line_width_px),
                        );
                    }
                });
        });
    };

    if ui.available_width() < 780.0 {
        render_pv(ui);
        ui.add_space(6.0);
        render_ptheta(ui);
    } else {
        ui.columns(2, |columns| {
            render_pv(&mut columns[0]);
            render_ptheta(&mut columns[1]);
        });
    }
}

fn render_status_row(ui: &mut egui::Ui, row: &StatusRow) {
    ui.label(row.label);
    ui.label(&row.value);
    ui.end_row();
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

fn fit_status_color(theme: DashboardTheme, phase: StartupFitPhase) -> egui::Color32 {
    match phase {
        StartupFitPhase::Priming => theme.lamp_off,
        StartupFitPhase::Optimizing => theme.amber,
        StartupFitPhase::Verifying => theme.cyan,
        StartupFitPhase::Ready => theme.green,
    }
}
