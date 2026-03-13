use eframe::egui;

use super::super::simple_view_model::{SimpleDashboardViewModel, StatusRow};
use super::super::startup_fit::RoughCalibrationPhase;
use super::super::state::DashboardState;
use super::super::theme::DashboardTheme;
use crate::config::UiConfig;

pub(crate) fn render_simple_dashboard(
    ctx: &egui::Context,
    theme: DashboardTheme,
    ui_config: &UiConfig,
    state: &mut DashboardState,
) {
    let view = SimpleDashboardViewModel::from_state(state, ui_config);
    render_header(ctx, theme, &view);
    render_controls_panel(ctx, theme, ui_config, state, &view);
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
                        "Rough-fit a self-sustaining 2000 rpm point, then unlock manual trim",
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
        .default_width(320.0)
        .resizable(true)
        .show(ctx, |ui| {
            theme.rack_frame().show(ui, |ui| {
                ui.heading("2000 rpm Startup Fit");
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

fn render_live_status_panel(
    ctx: &egui::Context,
    theme: DashboardTheme,
    view: &SimpleDashboardViewModel,
) {
    egui::CentralPanel::default().show(ctx, |ui| {
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
}

fn render_status_row(ui: &mut egui::Ui, row: &StatusRow) {
    ui.label(row.label);
    ui.label(&row.value);
    ui.end_row();
}

fn fit_status_color(theme: DashboardTheme, phase: RoughCalibrationPhase) -> egui::Color32 {
    match phase {
        RoughCalibrationPhase::Priming => theme.lamp_off,
        RoughCalibrationPhase::Searching => theme.amber,
        RoughCalibrationPhase::Settling => theme.cyan,
        RoughCalibrationPhase::Ready => theme.green,
    }
}
