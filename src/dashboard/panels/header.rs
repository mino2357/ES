use eframe::egui;

use super::super::state::DashboardState;
use super::super::theme::DashboardTheme;
use super::super::view_model::HeaderViewModel;
use super::super::widgets::{annunciator, digital_readout};
use crate::config::UiConfig;

pub(crate) fn render_header_panel(
    ctx: &egui::Context,
    theme: DashboardTheme,
    ui_config: &UiConfig,
    state: &DashboardState,
) {
    let vm = HeaderViewModel::build(state, ui_config);

    egui::TopBottomPanel::top("header")
        .frame(theme.header_frame())
        .show(ctx, |ui| {
            ui.horizontal(|ui| {
                ui.vertical(|ui| {
                    ui.label(
                        egui::RichText::new("EDM TEST CELL // INLINE-4 DYNAMOMETER")
                            .color(theme.text_main)
                            .strong()
                            .size(26.0),
                    );
                    ui.label(
                        egui::RichText::new(&vm.subtitle)
                            .color(theme.text_soft)
                            .size(12.0),
                    );
                });
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    digital_readout(
                        ui,
                        theme,
                        if vm.fit_ready {
                            theme.green
                        } else {
                            theme.amber
                        },
                        "STARTUP FIT",
                        &vm.fit_status_value,
                        "",
                        &vm.fit_status_footer,
                        168.0,
                    );
                    ui.add_space(8.0);
                    digital_readout(
                        ui,
                        theme,
                        theme.cyan,
                        "SIM STATUS",
                        &vm.sim_status_value,
                        "",
                        &vm.sim_status_footer,
                        160.0,
                    );
                });
            });
            ui.add_space(8.0);
            ui.horizontal_wrapped(|ui| {
                annunciator(ui, theme, "RUN", vm.run_active, theme.green);
                annunciator(ui, theme, "FUEL", vm.fuel_active, theme.amber);
                annunciator(ui, theme, "SPARK", vm.spark_active, theme.cyan);
                annunciator(ui, theme, "MOTOR", vm.motor_active, theme.red);
                annunciator(ui, theme, "FIRING", vm.firing_active, theme.green);
                annunciator(ui, theme, "LOAD CTRL", vm.load_ctrl_active, theme.amber);
                annunciator(ui, theme, "PWR LIM", vm.power_limit_active, theme.red);
                annunciator(ui, theme, "ACCURACY", vm.accuracy_active, theme.green);
                annunciator(ui, theme, "2000 FIT", vm.fit_active, theme.amber);
                annunciator(ui, theme, "FIT READY", vm.fit_ready, theme.green);
            });
        });
}
