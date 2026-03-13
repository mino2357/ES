use eframe::egui;

use super::super::state::DashboardState;
use super::super::theme::DashboardTheme;
use super::super::view_model::{OverviewViewModel, responsive_card_columns, responsive_card_width};
use super::super::widgets::{
    GaugeSpec, LinearMeterSpec, digital_readout, gauge, linear_meter, monitor_heading,
};
use crate::config::UiConfig;

pub(crate) fn render_console_overview(
    ui: &mut egui::Ui,
    theme: DashboardTheme,
    ui_config: &UiConfig,
    state: &DashboardState,
) {
    let vm = OverviewViewModel::build(state, ui_config, theme);
    let panel_width = ui.available_width().max(320.0);
    let readout_columns = responsive_card_columns(panel_width, 176.0, 6);
    let readout_width = responsive_card_width(panel_width, readout_columns).min(228.0);
    let gauge_columns = responsive_card_columns(panel_width, 136.0, 6);
    let gauge_width = responsive_card_width(panel_width, gauge_columns).min(172.0);
    let meter_columns = responsive_card_columns(panel_width, 188.0, 3);
    let meter_width = responsive_card_width(panel_width, meter_columns).min(252.0);

    theme.monitor_frame().show(ui, |ui| {
        monitor_heading(
            ui,
            theme,
            "Operator Display",
            "primary live instruments",
            theme.amber,
        );

        for row in vm.readouts.chunks(readout_columns) {
            ui.horizontal(|ui| {
                ui.spacing_mut().item_spacing.x = 8.0;
                for card in row {
                    digital_readout(
                        ui,
                        theme,
                        card.accent,
                        card.label,
                        &card.value,
                        card.unit,
                        &card.footer,
                        readout_width,
                    );
                }
            });
        }

        ui.add_space(6.0);
        for row in vm.gauges.chunks(gauge_columns) {
            ui.horizontal(|ui| {
                ui.spacing_mut().item_spacing.x = 8.0;
                for card in row {
                    gauge(
                        ui,
                        theme,
                        GaugeSpec {
                            label: card.label,
                            value: card.value,
                            min: card.min,
                            max: card.max,
                            unit: card.unit,
                            accent: card.accent,
                            footer: card.footer,
                            width: gauge_width,
                            height: 112.0,
                        },
                    );
                }
            });
        }

        ui.add_space(6.0);
        for row in vm.meters.chunks(meter_columns) {
            ui.horizontal(|ui| {
                ui.spacing_mut().item_spacing.x = 8.0;
                for card in row {
                    linear_meter(
                        ui,
                        theme,
                        LinearMeterSpec {
                            label: card.label,
                            value: card.value,
                            min: card.min,
                            max: card.max,
                            accent: card.accent,
                            value_text: card.value_text.clone(),
                            width: meter_width,
                        },
                    );
                }
            });
        }
    });
    ui.add_space(8.0);
}
