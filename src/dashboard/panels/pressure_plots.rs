use eframe::egui;
use egui_plot::{Legend, Line, Plot, PlotBounds, PlotPoints};

use super::super::state::DashboardState;
use super::super::theme::DashboardTheme;
use super::super::view_model::{GraphLayoutHeights, PressurePlotsViewModel};
use super::super::widgets::monitor_heading;
use crate::config::UiConfig;

pub(crate) fn render_pressure_plots(
    ui: &mut egui::Ui,
    theme: DashboardTheme,
    ui_config: &UiConfig,
    state: &DashboardState,
    graph_heights: GraphLayoutHeights,
) {
    let vm = PressurePlotsViewModel::build(state, ui.available_width(), ui_config);

    ui.columns(2, |columns| {
        render_pv_plot(&mut columns[0], theme, state, &vm, graph_heights, ui_config);
        render_ptheta_plot(&mut columns[1], theme, state, &vm, graph_heights, ui_config);
    });
    ui.add_space(graph_heights.section_spacing_px);
}

fn render_pv_plot(
    ui: &mut egui::Ui,
    theme: DashboardTheme,
    state: &DashboardState,
    vm: &PressurePlotsViewModel,
    graph_heights: GraphLayoutHeights,
    ui_config: &UiConfig,
) {
    let pv_points: PlotPoints<'_> = vm.pv_points.iter().copied().collect();
    let pv_line = Line::new(pv_points)
        .name("p-V")
        .color(egui::Color32::from_rgb(255, 170, 40))
        .width(ui_config.line_width_px);

    theme.monitor_frame().show(ui, |ui| {
        monitor_heading(
            ui,
            theme,
            "Cylinder p-V",
            "reconstructed display loop",
            theme.amber,
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
                    [state.sim.plot.pv_x_min, state.sim.plot.pv_y_min_kpa],
                    [state.sim.plot.pv_x_max, vm.pv_y_max_plot],
                ));
                plot_ui.line(pv_line);
            });
    });
}

fn render_ptheta_plot(
    ui: &mut egui::Ui,
    theme: DashboardTheme,
    state: &DashboardState,
    vm: &PressurePlotsViewModel,
    graph_heights: GraphLayoutHeights,
    ui_config: &UiConfig,
) {
    theme.monitor_frame().show(ui, |ui| {
        monitor_heading(
            ui,
            theme,
            "Cylinder p-theta",
            "4-cylinder overlay / 0..720 degCA",
            theme.cyan,
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
                    [0.0, state.sim.plot.pv_y_min_kpa],
                    [720.0, vm.ptheta_y_max_plot],
                ));
                for curve in &vm.ptheta_curves {
                    let points: PlotPoints<'_> = curve.points.iter().copied().collect();
                    let mut line = Line::new(points)
                        .color(curve.color)
                        .width(ui_config.line_width_px);
                    if let Some(name) = &curve.name {
                        line = line.name(name.clone());
                    }
                    plot_ui.line(line);
                }
            });
    });
}
