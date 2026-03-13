use eframe::egui;
use egui_plot::{Legend, Line, Plot, PlotBounds, PlotPoints, VLine};

use super::super::state::DashboardState;
use super::super::theme::DashboardTheme;
use super::super::view_model::{CycleMonitorsViewModel, GraphLayoutHeights, SchematicViewModel};
use super::schematic::render_engine_motion_schematic;
use super::show_collapsible_module;
use crate::config::UiConfig;

pub(crate) fn render_cycle_monitors(
    ui: &mut egui::Ui,
    theme: DashboardTheme,
    ui_config: &UiConfig,
    state: &mut DashboardState,
    graph_heights: GraphLayoutHeights,
) {
    show_collapsible_module(
        ui,
        theme,
        "cycle_monitors",
        "Cycle Monitors",
        "recent histories and valve event trace",
        theme.cyan,
        true,
        |ui| {
            let vm = CycleMonitorsViewModel::build(state, ui_config);
            let schematic_vm = SchematicViewModel::build(state);

            ui.columns(2, |columns| {
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
                            [0.0, vm.rpm_y_min],
                            [720.0, vm.rpm_y_max],
                        ));
                        for line_vm in &vm.rpm_lines {
                            let points: PlotPoints<'_> = line_vm.points.iter().copied().collect();
                            let mut line = Line::new(points)
                                .color(line_vm.color)
                                .width(ui_config.line_width_px);
                            if let Some(name) = &line_vm.name {
                                line = line.name(name.clone());
                            }
                            plot_ui.line(line);
                        }
                        plot_ui.vline(
                            VLine::new(vm.cycle_deg)
                                .name("Crank cursor")
                                .color(theme.chrome),
                        );
                    });

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
                            [0.0, vm.torque_y_min],
                            [720.0, vm.torque_y_max],
                        ));
                        for series in [&vm.torque_lines, &vm.load_lines] {
                            for line_vm in series {
                                let points: PlotPoints<'_> =
                                    line_vm.points.iter().copied().collect();
                                let mut line = Line::new(points)
                                    .color(line_vm.color)
                                    .width(ui_config.line_width_px);
                                if let Some(name) = &line_vm.name {
                                    line = line.name(name.clone());
                                }
                                plot_ui.line(line);
                            }
                        }
                        plot_ui.vline(
                            VLine::new(vm.cycle_deg)
                                .name("Crank cursor")
                                .color(theme.chrome),
                        );
                    });
            });

            ui.add_space(graph_heights.section_spacing_px);

            ui.columns(2, |columns| {
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
                            [0.0, vm.trapped_air_y_min],
                            [720.0, vm.trapped_air_y_max],
                        ));
                        for line_vm in &vm.trapped_air_lines {
                            let points: PlotPoints<'_> = line_vm.points.iter().copied().collect();
                            let mut line = Line::new(points)
                                .color(line_vm.color)
                                .width(ui_config.line_width_px);
                            if let Some(name) = &line_vm.name {
                                line = line.name(name.clone());
                            }
                            plot_ui.line(line);
                        }
                        plot_ui.vline(
                            VLine::new(vm.cycle_deg)
                                .name("Crank cursor")
                                .color(theme.chrome),
                        );
                    });

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
                            [720.0, vm.cam_y_max],
                        ));
                        let intake_points: PlotPoints<'_> =
                            vm.intake_cam_points.iter().copied().collect();
                        let exhaust_points: PlotPoints<'_> =
                            vm.exhaust_cam_points.iter().copied().collect();
                        plot_ui.line(
                            Line::new(intake_points)
                                .name("Intake lift")
                                .color(egui::Color32::from_rgb(80, 190, 255))
                                .width(ui_config.line_width_px),
                        );
                        plot_ui.line(
                            Line::new(exhaust_points)
                                .name("Exhaust lift")
                                .color(egui::Color32::from_rgb(255, 110, 110))
                                .width(ui_config.line_width_px),
                        );
                        plot_ui.vline(
                            VLine::new(vm.cycle_deg)
                                .name("Crank")
                                .color(egui::Color32::from_rgb(170, 170, 170))
                                .width(ui_config.crank_line_width_px),
                        );
                    });
            });

            ui.add_space(graph_heights.section_spacing_px);
            render_engine_motion_schematic(ui, theme, graph_heights, &schematic_vm);
        },
    );
}
