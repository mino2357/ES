use eframe::egui;

use super::super::state::DashboardState;
use super::super::theme::DashboardTheme;
use super::super::view_model::{
    RoughCalibrationViewModel, SensorStateBusViewModel, StatusBusViewModel,
};
use super::super::widgets::{metric_row, section_label};
use super::show_collapsible_module;
use crate::config::{ExternalLoadMode, UiConfig};

pub(crate) fn render_control_rack(
    ctx: &egui::Context,
    theme: DashboardTheme,
    ui_config: &UiConfig,
    state: &mut DashboardState,
) {
    egui::SidePanel::left("controls")
        .default_width(272.0)
        .resizable(true)
        .frame(theme.rack_frame())
        .show(ctx, |ui| {
            egui::ScrollArea::vertical()
                .auto_shrink([false, false])
                .show(ui, |ui| {
                    section_label(ui, theme, "OPERATOR RACK", theme.amber);
                    let calibration_vm = RoughCalibrationViewModel::build(state);

                    show_collapsible_module(
                        ui,
                        theme,
                        "rack_startup_fit",
                        "2000 RPM Rough Fit",
                        "bootstraps a fired operating point and trims toward stable speed hold",
                        if calibration_vm.ready {
                            theme.green
                        } else {
                            theme.amber
                        },
                        true,
                        |ui| {
                            if calibration_vm.active {
                                ui.horizontal(|ui| {
                                    ui.spinner();
                                    ui.label(
                                        egui::RichText::new(
                                            "Calculating a coarse 2000 rpm fit before manual trim",
                                        )
                                        .color(theme.text_main),
                                    );
                                });
                            } else {
                                ui.label(
                                    egui::RichText::new(
                                        "Rough 2000 rpm fit is ready. Manual control is unlocked.",
                                    )
                                    .color(theme.text_main),
                                );
                            }
                            ui.label(
                                egui::RichText::new(calibration_vm.detail)
                                    .color(theme.text_soft)
                                    .size(10.5),
                            );
                            ui.add_space(4.0);
                            ui.add(
                                egui::ProgressBar::new(calibration_vm.progress_fraction)
                                    .desired_width(f32::INFINITY)
                                    .text(calibration_vm.progress_label.clone()),
                            );
                            ui.add_space(6.0);
                            egui::Grid::new("rough_fit_grid")
                                .num_columns(2)
                                .spacing(egui::vec2(16.0, 6.0))
                                .show(ui, |ui| {
                                    metric_row(
                                        ui,
                                        theme,
                                        "Target / elapsed",
                                        format!(
                                            "{} / {}",
                                            calibration_vm.target_text, calibration_vm.elapsed_text
                                        ),
                                    );
                                    metric_row(
                                        ui,
                                        theme,
                                        "Avg rpm / machine",
                                        format!(
                                            "{} / {}",
                                            calibration_vm.avg_rpm_text,
                                            calibration_vm.machine_torque_text
                                        ),
                                    );
                                    metric_row(
                                        ui,
                                        theme,
                                        "Throttle / ignition",
                                        format!(
                                            "{} / {}",
                                            calibration_vm.throttle_text,
                                            calibration_vm.ignition_text
                                        ),
                                    );
                                    metric_row(
                                        ui,
                                        theme,
                                        "Phase summary",
                                        calibration_vm.summary.clone(),
                                    );
                                });
                        },
                    );

                    ui.add_space(8.0);
                    let actuator_subtitle = if calibration_vm.active {
                        "rough 2000 rpm fit is driving throttle and ignition; controls unlock when ready"
                    } else {
                        "load-control console with throttle, ignition, and VVT inputs"
                    };

                    show_collapsible_module(
                        ui,
                        theme,
                        "rack_actuator",
                        "Actuator Deck",
                        actuator_subtitle,
                        theme.amber,
                        true,
                        |ui| {
                            if calibration_vm.active {
                                ui.label(
                                    egui::RichText::new(
                                        "Manual actuator edits are temporarily locked to keep the coarse fit stable.",
                                    )
                                    .color(theme.text_soft)
                                    .size(10.5),
                                );
                                ui.add_space(4.0);
                            }

                            let mut load_mode = state.sim.model.external_load.mode;
                            ui.add_enabled_ui(!calibration_vm.active, |ui| {
                                ui.add(
                                    egui::Slider::new(
                                        &mut state.sim.control.throttle_cmd,
                                        0.0..=1.0,
                                    )
                                    .text("Throttle cmd"),
                                );
                                let target_rpm_before = state.load_target_rpm;
                                ui.add(
                                    egui::Slider::new(
                                        &mut state.load_target_rpm,
                                        0.0..=state.sim.params.max_rpm,
                                    )
                                    .text("Target RPM"),
                                );
                                if (state.load_target_rpm - target_rpm_before).abs() > f64::EPSILON
                                {
                                    // Avoid carrying stale integral action across large operator changes.
                                    state.reset_load_speed_integral();
                                }
                                ui.label(format!(
                                    "Required brake torque: {:.1} Nm",
                                    state.required_brake_torque_nm
                                ));
                                ui.label(format!(
                                    "Required brake power: {:.1} kW",
                                    state.required_brake_power_kw()
                                ));
                                ui.label(format!(
                                    "RPM error: {:+.0} rpm / absorber torque {:.1} Nm",
                                    state.rpm_error(),
                                    state.latest.torque_load_nm
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
                                ui.label(format!(
                                    "Absorber command: {:+.3}",
                                    state.sim.control.load_cmd
                                ));
                                ui.checkbox(&mut state.sim.control.spark_cmd, "Spark");
                                ui.checkbox(&mut state.sim.control.fuel_cmd, "Fuel");
                                ui.separator();
                                ui.add(
                                    egui::Slider::new(
                                        &mut state.sim.control.vvt_intake_deg,
                                        ui_config.vvt_slider_min_deg
                                            ..=ui_config.vvt_slider_max_deg,
                                    )
                                    .text("VVT Intake [deg]"),
                                );
                                ui.add(
                                    egui::Slider::new(
                                        &mut state.sim.control.vvt_exhaust_deg,
                                        ui_config.vvt_slider_min_deg
                                            ..=ui_config.vvt_slider_max_deg,
                                    )
                                    .text("VVT Exhaust [deg]"),
                                );
                                ui.add(
                                    egui::Slider::new(
                                        &mut state.sim.control.ignition_timing_deg,
                                        ui_config.ignition_slider_min_deg
                                            ..=ui_config.ignition_slider_max_deg,
                                    )
                                    .text("Ignition [deg BTDC]"),
                                );
                            });
                            if !calibration_vm.active && load_mode != state.sim.model.external_load.mode {
                                state.sim.model.external_load.mode = load_mode;
                            }
                        },
                    );

                    ui.add_space(8.0);
                    let status_vm = StatusBusViewModel::build(state, ui_config);
                    show_collapsible_module(
                        ui,
                        theme,
                        "rack_status",
                        "Status Bus",
                        "cell and runtime telemetry",
                        theme.green,
                        true,
                        |ui| {
                            egui::Grid::new("status_bus_grid")
                                .num_columns(2)
                                .spacing(egui::vec2(16.0, 6.0))
                                .show(ui, |ui| {
                                    for row in &status_vm.rows {
                                        metric_row(ui, theme, row.label, row.value.clone());
                                    }
                                });
                        },
                    );

                    ui.add_space(8.0);
                    let sensor_vm = SensorStateBusViewModel::build(state);
                    render_state_bus(ui, theme, &sensor_vm);
                });
        });
}

fn render_state_bus(ui: &mut egui::Ui, theme: DashboardTheme, vm: &SensorStateBusViewModel) {
    show_collapsible_module(
        ui,
        theme,
        "rack_sensor_state",
        "Sensor / State Bus",
        "live reduced-order state and closures",
        theme.green,
        true,
        |ui| {
            egui::Grid::new("state_bus_grid")
                .num_columns(2)
                .spacing(egui::vec2(16.0, 6.0))
                .show(ui, |ui| {
                    for row in &vm.rows {
                        metric_row(ui, theme, row.label, row.value.clone());
                    }
                });
        },
    );
}
