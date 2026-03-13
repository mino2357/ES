use std::collections::VecDeque;

use eframe::egui;

use super::state::{DashboardState, RoughCalibrationPhase, wrap_cycle_deg};
use super::theme::DashboardTheme;
use crate::config::UiConfig;
use crate::constants::FIXED_CYLINDER_COUNT;
use crate::simulator::{CycleHistorySample, cam_lift_mm, shaft_power_hp, shaft_power_kw};

#[derive(Debug, Clone, Copy)]
pub(super) struct GraphLayoutHeights {
    pub(super) standard_plot_px: f32,
    pub(super) pv_plot_px: f32,
    pub(super) section_spacing_px: f32,
}

pub(super) fn graph_layout_heights(ui_config: &UiConfig) -> GraphLayoutHeights {
    let scale = 0.84;
    let standard_plot_px = (ui_config.plot_height_px * scale).max(72.0);
    let pv_plot_px = (ui_config.pv_plot_height_px * scale).max(standard_plot_px * 1.55);

    GraphLayoutHeights {
        standard_plot_px,
        pv_plot_px,
        section_spacing_px: 4.0,
    }
}

pub(super) fn responsive_card_columns(
    available_width: f32,
    min_card_width: f32,
    max_columns: usize,
) -> usize {
    let spacing = 8.0;
    let fitted = ((available_width + spacing) / (min_card_width + spacing)).floor() as usize;
    fitted.clamp(1, max_columns.max(1))
}

pub(super) fn responsive_card_width(available_width: f32, columns: usize) -> f32 {
    let spacing = 8.0;
    let total_spacing = spacing * columns.saturating_sub(1) as f32;
    ((available_width - total_spacing) / columns.max(1) as f32).max(96.0)
}

#[derive(Debug, Clone)]
pub(super) struct RoughCalibrationViewModel {
    pub(super) active: bool,
    pub(super) ready: bool,
    pub(super) phase_label: &'static str,
    pub(super) status_value: String,
    pub(super) detail: &'static str,
    pub(super) summary: String,
    pub(super) progress_label: String,
    pub(super) progress_fraction: f32,
    pub(super) target_text: String,
    pub(super) elapsed_text: String,
    pub(super) avg_rpm_text: String,
    pub(super) machine_torque_text: String,
    pub(super) throttle_text: String,
    pub(super) ignition_text: String,
}

impl RoughCalibrationViewModel {
    pub(super) fn build(state: &DashboardState) -> Self {
        let status = state.rough_calibration_status();
        let ready = !status.active && status.phase == RoughCalibrationPhase::Ready;
        let iteration_fraction =
            (status.iteration as f32 / status.max_iterations.max(1) as f32).clamp(0.0, 1.0);
        let stable_fraction = (status.stable_windows as f32
            / status.required_stable_windows.max(1) as f32)
            .clamp(0.0, 1.0);
        let progress_fraction = match status.phase {
            RoughCalibrationPhase::Priming => 0.14,
            RoughCalibrationPhase::Searching => 0.22 + 0.48 * iteration_fraction,
            RoughCalibrationPhase::Settling => 0.74 + 0.20 * stable_fraction,
            RoughCalibrationPhase::Ready => 1.0,
        }
        .clamp(0.0, 1.0);
        let status_value = match status.phase {
            RoughCalibrationPhase::Priming => "PRIME",
            RoughCalibrationPhase::Searching => "SEARCH",
            RoughCalibrationPhase::Settling => "SETTLE",
            RoughCalibrationPhase::Ready => "READY",
        }
        .to_owned();
        let progress_label = match status.phase {
            RoughCalibrationPhase::Priming => {
                format!(
                    "priming fired point / {:.2} s simulated",
                    status.simulated_elapsed_s
                )
            }
            RoughCalibrationPhase::Searching | RoughCalibrationPhase::Settling => format!(
                "step {}/{} / stable {}/{}",
                status.iteration,
                status.max_iterations,
                status.stable_windows,
                status.required_stable_windows
            ),
            RoughCalibrationPhase::Ready => "manual controls unlocked".to_owned(),
        };
        let summary = if ready {
            format!("2000 rpm coarse fit ready / avg {:.0} rpm", status.avg_rpm)
        } else {
            format!(
                "{} / avg {:.0} rpm / machine {:+.1} Nm",
                status.phase.label(),
                status.avg_rpm,
                status.avg_machine_torque_nm
            )
        };

        Self {
            active: status.active,
            ready,
            phase_label: status.phase.label(),
            status_value,
            detail: status.phase.detail(),
            summary,
            progress_label,
            progress_fraction,
            target_text: format!("{:.0} rpm", status.target_rpm),
            elapsed_text: format!("{:.2} s", status.simulated_elapsed_s),
            avg_rpm_text: format!("{:.0} rpm", status.avg_rpm),
            machine_torque_text: format!("{:+.1} Nm", status.avg_machine_torque_nm),
            throttle_text: format!("{:.3}", status.throttle_cmd),
            ignition_text: format!("{:.1} deg BTDC", status.ignition_timing_deg),
        }
    }
}

#[derive(Debug, Clone)]
pub(super) struct HeaderViewModel {
    pub(super) sim_status_value: String,
    pub(super) sim_status_footer: String,
    pub(super) fit_status_value: String,
    pub(super) fit_status_footer: String,
    pub(super) subtitle: String,
    pub(super) run_active: bool,
    pub(super) fuel_active: bool,
    pub(super) spark_active: bool,
    pub(super) motor_active: bool,
    pub(super) firing_active: bool,
    pub(super) load_ctrl_active: bool,
    pub(super) power_limit_active: bool,
    pub(super) accuracy_active: bool,
    pub(super) fit_active: bool,
    pub(super) fit_ready: bool,
}

impl HeaderViewModel {
    pub(super) fn build(state: &DashboardState, ui_config: &UiConfig) -> Self {
        let calibration = RoughCalibrationViewModel::build(state);
        let runtime_mode = if ui_config.sync_to_wall_clock {
            "Wall-clock synchronized"
        } else {
            "Accuracy-first"
        };
        Self {
            sim_status_value: if state.required_brake_torque_nm > 0.5 {
                "FIRED"
            } else {
                "MOTOR"
            }
            .to_owned(),
            sim_status_footer: state.sim.model.external_load.mode.label().to_owned(),
            fit_status_value: calibration.status_value,
            fit_status_footer: calibration.summary,
            subtitle: if calibration.active {
                format!(
                    "{runtime_mode} console: coarse 2000 rpm fit is running and manual actuators are locked"
                )
            } else if calibration.ready {
                format!(
                    "{runtime_mode} console: coarse 2000 rpm fit is ready and manual trim is unlocked"
                )
            } else {
                format!("{runtime_mode} console with p-V / p-theta monitoring")
            },
            run_active: state.latest.rpm > 120.0,
            fuel_active: state.sim.control.fuel_cmd,
            spark_active: state.sim.control.spark_cmd,
            motor_active: state.sim.control.load_cmd < -0.02,
            firing_active: state.required_brake_torque_nm > 0.5,
            load_ctrl_active: true,
            power_limit_active: state.dyno_power_limit_active(),
            accuracy_active: !ui_config.sync_to_wall_clock,
            fit_active: calibration.active,
            fit_ready: calibration.ready,
        }
    }
}

#[derive(Debug, Clone)]
pub(super) struct MetricRowViewModel {
    pub(super) label: &'static str,
    pub(super) value: String,
}

#[derive(Debug, Clone)]
pub(super) struct StatusBusViewModel {
    pub(super) rows: Vec<MetricRowViewModel>,
}

impl StatusBusViewModel {
    pub(super) fn build(state: &DashboardState, ui_config: &UiConfig) -> Self {
        let calibration = RoughCalibrationViewModel::build(state);
        let load_model = &state.sim.model.external_load;
        let mut rows = Vec::with_capacity(20);
        rows.push(MetricRowViewModel {
            label: "Engine layout",
            value: format!("fixed {}-cylinder", FIXED_CYLINDER_COUNT),
        });
        rows.push(MetricRowViewModel {
            label: "Throttle eff",
            value: format!("{:.3}", state.sim.state.throttle_eff),
        });
        rows.push(MetricRowViewModel {
            label: "Operator input",
            value: if calibration.active {
                "Auto fit -> hold 2000 rpm".to_owned()
            } else {
                "Throttle cmd + Target RPM".to_owned()
            },
        });
        rows.push(MetricRowViewModel {
            label: "Startup fit",
            value: if calibration.active {
                format!("{} / controls locked", calibration.phase_label)
            } else {
                "ready / manual controls".to_owned()
            },
        });
        rows.push(MetricRowViewModel {
            label: "Fit progress",
            value: calibration.progress_label.clone(),
        });
        rows.push(MetricRowViewModel {
            label: "Fit avg rpm / machine",
            value: format!(
                "{} / {}",
                calibration.avg_rpm_text, calibration.machine_torque_text
            ),
        });
        rows.push(MetricRowViewModel {
            label: "Fit throttle / ign",
            value: format!(
                "{} / {}",
                calibration.throttle_text, calibration.ignition_text
            ),
        });
        rows.push(MetricRowViewModel {
            label: "Target RPM",
            value: format!("{:.0}", state.load_target_rpm),
        });
        rows.push(MetricRowViewModel {
            label: "Required brake torque",
            value: format!("{:.1} Nm", state.required_brake_torque_nm),
        });
        rows.push(MetricRowViewModel {
            label: "Required brake power",
            value: format!("{:.1} kW", state.required_brake_power_kw()),
        });
        rows.push(MetricRowViewModel {
            label: "RPM error",
            value: format!("{:+.0} rpm", state.rpm_error()),
        });
        rows.push(MetricRowViewModel {
            label: "Absorber command",
            value: format!("{:+.3}", state.sim.control.load_cmd),
        });
        rows.push(MetricRowViewModel {
            label: "Absorber torque / shaft est",
            value: format!(
                "{:+.1} / {:+.1} Nm",
                state.latest.torque_load_nm,
                state.shaft_torque_estimate_nm()
            ),
        });
        rows.push(MetricRowViewModel {
            label: "Load model",
            value: load_model.mode.label().to_owned(),
        });
        rows.push(MetricRowViewModel {
            label: "Absorber limit",
            value: format!(
                "{:.0} kW / {:.0} rpm",
                load_model.absorber_power_limit_kw, load_model.absorber_speed_limit_rpm
            ),
        });
        if load_model.mode == crate::config::ExternalLoadMode::VehicleEquivalent {
            rows.push(MetricRowViewModel {
                label: "Vehicle eq",
                value: format!(
                    "{:.1} km/h / Jref {:.3} kg m^2",
                    crate::simulator::external_load_vehicle_speed_kph(
                        state.sim.state.omega_rad_s,
                        load_model,
                    ),
                    crate::simulator::external_load_reflected_inertia_kgm2(
                        state.sim.control.load_cmd,
                        load_model,
                    )
                ),
            });
        }
        rows.push(MetricRowViewModel {
            label: "Solver mode",
            value: state
                .solver_mode_label(ui_config)
                .replacen("Solver mode: ", "", 1),
        });
        rows.push(MetricRowViewModel {
            label: "State",
            value: state.firing_state_label().replacen("State: ", "", 1),
        });
        rows.push(MetricRowViewModel {
            label: "Machine mode",
            value: state.machine_mode_label().replacen("Machine mode: ", "", 1),
        });
        Self { rows }
    }
}

#[derive(Debug, Clone)]
pub(super) struct SensorStateBusViewModel {
    pub(super) rows: Vec<MetricRowViewModel>,
}

impl SensorStateBusViewModel {
    pub(super) fn build(state: &DashboardState) -> Self {
        let mut rows = Vec::with_capacity(22);
        rows.push(MetricRowViewModel {
            label: "Absorber torque act",
            value: format!("{:+.1} Nm", state.latest.torque_load_nm),
        });
        rows.push(MetricRowViewModel {
            label: "Absorber torque avail",
            value: format!("{:.1} Nm", state.dyno_available_torque_nm()),
        });
        rows.push(MetricRowViewModel {
            label: "Absorber power",
            value: format!(
                "{:.1} / {:.0} kW",
                shaft_power_kw(state.latest.rpm, state.latest.torque_load_nm.abs()),
                state.sim.model.external_load.absorber_power_limit_kw
            ),
        });
        rows.push(MetricRowViewModel {
            label: "Intake runner pressure",
            value: format!("{:.1} kPa", state.latest.intake_runner_kpa),
        });
        rows.push(MetricRowViewModel {
            label: "Exhaust pressure",
            value: format!("{:.1} kPa", state.latest.exhaust_kpa),
        });
        rows.push(MetricRowViewModel {
            label: "Exhaust runner pressure",
            value: format!("{:.1} kPa", state.latest.exhaust_runner_kpa),
        });
        rows.push(MetricRowViewModel {
            label: "Wave pressure I / E",
            value: format!(
                "{:.2} / {:.2} kPa",
                state.latest.intake_wave_kpa, state.latest.exhaust_wave_kpa
            ),
        });
        rows.push(MetricRowViewModel {
            label: "Air flow",
            value: format!("{:.2} g/s", state.latest.air_flow_gps),
        });
        rows.push(MetricRowViewModel {
            label: "Charge temp",
            value: format!("{:.1} K", state.latest.intake_charge_temp_k),
        });
        rows.push(MetricRowViewModel {
            label: "Pulse VE / ram / scav",
            value: format!(
                "{:.3} / {:.3} / {:.3}",
                state.latest.ve_pulse_multiplier,
                state.latest.intake_ram_multiplier,
                state.latest.exhaust_scavenge_multiplier
            ),
        });
        rows.push(MetricRowViewModel {
            label: "Internal EGR",
            value: format!("{:.1} %", state.latest.internal_egr_fraction * 100.0),
        });
        rows.push(MetricRowViewModel {
            label: "Ignition timing",
            value: format!("{:.1} deg BTDC", state.latest.ignition_timing_deg),
        });
        rows.push(MetricRowViewModel {
            label: "Heat loss",
            value: format!(
                "{:.3} kJ/cyl/cycle",
                state.latest.heat_loss_cycle_j * 1.0e-3
            ),
        });
        rows.push(MetricRowViewModel {
            label: "IMEP",
            value: format!("{:.2} bar", state.latest.imep_bar),
        });
        rows.push(MetricRowViewModel {
            label: "Theoretical eta",
            value: format!("{:.1} %", state.latest.eta_thermal_theoretical * 100.0),
        });
        rows.push(MetricRowViewModel {
            label: "Cycle angle / burn rate",
            value: format!(
                "{:.1} deg / {:.2}",
                state.latest.cycle_deg, state.latest.combustion_rate_norm
            ),
        });
        rows.push(MetricRowViewModel {
            label: "Net torque inst / filt",
            value: format!(
                "{:+.2} / {:+.2} Nm",
                state.latest.torque_net_inst_nm, state.latest.torque_net_nm
            ),
        });
        rows.push(MetricRowViewModel {
            label: "Torque comb inst / cycle mean",
            value: format!(
                "{:.2} / {:.2} Nm",
                state.latest.torque_combustion_nm, state.latest.torque_combustion_cycle_nm
            ),
        });
        rows.push(MetricRowViewModel {
            label: "Torque friction / pump",
            value: format!(
                "{:.2} / {:.2} Nm",
                state.latest.torque_friction_nm, state.latest.torque_pumping_nm
            ),
        });
        rows.push(MetricRowViewModel {
            label: "Shaft est / machine torque",
            value: format!(
                "{:+.2} / {:+.2} Nm",
                state.shaft_torque_estimate_nm(),
                state.latest.torque_load_nm
            ),
        });
        rows.push(MetricRowViewModel {
            label: "Brake power / load power",
            value: format!(
                "{:.2} / {:.2} kW",
                shaft_power_kw(state.latest.rpm, state.required_brake_torque_nm),
                shaft_power_kw(state.latest.rpm, state.latest.torque_load_nm)
            ),
        });
        Self { rows }
    }
}

#[derive(Debug, Clone)]
pub(super) struct ReadoutCardViewModel {
    pub(super) label: &'static str,
    pub(super) value: String,
    pub(super) unit: &'static str,
    pub(super) footer: String,
    pub(super) accent: egui::Color32,
}

#[derive(Debug, Clone)]
pub(super) struct GaugeCardViewModel {
    pub(super) label: &'static str,
    pub(super) value: f64,
    pub(super) min: f64,
    pub(super) max: f64,
    pub(super) unit: &'static str,
    pub(super) footer: &'static str,
    pub(super) accent: egui::Color32,
}

#[derive(Debug, Clone)]
pub(super) struct MeterCardViewModel {
    pub(super) label: &'static str,
    pub(super) value: f64,
    pub(super) min: f64,
    pub(super) max: f64,
    pub(super) value_text: String,
    pub(super) accent: egui::Color32,
}

#[derive(Debug, Clone)]
pub(super) struct OverviewViewModel {
    pub(super) readouts: Vec<ReadoutCardViewModel>,
    pub(super) gauges: Vec<GaugeCardViewModel>,
    pub(super) meters: Vec<MeterCardViewModel>,
}

impl OverviewViewModel {
    pub(super) fn build(
        state: &DashboardState,
        ui_config: &UiConfig,
        theme: DashboardTheme,
    ) -> Self {
        let calibration = RoughCalibrationViewModel::build(state);
        let brake_torque_out_nm = state.required_brake_torque_nm;
        let brake_power_kw = shaft_power_kw(state.latest.rpm, brake_torque_out_nm);
        let absorber_torque_nm = state.latest.torque_load_nm.abs();
        let absorber_power_kw = shaft_power_kw(state.latest.rpm, absorber_torque_nm);
        let intake_state = format!(
            "MAP {:.1} kPa / runner {:.1} kPa",
            state.latest.map_kpa, state.latest.intake_runner_kpa
        );
        let thermal_state = format!(
            "eta {:.1}% / Wi {:.3} kJ",
            state.latest.eta_thermal_indicated_pv * 100.0,
            state.latest.indicated_work_cycle_j * 1.0e-3
        );

        let readouts = vec![
            ReadoutCardViewModel {
                label: "STARTUP FIT",
                value: calibration.status_value.clone(),
                unit: "",
                footer: calibration.summary.clone(),
                accent: if calibration.ready {
                    theme.green
                } else {
                    theme.amber
                },
            },
            ReadoutCardViewModel {
                label: "ENGINE SPEED",
                value: format!("{:.0}", state.latest.rpm),
                unit: "rpm",
                footer: if calibration.active {
                    format!(
                        "target {} / {}",
                        calibration.target_text, calibration.phase_label
                    )
                } else if state.required_brake_torque_nm > 0.5 {
                    "fired operating point".to_owned()
                } else {
                    "motoring operating point".to_owned()
                },
                accent: theme.amber,
            },
            ReadoutCardViewModel {
                label: "BRAKE TORQUE",
                value: format!("{:.1}", brake_torque_out_nm),
                unit: "Nm",
                footer: format!("net {:.1} Nm", state.latest.torque_net_nm),
                accent: theme.red,
            },
            ReadoutCardViewModel {
                label: "BRAKE POWER FILT",
                value: format!("{:.1}", brake_power_kw),
                unit: "kW",
                footer: format!(
                    "{:.1} hp",
                    shaft_power_hp(state.latest.rpm, brake_torque_out_nm)
                ),
                accent: theme.cyan,
            },
            ReadoutCardViewModel {
                label: "ABSORBER TORQUE",
                value: format!("{:+.1}", state.latest.torque_load_nm),
                unit: "Nm",
                footer: format!("{:.1} kW", absorber_power_kw),
                accent: theme.red,
            },
            ReadoutCardViewModel {
                label: "AIR CHARGE",
                value: format!("{:.2}", state.latest.trapped_air_mg),
                unit: "mg/cyl",
                footer: format!("VE {:.1}%", state.latest.volumetric_efficiency * 100.0),
                accent: theme.green,
            },
            ReadoutCardViewModel {
                label: "INTAKE BUS",
                value: format!("{:.1}", state.latest.map_kpa),
                unit: "kPa",
                footer: intake_state,
                accent: theme.amber,
            },
            ReadoutCardViewModel {
                label: "DYNO LIMIT",
                value: format!("{:.0}", state.dyno_available_torque_nm()),
                unit: "Nm",
                footer: format!(
                    "{:.0} kW / {:.0} rpm",
                    state.sim.model.external_load.absorber_power_limit_kw,
                    state.sim.model.external_load.absorber_speed_limit_rpm
                ),
                accent: theme.green,
            },
            ReadoutCardViewModel {
                label: "INDIC EFF",
                value: format!("{:.1}", state.latest.eta_thermal_indicated_pv * 100.0),
                unit: "%",
                footer: thermal_state,
                accent: theme.cyan,
            },
        ];

        let gauges = vec![
            GaugeCardViewModel {
                label: "RPM",
                value: state.latest.rpm,
                min: 0.0,
                max: state.sim.params.max_rpm,
                unit: "rpm",
                footer: "crankshaft speed",
                accent: theme.amber,
            },
            GaugeCardViewModel {
                label: "MAP",
                value: state.latest.map_kpa,
                min: 20.0,
                max: 120.0,
                unit: "kPa",
                footer: "intake plenum",
                accent: theme.cyan,
            },
            GaugeCardViewModel {
                label: "LAMBDA",
                value: state.latest.lambda_target,
                min: 0.70,
                max: 1.20,
                unit: "-",
                footer: "target mixture",
                accent: theme.green,
            },
            GaugeCardViewModel {
                label: "BMEP",
                value: state.latest.brake_bmep_bar,
                min: 0.0,
                max: 16.0,
                unit: "bar",
                footer: "brake mean effective pressure",
                accent: theme.red,
            },
            GaugeCardViewModel {
                label: "EXH TEMP",
                value: state.latest.exhaust_temp_k,
                min: 300.0,
                max: 1_200.0,
                unit: "K",
                footer: "collector model",
                accent: theme.amber,
            },
            GaugeCardViewModel {
                label: "ABS TORQ",
                value: absorber_torque_nm,
                min: 0.0,
                max: state.sim.model.external_load.torque_max_nm.max(1.0),
                unit: "Nm",
                footer: "absorber applied torque",
                accent: theme.red,
            },
            GaugeCardViewModel {
                label: "INT EGR",
                value: state.latest.internal_egr_fraction * 100.0,
                min: 0.0,
                max: 30.0,
                unit: "%",
                footer: "internal residual fraction",
                accent: theme.green,
            },
        ];

        let meters = vec![
            MeterCardViewModel {
                label: "THROTTLE CMD",
                value: state.sim.control.throttle_cmd,
                min: 0.0,
                max: 1.0,
                value_text: format!("{:.3}", state.sim.control.throttle_cmd),
                accent: theme.amber,
            },
            MeterCardViewModel {
                label: "THROTTLE EFF",
                value: state.sim.state.throttle_eff,
                min: 0.0,
                max: 1.0,
                value_text: format!("{:.3}", state.sim.state.throttle_eff),
                accent: theme.cyan,
            },
            MeterCardViewModel {
                label: "TARGET RPM",
                value: state.load_target_rpm,
                min: 0.0,
                max: state.sim.params.max_rpm,
                value_text: if calibration.active {
                    format!(
                        "{:.0} rpm / fit {}",
                        state.load_target_rpm, calibration.phase_label
                    )
                } else {
                    format!(
                        "{:.0} rpm / err {:+.0}",
                        state.load_target_rpm,
                        state.rpm_error()
                    )
                },
                accent: theme.red,
            },
            MeterCardViewModel {
                label: "IGNITION",
                value: state.sim.control.ignition_timing_deg,
                min: ui_config.ignition_slider_min_deg,
                max: ui_config.ignition_slider_max_deg,
                value_text: format!("{:.1} deg BTDC", state.sim.control.ignition_timing_deg),
                accent: theme.green,
            },
            MeterCardViewModel {
                label: "VVT IN",
                value: state.sim.control.vvt_intake_deg,
                min: ui_config.vvt_slider_min_deg,
                max: ui_config.vvt_slider_max_deg,
                value_text: format!("{:.1} deg", state.sim.control.vvt_intake_deg),
                accent: theme.cyan,
            },
            MeterCardViewModel {
                label: "VVT EX",
                value: state.sim.control.vvt_exhaust_deg,
                min: ui_config.vvt_slider_min_deg,
                max: ui_config.vvt_slider_max_deg,
                value_text: format!("{:.1} deg", state.sim.control.vvt_exhaust_deg),
                accent: theme.red,
            },
            MeterCardViewModel {
                label: "BRAKE POWER",
                value: brake_power_kw,
                min: 0.0,
                max: 160.0,
                value_text: format!("{:.1} kW", brake_power_kw),
                accent: theme.cyan,
            },
        ];

        Self {
            readouts,
            gauges,
            meters,
        }
    }
}

#[derive(Debug, Clone)]
pub(super) struct PlotSeriesViewModel {
    pub(super) name: Option<String>,
    pub(super) color: egui::Color32,
    pub(super) points: Vec<[f64; 2]>,
}

#[derive(Debug, Clone)]
pub(super) struct PressurePlotsViewModel {
    pub(super) pv_points: Vec<[f64; 2]>,
    pub(super) pv_y_max_plot: f64,
    pub(super) ptheta_curves: Vec<PlotSeriesViewModel>,
    pub(super) ptheta_y_max_plot: f64,
}

impl PressurePlotsViewModel {
    pub(super) fn build(
        state: &DashboardState,
        available_width: f32,
        ui_config: &UiConfig,
    ) -> Self {
        let pv_points: Vec<[f64; 2]> = state
            .latest
            .pv_points
            .iter()
            .map(|(v, p)| [*v, *p * 1e-3])
            .collect();
        let pv_y_peak_kpa = state
            .latest
            .pv_points
            .iter()
            .map(|(_, p)| *p * 1e-3)
            .fold(state.sim.plot.pv_y_min_kpa, f64::max);
        let pv_y_max_plot = state.sim.plot.pv_y_max_kpa.max(
            (pv_y_peak_kpa * ui_config.pv_headroom_ratio)
                .max(state.sim.plot.pv_y_min_kpa + ui_config.pv_min_headroom_kpa),
        );

        let plot_width = ((available_width - 8.0).max(320.0)) * 0.5;
        let sample_count = ptheta_display_sample_count(plot_width, state.sim.model.pv_display_bins);
        let curves = state.sim.build_ptheta_display_curves(sample_count);
        let ptheta_y_peak_kpa = curves
            .iter()
            .flat_map(|curve| curve.iter().map(|(_, p)| *p * 1e-3))
            .fold(state.sim.plot.pv_y_min_kpa, f64::max);
        let ptheta_y_max_plot = state.sim.plot.pv_y_max_kpa.max(
            (ptheta_y_peak_kpa * ui_config.pv_headroom_ratio)
                .max(state.sim.plot.pv_y_min_kpa + ui_config.pv_min_headroom_kpa),
        );
        let ptheta_curves = curves
            .into_iter()
            .enumerate()
            .filter(|(_, curve)| !curve.is_empty())
            .map(|(idx, curve)| PlotSeriesViewModel {
                name: Some(format!("Cyl {}", idx + 1)),
                color: cylinder_trace_color(idx),
                points: curve
                    .into_iter()
                    .map(|(theta, p)| [theta, p * 1e-3])
                    .collect(),
            })
            .collect();

        Self {
            pv_points,
            pv_y_max_plot,
            ptheta_curves,
            ptheta_y_max_plot,
        }
    }
}

#[derive(Debug, Clone)]
pub(super) struct CycleMonitorsViewModel {
    pub(super) rpm_lines: Vec<PlotSeriesViewModel>,
    pub(super) rpm_y_min: f64,
    pub(super) rpm_y_max: f64,
    pub(super) torque_lines: Vec<PlotSeriesViewModel>,
    pub(super) load_lines: Vec<PlotSeriesViewModel>,
    pub(super) torque_y_min: f64,
    pub(super) torque_y_max: f64,
    pub(super) trapped_air_lines: Vec<PlotSeriesViewModel>,
    pub(super) trapped_air_y_min: f64,
    pub(super) trapped_air_y_max: f64,
    pub(super) intake_cam_points: Vec<[f64; 2]>,
    pub(super) exhaust_cam_points: Vec<[f64; 2]>,
    pub(super) cam_y_max: f64,
    pub(super) cycle_deg: f64,
}

impl CycleMonitorsViewModel {
    pub(super) fn build(state: &mut DashboardState, ui_config: &UiConfig) -> Self {
        let recent_cycles = state.sim.plot.history_recent_cycles.max(1);

        let rpm_lines = build_history_series(
            recent_cycle_plot_lines(&state.sim.history_rpm, recent_cycles),
            egui::Color32::LIGHT_GREEN,
            Some("RPM"),
        );
        let rpm_min_span = (state.sim.params.max_rpm * 0.04).max(150.0);
        let (rpm_y_min, rpm_y_max) = adaptive_plot_y_range(
            recent_cycle_min_max(&state.sim.history_rpm, recent_cycles),
            rpm_min_span,
            ui_config.torque_margin_ratio,
            Some(0.0),
        );

        let torque_lines_raw =
            recent_cycle_plot_lines(&state.sim.history_torque_net_nm, recent_cycles);
        let load_lines_raw =
            recent_cycle_plot_lines(&state.sim.history_torque_load_nm, recent_cycles);
        let (torque_y_min, torque_y_max) = adaptive_plot_y_range(
            combine_min_max(
                recent_cycle_min_max(&state.sim.history_torque_net_nm, recent_cycles),
                recent_cycle_min_max(&state.sim.history_torque_load_nm, recent_cycles),
            ),
            ui_config.torque_min_span_nm,
            ui_config.torque_margin_ratio,
            None,
        );
        let torque_lines = build_history_series(
            torque_lines_raw,
            egui::Color32::from_rgb(255, 120, 120),
            Some("Filtered net torque [Nm]"),
        );
        let load_lines = build_history_series(
            load_lines_raw,
            egui::Color32::from_rgb(255, 185, 70),
            Some("External load [Nm]"),
        );

        let trapped_air_lines_raw =
            recent_cycle_plot_lines(&state.sim.history_trapped_air_mg, recent_cycles);
        let trapped_air_min_span = (ui_config.trapped_air_min_y_max_mg * 0.25).max(20.0);
        let (trapped_air_y_min, trapped_air_y_max) = adaptive_plot_y_range(
            recent_cycle_min_max(&state.sim.history_trapped_air_mg, recent_cycles),
            trapped_air_min_span,
            ui_config.torque_margin_ratio,
            Some(0.0),
        );
        let trapped_air_lines = build_history_series(
            trapped_air_lines_raw,
            egui::Color32::from_rgb(120, 200, 255),
            Some("Trapped air inst [mg/cyl]"),
        );

        let (intake_cam_points, exhaust_cam_points) = {
            let (intake, exhaust) = state.cached_cam_profile_points();
            (intake.to_vec(), exhaust.to_vec())
        };

        Self {
            rpm_lines,
            rpm_y_min,
            rpm_y_max,
            torque_lines,
            load_lines,
            torque_y_min,
            torque_y_max,
            trapped_air_lines,
            trapped_air_y_min,
            trapped_air_y_max,
            intake_cam_points,
            exhaust_cam_points,
            cam_y_max: state.sim.cam.display_y_max_mm,
            cycle_deg: state.latest.cycle_deg,
        }
    }
}

#[derive(Debug, Clone)]
pub(super) struct SchematicViewModel {
    pub(super) local_cycle_deg: f64,
    pub(super) actual_cycle_deg: f64,
    pub(super) current_stroke_label: &'static str,
    pub(super) bore_m: f64,
    pub(super) stroke_m: f64,
    pub(super) intake_max_lift_mm: f64,
    pub(super) exhaust_max_lift_mm: f64,
    pub(super) intake_lift_mm: f64,
    pub(super) exhaust_lift_mm: f64,
    pub(super) intake_open_deg: f64,
    pub(super) intake_close_deg: f64,
    pub(super) exhaust_open_deg: f64,
    pub(super) exhaust_close_deg: f64,
    pub(super) spark_event_deg: f64,
    pub(super) burn_start_deg: f64,
    pub(super) flame_radius_norm: f32,
    pub(super) flame_speed_mps: f32,
    pub(super) ignition_flash: bool,
    pub(super) burn_active: bool,
    pub(super) fuel_cmd: bool,
    pub(super) intake_vvt_deg: f64,
    pub(super) exhaust_vvt_deg: f64,
}

impl SchematicViewModel {
    pub(super) fn build(state: &DashboardState) -> Self {
        let local_cycle_deg = state.schematic_cycle_deg;
        let intake_center_deg =
            state.sim.cam.intake_centerline_deg - state.sim.control.vvt_intake_deg;
        let exhaust_center_deg =
            state.sim.cam.exhaust_centerline_deg + state.sim.control.vvt_exhaust_deg;
        let intake_lift_mm = cam_lift_mm(
            local_cycle_deg,
            intake_center_deg,
            state.sim.cam.intake_duration_deg,
            state.sim.cam.intake_max_lift_mm,
            &state.sim.model,
        )
        .max(0.0);
        let exhaust_lift_mm = cam_lift_mm(
            local_cycle_deg,
            exhaust_center_deg,
            state.sim.cam.exhaust_duration_deg,
            state.sim.cam.exhaust_max_lift_mm,
            &state.sim.model,
        )
        .max(0.0);
        let intake_open_deg =
            wrap_cycle_deg(intake_center_deg - 0.5 * state.sim.cam.intake_duration_deg);
        let intake_close_deg =
            wrap_cycle_deg(intake_center_deg + 0.5 * state.sim.cam.intake_duration_deg);
        let exhaust_open_deg =
            wrap_cycle_deg(exhaust_center_deg - 0.5 * state.sim.cam.exhaust_duration_deg);
        let exhaust_close_deg =
            wrap_cycle_deg(exhaust_center_deg + 0.5 * state.sim.cam.exhaust_duration_deg);

        let spark_event_deg = finite_f64(
            (360.0 - state.latest.ignition_timing_deg).rem_euclid(720.0),
            360.0,
        );
        let spark_rel_deg =
            finite_f64((local_cycle_deg - spark_event_deg).rem_euclid(720.0), 720.0);
        let burn_duration_deg = finite_f64(state.latest.burn_duration_deg, 1.0).max(1.0);
        let burn_start_deg = finite_f64(state.latest.burn_start_deg.rem_euclid(720.0), 360.0);
        let burn_rel_deg = finite_f64((local_cycle_deg - burn_start_deg).rem_euclid(720.0), 720.0);
        let ignition_flash =
            state.sim.control.spark_cmd && state.sim.control.fuel_cmd && spark_rel_deg <= 12.0;
        let burn_active = state.sim.control.spark_cmd
            && state.sim.control.fuel_cmd
            && burn_rel_deg <= burn_duration_deg + 18.0;
        let crank_deg_per_s = finite_f32((state.latest.rpm.max(1.0) * 6.0) as f32, 6.0).max(1.0);
        let burn_elapsed_s = finite_f32(burn_rel_deg as f32 / crank_deg_per_s, 0.0).max(0.0);
        let burn_total_s =
            finite_f32(burn_duration_deg as f32 / crank_deg_per_s, 1.0e-4).max(1.0e-4);
        let characteristic_travel_m =
            finite_f32((0.48 * state.sim.params.bore_m.max(1.0e-4)) as f32, 0.048).max(1.0e-4);
        let combustion_rate_norm =
            finite_f64(state.latest.combustion_rate_norm, 0.0).clamp(0.0, 1.4) as f32;
        let combustion_rate_gain = (0.60 + 0.40 * combustion_rate_norm).clamp(0.55, 1.20);
        let flame_speed_mps = finite_f32(
            (characteristic_travel_m / burn_total_s).max(0.1) * combustion_rate_gain,
            0.0,
        )
        .max(0.0);
        let flame_radius_norm = if burn_active {
            finite_f32(
                burn_elapsed_s * flame_speed_mps / characteristic_travel_m,
                0.0,
            )
            .clamp(0.0, 1.0)
        } else {
            0.0
        };

        Self {
            local_cycle_deg,
            actual_cycle_deg: state.latest.cycle_deg,
            current_stroke_label: current_stroke_label(local_cycle_deg),
            bore_m: state.sim.params.bore_m,
            stroke_m: state.sim.params.stroke_m,
            intake_max_lift_mm: state.sim.cam.intake_max_lift_mm,
            exhaust_max_lift_mm: state.sim.cam.exhaust_max_lift_mm,
            intake_lift_mm,
            exhaust_lift_mm,
            intake_open_deg,
            intake_close_deg,
            exhaust_open_deg,
            exhaust_close_deg,
            spark_event_deg,
            burn_start_deg,
            flame_radius_norm,
            flame_speed_mps,
            ignition_flash,
            burn_active,
            fuel_cmd: state.sim.control.fuel_cmd,
            intake_vvt_deg: state.sim.control.vvt_intake_deg,
            exhaust_vvt_deg: state.sim.control.vvt_exhaust_deg,
        }
    }
}

fn current_stroke_label(cycle_deg: f64) -> &'static str {
    match wrap_cycle_deg(cycle_deg) {
        deg if deg < 180.0 => "intake stroke",
        deg if deg < 360.0 => "compression stroke",
        deg if deg < 540.0 => "expansion stroke",
        _ => "exhaust stroke",
    }
}

fn finite_f32(value: f32, fallback: f32) -> f32 {
    if value.is_finite() { value } else { fallback }
}

fn finite_f64(value: f64, fallback: f64) -> f64 {
    if value.is_finite() { value } else { fallback }
}

fn ptheta_display_sample_count(column_width_px: f32, pv_display_bins: usize) -> usize {
    let max_samples = (pv_display_bins / FIXED_CYLINDER_COUNT).max(180);
    let width_based = (column_width_px.max(240.0) * 0.72).round() as usize;
    width_based.clamp(180, max_samples)
}

fn build_history_series(
    lines: Vec<Vec<[f64; 2]>>,
    base_color: egui::Color32,
    label_last: Option<&str>,
) -> Vec<PlotSeriesViewModel> {
    let total = lines.len();
    lines
        .into_iter()
        .enumerate()
        .map(|(idx, points)| PlotSeriesViewModel {
            name: (idx + 1 == total)
                .then(|| label_last.unwrap_or_default().to_owned())
                .filter(|name| !name.is_empty()),
            color: faded_cycle_color(base_color, idx, total),
            points,
        })
        .collect()
}

fn recent_cycle_plot_lines(
    history: &VecDeque<CycleHistorySample>,
    recent_cycles: usize,
) -> Vec<Vec<[f64; 2]>> {
    let Some(latest_cycle) = history.back().map(|sample| sample.cycle) else {
        return Vec::new();
    };
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
    history: &VecDeque<CycleHistorySample>,
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

fn cylinder_trace_color(index: usize) -> egui::Color32 {
    const COLORS: [egui::Color32; 4] = [
        egui::Color32::from_rgb(255, 170, 40),
        egui::Color32::from_rgb(90, 180, 255),
        egui::Color32::from_rgb(110, 220, 150),
        egui::Color32::from_rgb(255, 105, 125),
    ];
    COLORS[index % COLORS.len()]
}

#[cfg(test)]
mod tests {
    use std::collections::VecDeque;

    use super::{
        adaptive_plot_y_range, graph_layout_heights, ptheta_display_sample_count,
        recent_cycle_plot_lines,
    };
    use crate::config::AppConfig;
    use crate::simulator::CycleHistorySample;

    #[test]
    fn graph_layout_uses_positive_sizes() {
        let cfg = AppConfig::default();
        let layout = graph_layout_heights(&cfg.ui);

        assert!(layout.standard_plot_px >= 72.0);
        assert!(layout.pv_plot_px > layout.standard_plot_px);
        assert!(layout.section_spacing_px > 0.0);
    }

    #[test]
    fn adaptive_plot_y_range_tracks_positive_operating_window() {
        let (y_min, y_max) =
            adaptive_plot_y_range(Some((1_990.0, 2_010.0)), 150.0, 0.15, Some(0.0));

        assert!(y_min > 1_800.0);
        assert!(y_max < 2_200.0);
    }

    #[test]
    fn adaptive_plot_y_range_keeps_signed_trace_centered() {
        let (y_min, y_max) = adaptive_plot_y_range(Some((145.0, 150.0)), 10.0, 0.15, None);

        assert!(y_min > 130.0);
        assert!(y_max < 170.0);
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

    #[test]
    fn ptheta_sample_count_tracks_plot_width_without_exceeding_model_limit() {
        let low = ptheta_display_sample_count(260.0, 1_440);
        let high = ptheta_display_sample_count(900.0, 1_440);

        assert!(low >= 180);
        assert!(high <= 360);
        assert!(high >= low);
    }
}
