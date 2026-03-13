use super::startup_fit::{RoughCalibrationPhase, RoughCalibrationStatus};
use super::state::DashboardState;
use crate::config::UiConfig;

pub(super) struct StatusRow {
    pub(super) label: &'static str,
    pub(super) value: String,
}

pub(super) struct SimpleDashboardViewModel {
    pub(super) fit_active: bool,
    pub(super) fit_phase: RoughCalibrationPhase,
    pub(super) fit_status_value: &'static str,
    pub(super) header_fit_text: String,
    pub(super) header_rpm_text: String,
    pub(super) header_target_text: String,
    pub(super) header_error_text: String,
    pub(super) fit_detail: &'static str,
    pub(super) fit_progress_fraction: f32,
    pub(super) fit_progress_label: String,
    pub(super) fit_rows: [StatusRow; 6],
    pub(super) manual_control_hint: Option<&'static str>,
    pub(super) external_load_note: &'static str,
    pub(super) live_status_message: &'static str,
    pub(super) live_rows: [StatusRow; 10],
}

impl SimpleDashboardViewModel {
    pub(super) fn from_state(state: &DashboardState, ui_config: &UiConfig) -> Self {
        let fit = state.rough_calibration_status();
        let fit_status_value = fit_status_value(fit.phase);
        let fit_progress_label = fit_progress_label(fit);
        let fit_phase_label = fit.phase.label();
        let fit_progress_fraction = fit_progress_fraction(fit);
        let header_fit_text = if fit.active {
            format!("Startup fit: {} / {}", fit_status_value, fit_progress_label)
        } else {
            format!("Startup fit: {} / manual control ready", fit_status_value)
        };
        let live_status_message = if fit.active {
            "Calculation is trimming throttle and spark until the engine holds near 2000 rpm without external assist."
        } else {
            "Coarse operating point is holding on its own. You can now trim throttle and timing manually."
        };

        Self {
            fit_active: fit.active,
            fit_phase: fit.phase,
            fit_status_value,
            header_fit_text,
            header_rpm_text: format!("RPM {:.0}", state.latest.rpm),
            header_target_text: format!("Target {:.0}", state.load_target_rpm),
            header_error_text: format!("RPM err {:+.0}", state.rpm_error()),
            fit_detail: fit.phase.detail(),
            fit_progress_fraction,
            fit_progress_label: fit_progress_label.clone(),
            fit_rows: [
                StatusRow {
                    label: "Phase",
                    value: fit_status_value.to_owned(),
                },
                StatusRow {
                    label: "Target / elapsed",
                    value: format!("{:.0} rpm / {:.2} s", fit.target_rpm, fit.simulated_elapsed_s),
                },
                StatusRow {
                    label: "Average rpm",
                    value: format!("{:.0} rpm", fit.avg_rpm),
                },
                StatusRow {
                    label: "Average net torque",
                    value: format!("{:+.1} Nm", fit.avg_net_torque_nm),
                },
                StatusRow {
                    label: "Throttle / ignition",
                    value: format!("{:.3} / {:.1} deg BTDC", fit.throttle_cmd, fit.ignition_timing_deg),
                },
                StatusRow {
                    label: "Fit summary",
                    value: format!(
                        "{} / avg {:.0} rpm / net {:+.1} Nm",
                        fit_phase_label, fit.avg_rpm, fit.avg_net_torque_nm
                    ),
                },
            ],
            manual_control_hint: fit.active.then_some(
                "Coarse fit is running. Manual throttle and spark settings stay locked until the engine holds near 2000 rpm on its own.",
            ),
            external_load_note: "External load is fixed at zero in this simplified view.",
            live_status_message,
            live_rows: [
                StatusRow {
                    label: "Engine speed",
                    value: format!("{:.0} rpm", state.latest.rpm),
                },
                StatusRow {
                    label: "Target speed",
                    value: format!("{:.0} rpm", state.load_target_rpm),
                },
                StatusRow {
                    label: "Net torque",
                    value: format!("{:+.1} Nm", state.latest.torque_net_nm),
                },
                StatusRow {
                    label: "Net shaft power",
                    value: format!(
                        "{:+.1} kW / {:+.1} hp",
                        state.net_shaft_power_kw(),
                        state.net_shaft_power_hp()
                    ),
                },
                StatusRow {
                    label: "Load torque",
                    value: format!("{:+.1} Nm", state.latest.torque_load_nm),
                },
                StatusRow {
                    label: "Throttle cmd / eff",
                    value: format!(
                        "{:.3} / {:.3}",
                        state.sim.control.throttle_cmd, state.sim.state.throttle_eff
                    ),
                },
                StatusRow {
                    label: "Ignition",
                    value: format!("{:.1} deg BTDC", state.sim.control.ignition_timing_deg),
                },
                StatusRow {
                    label: "Combustion / friction",
                    value: format!(
                        "{:+.1} / {:+.1} Nm",
                        state.latest.torque_combustion_cycle_nm, state.latest.torque_friction_nm
                    ),
                },
                StatusRow {
                    label: "Pumping / shaft est",
                    value: format!(
                        "{:+.1} / {:+.1} Nm",
                        state.latest.torque_pumping_nm,
                        state.shaft_torque_estimate_nm()
                    ),
                },
                StatusRow {
                    label: "Run state",
                    value: format!(
                        "{} / {} / MAP {:.1} kPa / lambda {:.3}",
                        state.firing_state_label(),
                        state.solver_mode_label(ui_config),
                        state.latest.map_kpa,
                        state.latest.lambda_target
                    ),
                },
            ],
        }
    }
}

fn fit_status_value(phase: RoughCalibrationPhase) -> &'static str {
    match phase {
        RoughCalibrationPhase::Priming => "PRIME",
        RoughCalibrationPhase::Searching => "SEARCH",
        RoughCalibrationPhase::Settling => "SETTLE",
        RoughCalibrationPhase::Ready => "READY",
    }
}

fn fit_progress_fraction(status: RoughCalibrationStatus) -> f32 {
    let iteration_fraction =
        (status.iteration as f32 / status.max_iterations.max(1) as f32).clamp(0.0, 1.0);
    let stable_fraction = (status.stable_windows as f32
        / status.required_stable_windows.max(1) as f32)
        .clamp(0.0, 1.0);

    match status.phase {
        RoughCalibrationPhase::Priming => 0.14,
        RoughCalibrationPhase::Searching => 0.22 + 0.48 * iteration_fraction,
        RoughCalibrationPhase::Settling => 0.74 + 0.20 * stable_fraction,
        RoughCalibrationPhase::Ready => 1.0,
    }
    .clamp(0.0, 1.0)
}

fn fit_progress_label(status: RoughCalibrationStatus) -> String {
    match status.phase {
        RoughCalibrationPhase::Priming => {
            format!(
                "settling fired initial condition / {:.2} s simulated",
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
    }
}
