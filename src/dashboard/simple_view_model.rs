use super::startup_fit::{STARTUP_FIT_MAX_WALL_TIME_S, StartupFitPhase, StartupFitStatus};
use super::state::DashboardState;
use crate::config::UiConfig;

pub(super) struct StatusRow {
    pub(super) label: &'static str,
    pub(super) value: String,
}

pub(super) struct SimpleDashboardViewModel {
    pub(super) fit_active: bool,
    pub(super) fit_phase: StartupFitPhase,
    pub(super) fit_status_value: &'static str,
    pub(super) header_fit_text: String,
    pub(super) header_rpm_text: String,
    pub(super) header_target_text: String,
    pub(super) header_error_text: String,
    pub(super) fit_detail: &'static str,
    pub(super) fit_progress_fraction: f32,
    pub(super) fit_progress_label: String,
    pub(super) fit_rows: [StatusRow; 10],
    pub(super) manual_control_hint: Option<&'static str>,
    pub(super) external_load_note: &'static str,
    pub(super) live_status_message: &'static str,
    pub(super) live_rows: [StatusRow; 10],
}

impl SimpleDashboardViewModel {
    pub(super) fn from_state(state: &DashboardState, ui_config: &UiConfig) -> Self {
        let fit = state.startup_fit_status();
        let fit_status_value = fit_status_value(fit.phase);
        let fit_progress_label = fit_progress_label(&fit);
        let fit_phase_label = fit.phase.label();
        let fit_progress_fraction = fit_progress_fraction(&fit);
        let header_fit_text = if fit.active {
            format!("Startup fit: {} / {}", fit_status_value, fit_progress_label)
        } else if fit.timed_out {
            format!(
                "Startup fit: READY / {:.1} s cap reached, best candidate applied",
                STARTUP_FIT_MAX_WALL_TIME_S
            )
        } else {
            format!("Startup fit: {} / manual control ready", fit_status_value)
        };
        let live_status_message = if fit.active {
            "Discrete startup-fit search is evaluating throttle bins, local MBT spark, and required brake torque until the selected point holds the target speed."
        } else if fit.timed_out {
            "The startup fit hit its 30 s wall-clock limit, applied the best candidate found so far, and unlocked manual trim."
        } else {
            "The fitted operating point is holding with the selected brake load. You can now trim throttle, ignition, and VVT manually."
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
                    label: "Target / wall / sim",
                    value: format!(
                        "{:.0} rpm / {:.1}/{:.1} / {:.1} s",
                        fit.target_rpm,
                        fit.wall_elapsed_s,
                        STARTUP_FIT_MAX_WALL_TIME_S,
                        fit.simulated_elapsed_s
                    ),
                },
                StatusRow {
                    label: "Average rpm",
                    value: format!("{:.0} rpm", fit.avg_rpm),
                },
                StatusRow {
                    label: "Req brake / best",
                    value: format!(
                        "{:+.1} / {:+.1} Nm",
                        fit.required_brake_torque_nm, fit.best_required_brake_torque_nm
                    ),
                },
                StatusRow {
                    label: "Net / periodic",
                    value: format!(
                        "{:+.1} Nm / {:.3}",
                        fit.avg_net_torque_nm, fit.periodic_error_norm
                    ),
                },
                StatusRow {
                    label: "Search",
                    value: fit.candidate_label.clone(),
                },
                StatusRow {
                    label: "Throttle / spark",
                    value: format!(
                        "{:.3} / {:.1} deg BTDC",
                        fit.throttle_cmd, fit.ignition_timing_deg
                    ),
                },
                StatusRow {
                    label: "Load cmd / margin",
                    value: format!("{:+.3} / {:.1} Nm", fit.load_cmd, fit.torque_margin_to_best_nm),
                },
                StatusRow {
                    label: "Fit summary",
                    value: format!(
                        "{} / avg {:.0} rpm / brake {:+.1} Nm",
                        fit_phase_label, fit.avg_rpm, fit.required_brake_torque_nm
                    ),
                },
                StatusRow {
                    label: "VVT fixed",
                    value: format!("{:+.1} / {:+.1} deg", fit.vvt_intake_deg, fit.vvt_exhaust_deg),
                },
            ],
            manual_control_hint: fit.active.then_some(
                "Startup fit is running. Manual actuator edits stay locked until the MBT search and finite-cycle verification finish, or the 30 s cap is reached.",
            ),
            external_load_note:
                "Required brake torque is solved during startup fit and applied as the bench load path; indicated torque remains a separate p-V diagnostic.",
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
                    label: "Required brake torque",
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
                    label: "Pumping / brake req est",
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

fn fit_status_value(phase: StartupFitPhase) -> &'static str {
    match phase {
        StartupFitPhase::Priming => "PRIME",
        StartupFitPhase::Optimizing => "OPTIMIZE",
        StartupFitPhase::Verifying => "VERIFY",
        StartupFitPhase::Ready => "READY",
    }
}

fn fit_progress_fraction(status: &StartupFitStatus) -> f32 {
    let iteration_fraction =
        (status.iteration as f32 / status.max_iterations.max(1) as f32).clamp(0.0, 1.0);
    let stable_fraction = (status.stable_windows as f32
        / status.required_stable_windows.max(1) as f32)
        .clamp(0.0, 1.0);
    let wall_fraction =
        (status.wall_elapsed_s / STARTUP_FIT_MAX_WALL_TIME_S).clamp(0.0, 1.0) as f32;

    let phase_fraction = match status.phase {
        StartupFitPhase::Priming => 0.12,
        StartupFitPhase::Optimizing => 0.18 + 0.62 * iteration_fraction,
        StartupFitPhase::Verifying => 0.82 + 0.16 * stable_fraction,
        StartupFitPhase::Ready => 1.0,
    };

    phase_fraction.max(wall_fraction).clamp(0.0, 1.0)
}

fn fit_progress_label(status: &StartupFitStatus) -> String {
    match status.phase {
        StartupFitPhase::Priming => {
            format!(
                "priming / {:.1} of {:.1} s wall-clock",
                status.wall_elapsed_s, STARTUP_FIT_MAX_WALL_TIME_S
            )
        }
        StartupFitPhase::Optimizing => format!(
            "iter {}/{} / {} / brake {:+.1} / {:.1}s",
            status.iteration,
            status.max_iterations,
            status.candidate_label,
            status.required_brake_torque_nm,
            status.wall_elapsed_s
        ),
        StartupFitPhase::Verifying => format!(
            "stable {}/{} / best brake {:+.1} / {:.1}s",
            status.stable_windows,
            status.required_stable_windows,
            status.best_required_brake_torque_nm,
            status.wall_elapsed_s
        ),
        StartupFitPhase::Ready => {
            if status.timed_out {
                format!(
                    "{:.1} s cap reached / best candidate unlocked",
                    STARTUP_FIT_MAX_WALL_TIME_S
                )
            } else {
                "manual controls unlocked".to_owned()
            }
        }
    }
}
