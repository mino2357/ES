use super::startup_fit::{
    StartupFitPhase, StartupFitStatus, startup_fit_wall_limit_label, startup_fit_wall_limit_s,
};
use super::state::DashboardState;
use crate::config::UiConfig;

pub(super) struct StatusRow {
    pub(super) label: &'static str,
    pub(super) value: String,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) struct OperatingPointCell {
    pub(super) row: usize,
    pub(super) col: usize,
}

pub(super) struct OperatingPointMarkerViewModel {
    pub(super) cell: OperatingPointCell,
}

#[derive(Debug, Clone, Copy, Default)]
pub(super) struct OperatingPointCellHeatViewModel {
    pub(super) coarse_hits: u16,
    pub(super) refine_hits: u16,
}

pub(super) struct OperatingPointTableViewModel {
    pub(super) fit_marker: OperatingPointMarkerViewModel,
    pub(super) result_marker: Option<OperatingPointMarkerViewModel>,
    pub(super) live_marker: OperatingPointMarkerViewModel,
    pub(super) cell_heat: Vec<OperatingPointCellHeatViewModel>,
    pub(super) max_heat_hits: u16,
    pub(super) fit_summary: String,
    pub(super) result_summary: Option<String>,
    pub(super) live_summary: String,
    pub(super) note: &'static str,
}

pub(super) const OPERATING_POINT_SPEED_LABELS: [&str; 16] = [
    "<0.5k", "0.5-1.0", "1.0-1.5", "1.5-2.0", "2.0-2.5", "2.5-3.0", "3.0-3.5", "3.5-4.0",
    "4.0-4.5", "4.5-5.0", "5.0-5.5", "5.5-6.0", "6.0-6.5", "6.5-7.0", "7.0-7.5", ">=7.5k",
];
const OPERATING_POINT_SPEED_UPPER_BOUNDS_RPM: [f64; 15] = [
    500.0, 1000.0, 1500.0, 2000.0, 2500.0, 3000.0, 3500.0, 4000.0, 4500.0, 5000.0, 5500.0, 6000.0,
    6500.0, 7000.0, 7500.0,
];

pub(super) const OPERATING_POINT_BRAKE_LABELS: [&str; 16] = [
    "<0", "0-5", "5-10", "10-15", "15-20", "20-25", "25-30", "30-35", "35-40", "40-45", "45-50",
    "50-55", "55-60", "60-65", "65-70", ">=70",
];
const OPERATING_POINT_BRAKE_UPPER_BOUNDS_NM: [f64; 15] = [
    0.0, 5.0, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0, 50.0, 55.0, 60.0, 65.0, 70.0,
];

pub(super) struct SimpleDashboardViewModel {
    pub(super) fit_active: bool,
    pub(super) fit_phase: StartupFitPhase,
    pub(super) fit_status_value: &'static str,
    pub(super) header_fit_text: String,
    pub(super) header_rpm_text: String,
    pub(super) header_target_text: String,
    pub(super) header_error_text: String,
    pub(super) fit_detail: String,
    pub(super) fit_progress_fraction: f32,
    pub(super) fit_progress_label: String,
    pub(super) fit_rows: [StatusRow; 10],
    pub(super) manual_control_hint: Option<String>,
    pub(super) external_load_note: String,
    pub(super) live_status_message: String,
    pub(super) live_rows: [StatusRow; 10],
    pub(super) operating_point_table: OperatingPointTableViewModel,
}

impl SimpleDashboardViewModel {
    pub(super) fn from_state(state: &DashboardState, ui_config: &UiConfig) -> Self {
        let fit = state.startup_fit_status();
        let wall_limit_label = startup_fit_wall_limit_label();
        let wall_limit_s = startup_fit_wall_limit_s();
        let fit_status_value = fit_status_value(fit.phase);
        let fit_progress_label = fit_progress_label(&fit);
        let fit_phase_label = fit.phase.label();
        let fit_progress_fraction = fit_progress_fraction(&fit);
        let runtime_mode_label = state.post_fit_mode.label();
        let header_fit_text = if fit.active {
            format!("Startup fit: {} / {}", fit_status_value, fit_progress_label)
        } else if fit.loaded_from_cache {
            format!(
                "Startup fit: READY / cache hit / {} live",
                runtime_mode_label
            )
        } else if fit.timed_out {
            format!(
                "Startup fit: READY / {} cap reached, best candidate applied",
                wall_limit_label
            )
        } else {
            format!(
                "Startup fit: {} / {} live",
                fit_status_value, runtime_mode_label
            )
        };
        let live_status_message = if fit.active {
            "Discrete startup-fit search is evaluating throttle bins, local MBT spark, and required brake torque until the selected point holds the target speed."
                .to_owned()
        } else if fit.loaded_from_cache {
            "Cached startup-fit map loaded from disk: the heavy startup-fit search was skipped, and post-fit runtime control is live immediately."
                .to_owned()
        } else if fit.timed_out {
            format!(
                "The startup fit hit its {} wall-clock limit, applied the best candidate found so far, and switched into post-fit runtime control.",
                wall_limit_label
            )
        } else if state.post_fit_mode == super::state::PostFitRuntimeMode::StandardRuntime {
            "Standard runtime is live: driver demand sets torque request, and throttle, ignition, and VVT follow the fitted baseline in real time."
                .to_owned()
        } else {
            "Actuator lab is live: driver demand still sets torque request, while throttle, ignition, and VVT are manual overrides against the auto baseline."
                .to_owned()
        };

        Self {
            fit_active: fit.active,
            fit_phase: fit.phase,
            fit_status_value,
            header_fit_text,
            header_rpm_text: format!("RPM {:.0}", state.latest.rpm),
            header_target_text: format!("Target {:.0}", state.load_target_rpm),
            header_error_text: format!("RPM err {:+.0}", state.rpm_error()),
            fit_detail: if fit.loaded_from_cache {
                "startup-fit artifact was restored from a saved cache, so standard runtime and actuator-lab features start from the previously fitted map".to_owned()
            } else {
                fit.phase.detail().to_owned()
            },
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
                        "{:.0} rpm / {:.1}/{} / {:.1} s",
                        fit.target_rpm,
                        fit.wall_elapsed_s,
                        wall_limit_label,
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
            manual_control_hint: fit.active.then_some(match wall_limit_s {
                Some(_) => format!(
                    "Startup fit is running. Manual actuator edits stay locked until the MBT search and finite-cycle verification finish, or the {} cap is reached.",
                    wall_limit_label
                ),
                None => "Startup fit is running. Manual actuator edits stay locked until the MBT search and finite-cycle verification finish.".to_owned(),
            }),
            external_load_note: if state.post_fit_mode
                == super::state::PostFitRuntimeMode::StandardRuntime
            {
                "This dashboard uses a brake-dyno bench load for startup fit and RPM hold; driver demand sets torque request while throttle, ignition, and VVT follow the auto baseline."
                    .to_owned()
            } else {
                "This dashboard uses a brake-dyno bench load for startup fit and RPM hold; actuator lab keeps the same torque-request path but exposes manual overrides against the auto baseline."
                    .to_owned()
            },
            live_status_message,
            live_rows: [
                StatusRow {
                    label: "Runtime mode",
                    value: format!(
                        "{} / demand {:.0}%",
                        runtime_mode_label,
                        state.driver_demand * 100.0
                    ),
                },
                StatusRow {
                    label: "Engine speed",
                    value: format!("{:.0} rpm", state.latest.rpm),
                },
                StatusRow {
                    label: "Target speed",
                    value: format!("{:.0} rpm", state.load_target_rpm),
                },
                StatusRow {
                    label: "Demand / torque req",
                    value: format!(
                        "{:.0}% / {:+.1} Nm",
                        state.driver_demand * 100.0,
                        state.post_fit_baseline.requested_brake_torque_nm
                    ),
                },
                StatusRow {
                    label: "Required brake torque",
                    value: format!(
                        "{:+.1} Nm / shaft est {:+.1}",
                        state.latest.torque_load_nm,
                        state.shaft_torque_estimate_nm()
                    ),
                },
                StatusRow {
                    label: "Net torque / power",
                    value: format!(
                        "{:+.1} Nm / {:+.1} kW / {:+.1} hp",
                        state.latest.torque_net_nm,
                        state.net_shaft_power_kw(),
                        state.net_shaft_power_hp()
                    ),
                },
                StatusRow {
                    label: "Throttle act / auto",
                    value: format!(
                        "{:.3} / {:.3}",
                        state.sim.control.throttle_cmd, state.post_fit_baseline.throttle_cmd
                    ),
                },
                StatusRow {
                    label: "Ignition act / auto",
                    value: format!(
                        "{:.1} / {:.1} deg BTDC",
                        state.sim.control.ignition_timing_deg,
                        state.post_fit_baseline.ignition_timing_deg
                    ),
                },
                StatusRow {
                    label: "VVT IN/EX act / auto",
                    value: format!(
                        "{:+.1}/{:+.1} / {:+.1}/{:+.1} deg",
                        state.sim.control.vvt_intake_deg,
                        state.sim.control.vvt_exhaust_deg,
                        state.post_fit_baseline.vvt_intake_deg,
                        state.post_fit_baseline.vvt_exhaust_deg
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
            operating_point_table: build_operating_point_table(state, &fit),
        }
    }
}

fn build_operating_point_table(
    state: &DashboardState,
    fit: &StartupFitStatus,
) -> OperatingPointTableViewModel {
    let fit_label = if fit.active {
        "CAND"
    } else if fit.timed_out {
        "BEST"
    } else {
        "FIT"
    };
    let result_marker = (!fit.active).then_some(OperatingPointMarkerViewModel {
        cell: OperatingPointCell {
            row: operating_point_brake_index(fit.required_brake_torque_nm),
            col: operating_point_speed_index(fit.avg_rpm),
        },
    });
    let mut cell_heat = vec![
        OperatingPointCellHeatViewModel::default();
        OPERATING_POINT_SPEED_LABELS.len() * OPERATING_POINT_BRAKE_LABELS.len()
    ];
    for point in &fit.coarse_candidate_points {
        let row = operating_point_brake_index(point[1]);
        let col = operating_point_speed_index(point[0]);
        let index = row * OPERATING_POINT_SPEED_LABELS.len() + col;
        cell_heat[index].coarse_hits = cell_heat[index].coarse_hits.saturating_add(1);
    }
    for point in &fit.refine_candidate_points {
        let row = operating_point_brake_index(point[1]);
        let col = operating_point_speed_index(point[0]);
        let index = row * OPERATING_POINT_SPEED_LABELS.len() + col;
        cell_heat[index].refine_hits = cell_heat[index].refine_hits.saturating_add(1);
    }
    let max_heat_hits = cell_heat
        .iter()
        .map(|cell| cell.coarse_hits.saturating_add(cell.refine_hits))
        .max()
        .unwrap_or(0);

    OperatingPointTableViewModel {
        fit_marker: OperatingPointMarkerViewModel {
            cell: OperatingPointCell {
                row: operating_point_brake_index(fit.release_required_brake_torque_nm),
                col: operating_point_speed_index(fit.release_avg_rpm),
            },
        },
        result_marker,
        cell_heat,
        max_heat_hits,
        live_marker: OperatingPointMarkerViewModel {
            cell: OperatingPointCell {
                row: operating_point_brake_index(state.latest.torque_load_nm),
                col: operating_point_speed_index(state.latest.rpm),
            },
        },
        fit_summary: format!(
            "{} {:.0} rpm / {:+.1} Nm / thr {:.3} / spark {:.1}",
            fit_label,
            fit.release_avg_rpm,
            fit.release_required_brake_torque_nm,
            fit.release_throttle_cmd,
            fit.release_ignition_timing_deg
        ),
        result_summary: (!fit.active).then_some(format!(
            "RES {:.0} rpm / {:+.1} Nm / thr {:.3} / spark {:.1}",
            fit.avg_rpm, fit.required_brake_torque_nm, fit.throttle_cmd, fit.ignition_timing_deg
        )),
        live_summary: format!(
            "LIVE {:.0} rpm / req {:+.1} / load {:+.1} / thr {:.3} / spark {:.1}",
            state.latest.rpm,
            state.post_fit_baseline.requested_brake_torque_nm,
            state.latest.torque_load_nm,
            state.sim.control.throttle_cmd,
            state.sim.control.ignition_timing_deg
        ),
        note: "16 x 16 bands. Cell color fills as candidates are evaluated: cool=coarse, green=refined. F=selected fit release, R=post-fit result, L=current live point under the active torque-request path.",
    }
}

fn operating_point_speed_index(rpm: f64) -> usize {
    band_index(rpm, &OPERATING_POINT_SPEED_UPPER_BOUNDS_RPM)
}

fn operating_point_brake_index(brake_torque_nm: f64) -> usize {
    band_index(brake_torque_nm, &OPERATING_POINT_BRAKE_UPPER_BOUNDS_NM)
}

fn band_index(value: f64, upper_bounds: &[f64]) -> usize {
    upper_bounds
        .iter()
        .position(|bound| value < *bound)
        .unwrap_or(upper_bounds.len())
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
    let wall_fraction = startup_fit_wall_limit_s()
        .map(|limit_s| (status.wall_elapsed_s / limit_s).clamp(0.0, 1.0) as f32)
        .unwrap_or(0.0);

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
        StartupFitPhase::Priming => match startup_fit_wall_limit_s() {
            Some(limit_s) => format!(
                "priming / {:.1} of {:.1} s wall-clock",
                status.wall_elapsed_s, limit_s
            ),
            None => format!(
                "priming / {:.1} s wall-clock / no cap",
                status.wall_elapsed_s
            ),
        },
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
                    "{} cap reached / best candidate applied",
                    startup_fit_wall_limit_label()
                )
            } else {
                "standard runtime live / actuator lab ready".to_owned()
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{
        OPERATING_POINT_BRAKE_LABELS, OPERATING_POINT_SPEED_LABELS, OperatingPointCell,
        operating_point_brake_index, operating_point_speed_index,
    };

    #[test]
    fn operating_point_speed_bands_cover_target_bin_edges() {
        assert_eq!(operating_point_speed_index(499.0), 0);
        assert_eq!(operating_point_speed_index(500.0), 1);
        assert_eq!(operating_point_speed_index(1999.0), 3);
        assert_eq!(operating_point_speed_index(2000.0), 4);
        assert_eq!(
            operating_point_speed_index(8200.0),
            OPERATING_POINT_SPEED_LABELS.len() - 1
        );
    }

    #[test]
    fn operating_point_brake_bands_cover_negative_and_high_load_cells() {
        assert_eq!(operating_point_brake_index(-0.1), 0);
        assert_eq!(operating_point_brake_index(0.0), 1);
        assert_eq!(operating_point_brake_index(9.9), 2);
        assert_eq!(
            operating_point_brake_index(70.0),
            OPERATING_POINT_BRAKE_LABELS.len() - 1
        );
    }

    #[test]
    fn operating_point_cell_is_copyable_for_table_markers() {
        let cell = OperatingPointCell { row: 2, col: 3 };
        assert_eq!(cell, OperatingPointCell { row: 2, col: 3 });
    }
}
