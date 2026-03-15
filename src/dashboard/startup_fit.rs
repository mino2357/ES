use std::sync::mpsc::{self, Receiver, TryRecvError};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use serde::{Deserialize, Serialize};

use super::startup_fit_artifact::{
    StartupFitArtifact, StartupFitArtifactEvaluation, StartupFitArtifactSnapshot,
};
use crate::config::{AppConfig, ExternalLoadMode};
use crate::simulator::{
    EngineState, Simulator, accuracy_priority_dt, estimate_mbt_deg,
    external_load_command_for_torque_nm, rad_s_to_rpm, running_state_error_norm, state_error_norm,
};

pub(super) const STARTUP_FIT_TARGET_RPM: f64 = 2_000.0;
pub(super) const STARTUP_FIT_INITIAL_THROTTLE: f64 = 1.0;
pub(super) const STARTUP_FIT_PRIMING_S: f64 = 0.30;
pub(super) const STARTUP_FIT_FAST_FORWARD_FACTOR: f64 = 5.0;
pub(super) const STARTUP_FIT_TARGET_WALL_TIME_S: f64 = 600.0;
pub(super) const STARTUP_FIT_MAX_WALL_TIME_S: Option<f64> = Some(STARTUP_FIT_TARGET_WALL_TIME_S);
pub(super) const STARTUP_FIT_REQUIRED_STABLE_WINDOWS: usize = 3;

const STARTUP_FIT_VERIFY_SETTLE_S: f64 = 0.08;
const STARTUP_FIT_VERIFY_MEASURE_S: f64 = 0.18;
const STARTUP_FIT_VERIFY_RPM_ERR: f64 = 24.0;
const STARTUP_FIT_VERIFY_NET_TORQUE_NM: f64 = 2.0;
const STARTUP_FIT_VERIFY_STALL_RPM: f64 = 240.0;
const STARTUP_FIT_VERIFY_MAX_FAILED_WINDOWS: usize = 3;
const STARTUP_FIT_THROTTLE_BIN_COUNT: usize = 1;
const STARTUP_FIT_COARSE_IGNITION_BIN_COUNT: usize = 10;
const STARTUP_FIT_REFINE_IGNITION_BIN_COUNT: usize = 6;
const STARTUP_FIT_MAX_CYCLES_PER_CANDIDATE: usize = 6;
const STARTUP_FIT_MIN_CYCLES_PER_CANDIDATE: usize = 2;
const STARTUP_FIT_MAX_STEPS_PER_CYCLE: usize = 1_200;
const STARTUP_FIT_LOAD_TORQUE_GAIN_NM_PER_RPM: f64 = 0.06;
const STARTUP_FIT_TORQUE_MARGIN_FROM_BEST_NM: f64 = 6.0;
const STARTUP_FIT_SELECTION_SCORE_MARGIN: f64 = 8.0;
const STARTUP_FIT_PERIODIC_ERROR_LIMIT: f64 = 0.05;
const STARTUP_FIT_CANDIDATE_RPM_ERR: f64 = 28.0;
const STARTUP_FIT_CANDIDATE_NET_TORQUE_NM: f64 = 2.5;
const STARTUP_FIT_MIN_DT_S: f64 = 1.0e-4;
const STARTUP_FIT_RKF_SAFETY: f64 = 0.90;
const STARTUP_FIT_RKF_MIN_FACTOR: f64 = 0.25;
const STARTUP_FIT_RKF_MAX_FACTOR: f64 = 2.20;
pub(super) const STARTUP_FIT_FRAME_WALL_BUDGET_S: f64 = 0.012;
const STARTUP_FIT_WORKER_SIMULATED_BATCH_S: f64 = 0.18;
const STARTUP_FIT_WORKER_STATUS_INTERVAL_S: f64 = 0.20;
const STARTUP_FIT_WORKER_ACCURACY_TARGET_DEG_PER_STEP: f64 = 4.5;
const STARTUP_FIT_WORKER_ACCURACY_DT_MAX_S: f64 = 0.0025;
const STARTUP_FIT_WORKER_DT_MIN_FLOOR_S: f64 = 1.0e-4;
const STARTUP_FIT_WOT_SWEEP_SETTLE_S: f64 = 0.10;
const STARTUP_FIT_WOT_SWEEP_MEASURE_S: f64 = 0.08;
const STARTUP_FIT_WOT_SWEEP_MAX_WINDOWS: usize = 4;
const STARTUP_FIT_WOT_SWEEP_RPM_ERR: f64 = 36.0;
const STARTUP_FIT_WOT_SWEEP_NET_TORQUE_NM: f64 = 4.0;
const STARTUP_FIT_WOT_CURVE_SMOOTH_PASSES: usize = 2;
const STARTUP_FIT_WOT_CURVE_SMOOTH_BLEND: f64 = 0.70;
const STARTUP_FIT_WOT_SWEEP_RPMS: [f64; 13] = [
    1_000.0, 1_500.0, 2_000.0, 2_500.0, 3_000.0, 3_500.0, 4_000.0, 4_500.0, 5_000.0, 5_500.0,
    6_000.0, 6_500.0, 7_000.0,
];

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum StartupFitPhase {
    Priming,
    Optimizing,
    Verifying,
    Ready,
}

impl StartupFitPhase {
    pub(super) fn label(self) -> &'static str {
        match self {
            Self::Priming => "priming",
            Self::Optimizing => "optimizing",
            Self::Verifying => "verifying",
            Self::Ready => "ready",
        }
    }

    pub(super) fn detail(self) -> &'static str {
        match self {
            Self::Priming => {
                "preparing a fired seed before discrete MBT search and finite-cycle load solving"
            }
            Self::Optimizing => {
                "evaluating WOT ignition bins, refining local MBT spark, and solving required brake torque with finite-cycle iteration"
            }
            Self::Verifying => {
                "applying the selected WOT spark, then trimming brake load on the live model to confirm it settles at target speed"
            }
            Self::Ready => {
                "startup fit completed; standard runtime is live and actuator-lab overrides are available"
            }
        }
    }
}

pub(super) fn startup_fit_wall_limit_label() -> String {
    match STARTUP_FIT_MAX_WALL_TIME_S {
        Some(limit_s) if limit_s >= 60.0 && (limit_s % 60.0).abs() <= f64::EPSILON => {
            format!("{:.0} min", limit_s / 60.0)
        }
        Some(limit_s) => format!("{:.1} s", limit_s),
        None => "no cap".to_owned(),
    }
}

pub(super) fn startup_fit_wall_limit_s() -> Option<f64> {
    STARTUP_FIT_MAX_WALL_TIME_S
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub(super) struct StartupFitControls {
    pub(super) throttle_cmd: f64,
    pub(super) ignition_timing_deg: f64,
    pub(super) vvt_intake_deg: f64,
    pub(super) vvt_exhaust_deg: f64,
    pub(super) load_cmd: f64,
}

impl StartupFitControls {
    fn clamp(self, bounds: StartupFitControlBounds) -> Self {
        Self {
            throttle_cmd: finite_f64(self.throttle_cmd, bounds.throttle_min)
                .clamp(bounds.throttle_min, bounds.throttle_max),
            ignition_timing_deg: finite_f64(self.ignition_timing_deg, bounds.ignition_min_deg)
                .clamp(bounds.ignition_min_deg, bounds.ignition_max_deg),
            vvt_intake_deg: bounds.vvt_default_deg,
            vvt_exhaust_deg: bounds.vvt_default_deg,
            load_cmd: finite_f64(self.load_cmd, 0.0).clamp(-1.0, 1.0),
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub(super) struct StartupFitControlBounds {
    pub(super) throttle_min: f64,
    pub(super) throttle_max: f64,
    pub(super) ignition_min_deg: f64,
    pub(super) ignition_max_deg: f64,
    pub(super) vvt_default_deg: f64,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub(super) struct StartupFitTorqueCurvePoint {
    pub(super) throttle_cmd: f64,
    pub(super) required_brake_torque_nm: f64,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub(super) struct StartupFitWotTorquePoint {
    pub(super) engine_speed_rpm: f64,
    pub(super) available_brake_torque_nm: f64,
    pub(super) ignition_timing_deg: f64,
}

#[derive(Debug, Clone)]
pub(super) struct StartupFitStatus {
    pub(super) active: bool,
    pub(super) loaded_from_cache: bool,
    pub(super) timed_out: bool,
    pub(super) phase: StartupFitPhase,
    pub(super) target_rpm: f64,
    pub(super) wall_elapsed_s: f64,
    pub(super) simulated_elapsed_s: f64,
    pub(super) iteration: usize,
    pub(super) max_iterations: usize,
    pub(super) stable_windows: usize,
    pub(super) required_stable_windows: usize,
    pub(super) avg_rpm: f64,
    pub(super) avg_net_torque_nm: f64,
    pub(super) required_brake_torque_nm: f64,
    pub(super) best_required_brake_torque_nm: f64,
    pub(super) torque_margin_to_best_nm: f64,
    pub(super) periodic_error_norm: f64,
    pub(super) throttle_cmd: f64,
    pub(super) ignition_timing_deg: f64,
    pub(super) vvt_intake_deg: f64,
    pub(super) vvt_exhaust_deg: f64,
    pub(super) load_cmd: f64,
    pub(super) release_avg_rpm: f64,
    pub(super) release_required_brake_torque_nm: f64,
    pub(super) release_throttle_cmd: f64,
    pub(super) release_ignition_timing_deg: f64,
    pub(super) coarse_candidate_points: Vec<[f64; 2]>,
    pub(super) refine_candidate_points: Vec<[f64; 2]>,
    pub(super) coarse_torque_curve: Vec<StartupFitTorqueCurvePoint>,
    pub(super) refine_torque_curve: Vec<StartupFitTorqueCurvePoint>,
    pub(super) wot_torque_curve: Vec<StartupFitWotTorquePoint>,
    pub(super) candidate_label: String,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum SearchStage {
    Coarse,
    Refine,
}

impl SearchStage {
    fn label(self) -> &'static str {
        match self {
            Self::Coarse => "coarse grid",
            Self::Refine => "local MBT",
        }
    }
}

#[derive(Debug, Clone, Copy)]
struct CandidateSpec {
    throttle_index: usize,
    ignition_deg: f64,
    search_stage: SearchStage,
}

#[derive(Debug, Clone, Copy, Default)]
struct CandidateEvaluation {
    avg_rpm: f64,
    avg_net_torque_nm: f64,
    required_brake_torque_nm: f64,
    load_cmd: f64,
    periodic_error_norm: f64,
    converged: bool,
}

#[derive(Debug, Clone, Copy)]
struct CandidateResult {
    controls: StartupFitControls,
    evaluation: CandidateEvaluation,
}

impl CandidateResult {
    fn ranking_score(self, target_rpm: f64) -> f64 {
        self.evaluation.required_brake_torque_nm
            - 0.04 * (self.evaluation.avg_rpm - target_rpm).abs()
            - 12.0 * self.evaluation.periodic_error_norm
            - 0.60 * self.evaluation.avg_net_torque_nm.abs()
            + if self.evaluation.converged { 50.0 } else { 0.0 }
    }
}

#[derive(Debug, Clone, Copy)]
struct ThrottleTrack {
    throttle_cmd: f64,
    coarse_best: Option<CandidateResult>,
    refined_best: Option<CandidateResult>,
}

impl ThrottleTrack {
    fn best(self) -> Option<CandidateResult> {
        self.refined_best.or(self.coarse_best)
    }
}

#[derive(Debug, Clone)]
struct ActiveCandidateEvaluation {
    candidate: CandidateSpec,
    controls: StartupFitControls,
    sim: Simulator,
    state: EngineState,
    next_dt_s: f64,
    last_cycle_end: Option<EngineState>,
    cycle_index: usize,
    cycle_step_count: usize,
    cycle_elapsed_s: f64,
    cycle_rpm_dt_sum: f64,
    cycle_net_torque_dt_sum: f64,
    cycle_load_torque_dt_sum: f64,
    last_evaluation: CandidateEvaluation,
}

enum CandidateAdvance {
    Pending(ActiveCandidateEvaluation),
    Completed {
        candidate: CandidateSpec,
        result: CandidateResult,
    },
}

#[derive(Debug)]
pub(super) struct StartupFitState {
    phase: StartupFitPhase,
    target_rpm: f64,
    started_sim_time_s: f64,
    started_wall_at: Instant,
    primed_seed: Option<Simulator>,
    search_stage: SearchStage,
    coarse_candidates: Vec<CandidateSpec>,
    refine_candidates: Vec<CandidateSpec>,
    next_candidate_index: usize,
    throttle_tracks: Vec<ThrottleTrack>,
    iteration: usize,
    stable_windows: usize,
    verify_elapsed_s: f64,
    verify_measure_elapsed_s: f64,
    verify_sample_count: usize,
    verify_rpm_sum: f64,
    verify_net_torque_sum_nm: f64,
    verify_load_torque_sum_nm: f64,
    verify_failed_windows: usize,
    coarse_candidate_points: Vec<[f64; 2]>,
    refine_candidate_points: Vec<[f64; 2]>,
    last_evaluation: CandidateEvaluation,
    best_result: Option<CandidateResult>,
    best_required_brake_torque_nm: f64,
    torque_margin_to_best_nm: f64,
    current_controls: StartupFitControls,
    best_controls: StartupFitControls,
    ready_fit_torque_curve: Vec<StartupFitTorqueCurvePoint>,
    ready_wot_torque_curve: Vec<StartupFitWotTorquePoint>,
    rejected_release_controls: Vec<StartupFitControls>,
    active_candidate: Option<ActiveCandidateEvaluation>,
    pending_live_reset: bool,
    last_candidate_label: String,
    timed_out: bool,
    loaded_from_cache: bool,
}

impl ActiveCandidateEvaluation {
    fn new(
        seed: &Simulator,
        target_rpm: f64,
        candidate: CandidateSpec,
        mut controls: StartupFitControls,
    ) -> Self {
        let mut sim = seed.clone();
        sim.control.spark_cmd = true;
        sim.control.fuel_cmd = true;
        sim.control.throttle_cmd = controls.throttle_cmd;
        sim.control.ignition_timing_deg = controls.ignition_timing_deg;
        sim.control.vvt_intake_deg = controls.vvt_intake_deg;
        sim.control.vvt_exhaust_deg = controls.vvt_exhaust_deg;
        sim.control.load_cmd = 0.0;
        sim.seed_operating_point(
            target_rpm,
            controls.throttle_cmd,
            controls.ignition_timing_deg,
        );

        let unloaded = sim.locked_cycle_average(sim.state, 96);
        let load_cmd = external_load_command_for_torque_nm(
            (unloaded.torque_net_nm + unloaded.torque_load_nm).max(0.0),
            sim.state.omega_rad_s,
            &sim.model.external_load,
        );
        sim.control.load_cmd = load_cmd;
        controls.load_cmd = load_cmd;

        Self {
            candidate,
            controls,
            state: sim.state,
            next_dt_s: accuracy_priority_dt(target_rpm, &sim.numerics)
                .clamp(STARTUP_FIT_MIN_DT_S, sim.numerics.accuracy_dt_max_s),
            sim,
            last_cycle_end: None,
            cycle_index: 0,
            cycle_step_count: 0,
            cycle_elapsed_s: 0.0,
            cycle_rpm_dt_sum: 0.0,
            cycle_net_torque_dt_sum: 0.0,
            cycle_load_torque_dt_sum: 0.0,
            last_evaluation: CandidateEvaluation::default(),
        }
    }

    fn progress_label(&self) -> String {
        format!(
            "{} / thr {:.3} / spk {:.1} / cyc {}/{}",
            self.candidate.search_stage.label(),
            self.controls.throttle_cmd,
            self.controls.ignition_timing_deg,
            self.cycle_index + 1,
            STARTUP_FIT_MAX_CYCLES_PER_CANDIDATE,
        )
    }

    fn advance_until(
        mut self,
        target_rpm: f64,
        fit_deadline: Option<Instant>,
        frame_deadline: Option<Instant>,
    ) -> CandidateAdvance {
        if STARTUP_FIT_THROTTLE_BIN_COUNT == 1 {
            return self.advance_wot_locked_cycle(target_rpm);
        }
        while self.cycle_index < STARTUP_FIT_MAX_CYCLES_PER_CANDIDATE {
            if deadline_reached(fit_deadline) || deadline_reached(frame_deadline) {
                return CandidateAdvance::Pending(self);
            }
            if self.cycle_step_count >= STARTUP_FIT_MAX_STEPS_PER_CYCLE {
                break;
            }

            let current_sample = self.sim.sample_fit_state(self.state);
            let event_dt_s =
                time_to_next_ignition_event_s(self.state, current_sample.burn_start_deg);
            let mut try_dt_s = self
                .next_dt_s
                .min(event_dt_s)
                .clamp(STARTUP_FIT_MIN_DT_S, self.sim.numerics.accuracy_dt_max_s);

            loop {
                let (rk4, rk5) = self.sim.advance_state_rkf45_pair(self.state, try_dt_s);
                let error_norm = state_error_norm(rk4, rk5, &self.sim.numerics);
                if error_norm <= 1.0 || try_dt_s <= STARTUP_FIT_MIN_DT_S * 1.01 {
                    let accepted = rk5;
                    let accepted_sample = self.sim.sample_fit_state(accepted);
                    self.cycle_rpm_dt_sum += accepted_sample.rpm * try_dt_s;
                    self.cycle_net_torque_dt_sum += accepted_sample.torque_net_nm * try_dt_s;
                    self.cycle_load_torque_dt_sum += accepted_sample.torque_load_nm * try_dt_s;
                    self.cycle_elapsed_s += try_dt_s;
                    self.cycle_step_count = self.cycle_step_count.saturating_add(1);
                    let crossed_cycle = accepted.theta_rad < self.state.theta_rad;
                    self.next_dt_s = rkf45_next_dt(try_dt_s, error_norm)
                        .clamp(STARTUP_FIT_MIN_DT_S, self.sim.numerics.accuracy_dt_max_s);
                    self.state = accepted;
                    if crossed_cycle {
                        let completed_cycles = self.cycle_index + 1;
                        let duration = self.cycle_elapsed_s.max(f64::EPSILON);
                        let avg_rpm = self.cycle_rpm_dt_sum / duration;
                        let avg_net_torque_nm = self.cycle_net_torque_dt_sum / duration;
                        let avg_load_torque_nm = self.cycle_load_torque_dt_sum / duration;
                        let periodic_error_norm =
                            self.last_cycle_end.map_or(f64::INFINITY, |prev| {
                                running_state_error_norm(prev, self.state, &self.sim.numerics)
                            });
                        let required_brake_torque_nm = avg_load_torque_nm.max(0.0);
                        let torque_request_nm = (required_brake_torque_nm
                            + avg_net_torque_nm
                            + STARTUP_FIT_LOAD_TORQUE_GAIN_NM_PER_RPM * (avg_rpm - target_rpm))
                            .max(0.0);
                        let load_cmd = external_load_command_for_torque_nm(
                            torque_request_nm,
                            self.state.omega_rad_s,
                            &self.sim.model.external_load,
                        );
                        self.sim.control.load_cmd = load_cmd;
                        self.controls.load_cmd = load_cmd;
                        self.last_evaluation = CandidateEvaluation {
                            avg_rpm,
                            avg_net_torque_nm,
                            required_brake_torque_nm,
                            load_cmd,
                            periodic_error_norm,
                            converged: completed_cycles >= STARTUP_FIT_MIN_CYCLES_PER_CANDIDATE
                                && (avg_rpm - target_rpm).abs() <= STARTUP_FIT_CANDIDATE_RPM_ERR
                                && avg_net_torque_nm.abs() <= STARTUP_FIT_CANDIDATE_NET_TORQUE_NM
                                && periodic_error_norm <= STARTUP_FIT_PERIODIC_ERROR_LIMIT,
                        };
                        self.last_cycle_end = Some(self.state);
                        self.cycle_index = completed_cycles;
                        self.cycle_step_count = 0;
                        self.cycle_elapsed_s = 0.0;
                        self.cycle_rpm_dt_sum = 0.0;
                        self.cycle_net_torque_dt_sum = 0.0;
                        self.cycle_load_torque_dt_sum = 0.0;
                        if self.last_evaluation.converged
                            || self.cycle_index >= STARTUP_FIT_MAX_CYCLES_PER_CANDIDATE
                        {
                            return CandidateAdvance::Completed {
                                candidate: self.candidate,
                                result: CandidateResult {
                                    controls: self.controls,
                                    evaluation: self.last_evaluation,
                                },
                            };
                        }
                    }
                    break;
                }
                try_dt_s = rkf45_next_dt(try_dt_s, error_norm)
                    .clamp(STARTUP_FIT_MIN_DT_S, self.sim.numerics.accuracy_dt_max_s);
                if deadline_reached(fit_deadline) || deadline_reached(frame_deadline) {
                    return CandidateAdvance::Pending(self);
                }
            }
        }

        CandidateAdvance::Completed {
            candidate: self.candidate,
            result: CandidateResult {
                controls: self.controls,
                evaluation: self.last_evaluation,
            },
        }
    }

    fn advance_wot_locked_cycle(mut self, target_rpm: f64) -> CandidateAdvance {
        let mut previous_load_cmd = self.controls.load_cmd;

        while self.cycle_index < STARTUP_FIT_MAX_CYCLES_PER_CANDIDATE {
            let average = self.sim.locked_cycle_average(self.state, 96);
            let required_brake_torque_nm = average.torque_load_nm.max(0.0);
            let periodic_error_norm = if self.cycle_index == 0 {
                f64::INFINITY
            } else {
                (self.controls.load_cmd - previous_load_cmd).abs()
            };
            self.last_evaluation = CandidateEvaluation {
                avg_rpm: target_rpm,
                avg_net_torque_nm: average.torque_net_nm,
                required_brake_torque_nm,
                load_cmd: self.controls.load_cmd,
                periodic_error_norm,
                converged: self.cycle_index + 1 >= STARTUP_FIT_MIN_CYCLES_PER_CANDIDATE
                    && average.torque_net_nm.abs() <= STARTUP_FIT_CANDIDATE_NET_TORQUE_NM
                    && periodic_error_norm <= STARTUP_FIT_PERIODIC_ERROR_LIMIT,
            };
            self.cycle_index = self.cycle_index.saturating_add(1);
            if self.last_evaluation.converged
                || self.cycle_index >= STARTUP_FIT_MAX_CYCLES_PER_CANDIDATE
            {
                return CandidateAdvance::Completed {
                    candidate: self.candidate,
                    result: CandidateResult {
                        controls: self.controls,
                        evaluation: self.last_evaluation,
                    },
                };
            }

            previous_load_cmd = self.controls.load_cmd;
            let target_brake_torque_nm =
                (required_brake_torque_nm + average.torque_net_nm).max(0.0);
            self.controls.load_cmd = external_load_command_for_torque_nm(
                target_brake_torque_nm,
                self.state.omega_rad_s,
                &self.sim.model.external_load,
            );
            self.sim.control.load_cmd = self.controls.load_cmd;
        }

        CandidateAdvance::Completed {
            candidate: self.candidate,
            result: CandidateResult {
                controls: self.controls,
                evaluation: self.last_evaluation,
            },
        }
    }
}

impl StartupFitState {
    pub(super) fn new(
        started_sim_time_s: f64,
        target_rpm: f64,
        initial_controls: StartupFitControls,
    ) -> Self {
        Self {
            phase: StartupFitPhase::Priming,
            target_rpm,
            started_sim_time_s,
            started_wall_at: Instant::now(),
            primed_seed: None,
            search_stage: SearchStage::Coarse,
            coarse_candidates: Vec::new(),
            refine_candidates: Vec::new(),
            next_candidate_index: 0,
            throttle_tracks: Vec::new(),
            iteration: 0,
            stable_windows: 0,
            verify_elapsed_s: 0.0,
            verify_measure_elapsed_s: 0.0,
            verify_sample_count: 0,
            verify_rpm_sum: 0.0,
            verify_net_torque_sum_nm: 0.0,
            verify_load_torque_sum_nm: 0.0,
            verify_failed_windows: 0,
            coarse_candidate_points: Vec::new(),
            refine_candidate_points: Vec::new(),
            last_evaluation: CandidateEvaluation::default(),
            best_result: None,
            best_required_brake_torque_nm: 0.0,
            torque_margin_to_best_nm: STARTUP_FIT_TORQUE_MARGIN_FROM_BEST_NM,
            current_controls: initial_controls,
            best_controls: initial_controls,
            ready_fit_torque_curve: Vec::new(),
            ready_wot_torque_curve: Vec::new(),
            rejected_release_controls: Vec::new(),
            active_candidate: None,
            pending_live_reset: false,
            last_candidate_label: "waiting for priming".to_owned(),
            timed_out: false,
            loaded_from_cache: false,
        }
    }

    pub(super) fn from_artifact(started_sim_time_s: f64, artifact: &StartupFitArtifact) -> Self {
        let release_evaluation = artifact.release_evaluation;
        let release_controls = artifact.release_controls;
        let fit_torque_curve = if artifact.torque_curve.is_empty() {
            vec![StartupFitTorqueCurvePoint {
                throttle_cmd: release_controls.throttle_cmd,
                required_brake_torque_nm: release_evaluation.required_brake_torque_nm,
            }]
        } else {
            artifact.torque_curve.clone()
        };
        let wot_torque_curve = if artifact.wot_torque_curve.is_empty() {
            vec![StartupFitWotTorquePoint {
                engine_speed_rpm: artifact.target_rpm,
                available_brake_torque_nm: release_evaluation.required_brake_torque_nm.max(0.0),
                ignition_timing_deg: release_controls.ignition_timing_deg,
            }]
        } else {
            artifact.wot_torque_curve.clone()
        };
        let last_evaluation = CandidateEvaluation {
            avg_rpm: release_evaluation.avg_rpm,
            avg_net_torque_nm: release_evaluation.avg_net_torque_nm,
            required_brake_torque_nm: release_evaluation.required_brake_torque_nm,
            load_cmd: release_evaluation.load_cmd,
            periodic_error_norm: release_evaluation.periodic_error_norm,
            converged: release_evaluation.converged,
        };
        let best_result = CandidateResult {
            controls: release_controls,
            evaluation: last_evaluation,
        };

        Self {
            phase: StartupFitPhase::Ready,
            target_rpm: artifact.target_rpm,
            started_sim_time_s,
            started_wall_at: Instant::now(),
            primed_seed: None,
            search_stage: SearchStage::Refine,
            coarse_candidates: Vec::new(),
            refine_candidates: Vec::new(),
            next_candidate_index: 0,
            throttle_tracks: Vec::new(),
            iteration: 0,
            stable_windows: STARTUP_FIT_REQUIRED_STABLE_WINDOWS,
            verify_elapsed_s: 0.0,
            verify_measure_elapsed_s: 0.0,
            verify_sample_count: 0,
            verify_rpm_sum: 0.0,
            verify_net_torque_sum_nm: 0.0,
            verify_load_torque_sum_nm: 0.0,
            verify_failed_windows: 0,
            coarse_candidate_points: Vec::new(),
            refine_candidate_points: Vec::new(),
            last_evaluation,
            best_result: Some(best_result),
            best_required_brake_torque_nm: artifact.best_required_brake_torque_nm,
            torque_margin_to_best_nm: artifact.torque_margin_to_best_nm,
            current_controls: release_controls,
            best_controls: release_controls,
            ready_fit_torque_curve: fit_torque_curve,
            ready_wot_torque_curve: wot_torque_curve,
            rejected_release_controls: Vec::new(),
            active_candidate: None,
            pending_live_reset: false,
            last_candidate_label: "loaded cached startup-fit map".to_owned(),
            timed_out: artifact.timed_out,
            loaded_from_cache: true,
        }
    }

    pub(super) fn from_snapshot(
        started_sim_time_s: f64,
        snapshot: &StartupFitArtifactSnapshot,
    ) -> Self {
        let release_evaluation = snapshot.release_evaluation;
        let release_controls = snapshot.release_controls;
        let fit_torque_curve = if snapshot.torque_curve.is_empty() {
            vec![StartupFitTorqueCurvePoint {
                throttle_cmd: release_controls.throttle_cmd,
                required_brake_torque_nm: release_evaluation.required_brake_torque_nm,
            }]
        } else {
            snapshot.torque_curve.clone()
        };
        let wot_torque_curve = if snapshot.wot_torque_curve.is_empty() {
            vec![StartupFitWotTorquePoint {
                engine_speed_rpm: snapshot.target_rpm,
                available_brake_torque_nm: release_evaluation.required_brake_torque_nm.max(0.0),
                ignition_timing_deg: release_controls.ignition_timing_deg,
            }]
        } else {
            snapshot.wot_torque_curve.clone()
        };
        let last_evaluation = CandidateEvaluation {
            avg_rpm: release_evaluation.avg_rpm,
            avg_net_torque_nm: release_evaluation.avg_net_torque_nm,
            required_brake_torque_nm: release_evaluation.required_brake_torque_nm,
            load_cmd: release_evaluation.load_cmd,
            periodic_error_norm: release_evaluation.periodic_error_norm,
            converged: release_evaluation.converged,
        };
        let best_result = CandidateResult {
            controls: release_controls,
            evaluation: last_evaluation,
        };

        Self {
            phase: StartupFitPhase::Ready,
            target_rpm: snapshot.target_rpm,
            started_sim_time_s,
            started_wall_at: Instant::now(),
            primed_seed: None,
            search_stage: SearchStage::Refine,
            coarse_candidates: Vec::new(),
            refine_candidates: Vec::new(),
            next_candidate_index: 0,
            throttle_tracks: Vec::new(),
            iteration: 0,
            stable_windows: STARTUP_FIT_REQUIRED_STABLE_WINDOWS,
            verify_elapsed_s: 0.0,
            verify_measure_elapsed_s: 0.0,
            verify_sample_count: 0,
            verify_rpm_sum: 0.0,
            verify_net_torque_sum_nm: 0.0,
            verify_load_torque_sum_nm: 0.0,
            verify_failed_windows: 0,
            coarse_candidate_points: Vec::new(),
            refine_candidate_points: Vec::new(),
            last_evaluation,
            best_result: Some(best_result),
            best_required_brake_torque_nm: snapshot.best_required_brake_torque_nm,
            torque_margin_to_best_nm: snapshot.torque_margin_to_best_nm,
            current_controls: release_controls,
            best_controls: release_controls,
            ready_fit_torque_curve: fit_torque_curve,
            ready_wot_torque_curve: wot_torque_curve,
            rejected_release_controls: Vec::new(),
            active_candidate: None,
            pending_live_reset: false,
            last_candidate_label: "startup fit worker completed".to_owned(),
            timed_out: snapshot.timed_out,
            loaded_from_cache: false,
        }
    }

    pub(super) fn is_active(&self) -> bool {
        self.phase != StartupFitPhase::Ready
    }

    pub(super) fn phase(&self) -> StartupFitPhase {
        self.phase
    }

    pub(super) fn applied_controls(&self) -> StartupFitControls {
        self.current_controls
    }

    pub(super) fn selected_required_brake_torque_nm(&self) -> f64 {
        self.best_result
            .map(|result| result.evaluation.required_brake_torque_nm)
            .unwrap_or(self.last_evaluation.required_brake_torque_nm)
            .max(0.0)
    }

    pub(super) fn release_controls(&self) -> StartupFitControls {
        self.best_result
            .map(|result| result.controls)
            .unwrap_or(self.best_controls)
    }

    pub(super) fn best_torque_curve(&self) -> Vec<StartupFitTorqueCurvePoint> {
        if !self.ready_fit_torque_curve.is_empty() {
            return self.ready_fit_torque_curve.clone();
        }
        self.throttle_tracks
            .iter()
            .filter_map(|track| track.best().map(torque_curve_point))
            .collect()
    }

    pub(super) fn wot_torque_curve(&self) -> Vec<StartupFitWotTorquePoint> {
        if !self.ready_wot_torque_curve.is_empty() {
            return self.ready_wot_torque_curve.clone();
        }
        vec![StartupFitWotTorquePoint {
            engine_speed_rpm: self.target_rpm,
            available_brake_torque_nm: self.selected_required_brake_torque_nm(),
            ignition_timing_deg: self.release_controls().ignition_timing_deg,
        }]
    }

    pub(super) fn release_avg_rpm(&self) -> f64 {
        self.best_result
            .map(|result| result.evaluation.avg_rpm)
            .unwrap_or(self.last_evaluation.avg_rpm)
            .max(0.0)
    }

    fn set_wot_torque_curve(&mut self, curve: Vec<StartupFitWotTorquePoint>) {
        self.ready_wot_torque_curve = curve;
    }

    pub(super) fn artifact(&self) -> Option<StartupFitArtifactSnapshot> {
        let release = self.best_result.unwrap_or(CandidateResult {
            controls: self.best_controls,
            evaluation: self.last_evaluation,
        });
        let mut torque_curve = self.best_torque_curve();
        if torque_curve.is_empty() {
            torque_curve.push(StartupFitTorqueCurvePoint {
                throttle_cmd: release.controls.throttle_cmd,
                required_brake_torque_nm: release.evaluation.required_brake_torque_nm,
            });
        }
        let release_evaluation = StartupFitArtifactEvaluation {
            avg_rpm: release.evaluation.avg_rpm,
            avg_net_torque_nm: release.evaluation.avg_net_torque_nm,
            required_brake_torque_nm: release.evaluation.required_brake_torque_nm,
            load_cmd: release.evaluation.load_cmd,
            periodic_error_norm: release.evaluation.periodic_error_norm,
            converged: release.evaluation.converged,
        };

        Some(StartupFitArtifactSnapshot {
            target_rpm: self.target_rpm,
            timed_out: self.timed_out,
            release_controls: release.controls,
            release_evaluation,
            best_required_brake_torque_nm: self
                .best_required_brake_torque_nm
                .max(release.evaluation.required_brake_torque_nm),
            torque_margin_to_best_nm: self.torque_margin_to_best_nm.max(0.0),
            torque_curve,
            wot_torque_curve: self.wot_torque_curve(),
        })
    }

    pub(super) fn snapshot(&self, simulated_time_s: f64) -> StartupFitStatus {
        let release = self.best_result.unwrap_or(CandidateResult {
            controls: self.best_controls,
            evaluation: self.last_evaluation,
        });
        StartupFitStatus {
            active: self.is_active(),
            loaded_from_cache: self.loaded_from_cache,
            timed_out: self.timed_out,
            phase: self.phase,
            target_rpm: self.target_rpm,
            wall_elapsed_s: self.started_wall_at.elapsed().as_secs_f64(),
            simulated_elapsed_s: (simulated_time_s - self.started_sim_time_s).max(0.0),
            iteration: self.iteration,
            max_iterations: self.coarse_candidates.len() + self.refine_candidates.len(),
            stable_windows: self.stable_windows,
            required_stable_windows: STARTUP_FIT_REQUIRED_STABLE_WINDOWS,
            avg_rpm: self.last_evaluation.avg_rpm,
            avg_net_torque_nm: self.last_evaluation.avg_net_torque_nm,
            required_brake_torque_nm: self.last_evaluation.required_brake_torque_nm,
            best_required_brake_torque_nm: self.best_required_brake_torque_nm,
            torque_margin_to_best_nm: self.torque_margin_to_best_nm,
            periodic_error_norm: self.last_evaluation.periodic_error_norm,
            throttle_cmd: self.current_controls.throttle_cmd,
            ignition_timing_deg: self.current_controls.ignition_timing_deg,
            vvt_intake_deg: self.current_controls.vvt_intake_deg,
            vvt_exhaust_deg: self.current_controls.vvt_exhaust_deg,
            load_cmd: self.current_controls.load_cmd,
            release_avg_rpm: release.evaluation.avg_rpm,
            release_required_brake_torque_nm: release.evaluation.required_brake_torque_nm,
            release_throttle_cmd: release.controls.throttle_cmd,
            release_ignition_timing_deg: release.controls.ignition_timing_deg,
            coarse_candidate_points: self.coarse_candidate_points.clone(),
            refine_candidate_points: self.refine_candidate_points.clone(),
            coarse_torque_curve: self
                .throttle_tracks
                .iter()
                .filter_map(|track| track.coarse_best.map(torque_curve_point))
                .collect(),
            refine_torque_curve: if !self.ready_fit_torque_curve.is_empty() {
                self.ready_fit_torque_curve.clone()
            } else {
                self.throttle_tracks
                    .iter()
                    .filter_map(|track| track.refined_best.map(torque_curve_point))
                    .collect()
            },
            wot_torque_curve: self.wot_torque_curve(),
            candidate_label: self.last_candidate_label.clone(),
        }
    }

    pub(super) fn take_live_reset_seed(&mut self) -> Option<Simulator> {
        if !self.pending_live_reset {
            return None;
        }
        self.pending_live_reset = false;
        let mut seed = self.primed_seed.clone()?;
        seed.control.spark_cmd = true;
        seed.control.fuel_cmd = true;
        seed.control.throttle_cmd = self.current_controls.throttle_cmd;
        seed.control.ignition_timing_deg = self.current_controls.ignition_timing_deg;
        seed.control.vvt_intake_deg = self.current_controls.vvt_intake_deg;
        seed.control.vvt_exhaust_deg = self.current_controls.vvt_exhaust_deg;
        seed.control.load_cmd = self.current_controls.load_cmd;
        seed.seed_operating_point(
            self.target_rpm,
            self.current_controls.throttle_cmd,
            self.current_controls.ignition_timing_deg,
        );
        Some(seed)
    }

    pub(super) fn record_live_sample(
        &mut self,
        frame_simulated_s: f64,
        rpm: f64,
        net_torque_nm: f64,
        load_torque_nm: f64,
    ) {
        if self.phase != StartupFitPhase::Verifying {
            return;
        }

        let dt = finite_f64(frame_simulated_s, 0.0).max(0.0);
        self.verify_elapsed_s += dt;
        if self.verify_elapsed_s <= STARTUP_FIT_VERIFY_SETTLE_S {
            return;
        }
        if rpm.is_finite() && net_torque_nm.is_finite() && load_torque_nm.is_finite() {
            self.verify_measure_elapsed_s += dt;
            self.verify_sample_count = self.verify_sample_count.saturating_add(1);
            self.verify_rpm_sum += rpm;
            self.verify_net_torque_sum_nm += net_torque_nm;
            self.verify_load_torque_sum_nm += load_torque_nm;
        }
    }

    pub(super) fn update(
        &mut self,
        simulated_time_s: f64,
        live_sim: &Simulator,
        bounds: StartupFitControlBounds,
        frame_deadline: Option<Instant>,
    ) {
        if let Some(limit_s) = STARTUP_FIT_MAX_WALL_TIME_S {
            if self.phase != StartupFitPhase::Ready
                && self.started_wall_at.elapsed() >= Duration::from_secs_f64(limit_s)
            {
                self.finish_ready(true);
                return;
            }
        }

        if self.phase == StartupFitPhase::Priming {
            if simulated_time_s - self.started_sim_time_s < STARTUP_FIT_PRIMING_S {
                return;
            }
            self.primed_seed = Some(live_sim.clone());
            self.initialize_search(bounds);
            self.phase = StartupFitPhase::Optimizing;
        }

        match self.phase {
            StartupFitPhase::Optimizing => {
                self.advance_search(bounds, frame_deadline);
            }
            StartupFitPhase::Verifying => {
                self.advance_verification();
            }
            StartupFitPhase::Priming | StartupFitPhase::Ready => {}
        }
    }

    fn initialize_search(&mut self, bounds: StartupFitControlBounds) {
        let throttle_bins = vec![bounds.throttle_max];
        self.throttle_tracks = throttle_bins
            .iter()
            .copied()
            .map(|throttle_cmd| ThrottleTrack {
                throttle_cmd,
                coarse_best: None,
                refined_best: None,
            })
            .collect();
        let ignition_bins = linspace(
            bounds.ignition_min_deg,
            bounds.ignition_max_deg,
            STARTUP_FIT_COARSE_IGNITION_BIN_COUNT,
        );
        self.coarse_candidates.clear();
        for (throttle_index, _) in self.throttle_tracks.iter().enumerate() {
            for ignition_deg in ignition_bins.iter().copied() {
                self.coarse_candidates.push(CandidateSpec {
                    throttle_index,
                    ignition_deg,
                    search_stage: SearchStage::Coarse,
                });
            }
        }
        self.refine_candidates.clear();
        self.next_candidate_index = 0;
        self.search_stage = SearchStage::Coarse;
        self.coarse_candidate_points.clear();
        self.refine_candidate_points.clear();
        self.active_candidate = None;
        self.last_candidate_label = "building coarse MBT grid".to_owned();
    }

    fn advance_search(&mut self, bounds: StartupFitControlBounds, frame_deadline: Option<Instant>) {
        if self.primed_seed.is_none() {
            return;
        }
        let fit_deadline = startup_fit_deadline(self.started_wall_at);

        loop {
            if deadline_reached(frame_deadline) {
                return;
            }

            if self.active_candidate.is_none() {
                let candidate = match self.search_stage {
                    SearchStage::Coarse => self
                        .coarse_candidates
                        .get(self.next_candidate_index)
                        .copied(),
                    SearchStage::Refine => self
                        .refine_candidates
                        .get(self.next_candidate_index)
                        .copied(),
                };

                let Some(candidate) = candidate else {
                    match self.search_stage {
                        SearchStage::Coarse => {
                            self.build_refine_candidates(bounds);
                            self.search_stage = SearchStage::Refine;
                            self.next_candidate_index = 0;
                            self.last_candidate_label =
                                "switching to local MBT refinement".to_owned();
                        }
                        SearchStage::Refine => {
                            self.select_final_candidate();
                            if STARTUP_FIT_THROTTLE_BIN_COUNT == 1 {
                                self.finish_ready(false);
                            } else {
                                self.phase = StartupFitPhase::Verifying;
                                self.reset_verify_window();
                            }
                        }
                    }
                    return;
                };

                let throttle_cmd = self.throttle_tracks[candidate.throttle_index].throttle_cmd;
                let candidate_controls = StartupFitControls {
                    throttle_cmd,
                    ignition_timing_deg: candidate.ignition_deg,
                    vvt_intake_deg: bounds.vvt_default_deg,
                    vvt_exhaust_deg: bounds.vvt_default_deg,
                    load_cmd: self.current_controls.load_cmd,
                }
                .clamp(bounds);
                let seed = self
                    .primed_seed
                    .as_ref()
                    .expect("primed seed available while optimizing");
                self.iteration = self.iteration.saturating_add(1);
                self.current_controls = candidate_controls;
                self.active_candidate = Some(ActiveCandidateEvaluation::new(
                    seed,
                    self.target_rpm,
                    candidate,
                    candidate_controls,
                ));
            }

            let active = self
                .active_candidate
                .take()
                .expect("active candidate present");
            match active.advance_until(self.target_rpm, fit_deadline, frame_deadline) {
                CandidateAdvance::Pending(active) => {
                    self.last_candidate_label = active.progress_label();
                    self.current_controls = active.controls;
                    self.last_evaluation = active.last_evaluation;
                    self.active_candidate = Some(active);
                    return;
                }
                CandidateAdvance::Completed { candidate, result } => {
                    self.last_evaluation = result.evaluation;
                    self.last_candidate_label = format!(
                        "{} / thr {:.3} / spk {:.1}",
                        candidate.search_stage.label(),
                        result.controls.throttle_cmd,
                        result.controls.ignition_timing_deg
                    );
                    record_candidate_point(
                        match candidate.search_stage {
                            SearchStage::Coarse => &mut self.coarse_candidate_points,
                            SearchStage::Refine => &mut self.refine_candidate_points,
                        },
                        result.evaluation,
                    );
                    self.update_best_result(candidate.throttle_index, result);
                    self.current_controls = self.best_controls;
                    self.next_candidate_index = self.next_candidate_index.saturating_add(1);
                }
            }
        }
    }

    fn update_best_result(&mut self, throttle_index: usize, result: CandidateResult) {
        let track = &mut self.throttle_tracks[throttle_index];
        let target = match self.search_stage {
            SearchStage::Coarse => &mut track.coarse_best,
            SearchStage::Refine => &mut track.refined_best,
        };
        let replace = target
            .map(|current| {
                result.ranking_score(self.target_rpm) > current.ranking_score(self.target_rpm)
            })
            .unwrap_or(true);
        if replace {
            *target = Some(result);
        }

        let improved = self
            .best_result
            .map(|current| {
                result.ranking_score(self.target_rpm) > current.ranking_score(self.target_rpm)
            })
            .unwrap_or(true);
        if improved {
            self.best_result = Some(result);
            self.best_controls = result.controls;
            self.current_controls = result.controls;
            self.best_required_brake_torque_nm = result.evaluation.required_brake_torque_nm;
        }
    }

    fn build_refine_candidates(&mut self, bounds: StartupFitControlBounds) {
        self.refine_candidates.clear();
        let coarse_span = (bounds.ignition_max_deg - bounds.ignition_min_deg)
            / (STARTUP_FIT_COARSE_IGNITION_BIN_COUNT
                .saturating_sub(1)
                .max(1) as f64);
        for (throttle_index, track) in self.throttle_tracks.iter().enumerate() {
            let Some(best) = track.coarse_best else {
                continue;
            };
            let min_deg = (best.controls.ignition_timing_deg - coarse_span)
                .clamp(bounds.ignition_min_deg, bounds.ignition_max_deg);
            let max_deg = (best.controls.ignition_timing_deg + coarse_span)
                .clamp(bounds.ignition_min_deg, bounds.ignition_max_deg);
            for ignition_deg in linspace(min_deg, max_deg, STARTUP_FIT_REFINE_IGNITION_BIN_COUNT) {
                self.refine_candidates.push(CandidateSpec {
                    throttle_index,
                    ignition_deg,
                    search_stage: SearchStage::Refine,
                });
            }
        }
    }

    fn select_final_candidate(&mut self) {
        let throttle_bests: Vec<CandidateResult> = self
            .throttle_tracks
            .iter()
            .filter_map(|track| track.best())
            .collect();
        if throttle_bests.is_empty() {
            self.last_candidate_label = "no release candidate available".to_owned();
            self.finish_ready(false);
            return;
        }
        self.rejected_release_controls.clear();
        let Some(selected) = select_candidate_for_release(
            &throttle_bests,
            self.target_rpm,
            &self.rejected_release_controls,
        ) else {
            self.last_candidate_label = "no converged release candidate available".to_owned();
            self.finish_ready(false);
            return;
        };
        let best_torque = throttle_bests
            .iter()
            .map(|result| result.evaluation.required_brake_torque_nm)
            .fold(f64::NEG_INFINITY, f64::max);
        self.torque_margin_to_best_nm =
            (best_torque - selected.evaluation.required_brake_torque_nm).max(0.0);
        self.best_controls = selected.controls;
        self.current_controls = selected.controls;
        self.best_result = Some(selected);
        self.best_required_brake_torque_nm = selected.evaluation.required_brake_torque_nm;
        self.last_evaluation = selected.evaluation;
        self.verify_failed_windows = 0;
        self.pending_live_reset = true;
        self.last_candidate_label = format!(
            "selected / thr {:.3} / MBT {:.1} / {:+.1} Nm",
            selected.controls.throttle_cmd,
            selected.controls.ignition_timing_deg,
            selected.evaluation.required_brake_torque_nm
        );
    }

    fn advance_verification(&mut self) {
        if self.verify_measure_elapsed_s < STARTUP_FIT_VERIFY_MEASURE_S
            || self.verify_sample_count == 0
        {
            return;
        }
        let avg_rpm = self.verify_rpm_sum / self.verify_sample_count as f64;
        let avg_net_torque_nm = self.verify_net_torque_sum_nm / self.verify_sample_count as f64;
        let avg_load_torque_nm = self.verify_load_torque_sum_nm / self.verify_sample_count as f64;
        self.last_evaluation.avg_rpm = avg_rpm;
        self.last_evaluation.avg_net_torque_nm = avg_net_torque_nm;
        self.last_evaluation.required_brake_torque_nm = avg_load_torque_nm;
        let stable = avg_rpm > STARTUP_FIT_VERIFY_STALL_RPM
            && (avg_rpm - self.target_rpm).abs() <= STARTUP_FIT_VERIFY_RPM_ERR
            && avg_net_torque_nm.abs() <= STARTUP_FIT_VERIFY_NET_TORQUE_NM;
        if stable {
            self.stable_windows = self.stable_windows.saturating_add(1);
            self.verify_failed_windows = 0;
        } else {
            self.stable_windows = 0;
            self.verify_failed_windows = self.verify_failed_windows.saturating_add(1);
        }
        self.reset_verify_window();
        let stalled = avg_rpm <= STARTUP_FIT_VERIFY_STALL_RPM;
        if stalled || self.verify_failed_windows >= STARTUP_FIT_VERIFY_MAX_FAILED_WINDOWS {
            if self.retry_release_candidate() {
                return;
            }
        }
        if self.stable_windows >= STARTUP_FIT_REQUIRED_STABLE_WINDOWS {
            self.finish_ready(false);
        }
    }

    fn finish_ready(&mut self, timed_out: bool) {
        self.phase = StartupFitPhase::Ready;
        self.timed_out = timed_out;
        self.current_controls = self.best_controls;
        if timed_out {
            self.last_candidate_label = format!(
                "{} cap reached / best candidate applied",
                startup_fit_wall_limit_label()
            );
        }
    }

    fn reset_verify_window(&mut self) {
        self.verify_elapsed_s = 0.0;
        self.verify_measure_elapsed_s = 0.0;
        self.verify_sample_count = 0;
        self.verify_rpm_sum = 0.0;
        self.verify_net_torque_sum_nm = 0.0;
        self.verify_load_torque_sum_nm = 0.0;
    }

    fn retry_release_candidate(&mut self) -> bool {
        if !self
            .rejected_release_controls
            .iter()
            .any(|candidate| same_controls(*candidate, self.current_controls))
        {
            self.rejected_release_controls.push(self.current_controls);
        }
        let throttle_bests: Vec<CandidateResult> = self
            .throttle_tracks
            .iter()
            .filter_map(|track| track.best())
            .collect();
        let Some(selected) = select_candidate_for_release(
            &throttle_bests,
            self.target_rpm,
            &self.rejected_release_controls,
        ) else {
            self.last_candidate_label =
                "verify failed / no alternate release candidate remains".to_owned();
            return false;
        };
        self.best_controls = selected.controls;
        self.current_controls = selected.controls;
        self.best_result = Some(selected);
        self.best_required_brake_torque_nm = selected.evaluation.required_brake_torque_nm;
        self.last_evaluation = selected.evaluation;
        self.verify_failed_windows = 0;
        self.pending_live_reset = true;
        self.last_candidate_label = format!(
            "verify retry / thr {:.3} / MBT {:.1} / {:+.1} Nm",
            selected.controls.throttle_cmd,
            selected.controls.ignition_timing_deg,
            selected.evaluation.required_brake_torque_nm
        );
        true
    }
}

#[cfg_attr(not(test), allow(dead_code))]
#[derive(Debug, Clone, Copy)]
pub(super) struct StartupFitWorkloadContract {
    pub(super) target_wall_time_s: f64,
    pub(super) max_wall_time_s: Option<f64>,
    pub(super) throttle_bins: usize,
    pub(super) coarse_ignition_bins: usize,
    pub(super) refine_ignition_bins: usize,
    pub(super) wot_curve_speed_points: usize,
    pub(super) coarse_candidates: usize,
    pub(super) refine_candidates: usize,
    pub(super) max_cycles_per_candidate: usize,
    pub(super) max_steps_per_cycle: usize,
    pub(super) required_stable_windows: usize,
    pub(super) verify_settle_s: f64,
    pub(super) verify_measure_s: f64,
    pub(super) worker_accuracy_target_deg_per_step: f64,
    pub(super) worker_accuracy_dt_max_s: f64,
}

#[cfg_attr(not(test), allow(dead_code))]
impl StartupFitWorkloadContract {
    pub(super) fn total_candidates(self) -> usize {
        self.coarse_candidates + self.refine_candidates
    }

    pub(super) fn worst_case_rkf_steps(self) -> usize {
        self.total_candidates() * self.max_cycles_per_candidate * self.max_steps_per_cycle
    }

    pub(super) fn worst_case_verify_simulated_s(self) -> f64 {
        self.required_stable_windows as f64 * (self.verify_settle_s + self.verify_measure_s)
    }
}

#[cfg_attr(not(test), allow(dead_code))]
pub(super) fn startup_fit_workload_contract() -> StartupFitWorkloadContract {
    StartupFitWorkloadContract {
        target_wall_time_s: STARTUP_FIT_TARGET_WALL_TIME_S,
        max_wall_time_s: STARTUP_FIT_MAX_WALL_TIME_S,
        throttle_bins: STARTUP_FIT_THROTTLE_BIN_COUNT,
        coarse_ignition_bins: STARTUP_FIT_COARSE_IGNITION_BIN_COUNT,
        refine_ignition_bins: STARTUP_FIT_REFINE_IGNITION_BIN_COUNT,
        wot_curve_speed_points: STARTUP_FIT_WOT_SWEEP_RPMS.len(),
        coarse_candidates: STARTUP_FIT_THROTTLE_BIN_COUNT * STARTUP_FIT_COARSE_IGNITION_BIN_COUNT,
        refine_candidates: STARTUP_FIT_THROTTLE_BIN_COUNT * STARTUP_FIT_REFINE_IGNITION_BIN_COUNT,
        max_cycles_per_candidate: STARTUP_FIT_MAX_CYCLES_PER_CANDIDATE,
        max_steps_per_cycle: STARTUP_FIT_MAX_STEPS_PER_CYCLE,
        required_stable_windows: STARTUP_FIT_REQUIRED_STABLE_WINDOWS,
        verify_settle_s: STARTUP_FIT_VERIFY_SETTLE_S,
        verify_measure_s: STARTUP_FIT_VERIFY_MEASURE_S,
        worker_accuracy_target_deg_per_step: STARTUP_FIT_WORKER_ACCURACY_TARGET_DEG_PER_STEP,
        worker_accuracy_dt_max_s: STARTUP_FIT_WORKER_ACCURACY_DT_MAX_S,
    }
}

pub(super) fn startup_fit_control_bounds(sim: &Simulator) -> StartupFitControlBounds {
    StartupFitControlBounds {
        throttle_min: 0.08,
        throttle_max: 1.0,
        ignition_min_deg: sim.model.mbt_min_deg,
        ignition_max_deg: sim.model.mbt_max_deg,
        vvt_default_deg: 0.0,
    }
}

pub(super) fn fit_speed_hold_target_torque_nm(
    reference_brake_torque_nm: f64,
    actual_rpm: f64,
    target_rpm: f64,
) -> f64 {
    (reference_brake_torque_nm
        + STARTUP_FIT_LOAD_TORQUE_GAIN_NM_PER_RPM * (actual_rpm - target_rpm))
        .max(0.0)
}

fn apply_fit_controls_to_simulator(
    sim: &mut Simulator,
    phase: StartupFitPhase,
    target_rpm: f64,
    reference_brake_torque_nm: f64,
    controls: StartupFitControls,
) {
    sim.control.throttle_cmd = controls.throttle_cmd;
    sim.control.ignition_timing_deg = controls.ignition_timing_deg;
    sim.control.vvt_intake_deg = controls.vvt_intake_deg;
    sim.control.vvt_exhaust_deg = controls.vvt_exhaust_deg;
    if phase == StartupFitPhase::Verifying {
        let actual_rpm = rad_s_to_rpm(sim.state.omega_rad_s.max(0.0));
        let target_torque_nm =
            fit_speed_hold_target_torque_nm(reference_brake_torque_nm, actual_rpm, target_rpm);
        sim.control.load_cmd = external_load_command_for_torque_nm(
            target_torque_nm,
            sim.state.omega_rad_s,
            &sim.model.external_load,
        );
    } else {
        sim.control.load_cmd = controls.load_cmd;
    }
}

pub(super) fn apply_startup_fit_state_controls(sim: &mut Simulator, fit: &StartupFitState) {
    apply_fit_controls_to_simulator(
        sim,
        fit.phase(),
        fit.target_rpm,
        fit.selected_required_brake_torque_nm(),
        fit.applied_controls(),
    );
}

pub(super) fn apply_startup_fit_status_controls(sim: &mut Simulator, status: &StartupFitStatus) {
    apply_fit_controls_to_simulator(
        sim,
        status.phase,
        status.target_rpm,
        status
            .release_required_brake_torque_nm
            .max(status.best_required_brake_torque_nm)
            .max(status.required_brake_torque_nm),
        StartupFitControls {
            throttle_cmd: status.throttle_cmd,
            ignition_timing_deg: status.ignition_timing_deg,
            vvt_intake_deg: status.vvt_intake_deg,
            vvt_exhaust_deg: status.vvt_exhaust_deg,
            load_cmd: status.load_cmd,
        },
    );
}

enum StartupFitWorkerMessage {
    Progress(StartupFitStatus),
    Completed(StartupFitArtifactSnapshot),
    Failed(String),
}

pub(super) enum StartupFitWorkerPoll {
    Running,
    Completed(StartupFitArtifactSnapshot),
    Failed(String),
}

pub(super) struct StartupFitWorkerHandle {
    receiver: Receiver<StartupFitWorkerMessage>,
    latest_status: StartupFitStatus,
    join_handle: Option<JoinHandle<()>>,
}

impl StartupFitWorkerHandle {
    pub(super) fn spawn(
        config: AppConfig,
        started_sim_time_s: f64,
        target_rpm: f64,
        initial_controls: StartupFitControls,
    ) -> Self {
        let initial_status = StartupFitState::new(started_sim_time_s, target_rpm, initial_controls)
            .snapshot(started_sim_time_s);
        let (tx, receiver) = mpsc::channel();
        let join_handle = thread::spawn(move || {
            if let Err(err) = run_startup_fit_worker(
                config,
                started_sim_time_s,
                target_rpm,
                initial_controls,
                tx.clone(),
            ) {
                let _ = tx.send(StartupFitWorkerMessage::Failed(err));
            }
        });
        Self {
            receiver,
            latest_status: initial_status,
            join_handle: Some(join_handle),
        }
    }

    pub(super) fn latest_status(&self) -> &StartupFitStatus {
        &self.latest_status
    }

    pub(super) fn poll(&mut self) -> StartupFitWorkerPoll {
        let mut outcome = StartupFitWorkerPoll::Running;
        loop {
            match self.receiver.try_recv() {
                Ok(StartupFitWorkerMessage::Progress(status)) => {
                    self.latest_status = status;
                }
                Ok(StartupFitWorkerMessage::Completed(snapshot)) => {
                    self.latest_status =
                        StartupFitState::from_snapshot(0.0, &snapshot).snapshot(0.0);
                    if let Some(join_handle) = self.join_handle.take() {
                        let _ = join_handle.join();
                    }
                    outcome = StartupFitWorkerPoll::Completed(snapshot);
                }
                Ok(StartupFitWorkerMessage::Failed(message)) => {
                    if let Some(join_handle) = self.join_handle.take() {
                        let _ = join_handle.join();
                    }
                    outcome = StartupFitWorkerPoll::Failed(message);
                }
                Err(TryRecvError::Empty) => return outcome,
                Err(TryRecvError::Disconnected) => {
                    return match outcome {
                        StartupFitWorkerPoll::Running => StartupFitWorkerPoll::Failed(
                            "startup fit worker disconnected unexpectedly".to_owned(),
                        ),
                        _ => outcome,
                    };
                }
            }
        }
    }
}

#[cfg_attr(not(test), allow(dead_code))]
#[derive(Debug, Clone)]
pub(super) struct StartupFitOfflineOutcome {
    pub(super) snapshot: StartupFitArtifactSnapshot,
    pub(super) final_status: StartupFitStatus,
}

#[cfg_attr(not(test), allow(dead_code))]
pub(super) fn run_startup_fit_offline<F>(
    mut config: AppConfig,
    started_sim_time_s: f64,
    target_rpm: f64,
    initial_controls: StartupFitControls,
    mut publish_status: F,
) -> Result<StartupFitOfflineOutcome, String>
where
    F: FnMut(StartupFitStatus) -> Result<(), String>,
{
    config.numerics.accuracy_target_deg_per_step = STARTUP_FIT_WORKER_ACCURACY_TARGET_DEG_PER_STEP;
    config.numerics.accuracy_dt_max_s = config
        .numerics
        .accuracy_dt_max_s
        .max(STARTUP_FIT_WORKER_ACCURACY_DT_MAX_S);
    config.numerics.rpm_link_dt_min_floor_s = config
        .numerics
        .rpm_link_dt_min_floor_s
        .max(STARTUP_FIT_WORKER_DT_MIN_FLOOR_S);

    let mut sim = Simulator::new(&config);
    sim.model.external_load.mode = ExternalLoadMode::BrakeMap;
    sim.control.spark_cmd = true;
    sim.control.fuel_cmd = true;
    sim.control.vvt_intake_deg = initial_controls.vvt_intake_deg;
    sim.control.vvt_exhaust_deg = initial_controls.vvt_exhaust_deg;
    sim.control.load_cmd = 0.0;
    sim.seed_operating_point(
        target_rpm,
        initial_controls.throttle_cmd,
        initial_controls.ignition_timing_deg,
    );

    let bounds = startup_fit_control_bounds(&sim);
    let initial_dt_s = accuracy_priority_dt(target_rpm, &sim.numerics).clamp(
        config.numerics.rpm_link_dt_min_floor_s,
        config.numerics.accuracy_dt_max_s,
    );
    sim.step(initial_dt_s);
    let mut simulated_time_s = started_sim_time_s + initial_dt_s;
    let mut fit = StartupFitState::new(started_sim_time_s, target_rpm, initial_controls);
    let mut last_published_at =
        Instant::now() - Duration::from_secs_f64(STARTUP_FIT_WORKER_STATUS_INTERVAL_S);
    publish_status(fit.snapshot(simulated_time_s))?;

    while fit.is_active() {
        let mut simulated_batch_s = 0.0;
        while simulated_batch_s < STARTUP_FIT_WORKER_SIMULATED_BATCH_S {
            apply_startup_fit_state_controls(&mut sim, &fit);
            let rpm = rad_s_to_rpm(sim.state.omega_rad_s.max(0.0));
            let dt_step = accuracy_priority_dt(rpm, &sim.numerics)
                .clamp(
                    config.numerics.rpm_link_dt_min_floor_s,
                    config.numerics.accuracy_dt_max_s,
                )
                .min(STARTUP_FIT_WORKER_SIMULATED_BATCH_S - simulated_batch_s);
            let latest = sim.step(dt_step);
            simulated_time_s += dt_step;
            simulated_batch_s += dt_step;
            fit.record_live_sample(
                dt_step,
                latest.rpm,
                latest.torque_net_nm,
                latest.torque_load_nm,
            );
        }

        fit.update(simulated_time_s, &sim, bounds, None);
        if let Some(seed) = fit.take_live_reset_seed() {
            sim = seed;
            sim.step(config.numerics.rpm_link_dt_min_floor_s.max(1.0e-4));
            simulated_time_s += config.numerics.rpm_link_dt_min_floor_s.max(1.0e-4);
        }

        if last_published_at.elapsed().as_secs_f64() >= STARTUP_FIT_WORKER_STATUS_INTERVAL_S {
            publish_status(fit.snapshot(simulated_time_s))?;
            last_published_at = Instant::now();
        }
    }

    fit.set_wot_torque_curve(build_wot_torque_curve(&sim, bounds));
    let snapshot = fit
        .artifact()
        .ok_or_else(|| "startup fit worker finished without artifact snapshot".to_owned())?;
    let final_status = fit.snapshot(simulated_time_s);
    publish_status(final_status.clone())?;
    Ok(StartupFitOfflineOutcome {
        snapshot,
        final_status,
    })
}

fn run_startup_fit_worker(
    config: AppConfig,
    started_sim_time_s: f64,
    target_rpm: f64,
    initial_controls: StartupFitControls,
    tx: mpsc::Sender<StartupFitWorkerMessage>,
) -> Result<(), String> {
    let outcome = run_startup_fit_offline(
        config,
        started_sim_time_s,
        target_rpm,
        initial_controls,
        |status| {
            tx.send(StartupFitWorkerMessage::Progress(status))
                .map_err(|err| format!("failed to publish startup-fit worker status: {err}"))
        },
    )?;
    tx.send(StartupFitWorkerMessage::Completed(outcome.snapshot))
        .map_err(|err| format!("failed to publish startup-fit completion: {err}"))?;
    Ok(())
}

fn build_wot_torque_curve(
    seed: &Simulator,
    bounds: StartupFitControlBounds,
) -> Vec<StartupFitWotTorquePoint> {
    let mut sim = seed.clone();
    sim.model.external_load.mode = ExternalLoadMode::BrakeMap;
    sim.control.spark_cmd = true;
    sim.control.fuel_cmd = true;
    sim.control.vvt_intake_deg = bounds.vvt_default_deg;
    sim.control.vvt_exhaust_deg = bounds.vvt_default_deg;
    sim.control.load_cmd = 0.0;
    let mut previous_available_brake_torque_nm = 0.0;
    let raw_curve: Vec<StartupFitWotTorquePoint> = STARTUP_FIT_WOT_SWEEP_RPMS
        .iter()
        .copied()
        .map(|target_rpm| {
            let point = solve_wot_torque_curve_point(
                &mut sim,
                bounds,
                target_rpm,
                previous_available_brake_torque_nm,
            );
            previous_available_brake_torque_nm = point.available_brake_torque_nm;
            point
        })
        .collect();

    smooth_wot_torque_curve(&raw_curve)
}

#[derive(Debug, Clone, Copy, Default)]
struct WotSweepWindowAverage {
    elapsed_s: f64,
    rpm_dt_sum: f64,
    net_torque_dt_sum: f64,
    load_torque_dt_sum: f64,
}

impl WotSweepWindowAverage {
    fn accumulate(&mut self, rpm: f64, net_torque_nm: f64, load_torque_nm: f64, dt_s: f64) {
        let dt_s = dt_s.max(0.0);
        self.elapsed_s += dt_s;
        self.rpm_dt_sum += rpm * dt_s;
        self.net_torque_dt_sum += net_torque_nm * dt_s;
        self.load_torque_dt_sum += load_torque_nm * dt_s;
    }

    fn mean_rpm(self) -> f64 {
        self.rpm_dt_sum / self.elapsed_s.max(f64::EPSILON)
    }

    fn mean_net_torque_nm(self) -> f64 {
        self.net_torque_dt_sum / self.elapsed_s.max(f64::EPSILON)
    }

    fn mean_load_torque_nm(self) -> f64 {
        self.load_torque_dt_sum / self.elapsed_s.max(f64::EPSILON)
    }
}

fn solve_wot_torque_curve_point(
    sim: &mut Simulator,
    bounds: StartupFitControlBounds,
    target_rpm: f64,
    initial_brake_torque_guess_nm: f64,
) -> StartupFitWotTorquePoint {
    let ignition_timing_deg =
        estimate_mbt_deg(&sim.model, target_rpm, sim.model.load_max.max(0.95))
            .clamp(bounds.ignition_min_deg, bounds.ignition_max_deg);
    sim.control.throttle_cmd = bounds.throttle_max;
    sim.control.ignition_timing_deg = ignition_timing_deg;
    sim.control.vvt_intake_deg = bounds.vvt_default_deg;
    sim.control.vvt_exhaust_deg = bounds.vvt_default_deg;
    sim.control.spark_cmd = true;
    sim.control.fuel_cmd = true;
    sim.seed_operating_point(target_rpm, bounds.throttle_max, ignition_timing_deg);

    let initial_unloaded = sim.locked_cycle_average(sim.state, 96);
    let mut reference_brake_torque_nm = finite_f64(
        initial_brake_torque_guess_nm,
        (initial_unloaded.torque_net_nm + initial_unloaded.torque_load_nm).max(0.0),
    )
    .max(0.0);
    let mut last_available_brake_torque_nm = reference_brake_torque_nm;

    for _ in 0..STARTUP_FIT_WOT_SWEEP_MAX_WINDOWS {
        simulate_wot_speed_hold_window(
            sim,
            target_rpm,
            reference_brake_torque_nm,
            STARTUP_FIT_WOT_SWEEP_SETTLE_S,
        );
        let measurement = simulate_wot_speed_hold_window(
            sim,
            target_rpm,
            reference_brake_torque_nm,
            STARTUP_FIT_WOT_SWEEP_MEASURE_S,
        );
        let measured_available_brake_torque_nm =
            (measurement.mean_load_torque_nm() + measurement.mean_net_torque_nm()).max(0.0);
        last_available_brake_torque_nm = measured_available_brake_torque_nm;
        reference_brake_torque_nm = lerp(
            reference_brake_torque_nm,
            measured_available_brake_torque_nm,
            0.65,
        )
        .max(0.0);

        if (measurement.mean_rpm() - target_rpm).abs() <= STARTUP_FIT_WOT_SWEEP_RPM_ERR
            && measurement.mean_net_torque_nm().abs() <= STARTUP_FIT_WOT_SWEEP_NET_TORQUE_NM
        {
            reference_brake_torque_nm = measured_available_brake_torque_nm;
            break;
        }
    }

    StartupFitWotTorquePoint {
        engine_speed_rpm: target_rpm,
        available_brake_torque_nm: reference_brake_torque_nm
            .max(last_available_brake_torque_nm)
            .max(0.0),
        ignition_timing_deg,
    }
}

fn simulate_wot_speed_hold_window(
    sim: &mut Simulator,
    target_rpm: f64,
    reference_brake_torque_nm: f64,
    duration_s: f64,
) -> WotSweepWindowAverage {
    let mut average = WotSweepWindowAverage::default();
    let mut elapsed_s = 0.0;

    while elapsed_s < duration_s {
        let actual_rpm = rad_s_to_rpm(sim.state.omega_rad_s.max(0.0));
        let target_torque_nm =
            fit_speed_hold_target_torque_nm(reference_brake_torque_nm, actual_rpm, target_rpm);
        sim.control.load_cmd = external_load_command_for_torque_nm(
            target_torque_nm,
            sim.state.omega_rad_s,
            &sim.model.external_load,
        );
        let dt_s = accuracy_priority_dt(actual_rpm.max(target_rpm.min(600.0)), &sim.numerics)
            .clamp(
                STARTUP_FIT_WORKER_DT_MIN_FLOOR_S,
                sim.numerics.accuracy_dt_max_s,
            )
            .min((duration_s - elapsed_s).max(STARTUP_FIT_WORKER_DT_MIN_FLOOR_S));
        sim.step(dt_s);
        let sample = sim.sample_fit_state(sim.state);
        average.accumulate(
            sample.rpm,
            sample.torque_net_nm,
            sample.torque_load_nm,
            dt_s,
        );
        elapsed_s += dt_s;
    }

    average
}

fn smooth_wot_torque_curve(curve: &[StartupFitWotTorquePoint]) -> Vec<StartupFitWotTorquePoint> {
    if curve.len() <= 2 {
        return curve.to_vec();
    }

    let mut smoothed = curve.to_vec();
    for _ in 0..STARTUP_FIT_WOT_CURVE_SMOOTH_PASSES {
        let previous = smoothed.clone();
        for idx in 1..previous.len() - 1 {
            let neighborhood_torque_nm = 0.25 * previous[idx - 1].available_brake_torque_nm
                + 0.50 * previous[idx].available_brake_torque_nm
                + 0.25 * previous[idx + 1].available_brake_torque_nm;
            smoothed[idx].available_brake_torque_nm = lerp(
                previous[idx].available_brake_torque_nm,
                neighborhood_torque_nm,
                STARTUP_FIT_WOT_CURVE_SMOOTH_BLEND,
            )
            .max(0.0);
        }
    }
    smoothed
}

fn time_to_next_ignition_event_s(state: EngineState, burn_start_deg: f64) -> f64 {
    let current_deg = state.theta_rad.to_degrees().rem_euclid(720.0);
    let mut delta_deg = (burn_start_deg - current_deg).rem_euclid(720.0);
    if delta_deg <= 1.0e-6 {
        delta_deg = 720.0;
    }
    let omega = state.omega_rad_s.abs().max(1.0e-6);
    (delta_deg.to_radians() / omega * 0.98).max(STARTUP_FIT_MIN_DT_S)
}

fn rkf45_next_dt(dt_s: f64, error_norm: f64) -> f64 {
    let safe_error = finite_f64(error_norm, 1.0).max(1.0e-9);
    let factor = (STARTUP_FIT_RKF_SAFETY * safe_error.powf(-0.2))
        .clamp(STARTUP_FIT_RKF_MIN_FACTOR, STARTUP_FIT_RKF_MAX_FACTOR);
    dt_s * factor
}

fn linspace(min: f64, max: f64, count: usize) -> Vec<f64> {
    let count = count.max(1);
    if count == 1 {
        return vec![min];
    }
    let step = (max - min) / (count - 1) as f64;
    (0..count).map(|idx| min + step * idx as f64).collect()
}

fn select_candidate_for_release(
    throttle_bests: &[CandidateResult],
    target_rpm: f64,
    rejected_release_controls: &[StartupFitControls],
) -> Option<CandidateResult> {
    let filtered: Vec<CandidateResult> = throttle_bests
        .iter()
        .copied()
        .filter(|result| {
            !rejected_release_controls
                .iter()
                .any(|rejected| same_controls(*rejected, result.controls))
        })
        .collect();
    if filtered.is_empty() {
        return None;
    }

    if let Some(selected) = filtered
        .iter()
        .copied()
        .filter(|result| result.evaluation.converged)
        .min_by(|a, b| {
            a.controls
                .throttle_cmd
                .total_cmp(&b.controls.throttle_cmd)
                .then_with(|| {
                    b.ranking_score(target_rpm)
                        .total_cmp(&a.ranking_score(target_rpm))
                })
        })
    {
        return Some(selected);
    }

    let top_score = filtered
        .iter()
        .map(|result| result.ranking_score(target_rpm))
        .fold(f64::NEG_INFINITY, f64::max);
    filtered
        .iter()
        .copied()
        .filter(|result| {
            top_score - result.ranking_score(target_rpm) <= STARTUP_FIT_SELECTION_SCORE_MARGIN
        })
        .min_by(|a, b| {
            a.controls
                .throttle_cmd
                .total_cmp(&b.controls.throttle_cmd)
                .then_with(|| {
                    (a.evaluation.avg_rpm - target_rpm)
                        .abs()
                        .total_cmp(&(b.evaluation.avg_rpm - target_rpm).abs())
                })
                .then_with(|| {
                    b.ranking_score(target_rpm)
                        .total_cmp(&a.ranking_score(target_rpm))
                })
        })
        .or_else(|| {
            filtered.iter().copied().max_by(|a, b| {
                a.ranking_score(target_rpm)
                    .total_cmp(&b.ranking_score(target_rpm))
            })
        })
        .or_else(|| filtered.first().copied())
}

fn startup_fit_deadline(started_wall_at: Instant) -> Option<Instant> {
    STARTUP_FIT_MAX_WALL_TIME_S.map(|limit_s| started_wall_at + Duration::from_secs_f64(limit_s))
}

fn deadline_reached(deadline: Option<Instant>) -> bool {
    deadline
        .map(|value| Instant::now() >= value)
        .unwrap_or(false)
}

fn same_controls(a: StartupFitControls, b: StartupFitControls) -> bool {
    const EPS: f64 = 1.0e-9;
    (a.throttle_cmd - b.throttle_cmd).abs() <= EPS
        && (a.ignition_timing_deg - b.ignition_timing_deg).abs() <= EPS
        && (a.vvt_intake_deg - b.vvt_intake_deg).abs() <= EPS
        && (a.vvt_exhaust_deg - b.vvt_exhaust_deg).abs() <= EPS
        && (a.load_cmd - b.load_cmd).abs() <= EPS
}

fn torque_curve_point(result: CandidateResult) -> StartupFitTorqueCurvePoint {
    StartupFitTorqueCurvePoint {
        throttle_cmd: result.controls.throttle_cmd,
        required_brake_torque_nm: result.evaluation.required_brake_torque_nm,
    }
}

fn record_candidate_point(points: &mut Vec<[f64; 2]>, evaluation: CandidateEvaluation) {
    let point = [evaluation.avg_rpm, evaluation.required_brake_torque_nm];
    if point[0].is_finite() && point[1].is_finite() {
        points.push(point);
    }
}

fn finite_f64(value: f64, fallback: f64) -> f64 {
    if value.is_finite() { value } else { fallback }
}

fn lerp(a: f64, b: f64, t: f64) -> f64 {
    a + (b - a) * t.clamp(0.0, 1.0)
}

#[cfg(test)]
mod tests {
    use std::path::PathBuf;
    use std::time::Instant;

    use crate::config::{AppConfig, load_config};
    use crate::simulator::Simulator;

    use super::{
        CandidateEvaluation, CandidateResult, STARTUP_FIT_INITIAL_THROTTLE, STARTUP_FIT_TARGET_RPM,
        STARTUP_FIT_TORQUE_MARGIN_FROM_BEST_NM, SearchStage, StartupFitControlBounds,
        StartupFitControls, StartupFitPhase, StartupFitState, StartupFitWotTorquePoint,
        ThrottleTrack, linspace, rkf45_next_dt, run_startup_fit_offline,
        select_candidate_for_release, smooth_wot_torque_curve, startup_fit_wall_limit_label,
        startup_fit_workload_contract,
    };

    fn controls() -> StartupFitControls {
        StartupFitControls {
            throttle_cmd: STARTUP_FIT_INITIAL_THROTTLE,
            ignition_timing_deg: 18.0,
            vvt_intake_deg: 0.0,
            vvt_exhaust_deg: 0.0,
            load_cmd: 0.0,
        }
    }

    fn bounds() -> StartupFitControlBounds {
        StartupFitControlBounds {
            throttle_min: 0.08,
            throttle_max: 1.0,
            ignition_min_deg: -10.0,
            ignition_max_deg: 50.0,
            vvt_default_deg: 0.0,
        }
    }

    #[test]
    fn linspace_includes_endpoints() {
        let values = linspace(0.1, 0.9, 8);

        assert_eq!(values.len(), 8);
        assert!((values[0] - 0.1).abs() < 1.0e-12);
        assert!((values[7] - 0.9).abs() < 1.0e-12);
    }

    #[test]
    fn rkf45_dt_shrinks_when_error_is_large() {
        let small = rkf45_next_dt(1.0e-3, 8.0);
        let large = rkf45_next_dt(1.0e-3, 0.1);

        assert!(small < 1.0e-3);
        assert!(large > 1.0e-3);
    }

    #[test]
    fn state_enters_optimizer_after_priming() {
        let cfg = AppConfig::default();
        let sim = Simulator::new(&cfg);
        let mut state = StartupFitState::new(0.0, STARTUP_FIT_TARGET_RPM, controls());

        state.update(0.31, &sim, bounds(), Some(Instant::now()));
        let snapshot = state.snapshot(0.31);

        assert_eq!(snapshot.phase, StartupFitPhase::Optimizing);
        assert!(matches!(state.search_stage, SearchStage::Coarse));
    }

    #[test]
    fn selected_margin_constant_is_positive() {
        assert!(STARTUP_FIT_TORQUE_MARGIN_FROM_BEST_NM > 0.0);
    }

    #[test]
    fn smoothing_wot_curve_reduces_second_difference_roughness() {
        let raw = vec![
            StartupFitWotTorquePoint {
                engine_speed_rpm: 1_000.0,
                available_brake_torque_nm: 170.0,
                ignition_timing_deg: 18.0,
            },
            StartupFitWotTorquePoint {
                engine_speed_rpm: 1_500.0,
                available_brake_torque_nm: 180.0,
                ignition_timing_deg: 19.0,
            },
            StartupFitWotTorquePoint {
                engine_speed_rpm: 2_000.0,
                available_brake_torque_nm: 188.0,
                ignition_timing_deg: 20.0,
            },
            StartupFitWotTorquePoint {
                engine_speed_rpm: 2_500.0,
                available_brake_torque_nm: 197.0,
                ignition_timing_deg: 21.0,
            },
            StartupFitWotTorquePoint {
                engine_speed_rpm: 3_000.0,
                available_brake_torque_nm: 216.0,
                ignition_timing_deg: 22.0,
            },
            StartupFitWotTorquePoint {
                engine_speed_rpm: 3_500.0,
                available_brake_torque_nm: 202.0,
                ignition_timing_deg: 23.0,
            },
            StartupFitWotTorquePoint {
                engine_speed_rpm: 4_000.0,
                available_brake_torque_nm: 235.0,
                ignition_timing_deg: 24.0,
            },
            StartupFitWotTorquePoint {
                engine_speed_rpm: 4_500.0,
                available_brake_torque_nm: 210.0,
                ignition_timing_deg: 25.0,
            },
            StartupFitWotTorquePoint {
                engine_speed_rpm: 5_000.0,
                available_brake_torque_nm: 180.0,
                ignition_timing_deg: 26.0,
            },
        ];

        let roughness = |curve: &[StartupFitWotTorquePoint]| -> f64 {
            curve
                .windows(3)
                .map(|window| {
                    (window[2].available_brake_torque_nm
                        - 2.0 * window[1].available_brake_torque_nm
                        + window[0].available_brake_torque_nm)
                        .abs()
                })
                .sum()
        };

        let smoothed = smooth_wot_torque_curve(&raw);
        assert!(roughness(&smoothed) < roughness(&raw));
    }

    #[test]
    fn update_best_result_prefers_fit_quality_over_brake_only() {
        let mut state = StartupFitState::new(0.0, STARTUP_FIT_TARGET_RPM, controls());
        state.search_stage = SearchStage::Coarse;
        state.throttle_tracks = vec![ThrottleTrack {
            throttle_cmd: 0.24,
            coarse_best: None,
            refined_best: None,
        }];

        let well_matched = CandidateResult {
            controls: StartupFitControls {
                throttle_cmd: 0.24,
                ..controls()
            },
            evaluation: CandidateEvaluation {
                avg_rpm: 2_005.0,
                avg_net_torque_nm: 0.8,
                required_brake_torque_nm: 9.0,
                load_cmd: 0.32,
                periodic_error_norm: 0.03,
                converged: true,
            },
        };
        let runaway = CandidateResult {
            controls: StartupFitControls {
                throttle_cmd: 0.62,
                ..controls()
            },
            evaluation: CandidateEvaluation {
                avg_rpm: 2_820.0,
                avg_net_torque_nm: 132.0,
                required_brake_torque_nm: 17.0,
                load_cmd: 1.0,
                periodic_error_norm: 0.95,
                converged: false,
            },
        };

        state.update_best_result(0, well_matched);
        state.update_best_result(0, runaway);

        assert!((state.best_controls.throttle_cmd - 0.24).abs() < 1.0e-12);
        assert!((state.best_required_brake_torque_nm - 9.0).abs() < 1.0e-12);
    }

    #[test]
    fn final_selection_prefers_lowest_throttle_converged_candidate() {
        let mut state = StartupFitState::new(0.0, STARTUP_FIT_TARGET_RPM, controls());
        state.throttle_tracks = vec![
            ThrottleTrack {
                throttle_cmd: 0.22,
                coarse_best: Some(CandidateResult {
                    controls: StartupFitControls {
                        throttle_cmd: 0.22,
                        ..controls()
                    },
                    evaluation: CandidateEvaluation {
                        avg_rpm: 1_998.0,
                        avg_net_torque_nm: 0.4,
                        required_brake_torque_nm: 8.5,
                        load_cmd: 0.34,
                        periodic_error_norm: 0.02,
                        converged: true,
                    },
                }),
                refined_best: None,
            },
            ThrottleTrack {
                throttle_cmd: 0.46,
                coarse_best: Some(CandidateResult {
                    controls: StartupFitControls {
                        throttle_cmd: 0.46,
                        ..controls()
                    },
                    evaluation: CandidateEvaluation {
                        avg_rpm: 2_002.0,
                        avg_net_torque_nm: 0.2,
                        required_brake_torque_nm: 15.0,
                        load_cmd: 0.58,
                        periodic_error_norm: 0.02,
                        converged: true,
                    },
                }),
                refined_best: None,
            },
            ThrottleTrack {
                throttle_cmd: 0.62,
                coarse_best: Some(CandidateResult {
                    controls: StartupFitControls {
                        throttle_cmd: 0.62,
                        ..controls()
                    },
                    evaluation: CandidateEvaluation {
                        avg_rpm: 2_780.0,
                        avg_net_torque_nm: 118.0,
                        required_brake_torque_nm: 16.2,
                        load_cmd: 1.0,
                        periodic_error_norm: 1.02,
                        converged: false,
                    },
                }),
                refined_best: None,
            },
        ];

        state.select_final_candidate();

        assert!((state.best_controls.throttle_cmd - 0.22).abs() < 1.0e-12);
        assert!((state.best_required_brake_torque_nm - 8.5).abs() < 1.0e-12);
    }

    #[test]
    fn wall_limit_label_reports_target_cap() {
        assert_eq!(startup_fit_wall_limit_label(), "10 min");
    }

    #[test]
    fn workload_contract_matches_current_fit_budget() {
        let contract = startup_fit_workload_contract();

        assert_eq!(contract.max_wall_time_s, Some(contract.target_wall_time_s));
        assert_eq!(
            contract.coarse_candidates,
            contract.throttle_bins * contract.coarse_ignition_bins
        );
        assert_eq!(
            contract.refine_candidates,
            contract.throttle_bins * contract.refine_ignition_bins
        );
        assert_eq!(contract.throttle_bins, 1);
        assert_eq!(contract.total_candidates(), 16);
        assert_eq!(contract.wot_curve_speed_points, 13);
        assert_eq!(contract.max_cycles_per_candidate, 6);
        assert_eq!(contract.max_steps_per_cycle, 1_200);
        assert_eq!(contract.worst_case_rkf_steps(), 115_200);
        assert!((contract.worker_accuracy_target_deg_per_step - 4.5).abs() < 1.0e-12);
        assert!((contract.worker_accuracy_dt_max_s - 0.0025).abs() < 1.0e-12);
        assert!((contract.worst_case_verify_simulated_s() - 0.78).abs() < 1.0e-12);
    }

    #[test]
    fn snapshot_exposes_per_throttle_torque_curve_points() {
        let mut state = StartupFitState::new(0.0, STARTUP_FIT_TARGET_RPM, controls());
        state.coarse_candidate_points = vec![[1_950.0, 7.8], [2_060.0, 9.4]];
        state.refine_candidate_points = vec![[2_002.0, 8.1]];
        state.throttle_tracks = vec![
            ThrottleTrack {
                throttle_cmd: 0.22,
                coarse_best: Some(CandidateResult {
                    controls: StartupFitControls {
                        throttle_cmd: 0.22,
                        ignition_timing_deg: 18.0,
                        ..controls()
                    },
                    evaluation: CandidateEvaluation {
                        avg_rpm: 1_998.0,
                        avg_net_torque_nm: 0.5,
                        required_brake_torque_nm: 8.4,
                        load_cmd: 0.32,
                        periodic_error_norm: 0.02,
                        converged: true,
                    },
                }),
                refined_best: Some(CandidateResult {
                    controls: StartupFitControls {
                        throttle_cmd: 0.22,
                        ignition_timing_deg: 18.5,
                        ..controls()
                    },
                    evaluation: CandidateEvaluation {
                        avg_rpm: 2_001.0,
                        avg_net_torque_nm: 0.2,
                        required_brake_torque_nm: 8.1,
                        load_cmd: 0.31,
                        periodic_error_norm: 0.01,
                        converged: true,
                    },
                }),
            },
            ThrottleTrack {
                throttle_cmd: 0.34,
                coarse_best: Some(CandidateResult {
                    controls: StartupFitControls {
                        throttle_cmd: 0.34,
                        ignition_timing_deg: 21.0,
                        ..controls()
                    },
                    evaluation: CandidateEvaluation {
                        avg_rpm: 2_015.0,
                        avg_net_torque_nm: 1.0,
                        required_brake_torque_nm: 12.0,
                        load_cmd: 0.40,
                        periodic_error_norm: 0.03,
                        converged: true,
                    },
                }),
                refined_best: None,
            },
        ];

        let snapshot = state.snapshot(0.0);

        assert_eq!(snapshot.coarse_torque_curve.len(), 2);
        assert_eq!(snapshot.refine_torque_curve.len(), 1);
        assert_eq!(snapshot.wot_torque_curve.len(), 1);
        assert_eq!(snapshot.coarse_candidate_points.len(), 2);
        assert_eq!(snapshot.refine_candidate_points.len(), 1);
        assert!((snapshot.coarse_torque_curve[0].throttle_cmd - 0.22).abs() < 1.0e-12);
        assert!((snapshot.refine_torque_curve[0].required_brake_torque_nm - 8.1).abs() < 1.0e-12);
        assert!(
            (snapshot.wot_torque_curve[0].engine_speed_rpm - STARTUP_FIT_TARGET_RPM).abs()
                < 1.0e-12
        );
    }

    #[test]
    fn release_selection_skips_rejected_candidate() {
        let low = CandidateResult {
            controls: StartupFitControls {
                throttle_cmd: 0.22,
                ..controls()
            },
            evaluation: CandidateEvaluation {
                avg_rpm: 2_000.0,
                avg_net_torque_nm: 0.3,
                required_brake_torque_nm: 8.0,
                load_cmd: 0.30,
                periodic_error_norm: 0.02,
                converged: true,
            },
        };
        let high = CandidateResult {
            controls: StartupFitControls {
                throttle_cmd: 0.40,
                ..controls()
            },
            evaluation: CandidateEvaluation {
                avg_rpm: 2_001.0,
                avg_net_torque_nm: 0.2,
                required_brake_torque_nm: 11.0,
                load_cmd: 0.42,
                periodic_error_norm: 0.02,
                converged: true,
            },
        };

        let selected =
            select_candidate_for_release(&[low, high], STARTUP_FIT_TARGET_RPM, &[low.controls])
                .unwrap();

        assert!((selected.controls.throttle_cmd - 0.40).abs() < 1.0e-12);
    }

    #[test]
    #[ignore = "expensive release-mode startup-fit offline contract run"]
    fn startup_fit_offline_reaches_ready_under_wall_time_contract() {
        let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("config")
            .join("sim.yaml");
        let mut cfg = load_config(&path);
        cfg.ui.sync_to_wall_clock = false;
        let contract = startup_fit_workload_contract();

        let outcome =
            run_startup_fit_offline(cfg, 0.0, STARTUP_FIT_TARGET_RPM, controls(), |_| Ok(()))
                .expect("startup-fit offline run succeeds");
        let status = outcome.final_status;

        assert_eq!(status.phase, StartupFitPhase::Ready);
        assert!(!status.active);
        assert!(!status.timed_out, "startup fit hit the wall-time cap");
        assert!(
            status.wall_elapsed_s <= contract.target_wall_time_s,
            "startup fit exceeded its wall-time target: {:.1}s > {:.1}s",
            status.wall_elapsed_s,
            contract.target_wall_time_s
        );
        assert_eq!(status.max_iterations, contract.total_candidates());
        assert!(
            (status.avg_rpm - STARTUP_FIT_TARGET_RPM).abs() <= 24.0,
            "verify average rpm drifted too far: {:.2}",
            status.avg_rpm
        );
        assert!(
            status.avg_net_torque_nm.abs() <= 2.0,
            "verify net torque stayed too large: {:+.3}",
            status.avg_net_torque_nm
        );
        assert_eq!(outcome.snapshot.target_rpm, STARTUP_FIT_TARGET_RPM);
    }
}
