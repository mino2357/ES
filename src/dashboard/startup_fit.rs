use std::time::{Duration, Instant};

use serde::{Deserialize, Serialize};

use super::startup_fit_artifact::{
    StartupFitArtifact, StartupFitArtifactEvaluation, StartupFitArtifactSnapshot,
};
use crate::simulator::{
    EngineState, Simulator, accuracy_priority_dt, external_load_command_for_torque_nm,
    running_state_error_norm, state_error_norm,
};

pub(super) const STARTUP_FIT_TARGET_RPM: f64 = 2_000.0;
pub(super) const STARTUP_FIT_INITIAL_THROTTLE: f64 = 0.22;
pub(super) const STARTUP_FIT_PRIMING_S: f64 = 0.30;
pub(super) const STARTUP_FIT_FAST_FORWARD_FACTOR: f64 = 5.0;
pub(super) const STARTUP_FIT_MAX_WALL_TIME_S: Option<f64> = None;
pub(super) const STARTUP_FIT_REQUIRED_STABLE_WINDOWS: usize = 5;

const STARTUP_FIT_VERIFY_SETTLE_S: f64 = 0.12;
const STARTUP_FIT_VERIFY_MEASURE_S: f64 = 0.30;
const STARTUP_FIT_VERIFY_RPM_ERR: f64 = 18.0;
const STARTUP_FIT_VERIFY_NET_TORQUE_NM: f64 = 1.5;
const STARTUP_FIT_VERIFY_STALL_RPM: f64 = 240.0;
const STARTUP_FIT_VERIFY_MAX_FAILED_WINDOWS: usize = 4;
const STARTUP_FIT_THROTTLE_BIN_COUNT: usize = 16;
const STARTUP_FIT_IGNITION_BIN_COUNT: usize = 16;
const STARTUP_FIT_MAX_CYCLES_PER_CANDIDATE: usize = 12;
const STARTUP_FIT_MIN_CYCLES_PER_CANDIDATE: usize = 3;
const STARTUP_FIT_MAX_STEPS_PER_CYCLE: usize = 3_600;
const STARTUP_FIT_LOAD_TORQUE_GAIN_NM_PER_RPM: f64 = 0.06;
const STARTUP_FIT_TORQUE_MARGIN_FROM_BEST_NM: f64 = 6.0;
const STARTUP_FIT_SELECTION_SCORE_MARGIN: f64 = 8.0;
const STARTUP_FIT_PERIODIC_ERROR_LIMIT: f64 = 0.05;
const STARTUP_FIT_CANDIDATE_RPM_ERR: f64 = 28.0;
const STARTUP_FIT_CANDIDATE_NET_TORQUE_NM: f64 = 2.5;
const STARTUP_FIT_MIN_DT_S: f64 = 2.0e-5;
const STARTUP_FIT_RKF_SAFETY: f64 = 0.90;
const STARTUP_FIT_RKF_MIN_FACTOR: f64 = 0.25;
const STARTUP_FIT_RKF_MAX_FACTOR: f64 = 2.20;

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
                "evaluating throttle bins, refining local MBT spark, and solving required brake torque with finite-cycle iteration"
            }
            Self::Verifying => {
                "applying the selected throttle and spark, then trimming brake load on the live model to confirm it settles at target speed"
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

#[derive(Debug, Clone, Copy)]
struct CycleAverages {
    avg_rpm: f64,
    avg_net_torque_nm: f64,
    avg_load_torque_nm: f64,
    end_state: EngineState,
    next_dt_s: f64,
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
    ready_torque_curve: Vec<StartupFitTorqueCurvePoint>,
    rejected_release_controls: Vec<StartupFitControls>,
    pending_live_reset: bool,
    last_candidate_label: String,
    timed_out: bool,
    loaded_from_cache: bool,
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
            ready_torque_curve: Vec::new(),
            rejected_release_controls: Vec::new(),
            pending_live_reset: false,
            last_candidate_label: "waiting for priming".to_owned(),
            timed_out: false,
            loaded_from_cache: false,
        }
    }

    pub(super) fn from_artifact(started_sim_time_s: f64, artifact: &StartupFitArtifact) -> Self {
        let release_evaluation = artifact.release_evaluation;
        let release_controls = artifact.release_controls;
        let torque_curve = if artifact.torque_curve.is_empty() {
            vec![StartupFitTorqueCurvePoint {
                throttle_cmd: release_controls.throttle_cmd,
                required_brake_torque_nm: release_evaluation.required_brake_torque_nm,
            }]
        } else {
            artifact.torque_curve.clone()
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
            ready_torque_curve: torque_curve,
            rejected_release_controls: Vec::new(),
            pending_live_reset: false,
            last_candidate_label: "loaded cached startup-fit map".to_owned(),
            timed_out: artifact.timed_out,
            loaded_from_cache: true,
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
        if !self.ready_torque_curve.is_empty() {
            return self.ready_torque_curve.clone();
        }
        self.throttle_tracks
            .iter()
            .filter_map(|track| track.best().map(torque_curve_point))
            .collect()
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
            refine_torque_curve: if !self.ready_torque_curve.is_empty() {
                self.ready_torque_curve.clone()
            } else {
                self.throttle_tracks
                    .iter()
                    .filter_map(|track| track.refined_best.map(torque_curve_point))
                    .collect()
            },
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
                self.advance_search(bounds);
            }
            StartupFitPhase::Verifying => {
                self.advance_verification();
            }
            StartupFitPhase::Priming | StartupFitPhase::Ready => {}
        }
    }

    fn initialize_search(&mut self, bounds: StartupFitControlBounds) {
        let throttle_bins = linspace(
            bounds.throttle_min,
            bounds.throttle_max,
            STARTUP_FIT_THROTTLE_BIN_COUNT,
        );
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
            STARTUP_FIT_IGNITION_BIN_COUNT,
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
        self.last_candidate_label = "building coarse MBT grid".to_owned();
    }

    fn advance_search(&mut self, bounds: StartupFitControlBounds) {
        let Some(seed) = self.primed_seed.as_ref() else {
            return;
        };

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
                    self.last_candidate_label = "switching to local MBT refinement".to_owned();
                }
                SearchStage::Refine => {
                    self.select_final_candidate();
                    self.phase = StartupFitPhase::Verifying;
                    self.reset_verify_window();
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
        let deadline = startup_fit_deadline(self.started_wall_at);
        let evaluation = evaluate_candidate(seed, self.target_rpm, candidate_controls, deadline);
        let result = CandidateResult {
            controls: StartupFitControls {
                load_cmd: evaluation.load_cmd,
                ..candidate_controls
            },
            evaluation,
        };

        self.iteration = self.iteration.saturating_add(1);
        self.last_evaluation = evaluation;
        self.last_candidate_label = format!(
            "{} / throttle {:.3} / spark {:.1}",
            candidate.search_stage.label(),
            candidate_controls.throttle_cmd,
            candidate_controls.ignition_timing_deg
        );
        self.current_controls = self.best_controls;
        record_candidate_point(
            match candidate.search_stage {
                SearchStage::Coarse => &mut self.coarse_candidate_points,
                SearchStage::Refine => &mut self.refine_candidate_points,
            },
            evaluation,
        );
        self.update_best_result(candidate.throttle_index, result);
        self.next_candidate_index = self.next_candidate_index.saturating_add(1);
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
            / (STARTUP_FIT_IGNITION_BIN_COUNT.saturating_sub(1).max(1) as f64);
        for (throttle_index, track) in self.throttle_tracks.iter().enumerate() {
            let Some(best) = track.coarse_best else {
                continue;
            };
            let min_deg = (best.controls.ignition_timing_deg - coarse_span)
                .clamp(bounds.ignition_min_deg, bounds.ignition_max_deg);
            let max_deg = (best.controls.ignition_timing_deg + coarse_span)
                .clamp(bounds.ignition_min_deg, bounds.ignition_max_deg);
            for ignition_deg in linspace(min_deg, max_deg, STARTUP_FIT_IGNITION_BIN_COUNT) {
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
            "selected / throttle {:.3} / MBT {:.1} / brake {:+.1} Nm",
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
            "verify retry / throttle {:.3} / MBT {:.1} / brake {:+.1} Nm",
            selected.controls.throttle_cmd,
            selected.controls.ignition_timing_deg,
            selected.evaluation.required_brake_torque_nm
        );
        true
    }
}

fn evaluate_candidate(
    seed: &Simulator,
    target_rpm: f64,
    controls: StartupFitControls,
    deadline: Option<Instant>,
) -> CandidateEvaluation {
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
    let mut load_cmd = external_load_command_for_torque_nm(
        (unloaded.torque_net_nm + unloaded.torque_load_nm).max(0.0),
        sim.state.omega_rad_s,
        &sim.model.external_load,
    );
    sim.control.load_cmd = load_cmd;

    let mut state = sim.state;
    let mut next_dt_s = accuracy_priority_dt(target_rpm, &sim.numerics)
        .clamp(STARTUP_FIT_MIN_DT_S, sim.numerics.accuracy_dt_max_s);
    let mut last_cycle_end: Option<EngineState> = None;
    let mut last_evaluation = CandidateEvaluation::default();

    for cycle_index in 0..STARTUP_FIT_MAX_CYCLES_PER_CANDIDATE {
        if deadline_reached(deadline) {
            break;
        }

        let Some(cycle) = integrate_candidate_cycle(&sim, state, next_dt_s, deadline) else {
            break;
        };
        state = cycle.end_state;
        next_dt_s = cycle.next_dt_s;
        let periodic_error_norm = last_cycle_end.map_or(f64::INFINITY, |prev| {
            running_state_error_norm(prev, state, &sim.numerics)
        });
        let required_brake_torque_nm = cycle.avg_load_torque_nm.max(0.0);
        let torque_request_nm = (required_brake_torque_nm
            + cycle.avg_net_torque_nm
            + STARTUP_FIT_LOAD_TORQUE_GAIN_NM_PER_RPM * (cycle.avg_rpm - target_rpm))
            .max(0.0);
        load_cmd = external_load_command_for_torque_nm(
            torque_request_nm,
            state.omega_rad_s,
            &sim.model.external_load,
        );
        sim.control.load_cmd = load_cmd;
        last_evaluation = CandidateEvaluation {
            avg_rpm: cycle.avg_rpm,
            avg_net_torque_nm: cycle.avg_net_torque_nm,
            required_brake_torque_nm,
            load_cmd,
            periodic_error_norm,
            converged: cycle_index + 1 >= STARTUP_FIT_MIN_CYCLES_PER_CANDIDATE
                && (cycle.avg_rpm - target_rpm).abs() <= STARTUP_FIT_CANDIDATE_RPM_ERR
                && cycle.avg_net_torque_nm.abs() <= STARTUP_FIT_CANDIDATE_NET_TORQUE_NM
                && periodic_error_norm <= STARTUP_FIT_PERIODIC_ERROR_LIMIT,
        };
        last_cycle_end = Some(state);
        if last_evaluation.converged {
            break;
        }
    }

    last_evaluation
}

fn integrate_candidate_cycle(
    sim: &Simulator,
    start_state: EngineState,
    initial_dt_s: f64,
    deadline: Option<Instant>,
) -> Option<CycleAverages> {
    let mut state = start_state;
    let mut next_dt_s = initial_dt_s.clamp(STARTUP_FIT_MIN_DT_S, sim.numerics.accuracy_dt_max_s);
    let mut elapsed_s = 0.0;
    let mut rpm_dt_sum = 0.0;
    let mut net_torque_dt_sum = 0.0;
    let mut load_torque_dt_sum = 0.0;

    for _ in 0..STARTUP_FIT_MAX_STEPS_PER_CYCLE {
        if deadline_reached(deadline) {
            return None;
        }

        let current_sample = sim.sample_fit_state(state);
        let event_dt_s = time_to_next_ignition_event_s(state, current_sample.burn_start_deg);
        let mut try_dt_s = next_dt_s
            .min(event_dt_s)
            .clamp(STARTUP_FIT_MIN_DT_S, sim.numerics.accuracy_dt_max_s);

        loop {
            let (rk4, rk5) = sim.advance_state_rkf45_pair(state, try_dt_s);
            let error_norm = state_error_norm(rk4, rk5, &sim.numerics);
            if error_norm <= 1.0 || try_dt_s <= STARTUP_FIT_MIN_DT_S * 1.01 {
                let accepted = rk5;
                let accepted_sample = sim.sample_fit_state(accepted);
                rpm_dt_sum += accepted_sample.rpm * try_dt_s;
                net_torque_dt_sum += accepted_sample.torque_net_nm * try_dt_s;
                load_torque_dt_sum += accepted_sample.torque_load_nm * try_dt_s;
                elapsed_s += try_dt_s;
                let crossed_cycle = accepted.theta_rad < state.theta_rad;
                next_dt_s = rkf45_next_dt(try_dt_s, error_norm)
                    .clamp(STARTUP_FIT_MIN_DT_S, sim.numerics.accuracy_dt_max_s);
                state = accepted;
                if crossed_cycle {
                    let duration = elapsed_s.max(f64::EPSILON);
                    return Some(CycleAverages {
                        avg_rpm: rpm_dt_sum / duration,
                        avg_net_torque_nm: net_torque_dt_sum / duration,
                        avg_load_torque_nm: load_torque_dt_sum / duration,
                        end_state: state,
                        next_dt_s,
                    });
                }
                break;
            }
            try_dt_s = rkf45_next_dt(try_dt_s, error_norm)
                .clamp(STARTUP_FIT_MIN_DT_S, sim.numerics.accuracy_dt_max_s);
        }
    }

    None
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

#[cfg(test)]
mod tests {
    use crate::config::AppConfig;
    use crate::simulator::Simulator;

    use super::{
        CandidateEvaluation, CandidateResult, STARTUP_FIT_INITIAL_THROTTLE, STARTUP_FIT_TARGET_RPM,
        STARTUP_FIT_TORQUE_MARGIN_FROM_BEST_NM, SearchStage, StartupFitControlBounds,
        StartupFitControls, StartupFitPhase, StartupFitState, ThrottleTrack, linspace,
        rkf45_next_dt, select_candidate_for_release, startup_fit_wall_limit_label,
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

        state.update(0.31, &sim, bounds());
        let snapshot = state.snapshot(0.31);

        assert_eq!(snapshot.phase, StartupFitPhase::Optimizing);
        assert!(matches!(state.search_stage, SearchStage::Coarse));
    }

    #[test]
    fn selected_margin_constant_is_positive() {
        assert!(STARTUP_FIT_TORQUE_MARGIN_FROM_BEST_NM > 0.0);
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
    fn wall_limit_label_reports_no_cap_when_disabled() {
        assert_eq!(startup_fit_wall_limit_label(), "no cap");
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
        assert_eq!(snapshot.coarse_candidate_points.len(), 2);
        assert_eq!(snapshot.refine_candidate_points.len(), 1);
        assert!((snapshot.coarse_torque_curve[0].throttle_cmd - 0.22).abs() < 1.0e-12);
        assert!((snapshot.refine_torque_curve[0].required_brake_torque_nm - 8.1).abs() < 1.0e-12);
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
}
