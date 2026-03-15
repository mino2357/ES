use std::thread;
use std::time::{Duration, Instant};

use eframe::egui;

use super::startup_fit::{
    STARTUP_FIT_FAST_FORWARD_FACTOR, STARTUP_FIT_FRAME_WALL_BUDGET_S, STARTUP_FIT_INITIAL_THROTTLE,
    STARTUP_FIT_TARGET_RPM, StartupFitControls, StartupFitState, StartupFitStatus,
    StartupFitTorqueCurvePoint, StartupFitWorkerHandle, StartupFitWorkerPoll,
    StartupFitWotTorquePoint, apply_startup_fit_state_controls, apply_startup_fit_status_controls,
    startup_fit_control_bounds,
};
use super::startup_fit_artifact::{
    StartupFitArtifact, StartupFitArtifactSnapshot, StartupFitCacheContext, load_matching_artifact,
    save_artifact,
};
use crate::config::{AppConfig, ExternalLoadMode, LoadedAppConfig, UiConfig};
use crate::simulator::{
    Observation, Simulator, accuracy_priority_dt, estimate_mbt_deg, estimate_realtime_performance,
    external_load_command_for_torque_nm, rad_s_to_rpm, rpm_linked_dt, shaft_power_hp,
    shaft_power_kw,
};

const POST_FIT_SPEED_HOLD_GAIN_NM_PER_RPM: f64 = 0.08;
const DRIVER_DEMAND_DEFAULT: f64 = 0.50;
const STARTUP_FIT_HEADLESS_BUDGET_DISABLE_THRESHOLD_S: f64 = 0.005;
const STARTUP_FIT_WORKER_LIVE_MAX_STEPS_PER_FRAME: usize = 240;
const RUNTIME_FRAME_WALL_BUDGET_S: f64 = 0.006;
const RUNTIME_INTERACTION_FRAME_WALL_BUDGET_S: f64 = 0.0025;
const INTERACTION_MAX_STEPS_PER_FRAME_CAP: usize = 96;
const SIMULATION_BACKLOG_FRAME_CAP: f64 = 6.0;
const BACKGROUND_PV_SUBSAMPLES_DIVISOR: usize = 12;
const BACKGROUND_PV_SUBSAMPLES_MIN: usize = 6;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum PostFitRuntimeMode {
    StandardRuntime,
    ActuatorLab,
}

impl PostFitRuntimeMode {
    pub(super) fn label(self) -> &'static str {
        match self {
            Self::StandardRuntime => "Standard runtime",
            Self::ActuatorLab => "Actuator lab",
        }
    }

    pub(super) fn detail(self) -> &'static str {
        match self {
            Self::StandardRuntime => {
                "Driver demand sets torque request; throttle, ignition, and VVT follow the fitted baseline automatically."
            }
            Self::ActuatorLab => {
                "Driver demand still sets torque request, while throttle, ignition, and VVT become live manual overrides against the auto baseline."
            }
        }
    }
}

#[derive(Debug, Clone, Copy, Default)]
pub(super) struct DashboardUiLoadHint {
    pub(super) pressure_tab_visible: bool,
    pub(super) interaction_active: bool,
}

#[derive(Debug, Clone, Copy)]
pub(super) struct PostFitBaseline {
    pub(super) requested_brake_torque_nm: f64,
    pub(super) equilibrium_rpm: f64,
    pub(super) throttle_cmd: f64,
    pub(super) ignition_timing_deg: f64,
    pub(super) vvt_intake_deg: f64,
    pub(super) vvt_exhaust_deg: f64,
}

impl PostFitBaseline {
    fn new(
        requested_brake_torque_nm: f64,
        equilibrium_rpm: f64,
        throttle_cmd: f64,
        ignition_timing_deg: f64,
        vvt_intake_deg: f64,
        vvt_exhaust_deg: f64,
    ) -> Self {
        Self {
            requested_brake_torque_nm,
            equilibrium_rpm,
            throttle_cmd,
            ignition_timing_deg,
            vvt_intake_deg,
            vvt_exhaust_deg,
        }
    }
}

// Owns runtime pacing, controls, and the latest simulation state shown in the dashboard.
pub(super) struct DashboardState {
    pub(super) sim: Simulator,
    pub(super) latest: Observation,
    last_tick: Instant,
    simulated_time_s: f64,
    dt_base: f64,
    dt_next: f64,
    dt_min_bound: f64,
    dt_max_bound: f64,
    realtime_fixed_dt_s: Option<f64>,
    pub(super) load_target_rpm: f64,
    pub(super) driver_demand: f64,
    pub(super) post_fit_mode: PostFitRuntimeMode,
    pub(super) post_fit_baseline: PostFitBaseline,
    startup_fit: StartupFitState,
    startup_fit_worker: Option<StartupFitWorkerHandle>,
    startup_fit_cache_context: Option<StartupFitCacheContext>,
    startup_fit_artifact_persisted: bool,
    post_fit_defaults_initialized: bool,
    ui_load_hint: DashboardUiLoadHint,
    simulated_time_backlog_s: f64,
    pressure_focus_pv_subsamples_per_step: usize,
    background_pv_subsamples_per_step: usize,
}

impl DashboardState {
    #[cfg_attr(not(test), allow(dead_code))]
    pub(super) fn new(config: AppConfig) -> Self {
        Self::build(config, None)
    }

    pub(super) fn from_loaded_config(loaded: LoadedAppConfig) -> Self {
        let cache_context = StartupFitCacheContext::from_loaded_config(&loaded);
        Self::build(loaded.config, cache_context)
    }

    fn build(config: AppConfig, startup_fit_cache_context: Option<StartupFitCacheContext>) -> Self {
        let ui_config = config.ui.clone();
        let dt = config.environment.dt.max(ui_config.min_base_dt_s);
        let (dt_min_bound, dt_max_bound, realtime_fixed_dt_s, dt_next) = if ui_config
            .sync_to_wall_clock
        {
            // Benchmark a safe realtime floor once at startup and keep fixed dt when headroom is ample.
            let realtime_perf = estimate_realtime_performance(&config, dt);
            let dt_min_bound = realtime_perf
                .floor_dt_s
                .max(dt * ui_config.realtime_dt_min_factor);
            let dt_max_bound = (dt * ui_config.realtime_dt_max_factor)
                .max(dt_min_bound * ui_config.realtime_dt_max_over_min_factor);
            let realtime_fixed_dt_s = (realtime_perf.fixed_dt_headroom_ratio
                >= config.numerics.realtime_fixed_dt_headroom_ratio)
                .then_some(
                    realtime_perf
                        .fixed_dt_candidate_s
                        .clamp(dt_min_bound, dt_max_bound),
                );
            let dt_next =
                realtime_fixed_dt_s.unwrap_or_else(|| dt.clamp(dt_min_bound, dt_max_bound));
            (dt_min_bound, dt_max_bound, realtime_fixed_dt_s, dt_next)
        } else {
            let dt_min_bound = config
                .numerics
                .rpm_link_dt_min_floor_s
                .max(ui_config.dt_epsilon_s);
            let dt_max_bound = config.numerics.accuracy_dt_max_s.max(dt_min_bound);
            let dt_next = accuracy_priority_dt(config.engine.default_target_rpm, &config.numerics)
                .clamp(dt_min_bound, dt_max_bound);
            (dt_min_bound, dt_max_bound, None, dt_next)
        };
        let mut sim = Simulator::new(&config);
        // The startup-fit dashboard solves a fixed-RPM bench point, so use the brake-dyno load
        // path even if the checked-in config defaults to a vehicle-equivalent road load.
        sim.model.external_load.mode = ExternalLoadMode::BrakeMap;
        let mut load_target_rpm = STARTUP_FIT_TARGET_RPM;
        let initial_load_est = 0.60;
        let ignition_timing_deg = estimate_mbt_deg(&sim.model, load_target_rpm, initial_load_est);
        let initial_fit_controls = StartupFitControls {
            throttle_cmd: STARTUP_FIT_INITIAL_THROTTLE,
            ignition_timing_deg,
            vvt_intake_deg: 0.0,
            vvt_exhaust_deg: 0.0,
            load_cmd: 0.0,
        };
        sim.control.spark_cmd = true;
        sim.control.fuel_cmd = true;
        sim.control.vvt_intake_deg = 0.0;
        sim.control.vvt_exhaust_deg = 0.0;
        sim.control.load_cmd = 0.0;
        let cached_artifact = startup_fit_cache_context.as_ref().and_then(|context| {
            match load_matching_artifact(context) {
                Ok(artifact) => artifact,
                Err(err) => {
                    eprintln!("{err}");
                    None
                }
            }
        });
        let (startup_fit, startup_fit_artifact_persisted, startup_fit_worker) =
            if let Some(ref artifact) = cached_artifact {
                load_target_rpm = artifact.target_rpm;
                sim.control.throttle_cmd = artifact.release_controls.throttle_cmd;
                sim.control.ignition_timing_deg = artifact.release_controls.ignition_timing_deg;
                sim.control.vvt_intake_deg = artifact.release_controls.vvt_intake_deg;
                sim.control.vvt_exhaust_deg = artifact.release_controls.vvt_exhaust_deg;
                sim.control.load_cmd = artifact.release_controls.load_cmd;
                sim.seed_operating_point(
                    artifact.target_rpm,
                    artifact.release_controls.throttle_cmd,
                    artifact.release_controls.ignition_timing_deg,
                );
                (
                    StartupFitState::from_artifact(dt_next, artifact),
                    true,
                    None,
                )
            } else {
                sim.seed_operating_point(
                    load_target_rpm,
                    initial_fit_controls.throttle_cmd,
                    initial_fit_controls.ignition_timing_deg,
                );
                (
                    StartupFitState::new(dt_next, load_target_rpm, initial_fit_controls),
                    startup_fit_cache_context.is_none(),
                    Some(StartupFitWorkerHandle::spawn(
                        config.clone(),
                        dt_next,
                        load_target_rpm,
                        initial_fit_controls,
                    )),
                )
            };
        let latest = sim.step(dt_next);
        let pressure_focus_pv_subsamples_per_step = config.plot.pv_subsamples_per_step.max(1);
        let background_pv_subsamples_per_step =
            background_pv_subsamples_per_step(pressure_focus_pv_subsamples_per_step);
        let mut state = Self {
            sim,
            latest,
            last_tick: Instant::now(),
            simulated_time_s: dt_next,
            dt_base: dt,
            dt_next,
            dt_min_bound,
            dt_max_bound,
            realtime_fixed_dt_s,
            load_target_rpm,
            driver_demand: DRIVER_DEMAND_DEFAULT,
            post_fit_mode: PostFitRuntimeMode::StandardRuntime,
            post_fit_baseline: PostFitBaseline::new(
                0.0,
                STARTUP_FIT_TARGET_RPM,
                STARTUP_FIT_INITIAL_THROTTLE,
                ignition_timing_deg,
                0.0,
                0.0,
            ),
            startup_fit,
            startup_fit_worker,
            startup_fit_cache_context,
            startup_fit_artifact_persisted,
            post_fit_defaults_initialized: false,
            ui_load_hint: DashboardUiLoadHint::default(),
            simulated_time_backlog_s: 0.0,
            pressure_focus_pv_subsamples_per_step,
            background_pv_subsamples_per_step,
        };
        state.synchronize_post_fit_runtime_now();
        state
    }

    pub(super) fn displayed_target_rpm(&self) -> f64 {
        if self.startup_fit_active() {
            self.load_target_rpm
        } else {
            self.post_fit_baseline.equilibrium_rpm
        }
    }

    pub(super) fn rpm_error(&self) -> f64 {
        self.displayed_target_rpm() - self.latest.rpm
    }

    pub(super) fn net_shaft_power_kw(&self) -> f64 {
        shaft_power_kw(self.latest.rpm, self.latest.torque_net_nm)
    }

    pub(super) fn net_shaft_power_hp(&self) -> f64 {
        shaft_power_hp(self.latest.rpm, self.latest.torque_net_nm)
    }

    pub(super) fn shaft_torque_estimate_nm(&self) -> f64 {
        self.latest.torque_combustion_cycle_nm
            - self.latest.torque_friction_nm
            - self.latest.torque_pumping_nm
    }

    pub(super) fn solver_mode_label(&self, ui_config: &UiConfig) -> &'static str {
        if ui_config.sync_to_wall_clock {
            "Solver mode: wall-clock synchronized"
        } else {
            "Solver mode: accuracy first"
        }
    }

    pub(super) fn firing_state_label(&self) -> &'static str {
        let combustion_active = self.sim.control.spark_cmd
            && self.sim.control.fuel_cmd
            && self.latest.rpm > self.sim.model.combustion_enable_rpm_min
            && self.latest.torque_combustion_cycle_nm > 1.0;
        if combustion_active {
            "State: FIRED"
        } else if self.latest.rpm > 120.0 {
            "State: COASTING"
        } else {
            "State: STOPPED"
        }
    }

    pub(super) fn startup_fit_active(&self) -> bool {
        self.startup_fit_worker.is_some() || self.startup_fit.is_active()
    }

    pub(super) fn startup_fit_status(&self) -> StartupFitStatus {
        self.startup_fit_worker
            .as_ref()
            .map(|worker| worker.latest_status().clone())
            .unwrap_or_else(|| self.startup_fit.snapshot(self.simulated_time_s))
    }

    pub(super) fn set_post_fit_runtime_mode(&mut self, mode: PostFitRuntimeMode) {
        if self.post_fit_mode == mode {
            return;
        }
        self.post_fit_mode = mode;
        self.initialize_post_fit_runtime_if_needed();
        self.update_post_fit_baseline();
        if mode == PostFitRuntimeMode::ActuatorLab {
            self.seed_manual_actuator_lab_from_baseline();
        }
    }

    pub(super) fn refresh_post_fit_preview(&mut self) {
        self.initialize_post_fit_runtime_if_needed();
        self.update_post_fit_baseline();
    }

    pub(super) fn apply_shortcuts(&mut self, ctx: &egui::Context, ui_config: &UiConfig) {
        let manual_shortcuts_enabled = !self.startup_fit_active();
        ctx.input(|i| {
            if i.key_pressed(egui::Key::Q) {
                ctx.send_viewport_cmd(egui::ViewportCommand::Close);
            }
            if manual_shortcuts_enabled {
                if i.key_pressed(egui::Key::I) {
                    self.sim.control.spark_cmd = !self.sim.control.spark_cmd;
                }
                if i.key_pressed(egui::Key::F) {
                    self.sim.control.fuel_cmd = !self.sim.control.fuel_cmd;
                }
                if i.key_pressed(egui::Key::W) {
                    match self.post_fit_mode {
                        PostFitRuntimeMode::StandardRuntime => {
                            self.driver_demand =
                                (self.driver_demand + ui_config.throttle_key_step).clamp(0.0, 1.0);
                        }
                        PostFitRuntimeMode::ActuatorLab => {
                            self.sim.control.throttle_cmd = (self.sim.control.throttle_cmd
                                + ui_config.throttle_key_step)
                                .clamp(0.0, 1.0);
                        }
                    }
                }
                if i.key_pressed(egui::Key::X) {
                    match self.post_fit_mode {
                        PostFitRuntimeMode::StandardRuntime => {
                            self.driver_demand =
                                (self.driver_demand - ui_config.throttle_key_step).clamp(0.0, 1.0);
                        }
                        PostFitRuntimeMode::ActuatorLab => {
                            self.sim.control.throttle_cmd = (self.sim.control.throttle_cmd
                                - ui_config.throttle_key_step)
                                .clamp(0.0, 1.0);
                        }
                    }
                }
            }
        });
    }

    pub(super) fn set_ui_load_hint(&mut self, hint: DashboardUiLoadHint) {
        self.ui_load_hint = hint;
    }

    pub(super) fn advance_simulation(&mut self, ui_config: &UiConfig) {
        self.apply_live_diagnostics_hint();
        self.poll_startup_fit_worker();
        let now = Instant::now();
        let elapsed_since_last_frame_s = (now - self.last_tick).as_secs_f64();
        let worker_active = self.startup_fit_worker.is_some();
        let frame_deadline = now.checked_add(Duration::from_secs_f64(frame_wall_budget_s(
            self.startup_fit.is_active(),
            worker_active,
            elapsed_since_last_frame_s,
            self.ui_load_hint.interaction_active,
        )));
        let requested_time = if ui_config.sync_to_wall_clock {
            elapsed_since_last_frame_s.max(self.dt_base)
        } else {
            ui_config.simulated_time_per_frame_s.max(self.dt_base)
        };
        let mut target_time = (requested_time
            + self
                .simulated_time_backlog_s
                .min(simulation_backlog_cap_s(ui_config)))
        .max(0.0);
        if worker_active {
            target_time = 0.0;
            self.simulated_time_backlog_s = 0.0;
        }
        if self.startup_fit.is_active() && !worker_active && !ui_config.sync_to_wall_clock {
            target_time *= STARTUP_FIT_FAST_FORWARD_FACTOR;
        }
        let mut simulated = 0.0;
        let mut steps = 0usize;
        let max_steps = if worker_active {
            ui_config
                .max_steps_per_frame
                .min(STARTUP_FIT_WORKER_LIVE_MAX_STEPS_PER_FRAME)
                .max(1)
        } else if self.ui_load_hint.interaction_active {
            ui_config
                .max_steps_per_frame
                .min(INTERACTION_MAX_STEPS_PER_FRAME_CAP)
                .max(1)
        } else {
            ui_config.max_steps_per_frame.max(1)
        };
        if self.startup_fit_active() {
            self.apply_fit_controls_if_active();
        }
        // In accuracy-first mode the solver advances a fixed slice of simulated time per frame
        // and does not try to stay phase-locked to wall clock.
        while simulated < target_time && steps < max_steps {
            if frame_deadline
                .map(|deadline| Instant::now() >= deadline)
                .unwrap_or(false)
            {
                break;
            }
            self.apply_runtime_controls();
            let rpm = rad_s_to_rpm(self.sim.state.omega_rad_s.max(0.0));
            let dt_target = if ui_config.sync_to_wall_clock {
                if let Some(fixed_dt) = self.realtime_fixed_dt_s {
                    fixed_dt
                } else {
                    rpm_linked_dt(
                        self.dt_base,
                        rpm,
                        self.load_target_rpm.max(self.sim.params.default_target_rpm),
                        &self.sim.numerics,
                    )
                }
            } else {
                accuracy_priority_dt(rpm, &self.sim.numerics)
            };
            let dt_target = dt_target.clamp(self.dt_min_bound, self.dt_max_bound);
            let dt_smoothed =
                self.dt_next + ui_config.dt_smoothing_factor * (dt_target - self.dt_next);
            let dt_step = dt_smoothed
                .clamp(self.dt_min_bound, self.dt_max_bound)
                .max(ui_config.dt_epsilon_s)
                .min((target_time - simulated).max(ui_config.dt_epsilon_s));

            self.latest = self.sim.step(dt_step);
            self.dt_next = self
                .realtime_fixed_dt_s
                .unwrap_or_else(|| dt_smoothed.clamp(self.dt_min_bound, self.dt_max_bound));
            simulated += dt_step;
            steps += 1;
            self.record_fit_live_sample(dt_step);
        }

        let deadline_remaining = frame_deadline
            .map(|deadline| Instant::now() < deadline)
            .unwrap_or(true);
        if simulated < target_time && deadline_remaining {
            // Render the newest computed state even if the requested simulated horizon was not
            // fully consumed in this frame.
            let dt_tail = (target_time - simulated)
                .max(ui_config.dt_epsilon_s)
                .min(self.dt_max_bound);
            self.apply_runtime_controls();
            self.latest = self.sim.step(dt_tail);
            self.dt_next = self
                .realtime_fixed_dt_s
                .unwrap_or_else(|| dt_tail.clamp(self.dt_min_bound, self.dt_max_bound));
            simulated += dt_tail;
            self.record_fit_live_sample(dt_tail);
        }
        self.simulated_time_backlog_s = (target_time - simulated)
            .max(0.0)
            .min(simulation_backlog_cap_s(ui_config));
        self.simulated_time_s += simulated;
        self.advance_startup_fit(elapsed_since_last_frame_s, simulated, frame_deadline);
        self.poll_startup_fit_worker();
        if worker_active {
            if elapsed_since_last_frame_s < STARTUP_FIT_HEADLESS_BUDGET_DISABLE_THRESHOLD_S {
                thread::sleep(Duration::from_millis(1));
            } else {
                thread::yield_now();
            }
        }
        if !self.startup_fit_active() {
            self.persist_startup_fit_artifact_if_needed();
            self.initialize_post_fit_runtime_if_needed();
            self.update_post_fit_baseline();
            if self.post_fit_mode == PostFitRuntimeMode::StandardRuntime {
                self.apply_standard_runtime_actuator_baseline();
            }
        }
        self.sanitize_control_inputs();
        self.last_tick = now;
    }

    fn apply_runtime_controls(&mut self) {
        if self.startup_fit_active() {
            self.apply_fit_controls_if_active();
        } else {
            self.initialize_post_fit_runtime_if_needed();
            self.update_post_fit_baseline();
            match self.post_fit_mode {
                PostFitRuntimeMode::StandardRuntime => self.apply_standard_runtime_controls(),
                PostFitRuntimeMode::ActuatorLab => self.apply_actuator_lab_controls(),
            }
        }
    }

    fn apply_live_diagnostics_hint(&mut self) {
        let pressure_focus = self.ui_load_hint.pressure_tab_visible;
        self.sim.plot.pv_subsamples_per_step = if pressure_focus {
            self.pressure_focus_pv_subsamples_per_step
        } else {
            self.background_pv_subsamples_per_step
        };
        self.sim.set_pressure_trace_output_enabled(pressure_focus);
    }

    fn apply_fit_controls_if_active(&mut self) {
        if !self.startup_fit_active() {
            return;
        }
        if let Some(worker) = self.startup_fit_worker.as_ref() {
            apply_startup_fit_status_controls(&mut self.sim, worker.latest_status());
        } else {
            apply_startup_fit_state_controls(&mut self.sim, &self.startup_fit);
        }
    }

    fn apply_standard_runtime_controls(&mut self) {
        self.apply_standard_runtime_actuator_baseline();
        self.apply_post_fit_speed_hold();
    }

    fn apply_standard_runtime_actuator_baseline(&mut self) {
        self.sim.control.throttle_cmd = self.post_fit_baseline.throttle_cmd;
        self.sim.control.ignition_timing_deg = self.post_fit_baseline.ignition_timing_deg;
        self.sim.control.vvt_intake_deg = self.post_fit_baseline.vvt_intake_deg;
        self.sim.control.vvt_exhaust_deg = self.post_fit_baseline.vvt_exhaust_deg;
    }

    fn apply_actuator_lab_controls(&mut self) {
        self.apply_post_fit_speed_hold();
    }

    fn seed_manual_actuator_lab_from_baseline(&mut self) {
        self.sim.control.throttle_cmd = self.post_fit_baseline.throttle_cmd;
        self.sim.control.ignition_timing_deg = self.post_fit_baseline.ignition_timing_deg;
        self.sim.control.vvt_intake_deg = self.post_fit_baseline.vvt_intake_deg;
        self.sim.control.vvt_exhaust_deg = self.post_fit_baseline.vvt_exhaust_deg;
    }

    fn initialize_post_fit_runtime_if_needed(&mut self) {
        if self.startup_fit_active() || self.post_fit_defaults_initialized {
            return;
        }
        let release_torque_nm = self.startup_fit.selected_required_brake_torque_nm();
        let wot_curve = self.startup_fit.wot_torque_curve();
        self.driver_demand =
            driver_demand_for_wot_torque_request(release_torque_nm, &wot_curve, release_torque_nm);
        self.post_fit_defaults_initialized = true;
    }

    fn update_post_fit_baseline(&mut self) {
        if self.startup_fit_active() {
            return;
        }
        let release_controls = self.startup_fit.release_controls();
        let release_rpm = self
            .startup_fit
            .release_avg_rpm()
            .max(STARTUP_FIT_TARGET_RPM)
            .max(600.0);
        let wot_curve = self.startup_fit.wot_torque_curve();
        let requested_brake_torque_nm =
            torque_request_from_driver_demand_wot(self.driver_demand, &wot_curve, 0.0);
        let equilibrium_fallback_rpm = self
            .post_fit_baseline
            .equilibrium_rpm
            .max(release_rpm)
            .max(600.0);
        let equilibrium_rpm = rpm_for_requested_torque(
            requested_brake_torque_nm,
            &wot_curve,
            equilibrium_fallback_rpm,
        );
        let throttle_cmd = 1.0;
        let ignition_timing_deg = wot_ignition_for_rpm(
            equilibrium_rpm,
            &wot_curve,
            estimate_mbt_deg(
                &self.sim.model,
                equilibrium_rpm.max(600.0),
                self.sim.model.load_max.max(0.95),
            ),
        );
        self.post_fit_baseline = PostFitBaseline::new(
            requested_brake_torque_nm,
            equilibrium_rpm,
            throttle_cmd,
            ignition_timing_deg,
            release_controls.vvt_intake_deg,
            release_controls.vvt_exhaust_deg,
        );
    }

    fn synchronize_post_fit_runtime_now(&mut self) {
        if self.startup_fit_active() {
            return;
        }
        self.initialize_post_fit_runtime_if_needed();
        self.update_post_fit_baseline();
        match self.post_fit_mode {
            PostFitRuntimeMode::StandardRuntime => self.apply_standard_runtime_controls(),
            PostFitRuntimeMode::ActuatorLab => self.apply_actuator_lab_controls(),
        }
        self.latest = self.sim.step(self.dt_min_bound.max(1.0e-4));
    }

    fn apply_post_fit_speed_hold(&mut self) {
        if !self.sim.control.spark_cmd || !self.sim.control.fuel_cmd || self.latest.rpm <= 120.0 {
            return;
        }

        let actual_rpm = rad_s_to_rpm(self.sim.state.omega_rad_s.max(0.0));
        let target_torque_nm = fit_speed_hold_target_torque_nm(
            self.post_fit_baseline.requested_brake_torque_nm,
            actual_rpm,
            self.post_fit_baseline.equilibrium_rpm,
        );
        self.sim.control.load_cmd = external_load_command_for_torque_nm(
            target_torque_nm,
            self.sim.state.omega_rad_s,
            &self.sim.model.external_load,
        );
    }

    fn advance_startup_fit(
        &mut self,
        _frame_wall_s: f64,
        _frame_simulated_s: f64,
        frame_deadline: Option<Instant>,
    ) {
        if !self.startup_fit.is_active() || self.startup_fit_worker.is_some() {
            return;
        }
        let bounds = startup_fit_control_bounds(&self.sim);
        self.startup_fit
            .update(self.simulated_time_s, &self.sim, bounds, frame_deadline);
        if let Some(seed) = self.startup_fit.take_live_reset_seed() {
            self.sim = seed;
            self.apply_fit_controls_if_active();
            self.latest = self.sim.step(self.dt_min_bound.max(1.0e-4));
        }
        self.apply_fit_controls_if_active();
    }

    fn record_fit_live_sample(&mut self, dt_s: f64) {
        if !self.startup_fit.is_active() || self.startup_fit_worker.is_some() {
            return;
        }
        self.startup_fit.record_live_sample(
            dt_s,
            self.latest.rpm,
            self.latest.torque_net_nm,
            self.latest.torque_load_nm,
        );
    }

    fn sanitize_control_inputs(&mut self) {
        self.load_target_rpm = finite_f64(self.load_target_rpm, STARTUP_FIT_TARGET_RPM)
            .clamp(0.0, self.sim.params.max_rpm);
        self.driver_demand = finite_f64(self.driver_demand, DRIVER_DEMAND_DEFAULT).clamp(0.0, 1.0);
        self.sim.control.throttle_cmd =
            finite_f64(self.sim.control.throttle_cmd, STARTUP_FIT_INITIAL_THROTTLE).clamp(0.0, 1.0);
        self.sim.control.ignition_timing_deg = finite_f64(
            self.sim.control.ignition_timing_deg,
            estimate_mbt_deg(&self.sim.model, self.load_target_rpm.max(600.0), 0.60),
        )
        .clamp(self.sim.model.mbt_min_deg, self.sim.model.mbt_max_deg);
        self.sim.control.vvt_intake_deg =
            finite_f64(self.sim.control.vvt_intake_deg, 0.0).clamp(-90.0, 90.0);
        self.sim.control.vvt_exhaust_deg =
            finite_f64(self.sim.control.vvt_exhaust_deg, 0.0).clamp(-90.0, 90.0);
        self.sim.control.load_cmd = finite_f64(self.sim.control.load_cmd, 0.0).clamp(-1.0, 1.0);
    }

    fn persist_startup_fit_artifact_if_needed(&mut self) {
        if self.startup_fit_artifact_persisted {
            return;
        }
        let Some(context) = self.startup_fit_cache_context.as_ref() else {
            self.startup_fit_artifact_persisted = true;
            return;
        };
        let Some(snapshot) = self.startup_fit.artifact() else {
            self.startup_fit_artifact_persisted = true;
            return;
        };
        let artifact = StartupFitArtifact::new(context, snapshot);
        if let Err(err) = save_artifact(context, &artifact) {
            eprintln!("{err}");
        }
        self.startup_fit_artifact_persisted = true;
    }

    fn poll_startup_fit_worker(&mut self) {
        let Some(mut worker) = self.startup_fit_worker.take() else {
            return;
        };
        match worker.poll() {
            StartupFitWorkerPoll::Running => {
                self.startup_fit_worker = Some(worker);
            }
            StartupFitWorkerPoll::Completed(snapshot) => {
                self.startup_fit = StartupFitState::from_snapshot(self.simulated_time_s, &snapshot);
                self.startup_fit_artifact_persisted = self.startup_fit_cache_context.is_none();
                self.post_fit_defaults_initialized = false;
                self.sim.control.spark_cmd = true;
                self.sim.control.fuel_cmd = true;
                self.sim.control.throttle_cmd = snapshot.release_controls.throttle_cmd;
                self.sim.control.ignition_timing_deg =
                    snapshot.release_controls.ignition_timing_deg;
                self.sim.control.vvt_intake_deg = snapshot.release_controls.vvt_intake_deg;
                self.sim.control.vvt_exhaust_deg = snapshot.release_controls.vvt_exhaust_deg;
                self.sim.control.load_cmd = snapshot.release_controls.load_cmd;
                self.sim.seed_operating_point(
                    snapshot.target_rpm,
                    snapshot.release_controls.throttle_cmd,
                    snapshot.release_controls.ignition_timing_deg,
                );
                self.synchronize_post_fit_runtime_now();
            }
            StartupFitWorkerPoll::Failed(message) => {
                eprintln!("{message}");
                let status = worker.latest_status().clone();
                let fallback_snapshot = StartupFitArtifactSnapshot {
                    target_rpm: status.target_rpm,
                    timed_out: true,
                    release_controls: StartupFitControls {
                        throttle_cmd: status.throttle_cmd,
                        ignition_timing_deg: status.ignition_timing_deg,
                        vvt_intake_deg: status.vvt_intake_deg,
                        vvt_exhaust_deg: status.vvt_exhaust_deg,
                        load_cmd: status.load_cmd,
                    },
                    release_evaluation: super::startup_fit_artifact::StartupFitArtifactEvaluation {
                        avg_rpm: status.avg_rpm,
                        avg_net_torque_nm: status.avg_net_torque_nm,
                        required_brake_torque_nm: status.required_brake_torque_nm,
                        load_cmd: status.load_cmd,
                        periodic_error_norm: status.periodic_error_norm,
                        converged: false,
                    },
                    best_required_brake_torque_nm: status.best_required_brake_torque_nm,
                    torque_margin_to_best_nm: status.torque_margin_to_best_nm,
                    torque_curve: status.refine_torque_curve.clone(),
                    wot_torque_curve: status.wot_torque_curve.clone(),
                };
                self.startup_fit =
                    StartupFitState::from_snapshot(self.simulated_time_s, &fallback_snapshot);
                self.startup_fit_artifact_persisted = true;
            }
        }
    }
}

#[cfg_attr(not(test), allow(dead_code))]
pub(super) fn wrap_cycle_deg(value: f64) -> f64 {
    value.rem_euclid(720.0)
}

#[cfg_attr(not(test), allow(dead_code))]
pub(super) fn shortest_cycle_delta_deg(from: f64, to: f64) -> f64 {
    let wrapped = (to - from + 360.0).rem_euclid(720.0) - 360.0;
    if wrapped <= -360.0 {
        wrapped + 720.0
    } else {
        wrapped
    }
}

fn finite_f64(value: f64, fallback: f64) -> f64 {
    if value.is_finite() { value } else { fallback }
}

fn frame_wall_budget_s(
    startup_fit_active: bool,
    worker_active: bool,
    elapsed_since_last_frame_s: f64,
    interaction_active: bool,
) -> f64 {
    if startup_fit_active
        && !worker_active
        && elapsed_since_last_frame_s >= STARTUP_FIT_HEADLESS_BUDGET_DISABLE_THRESHOLD_S
    {
        STARTUP_FIT_FRAME_WALL_BUDGET_S
    } else if interaction_active {
        RUNTIME_INTERACTION_FRAME_WALL_BUDGET_S
    } else {
        RUNTIME_FRAME_WALL_BUDGET_S
    }
}

fn simulation_backlog_cap_s(ui_config: &UiConfig) -> f64 {
    (ui_config
        .simulated_time_per_frame_s
        .max(ui_config.min_base_dt_s)
        * SIMULATION_BACKLOG_FRAME_CAP)
        .max(ui_config.min_base_dt_s)
}

fn background_pv_subsamples_per_step(full_rate_subsamples: usize) -> usize {
    (full_rate_subsamples / BACKGROUND_PV_SUBSAMPLES_DIVISOR)
        .max(BACKGROUND_PV_SUBSAMPLES_MIN)
        .min(full_rate_subsamples.max(1))
}

fn fit_speed_hold_target_torque_nm(
    reference_brake_torque_nm: f64,
    actual_rpm: f64,
    target_rpm: f64,
) -> f64 {
    (reference_brake_torque_nm + POST_FIT_SPEED_HOLD_GAIN_NM_PER_RPM * (actual_rpm - target_rpm))
        .max(0.0)
}

#[cfg_attr(not(test), allow(dead_code))]
fn driver_demand_for_torque_request(
    requested_brake_torque_nm: f64,
    torque_curve: &[StartupFitTorqueCurvePoint],
    fallback_torque_nm: f64,
) -> f64 {
    let (min_torque_nm, max_torque_nm) = torque_request_bounds(torque_curve, fallback_torque_nm);
    if (max_torque_nm - min_torque_nm).abs() <= f64::EPSILON {
        return 1.0;
    }
    ((requested_brake_torque_nm - min_torque_nm) / (max_torque_nm - min_torque_nm)).clamp(0.0, 1.0)
}

#[cfg_attr(not(test), allow(dead_code))]
fn torque_request_from_driver_demand(
    driver_demand: f64,
    torque_curve: &[StartupFitTorqueCurvePoint],
    fallback_torque_nm: f64,
) -> f64 {
    let demand = finite_f64(driver_demand, DRIVER_DEMAND_DEFAULT).clamp(0.0, 1.0);
    let (min_torque_nm, max_torque_nm) = torque_request_bounds(torque_curve, fallback_torque_nm);
    min_torque_nm + demand * (max_torque_nm - min_torque_nm)
}

fn driver_demand_for_wot_torque_request(
    requested_brake_torque_nm: f64,
    torque_curve: &[StartupFitWotTorquePoint],
    fallback_torque_nm: f64,
) -> f64 {
    let (min_torque_nm, max_torque_nm) =
        wot_torque_request_bounds(torque_curve, fallback_torque_nm);
    if (max_torque_nm - min_torque_nm).abs() <= f64::EPSILON {
        return 1.0;
    }
    ((requested_brake_torque_nm - min_torque_nm) / (max_torque_nm - min_torque_nm)).clamp(0.0, 1.0)
}

fn torque_request_from_driver_demand_wot(
    driver_demand: f64,
    torque_curve: &[StartupFitWotTorquePoint],
    fallback_torque_nm: f64,
) -> f64 {
    let demand = finite_f64(driver_demand, DRIVER_DEMAND_DEFAULT).clamp(0.0, 1.0);
    let (min_torque_nm, max_torque_nm) =
        wot_torque_request_bounds(torque_curve, fallback_torque_nm);
    min_torque_nm + demand * (max_torque_nm - min_torque_nm)
}

fn rpm_for_requested_torque(
    requested_brake_torque_nm: f64,
    torque_curve: &[StartupFitWotTorquePoint],
    fallback_rpm: f64,
) -> f64 {
    let mut samples: Vec<(f64, f64)> = torque_curve
        .iter()
        .filter_map(|point| {
            let rpm = finite_f64(point.engine_speed_rpm, f64::NAN);
            let torque_nm = finite_f64(point.available_brake_torque_nm, f64::NAN);
            (rpm.is_finite() && torque_nm.is_finite()).then_some((rpm.max(0.0), torque_nm.max(0.0)))
        })
        .collect();
    samples.sort_by(|a, b| a.0.total_cmp(&b.0));
    samples.dedup_by(|a, b| (a.0 - b.0).abs() <= 1.0e-9);

    let Some((min_rpm, _)) = samples.first().copied() else {
        return fallback_rpm.max(0.0);
    };
    let (max_rpm, _) = samples.last().copied().unwrap_or((min_rpm, 0.0));
    let fallback_rpm = fallback_rpm.clamp(min_rpm, max_rpm).max(0.0);
    let requested_brake_torque_nm = requested_brake_torque_nm.max(0.0);
    let min_available_torque_nm = samples
        .iter()
        .map(|sample| sample.1)
        .fold(f64::INFINITY, f64::min);
    let max_available_torque_nm = samples.iter().map(|sample| sample.1).fold(0.0, f64::max);

    if requested_brake_torque_nm <= min_available_torque_nm {
        return fallback_rpm;
    }
    if requested_brake_torque_nm >= max_available_torque_nm {
        let peak = samples
            .iter()
            .copied()
            .max_by(|a, b| a.1.total_cmp(&b.1))
            .unwrap_or((fallback_rpm, requested_brake_torque_nm));
        return peak.0.max(0.0);
    }

    let mut best_crossing_rpm = None;
    let mut best_crossing_distance = f64::INFINITY;
    for window in samples.windows(2) {
        let (rpm0, torque0) = window[0];
        let (rpm1, torque1) = window[1];
        let rel0 = torque0 - requested_brake_torque_nm;
        let rel1 = torque1 - requested_brake_torque_nm;
        if rel0.abs() <= 1.0e-9 {
            let distance = (rpm0 - fallback_rpm).abs();
            if distance < best_crossing_distance {
                best_crossing_distance = distance;
                best_crossing_rpm = Some(rpm0);
            }
            continue;
        }
        if rel0 * rel1 > 0.0 {
            continue;
        }
        let span = (torque1 - torque0).abs().max(f64::EPSILON);
        let t = ((requested_brake_torque_nm - torque0) / span).clamp(0.0, 1.0);
        let rpm = rpm0 + (rpm1 - rpm0) * t;
        let distance = (rpm - fallback_rpm).abs();
        if distance < best_crossing_distance {
            best_crossing_distance = distance;
            best_crossing_rpm = Some(rpm);
        }
    }

    best_crossing_rpm.unwrap_or(fallback_rpm).max(0.0)
}

fn wot_ignition_for_rpm(
    engine_speed_rpm: f64,
    torque_curve: &[StartupFitWotTorquePoint],
    fallback_ignition_deg: f64,
) -> f64 {
    let samples = sorted_wot_ignition_samples(torque_curve);
    interpolate_curve(engine_speed_rpm, &samples, fallback_ignition_deg)
}

#[cfg_attr(not(test), allow(dead_code))]
fn throttle_for_requested_torque(
    requested_brake_torque_nm: f64,
    torque_curve: &[StartupFitTorqueCurvePoint],
    fallback_throttle_cmd: f64,
) -> f64 {
    let samples = sorted_torque_curve_samples(torque_curve);
    interpolate_curve(requested_brake_torque_nm, &samples, fallback_throttle_cmd).clamp(0.0, 1.0)
}

#[cfg_attr(not(test), allow(dead_code))]
fn torque_request_bounds(
    torque_curve: &[StartupFitTorqueCurvePoint],
    fallback_torque_nm: f64,
) -> (f64, f64) {
    let samples = sorted_torque_curve_samples(torque_curve);
    let Some((min_torque_nm, _)) = samples.first().copied() else {
        let fallback = fallback_torque_nm.max(0.0);
        return (fallback, fallback);
    };
    let max_torque_nm = samples
        .last()
        .map(|sample| sample.0)
        .unwrap_or(min_torque_nm);
    (
        min_torque_nm.max(0.0),
        max_torque_nm.max(min_torque_nm).max(0.0),
    )
}

fn wot_torque_request_bounds(
    torque_curve: &[StartupFitWotTorquePoint],
    fallback_torque_nm: f64,
) -> (f64, f64) {
    let samples = sorted_wot_torque_samples(torque_curve);
    let Some((_, _)) = samples.first().copied() else {
        let fallback = fallback_torque_nm.max(0.0);
        return (0.0, fallback);
    };
    let max_torque_nm = samples.last().map(|sample| sample.0).unwrap_or(0.0);
    (0.0, max_torque_nm.max(0.0))
}

fn sorted_torque_curve_samples(torque_curve: &[StartupFitTorqueCurvePoint]) -> Vec<(f64, f64)> {
    let mut samples: Vec<(f64, f64)> = torque_curve
        .iter()
        .filter_map(|point| {
            let torque_nm = finite_f64(point.required_brake_torque_nm, f64::NAN);
            let throttle_cmd = finite_f64(point.throttle_cmd, f64::NAN);
            (torque_nm.is_finite() && throttle_cmd.is_finite())
                .then_some((torque_nm, throttle_cmd.clamp(0.0, 1.0)))
        })
        .collect();
    samples.sort_by(|a, b| a.0.total_cmp(&b.0));
    samples.dedup_by(|a, b| (a.0 - b.0).abs() <= 1.0e-9);
    samples
}

fn sorted_wot_torque_samples(torque_curve: &[StartupFitWotTorquePoint]) -> Vec<(f64, f64)> {
    let mut samples: Vec<(f64, f64)> = torque_curve
        .iter()
        .filter_map(|point| {
            let torque_nm = finite_f64(point.available_brake_torque_nm, f64::NAN);
            let rpm = finite_f64(point.engine_speed_rpm, f64::NAN);
            (torque_nm.is_finite() && rpm.is_finite()).then_some((torque_nm.max(0.0), rpm.max(0.0)))
        })
        .collect();
    samples.sort_by(|a, b| a.0.total_cmp(&b.0));
    samples.dedup_by(|a, b| (a.0 - b.0).abs() <= 1.0e-9);
    samples
}

fn sorted_wot_ignition_samples(torque_curve: &[StartupFitWotTorquePoint]) -> Vec<(f64, f64)> {
    let mut samples: Vec<(f64, f64)> = torque_curve
        .iter()
        .filter_map(|point| {
            let rpm = finite_f64(point.engine_speed_rpm, f64::NAN);
            let ignition = finite_f64(point.ignition_timing_deg, f64::NAN);
            (rpm.is_finite() && ignition.is_finite()).then_some((rpm.max(0.0), ignition))
        })
        .collect();
    samples.sort_by(|a, b| a.0.total_cmp(&b.0));
    samples.dedup_by(|a, b| (a.0 - b.0).abs() <= 1.0e-9);
    samples
}

fn interpolate_curve(value: f64, samples: &[(f64, f64)], fallback: f64) -> f64 {
    let Some(first) = samples.first().copied() else {
        return fallback;
    };
    if value <= first.0 {
        return first.1;
    }
    let last = samples.last().copied().unwrap_or(first);
    if value >= last.0 {
        return last.1;
    }
    for window in samples.windows(2) {
        let (x0, y0) = window[0];
        let (x1, y1) = window[1];
        if value <= x1 {
            let span = (x1 - x0).abs().max(f64::EPSILON);
            let t = ((value - x0) / span).clamp(0.0, 1.0);
            return y0 + t * (y1 - y0);
        }
    }
    last.1
}

#[cfg(test)]
mod tests {
    use std::fs;
    use std::path::PathBuf;
    use std::thread;
    use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

    use crate::config::{AppConfig, ExternalLoadMode, LoadedAppConfig, load_config};
    use crate::dashboard::startup_fit::{
        StartupFitControls, StartupFitWotTorquePoint, startup_fit_workload_contract,
    };
    use crate::dashboard::startup_fit_artifact::{
        StartupFitArtifact, StartupFitArtifactEvaluation, StartupFitArtifactSnapshot,
        StartupFitCacheContext, save_artifact,
    };

    use super::{
        DRIVER_DEMAND_DEFAULT, DashboardState, PostFitRuntimeMode, STARTUP_FIT_TARGET_RPM,
        StartupFitTorqueCurvePoint, driver_demand_for_torque_request,
        driver_demand_for_wot_torque_request, fit_speed_hold_target_torque_nm,
        rpm_for_requested_torque, shortest_cycle_delta_deg, throttle_for_requested_torque,
        torque_request_from_driver_demand, torque_request_from_driver_demand_wot, wrap_cycle_deg,
    };

    fn unique_temp_root() -> PathBuf {
        let nanos = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .expect("system time before unix epoch")
            .as_nanos();
        std::env::temp_dir().join(format!("es_sim_dashboard_state_cache_test_{nanos}"))
    }

    #[test]
    fn cycle_wrap_helpers_follow_shortest_direction() {
        assert!((wrap_cycle_deg(725.0) - 5.0).abs() < 1.0e-12);
        assert!((shortest_cycle_delta_deg(710.0, 8.0) - 18.0).abs() < 1.0e-12);
        assert!((shortest_cycle_delta_deg(8.0, 710.0) + 18.0).abs() < 1.0e-12);
    }

    #[test]
    fn background_pressure_sampling_stays_bounded() {
        assert_eq!(super::background_pv_subsamples_per_step(120), 10);
        assert_eq!(super::background_pv_subsamples_per_step(12), 6);
        assert_eq!(super::background_pv_subsamples_per_step(4), 4);
    }

    #[test]
    fn interaction_budget_is_tighter_than_default_runtime_budget() {
        let runtime = super::frame_wall_budget_s(false, false, 0.0, false);
        let interaction = super::frame_wall_budget_s(false, false, 0.0, true);
        let startup_fit = super::frame_wall_budget_s(
            true,
            false,
            super::STARTUP_FIT_HEADLESS_BUDGET_DISABLE_THRESHOLD_S,
            true,
        );
        assert!(interaction < runtime);
        assert!((startup_fit - super::STARTUP_FIT_FRAME_WALL_BUDGET_S).abs() < 1.0e-12);
    }

    #[test]
    fn dashboard_state_starts_with_startup_fit_enabled() {
        let cfg = AppConfig::default();
        let state = DashboardState::new(cfg);

        assert!((state.load_target_rpm - STARTUP_FIT_TARGET_RPM).abs() < f64::EPSILON);
        assert!((state.driver_demand - DRIVER_DEMAND_DEFAULT).abs() < f64::EPSILON);
        assert_eq!(state.post_fit_mode, PostFitRuntimeMode::StandardRuntime);
        assert_eq!(
            state.sim.model.external_load.mode,
            ExternalLoadMode::BrakeMap
        );
        assert!(state.sim.control.spark_cmd);
        assert!(state.sim.control.fuel_cmd);
        assert!(state.sim.control.load_cmd.abs() < f64::EPSILON);
        assert!(state.startup_fit_active());
    }

    #[test]
    fn dashboard_state_overrides_manual_load_while_fit_active() {
        let cfg = AppConfig::default();
        let ui = cfg.ui.clone();
        let mut state = DashboardState::new(cfg);

        state.sim.control.load_cmd = 0.75;
        state.advance_simulation(&ui);

        assert!(state.sim.control.load_cmd < 0.75);
    }

    #[test]
    fn dashboard_state_skips_startup_fit_when_cached_artifact_matches() {
        let root = unique_temp_root();
        let loaded = LoadedAppConfig {
            config: AppConfig::default(),
            resolved_path: Some(root.join("config").join("sim.yaml")),
            source_text: Some("engine:\n  max_rpm: 7000\n".to_owned()),
        };
        let context =
            StartupFitCacheContext::from_loaded_config(&loaded).expect("cache context exists");
        let artifact = StartupFitArtifact::new(
            &context,
            StartupFitArtifactSnapshot {
                target_rpm: STARTUP_FIT_TARGET_RPM,
                timed_out: false,
                release_controls: StartupFitControls {
                    throttle_cmd: 0.24,
                    ignition_timing_deg: 20.5,
                    vvt_intake_deg: 0.0,
                    vvt_exhaust_deg: 0.0,
                    load_cmd: 0.18,
                },
                release_evaluation: StartupFitArtifactEvaluation {
                    avg_rpm: 1_999.0,
                    avg_net_torque_nm: 0.2,
                    required_brake_torque_nm: 18.6,
                    load_cmd: 0.18,
                    periodic_error_norm: 0.010,
                    converged: true,
                },
                best_required_brake_torque_nm: 18.8,
                torque_margin_to_best_nm: 0.2,
                torque_curve: vec![
                    StartupFitTorqueCurvePoint {
                        throttle_cmd: 0.20,
                        required_brake_torque_nm: 15.0,
                    },
                    StartupFitTorqueCurvePoint {
                        throttle_cmd: 0.24,
                        required_brake_torque_nm: 18.6,
                    },
                    StartupFitTorqueCurvePoint {
                        throttle_cmd: 0.32,
                        required_brake_torque_nm: 25.0,
                    },
                ],
                wot_torque_curve: vec![
                    StartupFitWotTorquePoint {
                        engine_speed_rpm: 1_500.0,
                        available_brake_torque_nm: 15.0,
                        ignition_timing_deg: 18.0,
                    },
                    StartupFitWotTorquePoint {
                        engine_speed_rpm: 2_000.0,
                        available_brake_torque_nm: 18.6,
                        ignition_timing_deg: 20.5,
                    },
                    StartupFitWotTorquePoint {
                        engine_speed_rpm: 3_000.0,
                        available_brake_torque_nm: 25.0,
                        ignition_timing_deg: 24.0,
                    },
                ],
            },
        );
        save_artifact(&context, &artifact).expect("artifact saved");

        let state = DashboardState::from_loaded_config(loaded);
        let status = state.startup_fit_status();

        assert!(!state.startup_fit_active());
        assert!(status.loaded_from_cache);
        assert_eq!(
            status.phase,
            super::super::startup_fit::StartupFitPhase::Ready
        );
        assert!((status.release_throttle_cmd - 0.24).abs() < 1.0e-12);
        assert!((status.release_required_brake_torque_nm - 18.6).abs() < 1.0e-12);

        let _ = fs::remove_dir_all(root);
    }

    #[test]
    fn cached_ready_state_syncs_live_controls_to_post_fit_baseline() {
        let root = unique_temp_root();
        let loaded = LoadedAppConfig {
            config: AppConfig::default(),
            resolved_path: Some(root.join("config").join("sim.yaml")),
            source_text: Some("engine:\n  max_rpm: 7000\n".to_owned()),
        };
        let context =
            StartupFitCacheContext::from_loaded_config(&loaded).expect("cache context exists");
        let artifact = StartupFitArtifact::new(
            &context,
            StartupFitArtifactSnapshot {
                target_rpm: STARTUP_FIT_TARGET_RPM,
                timed_out: false,
                release_controls: StartupFitControls {
                    throttle_cmd: 1.0,
                    ignition_timing_deg: 22.8,
                    vvt_intake_deg: 0.0,
                    vvt_exhaust_deg: 0.0,
                    load_cmd: 0.18,
                },
                release_evaluation: StartupFitArtifactEvaluation {
                    avg_rpm: 2_000.0,
                    avg_net_torque_nm: 0.2,
                    required_brake_torque_nm: 21.9,
                    load_cmd: 0.18,
                    periodic_error_norm: 0.010,
                    converged: true,
                },
                best_required_brake_torque_nm: 21.9,
                torque_margin_to_best_nm: 0.0,
                torque_curve: vec![StartupFitTorqueCurvePoint {
                    throttle_cmd: 1.0,
                    required_brake_torque_nm: 21.9,
                }],
                wot_torque_curve: vec![StartupFitWotTorquePoint {
                    engine_speed_rpm: 2_000.0,
                    available_brake_torque_nm: 21.9,
                    ignition_timing_deg: 15.2,
                }],
            },
        );
        save_artifact(&context, &artifact).expect("artifact saved");

        let state = DashboardState::from_loaded_config(loaded);

        assert!(!state.startup_fit_active());
        assert!(
            (state.post_fit_baseline.ignition_timing_deg - 15.2).abs() < 1.0e-12,
            "post-fit auto ignition should come from the cached WOT map"
        );
        assert!(
            (state.sim.control.ignition_timing_deg - state.post_fit_baseline.ignition_timing_deg)
                .abs()
                < 1.0e-12,
            "live ignition should be synchronized to the standard-runtime baseline"
        );
        assert!(
            (state.post_fit_baseline.requested_brake_torque_nm - 21.9).abs() < 1.0e-12,
            "cached ready state should carry a nonzero requested torque into runtime"
        );

        let _ = fs::remove_dir_all(root);
    }

    #[test]
    fn fit_speed_hold_trims_around_reference_brake_torque() {
        let low = fit_speed_hold_target_torque_nm(20.0, 1_900.0, 2_000.0);
        let nominal = fit_speed_hold_target_torque_nm(20.0, 2_000.0, 2_000.0);
        let high = fit_speed_hold_target_torque_nm(20.0, 2_100.0, 2_000.0);

        assert!(high > low);
        assert!((nominal - 20.0).abs() < 1.0e-12);
        assert!(low >= 0.0);
    }

    #[test]
    fn driver_demand_maps_across_curve_bounds() {
        let curve = [
            StartupFitTorqueCurvePoint {
                throttle_cmd: 0.18,
                required_brake_torque_nm: 12.0,
            },
            StartupFitTorqueCurvePoint {
                throttle_cmd: 0.42,
                required_brake_torque_nm: 28.0,
            },
            StartupFitTorqueCurvePoint {
                throttle_cmd: 0.68,
                required_brake_torque_nm: 44.0,
            },
        ];

        assert!((torque_request_from_driver_demand(0.0, &curve, 0.0) - 12.0).abs() < 1.0e-12);
        assert!((torque_request_from_driver_demand(1.0, &curve, 0.0) - 44.0).abs() < 1.0e-12);
        assert!((driver_demand_for_torque_request(28.0, &curve, 0.0) - 0.5).abs() < 1.0e-12);
    }

    #[test]
    fn throttle_interpolates_from_requested_torque() {
        let curve = [
            StartupFitTorqueCurvePoint {
                throttle_cmd: 0.22,
                required_brake_torque_nm: 16.0,
            },
            StartupFitTorqueCurvePoint {
                throttle_cmd: 0.40,
                required_brake_torque_nm: 26.0,
            },
            StartupFitTorqueCurvePoint {
                throttle_cmd: 0.72,
                required_brake_torque_nm: 46.0,
            },
        ];

        let interpolated = throttle_for_requested_torque(21.0, &curve, 0.30);
        assert!(
            (interpolated - 0.31).abs() < 1.0e-12,
            "expected 0.31, got {interpolated}"
        );
    }

    #[test]
    fn driver_demand_maps_across_wot_curve_bounds() {
        let curve = [
            StartupFitWotTorquePoint {
                engine_speed_rpm: 1_500.0,
                available_brake_torque_nm: 18.0,
                ignition_timing_deg: 18.0,
            },
            StartupFitWotTorquePoint {
                engine_speed_rpm: 3_000.0,
                available_brake_torque_nm: 42.0,
                ignition_timing_deg: 24.0,
            },
            StartupFitWotTorquePoint {
                engine_speed_rpm: 6_000.0,
                available_brake_torque_nm: 58.0,
                ignition_timing_deg: 28.0,
            },
        ];

        assert!((torque_request_from_driver_demand_wot(0.0, &curve, 0.0) - 0.0).abs() < 1.0e-12);
        assert!((torque_request_from_driver_demand_wot(1.0, &curve, 0.0) - 58.0).abs() < 1.0e-12);
        assert!((driver_demand_for_wot_torque_request(0.0, &curve, 0.0) - 0.0).abs() < 1.0e-12);
        assert!(
            (driver_demand_for_wot_torque_request(42.0, &curve, 0.0) - (42.0 / 58.0)).abs()
                < 1.0e-12
        );
    }

    #[test]
    fn equilibrium_rpm_interpolates_from_wot_curve() {
        let curve = [
            StartupFitWotTorquePoint {
                engine_speed_rpm: 1_500.0,
                available_brake_torque_nm: 18.0,
                ignition_timing_deg: 18.0,
            },
            StartupFitWotTorquePoint {
                engine_speed_rpm: 3_000.0,
                available_brake_torque_nm: 42.0,
                ignition_timing_deg: 24.0,
            },
            StartupFitWotTorquePoint {
                engine_speed_rpm: 6_000.0,
                available_brake_torque_nm: 58.0,
                ignition_timing_deg: 28.0,
            },
        ];

        let interpolated_rpm = rpm_for_requested_torque(30.0, &curve, 2_000.0);
        assert!(
            (interpolated_rpm - 2_250.0).abs() < 1.0e-12,
            "expected 2250 rpm, got {interpolated_rpm}"
        );
    }

    #[test]
    fn equilibrium_rpm_uses_fallback_branch_below_wot_envelope() {
        let curve = [
            StartupFitWotTorquePoint {
                engine_speed_rpm: 1_000.0,
                available_brake_torque_nm: 170.0,
                ignition_timing_deg: 18.0,
            },
            StartupFitWotTorquePoint {
                engine_speed_rpm: 2_000.0,
                available_brake_torque_nm: 186.0,
                ignition_timing_deg: 22.0,
            },
            StartupFitWotTorquePoint {
                engine_speed_rpm: 3_000.0,
                available_brake_torque_nm: 215.0,
                ignition_timing_deg: 24.0,
            },
            StartupFitWotTorquePoint {
                engine_speed_rpm: 4_000.0,
                available_brake_torque_nm: 235.0,
                ignition_timing_deg: 26.0,
            },
            StartupFitWotTorquePoint {
                engine_speed_rpm: 5_000.0,
                available_brake_torque_nm: 180.0,
                ignition_timing_deg: 28.0,
            },
        ];

        let interpolated_rpm = rpm_for_requested_torque(21.9, &curve, 2_000.0);
        assert!(
            (interpolated_rpm - 2_000.0).abs() < 1.0e-12,
            "low torque requests should stay on the fallback branch instead of jumping to the WOT envelope edge"
        );
    }

    #[test]
    #[ignore = "expensive headless startup-fit convergence run"]
    fn startup_fit_reaches_ready_and_holds_target_on_checked_in_config() {
        let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("config")
            .join("sim.yaml");
        let mut cfg = load_config(&path);
        cfg.ui.sync_to_wall_clock = false;
        cfg.ui.simulated_time_per_frame_s = 0.05;
        cfg.ui.max_steps_per_frame = 20_000;

        let ui = cfg.ui.clone();
        let mut state = DashboardState::new(cfg);
        let contract = startup_fit_workload_contract();
        let deadline = Instant::now() + Duration::from_secs_f64(contract.target_wall_time_s + 30.0);
        let mut ready_status = None;
        let mut frame = 0usize;

        while Instant::now() < deadline {
            state.advance_simulation(&ui);
            let status = state.startup_fit_status();
            if frame % 200 == 0 {
                println!(
                    "frame={frame} phase={:?} iter={}/{} stable={}/{} avg_rpm={:.1} req_brake={:+.1} sim_elapsed={:.1}s",
                    status.phase,
                    status.iteration,
                    status.max_iterations,
                    status.stable_windows,
                    status.required_stable_windows,
                    status.avg_rpm,
                    status.required_brake_torque_nm,
                    status.simulated_elapsed_s
                );
            }
            if !status.active {
                ready_status = Some(status);
                break;
            }
            frame += 1;
            thread::sleep(Duration::from_millis(1));
        }

        let status = ready_status.unwrap_or_else(|| {
            let status = state.startup_fit_status();
            panic!(
                "startup fit did not reach READY: phase={:?} iter={}/{} stable={}/{} avg_rpm={:.1} req_brake={:+.1} sim_elapsed={:.1}s",
                status.phase,
                status.iteration,
                status.max_iterations,
                status.stable_windows,
                status.required_stable_windows,
                status.avg_rpm,
                status.required_brake_torque_nm,
                status.simulated_elapsed_s
            );
        });

        assert!(
            !status.timed_out,
            "startup fit hit the wall-time cap after {:.1}s",
            status.wall_elapsed_s
        );
        assert!(
            (status.avg_rpm - STARTUP_FIT_TARGET_RPM).abs() <= 24.0,
            "verify average rpm drifted too far: avg_rpm={:.2}",
            status.avg_rpm
        );
        assert!(
            status.avg_net_torque_nm.abs() <= 2.0,
            "verify net torque stayed too large: net={:+.3}",
            status.avg_net_torque_nm
        );

        let mut rpm_sum = 0.0;
        let mut samples = 0usize;
        for _ in 0..400 {
            state.advance_simulation(&ui);
            rpm_sum += state.latest.rpm;
            samples += 1;
        }
        let avg_hold_rpm = rpm_sum / samples as f64;
        assert!(
            (avg_hold_rpm - STARTUP_FIT_TARGET_RPM).abs() <= 40.0,
            "post-fit hold drifted too far: avg_hold_rpm={:.2}",
            avg_hold_rpm
        );
        assert!(
            state.latest.rpm > 500.0,
            "post-fit live rpm collapsed unexpectedly: {:.2}",
            state.latest.rpm
        );
    }

    #[test]
    #[ignore = "diagnostic trace for startup-fit verify behavior"]
    fn startup_fit_debug_verify_trace() {
        let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("config")
            .join("sim.yaml");
        let mut cfg = load_config(&path);
        cfg.ui.sync_to_wall_clock = false;
        cfg.ui.simulated_time_per_frame_s = 0.05;
        cfg.ui.max_steps_per_frame = 20_000;

        let ui = cfg.ui.clone();
        let mut state = DashboardState::new(cfg);
        let mut last_label = String::new();
        for frame in 0..2_000usize {
            state.advance_simulation(&ui);
            let status = state.startup_fit_status();
            if status.candidate_label != last_label || frame % 20 == 0 {
                println!(
                    "frame={frame} phase={:?} label={} avg_rpm={:.1} live_rpm={:.1} req={:+.1} rel_req={:+.1} load_cmd={:+.3} rel_thr={:.3} rel_spk={:.1}",
                    status.phase,
                    status.candidate_label,
                    status.avg_rpm,
                    state.latest.rpm,
                    status.required_brake_torque_nm,
                    status.release_required_brake_torque_nm,
                    state.sim.control.load_cmd,
                    status.release_throttle_cmd,
                    status.release_ignition_timing_deg,
                );
                last_label = status.candidate_label.clone();
            }
            if !status.active {
                break;
            }
        }
    }
}
