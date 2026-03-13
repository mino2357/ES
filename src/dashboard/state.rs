use std::time::Instant;

use eframe::egui;

use super::startup_fit::{
    ROUGH_IDLE_FAST_FORWARD_FACTOR, ROUGH_IDLE_INITIAL_THROTTLE, ROUGH_IDLE_MAX_ITERATIONS,
    ROUGH_IDLE_STABLE_NET_TORQUE_NM, ROUGH_IDLE_STABLE_RPM_ERR, ROUGH_IDLE_STABLE_WINDOWS_REQUIRED,
    ROUGH_IDLE_TARGET_RPM, RoughCalibrationPhase, RoughCalibrationState, RoughCalibrationStatus,
};
use crate::config::{AppConfig, UiConfig};
use crate::simulator::{
    Observation, Simulator, accuracy_priority_dt, estimate_mbt_deg, estimate_realtime_performance,
    rad_s_to_rpm, rpm_linked_dt, shaft_power_hp, shaft_power_kw,
};

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
    rough_calibration: RoughCalibrationState,
}

impl DashboardState {
    pub(super) fn new(config: AppConfig) -> Self {
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
        let load_target_rpm = ROUGH_IDLE_TARGET_RPM;
        let initial_load_est = 0.60;
        let ignition_timing_deg = estimate_mbt_deg(&sim.model, load_target_rpm, initial_load_est);
        sim.control.spark_cmd = true;
        sim.control.fuel_cmd = true;
        sim.control.vvt_intake_deg = 0.0;
        sim.control.vvt_exhaust_deg = 0.0;
        sim.control.load_cmd = 0.0;
        sim.seed_operating_point(
            load_target_rpm,
            ROUGH_IDLE_INITIAL_THROTTLE,
            ignition_timing_deg,
        );
        let latest = sim.step(dt_next);
        Self {
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
            rough_calibration: RoughCalibrationState::new(dt_next, load_target_rpm),
        }
    }

    pub(super) fn rpm_error(&self) -> f64 {
        self.load_target_rpm - self.latest.rpm
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

    pub(super) fn rough_calibration_active(&self) -> bool {
        self.rough_calibration.is_active()
    }

    pub(super) fn rough_calibration_status(&self) -> RoughCalibrationStatus {
        self.rough_calibration.snapshot(
            self.simulated_time_s,
            self.sim.control.throttle_cmd,
            self.sim.control.ignition_timing_deg,
        )
    }

    pub(super) fn apply_shortcuts(&mut self, ctx: &egui::Context, ui_config: &UiConfig) {
        let manual_shortcuts_enabled = !self.rough_calibration_active();
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
                    self.sim.control.throttle_cmd = (self.sim.control.throttle_cmd
                        + ui_config.throttle_key_step)
                        .clamp(0.0, 1.0);
                }
                if i.key_pressed(egui::Key::X) {
                    self.sim.control.throttle_cmd = (self.sim.control.throttle_cmd
                        - ui_config.throttle_key_step)
                        .clamp(0.0, 1.0);
                }
            }
        });
    }

    pub(super) fn advance_simulation(&mut self, ui_config: &UiConfig) {
        let now = Instant::now();
        let elapsed_since_last_frame_s = (now - self.last_tick).as_secs_f64();
        let mut target_time = if ui_config.sync_to_wall_clock {
            elapsed_since_last_frame_s.max(self.dt_base)
        } else {
            ui_config.simulated_time_per_frame_s.max(self.dt_base)
        };
        if self.rough_calibration_active() && !ui_config.sync_to_wall_clock {
            target_time *= ROUGH_IDLE_FAST_FORWARD_FACTOR;
        }
        let mut simulated = 0.0;
        let mut steps = 0usize;
        let max_steps = ui_config.max_steps_per_frame.max(1);
        // In accuracy-first mode the solver advances a fixed slice of simulated time per frame
        // and does not try to stay phase-locked to wall clock.
        while simulated < target_time && steps < max_steps {
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

            self.apply_load_input();
            self.latest = self.sim.step(dt_step);
            self.dt_next = self
                .realtime_fixed_dt_s
                .unwrap_or_else(|| dt_smoothed.clamp(self.dt_min_bound, self.dt_max_bound));
            simulated += dt_step;
            steps += 1;
        }

        if simulated < target_time {
            // Render the newest computed state even if the requested simulated horizon was not
            // fully consumed in this frame.
            let dt_tail = (target_time - simulated)
                .max(ui_config.dt_epsilon_s)
                .min(self.dt_max_bound);
            self.apply_load_input();
            self.latest = self.sim.step(dt_tail);
            self.dt_next = self
                .realtime_fixed_dt_s
                .unwrap_or_else(|| dt_tail.clamp(self.dt_min_bound, self.dt_max_bound));
            simulated += dt_tail;
        }
        self.simulated_time_s += simulated;
        self.advance_rough_calibration(simulated);
        self.sanitize_control_inputs();
        self.last_tick = now;
    }

    fn apply_load_input(&mut self) {
        // The simple dashboard runs the engine against its internal losses only.
        // External dyno torque is held at zero so startup fit reflects throttle/spark behavior.
        self.sim.control.load_cmd = 0.0;
    }

    fn advance_rough_calibration(&mut self, frame_simulated_s: f64) {
        if !self.rough_calibration.is_active() {
            return;
        }

        self.rough_calibration.record_sample(
            frame_simulated_s,
            self.latest.rpm,
            self.latest.torque_net_nm,
        );

        if self
            .rough_calibration
            .should_leave_priming(self.simulated_time_s)
        {
            self.rough_calibration
                .set_phase(RoughCalibrationPhase::Searching);
        }
        if self.rough_calibration.phase() == RoughCalibrationPhase::Priming {
            return;
        }

        let Some((avg_rpm, avg_net_torque_nm)) = self.rough_calibration.take_window_average()
        else {
            return;
        };

        let avg_rpm = finite_f64(avg_rpm, self.rough_calibration.target_rpm());
        let avg_net_torque_nm = finite_f64(avg_net_torque_nm, 0.0);
        let rpm_error = self.rough_calibration.target_rpm() - avg_rpm;
        let load_est = finite_f64(
            self.latest.map_kpa * 1.0e3 / self.sim.env.ambient_pressure_pa,
            0.60,
        )
        .clamp(self.sim.model.load_min, self.sim.model.load_max);
        self.sim.control.ignition_timing_deg =
            estimate_mbt_deg(&self.sim.model, avg_rpm.max(600.0), load_est);

        let throttle_step =
            finite_f64(0.00012 * rpm_error - 0.0016 * avg_net_torque_nm, 0.0).clamp(-0.03, 0.03);
        self.sim.control.throttle_cmd = finite_f64(
            self.sim.control.throttle_cmd + throttle_step,
            ROUGH_IDLE_INITIAL_THROTTLE,
        )
        .clamp(0.08, 0.55);

        if avg_rpm < self.rough_calibration.target_rpm() * 0.7 {
            self.sim.control.throttle_cmd = self.sim.control.throttle_cmd.max(0.18);
        }

        let stable = rpm_error.abs() <= ROUGH_IDLE_STABLE_RPM_ERR
            && avg_net_torque_nm.abs() <= ROUGH_IDLE_STABLE_NET_TORQUE_NM
            && finite_f64(self.latest.rpm, 0.0) > 1_500.0;
        if stable {
            self.rough_calibration.bump_stable_windows();
            self.rough_calibration.set_phase(
                if self.rough_calibration.stable_windows() >= ROUGH_IDLE_STABLE_WINDOWS_REQUIRED {
                    RoughCalibrationPhase::Ready
                } else {
                    RoughCalibrationPhase::Settling
                },
            );
        } else {
            self.rough_calibration.set_stable_windows(0);
            self.rough_calibration
                .set_phase(RoughCalibrationPhase::Searching);
        }

        if self.rough_calibration.iteration() >= ROUGH_IDLE_MAX_ITERATIONS {
            self.rough_calibration
                .set_phase(RoughCalibrationPhase::Ready);
        }
    }

    fn sanitize_control_inputs(&mut self) {
        self.load_target_rpm = finite_f64(self.load_target_rpm, ROUGH_IDLE_TARGET_RPM)
            .clamp(0.0, self.sim.params.max_rpm);
        self.sim.control.throttle_cmd =
            finite_f64(self.sim.control.throttle_cmd, ROUGH_IDLE_INITIAL_THROTTLE).clamp(0.0, 1.0);
        self.sim.control.ignition_timing_deg = finite_f64(
            self.sim.control.ignition_timing_deg,
            estimate_mbt_deg(&self.sim.model, self.load_target_rpm.max(600.0), 0.60),
        )
        .clamp(self.sim.model.mbt_min_deg, self.sim.model.mbt_max_deg);
        self.sim.control.vvt_intake_deg =
            finite_f64(self.sim.control.vvt_intake_deg, 0.0).clamp(-90.0, 90.0);
        self.sim.control.vvt_exhaust_deg =
            finite_f64(self.sim.control.vvt_exhaust_deg, 0.0).clamp(-90.0, 90.0);
        self.sim.control.load_cmd = 0.0;
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

#[cfg(test)]
mod tests {
    use crate::config::AppConfig;

    use super::{DashboardState, ROUGH_IDLE_TARGET_RPM, shortest_cycle_delta_deg, wrap_cycle_deg};

    #[test]
    fn cycle_wrap_helpers_follow_shortest_direction() {
        assert!((wrap_cycle_deg(725.0) - 5.0).abs() < 1.0e-12);
        assert!((shortest_cycle_delta_deg(710.0, 8.0) - 18.0).abs() < 1.0e-12);
        assert!((shortest_cycle_delta_deg(8.0, 710.0) + 18.0).abs() < 1.0e-12);
    }

    #[test]
    fn dashboard_state_starts_with_rough_2000rpm_calibration() {
        let cfg = AppConfig::default();
        let state = DashboardState::new(cfg);

        assert!((state.load_target_rpm - ROUGH_IDLE_TARGET_RPM).abs() < f64::EPSILON);
        assert!(state.sim.control.spark_cmd);
        assert!(state.sim.control.fuel_cmd);
        assert!(state.sim.control.load_cmd.abs() < f64::EPSILON);
        assert!(state.rough_calibration_active());
    }

    #[test]
    fn dashboard_state_keeps_external_load_disabled() {
        let cfg = AppConfig::default();
        let ui = cfg.ui.clone();
        let mut state = DashboardState::new(cfg);

        state.sim.control.load_cmd = 0.75;
        state.advance_simulation(&ui);

        assert!(state.sim.control.load_cmd.abs() < f64::EPSILON);
    }
}
