pub(super) const ROUGH_IDLE_TARGET_RPM: f64 = 2_000.0;
pub(super) const ROUGH_IDLE_INITIAL_THROTTLE: f64 = 0.22;
pub(super) const ROUGH_IDLE_WINDOW_S: f64 = 0.24;
pub(super) const ROUGH_IDLE_PRIMING_S: f64 = 0.40;
pub(super) const ROUGH_IDLE_FAST_FORWARD_FACTOR: f64 = 3.0;
pub(super) const ROUGH_IDLE_MAX_ITERATIONS: usize = 18;
pub(super) const ROUGH_IDLE_STABLE_RPM_ERR: f64 = 75.0;
pub(super) const ROUGH_IDLE_STABLE_NET_TORQUE_NM: f64 = 6.0;
pub(super) const ROUGH_IDLE_STABLE_WINDOWS_REQUIRED: usize = 3;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum RoughCalibrationPhase {
    Priming,
    Searching,
    Settling,
    Ready,
}

impl RoughCalibrationPhase {
    pub(super) fn label(self) -> &'static str {
        match self {
            Self::Priming => "priming",
            Self::Searching => "searching",
            Self::Settling => "settling",
            Self::Ready => "ready",
        }
    }

    pub(super) fn detail(self) -> &'static str {
        match self {
            Self::Priming => "settling the fired initial condition near 2000 rpm",
            Self::Searching => "trimming throttle for a self-sustaining 2000 rpm point",
            Self::Settling => "checking that 2000 rpm holds without external assist",
            Self::Ready => "rough 2000 rpm fit is holding and ready for manual trim",
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub(super) struct RoughCalibrationStatus {
    pub(super) active: bool,
    pub(super) phase: RoughCalibrationPhase,
    pub(super) target_rpm: f64,
    pub(super) simulated_elapsed_s: f64,
    pub(super) iteration: usize,
    pub(super) max_iterations: usize,
    pub(super) stable_windows: usize,
    pub(super) required_stable_windows: usize,
    pub(super) avg_rpm: f64,
    pub(super) avg_net_torque_nm: f64,
    pub(super) throttle_cmd: f64,
    pub(super) ignition_timing_deg: f64,
}

#[derive(Debug, Clone)]
pub(super) struct RoughCalibrationState {
    phase: RoughCalibrationPhase,
    target_rpm: f64,
    started_sim_time_s: f64,
    window_elapsed_s: f64,
    window_samples: usize,
    rpm_sum: f64,
    net_torque_sum_nm: f64,
    iteration: usize,
    stable_windows: usize,
    last_avg_rpm: f64,
    last_avg_net_torque_nm: f64,
}

impl RoughCalibrationState {
    pub(super) fn new(started_sim_time_s: f64, target_rpm: f64) -> Self {
        Self {
            phase: RoughCalibrationPhase::Priming,
            target_rpm,
            started_sim_time_s,
            window_elapsed_s: 0.0,
            window_samples: 0,
            rpm_sum: 0.0,
            net_torque_sum_nm: 0.0,
            iteration: 0,
            stable_windows: 0,
            last_avg_rpm: target_rpm,
            last_avg_net_torque_nm: 0.0,
        }
    }

    pub(super) fn phase(&self) -> RoughCalibrationPhase {
        self.phase
    }

    pub(super) fn set_phase(&mut self, phase: RoughCalibrationPhase) {
        self.phase = phase;
    }

    pub(super) fn target_rpm(&self) -> f64 {
        self.target_rpm
    }

    pub(super) fn iteration(&self) -> usize {
        self.iteration
    }

    pub(super) fn stable_windows(&self) -> usize {
        self.stable_windows
    }

    pub(super) fn set_stable_windows(&mut self, stable_windows: usize) {
        self.stable_windows = stable_windows;
    }

    pub(super) fn bump_stable_windows(&mut self) {
        self.stable_windows = self.stable_windows.saturating_add(1);
    }

    pub(super) fn is_active(&self) -> bool {
        self.phase != RoughCalibrationPhase::Ready
    }

    pub(super) fn record_sample(&mut self, frame_simulated_s: f64, rpm: f64, net_torque_nm: f64) {
        let dt = finite_f64(frame_simulated_s, 0.0).max(0.0);
        self.window_elapsed_s += dt;
        if rpm.is_finite() && net_torque_nm.is_finite() {
            self.window_samples = self.window_samples.saturating_add(1);
            self.rpm_sum += rpm;
            self.net_torque_sum_nm += net_torque_nm;
        }
    }

    pub(super) fn should_leave_priming(&self, simulated_time_s: f64) -> bool {
        self.phase == RoughCalibrationPhase::Priming
            && simulated_time_s - self.started_sim_time_s >= ROUGH_IDLE_PRIMING_S
    }

    pub(super) fn take_window_average(&mut self) -> Option<(f64, f64)> {
        if self.window_elapsed_s < ROUGH_IDLE_WINDOW_S || self.window_samples == 0 {
            return None;
        }
        let avg_rpm = self.rpm_sum / self.window_samples as f64;
        let avg_net_torque_nm = self.net_torque_sum_nm / self.window_samples as f64;
        self.last_avg_rpm = avg_rpm;
        self.last_avg_net_torque_nm = avg_net_torque_nm;
        self.window_elapsed_s = 0.0;
        self.window_samples = 0;
        self.rpm_sum = 0.0;
        self.net_torque_sum_nm = 0.0;
        self.iteration = self.iteration.saturating_add(1);
        Some((avg_rpm, avg_net_torque_nm))
    }

    pub(super) fn snapshot(
        &self,
        simulated_time_s: f64,
        throttle_cmd: f64,
        ignition_timing_deg: f64,
    ) -> RoughCalibrationStatus {
        RoughCalibrationStatus {
            active: self.is_active(),
            phase: self.phase,
            target_rpm: self.target_rpm,
            simulated_elapsed_s: (simulated_time_s - self.started_sim_time_s).max(0.0),
            iteration: self.iteration,
            max_iterations: ROUGH_IDLE_MAX_ITERATIONS,
            stable_windows: self.stable_windows,
            required_stable_windows: ROUGH_IDLE_STABLE_WINDOWS_REQUIRED,
            avg_rpm: self.last_avg_rpm,
            avg_net_torque_nm: self.last_avg_net_torque_nm,
            throttle_cmd,
            ignition_timing_deg,
        }
    }
}

fn finite_f64(value: f64, fallback: f64) -> f64 {
    if value.is_finite() { value } else { fallback }
}
