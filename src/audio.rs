use std::f32::consts::TAU as TAU_F32;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use rodio::{OutputStream, Sink, Source};

use crate::config::{AudioConfig, AudioModelConfig};
use crate::constants::{FIXED_CYLINDER_COUNT, GAMMA_AIR, R_AIR};

// Audio input is the reduced engine state exported by the simulator at GUI cadence.
#[derive(Debug, Clone, Copy)]
pub(crate) struct AudioParams {
    pub(crate) exhaust_pressure_kpa: f32,
    pub(crate) exhaust_runner_pressure_kpa: f32,
    pub(crate) intake_runner_pressure_kpa: f32,
    pub(crate) exhaust_runner_flow_gps: f32,
    pub(crate) engine_speed_rpm: f32,
    pub(crate) exhaust_temp_k: f32,
    pub(crate) output_gain: f32,
}

impl Default for AudioParams {
    fn default() -> Self {
        Self {
            exhaust_pressure_kpa: 101.325,
            exhaust_runner_pressure_kpa: 101.325,
            intake_runner_pressure_kpa: 101.325,
            exhaust_runner_flow_gps: 0.0,
            engine_speed_rpm: 0.0,
            exhaust_temp_k: 880.0,
            output_gain: 1.0,
        }
    }
}

// Each resonator is a retunable RBJ-style band-pass section used to color the pulse train.
#[derive(Clone, Copy)]
struct Resonator {
    b0: f32,
    b1: f32,
    b2: f32,
    a1: f32,
    a2: f32,
    x1: f32,
    x2: f32,
    y1: f32,
    y2: f32,
}

impl Resonator {
    fn new(
        sample_rate: f32,
        freq_hz: f32,
        q: f32,
        freq_min_hz: f32,
        freq_max_nyquist_ratio: f32,
        q_min: f32,
    ) -> Self {
        let mut r = Self {
            b0: 0.0,
            b1: 0.0,
            b2: 0.0,
            a1: 0.0,
            a2: 0.0,
            x1: 0.0,
            x2: 0.0,
            y1: 0.0,
            y2: 0.0,
        };
        r.set_band_pass(
            sample_rate,
            freq_hz,
            q,
            freq_min_hz,
            freq_max_nyquist_ratio,
            q_min,
        );
        r
    }

    fn set_band_pass(
        &mut self,
        sample_rate: f32,
        freq_hz: f32,
        q: f32,
        freq_min_hz: f32,
        freq_max_nyquist_ratio: f32,
        q_min: f32,
    ) {
        let f = freq_hz.clamp(freq_min_hz, sample_rate * freq_max_nyquist_ratio);
        let q = q.max(q_min);
        let w0 = TAU_F32 * f / sample_rate;
        let alpha = w0.sin() / (2.0 * q);

        let b0 = alpha;
        let b1 = 0.0;
        let b2 = -alpha;
        let a0 = 1.0 + alpha;
        let a1 = -2.0 * w0.cos();
        let a2 = 1.0 - alpha;

        self.b0 = b0 / a0;
        self.b1 = b1 / a0;
        self.b2 = b2 / a0;
        self.a1 = a1 / a0;
        self.a2 = a2 / a0;
    }

    fn process(&mut self, x: f32) -> f32 {
        let y = self.b0 * x + self.b1 * self.x1 + self.b2 * self.x2
            - self.a1 * self.y1
            - self.a2 * self.y2;
        self.x2 = self.x1;
        self.x1 = x;
        self.y2 = self.y1;
        self.y1 = y;
        y
    }
}

// Audio runs at sample rate, so it keeps its own smoothed pressure and resonator states.
struct EngineSoundCore {
    sample_rate: u32,
    model: AudioModelConfig,
    params: AudioParams,
    frame_count: u64,
    collector_pressure_smooth: f32,
    runner_pressure_smooth: f32,
    prev_runner_pressure_smooth: f32,
    intake_pressure_smooth: f32,
    flow_smooth: f32,
    pulse_env: f32,
    pulse_phase: f32,
    dc_state: f32,
    loudness_env_state: f32,
    loudness_gain_state: f32,
    res_1: Resonator,
    res_2: Resonator,
    res_3: Resonator,
}

impl EngineSoundCore {
    fn new(sample_rate: u32, model: AudioModelConfig) -> Self {
        let sr = sample_rate as f32;
        let initial_params = AudioParams::default();
        // Seed the filter bank from a nominal hot-exhaust temperature before live data arrives.
        let clamped_temp_k = initial_params
            .exhaust_temp_k
            .clamp(model.exhaust_temp_min_k, model.exhaust_temp_max_k);
        let sound_speed_mps = (GAMMA_AIR as f32 * R_AIR as f32 * clamped_temp_k).sqrt();
        let quarter_wave_hz = sound_speed_mps / (4.0 * model.exhaust_pipe_length_m.max(0.1));
        let res_1_freq = quarter_wave_hz * model.resonator_mode_1;
        let res_2_freq = quarter_wave_hz * model.resonator_mode_2;
        let res_3_freq = quarter_wave_hz * model.resonator_mode_3;
        let res_1_q = model.resonator_1_q;
        let res_2_q = model.resonator_2_q;
        let res_3_q = model.resonator_3_q;
        let res_freq_min = model.resonator_freq_min_hz;
        let res_freq_max_ratio = model.resonator_freq_max_nyquist_ratio;
        let res_q_min = model.resonator_q_min;
        Self {
            sample_rate,
            model,
            params: initial_params,
            frame_count: 0,
            collector_pressure_smooth: 0.0,
            runner_pressure_smooth: 0.0,
            prev_runner_pressure_smooth: 0.0,
            intake_pressure_smooth: 0.0,
            flow_smooth: 0.0,
            pulse_env: 0.0,
            pulse_phase: 0.0,
            dc_state: 0.0,
            loudness_env_state: 0.0,
            loudness_gain_state: 1.0,
            res_1: Resonator::new(
                sr,
                res_1_freq,
                res_1_q,
                res_freq_min,
                res_freq_max_ratio,
                res_q_min,
            ),
            res_2: Resonator::new(
                sr,
                res_2_freq,
                res_2_q,
                res_freq_min,
                res_freq_max_ratio,
                res_q_min,
            ),
            res_3: Resonator::new(
                sr,
                res_3_freq,
                res_3_q,
                res_freq_min,
                res_freq_max_ratio,
                res_q_min,
            ),
        }
    }

    fn set_params(&mut self, params: AudioParams) {
        self.params = params;
    }

    fn next_sample(&mut self) -> f32 {
        // Convert the slowly updated engine state into one deterministic exhaust-audio sample.
        let sr = self.sample_rate as f32;
        let firing_hz =
            (self.params.engine_speed_rpm.max(0.0) * FIXED_CYLINDER_COUNT as f32 / 120.0).max(0.0);
        let rpm_gate_span = (self.model.rpm_gate_full_rpm - self.model.rpm_gate_floor_rpm).max(1.0);
        let rpm_gate = ((self.params.engine_speed_rpm - self.model.rpm_gate_floor_rpm)
            / rpm_gate_span)
            .clamp(0.0, 1.0)
            .powf(self.model.rpm_gate_exponent.max(0.1));
        let exhaust_temp_k = self
            .params
            .exhaust_temp_k
            .clamp(self.model.exhaust_temp_min_k, self.model.exhaust_temp_max_k);
        let target_collector_norm = ((self.params.exhaust_pressure_kpa
            - self.model.ambient_pressure_kpa)
            / self.model.pressure_span_kpa)
            .clamp(0.0, 1.0)
            * rpm_gate;
        let target_runner_norm = ((self.params.exhaust_runner_pressure_kpa
            - self.model.ambient_pressure_kpa)
            / self.model.pressure_span_kpa)
            .clamp(0.0, 1.0)
            * rpm_gate;
        let target_intake_norm = ((self.model.ambient_pressure_kpa
            - self.params.intake_runner_pressure_kpa)
            / self.model.pressure_span_kpa)
            .clamp(0.0, 1.0)
            * rpm_gate;
        let target_flow_norm = (self.params.exhaust_runner_flow_gps.abs()
            / self.model.flow_span_gps.max(1.0e-3))
        .clamp(0.0, 1.0)
            * rpm_gate;
        self.collector_pressure_smooth += self.model.pressure_smoothing_alpha
            * (target_collector_norm - self.collector_pressure_smooth);
        self.runner_pressure_smooth += self.model.pressure_smoothing_alpha
            * (target_runner_norm - self.runner_pressure_smooth);
        self.intake_pressure_smooth += self.model.pressure_smoothing_alpha
            * (target_intake_norm - self.intake_pressure_smooth);
        self.flow_smooth +=
            self.model.pressure_smoothing_alpha * (target_flow_norm - self.flow_smooth);
        let dp_runner = self.runner_pressure_smooth - self.prev_runner_pressure_smooth;
        self.prev_runner_pressure_smooth = self.runner_pressure_smooth;

        if self
            .frame_count
            .is_multiple_of(self.model.resonator_retarget_interval.max(1) as u64)
        {
            // Retune the resonators lazily; exhaust temperature evolves far slower than audio rate.
            let sound_speed_mps = (GAMMA_AIR as f32 * R_AIR as f32 * exhaust_temp_k).sqrt();
            let quarter_wave_hz =
                sound_speed_mps / (4.0 * self.model.exhaust_pipe_length_m.max(0.1));
            self.res_1.set_band_pass(
                sr,
                quarter_wave_hz * self.model.resonator_mode_1,
                self.model.resonator_1_q,
                self.model.resonator_freq_min_hz,
                self.model.resonator_freq_max_nyquist_ratio,
                self.model.resonator_q_min,
            );
            self.res_2.set_band_pass(
                sr,
                quarter_wave_hz * self.model.resonator_mode_2,
                self.model.resonator_2_q,
                self.model.resonator_freq_min_hz,
                self.model.resonator_freq_max_nyquist_ratio,
                self.model.resonator_q_min,
            );
            self.res_3.set_band_pass(
                sr,
                quarter_wave_hz * self.model.resonator_mode_3,
                self.model.resonator_3_q,
                self.model.resonator_freq_min_hz,
                self.model.resonator_freq_max_nyquist_ratio,
                self.model.resonator_q_min,
            );
        }

        // Pressure rise excites the pulse envelope, while firing frequency drives the periodic train.
        self.pulse_env = (self.pulse_env * self.model.pulse_env_decay
            + (dp_runner.max(0.0) * self.model.pulse_env_dp_gain)
            + self.model.flow_pulse_gain * self.flow_smooth)
            .clamp(self.model.pulse_env_min, self.model.pulse_env_max);
        self.pulse_phase = (self.pulse_phase + firing_hz / sr).fract();
        let x = self.pulse_phase;
        // Deterministic pulse-train from a decaying exhaust-event shape.
        let pulse_train = ((-self.model.pulse_shape_decay_fast * x).exp()
            - (-self.model.pulse_shape_decay_slow * x).exp())
        .max(0.0);
        let pressure_drive = self.model.runner_pressure_mix * self.runner_pressure_smooth
            + self.model.collector_pressure_mix * self.collector_pressure_smooth
            + self.model.intake_pressure_mix * self.intake_pressure_smooth
            + self.flow_smooth;
        let pressure_pulse = (dp_runner * self.model.pressure_pulse_gain)
            .clamp(self.model.pressure_pulse_min, self.model.pressure_pulse_max);
        let exhaust_pulse = pulse_train
            * (self.model.exhaust_pulse_base + self.model.exhaust_pulse_gain * pressure_drive)
            * rpm_gate;
        let excitation = pressure_pulse
            + exhaust_pulse
            + self.model.pulse_sine_gain * self.pulse_env * rpm_gate * (TAU_F32 * x).sin();

        let resonated = self.model.resonator_mix_1 * self.res_1.process(excitation)
            + self.model.resonator_mix_2 * self.res_2.process(excitation)
            + self.model.resonator_mix_3 * self.res_3.process(excitation);
        let direct_pulse = self.model.direct_pulse_mix * excitation;
        let rumble = self.model.rumble_gain
            * self.pulse_env
            * (TAU_F32 * self.model.rumble_harmonic * x).sin();
        let mut raw = resonated + direct_pulse + rumble;

        // Remove slowly varying bias before the final soft limiter.
        self.dc_state =
            self.model.dc_filter_decay * self.dc_state + self.model.dc_filter_input_gain * raw;
        raw -= self.dc_state;

        let base_gain = (self.model.loudness_base
            + self.model.loudness_pressure_gain
                * (0.65 * self.runner_pressure_smooth + 0.35 * self.collector_pressure_smooth)
            + self.model.loudness_env_gain * self.pulse_env.min(1.0))
            * self.params.output_gain.max(self.model.output_gain_floor)
            * rpm_gate;
        let driven = raw * base_gain;
        let level = driven.abs();
        let env_alpha = if level > self.loudness_env_state {
            self.model.loudness_env_attack
        } else {
            self.model.loudness_env_release
        }
        .clamp(0.0, 1.0);
        self.loudness_env_state += env_alpha * (level - self.loudness_env_state);
        let desired_normalize_gain = (self.model.loudness_target_level
            / self
                .loudness_env_state
                .max(self.model.loudness_level_floor.max(1.0e-4)))
        .clamp(
            self.model.loudness_normalize_gain_min,
            self.model.loudness_normalize_gain_max,
        );
        self.loudness_gain_state += self.model.loudness_gain_smoothing.clamp(0.0, 1.0)
            * (desired_normalize_gain - self.loudness_gain_state);
        let normalization_gain = 1.0
            + self.model.loudness_normalize_mix.clamp(0.0, 1.0) * (self.loudness_gain_state - 1.0);

        self.frame_count = self.frame_count.wrapping_add(1);
        (driven * normalization_gain).tanh() * self.model.limiter_out_gain
    }
}

struct ExhaustSynthSource {
    shared: Arc<Mutex<AudioParams>>,
    core: EngineSoundCore,
    read_counter: u32,
}

impl ExhaustSynthSource {
    fn new(shared: Arc<Mutex<AudioParams>>, sample_rate: u32, model: AudioModelConfig) -> Self {
        Self {
            shared,
            core: EngineSoundCore::new(sample_rate, model),
            read_counter: 0,
        }
    }
}

impl Iterator for ExhaustSynthSource {
    type Item = f32;

    fn next(&mut self) -> Option<Self::Item> {
        // The UI updates shared params at frame rate; the audio thread samples them periodically.
        if self
            .read_counter
            .is_multiple_of(self.core.model.resonator_retarget_interval.max(1))
        {
            if let Ok(v) = self.shared.lock() {
                self.core.set_params(*v);
            }
        }
        self.read_counter = self.read_counter.wrapping_add(1);
        Some(self.core.next_sample())
    }
}

impl Source for ExhaustSynthSource {
    fn current_frame_len(&self) -> Option<usize> {
        None
    }

    fn channels(&self) -> u16 {
        1
    }

    fn sample_rate(&self) -> u32 {
        self.core.sample_rate
    }

    fn total_duration(&self) -> Option<Duration> {
        None
    }
}

pub(crate) struct AudioEngine {
    _stream: OutputStream,
    sink: Sink,
    shared: Arc<Mutex<AudioParams>>,
}

impl AudioEngine {
    pub(crate) fn new(config: &AudioConfig, initial_params: AudioParams) -> Option<Self> {
        let (stream, handle) = OutputStream::try_default().ok()?;
        let sample_rate = config.sample_rate_hz.max(8_000);
        let sink = Sink::try_new(&handle).ok()?;
        let shared = Arc::new(Mutex::new(AudioParams {
            output_gain: config.output_gain,
            ..initial_params
        }));
        sink.append(ExhaustSynthSource::new(
            shared.clone(),
            sample_rate,
            config.model.clone(),
        ));
        sink.play();
        Some(Self {
            _stream: stream,
            sink,
            shared,
        })
    }

    pub(crate) fn update(&self, params: AudioParams) {
        if let Ok(mut v) = self.shared.lock() {
            *v = params;
        }
    }
}

impl Drop for AudioEngine {
    fn drop(&mut self) {
        self.sink.stop();
    }
}

#[cfg(test)]
pub(crate) fn render_engine_audio(params: AudioParams, seconds: f32, sample_rate: u32) -> Vec<f32> {
    // Test helper that renders offline without touching the host audio device.
    let frames = (seconds * sample_rate as f32).round() as usize;
    let mut core = EngineSoundCore::new(sample_rate, AudioModelConfig::default());
    core.set_params(params);
    (0..frames).map(|_| core.next_sample()).collect()
}
