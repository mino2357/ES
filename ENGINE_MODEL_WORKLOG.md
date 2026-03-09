# Engine Model Worklog

## 2026-03-08

### Intake/Exhaust Manifold Upgrade Kickoff

- Audited the current gas-path implementation in `src/simulator.rs`.
- Confirmed the baseline is a 0D mean-value engine with:
  - intake/exhaust plenum pressures,
  - intake/exhaust runner-side pressures,
  - runner mass-flow states,
  - empirical grouped wave-action closure layered on top.
- Chose the next realism step to be geometry-driven rather than adding a full 1D PDE solver in one jump.

### Planned Upgrade Path

1. Introduce geometry-derived runner inertance instead of relying only on tuned inertance constants.
2. Add nonlinear runner/primary pressure losses from duct friction and local-loss coefficients.
3. Scale the existing wave-action closure with geometry-derived resonance metrics so intake/exhaust tuning shifts come from runner dimensions more directly.
4. Keep the model numerically stable and retain the existing test envelope.

### Notes

- This remains a reduced-order engine model, not a full GT-Power / Ricardo WAVE style 1D solver.
- The immediate target is a more defensible manifold / runner submodel that still runs interactively.

### Implemented In This Pass

- Added geometry parameters to `model.gas_path`:
  - runner/primary effective lengths,
  - hydraulic diameters,
  - Darcy friction factors,
  - lumped local-loss coefficients.
- Added geometry-scaled runner inertance in the ODE path:
  - the existing calibrated inertance coefficients are now scaled by relative duct length and area,
  - this keeps default behavior anchored while making geometry changes move the dynamics in a physically meaningful direction.
- Added nonlinear pressure loss terms that oppose intake/exhaust runner flow:
  - loss magnitude follows a duct-loss form based on `f L / D + K`,
  - loss rises with the square of mass flow,
  - the sign flips correctly when the runner reverses flow.
- Added geometry-aware resonance weighting on top of the existing wave-action closure:
  - approximate Helmholtz frequencies are estimated from runner neck area, effective length, and plenum/collector volume,
  - the existing wave-pressure gains are scaled by a damped frequency-response factor instead of being purely constant with RPM.
- Corrected a calibration issue found during validation:
  - runner pressure loss was initially overestimated because the flow state is aggregate across all cylinders,
  - the loss model now uses the effective total area of the parallel runners rather than a single-runner area.

### Validation

- Added helper tests for:
  - geometry-scaled inertance trend,
  - nonlinear runner loss sign and growth,
  - Helmholtz frequency shift with runner length.
- Re-ran the full project test suite after calibration repair.
- Current status: `cargo test` passed (`73 passed`), and `cargo build --release` passed.

### Still To Do

- Replace or augment the current event-memory wave closure with a more explicit resonance state or transmission-line surrogate.
- Consider adding plenum / collector acoustic compliance states so Helmholtz behavior is represented directly in the ODE instead of only through a closure.
- Recalibrate runner geometry defaults against target torque / VE curves after the new loss model is in place.

## 2026-03-09

### Offline Headless High-Precision Path

- Added a separate `std`-only binary `es_hp` behind `cargo run --no-default-features --bin es_hp`.
- Moved GUI dependencies behind Cargo feature `gui` so the offline solver can be compiled without `eframe`, `rodio`, `serde`, or `serde_yaml`.
- Added a dependency-free YAML subset parser dedicated to the headless path.
- Added a first offline crank-angle solver with:
  - 4 phased cylinders,
  - single-zone cylinder thermodynamics,
  - slider-crank geometry,
  - Wiebe heat release,
  - wall heat transfer,
  - common plenum / collector,
  - segmented intake and exhaust runners with pressure cells and face mass-flow states.
- Added CSV outputs:
  - `torque_curve.csv`
  - `pv_<rpm>rpm.csv`
- Added documentation in `HIGH_PRECISION_HEADLESS_MODEL.md`.
- Added exact / structural tests for:
  - RK4 against exponential decay,
  - YAML parsing,
  - bidirectional orifice sign behavior,
  - finite equilibrium-like derivatives,
  - positive torque over a short reference sweep.

### Known Gaps

- The offline headless solver is not yet calibrated to production torque levels; current sample output is intentionally treated as a numerical / architectural baseline.
- Runner temperature is still isothermal and the cylinder remains single-zone.
- No automated comparison pipeline against measured engine data has been added yet.

### YAML Parameter Audit

- Audited `config/high_precision.yaml` with the explicit goal of reducing non-physical free knobs.
- Removed `engine.cylinders` from YAML and fixed the headless path to the currently implemented inline-4 architecture.
- Removed `combustion.wiebe_a` and `combustion.wiebe_m` from YAML and fixed them internally so burn-trace shape tuning cannot sprawl in the config.
- Moved `intake_runner_cells` and `exhaust_runner_cells` from `engine` to `numerics` because they are discretization controls, not hardware.
- Added strict schema validation so unknown keys now fail fast instead of silently accumulating.
- Added range checks for every remaining YAML field plus a few derived checks:
  - rod-to-stroke ratio,
  - plenum / collector volume relative to displacement,
  - equivalent diameters from area-based entries,
  - friction torque sanity at representative RPMs,
  - valve-duration and lift-to-diameter sanity.
- Wrote the retained envelopes and rationale to `HIGH_PRECISION_PARAMETER_AUDIT.md`.

## 2026-03-10

### Android GUI Bring-Up

- Split the desktop GUI entry path into a shared library plus thin desktop binary.
- Added `src/lib.rs` with:
  - `launch_desktop()` for the existing Windows/Linux/macOS flow,
  - `android_main()` for Android NativeActivity launch.
- Updated `Cargo.toml` to:
  - emit a `cdylib` for Android packaging,
  - enable `eframe`'s `android-native-activity` support,
  - add Android package metadata for `cargo apk`,
  - add a direct Android-only `winit` dependency for the `AndroidApp` entry type.
- Added `dashboard::run_android_app(...)` so the same GUI code path can be started with an `AndroidApp`.
- Removed the desktop-only config-file assumption on Android:
  - `src/config.rs` now embeds the checked-in `config/sim.yaml` into the APK via `include_str!`,
  - Android boot therefore uses the same baseline calibration without relying on relative paths or executable-adjacent files.
- Added `ANDROID.md` with host setup, build commands, install commands, and current limitations.

### Notes

- This is a pragmatic first Android path, not a touch-optimized mobile UI pass.
- Runtime loading of user-provided YAML from Android storage is still intentionally deferred.
- Verification in this pass is limited to desktop regression checks because the local machine does not currently have the Android Rust target or SDK/NDK configured.

### Exhaust Audio Pass

- Reworked `src/audio.rs` so the synth is less like a free-running tone generator and more like a crank-synchronous exhaust model.
- Added phase locking from the simulated `cycle_deg` observation into the firing-phase oscillator.
- Added a short round-trip tailpipe reflection surrogate whose delay is tied to `2 L_{pipe} / c_{exh}`.
- Added a deterministic high-passed broadband component whose level scales with exhaust runner flow and pressure, so higher-load points pick up some jet-like edge without using sampled audio.
- Reduced explicitly synthetic support from:
  - `audio.model.pulse_sine_gain`: `0.04 -> 0.018`
  - `audio.model.loudness_normalize_mix`: `1.00 -> 0.80`
- Added an audio regression test confirming that higher exhaust flow increases broadband content while preserving the existing checks for:
  - zero-RPM silence,
  - pressure/load loudness increase,
  - bounded normalization,
  - pitch tracking with firing frequency.

### Reference Engine Calibration Pass

- Added primary-source calibration presets under `config/presets/` for:
  - `honda_s2000_f20c.yaml`
  - `nissan_tiida_hr16de.yaml`
- Added `ENGINE_REFERENCE_TARGETS.md` to track:
  - source URLs,
  - official bore / stroke / compression / rated power / rated torque,
  - current headless solver outputs,
  - an explicit note on whether the preset is already a tight fit or only an approximation band.
- Calibration loop outcome in this pass:
  - `Honda F20C`: high-rpm power point is close enough to use as a strong NA-4 reference anchor, but torque around `7500 rpm` is still high.
  - `Nissan HR16DE`: rated power is close, but the midrange torque point remains high for an economy-oriented 1.6 L engine.
  - `Ferrari 458 Italia`: retained as a future benchmark only, because the current solver is fixed to inline-4.

### Bench Dyno Speed-Hold Pass

- Replaced the old bench speed clamp with a bench-only dyno controller that adjusts absorber load automatically to hold target RPM.
- Split the bench load path from the generic `model.external_load` surrogate:
  - `bench.dyno.absorber_model` now defines the automatic bench absorber capacity,
  - `model.external_load` remains the manual / generic runtime load surrogate.
- Changed bench reporting semantics so the GUI bench curve shows `brake torque` measured by the dyno path, not the residual shaft `net torque`.
- Added live bench telemetry for:
  - measured RPM vs target RPM,
  - dyno load command,
  - applied absorber torque.
- Added automatic GUI bench CSV export to:
  - `dist/bench/bench-rich_charge_cooling-latest.csv`
  - `dist/bench/bench-lambda_one-latest.csv`
- Added regression coverage for:
  - dyno speed-hold seeding,
  - positive absorber torque during a held bench point,
  - dashboard visibility of the new bench curve data.

### Burned-Gas cp and Internal-EGR Pass

- Added two new realtime-model configuration groups under `model`:
  - `gas_thermo` for fresh-charge and burned-gas `c_p` bounds / temperature sensitivity,
  - `internal_egr` for overlap-backflow-driven residual-gas trapping.
- Extended the reduced-order combustion path so the solver now:
  - estimates a pulsation / backflow based internal-EGR fraction from overlap lift, runner pressure head, wave scavenging head, and reverse exhaust-runner flow,
  - mixes a cooled fresh charge with hot residual gas before evaluating the trapped-charge temperature,
  - derives mixture `\gamma` from the mixed `c_p`,
  - uses burned-gas `c_p` in the single-zone heat-loss temperature-rise estimate.
- Applied the internal-EGR state to combustion behavior:
  - higher trapped residual raises charge temperature,
  - burn duration stretches with dilution,
  - ignition-phase effectiveness is reduced with a lower clamp.
- Added regression coverage for:
  - burned-gas `c_p` increasing with temperature / dilution while `\gamma` falls,
  - overlap backflow increasing the modeled internal-EGR fraction,
  - internal EGR heating the charge and stretching the burn.

### Realtime Config Audit and Zero-Dependency CLI Pass

- Added a dedicated realtime-config plausibility audit in `src/config/audit.rs`.
- Classified YAML fields by role:
  - physical / semi-physical quantities must stay inside engine-plausible bands,
  - numerical / UI / audio shaping terms must stay inside runtime-practical bands.
- Wired the GUI loader so malformed-but-parsable `sim.yaml` now fails the audit and falls back to defaults instead of silently running with absurd values.
- Added a regression test that the checked-in `config/sim.yaml` passes the new audit.
- Expanded the std-only headless entry path into a fuller CLI:
  - `es_cli validate`
  - `es_cli sweep`
  - `es_cli report`
  - `es_cli point --rpm ...`
  - `es_cli list-presets`
- Kept `es_hp` as a compatibility alias calling the same CLI implementation.
- Added extra headless outputs:
  - `torque_summary.csv`
  - `sweep_report.txt`
  - `point_<rpm>rpm_summary.txt`
