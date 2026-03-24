# ES User Manual

## 1. Purpose

`ES` is a CLI tool for sweeping steady operating points of the repository's inline-4 engine model.
Every operating point is solved by transiently integrating the same ODE system documented in `MODEL_REFERENCE.md`; the CLI does **not** switch to a separate map-only runtime, surrogate runtime, or MVEM runtime for the exported torque curve.

## 2. Recommended reference run

```bash
cargo run --release -- sweep \
  --config config/reference_na_i4.yaml \
  --output-dir output/reference_high_rev_na \
  --rpm-start 1000 \
  --rpm-end 8500 \
  --rpm-step 1000 \
  --settle-time 0.30 \
  --average-time 0.10 \
  --diagnostic-samples 180
```

This is the repository's recommended reference run when you want a dyno-style naturally aspirated 2.0 L brake torque curve while keeping the YAML-defined ignition timing, VVT, fuel enable, and spark enable inputs fixed and letting only the documented ODE system settle. In the generated `run_manifest.yaml` this appears as `input_mode: fixed_input`.

## 3. Why the CLI reports brake torque

For a speed-held dyno-style sweep, users normally expect **brake torque**, not residual acceleration torque.
The CLI therefore writes:

- `brake_torque_nm = net_torque_nm + load_torque_nm`
- `brake_power_kw = brake_torque_nm * omega`
- `net_torque_nm`: residual shaft acceleration torque after subtracting absorber load
- `load_torque_nm`: torque currently carried by the brake / absorber model

This makes the exported torque curve comparable to published dyno charts while still preserving the internal ODE bookkeeping.

## 4. Output files

### 4.1 `torque_curve.tsv`

Columns:

- `target_rpm`
- `mean_rpm`
- `brake_torque_nm`
- `brake_power_kw`
- `net_torque_nm`
- `load_torque_nm`
- `map_kpa`
- `air_flow_gps`
- `eta_indicated`
- `load_cmd`
- `ignition_timing_deg`
- `vvt_intake_deg`
- `vvt_exhaust_deg`
- `output_dir`

### 4.2 `point_XXXXrpm/summary.tsv`

Per-point summary columns:

- `rpm`
- `brake_torque_nm`
- `brake_power_kw`
- `net_torque_nm`
- `load_torque_nm`
- `brake_bmep_bar`
- `map_kpa`
- `air_flow_gps`
- `eta_indicated`

### 4.4 `torque_curve_metrics.tsv`

Scalar torque-shape metrics exported from the fixed-input sweep:
The same run also writes `torque_curve_assessment.md`, a Markdown audit note that explains why the fixed-input curve does or does not resemble a plausible naturally aspirated full-load shape.


- `peak_brake_torque_nm`
- `peak_torque_rpm`
- `peak_brake_power_kw`
- `peak_power_rpm`
- `low_to_peak_gain_nm`
- `peak_to_high_drop_nm`
- `high_rpm_retention_ratio` with `r_hi = tau(8000 rpm) / tau_max`
- `monotonic_before_peak_ratio`
- `monotonic_after_peak_ratio`

These metrics are designed for regression testing and for identifying whether a shape problem comes from low-speed build, mid-range peak placement, or excessive high-rpm falloff.

### 4.3 Diagnostic traces

Each `point_XXXXrpm/` directory also contains:

- `pv.tsv`: `volume_ratio`, `pressure_pa`
- `ptheta.tsv`: crank angle and four-cylinder pressure traces
- `ts.tsv`: crank angle, temperature, relative entropy, pressure, and volume ratio

## 5. High-rev naturally aspirated 2.0 L reference table

The current checked-in reference case is intentionally only an **approximate** match to a generic high-rev naturally aspirated torque trend.
Its goal is to reproduce the broad naturally aspirated high-rev trend using the documented reduced-order model and plausible constants.

Representative result from a short regression sweep (`rpm-step = 1000`, `settle-time = 0.08 s`, `average-time = 0.04 s`):

| Mean rpm | Brake torque [Nm] | Brake power [kW] | Net torque [Nm] |
|---:|---:|---:|---:|
| 978 | 44.5 | 4.4 | 25.1 |
| 1990 | 187.5 | 39.1 | 28.9 |
| 2996 | 187.4 | 58.8 | 19.3 |
| 4001 | 185.6 | 77.7 | 13.2 |
| 5001 | 134.4 | 70.4 | 6.4 |
| 6000 | 138.8 | 87.2 | 5.4 |
| 6999 | 108.0 | 79.2 | 3.5 |
| 8001 | 60.6 | 50.8 | -1.4 |
| 8505 | 39.6 | 35.3 | -3.1 |

Representative torque-shape metrics from the same fixed_input sweep:

| Metric | Value |
|---|---:|
| Peak brake torque rpm | 1990 rpm |
| Peak brake power rpm | 6000 rpm |
| Low-to-peak gain | 143.0 Nm |
| Peak-to-high drop | 147.9 Nm |
| High-rpm retention ratio `r_hi` | 0.323 |
| Pre-peak monotonicity | 1.000 |
| Post-peak monotonicity | 1.000 |

Interpretation:

- the updated closure set improves high-rpm retention compared with the previous fixed-input reference run because the cylinder boundary pressure follows runner pressure more strongly at high speed and the high-rpm side of `VE_base` decays more slowly
- the same run still peaks too early for a true high-rev NA 2.0 L reference, which indicates that further work is still needed in combustion phasing, trapped-residual modeling, and possibly fueling / lambda treatment rather than adding any hidden helper or torque remap
- the documentation keeps this mismatch explicit so further calibration can be traced back to the exact model terms involved

## 6. gnuplot example

```bash
gnuplot -e "cd 'output/reference_high_rev_na'" plot_torque_curve.gp
```

## 7. Model-to-code traceability

When adjusting the curve, start from these documented code paths:

- CLI sweep and brake-torque export: `src/cli.rs`
- ODE state integration and torque bookkeeping: `src/simulator.rs`
- parameter defaults and config schema: `src/config.rs`
- high-rev naturally aspirated 2.0 L reference case: `config/reference_na_i4.yaml`

## 8. Sources used for the reference case

The following sources are the explicit anchors for the documentation and parameter choices:

1. public catalog / brochure specs for high-rev naturally aspirated 2.0 L SI-engine power and torque envelopes.
2. Heywood, *Internal Combustion Engine Fundamentals* (2nd ed.), for filling-and-emptying, throttling, and indicated-work interpretation.
3. Stone, *Introduction to Internal Combustion Engines* (4th ed.), for naturally aspirated SI-engine trend ranges.
4. Woschni-style cylinder heat-transfer practice and standard single-zone heat-release modeling literature for the heat-loss / display-side pressure reconstruction assumptions.

Keep these references visible whenever you change the calibration or the documentation.


## 9. Model refinement workflow

When the computed torque curve drifts away from reality, the next step is not to bolt on an external correction layer, a surrogate correction, or an MVEM side path.
Instead, keep solving the documented ODE system, identify which physical effects are missing from the current closures, add those effects to the mathematical model, and solve the updated system again.

In practice this repository should repeat the same loop:

- solve the ODE model over the operating-point sweep
- inspect where the curve departs from expected behavior
- identify the missing or oversimplified physical phenomenon
- revise the reduced-order model and its closures
- rerun the ODE solve and inspect the new result

The priority is to improve the mathematical model itself, not to mask the error with a separate correction path.
