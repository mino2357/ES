# ES User Manual

## 1. Purpose

`ES` is a CLI tool for sweeping steady operating points of the repository's inline-4 engine model.
Every operating point is solved by transiently integrating the same ODE system documented in `MODEL_REFERENCE.md`; the CLI does **not** switch to a separate map-only runtime for the exported torque curve.

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

This is the repository's recommended reference run when you want a dyno-style naturally aspirated 2.0 L brake torque curve with enough settling time to suppress controller transients.

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

### 4.3 Diagnostic traces

Each `point_XXXXrpm/` directory also contains:

- `pv.tsv`: `volume_ratio`, `pressure_pa`
- `ptheta.tsv`: crank angle and four-cylinder pressure traces
- `ts.tsv`: crank angle, temperature, relative entropy, pressure, and volume ratio

## 5. High-rev naturally aspirated 2.0 L reference table

The current checked-in reference case is intentionally only an **approximate** match to a generic high-rev naturally aspirated torque trend.
Its goal is to reproduce the broad naturally aspirated high-rev trend using the documented reduced-order model and plausible constants.

Representative result from the recommended command above:

| Mean rpm | Brake torque [Nm] | Brake power [kW] | Net torque [Nm] |
|---:|---:|---:|---:|
| 1005 | 157.6 | 16.6 | 16.3 |
| 1999 | 134.3 | 28.1 | 15.5 |
| 2999 | 173.0 | 54.3 | 8.9 |
| 4002 | 178.5 | 74.8 | 6.8 |
| 5000 | 149.4 | 78.2 | 4.4 |
| 6000 | 158.5 | 99.6 | 4.6 |
| 6999 | 141.6 | 103.8 | 3.4 |
| 7999 | 92.9 | 77.8 | 2.0 |
| 8499 | 68.4 | 60.9 | 1.1 |

Interpretation:

- the model currently captures a positive WOT brake curve and a power peak after the torque peak
- high-rpm torque retention remains the weakest part of the current reference sweep, so closing that gap requires revisiting the mathematical model and the missing high-speed gas-exchange physics
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

When the computed torque curve drifts away from reality, the next step is not to bolt on an external correction layer.
Instead, keep solving the documented ODE system, identify which physical effects are missing from the current closures, add those effects to the mathematical model, and solve the updated system again.

In practice this repository should repeat the same loop:

- solve the ODE model over the operating-point sweep
- inspect where the curve departs from expected behavior
- identify the missing or oversimplified physical phenomenon
- revise the reduced-order model and its closures
- rerun the ODE solve and inspect the new result

The priority is to improve the mathematical model itself, not to mask the error with a separate correction path.
