# High Precision YAML Parameter Audit

## Scope

This audit applies to the std-only offline solver driven by `config/high_precision.yaml`.
It does not try to collapse the legacy realtime GUI config, which still contains UI, audio, and calibration-oriented knobs.

The goal of this pass was:

1. keep only parameters that correspond to geometry, boundary conditions, operating conditions, or numerical resolution,
2. remove free knobs that only shape behavior without a direct physical counterpart,
3. enforce a strict schema so obsolete or ad hoc keys do not silently accumulate,
4. validate every retained YAML field against a physically plausible sanity envelope.

## Terms Used In This Audit

This audit note is intended to be self-contained.

- `strict schema`: unknown keys are rejected instead of being ignored.
- `sanity envelope`: a deliberately broad allowed range that screens out obviously non-physical or numerically reckless inputs without claiming that every value inside the band is a good calibration.
- `physical parameter`: a quantity that maps directly to geometry, boundary conditions, or hardware.
- `semi-empirical closure`: a retained coefficient set that is not first-principles geometry, but is still needed to model effects such as friction or wall heat transfer.
- `numerical setting`: a discretization or solver-resolution choice such as timestep or runner cell count, not a hardware property.
- `SI`: spark ignition.

## Review Loop 1: Classification

The YAML is now split into three categories:

- `engine`: physical geometry, gas-path hardware, friction closure, and valve-event timing
- `combustion`: fuel and burn phasing inputs, plus the retained wall-heat-transfer closure
- `sweep` and `numerics`: operating sweep, output selection, and discretization

## Review Loop 2: Removed Keys

Removed from YAML:

- `engine.cylinders`
  Rationale: this headless solver is currently a fixed inline-4 path. Leaving cylinder count configurable only created schema width without supporting a distinct model family.
- `combustion.wiebe_a`
- `combustion.wiebe_m`
  Rationale: these are burn-shape tuning knobs. They were useful for reproducing a target trace shape but are not direct hardware inputs. The solver now fixes them internally at `a = 5.0`, `m = 2.0`.
- `engine.intake_runner_cells`
- `engine.exhaust_runner_cells`
  Rationale: runner cell count is not hardware. It is a discretization setting and therefore lives under `numerics`.

The parser now rejects unknown keys, so removed items cannot silently remain in the file.

## Review Loop 3: Retained Fields and Sanity Envelopes

These envelopes are intentionally broad. They are not a calibration target. They are there to prevent obviously non-physical YAML from entering the solver.

### Output

- `output_dir`: non-empty string

### Engine Geometry and Boundary Conditions

- `engine.bore_m`: `0.055 .. 0.110`
- `engine.stroke_m`: `0.055 .. 0.120`
- `engine.conrod_m`: `0.090 .. 0.230`
- `engine.conrod_m / engine.stroke_m`: `1.35 .. 2.20`
- `engine.compression_ratio`: `7.0 .. 16.5`
- `engine.ambient_pressure_pa`: `80000 .. 110000`
- `engine.intake_temp_k`: `250 .. 370`
- `engine.wall_temp_k`: `320 .. 650`
- `engine.exhaust_temp_k`: `500 .. 1300`
- ordering constraint: `intake_temp_k < wall_temp_k < exhaust_temp_k`

### Plenum, Tailpipe, and Runner Hardware

- `engine.plenum_volume_m3`: `0.0005 .. 0.020`
- `engine.collector_volume_m3`: `0.0004 .. 0.020`
- `engine.plenum_volume_m3 / total_displacement`: `0.25 .. 6.0`
- `engine.collector_volume_m3 / total_displacement`: `0.20 .. 6.0`
- `engine.throttle_area_max_m2`: equivalent diameter `0.025 .. 0.090 m`
- `engine.throttle_discharge_coeff`: `0.55 .. 1.00`
- `engine.tailpipe_area_m2`: equivalent diameter `0.030 .. 0.090 m`
- `engine.tailpipe_discharge_coeff`: `0.55 .. 1.00`
- `engine.intake_runner_length_m`: `0.15 .. 0.80`
- `engine.intake_runner_diameter_m`: `0.020 .. 0.060`
- `engine.intake_runner_loss_coeff`: `0.10 .. 10.0`
- `engine.exhaust_runner_length_m`: `0.30 .. 1.20`
- `engine.exhaust_runner_diameter_m`: `0.020 .. 0.070`
- `engine.exhaust_runner_loss_coeff`: `0.10 .. 12.0`

### Friction Closure

These three coefficients remain semi-empirical. They are retained because the current solver still needs a brake-torque closure.

- `engine.friction_c0_nm`: `0.0 .. 25.0`
- `engine.friction_c1_nms`: `0.0 .. 0.05`
- `engine.friction_c2_nms2`: `0.0 .. 0.001`
- derived friction torque must stay within `2 .. 80 Nm` at `1000`, `3000`, and `7000 rpm`

### Valve Timing and Valve Hardware

- `engine.intake_open_deg`: `0 .. 720`
- `engine.intake_close_deg`: `0 .. 720`
- `engine.exhaust_open_deg`: `0 .. 720`
- `engine.exhaust_close_deg`: `0 .. 720`
- wrapped valve duration for intake and exhaust: `180 .. 320 degCA`
- `engine.intake_valve_diameter_m`: `0.020 .. 0.050`
- `engine.exhaust_valve_diameter_m`: `0.018 .. 0.045`
- `engine.intake_valve_cd`: `0.55 .. 0.95`
- `engine.exhaust_valve_cd`: `0.55 .. 0.95`
- `engine.intake_max_lift_m`: `0.004 .. 0.018`
- `engine.exhaust_max_lift_m`: `0.004 .. 0.018`
- lift-to-diameter ratio for both valves: `0.10 .. 0.45`

### Combustion and Heat Transfer

These are retained even though they are not all first-principles state variables, because they still map to real fuel chemistry, ignition scheduling, and wall heat loss.

- `combustion.stoich_afr`: `13.5 .. 15.5`
- `combustion.lambda_target`: `0.75 .. 1.60`
- `combustion.fuel_lhv_j_per_kg`: `40e6 .. 45e6`
- `combustion.ignition_advance_deg`: `0 .. 45`
- `combustion.burn_duration_deg`: `20 .. 90`
- `combustion.wall_htc_w_m2k`: `50 .. 1000`

### Sweep and Numerical Controls

- `sweep.throttle`: `0 .. 1`
- `sweep.rpm_points[*]`: strictly increasing, each `600 .. 10000`
- `sweep.pv_rpm_points[*]`: optional list; if non-empty, strictly increasing, each `600 .. 10000`, and each must also appear in `rpm_points`
- `numerics.dt_deg`: `0.05 .. 1.00`
- `numerics.intake_runner_cells`: `2 .. 32`
- `numerics.exhaust_runner_cells`: `2 .. 32`
- `numerics.warmup_cycles`: `1 .. 20`
- `numerics.sample_cycles`: `1 .. 20`

## Retained Semi-Empirical Closures

The following items are still closures rather than strict first-principles geometry:

- throttle discharge coefficient
- tailpipe discharge coefficient
- valve discharge coefficients
- friction polynomial coefficients
- wall heat-transfer coefficient

They were kept because removing them now would make the solver less realistic, not more realistic. What changed in this pass is that they are:

- bounded,
- documented as closures,
- prevented from proliferating into extra free shape parameters.

## Literature Basis

The model structure follows spark-ignition (SI) mean-value and filling-dynamics literature:

- E. Hendricks, S. C. Sorenson, "Mean Value Modelling of Spark Ignition Engines," SAE 900616
- E. Hendricks, S. C. Sorenson, "SI Engine Controls and Mean Value Engine Modelling," SAE 910258
- E. Hendricks, T. Vesterholm, "The Analysis of Mean Value SI Engine Models," SAE 920682
- E. Hendricks, A. Chevalier, M. Jensen, S. C. Sorenson, D. Trumpy, J. Asik, "Modelling of the Intake Manifold Filling Dynamics," SAE 960037

The headless cylinder path also uses the standard single-zone Wiebe-plus-wall-loss pattern common in SI cycle simulation. In this repo, the Wiebe shape pair is intentionally frozen to stop it from becoming a free trace-fitting knob in YAML.
