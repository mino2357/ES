# ES Simulator User Manual

This document is self-contained.
It explains how to build, run, operate, and interpret the current GUI-centered simulator.

## Terms

- `GUI`: graphical user interface
- `bench`: automated speed-hold sweep that imitates a dynamometer workflow
- `dyno`: absorber and controller used by the bench
- `lambda`: air-fuel equivalence ratio
- `VVT`: variable valve timing
- `FHD`: full high definition, typically `1920x1080`
- `WQHD`: wide quad high definition, typically `2560x1440`
- `vehicle-equivalent load`: a road-load model based on vehicle mass and driveline parameters

## Scope

`ES Simulator` is a reduced-order inline-4 engine simulator with a stylized test-cell dashboard.
It is intended for interactive transient studies, bench-style sweeps, and physically interpretable visualization.

It is not:

- CFD
- a full 1D gas-dynamics solver
- a certified emissions model
- a substitute for production ECU calibration data

## Build And Run

Start the desktop application:

```bash
cargo run --release
```

Run the automated tests:

```bash
cargo test -- --nocapture
```

## Main Screen Layout

The GUI is organized like a compact engine test cell.

### Header

The header shows machine-state annunciators such as:

- `RUN`
- `FUEL`
- `SPARK`
- `STARTER`
- `AUTO IDLE`
- `WOT SEARCH`
- `BENCH`
- `ACCURACY`

`ACCURACY` means the simulator is not forcing wall-clock synchronization and is instead advancing a fixed amount of simulated time per rendered frame.

### Operator Rack

The left rack contains collapsible modules:

- `Automation`: idle automation and WOT efficiency search
- `Bench Sequencer`: automated dyno sweep controls
- `Actuator Deck`: manual throttle, load, starter, spark, fuel, ignition, and VVT commands
- `Status Bus`: runtime status, load-model mode, and solver mode
- `Sensor / State Bus`: lower-level reduced-order states and closure outputs

The left rack is scrollable.
Lower-priority modules are collapsible so the dashboard remains usable on FHD-class displays.

### Operator Display

The top-center display shows:

- large digital readouts for speed, torque, power, trapped air, intake pressure, and indicated efficiency
- gauges for RPM, MAP, lambda, BMEP, exhaust temperature, and combustion power
- linear meters for throttle, load command, ignition, and VVT

### Plots

The center and lower areas show:

- bench torque and power plots
- cylinder `p-V`
- cylinder `p-theta`
- cycle-history plots for RPM, torque, and trapped air

The central region is scrollable when the full layout does not fit vertically.

## Basic Operation

### Manual Transient Operation

1. Start the application.
2. Use `Throttle cmd` to request airflow.
3. Use `Load cmd` to apply absorber or vehicle-equivalent load.
4. Toggle `Starter`, `Spark`, and `Fuel` as needed.
5. Adjust `Ignition`, `VVT Intake`, and `VVT Exhaust` to inspect transient response.

### Choosing A Load Model

In `Actuator Deck`, `Load model` can be:

- `Brake map`: a speed-dependent absorber torque surrogate
- `Vehicle eq.`: a road-load model reflected to engine torque and engine-side inertia

`Vehicle eq.` is the default checked-in path because it gives a more interpretable transient response.

### Bench Sweep

`Bench Sequencer` performs an automated speed-hold sweep.

1. Choose a bench mixture mode.
2. Press `Run Bench`.
3. The dyno controller adjusts load automatically to hold each target RPM.
4. Brake torque and brake power are accumulated and exported.

The latest CSV is written to:

- `dist/bench/bench-rich_charge_cooling-latest.csv`
- `dist/bench/bench-lambda_one-latest.csv`

depending on the selected mixture mode.

## Configuration

The runtime configuration file is [../config/sim.yaml](../config/sim.yaml).
It is parsed by `AppConfig` in `src/config.rs` and checked by the plausibility audit in `src/config/audit.rs`.

### Important Sections

- `environment`: ambient pressure, temperature, and base timestep
- `engine`: displacement geometry, inertia, idle target, manifold volumes, runner dimensions
- `cam`: valve event locations and durations
- `control_defaults`: initial throttle, spark, fuel, load, and calibration commands
- `auto_control`: idle and WOT search behavior
- `model`: combustion, flow, friction, internal-EGR, and load closures
- `numerics`: timestep ceilings, floors, and accuracy-target settings
- `ui`: window size, scroll behavior, and wall-clock versus accuracy-first stepping
- `plot`: plot histories and p-V sampling
- `bench`: dyno sweep setup and absorber model

### Accuracy-First Mode

`ui.sync_to_wall_clock` selects the stepping policy:

- `true`: try to follow wall clock
- `false`: integrate a fixed amount of physical time per rendered frame

The checked-in configuration uses `false`.

`ui.simulated_time_per_frame_s` sets the target amount of physical time to advance per GUI frame.
The solver then chooses a numerically smaller step internally when engine speed rises.

## Reading The Plots Correctly

### `p-V`

The displayed `p-V` loop is a reconstructed diagnostic view.
It is based on the reduced-order state and cycle closures.
It is useful for relative trends such as:

- indicated work changes
- combustion phasing changes
- pumping-loss changes

It is not a direct measured cylinder pressure trace.

### `p-theta`

The `p-theta` view overlays all four cylinders over `0..720 degCA`.
It is intended to show:

- phase spacing between cylinders
- pressure-peak movement with ignition or VVT changes
- cycle-shape differences under changing load and charge conditions

## What To Use This Simulator For

Good uses:

- studying transient trends with physically interpretable controls
- comparing load-model behavior
- visualizing the qualitative effect of ignition and VVT changes
- producing bench-style torque curves

Poor uses:

- final calibration numbers for a production ECU
- exact emissions prediction
- exact in-cylinder pressure reconstruction without experimental calibration

## Related Documents

- [../README.md](../README.md): repository overview
- [MODEL_REFERENCE.md](MODEL_REFERENCE.md): equations, closures, implementation map, and sources
- [USER_MANUAL.ja.md](USER_MANUAL.ja.md): Japanese version of this manual
