# ES Simulator User Manual

This document is self-contained.
It explains how to build, run, operate, and interpret the current GUI-centered simulator.

## Terms

- `GUI`: graphical user interface
- `dyno`: a dynamometer or absorber load path
- `lambda`: air-fuel equivalence ratio
- `VVT`: variable valve timing
- `FHD`: full high definition, typically `1920x1080`
- `WQHD`: wide quad high definition, typically `2560x1440`
- `vehicle-equivalent load`: a road-load model based on vehicle mass and driveline parameters

## Scope

`ES Simulator` is a reduced-order inline-4 engine simulator with a stylized test-cell dashboard.
It is intended for interactive transient studies, physically interpretable load-response studies, and visualization.

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
- `MOTOR`
- `FIRING`
- `LOAD CTRL`
- `PWR LIM`
- `ACCURACY`

`ACCURACY` means the simulator is not forcing wall-clock synchronization and is instead advancing a fixed amount of simulated time per rendered frame.

### Operator Rack

The left rack now shows the `Startup Numerical Fit` status together with the manual controls:

- on first fire, each throttle bin gets a local `MBT` spark search and the required brake torque for the requested RPM is solved numerically
- manual actuator edits stay locked until that startup fit converges
- once the fit reaches `READY`, its result is saved under `cache/startup_fit/` and reused on the next launch when the build identity and raw YAML config text still match
- once the fit is ready, throttle, spark, fuel, ignition, and VVT can be edited manually

### Operator Display

The top-center display shows:

- large digital readouts for speed, torque, power, trapped air, intake pressure, and indicated efficiency
- gauges for RPM, MAP, lambda, BMEP, exhaust temperature, combustion power, and internal EGR
- linear meters for throttle, `Target RPM`, ignition, and VVT

### Plots

The center and lower areas show:

- cylinder `p-V`
- cylinder `p-theta`
- indicated torque, net torque, net shaft power, and IMEP / indicated efficiency

These remain visible while the startup fit is running.

The central region is scrollable when the full layout does not fit vertically.

## Basic Operation

### Manual Transient Operation

1. Start the application.
2. Let the startup numerical fit run immediately after first fire.
3. Watch `p-V`, `p-theta`, and indicated torque while the fit converges.
4. Once the fit reaches `READY`, adjust `Throttle cmd`, `Ignition`, `VVT Intake`, and `VVT Exhaust` manually.
5. Toggle `Spark` and `Fuel` as needed.

### Operator Inputs And Outputs

The primary operator inputs are:

- `Throttle cmd`
- `Target RPM`

The dashboard-side speed-hold controller estimates current shaft torque, then adjusts the internal
`load_cmd` so the machine torque drives RPM error toward zero.
The primary outputs are:

- `Required brake torque`
- `Brake power`
- `RPM error`
- `Machine torque act / shaft est`

### Choosing A Load Model

In `Actuator Deck`, `Load model` can be:

- `Brake dyno`: a speed-dependent absorber torque surrogate with absorber power limit
- `Vehicle eq.`: a road-load model reflected to engine torque and engine-side inertia

`Vehicle eq.` is the default checked-in path because it gives a more interpretable transient response.

## Configuration

The runtime configuration file is [../config/sim.yaml](../config/sim.yaml).
It is parsed by `AppConfig` in `src/config.rs` and checked by the plausibility audit in `src/config/audit.rs`.

Startup-fit artifacts are saved under [../cache/startup_fit](../cache/startup_fit).
They are reused only when both the build identity and the raw YAML config text still match.

### Important Sections

- `environment`: ambient pressure, temperature, and base timestep
- `engine`: displacement geometry, inertia, default target RPM, manifold volumes, runner dimensions
- `cam`: valve event locations and durations
- `control_defaults`: initial throttle, spark, fuel, load, and calibration commands
- `model`: combustion, flow, friction, internal-EGR, and load closures
- `numerics`: timestep ceilings, floors, and accuracy-target settings
- `ui`: window size, scroll behavior, and wall-clock versus accuracy-first stepping
- `plot`: plot histories and p-V sampling

### Accuracy-First Mode

`ui.sync_to_wall_clock` selects the stepping policy:

- `true`: try to follow wall clock
- `false`: integrate a fixed amount of physical time per rendered frame

The checked-in configuration uses `false`.

`ui.simulated_time_per_frame_s` sets the target amount of physical time to advance per GUI frame.
The solver then chooses a numerically smaller step internally when engine speed rises.

## Reading The Plots Correctly

### `p-V`

The displayed `p-V` loop uses a display-oriented single-zone cylinder-pressure solve.
Over the closed cycle it integrates `dp/dtheta` from the Wiebe burn fraction and the cycle heat-loss estimate,
then blends the gas-exchange strokes back to the intake and exhaust boundary pressures.
It is useful for relative trends such as:

- indicated work changes
- combustion phasing changes
- pumping-loss changes

When combustion is active, the plot also overlays ignition, `SOC/EOC`, and `CA10/50/90` markers.

It is not the main engine-state ODE itself, and it is not a direct measured cylinder-pressure trace.

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
- studying load transients with `Brake dyno` and `Vehicle eq.`

Poor uses:

- final calibration numbers for a production ECU
- exact emissions prediction
- exact in-cylinder pressure reconstruction without experimental calibration

## Related Documents

- [../README.md](../README.md): repository overview
- [MODEL_REFERENCE.md](MODEL_REFERENCE.md): equations, closures, implementation map, and sources
- [USER_MANUAL.ja.md](USER_MANUAL.ja.md): Japanese version of this manual
