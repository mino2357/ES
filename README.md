# ES Simulator

`ES Simulator` is a GUI-centered inline-4 engine simulator with an EDM-style dashboard.
It focuses on transient reduced-order engine simulation, physically interpretable load response, and diagnostic plots.

This repository supports one product path only:

- desktop GUI simulation
- reduced-order transient engine model
- physically interpretable external-load modeling
- operator-side `Throttle cmd` and `Target RPM` inputs with displayed brake torque and power
- `p-V` and `p-theta` visualization

There is no separate offline high-precision solver and there is no audio synthesis path.

## Terms

- `ODE`: ordinary differential equation
- `VVT`: variable valve timing
- `lambda`: air-fuel equivalence ratio, where `lambda = 1.0` means stoichiometric fueling
- `internal EGR`: residual burned gas trapped by overlap, backflow, and incomplete scavenging
- `p-V`: cylinder pressure versus normalized cylinder volume
- `p-theta`: cylinder pressure versus crank angle over `0..720 degCA`
- `dyno`: a dynamometer or absorber load path; in this repository the active GUI path uses manual external-load control rather than an automated sweep
- `vehicle-equivalent load`: a road-load model based on vehicle mass, tire radius, gearing, rolling resistance, drag, and grade

## Documentation Map

Start here, then follow the document that matches your task:

- [docs/USER_MANUAL.md](docs/USER_MANUAL.md): build, run, dashboard operation, and configuration usage
- [docs/MODEL_REFERENCE.md](docs/MODEL_REFERENCE.md): complete ODE system summary, closures, implementation map, validation limits, and literature sources
- [docs/BENCH_CONSOLE_REFERENCE.md](docs/BENCH_CONSOLE_REFERENCE.md): source-backed bench-console feature map and reduced-order dyno-load rationale
- [README.ja.md](README.ja.md): Japanese overview
- [docs/USER_MANUAL.ja.md](docs/USER_MANUAL.ja.md): Japanese user manual
- [docs/MODEL_REFERENCE.ja.md](docs/MODEL_REFERENCE.ja.md): Japanese model reference
- [docs/BENCH_CONSOLE_REFERENCE.ja.md](docs/BENCH_CONSOLE_REFERENCE.ja.md): Japanese bench-console reference
- [ANDROID.md](ANDROID.md): Android packaging notes
- [ENGINE_MODEL_WORKLOG.md](ENGINE_MODEL_WORKLOG.md): chronological work log

Each linked document is written to be self-contained.
Complex model details are intentionally moved out of this top-level README.
The detailed documents also link back to this README and to each other.

## Direction

The current direction is accuracy-first GUI simulation:

- transient fidelity matters more than wall-clock synchronization
- the dashboard may render the latest completed state instead of forcing real-time lockstep
- the load path should remain physically interpretable
- complex model explanations should stay tied to explicit source references

## Build And Run

Desktop:

```bash
cargo run --release
```

Tests:

```bash
cargo test -- --nocapture
```

## Configuration Summary

Runtime configuration lives in [config/sim.yaml](config/sim.yaml).
It is parsed into `AppConfig` in `src/config.rs` and checked by the plausibility audit in `src/config/audit.rs`.

Important sections:

- `environment`: ambient boundary conditions and base timestep
- `engine`: geometry, inertia, and manifold volumes
- `cam`: valve-event geometry and VVT-sensitive inputs
- `control_defaults`: initial operator commands
- `model`: combustion, flow, heat-transfer, fuel, internal-EGR, and load closures
- `numerics`: timestep policy and accuracy settings
- `ui`: window, plot, and dashboard behavior
- `plot`: history sizes and plot sampling
The checked-in `sim.yaml` uses:

- `model.external_load.mode: vehicle_equivalent`
- `ui.sync_to_wall_clock: false`

That means the default path is an accuracy-first transient simulation with a vehicle-like external load model.

## Code Entry Points

Main implementation files:

- `src/simulator.rs`: reduced-order solver, display reconstruction, and load helpers
- `src/dashboard.rs`: EDM-style dashboard, operator controls, plots, and layout logic
- `src/config.rs`: configuration schema and defaults
- `src/config/audit.rs`: plausibility audit for physical, numerical, and UI parameters

For the detailed model equations and exact function-level mapping, use [docs/MODEL_REFERENCE.md](docs/MODEL_REFERENCE.md).
