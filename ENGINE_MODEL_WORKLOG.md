# Engine Model Worklog

## Related Documents

- [README.md](README.md): repository overview and documentation map
- [README.ja.md](README.ja.md): Japanese overview
- [docs/USER_MANUAL.md](docs/USER_MANUAL.md): GUI operation manual
- [docs/MODEL_REFERENCE.md](docs/MODEL_REFERENCE.md): mathematical model and source map

## 2026-03-10

### Product Direction Reset

- Removed the separate high-precision offline branch.
- Removed all audio synthesis code and configuration.
- Kept one supported product path: the realtime GUI simulator with plots and physically interpretable external-load operation.

### Retained Technical Direction

- Improve the reduced-order intake and exhaust model instead of maintaining a second solver.
- Keep parameter ranges physically plausible through config auditing.
- Improve external-load behavior so transient response stays physically interpretable.
- Keep the dashboard readable on real displays by tightening layout density, enabling scrolling, and collapsing lower-priority modules.

### Recent Solver Features Still In Scope

- `p-theta` overlay for all 4 cylinders over `0..720 degCA`
- external load path with manual operator control and vehicle-equivalent mode
- internal-EGR and charge-property effects in the realtime path
- vehicle-equivalent road load with reflected inertia
- accuracy-first GUI stepping that is not forced to follow wall clock
- Removed the automated WOT efficiency search path so the supported control surface stays manual and explicit.

### Documentation Restructure

- Split the top-level README into overview-only content plus dedicated manuals in `docs/`.
- Added a self-contained user manual in English and Japanese.
- Added a self-contained model reference in English and Japanese.
- Moved equations, implementation mapping, and literature sources out of the top-level README so complex material can evolve without turning README into a monolith.

### Equation Expansion

- Expanded `docs/MODEL_REFERENCE.md` and `docs/MODEL_REFERENCE.ja.md` so the differential equations, algebraic closures, `p-V` reconstruction, and RK3 update rule are written explicitly.
- Added a source map so each major equation family can be traced to the literature base or identified as an implementation-specific surrogate.

### VVT Modeling

- Replaced the purely linear `VVT` term inside `volumetric_efficiency()` with a speed-dependent optimum-phasing surrogate.
- Tuned the default `VVT` envelope so the model shows the expected qualitative direction: intake advance helping low-speed torque, and more retarded intake / added overlap becoming more favorable near rated speed.
- Added regression tests for those qualitative VVT trends.
- Linked that surrogate explicitly to SAE references in `docs/MODEL_REFERENCE.md` and `docs/MODEL_REFERENCE.ja.md` so the implementation-specific part is still source-anchored.

### Immediate Next Steps

- tighten runner and manifold closures using literature-backed reduced-order models
- calibrate the realtime path against well-documented production engines
- improve per-cylinder fueling controls in the GUI without bloating the config surface

## 2026-03-11

### Bench Console Pass

- Added a source-backed `Bench Console` module to the left rack instead of trying to reproduce any one vendor layout.
- Kept the existing operator display, plots, and motion schematic, then added a reduced subset of engine-dyno switches and annunciators:
  `E-STOP`, `INTERLOCK`, `DYNO EN`, `VENT`, `COOLING`, `SPD CTRL`, and `PWR LIM`.
- Added absorber-side instrumentation to the live display path so operator-visible bench quantities now include absorber torque, absorber power, available torque, overspeed threshold, and power-limit torque.

### External-Load Model Revision

- Promoted the old `Brake map` path into `Brake dyno` terminology for the GUI and manual.
- Added absorber rotor inertia to the reduced-order load model so a coupled dynamometer contributes inertia even when road-load reflection is not active.
- Added an absorber power limit to the available-torque envelope so high-speed load capability falls with `P = \tau \omega`.
- Kept the `Vehicle equivalent` mode and layered the same absorber power limit on top of the road-load reference.

### Documentation

- Added `docs/BENCH_CONSOLE_REFERENCE.md` and `docs/BENCH_CONSOLE_REFERENCE.ja.md` as self-contained references for the bench-side UI rationale and load-model simplifications.
- Updated the user manual and model reference so the new bench-side switches, annunciators, and reduced-order absorber model are tied back to the implementation and to public sources.

## 2026-03-13

### Development Mode Shift

- Up to this point, much of the code and experimentation had been aimed at exploring what AI-assisted development could do in this repository.
- From here on, development is no longer framed as open-ended exploration of AI capability.
- Added `docs/IMPLEMENTATION_DIRECTION.ja.md` as the default implementation charter for architecture, calibration workflow, educational goals, numerical credibility, legal/compliance handling, and documentation discipline.
- Future work is expected to follow that document unless the user explicitly overrides it.

### Direction-Aligned Slimming

- Removed the Android packaging path and Android-specific launch/config code so the supported runtime stays focused on the desktop GUI path.
- Removed the bench-console UI layer and its dedicated reference documents, keeping the external-load model while reducing nonessential dashboard state and documentation surface.
- Updated the manuals and model-reference links so the remaining document set matches the slimmer runtime path.
