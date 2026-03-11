# Engine Model Worklog

## 2026-03-10

### Product Direction Reset

- Removed the separate high-precision offline branch.
- Removed all audio synthesis code and configuration.
- Kept one supported product path: the realtime GUI simulator with plots and dyno-style bench operation.

### Retained Technical Direction

- Improve the reduced-order intake and exhaust model instead of maintaining a second solver.
- Keep parameter ranges physically plausible through config auditing.
- Make bench behavior closer to an EDM-style speed-hold test.
- Keep the dashboard readable on real displays by tightening layout density, enabling scrolling, and collapsing lower-priority modules.

### Recent Solver Features Still In Scope

- `p-theta` overlay for all 4 cylinders over `0..720 degCA`
- bench absorber with automatic load control
- internal-EGR and charge-property effects in the realtime path
- vehicle-equivalent road load with reflected inertia
- accuracy-first GUI stepping that is not forced to follow wall clock

### Documentation Restructure

- Split the top-level README into overview-only content plus dedicated manuals in `docs/`.
- Added a self-contained user manual in English and Japanese.
- Added a self-contained model reference in English and Japanese.
- Moved equations, implementation mapping, and literature sources out of the top-level README so complex material can evolve without turning README into a monolith.

### Equation Expansion

- Expanded `docs/MODEL_REFERENCE.md` and `docs/MODEL_REFERENCE.ja.md` so the differential equations, algebraic closures, `p-V` reconstruction, and RK3 update rule are written explicitly.
- Added a source map so each major equation family can be traced to the literature base or identified as an implementation-specific surrogate.

### Immediate Next Steps

- tighten runner and manifold closures using literature-backed reduced-order models
- calibrate the realtime path against well-documented production engines
- improve per-cylinder fueling controls in the GUI without bloating the config surface
