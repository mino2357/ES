# High Precision Headless Model

## Purpose

`es_cli` is the primary offline solver name and `es_hp` remains as a compatibility alias for environments where:

- realtime GUI is unnecessary,
- external dependencies are undesirable,
- input should be YAML,
- output should be CSV.

This path is compiled with:

```bash
cargo run --no-default-features --bin es_cli -- sweep config/high_precision.yaml
```

The command writes:

- `torque_curve.csv`
- `pv_<rpm>rpm.csv`

to the configured `output_dir`.

Parameter audit and retained physical ranges are summarized in `HIGH_PRECISION_PARAMETER_AUDIT.md`.

## Terminology And Reading Conventions

This document is also intended to be self-contained. The terms below are the local definitions used in this file.

- `headless`: runs without GUI or audio dependencies.
- `offline`: not constrained by realtime wall-clock execution.
- `std-only`: implemented only with the Rust standard library on this execution path.
- `single-zone cylinder model`: one thermodynamic gas zone per cylinder rather than a spatially resolved burned / unburned field model.
- `finite-volume / finite-inertance runner line`: a runner model represented by pressure cells plus face mass-flow states, instead of an analytic steady-flow pipe map.
- `ODE`: ordinary differential equation.
- `RK4`: explicit fourth-order Runge-Kutta integration.
- `SI`: spark ignition.
- `plenum`: shared intake control volume upstream of the intake runners.
- `collector`: shared exhaust control volume downstream of the exhaust runners.
- `runner`: the duct between the shared volume and the cylinder-side valve boundary.
- `lambda`: air-fuel equivalence ratio relative to stoichiometric air-fuel ratio.
- `degCA`: crank-angle degrees over a `0 .. 720` four-stroke cycle.
- `TDC` / `BDC`: top dead center / bottom dead center.
- `fixed-step`: numerical integration with one configured timestep size rather than an adaptive local-error controller.
- `sweep`: the ordered list of RPM points used to build a torque curve.

## Dependency Isolation

The existing GUI binary remains behind the Cargo feature `gui`.
The headless solver lives in `src/bin/es_cli.rs`, `src/bin/es_hp.rs`, plus `src/hp/*` and uses only `std`.

Build modes:

- GUI: `cargo run`
- std-only headless: `cargo run --no-default-features --bin es_cli -- sweep config/high_precision.yaml`

## Mathematical Model

The current high-precision path is an offline crank-angle solver with:

- fixed 4 cylinders phased by 180 crank degrees,
- single-zone cylinder thermodynamics,
- slider-crank geometry,
- Wiebe heat release,
- wall heat transfer,
- common intake plenum and exhaust collector,
- segmented intake and exhaust runners represented as finite-volume / finite-inertance lines.

### State

For each cylinder:

```math
\mathbf{x}_{cyl} =
\begin{bmatrix}
m_c & T_c
\end{bmatrix}^{\mathsf T}
```

Global states:

```math
\mathbf{x}_{sys} =
\begin{bmatrix}
p_{pl} & p_{col} & \{p_{ir,j}, \dot m_{ir,j}\} & \{p_{er,j}, \dot m_{er,j}\}
\end{bmatrix}^{\mathsf T}
```

### Cylinder Equations

Ideal gas closure:

```math
p_c = \frac{m_c R T_c}{V(\theta)}
```

Mass conservation:

```math
\dot m_c = \dot m_{iv} - \dot m_{ev}
```

Open-system first law:

```math
\frac{dU_c}{dt} =
\dot Q_{comb}
- \dot Q_{wall}
- p_c \frac{dV}{dt}
+ \sum \dot m_{in} h_{in}
- \sum \dot m_{out} h_{out}
```

with

```math
U_c = m_c c_v T_c
```

which gives

```math
m_c c_v \dot T_c
=
\frac{dU_c}{dt}
- c_v T_c \dot m_c
```

### Runner / Manifold Equations

Each runner is discretized into `N` isothermal pressure cells and `N` face mass-flow states.
`N` is treated as a numerical-resolution setting in YAML, not as a physical engine parameter.

Cell pressure dynamics:

```math
\dot p_j = \frac{R T}{V_j} (\dot m_{j,in} - \dot m_{j,out})
```

Face momentum surrogate:

```math
\frac{d \dot m_j}{dt}
=
\frac{A}{L_j}
\left(
p_{left} - p_{right} - \Delta p_{loss,j}
\right)
```

with quadratic loss:

```math
\Delta p_{loss,j}
=
\operatorname{sgn}(\dot m_j)
K_j
\frac{\dot m_j^2}{2 \rho A^2}
```

### Valves and Throttle

Valve lift follows a smooth cosine lobe between configured opening and closing angles.

Effective area:

```math
A_{eff} = \min(\pi D_v L_v,\; A_{seat})
```

Throttle, tailpipe, intake valve, and exhaust valve flows all use the same bidirectional compressible-orifice model, with the upstream side selected from the instantaneous pressure ordering.

### Combustion

Combustion heat release is modeled with a Wiebe function:

```math
x_b(\theta) = 1 - \exp(-a y^{m+1}), \qquad y = \frac{\theta - \theta_0}{\Delta \theta_b}
```

```math
\dot Q_{comb} =
m_f \, LHV \,
\frac{dx_b}{d\theta}
\frac{d\theta}{dt}
```

Fuel mass for the next cycle is updated at intake valve closing from the trapped cylinder mass and the configured lambda target.

To keep the YAML from growing non-physical shape knobs, the checked-in headless path fixes the Wiebe-shape pair internally at `a = 5.0`, `m = 2.0`. The YAML retains only start timing, burn duration, lambda, and fuel LHV.

### Numerical Integration

The solver uses fixed-step classical RK4 in time, with crank-angle step `dt_deg` from YAML converted by

```math
\Delta t = \frac{\Delta \theta}{\omega}
```

There is no realtime pacing logic in this path.
The runner cell counts also live in `numerics`, since they control discretization rather than hardware.

## Validation Intent

This solver is intended to be reviewed against:

- mean-value / control-oriented spark-ignition (SI) engine literature for manifold filling and air-path structure,
- single-zone combustion literature for the closed-cylinder energy equation,
- reference production engines for calibration targets.

The headless YAML is intentionally strict:

- unknown keys are rejected,
- every remaining field is range-checked against a physically plausible sanity envelope,
- removed keys such as cylinder count and free Wiebe shape coefficients are treated as schema errors.

The current checked-in sample configuration is intentionally close to a modern high-compression long-stroke `2.0 L` naturally aspirated inline-4.

## Tests

The std-only path includes tests for:

- YAML subset parsing,
- exact RK4 agreement for exponential decay,
- sign consistency of bidirectional orifice flow,
- finite derivatives at a resting equilibrium-like state,
- positive brake torque over a short reference sweep.

Run them with:

```bash
cargo test --no-default-features --bin es_cli -- --nocapture
```

## Current Limits

This is the first offline foundation, not yet a full production-grade 1D gas-dynamics code.
Current simplifications include:

- runners are isothermal,
- cylinder is single-zone,
- no knock model,
- no residual-gas chemistry,
- no detailed friction or heat-transfer correlation switching,
- no measured-engine auto-fit pipeline yet.

Those are the next expansion points.
