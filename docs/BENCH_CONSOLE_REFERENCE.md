# Bench Console Reference

This document is self-contained.
It explains which published engine-test-cell features informed the current bench-side dashboard, and how those features map to the implementation in this repository.

## Scope

This dashboard is not a pixel-level reproduction of any one vendor console.
The intent is narrower:

- preserve the repository's existing operator display and plots
- add a reasonable subset of engine-dyno switches, annunciators, and instruments
- keep the load model reduced-order and numerically stable
- tie each added feature to public source material

## Source-Informed Features

### Controller Functions

The current bench console reflects common engine-dynamometer controller functions described by HORIBA SPARC Engine and Froude Texcel / InCell:

- speed control
- torque or absorber control
- throttle and dynamometer control from a manual console
- status and message annunciation
- safety / interlock signaling
- road-load simulation as an optional control mode

These are drawn from:

- HORIBA SPARC Engine:
  https://www.horiba.com/usa/medical/products/detail/action/show/Product/sparc-95/
- Froude Texcel V12 PRO:
  https://www.froudedyno.com/products/control-systems/texcel-v12-controller
- Froude InCell:
  https://www.froudedyno.com/products/control-systems/incell-control-and-data-acquisition-system

### Switches And Annunciators

The current `Bench Console` and header annunciators use a source-backed subset of common test-cell controls:

- `E-STOP`
- `DYNO EN`
- `INTERLOCK`
- `VENT`
- `COOLING`
- `SPD CTRL`
- `PWR LIM`

Why these were chosen:

- Froude InCell explicitly describes a `Desk Top Control (DTC) Panel` with `Mode selection/engine controls` and `E-Stop/reset built in`.
- HORIBA SPARC Engine explicitly describes controller-side `control`, `monitoring`, and `data acquisition`, and highlights integrated `safety modules`.
- Froude Texcel V12 PRO explicitly describes alarm-checked measured data and manual control of both engine and dynamometer.

The repository does not attempt to simulate every peripheral named by those sources.
It implements only the switches that materially change operator understanding or solver behavior in this reduced-order path.

## Instrument Mapping

The current bench-facing instrumentation is grouped into three layers.

### Primary Live Instruments

These remain in the central `Operator Display`:

- engine speed
- brake torque
- brake power
- absorber torque
- air charge
- intake pressure
- indicated efficiency
- absorber limit

This matches the source pattern of manual-control consoles that foreground speed, torque, power, and status rather than raw state variables.

### Bench Console Metrics

The `Bench Console` module adds:

- interlock status
- control mode
- absorber torque
- absorber power versus rated power
- available torque at current speed
- overspeed threshold
- absorber rotor inertia
- power-limit torque at current speed

This follows the official controller descriptions that combine control, instrumentation, alarms, and peripheral-state monitoring on one operator-facing page.

### Lower-Level State Bus

The `Status Bus` and `Sensor / State Bus` retain the reduced-order model observables:

- shaft and absorber torque split
- road-load or absorber mode
- reflected inertia
- manifold and runner pressures
- charge temperature
- internal EGR

This is a repository-specific extension, not a claim that commercial controllers present the same exact layout.

## Reduced-Order Bench Model

The current load path is intentionally simple.

### Brake Dyno Mode

The resisting-torque envelope is

```math
\tau_{ref}(\omega) = a_0 + a_1 \omega + a_2 \omega^2
```

and the actual available absorbing torque is limited by power:

```math
\tau_{avail}(\omega) =
\min\left(
\tau_{ref}(\omega),
\frac{P_{max}}{\omega}
\right)
```

The command signal is a shaped fraction

```math
s = \operatorname{sgn}(u_{load}) |u_{load}|^{n_{load}}
```

and the absorber torque is

```math
\tau_{load} = s \tau_{avail}
```

### Vehicle-Equivalent Mode

The road-load part remains

```math
F_{road} = F_{roll} + F_{grade} + F_{drag}
```

with the wheel-side torque reflected back to the engine shaft.
The same absorber power limit is then applied on top of that reference torque.

### Reflected Inertia

The bench model now includes a constant absorber rotor inertia:

```math
J_{ref} = J_{abs} + J_{vehicle,ref}
```

where

```math
J_{vehicle,ref}
=
|s| m_{eq} \left(\frac{r_w}{i_{tot}}\right)^2
```

and `J_abs` is present even in `Brake dyno` mode.
This reflects the fact that a coupled dynamometer rotor contributes inertia even when the road-load portion is not active.

## Implementation Map

- bench-side switches and annunciators:
  [src/dashboard.rs](../src/dashboard.rs)
- bench-side widgets and cards:
  [src/dashboard/widgets.rs](../src/dashboard/widgets.rs)
- external-load configuration and defaults:
  [src/config.rs](../src/config.rs)
- plausibility audit for the load model:
  [src/config/audit.rs](../src/config/audit.rs)
- reduced-order external-load equations:
  [src/simulator.rs](../src/simulator.rs)
- checked-in runtime parameters:
  [../config/sim.yaml](../config/sim.yaml)

## Limits

This bench model is still a reduced-order surrogate.
It does not yet include:

- absorber-current dynamics
- coolant or oil thermal loops
- load-cell compliance
- detailed driveline lash
- vendor-specific automation workflows

The current goal is not certification-grade stand simulation.
It is an operator-readable, source-informed bench abstraction that stays aligned with the live engine model.

## Related Documents

- [../README.md](../README.md)
- [USER_MANUAL.md](USER_MANUAL.md)
- [MODEL_REFERENCE.md](MODEL_REFERENCE.md)
- [BENCH_CONSOLE_REFERENCE.ja.md](BENCH_CONSOLE_REFERENCE.ja.md)
