# ES

`ES` is a headless CLI inline-4 engine simulator for accuracy-first operating-point sweeps.
It solves the repository's non-averaged 0D engine model and writes tabular outputs that are easy to inspect with `gnuplot` or other plotting tools.

## Scope

This repository now supports one product path only:

- command-line operating-point sweep
- torque-curve export from idle speed upward at a configurable RPM increment
- per-operating-point `p-V`, `p-theta`, and `T-S` diagnostic data export
- YAML-driven configuration with defaults for omitted sections

There is no GUI runtime in the current product path.

## Build And Run

```bash
cargo run --release -- sweep --config config/sim.yaml --output-dir output/cli
```

Common options:

- `--rpm-step 200`: RPM interval for the sweep
- `--rpm-start 850`: explicit sweep start speed
- `--rpm-end 7000`: explicit sweep end speed
- `--settle-time 12`: transient settling time per operating point
- `--average-time 1.5`: averaging window after settling

## Outputs

The CLI writes these files under the output directory:

- `torque_curve.tsv`: one row per operating point for quick `gnuplot` use
- `plot_torque_curve.gp`: minimal `gnuplot` script for the torque curve
- `run_manifest.yaml`: run metadata
- `point_XXXXrpm/pv.tsv`: `p-V` loop data
- `point_XXXXrpm/ptheta.tsv`: four-cylinder `p-theta` data
- `point_XXXXrpm/ts.tsv`: `T-S` data
- `point_XXXXrpm/summary.tsv`: operating-point summary

Example:

```bash
gnuplot -e "cd 'output/cli'" plot_torque_curve.gp
```

## Configuration

The checked-in [config/sim.yaml](config/sim.yaml) is intentionally minimal.
Any omitted section falls back to the defaults defined in `src/config.rs`.
This lets the CLI work from a small YAML file while keeping the detailed physical and numerical model in code.

## Documentation Map

- [docs/USER_MANUAL.md](docs/USER_MANUAL.md): CLI usage and output interpretation
- [docs/MODEL_REFERENCE.md](docs/MODEL_REFERENCE.md): model equations, closures, and implementation map
- [docs/IMPLEMENTATION_DIRECTION.ja.md](docs/IMPLEMENTATION_DIRECTION.ja.md): default implementation charter, now aligned to a CLI-first workflow
- [README.ja.md](README.ja.md): Japanese overview
- [docs/USER_MANUAL.ja.md](docs/USER_MANUAL.ja.md): Japanese user manual
- [docs/MODEL_REFERENCE.ja.md](docs/MODEL_REFERENCE.ja.md): Japanese model reference
