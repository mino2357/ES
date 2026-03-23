# ES

`ES` is a headless CLI inline-4 engine simulator for accuracy-first operating-point sweeps.
It integrates the repository's documented 0D ODE model and exports brake-torque-centered TSV artifacts for later analysis with `gnuplot` or spreadsheets.

## Scope

This repository's maintained product path is the CLI sweep workflow:

- YAML-driven operating-point sweeps from idle to redline
- brake torque / power export suitable for dyno-style comparison
- per-point `p-V`, `p-theta`, and `T-S` diagnostic data
- documentation that explicitly maps equations, parameters, and outputs back to Rust code

Historical GUI planning documents have been removed from the maintained documentation set.
The solver and documents now assume a CLI-first workflow only.

## Build And Run

```bash
cargo run --release -- sweep --config config/reference_na_i4.yaml --output-dir output/reference_s2000_like
```

Common options:

- `--rpm-step 200`: RPM interval for the sweep
- `--rpm-start 1000`: explicit sweep start speed
- `--rpm-end 8500`: explicit sweep end speed
- `--settle-time 0.30`: transient settling time per operating point
- `--average-time 0.10`: averaging window after settling
- `--diagnostic-samples 180`: samples for `p-theta` / `T-S`

## Outputs

The CLI writes these files under the output directory:

- `torque_curve.tsv`: one row per operating point with `brake_torque_nm`, `brake_power_kw`, `net_torque_nm`, and `load_torque_nm`
- `plot_torque_curve.gp`: minimal `gnuplot` script for the brake torque curve
- `run_manifest.yaml`: run metadata and high-level plausibility summary
- `torque_curve_assessment.md`: heuristic audit of the generated brake torque curve
- `point_XXXXrpm/pv.tsv`: `p-V` loop data
- `point_XXXXrpm/ptheta.tsv`: four-cylinder `p-theta` data
- `point_XXXXrpm/ts.tsv`: `T-S` data
- `point_XXXXrpm/summary.tsv`: per-point brake / net torque summary

Example:

```bash
gnuplot -e "cd 'output/reference_s2000_like'" plot_torque_curve.gp
```

## Reference Calibration Intent

The checked-in `config/reference_na_i4.yaml` is now tuned as an **high-rev naturally aspirated 2.0 L teaching case**:

- geometry remains approximately `1.998 L` (`86 mm x 86 mm`)
- redline-side sweep extends to `8600 rpm`
- VE and wave-action surrogates are biased toward a high-speed naturally aspirated character
- the target is a curve that roughly resembles published high-rev naturally aspirated 2.0 L trends, not an exact production re-creation

A representative sweep generated with the current model is summarized in `docs/USER_MANUAL.md` and `docs/USER_MANUAL.ja.md`.

## Documentation Map

- [docs/USER_MANUAL.md](docs/USER_MANUAL.md): CLI workflow, artifact definitions, and high-rev NA 2.0 L sample table
- [docs/MODEL_REFERENCE.md](docs/MODEL_REFERENCE.md): ODE system, algebraic closures, implementation map, and source list
- [docs/IMPLEMENTATION_DIRECTION.ja.md](docs/IMPLEMENTATION_DIRECTION.ja.md): repository-wide implementation charter
- [README.ja.md](README.ja.md): Japanese overview
- [docs/USER_MANUAL.ja.md](docs/USER_MANUAL.ja.md): Japanese user manual
- [docs/MODEL_REFERENCE.ja.md](docs/MODEL_REFERENCE.ja.md): Japanese model reference
