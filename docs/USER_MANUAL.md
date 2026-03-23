# ES User Manual

## Scope

`ES` is a CLI tool for sweeping steady operating points of the repository's inline-4 engine model.
It uses YAML input, solves the physical model transiently until each operating point settles, and writes text files for later plotting.

## Build And Run

```bash
cargo run --release -- sweep --config config/sim.yaml --output-dir output/cli
```

## Command-Line Options

- `--config <path>`: YAML file to load. Missing sections fall back to defaults.
- `--output-dir <dir>`: destination directory for all generated files.
- `--rpm-start <rpm>`: optional sweep start. Default is `engine.default_target_rpm`.
- `--rpm-end <rpm>`: optional sweep end. Default is `engine.max_rpm`.
- `--rpm-step <rpm>`: sweep interval. Default is `200`.
- `--settle-time <s>`: transient settling time before averaging. Default is `12`.
- `--average-time <s>`: averaging window after settling. Default is `1.5`.
- `--diagnostic-samples <n>`: samples used for `p-theta` and `T-S`. Minimum is `180`, default is `720`.

## Minimal YAML

The checked-in configuration is intentionally minimal:

```yaml
environment:
  ambient_pressure_pa: 101325.0
  intake_temp_k: 305.0
  exhaust_temp_k: 880.0
  dt: 0.001
engine:
  compression_ratio: 13.0
  bore_m: 0.0805
  stroke_m: 0.0976
  default_target_rpm: 850.0
  max_rpm: 7000.0
control_defaults:
  ignition_timing_deg: 12.0
```

The CLI fills the rest from Rust defaults.

## Output Files

### `torque_curve.tsv`

Columns:

- `target_rpm`
- `mean_rpm`
- `torque_nm`
- `power_kw`
- `map_kpa`
- `air_flow_gps`
- `eta_indicated`
- `load_cmd`
- `output_dir`

### Per-point directories

Each `point_XXXXrpm/` directory contains:

- `pv.tsv`: `volume_ratio`, `pressure_pa`
- `ptheta.tsv`: crank angle and four cylinder pressure traces
- `ts.tsv`: crank angle, apparent single-zone temperature, relative entropy, pressure, and volume ratio
- `summary.tsv`: single-row summary of the settled operating point

## Quick gnuplot Usage

```bash
gnuplot -e "cd 'output/cli'" plot_torque_curve.gp
```

You can also plot any point-level file directly, for example:

```gnuplot
plot 'point_3000rpm/pv.tsv' using 'volume_ratio':'pressure_pa' with lines
plot 'point_3000rpm/ts.tsv' using 'entropy_rel_j_per_kgk':'temperature_k' with lines
```
