# ES User Manual

## 対象範囲

`ES` は、repository の inline-4 engine model の定常運転点を sweep する CLI tool です。
YAML を入力に取り、各 operating point が整定するまで過渡計算し、その後に plot 用の text file を出力します。

## Build と Run

```bash
cargo run --release -- sweep --config config/sim.yaml --output-dir output/cli
```

## Command-Line Option

- `--config <path>`: 読み込む YAML。足りない section は default で補完されます
- `--output-dir <dir>`: 出力先 directory
- `--rpm-start <rpm>`: sweep 開始回転数。既定は `engine.default_target_rpm`
- `--rpm-end <rpm>`: sweep 終了回転数。既定は `engine.max_rpm`
- `--rpm-step <rpm>`: sweep 刻み。既定は `200`
- `--settle-time <s>`: 平均化前の整定時間。既定は `12`
- `--average-time <s>`: 整定後の平均化時間。既定は `1.5`
- `--diagnostic-samples <n>`: `p-theta` と `T-S` の sample 数。最小 `180`、既定 `720`

## 最小 YAML

checked-in の設定は意図的に最小です。

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

足りない値は Rust 側 default で補われます。

## 出力ファイル

### `torque_curve.tsv`

列:

- `target_rpm`
- `mean_rpm`
- `torque_nm`
- `power_kw`
- `map_kpa`
- `air_flow_gps`
- `eta_indicated`
- `load_cmd`
- `output_dir`

### 各 operating point directory

各 `point_XXXXrpm/` には次が入ります。

- `pv.tsv`: `volume_ratio`, `pressure_pa`
- `ptheta.tsv`: crank angle と 4 気筒 pressure trace
- `ts.tsv`: crank angle、apparent single-zone temperature、relative entropy、pressure、volume ratio
- `summary.tsv`: その operating point の 1 行要約
- `../torque_curve_assessment.md`: sweep 全体に対する教育用 plausibility report

## gnuplot の簡単な使い方

```bash
gnuplot -e "cd 'output/cli'" plot_torque_curve.gp
```

個別 file も直接描けます。たとえば:

```gnuplot
plot 'point_3000rpm/pv.tsv' using 'volume_ratio':'pressure_pa' with lines
plot 'point_3000rpm/ts.tsv' using 'entropy_rel_j_per_kgk':'temperature_k' with lines
```


## 教育用の plausibility check

`torque_curve_assessment.md` では、次の 4 つの経験則を明示します。

1. sweep 全域で正のトルクか
2. 低回転から中回転へ向かってトルクが立ち上がるか
3. 高回転端でトルクが再び低下するか
4. `P = \tau \omega` に従い power peak が torque peak 以後に来るか

これは厳密な妥当性証明ではありませんが、研究メモや教育演習で「まず何を見るべきか」を共有する簡潔な監査票として使えます。

推奨の教育用参照ケースは [../config/reference_na_i4.yaml](../config/reference_na_i4.yaml) です。
まずは `--rpm-end 5000` 付近までの sweep で、極端な上限回転数 artifacts を避けつつ曲線形状を確認してください。
