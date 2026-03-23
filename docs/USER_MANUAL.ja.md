# ES User Manual

## 1. 目的

`ES` は、repository 内の inline-4 engine model の周期定常 operating point を sweep するための CLI tool です。
各 operating point は `MODEL_REFERENCE.ja.md` に書いた同じ ODE を過渡積分して求めます。つまり、torque curve 出力のためだけの別 map runtime には切り替えていません。

## 2. 推奨 reference run

```bash
cargo run --release -- sweep \
  --config config/reference_na_i4.yaml \
  --output-dir output/reference_high_rev_na \
  --rpm-start 1000 \
  --rpm-end 8500 \
  --rpm-step 1000 \
  --settle-time 0.30 \
  --average-time 0.10 \
  --diagnostic-samples 180
```

高回転型自然吸気 2.0 L の brake torque curve を、controller 過渡をある程度抑えつつ確認したいときの標準 run です。

## 3. なぜ CLI は brake torque を出すのか

speed-hold された dyno 的な sweep では、通常ほしいのは residual acceleration torque ではなく **brake torque** です。
そのため CLI は次を出力します。

- `brake_torque_nm = net_torque_nm + load_torque_nm`
- `brake_power_kw = brake_torque_nm * omega`
- `net_torque_nm`: absorber 負荷を引いた後に残る加速 torque
- `load_torque_nm`: brake / absorber model が現在受け持っている torque

これにより、公開 dyno chart と見比べやすい列を出しつつ、内部の ODE bookkeeping も失いません。

## 4. 出力 file

### 4.1 `torque_curve.tsv`

列:

- `target_rpm`
- `mean_rpm`
- `brake_torque_nm`
- `brake_power_kw`
- `net_torque_nm`
- `load_torque_nm`
- `map_kpa`
- `air_flow_gps`
- `eta_indicated`
- `load_cmd`
- `output_dir`

### 4.2 `point_XXXXrpm/summary.tsv`

各点 summary の列:

- `rpm`
- `brake_torque_nm`
- `brake_power_kw`
- `net_torque_nm`
- `load_torque_nm`
- `brake_bmep_bar`
- `map_kpa`
- `air_flow_gps`
- `eta_indicated`

### 4.3 診断 trace

各 `point_XXXXrpm/` には次も含まれます。

- `pv.tsv`: `volume_ratio`, `pressure_pa`
- `ptheta.tsv`: crank angle と 4 気筒 pressure trace
- `ts.tsv`: crank angle、temperature、relative entropy、pressure、volume ratio

## 5. 高回転型自然吸気 2.0 L reference table

現在の checked-in reference case は、汎用的な高回転自然吸気 torque trend に対して **近似的** な一致を狙ったものです。
目的は、文書化された reduced-order model と妥当な物理定数を使って「高回転型 NA engine らしい大勢」を再現することです。

上の推奨 command で得られる代表結果:

| Mean rpm | Brake torque [Nm] | Brake power [kW] | Net torque [Nm] |
|---:|---:|---:|---:|
| 1005 | 157.6 | 16.6 | 16.3 |
| 1999 | 134.3 | 28.1 | 15.5 |
| 2999 | 173.0 | 54.3 | 8.9 |
| 4002 | 178.5 | 74.8 | 6.8 |
| 5000 | 149.4 | 78.2 | 4.4 |
| 6000 | 158.5 | 99.6 | 4.6 |
| 6999 | 141.6 | 103.8 | 3.4 |
| 7999 | 92.9 | 77.8 | 2.0 |
| 8499 | 68.4 | 60.9 | 1.1 |

読み方:

- 現在の model は、正の WOT brake torque と、torque peak より後ろにある power peak は捉えています
- 一方で、高回転側の torque retention は現状でも弱く、この差を詰めるには数理 model と高回転域の gas exchange 物理の見直しが必要です
- この不一致を隠さずに明記することで、今後の calibration がどの model 項に由来するか追跡しやすくしています

## 6. gnuplot 例

```bash
gnuplot -e "cd 'output/reference_high_rev_na'" plot_torque_curve.gp
```

## 7. model と code の対応

curve 調整時は、まず次の code path を見るのが安全です。

- CLI sweep と brake torque 出力: `src/cli.rs`
- ODE 積分と torque bookkeeping: `src/simulator.rs`
- parameter default と config schema: `src/config.rs`
- 高回転型自然吸気 2.0 L reference case: `config/reference_na_i4.yaml`

## 8. 参照出典

reference case の文書化と parameter 選定では、次を明示的な anchor としています。

1. 高回転型自然吸気 2.0 L SI engine の公開 brochure / catalog にある出力・トルク帯。
2. Heywood, *Internal Combustion Engine Fundamentals* (2nd ed.)。
3. Stone, *Introduction to Internal Combustion Engines* (4th ed.)。
4. Woschni 系の cylinder heat transfer と、single-zone heat-release / pressure reconstruction の標準文献。

calibration や文書を更新するときは、これらの出典を見える形で残してください。


## 9. model 改良 workflow

計算した torque curve が現実から離れたときにやるべきことは、外部補正の経路を足すことではありません。
やるべきことは、文書化された ODE 系を解くことに専念しつつ、現在の closure で捉えられていない物理現象を特定し、その現象を数理 model に組み込んで、更新後の系をもう一度解くことです。

この repository では、次の反復を基本にします。

- ODE model を operating-point sweep で解く
- curve が期待挙動から外れる箇所を確認する
- 足りない、または単純化しすぎた物理現象を特定する
- reduced-order model と closure を改良する
- 改良後の ODE 系を再び解いて結果を確認する

優先すべきなのは、別経路の補正ではなく、数理 model そのものの改善です。
