# ES

`ES` は、accuracy-first の運転点 sweep を行う headless CLI inline-4 engine simulator です。
repository 内の非平均化 `0D` engine model を解き、`gnuplot` などで扱いやすい表形式データを出力します。

## 対象範囲

現在この repository がサポートする製品経路は 1 つだけです。

- command-line operating-point sweep
- idle 付近から指定 RPM 刻みでの torque curve 出力
- 各 operating point に対する `p-V`、`p-theta`、`T-S` 診断データ出力
- 省略可能 section を持つ YAML 駆動設定

現行の製品経路に GUI runtime はありません。

## Build と Run

```bash
cargo run --release -- sweep --config config/sim.yaml --output-dir output/cli
```

代表的な option:

- `--rpm-step 200`: sweep の RPM 刻み
- `--rpm-start 850`: sweep 開始回転数
- `--rpm-end 7000`: sweep 終了回転数
- `--settle-time 12`: 各 operating point の過渡整定時間
- `--average-time 1.5`: 整定後の平均化時間

## 出力

CLI は output directory 配下へ次を出力します。

- `torque_curve.tsv`: 各 operating point の要約。`gnuplot` にそのまま渡しやすい
- `plot_torque_curve.gp`: torque curve 用の最小 `gnuplot` script
- `run_manifest.yaml`: 実行 metadata
- `torque_curve_assessment.md`: トルクカーブが「それらしい」かを教育用の経験則で点検した report
- `point_XXXXrpm/pv.tsv`: `p-V` loop
- `point_XXXXrpm/ptheta.tsv`: 4 気筒 `p-theta`
- `point_XXXXrpm/ts.tsv`: `T-S`
- `point_XXXXrpm/summary.tsv`: その operating point の要約

例:

```bash
gnuplot -e "cd 'output/cli'" plot_torque_curve.gp
```

## 設定

checked-in の [config/sim.yaml](config/sim.yaml) は意図的に最小構成です。
足りない section は `src/config.rs` の default で補われます。
これにより、CLI では必要最低限の YAML を保ちつつ、詳細な物理・数値 model は code 側に維持できます。

## ドキュメント構成

- [docs/USER_MANUAL.md](docs/USER_MANUAL.md): 英語版 CLI manual
- [docs/MODEL_REFERENCE.md](docs/MODEL_REFERENCE.md): 数式、closure、実装対応
- [docs/IMPLEMENTATION_DIRECTION.ja.md](docs/IMPLEMENTATION_DIRECTION.ja.md): CLI-first に更新した既定実装方針
- [README.md](README.md): 英語版 overview
- [docs/USER_MANUAL.ja.md](docs/USER_MANUAL.ja.md): 日本語版 user manual
- [docs/MODEL_REFERENCE.ja.md](docs/MODEL_REFERENCE.ja.md): 日本語版 model reference

## 教育用の参照ケース

実在しそうな自然吸気 2.0 L 級 inline-4 の教育用参照ケースを [config/reference_na_i4.yaml](config/reference_na_i4.yaml) に追加しています。

まず 1000〜5000 rpm で次を実行すると、低中速でトルクが立ち上がり、その後に落ちていく「教科書的に読みやすい」曲線かを `torque_curve_assessment.md` で確認できます。

```bash
cargo run --release -- sweep \
  --config config/reference_na_i4.yaml \
  --output-dir output/reference_na_i4 \
  --rpm-start 1000 --rpm-end 5000 --rpm-step 1000 \
  --settle-time 0.2 --average-time 0.1
```

注: 現行 reduced-order model は回転上限付近で brake/load のつり合いが鋭敏になりやすいため、教育用途ではまず安定域の sweep で傾向を確認し、その後に上限側を詰めるのが安全です。
