# ES

`ES` は、accuracy-first な operating-point sweep 用の headless CLI inline-4 engine simulator です。
repository で定義している 0D ODE model を積分し、dyno 的に読みやすい brake torque 中心の TSV artifact を出力します。

## 対象範囲

保守対象の製品経路は CLI sweep workflow のみです。

- YAML 駆動の operating-point sweep
- brake torque / power の表形式出力
- 各 operating point の `p-V`、`p-theta`、`T-S` 診断データ
- 数式、parameter、出力列が Rust code にどう対応するかを明記した文書群

過去の GUI 計画文書は、現行の保守対象文書から削除しました。
現在の solver と文書は CLI-first 前提で統一しています。

## Build And Run

```bash
cargo run --release -- sweep --config config/reference_na_i4.yaml --output-dir output/reference_s2000_like
```

主な option:

- `--rpm-step 200`: RPM 刻み
- `--rpm-start 1000`: sweep 開始回転数
- `--rpm-end 8500`: sweep 終了回転数
- `--settle-time 0.30`: 各 operating point の整定時間
- `--average-time 0.10`: 整定後の平均化時間
- `--diagnostic-samples 180`: `p-theta` / `T-S` の出力点数

## 出力

output directory 直下に次を書き出します。

- `torque_curve.tsv`: `brake_torque_nm`、`brake_power_kw`、`net_torque_nm`、`load_torque_nm` を含む torque curve
- `plot_torque_curve.gp`: brake torque 用の最小 `gnuplot` script
- `run_manifest.yaml`: 実行 metadata と簡易妥当性評価
- `torque_curve_assessment.md`: brake torque curve の評価記録
- `point_XXXXrpm/pv.tsv`: `p-V`
- `point_XXXXrpm/ptheta.tsv`: 4 気筒 `p-theta`
- `point_XXXXrpm/ts.tsv`: `T-S`
- `point_XXXXrpm/summary.tsv`: 各点の brake / net torque summary

例:

```bash
gnuplot -e "cd 'output/reference_high_rev_na'" plot_torque_curve.gp
```

## 参照 calibration の意図

checked-in の `config/reference_na_i4.yaml` は、**高回転型自然吸気 2.0 L の参照ケース**として調整しています。

- 幾何はほぼ `1.998 L` (`86 mm x 86 mm`)
- 赤線側の sweep を `8600 rpm` まで拡張
- VE と wave-action surrogate を高回転型の自然吸気らしい傾向へ寄せる
- 目的は 高回転型自然吸気 2.0 L の公開 torque trend におおよそ似た曲線を作ること

現在の代表 sweep 結果は `docs/USER_MANUAL.md` / `docs/USER_MANUAL.ja.md` に数表で記載しています。

## 文書マップ

- [docs/USER_MANUAL.md](docs/USER_MANUAL.md): CLI workflow、artifact 定義、高回転 NA 2.0 L 比較表
- [docs/MODEL_REFERENCE.md](docs/MODEL_REFERENCE.md): ODE、algebraic closure、実装対応、出典
- [docs/IMPLEMENTATION_DIRECTION.ja.md](docs/IMPLEMENTATION_DIRECTION.ja.md): repository 全体の実装方針
- [docs/USER_MANUAL.ja.md](docs/USER_MANUAL.ja.md): 日本語 user manual
- [docs/MODEL_REFERENCE.ja.md](docs/MODEL_REFERENCE.ja.md): 日本語 model reference
