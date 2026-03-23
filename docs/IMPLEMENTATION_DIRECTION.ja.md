# 実装方針書

この文書は、本 repository の既定実装方針を CLI-first の headless engine solver として定義するものです。
ユーザーから明示的な上書きがない限り、今後の設計、実装、文書更新はこの文書に従います。

## 1. 目標

- 物理ベースの `0D CAE` engine model を保つ
- 平均化 model へ逃げず、数理 model の式を忠実に解く
- idle 回転数から高回転までの torque curve を CLI から再現可能に出力する
- 各 operating point で `p-V`、`p-theta`、`T-S` を再描画できるデータを残す
- 必要最低限の YAML で再現できる headless workflow を維持する
- 出力を `gnuplot` など外部 tool で簡単に可視化できるようにする

## 2. 基本方針

- 製品経路は GUI ではなく CLI に一本化する
- UI 都合の近似や frame budget は設計理由から外す
- 物理 model、数値積分、入出力生成、文書は責務を分離する
- 省略された YAML 項目は Rust 側 default で補い、最小設定でも再現可能にする
- 運転点 sweep は transient integration で整定させ、その後に平均値と診断波形を保存する
- `gnuplot` 互換の TSV を標準 artifact にする

## 3. 推奨アーキテクチャ

### 3.1 Core model

- 物理ベースの `0D` engine model
- valve / combustion / gas-path / friction / load の closure
- crank-angle を含む状態方程式の時間積分

### 3.2 CLI workflow

- YAML 読み込み
- operating point sweep
- speed-hold 相当の load adjustment
- torque curve 集約出力
- point-by-point diagnostics 出力

### 3.3 Artifact

- `torque_curve.tsv`
- `plot_torque_curve.gp`
- per-point `pv.tsv`
- per-point `ptheta.tsv`
- per-point `ts.tsv`
- run metadata
- torque-curve 評価記録 (`torque_curve_assessment.md`)

## 4. 実装判断の優先順位

1. 追跡可能性
2. 再現可能性
3. 物理解釈性
4. 数値的信頼性
5. CLI と artifact の明瞭性
6. 実行時間

## 5. 文書方針

- README と user manual は CLI workflow を主語に書く
- GUI 前提の説明は残さない
- 最小 YAML の例と出力 file の使い方を必ず記す
- 数式説明は model reference に集約し、操作説明は user manual に集約する


## 6. 追跡可能な出典と calibration

- calibration や基準 curve を更新するときは、どの公開 spec / 文献 / 教科書を anchor にしたかを文書へ明記する
- user manual には、代表 run の数表を載せ、reference case が code とどう結び付くかを書く
- model reference には、式だけでなく `src/cli.rs`、`src/simulator.rs`、`src/config.rs`、`config/reference_na_i4.yaml` の対応を書く
- torque curve を出す artifact は、`net torque` と `brake torque` を混同しない
