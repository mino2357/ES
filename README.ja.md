# ES

Language: [English](README.md) | 日本語

Rust で実装した固定 4 気筒・4 ストロークのエンジンシミュレータです。  
このリポジトリには、目的の異なる 2 つの実行経路があります。

| 実行経路 | 目的 | 入力 | 出力 | 実装 |
| --- | --- | --- | --- | --- |
| `es_sim` | リアルタイム GUI 可視化 | `config/sim.yaml` | 画面表示、合成音、ベンチ曲線 | `src/main.rs`, `src/dashboard.rs`, `src/simulator.rs`, `src/audio.rs` |
| `es_cli` / `es_hp` | 依存ゼロの headless / offline 高精度 CLI | `config/high_precision.yaml` | `torque_curve.csv`, `torque_summary.csv`, `pv_<rpm>rpm.csv`, report text | `src/bin/es_cli.rs`, `src/bin/es_hp.rs`, `src/hp/*` |

英語版 README は詳細な長文版です。  
日本語版では、実装と直接対応づけながら「何が動くか」「何を仮定しているか」「どこまで物理と言えるか」を追いやすい形にまとめます。

## 用語と表記

この README は、この文書だけで読めるように書きます。  
日本語で定着した訳語がないか、訳すと意味が狭まる語は English のまま使い、直後で意味を定義します。

- `realtime GUI path`
  `es_sim` の実行経路です。リアルタイム表示と音声合成を伴う対話的な経路を指します。
- `GUI`
  graphical user interface です。
- `headless CLI path`
  `es_cli` / `es_hp` の実行経路です。GUI や音声依存を持たず、CSV や text report を出力する非対話的な経路を指します。
- `CLI`
  command-line interface です。
- `compatibility alias`
  主バイナリ名とは別に残している互換名です。このリポジトリでは `es_hp` が `es_cli` と同じ実装を呼びます。
- `offline`
  wall-clock のリアルタイム制約を持たない実行形態を指します。headless path は headless であると同時に offline です。
- `plausibility audit`
  設定ファイル読込後に行う range / sanity check です。値が parse できても、実機として不自然なら reject します。
- `reduced-order mean-value model`
  状態数を抑えた集約モデルです。吸排気系、燃焼、トルクの平均的・代表的な挙動を保持しますが、CFD や full 1D gas dynamics code そのものではありません。
- `single-zone`
  気筒内を空間的に複数領域へ分けず、1 つの代表熱力学状態で扱う cylinder model を指します。
- `ODE`
  ordinary differential equation です。常微分方程式を指します。
- `RK3` / `RK4`
  explicit 3 次 / 4 次 Runge-Kutta 積分公式です。
- `PI controller`
  proportional-integral controller です。
- `bench`
  GUI 内の自動トルクカーブ計測機能です。物理的な試験設備そのものではなく、software 上の test-bench mode を意味します。
- `dyno`
  `bench` 中で回転数保持と brake torque 計測を担う absorber / load-holding surrogate を指します。
- `sweep`
  throttle や mixture 条件を固定したまま、RPM を掃引して torque / power curve を取る一連の operating point 列です。
- `lambda`
  空燃比を stoichiometric air-fuel ratio で正規化した air-fuel equivalence ratio です。`lambda < 1` は rich、`lambda = 1` は stoichiometric、`lambda > 1` は lean です。
- `YAML`
  設定入力に使う human-readable text format です。
- `CSV`
  計算結果出力に使う comma-separated values format です。
- `VVT`
  variable valve timing です。
- `EGR`
  exhaust gas recirculation です。このリポジトリで `internal EGR` と書く場合は、外部 EGR ループではなく overlap と backflow によって残る residual gas を指します。
- `SI`
  spark ignition です。
- `WOT`
  wide-open throttle です。
- `NA`
  naturally aspirated です。過給のないエンジンを指します。
- `OEM`
  original equipment manufacturer です。この README では、較正比較に使うメーカー公表スペックを指します。
- `plenum`
  intake runner の上流にある共通吸気容積です。
- `runner`
  共通容積と cylinder-side boundary を結ぶ intake / exhaust duct です。
- `collector`
  exhaust runner / header primary の下流にある共通排気容積です。
- `degCA`
  crank-angle degree です。4 ストローク 1 cycle を `0 .. 720` で表します。
- `TDC` / `BDC`
  top dead center / bottom dead center です。
- `BTDC`
  before top dead center です。
- `VE`
  volumetric efficiency です。
- `IMEP` / `BMEP`
  indicated / brake mean effective pressure です。
- `BSFC`
  brake-specific fuel consumption です。
- `APK`
  Android application package です。

以下の節で略語を再展開しない場合は、この節の定義を優先します。

## 概要

### `es_sim` が行うこと

`es_sim` は、0 次元の reduced-order mean-value エンジンモデルをリアルタイム積分し、次を可視化します。

- 回転数履歴
- 正味トルク / 外部負荷トルク
- 吸入空気量
- 燃焼出力 / 正味出力
- `p-V` 線図
- `p-theta` 線図（4 気筒重ね描き、`0..720 degCA`）
- カムリフト
- ベンチのトルク / 出力曲線

音は実録音ではなく、クランク同期の排気イベント、排気圧力 / 流量、簡易な管反射、排気共鳴を使った合成音です。

### `es_cli` が行うこと

`es_cli` は、GUI や音声依存を持たない headless / offline solver です。`es_hp` は同じ実装を呼ぶ互換 alias です。  
入力は YAML、出力は CSV に限定し、リアルタイム性よりも crank-angle 基準の解析を優先します。

- `torque_curve.csv`
- `pv_<rpm>rpm.csv`

headless 高精度経路の詳細は [HIGH_PRECISION_HEADLESS_MODEL.md](HIGH_PRECISION_HEADLESS_MODEL.md) に、YAML 監査結果は [HIGH_PRECISION_PARAMETER_AUDIT.md](HIGH_PRECISION_PARAMETER_AUDIT.md) にまとめています。
公開スペックに対する較正ターゲットと現在の一致度は [ENGINE_REFERENCE_TARGETS.md](ENGINE_REFERENCE_TARGETS.md) にまとめています。

### 固定 4 気筒について

このリポジトリの GUI / headless ともに、現在の実装は固定 4 気筒です。  
README 中の「サイクル」は履歴表示しているエンジンサイクルを指し、気筒数を指しません。

`config/sim.yaml` は [src/config/audit.rs](src/config/audit.rs) で physical / practical range audit を通します。  
物理量は実機としてありうる範囲、audio / UI / numerics は runtime 上の実用範囲で検証します。

## ビルドと実行

### GUI 版

`Cargo.toml` では `gui` feature が default です。  
そのため通常の `cargo run` は GUI バイナリ `es_sim` を起動します。

```bash
cargo run --release
```

対応箇所:

- バイナリ定義: [Cargo.toml](Cargo.toml)
- エントリポイント: [src/main.rs](src/main.rs)
- 設定ロード: [src/config.rs](src/config.rs)
- GUI 描画: [src/dashboard.rs](src/dashboard.rs)
- エンジン状態更新: [src/simulator.rs](src/simulator.rs)
- 音声合成: [src/audio.rs](src/audio.rs)

### Android 実行

Android でも `es_sim` を動かせるように、GUI 側に `cdylib` と `android_main` を追加しています。  
実装の入口は [src/lib.rs](src/lib.rs) で、`eframe` の Android 起動は [src/dashboard.rs](src/dashboard.rs) にあります。

現状の方針:

- 画面と数理モデルは desktop GUI 版と同じ
- `config/sim.yaml` は APK に埋め込んで使う
- 端末ストレージ上の外部 YAML 読み込みはまだ未実装

手順は [ANDROID.md](ANDROID.md) にまとめています。  
最短実行コマンドは次です。

```bash
cargo apk run --lib --release --target aarch64-linux-android
```

### headless 高精度 CLI

headless solver `es_cli` は、`eframe`、`rodio`、`serde_yaml` などの GUI 依存を使わない別バイナリです。`es_hp` は互換 alias として残しています。

```bash
cargo run --no-default-features --bin es_cli -- validate config/high_precision.yaml
cargo run --no-default-features --bin es_cli -- sweep config/high_precision.yaml
cargo run --no-default-features --bin es_cli -- point config/high_precision.yaml --rpm 3500 --write-pv
```

対応箇所:

- feature 分離: [Cargo.toml](Cargo.toml)
- エントリポイント: [src/bin/es_cli.rs](src/bin/es_cli.rs), [src/bin/es_hp.rs](src/bin/es_hp.rs)
- YAML parser / validator: [src/hp/config.rs](src/hp/config.rs), [src/hp/yaml.rs](src/hp/yaml.rs)
- solver: [src/hp/model.rs](src/hp/model.rs)
- CLI / report: [src/hp/cli.rs](src/hp/cli.rs), [src/hp/report.rs](src/hp/report.rs)
- CSV 出力: [src/hp/csv.rs](src/hp/csv.rs)

## Windows 11 バイナリと Release 配布

GitHub Releases では zip ではなく、個別の release asset として配布する前提です。

主要アセット:

- `es_sim.exe`
- `sim.yaml`
- `es_sim-readme-en.pdf`
- `es_sim-readme-ja.pdf`

補助アセット:

- `LICENSE`
- `es_sim-windows-x64-vX.Y.Z.sha256`

実行手順:

1. 書き込み可能なフォルダを用意します。
2. `es_sim.exe` と `sim.yaml` を同じフォルダに置きます。
3. `es_sim.exe` を実行します。

設定探索の実装は [src/config.rs](src/config.rs) にあり、従来の `config/sim.yaml` に加え、`es_sim.exe` の横の `sim.yaml` も読みます。  
これは flat な release asset 配布をそのまま動かすためです。

## GitHub Actions

このリポジトリには、配布に関係する workflow が 2 本あります。

- [ci.yml](.github/workflows/ci.yml)
  Windows の build/test と snapshot artifact、README PDF artifact を生成します。
- [release-windows.yml](.github/workflows/release-windows.yml)
  `v*` タグまたは manual dispatch で Windows release asset と README PDF を GitHub Releases に公開します。

ローカルで release asset を揃える場合:

```powershell
cargo build --release
powershell -ExecutionPolicy Bypass -File .\scripts\prepare-release-assets.ps1 -Tag v0.1.0
```

生成物は `dist/release-assets/` に出力されます。

## キーボードショートカット

- `A`: Auto Start + Idle の ON/OFF
- `O`: Auto WOT Efficiency Search の ON/OFF
- `B`: 選択中の自動ベンチを開始 / 停止
- `L`: `lambda=1` ベンチを開始 / 停止
- `S`: Starter の ON/OFF
- `I`: Spark の ON/OFF
- `F`: Fuel の ON/OFF
- `W` / `X`: Throttle の増減
- `Q`: 終了

ショートカット処理は [src/dashboard.rs](src/dashboard.rs) にあります。

## YAML 設定

### GUI 用 `config/sim.yaml`

リアルタイム GUI 経路の設定は [config/sim.yaml](config/sim.yaml) です。  
`src/config.rs` の struct は基本的にこの YAML の section と対応しています。

主要セクション:

- `environment.*`: 周囲圧、吸排気温度、基準 `dt`
- `engine.*`: ボア / ストローク、圧縮比、慣性、容積、スロットル面積
- `cam.*`: 吸排気カム中心角、作用角、最大リフト
- `control_defaults.*`: 起動直後の操作入力
- `auto_control.*`: auto idle / auto WOT の制御係数
- `model.*`: 燃焼、流量、VE、波動、熱損失、`p-V` 表示モデル
- `numerics.*`: リアルタイム経路とベンチ経路の数値積分設定
- `bench.*`: ベンチ sweep、locked-RPM、dyno 速度保持 / 吸収器、混合気条件
- `plot.*`: 履歴長、表示レンジ
- `ui.*`: GUI の更新周期、スライダ範囲、表示寸法
- `audio.*`: 合成音の係数

### headless 用 `config/high_precision.yaml`

headless 高精度経路は [config/high_precision.yaml](config/high_precision.yaml) を使います。  
こちらは GUI 用 config と別物で、目的をかなり絞っています。

- `engine.*`: 幾何、境界条件、runner / tailpipe / valve の物理量
- `combustion.*`: 燃料、点火、燃焼期間、壁面熱伝達
- `sweep.*`: throttle、トルクカーブ用 RPM 点、`p-V` を出す RPM 点
- `numerics.*`: crank-angle 刻み、runner cell 数、warmup / sample cycle 数

headless YAML は [src/hp/config.rs](src/hp/config.rs) で strict schema を取っています。

- unknown key はエラー
- すべての項目に sanity range を適用
- `engine.cylinders` のような旧キーは受け付けない
- `wiebe_a` / `wiebe_m` のような trace-shaping 用の自由パラメータは YAML から外し、内部固定

この判断の根拠は [HIGH_PRECISION_PARAMETER_AUDIT.md](HIGH_PRECISION_PARAMETER_AUDIT.md) にあります。

## 可視化と解釈

### GUI で見えているもの

- 回転数、トルク、吸気量の履歴は `0..720 degCA` の固定 x 軸で表示します。
- `p-V` は表示用に再構成した cylinder pressure model です。
- `p-theta` は 4 気筒分を `0..720 degCA` 上に重ね描きします。
- ベンチは「即時プレビュー」と「予算付き sweep」の 2 層です。
- 実測 sweep 側は、角速度を直接固定するのではなく、bench 専用の dyno controller が吸収負荷を自動で増減して target RPM を保持します。
- そのためベンチ表示のトルクは残差の `net torque` ではなく、dyno が受けた `brake torque` として読むのが正しいです。
- ベンチ完了時には結果を `dist/bench/bench-rich_charge_cooling-latest.csv` か `dist/bench/bench-lambda_one-latest.csv` に自動出力します。

対応箇所:

- 履歴 plot / `p-V` / `p-theta`: [src/dashboard.rs](src/dashboard.rs)
- `p-theta` データ生成: [src/simulator.rs](src/simulator.rs)
- ベンチ即時プレビュー: [src/simulator.rs](src/simulator.rs)
- ベンチ進行表示: [src/dashboard.rs](src/dashboard.rs)

### 読み方の注意

- このモデルは、燃焼解析専用の multi-zone solver ではありません。
- `p-V` / `p-theta` は表示用の再構成であり、実測筒内圧そのものではありません。
- VVT、オーバーラップ、吸排気波動の効果は、現時点では主に定性的な指標として解釈すべきです。
- 実機のノック限界、排気音評価、CA10/50/90、apparent heat release を直接置き換えるものではありません。

## 数理モデルの要約

### 1. GUI リアルタイム経路

GUI の主モデルは、多気筒 1D gas dynamics や CFD ではなく、吸気 / 排気を plenum-runner の 2 容積 + runner 流量状態で表した reduced-order mean-value model です。  
`p-V` と `p-theta` は、同じ燃料量と燃焼位相入力から再構成した表示モデルであり、筒内状態 ODE を直接解いているわけではありません。

#### 状態ベクトル

```math
\mathbf{x}
=
\begin{bmatrix}
\omega & \theta & p_{im} & p_{ir} & p_{em} & p_{er} & \dot m_{ir} & \dot m_{er} & \alpha_{th}
\end{bmatrix}^{\mathsf T}
```

- $\omega$: クランク角速度
- $\theta$: クランク角
- $p_{im}$: 吸気 plenum 圧
- $p_{ir}$: 吸気 runner / port 圧
- $p_{em}$: 排気 collector 圧
- $p_{er}$: 排気 runner / primary 圧
- $\dot m_{ir}$: 吸気 runner 流量状態
- $\dot m_{er}$: 排気 runner 流量状態
- $\alpha_{th}$: 実効スロットル開度

#### 制御入力

```math
\mathbf{u}
=
\begin{bmatrix}
\alpha_{cmd} & u_{load} & u_{st} & u_{spk} & u_f & \Delta \theta_{ign} & \Delta \theta_{VVT,I} & \Delta \theta_{VVT,E}
\end{bmatrix}^{\mathsf T}
```

#### 連続時間 ODE

クランク系:

```math
\dot \omega
=
\frac{
\tau_{comb}
+ \tau_{start}
- \tau_{fric}
- \tau_{pump}
- \tau_{load}
}{J}
```

```math
\dot \theta = \omega
```

吸排気容積:

```math
\dot p_{im}
=
\frac{R_{air} T_{im}}{V_{im}}
\left(
\dot m_{th} - \dot m_{ir}
\right)
```

```math
\dot p_{ir}
=
\frac{R_{air} T_{im}}{V_{ir}}
\left(
\dot m_{ir} - \dot m_{cyl}
\right)
```

```math
\dot p_{em}
=
\frac{R_{air} T_{exh,eff}}{V_{em}}
\left(
\dot m_{er} - \dot m_{tail}
\right)
```

```math
\dot p_{er}
=
\frac{R_{air} T_{exh,eff}}{V_{er}}
\left(
\dot m_{exh,in} - \dot m_{er}
\right)
```

runner 流量状態:

```math
\frac{d \dot m_{ir}}{dt}
=
\frac{
\left(p_{im} - p_{ir}\right) - \Delta p_{loss,ir}
}{L_{ir}}
- d_{ir}\dot m_{ir}
```

```math
\frac{d \dot m_{er}}{dt}
=
\frac{
\left(p_{er} - p_{em}\right) - \Delta p_{loss,er}
}{L_{er}}
- d_{er}\dot m_{er}
```

スロットル遅れ:

```math
\dot \alpha_{th}
=
\frac{
\alpha_{cmd} - \alpha_{th}
}{\tau_{th}}
```

runner 損失は `f L / D + K` 形のダクト損失で近似します。

```math
\Delta p_{loss}
=
\operatorname{sgn}(\dot m)
\left(
f \frac{L}{D} + K
\right)
\frac{\dot m^2}{2 \rho A_{eff}^2}
```

圧力状態に入る流量 closure は、実装では次の形です。

```math
\dot m_{cyl}
=
\dot m_{cyl,mean}\,
\phi_I(\theta)\,
\psi_{I,wave}(\theta)
```

```math
\dot m_{exh,in}
=
\left(\dot m_{cyl,mean}+\dot m_f\right)\,
\phi_E(\theta)\,
\psi_{E,wave}(\theta)
```

ここで `\phi_I,\phi_E` はバルブイベント由来の pulse factor、`\psi_{I,wave},\psi_{E,wave}` は瞬時の wave-action multiplier です。

燃焼・熱損失・排気温度へ渡す charge state は、internal EGR を含めて次で近似します。

```math
x_{egr}
=
\operatorname{clamp}\!\left[
\chi_{ov}
\left(
x_0
+k_p\Delta p_{back}^{+}
+k_w s_{scav}^{-}
+k_r \dot m_{rev}^{+}
\right)
\right]
```

```math
c_{p,burn}
=
\operatorname{clamp}\!\left(
c_{p,ref}
+k_T\left(T_{res}-T_{ref}\right)
+k_{egr}x_{egr}
\right)
```

```math
\gamma_{mix}
=
\frac{c_{p,mix}}{c_{p,mix}-R_{air}}
```

```math
T_{charge}
=
\frac{m_{fresh} c_{p,f}T_f + m_r c_{p,burn}T_{res}}
{m_{fresh} c_{p,f}+m_r c_{p,burn}}
```

`x_{egr}` は overlap lift `\chi_{ov}`、排気側優勢の pressure head `\Delta p_{back}^{+}`、負の scavenging head `s_{scav}^{-}`、逆向き排気 runner 流量 `\dot m_{rev}^{+}` から作っています。

#### 補助モデル

- スロットル / テールパイプ流量は準 1 次元 compressible orifice 近似
- VE は baseline 回転依存項に、VVT、overlap、grouped wave / resonance 補正を乗算
- 燃料蒸発冷却、燃焼後ガスの `c_p / \gamma` 変化、overlap backflow 由来の internal EGR を charge state に反映
- 燃焼トルク、摩擦、ポンピング、スタータ、外部負荷は reduced-order closure
- 図示熱効率表示は再構成 `p-V` ループ面積から算出

#### 数値積分

通常のリアルタイム経路では、3 段 3 次の classical Kutta RK3 を使います。  
対応実装は [src/simulator.rs](src/simulator.rs) の `advance_state_rk3*` です。

```math
\mathbf{k}_1 = f(\mathbf{x}_n)
```

```math
\mathbf{k}_2 = f\!\left(\mathbf{x}_n + \frac{\Delta t}{2}\mathbf{k}_1\right)
```

```math
\mathbf{k}_3 = f\!\left(\mathbf{x}_n + \Delta t\left(-\mathbf{k}_1 + 2\mathbf{k}_2\right)\right)
```

```math
\mathbf{x}_{n+1}
=
\mathbf{x}_n
+
\frac{\Delta t}{6}
\left(
\mathbf{k}_1 + 4\mathbf{k}_2 + \mathbf{k}_3
\right)
```

リアルタイム GUI は起動時に 1 step あたりの wall-clock 時間を見積もり、余裕がある場合は固定 `dt` を優先し、余裕が不足する場合だけ RPM 連動 `dt` に落とします。  
この判定は [src/simulator.rs](src/simulator.rs) の `estimate_realtime_performance` にあります。

ベンチ経路では、trial step と 2 回の half step の差を見て step-doubling を行います。  
対応実装は [src/simulator.rs](src/simulator.rs) の `bench_adaptive_step_with_deg_per_step` です。

```math
\varepsilon_{bench}
=
\left\lVert
\mathbf{x}_{RK3}(\Delta t)
- \mathbf{x}_{RK3}\!\left(\frac{\Delta t}{2}\right)^{(2)}
\right\rVert_{norm}
```

### 2. headless 高精度経路

`es_cli` は、GUI の reduced-order path とは別に、single-zone cylinder と segmented runner line を持つ offline solver です。`es_hp` でも同じ solver を起動できます。

- 固定 4 気筒、180 deg 位相差
- slider-crank 幾何
- 吸気 plenum / 排気 collector
- intake / exhaust runner の pressure cell + face mass-flow state
- Wiebe 燃焼
- 壁面熱損失
- fixed-step RK4

対応実装:

- solver 本体: [src/hp/model.rs](src/hp/model.rs)
- 数学補助: [src/hp/math.rs](src/hp/math.rs)
- 設定検証: [src/hp/config.rs](src/hp/config.rs)

headless 側の式展開と前提は [HIGH_PRECISION_HEADLESS_MODEL.md](HIGH_PRECISION_HEADLESS_MODEL.md) に分離しています。  
GUI README にすべて重複させず、役割ごとに文書を分けています。

## テストと検証

### GUI / realtime 側

```bash
cargo test --release -- --nocapture
```

主な検証対象:

- YAML 読み込みと `sim.yaml` 探索
- RK3 の解析解一致テスト
- ベンチ積分の安定性と step refinement
- burned-gas 比熱と internal EGR の backflow 回帰
- `p-theta` のピーク位相が 4 気筒でずれること
- トルク / 出力の broad envelope が破綻していないこと

主な実装箇所:

- [src/config.rs](src/config.rs)
- [src/simulator.rs](src/simulator.rs)
- [src/dashboard.rs](src/dashboard.rs)

### headless / offline 側

```bash
cargo test --no-default-features --bin es_cli -- --nocapture
```

主な検証対象:

- YAML subset parser
- strict schema validation
- RK4 の解析解一致
- bidirectional orifice flow の符号
- equilibrium 近傍での有限導関数
- short sweep で正の brake torque が得られること

実装箇所:

- [src/hp/yaml.rs](src/hp/yaml.rs)
- [src/hp/config.rs](src/hp/config.rs)
- [src/hp/math.rs](src/hp/math.rs)
- [src/hp/model.rs](src/hp/model.rs)

## 実装対応表

README と実装の対応を追いやすくするため、主要責務をファイル単位で明示します。

- GUI エントリポイント: [src/main.rs](src/main.rs)
- GUI config 読み込みと `sim.yaml` 探索 / plausibility audit: [src/config.rs](src/config.rs), [src/config/audit.rs](src/config/audit.rs)
- GUI レイアウト、`p-V` / `p-theta` / ベンチ描画: [src/dashboard.rs](src/dashboard.rs)
- GUI エンジン状態方程式、RK3、bench step-doubling、preview curve: [src/simulator.rs](src/simulator.rs)
- 音声合成: [src/audio.rs](src/audio.rs)
- headless エントリポイント: [src/bin/es_cli.rs](src/bin/es_cli.rs), [src/bin/es_hp.rs](src/bin/es_hp.rs)
- headless YAML parser / validator: [src/hp/yaml.rs](src/hp/yaml.rs), [src/hp/config.rs](src/hp/config.rs)
- headless solver / CLI / CSV / report: [src/hp/model.rs](src/hp/model.rs), [src/hp/cli.rs](src/hp/cli.rs), [src/hp/csv.rs](src/hp/csv.rs), [src/hp/report.rs](src/hp/report.rs)
- headless 数理モデル文書: [HIGH_PRECISION_HEADLESS_MODEL.md](HIGH_PRECISION_HEADLESS_MODEL.md)
- headless パラメータ監査: [HIGH_PRECISION_PARAMETER_AUDIT.md](HIGH_PRECISION_PARAMETER_AUDIT.md)
- 作業ログ: [ENGINE_MODEL_WORKLOG.md](ENGINE_MODEL_WORKLOG.md)

## 解釈上の注意

- このリポジトリの主眼は「リアルタイム性をある程度保った reduced-order 物理モデル」と「依存ゼロの offline solver」の両立です。
- GUI 経路は、実機燃焼解析装置の代替ではありません。
- headless 経路は GUI より物理状態が細かい一方で、まだ full 1D gas dynamics code ではありません。
- 実機比較は calibration anchor として扱っており、モデル骨格の根拠は基本的に論文です。

## 参考論文と資料

モデルの骨格は、SI エンジン向け mean value engine model と manifold filling dynamics の文献を主な根拠にしています。

- E. Hendricks, S. C. Sorenson, "Mean Value Modelling of Spark Ignition Engines," SAE Technical Paper 900616, 1990. DOI: `10.4271/900616`
- E. Hendricks, S. C. Sorenson, "SI Engine Controls and Mean Value Engine Modelling," SAE Technical Paper 910258, 1991. DOI: `10.4271/910258`
- E. Hendricks, T. Vesterholm, "The Analysis of Mean Value SI Engine Models," SAE Technical Paper 920682, 1992. DOI: `10.4271/920682`
- E. Hendricks, A. Chevalier, M. Jensen, S. C. Sorenson, D. Trumpy, J. Asik, "Modelling of the Intake Manifold Filling Dynamics," SAE Technical Paper 960037, 1996. DOI: `10.4271/960037`
- O. Vogel, K. Roussopoulos, L. Guzzella, J. Czekaj, "Variable Valve Timing Implemented with a Secondary Valve on a Four Cylinder SI Engine," SAE Technical Paper 970335, 1997. DOI: `10.4271/970335`

補足:

- GUI 側の吸排気波動と VVT は、上記論文を踏まえた reduced-order 近似です。
- headless 側の cylinder / runner solver も、single-zone + finite-volume 的な offline path であり、完全な 1D commercial code の置き換えではありません。
- 実トルク / 実出力は、モデルの妥当性確認用アンカーとして別に扱っています。
