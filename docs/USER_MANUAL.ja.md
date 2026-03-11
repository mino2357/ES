# ES Simulator User Manual

この文書は self-contained です。
現在の GUI 中心 simulator を build、run、操作、解釈するための manual をまとめています。

## 用語

- `GUI`: graphical user interface。画面操作の user interface
- `bench`: dynamometer workflow を模した automated speed-hold sweep
- `dyno`: bench で使う absorber と controller
- `lambda`: air-fuel equivalence ratio
- `VVT`: variable valve timing
- `FHD`: full high definition。一般に `1920x1080`
- `WQHD`: wide quad high definition。一般に `2560x1440`
- `vehicle-equivalent load`: vehicle mass と driveline parameter から作る road-load model

## 対象範囲

`ES Simulator` は、stylized な test-cell dashboard を持つ reduced-order inline-4 engine simulator です。
目的は、対話的な過渡計算、bench-style sweep、物理的に解釈しやすい可視化です。

次のものではありません。

- CFD
- full 1D gas-dynamics solver
- 認証用の emissions model
- 実機 ECU calibration data の代替

## Build と Run

desktop application の起動:

```bash
cargo run --release
```

自動テスト:

```bash
cargo test -- --nocapture
```

## 画面構成

GUI 全体は、compact な engine test cell を模した構成です。

### Header

header には次のような annunciator が並びます。

- `RUN`
- `FUEL`
- `SPARK`
- `STARTER`
- `AUTO IDLE`
- `WOT SEARCH`
- `BENCH`
- `ACCURACY`

`ACCURACY` は、wall-clock 同期を優先せず、1 frame ごとに一定量の simulated time を進める mode であることを示します。

### Operator Rack

左の rack には collapsible module が入っています。

- `Automation`: idle automation と WOT efficiency search
- `Bench Sequencer`: automated dyno sweep の操作
- `Actuator Deck`: throttle、load、starter、spark、fuel、ignition、VVT の manual command
- `Status Bus`: runtime status、load-model mode、solver mode
- `Sensor / State Bus`: reduced-order state と closure output

左 rack 自体は scrollable です。
優先度の低い module は collapse できるので、FHD 級の画面でも使いやすくしています。

### Operator Display

中央上段には次が表示されます。

- speed、torque、power、trapped air、intake pressure、indicated efficiency の digital readout
- RPM、MAP、lambda、BMEP、exhaust temperature、combustion power の gauge
- throttle、load command、ignition、VVT の linear meter

### Plot

中央から下段には次が表示されます。

- bench torque / power plot
- cylinder `p-V`
- cylinder `p-theta`
- RPM、torque、trapped air の cycle-history plot

縦方向に収まりきらない場合は、中央領域を scroll して追えます。

## 基本操作

### 手動の過渡運転

1. application を起動する
2. `Throttle cmd` で airflow を要求する
3. `Load cmd` で absorber または vehicle-equivalent load をかける
4. 必要に応じて `Starter`、`Spark`、`Fuel` を切り替える
5. `Ignition`、`VVT Intake`、`VVT Exhaust` を操作して応答を見る

### Load model の選択

`Actuator Deck` の `Load model` では次を選べます。

- `Brake map`: speed 依存の absorber torque surrogate
- `Vehicle eq.`: road load と reflected inertia を engine 側へ写した model

checked-in の既定値は `Vehicle eq.` です。
過渡応答を物理的に解釈しやすいためです。

### Bench sweep

`Bench Sequencer` は自動 speed-hold sweep を実行します。

1. bench mixture mode を選ぶ
2. `Run Bench` を押す
3. dyno controller が load を自動調整して各 target RPM を保持する
4. brake torque と brake power を集計して CSV に出力する

最新 CSV は次に出力されます。

- `dist/bench/bench-rich_charge_cooling-latest.csv`
- `dist/bench/bench-lambda_one-latest.csv`

## 設定ファイル

実行時設定は [../config/sim.yaml](../config/sim.yaml) です。
`src/config.rs` の `AppConfig` へ読み込まれ、`src/config/audit.rs` の plausibility audit で検査されます。

### 主な section

- `environment`: ambient pressure、temperature、base timestep
- `engine`: displacement geometry、inertia、idle target、manifold volume、runner dimension
- `cam`: valve event location と duration
- `control_defaults`: 初期 throttle、spark、fuel、load、calibration command
- `auto_control`: idle と WOT search の挙動
- `model`: combustion、flow、friction、internal EGR、load の closure
- `numerics`: timestep ceiling、floor、accuracy-target setting
- `ui`: window size、scroll behavior、wall-clock / accuracy-first の切り替え
- `plot`: plot history と `p-V` sampling
- `bench`: dyno sweep と absorber 設定

### Accuracy-first mode

`ui.sync_to_wall_clock` が stepping policy を決めます。

- `true`: wall clock 追従を試みる
- `false`: 1 frame ごとに一定量の physical time を積分する

checked-in の設定は `false` です。

`ui.simulated_time_per_frame_s` は 1 frame あたりに進めたい physical time を表します。
solver は engine speed が上がると、その内部でさらに小さい numerical step を選びます。

## Plot の読み方

### `p-V`

表示される `p-V` loop は reconstructed diagnostic view です。
reduced-order state と cycle closure から再構成しています。
そのため、次の相対比較には有効です。

- indicated work の増減
- combustion phasing の変化
- pumping loss の変化

一方で、実測 cylinder pressure をそのまま積分したものではありません。

### `p-theta`

`p-theta` は 4 気筒を `0..720 degCA` 上に重ね描きします。
主な用途は次です。

- cylinder 間の phase spacing の確認
- ignition や VVT による pressure peak 移動の確認
- load や charge condition の変化による pressure shape の比較

## この simulator の向いている用途

向いている用途:

- 物理的に解釈しやすい control で過渡傾向を見る
- load model の違いを比較する
- ignition や VVT の定性的効果を可視化する
- bench-style torque curve を作る

向いていない用途:

- production ECU の最終 calibration 数値を決めること
- 厳密な emissions 予測
- 実測校正なしで正確な cylinder pressure を再構成すること

## 関連文書

- [../README.ja.md](../README.ja.md): repository 全体の概要
- [MODEL_REFERENCE.ja.md](MODEL_REFERENCE.ja.md): 数式、closure、実装対応、出典
- [USER_MANUAL.md](USER_MANUAL.md): この manual の英語版
