# ES User Manual

この文書は self-contained です。
現在の GUI 中心 simulator を build、run、操作、解釈するための manual をまとめています。

## 用語

- `GUI`: graphical user interface。画面操作の user interface
- `dyno`: dynamometer や absorber を指す一般語
- `lambda`: air-fuel equivalence ratio
- `VVT`: variable valve timing
- `FHD`: full high definition。一般に `1920x1080`
- `WQHD`: wide quad high definition。一般に `2560x1440`
- `vehicle-equivalent load`: vehicle mass と driveline parameter から作る road-load model

## 対象範囲

`ES` は、stylized な test-cell dashboard を持つ reduced-order inline-4 engine simulator です。
目的は、対話的な過渡計算、物理的に解釈しやすい load response の観察、可視化です。

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
- `MOTOR`
- `FIRING`
- `LOAD CTRL`
- `PWR LIM`
- `ACCURACY`

`ACCURACY` は、wall-clock 同期を優先せず、1 frame ごとに一定量の simulated time を進める mode であることを示します。

### Operator Rack

左の rack には `Startup Numerical Fit` の進捗表示と manual control が入っています。

- 起動直後は throttle 候補ごとに ignition の `MBT` を探索し、requested RPM を保持する `Required brake torque` を数値的に解きます
- 適合が収束するまでは manual actuator edit は lock されます
- fit が `READY` になると、結果は `cache/startup_fit/` に保存され、同じ build identity と同じ YAML 設定なら次回起動時に再利用されます
- 現行 GUI では、収束後の `Runtime` タブで `Standard runtime` と `Actuator lab` を切り替えられます
- `Standard runtime` では `Driver demand` を主入力にし、throttle、ignition、VVT は auto baseline に従います
- `Actuator lab` では throttle、ignition、VVT を全部手動 override できます

### Operator Display

中央上段には次が表示されます。

- speed、torque、power、absorber torque、trapped air、intake pressure、absorber limit、indicated efficiency の digital readout
- RPM、MAP、lambda、BMEP、exhaust temperature、absorber torque、internal EGR の gauge
- throttle、`Eq RPM`、ignition、VVT の linear meter

### Plot

中央から下段には次が表示されます。

- `Map` view の `Map fill` toggle で、`BSFC + limit` と `Full BSFC` を切り替えられます
- cylinder `p-V`
- cylinder `p-theta`
- 図示トルク、正味トルク、正味出力、IMEP / indicated efficiency

これらは startup fit 中も常時表示されます。

縦方向に収まりきらない場合は、中央領域を scroll して追えます。

## 基本操作

### 手動の過渡運転

1. application を起動する
2. 起動直後は startup numerical fit が自動で走る
3. その間も `p-V`、`p-theta`、図示トルクを見ながら収束を確認する
4. fit が `READY` になったら、`Runtime` タブで `Driver demand` を使って standard runtime を動かせる
5. actuator を直接触りたい場合は `Actuator lab` に切り替え、`Throttle override`、`Ignition override`、`VVT` を操作する
6. 必要に応じて `Spark` と `Fuel` を切り替える

### Operator input と output

主要な operator input は次です。

- `Driver demand`

dashboard 側の load controller は `Driver demand -> torque request -> WOT torque curve inverse -> equilibrium rpm`
を使い、その平衡回転数へ向かうよう内部の `load_cmd` を自動調整します。
主要な output は次です。

- `Required brake torque`
- `Brake power`
- `RPM error`
- `Machine torque act / shaft est`

### 適合後に何が決まっているとみなすか

設計上は、fit 後に決まっているものは単なる 1 点の表示値ではありません。
少なくとも次を map 側の成果物として扱う想定です。

- request に対する nominal actuator set
- その actuator で成立する代表的な engine state
- torque、MAP、air charge、efficiency などの予測値
- 制約余裕と有効範囲

この考え方に立つと、標準の post-fit runtime は

- manual request
- map feedforward
- feedback trim

の順で状態が決まる構成が本筋です。

そのため、manual ignition や manual VVT は
標準 runtime の主入力というより、
教育用の override / compare 操作と解釈する方が整合的です。

議論の整理は
[CALIBRATION_MAP_RUNTIME_DECISION_MEMO.ja.md](CALIBRATION_MAP_RUNTIME_DECISION_MEMO.ja.md)
を参照してください。

### Load model の選択

`Actuator Deck` の `Load model` では次を選べます。

- `Brake dyno`: speed 依存の absorber torque surrogate に absorber power limit を加えたもの
- `Vehicle eq.`: road load と reflected inertia を engine 側へ写した model

checked-in の既定値は `Vehicle eq.` です。
過渡応答を物理的に解釈しやすいためです。

## 設定ファイル

実行時設定は [../config/sim.yaml](../config/sim.yaml) です。
`src/config.rs` の `AppConfig` へ読み込まれ、`src/config/audit.rs` の plausibility audit で検査されます。

startup fit artifact は [../cache/startup_fit](../cache/startup_fit) に保存されます。
再利用条件は「同じ build identity」と「同じ生 YAML text」です。
startup fit 自体は UI frame loop と切り離した background worker として走ります。
fit 用 numerics は runtime より粗く、既定 contract は `10 min` cap、release fit は `WOT` 固定の `16` candidates、各 candidate 最大 `6` cycles、`1200` RKF steps / cycle です。
release fit の後には `1000..7000 rpm` の `13` 点で WOT torque curve を構築し、post-fit runtime は `Driver demand -> torque request -> WOT torque curve inverse -> equilibrium rpm` で状態を決めます。

### 主な section

- `environment`: ambient pressure、temperature、base timestep
- `engine`: displacement geometry、inertia、default target RPM、manifold volume、runner dimension
- `cam`: valve event location と duration
- `control_defaults`: 初期 throttle、spark、fuel、load、calibration command
- `model`: combustion、flow、friction、internal EGR、load の closure
- `numerics`: timestep ceiling、floor、accuracy-target setting
- `ui`: window size、scroll behavior、wall-clock / accuracy-first の切り替え
- `plot`: plot history と `p-V` sampling

### Accuracy-first mode

`ui.sync_to_wall_clock` が stepping policy を決めます。

- `true`: wall clock 追従を試みる
- `false`: 1 frame ごとに一定量の physical time を積分する

checked-in の設定は `false` です。

`ui.simulated_time_per_frame_s` は 1 frame あたりに進めたい physical time を表します。
solver は engine speed が上がると、その内部でさらに小さい numerical step を選びます。

GUI の操作性を優先するため、dashboard は 1 frame あたりの simulation wall-clock 予算を持っています。
そのため、重い条件では 1 frame で要求した simulated time を全部消化せず、次 frame へ backlog することがあります。
また `Pressure` view を開いていない間は `p-V` 系の診断 sampling を background density に落として、slider や tab 操作を受け付けやすくしています。

## Plot の読み方

### `p-V`

表示される `p-V` loop は display 用の single-zone cylinder pressure solve です。
閉サイクル区間では Wiebe の burn fraction と cycle heat-loss estimate から `dp/dtheta` を積分し、
吸気・排気 stroke は boundary pressure へ滑らかにつないでいます。
そのため、次の相対比較には有効です。

- indicated work の増減
- combustion phasing の変化
- pumping loss の変化

combustion が有効なときは ignition、`SOC/EOC`、`CA10/50/90` の marker も重ねて表示します。

`Pressure` view を開いている間は、この表示用 diagnostic を full density で更新します。
他の view では操作性優先のため background density へ落ちるので、`IMEP / eta_i` は相対的に粗い更新になります。

一方で、これは main engine state ODE そのものではなく、実測 cylinder pressure でもありません。

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
- `Brake dyno` と `Vehicle eq.` の負荷応答を比較する

向いていない用途:

- production ECU の最終 calibration 数値を決めること
- 厳密な emissions 予測
- 実測校正なしで正確な cylinder pressure を再構成すること

## 関連文書

- [../README.ja.md](../README.ja.md): repository 全体の概要
- [MODEL_REFERENCE.ja.md](MODEL_REFERENCE.ja.md): 数式、closure、実装対応、出典
- [SURROGATE_GUIDE.ja.md](SURROGATE_GUIDE.ja.md): この simulator で `surrogate` をどういう意味で使うかの共通基準
- [STARTUP_FIT_OPTIMIZATION_FORMULATION.ja.md](STARTUP_FIT_OPTIMIZATION_FORMULATION.ja.md): startup fit を最適化問題として見るための定式化メモ
- [STARTUP_FIT_MBT_DISCUSSION.ja.md](STARTUP_FIT_MBT_DISCUSSION.ja.md): MBT 基準と adaptive 探索の議論メモ
- [STARTUP_FIT_DECISION_MEMO.ja.md](STARTUP_FIT_DECISION_MEMO.ja.md): startup fit の合意点、保留点、選択肢をまとめた意思決定メモ
- [STARTUP_FIT_PREIMPLEMENTATION_SPEC.ja.md](STARTUP_FIT_PREIMPLEMENTATION_SPEC.ja.md): 実装開始前に固定した startup fit の仕様書
- [CALIBRATION_MAP_RUNTIME_DECISION_MEMO.ja.md](CALIBRATION_MAP_RUNTIME_DECISION_MEMO.ja.md): 適合後の map runtime、manual control、教育用 override の整理メモ
- [POST_FIT_RUNTIME_FORMULATION.ja.md](POST_FIT_RUNTIME_FORMULATION.ja.md): post-fit runtime が `Driver demand`、torque request、baseline actuator、speed-hold trim をどう計算しているかの数式メモ
- [USER_MANUAL.md](USER_MANUAL.md): この manual の英語版
