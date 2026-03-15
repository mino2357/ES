# ES

`ES` は、EDM 風の操作盤を持つ inline-4 engine simulator です。
主眼は、GUI 上で reduced-order の過渡 engine model を動かし、物理的に解釈しやすい load response と `p-V` / `p-theta` 可視化を一体で扱うことにあります。

この repository で現在サポートしている製品経路は 1 つだけです。

- desktop GUI simulation
- reduced-order transient engine model
- physically interpretable external-load modeling
- operator 側の `Driver demand` 入力と、教育用 `Actuator lab` での manual throttle / ignition / VVT override
- `p-V` と `p-theta` の可視化

別製品としての offline solver はありません。
ただし startup fit 自体は、UI frame loop と切り離した background worker / headless runner で実行します。
audio synthesis path もありません。

## 用語

- `ODE`: ordinary differential equation。常微分方程式
- `VVT`: variable valve timing。可変 valve timing
- `lambda`: air-fuel equivalence ratio。`lambda = 1.0` は stoichiometric fueling を意味する
- `internal EGR`: valve overlap、backflow、不完全 scavenging によって cylinder に残る residual burned gas
- `p-V`: cylinder pressure と normalized cylinder volume の関係
- `p-theta`: cylinder pressure と crank angle の関係。表示範囲は `0..720 degCA`
- `dyno`: dynamometer や absorber を指す一般語。このリポジトリの現行 GUI では automated sweep ではなく手動の external-load 制御を使います
- `vehicle-equivalent load`: vehicle mass、tire radius、gear ratio、rolling resistance、drag、grade を使って engine 側へ反映する road-load model

## ドキュメント構成

入口はこの README にして、詳細は用途ごとに分離しています。

- [docs/USER_MANUAL.md](docs/USER_MANUAL.md): 英語版 user manual。build、run、dashboard 操作、設定の使い方
- [docs/MODEL_REFERENCE.md](docs/MODEL_REFERENCE.md): 英語版 model reference。状態方程式、closure、実装対応、制約、出典
- [docs/IMPLEMENTATION_DIRECTION.ja.md](docs/IMPLEMENTATION_DIRECTION.ja.md): 日本語版の実装方針書。今後の設計・実装判断、段階的な開発順、知財・秘匿配慮をまとめた既定指針
- [docs/SURROGATE_GUIDE.ja.md](docs/SURROGATE_GUIDE.ja.md): 日本語版の用語整理メモ。`surrogate`、`closure`、`map`、display-model の違いと、現行 surrogate の代表式
- [docs/CALIBRATION_MAP_RUNTIME_DECISION_MEMO.ja.md](docs/CALIBRATION_MAP_RUNTIME_DECISION_MEMO.ja.md): 日本語版の意思決定メモ。適合で何を決めるか、適合後に何を manual input として残すか、map runtime と教育用 override をどう分けるかを整理したもの
- [docs/POST_FIT_RUNTIME_FORMULATION.ja.md](docs/POST_FIT_RUNTIME_FORMULATION.ja.md): 日本語版の数式メモ。post-fit runtime の `Driver demand -> torque request -> baseline actuator -> load trim` を現行実装に沿って整理したもの
- [docs/GUI_REFACTOR_PLAN.ja.md](docs/GUI_REFACTOR_PLAN.ja.md): 日本語版の GUI 改修整理メモ。現状の画面構成、責務の偏り、簡素な再構成案をまとめたもの
- [docs/USER_MANUAL.ja.md](docs/USER_MANUAL.ja.md): 日本語版 user manual
- [docs/MODEL_REFERENCE.ja.md](docs/MODEL_REFERENCE.ja.md): 日本語版 model reference。ODE system 全体のまとめ、closure、実装対応、制約、出典
- [ENGINE_MODEL_WORKLOG.md](ENGINE_MODEL_WORKLOG.md): 作業ログ

各文書は、それ単体で読んでも意味が閉じるように書いています。
複雑な数式や出典は README に抱え込まず、詳細文書へ切り出しています。
詳細文書側からも、この README と相互に辿れるようにリンクを付けています。

## 方針

現在の方針は accuracy-first の GUI simulation です。

実装の中長期方針、適合 workflow、map runtime、知財・秘匿配慮を含む既定ルールは
[docs/IMPLEMENTATION_DIRECTION.ja.md](docs/IMPLEMENTATION_DIRECTION.ja.md) にまとめています。

- wall clock 同期よりも transient fidelity を優先する
- 描画は lockstep realtime を強制せず、計算できた最新状態を表示する
- 負荷 model は、抽象 brake map だけでなく physical に解釈しやすい形を優先する
- 数理モデルの説明は、出典と実装の両方に結び付ける

## Build と Run

Desktop:

```bash
cargo run --release
```

テスト:

```bash
cargo test -- --nocapture
```

## 設定ファイルの要約

実行時設定は [config/sim.yaml](config/sim.yaml) にあります。
これは `src/config.rs` の `AppConfig` に読み込まれ、`src/config/audit.rs` の plausibility audit で検査されます。

主な section は次です。

- `environment`: ambient boundary condition と base timestep
- `engine`: geometry、inertia、manifold volume
- `cam`: valve event geometry と VVT-sensitive input
- `control_defaults`: 初期 operator command
- `model`: combustion、flow、heat transfer、fuel、internal EGR、load の closure
- `numerics`: timestep policy と accuracy 設定
- `ui`: window、plot、dashboard の挙動
- `plot`: history サイズと plot sampling
checked-in の `sim.yaml` は、

- `model.external_load.mode: vehicle_equivalent`
- `ui.sync_to_wall_clock: false`

を使います。
つまり既定では、vehicle-like な external load を持つ accuracy-first transient simulation です。

startup fit が `READY` になると、その結果は `cache/startup_fit/` に YAML artifact として保存されます。
次回起動時に build identity と入力 YAML text が一致すれば、その artifact を読み込んで重い fit を再実行しません。

startup fit の既定 contract は次です。

- wall-clock 合格条件: `10 min` 以内
- release fit 候補数: `WOT` 固定 x (`10` coarse ignition + `6` local-refine ignition) = `16` candidates
- post-fit WOT torque curve: `1000..7000 rpm` の `13` 点を headless sweep して構築
- 1 候補あたりの上限: `6` cycles, `1200` RKF steps / cycle
- fit 専用 numerics: `accuracy_target_deg_per_step = 4.5 deg`, `accuracy_dt_max_s = 2.5 ms`, `rpm_link_dt_min_floor_s = 0.1 ms`

したがって release fit 本体の worst-case accepted RKF step budget は `16 x 6 x 1200 = 115,200` steps です。
post-fit runtime では throttle baseline は基本 `WOT` に固定し、`Driver demand -> torque request -> WOT torque curve inverse -> equilibrium rpm` の経路で状態を決めます。

## 実装の入口

主な実装ファイルは次です。

- `src/simulator.rs`: reduced-order solver、display reconstruction、load helper
- `src/dashboard.rs`: EDM 風 dashboard、operator control、plot、layout
- `src/config.rs`: 設定 schema と default
- `src/config/audit.rs`: physical / numerical / UI parameter の plausibility audit

詳細な数理モデルと関数単位の対応表は [docs/MODEL_REFERENCE.ja.md](docs/MODEL_REFERENCE.ja.md) を参照してください。
