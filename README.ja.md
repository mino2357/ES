# ES Simulator

`ES Simulator` は、EDM 風の操作盤を持つ inline-4 engine simulator です。
主眼は、GUI 上で reduced-order の過渡 engine model を動かし、物理的に解釈しやすい load response と `p-V` / `p-theta` 可視化を一体で扱うことにあります。

この repository で現在サポートしている製品経路は 1 つだけです。

- desktop GUI simulation
- reduced-order transient engine model
- physically interpretable external-load modeling
- operator 側の `Throttle cmd` と `Target RPM` 入力、それに対する brake torque / power 表示
- `p-V` と `p-theta` の可視化

別系統の offline solver はありません。
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

## 実装の入口

主な実装ファイルは次です。

- `src/simulator.rs`: reduced-order solver、display reconstruction、load helper
- `src/dashboard.rs`: EDM 風 dashboard、operator control、plot、layout
- `src/config.rs`: 設定 schema と default
- `src/config/audit.rs`: physical / numerical / UI parameter の plausibility audit

詳細な数理モデルと関数単位の対応表は [docs/MODEL_REFERENCE.ja.md](docs/MODEL_REFERENCE.ja.md) を参照してください。
