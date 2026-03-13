# GUI 改修整理メモ

この文書は、現在の GUI 実装を把握し、今後の改修を進めやすくするための整理メモです。
目的は「見た目を変えること」ではなく、「構造を簡素にして、責務を追いやすくすること」です。

現状の確認対象は主に次です。

- `src/dashboard.rs`
- `src/dashboard/widgets.rs`
- `src/dashboard/theme.rs`
- `src/config.rs`
- `docs/USER_MANUAL.ja.md`

## 1. 現在の GUI の要約

現行 GUI は `eframe` / `egui` ベースの desktop dashboard です。
起動経路は次です。

1. `src/main.rs` から `es_sim::launch_desktop()` を呼ぶ
2. `src/lib.rs` で設定を読み込み、`dashboard::run_app()` へ渡す
3. `DashboardApp` が simulator と GUI state をまとめて保持する
4. `DashboardApp::update()` が毎 frame ごとに
   - shortcut 処理
   - simulation 進行
   - header / side panel / central panel の描画
   を行う

実装上の中心は `src/dashboard.rs` で、ここに次が同居しています。

- `DashboardApp` の state 管理
- simulation の frame 進行
- load-control 補助ロジック
- plot 用の前処理 helper
- panel / module 単位の描画
- engine motion schematic の custom drawing
- 一部の unit test

`src/dashboard/widgets.rs` は再利用 widget の置き場として機能していますが、画面全体の構成と表示ロジックの大半は `src/dashboard.rs` に集中しています。

## 2. 現在の画面構成

画面の大枠は次の 3 領域です。

### 2.1 Header

- title
- simulation status readout
- annunciator 群
  - `RUN`
  - `FUEL`
  - `SPARK`
  - `MOTOR`
  - `FIRING`
  - `LOAD CTRL`
  - `PWR LIM`
  - `ACCURACY`

### 2.2 Left Control Rack

- `Actuator Deck`
  - throttle
  - target RPM
  - load model
  - spark / fuel
  - ignition / VVT
- `Status Bus`
  - load model 状態
  - solver mode
  - required brake torque / power
  - machine mode
- `Sensor / State Bus`
  - reduced-order state
  - closure output
  - internal metric

左側は `SidePanel` で、内部は縦 scroll と collapsible module です。

### 2.3 Central Display

上から順に次を表示します。

1. `Operator Display`
   - digital readout
   - gauge
   - linear meter
2. `Cylinder p-V`
3. `Cylinder p-theta`
4. `Cycle Monitors`
   - RPM history
   - torque / load history
   - trapped air history
   - cam lift plot
5. `Engine Motion Schematic`

中央は `CentralPanel + ScrollArea` で、縦に積む構成です。

## 3. 現在の責務分布

現在は、`DashboardApp` が次の責務をまとめて持っています。

### 3.1 実行制御

- `advance_simulation()`
- `apply_load_input()`
- `apply_shortcuts()`
- `update_schematic_phase()`

### 3.2 画面レイアウト

- `render_header_panel()`
- `render_control_rack()`
- `render_console_overview()`
- `render_pressure_plots()`
- `render_cycle_monitors()`
- `render_engine_motion_schematic()`

### 3.3 表示用の派生計算

- brake power / absorber power の算出
- current stroke の判定
- history plot 用の line 分割
- plot の y-range 調整
- schematic 用の flame front / geometry 計算

### 3.4 共有 UI 部品

- `theme.rs`
  - 色
  - frame
  - global style
- `widgets.rs`
  - digital readout
  - gauge
  - linear meter
  - annunciator
  - section label

## 4. 改修しづらい点

現状は動作としてはまとまっていますが、改修しづらさは主に次の点にあります。

### 4.1 `src/dashboard.rs` への集中

`src/dashboard.rs` は 2200 行超で、GUI のほぼ全責務が 1 ファイルに集まっています。
「どこを直せばよいか」が分かっても、「どこまで影響するか」が追いにくい構造です。

### 4.2 state 更新と描画が同じ場所にある

simulation の進行、input の反映、表示用の派生値計算、panel 描画が同じ `DashboardApp` にあります。
そのため、描画だけを差し替えたい場合でも runtime state の流れを一緒に読む必要があります。

### 4.3 panel が simulator 内部へ直接深く触る

多くの描画関数が `self.sim` や `self.latest` を直接参照し、表示用の値もその場で組み立てています。
このため、panel 単位で独立させにくく、テストや差し替えもしづらくなります。

### 4.4 複雑な表示ロジックが局所化されていない

次のようなロジックは、それぞれ独立したモジュールに切り出したい規模です。

- history plot の整形
- `p-V` / `p-theta` 表示データの準備
- engine motion schematic の custom drawing

現状は helper 関数としては分かれていても、ファイル単位では混ざっています。

### 4.5 `UiConfig` の責務が広い

`UiConfig` には次が同居しています。

- simulation frame 進行に関わる設定
- input range
- plot 見た目
- repaint rate
- window size

「runtime policy」と「layout / rendering policy」が混ざっているため、設定変更の意図が追いづらいです。

### 4.6 繰り返しのレイアウト記述が多い

card の列数計算、`match idx` による widget 群の生成、plot の共通設定などが各所に散っています。
大きな不具合ではありませんが、項目追加や並び替えのたびに編集範囲が広がりやすいです。

## 5. 改修のために目指す簡素な構造

過剰な abstraction は増やさず、まずは次の 4 層に分けるのが最も素直です。

1. `app`
   - frame 更新
   - panel の並び順
   - `eframe::App` 実装
2. `state`
   - simulation 進行
   - operator input の反映
   - current state の保持
3. `panels`
   - header
   - control rack
   - overview
   - plots
   - schematic
4. `widgets`
   - gauge
   - readout
   - meter
   - small reusable view parts

この分け方なら、現在の GUI の複雑さに対して十分で、message bus や過度な UI framework 風 abstraction を追加せずに済みます。

## 6. 提案するファイル構成

段階的に次の形へ寄せるのが分かりやすいです。

```text
src/
  dashboard/
    mod.rs
    app.rs
    state.rs
    view_model.rs
    theme.rs
    widgets.rs
    panels/
      mod.rs
      header.rs
      control_rack.rs
      overview.rs
      pressure_plots.rs
      cycle_monitors.rs
      schematic.rs
```

### 6.1 `app.rs`

- `DashboardApp`
- `eframe::App` 実装
- panel の表示順と top-level layout だけを持つ

ここでは「何を表示するか」は決めますが、「どう計算するか」は持たせません。

### 6.2 `state.rs`

- `Simulator`
- `latest observation`
- dt 制御
- shortcut 処理
- load-control 補助
- schematic animation phase

`DashboardApp` から simulation 管理責務を外し、更新系の入口をここに寄せます。

### 6.3 `view_model.rs`

- panel が読みやすい形に整形した snapshot
- display 用の文字列や数値
- 画面で何度も再計算している派生値

重要なのは、panel 側が `Simulator` の細部を直接読まなくてよい状態にすることです。
「表示に必要な情報」を 1 回まとめて作ると、描画関数がかなり短くなります。

### 6.4 `panels/*`

panel 単位で責務を分けます。

- `header.rs`: header と annunciator 群
- `control_rack.rs`: operator input と status module
- `overview.rs`: digital readout / gauge / meter
- `pressure_plots.rs`: `p-V` / `p-theta`
- `cycle_monitors.rs`: history plot と cam plot
- `schematic.rs`: engine motion schematic

### 6.5 `widgets.rs`

現在の再利用 widget の置き場として維持します。
ただし panel 固有のものはここへ増やしすぎず、「複数 panel から使う最小部品」だけを残します。

## 7. 実装時の分離ルール

簡素さを保つため、次のルールで十分です。

- `app` は top-level layout だけを持つ
- `state` は mutable な runtime state を持つ
- `view_model` は read-only な display snapshot を持つ
- `panels` は snapshot を受けて描画する
- `widgets` は panel から使う最小部品だけを持つ

特に守りたいのは次の 3 点です。

- panel から `Simulator` の奥まで直接触らない
- 派生値の計算を panel 内へ散らさない
- custom drawing は専用 module へ閉じ込める

## 8. 最小リファクタリング手順

一度に全部変えず、次の順で進めるのが安全です。

### Step 1. ファイル分割だけ先に行う

- `src/dashboard.rs` を `src/dashboard/mod.rs` へ移す
- `render_*` 関数群を `panels/*` へ分ける
- 挙動は変えない

この段階では「責務を見える化する」ことが目的です。

### Step 2. `DashboardState` を切り出す

- simulation 更新
- shortcut
- load-control
- dt 管理

を `state.rs` に寄せます。

### Step 3. `DashboardViewModel` を導入する

- brake power
- status text
- gauge / readout 用数値
- plot 用の整形済み入力

をまとめ、panel 側の `self.sim.*` 直読みを減らします。

### Step 4. `schematic` を独立させる

もっとも長く、描画密度も高い部分なので、最後に module を分離します。
ここが分かれるだけでも `dashboard` 全体の見通しはかなり改善します。

### Step 5. 必要なら `UiConfig` を再分割する

最後に必要があれば、次のように整理します。

- `UiRuntimeConfig`
- `UiLayoutConfig`
- `UiInputConfig`

ただし、これは最初からやらなくてよいです。
まずは code structure を分ける方が効果が大きいです。

## 9. 変更後に期待できる効果

- panel 単位で修正箇所を見つけやすくなる
- simulation 更新と描画の影響範囲を分けて考えられる
- plot や schematic の独立改修がしやすくなる
- 表示値の算出元を追いやすくなる
- 将来の GUI 追加でも `dashboard.rs` 1 ファイルへ再集中しにくくなる

## 10. 今回の結論

現状 GUI は、画面としての整理はできていますが、コード構造は `DashboardApp` と `src/dashboard.rs` に集中しています。
改修しやすくするための第一歩は、見た目の再設計より先に、次の単位へ責務を分けることです。

- `app`
- `state`
- `view_model`
- `panels`
- `widgets`

この単位なら、現在の GUI の規模に対して十分に簡素で、なおかつ今後の追加にも耐えやすい構成になります。
