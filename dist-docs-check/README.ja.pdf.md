# ES

Language: [English](README.md) | 日本語

Rust + egui で実装した 4 ストロークエンジン物理シミュレータです。

この日本語版 README は、利用方法、GitHub からの配布導線、数理モデルの要点、代表的な検証結果を日本語でまとめた補助ドキュメントです。  
実装に対する一次資料は、英語版の [README.md](README.md) と `config/sim.yaml` のコメントです。

## 概要

本シミュレータは 0 次元の平均値エンジンモデルを RK2（midpoint）で積分し、次を可視化します。

- 回転数の「直近に完了した `N` サイクル + 現在進行中のサイクル」の重ね描き（`0..720 degCA`）
- ネットトルク履歴と外部負荷トルク履歴の「直近に完了した `N` サイクル + 現在進行中のサイクル」の重ね描き（`0..720 degCA`）
- 燃焼総出力と正味ブレーキ出力（`kW`, `HP`）
- 吸入空気質量の「直近に完了した `N` サイクル + 現在進行中のサイクル」の重ね描き（`mg/cyl`, `0..720 degCA`）
- 直近数サイクルの `p-V` 線図
- カムリフトプロファイル
- 熱効率指標

音は実録音アセットではなく、着火周波数、排気圧力脈動、排気系共鳴から合成します。

## Windows 11 バイナリと PDF の取得

GitHub Releases から、Windows x64 のポータブル zip をダウンロードできます。  
典型的なアセット名は次です。

```text
es_sim-windows-x64-v0.1.0.zip
```

Release には README の PDF も含めます。

```text
es_sim-readme-en.pdf
es_sim-readme-ja.pdf
```

ポータブル zip の中身は次です。

- `es_sim.exe`
- `config/sim.yaml`
- `README.md`
- `README.ja.md`
- `LICENSE`

同じ release には整合確認用の SHA-256 も含めます。

```text
es_sim-windows-x64-v0.1.0.sha256
```

GitHub Actions の成功 run から取得する場合、アーティファクト名は次です。

- `es_sim-windows-x64-snapshot`
- `es_sim-docs-pdf-snapshot`

## ソースから実行

```bash
cargo run --release
```

## ローカルで Windows 用 zip を作る

```powershell
cargo build --release
powershell -ExecutionPolicy Bypass -File .\scripts\package-windows.ps1 -Tag v0.1.0
```

生成物は `dist/` に出力されます。

## GitHub Automation

このリポジトリでは、配布に関係するワークフローを 2 本用意しています。

- `ci`
  push / pull request ごとに release build とテストを実行し、Windows スナップショット zip と英日 README PDF をアーティファクトとしてアップロードします。
- `release-windows`
  `v*` tag push または manual dispatch で、Windows zip、`.sha256`、英日 README PDF を GitHub Releases に公開します。

保守者向けの基本フローは次です。

1. 必要なら `Cargo.toml` の version を更新する
2. `v0.1.0` のような tag を push する
3. `release-windows` の完了を待つ
4. GitHub Releases から zip と PDF を取得する

```bash
git tag v0.1.0
git push origin v0.1.0
```

## キーボードショートカット

- `A`: Auto Start + Idle の ON/OFF
- `O`: Auto WOT Efficiency Search の ON/OFF
- `B`: 選択中の自動ベンチスイープの開始 / 停止
- `L`: `lambda=1` ベンチの開始 / 停止
- `S`: Starter の ON/OFF
- `I`: Spark の ON/OFF
- `F`: Fuel の ON/OFF
- `W` / `X`: Throttle の増減
- `Q`: 終了

## YAML 設定

設定ファイルは `config/sim.yaml` です。  
現在はマジックナンバーの大半を YAML に寄せています。

主要セクション:

- `environment.*`: 周囲圧力、吸気温度、排気温度、基準時間刻み
- `engine.*`: 幾何、慣性、摩擦、各制御体積、面積、目標アイドル回転数、最大回転数
- `cam.*`: センターライン、作動角、最大リフト
- `auto_control.*`: auto idle / auto WOT のしきい値、PIゲイン、探索条件
- `model.*`: 燃焼、流量、VE、熱損失、脈動、`p-V` 再構成、トルク平滑化などの低次元モデル係数
- `numerics.*`: RPM 連動時間刻みと誤差正規化
- `bench.*`: 自動ベンチのスイープ条件、混合気モード、積分設定
- `plot.*`: 履歴長と軸範囲
- `ui.*`: GUI 更新周期と各種描画寸法
- `audio.model.*`: 着火周波数ベース音源の係数

全項目の説明は `config/sim.yaml` のコメントを参照してください。英語版 README には、実装から読み出されるパラメータ一覧もあります。

## 可視化の見方

現在のダッシュボードは、燃焼解析専用計測器というより、平均値エンジンモデル、ドライバビリティ、制御の観点から読む表示と考えるのが適切です。

- 回転数、トルク、空気量履歴の横軸はサンプル番号です
- 実時間 `s` やクランク角の軸ではありません
- `p-V` 線図は可視化用の再構成モデルで、厳密な筒内圧 ODE の直接解ではありません
- ただし、燃料投入量、燃焼位相、熱損失の入力は本体モデルと一致させています
- カム図はバルブイベント時期とリフトの可視化です
- ベンチは連続 WOT スイープで、開始直後に粗い locked-cycle 近似から即時 preview 曲線を描き、その後に GUI のリアルタイム制約から切り離した積分でトルク / 出力曲線を追い込みます

## 実装と対応する数理モデル

### 状態ベクトル

$$
\mathbf{x}
=
\begin{bmatrix}
\omega & \theta & p_{im} & p_{ir} & p_{em} & p_{er} & \dot m_{ir} & \dot m_{er} & \alpha_{th}
\end{bmatrix}^{\mathsf T}
$$

- $\omega$: クランク角速度 `rad/s`
- $\theta$: クランク角 `rad`
- $p_{im}$: 吸気プレナム圧 `Pa`
- $p_{ir}$: 吸気ランナー / ポート圧 `Pa`
- $p_{em}$: 排気コレクタ圧 `Pa`
- $p_{er}$: 排気ランナー / プライマリ圧 `Pa`
- $\dot m_{ir}$: 吸気ランナー流量状態 `kg/s`
- $\dot m_{er}$: 排気ランナー流量状態 `kg/s`
- $\alpha_{th}$: 実効スロットル開度 `-`

### 入力

$$
\mathbf{u}
=
\begin{bmatrix}
\alpha_{cmd} & u_{load} & u_{st} & u_{spk} & u_f & \Delta\theta_{ign} & \Delta\theta_{VVT,I} & \Delta\theta_{VVT,E}
\end{bmatrix}^{\mathsf T}
$$

### 連続時間 ODE

$$
\dot \omega = \frac{\tau_{comb}+\tau_{start}-\tau_{fric}-\tau_{pump}-\tau_{load}}{J}
$$

$$
\dot \theta = \omega
$$

$$
\dot p_{im} = \frac{R_{air}T_{im}}{V_{im}}\left(\dot m_{th}-\dot m_{ir}\right)
$$

$$
\dot p_{ir} = \frac{R_{air}T_{im}}{V_{ir}}\left(\dot m_{ir}-\dot m_{cyl}\right)
$$

$$
\dot p_{em} = \frac{R_{air}T_{exh,eff}}{V_{em}}\left(\dot m_{er}-\dot m_{tail}\right)
$$

$$
\dot p_{er} = \frac{R_{air}T_{exh,eff}}{V_{er}}\left(\dot m_{exh,in}-\dot m_{er}\right)
$$

$$
\frac{d\dot m_{ir}}{dt} = \frac{p_{im}-p_{ir}}{L_{ir}} - d_{ir}\dot m_{ir}
$$

$$
\frac{d\dot m_{er}}{dt} = \frac{p_{er}-p_{em}}{L_{er}} - d_{er}\dot m_{er}
$$

$$
\dot \alpha_{th} = \frac{\alpha_{cmd}-\alpha_{th}}{\tau_{th}}
$$

### 走行中の定常状態の解釈

運転中のエンジンは、通常の意味では full-state の静的平衡点ではありません。  
理由は

$$
\dot \theta = \omega
$$

であり、さらに吸排気・燃焼の関係式がクランク角に対する周期関数だからです。

したがって本モデルでの「定常運転」は、固定位相断面上の周期定常として定義します。  
位相座標を除いた縮約状態

$$
\mathbf{x}_r
=
\begin{bmatrix}
\omega & p_{im} & p_{ir} & p_{em} & p_{er} & \dot m_{ir} & \dot m_{er} & \alpha_{th}
\end{bmatrix}^{\mathsf T}
$$

に対し、同じ $\theta=\theta_s$ で 720 degCA ごとにサンプリングした写像

$$
\mathbf{x}_{r,k+1} = \mathcal{P}\!\left(\mathbf{x}_{r,k}\right)
$$

が

$$
\mathbf{x}_r^* = \mathcal{P}\!\left(\mathbf{x}_r^*\right)
$$

を満たすとき、周期定常とみなします。

同値な cycle-average の表現は

$$
\frac{\mathbf{x}_r(t+T_c)-\mathbf{x}_r(t)}{T_c}
=
\frac{1}{T_c}\int_t^{t+T_c}\dot{\mathbf{x}}_r(\tau)\,d\tau
=
\mathbf{0}
$$

です。

### 代表運転点での周期定常チェック

実装には `representative_operating_points_are_periodic_steady_states` という回帰テストがあり、制御入力を固定したまま十分に収束させた後、同じ `360 degCA` 断面に戻る縮約状態のドリフトを見ています。

| 運転点 | Throttle [-] | Load cmd [-] | Mean RPM [rpm] | 同位相断面誤差ノルム [-] | `Δrpm` [rpm/cycle] | 最大 `Δp` [kPa/cycle] | 最大 `Δm_runner` [g/s/cycle] |
|---|---:|---:|---:|---:|---:|---:|---:|
| Light throttle | 0.10 | 0.80 | 2075 | 0.135 | 0.170 | 0.159 | 0.022 |
| Mid throttle | 0.34 | 0.80 | 5060 | 0.709 | 2.488 | 0.852 | 0.380 |
| WOT | 1.00 | 0.80 | 6976 | 2.524 | 0.091 | 3.222 | 3.812 |

WOT でも同位相での `Δrpm` は平均回転数の `0.1%` 未満に収まっており、この低次元 ODE に対して周期定常として扱えることを確認しています。圧力ドリフトもおおむね `3.3 kPa/cycle` 以下です。

### 圧縮性流量と VE

スロットルとテールパイプは、準 1 次元の圧縮性オリフィス流量式を使います。

$$
\Pi=\mathrm{clamp}\!\left(\frac{p_d}{p_u},0,1\right),\quad
\Pi_*=\left(\frac{2}{\gamma+1}\right)^{\frac{\gamma}{\gamma-1}}
$$

$$
\dot m=
\begin{cases}
C_dA\frac{p_u}{\sqrt{T_u}}
\left(\frac{\gamma}{R}\right)^{1/2}
\left(\frac{2}{\gamma+1}\right)^{\frac{\gamma+1}{2(\gamma-1)}},
& \Pi\le \Pi_*\\[6pt]
C_dA\frac{p_u}{\sqrt{T_u}}
\left[
\frac{2\gamma}{R(\gamma-1)}
\left(\Pi^{2/\gamma}-\Pi^{(\gamma+1)/\gamma}\right)
\right]^{1/2},
& \Pi>\Pi_*
\end{cases}
$$

VE は、回転数依存の基本項、VVT 補正、スロットル補正、オーバーラップ補正、wave-action 補正を掛け合わせた低次元モデルです。

$$
\eta_v=\mathrm{clamp}\left(\eta_{rpm}\eta_{vvt}\eta_{th}g_{ov}g_{wave},\ \eta_{v,min},\eta_{v,max}\right)
$$

### 燃焼、トルク、熱損失

燃焼の総図示トルクは、気筒あたり燃料質量、LHV、Otto 効率ベース、点火位相損失を使って組み立てています。  
壁面熱損失は Woschni 型の単一領域モデルで `heat_loss_cycle_j` として扱い、表示用 `p-V` と有効排気温度の双方に反映します。

表示される正味トルクは概ね

$$
\tau_{net}=\tau_{comb}+\tau_{start}-\tau_{fric}-\tau_{pump}-\tau_{load}
$$

です。

### `p-V` 線図

GUI の `p-V` 線図は、同じ燃料投入量、燃焼開始角、燃焼期間、位相効率、熱損失を使って再構成した表示用モデルです。  
定性的なサイクル形状確認と図示仕事推定を目的としており、直接計測の筒内圧波形を完全に代替するものではありません。

### 音響モデル

音源は録音ではなく、4 ストロークの着火周波数

$$
f_{fire}=\frac{\mathrm{rpm}\,N_{cyl}}{120}
$$

を基本音高にし、排気圧、排気温度、管長からパルス源と共鳴器を駆動する形です。  
回転数 0 ではゲートにより実質無音になります。

## 熱効率指標

GUI には次を表示します。

- 理論 Otto 効率
- `p-V` から積分した図示熱効率
- IMEP
- 燃焼総出力
- 正味ブレーキ出力
- ブレーキ BMEP

現在の既定設定では、点火時期固定の定常スイープが `eta_indicated_max = 0.44` に張り付く領域があります。  
本シミュレータは 4 気筒固定です。`history_recent_cycles` は気筒数ではなく、クランク角プロットに残す履歴サイクル数を意味します。
そのため、英語版 README の定常効率表は「現行実装の回帰テスト用基準値」として扱っています。

## 実機との照合

既定較正は、現行の `2.0 L` 級自然吸気ガソリン量産機のうち、特に Toyota Dynamic Force と Mazda SKYACTIV-G の帯域に合わせています。

| 基準エンジン | 公称トルク | 公称出力 | 位置づけ |
|---|---:|---:|---|
| Toyota Corolla Hatchback 2.0 Dynamic Force | `205 Nm @ 4800 rpm` | `126 kW @ 6600 rpm` | 主基準 |
| Mazda SKYACTIV-G 2.0 | `194 Nm @ 4100 rpm` | `113 kW @ 6000 rpm` | 副基準 |
| Toyota 86 2.0 | `205 Nm @ 6600 rpm` | `147 kW @ 7000 rpm` | スポーツ寄り上限 |

現在の既定 bench 曲線は `198.9 Nm @ 4500 rpm`, `129.0 kW @ 6500 rpm` で、量産 2.0L NA の帯域には入りつつ、Toyota 86 のようなスポーツ寄り高出力側へは行き過ぎないようにしています。
Toyota / Mazda を主基準、Honda Civic 2.0 と Ford EcoSport 2.0 を広い量産帯の確認用、Toyota 86 を上限確認用として扱う方針です。

## パラメータ参照

詳細な全パラメータ表は次を参照してください。

- 英語版 [README.md](README.md) の `Parameter List (Exhaustive, with Units)`
- `config/sim.yaml` の各項目コメント

特に実機感に効く主要設定は次です。

| YAML key | 単位 | 例 | 役割 |
|---|---:|---:|---|
| `engine.bore_m` | m | 0.0805 | ボア |
| `engine.stroke_m` | m | 0.0976 | ストローク |
| `engine.compression_ratio` | - | 13.0 | 圧縮比 |
| `engine.intake_volume_m3` | m^3 | 0.0032 | 吸気プレナム体積 |
| `engine.exhaust_volume_m3` | m^3 | 0.0050 | 排気コレクタ体積 |
| `model.gas_path.*` | mixed | - | 2 容積ガスパスとオーバーラップ / ランナー動特性 |
| `model.wave_action.*` | mixed | - | 吸排気パルス群化とラム効果 / 掃気補正 |
| `model.heat_transfer.*` | mixed | - | 単一領域熱損失モデル |
| `bench.*` | mixed | - | WOT 連続スイープと混合気モード |
| `audio.model.*` | mixed | - | 着火周波数ベース音源 |

## 主なテスト

実装には次の種類のテストを入れています。

- 厳密解がある線形部分に対する RK2 精度確認
- 高回転域での時間刻み細分化に対する収束確認
- `p-V` 線図の直近サイクル再構成確認
- 回転数と着火周波数の対応確認
- 熱損失 / オーバーラップ / wave-action の符号と有界性確認
- WOT ベンチ曲線が 2.0L 級 NA の形におおむね似るかの確認
- 既定 geometry が Dynamic Force / SKYACTIV-G 系の `2.0 L` 帯に入るかの確認
- rich WOT ベンチが Corolla / Mazda の量産帯に収まり、Toyota 86 の上限を超えないことの確認
- 代表運転点が周期定常状態として振る舞うかの確認

## 出典

主な出典は英語版 README と同じです。

- DTU Orbit, Hendricks, *A Generic Mean Value Engine Model for Spark Ignition Engines*
- NASA Glenn, *Mass Flow Choking*
- NASA Glenn, *Ideal Otto Cycle*
- Toyota Dynamic Force / Mazda SKYACTIV-G / Toyota 86 の公式トルク・出力・主要諸元
- GitHub Docs の数式レンダリング仕様

詳細な URL は英語版 [README.md](README.md) の `Sources and Reference Anchors` を参照してください。
