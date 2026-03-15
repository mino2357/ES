# Post-Fit Runtime の数理定式化メモ

この文書は、startup fit 完了後の `Standard runtime` と `Actuator lab` が
現行実装で何を計算しているかを、
数理モデルと物理の視点で整理するためのメモです。

ここで扱うのは、

- `Driver demand` をどう torque request に変えているか
- fit 結果をどう baseline actuator に変えているか
- speed-hold feedback がどこに掛かっているか
- `Standard runtime` と `Actuator lab` の違いは何か

です。

## 関連文書

- [CALIBRATION_MAP_RUNTIME_DECISION_MEMO.ja.md](CALIBRATION_MAP_RUNTIME_DECISION_MEMO.ja.md): 設計判断の整理
- [STARTUP_FIT_OPTIMIZATION_FORMULATION.ja.md](STARTUP_FIT_OPTIMIZATION_FORMULATION.ja.md): startup fit 側の定式化
- [MODEL_REFERENCE.ja.md](MODEL_REFERENCE.ja.md): reduced-order engine model 全体の ODE と closure
- [USER_MANUAL.ja.md](USER_MANUAL.ja.md): GUI 上の操作説明

## 1. 現行実装の主な対応箇所

- post-fit runtime の状態管理: [../src/dashboard/state.rs](../src/dashboard/state.rs)
- startup fit の release point / torque curve: [../src/dashboard/startup_fit.rs](../src/dashboard/startup_fit.rs)
- engine model と torque / MBT surrogate: [../src/simulator.rs](../src/simulator.rs)

## 2. 前提: engine 本体は同じ ODE を積分している

post-fit runtime でも、engine 本体で積分している状態方程式そのものは変わりません。

```math
\frac{d\mathbf{x}}{dt}
=
f(\mathbf{x}, \mathbf{u})
```

ここで状態は、現行 model reference と同じく

```math
\mathbf{x} =
\begin{bmatrix}
\omega &
\theta &
p_{im} &
p_{ir} &
p_{em} &
p_{er} &
\dot m_{ir} &
\dot m_{er} &
\alpha_{th}
\end{bmatrix}^{\mathsf T}
```

です。

重要なのは、
post-fit runtime が新しい engine model を使うのではなく、
同じ reduced-order ODE に対して
`どの入力 \mathbf{u} を与えるか`
を変えているだけだ、という点です。

## 3. torque の物理的な整理

runtime で見ている torque は、少なくとも次の 3 種に分ける必要があります。

### 3.1 Indicated torque

`p-V` loop 由来の図示仕事ベースの内部指標です。

```math
\tau_{ind}
=
\frac{N_{cyl}}{4\pi}\,\bar{W}_{ind}
```

これは cylinder 内部の仕事であり、
まだ friction や外部負荷を引いていません。

### 3.2 Required brake torque

requested speed を保持するために bench 側が受け持つべき torque です。

```math
\tau_{brake,req}
=
\tau_{comb} - \tau_{fric} - \tau_{pump}
```

startup fit が主に決めているのは、この量です。

### 3.3 Net torque

実際に shaft を加速させる残りの torque です。

```math
\tau_{net}
=
\tau_{comb} - \tau_{fric} - \tau_{pump} - \tau_{load}
```

target speed で安定しているときは、cycle-average として

```math
\bar{\tau}_{net} \approx 0
```

が目標になります。

## 4. startup fit が runtime へ渡すもの

現行の startup fit は、target speed `n^\star` に対して
throttle 候補ごとの `MBT` を調べ、
その throttle で成立する `Required brake torque` を評価します。

その結果、release point だけでなく、
次のような離散曲線を runtime へ渡せます。

```math
\mathcal{C}
=
\left\{
(\alpha_{th,j}, \tau_{brake,req,j})
\right\}_{j=1}^{N}
```

ここで

- `\alpha_{th,j}`: throttle 候補
- `\tau_{brake,req,j}`: その throttle の MBT 点で得られた required brake torque

です。

現行実装では、この curve は
「各 throttle bin に対する best result」
から作られます。

## 5. `Driver demand` は何を意味しているか

現行の `Driver demand` は、
physical throttle angle そのものではありません。

まず区間

```math
d \in [0, 1]
```

の無次元 demand として受け取り、
それを
`required brake torque` の request に写します。

### 5.1 torque curve の上下限

startup fit の best torque curve から、

```math
\tau_{\min} = \min_j \tau_{brake,req,j}, \qquad
\tau_{\max} = \max_j \tau_{brake,req,j}
```

を取ります。

### 5.2 demand から torque request への写像

現行実装では線形に

```math
\tau_{req}(d)
=
\tau_{\min} + d(\tau_{\max} - \tau_{\min})
```

としています。

したがって `Driver demand` は、
物理的には

- pedal 相当の抽象要求
- required brake torque の正規化ノブ

として解釈するのが適切です。

### 5.3 release point から demand を初期化する式

fit 完了直後は、selected release point の torque
` \tau_{rel} `
に対応する demand を逆写像で与えています。

```math
d_{rel}
=
\frac{\tau_{rel} - \tau_{\min}}{\tau_{\max} - \tau_{\min}}
```

ただし分母が 0 に近い場合は、実装上は `1.0` 側へ倒しています。

## 6. torque request から baseline throttle を作る

次に、request torque を
`best torque curve` 上の throttle へ逆補間します。

曲線を torque 昇順に並べ替えた

```math
(\tau_0, \alpha_0), (\tau_1, \alpha_1), \dots, (\tau_k, \alpha_k)
```

を用意し、

```math
\tau_i \le \tau_{req} \le \tau_{i+1}
```

を満たす区間で線形補間して

```math
\alpha_{th}^{base}
=
\alpha_i
+
\frac{\tau_{req} - \tau_i}{\tau_{i+1} - \tau_i}
(\alpha_{i+1} - \alpha_i)
```

とします。

範囲外では端点値をそのまま使います。

これは現時点では
「fit で得られた per-throttle MBT curve を inverse に読む」
という簡易 inverse map です。

## 7. baseline ignition はどう決めているか

ignition baseline は、現在の実装では
full map lookup ではなく、
既存の MBT surrogate を使って作っています。

まず throttle baseline から load surrogate を作ります。

```math
\hat{\ell}
=
\mathrm{clamp}(0.45 + 0.50\,\alpha_{th}^{base},\ \ell_{\min},\ \ell_{\max})
```

次に MBT surrogate

```math
\theta_{ign}^{base}
=
\mathrm{clamp}
\left(
\theta_0
+
k_n (n_e - n_{ref})
+
k_\ell (1 - \hat{\ell}),
\theta_{\min},
\theta_{\max}
\right)
```

で ignition baseline を決めています。

ここで

- `n_e`: 現在の target speed 近傍で使う回転数
- `\theta_0, k_n, k_\ell`: model 内の MBT surrogate parameter

です。

つまり ignition は、
現時点では
`fit curve から直接補間した map`
というより、
`MBT surrogate による軽量 feedforward`
として扱われています。

## 8. baseline VVT はどう決めているか

ここは重要な制約です。

現行の startup fit 一次段では
`VVT_i, VVT_e` を固定しており、
VVT を探索していません。

そのため post-fit runtime の baseline VVT は、
今の実装では schedule から再計算せず、
release control の値をそのまま使っています。

```math
VVT_i^{base} = VVT_i^{rel}, \qquad
VVT_e^{base} = VVT_e^{rel}
```

これは

- まだ VVT を含む map を持っていない
- fit と runtime の整合を壊したくない

ための暫定整理です。

将来 VVT を含む map を持てば、
ここは `VVT^{base}(n_e, \tau_{req})` へ置き換えられます。

## 9. speed-hold feedback はどこに掛かっているか

post-fit runtime の feedback は、
throttle や ignition ではなく、
bench load 側に掛かっています。

基準 torque request `\tau_{req}` に対して、
actual speed `n_e` と target speed `n^\star` の差から
load torque target を作ります。

```math
\tau_{load,target}
=
\max\left(
0,\,
\tau_{req} + K_n (n_e - n^\star)
\right)
```

ここで `K_n` は現行実装では定数 gain です。

速度が target より高ければ負荷を増やし、
低ければ負荷を減らす方向に働きます。

その後、bench / absorber model の逆写像

```math
u_{load}
=
g_{load}^{-1}(\tau_{load,target}, \omega)
```

を使って `load_cmd` を決めています。

したがって、現行 post-fit runtime の feedback は

- `torque request` を毎回解き直すものではない
- map / surrogate で作った基準点の周りで bench load を整える trim

と理解するのが適切です。

## 10. `Standard runtime` と `Actuator lab` の違い

### 10.1 `Standard runtime`

`Standard runtime` では、
実際に engine へ与える入力を

```math
\mathbf{u}_{std}
=
\begin{bmatrix}
\alpha_{th}^{base} \\
\theta_{ign}^{base} \\
VVT_i^{base} \\
VVT_e^{base} \\
u_{load}
\end{bmatrix}
```

とみなせます。

つまり

1. `Driver demand` から `\tau_{req}` を作る
2. `\tau_{req}` から baseline throttle を作る
3. baseline throttle と current speed から ignition baseline を作る
4. VVT は release 値を使う
5. speed-hold feedback は load 側だけに掛ける

という構成です。

### 10.2 `Actuator lab`

`Actuator lab` では、
load path はそのまま残しつつ、
air-path / combustion-phase 側の actuator を manual override します。

```math
\mathbf{u}_{lab}
=
\begin{bmatrix}
\alpha_{th}^{man} \\
\theta_{ign}^{man} \\
VVT_i^{man} \\
VVT_e^{man} \\
u_{load}
\end{bmatrix}
```

ここで `u_{load}` は依然として
`Driver demand` から作った `\tau_{req}` に従っています。

つまり `Actuator lab` は

- demand-to-load の骨格は維持する
- その上で throttle / ignition / VVT の因果だけを崩して観察する

ための compare / override モードです。

## 11. 現行実装を 1 つの式でまとめる

post-fit runtime をまとめて書くと、
現行実装は概ね次の流れです。

```text
Driver demand d
-> requested brake torque tau_req(d)
-> baseline actuators u_base(tau_req, n*)
-> load trim tau_load,target = tau_req + K_n (n_e - n*)
-> realized control input u
-> engine ODE xdot = f(x, u)
```

ここで

- `Standard runtime`: `u = u_base + [0,0,0,0,\Delta u_{load}]`
- `Actuator lab`: `u = u_{manual-actuator} + [0,0,0,0,\Delta u_{load}]`

と見なせます。

## 12. 物理的な見方

この構成を物理的に読むと、
`Driver demand` は
「どれだけ空気を入れたいか」そのものではなく、
「どれだけ brake torque を欲しいか」の request です。

throttle はその request を実現するための feedforward actuator であり、
標準 runtime では主入力ではありません。

また speed-hold feedback は
air-path や combustion phase に直接入るのではなく、
bench load へ掛かっています。

したがって現行 post-fit runtime は、
ECU 的な意味で完全な production torque structure ではないにせよ、
少なくとも

- request
- feedforward actuator choice
- load-side trim

を分けて見せる構成になっています。

## 13. 現時点の限界

現行実装には次の限界があります。

1. まだ `speed x requested torque` の full map ではなく、startup fit 由来の 1 本の torque curve を主に使っている
2. ignition baseline は full map lookup ではなく、MBT surrogate で作っている
3. VVT baseline はまだ release 値固定で、runtime map から再計算していない
4. `Driver demand -> torque request` は現在は単純な線形写像であり、pedal map として最終形ではない
5. throttle baseline は piecewise linear inverse curve であり、補間外や mode 切替の扱いは今後の拡張余地がある

## 14. 今後の自然な拡張

将来は、この runtime を次のように拡張するのが自然です。

1. `speed x requested brake torque` の 2D map 化
2. `ignition`, `VVT`, `lambda` を含む baseline actuator map 化
3. demand-to-torque の pedal map 層を明示化
4. transient correction を `base map` の上に追加
5. active constraint と protection state の map / runtime 表示

そのとき、現行の

```math
d \rightarrow \tau_{req} \rightarrow u_{base} \rightarrow \mathbf{x}
```

という骨格はそのまま残せます。
