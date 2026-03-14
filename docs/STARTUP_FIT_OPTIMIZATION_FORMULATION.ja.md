# Startup Fit の数式定式化メモ

この文書は、起動直後の `2000 rpm` startup fit を

- なぜ今は時間がかかるのか
- 何を変数として動かすべきか
- simulator が何の方程式を解いているのか
- そこから最適化問題へどう落とすか

を議論するための整理メモです。

現行実装の主な対応箇所は次です。

- system ODE と closure: [../src/simulator.rs](../src/simulator.rs)
- startup fit の探索則: [../src/dashboard/startup_fit.rs](../src/dashboard/startup_fit.rs)
- startup fit の適用タイミング: [../src/dashboard/state.rs](../src/dashboard/state.rs)
- ODE の全体整理: [MODEL_REFERENCE.ja.md](MODEL_REFERENCE.ja.md)

## 1. 今回の設計条件

今回の議論では、startup fit の設計条件を次のように置きます。

1. 適合中に欲しいのは最終的な定常運転点であり、途中の時間波形そのものは主目的ではない
2. fit の一次段では `VVT_i, VVT_e` は既定値に固定し、探索次元を下げる
3. 外部負荷トルクは `requested rpm` で定常になるために必要な bench 側の負荷として扱う
4. outer search は連続最適化でなく、離散化した有限個の候補から最良点を選ぶ

この前提に立つと、現行の

- `4` 変数近傍探索
- 外部負荷固定ゼロ
- 固定刻みベースの forward integration

は、最終的に解きたい問題に対して少し遠回りです。

## 2. 何を合わせたいのか

startup fit で本当にやりたいことは、
`requested rpm` を指定したときに、その回転数で定常に保てる operating point と
必要 bench torque を数秒で求めることです。

このときの定常条件は

```math
\bar{\omega} = \omega^\star
```

と

```math
x(t + T_{cyc}) = x(t)
```

です。

ここで

- `\omega^\star = 2000 rpm`
- `\bar{\omega}`: cycle-average の crank speed
- `T_{cyc} = 120 / \omega^\star` [`s/cycle`]

です。

また bench が一定回転を保持するなら、必要な外部負荷トルクは

```math
\bar{\tau}_{load}^{\,req}
=
\bar{\tau}_{comb}
- \bar{\tau}_{fric}
- \bar{\tau}_{pump}
```

です。

これは brake / absorber 側が受け持つべきトルクであり、
物理用語としての indicated torque そのものではありません。
UI では

- `Indicated torque`: cylinder `p-V` 由来の内部指標
- `Required brake torque`: requested rpm を保持するための bench 負荷

を分けて表示するのが整合的です。

## 2.1 torque の用語を厳密に分ける

startup fit の議論では、少なくとも次を分けて使うべきです。

### Indicated torque

`p-V` loop 由来の図示仕事を torque に換算した量で、

```math
\tau_{ind}
=
\frac{N_{cyl}}{4\pi}\,\bar{W}_{ind}
```

です。

これは cylinder 内部の仕事ベースの指標であり、
摩擦や外部負荷をまだ引いていません。

### Pumping / friction loss torque

model の損失項は

```math
\tau_{loss} = \tau_{fric} + \tau_{pump}
```

です。

### Net torque

コード上の `torque_net_nm` は

```math
\tau_{net}
=
\tau_{comb} - \tau_{fric} - \tau_{pump} - \tau_{load}
```

です。

これは「外部負荷を掛けたあとの残りの加速トルク」であり、
定常では cycle-average として

```math
\bar{\tau}_{net}=0
```

になります。

### Required brake torque

requested speed を保持するために bench 側が受け持つべき torque は

```math
\tau_{brake,req}
=
\tau_{load}^{\star}
=
\tau_{comb} - \tau_{fric} - \tau_{pump}
```

です。

これは steady periodic condition の下で

```math
\bar{\tau}_{brake,req}
=
\bar{\tau}_{net} + \bar{\tau}_{load}
=
\bar{\tau}_{load}
```

と書けます。

したがって startup fit の MBT 議論で最大化したいのは、
原則として `indicated torque` ではなく
`required brake torque` です。

## 3. 状態方程式

simulator の積分状態は

```math
x =
\begin{bmatrix}
\omega \\
\theta \\
p_{im} \\
p_{ir} \\
p_{em} \\
p_{er} \\
\dot m_{ir} \\
\dot m_{er} \\
\alpha_{th}
\end{bmatrix}
```

です。

model reference と実装を揃えると、積分している ODE は

```math
\frac{d x}{dt}
=
f\!\left(x,u,z(x,u)\right)
```

で、主な成分は

```math
\frac{d}{dt}
\begin{bmatrix}
\omega \\
\theta \\
p_{im} \\
p_{ir} \\
p_{em} \\
p_{er} \\
\dot m_{ir} \\
\dot m_{er} \\
\alpha_{th}
\end{bmatrix}
=
\begin{bmatrix}
\dfrac{\tau_{comb}-\tau_{fric}-\tau_{pump}-\tau_{load}}{J_{eff}} \\
\omega \\
\dfrac{R T_i}{V_{im}}(\dot m_{th}-\dot m_{ir}) \\
\dfrac{R T_i}{V_{ir}}(\dot m_{ir}-\dot m_{cyl}) \\
\dfrac{R T_e}{V_{em}}(\dot m_{er}-\dot m_{tp}) \\
\dfrac{R T_e}{V_{er}}(\dot m_{exh,in}-\dot m_{er}) \\
\dfrac{(p_{im}-p_{ir})-\Delta p_{loss,ir}}{L_{ir}} - c_{ir}\dot m_{ir} \\
\dfrac{(p_{er}-p_{em})-\Delta p_{loss,er}}{L_{er}} - c_{er}\dot m_{er} \\
\dfrac{\alpha_{cmd}-\alpha_{th}}{\tau_{th}}
\end{bmatrix}
```

です。

ここで startup fit の inner solve では、途中の過渡波形そのものではなく

- 周期残差
- cycle-average の `rpm`
- cycle-average の torque / efficiency

が評価対象です。

## 4. 変数の切り分け

## 4.1 一次段で動かす変数

VVT を固定する一次段では、outer search の操作変数を

```math
u =
\begin{bmatrix}
\alpha_{cmd} \\
\theta_{ign}
\end{bmatrix}
```

とします。

固定する変数は

```math
v =
\begin{bmatrix}
VVT_i \\
VVT_e
\end{bmatrix}
=
v_{default}
```

です。

## 4.2 定常条件を満たすために内側で解く量

所要回転数を bench が保持する定常点では、必要な負荷トルク `\tau_{load}` は
search の自由変数というより、定常条件から決まる unknown です。

したがって inner solve の unknown は

```math
\xi =
\begin{bmatrix}
x_s \\
\tau_{load}
\end{bmatrix}
```

と置くのが自然です。

ここで `x_s` は Poincare section 上の状態です。

## 4.3 bench 表示量

candidate `u` に対し、定常解 `\xi^\star(u)` が得られたら

```math
\tau_{bench}(u) := \tau_{load}^\star(u)
```

を bench の `Required brake torque` として表示します。

一方、内部診断用の indicated torque は

```math
\tau_{ind}(u)
=
\frac{N_{cyl}}{4\pi}\,\bar{W}_{ind}(u)
```

で別に保持します。

## 5. 今の startup fit が遅い理由

現行の startup fit は、名前を付けるなら

`multi-actuator coordinate pattern search`

です。

best control `u_k^\star` と step size `\Delta_k` から、候補集合

```math
\mathcal{C}_k =
\left\{
u_k^\star \pm \Delta_{\alpha} e_{\alpha},
u_k^\star \pm \Delta_{ign} e_{ign},
u_k^\star \pm \Delta_{VVT_i} e_{VVT_i},
u_k^\star \pm \Delta_{VVT_e} e_{VVT_e}
\right\}
```

を順番に評価しています。

評価関数は

```math
J(u)
=
\left(
\frac{\omega^\star - \bar{\omega}(u)}{180}
\right)^2
+
\left(
\frac{\bar{\tau}_{net}(u)}{8}
\right)^2
```

です。

現行定数は

```math
T_{prime}=0.30\ \text{s},\quad
T_{settle}=0.10\ \text{s},\quad
T_{measure}=0.16\ \text{s}
```

なので、1 candidate の評価に必要な simulated time は

```math
T_{cand}=T_{settle}+T_{measure}=0.26\ \text{s}
```

です。

`2000 rpm` における 4-stroke の 1 cycle は

```math
T_{cyc} = \frac{120}{2000} = 0.06\ \text{s}
```

なので、candidate 1 個ごとに少なくとも

```math
\frac{T_{cand}}{T_{cyc}} \approx 4.33\ \text{cycles}
```

ぶん system を forward simulate しています。

さらに accuracy-first mode の既定では

```math
\Delta t
\approx
\frac{\Delta \theta_{target}}{6\,rpm}
```

で、`2000 rpm` かつ `\Delta \theta_{target}=1.5 deg` なら

```math
\Delta t \approx 1.25\times 10^{-4}\ \text{s}
```

です。

したがって 1 candidate あたりの概算 step 数は

```math
N_{step,cand}
\approx
\frac{0.26}{1.25\times 10^{-4}}
\approx
2080
```

です。

遅い主因は次です。

1. 候補評価のたびに長い forward simulation をしている
2. periodic orbit を直接解かず、settle + measure window で近似している
3. `VVT` を含めた `4` 変数探索で outer search が漂いやすい
4. timestep が crank-angle 条件に縛られ、過渡全域を細かく追っている

## 6. 目指すべき定式化

## 6.1 inner solve: 所要回転数つき periodic steady solve

candidate `u` と固定 `v_{default}` に対し、
Poincare map を

```math
x_{k+1} = P(x_k, u, v_{default}, \tau_{load})
```

と置きます。

このとき inner solve は

```math
P(x_s,u,v_{default},\tau_{load}) - x_s = 0
```

```math
\bar{\omega}(x_s,u,v_{default},\tau_{load}) - \omega^\star = 0
```

を満たす `x_s, \tau_{load}` を求める問題です。

つまり

```math
\text{find } \xi =
\begin{bmatrix}
x_s \\
\tau_{load}
\end{bmatrix}
\text{ such that }
R(\xi;u)=0
```

であり、残差は

```math
R(\xi;u)
=
\begin{bmatrix}
P(x_s,u,v_{default},\tau_{load}) - x_s \\
\bar{\omega}(x_s,u,v_{default},\tau_{load}) - \omega^\star
\end{bmatrix}
```

です。

### `root-finding` で解くとは何か

ここでいう `root-finding` とは、
上の残差方程式

```math
R(\xi;u)=0
```

を unknown `\xi=[x_s,\tau_{load}]^\mathsf{T}` について
直接解くことです。

例えば Newton 型なら

```math
\xi^{(m+1)}
=
\xi^{(m)} - \left[\frac{\partial R}{\partial \xi}\right]^{-1} R(\xi^{(m)};u)
```

です。

つまり

1. 今の guess `\xi^{(m)}` を置く
2. 1 cycle 積分して残差 `R(\xi^{(m)};u)` を測る
3. Jacobian かその近似を使って `\xi` を修正する

というやり方です。

利点は、うまく近い初期値を入れられれば
少ない反復で periodic steady solve に近づけることです。
一方で、Jacobian 近似や初期値が悪いと実装も挙動も難しくなります。

### `有限 cycle 反復` で近似するとは何か

一方の `有限 cycle 反復` は、
残差方程式を直接解くのでなく、
cycle を何回か回しながら steady に近づくことを期待する方法です。

例えば

```math
x_s^{(k+1)} = P(x_s^{(k)},u,\tau_{load}^{(k)})
```

```math
\tau_{load}^{(k+1)}
=
\tau_{load}^{(k)} + K_\omega\left(\bar{\omega}^{(k)}-\omega^\star\right)
```

のように、
state はそのまま次 cycle の初期値へ渡し、
load torque だけ rpm 誤差で少しずつ修正します。

これは

```math
\xi^{(k+1)} = G(\xi^{(k)};u)
```

という fixed-point iteration です。

利点は実装が単純で頑健に始めやすいことです。
ただし直接 `R(\xi;u)=0` を解くわけではないので、

- 何 cycle で十分か
- 収束が遅いときどう打ち切るか
- gain `K_\omega` をどう置くか

を別に決める必要があります。

### 何が違うのか

感覚的には次の違いです。

- `root-finding`: 「定常条件の方程式を解く」
- `有限 cycle 反復`: 「何 cycle か回して落ち着くところを近似する」

今回の startup fit では、最初の実装は

1. `有限 cycle 反復` で近い点を作る
2. 必要ならその後 `root-finding` で polishing する

というハイブリッドも十分ありえます。

解 `\xi^\star(u)` が得られれば

```math
\tau_{bench}(u)=\tau_{load}^\star(u)
```

をその candidate の bench torque とみなせます。

この定式化の利点は、
`net torque = 0` を外側 objective に置く代わりに
「指定回転で周期定常を作るのに必要な `\tau_{load}`」を
内側 unknown として直接求められることです。

## 6.2 outer solve: 離散化した有限候補から選ぶ

throttle と ignition を離散化して

```math
\mathcal{A}_N = \{\alpha_1,\alpha_2,\dots,\alpha_{N_\alpha}\}
```

```math
\Theta_N = \{\theta_1,\theta_2,\dots,\theta_{N_\theta}\}
```

```math
\mathcal{U}_N = \mathcal{A}_N \times \Theta_N
```

を作ります。

outer optimization は

```math
u^\star = \arg\min_{u \in \mathcal{U}_{N,\text{feas}}} J_{disc}(u)
```

です。

ここで

```math
\mathcal{U}_{N,\text{feas}}
=
\left\{
u \in \mathcal{U}_N
\mid
\|R(\xi^\star(u);u)\| \le \varepsilon_R
\right\}
```

です。

有限集合上の最適化なので、
これは十分に「最適化問題」です。
連続 NLP ではなく、simulation-based discrete optimization と見なせます。

## 6.3 objective の候補

離散候補の評価基準は、map の意味付けに応じて変えられます。

例えば bench torque を主目的にするなら

```math
J_{disc}(u) = -\tau_{bench}(u)
```

です。

効率や極端操作の回避も入れるなら

```math
J_{disc}(u)
=
-w_\tau \tau_{bench}(u)
-w_\eta \bar{\eta}_{ind}(u)
+w_\alpha (\alpha_{cmd}-\alpha_{ref})^2
+w_{mbt} (\theta_{ign}-\theta_{MBT})^2
```

と書けます。

このとき `u` は 2 次元で、`VVT` は固定です。
したがって探索空間は

```math
N_{cand} = N_\alpha N_\theta
```

個の有限候補に明示的に制限され、
runtime の見積りが立てやすくなります。

## 6.4 `MBT` を第一目的にする場合

今回の議論で ignition 側の第一基準を `MBT` に置くなら、
`MBT` は「固定した throttle と requested speed に対し、
必要 bench torque が最大になる ignition」と定義するのが自然です。

つまり各 throttle `\alpha_i` に対して

```math
\theta_{MBT}(\alpha_i)
=
\arg\max_{\theta \in \Theta_i}
\tau_{bench}(\alpha_i,\theta)
```

です。

ここで `\Theta_i` はその throttle 候補の周囲で評価する ignition 候補集合です。

このとき重要なのは、
`MBT` は ignition を選ぶ基準としては強い一方で、
throttle を一意には決めないことです。

実際、もし objective を

```math
\max_{\alpha,\theta} \tau_{bench}(\alpha,\theta)
```

だけにすると、一般には高 throttle 側へ寄りやすく、
「MBT が決まった」ことと「採用 throttle が決まった」ことは別問題です。

したがって実用上は

1. 各 throttle に対し `\theta_{MBT}(\alpha_i)` を求める
2. 得られた `MBT` 点群の中から throttle を二次基準で選ぶ

という二段構えが素直です。

throttle の二次基準としては、例えば次が候補です。

```math
\alpha^\star
=
\arg\min_{\alpha_i}
\alpha_i
\quad
\text{s.t.}
\quad
\tau_{bench}(\alpha_i,\theta_{MBT}(\alpha_i))
\ge
\tau_{bench}^{max} - \tau_{margin}
```

これは
「best brake torque からの margin を保つ中で最小 throttle を選ぶ」
という考え方です。

あるいは

```math
\alpha^\star
=
\arg\max_{\alpha_i}
\bar{\eta}_{ind}(\alpha_i,\theta_{MBT}(\alpha_i))
```

とすれば、
「`MBT` 点群の中で indicated efficiency が最も良い throttle を採る」
という整理になります。

## 6.5 coarse-to-fine も可能

必要なら 1 回の細密格子にせず、

1. 粗い格子 `\mathcal{U}_{N_1}`
2. 上位数点の近傍に細かい格子 `\mathcal{U}_{N_2}`

の 2 段構えにできます。

この場合の総評価回数は

```math
N_{eval} = N_1 + N_2
```

で制御でき、wall-clock 上限を置く場合でも整合を取りやすいです。

## 7. inner evaluator の時間積分

途中の時間波形そのものが適合 target でないなら、
inner evaluator は固定刻みより
誤差制御つきの埋め込み型 Runge-Kutta の方が筋が良いです。

一般の埋め込み対では

```math
k_i = f\!\left(
t_n + c_i h,\;
x_n + h\sum_{j=1}^{i-1} a_{ij} k_j
\right)
```

から

```math
x_{n+1}^{[p]}
=
x_n + h \sum_{i=1}^{s} b_i k_i
```

```math
\hat{x}_{n+1}^{[p-1]}
=
x_n + h \sum_{i=1}^{s} \hat{b}_i k_i
```

を同時に作り、
局所誤差推定を

```math
e_{n+1}=x_{n+1}^{[p]}-\hat{x}_{n+1}^{[p-1]}
```

と置きます。

重み付き誤差ノルム

```math
\|e_{n+1}\|_W
=
\max_i
\frac{|e_{n+1,i}|}
{\mathrm{atol}_i + \mathrm{rtol}_i \max(|x_{n,i}|,|x_{n+1,i}|)}
```

が

```math
\|e_{n+1}\|_W \le 1
```

なら step を accept し、次の刻みは

```math
h_{new}
=
s\,h\,
\|e_{n+1}\|_W^{-1/(p+1)}
```

で更新します。

ここで `s` は safety factor です。

startup fit では、inner evaluator が欲しいのは

- Poincare section までの到達
- cycle-average quantity
- periodicity residual

だけなので、全区間を一定 crank-angle 刻みで密に記録する必要はありません。

### event は「角度は既知、時刻は未知」として扱える

今回の用途では、重要な event はほぼ

- 点火角
- Poincare section crossing

です。

これらは crank angle 上の位置 `\theta_{evt}` 自体は事前に分かります。
分からないのは、その event が wall-clock / simulation time のどこで起きるかだけです。

したがって fit 用 solver では、
一般の未知 event function を毎回探すより

1. 次の event angle `\theta_{evt,next}` を事前計算する
2. adaptive RK の step をその event をまたがないよう制限する
3. 必要なら最後だけ短い補正 step か dense output で event 角へ着地する

という phase-scheduled event handling が第一候補です。

つまり
「event を完全に事前に時刻表化する」のは無理でも、
「event の位相は事前に分かるので、それを solver に教える」のはできます。

### tolerance は一律 `rtol = 1e-9` より重み付きがよい

一律の相対誤差

```math
\mathrm{rtol}=10^{-9}
```

を全状態に掛けるのは、
fit の目的に対しては厳しすぎる可能性があります。

理由は次です。

1. `\omega`, pressure, flow, throttle state で scale が大きく違う
2. fit で欲しいのは state の最終桁ではなく、candidate ranking と `MBT` の安定性
3. wall-clock 制約を置かない場合でも over-solving は避けたい

したがって ODE solver の誤差判定は

```math
\|e\|_W
=
\max_i
\frac{|e_i|}{\mathrm{atol}_i + \mathrm{rtol}\,\max(|x_i^{old}|,|x_i^{new}|)}
```

のような重み付きノルムを使い、
`atol_i` は状態ごとの物理 scale で置く方が自然です。

さらに solver tolerance の妥当性は、
単純な state 誤差でなく

- `\tau_{bench}` の順位が変わるか
- `\theta_{MBT}` が何度ずれるか
- feasible / infeasible 判定が変わるか

で確認するのがよいです。

## 7.1 期待できる効果

埋め込み RK に変えると、

1. 過渡の緩やかな区間では step を大きくできる
2. 変化の急な燃焼近傍だけ step を細かくできる
3. tolerance で精度と速度を明示的に `trade-off` できる

ようになります。

つまり
「途中経過を均一に高密度で見るための solver」から
「周期残差と cycle-average を早く評価する solver」へ
役割を合わせられます。

## 7.2 注意点

ただし、次の点は別途設計が必要です。

1. 燃焼開始や phase section crossing の event をどう検出するか
2. pressure / flow state に対する tolerance をどう正規化するか
3. UI 表示用の fixed-angle sampling と fit 用 inner solve をどう分離するか

したがって実装としては

- fit 用の adaptive evaluator
- 可視化用の diagnostic sampler

を分けるのが自然です。

## 8. これを最適化問題と呼べるか

呼べます。

構造としては

1. `u = [\alpha_{cmd},\theta_{ign}]^\mathsf{T}` を outer の離散変数として持つ
2. 各 candidate ごとに inner periodic solve `R(\xi;u)=0` を解く
3. 得られた `\tau_{bench}(u)`, `\eta_{ind}(u)` などで有限集合上の最良点を選ぶ

という

`continuous inner solve + discrete outer search`

です。

これは gradient-based の連続 NLP ではありませんが、
十分に明確な optimization workflow です。

## 9. 暫定結論

今回の前提に立つと、startup fit の一次段は次の形が最も筋が良いです。

1. `VVT_i, VVT_e` は既定値に固定する
2. outer search は throttle と ignition の有限格子 `\mathcal{U}_N` に限定する
3. 各 candidate では `requested rpm` を満たす periodic steady solve を行い、未知の `\tau_{load}` を求める
4. `\tau_{load}^\star` を bench の `Required brake torque` として表示する
5. indicated torque は従来どおり `p-V` 由来の内部指標として別表示する
6. inner evaluator には可変刻みの埋め込み型 RK を使い、周期残差と cycle-average を高速評価する

この定式化なら、
現行の「4 変数の連続近傍探索で self-sustaining point を追う」より

- 探索次元が低い
- objective が明確
- bench torque の意味が明確
- 計算量を `N_\alpha N_\theta` で予算化しやすい

という利点があります。

## 10. 次に詰める論点

次の議論では、特に以下を決めると実装へ落としやすいです。

1. outer objective を `max bench torque` にするか、効率や MBT 近傍を併せて評価するか
2. throttle / ignition の離散範囲と分割数をどう置くか
3. inner periodic solve を root-finding で行うか、有限 cycle 反復で近似するか
4. fit 用 adaptive RK にどの埋め込み対を使うか
5. bench torque と indicated torque の UI ラベルをどう分けるか
