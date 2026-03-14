# Startup Fit MBT 議論メモ

この文書は、startup fit を

- `VVT` 固定
- `throttle / ignition` の離散探索
- `MBT` を ignition 選定の第一基準
- 可変刻みの埋め込み型 `Runge-Kutta`

で組み直すときの議論材料をまとめたメモです。

基礎の定式化は
[STARTUP_FIT_OPTIMIZATION_FORMULATION.ja.md](STARTUP_FIT_OPTIMIZATION_FORMULATION.ja.md)
を参照してください。

## 1. 先に固定した前提

このメモでは次を前提にします。

1. requested speed は `2000 rpm`
2. startup fit 一次段では `VVT_i = VVT_{i,default}`, `VVT_e = VVT_{e,default}`
3. bench 側に必要な負荷トルク `\tau_{bench}` を解く
4. `Indicated torque` と `Required brake torque` は別物として扱う
5. ignition は `MBT` で選ぶ
6. throttle は `MBT` 点群の中から別基準で選ぶ

## 2. `MBT` をどう定義するか

この simulator では、固定した throttle `\alpha` と requested speed `\omega^\star` に対し、
`MBT` は

```math
\theta_{MBT}(\alpha)
=
\arg\max_{\theta}
\tau_{bench}(\alpha,\theta)
```

と置くのが自然です。

ここで

```math
\tau_{bench}(\alpha,\theta)
=
\tau_{load}^{\star}(\alpha,\theta)
```

は、その `(\alpha,\theta)` で周期定常かつ `\bar{\omega}=\omega^\star` を満たすために
必要な負荷トルクです。

## 3. `MBT` だけでは throttle は決まらない

重要なのは、`MBT` は ignition の定義であって、
throttle の定義ではないことです。

もし

```math
\max_{\alpha,\theta}\tau_{bench}(\alpha,\theta)
```

だけを目的にすると、通常は高 throttle 側に寄りやすくなります。

したがって、実装の論理としては

1. 各 throttle で `MBT ignition` を求める
2. その `MBT` 点群から throttle を二次基準で選ぶ

と分けるのがよいです。

## 4. throttle を選ぶ候補基準

### 案 A: 最小 throttle

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

利点:

- startup fit 後の操作量が穏やか
- 高負荷側へ不用意に寄りにくい
- 起動直後の保守的な基準として説明しやすい

懸念:

- brake torque からどこまで低下を許容するか `\tau_{margin}` を決める必要がある

### 案 B: 最高効率 throttle

```math
\alpha^\star
=
\arg\max_{\alpha_i}
\bar{\eta}_{ind}(\alpha_i,\theta_{MBT}(\alpha_i))
```

利点:

- map 生成の思想とつなげやすい
- 物理量ベースで説明しやすい

懸念:

- startup 直後の扱いとしては少し攻めすぎる可能性がある

### 案 C: 基準 throttle 近傍

```math
\alpha^\star
=
\arg\min_{\alpha_i}
\left[
w_\alpha(\alpha_i-\alpha_{ref})^2
- w_\tau \tau_{bench}(\alpha_i,\theta_{MBT}(\alpha_i))
\right]
```

利点:

- UI 上の既定値や過去 fit との連続性が作りやすい

懸念:

- `\alpha_{ref}` の根拠が必要

## 5. adaptive 探索案

注:
現行実装は `16 x 16` の coarse grid を使うが、この節の `8 x 8` 例は計算量の見積もりを説明するための簡略例として残す。

## 5.1 一番わかりやすい構成

初期区間を

```math
\alpha \in [\alpha_{min},\alpha_{max}]
```

```math
\theta \in [\theta_{min},\theta_{max}]
```

とし、それぞれをまず `8` 分割します。

```math
\mathcal{A}^{(0)} = \operatorname{linspace}(\alpha_{min},\alpha_{max},8)
```

```math
\Theta^{(0)} = \operatorname{linspace}(\theta_{min},\theta_{max},8)
```

そして各 throttle `\alpha_i \in \mathcal{A}^{(0)}` について
全 ignition 候補 `\Theta^{(0)}` を評価し、
粗い `MBT` 候補

```math
\theta_{MBT}^{(0)}(\alpha_i)
=
\arg\max_{\theta_j \in \Theta^{(0)}}
\tau_{bench}(\alpha_i,\theta_j)
```

を求めます。

## 5.2 ignition だけを再分割する案

`MBT` は本質的に ignition の探索なので、
粗探索のあとに throttle 全域をもう一度 2 次元再分割するより、
各 throttle ごとに ignition の近傍だけを `8` 分割し直す方が解釈しやすいです。

例えば粗探索で最良だった index を `j^\star(i)` とすると、
その近傍区間

```math
I_i^{(1)} =
[\theta_{j^\star(i)-1},\theta_{j^\star(i)+1}]
```

を再度 `8` 分割し、

```math
\Theta_i^{(1)} = \operatorname{linspace}(I_{i,\min}^{(1)}, I_{i,\max}^{(1)}, 8)
```

の上で

```math
\theta_{MBT}^{(1)}(\alpha_i)
=
\arg\max_{\theta \in \Theta_i^{(1)}}
\tau_{bench}(\alpha_i,\theta)
```

を取り直します。

この方法なら、`MBT` を ignition の局所探索として素直に説明できます。

## 5.3 評価回数

最も単純な `8 x 8` 粗探索だけなら

```math
N_{eval,0}=8\times 8=64
```

です。

粗探索のあとに全 throttle で ignition を再分割するなら

```math
N_{eval,1}=8\times 8=64
```

を追加して、合計

```math
N_{eval,total}=128
```

です。

もし粗探索の上位 `K` 本の throttle だけを再精査するなら

```math
N_{eval,total}=64+8K
```

まで落とせます。

## 5.4 30 秒制約との関係

平均 1 candidate 評価時間を `t_{eval}` [`s_wall`] とすると、
wall-clock 予算は

```math
T_{wall} \approx N_{eval,total}\,t_{eval}
```

です。

wall-clock 制約を置く場合には

```math
t_{eval} \le \frac{T_{wall}}{N_{eval,total}}
```

が一つの目安です。

例えば `N_{eval,total}=128` なら

```math
t_{eval} \le 0.234\ \text{s}
```

が目安になります。

このため、
候補評価の inner solve は fixed-step の長い transient 積分ではなく、
adaptive RK と周期残差評価へ寄せる必要があります。

## 6. 実装形の候補

## 6.1 分離型 `MBT` 探索

```text
for each throttle in 8 bins:
    evaluate 8 ignition bins
    pick coarse MBT ignition
    refine ignition locally with 8 bins
choose throttle from MBT points by secondary rule
```

利点:

- `MBT` の意味が最も明確
- 2 次元全体を何度も張り直さなくてよい
- throttle の選択問題を明示的に分離できる

## 6.2 2 次元 best-cell 再分割

```text
evaluate 8 x 8 coarse grid
take best cell
subdivide that cell into 8 x 8
repeat until budget is exhausted
```

利点:

- 実装が単純
- generic な離散最適化として扱いやすい

懸念:

- `MBT` という言葉との対応はやや弱い

## 7. 可変刻み RK で考えるべきこと

判断材料として、少なくとも次を比較するとよいです。

1. どの埋め込み対を使うか  
候補: `Bogacki-Shampine 3(2)`、`Dormand-Prince 5(4)` など

2. event 処理をどうするか  
phase section crossing、点火時期、燃焼窓の扱い

3. tolerance をどう置くか  
`omega`、pressure、flow state、throttle state でスケールが違う

4. fit 用 solver と plot 用 sampler をどう分けるか  
fit は速さ優先、plot は見やすさ優先

## 8. 次の議論で決めると良いこと

1. throttle の二次基準を `最小 throttle`、`最高効率`、`基準値近傍` のどれにするか
2. 粗探索後の再分割を `ignition のみ` にするか、`2 次元 best-cell` にするか
3. 上位 `K` throttle を何本残すか
4. candidate 評価の収束判定を `周期残差` ベースでどう定めるか
5. UI 上で `Indicated torque` と `Required brake torque` をどう並べるか

## 9. 暫定のおすすめ

今の情報だけで一番筋が良いのは次です。

1. 各 throttle 候補で ignition の `MBT` を取る
2. 再分割はまず ignition 方向だけ `8` 分割で局所化する
3. throttle は `必要 torque margin を満たす最小 throttle` で選ぶ
4. inner solve は adaptive RK + periodic residual で評価する

この構成なら、

- `MBT` という言葉の意味が保たれ
- throttle の役割が曖昧にならず
- 計算予算も見積もりやすい

ので、startup fit の一次段としてかなり扱いやすいです。
