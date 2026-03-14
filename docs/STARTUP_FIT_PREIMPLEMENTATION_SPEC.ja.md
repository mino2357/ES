# Startup Fit 実装前仕様書

この文書は、startup fit の実装に入る前に
現在の合意事項を仕様として固定するための文書です。

この仕様は初回実装の対象を定めるものであり、
実装中に方向を変える場合は、この文書と関連文書を同時に更新します。

関連文書:

- [STARTUP_FIT_OPTIMIZATION_FORMULATION.ja.md](STARTUP_FIT_OPTIMIZATION_FORMULATION.ja.md)
- [STARTUP_FIT_MBT_DISCUSSION.ja.md](STARTUP_FIT_MBT_DISCUSSION.ja.md)
- [STARTUP_FIT_DECISION_MEMO.ja.md](STARTUP_FIT_DECISION_MEMO.ja.md)
- [IMPLEMENTATION_DIRECTION.ja.md](IMPLEMENTATION_DIRECTION.ja.md)

## 1. 目的

起動直後の startup fit を、
雑な rough fit ではなく、数値計算ベースの operating-point search として実装する。

初回実装で求めたいのは、
`requested rpm = 2000 rpm` における周期定常な operating point と、
その点での `Required brake torque` である。

## 2. 今回固定する設計判断

初回実装では次を採用する。

1. `VVT_i, VVT_e` は既定値に固定する
2. outer search の変数は `throttle` と `ignition`
3. throttle 戦略は `Option A`
4. ignition は各 throttle 候補に対する `MBT` で選ぶ
5. throttle の二次基準は `最小 throttle with torque margin`
6. inner periodic solve は `有限 cycle 反復`
7. fit 用 solver は可変刻みの埋め込み型 `Fehlberg 4(5)`
8. step 制限 event は初回実装では `点火角だけ`
9. fit 用 inner solve と UI 用 fixed-angle sampling は分離する
10. torque の用語は `Indicated torque`、`Required brake torque`、`Net torque` を厳密に分ける

## 3. torque の定義

### `Indicated torque`

`p-V` 由来の図示仕事ベースの torque。

```math
\tau_{ind}
=
\frac{N_{cyl}}{4\pi}\,\bar{W}_{ind}
```

### `Required brake torque`

requested speed で周期定常を成立させるために
bench 側が受け持つべき torque。

```math
\tau_{brake,req} = \tau_{load}^{\star}
```

### `Net torque`

combustion から friction, pumping, load を引いた残りの torque。

```math
\tau_{net}
=
\tau_{comb}-\tau_{fric}-\tau_{pump}-\tau_{load}
```

定常では cycle-average として

```math
\bar{\tau}_{net}=0
```

に近づく。

## 4. outer search の仕様

## 4.1 候補集合

outer search は throttle / ignition の離散探索とする。

初回は各軸をおよそ `8` 分割した候補から始める。

```math
\mathcal{A}^{(0)} = \{\alpha_1,\dots,\alpha_8\}
```

```math
\Theta^{(0)} = \{\theta_1,\dots,\theta_8\}
```

ただし探索は全面的な固定格子ではなく、
良い領域だけを再分割する adaptive coarse-to-fine を前提にする。

## 4.2 `MBT` の定義

各 throttle 候補 `\alpha_i` に対し、
ignition は

```math
\theta_{MBT}(\alpha_i)
=
\arg\max_{\theta \in \Theta_i}
\tau_{brake,req}(\alpha_i,\theta)
```

で決める。

つまり ignition の第一基準は
`Required brake torque` を最大にすることとする。

## 4.3 throttle の選定

`MBT` 点群の中から throttle は

```math
\alpha^\star
=
\arg\min_{\alpha_i}\alpha_i
\quad
\text{s.t.}
\quad
\tau_{brake,req}(\alpha_i,\theta_{MBT}(\alpha_i))
\ge
\tau_{brake,req}^{max} - \tau_{margin}
```

で選ぶ。

ここで

```math
\tau_{brake,req}^{max}
=
\max_i \tau_{brake,req}(\alpha_i,\theta_{MBT}(\alpha_i))
```

であり、`\tau_{margin}` は
best brake torque からどこまで低下を許容するかを表す margin である。

## 5. inner periodic solve の仕様

## 5.1 採用方式

inner periodic solve は、初回実装では `root-finding` ではなく
`有限 cycle 反復` を採用する。

考え方は次である。

1. ある candidate `(\alpha,\theta)` を固定する
2. その candidate で 1 cycle 積分する
3. cycle 終了時の state を次 cycle の初期値へ渡す
4. `rpm` 誤差に応じて `load torque` を更新する
5. 周期残差と speed 誤差が十分小さくなるまで繰り返す

## 5.2 更新の概念式

概念的には

```math
x_s^{(k+1)} = P(x_s^{(k)},u,\tau_{load}^{(k)})
```

```math
\tau_{load}^{(k+1)}
=
\tau_{load}^{(k)} + K_\omega(\bar{\omega}^{(k)}-\omega^\star)
```

のような fixed-point 反復とする。

`K_\omega`、最大 cycle 数、周期残差閾値、speed 誤差閾値は
初回実装の数値パラメータとして切り出す。

## 5.3 収束判定

少なくとも次を持つ。

1. speed 誤差

```math
|\bar{\omega}-\omega^\star| \le \varepsilon_\omega
```

2. 周期残差

```math
\|x_s^{(k+1)}-x_s^{(k)}\|_{W,per} \le \varepsilon_{per}
```

3. 最大 cycle 数または wall-clock budget

```math
k \le k_{max},\qquad T_{wall} \le 30\ \text{s}
```

## 6. fit 用 solver の仕様

## 6.1 solver 方式

fit 用の時間積分は、誤差制御つきの埋め込み型 `Fehlberg 4(5)` を採用する。

目的は途中波形の高密度描画ではなく、

- cycle-average quantity
- periodicity residual
- candidate ranking

を高速かつ安定に評価することである。

## 6.2 tolerance 方針

solver tolerance は一律 `rtol = 1e-9` としない。

状態ごとの scale を持つ重み付き誤差ノルム

```math
\|e\|_W
=
\max_i
\frac{|e_i|}{\mathrm{atol}_i + \mathrm{rtol}\,\max(|x_i^{old}|,|x_i^{new}|)}
```

で管理し、妥当性は

1. `MBT` の順位が変わるか
2. `Required brake torque` の順位が変わるか
3. feasible / infeasible 判定が変わるか

で判断する。

## 6.3 event と step 制限

初回実装で step 制限に使う event は `点火角だけ` とする。

したがって solver は

1. 次の点火角を事前計算する
2. 現在の step がその角度をまたがないよう `h` を制限する
3. 点火角通過後に必要な combustion / cycle bookkeeping を更新する

という構成を取る。

phase section crossing は初回実装では
専用の step 制限 event にしない。

## 7. UI と fit solver の分離

fit 用 inner solve と UI 表示用の fixed-angle sampling は分離する。

初回実装では次の責務分離を守る。

1. fit solver は candidate evaluation 専用
2. UI sampler は `p-V`, `p-theta`, torque display 用の診断データ専用
3. fit solver の timestep や event 都合で plot のサンプリング仕様を決めない

## 8. 初回実装の非対象

初回実装では次をやらない。

1. `VVT` を outer search の自由変数に戻す
2. 直接 `root-finding` で periodic steady を解く
3. phase section crossing を step 制限 event として最適化する
4. `WOT` 固定のみを既定戦略にする
5. fit solver と UI sampler を共通化する

## 9. 実装時にパラメータ化する項目

実装前に文書で固定できたのは方針であり、
数値は実装時にパラメータ化する。

少なくとも次を切り出す。

1. throttle の探索範囲
2. ignition の探索範囲
3. coarse-to-fine の再分割幅
4. `\tau_{margin,min}`
5. `K_\omega`
6. `\varepsilon_\omega`
7. `\varepsilon_{per}`
8. `k_{max}`
9. solver の `rtol`
10. state ごとの `atol_i`

## 10. 実装完了の受け入れ条件

初回実装の受け入れ条件は、少なくとも次とする。

1. startup fit が rough fit を使わず、数値的な candidate evaluation で開始する
2. `VVT` が固定されたまま `throttle / ignition` の探索が動く
3. 各 throttle で `MBT ignition` が求まる
4. throttle が `最小 throttle with torque margin` で選ばれる
5. inner periodic solve が有限 cycle 反復で動く
6. fit solver が UI 用 sampling から独立している
7. `Indicated torque`、`Required brake torque`、`Net torque` の意味が UI と文書で混ざらない
8. fit 中でも `p-V` と `p-theta` が可視化される
9. wall-clock `30 s` 上限が守られる

## 11. 実装開始前の最終確認

この仕様は、初回実装の土台として採用する。

実装開始時は、

1. 本文書
2. [STARTUP_FIT_DECISION_MEMO.ja.md](STARTUP_FIT_DECISION_MEMO.ja.md)
3. [IMPLEMENTATION_DIRECTION.ja.md](IMPLEMENTATION_DIRECTION.ja.md)

の整合を確認してから進める。
