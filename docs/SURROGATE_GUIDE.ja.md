# Surrogate Guide

この文書は、この repository で `surrogate` という言葉をどう使うかの共通認識をまとめたものです。
「何を surrogate と呼ぶか」「何と区別するか」「どう文書化するか」を明示し、
今後の設計・実装・レビューで言葉の意味がぶれないようにすることを目的にしています。

## 1. この repository での `surrogate` の意味

この repository で `surrogate` とは、
厳密な物理式や高次元モデルや実測ベースの詳細モデルをそのまま解かずに、
その代わりとして挙動の主要傾向を軽量に表す近似関係を指します。

重要なのは、`surrogate` は単なる「雑なダミー」ではないことです。
少なくとも次のどれかを狙って導入します。

- realtime 実行を成立させる
- `0D CAE` の範囲で説明可能性を保つ
- fit や map 生成を tractable にする
- 詳細モデルが未実装でも、主要な傾向を教育用途で追えるようにする

## 2. 何と区別するか

### `surrogate` と `closure`

- `closure` は ODE の外で algebraic に決める関係の置き場所です
- `surrogate` はその関係のモデリング上の性質です

したがって、ある `closure` が `surrogate` であることはありますが、
すべての `closure` が `surrogate` とは限りません。

### `surrogate` と `map`

- `map` は request や operating point に対する lookup 成果物です
- `surrogate` は詳細現象を置き換える近似関係です

`map` が `surrogate` 的な役割を持つことはありますが、
この repository では両者を同一視しません。
`map` は fit の成果物、`surrogate` はモデル化の近似です。

### `surrogate` と `display model`

- `display model` は可視化専用の再構成です
- その `display model` が詳細圧力 solve を置き換えているなら、display 用 `surrogate` でもあります

現行の `p-V` は、以前の piecewise な display surrogate から、
display-oriented single-zone pressure ODE に移っています。
main engine state ODE そのものではありませんが、単純な surrogate だけではなくなっています。

### `surrogate` と数値安定化

- `clamp`
- `floor`
- `guard`
- `fallback`
- 補間や外挿の保護

これらは基本的には `surrogate` ではありません。
主目的が現象置換ではなく、数値安定化や異常入力保護だからです。

## 3. この repository で surrogate を使う理由

この simulator は最後まで `0D CAE` を主軸に置きます。
そのため、空間分布や乱流燃焼や詳細化学や実圧トレースを全部そのまま扱うのではなく、
物理解釈を保ちながら reduced-order にまとめる必要があります。

ここで surrogate は次の役割を持ちます。

- 高次元・高計算コストの現象を `0D` のまま扱う
- 実装と数式の対応を追いやすくする
- fit 後の runtime を map lookup と軽量補正で回せるようにする
- 教育用途で「どの入力がどの傾向を作るか」を見せやすくする

## 4. surrogate を導入するときに最低限書くべきこと

新しい surrogate を導入するときは、少なくとも次を文書か code comment で追えるようにします。

- 何を置き換えているか
- 入力と出力と単位
- その surrogate が効く範囲
- 物理 anchor、文献傾向、経験則、fit 結果のどれに基づくか
- ODE 本体に効くのか、fit にだけ効くのか、display 専用なのか
- 外したときに何が不正確になりうるか

### 数式で書くと

この repository では surrogate を、まず次の形で読むと整理しやすいです。

```math
y = \mathcal{S}(x; \beta)
```

- `x`: state, request, operating point などの入力
- `beta`: surrogate 固有の parameter 群
- `y`: 詳細現象の代わりに model が使う reduced-order 出力

実装上はさらに安全側の制約を入れて、次のように使うことが多いです。

```math
\hat{y} = \operatorname{clamp}\left(\mathcal{S}(x; \beta), y_{min}, y_{max}\right)
```

ここで大事なのは、
`clamp` 自体は surrogate ではなく、
その内側の `\mathcal{S}` が「何を何で置き換えているか」です。

## 5. 現在の主な surrogate

### `MBT surrogate`

- 対象: 点火時期に対する MBT 傾向
- 主な実装: `estimate_mbt_deg()`
- 役割: ignition baseline や spark sensitivity の軽量 feedforward

```math
\theta_{MBT}(n, L)
=
\operatorname{clamp}
\left(
\theta_0 + k_n (n-n_{ref}) + k_L (1-L),
\theta_{min},
\theta_{max}
\right)
```

- `n`: engine speed
- `L`: load surrogate。現行実装ではおおむね `p_{int}/p_{amb}` に相当
- `theta_MBT`: MBT timing の reduced-order 推定値

この MBT surrogate の上に、点火ずれに対する phase penalty も置いています。

```math
\Delta \theta_{spk} = \theta_{ign} - \theta_{MBT}
```

```math
\eta_{phase} =
\begin{cases}
\exp\left[-\left(\frac{\Delta \theta_{spk}}{\sigma_{adv}}\right)^2\right], & \Delta \theta_{spk} \ge 0 \\
\exp\left[-\left(\frac{\Delta \theta_{spk}}{\sigma_{ret}}\right)^2\right], & \Delta \theta_{spk} < 0
\end{cases}
```

これは「詳細燃焼解析で MBT を毎回解かずに、点火進退の傾向だけを軽量に与える surrogate」です。

### volumetric-efficiency surrogate

- 対象: speed、throttle、VVT に対する充填効率傾向
- 主な実装: `volumetric_efficiency()`
- 役割: trapped air、torque、load sensitivity を reduced-order に表す

現行実装は、`rpm`、`VVT`、`throttle` の積で組んでいます。

```math
b_n = \operatorname{clamp}\left(\frac{n-n_{low}}{n_{high}-n_{low}}, 0, 1\right)
```

```math
VE_{rpm}
=
\operatorname{clamp}
\left(
VE_0 + K_{rpm}\exp\left[-\frac{(n-n_c)^2}{W_n}\right],
VE_{rpm,min},
VE_{rpm,max}
\right)
```

```math
\theta_{i,opt} = \operatorname{lerp}(\theta_{i,low}, \theta_{i,high}, b_n)
```

```math
\theta_{e,opt} = \operatorname{lerp}(\theta_{e,low}, \theta_{e,high}, b_n)
```

```math
M_{i,opt} = \max\left(1-K_i \hat{\theta}_i^2, 0.75\right),
\qquad
\hat{\theta}_i = \frac{VVT_i-\theta_{i,opt}}{\Delta \theta_i}
```

```math
M_{e,opt} = \max\left(1-K_e \hat{\theta}_e^2, 0.75\right),
\qquad
\hat{\theta}_e = \frac{VVT_e-\theta_{e,opt}}{\Delta \theta_e}
```

```math
M_{VVT}
=
\left(1 + k_i VVT_i - k_e VVT_e\right) M_{i,opt} M_{e,opt}
```

```math
M_{th}
=
\operatorname{clamp}
\left(
c_{th,0} + c_{th,1}\sqrt{\alpha_{th}},
M_{th,min},
M_{th,max}
\right)
```

```math
VE_{base}
=
\operatorname{clamp}
\left(
VE_{rpm} M_{VVT} M_{th},
VE_{min},
VE_{max}
\right)
```

この `VE_base` は、そのまま trapped air に入るわけではなく、
さらに overlap / wave-action surrogate で補正された `VE_eff` に進みます。

### overlap / wave-action surrogate

- 対象: overlap、pulse、wave reflection による充填傾向の補正
- 主な実装: `overlap_ve_multiplier()`, `wave_action_state()`
- 役割: `VE_base` に対して、runner / overlap の向きを reduced-order に乗せる

overlap による充填補正は次の形です。

```math
M_{ov}
=
\operatorname{clamp}
\left(
1 + \phi_{ov}(k_p \Pi_{ov} + k_f \Phi_{ov}),
M_{ov,min},
M_{ov,max}
\right)
```

```math
\Pi_{ov} = \frac{p_{int,runner}-p_{exh,runner}}{p_{amb}},
\qquad
\Phi_{ov} = \frac{\dot{m}_{exh,runner}}{\dot{m}_{ref}}
```

wave-action 側では、境界 wave を無次元 head として集約し、

```math
M_{ram} = 1 + k_{ram}\frac{p_{int,wave}}{p_{amb}}
```

```math
H_{scv} = \frac{p_{int,ov}-p_{exh,ov}}{p_{amb}}
```

```math
M_{scv} = 1 + k_{scv} H_{scv}
```

```math
M_{pulse} = \operatorname{clamp}(M_{ram} M_{scv}, M_{pulse,min}, M_{pulse,max})
```

としています。最終的な effective VE は

```math
VE_{eff} = \operatorname{clamp}(VE_{base} M_{ov} M_{pulse}, VE_{eff,min}, VE_{eff,max})
```

です。

### `internal EGR` surrogate

- 対象: overlap、pressure backflow、reverse flow による residual gas 傾向
- 主な実装: `internal_egr_fraction()`
- 役割: 燃焼位相、charge temperature、burn duration への影響を簡約表現する

```math
\Pi_{back} = \max\left(\frac{p_{exh,runner}-p_{int,runner}}{p_{amb}}, 0\right)
```

```math
W_{back} = \max(-H_{scv}, 0)
```

```math
R_{rev}
=
\max\left(
-\frac{\dot{m}_{exh,runner}}{\dot{m}_{rev,ref}},
0
\right)
```

```math
x_{EGR}
=
\operatorname{clamp}
\left(
\phi_{ov}
\left(
x_0 + k_p \Pi_{back} + k_w W_{back} + k_r R_{rev}
\right),
x_{EGR,min},
x_{EGR,max}
\right)
```

この surrogate で得た `x_EGR` は、その後の charge mixing に入り、

```math
m_{res}
=
m_{fresh}\frac{x_{EGR}}{1-x_{EGR}}
```

```math
T_{chg}
=
\frac{
m_{fresh} c_{p,f} T_f + m_{res} c_{p,b} T_{res}
}{
m_{fresh} c_{p,f} + m_{res} c_{p,b}
}
```

の形で charge temperature や `gamma` に影響します。

### efficiency-base surrogate

- 対象: load と rpm に対する熱効率ベース傾向
- 主な実装: `eta_base_offset`, `eta_load_coeff`, `eta_rpm_abs_coeff` を使う部分
- 役割: 絶対熱効率の詳細 solve を置かずに torque curve の大枠を作る

```math
\eta_{base}(n, L)
=
\operatorname{clamp}
\left(
\eta_0 + k_L L - k_n |n-n_{ref}|,
\eta_{base,min},
\eta_{base,max}
\right)
```

実際に torque に入るときは、点火位相 penalty と heat-loss estimate を重ねて

```math
\chi_{loss} = \frac{Q_{loss}}{m_f LHV}
```

```math
\eta_{th}
=
\operatorname{clamp}
\left(
\eta_{base}\eta_{phase} - \chi_{loss},
\eta_{min},
\eta_{max}
\right)
```

としています。これは「詳細熱発生と壁面熱伝達を完全には解かずに、
絶対 torque level の大枠を作る surrogate」です。

### dyno-load surrogate

- 対象: absorber / vehicle-equivalent load の operator-facing 負荷表現
- 主な実装: `external_load_*`
- 役割: engine 単体 simulation 上で負荷の意味を保つ

command 側はまず非線形 scale に変換します。

```math
s_{cmd} = \operatorname{sign}(u)|u|^\gamma
```

`BrakeMap` mode では基準負荷トルクを

```math
\tau_{ref}(\omega)
=
\tau_0 + c_1 \omega + c_2 \omega^2
```

で置きます。

`VehicleEquivalent` mode では、

```math
F_{roll} = C_{rr} m_{eq} g \cos\alpha
```

```math
F_{grade} = m_{eq} g \sin\alpha
```

```math
F_{aero} = \frac{1}{2}\rho C_d A v^2
```

```math
\tau_{ref}
=
\frac{(F_{roll}+F_{grade}+F_{aero})r_w}{i_d \eta_d}
+ \tau_{acc}
```

で engine side の基準負荷トルクに変換します。

最終的な負荷トルクは

```math
\tau_{load}
=
\operatorname{clamp}
\left(
s_{cmd}\tau_{ref},
\tau_{min},
\tau_{max}
\right)
```

です。vehicle equivalent のときは reflected inertia も

```math
J_{ref}
=
J_{abs}
+ |s_{cmd}|\, m_{eq}\left(\frac{r_w}{i_d}\right)^2
```

で与えています。

### `p-V` display pressure solve

- 対象: 表示用 cylinder pressure trend
- 主な実装: `instantaneous_pv_sample()`
- 分類: display-oriented single-zone ODE。main state ODE ではないが、単純な surrogate とも区別する
- 役割: indicated work、phasing、pumping の比較に使う診断表示

これはいまは「surrogate だけ」ではなく、display 用 single-zone pressure ODE です。

```math
\frac{dp}{d\theta}
=
\frac{\gamma(\theta)-1}{V(\theta)}\frac{dQ_{net}}{d\theta}
- \gamma(\theta)\frac{p}{V(\theta)}\frac{dV}{d\theta}
```

```math
\frac{dQ_{net}}{d\theta}
=
m_f LHV \frac{dx_b}{d\theta}
- \frac{dQ_{loss}}{d\theta}
```

つまり分類としては

- `main state ODE` ではない
- ただし単なる algebraic surrogate でもない
- display-oriented pressure solve

と捉えるのがいちばん正確です。

## 6. surrogate ではないが混同しやすいもの

### reduced-order model そのもの

reduced-order であることと surrogate であることは同じではありません。
state ODE の中には、物理的意味を保った簡約式もあれば、
その上に載っている surrogate もあります。

### fit で保存した map

fit で得た map は成果物です。
それ自体が近似表現ではあっても、
この repository ではまず `artifact / map` として扱い、
`surrogate` とは役割を分けます。

### 数値保護

`clamp`、`pressure floor`、`epsilon guard` などは、
まず数値保護や異常値防止のための処理です。
現象を代表させるために入れているのでなければ、
`surrogate` とは呼びません。

## 7. 今後の議論での使い方

この repository で「これは surrogate です」と言うときは、
少なくとも次の 1 行で言い直せるようにします。

- 何を置き換える surrogate か
- 何のために入れている surrogate か
- どこまで信用してよい surrogate か

例:

- 「これは detailed combustion solve の代わりに使う display 用 surrogate」
- 「これは VVT の torque trend を与える reduced-order surrogate」
- 「これは map ではなく、fit 前でも使う model-side surrogate」

## 関連文書

- [IMPLEMENTATION_DIRECTION.ja.md](IMPLEMENTATION_DIRECTION.ja.md): 実装方針と既定ルール
- [MODEL_REFERENCE.ja.md](MODEL_REFERENCE.ja.md): 現行 model の数式、closure、surrogate の実装対応
- [USER_MANUAL.ja.md](USER_MANUAL.ja.md): operator 視点での使い方
