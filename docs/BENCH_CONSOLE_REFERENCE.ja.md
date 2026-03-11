# Bench Console Reference

この文書は self-contained です。
現行 dashboard の bench-side UI と reduced-order bench model が、どの公開資料を参考にしているか、そして実装へどう対応しているかをまとめます。

## 目的

この dashboard は、特定 vendor の console をそのまま再現することを目的にしていません。
目的は次です。

- 既存の operator display と plot を維持する
- engine dyno に妥当な switch、annunciator、instrument を追加する
- load model は reduced-order のまま保ち、数値的に安定させる
- 追加した要素を公開資料に結び付ける

## 参考にした公開資料

現行の `Bench Console` は、主に次の公式資料に出てくる engine test cell / dynamometer controller の機能群を参考にしています。

- HORIBA SPARC Engine  
  https://www.horiba.com/usa/medical/products/detail/action/show/Product/sparc-95/
- Froude Texcel V12 PRO  
  https://www.froudedyno.com/products/control-systems/texcel-v12-controller
- Froude InCell  
  https://www.froudedyno.com/products/control-systems/incell-control-and-data-acquisition-system

これらの資料から、少なくとも次の機能が engine dyno の operator-facing console で重要だと読めます。

- speed control
- torque / absorber control
- throttle と dynamometer の manual control
- safety / interlock
- alarm / status annunciation
- road-load simulation

## 現行 UI へ落とした要素

### Header Annunciators

現行 header には次を出しています。

- `E-STOP`
- `INTERLOCK`
- `DYNO EN`
- `VENT`
- `COOLING`
- `RUN`
- `FUEL`
- `SPARK`
- `MOTOR`
- `FIRING`
- `SPD CTRL`
- `PWR LIM`
- `ACCURACY`

このうち `E-STOP`、`INTERLOCK`、`DYNO EN`、`VENT`、`COOLING`、`SPD CTRL`、`PWR LIM` が、bench console を意識して今回追加した group です。

### Bench Console Module

`Bench Console` module には次をまとめています。

- `E-STOP`
- `Dyno enable`
- `Cell vent`
- `Cooling cond`
- interlock state
- control mode
- absorber torque / power
- available torque
- overspeed threshold
- rotor inertia
- power-limit torque

これは資料にある `manual control console`、`E-Stop/reset`、`alarm / safety module`、`speed / torque control` の役割を、この repository の reduced-order GUI に合わせて抽出したものです。

### 既存 Display の維持

layout は資料の console をそのまま模写していません。
既存の `Operator Display`、`Status Bus`、`Sensor / State Bus`、`p-V`、`p-theta`、`Engine Motion Schematic` は維持したまま、その上に bench-side 要素を追加しています。

## Reduced-Order Bench Model

### Brake Dyno Mode

`Brake dyno` mode では、吸収器の基準 torque を

```math
\tau_{ref}(\omega) = a_0 + a_1 \omega + a_2 \omega^2
```

で与えます。

高回転側では power rating を超えないように

```math
\tau_{avail}(\omega) =
\min\left(
\tau_{ref}(\omega),
\frac{P_{max}}{\omega}
\right)
```

で available torque を制限します。

command は

```math
s = \operatorname{sgn}(u_{load}) |u_{load}|^{n_{load}}
```

で整形し、最終的な absorber torque は

```math
\tau_{load} = s \tau_{avail}
```

です。

### Vehicle-Equivalent Mode

`Vehicle equivalent` mode では、road load

```math
F_{road} = F_{roll} + F_{grade} + F_{drag}
```

から engine 側へ反映した torque を作ります。
その reference torque に対しても、同じ absorber power limit を掛けます。

### Reflected Inertia

bench 側の coupled rotor inertia を

```math
J_{abs}
```

として常に持たせ、`Vehicle equivalent` では

```math
J_{ref} = J_{abs} + J_{vehicle,ref}
```

としています。

ここで

```math
J_{vehicle,ref}
=
|s| m_{eq} \left(\frac{r_w}{i_{tot}}\right)^2
```

です。

つまり、road-load を切っていても dyno rotor 自体の inertia は engine に見える、という扱いです。

## 実装対応

- bench-side switch / annunciator: [../src/dashboard.rs](../src/dashboard.rs)
- bench widget: [../src/dashboard/widgets.rs](../src/dashboard/widgets.rs)
- external-load config と default: [../src/config.rs](../src/config.rs)
- plausibility audit: [../src/config/audit.rs](../src/config/audit.rs)
- reduced-order load equations: [../src/simulator.rs](../src/simulator.rs)
- checked-in parameter: [../config/sim.yaml](../config/sim.yaml)

## 限界

現状の bench model は reduced-order です。
まだ入れていないものは次です。

- absorber current dynamics
- coolant / oil thermal loop
- load-cell compliance
- driveline lash
- vendor 固有の automation workflow

それでも、speed / torque control、interlock、power limit、reflected inertia までは入っているので、単なる飾りではなく、物理と操作が最低限つながった bench-side abstraction にはなっています。

## Related Documents

- [../README.ja.md](../README.ja.md)
- [USER_MANUAL.ja.md](USER_MANUAL.ja.md)
- [MODEL_REFERENCE.ja.md](MODEL_REFERENCE.ja.md)
- [BENCH_CONSOLE_REFERENCE.md](BENCH_CONSOLE_REFERENCE.md)
