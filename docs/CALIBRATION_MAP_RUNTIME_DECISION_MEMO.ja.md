# Calibration Map / Post-Fit Runtime 意思決定メモ

この文書は、適合後の可視化と操作系について

- 適合で何を決めたことにするか
- マップ作成が何を確定する作業か
- 適合後に何を手動入力として残すか

を整理するための意思決定メモです。

startup fit そのものの定式化は
[STARTUP_FIT_OPTIMIZATION_FORMULATION.ja.md](STARTUP_FIT_OPTIMIZATION_FORMULATION.ja.md)
を参照してください。

現行実装が実際に何を計算しているかの数式整理は
[POST_FIT_RUNTIME_FORMULATION.ja.md](POST_FIT_RUNTIME_FORMULATION.ja.md)
を参照してください。

repository 全体の既定方針は
[IMPLEMENTATION_DIRECTION.ja.md](IMPLEMENTATION_DIRECTION.ja.md)
を参照してください。

## 0. Selected For Current Direction

現時点の議論として、次を現在案とします。

1. 適合は、「所望 operating point から nominal actuator set と予測定常状態を決める inverse problem」と定義する
2. 標準の post-fit runtime では、manual input の主役を `requested brake torque` または `accelerator-equivalent demand` に置く
3. `ignition timing` は標準 runtime では primary manual input にせず、適合で決めた map 値を自動で使う
4. `throttle` の手動操作は教育用途では残してよいが、標準 runtime とは分離した learning mode / actuator-lab mode として扱う
5. fit 後の feedback は map feedforward を置き換えるものではなく、その周辺を整える trim として扱う
6. fit 自体は当面重くてもよいが、fit 後の runtime は map lookup と軽量補正で realtime 可視化を優先する

## 1. Background

現行の GUI は、startup fit 完了後に

- throttle
- spark / fuel on-off
- ignition
- VVT

を手動で触れる構成です。

これは教育用の試行としては有用ですが、
「適合してマップを作ると、その参照だけで状態を速く決められる」
という本来の map runtime の思想とは少し混ざりやすい状態でもあります。

もし適合後も毎回 manual ignition を主入力として動かすなら、

- 適合で何を学習したのか
- マップが何を自動で決めるのか
- runtime がどこまで `map-based` なのか

が曖昧になります。

そのため、

- 標準 runtime で残す manual input
- 教育用に残す manual actuator override

を分けて考える必要があります。

## 2. 適合で何を決めるのか

適合で決めたい対象を、まず次の reference request で表します。

```math
r = \left[n_e,\ \tau_{brake}^{req},\ z\right]
```

ここで

- `n_e`: 現在または要求される engine speed
- `\tau_{brake}^{req}`: 所望 brake torque、またはそれに等価な driver demand
- `z`: mode、ambient、gear、保護状態などの追加条件

です。

この request に対して、適合が求めるのは単なる 1 つの表示値ではなく、
その operating point を成立させるための代表解です。

```math
u^\star(r) = \arg\min_u J(x,u;r)
```

subject to

```math
x^\star = \Phi_T(x^\star, u^\star; r), \quad
c(x^\star, u^\star; r) \le 0
```

と書くと、適合で決めるものは少なくとも次です。

1. `u^\star(r)`: その operating point を作る nominal actuator set
2. `x^\star(r)`: そのときの代表的な内部状態、または周期定常状態
3. `y^\star(r) = g(x^\star, u^\star)`: torque、MAP、air charge、efficiency、temperature などの予測出力
4. `m^\star(r)`: knock、temperature、lambda、pressure などの制約余裕
5. `valid(r)`: そのセルが有効か、補間だけで扱えるか、fallback が必要か

ここで actuator set `u^\star` には、少なくとも

- `throttle`
- `ignition timing`
- `VVT_i`, `VVT_e`
- 必要なら `lambda target`

などが入ります。

つまり適合は、
「要求点に対してどの actuator をどこへ置けばよいか」を決めるだけでなく、
「そのとき engine がどんな状態になるとみなすか」まで含めて決める作業です。

## 3. マップは何を決める作業か

`map を作る` とは、
各セルに数値を埋めること自体ではなく、
次の対応関係を固定することです。

```text
reference request
-> preferred actuator set
-> predicted engine state
-> constraint margins
-> interpolation / fallback policy
```

重要なのは、
map が「forward の見た目表」ではなく、
request から runtime の基準状態を引くための辞書になっていることです。

この意味で、map 作成は次の 4 つを決める作業だと言えます。

1. どの request 変数を外部入力とみなすか
2. 複数の feasible 解のうち、どれを代表解として採用するか
3. その代表解に対応する内部状態と制約余裕をどう記録するか
4. セル間補間と範囲外入力の扱いをどう固定するか

したがって、適合後の runtime で本当に map を使うなら、
利用者が毎回 manual ignition を決めなくても、
まず map lookup だけで nominal state が立つ構成の方が筋が良いです。

## 4. 適合後に何を手動で残すべきか

### Option A: throttle / ignition / VVT を標準 runtime の主入力にする

利点:

- actuator が出力へどう効くかを直感的に見せやすい
- 教育用途では変化の因果が追いやすい

弱み:

- map runtime を毎回迂回しやすい
- 「適合で何を決めたのか」が薄くなる
- request から state が一意に近く定まる、という map の利点を弱める

結論として、これは learning mode には向いていますが、
標準 runtime の既定にはしない方がよい、というのが現在の整理です。

### Option B: requested torque を標準 runtime の主入力にする

利点:

- map runtime の思想と最も整合的
- fit の成果がそのまま feedforward に使える
- 将来の vehicle model や `controller` 入力へつなぎやすい

弱み:

- 初学者には throttle より抽象的に見えやすい
- pedal 感覚と torque request の違いを説明する必要がある

標準 runtime の default としては、現時点で最も筋が良い案です。

### Option C: manual throttle を残し、ignition は map 自動にする

利点:

- pedal あるいは air-charge の感覚を残せる
- `p-V`、`p-theta`、MAP、pumping の変化が見やすい
- `ignition timing` を map 自動にすることで、適合結果の意味も残しやすい

弱み:

- `throttle` が本当に physical throttle angle なのか、driver demand を模した lever なのかを明示しないと誤解されやすい
- map の主軸を `speed x torque` にするのか `speed x throttle` にするのか、追加整理が必要になる

教育用途ではかなり有力です。
ただしこれは「標準 runtime の唯一の姿」というより、
map runtime に重ねる learning mode として置く方が扱いやすいです。

## 5. ignition timing をどう扱うべきか

この論点については、かなり明確に整理できます。

標準 runtime では、
`ignition timing` は基本的に

- map が返す nominal 値
- あるいはその nominal 値に対する小さな feedback / protection trim

として扱うのが自然です。

理由は次です。

1. ignition は適合で学習した代表解の中核そのものだから
2. manual ignition を primary input にすると、map が request から state を決めるという構造が崩れやすいから
3. 教育用途でも、まず `auto ignition from map` を基準線として見せた方が、manual compare の意味が明確だから

したがって、
「点火時期は適合で学習した値を自動で取ればよいのではないか」
という問いに対する現在案は、
標準 runtime では `yes` です。

manual ignition は残してもよいですが、
その場合は

- expert override
- learning compare
- map suggestion との差分観察

として明示的に位置付けるべきです。

## 6. feedback は何をするのか

fit 後に map を使うなら、feedback の役割は
「毎回 operating point を一から決め直すこと」ではなく、
map feedforward の周囲を整えることです。

代表的には次です。

- speed hold や idle stabilization のための load / torque trim
- lambda、air-path、温度保護の微修正
- transient 中の小さな補正
- limit 到達時の保護側補正

整理すると、

```text
manual request
-> map feedforward
-> feedback trim
-> realized state
```

であり、

```text
manual actuator
-> realized state
```

だけで標準 runtime を構成するのとは役割が違います。

## 7. 教育用途として何を残すとよいか

教育用途では、manual throttle を残す価値は高いです。

理由は次です。

1. air charge、MAP、pumping loss、torque の関係が見えやすい
2. 利用者が pedal 感覚に近い入力として理解しやすい
3. `p-V` と `p-theta` の変化が視覚的に追いやすい

一方で manual ignition は、
教育効果がないわけではありませんが、
標準 runtime の主入力というよりは
「auto map 値と比べて、advance / retard が何を変えるかを見る補助入力」
として置く方が筋が通ります。

そのため、教育用 UI は次の 2 層に分ける案が良いです。

1. `Standard map runtime`
   - input は `requested torque` または `accelerator-equivalent demand`
   - ignition / VVT / throttle は map が自動で決める
   - feedback trim と active constraints を見せる
2. `Actuator lab`
   - manual throttle を許可する
   - manual ignition / VVT は optional override として扱う
   - 常に map suggestion と override 差分を表示する

## 8. どうすれば「重い適合」と「触れる realtime runtime」を両立できるか

両立の鍵は、
適合と runtime を役割ごとに明確に分けることです。

### Phase 1: fit は重くてよい

fit では

- 各 operating point で代表解を丁寧に解く
- `throttle`、`ignition`、必要なら `VVT` を決める
- 予測状態、制約余裕、有効範囲も一緒に保存する

ことを優先します。

ここでは wall-clock の速さより、

- 再現性
- 代表解の納得感
- map の質

を優先してよいです。

### Phase 2: post-fit runtime は map 中心で realtime にする

fit 後は、

```text
current speed + manual request
-> map lookup
-> actuator feedforward
-> lightweight feedback / transient correction
-> realtime visualization
```

で回します。

この層では毎回 heavy solve をしません。
state の基準値は map から取り、
realtime 側は補間、軽い feedback、必要なら簡単な transient correction に寄せます。

### Phase 3: curiosity は compare / override mode で受け止める

VVT や点火時期を触りたい興味は、
標準 runtime を壊さずに
次の形でかなえられます。

1. まず map 自動値を baseline として表示する
2. `ignition override` や `VVT override` を個別に on にする
3. 現在値と map baseline の差分を常に表示する
4. override を切れば即座に map 自動値へ戻る

これなら、

- 適合で何を学習したかが残る
- 利用者は自分で advance / retard や VVT 変更を試せる
- realtime の見た目と反応も維持しやすい

## 9. 表示上の注意

## 8. 表示上の注意

post-fit 可視化では、少なくとも次を明示した方がよいです。

1. いまの値が `map feedforward` か `feedback trim` か `manual override` か
2. いま参照している map cell、補間位置、適用中の制約
3. manual override が map の nominal 値からどれだけ外れているか
4. 現在の request が physical throttle angle なのか、driver demand なのか

特に `Throttle cmd` というラベルは、
physical throttle を意味するのか、
pedal 相当要求を意味するのかで解釈が変わります。

標準 runtime の主入力が torque request なら、
表示名も `Driver demand` や `Torque request` に寄せた方が混乱が少ないです。

## 10. Open Choices

まだ詰めるべき論点は次です。

1. 標準 map の主軸を `speed x requested brake torque` に固定するか、`accelerator-equivalent demand` を外部入力層として前面に出すか
2. 教育用の manual lever を、physical `throttle angle` として残すか、pedal 相当の抽象 demand として残すか
3. 初回の runtime map に `VVT` を含めるか、当面は default / scheduled 値で固定するか
4. bench speed-hold と vehicle mode で、同じ map request 定義をどこまで共有するか

## 11. 現時点のおすすめ

現時点では、次の整理が最もバランスが良いです。

1. 標準 post-fit runtime は `requested torque` 系の入力を主役にする
2. ignition は map 自動を既定にし、manual ignition は expert / compare 用に限定する
3. VVT も map 自動を既定にし、manual VVT は compare / override 用に限定する
4. manual throttle は教育用途で残すが、標準 runtime とはモードを分ける
5. 可視化では `map feedforward`, `feedback trim`, `manual override` を混ぜずに見せる

この整理なら、

- 適合の意味
- map runtime の速さ
- 教育用の触って分かる体験

を同時に残しやすいです。
