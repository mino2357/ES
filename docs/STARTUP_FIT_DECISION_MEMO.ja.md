# Startup Fit 意思決定メモ

この文書は、startup fit に関する議論を
今後の commit でも追跡できるように残すための意思決定メモです。

- 方向が固まった点は `Current direction`
- まだ最終決定していない点は `Open choices`
- 将来の commit では、この文書か関連文書を更新して整合を取る

基礎の定式化は
[STARTUP_FIT_OPTIMIZATION_FORMULATION.ja.md](STARTUP_FIT_OPTIMIZATION_FORMULATION.ja.md)
を参照してください。

`MBT` と throttle 議論の詳細は
[STARTUP_FIT_MBT_DISCUSSION.ja.md](STARTUP_FIT_MBT_DISCUSSION.ja.md)
を参照してください。

## 0. Selected For First Implementation

実装前の選択として、次を採用します。

1. throttle 戦略は `Option A`
2. throttle の二次基準は `最小 throttle with torque margin`
3. event step 制限は `点火角だけ`
4. inner periodic solve は `有限 cycle 反復`
5. fit 用 solver と UI 用 sampling は分離する

## 1. Current Direction

現時点で、議論上かなり固まっている方向は次です。

1. startup fit は rough fit ではなく、数値計算ベースの適合として扱う
2. 一次段では `VVT_i, VVT_e` は既定値に固定する
3. ignition の第一基準は `MBT` とする
4. outer search は `throttle / ignition` の離散探索とし、現行実装では初期から各軸 `16` 分割を使う
5. coarse-to-fine の adaptive 探索を前提にする
6. periodic steady の内側評価は、まず `有限 cycle 反復` で進める
7. fit 用の時間積分は、可変刻みの埋め込み型 `Runge-Kutta` のうち `Fehlberg 4(5)` を第一候補にする
8. fit 用 inner solve と UI 表示用の fixed-angle sampling は分離する
9. event は一般 root search を多用せず、初回実装では次の点火角までを意識した phase-scheduled な処理を優先する
10. tolerance は一律 `rtol = 1e-9` のように置かず、状態量ごとの scale を持つ重み付き誤差と candidate ranking の安定性で決める

## 2. torque の言葉の使い分け

ここは今後も曖昧にしない前提です。

### `Indicated torque`

`p-V` 由来の図示仕事ベースの torque。
内部診断量であり、摩擦や外部負荷をまだ引いていません。

### `Required brake torque`

requested speed で周期定常を成立させるために
bench 側が受け持つべき torque。

### `Net torque`

combustion から friction, pumping, load を引いた残りの torque。
定常点では cycle-average として `0` に近づく量です。

startup fit の `MBT` 議論で最大化したいのは、
原則として `Indicated torque` ではなく
`Required brake torque` です。

## 3. 有限 cycle 反復を選ぶ理由

`root-finding` は定常条件の方程式を直接解くので洗練されていますが、
最初の実装としては少し重いです。

一方、`有限 cycle 反復` は

1. 1 cycle 回す
2. state を次 cycle へ渡す
3. `rpm` 誤差を見て `load torque` を少し修正する
4. 周期残差と speed 誤差が十分小さくなるまで繰り返す

という形で作れるので、実装の見通しが良いです。

今回の startup fit では

- まず堅く動くこと
- inner solve を高速化すること
- 後から `root-finding` polishing を足せること

を重視すると、初手としては `有限 cycle 反復` が妥当です。

## 4. event と時間刻みの扱い

今回の前提では、重要な event 候補は

- 点火角
- phase section crossing

ですが、初回実装では step 制限対象を点火角に限定します。

fit 用 solver では

1. 次の点火角を先に計算する
2. 現在 step がその点火角をまたがないよう `h` を制限する
3. 点火角通過後に必要な cycle-average 更新を行う

という構成が自然です。

この意味で、
「時間刻みを次の点火時期以内に制限する」は
採用済みの方針です。

## 5. throttle は制御変数に入れるべきか

ここはまだ最終決定していない論点です。

争点は次です。

1. `MBT` は ignition の選定基準として明快だが、throttle は別に決める必要がある
2. `WOT` 固定にすると探索次元は下がるが、startup / requested-speed 問題には強すぎる単純化かもしれない
3. throttle を変数に残すと、air charge と pumping の影響を表現できる

## 6. Open Choices

### Option A: throttle を制御変数に入れる

構成:

1. throttle を `16` 分割の離散候補にする
2. 各 throttle で ignition の `MBT` を求める
3. `MBT` 点群から throttle を二次基準で選ぶ

向いている理由:

- startup / 2000 rpm hold という part-load 寄りの問題設定に合いやすい
- pumping loss と air charge の違いをきちんと残せる
- `WOT で十分か` という論点を、比較対象として後で検証できる

弱み:

- `WOT` 固定よりは候補数が増える
- throttle の二次基準を決める必要がある

候補となる二次基準:

- 必要 torque margin を満たす最小 throttle
- `MBT` 点群の中で indicated efficiency 最大
- 基準 throttle 近傍

### Option B: throttle は固定し、`WOT` で評価する

構成:

1. throttle は `WOT` 固定
2. ignition と required brake torque だけを inner solve で評価する
3. `MBT ignition` だけを求める

向いている理由:

- 探索次元が小さくて速い
- full-load の `MBT` 曲線を見るだけなら解釈が簡単

弱み:

- startup / fixed-speed hold の operating point としては air charge が過大になりやすい
- throttle を通じた pumping loss の違いを捨ててしまう
- 結果として得られる点が「2000 rpm を保つ現実的な startup fit 点」より
  「2000 rpm WOT 時の full-air MBT 点」に寄る可能性がある

### Option C: hybrid

構成:

1. まず `WOT` で ignition の粗い `MBT` 傾向を見る
2. その後 throttle 候補を開放し、`WOT MBT` 近傍を初期値として各 throttle の `MBT` を詰める
3. 最後に throttle を二次基準で選ぶ

向いている理由:

- `WOT` を完全に捨てず、比較用ベースラインとして使える
- throttle を残した最終点も求められる

弱み:

- 実装が少し複雑になる
- 初手としては Option A より説明が増える

## 7. `WOT` は一般的か

ここは文脈を分けて考える必要があります。

### full-load / power sweep の文脈

この文脈では、`WOT` で spark sweep して `MBT` や knock limit を見るのは自然です。
つまり

- `WOT` は full-load の基準条件として一般的
- そのとき throttle は探索変数ではない

という整理はかなり自然です。

### 今回の startup fit / requested-speed hold の文脈

今回の問題は

- `2000 rpm` を保つ
- bench 側の `required brake torque` を求める
- startup fit の operating point を決める

というもので、full-load sweep とは目的が違います。

したがってこの文脈では、
`WOT` 固定は有力な比較ベースラインにはなっても、
最初から既定の本命にするかは別問題です。

言い換えると、

- `WOT` 評価は一般的な場面がある
- ただし今回の startup fit 問題にそのまま採用すべきかは未決定

です。

## 8. 現時点のおすすめ

現時点では、次の順が一番筋が良いです。

1. inner solve は `有限 cycle 反復`
2. event は次の点火角と section angle までで step を制限
3. throttle はいったん制御変数に残す
4. ただし `WOT` 固定は baseline case として別に比較できるようにする
5. throttle 選択の二次基準は、まず `必要 torque margin を満たす最小 throttle` を第一候補にする

この構成なら、

- startup fit の意味が崩れにくい
- `WOT` 派の議論にも比較で答えられる
- 将来 `WOT only` へ寄せるかどうかを後で判断できる

## 9. 実装前に固定された項目

今回の議論で、初回実装に向けて次を固定します。

1. throttle 戦略は `Option A`
2. throttle の二次基準は `最小 throttle with torque margin`
3. section event の step 制限は `点火角だけ`

このメモは、今後の commit でも

- 採用済みの方針
- 却下しなかった代替案
- 将来見直す可能性のある論点

を追跡するために残します。
