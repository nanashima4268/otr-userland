# otr-userland 設計（draft-1、2026-11-01）

OTR-4TS3-U1（https://github.com/nanashima4268/otr-4ts3-u1）のホスト側ソフト。
Khronos31 の `*-userland` 製品群（siano-userland などの兄弟製品）と同じ作り方にする。共通基盤 `dtv-userland-core` と `px4-userland` は、版を固定して参照する。

## 1. 構成

| 実行ファイル | 役割 | 互換性 |
|---|---|---|
| `otrd` | USB 機器（FX3）を 1 つのプロセスだけで所有する常駐プロセス。FPGA の設定、チューナーの制御、TS の受け取りと系統ごとの振り分けを行う | `px4d` と同じ運用（フォアグラウンドで動き、runtime-dir に IPC の窓口を作る） |
| `otr-ts` | 1 つの受信機を借りて、TS を標準出力かファイルに出す | **px4-ts／`*-ts` の共通 CLI 仕様**に合わせる（`--instance`、`--receiver`、`--system`、`--freq`（Hz）、`--runtime-dir`、`--output -`、`--tsid`／`--slot`、`--channel`）。これで BonDriver_Userland と mirakc からそのまま使える |
| `otrctl` | 一覧、状態、統計、FPGA の取りこぼしカウンタ、LNB の制御 | `px4ctl` に合わせる |

カードリーダーは扱わない（D-13）。ホスト側で外付けの PC/SC リーダーを使う。

## 2. 依存関係とライセンス

- `dtv-userland-core`（MIT、`cmake/dtv-userland-core.revision`）：常駐プロセスと IPC の部品、CLI、受信機エンジン（`dtv::host_*`、`dtv::receiver`、`dtv::capture` など）。siano-userland と同じリンク構成にする。
- `px4-userland`（GPL-2.0-only、`cmake/px4-userland.revision`）：次のものを流用する。
  - `cxd2856er.*`、`cxd2858er.*`：復調 IC とチューナーの制御
  - `tagged_ts_demux.*`：タグ付き TS の振り分け
  - 公開ヘッダ
- libusb 1.0（Windows では WinUSB を経由する）
- 本体のライセンスは GPL-2.0-only（流用元に合わせる）。

## 3. ハードウェアとのつなぎ目

### 3.1 I²C（`Fx3I2cMaster : px4::userland::BridgeI2cMaster`）

- 受信機ごとに 1 つずつ作る（4 つ）。基板では TCA9546A で I²C を 4 本に分け、すべての復調 IC を 0x64 に置いている（D-24）。
- `request()` は、`BridgeI2cRequest[]`（書き込みと読み出しの並び）を、§4 の I2C_RUN 1 回に変換する。マルチプレクサのチャンネル選択は FX3 ファームが行う。そのため、ホストからはチャンネル番号を渡すだけでよい。
- チューナー（0x60）へのアクセスは、px4-userland と同じく、復調 IC の I²C ゲートを開いて行う。ゲート操作は px4-userland の既存コードがする。

### 3.2 受信機の制御（`OtrFrontend`）

- `Mlt5PeFrontend` と同じ形にする。違いは次の 3 点。
  - 受信機 4 つが、それぞれ専用のバスを持つ（バスの排他制御は受信機ごとに行う）。
  - 電源とリセットは、系統ごとの GPIO で行う（§4 の GPIO）。系統ごとに切り離せるようにする設計（D-06）を、ソフトからも使えるようにする。
  - 水晶は復調 IC が 24MHz、チューナーが 16MHz（参照ボードと同じ）。
- px4-userland の `mlt5pe_frontend` の流用を優先し、私たちの基板に固有の部分だけを書く。

### 3.3 TS（`Fx3Stream`）

- FX3 の EP1 IN バルク転送（USB3、最大パケット 1024 バイト、バースト 16）を、libusb の非同期転送で常に複数本出しておく。
- 受け取ったデータは `TaggedTsDemux(max_tag = 4)` に流す。FPGA は sync バイトを 0x17、0x27、0x37、0x47 に書き換えて送ってくる（D-38）。これは px4_drv／px4-userland の IT930x と同じ形式。
- FX3 は 16KiB のバッファを満杯にしてから送る（D-43）。そのため、1 系統だけを低いレートで受信すると、遅延が大きくなる（約 30Mbps で 4ms 程度。許容範囲）。

### 3.4 FPGA の設定

- 起動時に `otrd` がビットストリーム（`--fpga PATH`、HX8K で約 135KB）を FX3 に送る。FX3 はそれを SPI で iCE40 に書き込む（D-25）。
- ビットストリームは otr-4ts3-u1 の `fpga/` からビルドしたもの。ファームとビットストリームの組み合わせは、§4 の GET_INFO で版を照合する。

## 4. USB プロトコル（v1）

- VID:PID は、開発中は `1209:0001`（pid.codes のテスト用、D-31）。
- インターフェースは 1 つ。EP0（制御）と EP1 IN（バルク、TS）を使う。
- Windows でドライバの手動導入をなくすため、FX3 ファームに MS OS 2.0 ディスクリプタを入れ、WinUSB に自動で結びつける（px4-userland と同じく INF を使わない）。

ベンダーリクエスト（`bmRequestType` は vendor、recipient は device）：

| bRequest | 方向 | wValue / wIndex | データ | 内容 |
|---|---|---|---|---|
| 0xB0 GET_INFO | IN | – | 32 バイト | プロトコル版、ファーム版、FPGA の状態（設定済みか、ビットストリームの ID）、DMA バッファの大きさ |
| 0xB1 I2C_RUN | OUT | wIndex = I²C チャンネル（0〜3） | I2C プログラム（下記） | 書き込みと読み出しの並びを 1 回のトランザクションで実行する（途中は repeated start） |
| 0xB2 I2C_RESULT | IN | – | 状態＋読み出したバイト | 直前の I2C_RUN の結果（どの段で NAK になったか）と、読み出したデータ |
| 0xB3 GPIO | OUT/IN | wValue = 値、wIndex = マスク | – / 4 バイト | 系統ごとの電源とリセット、LNA のバイパス、LNB の有効化、LED。IN で現在値を読む |
| 0xB4 FPGA_LOAD | OUT | wValue = 段階（0 = 開始、1 = データ、2 = 終了） | 最大 4096 バイト | ビットストリームを分割して送る |
| 0xB5 FPGA_REG | OUT/IN | wValue = アドレス | 4 バイト | FPGA の制御レジスタ（系統の有効化、取りこぼしカウンタ、版）。設定後の SPI をレジスタアクセスに使う |
| 0xB6 STREAM | OUT | wValue = 1（開始）／0（停止） | – | GPIF と DMA を開始・停止する。停止時は FIFO を空にする |

I2C プログラムの形式は `{op, addr, len, data...}` の並びとし、合計 512 バイト以下にする。

- op 0x01：write
- op 0x02：read

FX3 側は、受け取った並びを順に実行し、読み出した結果を I2C_RESULT 用のバッファに入れる。

`0xA0` は FX3 のブートローダー（USB からファームを読み込む機能）が使うので避けている。

## 5. FPGA の制御レジスタ（案）

| アドレス | 名前 | 内容 |
|---|---|---|
| 0x00 | ID | 定数 `"OTR1"` |
| 0x01 | VERSION | ビットストリームの版 |
| 0x02 | CH_ENABLE | 系統ごとの TS 受信の有効化（bit0〜3） |
| 0x04〜0x07 | DROP_CNT[0..3] | `ts_pkt_fifo` の取りこぼしカウンタ |
| 0x08 | GPIO_OUT | 系統ごとの電源とリセット（FX3 の GPIO が足りない場合は FPGA 経由にする） |

## 6. 試験

- **実機なしの試験**：dtv-userland-core の `MockTransport` を使い、otr-4ts3-u1 の FPGA シミュレーションが出力したタグ付き TS（`out.hex`）を EP1 IN の入力として流す。`otrd` の振り分け結果が元の TS と一致することを確かめる。これで、FPGA からホストまでの形式の整合を CI で保証できる。
- **I²C**：`Fx3I2cMaster` が組み立てる I2C プログラムを、px4-userland の CXD 系の選局シーケンスと照合する。px4_drv と同じ値になることを確かめる。
- **実機**：v1 基板ができてから行う。参照用の比較は、DTV02A-4TS-P で px4-userland を動かした記録（otr-4ts3-u1 の `docs/reference/px4-userland-dtv02a-4ts-p.md`）と、S0 の C/N。

## 7. 未決事項

- **dtv-userland-core の host 系 API が固まっていない**：README に stable Core API ではないと書かれている。版を固定して追従する。実機が動いたら、上流の作者に相談する。
- **受信停止時の残りの扱い**：FX3 は 16KiB のバッファが満杯になるまで送らないので、受信を止めたときに最後の半端なバッファが届かない。FPGA が止める前にヌルパケットを詰めて満杯にするか、PKTEND を使うかを決める。
- **LNB 給電の安全**：px4-userland と同じく、`--allow-lnb-power` を付けたときだけ有効にする。加えて、ファームが DC 入力を検出できないときは拒否する（D-30）。
