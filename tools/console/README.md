# tools/console — USB コンソール

VZ80 の RP2350 は USB-C に USB シリアル（CDC）のコンソールを出しています。普段の操作は Web UI で足りますが、Wi-Fi が使えないときの設定や復旧、デバッグにはこのコンソールを使います。同じコマンドは Web UI の「コンソール」欄や `POST /api/cmd` からも使えます。

## つなぐ

VZ80 の USB-C と PC を USB ケーブルでつなぎます。USB から給電されるので、MSX に挿していない基板単体でも動きます。

- macOS: `/dev/cu.usbmodemXXXX`。初めてつないだときは「アクセサリの接続を許可しますか」のダイアログを許可するまでポートが現れません。
- Windows: デバイスマネージャの「ポート (COM と LPT)」に `USB シリアル デバイス (COMn)`。ドライバは不要です。
- Linux: `/dev/ttyACM0` など。ユーザーを `dialout` グループに入れます。

速度は何でも構いません（USB なので無視されます）。改行は CR か LF。つながると `> ` のプロンプトが出ます。

```sh
screen /dev/cu.usbmodemXXXX                      # 終了は Ctrl-A → k → y
python3 tools/console/vz80ctl.py info            # このディレクトリのスクリプト（下記）
```

Windows は Tera Term か PuTTY で該当の COM ポートを開きます。

## vz80ctl.py

コマンドを送って応答を表示するだけのスクリプトです。引数はコマンドとして順に送られます。スペースを含むコマンド（`fd V1 virtual 3` など）は引用符で 1 引数にしてください。

```sh
python3 tools/console/vz80ctl.py info                     # ボード情報を表示
python3 tools/console/vz80ctl.py jstatus                  # 状態を JSON で
python3 tools/console/vz80ctl.py espstatus                # ESP32 の版と Wi-Fi の状態
python3 tools/console/vz80ctl.py fd cart shelf            # 続けて複数のコマンド
python3 tools/console/vz80ctl.py "fd V1 virtual 3"        # スペースを含むコマンド
python3 tools/console/vz80ctl.py --port /dev/cu.usbmodem101 help
```

## コマンド一覧

`help` で同じ一覧が出ます。「停止中」とあるものは `stop` で Z80 を止めてから使います（`run` で再開）。引数の `ADDR`、`OFF`、`PORT`、`VAL` は 16 進です。

### 状態

| コマンド | 内容 |
|---|---|
| `help` | コマンド一覧 |
| `info` | ファームウェアの版、チップ ID、クロック、温度 |
| `status` | MSX の稼働状態（run、bus、fault など） |
| `jstatus` | 状態を JSON で。`fw` が版、`crash` が直前の異常終了の記録、`slot` が動作中のファームウェアスロット |
| `espstatus` | ESP32 の版、Wi-Fi のモード（`sta` 接続 / `ap` 設定用アクセスポイント）、IP、空きヒープ |
| `slot` | 動作中のファームウェアスロット（A = 確定版、B = テスト起動中） |
| `slotmap` | スロット × サブスロット × ページの割り当てを JSON で |
| `shelf` | キャッシュ（SD カードの ROM / FDD / HDD イメージのフラッシュ上のコピー）の一覧 |
| `fd` | ドライブの一覧（JSON） |
| `cart` | 仮想カートリッジの一覧（JSON） |
| `linkstat` | ESP32 とのリンク（UART / SPI）のエラーと受信リングの状態 |
| `regs` | Z80 レジスタ |
| `screen` / `jscreen` | テキスト画面（VRAM シャドウから。`jscreen` は JSON） |
| `vdpstate` / `vdpregs` | VDP レジスタとパレット（JSON）/ 観測した VDP レジスタ |
| `gpio` | バス側ピンのスナップショット |
| `pins [MS]` | NMI / INT / BUSRQ / RESET を MS ミリ秒サンプリング |

### 設定

`set` と `vdpgap`、`refresh` の変更は `save` を打つまでフラッシュに保存されません。

| コマンド | 内容 |
|---|---|
| `set speed 0\|1\|2` | 0 = MSX 等速、1 = Max、2 = MSXturboR 相当 |
| `set sysclk MHZ` | RP2350 のクロック（150 / 200 / 250 / 300）。起動に失敗すると自動で 150 に戻る |
| `set ram KB` | メインメモリ（メモリマッパ）の容量（64 / 256 / 512 / 1024）。次の起動から |
| `set vdpgap US` | VDP アクセスの最小間隔（µs）。画面が化けるときは大きく |
| `set refresh US` | リフレッシュの間隔（µs） |
| `set autostart 0\|1` | 電源投入時に Z80 を自動で走らせるか |
| `set reset` | 設定を既定値に戻す |
| `save` | 設定をフラッシュに保存 |
| `vdpgap [US]` / `refresh [US]` | 現在値の表示と一時変更 |
| `board [auto\|1\|2]` | 基板の版（通常は自動判別）。`auto` で判別結果を消す |
| `link [uart\|spi]` | ESP32 とのリンクの種類（V2.0 は SPI）。通常は触らない |
| `wifi SSID PASS` | Wi-Fi の設定を ESP32 に保存して接続し直す（空白は使えない）。通常は SD カードの `vz80.cfg` で設定する |
| `wifi clear` | Wi-Fi の設定を消す |
| `led auto \| R G B` | ステータス LED を自動 / 手動の色に |
| `locate [MS]` | LED を白で点滅させて基板を探す |

### 実行制御

| コマンド | 内容 |
|---|---|
| `run` | Z80 を走らせる（バスを有効にする） |
| `stop` | Z80 を止める |
| `reset` | Z80 とスロットモデルをリセット |
| `keys TEXT\|status\|cancel` | キー入力を送る（`\n` が Enter）。`status` で残り、`cancel` で中止 |
| `reboot` | RP2350 を再起動する（MSX は BIOS から起動し直す） |
| `bootsel` | RP2350 を USB ブートローダ（BOOTSEL）で再起動する |
| `esp reset` | ESP32 だけを再起動する（MSX には影響しない） |

### ドライブとカートリッジ

マウントの変更は MSX の再起動後（`reboot` または電源の入れ直し）に反映されます。`I` はキャッシュの番号（`shelf` で確認）。

| コマンド | 内容 |
|---|---|
| `fd A\|B physical\|none\|virtual [I [wp]]` | ドライブ A / B を内蔵 FDD / なし / 仮想（イメージ I、`wp` で書き込み禁止）に |
| `fd V1.. virtual I [wp]` / `fd V1 eject` | Nextor の行にイメージをマウント / アンマウント |
| `fd swap X Y` | 行の入れ替え |
| `vd add fdd\|hdd` / `vd del N` / `vd order 1,2,..` | Nextor のドライブ行の追加・削除・並べ替え |
| `cart set CELL I [TYPE]` | スロット CELL（`1`、`2`、`0-1` ...）に ROM I をマウント。TYPE 省略でマッパ自動判別（0 自動 1 プレーン 2 ASCII 8K 3 ASCII 16K 4 コナミ 5 コナミ SCC） |
| `cart eject CELL` | アンマウント |
| `cart learn CELL` | マッパの再判別 |
| `shelf put SIZE TYPE MTIME PATH` / `shelf get I` / `shelf del I` / `shelf type I T` / `shelf dirty I 0\|1` / `shelf find PATH` | キャッシュの操作（ESP32 が使う。手で使うのは `del` と `type` くらい） |
| `fdsync` / `fddirty SHELF SECTOR` / `hdsync` / `hddirty SHELF BLOCK` | 仮想 FDD / HDD の変更をカードへ書き戻すための内部コマンド（ESP32 が使う） |

### ファームウェア

| コマンド | 内容 |
|---|---|
| `promote` | テスト起動中の版（スロット B）をスロット A にコピーして確定する |
| `fwup SIZE` | ESP32 のリンク経由で RP2350 のファームを受け取る（内部コマンド） |
| `esp dl` | ESP32 をダウンロードモードで再起動する（USB 書き込み用） |
| `esp bridge [IDLE_MS]` | USB と ESP32 の UART を直結する。閉じるには 0x1D（Ctrl-]）を 3 回送って 300 ms 待つ |
| `esp log` | ESP32 のログを表示 |
| `baud BPS` | ESP32 リンクの UART 速度を切り替える（内部コマンド） |

### デバッグ

| コマンド | 内容 |
|---|---|
| `watch [DIV] [io\|all]` | 動作中のバスを受動的にキャプチャ |
| `iotrace [N]` | 直近 N 件の I/O アクセス |
| `fdtrace [N]` | 直近 N 件の FDC レジスタアクセス（7FF8-7FFF） |
| `vzlog` | vz80 ディスクポートの直近のレジスタアクセス |
| `vdpcmds [N]` | 直近 N 件のエミュレートした VDP コマンド |
| `pctrace on\|off\|dump [N]` | 全命令の PC / SP を記録 |
| `brk pc LO HI \| sp MIN \| off` | 条件で Z80 を止める（`pctrace on` が必要） |
| `peek ADDR [LEN]` / `poke ADDR VAL...` | 物理アドレスの読み書き（停止中） |
| `speek ADDR [LEN]` | ページテーブル経由の読み出し |
| `ioread PORT [N] [GAP_US]` / `iowrite PORT VAL` | I/O の直接読み書き（停止中） |
| `vram ADDR [LEN]` | VRAM シャドウの 16 進ダンプ |
| `vramdelta [full]` | 前回から変わった VRAM ブロック（バイナリ、Web UI が使う） |
| `snapshot` | VDP から VRAM 全体を読み戻す（停止中） |
| `slotscan` | 実機の全スロット / サブスロット / ページを探る（停止中） |
| `romcheck [fix]` | ROM キャッシュを実機から読み直して比較（停止中） |
| `romhex NAME OFF LEN` | キャッシュした ROM の 16 進ダンプ |
| `flashdump OFF [LEN]` / `far read ADDR [LEN]` / `far test ADDR` | フラッシュのダンプ（`far` は 16 MB より上） |
| `rpmem ADDR [LEN]` | RP2350 メモリのダンプ |
| `qmi` / `atrans [BASE_KB SIZE_KB]` | QMI / XIP レジスタ、ATRANS0 |
| `flash` / `psram [BYTES] [nocache]` / `psram init CLKDIV` | フラッシュの JEDEC ID、PSRAM のテストと初期化 |
| `sram` | 64 KiB SRAM のパターンテスト（停止中） |
| `bus` / `z80bench [TSTATES]` | バスエンジンのベンチ / バスなしのコア速度 |
| `audio tone HZ [LEVEL] \| off` | V2.0 のテストトーン |
| `espstate up\|joining\|unset` | ESP32 が LED の状態を伝えるための内部コマンド |

## ファームウェアの更新

通常は Web UI の「ファームウェア」か、microSD カードのルートに `vz80-X.Y.Z.vzp` を置く自動更新を使います。Wi-Fi 経由のコマンドライン版は [tools/ota](../ota/README.md)。ここでは USB で書く手順をまとめます。

### RP2350

`picotool`（Raspberry Pi のツール）を使います。vz80 が起動して USB シリアルが見えている基板なら、1 行で書けます。基板は自動で BOOTSEL に落ち、書き込み後に再起動します。

```sh
picotool load -f -v firmware/rp2350/build/vz80.uf2
```

macOS では BOOTSEL のデバイスが別の USB 機器として初めて見えるため、「アクセサリの接続を許可しますか」のダイアログが出ることがあります。picotool が「no accessible RP-series devices in BOOTSEL mode」と言って止まったときはダイアログを許可し、BOOTSEL 状態のまま待っている基板に続けて書きます。

```sh
picotool load -v -x firmware/rp2350/build/vz80.uf2
```

USB シリアルが見えない基板は、BOOT ボタンを押しながら USB をつなぐとブートローダで起動します。`RP2350` という USB ストレージとして見えるので、`picotool load -v -x` で書くか、`vz80.uf2` をそのストレージにコピーします。他のファームウェアが残っている基板は、先に `picotool erase -a` で消してから書きます。

書き込み後は `info` で版を確認します。

### ESP32

ESP32 は RP2350 のコンソール経由で書きます（`esp dl` → `esp bridge` → esptool → `esp reset` を [tools/flash/espflash.py](../flash/README.md) が順に行います）。ESP-IDF の esptool が必要です。

```sh
python3 tools/flash/espflash.py --app-only firmware/esp32c3/build      # アプリだけ（Wi-Fi 設定は残る）
python3 tools/flash/espflash.py firmware/esp32c3/build                 # ブートローダ + パーティション + アプリ（初回や全消去後）
python3 tools/flash/espflash.py --both-slots firmware/esp32c3/build    # 両方の OTA スロットに書く
```

`--both-slots` は壊れた版が入ったスロットを確実に消したいときだけ使います。書いた版が起動しなかったときに戻る先がなくなるので、まず片方に書いて起動を確認してからにしてください。書き込み後は `espstatus` で版を確認します。

### 更新のあとで

RP2350 の新しい版は「テスト起動」（スロット B）で起動し、20 秒問題なく動けば自動で確定、起動に失敗すれば元の版に戻ります。`slot` で状態を、`promote` で手動の確定ができます。USB の `picotool` で書いた場合はスロット A に直接書かれるので、この仕組みは通りません。
