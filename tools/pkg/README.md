# tools/pkg — ファームウェアパッケージ（.vzp）

RP2350 の UF2 と ESP32-C3 のアプリイメージを 1 つのファイル `vz80-X.Y.Z.vzp` にまとめるスクリプトです。Web UI の「ファームウェア」、`tools/ota/vz80ota.py`、SD カードのルートに置く自動更新（0.6.0 以降）は、いずれもこの形式を受け取ります。

## 形式

64 バイトのヘッダに続けて RP2350 の UF2、ESP32 のアプリの順に連結したものです。

| オフセット | 内容 |
|---|---|
| 0 | `VZP1`（マジック）、ヘッダ長 64 |
| 8 | 版のラベル（24 バイト、`VERSION` の文字列） |
| 32 | RP2350 部分のオフセット・長さ・チェックサム |
| 44 | ESP32 部分のオフセット・長さ・チェックサム |
| 56 | フラグ（未使用）、ヘッダのチェックサム |

チェックサムは `s = s * 31 + byte` の 32 ビット和です。受け取る側（`firmware/esp32c3/main/web.c` の `fw_check_header` / `fw_install`）はヘッダを検証してから、RP2350 部分を link 経由でスロット B にお試し書き込みし、ESP32 部分を空いている OTA スロットに書いて再起動します。SD からの自動更新はラベルの版が動作中より新しいときだけ書き込まれます。

## ビルド

先に両方のファームをビルドしておきます（`README.md` の「ビルド」を参照。版は `VERSION` の 1 行で決まります）。

```sh
python3 tools/pkg/mkpkg.py $(cat VERSION) firmware/rp2350/build/vz80.uf2 firmware/esp32c3/build/vz80-esp.bin vz80-$(cat VERSION).vzp
```

できた `.vzp` は次のどれかで書き込みます。

- Web UI の「ファームウェア」でファイルを選んで「適用+再起動」。
- `python3 tools/ota/vz80ota.py <ip> pkg vz80-X.Y.Z.vzp`。
- SD カードのルートに置く。起動時、起動後にSDカードを刺した時、Web UIなどから更新された場合、30秒ごとのポーリングで発見された時に書き込まれます。
