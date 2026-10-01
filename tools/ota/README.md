# tools/ota — Wi-Fi 経由のファームウェア更新

ESP32 の HTTP API にファームを送るスクリプトです。  
Web UI の「ファームウェア」と同じ経路を、コマンドラインから使えます。  

## 使い方

```sh
python3 tools/ota/vz80ota.py <ip addr> pkg release/vz80-X.Y.Z.vzp     # RP2350 + ESP32 をパッケージで（/api/fw）
python3 tools/ota/vz80ota.py <ip addr> rp  firmware/rp2350/build/vz80.uf2   # RP2350 だけ（/api/rp-fw）
python3 tools/ota/vz80ota.py <ip addr> esp firmware/esp32c3/build/vz80-esp.bin   # ESP32 だけ（/api/esp-fw）
```

`<ip addr>` は vz80 の IP アドレス。

## 動作

- **pkg**: ESP32 がヘッダを検証し、RP2350 部分を link 経由でスロット B に書いて RP2350 を再起動（テスト起動）、続けて ESP32 部分を空いている OTA スロットに書いて ESP32 自身を再起動します。応答は「ok X.Y.Z installed (...)」。
- **rp**: RP2350 だけをスロット B に書いて再起動します。
- **esp**: ESP32 だけを OTA スロットに書いて再起動します。

RP2350 の新しい版は「テスト起動」で起動し、20 秒問題なく動けば自動で確定（スロット A にコピー）、起動に失敗すれば元の版に戻ります。ESP32 も同様に、起動後に確定するまではロールバックの対象です。Web UI の「昇格」「元に戻す」で手動でも操作できます。

RP2350 を更新すると MSX は再起動します。
