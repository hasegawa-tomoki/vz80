# tools/flash — USB 経由の ESP32-C3 書き込み

RP2350 のコンソール（USB CDC）を通して ESP32 をダウンロードモードに入れ、RP2350 の `esp bridge` （USB と ESP の UART をつなぐモード）越しに esptool で書き込みます。    
ESP32に何も書き込まれていない状態でのみ必要です。

## 前提

- RP2350 に vz80 のファームが入っていること。
- ESP-IDF が入っていて、`~/.espressif/python_env/*/bin/python` に esptool があること（ESP-IDF の環境なら自動で見つけます。別の場所なら `--python` で指定）。
- ESP32 側のファームをビルド済みであること（`firmware/esp32c3/build/`）。

## 使い方

```sh
python3 tools/flash/espflash.py firmware/esp32c3/build                 # ブートローダ + パーティション + アプリ（初回や全消去後）
python3 tools/flash/espflash.py --app-only firmware/esp32c3/build      # アプリだけ（NVS の Wi-Fi 設定などを残す）
python3 tools/flash/espflash.py --both-slots firmware/esp32c3/build    # 両方の OTA スロットにアプリを入れて ota_0 から起動
```

USB ポートは `/dev/cu.usbmodem*` を自動で選びます（複数あるときは `--port` で指定）。  
パーティション配置は `firmware/esp32c3/partitions.csv` で指定します（ota_0 が 0x10000、ota_1 が 0x200000）。

## 使い方

1. コンソールに `esp dl` を送り、ESP32 を EN と IO9 でダウンロードモードに入れる（ROM の "waiting for download" を確認）。
2. `esp bridge 120000` でブリッジを開き、esptool を `--before no_reset --after no_reset` で実行。
3. 終わったら 0x1D を 3 回送ってブリッジを閉じ、`esp reset` で ESP32 を通常起動させる。

書き込み後は `python3 tools/console/vz80ctl.py espstatus` で ESP32 の版と Wi-Fi の状態が見えます。

## RP2350 側の書き込み

RP2350 は `picotool` で書込します。  
vz80 のファームが起動していて USB シリアルが見えている基板なら `picotool load -f -v firmware/rp2350/build/vz80.uf2` で書込が完了します。  
USBシリアルが見えない基板はBOOTボタンを押しながらPCに USB を接続すると RP2350 のドライブが認識されるので `picotool load -v ... && picotool reboot` とするとファームウェアを書込できます。    
他のファームウェアが残っている基板は先に `picotool erase -a` してファームウェアを削除してください。  
