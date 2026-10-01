# tools/nextor — vz80 用 Nextor ドライバ

Nextor カーネルに vz80 の仮想ドライブ（仮想 FDD / 仮想 HDD）を見せるデバイスベースドライバと、カーネルと結合して ROM にするスクリプトです。できた `nextor-vz80.rom` を SD カードに置いてスロットにマウントすると、vz80 の仮想ドライブが Nextor のドライブ（A:、B:、…）になります。

| ファイル | 内容                                                                                                                          |
|---|-----------------------------------------------------------------------------------------------------------------------------|
| `vz80drv.mac` | ドライバ本体。RP2350 が I/O ポート 40h〜4Ch に出す「vz80 ディスクポート」を叩くだけで、機種に依存しません。基板が名乗るユニット数と種類の分だけデバイスを名乗り、Nextor がその順にドライブ文字を付けます。       |
| `mknexrom.py` | カーネル本体（`Nextor-2.1.2.base.dat`）とドライバのバイナリを、Nextor 2.1 Driver Development Guide の手順どおりに結合して 128 KB（ASCII 16K マッパ）の ROM にします。 |
| `build.sh` | 上記2つを使用して nextor-vz80.rom を生成するビルドスクリプト。                                                                                    |

## ビルド

必要なもの:

- Nestor80（`N80`）: https://github.com/Konamiman/Nestor80/releases
- Nextor 2.1.2 のカーネル本体 `Nextor-2.1.2.base.dat`: https://github.com/Konamiman/Nextor/releases

```sh
N80=/path/to/N80 BASE=/path/to/Nextor-2.1.2.base.dat sh tools/nextor/build.sh
```

`tools/nextor/nextor-vz80.rom` が生成されます。  
SD カードの `/vz80/nextor-vz80.rom` に置くと、Web UI の「ドライブ設定」の「仮想 FDD を追加」が使えます。  
