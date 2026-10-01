# tools/bootsec — 自作ブートセクタ

Web UI の「イメージ作成」が作る空のディスクイメージ（DOS1 / DOS2 形式の 2DD）に書き込む、ブートセクタの Z80 コードです。ソースは `vzboot.mac`（MIT、約 170 バイト）、`gen.sh` が 2 通りにアセンブルして `firmware/esp32c3/main/bootcode.h` に C の配列として書き出します。

## 生成

```sh
N80=/path/to/N80 sh tools/bootsec/gen.sh
```

Nestor80（N80）で `dos1.bin` と `dos2.bin` をアセンブルし、`bootcode_dos1[]` / `bootcode_dos2[]` として `bootcode.h` を書き換えます。生成物の `bootcode.h` もリポジトリに入れてあるので、Nestor80 がなくても ESP32 側のファームはビルドできます。ESP32 の `h_mkimg`（`files.c`）がこの配列をセクタ 0 のオフセット 1Eh に置きます。
