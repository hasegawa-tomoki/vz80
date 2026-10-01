# CPU 速度ベンチマーク

vz80 の速度設定（MSX2 / turboR / Max）を、WebMSX（https://webmsx.org）の MSX2+ と turboR と同じ BASIC プログラムで比べた記録です（2026-09-23、ファームウェア 0.3.18、RP2350 150 MHz、Sony HB-F1XDJ）。

## プログラム

MSX BASIC で入力します。`TIME` は 1/60 秒ごとに増えるシステム変数なので、表示される数値を 60 で割ると秒になります。

```basic
10 TIME=0
20 FOR I=0 TO 30000:A=A+1:NEXT
30 PRINT TIME
40 TIME=0
50 FOR I=1 TO 5000:A=SQR(I)*3.14:NEXT
60 PRINT TIME
```

- ループ 1: 倍精度の加算とループ制御だけの、インタプリタの基本速度。
- ループ 2: SQR と乗算で、数値演算（math pack）中心。

WebMSX では次の URL を開くと、起動直後に同じ内容が 1 行で自動入力されます（`MACHINE=MSX2P` を `MSXTR` にすると turboR）。

```
https://webmsx.org/?MACHINE=MSX2P&BASIC_ENTER=TIME%3D0%3AFOR%20I%3D0%20TO%2030000%3AA%3DA%2B1%3ANEXT%3APRINT%20TIME%3ATIME%3D0%3AFOR%20I%3D1%20TO%205000%3AA%3DSQR(I)*3.14%3ANEXT%3APRINT%20TIME
```

vz80 では Web UI の「速度」を切り替えてから `RUN` します。コンソールなら `set speed 0|2|1`（0 = MSX2、2 = turboR、1 = Max）です。

## 結果

TIME のティック数（1/60 秒）。小さいほど速い。

| 機種・設定 |                     | ループ 1 | ループ 2 | 対 MSX2+（ループ 1） |
|---|---------------------|---:|---:|---:|
| WebMSX MSX2+ |                     | 8,828 | 未計測（13 分以上） | 1.00 |
| WebMSX turboR | R800, MSX BASIC 4.0 | 1,584 | 7,689 | 5.57 |
| vz80 MSX2 | MSX2+相当に調整          | 8,968 | 未計測 | 0.98 |
| vz80 turboR | turboR相当に調整       | 1,558 | 7,799 | 5.67 |
| vz80 Max | 150 MHz, 制限なし       | 956 | 5,219 | 9.23 |

### RP2350 のクロック別（Max モード）

MSX2 / turboR モードは実時間に調整するので処理速度はRP2350のクロックに依存しません。  
Max モードは速度調整なしに RP2350 の最高速で動作します。  
  
| RP2350 クロック | ループ 1 | ループ 2 | 対 MSX2+（ループ 1） | RP2350 温度 |
|---:|---:|---:|---:|---:|
| 150 MHz | 956 | 5,219 | 9.2 | 48 ℃ |
| 200 MHz | 707 | 3,829 | 12.5 | 49 ℃ |
| 250 MHz | 562 | 3,045 | 15.7 | 51 ℃ |
| 300 MHz | 464 | 2,525 | 19.0 | 52 ℃ |

* クロックにほぼ比例して速くなります。  
* 300 MHz は電圧を 1.30 V に上げるオーバークロック設定です。    
* turboR の BASIC は 4.0、HB-F1XDJ は 3.0 なので厳密には一致しません。
