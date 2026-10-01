#pragma once
#include <stdbool.h>
#include <stdint.h>
#define PSRAM_BASE   0x11000000u
#define PSRAM_SIZE   (8u * 1024 * 1024)
// Layout: 0-960 KB mapper RAM expansion (segments 4..63), 960 KB-1 MB work area (below), 1-4.75 MB
// virtual cartridges, 4.75-7.75 MB four floppy image units (768 KB each, shared by the internal disk
// ROM's virtual drives and the Nextor FDD rows), top 256 KB the VRAM shadow.
// Work area (64 KB): +0 hard disk write-back blocks (16 KB), +0x4000 hard disk dirty maps (4 KB),
// +0x5000 FDC format track buffer (8 KB), +0x7000 Z80 PC/SP trace rings (8 KB).
#define PSRAM_WORK     (PSRAM_BASE + 0xF0000u)
#define PSRAM_WORK_HDD (PSRAM_WORK)
#define PSRAM_WORK_TRK (PSRAM_WORK + 0x5000u)
#define PSRAM_WORK_TRC (PSRAM_WORK + 0x7000u)
#define PSRAM_WORK_AUD (PSRAM_WORK + 0xA000u)   // 8 KiB audio sample ring (board 2.0), 8 KiB aligned for the DMA wrap
#define PSRAM_CART_LO  (PSRAM_BASE + 0x100000u)
#define PSRAM_CART_HI  (PSRAM_BASE + 0x4C0000u)
#define PSRAM_FDD_MAX  0xC0000u
#define PSRAM_FDD(d)   (PSRAM_BASE + 0x4C0000u + (uint32_t)(d) * PSRAM_FDD_MAX)
#define PSRAM_VRAM     (PSRAM_BASE + PSRAM_SIZE - 0x40000u)
#define PSRAM_NOCACHE_BASE 0x15000000u   // same window, cache bypassed
typedef struct { uint8_t mfid, kgd, eid[6]; bool ok; uint32_t sck_hz; } psram_info_t;
// Bring the APS6404L on XIP CS1 up in QPI mode. clkdiv = QMI clock divider (>=2).
psram_info_t psram_init(unsigned clkdiv);
psram_info_t psram_info(void);
