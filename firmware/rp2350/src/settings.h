#pragma once
#include <stdint.h>
#include <stdbool.h>
// Persistent settings in the last 4 KiB sector of the 16 MiB XIP window.
typedef struct {
    uint32_t magic, version, size;   // size = sizeof(settings_t) when written: fields are append-only, newer
                                     // firmware keeps everything an older one stored (crc sits right before size bytes end)
    uint8_t autostart;          // run automatically when an MSX clock is present at boot
    uint8_t speed;              // 0 = MSX2 (paced to 3.58 MHz), 1 = max (unthrottled), 2 = turboR-like (5x)
    uint8_t reserved[2];
    uint32_t vdp_gap_us;
    uint32_t refresh_us;
    uint32_t try_b;             // boot the slot B image once on the next boot (see slot.h)
    uint32_t b_len;             // bytes of the image in slot B
    uint32_t sys_mhz;           // RP2350 system clock (150 default; 200 / 250 overclock), applied at boot
    uint32_t clk_pending;       // set while booting with a non-default clock; still set at the next boot = it failed -> back to 150
    uint32_t launch_log;        // debug trace of the last trial launch (bit0 A jumped, bit1 B started, bit2 B not plausible, bit3 A fell back)
    uint32_t unused_gap_text_us, unused_gap_blank_us;   // former per-mode VDP gaps (kept for the layout; ignored since 0.3.19)
    uint32_t ram_kb;            // mapper RAM size: 64 (as the machine) / 256 / 512 / 1024; segments beyond 64 KiB live in PSRAM
    uint32_t cart_sel[2];       // virtual cartridge in slot 1 / 2: (shelf index + 1) | mapper type << 8; 0 = none
    uint32_t fd_cfg[2];         // drive A / B: mode (0 physical, 1 virtual, 2 none) | (shelf index + 1) << 8 | write-protect << 24
    uint32_t cart_cell[4];      // virtual cartridges (0.4.1): (shelf index + 1) | type << 8 | primary slot << 16 | secondary << 20; 0 = none
    uint32_t fd_port[2];        // vz80 disk port units 1 / 2 (Nextor, 0.4.1): (shelf index + 1) | write-protect << 24; 0 = none
    uint32_t vd[6];             // Nextor rows V1..V6 in order (0.4.2): kind (1 FDD, 2 HDD) << 28 | (shelf index + 1) | write-protect << 24; 0 = no row
    uint32_t board;             // 0 auto-detect, 1 = board 1.0 (UART link, LED on GPIO45), 2 = board 2.0 (SPI link, LED via ESP32, audio on GPIO45)
    uint32_t board_seen;        // last auto-detected board (1 / 2, 0 unknown): decides the GPIO45 role at boot before the ESP32 is up
    uint32_t crc;
} settings_t;
extern settings_t settings;
void settings_load(void);       // load or fall back to defaults
bool settings_save(void);
// Re-read the stored settings through XIP and check their CRC: false means flash reads are corrupted right now.
bool settings_flash_check(void);
