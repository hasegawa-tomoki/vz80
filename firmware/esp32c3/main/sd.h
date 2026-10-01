#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
// microSD on the SPI bus (CS 4, CLK 5, MOSI 6, MISO 7, card detect 10). Mounted at /sdcard.
// /vz80.cfg on the card holds "ssid=", "pass=" and "name=" lines (same format as vfd68.cfg).
void sd_init(void);                 // mount if a card is present (call before wifi_start)
void sd_start_monitor(void);        // task: mount/unmount on insertion/removal, apply /vz80.cfg
void sd_reload_cfg(void);           // re-read /vz80.cfg and apply it (after the file was edited)
bool sd_mounted(void);
uint32_t sd_mount_gen(void);          // incremented on every mount (the SD updater rescans per mount)
bool sd_present(void);              // card-detect level
uint64_t sd_size_bytes(void);
// Wi-Fi settings read from /vz80.cfg at the last mount ("" when absent). Returns true if ssid is set.
bool sd_cfg(const char **ssid, const char **pass, const char **name);
const char *sd_cfg_file(void);      // "vz80.cfg" or ""
bool sd_write_cfg(const char *ssid, const char *pass, const char *name);   // create/overwrite /vz80.cfg
// JSON directory listing of PATH (relative to the card root) into out; returns length or -1.
int sd_list_json(const char *path, char *out, size_t n);
