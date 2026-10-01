#include <string.h>
#include <stddef.h>
#include "settings.h"
#include "flashxip.h"
#include "pico/stdlib.h"
#include "hardware/flash.h"
#include "pico/multicore.h"
#include "hardware/sync.h"

#define SET_MAGIC   0x38305A56u   // "VZ80"
#define SET_VERSION 6
#define SET_OFFSET  (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)

settings_t settings;

static uint32_t crc32(const void *p, size_t n) {
    const uint8_t *b = p; uint32_t c = 0xFFFFFFFFu;
    for (size_t i = 0; i < n; i++) { c ^= b[i]; for (int k = 0; k < 8; k++) c = (c >> 1) ^ (0xEDB88320u & -(c & 1)); }
    return ~c;
}
static void defaults(void) {
    memset(&settings, 0, sizeof settings);
    settings.magic = SET_MAGIC; settings.version = SET_VERSION;
    settings.autostart = 1; settings.speed = 1; settings.vdp_gap_us = 8;   // 7 us lost VDP writes on one board after a re-seat
    settings.refresh_us = 12; settings.sys_mhz = 150; settings.ram_kb = 64; settings.fd_cfg[0] = 0; settings.fd_cfg[1] = 2;
}
void settings_load(void) {
    const uint8_t *f = (const uint8_t *)(XIP_BASE + SET_OFFSET);
    uint32_t magic, version, size; memcpy(&magic, f, 4); memcpy(&version, f + 4, 4); memcpy(&size, f + 8, 4);
    defaults();
    if (magic != SET_MAGIC) return;
    if (version >= 6) {
        if (size < 16 || size > sizeof(settings_t)) return;
        uint32_t crc; memcpy(&crc, f + size - 4, 4);
        if (crc != crc32(f, size - 4)) return;
        settings_t cur = settings; memcpy(&cur, f, size - 4); settings = cur;   // fields beyond `size` keep their defaults
        return;
    }
    // older layouts (no size field): salvage the user-facing settings
    struct { uint32_t magic, version; uint8_t autostart, speed, res[2]; uint32_t vdp_gap_us, refresh_us; } old;
    memcpy(&old, f, sizeof old);
    settings.autostart = old.autostart; settings.speed = old.speed;
    if (old.vdp_gap_us >= 2 && old.vdp_gap_us <= 32) settings.vdp_gap_us = old.vdp_gap_us;
    if (old.refresh_us <= 64) settings.refresh_us = old.refresh_us;
    if (version == 5) { uint32_t mhz; memcpy(&mhz, f + 24, 4); if (mhz == 150 || mhz == 200 || mhz == 250 || mhz == 300) settings.sys_mhz = mhz; }
}
bool settings_flash_check(void) {
    const uint8_t *f = (const uint8_t *)(XIP_BASE + SET_OFFSET);
    uint32_t magic, size; memcpy(&magic, f, 4); memcpy(&size, f + 8, 4);
    if (magic != SET_MAGIC || size < 16 || size > sizeof(settings_t)) return false;
    uint32_t crc; memcpy(&crc, f + size - 4, 4);
    return crc == crc32(f, size - 4);
}
bool settings_save(void) {
    static uint8_t page[FLASH_PAGE_SIZE];
    settings.magic = SET_MAGIC; settings.version = SET_VERSION; settings.size = sizeof(settings_t);
    settings.crc = crc32(&settings, offsetof(settings_t, crc));
    memset(page, 0xFF, sizeof page); memcpy(page, &settings, sizeof settings);
    uint32_t irq = save_and_disable_interrupts();
    bool lock = multicore_lockout_victim_is_initialized(1);   // core 1 must not touch PSRAM/XIP while the QMI is in direct mode
    if (lock) multicore_lockout_start_blocking();
    flash_range_erase(SET_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(SET_OFFSET, page, FLASH_PAGE_SIZE);
    flash_xip_safe();
    if (lock) multicore_lockout_end_blocking();
    restore_interrupts(irq);
    return memcmp((const void *)(XIP_BASE + SET_OFFSET), page, sizeof settings) == 0;
}
