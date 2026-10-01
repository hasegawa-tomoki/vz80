#include "flashxip.h"
#include <string.h>
#include "pico/stdlib.h"
#include "pico/bootrom.h"
#include "pico/multicore.h"
#include "hardware/sync.h"
#include "hardware/flash.h"
#include "hardware/xip_cache.h"
#include "hardware/structs/qmi.h"

void __no_inline_not_in_flash_func(flash_xip_safe)(void) {
    rom_connect_internal_flash_fn connect_flash = (rom_connect_internal_flash_fn)rom_func_lookup_inline(ROM_FUNC_CONNECT_INTERNAL_FLASH);
    rom_flash_exit_xip_fn exit_xip = (rom_flash_exit_xip_fn)rom_func_lookup_inline(ROM_FUNC_FLASH_EXIT_XIP);
    rom_flash_flush_cache_fn flush_cache = (rom_flash_flush_cache_fn)rom_func_lookup_inline(ROM_FUNC_FLASH_FLUSH_CACHE);
    rom_flash_enter_cmd_xip_fn enter_xip = (rom_flash_enter_cmd_xip_fn)rom_func_lookup_inline(ROM_FUNC_FLASH_ENTER_CMD_XIP);
    uint32_t irq = save_and_disable_interrupts();
    uint32_t t1 = qmi_hw->m[1].timing, rf1 = qmi_hw->m[1].rfmt, rc1 = qmi_hw->m[1].rcmd, wf1 = qmi_hw->m[1].wfmt, wc1 = qmi_hw->m[1].wcmd;   // PSRAM window: the ROM resets it too
    connect_flash(); exit_xip();      // leaves continuous-read mode, QMI window 0 back to defaults
    flush_cache(); enter_xip();       // 03h serial reads, as right after boot
    qmi_hw->m[1].timing = t1; qmi_hw->m[1].rfmt = rf1; qmi_hw->m[1].rcmd = rc1; qmi_hw->m[1].wfmt = wf1; qmi_hw->m[1].wcmd = wc1;
    xip_cache_invalidate_all();
    restore_interrupts(irq);
}

bool flash_program_sector(uint32_t off, const uint8_t *data) {
    uint32_t irq = save_and_disable_interrupts();
    bool lock = multicore_lockout_victim_is_initialized(1);
    if (lock) multicore_lockout_start_blocking();
    flash_range_erase(off, FLASH_SECTOR_SIZE);
    flash_range_program(off, data, FLASH_SECTOR_SIZE);
    flash_xip_safe();
    if (lock) multicore_lockout_end_blocking();
    restore_interrupts(irq);
    return memcmp((const void *)(0x1C000000u + off), data, FLASH_SECTOR_SIZE) == 0;   // untranslated, uncached alias: valid in slot B too, never stale
}

static uint32_t lock_begin(bool *lock) { uint32_t irq = save_and_disable_interrupts(); *lock = multicore_lockout_victim_is_initialized(1); if (*lock) multicore_lockout_start_blocking(); return irq; }
static void lock_end(bool lock, uint32_t irq) { flash_xip_safe(); if (lock) multicore_lockout_end_blocking(); restore_interrupts(irq); }
void flash_erase(uint32_t off, uint32_t len) {
    bool lock; uint32_t irq = lock_begin(&lock);
    flash_range_erase(off, len);   // the SDK uses 64 KiB block erases for aligned runs, sector erases otherwise
    lock_end(lock, irq);
}
bool flash_program(uint32_t off, const uint8_t *data, uint32_t len) {
    bool lock; uint32_t irq = lock_begin(&lock);
    flash_range_program(off, data, len);
    lock_end(lock, irq);
    return memcmp((const void *)(0x1C000000u + off), data, len) == 0;
}
uint32_t crc32_buf(const void *p, size_t n) {
    const uint8_t *b = p; uint32_t c = 0xFFFFFFFFu;
    for (size_t i = 0; i < n; i++) { c ^= b[i]; for (int k = 0; k < 8; k++) c = (c >> 1) ^ (0xEDB88320u & -(c & 1)); }
    return ~c;
}
