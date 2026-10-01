#include "crash.h"
#include <stdio.h>
#include <string.h>
#include "fwup.h"
#include "flashxip.h"
#include "led.h"
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "link.h"
#include "hardware/flash.h"
#include "pico/multicore.h"
#include "hardware/sync.h"
#include "hardware/watchdog.h"
#include "msx.h"
#include "slot.h"
#include "settings.h"

#define LINK_UART uart1
#define PIECE 4096
#define SECTOR 4096
#define UF2_MAGIC0 0x0A324655u
#define UF2_MAGIC1 0x9E5D5157u
#define UF2_FAMILY_RP2350_ARM_S 0xE48BFF59u

static uint8_t piece[PIECE];
static uint8_t sector[SECTOR], first[SECTOR];
static uint32_t sector_base = 0xFFFFFFFFu, first_base = 0xFFFFFFFFu;
static bool have_first, sector_dirty;
static uint32_t slot_base;      // XIP address of the slot being written (A: XIP_BASE, B: XIP_BASE + SLOT_B_OFF)

static bool read_exact(uint8_t *dst, uint32_t n) { return link_rx_read(dst, n, 5000); }

static bool program_sector(uint32_t base, const uint8_t *data) {
    led_note_disk();
    uint32_t off = base - XIP_BASE;
    uint32_t irq = save_and_disable_interrupts();
    bool lock = multicore_lockout_victim_is_initialized(1);
    if (lock) multicore_lockout_start_blocking();
    flash_range_erase(off, SECTOR);
    flash_range_program(off, data, SECTOR);
    flash_xip_safe();
    if (lock) multicore_lockout_end_blocking();
    restore_interrupts(irq);
    return memcmp((const void *)base, data, SECTOR) == 0;
}

static bool flush_sector(void) {
    if (!sector_dirty) return true;
    sector_dirty = false;
    if (sector_base == slot_base) { memcpy(first, sector, SECTOR); first_base = sector_base; have_first = true; return true; }
    return program_sector(sector_base, sector);
}

void fwup_receive(uint32_t total, bool to_b) {
    uint32_t got = 0, blocks = 0, top = 0; bool ok = true;
    slot_base = XIP_BASE + (to_b ? SLOT_B_OFF : 0);
    sector_base = 0xFFFFFFFFu; sector_dirty = false; have_first = false;
    memset(sector, 0xFF, SECTOR);
    printf("ready\n"); fflush(stdout);
    while (got < total && ok) {
        uint32_t n = total - got < PIECE ? total - got : PIECE;
        if (!read_exact(piece, n)) { printf("err timeout at %lu\n", (unsigned long)got); ok = false; break; }
        got += n;
        for (uint32_t o = 0; o + 512 <= n && ok; o += 512) {
            uint32_t *h = (uint32_t *)(piece + o);
            if (h[0] != UF2_MAGIC0 || h[1] != UF2_MAGIC1) { printf("err bad uf2 block\n"); ok = false; break; }
            if (h[7] != UF2_FAMILY_RP2350_ARM_S) { printf("err wrong family %08lx\n", (unsigned long)h[7]); ok = false; break; }
            uint32_t addr = h[3], size = h[4];
            if (size != 256 || addr < XIP_BASE || addr + size > XIP_BASE + SLOT_SIZE) { printf("err bad block addr %08lx\n", (unsigned long)addr); ok = false; break; }
            addr = addr - XIP_BASE + slot_base;               // images are linked for slot A; B is reached through address translation
            if (addr + size - slot_base > top) top = addr + size - slot_base;
            uint32_t base = addr & ~(SECTOR - 1);
            if (base != sector_base) {
                if (!flush_sector()) { printf("err verify %08lx\n", (unsigned long)sector_base); ok = false; break; }
                sector_base = base; memset(sector, 0xFF, SECTOR);
            }
            memcpy(sector + (addr - base), piece + o + 32, 256); sector_dirty = true; blocks++;
        }
        if (ok) { printf("ok\n"); fflush(stdout); }
    }
    if (ok && !flush_sector()) { printf("err verify %08lx\n", (unsigned long)sector_base); ok = false; }
    if (ok && have_first && !program_sector(first_base, first)) { printf("err verify first sector\n"); ok = false; }
    if (!ok) { printf("failed after %lu blocks\n", (unsigned long)blocks); return; }
    if (to_b) { settings.try_b = 1; settings.b_len = top; if (!settings_save()) { printf("err settings\n"); return; } }
    printf("done %lu blocks, rebooting%s\n", (unsigned long)blocks, to_b ? " into the trial image" : ""); fflush(stdout);
    msx_request_start_after_reboot();
    sleep_ms(200);
    crash_mark_planned_reboot(); watchdog_reboot(0, 0, 0);
}
