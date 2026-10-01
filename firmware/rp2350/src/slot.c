#include "crash.h"
#include <string.h>
#include <stdio.h>
#include "slot.h"
#include "flashxip.h"
#include "settings.h"
#include "pico/stdlib.h"
#include "pico/bootrom.h"
#include "hardware/structs/scb.h"
#include "hardware/xip_cache.h"
#include "pico/multicore.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "hardware/structs/qmi.h"
#include "hardware/structs/watchdog.h"
#include "hardware/watchdog.h"
#include "hardware/irq.h"
#include "hardware/resets.h"
#include "bus.h"

static bool in_b, promoted, launch_pending;
extern char __flash_binary_end;
#define AUTO_PROMOTE_MS 20000

// QMI address translation for the first 4 MiB of the XIP window (ATRANS0): BASE and SIZE are in 4 KiB
// pages, SIZE is 11 bits (max 0x400 = 4 MiB; 0 disables the region and every access there hangs).
#define ATRANS_4MB 0x400000u
static void atrans(uint32_t base_phys, uint32_t size) {
    if (size > ATRANS_4MB) size = ATRANS_4MB;
    xip_cache_clean_all();                                  // (PSRAM lines) then drop everything: the bootrom's
    qmi_hw->atrans[0] = ((base_phys >> 12) & 0xFFFu) | (((size >> 12) & 0x7FFu) << 16);
    xip_cache_invalidate_all();                             // flash_flush_cache() left stale A lines that B's copy picked up
}

void slot_init(void) {
    // Launched from A through the translated window, B finds ATRANS0 pointing at its own slot; after any
    // real reset the QMI is back at 1:1, so A can never mistake itself for B.
    in_b = (qmi_hw->atrans[0] & 0xFFFu) == (SLOT_B_OFF >> 12);
    hw_clear_bits(&watchdog_hw->ctrl, WATCHDOG_CTRL_ENABLE_BITS);   // the launch watchdog armed by A
    atrans(0, ATRANS_4MB);                                  // B was chained through a translated window: back to 1:1
    settings_load();
    if (in_b) { settings.launch_log |= 2; settings_save(); return; }
    if (!settings.try_b) return;
    settings.try_b = 0; settings.launch_log = 0; settings_save();                    // one try only: a plain reboot returns to A
    launch_pending = true;                                  // launched from slot_launch_if_pending() once A is fully up:
}                                                           // jumping this early (before the peripherals are up) lands back in A
void slot_launch_if_pending(void) {
    if (!launch_pending) return;
    launch_pending = false;
    crash_mark_planned_reboot();                            // A/B launches go through the watchdog too
    watchdog_enable(4000, true);                            // if B hangs before its own init re-arms and clears this, A boots again
    slot_launch_b();                                        // returns only if B holds no plausible image
    hw_clear_bits(&watchdog_hw->ctrl, WATCHDOG_CTRL_ENABLE_BITS);
}
void slot_launch_b(void) {
    // Launch B ourselves: rom_chain_image() reboots the chip to start the image, which loses the address
    // translation and lands back in A. Our images are copy_to_ram: the entry point in the vector table is
    // flash code (crt0) that copies the image from its flash link address, so with B translated to
    // 0x10000000 a plain jump to B's reset vector boots B exactly like the bootrom would.
    const uint32_t *vt = (const uint32_t *)XIP_BASE;
    atrans(SLOT_B_OFF, SLOT_SIZE);                          // slot B appears at 0x10000000
    uint32_t sp = vt[0], pc = vt[1];
    if (sp < 0x20000000u || sp > 0x20082000u || (pc & 0xFF000000u) != 0x10000000u || !(pc & 1)) {
        atrans(0, ATRANS_4MB); settings.launch_log |= 4; settings_save(); return;      // no plausible image in B: stay on A
    }
    settings.launch_log |= 1; settings_save();
    atrans(SLOT_B_OFF, SLOT_SIZE);                          // (settings_save re-entered XIP: make sure B is mapped again)
    multicore_reset_core1();
    save_and_disable_interrupts();
    irq_set_enabled(USBCTRL_IRQ, false);                    // (USB controller left as is: B re-initialises it)
    scb_hw->vtor = XIP_BASE;
    __asm volatile("msr msp, %0\n bx %1" :: "r"(sp), "r"(pc) : "memory");
    __builtin_unreachable();
}
bool slot_is_b(void) { return in_b && !promoted; }

static bool program_from_b(uint32_t off) {
    static uint8_t buf[FLASH_SECTOR_SIZE];
    memcpy(buf, (const void *)(XIP_BASE + SLOT_B_OFF + off), FLASH_SECTOR_SIZE);   // XIP is off during programming
    uint32_t irq = save_and_disable_interrupts();
    bool lock = multicore_lockout_victim_is_initialized(1);
    if (lock) multicore_lockout_start_blocking();
    flash_range_erase(off, FLASH_SECTOR_SIZE);
    flash_range_program(off, buf, FLASH_SECTOR_SIZE);
    flash_xip_safe();
    if (lock) multicore_lockout_end_blocking();
    restore_interrupts(irq);
    return memcmp((const void *)(XIP_BASE + off), buf, FLASH_SECTOR_SIZE) == 0;
}

static bool program_sector_from(uint32_t off, const uint8_t *src) {
    static uint8_t buf[FLASH_SECTOR_SIZE];
    memcpy(buf, src, FLASH_SECTOR_SIZE);
    uint32_t irq = save_and_disable_interrupts();
    bool lock = multicore_lockout_victim_is_initialized(1);
    if (lock) multicore_lockout_start_blocking();
    flash_range_erase(off, FLASH_SECTOR_SIZE);
    flash_range_program(off, buf, FLASH_SECTOR_SIZE);
    flash_xip_safe();
    if (lock) multicore_lockout_end_blocking();
    restore_interrupts(irq);
    return memcmp((const void *)(XIP_BASE + off), buf, FLASH_SECTOR_SIZE) == 0;
}
bool slot_copy_a_to_b(char *msg, size_t n) {
    if (in_b) { snprintf(msg, n, "err running from B"); return false; }
    watchdog_enable(8000, true);   // a hang here reboots into A instead of freezing
    uint32_t len = ((uint32_t)&__flash_binary_end - XIP_BASE + FLASH_SECTOR_SIZE - 1) & ~(FLASH_SECTOR_SIZE - 1);
    for (uint32_t off = 0; off < len; off += FLASH_SECTOR_SIZE)
        if (!program_sector_from(SLOT_B_OFF + off, (const uint8_t *)(XIP_BASE + off))) { snprintf(msg, n, "err verify at %lx", (unsigned long)off); return false; }
    settings.b_len = len; settings_save();
    hw_clear_bits(&watchdog_hw->ctrl, WATCHDOG_CTRL_ENABLE_BITS);
    snprintf(msg, n, "ok copied %lu bytes to slot B", (unsigned long)len);
    return true;
}
void slot_request_try(void) { settings.try_b = 1; settings_save(); }

bool slot_promote(char *msg, size_t n) {
    if (!in_b) { snprintf(msg, n, "err not running the trial image"); return false; }
    if (promoted) { snprintf(msg, n, "ok already promoted"); return true; }
    // We are the image in B: our own extent is the authoritative length (settings.b_len may be lost
    // across a settings-layout change).
    uint32_t len = ((uint32_t)&__flash_binary_end - XIP_BASE + FLASH_SECTOR_SIZE - 1) & ~(FLASH_SECTOR_SIZE - 1);
    if (len < 2 * FLASH_SECTOR_SIZE || len > SLOT_SIZE) { snprintf(msg, n, "err bad image length %lu", (unsigned long)len); return false; }
    watchdog_enable(8000, true);
    for (uint32_t off = FLASH_SECTOR_SIZE; off < len; off += FLASH_SECTOR_SIZE)
        if (!program_from_b(off)) { snprintf(msg, n, "err verify at %lx", (unsigned long)off); return false; }
    if (!program_from_b(0)) { snprintf(msg, n, "err verify first sector"); return false; }
    hw_clear_bits(&watchdog_hw->ctrl, WATCHDOG_CTRL_ENABLE_BITS);
    promoted = true;
    snprintf(msg, n, "ok promoted %lu bytes to slot A", (unsigned long)len);
    return true;
}

// Like vmpu68: a trial image that has run for 20 s without a bus fault is promoted automatically.
void slot_poll(void) {
    static uint32_t next_ms = AUTO_PROMOTE_MS;
    if (!in_b || promoted) return;
    uint32_t now = to_ms_since_boot(get_absolute_time());
    if (now < next_ms) return;
    next_ms = now + AUTO_PROMOTE_MS;                       // retry every 20 s until it succeeds
    if (bus_faulted()) return;
    char m[80]; slot_promote(m, sizeof m); printf("auto-promote: %s\n", m);
}
