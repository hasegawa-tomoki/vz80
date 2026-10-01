#include "flashfar.h"
#include "flashxip.h"
#include "pico/stdlib.h"
#include "pico/bootrom.h"
#include "pico/multicore.h"
#include "hardware/sync.h"
#include "hardware/xip_cache.h"
#include "hardware/structs/qmi.h"
#include "hardware/regs/qmi.h"
#include <string.h>

// Everything runs from SRAM with interrupts off and core 1 parked: while the QMI is in direct mode
// neither the flash nor the PSRAM window is reachable.
static bool far_lock; static uint32_t far_irq; static uint32_t m1[5];
static void __no_inline_not_in_flash_func(far_begin)(void) {
    far_irq = save_and_disable_interrupts();
    m1[0] = qmi_hw->m[1].timing; m1[1] = qmi_hw->m[1].rfmt; m1[2] = qmi_hw->m[1].rcmd; m1[3] = qmi_hw->m[1].wfmt; m1[4] = qmi_hw->m[1].wcmd;   // PSRAM window (the ROM may touch it)
    far_lock = multicore_lockout_victim_is_initialized(1); if (far_lock) multicore_lockout_start_blocking();
    xip_cache_clean_all();
    rom_connect_internal_flash_fn connect_flash = (rom_connect_internal_flash_fn)rom_func_lookup_inline(ROM_FUNC_CONNECT_INTERNAL_FLASH);
    rom_flash_exit_xip_fn exit_xip = (rom_flash_exit_xip_fn)rom_func_lookup_inline(ROM_FUNC_FLASH_EXIT_XIP);
    connect_flash(); exit_xip();
    qmi_hw->direct_csr = (6u << QMI_DIRECT_CSR_CLKDIV_LSB) | QMI_DIRECT_CSR_EN_BITS;   // sys/6: 25 MHz at 150, 50 MHz at 300 (13h reads are rated to 50 MHz)
    while (qmi_hw->direct_csr & QMI_DIRECT_CSR_BUSY_BITS) tight_loop_contents();
}
static void __no_inline_not_in_flash_func(far_end)(void) {
    while (qmi_hw->direct_csr & QMI_DIRECT_CSR_BUSY_BITS) tight_loop_contents();
    hw_clear_bits(&qmi_hw->direct_csr, QMI_DIRECT_CSR_EN_BITS | QMI_DIRECT_CSR_ASSERT_CS0N_BITS);
    qmi_hw->m[1].timing = m1[0]; qmi_hw->m[1].rfmt = m1[1]; qmi_hw->m[1].rcmd = m1[2]; qmi_hw->m[1].wfmt = m1[3]; qmi_hw->m[1].wcmd = m1[4];
    flash_xip_safe();                     // connect/exit/flush/enter: 03h serial XIP again, PSRAM window restored
    if (far_lock) multicore_lockout_end_blocking();
    restore_interrupts(far_irq);
}
static inline void cs(bool low) { if (low) hw_set_bits(&qmi_hw->direct_csr, QMI_DIRECT_CSR_ASSERT_CS0N_BITS); else hw_clear_bits(&qmi_hw->direct_csr, QMI_DIRECT_CSR_ASSERT_CS0N_BITS); }
// Byte pipeline through the direct FIFOs (tx NULL = 0xFF dummies, rx NULL = discard).
static void __no_inline_not_in_flash_func(bulk)(const uint8_t *tx, uint8_t *rx, uint32_t n) {
    uint32_t txr = n, rxr = n;
    while (txr || rxr) {
        uint32_t f = qmi_hw->direct_csr;
        if (txr && !(f & QMI_DIRECT_CSR_TXFULL_BITS)) { qmi_hw->direct_tx = tx ? *tx++ : 0xFF; txr--; }
        if (rxr && !(f & QMI_DIRECT_CSR_RXEMPTY_BITS)) { uint8_t v = (uint8_t)qmi_hw->direct_rx; if (rx) *rx++ = v; rxr--; }
    }
}
static void __no_inline_not_in_flash_func(cmd_addr)(uint8_t op, uint32_t addr) {
    uint8_t h[5] = { op, (uint8_t)(addr >> 24), (uint8_t)(addr >> 16), (uint8_t)(addr >> 8), (uint8_t)addr };
    bulk(h, NULL, 5);
}
static uint8_t __no_inline_not_in_flash_func(status)(void) { uint8_t t[2] = { 0x05, 0xFF }, r[2]; cs(true); bulk(t, r, 2); cs(false); return r[1]; }
static bool __no_inline_not_in_flash_func(wait_wip)(uint32_t max_us) {
    absolute_time_t end = make_timeout_time_us(max_us);
    while (status() & 1) { if (time_reached(end)) return false; }
    return true;
}
static void __no_inline_not_in_flash_func(wren)(void) { uint8_t t = 0x06; cs(true); bulk(&t, NULL, 1); cs(false); }
static inline bool in_far(uint32_t a, uint32_t n) { return a >= FAR_BASE && a + n <= FAR_END && a + n >= a; }

bool __no_inline_not_in_flash_func(far_read)(uint32_t addr, uint8_t *buf, uint32_t n) {
    if (!in_far(addr, n)) return false;
    far_begin();
    cs(true); cmd_addr(0x13, addr); bulk(NULL, buf, n); cs(false);
    far_end();
    return true;
}
bool __no_inline_not_in_flash_func(far_program)(uint32_t addr, const uint8_t *data, uint32_t n) {
    if (!in_far(addr, n) || (addr & 0xFF) || (n & 0xFF)) return false;
    bool ok = true;
    far_begin();
    for (uint32_t off = 0; off < n && ok; off += 256) {
        wren();
        cs(true); cmd_addr(0x12, addr + off); bulk(data + off, NULL, 256); cs(false);
        ok = wait_wip(10000);   // tPP max 3 ms
    }
    far_end();
    return ok;
}
bool __no_inline_not_in_flash_func(far_erase_sector)(uint32_t addr) {   // 4 KB (21h), tSE max 400 ms
    if (!in_far(addr, 4096) || (addr & 4095)) return false;
    far_begin();
    wren(); cs(true); cmd_addr(0x21, addr); cs(false);
    bool ok = wait_wip(600000);
    far_end();
    return ok;
}
bool __no_inline_not_in_flash_func(far_erase)(uint32_t addr, uint32_t n) {
    if (!in_far(addr, n) || (addr & (FAR_BLOCK - 1)) || (n & (FAR_BLOCK - 1))) return false;
    bool ok = true;
    for (uint32_t off = 0; off < n && ok; off += FAR_BLOCK) {   // one session per block: interrupts are off inside (tBE max 2 s)
        far_begin();
        wren(); cs(true); cmd_addr(0xDC, addr + off); cs(false);
        ok = wait_wip(3000000);
        far_end();
    }
    return ok;
}
