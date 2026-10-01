// APS6404L-3SQR (8 MiB QSPI PSRAM) on RP2350 QMI chip-select 1, memory-mapped at 0x11000000.
// Sequence per RP2350 datasheet 12.14 (QMI direct mode) and the APS6404L datasheet.
#include "psram.h"
#include "pins.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "hardware/structs/qmi.h"
#include "hardware/structs/xip_ctrl.h"
#include "hardware/sync.h"
#include "pico/bootrom.h"

static psram_info_t info;

static void __no_inline_not_in_flash_func(direct_xfer)(uint32_t tx, uint8_t *rx) {
    qmi_hw->direct_tx = tx;
    while (!(qmi_hw->direct_csr & QMI_DIRECT_CSR_TXEMPTY_BITS)) tight_loop_contents();
    while (qmi_hw->direct_csr & QMI_DIRECT_CSR_BUSY_BITS) tight_loop_contents();
    uint8_t v = (uint8_t)qmi_hw->direct_rx;
    if (rx) *rx = v;
}

// Everything below runs from SRAM with interrupts off: while the QMI is in direct mode the
// flash (XIP) is unreachable, so no flash-resident function may be called here.
static void __no_inline_not_in_flash_func(psram_setup)(unsigned clkdiv, uint32_t sys) {
    uint32_t save = save_and_disable_interrupts();
    gpio_set_function(PIN_PSRAM_CS, GPIO_FUNC_XIP_CS1);
    // The flash sits in continuous-read (XIP) mode; direct mode must not interrupt that or every later
    // uncached flash read hangs. Same dance as the SDK's flash_range_program: exit XIP first, re-enter after.
    rom_connect_internal_flash_fn connect_flash = (rom_connect_internal_flash_fn)rom_func_lookup_inline(ROM_FUNC_CONNECT_INTERNAL_FLASH);
    rom_flash_exit_xip_fn exit_xip = (rom_flash_exit_xip_fn)rom_func_lookup_inline(ROM_FUNC_FLASH_EXIT_XIP);
    rom_flash_flush_cache_fn flush_cache = (rom_flash_flush_cache_fn)rom_func_lookup_inline(ROM_FUNC_FLASH_FLUSH_CACHE);
    rom_flash_enter_cmd_xip_fn enter_xip = (rom_flash_enter_cmd_xip_fn)rom_func_lookup_inline(ROM_FUNC_FLASH_ENTER_CMD_XIP);
    connect_flash(); exit_xip();

    // Direct mode, slow clock for the ID/config phase.
    qmi_hw->direct_csr = (30u << QMI_DIRECT_CSR_CLKDIV_LSB) | QMI_DIRECT_CSR_EN_BITS;
    while (qmi_hw->direct_csr & QMI_DIRECT_CSR_BUSY_BITS) tight_loop_contents();

    // Leave QPI if a previous run left the chip in it (0xF5 sent as quad).
    qmi_hw->direct_csr |= QMI_DIRECT_CSR_ASSERT_CS1N_BITS;
    direct_xfer(QMI_DIRECT_TX_OE_BITS | (QMI_DIRECT_TX_IWIDTH_VALUE_Q << QMI_DIRECT_TX_IWIDTH_LSB) | 0xF5, NULL);
    qmi_hw->direct_csr &= ~QMI_DIRECT_CSR_ASSERT_CS1N_BITS;
    for (volatile int i = 0; i < 2000; i++) tight_loop_contents();

    // Read ID: 0x9F, 24 dummy address bits, then MFID, KGD, EID[6].
    uint8_t rx[16];
    qmi_hw->direct_csr |= QMI_DIRECT_CSR_ASSERT_CS1N_BITS;
    for (int i = 0; i < 12; i++) direct_xfer(i == 0 ? 0x9F : 0xFF, &rx[i]);
    qmi_hw->direct_csr &= ~QMI_DIRECT_CSR_ASSERT_CS1N_BITS;
    info.mfid = rx[4]; info.kgd = rx[5];
    for (int i = 0; i < 6; i++) info.eid[i] = rx[6 + i];
    info.ok = (info.kgd == 0x5D);

    if (info.ok) {
        // Enter QPI (0x35).
        qmi_hw->direct_csr |= QMI_DIRECT_CSR_ASSERT_CS1N_BITS;
        direct_xfer(0x35, NULL);
        qmi_hw->direct_csr &= ~QMI_DIRECT_CSR_ASSERT_CS1N_BITS;

        // tCEM max 8 us -> MAX_SELECT in units of 64 sys clocks, keep margin (6 us).
        uint32_t max_select = (sys / 1000000u * 6u) / 64u; if (max_select > 63) max_select = 63;
        // tCPH min 50 ns -> MIN_DESELECT in sys clocks.
        uint32_t min_desel = (sys / 1000000u * 50u + 999u) / 1000u + 1; if (min_desel > 31) min_desel = 31;
        uint32_t rxdelay = clkdiv / 2; if (rxdelay < 1) rxdelay = 1;
        qmi_hw->m[1].timing =
            (1u << QMI_M1_TIMING_COOLDOWN_LSB) |
            (QMI_M1_TIMING_PAGEBREAK_VALUE_1024 << QMI_M1_TIMING_PAGEBREAK_LSB) |
            (max_select << QMI_M1_TIMING_MAX_SELECT_LSB) |
            (min_desel << QMI_M1_TIMING_MIN_DESELECT_LSB) |
            (rxdelay << QMI_M1_TIMING_RXDELAY_LSB) |
            (clkdiv << QMI_M1_TIMING_CLKDIV_LSB);
        // Fast quad read 0xEB: 8-bit prefix, 24-bit address, 24 dummy clocks(6 quad cycles), all quad.
        qmi_hw->m[1].rfmt =
            (QMI_M1_RFMT_PREFIX_WIDTH_VALUE_Q << QMI_M1_RFMT_PREFIX_WIDTH_LSB) |
            (QMI_M1_RFMT_ADDR_WIDTH_VALUE_Q << QMI_M1_RFMT_ADDR_WIDTH_LSB) |
            (QMI_M1_RFMT_SUFFIX_WIDTH_VALUE_Q << QMI_M1_RFMT_SUFFIX_WIDTH_LSB) |
            (QMI_M1_RFMT_DUMMY_WIDTH_VALUE_Q << QMI_M1_RFMT_DUMMY_WIDTH_LSB) |
            (QMI_M1_RFMT_DATA_WIDTH_VALUE_Q << QMI_M1_RFMT_DATA_WIDTH_LSB) |
            (QMI_M1_RFMT_PREFIX_LEN_VALUE_8 << QMI_M1_RFMT_PREFIX_LEN_LSB) |
            (QMI_M1_RFMT_DUMMY_LEN_VALUE_24 << QMI_M1_RFMT_DUMMY_LEN_LSB);
        qmi_hw->m[1].rcmd = 0xEB;
        // Quad write 0x38, no dummy.
        qmi_hw->m[1].wfmt =
            (QMI_M1_WFMT_PREFIX_WIDTH_VALUE_Q << QMI_M1_WFMT_PREFIX_WIDTH_LSB) |
            (QMI_M1_WFMT_ADDR_WIDTH_VALUE_Q << QMI_M1_WFMT_ADDR_WIDTH_LSB) |
            (QMI_M1_WFMT_SUFFIX_WIDTH_VALUE_Q << QMI_M1_WFMT_SUFFIX_WIDTH_LSB) |
            (QMI_M1_WFMT_DUMMY_WIDTH_VALUE_Q << QMI_M1_WFMT_DUMMY_WIDTH_LSB) |
            (QMI_M1_WFMT_DATA_WIDTH_VALUE_Q << QMI_M1_WFMT_DATA_WIDTH_LSB) |
            (QMI_M1_WFMT_PREFIX_LEN_VALUE_8 << QMI_M1_WFMT_PREFIX_LEN_LSB);
        qmi_hw->m[1].wcmd = 0x38;
        info.sck_hz = sys / clkdiv;
    }
    uint32_t m1_timing = qmi_hw->m[1].timing, m1_rfmt = qmi_hw->m[1].rfmt, m1_rcmd = qmi_hw->m[1].rcmd, m1_wfmt = qmi_hw->m[1].wfmt, m1_wcmd = qmi_hw->m[1].wcmd;
    qmi_hw->direct_csr = 0;   // leave direct mode
    flush_cache(); enter_xip();                                  // flash back to (serial 03h) XIP; may reset the CS1 window config
    qmi_hw->m[1].timing = m1_timing; qmi_hw->m[1].rfmt = m1_rfmt; qmi_hw->m[1].rcmd = m1_rcmd; qmi_hw->m[1].wfmt = m1_wfmt; qmi_hw->m[1].wcmd = m1_wcmd;
    if (info.ok) xip_ctrl_hw->ctrl |= XIP_CTRL_WRITABLE_M1_BITS;
    restore_interrupts(save);
}

psram_info_t psram_init(unsigned clkdiv) {
    if (clkdiv < 2) clkdiv = 2;
    psram_setup(clkdiv, clock_get_hz(clk_sys));
    return info;
}
psram_info_t psram_info(void) { return info; }
