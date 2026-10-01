// Board self-tests: identity, clocks, SRAM, flash JEDEC ID, PSRAM, bus-pin snapshot.
#include <stdio.h>
#include <string.h>
#include "diag.h"
#include "pins.h"
#include "psram.h"
#include "pico/stdlib.h"
#include "pico/unique_id.h"
#include "hardware/clocks.h"
#include "hardware/flash.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/structs/qmi.h"

void diag_info(void) {
    pico_unique_board_id_t id; pico_get_unique_board_id(&id);
    printf("fw=%s chipid=", VZ80_VERSION);
    for (int i = 0; i < PICO_UNIQUE_BOARD_ID_SIZE_BYTES; i++) printf("%02x", id.id[i]);
    printf(" sys=%luHz usb=%luHz ref=%luHz", clock_get_hz(clk_sys), clock_get_hz(clk_usb), clock_get_hz(clk_ref));
    printf(" flash_clkdiv=%lu psram_clkdiv=%lu\n",
           qmi_hw->m[0].timing & 0xff, qmi_hw->m[1].timing & 0xff);
    printf("temp=%.1fC (uncalibrated)\n", diag_temp());
}
float diag_temp(void) {
    static bool init;
    if (!init) { adc_init(); adc_set_temp_sensor_enabled(true); init = true; }
    adc_select_input(8);  // RP2350B: temp = channel 8
    float v = adc_read() * 3.3f / 4096.0f;
    return 27.0f - (v - 0.706f) / 0.001721f;
}

// Address-dependent patterns over the 64 KiB MSX RAM buffer (only while the Z80 is stopped: SRAM has no
// room for a dedicated test window since the board 2.0 link buffers).
#include "msx.h"
extern uint8_t msx_ram[65536];
bool diag_sram(void) {
    if (msx.running) { printf("err stop first (the test uses the MSX RAM buffer)\n"); return false; }
    static const uint32_t pat[4] = {0x00000000u, 0xFFFFFFFFu, 0xA5A55A5Au, 0x12345678u};
    uint32_t *scratch = (uint32_t *)msx_ram; unsigned errors = 0;
    for (int p = 0; p < 4; p++) {
        for (uint32_t i = 0; i < 16384; i++) scratch[i] = pat[p] ^ (i * 0x9E3779B1u);
        for (uint32_t i = 0; i < 16384; i++) if (scratch[i] != (pat[p] ^ (i * 0x9E3779B1u))) errors++;
    }
    printf("sram: 64KiB x4 patterns, errors=%u (MSX RAM buffer: restart the MSX afterwards)\n", errors);
    return errors == 0;
}

bool diag_psram(uint32_t bytes, bool nocache) {
    psram_info_t pi = psram_info();
    printf("psram: mfid=%02x kgd=%02x eid=%02x%02x%02x%02x%02x%02x ok=%d sck=%luHz\n",
           pi.mfid, pi.kgd, pi.eid[0], pi.eid[1], pi.eid[2], pi.eid[3], pi.eid[4], pi.eid[5], pi.ok, pi.sck_hz);
    if (!pi.ok) return false;
    if (bytes == 0 || bytes > PSRAM_SIZE) bytes = PSRAM_SIZE;
    volatile uint32_t *m = (volatile uint32_t *)(nocache ? PSRAM_NOCACHE_BASE : PSRAM_BASE);
    uint32_t words = bytes / 4, errors = 0;
    absolute_time_t t0 = get_absolute_time();
    for (uint32_t i = 0; i < words; i++) m[i] = i * 0x9E3779B1u;
    absolute_time_t t1 = get_absolute_time();
    for (uint32_t i = 0; i < words; i++) if (m[i] != i * 0x9E3779B1u) { if (errors < 4) printf("  mismatch @%08lx got %08lx\n", (unsigned long)(i*4), m[i]); errors++; }
    absolute_time_t t2 = get_absolute_time();
    for (uint32_t i = 0; i < words; i++) m[i] = ~(i * 0x9E3779B1u);
    for (uint32_t i = 0; i < words; i++) if (m[i] != ~(i * 0x9E3779B1u)) errors++;
    int64_t wus = absolute_time_diff_us(t0, t1), rus = absolute_time_diff_us(t1, t2);
    printf("psram: %lu bytes %s, errors=%lu, write %.1f MB/s read %.1f MB/s\n",
           (unsigned long)bytes, nocache ? "nocache" : "cached", (unsigned long)errors,
           bytes / (double)wus, bytes / (double)rus);
    return errors == 0;
}

void diag_flash(void) {
    uint8_t tx[4] = {0x9F, 0, 0, 0}, rx[4];
    flash_do_cmd(tx, rx, 4);
    printf("flash: jedec=%02x %02x %02x (W25Q512JV expects ef 40 20)\n", rx[1], rx[2], rx[3]);
}

void diag_gpio(void) {
    // Snapshot of the Z80-side inputs. With buffers disabled these float unless the MSX drives them.
    uint32_t lo = gpio_get_all();
    uint32_t hi = gpio_get_all64() >> 32;
    printf("gpio: A=%04lx D=%02lx CLK=%lu MREQ=%lu IORQ=%lu RD=%lu WR=%lu WAIT=%lu M1=%lu RFSH=%lu\n",
           lo & 0xffff, (lo >> 16) & 0xff, (lo >> PIN_CLK) & 1, (lo >> PIN_MMREQ) & 1, (lo >> PIN_MIORQ) & 1,
           (lo >> PIN_MRD) & 1, (lo >> PIN_MWR) & 1, (lo >> PIN_WAIT) & 1, (lo >> PIN_MM1) & 1, (lo >> PIN_MRFSH) & 1);
    printf("      NMI=%lu INT=%lu HALT=%lu BUSAK=%lu BUSRQ=%lu RESET=%lu DATA_DIR=%lu DATA_OE=%lu BUS_EN=%lu ESP_EN=%lu IO9=%lu\n",
           (hi >> (PIN_NMI-32)) & 1, (hi >> (PIN_INT-32)) & 1, (hi >> (PIN_MHALT-32)) & 1, (hi >> (PIN_MBUSAK-32)) & 1,
           (hi >> (PIN_BUSRQ-32)) & 1, (hi >> (PIN_RESET-32)) & 1, (hi >> (PIN_DATA_DIR-32)) & 1, (hi >> (PIN_DATA_OE-32)) & 1,
           (hi >> (PIN_BUS_EN-32)) & 1, (hi >> (PIN_ESP_EN-32)) & 1, (hi >> (PIN_ESP_IO9-32)) & 1);
}
