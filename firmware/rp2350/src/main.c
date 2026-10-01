// vz80 RP2350 firmware entry. Phase 0: bus parked, USB console, self-tests.
#include <stdio.h>
#include "pico/stdlib.h"
#include "pins.h"
#include "led.h"
#include "psram.h"
#include "console.h"
#include "esp.h"
#include "link.h"
#include "bus.h"
#include "msx.h"
#include "settings.h"
#include "shelf.h"
#include "vdpcmd.h"
#include "slot.h"
#include "audio.h"
#include "hardware/vreg.h"
#include "hardware/clocks.h"
#include "hardware/watchdog.h"
#include "crash.h"
#include "hardware/structs/watchdog.h"

// Disable every 5V-side buffer before anything else, then leave all Z80 signals as
// high-impedance inputs. This holds even if the board is plugged into a live MSX.
static void bus_park(void) {
    gpio_init(PIN_BUS_EN);   gpio_put(PIN_BUS_EN, 1);   gpio_set_dir(PIN_BUS_EN, GPIO_OUT);
    gpio_init(PIN_DATA_OE);  gpio_put(PIN_DATA_OE, 1);  gpio_set_dir(PIN_DATA_OE, GPIO_OUT);
    gpio_init(PIN_DATA_DIR); gpio_put(PIN_DATA_DIR, 0); gpio_set_dir(PIN_DATA_DIR, GPIO_OUT);
    for (uint p = PIN_MA0; p <= PIN_MRFSH; p++) { gpio_init(p); gpio_disable_pulls(p); }
    for (uint p = PIN_NMI; p <= PIN_RESET; p++) { gpio_init(p); gpio_disable_pulls(p); }
    // /HALT and /BUSAK are outputs (through U3): hold them inactive so the MSX never sees a bus grant.
    gpio_put(PIN_MHALT, 1); gpio_set_dir(PIN_MHALT, GPIO_OUT);
    gpio_put(PIN_MBUSAK, 1); gpio_set_dir(PIN_MBUSAK, GPIO_OUT);
    gpio_pull_up(PIN_RESET); gpio_pull_up(PIN_INT);   // inactive when nothing drives them (bench)
}

// Overclock: the requested clock is applied only if the previous attempt booted through (clk_pending
// cleared by the console loop after 10 s); a boot that never got that far falls back to 150 MHz.
static void apply_sys_clock(void) {
    settings_load(); shelf_init();
    uint32_t mhz = settings.sys_mhz;
    if (mhz != 200 && mhz != 250 && mhz != 300) return;
    if (settings.clk_pending) { settings.sys_mhz = 150; settings.clk_pending = 0; settings_save(); return; }
    settings.clk_pending = 1; settings_save();
    vreg_set_voltage(mhz >= 300 ? VREG_VOLTAGE_1_30 : mhz >= 250 ? VREG_VOLTAGE_1_20 : VREG_VOLTAGE_1_15);
    sleep_ms(2);
    set_sys_clock_khz(mhz * 1000, true);
}
void sysclock_confirm(void) {   // called by the console loop once the firmware has been running for a while
    if (settings.clk_pending) { settings.clk_pending = 0; settings_save(); }
}
int main(void) {
    bus_park();
    apply_sys_clock();
    crash_init();                // read (and clear) a crash record left by the previous run
    stdio_init_all();            // before the slot code: the XIP cache flush there breaks USB stdio if done first
    slot_init();                 // decides A/B; a pending trial is launched later by slot_launch_if_pending()
    esp_init();
    bus_init();
    settings_load();
    link_init();                 // after bus_init: the SPI slave (board 2.0) shares PIO1 with the bus data engine
    led_init();                  // shares PIO1 with the bus data engine: after bus_init (dropped by mistake in 0.3.9); no-op on 2.0
    if (board_kind() == 2) audio_init();
    msx_init();
    // PSRAM (APS6404L) is reliable up to ~67 MHz here: 150 -> /3 (50), 200 -> /3 (67), 250 -> /4 (62.5)
    unsigned pdiv = (clock_get_hz(clk_sys) + 66999999u) / 67000000u; if (pdiv < 3) pdiv = 3;
    psram_info_t pi = psram_init(pdiv);
    if (!pi.ok) led_set(16, 0, 0);   // PSRAM missing: hold red
    vdpcmd_init();               // core 1: VDP command engine on the VRAM shadow
    slot_launch_if_pending();    // trial image in slot B: jump into it now (never returns)
    printf("\nvz80 %s\n> ", VZ80_VERSION);
    if (crash_last()[0]) printf("last run ended with: %s\n", crash_last());
    console_run();
}
