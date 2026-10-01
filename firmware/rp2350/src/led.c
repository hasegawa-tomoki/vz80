// WS2812B-2020 status LED on PIN_LED_DATA, driven by PIO1 (gpio base 16).
#include "led.h"
#include "pins.h"
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "ws2812.pio.h"
#include "link.h"

static PIO pio;
static uint sm, off; static bool pio_ok;
static uint32_t cur = 0xFFFFFFFFu;
static bool manual, booted;
static uint32_t disk_ms, locate_until, wifi_ms;
static int wifi_state;
static const char *state_name = "boot";

// The colour is sent only when it changes, plus a refresh every second: a WS2812 that misread one
// frame (the user saw the LED go dark while the firmware thought it was blue) shows the right colour again.
static uint32_t sent_ms;
static void put(uint8_t r, uint8_t g, uint8_t b) {
    uint32_t v = ((uint32_t)g << 24) | ((uint32_t)r << 16) | ((uint32_t)b << 8);
    uint32_t now = to_ms_since_boot(get_absolute_time());
    if (v == cur && now - sent_ms < 1000) return;
    cur = v; sent_ms = now; if (pio_ok) pio_sm_put_blocking(pio, sm, v);
}
void led_rgb(uint8_t out[3]) { uint32_t v = cur == 0xFFFFFFFFu ? 0 : cur; out[0] = (uint8_t)(v >> 16); out[1] = (uint8_t)(v >> 24); out[2] = (uint8_t)(v >> 8); }
void led_disable(void) {
    if (!pio_ok) return;
    pio_sm_set_enabled(pio, sm, false); pio_remove_program(pio, &ws2812_program, off); pio_sm_unclaim(pio, sm); pio_ok = false;
    gpio_init(PIN_LED_DATA);   // back to a plain input until audio_init takes it
}
void led_init(void) {
    if (board_kind() == 2) { cur = 0x08080800u; return; }   // 2.0: GPIO45 is the audio output; the colour goes to the ESP32 in the SPI header
    pio = pio1;
    if (pio_get_gpio_base(pio) != PIO_DATA_GPIOBASE) pio_set_gpio_base(pio, PIO_DATA_GPIOBASE);   // bus_init already set it (its SM is running)
    off = pio_add_program(pio, &ws2812_program);
    sm = pio_claim_unused_sm(pio, true);
    pio_gpio_init(pio, PIN_LED_DATA);
    pio_sm_set_consecutive_pindirs(pio, sm, PIN_LED_DATA, 1, true);
    pio_sm_config c = ws2812_program_get_default_config(off);
    sm_config_set_sideset_pins(&c, PIN_LED_DATA);
    sm_config_set_out_shift(&c, false, true, 24);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
    sm_config_set_clkdiv(&c, (float)clock_get_hz(clk_sys) / 8000000.0f);
    pio_sm_init(pio, sm, off, &c);
    pio_sm_set_enabled(pio, sm, true); pio_ok = true;
    put(8, 8, 8);   // booting: white
}
void led_set(uint8_t r, uint8_t g, uint8_t b) { manual = true; state_name = "manual"; put(r, g, b); }
void led_auto(void) { manual = false; }
void led_booted(void) { booted = true; }
void led_note_disk(void) { disk_ms = to_ms_since_boot(get_absolute_time()) | 1; }
void led_locate(uint32_t ms) { locate_until = (to_ms_since_boot(get_absolute_time()) + ms) | 1; }
void led_set_wifi(int state) { wifi_state = state; wifi_ms = to_ms_since_boot(get_absolute_time()) | 1; }
const char *led_state_str(void) { return state_name; }

#define WIFI_GIVEUP_MS 60000   // credentials set but no connection for 60 s: green (back to blue whenever it connects)
#define WIFI_STALE_MS  30000   // no word from the ESP32 for 30 s: treat as unknown
void led_task(void) {
    uint32_t now = to_ms_since_boot(get_absolute_time());
    if (locate_until) {
        if ((int32_t)(now - locate_until) < 0) { state_name = "locate"; if ((now / 125) & 1) put(40, 40, 40); else put(0, 0, 0); return; }
        locate_until = 0;
    }
    if (manual) return;
    if (!booted) { state_name = "boot"; put(8, 8, 8); return; }
    if (disk_ms && now - disk_ms < 120) { state_name = "disk"; put(16, 0, 0); return; }
    int w = (wifi_ms && now - wifi_ms < WIFI_STALE_MS) ? wifi_state : LED_WIFI_UNKNOWN;
    if (w == LED_WIFI_JOINING) {
        if (now < WIFI_GIVEUP_MS) { state_name = "wifi-joining"; put(0, 0, (now / 250) & 1 ? 8 : 0); }
        else { state_name = "wifi-giveup"; put(0, 8, 0); }
        return;
    }
    if (w == LED_WIFI_UP) { state_name = "wifi-up"; put(0, 0, 8); return; }
    state_name = w == LED_WIFI_UNSET ? "wifi-unset" : "no-esp";
    put(0, 8, 0);
}
