#pragma once
#include <stdint.h>
#include <stdbool.h>
// WS2812 status LED. The same ladder as vmpu68 / vfd68 / vhd68, highest priority first:
//   locate            white, 4 Hz blink, for the requested time
//   booting           white, solid (until the main loop starts)
//   disk/flash access red (120 ms after the last virtual floppy / shelf / firmware write)
//   Wi-Fi joining     blue, 2 Hz blink (first 60 s; then green until it connects)
//   Wi-Fi up          blue, solid
//   otherwise         green (no credentials, or the ESP32 is silent)
void led_init(void);
void led_set(uint8_t r, uint8_t g, uint8_t b);   // manual hold (console `led R G B`); led_auto() returns to the ladder
void led_auto(void);
void led_booted(void);
void led_note_disk(void);
void led_locate(uint32_t ms);
enum { LED_WIFI_UNKNOWN = 0, LED_WIFI_UNSET, LED_WIFI_JOINING, LED_WIFI_UP };
void led_set_wifi(int state);                    // from the ESP32 ("espstate" line)
void led_task(void);                             // call often (console loop)
void led_disable(void);                          // board 2.0: release GPIO45 and the PIO program (the ESP32 drives the LED)
void led_rgb(uint8_t out[3]);                    // current colour (sent to the ESP32 in every SPI frame)
const char *led_state_str(void);
