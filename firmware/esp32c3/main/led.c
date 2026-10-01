// Board 2.0 status LED: WS2812B-2020 on IO3 driven by RMT (on board 1.0 IO3 is unconnected; the RP2350
// drives the LED there). The colour comes from the RP2350 in every SPI frame header.
#include <string.h>
#include "led.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#define LED_GPIO 3
static rmt_channel_handle_t chan; static rmt_encoder_handle_t enc; static bool ok; static uint8_t last[3] = {255, 255, 255};

void led_init(void) {
    rmt_tx_channel_config_t c = { .gpio_num = LED_GPIO, .clk_src = RMT_CLK_SRC_DEFAULT, .resolution_hz = 10000000, .mem_block_symbols = 64, .trans_queue_depth = 2 };
    if (rmt_new_tx_channel(&c, &chan) != ESP_OK) return;
    rmt_bytes_encoder_config_t e = {
        .bit0 = { .level0 = 1, .duration0 = 3, .level1 = 0, .duration1 = 9 },   // 0.3 us high, 0.9 us low
        .bit1 = { .level0 = 1, .duration0 = 9, .level1 = 0, .duration1 = 3 },   // 0.9 us high, 0.3 us low
        .flags.msb_first = 1 };
    if (rmt_new_bytes_encoder(&e, &enc) != ESP_OK) return;
    if (rmt_enable(chan) != ESP_OK) return;
    ok = true;
    led_set(8, 8, 8);
}
void led_set(uint8_t r, uint8_t g, uint8_t b) {
    if (!ok) return;
    if (r == last[0] && g == last[1] && b == last[2]) return;
    uint8_t grb[3] = { g, r, b };
    rmt_transmit_config_t t = { .loop_count = 0 };
    if (rmt_transmit(chan, enc, grb, sizeof grb, &t) == ESP_OK) { rmt_tx_wait_all_done(chan, pdMS_TO_TICKS(20)); last[0] = r; last[1] = g; last[2] = b; }
}
bool led_ready(void) { return ok; }
