#include <stdio.h>
#include <string.h>
#include "esp.h"
#include "hardware/watchdog.h"
#include "pins.h"
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "link.h"

#define ESP_UART uart1
#define ESP_BAUD 115200

void esp_init(void) {
    gpio_init(PIN_ESP_EN);  gpio_set_dir(PIN_ESP_EN, GPIO_IN);   // R11 pulls EN high; drive low only to reset
    gpio_init(PIN_ESP_IO9); gpio_set_dir(PIN_ESP_IO9, GPIO_IN);  // module pull-up; drive low only for download
    uart_init(ESP_UART, ESP_BAUD);
    gpio_set_function(PIN_UART_RX, GPIO_FUNC_UART);      // GPIO41: UART1 RX (F2)
    gpio_set_function(PIN_UART_TX, GPIO_FUNC_UART_AUX);  // GPIO42: UART1 TX (F11)
    uart_set_fifo_enabled(ESP_UART, true);
}

void esp_reset(bool download) {
    // Back to UART either way: the ESP32 re-detects the board when it comes up (2.0: SPI polling starts,
    // the CS edges switch us over again). In download mode IO9 (= LINK_CS on 2.0) is ours until esp_strap_release().
    if (download) link_dl_begin(); else link_set_mode(LINK_UART);
    link_note_esp_reset();
    if (download) { gpio_put(PIN_ESP_IO9, 0); gpio_set_dir(PIN_ESP_IO9, GPIO_OUT); }
    else gpio_set_dir(PIN_ESP_IO9, GPIO_IN);
    gpio_put(PIN_ESP_EN, 0); gpio_set_dir(PIN_ESP_EN, GPIO_OUT);
    sleep_ms(20);
    while (link_rx_getc() >= 0) ;
    gpio_set_dir(PIN_ESP_EN, GPIO_IN);
    // IO9 stays low until esp_strap_release(): the strap is only sampled at EN rise and the
    // caller wants to capture the ROM banner first.
}
void esp_strap_release(void) { gpio_set_dir(PIN_ESP_IO9, GPIO_IN); link_dl_end(); }

void esp_dump_log(unsigned wait_ms) {
    // Print lines as they arrive; collapse consecutive identical lines (the blank-flash ROM loops forever).
    static char line[160], last[160]; unsigned n = 0, repeat = 0;
    absolute_time_t end = make_timeout_time_ms(wait_ms);
    while (!time_reached(end)) {
        int ci = link_rx_getc(); if (ci < 0) continue;
        char c = (char)ci;
        if (c == '\r') continue;
        if (c != '\n' && n < sizeof line - 1) { line[n++] = (c >= 32 && c < 127) ? c : '?'; continue; }
        line[n] = 0;
        if (!strcmp(line, last)) repeat++;
        else { if (repeat) printf("  (x%u)\n", repeat + 1); repeat = 0; printf("%s\n", line); strcpy(last, line); }
        n = 0;
    }
    if (repeat) printf("  (x%u)\n", repeat + 1);
    line[0] = last[0] = 0;
}

void esp_bridge(unsigned idle_ms) {
    stdio_set_translate_crlf(&stdio_usb, false);
    absolute_time_t end = make_timeout_time_ms(idle_ms), esc_at = nil_time;
    int esc = 0;                          // three 0x1D bytes in a row close the bridge, but only if nothing follows them for
    while (!time_reached(end)) {          // 300 ms: a compressed esptool stream can contain 1D 1D 1D (it once closed the bridge
        int c = getchar_timeout_us(0);    // mid-write), and the closing sequence used to require silence *before* it, which the
        if (c != PICO_ERROR_TIMEOUT) {    // flashing tool never had (0.6.0 and earlier: the console looked dead until a re-plug)
            if (c == 0x1D && esc < 3) { if (++esc == 3) esc_at = get_absolute_time(); continue; }
            while (esc) { uart_putc_raw(ESP_UART, 0x1D); esc--; }   // not a close after all: pass the held bytes on
            uart_putc_raw(ESP_UART, (char)c); end = make_timeout_time_ms(idle_ms);
        } else if (esc == 3 && absolute_time_diff_us(esc_at, get_absolute_time()) > 300000) break;
        for (int ci; (ci = link_rx_getc()) >= 0; ) putchar_raw((char)ci);
        watchdog_update();                // the console loop's 8 s watchdog is not fed while we sit here (an esptool session takes a minute)
    }
    stdio_set_translate_crlf(&stdio_usb, true);
    printf("\nbridge closed\n");
}
