#pragma once
#include <stdbool.h>
#include <stdint.h>
// ESP32-C3 link over UART1 (PIN_UART_TX/RX), EN and IO9 strap control.
void esp_init(void);
// Reset the ESP32. download=true holds IO9 low so the ROM enters UART download mode.
void esp_reset(bool download);
void esp_strap_release(void);
// Print whatever the ESP has sent since the last call (for up to wait_ms).
void esp_dump_log(unsigned wait_ms);
// Transparent USB<->UART bridge for esptool. Returns after idle_ms without USB traffic.
void esp_bridge(unsigned idle_ms);
