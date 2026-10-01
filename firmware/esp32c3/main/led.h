#pragma once
#include <stdint.h>
#include <stdbool.h>
void led_init(void);                          // RMT on IO3 (board 2.0 LED)
void led_set(uint8_t r, uint8_t g, uint8_t b);
bool led_ready(void);
