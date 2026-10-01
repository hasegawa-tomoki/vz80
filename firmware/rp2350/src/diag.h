#pragma once
#include <stdint.h>
#include <stdbool.h>
void diag_info(void);
bool diag_sram(void);
bool diag_psram(uint32_t bytes, bool nocache);
void diag_flash(void);
void diag_gpio(void);
float diag_temp(void);
