#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
// After any flash erase/program the SDK re-runs boot2, which leaves the flash in quad continuous-read
// mode at a clock that this board (W25Q512JV next to the PSRAM on the same QSPI lines, inside a warm
// MSX) has been seen to read back corrupted. Code runs from SRAM, so put XIP back into the bootrom's
// plain serial command mode instead: slow but reliable for settings, verification and dumps.
void flash_xip_safe(void);
// Erase and program one 4 KiB sector at flash offset `off` (core 1 locked out, XIP restored), verify.
bool flash_program_sector(uint32_t off, const uint8_t *data);
uint32_t crc32_buf(const void *p, size_t n);
// Bulk writes: erase `len` bytes (4 KiB multiples; 64 KiB-aligned runs use block erase) / program pre-erased flash.
void flash_erase(uint32_t off, uint32_t len);
bool flash_program(uint32_t off, const uint8_t *data, uint32_t len);
