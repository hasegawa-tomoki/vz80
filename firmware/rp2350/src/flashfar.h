#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
// The part of the W25Q512JV beyond the 16 MB XIP window (16-64 MB). Reached with 4-byte-address
// commands over the QMI in direct mode (13h read, 12h page program, DCh 64 KB erase), so nothing here
// is memory-mapped: use these calls. Each call parks core 1 and runs with interrupts off (like the
// XIP flash writes), then puts XIP back with flash_xip_safe().
#define FAR_BASE  0x1000000u
#define FAR_END   0x4000000u
#define FAR_BLOCK 0x10000u
bool far_read(uint32_t addr, uint8_t *buf, uint32_t n);           // any addr/len inside the far region
bool far_program(uint32_t addr, const uint8_t *data, uint32_t n); // 256-byte aligned addr and n
bool far_erase(uint32_t addr, uint32_t n);                        // 64 KB aligned
bool far_erase_sector(uint32_t addr);                             // one 4 KB sector
