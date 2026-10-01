#pragma once
#include <stdint.h>
#include <stdbool.h>
// Receive a UF2 stream of `total` bytes from the ESP link in 4096-byte pieces (each acknowledged
// with "ok"), program it into flash and reboot. The whole firmware runs from SRAM, so flash
// can be rewritten while running. The first sector is written last.
void fwup_receive(uint32_t total, bool to_b);   // to_b: write slot B and boot it once (trial)
