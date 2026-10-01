#pragma once
#include <stdint.h>
#include <stdbool.h>
// Virtual keyboard: injects key presses into the PPI keyboard matrix as the BIOS scans it.
void kbd_init(void);
// Queue ASCII text (Japanese JIS layout). Returns false if a character has no key or the queue is full.
bool kbd_type(const char *s);
void kbd_cancel(void);
int kbd_pending(void);
// Called from the I/O layer.
void kbd_row_select(uint8_t row);        // PPI port AA low nibble
uint8_t kbd_row_mask(void);              // bits to force low in the port A9 read for the selected row
