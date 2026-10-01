#pragma once
#include <stdint.h>
#include <stdbool.h>
// "vz80 disk": the virtual floppy images exposed on the MSX device-switched I/O ports for a machine-
// independent DOS driver (the Nextor kernel in tools/nextor). Port 40h selects the device by ID; while
// selected, ports 41h-48h are ours (see vzdisk.c for the register map).
#define VZDISK_ID 0xC8
extern uint32_t vzdisk_reads, vzdisk_writes, vzdisk_errors, vzdisk_selects, vzdisk_regs;
void vzdisk_log_print(void);                   // console: recent register accesses
bool vzdisk_io_wr(uint8_t port, uint8_t v);   // true = consumed (do not send to the bus)
bool vzdisk_io_rd(uint8_t port, uint8_t *v);  // true = value supplied
