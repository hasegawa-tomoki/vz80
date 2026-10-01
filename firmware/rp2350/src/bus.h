#pragma once
#include <stdbool.h>
#include <stdint.h>
// Physical Z80 bus engine (PIO0 ctrl+addr, PIO1 data). Writes and refreshes are posted;
// reads block until the data SM delivers the byte. A stuck /WAIT or missing CLK sets the
// fault flag after ~2 ms and the engine is reset with all strobes released.
void bus_init(void);
void bus_enable(bool on);          // hand the pins to the PIO and enable the 5 V buffers (or park)
bool bus_enabled(void);
bool bus_faulted(void);
void bus_clear_fault(void);
uint8_t bus_mem_read(uint16_t a);
void bus_mem_write(uint16_t a, uint8_t d);
uint8_t bus_io_read(uint16_t a);
void bus_io_write(uint16_t a, uint8_t d);
uint8_t bus_m1(uint16_t a);        // opcode fetch (memory read with /M1)
void bus_refresh(uint8_t r);
uint8_t bus_int_ack(uint16_t pc);
void bus_drain(void);              // wait until every posted transfer has left the FIFOs
// Bench helpers (USB-only board): local 3.58 MHz CLK on the CLK pin, pull-based data injection.
void bus_bench_clock(bool on);
void bus_bench_data(uint8_t pattern);
// Passive capture of GPIO0-31 (PIO2) into buf via DMA; stop returns the number of samples.
void bus_capture_div(uint32_t d);    // sample clock divider (2 = 75 MS/s at 150 MHz)
void bus_capture_start(uint32_t *buf, uint32_t n, uint32_t *sample_hz);
uint32_t bus_capture_stop(void);
