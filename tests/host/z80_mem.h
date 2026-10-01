// Host test memory model: flat 64 KiB, port table for the SingleStepTests harness.
#pragma once
#include <stdint.h>
extern uint8_t host_mem[65536];
extern uint8_t host_port_in[65536];
extern uint32_t host_io_log[64][2]; extern int host_io_n;
static inline uint8_t mem_rd(uint16_t a) { return host_mem[a]; }
static inline uint8_t fetch_op(uint16_t a) { return host_mem[a]; }
static inline void mem_wr(uint16_t a, uint8_t v) { host_mem[a] = v; }
static inline uint8_t io_rd(uint16_t p) { if (host_io_n < 64) { host_io_log[host_io_n][0] = p; host_io_log[host_io_n][1] = 0x100 | host_port_in[p]; host_io_n++; } return host_port_in[p]; }
static inline void io_wr(uint16_t p, uint8_t v) { if (host_io_n < 64) { host_io_log[host_io_n][0] = p; host_io_log[host_io_n][1] = 0x200 | v; host_io_n++; } }
static inline uint8_t int_ack(uint16_t pc) { (void)pc; return 0xFF; }
