// Platform memory hooks for the Z80 core (RP2350 / MSX).
// 256-byte pages: msx_rd[p] / msx_wr[p] hold (backing - page_base) so that p[a] indexes with the
// full 16-bit address, or NULL to take the slow path (physical bus, MMIO, ROM fill).
#pragma once
#include <stdint.h>
#include <string.h>
extern uint8_t *msx_rd[256];
extern uint8_t *msx_wr[256];
uint8_t msx_rd_slow(uint16_t a);
uint8_t msx_fetch_slow(uint16_t a);
void msx_wr_slow(uint16_t a, uint8_t v);
uint8_t msx_io_rd(uint16_t port);
void msx_io_wr(uint16_t port, uint8_t v);
uint8_t msx_int_ack(uint16_t pc);

static inline uint8_t mem_rd(uint16_t a)   { const uint8_t *p = msx_rd[a >> 8]; return p ? p[a] : msx_rd_slow(a); }
static inline uint8_t fetch_op(uint16_t a) { const uint8_t *p = msx_rd[a >> 8]; return p ? p[a] : msx_fetch_slow(a); }
static inline void mem_wr(uint16_t a, uint8_t v) { uint8_t *p = msx_wr[a >> 8]; if (p) p[a] = v; else msx_wr_slow(a, v); }
// 16-bit access: one lookup and an unaligned halfword access when both bytes sit in the same page
// (the Cortex-M33 handles unaligned ldrh/strh); otherwise two byte accesses.
#define HAVE_MEM16 1
static inline uint16_t mem_rd16(uint16_t a) {
    const uint8_t *p = msx_rd[a >> 8];
    if (p && (uint8_t)a != 0xFF) { uint16_t v; memcpy(&v, p + a, 2); return v; }
    return (uint16_t)(mem_rd(a) | (mem_rd((uint16_t)(a + 1)) << 8));
}
static inline void mem_wr16(uint16_t a, uint16_t v) {
    uint8_t *p = msx_wr[a >> 8];
    if (p && (uint8_t)a != 0xFF) { memcpy(p + a, &v, 2); return; }
    mem_wr(a, (uint8_t)v); mem_wr((uint16_t)(a + 1), (uint8_t)(v >> 8));
}
static inline uint8_t io_rd(uint16_t port) { return msx_io_rd(port); }
static inline void io_wr(uint16_t port, uint8_t v) { msx_io_wr(port, v); }
static inline uint8_t int_ack(uint16_t pc) { return msx_int_ack(pc); }
