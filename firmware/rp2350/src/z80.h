// vz80 Z80 core. MIT.
// Instruction-level interpreter tuned for Cortex-M33: memory and I/O go through the
// static inline hooks in z80_mem.h (provided by the platform), so the fast paths inline.
#pragma once
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint16_t pc, sp, ix, iy, wz;
    // pairs are unions so that 16-bit operations are single loads/stores (little-endian: low byte first)
    union { struct { uint8_t f, a; }; uint16_t af; };
    union { struct { uint8_t c, b; }; uint16_t bc; };
    union { struct { uint8_t e, d; }; uint16_t de; };
    union { struct { uint8_t l, h; }; uint16_t hl; };
    uint8_t a2, f2, b2, c2, d2, e2, h2, l2;   // alternate set
    uint8_t i, r, im;
    uint8_t iff1, iff2;
    uint8_t halted;
    uint8_t ei_delay;       // EI executed: interrupts enabled after the next instruction
    uint8_t q;              // flags written by the last executed instruction (0 if it left F alone); SCF/CCF look at it
    uint8_t int_line;       // level of /INT (1 = asserted); host tests set this directly
    uint8_t nmi_pending;    // edge latched by the platform
    uint8_t after_ld_ai;    // previous instruction was LD A,I / LD A,R (NMOS P/V quirk)
    uint32_t cycles;        // T-states, free-running
    uint32_t insns;         // M1 cycles retired (instructions + prefix bytes): what the MSX adds a wait state to
} z80_t;

void z80_reset(z80_t *z);
// Longest single core run (T-states); z80_run splits longer requests.
#define Z80_RUN_MAX 16000
// The interpreter proper; on the target it clobbers r4-r11 and must be called through z80_run/z80_step.
void z80_core_run(z80_t *z, uint32_t until);
extern uint16_t z80_io_pc;
// Debug trace: when z80_trace_on, every instruction start records PC/SP in a ring; a break condition
// (PC inside [lo, hi], or SP below spmin) stops the core (z80_brk_hit) after recording that entry.
#define Z80_TRACE_N 2048
extern uint8_t z80_trace_on, z80_brk_hit;
extern uint16_t *const z80_trace_pc, *const z80_trace_sp;   // Z80_TRACE_N entries each, in PSRAM
extern uint32_t z80_trace_i;
extern uint16_t z80_brk_lo, z80_brk_hi, z80_brk_spmin;
// Execute instructions until z->cycles >= until or *stop becomes nonzero. Returns cycles run.
uint32_t z80_run(z80_t *z, uint32_t until, volatile uint8_t *stop);
// Execute exactly one instruction (used by the host test harness).
void z80_step(z80_t *z);
