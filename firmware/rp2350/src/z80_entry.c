// vz80 Z80 core: entry points. MIT.
// z80_core_run() keeps the CPU state in r4-r11 as GCC global register variables, so it does not
// save those registers itself; this unit (compiled without the reservations) does.
#include "z80.h"

static inline void run_saved(z80_t *z, uint32_t until) {
    register uint32_t r0 asm("r0") = (uint32_t)z;
    register uint32_t r1 asm("r1") = until;
    asm volatile(
        "push {r4-r11}\n\t"
        "bl z80_core_run\n\t"
        "pop {r4-r11}"
        : "+r"(r0), "+r"(r1)
        :
        : "r2", "r3", "r12", "lr", "memory", "cc");
}

uint32_t z80_run(z80_t *z, uint32_t until, volatile uint8_t *stop) {
    (void)stop;   // chunks are short (a few hundred T); a stop request is honoured between them
    uint32_t start = z->cycles;
    while ((int32_t)(until - z->cycles) > 0) {
        uint32_t u = until;
        if ((int32_t)(u - z->cycles) > Z80_RUN_MAX) u = z->cycles + Z80_RUN_MAX;
        run_saved(z, u);
    }
    return z->cycles - start;
}

void z80_step(z80_t *z) { run_saved(z, z->cycles + 1); }
