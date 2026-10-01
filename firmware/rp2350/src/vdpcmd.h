#pragma once
#include <stdint.h>
#include <stdbool.h>
// V9938/V9958 command engine emulation on core 1, applied to the VRAM shadow only (the real VDP does
// the real work). Core 0 feeds it every write to R#0/R#1/R#25 and R#32..R#46; core 1 executes PSET,
// LINE, LMMV, LMMM, LMMC, HMMV, HMMM, YMMM and HMMC on the shadow and marks the dirty blocks.
void vdpcmd_init(void);                 // launch core 1
void vdpcmd_push(uint8_t reg, uint8_t v);
void vdpcmd_reset(void);                // drop the queue and the pending command (CPU reset)
void vdpcmd_drain(void);                // wait until core 1 has consumed everything queued so far
void vdpcmd_take_dirty(uint32_t *bits, unsigned words);   // OR core 1's dirty bits into bits[], clearing them
typedef struct { uint32_t executed, dropped, unsupported, queue_max; uint8_t last_cmd; bool busy; } vdpcmd_stats_t;
void vdpcmd_stats(vdpcmd_stats_t *s);
typedef struct { uint32_t t_us; uint16_t sx, sy, dx, dy, nx, ny; uint8_t clr, arg, cmd, mode; } vdpcmd_log_t;
unsigned vdpcmd_log(vdpcmd_log_t *out, unsigned max);   // most recent commands, oldest first
