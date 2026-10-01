#pragma once
#include <stdint.h>
#include <stdbool.h>
// V9958 observer: tracks registers, palette and the VRAM address from the CPU's port accesses
// and keeps a 128 KiB VRAM shadow (in PSRAM) so the screen can be rendered remotely.
// Limitation: VRAM changed by VDP commands (LINE, HMMC...) is not reflected until vdp_snapshot().
#define VDP_VRAM_SIZE   0x20000u
#define VDP_BLOCK       512u
#define VDP_BLOCKS      (VDP_VRAM_SIZE / VDP_BLOCK)
// 0 = display off, 1 = text mode (T1/T2), 2 = graphic/multicolor: drives the CPU access pacing
extern volatile uint8_t vdp_display_class;
void vdp_init(void);
void vdp_reset(void);
void vdp_observe_write(uint8_t port, uint8_t v);
void vdp_observe_read(uint8_t port, uint8_t v);
int vdp_screen_text(char *out, int outsz, int *cols);
uint8_t vdp_reg(int r);
uint16_t vdp_palette(int i);              // 0x0GRB? no: bits 0-2 B, 4-6 R, 8-10 G (V9938 order R,B / G)
uint32_t vdp_vram_addr(void);
const uint8_t *vdp_vram(void);            // shadow base
bool vdp_block_valid(unsigned b);
// Dirty tracking for delta transfers: returns and clears the dirty bitmap (VDP_BLOCKS bits).
void vdp_take_dirty(uint32_t *bits);
void vdp_mark_all_dirty(void);
void vdp_mark_dirty(unsigned b);
void vdp_dump(uint32_t addr, unsigned n);
void vdp_dump_regs(void);
// Read the whole VRAM back from the real VDP through port 98 (CPU must be stopped): ~250 ms.
bool vdp_snapshot(unsigned gap_us);   // gap between port 98 reads (2 us is too fast for the V9938: garbage)
