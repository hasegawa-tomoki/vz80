#include <string.h>
#include <stdio.h>
#include "vdp.h"
#include "vdpcmd.h"
#include "z80_mem.h"
#include "bus.h"
#include "psram.h"
#include "pico/stdlib.h"

// VRAM shadow lives in the top of PSRAM (cached window).
#define VRAM_SHADOW ((uint8_t *)(PSRAM_BASE + PSRAM_SIZE - 0x40000))
static uint8_t *vram = VRAM_SHADOW;
static uint32_t block_valid[VDP_BLOCKS / 32], block_dirty[VDP_BLOCKS / 32];
static uint8_t regs[64];
static uint16_t palette[16];
static uint32_t addr;          // 17-bit VRAM address (A16..A14 from R#14)
static bool latch;             // second byte of a port 99 pair pending
static uint8_t latch_lo;
static uint8_t indirect;       // R#17: indirect register pointer
static uint8_t pal_ptr; static bool pal_second; static uint8_t pal_lo;
static bool cmd_seen;

static inline void touch(uint32_t a) { unsigned b = a >> 9; block_valid[b >> 5] |= 1u << (b & 31); block_dirty[b >> 5] |= 1u << (b & 31); }
static inline bool valid(uint32_t a) { unsigned b = a >> 9; return block_valid[b >> 5] >> (b & 31) & 1; }
static inline void step_addr(void) { addr = (addr + 1) & (VDP_VRAM_SIZE - 1); regs[14] = (uint8_t)(addr >> 14); }   // the counter carries into A14-A16 (verified: a port 98 stream crosses 3FFF->4000 on the real chip)

static const uint16_t default_palette[16] = {   // V9938 defaults, 0RRR0BBB0GGG? stored as (G<<8)|(R<<4)|B
    0x000, 0x000, 0x611, 0x733, 0x117, 0x327, 0x151, 0x627, 0x171, 0x373, 0x661, 0x664, 0x411, 0x265, 0x555, 0x777 };
// Actual V9938 default palette (R,G,B 0-7): 0:0,0,0 1:0,0,0 2:1,6,1 3:3,7,3 4:1,1,7 5:2,3,7 6:5,1,1 7:2,6,7 8:7,1,1 9:7,3,3 10:6,6,1 11:6,6,4 12:1,4,1 13:6,2,5 14:5,5,5 15:7,7,7
static void load_default_palette(void) {
    static const uint8_t rgb[16][3] = {{0,0,0},{0,0,0},{1,6,1},{3,7,3},{1,1,7},{2,3,7},{5,1,1},{2,6,7},{7,1,1},{7,3,3},{6,6,1},{6,6,4},{1,4,1},{6,2,5},{5,5,5},{7,7,7}};
    for (int i = 0; i < 16; i++) palette[i] = (uint16_t)((rgb[i][1] << 8) | (rgb[i][0] << 4) | rgb[i][2]);
    (void)default_palette;
}

void vdp_init(void) { vdp_reset(); memset(vram, 0, VDP_VRAM_SIZE); memset(block_valid, 0, sizeof block_valid); }
void vdp_reset(void) {
    memset(regs, 0, sizeof regs); addr = 0; latch = false; indirect = 0; cmd_seen = false; pal_ptr = 0; pal_second = false; vdp_display_class = 2;
    load_default_palette();
    memset(block_dirty, 0xFF, sizeof block_dirty);
    vdpcmd_reset();
}

volatile uint8_t vdp_display_class = 2;
static void classify(void) {
    if (!(regs[1] & 0x40)) vdp_display_class = 0;                 // BL = 0: display off
    else vdp_display_class = (regs[1] & 0x10) ? 1 : 2;            // M1: TEXT1/TEXT2
}
static void set_reg(uint8_t r, uint8_t v) {
    r &= 0x3F; regs[r] = v;
    if (r == 0 || r == 1) classify();
    if (r == 14) addr = (addr & 0x3FFF) | ((uint32_t)(v & 7) << 14);
    if (r == 16) { pal_ptr = v & 15; pal_second = false; }
    if (r == 17) indirect = v;
    if (r == 0 || r == 1 || r == 25 || (r >= 32 && r <= 46)) vdpcmd_push(r, v);   // command engine on core 1
}

void vdp_observe_write(uint8_t port, uint8_t v) {
    switch (port) {
        case 0x98: vram[addr] = v; touch(addr); step_addr(); latch = false; break;
        case 0x99:
            if (!latch) { latch_lo = v; latch = true; }
            else {
                latch = false;
                if (v & 0x80) set_reg(v & 0x3F, latch_lo);
                else addr = (addr & 0x1C000) | ((uint32_t)(v & 0x3F) << 8) | latch_lo;
            }
            break;
        case 0x9A:
            if (!pal_second) { pal_lo = v; pal_second = true; }
            else { pal_second = false; palette[pal_ptr] = (uint16_t)(((v & 7) << 8) | (pal_lo & 0x77)); pal_ptr = (pal_ptr + 1) & 15; block_dirty[0] |= 1; }
            latch = false; break;
        case 0x9B:
            // V9938: R#17 cannot be written through the indirect port; the BIOS relies on this when it
            // OTIRs R#8..R#23 through port 9B (the byte for R#17 is skipped, the pointer keeps counting).
            if ((indirect & 0x3F) != 17) set_reg(indirect & 0x3F, v);
            if (!(indirect & 0x80)) indirect = (uint8_t)((indirect & 0x80) | ((indirect + 1) & 0x3F));
            latch = false; break;
    }
}

void vdp_observe_read(uint8_t port, uint8_t v) {
    if (port == 0x98) { vram[addr] = v; touch(addr); step_addr(); latch = false; }
    else if (port == 0x99) latch = false;     // status read resets the address latch
}

uint8_t vdp_reg(int r) { return regs[r & 0x3F]; }
uint16_t vdp_palette(int i) { return palette[i & 15]; }
uint32_t vdp_vram_addr(void) { return addr; }
const uint8_t *vdp_vram(void) { return vram; }
bool vdp_block_valid(unsigned b) { return b < VDP_BLOCKS && (block_valid[b >> 5] >> (b & 31) & 1); }
void vdp_take_dirty(uint32_t *bits) {
    memcpy(bits, block_dirty, sizeof block_dirty); memset(block_dirty, 0, sizeof block_dirty);
    uint32_t c1[VDP_BLOCKS / 32] = {0}; vdpcmd_take_dirty(c1, VDP_BLOCKS / 32);        // blocks written by the command engine
    for (unsigned i = 0; i < VDP_BLOCKS / 32; i++) { bits[i] |= c1[i]; block_valid[i] |= c1[i]; }
}
void vdp_mark_all_dirty(void) { memset(block_dirty, 0xFF, sizeof block_dirty); }
void vdp_mark_dirty(unsigned b) { if (b < VDP_BLOCKS) block_dirty[b >> 5] |= 1u << (b & 31); }

// Text screen from the BIOS work area (SCRMOD FCAFh, NAMBAS F922h, LINL40 F3AEh) and the VRAM shadow.
int vdp_screen_text(char *out, int outsz, int *cols) {
    if (!bus_enabled()) { *cols = 0; out[0] = 0; return 0; }   // work area reads would post to a parked bus engine
    uint8_t scrmod = mem_rd(0xFCAF);
    uint32_t base = mem_rd(0xF922) | (mem_rd(0xF923) << 8);
    int w, rows = 24;
    if (scrmod == 0) w = mem_rd(0xF3AE) > 40 ? 80 : 40;
    else if (scrmod == 1) w = 32;
    else { *cols = 0; return 0; }
    *cols = w;
    int n = 0;
    for (int r = 0; r < rows && n < outsz - w - 2; r++) {
        for (int c = 0; c < w; c++) {
            uint32_t a = base + r * w + c;
            uint8_t ch = valid(a) ? vram[a] : '?';
            out[n++] = (ch >= 32 && ch < 127) ? (char)ch : (valid(a) ? '.' : '?');
        }
        while (n > 0 && out[n - 1] == ' ') n--;
        out[n++] = '\n';
    }
    out[n] = 0;
    return rows;
}

void vdp_dump(uint32_t addr_, unsigned n) {
    for (unsigned i = 0; i < n; i++) {
        uint32_t a = (addr_ + i) & (VDP_VRAM_SIZE - 1);
        if (i % 32 == 0) printf("%05lx:", (unsigned long)a);
        if (valid(a)) printf(" %02x", vram[a]); else printf(" ??");
        if (i % 32 == 31 || i == n - 1) printf("\n");
    }
}
void vdp_dump_regs(void) {
    printf("vdp regs:");
    for (int r = 0; r < 24; r++) printf(" R%d=%02x", r, regs[r]);
    printf(" R25=%02x R44=%02x R46=%02x addr=%05lx latch=%d cmd_seen=%d pal:", regs[25], regs[44], regs[46], (unsigned long)addr, latch, cmd_seen);
    for (int i = 0; i < 16; i++) printf(" %03x", palette[i]);
    printf("\n");
}

// Full VRAM read-back through port 98 with the CPU stopped. Restores the VDP address afterwards
// (write pointer to the current address; the read-mode prefetch is not restored).
bool vdp_snapshot(unsigned gap_us) {
    uint32_t saved = addr;
    vdpcmd_drain();
    for (uint32_t bank = 0; bank < 8; bank++) {
        busy_wait_us(gap_us); bus_io_write(0x99, (uint8_t)bank); busy_wait_us(gap_us); bus_io_write(0x99, 0x8E);   // R#14 = bank
        busy_wait_us(gap_us); bus_io_write(0x99, 0x00); busy_wait_us(gap_us); bus_io_write(0x99, 0x00);           // address 0, read mode
        busy_wait_us(gap_us);
        uint8_t *dst = vram + bank * 0x4000;
        for (uint32_t i = 0; i < 0x4000; i++) { dst[i] = bus_io_read(0x98); busy_wait_us(gap_us); }
        if (bus_faulted()) return false;
    }
    memset(block_valid, 0xFF, sizeof block_valid); memset(block_dirty, 0xFF, sizeof block_dirty); cmd_seen = false;
    // restore: bank and address (write mode, as the BIOS mostly writes)
    busy_wait_us(gap_us); bus_io_write(0x99, (uint8_t)(saved >> 14)); busy_wait_us(gap_us); bus_io_write(0x99, 0x8E); busy_wait_us(gap_us);
    bus_io_write(0x99, (uint8_t)saved); busy_wait_us(gap_us); bus_io_write(0x99, (uint8_t)(0x40 | ((saved >> 8) & 0x3F)));
    bus_drain();
    return true;
}
