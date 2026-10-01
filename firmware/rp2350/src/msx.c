// MSX machine layer for the Sony HB-F1XDJ: slot/mapper tracking, ROM cache, RAM shadow,
// I/O dispatch, interrupt/reset pins, run loop.
#include <string.h>
#include <stdio.h>
#include "msx.h"
#include "z80_mem.h"
#include "bus.h"
#include "vdp.h"
#include "kbd.h"
#include "vzdisk.h"
#include "pins.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/structs/powman.h"
#include "hardware/structs/watchdog.h"
#include "hardware/timer.h"
#include "settings.h"
#include "psram.h"
#include "cart.h"
#include "fdc.h"

z80_t cpu;
msx_state_t msx;
uint8_t msx_ram[65536];
// Mapper segments 0-3 are the SRAM shadow above; 4..63 (optional expansion, see settings.ram_kb) sit in PSRAM.
static inline uint8_t *seg_ptr(unsigned seg) { return seg < 4 ? msx_ram + seg * 0x4000 : (uint8_t *)PSRAM_BASE + (seg - 4) * 0x4000; }
uint8_t *msx_rd[256], *msx_wr[256];

// ---- machine description ----------------------------------------------------------------
enum { K_BUS, K_RAM, K_ROM, K_CART, K_FF };   // K_FF: reads as an empty slot (FFh), writes ignored
static const uint8_t ff_page[256] __attribute__((aligned(4))) = { [0 ... 255] = 0xFF };   // open bus inside a virtual cartridge
typedef struct { uint8_t kind; uint8_t *backing; uint16_t line0; } region_t;   // per (pslot, sslot, 16K page)
static const uint8_t expanded[4] = { 1, 0, 0, 1 };

static uint8_t rom_main[0x8000], rom_sub[0x4000], rom_kanji[0x8000], rom_disk[0x4000], rom_fm[0x4000];
#define ROM_LINES (( 0x8000 + 0x4000 + 0x8000 + 0x4000 + 0x4000) / 256)
static uint32_t rom_valid[(ROM_LINES + 31) / 32];
static region_t region[4][4][4], model[4][4][4];   // model: as described, before any virtual cartridge

static void describe_machine(void) {
    // default everything to the bus
    for (int p = 0; p < 4; p++) for (int s = 0; s < 4; s++) for (int pg = 0; pg < 4; pg++) region[p][s][pg] = (region_t){ K_BUS, NULL, 0 };
    uint16_t line = 0;
    #define ROM(p, s, pg, buf, off) do { region[p][s][pg] = (region_t){ K_ROM, (buf) + (off), line }; line += 64; } while (0)
    ROM(0, 0, 0, rom_main, 0x0000); ROM(0, 0, 1, rom_main, 0x4000);              // MAIN ROM 0000-7FFF
    ROM(3, 1, 0, rom_sub, 0x0000);                                             // SUB ROM 0000-3FFF
    ROM(3, 1, 1, rom_kanji, 0x0000); ROM(3, 1, 2, rom_kanji, 0x4000);          // Kanji driver/BASIC 4000-BFFF
    ROM(3, 2, 1, rom_disk, 0x0000);                                            // Disk ROM 4000-7FFF (FDC at 7FF8)
    ROM(3, 3, 1, rom_fm, 0x0000);                                              // FM-BASIC 4000-7FFF
    #undef ROM
    for (int pg = 0; pg < 4; pg++) region[3][0][pg] = (region_t){ K_RAM, NULL, 0 };   // mapper RAM
    memcpy(model, region, sizeof model);
    memset(rom_valid, 0, sizeof rom_valid);
}

static inline int pslot_of(int pg) { return (msx.ppi_a8 >> (pg * 2)) & 3; }
static inline int sslot_of(int p, int pg) { return expanded[p] ? (msx.sec[p] >> (pg * 2)) & 3 : 0; }
static inline const region_t *region_of(uint16_t a) { int pg = a >> 14, p = pslot_of(pg); return &region[p][sslot_of(p, pg)][pg]; }
static inline bool line_valid(const region_t *r, uint16_t a) { unsigned l = r->line0 + ((a & 0x3FFF) >> 8); return rom_valid[l >> 5] >> (l & 31) & 1; }
static inline void line_set(const region_t *r, uint16_t a) { unsigned l = r->line0 + ((a & 0x3FFF) >> 8); rom_valid[l >> 5] |= 1u << (l & 31); }

// Rebuild the 256-entry page tables from the slot state.
static void remap(void) {
    for (int pg = 0; pg < 4; pg++) {
        int p = pslot_of(pg); const region_t *r = &region[p][sslot_of(p, pg)][pg];
        uint8_t *rd = NULL, *wr = NULL;
        if (r->kind == K_RAM) rd = wr = seg_ptr(msx.mapper[pg]) - pg * 0x4000;
        for (int sub = 0; sub < 64; sub++) {
            int e = pg * 64 + sub; uint16_t a = (uint16_t)(e << 8);
            if (r->kind == K_ROM) { msx_rd[e] = line_valid(r, a) ? r->backing - pg * 0x4000 : NULL; msx_wr[e] = NULL; }
            else if (r->kind == K_CART) { const uint8_t *q = cart_read_ptr(p, sslot_of(p, pg), a); msx_rd[e] = (uint8_t *)(q ? q : ff_page) - (q ? a : (a & 0xFF00)); msx_wr[e] = NULL; }
            else if (r->kind == K_FF) { msx_rd[e] = (uint8_t *)ff_page - (a & 0xFF00); msx_wr[e] = NULL; }
            else { msx_rd[e] = rd; msx_wr[e] = wr; }
        }
        // Disk ROM: the last 256 bytes of page 1 hold the FDC registers -> always physical.
        if (p == 3 && sslot_of(p, pg) == 2 && pg == 1) { msx_rd[0x7F] = NULL; msx_wr[0x7F] = NULL; }
    }
    // FFxx: the secondary slot register lives at FFFF when page 3 is in an expanded slot.
    if (expanded[pslot_of(3)]) { msx_rd[0xFF] = NULL; msx_wr[0xFF] = NULL; }
}

// ---- FDC access trace (7FF8-7FFF, the WD2793 interface inside the disk ROM page) ------------
#define FDTRACE_N 1024
static uint32_t fdtrace[FDTRACE_N][2];   // [0] time_us, [1] = addr | val<<16 | rd<<24 | (pc>>? no: pc low 7 bits<<25)
static uint32_t fdtrace_i;
static uint32_t fd_polls, fd_data;   // 7FFF polls and 7FFB data bytes since the last other access (summarised)
static inline void fdtrace_raw(uint32_t key) { fdtrace[fdtrace_i & (FDTRACE_N - 1)][0] = time_us_32(); fdtrace[fdtrace_i & (FDTRACE_N - 1)][1] = key; fdtrace_i++; }
static inline void fdtrace_add(uint16_t a, uint8_t v, bool rd) {
    if (rd && a == 0x7FFF) { fd_polls++; return; }
    if (a == 0x7FFB) { fd_data++; return; }
    if (fd_polls || fd_data) { fdtrace_raw(0xFFFE | (fd_polls << 16)); fdtrace_raw(0xFFFD | (fd_data << 16)); fd_polls = fd_data = 0; }
    uint32_t key = a | ((uint32_t)v << 16) | (rd ? 1u << 24 : 0);
    if (rd && fdtrace_i) {   // identical consecutive status reads collapse into a repeat count (bits 25-31)
        uint32_t *prev = fdtrace[(fdtrace_i - 1) & (FDTRACE_N - 1)];
        if ((prev[1] & 0x1FFFFFF) == key && (prev[1] >> 25) < 127) { prev[1] += 1u << 25; return; }
    }
    fdtrace_raw(key);
}
// Hex dump of a cached ROM (main, sub, kanji, disk, fm) for analysis over the console.
void msx_romhex(const char *name, uint32_t off, uint32_t len) {
    const uint8_t *r = NULL; uint32_t size = 0;
    if (!strcmp(name, "main")) { r = rom_main; size = sizeof rom_main; } else if (!strcmp(name, "sub")) { r = rom_sub; size = sizeof rom_sub; }
    else if (!strcmp(name, "kanji")) { r = rom_kanji; size = sizeof rom_kanji; } else if (!strcmp(name, "disk")) { r = rom_disk; size = sizeof rom_disk; }
    else if (!strcmp(name, "fm")) { r = rom_fm; size = sizeof rom_fm; }
    if (!r) { printf("err main|sub|kanji|disk|fm\n"); return; }
    if (off >= size) { printf("err offset\n"); return; }
    if (off + len > size) len = size - off;
    for (uint32_t i = 0; i < len; i += 32) { printf("%04lx:", (unsigned long)(off + i)); for (uint32_t k = 0; k < 32 && i + k < len; k++) printf(" %02x", r[off + i + k]); printf("\n"); }
}
void msx_fdtrace_reset(void) { fdtrace_i = 0; fd_polls = fd_data = 0; }
void msx_fdtrace_print(int n) {
    uint32_t total = fdtrace_i; if (n <= 0 || (uint32_t)n > FDTRACE_N) n = 64; if ((uint32_t)n > total) n = (int)total;
    printf("fdtrace: %lu accesses\n", (unsigned long)total);
    uint32_t t0 = fdtrace[(total - n) & (FDTRACE_N - 1)][0];
    for (uint32_t i = total - n; i < total; i++) {
        uint32_t t = fdtrace[i & (FDTRACE_N - 1)][0], w = fdtrace[i & (FDTRACE_N - 1)][1];
        if ((w & 0xFFFF) == 0xFFFE) printf("%8lu    [%lu polls of 7FFF]\n", (unsigned long)(t - t0), (unsigned long)(w >> 16));
        else if ((w & 0xFFFF) == 0xFFFD) printf("%8lu    [%lu data bytes via 7FFB]\n", (unsigned long)(t - t0), (unsigned long)(w >> 16));
        else printf("%8lu %s %04lx %02lx%s%lu\n", (unsigned long)(t - t0), (w >> 24) & 1 ? "rd" : "wr", (unsigned long)(w & 0xFFFF), (unsigned long)((w >> 16) & 0xFF), (w >> 25) ? " x" : "", (unsigned long)((w >> 25) + 1));
    }
}
// ---- slow paths ----------------------------------------------------------------------------
static uint8_t rom_fill(const region_t *r, uint16_t a) {
    uint16_t base = (uint16_t)(a & 0xFF00);
    uint8_t *dst = r->backing + (base & 0x3FFF);
    for (int i = 0; i < 256; i++) dst[i] = bus_mem_read((uint16_t)(base + i));
    if (bus_faulted()) return dst[a & 0xFF];
    line_set(r, a); msx.rom_fills++;
    msx_rd[a >> 8] = r->backing - (a >> 14) * 0x4000;
    return dst[a & 0xFF];
}

static uint8_t slow_read(uint16_t a, bool m1) {
    if (a >= 0xFF00) {
        int p = pslot_of(3);
        if (a == 0xFFFF && expanded[p]) return (uint8_t)~msx.sec[p];
        const region_t *r = &region[p][sslot_of(p, 3)][3];
        if (r->kind == K_RAM) return seg_ptr(msx.mapper[3])[a & 0x3FFF];
        if (r->kind == K_CART) { const uint8_t *q = cart_read_ptr(p, sslot_of(p, 3), a); return q ? *q : 0xFF; }
        if (r->kind == K_FF) return 0xFF;
    }
    const region_t *r = region_of(a);
    if (r->kind == K_FF) return 0xFF;
    if (r->kind == K_ROM && !(a >= 0x7F00 && a <= 0x7FFF && r->backing == rom_disk)) return rom_fill(r, a);
    if (a >= 0x7FF8 && r->kind == K_ROM && r->backing == rom_disk) { uint8_t v; fdc_read(a, &v); fdtrace_add(a, v, true); return v; }
    return m1 ? bus_m1(a) : bus_mem_read(a);
}
uint8_t __not_in_flash_func(msx_rd_slow)(uint16_t a)    { return slow_read(a, false); }
uint8_t __not_in_flash_func(msx_fetch_slow)(uint16_t a) { return slow_read(a, true); }

void __not_in_flash_func(msx_wr_slow)(uint16_t a, uint8_t v) {
    if (a >= 0xFF00) {
        int p = pslot_of(3);
        if (a == 0xFFFF && expanded[p]) { msx.sec[p] = v; bus_mem_write(a, v); remap(); return; }
        const region_t *r = &region[p][sslot_of(p, 3)][3];
        if (r->kind == K_RAM) { seg_ptr(msx.mapper[3])[a & 0x3FFF] = v; return; }
    }
    { const region_t *r = region_of(a); if (r->kind == K_FF) return;
      if (r->kind == K_CART) { int ps = pslot_of(a >> 14), ss = sslot_of(ps, a >> 14); cart_observe(ps, ss, a, v); if (cart_write(ps, ss, a, v)) remap(); return; }
      if (a >= 0x7FF8 && r->kind == K_ROM && r->backing == rom_disk) { fdtrace_add(a, v, false); fdc_write(a, v); return; } }   // virtual cartridge: mapper registers, never the bus; FDC window: fdc.c
    bus_mem_write(a, v);     // ROM (no effect), MMIO, cartridges
}

// Virtual cartridge in slot cell (p, s): pages in `mask` come from cart.c, the rest is what the model says.
void msx_set_cart_pages(int p, int s, uint8_t mask) {
    if (p < 0 || p > 3 || s < 0 || s > 3) return;
    for (int pg = 0; pg < 4; pg++) region[p][s][pg] = (mask >> pg) & 1 ? (region_t){ K_CART, NULL, 0 } : model[p][s][pg];
    remap();
}
bool msx_running(void) { return msx.running; }
bool msx_slot_expanded(int p) { return p >= 0 && p <= 3 && expanded[p]; }
uint8_t msx_disk_rom_slotid(void) {
    for (int p = 0; p < 4; p++) for (int s = 0; s < 4; s++) if (model[p][s][1].kind == K_ROM && model[p][s][1].backing == rom_disk) return expanded[p] ? (uint8_t)(0x80 | p | (s << 2)) : (uint8_t)p;
    return 0xFF;
}
void msx_restart(void) { msx_stop(); msx.started = false; msx_resume(); }

// ---- I/O trace ring ------------------------------------------------------------------------
#define IOTRACE_N 2048
static uint32_t iotrace[IOTRACE_N][2];   // [0] = time_us, [1] = port | value<<8 | rd<<16 | (pc<<17 & ...)
static uint32_t iotrace_i; static bool iotrace_frozen, iotrace_freeze_en; static uint32_t iotrace_rep;
static uint32_t iotrace_skip_mask;   // bit n set: do not record port 0x98+n (n < 32)
static int iotrace_trig_lo = -1, iotrace_trig_hi = -1, iotrace_trig_after = -1, iotrace_trig_len = 200;   // freeze N entries after `out 99,lo; out 99,hi`
static inline void iotrace_add(uint16_t port, uint8_t v, bool rd) {
    if (iotrace_frozen) return;
    if ((uint8_t)(port - 0x98) < 32 && (iotrace_skip_mask >> (uint8_t)(port - 0x98) & 1)) return;
    // Freeze once the BIOS "wait for VDP command" poll loop (out 99,02 / out 99,8F / in 99 /
    // out 99,00 / out 99,8F) has repeated 50 times: the ring then holds what led up to it.
    static const uint32_t pat[5] = { 0x99 | 0x02 << 16, 0x99 | 0x8F << 16, 0x99 | 1u << 24, 0x99 | 0x00 << 16, 0x99 | 0x8F << 16 };
    uint32_t key = (port & 0xFF) | (rd ? 1u << 24 : (uint32_t)v << 16);
    static uint32_t pi;
    if (iotrace_freeze_en && key == pat[pi]) { if (++pi == 5) { pi = 0; if (++iotrace_rep >= 50) { iotrace_frozen = true; return; } } }
    else { pi = key == pat[0] ? 1 : 0; iotrace_rep = 0; }
    iotrace[iotrace_i & (IOTRACE_N - 1)][0] = time_us_32();
    iotrace[iotrace_i & (IOTRACE_N - 1)][1] = port | ((uint32_t)v << 16) | (rd ? 1u << 24 : 0) | ((uint32_t)(z80_io_pc & 0x7F) << 25);
    iotrace_i++;
    if (iotrace_trig_after >= 0) { if (--iotrace_trig_after < 0) iotrace_frozen = true; }
    else if (iotrace_trig_lo >= 0 && !rd && (port & 0xFF) == 0x99) {
        static int prev = -1;
        if (prev == iotrace_trig_lo && v == iotrace_trig_hi) iotrace_trig_after = iotrace_trig_len;
        prev = v;
    }
}
void msx_iotrace_reset(void) { iotrace_i = 0; iotrace_frozen = false; iotrace_rep = 0; iotrace_trig_after = -1; }
void msx_iotrace_trigger(int lo, int hi, int after) { iotrace_trig_lo = lo; iotrace_trig_hi = hi; iotrace_trig_len = after > 0 ? after : 200; iotrace_trig_after = -1; iotrace_frozen = false; }
void msx_iotrace_skip(uint32_t mask) { iotrace_skip_mask = mask; }
void msx_iotrace_freeze(bool en) { iotrace_freeze_en = en; iotrace_frozen = false; iotrace_rep = 0; }
void msx_iotrace_dump(unsigned n, unsigned skip) {
    printf("iotrace: %lu accesses, %s\n", (unsigned long)iotrace_i, iotrace_frozen ? "frozen (loop detected)" : "recording");
    if (n + skip > IOTRACE_N) n = IOTRACE_N - skip; if (n + skip > iotrace_i) n = iotrace_i > skip ? iotrace_i - skip : 0;
    uint32_t end = iotrace_i - skip;
    uint32_t t0 = iotrace[(end - n) & (IOTRACE_N - 1)][0];
    for (unsigned k = end - n; k < end; k++) {
        uint32_t *e = iotrace[k & (IOTRACE_N - 1)];
        printf("%8lu us %s %04lx %02lx\n", (unsigned long)(e[0] - t0), (e[1] >> 24 & 1) ? "in " : "out", e[1] & 0xFFFF, (e[1] >> 16) & 0xFF);
    }
}

// ---- I/O ----------------------------------------------------------------------------------
static uint32_t vdp_last_us;
// One minimum interval between any two VDP port accesses, whatever the screen mode or display state
// (per-mode gaps were tried: the display-off one lost bytes at 3 us and bought ~40 % on screen
// initialisation only, so it went).
static inline void vdp_pace(void) {
    uint32_t gap = msx.vdp_gap_us;
    if (gap) { while ((uint32_t)(time_us_32() - vdp_last_us) < gap) tight_loop_contents(); }
    vdp_last_us = time_us_32();
}

static inline uint8_t io_rd_impl(uint16_t port) {
    uint8_t p = (uint8_t)port, v;
    switch (p) {
        case 0x98: vdp_pace(); v = bus_io_read(port); vdp_observe_read(p, v); return v;
        case 0x99: vdp_pace(); v = bus_io_read(port); vdp_observe_read(p, v); return v;
        case 0xA8: return msx.ppi_a8;
        case 0xFC: case 0xFD: case 0xFE: case 0xFF:   // emulated mapper: unused upper bits read as 1 (as on the real chips)
            if (msx.ram_segs > 4) return (uint8_t)(msx.mapper[p - 0xFC] | ~msx.seg_mask);
            return bus_io_read(port);
        case 0xA9: v = bus_io_read(port); return (uint8_t)(v & ~kbd_row_mask());
        case 0x40: case 0x41: case 0x42: case 0x43: case 0x44: case 0x45: case 0x46: case 0x47: case 0x48: case 0x49: case 0x4A: case 0x4B:
            if (vzdisk_io_rd(p, &v)) return v;
            return bus_io_read(port);
        default: return bus_io_read(port);
    }
}
uint8_t __not_in_flash_func(msx_io_rd)(uint16_t port) { uint8_t v = io_rd_impl(port); iotrace_add(port, v, true); return v; }

// MSX-MUSIC (YM2413, ports 7C/7D): the chip needs 12 cycles after an address write and 84 cycles after a
// data write before the next access. Real software has those delays in Z80 cycles; here a paced chunk (and
// the Max mode) runs them at core speed, so the gaps are enforced in real time like the VDP gap.
static uint32_t opll_last_us; static uint8_t opll_last_gap;
static inline void opll_pace(uint8_t next_gap_us) {
    while ((uint32_t)(time_us_32() - opll_last_us) < opll_last_gap) tight_loop_contents();
    opll_last_us = time_us_32(); opll_last_gap = next_gap_us;
}
void __not_in_flash_func(msx_io_wr)(uint16_t port, uint8_t v) {
    uint8_t p = (uint8_t)port;
    iotrace_add(port, v, false);
    switch (p) {
        case 0x7C: opll_pace(4); bus_io_write(port, v); return;    // OPLL register address: >= 12 cycles (3.4 us) before the data
        case 0x7D: opll_pace(24); bus_io_write(port, v); return;   // OPLL data: >= 84 cycles (23.5 us) before the next access
        case 0x98: vdp_pace(); bus_io_write(port, v); vdp_observe_write(p, v); return;
        case 0x99: case 0x9A: case 0x9B: vdp_pace(); bus_io_write(port, v); vdp_observe_write(p, v); return;
        case 0xA8: msx.ppi_a8 = v; bus_io_write(port, v); remap(); return;
        case 0xAA: bus_io_write(port, v); kbd_row_select(v & 0x0F); return;
        case 0xFC: case 0xFD: case 0xFE: case 0xFF: msx.mapper[p - 0xFC] = v & msx.seg_mask; bus_io_write(port, v); remap(); return;
        case 0x40: case 0x41: case 0x42: case 0x43: case 0x44: case 0x45: case 0x46: case 0x47: case 0x48:
            if (vzdisk_io_wr(p, v)) return;
            bus_io_write(port, v); return;
        default: bus_io_write(port, v); return;
    }
}

uint8_t __not_in_flash_func(msx_int_ack)(uint16_t pc) { msx.ints++; return bus_int_ack(pc); }

static bool pace_armed;   // standard-speed pacing origin valid (see pace())
// ---- control -------------------------------------------------------------------------------
void msx_init(void) {
    describe_machine();
    z80_reset(&cpu);
    msx.vdp_gap_us = settings.vdp_gap_us;
    msx.refresh_us = settings.refresh_us; msx.speed = settings.speed;
    msx.mapper[0] = 3; msx.mapper[1] = 2; msx.mapper[2] = 1; msx.mapper[3] = 0;
    msx.ram_segs = 4; msx.seg_mask = 3;
    remap();
    vdp_init(); kbd_init();
    msx_autostart_init();
}

void msx_map_flat(void) {
    msx_stop();
    for (int p = 0; p < 256; p++) { msx_rd[p] = msx_ram; msx_wr[p] = msx_ram; }
}

// The real PPI, mapper and secondary slot registers keep their own state: mirror it.
// Secondary registers are read through FFFF with page 3 temporarily switched to each
// expanded primary slot (the same trick the BIOS uses).
// We cannot pulse the machine's /RESET, so before a Z80 reset put the slot hardware into its power-on
// state ourselves: PPI A8 = 0 (slot 0 in every page) and secondary slot register 0 in every expanded slot.
static void reset_slot_hardware(void) {
    for (int p = 0; p < 4; p++) {
        if (!expanded[p]) continue;
        bus_io_write(0xA8, (uint8_t)(p << 6)); bus_mem_write(0xFFFF, 0x00);
    }
    bus_io_write(0xA8, 0x00);
    bus_drain();
}
// Pull every ROM the machine model knows about into the line cache in one go (~110 KiB, ~0.2 s) so
// that no first-touch bus fetches are left for later (each costs ~0.4 ms of stalled Z80 time).
// Each ROM is reachable only while its slot is selected: page 3 stays on the primary slot being
// read so that the secondary slot register (FFFF) of that slot is writable.
static void prefetch_roms(void) {
    for (int p = 0; p < 4; p++) for (int s = 0; s < 4; s++) for (int pg = 0; pg < 4; pg++) {
        const region_t *r = &region[p][s][pg];
        if (r->kind != K_ROM) continue;
        uint8_t a8 = (uint8_t)((p << (pg * 2)) | (p << 6));               // page pg and page 3 -> primary p
        bus_io_write(0xA8, a8);
        if (expanded[p]) bus_mem_write(0xFFFF, (uint8_t)(s << (pg * 2)));   // page pg -> secondary s (others 0)
        for (uint32_t line = 0; line < 64; line++) {
            uint16_t base = (uint16_t)((pg << 14) | (line << 8));
            if (r->backing == rom_disk && base >= 0x7F00) continue;           // FDC registers, never cache
            if (rom_valid[(r->line0 + line) >> 5] >> ((r->line0 + line) & 31) & 1) continue;
            uint8_t *dst = r->backing + (line << 8);
            for (int i = 0; i < 256; i++) dst[i] = bus_mem_read((uint16_t)(base + i));
            if (bus_faulted()) return;
            rom_valid[(r->line0 + line) >> 5] |= 1u << ((r->line0 + line) & 31); msx.rom_fills++;
        }
        if (expanded[p]) bus_mem_write(0xFFFF, 0);
    }
    bus_drain();
}
// Re-read every cached ROM line from the machine and compare (Z80 stopped). Returns the number of
// lines that differ; with fix, the cache takes the fresh copy. Diagnoses bus-read corruption.
uint32_t msx_rom_check(bool fix, uint32_t *bytes_out) {
    if (msx.running || !bus_enabled()) return 0xFFFFFFFFu;
    uint32_t lines_bad = 0, bytes = 0; uint8_t a8 = bus_io_read(0xA8);
    for (int p = 0; p < 4; p++) for (int s = 0; s < 4; s++) for (int pg = 0; pg < 4; pg++) {
        const region_t *r = &region[p][s][pg];
        if (r->kind != K_ROM) continue;
        bus_io_write(0xA8, (uint8_t)((p << (pg * 2)) | (p << 6)));
        if (expanded[p]) bus_mem_write(0xFFFF, (uint8_t)(s << (pg * 2)));
        for (uint32_t line = 0; line < 64; line++) {
            uint16_t base = (uint16_t)((pg << 14) | (line << 8));
            if (r->backing == rom_disk && base >= 0x7F00) continue;
            if (!(rom_valid[(r->line0 + line) >> 5] >> ((r->line0 + line) & 31) & 1)) continue;
            uint8_t *dst = r->backing + (line << 8); uint8_t tmp[256]; int diff = 0;
            for (int i = 0; i < 256; i++) { tmp[i] = bus_mem_read((uint16_t)(base + i)); if (tmp[i] != dst[i]) diff++; }
            if (bus_faulted()) return 0xFFFFFFFFu;
            if (diff) { lines_bad++; bytes += diff; printf("rom %d-%d page %d line %02lx: %d bytes differ (e.g. %04x cache %02x bus %02x)\n", p, s, pg, (unsigned long)line, diff,
                        base, dst[0], tmp[0]); if (fix) memcpy(dst, tmp, 256); }
        }
        if (expanded[p]) bus_mem_write(0xFFFF, 0);
    }
    // restore the machine's slot state
    for (int p = 0; p < 4; p++) { if (!expanded[p]) continue; bus_io_write(0xA8, (uint8_t)((a8 & 0x3F) | (p << 6))); bus_mem_write(0xFFFF, msx.sec[p]); }
    bus_io_write(0xA8, a8); bus_drain();
    if (bytes_out) *bytes_out = bytes;
    return lines_bad;
}
// ---- slot scan --------------------------------------------------------------------------------
// Probe the real slot hardware (Z80 stopped): expansion of each primary slot (FFFF read-back), then a
// few bytes of every (slot, subslot, page) cell. A cell that reads all FF is empty (floating bus);
// "AB" at the page start is a ROM header; pages 0 and 3 also get a write/read-back test for RAM
// (pages 1/2 are never written: MegaROM bank registers live there). Runs at every MSX start; the
// result feeds the slot map (Web UI) and the check before a virtual cartridge takes a cell.
enum { SK_NONE, SK_EMPTY, SK_RAM, SK_ROM, SK_DATA, SK_ZERO };
static uint8_t scan_exp[4], scan_kind[4][4][4], scan_head[4][4][4][32]; static bool scanned;
static const char *model_name(int p, int s, int pg) {
    const region_t *r = &model[p][s][pg];
    if (r->kind == K_RAM) return "ram";
    if (r->kind != K_ROM) return "";
    if (r->backing >= rom_main && r->backing < rom_main + sizeof rom_main) return pg == 0 ? "bios" : "basic";
    if (r->backing >= rom_sub && r->backing < rom_sub + sizeof rom_sub) return "sub";
    if (r->backing >= rom_kanji && r->backing < rom_kanji + sizeof rom_kanji) return "kanji";
    if (r->backing >= rom_disk && r->backing < rom_disk + sizeof rom_disk) return "disk";
    if (r->backing >= rom_fm && r->backing < rom_fm + sizeof rom_fm) return "fm";
    return "rom";
}
static void probe_cells(void) {
    if (msx.running || !bus_enabled()) return;
    uint8_t a8 = bus_io_read(0xA8), sec_orig[4] = { 0, 0, 0, 0 };
    memset(scan_kind, 0, sizeof scan_kind);
    for (int p = 0; p < 4; p++) {
        bus_io_write(0xA8, (uint8_t)((a8 & 0x3F) | (p << 6)));                  // page 3 -> primary p
        uint8_t orig = bus_mem_read(0xFFFF);                                     // ~sec when expanded
        bus_mem_write(0xFFFF, 0xA5); bus_drain();
        bool exp = bus_mem_read(0xFFFF) == (uint8_t)~0xA5;
        scan_exp[p] = exp; sec_orig[p] = (uint8_t)~orig;
        if (exp) { bus_mem_write(0xFFFF, sec_orig[p]); bus_drain(); }
        for (int s = 0; s < (exp ? 4 : 1); s++) {
            for (int pg = 0; pg < 4; pg++) {
                bus_io_write(0xA8, (uint8_t)((p << (pg * 2)) | (p << 6)));       // page pg and page 3 -> p
                if (exp) bus_mem_write(0xFFFF, (uint8_t)((s << (pg * 2)) | (s << 6)));
                bus_drain();
                uint16_t base = (uint16_t)(pg << 14); uint8_t *b = scan_head[p][s][pg]; bool all_ff = true, all_00 = true;
                for (int i = 0; i < 32; i++) { b[i] = bus_mem_read((uint16_t)(base + i)); if (b[i] != 0xFF) all_ff = false; if (b[i] != 0x00) all_00 = false; }
                static const uint16_t probe[] = { 0x0800, 0x1000, 0x2000, 0x3000, 0x3F00, 0x3FF0 };
                for (unsigned k = 0; k < sizeof probe / sizeof probe[0]; k++) { uint8_t v = bus_mem_read((uint16_t)(base + probe[k])); if (v != 0xFF) all_ff = false; if (v != 0x00) all_00 = false; }
                bool ram = false;
                if (pg == 0 || pg == 3) {                                          // write test away from FFFF and from any register window
                    uint16_t t = (uint16_t)(base + 0x3FF0); uint8_t v0 = bus_mem_read(t);
                    bus_mem_write(t, (uint8_t)~v0); bus_drain(); uint8_t v1 = bus_mem_read(t);
                    bus_mem_write(t, v0); bus_drain(); uint8_t v2 = bus_mem_read(t);
                    ram = v1 == (uint8_t)~v0 && v2 == v0;
                }
                if (bus_faulted()) goto restore;
                scan_kind[p][s][pg] = ram ? SK_RAM : all_ff ? SK_EMPTY : all_00 ? SK_ZERO : (b[0] == 'A' && b[1] == 'B') ? SK_ROM : SK_DATA;
            }
            if (exp) bus_mem_write(0xFFFF, 0);
        }
    }
    scanned = true;
restore:
    for (int p = 0; p < 4; p++) { if (!scan_exp[p]) continue; bus_io_write(0xA8, (uint8_t)((a8 & 0x3F) | (p << 6))); bus_mem_write(0xFFFF, sec_orig[p]); }
    bus_io_write(0xA8, a8); bus_drain();
}
// What the Web UI calls the cell: the model's name, else what the scan saw. "je" = the MSX-JE ROM of
// the HB-F1XDJ (slot 0-3), the one built-in ROM the model does not cache: "SONY  JFEP  TINY DUMMY" at
// 4010h of page 1 and "SONY JFEP2" at the start of page 2, so look for "JFE" anywhere in the 32-byte heads.
static const char *cell_kind(int p, int s, int pg) {
    const char *m = model_name(p, s, pg); if (*m) return m;
    uint8_t k = scan_kind[p][s][pg];
    if (k == SK_ROM || k == SK_DATA) {
        for (int q = 0; q < 4; q++) for (unsigned i = 0; i + 3 <= sizeof scan_head[p][s][q]; i++) if (!memcmp(scan_head[p][s][q] + i, "JFE", 3)) return "je";
        return k == SK_ROM ? "rom" : "data";
    }
    return k == SK_RAM ? "ram" : k == SK_EMPTY ? "empty" : k == SK_ZERO ? "zeros" : "unknown";
}
// Can a virtual cartridge take pages `mask` of cell (p, s)? The model must have nothing there and the
// scan must have seen an empty bus (a real cartridge in slot 1 / 2 keeps its slot).
bool msx_cell_free(int p, int s, uint8_t mask, bool ignore_live, char *err, size_t n) {
    if (p < 0 || p > 3 || s < 0 || s > 3 || (!expanded[p] && s != 0)) { snprintf(err, n, "no such slot cell"); return false; }
    for (int pg = 0; pg < 4; pg++) {
        if (!((mask >> pg) & 1)) continue;
        if (model[p][s][pg].kind != K_BUS) { snprintf(err, n, "page %d of %d-%d is %s", pg, p, s, model_name(p, s, pg)); return false; }
        if (!ignore_live && region[p][s][pg].kind == K_CART) { snprintf(err, n, "page %d of %d-%d has a cartridge", pg, p, s); return false; }
        if (scanned && scan_kind[p][s][pg] != SK_EMPTY && scan_kind[p][s][pg] != SK_NONE) { snprintf(err, n, "page %d of %d-%d is not empty on the machine", pg, p, s); return false; }
    }
    return true;
}
// Internal floppy drives: put the disk ROM cell into page 1 (its FDC registers live at 7FF8h-7FFFh
// there) and let fdc.c count them. reset_slot_hardware() follows in msx_start().
static void probe_drives(void) {
    if (msx.running || !bus_enabled()) return;
    for (int p = 0; p < 4; p++) for (int s = 0; s < 4; s++) {
        if (model[p][s][1].kind != K_ROM || model[p][s][1].backing != rom_disk) continue;
        bus_io_write(0xA8, (uint8_t)((p << 2) | (p << 6)));                     // page 1 and page 3 -> p
        if (expanded[p]) bus_mem_write(0xFFFF, (uint8_t)((s << 2) | (s << 6)));
        bus_drain();
        fdc_probe_physical();
        if (expanded[p]) bus_mem_write(0xFFFF, 0);
        bus_io_write(0xA8, 0); bus_drain();
        return;
    }
}
void msx_slot_scan(void) {   // console: re-probe and print the table
    if (msx.running || !bus_enabled()) { printf("err stop first\n"); return; }
    probe_cells();
    for (int p = 0; p < 4; p++) {
        printf("slot %d: %s (model %s)%s\n", p, scan_exp[p] ? "expanded" : "not expanded", expanded[p] ? "expanded" : "not expanded", (bool)scan_exp[p] == (bool)expanded[p] ? "" : "  <-- differs");
        for (int s = 0; s < (scan_exp[p] ? 4 : 1); s++) for (int pg = 0; pg < 4; pg++) {
            const uint8_t *b = scan_head[p][s][pg]; uint8_t k = scan_kind[p][s][pg];
            printf("  %d-%d page %d: ", p, s, pg);
            if (k == SK_RAM) printf("RAM"); else if (k == SK_EMPTY) printf("empty (FF)"); else if (k == SK_ZERO) printf("zeros");
            else if (k == SK_ROM) printf("ROM \"AB\" init=%02x%02x stmt=%02x%02x dev=%02x%02x text=%02x%02x", b[3], b[2], b[5], b[4], b[7], b[6], b[9], b[8]);
            else printf("data %02x %02x %02x %02x %02x %02x %02x %02x ..", b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7]);
            const char *m = model_name(p, s, pg); if (*m) printf("   [model: %s]", m);
            printf("\n");
        }
    }
    printf("ok\n");
}
void msx_slotmap_json(void) {
    printf("{\"scanned\":%s,\"slots\":[", scanned ? "true" : "false");
    for (int p = 0; p < 4; p++) {
        printf("%s{\"p\":%d,\"exp\":%s,\"subs\":[", p ? "," : "", p, expanded[p] ? "true" : "false");
        for (int s = 0; s < (expanded[p] ? 4 : 1); s++) {
            printf("%s{\"s\":%d,\"pages\":[", s ? "," : "", s);
            for (int pg = 0; pg < 4; pg++) {
                int shelf; uint8_t type, pages;
                if (cart_pending_info(p, s, &shelf, &type, &pages) && ((pages >> pg) & 1)) printf("%s{\"k\":\"cart\",\"shelf\":%d,\"type\":%u,\"pages\":%u}", pg ? "," : "", shelf, type, pages);
                else printf("%s{\"k\":\"%s\"}", pg ? "," : "", cell_kind(p, s, pg));
            }
            printf("]}");
        }
        printf("]}");
    }
    printf("]}\n");
}
static void sync_from_machine(void) {
    uint8_t a8 = bus_io_read(0xA8);
    msx.ppi_a8 = a8;
    if (msx.ram_segs == 4) { for (int i = 0; i < 4; i++) msx.mapper[i] = bus_io_read((uint16_t)(0xFC + i)) & 3; }
    else { msx.mapper[0] = 3; msx.mapper[1] = 2; msx.mapper[2] = 1; msx.mapper[3] = 0; }   // the real 2-bit registers cannot hold ours: power-on layout
    for (int p = 0; p < 4; p++) {
        if (!expanded[p]) { msx.sec[p] = 0; continue; }
        bus_io_write(0xA8, (uint8_t)((a8 & 0x3F) | (p << 6)));
        msx.sec[p] = (uint8_t)~bus_mem_read(0xFFFF);
    }
    bus_io_write(0xA8, a8);
    bus_drain();
    remap();
}

// Z80 reset. The rest of the machine is not reset by us (we cannot drive /RESET), so the
// slot model is re-synchronised from the hardware instead of assumed.
void msx_reset_cpu(void) {
    bool was_running = msx.running;
    msx.running = false; bus_drain();
    z80_reset(&cpu);
    if (bus_enabled()) { reset_slot_hardware(); sync_from_machine(); }
    vdp_reset(); kbd_init();
    msx.resets++;
    msx.running = was_running;
}

// Mapper size from the settings; needs the PSRAM for anything above 64 KiB. Applied when the MSX (re)starts.
static void apply_ram_size(void) {
    uint32_t kb = settings.ram_kb;
    if (kb != 256 && kb != 512 && kb != 1024) kb = 64;
    if (kb > 64 && !psram_info().ok) kb = 64;
    msx.ram_segs = (uint8_t)(kb / 16); msx.seg_mask = (uint8_t)(msx.ram_segs - 1);
    for (int i = 0; i < 4; i++) msx.mapper[i] &= msx.seg_mask;
}
void msx_start(void) {
    if (msx.running) return;
    apply_ram_size();
    fdc_init();
    bus_clear_fault();
    bus_enable(true);
    reset_slot_hardware();
    probe_cells();
    probe_drives();
    cart_unload_all();
    cart_restore();
    // Both internal drives "none": hide the internal disk ROM from the BIOS (its page reads as an empty slot),
    // so it registers no drive and keeps no buffers - about 1.6 KB more for programs that need it (Nextor only).
    for (int p = 0; p < 4; p++) for (int s = 0; s < 4; s++) for (int pg = 0; pg < 4; pg++)
        if (model[p][s][pg].kind == K_ROM && model[p][s][pg].backing == rom_disk) region[p][s][pg] = fdc_internal_hidden() ? (region_t){ K_FF, NULL, 0 } : model[p][s][pg];
    reset_slot_hardware();
    prefetch_roms();
    reset_slot_hardware();
    sync_from_machine();
    z80_reset(&cpu); vdp_reset(); kbd_init();
    msx_iotrace_reset();
    msx.running = true; msx.in_reset = false; msx.started = true; pace_armed = false;
}
void msx_resume(void) {
    if (msx.running || !msx.started || !bus_enabled() || bus_faulted()) { msx_start(); return; }
    msx.running = true; msx.in_reset = false; pace_armed = false;
}

void msx_stop(void) {
    if (!msx.running) return;
    msx.running = false;
    bus_drain();
}

uint8_t msx_phys_read(uint16_t a) { return bus_mem_read(a); }
void msx_phys_write(uint16_t a, uint8_t v) { bus_mem_write(a, v); bus_drain(); }

// Standard speed: hold the emulated T-state count to a 3.579545 MHz clock plus one wait state per M1
// cycle (the MSX inserts it), approximated as one T per instruction. Origin is re-armed whenever we fall
// more than PACE_SLIP_US behind (after a stop, a snapshot, a slow console command...).
#define PACE_SLIP_US 4000
#define TURBOR_HZ 19150000ull   // speed 2: turboR-like, 5.35x a 3.58 MHz Z80 (calibrated against WebMSX turboR on a BASIC loop, 2026-09-23)
// cpu.cycles / cpu.insns are 32-bit and wrap every few minutes at full speed: keep the origin 32-bit too
// and work with differences modulo 2^32 (a 64-bit origin made the wrap look like minutes of waiting -> hang).
static uint32_t pace_cyc0; static uint32_t pace_t0;
static inline void pace(uint32_t now) {
    uint32_t t = cpu.cycles + cpu.insns;
    if (!pace_armed) { pace_armed = true; pace_cyc0 = t; pace_t0 = now; return; }
    uint32_t dt = t - pace_cyc0;                                          // T-states (+1 per M1) since the origin
    uint32_t target = (uint32_t)((uint64_t)dt * 1000000ull / (msx.speed == 2 ? TURBOR_HZ : 3579545ull));   // us the emulated CPU would need
    uint32_t elapsed = now - pace_t0;
    if (elapsed > target + PACE_SLIP_US || target - elapsed > 100000) { pace_cyc0 = t; pace_t0 = now; return; }   // fell behind, or something is off: re-arm (never wait > 100 ms)
    while ((uint32_t)(time_us_32() - pace_t0) < target) tight_loop_contents();
    if (dt > 0x40000000u) { pace_cyc0 = t; pace_t0 = now; }              // re-base well before the modular difference could wrap
}
void msx_set_speed(uint8_t speed) { msx.speed = speed > 2 ? 1 : speed; pace_armed = false; }   // 0 MSX2 (3.58 MHz), 1 max, 2 turboR-like

// Run for a short burst, then look at the pins. Called from the console loop.
#define CHUNK_T 300
#define CHUNK_T_PACED 48   // standard speed: short bursts so the catch-up stalls stay well under the FDC's 32 us/byte
static volatile uint8_t stop_flag;
static uint32_t last_refresh_us; static uint8_t refresh_row;

void __not_in_flash_func(msx_service)(void) {
    if (!msx.running) return;
    uint64_t hi = gpio_get_all64() >> 32;
    bool reset_low = !((hi >> (PIN_RESET - 32)) & 1);
    if (reset_low) {
        if (!msx.in_reset) { msx.in_reset = true; bus_drain(); }
        return;                                   // hold while /RESET is asserted
    }
    if (msx.in_reset) { msx.in_reset = false; msx_reset_cpu(); }
    cpu.int_line = !((hi >> (PIN_INT - 32)) & 1);
    z80_run(&cpu, cpu.cycles + (msx.speed == 1 ? CHUNK_T : msx.speed == 2 ? CHUNK_T_PACED * 5 : CHUNK_T_PACED), &stop_flag);
    msx.chunks++;
    if (z80_brk_hit) { z80_brk_hit = 0; msx.running = false; bus_drain(); printf("break: pc=%04x sp=%04x\n", cpu.pc, cpu.sp); return; }
    uint32_t now = time_us_32();
    if (msx.refresh_us && (uint32_t)(now - last_refresh_us) >= msx.refresh_us) { last_refresh_us = now; bus_refresh(refresh_row++); }
    if (bus_faulted()) { msx.faults++; msx.running = false; }
    if (msx.speed != 1) pace(now);
}

// ---- clock detect / autostart --------------------------------------------------------------
bool msx_clock_present(void) {
    // Sample CLK for 20 us: a 3.58 MHz clock toggles many times.
    int toggles = 0; bool last = gpio_get(PIN_CLK);
    absolute_time_t end = make_timeout_time_us(20);
    while (!time_reached(end)) { bool v = gpio_get(PIN_CLK); if (v != last) { toggles++; last = v; } }
    return toggles > 10;
}
uint32_t msx_clock_hz(void) {
    // Count rising edges of CLK for 200 us (polled at ~5 samples per 3.58 MHz period).
    uint32_t edges = 0; bool last = gpio_get(PIN_CLK);
    uint32_t t0 = time_us_32();
    while ((uint32_t)(time_us_32() - t0) < 200) { bool v = gpio_get(PIN_CLK); if (v && !last) edges++; last = v; }
    return edges < 20 ? 0 : edges * 5000;
}
#define BOOT_START_MAGIC 0x5A383053u   // watchdog scratch[0]: start the Z80 after this software reboot
void msx_request_start_after_reboot(void) { watchdog_hw->scratch[0] = BOOT_START_MAGIC; }
static bool autostart_done, reset_seen, power_on_boot, boot_start_requested;
// The RP2350 is powered by the MSX: a power-on/brown-out reset of the chip means the MSX was just switched on
// (a watchdog reboot after fwup/`reboot` is not). Sampling /RESET alone misses short reset pulses that end before
// the firmware is up.
void msx_autostart_init(void) {
    power_on_boot = (powman_hw->chip_reset & (POWMAN_CHIP_RESET_HAD_POR_BITS | POWMAN_CHIP_RESET_HAD_BOR_BITS)) != 0;
    boot_start_requested = watchdog_hw->scratch[0] == BOOT_START_MAGIC;   // fwup / `reboot`: restart the MSX from the BIOS
    watchdog_hw->scratch[0] = 0;
}
static uint32_t autostart_ms, autostart_tries, faults_at_start;
#define AUTOSTART_TRIES   4          // first start + 3 retries
#define AUTOSTART_RETRY_MS 300       // gap before a retry
#define AUTOSTART_WINDOW_MS 5000     // only faults this soon after a start are retried
void msx_autostart_poll(void) {
    if (!gpio_get(PIN_RESET)) reset_seen = true;                  // MSX power-on: /RESET is held low for a while
    if (msx.running || (!settings.autostart && !boot_start_requested)) return;
    uint32_t now = to_ms_since_boot(get_absolute_time());
    if (autostart_done) {
        // Retry after an early bus fault (seen once right after a cold boot): bounded, and only when the
        // stop came from a fault, not from a console `stop`.
        if (msx.faults == faults_at_start || autostart_tries >= AUTOSTART_TRIES) return;
        if (now - autostart_ms < AUTOSTART_RETRY_MS || now - autostart_ms > AUTOSTART_WINDOW_MS) return;
    } else if (!reset_seen && !power_on_boot && !boot_start_requested) return;             // bare RP reboot on a running MSX: RAM shadow is empty, do not start
    if (gpio_get(PIN_RESET) == 0) return;                         // still in reset
    if (!msx_clock_present()) return;
    autostart_done = true; autostart_tries++; autostart_ms = now;
    msx_start();
    faults_at_start = msx.faults;
    if (autostart_tries > 1) printf("autostart: retry %lu after bus fault\n", (unsigned long)autostart_tries - 1);
}
bool msx_power_on_boot(void) { return power_on_boot; }
uint32_t msx_uptime_s(void) { return to_ms_since_boot(get_absolute_time()) / 1000; }
