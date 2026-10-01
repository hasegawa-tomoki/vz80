// V9938 command engine emulation on core 1 (see vdpcmd.h). Only the VRAM shadow is touched; timing is
// irrelevant (the real chip paces the CPU through S#2.CE/TR), we only need the final VRAM contents.
#include <string.h>
#include "vdpcmd.h"
#include "vdp.h"
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/sync.h"

// ---- core 0 -> core 1 queue (single producer / single consumer) ----------------------------
#define QN 8192
static volatile uint16_t q[QN];            // reg << 8 | value ; 0xFF00 = reset
static volatile uint32_t q_head, q_tail;   // head: producer writes, tail: consumer reads
static vdpcmd_stats_t st;

void vdpcmd_push(uint8_t reg, uint8_t v) {
    uint32_t h = q_head, n = h - q_tail;
    if (n >= QN) { st.dropped++; return; }
    if (n > st.queue_max) st.queue_max = n;
    q[h & (QN - 1)] = (uint16_t)(reg << 8 | v);
    __dmb();
    q_head = h + 1;
    __sev();
}
static bool launched;
void vdpcmd_reset(void) { if (!launched) { q_head = q_tail = 0; return; } vdpcmd_push(0xFF, 0); vdpcmd_drain(); }
void vdpcmd_drain(void) { if (launched) while (q_tail != q_head) tight_loop_contents(); }
void vdpcmd_stats(vdpcmd_stats_t *s) { *s = st; }

// ---- dirty bits owned by core 1 ---------------------------------------------------------------
static uint32_t dirty1[VDP_BLOCKS / 32];
void vdpcmd_take_dirty(uint32_t *bits, unsigned words) {
    for (unsigned i = 0; i < words && i < VDP_BLOCKS / 32; i++) bits[i] |= __atomic_exchange_n(&dirty1[i], 0, __ATOMIC_ACQ_REL);
}

// ---- engine state (core 1 only) -----------------------------------------------------------------
static uint8_t R[64];              // R#0..R#46 as seen by the engine
static uint8_t *vram;
enum { MODE_NONE, G4, G5, G6, G7 };
static int mode;
static int shift;                  // pixels per byte = 1 << shift
static uint32_t pitch, width, height;

static void update_mode(void) {
    int m3 = R[0] >> 1 & 1, m4 = R[0] >> 2 & 1, m5 = R[0] >> 3 & 1;
    int code = m5 << 2 | m4 << 1 | m3;
    if (code == 3) mode = G4; else if (code == 4) mode = G5; else if (code == 5) mode = G6; else if (code == 7) mode = G7;
    else mode = (R[25] & 0x40) ? G7 : MODE_NONE;         // V9958: CMD bit allows commands in other modes (G7 addressing)
    switch (mode) {
        case G4: shift = 1; pitch = 128; width = 256; height = 1024; break;
        case G5: shift = 2; pitch = 128; width = 512; height = 1024; break;
        case G6: shift = 1; pitch = 256; width = 512; height = 512; break;
        case G7: shift = 0; pitch = 256; width = 256; height = 512; break;
        default: shift = 0; pitch = 256; width = 256; height = 512; break;
    }
}

static inline void mark(uint32_t a) { unsigned b = (a & (VDP_VRAM_SIZE - 1)) >> 9; __atomic_fetch_or(&dirty1[b >> 5], 1u << (b & 31), __ATOMIC_RELAXED); }
static inline uint32_t addr_of(uint32_t x, uint32_t y) { return (y * pitch + (x >> shift)) & (VDP_VRAM_SIZE - 1); }
static inline void wr_byte(uint32_t a, uint8_t v) { a &= VDP_VRAM_SIZE - 1; vram[a] = v; mark(a); }

static inline uint8_t rd_px(uint32_t x, uint32_t y) {
    uint8_t b = vram[addr_of(x, y)];
    switch (shift) {
        case 1: return (x & 1) ? b & 15 : b >> 4;
        case 2: return (b >> ((3 - (x & 3)) * 2)) & 3;
        default: return b;
    }
}
static inline uint8_t lop(uint8_t op, uint8_t dst, uint8_t src) {
    switch (op & 7) {
        case 0: return src;
        case 1: return dst & src;
        case 2: return dst | src;
        case 3: return dst ^ src;
        case 4: return (uint8_t)~src;
        default: return dst;
    }
}
static inline void wr_px(uint32_t x, uint32_t y, uint8_t c, uint8_t op) {
    if (x >= width || y >= height) return;
    if ((op & 8) && c == 0) return;                      // transparent ops leave color 0 alone
    uint32_t a = addr_of(x, y); uint8_t b = vram[a], n;
    switch (shift) {
        case 1: c &= 15; if (x & 1) n = (uint8_t)((b & 0xF0) | (lop(op, b & 15, c) & 15)); else n = (uint8_t)((b & 0x0F) | ((lop(op, b >> 4, c) & 15) << 4)); break;
        case 2: { int s = (3 - (x & 3)) * 2; c &= 3; n = (uint8_t)((b & ~(3 << s)) | (lop(op, (b >> s) & 3, c) & 3) << s); break; }
        default: n = lop(op, b, c); break;
    }
    if (n != b) { vram[a] = n; mark(a); }
}

// register helpers
#define SX  (uint32_t)(R[32] | (R[33] & 1) << 8)
#define SY  (uint32_t)(R[34] | (R[35] & 3) << 8)
#define DX  (uint32_t)(R[36] | (R[37] & 1) << 8)
#define DY  (uint32_t)(R[38] | (R[39] & 3) << 8)
#define NXR (uint32_t)(R[40] | (R[41] & 1) << 8)
#define NYR (uint32_t)(R[42] | (R[43] & 3) << 8)
#define CLR R[44]
#define ARG R[45]
#define DIX ((ARG & 4) ? -1 : 1)
#define DIY ((ARG & 8) ? -1 : 1)

// pending CPU->VRAM command (HMMC / LMMC): consumes R#44 writes
static struct { bool active; bool logical; uint32_t x, y, x0, nx, ny, cnt_x, cnt_y; uint8_t op; } pend;

static void cmd_pset(void) { wr_px(DX, DY, CLR, R[46] & 15); }

static void cmd_line(void) {
    uint32_t nx = NXR, ny = NYR; uint8_t op = R[46] & 15;   // LINE: NX/NY are plain counts (0 = no minor step)
    int32_t x = (int32_t)DX, y = (int32_t)DY; int tx = DIX, ty = DIY;
    uint32_t asx = nx ? (nx - 1) >> 1 : 0;
    for (uint32_t i = 0; ; i++) {
        wr_px((uint32_t)x, (uint32_t)y, CLR, op);
        if (i == nx) break;
        if (!(ARG & 1)) {                 // X major
            x += tx; if (x < 0 || (uint32_t)x >= width) break;
            if (asx < ny) { asx += nx; y += ty; }
            asx -= ny;
        } else {
            y += ty; if (y < 0 || (uint32_t)y >= height) break;
            if (asx < ny) { asx += nx; x += tx; }
            asx -= ny;
        }
    }
}

// pixel rectangle: rows*cols with per-pixel callback logic inlined for the 3 shapes
static void cmd_lmmv(void) {
    uint32_t nx = NXR ? NXR : 512, ny = NYR ? NYR : 1024; uint8_t op = R[46] & 15;
    int32_t x0 = (int32_t)DX, y = (int32_t)DY;
    for (uint32_t j = 0; j < ny && y >= 0 && (uint32_t)y < height; j++, y += DIY) {
        int32_t x = x0;
        for (uint32_t i = 0; i < nx && x >= 0 && (uint32_t)x < width; i++, x += DIX) wr_px((uint32_t)x, (uint32_t)y, CLR, op);
    }
}
static void cmd_lmmm(void) {
    uint32_t nx = NXR ? NXR : 512, ny = NYR ? NYR : 1024; uint8_t op = R[46] & 15;
    int32_t sx0 = (int32_t)SX, sy = (int32_t)SY, dx0 = (int32_t)DX, dy = (int32_t)DY;
    for (uint32_t j = 0; j < ny && sy >= 0 && dy >= 0 && (uint32_t)sy < height && (uint32_t)dy < height; j++, sy += DIY, dy += DIY) {
        int32_t sx = sx0, dx = dx0;
        for (uint32_t i = 0; i < nx && sx >= 0 && dx >= 0 && (uint32_t)sx < width && (uint32_t)dx < width; i++, sx += DIX, dx += DIX)
            wr_px((uint32_t)dx, (uint32_t)dy, rd_px((uint32_t)sx, (uint32_t)sy), op);
    }
}
static void cmd_hmmv(void) {
    uint32_t nx = (NXR ? NXR : 512) >> shift, ny = NYR ? NYR : 1024; uint32_t bpl = pitch;
    int32_t bx0 = (int32_t)(DX >> shift), y = (int32_t)DY;
    for (uint32_t j = 0; j < ny && y >= 0 && (uint32_t)y < height; j++, y += DIY) {
        int32_t bx = bx0;
        for (uint32_t i = 0; i < nx && bx >= 0 && (uint32_t)bx < bpl; i++, bx += DIX) wr_byte((uint32_t)y * pitch + bx, CLR);
    }
}
static void cmd_hmmm(void) {
    uint32_t nx = (NXR ? NXR : 512) >> shift, ny = NYR ? NYR : 1024; uint32_t bpl = pitch;
    int32_t sbx0 = (int32_t)(SX >> shift), sy = (int32_t)SY, dbx0 = (int32_t)(DX >> shift), dy = (int32_t)DY;
    for (uint32_t j = 0; j < ny && sy >= 0 && dy >= 0 && (uint32_t)sy < height && (uint32_t)dy < height; j++, sy += DIY, dy += DIY) {
        int32_t sbx = sbx0, dbx = dbx0;
        for (uint32_t i = 0; i < nx && sbx >= 0 && dbx >= 0 && (uint32_t)sbx < bpl && (uint32_t)dbx < bpl; i++, sbx += DIX, dbx += DIX)
            wr_byte((uint32_t)dy * pitch + dbx, vram[((uint32_t)sy * pitch + sbx) & (VDP_VRAM_SIZE - 1)]);
    }
}
static void cmd_ymmm(void) {
    // vertical move: rows SY.. -> DY.., X from DX to the edge in the DIX direction
    uint32_t ny = NYR ? NYR : 1024; uint32_t bpl = pitch;
    int32_t sy = (int32_t)SY, dy = (int32_t)DY, bx0 = (int32_t)(DX >> shift);
    for (uint32_t j = 0; j < ny && sy >= 0 && dy >= 0 && (uint32_t)sy < height && (uint32_t)dy < height; j++, sy += DIY, dy += DIY) {
        for (int32_t bx = bx0; bx >= 0 && (uint32_t)bx < bpl; bx += DIX)
            wr_byte((uint32_t)dy * pitch + bx, vram[((uint32_t)sy * pitch + bx) & (VDP_VRAM_SIZE - 1)]);
    }
}
static void cpu_data(uint8_t v);
static void start_cpu_to_vram(bool logical) {
    pend.active = true; pend.logical = logical; pend.op = R[46] & 15;
    pend.x0 = pend.x = logical ? DX : (DX >> shift) << shift; pend.y = DY;
    pend.nx = logical ? (NXR ? NXR : 512) : ((NXR ? NXR : 512) >> shift) << shift; pend.ny = NYR ? NYR : 1024;
    pend.cnt_x = 0; pend.cnt_y = 0;
    if (pend.nx == 0 || pend.ny == 0) pend.active = false;
    cpu_data(R[44]);   // the V9938 takes the first byte from R#44 as written BEFORE the command (verified against the real chip)
}
static void cpu_data(uint8_t v) {
    if (!pend.active) return;
    if (pend.logical) wr_px(pend.x, pend.y, v, pend.op);
    else if (pend.x < width && pend.y < height) wr_byte(pend.y * pitch + (pend.x >> shift), v);
    uint32_t step = pend.logical ? 1 : 1u << shift;
    pend.cnt_x += step; pend.x = (uint32_t)((int32_t)pend.x + DIX * (int32_t)step);
    if (pend.cnt_x >= pend.nx || pend.x >= width) {
        pend.cnt_x = 0; pend.x = pend.x0; pend.cnt_y++; pend.y = (uint32_t)((int32_t)pend.y + DIY);
        if (pend.cnt_y >= pend.ny || pend.y >= height) pend.active = false;
    }
}

#define LOGN 64
static vdpcmd_log_t logring[LOGN]; static volatile uint32_t log_n;
unsigned vdpcmd_log(vdpcmd_log_t *out, unsigned max) {
    uint32_t n = log_n, cnt = n < LOGN ? n : LOGN; if (cnt > max) cnt = max;
    for (uint32_t i = 0; i < cnt; i++) out[i] = logring[(n - cnt + i) % LOGN];
    return cnt;
}
static void execute(uint8_t cmd) {
    pend.active = false;
    st.last_cmd = cmd;
    logring[log_n % LOGN] = (vdpcmd_log_t){ time_us_32(), SX, SY, DX, DY, NXR, NYR, CLR, ARG, cmd, (uint8_t)mode }; log_n++;
    if (mode == MODE_NONE) return;
    switch (cmd >> 4) {
        case 0x5: cmd_pset(); break;
        case 0x7: cmd_line(); break;
        case 0x8: cmd_lmmv(); break;
        case 0x9: cmd_lmmm(); break;
        case 0xB: start_cpu_to_vram(true); break;
        case 0xC: cmd_hmmv(); break;
        case 0xD: cmd_hmmm(); break;
        case 0xE: cmd_ymmm(); break;
        case 0xF: start_cpu_to_vram(false); break;
        case 0x0: break;                                  // STOP
        case 0x4: case 0x6: case 0xA: break;              // POINT, SRCH, LMCM: read-only, results come from the real VDP
        default: st.unsupported++; break;
    }
    st.executed++;
}

static void core1_main(void) {
    multicore_lockout_victim_init();                     // core 0 parks us during flash programming
    vram = (uint8_t *)vdp_vram();
    update_mode();
    for (;;) {
        if (q_tail == q_head) { st.busy = false; __wfe(); continue; }
        st.busy = true;
        uint16_t e = q[q_tail & (QN - 1)];
        uint8_t reg = e >> 8, v = (uint8_t)e;
        if (reg == 0xFF) { pend.active = false; memset(R, 0, sizeof R); update_mode(); }
        else {
            R[reg & 63] = v;
            if (reg == 0 || reg == 1 || reg == 25) update_mode();
            else if (reg == 44) cpu_data(v);
            else if (reg == 46) execute(v);
        }
        __dmb();
        q_tail++;
    }
}

void vdpcmd_init(void) {
    launched = true;
    multicore_launch_core1(core1_main);
}
