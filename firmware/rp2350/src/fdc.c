#include "fdc.h"
#include "shelf.h"
#include "psram.h"
#include "settings.h"
#include "bus.h"
#include "pico/time.h"
#include "led.h"
#include "msx.h"
#include "z80_mem.h"
#include <string.h>
#include <stdio.h>
#include "pico/stdlib.h"

// Machine interface profile. The WD2793 itself is common to (almost) all MSX floppy interfaces; what
// differs per machine family is where its four registers sit and how drive select, side, motor,
// DRQ and IRQ are wired. Sony HB-F1XDJ, Philips NMS/VG and their clones share this layout
// ("Philips style"); other families (National, Toshiba, Sanyo, Victor...) get their own profile.
typedef struct {
    uint16_t base;            // address of the WD2793 status/command register (track/sector/data follow)
    uint16_t side_reg;        // side select register (bit 0)
    uint16_t sel_reg;         // drive select / motor register
    uint8_t  sel_mask;        // drive number bits in sel_reg (0 = A, 1 = B, other = none)
    uint8_t  motor_bit;       // motor on bit in sel_reg
    uint8_t  present_bit;     // bit read back from sel_reg that is 0 when the selected drive exists
    uint16_t stat2_reg;       // DRQ / IRQ status register
    uint8_t  drq_bit, irq_bit; bool stat2_inverted;   // stat2: DRQ/IRQ bits, active-low if inverted
    const char *chip, *iface;  // shown in the Web UI
    uint8_t  max_drives;       // internal drives this interface wiring can address
    uint8_t  idle_sel;         // select register value the ROM leaves when idle (no drive, motor off)
} fdc_profile_t;
static const fdc_profile_t PHILIPS = { 0x7FF8, 0x7FFC, 0x7FFD, 0x03, 0x80, 0x04, 0x7FFF, 0x80, 0x40, true, "WD2793", "Sony / Philips 方式 (7FF8h-7FFFh)", 2, 0x3F };
static const fdc_profile_t *prof = &PHILIPS;
// mode (A / B only): physical (hw = hardware drive) / virtual / none. kind (rows only): VK_FDD / VK_HDD.
// unit: PSRAM floppy image unit (-1 = none; the image is at img). HDD rows have no PSRAM copy: hslot is
// their index into the far-flash write-back state.
typedef struct { uint8_t mode, hw, kind; int shelf; bool wp; uint32_t size; uint8_t sides; uint8_t *img; int unit, hslot; } drive_t;
static drive_t drv[FD_UNITS];
static bool fdc_ready;               // fdc_init() ran (drv[], the unit pool and the PSRAM work area are valid); before that every entry point is a no-op
static int unit_owner[FDD_POOL];     // drive that holds each PSRAM unit, -1 = free
static int hslot_owner[HDD_MAX];
static int phys_count = -1;   // internal drives found by fdc_probe_physical()
static uint8_t changed_bits;
static uint8_t sel = 3, side, cmd, sel_latch = 0x3F; static bool motor;   // sel_latch: last value written to the select register (read back by the ROM)
static uint8_t r_track, r_sector, r_data, status; static bool busy, drq, irq, type1 = true, multi, wrcmd;
static uint8_t buf[512]; static int pos, len;
// write track (format) parser
static int wt_count, wt_state, wt_idpos; static uint8_t wt_id[4];
#define TRACKBUF_SZ (9 * 512)
static uint8_t *const trackbuf = (uint8_t *)PSRAM_WORK_TRK; static uint16_t wt_mask;   // formatted sectors of the track (PSRAM work area), written to flash at once
// Floppy sectors written to the flash cache but not yet copied back to the card, per PSRAM unit (2DD: 1440 sectors).
#define FD_SECTORS 1440
static uint32_t dirty_bits[FDD_POOL][FD_SECTORS / 32];
static void mark_dirty(int d, uint32_t off, uint32_t n) {
    uint32_t s0 = off / 512, s1 = (off + n + 511) / 512; int u = drv[d].unit;
    if (u < 0) return;
    for (uint32_t s = s0; s < s1 && s < FD_SECTORS; s++) dirty_bits[u][s >> 5] |= 1u << (s & 31);
    if (drv[d].shelf >= 0) shelf_set_dirty(drv[d].shelf, true);   // persisted: a full copy-back happens if we lose the bitmap
}

// The BIOS's DRVTBL (FB21h) tells who owns which drive letters (shown in the Web UI).
// Disk interfaces other than the internal ROM registered in DRVTBL (Nextor and the like).
static int ext_count(void) { uint8_t id = msx_disk_rom_slotid(); int n = 0; for (int i = 0; i < 4; i++) { uint8_t c = mem_rd((uint16_t)(0xFB21 + 2 * i)); if (c >= 1 && c <= 8 && mem_rd((uint16_t)(0xFB22 + 2 * i)) != id) n += c; } return n; }
static bool ext_master(void) { uint8_t n = mem_rd(0xFB21), s = mem_rd(0xFB22); return n >= 1 && n <= 8 && s != msx_disk_rom_slotid(); }   // a Nextor with up to 6 rows
static int internal_count(void) { uint8_t id = msx_disk_rom_slotid(); for (int i = 0; i < 4; i++) if (mem_rd((uint16_t)(0xFB22 + 2 * i)) == id) { uint8_t n = mem_rd((uint16_t)(0xFB21 + 2 * i)); return n <= 2 ? n : 0; } return 0; }
static inline bool sel_virtual(void) { return sel < 2 && drv[sel].mode == FD_VIRTUAL; }
static inline bool sel_physical(void) { return sel < 2 && drv[sel].mode == FD_PHYSICAL; }
static inline bool index_pulse(void) { return motor && (time_us_32() % 200000u) < 1500u; }
static inline bool mounted(int d) { return drv[d].shelf >= 0 && drv[d].img != NULL; }   // an image is mounted whatever the internal-ROM mode: the vz80 disk port (Nextor) uses it too

static bool locate(uint32_t *off) {
    drive_t *d = &drv[sel];
    if (!mounted(sel) || r_sector < 1 || r_sector > 9 || side >= d->sides || r_track >= 80) return false;
    *off = ((uint32_t)(r_track * d->sides + side) * 9 + (r_sector - 1)) * 512;
    return *off + 512 <= d->size;
}
static void finish(uint8_t st) { busy = false; drq = false; irq = true; status = st; type1 = false; }
static void start_transfer(void) {
    uint32_t off;
    if (!locate(&off)) { finish(0x10); return; }                     // record not found
    led_note_disk();
    if (wrcmd) { if (drv[sel].wp) { finish(0x40); return; } pos = 0; len = 512; }
    else { memcpy(buf, drv[sel].img + off, 512); pos = 0; len = 512; }
    busy = true; drq = true; irq = false; type1 = false; status = 0x03;
}
static void commit_sector(void) {
    uint32_t off;
    if (locate(&off)) {
        memcpy(drv[sel].img + off, buf, 512);                                       // the PSRAM copy is what reads see
        if (!shelf_write(drv[sel].shelf, off, buf, 512)) { finish(0x20); return; }   // write fault (flash copy)
        mark_dirty(sel, off, 512);
    }
    if (multi) { r_sector++; start_transfer(); } else finish(0x00);
}
static void command(uint8_t c) {
    cmd = c;
    if ((c & 0xF0) == 0xD0) {                                          // force interrupt
        busy = false; drq = false; type1 = true; irq = (c & 0x08) != 0; wt_state = -1; return;
    }
    if (busy) return;
    irq = false;
    if (!(c & 0x80)) {                                                 // type I
        int dir = 1; uint8_t t = c >> 4;
        if (t == 0) r_track = 0;
        else if (t == 1) r_track = r_data;
        else { if (t >= 6) dir = -1; else if (t >= 4) dir = 1; if (t >= 2 && t <= 3) dir = (r_track ? dir : 1); if (c & 0x10 || t >= 4) r_track = (uint8_t)(r_track + dir); }
        if (r_track > 79) r_track = r_track > 200 ? 0 : 79;
        type1 = true; busy = false; drq = false; irq = true; return;
    }
    multi = (c & 0x10) != 0;
    switch (c & 0xE0) {
        case 0x80: wrcmd = false; start_transfer(); break;              // read sector
        case 0xA0: wrcmd = true; start_transfer(); break;               // write sector
        case 0xC0: {                                                    // read address
            if (!mounted(sel)) { finish(0x10); break; }
            buf[0] = r_track; buf[1] = side; buf[2] = 1; buf[3] = 2; buf[4] = 0; buf[5] = 0; pos = 0; len = 6; r_sector = r_track;
            busy = true; drq = true; wrcmd = false; type1 = false; status = 0x03; break; }
        case 0xE0: finish(0x10); break;                                 // read track: not supported
        case 0xF0:                                                      // write track (format)
            if (!mounted(sel)) { finish(0x10); break; }
            if (drv[sel].wp) { finish(0x40); break; }
            wt_count = 0; wt_state = 0; wt_mask = 0; busy = true; drq = true; wrcmd = true; type1 = false; status = 0x03; break;
        default: finish(0x10); break;
    }
}
static void write_track_flush(void) {
    // Whole track (sectors 1..9 of the selected side) in one contiguous run of the image.
    drive_t *d = &drv[sel];
    if (!mounted(sel) || side >= d->sides || r_track >= 80) return;
    uint32_t off = (uint32_t)(r_track * d->sides + side) * 9 * 512;
    if (off + TRACKBUF_SZ > d->size) return;
    for (int i = 0; i < 9; i++) if (!(wt_mask & (1 << i))) memcpy(trackbuf + i * 512, d->img + off + i * 512, 512);   // keep sectors the format did not write
    memcpy(d->img + off, trackbuf, TRACKBUF_SZ);
    shelf_write(d->shelf, off, trackbuf, TRACKBUF_SZ);
    mark_dirty(sel, off, TRACKBUF_SZ);
}
static void write_track_byte(uint8_t v) {
    wt_count++;
    switch (wt_state) {
        case 0: if (v == 0xFE) { wt_state = 1; wt_idpos = 0; } else if (v == 0xFB) { wt_state = 2; pos = 0; } break;
        case 1: wt_id[wt_idpos++] = v; if (wt_idpos == 4) wt_state = 0; break;                 // track side sector len
        case 2: buf[pos++] = v;
            if (pos == 512) {
                int sec = wt_id[2];
                if (sec >= 1 && sec <= 9) { memcpy(trackbuf + (sec - 1) * 512, buf, 512); wt_mask |= (uint16_t)(1 << (sec - 1)); }
                wt_state = 0;
            }
            break;
        default: break;
    }
    if (wt_count >= 6250) { write_track_flush(); finish(0x00); }        // one revolution
}

bool fdc_read(uint16_t a, uint8_t *v) {
    if (!sel_virtual()) {
        if (a == prof->sel_reg && sel >= 2 && (drv[0].mode == FD_VIRTUAL || drv[1].mode == FD_VIRTUAL) && drv[0].mode != FD_PHYSICAL && drv[1].mode != FD_PHYSICAL) { *v = 0xBF; return true; }
        *v = bus_mem_read(a); return true;                              // physical drive or nothing selected: the real FDC
    }
    if (a == prof->side_reg) { *v = (uint8_t)(0xF8 | side); return true; }
    if (a == prof->sel_reg) { *v = (uint8_t)(sel_latch & ~prof->present_bit); return true; }   // a virtual drive is always present (empty = no disk); the ROM saves, changes and restores this value
    if (a == prof->stat2_reg) {
        uint8_t b = (uint8_t)((drq ? prof->drq_bit : 0) | (irq ? prof->irq_bit : 0));
        *v = prof->stat2_inverted ? (uint8_t)(~b) : (uint8_t)(b | (0xFF & ~prof->drq_bit & ~prof->irq_bit)); return true;
    }
    switch (a - prof->base) {
        case 0:
            irq = false;
            if (type1) *v = (uint8_t)((motor ? 0 : 0x80) | 0x20 | (r_track == 0 ? 4 : 0) | (index_pulse() ? 2 : 0) | (busy ? 1 : 0));
            else *v = (uint8_t)((motor ? 0 : 0x80) | (busy ? (status | (drq ? 2 : 0)) : status));
            return true;
        case 1: *v = r_track; return true;
        case 2: *v = r_sector; return true;
        case 3:
            if (drq && !wrcmd) { *v = buf[pos++]; if (pos >= len) { if (len == 512 && multi) { r_sector++; start_transfer(); } else finish(0x00); } }
            else *v = r_data;
            return true;
        default: *v = 0xFF; return true;
    }
}
void fdc_write(uint16_t a, uint8_t v) {
    if (a == prof->sel_reg) {
        sel = v & prof->sel_mask; motor = (v & prof->motor_bit) != 0; sel_latch = v;
        if (sel_virtual()) return;                                       // virtual: nothing on the bus
        if (sel_physical()) v = (uint8_t)((v & ~prof->sel_mask) | drv[sel].hw);   // logical drive -> its hardware drive
        bus_mem_write(a, v); return;
    }
    // The WD2793 is one chip shared by both drives and the ROM programs its registers before selecting
    // the drive: keep a mirror of every register write regardless of the selection.
    if (a == prof->side_reg) side = v & 1;
    else if (a == prof->base + 1) r_track = v;
    else if (a == prof->base + 2) r_sector = v;
    else if (a == prof->base + 3) r_data = v;
    if (!sel_virtual()) { bus_mem_write(a, v); return; }
    if (a == prof->base) command(v);
    else if (a == prof->base + 3 && drq && wrcmd) { if (wt_state >= 0 && (cmd & 0xF0) == 0xF0) write_track_byte(v); else { buf[pos++] = v; if (pos >= 512) commit_sector(); } }
}


// ---- hard disk rows: far flash in place, with a small write-back cache ------------------------------
// Reads come straight from the flash copy (shelf_read). Writes gather in 4 KB blocks kept in PSRAM (the
// 64 KB above the mapper RAM expansion is free: segments 4..63 end at 0xF0000) and go to the flash when
// the block is evicted, on eject, or after HDD_FLUSH_MS of quiet: one 4 KB erase + program per block
// instead of one per sector. Blocks written are also noted for the copy-back to the card (4 KB records).
#define HDD_WORK   PSRAM_WORK_HDD
#define WB_SLOTS   4
#define HDD_BLOCKS (HDD_MAX_BYTES / 4096u)
#define HDD_FLUSH_MS 300
typedef struct { int shelf; uint32_t base; bool dirty; uint32_t used; } wb_t;   // base: byte offset of the block in the image
static wb_t wb[WB_SLOTS]; static uint32_t wb_seq, wb_last_ms;
#define WB_DATA(s) ((uint8_t *)(HDD_WORK + (uint32_t)(s) * 4096u))
#define HDD_DIRTY(h) ((uint32_t *)(HDD_WORK + 0x4000u + (uint32_t)(h) * (HDD_BLOCKS / 8)))   // HDD_MAX x 1536 bytes
static bool wb_flush(int s) {
    if (wb[s].shelf < 0 || !wb[s].dirty) return true;
    led_note_disk();
    bool ok = shelf_write(wb[s].shelf, wb[s].base, WB_DATA(s), 4096);
    if (ok) wb[s].dirty = false;
    return ok;
}
static void wb_flush_all(void) { for (int s = 0; s < WB_SLOTS; s++) wb_flush(s); }
static void wb_drop(int shelf) { for (int s = 0; s < WB_SLOTS; s++) if (wb[s].shelf == shelf) { wb_flush(s); wb[s].shelf = -1; } }
static int wb_find(int shelf, uint32_t base) { for (int s = 0; s < WB_SLOTS; s++) if (wb[s].shelf == shelf && wb[s].base == base) return s; return -1; }
static int wb_get(int shelf, uint32_t base) {              // slot holding that block, loaded from the flash if needed
    int s = wb_find(shelf, base);
    if (s >= 0) return s;
    int victim = 0;
    for (int i = 1; i < WB_SLOTS; i++) if (wb[i].shelf < 0 || (wb[victim].shelf >= 0 && wb[i].used < wb[victim].used)) victim = i;
    if (!wb_flush(victim)) return -1;
    if (!shelf_read(shelf, base, WB_DATA(victim), 4096)) { wb[victim].shelf = -1; return -1; }
    wb[victim].shelf = shelf; wb[victim].base = base; wb[victim].dirty = false;
    return victim;
}
static void hdd_mark(int d, uint32_t block) {
    int h = drv[d].hslot; if (h < 0 || block >= HDD_BLOCKS) return;
    HDD_DIRTY(h)[block >> 5] |= 1u << (block & 31);
    if (drv[d].shelf >= 0) shelf_set_dirty(drv[d].shelf, true);
}

// ---- configuration ----------------------------------------------------------------------------
static bool is_row(int d) { return d >= FD_INT && d < FD_UNITS && drv[d].kind != VK_NONE; }
static int unit_alloc(int d) {
    if (drv[d].unit >= 0) return drv[d].unit;
    for (int u = 0; u < FDD_POOL; u++) if (unit_owner[u] < 0) { unit_owner[u] = d; drv[d].unit = u; memset(dirty_bits[u], 0, sizeof dirty_bits[u]); return u; }
    return -1;
}
static void unit_free(int d) { if (drv[d].unit >= 0) { unit_owner[drv[d].unit] = -1; drv[d].unit = -1; } drv[d].img = NULL; drv[d].size = 0; }
static int hslot_alloc(int d) {
    if (drv[d].hslot >= 0) return drv[d].hslot;
    for (int h = 0; h < HDD_MAX; h++) if (hslot_owner[h] < 0) { hslot_owner[h] = d; drv[d].hslot = h; memset(HDD_DIRTY(h), 0, HDD_BLOCKS / 8); return h; }
    return -1;
}
static void hslot_free(int d) { if (drv[d].hslot >= 0) { hslot_owner[drv[d].hslot] = -1; drv[d].hslot = -1; } drv[d].size = 0; }
static bool wants_unit(int d) { return d < FD_INT ? drv[d].mode == FD_VIRTUAL : drv[d].kind == VK_FDD; }
static int units_in_use(void) { int n = 0; for (int d = 0; d < FD_UNITS; d++) if (wants_unit(d)) n++; return n; }
static int hdd_rows(void) { int n = 0; for (int d = FD_INT; d < FD_UNITS; d++) if (drv[d].kind == VK_HDD) n++; return n; }
static void load_image(int d) {
    drive_t *x = &drv[d]; const shelf_ent_t *e = shelf_get(x->shelf);
    changed_bits |= (uint8_t)(1 << d);
    if (d >= FD_INT && x->kind == VK_HDD) {
        wb_drop(x->shelf);
        if (!e || e->type != IMG_HDD || e->size > HDD_MAX_BYTES || hslot_alloc(d) < 0) { x->shelf = -1; x->size = 0; return; }
        x->size = e->size; return;
    }
    x->img = NULL; x->size = 0;
    if (!e || e->type != IMG_DISK || e->size > PSRAM_FDD_MAX) { x->shelf = -1; return; }
    if (!wants_unit(d) || unit_alloc(d) < 0) { x->shelf = -1; return; }
    uint8_t *img = (uint8_t *)PSRAM_FDD(x->unit);
    if (!shelf_read(x->shelf, 0, img, e->size)) { x->shelf = -1; return; }   // XIP memcpy or far flash
    x->img = img; x->size = e->size; x->sides = e->size > 400000 ? 2 : 1;
}
static uint32_t row_word(int d) { return ((uint32_t)drv[d].kind << 28) | (uint32_t)(drv[d].shelf + 1) | (drv[d].wp ? 1u << 24 : 0); }
static void save_cfg(void) {
    for (int d = 0; d < FD_INT; d++) settings.fd_cfg[d] = (uint32_t)drv[d].mode | ((uint32_t)drv[d].hw << 4) | ((uint32_t)(drv[d].shelf + 1) << 8) | (drv[d].wp ? 1u << 24 : 0);
    for (int i = 0; i < VD_MAX; i++) settings.vd[i] = is_row(FD_INT + i) ? row_word(FD_INT + i) : 0;
    for (int i = 0; i < 2; i++) settings.fd_port[i] = (is_row(FD_INT + i) && drv[FD_INT + i].kind == VK_FDD) ? (settings.vd[i] & 0x010000FFu) : 0;   // what a 0.4.1 firmware would read
    settings_save();
}
static void compact_rows(void) {     // rows are the first fdc_vd_count() units after A / B
    int w = FD_INT;
    for (int d = FD_INT; d < FD_UNITS; d++) if (drv[d].kind != VK_NONE) { if (w != d) { drv[w] = drv[d]; if (drv[w].unit >= 0) unit_owner[drv[w].unit] = w; if (drv[w].hslot >= 0) hslot_owner[drv[w].hslot] = w; memset(&drv[d], 0, sizeof drv[d]); drv[d].shelf = -1; drv[d].unit = drv[d].hslot = -1; } w++; }
    changed_bits |= 0xFC;
}
int fdc_vd_count(void) { int n = 0; if (!fdc_ready) return 0; for (int d = FD_INT; d < FD_UNITS; d++) if (drv[d].kind != VK_NONE) n++; return n; }
static uint32_t boot_sig;            // row kinds in order at the last MSX start
static uint32_t rows_sig(void) { uint32_t s = 0; for (int d = FD_INT; d < FD_UNITS; d++) s = s * 4 + drv[d].kind; return s; }
bool fdc_restart_needed(void) { return rows_sig() != boot_sig; }
int fdc_unit_by_name(const char *s) {
    if (!s || !s[0]) return -1;
    if ((s[0] | 0x20) == 'a' && !s[1]) return 0;
    if ((s[0] | 0x20) == 'b' && !s[1]) return 1;
    if ((s[0] | 0x20) == 'v' && s[1] >= '1' && s[1] <= '0' + VD_MAX && !s[2]) return FD_INT + (s[1] - '1');
    if ((s[0] | 0x20) == 'p' && (s[1] == '1' || s[1] == '2') && !s[2]) return FD_INT + (s[1] - '1');   // 0.4.1 names
    return -1;
}
const char *fdc_unit_name(int d) { static const char *n[FD_UNITS] = { "A", "B", "V1", "V2", "V3", "V4", "V5", "V6" }; return d >= 0 && d < FD_UNITS ? n[d] : "?"; }
void fdc_init(void) {
    wb_flush_all();
    for (int u = 0; u < FDD_POOL; u++) unit_owner[u] = -1;
    for (int h = 0; h < HDD_MAX; h++) hslot_owner[h] = -1;
    for (int s = 0; s < WB_SLOTS; s++) wb[s].shelf = -1;
    memset(drv, 0, sizeof drv);
    for (int d = 0; d < FD_UNITS; d++) { drv[d].shelf = -1; drv[d].unit = drv[d].hslot = -1; }
    for (int d = 0; d < FD_INT; d++) {
        uint32_t s = settings.fd_cfg[d];
        drv[d].mode = (uint8_t)(s & 0x0F); if (drv[d].mode > FD_NONE) drv[d].mode = d == 0 ? FD_PHYSICAL : FD_NONE;
        drv[d].hw = (uint8_t)((s >> 4) & 1);
        drv[d].shelf = (int)((s >> 8) & 0xFF) - 1; drv[d].wp = (s >> 24) & 1;
    }
    bool any = false; for (int i = 0; i < VD_MAX; i++) if (settings.vd[i]) any = true;
    if (!any) for (int i = 0; i < 2; i++) settings.vd[i] = ((uint32_t)VK_FDD << 28) | (settings.fd_port[i] & 0x010000FFu);   // first boot of 0.4.2: the two disk-port units become rows V1 / V2
    for (int i = 0; i < VD_MAX; i++) {
        uint32_t s = settings.vd[i]; int d = FD_INT + i; uint8_t k = (uint8_t)(s >> 28);
        if (k != VK_FDD && k != VK_HDD) continue;
        drv[d].kind = k; drv[d].mode = FD_VIRTUAL; drv[d].shelf = (int)(s & 0xFF) - 1; drv[d].wp = (s >> 24) & 1;
    }
    compact_rows();
    if (drv[0].mode == FD_PHYSICAL && drv[1].mode == FD_PHYSICAL && drv[0].hw == drv[1].hw) drv[1].mode = FD_NONE;   // one logical drive per hardware drive
    for (int d = 0; d < FD_UNITS; d++) if (d < FD_INT || is_row(d)) load_image(d);
    boot_sig = rows_sig();
    sel = 3; motor = false; busy = drq = irq = false; type1 = true;
    fdc_ready = true;
}
bool fdc_set_mode(int d, int mode, int hw, char *err, size_t n) {
    if (d < 0 || d >= FD_INT || mode < 0 || mode > FD_NONE || hw < 0 || hw > 1) { snprintf(err, n, "drive A|B, mode physical [1|2]|virtual|none"); return false; }
    if (mode == FD_PHYSICAL && hw >= (phys_count > 0 ? phys_count : 1)) { snprintf(err, n, "the machine has %d internal drive(s)", phys_count > 0 ? phys_count : 1); return false; }
    if (mode == FD_VIRTUAL && drv[d].mode != FD_VIRTUAL && units_in_use() >= FDD_POOL) { snprintf(err, n, "no floppy unit left: %d virtual floppies at most (internal drives and Nextor FDD rows together)", FDD_POOL); return false; }
    if (mode == FD_PHYSICAL && drv[1 - d].mode == FD_PHYSICAL && drv[1 - d].hw == hw) drv[1 - d].mode = FD_NONE;   // one logical drive per hardware drive
    drv[d].mode = (uint8_t)mode; if (mode == FD_PHYSICAL) drv[d].hw = (uint8_t)hw;
    if (mode != FD_VIRTUAL) { drv[d].shelf = -1; unit_free(d); }     // a drive that is not virtual any more holds no image
    load_image(d); save_cfg(); return true;
}
bool fdc_mount(int d, int idx, bool wp, char *err, size_t n) {
    const shelf_ent_t *e = shelf_get(idx);
    if (d < 0 || d >= FD_UNITS || (d >= FD_INT && !is_row(d))) { snprintf(err, n, "drive A|B|V1..V%d", fdc_vd_count()); return false; }
    bool hdd = d >= FD_INT && drv[d].kind == VK_HDD;
    if (!e) { snprintf(err, n, "no such cache entry"); return false; }
    if (hdd) {
        if (e->type != IMG_HDD) { snprintf(err, n, "not a hard disk image"); return false; }
        if (e->size > HDD_MAX_BYTES || e->size < 4096 || (e->size & 511)) { snprintf(err, n, "unsupported image size %lu (whole sectors, %u MB at most)", (unsigned long)e->size, (unsigned)(HDD_MAX_BYTES >> 20)); return false; }
    } else {
        if (e->type != IMG_DISK) { snprintf(err, n, "not a floppy image"); return false; }
        if (e->size != 737280 && e->size != 368640) { snprintf(err, n, "unsupported image size %lu (720 KB / 360 KB raw)", (unsigned long)e->size); return false; }
        if (d < FD_INT && drv[d].mode != FD_VIRTUAL && units_in_use() >= FDD_POOL) { snprintf(err, n, "no floppy unit left: %d virtual floppies at most", FDD_POOL); return false; }
    }
    for (int o = 0; o < FD_UNITS; o++) if (o != d && drv[o].shelf == idx) { drv[o].shelf = -1; load_image(o); }   // one mount per image
    if (d < FD_INT) drv[d].mode = FD_VIRTUAL;
    drv[d].shelf = idx; drv[d].wp = wp; load_image(d);
    if (drv[d].shelf < 0) { snprintf(err, n, hdd ? "cannot open the image" : "no floppy unit left: %d virtual floppies at most", FDD_POOL); save_cfg(); return false; }
    shelf_touch(idx); save_cfg(); return true;
}
void fdc_eject(int d) { if (d < 0 || d >= FD_UNITS) return; if (drv[d].kind == VK_HDD) wb_drop(drv[d].shelf); drv[d].shelf = -1; load_image(d); save_cfg(); }
bool fdc_swap(int a, int b, char *err, size_t n) {
    if (a < 0 || a >= FD_UNITS || b < 0 || b >= FD_UNITS || a == b || (a >= FD_INT && !is_row(a)) || (b >= FD_INT && !is_row(b))) { snprintf(err, n, "two different drives A|B|V1.."); return false; }
    if (drv[a].mode != FD_VIRTUAL || drv[b].mode != FD_VIRTUAL) { snprintf(err, n, "both drives must be virtual"); return false; }
    if ((a >= FD_INT && drv[a].kind == VK_HDD) != (b >= FD_INT && drv[b].kind == VK_HDD)) { snprintf(err, n, "a floppy and a hard disk cannot swap"); return false; }
    wb_flush_all();
    int sh = drv[a].shelf; bool wp = drv[a].wp; drv[a].shelf = drv[b].shelf; drv[a].wp = drv[b].wp; drv[b].shelf = sh; drv[b].wp = wp;
    load_image(a); load_image(b); save_cfg(); return true;
}
bool fdc_vd_add(int kind, char *err, size_t n) {
    int cnt = fdc_vd_count();
    if (kind != VK_FDD && kind != VK_HDD) { snprintf(err, n, "vd add fdd|hdd"); return false; }
    if (cnt >= VD_MAX) { snprintf(err, n, "%d rows at most", VD_MAX); return false; }
    if (kind == VK_FDD && units_in_use() >= FDD_POOL) { snprintf(err, n, "no floppy unit left: %d virtual floppies at most (internal drives and Nextor FDD rows together)", FDD_POOL); return false; }
    if (kind == VK_HDD && hdd_rows() >= HDD_MAX) { snprintf(err, n, "%d hard disks at most", HDD_MAX); return false; }
    int d = FD_INT + cnt; drv[d].kind = (uint8_t)kind; drv[d].mode = FD_VIRTUAL; drv[d].shelf = -1; drv[d].wp = false; drv[d].unit = drv[d].hslot = -1;
    if (kind == VK_FDD) unit_alloc(d); else hslot_alloc(d);
    changed_bits |= (uint8_t)(1 << d); save_cfg(); return true;
}
bool fdc_vd_del(int row, char *err, size_t n) {
    int d = FD_INT + row;
    if (row < 0 || row >= fdc_vd_count()) { snprintf(err, n, "no such row"); return false; }
    if (fdc_vd_count() <= 1) { snprintf(err, n, "the last row stays (Nextor needs one drive)"); return false; }
    if (drv[d].kind == VK_HDD) wb_drop(drv[d].shelf);
    unit_free(d); hslot_free(d); memset(&drv[d], 0, sizeof drv[d]); drv[d].shelf = -1; drv[d].unit = drv[d].hslot = -1;
    compact_rows(); save_cfg(); return true;
}
bool fdc_vd_order(const int *rows, int count, char *err, size_t n) {
    int cnt = fdc_vd_count(); bool seen[VD_MAX] = { 0 };
    if (count != cnt) { snprintf(err, n, "list all %d rows", cnt); return false; }
    for (int i = 0; i < count; i++) { if (rows[i] < 0 || rows[i] >= cnt || seen[rows[i]]) { snprintf(err, n, "each row once, 1..%d", cnt); return false; } seen[rows[i]] = true; }
    drive_t tmp[VD_MAX]; for (int i = 0; i < cnt; i++) tmp[i] = drv[FD_INT + rows[i]];
    for (int i = 0; i < cnt; i++) { drv[FD_INT + i] = tmp[i]; if (tmp[i].unit >= 0) unit_owner[tmp[i].unit] = FD_INT + i; if (tmp[i].hslot >= 0) hslot_owner[tmp[i].hslot] = FD_INT + i; }
    changed_bits |= 0xFC; save_cfg(); return true;
}
// ---- access for the vz80 disk port (vzdisk.c) -------------------------------------------------
int fdc_vd_kind(int row) { if (!fdc_ready) return 0; int d = FD_INT + row; return row >= 0 && row < VD_MAX ? drv[d].kind : VK_NONE; }
uint32_t fdc_vd_sectors(int row) { if (!fdc_ready) return 0; int d = FD_INT + row; return row >= 0 && row < VD_MAX && is_row(d) && drv[d].shelf >= 0 ? drv[d].size / 512 : 0; }
uint8_t fdc_vd_read(int row, uint32_t sector, uint8_t *buf) {
    if (!fdc_ready) return 2;
    int d = FD_INT + row;
    if (row < 0 || row >= VD_MAX || !is_row(d) || drv[d].shelf < 0) return 2;
    if ((sector + 1) * 512 > drv[d].size) return 8;
    led_note_disk();
    if (drv[d].kind == VK_FDD) { if (!drv[d].img) return 2; memcpy(buf, drv[d].img + sector * 512, 512); return 0; }
    uint32_t off = sector * 512, base = off & ~4095u; int s = wb_find(drv[d].shelf, base);
    if (s >= 0) { memcpy(buf, WB_DATA(s) + (off - base), 512); return 0; }
    return shelf_read(drv[d].shelf, off, buf, 512) ? 0 : 10;
}
uint8_t fdc_vd_write(int row, uint32_t sector, const uint8_t *buf) {
    if (!fdc_ready) return 2;
    int d = FD_INT + row;
    if (row < 0 || row >= VD_MAX || !is_row(d) || drv[d].shelf < 0) return 2;
    if (drv[d].wp) return 1;
    if ((sector + 1) * 512 > drv[d].size) return 8;
    led_note_disk();
    if (drv[d].kind == VK_FDD) {
        if (!drv[d].img) return 2;
        memcpy(drv[d].img + sector * 512, buf, 512);
        if (!shelf_write(drv[d].shelf, sector * 512, buf, 512)) return 10;
        mark_dirty(d, sector * 512, 512); return 0;
    }
    uint32_t off = sector * 512, base = off & ~4095u; int s = wb_get(drv[d].shelf, base);
    if (s < 0) return 10;
    memcpy(WB_DATA(s) + (off - base), buf, 512); wb[s].dirty = true; wb[s].used = ++wb_seq; wb_last_ms = to_ms_since_boot(get_absolute_time());
    hdd_mark(d, base / 4096); return 0;
}
uint8_t fdc_take_changed(void) { uint8_t b = (uint8_t)(changed_bits >> FD_INT); changed_bits &= 3; return b; }
void fdc_shelf_deleted(int i) {                   // catalog entry i removed: indices above it shift down
    bool ch = false;
    if (!fdc_ready) return;
    for (int s = 0; s < WB_SLOTS; s++) { if (wb[s].shelf == i) wb[s].shelf = -1; else if (wb[s].shelf > i) wb[s].shelf--; }
    for (int d = 0; d < FD_UNITS; d++) { if (drv[d].shelf == i) { drv[d].shelf = -1; load_image(d); ch = true; } else if (drv[d].shelf > i) { drv[d].shelf--; ch = true; } }
    if (ch) save_cfg();
}
bool fdc_uses_shelf(int shelf_idx) { if (!fdc_ready) return false; for (int d = 0; d < FD_UNITS; d++) if (drv[d].shelf == shelf_idx) return true; return false; }
bool fdc_has_dirty(void) {
    if (!fdc_ready) return false;
    for (int u = 0; u < FDD_POOL; u++) for (int w = 0; w < FD_SECTORS / 32; w++) if (dirty_bits[u][w]) return true;
    for (int h = 0; h < HDD_MAX; h++) for (uint32_t w = 0; w < HDD_BLOCKS / 32; w++) if (HDD_DIRTY(h)[w]) return true;
    return false;
}
void fdc_mark_dirty_sector(int shelf_idx, uint32_t sector) {   // the card write failed on the ESP: try again later
    if (!fdc_ready) return;
    for (int d = 0; d < FD_UNITS; d++) if (drv[d].shelf == shelf_idx && drv[d].unit >= 0 && sector < FD_SECTORS) dirty_bits[drv[d].unit][sector >> 5] |= 1u << (sector & 31);
}
void fdc_mark_dirty_block(int shelf_idx, uint32_t block) {
    if (!fdc_ready) return;
    for (int d = FD_INT; d < FD_UNITS; d++) if (drv[d].shelf == shelf_idx && drv[d].hslot >= 0) hdd_mark(d, block);
}
bool fdc_shelf_clean(int shelf_idx) {                          // no unsynced sectors left for this image?
    if (!fdc_ready) return true;
    for (int d = 0; d < FD_UNITS; d++) {
        if (drv[d].shelf != shelf_idx) continue;
        if (drv[d].unit >= 0) for (int w = 0; w < FD_SECTORS / 32; w++) if (dirty_bits[drv[d].unit][w]) return false;
        if (drv[d].hslot >= 0) { for (int s = 0; s < WB_SLOTS; s++) if (wb[s].shelf == shelf_idx && wb[s].dirty) return false; for (uint32_t w = 0; w < HDD_BLOCKS / 32; w++) if (HDD_DIRTY(drv[d].hslot)[w]) return false; }
    }
    return true;
}
// Up to `max` dirty floppy sectors as records { u8 shelf, u8 pad, u16 sector, u8 data[512] }; the bits are
// cleared on the way out (the ESP re-marks a sector with `fddirty` if the card write fails).
int fdc_take_dirty(uint8_t *out, int max) {
    int n = 0;
    if (!fdc_ready) return 0;
    for (int d = 0; d < FD_UNITS && n < max; d++) {
        int u = drv[d].unit; if (u < 0) continue;
        if (drv[d].shelf < 0 || !drv[d].img) { memset(dirty_bits[u], 0, sizeof dirty_bits[u]); continue; }
        for (uint32_t s = 0; s < FD_SECTORS && n < max; s++) {
            if (!(dirty_bits[u][s >> 5] >> (s & 31) & 1)) continue;
            dirty_bits[u][s >> 5] &= ~(1u << (s & 31));
            uint8_t *r = out + n * 516;
            r[0] = (uint8_t)drv[d].shelf; r[1] = 0; r[2] = (uint8_t)s; r[3] = (uint8_t)(s >> 8);
            memcpy(r + 4, drv[d].img + s * 512, 512);
            n++;
        }
    }
    return n;
}
// One dirty hard disk block as { u8 shelf, u8 pad[3], u32 block, u8 data[4096] } (the flash copy, flushed first).
int fdc_take_hdd_dirty(uint8_t *out) {
    if (!fdc_ready) return 0;
    for (int d = FD_INT; d < FD_UNITS; d++) {
        int h = drv[d].hslot; if (h < 0) continue;
        if (drv[d].shelf < 0) { memset(HDD_DIRTY(h), 0, HDD_BLOCKS / 8); continue; }
        for (uint32_t b = 0; b < HDD_BLOCKS; b++) {
            if (!(HDD_DIRTY(h)[b >> 5] >> (b & 31) & 1)) continue;
            int s = wb_find(drv[d].shelf, b * 4096); if (s >= 0 && !wb_flush(s)) return 0;
            HDD_DIRTY(h)[b >> 5] &= ~(1u << (b & 31));
            out[0] = (uint8_t)drv[d].shelf; out[1] = out[2] = out[3] = 0; memcpy(out + 4, &b, 4);
            if (!shelf_read(drv[d].shelf, b * 4096, out + 8, 4096)) return 0;
            return 1;
        }
    }
    return 0;
}
// How many internal drives are wired in: the disk ROM's own boot test, done once at MSX start with the
// Z80 stopped and the disk ROM cell in page 1 (msx.c). Per hardware drive: select it (motor off, as
// the ROM does), step in once, restore; a drive that exists leaves track 0 and comes back (TRACK0 in
// the WD2793 status). ~15 ms per present drive; an absent one hits the timeouts (~180 ms) and the
// command is aborted with force-interrupt. The ROM repeats the same dance a moment later.
static bool wait_idle(uint32_t us) { absolute_time_t end = make_timeout_time_us(us); while (bus_mem_read(prof->base) & 0x01) if (time_reached(end)) return false; return true; }
static void fdc_abort(void) { bus_mem_write(prof->base, 0xD0); bus_drain(); busy_wait_us(100); (void)bus_mem_read(prof->base); }
void fdc_probe_physical(void) {
    int n = 0;
    for (int h = 0; h < 2 && h < prof->max_drives; h++) {
        bus_mem_write(prof->sel_reg, (uint8_t)((prof->idle_sel & ~prof->sel_mask) | h)); bus_drain(); busy_wait_us(100);
        fdc_abort();
        bool present = false;
        bus_mem_write(prof->base, 0x50); bus_drain();                                   // step in
        if (wait_idle(60000)) {
            bool moved = !(bus_mem_read(prof->base) & 0x04);                            // head left track 0
            bus_mem_write(prof->base, 0x00); bus_drain();                               // restore
            bool home = wait_idle(120000) && (bus_mem_read(prof->base) & 0x04);
            present = moved && home;
        }
        if (bus_mem_read(prof->base) & 0x01) fdc_abort();
        if (present) n++;
    }
    bus_mem_write(prof->sel_reg, prof->idle_sel); bus_drain();
    phys_count = n;
    for (int d = 0; d < 2; d++) if (drv[d].mode == FD_PHYSICAL && drv[d].hw >= (n > 0 ? n : 1)) drv[d].hw = 0;   // a second drive that is not there any more
    if (drv[0].mode == FD_PHYSICAL && drv[1].mode == FD_PHYSICAL && drv[0].hw == drv[1].hw) drv[1].mode = FD_NONE;
}
int fdc_physical_count(void) { return phys_count; }
bool fdc_internal_hidden(void) { return drv[0].mode == FD_NONE && drv[1].mode == FD_NONE; }
// Console loop, about once a second: once the disk ROMs have registered (the internal one comes last
// in slot order) and no external disk interface is among them, the disk port's drives do not exist for
// the MSX: drop their images so nothing stays mounted on a drive nobody can reach.
void fdc_poll(void) {
    static uint32_t last; uint32_t now = to_ms_since_boot(get_absolute_time());
    if (!fdc_ready) return;
    bool pending = false; for (int s = 0; s < WB_SLOTS; s++) if (wb[s].shelf >= 0 && wb[s].dirty) pending = true;
    if (pending && now - wb_last_ms >= HDD_FLUSH_MS) wb_flush_all();
    if (now - last < 1000) return; last = now;
    if (!msx_running() || internal_count() == 0 || ext_count() > 0) return;
    bool ch = false;
    for (int d = FD_INT; d < FD_UNITS; d++) if (is_row(d) && drv[d].shelf >= 0) { if (drv[d].kind == VK_HDD) wb_drop(drv[d].shelf); drv[d].shelf = -1; load_image(d); ch = true; }
    if (ch) save_cfg();
}
void fdc_print_json(void) {
    static const char *mn[] = { "physical", "virtual", "none" };
    printf("{\"fdc\":\"%s\",\"iface\":\"%s\",\"phys_drives\":%d,\"max_drives\":%u,\"drives\":[", prof->chip, prof->iface, phys_count, prof->max_drives);
    for (int d = 0; d < 2; d++) {
        printf("%s{\"drive\":\"%c\",\"mode\":\"%s\",\"hw\":%u,\"shelf\":%d,\"wp\":%s}", d ? "," : "", 'A' + d, mn[drv[d].mode], drv[d].hw, drv[d].shelf, drv[d].wp ? "true" : "false");
    }
    for (int d = FD_INT; d < FD_UNITS; d++) if (is_row(d)) printf(",{\"drive\":\"%s\",\"kind\":\"%s\",\"shelf\":%d,\"wp\":%s,\"size\":%lu}", fdc_unit_name(d), drv[d].kind == VK_HDD ? "hdd" : "fdd", drv[d].shelf, drv[d].wp ? "true" : "false", (unsigned long)drv[d].size);
    // DRVTBL (FB21h): the BIOS's own list of disk interfaces in slot order, [drives, slot id] x 4 - what the MSX calls A:, B:, ...
    printf("],\"vd_max\":%d,\"fdd_free\":%d,\"hdd_free\":%d,\"restart\":%d,\"diskrom\":%u,\"ext_master\":%s,\"int_count\":%d,\"drvtbl\":[", VD_MAX, FDD_POOL - units_in_use(), HDD_MAX - hdd_rows(), fdc_restart_needed() ? 1 : 0, msx_disk_rom_slotid(), ext_master() ? "true" : "false", internal_count());
    for (int i = 0; i < 4; i++) printf("%s[%u,%u]", i ? "," : "", mem_rd((uint16_t)(0xFB21 + 2 * i)), mem_rd((uint16_t)(0xFB22 + 2 * i)));
    printf("],\"sel\":%u,\"motor\":%d,\"track\":%u,\"sector\":%u,\"side\":%u}\n", sel, motor, r_track, r_sector, side);
}
