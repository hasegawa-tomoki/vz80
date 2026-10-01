#include "vzdisk.h"
#include "fdc.h"
#include <string.h>
#include <stdio.h>
// Register map (after OUT (40h),VZDISK_ID; IN A,(40h) then returns ~VZDISK_ID):
//   41h W  command: 0 = read sector into the buffer, 1 = write the buffer to the sector
//   41h R  status: 0 = ok, 1 write protected, 2 not ready, 8 record not found, 10 write fault, 12 bad command
//   42h/43h/44h  W sector number, low/mid/high
//   45h W  unit (0-based row: V1 = 0, V2 = 1, ...)
//   46h RW the 512-byte buffer, auto-incrementing; the index wraps
//   47h W  reset the buffer index; R = number of units (rows)
//   48h R  media-changed bits (bit n = unit n), cleared by the read
//   49h/4Ah/4Bh R  size of the unit in sectors, low/mid/high (0 when no image)
//   4Ch R  kind of the unit: 1 floppy, 2 hard disk, 0 no such unit
uint32_t vzdisk_bpbfix, vzdisk_ptfix;
static bool selected; static uint8_t stat, unit; static uint32_t sector; static uint16_t idx; static uint8_t buf[512];
uint32_t vzdisk_reads, vzdisk_writes, vzdisk_errors, vzdisk_selects, vzdisk_regs;   // for jstatus
// event log: every register access except the data port, which is counted per command instead
#define LOGN 512
static uint16_t logv[LOGN]; static uint32_t logn; static uint16_t data_cnt;
static void logev(uint8_t port, uint8_t v, bool rd) {
    uint16_t e = (uint16_t)((rd ? 0x8000 : 0) | ((port - 0x40) << 8) | v);
    if (logn && logv[(logn - 1) % LOGN] == e && (port == 0x40 || rd)) return;   // a select or status polled in a loop: one entry
    logv[logn % LOGN] = e; logn++;
}
static void logdata(void) { if (data_cnt) { logv[logn % LOGN] = (uint16_t)(0x7000 | (data_cnt & 0xFFF)); logn++; data_cnt = 0; } }
void vzdisk_log_print(void) {
    uint32_t n = logn < LOGN ? logn : LOGN;
    for (uint32_t i = logn - n; i < logn; i++) { uint16_t e = logv[i % LOGN]; if ((e & 0xF000) == 0x7000) printf(" data%u", e & 0xFFF); else printf(" %c4%X=%02X", (e & 0x8000) ? 'i' : 'o', (e >> 8) & 0xF, e & 0xFF); }
    printf("\n");
}

// Nextor maps a device only if its boot sector carries a usable FAT12 BPB; the internal disk ROM does
// not care. Home-made formats (Ys II: 0 reserved sectors, 3 FATs, a private directory read by the boot
// code through BDOS absolute sector reads) would never boot through Nextor. When the BPB is plainly
// invalid, the copy handed to the driver gets the standard 2DD / 1DD values instead; the boot code and
// everything else stay untouched, so the game's own loader still sees the real sectors.
static void fix_bpb(uint8_t *b, uint32_t size) {
    uint16_t bps = b[11] | (b[12] << 8), res = b[14] | (b[15] << 8), root = b[17] | (b[18] << 8), tot = b[19] | (b[20] << 8), spf = b[22] | (b[23] << 8);
    uint8_t spc = b[13], nfat = b[16];
    // Same test as Nextor's CHECK_FAT_BOOT (DOS 1 mode): patch only what Nextor itself would reject, because
    // the DPB (and so the FAT buffer size, i.e. free RAM) is built from these values. (res is not checked there.)
    bool ok = bps == 512 && spc && !(spc & (spc - 1)) && nfat >= 1 && nfat <= 7 && root && tot && spf >= 1 && spf <= 3;
    (void)res;
    if (ok) return;
    bool dd = size > 400000;
    static const uint8_t bpb2dd[19] = { 0x00, 0x02, 2, 1, 0, 2, 112, 0, 0xA0, 0x05, 0xF9, 3, 0, 9, 0, 2, 0, 0, 0 };   // 720 KB
    static const uint8_t bpb1dd[19] = { 0x00, 0x02, 2, 1, 0, 2, 112, 0, 0xD0, 0x02, 0xF8, 2, 0, 9, 0, 1, 0, 0, 0 };   // 360 KB
    memcpy(b + 11, dd ? bpb2dd : bpb1dd, 19);
    vzdisk_bpbfix++;
}
// Nextor 2.1.2 (DOS 1 mode) reads the four partition-type bytes of sector 0 (offsets 1C2h, 1D2h, 1E2h,
// 1F2h) even on a device it was told is a floppy, without looking for the 55AA signature. Home-made
// boot sectors that keep loader data there (Ys II) then count as a partitioned device and never boot.
// A floppy image whose sector 0 carries no 55AA signature cannot be an MBR: hand Nextor a copy with
// those four bytes zeroed. Everything else, including the loader, stays as it is.
static void fix_ptable(uint8_t *b) {
    if (b[0x1FE] == 0x55 && b[0x1FF] == 0xAA) return;
    bool any = b[0x1C2] || b[0x1D2] || b[0x1E2] || b[0x1F2];
    if (!any) return;
    b[0x1C2] = b[0x1D2] = b[0x1E2] = b[0x1F2] = 0;
    vzdisk_ptfix++;
}
static void do_cmd(uint8_t c) {
    idx = 0;
    if (c == 0) { stat = fdc_vd_read(unit, sector, buf); if (!stat && sector == 0 && fdc_vd_kind(unit) == VK_FDD) { fix_bpb(buf, fdc_vd_sectors(unit) * 512); fix_ptable(buf); } vzdisk_reads++; }
    else if (c == 1) { stat = fdc_vd_write(unit, sector, buf); vzdisk_writes++; }
    else stat = 12;
    if (stat) vzdisk_errors++;
}
bool vzdisk_io_wr(uint8_t port, uint8_t v) {
    if (port == 0x40) { selected = v == VZDISK_ID; if (selected) { vzdisk_selects++; logdata(); logev(0x40, v, false); } return false; }    // the write also goes to the bus: other devices deselect
    if (!selected || port < 0x41 || port > 0x48) return false;
    vzdisk_regs++;
    if (port == 0x46) data_cnt++; else { logdata(); logev(port, v, false); }
    switch (port) {
        case 0x41: do_cmd(v); break;
        case 0x42: sector = (sector & 0xFFFF00u) | v; break;
        case 0x43: sector = (sector & 0xFF00FFu) | ((uint32_t)v << 8); break;
        case 0x44: sector = (sector & 0x00FFFFu) | ((uint32_t)v << 16); break;
        case 0x45: unit = v; break;
        case 0x46: buf[idx] = v; idx = (idx + 1) & 511; break;
        case 0x47: idx = 0; break;
        default: break;
    }
    return true;
}
bool vzdisk_io_rd(uint8_t port, uint8_t *v) {
    if (port == 0x40) { if (!selected) return false; *v = (uint8_t)~VZDISK_ID; return true; }
    if (!selected || port < 0x41 || port > 0x4C) return false;
    switch (port) {
        case 0x41: *v = stat; break;
        case 0x49: case 0x4A: case 0x4B: *v = (uint8_t)(fdc_vd_sectors(unit) >> (8 * (port - 0x49))); break;
        case 0x4C: *v = (uint8_t)fdc_vd_kind(unit); break;
        case 0x46: *v = buf[idx]; idx = (idx + 1) & 511; break;
        case 0x47: *v = (uint8_t)fdc_vd_count(); break;
        case 0x48: *v = fdc_take_changed(); break;
        default: *v = 0xFF; break;
    }
    if (port == 0x46) data_cnt++; else { logdata(); logev(port, *v, true); }
    return true;
}
