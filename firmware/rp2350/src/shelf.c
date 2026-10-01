#include "shelf.h"
#include "settings.h"
#include "flashxip.h"
#include "flashfar.h"
#include "link.h"
#include "led.h"
#include "fdc.h"
#include "cart.h"
#include <string.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/flash.h"

// Cache reads go through the untranslated, uncached XIP alias: a trial image in slot B runs with ATRANS0
// remapped (or disabled after a flash write), and 0x10200000+ then faults; 0x1C000000+ always means the
// physical flash address.
#define XIP_RAW 0x1C000000u
#define CAT_MAGIC 0x32464853u   // "SHF2"
#define CAT_BYTES 16384u
#define BLK 0x10000u
typedef struct { uint32_t magic, version, count, rsv; shelf_ent_t e[SHELF_MAX]; uint32_t crc; } catalog_t;
static union { catalog_t c; uint8_t raw[(sizeof(catalog_t) + 255u) & ~255u]; } catu;   // raw: what goes to flash (256-byte pages)
#define cat catu.c
static uint8_t sec[FLASH_SECTOR_SIZE];   // shared 4 KiB work buffer (receive / in-place write)
static uint8_t bounce[8192];             // far flash <-> PSRAM / link staging (direct-mode reads cannot land in PSRAM)
static inline bool is_far(const shelf_ent_t *e) { return e->off >= FAR_BASE; }
static const char *type_names[16] = { "auto", "plain", "ascii8", "ascii16", "konami", "konami-scc", "?", "?", "?", "disk", "hdd" };
const char *img_type_name(uint8_t t) { return t < 16 && type_names[t] ? type_names[t] : "?"; }

static bool cat_save(void) {
    cat.magic = CAT_MAGIC; cat.version = 3;
    cat.crc = crc32_buf(&cat, offsetof(catalog_t, crc));
    led_note_disk();
    flash_erase(SHELF_BASE, CAT_BYTES);                      // the bodies start at SHELF_BASE + 64 KiB: room to spare
    return flash_program(SHELF_BASE, catu.raw, sizeof catu.raw);
}
void shelf_init(void) {
    _Static_assert(sizeof(catalog_t) <= CAT_BYTES, "catalog must fit four sectors");
    memcpy(&cat, (const void *)(XIP_RAW + SHELF_BASE), sizeof cat);
    if (cat.magic != CAT_MAGIC || cat.count > SHELF_MAX || cat.crc != crc32_buf(&cat, offsetof(catalog_t, crc))) {
        memset(&cat, 0, sizeof cat);
        // No catalog (first boot, or the format changed): the settings' shelf indices would point at
        // whatever gets cached next, so forget every mounted image and cartridge.
        bool ch = false;
        for (int i = 0; i < 4; i++) if (settings.cart_cell[i]) { settings.cart_cell[i] = 0; ch = true; }
        for (int i = 0; i < 2; i++) { if (settings.cart_sel[i]) { settings.cart_sel[i] = 0; ch = true; } if (settings.fd_port[i]) { settings.fd_port[i] = 0; ch = true; } if (settings.fd_cfg[i] & 0xFFFF00u) { settings.fd_cfg[i] &= 0xFFu; ch = true; } }
        for (int i = 0; i < 6; i++) if (settings.vd[i] & 0x0FFFFFFFu) { settings.vd[i] &= 0xF0000000u; ch = true; }   // the rows stay, their images go
        if (ch) settings_save();
    }
}
int shelf_count(void) { return (int)cat.count; }
const shelf_ent_t *shelf_get(int i) { return (i >= 0 && i < (int)cat.count) ? &cat.e[i] : NULL; }
int shelf_find(const char *path) { for (uint32_t i = 0; i < cat.count; i++) if (!strcmp(cat.e[i].path, path)) return (int)i; return -1; }
const uint8_t *shelf_data(int i) { const shelf_ent_t *e = shelf_get(i); return e && !is_far(e) ? (const uint8_t *)(XIP_RAW + e->off) : NULL; }
bool shelf_is_far(int i) { const shelf_ent_t *e = shelf_get(i); return e && is_far(e); }
bool shelf_read(int i, uint32_t off, uint8_t *dst, uint32_t n) {
    const shelf_ent_t *e = shelf_get(i);
    if (!e || off + n > e->size) return false;
    if (!is_far(e)) { memcpy(dst, (const void *)(XIP_RAW + e->off + off), n); return true; }
    while (n) {
        uint32_t k = n < sizeof bounce ? n : sizeof bounce;
        if (!far_read(e->off + off, bounce, k)) return false;
        memcpy(dst, bounce, k); dst += k; off += k; n -= k;
    }
    return true;
}

// First gap that holds `size` bytes, 64 KiB granularity: the XIP region first (the catalog owns its
// first block), then the far flash.
static bool find_gap_in(uint32_t lo, uint32_t hi, uint32_t size, uint32_t *off_out, int skip) {
    uint32_t need = (size + BLK - 1) & ~(BLK - 1), cur = lo;
    for (;;) {
        uint32_t next = hi; const shelf_ent_t *blocker = NULL;
        for (uint32_t i = 0; i < cat.count; i++) {
            if ((int)i == skip) continue;
            const shelf_ent_t *e = &cat.e[i]; uint32_t start = e->off & ~(BLK - 1), end = (e->off + e->size + BLK - 1) & ~(BLK - 1);
            if (end <= lo || start >= hi) continue;
            if (end > cur && start < next) { next = start; blocker = e; }
        }
        if (cur + need <= next) { *off_out = cur; return true; }
        if (!blocker) return false;
        cur = (blocker->off + blocker->size + BLK - 1) & ~(BLK - 1);
    }
}
static bool find_gap(uint32_t size, uint32_t *off_out, int skip, bool far_only) {   // hard disk images: the far flash only
    return (!far_only && find_gap_in(SHELF_BASE + BLK, SHELF_END, size, off_out, skip)) || find_gap_in(FAR_BASE, FAR_END, size, off_out, skip);
}
uint32_t shelf_total_bytes(void) { return (SHELF_END - SHELF_BASE - BLK) + (FAR_END - FAR_BASE); }
uint32_t shelf_free_bytes(void) {
    uint32_t used = 0;
    for (uint32_t i = 0; i < cat.count; i++) used += (cat.e[i].size + BLK - 1) & ~(BLK - 1);
    return shelf_total_bytes() - used;
}

// ---- LRU: every mount stamps the entry with a sequence number (16 bits in rsv[]; the catalog header's
// rsv word is the counter). When a put finds no room, the least recently used entry that nothing refers
// to (no drive, no cartridge, no pending cartridge, no unsynced writes) is deleted and the search retried.
static inline uint16_t used_of(const shelf_ent_t *e) { return (uint16_t)(e->rsv[0] | (e->rsv[1] << 8)); }
static void set_used(shelf_ent_t *e, uint16_t u) { e->rsv[0] = (uint8_t)u; e->rsv[1] = (uint8_t)(u >> 8); }
static uint16_t next_seq(void) {
    if (cat.rsv >= 0xFFF0) {                                   // renumber by rank so the counter never wraps
        for (uint32_t r = 1; r <= cat.count; r++) { int best = -1; for (uint32_t i = 0; i < cat.count; i++) if (used_of(&cat.e[i]) >= 0x8000 || used_of(&cat.e[i]) == 0) continue; else if (best < 0 || used_of(&cat.e[i]) < used_of(&cat.e[best])) best = (int)i; if (best < 0) break; set_used(&cat.e[best], (uint16_t)(0x8000 | r)); }
        for (uint32_t i = 0; i < cat.count; i++) set_used(&cat.e[i], (uint16_t)(used_of(&cat.e[i]) & 0x7FFF));
        cat.rsv = cat.count;
    }
    return (uint16_t)++cat.rsv;
}
bool shelf_touch(int i) { if (i < 0 || i >= (int)cat.count) return false; set_used(&cat.e[i], next_seq()); return cat_save(); }
static bool evictable(int i, int keep) {
    if (i == keep || (cat.e[i].flags & SHELF_DIRTY)) return false;
    if (fdc_uses_shelf(i) || cart_any_uses_shelf(i, NULL, NULL) || cart_pending_uses_shelf(i)) return false;
    return true;
}
static bool evict_one(int *keep, bool far_only) {   // far_only: make room in the far flash without touching the XIP region's entries
    int victim = -1;
    for (int i = 0; i < (int)cat.count; i++) if (evictable(i, *keep) && (!far_only || is_far(&cat.e[i])) && (victim < 0 || used_of(&cat.e[i]) < used_of(&cat.e[victim]))) victim = i;
    if (victim < 0) return false;
    shelf_delete(victim);
    if (*keep > victim) (*keep)--;
    return true;
}
static bool make_room(uint32_t size, uint32_t *off, int *replace, bool far_only) {
    while (*replace < 0 && cat.count >= SHELF_MAX) if (!evict_one(replace, false)) return false;
    while (!find_gap(size, off, *replace, far_only)) if (!evict_one(replace, far_only)) return false;
    return true;
}
void shelf_receive(const char *path, uint32_t size, uint8_t type, uint32_t mtime) {
    uint32_t off; bool hdd = type == IMG_HDD;
    if (!path || !path[0] || strlen(path) >= SHELF_PATH || size == 0 || size > (hdd ? FAR_END - FAR_BASE : 8u * 1024 * 1024)) { printf("err bad path or size\n"); return; }
    int replace = shelf_find(path);
    if (replace >= 0 && hdd != is_far(&cat.e[replace])) { shelf_delete(replace); replace = -1; }   // a hard disk image never sits in the XIP region
    if (!make_room(size, &off, &replace, hdd)) { printf("err no room (%lu bytes free%s, nothing evictable)\n", (unsigned long)shelf_free_bytes(), hdd ? " in the far flash" : ""); return; }
    printf("ready\n"); fflush(stdout);
    bool far = off >= FAR_BASE; uint32_t got = 0, wr = off, crc = 0xFFFFFFFFu;
    while (got < size) {
        uint32_t n = size - got < 4096 ? size - got : 4096;
        if (!link_rx_read(sec, n, 5000)) { printf("err timeout at %lu\n", (unsigned long)got); return; }
        if (n < FLASH_SECTOR_SIZE) memset(sec + n, 0xFF, FLASH_SECTOR_SIZE - n);
        got += n;
        printf("ok\n"); fflush(stdout);          // the next piece streams in (DMA ring) while this one is programmed
        led_note_disk();
        for (uint32_t i = 0; i < n; i++) { crc ^= sec[i]; for (int k = 0; k < 8; k++) crc = (crc >> 1) ^ (0xEDB88320u & -(crc & 1)); }
        if (far) {
            if ((wr & (BLK - 1)) == 0 && !far_erase(wr, BLK)) { printf("err erase at %08lx\n", (unsigned long)wr); return; }
            if (!far_program(wr, sec, FLASH_SECTOR_SIZE)) { printf("err program at %08lx\n", (unsigned long)wr); return; }
        } else {
            if ((wr & (BLK - 1)) == 0) flash_erase(wr, BLK);   // one 64 KiB block erase per 16 pieces (~150 ms)
            if (!flash_program(wr, sec, FLASH_SECTOR_SIZE)) { printf("err verify at %08lx\n", (unsigned long)wr); return; }
        }
        wr += FLASH_SECTOR_SIZE;
    }
    shelf_ent_t *e = replace >= 0 ? &cat.e[replace] : &cat.e[cat.count];
    memset(e, 0, sizeof *e);
    strncpy(e->path, path, SHELF_PATH - 1);
    e->off = off; e->size = size; e->type = type; e->mtime = mtime; e->crc = ~crc; set_used(e, next_seq());
    if (replace < 0) cat.count++;
    if (!cat_save()) { if (replace < 0) cat.count--; printf("err catalog\n"); return; }
    printf("done %lu bytes as #%d\n", (unsigned long)size, replace >= 0 ? replace : (int)(cat.count - 1));
}
bool shelf_send(int i, uint32_t off, uint32_t len) {
    const shelf_ent_t *e = shelf_get(i);
    if (!e || off > e->size) return false;
    if (off + len > e->size || len == 0) len = e->size - off;     // len 0: the rest (the ESP asks in pieces: no flow control on the link)
    // The ESP asks for <= 8 KB pieces; every piece is staged in SRAM. The far region needs that anyway,
    // and the link's TX DMA cannot read the untranslated XIP alias (0x1C000000): handing it a flash
    // pointer left the DMA stalled for good and the RP2350 looked dead on the link (0.4.0).
    static link_piece_t pc[1];
    if (len > sizeof bounce) len = sizeof bounce;
    if (len && !shelf_read(i, off, bounce, len)) return false;
    pc[0] = (link_piece_t){ bounce, len };
    printf("bin %lu\n", (unsigned long)len);
    return len == 0 || link_send_pieces(pc, 1);
}
bool shelf_write(int i, uint32_t off, const uint8_t *data, uint32_t n) {
    const shelf_ent_t *e = shelf_get(i);
    if (!e || off + n > e->size) return false;
    led_note_disk();
    while (n) {
        uint32_t fo = e->off + off, base = fo & ~(uint32_t)(FLASH_SECTOR_SIZE - 1), in = fo - base;
        uint32_t k = FLASH_SECTOR_SIZE - in; if (k > n) k = n;
        if (is_far(e)) {
            if (!far_read(base, sec, FLASH_SECTOR_SIZE)) return false;
            if (memcmp(sec + in, data, k)) { memcpy(sec + in, data, k); if (!far_erase_sector(base) || !far_program(base, sec, FLASH_SECTOR_SIZE)) return false; }
        } else {
            memcpy(sec, (const void *)(XIP_RAW + base), FLASH_SECTOR_SIZE);
            if (memcmp(sec + in, data, k)) { memcpy(sec + in, data, k); if (!flash_program_sector(base, sec)) return false; }
        }
        off += k; data += k; n -= k;
    }
    return true;
}
bool shelf_delete(int i) {
    if (i < 0 || i >= (int)cat.count) return false;
    memmove(&cat.e[i], &cat.e[i + 1], (cat.count - 1 - i) * sizeof(shelf_ent_t));
    cat.count--; memset(&cat.e[cat.count], 0, sizeof(shelf_ent_t));
    fdc_shelf_deleted(i); cart_shelf_deleted(i);   // entries above i moved down one: fix the drive/slot references
    return cat_save();
}
bool shelf_set_mtime(int i, uint32_t mtime) {
    if (i < 0 || i >= (int)cat.count) return false;
    if (cat.e[i].mtime == mtime) return true;
    cat.e[i].mtime = mtime; return cat_save();
}
bool shelf_set_type(int i, uint8_t type) {
    if (i < 0 || i >= (int)cat.count) return false;
    cat.e[i].type = type; return cat_save();
}
bool shelf_set_dirty(int i, bool dirty) {
    if (i < 0 || i >= (int)cat.count) return false;
    uint8_t f = dirty ? (uint8_t)(cat.e[i].flags | SHELF_DIRTY) : (uint8_t)(cat.e[i].flags & ~SHELF_DIRTY);
    if (f == cat.e[i].flags) return true;
    cat.e[i].flags = f; return cat_save();
}
static void jstr(const char *s) { for (; *s; s++) { if (*s == '"' || *s == '\\') putchar('\\'); if ((unsigned char)*s < 0x20) putchar(' '); else putchar(*s); } }
void shelf_print_json(void) {
    printf("{\"free\":%lu,\"total\":%lu,\"entries\":[", (unsigned long)shelf_free_bytes(), (unsigned long)shelf_total_bytes());
    for (uint32_t i = 0; i < cat.count; i++) {
        const shelf_ent_t *e = &cat.e[i];
        printf("%s{\"i\":%lu,\"path\":\"", i ? ",\n" : "\n", (unsigned long)i); jstr(e->path);   // one entry per line: the ESP32 reads lines of at most 2 KB
        printf("\",\"size\":%lu,\"type\":%u,\"tname\":\"%s\",\"crc\":\"%08lx\",\"mtime\":%lu,\"dirty\":%d,\"far\":%d,\"used\":%u,\"pages\":%u}", (unsigned long)e->size, e->type, img_type_name(e->type), (unsigned long)e->crc, (unsigned long)e->mtime, (e->flags & SHELF_DIRTY) ? 1 : 0, is_far(e) ? 1 : 0, used_of(e), cart_pages_of((int)i));
    }
    printf("]}\n");
}
