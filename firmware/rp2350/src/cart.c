#include "cart.h"
#include "shelf.h"
#include "settings.h"
#include "psram.h"
#include "msx.h"
#include <string.h>
#include <stdio.h>

typedef struct { bool set; int8_t p, s; int shelf; uint8_t type, basepage, pages, nbanks8; uint32_t size; uint8_t *base; uint8_t bank[4];
                 bool learn; uint8_t hits[9]; uint8_t nhits; uint8_t maxval; } cart_t;   // learn: watch the first mapper writes and fix a wrong guess (cart_observe)
static cart_t carts[CART_MAX];
static bool restart_needed;   // the pending configuration (settings.cart_cell) differs from what runs
bool cart_restart_needed(void) { return restart_needed; }
#define CART_LO PSRAM_CART_LO                             // above the mapper RAM expansion
#define CART_HI PSRAM_CART_HI                             // below the virtual floppy images and the VRAM shadow
static cart_t *find(int p, int s) { for (int i = 0; i < CART_MAX; i++) if (carts[i].set && carts[i].p == p && carts[i].s == s) return &carts[i]; return NULL; }
int cart_index(int p, int s) { cart_t *c = find(p, s); return c ? (int)(c - carts) : -1; }
bool cart_parse_cell(const char *txt, int *p, int *s) {
    if (!txt || txt[0] < '0' || txt[0] > '3') return false;
    *p = txt[0] - '0'; *s = 0;
    if (txt[1] == 0) return true;
    if (txt[1] != '-' || txt[2] < '0' || txt[2] > '3' || txt[3] != 0) return false;
    *s = txt[2] - '0'; return true;
}

int cart_guess_type(const uint8_t *rom, uint32_t size) {
    // Static guess: tally the targets of `LD (nnnn),A` (opcode 32h) that hit a mapper register window.
    // It works for most cartridges but not for those that compute the register address (Aleste writes
    // via HL): those are corrected at run time by cart_observe() from the writes actually made.
    if (size <= 0x8000) return IMG_PLAIN;
    int a8 = 0, a16 = 0, kon = 0, scc = 0;
    for (uint32_t i = 0; i + 2 < size; i++) {
        if (rom[i] != 0x32) continue;
        uint16_t ad = rom[i + 1] | (rom[i + 2] << 8);
        switch (ad) {
            case 0x4000: case 0x8000: case 0xA000: kon++; break;
            case 0x5000: case 0x9000: case 0xB000: scc++; break;
            case 0x6800: case 0x7800: a8++; break;
            case 0x6000: kon++; a8++; a16++; break;
            case 0x7000: scc++; a8++; a16++; break;
            case 0x77FF: a16++; break;
            default: break;
        }
    }
    if (a8) a8--;                                          // ASCII 8K shares most addresses: needs a clear lead
    int best = IMG_ASCII8, bv = a8;
    if (a16 >= bv) { best = IMG_ASCII16; bv = a16; }
    if (kon >= bv && kon) { best = IMG_KONAMI; bv = kon; }
    if (scc >= bv && scc) { best = IMG_KONAMI_SCC; bv = scc; }
    return best;
}

// ---- run-time check of the mapper type -------------------------------------------------------
// The bank-register windows of the four MegaROM mappers are almost disjoint:
//   ASCII 8K   6000 6800 7000 7800         ASCII 16K  6000 7000
//   Konami     6000 8000 A000 (4000-5FFF fixed)   Konami SCC  5000 7000 9000 B000
// While `learn` is set (type came from the static guess), every write into the cartridge pages is
// tallied per window; after a few writes the pattern names the mapper. A contradiction with the
// current type is corrected once: the type is changed, remembered in the cache catalog, and the MSX
// restarted (cart_learn_poll, from the console loop).
static void reset_banks(cart_t *c);
static int learn_slot_pending;
static int window_of(uint16_t a) {                     // 0:5000 1:6000 2:6800 3:7000 4:7800 5:8000 6:9000 7:A000 8:B000, -1 other
    if (a >= 0x5000 && a < 0x5800) return 0;
    if (a >= 0x6000 && a < 0x8000) return 1 + ((a - 0x6000) >> 11);
    if (a >= 0x8000 && a < 0x8800) return 5;
    if (a >= 0x9000 && a < 0x9800) return 6;
    if (a >= 0xA000 && a < 0xA800) return 7;
    if (a >= 0xB000 && a < 0xB800) return 8;
    return -1;
}
static int learn_decide(const cart_t *c) {
    const uint8_t *h = c->hits;
    // Wait for a handful of writes and take a clear majority of the exclusive windows: Dragon Quest II
    // (ASCII 8K) writes 8000h / A000h (no-ops for its mapper) before its first 6800h / 7800h, and a
    // first-write verdict turned it into a Konami cartridge for good (stored in the catalog).
    if (c->nhits < 6) return IMG_AUTO;
    int scc = h[0] + h[6] + h[8], kon = h[5] + h[7], a8only = h[2] + h[4];
    int best = a8only, t = IMG_ASCII8;
    if (kon > best) { best = kon; t = IMG_KONAMI; }
    if (scc > best) { best = scc; t = IMG_KONAMI_SCC; }
    if (best >= 2 && best > scc + kon + a8only - best) return t;
    if (!scc && !kon && !a8only && (h[1] || h[3])) {   // only 6000/7000 seen: 16K unless a value exceeds the 16K bank count
        unsigned n16 = (c->nbanks8 + 1) / 2;
        return c->maxval >= n16 ? IMG_ASCII8 : IMG_ASCII16;
    }
    return IMG_AUTO;                                       // mixed or nothing yet: keep watching
}
void cart_observe(int p, int s, uint16_t a, uint8_t v) {
    cart_t *c = find(p, s); if (!c) return;
    if (!c->learn) return;
    int w = window_of(a);
    if (w < 0) return;
    if (c->hits[w] < 255) c->hits[w]++;
    if (v > c->maxval) c->maxval = v;
    if (c->nhits < 255) c->nhits++;
    int t = learn_decide(c);
    if (t == IMG_AUTO) return;
    if (t != c->type) { learn_slot_pending = (int)(c - carts) + 1; c->learn = false; }        // contradiction: fix it from the console loop
    else if (c->nhits >= 24) { c->learn = false; learn_slot_pending = (int)(c - carts) + 1; }   // confirmed: remember it in the catalog (console loop)
}
void cart_learn(int p, int s) { cart_t *c = find(p, s); if (c) { shelf_set_type(c->shelf, IMG_AUTO); restart_needed = true; } }   // forget the stored type; the next start guesses again
void cart_learn_poll(void) {
    if (!learn_slot_pending) return;
    cart_t *c = &carts[learn_slot_pending - 1]; learn_slot_pending = 0; if (!c->set) return;
    int t = learn_decide(c); if (t == IMG_AUTO) return;
    if (t == c->type) { shelf_set_type(c->shelf, (uint8_t)t); return; }        // confirmed guess: stored so the next mount does not guess
    printf("cart %d-%d: mapper looks like %s (writes 5000:%u 6000:%u 6800:%u 7000:%u 7800:%u 8000:%u 9000:%u A000:%u B000:%u), was %s: restarting\n",
           c->p, c->s, img_type_name((uint8_t)t), c->hits[0], c->hits[1], c->hits[2], c->hits[3], c->hits[4], c->hits[5], c->hits[6], c->hits[7], c->hits[8], img_type_name(c->type));
    shelf_set_type(c->shelf, (uint8_t)t);              // remembered: the pending config says IMG_AUTO, so the next start uses it
    restart_needed = true;                              // the user restarts when convenient (the game is running with the wrong mapper)
}

static void reset_banks(cart_t *c) {
    // Konami mappers power up with the first 32 KB laid out contiguously. While the type is still a
    // guess the ASCII mappers start the same way: whatever the real mapper is, the ROM then boots as
    // it expects and its first register writes reveal the type (real ASCII cartridges start at 0s,
    // but their code always sets the banks before relying on them).
    if (c->type == IMG_KONAMI || c->type == IMG_KONAMI_SCC || (c->learn && c->type == IMG_ASCII8)) { c->bank[0] = 0; c->bank[1] = 1; c->bank[2] = 2; c->bank[3] = 3; }
    else if (c->learn && c->type == IMG_ASCII16) { c->bank[0] = 0; c->bank[1] = 1; c->bank[2] = c->bank[3] = 0; }
    else memset(c->bank, 0, sizeof c->bank);
}
static bool alloc_psram(uint32_t size, uint8_t **base) {   // first fit between the carts already placed (64 KB aligned)
    uint32_t lo = CART_LO;
    for (;;) {
        bool moved = false;
        for (int i = 0; i < CART_MAX; i++) {
            const cart_t *o = &carts[i]; if (!o->set) continue;
            uint32_t ob = (uint32_t)o->base, oe = (ob + o->size + 0xFFFF) & ~0xFFFFu;
            if (lo < oe && lo + size > ob) { lo = oe; moved = true; }
        }
        if (!moved) break;
    }
    if (lo + size > CART_HI) return false;
    *base = (uint8_t *)lo; return true;
}
static void save_sel(void) {
    settings.cart_sel[0] = settings.cart_sel[1] = 0;
    for (int i = 0; i < CART_MAX; i++) {
        const cart_t *c = &carts[i];
        settings.cart_cell[i] = c->set ? (uint32_t)(c->shelf + 1) | ((uint32_t)c->type << 8) | ((uint32_t)c->p << 16) | ((uint32_t)c->s << 20) : 0;
        if (c->set && c->s == 0 && (c->p == 1 || c->p == 2)) settings.cart_sel[c->p - 1] = (uint32_t)(c->shelf + 1) | ((uint32_t)c->type << 8);   // older firmware reads these
    }
    settings_save();
}

static bool set_nosave(int p, int s, int shelf_idx, uint8_t type, char *err, size_t n) {
    const shelf_ent_t *e = shelf_get(shelf_idx);
    if (p < 0 || p > 3 || s < 0 || s > 3 || (!msx_slot_expanded(p) && s != 0)) { snprintf(err, n, "no such slot cell"); return false; }
    if (!e) { snprintf(err, n, "no such shelf entry"); return false; }
    if (e->type == IMG_DISK) { snprintf(err, n, "that is a disk image"); return false; }
    if (!psram_info().ok) { snprintf(err, n, "PSRAM not available"); return false; }
    cart_t *c = find(p, s);
    if (c) cart_eject(p, s);
    for (int i = 0; i < CART_MAX && !c; i++) if (!carts[i].set) c = &carts[i];
    if (!c) { snprintf(err, n, "no free cartridge slot (%d in use)", CART_MAX); return false; }
    memset(c, 0, sizeof *c);
    if (!alloc_psram(e->size, &c->base)) { snprintf(err, n, "no PSRAM room for %lu bytes", (unsigned long)e->size); return false; }
    if (!shelf_read(shelf_idx, 0, c->base, e->size)) { snprintf(err, n, "cache read failed"); return false; }   // XIP copy or far flash
    const uint8_t *src = c->base;
    bool guessed = type == IMG_AUTO && e->type == IMG_AUTO;
    if (type == IMG_AUTO) type = e->type != IMG_AUTO ? e->type : (uint8_t)cart_guess_type(src, e->size);
    c->learn = guessed && type != IMG_PLAIN; memset(c->hits, 0, sizeof c->hits); c->nhits = 0; c->maxval = 0;
    c->p = (int8_t)p; c->s = (int8_t)s; c->shelf = shelf_idx; c->type = type; c->size = e->size;
    c->nbanks8 = (uint8_t)((e->size + 0x1FFF) / 0x2000);
    if (type == IMG_PLAIN) {
        uint32_t init = (src[0] == 'A' && src[1] == 'B') ? (src[2] | (src[3] << 8)) : 0;
        c->basepage = e->size > 0x8000 ? 0 : (init >= 0x8000 && e->size <= 0x4000) ? 2 : 1;
        if (e->size <= 0x8000 && src[0x4000] == 'A' && src[0x4001] == 'B' && !(src[0] == 'A' && src[1] == 'B')) c->basepage = 0;   // header in the 2nd 16K: ROM starts at 0000
        int np = (int)((e->size + 0x3FFF) / 0x4000); c->pages = 0;
        for (int i = 0; i < np && c->basepage + i < 4; i++) c->pages |= 1 << (c->basepage + i);
    } else { c->basepage = 1; c->pages = 0x06; }
    if (!msx_cell_free(p, s, c->pages, false, err, n)) return false;
    reset_banks(c);
    c->set = true;
    msx_set_cart_pages(p, s, c->pages);
    return true;
}
// Pages a ROM will occupy, from its header (cheap: two short reads from the cache), for checks before it is loaded.
static uint8_t pages_for(const shelf_ent_t *e, int idx, uint8_t type) {
    if (type == IMG_AUTO) type = e->type;
    if (type != IMG_PLAIN && (type != IMG_AUTO || e->size > 0x8000)) return 0x06;   // MegaROM (or an unknown big ROM): 4000h-BFFFh
    uint8_t h[4] = { 0, 0, 0, 0 }, h2[2] = { 0, 0 }; shelf_read(idx, 0, h, 4); if (e->size > 0x4000) shelf_read(idx, 0x4000, h2, 2);
    uint32_t init = (h[0] == 'A' && h[1] == 'B') ? (h[2] | (h[3] << 8)) : 0;
    int basepage = e->size > 0x8000 ? 0 : (init >= 0x8000 && e->size <= 0x4000) ? 2 : 1;
    if (e->size <= 0x8000 && h2[0] == 'A' && h2[1] == 'B' && !(h[0] == 'A' && h[1] == 'B')) basepage = 0;
    int np = (int)((e->size + 0x3FFF) / 0x4000); uint8_t pages = 0;
    for (int i = 0; i < np && basepage + i < 4; i++) pages |= 1 << (basepage + i);
    return pages;
}
uint8_t cart_pages_of(int shelf_idx) { const shelf_ent_t *e = shelf_get(shelf_idx); return e && e->type != IMG_DISK && e->type != IMG_HDD ? pages_for(e, shelf_idx, e->type) : 0; }
static int pend_index(int p, int s) { for (int i = 0; i < CART_MAX; i++) { uint32_t v = settings.cart_cell[i]; if (v && (int)((v >> 16) & 3) == p && (int)((v >> 20) & 3) == s) return i; } return -1; }
static void pend_save(void) {
    settings.cart_sel[0] = settings.cart_sel[1] = 0;
    for (int i = 0; i < CART_MAX; i++) { uint32_t v = settings.cart_cell[i]; if (v && ((v >> 20) & 3) == 0 && (((v >> 16) & 3) == 1 || ((v >> 16) & 3) == 2)) settings.cart_sel[((v >> 16) & 3) - 1] = v & 0xFFFF; }
    settings_save();
}
// The configuration is written to the settings only; it is applied by the next MSX start (cart_restore),
// so a ROM that is running keeps running until the user restarts.
bool cart_set(int p, int s, int shelf_idx, uint8_t type, char *err, size_t n) {
    const shelf_ent_t *e = shelf_get(shelf_idx);
    if (p < 0 || p > 3 || s < 0 || s > 3 || (!msx_slot_expanded(p) && s != 0)) { snprintf(err, n, "no such slot cell"); return false; }
    if (!e) { snprintf(err, n, "no such shelf entry"); return false; }
    if (e->type == IMG_DISK) { snprintf(err, n, "that is a disk image"); return false; }
    if (!msx_cell_free(p, s, pages_for(e, shelf_idx, type), true, err, n)) return false;
    for (int k = 0; k < CART_MAX; k++) { uint32_t v = settings.cart_cell[k]; if (v && (int)(v & 0xFF) - 1 == shelf_idx && !((int)((v >> 16) & 3) == p && (int)((v >> 20) & 3) == s)) { snprintf(err, n, "already in slot %d-%d", (int)((v >> 16) & 3), (int)((v >> 20) & 3)); return false; } }
    int i = pend_index(p, s);
    if (i < 0) for (int k = 0; k < CART_MAX && i < 0; k++) if (!settings.cart_cell[k]) i = k;
    if (i < 0) { snprintf(err, n, "no free cartridge slot (%d in use)", CART_MAX); return false; }
    settings.cart_cell[i] = (uint32_t)(shelf_idx + 1) | ((uint32_t)type << 8) | ((uint32_t)p << 16) | ((uint32_t)s << 20);
    pend_save(); shelf_touch(shelf_idx);
    const cart_t *live = find(p, s); if (!live || live->shelf != shelf_idx) restart_needed = true;   // already running that ROM: nothing to apply
    return true;
}
void cart_eject(int p, int s) {
    int i = pend_index(p, s); if (i < 0) return;
    settings.cart_cell[i] = 0; pend_save(); if (find(p, s)) restart_needed = true;   // only a running ROM needs the restart to go away
}
void cart_unload_all(void) {   // MSX start: drop what runs, then cart_restore() loads the pending configuration
    for (int i = 0; i < CART_MAX; i++) if (carts[i].set) { int p = carts[i].p, s = carts[i].s; memset(&carts[i], 0, sizeof carts[i]); msx_set_cart_pages(p, s, 0); }
    restart_needed = false;
}
bool cart_pending_info(int p, int s, int *shelf, uint8_t *type, uint8_t *pages) {
    int i = pend_index(p, s); if (i < 0) return false;
    uint32_t v = settings.cart_cell[i]; int idx = (int)(v & 0xFF) - 1; const shelf_ent_t *e = shelf_get(idx); if (!e) return false;
    if (shelf) *shelf = idx; if (type) *type = (uint8_t)(v >> 8); if (pages) *pages = pages_for(e, idx, (uint8_t)(v >> 8)); return true;
}
bool cart_present(int p, int s) { return find(p, s) != NULL; }
bool cart_pending_uses_shelf(int shelf_idx) { for (int i = 0; i < CART_MAX; i++) { uint32_t v = settings.cart_cell[i]; if (v && (int)(v & 0xFF) - 1 == shelf_idx) return true; } return false; }
bool cart_any_uses_shelf(int shelf_idx, int *p, int *s) {
    for (int i = 0; i < CART_MAX; i++) if (carts[i].set && carts[i].shelf == shelf_idx) { if (p) *p = carts[i].p; if (s) *s = carts[i].s; return true; }
    return false;
}
void cart_shelf_deleted(int i) {                  // catalog entry i removed: indices above it shift down (an inserted ROM is never deleted)
    bool ch = false;
    for (int k = 0; k < CART_MAX; k++) if (carts[k].set && carts[k].shelf > i) { carts[k].shelf--; ch = true; }
    for (int k = 0; k < CART_MAX; k++) { uint32_t v = settings.cart_cell[k]; if (v && (int)(v & 0xFF) - 1 > i) { settings.cart_cell[k] = v - 1; ch = true; } else if (v && (int)(v & 0xFF) - 1 == i) { settings.cart_cell[k] = 0; ch = true; } }
    if (ch) pend_save();
}
void cart_restore(void) {
    char err[64]; bool any = false;
    for (int i = 0; i < CART_MAX; i++) if (settings.cart_cell[i]) any = true;
    for (int i = 0; i < CART_MAX; i++) {
        uint32_t v = any ? settings.cart_cell[i] : (i < 2 ? settings.cart_sel[i] : 0);    // pre-0.4.1 settings: slot 1 / 2 only
        if (!v) continue;
        int p = any ? (int)((v >> 16) & 3) : i + 1, s = any ? (int)((v >> 20) & 3) : 0;
        if (cart_present(p, s)) continue;
        if (!set_nosave(p, s, (int)(v & 0xFF) - 1, (uint8_t)(v >> 8), err, sizeof err)) printf("cart %d-%d: %s\n", p, s, err);
    }
}
bool cart_info(int p, int s, int *shelf, uint8_t *type, uint8_t *pages) {
    const cart_t *c = find(p, s); if (!c) return false;
    if (shelf) *shelf = c->shelf; if (type) *type = c->type; if (pages) *pages = c->pages; return true;
}

const uint8_t *cart_read_ptr(int p, int s, uint16_t a) {
    const cart_t *c = find(p, s);
    if (!c) return NULL;
    switch (c->type) {
        case IMG_PLAIN: { uint32_t off = a - (uint32_t)c->basepage * 0x4000; return (a >> 14) >= c->basepage && off < c->size ? c->base + off : NULL; }
        case IMG_ASCII16: { if (a < 0x4000 || a >= 0xC000) return NULL; uint32_t off = (uint32_t)c->bank[(a - 0x4000) >> 14] * 0x4000 + (a & 0x3FFF); return off < c->size ? c->base + off : NULL; }
        default: { if (a < 0x4000 || a >= 0xC000) return NULL; uint32_t off = (uint32_t)c->bank[(a - 0x4000) >> 13] * 0x2000 + (a & 0x1FFF); return off < c->size ? c->base + off : NULL; }
    }
}
bool cart_write(int p, int s, uint16_t a, uint8_t v) {
    cart_t *c = find(p, s);
    if (!c) return false;
    int i = -1; uint8_t mask;
    switch (c->type) {
        case IMG_ASCII8:  if (a >= 0x6000 && a < 0x8000) i = (a - 0x6000) >> 11; mask = (uint8_t)(c->nbanks8 - 1); break;
        case IMG_ASCII16: if (a >= 0x6000 && a < 0x6800) i = 0; else if (a >= 0x7000 && a < 0x7800) i = 1; mask = (uint8_t)((c->nbanks8 + 1) / 2 - 1); break;
        case IMG_KONAMI:  if (a >= 0x6000 && a < 0xC000) i = (a - 0x4000) >> 13; mask = (uint8_t)(c->nbanks8 - 1); if (i == 0) i = -1; break;
        case IMG_KONAMI_SCC: if ((a & 0x1800) == 0x1000 && a >= 0x5000 && a < 0xC000) i = (a - 0x4000) >> 13; mask = (uint8_t)(c->nbanks8 - 1); break;
        default: return false;
    }
    if (i < 0 || i > 3) return false;
    uint8_t nb = v & mask;
    if (c->bank[i] == nb) return false;
    c->bank[i] = nb; return true;
}
void cart_print_json(void) {
    printf("{\"slots\":[");
    bool first = true;
    for (int i = 0; i < CART_MAX; i++) {
        const cart_t *c = &carts[i]; if (!c->set) continue;
        char cell[8]; if (msx_slot_expanded(c->p)) snprintf(cell, sizeof cell, "%d-%d", c->p, c->s); else snprintf(cell, sizeof cell, "%d", c->p);
        printf("%s{\"slot\":\"%s\",\"p\":%d,\"s\":%d,\"set\":true,\"pages\":%u", first ? "" : ",", cell, c->p, c->s, c->pages);
        first = false;
        printf(",\"shelf\":%d,\"type\":%u,\"tname\":\"%s\",\"size\":%lu,\"banks\":[%u,%u,%u,%u],\"learn\":%d,\"hits\":[%u,%u,%u,%u,%u,%u,%u,%u,%u]}", c->shelf, c->type, img_type_name(c->type), (unsigned long)c->size, c->bank[0], c->bank[1], c->bank[2], c->bank[3], c->learn ? 1 : 0, c->hits[0], c->hits[1], c->hits[2], c->hits[3], c->hits[4], c->hits[5], c->hits[6], c->hits[7], c->hits[8]);
    }
    printf("]}\n");
}
