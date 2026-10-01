#include "cache.h"
#include "link.h"
#include "sd.h"
#include "web.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/stream_buffer.h"
static const char *TAG = "cache";
#define MOUNT "/sdcard"

bool sd_path_ok(const char *p) { return p && p[0] == '/' && !strstr(p, "..") && strlen(p) < 200; }
const char *sd_path_err(const char *p) {
    if (!p || !p[0]) return "err no path\n";
    if (strlen(p) >= 200) return "err path too long (200 文字以上。フォルダを含めて 199 文字まで)\n";
    if (p[0] != '/' || strstr(p, "..")) return "err bad path\n";
    if (!sd_mounted()) return "err no card\n";
    return "err bad path\n";
}
int img_type_for(const char *path, long size) {
    const char *e = strrchr(path, '.'); if (!e) return 0;
    if (!strcasecmp(e, ".hdd")) return 10;
    if (!strcasecmp(e, ".dsk") || !strcasecmp(e, ".dim") || !strcasecmp(e, ".img")) return size > 737280 ? 10 : 9;
    return 0;
}
static bool reply_ok(const char *cmd, char *out, size_t n, int ms) { return link_command(cmd, out, n, ms) > 0 && !strncmp(out, "ok", 2); }

// ---- catalog (JSON from the RP) ----------------------------------------------------------------
static char catalog[12288]; static bool have_catalog;
const char *cache_catalog_json(bool refresh) {
    if (refresh || !have_catalog) { if (link_command("shelf", catalog, sizeof catalog, 3000) > 0 && catalog[0] == '{') have_catalog = true; else { catalog[0] = 0; have_catalog = false; } }
    return catalog;
}
static const char *find_entry(int idx) {   // pointer to "{\"i\":idx," in the catalog
    char key[24]; snprintf(key, sizeof key, "{\"i\":%d,", idx);
    return strstr(catalog, key);
}
static uint32_t field_u(const char *obj, const char *name) { char k[24]; snprintf(k, sizeof k, "\"%s\":", name); const char *p = strstr(obj, k); return p ? strtoul(p + strlen(k), NULL, 10) : 0; }
bool cache_entry(int idx, char *path, size_t pn, uint32_t *size, uint32_t *mtime, int *dirty) {
    cache_catalog_json(false);
    const char *o = find_entry(idx); if (!o) return false;
    const char *p = strstr(o, "\"path\":\""); if (!p) return false; p += 8;
    size_t i = 0; while (*p && *p != '"' && i < pn - 1) { if (*p == '\\') p++; path[i++] = *p++; } path[i] = 0;
    if (size) *size = field_u(o, "size");
    if (mtime) *mtime = field_u(o, "mtime");
    if (dirty) *dirty = (int)field_u(o, "dirty");
    return true;
}
int cache_lookup(const char *path, uint32_t *size, uint32_t *mtime, int *dirty) {
    cache_catalog_json(false);
    for (int i = 0; i < 64; i++) { char p[208]; if (!cache_entry(i, p, sizeof p, size, mtime, dirty)) continue; if (!strcmp(p, path)) return i; }
    return -1;
}
int cache_find(const char *path) { return cache_lookup(path, NULL, NULL, NULL); }

// ---- import / uncache ----------------------------------------------------------------------------
bool cache_import(const char *path, char *status, size_t n) {
    static uint8_t buf[4096]; char full[256], cmd[320], st[200];
    if (!sd_path_ok(path) || !sd_mounted()) { snprintf(status, n, "%s", sd_path_err(path)); return false; }
    snprintf(full, sizeof full, MOUNT "%s", path);
    struct stat sb; if (stat(full, &sb) != 0 || !S_ISREG(sb.st_mode) || sb.st_size <= 0) { snprintf(status, n, "err not a file"); return false; }
    FILE *f = fopen(full, "rb"); if (!f) { snprintf(status, n, "err cannot open"); return false; }
    snprintf(cmd, sizeof cmd, "shelf put %ld %d %lu %s", (long)sb.st_size, img_type_for(path, (long)sb.st_size), (unsigned long)sb.st_mtime, path);
    if (link_push_begin(cmd, st, sizeof st)) { fclose(f); snprintf(status, n, "%s", st); return false; }
    long got = 0;
    while (got < sb.st_size) {
        size_t k = fread(buf, 1, sizeof buf, f);
        if (!k || link_fwup_piece(buf, k, 8000)) { fclose(f); link_fwup_end(1000); snprintf(status, n, "%s", k ? st : "err read"); return false; }
        got += k;
        if ((got & 0xFFFFF) == 0) web_touch();   // a 48 MB hard disk image takes minutes: not a stalled handler
    }
    fclose(f);
    int rc = link_fwup_end(10000);
    snprintf(status, n, "%s", st); have_catalog = false;
    return rc == 0;
}
bool cache_uncache(const char *path, char *status, size_t n) {
    int i = cache_lookup(path, NULL, NULL, NULL); char cmd[32], out[64];
    if (i < 0) { snprintf(status, n, "err not cached"); return false; }
    snprintf(cmd, sizeof cmd, "shelf del %d", i);
    bool ok = reply_ok(cmd, out, sizeof out, 3000); snprintf(status, n, "%s", out); have_catalog = false; return ok;
}

// ---- mount / eject ------------------------------------------------------------------------------
// Slot cell names accepted by the RP: "1", "2" (cartridge slots) or "P-S" (expanded slot P, secondary S).
static bool cell_ok(const char *t) { size_t l = strlen(t); return (l == 1 && t[0] >= '0' && t[0] <= '3') || (l == 3 && t[0] >= '0' && t[0] <= '3' && t[1] == '-' && t[2] >= '0' && t[2] <= '3'); }
static void hist_add(const char *path);
bool cache_mount(const char *path, const char *target, int type, char *status, size_t n) {
    char full[256], cmd[48], out[96]; uint32_t csize = 0, cmtime = 0;
    if (!sd_path_ok(path) || !target || !target[0]) { snprintf(status, n, "err bad path or target"); return false; }
    snprintf(full, sizeof full, MOUNT "%s", path);
    struct stat sb; if (stat(full, &sb) != 0) { snprintf(status, n, "err file not found on the card"); return false; }
    int dirty = 0; int idx = cache_lookup(path, &csize, &cmtime, &dirty);
    if (idx < 0 || (!dirty && (csize != (uint32_t)sb.st_size || cmtime != (uint32_t)sb.st_mtime))) {   // not cached, or the card copy changed (a dirty entry is newer than the card: keep it)
        if (!cache_import(path, status, n)) return false;
        for (int tries = 0; tries < 5; tries++) {                       // the RP is still saving its catalog: give it a moment
            have_catalog = false; idx = cache_lookup(path, NULL, NULL, NULL);
            if (idx >= 0) break;
            vTaskDelay(pdMS_TO_TICKS(400));
        }
        if (idx < 0) { snprintf(status, n, "err cache lookup after import"); return false; }
    }
    if (type >= 0 && type != 9 && img_type_for(path, (long)sb.st_size) == 0) {   // ROM mapper chosen by the user (0 = auto)
        char tc[48], to[32]; snprintf(tc, sizeof tc, "shelf type %d %d", idx, type);
        if (!reply_ok(tc, to, sizeof to, 3000)) { snprintf(status, n, "%s", to); return false; }
    }
    bool disk = target[0] == 'A' || target[0] == 'B' || target[0] == 'V' || target[0] == 'P';
    if (disk) snprintf(cmd, sizeof cmd, "fd %s virtual %d", target, idx);
    else if (cell_ok(target)) snprintf(cmd, sizeof cmd, "cart set %s %d", target, idx);
    else { snprintf(status, n, "err target A|B|V1..V6|1|2|P-S"); return false; }
    bool ok = reply_ok(cmd, out, sizeof out, 8000); snprintf(status, n, "%s", out);
    if (ok && disk) hist_add(path);
    return ok;
}
// Mount history (disk images only): newest first, 10 entries, kept in NVS so every browser sees the same list.
#include "nvs.h"
#define HIST_N 10
static void hist_add(const char *path) {
    nvs_handle_t h; if (nvs_open("vz80", NVS_READWRITE, &h) != ESP_OK) return;
    static char buf[HIST_N * 208 + 8], out[HIST_N * 208 + 8]; size_t len = sizeof buf; buf[0] = 0;
    if (nvs_get_str(h, "hist", buf, &len) != ESP_OK) buf[0] = 0;
    size_t o = 0; o += snprintf(out + o, sizeof out - o, "%s\n", path); int cnt = 1;
    for (char *p = buf; *p && cnt < HIST_N; ) { char *e = strchr(p, '\n'); size_t l = e ? (size_t)(e - p) : strlen(p); if (l && !(l == strlen(path) && !strncmp(p, path, l))) { o += snprintf(out + o, sizeof out - o, "%.*s\n", (int)l, p); cnt++; } if (!e) { break; } p = e + 1; }
    nvs_set_str(h, "hist", out); nvs_commit(h); nvs_close(h);
}
int cache_hist_json(char *out, size_t n) {
    nvs_handle_t h; static char buf[HIST_N * 208 + 8]; buf[0] = 0; size_t len = sizeof buf;
    if (nvs_open("vz80", NVS_READONLY, &h) == ESP_OK) { if (nvs_get_str(h, "hist", buf, &len) != ESP_OK) buf[0] = 0; nvs_close(h); }
    size_t o = 0; o += snprintf(out + o, n - o, "{\"hist\":["); bool first = true;
    for (char *p = buf; *p; ) { char *e = strchr(p, '\n'); size_t l = e ? (size_t)(e - p) : strlen(p);
        if (l) { o += snprintf(out + o, n - o, "%s\"", first ? "" : ","); first = false; for (size_t i = 0; i < l && o < n - 8; i++) { char c = p[i]; if (c == '"' || c == '\\') { out[o++] = '\\'; } out[o++] = c; } out[o] = 0; o += snprintf(out + o, n - o, "\""); }
        if (!e) { break; } p = e + 1; }
    o += snprintf(out + o, n - o, "]}\n"); return (int)o;
}
bool cache_vd(const char *op, const char *arg, char *status, size_t n) {
    char cmd[64], out[160];
    if (!op || !arg || !*op || !*arg || strlen(arg) > 24) { snprintf(status, n, "err vd op=add|del|order"); return false; }
    snprintf(cmd, sizeof cmd, "vd %s %s", op, arg);
    bool ok = reply_ok(cmd, out, sizeof out, 5000); snprintf(status, n, "%s", out); return ok;
}
bool cache_fdswap(const char *a, const char *b, char *status, size_t n) {
    char cmd[40], out[96]; snprintf(cmd, sizeof cmd, "fd swap %s %s", a, b);
    bool ok = reply_ok(cmd, out, sizeof out, 5000); snprintf(status, n, "%s", out); return ok;
}
bool cache_eject(const char *target, char *status, size_t n) {
    char cmd[32], out[64];
    if (target && (target[0] == 'A' || target[0] == 'B' || target[0] == 'V' || target[0] == 'P')) snprintf(cmd, sizeof cmd, "fd %s eject", target);
    else if (target && cell_ok(target)) snprintf(cmd, sizeof cmd, "cart eject %s", target);
    else { snprintf(status, n, "err target A|B|V1..V6|1|2|P-S"); return false; }
    bool ok = reply_ok(cmd, out, sizeof out, 8000); snprintf(status, n, "%s", out); return ok;
}

// ---- copy-back of virtual floppy writes ----------------------------------------------------------
// Every 2 s: `fdsync` hands over dirty sectors (516-byte records: shelf idx, sector, data), written to
// the card file at sector * 512. When none are left and an entry still carries the persisted dirty
// flag, the whole image is copied back once (`shelf get`) unless every write since the flag went up
// passed through here, then the flag is cleared (`shelf dirty I 0`).
static bool written_ok[64], write_fail[64];
static uint32_t last_negs;
static void note_mtime(int idx, const char *full) {   // the copy-back changed the card file: refresh the cached mtime so the entry stays "fresh"
    struct stat sb; char cmd[40], out[32];
    if (stat(full, &sb) != 0) return;
    snprintf(cmd, sizeof cmd, "shelf mtime %d %lu", idx, (unsigned long)sb.st_mtime);
    if (reply_ok(cmd, out, sizeof out, 3000)) have_catalog = false;
}
static bool write_sectors(int idx, const uint8_t *rec, int count) {
    char path[208], full[256]; if (!cache_entry(idx, path, sizeof path, NULL, NULL, NULL)) return false;
    snprintf(full, sizeof full, MOUNT "%s", path);
    FILE *f = fopen(full, "r+b"); if (!f) return false;
    bool ok = true;
    for (int i = 0; i < count; i++) {
        const uint8_t *r = rec + i * 516; uint32_t sector = r[2] | (r[3] << 8);
        if (fseek(f, (long)sector * 512, SEEK_SET) != 0 || fwrite(r + 4, 1, 512, f) != 512) { ok = false; char cmd[32], out[32]; snprintf(cmd, sizeof cmd, "fddirty %d %lu", idx, (unsigned long)sector); link_command(cmd, out, sizeof out, 1000); }
    }
    fclose(f); note_mtime(idx, full); return ok;
}
static bool write_block(int idx, const uint8_t *rec) {   // hdsync record: shelf, pad[3], u32 block, 4096 bytes
    char path[208], full[256]; if (!cache_entry(idx, path, sizeof path, NULL, NULL, NULL)) return false;
    snprintf(full, sizeof full, MOUNT "%s", path);
    uint32_t block; memcpy(&block, rec + 4, 4);
    FILE *f = fopen(full, "r+b"); if (!f) return false;
    bool ok = fseek(f, (long)block * 4096, SEEK_SET) == 0 && fwrite(rec + 8, 1, 4096, f) == 4096;
    if (!ok) { char cmd[32], out[32]; snprintf(cmd, sizeof cmd, "hddirty %d %lu", idx, (unsigned long)block); link_command(cmd, out, sizeof out, 1000); }
    fclose(f); note_mtime(idx, full); return ok;
}
static bool full_copy_back(int idx) {
    // The link has no flow control, so the body is pulled in 8 KB requests (each answered "bin 8192" + data).
    char path[208], full[256]; uint32_t size = 0; if (!cache_entry(idx, path, sizeof path, &size, NULL, NULL)) return false;
    snprintf(full, sizeof full, MOUNT "%s", path);
    FILE *f = fopen(full, "r+b"); if (!f) { ESP_LOGW(TAG, "copy-back %s: cannot open", path); return false; }
    static uint8_t buf[2048]; bool ok = true;
    for (uint32_t off = 0; off < size && ok; off += 8192) {
        uint32_t want = size - off < 8192 ? size - off : 8192;
        char cmd[40]; snprintf(cmd, sizeof cmd, "shelf get %d %lu %lu", idx, (unsigned long)off, (unsigned long)want);
        // link_negotiate() (every 5 s) raises neg_wanted, which makes link_binary_begin() refuse at once;
        // a whole image takes longer than that, so wait for the negotiation to pass instead of failing.
        uint32_t total; StreamBufferHandle_t sb = NULL;
        for (int tries = 0; tries < 40 && !(sb = link_binary_begin(cmd, &total, 3000)); tries++) { if (!link_healthy()) break; vTaskDelay(pdMS_TO_TICKS(100)); }
        if (!sb) { ESP_LOGW(TAG, "copy-back %s: no bin header at %lu (%s)", path, (unsigned long)off, link_last_reply()); ok = false; break; }
        if (total != want) { ESP_LOGW(TAG, "copy-back %s: bin %lu, wanted %lu at %lu", path, (unsigned long)total, (unsigned long)want, (unsigned long)off); ok = false; }
        uint32_t left = ok ? total : 0;
        while (left) { size_t k = xStreamBufferReceive(sb, buf, left < sizeof buf ? left : sizeof buf, pdMS_TO_TICKS(2000)); if (!k) { ESP_LOGW(TAG, "copy-back %s: short read, %lu left at %lu", path, (unsigned long)left, (unsigned long)off); ok = false; break; } if (fwrite(buf, 1, k, f) != k) { ESP_LOGW(TAG, "copy-back %s: card write failed at %lu", path, (unsigned long)off); ok = false; } left -= k; }
        link_binary_end(ok ? 2000 : 200);
        if (!ok) break;
        if (off % (8192 * 16) == 0) vTaskDelay(1);   // let UI commands through now and then
    }
    fclose(f); if (ok) note_mtime(idx, full);
    ESP_LOGI(TAG, "copy-back %s: %s", path, ok ? "ok" : "FAILED");
    return ok;
}
static void sync_task(void *arg) {
    static uint8_t recs[8 * 516];
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(2000));
        if (!link_healthy() || !sd_mounted()) continue;
        uint32_t negs = link_resyncs();
        if (negs != last_negs) { last_negs = negs; memset(written_ok, 0, sizeof written_ok); }   // the RP rebooted (baud re-synced): its bitmap is gone
        uint32_t total; StreamBufferHandle_t sb = link_binary_begin("fdsync", &total, 2000);
        if (!sb) continue;
        uint32_t left = total; bool ok = true; int count = 0;
        while (left && count * 516 < (int)sizeof recs) {   // the stream buffer wakes us on the first byte: gather a whole record
            size_t got = 0;
            while (got < 516) { size_t k = xStreamBufferReceive(sb, recs + count * 516 + got, 516 - got, pdMS_TO_TICKS(2000)); if (!k) break; got += k; }
            if (got != 516) { ok = false; break; }
            left -= 516; count++;
        }
        link_binary_end(ok ? 2000 : 200);
        if (!ok) { ESP_LOGW(TAG, "fdsync: %lu of %lu bytes lost", (unsigned long)left, (unsigned long)total); continue; }
        if (count) ESP_LOGI(TAG, "fdsync: %d sector(s), first idx %d sector %u", count, recs[0], recs[2] | (recs[3] << 8));
        if (count) {
            have_catalog = false;                                     // entries may have gone dirty
            for (int i = 0; i < count; ) {                            // group by entry
                int idx = recs[i * 516]; int j = i; while (j < count && recs[j * 516] == idx) j++;
                if (write_sectors(idx, recs + i * 516, j - i)) { if (!write_fail[idx]) written_ok[idx] = true; } else { write_fail[idx] = true; }
                i = j;
            }
            continue;
        }
        // Hard disk rows: one 4 KB block per round (4104-byte record), until none is left.
        sb = link_binary_begin("hdsync", &total, 2000);
        if (!sb) continue;
        left = total; ok = true; size_t got = 0;
        while (left && got < 4104) { size_t k = xStreamBufferReceive(sb, recs + got, 4104 - got, pdMS_TO_TICKS(2000)); if (!k) break; got += k; left -= k; }
        if (total && got != 4104) ok = false;
        link_binary_end(ok ? 2000 : 200);
        if (!ok) { ESP_LOGW(TAG, "hdsync: %lu of %lu bytes lost", (unsigned long)left, (unsigned long)total); continue; }
        if (total) {
            int idx = recs[0]; uint32_t block; memcpy(&block, recs + 4, 4);
            have_catalog = false;
            if (write_block(idx, recs)) { if (!write_fail[idx]) written_ok[idx] = true; } else { write_fail[idx] = true; ESP_LOGW(TAG, "hdsync: block %lu of #%d failed", (unsigned long)block, idx); }
            continue;
        }
        // Nothing pending: clear persisted flags (full copy first if this session did not see every write).
        cache_catalog_json(true);
        for (int idx = 0; idx < 64; idx++) {
            char path[208]; int dirty = 0; if (!cache_entry(idx, path, sizeof path, NULL, NULL, &dirty) || !dirty) continue;
            if (!written_ok[idx] || write_fail[idx]) { ESP_LOGI(TAG, "copy-back %s: whole image (seen=%d fail=%d)", path, written_ok[idx], write_fail[idx]); if (!full_copy_back(idx)) continue; }
            char cmd[32], out[32]; snprintf(cmd, sizeof cmd, "shelf dirty %d 0", idx);
            if (reply_ok(cmd, out, sizeof out, 3000)) { written_ok[idx] = false; write_fail[idx] = false; have_catalog = false; }
        }
    }
}
void cache_start(void) { xTaskCreate(sync_task, "fdsync", 6144, NULL, 2, NULL); }
