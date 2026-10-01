#include "files.h"
#include "esp_log.h"
#include "bootcode.h"
#include "esp_random.h"
#include "cache.h"
#include "sd.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/stat.h>
#include <dirent.h>
#include <errno.h>
#include <ctype.h>
#define MOUNT "/sdcard"
static uint8_t iobuf[4096];   // shared by the handlers (they run one at a time)

void url_decode(char *s) {      // %XX and '+' in place (httpd_query_key_value leaves them encoded)
    char *w = s;
    for (; *s; s++) {
        if (*s == '+') *w++ = ' ';
        else if (*s == '%' && isxdigit((unsigned char)s[1]) && isxdigit((unsigned char)s[2])) { char h[3] = { s[1], s[2], 0 }; *w++ = (char)strtol(h, NULL, 16); s += 2; }
        else *w++ = *s;
    }
    *w = 0;
}
static bool q_arg(httpd_req_t *r, const char *key, char *out, size_t n) {
    char q[700]; out[0] = 0;
    if (httpd_req_get_url_query_str(r, q, sizeof q) != ESP_OK) return false;
    if (httpd_query_key_value(q, key, out, n) != ESP_OK) return false;
    url_decode(out);
    return out[0] != 0;
}
static esp_err_t text(httpd_req_t *r, const char *status, const char *msg) {
    httpd_resp_set_type(r, "text/plain"); if (status) httpd_resp_set_status(r, status);
    return httpd_resp_send(r, msg, HTTPD_RESP_USE_STRLEN);
}
static bool full_path(const char *p, char *out, size_t n) { if (!sd_path_ok(p) || !sd_mounted()) return false; snprintf(out, n, MOUNT "%s", p); return true; }
static bool textish(const char *p) { const char *e = strrchr(p, '.'); return e && (!strcasecmp(e, ".txt") || !strcasecmp(e, ".cfg") || !strcasecmp(e, ".bat") || !strcasecmp(e, ".ini") || !strcasecmp(e, ".log") || !strcasecmp(e, ".md") || !strcasecmp(e, ".bas") || !strcasecmp(e, ".asm") || !strcasecmp(e, ".sys")); }

esp_err_t h_file_get(httpd_req_t *r) {
    char path[224], full[256]; uint8_t *buf = iobuf;
    if (!q_arg(r, "path", path, sizeof path) || !full_path(path, full, sizeof full)) return text(r, "400 Bad Request", sd_path_err(path));
    FILE *f = fopen(full, "rb"); if (!f) return text(r, "404 Not Found", "err not found\n");
    httpd_resp_set_type(r, textish(path) ? "text/plain; charset=utf-8" : "application/octet-stream");
    httpd_resp_set_hdr(r, "Cache-Control", "no-store");
    size_t k; while ((k = fread(buf, 1, 4096, f)) > 0) if (httpd_resp_send_chunk(r, (char *)buf, k) != ESP_OK) break;
    fclose(f); return httpd_resp_send_chunk(r, NULL, 0);
}
// POST /api/file?path= and POST /api/upload?path=[&off=N] : the body becomes the file (streamed, any size).
// With off, the body is one piece of the file written at that offset (off 0 creates the file): the Web UI
// sends files in pieces to get progress in every browser.
esp_err_t h_file_put(httpd_req_t *r) {
    char path[224], full[256], offs[16] = ""; uint8_t *buf = iobuf;
    if (!q_arg(r, "path", path, sizeof path) || !full_path(path, full, sizeof full)) return text(r, "400 Bad Request", sd_path_err(path));
    long off = q_arg(r, "off", offs, sizeof offs) ? atol(offs) : -1;
    if (off <= 0 && cache_find(path) >= 0) { char st[64]; cache_uncache(path, st, sizeof st); if (cache_find(path) >= 0) return text(r, "409 Conflict", "err in use (mounted)\n"); }   // the card copy is about to change: drop the stale cache
    // Safari (CFNetwork) uploads a file with "Expect: 100-continue" and waits for the interim reply before
    // sending the body; esp_http_server never sends one, so answer it here.
    char hv[24] = "", te[24] = "";
    httpd_req_get_hdr_value_str(r, "Expect", hv, sizeof hv); httpd_req_get_hdr_value_str(r, "Transfer-Encoding", te, sizeof te);
    char cn[24] = "", ct[48] = "", ua[80] = "";
    httpd_req_get_hdr_value_str(r, "Connection", cn, sizeof cn); httpd_req_get_hdr_value_str(r, "Content-Type", ct, sizeof ct); httpd_req_get_hdr_value_str(r, "User-Agent", ua, sizeof ua);
    ESP_LOGI("files", "upload %s: %lu bytes, expect='%s' te='%s' conn='%s' ct='%s' ua='%s'", path, (unsigned long)r->content_len, hv, te, cn, ct, ua);
    if (strcasestr(hv, "100-continue")) { static const char cont[] = "HTTP/1.1 100 Continue\r\n\r\n"; httpd_send(r, cont, sizeof cont - 1); }
    FILE *f = fopen(full, off > 0 ? "r+b" : "wb"); if (!f) return text(r, "500 Internal Server Error", off > 0 ? "err cannot open\n" : "err cannot create\n");
    if (off > 0 && fseek(f, off, SEEK_SET) != 0) { fclose(f); return text(r, "500 Internal Server Error", "err seek failed\n"); }
    size_t left = r->content_len; bool ok = true;
    while (left) { int k = httpd_req_recv(r, (char *)buf, left < 4096 ? left : 4096); if (k <= 0) { ok = false; ESP_LOGW("files", "upload %s: recv %d with %lu left", path, k, (unsigned long)left); break; } if (fwrite(buf, 1, k, f) != (size_t)k) { ok = false; break; } left -= k; }
    fclose(f);
    ESP_LOGI("files", "upload %s: %s (%lu left)", path, ok ? "written" : "FAILED", (unsigned long)left);
    if (!ok) { if (off <= 0) unlink(full); return text(r, "500 Internal Server Error", "err write failed\n"); }
    if (!strcasecmp(path, "/vz80.cfg")) sd_reload_cfg();          // Wi-Fi settings live on the card: apply the edit right away
    char out[160]; snprintf(out, sizeof out, "ok %lu bytes\n", (unsigned long)r->content_len); esp_err_t e = text(r, NULL, out);
    ESP_LOGI("files", "upload %s: reply %s", path, esp_err_to_name(e)); return e;
}
esp_err_t h_mkdir(httpd_req_t *r) {
    char path[224], full[256];
    if (!q_arg(r, "path", path, sizeof path) || !full_path(path, full, sizeof full)) return text(r, "400 Bad Request", "err bad path\n");
    int rc = mkdir(full, 0777);
    if (rc == 0 || errno == EEXIST) return text(r, NULL, "ok\n");
    return text(r, "409 Conflict", "err mkdir failed\n");
}
esp_err_t h_mv(httpd_req_t *r) {
    char from[128], to[128], f1[160], f2[160], st[64];
    if (!q_arg(r, "from", from, sizeof from) || !q_arg(r, "to", to, sizeof to) || !full_path(from, f1, sizeof f1) || !full_path(to, f2, sizeof f2)) return text(r, "400 Bad Request", "err bad path\n");
    if (cache_find(from) >= 0) cache_uncache(from, st, sizeof st);   // (a mounted image refuses: keep the card file where it is)
    if (cache_find(from) >= 0) return text(r, "409 Conflict", "err in use (mounted)\n");
    struct stat st2;
    if (stat(f2, &st2) == 0) return text(r, "409 Conflict", "err 移動先に同じ名前のファイルがあります\n");   // FAT rename cannot overwrite
    if (stat(f1, &st2) != 0) return text(r, "404 Not Found", "err not found\n");
    int rc = rename(f1, f2);
    if (rc != 0) { char out[96]; snprintf(out, sizeof out, "err move failed (%s)\n", strerror(errno)); return text(r, "409 Conflict", out); }
    return text(r, NULL, "ok\n");
}
esp_err_t h_rm(httpd_req_t *r) {
    char path[224], full[256], st[64];
    if (!q_arg(r, "path", path, sizeof path) || !full_path(path, full, sizeof full)) return text(r, "400 Bad Request", "err bad path\n");
    if (cache_find(path) >= 0) { cache_uncache(path, st, sizeof st); if (cache_find(path) >= 0) return text(r, "409 Conflict", "err in use (mounted)\n"); }
    struct stat sb; if (stat(full, &sb) != 0) return text(r, "404 Not Found", "err not found\n");
    int rc = S_ISDIR(sb.st_mode) ? rmdir(full) : unlink(full);
    return text(r, rc == 0 ? NULL : "409 Conflict", rc == 0 ? "ok\n" : "err delete failed (folder not empty?)\n");
}
// POST /api/mkimg?path=/name.dsk[&fmt=dos1|dos2] : blank MSX 2DD image (720 KB, FAT12, 9 sectors, 2 sides,
// 112 root entries). fmt=dos1 (default) writes an MSX-DOS 1 style boot sector (code at 1Eh); fmt=dos2 writes the
// MSX-DOS 2 style one (VOL_ID block + code at 30h), which makes Nextor boot in DOS 2 mode. Boot code: tools/bootsec.
// fmt=hdd&mb=N (1..48): hard disk image for the Nextor rows, one FAT16 volume without a partition table
// (Nextor maps a device without partitions to its sector 0). Layout as Nextor's FDISK would make it for
// a volume of up to 128 MB (4 sectors per cluster, 512 root entries, standard boot sector with the 29h
// extended block and the 32-bit sector count), except that the FATs are sized for 48 MB whatever the
// size, so the volume can grow later without moving the data (h_imgresize).
#define HDD_SPC 4
#define HDD_ROOT_SECS 32
#define HDD_SPF 96
#define HDD_DATA_SEC (1 + 2 * HDD_SPF + HDD_ROOT_SECS)   // 225
#define HDD_MAX_MB 48
static void hdd_boot_sector(uint8_t *sec, uint32_t total, uint32_t serial) {
    memset(sec, 0, 512);
    memcpy(sec, "\xEB\xFE\x90VZ80    ", 11);
    sec[11] = 0x00; sec[12] = 0x02; sec[13] = HDD_SPC; sec[14] = 1; sec[15] = 0; sec[16] = 2; sec[17] = 0x00; sec[18] = 0x02;   // 512 root entries
    if (total < 65536) { sec[19] = (uint8_t)total; sec[20] = (uint8_t)(total >> 8); }
    sec[21] = 0xF0; sec[22] = HDD_SPF; sec[23] = 0;
    memcpy(sec + 0x20, &total, 4);
    sec[0x26] = 0x29; memcpy(sec + 0x27, &serial, 4); memcpy(sec + 0x2B, "VZ80 HDD   ", 11); memcpy(sec + 0x36, "FAT16   ", 8);
    sec[510] = 0x55; sec[511] = 0xAA;
}
static bool hdd_layout_ok(const uint8_t *b) {   // a boot sector this firmware made
    return b[11] == 0 && b[12] == 2 && b[13] == HDD_SPC && b[14] == 1 && b[15] == 0 && b[16] == 2 && b[17] == 0 && b[18] == 2 && b[22] == HDD_SPF && b[23] == 0 && !memcmp(b + 0x36, "FAT16   ", 8) && b[510] == 0x55 && b[511] == 0xAA;
}
static esp_err_t mk_hdd(httpd_req_t *r, const char *full, int mb) {
    static uint8_t sec[512];
    if (mb < 1 || mb > HDD_MAX_MB) return text(r, "400 Bad Request", "err mb must be 1..48\n");
    FILE *f = fopen(full, "wb"); if (!f) return text(r, "500 Internal Server Error", "err cannot create\n");
    uint32_t total = (uint32_t)mb * 2048; bool ok = true;
    hdd_boot_sector(sec, total, esp_random());
    ok = fwrite(sec, 1, 512, f) == 512;
    for (int s = 1; s < HDD_DATA_SEC && ok; s++) {
        memset(sec, 0, 512);
        if (s == 1 || s == 1 + HDD_SPF) { sec[0] = 0xF0; sec[1] = 0xFF; sec[2] = 0xFF; sec[3] = 0xFF; }   // FAT16: media byte entry and end mark
        ok = fwrite(sec, 1, 512, f) == 512;
    }
    // The data area is only allocated (free clusters are never read): seek to the end and write one byte.
    if (ok) ok = fseek(f, (long)total * 512 - 1, SEEK_SET) == 0 && fputc(0, f) == 0;
    fclose(f);
    if (!ok) { unlink(full); return text(r, "500 Internal Server Error", "err write failed (card full?)\n"); }
    char out[80]; snprintf(out, sizeof out, "ok %lu bytes (HDD %d MB)\n", (unsigned long)total * 512, mb); return text(r, NULL, out);
}
// POST /api/imgresize?path=/name.hdd&mb=N : change the size of a vz80 hard disk image. Growing extends
// the file; shrinking is refused while clusters beyond the new end are in use (no data is moved).
esp_err_t h_imgresize(httpd_req_t *r) {
    char path[224], full[256], mbs[8] = ""; static uint8_t sec[512];
    if (!q_arg(r, "path", path, sizeof path) || !full_path(path, full, sizeof full)) return text(r, "400 Bad Request", "err bad path\n");
    int mb = q_arg(r, "mb", mbs, sizeof mbs) ? atoi(mbs) : 0;
    if (mb < 1 || mb > HDD_MAX_MB) return text(r, "400 Bad Request", "err mb must be 1..48\n");
    if (cache_find(path) >= 0) { char st[64]; cache_uncache(path, st, sizeof st); if (cache_find(path) >= 0) return text(r, "409 Conflict", "err in use (mounted)\n"); }   // the card copy is about to change
    FILE *f = fopen(full, "r+b"); if (!f) return text(r, "404 Not Found", "err not found\n");
    if (fread(sec, 1, 512, f) != 512 || !hdd_layout_ok(sec)) { fclose(f); return text(r, "409 Conflict", "err not a vz80 hard disk image\n"); }
    uint32_t old_total = sec[19] | (sec[20] << 8); if (!old_total) memcpy(&old_total, sec + 0x20, 4);
    uint32_t total = (uint32_t)mb * 2048; bool ok = true; char out[160];
    if (total < old_total) {
        // FAT16 entries of the clusters that would disappear must all be free (data is not moved).
        uint32_t new_clu = (total - HDD_DATA_SEC) / HDD_SPC, old_clu = (old_total - HDD_DATA_SEC) / HDD_SPC, used = 0;
        for (uint32_t c = new_clu + 2; c < old_clu + 2 && ok; c += 256) {
            ok = fseek(f, 512 + (long)c * 2, SEEK_SET) == 0 && fread(sec, 1, 512, f) == 512;
            for (uint32_t i = 0; ok && i < 256 && c + i < old_clu + 2; i++) if (sec[i * 2] || sec[i * 2 + 1]) used++;
        }
        if (!ok) { fclose(f); return text(r, "500 Internal Server Error", "err read failed\n"); }
        if (used) { fclose(f); snprintf(out, sizeof out, "err 縮小後の範囲より後ろに使用中のクラスタが %lu 個あります（先にファイルを減らすか整理してください）\n", (unsigned long)used); return text(r, "409 Conflict", out); }
    }
    uint32_t serial; fseek(f, 0, SEEK_SET); fread(sec, 1, 512, f); memcpy(&serial, sec + 0x27, 4);
    hdd_boot_sector(sec, total, serial);
    ok = fseek(f, 0, SEEK_SET) == 0 && fwrite(sec, 1, 512, f) == 512;
    if (ok && total > old_total) ok = fseek(f, (long)total * 512 - 1, SEEK_SET) == 0 && fputc(0, f) == 0;
    if (ok && total < old_total) ok = fflush(f) == 0 && ftruncate(fileno(f), (off_t)total * 512) == 0;
    fclose(f);
    if (!ok) return text(r, "500 Internal Server Error", "err write failed (card full?)\n");
    snprintf(out, sizeof out, "ok %lu bytes (HDD %d MB)\n", (unsigned long)total * 512, mb); return text(r, NULL, out);
}
// GET /api/romhead?path= : what decides a ROM's pages: size, the 4 bytes at 0 and the 2 bytes at 4000h (hex).
esp_err_t h_romhead(httpd_req_t *r) {
    char path[224], full[256]; uint8_t h0[4] = { 0 }, h1[2] = { 0 };
    if (!q_arg(r, "path", path, sizeof path) || !full_path(path, full, sizeof full)) return text(r, "400 Bad Request", "err bad path\n");
    struct stat sb; if (stat(full, &sb) != 0) return text(r, "404 Not Found", "err not found\n");
    FILE *f = fopen(full, "rb"); if (!f) return text(r, "404 Not Found", "err not found\n");
    fread(h0, 1, 4, f); if (sb.st_size > 0x4000 && fseek(f, 0x4000, SEEK_SET) == 0) fread(h1, 1, 2, f);
    fclose(f);
    char out[120]; snprintf(out, sizeof out, "{\"size\":%ld,\"h0\":\"%02x%02x%02x%02x\",\"h1\":\"%02x%02x\"}\n", (long)sb.st_size, h0[0], h0[1], h0[2], h0[3], h1[0], h1[1]);
    httpd_resp_set_type(r, "application/json"); httpd_resp_set_hdr(r, "Cache-Control", "no-store"); return httpd_resp_send(r, out, HTTPD_RESP_USE_STRLEN);
}
esp_err_t h_mkimg(httpd_req_t *r) {
    char path[224], full[256], fmt[8] = "dos1", mbs[8] = ""; static uint8_t sec[512];
    if (!q_arg(r, "path", path, sizeof path) || !full_path(path, full, sizeof full)) return text(r, "400 Bad Request", "err bad path\n");
    if (!q_arg(r, "fmt", fmt, sizeof fmt)) strcpy(fmt, "dos1");
    bool dos2 = strcmp(fmt, "dos2") == 0, hdd = strcmp(fmt, "hdd") == 0;
    if (!dos2 && !hdd && strcmp(fmt, "dos1") != 0) return text(r, "400 Bad Request", "err fmt must be dos1, dos2 or hdd\n");
    struct stat sb; if (stat(full, &sb) == 0) return text(r, "409 Conflict", "err exists\n");
    if (hdd) return mk_hdd(r, full, q_arg(r, "mb", mbs, sizeof mbs) ? atoi(mbs) : 0);
    FILE *f = fopen(full, "wb"); if (!f) return text(r, "500 Internal Server Error", "err cannot create\n");
    memset(sec, 0, sizeof sec);
    memcpy(sec, "\xEB\xFE\x90VZ80    ", 11);
    sec[11] = 0x00; sec[12] = 0x02; sec[13] = 2; sec[14] = 1; sec[15] = 0; sec[16] = 2; sec[17] = 112; sec[18] = 0;
    sec[19] = 0xA0; sec[20] = 0x05; sec[21] = 0xF9; sec[22] = 3; sec[23] = 0; sec[24] = 9; sec[25] = 0; sec[26] = 2; sec[27] = 0;
    if (dos2) {
        memcpy(sec + 0x1E, bootcode_dos2, sizeof bootcode_dos2);
        uint32_t id = esp_random(); memcpy(sec + 0x27, &id, 4);       // volume id (disk-change detection)
    } else memcpy(sec + 0x1E, bootcode_dos1, sizeof bootcode_dos1);
    sec[510] = 0x55; sec[511] = 0xAA;
    bool ok = fwrite(sec, 1, 512, f) == 512;
    for (int s = 1; s < 1440 && ok; s++) {
        memset(sec, (s >= 14) ? 0xE5 : 0x00, 512);                 // data area filled like a freshly formatted disk
        if (s == 1 || s == 4) { sec[0] = 0xF9; sec[1] = 0xFF; sec[2] = 0xFF; }   // FAT1 at sector 1, FAT2 at sector 4 (3 sectors each)
        if (s >= 1 && s < 7 && !(s == 1 || s == 4)) memset(sec, 0, 512);
        if (s >= 7 && s < 14) memset(sec, 0, 512);                 // root directory
        ok = fwrite(sec, 1, 512, f) == 512;
    }
    fclose(f);
    if (!ok) { unlink(full); return text(r, "500 Internal Server Error", "err write failed\n"); }
    return text(r, NULL, dos2 ? "ok 737280 bytes (DOS2)\n" : "ok 737280 bytes (DOS1)\n");
}
