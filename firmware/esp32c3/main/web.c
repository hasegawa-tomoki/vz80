// HTTP API: POST /api/cmd (body = console line) -> text; GET /api/esp -> status; Wi-Fi settings come from /vz80.cfg on the card.
#include <string.h>
#include <stdio.h>
#include "web.h"
#include "link.h"
#include "wifi.h"
#include "sd.h"
#include "sdfw.h"
#include "cache.h"
#include "files.h"
#include "discovery.h"
#include "esp_http_server.h"
#include "esp_system.h"
#include "esp_ota_ops.h"
static const char *reset_reason_str(void) { switch (esp_reset_reason()) { case ESP_RST_POWERON: return "poweron"; case ESP_RST_SW: return "sw"; case ESP_RST_PANIC: return "panic"; case ESP_RST_INT_WDT: return "int_wdt"; case ESP_RST_TASK_WDT: return "task_wdt"; case ESP_RST_WDT: return "wdt"; case ESP_RST_BROWNOUT: return "brownout"; case ESP_RST_EXT: return "ext"; default: return "other"; } }
#include "esp_app_format.h"
#include "esp_app_desc.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

extern const char page_start[] asm("_binary_index_html_gz_start");   // gzip-compressed at build time (main/CMakeLists.txt)
extern const char page_end[] asm("_binary_index_html_gz_end");

static int read_body(httpd_req_t *r, char *buf, size_t n);
static esp_err_t h_root(httpd_req_t *r) { httpd_resp_set_type(r, "text/html; charset=utf-8"); httpd_resp_set_hdr(r, "Content-Encoding", "gzip"); return httpd_resp_send(r, page_start, page_end - page_start); }
static char *jstr(char *o, char *end, const char *s) {   // JSON-escape into o
    for (; *s && o < end - 2; s++) {
        if ((unsigned char)*s < 32) continue;
        if (*s == '"' || *s == '\\') *o++ = '\\';
        *o++ = *s;
    }
    *o = 0; return o;
}
// GET /api/status: discovery JSON (lan-finder / 近くの機器 compatible).
static char fw_note[96] = "idle";   // last package-install step, for /api/status post-mortems
static const char *volatile cur_uri = "-"; static volatile uint32_t h_entered, h_done; static int64_t h_enter_us;   // which handler the httpd task is in
static char bigbuf[12288];   // shared reply buffer: handlers run one at a time on the httpd task
#define BIGBUF (sizeof bigbuf)
static esp_err_t wrap(httpd_req_t *r) {
    cur_uri = r->uri; h_enter_us = esp_timer_get_time(); h_entered++;
    esp_err_t e = ((esp_err_t (*)(httpd_req_t *))r->user_ctx)(r);
    h_done++; return e;
}
void web_touch(void) { h_enter_us = esp_timer_get_time(); }
bool web_stalled(int64_t now_us) { return h_entered != h_done && now_us - h_enter_us > 90000000; }
void web_diag(char *buf, size_t n) {
    snprintf(buf, n, "http=%s/%lu/%lu fw=%s heap=%lu", cur_uri, (unsigned long)h_entered, (unsigned long)h_done, fw_note, (unsigned long)esp_get_free_heap_size());
}
// ---- ESP log capture: the last 4 KB of ESP_LOG output, GET /api/log (Wi-Fi-only debugging) --------
#include "esp_log.h"
#include <stdarg.h>
static char logring[2048]; static volatile size_t logw; static vprintf_like_t log_prev;
static int log_capture(const char *fmt, va_list ap) {
    char tmp[200]; va_list ap2; va_copy(ap2, ap);
    int n = vsnprintf(tmp, sizeof tmp, fmt, ap2); va_end(ap2);
    if (n > (int)sizeof tmp - 1) n = sizeof tmp - 1;
    for (int i = 0; i < n; i++) { logring[logw % sizeof logring] = tmp[i]; logw++; }
    return log_prev ? log_prev(fmt, ap) : n;
}
static esp_err_t h_log(httpd_req_t *r) {
    httpd_resp_set_type(r, "text/plain; charset=utf-8");
    size_t w = logw; size_t n = w < sizeof logring ? w : sizeof logring; size_t start = w - n;
    for (size_t i = 0; i < n; i += 512) { size_t k = n - i < 512 ? n - i : 512; char buf[512]; for (size_t j = 0; j < k; j++) buf[j] = logring[(start + i + j) % sizeof logring]; httpd_resp_send_chunk(r, buf, k); }
    return httpd_resp_send_chunk(r, NULL, 0);
}
static esp_err_t h_status(httpd_req_t *r) {
    static char out[1024]; char *o = out, *end = out + sizeof out;
    uint32_t ip = wifi_ip4(); const uint8_t *b = (const uint8_t *)&ip;
    o += snprintf(o, end - o, "{\"device\":\"vz80\",\"version\":\"%s\",\"wifi\":\"%s\",\"mac\":\"%s\",\"ip\":\"%u.%u.%u.%u\",\"txpower\":%d,\"ssid\":\"",
                  VZ80_VERSION, wifi_state_str(), wifi_mac_str(), b[0], b[1], b[2], b[3], wifi_tx_power());
    o = jstr(o, end, wifi_ssid());
    o += snprintf(o, end - o, "\",\"name\":\"");
    o = jstr(o, end, wifi_name());
    uint32_t lrx, lok, lneg; int lstage; link_stats(&lrx, &lok, &lneg, &lstage);
    esp_ota_img_states_t ost = ESP_OTA_IMG_VALID; esp_ota_get_state_partition(esp_ota_get_running_partition(), &ost);
    int pending = ost == ESP_OTA_IMG_PENDING_VERIFY || ost == ESP_OTA_IMG_NEW;
    TaskHandle_t mt = xTaskGetHandle("main"); int mstate = mt ? (int)eTaskGetState(mt) : -1;
    o += snprintf(o, end - o, "\",\"sd\":{\"present\":%d,\"mounted\":%d,\"size_mb\":%llu,\"cfg\":%d,\"cfg_file\":\"%s\"},\"uptime\":%lld,\"esp\":\"%s\",\"link\":{\"mode\":\"%s\",\"baud\":%lu,\"spi_frames\":%lu,\"spi_bad\":%lu,\"rx\":%lu,\"ok_ago_ms\":%lu,\"negs\":%lu,\"resyncs\":%lu,\"stage\":%d,\"main\":%d},\"fw\":\"%s\",\"sdfw\":\"%s\",\"esp_pending\":%d,\"heap\":%lu,\"heap_min\":%lu,\"reset\":\"%s\"}", sd_present(), sd_mounted(), (unsigned long long)(sd_size_bytes() >> 20), sd_cfg(NULL, NULL, NULL), sd_cfg_file(), (long long)(esp_timer_get_time() / 1000000), esp_app_get_description()->version, link_mode() == LINK_MODE_SPI ? "spi" : "uart", (unsigned long)link_baud(), (unsigned long)link_spi_stats()->frames, (unsigned long)(link_spi_stats()->bad_hdr + link_spi_stats()->bad_pay), (unsigned long)lrx, (unsigned long)lok, (unsigned long)lneg, (unsigned long)link_resyncs(), lstage, mstate, fw_note, sdfw_status(), pending, (unsigned long)esp_get_free_heap_size(), (unsigned long)esp_get_minimum_free_heap_size(), reset_reason_str());
    if (o > end - 1) o = end - 1;
    httpd_resp_set_type(r, "application/json");
    httpd_resp_set_hdr(r, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(r, "Cache-Control", "no-store");
    return httpd_resp_send(r, out, o - out);
}
// GET /api/peers: beacon table.
static esp_err_t h_peers(httpd_req_t *r) {
    static char out[1024]; char *o = out, *end = out + sizeof out - 8;
    o += snprintf(o, end - o, "{\"ok\":true,\"peers\":[");
    bool first = true;
    for (int i = 0; i < DISC_MAX_PEERS; i++) {
        const disc_peer_t *p = disc_peer(i); if (!p) continue;
        const uint8_t *b = (const uint8_t *)&p->ip;
        if (!first) { *o++ = ','; }
        first = false;
        o += snprintf(o, end - o, "{\"device\":\"%s\",\"version\":\"%s\",\"ip\":\"%u.%u.%u.%u\",\"port\":%u,\"name\":\"", p->device, p->version, b[0], b[1], b[2], b[3], p->port);
        o = jstr(o, end, p->name); o += snprintf(o, end - o, "\"}");
    }
    o += snprintf(o, end - o, "]}");
    httpd_resp_set_type(r, "application/json");
    httpd_resp_set_hdr(r, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(r, out, o - out);
}
// POST /api/name: body = display name
static esp_err_t h_name(httpd_req_t *r) {
    char body[64]; if (read_body(r, body, sizeof body) < 0) return ESP_FAIL;
    char *nl = strpbrk(body, "\r\n"); if (nl) *nl = 0;
    wifi_set_name(body);
    httpd_resp_set_type(r, "text/plain"); return httpd_resp_send(r, "ok\n", HTTPD_RESP_USE_STRLEN);
}
static esp_err_t h_esp(httpd_req_t *r) { char s[240]; wifi_status(s, sizeof s); httpd_resp_set_type(r, "text/plain"); return httpd_resp_send(r, s, HTTPD_RESP_USE_STRLEN); }

static int read_body(httpd_req_t *r, char *buf, size_t n) {
    size_t len = r->content_len < n - 1 ? r->content_len : n - 1; size_t got = 0;
    while (got < len) { int k = httpd_req_recv(r, buf + got, len - got); if (k <= 0) return -1; got += k; }
    buf[got] = 0; return (int)got;
}

static esp_err_t h_cmd(httpd_req_t *r) {
    char cmd[256]; char *out = bigbuf;
    if (read_body(r, cmd, sizeof cmd) < 0) return ESP_FAIL;
    char *nl = strpbrk(cmd, "\r\n"); if (nl) *nl = 0;
    int n = link_command(cmd, out, BIGBUF, link_healthy() ? 10000 : 3000);   // fail fast while the RP is down
    httpd_resp_set_type(r, "text/plain");
    if (n < 0) { httpd_resp_set_status(r, "504 Gateway Timeout"); return httpd_resp_send(r, "err rp2350 timeout\n", HTTPD_RESP_USE_STRLEN); }
    return httpd_resp_send(r, out, n);
}

// GET /api/sd?path=/dir : directory listing of the microSD card as JSON.
static esp_err_t h_sd(httpd_req_t *r) {
    char *out = bigbuf; char q[700] = "", path[260] = "/";
    if (httpd_req_get_url_query_str(r, q, sizeof q) == ESP_OK) httpd_query_key_value(q, "path", path, sizeof path);
    url_decode(path);
    int n = sd_list_json(path, out, BIGBUF);
    httpd_resp_set_type(r, "application/json"); httpd_resp_set_hdr(r, "Cache-Control", "no-store");
    return httpd_resp_send(r, out, n > 0 ? n : 0);
}
static esp_err_t h_cache(httpd_req_t *r) {     // POST /api/cache?path=  |  /api/uncache?path=
    char q[700] = "", path[224] = "", st[200]; bool un = strstr(r->uri, "uncache") != NULL;
    if (httpd_req_get_url_query_str(r, q, sizeof q) == ESP_OK) httpd_query_key_value(q, "path", path, sizeof path);
    url_decode(path);
    bool ok = un ? cache_uncache(path, st, sizeof st) : cache_import(path, st, sizeof st);
    httpd_resp_set_type(r, "text/plain"); if (!ok) httpd_resp_set_status(r, "409 Conflict");
    strlcat(st, "\n", sizeof st); return httpd_resp_send(r, st, HTTPD_RESP_USE_STRLEN);
}
static esp_err_t h_mount(httpd_req_t *r) {     // POST /api/mount?path=&target=A|B|1|2  |  /api/eject?target=
    char q[700] = "", path[224] = "", target[8] = "", type[8] = "", st[200]; bool ej = strstr(r->uri, "eject") != NULL;
    if (httpd_req_get_url_query_str(r, q, sizeof q) == ESP_OK) { httpd_query_key_value(q, "path", path, sizeof path); httpd_query_key_value(q, "target", target, sizeof target); httpd_query_key_value(q, "type", type, sizeof type); }
    url_decode(path);
    bool ok = ej ? cache_eject(target, st, sizeof st) : cache_mount(path, target, type[0] ? atoi(type) : -1, st, sizeof st);
    httpd_resp_set_type(r, "text/plain"); if (!ok) httpd_resp_set_status(r, "409 Conflict");
    strlcat(st, "\n", sizeof st); return httpd_resp_send(r, st, HTTPD_RESP_USE_STRLEN);
}
static esp_err_t h_hist(httpd_req_t *r) {      // GET /api/hist
    static char out[2400]; cache_hist_json(out, sizeof out);
    httpd_resp_set_type(r, "application/json"); httpd_resp_set_hdr(r, "Cache-Control", "no-store"); return httpd_resp_send(r, out, HTTPD_RESP_USE_STRLEN);
}
static esp_err_t h_vd(httpd_req_t *r) {        // POST /api/vd?op=add&arg=fdd|hdd | op=del&arg=N | op=order&arg=1,2,3
    char q[96] = "", op[8] = "", arg[32] = "", st[200];
    if (httpd_req_get_url_query_str(r, q, sizeof q) == ESP_OK) { httpd_query_key_value(q, "op", op, sizeof op); httpd_query_key_value(q, "arg", arg, sizeof arg); }
    url_decode(arg);
    bool ok = cache_vd(op, arg, st, sizeof st);
    httpd_resp_set_type(r, "text/plain"); if (!ok) httpd_resp_set_status(r, "409 Conflict");
    strlcat(st, "\n", sizeof st); return httpd_resp_send(r, st, HTTPD_RESP_USE_STRLEN);
}
static esp_err_t h_fdswap(httpd_req_t *r) {    // POST /api/fdswap?a=A&b=V1
    char q[64] = "", a[8] = "", b[8] = "", st[120];
    if (httpd_req_get_url_query_str(r, q, sizeof q) == ESP_OK) { httpd_query_key_value(q, "a", a, sizeof a); httpd_query_key_value(q, "b", b, sizeof b); }
    bool ok = cache_fdswap(a, b, st, sizeof st);
    httpd_resp_set_type(r, "text/plain"); if (!ok) httpd_resp_set_status(r, "409 Conflict");
    strlcat(st, "\n", sizeof st); return httpd_resp_send(r, st, HTTPD_RESP_USE_STRLEN);
}
static esp_err_t h_rpfw(httpd_req_t *r) {
    static uint8_t buf[4096]; char status[200];
    uint32_t total = r->content_len;
    httpd_resp_set_type(r, "text/plain");
    if (total == 0 || total % 512) return httpd_resp_send(r, "err body must be a UF2\n", HTTPD_RESP_USE_STRLEN);
    if (link_fwup_begin(total, false, status, sizeof status)) { httpd_resp_set_status(r, "502 Bad Gateway"); return httpd_resp_send(r, status, HTTPD_RESP_USE_STRLEN); }
    uint32_t got = 0;
    while (got < total) {
        size_t want = total - got < sizeof buf ? total - got : sizeof buf; size_t have = 0;
        while (have < want) { int k = httpd_req_recv(r, (char *)buf + have, want - have); if (k <= 0) { link_fwup_end(1000); return ESP_FAIL; } have += k; }
        if (link_fwup_piece(buf, have, 5000)) { link_fwup_end(1000); httpd_resp_set_status(r, "502 Bad Gateway"); return httpd_resp_send(r, status, HTTPD_RESP_USE_STRLEN); }
        got += have;
    }
    int rc = link_fwup_end(30000);
    if (rc) httpd_resp_set_status(r, "502 Bad Gateway");
    char out[240]; snprintf(out, sizeof out, "%s %lu bytes: %s\n", rc ? "FAILED" : "ok", (unsigned long)got, status);
    return httpd_resp_send(r, out, HTTPD_RESP_USE_STRLEN);
}

// POST /api/esp-fw: body = ESP32 app image (vz80-esp.bin); written to the other OTA slot.
static esp_err_t h_espfw(httpd_req_t *r) {
    static uint8_t buf[4096];
    httpd_resp_set_type(r, "text/plain");
    const esp_partition_t *part = esp_ota_get_next_update_partition(NULL);
    if (!part) return httpd_resp_send(r, "err no ota partition\n", HTTPD_RESP_USE_STRLEN);
    esp_ota_handle_t h;
    if (esp_ota_begin(part, r->content_len, &h) != ESP_OK) return httpd_resp_send(r, "err ota begin\n", HTTPD_RESP_USE_STRLEN);
    uint32_t got = 0;
    while (got < r->content_len) {
        int k = httpd_req_recv(r, (char *)buf, sizeof buf); if (k <= 0) { esp_ota_abort(h); return ESP_FAIL; }
        if (esp_ota_write(h, buf, k) != ESP_OK) { esp_ota_abort(h); return httpd_resp_send(r, "err ota write\n", HTTPD_RESP_USE_STRLEN); }
        got += k;
    }
    if (esp_ota_end(h) != ESP_OK) return httpd_resp_send(r, "err ota end (bad image?)\n", HTTPD_RESP_USE_STRLEN);
    if (esp_ota_set_boot_partition(part) != ESP_OK) return httpd_resp_send(r, "err set boot\n", HTTPD_RESP_USE_STRLEN);
    char out[120]; snprintf(out, sizeof out, "ok %lu bytes to %s, restarting\n", (unsigned long)got, part->label);
    httpd_resp_send(r, out, HTTPD_RESP_USE_STRLEN);
    vTaskDelay(pdMS_TO_TICKS(300)); esp_restart(); return ESP_OK;
}

// GET /api/vram?full=1 : binary VRAM delta from the RP (see console vramdelta), streamed.
static esp_err_t h_vram(httpd_req_t *r) {
    char q[32]; bool full = httpd_req_get_url_query_str(r, q, sizeof q) == ESP_OK && strstr(q, "full");
    uint32_t total;
    StreamBufferHandle_t sb = link_binary_begin(full ? "vramdelta full" : "vramdelta", &total, 3000);
    httpd_resp_set_type(r, "application/octet-stream");
    httpd_resp_set_hdr(r, "Cache-Control", "no-store");
    if (!sb) { httpd_resp_set_status(r, "504 Gateway Timeout"); return httpd_resp_send(r, "", 0); }
    static uint8_t buf[2048]; uint32_t left = total; bool ok = true;
    while (left) {
        size_t k = xStreamBufferReceive(sb, buf, left < sizeof buf ? left : sizeof buf, pdMS_TO_TICKS(3000));
        if (!k) { ok = false; break; }
        if (httpd_resp_send_chunk(r, (char *)buf, k) != ESP_OK) { ok = false; break; }
        left -= k;
    }
    link_binary_end(ok ? 2000 : 200);
    return httpd_resp_send_chunk(r, NULL, 0);
}
// GET /api/vdp : VDP registers/palette JSON (console vdpstate)
static esp_err_t h_vdp(httpd_req_t *r) {
    static char out[1024]; int n = link_command("vdpstate", out, sizeof out, 3000);
    httpd_resp_set_type(r, "application/json"); httpd_resp_set_hdr(r, "Cache-Control", "no-store");
    if (n < 0) { httpd_resp_set_status(r, "504 Gateway Timeout"); return httpd_resp_send(r, "{}", 2); }
    return httpd_resp_send(r, out, n);
}
// GET /api/config?name=... : set the display name (vmpu68-compatible), returns {"ok":true,"name":...}
static esp_err_t h_config(httpd_req_t *r) {
    char q[128], v[64], out[128];
    if (httpd_req_get_url_query_str(r, q, sizeof q) == ESP_OK && httpd_query_key_value(q, "name", v, sizeof v) == ESP_OK) {
        // %xx-decode
        char d[64]; size_t j = 0;
        for (size_t i = 0; v[i] && j < sizeof d - 1; i++) { if (v[i] == '%' && v[i+1] && v[i+2]) { char h[3] = { v[i+1], v[i+2], 0 }; d[j++] = (char)strtol(h, NULL, 16); i += 2; } else d[j++] = v[i] == '+' ? ' ' : v[i]; }
        d[j] = 0; wifi_set_name(d);
    }
    char *o = out; o += snprintf(o, sizeof out, "{\"ok\":true,\"name\":\""); o = jstr(o, out + sizeof out - 4, wifi_name()); o += snprintf(o, out + sizeof out - o, "\"}");
    httpd_resp_set_type(r, "application/json"); httpd_resp_set_hdr(r, "Cache-Control", "no-store");
    return httpd_resp_send(r, out, o - out);
}
// POST /api/fw : vz80 package (.vzp): 64-byte header "VZP1", then the RP2350 UF2 and the ESP32 app.
static uint32_t vsum(uint32_t s, const uint8_t *p, size_t n) { for (size_t i = 0; i < n; i++) s = s * 31 + p[i]; return s; }
#define FWNOTE(...) snprintf(fw_note, sizeof fw_note, __VA_ARGS__)
// POST /api/promote : make the trial firmware permanent (RP2350: copy slot B over A; ESP32: cancel rollback).
static esp_err_t h_promote(httpd_req_t *r) {
    char out[160] = "";
    int n = link_command("promote", out, sizeof out, 30000);
    httpd_resp_set_type(r, "text/plain");
    if (n < 0) return httpd_resp_send(r, "err rp2350 timeout\n", HTTPD_RESP_USE_STRLEN);
    if (strncmp(out, "ok", 2) && strstr(out, "not running the trial") == NULL) return httpd_resp_send(r, out, n);
    esp_ota_mark_app_valid_cancel_rollback();
    char msg[220]; snprintf(msg, sizeof msg, "ok promoted (rp: %.120s)\n", out);
    return httpd_resp_send(r, msg, HTTPD_RESP_USE_STRLEN);
}
// POST /api/rollback : leave the trial: RP2350 reboots into slot A, ESP32 returns to its previous app.
// POST /api/msxreset : board 2.0 only: pulse the MSX /RESET line through Q1 (the RP2350 sees the reset and restarts the Z80).
static esp_err_t h_msxreset(httpd_req_t *r) {
    link_msx_reset();
    httpd_resp_set_type(r, "text/plain"); return httpd_resp_sendstr(r, "ok");
}
// GET /api/txpower -> current dBm; POST /api/txpower?dbm=N (2..20): Wi-Fi transmit power. Lower it when the MSX's 5 V sags on
// transmit peaks (the display dropped out on every burst in a camper van with -84 dBm RSSI, 2026-10-01).
static esp_err_t h_txpower(httpd_req_t *r) {
    char q[32], v[8];
    if (r->method == HTTP_POST) {
        if (httpd_req_get_url_query_str(r, q, sizeof q) != ESP_OK || httpd_query_key_value(q, "dbm", v, sizeof v) != ESP_OK || !wifi_set_tx_power(atoi(v))) { httpd_resp_set_status(r, "400 Bad Request"); httpd_resp_set_type(r, "text/plain"); return httpd_resp_sendstr(r, "err dbm 2..20"); }
    }
    char out[32]; snprintf(out, sizeof out, "%d", wifi_tx_power());
    httpd_resp_set_type(r, "text/plain"); return httpd_resp_sendstr(r, out);
}
static esp_err_t h_rollback(httpd_req_t *r) {
    char out[64];
    link_command("reboot", out, sizeof out, 1500);
    httpd_resp_set_type(r, "text/plain");
    httpd_resp_send(r, "ok rolling back\n", HTTPD_RESP_USE_STRLEN);
    vTaskDelay(pdMS_TO_TICKS(300));
    if (esp_ota_check_rollback_is_possible()) esp_ota_mark_app_invalid_rollback_and_reboot();
    return ESP_OK;
}
// Package install shared by POST /api/fw and the SD-card updater (sdfw.c): the header was read and
// checked by fw_check_header(); rd() supplies the rest of the package in order (RP2350 UF2, then ESP app).
bool fw_check_header(const uint8_t *hb, vzp_hdr_t *h, uint32_t total_len, char *status, size_t n) {
    memcpy(h, hb, 64);
    if (memcmp(h->magic, "VZP1", 4) || h->hdr_size != 64 || h->hdr_sum != vsum(0, (uint8_t *)h, 60)) { snprintf(status, n, "err not a vz80 package"); return false; }
    if (h->rp_off != 64 || h->esp_off != 64 + h->rp_len || total_len != 64 + h->rp_len + h->esp_len) { snprintf(status, n, "err package layout"); return false; }
    return true;
}
int fw_install(const vzp_hdr_t *h, fw_read_fn rd, void *ctx, char *status, size_t n) {
    static uint8_t buf[4096]; char st[200];
    FWNOTE("start rp=%lu esp=%lu", (unsigned long)h->rp_len, (unsigned long)h->esp_len);
    uint32_t sum = 0;
    if (h->rp_len) {   // 1. RP2350 part through the link (slot B, trial boot)
        if (link_fwup_begin(h->rp_len, true, st, sizeof st)) { FWNOTE("rp begin failed: %.60s", st); snprintf(status, n, "%s", st); return -1; }
        FWNOTE("rp begin ok");
        for (uint32_t got = 0; got < h->rp_len; ) {
            size_t want = h->rp_len - got < sizeof buf ? h->rp_len - got : sizeof buf, have = 0;
            while (have < want) { int k = rd(ctx, buf + have, want - have); if (k <= 0) { FWNOTE("rp read failed at %lu (%d)", (unsigned long)got, k); link_fwup_end(1000); snprintf(status, n, "err read"); return -2; } have += k; }
            sum = vsum(sum, buf, have);
            if (link_fwup_piece(buf, have, 5000)) { FWNOTE("rp piece failed at %lu: %.50s", (unsigned long)got, st); link_fwup_end(1000); snprintf(status, n, "%s", st); return -1; }
            got += have; FWNOTE("rp %lu/%lu", (unsigned long)got, (unsigned long)h->rp_len);
            web_touch();
        }
        if (link_fwup_end(30000)) { FWNOTE("rp end failed: %.60s", st); snprintf(status, n, "%s", st); return -1; }
        FWNOTE("rp done");
        if (sum != h->rp_sum) { snprintf(status, n, "err rp checksum (RP2350 already rebooted with the new image)"); return -3; }
    }
    if (h->esp_len) {   // 2. ESP32 part into the other OTA slot
        const esp_partition_t *part = esp_ota_get_next_update_partition(NULL); esp_ota_handle_t oh;
        if (!part || esp_ota_begin(part, h->esp_len, &oh) != ESP_OK) { snprintf(status, n, "err ota begin"); return -4; }
        sum = 0;
        for (uint32_t got = 0; got < h->esp_len; ) {
            int k = rd(ctx, buf, h->esp_len - got < sizeof buf ? h->esp_len - got : sizeof buf); if (k <= 0) { FWNOTE("esp read failed at %lu (%d)", (unsigned long)got, k); esp_ota_abort(oh); snprintf(status, n, "err read"); return -2; }
            sum = vsum(sum, buf, k);
            if (esp_ota_write(oh, buf, k) != ESP_OK) { esp_ota_abort(oh); snprintf(status, n, "err ota write"); return -4; }
            got += k; web_touch();
        }
        if (sum != h->esp_sum) { esp_ota_abort(oh); snprintf(status, n, "err esp checksum"); return -3; }
        if (esp_ota_end(oh) != ESP_OK || esp_ota_set_boot_partition(part) != ESP_OK) { snprintf(status, n, "err ota end"); return -4; }
    }
    FWNOTE("ok");
    snprintf(status, n, "ok %.24s installed (rp %lu bytes, esp %lu bytes)", h->label, (unsigned long)h->rp_len, (unsigned long)h->esp_len);
    return 0;
}
static int http_rd(void *ctx, uint8_t *buf, size_t n) { return httpd_req_recv((httpd_req_t *)ctx, (char *)buf, n); }
static esp_err_t h_fw(httpd_req_t *r) {
    char status[200]; vzp_hdr_t h; char hb[66];
    FWNOTE("start len=%u", (unsigned)r->content_len);
    httpd_resp_set_type(r, "text/plain");
    if (read_body(r, hb, 65) != 64) return httpd_resp_send(r, "err short header\n", HTTPD_RESP_USE_STRLEN);
    if (!fw_check_header((uint8_t *)hb, &h, r->content_len, status, sizeof status)) { strlcat(status, "\n", sizeof status); return httpd_resp_send(r, status, HTTPD_RESP_USE_STRLEN); }
    int rc = fw_install(&h, http_rd, r, status, sizeof status);
    if (rc == -2) return ESP_FAIL;
    if (rc == -1) httpd_resp_set_status(r, "502 Bad Gateway");
    strlcat(status, rc == 0 ? ", restarting ESP32\n" : "\n", sizeof status);
    httpd_resp_send(r, status, HTTPD_RESP_USE_STRLEN);
    if (rc == 0 && h.esp_len) { vTaskDelay(pdMS_TO_TICKS(400)); esp_restart(); }
    return ESP_OK;
}

void web_start(void) {
    log_prev = esp_log_set_vprintf(log_capture);
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG(); cfg.max_uri_handlers = 40; cfg.stack_size = 10240; cfg.lru_purge_enable = true; cfg.max_open_sockets = 8; cfg.backlog_conn = 8; cfg.recv_wait_timeout = 30; cfg.send_wait_timeout = 30;
    httpd_handle_t h;
    if (httpd_start(&h, &cfg) != ESP_OK) return;
    httpd_uri_t u[] = {
        { .uri = "/",         .method = HTTP_GET,  .handler = wrap, .user_ctx = h_root },
        { .uri = "/api/esp",  .method = HTTP_GET,  .handler = wrap, .user_ctx = h_esp },
        { .uri = "/api/status", .method = HTTP_GET, .handler = wrap, .user_ctx = h_status },
        { .uri = "/api/peers", .method = HTTP_GET, .handler = wrap, .user_ctx = h_peers },
        { .uri = "/api/name", .method = HTTP_POST, .handler = wrap, .user_ctx = h_name },
        { .uri = "/api/cmd",  .method = HTTP_POST, .handler = wrap, .user_ctx = h_cmd },
        { .uri = "/api/promote", .method = HTTP_POST, .handler = wrap, .user_ctx = h_promote },
        { .uri = "/api/rollback", .method = HTTP_POST, .handler = wrap, .user_ctx = h_rollback },
        { .uri = "/api/msxreset", .method = HTTP_POST, .handler = wrap, .user_ctx = h_msxreset },
        { .uri = "/api/txpower", .method = HTTP_GET, .handler = wrap, .user_ctx = h_txpower },
        { .uri = "/api/txpower", .method = HTTP_POST, .handler = wrap, .user_ctx = h_txpower },
        { .uri = "/api/sd", .method = HTTP_GET, .handler = wrap, .user_ctx = h_sd },
        { .uri = "/api/log",    .method = HTTP_GET,  .handler = wrap, .user_ctx = h_log },
        { .uri = "/api/upload", .method = HTTP_POST, .handler = wrap, .user_ctx = h_file_put },
        { .uri = "/api/file",   .method = HTTP_GET,  .handler = wrap, .user_ctx = h_file_get },
        { .uri = "/api/file",   .method = HTTP_POST, .handler = wrap, .user_ctx = h_file_put },
        { .uri = "/api/mkdir",  .method = HTTP_POST, .handler = wrap, .user_ctx = h_mkdir },
        { .uri = "/api/mv",     .method = HTTP_POST, .handler = wrap, .user_ctx = h_mv },
        { .uri = "/api/rm",     .method = HTTP_POST, .handler = wrap, .user_ctx = h_rm },
        { .uri = "/api/mkimg",  .method = HTTP_POST, .handler = wrap, .user_ctx = h_mkimg },
        { .uri = "/api/cache",  .method = HTTP_POST, .handler = wrap, .user_ctx = h_cache },
        { .uri = "/api/uncache", .method = HTTP_POST, .handler = wrap, .user_ctx = h_cache },
        { .uri = "/api/mount",  .method = HTTP_POST, .handler = wrap, .user_ctx = h_mount },
        { .uri = "/api/hist",   .method = HTTP_GET,  .handler = wrap, .user_ctx = h_hist },
        { .uri = "/api/fdswap", .method = HTTP_POST, .handler = wrap, .user_ctx = h_fdswap },
        { .uri = "/api/vd",     .method = HTTP_POST, .handler = wrap, .user_ctx = h_vd },
        { .uri = "/api/imgresize", .method = HTTP_POST, .handler = wrap, .user_ctx = h_imgresize },
        { .uri = "/api/romhead", .method = HTTP_GET, .handler = wrap, .user_ctx = h_romhead },
        { .uri = "/api/eject",  .method = HTTP_POST, .handler = wrap, .user_ctx = h_mount },
        { .uri = "/api/rp-fw", .method = HTTP_POST, .handler = wrap, .user_ctx = h_rpfw },
        { .uri = "/api/esp-fw", .method = HTTP_POST, .handler = wrap, .user_ctx = h_espfw },
        { .uri = "/api/vram", .method = HTTP_GET, .handler = wrap, .user_ctx = h_vram },
        { .uri = "/api/vdp", .method = HTTP_GET, .handler = wrap, .user_ctx = h_vdp },
        { .uri = "/api/config", .method = HTTP_GET, .handler = wrap, .user_ctx = h_config },
        { .uri = "/api/fw", .method = HTTP_POST, .handler = wrap, .user_ctx = h_fw },
    };
    for (unsigned i = 0; i < sizeof u / sizeof u[0]; i++) httpd_register_uri_handler(h, &u[i]);
}
