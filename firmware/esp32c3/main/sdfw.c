// Firmware update from the microSD card: a vz80-*.vzp package in the card's root that carries a newer
// version than the running firmware is installed the same way as POST /api/fw (RP2350 to slot B as a
// trial, then the ESP32 OTA slot; the usual trial/rollback rules apply). A package is tried once: its
// label, size and checksums are remembered in NVS, so a failed or downgraded file is not retried until it changes.
#include "sdfw.h"
#include "sd.h"
#include "web.h"
#include "link.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <dirent.h>
#include <sys/stat.h>
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_app_desc.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
static const char *TAG = "sdfw";
#define MOUNT "/sdcard"
static char last_result[200] = "";
const char *sdfw_status(void) { return last_result; }
static bool parse_ver(const char *s, int v[3]) { v[0] = v[1] = v[2] = 0; return sscanf(s, "%d.%d.%d", &v[0], &v[1], &v[2]) >= 2; }
static int cmp_ver(const int a[3], const int b[3]) { for (int i = 0; i < 3; i++) if (a[i] != b[i]) return a[i] < b[i] ? -1 : 1; return 0; }
static int file_rd(void *ctx, uint8_t *buf, size_t n) { size_t k = fread(buf, 1, n, (FILE *)ctx); return k ? (int)k : -1; }
static void led(const char *cmd) { char out[32]; link_command(cmd, out, sizeof out, 1000); }
// The newest valid package in the root, newer than the running firmware. Returns false if none.
static bool find_package(char *name, size_t nn, vzp_hdr_t *hout, uint32_t *size_out) {
    DIR *d = opendir(MOUNT); if (!d) return false;
    int run[3]; parse_ver(esp_app_get_description()->version, run);
    int best[3] = { -1, -1, -1 }; bool found = false; struct dirent *e;
    while ((e = readdir(d))) {
        const char *n = e->d_name; size_t l = strlen(n);
        if (strncasecmp(n, "vz80-", 5) || l < 9 || strcasecmp(n + l - 4, ".vzp")) continue;
        char full[280]; snprintf(full, sizeof full, MOUNT "/%s", n);
        struct stat sb; if (stat(full, &sb) != 0 || !S_ISREG(sb.st_mode) || sb.st_size < 64) continue;
        FILE *f = fopen(full, "rb"); if (!f) continue;
        uint8_t hb[64]; size_t k = fread(hb, 1, 64, f); fclose(f);
        vzp_hdr_t h; char st[64];
        if (k != 64 || !fw_check_header(hb, &h, (uint32_t)sb.st_size, st, sizeof st)) { ESP_LOGW(TAG, "%s: %s", n, k != 64 ? "short" : st); continue; }
        char label[25]; memcpy(label, h.label, 24); label[24] = 0;
        int v[3]; if (!parse_ver(label, v)) continue;
        if (cmp_ver(v, run) <= 0 || cmp_ver(v, best) <= 0) continue;
        memcpy(best, v, sizeof best); snprintf(name, nn, "%s", n); *hout = h; *size_out = (uint32_t)sb.st_size; found = true;
    }
    closedir(d); return found;
}
static void try_update(void) {
    char name[80]; vzp_hdr_t h; uint32_t size;
    if (!find_package(name, sizeof name, &h, &size)) return;
    char label[25]; memcpy(label, h.label, 24); label[24] = 0;
    char key[64]; snprintf(key, sizeof key, "%s:%lu:%08lx%08lx", label, (unsigned long)size, (unsigned long)h.rp_sum, (unsigned long)h.esp_sum);   // label + size + both checksums: two builds with the same label are told apart
    nvs_handle_t nv; char last[64] = ""; size_t ll = sizeof last;
    if (nvs_open("vz80", NVS_READWRITE, &nv) == ESP_OK) { if (nvs_get_str(nv, "sdfw", last, &ll) != ESP_OK) last[0] = 0; }
    else { ESP_LOGW(TAG, "nvs"); return; }
    if (!strcmp(last, key)) { nvs_close(nv); snprintf(last_result, sizeof last_result, "%s: already tried", name); return; }
    nvs_set_str(nv, "sdfw", key); nvs_commit(nv); nvs_close(nv);   // remembered before the attempt: never loop on a bad package
    ESP_LOGI(TAG, "installing %s (%s, %lu bytes)", name, label, (unsigned long)size);
    char full[280]; snprintf(full, sizeof full, MOUNT "/%s", name);
    FILE *f = fopen(full, "rb"); if (!f) { snprintf(last_result, sizeof last_result, "%s: cannot open", name); return; }
    fseek(f, 64, SEEK_SET);
    led("led 255 0 0");                                              // solid red while the package goes in
    char st[200]; int rc = fw_install(&h, file_rd, f, st, sizeof st);
    fclose(f);
    snprintf(last_result, sizeof last_result, "%s: %.70s", name, st);
    ESP_LOGI(TAG, "%s", last_result);
    if (rc == 0) { if (h.esp_len) { vTaskDelay(pdMS_TO_TICKS(400)); esp_restart(); } led("led auto"); }
    else led("led auto");
}
static void task(void *arg) {
    uint32_t done_gen = 0; int64_t last_scan = 0;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(2000));
        if (!sd_mounted() || !link_healthy()) continue;
        int64_t now = esp_timer_get_time();
        bool remount = sd_mount_gen() != done_gen;
        if (!remount && now - last_scan < 30000000) continue;   // a new mount right away, otherwise every 30 s (a file copied in over Wi-Fi)
        done_gen = sd_mount_gen(); last_scan = now;
        try_update();
    }
}
void sdfw_start(void) { xTaskCreate(task, "sdfw", 6144, NULL, 3, NULL); }
