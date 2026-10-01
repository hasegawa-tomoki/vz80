#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <dirent.h>
#include <sys/stat.h>
#include "sd.h"
#include "wifi.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define PIN_CS 4
#define PIN_CLK 5
#define PIN_MOSI 6
#define PIN_MISO 7
#define PIN_CD 10          // card-detect switch: low = card inserted (pull-up)
#define MOUNT "/sdcard"
#define CFG_PATH MOUNT "/vz80.cfg"
static const char *TAG = "sd";

static sdmmc_card_t *card; static bool mounted, bus_ready; static uint32_t mount_gen;
uint32_t sd_mount_gen(void) { return mount_gen; }
static char cfg_ssid[33], cfg_pass[65], cfg_name[32]; static int cfg_txpower; static const char *cfg_file = "";

bool sd_present(void) { return gpio_get_level(PIN_CD) == 0; }
bool sd_mounted(void) { return mounted; }
uint64_t sd_size_bytes(void) { return mounted ? (uint64_t)card->csd.capacity * card->csd.sector_size : 0; }

static void read_cfg(void) {
    cfg_ssid[0] = cfg_pass[0] = cfg_name[0] = 0; cfg_txpower = 0; cfg_file = "";
    FILE *f = fopen(CFG_PATH, "r");
    if (!f) return;
    cfg_file = "vz80.cfg";
    char line[160]; bool first = true;
    while (fgets(line, sizeof line, f)) {
        char *p = line;
        if (first && !memcmp(p, "\xEF\xBB\xBF", 3)) p += 3;   // UTF-8 BOM
        first = false;
        char *nl = strpbrk(p, "\r\n"); if (nl) *nl = 0;
        if (!strncmp(p, "ssid=", 5)) strncpy(cfg_ssid, p + 5, sizeof cfg_ssid - 1);
        else if (!strncmp(p, "pass=", 5)) strncpy(cfg_pass, p + 5, sizeof cfg_pass - 1);
        else if (!strncmp(p, "name=", 5)) strncpy(cfg_name, p + 5, sizeof cfg_name - 1);
        else if (!strncmp(p, "txpower=", 8)) cfg_txpower = atoi(p + 8);
    }
    fclose(f);
    if (cfg_txpower) wifi_set_tx_power(cfg_txpower);
    ESP_LOGI(TAG, "%s: ssid %s, name %s, txpower %d", cfg_file, cfg_ssid[0] ? "set" : "-", cfg_name[0] ? cfg_name : "-", cfg_txpower);
}

// The SPI2 bus is shared with the RP2350 link on board 2.0 (link.c mirrors SCK/MOSI to IO21/IO20 and
// switches MISO), so it is brought up at init even with no card in the slot.
static bool bus_init(void) {
    if (bus_ready) return true;
    spi_bus_config_t b = { .mosi_io_num = PIN_MOSI, .miso_io_num = PIN_MISO, .sclk_io_num = PIN_CLK, .quadwp_io_num = -1, .quadhd_io_num = -1, .max_transfer_sz = 4096 };
    if (spi_bus_initialize(SPI2_HOST, &b, SDSPI_DEFAULT_DMA) != ESP_OK) { ESP_LOGE(TAG, "spi bus"); return false; }
    bus_ready = true; return true;
}
static bool mount(void) {
    if (mounted) return true;
    if (!bus_init()) return false;
    sdmmc_host_t host = SDSPI_HOST_DEFAULT(); host.slot = SPI2_HOST; host.max_freq_khz = 20000;
    sdspi_device_config_t sc = SDSPI_DEVICE_CONFIG_DEFAULT(); sc.gpio_cs = PIN_CS; sc.host_id = SPI2_HOST;
    esp_vfs_fat_sdmmc_mount_config_t mc = { .format_if_mount_failed = false, .max_files = 4, .allocation_unit_size = 16 * 1024 };
    esp_err_t e = esp_vfs_fat_sdspi_mount(MOUNT, &host, &sc, &mc, &card);
    if (e != ESP_OK) { ESP_LOGW(TAG, "mount: %s", esp_err_to_name(e)); return false; }
    mounted = true; mount_gen++;
    ESP_LOGI(TAG, "mounted: %s %llu MB", card->cid.name, (unsigned long long)(sd_size_bytes() >> 20));
    read_cfg();
    return true;
}
static void unmount(void) {
    if (!mounted) return;
    esp_vfs_fat_sdcard_unmount(MOUNT, card); card = NULL; mounted = false;
    ESP_LOGI(TAG, "unmounted");
}

void sd_init(void) {
    gpio_config_t g = { .pin_bit_mask = 1ull << PIN_CD, .mode = GPIO_MODE_INPUT, .pull_up_en = GPIO_PULLUP_ENABLE };
    gpio_config(&g);
    bus_init();
    vTaskDelay(pdMS_TO_TICKS(20));
    if (sd_present()) mount();
}

const char *sd_cfg_file(void) { return mounted ? cfg_file : ""; }
// Write /vz80.cfg from the given settings (the Web UI's "save Wi-Fi settings to the card").
bool sd_write_cfg(const char *ssid, const char *pass, const char *name) {
    if (!mounted || !ssid || !ssid[0]) return false;
    FILE *f = fopen(CFG_PATH, "w");
    if (!f) return false;
    fprintf(f, "ssid=%s\npass=%s\n", ssid, pass ? pass : "");
    if (name && name[0]) fprintf(f, "name=%s\n", name);
    fclose(f);
    read_cfg();
    return cfg_ssid[0] != 0;
}
bool sd_cfg(const char **ssid, const char **pass, const char **name) {
    if (ssid) *ssid = cfg_ssid;
    if (pass) *pass = cfg_pass;
    if (name) *name = cfg_name;
    return mounted && cfg_ssid[0] != 0;
}

// Insertion / removal: poll the switch; a newly mounted card's /vz80.cfg is applied at once.
static void monitor(void *arg) {
    bool was = sd_present();
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        bool now = sd_present();
        if (now == was) continue;
        vTaskDelay(pdMS_TO_TICKS(500));                  // debounce / let the card power up
        if (sd_present() != now) continue;
        was = now;
        if (!now) { unmount(); continue; }
        if (mount() && cfg_ssid[0]) wifi_apply_cfg(cfg_ssid, cfg_pass, cfg_name);
    }
}
void sd_reload_cfg(void) {                      // /vz80.cfg was rewritten (SD browser): apply it now
    if (!mounted) return;
    read_cfg();
    if (cfg_ssid[0]) wifi_apply_cfg(cfg_ssid, cfg_pass, cfg_name);
}
void sd_start_monitor(void) { xTaskCreate(monitor, "sdmon", 4096, NULL, 3, NULL); }

static char *jesc(char *o, char *end, const char *s) {
    for (; *s && o < end - 2; s++) { if (*s == '"' || *s == '\\') *o++ = '\\'; if ((unsigned char)*s < 0x20) { *o++ = ' '; continue; } *o++ = *s; }
    return o;
}
int sd_list_json(const char *path, char *out, size_t n) {
    char *o = out, *end = out + n;
    if (!mounted) { return snprintf(out, n, "{\"ok\":false,\"err\":\"no card\"}"); }
    if (!path || !*path) path = "/";
    if (strstr(path, "..") || path[0] != '/') return snprintf(out, n, "{\"ok\":false,\"err\":\"bad path\"}");
    char full[300]; snprintf(full, sizeof full, MOUNT "%s", strcmp(path, "/") ? path : "");
    DIR *d = opendir(full);
    if (!d) return snprintf(out, n, "{\"ok\":false,\"err\":\"not found\"}");
    o += snprintf(o, end - o, "{\"ok\":true,\"path\":\""); o = jesc(o, end, path); o += snprintf(o, end - o, "\",\"entries\":[");
    struct dirent *e; int cnt = 0; bool trunc = false;
    while ((e = readdir(d))) {
        if (o > end - 96) { trunc = true; break; }
        char fp[560]; snprintf(fp, sizeof fp, "%.300s/%.255s", full, e->d_name);
        struct stat st; long long sz = 0; int isdir = e->d_type == DT_DIR;
        if (!isdir && stat(fp, &st) == 0) sz = st.st_size;
        o += snprintf(o, end - o, "%s{\"n\":\"", cnt ? "," : ""); o = jesc(o, end, e->d_name);
        o += snprintf(o, end - o, "\",\"d\":%d,\"s\":%lld}", isdir, sz); cnt++;
    }
    closedir(d);
    o += snprintf(o, end - o, "],\"truncated\":%s}", trunc ? "true" : "false");
    return o - out;
}
