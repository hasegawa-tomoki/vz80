// Wi-Fi manager: STA with stored credentials, SoftAP fallback (also started if STA has no IP after 20 s).
#include <string.h>
#include <stdio.h>
#include "wifi.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_timer.h"
#include "esp_mac.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "sd.h"

#define AP_PASS "vz80setup"

static char ssid[33], pass[65], ap_ssid[16], dispname[32], macstr[18];
static uint32_t ip_u32;
static bool have_creds, sta_ip, ap_up;
static char ip_str[16] = "0.0.0.0";
static int retries, last_reason;
static bool started; static int8_t txpower_dbm;   // 0 = chip default (20 dBm). Lower it when the MSX's 5 V cannot take the transmit peaks (display drop-outs)
static esp_timer_handle_t fallback_timer;

static void load_creds(void) {
    nvs_handle_t h;
    if (nvs_open("vz80", NVS_READONLY, &h) != ESP_OK) return;
    size_t n = sizeof ssid; if (nvs_get_str(h, "ssid", ssid, &n) != ESP_OK) ssid[0] = 0;
    n = sizeof pass; if (nvs_get_str(h, "pass", pass, &n) != ESP_OK) pass[0] = 0;
    n = sizeof dispname; if (nvs_get_str(h, "name", dispname, &n) != ESP_OK) dispname[0] = 0;
    if (nvs_get_i8(h, "txpower", &txpower_dbm) != ESP_OK) txpower_dbm = 0;
    nvs_close(h);
    have_creds = ssid[0] != 0;
}
static void apply_tx_power(void) { if (txpower_dbm >= 2 && txpower_dbm <= 20) esp_wifi_set_max_tx_power((int8_t)(txpower_dbm * 4)); }
int wifi_tx_power(void) { int8_t q = 0; if (esp_wifi_get_max_tx_power(&q) != ESP_OK) return 0; return q / 4; }
bool wifi_set_tx_power(int dbm) {
    if (dbm < 2 || dbm > 20) return false;
    txpower_dbm = (int8_t)dbm; if (started) apply_tx_power();
    nvs_handle_t h; if (nvs_open("vz80", NVS_READWRITE, &h) == ESP_OK) { nvs_set_i8(h, "txpower", txpower_dbm); nvs_commit(h); nvs_close(h); }
    return true;
}

static void start_ap(void) {
    if (ap_up) return;
    wifi_config_t ap = {0};
    strncpy((char *)ap.ap.ssid, ap_ssid, sizeof ap.ap.ssid);
    strncpy((char *)ap.ap.password, AP_PASS, sizeof ap.ap.password);
    ap.ap.ssid_len = strlen(ap_ssid); ap.ap.channel = 1; ap.ap.max_connection = 2;
    ap.ap.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_mode_t m; esp_wifi_get_mode(&m);
    esp_wifi_set_mode(m == WIFI_MODE_STA ? WIFI_MODE_APSTA : WIFI_MODE_AP);
    esp_wifi_set_config(WIFI_IF_AP, &ap);
    ap_up = true;
}

static void on_fallback(void *arg) { if (!sta_ip) start_ap(); }

static void on_event(void *arg, esp_event_base_t base, int32_t id, void *data) {
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) esp_wifi_connect();
    else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t *d = data;
        last_reason = d->reason; sta_ip = false; ip_u32 = 0; strcpy(ip_str, "0.0.0.0"); retries++;
        esp_wifi_connect();   // keep trying forever; the AP fallback covers reachability meanwhile
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = data;
        snprintf(ip_str, sizeof ip_str, IPSTR, IP2STR(&e->ip_info.ip));
        ip_u32 = e->ip_info.ip.addr; sta_ip = true;
    }
}

static void save_creds(const char *s, const char *p) {
    nvs_handle_t h;
    if (nvs_open("vz80", NVS_READWRITE, &h) == ESP_OK) {
        if (s && s[0]) { nvs_set_str(h, "ssid", s); nvs_set_str(h, "pass", p ? p : ""); }
        else { nvs_erase_key(h, "ssid"); nvs_erase_key(h, "pass"); }
        nvs_commit(h); nvs_close(h);
    }
}
// /vz80.cfg on the card wins over what NVS holds (the card is the thing the user edits).
static void take_sd_cfg(void) {
    const char *s, *p, *nm;
    if (!sd_cfg(&s, &p, &nm)) return;
    if (strcmp(s, ssid) || strcmp(p, pass)) { strncpy(ssid, s, sizeof ssid - 1); strncpy(pass, p, sizeof pass - 1); save_creds(ssid, pass); }
    have_creds = true;
    if (nm[0] && strcmp(nm, dispname)) wifi_set_name(nm);
}
void wifi_apply_cfg(const char *s, const char *p, const char *nm) {
    if (!s || !s[0]) return;
    bool changed = strcmp(s, ssid) || strcmp(p ? p : "", pass);
    if (nm && nm[0] && strcmp(nm, dispname)) wifi_set_name(nm);
    if (!changed) return;
    strncpy(ssid, s, sizeof ssid - 1); strncpy(pass, p ? p : "", sizeof pass - 1); have_creds = true;
    save_creds(ssid, pass);
    if (!started) return;
    wifi_config_t sc = {0};
    strncpy((char *)sc.sta.ssid, ssid, sizeof sc.sta.ssid);
    strncpy((char *)sc.sta.password, pass, sizeof sc.sta.password);
    sc.sta.threshold.authmode = pass[0] ? WIFI_AUTH_WPA_PSK : WIFI_AUTH_OPEN;
    wifi_mode_t m; esp_wifi_get_mode(&m);
    if (m == WIFI_MODE_AP) esp_wifi_set_mode(WIFI_MODE_APSTA);
    esp_wifi_set_config(WIFI_IF_STA, &sc);
    esp_wifi_disconnect(); esp_wifi_connect();
}
void wifi_start(void) {
    load_creds();
    take_sd_cfg();
    uint8_t mac[6]; esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);
    snprintf(ap_ssid, sizeof ap_ssid, "vz80-%02X%02X", mac[4], mac[5]);
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    snprintf(macstr, sizeof macstr, "%02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    esp_netif_init(); esp_event_loop_create_default();
    esp_netif_t *sta = esp_netif_create_default_wifi_sta(); esp_netif_create_default_wifi_ap();
    esp_netif_set_hostname(sta, "vz80");
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, on_event, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, on_event, NULL);
    esp_wifi_set_storage(WIFI_STORAGE_RAM);
    if (have_creds) {
        wifi_config_t sc = {0};
        strncpy((char *)sc.sta.ssid, ssid, sizeof sc.sta.ssid);
        strncpy((char *)sc.sta.password, pass, sizeof sc.sta.password);
        sc.sta.threshold.authmode = pass[0] ? WIFI_AUTH_WPA_PSK : WIFI_AUTH_OPEN;
        esp_wifi_set_mode(WIFI_MODE_STA);
        esp_wifi_set_config(WIFI_IF_STA, &sc);
        const esp_timer_create_args_t ta = { .callback = on_fallback, .name = "apfb" };
        esp_timer_create(&ta, &fallback_timer);
        esp_timer_start_once(fallback_timer, 20 * 1000000);
    } else {
        esp_wifi_set_mode(WIFI_MODE_AP);
        start_ap();
    }
    esp_wifi_start();
    esp_wifi_set_ps(WIFI_PS_NONE);
    apply_tx_power();
    started = true;
}

void wifi_status(char *out, size_t n) {
    snprintf(out, n, "esp fw=%s mode=%s%s ssid=%s ip=%s ap=%s retries=%d reason=%d heap=%lu",
             VZ80_VERSION, have_creds ? "sta" : "", ap_up ? "ap" : "", have_creds ? ssid : "-", ip_str,
             ap_up ? ap_ssid : "-", retries, last_reason, (unsigned long)esp_get_free_heap_size());
}

void wifi_set_credentials(const char *s, const char *p) { save_creds(s, p); }

uint32_t wifi_ip4(void) { return sta_ip ? ip_u32 : 0; }
const char *wifi_mac_str(void) { return macstr; }
const char *wifi_state_str(void) { return sta_ip ? "up" : have_creds ? "joining" : "unset"; }
const char *wifi_ssid(void) { return have_creds ? ssid : ""; }
const char *wifi_pass(void) { return have_creds ? pass : ""; }
const char *wifi_name(void) { return dispname; }
void wifi_set_name(const char *name) {
    nvs_handle_t h;
    strncpy(dispname, name ? name : "", sizeof dispname - 1); dispname[sizeof dispname - 1] = 0;
    if (nvs_open("vz80", NVS_READWRITE, &h) == ESP_OK) { nvs_set_str(h, "name", dispname); nvs_commit(h); nvs_close(h); }
}
