#include "nvs_flash.h"
#include "wifi.h"
#include "sd.h"
#include "cache.h"
#include "sdfw.h"
#include "link.h"
#include "web.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "discovery.h"
#include "esp_ota_ops.h"
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

void app_main(void) {
    esp_err_t e = nvs_flash_init();
    if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND) { nvs_flash_erase(); nvs_flash_init(); }
    link_init();
    sd_init();
    wifi_start();
    web_start();
    disc_start();
    sd_start_monitor();
    cache_start();
    sdfw_start();
    vTaskDelay(pdMS_TO_TICKS(1500));
    link_negotiate();
    // The ESP32 "pending verify" state mirrors the RP2350 trial: once the RP reports slot A (no trial
    // in progress, e.g. after a rollback or a plain restart) this app is confirmed too.
    bool confirmed = false; const char *last_state = "";
    for (;;) {
        for (int i = 0; i < 10; i++) { vTaskDelay(pdMS_TO_TICKS(500)); if (strcmp(wifi_state_str(), last_state)) break; }   // status LED on the RP2350: report changes quickly, otherwise every 5 s
        link_negotiate();   // re-negotiate if the RP rebooted
        { char cmd[32], out[32]; snprintf(cmd, sizeof cmd, "espstate %s", wifi_state_str()); if (link_command(cmd, out, sizeof out, 500) >= 0) last_state = wifi_state_str(); }
        if (!confirmed) { char out[64]; if (link_command("slot", out, sizeof out, 800) > 0 && out[0] == 'A') { esp_ota_mark_app_valid_cancel_rollback(); confirmed = true; } }
    }
}
