#include <string.h>
#include <stdio.h>
#include "discovery.h"
#include "wifi.h"
#include "web.h"
#include "esp_system.h"
#include "link.h"
#include "lwip/sockets.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static disc_peer_t peers[DISC_MAX_PEERS];
static SemaphoreHandle_t lock;

static void upsert(uint32_t ip, const char *line) {
    char dev[16], ver[16], nm[32] = ""; unsigned hp;
    if (sscanf(line, "x68pico %15s %15s %u %31[^\n]", dev, ver, &hp, nm) < 3) return;
    xSemaphoreTake(lock, portMAX_DELAY);
    int slot = -1;
    for (int i = 0; i < DISC_MAX_PEERS; i++) { if (peers[i].used && peers[i].ip == ip) { slot = i; break; } if (!peers[i].used && slot < 0) slot = i; }
    if (slot >= 0) {
        disc_peer_t *p = &peers[slot]; p->used = true; p->ip = ip; p->port = (uint16_t)hp;
        strncpy(p->device, dev, 15); p->device[15] = 0; strncpy(p->version, ver, 15); p->version[15] = 0; strncpy(p->name, nm, 31); p->name[31] = 0;
        p->last_us = esp_timer_get_time();
    }
    xSemaphoreGive(lock);
}

static void task(void *arg) {
    int s = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    int on = 1; setsockopt(s, SOL_SOCKET, SO_BROADCAST, &on, sizeof on);
    struct sockaddr_in a = { .sin_family = AF_INET, .sin_port = htons(DISC_PORT), .sin_addr.s_addr = htonl(INADDR_ANY) };
    bind(s, (struct sockaddr *)&a, sizeof a);
    struct timeval tv = { .tv_sec = 0, .tv_usec = 200000 }; setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);
    int64_t next_tx = 0;
    for (;;) {
        char line[128]; struct sockaddr_in from; socklen_t fl = sizeof from;
        int n = recvfrom(s, line, sizeof line - 1, 0, (struct sockaddr *)&from, &fl);
        uint32_t myip = wifi_ip4();
        if (n > 0) { line[n] = 0; if (from.sin_addr.s_addr != myip) upsert(from.sin_addr.s_addr, line); }
        int64_t now = esp_timer_get_time();
        xSemaphoreTake(lock, portMAX_DELAY);
        for (int i = 0; i < DISC_MAX_PEERS; i++) if (peers[i].used && now - peers[i].last_us > 10000000) peers[i].used = false;
        xSemaphoreGive(lock);
        if (now >= next_tx && myip) {
            next_tx = now + 2000000;
            const char *nm = wifi_name();
            int len = snprintf(line, sizeof line, "x68pico vz80 %s 80%s%s", VZ80_VERSION, nm[0] ? " " : "", nm);
            struct sockaddr_in b = { .sin_family = AF_INET, .sin_port = htons(DISC_PORT), .sin_addr.s_addr = htonl(INADDR_BROADCAST) };
            sendto(s, line, len, 0, (struct sockaddr *)&b, sizeof b);
            // second packet: diagnostics readable even when the web server is stuck
            static char diag[300], wd[120], ld[160];   // static: this task's stack is small and vsnprintf is not
            uint32_t lrx, lok, lneg; int lst; link_stats(&lrx, &lok, &lneg, &lst); web_diag(wd, sizeof wd);
            link_diag(ld, sizeof ld);
            len = snprintf(diag, sizeof diag, "vz80diag up=%lld link=%lu/%lu/%lu/%lu/%d %s hmin=%lu %s", (long long)(now / 1000000), (unsigned long)link_baud(), (unsigned long)lrx, (unsigned long)lok, (unsigned long)lneg, lst, wd, (unsigned long)esp_get_minimum_free_heap_size(), ld);
            sendto(s, diag, len, 0, (struct sockaddr *)&b, sizeof b);
            if (web_stalled(now)) { vTaskDelay(pdMS_TO_TICKS(100)); esp_restart(); }   // last resort: a stuck web server task
            // Out of heap for a minute (seen 2026-10-01: 5 KB free, the web server could not accept a connection while
            // the link and the beacon kept running): restart; the RP2350 and the MSX are not affected.
            static int64_t low_since; uint32_t freeh = esp_get_free_heap_size();
            if (freeh >= 8192) low_since = 0; else if (!low_since) low_since = now;
            else if (now - low_since > 60000000) { ESP_LOGE("disc", "heap %lu bytes for 60 s: restarting", (unsigned long)freeh); vTaskDelay(pdMS_TO_TICKS(100)); esp_restart(); }
        }
    }
}

void disc_start(void) { lock = xSemaphoreCreateMutex(); xTaskCreate(task, "disc", 4096, NULL, 4, NULL); }   // 3072 overflowed once link_diag grew (0.7.0)
const disc_peer_t *disc_peer(int i) { return (i >= 0 && i < DISC_MAX_PEERS && peers[i].used) ? &peers[i] : NULL; }
