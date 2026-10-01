#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
void web_start(void);
void web_diag(char *buf, size_t n);
void web_touch(void);             // a handler doing a long transfer: not stalled
bool web_stalled(int64_t now_us);   // a request handler has been running for more than 90 s   // one-line httpd/install diagnostics for the beacon

// Firmware package (.vzp, tools/pkg/mkpkg.py): 64-byte header, RP2350 UF2, ESP32 app.
typedef struct { char magic[4]; uint32_t hdr_size; char label[24]; uint32_t rp_off, rp_len, rp_sum, esp_off, esp_len, esp_sum, flags, hdr_sum; } vzp_hdr_t;
typedef int (*fw_read_fn)(void *ctx, uint8_t *buf, size_t n);   // bytes read, <= 0 on failure
bool fw_check_header(const uint8_t *hb, vzp_hdr_t *h, uint32_t total_len, char *status, size_t n);
int  fw_install(const vzp_hdr_t *h, fw_read_fn rd, void *ctx, char *status, size_t n);   // 0 ok (caller restarts the ESP32 when h->esp_len)
