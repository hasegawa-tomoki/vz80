// Link to the RP2350. Board 1.0: UART0 (IO21 TX -> RP GPIO41, IO20 RX <- RP GPIO42). Board 2.0: SPI, we
// are the master on SPI2 shared with the SD card (SCK/MOSI are mirrored to IO21/IO20 through the GPIO
// matrix, MISO is switched between the card's IO7 and the RP's IO1 per transaction, CS = IO9).
// Plain lines = console commands (from Wi-Fi); the RP answers with the command output terminated by RS
// (0x1e) '\n'. Lines from the RP starting with '?' are requests to us (status / wifi credentials /
// reboot); we answer with a '!' line. The byte pipe is the same on both transports.
//
// SPI frames: 16-byte header both ways, then max(len_esp, len_rp) payload bytes rounded up to 4 (at
// least 4). One payload in flight per direction until the peer acknowledges its sequence number.
// Board detection: the RP drives its GPIO46 (= our IO1 on 2.0) high while its link is in UART mode; on
// 1.0 IO1 is unconnected and reads low through the pull-down.
#include <string.h>
#include <stdio.h>
#include "link.h"
#include "wifi.h"
#include "led.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_rom_gpio.h"
#include "soc/gpio_sig_map.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "esp_log.h"
#include <stdlib.h>
#include "esp_timer.h"

#define LINK_UART UART_NUM_0
#define RS 0x1e
#define FAST_BAUD 3000000
#define LINK_SPI_HZ 10000000       // bring-up value; the RP2350's PIO slave keeps up to ~20 MHz at 150 MHz sys clock
#define PIN_MISO_RP 1
#define PIN_MISO_SD 7
#define PIN_SCK_RP 21
#define PIN_MOSI_RP 20
#define PIN_CS_RP 9
#define PIN_MSX_RST 18
#define SPI_HDR 16
#define SPI_MAX 512
#define SPI_MAGIC 0xA5
#define F_RESET 1
static const char *TAG = "link";

typedef struct __attribute__((packed)) { uint8_t magic, seq; uint16_t len, window; uint8_t led[3], flags, ack, pad; uint16_t pcrc, hcrc; } spi_hdr_t;
_Static_assert(sizeof(spi_hdr_t) == SPI_HDR, "spi header size");

static SemaphoreHandle_t cmd_mutex, done_sem;
static char *reply_buf; static size_t reply_cap, reply_len; static volatile bool collecting;
static volatile bool fw_mode; static SemaphoreHandle_t fw_ok_sem; static char fw_last[200];
static uint32_t resync_count;
static volatile bool bin_mode; static uint32_t bin_left; static StreamBufferHandle_t bin_sb; static SemaphoreHandle_t bin_hdr_sem; static uint32_t bin_total;
static uint32_t cur_baud = 115200;
static uint32_t rx_bytes, ok_ms, neg_count; static volatile int neg_stage;   // diagnostics for /api/status
static volatile uint32_t rxtask_ms; static volatile int cmd_stage;   // rx_task liveness, link_command progress
static volatile bool neg_wanted;   // negotiation is waiting for the mutex: commands yield (the httpd task outranks main)
static volatile bool rx_reset_req;   // drop the partial line after a transport change

// ---- transport ----
static volatile int mode = LINK_MODE_UART; static bool uart_up;
static StreamBufferHandle_t spi_rx_sb, spi_tx_sb;
static spi_device_handle_t spi_dev;
static uint8_t m_seq = 1, m_rxseq; static bool m_synced; static uint8_t m_pend[SPI_MAX + 4] __attribute__((aligned(4))); static size_t m_pend_len; static uint16_t s_window = SPI_MAX;
static uint8_t s_rx[SPI_MAX + 4] __attribute__((aligned(4)));
static link_spi_stats_t sst; static uint8_t last_led[3] = {255, 255, 255};
static uint16_t crc_tab[256];
static void crc_init(void) { for (unsigned i = 0; i < 256; i++) { uint16_t c = (uint16_t)(i << 8); for (int k = 0; k < 8; k++) c = (c & 0x8000) ? (uint16_t)((c << 1) ^ 0x1021) : (uint16_t)(c << 1); crc_tab[i] = c; } }
static uint16_t crc16(const uint8_t *p, size_t n) { uint16_t c = 0xFFFF; while (n--) c = (uint16_t)((c << 8) ^ crc_tab[(c >> 8) ^ *p++]); return c; }
static inline uint32_t now_ms(void) { return (uint32_t)(esp_timer_get_time() / 1000); }

static void uart_start(void) {
    if (uart_up) return;
    uart_config_t cfg = { .baud_rate = 115200, .data_bits = UART_DATA_8_BITS, .parity = UART_PARITY_DISABLE,
                          .stop_bits = UART_STOP_BITS_1, .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, .source_clk = UART_SCLK_DEFAULT };
    uart_driver_install(LINK_UART, 16384, 4096, 0, NULL, 0);
    uart_param_config(LINK_UART, &cfg);
    uart_set_pin(LINK_UART, PIN_SCK_RP, PIN_MOSI_RP, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);   // TX=GPIO21 -> RP GPIO41, RX=GPIO20 <- RP GPIO42
    cur_baud = 115200; uart_up = true;
    gpio_set_direction(PIN_MISO_RP, GPIO_MODE_INPUT); gpio_set_pull_mode(PIN_MISO_RP, GPIO_PULLDOWN_ONLY);   // board probe (see link_board_probe)
}
static void uart_stop(void) { if (!uart_up) return; uart_wait_tx_done(LINK_UART, pdMS_TO_TICKS(100)); uart_driver_delete(LINK_UART); uart_up = false; }
bool link_board_probe(void) { return mode == LINK_MODE_UART && gpio_get_level(PIN_MISO_RP) == 1; }

static void spi_task(void *arg);
static void miso_select(int gpio) { esp_rom_gpio_connect_in_signal(gpio, FSPIQ_IN_IDX, false); }
static bool spi_start(void) {
    // buffers and the poll task exist only once a 2.0 board is seen: the ESP32's heap is tight (~30 KB free on 1.0)
    if (!spi_rx_sb) { spi_rx_sb = xStreamBufferCreate(4096, 1); spi_tx_sb = xStreamBufferCreate(4096, 1); if (!spi_rx_sb || !spi_tx_sb) { ESP_LOGE(TAG, "no memory for the SPI link"); return false; } }
    if (!spi_dev) {
        spi_device_interface_config_t d = { .clock_speed_hz = LINK_SPI_HZ, .mode = 0, .spics_io_num = PIN_CS_RP, .queue_size = 2, .input_delay_ns = 25 };
        esp_err_t e = spi_bus_add_device(SPI2_HOST, &d, &spi_dev);
        if (e != ESP_OK) { ESP_LOGE(TAG, "spi device: %s", esp_err_to_name(e)); spi_dev = NULL; return false; }
    }
    uart_stop();
    // mirror the bus clock / MOSI onto the RP's pins (the SD card keeps IO5 / IO6); MISO is chosen per transaction
    gpio_set_direction(PIN_SCK_RP, GPIO_MODE_OUTPUT);  esp_rom_gpio_connect_out_signal(PIN_SCK_RP, FSPICLK_OUT_IDX, false, false);
    gpio_set_direction(PIN_MOSI_RP, GPIO_MODE_OUTPUT); esp_rom_gpio_connect_out_signal(PIN_MOSI_RP, FSPID_OUT_IDX, false, false);
    gpio_set_direction(PIN_MISO_RP, GPIO_MODE_INPUT); gpio_set_pull_mode(PIN_MISO_RP, GPIO_FLOATING);
    m_seq = 1; m_rxseq = 0; m_synced = false; m_pend_len = 0; s_window = SPI_MAX;
    xStreamBufferReset(spi_rx_sb); xStreamBufferReset(spi_tx_sb);
    sst.resync++; rx_reset_req = true;
    mode = LINK_MODE_SPI;
    if (!led_ready()) led_init();                                   // the RMT driver lives on the heap: only for a 2.0 board
    static bool task_started; if (!task_started) { task_started = true; xTaskCreate(spi_task, "link_spi", 3072, NULL, 9, NULL); }
    ESP_LOGI(TAG, "SPI link");
    return true;
}
static void spi_stop(void) {
    if (mode != LINK_MODE_SPI) return;
    mode = LINK_MODE_UART;
    gpio_reset_pin(PIN_SCK_RP); gpio_reset_pin(PIN_MOSI_RP);
    uart_start(); rx_reset_req = true;
    ESP_LOGI(TAG, "UART link");
}
int link_mode(void) { return mode; }
const link_spi_stats_t *link_spi_stats(void) { return &sst; }

// One SPI exchange: header phase (CS kept low), then the payload phase.
static void spi_exchange(void) {
    if (!m_pend_len) {
        size_t lim = s_window < SPI_MAX ? s_window : SPI_MAX;
        if (lim) { m_pend_len = xStreamBufferReceive(spi_tx_sb, m_pend, lim, 0); }
    }
    spi_hdr_t h = { .magic = SPI_MAGIC, .seq = m_seq, .len = (uint16_t)m_pend_len, .window = 0, .flags = m_synced ? 0 : F_RESET, .ack = m_rxseq };
    memset(h.led, 0, 3); h.pad = 0;
    h.pcrc = crc16(m_pend, m_pend_len); h.hcrc = crc16((const uint8_t *)&h, SPI_HDR - 2);
    spi_hdr_t sh; memset(&sh, 0, sizeof sh);
    spi_device_acquire_bus(spi_dev, portMAX_DELAY);
    miso_select(PIN_MISO_RP);
    spi_transaction_t t1 = { .flags = SPI_TRANS_CS_KEEP_ACTIVE, .length = SPI_HDR * 8, .tx_buffer = &h, .rx_buffer = &sh };
    esp_err_t e1 = spi_device_polling_transmit(spi_dev, &t1);
    bool ok = e1 == ESP_OK && sh.magic == SPI_MAGIC && crc16((const uint8_t *)&sh, SPI_HDR - 2) == sh.hcrc;
    size_t L = m_pend_len;
    if (ok && sh.len <= SPI_MAX && sh.len > L) L = sh.len;
    L = (L + 3) & ~3u; if (L < 4) L = 4;
    spi_transaction_t t2 = { .length = L * 8, .tx_buffer = m_pend, .rx_buffer = s_rx };
    esp_err_t e2 = spi_device_polling_transmit(spi_dev, &t2);
    miso_select(PIN_MISO_SD);
    spi_device_release_bus(spi_dev);
    sst.frames++;
    if (!ok || e2 != ESP_OK) { sst.bad_hdr++; return; }
    sst.last_ok_ms = now_ms(); s_window = sh.window;
    if (!m_synced || (sh.flags & F_RESET)) { m_synced = true; m_rxseq = (uint8_t)(sh.seq - 1); }
    if (m_pend_len && sh.ack == m_seq) { sst.tx_bytes += m_pend_len; m_pend_len = 0; m_seq++; }
    if (sh.len) {
        if (sh.len > SPI_MAX || crc16(s_rx, sh.len) != sh.pcrc) sst.bad_pay++;
        else if (sh.seq == (uint8_t)(m_rxseq + 1)) {
            if (xStreamBufferSend(spi_rx_sb, s_rx, sh.len, pdMS_TO_TICKS(1000)) == sh.len) { m_rxseq = sh.seq; sst.rx_bytes += sh.len; rx_bytes += sh.len; }
        }
    }
    if (memcmp(sh.led, last_led, 3)) { memcpy(last_led, sh.led, 3); led_set(sh.led[0], sh.led[1], sh.led[2]); }
    sst.busy = sh.len || m_pend_len || xStreamBufferBytesAvailable(spi_tx_sb) > 0;
}
static void spi_task(void *arg) {
    for (;;) {
        if (mode != LINK_MODE_SPI) { vTaskDelay(pdMS_TO_TICKS(100)); continue; }
        spi_exchange();
        if (sst.busy) esp_rom_delay_us(60);       // the RP re-arms its DMA in the CS interrupt: leave it that much
        else vTaskDelay(1);
    }
}

static int xport_read(uint8_t *buf, size_t cap, int timeout_ms) {
    if (mode == LINK_MODE_SPI) return (int)xStreamBufferReceive(spi_rx_sb, buf, cap, pdMS_TO_TICKS(timeout_ms));
    if (!uart_up) { vTaskDelay(pdMS_TO_TICKS(timeout_ms)); return 0; }
    int len = uart_read_bytes(LINK_UART, buf, 1, pdMS_TO_TICKS(timeout_ms));
    if (len == 1) { int more = uart_read_bytes(LINK_UART, buf + 1, cap - 1, 0); if (more > 0) len += more; }
    return len;
}
static void xport_write(const void *data, size_t len) {
    if (mode == LINK_MODE_SPI) { xStreamBufferSend(spi_tx_sb, data, len, pdMS_TO_TICKS(5000)); return; }
    if (uart_up) uart_write_bytes(LINK_UART, data, len);
}
static void send_line(const char *s) { xport_write(s, strlen(s)); xport_write("\n", 1); }

static void handle_request(char *req) {
    char out[240];
    if (!strcmp(req, "status")) { wifi_status(out, sizeof out); }
    else if (!strncmp(req, "wifi", 4)) {
        char *s = strtok(req + 4, " "), *p = strtok(NULL, " ");
        wifi_set_credentials(s, p);
        snprintf(out, sizeof out, "ok %s, restarting esp", s ? "stored" : "cleared");
        xport_write("!", 1); send_line(out);
        vTaskDelay(pdMS_TO_TICKS(200)); esp_restart();
    }
    else if (!strcmp(req, "reboot")) { send_line("!ok"); vTaskDelay(pdMS_TO_TICKS(100)); esp_restart(); }
    else if (!strcmp(req, "msxreset")) { link_msx_reset(); snprintf(out, sizeof out, "ok"); }
    else snprintf(out, sizeof out, "err unknown request");
    xport_write("!", 1); send_line(out);
}

static void rx_task(void *arg) {
    static char line[2048]; size_t n = 0; static uint8_t buf[1024];
    for (;;) {
        // Wait for the first byte, then take whatever else is already there: a short "ok" line must not
        // sit behind a 50 ms read timeout (it cost ~50 ms per 4 KiB upload piece).
        int len = xport_read(buf, sizeof buf, 50);
        rxtask_ms = now_ms();
        if (rx_reset_req) { rx_reset_req = false; n = 0; bin_mode = false; len = 0; }
        if (len > 0 && mode == LINK_MODE_UART) rx_bytes += (uint32_t)len;
        for (int i = 0; i < len; i++) {
            if (bin_mode) {                              // raw payload: hand whole runs to the stream buffer
                uint32_t take = (uint32_t)(len - i) < bin_left ? (uint32_t)(len - i) : bin_left;
                xStreamBufferSend(bin_sb, buf + i, take, pdMS_TO_TICKS(2000));
                bin_left -= take; i += take - 1;
                if (bin_left == 0) bin_mode = false;
                continue;
            }
            char c = (char)buf[i];
            if (c == RS) { ok_ms = now_ms(); if (collecting) { collecting = false; xSemaphoreGive(done_sem); } n = 0; continue; }   // any complete reply proves the RP is alive (long binary transfers included)
            if (c == '\r') continue;
            if (c != '\n') { if (n < sizeof line - 1) line[n++] = c; continue; }
            line[n] = 0;
            if (fw_mode) {
                if (!strcmp(line, "ok") || !strcmp(line, "ready")) xSemaphoreGive(fw_ok_sem);
                else if (n) { strncpy(fw_last, line, sizeof fw_last - 1); fw_last[sizeof fw_last - 1] = 0; if (!strncmp(line, "err", 3) || !strncmp(line, "failed", 6) || !strncmp(line, "done", 4)) xSemaphoreGive(fw_ok_sem); }
                n = 0; continue;
            }
            if (line[0] == '?') handle_request(line + 1);
            else if (collecting && bin_sb && !strncmp(line, "bin ", 4)) { bin_total = strtoul(line + 4, NULL, 10); bin_left = bin_total; bin_mode = bin_left > 0; xSemaphoreGive(bin_hdr_sem); }
            else if (collecting && n) {
                size_t room = reply_cap - reply_len - 1;
                size_t take = n < room ? n : room;
                memcpy(reply_buf + reply_len, line, take); reply_len += take;
                if (reply_len < reply_cap - 1) reply_buf[reply_len++] = '\n';
            }
            n = 0;
        }
    }
}

void link_init(void) {
    crc_init();
    cmd_mutex = xSemaphoreCreateMutex(); done_sem = xSemaphoreCreateBinary(); fw_ok_sem = xSemaphoreCreateBinary(); bin_hdr_sem = xSemaphoreCreateBinary();
    gpio_reset_pin(PIN_MSX_RST); gpio_set_direction(PIN_MSX_RST, GPIO_MODE_OUTPUT); gpio_set_level(PIN_MSX_RST, 0);   // 2.0: Q1 gate (active high pulls /RESET low)
    uart_start();
    xTaskCreate(rx_task, "link_rx", 4096, NULL, 10, NULL);
}
void link_msx_reset(void) { gpio_set_level(PIN_MSX_RST, 1); vTaskDelay(pdMS_TO_TICKS(120)); gpio_set_level(PIN_MSX_RST, 0); }

// Send a command and collect its reply; the caller must hold cmd_mutex.
static int cmd_locked(const char *cmd, char *out, size_t n, int timeout_ms) {
    reply_buf = out; reply_cap = n; reply_len = 0; out[0] = 0;
    xSemaphoreTake(done_sem, 0);
    collecting = true;
    cmd_stage = 2; send_line(cmd); cmd_stage = 3;
    bool ok = xSemaphoreTake(done_sem, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
    cmd_stage = 0;
    collecting = false;
    if (ok) ok_ms = now_ms();
    out[reply_len] = 0;
    return ok ? (int)reply_len : -1;
}
int link_command(const char *cmd, char *out, size_t n, int timeout_ms) {
    // While the RP is not answering (e.g. it just rebooted at 115200), fail fast so that the negotiation
    // task can take the mutex and re-sync instead of starving behind UI polling.
    if (!link_healthy()) timeout_ms = timeout_ms > 700 ? 700 : timeout_ms;
    if (neg_wanted) return -1;
    cmd_stage = 1;
    if (xSemaphoreTake(cmd_mutex, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) { cmd_stage = 0; return -1; }
    int rc = cmd_locked(cmd, out, n, timeout_ms);
    xSemaphoreGive(cmd_mutex);
    return rc;
}

static char *fw_status; static size_t fw_status_n;
// Any RP command that answers "ready" and then takes the payload in 4096-byte pieces (fwup, shelf put).
int link_push_begin(const char *cmd, char *status, size_t n) {
    if (xSemaphoreTake(cmd_mutex, pdMS_TO_TICKS(2000)) != pdTRUE) return -1;
    fw_status = status; fw_status_n = n; fw_last[0] = 0; status[0] = 0;
    xSemaphoreTake(fw_ok_sem, 0);
    fw_mode = true;
    send_line(cmd);
    if (xSemaphoreTake(fw_ok_sem, pdMS_TO_TICKS(5000)) != pdTRUE || strncmp(fw_last, "err", 3) == 0) { snprintf(status, n, "no ready: %s", fw_last); fw_mode = false; xSemaphoreGive(cmd_mutex); return -1; }
    return 0;
}
int link_fwup_begin(uint32_t size, bool trial, char *status, size_t n) {
    char cmd[32]; snprintf(cmd, sizeof cmd, "fwup %lu%s", (unsigned long)size, trial ? " b" : "");   // "b": slot B, booted once as a trial
    return link_push_begin(cmd, status, n);
}
int link_fwup_piece(const uint8_t *data, size_t len, int timeout_ms) {
    xport_write(data, len);
    if (xSemaphoreTake(fw_ok_sem, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) { snprintf(fw_status, fw_status_n, "piece timeout: %s", fw_last); return -1; }
    if (fw_last[0] && (!strncmp(fw_last, "err", 3) || !strncmp(fw_last, "failed", 6))) { snprintf(fw_status, fw_status_n, "%s", fw_last); return -1; }
    return 0;
}
int link_fwup_end(int timeout_ms) {
    // wait for "done ..." (or an error) then leave fw mode
    int rc = 0;
    if (!strncmp(fw_last, "done", 4)) rc = 0;
    else if (xSemaphoreTake(fw_ok_sem, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) rc = -1;
    if (strncmp(fw_last, "done", 4)) rc = -1;
    snprintf(fw_status, fw_status_n, "%s", fw_last);
    fw_mode = false; xSemaphoreGive(cmd_mutex);
    return rc;
}

static char bin_reply[64];
StreamBufferHandle_t link_binary_begin(const char *cmd, uint32_t *total, int timeout_ms) {
    if (neg_wanted) return NULL;
    if (!link_healthy()) timeout_ms = timeout_ms > 700 ? 700 : timeout_ms;
    if (xSemaphoreTake(cmd_mutex, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) return NULL;
    if (!bin_sb) bin_sb = xStreamBufferCreate(4096, 1);   // 4 KB is plenty: web.c drains it in 2 KB pieces (the ESP32-C3 heap is tight)
    xStreamBufferReset(bin_sb); xSemaphoreTake(bin_hdr_sem, 0); xSemaphoreTake(done_sem, 0);
    reply_buf = bin_reply; reply_cap = sizeof bin_reply; reply_len = 0; bin_reply[0] = 0; bin_total = 0;
    collecting = true;
    send_line(cmd);
    if (xSemaphoreTake(bin_hdr_sem, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) { collecting = false; bin_mode = false; xSemaphoreGive(cmd_mutex); return NULL; }
    *total = bin_total;
    return bin_sb;
}
const char *link_last_reply(void) { return bin_reply; }
void link_binary_end(int timeout_ms) {
    xSemaphoreTake(done_sem, pdMS_TO_TICKS(timeout_ms));
    collecting = false; bin_mode = false;
    xSemaphoreGive(cmd_mutex);
}

uint32_t link_baud(void) { return mode == LINK_MODE_SPI ? LINK_SPI_HZ : cur_baud; }
uint32_t link_resyncs(void) { return resync_count; }
bool link_healthy(void) { return ok_ms && now_ms() - ok_ms < 8000; }
void link_stats(uint32_t *rx, uint32_t *ok_ago_ms, uint32_t *negs, int *stage) { *rx = rx_bytes; *ok_ago_ms = ok_ms ? now_ms() - ok_ms : 0; *negs = neg_count; *stage = neg_stage; }
void link_diag(char *buf, size_t n) {
    size_t txfree = 0; if (uart_up) uart_get_tx_buffer_free_size(LINK_UART, &txfree);
    snprintf(buf, n, "%s txfree=%u rxage=%lu cmd=%d bin=%d/%lu col=%d spi f=%lu bh=%lu bp=%lu", mode == LINK_MODE_SPI ? "spi" : "uart", (unsigned)txfree, (unsigned long)(now_ms() - rxtask_ms), cmd_stage, bin_mode, (unsigned long)bin_left, collecting, (unsigned long)sst.frames, (unsigned long)sst.bad_hdr, (unsigned long)sst.bad_pay);
}
static bool ping(void) { char out[64]; return cmd_locked("baud", out, sizeof out, 800) >= 0 && strstr(out, "usage") != NULL; }   // mutex held by caller
static void set_baud(uint32_t b) { resync_count++; uart_wait_tx_done(LINK_UART, pdMS_TO_TICKS(100)); uart_set_baudrate(LINK_UART, b); cur_baud = b; vTaskDelay(pdMS_TO_TICKS(30)); rx_reset_req = true; vTaskDelay(pdMS_TO_TICKS(60)); }
// UART: the RP boots at 115200 and switches to FAST_BAUD when asked; either side may have restarted on
// its own. Ping at the current speed, then at the other one, and ask for FAST_BAUD whenever the RP
// answers at 115200. SPI (board 2.0): a healthy link needs no negotiation; after 3 s without a valid
// frame fall back to UART (the RP may be running an older firmware, or is in the ESP download bridge).
// From UART, move to SPI when the board probe says 2.0 and the RP does not answer on the UART.
void link_negotiate(void) {
    static bool first = true;
    if (mode == LINK_MODE_SPI) {
        if (sst.last_ok_ms && now_ms() - sst.last_ok_ms < 3000) { neg_stage = 30; return; }
        if (xSemaphoreTake(cmd_mutex, pdMS_TO_TICKS(3000)) != pdTRUE) { neg_stage = 20; return; }
        neg_stage = 31; ESP_LOGW(TAG, "no SPI frames: back to UART");
        spi_stop(); resync_count++;
        xSemaphoreGive(cmd_mutex);
    }
    if (first) { first = false; if (link_board_probe()) { if (xSemaphoreTake(cmd_mutex, pdMS_TO_TICKS(3000)) == pdTRUE) { neg_stage = 32; spi_start(); xSemaphoreGive(cmd_mutex); return; } } }
    // Never touch the baud rate while a command or a firmware transfer owns the link: a busy mutex is
    // not a silent RP.
    neg_wanted = true;
    if (xSemaphoreTake(cmd_mutex, pdMS_TO_TICKS(3000)) != pdTRUE) { neg_wanted = false; neg_stage = 20; return; }
    neg_wanted = false;
    neg_count++; neg_stage = 1;
    static int misses;
    do {
        if (!ping()) {
            // One silent ping is not proof of a reboot: the RP stalls the console for a flash write
            // (catalog/settings) or a long MSX bus stall. Switch speeds only after two misses in a row.
            if (++misses < 2) { neg_stage = 14; break; }
            if (link_board_probe()) { neg_stage = 33; spi_start(); misses = 0; break; }   // 2.0 with the RP waiting for SPI
            neg_stage = 2;
            set_baud(cur_baud == FAST_BAUD ? 115200 : FAST_BAUD);
            neg_stage = 3;
            if (!ping()) { neg_stage = 10; break; }   // silent at both: keep alternating on the next call
        }
        misses = 0;
        if (cur_baud == FAST_BAUD) { neg_stage = 11; break; }
        neg_stage = 4;
        char out[64]; char cmd[32]; snprintf(cmd, sizeof cmd, "baud %u", FAST_BAUD);
        if (cmd_locked(cmd, out, sizeof out, 800) < 0 || strncmp(out, "ok", 2)) { neg_stage = 12; break; }
        neg_stage = 5;
        set_baud(FAST_BAUD);
        neg_stage = 6;
        if (!ping()) set_baud(115200);
        neg_stage = 13;
    } while (0);
    xSemaphoreGive(cmd_mutex);
}
