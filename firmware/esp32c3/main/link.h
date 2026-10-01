#pragma once
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
void link_init(void);
// Run one console command on the RP2350; output (without the RS terminator) lands in out.
// Returns bytes written, or -1 on timeout / busy.
int link_command(const char *cmd, char *out, size_t n, int timeout_ms);
// Firmware update relay: send "fwup SIZE" then stream pieces; each piece must be acked by "ok".
// Returns 0 on success, -1 on error; `status` receives the RP's final line.
int link_push_begin(const char *cmd, char *status, size_t n);   // generic "ready"-then-pieces push (see link_fwup_*)
int link_fwup_begin(uint32_t size, bool trial, char *status, size_t n);
int link_fwup_piece(const uint8_t *data, size_t len, int timeout_ms);
int link_fwup_end(int timeout_ms);
// Binary reply: run `cmd`; the RP answers "bin N" + N raw bytes. Bytes are delivered through
// the returned stream buffer; the caller reads until `total` bytes were consumed. Returns NULL
// on error. Call link_binary_end() afterwards (waits for the RS terminator, releases the link).
#include "freertos/FreeRTOS.h"
#include "freertos/stream_buffer.h"
StreamBufferHandle_t link_binary_begin(const char *cmd, uint32_t *total, int timeout_ms);
void link_binary_end(int timeout_ms);
const char *link_last_reply(void);   // the header line of the last binary request (e.g. "err ...")
// Negotiate the link speed (called at boot and whenever the RP seems to have rebooted).
void link_negotiate(void);
uint32_t link_baud(void);
uint32_t link_resyncs(void);   // number of baud-rate changes so far (bumps when the RP was found rebooted)
bool link_healthy(void);
void link_diag(char *buf, size_t n);   // uart/rx_task/link_command state for the beacon   // a reply arrived within the last 8 s
void link_stats(uint32_t *rx, uint32_t *ok_ago_ms, uint32_t *negs, int *stage);
// ---- transport (board 1.0 UART / board 2.0 SPI) ----
enum { LINK_MODE_UART = 0, LINK_MODE_SPI = 1 };
int link_mode(void);
bool link_board_probe(void);          // UART mode only: IO1 high = the RP2350 says "board 2.0"
typedef struct { uint32_t frames, bad_hdr, bad_pay, resync, tx_bytes, rx_bytes, last_ok_ms; bool busy; } link_spi_stats_t;
const link_spi_stats_t *link_spi_stats(void);
void link_msx_reset(void);            // board 2.0: pulse IO18 (Q1 pulls the MSX /RESET low)
