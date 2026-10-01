#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
// Command link to the ESP32. Board 1.0: UART1 (GPIO41/42). Board 2.0: SPI, the ESP32 is the master and
// PIO1 runs a slave on SCK 41 / MOSI 42 / MISO 46 / CS 44 (link_spi.c side of this file). Both carry the
// same byte pipe: the ESP forwards console commands as plain lines; replies are the command output
// terminated by RS (0x1e) + '\n'. Lines the RP sends starting with '?' are requests to the ESP; it
// answers with a '!' line. Binary payloads: "bin N\n" followed by exactly N raw bytes (queued with
// link_send_pieces, pumped in the background) before the RS terminator.
void link_init(void);
char *link_poll(void);
uint32_t link_last_rx_ms(void);   // time of the last complete line from the ESP32 (0 = never)
void link_begin(void);
void link_end(void);
bool link_request(const char *req, char *reply, size_t n, unsigned timeout_ms);
void link_set_baud(uint32_t baud);     // UART mode only; after the current output drained
typedef struct { const void *p; uint32_t len; } link_piece_t;
bool link_send_pieces(const link_piece_t *pieces, unsigned count);   // false if a send is in progress
bool link_sending(void);
void link_pump(void);
int link_rx_getc(void);                                    // -1 if nothing pending
bool link_rx_read(uint8_t *dst, uint32_t n, unsigned timeout_ms);
void link_stat_print(void);
// ---- transport (board 1.0 / 2.0) ----
enum { LINK_UART = 0, LINK_SPI = 1 };
int  link_mode(void);
void link_set_mode(int mode);          // switches the pins between UART1 and the PIO SPI slave
void link_task(void);                  // console loop: auto-detection (CS activity -> SPI, silence -> back to UART)
void link_dl_begin(void);              // esp dl: UART mode, CS/IO9 driven by us, detection off
void link_dl_end(void);
void link_note_esp_reset(void);      // esp.c: ignore CS edges for a while (IO9 floats while the ESP32 is in reset)
int  board_kind(void);                 // 1 / 2 as forced (settings.board) or detected; 0 = still unknown
typedef struct { uint32_t frames, bad_hdr, bad_pay, rx_drop, resync, tx_bytes, rx_bytes; uint32_t last_ok_ms; uint8_t rx_seq, tx_seq; } link_spi_stats_t;
const link_spi_stats_t *link_spi_stats(void);
