// Command link to the ESP32: UART1 on board 1.0, SPI slave (PIO1 + DMA) on board 2.0. See link.h.
//
// SPI framing (both directions, the ESP32 clocks everything): a 16-byte header, then max(len_esp,
// len_rp) payload bytes rounded up to 4. Each side keeps one payload "in flight" until the other side
// acknowledges its sequence number in the next header (stop-and-wait; a corrupted frame is simply
// repeated). The RP prepares its next frame in the CS rising-edge interrupt, so the ESP32 must leave a
// few tens of microseconds between frames. `window` tells the ESP32 how much receive room the RP has.
#include <stdio.h>
#include <string.h>
#include "link.h"
#include "settings.h"
#include "led.h"
#include "hardware/watchdog.h"
#include "pins.h"
#include "pico/stdlib.h"
#include "pico/stdio.h"
#include "pico/stdio/driver.h"
#include "pico/stdio_usb.h"
#include "hardware/uart.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/sync.h"
#include "spi_slave.pio.h"

#define LUART uart1
#define RX_RING 8192                       // power of two; UART: DMA ring, SPI: filled from the CS interrupt
#define TX_RING 4096                       // SPI only: bytes waiting to go out
#define SPI_HDR 16
#define SPI_MAX 512
#define SPI_MAGIC 0xA5
#define F_RESET 1                          // header flag: the sender has not seen a valid frame from the peer since it started

typedef struct __attribute__((packed)) { uint8_t magic, seq; uint16_t len, window; uint8_t led[3], flags, ack, pad; uint16_t pcrc, hcrc; } spi_hdr_t;
_Static_assert(sizeof(spi_hdr_t) == SPI_HDR, "spi header size");

static void link_out_chars(const char *buf, int len);
static stdio_driver_t link_driver = { .out_chars = link_out_chars, .crlf_enabled = false };

static uint8_t rx_ring[RX_RING] __attribute__((aligned(RX_RING)));
static volatile uint32_t rx_rd, rx_wr_sw;          // rx_wr_sw: write index in SPI mode (free-running)
static int dma_rx = -1, dma_tx = -1;                // UART DMA channels
static uint32_t cur_baud = 115200;
static char line[256]; static unsigned n;
static char pending[256]; static bool have_pending;
static int mode = LINK_UART; static bool dl_active; static volatile bool cs_seen; static int detected;
static uint32_t spi_since_ms, esp_reset_ms; static bool committed;   // SPI trial: pins switched on a CS edge, the board is 2.0 only once a valid frame arrived

// ---- SPI slave state ----
static uint8_t tx_ring[TX_RING]; static volatile uint32_t tx_rd, tx_wr;   // free-running indices
static uint8_t spi_rxbuf[SPI_HDR + SPI_MAX] __attribute__((aligned(4))), spi_txbuf[SPI_HDR + SPI_MAX + 4] __attribute__((aligned(4)));
static PIO spio; static int ssm = -1, dma_srx = -1, dma_stx = -1; static uint soff; static bool spi_claimed;
static volatile link_spi_stats_t st; static volatile bool rx_synced; static volatile uint32_t tx_inflight;
static uint16_t crc_tab[256];

// ---- background pieces ----
#define MAX_PIECES 264
static link_piece_t pieces[MAX_PIECES]; static unsigned piece_n, piece_i; static uint32_t piece_off; static bool sending, end_deferred;

static void crc_init(void) {
    for (unsigned i = 0; i < 256; i++) { uint16_t c = (uint16_t)(i << 8); for (int k = 0; k < 8; k++) c = (c & 0x8000) ? (uint16_t)((c << 1) ^ 0x1021) : (uint16_t)(c << 1); crc_tab[i] = c; }
}
static inline uint16_t __not_in_flash_func(crc16)(const uint8_t *p, uint32_t n) { uint16_t c = 0xFFFF; while (n--) c = (uint16_t)((c << 8) ^ crc_tab[(c >> 8) ^ *p++]); return c; }

// ---- receive ring (shared by both transports) ----
static void rx_dma_start(void) {
    dma_channel_config c = dma_channel_get_default_config(dma_rx);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_ring(&c, true, __builtin_ctz(RX_RING));   // wrap the write address at the ring size
    channel_config_set_dreq(&c, uart_get_dreq(LUART, false));
    dma_channel_configure(dma_rx, &c, rx_ring, &uart_get_hw(LUART)->dr, 0x0FFFFFFF, true);
}
static inline uint32_t rx_wr(void) { return mode == LINK_SPI ? (rx_wr_sw & (RX_RING - 1)) : (uint32_t)(dma_channel_hw_addr(dma_rx)->write_addr - (uintptr_t)rx_ring) & (RX_RING - 1); }
static inline bool rx_avail(void) { return (rx_rd & (RX_RING - 1)) != rx_wr(); }
static inline uint8_t rx_get(void) { uint8_t c = rx_ring[rx_rd & (RX_RING - 1)]; rx_rd = (rx_rd + 1); return c; }

// ---- SPI frame handling (CS rising edge) ----
static void __not_in_flash_func(spi_arm)(void) {
    // next frame: resend the in-flight payload or take a fresh one from the ring
    uint32_t used = tx_wr - tx_rd;
    if (!tx_inflight) tx_inflight = used > SPI_MAX ? SPI_MAX : used;
    uint32_t len = tx_inflight;
    for (uint32_t i = 0; i < len; i++) spi_txbuf[SPI_HDR + i] = tx_ring[(tx_rd + i) & (TX_RING - 1)];
    spi_hdr_t *h = (spi_hdr_t *)spi_txbuf;
    h->magic = SPI_MAGIC; h->seq = st.tx_seq; h->len = (uint16_t)len;
    uint32_t rfree = RX_RING - 1 - (rx_wr_sw - rx_rd); h->window = rfree > 65535 ? 65535 : (uint16_t)rfree;
    led_rgb((uint8_t *)h->led); h->flags = rx_synced ? 0 : F_RESET; h->ack = st.rx_seq; h->pad = 0;
    h->pcrc = crc16(spi_txbuf + SPI_HDR, len); h->hcrc = crc16(spi_txbuf, SPI_HDR - 2);
    uint32_t tlen = SPI_HDR + ((len + 3) & ~3u);
    dma_channel_set_read_addr(dma_stx, spi_txbuf, false); dma_channel_set_trans_count(dma_stx, tlen, true);
    dma_channel_set_write_addr(dma_srx, spi_rxbuf, false); dma_channel_set_trans_count(dma_srx, SPI_HDR + SPI_MAX, true);
    pio_sm_set_enabled(spio, (uint)ssm, true);
}
static void __not_in_flash_func(spi_frame_end)(void) {
    uint32_t got = SPI_HDR + SPI_MAX - dma_channel_hw_addr(dma_srx)->transfer_count;
    dma_channel_abort(dma_srx); dma_channel_abort(dma_stx);
    pio_sm_set_enabled(spio, (uint)ssm, false);
    pio_sm_clear_fifos(spio, (uint)ssm);
    pio_sm_restart(spio, (uint)ssm);
    pio_sm_exec(spio, (uint)ssm, pio_encode_jmp(soff));
    st.frames++;
    spi_hdr_t *m = (spi_hdr_t *)spi_rxbuf;
    if (got < SPI_HDR || m->magic != SPI_MAGIC || crc16(spi_rxbuf, SPI_HDR - 2) != m->hcrc) st.bad_hdr++;
    else {
        st.last_ok_ms = to_ms_since_boot(get_absolute_time());
        if (!rx_synced || (m->flags & F_RESET)) { rx_synced = true; st.rx_seq = (uint8_t)(m->seq - 1); }   // adopt the peer's numbering
        if (tx_inflight && m->ack == st.tx_seq) { tx_rd += tx_inflight; st.tx_bytes += tx_inflight; tx_inflight = 0; st.tx_seq++; }
        if (m->len) {
            if (m->len > SPI_MAX || got < SPI_HDR + m->len || crc16(spi_rxbuf + SPI_HDR, m->len) != m->pcrc) st.bad_pay++;
            else if (m->seq == (uint8_t)(st.rx_seq + 1)) {
                uint32_t rfree = RX_RING - 1 - (rx_wr_sw - rx_rd);
                if (m->len > rfree) st.rx_drop++;
                else { for (uint32_t i = 0; i < m->len; i++) rx_ring[(rx_wr_sw + i) & (RX_RING - 1)] = spi_rxbuf[SPI_HDR + i]; rx_wr_sw += m->len; st.rx_seq = m->seq; st.rx_bytes += m->len; }
            }   // m->seq == rx_seq: a repeat of what we already took; anything else: out of step, the ack tells the peer
        }
    }
    spi_arm();
}
static void __not_in_flash_func(cs_irq)(uint gpio, uint32_t events) {
    if (gpio != PIN_LINK_CS) return;
    if (mode == LINK_SPI) { if (events & GPIO_IRQ_EDGE_RISE) spi_frame_end(); }
    else if ((events & GPIO_IRQ_EDGE_FALL) && !dl_active) cs_seen = true;
}

static bool spi_claim(void) {
    if (spi_claimed) return true;
    spio = pio1;
    if (!pio_can_add_program(spio, &spi_slave_program)) return false;
    int sm = pio_claim_unused_sm(spio, false); if (sm < 0) return false;
    int a = dma_claim_unused_channel(false), b = dma_claim_unused_channel(false);
    if (a < 0 || b < 0) { pio_sm_unclaim(spio, (uint)sm); if (a >= 0) dma_channel_unclaim((uint)a); if (b >= 0) dma_channel_unclaim((uint)b); return false; }
    ssm = sm; dma_srx = a; dma_stx = b;
    soff = pio_add_program(spio, &spi_slave_program);
    crc_init();
    dma_channel_config c = dma_channel_get_default_config(dma_stx);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8); channel_config_set_read_increment(&c, true); channel_config_set_write_increment(&c, false);
    channel_config_set_dreq(&c, pio_get_dreq(spio, (uint)ssm, true));
    dma_channel_configure(dma_stx, &c, &spio->txf[ssm], spi_txbuf, 0, false);
    c = dma_channel_get_default_config(dma_srx);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8); channel_config_set_read_increment(&c, false); channel_config_set_write_increment(&c, true);
    channel_config_set_dreq(&c, pio_get_dreq(spio, (uint)ssm, false));
    dma_channel_configure(dma_srx, &c, spi_rxbuf, &spio->rxf[ssm], 0, false);
    spi_claimed = true;
    return true;
}

int link_mode(void) { return mode; }
int board_kind(void) { return settings.board ? (int)settings.board : detected ? detected : (int)settings.board_seen; }
const link_spi_stats_t *link_spi_stats(void) { return (const link_spi_stats_t *)&st; }

void link_set_mode(int m) {
    if (m == LINK_SPI) {
        if (mode == LINK_SPI) return;
        if (!spi_claim()) { printf("link: no PIO/DMA for SPI\n"); return; }
        uart_tx_wait_blocking(LUART);
        dma_channel_abort(dma_rx);
        pio_gpio_init(spio, PIN_LINK_SCK); pio_gpio_init(spio, PIN_LINK_MOSI); pio_gpio_init(spio, PIN_LINK_CS); pio_gpio_init(spio, PIN_LINK_MISO);
        gpio_set_pulls(PIN_LINK_CS, true, false);                        // idle high while the ESP32 is in reset
        pio_sm_set_consecutive_pindirs(spio, (uint)ssm, PIN_LINK_SCK, 2, false);   // SCK, MOSI in
        pio_sm_set_consecutive_pindirs(spio, (uint)ssm, PIN_LINK_CS, 1, false);    // CS in
        pio_sm_set_consecutive_pindirs(spio, (uint)ssm, PIN_LINK_MISO, 1, true);
        pio_sm_config c = spi_slave_program_get_default_config(soff);
        sm_config_set_out_pins(&c, PIN_LINK_MISO, 1);
        sm_config_set_in_pins(&c, PIN_LINK_MOSI);
        sm_config_set_jmp_pin(&c, PIN_LINK_CS);
        sm_config_set_out_shift(&c, false, true, 8);
        sm_config_set_in_shift(&c, false, true, 8);
        sm_config_set_clkdiv(&c, 1.0f);
        pio_sm_init(spio, (uint)ssm, soff, &c);
        rx_rd = rx_wr_sw = 0; tx_rd = tx_wr = 0; tx_inflight = 0; rx_synced = false; n = 0; have_pending = false;
        st.tx_seq = 1; st.rx_seq = 0; st.resync++;
        mode = LINK_SPI;
        spi_arm();
    } else {
        if (mode == LINK_UART) return;
        pio_sm_set_enabled(spio, (uint)ssm, false); dma_channel_abort(dma_srx); dma_channel_abort(dma_stx);
        gpio_set_function(PIN_UART_RX, GPIO_FUNC_UART); gpio_set_function(PIN_UART_TX, GPIO_FUNC_UART_AUX);
        gpio_init(PIN_LINK_CS); gpio_set_dir(PIN_LINK_CS, GPIO_IN); gpio_pull_up(PIN_LINK_CS);   // ESP IO9 (strap, its pull-up is weak)
        gpio_init(PIN_LINK_MISO); gpio_put(PIN_LINK_MISO, 1); gpio_set_dir(PIN_LINK_MISO, GPIO_OUT);   // "2.0 here" for the ESP32's board probe
        rx_rd = 0; n = 0; have_pending = false; rx_dma_start();
        mode = LINK_UART; cs_seen = false;
    }
}
void link_dl_begin(void) { dl_active = true; link_set_mode(LINK_UART); }
void link_dl_end(void) { dl_active = false; cs_seen = false; }

static void commit_board2(void) {
    if (committed) return;
    committed = true; detected = 2;
    printf("link: SPI frames from the ESP32: board 2.0\n");
    led_disable();                                                       // GPIO45 becomes the audio output
    if (settings.board_seen != 2) { settings.board_seen = 2; settings_save(); }
    extern bool audio_init(void); audio_init();
}
void link_note_esp_reset(void) { esp_reset_ms = to_ms_since_boot(get_absolute_time()) | 1; }
void link_task(void) {
    if (dl_active) return;
    uint32_t now = to_ms_since_boot(get_absolute_time());
    if (mode == LINK_UART && cs_seen) {
        cs_seen = false;
        if (settings.board == 1) return;
        if (esp_reset_ms && now - esp_reset_ms < 3000) return;         // IO9 floats while the ESP32 restarts: not a CS
        link_set_mode(LINK_SPI); spi_since_ms = now | 1;                // trial: back to UART in 3 s unless frames arrive
    } else if (mode == LINK_SPI) {
        bool have_frame = st.last_ok_ms && (int32_t)(st.last_ok_ms - spi_since_ms) >= 0;
        if (have_frame && !committed) commit_board2();
        if (!have_frame && now - spi_since_ms > 3000) { link_set_mode(LINK_UART); }            // false alarm (1.0) or the ESP32 is not talking SPI yet
        else if (have_frame && now - st.last_ok_ms > 5000) { printf("link: no SPI frame for 5 s: back to UART\n"); link_set_mode(LINK_UART); }
    }
}

void link_init(void) {
    n = 0; have_pending = false; rx_rd = 0;
    dma_rx = dma_claim_unused_channel(true); dma_tx = dma_claim_unused_channel(true);
    uart_set_fifo_enabled(LUART, true);
    hw_set_bits(&uart_get_hw(LUART)->dmacr, UART_UARTDMACR_RXDMAE_BITS | UART_UARTDMACR_TXDMAE_BITS);
    rx_dma_start();
    gpio_init(PIN_LINK_MISO); gpio_put(PIN_LINK_MISO, 1); gpio_set_dir(PIN_LINK_MISO, GPIO_OUT);
    gpio_set_irq_enabled_with_callback(PIN_LINK_CS, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, cs_irq);
    gpio_pull_up(PIN_LINK_CS);                                           // the ESP32's own pull-up is weak and gone while it is held in reset
    if (board_kind() == 2) { committed = true; link_set_mode(LINK_SPI); spi_since_ms = to_ms_since_boot(get_absolute_time()) | 1; }
}

static uint32_t last_rx_ms;
uint32_t link_last_rx_ms(void) { return last_rx_ms; }
static char *poll_line(void) {
    while (rx_avail()) {
        char c = (char)rx_get();
        if (c == '\r') continue;
        if (c == '\n') { line[n] = 0; n = 0; if (line[0]) { last_rx_ms = to_ms_since_boot(get_absolute_time()); return line; } continue; }
        if (n < sizeof line - 1 && c >= 32 && c < 127) line[n++] = c;
    }
    return NULL;
}

char *link_poll(void) {
    if (sending) return NULL;                       // commands wait until the binary reply is out
    if (have_pending) { have_pending = false; strcpy(line, pending); return line; }
    char *l = poll_line();
    if (l && l[0] == '!') return NULL;
    return l;
}

// ---- output ----
static void tx_put(const uint8_t *p, uint32_t len) {
    while (len) {
        uint32_t room = TX_RING - (tx_wr - tx_rd);
        if (!room) {
            // the interrupt drains the ring one frame at a time; if the ESP32 stopped polling, give up after 2 s
            uint32_t now = to_ms_since_boot(get_absolute_time());
            if (!st.last_ok_ms || now - st.last_ok_ms > 2000) return;
            watchdog_update(); continue;
        }
        uint32_t k = len < room ? len : room;
        for (uint32_t i = 0; i < k; i++) tx_ring[(tx_wr + i) & (TX_RING - 1)] = p[i];
        __dmb(); tx_wr += k; p += k; len -= k;
    }
}
static void link_out_chars(const char *buf, int len) {
    if (mode == LINK_SPI) tx_put((const uint8_t *)buf, (uint32_t)len);
    else uart_write_blocking(LUART, (const uint8_t *)buf, len);
}
static void raw_puts(const char *s) { link_out_chars(s, (int)strlen(s)); }

void link_begin(void) { stdio_set_driver_enabled(&stdio_usb, false); stdio_set_driver_enabled(&link_driver, true); }
static void end_now(void) { printf("\x1e\n"); stdio_set_driver_enabled(&link_driver, false); stdio_set_driver_enabled(&stdio_usb, true); }
void link_end(void) { if (sending) end_deferred = true; else end_now(); }

bool link_request(const char *req, char *reply, size_t rn, unsigned timeout_ms) {
    if (sending) return false;
    raw_puts("?"); raw_puts(req); raw_puts("\n");
    absolute_time_t end = make_timeout_time_ms(timeout_ms);
    while (!time_reached(end)) {
        char *l = poll_line();
        if (!l) continue;
        if (l[0] == '!') { strncpy(reply, l + 1, rn - 1); reply[rn - 1] = 0; return true; }
        if (!have_pending) { strcpy(pending, l); have_pending = true; }
    }
    return false;
}

void link_set_baud(uint32_t baud) { if (mode != LINK_UART) return; uart_tx_wait_blocking(LUART); sleep_ms(2); uart_set_baudrate(LUART, baud); cur_baud = baud; }

bool link_send_pieces(const link_piece_t *p, unsigned count) {
    if (sending || count > MAX_PIECES) return false;
    memcpy(pieces, p, count * sizeof *p); piece_n = count; piece_i = 0; piece_off = 0; sending = true; end_deferred = false;
    if (mode == LINK_UART) uart_tx_wait_blocking(LUART);
    return true;
}
bool link_sending(void) { return sending; }

void link_pump(void) {
    if (!sending) return;
    if (mode == LINK_SPI) {
        // copy as much as fits into the ring; the CS interrupt sends it
        while (piece_i < piece_n) {
            uint32_t room = TX_RING - (tx_wr - tx_rd); if (!room) return;
            const link_piece_t *pc = &pieces[piece_i];
            uint32_t left = pc->len - piece_off, k = left < room ? left : room;
            tx_put((const uint8_t *)pc->p + piece_off, k); piece_off += k;
            if (piece_off >= pc->len) { piece_i++; piece_off = 0; }
        }
        sending = false;
        if (end_deferred) { end_deferred = false; end_now(); }
        return;
    }
    if (dma_channel_is_busy(dma_tx)) return;
    if (piece_i >= piece_n) {
        uart_tx_wait_blocking(LUART);
        sending = false;
        if (end_deferred) { end_deferred = false; end_now(); }
        return;
    }
    dma_channel_config c = dma_channel_get_default_config(dma_tx);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_dreq(&c, uart_get_dreq(LUART, true));
    dma_channel_configure(dma_tx, &c, &uart_get_hw(LUART)->dr, pieces[piece_i].p, pieces[piece_i].len, true);
    piece_i++;
}

int link_rx_getc(void) { return rx_avail() ? rx_get() : -1; }
// Diagnostics: UART receive status (sticky error flags) and ring fill, or the SPI frame counters.
void link_stat_print(void) {
    if (mode == LINK_SPI) {
        printf("spi frames=%lu bad_hdr=%lu bad_pay=%lu drop=%lu resync=%lu tx=%lu rx=%lu ok_ago=%lu ms seq tx=%u rx=%u inflight=%lu ring tx=%lu rx=%lu\n",
               (unsigned long)st.frames, (unsigned long)st.bad_hdr, (unsigned long)st.bad_pay, (unsigned long)st.rx_drop, (unsigned long)st.resync, (unsigned long)st.tx_bytes, (unsigned long)st.rx_bytes,
               (unsigned long)(to_ms_since_boot(get_absolute_time()) - st.last_ok_ms), st.tx_seq, st.rx_seq, (unsigned long)tx_inflight, (unsigned long)(tx_wr - tx_rd), (unsigned long)(rx_wr_sw - rx_rd));
        return;
    }
    uint32_t rsr = uart_get_hw(LUART)->rsr, wr = rx_wr(), avail = (wr - (rx_rd & (RX_RING - 1))) & (RX_RING - 1);
    printf("uart rsr=%02lx (OE=%lu BE=%lu PE=%lu FE=%lu) ring wr=%lu rd=%lu avail=%lu baud=%lu\n", (unsigned long)rsr, (rsr >> 3) & 1, (rsr >> 2) & 1, (rsr >> 1) & 1, rsr & 1, (unsigned long)wr, (unsigned long)(rx_rd & (RX_RING - 1)), (unsigned long)avail, (unsigned long)cur_baud);
    uart_get_hw(LUART)->rsr = 0xF;   // clear
}
bool link_rx_read(uint8_t *dst, uint32_t cnt, unsigned timeout_ms) {
    absolute_time_t end = make_timeout_time_ms(timeout_ms);
    for (uint32_t i = 0; i < cnt; ) {
        if (rx_avail()) { dst[i++] = rx_get(); end = make_timeout_time_ms(timeout_ms); }
        else if (time_reached(end)) return false;
        else watchdog_update();
    }
    return true;
}
