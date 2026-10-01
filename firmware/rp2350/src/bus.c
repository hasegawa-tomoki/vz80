#include <stdio.h>
#include <string.h>
#include "bus.h"
#include "pins.h"
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "bus.pio.h"

// ---- word encoding for bus_ctrl -------------------------------------------------------
#define EDGE_RISE 1u
#define EDGE_FALL 0u
#define SKIP(n)   ((uint32_t)(n) << 1)
#define ADDR      (1u << 3)
#define SAMPLE    (1u << 4)
#define STATE(s)  ((uint32_t)(s) << 5)
#define WAITCHK   (1u << 12)
#define DATA      (1u << 13)
// STATE bits (1 = inactive): 0 MREQ, 1 IORQ, 2 RD, 3 WR, 4 (WAIT, input), 5 M1, 6 RFSH
#define ST_IDLE   0x7Fu
#define L_MREQ    (1u << 0)
#define L_IORQ    (1u << 1)
#define L_RD      (1u << 2)
#define L_WR      (1u << 3)
#define L_M1      (1u << 5)
#define L_RFSH    (1u << 6)
#define ST(low)   (ST_IDLE & ~(low))

typedef struct { uint32_t w[4]; } script_t;
static const script_t S_MEMRD = {{ EDGE_RISE | ADDR | STATE(ST_IDLE),
                                   EDGE_FALL | STATE(ST(L_MREQ | L_RD)),
                                   EDGE_FALL | STATE(ST(L_MREQ | L_RD)) | WAITCHK,
                                   EDGE_FALL | SAMPLE | STATE(ST_IDLE) }};
static const script_t S_M1    = {{ EDGE_RISE | ADDR | STATE(ST(L_M1)),
                                   EDGE_FALL | STATE(ST(L_M1 | L_MREQ | L_RD)),
                                   EDGE_FALL | STATE(ST(L_M1 | L_MREQ | L_RD)) | WAITCHK,
                                   EDGE_FALL | SAMPLE | STATE(ST_IDLE) }};
static const script_t S_MEMWR = {{ EDGE_RISE | ADDR | STATE(ST_IDLE),
                                   EDGE_FALL | STATE(ST(L_MREQ)) | DATA,
                                   EDGE_FALL | STATE(ST(L_MREQ | L_WR)) | WAITCHK,
                                   EDGE_FALL | STATE(ST_IDLE) | DATA }};
static const script_t S_IORD  = {{ EDGE_RISE | ADDR | STATE(ST_IDLE),
                                   EDGE_RISE | STATE(ST(L_IORQ | L_RD)),
                                   EDGE_FALL | SKIP(1) | STATE(ST(L_IORQ | L_RD)) | WAITCHK,
                                   EDGE_FALL | SAMPLE | STATE(ST_IDLE) }};
static const script_t S_IOWR  = {{ EDGE_RISE | ADDR | STATE(ST_IDLE) | DATA,
                                   EDGE_RISE | STATE(ST(L_IORQ | L_WR)),
                                   EDGE_FALL | SKIP(1) | STATE(ST(L_IORQ | L_WR)) | WAITCHK,
                                   EDGE_FALL | STATE(ST_IDLE) | DATA }};
static const script_t S_RFSH  = {{ EDGE_RISE | ADDR | STATE(ST(L_RFSH)),
                                   EDGE_FALL | STATE(ST(L_RFSH | L_MREQ)),
                                   EDGE_FALL | STATE(ST(L_RFSH)),
                                   EDGE_RISE | STATE(ST_IDLE) }};
static const script_t S_ACK   = {{ EDGE_RISE | ADDR | STATE(ST(L_M1)),
                                   EDGE_RISE | SKIP(1) | STATE(ST(L_M1 | L_IORQ)),
                                   EDGE_FALL | SKIP(1) | STATE(ST(L_M1 | L_IORQ)) | WAITCHK,
                                   EDGE_FALL | SAMPLE | STATE(ST_IDLE) }};

// ---- state ----------------------------------------------------------------------------
#define PIO_CTRL pio0
#define PIO_DATA pio1
#define PIO_CAP  pio2
static uint sm_ctrl, sm_addr, sm_data, sm_cap, sm_clk;
static uint off_ctrl, off_addr, off_data, off_cap, off_clk;
static bool enabled, faulted, bench_clk, bench_mode;
static int dma_cap = -1;

#define TIMEOUT_US 2000
#define SPIN_LIMIT 400000u        // ~2 ms of polling at 150 MHz

static void engine_reset(void);
static void fault(void) { faulted = true; engine_reset(); }

// Wait until the SM's TX FIFO has at least `slots` free entries. FIFO depth: ctrl 8 (joined), others 4.
static inline bool tx_room(PIO p, uint sm, uint depth, uint slots) {
    for (uint32_t i = 0; i < SPIN_LIMIT; i++) if (pio_sm_get_tx_fifo_level(p, sm) + slots <= depth) return true;
    return false;
}

static inline bool post(const script_t *s, uint16_t addr, bool has_data, uint32_t dataword) {
    if (faulted) return false;
    if (!tx_room(PIO_CTRL, sm_addr, 4, 1) || (has_data && !tx_room(PIO_DATA, sm_data, 4, 1)) || !tx_room(PIO_CTRL, sm_ctrl, 8, 4)) { fault(); return false; }
    PIO_CTRL->txf[sm_addr] = addr;
    if (has_data) PIO_DATA->txf[sm_data] = dataword;
    io_wo_32 *tx = &PIO_CTRL->txf[sm_ctrl];
    *tx = s->w[0]; *tx = s->w[1]; *tx = s->w[2]; *tx = s->w[3];
    return true;
}

static inline uint8_t read_result(void) {
    for (uint32_t i = 0; pio_sm_is_rx_fifo_empty(PIO_DATA, sm_data); i++) if (i > SPIN_LIMIT) { fault(); return 0xFF; }
    return (uint8_t)pio_sm_get(PIO_DATA, sm_data);   // ISR shifts left, autopush at 8: byte in bits 0..7
}

uint8_t __not_in_flash_func(bus_mem_read)(uint16_t a)  { return post(&S_MEMRD, a, true, 0) ? read_result() : 0xFF; }
uint8_t __not_in_flash_func(bus_m1)(uint16_t a)        { return post(&S_M1, a, true, 0) ? read_result() : 0xFF; }
uint8_t __not_in_flash_func(bus_io_read)(uint16_t a)   { return post(&S_IORD, a, true, 0) ? read_result() : 0xFF; }
uint8_t __not_in_flash_func(bus_int_ack)(uint16_t pc)  { return post(&S_ACK, pc, true, 0) ? read_result() : 0xFF; }
void __not_in_flash_func(bus_mem_write)(uint16_t a, uint8_t d) { post(&S_MEMWR, a, true, 1u | ((uint32_t)d << 1)); }
void __not_in_flash_func(bus_io_write)(uint16_t a, uint8_t d)  { post(&S_IOWR, a, true, 1u | ((uint32_t)d << 1)); }
void __not_in_flash_func(bus_refresh)(uint8_t r)               { post(&S_RFSH, r, false, 0); }

void bus_drain(void) {
    absolute_time_t end = make_timeout_time_us(TIMEOUT_US * 4);
    while (!pio_sm_is_tx_fifo_empty(PIO_CTRL, sm_ctrl) || !pio_sm_is_tx_fifo_empty(PIO_CTRL, sm_addr) ||
           !pio_sm_is_tx_fifo_empty(PIO_DATA, sm_data)) if (time_reached(end)) { fault(); return; }
    sleep_us(2);   // last word's trailing edge
}

bool bus_faulted(void) { return faulted; }
void bus_clear_fault(void) { faulted = false; }
bool bus_enabled(void) { return enabled; }

// ---- setup ----------------------------------------------------------------------------
static void configure_sms(void) {
    // ctrl: OUT base 25 count 7, JMP pin = WAIT, TX-joined FIFO (8 deep)
    pio_sm_config c = bus_ctrl_program_get_default_config(off_ctrl);
    sm_config_set_out_pins(&c, PIN_MMREQ, 7);
    sm_config_set_out_shift(&c, true, false, 32);
    sm_config_set_jmp_pin(&c, PIN_WAIT);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
    pio_sm_init(PIO_CTRL, sm_ctrl, off_ctrl, &c);
    // addr: OUT base 0 count 16
    pio_sm_config a = bus_addr_program_get_default_config(off_addr);
    sm_config_set_out_pins(&a, PIN_MA_BASE, 16);
    sm_config_set_out_shift(&a, true, false, 32);
    sm_config_set_fifo_join(&a, PIO_FIFO_JOIN_TX);
    pio_sm_init(PIO_CTRL, sm_addr, off_addr, &a);
    // data: OUT/IN base 16 count 8, SET base 38 count 2 (DIR, OE), autopush 8
    pio_sm_config d = bus_data_program_get_default_config(off_data);
    sm_config_set_out_pins(&d, PIN_MD_BASE, 8);
    sm_config_set_in_pins(&d, PIN_MD_BASE);
    sm_config_set_set_pins(&d, PIN_DATA_DIR, 1);
    sm_config_set_out_shift(&d, true, false, 32);
    sm_config_set_in_shift(&d, false, true, 8);
    pio_sm_init(PIO_DATA, sm_data, off_data, &d);
}

static void engine_reset(void) {
    pio_sm_set_enabled(PIO_CTRL, sm_ctrl, false); pio_sm_set_enabled(PIO_CTRL, sm_addr, false); pio_sm_set_enabled(PIO_DATA, sm_data, false);
    pio_sm_clear_fifos(PIO_CTRL, sm_ctrl); pio_sm_clear_fifos(PIO_CTRL, sm_addr); pio_sm_clear_fifos(PIO_DATA, sm_data);
    pio_interrupt_clear(PIO_CTRL, 0); pio_interrupt_clear(PIO_DATA, 1); pio_interrupt_clear(PIO_DATA, 2);
    configure_sms();
    // strobes idle, data released, DIR=0
    pio_sm_set_pins_with_mask64(PIO_CTRL, sm_ctrl, 0x7Full << PIN_MMREQ, 0x7Full << PIN_MMREQ);
    pio_sm_set_pindirs_with_mask64(PIO_DATA, sm_data, 0, 0xFFull << PIN_MD_BASE);
    pio_sm_set_pins_with_mask64(PIO_DATA, sm_data, 0, 1ull << PIN_DATA_DIR);
    if (enabled) { pio_sm_set_enabled(PIO_DATA, sm_data, true); pio_sm_set_enabled(PIO_CTRL, sm_addr, true); pio_sm_set_enabled(PIO_CTRL, sm_ctrl, true); }
}

void bus_init(void) {
    pio_set_gpio_base(PIO_CTRL, 0);
    pio_set_gpio_base(PIO_DATA, PIO_DATA_GPIOBASE);
    pio_set_gpio_base(PIO_CAP, 0);
    off_ctrl = pio_add_program(PIO_CTRL, &bus_ctrl_program);
    off_addr = pio_add_program(PIO_CTRL, &bus_addr_program);
    off_data = pio_add_program(PIO_DATA, &bus_data_program);
    off_cap  = pio_add_program(PIO_CAP, &bus_capture_program);
    sm_ctrl = pio_claim_unused_sm(PIO_CTRL, true);
    sm_addr = pio_claim_unused_sm(PIO_CTRL, true);
    sm_data = pio_claim_unused_sm(PIO_DATA, true);
    sm_cap  = pio_claim_unused_sm(PIO_CAP, true);
    dma_cap = dma_claim_unused_channel(true);
    configure_sms();
}

void bus_enable(bool on) {
    if (on == enabled) return;
    if (on) {
        // Output pins to PIO with idle levels preloaded; inputs stay SIO inputs.
        pio_sm_set_pins_with_mask64(PIO_CTRL, sm_ctrl, 0x7Full << PIN_MMREQ, 0x7Full << PIN_MMREQ);
        pio_sm_set_pindirs_with_mask64(PIO_CTRL, sm_ctrl, (uint64_t)MASK_STROBES, (uint64_t)MASK_STROBES);
        pio_sm_set_pindirs_with_mask64(PIO_CTRL, sm_addr, (uint64_t)MASK_MA, (uint64_t)MASK_MA);
        pio_sm_set_pindirs_with_mask64(PIO_DATA, sm_data, 0, (uint64_t)MASK_MD);
        pio_sm_set_pins_with_mask64(PIO_DATA, sm_data, 0, 1ull << PIN_DATA_DIR);
        pio_sm_set_pindirs_with_mask64(PIO_DATA, sm_data, 1ull << PIN_DATA_DIR, 1ull << PIN_DATA_DIR);
        for (uint p = PIN_MA0; p < PIN_MA0 + 16; p++) pio_gpio_init(PIO_CTRL, p);
        for (uint p = PIN_MMREQ; p <= PIN_MRFSH; p++) if (p != PIN_WAIT) pio_gpio_init(PIO_CTRL, p);
        for (uint p = PIN_MD0; p < PIN_MD0 + 8; p++) pio_gpio_init(PIO_DATA, p);
        pio_gpio_init(PIO_DATA, PIN_DATA_DIR);
        enabled = true; faulted = false;
        engine_reset();
        if (!bench_mode) { gpio_put(PIN_DATA_OE, 0); gpio_put(PIN_BUS_EN, 0); }   // buffers on (never on the bench)
    } else {
        gpio_put(PIN_BUS_EN, 1); gpio_put(PIN_DATA_OE, 1);
        enabled = false;
        engine_reset();
        for (uint p = PIN_MA0; p <= PIN_MRFSH; p++) { gpio_init(p); gpio_disable_pulls(p); }
        gpio_put(PIN_MHALT, 1); gpio_set_dir(PIN_MHALT, GPIO_OUT); gpio_put(PIN_MBUSAK, 1); gpio_set_dir(PIN_MBUSAK, GPIO_OUT);
        gpio_init(PIN_DATA_DIR); gpio_put(PIN_DATA_DIR, 0); gpio_set_dir(PIN_DATA_DIR, GPIO_OUT);
    }
}

// ---- bench helpers --------------------------------------------------------------------
void bus_bench_clock(bool on) {
    bench_mode = on;               // bench clock implies a board that is not in an MSX: keep the 5 V buffers off
    if (on == bench_clk) return;
    if (on) {
        // 3.57 MHz: 21 cycles high, 21 low at 150 MHz.
        static uint16_t prog[2];
        prog[0] = pio_encode_set(pio_pins, 1) | pio_encode_delay(20); prog[1] = pio_encode_set(pio_pins, 0) | pio_encode_delay(20);
        static struct pio_program p = { .instructions = prog, .length = 2, .origin = -1, .pio_version = 0 };
        off_clk = pio_add_program(PIO_CAP, &p);
        sm_clk = pio_claim_unused_sm(PIO_CAP, true);
        pio_sm_config c = pio_get_default_sm_config();
        sm_config_set_wrap(&c, off_clk, off_clk + 1);
        sm_config_set_set_pins(&c, PIN_CLK, 1);
        sm_config_set_clkdiv(&c, (float)clock_get_hz(clk_sys) / 150000000.0f);
        pio_sm_set_consecutive_pindirs(PIO_CAP, sm_clk, PIN_CLK, 1, true);
        pio_gpio_init(PIO_CAP, PIN_CLK);
        pio_sm_init(PIO_CAP, sm_clk, off_clk, &c);
        pio_sm_set_enabled(PIO_CAP, sm_clk, true);
        gpio_pull_up(PIN_WAIT);            // no MSX: /WAIT inactive
    } else {
        pio_sm_set_enabled(PIO_CAP, sm_clk, false);
        pio_sm_unclaim(PIO_CAP, sm_clk);
        gpio_init(PIN_CLK); gpio_disable_pulls(PIN_CLK); gpio_disable_pulls(PIN_WAIT);
    }
    bench_clk = on;
}

void bus_bench_data(uint8_t pattern) {
    for (int i = 0; i < 8; i++) { uint p = PIN_MD0 + i; if (pattern >> i & 1) gpio_pull_up(p); else gpio_pull_down(p); }
}

static uint32_t cap_n; static uint32_t cap_div = 2;
void bus_capture_div(uint32_t d) { cap_div = d < 1 ? 1 : d; }
void bus_capture_start(uint32_t *buf, uint32_t n, uint32_t *sample_hz) {
    pio_sm_config c = bus_capture_program_get_default_config(off_cap);
    sm_config_set_in_pins(&c, 0);
    sm_config_set_in_shift(&c, false, true, 32);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);
    sm_config_set_clkdiv_int_frac(&c, cap_div, 0);
    *sample_hz = clock_get_hz(clk_sys) / cap_div;
    pio_sm_set_enabled(PIO_CAP, sm_cap, false);
    pio_sm_clear_fifos(PIO_CAP, sm_cap);
    pio_sm_init(PIO_CAP, sm_cap, off_cap, &c);
    dma_channel_config dc = dma_channel_get_default_config(dma_cap);
    channel_config_set_read_increment(&dc, false);
    channel_config_set_write_increment(&dc, true);
    channel_config_set_dreq(&dc, pio_get_dreq(PIO_CAP, sm_cap, false));
    dma_channel_configure(dma_cap, &dc, buf, &PIO_CAP->rxf[sm_cap], n, true);
    cap_n = n;
    pio_sm_set_enabled(PIO_CAP, sm_cap, true);
}
uint32_t bus_capture_stop(void) {
    absolute_time_t end = make_timeout_time_ms(50);
    while (dma_channel_is_busy(dma_cap) && !time_reached(end)) tight_loop_contents();
    pio_sm_set_enabled(PIO_CAP, sm_cap, false);
    uint32_t got = cap_n - dma_channel_hw_addr(dma_cap)->transfer_count;
    dma_channel_abort(dma_cap);
    return got;
}

