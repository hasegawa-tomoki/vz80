#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "console.h"
#include "diag.h"
#include "led.h"
#include "psram.h"
#include "vdpcmd.h"
#include "slot.h"
static char raw_args[256];   // the command line after the command name, untouched (for `keys`)
#include "pico/multicore.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "hardware/structs/qmi.h"
#include "hardware/xip_cache.h"
#include "hardware/watchdog.h"
#include "hardware/structs/qmi.h"
#include "hardware/structs/xip_ctrl.h"
#include "hardware/structs/watchdog.h"
#include "esp.h"
#include "link.h"
#include "audio.h"
#include "bus.h"
#include "msx.h"
#include "vdp.h"
#include "kbd.h"
#include "crash.h"
#include "vzdisk.h"
#include "hardware/watchdog.h"
#include "z80_mem.h"
#include "fwup.h"
#include "shelf.h"
#include "cart.h"
#include "fdc.h"
#include "settings.h"
#include "hardware/clocks.h"
#include "pico/stdlib.h"
#include "pico/bootrom.h"
#include "hardware/watchdog.h"

typedef struct { const char *name; const char *help; void (*fn)(int argc, char **argv); } cmd_t;
static uint32_t link_end_then_baud;
static const cmd_t cmds[];

static void c_help(int argc, char **argv) { for (const cmd_t *c = cmds; c->name; c++) printf("  %-8s %s\n", c->name, c->help); }
static void c_info(int argc, char **argv) { diag_info(); }
static void c_led(int argc, char **argv) {
    // led | led auto | led R G B (0-255, held until `led auto`)
    if (argc < 2) { printf("%s\n", led_state_str()); return; }
    if (!strcmp(argv[1], "auto")) { led_auto(); printf("ok\n"); return; }
    if (argc < 4) { printf("usage: led [auto | R G B]\n"); return; }
    led_set(atoi(argv[1]), atoi(argv[2]), atoi(argv[3])); printf("ok\n");
}
static void c_locate(int argc, char **argv) { led_locate(argc > 1 ? strtoul(argv[1], NULL, 10) : 10000); printf("ok\n"); }
static void c_espstate(int argc, char **argv) {   // from the ESP32 every few seconds: espstate up|joining|unset
    if (argc < 2) { printf("err\n"); return; }
    led_set_wifi(!strcmp(argv[1], "up") ? LED_WIFI_UP : !strcmp(argv[1], "joining") ? LED_WIFI_JOINING : LED_WIFI_UNSET);
    printf("ok\n");
}
static void c_sram(int argc, char **argv) { printf(diag_sram() ? "ok\n" : "FAIL\n"); }
static void c_flash(int argc, char **argv) { diag_flash(); }
static void c_psram(int argc, char **argv) {
    // psram [bytes] [nocache] | psram init CLKDIV
    if (argc >= 3 && !strcmp(argv[1], "init")) { psram_init(atoi(argv[2])); }
    uint32_t bytes = (argc >= 2 && strcmp(argv[1], "init")) ? strtoul(argv[1], NULL, 0) : 65536;
    bool nocache = argc >= 3 && !strcmp(argv[argc-1], "nocache");
    printf(diag_psram(bytes, nocache) ? "ok\n" : "FAIL\n");
}
static void c_esp(int argc, char **argv) {
    // esp reset | esp dl | esp log | esp bridge [idle_ms]
    if (argc < 2) { printf("usage: esp reset|dl|log|bridge [idle_ms]\n"); return; }
    if (!strcmp(argv[1], "reset")) { esp_reset(false); esp_dump_log(1500); }
    else if (!strcmp(argv[1], "dl")) { link_set_baud(115200); esp_reset(true); esp_dump_log(1500); esp_strap_release(); }   // the ESP ROM loader talks 115200
    else if (!strcmp(argv[1], "log")) esp_dump_log(argc > 2 ? atoi(argv[2]) : 500);
    else if (!strcmp(argv[1], "bridge")) { printf("bridge open\n"); esp_bridge(argc > 2 ? atoi(argv[2]) : 15000); }
    else if (!strcmp(argv[1], "reboot")) { char r[64]; printf(link_request("reboot", r, sizeof r, 1500) ? "ok\n" : "sent\n"); }
    else printf("err\n");
}
static void c_espstatus(int argc, char **argv) {
    char r[200];
    if (link_request("status", r, sizeof r, 2000)) printf("%s\n", r); else printf("err esp no reply\n");
}
static void c_wifi(int argc, char **argv) {
    // wifi SSID PASS | wifi clear
    char req[160], r[200];
    if (argc == 2 && !strcmp(argv[1], "clear")) snprintf(req, sizeof req, "wifi");
    else if (argc == 3) snprintf(req, sizeof req, "wifi %s %s", argv[1], argv[2]);
    else { printf("usage: wifi SSID PASS | wifi clear\n"); return; }
    if (link_request(req, r, sizeof r, 3000)) printf("%s\n", r); else printf("err esp no reply\n");
}
static void c_pins(int argc, char **argv) {
    // pins [MS]: sample GPIO32..39 continuously for MS milliseconds; report low counts and longest low run (us)
    uint32_t ms = argc > 1 ? strtoul(argv[1], NULL, 10) : 1000;
    uint32_t low[8] = {0}, run[8] = {0}, maxrun[8] = {0}, total = 0;
    absolute_time_t end = make_timeout_time_ms(ms);
    while (!time_reached(end)) {
        uint32_t hi = (uint32_t)(gpio_get_all64() >> 32);
        for (int b = 0; b < 8; b++) { if (!(hi >> b & 1)) { low[b]++; run[b]++; if (run[b] > maxrun[b]) maxrun[b] = run[b]; } else run[b] = 0; }
        total++;
    }
    static const char *nm[8] = {"NMI","INT","HALT","BUSAK","BUSRQ","RESET","DATA_DIR","DATA_OE"};
    printf("pins: %lu samples in %lu ms (%.2f us each)\n", (unsigned long)total, (unsigned long)ms, ms * 1000.0 / total);
    for (int b = 0; b < 8; b++) printf("  %-8s low %lu (%.3f%%) longest low run %.1f us\n", nm[b], (unsigned long)low[b], 100.0 * low[b] / total, maxrun[b] * ms * 1000.0 / total);
}
static void c_gpio(int argc, char **argv) { diag_gpio(); }
static void c_reboot(int argc, char **argv) { printf("rebooting\n"); msx_request_start_after_reboot(); sleep_ms(50); crash_mark_planned_reboot(); watchdog_reboot(0, 0, 0); }
static void c_bootsel(int argc, char **argv) { printf("entering BOOTSEL\n"); sleep_ms(50); rom_reset_usb_boot(0, 0); }


// ---- bus bench ------------------------------------------------------------------------
static uint32_t capbuf[2048];
static const char *sig[] = {"CLK","MREQ","IORQ","RD","WR","WAIT","M1","RFSH"};
static void print_capture(uint32_t n, uint32_t hz) {
    // Print every change of A/D/control with a timestamp (ns) and CLK rising-edge count, relative to the first strobe activity.
    int start = -1;
    for (uint32_t i = 0; i < n; i++) { if ((~capbuf[i] >> 25) & 0x6F) { start = (int)i; break; } }   // any strobe low (ignore WAIT bit 29)
    if (start < 0) { printf("capture: no bus activity in %lu samples\n", (unsigned long)n); return; }
    uint32_t prev = capbuf[0], lines = 0; int clks = 0;
    for (uint32_t i = 1; i < n && lines < 60; i++) {
        uint32_t v = capbuf[i], ch = v ^ prev;
        if ((ch & (1u << 24)) && (v & (1u << 24))) clks++;
        if (ch & ~(1u << 24)) {
            int64_t t = ((int64_t)i - start) * 1000000000LL / hz;
            printf("%7lld ns clk%3d  A=%04lx D=%02lx", (long long)t, clks, v & 0xFFFF, (v >> 16) & 0xFF);
            for (int b = 0; b < 8; b++) printf(" %s=%lu", sig[b], (v >> (24 + b)) & 1);
            printf("\n"); lines++;
        }
        prev = v;
    }
    printf("clk rising edges in capture: %d over %lu samples\n", clks, (unsigned long)n);
}
static void c_bus(int argc, char **argv) {
    // bus on|off|clk on|off|data XX|rd A|wr A V|in P|out P V|m1 A|rfsh R|ack|cap TYPE [A [V]]|test N|drain
    if (argc < 2) { printf("usage: bus on|off|clk on|off|data XX|rd A|wr A V|in P|out P V|m1 A|rfsh R|ack|cap TYPE [A [V]]|test N|status\n"); return; }
    const char *op = argv[1];
    uint32_t a = argc > 2 ? strtoul(argv[2], NULL, 16) : 0, v = argc > 3 ? strtoul(argv[3], NULL, 16) : 0;
    if (!strcmp(op, "on")) { bus_enable(true); printf("ok\n"); }
    else if (!strcmp(op, "off")) { bus_enable(false); printf("ok\n"); }
    else if (!strcmp(op, "status")) printf("enabled=%d faulted=%d\n", bus_enabled(), bus_faulted());
    else if (!strcmp(op, "clk")) { bus_bench_clock(argc > 2 && !strcmp(argv[2], "on")); printf("ok\n"); }
    else if (!strcmp(op, "data")) { bus_bench_data(a); printf("ok\n"); }
    else if (!strcmp(op, "rd")) printf("%02x%s\n", bus_mem_read(a), bus_faulted() ? " FAULT" : "");
    else if (!strcmp(op, "m1")) printf("%02x%s\n", bus_m1(a), bus_faulted() ? " FAULT" : "");
    else if (!strcmp(op, "in")) printf("%02x%s\n", bus_io_read(a), bus_faulted() ? " FAULT" : "");
    else if (!strcmp(op, "ack")) printf("%02x%s\n", bus_int_ack(a), bus_faulted() ? " FAULT" : "");
    else if (!strcmp(op, "wr")) { bus_mem_write(a, v); bus_drain(); printf(bus_faulted() ? "FAULT\n" : "ok\n"); }
    else if (!strcmp(op, "out")) { bus_io_write(a, v); bus_drain(); printf(bus_faulted() ? "FAULT\n" : "ok\n"); }
    else if (!strcmp(op, "rfsh")) { bus_refresh(a); bus_drain(); printf(bus_faulted() ? "FAULT\n" : "ok\n"); }
    else if (!strcmp(op, "drain")) { bus_drain(); printf("ok\n"); }
    else if (!strcmp(op, "clear")) { bus_clear_fault(); printf("ok\n"); }
    else if (!strcmp(op, "cap") && argc > 2) {
        uint32_t hz; const char *t = argv[2]; uint32_t addr = argc > 3 ? strtoul(argv[3], NULL, 16) : 0x1234, val = argc > 4 ? strtoul(argv[4], NULL, 16) : 0x5A;
        bus_capture_start(capbuf, 2048, &hz);
        uint8_t r = 0; bool has_r = false;
        if (!strcmp(t, "rd3")) { r = bus_mem_read(addr); r += bus_mem_read(addr + 1); r += bus_mem_read(addr + 2); has_r = true; }
        else if (!strcmp(t, "wr3")) { bus_mem_write(addr, val); bus_mem_write(addr + 1, val + 1); bus_mem_write(addr + 2, val + 2); }
        else if (!strcmp(t, "rd")) { r = bus_mem_read(addr); has_r = true; }
        else if (!strcmp(t, "m1")) { r = bus_m1(addr); has_r = true; }
        else if (!strcmp(t, "in")) { r = bus_io_read(addr); has_r = true; }
        else if (!strcmp(t, "ack")) { r = bus_int_ack(addr); has_r = true; }
        else if (!strcmp(t, "wr")) bus_mem_write(addr, val);
        else if (!strcmp(t, "out")) bus_io_write(addr, val);
        else if (!strcmp(t, "rfsh")) bus_refresh(addr);
        else { bus_capture_stop(); printf("err type\n"); return; }
        bus_drain();
        uint32_t n = bus_capture_stop();
        printf("capture %lu samples @%lu Hz%s%s\n", (unsigned long)n, (unsigned long)hz, has_r ? " result=" : "", has_r ? "" : "");
        if (has_r) printf("result=%02x\n", r);
        if (bus_faulted()) printf("FAULT\n");
        print_capture(n, hz);
    }
    else if (!strcmp(op, "test")) {
        uint32_t n = argc > 2 ? strtoul(argv[2], NULL, 10) : 1000; if (!n) n = 1;
        struct { const char *name; int kind; } k[] = {{"memrd",0},{"memwr",1},{"iord",2},{"iowr",3},{"m1",4},{"rfsh",5},{"ack",6}};
        for (int i = 0; i < 7; i++) {
            absolute_time_t t0 = get_absolute_time(); uint32_t sum = 0;
            for (uint32_t j = 0; j < n; j++) {
                uint16_t addr = (uint16_t)(j * 0x1357);
                switch (k[i].kind) {
                    case 0: sum += bus_mem_read(addr); break;
                    case 1: bus_mem_write(addr, (uint8_t)j); break;
                    case 2: sum += bus_io_read(addr); break;
                    case 3: bus_io_write(addr, (uint8_t)j); break;
                    case 4: sum += bus_m1(addr); break;
                    case 5: bus_refresh((uint8_t)j); break;
                    case 6: sum += bus_int_ack(addr); break;
                }
                if (bus_faulted()) break;
            }
            bus_drain();
            int64_t us = absolute_time_diff_us(t0, get_absolute_time());
            printf("%-6s %lu x  %.3f us/transfer  sum=%08lx%s\n", k[i].name, (unsigned long)n, (double)us / n, (unsigned long)sum, bus_faulted() ? " FAULT" : "");
            if (bus_faulted()) { bus_clear_fault(); }
        }
    }
    else printf("err\n");
}


// ---- Z80 core bench (no bus): a mixed loop in SRAM ---------------------------------------
static const uint8_t bench_prog[] = {
    0x21, 0x00, 0x80,          // 0000 LD HL,8000h
    0x11, 0x00, 0x90,          // 0003 LD DE,9000h
    0x01, 0x00, 0x01,          // 0006 LD BC,0100h
    0xED, 0xB0,                // 0009 LDIR
    0x3E, 0x00,                // 000B LD A,0
    0x06, 0x40,                // 000D LD B,40h
    0x21, 0x00, 0xA0,          // 000F LD HL,A000h
    0x86,                      // 0012 loop: ADD A,(HL)
    0x23,                      // 0013 INC HL
    0xCB, 0x27,                // 0014 SLA A
    0xDD, 0x21, 0x00, 0xB0,    // 0016 LD IX,B000h
    0xDD, 0x77, 0x05,          // 001A LD (IX+5),A
    0xCD, 0x30, 0x00,          // 001D CALL 0030h
    0x10, 0xF1,                // 0020 DJNZ loop (0012)
    0xC3, 0x00, 0x00,          // 0022 JP 0000h
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,   // pad to 0030
    0xF5,                      // 0030 PUSH AF
    0xE5,                      // 0031 PUSH HL
    0x2A, 0x00, 0xC0,          // 0032 LD HL,(C000h)
    0x23,                      // 0035 INC HL
    0x22, 0x00, 0xC0,          // 0036 LD (C000h),HL
    0xE1,                      // 0039 POP HL
    0xF1,                      // 003A POP AF
    0xC9,                      // 003B RET
};
static void c_z80bench(int argc, char **argv) {
    uint32_t tstates = argc > 1 ? strtoul(argv[1], NULL, 10) : 10000000;
    memset(msx_ram, 0, sizeof msx_ram);
    memcpy(msx_ram, bench_prog, sizeof bench_prog);
    msx_map_flat();
    z80_reset(&cpu); cpu.sp = 0xF000;
    volatile uint8_t stop = 0;
    absolute_time_t t0 = get_absolute_time();
    uint32_t ran = z80_run(&cpu, cpu.cycles + tstates, &stop);
    int64_t us = absolute_time_diff_us(t0, get_absolute_time());
    printf("z80bench: %lu T-states, %lu insns in %lld us: %.2f MT/s, %.2f Minsn/s, %.1f ns/insn, counter=%04x\n",
           (unsigned long)ran, (unsigned long)cpu.insns, (long long)us, ran / (double)us, cpu.insns / (double)us,
           1000.0 * us / cpu.insns, msx_ram[0xC000] | (msx_ram[0xC001] << 8));
    printf("  equivalent Z80 clock: %.2f MHz (real MSX: 3.58)\n", ran / (double)us);
}


// ---- MSX control -------------------------------------------------------------------------
// Passive capture while the CPU runs: decode I/O and memory strobes into an access list.
static void c_watch(int argc, char **argv) {
    // watch [DIV] [io|all]
    uint32_t div = argc > 1 ? strtoul(argv[1], NULL, 10) : 8; bool io_only = !(argc > 2 && !strcmp(argv[2], "all"));
    uint32_t hz; bus_capture_div(div);
    bus_capture_start(capbuf, 2048, &hz);
    for (int k = 0; k < 400; k++) msx_service();
    uint32_t n = bus_capture_stop(); bus_capture_div(2);
    printf("watch: %lu samples @%lu Hz (%.1f us)\n", (unsigned long)n, (unsigned long)hz, n * 1e6 / hz);
    uint32_t prev = capbuf[0]; int lines = 0;
    for (uint32_t i = 1; i < n && lines < 80; i++) {
        uint32_t v = capbuf[i];
        bool mreq = !(v >> 25 & 1), iorq = !(v >> 26 & 1), rd = !(v >> 27 & 1), wr = !(v >> 28 & 1), m1 = !(v >> 30 & 1), rfsh = !(v >> 31 & 1);
        bool pmreq = !(prev >> 25 & 1), piorq = !(prev >> 26 & 1), prd = !(prev >> 27 & 1), pwr = !(prev >> 28 & 1);
        // report at the rising edge of RD/WR (end of the access): data is sampled just before
        if ((prd && !rd) || (pwr && !wr)) {
            bool was_io = piorq, was_mem = pmreq;
            if (io_only && !was_io) { prev = v; continue; }
            printf("%7.2f us %s %s %s%04lx %02lx%s\n", i * 1e6 / hz, was_io ? "IO " : was_mem ? "MEM" : "?? ", prd ? "rd" : "wr",
                   was_io ? "port " : "", was_io ? (prev & 0xFF) : (prev & 0xFFFF), (prev >> 16) & 0xFF, m1 ? " M1" : "");
            lines++;
        }
        (void)rfsh; (void)mreq; (void)iorq;
        prev = v;
    }
}
static void c_iotrace(int argc, char **argv) {
    if (argc > 1 && !strcmp(argv[1], "freeze")) { msx_iotrace_freeze(argc > 2 && atoi(argv[2])); printf("ok\n"); return; }
    if (argc > 1 && !strcmp(argv[1], "trig")) { msx_iotrace_trigger(argc > 2 ? (int)strtoul(argv[2], NULL, 16) : -1, argc > 3 ? (int)strtoul(argv[3], NULL, 16) : -1, argc > 4 ? atoi(argv[4]) : 200); printf("ok\n"); return; }
    if (argc > 1 && !strcmp(argv[1], "skip")) { msx_iotrace_skip(argc > 2 ? strtoul(argv[2], NULL, 16) : 0); printf("ok\n"); return; } msx_iotrace_dump(argc > 1 ? strtoul(argv[1], NULL, 10) : 64, argc > 2 ? strtoul(argv[2], NULL, 10) : 0); }
// vdpstate: JSON with registers and palette. vramdelta full|delta: "bin N" + binary
// [64 regs][16 x u16 palette][u16 count][u16 block, 512 bytes]*count  (blocks of the VRAM shadow).
static void c_vdpstate(int argc, char **argv) {
    printf("{\"regs\":[");
    for (int r = 0; r < 64; r++) printf("%s%u", r ? "," : "", vdp_reg(r));
    printf("],\"pal\":[");
    for (int i = 0; i < 16; i++) printf("%s%u", i ? "," : "", vdp_palette(i));
    printf("],\"scrmod\":%u,\"addr\":%lu}\n", mem_rd(0xFCAF), (unsigned long)vdp_vram_addr());
}
static uint8_t vd_hdr[64 + 32 + 2]; static uint16_t vd_idx[VDP_BLOCKS];
static void c_vramdelta(int argc, char **argv) {
    static link_piece_t pc[2 + 2 * VDP_BLOCKS / 2];   // header + (index, block) per dirty block (max 128 blocks per call)
    bool full = argc > 1 && !strcmp(argv[1], "full");
    uint32_t dirty[VDP_BLOCKS / 32]; vdp_take_dirty(dirty);
    if (full) vdp_mark_all_dirty(), vdp_take_dirty(dirty);
    unsigned cnt = 0;
    for (unsigned b = 0; b < VDP_BLOCKS; b++) {
        if (!(dirty[b >> 5] >> (b & 31) & 1) || !vdp_block_valid(b)) continue;
        if (cnt < 128) vd_idx[cnt++] = (uint16_t)b; else vdp_mark_dirty(b);   // the rest goes with the next call
    }
    for (int r = 0; r < 64; r++) vd_hdr[r] = vdp_reg(r);
    for (int i = 0; i < 16; i++) { vd_hdr[64 + 2 * i] = (uint8_t)vdp_palette(i); vd_hdr[65 + 2 * i] = (uint8_t)(vdp_palette(i) >> 8); }
    vd_hdr[96] = (uint8_t)cnt; vd_hdr[97] = (uint8_t)(cnt >> 8);
    unsigned np = 0; pc[np++] = (link_piece_t){ vd_hdr, sizeof vd_hdr };
    for (unsigned i = 0; i < cnt; i++) { pc[np++] = (link_piece_t){ &vd_idx[i], 2 }; pc[np++] = (link_piece_t){ vdp_vram() + (uint32_t)vd_idx[i] * VDP_BLOCK, VDP_BLOCK }; }
    uint32_t total = sizeof vd_hdr + cnt * (2 + VDP_BLOCK);
    printf("bin %lu\n", (unsigned long)total);
    if (!link_send_pieces(pc, np)) printf("err busy\n");
}
static void c_snapshot(int argc, char **argv) {
    if (msx.running) { printf("err stop first\n"); return; }
    unsigned gap = argc > 1 ? (unsigned)strtoul(argv[1], NULL, 10) : msx.vdp_gap_us;   // snapshot [GAP_US]
    printf(vdp_snapshot(gap) ? "ok\n" : "err bus\n");
}
static void c_linkstat(int argc, char **argv) { link_stat_print(); }
static void c_link(int argc, char **argv) {
    // link [uart|spi]: transport state / force a switch (board 2.0 bring-up)
    if (argc > 1 && !strcmp(argv[1], "uart")) link_set_mode(LINK_UART);
    else if (argc > 1 && !strcmp(argv[1], "spi")) link_set_mode(LINK_SPI);
    else if (argc > 1) { printf("usage: link [uart|spi]\n"); return; }
    printf("mode=%s board=%d (setting %lu, seen %lu)\n", link_mode() == LINK_SPI ? "spi" : "uart", board_kind(), (unsigned long)settings.board, (unsigned long)settings.board_seen);
    link_stat_print();
}
static void c_board(int argc, char **argv) {
    // board [auto|1|2]: which board this is (auto: detected from the ESP32's link activity)
    if (argc > 1) {
        uint32_t v = !strcmp(argv[1], "auto") ? 0 : !strcmp(argv[1], "1") ? 1 : !strcmp(argv[1], "2") ? 2 : 99;
        if (v == 99) { printf("usage: board [auto|1|2]\n"); return; }
        settings.board = v; settings.board_seen = v; settings_save();   // auto: forget the detection too
        printf("ok (takes effect at the next boot)\n"); return;
    }
    printf("board %d.0 (setting %s, seen %lu, link %s)\n", board_kind() ? board_kind() : 1, settings.board ? (settings.board == 1 ? "1" : "2") : "auto", (unsigned long)settings.board_seen, link_mode() == LINK_SPI ? "spi" : "uart");
}
static void c_audio(int argc, char **argv) {
    // audio tone HZ [LEVEL%] | audio off | audio: board 2.0 PWM test tone
    if (argc > 1 && !strcmp(argv[1], "tone")) {
        if (!audio_ready()) { printf("err no audio (board 1.0, or GPIO45 still owned by the LED)\n"); return; }
        audio_tone(argc > 2 ? strtoul(argv[2], NULL, 10) : 1000, argc > 3 ? strtoul(argv[3], NULL, 10) : 50); printf("ok rate %lu\n", (unsigned long)audio_rate_hz());
    } else if (argc > 1 && !strcmp(argv[1], "off")) { audio_off(); printf("ok\n"); }
    else printf("audio %s rate %lu\n", audio_ready() ? "ready" : "off", (unsigned long)audio_rate_hz());
}
static void c_baud(int argc, char **argv) {
    if (argc < 2) { printf("usage: baud BPS\n"); return; }
    uint32_t b = strtoul(argv[1], NULL, 10); if (b < 9600 || b > 4000000) { printf("err\n"); return; }
    printf("ok %lu\n", (unsigned long)b); link_end_then_baud = b;
}
static void c_vram(int argc, char **argv) { vdp_dump(argc > 1 ? strtoul(argv[1], NULL, 16) : 0, argc > 2 ? strtoul(argv[2], NULL, 16) : 0x100); }
static void c_vdpregs(int argc, char **argv) { vdp_dump_regs(); }
static void c_slot(int argc, char **argv) {
    // slot | slot copy2b (bench: duplicate A into B) | slot try (boot B once on the next reboot)
    char m[80];
    if (argc > 1 && !strcmp(argv[1], "copy2b")) { slot_copy_a_to_b(m, sizeof m); printf("%s\n", m); return; }
    if (argc > 1 && !strcmp(argv[1], "try")) { slot_request_try(); printf("ok try_b=1\n"); return; }
    if (argc > 1 && !strcmp(argv[1], "launch")) { printf("launching B\n"); sleep_ms(100); slot_launch_b(); printf("launch failed\n"); return; }
    printf("%s try_b=%lu b_len=%lu log=%lx\n", slot_is_b() ? "B (trial)" : "A", (unsigned long)settings.try_b, (unsigned long)settings.b_len, (unsigned long)settings.launch_log);
}
static void c_promote(int argc, char **argv) { char m[80]; slot_promote(m, sizeof m); printf("%s\n", m); }
static void c_atrans(int argc, char **argv) {
    // atrans [BASE_KB SIZE_KB]: set/show QMI ATRANS0 (XIP window 0 -> flash physical), flushes the cache
    if (argc > 2) { uint32_t b = strtoul(argv[1], NULL, 10) * 1024, sz = strtoul(argv[2], NULL, 10) * 1024; xip_cache_clean_all(); qmi_hw->atrans[0] = ((b >> 12) & 0xFFFu) | (((sz >> 12) & 0x7FFu) << 16); xip_cache_invalidate_all(); }
    printf("atrans0=%08lx atrans1=%08lx atrans2=%08lx atrans3=%08lx\n", (unsigned long)qmi_hw->atrans[0], (unsigned long)qmi_hw->atrans[1], (unsigned long)qmi_hw->atrans[2], (unsigned long)qmi_hw->atrans[3]);
}
// QSPI memory interface state (flash = M0, PSRAM = M1) for chasing corrupted XIP reads.
static void c_qmi(int argc, char **argv) {
    for (int m = 0; m < 2; m++)
        printf("m%d timing=%08lx rfmt=%08lx rcmd=%08lx wfmt=%08lx wcmd=%08lx\n", m, (unsigned long)qmi_hw->m[m].timing, (unsigned long)qmi_hw->m[m].rfmt,
               (unsigned long)qmi_hw->m[m].rcmd, (unsigned long)qmi_hw->m[m].wfmt, (unsigned long)qmi_hw->m[m].wcmd);
    printf("direct_csr=%08lx atrans0=%08lx xip_ctrl=%08lx flashrd=%d\n", (unsigned long)qmi_hw->direct_csr, (unsigned long)qmi_hw->atrans[0], (unsigned long)xip_ctrl_hw->ctrl, settings_flash_check());
}
static void c_romcheck(int argc, char **argv) {
    bool fix = argc > 1 && !strcmp(argv[1], "fix"); uint32_t bytes = 0;
    uint32_t bad = msx_rom_check(fix, &bytes);
    if (bad == 0xFFFFFFFFu) { printf("err stop first (bus must be enabled)\n"); return; }
    printf("%s%lu lines differ, %lu bytes%s\n", bad ? "" : "ok ", (unsigned long)bad, (unsigned long)bytes, fix && bad ? " (cache refreshed)" : "");
}
static void c_flashdump(int argc, char **argv) {
    // flashdump OFF [LEN]: hex dump of flash (XIP window) at offset OFF (hex)
    uint32_t off = argc > 1 ? strtoul(argv[1], NULL, 16) : 0, len = argc > 2 ? strtoul(argv[2], NULL, 16) : 64;
    if (len > 1024) len = 1024;
    const uint8_t *p = (const uint8_t *)(XIP_BASE + off);
    for (uint32_t i = 0; i < len; i++) { if (i % 32 == 0) printf("%06lx:", (unsigned long)(off + i)); printf(" %02x", p[i]); if (i % 32 == 31 || i == len - 1) printf("\n"); }
}
static void c_iowrite(int argc, char **argv) {
    // iowrite PORT VAL (hex): raw I/O write on the MSX bus (CPU stopped)
    if (argc < 3) { printf("usage: iowrite PORT VAL\n"); return; }
    if (msx.running) { printf("err stop first\n"); return; }
    if (!bus_enabled()) bus_enable(true);
    bus_io_write((uint16_t)strtoul(argv[1], NULL, 16), (uint8_t)strtoul(argv[2], NULL, 16)); bus_drain(); printf("ok\n");
}
static void c_vdpcmds(int argc, char **argv) {
    static vdpcmd_log_t l[64]; unsigned n = vdpcmd_log(l, argc > 1 ? (unsigned)atoi(argv[1]) : 16);
    for (unsigned i = 0; i < n; i++) printf("%lu us cmd %02x mode %u sx %u sy %u dx %u dy %u nx %u ny %u clr %02x arg %02x\n", (unsigned long)l[i].t_us, l[i].cmd, l[i].mode, l[i].sx, l[i].sy, l[i].dx, l[i].dy, l[i].nx, l[i].ny, l[i].clr, l[i].arg);
}
static void c_jstatus(int argc, char **argv) {
    vdpcmd_stats_t vcs; vdpcmd_stats(&vcs);
    printf("{\"fw\":\"%s\",\"run\":%d,\"reset\":%d,\"bus\":%d,\"fault\":%d,\"clock\":%d,\"uptime\":%lu,\"cycles\":%lu,\"insns\":%lu,\"ints\":%lu,\"romfills\":%lu,"
           "\"a8\":%u,\"sec\":[%u,%u,%u,%u],\"map\":[%u,%u,%u,%u],\"vdpgap\":%lu,\"refresh\":%lu,\"autostart\":%u,\"por\":%u,\"speed\":%u,\"sysclk_set\":%lu,\"board\":\"V%d.0\",\"link\":\"%s\",\"clk_hz\":%lu,\"ram_kb\":%u,\"ram_set\":%lu,\"flashrd\":%d,\"psram_kb\":%u,\"vram_kb\":128,\"vdpcmds\":%lu,\"vdpcmd_drop\":%lu,\"slot\":\"%s\",\"crash\":\"%s\",\"restart_needed\":%d,\"vzdisk\":[%lu,%lu,%lu,%lu,%lu],\"b_len\":%lu,\"sys_mhz\":%lu,\"temp\":%.1f,"
           "\"pc\":%u,\"sp\":%u,\"af\":%u,\"bc\":%u,\"de\":%u,\"hl\":%u,\"ix\":%u,\"iy\":%u,\"i\":%u,\"r\":%u,\"im\":%u,\"iff\":%u,\"halt\":%u,\"keys\":%d}\n",
           VZ80_VERSION, msx.running, msx.in_reset, bus_enabled(), bus_faulted(), msx_clock_present(), (unsigned long)msx_uptime_s(),
           (unsigned long)cpu.cycles, (unsigned long)cpu.insns, (unsigned long)msx.ints, (unsigned long)msx.rom_fills,
           msx.ppi_a8, msx.sec[0], msx.sec[1], msx.sec[2], msx.sec[3], msx.mapper[0], msx.mapper[1], msx.mapper[2], msx.mapper[3],
           (unsigned long)msx.vdp_gap_us, (unsigned long)msx.refresh_us, settings.autostart, msx_power_on_boot(), msx.speed, (unsigned long)settings.sys_mhz, board_kind() == 2 ? 2 : 1, link_mode() == LINK_SPI ? "spi" : "uart", (unsigned long)msx_clock_hz(), (unsigned)msx.ram_segs * 16, (unsigned long)settings.ram_kb, settings_flash_check(), (unsigned)(PSRAM_SIZE / 1024), (unsigned long)vcs.executed, (unsigned long)vcs.dropped, slot_is_b() ? "B" : "A", crash_last(), (cart_restart_needed() || fdc_restart_needed()) ? 1 : 0, (unsigned long)vzdisk_reads, (unsigned long)vzdisk_writes, (unsigned long)vzdisk_errors, (unsigned long)vzdisk_selects, (unsigned long)vzdisk_regs, (unsigned long)settings.b_len, (unsigned long)(clock_get_hz(clk_sys) / 1000000), diag_temp(),
           cpu.pc, cpu.sp, (cpu.a << 8) | cpu.f, (cpu.b << 8) | cpu.c, (cpu.d << 8) | cpu.e, (cpu.h << 8) | cpu.l, cpu.ix, cpu.iy, cpu.i, cpu.r, cpu.im, cpu.iff1 | (cpu.iff2 << 1), cpu.halted, kbd_pending());
}
static void c_jscreen(int argc, char **argv) {
    static char buf[80 * 27 + 64]; int cols;
    int rows = vdp_screen_text(buf, sizeof buf, &cols);
    printf("{\"mode\":\"%s\",\"cols\":%d,\"rows\":%d,\"r0\":%u,\"r1\":%u,\"text\":\"", rows ? "text" : "graphic", cols, rows, vdp_reg(0), vdp_reg(1));
    for (char *p = buf; rows && *p; p++) { if (*p == '"' || *p == '\\') putchar('\\'); if (*p == '\n') printf("\\n"); else putchar(*p); }
    printf("\"}\n");
}
static void c_set(int argc, char **argv) {
    // set autostart 0|1 | set vdpgap US | set refresh US ; save
    if (argc < 3) { printf("autostart=%u speed=%u vdpgap=%lu refresh=%lu sysclk=%lu ram=%lu\n", settings.autostart, settings.speed, (unsigned long)settings.vdp_gap_us, (unsigned long)settings.refresh_us, (unsigned long)settings.sys_mhz, (unsigned long)settings.ram_kb); return; }
    uint32_t v = strtoul(argv[2], NULL, 10);
    if (!strcmp(argv[1], "autostart")) settings.autostart = v ? 1 : 0;
    else if (!strcmp(argv[1], "speed")) { settings.speed = v > 2 ? 1 : (uint8_t)v; msx_set_speed(settings.speed); }   // 0 MSX2, 1 max, 2 turboR
    else if (!strcmp(argv[1], "sysclk")) { if (v != 150 && v != 200 && v != 250 && v != 300) { printf("err 150|200|250|300\n"); return; } settings.sys_mhz = v; settings.clk_pending = 0; }   // applied at the next boot
    else if (!strcmp(argv[1], "ram")) { if (v != 64 && v != 256 && v != 512 && v != 1024) { printf("err 64|256|512|1024\n"); return; } settings.ram_kb = v; }   // applied when the MSX restarts
    else if (!strcmp(argv[1], "vdpgap")) { settings.vdp_gap_us = v; msx.vdp_gap_us = v; }
    else if (!strcmp(argv[1], "refresh")) { settings.refresh_us = v; msx.refresh_us = v; }
    else { printf("err unknown setting\n"); return; }
    printf("ok\n");
}
static void c_save(int argc, char **argv) { printf(settings_save() ? "ok saved\n" : "err save\n"); }
static void c_run(int argc, char **argv) {
    // run: resume after `stop`; run reset: full restart (Z80 reset, slot re-sync)
    if (argc > 1 && !strcmp(argv[1], "reset")) { msx_stop(); msx.started = false; }
    msx_resume(); printf(msx.running ? "ok running\n" : "err\n");
}
static void c_stop(int argc, char **argv) { msx_stop(); printf("ok stopped pc=%04x\n", cpu.pc); }
static void c_reset(int argc, char **argv) { msx_reset_cpu(); printf("ok reset\n"); }
static void c_status(int argc, char **argv) {
    if (crash_last()[0]) printf("last run ended with: %s\n", crash_last());
    printf("vz80 fw=%s run=%d reset=%d bus=%d fault=%d cycles=%lu insns=%lu ints=%lu chunks=%lu romfills=%lu a8=%02x sec=%02x/%02x/%02x/%02x map=%d%d%d%d\n",
           VZ80_VERSION, msx.running, msx.in_reset, bus_enabled(), bus_faulted(), (unsigned long)cpu.cycles, (unsigned long)cpu.insns,
           (unsigned long)msx.ints, (unsigned long)msx.chunks, (unsigned long)msx.rom_fills, msx.ppi_a8,
           msx.sec[0], msx.sec[1], msx.sec[2], msx.sec[3], msx.mapper[0], msx.mapper[1], msx.mapper[2], msx.mapper[3]);
}
static void c_regs(int argc, char **argv) {
    printf("PC=%04x SP=%04x AF=%02x%02x BC=%02x%02x DE=%02x%02x HL=%02x%02x IX=%04x IY=%04x I=%02x R=%02x IM=%d IFF=%d%d HALT=%d WZ=%04x\n",
           cpu.pc, cpu.sp, cpu.a, cpu.f, cpu.b, cpu.c, cpu.d, cpu.e, cpu.h, cpu.l, cpu.ix, cpu.iy, cpu.i, cpu.r, cpu.im, cpu.iff1, cpu.iff2, cpu.halted, cpu.wz);
}
static void c_screen(int argc, char **argv) {
    static char buf[80 * 27 + 64]; int cols;
    int rows = vdp_screen_text(buf, sizeof buf, &cols);
    printf("screen mode=%s cols=%d r0=%02x r1=%02x r2=%02x vaddr=%05lx\n", rows ? "text" : "graphic", cols, vdp_reg(0), vdp_reg(1), vdp_reg(2), (unsigned long)vdp_vram_addr());
    if (rows) printf("%s", buf);
}
static void c_keys(int argc, char **argv) {
    // keys TEXT... (spaces between args become spaces; \n = Enter) | keys status | keys cancel
    if (argc < 2) { printf("usage: keys TEXT | keys status | keys cancel  (use \\n for Enter)\n"); return; }
    if (!strcmp(argv[1], "status")) { printf("pending=%d\n", kbd_pending()); return; }
    if (!strcmp(argv[1], "cancel")) { kbd_cancel(); printf("ok\n"); return; }
    char text[256]; int n = 0;
    {
        for (const char *p = raw_args; *p && n < 254; p++) {
            if (p[0] == '\\' && p[1] == 'n') { text[n++] = '\r'; p++; }
            else text[n++] = *p;
        }
    }
    text[n] = 0;
    printf(kbd_type(text) ? "ok queued %d\n" : "err (unmapped char or queue full)\n", kbd_pending());
}
#include "flashfar.h"
static void c_far(int argc, char **argv) {
    // far read ADDR LEN | far test ADDR : the flash above the 16 MB XIP window (4-byte address commands)
    static uint8_t buf[4096];
    if (argc > 2 && !strcmp(argv[1], "read")) {
        uint32_t a = strtoul(argv[2], NULL, 16), n = argc > 3 ? strtoul(argv[3], NULL, 16) : 64; if (n > sizeof buf) n = sizeof buf;
        absolute_time_t t0 = get_absolute_time(); bool ok = far_read(a, buf, n); int64_t us = absolute_time_diff_us(t0, get_absolute_time());
        if (!ok) { printf("err range\n"); return; }
        for (uint32_t i = 0; i < n; i += 16) { printf("%07lx:", (unsigned long)(a + i)); for (uint32_t k = i; k < i + 16 && k < n; k++) printf(" %02x", buf[k]); printf("\n"); }
        printf("%lu bytes in %lld us\n", (unsigned long)n, (long long)us); return;
    }
    if (argc > 2 && !strcmp(argv[1], "test")) {
        uint32_t a = strtoul(argv[2], NULL, 16) & ~(FAR_BLOCK - 1);
        absolute_time_t t0 = get_absolute_time(); bool e = far_erase(a, FAR_BLOCK); int64_t te = absolute_time_diff_us(t0, get_absolute_time());
        far_read(a, buf, 256); bool blank = true; for (int i = 0; i < 256; i++) if (buf[i] != 0xFF) blank = false;
        for (uint32_t i = 0; i < sizeof buf; i++) buf[i] = (uint8_t)(i * 7 + (i >> 8));
        t0 = get_absolute_time(); bool p = far_program(a, buf, sizeof buf); int64_t tp = absolute_time_diff_us(t0, get_absolute_time());
        static uint8_t rd[4096]; t0 = get_absolute_time(); far_read(a, rd, sizeof rd); int64_t tr = absolute_time_diff_us(t0, get_absolute_time());
        int bad = 0; for (uint32_t i = 0; i < sizeof rd; i++) if (rd[i] != (uint8_t)(i * 7 + (i >> 8))) bad++;
        printf("erase %s (%lld us, blank=%d), program %s (%lld us), read 4096 in %lld us, mismatches %d, xip ok=%d\n", e ? "ok" : "FAIL", (long long)te, blank, p ? "ok" : "FAIL", (long long)tp, (long long)tr, bad, settings_flash_check());
        return;
    }
    printf("usage: far read ADDR [LEN] | far test ADDR\n");
}
static void c_rpmem(int argc, char **argv) {   // rpmem ADDR [LEN]: hex dump of RP2350 memory (SRAM, XIP, PSRAM windows)
    if (argc < 2) { printf("usage: rpmem ADDR [LEN]\n"); return; }
    uint32_t a = strtoul(argv[1], NULL, 16), n = argc > 2 ? strtoul(argv[2], NULL, 16) : 32; if (n > 512) n = 512;
    const uint8_t *p = (const uint8_t *)a;
    for (uint32_t i = 0; i < n; i += 16) { printf("%08lx:", (unsigned long)(a + i)); for (uint32_t k = i; k < i + 16 && k < n; k++) printf(" %02x", p[k]); printf("\n"); }
}
static void c_vzlog(int argc, char **argv) { vzdisk_log_print(); }
static void c_ioread(int argc, char **argv) {
    // ioread PORT [N] [GAP_US]: raw I/O reads (CPU stopped), N times with GAP_US between them
    if (argc < 2) { printf("usage: ioread PORT [N] [GAP_US]\n"); return; }
    if (msx.running) { printf("err stop first\n"); return; }
    if (!bus_enabled()) { printf("err bus off\n"); return; }
    uint16_t port = (uint16_t)strtoul(argv[1], NULL, 16); int n = argc > 2 ? atoi(argv[2]) : 1; uint32_t gap = argc > 3 ? strtoul(argv[3], NULL, 10) : 0;
    for (int i = 0; i < n; i++) { if (i && gap) busy_wait_us(gap); printf("%s%02x", i ? " " : "", bus_io_read(port)); }
    printf("\n");
}
static void c_poke(int argc, char **argv) {
    // poke ADDR VAL [VAL...]: raw physical bus writes (CPU stopped); the slot model is not updated
    if (argc < 3) { printf("usage: poke ADDR VAL...\n"); return; }
    if (msx.running) { printf("err stop first\n"); return; }
    if (!bus_enabled()) { printf("err bus off\n"); return; }
    uint32_t a = strtoul(argv[1], NULL, 16);
    for (int i = 2; i < argc; i++) msx_phys_write((uint16_t)(a + i - 2), (uint8_t)strtoul(argv[i], NULL, 16));
    printf("ok\n");
}
static void c_pctrace(int argc, char **argv) {   // pctrace on | off | dump [N]
    if (argc > 1 && !strcmp(argv[1], "on")) { z80_trace_i = 0; z80_brk_hit = 0; z80_trace_on = 1; printf("ok\n"); return; }
    if (argc > 1 && !strcmp(argv[1], "off")) { z80_trace_on = 0; printf("ok\n"); return; }
    uint32_t n = argc > 2 ? strtoul(argv[2], NULL, 10) : 64; if (n > Z80_TRACE_N) n = Z80_TRACE_N; if (n > z80_trace_i) n = z80_trace_i;
    printf("trace %lu entries, last %lu (pc/sp):\n", (unsigned long)z80_trace_i, (unsigned long)n);
    for (uint32_t k = z80_trace_i - n; k < z80_trace_i; k++) { printf("%04x/%04x%s", z80_trace_pc[k & (Z80_TRACE_N - 1)], z80_trace_sp[k & (Z80_TRACE_N - 1)], ((k - (z80_trace_i - n)) % 8 == 7) ? "\n" : " "); }
    printf("\n");
}
static void c_brk(int argc, char **argv) {       // brk pc LO HI | brk sp MIN | brk off
    if (argc > 3 && !strcmp(argv[1], "pc")) { z80_brk_lo = (uint16_t)strtoul(argv[2], NULL, 16); z80_brk_hi = (uint16_t)strtoul(argv[3], NULL, 16); printf("ok\n"); return; }
    if (argc > 2 && !strcmp(argv[1], "sp")) { z80_brk_spmin = (uint16_t)strtoul(argv[2], NULL, 16); printf("ok\n"); return; }
    if (argc > 1 && !strcmp(argv[1], "off")) { z80_brk_lo = 1; z80_brk_hi = 0; z80_brk_spmin = 0; printf("ok\n"); return; }
    printf("brk pc %04x-%04x sp<%04x hit=%d trace=%d\n", z80_brk_lo, z80_brk_hi, z80_brk_spmin, z80_brk_hit, z80_trace_on);
}
static void c_slotmap(int argc, char **argv) { msx_slotmap_json(); }
static void c_slotscan(int argc, char **argv) { if (msx.running) { printf("err stop first\n"); return; } if (!bus_enabled()) bus_enable(true); msx_slot_scan(); }
static void c_peek(int argc, char **argv) {
    // peek ADDR [LEN]: physical bus read (CPU stopped) ; speek ADDR [LEN]: shadow view through the page tables
    if (argc < 2) { printf("usage: peek ADDR [LEN]\n"); return; }
    uint32_t a = strtoul(argv[1], NULL, 16), len = argc > 2 ? strtoul(argv[2], NULL, 16) : 16;
    if (msx.running && !strcmp(argv[0], "peek")) { printf("err stop first\n"); return; }
    if (!bus_enabled() && !strcmp(argv[0], "peek")) bus_enable(true);
    if (!bus_enabled()) { printf("err bus off\n"); return; }
    for (uint32_t i = 0; i < len; i++) {
        if (i % 16 == 0) printf("%04lx:", (unsigned long)(a + i));
        uint8_t v = !strcmp(argv[0], "peek") ? msx_phys_read((uint16_t)(a + i)) : mem_rd((uint16_t)(a + i));
        printf(" %02x", v);
        if (i % 16 == 15 || i == len - 1) printf("\n");
    }
}
static void c_shelf(int argc, char **argv) {
    // shelf | shelf put SIZE TYPE MTIME PATH... | shelf get I | shelf del I | shelf type I T | shelf dirty I 0|1 | shelf find PATH...
    if (argc < 2) { shelf_print_json(); return; }
    if (!strcmp(argv[1], "put")) {
        if (argc < 6) { printf("usage: shelf put SIZE TYPE MTIME PATH (bytes follow in 4096-byte pieces, each acked with ok)\n"); return; }
        const char *path = raw_args; for (int k = 0; k < 4 && path; k++) { path = strchr(path, ' '); if (path) while (*path == ' ') path++; }
        bool was = msx.running; msx_stop();
        shelf_receive(path ? path : "", strtoul(argv[2], NULL, 10), (uint8_t)strtoul(argv[3], NULL, 10), strtoul(argv[4], NULL, 10));
        if (was) msx_resume();
        return;
    }
    if (!strcmp(argv[1], "get") && argc > 2) {     // shelf get I [OFF LEN]
        if (!shelf_send(atoi(argv[2]), argc > 3 ? strtoul(argv[3], NULL, 10) : 0, argc > 4 ? strtoul(argv[4], NULL, 10) : 0)) printf("err\n");
        return;
    }
    if (!strcmp(argv[1], "del") && argc > 2) {
        int i = atoi(argv[2]);
        { int cp, cs; if (cart_any_uses_shelf(i, &cp, &cs)) { printf("err in use by slot %d-%d\n", cp, cs); return; } }
        if (fdc_uses_shelf(i)) { printf("err in use by a drive\n"); return; }
        printf(shelf_delete(i) ? "ok\n" : "err\n"); return;
    }
    if (!strcmp(argv[1], "type") && argc > 3) { printf(shelf_set_type(atoi(argv[2]), (uint8_t)atoi(argv[3])) ? "ok\n" : "err\n"); return; }
    if (!strcmp(argv[1], "mtime") && argc > 3) { printf(shelf_set_mtime(atoi(argv[2]), strtoul(argv[3], NULL, 10)) ? "ok\n" : "err\n"); return; }
    if (!strcmp(argv[1], "dirty") && argc > 3) {
        int i = atoi(argv[2]); bool d = atoi(argv[3]) != 0;
        if (!d && !fdc_shelf_clean(i)) { printf("err still dirty\n"); return; }
        printf(shelf_set_dirty(i, d) ? "ok\n" : "err\n"); return;
    }
    if (!strcmp(argv[1], "find") && argc > 2) {
        const char *path = raw_args; for (int k = 0; k < 2 && path; k++) { path = strchr(path, ' '); if (path) while (*path == ' ') path++; }
        printf("%d\n", shelf_find(path ? path : "")); return;
    }
    printf("err usage\n");
}
static uint8_t sync_recs[8 * 516];   // fdsync: 8 floppy sector records; hdsync: one 4104-byte hard disk block record
static void c_fdsync(int argc, char **argv) {   // fdsync: "bin N" + up to 8 dirty sector records | fddirty SHELF SECTOR
    static link_piece_t pc[1];
    int n = fdc_take_dirty(sync_recs, 8);
    printf("bin %lu\n", (unsigned long)(n * 516));
    if (n) { pc[0] = (link_piece_t){ sync_recs, (uint32_t)(n * 516) }; if (!link_send_pieces(pc, 1)) printf("err busy\n"); }
}
static void c_hdsync(int argc, char **argv) {   // hdsync: "bin 0|4104" + one dirty hard disk block record | hddirty SHELF BLOCK
    static link_piece_t pc[1];
    int n = fdc_take_hdd_dirty(sync_recs);
    printf("bin %lu\n", (unsigned long)(n * 4104));
    if (n) { pc[0] = (link_piece_t){ sync_recs, 4104 }; if (!link_send_pieces(pc, 1)) printf("err busy\n"); }
}
static void c_fddirty(int argc, char **argv) { if (argc < 3) { printf("err\n"); return; } fdc_mark_dirty_sector(atoi(argv[1]), strtoul(argv[2], NULL, 10)); printf("ok\n"); }
static void c_hddirty(int argc, char **argv) { if (argc < 3) { printf("err\n"); return; } fdc_mark_dirty_block(atoi(argv[1]), strtoul(argv[2], NULL, 10)); printf("ok\n"); }
static void c_vd(int argc, char **argv) {
    // vd add fdd|hdd | vd del N | vd order 2,1,3 ...  (N: 1-based row; the MSX restarts for the new drive letters)
    char err[128];
    if (argc > 2 && !strcmp(argv[1], "add")) { if (!fdc_vd_add(!strcmp(argv[2], "hdd") ? VK_HDD : !strcmp(argv[2], "fdd") ? VK_FDD : 0, err, sizeof err)) { printf("err %s\n", err); return; } printf("ok\n"); return; }
    if (argc > 2 && !strcmp(argv[1], "del")) { if (!fdc_vd_del(atoi(argv[2]) - 1, err, sizeof err)) { printf("err %s\n", err); return; } printf("ok\n"); return; }
    if (argc > 2 && !strcmp(argv[1], "order")) {
        int rows[VD_MAX], n = 0; for (const char *p = argv[2]; *p && n < VD_MAX; ) { rows[n++] = atoi(p) - 1; const char *c = strchr(p, ','); if (!c) break; p = c + 1; }
        if (!fdc_vd_order(rows, n, err, sizeof err)) { printf("err %s\n", err); return; } printf("ok\n"); return;
    }
    printf("err usage: vd add fdd|hdd | vd del N | vd order 1,2,3\n");
}
static void c_fd(int argc, char **argv) {
    // fd | fd A|B physical [1|2]|none | fd A|B|V1.. virtual SHELF_INDEX [wp] | fd A|B|V1.. eject | fd swap X Y
    if (argc < 2) { fdc_print_json(); return; }
    char err[128];
    if (!strcmp(argv[1], "swap") && argc > 3) { if (!fdc_swap(fdc_unit_by_name(argv[2]), fdc_unit_by_name(argv[3]), err, sizeof err)) { printf("err %s\n", err); return; } printf("ok\n"); return; }
    int d = argc > 2 ? fdc_unit_by_name(argv[1]) : -1;
    if (d < 0) { printf("err usage: fd A|B|V1.. ... | fd swap X Y\n"); return; }
    if (d >= FD_INT && strcmp(argv[2], "virtual") && strcmp(argv[2], "eject")) { printf("err V1.. take virtual I [wp] or eject\n"); return; }
    // Mode changes take effect immediately (the FDC is intercepted per access); only the number of drives
    // is counted by the disk ROM at boot, so "none" <-> present shows up after the next MSX restart.
    if (!strcmp(argv[2], "physical") || !strcmp(argv[2], "none")) { int hw = argc > 3 ? atoi(argv[3]) - 1 : 0; if (!fdc_set_mode(d, argv[2][0] == 'p' ? FD_PHYSICAL : FD_NONE, hw, err, sizeof err)) { printf("err %s\n", err); return; } printf("ok\n"); return; }
    if (!strcmp(argv[2], "virtual")) {
        if (argc > 3) { if (!fdc_mount(d, atoi(argv[3]), argc > 4 && !strcmp(argv[4], "wp"), err, sizeof err)) { printf("err %s\n", err); return; } }
        else if (!fdc_set_mode(d, FD_VIRTUAL, 0, err, sizeof err)) { printf("err %s\n", err); return; }
        printf("ok\n"); return;
    }
    if (!strcmp(argv[2], "eject")) { fdc_eject(d); printf("ok\n"); return; }
    printf("err usage\n");
}
static void c_romhex(int argc, char **argv) { if (argc < 4) { printf("usage: romhex main|sub|kanji|disk|fm OFF LEN (hex)\n"); return; } msx_romhex(argv[1], strtoul(argv[2], NULL, 16), strtoul(argv[3], NULL, 16)); }
static void c_fdtrace(int argc, char **argv) { if (argc > 1 && !strcmp(argv[1], "reset")) { msx_fdtrace_reset(); printf("ok\n"); return; } msx_fdtrace_print(argc > 1 ? atoi(argv[1]) : 64); }
static void c_cart(int argc, char **argv) {
    // cart | cart set CELL I [TYPE] | cart eject CELL | cart learn CELL   (CELL: 1, 2, 0-1, 3-3 ...)
    if (argc < 2) { cart_print_json(); return; }
    int p, s; char err[96];
    if (argc < 3 || !cart_parse_cell(argv[2], &p, &s)) { printf("err usage: cart set|eject|learn CELL ...\n"); return; }
    if (!strcmp(argv[1], "set") && argc > 3) {
        if (!cart_set(p, s, atoi(argv[3]), argc > 4 ? (uint8_t)atoi(argv[4]) : IMG_AUTO, err, sizeof err)) { printf("err %s\n", err); return; }
        printf("ok restart needed\n"); return;
    }
    if (!strcmp(argv[1], "eject")) { cart_eject(p, s); printf("ok restart needed\n"); return; }
    if (!strcmp(argv[1], "learn")) { cart_learn(p, s); printf("ok\n"); return; }
    printf("err usage\n");
}
static void c_fwup(int argc, char **argv) {
    if (argc < 2) { printf("usage: fwup SIZE (UF2 bytes follow in 4096-byte pieces, each acked with ok)\n"); return; }
    msx_stop();
    fwup_receive(strtoul(argv[1], NULL, 10), argc > 2 && !strcmp(argv[2], "b"));
}
static void c_vdpgap(int argc, char **argv) { if (argc > 1) msx.vdp_gap_us = strtoul(argv[1], NULL, 10); printf("vdp_gap_us=%lu refresh_us=%lu\n", (unsigned long)msx.vdp_gap_us, (unsigned long)msx.refresh_us); }
static void c_refresh(int argc, char **argv) { if (argc > 1) msx.refresh_us = strtoul(argv[1], NULL, 10); printf("refresh_us=%lu\n", (unsigned long)msx.refresh_us); }

static const cmd_t cmds[] = {
    {"help",   "list commands", c_help},
    {"info",   "chip id, clocks, temperature", c_info},
    {"led",    "led [auto | R G B]: status LED (manual hold)", c_led},
    {"locate", "locate [MS]: blink the LED white to find the board", c_locate},
    {"espstate", "espstate up|joining|unset (sent by the ESP32)", c_espstate},
    {"sram",   "64KiB SRAM pattern test", c_sram},
    {"flash",  "flash JEDEC id", c_flash},
    {"psram",  "psram [BYTES] [nocache] | psram init CLKDIV", c_psram},
    {"gpio",   "snapshot of bus-side pins", c_gpio},
    {"pins",   "pins [MS]: sample NMI/INT/BUSRQ/RESET for MS ms", c_pins},
    {"bus",    "bus engine bench: bus (no args for usage)", c_bus},
    {"z80bench", "z80bench [TSTATES]: core speed with SRAM memory, no bus", c_z80bench},
    {"run",    "start the Z80 (enables the bus)", c_run},
    {"jstatus","status as JSON", c_jstatus},
    {"jscreen","text screen as JSON", c_jscreen},
    {"set",    "set autostart|vdpgap|refresh VALUE", c_set},
    {"save",   "save settings to flash", c_save},
    {"watch",  "watch [DIV] [io|all]: passive bus capture while running", c_watch},
    {"iotrace","iotrace [N]: last N I/O accesses", c_iotrace},
    {"stop",   "stop the Z80", c_stop},
    {"reset",  "reset the Z80 and the slot model", c_reset},
    {"status", "machine status", c_status},
    {"regs",   "Z80 registers", c_regs},
    {"screen", "text screen from the VRAM shadow", c_screen},
    {"vram",   "vram ADDR [LEN]: VRAM shadow dump (hex)", c_vram},
    {"vdpstate","VDP registers and palette as JSON", c_vdpstate},
    {"vramdelta","vramdelta [full]: binary VRAM blocks changed since the last call", c_vramdelta},
    {"snapshot","read the whole VRAM back from the VDP (CPU stopped)", c_snapshot},
    {"baud",   "baud BPS: switch the ESP link speed after this reply", c_baud},
    {"linkstat", "ESP link UART error flags and ring state", c_linkstat},
    {"link",    "link [uart|spi]: ESP link transport (2.0: SPI)", c_link},
    {"board",   "board [auto|1|2]: board revision (auto-detected)", c_board},
    {"audio",   "audio tone HZ [LEVEL] | off: board 2.0 test tone", c_audio},
    {"vdpcmds", "vdpcmds [N]: last N emulated VDP commands", c_vdpcmds},
    {"slot",    "firmware slot: A, or B (trial image)", c_slot},
    {"flashdump", "flashdump OFF [LEN]: hex dump of flash at OFF (hex)", c_flashdump},
    {"qmi",    "QMI/XIP registers and a flash read check", c_qmi},
    {"romcheck", "romcheck [fix]: re-read the ROM cache from the machine and compare (stopped)", c_romcheck},
    {"atrans", "atrans [BASE_KB SIZE_KB]: QMI ATRANS0 (debug)", c_atrans},
    {"iowrite", "iowrite PORT VAL: raw I/O write (stopped)", c_iowrite},
    {"promote", "copy the trial image (slot B) over slot A", c_promote},
    {"vdpregs","observed VDP registers", c_vdpregs},
    {"keys",   "keys TEXT|status|cancel (\\n = Enter)", c_keys},
    {"peek",   "peek ADDR [LEN]: physical read (stopped)", c_peek},
    {"slotscan","probe every slot/subslot/page of the real machine (stopped)", c_slotscan},
    {"slotmap", "slot x subslot x page map as JSON", c_slotmap},
    {"pctrace", "pctrace on|off|dump [N]: record PC/SP of every instruction (debug)", c_pctrace},
    {"brk",     "brk pc LO HI | sp MIN | off: stop the Z80 when hit (needs pctrace on)", c_brk},
    {"poke",   "poke ADDR VAL...: physical write (stopped)", c_poke},
    {"ioread", "ioread PORT [N] [GAP_US]: raw I/O reads (stopped)", c_ioread},
    {"far",    "far read ADDR [LEN] | far test ADDR: flash above 16 MB", c_far},
    {"rpmem",  "rpmem ADDR [LEN]: RP2350 memory dump", c_rpmem},
    {"vzlog",  "vz80 disk port: recent register accesses", c_vzlog},
    {"speek",  "speek ADDR [LEN]: read through the page tables", c_peek},
    {"vdpgap", "vdpgap [US]: min interval between VRAM data accesses", c_vdpgap},
    {"refresh","refresh [US]: posted refresh interval", c_refresh},
    {"fwup",   "fwup SIZE: RP2350 firmware update over the ESP link", c_fwup},
    {"shelf",  "shelf [put SIZE TYPE MTIME PATH | get I | del I | type I T | dirty I 0|1 | find PATH]: flash cache of card images", c_shelf},
    {"fdsync", "dirty virtual-floppy sectors to copy back to the card (binary)", c_fdsync},
    {"fddirty", "fddirty SHELF SECTOR: re-mark a sector after a failed card write", c_fddirty},
    {"hdsync", "one dirty virtual hard disk block to copy back to the card (binary)", c_hdsync},
    {"hddirty", "hddirty SHELF BLOCK: re-mark a 4 KB block after a failed card write", c_hddirty},
    {"vd",     "vd add fdd|hdd | del N | order 1,2,..: Nextor drive rows", c_vd},
    {"cart",   "cart [set CELL I [TYPE] | eject CELL | learn CELL]: virtual cartridge in a slot cell (1, 2, 0-1 ...)", c_cart},
    {"fdtrace", "fdtrace [N]: last N FDC register accesses (7FF8-7FFF)", c_fdtrace},
    {"romhex", "romhex NAME OFF LEN: dump a cached ROM (hex)", c_romhex},
    {"fd",     "fd [A|B|V1.. physical|none|virtual [I [wp]]|eject | swap X Y]: drives", c_fd},
    {"esp",    "esp reset|dl|log|bridge [idle_ms]", c_esp},
    {"espstatus","ESP32 firmware/wifi status", c_espstatus},
    {"wifi",   "wifi SSID PASS | wifi clear (stored in ESP32 NVS)", c_wifi},
    {"reboot", "watchdog reboot", c_reboot},
    {"bootsel","reboot into USB bootloader", c_bootsel},
    {0, 0, 0},
};

static void dispatch(char *line) {
    char *argv[16]; int argc = 0;
    { const char *sp = strchr(line, ' '); const char *r = sp ? sp + 1 : ""; while (*r == ' ') r++; strncpy(raw_args, r, sizeof raw_args - 1); raw_args[sizeof raw_args - 1] = 0; }
    for (char *t = strtok(line, " \t"); t && argc < 16; t = strtok(NULL, " \t")) argv[argc++] = t;
    if (!argc) return;
    for (const cmd_t *c = cmds; c->name; c++) if (!strcmp(c->name, argv[0])) { c->fn(argc, argv); return; }
    printf("err unknown command '%s'\n", argv[0]);
}

void console_run(void) {
    led_booted();
    watchdog_enable(8000, true);   // a hang anywhere in this loop reboots instead of freezing (crash.c reports it)
    static char line[384]; unsigned n = 0;
    for (;;) {
        watchdog_update();
        msx_autostart_poll();
        slot_poll();
        led_task();
        { extern void sysclock_confirm(void); static bool clk_ok; if (!clk_ok && to_ms_since_boot(get_absolute_time()) > 10000) { clk_ok = true; sysclock_confirm(); } }
        for (int k = 0; k < 64; k++) msx_service();
        cart_learn_poll();
        fdc_poll();
        link_pump();
        link_task();
        // ESP32 link watchdog: no complete line from the ESP32 for 90 s (it pings every 5 s) -> pulse its
        // reset once (rate-limited). Every wedge seen so far cleared with an ESP32 reboot.
        { static uint32_t last_esp_reset_ms; uint32_t now_ms = to_ms_since_boot(get_absolute_time()), lr = link_last_rx_ms();
          uint32_t ref = lr ? lr : 0, silent = now_ms - ref;
          if (now_ms > 120000 && silent > 90000 && now_ms - last_esp_reset_ms > 300000) { last_esp_reset_ms = now_ms; printf("esp link silent %lu s: resetting ESP32\n", (unsigned long)(silent / 1000)); esp_reset(false); } }
        char *l = link_poll();
        if (l) { link_begin(); dispatch(l); link_end(); if (link_end_then_baud) { link_set_baud(link_end_then_baud); link_end_then_baud = 0; } }
        int ch = getchar_timeout_us(msx.running ? 0 : 200);
        if (ch == PICO_ERROR_TIMEOUT) continue;
        if (ch == '\r' || ch == '\n') { line[n] = 0; n = 0; dispatch(line); printf("> "); fflush(stdout); }
        else if (ch == 8 || ch == 127) { if (n) n--; }
        else if (n < sizeof line - 1 && ch >= 32) line[n++] = (char)ch;
    }
}
