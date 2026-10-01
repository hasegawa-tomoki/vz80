#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "z80.h"

extern z80_t cpu;
extern uint8_t msx_ram[65536];      // mapper RAM shadow (4 x 16 KiB segments)

typedef struct {
    bool running, in_reset, started;   // started: a run happened since boot (stop/run then resumes instead of resetting)
    uint8_t ppi_a8, sec[4], mapper[4];
    uint8_t ram_segs, seg_mask;     // mapper size in 16 KiB segments (4 = the machine's 64 KiB) and the register mask
    uint32_t chunks, faults, ints, resets;
    uint32_t rom_fills;             // 256-byte ROM lines fetched from the real machine
    uint32_t vdp_gap_us;            // graphic modes with the display on
    uint32_t refresh_us;            // interval between posted refresh cycles (0 = off)
    uint8_t speed;                  // 0 = paced to the real Z80 (3.58 MHz + M1 wait), 1 = unthrottled
} msx_state_t;
extern msx_state_t msx;

void msx_init(void);
void msx_map_flat(void);
uint32_t msx_rom_check(bool fix, uint32_t *bytes_out);   // compare the ROM cache with the machine (stopped); 0xFFFFFFFF = cannot            // bench: all 64 KiB -> msx_ram, no bus
void msx_start(void);               // enable bus, sync slot state from the machine, reset CPU, run
void msx_stop(void);
void msx_restart(void);
void msx_fdtrace_print(int n);
void msx_fdtrace_reset(void);
void msx_romhex(const char *name, uint32_t off, uint32_t len);             // full restart: Z80 reset, slot re-sync (as `run reset`)
void msx_set_cart_pages(int p, int s, uint8_t page_mask);   // cart.c: pages of a virtual cartridge in slot cell (p, s)
void msx_resume(void);              // continue after msx_stop() without a reset
void msx_reset_cpu(void);           // Z80 reset while staying enabled
void msx_service(void);             // run a chunk of instructions and poll pins; call continuously
// Raw physical access (CPU must be stopped).
uint8_t msx_phys_read(uint16_t a);
void msx_slot_scan(void);   // probe the real slots (stopped) and print a table
void msx_slotmap_json(void);   // slot x subslot x page map (model + scan + virtual cartridges)
bool msx_slot_expanded(int p);
bool msx_running(void);
uint8_t msx_disk_rom_slotid(void);   // BIOS slot id (bit 7 expanded, bits 0-1 primary, 2-3 secondary) of the internal disk ROM, 0xFF if none
bool msx_cell_free(int p, int s, uint8_t mask, bool ignore_live, char *err, size_t n);
void msx_phys_write(uint16_t a, uint8_t v);
void msx_iotrace_dump(unsigned n, unsigned skip);
void msx_iotrace_reset(void);
void msx_iotrace_trigger(int lo, int hi, int after);   // freeze the I/O trace 200 entries after out 99,lo / out 99,hi (-1 = off)
void msx_iotrace_skip(uint32_t mask);   // bit n: skip port 0x98+n
bool msx_clock_present(void);
uint32_t msx_clock_hz(void);        // measured CLK frequency (blocks ~200 us), 0 if absent
void msx_set_speed(uint8_t speed);
void msx_request_start_after_reboot(void);   // next boot starts the Z80 even without a power-on reset
void msx_autostart_poll(void);
void msx_autostart_init(void);
bool msx_power_on_boot(void);
uint32_t msx_uptime_s(void);
void msx_iotrace_freeze(bool en);
