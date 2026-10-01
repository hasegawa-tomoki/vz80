#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
// Virtual drives. Units 0 / 1 (A, B) are the internal disk ROM's drives behind the WD2793 emulation in
// the 7FF8-7FFF window (Sony HB-F1XDJ / Philips-style interface): each is one of the machine's internal
// drives (physical: hardware drive hw 0 or 1, found at start by fdc_probe_physical), a virtual floppy
// backed by a disk image on the shelf, or absent. Units 2.. (V1..V6) are the vz80 disk port's devices
// for the Nextor driver (vzdisk.c): an ordered list of virtual floppies (FDD, image in PSRAM) and
// virtual hard disks (HDD, image read and written in the far flash), one Nextor drive letter each in
// list order. Floppy images live in PSRAM units shared by the internal virtual drives and the FDD rows.
#define FD_INT   2
#define VD_MAX   6
#define FD_UNITS (FD_INT + VD_MAX)
#define FDD_POOL 4                          // PSRAM floppy image units (768 KB each, psram.h)
#define HDD_MAX  2                          // hard disk rows at once
#define HDD_MAX_BYTES (48u * 1024 * 1024)   // the far flash
enum { FD_PHYSICAL = 0, FD_VIRTUAL = 1, FD_NONE = 2 };
enum { VK_NONE = 0, VK_FDD = 1, VK_HDD = 2 };
void fdc_init(void);
void fdc_probe_physical(void);   // msx.c, at start (Z80 stopped, disk ROM cell in page 1): count the internal drives the ROM's way
int  fdc_physical_count(void);   // drives found, -1 = not probed yet
bool fdc_internal_hidden(void);  // both internal drives set to "none": the disk ROM itself is hidden from the BIOS (frees its RAM)
bool fdc_restart_needed(void);   // the row list (kinds / order) differs from what the running MSX registered at start
void fdc_poll(void);             // console loop: flush hard disk writes, unmount disk-port images when no external disk ROM exists
bool fdc_read(uint16_t a, uint8_t *v);            // a in 7FF8-7FFF while the disk ROM is mapped; always handles it
void fdc_write(uint16_t a, uint8_t v);
bool fdc_set_mode(int drive, int mode, int hw, char *err, size_t n);   // FD_PHYSICAL (hw 0/1) / FD_VIRTUAL / FD_NONE (A / B)
bool fdc_mount(int drive, int shelf_idx, bool wp, char *err, size_t n);
void fdc_eject(int drive);
bool fdc_swap(int a, int b, char *err, size_t n);              // exchange the images of two units of the same kind
void fdc_shelf_deleted(int i);
int  fdc_unit_by_name(const char *s);                          // "A" "B" "V1".."V6" -> 0..7, -1 otherwise
const char *fdc_unit_name(int d);
bool fdc_uses_shelf(int shelf_idx);
void fdc_print_json(void);
// Nextor rows (V1..): add / delete / reorder; each changes the drive letters, so the MSX must restart.
int  fdc_vd_count(void);
bool fdc_vd_add(int kind, char *err, size_t n);
bool fdc_vd_del(int row, char *err, size_t n);                 // row 0-based
bool fdc_vd_order(const int *rows, int count, char *err, size_t n);   // new order, current 0-based row indices
// Access for the vz80 disk port (vzdisk.c), row 0-based. Status: 0 ok, 1 write protected, 2 not ready, 8 not found, 10 write fault.
int  fdc_vd_kind(int row);
uint32_t fdc_vd_sectors(int row);                              // 0 when no image
uint8_t fdc_vd_read(int row, uint32_t sector, uint8_t *buf);
uint8_t fdc_vd_write(int row, uint32_t sector, const uint8_t *buf);
uint8_t fdc_take_changed(void);                                // media-changed bits of the rows (bit n = row n), cleared on read
// Copy-back of cached writes to the card (driven by the ESP32): floppy sector records (516 bytes:
// shelf, sector, data) and hard disk block records (4104 bytes: shelf, 4 KB block number, data).
int fdc_take_dirty(uint8_t *out, int max);
int fdc_take_hdd_dirty(uint8_t *out);                          // 0 or 1 record
bool fdc_has_dirty(void);
void fdc_mark_dirty_sector(int shelf_idx, uint32_t sector);
void fdc_mark_dirty_block(int shelf_idx, uint32_t block);
bool fdc_shelf_clean(int shelf_idx);
