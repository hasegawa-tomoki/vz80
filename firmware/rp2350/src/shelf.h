#pragma once
#include <stdint.h>
#include <stdbool.h>
// Flash cache of images that live on the microSD card (the card is the master copy): the memory-mapped
// part of the flash from 2 MiB to 16 MiB - 4 KiB. An 8 KiB catalog at SHELF_BASE lists the entries
// (keyed by the card path); bodies follow, 64 KiB aligned so that they can be block-erased.
#define SHELF_BASE  0x200000u
#define SHELF_END   (0x1000000u - 0x1000u)   // the settings sector sits in the last 4 KiB
// Entries may also live in the far flash (16-64 MB, flashfar.h): no XIP pointer there, use shelf_read().
#define SHELF_MAX   64
#define SHELF_PATH  200
#define SHELF_DIRTY 1                          // flags: the cache may hold writes not yet copied back to the card
enum { IMG_AUTO = 0, IMG_PLAIN = 1, IMG_ASCII8 = 2, IMG_ASCII16 = 3, IMG_KONAMI = 4, IMG_KONAMI_SCC = 5, IMG_DISK = 9, IMG_HDD = 10 };   // IMG_HDD: hard disk image, far flash only, up to 48 MB
typedef struct { char path[SHELF_PATH]; uint32_t off, size, mtime; uint8_t type, flags, rsv[2]; uint32_t crc; } shelf_ent_t;   // rsv: 16-bit last-use sequence (LRU)
void shelf_init(void);
int shelf_count(void);
const shelf_ent_t *shelf_get(int i);            // NULL if out of range
int shelf_find(const char *path);               // index or -1
const uint8_t *shelf_data(int i);               // XIP pointer, NULL for a far-flash entry
bool shelf_is_far(int i);
bool shelf_read(int i, uint32_t off, uint8_t *dst, uint32_t n);   // works for both regions; dst may be PSRAM
uint32_t shelf_total_bytes(void);               // XIP pointer to the body (slow serial reads: copy, don't execute)
uint32_t shelf_free_bytes(void);
// Receive `size` bytes over the ESP link in 4096-byte pieces (each acked "ok"), store as a new entry
// (an existing entry with the same path is replaced). Prints "ready" first, then "done ..." or "err ...".
void shelf_receive(const char *path, uint32_t size, uint8_t type, uint32_t mtime);
bool shelf_send(int i, uint32_t off, uint32_t len);   // "bin LEN" + that part of the body (len 0 = to the end); cache -> card copy
bool shelf_delete(int i);                       // also renumbers the drive/slot references above i
bool shelf_set_mtime(int i, uint32_t mtime);    // card copy rewritten by the copy-back: keep the cache "fresh"
bool shelf_set_type(int i, uint8_t type);
bool shelf_set_dirty(int i, bool dirty);        // persisted flag
bool shelf_touch(int i);                        // "used now": LRU order for the eviction when a put needs room
bool shelf_write(int i, uint32_t off, const uint8_t *data, uint32_t n);   // in-place write (4 KiB read-modify-write)
void shelf_print_json(void);
const char *img_type_name(uint8_t t);
