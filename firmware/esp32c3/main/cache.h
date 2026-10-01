#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
// The microSD card holds the images; the RP2350 keeps copies in its flash cache ("shelf") and runs
// virtual drives/cartridges from there. This module drives the cache from the card side: import,
// mount/eject orchestration and the copy-back of virtual-floppy writes to the card.
bool sd_path_ok(const char *p);                       // "/dir/file", no "..", < 200 chars
const char *sd_path_err(const char *p);               // "err ...\n" explaining why sd_path_ok / the card refuses it
int  img_type_for(const char *path, long size);       // 9 floppy image (.dsk up to 720 KB), 10 hard disk image (.hdd, or a bigger .dsk), 0 (auto ROM mapper) otherwise
int  cache_find(const char *path);                    // shelf index or -1
bool cache_import(const char *path, char *status, size_t n);    // copy the card file into the flash cache
bool cache_uncache(const char *path, char *status, size_t n);
bool cache_mount(const char *path, const char *target, int type, char *status, size_t n);   // target A|B|V1..|1|2|P-S; type: ROM mapper (0 auto) or -1 = keep
bool cache_eject(const char *target, char *status, size_t n);
bool cache_fdswap(const char *a, const char *b, char *status, size_t n);
bool cache_vd(const char *op, const char *arg, char *status, size_t n);   // Nextor rows: add fdd|hdd, del N, order 1,2,..
int  cache_hist_json(char *out, size_t n);           // {"hist":[...]} the last mounted disk images, newest first
// Cache catalog as the RP reports it (JSON text), refreshed on demand; and helpers over it.
const char *cache_catalog_json(bool refresh);
bool cache_entry(int idx, char *path, size_t pn, uint32_t *size, uint32_t *mtime, int *dirty);
int  cache_lookup(const char *path, uint32_t *size, uint32_t *mtime, int *dirty);   // idx or -1
void cache_start(void);                               // copy-back task
