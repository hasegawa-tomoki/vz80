#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
// Virtual cartridges: a shelf image copied into PSRAM and mapped into a slot cell (primary slot p,
// secondary slot s; s = 0 for the unexpanded cartridge slots 1 / 2) with the usual MegaROM mappers
// (plain, ASCII 8K/16K, Konami, Konami SCC without sound). Up to CART_MAX at once.
#define CART_MAX 4
int cart_guess_type(const uint8_t *rom, uint32_t size);         // IMG_PLAIN / IMG_ASCII8 / ...
bool cart_parse_cell(const char *txt, int *p, int *s);          // "1", "2", "0-1", "3-3"
bool cart_set(int p, int s, int shelf_idx, uint8_t type, char *err, size_t n);   // type IMG_AUTO = guess
void cart_eject(int p, int s);
bool cart_present(int p, int s);
int  cart_index(int p, int s);                                  // slot of the cart in that cell, -1 if none
bool cart_any_uses_shelf(int shelf_idx, int *p, int *s);
bool cart_pending_uses_shelf(int shelf_idx);                    // referenced by the configuration the next start will load
void cart_shelf_deleted(int i);
void cart_restore(void);                                         // from settings, at MSX start (after cart_unload_all)
void cart_unload_all(void);
bool cart_restart_needed(void);                                  // pending configuration not yet applied
bool cart_pending_info(int p, int s, int *shelf, uint8_t *type, uint8_t *pages);   // the configuration the next start will load
const uint8_t *cart_read_ptr(int p, int s, uint16_t a);         // byte at Z80 address a, NULL if open bus
bool cart_write(int p, int s, uint16_t a, uint8_t v);           // mapper register write; true = banks changed
void cart_observe(int p, int s, uint16_t a, uint8_t v);         // run-time mapper detection (writes into the cartridge pages)
void cart_learn_poll(void);                                     // console loop: apply a corrected mapper type (restarts the MSX)
void cart_learn(int p, int s);                                  // console: watch this cart again (also for a user-chosen type)
bool cart_info(int p, int s, int *shelf, uint8_t *type, uint8_t *pages);   // for the slot map
uint8_t cart_pages_of(int shelf_idx);                           // pages (bit n = page n) a cached ROM would take with its stored type
void cart_print_json(void);
