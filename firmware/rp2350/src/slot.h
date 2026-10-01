#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
// A/B firmware slots without a partition table: slot A (flash 0) is what the bootrom boots; slot B
// (flash +1 MiB) holds a trial image. When settings.try_b is set, slot A clears it and chains into B
// through a QMI address translation (so the same image bytes run from either slot). B is "promoted"
// by copying it over A (first sector last). A power cycle or reboot without promotion boots A again.
#define SLOT_SIZE   0x100000u
#define SLOT_B_OFF  0x100000u
void slot_init(void);                     // call first in main(): may chain into slot B and never return
bool slot_is_b(void);                     // running the trial image from slot B (false once promoted)
void slot_poll(void);                     // auto-promote the trial image after 20 s of healthy running (vmpu68 rule)
void slot_launch_b(void);
void slot_launch_if_pending(void);        // call once after all peripherals are initialised                 // jump into the slot B image now (debug / trial)
bool slot_promote(char *msg, size_t n);   // copy B -> A (verified); only meaningful while running B
bool slot_copy_a_to_b(char *msg, size_t n); // bench test helper: duplicate the running slot A image into B
void slot_request_try(void);              // boot slot B once on the next reboot
