#include "crash.h"
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/watchdog.h"
#include "hardware/structs/watchdog.h"

// scratch[0..3]: ours (4..7 belong to the bootrom: watchdog_reboot() overwrites them)
#define CRASH_MAGIC 0xC5A5D00Du
#define PLANNED_MAGIC 0x600DB007u   // a deliberate watchdog reboot (reboot command, firmware update, slot launch)
static char last[96];

void crash_init(void) {
    if (watchdog_hw->scratch[0] == CRASH_MAGIC) {
        snprintf(last, sizeof last, "hardfault pc=%08lx lr=%08lx at %lu ms", (unsigned long)watchdog_hw->scratch[1], (unsigned long)watchdog_hw->scratch[2], (unsigned long)watchdog_hw->scratch[3]);
        watchdog_hw->scratch[0] = 0;
    } else if (watchdog_hw->scratch[0] == PLANNED_MAGIC) {
        watchdog_hw->scratch[0] = 0;
    } else if (watchdog_caused_reboot()) {
        snprintf(last, sizeof last, "watchdog reboot (hang)");
    }
}
void crash_mark_planned_reboot(void) { watchdog_hw->scratch[0] = PLANNED_MAGIC; }
const char *crash_last(void) { return last; }

void __attribute__((used)) hardfault_c(uint32_t *sp) {
    watchdog_hw->scratch[0] = CRASH_MAGIC;
    watchdog_hw->scratch[1] = sp[6];   // stacked PC
    watchdog_hw->scratch[2] = sp[5];   // stacked LR
    watchdog_hw->scratch[3] = to_ms_since_boot(get_absolute_time());
    watchdog_reboot(0, 0, 10);
    for (;;) tight_loop_contents();
}
void __attribute__((naked)) isr_hardfault(void) {
    __asm volatile("tst lr, #4\n ite eq\n mrseq r0, msp\n mrsne r0, psp\n b hardfault_c");
}
