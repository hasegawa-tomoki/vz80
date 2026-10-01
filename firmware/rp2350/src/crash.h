#pragma once
#include <stddef.h>
// Hard-fault capture and the run-time watchdog. A fault records PC/LR/uptime in the watchdog scratch
// registers and reboots; a hang trips the 8 s watchdog. crash_init() (early in main) turns the record
// into a string that `status` / `jstatus` report until the next crash.
void crash_init(void);
const char *crash_last(void);   // "" when the last boot was clean
void crash_mark_planned_reboot(void);   // call before a deliberate watchdog reboot so it is not reported as a hang
