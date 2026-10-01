#pragma once
// Firmware update from a vz80-*.vzp package in the microSD card root (newer versions only).
void sdfw_start(void);
const char *sdfw_status(void);   // last result, for /api/status
