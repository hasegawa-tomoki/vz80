// vz80 custom board definition (RP2350B, 48 GPIO)
#ifndef _BOARDS_VZ80_H
#define _BOARDS_VZ80_H

#define PICO_RP2350A 0                       // B variant: 48 GPIOs
#define PICO_XOSC_STARTUP_DELAY_MULTIPLIER 64

// W25Q512JVEIQ: 64MB QSPI flash. Only the first 16MB is XIP-mappable
// (RP2350 3-byte address window), so PICO_FLASH_SIZE_BYTES stays at 16MB;
// the upper 48MB is the ROM library, accessed with the 4-byte-address
// driver (romstore, phase 2). Settings live at 16MB-4KB.
#define VZ80_FLASH_CHIP_BYTES (64 * 1024 * 1024)
#define PICO_FLASH_SPI_CLKDIV 2
#define PICO_FLASH_SIZE_BYTES (16 * 1024 * 1024)
#define PICO_BOOT_STAGE2_CHOOSE_W25Q080 1    // W25Qxx JV series compatible

// No default LED / no default UART (ESP link uses uart1 with explicit,
// role-swapped pins — see esp_link.c and finding F-2).

#define PICO_RP2350_B0_SUPPORTED 1
#endif
