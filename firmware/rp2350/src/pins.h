// vz80 pin map — single source of truth.
// Mirrors docs/signal-spec.html (2026-09-02). If the schematic changes, change this
// file and the spec together.
//
// PIO window rule (RP2350B): each PIO sees GPIO [base, base+31], base = 0 or 16.
//   - PIO_ADDR uses base 0  -> sees MA0-15 + everything up to WAIT/MM1/MRFSH.
//   - PIO_DATA uses base 16 -> sees MD0-7, all strobes, CLK/WAIT, DATA_DIR/OE.
// Everything the bus engine needs lives in the 16..31 overlap, so both PIOs
// can watch CLK, WAIT and the strobes.

#ifndef VZ80_PINS_H
#define VZ80_PINS_H

// ---- Z80 bus: address (outputs, via 74HCT541 U4/U5) ----
#define PIN_MA0        0    // ..PIN_MA15 = 15, contiguous, bit-ordered
#define PIN_MA_BASE    0
#define PIN_MA_COUNT   16

// ---- Z80 bus: data (bidirectional, via SN74LVC8T245 U6) ----
#define PIN_MD0        16   // ..PIN_MD7 = 23, contiguous, bit-ordered
#define PIN_MD_BASE    16
#define PIN_MD_COUNT   8

// ---- Z80 bus: clock / strobes / wait (overlap window 16..31) ----
#define PIN_CLK        24   // in:  Z80 clock from MSX (3.58MHz), 5V -> FT pad
#define PIN_MMREQ      25   // out: /MREQ  (via U3)
#define PIN_MIORQ      26   // out: /IORQ  (via U3)
#define PIN_MRD        27   // out: /RD    (via U3)
#define PIN_MWR        28   // out: /WR    (via U3)
#define PIN_WAIT       29   // in:  /WAIT from MSX
#define PIN_MM1        30   // out: /M1    (via U3)
#define PIN_MRFSH      31   // out: /RFSH  (via U3)

// ---- Slow signals (GPIO >= 32: SIO only, no PIO needed) ----
#define PIN_NMI        32   // in:  /NMI from MSX
#define PIN_INT        33   // in:  /INT from MSX
#define PIN_MHALT      34   // out: /HALT (via U3)
#define PIN_MBUSAK     35   // out: /BUSAK (via U3)
#define PIN_BUSRQ      36   // in:  /BUSRQ from MSX
#define PIN_RESET      37   // in:  /RESET from MSX

// ---- Data transceiver control (upper window only: PIO_DATA or SIO) ----
#define PIN_DATA_DIR   38   // out: U6 DIR. 1 = A->B (drive MSX bus), 0 = B->A (read)
#define PIN_DATA_OE    39   // out: U6 /OE. 0 = enabled. R10 pulls high at boot.

// ---- Misc ----
#define PIN_BUS_EN     40   // out: U3/U4/U5 /OE. 0 = drive bus. R9 pulls high at boot.
#define PIN_UART_RX    41   // in:  from ESP32-C3 TX (GPIO21/U0TXD). F2 = UART1 RX      (2.0: LINK_SCK, SPI slave clock)
#define PIN_UART_TX    42   // out: to ESP32-C3 RX (GPIO20/U0RXD). F11(aux) = UART1 TX   (2.0: LINK_MOSI, ESP32 -> RP)
#define PIN_ESP_EN     43   // out: ESP32 EN (also has R11 pullup + C20)
#define PIN_ESP_IO9    44   // out: ESP32 IO9 boot strap (low at ESP reset = download mode); 2.0: also LINK_CS in (via R16 1k)
#define PIN_LED_DATA   45   // out: 1.0: WS2812B-2020 DIN (LED_VDD = 5V - B5819W drop). 2.0: AUDIO_PWM -> R19 (the LED hangs off the ESP32)
#define PIN_LINK_MISO  46   // out: 2.0: LINK_MISO to ESP32 IO1 (1.0: not connected; driven high as the "2.0 here" flag)
#define PIN_PSRAM_CS   47   // out: APS6404L PSRAM /CE (XIP CS1, mapped at 0x11000000)
// Board 2.0 aliases (same pads, new roles; link.c / audio.c pick the role from the detected board)
#define PIN_LINK_SCK   PIN_UART_RX
#define PIN_LINK_MOSI  PIN_UART_TX
#define PIN_LINK_CS    PIN_ESP_IO9
#define PIN_AUDIO_PWM  PIN_LED_DATA

// Masks (SIO_HI bank for GPIO >= 32 handled by the SDK's gpio_* on RP2350B)
#define MASK_MA        (0xFFFFu << PIN_MA_BASE)
#define MASK_MD        (0xFFu   << PIN_MD_BASE)
#define MASK_STROBES   ((1u<<PIN_MMREQ)|(1u<<PIN_MIORQ)|(1u<<PIN_MRD)|(1u<<PIN_MWR)|(1u<<PIN_MM1)|(1u<<PIN_MRFSH))

// PIO window bases
#define PIO_ADDR_GPIOBASE  0
#define PIO_DATA_GPIOBASE  16

// Compile-time checks: window membership
_Static_assert(PIN_MA_BASE + PIN_MA_COUNT - 1 <= 31, "MA must fit PIO base-0 window");
_Static_assert(PIN_MD_BASE >= 16 && PIN_MD_BASE + PIN_MD_COUNT - 1 <= 47, "MD must fit PIO base-16 window");
_Static_assert(PIN_CLK >= 16 && PIN_CLK <= 31,  "CLK must be visible to both PIO windows");
_Static_assert(PIN_WAIT >= 16 && PIN_WAIT <= 31, "WAIT must be visible to both PIO windows");
_Static_assert(PIN_MMREQ >= 16 && PIN_MWR <= 31, "strobes must be visible to both PIO windows");
_Static_assert(PIN_DATA_DIR >= 16 && PIN_DATA_DIR <= 47, "DATA_DIR must be in the base-16 window");
_Static_assert(PIN_DATA_OE  >= 16 && PIN_DATA_OE  <= 47, "DATA_OE must be in the base-16 window");

#endif
