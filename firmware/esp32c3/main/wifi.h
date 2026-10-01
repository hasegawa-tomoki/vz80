#pragma once
#include <stddef.h>
#include <stdbool.h>
void wifi_start(void);
// One-line status: "esp fw=.. mode=.. ssid=.. ip=.. ap=.. retries=.. reason=.."
void wifi_status(char *out, size_t n);
// Store credentials (empty ssid clears) and restart.
void wifi_set_credentials(const char *ssid, const char *pass);
uint32_t wifi_ip4(void);               // network byte order, 0 if no STA address
const char *wifi_mac_str(void);        // "aa:bb:cc:dd:ee:ff"
const char *wifi_state_str(void);      // "up" / "joining" / "unset" (vhd68 vocabulary)
const char *wifi_ssid(void);
const char *wifi_pass(void);           // stored password (only for writing the card's vz80.cfg)
const char *wifi_name(void);           // display name from NVS ("" if unset)
void wifi_set_name(const char *name);
// Settings from the SD card's /vz80.cfg: stored to NVS and, when Wi-Fi is already running, applied live.
void wifi_apply_cfg(const char *ssid, const char *pass, const char *name);
int wifi_tx_power(void);                 // current maximum transmit power, dBm
bool wifi_set_tx_power(int dbm);        // 2..20 dBm, applied now and kept in NVS (vz80.cfg txpower= also sets it)
