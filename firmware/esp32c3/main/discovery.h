// LAN peer discovery, wire-compatible with vhd68/vfd68/vmpu68: one UDP broadcast line on
// port 6868 every 2 s: "x68pico <device> <version> <http_port> [name]"; peers heard from
// other senders are kept for 10 s and listed by /api/peers.
#pragma once
#include <stdint.h>
#include <stdbool.h>
#define DISC_PORT 6868
#define DISC_MAX_PEERS 8
typedef struct { bool used; uint32_t ip; uint16_t port; char device[16], version[16], name[32]; int64_t last_us; } disc_peer_t;
void disc_start(void);
const disc_peer_t *disc_peer(int i);
