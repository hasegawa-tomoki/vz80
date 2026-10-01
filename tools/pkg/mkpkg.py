#!/usr/bin/env python3
"""Build a vz80 firmware package (.vzp) from the RP2350 UF2 and the ESP32 app image.
usage: mkpkg.py VERSION vz80.uf2 vz80-esp.bin OUT.vzp
Layout: 64-byte header "VZP1" (see firmware/esp32c3/main/web.c), then the UF2, then the ESP app."""
import struct, sys

def vsum(b):
    s = 0
    for x in b: s = (s * 31 + x) & 0xFFFFFFFF
    return s

def main():
    if len(sys.argv) != 5: sys.exit(__doc__)
    ver, rp, esp, out = sys.argv[1:]
    rpb = open(rp, 'rb').read(); espb = open(esp, 'rb').read()
    label = ver.encode()[:23]
    hdr = struct.pack('<4sI24sIIIIIII', b'VZP1', 64, label, 64, len(rpb), vsum(rpb), 64 + len(rpb), len(espb), vsum(espb), 0)
    hdr += struct.pack('<I', vsum(hdr))
    assert len(hdr) == 64
    with open(out, 'wb') as f: f.write(hdr); f.write(rpb); f.write(espb)
    print(f'{out}: {ver}, rp {len(rpb)} bytes, esp {len(espb)} bytes')

if __name__ == '__main__': main()
