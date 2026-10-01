#!/usr/bin/env python3
"""Over-the-air firmware update through the ESP32 (uses curl: large bodies need Expect/keep-alive handling
that urllib does not do well against the ESP32 httpd).
usage: vz80ota.py HOST pkg release/vz80-X.Y.Z.vzp     # RP2350 + ESP32 in one package (/api/fw)
       vz80ota.py HOST rp  firmware/rp2350/build/vz80.uf2
       vz80ota.py HOST esp firmware/esp32c3/build/vz80-esp.bin
The MSX must be cold-booted after the RP2350 has been updated (its RAM lives in the RP2350)."""
import os, subprocess, sys

API = {'pkg': 'fw', 'rp': 'rp-fw', 'esp': 'esp-fw'}

def main():
    if len(sys.argv) != 4 or sys.argv[2] not in API: sys.exit(__doc__)
    host, kind, path = sys.argv[1:]
    url = f'http://{host}/api/{API[kind]}'
    print(f'POST {url} ({os.path.getsize(path)} bytes)')
    r = subprocess.run(['curl', '-s', '-S', '-m', '600', '-X', 'POST', '-H', 'Content-Type: application/octet-stream',
                        '--data-binary', '@' + path, url], capture_output=True, text=True)
    out = (r.stdout + r.stderr).strip()
    print(out)
    if r.returncode != 0 or not out.startswith('ok'): sys.exit(1)

if __name__ == '__main__': main()
