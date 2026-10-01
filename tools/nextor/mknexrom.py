#!/usr/bin/env python3
"""Build a Nextor kernel ROM from the kernel base file and a driver bank (MIT, part of vz80).
usage: mknexrom.py Nextor-x.y.z.base.dat driver.bin OUT.ROM
Follows the "manual creation" recipe of the Nextor 2.1 Driver Development Guide (ASCII16 mapper):
base + page-0 code (255 bytes) + bank id + driver (16080 bytes) + bank-switching code (48 bytes).
"""
import sys
base = open(sys.argv[1], 'rb').read(); drv = open(sys.argv[2], 'rb').read()
k = base[254]
if len(drv) != 16080: sys.exit('driver must be 16080 bytes, got %d' % len(drv))
rom = base + base[:255] + bytes([k]) + drv + base[-48:]
open(sys.argv[3], 'wb').write(rom)
print('%s: %d bytes, %d base banks + driver bank %d' % (sys.argv[3], len(rom), k, k))
