#!/bin/sh
# Builds the vz80 Nextor kernel ROM. Needs Nestor80 (N80) and the Nextor kernel base file
# (Nextor-2.1.2.base.dat from https://github.com/Konamiman/Nextor/releases).
#   N80=/path/to/N80 BASE=/path/to/Nextor-2.1.2.base.dat sh build.sh
set -e
cd "$(dirname "$0")"
: "${N80:=N80}"; : "${BASE:=Nextor-2.1.2.base.dat}"
"$N80" vz80drv.mac vz80drv.bin --build-type abs --no-show-banner
python3 mknexrom.py "$BASE" vz80drv.bin nextor-vz80.rom
