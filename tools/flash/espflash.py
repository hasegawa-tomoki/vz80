#!/usr/bin/env python3
"""Flash the ESP32-C3 through the RP2350 USB<->UART bridge. Partition layout: firmware/esp32c3/partitions.csv (ota_0 at 0x10000, ota_1 at 0x200000).
usage: espflash.py [--port PORT] [--python ESPTOOL_PYTHON] BUILD_DIR   (writes bootloader, partition table, ota data, app)
       espflash.py ... --app-only BUILD_DIR                            (app only, keeps NVS/Wi-Fi settings)
       espflash.py ... --both-slots BUILD_DIR                          (app into ota_0 and ota_1, boot ota_0)"""
import glob, os, sys, termios, time, select, subprocess

def main():
    args = sys.argv[1:]; port = None; py = None; app_only = False; both = False
    while args and args[0].startswith('--'):
        if args[0] == '--port': port = args[1]; args = args[2:]
        elif args[0] == '--python': py = args[1]; args = args[2:]
        elif args[0] == '--app-only': app_only = True; args = args[1:]
        elif args[0] == '--both-slots': app_only = True; both = True; args = args[1:]
        else: sys.exit(__doc__)
    if len(args) != 1: sys.exit(__doc__)
    build = args[0]
    port = port or sorted(glob.glob('/dev/cu.usbmodem*'))[0]
    py = py or glob.glob(os.path.expanduser('~/.espressif/python_env/*/bin/python'))[0]
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    a = termios.tcgetattr(fd); a[0] = 0; a[1] = 0; a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL; a[3] = 0; a[4] = a[5] = termios.B115200
    termios.tcsetattr(fd, termios.TCSANOW, a)
    def rd(t):
        out = b''; end = time.time() + t
        while time.time() < end:
            r, _, _ = select.select([fd], [], [], 0.05)
            if r:
                try: out += os.read(fd, 4096)
                except OSError: break
        return out.decode(errors='replace')
    os.write(fd, b'esp dl\n'); banner = rd(2.5)
    if 'DOWNLOAD' not in banner: sys.exit('ESP did not enter download mode:\n' + banner)
    os.write(fd, b'esp bridge 120000\n'); rd(0.3)   # keep fd open: closing it drops DTR while esptool reopens the port
    cmd = [py, '-m', 'esptool', '--chip', 'esp32c3', '--port', port, '--baud', '115200',
           '--before', 'no_reset', '--after', 'no_reset', 'write_flash', '--flash_mode', 'dio', '--flash_size', '4MB', '--flash_freq', '80m']
    if not app_only:
        cmd += ['0x0', f'{build}/bootloader/bootloader.bin', '0x8000', f'{build}/partition_table/partition-table.bin', '0xd000', f'{build}/ota_data_initial.bin']
    cmd += ['0x10000', f'{build}/vz80-esp.bin']
    if both: cmd += ['0x200000', f'{build}/vz80-esp.bin', '0xd000', f'{build}/ota_data_initial.bin']   # both OTA slots (partitions.csv: ota_1 at 0x200000) + boot ota_0
    rc = subprocess.call(cmd)
    time.sleep(0.5); os.write(fd, b'\x1d\x1d\x1d'); time.sleep(0.6)   # quiet, escapes, quiet: the bridge closes when nothing follows the three 0x1D
    closed = rd(1.5)
    if 'bridge closed' not in closed: print('warning: bridge did not report closing (RP firmware < 0.6.1?)')
    os.write(fd, b'esp reset\n'); time.sleep(2.5); print(rd(0.5)[-200:])
    os.close(fd)
    print('esptool exit', rc)
    sys.exit(rc)

if __name__ == '__main__': main()
