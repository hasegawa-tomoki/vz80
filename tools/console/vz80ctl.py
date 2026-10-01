#!/usr/bin/env python3
"""Send console commands to a vz80 board over USB CDC and print the replies.
usage: vz80ctl.py [--port /dev/cu.usbmodemXXX] CMD [CMD ...]   (no pyserial needed)"""
import glob, os, sys, termios, time, select

def find_port():
    ports = sorted(glob.glob('/dev/cu.usbmodem*'))
    if not ports: sys.exit('no /dev/cu.usbmodem* found')
    return ports[0]

def main():
    args = sys.argv[1:]
    port = None
    if args and args[0] == '--port': port = args[1]; args = args[2:]
    if not args: sys.exit(__doc__)
    port = port or find_port()
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    attr = termios.tcgetattr(fd)
    attr[0] = 0; attr[1] = 0; attr[2] = termios.CS8 | termios.CREAD | termios.CLOCAL; attr[3] = 0
    attr[4] = attr[5] = termios.B115200
    termios.tcsetattr(fd, termios.TCSANOW, attr)
    def drain(timeout, until_prompt=False):
        out = b''; end = time.time() + timeout
        while time.time() < end:
            r, _, _ = select.select([fd], [], [], 0.05)
            if r:
                try: chunk = os.read(fd, 4096)
                except BlockingIOError: chunk = b''
                except OSError: break
                if chunk:
                    out += chunk
                    if until_prompt and out.endswith(b'> '): break
                    if not until_prompt: end = time.time() + 0.15
        return out.decode(errors='replace')
    drain(0.1)
    for cmd in args:
        os.write(fd, (cmd + '\n').encode())
        sys.stdout.write(drain(30.0, True)); sys.stdout.flush()
    os.close(fd)

if __name__ == '__main__': main()
