#!/usr/bin/env python3
"""Convert SingleStepTests/z80 v1 JSON files into one compact binary for tests/host/sst_run.
usage: sst_convert.py SST_V1_DIR OUT.bin"""
import json, os, struct, sys, glob

REGS = ['pc','sp','a','b','c','d','e','f','h','l','i','r','ei','wz','ix','iy','af_','bc_','de_','hl_','im','p','q','iff1','iff2']
def pack_state(s):
    return struct.pack('<25I', *[int(s.get(k, 0)) for k in REGS])

def main():
    src, out = sys.argv[1], sys.argv[2]
    files = sorted(glob.glob(os.path.join(src, '*.json')))
    n = 0
    with open(out, 'wb') as f:
        f.write(b'SST1'); f.write(struct.pack('<I', 0))
        for fn in files:
            with open(fn) as jf: tests = json.load(jf)
            for t in tests:
                name = t['name'].encode()[:31]
                ini, fin = t['initial'], t['final']
                ports = t.get('ports', [])
                f.write(struct.pack('<32s', name))
                f.write(pack_state(ini)); f.write(pack_state(fin))
                f.write(struct.pack('<H', len(ini['ram'])))
                for a, v in ini['ram']: f.write(struct.pack('<HB', a, v))
                f.write(struct.pack('<H', len(fin['ram'])))
                for a, v in fin['ram']: f.write(struct.pack('<HB', a, v))
                f.write(struct.pack('<H', len(ports)))
                for p, v, d in ports: f.write(struct.pack('<HBB', p, v, 1 if d == 'r' else 2))
                f.write(struct.pack('<H', len(t['cycles'])))
                n += 1
        f.seek(4); f.write(struct.pack('<I', n))
    print('tests:', n, 'files:', len(files))

if __name__ == '__main__': main()
