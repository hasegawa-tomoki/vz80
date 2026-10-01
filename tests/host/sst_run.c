// Run the SingleStepTests/z80 corpus (converted by tools/sst_convert.py) against the vz80 core.
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "z80.h"
#include "z80_mem.h"

uint8_t host_mem[65536], host_port_in[65536];
uint32_t host_io_log[64][2]; int host_io_n;

typedef struct { uint32_t r[25]; } state_t;
enum { PC, SP, A, B, C, D, E, F, H, L, I, R, EI, WZ, IX, IY, AF_, BC_, DE_, HL_, IM, P, Q, IFF1, IFF2 };

static void load(z80_t *z, const state_t *s) {
    memset(z, 0, sizeof *z); z80_reset(z);
    z->pc = s->r[PC]; z->sp = s->r[SP]; z->a = s->r[A]; z->b = s->r[B]; z->c = s->r[C]; z->d = s->r[D]; z->e = s->r[E];
    z->f = s->r[F]; z->h = s->r[H]; z->l = s->r[L]; z->i = s->r[I]; z->r = s->r[R]; z->wz = s->r[WZ]; z->ix = s->r[IX]; z->iy = s->r[IY];
    z->a2 = s->r[AF_] >> 8; z->f2 = s->r[AF_]; z->b2 = s->r[BC_] >> 8; z->c2 = s->r[BC_]; z->d2 = s->r[DE_] >> 8; z->e2 = s->r[DE_];
    z->h2 = s->r[HL_] >> 8; z->l2 = s->r[HL_]; z->im = s->r[IM]; z->iff1 = s->r[IFF1]; z->iff2 = s->r[IFF2]; z->q = s->r[Q];
    z->ei_delay = s->r[EI];
}
static int cmp(const z80_t *z, const state_t *s, char *why) {
    #define CK(name, got, exp) if ((uint32_t)(got) != (uint32_t)(exp)) { why += sprintf(why, " %s=%04x(exp %04x)", name, (unsigned)(got), (unsigned)(exp)); bad++; }
    int bad = 0;
    CK("pc", z->pc, s->r[PC]); CK("sp", z->sp, s->r[SP]); CK("a", z->a, s->r[A]); CK("f", z->f, s->r[F]);
    CK("b", z->b, s->r[B]); CK("c", z->c, s->r[C]); CK("d", z->d, s->r[D]); CK("e", z->e, s->r[E]); CK("h", z->h, s->r[H]); CK("l", z->l, s->r[L]);
    CK("i", z->i, s->r[I]); CK("r", z->r, s->r[R]); CK("wz", z->wz, s->r[WZ]); CK("ix", z->ix, s->r[IX]); CK("iy", z->iy, s->r[IY]);
    CK("af_", (z->a2 << 8) | z->f2, s->r[AF_]); CK("bc_", (z->b2 << 8) | z->c2, s->r[BC_]); CK("de_", (z->d2 << 8) | z->e2, s->r[DE_]); CK("hl_", (z->h2 << 8) | z->l2, s->r[HL_]);
    CK("im", z->im, s->r[IM]); CK("iff1", z->iff1, s->r[IFF1]); CK("iff2", z->iff2, s->r[IFF2]); CK("q", z->q, s->r[Q]);
    return bad;
}

int main(int argc, char **argv) {
    if (argc < 2) { fprintf(stderr, "usage: sst_run corpus.bin [max_failures_to_print]\n"); return 2; }
    FILE *f = fopen(argv[1], "rb"); if (!f) { perror("open"); return 2; }
    int maxprint = argc > 2 ? atoi(argv[2]) : 20;
    char magic[4]; uint32_t n; fread(magic, 1, 4, f); fread(&n, 4, 1, f);
    if (memcmp(magic, "SST1", 4)) { fprintf(stderr, "bad corpus\n"); return 2; }
    uint32_t pass = 0, fail = 0, cyc_fail = 0; z80_t z;
    // per-file aggregation: key = name without the trailing " NNNN" index
    static struct { char key[16]; uint32_t fail, cyc; char example[200]; } agg[2048]; int nagg = 0;
    for (uint32_t t = 0; t < n; t++) {
        char name[32]; state_t ini, fin; uint16_t k;
        fread(name, 1, 32, f); fread(&ini, sizeof ini, 1, f); fread(&fin, sizeof fin, 1, f);
        memset(host_mem, 0, sizeof host_mem);
        fread(&k, 2, 1, f); for (int i = 0; i < k; i++) { uint16_t a; uint8_t v; fread(&a, 2, 1, f); fread(&v, 1, 1, f); host_mem[a] = v; }
        struct { uint16_t a; uint8_t v; } fram[64]; uint16_t nf;
        fread(&nf, 2, 1, f); for (int i = 0; i < nf; i++) { fread(&fram[i].a, 2, 1, f); fread(&fram[i].v, 1, 1, f); }
        uint16_t np; fread(&np, 2, 1, f);
        struct { uint16_t p; uint8_t v, d; } ports[16]; 
        for (int i = 0; i < np; i++) { fread(&ports[i].p, 2, 1, f); fread(&ports[i].v, 1, 1, f); fread(&ports[i].d, 1, 1, f); if (ports[i].d == 1) host_port_in[ports[i].p] = ports[i].v; }
        uint16_t ncyc; fread(&ncyc, 2, 1, f);
        load(&z, &ini); host_io_n = 0;
        uint32_t c0 = z.cycles;
        z80_step(&z);
        char why[1024] = ""; char *w = why;
        int bad = cmp(&z, &fin, w); w += strlen(w);
        for (int i = 0; i < nf; i++) if (host_mem[fram[i].a] != fram[i].v) { w += sprintf(w, " mem[%04x]=%02x(exp %02x)", fram[i].a, host_mem[fram[i].a], fram[i].v); bad++; }
        for (int i = 0; i < np; i++) if (ports[i].d == 2) {
            int found = 0; for (int j = 0; j < host_io_n; j++) if (host_io_log[j][0] == ports[i].p && host_io_log[j][1] == (0x200u | ports[i].v)) found = 1;
            if (!found) { w += sprintf(w, " out[%04x]!=%02x", ports[i].p, ports[i].v); bad++; }
        }
        int cyc_bad = (z.cycles - c0 != ncyc);
        if (cyc_bad) { cyc_fail++; w += sprintf(w, " cycles=%u(exp %u)", z.cycles - c0, ncyc); }
        if (bad) fail++; else pass++;
        if (bad || cyc_bad) {
            char key[16]; strncpy(key, name, 15); key[15] = 0; char *sp = strrchr(key, ' '); if (sp) *sp = 0;
            int j; for (j = 0; j < nagg; j++) if (!strcmp(agg[j].key, key)) break;
            if (j == nagg && nagg < 2048) { strcpy(agg[nagg].key, key); agg[nagg].fail = agg[nagg].cyc = 0; snprintf(agg[nagg].example, 200, "%s:%s", name, why); nagg++; }
            if (j < nagg) { agg[j].fail += bad ? 1 : 0; agg[j].cyc += cyc_bad; }
        }
    }
    // sort groups by failure count, descending
    for (int a = 0; a < nagg; a++) for (int b = a + 1; b < nagg; b++) if (agg[b].fail + agg[b].cyc > agg[a].fail + agg[a].cyc) { typeof(agg[0]) t = agg[a]; agg[a] = agg[b]; agg[b] = t; }
    for (int j = 0; j < nagg && j < maxprint; j++) printf("%-8s fail=%-5u cyc=%-5u  e.g. %s\n", agg[j].key, agg[j].fail, agg[j].cyc, agg[j].example);
    printf("groups with failures: %d\n", nagg);
    printf("pass %u fail %u (cycle mismatches %u) of %u\n", pass, fail, cyc_fail, n);
    return fail ? 1 : 0;
}
