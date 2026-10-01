#include "psram.h"
// vz80 Z80 core. MIT.
// Instruction-level interpreter written as one big switch whose CPU state lives in register
// variables (see below), with every opcode spelled out so register numbers and condition codes are
// compile-time constants. Memory and I/O go through the static inline hooks in z80_mem.h (platform).
// Rarely touched state (alternate set, I, R, IM, IFF, interrupt bookkeeping) stays in the z80_t.
#include "z80.h"
#include "z80_mem.h"     // platform: mem_rd, mem_wr, fetch_op, io_rd, io_wr, int_ack
uint16_t z80_io_pc;      // pc of the instruction doing I/O (for tracing)

// The CPU state the interpreter touches on every instruction. On the target these are pinned to
// callee-saved registers for the whole compilation unit (GCC global register variables): the
// compiler's own allocator, faced with ~700 basic blocks, otherwise leaves most of them on the
// stack. Callers of core_run() must save/restore r4-r11 around it (see z80_entry.c). On the host
// (test harness) they are plain statics.
#if defined(__thumb__)
#define Z80_PINNED 1
#define PIN(type, name, reg) register type name asm(reg)
#else
#define PIN(type, name, reg) static type name
#endif
PIN(uint16_t, pc, "r4");
PIN(uint32_t, cr,  "r5");   // (T-states remaining in this run) << 12 | M1 cycles done (R increments)
PIN(uint8_t,  a,  "r6");
PIN(uint8_t,  f,  "r7");
PIN(uint16_t, hl, "r8");
PIN(uint16_t, bc, "r9");
PIN(uint16_t, de, "r10");
PIN(uint32_t, q,  "r11");   // bits 0-7: flags written by this instruction (0 if none); bits 8-15: by the previous one
#define CR_T(n)   ((uint32_t)(n) << 12)
#define SLOW()    (cr -= 1u << 30)   // make the loop head take the slow path before the next instruction
#define RCNT()    (cr & 0xFFF)
// End of an opcode: charge its T-states and go to the next instruction, or to the slow path when the
// count went negative. With the pinned register this is `subs; bmi; b` (GCC would otherwise add a
// move and a compare).
#ifdef Z80_PINNED
#define END(n)    do { asm goto("subs %0, %0, %1\n\tbmi %l[slow]" : "+r"(cr) : "rI"(CR_T(n)) : "cc" : slow); goto next; } while (0)
#else
#define END(n)    do { cr -= CR_T(n); if (__builtin_expect((int32_t)cr < 0, 0)) goto slow; goto next; } while (0)
#endif

#define SF 0x80
#define ZF 0x40
#define YF 0x20
#define HF 0x10
#define XF 0x08
#define PF 0x04
#define NF 0x02
#define CF 0x01

static uint8_t szyx[256], szyxp[256];
static bool tables_ready;
static void build_tables(void) {
    for (int i = 0; i < 256; i++) {
        uint8_t fl = (i & (SF | YF | XF)) | (i == 0 ? ZF : 0);
        int p = 0; for (int b = 0; b < 8; b++) p ^= (i >> b) & 1;
        szyx[i] = fl; szyxp[i] = fl | (p ? 0 : PF);
    }
    tables_ready = true;
}

void z80_reset(z80_t *z) {
    if (!tables_ready) build_tables();
    z->pc = 0; z->sp = 0xFFFF; z->ix = z->iy = 0xFFFF; z->wz = 0;
    z->a = 0xFF; z->f = 0xFF; z->b = z->c = z->d = z->e = z->h = z->l = 0;
    z->a2 = z->f2 = z->b2 = z->c2 = z->d2 = z->e2 = z->h2 = z->l2 = 0;
    z->i = z->r = 0; z->im = 0; z->iff1 = z->iff2 = 0;
    z->halted = 0; z->ei_delay = 0; z->q = 0; z->nmi_pending = 0; z->after_ld_ai = 0;
}

// ---- local-register accessors (valid inside core_run only) ------------------------------
#define B ((uint8_t)(bc >> 8))
#define C ((uint8_t)bc)
#define D ((uint8_t)(de >> 8))
#define E ((uint8_t)de)
#define H ((uint8_t)(hl >> 8))
#define L ((uint8_t)hl)
#define SET_B(v) (bc = (uint16_t)((bc & 0x00FF) | ((uint8_t)(v) << 8)))
#define SET_C(v) (bc = (uint16_t)((bc & 0xFF00) | (uint8_t)(v)))
#define SET_D(v) (de = (uint16_t)((de & 0x00FF) | ((uint8_t)(v) << 8)))
#define SET_E(v) (de = (uint16_t)((de & 0xFF00) | (uint8_t)(v)))
#define SET_H(v) (hl = (uint16_t)((hl & 0x00FF) | ((uint8_t)(v) << 8)))
#define SET_L(v) (hl = (uint16_t)((hl & 0xFF00) | (uint8_t)(v)))
#define SET_BC(v) (bc = (uint16_t)(v))
#define SET_DE(v) (de = (uint16_t)(v))
#define SET_HL(v) (hl = (uint16_t)(v))
#define AFV ((uint16_t)(((uint16_t)a << 8) | f))
#define SET_AF(v) do { uint16_t v_ = (uint16_t)(v); a = (uint8_t)(v_ >> 8); f = (uint8_t)v_; } while (0)
#define R_NOW() ((uint8_t)((zp->r & 0x80) | ((zp->r + RCNT()) & 0x7F)))

#ifdef HAVE_MEM16   // platform provides fused 16-bit accessors
static inline uint16_t rd16(uint16_t ad) { return mem_rd16(ad); }
static inline void wr16(uint16_t ad, uint16_t v) { mem_wr16(ad, v); }
#else
static inline uint16_t rd16(uint16_t ad) { return (uint16_t)(mem_rd(ad) | (mem_rd((uint16_t)(ad + 1)) << 8)); }
static inline void wr16(uint16_t ad, uint16_t v) { mem_wr(ad, (uint8_t)v); mem_wr((uint16_t)(ad + 1), (uint8_t)(v >> 8)); }
#endif
#define PUSH(v)   do { uint16_t _v = (v); sp -= 2; wr16(sp, _v); } while (0)
#define POP()     ({ uint16_t _v = rd16(sp); sp += 2; _v; })
#define IMM8()    mem_rd(pc++)
#define IMM16()   ({ uint16_t _v = rd16(pc); pc += 2; _v; })
#define IORD(p)    (z80_io_pc = pc, io_rd(p))
#define IOWR(p, v) (z80_io_pc = pc, io_wr((p), (v)))

// ---- flags / ALU (everything that writes F goes through SETF, which also records Q) ------
#define SETF(v)   (f = (uint8_t)(v), q = (q & ~0xFFu) | f)
#define QPREV     ((uint8_t)(q >> 8))   // flags written by the previous instruction, else 0
#define ALU_ADD(v, cy) do { uint8_t v_ = (v); unsigned r_ = a + v_ + (cy); SETF(szyx[r_ & 0xFF] | ((a ^ v_ ^ r_) & HF) | (((a ^ r_) & (v_ ^ r_) & 0x80) >> 5) | (r_ >> 8)); a = (uint8_t)r_; } while (0)
#define ALU_SUB(v, cy) do { uint8_t v_ = (v); unsigned r_ = a - v_ - (cy); SETF(szyx[r_ & 0xFF] | ((a ^ v_ ^ r_) & HF) | (((a ^ v_) & (a ^ r_) & 0x80) >> 5) | ((r_ >> 8) & CF) | NF); a = (uint8_t)r_; } while (0)
#define ALU_CP(v)      do { uint8_t v_ = (v); unsigned r_ = a - v_; SETF((szyx[r_ & 0xFF] & (SF | ZF)) | (v_ & (YF | XF)) | ((a ^ v_ ^ r_) & HF) | (((a ^ v_) & (a ^ r_) & 0x80) >> 5) | ((r_ >> 8) & CF) | NF); } while (0)
#define ALU_AND(v)     do { a &= (v); SETF(szyxp[a] | HF); } while (0)
#define ALU_OR(v)      do { a |= (v); SETF(szyxp[a]); } while (0)
#define ALU_XOR(v)     do { a ^= (v); SETF(szyxp[a]); } while (0)
#define ALU_INC(v)     ({ uint8_t r_ = (uint8_t)((v) + 1); SETF(szyx[r_] | (f & CF) | ((r_ & 0x0F) == 0 ? HF : 0) | (r_ == 0x80 ? PF : 0)); r_; })
#define ALU_DEC(v)     ({ uint8_t r_ = (uint8_t)((v) - 1); SETF(szyx[r_] | (f & CF) | ((r_ & 0x0F) == 0x0F ? HF : 0) | (r_ == 0x7F ? PF : 0) | NF); r_; })
#define ADD16(x, y)    ({ uint16_t x_ = (x), y_ = (y); unsigned r_ = x_ + y_; wz = (uint16_t)(x_ + 1); SETF((f & (SF | ZF | PF)) | ((r_ >> 8) & (YF | XF)) | (((x_ ^ y_ ^ r_) >> 8) & HF) | (r_ >> 16)); (uint16_t)r_; })
#define ADC16(x, y)    ({ uint16_t x_ = (x), y_ = (y); unsigned r_ = x_ + y_ + (f & CF); wz = (uint16_t)(x_ + 1); SETF(((r_ >> 8) & (SF | YF | XF)) | ((r_ & 0xFFFF) == 0 ? ZF : 0) | (((x_ ^ y_ ^ r_) >> 8) & HF) | (((x_ ^ r_) & (y_ ^ r_) & 0x8000) >> 13) | (r_ >> 16)); (uint16_t)r_; })
#define SBC16(x, y)    ({ uint16_t x_ = (x), y_ = (y); unsigned r_ = x_ - y_ - (f & CF); wz = (uint16_t)(x_ + 1); SETF(((r_ >> 8) & (SF | YF | XF)) | ((r_ & 0xFFFF) == 0 ? ZF : 0) | (((x_ ^ y_ ^ r_) >> 8) & HF) | (((x_ ^ y_) & (x_ ^ r_) & 0x8000) >> 13) | ((r_ >> 16) & CF) | NF); (uint16_t)r_; })
#define OP_RLC(v)  ({ uint8_t v_ = (v); v_ = (uint8_t)((v_ << 1) | (v_ >> 7)); SETF(szyxp[v_] | (v_ & CF)); v_; })
#define OP_RRC(v)  ({ uint8_t v_ = (v); uint8_t c_ = v_ & CF; v_ = (uint8_t)((v_ >> 1) | (v_ << 7)); SETF(szyxp[v_] | c_); v_; })
#define OP_RL(v)   ({ uint8_t v_ = (v); uint8_t c_ = v_ >> 7; v_ = (uint8_t)((v_ << 1) | (f & CF)); SETF(szyxp[v_] | c_); v_; })
#define OP_RR(v)   ({ uint8_t v_ = (v); uint8_t c_ = v_ & CF; v_ = (uint8_t)((v_ >> 1) | (f << 7)); SETF(szyxp[v_] | c_); v_; })
#define OP_SLA(v)  ({ uint8_t v_ = (v); uint8_t c_ = v_ >> 7; v_ <<= 1; SETF(szyxp[v_] | c_); v_; })
#define OP_SRA(v)  ({ uint8_t v_ = (v); uint8_t c_ = v_ & CF; v_ = (uint8_t)((v_ >> 1) | (v_ & 0x80)); SETF(szyxp[v_] | c_); v_; })
#define OP_SLL(v)  ({ uint8_t v_ = (v); uint8_t c_ = v_ >> 7; v_ = (uint8_t)((v_ << 1) | 1); SETF(szyxp[v_] | c_); v_; })
#define OP_SRL(v)  ({ uint8_t v_ = (v); uint8_t c_ = v_ & CF; v_ >>= 1; SETF(szyxp[v_] | c_); v_; })
#define OP_BIT(v, n)            do { uint8_t v_ = (v), r_ = (uint8_t)(v_ & (1 << (n))); SETF((r_ ? (r_ & SF) : (ZF | PF)) | HF | (f & CF) | (v_ & (YF | XF))); } while (0)
#define OP_BIT_MEM(v, n, ah)    do { uint8_t v_ = (v), r_ = (uint8_t)(v_ & (1 << (n))); SETF((r_ ? (r_ & SF) : (ZF | PF)) | HF | (f & CF) | ((ah) & (YF | XF))); } while (0)
// register access by index (0..7: B C D E H L (HL) A)
#define GET_R(r) ({ uint8_t v_; switch (r) { case 0: v_ = B; break; case 1: v_ = C; break; case 2: v_ = D; break; case 3: v_ = E; break; \
                    case 4: v_ = H; break; case 5: v_ = L; break; case 6: v_ = mem_rd(hl); break; default: v_ = a; } v_; })
#define SET_R(r, v) do { uint8_t v_ = (v); switch (r) { case 0: SET_B(v_); break; case 1: SET_C(v_); break; case 2: SET_D(v_); break; case 3: SET_E(v_); break; \
                    case 4: SET_H(v_); break; case 5: SET_L(v_); break; case 6: mem_wr(hl, v_); break; default: a = v_; } } while (0)
#define GET_RP(p) ({ uint16_t v_; switch (p) { case 0: v_ = bc; break; case 1: v_ = de; break; case 2: v_ = hl; break; default: v_ = sp; } v_; })
#define SET_RP_I(p, v) do { uint16_t v_ = (v); switch (p) { case 0: bc = v_; break; case 1: de = v_; break; case 2: hl = v_; break; default: sp = v_; } } while (0)
#define CC(c) ({ bool r_; switch (c) { case 0: r_ = !(f & ZF); break; case 1: r_ = (f & ZF) != 0; break; case 2: r_ = !(f & CF); break; case 3: r_ = (f & CF) != 0; break; \
                 case 4: r_ = !(f & PF); break; case 5: r_ = (f & PF) != 0; break; case 6: r_ = !(f & SF); break; default: r_ = (f & SF) != 0; } r_; })

// ---- the core -----------------------------------------------------------------------------
// `cr` packs the T-states still to run (upper 20 bits, signed) with the M1 count (low 12 bits), so
// the per-instruction bookkeeping is one add at the fetch and one subtract per opcode, and the loop
// head only looks at the sign. Anything that needs attention before the next instruction (interrupt
// acceptance, HALT, the one-instruction EI and LD A,I quirks) calls SLOW(), which pushes cr far
// negative; the slow path undoes that and decides between stopping and continuing.
// A run is at most Z80_RUN_MAX T (see z80_entry.c) so the M1 count fits in 12 bits.
uint8_t z80_trace_on, z80_brk_hit;
uint16_t *const z80_trace_pc = (uint16_t *)PSRAM_WORK_TRC, *const z80_trace_sp = (uint16_t *)(PSRAM_WORK_TRC + 2 * Z80_TRACE_N);   // PSRAM work area (SRAM is full)
uint32_t z80_trace_i;
uint16_t z80_brk_lo = 1, z80_brk_hi = 0, z80_brk_spmin;
static inline bool __attribute__((always_inline)) z80_trace_hook(uint16_t pc_, uint16_t sp_) {
    z80_trace_pc[z80_trace_i & (Z80_TRACE_N - 1)] = pc_; z80_trace_sp[z80_trace_i & (Z80_TRACE_N - 1)] = sp_; z80_trace_i++;
    if ((pc_ >= z80_brk_lo && pc_ <= z80_brk_hi) || (z80_brk_spmin && sp_ < z80_brk_spmin && sp_ >= 0x0100)) { z80_brk_hit = 1; z80_trace_on = 0; return true; }
    return false;
}
void __attribute__((noinline)) z80_core_run(z80_t *zp, uint32_t until) {
    uint16_t sp = zp->sp, wz = zp->wz;
    pc = zp->pc; hl = zp->hl; bc = zp->bc; de = zp->de; a = zp->a; f = zp->f; q = zp->q;
    cr = CR_T(until - zp->cycles);
    SLOW();   // int_line / nmi_pending may have changed outside: start on the slow path
    for (;;) {
        if (__builtin_expect((int32_t)cr < 0, 0)) {
slow:
            if ((int32_t)cr < -(1 << 29)) cr += 1u << 30;
            if ((int32_t)cr < 0) break;
            if (zp->nmi_pending) {                                       // NMI
                zp->nmi_pending = 0; zp->halted = 0; zp->iff1 = 0;
                cr += 1; PUSH(pc); pc = 0x66; wz = 0x66; cr -= CR_T(11);
            } else if (zp->int_line && zp->iff1 && !zp->ei_delay) {     // maskable interrupt
                zp->halted = 0; zp->iff1 = zp->iff2 = 0;
                if (zp->after_ld_ai) { f = (uint8_t)(f & ~PF); q = (q & ~0xFFu) | f; }   // NMOS: P/V cleared when INT follows LD A,I/R
                cr += 1;
                uint8_t vec = int_ack(pc);
                if (zp->im == 2) { PUSH(pc); pc = rd16((uint16_t)((zp->i << 8) | vec)); wz = pc; cr -= CR_T(19); }
                else { PUSH(pc); pc = 0x38; wz = 0x38; cr -= CR_T(13); }   // IM0 (assume RST 38h on the bus) and IM1
            }
            zp->ei_delay = 0; zp->after_ld_ai = 0;
            if (zp->halted) { cr += 1; cr -= CR_T(4); SLOW(); continue; }
            if (zp->nmi_pending | (zp->int_line & zp->iff1)) SLOW();
        }
next:
        if (__builtin_expect(z80_trace_on, 0)) { if (z80_trace_hook(pc, sp)) break; }
        q <<= 8;
        uint8_t op = fetch_op(pc++); cr += 1;
dispatch:
        switch (op & 0xFF) {
            case 0x00: END(4);
            case 0x01: SET_BC(IMM16()); END(10);
            case 0x02: mem_wr(bc, a); wz = (uint16_t)(((uint16_t)a << 8) | ((bc + 1) & 0xFF)); END(7);
            case 0x03: SET_BC((uint16_t)(bc + 1)); END(6);
            case 0x04: SET_B(ALU_INC(B)); END(4);
            case 0x05: SET_B(ALU_DEC(B)); END(4);
            case 0x06: SET_B(IMM8()); END(7);
            case 0x07: { uint8_t c = a >> 7; a = (uint8_t)((a << 1) | c); SETF((f & (SF | ZF | PF)) | (a & (YF | XF)) | c); END(4); }
            case 0x08: { uint8_t t; t = a; a = zp->a2; zp->a2 = t; t = f; f = zp->f2; zp->f2 = t; END(4); }
            case 0x09: SET_HL(ADD16(hl, bc)); END(11);
            case 0x0A: a = mem_rd(bc); wz = (uint16_t)(bc + 1); END(7);
            case 0x0B: SET_BC((uint16_t)(bc - 1)); END(6);
            case 0x0C: SET_C(ALU_INC(C)); END(4);
            case 0x0D: SET_C(ALU_DEC(C)); END(4);
            case 0x0E: SET_C(IMM8()); END(7);
            case 0x0F: { uint8_t c = a & CF; a = (uint8_t)((a >> 1) | (c << 7)); SETF((f & (SF | ZF | PF)) | (a & (YF | XF)) | c); END(4); }
            case 0x10: { int8_t d = (int8_t)IMM8(); bc = (uint16_t)(bc - 0x100); if (B) { pc = (uint16_t)(pc + d); wz = pc; END(13); } END(8); }
            case 0x11: SET_DE(IMM16()); END(10);
            case 0x12: mem_wr(de, a); wz = (uint16_t)(((uint16_t)a << 8) | ((de + 1) & 0xFF)); END(7);
            case 0x13: SET_DE((uint16_t)(de + 1)); END(6);
            case 0x14: SET_D(ALU_INC(D)); END(4);
            case 0x15: SET_D(ALU_DEC(D)); END(4);
            case 0x16: SET_D(IMM8()); END(7);
            case 0x17: { uint8_t c = a >> 7; a = (uint8_t)((a << 1) | (f & CF)); SETF((f & (SF | ZF | PF)) | (a & (YF | XF)) | c); END(4); }
            case 0x18: { int8_t d = (int8_t)IMM8(); pc = (uint16_t)(pc + d); wz = pc; END(12); }
            case 0x19: SET_HL(ADD16(hl, de)); END(11);
            case 0x1A: a = mem_rd(de); wz = (uint16_t)(de + 1); END(7);
            case 0x1B: SET_DE((uint16_t)(de - 1)); END(6);
            case 0x1C: SET_E(ALU_INC(E)); END(4);
            case 0x1D: SET_E(ALU_DEC(E)); END(4);
            case 0x1E: SET_E(IMM8()); END(7);
            case 0x1F: { uint8_t c = a & CF; a = (uint8_t)((a >> 1) | (f << 7)); SETF((f & (SF | ZF | PF)) | (a & (YF | XF)) | c); END(4); }
            case 0x20: { int8_t d = (int8_t)IMM8(); if (!(f & ZF)) { pc = (uint16_t)(pc + d); wz = pc; END(12); } END(7); }
            case 0x28: { int8_t d = (int8_t)IMM8(); if ((f & ZF)) { pc = (uint16_t)(pc + d); wz = pc; END(12); } END(7); }
            case 0x30: { int8_t d = (int8_t)IMM8(); if (!(f & CF)) { pc = (uint16_t)(pc + d); wz = pc; END(12); } END(7); }
            case 0x38: { int8_t d = (int8_t)IMM8(); if ((f & CF)) { pc = (uint16_t)(pc + d); wz = pc; END(12); } END(7); }
            case 0x21: SET_HL(IMM16()); END(10);
            case 0x22: { uint16_t ad = IMM16(); wr16(ad, hl); wz = (uint16_t)(ad + 1); END(16); }
            case 0x23: SET_HL((uint16_t)(hl + 1)); END(6);
            case 0x24: SET_H(ALU_INC(H)); END(4);
            case 0x25: SET_H(ALU_DEC(H)); END(4);
            case 0x26: SET_H(IMM8()); END(7);
            case 0x27: {   // DAA
                uint8_t a0 = a, f0 = f, adj = 0, c = f0 & CF;
                if ((f0 & HF) || (a0 & 0x0F) > 9) adj = 0x06;
                if (c || a0 > 0x99) { adj |= 0x60; c = CF; }
                if (f0 & NF) { a = (uint8_t)(a0 - adj); }
                else { a = (uint8_t)(a0 + adj); }
                uint8_t h = (uint8_t)((a0 ^ a) & HF);
                SETF(szyxp[a] | h | (f0 & NF) | c); END(4); }
            case 0x29: SET_HL(ADD16(hl, hl)); END(11);
            case 0x2A: { uint16_t ad = IMM16(); SET_HL(rd16(ad)); wz = (uint16_t)(ad + 1); END(16); }
            case 0x2B: SET_HL((uint16_t)(hl - 1)); END(6);
            case 0x2C: SET_L(ALU_INC(L)); END(4);
            case 0x2D: SET_L(ALU_DEC(L)); END(4);
            case 0x2E: SET_L(IMM8()); END(7);
            case 0x2F: a = (uint8_t)~a; SETF((f & (SF | ZF | PF | CF)) | (a & (YF | XF)) | HF | NF); END(4);
            case 0x31: sp = IMM16(); END(10);
            case 0x32: { uint16_t ad = IMM16(); mem_wr(ad, a); wz = (uint16_t)(((uint16_t)a << 8) | ((ad + 1) & 0xFF)); END(13); }
            case 0x33: sp++; END(6);
            case 0x34: mem_wr(hl, ALU_INC(mem_rd(hl))); END(11);
            case 0x35: mem_wr(hl, ALU_DEC(mem_rd(hl))); END(11);
            case 0x36: mem_wr(hl, IMM8()); END(10);
            case 0x37: SETF((f & (SF | ZF | PF)) | (((QPREV ^ f) | a) & (YF | XF)) | CF); END(4);   // SCF
            case 0x39: SET_HL(ADD16(hl, sp)); END(11);
            case 0x3A: { uint16_t ad = IMM16(); a = mem_rd(ad); wz = (uint16_t)(ad + 1); END(13); }
            case 0x3B: sp--; END(6);
            case 0x3C: a = ALU_INC(a); END(4);
            case 0x3D: a = ALU_DEC(a); END(4);
            case 0x3E: a = IMM8(); END(7);
            case 0x3F: SETF((f & (SF | ZF | PF)) | (((QPREV ^ f) | a) & (YF | XF)) | ((f & CF) << 4) | ((f & CF) ^ CF)); END(4);   // CCF
            case 0x76: zp->halted = 1; SLOW(); END(4);   // PC already points past HALT (pushed as-is on interrupt)
            // LD r,r' (0x40-0x7F except 0x76)
            case 0x40: SET_B(B); END(4);
            case 0x41: SET_B(C); END(4);
            case 0x42: SET_B(D); END(4);
            case 0x43: SET_B(E); END(4);
            case 0x44: SET_B(H); END(4);
            case 0x45: SET_B(L); END(4);
            case 0x46: SET_B(mem_rd(hl)); END(7);
            case 0x47: SET_B(a); END(4);
            case 0x48: SET_C(B); END(4);
            case 0x49: SET_C(C); END(4);
            case 0x4A: SET_C(D); END(4);
            case 0x4B: SET_C(E); END(4);
            case 0x4C: SET_C(H); END(4);
            case 0x4D: SET_C(L); END(4);
            case 0x4E: SET_C(mem_rd(hl)); END(7);
            case 0x4F: SET_C(a); END(4);
            case 0x50: SET_D(B); END(4);
            case 0x51: SET_D(C); END(4);
            case 0x52: SET_D(D); END(4);
            case 0x53: SET_D(E); END(4);
            case 0x54: SET_D(H); END(4);
            case 0x55: SET_D(L); END(4);
            case 0x56: SET_D(mem_rd(hl)); END(7);
            case 0x57: SET_D(a); END(4);
            case 0x58: SET_E(B); END(4);
            case 0x59: SET_E(C); END(4);
            case 0x5A: SET_E(D); END(4);
            case 0x5B: SET_E(E); END(4);
            case 0x5C: SET_E(H); END(4);
            case 0x5D: SET_E(L); END(4);
            case 0x5E: SET_E(mem_rd(hl)); END(7);
            case 0x5F: SET_E(a); END(4);
            case 0x60: SET_H(B); END(4);
            case 0x61: SET_H(C); END(4);
            case 0x62: SET_H(D); END(4);
            case 0x63: SET_H(E); END(4);
            case 0x64: SET_H(H); END(4);
            case 0x65: SET_H(L); END(4);
            case 0x66: SET_H(mem_rd(hl)); END(7);
            case 0x67: SET_H(a); END(4);
            case 0x68: SET_L(B); END(4);
            case 0x69: SET_L(C); END(4);
            case 0x6A: SET_L(D); END(4);
            case 0x6B: SET_L(E); END(4);
            case 0x6C: SET_L(H); END(4);
            case 0x6D: SET_L(L); END(4);
            case 0x6E: SET_L(mem_rd(hl)); END(7);
            case 0x6F: SET_L(a); END(4);
            case 0x70: mem_wr(hl, B); END(7);
            case 0x71: mem_wr(hl, C); END(7);
            case 0x72: mem_wr(hl, D); END(7);
            case 0x73: mem_wr(hl, E); END(7);
            case 0x74: mem_wr(hl, H); END(7);
            case 0x75: mem_wr(hl, L); END(7);
            case 0x77: mem_wr(hl, a); END(7);
            case 0x78: a = B; END(4);
            case 0x79: a = C; END(4);
            case 0x7A: a = D; END(4);
            case 0x7B: a = E; END(4);
            case 0x7C: a = H; END(4);
            case 0x7D: a = L; END(4);
            case 0x7E: a = mem_rd(hl); END(7);
            case 0x7F: a = a; END(4);
            // ALU A,r
            case 0x80: ALU_ADD(B, 0); END(4);
            case 0x81: ALU_ADD(C, 0); END(4);
            case 0x82: ALU_ADD(D, 0); END(4);
            case 0x83: ALU_ADD(E, 0); END(4);
            case 0x84: ALU_ADD(H, 0); END(4);
            case 0x85: ALU_ADD(L, 0); END(4);
            case 0x86: ALU_ADD(mem_rd(hl), 0); END(7);
            case 0x87: ALU_ADD(a, 0); END(4);
            case 0x88: ALU_ADD(B, f & CF); END(4);
            case 0x89: ALU_ADD(C, f & CF); END(4);
            case 0x8A: ALU_ADD(D, f & CF); END(4);
            case 0x8B: ALU_ADD(E, f & CF); END(4);
            case 0x8C: ALU_ADD(H, f & CF); END(4);
            case 0x8D: ALU_ADD(L, f & CF); END(4);
            case 0x8E: ALU_ADD(mem_rd(hl), f & CF); END(7);
            case 0x8F: ALU_ADD(a, f & CF); END(4);
            case 0x90: ALU_SUB(B, 0); END(4);
            case 0x91: ALU_SUB(C, 0); END(4);
            case 0x92: ALU_SUB(D, 0); END(4);
            case 0x93: ALU_SUB(E, 0); END(4);
            case 0x94: ALU_SUB(H, 0); END(4);
            case 0x95: ALU_SUB(L, 0); END(4);
            case 0x96: ALU_SUB(mem_rd(hl), 0); END(7);
            case 0x97: ALU_SUB(a, 0); END(4);
            case 0x98: ALU_SUB(B, f & CF); END(4);
            case 0x99: ALU_SUB(C, f & CF); END(4);
            case 0x9A: ALU_SUB(D, f & CF); END(4);
            case 0x9B: ALU_SUB(E, f & CF); END(4);
            case 0x9C: ALU_SUB(H, f & CF); END(4);
            case 0x9D: ALU_SUB(L, f & CF); END(4);
            case 0x9E: ALU_SUB(mem_rd(hl), f & CF); END(7);
            case 0x9F: ALU_SUB(a, f & CF); END(4);
            case 0xA0: ALU_AND(B); END(4);
            case 0xA1: ALU_AND(C); END(4);
            case 0xA2: ALU_AND(D); END(4);
            case 0xA3: ALU_AND(E); END(4);
            case 0xA4: ALU_AND(H); END(4);
            case 0xA5: ALU_AND(L); END(4);
            case 0xA6: ALU_AND(mem_rd(hl)); END(7);
            case 0xA7: ALU_AND(a); END(4);
            case 0xA8: ALU_XOR(B); END(4);
            case 0xA9: ALU_XOR(C); END(4);
            case 0xAA: ALU_XOR(D); END(4);
            case 0xAB: ALU_XOR(E); END(4);
            case 0xAC: ALU_XOR(H); END(4);
            case 0xAD: ALU_XOR(L); END(4);
            case 0xAE: ALU_XOR(mem_rd(hl)); END(7);
            case 0xAF: ALU_XOR(a); END(4);
            case 0xB0: ALU_OR(B); END(4);
            case 0xB1: ALU_OR(C); END(4);
            case 0xB2: ALU_OR(D); END(4);
            case 0xB3: ALU_OR(E); END(4);
            case 0xB4: ALU_OR(H); END(4);
            case 0xB5: ALU_OR(L); END(4);
            case 0xB6: ALU_OR(mem_rd(hl)); END(7);
            case 0xB7: ALU_OR(a); END(4);
            case 0xB8: ALU_CP(B); END(4);
            case 0xB9: ALU_CP(C); END(4);
            case 0xBA: ALU_CP(D); END(4);
            case 0xBB: ALU_CP(E); END(4);
            case 0xBC: ALU_CP(H); END(4);
            case 0xBD: ALU_CP(L); END(4);
            case 0xBE: ALU_CP(mem_rd(hl)); END(7);
            case 0xBF: ALU_CP(a); END(4);
            case 0xC0: if (!(f & ZF)) { pc = POP(); wz = pc; END(11); } END(5);
            case 0xC8: if ((f & ZF)) { pc = POP(); wz = pc; END(11); } END(5);
            case 0xD0: if (!(f & CF)) { pc = POP(); wz = pc; END(11); } END(5);
            case 0xD8: if ((f & CF)) { pc = POP(); wz = pc; END(11); } END(5);
            case 0xE0: if (!(f & PF)) { pc = POP(); wz = pc; END(11); } END(5);
            case 0xE8: if ((f & PF)) { pc = POP(); wz = pc; END(11); } END(5);
            case 0xF0: if (!(f & SF)) { pc = POP(); wz = pc; END(11); } END(5);
            case 0xF8: if ((f & SF)) { pc = POP(); wz = pc; END(11); } END(5);
            case 0xC1: SET_BC(POP()); END(10);
            case 0xD1: SET_DE(POP()); END(10);
            case 0xE1: SET_HL(POP()); END(10);
            case 0xF1: SET_AF(POP()); END(10);
            case 0xC2: { uint16_t ad = IMM16(); wz = ad; if (!(f & ZF)) pc = ad; END(10); }
            case 0xCA: { uint16_t ad = IMM16(); wz = ad; if ((f & ZF)) pc = ad; END(10); }
            case 0xD2: { uint16_t ad = IMM16(); wz = ad; if (!(f & CF)) pc = ad; END(10); }
            case 0xDA: { uint16_t ad = IMM16(); wz = ad; if ((f & CF)) pc = ad; END(10); }
            case 0xE2: { uint16_t ad = IMM16(); wz = ad; if (!(f & PF)) pc = ad; END(10); }
            case 0xEA: { uint16_t ad = IMM16(); wz = ad; if ((f & PF)) pc = ad; END(10); }
            case 0xF2: { uint16_t ad = IMM16(); wz = ad; if (!(f & SF)) pc = ad; END(10); }
            case 0xFA: { uint16_t ad = IMM16(); wz = ad; if ((f & SF)) pc = ad; END(10); }
            case 0xC3: pc = IMM16(); wz = pc; END(10);
            case 0xC4: { uint16_t ad = IMM16(); wz = ad; if (!(f & ZF)) { PUSH(pc); pc = ad; END(17); } END(10); }
            case 0xCC: { uint16_t ad = IMM16(); wz = ad; if ((f & ZF)) { PUSH(pc); pc = ad; END(17); } END(10); }
            case 0xD4: { uint16_t ad = IMM16(); wz = ad; if (!(f & CF)) { PUSH(pc); pc = ad; END(17); } END(10); }
            case 0xDC: { uint16_t ad = IMM16(); wz = ad; if ((f & CF)) { PUSH(pc); pc = ad; END(17); } END(10); }
            case 0xE4: { uint16_t ad = IMM16(); wz = ad; if (!(f & PF)) { PUSH(pc); pc = ad; END(17); } END(10); }
            case 0xEC: { uint16_t ad = IMM16(); wz = ad; if ((f & PF)) { PUSH(pc); pc = ad; END(17); } END(10); }
            case 0xF4: { uint16_t ad = IMM16(); wz = ad; if (!(f & SF)) { PUSH(pc); pc = ad; END(17); } END(10); }
            case 0xFC: { uint16_t ad = IMM16(); wz = ad; if ((f & SF)) { PUSH(pc); pc = ad; END(17); } END(10); }
            case 0xC5: PUSH(bc); END(11);
            case 0xD5: PUSH(de); END(11);
            case 0xE5: PUSH(hl); END(11);
            case 0xF5: PUSH(AFV); END(11);
            case 0xC6: ALU_ADD(IMM8(), 0); END(7);
            case 0xCE: ALU_ADD(IMM8(), f & CF); END(7);
            case 0xD6: ALU_SUB(IMM8(), 0); END(7);
            case 0xDE: ALU_SUB(IMM8(), f & CF); END(7);
            case 0xE6: ALU_AND(IMM8()); END(7);
            case 0xEE: ALU_XOR(IMM8()); END(7);
            case 0xF6: ALU_OR(IMM8()); END(7);
            case 0xFE: ALU_CP(IMM8()); END(7);
            case 0xC7: PUSH(pc); pc = 0x00; wz = pc; END(11);
            case 0xCF: PUSH(pc); pc = 0x08; wz = pc; END(11);
            case 0xD7: PUSH(pc); pc = 0x10; wz = pc; END(11);
            case 0xDF: PUSH(pc); pc = 0x18; wz = pc; END(11);
            case 0xE7: PUSH(pc); pc = 0x20; wz = pc; END(11);
            case 0xEF: PUSH(pc); pc = 0x28; wz = pc; END(11);
            case 0xF7: PUSH(pc); pc = 0x30; wz = pc; END(11);
            case 0xFF: PUSH(pc); pc = 0x38; wz = pc; END(11);
            case 0xC9: pc = POP(); wz = pc; END(10);
            case 0xCB: { uint8_t cop = fetch_op((uint16_t)pc); pc = (uint16_t)(pc + 1); cr += 1; int r = cop & 7, n = (cop >> 3) & 7; uint8_t v = GET_R(r);
                  switch (cop >> 6) {
                      case 0:
                          switch (n) { case 0: v = OP_RLC(v); break; case 1: v = OP_RRC(v); break; case 2: v = OP_RL(v); break; case 3: v = OP_RR(v); break;
                                       case 4: v = OP_SLA(v); break; case 5: v = OP_SRA(v); break; case 6: v = OP_SLL(v); break; default: v = OP_SRL(v); }
                          SET_R(r, v); END(r == 6 ? 15 : 8);
                      case 1:
                          if (r == 6) { OP_BIT_MEM(v, n, (uint8_t)(wz >> 8)); cr -= CR_T(12); } else { OP_BIT(v, n); cr -= CR_T(8); }
                          break;
                      case 2: SET_R(r, (uint8_t)(v & ~(1 << n))); END(r == 6 ? 15 : 8);
                      default: SET_R(r, (uint8_t)(v | (1 << n))); END(r == 6 ? 15 : 8);
                  } } break;
            case 0xCD: { uint16_t ad = IMM16(); PUSH(pc); pc = ad; wz = ad; END(17); }
            case 0xD3: { uint8_t p = IMM8(); IOWR((uint16_t)(((uint16_t)a << 8) | p), a); wz = (uint16_t)(((uint16_t)a << 8) | ((p + 1) & 0xFF)); END(11); }
            case 0xDB: { uint8_t p = IMM8(); uint16_t port = (uint16_t)(((uint16_t)a << 8) | p); a = IORD(port); wz = (uint16_t)(port + 1); END(11); }
            case 0xD9: { uint8_t t;
                t = B; SET_B(zp->b2); zp->b2 = t; t = C; SET_C(zp->c2); zp->c2 = t;
                t = D; SET_D(zp->d2); zp->d2 = t; t = E; SET_E(zp->e2); zp->e2 = t;
                t = H; SET_H(zp->h2); zp->h2 = t; t = L; SET_L(zp->l2); zp->l2 = t; END(4); }
            case 0xDD: { uint16_t xy = zp->ix; uint8_t xop = fetch_op((uint16_t)pc); pc = (uint16_t)(pc + 1); cr += 1;
              uint8_t xh = (uint8_t)(xy >> 8), xl = (uint8_t)xy; (void)xh; (void)xl;
            #define XY_D() ({ int8_t d_ = (int8_t)IMM8(); uint16_t a_ = (uint16_t)(xy + d_); wz = a_; a_; })
                  switch (xop) {
                      case 0x09: xy = ADD16(xy, bc); cr -= CR_T(15); break;
                      case 0x19: xy = ADD16(xy, de); cr -= CR_T(15); break;
                      case 0x29: xy = ADD16(xy, xy); cr -= CR_T(15); break;
                      case 0x39: xy = ADD16(xy, sp); cr -= CR_T(15); break;
                      case 0x21: xy = IMM16(); cr -= CR_T(14); break;
                      case 0x22: { uint16_t ad = IMM16(); wr16(ad, xy); wz = (uint16_t)(ad + 1); cr -= CR_T(20); break; }
                      case 0x2A: { uint16_t ad = IMM16(); xy = rd16(ad); wz = (uint16_t)(ad + 1); cr -= CR_T(20); break; }
                      case 0x23: xy++; cr -= CR_T(10); break;
                      case 0x2B: xy--; cr -= CR_T(10); break;
                      case 0x24: xh = ALU_INC(xh); xy = (uint16_t)((xh << 8) | xl); cr -= CR_T(8); break;
                      case 0x25: xh = ALU_DEC(xh); xy = (uint16_t)((xh << 8) | xl); cr -= CR_T(8); break;
                      case 0x26: xh = IMM8(); xy = (uint16_t)((xh << 8) | xl); cr -= CR_T(11); break;
                      case 0x2C: xl = ALU_INC(xl); xy = (uint16_t)((xh << 8) | xl); cr -= CR_T(8); break;
                      case 0x2D: xl = ALU_DEC(xl); xy = (uint16_t)((xh << 8) | xl); cr -= CR_T(8); break;
                      case 0x2E: xl = IMM8(); xy = (uint16_t)((xh << 8) | xl); cr -= CR_T(11); break;
                      case 0x34: { uint16_t ad = XY_D(); mem_wr(ad, ALU_INC(mem_rd(ad))); cr -= CR_T(23); break; }
                      case 0x35: { uint16_t ad = XY_D(); mem_wr(ad, ALU_DEC(mem_rd(ad))); cr -= CR_T(23); break; }
                      case 0x36: { uint16_t ad = XY_D(); mem_wr(ad, IMM8()); cr -= CR_T(19); break; }
                      case 0x44: SET_B(xh); cr -= CR_T(8); break;  case 0x45: SET_B(xl); cr -= CR_T(8); break;
                      case 0x4C: SET_C(xh); cr -= CR_T(8); break;  case 0x4D: SET_C(xl); cr -= CR_T(8); break;
                      case 0x54: SET_D(xh); cr -= CR_T(8); break;  case 0x55: SET_D(xl); cr -= CR_T(8); break;
                      case 0x5C: SET_E(xh); cr -= CR_T(8); break;  case 0x5D: SET_E(xl); cr -= CR_T(8); break;
                      case 0x7C: a = xh; cr -= CR_T(8); break;  case 0x7D: a = xl; cr -= CR_T(8); break;
                      case 0x60: xh = B; xy = (uint16_t)((xh << 8) | xl); cr -= CR_T(8); break;
                      case 0x61: xh = C; xy = (uint16_t)((xh << 8) | xl); cr -= CR_T(8); break;
                      case 0x62: xh = D; xy = (uint16_t)((xh << 8) | xl); cr -= CR_T(8); break;
                      case 0x63: xh = E; xy = (uint16_t)((xh << 8) | xl); cr -= CR_T(8); break;
                      case 0x64: cr -= CR_T(8); break;
                      case 0x65: xh = xl; xy = (uint16_t)((xh << 8) | xl); cr -= CR_T(8); break;
                      case 0x67: xh = a; xy = (uint16_t)((xh << 8) | xl); cr -= CR_T(8); break;
                      case 0x68: xl = B; xy = (uint16_t)((xh << 8) | xl); cr -= CR_T(8); break;
                      case 0x69: xl = C; xy = (uint16_t)((xh << 8) | xl); cr -= CR_T(8); break;
                      case 0x6A: xl = D; xy = (uint16_t)((xh << 8) | xl); cr -= CR_T(8); break;
                      case 0x6B: xl = E; xy = (uint16_t)((xh << 8) | xl); cr -= CR_T(8); break;
                      case 0x6C: xl = xh; xy = (uint16_t)((xh << 8) | xl); cr -= CR_T(8); break;
                      case 0x6D: cr -= CR_T(8); break;
                      case 0x6F: xl = a; xy = (uint16_t)((xh << 8) | xl); cr -= CR_T(8); break;
                      case 0x46: case 0x4E: case 0x56: case 0x5E: case 0x66: case 0x6E: case 0x7E: {   // LD r,(IX+d)
                          uint16_t ad = XY_D(); SET_R((xop >> 3) & 7, mem_rd(ad)); cr -= CR_T(19); break; }
                      case 0x70: case 0x71: case 0x72: case 0x73: case 0x74: case 0x75: case 0x77: {   // LD (IX+d),r
                          uint16_t ad = XY_D(); mem_wr(ad, GET_R(xop & 7)); cr -= CR_T(19); break; }
                      case 0x84: ALU_ADD(xh, 0); cr -= CR_T(8); break;  case 0x85: ALU_ADD(xl, 0); cr -= CR_T(8); break;
                      case 0x8C: ALU_ADD(xh, f & CF); cr -= CR_T(8); break;  case 0x8D: ALU_ADD(xl, f & CF); cr -= CR_T(8); break;
                      case 0x94: ALU_SUB(xh, 0); cr -= CR_T(8); break;  case 0x95: ALU_SUB(xl, 0); cr -= CR_T(8); break;
                      case 0x9C: ALU_SUB(xh, f & CF); cr -= CR_T(8); break;  case 0x9D: ALU_SUB(xl, f & CF); cr -= CR_T(8); break;
                      case 0xA4: ALU_AND(xh); cr -= CR_T(8); break;  case 0xA5: ALU_AND(xl); cr -= CR_T(8); break;
                      case 0xAC: ALU_XOR(xh); cr -= CR_T(8); break;  case 0xAD: ALU_XOR(xl); cr -= CR_T(8); break;
                      case 0xB4: ALU_OR(xh); cr -= CR_T(8); break;   case 0xB5: ALU_OR(xl); cr -= CR_T(8); break;
                      case 0xBC: ALU_CP(xh); cr -= CR_T(8); break;   case 0xBD: ALU_CP(xl); cr -= CR_T(8); break;
                      case 0x86: { uint16_t ad = XY_D(); ALU_ADD(mem_rd(ad), 0); cr -= CR_T(19); break; }
                      case 0x8E: { uint16_t ad = XY_D(); ALU_ADD(mem_rd(ad), f & CF); cr -= CR_T(19); break; }
                      case 0x96: { uint16_t ad = XY_D(); ALU_SUB(mem_rd(ad), 0); cr -= CR_T(19); break; }
                      case 0x9E: { uint16_t ad = XY_D(); ALU_SUB(mem_rd(ad), f & CF); cr -= CR_T(19); break; }
                      case 0xA6: { uint16_t ad = XY_D(); ALU_AND(mem_rd(ad)); cr -= CR_T(19); break; }
                      case 0xAE: { uint16_t ad = XY_D(); ALU_XOR(mem_rd(ad)); cr -= CR_T(19); break; }
                      case 0xB6: { uint16_t ad = XY_D(); ALU_OR(mem_rd(ad)); cr -= CR_T(19); break; }
                      case 0xBE: { uint16_t ad = XY_D(); ALU_CP(mem_rd(ad)); cr -= CR_T(19); break; }
                      case 0xCB: { int8_t d = (int8_t)IMM8(); uint8_t xcop = IMM8();
                        uint16_t xa = (uint16_t)(xy + d); wz = xa; int r = xcop & 7, n = (xcop >> 3) & 7; uint8_t v = mem_rd(xa);
                            switch (xcop >> 6) {
                                case 0:
                                    switch (n) { case 0: v = OP_RLC(v); break; case 1: v = OP_RRC(v); break; case 2: v = OP_RL(v); break; case 3: v = OP_RR(v); break;
                                                 case 4: v = OP_SLA(v); break; case 5: v = OP_SRA(v); break; case 6: v = OP_SLL(v); break; default: v = OP_SRL(v); }
                                    break;
                                case 1: OP_BIT_MEM(v, n, (uint8_t)(xa >> 8)); cr -= CR_T(20); goto xycb_done_ix;
                                case 2: v = (uint8_t)(v & ~(1 << n)); break;
                                default: v = (uint8_t)(v | (1 << n)); break;
                            }
                        mem_wr(xa, v); if (r != 6) SET_R(r, v); cr -= CR_T(23); xycb_done_ix: ; } break;
                      case 0xE1: xy = POP(); cr -= CR_T(14); break;
                      case 0xE3: { uint16_t t = rd16(sp); wr16(sp, xy); xy = t; wz = t; cr -= CR_T(23); break; }
                      case 0xE5: PUSH(xy); cr -= CR_T(15); break;
                      case 0xE9: pc = xy; cr -= CR_T(8); break;
                      case 0xF9: sp = xy; cr -= CR_T(10); break;
                      default:
                          // Prefix has no effect on this opcode: it costs 4 cycles and the opcode runs unprefixed.
                          // The prefix counts as an instruction that left F alone (SCF/CCF see Q = 0).
                          cr -= CR_T(4); q &= 0xFFu;   // prefix wrote no flags: neither "previous" nor "current"
                          op = xop; goto dispatch;   // run the opcode unprefixed, as part of this instruction (no interrupt in between)
                          break;
                  }
            #undef XY_D
              zp->ix = xy; } break;
            case 0xFD: { uint16_t xy = zp->iy; uint8_t xop = fetch_op((uint16_t)pc); pc = (uint16_t)(pc + 1); cr += 1;
              uint8_t xh = (uint8_t)(xy >> 8), xl = (uint8_t)xy; (void)xh; (void)xl;
            #define XY_D() ({ int8_t d_ = (int8_t)IMM8(); uint16_t a_ = (uint16_t)(xy + d_); wz = a_; a_; })
                  switch (xop) {
                      case 0x09: xy = ADD16(xy, bc); cr -= CR_T(15); break;
                      case 0x19: xy = ADD16(xy, de); cr -= CR_T(15); break;
                      case 0x29: xy = ADD16(xy, xy); cr -= CR_T(15); break;
                      case 0x39: xy = ADD16(xy, sp); cr -= CR_T(15); break;
                      case 0x21: xy = IMM16(); cr -= CR_T(14); break;
                      case 0x22: { uint16_t ad = IMM16(); wr16(ad, xy); wz = (uint16_t)(ad + 1); cr -= CR_T(20); break; }
                      case 0x2A: { uint16_t ad = IMM16(); xy = rd16(ad); wz = (uint16_t)(ad + 1); cr -= CR_T(20); break; }
                      case 0x23: xy++; cr -= CR_T(10); break;
                      case 0x2B: xy--; cr -= CR_T(10); break;
                      case 0x24: xh = ALU_INC(xh); xy = (uint16_t)((xh << 8) | xl); cr -= CR_T(8); break;
                      case 0x25: xh = ALU_DEC(xh); xy = (uint16_t)((xh << 8) | xl); cr -= CR_T(8); break;
                      case 0x26: xh = IMM8(); xy = (uint16_t)((xh << 8) | xl); cr -= CR_T(11); break;
                      case 0x2C: xl = ALU_INC(xl); xy = (uint16_t)((xh << 8) | xl); cr -= CR_T(8); break;
                      case 0x2D: xl = ALU_DEC(xl); xy = (uint16_t)((xh << 8) | xl); cr -= CR_T(8); break;
                      case 0x2E: xl = IMM8(); xy = (uint16_t)((xh << 8) | xl); cr -= CR_T(11); break;
                      case 0x34: { uint16_t ad = XY_D(); mem_wr(ad, ALU_INC(mem_rd(ad))); cr -= CR_T(23); break; }
                      case 0x35: { uint16_t ad = XY_D(); mem_wr(ad, ALU_DEC(mem_rd(ad))); cr -= CR_T(23); break; }
                      case 0x36: { uint16_t ad = XY_D(); mem_wr(ad, IMM8()); cr -= CR_T(19); break; }
                      case 0x44: SET_B(xh); cr -= CR_T(8); break;  case 0x45: SET_B(xl); cr -= CR_T(8); break;
                      case 0x4C: SET_C(xh); cr -= CR_T(8); break;  case 0x4D: SET_C(xl); cr -= CR_T(8); break;
                      case 0x54: SET_D(xh); cr -= CR_T(8); break;  case 0x55: SET_D(xl); cr -= CR_T(8); break;
                      case 0x5C: SET_E(xh); cr -= CR_T(8); break;  case 0x5D: SET_E(xl); cr -= CR_T(8); break;
                      case 0x7C: a = xh; cr -= CR_T(8); break;  case 0x7D: a = xl; cr -= CR_T(8); break;
                      case 0x60: xh = B; xy = (uint16_t)((xh << 8) | xl); cr -= CR_T(8); break;
                      case 0x61: xh = C; xy = (uint16_t)((xh << 8) | xl); cr -= CR_T(8); break;
                      case 0x62: xh = D; xy = (uint16_t)((xh << 8) | xl); cr -= CR_T(8); break;
                      case 0x63: xh = E; xy = (uint16_t)((xh << 8) | xl); cr -= CR_T(8); break;
                      case 0x64: cr -= CR_T(8); break;
                      case 0x65: xh = xl; xy = (uint16_t)((xh << 8) | xl); cr -= CR_T(8); break;
                      case 0x67: xh = a; xy = (uint16_t)((xh << 8) | xl); cr -= CR_T(8); break;
                      case 0x68: xl = B; xy = (uint16_t)((xh << 8) | xl); cr -= CR_T(8); break;
                      case 0x69: xl = C; xy = (uint16_t)((xh << 8) | xl); cr -= CR_T(8); break;
                      case 0x6A: xl = D; xy = (uint16_t)((xh << 8) | xl); cr -= CR_T(8); break;
                      case 0x6B: xl = E; xy = (uint16_t)((xh << 8) | xl); cr -= CR_T(8); break;
                      case 0x6C: xl = xh; xy = (uint16_t)((xh << 8) | xl); cr -= CR_T(8); break;
                      case 0x6D: cr -= CR_T(8); break;
                      case 0x6F: xl = a; xy = (uint16_t)((xh << 8) | xl); cr -= CR_T(8); break;
                      case 0x46: case 0x4E: case 0x56: case 0x5E: case 0x66: case 0x6E: case 0x7E: {   // LD r,(IX+d)
                          uint16_t ad = XY_D(); SET_R((xop >> 3) & 7, mem_rd(ad)); cr -= CR_T(19); break; }
                      case 0x70: case 0x71: case 0x72: case 0x73: case 0x74: case 0x75: case 0x77: {   // LD (IX+d),r
                          uint16_t ad = XY_D(); mem_wr(ad, GET_R(xop & 7)); cr -= CR_T(19); break; }
                      case 0x84: ALU_ADD(xh, 0); cr -= CR_T(8); break;  case 0x85: ALU_ADD(xl, 0); cr -= CR_T(8); break;
                      case 0x8C: ALU_ADD(xh, f & CF); cr -= CR_T(8); break;  case 0x8D: ALU_ADD(xl, f & CF); cr -= CR_T(8); break;
                      case 0x94: ALU_SUB(xh, 0); cr -= CR_T(8); break;  case 0x95: ALU_SUB(xl, 0); cr -= CR_T(8); break;
                      case 0x9C: ALU_SUB(xh, f & CF); cr -= CR_T(8); break;  case 0x9D: ALU_SUB(xl, f & CF); cr -= CR_T(8); break;
                      case 0xA4: ALU_AND(xh); cr -= CR_T(8); break;  case 0xA5: ALU_AND(xl); cr -= CR_T(8); break;
                      case 0xAC: ALU_XOR(xh); cr -= CR_T(8); break;  case 0xAD: ALU_XOR(xl); cr -= CR_T(8); break;
                      case 0xB4: ALU_OR(xh); cr -= CR_T(8); break;   case 0xB5: ALU_OR(xl); cr -= CR_T(8); break;
                      case 0xBC: ALU_CP(xh); cr -= CR_T(8); break;   case 0xBD: ALU_CP(xl); cr -= CR_T(8); break;
                      case 0x86: { uint16_t ad = XY_D(); ALU_ADD(mem_rd(ad), 0); cr -= CR_T(19); break; }
                      case 0x8E: { uint16_t ad = XY_D(); ALU_ADD(mem_rd(ad), f & CF); cr -= CR_T(19); break; }
                      case 0x96: { uint16_t ad = XY_D(); ALU_SUB(mem_rd(ad), 0); cr -= CR_T(19); break; }
                      case 0x9E: { uint16_t ad = XY_D(); ALU_SUB(mem_rd(ad), f & CF); cr -= CR_T(19); break; }
                      case 0xA6: { uint16_t ad = XY_D(); ALU_AND(mem_rd(ad)); cr -= CR_T(19); break; }
                      case 0xAE: { uint16_t ad = XY_D(); ALU_XOR(mem_rd(ad)); cr -= CR_T(19); break; }
                      case 0xB6: { uint16_t ad = XY_D(); ALU_OR(mem_rd(ad)); cr -= CR_T(19); break; }
                      case 0xBE: { uint16_t ad = XY_D(); ALU_CP(mem_rd(ad)); cr -= CR_T(19); break; }
                      case 0xCB: { int8_t d = (int8_t)IMM8(); uint8_t xcop = IMM8();
                        uint16_t xa = (uint16_t)(xy + d); wz = xa; int r = xcop & 7, n = (xcop >> 3) & 7; uint8_t v = mem_rd(xa);
                            switch (xcop >> 6) {
                                case 0:
                                    switch (n) { case 0: v = OP_RLC(v); break; case 1: v = OP_RRC(v); break; case 2: v = OP_RL(v); break; case 3: v = OP_RR(v); break;
                                                 case 4: v = OP_SLA(v); break; case 5: v = OP_SRA(v); break; case 6: v = OP_SLL(v); break; default: v = OP_SRL(v); }
                                    break;
                                case 1: OP_BIT_MEM(v, n, (uint8_t)(xa >> 8)); cr -= CR_T(20); goto xycb_done_iy;
                                case 2: v = (uint8_t)(v & ~(1 << n)); break;
                                default: v = (uint8_t)(v | (1 << n)); break;
                            }
                        mem_wr(xa, v); if (r != 6) SET_R(r, v); cr -= CR_T(23); xycb_done_iy: ; } break;
                      case 0xE1: xy = POP(); cr -= CR_T(14); break;
                      case 0xE3: { uint16_t t = rd16(sp); wr16(sp, xy); xy = t; wz = t; cr -= CR_T(23); break; }
                      case 0xE5: PUSH(xy); cr -= CR_T(15); break;
                      case 0xE9: pc = xy; cr -= CR_T(8); break;
                      case 0xF9: sp = xy; cr -= CR_T(10); break;
                      default:
                          // Prefix has no effect on this opcode: it costs 4 cycles and the opcode runs unprefixed.
                          // The prefix counts as an instruction that left F alone (SCF/CCF see Q = 0).
                          cr -= CR_T(4); q &= 0xFFu;   // prefix wrote no flags: neither "previous" nor "current"
                          op = xop; goto dispatch;   // run the opcode unprefixed, as part of this instruction (no interrupt in between)
                          break;
                  }
            #undef XY_D
              zp->iy = xy; } break;
            case 0xE3: { uint16_t t = rd16(sp); wr16(sp, hl); SET_HL(t); wz = t; END(19); }
            case 0xE9: pc = hl; END(4);
            case 0xEB: { uint8_t t; t = D; SET_D(H); SET_H(t); t = E; SET_E(L); SET_L(t); END(4); }
            case 0xED: { uint8_t eop = fetch_op((uint16_t)pc); pc = (uint16_t)(pc + 1); cr += 1;
                  switch (eop) {
                      case 0x40: case 0x48: case 0x50: case 0x58: case 0x60: case 0x68: case 0x78: {   // IN r,(C)
                          wz = (uint16_t)(bc + 1); uint8_t v = IORD(bc); SET_R((eop >> 3) & 7, v);
                          SETF(szyxp[v] | (f & CF)); END(12); }
                      case 0x70: { wz = (uint16_t)(bc + 1); uint8_t v = IORD(bc); SETF(szyxp[v] | (f & CF)); END(12); }
                      case 0x41: case 0x49: case 0x51: case 0x59: case 0x61: case 0x69: case 0x79:     // OUT (C),r
                          wz = (uint16_t)(bc + 1); IOWR(bc, GET_R((eop >> 3) & 7)); END(12);
                      case 0x71: wz = (uint16_t)(bc + 1); IOWR(bc, 0); END(12);
                      case 0x42: case 0x52: case 0x62: case 0x72: SET_HL(SBC16(hl, GET_RP((eop >> 4) & 3))); END(15);
                      case 0x4A: case 0x5A: case 0x6A: case 0x7A: SET_HL(ADC16(hl, GET_RP((eop >> 4) & 3))); END(15);
                      case 0x43: case 0x53: case 0x63: case 0x73: { uint16_t ad = IMM16(); wr16(ad, GET_RP((eop >> 4) & 3)); wz = (uint16_t)(ad + 1); END(20); }
                      case 0x4B: case 0x5B: case 0x6B: case 0x7B: { uint16_t ad = IMM16(); SET_RP_I((eop >> 4) & 3, rd16(ad)); wz = (uint16_t)(ad + 1); END(20); }
                      case 0x44: case 0x4C: case 0x54: case 0x5C: case 0x64: case 0x6C: case 0x74: case 0x7C: {   // NEG
                          uint8_t a0 = a; a = 0; ALU_SUB(a0, 0); END(8); }
                      case 0x45: case 0x4D: case 0x55: case 0x5D: case 0x65: case 0x6D: case 0x75: case 0x7D:     // RETN / RETI
                          zp->iff1 = zp->iff2; SLOW(); pc = POP(); wz = pc; END(14);
                      case 0x46: case 0x4E: case 0x66: case 0x6E: zp->im = 0; END(8);
                      case 0x56: case 0x76: zp->im = 1; END(8);
                      case 0x5E: case 0x7E: zp->im = 2; END(8);
                      case 0x47: zp->i = a; END(9);
                      case 0x4F: zp->r = a; cr &= ~0xFFFu; END(9);
                      case 0x57: a = zp->i; SETF(szyx[a] | (f & CF) | (zp->iff2 ? PF : 0)); zp->after_ld_ai = 1; SLOW(); END(9);
                      case 0x5F: a = R_NOW(); SETF(szyx[a] | (f & CF) | (zp->iff2 ? PF : 0)); zp->after_ld_ai = 1; SLOW(); END(9);
                      case 0x67: {   // RRD
                          uint8_t m = mem_rd(hl); wz = (uint16_t)(hl + 1);
                          mem_wr(hl, (uint8_t)((a << 4) | (m >> 4))); a = (uint8_t)((a & 0xF0) | (m & 0x0F));
                          SETF(szyxp[a] | (f & CF)); END(18); }
                      case 0x6F: {   // RLD
                          uint8_t m = mem_rd(hl); wz = (uint16_t)(hl + 1);
                          mem_wr(hl, (uint8_t)((m << 4) | (a & 0x0F))); a = (uint8_t)((a & 0xF0) | (m >> 4));
                          SETF(szyxp[a] | (f & CF)); END(18); }
                      // block transfers
                      case 0xA0: case 0xA8: case 0xB0: case 0xB8: {   // LDI LDD LDIR LDDR
                          uint8_t v = mem_rd(hl); mem_wr(de, v);
                          int dir = (eop & 8) ? -1 : 1;
                          SET_HL((uint16_t)(hl + dir)); SET_DE((uint16_t)(de + dir)); SET_BC((uint16_t)(bc - 1));
                          uint8_t n = (uint8_t)(v + a);
                          uint8_t f1 = (uint8_t)((f & (SF | ZF | CF)) | (n & XF) | ((n & 2) ? YF : 0) | (bc ? PF : 0));
                          cr -= CR_T(16);
                          if ((eop & 0x10) && bc) { pc = (uint16_t)(pc - 2); wz = (uint16_t)(pc + 1); f1 = (uint8_t)((f1 & ~(YF | XF)) | ((pc >> 8) & (YF | XF))); cr -= CR_T(5); }
                          SETF(f1); break; }
                      case 0xA1: case 0xA9: case 0xB1: case 0xB9: {   // CPI CPD CPIR CPDR
                          uint8_t v = mem_rd(hl); int dir = (eop & 8) ? -1 : 1;
                          unsigned r = a - v; uint8_t hf = (uint8_t)((a ^ v ^ r) & HF);
                          wz = (uint16_t)(wz + dir);
                          SET_HL((uint16_t)(hl + dir)); SET_BC((uint16_t)(bc - 1));
                          uint8_t n = (uint8_t)(r - (hf ? 1 : 0));
                          uint8_t f1 = (uint8_t)((f & CF) | (szyx[r & 0xFF] & (SF | ZF)) | hf | (n & XF) | ((n & 2) ? YF : 0) | (bc ? PF : 0) | NF);
                          cr -= CR_T(16);
                          if ((eop & 0x10) && bc && !(f1 & ZF)) { pc = (uint16_t)(pc - 2); wz = (uint16_t)(pc + 1); f1 = (uint8_t)((f1 & ~(YF | XF)) | ((pc >> 8) & (YF | XF))); cr -= CR_T(5); }
                          SETF(f1); break; }
                      case 0xA2: case 0xAA: case 0xB2: case 0xBA: {   // INI IND INIR INDR
                          int dir = (eop & 8) ? -1 : 1;
                          wz = (uint16_t)(bc + dir);
                          uint8_t v = IORD(bc); mem_wr(hl, v);
                          bc = (uint16_t)(bc - 0x100); SET_HL((uint16_t)(hl + dir));
                          unsigned k = v + (uint8_t)(C + dir);
                          uint8_t f1 = (uint8_t)(szyx[B] | ((v >> 7) ? NF : 0) | (k > 255 ? (HF | CF) : 0) | (szyxp[(k & 7) ^ B] & PF));
                          cr -= CR_T(16);
                          if ((eop & 0x10) && B) {
                              pc = (uint16_t)(pc - 2); cr -= CR_T(5); wz = (uint16_t)(pc + 1);
                              uint8_t p = f1 & PF;
                              if (f1 & CF) {
                                  if (v & 0x80) { p ^= (szyxp[(B - 1) & 7] & PF) ^ PF; if ((B & 0x0F) == 0) f1 |= HF; else f1 &= ~HF; }
                                  else          { p ^= (szyxp[(B + 1) & 7] & PF) ^ PF; if ((B & 0x0F) == 0x0F) f1 |= HF; else f1 &= ~HF; }
                              } else p ^= (szyxp[B & 7] & PF) ^ PF;
                              f1 = (uint8_t)((f1 & ~(PF | YF | XF)) | p | ((pc >> 8) & (YF | XF)));
                          }
                          SETF(f1); break; }
                      case 0xA3: case 0xAB: case 0xB3: case 0xBB: {   // OUTI OUTD OTIR OTDR
                          int dir = (eop & 8) ? -1 : 1;
                          uint8_t v = mem_rd(hl);
                          bc = (uint16_t)(bc - 0x100); IOWR(bc, v);
                          SET_HL((uint16_t)(hl + dir));
                          wz = (uint16_t)(bc + dir);
                          unsigned k = v + L;
                          uint8_t f1 = (uint8_t)(szyx[B] | ((v >> 7) ? NF : 0) | (k > 255 ? (HF | CF) : 0) | (szyxp[(k & 7) ^ B] & PF));
                          cr -= CR_T(16);
                          if ((eop & 0x10) && B) {
                              pc = (uint16_t)(pc - 2); cr -= CR_T(5); wz = (uint16_t)(pc + 1);
                              uint8_t p = f1 & PF;
                              if (f1 & CF) {
                                  if (v & 0x80) { p ^= (szyxp[(B - 1) & 7] & PF) ^ PF; if ((B & 0x0F) == 0) f1 |= HF; else f1 &= ~HF; }
                                  else          { p ^= (szyxp[(B + 1) & 7] & PF) ^ PF; if ((B & 0x0F) == 0x0F) f1 |= HF; else f1 &= ~HF; }
                              } else p ^= (szyxp[B & 7] & PF) ^ PF;
                              f1 = (uint8_t)((f1 & ~(PF | YF | XF)) | p | ((pc >> 8) & (YF | XF)));
                          }
                          SETF(f1); break; }
                      default: END(8);   // ED NOPs
                  } } break;
            case 0xF3: zp->iff1 = zp->iff2 = 0; SLOW(); END(4);
            case 0xFB: zp->iff1 = zp->iff2 = 1; zp->ei_delay = 1; SLOW(); END(4);
            case 0xF9: sp = hl; END(6);
            default: __builtin_unreachable();
        }
    }
    uint32_t rcnt = RCNT();
    if (z80_brk_hit) cr = 0;                 // stopped by the trace hook: report the chunk as complete so callers do not resume
    zp->pc = pc; zp->sp = sp; zp->wz = wz;
    zp->bc = bc; zp->de = de; zp->hl = hl; zp->a = a; zp->f = f; zp->q = (uint8_t)q;
    zp->cycles = until - (uint32_t)((int32_t)cr >> 12);
    zp->insns += rcnt;   // M1 cycles (instructions + prefixes)
    zp->r = (uint8_t)((zp->r & 0x80) | ((zp->r + rcnt) & 0x7F));
}

#ifndef Z80_PINNED
// Host build: nothing to save around the core.
uint32_t z80_run(z80_t *z, uint32_t until, volatile uint8_t *stop) {
    (void)stop; uint32_t start = z->cycles;
    while ((int32_t)(until - z->cycles) > 0) {
        uint32_t u = until; if ((int32_t)(u - z->cycles) > Z80_RUN_MAX) u = z->cycles + Z80_RUN_MAX;
        z80_core_run(z, u);
    }
    return z->cycles - start;
}
void z80_step(z80_t *z) { z80_core_run(z, z->cycles + 1); }
#endif
