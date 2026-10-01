#include <string.h>
#include "kbd.h"

// MSX JIS keyboard matrix: row, bit, shift.
typedef struct { uint8_t row, bit, shift; } key_t;
#define K(r, b, s) { r, b, s }
#define NOKEY { 0xFF, 0, 0 }

static const key_t table[128] = {
    // 0x00-0x1F
    NOKEY, NOKEY, NOKEY, NOKEY, NOKEY, NOKEY, NOKEY, NOKEY, K(7,5,0) /*BS*/, K(7,3,0) /*TAB*/, NOKEY, NOKEY, NOKEY, K(7,7,0) /*CR*/, NOKEY, NOKEY,
    NOKEY, NOKEY, NOKEY, NOKEY, NOKEY, NOKEY, NOKEY, NOKEY, NOKEY, NOKEY, NOKEY, K(7,2,0) /*ESC*/, NOKEY, NOKEY, NOKEY, NOKEY,
    // 0x20-0x2F  space ! " # $ % & ' ( ) * + , - . /
    K(8,0,0), K(0,1,1), K(0,2,1), K(0,3,1), K(0,4,1), K(0,5,1), K(0,6,1), K(0,7,1), K(1,0,1), K(1,1,1), K(2,0,1), K(1,7,1), K(2,2,0), K(1,2,0), K(2,3,0), K(2,4,0),
    // 0x30-0x3F  0-9 : ; < = > ?
    K(0,0,0), K(0,1,0), K(0,2,0), K(0,3,0), K(0,4,0), K(0,5,0), K(0,6,0), K(0,7,0), K(1,0,0), K(1,1,0), K(2,0,0), K(1,7,0), K(2,2,1), K(1,2,1), K(2,3,1), K(2,4,1),
    // 0x40-0x4F  @ A-O
    K(1,5,0), K(2,6,0), K(2,7,0), K(3,0,0), K(3,1,0), K(3,2,0), K(3,3,0), K(3,4,0), K(3,5,0), K(3,6,0), K(3,7,0), K(4,0,0), K(4,1,0), K(4,2,0), K(4,3,0), K(4,4,0),
    // 0x50-0x5F  P-Z [ \ ] ^ _
    K(4,5,0), K(4,6,0), K(4,7,0), K(5,0,0), K(5,1,0), K(5,2,0), K(5,3,0), K(5,4,0), K(5,5,0), K(5,6,0), K(5,7,0), K(1,6,0), K(1,4,0), K(2,1,0), K(1,3,0), K(2,5,1),
    // 0x60-0x6F  ` a-o (same keys as upper case)
    K(1,5,1), K(2,6,0), K(2,7,0), K(3,0,0), K(3,1,0), K(3,2,0), K(3,3,0), K(3,4,0), K(3,5,0), K(3,6,0), K(3,7,0), K(4,0,0), K(4,1,0), K(4,2,0), K(4,3,0), K(4,4,0),
    // 0x70-0x7F  p-z { | } ~ DEL
    K(4,5,0), K(4,6,0), K(4,7,0), K(5,0,0), K(5,1,0), K(5,2,0), K(5,3,0), K(5,4,0), K(5,5,0), K(5,6,0), K(5,7,0), K(1,6,1), K(1,4,1), K(2,1,1), K(1,3,1), K(8,3,0),
};

#define QLEN 1024
static char queue[QLEN]; static int qhead, qtail;
static uint8_t matrix[16];             // bits currently held down
static uint8_t cur_row;
static int phase;                      // 0 idle, 1 pressed, 2 released
static int phase_scans;
static bool rows_seen;

void kbd_init(void) { qhead = qtail = 0; memset(matrix, 0, sizeof matrix); phase = 0; phase_scans = 0; cur_row = 0; }
bool kbd_type(const char *s) {
    for (const char *p = s; *p; p++) {
        if ((uint8_t)*p >= 128 || table[(uint8_t)*p].row == 0xFF) return false;
        if ((qtail + 1) % QLEN == qhead) return false;
        queue[qtail] = *p; qtail = (qtail + 1) % QLEN;
    }
    return true;
}
void kbd_cancel(void) { qhead = qtail = 0; memset(matrix, 0, sizeof matrix); phase = 0; }
int kbd_pending(void) { return (qtail - qhead + QLEN) % QLEN; }

// The BIOS scans rows 0..10 in order once per interrupt; a wrap back to a low row marks one full scan.
static void on_scan(void) {
    if (phase == 0) {
        if (qhead == qtail) return;
        const key_t *k = &table[(uint8_t)queue[qhead]];
        matrix[k->row] |= (uint8_t)(1 << k->bit);
        if (k->shift) matrix[6] |= 0x01;
        phase = 1; phase_scans = 0;
    } else if (phase == 1) {
        if (++phase_scans >= 2) { memset(matrix, 0, sizeof matrix); qhead = (qhead + 1) % QLEN; phase = 2; phase_scans = 0; }
    } else {
        if (++phase_scans >= 2) phase = 0;
    }
}

void kbd_row_select(uint8_t row) {
    if (row < cur_row) on_scan();          // wrapped: a full scan completed
    cur_row = row; rows_seen = true;
}
uint8_t kbd_row_mask(void) { return matrix[cur_row & 15]; }
