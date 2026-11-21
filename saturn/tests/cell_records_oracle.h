/* Independent oracle for cell-records-1 (cell_records_experimental.h): one cell
 * at a time, signed int arithmetic and explicit branches, no shared code with
 * the library. Each CRO_MUT_* bit plants one deliberate bug; the test requires
 * every mutant to disagree with the library (negative controls).
 * MIT; Copyright (c) 2026 SH2ZAM Contributors. */
#ifndef SH2ZAM_CELL_RECORDS_ORACLE_H
#define SH2ZAM_CELL_RECORDS_ORACLE_H

#include <stddef.h>
#include <stdint.h>

enum {
    CRO_MUT_FLIP_BIT = 1u,      /* the horizontal flip toggles bit 6 instead of bit 5 */
    CRO_MUT_WIDTH_SHIFT = 2u,   /* the width is read from attribute bits 12 to 15 */
    CRO_MUT_MIRROR_SIGN = 4u,   /* the mirrored x moves with +dx */
    CRO_MUT_BIAS_IGNORED = 8u,  /* the mirror bias is not subtracted */
    CRO_MUT_ADD_CARRY = 16u,    /* the added byte carries into the next byte */
    CRO_MUT_REPLACE_ORDER = 32u,/* replace ORs before it masks */
    CRO_MUT_S8_UNSIGNED = 64u,  /* byte offsets are read unsigned */
    CRO_MUT_MASK_BEFORE_OR = 128u, /* the position OR is applied before the mask */
    CRO_MUT_Y_SIGN = 256u,      /* the mirrored dy is subtracted */
    CRO_MUT_STATE_X = 512u      /* the running x is not carried to the next call */
};

static int cro_be16(const uint8_t *p) { return (int)((unsigned)p[0] << 8 | p[1]); }
static uint32_t cro_be32(const uint8_t *p) { return (uint32_t)p[0] << 24 | (uint32_t)p[1] << 16 | (uint32_t)p[2] << 8 | p[3]; }
static void cro_put32(uint8_t *p, uint32_t v) { p[0] = (uint8_t)(v >> 24); p[1] = (uint8_t)(v >> 16); p[2] = (uint8_t)(v >> 8); p[3] = (uint8_t)v; }

/* form bits: 1 s16, 2 mirror, 4 replace, 8 add byte */
static void cro_expand(uint32_t *px, uint32_t *py, unsigned form, uint32_t mask, uint32_t bits, uint32_t bias,
    uint32_t keep, uint32_t orv, const uint8_t *rec, const uint8_t *off, uint8_t *out, size_t count, unsigned mut) {
    long x = (long)(*px & 0xffffu), y = (long)(*py & 0xffffu);
    for(size_t i = 0; i < count; i++) {
        uint32_t a = cro_be32(rec + 4 * i);
        long dx, dy, xo;
        if(form & 4u) {
            if(mut & CRO_MUT_REPLACE_ORDER) a = (a | orv) & keep; else a = (a & keep) | orv;
        } else if(form & 8u) {
            unsigned low = a & 0xffu, sum = low + (orv & 0xffu);
            if(mut & CRO_MUT_ADD_CARRY) a = (a & 0xffffff00u) + sum; else a = (a & 0xffffff00u) | (sum & 0xffu);
        }
        if(form & 1u) { dx = (int16_t)cro_be16(off + 4 * i); dy = (int16_t)cro_be16(off + 4 * i + 2); }
        else {
            const unsigned ux = off[2 * i], uy = off[2 * i + 1];
            dx = (mut & CRO_MUT_S8_UNSIGNED) ? (long)ux : (ux & 0x80u ? (long)ux - 256 : (long)ux);
            dy = uy & 0x80u ? (long)uy - 256 : (long)uy;
        }
        if(form & 2u) {
            long width;
            a ^= (mut & CRO_MUT_FLIP_BIT) ? 0x40u : 0x20u;
            width = (mut & CRO_MUT_WIDTH_SHIFT) ? (long)(((a >> 12) & 0xfu) * 16u) : (long)(((a >> 8) & 0xfu) * 16u);
            x = (mut & CRO_MUT_MIRROR_SIGN) ? x + dx : x - dx;
            y = (mut & CRO_MUT_Y_SIGN) ? y - dy : y + dy;
            x &= 0xffff; y &= 0xffff;
            xo = x - width - ((mut & CRO_MUT_BIAS_IGNORED) ? 0 : (long)(bias & 0xffffu));
        } else {
            x = (x + dx) & 0xffff; y = (y + dy) & 0xffff;
            xo = x;
        }
        {
            const uint32_t packed = (uint32_t)(((unsigned long)(xo & 0xffff) << 16) | (unsigned long)y);
            const uint32_t pos = (mut & CRO_MUT_MASK_BEFORE_OR) ? ((packed | bits) & mask) : ((packed & mask) | bits);
            cro_put32(out + 8 * i, pos); cro_put32(out + 8 * i + 4, a);
        }
    }
    *px = (mut & CRO_MUT_STATE_X) ? 0u : (uint32_t)x; *py = (uint32_t)y;
}

#endif
