/* SH-2 bridge for the executed-instruction model of cell-records-1
 * (cell_records_experimental.h). MIT; Copyright (c) 2026 SH2ZAM Contributors.
 * Every entry point is called directly by the model harness with the SH-2
 * calling convention and works on the arrays below, which the harness writes
 * before each call; nothing here uses libc.
 *
 *   fast_expand     the library, trusted entry (no checks)
 *   checked_expand  the library, checked entry
 *   plain_expand    an independent straightforward C control with the same
 *                   contract: one cell at a time, every decision taken per cell,
 *                   every stream value read a byte at a time */
#include "sh2zam/cell_records_experimental.h"

#define NOINLINE __attribute__((noinline))
#define N 64u

uint8_t rec[N * 4u + 8u] __attribute__((aligned(4)));
uint8_t off[N * 4u + 8u] __attribute__((aligned(4)));
uint8_t out[N * 8u + 8u] __attribute__((aligned(4)));
sh2z_cellrec_pos_t pos;
sh2z_cellrec_params_t prm;

NOINLINE uint32_t fast_expand(uint32_t n) { sh2z_cellrec_expand_trusted(&pos, &prm, rec, off, out, n); return n; }
NOINLINE uint32_t checked_expand(uint32_t n) { return sh2z_cellrec_expand(&pos, &prm, rec, off, out, n) ? n : 0u; }

static uint32_t be32(const uint8_t *p) { return (uint32_t)p[0] << 24 | (uint32_t)p[1] << 16 | (uint32_t)p[2] << 8 | p[3]; }
static void put32(uint8_t *p, uint32_t v) { p[0] = (uint8_t)(v >> 24); p[1] = (uint8_t)(v >> 16); p[2] = (uint8_t)(v >> 8); p[3] = (uint8_t)v; }

NOINLINE uint32_t plain_expand(uint32_t n) {
    int32_t x = (int32_t)(pos.x & 0xffffu), y = (int32_t)(pos.y & 0xffffu);
    for(uint32_t i = 0; i < n; i++) {
        uint32_t a = be32(rec + 4u * i), packed;
        int32_t dx, dy, xo;
        if(prm.form & SH2Z_CELLREC_ATTR_REPLACE) a = (a & prm.attr_keep) | prm.attr_or;
        else if(prm.form & SH2Z_CELLREC_ATTR_ADD_BYTE) a = (a & 0xffffff00u) | (((a & 0xffu) + (prm.attr_or & 0xffu)) & 0xffu);
        if(prm.form & SH2Z_CELLREC_S16) {
            dx = (int16_t)((uint32_t)off[4u * i] << 8 | off[4u * i + 1u]);
            dy = (int16_t)((uint32_t)off[4u * i + 2u] << 8 | off[4u * i + 3u]);
        } else {
            dx = (int8_t)off[2u * i]; dy = (int8_t)off[2u * i + 1u];
        }
        y = (y + dy) & 0xffff;
        if(prm.form & SH2Z_CELLREC_MIRROR_X) {
            a ^= 0x20u;
            x = (x - dx) & 0xffff;
            xo = x - (int32_t)(((a >> 8) & 0xfu) * 16u) - (int32_t)(prm.mirror_bias & 0xffffu);
        } else {
            x = (x + dx) & 0xffff;
            xo = x;
        }
        packed = (((uint32_t)xo & 0xffffu) << 16) | (uint32_t)y;
        put32(out + 8u * i, (packed & prm.pos_mask) | prm.pos_or);
        put32(out + 8u * i + 4u, a);
    }
    pos.x = (uint32_t)x; pos.y = (uint32_t)y;
    return n;
}
