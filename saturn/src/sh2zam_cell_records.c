/* Experimental expansion of delta coded multi tile sprite records (contract
 * tag cell-records-1).
 *
 * Copyright (c) 2026 SH2ZAM Contributors. MIT License.
 * Inspired by SH4ZAM by Falco Girgis and SH4ZAM Contributors.
 */
#include "sh2zam/cell_records_experimental.h"

#if defined(__GNUC__)
#define CR_INLINE static inline __attribute__((always_inline))
#else
#define CR_INLINE static inline
#endif

/* Big-endian stream access that is the same on every host: plain loads on the
 * big-endian SH-2, byte swapped loads on a little-endian host. */
#if defined(__BYTE_ORDER__) && __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
#define CR_BE32(v) __builtin_bswap32(v)
#define CR_BE16(v) ((uint32_t)__builtin_bswap16((uint16_t)(v)))
#else
#define CR_BE32(v) (v)
#define CR_BE16(v) ((uint32_t)(uint16_t)(v))
#endif

CR_INLINE uint32_t cr_ld16(const uint8_t *p) { return CR_BE16(*(const uint16_t *)p); }
CR_INLINE uint32_t cr_ld32a(const uint8_t *p) { return CR_BE32(*(const uint32_t *)p); }
CR_INLINE uint32_t cr_ld32h(const uint8_t *p) { return (cr_ld16(p) << 16) | cr_ld16(p + 2); }
CR_INLINE void cr_st32(uint8_t *p, uint32_t v) { *(uint32_t *)p = CR_BE32(v); }

/* One loop per form; form, offset width and record alignment are constants so
 * that the compiler leaves no per-cell test. */
CR_INLINE void cr_run(uint32_t form, int aligned, sh2z_cellrec_pos_t *pos, const sh2z_cellrec_params_t *p,
    const uint8_t *rec, const uint8_t *off, uint8_t *out, size_t count) {
    const uint32_t mask = p->pos_mask, bits = p->pos_or, bias = p->mirror_bias, keep = p->attr_keep, add = p->attr_or;
    uint32_t x = pos->x & 0xffffu, y = pos->y & 0xffffu;
    while(count--) {
        uint32_t a = aligned ? cr_ld32a(rec) : cr_ld32h(rec), w, xo, dx, dy;
        rec += 4;
        if(form & SH2Z_CELLREC_ATTR_REPLACE) a = (a & keep) | add;
        else if(form & SH2Z_CELLREC_ATTR_ADD_BYTE) a = (a & 0xffffff00u) | ((a + add) & 0xffu);
        if(form & SH2Z_CELLREC_S16) {
            dx = cr_ld16(off); dy = cr_ld16(off + 2); off += 4;
        } else {
            const uint32_t u = cr_ld16(off);
            dx = (uint32_t)(int32_t)(int8_t)(u >> 8); dy = (uint32_t)(int32_t)(int8_t)u; off += 2;
        }
        y = (y + dy) & 0xffffu;
        if(form & SH2Z_CELLREC_MIRROR_X) {
            a ^= 0x20u;
            w = (a >> 4) & 0xf0u;
            x = (x - dx) & 0xffffu;
            xo = x - w - bias;
        } else {
            x = (x + dx) & 0xffffu;
            xo = x;
        }
        cr_st32(out, ((((xo & 0xffffu) << 16) | y) & mask) | bits);
        cr_st32(out + 4, a);
        out += 8;
    }
    pos->x = x; pos->y = y;
}

#define CR_FORM(f) case f: if(aligned) cr_run(f, 1, pos, p, rec, off, out, count); else cr_run(f, 0, pos, p, rec, off, out, count); break;

void sh2z_cellrec_expand_trusted(sh2z_cellrec_pos_t *pos, const sh2z_cellrec_params_t *p,
    const uint8_t *rec, const uint8_t *off, uint8_t *out, size_t count) {
    const int aligned = !((uintptr_t)rec & 2u);
    switch(p->form) {
    CR_FORM(0u) CR_FORM(1u) CR_FORM(2u) CR_FORM(3u)
    CR_FORM(4u) CR_FORM(5u) CR_FORM(6u) CR_FORM(7u)
    CR_FORM(8u) CR_FORM(9u) CR_FORM(10u) CR_FORM(11u)
    default: break;
    }
}

static bool cr_overlap(const void *a, size_t an, const void *b, size_t bn) {
    if(!an || !bn) return false;
    const uintptr_t aa = (uintptr_t)a, bb = (uintptr_t)b;
    return aa >= bb ? aa - bb < bn : bb - aa < an;
}

bool sh2z_cellrec_expand(sh2z_cellrec_pos_t *pos, const sh2z_cellrec_params_t *p,
    const uint8_t *rec, const uint8_t *off, uint8_t *out, size_t count) {
    if(!pos || !p || (uintptr_t)pos % _Alignof(sh2z_cellrec_pos_t) || (uintptr_t)p % _Alignof(sh2z_cellrec_params_t)) return false;
    if(p->form & ~(SH2Z_CELLREC_S16 | SH2Z_CELLREC_MIRROR_X | SH2Z_CELLREC_ATTR_REPLACE | SH2Z_CELLREC_ATTR_ADD_BYTE)) return false;
    if((p->form & SH2Z_CELLREC_ATTR_REPLACE) && (p->form & SH2Z_CELLREC_ATTR_ADD_BYTE)) return false;
    if(count > SH2Z_CELLREC_MAX_COUNT) return false;
    if(count) {
        const size_t off_bytes = count * ((p->form & SH2Z_CELLREC_S16) ? 4u : 2u), rec_bytes = count * 4u, out_bytes = count * 8u;
        if(!rec || !off || !out || (((uintptr_t)rec | (uintptr_t)off) & 1u) || ((uintptr_t)out & 3u)) return false;
        if(cr_overlap(out, out_bytes, rec, rec_bytes) || cr_overlap(out, out_bytes, off, off_bytes) ||
           cr_overlap(pos, sizeof *pos, out, out_bytes) || cr_overlap(pos, sizeof *pos, rec, rec_bytes) ||
           cr_overlap(pos, sizeof *pos, off, off_bytes) || cr_overlap(p, sizeof *p, out, out_bytes)) return false;
    }
    sh2z_cellrec_expand_trusted(pos, p, rec, off, out, count);
    return true;
}
