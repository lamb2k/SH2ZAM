/* Experimental scalar division by constants through magic-number multiplies,
 * as inline functions. The SH-2 has no divide instruction (C division calls a
 * library routine of about 40 to 70 cycles); a multiply-high by a magic number
 * costs one DMULU.L. Not selected by any default dispatch.
 * MIT; SH2ZAM Contributors.
 *
 * Contract tag: shade-2. Every function states the inputs on which it is
 * exact; the tests check each named helper against C division for every input
 * of that domain and one input past it. Outside the domain results are
 * defined (no undefined behaviour) but not quotients. No libc, no hardware
 * divider, no state. */
#ifndef SH2ZAM_CDIV_EXPERIMENTAL_H
#define SH2ZAM_CDIV_EXPERIMENTAL_H

#include "sh2zam.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Contract tag shade-2 ------------------------------------------------- */

/* ---- The family ----
 *
 * For a divisor d and a post-shift s with 2^s < d < 2^32 and s <= 31:
 *     m = SH2Z_CDIV_M(d, s) = ceil(2^(32 + s) / d)        (fits in 32 bits)
 *     e = m * d - 2^(32 + s)                               (0 <= e < d)
 *     floor(n / d) == ((n * m) >> 32) >> s   for every n with n * e < 2^(32 + s)
 * Proof: with n = q * d + r, n * m / 2^(32+s) = n / d + n * e / (d * 2^(32+s))
 * lies in [q, q + (r + n * e / 2^(32+s)) / d), below q + 1 when n * e < 2^(32+s)
 * because r <= d - 1. SH2Z_CDIV_N(d, s) is the largest n that guarantees it
 * (UINT32_MAX when e = 0); the true domain can be larger, and the named
 * helpers below state theirs exactly. Pass constants: with d and s known at
 * compile time m folds to an immediate and the shift to fixed shifts. */
#define SH2Z_CDIV_M(d,s) \
    ((uint32_t)(((UINT64_C(1)<<(32u+(unsigned)(s)))+(uint64_t)(d)-1u)/(uint64_t)(d)))
#define SH2Z_CDIV_E(d,s) \
    ((uint64_t)SH2Z_CDIV_M(d,s)*(uint64_t)(d)-(UINT64_C(1)<<(32u+(unsigned)(s))))
#define SH2Z_CDIV_N(d,s) \
    (SH2Z_CDIV_E(d,s)==0u||((UINT64_C(1)<<(32u+(unsigned)(s)))-1u)/SH2Z_CDIV_E(d,s)>UINT64_C(0xFFFFFFFF)? \
     UINT32_C(0xFFFFFFFF):(uint32_t)(((UINT64_C(1)<<(32u+(unsigned)(s)))-1u)/SH2Z_CDIV_E(d,s)))

/** floor(n / d) for n within the domain of (m, s) = (SH2Z_CDIV_M(d, s), s).
 *  On SH-2: DMULU.L, STS MACH and s fixed shifts. */
static inline uint32_t sh2z_cdiv_u32(uint32_t n,uint32_t m,unsigned s) {
    return (uint32_t)(((uint64_t)n*m)>>32)>>s;
}

/** trunc(n / d) (toward zero, as C's /) for |n| within the domain of (m, s):
 *  the quotient of the magnitude with n's sign. */
static inline int32_t sh2z_cdiv_s32(int32_t n,uint32_t m,unsigned s) {
    const uint32_t a=n<0?0u-(uint32_t)n:(uint32_t)n;
    const uint32_t q=sh2z_cdiv_u32(a,m,s);
    return n<0?-(int32_t)q:(int32_t)q;
}

/* ---- Named helpers (exact domains checked exhaustively by the tests) ---- */

/** floor(n / 255) for n <= 16,909,558 (first wrong at 16,909,559).
 *  m = 16,843,010 = ceil(2^32 / 255), s = 0. */
static inline uint32_t sh2z_cdiv255_u32(uint32_t n) {
    return (uint32_t)(((uint64_t)n*UINT32_C(16843010))>>32);
}

/** trunc(n / 255) for |n| <= 16,909,558. */
static inline int32_t sh2z_cdiv255_s32(int32_t n) {
    const uint32_t a=n<0?0u-(uint32_t)n:(uint32_t)n;
    const uint32_t q=sh2z_cdiv255_u32(a);
    return n<0?-(int32_t)q:(int32_t)q;
}

/** floor(n / 65025) (65025 = 255 * 255) for n <= 4,291,648 (first wrong at
 *  4,291,649). m = 2,113,633 = ceil(2^37 / 65025), s = 5. */
static inline uint32_t sh2z_cdiv65025_u32(uint32_t n) {
    return (uint32_t)(((uint64_t)n*UINT32_C(2113633))>>37);
}

#ifdef __cplusplus
}
#endif

#endif
