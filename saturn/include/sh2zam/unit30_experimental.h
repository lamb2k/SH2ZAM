/* Experimental Q2.30 unit-vector arithmetic for SH-2: inline rounded
 * multiply, dot and cross products, a seeded multiply-only reciprocal square
 * root, vector normalisation, a rounded integer square root and the two
 * cross-product orthonormalisations of a 3x3 basis. Not selected by any
 * default dispatch.
 *
 * Contract tag: fixed-unit-1. The numeric definitions below are frozen at
 * that tag. Every result is a fixed function of the inputs: the portable C
 * (any host) and the SH-2 code (GCC's DMULS.L/DMULU.L code and the MAC.L
 * sequence of sh2z_u30_dot3) give identical bits for identical inputs. A
 * faster path may replace an implementation only if it produces identical
 * bits; new behaviour gets a new function.
 *
 * Formats. Q2.30: int32_t, 1.0 is 2^30, range [-2, 2). Q20.12: int32_t, 1.0
 * is 4096. All operators below have their exact mathematical meaning (no C
 * overflow): products are exact 64-bit (or wider) values. "Saturate" means
 * clamp to [INT32_MIN, INT32_MAX]. No function uses floating point, the
 * hardware divider, libc or allocation. Only the optional overflow counter
 * (SH2Z_U30_COUNT_OVERFLOWS) is global state; everything else is reentrant.
 *
 * SH-2 notes: sh2z_u30_dot3 uses MAC.L. It saves SR, clears SR.S for its
 * three MAC.L and restores SR, so it is correct whatever the caller's S bit
 * and leaves it unchanged. All functions clobber MACH/MACL; interrupt
 * handlers that use the MAC must save it (as for the rest of the library).
 *
 * Copyright (c) 2026 SH2ZAM Contributors. MIT License.
 * Inspired by SH4ZAM by Falco Girgis and SH4ZAM Contributors.
 */
#ifndef SH2ZAM_UNIT30_EXPERIMENTAL_H
#define SH2ZAM_UNIT30_EXPERIMENTAL_H

#include "sh2zam.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Contract tag fixed-unit-1 ------------------------------------------- */

typedef int32_t sh2z_u30_t;                     /**< Q2.30; one is 2^30. */
#define SH2Z_U30_ONE  INT32_C(0x40000000)
#define SH2Z_Q12_ONE  INT32_C(4096)

/** A 3x3 basis of row vectors in the layout of a float[3][3] row-major basis:
 *  x, then y, then z, each three Q2.30 words (36 bytes, no padding). */
typedef struct { int32_t x[3],y[3],z[3]; } sh2z_basis30_t;

/** Overflow counter. When SH2Z_U30_COUNT_OVERFLOWS is defined non-zero where
 *  this header is included (and, for the out-of-line functions, where the
 *  library is compiled), every saturation below adds one to it. Otherwise it
 *  is never written. Results never depend on it. Not synchronised: with both
 *  CPUs counting, increments may be lost; treat non-zero as "saturated". */
extern volatile uint32_t sh2z_u30_overflows;

#if defined(__sh__) && defined(__SH2__) && (!defined(SH2Z_FORCE_PORTABLE) || !SH2Z_FORCE_PORTABLE)
#define SH2ZI_U30_NATIVE 1
#else
#define SH2ZI_U30_NATIVE 0
#endif
#if defined(SH2Z_U30_COUNT_OVERFLOWS) && SH2Z_U30_COUNT_OVERFLOWS
#define SH2ZI_U30_NOTE() (++sh2z_u30_overflows)
#else
#define SH2ZI_U30_NOTE() ((void)0)
#endif

/* Two's-complement uint32_t -> int32_t without implementation-defined
 * conversion; compiles to nothing on GCC and Clang. */
static inline int32_t sh2zi_u30_s32(uint32_t u) {
    return u<=UINT32_C(0x7fffffff) ? (int32_t)u : (int32_t)(u-UINT32_C(0x80000000))-INT32_C(0x7fffffff)-1;
}
/* q is (exact value + 2^29) mod 2^64, where the exact value is known to lie
 * in [-2^63, 2^63). Returns floor(q / 2^30), saturated. */
static inline int32_t sh2zi_u30_shift30(uint64_t q) {
    const uint32_t hi=(uint32_t)(q>>32);
    if(hi+UINT32_C(0x20000000)>=UINT32_C(0x40000000)) {
        SH2ZI_U30_NOTE();
        return (hi>>31) ? INT32_MIN : INT32_MAX;
    }
    return sh2zi_u30_s32((uint32_t)(q>>30));
}

/** Q2.30 product: floor((a*b + 2^29) / 2^30), saturated (round to nearest,
 *  ties up; |error| <= 0.5 LSB). Saturation needs |a*b| >= 2^61 - 2^29, so
 *  never happens when |a| < 2^30 or |b| < 2^30 (either factor inside
 *  (-1, 1)), nor for a == b == -2^30. */
static inline int32_t sh2z_u30_mul(int32_t a,int32_t b) {
    return sh2zi_u30_shift30((uint64_t)((int64_t)a*b)+UINT64_C(0x20000000));
}

/** Q20.12 times Q2.30 to Q20.12: floor((q12*u + 2^29) / 2^30), saturated.
 *  The same arithmetic as sh2z_u30_mul, named for scaling a Q20.12 length or
 *  position by a Q2.30 direction or factor; saturation needs |u| >= 1. */
static inline int32_t sh2z_q12_mul_u30(int32_t q12,int32_t u) {
    return sh2zi_u30_shift30((uint64_t)((int64_t)q12*u)+UINT64_C(0x20000000));
}

/** Saturation path of sh2z_u30_dot3 (out of line; returns INT32_MIN or
 *  INT32_MAX by the sign of the exact sum + 2^29). Not for direct use. */
int32_t sh2zi_u30_dot3_saturate(const int32_t a[3],const int32_t b[3]);
/** The mantissa kernel of K2, exposed for tests and the instruction model:
 *  about 2^31 / sqrt(m / 2^32) for m in [2^30, 2^32) (other m: unspecified
 *  value, no fault). Not for direct use. */
uint32_t sh2zi_u30_rsqrt_q31(uint32_t m);
/** The portable C definition of sh2z_vec3_normalize_u30 (the SH-2 build of
 *  that function is hand-written assembly), exposed so the instruction model
 *  can compare the two on SH-2. Not for direct use. */
bool sh2zi_u30_normalize_c(int32_t out[3],const int32_t v[3]);

/** Portable definition of sh2z_u30_dot3, also used on SH-2 for comparison. */
static inline int32_t sh2zi_u30_dot3_c(const int32_t a[3],const int32_t b[3]) {
    const uint64_t q=(uint64_t)((int64_t)a[0]*b[0])+(uint64_t)((int64_t)a[1]*b[1])
        +(uint64_t)((int64_t)a[2]*b[2])+UINT64_C(0x20000000);
    const uint32_t hi=(uint32_t)(q>>32);
    if(hi+UINT32_C(0x20000000)>=UINT32_C(0x40000000)) { SH2ZI_U30_NOTE(); return sh2zi_u30_dot3_saturate(a,b); }
    return sh2zi_u30_s32((uint32_t)(q>>30));
}

/** Q2.30 dot product, rounded once: floor((a0*b0 + a1*b1 + a2*b2 + 2^29) /
 *  2^30), saturated (|error| <= 0.5 LSB). The sum is exact (it may need 65
 *  bits; the saturation direction is the sign of the exact sum). Saturation
 *  needs |a| * |b| >= 2 (Euclidean lengths in Q2.30 units), so never happens
 *  when one argument is a unit vector and the other is shorter than 2. */
static inline int32_t sh2z_u30_dot3(const int32_t a[3],const int32_t b[3]) {
#if SH2ZI_U30_NATIVE
    const int32_t *pa=a,*pb=b;
    uint32_t hi,lo,sr,scratch;
    /* MAC.L accumulates the full 64 bits only with SR.S clear: save SR,
     * clear S, preload MACH:MACL with the rounding term 2^29, accumulate,
     * restore SR. Renesas SH-1/SH-2/SH-DSP Software Manual 6.1.31 (MAC.L),
     * 6.1.27/6.1.66 (LDC/STC). The modular 64-bit sum is exact whenever
     * the result does not saturate (the exact sum is at most 3 * 2^62). */
    __asm__ volatile(
        "stc sr,%[sr]\n\t"
        "mov #-3,%[scratch]\n\t"
        "and %[sr],%[scratch]\n\t"
        "ldc %[scratch],sr\n\t"
        "clrmac\n\t"
        "lds %[bias],macl\n\t"
        "mac.l @%[pa]+,@%[pb]+\n\t"
        "mac.l @%[pa]+,@%[pb]+\n\t"
        "mac.l @%[pa]+,@%[pb]+\n\t"
        "ldc %[sr],sr\n\t"
        "sts mach,%[hi]\n\t"
        "sts macl,%[lo]"
        : [pa] "+&r" (pa),[pb] "+&r" (pb),[hi] "=&r" (hi),[lo] "=&r" (lo),
          [sr] "=&r" (sr),[scratch] "=&r" (scratch)
        : [bias] "r" (UINT32_C(0x20000000)),
          "m" (*(const int32_t (*)[3])a),"m" (*(const int32_t (*)[3])b)
        : "mach","macl","t");
    if(hi+UINT32_C(0x20000000)>=UINT32_C(0x40000000)) { SH2ZI_U30_NOTE(); return sh2zi_u30_dot3_saturate(a,b); }
    return sh2zi_u30_s32(hi<<2|lo>>30);
#else
    return sh2zi_u30_dot3_c(a,b);
#endif
}

/** Q2.30 cross product, each component rounded once and saturated:
 *  out[0] = floor((a1*b2 - a2*b1 + 2^29) / 2^30), and cyclically
 *  out[1] from (a2*b0 - a0*b2), out[2] from (a0*b1 - a1*b0). |error| <= 0.5
 *  LSB per component. Saturation needs a component of a x b to reach 2 in
 *  magnitude, so never happens when |a| * |b| < 2. out may alias a or b. */
static inline void sh2z_u30_cross(int32_t out[3],const int32_t a[3],const int32_t b[3]) {
    const int32_t a0=a[0],a1=a[1],a2=a[2],b0=b[0],b1=b[1],b2=b[2];
    const uint64_t k=UINT64_C(0x20000000);
    const int32_t c0=sh2zi_u30_shift30((uint64_t)((int64_t)a1*b2)-(uint64_t)((int64_t)a2*b1)+k);
    const int32_t c1=sh2zi_u30_shift30((uint64_t)((int64_t)a2*b0)-(uint64_t)((int64_t)a0*b2)+k);
    const int32_t c2=sh2zi_u30_shift30((uint64_t)((int64_t)a0*b1)-(uint64_t)((int64_t)a1*b0)+k);
    out[0]=c0;out[1]=c1;out[2]=c2;
}

/* ---- Reciprocal square root, normalisation, square root ---- */

/** 1/sqrt(s / 2^scale) in Q2.30, i.e. about 2^(30 + scale/2) / sqrt(s), for
 *  scale in 0 .. 127 (for example 60 for a sum of squares of Q2.30 words, 24
 *  for Q20.12 words). |error| <= 2 LSB against the exact value. Results of 2
 *  or more (and s == 0) saturate to INT32_MAX; scale outside 0 .. 127
 *  returns 0. The mantissa comes from a 256-entry 16-bit seed table and two
 *  multiply-only Newton steps (UNIT30.md gives the arithmetic). */
int32_t sh2z_rsqrt64_u30(uint64_t s,int scale);

/** Unit vector of v in Q2.30: out[k] is v[k] / |v| * 2^30 to within 2 LSB
 *  (|error| <= 2 per component against the exact value), for v of any
 *  int32_t scale (Q20.12 positions, Q2.30 directions, raw integers). The
 *  sum of squares is exact (64-bit); the result is odd-symmetric
 *  (normalize(-v) == -normalize(v)) and every |out[k]| <= 2^30 + 2.
 *  Returns false, leaving out unchanged, for the zero vector. out may be v
 *  itself; other overlap is not allowed. */
bool sh2z_vec3_normalize_u30(int32_t out[3],const int32_t v[3]);

/** Rounded square root of a 64-bit integer: floor(sqrt(s) + 1/2), exact,
 *  or UINT32_MAX when that is 2^32. For s the sum of squares of Q20.12
 *  words this is the vector length in Q20.12 (error <= 0.5 LSB). */
uint32_t sh2z_isqrt64_q12(uint64_t s);

/* ---- Orthonormalisation by cross products ---- */

/** Make the basis orthonormal around its x (forward) row, in this order:
 *      if x == 0: return -1 (nothing written)
 *      x = normalize(x)                       (sh2z_vec3_normalize_u30)
 *      z = y cross x                          (sh2z_u30_cross)
 *      if z == 0: return -1 (x and z written, y unchanged)
 *      z = normalize(z)
 *      y = x cross z
 *      return 0
 *  x may hold any int32_t vector (only its direction is used: a Q20.12
 *  difference such as target - eye can be stored directly); y is Q2.30 and
 *  only enters the cross product. z on entry is ignored. When |y| < 2 no
 *  step saturates. UNIT30.md states the accuracy against a float64 oracle of
 *  the same algorithm. */
int sh2z_basis30_orthonormalize_forward(sh2z_basis30_t *basis);

/** Make the basis orthonormal around its y (up) row, in this order:
 *      if y == 0: return -1 (nothing written)
 *      y = normalize(y)
 *      z = y cross x
 *      if z == 0: return -1 (y and z written, x unchanged)
 *      z = normalize(z)
 *      x = z cross y
 *      return 0
 *  y may hold any int32_t vector; x is Q2.30 and only enters the cross
 *  product. When |x| < 2 no step saturates. */
int sh2z_basis30_orthonormalize_up(sh2z_basis30_t *basis);

#ifdef __cplusplus
}
#endif

#endif
