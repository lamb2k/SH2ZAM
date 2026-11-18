/* Experimental sine, cosine and tangent for 12-bit angles (4096 per turn).
 * Not selected by any default dispatch. MIT; SH2ZAM Contributors.
 *
 * Contract tag: trig12-1. The table values and every result below are frozen
 * at that tag; a faster path may replace an implementation only if it returns
 * identical values. New behaviour gets a new function.
 *
 * ---- Angles ----
 * angle12 is in units of 1/4096 turn. Only its low 12 bits are used, so a
 * signed angle converted to uint32_t (two's complement) wraps as expected.
 *
 * ---- Table ----
 * sh2z_sin12_quarter_u30[i], i = 0..1024, is round(S(i) * 2^30) where S(i) is
 * the binary64 value of the truncated Taylor series that is commonly used to
 * build a 4096-entry sine table at start-up:
 *     x = (i * (2 * 3.1415926535)) / 4096;  S = x - x^3/3! + ... - x^27/27!
 * evaluated term by term in binary64 (saturn/tools/sin12_table.py; the host
 * test re-evaluates it in C). Every entry is within 0.5 LSB (2^-31) of S(i);
 * the binary64 rounding is 2^-21 LSB or less and never changes an entry.
 * Entry 0 is 0, entry 1024 is 2^30 (1.0); the table is nondecreasing.
 *
 * ---- sin, cos ----
 * Quadrant symmetry of the quarter table, with o = angle12 & 1023:
 *     quadrant 0: sin = T[o],         cos = T[1024 - o]
 *     quadrant 1: sin = T[1024 - o],  cos = -T[o]
 *     quadrant 2: sin = -T[o],        cos = -T[1024 - o]
 *     quadrant 3: sin = -T[1024 - o], cos = T[o]
 * so cos12(a) == sin12(a + 1024) for every a, sin12(a + 2048) == -sin12(a),
 * sin and cos are exact at the quarter turns (0, +-2^30) and every result is
 * within 0.5 LSB of the series value of the reflected first-quadrant angle.
 * A table built from the series at all 4096 angles differs from these
 * results where the series itself is less accurate: up to 0.6 LSB in the
 * second and third quadrants and up to 332 LSB (3.1e-7) close to a full turn,
 * where the truncated series, evaluated without range reduction, loses
 * accuracy; TRIG12.md gives the measured bounds, also against that table
 * rounded to binary32. Results are within 0.514 LSB of sin(2 pi a / 4096).
 * Values are Q2.30 ("u30": unit range, 30 fraction bits), in [-2^30, 2^30].
 *
 * ---- tan ----
 * sh2z_tan12_q16 returns tan in Q16.16, rounded to nearest (ties away from
 * zero), from the same table: for quadrants 0 and 2, T[o] / T[1024 - o];
 * for quadrants 1 and 3, -T[1024 - o] / T[o]. Defined for every angle:
 * the poles 1024 (+90 degrees) and 3072 (-90 degrees) saturate by the sign of
 * the sine, to INT32_MAX and INT32_MIN; one step from a pole (|tan| = 651.9)
 * still fits. The division uses CPU division steps, not the memory-mapped
 * divider (about 100 cycles); it is meant for once-per-frame work such as a
 * projection's field of view. Accuracy: TRIG12.md (within 0.5 LSB of the
 * exact quotient of the two table values; at most 8.3 LSB from the true
 * tangent, all of it within 0.6 degrees of a pole).
 *
 * No function here uses floating point, the memory-mapped divider, global
 * state, libc or allocation. Host and SH-2 builds return identical values. */
#ifndef SH2ZAM_TRIG12_EXPERIMENTAL_H
#define SH2ZAM_TRIG12_EXPERIMENTAL_H

#include "sh2zam.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SH2Z_ANGLE12_TURN 4096u
#define SH2Z_ANGLE12_QUARTER 1024u
#define SH2Z_U30_ONE ((int32_t)1073741824)

/** Quarter-wave sine table, Q2.30, entries 0..1024 (see above). */
extern const int32_t sh2z_sin12_quarter_u30[1025];

/** sin(angle12) in Q2.30. */
static inline int32_t sh2z_sin12_u30(uint32_t angle12) {
    const uint32_t o=angle12&1023u;
    const int32_t v=sh2z_sin12_quarter_u30[(angle12&1024u)?1024u-o:o];
    return (angle12&2048u)?-v:v;
}

/** cos(angle12) in Q2.30; equals sh2z_sin12_u30(angle12 + 1024). */
static inline int32_t sh2z_cos12_u30(uint32_t angle12) {
    return sh2z_sin12_u30(angle12+1024u);
}

/** Both at once from two table reads. Writes *s, then *c. */
static inline void sh2z_sincos12_u30(uint32_t angle12,int32_t *s,int32_t *c) {
    const uint32_t o=angle12&1023u;
    int32_t sv=sh2z_sin12_quarter_u30[o],cv=sh2z_sin12_quarter_u30[1024u-o];
    if(angle12&1024u) {const int32_t t=sv;sv=cv;cv=-t;}
    if(angle12&2048u) {sv=-sv;cv=-cv;}
    *s=sv;*c=cv;
}

/** tan(angle12) in Q16.16, rounded to nearest; poles saturate (see above). */
int32_t sh2z_tan12_q16(uint32_t angle12);

#ifdef __cplusplus
}
#endif
#endif
