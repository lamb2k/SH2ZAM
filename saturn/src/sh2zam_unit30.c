/* Q2.30 unit-vector kernels (fixed-unit-1): reciprocal square root, vector
 * normalisation, rounded square root and basis orthonormalisation.
 * MIT; Copyright (c) 2026 SH2ZAM Contributors.
 *
 * Everything here is integer C. On SH-2, GCC turns each 32x32 -> 64 product
 * below into one DMULS.L or DMULU.L, so the SH-2 and host builds compute the
 * same bits by construction; the SH-2 instruction model checks that
 * (benchmarks/unit30-model). No libc, no floating point, no divider. */
#include "sh2zam/unit30_experimental.h"
#include "sh2zam_sections.h"

volatile uint32_t sh2z_u30_overflows;

/* Seed for 1/sqrt(x), x = m / 2^32 in [1/4, 1), indexed by
 *     i = m >> 24          for m in [2^31, 2^32)   x in [i/256, (i+1)/256)
 *     i = (m >> 23) - 128  for m in [2^30, 2^31)   x in [(128+i)/512, (129+i)/512)
 * so every interval is 2^-7 of its x. Entry i is round(2^15 * h), h the
 * harmonic mean of 1/sqrt at the two ends of the interval (the constant with
 * the smallest worst relative error, at most 2^-9 + 2^-16). Every entry lies
 * in (2^15, 2^16). tests/test_unit30.c rebuilds the table from this rule. */
extern const uint16_t sh2zi_u30_seed[256];
const uint16_t sh2zi_u30_seed[256] SH2ZI_RODATA(sh2zi_u30_seed)={
    65408,65155,64905,64658,64414,64172,63933,63697,63463,63232,63003,62777,
    62553,62331,62112,61896,61681,61469,61259,61051,60845,60641,60439,60239,
    60041,59845,59651,59459,59269,59081,58894,58709,58526,58344,58165,57986,
    57810,57635,57462,57290,57120,56951,56784,56618,56453,56291,56129,55969,
    55810,55653,55497,55342,55188,55036,54885,54735,54587,54439,54293,54148,
    54004,53862,53720,53580,53440,53302,53165,53029,52894,52760,52627,52495,
    52363,52233,52104,51976,51849,51722,51597,51473,51349,51226,51105,50984,
    50863,50744,50626,50508,50391,50275,50160,50046,49932,49819,49707,49596,
    49485,49376,49266,49158,49050,48943,48837,48731,48627,48522,48419,48316,
    48214,48112,48011,47911,47811,47712,47613,47516,47418,47322,47226,47130,
    47035,46941,46847,46754,46661,46569,46477,46386,46251,46072,45895,45720,
    45547,45376,45207,45040,44875,44712,44550,44390,44232,44075,43920,43767,
    43615,43465,43316,43169,43024,42880,42737,42596,42456,42317,42180,42044,
    41910,41776,41644,41514,41384,41256,41129,41003,40878,40754,40632,40510,
    40390,40270,40152,40035,39919,39803,39689,39576,39464,39352,39242,39133,
    39024,38916,38810,38704,38599,38494,38391,38289,38187,38086,37986,37887,
    37788,37690,37593,37497,37401,37307,37213,37119,37027,36935,36843,36753,
    36663,36573,36485,36397,36309,36222,36136,36051,35966,35882,35798,35715,
    35632,35550,35469,35388,35307,35228,35148,35070,34991,34914,34837,34760,
    34684,34608,34533,34458,34384,34310,34237,34164,34092,34020,33949,33878,
    33807,33737,33668,33599,33530,33461,33393,33326,33259,33192,33126,33060,
    32994,32929,32864,32800};

/* Powers of two for variable right shifts: SH-2 has no barrel shifter, but
 * x >> n == high word of x * 2^(32-n) (one DMULU.L) for n in 1 .. 31. */
static const uint32_t sh2zi_u30_pow2[32] SH2ZI_RODATA(sh2zi_u30_pow2)={
    UINT32_C(1),UINT32_C(1)<<1,UINT32_C(1)<<2,UINT32_C(1)<<3,UINT32_C(1)<<4,UINT32_C(1)<<5,
    UINT32_C(1)<<6,UINT32_C(1)<<7,UINT32_C(1)<<8,UINT32_C(1)<<9,UINT32_C(1)<<10,UINT32_C(1)<<11,
    UINT32_C(1)<<12,UINT32_C(1)<<13,UINT32_C(1)<<14,UINT32_C(1)<<15,UINT32_C(1)<<16,UINT32_C(1)<<17,
    UINT32_C(1)<<18,UINT32_C(1)<<19,UINT32_C(1)<<20,UINT32_C(1)<<21,UINT32_C(1)<<22,UINT32_C(1)<<23,
    UINT32_C(1)<<24,UINT32_C(1)<<25,UINT32_C(1)<<26,UINT32_C(1)<<27,UINT32_C(1)<<28,UINT32_C(1)<<29,
    UINT32_C(1)<<30,UINT32_C(1)<<31};
/* x >> n for n in 0 .. 31. */
static inline uint32_t sh2zi_u30_shr(uint32_t x,unsigned n) {
    return n ? (uint32_t)(((uint64_t)x*sh2zi_u30_pow2[32u-n])>>32) : x;
}

static inline uint32_t sh2zi_u30_hi(uint64_t v) { return (uint32_t)(v>>32); }
/* High word of the signed 64-bit product a*b, as two's-complement bits.
 * GCC compiles a mixed-range signed product to DMULU.L plus a MUL.L
 * correction, so the SH-2 build states the single DMULS.L it wants. */
static inline uint32_t sh2zi_u30_mulhs(int32_t a,int32_t b) {
#if SH2ZI_U30_NATIVE
    uint32_t hi;
    __asm__("dmuls.l %[a],%[b]\n\tsts mach,%[hi]"
        : [hi] "=r" (hi) : [a] "r" (a),[b] "r" (b) : "mach","macl");
    return hi;
#else
    return sh2zi_u30_hi((uint64_t)((int64_t)a*b));
#endif
}

int32_t sh2zi_u30_dot3_saturate(const int32_t a[3],const int32_t b[3]) {
    /* Exact sign of a0*b0 + a1*b1 + a2*b2 + 2^29: split each product into
     * a signed high part and an unsigned low word. */
    int64_t high=0; uint64_t low=UINT64_C(0x20000000);
    for(unsigned k=0;k<3;k++) {
        const int64_t p=(int64_t)a[k]*b[k];
        const uint32_t lo=(uint32_t)p;
        high+=(p-(int64_t)lo)/INT64_C(4294967296);
        low+=lo;
    }
    high+=(int64_t)(low>>32);
    return high<0 ? INT32_MIN : INT32_MAX;
}

/* ---- reciprocal square root of a normalised mantissa ----------------------
 * m in [2^30, 2^32) is x = m / 2^32 in [1/4, 1). Returns Y, about
 * 2^31 / sqrt(x) (y = 1/sqrt(x) in Q1.31), from the seed and two Newton
 * steps y' = y + y (1 - x y^2) / 2, using products only (high() is the high
 * word of the exact 64-bit product, all arithmetic mod 2^32):
 *   step 1: y0 = seed << 16
 *           e  = -(high(high(m y0) y0) << 2)      (1 - x y0^2) 2^32, |e| < 2^24
 *           y1 = y0 + high_signed((y0 >> 1) e)
 *   step 2: u  = m y1 (64 bits)
 *           r  = bits 16..47 of high(u) y1  +  high(low(u) y1) >> 16
 *                                             (x y1^2 2^46 mod 2^32: bits
 *                                             48..79 of the 96-bit m y1^2)
 *           e  = -r << 1                      (1 - x y1^2) 2^47, |e| < 2^31
 *           y2 = y1 + high_signed(((y1 >> 16) + 1) e)
 * Y - 2^31/sqrt(x) lies in [-1.2004, 0.2243] (every m in [2^30, 2^32) against
 * a float64 oracle: tests/test_unit30.c with SH2Z_UNIT30_EXHAUSTIVE=1). */
static inline uint32_t sh2zi_u30_rsqrt_core(uint32_t m) {
    const unsigned i=m>=UINT32_C(0x80000000) ? (unsigned)(m>>24) : (unsigned)(m>>23)-128u;
    uint32_t y=(uint32_t)sh2zi_u30_seed[i]<<16;
    /* step 1 */
    {
        const uint32_t xy=sh2zi_u30_hi((uint64_t)m*y);           /* x y, Q1.31 */
        const uint32_t xyy=sh2zi_u30_hi((uint64_t)xy*y);         /* x y^2, Q2.30 */
        const int32_t e=sh2zi_u30_s32(0u-(xyy<<2));
        y+=sh2zi_u30_mulhs(sh2zi_u30_s32(y>>1),e);
    }
    /* step 2 */
    {
        const uint64_t u=(uint64_t)m*y;
        const uint32_t uh=sh2zi_u30_hi(u),ul=(uint32_t)u;
        const uint32_t t1=(uint32_t)(((uint64_t)uh*y)>>16);
        const uint32_t t2=sh2zi_u30_hi((uint64_t)ul*y)>>16;
        const int32_t e=sh2zi_u30_s32((0u-(t1+t2))<<1);
        y+=sh2zi_u30_mulhs(sh2zi_u30_s32((y>>16)+1u),e);
    }
    return y;
}

uint32_t sh2zi_u30_rsqrt_q31(uint32_t m) {
    return sh2zi_u30_rsqrt_core(m);
}

/* ---- mantissa of a non-zero 64-bit value ----------------------------------
 * Shifts s left by an even L in 0 .. 62 so the top word is in [2^30, 2^32):
 * *hi_out:*lo_out = s << L (nothing lost), returns L. */
static inline unsigned sh2zi_u30_align(uint64_t s,uint32_t *hi_out,uint32_t *lo_out) {
    uint32_t hi=sh2zi_u30_hi(s),lo=(uint32_t)s;
    unsigned shift=0;
    if(!hi) { hi=lo; lo=0; shift=32; }
    if(hi<UINT32_C(0x10000)) { hi=hi<<16|lo>>16; lo<<=16; shift+=16; }
    if(hi<UINT32_C(0x1000000)) { hi=hi<<8|lo>>24; lo<<=8; shift+=8; }
    if(hi<UINT32_C(0x10000000)) { hi=hi<<4|lo>>28; lo<<=4; shift+=4; }
    if(hi<UINT32_C(0x40000000)) { hi=hi<<2|lo>>30; lo<<=2; shift+=2; }
    *hi_out=hi; *lo_out=lo;
    return shift;
}

int32_t sh2z_rsqrt64_u30(uint64_t s,int scale) {
    if(scale<0||scale>127) return 0;
    if(!s) { SH2ZI_U30_NOTE(); return INT32_MAX; }
    uint32_t m,lo;
    /* s = (m + frac) * 2^(32 - L); make 32 - L - scale even. */
    int k=32-(int)sh2zi_u30_align(s,&m,&lo);
    if((k-scale)&1) {
        if(m<UINT32_C(0x80000000)) { m=m<<1|lo>>31; k-=1; }
        else { m>>=1; k+=1; }
    }
    /* s / 2^scale = x * 4^E, x = m / 2^32; result = y * 2^(30 - E) = Y >> (E + 1) */
    const int E=(32+k-scale)/2;
    const uint32_t y=sh2zi_u30_rsqrt_core(m);
    if(E<0) { SH2ZI_U30_NOTE(); return INT32_MAX; }
    if(E>31) return 0;
    const uint32_t t=sh2zi_u30_shr(y,(unsigned)E);
    if(t==UINT32_MAX) { SH2ZI_U30_NOTE(); return INT32_MAX; }
    return (int32_t)((t+1u)>>1);
}

/* ---- normalisation ----------------------------------------------------------
 * The portable definition. With a_k = |v_k|:
 *   1. prescale: a_k <<= t, t the one shift that puts max a_k in [2^30, 2^31]
 *      (exact; nothing is lost);
 *   2. s = a0^2 + a1^2 + a2^2 (exact, in [2^60, 3 * 2^62]);
 *   3. if s >= 2^62: m = s >> 32; else m = s >> 30 and a_k <<= 1 (each a_k
 *      < 2^31 then, so 2 a_k fits), so that 1/sqrt(s) = Y / 2^63 in both;
 *   4. Y = the mantissa kernel of m; out_k = +-((high(a_k Y) + 1) >> 1),
 *      that is round(a_k Y / 2^33), with the sign of v_k.
 * Returns false for v == 0 without writing. */
bool sh2zi_u30_normalize_c(int32_t out[3],const int32_t v[3]) {
    const int32_t v0=v[0],v1=v[1],v2=v[2];
    uint32_t a0=v0<0 ? 0u-(uint32_t)v0 : (uint32_t)v0;
    uint32_t a1=v1<0 ? 0u-(uint32_t)v1 : (uint32_t)v1;
    uint32_t a2=v2<0 ? 0u-(uint32_t)v2 : (uint32_t)v2;
    uint32_t top=a0|a1|a2;
    if(!top) return false;
    if(top<UINT32_C(0x40000000)) {
        if(top<UINT32_C(0x20000000)) {
            if(top<UINT32_C(0x8000)) { top<<=16; a0<<=16; a1<<=16; a2<<=16; }
            if(top<UINT32_C(0x800000)) { top<<=8; a0<<=8; a1<<=8; a2<<=8; }
            if(top<UINT32_C(0x8000000)) { top<<=4; a0<<=4; a1<<=4; a2<<=4; }
            if(top<UINT32_C(0x20000000)) { top<<=2; a0<<=2; a1<<=2; a2<<=2; }
        }
        if(top<UINT32_C(0x40000000)) { a0<<=1; a1<<=1; a2<<=1; }
    }
    const uint64_t s=(uint64_t)a0*a0+(uint64_t)a1*a1+(uint64_t)a2*a2;
    uint32_t m=sh2zi_u30_hi(s);
    if(m<UINT32_C(0x40000000)) { m=(uint32_t)(s>>30); a0<<=1; a1<<=1; a2<<=1; }
    const uint32_t y=sh2zi_u30_rsqrt_core(m);
    const uint32_t r0=(sh2zi_u30_hi((uint64_t)a0*y)+1u)>>1;
    const uint32_t r1=(sh2zi_u30_hi((uint64_t)a1*y)+1u)>>1;
    const uint32_t r2=(sh2zi_u30_hi((uint64_t)a2*y)+1u)>>1;
    out[0]=v0<0 ? -(int32_t)r0 : (int32_t)r0;
    out[1]=v1<0 ? -(int32_t)r1 : (int32_t)r1;
    out[2]=v2<0 ? -(int32_t)r2 : (int32_t)r2;
    return true;
}

#if SH2ZI_U30_NATIVE
/* sh2z_vec3_normalize_u30 on SH-2: the definition above, instruction for
 * instruction (r4 out, r5 v; returns r0). Registers: r1-r3 the magnitudes a_k,
 * r0 top and scratch, r6:r7 the sum of squares, then m in r7 and Y in r6;
 * r4/r5 are pushed while the mantissa kernel uses them. Uses r0-r7, MACH,
 * MACL and T only (all call-clobbered); SR.S is not read or written (DMULU.L
 * and DMULS.L do not depend on it). out must be v or not overlap it: v[k] is
 * read again for its sign just before out[k] is written.
 * tests: the model harness compares every result with the portable C
 * (sh2zi_u30_normalize_c) on SH-2 and on the host. */
__asm__(
"    .pushsection .text.sh2z_vec3_normalize_u30,\"ax\",@progbits\n"
"    .align 2\n"
"    .global _sh2z_vec3_normalize_u30\n"
"    .type _sh2z_vec3_normalize_u30,@function\n"
"_sh2z_vec3_normalize_u30:\n"
"    mov.l   @r5,r1\n"
"    mov.l   @(4,r5),r2\n"
"    cmp/pz  r1\n"
"    bt/s    1f\n"
"    mov.l   @(8,r5),r3\n"
"    neg     r1,r1\n"
"1:  cmp/pz  r2\n"
"    bt      2f\n"
"    neg     r2,r2\n"
"2:  cmp/pz  r3\n"
"    bt      3f\n"
"    neg     r3,r3\n"
"3:  mov     r1,r0\n"
"    or      r2,r0\n"
"    or      r3,r0\n"                  /* top */
"    tst     r0,r0\n"
"    bt      .Lsh2zi_u30n_zero\n"
"    mov.l   .Lsh2zi_u30n_k30,r6\n"
"    cmp/hs  r6,r0\n"
"    bt      .Lsh2zi_u30n_scaled\n"    /* top >= 2^30 */
"    shlr    r6\n"
"    cmp/hs  r6,r0\n"
"    bt      .Lsh2zi_u30n_one\n"       /* top >= 2^29 */
"    mov.w   .Lsh2zi_u30n_k15,r6\n"
"    cmp/hi  r6,r0\n"
"    bt      4f\n"
"    shll16  r0\n"
"    shll16  r1\n"
"    shll16  r2\n"
"    shll16  r3\n"
"4:  mov.l   .Lsh2zi_u30n_k23,r6\n"
"    cmp/hi  r6,r0\n"
"    bt      5f\n"
"    shll8   r0\n"
"    shll8   r1\n"
"    shll8   r2\n"
"    shll8   r3\n"
"5:  mov.l   .Lsh2zi_u30n_k27,r6\n"
"    cmp/hi  r6,r0\n"
"    bt      6f\n"
"    shll2   r0\n"
"    shll2   r1\n"
"    shll2   r2\n"
"    shll2   r3\n"
"    shll2   r0\n"
"    shll2   r1\n"
"    shll2   r2\n"
"    shll2   r3\n"
"6:  mov.l   .Lsh2zi_u30n_k29,r6\n"
"    cmp/hi  r6,r0\n"
"    bt      7f\n"
"    shll2   r0\n"
"    shll2   r1\n"
"    shll2   r2\n"
"    shll2   r3\n"
"7:  mov.l   .Lsh2zi_u30n_k30,r6\n"
"    cmp/hs  r6,r0\n"
"    bt      .Lsh2zi_u30n_scaled\n"
".Lsh2zi_u30n_one:\n"
"    add     r1,r1\n"
"    add     r2,r2\n"
"    add     r3,r3\n"
".Lsh2zi_u30n_scaled:\n"
"    dmulu.l r1,r1\n"                  /* r7:r6 = sum of squares */
"    sts     macl,r6\n"
"    sts     mach,r7\n"
"    dmulu.l r2,r2\n"
"    sts     macl,r0\n"
"    clrt\n"
"    addc    r0,r6\n"
"    sts     mach,r0\n"
"    addc    r0,r7\n"
"    dmulu.l r3,r3\n"
"    sts     macl,r0\n"
"    clrt\n"
"    addc    r0,r6\n"
"    sts     mach,r0\n"
"    addc    r0,r7\n"
"    mov.l   .Lsh2zi_u30n_k30,r0\n"
"    cmp/hs  r0,r7\n"
"    bt      8f\n"
"    shll    r6\n"                     /* m = s >> 30, a <<= 1 */
"    rotcl   r7\n"
"    shll    r6\n"
"    rotcl   r7\n"
"    add     r1,r1\n"
"    add     r2,r2\n"
"    add     r3,r3\n"
"8:  mov.l   r4,@-r15\n"
"    mov.l   r5,@-r15\n"
"    mov     r7,r0\n"                  /* seed index */
"    cmp/pz  r7\n"
"    bf/s    9f\n"
"    shlr16  r0\n"
"    add     r0,r0\n"
"    shlr8   r0\n"
"    bra     10f\n"
"    add     #-128,r0\n"
"9:  shlr8   r0\n"
"10: mov.l   .Lsh2zi_u30n_seed,r4\n"
"    add     r0,r0\n"
"    mov.w   @(r0,r4),r6\n"
"    shll16  r6\n"                     /* y0 */
"    dmulu.l r7,r6\n"                  /* step 1 */
"    sts     mach,r4\n"
"    dmulu.l r4,r6\n"
"    sts     mach,r4\n"
"    shll2   r4\n"
"    neg     r4,r4\n"
"    mov     r6,r0\n"
"    shlr    r0\n"
"    dmuls.l r0,r4\n"
"    sts     mach,r4\n"
"    add     r4,r6\n"                  /* y1 */
"    dmulu.l r7,r6\n"                  /* step 2 */
"    sts     mach,r4\n"
"    sts     macl,r0\n"
"    dmulu.l r0,r6\n"
"    sts     mach,r0\n"
"    shlr16  r0\n"
"    dmulu.l r4,r6\n"
"    sts     mach,r4\n"
"    sts     macl,r5\n"
"    xtrct   r4,r5\n"
"    add     r5,r0\n"
"    neg     r0,r0\n"
"    add     r0,r0\n"
"    mov     r6,r5\n"
"    shlr16  r5\n"
"    add     #1,r5\n"
"    dmuls.l r5,r0\n"
"    sts     mach,r0\n"
"    add     r0,r6\n"                  /* Y */
"    dmulu.l r1,r6\n"                  /* (high(a Y) + 1) >> 1 */
"    sts     mach,r1\n"
"    dmulu.l r2,r6\n"
"    add     #1,r1\n"
"    sts     mach,r2\n"
"    shlr    r1\n"
"    dmulu.l r3,r6\n"
"    add     #1,r2\n"
"    sts     mach,r3\n"
"    shlr    r2\n"
"    add     #1,r3\n"
"    shlr    r3\n"
"    mov.l   @r15+,r5\n"
"    mov.l   @r15+,r4\n"
"    mov.l   @r5,r0\n"                 /* signs, then stores */
"    cmp/pz  r0\n"
"    bt      11f\n"
"    neg     r1,r1\n"
"11: mov.l   @(4,r5),r0\n"
"    mov.l   r1,@r4\n"
"    cmp/pz  r0\n"
"    bt      12f\n"
"    neg     r2,r2\n"
"12: mov.l   @(8,r5),r0\n"
"    mov.l   r2,@(4,r4)\n"
"    cmp/pz  r0\n"
"    bt      13f\n"
"    neg     r3,r3\n"
"13: mov.l   r3,@(8,r4)\n"
"    rts\n"
"    mov     #1,r0\n"
".Lsh2zi_u30n_zero:\n"
"    rts\n"
"    mov     #0,r0\n"
"    .align 2\n"
".Lsh2zi_u30n_k30:  .long 0x40000000\n"
".Lsh2zi_u30n_k23:  .long 0x007fffff\n"
".Lsh2zi_u30n_k27:  .long 0x07ffffff\n"
".Lsh2zi_u30n_k29:  .long 0x1fffffff\n"
".Lsh2zi_u30n_seed: .long _sh2zi_u30_seed\n"
".Lsh2zi_u30n_k15:  .word 0x7fff\n"
"    .align 2\n"
"    .size _sh2z_vec3_normalize_u30,.-_sh2z_vec3_normalize_u30\n"
"    .popsection\n"
);
#else
bool sh2z_vec3_normalize_u30(int32_t out[3],const int32_t v[3]) {
    return sh2zi_u30_normalize_c(out,v);
}
#endif

uint32_t sh2z_isqrt64_q12(uint64_t s) {
    if(!s) return 0;
    uint32_t hi,lo;
    const unsigned shift=sh2zi_u30_align(s,&hi,&lo);       /* S = s << shift, S in [2^62, 2^64) */
    /* q ~ sqrt(S) = S * y / 2^63 / 2^32 * 2^32, y from the top word:
     * sqrt(S) = sqrt(x) 2^32 = x y 2^32 with x = S / 2^64. */
    const uint32_t y=sh2zi_u30_rsqrt_core(hi);
    const uint64_t p=(uint64_t)hi*y+sh2zi_u30_hi((uint64_t)lo*y);
    const uint32_t qhi=sh2zi_u30_hi(p)>>31;    /* p >> 31 is floor(sqrt(S)) - 2 .. + 1 */
    uint32_t q=qhi ? UINT32_MAX : (uint32_t)(p>>31);
    /* exact floor(sqrt(S)) */
    uint64_t sq=(uint64_t)q*q;
    const uint64_t S=(uint64_t)hi<<32|lo;
    while(sq>S) { --q; sq-=2u*(uint64_t)q+1u; }
    while(q!=UINT32_MAX && S-sq>=2u*(uint64_t)q+1u) { sq+=2u*(uint64_t)q+1u; ++q; }
    const unsigned j=shift/2u;
    if(!j) {
        /* round: sqrt(S) >= q + 1/2  <=>  S - q^2 > q */
        if(S-sq>q) return q==UINT32_MAX ? UINT32_MAX : q+1u;
        return q;
    }
    return (sh2zi_u30_shr(q,j-1u)+1u)>>1;
}

/* ---- orthonormalisation ----
 * The last cross product of each takes two normalised rows (every component
 * within 2^30 + 2, length within 1 + 2^-28), so no component can saturate:
 * sh2z_u30_cross without the saturation test gives the same words. */
static inline void sh2zi_u30_cross_unit(int32_t out[3],const int32_t a[3],const int32_t b[3]) {
    const int32_t a0=a[0],a1=a[1],a2=a[2],b0=b[0],b1=b[1],b2=b[2];
    const uint64_t k=UINT64_C(0x20000000);
    const uint32_t c0=(uint32_t)(((uint64_t)((int64_t)a1*b2)-(uint64_t)((int64_t)a2*b1)+k)>>30);
    const uint32_t c1=(uint32_t)(((uint64_t)((int64_t)a2*b0)-(uint64_t)((int64_t)a0*b2)+k)>>30);
    const uint32_t c2=(uint32_t)(((uint64_t)((int64_t)a0*b1)-(uint64_t)((int64_t)a1*b0)+k)>>30);
    out[0]=sh2zi_u30_s32(c0); out[1]=sh2zi_u30_s32(c1); out[2]=sh2zi_u30_s32(c2);
}

int sh2z_basis30_orthonormalize_forward(sh2z_basis30_t *basis) {
    if(!sh2z_vec3_normalize_u30(basis->x,basis->x)) return -1;
    sh2z_u30_cross(basis->z,basis->y,basis->x);
    if(!sh2z_vec3_normalize_u30(basis->z,basis->z)) return -1;
    sh2zi_u30_cross_unit(basis->y,basis->x,basis->z);
    return 0;
}

int sh2z_basis30_orthonormalize_up(sh2z_basis30_t *basis) {
    if(!sh2z_vec3_normalize_u30(basis->y,basis->y)) return -1;
    sh2z_u30_cross(basis->z,basis->y,basis->x);
    if(!sh2z_vec3_normalize_u30(basis->z,basis->z)) return -1;
    sh2zi_u30_cross_unit(basis->x,basis->z,basis->y);
    return 0;
}
