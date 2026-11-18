/* Independent oracles for contract tag trig12-1 (trig12_experimental.h and the
 * fixed-point quotients in divu_experimental.h). Shared by the host tests and
 * the SH-2 model harness. MIT; Copyright (c) 2026 SH2ZAM Contributors.
 *
 * Nothing here calls the library or reads its table. The sine reference is
 * the series in binary64, written out again; the quotients use 128-bit
 * integers. Mutations (T12_MUT_*) are deliberately wrong variants used as
 * negative controls: a test must see each of them disagree. */
#ifndef SH2ZAM_TRIG12_ORACLE_H
#define SH2ZAM_TRIG12_ORACLE_H
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

__extension__ typedef __int128 t12_wide_t;

enum {
    T12_MUT_NONE=0,
    T12_MUT_TABLE_TRUNC=1<<0,     /* table entries truncated instead of rounded */
    T12_MUT_TABLE_PI=1<<1,        /* series at the libm value of pi */
    T12_MUT_TABLE_B32=1<<2,       /* table rounded from the binary32 series value */
    T12_MUT_COS_BACKWARD=1<<3,    /* cos(a) taken as sin(a - quarter) */
    T12_MUT_SIGN_BIT=1<<4,        /* sine sign from the quarter bit */
    T12_MUT_TAN_TRUNC=1<<5,       /* tangent truncated instead of rounded */
    T12_MUT_TAN_POLE=1<<6,        /* poles saturate to the opposite sign */
    T12_MUT_Q_FLOOR=1<<7,         /* quotient rounded toward minus infinity */
    T12_MUT_Q_FITS_EQUAL=1<<8,    /* |n| == |d| * 2^31 treated as fitting */
    T12_MUT_Q_ZERO_ZERO=1<<9,     /* 0 / 0 = 0, as sh2z_div answers */
    T12_MUT_Q_SHIFT=1<<10         /* dividend num * 2^(f - 1) */
};

/* The binary64 series value S(i) for i in [0, 4096): the statements of the
 * table definition, one operation per statement. */
static inline double t12_series(int i,unsigned mutation) {
    const double pi=(mutation&T12_MUT_TABLE_PI)?3.14159265358979323846:3.1415926535;
    double x=(i*(2*pi))/4096;
    double numerator=-x*x*x;
    const double minus_square=-x*x;
    double denominator=1*2*3;
    for(int j=2;j<14;j++) {
        x+=numerator/denominator;
        numerator*=minus_square;
        denominator*=j*((j<<2)+2);
    }
    return x;
}

/* Q2.30 table entry i in [0, 1024]: the nearest integer to S(i) * 2^30.
 * The scaling is exact; floor(v + 0.5) is exact for |v| < 2^31 because a
 * tie never occurs (the generator checks the closest distance). */
static inline int32_t t12_entry(int i,unsigned mutation) {
    double v=t12_series(i,mutation);
    if(mutation&T12_MUT_TABLE_B32)v=(double)(float)v;
    v*=1073741824.0;
    return (int32_t)((mutation&T12_MUT_TABLE_TRUNC)?floor(v):floor(v+0.5));
}

/* sin of a 12-bit angle from a half-turn reduction: a in [0, 2048) is the
 * positive half, sin(a) = sin(2048 - a). */
static inline int32_t t12_sin(uint32_t a,unsigned mutation) {
    a&=4095u;
    const uint32_t h=a&2047u;
    const int32_t v=t12_entry((int)(h<=1024u?h:2048u-h),mutation);
    const bool negative=(mutation&T12_MUT_SIGN_BIT)?(a&1024u)!=0:a>=2048u;
    return negative?-v:v;
}
static inline int32_t t12_cos(uint32_t a,unsigned mutation) {
    return t12_sin((mutation&T12_MUT_COS_BACKWARD)?a-1024u:a+1024u,mutation);
}

/* tan in Q16.16 from the oracle's sin and cos: round half away from zero of
 * sin * 2^16 / cos from magnitudes, (2 |n| 2^16 + |d|) / (2 |d|). Poles
 * (cos = 0) saturate by the sign of the sine. */
static inline int32_t t12_tan(uint32_t a,unsigned mutation) {
    const int64_t s=t12_sin(a,mutation&~(unsigned)T12_MUT_TAN_POLE),c=t12_cos(a,mutation);
    if(c==0) {
        const bool up=(s>0)!=((mutation&T12_MUT_TAN_POLE)!=0);
        return up?INT32_MAX:INT32_MIN;
    }
    const uint64_t n=(uint64_t)(s<0?-s:s),d=(uint64_t)(c<0?-c:c);
    const uint64_t q=(mutation&T12_MUT_TAN_TRUNC)?(n<<16)/d:((n<<17)+d)/(2u*d);
    return ((s<0)!=(c<0))?-(int32_t)q:(int32_t)q;
}

/* The quotient rule of divu_experimental.h for num * 2^f / den. */
static inline int32_t t12_divq(int32_t num,int32_t den,unsigned f,bool *overflow,unsigned mutation) {
    if(mutation&T12_MUT_Q_SHIFT)f=f?f-1u:0u;
    const t12_wide_t n=(t12_wide_t)num*((t12_wide_t)1<<f);
    const t12_wide_t an=n<0?-n:n,ad=den<0?-(t12_wide_t)den:(t12_wide_t)den;
    const t12_wide_t limit=ad*((t12_wide_t)1<<31);
    if(den==0) {
        *overflow=true;
        if((mutation&T12_MUT_Q_ZERO_ZERO)&&num==0)return 0;
        return num>=0?INT32_MAX:INT32_MIN;
    }
    *overflow=(mutation&T12_MUT_Q_FITS_EQUAL)?an>limit:an>=limit;
    const bool negative=(n<0)!=(den<0);
    t12_wide_t q=an/ad;
    if((mutation&T12_MUT_Q_FLOOR)&&negative&&an%ad!=0)q+=1;
    if(negative)q=-q;
    return q>INT32_MAX?INT32_MAX:q<INT32_MIN?INT32_MIN:(int32_t)q;
}
#endif
