/* SH2ZAM quarter-wave sine lookup: Q16.16, 257 samples, 1,028-byte table.
 * Values generated from the mathematical sin() definition; MIT, root LICENSE.
 * Copyright (c) 2026 SH2ZAM Contributors.
 */
#include "sh2zam/sh2zam.h"

/* Quarter turn = 16384 units. Linear interpolation between 64-unit steps. */
static const int32_t sin_quarter[257] = {
    0, 402, 804, 1206, 1608, 2010, 2412, 2814,
    3216, 3617, 4019, 4420, 4821, 5222, 5623, 6023,
    6424, 6824, 7224, 7623, 8022, 8421, 8820, 9218,
    9616, 10014, 10411, 10808, 11204, 11600, 11996, 12391,
    12785, 13180, 13573, 13966, 14359, 14751, 15143, 15534,
    15924, 16314, 16703, 17091, 17479, 17867, 18253, 18639,
    19024, 19409, 19792, 20175, 20557, 20939, 21320, 21699,
    22078, 22457, 22834, 23210, 23586, 23961, 24335, 24708,
    25080, 25451, 25821, 26190, 26558, 26925, 27291, 27656,
    28020, 28383, 28745, 29106, 29466, 29824, 30182, 30538,
    30893, 31248, 31600, 31952, 32303, 32652, 33000, 33347,
    33692, 34037, 34380, 34721, 35062, 35401, 35738, 36075,
    36410, 36744, 37076, 37407, 37736, 38064, 38391, 38716,
    39040, 39362, 39683, 40002, 40320, 40636, 40951, 41264,
    41576, 41886, 42194, 42501, 42806, 43110, 43412, 43713,
    44011, 44308, 44604, 44898, 45190, 45480, 45769, 46056,
    46341, 46624, 46906, 47186, 47464, 47741, 48015, 48288,
    48559, 48828, 49095, 49361, 49624, 49886, 50146, 50404,
    50660, 50914, 51166, 51417, 51665, 51911, 52156, 52398,
    52639, 52878, 53114, 53349, 53581, 53812, 54040, 54267,
    54491, 54714, 54934, 55152, 55368, 55582, 55794, 56004,
    56212, 56418, 56621, 56823, 57022, 57219, 57414, 57607,
    57798, 57986, 58172, 58356, 58538, 58718, 58896, 59071,
    59244, 59415, 59583, 59750, 59914, 60075, 60235, 60392,
    60547, 60700, 60851, 60999, 61145, 61288, 61429, 61568,
    61705, 61839, 61971, 62101, 62228, 62353, 62476, 62596,
    62714, 62830, 62943, 63054, 63162, 63268, 63372, 63473,
    63572, 63668, 63763, 63854, 63944, 64031, 64115, 64197,
    64277, 64354, 64429, 64501, 64571, 64639, 64704, 64766,
    64827, 64884, 64940, 64993, 65043, 65091, 65137, 65180,
    65220, 65259, 65294, 65328, 65358, 65387, 65413, 65436,
    65457, 65476, 65492, 65505, 65516, 65525, 65531, 65535,
    65536
};

static sh2z_fx_t lookup_quarter(uint16_t offset) {
    if (offset >= 16384) return SH2Z_ONE;
    const uint16_t index = (uint16_t)(offset >> 6);
    const uint16_t frac = (uint16_t)(offset & 63u);
    const int32_t a = sin_quarter[index], b = sin_quarter[index+1];
    /* The unchanged table is nonnegative and monotonic. Pulling a*64
     * outside the shift removes one multiply without changing rounding. */
    return (sh2z_fx_t)(a + (((b-a)*(int32_t)frac + 32) >> 6));
}

/* Offset is in [0,16383]. A cosine uses the reflected
 * table cell; +31 preserves the original +32 rounding at its opposite end. */
static void lookup_pair(uint16_t offset,sh2z_fx_t *s,sh2z_fx_t *c) {
    const unsigned index=(unsigned)(offset>>6);
    const int32_t frac=(int32_t)(offset&63u);
    const int32_t s0=sin_quarter[index],c0=sin_quarter[256u-index];
    *s=s0+(((sin_quarter[index+1u]-s0)*frac+32)>>6);
    *c=c0-(((c0-sin_quarter[255u-index])*frac+31)>>6);
}

sh2z_angle_t sh2z_angle_from_degrees(int32_t degrees) {
    int32_t normalized = degrees % 360;
    if (normalized < 0) normalized += 360;
    return (sh2z_angle_t)((normalized * 65536 + 180) / 360);
}
sh2z_fx_t sh2z_sin(sh2z_angle_t angle) {
    const unsigned quadrant = (unsigned)(angle >> 14);
    const uint16_t offset = (uint16_t)(angle & 16383u);
    if (quadrant == 0) return lookup_quarter(offset);
    if (quadrant == 1) return lookup_quarter((uint16_t)(16384u-offset));
    if (quadrant == 2) return -lookup_quarter(offset);
    return -lookup_quarter((uint16_t)(16384u-offset));
}
sh2z_fx_t sh2z_cos(sh2z_angle_t angle) {
    return sh2z_sin((sh2z_angle_t)(angle + SH2Z_ANGLE_QUARTER));
}

void sh2z_sincos(sh2z_angle_t angle,sh2z_fx_t *s,sh2z_fx_t *c) {
    /* Keep single-output calls cheap, and retain the public null semantics. */
    if(!s) { if(c) *c=sh2z_cos(angle); return; }
    if(!c) { *s=sh2z_sin(angle); return; }
    sh2z_fx_t sv,cv;
    lookup_pair((uint16_t)(angle&16383u),&sv,&cv);
    if(angle&SH2Z_ANGLE_QUARTER) {
        const sh2z_fx_t t=sv;
        sv=cv; cv=-t;
    }
    if(angle&SH2Z_ANGLE_HALF) { sv=-sv; cv=-cv; }
    /* Store in the same order as the separate calls, including s == c. */
    *s=sv; *c=cv;
}

sh2z_angle_t sh2z_atan2(sh2z_fx_t y,sh2z_fx_t x) {
    if(!y) return (sh2z_angle_t)(x<0 ? SH2Z_ANGLE_HALF : 0);
    if(!x) return y>0 ? SH2Z_ANGLE_QUARTER : (sh2z_angle_t)49152u;
    const uint32_t ax=x<0 ? 0u-(uint32_t)x : (uint32_t)x;
    const uint32_t ay=y<0 ? 0u-(uint32_t)y : (uint32_t)y;
    unsigned lo=0,hi=256;
    /* First find the table cell directly: eight comparisons, with no trig
     * calls or interpolation. The strict comparison preserves which side
     * contains an exactly represented direction, including table plateaus. */
    while(hi-lo>1u) {
        const unsigned mid=(lo+hi)>>1;
        if((uint64_t)ay*(uint32_t)sin_quarter[256u-mid]>
           (uint64_t)ax*(uint32_t)sin_quarter[mid]) lo=mid;
        else hi=mid;
    }
    const unsigned base=lo<<6;
    const int32_t s0=sin_quarter[lo],c0=sin_quarter[256u-lo];
    const int32_t ds=sin_quarter[lo+1u]-s0;
    const int32_t dc=c0-sin_quarter[255u-lo];
    lo=0; hi=64;
    /* Then resolve every original integer-angle candidate inside that cell.
     * Reuse its four table values; all interpolations remain bit-exact. */
    while(hi-lo>1u) {
        const unsigned mid=(lo+hi)>>1;
        const uint32_t sv=(uint32_t)(s0+((ds*(int32_t)mid+32)>>6));
        const uint32_t cv=(uint32_t)(c0-((dc*(int32_t)mid+31)>>6));
        if((uint64_t)ay*cv>(uint64_t)ax*sv) lo=mid;
        else hi=mid;
    }
    const uint32_t slo=(uint32_t)(s0+((ds*(int32_t)lo+32)>>6));
    const uint32_t shi=(uint32_t)(s0+((ds*(int32_t)hi+32)>>6));
    const uint32_t clo=(uint32_t)(c0-((dc*(int32_t)lo+31)>>6));
    const uint32_t chi=(uint32_t)(c0-((dc*(int32_t)hi+31)>>6));
    /* F(a)=ay*cos(a)-ax*sin(a) is monotonic. F(lo)>0 and F(hi)<=0,
     * so abs(F(lo))<=abs(F(hi)) iff F(lo)+F(hi)<=0. Comparing summed
     * table values needs only two 32x32 products and keeps the lower tie.
     * Even INT32_MIN magnitudes fit: each final product is at most 2^48. */
    unsigned a=base+((uint64_t)ay*(clo+chi)<=(uint64_t)ax*(slo+shi) ? lo:hi);
    if(x<0) a=32768u-a;
    if(y<0) a=65536u-a;
    return (sh2z_angle_t)a;
}
