/* Inverse trig and angle helpers. MIT; (c) 2026 SH2ZAM Contributors. */
#include "sh2zam_internal.h"

int32_t sh2z_angle_delta(sh2z_angle_t a,sh2z_angle_t b) {
    return (int32_t)(((uint32_t)b-(uint32_t)a+32768u)&65535u)-32768;
}
sh2z_angle_t sh2z_angle_lerp(sh2z_angle_t a,sh2z_angle_t b,sh2z_fx_t t) {
    t=sh2z_clamp(t,0,SH2Z_ONE);
    return (sh2z_angle_t)((int32_t)a+(int32_t)(((int64_t)sh2z_angle_delta(a,b)*t)/65536));
}
void sh2z_sincos(sh2z_angle_t a,sh2z_fx_t *s,sh2z_fx_t *c) {
    if(s) *s=sh2z_sin(a);
    if(c) *c=sh2z_cos(a);
}
sh2z_fx_t sh2z_tan(sh2z_angle_t a) { return sh2z_div(sh2z_sin(a),sh2z_cos(a)); }
sh2z_angle_t sh2z_atan2(sh2z_fx_t y,sh2z_fx_t x) {
    if(!y) return (sh2z_angle_t)(x<0 ? SH2Z_ANGLE_HALF : 0);
    if(!x) return y>0 ? SH2Z_ANGLE_QUARTER : (sh2z_angle_t)49152u;
    uint64_t ax=sh2zi_magnitude(x),ay=sh2zi_magnitude(y);
    unsigned lo=0,hi=16384;
    /* Invert monotonic first-quadrant direction; no division or float.
     * Products are at most 2^47, including INT32_MIN coordinates. */
    while(hi-lo>1) {
        unsigned mid=(lo+hi)/2;
        if(ay*(uint32_t)sh2z_cos((sh2z_angle_t)mid)>ax*(uint32_t)sh2z_sin((sh2z_angle_t)mid)) lo=mid;
        else hi=mid;
    }
    int64_t elo=(int64_t)(ay*(uint32_t)sh2z_cos((sh2z_angle_t)lo))-(int64_t)(ax*(uint32_t)sh2z_sin((sh2z_angle_t)lo));
    int64_t ehi=(int64_t)(ay*(uint32_t)sh2z_cos((sh2z_angle_t)hi))-(int64_t)(ax*(uint32_t)sh2z_sin((sh2z_angle_t)hi));
    if(elo<0) elo=-elo;
    if(ehi<0) ehi=-ehi;
    unsigned a=elo<=ehi ? lo:hi;
    if(x<0) a=32768u-a;
    if(y<0) a=65536u-a;
    return (sh2z_angle_t)a;
}
sh2z_angle_t sh2z_atan(sh2z_fx_t x) { return sh2z_atan2(x,SH2Z_ONE); }
sh2z_angle_t sh2z_asin(sh2z_fx_t x) {
    x=sh2z_clamp(x,-SH2Z_ONE,SH2Z_ONE);
    uint64_t square=(uint64_t)((int64_t)x*x);
    sh2z_fx_t c=(sh2z_fx_t)sh2zi_isqrt(UINT64_C(4294967296)-square);
    return sh2z_atan2(x,c);
}
sh2z_angle_t sh2z_acos(sh2z_fx_t x) { return (sh2z_angle_t)(16384u-sh2z_asin(x)); }
