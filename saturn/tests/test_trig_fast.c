/* Exact regression coverage for the shared-table trig implementation.
 * The reference is the unmodified pre-optimization source in
 * benchmarks/math-model/legacy, linked with its legacy_ symbol prefix.
 * MIT; Copyright (c) 2026 SH2ZAM Contributors. */
#include "sh2zam/sh2zam.h"
#include <inttypes.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>

sh2z_fx_t legacy_sh2z_sin(sh2z_angle_t a);
sh2z_fx_t legacy_sh2z_cos(sh2z_angle_t a);
sh2z_fx_t legacy_sh2z_tan(sh2z_angle_t a);
sh2z_angle_t legacy_sh2z_atan2(sh2z_fx_t y,sh2z_fx_t x);
sh2z_angle_t legacy_sh2z_atan(sh2z_fx_t x);
sh2z_angle_t legacy_sh2z_asin(sh2z_fx_t x);
sh2z_angle_t legacy_sh2z_acos(sh2z_fx_t x);

static uint64_t checks;
static void equal(const char *name,int32_t x,int32_t y,int32_t got,int32_t want) {
    ++checks;
    if(got!=want) {
        fprintf(stderr,"%s( x=%" PRId32 ", y=%" PRId32 "): got %" PRId32
                ", expected %" PRId32 "\n",name,x,y,got,want);
        exit(EXIT_FAILURE);
    }
}

static void all_angles(void) {
    for(uint32_t a=0;a<65536u;++a) {
        const sh2z_angle_t angle=(sh2z_angle_t)a;
        const sh2z_fx_t rs=legacy_sh2z_sin(angle),rc=legacy_sh2z_cos(angle);
        equal("sin",(int32_t)a,0,sh2z_sin(angle),rs);
        equal("cos",(int32_t)a,0,sh2z_cos(angle),rc);
        sh2z_fx_t s=INT32_MIN,c=INT32_MIN;
        sh2z_sincos(angle,&s,&c);
        equal("sincos.sin",(int32_t)a,0,s,rs);
        equal("sincos.cos",(int32_t)a,0,c,rc);
        sh2z_sincos(angle,&s,NULL);
        equal("sincos.sin_only",(int32_t)a,0,s,rs);
        sh2z_sincos(angle,NULL,&c);
        equal("sincos.cos_only",(int32_t)a,0,c,rc);
        sh2z_sincos(angle,&s,&s);
        equal("sincos.alias",(int32_t)a,0,s,rc);
        sh2z_sincos(angle,NULL,NULL);
        equal("tan",(int32_t)a,0,sh2z_tan(angle),legacy_sh2z_tan(angle));
    }
}

static void direction(int32_t x,int32_t y) {
    equal("atan2",x,y,sh2z_atan2(y,x),legacy_sh2z_atan2(y,x));
}

/* Positive magnitudes here stay below INT32_MAX; no INT32_MIN negation. */
static void quadrants(int32_t x,int32_t y) {
    direction(x,y); direction(-x,y);
    direction(x,-y); direction(-x,-y);
}

static void angular_boundaries(void) {
    for(uint32_t a=0;a<16384u;++a) {
        const int32_t s=legacy_sh2z_sin((sh2z_angle_t)a);
        const int32_t c=legacy_sh2z_cos((sh2z_angle_t)a);
        quadrants(c,s);
        /* This ray lies exactly at the residual-error tie between adjacent
         * output angles. Check the lower-angle rule and both sides of every
         * tie after scaling close to the full signed coordinate range. */
        const int32_t y=s+legacy_sh2z_sin((sh2z_angle_t)(a+1u));
        const int32_t x=c+legacy_sh2z_cos((sh2z_angle_t)(a+1u));
        equal("atan2.lower_tie",x,y,legacy_sh2z_atan2(y,x),(int32_t)a);
        quadrants(x,y);
        const int32_t scale=INT32_MAX/(x>y ? x:y);
        const int32_t sx=x*scale,sy=y*scale;
        quadrants(sx,sy);
        quadrants(sx-1,sy);
        quadrants(sx,sy-1);
        if(sx<INT32_MAX) quadrants(sx+1,sy);
        if(sy<INT32_MAX) quadrants(sx,sy+1);
    }
}

static void quotient_bins(void) {
    /* atan_octant[i] is generated as legacy_sh2z_atan2(i,256). Reconstruct
     * those exact values, then prove the one-unit estimate bound at both
     * inclusive endpoints of every Q16 ratio bin. The first-quadrant answer
     * is monotonic, so these exact rational bounds cover every input ratio
     * inside the bin, in both direct and reflected octants. */
    uint32_t table[257];
    for(uint32_t i=0;i<=256u;i++) table[i]=legacy_sh2z_atan2((int32_t)i,256);
    for(uint32_t ratio=0;ratio<65536u;ratio++) {
        const unsigned i=ratio>>8,f=ratio&255u;
        const int32_t seed=(int32_t)(table[i]+(((table[i+1u]-table[i])*f+128u)>>8));
        for(uint32_t endpoint=ratio;endpoint<=ratio+1u;endpoint++) {
            const int32_t direct=legacy_sh2z_atan2((int32_t)endpoint,65536);
            const int32_t reflected=legacy_sh2z_atan2(65536,(int32_t)endpoint);
            const int32_t delta=direct-seed,reflected_delta=reflected-(16384-seed);
            equal("atan2.direct_estimate_bound",(int32_t)ratio,(int32_t)endpoint,
                  delta>=-1 && delta<=1,1);
            equal("atan2.reflected_estimate_bound",(int32_t)ratio,(int32_t)endpoint,
                  reflected_delta>=-1 && reflected_delta<=1,1);
        }
        /* Verify the actual implementation on each bin boundary in all
         * quadrants and both octants, including tiny ratios and endpoints. */
        quadrants(65536,(int32_t)ratio);
        quadrants((int32_t)ratio,65536);
    }
}

static uint32_t random_state=UINT32_C(0x6d2b79f5);
static int32_t random_signed(void) {
    uint32_t u=random_state;
    u^=u<<13; u^=u>>17; u^=u<<5;
    random_state=u;
    return u<=INT32_MAX ? (int32_t)u : INT32_MIN+(int32_t)(u-UINT32_C(0x80000000));
}

static void full_range_inputs(void) {
    static const int32_t edges[]={
        INT32_MIN,INT32_MIN+1,-1073741824,-536870913,-536870912,
        -16777216,-8388609,-8388608,-1048576,-131073,-131072,
        -65537,-65536,-65535,-32768,-4096,-257,-256,-255,-65,-64,-63,
        -3,-2,-1,0,1,2,3,63,64,65,255,256,257,4096,32768,
        65535,65536,65537,131072,131073,1048576,8388608,8388609,
        16777216,536870912,536870913,1073741824,INT32_MAX-1,INT32_MAX
    };
    for(size_t i=0;i<sizeof(edges)/sizeof(edges[0]);++i) {
        for(size_t j=0;j<sizeof(edges)/sizeof(edges[0]);++j) direction(edges[i],edges[j]);
        equal("atan.edge",edges[i],0,sh2z_atan(edges[i]),legacy_sh2z_atan(edges[i]));
        equal("asin.clamp",edges[i],0,sh2z_asin(edges[i]),legacy_sh2z_asin(edges[i]));
        equal("acos.clamp",edges[i],0,sh2z_acos(edges[i]),legacy_sh2z_acos(edges[i]));
    }
    /* Preserve large magnitude ratios as well as uniformly distributed
     * signed coordinates, where INT32_MIN must remain a positive magnitude. */
    for(unsigned bit=0;bit<31u;++bit) {
        const int32_t power=(int32_t)(UINT32_C(1)<<bit);
        for(int32_t delta=-1;delta<=1;++delta) {
            const int32_t small=power+delta;
            for(size_t i=0;i<sizeof(edges)/sizeof(edges[0]);++i) {
                direction(small,edges[i]); direction(-small,edges[i]);
                direction(edges[i],small); direction(edges[i],-small);
            }
        }
    }
    for(unsigned i=0;i<200000u;++i) {
        const int32_t x=random_signed(),y=random_signed();
        direction(x,y);
        equal("atan.random",x,0,sh2z_atan(x),legacy_sh2z_atan(x));
    }
}

static void inverse_domain(void) {
    /* Every possible representable value inside asin/acos's Q16 domain,
     * including endpoint square-root zero and values one LSB from +/-1. */
    for(int32_t x=-SH2Z_ONE;x<=SH2Z_ONE;++x) {
        equal("asin.domain",x,0,sh2z_asin(x),legacy_sh2z_asin(x));
        equal("acos.domain",x,0,sh2z_acos(x),legacy_sh2z_acos(x));
    }
}

int main(void) {
    all_angles(); angular_boundaries(); quotient_bins(); full_range_inputs(); inverse_domain();
    printf("%" PRIu64 " exact trig checks passed against frozen pre-optimization source\n",checks);
    return EXIT_SUCCESS;
}
