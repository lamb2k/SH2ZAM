/* Independent oracle for view-camera-1 (sh2z_view_place_q12 and
 * sh2z_view_from_camera30). Host only: 128-bit integers, long double and
 * libm. It shares no code or table with the library: the cotangent entries
 * are re-derived from cosl/sinl, and every rounding is written as plain
 * integer division. MIT; SH2ZAM Contributors. */
#ifndef SH2ZAM_CAMERA_ORACLE_H
#define SH2ZAM_CAMERA_ORACLE_H

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "sh2zam/view_experimental.h"

__extension__ typedef __int128 co_wide_t;

/* Mutations of the oracle, for the negative controls. */
enum {
    CO_MUT_R_FLOOR=1u<<0,      /* rotation rounded by floor, not half away */
    CO_MUT_T_TRUNC=1u<<1,      /* t from a truncating, not flooring, division */
    CO_MUT_H_TRUNC=1u<<2,      /* half angle truncated, not rounded */
    CO_MUT_FOCAL_TRUNC=1u<<3,  /* focal truncated, not rounded */
    CO_MUT_COT_NEXT=1u<<4,     /* the next table entry */
    CO_MUT_CENTER_TRUNC=1u<<5, /* centre floors by truncation */
    CO_MUT_ROW_STRICT=1u<<6,   /* row bound with < instead of <= */
    CO_MUT_DEPTH_Q16=1u<<7     /* near/far scaled by 16, as if Q16.16 */
};

static co_wide_t co_floor_div(co_wide_t n,co_wide_t d) {
    co_wide_t q=n/d;
    if((n%d!=0) && ((n<0)!=(d<0)))--q;
    return q;
}

/* round(cot(pi h / 2048) * 2^20), ties up, from long double; aborts if an
 * entry is too close to a tie for long double to decide. */
static int64_t co_cot_q20(unsigned h) {
    static int64_t cache[1025];
    static int ready;
    if(!ready) {
        const long double pi=3.141592653589793238462643383279502884L;
        for(unsigned k=1;k<1024;++k) {
            const long double x=pi*(long double)k/2048.0L;
            const long double v=cosl(x)/sinl(x)*1048576.0L;
            const long double fl=floorl(v),frac=v-fl;
            if(fabsl(frac-0.5L)<1e-9L) {
                fprintf(stderr,"cot entry %u is within 1e-9 of a tie\n",k);
                exit(2);
            }
            cache[k]=(int64_t)fl+(frac>0.5L?1:0);
        }
        cache[0]=0;cache[1024]=0;
        ready=1;
    }
    return cache[h];
}

static int co_row_ok(const int16_t r[3],co_wide_t t,unsigned mutation) {
    co_wide_t s=0;
    for(unsigned j=0;j<3;++j)s+=r[j]<0?-(co_wide_t)r[j]:(co_wide_t)r[j];
    const co_wide_t total=s*32768+(t<0?-t:t);
    return (mutation&CO_MUT_ROW_STRICT)?total<(co_wide_t)INT32_MAX:total<=(co_wide_t)INT32_MAX;
}
static int co_view_ok(const sh2z_view_t *v) {
    if(v->focal_x<=0 || v->focal_y<=0 || v->near_z<=0 || v->far_z<=v->near_z ||
       v->clip[0]>v->clip[2] || v->clip[1]>v->clip[3])return 0;
    for(unsigned i=0;i<3;++i)if(!co_row_ok(v->r[i],v->t[i],0))return 0;
    return 1;
}
/* t for one row from an eye in Q20.12; 0 if it does not fit or fails the bound. */
static int co_place_row(const int16_t r[3],const int32_t eye[3],int32_t *t,unsigned mutation) {
    co_wide_t s=0;
    for(unsigned j=0;j<3;++j)s+=(co_wide_t)r[j]*eye[j];
    const co_wide_t q=(mutation&CO_MUT_T_TRUNC)?s/4096:co_floor_div(s,4096);
    const co_wide_t value=-q;
    if(value<INT32_MIN || value>INT32_MAX)return 0;
    if(!co_row_ok(r,value,mutation))return 0;
    *t=(int32_t)value;
    return 1;
}
static int co_place_q12(sh2z_view_t *v,const int32_t eye[3],unsigned mutation) {
    if(!v || !eye || !co_view_ok(v))return 0;
    int32_t t[3];
    for(unsigned i=0;i<3;++i)if(!co_place_row(v->r[i],eye,&t[i],mutation))return 0;
    for(unsigned i=0;i<3;++i)v->t[i]=t[i];
    return 1;
}

/* The half-angle index of a Q16.16 field of view, or 0 when out of range. */
static unsigned co_half_angle(int32_t fov,unsigned mutation) {
    const co_wide_t n=(mutation&CO_MUT_H_TRUNC)?(co_wide_t)fov:(co_wide_t)fov+5760;
    if(fov<=0)return 0;
    const co_wide_t h=co_floor_div(n,11520);
    return h>=1 && h<=1023?(unsigned)h:0u;
}
static int co_focal(int32_t scale,int64_t c,int32_t *out,unsigned mutation) {
    if(scale<=0)return 0;
    const co_wide_t p=(co_wide_t)scale*c;
    const co_wide_t q=(mutation&CO_MUT_FOCAL_TRUNC)?p/1048576:co_floor_div(p+524288,1048576);
    if(q<1 || q>INT32_MAX)return 0;
    *out=(int32_t)q;
    return 1;
}
static int co_round_u30(int32_t b,int16_t *r,unsigned mutation) {
    co_wide_t q;
    if(mutation&CO_MUT_R_FLOOR)q=co_floor_div(b,65536);
    else q=b>=0?((co_wide_t)b+32768)/65536:-((-(co_wide_t)b+32768)/65536);
    if(q<INT16_MIN || q>INT16_MAX)return 0;
    *r=(int16_t)q;
    return 1;
}

static int co_from_camera30(sh2z_view_center_t *out,const int32_t basis[9],const int32_t eye[3],
    const sh2z_view_lens_t *lens,unsigned mutation) {
    if(!out || !basis || !eye || !lens)return 0;
    /* The range rule is on fov itself; the mutation only moves the rounding. */
    if(lens->fov_q16<5760 || lens->fov_q16>11790719)return 0;
    unsigned h=co_half_angle(lens->fov_q16,mutation);
    if(!h)return 0;
    if((mutation&CO_MUT_COT_NEXT) && h<1023)++h;
    const int64_t c=co_cot_q20(h);
    sh2z_view_t v;
    if(!co_focal(lens->scale_x16,c,&v.focal_x,mutation) || !co_focal(lens->scale_y16,c,&v.focal_y,mutation))return 0;
    const co_wide_t mul=(mutation&CO_MUT_DEPTH_Q16)?16:4;
    if(lens->near_q12<=0 || lens->far_q12<=lens->near_q12 || (co_wide_t)lens->far_q12*4>INT32_MAX)return 0;
    if((co_wide_t)lens->far_q12*mul>INT32_MAX)return 0;
    v.near_z=(int32_t)((co_wide_t)lens->near_q12*mul);
    v.far_z=(int32_t)((co_wide_t)lens->far_q12*mul);
    if(lens->clip[0]>lens->clip[2] || lens->clip[1]>lens->clip[3])return 0;
    for(unsigned i=0;i<4;++i)v.clip[i]=lens->clip[i];
    for(unsigned i=0;i<3;++i)for(unsigned j=0;j<3;++j)if(!co_round_u30(basis[3*i+j],&v.r[i][j],mutation))return 0;
    for(unsigned i=0;i<3;++i)if(!co_place_row(v.r[i],eye,&v.t[i],mutation))return 0;
    const co_wide_t cx=lens->center_x16,cy=lens->center_y16;
    v.center_x=(int16_t)((mutation&CO_MUT_CENTER_TRUNC)?cx/65536:co_floor_div(cx,65536));
    v.center_y=(int16_t)((mutation&CO_MUT_CENTER_TRUNC)?cy/65536:co_floor_div(cy,65536));
    out->view=v;
    out->center_x16=lens->center_x16;
    out->center_y16=lens->center_y16;
    return 1;
}

/* Field-wise equality (padding ignored). */
static int co_same_view(const sh2z_view_t *a,const sh2z_view_t *b) {
    for(unsigned i=0;i<3;++i) {
        for(unsigned j=0;j<3;++j)if(a->r[i][j]!=b->r[i][j])return 0;
        if(a->t[i]!=b->t[i])return 0;
    }
    for(unsigned i=0;i<4;++i)if(a->clip[i]!=b->clip[i])return 0;
    return a->focal_x==b->focal_x && a->focal_y==b->focal_y && a->center_x==b->center_x &&
        a->center_y==b->center_y && a->near_z==b->near_z && a->far_z==b->far_z;
}
static int co_same_center(const sh2z_view_center_t *a,const sh2z_view_center_t *b) {
    return co_same_view(&a->view,&b->view) && a->center_x16==b->center_x16 && a->center_y16==b->center_y16;
}

#endif
