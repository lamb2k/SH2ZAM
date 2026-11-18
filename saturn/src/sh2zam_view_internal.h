/* Private to sh2zam_view.c and sh2zam_view_round.c: argument rules, the
 * edge stepper's fixed-point axes, the near clip and the guard clip's planes
 * and passes. Each file compiles its own static copy.
 *
 * Copyright (c) 2026 SH2ZAM Contributors. MIT License.
 * Inspired by SH4ZAM by Falco Girgis and SH4ZAM Contributors.
 */
#ifndef SH2ZAM_VIEW_INTERNAL_H
#define SH2ZAM_VIEW_INTERNAL_H

#include "sh2zam/view_experimental.h"
#include "sh2zam/divu_experimental.h"

static uint32_t magnitude32(int32_t a) {
    return a<0?(uint32_t)0-(uint32_t)a:(uint32_t)a;
}
static bool row_fits(const int16_t r[3],int32_t t) {
    uint64_t sum=0;
    for(unsigned j=0;j<3;++j)sum+=magnitude32(r[j]);
    return sum*UINT64_C(32768)+magnitude32(t)<=(uint64_t)INT32_MAX;
}
static bool view_valid(const sh2z_view_t *v) {
    if(!v || v->focal_x<=0 || v->focal_y<=0 || v->near_z<=0 || v->far_z<=v->near_z ||
       v->clip[0]>v->clip[2] || v->clip[1]>v->clip[3])return false;
    for(unsigned i=0;i<3;++i)if(!row_fits(v->r[i],v->t[i]))return false;
    return true;
}
static int16_t sat16(int32_t a) {
    return (int16_t)(a<INT16_MIN?INT16_MIN:a>INT16_MAX?INT16_MAX:a);
}
static int32_t shift16(int32_t q) {
    /* floor(q / 65536) without implementation-defined signed shifts:
     * for negative q, floor(q / 2^16) = -((~q) >> 16) - 1. */
    return q>=0?(int32_t)((uint32_t)q>>16):-(int32_t)(~(uint32_t)q>>16)-1;
}
static bool overlaps(const void *a,size_t a_bytes,const void *b,size_t b_bytes) {
    const uintptr_t x=(uintptr_t)a,y=(uintptr_t)b;
    return x<y+b_bytes && y<x+a_bytes;
}
/* Argument rules shared by every batch entry point. count > 0 here. */
static bool batch_args_ok(const sh2z_view_vertex_t *out,const void *positions,
    size_t in_stride,size_t count) {
    if(!out || !positions || in_stride<6 || (in_stride&1u) || ((uintptr_t)positions&1u))return false;
    if(count>SIZE_MAX/sizeof(*out) || (count-1)>(SIZE_MAX-6)/in_stride)return false;
    const size_t out_bytes=count*sizeof(*out),in_bytes=(count-1)*in_stride+6;
    return !((uintptr_t)out>UINTPTR_MAX-out_bytes || (uintptr_t)positions>UINTPTR_MAX-in_bytes ||
       overlaps(out,out_bytes,positions,in_bytes));
}
#if defined(__GNUC__)
#define SH2ZI_ALWAYS_INLINE inline __attribute__((always_inline))
#else
#define SH2ZI_ALWAYS_INLINE inline
#endif
/* trunc(magnitude * num * 65536 / den) exactly, for magnitude < 2^33,
 * 0 < den < 2^16 and num <= den (so the result is below 2^49).
 * Long division of (magnitude * num) << 16 in 16-bit digits: every step
 * divides a value below 2^32 (remainder < den < 2^16, shifted, plus a
 * digit), so only 32-bit divisions are needed, never a 64-bit one. */
static uint64_t scaled_fraction(uint64_t magnitude,uint32_t num,uint32_t den) {
    const uint64_t m=magnitude*num;          /* < 2^49 */
    const uint32_t digits[5]={(uint32_t)(m>>48)&0xFFFFu,(uint32_t)(m>>32)&0xFFFFu,
        (uint32_t)m>>16,(uint32_t)m&0xFFFFu,0u};
    uint64_t q=0;
    uint32_t r=0;
    for(unsigned i=0;i<5;++i) {
        const uint32_t cur=(r<<16)|digits[i];
        const uint32_t d=cur/den;
        r=cur-d*den;
        q=(q<<16)|d;
    }
    return q;
}
typedef sh2z_view_vertex_t (*project_fn)(const sh2z_view_t *,int32_t,int32_t,int32_t);
/* One axis of the stepper, in magnitudes: acc = |F| + k*|S| and
 * P = from +/- (acc >> 16). acc <= |delta| * 65536 < 2^49 by the bound on
 * count. Each P lies between from and to, so it fits int32 and only the low
 * 32 bits of acc >> 16 are needed: modular 32-bit arithmetic gives it. */
typedef struct { uint64_t acc,step; uint32_t base; bool negative; } edge_axis_t;
#if SH2Z_DIVU_NATIVE
/* scaled_fraction on the hardware divider, for magnitude < 2^31:
 * q1 = magnitude * num / den fits strictly (it is at most magnitude), its
 * remainder r1 < den < 2^16, and (r1 << 16) / den < 2^16 fits too. */
static uint64_t scaled_fraction_divu(uint64_t magnitude,uint32_t num,uint32_t den) {
    if(magnitude>=UINT64_C(0x80000000))return scaled_fraction(magnitude,num,den);
    const uint64_t m=magnitude*num;
    const uint32_t q1=(uint32_t)sh2z_divu_div64((int64_t)m,(int32_t)den,NULL);
    const uint32_t r1=(uint32_t)(m-(uint64_t)q1*den);
    const uint32_t q2=(uint32_t)sh2z_divu_div64((int64_t)((uint64_t)r1<<16),(int32_t)den,NULL);
    return ((uint64_t)q1<<16)+q2;
}
#endif
static edge_axis_t edge_axis(int32_t from,int32_t to,uint32_t first_num,uint32_t step_num,uint32_t den,bool divu) {
    const int64_t delta=(int64_t)to-from;
    const uint64_t mag=delta<0?(uint64_t)0-(uint64_t)delta:(uint64_t)delta;
#if SH2Z_DIVU_NATIVE
    if(divu) {
        edge_axis_t e={scaled_fraction_divu(mag,first_num,den),scaled_fraction_divu(mag,step_num,den),(uint32_t)from,delta<0};
        return e;
    }
#else
    (void)divu;
#endif
    edge_axis_t e={scaled_fraction(mag,first_num,den),scaled_fraction(mag,step_num,den),(uint32_t)from,delta<0};
    return e;
}
static SH2ZI_ALWAYS_INLINE int32_t edge_next(edge_axis_t *e) {
    const uint32_t whole=((uint32_t)(e->acc>>32)<<16)|((uint32_t)e->acc>>16);
    const uint32_t sum=e->negative?e->base-whole:e->base+whole;
    e->acc+=e->step;
    return sum<=(uint32_t)INT32_MAX?(int32_t)sum:(int32_t)(sum-UINT32_C(0x80000000))-INT32_MAX-1;
}

/* trunc(a * num / den) for a < 2^32 and 0 <= num <= den < 2^32, den >= 1;
 * the result is at most a. With divu on SH-2, a < 2^31 and den < 2^31 put
 * the quotient strictly inside the divider's range. */
static uint64_t crossing_offset(uint64_t a,uint64_t num,uint64_t den,bool divu) {
#if SH2Z_DIVU_NATIVE
    if(divu && a<UINT64_C(0x80000000) && den<UINT64_C(0x80000000))
        return (uint32_t)sh2z_divu_div64((int64_t)(a*num),(int32_t)den,NULL);
#else
    (void)divu;
#endif
    return a*num/den;
}
static unsigned clip_near(sh2z_view_vertex_t out[6],const sh2z_view_t *v,
    const sh2z_view_vertex_t in[4],project_fn proj,bool divu) {
    if(!view_valid(v) || !out || !in)return 0;
    sh2z_view_vertex_t q[4];
    for(unsigned i=0;i<4;++i)q[i]=in[i];
    const int32_t near_z=v->near_z;
    sh2z_view_vertex_t r[6];
    unsigned n=0;
    for(unsigned i=0;i<4;++i) {
        const sh2z_view_vertex_t *cur=&q[i],*next=&q[(i+1)&3u];
        const bool cur_in=cur->z>=near_z,next_in=next->z>=near_z;
        if(cur_in)r[n++]=*cur;
        if(cur_in!=next_in) {
            const sh2z_view_vertex_t *ins=cur_in?cur:next,*outs=cur_in?next:cur;
            /* 1 <= den < 2^32 and 0 <= num <= den, so each offset is at most
             * |outs - ins| < 2^32 and the product fits 64 unsigned bits. */
            const uint64_t num=(uint64_t)((int64_t)ins->z-near_z);
            const uint64_t den=(uint64_t)((int64_t)ins->z-outs->z);
            int32_t p[2];
            const int32_t ia[2]={ins->x,ins->y},oa[2]={outs->x,outs->y};
            for(unsigned j=0;j<2;++j) {
                const int64_t d=(int64_t)oa[j]-ia[j];
                const uint64_t mag=crossing_offset(d<0?(uint64_t)0-(uint64_t)d:(uint64_t)d,num,den,divu);
                p[j]=(int32_t)(d<0?(int64_t)ia[j]-(int64_t)mag:(int64_t)ia[j]+(int64_t)mag);
            }
            r[n++]=proj(v,p[0],p[1],near_z);
        }
    }
    for(unsigned i=0;i<n;++i)out[i]=r[i];
    return n;
}
static bool project_args_ok(const sh2z_view_vertex_t *out,const sh2z_view_vertex_t *in,size_t count) {
    if(!out || !in || count>SIZE_MAX/sizeof(*out))return false;
    const size_t bytes=count*sizeof(*out);
    if((uintptr_t)out>UINTPTR_MAX-bytes || (uintptr_t)in>UINTPTR_MAX-bytes)return false;
    return out==in || !overlaps(out,bytes,in,bytes);
}

/* ---- view-guard-1: planes and Sutherland-Hodgman passes ----------------- */
enum { GUARD_COORDS=3+SH2Z_VIEW_GUARD_ATTRS };
typedef struct { int32_t c[GUARD_COORDS]; } guard_point_t;   /* x y z attr0..3 */
typedef struct {
    int32_t near_z,far_z,fx,fy;
    int32_t kl,kr,kt,kb;     /* 65536*(left-cx), 65536*(right-cx), 65536*(cy-top), 65536*(cy-bottom) */
} guard_planes_t;

/* Every corner's distance to one plane; returns how many are inside
 * (d >= 0). |terms| <= 2^62 and 2^61, so every d fits int64. The switch is
 * outside the loops so each plane runs a straight loop. */
static unsigned guard_distances(int64_t *d,const guard_point_t *p,unsigned n,
    const guard_planes_t *g,unsigned plane) {
    unsigned inside=0;
    switch(plane) {
    case 0: for(unsigned i=0;i<n;++i){d[i]=(int64_t)p[i].c[2]-g->near_z;inside+=d[i]>=0;} break;
    case 1: for(unsigned i=0;i<n;++i){d[i]=(int64_t)g->far_z-p[i].c[2];inside+=d[i]>=0;} break;
    case 2: for(unsigned i=0;i<n;++i){d[i]=(int64_t)g->fx*p[i].c[0]-(int64_t)g->kl*p[i].c[2];inside+=d[i]>=0;} break;
    case 3: for(unsigned i=0;i<n;++i){d[i]=(int64_t)g->kr*p[i].c[2]-(int64_t)g->fx*p[i].c[0];inside+=d[i]>=0;} break;
    case 4: for(unsigned i=0;i<n;++i){d[i]=(int64_t)g->kt*p[i].c[2]-(int64_t)g->fy*p[i].c[1];inside+=d[i]>=0;} break;
    default: for(unsigned i=0;i<n;++i){d[i]=(int64_t)g->fy*p[i].c[1]-(int64_t)g->kb*p[i].c[2];inside+=d[i]>=0;} break;
    }
    return inside;
}
static void guard_cross(guard_point_t *p,const guard_point_t *in,const guard_point_t *outside,
    uint64_t num,uint64_t den,bool side,unsigned coords,bool divu) {
    uint32_t t=0;
    if(side) {
        /* dn = num >> s, dd = den >> s for the least s with dd < 2^31, using
         * constant shifts only (SH-2 has no barrel shifter): keep dd >= 2^31
         * while removing the excess, then one final shift. */
        uint64_t dn=num,dd=den;
        if(dd>=UINT64_C(0x80000000)) {
            if((dd>>32)>=UINT64_C(0x80000000)){dd>>=32;dn>>=32;}
            if((dd>>16)>=UINT64_C(0x80000000)){dd>>=16;dn>>=16;}
            if((dd>>8)>=UINT64_C(0x80000000)){dd>>=8;dn>>=8;}
            if((dd>>4)>=UINT64_C(0x80000000)){dd>>=4;dn>>=4;}
            if((dd>>2)>=UINT64_C(0x80000000)){dd>>=2;dn>>=2;}
            if((dd>>1)>=UINT64_C(0x80000000)){dd>>=1;dn>>=1;}
            dd>>=1;dn>>=1;
        }
        /* dn <= dd, 1 <= dd < 2^31 */
#if SH2Z_DIVU_NATIVE
        if(divu)t=(uint32_t)sh2z_divu_div64((int64_t)(dn<<30),(int32_t)dd,NULL);
        else
#endif
        t=(uint32_t)((dn<<30)/dd);
    }
#if !SH2Z_DIVU_NATIVE
    (void)divu;
#endif
    for(unsigned i=0;i<coords;++i) {
        const int32_t a=in->c[i],b=outside->c[i];
        const bool negative=b<a;
        const uint64_t mag=negative?(uint64_t)((int64_t)a-b):(uint64_t)((int64_t)b-a);
        const uint64_t off=side?(mag*t)>>30:crossing_offset(mag,num,den,divu);
        p->c[i]=(int32_t)(negative?(int64_t)a-(int64_t)off:(int64_t)a+(int64_t)off);
    }
    for(unsigned i=coords;i<GUARD_COORDS;++i)p->c[i]=0;
}
/* One Sutherland-Hodgman pass for a plane with corners on both sides; d holds
 * each corner's distance. Returns the new count, or -1 past capacity. */
static int guard_pass(guard_point_t *dst,const guard_point_t *src,unsigned n,
    const int64_t *d,unsigned plane,unsigned coords,bool divu) {
    unsigned m=0;
    for(unsigned i=0;i<n;++i) {
        const unsigned j=i+1u==n?0u:i+1u;
        const bool cur_in=d[i]>=0,next_in=d[j]>=0;
        if(cur_in) {
            if(m==SH2Z_VIEW_GUARD_MAX_OUT)return -1;
            dst[m++]=src[i];
        }
        if(cur_in!=next_in) {
            if(m==SH2Z_VIEW_GUARD_MAX_OUT)return -1;
            const unsigned a=cur_in?i:j,b=cur_in?j:i;
            /* d[a] >= 0 > d[b]; den = d[a] - d[b] < 2^64, exact modulo 2^64. */
            const uint64_t num=(uint64_t)d[a],den=(uint64_t)d[a]-(uint64_t)d[b];
            guard_cross(&dst[m++],&src[a],&src[b],num,den,plane>=2,coords,divu);
        }
    }
    return (int)m;
}
static bool guard_edge_ok(int32_t edge,int32_t center) {
    const int32_t delta=edge-center;
    return delta>=-16384 && delta<=16384;
}

#if SH2Z_DIVU_NATIVE
/* Camera block shared with the SH-2 assembly in sh2zam_view.c. */
#if !defined(__BYTE_ORDER__) || __BYTE_ORDER__ != __ORDER_BIG_ENDIAN__
#error "the SH-2 view path writes big-endian words"
#endif
typedef struct {
    int16_t r[9];                 /* z row, x row, y row */
    int16_t cx,cy,left,right,top,bottom,skip;
    int32_t t2,t0,t1,near_z,fx,fy,far_z,code;
    sh2z_view_vertex_t scratch;
} divu_block_t;
_Static_assert(offsetof(divu_block_t,cx)==18,"block cx");
_Static_assert(offsetof(divu_block_t,skip)==30,"block skip");
_Static_assert(offsetof(divu_block_t,t2)==32,"block t2");
_Static_assert(offsetof(divu_block_t,code)==60,"block code");
_Static_assert(offsetof(divu_block_t,scratch)==64,"block scratch");

void sh2zi_view_divu_run(sh2z_view_vertex_t *out,const divu_block_t *block,
    const void *positions,size_t count);
void sh2zi_view_divu_project(sh2z_view_vertex_t *out,const divu_block_t *block,
    const sh2z_view_vertex_t *in,size_t count);
/* view-round-1 forms of the two entries above (sh2zam_view_round.c). */
void sh2zi_view_divu_run_trunc(sh2z_view_vertex_t *out,const divu_block_t *block,
    const void *positions,size_t count);
void sh2zi_view_divu_project_trunc(sh2z_view_vertex_t *out,const divu_block_t *block,
    const sh2z_view_vertex_t *in,size_t count);
static void fill_block(divu_block_t *b,const sh2z_view_t *v,int16_t skip) {
    for(unsigned j=0;j<3;++j) {
        b->r[j]=v->r[2][j];b->r[3+j]=v->r[0][j];b->r[6+j]=v->r[1][j];
    }
    b->cx=v->center_x;b->cy=v->center_y;
    b->left=v->clip[0];b->top=v->clip[1];b->right=v->clip[2];b->bottom=v->clip[3];
    b->skip=skip;
    b->t2=v->t[2];b->t0=v->t[0];b->t1=v->t[1];
    b->near_z=v->near_z;b->fx=v->focal_x;b->fy=v->focal_y;b->far_z=v->far_z;b->code=0;
}
#endif

#endif
