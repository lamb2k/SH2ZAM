/* view-guard-src-1: the source-order guard clip with a Q16 centre, as
 * static functions. Included by sh2zam_view_guard_src.c (whole-pixel
 * centres) and sh2zam_view_guard_src_center.c (view-center-1), so each
 * object carries its own copy and the first compiles exactly as it did
 * before the split.
 *
 * Copyright (c) 2026 SH2ZAM Contributors. MIT License.
 * Inspired by SH4ZAM by Falco Girgis and SH4ZAM Contributors.
 */
#ifndef SH2ZAM_VIEW_GUARD_SRC_CORE_H
#define SH2ZAM_VIEW_GUARD_SRC_CORE_H

#include "sh2zam/view_experimental.h"
#include "sh2zam/divu_experimental.h"

enum { SRC_COORDS=3+SH2Z_VIEW_GUARD_ATTRS };
typedef struct { int32_t c[SRC_COORDS]; } src_point_t;   /* x y z attr0..3 */
typedef struct {
    int32_t near_z,far_z,fx,fy;
    /* Q16 plane offsets: 65536*left - cx, 65536*right - cx,
     * cy - 65536*top, cy - 65536*bottom; each within +/-2^30. */
    int32_t kl,kr,kt,kb;
    int32_t cx,cy;            /* Q16 centre */
} src_planes_t;

static uint32_t src_magnitude32(int32_t a) {
    return a<0?(uint32_t)0-(uint32_t)a:(uint32_t)a;
}
static bool src_row_fits(const int16_t r[3],int32_t t) {
    uint64_t sum=0;
    for(unsigned j=0;j<3;++j)sum+=src_magnitude32(r[j]);
    return sum*UINT64_C(32768)+src_magnitude32(t)<=(uint64_t)INT32_MAX;
}
static bool src_view_valid(const sh2z_view_t *v) {
    if(!v || v->focal_x<=0 || v->focal_y<=0 || v->near_z<=0 || v->far_z<=v->near_z ||
       v->clip[0]>v->clip[2] || v->clip[1]>v->clip[3])return false;
    for(unsigned i=0;i<3;++i)if(!src_row_fits(v->r[i],v->t[i]))return false;
    return true;
}
/* 65536 * edge - centre (Q16) when it lies within +/-2^30. */
static bool src_offset(int32_t *k,int16_t edge,int32_t center,bool negate) {
    int64_t d=(int64_t)edge*65536-center;
    if(negate)d=-d;
    if(d<-INT64_C(0x40000000) || d>INT64_C(0x40000000))return false;
    *k=(int32_t)d;
    return true;
}

/* floor(n / d) for n < 2^63 and 0 < d < 2^31 with a quotient below 2^31. */
static uint32_t src_quotient(uint64_t n,uint32_t d,bool divu) {
#if SH2Z_DIVU_NATIVE
    if(divu)return (uint32_t)sh2z_divu_div64((int64_t)n,(int32_t)d,NULL);
#else
    (void)divu;
#endif
    return (uint32_t)(n/d);
}
/* floor(a * 2^32 / d) for a < d <= 2^31 - 1, in one division with a
 * quotient below 2^31: when 2a >= d, a * 2^32 / d = 2^31 + (2a - d) * 2^31 / d
 * exactly, and 2a - d < d. */
static uint32_t src_fraction_small(uint32_t a,uint32_t d,bool divu) {
    if(a<d-a)return src_quotient((uint64_t)a<<32,d,divu);
    return UINT32_C(0x80000000)+src_quotient((uint64_t)(2u*a-d)<<31,d,divu);
}
/* t = floor(a * 2^32 / den) for 0 <= a < den < 2^64 (so t < 2^32). */
static uint32_t src_fraction(uint64_t a,uint64_t den,bool divu) {
    if(den<=UINT64_C(0x7FFFFFFF))return src_fraction_small((uint32_t)a,(uint32_t)den,divu);
    if(den<(UINT64_C(1)<<59)) {
        /* s = the least shift with (den >> s) <= 2^31 - 2, by constant shifts
         * (SH-2 has no barrel shifter): keep dd above the limit while
         * removing the excess, then one final shift. D = (den >> s) + 1 lies
         * in [2^30, 2^31 - 1] and A = a >> s < D. Then t0 = floor(A*2^32/D)
         * <= t, and t - t0 < 2^32 * (A + D) / (D * (D - 1)) + 1 <= 9.002, so
         * at most nine corrections follow; the remainder stays below
         * 10 * den < 2^63. */
        uint64_t dd=den,aa=a;
        const uint64_t limit=UINT64_C(0x7FFFFFFE);
        if((dd>>16)>limit){dd>>=16;aa>>=16;}
        if((dd>>8)>limit){dd>>=8;aa>>=8;}
        if((dd>>4)>limit){dd>>=4;aa>>=4;}
        if((dd>>2)>limit){dd>>=2;aa>>=2;}
        if((dd>>1)>limit){dd>>=1;aa>>=1;}
        dd>>=1;aa>>=1;
        uint32_t t=src_fraction_small((uint32_t)aa,(uint32_t)dd+1u,divu);
        /* r = a * 2^32 - t * den, modulo 2^64 (its true value is in range). */
        const uint64_t td=(uint64_t)t*(uint32_t)den+((uint64_t)(t*(uint32_t)(den>>32))<<32);
        uint64_t r=((uint64_t)(uint32_t)a<<32)-td;
        while(r>=den){r-=den;++t;}
        return t;
    }
    /* Huge distances (2^59 or more): restoring division, one bit at a time,
     * comparing before doubling so nothing wraps. */
    uint64_t r=a;
    uint32_t t=0;
    for(unsigned i=0;i<32;++i) {
        t<<=1;
        if(r>=den-r){r-=den-r;t|=1u;}
        else r+=r;
    }
    return t;
}
/* The point at fraction t / 2^32 (or the whole step when full) from p
 * towards c, each coordinate truncated towards p. */
static void src_cross(src_point_t *q,const src_point_t *p,const src_point_t *c,
    uint32_t t,bool full,unsigned coords) {
    for(unsigned i=0;i<coords;++i) {
        const int32_t a=p->c[i],b=c->c[i];
        const bool negative=b<a;
        const uint32_t m=negative?(uint32_t)a-(uint32_t)b:(uint32_t)b-(uint32_t)a;
        const uint32_t step=full?m:(uint32_t)(((uint64_t)m*t)>>32);
        const uint32_t s=negative?(uint32_t)a-step:(uint32_t)a+step;
        /* s lies between a and b, so it is an int32 value. */
        q->c[i]=s<=(uint32_t)INT32_MAX?(int32_t)s:(int32_t)(s-UINT32_C(0x80000000))-INT32_MAX-1;
    }
    for(unsigned i=coords;i<SRC_COORDS;++i)q->c[i]=0;
}
/* Whether ceil(b / f) (up) or floor(b / f) fits int32, for f > 0 and
 * |b| < 2^62: with F = f * 2^31, ceil fits iff -F - f < b <= F - f and
 * floor fits iff -F <= b < F. */
static bool src_fits(int64_t b,int32_t f,bool up) {
    const int64_t big=(int64_t)((uint64_t)(uint32_t)f<<31);
    return up?(b<=big-f && b>-big-f):(b<big && b>=-big);
}
/* ceil(b / f) (up) or floor(b / f), for f > 0 and a result known to fit
 * int32 (so the truncated quotient fits as well). */
static int32_t src_round_div(int64_t b,int32_t f,bool up,bool divu) {
    int32_t q;
#if SH2Z_DIVU_NATIVE
    if(divu)q=sh2z_divu_div64(b,f,NULL);
    else
#else
    (void)divu;
#endif
    q=(int32_t)(b/f);
    /* The remainder is below f in magnitude: its low 32 bits are exact. */
    const int32_t r=(int32_t)((uint32_t)(uint64_t)b-(uint32_t)q*(uint32_t)f);
    if(up && r>0)++q;
    if(!up && r<0)--q;
    return q;
}
/* Move a crossing back inside the planes clipped so far (0 .. plane): z into
 * [near_z, far_z], then x to the lattice of left and right, y to the lattice
 * of top and bottom, in that order. Fails when a lattice bound does not fit
 * int32 or when no lattice point lies between two opposite planes. */
static bool src_snap(src_point_t *p,unsigned plane,const src_planes_t *g,bool divu) {
    int32_t z=p->c[2];
    if(z<g->near_z)z=g->near_z;
    if(plane>=1u && z>g->far_z)z=g->far_z;
    p->c[2]=z;
    if(plane>=2u) {
        int32_t x=p->c[0];
        const int64_t bl=(int64_t)g->kl*z;
        if(!src_fits(bl,g->fx,true))return false;
        if((int64_t)g->fx*x<bl)x=src_round_div(bl,g->fx,true,divu);
        if(plane>=3u) {
            const int64_t br=(int64_t)g->kr*z;
            if(!src_fits(br,g->fx,false))return false;
            if((int64_t)g->fx*x>br)x=src_round_div(br,g->fx,false,divu);
            if((int64_t)g->fx*x<bl)return false;
        }
        p->c[0]=x;
    }
    if(plane>=4u) {
        int32_t y=p->c[1];
        const int64_t bt=(int64_t)g->kt*z;
        if(!src_fits(bt,g->fy,false))return false;
        if((int64_t)g->fy*y>bt)y=src_round_div(bt,g->fy,false,divu);
        if(plane>=5u) {
            const int64_t bb=(int64_t)g->kb*z;
            if(!src_fits(bb,g->fy,true))return false;
            if((int64_t)g->fy*y<bb)y=src_round_div(bb,g->fy,true,divu);
            if((int64_t)g->fy*y>bt)return false;
        }
        p->c[1]=y;
    }
    return true;
}
/* Every corner's distance to one plane; returns how many are inside
 * (d >= 0). |fx*x| < 2^62 and |k*z| <= 2^61, so every d fits int64. */
static unsigned src_distances(int64_t *d,const src_point_t *p,unsigned n,
    const src_planes_t *g,unsigned plane) {
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
static uint64_t src_abs64(int64_t a) {
    return a<0?(uint64_t)0-(uint64_t)a:(uint64_t)a;
}
/* One pass for a plane with corners on both sides: walk previous -> current
 * from (last, 0); emit the crossing of a sign change, then the current corner
 * if inside. Returns the new count or a negative error. */
static int src_pass(src_point_t *dst,const src_point_t *src,unsigned n,const int64_t *d,
    unsigned plane,unsigned coords,unsigned max_out,const src_planes_t *g,bool divu) {
    unsigned m=0,prev=n-1u;
    for(unsigned i=0;i<n;++i) {
        const bool cur_in=d[i]>=0;
        if((d[prev]>=0)!=cur_in) {
            if(m==max_out)return SH2Z_VIEW_GUARD_OVERFLOW;
            /* a + b < 2^64: each distance is below 2^63 in magnitude. */
            const uint64_t a=src_abs64(d[prev]),b=src_abs64(d[i]);
            const uint32_t t=b?src_fraction(a,a+b,divu):0u;
            src_cross(&dst[m],&src[prev],&src[i],t,b==0u,coords);
            if(!src_snap(&dst[m],plane,g,divu))return SH2Z_VIEW_GUARD_SNAP;
            ++m;
        }
        if(cur_in) {
            if(m==max_out)return SH2Z_VIEW_GUARD_OVERFLOW;
            dst[m++]=src[i];
        }
        prev=i;
    }
    return (int)m;
}
/* trunc(W / 65536) for |W| < 2^31. */
static int16_t src_pixel(int32_t w) {
    return (int16_t)(w>=0?(int32_t)((uint32_t)w>>16):-(int32_t)(((uint32_t)0-(uint32_t)w)>>16));
}
/* The guard clip with a Q16 centre (cx, cy). */
static int clip_guard_src(sh2z_view_guard_vertex_t *out,const sh2z_view_t *v,int32_t cx,int32_t cy,
    const int16_t guard[4],const sh2z_view_guard_vertex_t *in,unsigned count,unsigned attrs,
    unsigned max_out,bool divu) {
    if(!src_view_valid(v) || !out || !guard || !in || count<3u || count>SH2Z_VIEW_GUARD_MAX_IN ||
       attrs>SH2Z_VIEW_GUARD_ATTRS || max_out<3u || max_out>SH2Z_VIEW_GUARD_MAX_OUT)return SH2Z_VIEW_GUARD_INVALID;
    const int16_t gl=guard[0],gt=guard[1],gr=guard[2],gb=guard[3];
    src_planes_t g;
    if(gl>gr || gt>gb || !src_offset(&g.kl,gl,cx,false) || !src_offset(&g.kr,gr,cx,false) ||
       !src_offset(&g.kt,gt,cy,true) || !src_offset(&g.kb,gb,cy,true))return SH2Z_VIEW_GUARD_INVALID;
    g.near_z=v->near_z;g.far_z=v->far_z;g.fx=v->focal_x;g.fy=v->focal_y;g.cx=cx;g.cy=cy;
    const unsigned coords=3u+attrs;
    src_point_t buf[2][SH2Z_VIEW_GUARD_MAX_OUT];
    for(unsigned k=0;k<count;++k) {
        buf[0][k].c[0]=in[k].v.x;buf[0][k].c[1]=in[k].v.y;buf[0][k].c[2]=in[k].v.z;
        for(unsigned i=0;i<SH2Z_VIEW_GUARD_ATTRS;++i)buf[0][k].c[3+i]=i<attrs?in[k].attr[i]:0;
    }
    int n=(int)count;
    unsigned cur=0;
    for(unsigned plane=0;plane<6u;++plane) {
        /* A pass with every corner inside is the identity (it still counts
         * against max_out, which only an input longer than max_out can
         * exceed), and one with none inside empties the polygon: neither
         * needs the copy. */
        int64_t d[SH2Z_VIEW_GUARD_MAX_OUT];
        const unsigned inside=src_distances(d,buf[cur],(unsigned)n,&g,plane);
        if(inside==(unsigned)n) {
            if((unsigned)n>max_out)return SH2Z_VIEW_GUARD_OVERFLOW;
            continue;
        }
        if(!inside){n=0;break;}
        n=src_pass(buf[cur^1u],buf[cur],(unsigned)n,d,plane,coords,max_out,&g,divu);
        if(n<0)return n;
        cur^=1u;
    }
    /* Every point now satisfies all six planes, so with q = trunc(f*a/z) the
     * Q16 positions cx + qx and cy - qy lie in [65536*left, 65536*right] and
     * [65536*top, 65536*bottom]: the quotients fit the divider, the sums fit
     * int32, and the truncated pixels lie inside the guard rectangle. */
    for(int k=0;k<n;++k) {
        const src_point_t *p=&buf[cur][k];
        const int32_t x=p->c[0],y=p->c[1],z=p->c[2];
        sh2z_view_vertex_t o={x,y,z,0,0,0,{0,0,0}};
#if SH2Z_DIVU_NATIVE
        if(divu) {
            /* x's division runs while y's numerator is formed, y's while
             * sx is finished. */
            const uint64_t nx=(uint64_t)((int64_t)g.fx*x);
            sh2z_divu_t t;
            sh2z_divu_start64(&t,(int32_t)(uint32_t)(nx>>32),(uint32_t)nx,z);
            const uint64_t ny=(uint64_t)((int64_t)g.fy*y);
            const int32_t qx=sh2z_divu_finish(&t,NULL);
            sh2z_divu_start64(&t,(int32_t)(uint32_t)(ny>>32),(uint32_t)ny,z);
            o.sx=src_pixel(cx+qx);
            o.sy=src_pixel(cy-sh2z_divu_finish(&t,NULL));
        } else
#endif
        {
            o.sx=src_pixel(cx+(int32_t)((int64_t)g.fx*x/z));
            o.sy=src_pixel(cy-(int32_t)((int64_t)g.fy*y/z));
        }
        o.outcode=(uint8_t)((o.sx<v->clip[0]?SH2Z_OUT_LEFT:0u)|(o.sx>v->clip[2]?SH2Z_OUT_RIGHT:0u)|
            (o.sy<v->clip[1]?SH2Z_OUT_TOP:0u)|(o.sy>v->clip[3]?SH2Z_OUT_BOTTOM:0u));
        out[k].v=o;
        for(unsigned i=0;i<SH2Z_VIEW_GUARD_ATTRS;++i)out[k].attr[i]=p->c[3+i];
    }
    return n;
}

#endif
