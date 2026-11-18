/* Independent oracle for the fixed-unit-1 kernels (unit30_experimental.h).
 * Written from the header text: exact 128-bit integer arithmetic for the
 * rounded products and the integer square root, float64 (and binary32 for the
 * reference report) for the approximate kernels. Shares no code with the
 * library. Used by tests/test_unit30.c and the SH-2 model harness.
 * MIT; SH2ZAM Contributors. */
#ifndef SH2ZAM_UNIT30_ORACLE_H
#define SH2ZAM_UNIT30_ORACLE_H

#include "sh2zam/unit30_experimental.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

__extension__ typedef __int128 uo_i128;

/* floor(v / 2^30) for any 128-bit v (C division truncates toward zero). */
static inline uo_i128 uo_floor30(uo_i128 v) {
    const uo_i128 d=(uo_i128)1<<30;
    uo_i128 q=v/d;
    if(v%d!=0 && v<0) q-=1;
    return q;
}
static inline int32_t uo_sat(uo_i128 v) {
    if(v>INT32_MAX) return INT32_MAX;
    if(v<INT32_MIN) return INT32_MIN;
    return (int32_t)v;
}
/* The header's rule: floor((exact + 2^29) / 2^30), saturated. */
static inline int32_t uo_round30(uo_i128 exact) { return uo_sat(uo_floor30(exact+((uo_i128)1<<29))); }
static inline bool uo_saturates(uo_i128 exact) {
    const uo_i128 r=uo_floor30(exact+((uo_i128)1<<29));
    return r>INT32_MAX || r<INT32_MIN;
}

static inline int32_t uo_mul(int32_t a,int32_t b) { return uo_round30((uo_i128)a*b); }
static inline uo_i128 uo_dot_exact(const int32_t a[3],const int32_t b[3]) {
    return (uo_i128)a[0]*b[0]+(uo_i128)a[1]*b[1]+(uo_i128)a[2]*b[2];
}
static inline int32_t uo_dot3(const int32_t a[3],const int32_t b[3]) { return uo_round30(uo_dot_exact(a,b)); }
static inline void uo_cross_exact(uo_i128 out[3],const int32_t a[3],const int32_t b[3]) {
    out[0]=(uo_i128)a[1]*b[2]-(uo_i128)a[2]*b[1];
    out[1]=(uo_i128)a[2]*b[0]-(uo_i128)a[0]*b[2];
    out[2]=(uo_i128)a[0]*b[1]-(uo_i128)a[1]*b[0];
}
static inline void uo_cross(int32_t out[3],const int32_t a[3],const int32_t b[3]) {
    uo_i128 c[3]; uo_cross_exact(c,a,b);
    for(int k=0;k<3;k++) out[k]=uo_round30(c[k]);
}

/* floor(sqrt(n)) for n < 2^126, by Newton on 128-bit integers. */
static inline uo_i128 uo_isqrt128(uo_i128 n) {
    if(n<2) return n;
    uo_i128 x=(uo_i128)sqrt((double)n);           /* close start, then fix */
    while(x*x>n) x--;
    while((x+1)*(x+1)<=n) x++;
    return x;
}
/* The header's rounded square root: floor(sqrt(s) + 1/2), saturated. */
static inline uint32_t uo_isqrt_round(uint64_t s) {
    const uo_i128 r=uo_isqrt128((uo_i128)s);
    const uo_i128 v=((uo_i128)s-r*r>r) ? r+1 : r;   /* sqrt(s) >= r + 1/2 <=> s - r^2 > r */
    return v>(uo_i128)UINT32_MAX ? UINT32_MAX : (uint32_t)v;
}

/* Exact real values (float64) for the approximate kernels. */
static inline double uo_rsqrt_ref(uint64_t s,int scale) {      /* 2^(30+scale/2)/sqrt(s) */
    return ldexp(1.0/sqrt((double)s),30)*sqrt(ldexp(1.0,scale));
}
static inline double uo_rsqrt_q31_ref(uint32_t m) { return ldexp(1.0,47)/sqrt((double)m); }
/* 2^30 v[k] / |v|. The squared length is formed exactly in 128 bits. */
static inline void uo_unit_ref(double out[3],const int32_t v[3]) {
    const uo_i128 s=(uo_i128)v[0]*v[0]+(uo_i128)v[1]*v[1]+(uo_i128)v[2]*v[2];
    const double len=sqrt((double)s);
    for(int k=0;k<3;k++) out[k]=ldexp((double)v[k]/len,30);
}

/* The two orthonormalisations in float64 on real values (x and y rows as
 * given: the forward variant's x row and the up variant's y row may be any
 * scale; the other rows are Q2.30). Same operation order as the header.
 * Returns 0 or -1; out receives the rows in Q2.30 units (2^30 per unit). */
typedef struct { double x[3],y[3],z[3]; } uo_basis64_t;
static inline void uo_cross64(double o[3],const double a[3],const double b[3]) {
    const double c0=a[1]*b[2]-a[2]*b[1],c1=a[2]*b[0]-a[0]*b[2],c2=a[0]*b[1]-a[1]*b[0];
    o[0]=c0;o[1]=c1;o[2]=c2;
}
static inline bool uo_norm64(double v[3]) {
    const double s=v[0]*v[0]+v[1]*v[1]+v[2]*v[2];
    if(s==0.0) return false;
    const double r=1.0/sqrt(s);
    v[0]*=r;v[1]*=r;v[2]*=r;
    return true;
}
static inline void uo_load64(uo_basis64_t *o,const sh2z_basis30_t *b) {
    for(int k=0;k<3;k++){o->x[k]=ldexp((double)b->x[k],-30);o->y[k]=ldexp((double)b->y[k],-30);o->z[k]=ldexp((double)b->z[k],-30);}
}
static inline int uo_forward64(uo_basis64_t *b) {
    if(!uo_norm64(b->x)) return -1;
    uo_cross64(b->z,b->y,b->x);
    if(!uo_norm64(b->z)) return -1;
    uo_cross64(b->y,b->x,b->z);
    return 0;
}
static inline int uo_up64(uo_basis64_t *b) {
    if(!uo_norm64(b->y)) return -1;
    uo_cross64(b->z,b->y,b->x);
    if(!uo_norm64(b->z)) return -1;
    uo_cross64(b->x,b->z,b->y);
    return 0;
}

/* The binary32 reference of the same algorithm (x = 1/sqrtf of the float
 * sum of squares, then products), for the deviation report only. */
typedef struct { float x[3],y[3],z[3]; } uo_basis32_t;
static inline void uo_cross32(float o[3],const float a[3],const float b[3]) {
    const float c0=a[1]*b[2]-a[2]*b[1],c1=a[2]*b[0]-a[0]*b[2],c2=a[0]*b[1]-a[1]*b[0];
    o[0]=c0;o[1]=c1;o[2]=c2;
}
static inline bool uo_norm32(float v[3]) {
    float s=v[0]*v[0];s+=v[1]*v[1];s+=v[2]*v[2];
    if(s==0.0f) return false;
    const float r=1.0f/sqrtf(s);
    v[0]*=r;v[1]*=r;v[2]*=r;
    return true;
}
static inline void uo_load32(uo_basis32_t *o,const sh2z_basis30_t *b) {
    for(int k=0;k<3;k++){o->x[k]=ldexpf((float)b->x[k],-30);o->y[k]=ldexpf((float)b->y[k],-30);o->z[k]=ldexpf((float)b->z[k],-30);}
}
static inline int uo_forward32(uo_basis32_t *b) {
    if(!uo_norm32(b->x)) return -1;
    uo_cross32(b->z,b->y,b->x);
    if(!uo_norm32(b->z)) return -1;
    uo_cross32(b->y,b->x,b->z);
    return 0;
}
static inline int uo_up32(uo_basis32_t *b) {
    if(!uo_norm32(b->y)) return -1;
    uo_cross32(b->z,b->y,b->x);
    if(!uo_norm32(b->z)) return -1;
    uo_cross32(b->x,b->z,b->y);
    return 0;
}

#endif
