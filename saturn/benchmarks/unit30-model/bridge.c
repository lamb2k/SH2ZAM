/* SH-2 bridge for the executed-instruction model of fixed-unit-1
 * (unit30_experimental.h). MIT; Copyright (c) 2026 SH2ZAM Contributors.
 * Every entry point is called directly by the model harness with the SH-2
 * calling convention and works on the arrays below, which the harness writes
 * before each call; nothing here uses libc.
 *
 *   fast_*   the library (inline K1 operations, out-of-line K2/K3)
 *   port_*   the portable C form compiled for SH-2, where the library uses
 *            assembly (the MAC.L dot product and the normalisation)
 *   float_*  the same algorithm in binary32 C, compiled to libgcc soft-float
 *            calls (__mulsf3, __addsf3, __subsf3, __divsf3) with the
 *            correctly rounded integer square root below
 *   q16_*    the existing out-of-line SH2ZAM Q16.16 calls (sh2z_mul,
 *            sh2z_vec3_dot, sh2z_vec3_cross, sh2z_rsqrt, sh2z_vec3_normalize):
 *            a straight translation of the float code to today's library */
#include "sh2zam/unit30_experimental.h"

#define NOINLINE __attribute__((noinline))
#define N 128u
#define NB 32u

int32_t ia[N],ib[N],iout[N];
int32_t va[N][3],vb[N][3],vout[N][3];
uint32_t mm[N],uout[N];
uint32_t ss[N][2];                     /* uint64_t as two big-endian words */
int32_t sc[N];
sh2z_basis30_t bout[NB];
int32_t brc[N];
float fa[N],fb[N],fo[N];
float fva[N][3],fvb[N][3],fvo[N][3];
float fbase[NB][9];
sh2z_vec3_t qa[N],qb[N],qo[N];
sh2z_vec3_t qbase[NB][3];

/* ---- K1 ---- */
NOINLINE uint32_t fast_mul(uint32_t n) { for(uint32_t i=0;i<n;i++) iout[i]=sh2z_u30_mul(ia[i],ib[i]); return n; }
NOINLINE uint32_t fast_q12(uint32_t n) { for(uint32_t i=0;i<n;i++) iout[i]=sh2z_q12_mul_u30(ia[i],ib[i]); return n; }
NOINLINE uint32_t float_mul(uint32_t n) { for(uint32_t i=0;i<n;i++) fo[i]=fa[i]*fb[i]; return n; }
NOINLINE uint32_t q16_mul(uint32_t n) { for(uint32_t i=0;i<n;i++) iout[i]=sh2z_mul(ia[i],ib[i]); return n; }
NOINLINE uint32_t fast_dot(uint32_t n) { for(uint32_t i=0;i<n;i++) iout[i]=sh2z_u30_dot3(va[i],vb[i]); return n; }
NOINLINE uint32_t port_dot(uint32_t n) { for(uint32_t i=0;i<n;i++) iout[i]=sh2zi_u30_dot3_c(va[i],vb[i]); return n; }
NOINLINE uint32_t float_dot(uint32_t n) {
    for(uint32_t i=0;i<n;i++) fo[i]=fva[i][0]*fvb[i][0]+fva[i][1]*fvb[i][1]+fva[i][2]*fvb[i][2];
    return n;
}
NOINLINE uint32_t q16_dot(uint32_t n) { for(uint32_t i=0;i<n;i++) iout[i]=sh2z_vec3_dot(qa[i],qb[i]); return n; }
NOINLINE uint32_t fast_cross(uint32_t n) { for(uint32_t i=0;i<n;i++) sh2z_u30_cross(vout[i],va[i],vb[i]); return n; }
NOINLINE uint32_t float_cross(uint32_t n) {
    for(uint32_t i=0;i<n;i++) {
        const float *a=fva[i],*b=fvb[i];
        fvo[i][0]=a[1]*b[2]-a[2]*b[1]; fvo[i][1]=a[2]*b[0]-a[0]*b[2]; fvo[i][2]=a[0]*b[1]-a[1]*b[0];
    }
    return n;
}
NOINLINE uint32_t q16_cross(uint32_t n) { for(uint32_t i=0;i<n;i++) qo[i]=sh2z_vec3_cross(qa[i],qb[i]); return n; }

/* ---- binary32 square root: fdlibm e_sqrtf.c bit by bit, round to nearest ---- */
typedef union { float f; int32_t i; } fbits_t;
static float plain_sqrtf(float x) {
    fbits_t u; u.f=x;
    int32_t ix=u.i,s,q,m,t,i; uint32_t r;
    if((ix&0x7f800000)==0x7f800000) return x*x+x;
    if(ix<=0) { if((ix&0x7fffffff)==0) return x; return (x-x)/(x-x); }
    m=ix>>23;
    if(m==0) { for(i=0;(ix&0x00800000)==0;i++) ix<<=1; m-=i-1; }
    m-=127; ix=(ix&0x007fffff)|0x00800000;
    if(m&1) ix+=ix;
    m>>=1;
    ix+=ix; q=s=0; r=0x01000000;
    while(r!=0) { t=s+(int32_t)r; if(t<=ix) { s=t+(int32_t)r; ix-=t; q+=(int32_t)r; } ix+=ix; r>>=1; }
    if(ix!=0) q+=(q&1);
    ix=(q>>1)+0x3f000000; ix+=(int32_t)((uint32_t)m<<23);
    u.i=ix; return u.f;
}

/* ---- K2 ---- */
NOINLINE uint32_t fast_q31(uint32_t n) { for(uint32_t i=0;i<n;i++) uout[i]=sh2zi_u30_rsqrt_q31(mm[i]); return n; }
NOINLINE uint32_t fast_rsqrt64(uint32_t n) {
    for(uint32_t i=0;i<n;i++) iout[i]=sh2z_rsqrt64_u30((uint64_t)ss[i][0]<<32|ss[i][1],sc[i]);
    return n;
}
NOINLINE uint32_t float_rsqrt(uint32_t n) { for(uint32_t i=0;i<n;i++) fo[i]=1.0f/plain_sqrtf(fa[i]); return n; }
NOINLINE uint32_t q16_rsqrt(uint32_t n) { for(uint32_t i=0;i<n;i++) iout[i]=sh2z_rsqrt(ia[i]); return n; }
NOINLINE uint32_t fast_norm(uint32_t n) { for(uint32_t i=0;i<n;i++) brc[i]=sh2z_vec3_normalize_u30(vout[i],va[i]); return n; }
NOINLINE uint32_t fast_norm_inplace(uint32_t n) { for(uint32_t i=0;i<n;i++) brc[i]=sh2z_vec3_normalize_u30(va[i],va[i]); return n; }
NOINLINE uint32_t port_norm(uint32_t n) { for(uint32_t i=0;i<n;i++) brc[i]=sh2zi_u30_normalize_c(vout[i],va[i]); return n; }
static int float_norm3(float v[3]) {
    float s=v[0]*v[0]; s+=v[1]*v[1]; s+=v[2]*v[2];
    if(s==0.0f) return -1;
    const float r=1.0f/plain_sqrtf(s);
    v[0]*=r; v[1]*=r; v[2]*=r;
    return 0;
}
NOINLINE uint32_t float_norm(uint32_t n) {
    for(uint32_t i=0;i<n;i++) { fvo[i][0]=fva[i][0]; fvo[i][1]=fva[i][1]; fvo[i][2]=fva[i][2]; brc[i]=float_norm3(fvo[i]); }
    return n;
}
NOINLINE uint32_t q16_norm(uint32_t n) { for(uint32_t i=0;i<n;i++) qo[i]=sh2z_vec3_normalize(qa[i]); return n; }
NOINLINE uint32_t fast_isqrt(uint32_t n) {
    for(uint32_t i=0;i<n;i++) uout[i]=sh2z_isqrt64_q12((uint64_t)ss[i][0]<<32|ss[i][1]);
    return n;
}
NOINLINE uint32_t float_sqrt(uint32_t n) { for(uint32_t i=0;i<n;i++) fo[i]=plain_sqrtf(fa[i]); return n; }

/* ---- K3, in place on bout[] / fbase[] / qbase[] ---- */
NOINLINE uint32_t fast_forward(uint32_t n) { for(uint32_t i=0;i<n;i++) brc[i]=sh2z_basis30_orthonormalize_forward(&bout[i]); return n; }
NOINLINE uint32_t fast_up(uint32_t n) { for(uint32_t i=0;i<n;i++) brc[i]=sh2z_basis30_orthonormalize_up(&bout[i]); return n; }
static void float_cross3(float o[3],const float a[3],const float b[3]) {
    const float c0=a[1]*b[2]-a[2]*b[1],c1=a[2]*b[0]-a[0]*b[2],c2=a[0]*b[1]-a[1]*b[0];
    o[0]=c0; o[1]=c1; o[2]=c2;
}
NOINLINE uint32_t float_forward(uint32_t n) {
    for(uint32_t i=0;i<n;i++) {
        float *x=fbase[i],*y=fbase[i]+3,*z=fbase[i]+6;
        brc[i]=-1;
        if(float_norm3(x)) continue;
        float_cross3(z,y,x);
        if(float_norm3(z)) continue;
        float_cross3(y,x,z);
        brc[i]=0;
    }
    return n;
}
NOINLINE uint32_t float_up(uint32_t n) {
    for(uint32_t i=0;i<n;i++) {
        float *x=fbase[i],*y=fbase[i]+3,*z=fbase[i]+6;
        brc[i]=-1;
        if(float_norm3(y)) continue;
        float_cross3(z,y,x);
        if(float_norm3(z)) continue;
        float_cross3(x,z,y);
        brc[i]=0;
    }
    return n;
}
static int q16_zero(sh2z_vec3_t v) { return !v.x&&!v.y&&!v.z; }
NOINLINE uint32_t q16_forward(uint32_t n) {
    for(uint32_t i=0;i<n;i++) {
        sh2z_vec3_t *b=qbase[i];
        brc[i]=-1;
        if(q16_zero(b[0])) continue;
        b[0]=sh2z_vec3_normalize(b[0]);
        b[2]=sh2z_vec3_cross(b[1],b[0]);
        if(q16_zero(b[2])) continue;
        b[2]=sh2z_vec3_normalize(b[2]);
        b[1]=sh2z_vec3_cross(b[0],b[2]);
        brc[i]=0;
    }
    return n;
}
