/* SH-2 bridge for the executed-instruction trig12 model (contract tag
 * trig12-1). MIT; Copyright (c) 2026 SH2ZAM Contributors. Every entry point
 * is called directly by the model harness with the SH-2 calling convention;
 * the harness writes the input arrays before each call and reads the
 * outputs after it; nothing here uses libc.
 *
 * v_*  verification entries: every library form, outputs to arrays.
 * c_*  cost entries: one loop over n inputs; the harness takes the slope
 *      between n = 1 and n = N, which removes the call cost.
 * Controls (not library code):
 *   c_copy           the loop alone (load, store, count)
 *   c_float_lookup   two binary32 table loads, the float-table sin/cos idiom
 *   c_float_to_u30   the same plus conversion to Q2.30 (libgcc soft-float)
 *   c_float_tan      binary32 sin / cos (libgcc __divsf3)
 *   c_float_div      binary32 num / den (libgcc __divsf3)
 *   c_libgcc_q16     clamp(((int64_t)num << 16) / den) (libgcc __divdi3) */
#include "sh2zam/divu_experimental.h"
#include "sh2zam/trig12_experimental.h"

#define NOINLINE __attribute__((noinline))
#define N 4096u

uint32_t angles[N];
int32_t num[N],den[N];
int32_t out_s[N],out_c[N],out_t[N],out_a[N],out_b[N];
int32_t out_q[8][N];
uint8_t out_o[4][N];
float fsin[4096],fnum[N],fden[N],out_fs[N],out_fc[N];

/* ---- verification ---- */
NOINLINE uint32_t v_trig(uint32_t n) {
    for(uint32_t i=0;i<n;i++) {
        sh2z_sincos12_u30(angles[i],&out_s[i],&out_c[i]);
        out_a[i]=sh2z_sin12_u30(angles[i]);
        out_b[i]=sh2z_cos12_u30(angles[i]);
        out_t[i]=sh2z_tan12_q16(angles[i]);
    }
    return n;
}
NOINLINE uint32_t v_divq(uint32_t n) {
    for(uint32_t i=0;i<n;i++) {
        const int32_t a=num[i],b=den[i];
        bool o;
        sh2z_divu_t t;
        sh2z_divu_q_start(&t,a,b,16u);out_q[0][i]=sh2z_divu_finish(&t,&o);out_o[0][i]=o;
        sh2z_divu_q_start(&t,a,b,30u);out_q[1][i]=sh2z_divu_finish(&t,&o);out_o[1][i]=o;
        sh2z_divu_q_start(&t,a,b,0u);out_q[2][i]=sh2z_divu_finish(&t,&o);out_o[2][i]=o;
        sh2z_divu_q_start(&t,a,b,31u);out_q[3][i]=sh2z_divu_finish(&t,&o);out_o[3][i]=o;
        out_q[4][i]=sh2z_divu_q16(a,b);
        out_q[5][i]=sh2z_divu_q30(a,b);
        out_q[6][i]=sh2z_recip_q16(b);
        out_q[7][i]=sh2z_recip_q30(b);
    }
    return n;
}

/* ---- cost: trig ---- */
NOINLINE uint32_t c_copy(uint32_t n) {
    for(uint32_t i=0;i<n;i++)out_s[i]=(int32_t)angles[i];
    return n;
}
NOINLINE uint32_t c_sincos12(uint32_t n) {
    for(uint32_t i=0;i<n;i++)sh2z_sincos12_u30(angles[i],&out_s[i],&out_c[i]);
    return n;
}
NOINLINE uint32_t c_sin12(uint32_t n) {
    for(uint32_t i=0;i<n;i++)out_s[i]=sh2z_sin12_u30(angles[i]);
    return n;
}
NOINLINE uint32_t c_float_lookup(uint32_t n) {
    for(uint32_t i=0;i<n;i++) {
        out_fs[i]=fsin[angles[i]&0xFFFu];
        out_fc[i]=fsin[(angles[i]+0x400u)&0xFFFu];
    }
    return n;
}
NOINLINE uint32_t c_float_to_u30(uint32_t n) {
    for(uint32_t i=0;i<n;i++) {
        out_s[i]=(int32_t)(fsin[angles[i]&0xFFFu]*1073741824.0f);
        out_c[i]=(int32_t)(fsin[(angles[i]+0x400u)&0xFFFu]*1073741824.0f);
    }
    return n;
}
/* The existing 16-bit-angle Q16.16 call, for scale (a different contract). */
NOINLINE uint32_t c_sh2z_sincos(uint32_t n) {
    for(uint32_t i=0;i<n;i++)sh2z_sincos((sh2z_angle_t)(angles[i]<<4),&out_s[i],&out_c[i]);
    return n;
}
NOINLINE uint32_t c_tan12(uint32_t n) {
    for(uint32_t i=0;i<n;i++)out_t[i]=sh2z_tan12_q16(angles[i]);
    return n;
}
NOINLINE uint32_t c_float_tan(uint32_t n) {
    for(uint32_t i=0;i<n;i++)out_fs[i]=fsin[angles[i]&0xFFFu]/fsin[(angles[i]+0x400u)&0xFFFu];
    return n;
}
NOINLINE uint32_t c_sh2z_tan(uint32_t n) {
    for(uint32_t i=0;i<n;i++)out_t[i]=sh2z_tan((sh2z_angle_t)(angles[i]<<4));
    return n;
}

/* ---- cost: quotients ---- */
NOINLINE uint32_t c_qcopy(uint32_t n) {
    for(uint32_t i=0;i<n;i++)out_q[0][i]=num[i]^den[i];
    return n;
}
NOINLINE uint32_t c_q16(uint32_t n) {
    for(uint32_t i=0;i<n;i++)out_q[0][i]=sh2z_divu_q16(num[i],den[i]);
    return n;
}
NOINLINE uint32_t c_q30(uint32_t n) {
    for(uint32_t i=0;i<n;i++)out_q[0][i]=sh2z_divu_q30(num[i],den[i]);
    return n;
}
NOINLINE uint32_t c_recip30(uint32_t n) {
    for(uint32_t i=0;i<n;i++)out_q[0][i]=sh2z_recip_q30(den[i]);
    return n;
}
NOINLINE uint32_t c_sh2z_div(uint32_t n) {
    for(uint32_t i=0;i<n;i++)out_q[0][i]=sh2z_div(num[i],den[i]);
    return n;
}
NOINLINE uint32_t c_libgcc_q16(uint32_t n) {
    for(uint32_t i=0;i<n;i++) {
        if(!den[i]){out_q[0][i]=num[i]<0?INT32_MIN:INT32_MAX;continue;}
        const int64_t q=((int64_t)num[i]*65536)/den[i];
        out_q[0][i]=q>INT32_MAX?INT32_MAX:q<INT32_MIN?INT32_MIN:(int32_t)q;
    }
    return n;
}
NOINLINE uint32_t c_float_div(uint32_t n) {
    for(uint32_t i=0;i<n;i++)out_fs[i]=fnum[i]/fden[i];
    return n;
}
/* A quotient and a sincos per element: blocking, then overlapped (the
 * sincos runs while the divider works). */
NOINLINE uint32_t c_q16_then_sincos(uint32_t n) {
    for(uint32_t i=0;i<n;i++) {
        out_q[0][i]=sh2z_divu_q16(num[i],den[i]);
        sh2z_sincos12_u30(angles[i],&out_s[i],&out_c[i]);
    }
    return n;
}
NOINLINE uint32_t c_q16_overlap_sincos(uint32_t n) {
    for(uint32_t i=0;i<n;i++) {
        sh2z_divu_t t;
        sh2z_divu_q_start(&t,num[i],den[i],16u);
        sh2z_sincos12_u30(angles[i],&out_s[i],&out_c[i]);
        out_q[0][i]=sh2z_divu_finish(&t,NULL);
    }
    return n;
}
