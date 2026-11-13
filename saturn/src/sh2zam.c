/* SH2ZAM: independent C11 fixed-point backend for the Sega Saturn's SH-2 CPUs.
 * Copyright (c) 2026 SH2ZAM Contributors. MIT; see root LICENSE.
 */
#include "sh2zam/sh2zam.h"
#include <limits.h>
#include <stdint.h>

static sh2z_fx_t clamp64(int64_t n) {
    if (n > INT32_MAX) return INT32_MAX;
    if (n < INT32_MIN) return INT32_MIN;
    return (sh2z_fx_t)n;
}

sh2z_fx_t sh2z_from_int(int32_t n) {
    if (n >= 32768) return INT32_MAX;
    if (n < -32768) return INT32_MIN;
    return n * INT32_C(65536);
}
int32_t sh2z_to_int(sh2z_fx_t n) { return n / INT32_C(65536); }

sh2z_fx_t sh2z_add(sh2z_fx_t a, sh2z_fx_t b) {
    if (b > 0 && a > INT32_MAX - b) return INT32_MAX;
    if (b < 0 && a < INT32_MIN - b) return INT32_MIN;
    return a + b;
}
sh2z_fx_t sh2z_sub(sh2z_fx_t a, sh2z_fx_t b) {
    if (b < 0 && a > INT32_MAX + b) return INT32_MAX;
    if (b > 0 && a < INT32_MIN + b) return INT32_MIN;
    return a - b;
}

sh2z_fx_t sh2z_mul(sh2z_fx_t a, sh2z_fx_t b) {
#if defined(__sh__) && defined(__SH2__) && !defined(SH2Z_FORCE_PORTABLE)
    /* 32x32->64 signed SH-2 multiply; MACH/MACL are per-CPU registers.
     * Q16.16 extraction is the middle 32 bits of the product. */
    int32_t hi;
    uint32_t lo;
    __asm__ volatile(
        "dmuls.l %[a], %[b]\n\t"
        "sts mach, %[hi]\n\t"
        "sts macl, %[lo]"
        : [hi] "=&r"(hi), [lo] "=&r"(lo)
        : [a] "r"(a), [b] "r"(b)
        : "mach", "macl");
    if (hi > 32767) return INT32_MAX;
    if (hi < -32768) return INT32_MIN;
    return (int32_t)(((uint32_t)hi << 16) | (lo >> 16));
#else
    const int64_t product = (int64_t)a * (int64_t)b;
    /* Explicit mathematical floor, including negative fractional products.
     * Matches SH-2's signed high/middle-word extraction. */
    const int64_t q = (product >= 0)
        ? (product / INT64_C(65536))
        : -(((-product) + INT64_C(65535)) / INT64_C(65536));
    return clamp64(q);
#endif
}

sh2z_fx_t sh2z_div(sh2z_fx_t a, sh2z_fx_t b) {
    /* Portable, correct baseline: replace with DIVU after console validation.
     * Hardware DIVU is not transparently reentrant across interrupt handlers. */
    if (b == 0) return (a > 0) ? INT32_MAX : ((a < 0) ? INT32_MIN : 0);
    return clamp64(((int64_t)a * INT64_C(65536)) / b);
}

sh2z_fx_t sh2z_sqrt(sh2z_fx_t value) {
    if (value <= 0) return 0;
    uint64_t op = (uint64_t)(uint32_t)value << 16;
    uint64_t res = 0;
    uint64_t bit = UINT64_C(1) << 62;
    while (bit > op) bit >>= 2;
    while (bit) {
        if (op >= res + bit) {
            op -= res + bit;
            res = (res >> 1) + bit;
        } else {
            res >>= 1;
        }
        bit >>= 2;
    }
    return (sh2z_fx_t)res;
}

sh2z_fx_t sh2z_vec2_dot(sh2z_vec2_t a, sh2z_vec2_t b) {
    return sh2z_add(sh2z_mul(a.x, b.x), sh2z_mul(a.y, b.y));
}
sh2z_fx_t sh2z_vec3_dot(sh2z_vec3_t a, sh2z_vec3_t b) {
    return sh2z_add(sh2z_vec2_dot((sh2z_vec2_t){a.x,a.y}, (sh2z_vec2_t){b.x,b.y}), sh2z_mul(a.z,b.z));
}
sh2z_fx_t sh2z_vec4_dot(sh2z_vec4_t a, sh2z_vec4_t b) {
    return sh2z_add(sh2z_vec3_dot((sh2z_vec3_t){a.x,a.y,a.z}, (sh2z_vec3_t){b.x,b.y,b.z}), sh2z_mul(a.w,b.w));
}
sh2z_vec3_t sh2z_vec3_add(sh2z_vec3_t a, sh2z_vec3_t b) {
    return (sh2z_vec3_t){sh2z_add(a.x,b.x), sh2z_add(a.y,b.y), sh2z_add(a.z,b.z)};
}
sh2z_vec3_t sh2z_vec3_scale(sh2z_vec3_t a, sh2z_fx_t s) {
    return (sh2z_vec3_t){sh2z_mul(a.x,s), sh2z_mul(a.y,s), sh2z_mul(a.z,s)};
}
sh2z_vec3_t sh2z_vec3_cross(sh2z_vec3_t a, sh2z_vec3_t b) {
    return (sh2z_vec3_t){
        sh2z_sub(sh2z_mul(a.y,b.z),sh2z_mul(a.z,b.y)),
        sh2z_sub(sh2z_mul(a.z,b.x),sh2z_mul(a.x,b.z)),
        sh2z_sub(sh2z_mul(a.x,b.y),sh2z_mul(a.y,b.x))};
}
sh2z_fx_t sh2z_vec3_length(sh2z_vec3_t v) { return sh2z_sqrt(sh2z_vec3_dot(v,v)); }
sh2z_vec3_t sh2z_vec3_normalize(sh2z_vec3_t v) {
    sh2z_fx_t length = sh2z_vec3_length(v);
    if (!length) return (sh2z_vec3_t){0,0,0};
    return (sh2z_vec3_t){sh2z_div(v.x,length),sh2z_div(v.y,length),sh2z_div(v.z,length)};
}

sh2z_mat34_t sh2z_mat34_identity(void) {
    sh2z_mat34_t m = {{{SH2Z_ONE,0,0,0},{0,SH2Z_ONE,0,0},{0,0,SH2Z_ONE,0}}};
    return m;
}
void sh2z_mat34_multiply(sh2z_mat34_t *out, const sh2z_mat34_t *a, const sh2z_mat34_t *b) {
    sh2z_mat34_t result = {{{0}}};
    for (int r=0;r<3;r++) for (int c=0;c<4;c++) {
        sh2z_fx_t x = (c == 3) ? a->m[r][3] : 0;
        for (int k=0;k<3;k++) x = sh2z_add(x,sh2z_mul(a->m[r][k],b->m[k][c]));
        result.m[r][c] = x;
    }
    *out = result;
}
sh2z_vec3_t sh2z_mat34_transform_point(const sh2z_mat34_t *m, sh2z_vec3_t v) {
    sh2z_vec3_t o;
    sh2z_fx_t *p[3] = {&o.x,&o.y,&o.z};
    for (int i=0;i<3;i++) {
        *p[i] = sh2z_add(sh2z_add(sh2z_add(sh2z_mul(m->m[i][0],v.x),
            sh2z_mul(m->m[i][1],v.y)),sh2z_mul(m->m[i][2],v.z)),m->m[i][3]);
    }
    return o;
}
sh2z_vec3_t sh2z_mat34_transform_direction(const sh2z_mat34_t *m, sh2z_vec3_t v) {
    sh2z_vec3_t o;
    sh2z_fx_t *p[3] = {&o.x,&o.y,&o.z};
    for (int i=0;i<3;i++) {
        *p[i] = sh2z_add(sh2z_add(sh2z_mul(m->m[i][0],v.x),
            sh2z_mul(m->m[i][1],v.y)),sh2z_mul(m->m[i][2],v.z));
    }
    return o;
}
sh2z_mat44_t sh2z_mat44_identity(void) {
    sh2z_mat44_t m = {{{SH2Z_ONE,0,0,0},{0,SH2Z_ONE,0,0},{0,0,SH2Z_ONE,0},{0,0,0,SH2Z_ONE}}};
    return m;
}
void sh2z_mat44_multiply(sh2z_mat44_t *out, const sh2z_mat44_t *a, const sh2z_mat44_t *b) {
    sh2z_mat44_t result = {{{0}}};
    for (int r=0;r<4;r++) for (int c=0;c<4;c++) {
        sh2z_fx_t value=0;
        for (int k=0;k<4;k++) value=sh2z_add(value,sh2z_mul(a->m[r][k],b->m[k][c]));
        result.m[r][c]=value;
    }
    *out=result;
}
sh2z_vec4_t sh2z_mat44_transform(const sh2z_mat44_t *m, sh2z_vec4_t v) {
    sh2z_vec4_t out;
    sh2z_fx_t *p[4]={&out.x,&out.y,&out.z,&out.w};
    for(int r=0;r<4;r++) {
        sh2z_fx_t value=0;
        const sh2z_fx_t elements[4]={v.x,v.y,v.z,v.w};
        for(int c=0;c<4;c++) value=sh2z_add(value,sh2z_mul(m->m[r][c],elements[c]));
        *p[r]=value;
    }
    return out;
}

sh2z_quat_t sh2z_quat_identity(void) { return (sh2z_quat_t){0,0,0,SH2Z_ONE}; }
sh2z_quat_t sh2z_quat_multiply(sh2z_quat_t a, sh2z_quat_t b) {
    /* Hamilton product; quaternion scalar stored in w. */
    sh2z_quat_t out;
    out.w=sh2z_sub(sh2z_sub(sh2z_sub(sh2z_mul(a.w,b.w),sh2z_mul(a.x,b.x)),sh2z_mul(a.y,b.y)),sh2z_mul(a.z,b.z));
    out.x=sh2z_add(sh2z_add(sh2z_mul(a.w,b.x),sh2z_mul(a.x,b.w)),sh2z_sub(sh2z_mul(a.y,b.z),sh2z_mul(a.z,b.y)));
    out.y=sh2z_add(sh2z_add(sh2z_mul(a.w,b.y),sh2z_mul(a.y,b.w)),sh2z_sub(sh2z_mul(a.z,b.x),sh2z_mul(a.x,b.z)));
    out.z=sh2z_add(sh2z_add(sh2z_mul(a.w,b.z),sh2z_mul(a.z,b.w)),sh2z_sub(sh2z_mul(a.x,b.y),sh2z_mul(a.y,b.x)));
    return out;
}
sh2z_quat_t sh2z_quat_normalize(sh2z_quat_t q) {
    sh2z_fx_t len=sh2z_sqrt(sh2z_vec4_dot((sh2z_vec4_t){q.x,q.y,q.z,q.w},(sh2z_vec4_t){q.x,q.y,q.z,q.w}));
    if (!len) return sh2z_quat_identity();
    return (sh2z_quat_t){sh2z_div(q.x,len),sh2z_div(q.y,len),sh2z_div(q.z,len),sh2z_div(q.w,len)};
}
