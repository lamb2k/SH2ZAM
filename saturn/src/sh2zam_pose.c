/* Model pose from binary32 model state (contract tag pose-1).
 *
 * No floating-point arithmetic: each float is read as its bit pattern and the
 * three binary32 operations of the definition (a difference, a product and a
 * reciprocal) are emulated exactly with integers. Two facts keep that cheap.
 *
 *  1. Rounding to 24 significant bits at bit p and then truncating at a
 *     coarser bit f > p is one floor: floor(RNE_p(x) / 2^f) equals
 *     floor((x + 2^(p-1)) / 2^f) for every x >= 0, ties included (a tie
 *     rounds down only when the kept part is even, and then rounding up could
 *     not have crossed a multiple of 2^f either). The product and the
 *     reciprocal both truncate at a bit above their rounding bit.
 *  2. A binary32 difference rounds at most once, on an aligned 32-bit sum
 *     whose leading bit is 29, 30 or 31 whenever any bit can be lost.
 *
 * The SH-2 has no barrel shifter, so every variable shift is the high word of
 * a 32x32-bit product with a power of two from one table, indexed by a float
 * exponent: no branches on the exponent.
 *
 * The C below is the definition on every target. On SH-2 (with SR.S clear)
 * sh2z_pose_from_f32 is the hand-written sequence further down, which
 * computes the same values with MAC.L dot products, DMULU.L shifts and DIV1
 * steps, and takes a shortcut for the common difference of two floats with
 * equal sign and exponent; with SR.S set it runs the C.
 *
 * Copyright (c) 2026 SH2ZAM Contributors. MIT License.
 * Inspired by SH4ZAM by Falco Girgis and SH4ZAM Contributors.
 */
#include "sh2zam/view_experimental.h"
#include "sh2zam_sections.h"

#if defined(__sh__) && defined(__SH2__) && (!defined(SH2Z_FORCE_PORTABLE) || !SH2Z_FORCE_PORTABLE)
#define SH2ZI_POSE_NATIVE 1
#else
#define SH2ZI_POSE_NATIVE 0
#endif
#if defined(SH2Z_REQUIRE_SH2) && SH2Z_REQUIRE_SH2 && (!defined(__sh__) || !defined(__SH2__))
#error "SH2Z_REQUIRE_SH2 needs an SH-2 compiler target (-m2)"
#endif
#if defined(__GNUC__)
#define SH2ZI_POSE_INLINE static inline __attribute__((always_inline))
#else
#define SH2ZI_POSE_INLINE static inline
#endif

_Static_assert(sizeof(float)==sizeof(uint32_t),"binary32 float required");
#if defined(__FLT_MANT_DIG__) && defined(__FLT_MAX_EXP__)
_Static_assert(__FLT_MANT_DIG__==24 && __FLT_MAX_EXP__==128,"binary32 float required");
#endif

#define POSE_BASIS_MAX2   UINT32_C(0x7F0000A8)  /* 1.00001f (0x3F800054), shifted left once */
#define POSE_SCALE_MIN    UINT32_C(0x3D800000)  /* 0.0625f */
#define POSE_SCALE_SPAN   UINT32_C(0x02000000)  /* 1.0f - 0.0625f, in bits */
#define POSE_DELTA_MAX    UINT32_C(0x10000000)  /* 4096 in Q16 */
#define POSE_ROTATION_MAX UINT32_C(1100000000)

/* pose_g[i] = 2^(i - 113) for i >= 113, else 0. mulhi(x, pose_g[i]) is
 * floor(x / 2^(145 - i)): a right shift by 1..32, or zero past 32. */
#define POSE_G_ONE 113u
static const uint32_t pose_g[145] SH2ZI_RODATA(sh2zi_pose_g)={
    [113]=UINT32_C(1),[114]=UINT32_C(2),[115]=UINT32_C(4),[116]=UINT32_C(8),
    [117]=UINT32_C(16),[118]=UINT32_C(32),[119]=UINT32_C(64),[120]=UINT32_C(128),
    [121]=UINT32_C(256),[122]=UINT32_C(512),[123]=UINT32_C(1024),[124]=UINT32_C(2048),
    [125]=UINT32_C(4096),[126]=UINT32_C(8192),[127]=UINT32_C(16384),[128]=UINT32_C(32768),
    [129]=UINT32_C(65536),[130]=UINT32_C(131072),[131]=UINT32_C(262144),[132]=UINT32_C(524288),
    [133]=UINT32_C(1048576),[134]=UINT32_C(2097152),[135]=UINT32_C(4194304),[136]=UINT32_C(8388608),
    [137]=UINT32_C(16777216),[138]=UINT32_C(33554432),[139]=UINT32_C(67108864),[140]=UINT32_C(134217728),
    [141]=UINT32_C(268435456),[142]=UINT32_C(536870912),[143]=UINT32_C(1073741824),[144]=UINT32_C(2147483648)};

/* The bit pattern of a float, read through a union (no float operation). */
SH2ZI_POSE_INLINE uint32_t pose_bits(const float *p) {
    union { float f; uint32_t u; } v;
    v.f=*p;
    return v.u;
}
/* Two's complement int32 from its bits, without implementation-defined
 * conversion. */
SH2ZI_POSE_INLINE int32_t pose_s32(uint32_t u) {
    return u<=UINT32_C(0x7FFFFFFF)?(int32_t)u:-(int32_t)(~u)-1;
}
SH2ZI_POSE_INLINE uint32_t pose_mulhi(uint32_t a,uint32_t b) {
    return (uint32_t)(((uint64_t)a*b)>>32);
}
SH2ZI_POSE_INLINE uint32_t pose_abs(int32_t v) {
    return v<0?0u-(uint32_t)v:(uint32_t)v;
}

/* trunc(65536 * fl(a - b)) for finite a and b, when |fl(a - b)| <= 4096. */
static bool pose_delta(uint32_t a,uint32_t b,int32_t *out) {
    b^=UINT32_C(0x80000000);                       /* a + (-b) */
    uint32_t ma=a<<1,mb=b<<1;                       /* magnitudes, doubled */
    if(ma<mb){uint32_t t=a;a=b;b=t;t=ma;ma=mb;mb=t;}
    /* x = a has the larger magnitude. Significands with 7 spare bits:
     * |x| = X * 2^(ex - 157), X < 2^31; the same for y. */
    uint32_t ex=ma>>24,ey=mb>>24;
    uint32_t X=(a<<9)>>2,Y=(b<<9)>>2;
    if(ex)X|=UINT32_C(0x40000000);else ex=1;
    if(ey)Y|=UINT32_C(0x40000000);else ey=1;
    const uint32_t d=ex-ey;
    if(d) {
        /* Align y; bits shifted out become one sticky bit. Only d >= 2 can
         * lose bits (Y's low 7 bits are zero), and then the sum below keeps
         * its leading bit at 29 or above, so the sticky bit sits at least
         * five bits under the rounding bit. */
        if(d>31u)Y=Y!=0u;
        else {
            const uint64_t w=(uint64_t)Y*pose_g[145u-d];
            Y=(uint32_t)(w>>32)|((uint32_t)w!=0u);
        }
    }
    const uint32_t S=((a^b)>>31)?X-Y:X+Y;          /* X + Y < 2^32 */
    /* Round S to 24 significant bits (ties to even). Below 2^29 the sum is
     * exact (d <= 1) and has at most 23 significant bits: nothing to round. */
    uint32_t N,p;
#define POSE_RNE(n) ((S>>(n))+((S>>((n)-1u))&1u&((S&((UINT32_C(1)<<((n)-1u))-1u))!=0u||((S>>(n))&1u))))
    if(S>=UINT32_C(0x80000000)){N=POSE_RNE(8u);p=8;}
    else if(S>=UINT32_C(0x40000000)){N=POSE_RNE(7u);p=7;}
    else if(S>=UINT32_C(0x20000000)){N=POSE_RNE(6u);p=6;}
    else {N=S;p=0;}
#undef POSE_RNE
    /* fl(x + y) = N * 2^(ex + p - 157); Q16 = floor(N * 2^(ex + p - 141)),
     * with N < 2^29. */
    const uint32_t up=ex+p;
    uint32_t q;
    if(up<=141u)q=pose_mulhi(N<<3,pose_g[up+1u]);
    else {
        if(up>141u+28u)return N==0u?(*out=0,true):false;
        const uint64_t w=(uint64_t)N*pose_g[up-141u+POSE_G_ONE];
        if((uint32_t)(w>>32))return false;
        q=(uint32_t)w;
    }
    if(q>POSE_DELTA_MAX)return false;
    *out=(a>>31)?-(int32_t)q:(int32_t)q;
    return true;
}

/* One basis value b (|b| <= 1.00001f, checked by the caller) of a model axis
 * whose scale has significand ms and biased exponent es; gm = pose_g + es -
 * 123. With m the 24-bit significand and eb the biased exponent of b:
 *   B = trunc(2^30 * b)            = floor(m * 2^(eb - 120))
 *   M = trunc(65536 * fl(b * s))   = floor((P + half) / 2^(284 - eb - es))
 * by fact 1, where P = m * ms rounds at bit 24 (P >= 2^47) or 23. With
 * H = floor(P / 2^16) both are high words of products with pose_g. A
 * subnormal b has eb = 0, where both table entries are zero. */
SH2ZI_POSE_INLINE void pose_element(uint32_t b,uint32_t ms8,const uint32_t *gm,int32_t *B,int32_t *M) {
    const uint32_t eb=(b<<1)>>24,m8=(b<<8)|UINT32_C(0x80000000);
    uint32_t q=pose_mulhi(m8,pose_g[eb+17u]);
    uint32_t h=pose_mulhi(m8,ms8);
    h+=(h>>31)?UINT32_C(0x80):UINT32_C(0x40);    /* < 2^32: P <= (2^24 - 1)^2 */
    uint32_t m=pose_mulhi(h,gm[eb]);
    if(b>>31){q=0u-q;m=0u-m;}
    *B=pose_s32(q);
    *M=pose_s32(m);
}

/* trunc(65536 * fl(1 / s)) for s in [0.0625, 1]: with g = es - 119 in
 * [4, 8], it is floor(2^(47-g) / ms + 2^-(g+1)) by fact 1. */
static uint32_t pose_inverse(uint32_t ms,uint32_t es) {
    const uint32_t g=es-119u;
    uint32_t q;
#if SH2ZI_POSE_NATIVE
    /* floor(2^(47-g) / ms) by 21 DIV1 steps on a remainder of
     * 2^(26-g) < ms: the quotient has at most 21 bits, so the quotient
     * register's top bit, rotated into the dividend, stays zero. One
     * indivisible block owns Q/M/T; DIV0U leaves SR.S alone. */
    uint32_t rem=pose_g[139u-g];
    q=0;
#define POSE_DIV_STEP "rotcl %[q]\n\tdiv1 %[d], %[r]\n\t"
    __asm__ volatile(
        "div0u\n\t"
        POSE_DIV_STEP POSE_DIV_STEP POSE_DIV_STEP POSE_DIV_STEP POSE_DIV_STEP
        POSE_DIV_STEP POSE_DIV_STEP POSE_DIV_STEP POSE_DIV_STEP POSE_DIV_STEP
        POSE_DIV_STEP POSE_DIV_STEP POSE_DIV_STEP POSE_DIV_STEP POSE_DIV_STEP
        POSE_DIV_STEP POSE_DIV_STEP POSE_DIV_STEP POSE_DIV_STEP POSE_DIV_STEP
        POSE_DIV_STEP
        "rotcl %[q]"
        : [r] "+&r"(rem), [q] "+&r"(q)
        : [d] "r"(ms) : "t");
#undef POSE_DIV_STEP
#else
    q=(uint32_t)((UINT64_C(1)<<(47u-g))/ms);
#endif
    /* The remainder 2^(47-g) - q*ms < 2^24, so its low word suffices. */
    const uint32_t r=0u-q*ms;
    return q+((ms-r)<=pose_mulhi(ms,pose_g[144u-g]));
}

/* 64-bit dot products of three int32 pairs, as high and low words, and
 * their R_32 and R_30: floor((x + 2^(n-1) - [x < 0]) / 2^n) is the high word
 * of x + (2^(n-1) - 1) + [x >= 0], here for results that fit 32 bits. */
typedef struct { uint32_t hi,lo; } pose_w64_t;
SH2ZI_POSE_INLINE pose_w64_t pose_dot_c(const int32_t a[3],const int32_t b[3]) {
    const uint64_t x=(uint64_t)((int64_t)a[0]*b[0]+(int64_t)a[1]*b[1]+(int64_t)a[2]*b[2]);
    return (pose_w64_t){(uint32_t)(x>>32),(uint32_t)x};
}
/* x + (2^(n-1) - 1) + [x >= 0], as high and low words (n = 32: bias
 * 0x7FFFFFFF; n = 30: bias 0x1FFFFFFF). */
SH2ZI_POSE_INLINE pose_w64_t pose_bias(pose_w64_t x,uint32_t bias) {
    const uint32_t lo=x.lo+bias,lo2=lo+((x.hi>>31)^1u);
    return (pose_w64_t){x.hi+(lo<bias)+(lo2<lo),lo2};
}
SH2ZI_POSE_INLINE int32_t pose_r32_c(const int32_t a[3],const int32_t b[3]) {
    return pose_s32(pose_bias(pose_dot_c(a,b),UINT32_C(0x7FFFFFFF)).hi);
}
SH2ZI_POSE_INLINE uint32_t pose_r30_c(const int32_t a[3],const int32_t b[3]) {
    const pose_w64_t x=pose_bias(pose_dot_c(a,b),UINT32_C(0x1FFFFFFF));
    return x.hi<<2|x.lo>>30;
}
/* The work words of one pose, in this order: I[3] (offset 0), D[3] (12),
 * B as rows over world k per model axis j (24) and M likewise (60). */
enum {POSE_W_INV=0,POSE_W_D=3,POSE_W_B=6,POSE_W_M=15,POSE_W_WORDS=24};

/* Everything after decoding: view.r and t with their checks, then the Gram
 * test, the eye and the output, which cannot fail. */
static bool pose_finish(sh2z_pose_t *out,const sh2z_view_t *base,
    const int32_t rotation[3][3],const int32_t *w) {
    const int32_t (*BT)[3]=(const int32_t (*)[3])(w+POSE_W_B),(*MT)[3]=(const int32_t (*)[3])(w+POSE_W_M);
    const int32_t *D=w+POSE_W_D;
    int32_t r[3][3],t[3];
    for(unsigned i=0;i<3;++i) {
        uint32_t row=0;
        for(unsigned j=0;j<3;++j) {
            const int32_t v=pose_r32_c(rotation[i],MT[j]);
            if((uint32_t)v+UINT32_C(32768)>UINT32_C(65535))return false;
            r[i][j]=v;
            row+=pose_abs(v);
        }
        const int32_t v=pose_r32_c(rotation[i],D);
        t[i]=v;
        if(row>UINT32_C(65535) || row*UINT32_C(32768)+pose_abs(v)>UINT32_C(0x7FFFFFFF))return false;
    }
    /* |R_30(B . B)| < 2^32 and |R_30(B . D)| < 2^30 by the bounds on B and
     * D, so the wrapped words below are exact; |eye| < 2^28 likewise. */
    bool cull=true;
    for(unsigned i=0;i<3;++i)for(unsigned j=i;j<3;++j) {
        const uint32_t e=pose_r30_c(BT[i],BT[j])-(i==j?UINT32_C(0x40000000):0u);
        if(e+512u>1024u)cull=false;
    }
    uint32_t octant=0,bit=1;
    int32_t e[3];
    for(unsigned j=0;j<3;++j,bit<<=1) {
        const int32_t local=pose_s32(pose_r30_c(BT[j],D));
        const uint64_t x=(uint64_t)((int64_t)local*w[POSE_W_INV+j]);
        const int32_t v=-pose_s32(pose_bias((pose_w64_t){(uint32_t)(x>>32),(uint32_t)x},UINT32_C(0x7FFFFFFF)).hi);
        e[j]=v;
        if(v>=0)octant|=bit;
        if((uint32_t)v+30000u>60000u)cull=false;
    }
    for(unsigned i=0;i<3;++i) {
        for(unsigned j=0;j<3;++j)out->view.r[i][j]=(int16_t)r[i][j];
        out->view.t[i]=t[i];
        out->eye[i]=e[i];
    }
    out->view.focal_x=base->focal_x;out->view.focal_y=base->focal_y;
    out->view.center_x=base->center_x;out->view.center_y=base->center_y;
    out->view.near_z=base->near_z;out->view.far_z=base->far_z;
    out->view.clip[0]=base->clip[0];out->view.clip[1]=base->clip[1];
    out->view.clip[2]=base->clip[2];out->view.clip[3]=base->clip[3];
    for(unsigned k=0;k<3;++k)for(unsigned j=0;j<3;++j) {
        const int32_t b=BT[j][k];
        const uint32_t m=(pose_abs(b)+UINT32_C(32768))>>16;
        out->basis[k][j]=(int16_t)(b<0?-(int32_t)m:(int32_t)m);
    }
    out->eye_error=2;
    out->octant=(uint8_t)octant;
    out->cull_valid=cull;
    return true;
}

#if SH2ZI_POSE_NATIVE
/* ---- SH-2 path (SR.S clear) ----
 * sh2z_pose_from_f32 itself, in SH-2 instructions: the checks, the three
 * differences, the decode and pose_finish, computing exactly what the C above
 * computes. With SR.S set (MAC.L saturating at 48 bits) it jumps to the C
 * function, sh2zi_pose_from_f32_c, with its arguments untouched.
 *
 * Frame (r15 after the prologue): 0..31 staged view.r and view.t; 32, 36, 40
 * out, base, rotation; 44..55 I[3]; 56, 60 the last scale's bits and I;
 * w = 64: M rows (w+0), B rows (w+36), D (w+72). R_n of a MAC.L sum: CMP/PZ
 * puts [x >= 0] in T and two ADDCs add it with 2^(n-1) - 1, carrying into
 * MACH's word; for n = 30 two SHLL/ROTCL pairs then take bits 61..30.
 *
 * Internal routines (custom conventions, also reached by the test entries):
 *   .Lp_delta    r4 = a, r5 = b bits: r0 = pose_delta's value, T = success;
 *                clobbers r0-r7, MACH, MACL
 *   .Lp_inverse  r5 = ms << 8, r6 = 4 es (scale in range): r0 =
 *                pose_inverse; clobbers r1-r3, MACH, MACL, M, Q, T */
_Static_assert(offsetof(sh2z_view_t,t)==20 && offsetof(sh2z_view_t,focal_x)==32 &&
    offsetof(sh2z_view_t,focal_y)==36 && offsetof(sh2z_view_t,near_z)==44 &&
    offsetof(sh2z_view_t,far_z)==48 && offsetof(sh2z_view_t,clip)==52 &&
    sizeof(sh2z_view_t)==60,"view layout");
_Static_assert(offsetof(sh2z_pose_t,eye)==60 && offsetof(sh2z_pose_t,basis)==72 &&
    offsetof(sh2z_pose_t,eye_error)==90 && offsetof(sh2z_pose_t,octant)==92 &&
    offsetof(sh2z_pose_t,cull_valid)==93,"pose layout");
bool sh2zi_pose_from_f32_c(sh2z_pose_t *out,const sh2z_view_t *base,
    const int32_t rotation[3][3],const float basis[3][3],const float scale[3],
    const float position[3],const float eye[3],unsigned flags);
/* Test entries (C calling convention; declared by the model bridge only). */
int32_t sh2zi_pose_test_delta(uint32_t a,uint32_t b);       /* INT32_MIN on failure */
uint32_t sh2zi_pose_test_inverse(uint32_t s);                /* s in [0.0625, 1] */
uint32_t sh2zi_pose_test_element(uint32_t b,uint32_t s,int32_t out[2]); /* {B, M}; 0 if b fails */

#define PA_ROT(o) "mov.l @(" #o ",r6),r0\n\tadd r1,r0\n\tcmp/hi r2,r0\n\tbt .Lp_false\n\t"
#define PA_ROTF(o) "mov.l @(" #o ",r6),r0\n\tadd r1,r0\n\tor r0,r2\n\t"
#define PA_DELTA(o,d) "mov.l @(" #o ",r8),r4\n\tbsr .Lp_delta\n\tmov.l @(" #o ",r9),r5\n\tbf .Lp_fail0\n\tmov.l r0,@(" #d ",r10)\n\t"
/* One basis value: r8 row, r10 = w + 12j, r5 = ms << 8 (top bit set),
 * r6 = pose_g + es - 123, r7 = pose_g + 17, r13 = 2^31, r14 = 1.00001f << 1.
 * M[j][k] at r10 + 4k, B[j][k] at r10 + 36 + 4k. */
#define PA_ELEMENT(k,m,b,fail) \
    "mov.l @(" #k ",r8),r1\n\tmov r1,r0\n\tshll r0\n\tcmp/hi r14,r0\n\tbt " fail "\n\t" \
    "shlr16 r0\n\tmov r1,r2\n\tshlr8 r0\n\tshll8 r2\n\tshll2 r0\n\tor r13,r2\n\t" /* 4 eb, m8 */ \
    "dmulu.l r2,r5\n\tmov.l @(r0,r7),r3\n\tmov.l @(r0,r6),r4\n\tsts mach,r0\n\t"  /* H */ \
    "dmulu.l r2,r3\n\t" \
    "cmp/pz r0\n\tbt/s 1f\n\tadd #64,r0\n\tadd #64,r0\n1:\n\t" \
    "sts mach,r3\n\tdmulu.l r0,r4\n\tcmp/pz r1\n\tsts mach,r2\n\t"              /* |B|, |M| */ \
    "bt 2f\n\tneg r3,r3\n\tneg r2,r2\n2:\n\t" \
    "mov.l r2,@(" #m ",r10)\n\tmov.l r3,@(" #b ",r10)\n\t"
/* Model axis j: its scale (range, I with a one-entry memo), then 3 values. */
#define PA_AXIS(i,fail) \
    "mov.l @r11,r1\n\tadd r12,r11\n\t" \
    "mov.l .Lp_k_sneg,r0\n\tadd r1,r0\n\tmov.l .Lp_k_span,r2\n\tcmp/hi r2,r0\n\tbt " fail "\n\t" \
    "mov r1,r5\n\tshll8 r5\n\tor r13,r5\n\t"                              /* ms << 8 */ \
    "mov r1,r6\n\tshll r6\n\tshlr16 r6\n\tshlr8 r6\n\tshll2 r6\n\t"       /* 4 es */ \
    "mov.l @(56,r15),r0\n\tcmp/eq r0,r1\n\tbt/s 1f\n\tmov.l @(60,r15),r0\n\t" \
    "bsr .Lp_inverse\n\tmov.l r1,@(56,r15)\n\tmov.l r0,@(60,r15)\n1:\n\t" \
    "mov.l r0,@(" #i ",r15)\n\t" \
    "mov.l .Lp_k_gm,r0\n\tadd r0,r6\n\t"                                  /* pose_g + es - 123 */ \
    PA_ELEMENT(0,0,36,fail) PA_ELEMENT(4,4,40,fail) PA_ELEMENT(8,8,44,fail) \
    "add r9,r8\n\tadd #12,r10\n\t"
#define PA_STUB(name) "bra 1f\n\tnop\n" name ":\n\tbra .Lp_fail\n\tmov #0,r0\n1:\n\t"
#define PF_MAC3 "clrmac\n\tmac.l @r2+,@r3+\n\tmac.l @r2+,@r3+\n\tmac.l @r2+,@r3+\n\tsts mach,r1\n\tsts macl,r0\n\t"
#define PF_ROUND "cmp/pz r1\n\taddc r13,r0\n\taddc r12,r1\n\t"
#define PF_SHR30 "shll r0\n\trotcl r1\n\tshll r0\n\trotcl r1\n\t"
/* view.r[i][j] staged at r15 + o (failing to stub f), |r| summed in r14;
 * then the next statement (rewinding the rotation row). */
#define PF_R(o,f,next) \
    PF_MAC3 PF_ROUND "exts.w r1,r0\n\tcmp/eq r1,r0\n\tbf " f "\n\tmov.w r0,@(" #o ",r15)\n\t" \
    "cmp/pz r1\n\tbt 1f\n\tneg r1,r1\n1:\n\tadd r1,r14\n\t" next
/* Row i: r2 = rotation row i (left at row i + 1), r10 = 65535. */
#define PF_ROW(o0,o1,o2,ot,f) \
    "mov #0,r14\n\tmov r7,r3\n\t" \
    PF_R(o0,f,"add #-12,r2\n\t") PF_R(o1,f,"add #-12,r2\n\t") PF_R(o2,f,"add #-12,r2\n\t") \
    "add #36,r3\n\t" PF_MAC3 PF_ROUND "mov.l r1,@(" #ot ",r15)\n\t" \
    "cmp/hi r10,r14\n\tbt " f "\n\tshll16 r14\n\tshlr r14\n\t" \
    "cmp/pz r1\n\tbt 1f\n\tneg r1,r1\n1:\n\tadd r1,r14\n\tcmp/pz r14\n\tbf " f "\n\t"
/* Gram entry of the B rows at r2, r3 (both then advance a row). With the
 * accumulator preloaded with K (r10:r11), |R_30(x) - c| <= 512 exactly when
 * x + K < L as 64-bit words (L = r9:r13), which SUBC's borrow gives:
 *   diagonal (c = 2^30, x >= 0)  K = 2^29 - (2^30 - 512) 2^30, L = 1025 2^30
 *   off-diagonal (c = 0)         K = 1025 2^29 - 1,            L = 1025 2^30 - 1
 * r8 counts the cull tests passed. */
#define PF_GRAM \
    "lds r10,mach\n\tlds r11,macl\n\tmac.l @r2+,@r3+\n\tmac.l @r2+,@r3+\n\tmac.l @r2+,@r3+\n\t" \
    "sts macl,r0\n\tsts mach,r1\n\tclrt\n\tsubc r13,r0\n\tsubc r9,r1\n\taddc r12,r8\n\t"
/* eye[j] = -R_32(R_30(B_j . D) * I_j) to r5 + e (I_j at r15 + i); its
 * octant bit rotated into r14 (j = 2, 1, 0); r8 counts |eye| <= 30000. */
#define PF_EYE(i,e) \
    PF_MAC3 PF_ROUND PF_SHR30 \
    "mov.l @(" #i ",r15),r0\n\tdmuls.l r0,r1\n\tsts mach,r1\n\tsts macl,r0\n\t" \
    "cmp/pz r1\n\taddc r11,r0\n\taddc r12,r1\n\tneg r1,r1\n\tmov.l r1,@(" #e ",r5)\n\t" \
    "cmp/pz r1\n\trotcl r14\n\tmov r1,r0\n\tadd r10,r0\n\tcmp/hs r0,r9\n\taddc r12,r8\n\t"
/* basis[k][j] = R_16(B[j][k]): the high half of B + 0x7FFF + [B >= 0], from
 * r6 + src to r3 + dst (r10 = 0x7FFF); PF_Q14P stores two at once, XTRCT
 * joining the high halves. */
#define PF_Q14(src,dst) \
    "mov.l @(" #src ",r6),r1\n\tcmp/pz r1\n\taddc r10,r1\n\tswap.w r1,r0\n\tmov.w r0,@(" #dst ",r3)\n\t"
#define PF_Q14P(sa,sb,dst) \
    "mov.l @(" #sa ",r6),r1\n\tcmp/pz r1\n\taddc r10,r1\n\tswap.w r1,r1\n\t" \
    "mov.l @(" #sb ",r6),r0\n\tcmp/pz r0\n\taddc r10,r0\n\txtrct r1,r0\n\tmov.l r0,@(" #dst ",r3)\n\t"
#define PF_COPY(o) "mov.l @(" #o ",r5),r0\n\tmov.l r0,@(" #o ",r4)\n\t"
#define PF_STAGED(o) "mov.l @(" #o ",r15),r0\n\tmov.l r0,@(" #o ",r4)\n\t"
#define PD_DIV "rotcl r3\n\tdiv1 r2,r1\n\t"
#define PD_DIV7 PD_DIV PD_DIV PD_DIV PD_DIV PD_DIV PD_DIV PD_DIV
__asm__(
"    .pushsection .text.sh2z_pose_from_f32,\"ax\",@progbits\n"
"    .align 2\n"
"    .global _sh2z_pose_from_f32\n"
"    .type _sh2z_pose_from_f32,@function\n"
"_sh2z_pose_from_f32:\n\t"
    "stc sr,r0\n\ttst #2,r0\n\tbf .Lp_portable\n\t"
    "tst r4,r4\n\tbt .Lp_false\n\ttst r5,r5\n\tbt .Lp_false\n\t"
    "tst r6,r6\n\tbt .Lp_false\n\ttst r7,r7\n\tbt .Lp_false\n\t"
    "mov.l @(32,r5),r0\n\tcmp/pl r0\n\tbf .Lp_false\n\t"               /* focal_x */
    "mov.l @(36,r5),r0\n\tcmp/pl r0\n\tbf .Lp_false\n\t"               /* focal_y */
    "mov.l @(44,r5),r1\n\tcmp/pl r1\n\tbf .Lp_false\n\t"               /* near_z */
    "mov.l @(48,r5),r0\n\tcmp/gt r1,r0\n\tbf .Lp_false\n\t"            /* far_z */
    "mov r5,r1\n\tadd #52,r1\n\t"                                     /* clip */
    "mov.w @r1+,r2\n\tmov.w @r1+,r3\n\tmov.w @r1+,r0\n\tcmp/gt r0,r2\n\tbt .Lp_false\n\t"
    "mov.w @r1,r0\n\tcmp/gt r0,r3\n\tbt .Lp_false\n\t"
    /* Rotation: all nine in [-2^30, 2^30) (bit 31 of the OR of v + 2^30
     * clear) pass at once; otherwise each is checked against 1.1e9. */
    "mov #1,r1\n\trotr r1\n\tshlr r1\n\tmov #0,r2\n\t"
    PA_ROTF(0) PA_ROTF(4) PA_ROTF(8) PA_ROTF(12) PA_ROTF(16) PA_ROTF(20) PA_ROTF(24) PA_ROTF(28) PA_ROTF(32)
    "cmp/pz r2\n\tbf .Lp_rotation\n"
".Lp_rotation_ok:\n\t"
    "mov.l @(12,r15),r0\n\tshlr r0\n\ttst r0,r0\n\tbf .Lp_false\n\t"   /* flags <= 1 */
    "mov.l @r15,r1\n\ttst r1,r1\n\tbt .Lp_false\n\t"                   /* scale */
    "mov.l @(4,r15),r2\n\ttst r2,r2\n\tbt .Lp_false\n\t"               /* position */
    "mov.l @(8,r15),r3\n\ttst r3,r3\n\tbt .Lp_false\n\t"               /* eye */
    "bra .Lp_body\n\t"
    "mov.l @(12,r15),r0\n"                                            /* flags */
".Lp_false:\n\t"
    "rts\n\t"
    "mov #0,r0\n"
".Lp_portable:\n\t"
    "mov.l .Lp_k_c,r0\n\t"
    "jmp @r0\n\t"
    "nop\n"
".Lp_rotation:\n\t"
    "mov.l .Lp_k_rot,r1\n\tmov r1,r2\n\tadd r1,r2\n\t"
    PA_ROT(0) PA_ROT(4) PA_ROT(8) PA_ROT(12) PA_ROT(16) PA_ROT(20) PA_ROT(24) PA_ROT(28) PA_ROT(32)
    "bra .Lp_rotation_ok\n\t"
    "nop\n\t"
    ".align 2\n"
".Lp_k_rot:\n\t.long 1100000000\n"
".Lp_k_c:\n\t.long _sh2zi_pose_from_f32_c\n"
".Lp_body:\n\t"
    "mov.l r8,@-r15\n\tmov.l r9,@-r15\n\tmov.l r10,@-r15\n\tmov.l r11,@-r15\n\t"
    "mov.l r12,@-r15\n\tmov.l r13,@-r15\n\tmov.l r14,@-r15\n\t"
    "sts.l pr,@-r15\n\tsts.l mach,@-r15\n\tsts.l macl,@-r15\n\t"
    "add #-128,r15\n\tadd #-20,r15\n\t"
    "mov.l r4,@(32,r15)\n\tmov.l r5,@(36,r15)\n\tmov.l r6,@(40,r15)\n\t"
    "mov r2,r8\n\tmov r3,r9\n\tmov r1,r11\n\tmov r7,r12\n\tmov r0,r13\n\t"
    "mov r15,r10\n\tadd #100,r10\n\t"                                 /* w + 36: D at +36 */
    PA_DELTA(0,36) PA_DELTA(4,40) PA_DELTA(8,44)
    PA_STUB(".Lp_fail0")
    /* Axis order: rows and scales forward, or backward under the flag. */
    "mov r12,r8\n\tmov #12,r9\n\tmov #4,r12\n\t"
    "mov r13,r0\n\ttst #1,r0\n\tbt 1f\n\t"
    "add #24,r8\n\tmov #-12,r9\n\tadd #8,r11\n\tmov #-4,r12\n1:\n\t"
    "mov.l .Lp_k_2p31,r13\n\tmov.l .Lp_k_bmax,r14\n\tmov.l .Lp_k_g17,r7\n\t"
    "mov r15,r10\n\tadd #64,r10\n\t"
    "mov #0,r0\n\tmov.l r0,@(56,r15)\n\t"
    PA_AXIS(44,".Lp_fail1") PA_STUB(".Lp_fail1")
    PA_AXIS(48,".Lp_fail2") PA_STUB(".Lp_fail2")
    PA_AXIS(52,".Lp_fail3")
    "bra .Lp_finish\n\t"
    "nop\n"
".Lp_fail3:\n\t"
    "bra .Lp_fail\n\t"
    "mov #0,r0\n\t"
    ".align 2\n"
".Lp_k_2p31:\n\t.long 0x80000000\n"
".Lp_k_bmax:\n\t.long 0x7F0000A8\n"
".Lp_k_g17:\n\t.long _pose_g+68\n"
".Lp_k_gm:\n\t.long _pose_g-492\n"
".Lp_k_sneg:\n\t.long 0xC2800000\n"
".Lp_k_span:\n\t.long 0x02000000\n"
".Lpf_fail0:\n\t"
    "bra .Lp_fail\n\t"
    "mov #0,r0\n"
".Lp_finish:\n\t"
    "mov.l @(40,r15),r2\n\tmov r15,r7\n\tadd #64,r7\n\t"
    "mov #-1,r13\n\tshlr r13\n\t"        /* 0x7FFFFFFF */
    "mov #0,r12\n\t"
    "mov #-1,r10\n\textu.w r10,r10\n\t"  /* 65535 */
    PF_ROW(0,2,4,20,".Lpf_fail0")
    PA_STUB(".Lpf_fail1")
    PF_ROW(6,8,10,24,".Lpf_fail1")
    PA_STUB(".Lpf_fail2")
    PF_ROW(12,14,16,28,".Lpf_fail2")
    /* Nothing fails from here: the view, then the cull terms, eye, basis. */
    "mov.l @(32,r15),r4\n\tmov.l @(36,r15),r5\n\t"
    PF_STAGED(0) PF_STAGED(4) PF_STAGED(8) PF_STAGED(12)
    "mov.w @(16,r15),r0\n\tmov.w r0,@(16,r4)\n\t"
    PF_STAGED(20) PF_STAGED(24) PF_STAGED(28)
    PF_COPY(32) PF_COPY(36) PF_COPY(40) PF_COPY(44) PF_COPY(48) PF_COPY(52) PF_COPY(56)
    "mov.l .Lp_k_kd,r10\n\t"        /* K diagonal: 0xF0000080:20000000 */
    "mov #1,r11\n\trotr r11\n\tshlr2 r11\n\t"
    "mov #1,r9\n\tshll8 r9\n\t"     /* L: 0x100:40000000 */
    "mov r11,r13\n\tadd r11,r13\n\t"
    "mov #0,r8\n\t"
    "mov r7,r6\n\tadd #36,r6\n\t"   /* B rows */
    "mov r6,r2\n\tmov r6,r3\n\t"
    PF_GRAM PF_GRAM PF_GRAM                                   /* 00 11 22 */
    "mov #-128,r10\n\textu.b r10,r10\n\t"                     /* K: 0x80:1FFFFFFF */
    "add #-1,r11\n\tadd #-1,r13\n\t"                          /* L: 0x100:3FFFFFFF */
    "mov r6,r2\n\tmov r6,r3\n\tadd #12,r3\n\t"
    PF_GRAM PF_GRAM                                           /* 01 12 */
    "mov r6,r2\n\tmov r6,r3\n\tadd #24,r3\n\t"
    PF_GRAM                                                   /* 02 */
    "mov r11,r13\n\t"               /* 0x1FFFFFFF */
    "mov #-1,r11\n\tshlr r11\n\t"   /* 0x7FFFFFFF */
    "mov #117,r10\n\tshll8 r10\n\tadd #48,r10\n\t"  /* 30000 */
    "mov r10,r9\n\tadd r10,r9\n\t"  /* 60000 */
    "mov #0,r14\n\t"
    "mov r4,r5\n\tadd #60,r5\n\t"   /* eye */
    "mov r6,r2\n\tadd #24,r2\n\tmov r7,r3\n\tadd #72,r3\n\t"
    PF_EYE(52,8) "add #-24,r2\n\tadd #-12,r3\n\t"
    PF_EYE(48,4) "add #-24,r2\n\tadd #-12,r3\n\t"
    PF_EYE(44,0)
    "mov #-1,r10\n\tshlr16 r10\n\tshlr r10\n\t"     /* 0x7FFF */
    "mov r4,r3\n\tadd #72,r3\n\t"
    PF_Q14P(0,12,0) PF_Q14P(24,4,4) PF_Q14P(16,28,8) PF_Q14P(8,20,12) PF_Q14(32,16)
    "add #18,r3\n\t"
    "mov #2,r0\n\tmov.w r0,@r3\n\t"                 /* eye_error */
    "mov r14,r0\n\tmov.b r0,@(2,r3)\n\t"            /* octant */
    "mov r8,r0\n\tcmp/eq #9,r0\n\tmovt r0\n\tmov.b r0,@(3,r3)\n\t"  /* cull_valid */
    "mov #1,r0\n"
".Lp_exit:\n\t"
    "add #127,r15\n\tadd #21,r15\n\t"
    "lds.l @r15+,macl\n\tlds.l @r15+,mach\n\tlds.l @r15+,pr\n\t"
    "mov.l @r15+,r14\n\tmov.l @r15+,r13\n\tmov.l @r15+,r12\n\tmov.l @r15+,r11\n\t"
    "mov.l @r15+,r10\n\tmov.l @r15+,r9\n\t"
    "rts\n\t"
    "mov.l @r15+,r8\n"
".Lp_fail:\n\t"
    "bra .Lp_exit\n\t"
    "mov #0,r0\n\t"
    ".align 2\n"
".Lp_k_kd:\n\t.long 0xF0000080\n"
"    .size _sh2z_pose_from_f32,.-_sh2z_pose_from_f32\n"
    /* pose_delta: a in r4, b in r5. First the common case: equal signs and
     * exponent fields e. Then a - b is exact (Sterbenz) and is the
     * difference of the bit patterns in units of 2^(max(e, 1) - 150), so
     * Q16 = |a - b| 2^(e - 134), truncated (e = 0 gives 0 either way). */
".Lpd_fail:\n\t"
    "rts\n\t"
    "clrt\n"
".Lp_delta:\n\t"
    "mov r4,r0\n\txor r5,r0\n\tmov.l .Lp_k_2p23,r3\n\tcmp/hs r3,r0\n\tbt .Lpd_general\n\t"
    "mov r4,r1\n\tshll r1\n\tshlr16 r1\n\tshlr8 r1\n\t"              /* e */
    "mov r4,r2\n\tsub r5,r2\n\t"                                      /* significand difference */
    "cmp/pz r2\n\tbt 1f\n\tneg r2,r2\n\tnot r4,r4\n1:\n\t"           /* r4 bit 31: the sign */
    "shll8 r2\n\tmov r1,r0\n\tshll2 r0\n\t"
    "mov r1,r3\n\tadd #-128,r3\n\tadd #-14,r3\n\tcmp/pz r3\n\tbt .Lpd_fleft\n\t"  /* e >= 142 */
    "mov.l .Lp_k_g3,r7\n\tmov.l @(r0,r7),r7\n\t"                     /* 2^(e - 110) or 0 */
    "dmulu.l r7,r2\n\tsts mach,r0\n"
".Lpd_bound:\n\t"
    "mov.l .Lp_k_2p28,r7\n\tcmp/hi r7,r0\n\tbt .Lpd_fail\n\t"
    "cmp/pz r4\n\tbt 1f\n\tneg r0,r0\n1:\n\t"
    "rts\n\t"
    "sett\n"
".Lpd_fleft:\n\t"
    "mov #31,r7\n\tcmp/hi r7,r3\n\tbt .Lpd_fhuge\n\t"
    "mov.l .Lp_k_gm29,r7\n\tmov.l @(r0,r7),r7\n\t"                   /* 2^(e - 142) */
    "dmulu.l r7,r2\n\tsts mach,r7\n\ttst r7,r7\n\tbf .Lpd_fail\n\t"
    "bra .Lpd_bound\n\tsts macl,r0\n"
".Lpd_fhuge:\n\t"                                                     /* e >= 174 */
    "mov r1,r0\n\tadd #1,r0\n\tshlr8 r0\n\ttst r0,r0\n\tbf .Lpd_fail\n\t"   /* e = 255: inf or NaN */
    "mov #0,r0\n\trts\n\ttst r2,r2\n"                                  /* finite: 0, or too far (T clear) */
    /* Otherwise align, add and round as pose_delta does. */
".Lpd_general:\n\t"
    "mov r4,r0\n\tshll r0\n\tmov r5,r1\n\tshll r1\n\t"               /* magnitudes, doubled */
    "mov r4,r2\n\txor r5,r2\n\t"                                      /* < 0: they add */
    "cmp/hs r1,r0\n\tbt 1f\n\t"
    "mov r0,r3\n\tmov r1,r0\n\tmov r3,r1\n\tnot r5,r4\n"              /* r4: the sign of the larger */
"1:\n\t"
    "mov.l .Lp_k_fin,r3\n\tcmp/hi r3,r0\n\tbt .Lpd_fail\n\t"          /* the larger finite: both */
    "mov r0,r3\n\tshlr16 r3\n\tshlr8 r3\n\t"                          /* ex */
    "mov r1,r5\n\tshlr16 r5\n\tshlr8 r5\n\t"                          /* ey */
    "shll8 r0\n\tshlr2 r0\n\tshll8 r1\n\tshlr2 r1\n\t"
    "tst r5,r5\n\tbt .Lpd_sub\n\t"                                    /* y subnormal or zero */
    "mov.l .Lp_k_2p30,r6\n\tor r6,r0\n\tor r6,r1\n"                   /* X, Y */
".Lpd_ey1:\n\t"
    "mov r3,r6\n\tsub r5,r6\n\t"                                      /* d */
    "add #-128,r3\n\tadd #-13,r3\n\t"                                 /* ex - 141 */
    "tst r6,r6\n\tbf .Lpd_shift\n"
".Lpd_aligned:\n\t"
    "cmp/pz r2\n\tbt/s .Lpd_sum\n\tsub r1,r0\n\tadd r1,r0\n\tadd r1,r0\n"
".Lpd_sum:\n\t"                                                       /* S */
    "mov r0,r1\n\tshll r1\n\tbt .Lpd_p8\n\tshll r1\n\tbt .Lpd_p7\n\tshll r1\n\tbt .Lpd_p6\n\t"
    "mov r0,r1\n"
    /* fl(x + y) = N 2^(ex + p - 157): Q16 = N 2^u with u = ex + p - 141
     * (r3): the high word of N 2^(u + 32) for u < 0, else N 2^u. */
".Lpd_scale:\n\t"
    "mov r3,r0\n\tshll2 r0\n\tcmp/pz r3\n\tbt .Lpd_left\n\t"
    "mov.l .Lp_k_g145,r7\n\tmov.l @(r0,r7),r7\n\t"                   /* 2^(u + 32) or 0 */
    "dmulu.l r7,r1\n\tbra .Lpd_bound\n\tsts mach,r0\n"
".Lpd_left:\n\t"
    "mov #31,r7\n\tcmp/hi r7,r3\n\tbt .Lpd_huge\n\t"
    "mov.l .Lp_k_g113,r7\n\tmov.l @(r0,r7),r7\n\t"                   /* 2^u */
    "dmulu.l r7,r1\n\tsts mach,r7\n\ttst r7,r7\n\tbf .Lpd_fail\n\t"
    "bra .Lpd_bound\n\tsts macl,r0\n"
".Lpd_huge:\n\t"
    "mov #0,r0\n\trts\n\ttst r1,r1\n"                                  /* 0, or too far (T clear) */
".Lpd_shift:\n\t"                                                     /* align y: d > 0 */
    "mov #31,r7\n\tcmp/hi r7,r6\n\tbt .Lpd_far\n\t"
    "mov.l .Lp_k_g145,r7\n\tshll2 r6\n\tsub r6,r7\n\tmov.l @r7,r7\n\t" /* 2^(32-d) */
    "dmulu.l r7,r1\n\tsts mach,r1\n\tsts macl,r7\n\t"
    "tst r7,r7\n\tmov #-1,r6\n\tnegc r6,r7\n\tbra .Lpd_aligned\n\tor r7,r1\n"  /* sticky */
".Lpd_far:\n\t"
    "tst r1,r1\n\tmov #-1,r6\n\tbra .Lpd_aligned\n\tnegc r6,r1\n"
".Lpd_sub:\n\t"                                                       /* ey = 0: exponent 1 */
    "mov #1,r5\n\tmov.l .Lp_k_2p30,r6\n\ttst r3,r3\n\tbf/s .Lpd_ey1\n\tor r6,r0\n\t"
    "xor r6,r0\n\tbra .Lpd_ey1\n\tmov #1,r3\n"                        /* ex = 0 too */
".Lpd_p8:\n\t"
    "mov r0,r1\n\tadd #64,r1\n\tadd #64,r1\n\tshlr8 r1\n\t"
    "and #255,r0\n\txor #128,r0\n\ttst r0,r0\n\tbf 1f\n\tshlr r1\n\tshll r1\n1:\n\t"
    "bra .Lpd_scale\n\tadd #8,r3\n"
".Lpd_p7:\n\t"
    "mov r0,r1\n\tadd #64,r1\n\tshlr2 r1\n\tshlr2 r1\n\tshlr2 r1\n\tshlr r1\n\t"
    "and #127,r0\n\txor #64,r0\n\ttst r0,r0\n\tbf 1f\n\tshlr r1\n\tshll r1\n1:\n\t"
    "bra .Lpd_scale\n\tadd #7,r3\n"
".Lpd_p6:\n\t"
    "mov r0,r1\n\tadd #32,r1\n\tshlr2 r1\n\tshlr2 r1\n\tshlr2 r1\n\t"
    "and #63,r0\n\txor #32,r0\n\ttst r0,r0\n\tbf 1f\n\tshlr r1\n\tshll r1\n1:\n\t"
    "bra .Lpd_scale\n\tadd #6,r3\n"
    /* pose_inverse: ms << 8 in r5, 4 es in r6. */
".Lp_inverse:\n\t"
    "mov r5,r2\n\tshlr8 r2\n\t"                                      /* ms */
    "mov.l .Lp_k_g263,r1\n\tsub r6,r1\n\tmov.l @r1,r1\n\tdmulu.l r1,r2\n\t" /* ms >> (g + 1) */
    "mov.l .Lp_k_g258,r1\n\tsub r6,r1\n\tmov.l @r1,r1\n\t"          /* 2^(26 - g) */
    "mov #0,r3\n\tdiv0u\n\t"
    PD_DIV7 PD_DIV7 PD_DIV7
    "rotcl r3\n\t"
    "sts mach,r0\n\tmul.l r3,r2\n\tsts macl,r1\n\t"
    "add r1,r2\n\tcmp/hs r2,r0\n\tmovt r0\n\t"                      /* ms - remainder <= lim */
    "rts\n\t"
    "add r3,r0\n"
    /* Test entries. */
"    .global _sh2zi_pose_test_delta\n"
"    .type _sh2zi_pose_test_delta,@function\n"
"_sh2zi_pose_test_delta:\n\t"
    "sts.l pr,@-r15\n\tsts.l mach,@-r15\n\t"
    "bsr .Lp_delta\n\t"
    "sts.l macl,@-r15\n\t"
    "bt 1f\n\tmov #1,r0\n\trotr r0\n1:\n\t"
    "lds.l @r15+,macl\n\tlds.l @r15+,mach\n\tlds.l @r15+,pr\n\t"
    "rts\n\tnop\n"
"    .size _sh2zi_pose_test_delta,.-_sh2zi_pose_test_delta\n"
"    .global _sh2zi_pose_test_inverse\n"
"    .type _sh2zi_pose_test_inverse,@function\n"
"_sh2zi_pose_test_inverse:\n\t"
    "sts.l pr,@-r15\n\tsts.l mach,@-r15\n\tsts.l macl,@-r15\n\t"
    "mov #1,r1\n\trotr r1\n\tmov r4,r5\n\tshll8 r5\n\tor r1,r5\n\t"        /* ms << 8 */
    "mov r4,r6\n\tshll r6\n\tshlr16 r6\n\tshlr8 r6\n\t"                    /* es */
    "bsr .Lp_inverse\n\t"
    "shll2 r6\n\t"
    "lds.l @r15+,macl\n\tlds.l @r15+,mach\n\tlds.l @r15+,pr\n\t"
    "rts\n\tnop\n"
"    .size _sh2zi_pose_test_inverse,.-_sh2zi_pose_test_inverse\n"
"    .global _sh2zi_pose_test_element\n"
"    .type _sh2zi_pose_test_element,@function\n"
"_sh2zi_pose_test_element:\n\t"
    "mov.l r8,@-r15\n\tmov.l r10,@-r15\n\tmov.l r13,@-r15\n\tmov.l r14,@-r15\n\t"
    "sts.l mach,@-r15\n\tsts.l macl,@-r15\n\t"
    "add #-56,r15\n\t"
    "mov.l r4,@(48,r15)\n\tmov.l r6,@(52,r15)\n\t"
    "mov r15,r8\n\tadd #48,r8\n\tmov r15,r10\n\t"
    "mov #1,r13\n\trotr r13\n\tmov.l .Lp_k_bmax2,r14\n\tmov.l .Lp_k_g17b,r7\n\t"
    "mov r5,r4\n\t"
    "mov r4,r5\n\tshll8 r5\n\tor r13,r5\n\t"
    "mov r4,r6\n\tshll r6\n\tshlr16 r6\n\tshlr8 r6\n\tshll2 r6\n\tmov.l .Lp_k_gm2,r0\n\tadd r0,r6\n\t"
    PA_ELEMENT(0,0,36,".Lpe_fail")
    "mov.l @(52,r15),r1\n\tmov.l @(36,r15),r0\n\tmov.l r0,@r1\n\tmov.l @r15,r0\n\tmov.l r0,@(4,r1)\n\t"
    "mov #1,r0\n"
".Lpe_exit:\n\t"
    "add #56,r15\n\t"
    "lds.l @r15+,macl\n\tlds.l @r15+,mach\n\t"
    "mov.l @r15+,r14\n\tmov.l @r15+,r13\n\tmov.l @r15+,r10\n\t"
    "rts\n\tmov.l @r15+,r8\n"
".Lpe_fail:\n\t"
    "bra .Lpe_exit\n\tmov #0,r0\n"
"    .size _sh2zi_pose_test_element,.-_sh2zi_pose_test_element\n"
"    .align 2\n"
".Lp_k_fin:\n\t.long 0xFEFFFFFF\n"
".Lp_k_2p30:\n\t.long 0x40000000\n"
".Lp_k_2p28:\n\t.long 0x10000000\n"
".Lp_k_g145:\n\t.long _pose_g+580\n"
".Lp_k_2p23:\n\t.long 0x00800000\n"
".Lp_k_g3:\n\t.long _pose_g+12\n"
".Lp_k_gm29:\n\t.long _pose_g-116\n"
".Lp_k_g113:\n\t.long _pose_g+452\n"
".Lp_k_g263:\n\t.long _pose_g+1052\n"
".Lp_k_g258:\n\t.long _pose_g+1032\n"
".Lp_k_bmax2:\n\t.long 0x7F0000A8\n"
".Lp_k_g17b:\n\t.long _pose_g+68\n"
".Lp_k_gm2:\n\t.long _pose_g-492\n"
"    .popsection\n");
#undef PA_ROT
#undef PA_ROTF
#undef PF_Q14P
#undef PA_DELTA
#undef PA_ELEMENT
#undef PA_AXIS
#undef PA_STUB
#undef PF_MAC3
#undef PF_ROUND
#undef PF_SHR30
#undef PF_R
#undef PF_GRAM
#undef PF_EYE
#undef PF_Q14
#undef PF_COPY
#undef PF_STAGED
#undef PF_ROW
#undef PD_DIV
#undef PD_DIV7
#define POSE_C_ENTRY sh2zi_pose_from_f32_c
#else
#define POSE_C_ENTRY sh2z_pose_from_f32
#endif

/* The C procedure (every target; on SH-2 the path for SR.S set). */
bool POSE_C_ENTRY(sh2z_pose_t *out,const sh2z_view_t *base,
    const int32_t rotation[3][3],const float basis[3][3],const float scale[3],
    const float position[3],const float eye[3],unsigned flags) {
    if(!out || !base || !rotation || !basis || !scale || !position || !eye ||
       (flags&~SH2Z_POSE_REVERSE_AXES))return false;
    if(base->focal_x<=0 || base->focal_y<=0 || base->near_z<=0 || base->far_z<=base->near_z ||
       base->clip[0]>base->clip[2] || base->clip[1]>base->clip[3])return false;
    for(unsigned i=0;i<3;++i)for(unsigned k=0;k<3;++k)
        if((uint32_t)rotation[i][k]+POSE_ROTATION_MAX>2u*POSE_ROTATION_MAX)return false;
    /* D: the model origin relative to the eye, Q16. */
    int32_t w[POSE_W_WORDS];
    for(unsigned k=0;k<3;++k) {
        const uint32_t a=pose_bits(position+k),b=pose_bits(eye+k);
        if((a<<1)>=UINT32_C(0xFF000000) || (b<<1)>=UINT32_C(0xFF000000) || !pose_delta(a,b,w+POSE_W_D+k))return false;
    }
    /* Per model axis j: B (Q30) and M (Q16) as rows over world k, and I. */
    uint32_t last_bits=0,last_inv=0;
    const float *rows=basis[0],*scales=scale;
    int step=3;
    if(flags&SH2Z_POSE_REVERSE_AXES){rows=basis[2];scales=scale+2;step=-3;}
    for(unsigned j=0;j<3;++j,rows+=step,scales+=step/3) {
        const uint32_t s=pose_bits(scales);
        if(s-POSE_SCALE_MIN>POSE_SCALE_SPAN)return false;
        const uint32_t ms=(s&UINT32_C(0x7FFFFF))|UINT32_C(0x800000),es=s>>23;
        if(s!=last_bits){last_bits=s;last_inv=pose_inverse(ms,es);}
        w[POSE_W_INV+j]=(int32_t)last_inv;
        const uint32_t ms8=ms<<8;
        const uint32_t *gm=pose_g+es-123u;
        for(unsigned k=0;k<3;++k) {
            const uint32_t b=pose_bits(rows+k);
            if((b<<1)>POSE_BASIS_MAX2)return false;
            pose_element(b,ms8,gm,w+POSE_W_B+3*j+k,w+POSE_W_M+3*j+k);
        }
    }
    return pose_finish(out,base,rotation,w);
}
