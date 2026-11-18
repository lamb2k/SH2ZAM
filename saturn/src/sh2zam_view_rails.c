/* view-rails-1: both rails of a strip quad in one call, with the view
 * validated once per frame (sh2z_view_rails_prepare). Each stepped run is
 * byte for byte what sh2z_view_edge_steps_center writes. The per-rail setup
 * reaches the same fixed-point steps with one reciprocal of den, computed
 * once per rail, instead of twelve 64-bit divisions. On SH-2 the _divu
 * entry is hand-written assembly: the setup, then one loop that steps and
 * projects the points of both rails with the view-center-1 kernel
 * (sh2zam_view_asm.h). MIT; SH2ZAM Contributors. */
#include "sh2zam_view_internal.h"
#include "sh2zam_view_center_internal.h"

#define RAILS_CHECK UINT32_C(0x7261696C)
_Static_assert(sizeof(sh2z_view_rails_t)==112,"sh2z_view_rails_t is 112 bytes");
/* The SH-2 entry (below) reads the handle's check word at 108 and its
 * kernel words at 68, and writes 20-byte vertices. */
_Static_assert(offsetof(sh2z_view_rails_t,kernel)==68 && offsetof(sh2z_view_rails_t,check)==108 &&
    sizeof(sh2z_view_vertex_t)==20,"rails layout for the SH-2 entry");

/* ---- the stepper's fixed point, without 64-bit division ------------------
 *
 * sh2z_view_edge_steps_center steps each axis with (sh2zam_view_internal.h)
 *     F = floor(mag * first * 2^16 / den),  S = floor(mag * step * 2^16 / den)
 *     P(k) = from +/- floor((F + k*S) / 2^16)        (modulo 2^32)
 * where mag = |to - from| < 2^32 and 1 <= first, step < den <= 65535 here.
 *
 * One multiplier per fraction and rail: with G = ceil(first * 2^64 / den),
 * which is below 2^64 because first < den,
 *     mag * G / 2^48 = V + mag * e / 2^48,   V = mag * first * 2^16 / den,
 * for some 0 <= e < 1. V's fraction is j/den with j <= den - 1, and
 * mag * e / 2^48 < 2^32 / 2^48 = 2^-16 < 1/den, so floor(mag * G / 2^48) is
 * floor(V) = F exactly, for every mag < 2^32. With G = Gh * 2^32 + Gl the
 * 96-bit product is mag*Gh * 2^32 + mag*Gl; the stepper wants
 *     floor(F / 2^16) = bits 64 .. 95,   (F mod 2^16) << 16 = bits 48 .. 63
 * (bits 32 .. 47 are dropped). Likewise S with step.
 *
 * G from one reciprocal R = floor((2^32 - 1) / den) and rho = (2^32 - 1)
 * - R*den, so 2^32 = R*den + rho + 1 and, for 0 <= a < den,
 *     a * 2^32 / den = a*R + a * (rho + 1) / den,   a * (rho + 1) < den^2 < 2^32.
 * So Gh = floor(first * 2^32 / den) = first*R + q with q, p = the quotient
 * and remainder of first * (rho + 1) / den, and Gl = ceil(p * 2^32 / den) =
 * p*R + ceil(p * (rho + 1) / den); both are below 2^32.
 * The small divisions use R too: for n < 2^32, 0 <= n/den - n*R/2^32 <=
 * n/2^32 < 1, since 2^32/den - R <= (den - 1)/den + 1/den = 1. So
 * mulhi(n, R) is floor(n/den) or one less, and a remainder test corrects it. */
static SH2ZI_ALWAYS_INLINE uint32_t mulhi(uint32_t a,uint32_t b) {
    return (uint32_t)(((uint64_t)a*b)>>32);
}
/* floor(n / d) and its remainder, for 2 <= d and r = floor((2^32 - 1) / d). */
static SH2ZI_ALWAYS_INLINE uint32_t qdiv(uint32_t n,uint32_t d,uint32_t r,uint32_t *rem) {
    uint32_t q=mulhi(n,r),m=n-q*d;
    if(m>=d){++q;m-=d;}
    *rem=m;
    return q;
}
/* G = ceil(a * 2^64 / den) for 0 <= a < den, as g[0] = Gh, g[1] = Gl. */
static void ratio_g(uint32_t g[2],uint32_t a,uint32_t den,uint32_t r,uint32_t rho1) {
    uint32_t p,unused;
    g[0]=a*r+qdiv(a*rho1,den,r,&p);
    g[1]=p*r+qdiv(p*rho1+den-1u,den,r,&unused);
}

/* A run's stepping state, as the SH-2 loop reads it (14 words):
 *   [0..2]  f   fraction of x, y, z in the top 16 bits
 *   [3..5]  W   the next point's x, y, z
 *   [6..8]  sf  step fraction, top 16 bits
 *   [9..11] Sw  whole step (modulo 2^32)
 *   [12]    the next run's count (0 ends the call), [13] its output.
 * W, f is the accumulator F + k*S written as floor(./2^16) + from and
 * (. mod 2^16) << 16, so one ADDC carries the fraction into the whole part.
 * Towards smaller values (to < from) the stepper subtracts; there the
 * accumulator is kept as ~(F + k*S) = -(F + k*S) - 1, whose floor/2^16 is
 * -floor((F + k*S)/2^16) - 1, so W = from - floor((F + k*S)/2^16) still,
 * and the step is the 64-bit negation of (Sw, sf). */
enum { RUN_WORDS=14 };
/* bits 64 .. 95 and 48 .. 63 (as the top half) of mag * (gh * 2^32 + gl). */
static SH2ZI_ALWAYS_INLINE uint32_t frac_mul(uint32_t mag,uint32_t gh,uint32_t gl,uint32_t *top) {
    const uint64_t a=(uint64_t)mag*gh;
    const uint32_t lo=(uint32_t)a,mid=lo+mulhi(mag,gl);
    *top=mid&UINT32_C(0xFFFF0000);
    return (uint32_t)(a>>32)+(mid<lo?1u:0u);
}
static SH2ZI_ALWAYS_INLINE void axis_state(uint32_t *st,int32_t from,int32_t to,const uint32_t g[4]) {
    const bool negative=to<from;
    const uint32_t mag=negative?(uint32_t)from-(uint32_t)to:(uint32_t)to-(uint32_t)from;
    uint32_t ff,sf;
    const uint32_t wf=frac_mul(mag,g[0],g[1],&ff);
    uint32_t sw=frac_mul(mag,g[2],g[3],&sf);
    if(negative) {
        st[0]=~ff&UINT32_C(0xFFFF0000);st[3]=(uint32_t)from-wf;
        sw=(uint32_t)0-sw-(sf?1u:0u);sf=(uint32_t)0-sf;
    } else {
        st[0]=ff;st[3]=(uint32_t)from+wf;
    }
    st[6]=sf;st[9]=sw;
}
static void run_state(uint32_t *st,const sh2z_view_vertex_t *from,const sh2z_view_vertex_t *to,const uint32_t g[4]) {
    axis_state(st,from->x,to->x,g);
    axis_state(st+1,from->y,to->y,g);
    axis_state(st+2,from->z,to->z,g);
}
static SH2ZI_ALWAYS_INLINE int32_t as_int32(uint32_t a) {
    return a<=(uint32_t)INT32_MAX?(int32_t)a:(int32_t)(a-UINT32_C(0x80000000))-INT32_MAX-1;
}
/* Field by field: a struct assignment this size is a libgcc block copy
 * (___movmem) on SH-2, and the library calls no copy helper. */
static void copy_view(sh2z_view_center_t *d,const sh2z_view_center_t *s) {
    for(unsigned i=0;i<3;++i) {
        for(unsigned j=0;j<3;++j)d->view.r[i][j]=s->view.r[i][j];
        d->view.t[i]=s->view.t[i];
    }
    d->view.focal_x=s->view.focal_x;d->view.focal_y=s->view.focal_y;
    d->view.center_x=s->view.center_x;d->view.center_y=s->view.center_y;
    d->view.near_z=s->view.near_z;d->view.far_z=s->view.far_z;
    for(unsigned i=0;i<4;++i)d->view.clip[i]=s->view.clip[i];
    d->center_x16=s->center_x16;d->center_y16=s->center_y16;
}
static void run_portable(sh2z_view_vertex_t *out,const sh2z_view_center_t *v,uint32_t *st,unsigned n) {
    sh2z_view_center_t c;
    copy_view(&c,v);
    for(unsigned k=0;k<n;++k) {
        out[k]=project_point(&c,as_int32(st[3]),as_int32(st[4]),as_int32(st[5]));
        for(unsigned a=0;a<3;++a) {
            const uint32_t f=st[a]+st[6+a];
            st[3+a]+=st[9+a]+(f<st[a]?1u:0u);
            st[a]=f;
        }
    }
}

#if SH2Z_DIVU_NATIVE
/* view-center-1's projection loop with the stepper in front of it (see
 * sh2zam_view_asm.h, rails=1): out, the camera block (written: its code
 * word and scratch record), the first run's state and its count (> 0). */
void sh2zi_view_divu_rails_center(sh2z_view_vertex_t *out,uint32_t *block,uint32_t *state,size_t count);
_Static_assert(offsetof(divu_block_t,near_z)==44 && offsetof(divu_block_t,fx)==48 &&
    offsetof(divu_block_t,far_z)==56 && sizeof(divu_block_t)==84,"camera block words");

__asm__(
#include "sh2zam_view_asm.h"
/* One point of a run: x y z = W, stored, then W, f += Sw, sf per axis. */
"    .macro SH2ZI_VR_STEP\n"
"    mov.l   @(12,r5),r11\n"
"    mov.l   @r5,r0\n"
"    mov.l   @(24,r5),r1\n"
"    clrt\n"
"    addc    r1,r0\n"
"    mov.l   r0,@r5\n"
"    mov.l   @(36,r5),r1\n"
"    addc    r11,r1\n"
"    mov.l   r1,@(12,r5)\n"
"    mov.l   @(16,r5),r12\n"
"    mov.l   @(4,r5),r0\n"
"    mov.l   @(28,r5),r1\n"
"    clrt\n"
"    addc    r1,r0\n"
"    mov.l   r0,@(4,r5)\n"
"    mov.l   @(40,r5),r1\n"
"    addc    r12,r1\n"
"    mov.l   r1,@(16,r5)\n"
"    mov.l   @(20,r5),r13\n"
"    mov.l   @(8,r5),r0\n"
"    mov.l   @(32,r5),r1\n"
"    clrt\n"
"    addc    r1,r0\n"
"    mov.l   r0,@(8,r5)\n"
"    mov.l   @(44,r5),r1\n"
"    addc    r13,r1\n"
"    mov.l   r1,@(20,r5)\n"
"    mov.l   r11,@r4\n"
"    mov.l   r12,@(4,r4)\n"
"    mov.l   r13,@(8,r4)\n"
"    .endm\n"
"    .text\n"
"    .align 2\n"
"    .global _sh2zi_view_divu_rails_center\n"
"    .type _sh2zi_view_divu_rails_center,@function\n"
"_sh2zi_view_divu_rails_center:\n"
"    SH2ZI_VD_PROLOGUE 1\n"
".Lsh2zi_vd_loopRC:\n"
"    SH2ZI_VR_STEP\n"
"    SH2ZI_VD_REST RC, 0, 1, 1\n"
"    .size _sh2zi_view_divu_rails_center,.-_sh2zi_view_divu_rails_center\n"
/* sh2z_view_edge_rails_center_divu: the C path's rails() (below) in
 * assembly, then one call of the loop above for both runs.
 *
 * Registers on entry: r4 out_l, r5 out_r, r6 handle, r7 l_from; r14 points
 * at the stack arguments: @0 l_to, @4 r_from, @8 r_to, @12 boundary,
 * @16 count, @20 l_low, @24 l_high, @28 r_low, @32 r_high. The frame
 * (260 bytes at r15) holds the left and right rail at @0 and @24 (outF =
 * out + f, n, outE = out + e, den, first, step), the handle at @48 (later
 * the shared-span flag), l_from at @52, both run states at @56 and the
 * camera block at @168.
 *
 * SH2ZI_VR_SCAN: one rail's f and e as in run_of, the argument rules, and
 * its slot; r11 boundary, r10 boundary + count, uses r0 .. r7. Failing
 * rules branch to .Lsh2zi_vr_false before anything is written.
 * SH2ZI_VR_FILL: the copies of from below outF and of to from outE on; r9
 * is count * 20.
 * SH2ZI_VR_G: G = ceil(a * 2^64 / den) as gh, gl (the proof above), from
 * r1 den, r2 R, r3 rho + 1, r4 den - 1; qdiv is the mulhi, multiply-back
 * and one correction. Uses r5 .. r7.
 * SH2ZI_VR_AXIS: axis_state for the axis at byte offset off: r4 from, r5
 * to, r12 state, r8 r9 first's G, r10 r11 step's G, r13 0xFFFF0000, r6 0.
 *
 * The divider computes R for the first rail with points while the ends
 * are copied, and the right rail's R (unless it spans the same low and
 * high, which gives the same G) while the left state is built. */
"    .macro SH2ZI_VR_SCAN low, high, slot, out\n"
"    mov.l   @(\\low,r14),r2\n"
"    mov.l   @(\\high,r14),r3\n"
"    mov     r11,r7\n"
"1:  cmp/hs  r10,r7\n"
"    bt      2f\n"
"    mov.l   @r7,r0\n"
"    cmp/gt  r2,r0\n"
"    bf/s    1b\n"
"    add     #4,r7\n"
"    add     #-4,r7\n"
"2:  mov     r10,r6\n"
"3:  cmp/hi  r7,r6\n"
"    bf      4f\n"
"    add     #-4,r6\n"
"    mov.l   @r6,r0\n"
"    cmp/ge  r3,r0\n"
"    bt      3b\n"
"    add     #4,r6\n"
"4:  mov     r7,r0\n"
"    sub     r11,r0\n"
"    mov     r0,r1\n"
"    shll2   r1\n"
"    add     r0,r1\n"
"    add     \\out,r1\n"
"    mov.l   r1,@(\\slot,r15)\n"
"    mov     r6,r0\n"
"    sub     r11,r0\n"
"    mov     r0,r1\n"
"    shll2   r1\n"
"    add     r0,r1\n"
"    add     \\out,r1\n"
"    mov.l   r1,@(\\slot+8,r15)\n"
"    mov     r6,r5\n"
"    sub     r7,r5\n"
"    shlr2   r5\n"
"    tst     r5,r5\n"
"    bt/s    9f\n"
"    mov.l   r5,@(\\slot+4,r15)\n"
"    cmp/gt  r2,r3\n"
"    bf      7f\n"
"    sub     r2,r3\n"
"    mov     #-1,r0\n"
"    extu.w  r0,r0\n"
"    cmp/hi  r0,r3\n"
"    bt      7f\n"
"    mov.l   @r7,r1\n"
"    sub     r2,r1\n"
"    cmp/hs  r3,r1\n"
"    bt      7f\n"
"    mov.l   r3,@(\\slot+12,r15)\n"
"    mov.l   r1,@(\\slot+16,r15)\n"
"    dt      r5\n"
"    bt/s    8f\n"
"    mov     #0,r0\n"
"    mov.l   @(4,r7),r0\n"
"    mov.l   @r7,r4\n"
"    sub     r4,r0\n"
"    mov     r0,r4\n"
"    add     #-1,r4\n"
"    mov     r3,r7\n"
"    add     #-1,r7\n"
"    cmp/hs  r7,r4\n"
"    bt      7f\n"
"    add     #-4,r6\n"
"    mov.l   @r6,r6\n"
"    sub     r2,r6\n"
"    cmp/hs  r3,r6\n"
"    bt      7f\n"
"    cmp/hi  r1,r6\n"
"    bf      7f\n"
"    sub     r1,r6\n"
"    mul.l   r5,r0\n"
"    sts     macl,r4\n"
"    cmp/eq  r4,r6\n"
"    bf      7f\n"
"8:  bra     9f\n"
"    mov.l   r0,@(\\slot+20,r15)\n"
"7:  bra     .Lsh2zi_vr_false\n"
"    nop\n"
"9:\n"
"    .endm\n"
"    .macro SH2ZI_VR_FILL out, slot, foff, fbase, toff, tbase\n"
"    mov.l   @(\\slot,r15),r2\n"
"    mov     \\out,r1\n"
"    cmp/eq  r1,r2\n"
"    bt      2f\n"
"    mov.l   @(\\foff,\\fbase),r0\n"
"    mov.l   @r0,r3\n"
"    mov.l   @(4,r0),r4\n"
"    mov.l   @(8,r0),r5\n"
"    mov.l   @(12,r0),r6\n"
"    mov.l   @(16,r0),r7\n"
"1:  mov.l   r3,@r1\n"
"    mov.l   r4,@(4,r1)\n"
"    mov.l   r5,@(8,r1)\n"
"    mov.l   r6,@(12,r1)\n"
"    mov.l   r7,@(16,r1)\n"
"    add     #20,r1\n"
"    cmp/eq  r1,r2\n"
"    bf      1b\n"
"2:  mov.l   @(\\slot+8,r15),r1\n"
"    mov     \\out,r2\n"
"    add     r9,r2\n"
"    cmp/eq  r1,r2\n"
"    bt      4f\n"
"    mov.l   @(\\toff,\\tbase),r0\n"
"    mov.l   @r0,r3\n"
"    mov.l   @(4,r0),r4\n"
"    mov.l   @(8,r0),r5\n"
"    mov.l   @(12,r0),r6\n"
"    mov.l   @(16,r0),r7\n"
"3:  mov.l   r3,@r1\n"
"    mov.l   r4,@(4,r1)\n"
"    mov.l   r5,@(8,r1)\n"
"    mov.l   r6,@(12,r1)\n"
"    mov.l   r7,@(16,r1)\n"
"    add     #20,r1\n"
"    cmp/eq  r1,r2\n"
"    bf      3b\n"
"4:\n"
"    .endm\n"
"    .macro SH2ZI_VR_G a, gh, gl\n"
"    mul.l   \\a,r2\n"
"    sts     macl,\\gh\n"
"    mul.l   \\a,r3\n"
"    sts     macl,r5\n"
"    dmulu.l r5,r2\n"
"    sts     mach,r6\n"
"    mul.l   r6,r1\n"
"    sts     macl,r7\n"
"    sub     r7,r5\n"
"    cmp/hs  r1,r5\n"
"    bf      1f\n"
"    add     #1,r6\n"
"    sub     r1,r5\n"
"1:  add     r6,\\gh\n"
"    mul.l   r5,r2\n"
"    sts     macl,\\gl\n"
"    mul.l   r5,r3\n"
"    sts     macl,r5\n"
"    add     r4,r5\n"
"    dmulu.l r5,r2\n"
"    sts     mach,r6\n"
"    mul.l   r6,r1\n"
"    sts     macl,r7\n"
"    sub     r7,r5\n"
"    cmp/hs  r1,r5\n"
"    bf      2f\n"
"    add     #1,r6\n"
"2:  add     r6,\\gl\n"
"    .endm\n"
"    .macro SH2ZI_VR_PRODUCTS gh, gl\n"
"    dmulu.l r3,\\gh\n"
"    sts     macl,r0\n"
"    sts     mach,r1\n"
"    dmulu.l r3,\\gl\n"
"    sts     mach,r7\n"
"    clrt\n"
"    addc    r7,r0\n"
"    addc    r6,r1\n"
"    .endm\n"
"    .macro SH2ZI_VR_AXIS off\n"
"    mov.l   @(\\off,r5),r1\n"
"    mov.l   @(\\off,r4),r2\n"
"    mov     r1,r3\n"
"    cmp/gt  r1,r2\n"
"    bt/s    5f\n"
"    sub     r2,r3\n"
"    SH2ZI_VR_PRODUCTS r8, r9\n"
"    and     r13,r0\n"
"    mov.l   r0,@(\\off,r12)\n"
"    add     r2,r1\n"
"    mov.l   r1,@(12+\\off,r12)\n"
"    SH2ZI_VR_PRODUCTS r10, r11\n"
"    and     r13,r0\n"
"    mov.l   r0,@(24+\\off,r12)\n"
"    bra     6f\n"
"    mov.l   r1,@(36+\\off,r12)\n"
"5:  neg     r3,r3\n"
"    SH2ZI_VR_PRODUCTS r8, r9\n"
"    not     r0,r0\n"
"    and     r13,r0\n"
"    mov.l   r0,@(\\off,r12)\n"
"    sub     r1,r2\n"
"    mov.l   r2,@(12+\\off,r12)\n"
"    SH2ZI_VR_PRODUCTS r10, r11\n"
"    and     r13,r0\n"
"    clrt\n"
"    negc    r0,r0\n"
"    negc    r1,r1\n"
"    mov.l   r0,@(24+\\off,r12)\n"
"    mov.l   r1,@(36+\\off,r12)\n"
"6:\n"
"    .endm\n"
"    .text\n"
"    .align 2\n"
"    .global _sh2z_view_edge_rails_center_divu\n"
"    .type _sh2z_view_edge_rails_center_divu,@function\n"
"_sh2z_view_edge_rails_center_divu:\n"
"    mov.l   r8,@-r15\n"
"    mov.l   r9,@-r15\n"
"    mov.l   r10,@-r15\n"
"    mov.l   r11,@-r15\n"
"    mov.l   r12,@-r15\n"
"    mov.l   r13,@-r15\n"
"    mov.l   r14,@-r15\n"
"    sts.l   pr,@-r15\n"
"    mov     r15,r14\n"
"    add     #32,r14\n"
"    mov.w   .Lsh2zi_vr_frame,r0\n"
"    sub     r0,r15\n"
"    tst     r6,r6\n"
"    bt      .Lsh2zi_vr_false\n"
"    mov     #108,r0\n"
"    mov.l   @(r0,r6),r1\n"
"    mov.l   .Lsh2zi_vr_check,r2\n"
"    cmp/eq  r1,r2\n"
"    bf      .Lsh2zi_vr_false\n"
"    mov.l   @(16,r14),r8\n"
"    tst     r8,r8\n"
"    bt      .Lsh2zi_vr_true\n"
"    tst     r4,r4\n"
"    bt      .Lsh2zi_vr_false\n"
"    tst     r5,r5\n"
"    bt      .Lsh2zi_vr_false\n"
"    tst     r7,r7\n"
"    bt      .Lsh2zi_vr_false\n"
"    mov.l   @r14,r0\n"
"    tst     r0,r0\n"
"    bt      .Lsh2zi_vr_false\n"
"    mov.l   @(4,r14),r0\n"
"    tst     r0,r0\n"
"    bt      .Lsh2zi_vr_false\n"
"    mov.l   @(8,r14),r0\n"
"    tst     r0,r0\n"
"    bt      .Lsh2zi_vr_false\n"
"    mov.l   @(12,r14),r11\n"
"    tst     r11,r11\n"
"    bt      .Lsh2zi_vr_false\n"
"    mov     #1,r0\n"
"    shll16  r0\n"
"    cmp/hi  r0,r8\n"
"    bt      .Lsh2zi_vr_false\n"
"    mov     r8,r9\n"
"    shll2   r9\n"
"    add     r8,r9\n"
"    shll2   r9\n"
"    not     r9,r0\n"
"    cmp/hi  r0,r4\n"
"    bt      .Lsh2zi_vr_false\n"
"    cmp/hi  r0,r5\n"
"    bt      .Lsh2zi_vr_false\n"
"    mov     r5,r0\n"
"    add     r9,r0\n"
"    cmp/hi  r4,r0\n"
"    bf      1f\n"
"    mov     r4,r0\n"
"    add     r9,r0\n"
"    cmp/hi  r5,r0\n"
"    bt      .Lsh2zi_vr_false\n"
"1:  bra     .Lsh2zi_vr_scan\n"
"    mov     r4,r12\n"
".Lsh2zi_vr_true:\n"
"    bra     .Lsh2zi_vr_exit\n"
"    mov     #1,r0\n"
".Lsh2zi_vr_false:\n"
"    mov     #0,r0\n"
".Lsh2zi_vr_exit:\n"
"    mov.w   .Lsh2zi_vr_frame,r1\n"
"    add     r1,r15\n"
"    lds.l   @r15+,pr\n"
"    mov.l   @r15+,r14\n"
"    mov.l   @r15+,r13\n"
"    mov.l   @r15+,r12\n"
"    mov.l   @r15+,r11\n"
"    mov.l   @r15+,r10\n"
"    mov.l   @r15+,r9\n"
"    rts\n"
"    mov.l   @r15+,r8\n"
"    .align 2\n"
".Lsh2zi_vr_check:\n"
"    .long   0x7261696C\n"
".Lsh2zi_vr_frame:\n"
"    .short  260\n"
".Lsh2zi_vr_scan:\n"
"    mov     r5,r13\n"
"    mov.l   r6,@(48,r15)\n"
"    mov.l   r7,@(52,r15)\n"
"    mov     r8,r10\n"
"    shll2   r10\n"
"    add     r11,r10\n"
"    SH2ZI_VR_SCAN 20, 24, 0, r12\n"
"    SH2ZI_VR_SCAN 28, 32, 24, r13\n"
"    mov     #-1,r8\n"
"    shll8   r8\n"
"    mov.l   @(4,r15),r0\n"
"    tst     r0,r0\n"
"    bf/s    1f\n"
"    mov.l   @(12,r15),r1\n"
"    mov.l   @(28,r15),r0\n"
"    tst     r0,r0\n"
"    bt/s    2f\n"
"    mov.l   @(36,r15),r1\n"
"1:  mov     #0,r0\n"
"    mov.l   r1,@r8\n"
"    mov.l   r0,@(16,r8)\n"
"    mov     #-1,r0\n"
"    mov.l   r0,@(20,r8)\n"
"2:  SH2ZI_VR_FILL r12, 0, 52, r15, 0, r14\n"
"    SH2ZI_VR_FILL r13, 24, 4, r14, 8, r14\n"
"    mov.l   @(4,r15),r10\n"
"    mov.l   @(28,r15),r11\n"
"    mov     r10,r0\n"
"    or      r11,r0\n"
"    tst     r0,r0\n"
"    bf      1f\n"
"    bra     .Lsh2zi_vr_true\n"
"    nop\n"
"1:\n"
"    mov.l   @(48,r15),r0\n"
"    add     #68,r0\n"
"    mov     r15,r1\n"
"    add     #127,r1\n"
"    add     #41,r1\n"
"    mov.l   @r0,r2\n"
"    mov.l   r2,@(16,r1)\n"
"    mov.l   @(4,r0),r2\n"
"    mov.l   r2,@(20,r1)\n"
"    mov.l   @(8,r0),r2\n"
"    mov.l   r2,@(24,r1)\n"
"    mov.l   @(12,r0),r2\n"
"    mov.l   r2,@(28,r1)\n"
"    mov.l   @(16,r0),r2\n"
"    mov.l   r2,@(44,r1)\n"
"    mov.l   @(20,r0),r2\n"
"    mov.l   r2,@(48,r1)\n"
"    mov.l   @(24,r0),r2\n"
"    mov.l   r2,@(52,r1)\n"
"    mov.l   @(28,r0),r2\n"
"    mov.l   r2,@(56,r1)\n"
"    add     #60,r1\n"
"    mov.l   @(32,r0),r2\n"
"    mov.l   r2,@(24,r1)\n"
"    mov.l   @(36,r0),r2\n"
"    mov.l   r2,@(28,r1)\n"
"    mov     r15,r12\n"
"    add     #56,r12\n"
"    tst     r10,r10\n"
"    bt      3f\n"
"    mov.l   @(20,r8),r2\n"
"    mov.l   @(16,r8),r3\n"
"    tst     r11,r11\n"
"    bt/s    1f\n"
"    mov     #1,r0\n"
"    mov.l   @(20,r14),r0\n"
"    mov.l   @(28,r14),r1\n"
"    cmp/eq  r0,r1\n"
"    bf      4f\n"
"    mov.l   @(24,r14),r0\n"
"    mov.l   @(32,r14),r1\n"
"    cmp/eq  r0,r1\n"
"    bt/s    1f\n"
"    mov     #1,r0\n"
"4:  mov.l   @(36,r15),r1\n"
"    mov     #0,r0\n"
"    mov.l   r1,@r8\n"
"    mov.l   r0,@(16,r8)\n"
"    mov     #-1,r1\n"
"    mov.l   r1,@(20,r8)\n"
"1:  mov.l   r0,@(48,r15)\n"
"    mov.l   @(12,r15),r1\n"
"    mov.l   @(16,r15),r0\n"
"    bsr     .Lsh2zi_vr_ratio\n"
"    mov.l   @(20,r15),r4\n"
"    mov.l   @(52,r15),r4\n"
"    bsr     .Lsh2zi_vr_state\n"
"    mov.l   @r14,r5\n"
"    mov.l   @(28,r15),r0\n"
"    mov.l   @(24,r15),r1\n"
"    mov.l   r0,@(48,r12)\n"
"    tst     r0,r0\n"
"    bt/s    5f\n"
"    mov.l   r1,@(52,r12)\n"
"    add     #56,r12\n"
"    mov.l   @(48,r15),r0\n"
"    tst     r0,r0\n"
"    bf      6f\n"
"    mov     #-1,r8\n"
"    shll8   r8\n"
"    mov.l   @(20,r8),r2\n"
"    mov.l   @(16,r8),r3\n"
"    mov.l   @(36,r15),r1\n"
"    mov.l   @(40,r15),r0\n"
"    bsr     .Lsh2zi_vr_ratio\n"
"    mov.l   @(44,r15),r4\n"
"6:  mov.l   @(4,r14),r4\n"
"    bsr     .Lsh2zi_vr_state\n"
"    mov.l   @(8,r14),r5\n"
"    mov     #0,r0\n"
"    mov.l   r0,@(48,r12)\n"
"5:  mov.l   @(0,r15),r4\n"
"    bra     7f\n"
"    mov.l   @(4,r15),r7\n"
"3:  mov.l   @(20,r8),r2\n"
"    mov.l   @(16,r8),r3\n"
"    mov.l   @(36,r15),r1\n"
"    mov.l   @(40,r15),r0\n"
"    bsr     .Lsh2zi_vr_ratio\n"
"    mov.l   @(44,r15),r4\n"
"    mov.l   @(4,r14),r4\n"
"    bsr     .Lsh2zi_vr_state\n"
"    mov.l   @(8,r14),r5\n"
"    mov     #0,r0\n"
"    mov.l   r0,@(48,r12)\n"
"    mov.l   @(24,r15),r4\n"
"    mov.l   @(28,r15),r7\n"
"7:  mov     r15,r6\n"
"    add     #56,r6\n"
"    mov     r6,r5\n"
"    add     #112,r5\n"
"    mov.l   .Lsh2zi_vr_kernel,r0\n"
"    jsr     @r0\n"
"    nop\n"
"    bra     .Lsh2zi_vr_true\n"
"    nop\n"
".Lsh2zi_vr_ratio:\n"
"    add     #1,r3\n"
"    mov.l   r4,@-r15\n"
"    mov     r1,r4\n"
"    add     #-1,r4\n"
"    SH2ZI_VR_G r0, r8, r9\n"
"    mov.l   @r15+,r0\n"
"    tst     r0,r0\n"
"    bt      .Lsh2zi_vr_nostep\n"
"    SH2ZI_VR_G r0, r10, r11\n"
"    rts\n"
"    nop\n"
".Lsh2zi_vr_nostep:\n"
"    mov     #0,r10\n"
"    rts\n"
"    mov     #0,r11\n"
".Lsh2zi_vr_state:\n"
"    mov     #-1,r13\n"
"    shll16  r13\n"
"    mov     #0,r6\n"
"    SH2ZI_VR_AXIS 0\n"
"    SH2ZI_VR_AXIS 4\n"
"    SH2ZI_VR_AXIS 8\n"
"    rts\n"
"    nop\n"
"    .align 2\n"
".Lsh2zi_vr_kernel:\n"
"    .long   _sh2zi_view_divu_rails_center\n"
"    .size _sh2z_view_edge_rails_center_divu,.-_sh2z_view_edge_rails_center_divu\n"
);

#endif

static SH2ZI_ALWAYS_INLINE uint32_t recip_portable(uint32_t den,uint32_t *rho) {
    const uint32_t q=UINT32_C(0xFFFFFFFF)/den;
    *rho=UINT32_C(0xFFFFFFFF)-q*den;
    return q;
}

/* One rail, as the loop it replaces scans it: copies of from at the front
 * while b <= low, copies of to at the back while b >= high, and the n
 * entries between stepped. Returns false on the argument rules. Only
 * b[f], b[f+1] and b[e-1] of the run are read. */
typedef struct { unsigned f,e; uint32_t den,first,step; } run_t;
static SH2ZI_ALWAYS_INLINE bool run_of(run_t *r,const int32_t *b,unsigned count,int32_t low,int32_t high) {
    unsigned f=0,e=count;
    while(f<count && b[f]<=low)++f;
    while(e>f && b[e-1]>=high)--e;
    r->f=f;r->e=e;r->step=0;
    if(e==f)return true;
    /* b[f] > low and b[e-1] < high. With high > low, den is exact, and
     * first < den (last < den) holds only if low < b < high. */
    const uint32_t den=(uint32_t)high-(uint32_t)low,first=(uint32_t)b[f]-(uint32_t)low;
    r->den=den;r->first=first;
    if(high<=low || den>65535u || first>=den)return false;
    if(e-f>1u) {
        /* (e - f - 1) * step < 2^16 * 2^16: no wrap. */
        const uint32_t step=(uint32_t)b[f+1]-(uint32_t)b[f],last=(uint32_t)b[e-1]-(uint32_t)low;
        if(step-1u>=den-1u || last>=den || last<=first || last-first!=(e-f-1u)*step)return false;
        r->step=step;
    }
    return true;
}
static void ratio(uint32_t g[4],const run_t *r,uint32_t rcp,uint32_t rho) {
    ratio_g(g,r->first,r->den,rcp,rho+1u);
    if(r->step)ratio_g(g+2,r->step,r->den,rcp,rho+1u);
    else g[2]=g[3]=0;
}
static SH2ZI_ALWAYS_INLINE void fill_ends(sh2z_view_vertex_t *out,unsigned count,const run_t *r,
    const sh2z_view_vertex_t *from,const sh2z_view_vertex_t *to) {
    for(unsigned j=0;j<r->f;++j)out[j]=*from;
    for(unsigned j=r->e;j<count;++j)out[j]=*to;
}

static bool rails(sh2z_view_vertex_t *out_l,sh2z_view_vertex_t *out_r,const sh2z_view_rails_t *h,
    const sh2z_view_vertex_t *l_from,const sh2z_view_vertex_t *l_to,
    const sh2z_view_vertex_t *r_from,const sh2z_view_vertex_t *r_to,
    const int32_t *boundary,unsigned count,int32_t l_low,int32_t l_high,int32_t r_low,int32_t r_high) {
    if(!h || h->check!=RAILS_CHECK)return false;
    if(!count)return true;
    if(!out_l || !out_r || !l_from || !l_to || !r_from || !r_to || !boundary || count>SH2Z_VIEW_RAILS_MAX_COUNT)return false;
    const size_t bytes=(size_t)count*sizeof(*out_l);
    if((uintptr_t)out_l>UINTPTR_MAX-bytes || (uintptr_t)out_r>UINTPTR_MAX-bytes || overlaps(out_l,bytes,out_r,bytes))return false;
    run_t L,R;
    if(!run_of(&L,boundary,count,l_low,l_high) || !run_of(&R,boundary,count,r_low,r_high))return false;
    /* boundary is not read again; the ends must not overlap the outputs. */
    fill_ends(out_l,count,&L,l_from,l_to);
    fill_ends(out_r,count,&R,r_from,r_to);
    const unsigned nl=L.e-L.f,nr=R.e-R.f;
    uint32_t st[RUN_WORDS],g[4],rcp,rho;
    if(nl) {
        rcp=recip_portable(L.den,&rho);
        ratio(g,&L,rcp,rho);
        run_state(st,l_from,l_to,g);
        run_portable(out_l+L.f,&h->view,st,nl);
    }
    if(nr) {
        /* A right rail over the same span has the same first, step, den. */
        if(!nl || l_low!=r_low || l_high!=r_high) {
            rcp=recip_portable(R.den,&rho);
            ratio(g,&R,rcp,rho);
        }
        run_state(st,r_from,r_to,g);
        run_portable(out_r+R.f,&h->view,st,nr);
    }
    return true;
}

bool sh2z_view_rails_prepare(sh2z_view_rails_t *out,const sh2z_view_center_t *v) {
    /* Shared helpers of sh2zam_view_internal.h this file does not need. */
    (void)batch_args_ok;(void)edge_axis;(void)clip_near;(void)project_args_ok;
    (void)guard_distances;(void)guard_pass;(void)guard_edge_ok;
#if SH2Z_DIVU_NATIVE
    (void)fill_block;
#endif
    if(!out || !center_valid(v))return false;
    copy_view(&out->view,v);
    for(unsigned i=0;i<10u;++i)out->kernel[i]=0;
#if SH2Z_DIVU_NATIVE
    /* Big-endian words at block offsets 16, 20, 24, 28 (centre floor and
     * clip rectangle), 44 .. 56 (near, focal x, focal y, far) and 84, 88
     * (the centre's fraction bits << 16): see divu_block_t. */
    const sh2z_view_t *c=&v->view;
    out->kernel[0]=(uint32_t)(uint16_t)c->center_x;
    out->kernel[1]=((uint32_t)(uint16_t)c->center_y<<16)|(uint16_t)c->clip[0];
    out->kernel[2]=((uint32_t)(uint16_t)c->clip[2]<<16)|(uint16_t)c->clip[1];
    out->kernel[3]=(uint32_t)(uint16_t)c->clip[3]<<16;
    out->kernel[4]=(uint32_t)c->near_z;out->kernel[5]=(uint32_t)c->focal_x;
    out->kernel[6]=(uint32_t)c->focal_y;out->kernel[7]=(uint32_t)c->far_z;
    out->kernel[8]=(uint32_t)v->center_x16<<16;out->kernel[9]=(uint32_t)v->center_y16<<16;
#endif
    out->check=RAILS_CHECK;
    return true;
}
bool sh2z_view_edge_rails_center(sh2z_view_vertex_t *out_l,sh2z_view_vertex_t *out_r,
    const sh2z_view_rails_t *rails_handle,
    const sh2z_view_vertex_t *l_from,const sh2z_view_vertex_t *l_to,
    const sh2z_view_vertex_t *r_from,const sh2z_view_vertex_t *r_to,
    const int32_t *boundary,unsigned count,
    int32_t l_low,int32_t l_high,int32_t r_low,int32_t r_high) {
    return rails(out_l,out_r,rails_handle,l_from,l_to,r_from,r_to,boundary,count,l_low,l_high,r_low,r_high);
}
#if !SH2Z_DIVU_NATIVE
/* On SH-2 this entry is the assembly above. */
bool sh2z_view_edge_rails_center_divu(sh2z_view_vertex_t *out_l,sh2z_view_vertex_t *out_r,
    const sh2z_view_rails_t *rails_handle,
    const sh2z_view_vertex_t *l_from,const sh2z_view_vertex_t *l_to,
    const sh2z_view_vertex_t *r_from,const sh2z_view_vertex_t *r_to,
    const int32_t *boundary,unsigned count,
    int32_t l_low,int32_t l_high,int32_t r_low,int32_t r_high) {
    return rails(out_l,out_r,rails_handle,l_from,l_to,r_from,r_to,boundary,count,l_low,l_high,r_low,r_high);
}
#endif
