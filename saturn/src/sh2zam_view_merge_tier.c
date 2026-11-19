/* view-merge-2: the tier texture merge planner, sh2z_view_merge_tier. It is
 * in a file of its own so that every existing object compiles exactly as
 * before.
 *
 * With equal-height repetitions the view-merge-1 error of interior boundary
 * k of a run of r is E = h * F (h rows per repetition), where
 *     F = r * (P[k] - P[0]) - k * (P[r] - P[0]),
 * and the pixel test |E| * 256 <= tol_q8 * r * h becomes
 * |F| <= floor(tol_q8 * r / 256) = tol_q8 >> (8 - log2 r). F needs only
 * subtractions and constant shifts, so a run test is a few dozen
 * instructions with no multiply and no 64-bit value.
 *
 * Copyright (c) 2026 SH2ZAM Contributors. MIT License.
 * Inspired by SH4ZAM by Falco Girgis and SH4ZAM Contributors.
 */
#include "sh2zam/view_experimental.h"

#if defined(__sh__) && defined(__SH2__) && (!defined(SH2Z_FORCE_PORTABLE) || !SH2Z_FORCE_PORTABLE)
#define SH2ZI_TIER_NATIVE 1
#else
#define SH2ZI_TIER_NATIVE 0
#endif

#if defined(__GNUC__)
#define TIER_INLINE inline __attribute__((always_inline))
#define TIER_NOINLINE __attribute__((noinline))
#else
#define TIER_INLINE inline
#define TIER_NOINLINE
#endif

/* Outcode bits that stop merging: NEAR, FAR and the two unused bits. */
#define TIER_BLOCKING 0xC3u
#define TIER_RUN(r) (UINT32_C(1)<<(r))

typedef struct { uint32_t lim,span; } tier_limit_t;

/* |f| <= lim as one unsigned compare: f + lim must lie in 0 .. 2 * lim.
 * Here |f| < 2^22 and lim <= 2^20, so f + lim never wraps into that range. */
static TIER_INLINE bool tier_within(int32_t f,tier_limit_t l) {
    return (uint32_t)f+l.lim<=l.span;
}
/* Run of 2: F = 2 * P[1] - P[0] - P[2]. */
static TIER_INLINE bool tier_rail2(const sh2z_view_vertex_t *p,tier_limit_t l2) {
    return tier_within(2*p[1].sx-p[0].sx-p[2].sx,l2) &&
           tier_within(2*p[1].sy-p[0].sy-p[2].sy,l2);
}
/* Run of 4, middle boundary first: F2 = 2 * m with m = 2 * (P[2] - P[0]) - D,
 * and |2 m| <= tol >> 6 exactly when |m| <= tol >> 7 (m is an integer);
 * F1 = 4 * (P[1] - P[0]) - D and F3 = 4 * (P[3] - P[4]) + D. */
static TIER_INLINE bool tier_rail4(const sh2z_view_vertex_t *p,tier_limit_t l4,tier_limit_t l2) {
    const int32_t x0=p[0].sx,y0=p[0].sy,x4=p[4].sx,y4=p[4].sy,dx=x4-x0,dy=y4-y0;
    if(!tier_within(2*(p[2].sx-x0)-dx,l2) || !tier_within(2*(p[2].sy-y0)-dy,l2))return false;
    return tier_within(4*(p[1].sx-x0)-dx,l4) && tier_within(4*(p[3].sx-x4)+dx,l4) &&
           tier_within(4*(p[1].sy-y0)-dy,l4) && tier_within(4*(p[3].sy-y4)+dy,l4);
}
/* Runs of 8 and 16: F(k) = F(k-1) + r * (P[k] - P[k-1]) - D, F(0) = 0. */
static TIER_INLINE bool tier_rail_long(const sh2z_view_vertex_t *p,int32_t r,tier_limit_t l) {
    const int32_t dx=p[r].sx-p[0].sx,dy=p[r].sy-p[0].sy;
    int32_t fx=0,fy=0;
    for(int32_t k=1;k<r;++k) {
        fx+=r*(p[k].sx-p[k-1].sx)-dx;
        fy+=r*(p[k].sy-p[k-1].sy)-dy;
        if(!tier_within(fx,l) || !tier_within(fy,l))return false;
    }
    return true;
}
static TIER_INLINE bool tier_blocked(const sh2z_view_vertex_t *left,const sh2z_view_vertex_t *right,unsigned j) {
    return ((left[j].outcode|right[j].outcode)&TIER_BLOCKING)!=0u;
}
static TIER_INLINE tier_limit_t tier_limit(uint32_t lim) {
    const tier_limit_t l={lim,2u*lim};
    return l;
}
static bool tier_span_overlaps(const void *a,size_t a_bytes,const void *b,size_t b_bytes) {
    const uintptr_t x=(uintptr_t)a,y=(uintptr_t)b;
    return x<y+b_bytes && y<x+a_bytes;
}

/* ---- SH-2 kernel: runs of 2 and 4 ------------------------------------- */

#if SH2ZI_TIER_NATIVE
_Static_assert(sizeof(sh2z_view_vertex_t)==20 && offsetof(sh2z_view_vertex_t,sx)==12 &&
    offsetof(sh2z_view_vertex_t,sy)==14 && offsetof(sh2z_view_vertex_t,outcode)==16,"vertex layout");
int sh2zi_merge_tier24(uint8_t *out,const sh2z_view_vertex_t *left,
    const sh2z_view_vertex_t *right,const uint32_t *ends,unsigned count,
    uint32_t run_mask,uint32_t tol_q8);
/* sh2zi_merge_tier24 (the arguments of sh2z_view_merge_tier, already
 * checked) returns the number of runs written: the plan of tier_plan for
 * run_mask {2, 4} or {2}, with or without end masks (two copies of the
 * body). Every test is one unsigned CMP/HI:
 *     run of 4:  4 P[k] - (2 (P[0] + P[4]) - L4) + (2 - k) D  in 0 .. S4
 *                that is F(k) + L4, with D = P[4] - P[0], for k = 2, 1, 3;
 *     run of 2:  4 P[1] - (2 (P[0] + P[2]) - L4)              in 0 .. S4
 *                that is 2 F + L4, and |2 F| <= L4 exactly when
 *                |F| <= L4 >> 1 = tol_q8 >> 7;
 * L4 = tol_q8 >> 6 and S4 = 2 L4. |F| < 2^19 and L4 <= 2^18: no wrap.
 *
 * Blocked boundaries: a scan, four boundaries per step with one long load
 * per rail each and a TST of their OR against 0xC3000000 (the outcode is
 * the long's top byte), finds the first blocked boundary after i; lim4e
 * and lim2e then stop runs of 4 and 2 before it, and the scan resumes after
 * it. The rail tests read only sx and sy.
 *
 * Registers: r4 out, r5 &left[i].sx, r6 &right[i].sx, r7 &ends[i], r8 L4,
 * r9 S4, r14 0xC3000000; a rail test walks r1 over its boundaries with r0,
 * r2, r3, r10-r13 as scratch. Stack: @r15 lim4e, @(4,r15) lim2e, @(8,r15)
 * pend = &left[last].sx, @(12,r15) the first output byte, @(16,r15) lim4 =
 * pend - 80 (0 without runs of 4), @(20,r15) lim2 = pend - 40, @(24,r15)
 * the blocked boundary's sx address (pend + 20 if none). A run of 4 may
 * start at pl while pl <= lim4e, a run of 2 while pl <= lim2e; the wrapper
 * keeps left at 128 or above, so none of these addresses wraps. The fast
 * loop runs while pl <= lim4e; the slow path tries runs of 2 near the end or
 * a blocked boundary, and rescans once past one. Callee-saved r8-r14 are
 * saved and restored; MACH, MACL, GBR and PR are not touched. */
__asm__(
"    .macro SH2ZI_MT_T4 fail\n"
"    mov     #80,r0\n"
"    mov.w   @(r0,r1),r10\n"      /* x4 */
"    mov.w   @r1+,r11\n"          /* x0 */
"    sub     r11,r10\n"           /* Dx = x4 - x0 */
"    add     r11,r11\n"
"    add     r10,r11\n"           /* x0 + x4 */
"    add     r11,r11\n"
"    sub     r8,r11\n"            /* Kx = 2 (x0 + x4) - L4 */
"    mov.w   @(r0,r1),r12\n"      /* y4 */
"    mov.w   @r1+,r13\n"          /* y0 */
"    add     #36,r1\n"            /* &sx2 */
"    mov.w   @r1+,r2\n"
"    shll2   r2\n"
"    sub     r11,r2\n"            /* F2x + L4 */
"    cmp/hi  r9,r2\n"
"    bt      \\fail\n"
"    sub     r13,r12\n"           /* Dy */
"    add     r13,r13\n"
"    add     r12,r13\n"
"    add     r13,r13\n"
"    sub     r8,r13\n"            /* Ky */
"    mov.w   @r1+,r3\n"
"    shll2   r3\n"
"    sub     r13,r3\n"            /* F2y + L4 */
"    cmp/hi  r9,r3\n"
"    bt      \\fail\n"
"    add     #-24,r1\n"           /* &sx1 */
"    mov.w   @r1+,r2\n"
"    mov.w   @r1+,r3\n"
"    shll2   r2\n"
"    sub     r11,r2\n"
"    add     r10,r2\n"            /* F1x + L4 */
"    cmp/hi  r9,r2\n"
"    bt      \\fail\n"
"    shll2   r3\n"
"    sub     r13,r3\n"
"    add     r12,r3\n"            /* F1y + L4 */
"    cmp/hi  r9,r3\n"
"    bt      \\fail\n"
"    add     #36,r1\n"            /* &sx3 */
"    mov.w   @r1+,r2\n"
"    mov.w   @r1,r3\n"
"    shll2   r2\n"
"    sub     r11,r2\n"
"    sub     r10,r2\n"            /* F3x + L4 */
"    cmp/hi  r9,r2\n"
"    bt      \\fail\n"
"    shll2   r3\n"
"    sub     r13,r3\n"
"    sub     r12,r3\n"            /* F3y + L4 */
"    cmp/hi  r9,r3\n"
"    bt      \\fail\n"
"    .endm\n"
"    .macro SH2ZI_MT_T2 fail\n"
"    mov.w   @r1+,r10\n"          /* x0 */
"    mov.w   @r1+,r11\n"          /* y0 */
"    add     #16,r1\n"
"    mov.w   @r1+,r2\n"           /* x1 */
"    mov.w   @r1+,r3\n"           /* y1 */
"    add     #16,r1\n"
"    mov.w   @r1+,r12\n"          /* x2 */
"    add     r12,r10\n"
"    add     r10,r10\n"
"    sub     r8,r10\n"            /* 2 (x0 + x2) - L4 */
"    shll2   r2\n"
"    sub     r10,r2\n"            /* 2 Fx + L4 */
"    cmp/hi  r9,r2\n"
"    bt      \\fail\n"
"    mov.w   @r1,r13\n"           /* y2 */
"    add     r13,r11\n"
"    add     r11,r11\n"
"    sub     r8,r11\n"
"    shll2   r3\n"
"    sub     r11,r3\n"            /* 2 Fy + L4 */
"    cmp/hi  r9,r3\n"
"    bt      \\fail\n"
"    .endm\n"
"    .macro SH2ZI_MT_BODY ends,sfx\n"
".Lsh2zi_mt_done\\sfx\\():\n"
"    mov.l   @(12,r15),r0\n"
"    sub     r0,r4\n"
"    mov     r4,r0\n"             /* runs written */
"    add     #28,r15\n"
"    mov.l   @r15+,r14\n"
"    mov.l   @r15+,r13\n"
"    mov.l   @r15+,r12\n"
"    mov.l   @r15+,r11\n"
"    mov.l   @r15+,r10\n"
"    mov.l   @r15+,r9\n"
"    rts\n"
"    mov.l   @r15+,r8\n"
".Lsh2zi_mt_rescan\\sfx\\():\n"   /* from boundary i: skip blocked boundaries, then find the next one */
"    mov.l   @(8,r15),r12\n"      /* pend */
".Lsh2zi_mt_skip\\sfx\\():\n"     /* boundary i blocked: repetition i is a single */
"    cmp/eq  r12,r5\n"
"    bt      .Lsh2zi_mt_done\\sfx\n"
"    mov.l   @(4,r5),r10\n"       /* outcode i in the top byte */
"    mov.l   @(4,r6),r13\n"
"    or      r10,r13\n"
"    tst     r14,r13\n"
"    bt      .Lsh2zi_mt_clear\\sfx\n"
"    mov     #1,r0\n"
"    mov.b   r0,@r4\n"
"    add     #1,r4\n"
"    add     #20,r5\n"
"    .if \\ends\n"
"    add     #4,r7\n"
"    .endif\n"
"    bra     .Lsh2zi_mt_skip\\sfx\n"
"    add     #20,r6\n"
".Lsh2zi_mt_clear\\sfx\\():\n"    /* the first blocked boundary after i, four a step */
"    mov     r5,r2\n"
"    add     #24,r2\n"            /* &left[i+1].outcode */
"    mov     r6,r3\n"
"    add     #24,r3\n"
"    add     #4,r12\n"            /* &left[last].outcode */
"    mov     r12,r1\n"
"    add     #-60,r1\n"
"    cmp/hi  r1,r2\n"
"    bt      .Lsh2zi_mt_st\\sfx\n"
".Lsh2zi_mt_s4\\sfx\\():\n"
"    mov.l   @r2,r10\n"
"    mov.l   @r3,r13\n"
"    or      r10,r13\n"
"    mov.l   @(20,r2),r10\n"
"    or      r10,r13\n"
"    mov.l   @(20,r3),r10\n"
"    or      r10,r13\n"
"    mov.l   @(40,r2),r10\n"
"    or      r10,r13\n"
"    mov.l   @(40,r3),r10\n"
"    or      r10,r13\n"
"    mov.l   @(60,r2),r10\n"
"    or      r10,r13\n"
"    mov.l   @(60,r3),r10\n"
"    or      r10,r13\n"
"    tst     r14,r13\n"
"    bf      .Lsh2zi_mt_st\\sfx\n" /* one of the four is blocked: find it one by one */
"    add     #80,r2\n"
"    cmp/hi  r1,r2\n"
"    bf/s    .Lsh2zi_mt_s4\\sfx\n"
"    add     #80,r3\n"
".Lsh2zi_mt_st\\sfx\\():\n"       /* one at a time */
"    cmp/hi  r12,r2\n"
"    bt      .Lsh2zi_mt_hit\\sfx\n" /* none: r2 = &left[last + 1].outcode */
"    mov.l   @r2,r10\n"
"    mov.l   @r3,r13\n"
"    or      r10,r13\n"
"    tst     r14,r13\n"
"    bf      .Lsh2zi_mt_hit\\sfx\n"
"    add     #20,r3\n"
"    bra     .Lsh2zi_mt_st\\sfx\n"
"    add     #20,r2\n"
".Lsh2zi_mt_hit\\sfx\\():\n"
"    add     #-4,r2\n"            /* blk = &left[block].sx */
"    mov.l   r2,@(24,r15)\n"
"    mov     r2,r0\n"
"    add     #-100,r0\n"          /* a run of 4 must end before block */
"    mov.l   @(16,r15),r1\n"
"    cmp/hi  r0,r1\n"
"    bf      1f\n"
"    mov     r0,r1\n"
"1:  mov.l   r1,@r15\n"           /* lim4e = min(lim4, blk - 100) */
"    mov     r2,r0\n"
"    add     #-60,r0\n"
"    mov.l   @(20,r15),r1\n"
"    cmp/hi  r0,r1\n"
"    bf      2f\n"
"    mov     r0,r1\n"
"2:  bra     .Lsh2zi_mt_top\\sfx\n"
"    mov.l   r1,@(4,r15)\n"       /* lim2e = min(lim2, blk - 60) */
".Lsh2zi_mt_s\\sfx\\():\n"
"    bra     .Lsh2zi_mt_slow\\sfx\n"
"    nop\n"
".Lsh2zi_mt_top\\sfx\\():\n"
"    mov.l   @r15,r1\n"
"    cmp/hi  r1,r5\n"
"    bt      .Lsh2zi_mt_s\\sfx\n"
".Lsh2zi_mt_main\\sfx\\():\n"     /* i + 4 <= last and i + 4 < the next blocked boundary */
"    .if \\ends\n"
"    mov.l   @(16,r7),r0\n"
"    tst     #16,r0\n"
"    bt      .Lsh2zi_mt_try2\\sfx\n"
"    .endif\n"
"    mov     r5,r1\n"
"    SH2ZI_MT_T4 .Lsh2zi_mt_try2\\sfx\n"
"    mov     r6,r1\n"
"    SH2ZI_MT_T4 .Lsh2zi_mt_try2\\sfx\n"
"    mov     #4,r0\n"
"    add     #80,r5\n"
"    add     #80,r6\n"
"    .if \\ends\n"
"    add     #16,r7\n"
"    .endif\n"
".Lsh2zi_mt_next\\sfx\\():\n"
"    mov.b   r0,@r4\n"
"    mov.l   @r15,r1\n"
"    cmp/hi  r1,r5\n"
"    bf/s    .Lsh2zi_mt_main\\sfx\n"
"    add     #1,r4\n"
"    bra     .Lsh2zi_mt_slow\\sfx\n"
"    nop\n"
".Lsh2zi_mt_try2\\sfx\\():\n"
"    .if \\ends\n"
"    mov.l   @(8,r7),r0\n"
"    tst     #4,r0\n"
"    bt      .Lsh2zi_mt_one\\sfx\n"
"    .endif\n"
"    mov     r5,r1\n"
"    SH2ZI_MT_T2 .Lsh2zi_mt_one\\sfx\n"
"    mov     r6,r1\n"
"    SH2ZI_MT_T2 .Lsh2zi_mt_one\\sfx\n"
"    mov     #2,r0\n"
"    add     #40,r5\n"
"    add     #40,r6\n"
"    .if \\ends\n"
"    bra     .Lsh2zi_mt_next\\sfx\n"
"    add     #8,r7\n"
"    .else\n"
"    bra     .Lsh2zi_mt_next\\sfx\n"
"    nop\n"
"    .endif\n"
".Lsh2zi_mt_one\\sfx\\():\n"
"    mov     #1,r0\n"
"    add     #20,r5\n"
"    add     #20,r6\n"
"    .if \\ends\n"
"    bra     .Lsh2zi_mt_next\\sfx\n"
"    add     #4,r7\n"
"    .else\n"
"    bra     .Lsh2zi_mt_next\\sfx\n"
"    nop\n"
"    .endif\n"
".Lsh2zi_mt_slow\\sfx\\():\n"     /* no run of 4 from i: near the end or a blocked boundary */
"    mov.l   @(8,r15),r1\n"
"    cmp/eq  r1,r5\n"
"    bf      3f\n"
"    bra     .Lsh2zi_mt_done\\sfx\n"
"    nop\n"
"3:  mov.l   @(4,r15),r1\n"
"    cmp/hi  r1,r5\n"
"    bt      .Lsh2zi_mt_sone\\sfx\n"
"    .if \\ends\n"
"    mov.l   @(8,r7),r0\n"
"    tst     #4,r0\n"
"    bt      .Lsh2zi_mt_sone\\sfx\n"
"    .endif\n"
"    mov     r5,r1\n"
"    SH2ZI_MT_T2 .Lsh2zi_mt_sone\\sfx\n"
"    mov     r6,r1\n"
"    SH2ZI_MT_T2 .Lsh2zi_mt_sone\\sfx\n"
"    mov     #2,r0\n"
"    mov.b   r0,@r4\n"
"    add     #1,r4\n"
"    add     #40,r5\n"
"    .if \\ends\n"
"    add     #8,r7\n"
"    .endif\n"
"    bra     .Lsh2zi_mt_slow\\sfx\n" /* a run of 2 here ends before the blocked boundary */
"    add     #40,r6\n"
".Lsh2zi_mt_sone\\sfx\\():\n"
"    mov     #1,r0\n"
"    mov.b   r0,@r4\n"
"    add     #1,r4\n"
"    add     #20,r5\n"
"    add     #20,r6\n"
"    .if \\ends\n"
"    add     #4,r7\n"
"    .endif\n"
"    mov.l   @(24,r15),r1\n"
"    cmp/hi  r1,r5\n"
"    bf      .Lsh2zi_mt_slow\\sfx\n" /* not past the blocked boundary yet */
"    bra     .Lsh2zi_mt_rescan\\sfx\n"
"    nop\n"
"    .endm\n"
"    .pushsection .text.sh2zi_merge_tier24,\"ax\",@progbits\n"
"    .align 2\n"
"    .global _sh2zi_merge_tier24\n"
"    .type _sh2zi_merge_tier24,@function\n"
"_sh2zi_merge_tier24:\n"
"    mov.l   r8,@-r15\n"
"    mov.l   r9,@-r15\n"
"    mov.l   r10,@-r15\n"
"    mov.l   r11,@-r15\n"
"    mov.l   r12,@-r15\n"
"    mov.l   r13,@-r15\n"
"    mov.l   r14,@-r15\n"
"    mov.l   @(28,r15),r1\n"      /* count */
"    mov.l   @(32,r15),r2\n"      /* run_mask */
"    mov.l   @(36,r15),r8\n"      /* tol_q8 */
"    add     #-28,r15\n"
"    shlr2   r8\n"
"    shlr2   r8\n"
"    shlr2   r8\n"                /* L4 = tol_q8 >> 6 */
"    mov     r8,r9\n"
"    add     r9,r9\n"             /* S4 = 2 L4 */
"    add     #12,r5\n"            /* &left[0].sx */
"    add     #12,r6\n"
"    mov.l   r4,@(12,r15)\n"      /* first output byte */
"    add     #-1,r1\n"            /* last */
"    mov     r1,r0\n"
"    shll2   r0\n"
"    mov     r0,r3\n"
"    shll2   r3\n"
"    add     r0,r3\n"
"    add     r5,r3\n"             /* pend = &left[last].sx */
"    mov.l   r3,@(8,r15)\n"
"    mov     r3,r10\n"
"    add     #-40,r10\n"
"    mov.l   r10,@(20,r15)\n"     /* lim2 = pend - 40: below &left[0].sx if last < 2 */
"    mov     #0,r10\n"
"    mov     r2,r0\n"
"    tst     #16,r0\n"
"    bt      2f\n"
"    mov     r3,r10\n"
"    add     #-80,r10\n"
"2:  mov.l   r10,@(16,r15)\n"     /* lim4 = pend - 80, or 0 without runs of 4 */
"    mov     #-61,r14\n"
"    shll16  r14\n"
"    shll8   r14\n"               /* 0xC3000000: the blocking bits of a big-endian outcode long */
"    tst     r7,r7\n"
"    bt      .Lsh2zi_mt_rescann\n"
"    bra     .Lsh2zi_mt_rescane\n"
"    nop\n"
"    SH2ZI_MT_BODY 0,n\n"
"    SH2ZI_MT_BODY 1,e\n"
"    .size _sh2zi_merge_tier24,.-_sh2zi_merge_tier24\n"
"    .popsection\n"
"    .purgem SH2ZI_MT_BODY\n"
"    .purgem SH2ZI_MT_T4\n"
"    .purgem SH2ZI_MT_T2\n"
);
#endif

/* The plan in C, for the other run lengths and for hosts. top is the
 * longest run run_mask allows (2 .. 16); ends is NULL exactly when has_ends
 * is false, which the compiler folds. */
static TIER_INLINE unsigned tier_plan(uint8_t *out,const sh2z_view_vertex_t *left,
    const sh2z_view_vertex_t *right,const uint32_t *ends,bool has_ends,unsigned last,
    uint32_t mask,unsigned top,uint32_t tol_q8) {
    const tier_limit_t l2=tier_limit(tol_q8>>7),l4=tier_limit(tol_q8>>6),
        l8=tier_limit(tol_q8>>5),l16=tier_limit(tol_q8>>4);
    uint8_t *o=out;
    unsigned i=0,block=0;   /* block: the first blocked boundary >= i, or last + 1 */
    while(block<=last && !tier_blocked(left,right,block))++block;
    while(i<last) {
        if(block<i) {
            block=i;
            while(block<=last && !tier_blocked(left,right,block))++block;
        }
        unsigned r=1;
        if(block>i+2u) {
            /* Boundaries i .. block-1 are clear, and block <= last + 1. */
            unsigned room=block-1u-i;
            if(room>top)room=top;
            const sh2z_view_vertex_t *pl=left+i,*pr=right+i;
            const uint32_t *pe=has_ends?ends+i:ends;
            if(room>=16u && (mask&TIER_RUN(16)) && (!has_ends || (pe[16]&TIER_RUN(16))) &&
               tier_rail_long(pl,16,l16) && tier_rail_long(pr,16,l16))r=16;
            else if(room>=8u && (mask&TIER_RUN(8)) && (!has_ends || (pe[8]&TIER_RUN(8))) &&
               tier_rail_long(pl,8,l8) && tier_rail_long(pr,8,l8))r=8;
            else if(room>=4u && (mask&TIER_RUN(4)) && (!has_ends || (pe[4]&TIER_RUN(4))) &&
               tier_rail4(pl,l4,l2) && tier_rail4(pr,l4,l2))r=4;
            else if((mask&TIER_RUN(2)) && (!has_ends || (pe[2]&TIER_RUN(2))) &&
               tier_rail2(pl,l2) && tier_rail2(pr,l2))r=2;
        }
        *o++=(uint8_t)r;
        i+=r;
    }
    return (unsigned)(o-out);
}
static TIER_NOINLINE unsigned tier_plan_ends(uint8_t *out,const sh2z_view_vertex_t *left,
    const sh2z_view_vertex_t *right,const uint32_t *ends,unsigned last,uint32_t mask,unsigned top,uint32_t tol_q8) {
    return tier_plan(out,left,right,ends,true,last,mask,top,tol_q8);
}
static TIER_NOINLINE unsigned tier_plan_free(uint8_t *out,const sh2z_view_vertex_t *left,
    const sh2z_view_vertex_t *right,unsigned last,uint32_t mask,unsigned top,uint32_t tol_q8) {
    return tier_plan(out,left,right,NULL,false,last,mask,top,tol_q8);
}

/* Everything the kernel does not take, with the arguments already checked. */
static TIER_NOINLINE int tier_plan_any(uint8_t *runs_out,const sh2z_view_vertex_t *left,
    const sh2z_view_vertex_t *right,const uint32_t *end_masks,unsigned count,
    uint32_t mask,uint32_t tol_q8) {
    const unsigned last=count-1u;
    if(!mask) {   /* nothing to merge: every repetition is its own quad */
        for(unsigned j=0;j<last;++j)runs_out[j]=1u;
        return (int)last;
    }
    const unsigned top=(mask&TIER_RUN(16))?16u:(mask&TIER_RUN(8))?8u:(mask&TIER_RUN(4))?4u:2u;
    return (int)(end_masks?tier_plan_ends(runs_out,left,right,end_masks,last,mask,top,tol_q8):
        tier_plan_free(runs_out,left,right,last,mask,top,tol_q8));
}

int sh2z_view_merge_tier(uint8_t *runs_out,const sh2z_view_vertex_t *left,
    const sh2z_view_vertex_t *right,const uint32_t *end_masks,unsigned count,
    uint32_t run_mask,uint32_t tol_q8) {
    if(!runs_out || !left || !right || count<2u || count>SH2Z_VIEW_MERGE_MAX_COUNT ||
       tol_q8>SH2Z_VIEW_MERGE_MAX_TOL || (run_mask&~(SH2Z_VIEW_MERGE_TIER_RUNS|1u)))
        return SH2Z_VIEW_MERGE_INVALID;
    const unsigned last=count-1u;
    const size_t vb=(size_t)count*sizeof(*left);
    if(tier_span_overlaps(runs_out,last,left,vb) || tier_span_overlaps(runs_out,last,right,vb) ||
       (end_masks && tier_span_overlaps(runs_out,last,end_masks,(size_t)count*sizeof(*end_masks))))
        return SH2Z_VIEW_MERGE_INVALID;
    const uint32_t mask=run_mask&(SH2Z_VIEW_MERGE_TIER_RUNS&~TIER_RUN(1));
#if SH2ZI_TIER_NATIVE
    /* Runs of {2, 4} or {2}; the kernel's limit arithmetic needs the left
     * rail above address 100, which every SH-2 RAM region is. */
    if((mask&TIER_RUN(2)) && !(mask&~(TIER_RUN(2)|TIER_RUN(4))) && (uintptr_t)left>=128u)
        return sh2zi_merge_tier24(runs_out,left,right,end_masks,count,mask,tol_q8);
#endif
    return tier_plan_any(runs_out,left,right,end_masks,count,mask,tol_q8);
}
