/* Integer view transform, projection and clip codes. Portable definition of
 * the view-contract-1 results. MIT; SH2ZAM Contributors. */
#include "sh2zam_view_internal.h"

_Static_assert(sizeof(sh2z_pos16_t)==6,"sh2z_pos16_t is 6 bytes");
_Static_assert(sizeof(sh2z_view_vertex_t)==20,"sh2z_view_vertex_t is 20 bytes");

bool sh2z_view_init(sh2z_view_t *out,const sh2z_view_t *in) {
    if(!out || !view_valid(in))return false;
    if(out!=in)*out=*in;
    return true;
}
static int64_t floor_shift16(int64_t n) {
    /* Floor division by 65536 without relying on signed right shift. */
    int64_t q=n/65536;
    if(n%65536!=0 && n<0)--q;
    return q;
}
bool sh2z_view_place(sh2z_view_t *v,sh2z_vec3_t eye) {
    if(!view_valid(v))return false;
    const int64_t e[3]={eye.x,eye.y,eye.z};
    int32_t t[3];
    for(unsigned i=0;i<3;++i) {
        int64_t sum=0;
        for(unsigned j=0;j<3;++j)sum+=(int64_t)v->r[i][j]*e[j];
        const int64_t value=-floor_shift16(sum);
        if(value<INT32_MIN || value>INT32_MAX)return false;
        t[i]=(int32_t)value;
        if(!row_fits(v->r[i],t[i]))return false;
    }
    for(unsigned i=0;i<3;++i)v->t[i]=t[i];
    return true;
}
static int32_t sat32(int64_t a) {
    return a<INT32_MIN?INT32_MIN:a>INT32_MAX?INT32_MAX:(int32_t)a;
}
/* Projection and outcode of one view-space point: the batch definition after
 * the transform. Shared by every entry point that projects. */
static SH2ZI_ALWAYS_INLINE sh2z_view_vertex_t project_point(const sh2z_view_t *c,int32_t x,int32_t y,int32_t z) {
    sh2z_view_vertex_t o={x,y,z,0,0,0,{0,0,0}};
    if(z<c->near_z) {
        o.outcode=SH2Z_OUT_NEAR;
        return o;
    }
    const int32_t qx=sat32((int64_t)c->focal_x*x/z);
    const int32_t qy=sat32((int64_t)c->focal_y*y/z);
    o.sx=sat16((int32_t)c->center_x+shift16(qx));
    o.sy=sat16((int32_t)c->center_y-shift16(qy));
    unsigned code=0;
    if(z>c->far_z)code|=SH2Z_OUT_FAR;
    if(o.sx<c->clip[0])code|=SH2Z_OUT_LEFT;
    if(o.sx>c->clip[2])code|=SH2Z_OUT_RIGHT;
    if(o.sy<c->clip[1])code|=SH2Z_OUT_TOP;
    if(o.sy>c->clip[3])code|=SH2Z_OUT_BOTTOM;
    o.outcode=(uint8_t)code;
    return o;
}
/* Out-of-line copy for the entry points that select a projector. */
static sh2z_view_vertex_t project(const sh2z_view_t *c,int32_t x,int32_t y,int32_t z) {
    return project_point(c,x,y,z);
}
static void batch_portable(sh2z_view_vertex_t *out,const sh2z_view_t *v,
    const void *positions,size_t in_stride,size_t count) {
    const sh2z_view_t c=*v;
    const unsigned char *p=positions;
    for(size_t k=0;k<count;++k,p+=in_stride) {
        /* positions and in_stride are 2-byte aligned by contract. */
        const int16_t *in=(const int16_t *)(const void *)p;
        int32_t view[3];
        for(unsigned i=0;i<3;++i)
            view[i]=(int32_t)c.r[i][0]*in[0]+(int32_t)c.r[i][1]*in[1]+(int32_t)c.r[i][2]*in[2]+c.t[i];
        out[k]=project_point(&c,view[0],view[1],view[2]);
    }
}
#if SH2Z_DIVU_NATIVE
/* project() through this CPU's divider: start x, form y's numerator while it
 * runs, collect x and start y, classify x while y runs. The divu helpers
 * answer non-fitting quotients with exactly sat32, as project() defines. */
static sh2z_view_vertex_t project_divu(const sh2z_view_t *c,int32_t x,int32_t y,int32_t z) {
    sh2z_view_vertex_t o={x,y,z,0,0,0,{0,0,0}};
    if(z<c->near_z) {
        o.outcode=SH2Z_OUT_NEAR;
        return o;
    }
    const uint64_t nx=(uint64_t)((int64_t)c->focal_x*x);
    sh2z_divu_t t;
    sh2z_divu_start64(&t,(int32_t)(uint32_t)(nx>>32),(uint32_t)nx,z);
    const uint64_t ny=(uint64_t)((int64_t)c->focal_y*y);
    const int32_t qx=sh2z_divu_finish(&t,NULL);
    sh2z_divu_start64(&t,(int32_t)(uint32_t)(ny>>32),(uint32_t)ny,z);
    o.sx=sat16((int32_t)c->center_x+shift16(qx));
    unsigned code=0;
    if(z>c->far_z)code|=SH2Z_OUT_FAR;
    if(o.sx<c->clip[0])code|=SH2Z_OUT_LEFT;
    if(o.sx>c->clip[2])code|=SH2Z_OUT_RIGHT;
    const int32_t qy=sh2z_divu_finish(&t,NULL);
    o.sy=sat16((int32_t)c->center_y-shift16(qy));
    if(o.sy<c->clip[1])code|=SH2Z_OUT_TOP;
    if(o.sy>c->clip[3])code|=SH2Z_OUT_BOTTOM;
    o.outcode=(uint8_t)code;
    return o;
}
#define PROJECT_DIVU project_divu
#else
#define PROJECT_DIVU project
#endif
/* Project view-space records in[0..count-1] into out (x y z read first, so
 * in == out is allowed). */
static void project_portable(sh2z_view_vertex_t *out,const sh2z_view_t *v,
    const sh2z_view_vertex_t *in,size_t count) {
    const sh2z_view_t c=*v;
    for(size_t k=0;k<count;++k) {
        const int32_t x=in[k].x,y=in[k].y,z=in[k].z;
        out[k]=project_point(&c,x,y,z);
    }
}
#if SH2Z_DIVU_NATIVE
static void project_divu_native(sh2z_view_vertex_t *out,const sh2z_view_t *v,
    const sh2z_view_vertex_t *in,size_t count);
#endif

#define SH2ZI_VIEW_POINT project
#define SH2ZI_VIEW_POINT_DIVU project_divu
#define SH2ZI_VIEW_RANGE project_portable
#define SH2ZI_VIEW_RANGE_DIVU project_divu_native
#include "sh2zam_view_paths.h"

bool sh2z_view_edge_steps(sh2z_view_vertex_t *out,const sh2z_view_t *v,
    const sh2z_view_vertex_t *from,const sh2z_view_vertex_t *to,
    uint16_t first_num,uint16_t step_num,uint16_t den,size_t count) {
    return edge_steps(out,v,from,to,first_num,step_num,den,count,false);
}
bool sh2z_view_edge_steps_divu(sh2z_view_vertex_t *out,const sh2z_view_t *v,
    const sh2z_view_vertex_t *from,const sh2z_view_vertex_t *to,
    uint16_t first_num,uint16_t step_num,uint16_t den,size_t count) {
    return edge_steps(out,v,from,to,first_num,step_num,den,count,true);
}
unsigned sh2z_view_clip_near(sh2z_view_vertex_t out[6],const sh2z_view_t *v,
    const sh2z_view_vertex_t in[4]) {
    return clip_near(out,v,in,project,false);
}
unsigned sh2z_view_clip_near_divu(sh2z_view_vertex_t out[6],const sh2z_view_t *v,
    const sh2z_view_vertex_t in[4]) {
    return clip_near(out,v,in,PROJECT_DIVU,true);
}

bool sh2z_view_batch(sh2z_view_vertex_t *out,const sh2z_view_t *v,
    const void *positions,size_t in_stride,size_t count) {
    if(!view_valid(v))return false;
    if(!count)return true;
    if(!batch_args_ok(out,positions,in_stride,count))return false;
    batch_portable(out,v,positions,in_stride,count);
    return true;
}

#if SH2Z_DIVU_NATIVE
/* SH-2 path of sh2z_view_batch_divu: hand-scheduled assembly, same results
 * as batch_portable byte for byte.
 *
 * Divisions. Both quotients share the divisor z >= near_z > 0. With
 * n = focal * view and focal < 2^31, |n| <= 2^62, so m = |n| >> 31 fits 32
 * bits, and the quotient fits the divider strictly (|q| < 2^31) exactly when
 * m < z. Otherwise |trunc(n / z)| >= 2^31 and the defined result is INT32_MAX
 * or INT32_MIN by the sign of n. The kernel only ever uses q >> 16 (floor),
 * and INT32_MIN >> 16 == (INT32_MIN + 1) >> 16, so an out-of-range quotient is
 * replaced by a stand-in division (+/-(2^31 - 1) * z) / z whose quotient fits
 * strictly and has the same >> 16. The main path therefore has one shape.
 *
 * Software pipeline across vertices, so the CPU works while the divider runs:
 *
 *     transform k, store x y z  (y division of k-1 still running)
 *     collect y quotient of k-1
 *     near test; start x division of k
 *     finish k-1: sy, top/bottom bits, store words 3-4
 *     y numerator of k; collect x quotient; start y division of k
 *     sx of k, far/left/right bits
 *
 * "prev" starts as, and after a NEAR vertex returns to, a scratch vertex in
 * the block, so finishing it is unconditional. Outcode bits are shifted in
 * with ROTCR in the order FAR LEFT RIGHT TOP BOTTOM; a final SHLR2 leaves
 * code << 24, which is word 4 (outcode then three zero bytes, big-endian).
 *
 * MULS.W (16x16) and DMULS.L (32x32) do not depend on SR.S. Callee-saved
 * r8-r14, PR, MACH and MACL are preserved.
 *
 * view-round-1 (sh2zam_view_round.c) uses two more entries made from the
 * same macros with trunc=1. Where a quotient becomes a pixel, that adds a
 * fix-up from the quotient's low 16 bits, kept in r10, which is free there:
 * sx + 1 when sx < 0 and the bits are not all zero, sy - 1 when sy > 0 and
 * they are not. The stand-in quotient +/-(2^31 - 1) is exactly view-round-1's
 * symmetric clamp, so the main path keeps its single shape. With trunc=0
 * the macros expand to the original instructions. */

__asm__(
/* Shared assembler macros: SH2ZI_VD_PROLOGUE saves registers and sets up
 * r2-r7 and r14; SH2ZI_VD_REST is everything after a vertex's view-space
 * x y z are in r11 r12 r13 and stored: division pipeline, classification,
 * NEAR path, stubs, finish subroutine and epilogue. tag keeps labels unique. */
"    .macro SH2ZI_VD_PROLOGUE\n"
"    mov.l   r8,@-r15\n"
"    mov.l   r9,@-r15\n"
"    mov.l   r10,@-r15\n"
"    mov.l   r11,@-r15\n"
"    mov.l   r12,@-r15\n"
"    mov.l   r13,@-r15\n"
"    mov.l   r14,@-r15\n"
"    sts.l   mach,@-r15\n"
"    sts.l   macl,@-r15\n"
"    sts.l   pr,@-r15\n"
"    mov     r5,r0\n"
"    mov     r6,r5\n"              /* r5 positions */
"    mov     r7,r6\n"              /* r6 count (> 0) */
"    mov     r0,r7\n"              /* r7 block */
"    mov     #-1,r14\n"
"    shll8   r14\n"                /* r14 = 0xFFFFFF00, divider */
"    mov     r7,r2\n"
"    add     #64,r2\n"             /* r2 prev = scratch */
"    mov     #0,r3\n"              /* r3 prev sx << 16 */
"    .endm\n"
"    .macro SH2ZI_VD_REST tag, trunc=0\n"
"    mov.l   @(20,r14),r8\n"       /* collect y quotient of prev */
"    mov.l   @(44,r7),r1\n"
"    cmp/ge  r1,r13\n"
"    bf      .Lsh2zi_vd_near\\tag\n"
"    mov.l   r13,@r14\n"           /* DVSR = z */
"    mov.l   @(48,r7),r1\n"
"    dmuls.l r1,r11\n"
"    sts     mach,r9\n"
"    sts     macl,r10\n"
"    mov     r9,r1\n"
"    cmp/pz  r1\n"
"    bt/s    1f\n"
"    mov     r10,r0\n"
"    negc    r0,r0\n"
"    negc    r1,r1\n"
"1:  shll    r0\n"
"    rotcl   r1\n"
"    cmp/hs  r13,r1\n"
"    bt      .Lsh2zi_vd_xsat\\tag\n"
"    mov.l   r9,@(16,r14)\n"
"    mov.l   r10,@(20,r14)\n"      /* start x */
".Lsh2zi_vd_xback\\tag:\n"
"    .if \\trunc\n"
"    extu.w  r8,r10\n"             /* y quotient's fraction bits */
"    .endif\n"
"    shlr16  r8\n"
"    exts.w  r8,r8\n"
"    mov.w   @(20,r7),r0\n"
"    sub     r8,r0\n"
"    .if \\trunc\n"
"    cmp/pl  r0\n"
"    bf/s    6f\n"
"    tst     r10,r10\n"
"    movt    r10\n"
"    add     #-1,r0\n"
"    add     r10,r0\n"             /* sy - 1 unless the fraction is zero */
"6:\n"
"    .endif\n"
"    exts.w  r0,r1\n"
"    cmp/eq  r0,r1\n"
"    bf      .Lsh2zi_vd_sysat\\tag\n"
".Lsh2zi_vd_syback\\tag:\n"
"    mov.l   @(60,r7),r9\n"
"    mov.w   @(26,r7),r0\n"
"    cmp/gt  r1,r0\n"
"    rotcr   r9\n"
"    mov.w   @(28,r7),r0\n"
"    cmp/gt  r0,r1\n"
"    rotcr   r9\n"
"    shlr2   r9\n"
"    mov.l   r9,@(16,r2)\n"
"    extu.w  r1,r1\n"
"    or      r3,r1\n"
"    mov.l   r1,@(12,r2)\n"        /* prev finished */
"    mov.l   @(52,r7),r1\n"
"    dmuls.l r1,r12\n"
"    sts     mach,r9\n"
"    sts     macl,r10\n"
"    mov     r9,r1\n"
"    cmp/pz  r1\n"
"    bt/s    2f\n"
"    mov     r10,r0\n"
"    negc    r0,r0\n"
"    negc    r1,r1\n"
"2:  shll    r0\n"
"    rotcl   r1\n"
"    cmp/hs  r13,r1\n"
"    bt/s    .Lsh2zi_vd_ysat\\tag\n"
"    mov.l   @(20,r14),r8\n"       /* collect x quotient (both paths) */
"    mov.l   r9,@(16,r14)\n"
"    mov.l   r10,@(20,r14)\n"      /* start y */
".Lsh2zi_vd_yback\\tag:\n"
"    .if \\trunc\n"
"    extu.w  r8,r10\n"             /* x quotient's fraction bits */
"    .endif\n"
"    shlr16  r8\n"
"    exts.w  r8,r8\n"
"    mov.w   @(18,r7),r0\n"
"    add     r0,r8\n"
"    .if \\trunc\n"
"    cmp/pz  r8\n"
"    bt/s    7f\n"
"    tst     r10,r10\n"
"    movt    r10\n"
"    add     #1,r8\n"
"    sub     r10,r8\n"             /* sx + 1 unless the fraction is zero */
"7:\n"
"    .endif\n"
"    exts.w  r8,r3\n"
"    cmp/eq  r8,r3\n"
"    bf      .Lsh2zi_vd_sxsat\\tag\n"
".Lsh2zi_vd_sxback\\tag:\n"
"    mov.l   @(56,r7),r1\n"
"    mov     #0,r9\n"
"    cmp/gt  r1,r13\n"
"    rotcr   r9\n"
"    mov.w   @(22,r7),r0\n"
"    cmp/gt  r3,r0\n"
"    rotcr   r9\n"
"    mov.w   @(24,r7),r0\n"
"    cmp/gt  r0,r3\n"
"    rotcr   r9\n"
"    mov.l   r9,@(60,r7)\n"
"    shll16  r3\n"
"    mov     r4,r2\n"
"    dt      r6\n"
"    bf/s    .Lsh2zi_vd_loop\\tag\n"
"    add     #20,r4\n"
"    bra     .Lsh2zi_vd_done\\tag\n"
"    nop\n"
".Lsh2zi_vd_near\\tag:\n"
"    bsr     .Lsh2zi_vd_finish\\tag\n"
"    nop\n"
"    mov     #0,r0\n"
"    mov.l   r0,@(12,r4)\n"
"    mov     #1,r0\n"
"    shll8   r0\n"
"    shll16  r0\n"
"    mov.l   r0,@(16,r4)\n"
"    mov     r7,r2\n"
"    add     #64,r2\n"
"    dt      r6\n"
"    bt/s    .Lsh2zi_vd_done\\tag\n"
"    add     #20,r4\n"
"    bra     .Lsh2zi_vd_loop\\tag\n"
"    nop\n"
".Lsh2zi_vd_xsat\\tag:\n"
"    mov     #-1,r1\n"
"    shlr    r1\n"
"    cmp/pz  r9\n"
"    bt      3f\n"
"    neg     r1,r1\n"
"3:  dmuls.l r1,r13\n"
"    sts     mach,r0\n"
"    mov.l   r0,@(16,r14)\n"
"    sts     macl,r0\n"
"    bra     .Lsh2zi_vd_xback\\tag\n"
"    mov.l   r0,@(20,r14)\n"
".Lsh2zi_vd_ysat\\tag:\n"
"    mov     #-1,r1\n"
"    shlr    r1\n"
"    cmp/pz  r9\n"
"    bt      4f\n"
"    neg     r1,r1\n"
"4:  dmuls.l r1,r13\n"
"    sts     mach,r0\n"
"    mov.l   r0,@(16,r14)\n"
"    sts     macl,r0\n"
"    bra     .Lsh2zi_vd_yback\\tag\n"
"    mov.l   r0,@(20,r14)\n"
".Lsh2zi_vd_sysat\\tag:\n"
"    mov     #-128,r1\n"
"    shll8   r1\n"
"    cmp/pz  r0\n"
"    bf      .Lsh2zi_vd_syback\\tag\n"
"    bra     .Lsh2zi_vd_syback\\tag\n"
"    not     r1,r1\n"
".Lsh2zi_vd_sxsat\\tag:\n"
"    mov     #-128,r3\n"
"    shll8   r3\n"
"    cmp/pz  r8\n"
"    bf      .Lsh2zi_vd_sxback\\tag\n"
"    bra     .Lsh2zi_vd_sxback\\tag\n"
"    not     r3,r3\n"
/* Finish prev from r8 = its y quotient, r2 = prev, r3 = its sx << 16. */
".Lsh2zi_vd_finish\\tag:\n"
"    .if \\trunc\n"
"    extu.w  r8,r10\n"
"    .endif\n"
"    shlr16  r8\n"
"    exts.w  r8,r8\n"
"    mov.w   @(20,r7),r0\n"
"    sub     r8,r0\n"
"    .if \\trunc\n"
"    cmp/pl  r0\n"
"    bf/s    8f\n"
"    tst     r10,r10\n"
"    movt    r10\n"
"    add     #-1,r0\n"
"    add     r10,r0\n"
"8:\n"
"    .endif\n"
"    exts.w  r0,r1\n"
"    cmp/eq  r0,r1\n"
"    bt      5f\n"
"    mov     #-128,r1\n"
"    shll8   r1\n"
"    cmp/pz  r0\n"
"    bf      5f\n"
"    not     r1,r1\n"
"5:  mov.l   @(60,r7),r9\n"
"    mov.w   @(26,r7),r0\n"
"    cmp/gt  r1,r0\n"
"    rotcr   r9\n"
"    mov.w   @(28,r7),r0\n"
"    cmp/gt  r0,r1\n"
"    rotcr   r9\n"
"    shlr2   r9\n"
"    mov.l   r9,@(16,r2)\n"
"    extu.w  r1,r1\n"
"    or      r3,r1\n"
"    rts\n"
"    mov.l   r1,@(12,r2)\n"
".Lsh2zi_vd_done\\tag:\n"
"    mov.l   @(20,r14),r8\n"
"    bsr     .Lsh2zi_vd_finish\\tag\n"
"    nop\n"
"    lds.l   @r15+,pr\n"
"    lds.l   @r15+,macl\n"
"    lds.l   @r15+,mach\n"
"    mov.l   @r15+,r14\n"
"    mov.l   @r15+,r13\n"
"    mov.l   @r15+,r12\n"
"    mov.l   @r15+,r11\n"
"    mov.l   @r15+,r10\n"
"    mov.l   @r15+,r9\n"
"    rts\n"
"    mov.l   @r15+,r8\n"
"    .endm\n"
/* SH2ZI_VD_TRANSFORM reads one int16 position, transforms it and stores
 * x y z (batch entries); SH2ZI_VD_LOAD copies one view-space record's x y z
 * (project entries). Both leave x y z in r11 r12 r13 for SH2ZI_VD_REST. */
"    .macro SH2ZI_VD_TRANSFORM\n"
"    mov.w   @r5+,r8\n"
"    mov.w   @r5+,r9\n"
"    mov.w   @r5+,r10\n"
"    mov.w   @(30,r7),r0\n"
"    add     r0,r5\n"
"    mov.l   @(32,r7),r13\n"
"    mov.w   @(0,r7),r0\n"
"    muls.w  r0,r8\n"
"    mov.w   @(2,r7),r0\n"
"    sts     macl,r1\n"
"    muls.w  r0,r9\n"
"    add     r1,r13\n"
"    mov.w   @(4,r7),r0\n"
"    sts     macl,r1\n"
"    muls.w  r0,r10\n"
"    add     r1,r13\n"
"    mov.l   @(36,r7),r11\n"
"    sts     macl,r1\n"
"    add     r1,r13\n"             /* vz */
"    mov.w   @(6,r7),r0\n"
"    muls.w  r0,r8\n"
"    mov.w   @(8,r7),r0\n"
"    sts     macl,r1\n"
"    muls.w  r0,r9\n"
"    add     r1,r11\n"
"    mov.w   @(10,r7),r0\n"
"    sts     macl,r1\n"
"    muls.w  r0,r10\n"
"    add     r1,r11\n"
"    mov.l   @(40,r7),r12\n"
"    sts     macl,r1\n"
"    add     r1,r11\n"             /* vx */
"    mov.w   @(12,r7),r0\n"
"    muls.w  r0,r8\n"
"    mov.w   @(14,r7),r0\n"
"    sts     macl,r1\n"
"    muls.w  r0,r9\n"
"    add     r1,r12\n"
"    mov.w   @(16,r7),r0\n"
"    sts     macl,r1\n"
"    muls.w  r0,r10\n"
"    add     r1,r12\n"
"    sts     macl,r1\n"
"    add     r1,r12\n"             /* vy */
"    mov.l   r11,@r4\n"
"    mov.l   r12,@(4,r4)\n"
"    mov.l   r13,@(8,r4)\n"
"    .endm\n"
"    .macro SH2ZI_VD_LOAD\n"
"    mov.l   @r5,r11\n"
"    mov.l   @(4,r5),r12\n"
"    mov.l   @(8,r5),r13\n"
"    add     #20,r5\n"
"    mov.l   r11,@r4\n"
"    mov.l   r12,@(4,r4)\n"
"    mov.l   r13,@(8,r4)\n"
"    .endm\n"
"    .text\n"
"    .align 2\n"
"    .global _sh2zi_view_divu_run\n"
"    .type _sh2zi_view_divu_run,@function\n"
"_sh2zi_view_divu_run:\n"
"    SH2ZI_VD_PROLOGUE\n"
".Lsh2zi_vd_loopT:\n"
"    SH2ZI_VD_TRANSFORM\n"
"    SH2ZI_VD_REST T\n"
"    .size _sh2zi_view_divu_run,.-_sh2zi_view_divu_run\n"
/* View-space entry: r6 points at sh2z_view_vertex_t records (stride 20);
 * x y z are read, copied to out and projected. in == out is allowed: a
 * record is read before any byte of it or of a later record is written. */
"    .align 2\n"
"    .global _sh2zi_view_divu_project\n"
"    .type _sh2zi_view_divu_project,@function\n"
"_sh2zi_view_divu_project:\n"
"    SH2ZI_VD_PROLOGUE\n"
".Lsh2zi_vd_loopP:\n"
"    SH2ZI_VD_LOAD\n"
"    SH2ZI_VD_REST P\n"
"    .size _sh2zi_view_divu_project,.-_sh2zi_view_divu_project\n"
/* view-round-1 entries (sh2zam_view_round.c): the truncating fix-ups. */
"    .align 2\n"
"    .global _sh2zi_view_divu_run_trunc\n"
"    .type _sh2zi_view_divu_run_trunc,@function\n"
"_sh2zi_view_divu_run_trunc:\n"
"    SH2ZI_VD_PROLOGUE\n"
".Lsh2zi_vd_loopTT:\n"
"    SH2ZI_VD_TRANSFORM\n"
"    SH2ZI_VD_REST TT, 1\n"
"    .size _sh2zi_view_divu_run_trunc,.-_sh2zi_view_divu_run_trunc\n"
"    .align 2\n"
"    .global _sh2zi_view_divu_project_trunc\n"
"    .type _sh2zi_view_divu_project_trunc,@function\n"
"_sh2zi_view_divu_project_trunc:\n"
"    SH2ZI_VD_PROLOGUE\n"
".Lsh2zi_vd_loopPT:\n"
"    SH2ZI_VD_LOAD\n"
"    SH2ZI_VD_REST PT, 1\n"
"    .size _sh2zi_view_divu_project_trunc,.-_sh2zi_view_divu_project_trunc\n"
);

static void batch_divu_native(sh2z_view_vertex_t *out,const sh2z_view_t *v,
    const void *positions,size_t in_stride,size_t count) {
    if(in_stride-6u>(size_t)INT16_MAX) {
        /* The assembly loop keeps the stride step in 16 bits. */
        batch_portable(out,v,positions,in_stride,count);
        return;
    }
    divu_block_t b;
    fill_block(&b,v,(int16_t)(in_stride-6u));
    sh2zi_view_divu_run(out,&b,positions,count);
}
static void project_divu_native(sh2z_view_vertex_t *out,const sh2z_view_t *v,
    const sh2z_view_vertex_t *in,size_t count) {
    divu_block_t b;
    fill_block(&b,v,0);
    sh2zi_view_divu_project(out,&b,in,count);
}
#endif

bool sh2z_view_batch_divu(sh2z_view_vertex_t *out,const sh2z_view_t *v,
    const void *positions,size_t in_stride,size_t count) {
    if(!view_valid(v))return false;
    if(!count)return true;
    if(!batch_args_ok(out,positions,in_stride,count))return false;
#if SH2Z_DIVU_NATIVE
    batch_divu_native(out,v,positions,in_stride,count);
#else
    batch_portable(out,v,positions,in_stride,count);
#endif
    return true;
}

bool sh2z_view_project(sh2z_view_vertex_t *out,const sh2z_view_t *v,
    const sh2z_view_vertex_t *in,size_t count) {
    if(!view_valid(v))return false;
    if(!count)return true;
    if(!project_args_ok(out,in,count))return false;
    project_portable(out,v,in,count);
    return true;
}
bool sh2z_view_project_divu(sh2z_view_vertex_t *out,const sh2z_view_t *v,
    const sh2z_view_vertex_t *in,size_t count) {
    if(!view_valid(v))return false;
    if(!count)return true;
    if(!project_args_ok(out,in,count))return false;
#if SH2Z_DIVU_NATIVE
    project_divu_native(out,v,in,count);
#else
    project_portable(out,v,in,count);
#endif
    return true;
}

/* ---- view-guard-1: guard-band polygon clip (body in sh2zam_view_paths.h) - */
int sh2z_view_clip_guard(sh2z_view_guard_vertex_t out[SH2Z_VIEW_GUARD_MAX_OUT],
    const sh2z_view_t *v,const int16_t guard[4],
    const sh2z_view_guard_vertex_t *in,unsigned count,unsigned attrs) {
    return clip_guard(out,v,guard,in,count,attrs,false);
}
int sh2z_view_clip_guard_divu(sh2z_view_guard_vertex_t out[SH2Z_VIEW_GUARD_MAX_OUT],
    const sh2z_view_t *v,const int16_t guard[4],
    const sh2z_view_guard_vertex_t *in,unsigned count,unsigned attrs) {
    return clip_guard(out,v,guard,in,count,attrs,true);
}

/* ---- view-merge-1: bounded texture-merge planner ------------------------ */
#define MERGE_BLOCKING ((uint8_t)~(SH2Z_OUT_LEFT|SH2Z_OUT_RIGHT|SH2Z_OUT_TOP|SH2Z_OUT_BOTTOM))

static uint64_t magnitude64(int64_t a) {
    return a<0?(uint64_t)0-(uint64_t)a:(uint64_t)a;
}
/* a * b exactly for a < 2^32 and |b| < 2^31, from 32 x 32 -> 64-bit signed
 * products (dmuls.l on SH-2) instead of a general 64-bit multiply. */
static SH2ZI_ALWAYS_INLINE int64_t merge_mul(uint32_t a,int32_t b) {
    if(a<=(uint32_t)INT32_MAX)return (int64_t)(int32_t)a*b;
    return (int64_t)(int32_t)(a-UINT32_C(0x80000000))*b+(int64_t)b*INT64_C(0x80000000);
}
/* The k-th interior boundary to test, k = 0 .. r-2: the middle one first
 * (where a perspective rail usually strays most), then the others in order.
 * The order changes only how soon a failing run is rejected. */
static SH2ZI_ALWAYS_INLINE unsigned merge_order(unsigned j,unsigned mid) {
    return j==0u?mid:j<mid?j:j+1u;
}
/* Interior boundaries 1 .. r-1 of one rail of the run p[0] .. p[r] with rows
 * q[0] .. q[r]; scale is tol_q8 (pixel mode) or 256 (the rows mode guard).
 * For integer m, m * 256 <= scale * S exactly when m <= floor(scale * S / 256),
 * and likewise for the row test, so both limits are computed once per rail.
 * Ranges: P differences and D lie in +-65535 and S, t < 2^32, so |E| < 2^49.
 * In rows mode the guard runs first and gives |E.x|, |E.y| <= S < 2^32, so
 * |E . D| < 2^49; tol_q8 * |D|^2 <= 2^24 * 2^33. Nothing wraps. When
 * S < 2^14, |E| <= (S + t) * 65535 < 2^31 and int32 products suffice. */
static bool merge_rail_ok(const sh2z_view_vertex_t *p,const uint32_t *q,unsigned r,
    uint32_t scale,uint32_t tol_q8,bool row_mode) {
    const int32_t x0=p[0].sx,y0=p[0].sy,dx=p[r].sx-x0,dy=p[r].sy-y0;
    const uint32_t q0=q[0],s=q[r]-q0;
    const uint64_t pixel_limit=((uint64_t)scale*s)>>8;
    const unsigned mid=r>>1;
    uint64_t row_limit=0;
    if(row_mode) {
        const uint64_t d2=(uint64_t)((int64_t)dx*dx+(int64_t)dy*dy);
        row_limit=(d2<=UINT32_MAX?(uint64_t)tol_q8*(uint32_t)d2:tol_q8*d2)>>8;
    }
    if(s<16384u) {
        const int32_t s32=(int32_t)s;
        const uint32_t limit32=(uint32_t)pixel_limit;   /* < 2^24 * 2^14 / 2^8 */
        for(unsigned j=0;j+1u<r;++j) {
            const unsigned k=merge_order(j,mid);
            const int32_t t=(int32_t)(q[k]-q0);
            const int32_t ex=s32*(p[k].sx-x0)-t*dx,ey=s32*(p[k].sy-y0)-t*dy;
            const uint32_t ax=magnitude32(ex),ay=magnitude32(ey);
            if((ax>ay?ax:ay)>limit32)return false;
            if(row_mode && magnitude64((int64_t)ex*dx+(int64_t)ey*dy)>row_limit)return false;
        }
        return true;
    }
    for(unsigned j=0;j+1u<r;++j) {
        const unsigned k=merge_order(j,mid);
        const int32_t px=p[k].sx-x0,py=p[k].sy-y0;
        const uint32_t t=q[k]-q0;
        const int64_t ex=merge_mul(s,px)-merge_mul(t,dx),ey=merge_mul(s,py)-merge_mul(t,dy);
        const uint64_t ax=magnitude64(ex),ay=magnitude64(ey);
        if((ax>ay?ax:ay)>pixel_limit)return false;
        if(row_mode) {
            const int64_t dot=merge_mul((uint32_t)ax,ex<0?-dx:dx)+merge_mul((uint32_t)ay,ey<0?-dy:dy);
            if(magnitude64(dot)>row_limit)return false;
        }
    }
    return true;
}
static bool merge_blocked(const sh2z_view_vertex_t *left,const sh2z_view_vertex_t *right,unsigned j) {
    return ((left[j].outcode|right[j].outcode)&MERGE_BLOCKING)!=0;
}
static bool merge_span_ok(const void *p,size_t bytes) {
    return (uintptr_t)p<=UINTPTR_MAX-bytes;
}
int sh2z_view_merge_runs(uint8_t *runs_out,const sh2z_view_vertex_t *left,
    const sh2z_view_vertex_t *right,const uint32_t *rows,const uint32_t *end_masks,
    unsigned count,uint32_t run_mask,uint32_t tol_q8,unsigned mode) {
    if(!runs_out || !left || !right || !rows || count<2u || count>SH2Z_VIEW_MERGE_MAX_COUNT ||
       mode>SH2Z_VIEW_MERGE_ROWS || tol_q8>SH2Z_VIEW_MERGE_MAX_TOL)return SH2Z_VIEW_MERGE_INVALID;
    const unsigned last=count-1u;
    const size_t vb=(size_t)count*sizeof(*left),wb=(size_t)count*sizeof(*rows);
    if(!merge_span_ok(runs_out,last) || !merge_span_ok(left,vb) || !merge_span_ok(right,vb) ||
       !merge_span_ok(rows,wb) || (end_masks && !merge_span_ok(end_masks,wb)) ||
       overlaps(runs_out,last,left,vb) || overlaps(runs_out,last,right,vb) ||
       overlaps(runs_out,last,rows,wb) || (end_masks && overlaps(runs_out,last,end_masks,wb)))
        return SH2Z_VIEW_MERGE_INVALID;
    for(unsigned j=0;j<last;++j)if(rows[j+1u]<=rows[j])return SH2Z_VIEW_MERGE_INVALID;
    const bool row_mode=mode==SH2Z_VIEW_MERGE_ROWS;
    const uint32_t scale=row_mode?256u:tol_q8,mask=run_mask&~UINT32_C(3);
    unsigned top=0;   /* the longest run the mask allows */
    for(uint32_t m=mask>>1;m;m>>=1)++top;
    unsigned i=0,n=0,blocked=0;
    while(i<last) {
        /* blocked: the first boundary >= i that stops merging, or count. */
        if(blocked<i)blocked=i;
        while(blocked<=last && !merge_blocked(left,right,blocked))++blocked;
        unsigned limit=last-i;
        if(limit>top)limit=top;
        if(blocked-i<=limit)limit=blocked==i?0u:blocked-i-1u;
        unsigned r=1;
        /* bit tracks 1 << c with constant shifts (SH-2 has no barrel shifter). */
        uint32_t bit=limit>=2u?UINT32_C(1)<<limit:0u;
        for(unsigned c=limit;c>=2u;--c,bit>>=1) {
            if(!(mask&bit) || (end_masks && !(end_masks[i+c]&bit)))continue;
            if(merge_rail_ok(left+i,rows+i,c,scale,tol_q8,row_mode) &&
               merge_rail_ok(right+i,rows+i,c,scale,tol_q8,row_mode)){r=c;break;}
        }
        runs_out[n++]=(uint8_t)r;
        i+=r;
    }
    return (int)n;
}
