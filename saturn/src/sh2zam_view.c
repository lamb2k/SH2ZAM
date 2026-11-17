/* Integer view transform, projection and clip codes. Portable definition of
 * the view-contract-1 results. MIT; SH2ZAM Contributors. */
#include "sh2zam/view_experimental.h"
#include "sh2zam/divu_experimental.h"

_Static_assert(sizeof(sh2z_pos16_t)==6,"sh2z_pos16_t is 6 bytes");
_Static_assert(sizeof(sh2z_view_vertex_t)==20,"sh2z_view_vertex_t is 20 bytes");

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
        sh2z_view_vertex_t o={view[0],view[1],view[2],0,0,0,{0,0,0}};
        if(view[2]<c.near_z) {
            o.outcode=SH2Z_OUT_NEAR;
        } else {
            const int32_t qx=sat32((int64_t)c.focal_x*view[0]/view[2]);
            const int32_t qy=sat32((int64_t)c.focal_y*view[1]/view[2]);
            o.sx=sat16((int32_t)c.center_x+shift16(qx));
            o.sy=sat16((int32_t)c.center_y-shift16(qy));
            unsigned code=0;
            if(view[2]>c.far_z)code|=SH2Z_OUT_FAR;
            if(o.sx<c.clip[0])code|=SH2Z_OUT_LEFT;
            if(o.sx>c.clip[2])code|=SH2Z_OUT_RIGHT;
            if(o.sy<c.clip[1])code|=SH2Z_OUT_TOP;
            if(o.sy>c.clip[3])code|=SH2Z_OUT_BOTTOM;
            o.outcode=(uint8_t)code;
        }
        out[k]=o;
    }
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
 * r8-r14, PR, MACH and MACL are preserved. */
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

__asm__(
"    .text\n"
"    .align 2\n"
"    .global _sh2zi_view_divu_run\n"
"    .type _sh2zi_view_divu_run,@function\n"
"_sh2zi_view_divu_run:\n"
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
".Lsh2zi_vd_loop:\n"
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
"    mov.l   @(20,r14),r8\n"       /* collect y quotient of prev */
"    mov.l   @(44,r7),r1\n"
"    cmp/ge  r1,r13\n"
"    bf      .Lsh2zi_vd_near\n"
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
"    bt      .Lsh2zi_vd_xsat\n"
"    mov.l   r9,@(16,r14)\n"
"    mov.l   r10,@(20,r14)\n"      /* start x */
".Lsh2zi_vd_xback:\n"
"    shlr16  r8\n"
"    exts.w  r8,r8\n"
"    mov.w   @(20,r7),r0\n"
"    sub     r8,r0\n"
"    exts.w  r0,r1\n"
"    cmp/eq  r0,r1\n"
"    bf      .Lsh2zi_vd_sysat\n"
".Lsh2zi_vd_syback:\n"
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
"    bt/s    .Lsh2zi_vd_ysat\n"
"    mov.l   @(20,r14),r8\n"       /* collect x quotient (both paths) */
"    mov.l   r9,@(16,r14)\n"
"    mov.l   r10,@(20,r14)\n"      /* start y */
".Lsh2zi_vd_yback:\n"
"    shlr16  r8\n"
"    exts.w  r8,r8\n"
"    mov.w   @(18,r7),r0\n"
"    add     r0,r8\n"
"    exts.w  r8,r3\n"
"    cmp/eq  r8,r3\n"
"    bf      .Lsh2zi_vd_sxsat\n"
".Lsh2zi_vd_sxback:\n"
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
"    bf/s    .Lsh2zi_vd_loop\n"
"    add     #20,r4\n"
"    bra     .Lsh2zi_vd_done\n"
"    nop\n"
".Lsh2zi_vd_near:\n"
"    bsr     .Lsh2zi_vd_finish\n"
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
"    bt/s    .Lsh2zi_vd_done\n"
"    add     #20,r4\n"
"    bra     .Lsh2zi_vd_loop\n"
"    nop\n"
".Lsh2zi_vd_xsat:\n"
"    mov     #-1,r1\n"
"    shlr    r1\n"
"    cmp/pz  r9\n"
"    bt      3f\n"
"    neg     r1,r1\n"
"3:  dmuls.l r1,r13\n"
"    sts     mach,r0\n"
"    mov.l   r0,@(16,r14)\n"
"    sts     macl,r0\n"
"    bra     .Lsh2zi_vd_xback\n"
"    mov.l   r0,@(20,r14)\n"
".Lsh2zi_vd_ysat:\n"
"    mov     #-1,r1\n"
"    shlr    r1\n"
"    cmp/pz  r9\n"
"    bt      4f\n"
"    neg     r1,r1\n"
"4:  dmuls.l r1,r13\n"
"    sts     mach,r0\n"
"    mov.l   r0,@(16,r14)\n"
"    sts     macl,r0\n"
"    bra     .Lsh2zi_vd_yback\n"
"    mov.l   r0,@(20,r14)\n"
".Lsh2zi_vd_sysat:\n"
"    mov     #-128,r1\n"
"    shll8   r1\n"
"    cmp/pz  r0\n"
"    bf      .Lsh2zi_vd_syback\n"
"    bra     .Lsh2zi_vd_syback\n"
"    not     r1,r1\n"
".Lsh2zi_vd_sxsat:\n"
"    mov     #-128,r3\n"
"    shll8   r3\n"
"    cmp/pz  r8\n"
"    bf      .Lsh2zi_vd_sxback\n"
"    bra     .Lsh2zi_vd_sxback\n"
"    not     r3,r3\n"
/* Finish prev from r8 = its y quotient, r2 = prev, r3 = its sx << 16. */
".Lsh2zi_vd_finish:\n"
"    shlr16  r8\n"
"    exts.w  r8,r8\n"
"    mov.w   @(20,r7),r0\n"
"    sub     r8,r0\n"
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
".Lsh2zi_vd_done:\n"
"    mov.l   @(20,r14),r8\n"
"    bsr     .Lsh2zi_vd_finish\n"
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
"    .size _sh2zi_view_divu_run,.-_sh2zi_view_divu_run\n"
);

static void batch_divu_native(sh2z_view_vertex_t *out,const sh2z_view_t *v,
    const void *positions,size_t in_stride,size_t count) {
    if(in_stride-6u>(size_t)INT16_MAX) {
        /* The assembly loop keeps the stride step in 16 bits. */
        batch_portable(out,v,positions,in_stride,count);
        return;
    }
    divu_block_t b;
    for(unsigned j=0;j<3;++j) {
        b.r[j]=v->r[2][j];b.r[3+j]=v->r[0][j];b.r[6+j]=v->r[1][j];
    }
    b.cx=v->center_x;b.cy=v->center_y;
    b.left=v->clip[0];b.top=v->clip[1];b.right=v->clip[2];b.bottom=v->clip[3];
    b.skip=(int16_t)(in_stride-6u);
    b.t2=v->t[2];b.t0=v->t[0];b.t1=v->t[1];
    b.near_z=v->near_z;b.fx=v->focal_x;b.fy=v->focal_y;b.far_z=v->far_z;b.code=0;
    sh2zi_view_divu_run(out,&b,positions,count);
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
