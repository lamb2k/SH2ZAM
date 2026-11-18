/* vehicle-kernels-1: sh2z_view_batch through an index list, so a mesh
 * projects only the vertices its kept faces reference. A separate file, so
 * that every earlier view function compiles exactly as before. The SH-2
 * path instantiates the view-sh2-1 division pipeline (sh2zam_view_asm.h)
 * with an indexed load in place of the sequential one; the per-vertex code
 * after the load is the same instructions. MIT; SH2ZAM Contributors. */
#if defined(__GNUC__)
/* The shared header holds every view path's helpers; this file uses a few. */
#pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "sh2zam_view_internal.h"

static bool indexed_args_ok(const sh2z_view_vertex_t *out,const sh2z_view_t *v,const void *positions,
    size_t in_stride,size_t count,const uint8_t *index,size_t index_count) {
    if(!view_valid(v) || !count || !index || !batch_args_ok(out,positions,in_stride,count))return false;
    const size_t out_bytes=count*sizeof(*out);
    return (uintptr_t)index<=UINTPTR_MAX-index_count && !overlaps(out,out_bytes,index,index_count);
}
/* One record through the view-contract-1 batch: its definition, verbatim. */
static void indexed_portable(sh2z_view_vertex_t *out,const sh2z_view_t *v,const void *positions,
    size_t in_stride,const uint8_t *index,size_t index_count) {
    const unsigned char *p=positions;
    for(size_t k=0;k<index_count;++k) {
        const size_t i=index[k];
        (void)sh2z_view_batch(out+i,v,p+i*in_stride,in_stride,1);
    }
}

bool sh2z_view_batch_indexed(sh2z_view_vertex_t *out,const sh2z_view_t *v,
    const void *positions,size_t in_stride,size_t count,const uint8_t *index,
    size_t index_count) {
    if(!index_count)return true;
    if(!indexed_args_ok(out,v,positions,in_stride,count,index,index_count))return false;
    indexed_portable(out,v,positions,in_stride,index,index_count);
    return true;
}

#if SH2Z_DIVU_NATIVE
/* The block's scratch vertex keeps x, y, z unused by the pipeline (only its
 * sx/sy and outcode words are written, as the "previous" vertex before the
 * first one): this entry keeps the positions base at block + 64 and the
 * output base at block + 68 there, and the stride in the skip field. r5
 * walks the index list. Per vertex: index -> r4 = out + 20 * i and the
 * position at positions + stride * i, then the view-sh2-1 transform and
 * division pipeline unchanged. */
__asm__(
#include "sh2zam_view_asm.h"
"    .macro SH2ZI_VI_TRANSFORM\n"
"    mov.b   @r5+,r8\n"
"    extu.b  r8,r8\n"              /* i */
"    mov     #20,r0\n"
"    mulu.w  r0,r8\n"
"    mov     #68,r0\n"
"    mov.l   @(r0,r7),r4\n"        /* out base */
"    sts     macl,r1\n"
"    add     r1,r4\n"              /* r4 = out + 20 * i */
"    mov.w   @(30,r7),r0\n"        /* stride */
"    mulu.w  r0,r8\n"
"    mov     #64,r0\n"
"    mov.l   @(r0,r7),r1\n"        /* positions base */
"    sts     macl,r0\n"
"    add     r0,r1\n"
"    mov.w   @r1+,r8\n"
"    mov.w   @r1+,r9\n"
"    mov.w   @r1,r10\n"
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
"    .pushsection .text.sh2zi_view_divu_indexed,\"ax\",@progbits\n"
"    .align 2\n"
"    .global _sh2zi_view_divu_indexed\n"
"    .type _sh2zi_view_divu_indexed,@function\n"
"_sh2zi_view_divu_indexed:\n"
"    SH2ZI_VD_PROLOGUE\n"
".Lsh2zi_vd_loopVI:\n"
"    SH2ZI_VI_TRANSFORM\n"
"    SH2ZI_VD_REST VI\n"
"    .size _sh2zi_view_divu_indexed,.-_sh2zi_view_divu_indexed\n"
"    .popsection\n"
);
void sh2zi_view_divu_indexed(sh2z_view_vertex_t *out,const divu_block_t *block,
    const uint8_t *index,size_t index_count);
#endif

bool sh2z_view_batch_divu_indexed(sh2z_view_vertex_t *out,const sh2z_view_t *v,
    const void *positions,size_t in_stride,size_t count,const uint8_t *index,
    size_t index_count) {
    if(!index_count)return true;
    if(!indexed_args_ok(out,v,positions,in_stride,count,index,index_count))return false;
#if SH2Z_DIVU_NATIVE
    if(in_stride<=(size_t)INT16_MAX) {
        divu_block_t b;
        fill_block(&b,v,(int16_t)in_stride);
        b.scratch.x=(int32_t)(uintptr_t)positions;
        b.scratch.y=(int32_t)(uintptr_t)out;
        sh2zi_view_divu_indexed(out,&b,index,index_count);
        return true;
    }
#endif
    indexed_portable(out,v,positions,in_stride,index,index_count);
    return true;
}
