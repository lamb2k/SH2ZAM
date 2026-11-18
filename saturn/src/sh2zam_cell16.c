/* Batch placement of 16x16 sprite cells.
 *
 * Copyright (c) 2026 SH2ZAM Contributors. MIT License.
 * Inspired by SH4ZAM by Falco Girgis and SH4ZAM Contributors.
 */
#include "sh2zam/cell16.h"

static bool cell_overlap(const void *a,size_t an,const void *b,size_t bn) {
    if(!an||!bn) return false;
    const uintptr_t aa=(uintptr_t)a,bb=(uintptr_t)b;
    return aa>=bb?aa-bb<bn:bb-aa<an;
}

bool sh2z_cell16_place(sh2z_screen_t *out_pos,uint16_t *out_index,size_t out_capacity,size_t *visible,
    const sh2z_screen_t *offsets,size_t count,const sh2z_cell16_xform_t *xform,const sh2z_clip_rect_t *clip) {
    if(!visible||!clip||!xform||(uintptr_t)xform%_Alignof(sh2z_cell16_xform_t)||count>65536u||out_capacity<count) return false;
    const int32_t origin_x=xform->origin_x,origin_y=xform->origin_y,block_w=xform->block_w,block_h=xform->block_h;
    const uint32_t flip=xform->flip;
    if((flip&~(SH2Z_CELL16_FLIP_X|SH2Z_CELL16_FLIP_Y))||block_w>(1<<20)||block_w<-(1<<20)||block_h>(1<<20)||block_h<-(1<<20)||
       origin_x>(1<<20)||origin_x<-(1<<20)||origin_y>(1<<20)||origin_y<-(1<<20)) return false;
    if(clip->min_x< -32752||clip->min_y< -32752||(uintptr_t)clip%_Alignof(sh2z_clip_rect_t)||
       (uintptr_t)visible%_Alignof(size_t)) return false;
    if(count) {
        const size_t pos_bytes=count*sizeof *out_pos;
        if(!out_pos||!offsets||(uintptr_t)out_pos%_Alignof(sh2z_screen_t)||(uintptr_t)offsets%_Alignof(sh2z_screen_t)||
           cell_overlap(out_pos,pos_bytes,offsets,count*sizeof *offsets)||
           cell_overlap(visible,sizeof *visible,out_pos,pos_bytes)||cell_overlap(visible,sizeof *visible,offsets,count*sizeof *offsets)) return false;
        if(out_index) {
            if((uintptr_t)out_index%_Alignof(uint16_t)||cell_overlap(out_index,count*sizeof *out_index,out_pos,pos_bytes)||
               cell_overlap(out_index,count*sizeof *out_index,offsets,count*sizeof *offsets)||
               cell_overlap(visible,sizeof *visible,out_index,count*sizeof *out_index)) return false;
        }
    }
    const int32_t min_x=clip->min_x,min_y=clip->min_y,max_x=clip->max_x,max_y=clip->max_y;
    if(max_x<=min_x||max_y<=min_y||!count) { *visible=0; return true; }
    /* The clip test runs in offset space, so mirrored cells need no per-cell
     * arithmetic before it: a cell is visible iff its raw offset lies in
     * [xlo, xlo+span_x] x [ylo, ylo+span_y], one unsigned compare per axis. The
     * mirror, (v ^ m) - m with m = 0 or -1, is applied only to visible cells.
     * Unmirrored: s = origin + v in [min-15, max-1]; mirrored: s = (origin +
     * block - 16) - v in the same range. */
    const int32_t mx=(flip&SH2Z_CELL16_FLIP_X)?-1:0,my=(flip&SH2Z_CELL16_FLIP_Y)?-1:0;
    const int32_t base_x=origin_x+(mx?block_w-SH2Z_CELL16_SIZE:0),base_y=origin_y+(my?block_h-SH2Z_CELL16_SIZE:0);
    const int32_t first_x=min_x-(SH2Z_CELL16_SIZE-1),first_y=min_y-(SH2Z_CELL16_SIZE-1);
    const uint32_t span_x=(uint32_t)(max_x-first_x-1),span_y=(uint32_t)(max_y-first_y-1);
    const int32_t xlo=mx?base_x-(max_x-1):first_x-base_x,ylo=my?base_y-(max_y-1):first_y-base_y;
    size_t n=0;
    for(size_t i=0;i<count;i++) {
        const int32_t x=offsets[i].x,y=offsets[i].y;
        if((uint32_t)(x-xlo)<=span_x&&(uint32_t)(y-ylo)<=span_y) {
            out_pos[n].x=(int16_t)(base_x+((x^mx)-mx)); out_pos[n].y=(int16_t)(base_y+((y^my)-my));
            if(out_index) out_index[n]=(uint16_t)i;
            n++;
        }
    }
    *visible=n;
    return true;
}
