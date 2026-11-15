/* SH-2 bridge for the executed-instruction integer model. MIT; Copyright (c) 2026
 * SH2ZAM Contributors. Every entry point is called directly by the model harness
 * with the SH-2 calling convention; "fast_" entries run the library, "plain_"
 * entries are the independent straightforward C control for the same contract
 * (same inputs, same outputs, same flag nibble). */
#include "sh2zam/m68k.h"

volatile uint32_t ccr_out;

#define NOINLINE __attribute__((noinline))

NOINLINE uint32_t fast_mulu_w(uint32_t dn,uint32_t src) { uint32_t v=dn; ccr_out=sh2z_m68k_mulu_w(&v,(uint16_t)src); return v; }
NOINLINE uint32_t fast_muls_w(uint32_t dn,uint32_t src) { uint32_t v=dn; ccr_out=sh2z_m68k_muls_w(&v,(uint16_t)src); return v; }
NOINLINE uint32_t fast_divu_w(uint32_t dn,uint32_t src) { uint32_t v=dn; ccr_out=sh2z_m68k_divu_w(&v,(uint16_t)src); return v; }
NOINLINE uint32_t fast_divs_w(uint32_t dn,uint32_t src) { uint32_t v=dn; ccr_out=sh2z_m68k_divs_w(&v,(uint16_t)src); return v; }

/* Plain C controls: what a lifted 68000 routine would otherwise write (C division
 * and remainder operators). They share the library's N/Z helpers so the comparison
 * isolates the arithmetic, not flag generation. */
NOINLINE uint32_t plain_mulu_w(uint32_t dn,uint32_t src) {
    uint32_t r=(dn&0xFFFFu)*(src&0xFFFFu);
    ccr_out=sh2z_m68k_nz32(r);
    return r;
}
NOINLINE uint32_t plain_muls_w(uint32_t dn,uint32_t src) {
    int32_t r=(int32_t)(int16_t)dn*(int32_t)(int16_t)src;
    ccr_out=sh2z_m68k_nz32((uint32_t)r);
    return (uint32_t)r;
}
NOINLINE uint32_t plain_divu_w(uint32_t dn,uint32_t src) {
    uint32_t d=src&0xFFFFu;
    if(!d) { ccr_out=SH2Z_M68K_DIV_ZERO; return dn; }
    uint32_t q=dn/d,r=dn%d;
    if(q>0xFFFFu) { ccr_out=SH2Z_M68K_V|SH2Z_M68K_N; return dn; }
    ccr_out=sh2z_m68k_nz16(q);
    return (r<<16)|q;
}
NOINLINE uint32_t plain_divs_w(uint32_t dn,uint32_t src) {
    int32_t n=(int32_t)dn,d=(int16_t)src;
    if(!d) { ccr_out=SH2Z_M68K_DIV_ZERO; return dn; }
    int32_t q=n/d,r=n%d;
    if(q>32767||q<-32768) { ccr_out=SH2Z_M68K_V|SH2Z_M68K_N; return dn; }
    ccr_out=sh2z_m68k_nz16((uint32_t)q);
    return (((uint32_t)r&0xFFFFu)<<16)|((uint32_t)q&0xFFFFu);
}

/* ---- box overlap ---- (buffers are written by the harness before each call) */
#include "sh2zam/box2.h"
#define BOX_MAX_A 64u
#define BOX_MAX_B 256u
sh2z_ibox2_t box_query,box_a[BOX_MAX_A],box_b[BOX_MAX_B];
sh2z_ibox2c_t boxc_query,boxc_a[BOX_MAX_A],boxc_b[BOX_MAX_B];
uint32_t box_out[BOX_MAX_A*(BOX_MAX_B/32u)];
volatile uint32_t box_hits;

NOINLINE uint32_t fast_box_one_many(uint32_t count,uint32_t edges) {
    size_t n=0;
    uint32_t ok=sh2z_ibox2_overlap_one_many(box_out,BOX_MAX_B/32u,&n,&box_query,box_b,count,(sh2z_box_edges_t)edges);
    box_hits=(uint32_t)n; return ok;
}
NOINLINE uint32_t fast_box_one_many_unchecked(uint32_t count,uint32_t edges) {
    box_hits=(uint32_t)sh2z_ibox2_overlap_one_many_unchecked(box_out,&box_query,box_b,count,(sh2z_box_edges_t)edges); return 1;
}
NOINLINE uint32_t fast_boxc_one_many_unchecked(uint32_t count,uint32_t edges) {
    box_hits=(uint32_t)sh2z_ibox2c_overlap_one_many_unchecked(box_out,&boxc_query,boxc_b,count,(sh2z_box_edges_t)edges); return 1;
}
NOINLINE uint32_t fast_boxc_one_many(uint32_t count,uint32_t edges) {
    size_t n=0;
    uint32_t ok=sh2z_ibox2c_overlap_one_many(box_out,BOX_MAX_B/32u,&n,&boxc_query,boxc_b,count,(sh2z_box_edges_t)edges);
    box_hits=(uint32_t)n; return ok;
}
NOINLINE uint32_t fast_box_many_many(uint32_t a_count,uint32_t b_count,uint32_t edges) {
    size_t n=0;
    uint32_t ok=sh2z_ibox2_overlap_many_many(box_out,sizeof box_out/sizeof box_out[0],&n,box_a,a_count,box_b,b_count,(sh2z_box_edges_t)edges);
    box_hits=(uint32_t)n; return ok;
}
NOINLINE uint32_t fast_boxc_many_many(uint32_t a_count,uint32_t b_count,uint32_t edges) {
    size_t n=0;
    uint32_t ok=sh2z_ibox2c_overlap_many_many(box_out,sizeof box_out/sizeof box_out[0],&n,boxc_a,a_count,boxc_b,b_count,(sh2z_box_edges_t)edges);
    box_hits=(uint32_t)n; return ok;
}
/* Plain C controls: the usual loop that tests each pair and sets bit (i & 31),
 * counting hits; no argument validation (the library validates once per call). */
static inline int plain_mm(const sh2z_ibox2_t *a,const sh2z_ibox2_t *b,uint32_t edges) {
    return edges ? a->min_x<b->max_x&&b->min_x<a->max_x&&a->min_y<b->max_y&&b->min_y<a->max_y
                 : a->min_x<=b->max_x&&b->min_x<=a->max_x&&a->min_y<=b->max_y&&b->min_y<=a->max_y;
}
static inline uint32_t plain_absdiff(int32_t a,int32_t b) { return a>=b?(uint32_t)a-(uint32_t)b:(uint32_t)b-(uint32_t)a; }
static inline int plain_c(const sh2z_ibox2c_t *a,const sh2z_ibox2c_t *b,uint32_t edges) {
    const uint32_t dx=plain_absdiff(a->cx,b->cx),dy=plain_absdiff(a->cy,b->cy);
    const uint32_t sx=(uint32_t)a->hx+(uint32_t)b->hx,sy=(uint32_t)a->hy+(uint32_t)b->hy;
    return edges ? dx<sx&&dy<sy : dx<=sx&&dy<=sy;
}
NOINLINE uint32_t plain_box_one_many(uint32_t count,uint32_t edges) {
    uint32_t n=0;
    for(uint32_t w=0;w<(count+31u)/32u;w++) box_out[w]=0;
    for(uint32_t i=0;i<count;i++) if(plain_mm(&box_query,&box_b[i],edges)) { box_out[i>>5]|=1u<<(i&31u); n++; }
    box_hits=n; return 1;
}
NOINLINE uint32_t plain_boxc_one_many(uint32_t count,uint32_t edges) {
    uint32_t n=0;
    for(uint32_t w=0;w<(count+31u)/32u;w++) box_out[w]=0;
    for(uint32_t i=0;i<count;i++) if(plain_c(&boxc_query,&boxc_b[i],edges)) { box_out[i>>5]|=1u<<(i&31u); n++; }
    box_hits=n; return 1;
}
NOINLINE uint32_t plain_box_many_many(uint32_t a_count,uint32_t b_count,uint32_t edges) {
    uint32_t n=0,rw=(b_count+31u)/32u;
    for(uint32_t w=0;w<a_count*rw;w++) box_out[w]=0;
    for(uint32_t r=0;r<a_count;r++) for(uint32_t i=0;i<b_count;i++)
        if(plain_mm(&box_a[r],&box_b[i],edges)) { box_out[r*rw+(i>>5)]|=1u<<(i&31u); n++; }
    box_hits=n; return 1;
}
NOINLINE uint32_t plain_boxc_many_many(uint32_t a_count,uint32_t b_count,uint32_t edges) {
    uint32_t n=0,rw=(b_count+31u)/32u;
    for(uint32_t w=0;w<a_count*rw;w++) box_out[w]=0;
    for(uint32_t r=0;r<a_count;r++) for(uint32_t i=0;i<b_count;i++)
        if(plain_c(&boxc_a[r],&boxc_b[i],edges)) { box_out[r*rw+(i>>5)]|=1u<<(i&31u); n++; }
    box_hits=n; return 1;
}

/* ---- 16x16 cell placement ---- */
#include "sh2zam/cell16.h"
#define CELL_MAX 512u
sh2z_screen_t cell_off[CELL_MAX],cell_pos[CELL_MAX];
sh2z_cell16_xform_t cell_xform;
uint16_t cell_idx[CELL_MAX];
sh2z_clip_rect_t cell_clip;
volatile uint32_t cell_visible;

NOINLINE uint32_t fast_cell_place(uint32_t count,uint32_t flip) {
    size_t n=0;
    cell_xform.flip=flip;
    uint32_t ok=sh2z_cell16_place(cell_pos,cell_idx,CELL_MAX,&n,cell_off,count,&cell_xform,&cell_clip);
    cell_visible=(uint32_t)n; return ok;
}
/* Plain C control: per-cell branches for the flips and four clip comparisons, no
 * argument validation. */
NOINLINE uint32_t plain_cell_place(uint32_t count,uint32_t flip) {
    uint32_t n=0;
    for(uint32_t i=0;i<count;i++) {
        int32_t x=cell_off[i].x,y=cell_off[i].y;
        if(flip&1u) x=cell_xform.block_w-16-x;
        if(flip&2u) y=cell_xform.block_h-16-y;
        x+=cell_xform.origin_x; y+=cell_xform.origin_y;
        if(x+16>cell_clip.min_x&&x<cell_clip.max_x&&y+16>cell_clip.min_y&&y<cell_clip.max_y) {
            cell_pos[n].x=(int16_t)x; cell_pos[n].y=(int16_t)y; cell_idx[n]=(uint16_t)i; n++;
        }
    }
    cell_visible=n; return 1;
}
