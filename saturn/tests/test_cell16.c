/* 16x16 cell placement against a straightforward per-cell model, plus buffer and
 * argument contracts. MIT; SH2ZAM Contributors. */
#include "sh2zam/cell16.h"
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAXC 300u
static uint64_t checks;
#define CHECK(e) do { ++checks; if(!(e)) { fprintf(stderr,"FAIL %s:%d: %s\n",__FILE__,__LINE__,#e); exit(EXIT_FAILURE); } } while(0)
static uint32_t rng=UINT32_C(0x6d2b79f5);
static uint32_t rnd(void) { rng^=rng<<13; rng^=rng>>17; rng^=rng<<5; return rng; }
static int16_t rs16(void) { return (int16_t)(uint16_t)rnd(); }

/* The definition, one cell at a time, written with explicit branches. */
static size_t model(sh2z_screen_t *pos,uint16_t *idx,const sh2z_screen_t *off,size_t n,int ox,int oy,
                    int bw,int bh,unsigned flip,sh2z_clip_rect_t c) {
    size_t k=0;
    for(size_t i=0;i<n;i++) {
        int x=off[i].x,y=off[i].y;
        if(flip&SH2Z_CELL16_FLIP_X) x=bw-16-x;
        if(flip&SH2Z_CELL16_FLIP_Y) y=bh-16-y;
        long sx=(long)ox+x,sy=(long)oy+y;
        if(sx+16>c.min_x&&sx<c.max_x&&sy+16>c.min_y&&sy<c.max_y) {
            pos[k].x=(int16_t)sx; pos[k].y=(int16_t)sy; idx[k]=(uint16_t)i; k++;
        }
    }
    return k;
}
static void random_case(unsigned iter) {
    static sh2z_screen_t off[MAXC],pos[MAXC],mpos[MAXC];
    static uint16_t idx[MAXC],midx[MAXC];
    static const size_t counts[]={0,1,2,3,16,17,64,200,MAXC};
    const size_t n=counts[iter%9u];
    const unsigned flip=(rnd()>>8)&3u;
    const int bw=(int)(rnd()%513u)-8,bh=(int)(rnd()%513u)-8;
    const unsigned mode=iter%4u;
    for(size_t i=0;i<n;i++) {
        if(mode==0) { off[i].x=(int16_t)((rnd()%32u)*16u); off[i].y=(int16_t)((rnd()%32u)*16u); } /* tile grid */
        else if(mode==1) { off[i].x=(int16_t)((int)(rnd()%2049u)-1024); off[i].y=(int16_t)((int)(rnd()%2049u)-1024); }
        else { off[i].x=rs16(); off[i].y=rs16(); }
    }
    const sh2z_cell16_xform_t xf={mode==3?rs16():(int)(rnd()%700u)-150,mode==3?rs16():(int)(rnd()%500u)-150,bw,bh,flip};
    sh2z_clip_rect_t c={0,0,384,224};
    if(iter%3u==1u) { c.min_x=(int16_t)((int)(rnd()%64u)-32); c.min_y=(int16_t)((int)(rnd()%64u)-32); c.max_x=(int16_t)(c.min_x+(int)(rnd()%400u)); c.max_y=(int16_t)(c.min_y+(int)(rnd()%300u)); }
    if(iter%3u==2u) { c.min_x=(int16_t)((int)(rnd()%65520u)-32752); c.min_y=(int16_t)((int)(rnd()%65520u)-32752); c.max_x=(int16_t)((int)rnd()); c.max_y=(int16_t)((int)rnd()); }
    const size_t want=(c.max_x>c.min_x&&c.max_y>c.min_y)?model(mpos,midx,off,n,xf.origin_x,xf.origin_y,bw,bh,flip,c):0;
    size_t vis=999;
    memset(pos,0x5A,sizeof pos); memset(idx,0x5A,sizeof idx);
    CHECK(sh2z_cell16_place(pos,idx,MAXC,&vis,off,n,&xf,&c));
    CHECK(vis==want);
    for(size_t k=0;k<want;k++) CHECK(pos[k].x==mpos[k].x&&pos[k].y==mpos[k].y&&idx[k]==midx[k]);
    for(size_t k=want;k<MAXC;k++) CHECK(pos[k].x==0x5A5A&&pos[k].y==0x5A5A&&idx[k]==0x5A5A); /* nothing beyond visible */
    size_t vis2=999;
    CHECK(sh2z_cell16_place(pos,NULL,n,&vis2,off,n,&xf,&c)&&vis2==want);
}
static void arguments(void) {
    static sh2z_screen_t off[8],pos[8]; static uint16_t idx[8];
    const sh2z_clip_rect_t c={0,0,384,224}; const sh2z_cell16_xform_t o={0,0,16,16,0};
    const sh2z_cell16_xform_t x4={0,0,16,16,4},xw={0,0,(1<<20)+1,16,0},xh={0,0,16,-(1<<20)-1,0},xo={(1<<20)+1,0,16,16,0};
    size_t vis=77; memset(pos,0x11,sizeof pos);
    CHECK(!sh2z_cell16_place(pos,idx,8,&vis,off,8,&xo,&c));
    CHECK(!sh2z_cell16_place(pos,idx,8,&vis,off,8,NULL,&c));
    CHECK(!sh2z_cell16_place(pos,idx,7,&vis,off,8,&o,&c)&&vis==77);          /* capacity < count */
    CHECK(!sh2z_cell16_place(pos,idx,8,NULL,off,8,&o,&c));
    CHECK(!sh2z_cell16_place(pos,idx,8,&vis,off,8,&o,NULL));
    CHECK(!sh2z_cell16_place(pos,idx,8,&vis,off,8,&x4,&c));                   /* undefined flip bit */
    CHECK(!sh2z_cell16_place(pos,idx,8,&vis,off,8,&xw,&c));
    CHECK(!sh2z_cell16_place(pos,idx,8,&vis,off,8,&xh,&c));
    CHECK(!sh2z_cell16_place(NULL,idx,8,&vis,off,8,&o,&c));
    CHECK(!sh2z_cell16_place(pos,idx,8,&vis,NULL,8,&o,&c));
    CHECK(!sh2z_cell16_place(pos,idx,70000,&vis,off,70000,&o,&c));           /* index would not fit 16 bits */
    CHECK(!sh2z_cell16_place(off,idx,8,&vis,off,8,&o,&c));                   /* output overlaps input */
    CHECK(!sh2z_cell16_place(pos,(uint16_t*)(void*)pos,8,&vis,off,8,&o,&c)); /* index overlaps positions */
    const sh2z_clip_rect_t low={-32753,0,10,10},ok={-32752,-32752,0,0};
    CHECK(!sh2z_cell16_place(pos,idx,8,&vis,off,8,&o,&low));
    CHECK(sh2z_cell16_place(pos,idx,8,&vis,off,8,&o,&ok)&&vis==0);           /* empty clip rectangle */
    CHECK(sh2z_cell16_place(NULL,NULL,0,&vis,NULL,0,&o,&c)&&vis==0);
    CHECK(pos[0].x==0x1111);
    /* worked example: 2x2 block of 16-pixel cells, mirrored */
    sh2z_screen_t cells[4]={{0,0},{16,0},{0,16},{16,16}},out[4]; uint16_t oi[4]; size_t n=0;
    CHECK(sh2z_cell16_place(out,oi,4,&n,cells,4,&(sh2z_cell16_xform_t){100,50,32,32,SH2Z_CELL16_FLIP_X},&c)&&n==4);
    CHECK(out[0].x==116&&out[0].y==50&&out[1].x==100&&out[1].y==50&&out[2].x==116&&out[2].y==66&&out[3].x==100&&out[3].y==66);
    CHECK(sh2z_cell16_place(out,oi,4,&n,cells,4,&(sh2z_cell16_xform_t){-15,0,32,32,0},&c)&&n==4);   /* 1 px of cell 0 visible */
    CHECK(sh2z_cell16_place(out,oi,4,&n,cells,4,&(sh2z_cell16_xform_t){-16,0,32,32,0},&c)&&n==2&&oi[0]==1&&oi[1]==3); /* cell 0 and 2 just off */
}
int main(void) {
    arguments();
    for(unsigned i=0;i<20000;i++) random_case(i);
    printf("cell16: %" PRIu64 " checks passed\n",checks);
    return 0;
}
