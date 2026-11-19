/* Host tests for rotation sprite sets (contract tag rotation-sprites-1):
 * validation, selection against a floating-point reference over the whole
 * sphere, placement and distorted-sprite corners against references, the
 * command words, mirroring and every rejection path.
 *
 * Copyright (c) 2026 SH2ZAM Contributors. MIT License.
 * Inspired by SH4ZAM by Falco Girgis and SH4ZAM Contributors.
 */
#include "sh2zam/rotation_sprite_experimental.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static unsigned failures;
#define CHECK(cond) do { if(!(cond)) { ++failures; fprintf(stderr,"%s:%d: %s\n",__FILE__,__LINE__,#cond); } } while(0)

#define PI 3.14159265358979323846
enum {YAW=16,PITCH=3,CELLS=YAW*PITCH,CELL_W=24,CELL_H=16};

typedef struct {
    sh2z_rspr_set_t header;
    sh2z_rspr_cell_t cells[CELLS];
    uint16_t cluts[2][16];
    uint8_t pixels[CELLS][CELL_W*CELL_H/2];
} test_set_t;

static double yaw_deg[YAW],pitch_deg[PITCH]={-20.0,10.0,40.0};

static void build(test_set_t *s,bool mirror) {
    memset(s,0,sizeof(*s));
    s->header.magic=SH2Z_RSPR_MAGIC;s->header.bytes=(uint32_t)sizeof(*s);
    s->header.yaw_count=YAW;s->header.pitch_count=PITCH;s->header.flags=mirror?SH2Z_RSPR_MIRROR:0;
    s->header.clut_count=2;s->header.clut_offset=(uint32_t)offsetof(test_set_t,cluts);
    s->header.pixels_per_unit=3*65536/2; /* 1.5 cell pixels per model unit */
    for(unsigned k=0;k<YAW;++k) {
        /* Mirrored: 0..180 with a dense rear; full: -180..180 evenly. */
        yaw_deg[k]=mirror?(k<9?k*6.0:48.0+(k-8)*(132.0/7.0)):-180.0+k*(360.0/YAW);
        s->header.yaw[k][0]=(int16_t)lrint(16384.0*sin(yaw_deg[k]*PI/180.0));
        s->header.yaw[k][1]=(int16_t)lrint(16384.0*cos(yaw_deg[k]*PI/180.0));
    }
    for(unsigned k=0;k<PITCH;++k) {
        s->header.pitch[k][0]=(int16_t)lrint(16384.0*sin(pitch_deg[k]*PI/180.0));
        s->header.pitch[k][1]=(int16_t)lrint(16384.0*cos(pitch_deg[k]*PI/180.0));
    }
    for(unsigned i=0;i<CELLS;++i) {
        s->cells[i]=(sh2z_rspr_cell_t){.width=CELL_W,.height=CELL_H,.origin_x=10,.origin_y=12,
            .pixel_offset=(uint32_t)(offsetof(test_set_t,pixels)+i*sizeof(s->pixels[0])),.clut=(uint16_t)(i&1u)};
    }
}

/* Reference: the bin whose centre direction has the largest dot product with
 * the input, computed in double. Returns the margin to the runner-up. */
static unsigned ref_yaw(const test_set_t *s,double x,double z,double *margin) {
    double best=-1e300,second=-1e300;unsigned k_best=0;
    for(unsigned k=0;k<YAW;++k) {
        const double d=s->header.yaw[k][0]*x+s->header.yaw[k][1]*z;
        if(d>best){second=best;best=d;k_best=k;}else if(d>second)second=d;
    }
    *margin=(best-second)/16384.0;return k_best;
}

static void test_valid(void) {
    static test_set_t s;build(&s,true);
    CHECK(sh2z_rspr_valid(&s.header));
    CHECK(!sh2z_rspr_valid(NULL));
    test_set_t *t=malloc(sizeof(*t));
    #define BREAK(stmt) do { memcpy(t,&s,sizeof(s)); stmt; CHECK(!sh2z_rspr_valid(&t->header)); } while(0)
    BREAK(t->header.magic^=1u);
    BREAK(t->header.yaw_count=0);
    BREAK(t->header.yaw_count=SH2Z_RSPR_MAX_YAW+1);
    BREAK(t->header.pitch_count=SH2Z_RSPR_MAX_PITCH+1);
    BREAK(t->header.flags=2);
    BREAK(t->header.clut_count=0);
    BREAK(t->header.clut_offset+=4);
    BREAK(t->header.pixels_per_unit=0);
    BREAK(t->header.bytes=t->header.clut_offset+16);
    BREAK(t->cells[3].width=20);
    BREAK(t->cells[3].width=512);
    BREAK(t->cells[3].height=0);
    BREAK(t->cells[3].pixel_offset+=4);
    BREAK(t->cells[3].pixel_offset=8);
    BREAK(t->cells[CELLS-1].pixel_offset=t->header.bytes-8);
    BREAK(t->cells[3].clut=2);
    #undef BREAK
    free(t);
}

static void test_select(bool mirror) {
    static test_set_t s;build(&s,mirror);
    sh2z_rspr_pick_t pick;
    CHECK(!sh2z_rspr_select(&pick,&s.header,0,0,0));
    CHECK(!sh2z_rspr_select(NULL,&s.header,1,0,0));
    unsigned checked=0,ties=0;
    const double magnitudes[]={3.0,250.0,30000.0,2.0e9};
    for(unsigned m=0;m<4;++m)for(int py=-80;py<=80;py+=4)for(int yw=-180;yw<180;++yw) {
        const double a=yw*PI/180.0,b=py*PI/180.0,r=magnitudes[m];
        const double fx=r*cos(b)*sin(a),fy=r*sin(b),fz=r*cos(b)*cos(a);
        const int32_t x=(int32_t)lrint(fx),y=(int32_t)lrint(fy),z=(int32_t)lrint(fz);
        if(x==0&&y==0&&z==0)continue;
        if(!sh2z_rspr_select(&pick,&s.header,x,y,z)){CHECK(false);continue;}
        /* Reference on the same shifted integers the contract defines. */
        int32_t sx=x,sy=y,sz=z;
        while(labs((long)sx)>=32768||labs((long)sy)>=32768||labs((long)sz)>=32768){sx>>=1;sy>>=1;sz>>=1;}
        const bool flip=mirror&&sx<0;if(flip)sx=-sx;
        double margin;const unsigned yaw=ref_yaw(&s,sx,sz,&margin);
        const double h=floor((s.header.yaw[yaw][0]*(double)sx+s.header.yaw[yaw][1]*(double)sz)/16384.0);
        unsigned pitch=0;double best=-1e300;
        for(unsigned k=0;k<PITCH;++k) {
            const double d=s.header.pitch[k][0]*(double)sy+s.header.pitch[k][1]*h;
            if(d>best){best=d;pitch=k;}
        }
        if(margin<1e-9){++ties;continue;}
        CHECK(pick.cell==pitch*YAW+yaw);
        CHECK(pick.mirror==(flip?1:0));
        ++checked;
    }
    CHECK(checked>40000);
    printf("select(mirror=%d): %u directions, %u exact ties skipped\n",(int)mirror,checked,ties);
    /* Angular sanity: a camera straight behind (+z) at level picks the 0/near-0 bin. */
    CHECK(sh2z_rspr_select(&pick,&s.header,0,0,1000));
    if(mirror)CHECK(pick.cell%YAW==0);
}

/* A model view whose up axis rolls by alpha on screen, with model scale k. */
static sh2z_view_t view_for(double alpha,double k,int32_t tx,int32_t ty,int32_t tz) {
    sh2z_view_t v;memset(&v,0,sizeof(v));
    /* Model up = view (sin(-alpha)... ) : screen up rotated clockwise by alpha. */
    const double ux=-sin(alpha),uy=cos(alpha);
    v.r[0][0]=(int16_t)lrint(16384.0*k*cos(alpha));v.r[1][0]=(int16_t)lrint(16384.0*k*sin(alpha));
    v.r[0][1]=(int16_t)lrint(16384.0*k*ux);v.r[1][1]=(int16_t)lrint(16384.0*k*uy);
    v.r[2][2]=(int16_t)lrint(16384.0*k);
    v.t[0]=tx;v.t[1]=ty;v.t[2]=tz;
    v.focal_x=200*65536;v.focal_y=180*65536;v.center_x=160;v.center_y=112;
    v.near_z=16384;v.far_z=1<<30;v.clip[0]=0;v.clip[1]=0;v.clip[2]=319;v.clip[3]=223;
    return v;
}

static void test_place_emit(void) {
    static test_set_t s;build(&s,true);
    sh2z_rspr_place_t place;sh2z_rspr_pick_t pick={.cell=5,.mirror=0};
    unsigned cases=0;
    for(int deg=-180;deg<180;deg+=7)for(int depth=2;depth<=40;depth+=3) {
        const double alpha=deg*PI/180.0,k=0.75;
        const int32_t tz=depth*16384*8,tx=-123456,ty=98765;
        const sh2z_view_t v=view_for(alpha,k,tx,ty,tz);
        if(!sh2z_rspr_place(&place,&v,&s.header)){CHECK(false);continue;}
        const int64_t qx=((int64_t)v.focal_x*tx)/tz,qy=((int64_t)v.focal_y*ty)/tz;
        CHECK(place.x==(int16_t)(v.center_x+(qx>>16)));
        CHECK(place.y==(int16_t)(v.center_y-(qy>>16)));
        /* up = {ux, -uy} on screen. */
        CHECK(fabs(place.up[0]/16384.0-(-sin(alpha)))<2.5e-4);
        CHECK(fabs(place.up[1]/16384.0-(-cos(alpha)))<2.5e-4);
        const double want=180.0*k/(tz/16384.0)/1.5;
        CHECK(fabs(place.scale/65536.0-want)<=want*1e-3+2.0/65536.0);
        uint16_t cmd[16];
        for(unsigned mirror=0;mirror<2;++mirror) {
            pick.mirror=(uint8_t)mirror;
            if(!sh2z_rspr_emit(cmd,&s.header,&pick,&place,0x10000u,0x0088u)){CHECK(false);continue;}
            CHECK(cmd[0]==0x0002u&&cmd[1]==0&&cmd[2]==0x0088u&&cmd[14]==0&&cmd[15]==0);
            CHECK(cmd[3]==(uint16_t)((0x10000u+s.header.clut_offset+s.cells[5].clut*32u)>>3));
            CHECK(cmd[4]==(uint16_t)((0x10000u+s.cells[5].pixel_offset)>>3));
            CHECK(cmd[5]==(uint16_t)((CELL_W/8)<<8|CELL_H));
            const double sc=place.scale/65536.0,ux=place.up[0]/16384.0,uy=place.up[1]/16384.0;
            const int corner[4][2]={{0,0},{CELL_W,0},{CELL_W,CELL_H},{0,CELL_H}};
            for(unsigned c=0;c<4;++c) {
                double px=corner[c][0]-s.cells[5].origin_x,py=corner[c][1]-s.cells[5].origin_y;
                if(mirror)px=-px;
                const double ex=place.x+sc*(px*(-uy)-py*ux),ey=place.y+sc*(px*ux-py*uy);
                CHECK(fabs((int16_t)cmd[6+2*c]-ex)<=1.0&&fabs((int16_t)cmd[7+2*c]-ey)<=1.0);
            }
            ++cases;
        }
    }
    printf("place/emit: %u cases\n",cases);
    /* Rejections. */
    sh2z_view_t v=view_for(0.3,1.0,0,0,16384);CHECK(!sh2z_rspr_place(&place,&v,&s.header)); /* at near */
    v=view_for(0.3,1.0,0,0,-163840);CHECK(!sh2z_rspr_place(&place,&v,&s.header));           /* behind */
    v=view_for(0.3,1.0,1<<29,0,16384*20);CHECK(!sh2z_rspr_place(&place,&v,&s.header));      /* off screen */
    v=view_for(0.0,1.0,0,0,16384*20);v.r[0][1]=0;v.r[1][1]=0;v.r[2][1]=16384;
    CHECK(!sh2z_rspr_place(&place,&v,&s.header));                                           /* up along view */
    v=view_for(0.0,1.0,0,0,16384*20);CHECK(sh2z_rspr_place(&place,&v,&s.header));
    uint16_t cmd[16],keep[16];memset(cmd,0x5a,sizeof(cmd));memcpy(keep,cmd,sizeof(cmd));
    pick=(sh2z_rspr_pick_t){.cell=CELLS,.mirror=0};CHECK(!sh2z_rspr_emit(cmd,&s.header,&pick,&place,0,0));
    pick.cell=0;pick.mirror=2;CHECK(!sh2z_rspr_emit(cmd,&s.header,&pick,&place,0,0));
    pick.mirror=0;CHECK(!sh2z_rspr_emit(cmd,&s.header,&pick,&place,4,0));
    CHECK(!sh2z_rspr_emit(cmd,&s.header,&pick,&place,0x80000u-8u,0));
    sh2z_rspr_place_t far=place;far.x=1020;far.scale=place.scale*8;
    CHECK(!sh2z_rspr_emit(cmd,&s.header,&pick,&far,0,0));               /* corner off range */
    far=place;far.scale=(1<<24)+1;CHECK(!sh2z_rspr_emit(cmd,&s.header,&pick,&far,0,0));
    CHECK(memcmp(cmd,keep,sizeof(cmd))==0);
}

int main(void) {
    test_valid();
    test_select(true);
    test_select(false);
    test_place_emit();
    if(failures){fprintf(stderr,"rotation_sprite: %u failures\n",failures);return 1;}
    puts("rotation_sprite: all checks passed");
    return 0;
}
