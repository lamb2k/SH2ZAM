/* Host tests for rotation sprite sets (contract tag rotation-sprites-2):
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
        const int64_t ax=(int64_t)v.center_x*65536+qx,ay=(int64_t)v.center_y*65536-qy;
        CHECK(place.x==(int16_t)(ax>>16)&&place.frac[0]==(uint16_t)(ax&0xffff));
        CHECK(place.y==(int16_t)(ay>>16)&&place.frac[1]==(uint16_t)(ay&0xffff));
        /* up = {ux, -uy} on screen. */
        CHECK(fabs(place.up[0]/16384.0-(-sin(alpha)))<2.5e-4);
        CHECK(fabs(place.up[1]/16384.0-(-cos(alpha)))<2.5e-4);
        const double want=180.0*k/(tz/16384.0)/1.5;
        CHECK(fabs(place.scale/65536.0-want)<=want*1e-3+2.0/65536.0);
        uint16_t cmd[16];
        for(unsigned mirror=0;mirror<2;++mirror) {
            pick.mirror=(uint8_t)mirror;
            if(!sh2z_rspr_emit(cmd,&s.header,&pick,&place,0x10000u,0x0088u)){CHECK(false);continue;}
            uint16_t again[16];
            CHECK(sh2z_rspr_emit_valid(again,&s.header,&pick,&place,0x10000u,0x0088u)&&memcmp(again,cmd,sizeof(again))==0);
            CHECK(cmd[0]==0x0002u&&cmd[1]==0&&cmd[2]==0x0088u&&cmd[14]==0&&cmd[15]==0);
            CHECK(cmd[3]==(uint16_t)((0x10000u+s.header.clut_offset+s.cells[5].clut*32u)>>3));
            CHECK(cmd[4]==(uint16_t)((0x10000u+s.cells[5].pixel_offset)>>3));
            CHECK(cmd[5]==(uint16_t)((CELL_W/8)<<8|CELL_H));
            const double sc=place.scale/65536.0,ux=place.up[0]/16384.0,uy=place.up[1]/16384.0;
            const int corner[4][2]={{0,0},{CELL_W,0},{CELL_W,CELL_H},{0,CELL_H}};
            for(unsigned c=0;c<4;++c) {
                double px=corner[c][0]-s.cells[5].origin_x,py=corner[c][1]-s.cells[5].origin_y;
                if(mirror)px=-px;
                const double ox=place.x+place.frac[0]/65536.0,oy=place.y+place.frac[1]/65536.0;
                const double ex=ox+sc*(px*(-uy)-py*ux),ey=oy+sc*(px*ux-py*uy);
                /* One rounding from the sub-pixel origin: within half a pixel plus the Q14 up error. */
                CHECK(fabs((int16_t)cmd[6+2*c]-ex)<=0.5+1e-3*sc*CELL_W&&fabs((int16_t)cmd[7+2*c]-ey)<=0.5+1e-3*sc*CELL_W);
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


/* Rotation from yaw (about y), pitch (about x) and roll (about z), row-major. */
static void rot(double m[3][3],double a,double b,double c) {
    const double ca=cos(a),sa=sin(a),cb=cos(b),sb=sin(b),cc=cos(c),sc=sin(c);
    const double y[3][3]={{ca,0,sa},{0,1,0},{-sa,0,ca}},x[3][3]={{1,0,0},{0,cb,-sb},{0,sb,cb}},z[3][3]={{cc,-sc,0},{sc,cc,0},{0,0,1}};
    double t[3][3];
    for(int i=0;i<3;++i)for(int j=0;j<3;++j){t[i][j]=0;for(int k=0;k<3;++k)t[i][j]+=x[i][k]*y[k][j];}
    for(int i=0;i<3;++i)for(int j=0;j<3;++j){m[i][j]=0;for(int k=0;k<3;++k)m[i][j]+=z[i][k]*t[k][j];}
}

/* An object held still on screen, its origin a hair either side of a pixel
 * boundary (the chase-camera case: the player at the screen centre with
 * rounding noise in the view): every corner must stay put. rotation-sprites-1
 * floored the origin and then rounded the offsets, so its sprite stepped a
 * whole pixel with the sign of the noise. */
static void test_still(void) {
    static test_set_t s;build(&s,true);
    sh2z_rspr_camera_t cam;memset(&cam,0,sizeof(cam));
    cam.focal_x=139*65536+20000;cam.focal_y=cam.focal_x;cam.center_x=160;cam.center_y=112;cam.near_z=16384;
    cam.r[0][0]=16384;cam.r[1][1]=16384;cam.r[2][2]=16384;
    CHECK(sh2z_rspr_camera_scale(&cam,&s.header,1638));
    const int16_t basis[9]={0,0,-16384, 0,16384,0, -16384,0,0}; /* seen from behind */
    unsigned steps=0,cases=0;
    for(int32_t lift=-40;lift<=40;lift+=8) for(int32_t depth=60;depth<=400;depth+=17) {
        uint16_t first[16];
        for(int32_t e=-3;e<=3;++e) { /* view-space x of a few 1/65536 units either side of 0 */
            const int32_t pos[3]={e,lift*65536,depth*65536};
            sh2z_rspr_pick_t pick;sh2z_rspr_place_t place;int32_t dz;uint16_t cmd[16];
            if(!sh2z_rspr_pose(&pick,&place,&dz,&cam,&s.header,pos,basis)){CHECK(false);continue;}
            if(!sh2z_rspr_emit_valid(cmd,&s.header,&pick,&place,0,0)){CHECK(false);continue;}
            if(e==-3)memcpy(first,cmd,sizeof(cmd));
            else if(memcmp(first+6,cmd+6,16)!=0)++steps;
            ++cases;
        }
    }
    CHECK(cases>400);CHECK(steps==0);
    printf("still: %u poses, %u corner steps\n",cases,steps);
}

static void test_pose(void) {
    static test_set_t s;build(&s,true);
    sh2z_rspr_camera_t cam;memset(&cam,0,sizeof(cam));
    cam.focal_x=200*65536;cam.focal_y=180*65536;cam.center_x=160;cam.center_y=112;cam.near_z=16384;
    CHECK(!sh2z_rspr_camera_scale(&cam,&s.header,0));
    CHECK(sh2z_rspr_camera_scale(&cam,&s.header,12288)); /* model scale 0.75 */
    srand(12345);
    unsigned placed=0,cells=0,ties=0;
    for(unsigned n=0;n<20000;++n) {
        double R[3][3],B[3][3];
        rot(R,(rand()%6283)/1000.0,((rand()%1000)-500)/1000.0,((rand()%1000)-500)/2000.0);
        rot(B,(rand()%6283)/1000.0,((rand()%600)-300)/1000.0,((rand()%1000)-500)/1000.0);
        int16_t basis[9];
        /* basis rows: model +z, +y, +x in world space = columns 2,1,0 of B. */
        for(int k=0;k<3;++k){basis[k]=(int16_t)lrint(16384*B[k][2]);basis[3+k]=(int16_t)lrint(16384*B[k][1]);basis[6+k]=(int16_t)lrint(16384*B[k][0]);}
        for(int i=0;i<3;++i)for(int k=0;k<3;++k)cam.r[i][k]=(int16_t)lrint(16384*R[i][k]);
        for(int i=0;i<3;++i)cam.eye[i]=(rand()%2000000-1000000)*64;
        /* Object ahead of the camera: world d = R^T * view offset. */
        const double vz=200+rand()%3000,vx=(rand()%2001-1000)*vz/1600.0,vy=(rand()%2001-1000)*vz/1800.0;
        double dw[3];for(int k=0;k<3;++k)dw[k]=R[0][k]*vx+R[1][k]*vy+R[2][k]*vz;
        int32_t pos[3];for(int k=0;k<3;++k)pos[k]=cam.eye[k]+(int32_t)lrint(dw[k]*65536.0);
        sh2z_rspr_pick_t pick;sh2z_rspr_place_t place;int32_t depth;
        if(!sh2z_rspr_pose(&pick,&place,&depth,&cam,&s.header,pos,basis))continue;
        ++placed;
        const double tz=vz,fx=200.0,fy=180.0;
        const double dn=sqrt(dw[0]*dw[0]+dw[1]*dw[1]+dw[2]*dw[2]);
        CHECK(fabs(depth/16384.0-tz)<=3.0*dn/16384.0+0.01);
        CHECK(fabs(place.x+place.frac[0]/65536.0-(160+fx*vx/tz))<=0.5&&fabs(place.y+place.frac[1]/65536.0-(112-fy*vy/tz))<=0.5);
        const double want=fy*0.75/tz/1.5;
        CHECK(fabs(place.scale/65536.0-want)<=want*2e-3+3.0/65536.0);
        /* up = model +y in view, screen x/y. */
        double ux=0,uy=0;for(int k=0;k<3;++k){ux+=R[0][k]*B[k][1];uy+=R[1][k]*B[k][1];}
        const double l=sqrt(ux*ux+uy*uy);
        if(l>0.05){CHECK(fabs(place.up[0]/16384.0-ux/l)<2e-3&&fabs(place.up[1]/16384.0+uy/l)<2e-3);}
        /* Cell: same as sh2z_rspr_select on the exact model-frame direction (skip near-ties). */
        double e[3];for(int j=0;j<3;++j){e[j]=0;for(int k=0;k<3;++k)e[j]-=B[k][j]*dw[k];}
        sh2z_rspr_pick_t ref;
        if(sh2z_rspr_select(&ref,&s.header,(int32_t)lrint(e[0]*64),(int32_t)lrint(e[1]*64),(int32_t)lrint(e[2]*64))) {
            if(ref.cell==pick.cell&&ref.mirror==pick.mirror)++cells;else ++ties;
        }
    }
    CHECK(placed>15000);CHECK(ties*200<placed); /* bin-edge disagreements below 0.5% */
    printf("pose: %u placed, %u cells match, %u bin-edge differences\n",placed,cells,ties);
    int32_t pos[3]={cam.eye[0],cam.eye[1],cam.eye[2]};int16_t b[9]={0,0,16384,0,16384,0,16384,0,0};
    sh2z_rspr_pick_t pk;sh2z_rspr_place_t pl;int32_t dp;
    CHECK(!sh2z_rspr_pose(&pk,&pl,&dp,&cam,&s.header,pos,b)); /* at the eye: behind near */
    CHECK(!sh2z_rspr_pose(NULL,&pl,&dp,&cam,&s.header,pos,b));
}

int main(void) {
    test_valid();
    test_select(true);
    test_select(false);
    test_place_emit();
    test_still();
    test_pose();
    if(failures){fprintf(stderr,"rotation_sprite: %u failures\n",failures);return 1;}
    puts("rotation_sprite: all checks passed");
    return 0;
}
