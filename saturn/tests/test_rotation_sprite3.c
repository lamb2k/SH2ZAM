/* Host tests for contract tag rotation-sprites-3: each new form against the
 * rotation-sprites-2 function it must equal, bit for bit.
 *  - sh2z_rspr_yaw_convex on convex, mirrored, reordered and degenerate tables;
 *  - sh2z_rspr_yaw_climb against the scan of sh2z_rspr_select over dense,
 *    grid, random and tie (edge-perpendicular) directions from several starts,
 *    with two controls (no tie rule; climbing one way) that must differ;
 *  - sh2z_rspr_pose_terms against sh2z_rspr_pose over random poses, with and
 *    without the climb, rejections included;
 *  - sh2z_rspr_emit_cell against sh2z_rspr_emit_valid plus the caller's
 *    COLR/SRCA over random cells and places, rejections included.
 *
 * Copyright (c) 2026 SH2ZAM Contributors. MIT License.
 * Inspired by SH4ZAM by Falco Girgis and SH4ZAM Contributors.
 */
#include "sh2zam/rotation_sprite_experimental.h"

#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static unsigned failures;
#define CHECK(cond) do { if(!(cond)) { ++failures; fprintf(stderr,"%s:%d: %s\n",__FILE__,__LINE__,#cond); } } while(0)

#define PI 3.14159265358979323846
enum {MAXY=32,PITCH=3,CELL_W=24,CELL_H=16};

typedef struct {
    sh2z_rspr_set_t header;
    sh2z_rspr_cell_t cells[MAXY*PITCH];
    uint16_t cluts[2][16];
    uint8_t pixels[CELL_W*CELL_H/2];
} test_set_t;

static uint64_t rng=0x9E3779B97F4A7C15ull;
static uint32_t next(void) { rng^=rng<<13;rng^=rng>>7;rng^=rng<<17;return (uint32_t)(rng>>16); }
static int32_t span(int32_t lo,int32_t hi) { return lo+(int32_t)(next()%(uint32_t)(hi-lo+1)); }

/* yaw bins at the given angles (degrees); pitch -20, 10, 40. */
static void build(test_set_t *s,const double *deg,unsigned n,bool mirror) {
    memset(s,0,sizeof(*s));
    s->header.magic=SH2Z_RSPR_MAGIC;s->header.bytes=(uint32_t)sizeof(*s);
    s->header.yaw_count=(uint16_t)n;s->header.pitch_count=PITCH;s->header.flags=mirror?SH2Z_RSPR_MIRROR:0;
    s->header.clut_count=2;s->header.clut_offset=(uint32_t)offsetof(test_set_t,cluts);
    s->header.pixels_per_unit=3*65536/2;
    for(unsigned k=0;k<n;++k) {
        s->header.yaw[k][0]=(int16_t)lrint(16384.0*sin(deg[k]*PI/180.0));
        s->header.yaw[k][1]=(int16_t)lrint(16384.0*cos(deg[k]*PI/180.0));
    }
    static const double pd[PITCH]={-20.0,10.0,40.0};
    for(unsigned k=0;k<PITCH;++k) {
        s->header.pitch[k][0]=(int16_t)lrint(16384.0*sin(pd[k]*PI/180.0));
        s->header.pitch[k][1]=(int16_t)lrint(16384.0*cos(pd[k]*PI/180.0));
    }
    for(unsigned i=0;i<n*PITCH;++i)
        s->cells[i]=(sh2z_rspr_cell_t){.width=CELL_W,.height=CELL_H,.origin_x=10,.origin_y=12,
            .pixel_offset=(uint32_t)offsetof(test_set_t,pixels),.clut=(uint16_t)(i&1u)};
}

/* The tables: uniform 16 (-180..180), a dense rear (20 bins, 4.5 degrees near
 * 180, 30 near 0), 32 uniform, and the smallest polygon (3 bins). */
static unsigned table(int which,double *deg) {
    switch(which) {
    case 0: for(unsigned k=0;k<16u;++k)deg[k]=-180.0+k*22.5;return 16u;
    case 1: {
        unsigned n=0;
        for(double a=-180.0;a<-150.0;a+=4.5)deg[n++]=a;
        for(double a=-150.0;a<150.0;a+=30.0)deg[n++]=a;
        for(double a=150.0;a<180.0;a+=4.5)deg[n++]=a;
        return n;
    }
    case 2: for(unsigned k=0;k<32u;++k)deg[k]=k*11.25;return 32u;
    default: deg[0]=0.0;deg[1]=120.0;deg[2]=240.0;return 3u;
    }
}

static unsigned scan(const sh2z_rspr_set_t *s,int32_t x,int32_t z,int32_t *best) {
    unsigned kb=0;
    for(unsigned k=0;k<s->yaw_count;++k) {
        const int32_t v=s->yaw[k][0]*x+s->yaw[k][1]*z;
        if(k==0||v>*best){*best=v;kb=k;}
    }
    return kb;
}

/* Controls: 1 no tie rule, 2 climbs up the index only. Each must differ from the scan somewhere. */
static unsigned mutant(const sh2z_rspr_set_t *s,int mode,unsigned start,int32_t x,int32_t z) {
    const unsigned n=s->yaw_count;unsigned k=start;
    int32_t fk=s->yaw[k][0]*x+s->yaw[k][1]*z;
    for(;;) {
        const unsigned kn=k+1u==n?0u:k+1u,kp=k==0u?n-1u:k-1u;
        const int32_t fn=s->yaw[kn][0]*x+s->yaw[kn][1]*z;
        if(fn>fk){k=kn;fk=fn;continue;}
        if(mode==2)break;
        const int32_t fp=s->yaw[kp][0]*x+s->yaw[kp][1]*z;
        if(fp>fk){k=kp;fk=fp;continue;}
        break;
    }
    return k;
}

static unsigned long climbs,ctl_tie,ctl_up;
static void climb_check(const sh2z_rspr_set_t *s,int32_t x,int32_t z) {
    if(x==0&&z==0)return;
    int32_t sb=0;const unsigned sk=scan(s,x,z,&sb),n=s->yaw_count;
    const unsigned starts[5]={(sk+n/2u)%n,next()%n,0u,(sk+1u)%n,next()};
    for(unsigned i=0;i<5u;++i) {
        int32_t cb=0;const unsigned ck=sh2z_rspr_yaw_climb(s,starts[i],x,z,&cb);
        CHECK(ck==sk&&cb==sb);++climbs;
        const unsigned st=starts[i]<n?starts[i]:0u;
        if(mutant(s,1,st,x,z)!=sk)++ctl_tie;
        if(mutant(s,2,st,x,z)!=sk)++ctl_up;
    }
}

static void test_convex(void) {
    static test_set_t s;double deg[MAXY];
    for(int w=0;w<4;++w) {
        const unsigned n=table(w,deg);build(&s,deg,n,false);
        CHECK(sh2z_rspr_valid(&s.header));CHECK(sh2z_rspr_yaw_convex(&s.header));
        /* Reversed order (clockwise) is convex too. */
        double r[MAXY];for(unsigned k=0;k<n;++k)r[k]=deg[n-1u-k];
        build(&s,r,n,false);CHECK(sh2z_rspr_yaw_convex(&s.header));
        if(n>=4u) {
            double t[MAXY];memcpy(t,deg,sizeof(t));const double a=t[1];t[1]=t[2];t[2]=a; /* two bins swapped */
            build(&s,t,n,false);CHECK(!sh2z_rspr_yaw_convex(&s.header));
        }
        build(&s,deg,n,true);CHECK(!sh2z_rspr_yaw_convex(&s.header));                  /* mirrored */
    }
    /* Wound twice (a star: every second bin of 10 around 720 degrees). */
    for(unsigned k=0;k<5u;++k)deg[k]=k*144.0;
    build(&s,deg,5u,false);CHECK(!sh2z_rspr_yaw_convex(&s.header));
    /* Three collinear corners (not strict): a bin at the midpoint of its neighbours. */
    table(0,deg);build(&s,deg,16u,false);
    s.header.yaw[1][0]=(int16_t)((s.header.yaw[0][0]+s.header.yaw[2][0])/2);
    s.header.yaw[1][1]=(int16_t)((s.header.yaw[0][1]+s.header.yaw[2][1])/2);
    CHECK(!sh2z_rspr_yaw_convex(&s.header));
    /* A dent: one bin pulled toward the centre. */
    table(0,deg);build(&s,deg,16u,false);s.header.yaw[5][0]=(int16_t)(s.header.yaw[5][0]/2);s.header.yaw[5][1]=(int16_t)(s.header.yaw[5][1]/2);
    CHECK(!sh2z_rspr_yaw_convex(&s.header));
    build(&s,deg,2u,false);CHECK(!sh2z_rspr_yaw_convex(&s.header));
    CHECK(!sh2z_rspr_yaw_convex(NULL));
    puts("convex: tables, reversed, swapped, mirrored, star, collinear, dent, 2 bins");
}

static void test_climb(void) {
    static test_set_t s;double deg[MAXY];
    for(int w=0;w<4;++w) {
        const unsigned n=table(w,deg);build(&s,deg,n,false);
        const sh2z_rspr_set_t *h=&s.header;
        for(int32_t x=-300;x<=300;++x)for(int32_t z=-300;z<=300;++z)climb_check(h,x,z);
        for(int32_t x=-32767;x<=32767;x+=257)for(int32_t z=-32767;z<=32767;z+=257)climb_check(h,x,z);
        for(unsigned i=0;i<400000u;++i)climb_check(h,span(-32767,32767),span(-32767,32767));
        for(unsigned k=0;k<n;++k) { /* ties: perpendicular to each edge, both ways */
            const unsigned k1=(k+1u)%n;
            const int32_t dx=h->yaw[k1][0]-h->yaw[k][0],dz=h->yaw[k1][1]-h->yaw[k][1];
            climb_check(h,dz,-dx);climb_check(h,-dz,dx);
        }
    }
    CHECK(ctl_tie>0);CHECK(ctl_up>0);
    printf("climb: %lu climbs equal the scan; controls differ: no tie rule %lu, one way %lu\n",climbs,ctl_tie,ctl_up);
}

/* Rotation from yaw (about y), pitch (about x) and roll (about z), row-major. */
static void rot(double m[3][3],double a,double b,double c) {
    const double ca=cos(a),sa=sin(a),cb=cos(b),sb=sin(b),cc=cos(c),sc=sin(c);
    const double y[3][3]={{ca,0,sa},{0,1,0},{-sa,0,ca}},x[3][3]={{1,0,0},{0,cb,-sb},{0,sb,cb}},z[3][3]={{cc,-sc,0},{sc,cc,0},{0,0,1}};
    double t[3][3];
    for(int i=0;i<3;++i)for(int j=0;j<3;++j){t[i][j]=0;for(int k=0;k<3;++k)t[i][j]+=x[i][k]*y[k][j];}
    for(int i=0;i<3;++i)for(int j=0;j<3;++j){m[i][j]=0;for(int k=0;k<3;++k)m[i][j]+=z[i][k]*t[k][j];}
}

static int32_t dot16(const int16_t row[3],const int32_t d[3]) {
    const int64_t sum=(int64_t)row[0]*d[0]+(int64_t)row[1]*d[1]+(int64_t)row[2]*d[2];
    return (int32_t)(sum>>16);
}

static sh2z_rspr_terms_t terms_of(const sh2z_rspr_camera_t *c,const int32_t pos[3]) {
    sh2z_rspr_terms_t t;
    for(int k=0;k<3;++k)t.d[k]=(int32_t)((uint32_t)pos[k]-(uint32_t)c->eye[k]);
    t.tz=dot16(c->r[2],t.d);
    t.nx=(int64_t)c->focal_x*dot16(c->r[0],t.d);t.ny=(int64_t)c->focal_y*dot16(c->r[1],t.d);
    return t;
}

static void test_pose_terms(void) {
    static test_set_t s;double deg[MAXY];
    unsigned same=0,rejected=0,climbed=0;
    for(int w=0;w<3;++w) for(int m=0;m<2;++m) {
        const unsigned n=table(w,deg);build(&s,deg,n,m==1);
        const bool convex=sh2z_rspr_yaw_convex(&s.header);
        sh2z_rspr_camera_t cam;memset(&cam,0,sizeof(cam));
        cam.focal_x=200*65536+(int32_t)(next()%65536);cam.focal_y=180*65536;cam.center_x=160;cam.center_y=112;cam.near_z=16384;
        CHECK(sh2z_rspr_camera_scale(&cam,&s.header,12288));
        uint8_t hint=0;
        for(unsigned i=0;i<40000u;++i) {
            double R[3][3],B[3][3];
            rot(R,(next()%6283)/1000.0,((int)(next()%1000)-500)/1000.0,((int)(next()%1000)-500)/2000.0);
            rot(B,(next()%6283)/1000.0,((int)(next()%600)-300)/1000.0,((int)(next()%1000)-500)/1000.0);
            int16_t basis[9];
            for(int k=0;k<3;++k){basis[k]=(int16_t)lrint(16384*B[k][2]);basis[3+k]=(int16_t)lrint(16384*B[k][1]);basis[6+k]=(int16_t)lrint(16384*B[k][0]);}
            for(int a=0;a<3;++a)for(int k=0;k<3;++k)cam.r[a][k]=(int16_t)lrint(16384*R[a][k]);
            for(int k=0;k<3;++k)cam.eye[k]=span(-1000000,1000000)*64;
            /* Mostly ahead, some behind, beside, far off screen or at the eye. */
            const double vz=(i%9u==0u)?-(double)(next()%500):(i%13u==0u)?0.5:20.0+next()%6000;
            const double vx=((int)(next()%4001)-2000)*fabs(vz)/1600.0,vy=((int)(next()%4001)-2000)*fabs(vz)/1800.0;
            double dw[3];for(int k=0;k<3;++k)dw[k]=R[0][k]*vx+R[1][k]*vy+R[2][k]*vz;
            int32_t pos[3];for(int k=0;k<3;++k)pos[k]=cam.eye[k]+(int32_t)lrint(dw[k]*65536.0);
            if(i%101u==0u){pos[0]=cam.eye[0];pos[1]=cam.eye[1];pos[2]=cam.eye[2];}
            const sh2z_rspr_terms_t t=terms_of(&cam,pos);
            sh2z_rspr_pick_t p1,p2;sh2z_rspr_place_t q1,q2;int32_t d1=7,d2=7;
            memset(&p1,0x33,sizeof(p1));memset(&p2,0x33,sizeof(p2));memset(&q1,0x44,sizeof(q1));memset(&q2,0x44,sizeof(q2));
            const bool a=sh2z_rspr_pose(&p1,&q1,&d1,&cam,&s.header,pos,basis);
            const bool b=sh2z_rspr_pose_terms(&p2,&q2,&d2,&cam,&s.header,&t,basis,NULL);
            CHECK(a==b);CHECK(memcmp(&p1,&p2,sizeof(p1))==0&&memcmp(&q1,&q2,sizeof(q1))==0&&d1==d2);
            if(convex) {
                uint8_t h2=(i%7u==0u)?(uint8_t)next():hint;
                sh2z_rspr_pick_t p3;sh2z_rspr_place_t q3;int32_t d3=7;
                memset(&p3,0x33,sizeof(p3));memset(&q3,0x44,sizeof(q3));
                const bool c=sh2z_rspr_pose_terms(&p3,&q3,&d3,&cam,&s.header,&t,basis,&h2);
                CHECK(a==c);CHECK(memcmp(&p1,&p3,sizeof(p1))==0&&memcmp(&q1,&q3,sizeof(q1))==0&&d1==d3);
                if(c)hint=h2;
                ++climbed;
            }
            if(a)++same;else ++rejected;
        }
    }
    sh2z_rspr_terms_t t;memset(&t,0,sizeof(t));t.tz=1<<20;
    sh2z_rspr_pick_t p;sh2z_rspr_place_t q;int32_t d;const int16_t b[9]={0,0,16384,0,16384,0,16384,0,0};
    sh2z_rspr_camera_t cam;memset(&cam,0,sizeof(cam));
    CHECK(!sh2z_rspr_pose_terms(NULL,&q,&d,&cam,&s.header,&t,b,NULL));
    CHECK(!sh2z_rspr_pose_terms(&p,&q,&d,&cam,&s.header,NULL,b,NULL));
    CHECK(same>100000u&&rejected>10000u);
    printf("pose_terms: %u placed and %u rejected as sh2z_rspr_pose; %u also with the climb\n",same,rejected,climbed);
}

static void test_emit_cell(void) {
    enum {SW=504,SH=255};
    typedef struct {
        sh2z_rspr_set_t header;
        sh2z_rspr_cell_t cells[2];
        uint16_t cluts[2][16];
        uint8_t pixels[SW*SH/2];
    } emit_set_t;
    static emit_set_t s;
    unsigned same=0,rejected=0;
    for(unsigned i=0;i<400000u;++i) {
        memset(&s,0,sizeof(s));
        s.header.magic=SH2Z_RSPR_MAGIC;s.header.bytes=(uint32_t)sizeof(s);s.header.yaw_count=2;s.header.pitch_count=1;
        s.header.clut_count=2;s.header.clut_offset=(uint32_t)offsetof(emit_set_t,cluts);s.header.pixels_per_unit=65536;
        const uint16_t w=(uint16_t)(8u*(1u+next()%63u)),h=(uint16_t)(1u+next()%255u);
        s.cells[1]=(sh2z_rspr_cell_t){.width=w,.height=h,.origin_x=(int16_t)span(-64,w+64),.origin_y=(int16_t)span(-64,h+64),
            .pixel_offset=(uint32_t)offsetof(emit_set_t,pixels),.clut=1};
        s.cells[0]=s.cells[1];
        sh2z_rspr_place_t pl;
        pl.x=(int16_t)span(-1100,1100);pl.y=(int16_t)span(-1100,1100);
        const unsigned sc=next()%8u;
        pl.scale=sc==0?span(-4,4):sc==1?(1<<24)+span(-2,2):sc<5?span(1,65536):span(1,1<<20);
        const double a=(next()%62832)/10000.0;
        pl.up[0]=(int16_t)lrint(16384*sin(a));pl.up[1]=(int16_t)lrint(-16384*cos(a));
        if(i%17u==0u){pl.up[0]=(int16_t)span(-32768,32767);pl.up[1]=(int16_t)span(-32768,32767);}
        pl.frac[0]=(uint16_t)next();pl.frac[1]=(uint16_t)next();
        const sh2z_rspr_pick_t pick={.cell=1,.mirror=(uint8_t)(next()%2u)};
        const uint32_t vram=8u*(next()%1024u);const uint16_t pmod=(uint16_t)next(),colr=(uint16_t)next(),srca=(uint16_t)next();
        uint16_t c1[16],c2[16];memset(c1,0x5a,sizeof(c1));memset(c2,0x5a,sizeof(c2));
        const bool a1=sh2z_rspr_emit_valid(c1,&s.header,&pick,&pl,vram,pmod);
        if(a1){c1[3]=colr;c1[4]=srca;}
        const bool a2=sh2z_rspr_emit_cell(c2,&s.cells[1],pick.mirror,&pl,pmod,colr,srca);
        CHECK(a1==a2);CHECK(memcmp(c1,c2,sizeof(c1))==0);
        if(a1)++same;else ++rejected;
    }
    uint16_t c[16];sh2z_rspr_place_t pl={.x=0,.y=0,.scale=65536,.up={0,-16384},.frac={0,0}};
    CHECK(!sh2z_rspr_emit_cell(c,&s.cells[1],2u,&pl,0,0,0));
    CHECK(!sh2z_rspr_emit_cell(NULL,&s.cells[1],0u,&pl,0,0,0));
    CHECK(same>100000u&&rejected>10000u);
    printf("emit_cell: %u commands and %u rejections as sh2z_rspr_emit_valid\n",same,rejected);
}

int main(void) {
    test_convex();
    test_climb();
    test_pose_terms();
    test_emit_cell();
    if(failures){fprintf(stderr,"rotation_sprite3: %u failures\n",failures);return 1;}
    puts("rotation_sprite3: all checks passed");
    return 0;
}
