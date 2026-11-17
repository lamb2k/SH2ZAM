/* Independent oracle, hand cases, relation to view-contract-1, caller-recipe
 * controls and negative controls for view-round-1 (final-coordinate
 * truncation). MIT; SH2ZAM Contributors. */
#include "sh2zam/view_experimental.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

__extension__ typedef __int128 wide_t;

static uint64_t checks;
static uint32_t state=UINT32_C(0x70c0ffee);
static void check(bool value,const char *what) {
    ++checks;
    if(!value) {
        fprintf(stderr,"round check %llu failed: %s\n",(unsigned long long)checks,what);
        exit(EXIT_FAILURE);
    }
}
static uint32_t random_word(void) {state^=state<<13;state^=state>>17;state^=state<<5;return state;}
static int32_t random_range(int32_t lo,int32_t hi) {
    return (int32_t)((int64_t)lo+(int64_t)(random_word()%(uint64_t)((int64_t)hi-lo+1)));
}
static int32_t random_coord(void) {
    switch(random_word()%4u) {
    case 0: return (int32_t)random_word();
    case 1: return random_range(-100000,100000);
    case 2: return (random_word()&1u)?INT32_MAX-random_range(0,3):INT32_MIN+random_range(0,3);
    default: return random_range(-(1<<24),1<<24);
    }
}
/* Division rounded toward zero, from magnitudes (independent of C's '/'). */
static wide_t toward_zero(wide_t n,wide_t d) {
    const wide_t a=n<0?-n:n,b=d<0?-d:d,q=a/b;
    return ((n<0)!=(d<0))?-q:q;
}
static wide_t floor_wide(wide_t n,wide_t d) {wide_t q=toward_zero(n,d);if(q*d!=n&&((n<0)!=(d<0)))--q;return q;}
static wide_t ceil_wide(wide_t n,wide_t d) {wide_t q=toward_zero(n,d);if(q*d!=n&&((n<0)==(d<0)))++q;return q;}
static wide_t clampw(wide_t a,wide_t lo,wide_t hi) {return a<lo?lo:a>hi?hi:a;}

/* ---- oracle ---------------------------------------------------------------- */
enum { MUT_NONE=0, MUT_FLOOR_X=1, MUT_CEIL_Y=2, MUT_ASYM_CLAMP=4, MUT_IGNORE_FRACTION=8, MUT_NO_GUARD_CLAMP=16 };
static unsigned mutation;
static uint8_t codes(const sh2z_view_t *v,int16_t sx,int16_t sy,wide_t z) {
    return (uint8_t)((z>v->far_z?SH2Z_OUT_FAR:0u)|(sx<v->clip[0]?SH2Z_OUT_LEFT:0u)|(sx>v->clip[2]?SH2Z_OUT_RIGHT:0u)|
        (sy<v->clip[1]?SH2Z_OUT_TOP:0u)|(sy>v->clip[3]?SH2Z_OUT_BOTTOM:0u));
}
/* The Q16 screen position w = 65536 * center +/- q, rounded to a pixel. */
static int16_t pixel(wide_t w,bool is_x) {
    wide_t s;
    if(is_x && (mutation&MUT_FLOOR_X))s=floor_wide(w,65536);
    else if(!is_x && (mutation&MUT_CEIL_Y))s=ceil_wide(w,65536);
    else if(mutation&MUT_IGNORE_FRACTION)s=w<0?floor_wide(w,65536)+1:w>0?ceil_wide(w,65536)-1:0;
    else s=toward_zero(w,65536);
    return (int16_t)clampw(s,INT16_MIN,INT16_MAX);
}
static wide_t quotient(int32_t focal,wide_t a,wide_t z) {
    const wide_t q=toward_zero((wide_t)focal*a,z);
    return clampw(q,(mutation&MUT_ASYM_CLAMP)?-(wide_t)INT32_MAX-1:-(wide_t)INT32_MAX,INT32_MAX);
}
static sh2z_view_vertex_t oracle_point(const sh2z_view_t *v,wide_t x,wide_t y,wide_t z) {
    sh2z_view_vertex_t o;
    memset(&o,0,sizeof(o));
    check(x>=INT32_MIN&&x<=INT32_MAX&&y>=INT32_MIN&&y<=INT32_MAX&&z>=INT32_MIN&&z<=INT32_MAX,"oracle input fits");
    o.x=(int32_t)x;o.y=(int32_t)y;o.z=(int32_t)z;
    if(z<v->near_z){o.outcode=SH2Z_OUT_NEAR;return o;}
    o.sx=pixel((wide_t)65536*v->center_x+quotient(v->focal_x,x,z),true);
    o.sy=pixel((wide_t)65536*v->center_y-quotient(v->focal_y,y,z),false);
    o.outcode=codes(v,o.sx,o.sy,z);
    return o;
}
static bool same_vertex(const sh2z_view_vertex_t *a,const sh2z_view_vertex_t *b) {
    return a->x==b->x&&a->y==b->y&&a->z==b->z&&a->sx==b->sx&&a->sy==b->sy&&a->outcode==b->outcode&&
        !a->reserved[0]&&!a->reserved[1]&&!a->reserved[2];
}

/* ---- cameras and points ------------------------------------------------------ */
static int16_t q14(double x) {const double s=floor(x*16384.0+0.5);return (int16_t)(s>32767?32767:s<-32768?-32768:s);}
static sh2z_view_t random_view(bool rotate) {
    sh2z_view_t v;
    memset(&v,0,sizeof(v));
    v.r[0][0]=v.r[1][1]=v.r[2][2]=16384;
    if(rotate) {
        const double a=(random_word()%65536)*6.283185307179586/65536.0,b=((double)(random_word()%65536)/65536.0-0.5)*1.5;
        const double ca=cos(a),sa=sin(a),cb=cos(b),sb=sin(b);
        const double m[3][3]={{ca,0,-sa},{sa*sb,cb,ca*sb},{sa*cb,-sb,ca*cb}};
        for(unsigned i=0;i<3;++i)for(unsigned j=0;j<3;++j)v.r[i][j]=q14(m[i][j]);
    }
    /* Whole-pixel focal lengths put many points on exact pixel positions. */
    switch(random_word()%3u) {
    case 0: v.focal_x=v.focal_y=160*65536;break;
    case 1: v.focal_x=65536*random_range(16,1024);v.focal_y=65536*random_range(16,1024);break;
    default: v.focal_x=random_range(65536,1024*65536);v.focal_y=random_range(65536,1024*65536);
    }
    switch(random_word()%4u) {
    case 0: v.center_x=160;v.center_y=112;break;
    case 1: v.center_x=(int16_t)random_range(-32768,32767);v.center_y=(int16_t)random_range(-32768,32767);break;
    default: v.center_x=(int16_t)random_range(0,352);v.center_y=(int16_t)random_range(0,256);
    }
    v.near_z=random_range(1,16*16384);v.far_z=v.near_z+random_range(1,INT32_MAX-v.near_z);
    v.clip[0]=(int16_t)random_range(-64,160);v.clip[2]=(int16_t)random_range(v.clip[0],400);
    v.clip[1]=(int16_t)random_range(-64,120);v.clip[3]=(int16_t)random_range(v.clip[1],300);
    check(sh2z_view_init(&v,&v),"random view valid");
    return v;
}
/* A view-space point aimed at a pixel (exactly on it when the focal length
 * divides evenly), then sometimes moved by a unit or by up to a pixel. */
static void aimed_point(const sh2z_view_t *v,int32_t p[3]) {
    int32_t z=random_range(v->near_z,v->near_z+(1<<22)>v->far_z?v->far_z:v->near_z+(1<<22));
    if(random_word()%4u==0)z=random_range(v->near_z,v->far_z);
    if(random_word()%8u==0)z=random_range(1,v->near_z);
    const int32_t span=random_word()%4u==0?20000:1200;
    const int32_t k[2]={random_range(-span,span)-v->center_x,v->center_y-random_range(-span,span)};
    for(unsigned a=0;a<2;++a) {
        const int32_t f=a?v->focal_y:v->focal_x;
        wide_t c=toward_zero((wide_t)k[a]*65536*z,f);
        switch(random_word()%4u) {
        case 0: break;
        case 1: c+=random_range(-2,2);break;
        default: c+=random_range(-(z<1000000?z:1000000),z<1000000?z:1000000);
        }
        p[a]=(int32_t)clampw(c,INT32_MIN,INT32_MAX);
    }
    p[2]=z;
}

/* ---- hand cases ---------------------------------------------------------------- */
static sh2z_view_vertex_t one(const sh2z_view_t *v,int32_t x,int32_t y,int32_t z,bool divu) {
    sh2z_view_vertex_t r={x,y,z,77,77,0xEE,{1,2,3}};
    check(divu?sh2z_view_project_trunc_divu(&r,v,&r,1):sh2z_view_project_trunc(&r,v,&r,1),"one-point projection");
    return r;
}
static void hand_cases(void) {
    sh2z_view_t v;
    memset(&v,0,sizeof(v));
    v.r[0][0]=v.r[1][1]=v.r[2][2]=16384;
    v.focal_x=v.focal_y=160*65536;v.center_x=160;v.center_y=112;
    v.near_z=16*16384;v.far_z=10000*16384;
    v.clip[0]=12;v.clip[1]=16;v.clip[2]=308;v.clip[3]=224;
    check(sh2z_view_init(&v,&v),"port view valid");
    /* At z = 160 * 16384 the offset from the centre is exactly x / 16384 px
     * (and the quotient 4x), so positions are easy to place by hand. */
    const int32_t z=160*16384,u=16384;
    for(int divu=0;divu<2;++divu) {
        /* The requester's worked examples: view y = -600, z = 1000 and 1200. */
        check(one(&v,0,-600*16384,1000*16384,divu).sy==208,"y=-600 z=1000 gives 208");
        check(one(&v,0,-600*16384,1200*16384,divu).sy==192,"y=-600 z=1200 gives 192");
        /* Whole pixels on both sides of zero keep their value. */
        check(one(&v,0,-96*u,z,divu).sy==208&&one(&v,0,112*u,z,divu).sy==0&&one(&v,0,113*u,z,divu).sy==-1,"whole-pixel y");
        check(one(&v,-160*u,0,z,divu).sx==0&&one(&v,-170*u,0,z,divu).sx==-10&&one(&v,40*u,0,z,divu).sx==200,"whole-pixel x");
        /* Positive fractions round down, negative ones up (toward zero). */
        check(one(&v,16000,0,z,divu).sx==160&&one(&v,0,-16000,z,divu).sy==112,"160.98 and 112.98 truncate down");
        check(one(&v,0,-96*u-16000,z,divu).sy==208&&one(&v,0,-96*u+16000,z,divu).sy==207,"208.98 is 208, 207.02 is 207");
        check(one(&v,-160*u-16000,0,z,divu).sx==0&&one(&v,-161*u-16000,0,z,divu).sx==-1,"-0.98 is 0, -1.98 is -1");
        check(one(&v,0,112*u+16000,z,divu).sy==0&&one(&v,0,114*u+16000,z,divu).sy==-2,"y -0.98 is 0, -2.98 is -2");
        check(one(&v,-160*u-1,0,z,divu).sx==0&&one(&v,-160*u+1,0,z,divu).sx==0,"one unit either side of zero");
        /* Quotients clamp to +/-(2^31 - 1): 160 +/- 32767.99998 and 112 -/+ it. */
        const sh2z_view_vertex_t a=one(&v,INT32_MAX,INT32_MAX,v.near_z,divu),b=one(&v,-INT32_MAX,-INT32_MAX,v.near_z,divu);
        check(a.sx==32767&&a.sy==-32655&&b.sx==-32607&&b.sy==32767,"clamped quotients");
    }
    /* Symmetric clamp: trunc(65536 * -2^31 / 1) becomes -(2^31 - 1), which has
     * a fraction: 160 - 32767.99998 truncates to -32607 (not -32608). */
    v.focal_x=65536;v.focal_y=65536;v.near_z=1;
    check(sh2z_view_init(&v,&v),"unit view valid");
    for(int divu=0;divu<2;++divu) {
        check(one(&v,INT32_MIN,0,1,divu).sx==-32607,"x quotient -2^31 clamps to -(2^31 - 1)");
        check(one(&v,0,INT32_MIN,1,divu).sy==32767,"y saturates");
    }
    /* Odd symmetry: negating x and center_x negates sx (no saturation). */
    for(unsigned t=0;t<20000;++t) {
        sh2z_view_t w=random_view(false),n=w;
        n.center_x=(int16_t)-w.center_x;n.center_y=(int16_t)-w.center_y;
        if(w.center_x==INT16_MIN||w.center_y==INT16_MIN)continue;
        int32_t p[3];aimed_point(&w,p);
        if(p[0]==INT32_MIN||p[1]==INT32_MIN||p[2]<w.near_z)continue;
        const sh2z_view_vertex_t a=one(&w,p[0],p[1],p[2],false),b=one(&n,-p[0],-p[1],p[2],false);
        if(a.sx>INT16_MIN&&a.sx<INT16_MAX)check(b.sx==-a.sx,"odd in x");
        if(a.sy>INT16_MIN&&a.sy<INT16_MAX)check(b.sy==-a.sy,"odd in y");
    }
}

/* ---- relation to view-contract-1 and the caller recipes --------------------------- */
typedef struct { uint64_t points,x_up,y_down,adapter_bad,mirror_bad,mirror_bad_nonneg,mirror_fixed_bad,reprojected; } relation_t;
static void relation(relation_t *r) {
    for(unsigned t=0;t<2000;++t) {
        const sh2z_view_t v=random_view(false);
        if(v.center_y==INT16_MIN||v.center_x==INT16_MIN)continue;
        sh2z_view_vertex_t in[256],tr[256],old[256],ad[256],mi[256];
        for(unsigned k=0;k<256;++k){int32_t p[3];aimed_point(&v,p);if(p[0]==INT32_MIN)p[0]++;if(p[1]==INT32_MIN)p[1]++;
            in[k]=(sh2z_view_vertex_t){p[0],p[1],p[2],0,0,0,{0,0,0}};}
        check(sh2z_view_project_trunc(tr,&v,in,256)&&sh2z_view_project(old,&v,in,256),"relation projections");
        /* The centre-minus-one adapter on the old projection. */
        sh2z_view_t a=v;a.center_y=(int16_t)(v.center_y-1);
        /* The mirrored-y caller recipe: y negated, centre_y negated, sy = -sy'. */
        sh2z_view_t m=v;m.center_y=(int16_t)-v.center_y;m.clip[1]=(int16_t)-v.clip[3];m.clip[3]=(int16_t)-v.clip[1];
        for(unsigned k=0;k<256;++k){mi[k]=in[k];mi[k].y=-in[k].y;}
        check(sh2z_view_project(ad,&a,in,256)&&sh2z_view_project(mi,&m,mi,256),"recipe projections");
        for(unsigned k=0;k<256;++k) {
            if(in[k].z<v.near_z)continue;
            ++r->points;
            const wide_t qx=quotient(v.focal_x,in[k].x,in[k].z),qy=quotient(v.focal_y,in[k].y,in[k].z);
            const wide_t wx=(wide_t)65536*v.center_x+qx,wy=(wide_t)65536*v.center_y-qy;
            const bool fx=(qx%65536)!=0,fy=(qy%65536)!=0;
            const bool sat=tr[k].sx==INT16_MIN||tr[k].sx==INT16_MAX||tr[k].sy==INT16_MIN||tr[k].sy==INT16_MAX||
                old[k].sx==INT16_MIN||old[k].sx==INT16_MAX||old[k].sy==INT16_MIN||old[k].sy==INT16_MAX;
            if(!sat) {
                /* sx is the old value plus one exactly when negative and fractional;
                 * sy the old value less one exactly when positive and fractional. */
                check(tr[k].sx-old[k].sx==((wx<0&&fx)?1:0),"sx relation to view-contract-1");
                check(old[k].sy-tr[k].sy==((wy>0&&fy)?1:0),"sy relation to view-contract-1");
                r->x_up+=tr[k].sx!=old[k].sx;r->y_down+=tr[k].sy!=old[k].sy;
                if(ad[k].sy!=tr[k].sy)++r->adapter_bad;
                const int32_t sy=-(int32_t)mi[k].sy;
                if(mi[k].sx!=tr[k].sx||sy!=tr[k].sy) {
                    ++r->mirror_bad;
                    /* Every miss is one pixel on an axis whose position is negative. */
                    check((mi[k].sx==tr[k].sx||(wx<0&&mi[k].sx==tr[k].sx-1))&&(sy==tr[k].sy||(wy<0&&sy==tr[k].sy-1)),
                        "mirrored recipe misses only negative positions, by one");
                    if(wx>=0&&wy>=0)++r->mirror_bad_nonneg;
                }
                /* The exact but costly fix: reproject negative results with x mirrored. */
                int32_t fx2=mi[k].sx,fy2=sy;
                if(fx2<0||fy2<0) {
                    sh2z_view_t n=v;n.center_x=(int16_t)-v.center_x;n.clip[0]=(int16_t)-v.clip[2];n.clip[2]=(int16_t)-v.clip[0];
                    sh2z_view_vertex_t q={-in[k].x,in[k].y,in[k].z,0,0,0,{0,0,0}};
                    check(in[k].x!=INT32_MIN&&sh2z_view_project(&q,&n,&q,1),"recipe reprojection");
                    if(fx2<0)fx2=-q.sx;
                    if(fy2<0)fy2=q.sy;
                    ++r->reprojected;
                }
                if(fx2!=tr[k].sx||fy2!=tr[k].sy)++r->mirror_fixed_bad;
            }
        }
    }
}

/* ---- path sweeps against the oracle -------------------------------------------------- */
typedef struct { uint64_t batch,project,steps,clipped,guard,inside_checked,mismatches; } sweep_t;
static uint64_t compare_points(const sh2z_view_vertex_t *got,const sh2z_view_t *v,size_t n) {
    uint64_t bad=0;
    for(size_t k=0;k<n;++k){const sh2z_view_vertex_t w=oracle_point(v,got[k].x,got[k].y,got[k].z);bad+=!same_vertex(&got[k],&w);}
    return bad;
}
static void sweep(sweep_t *s) {
    for(unsigned t=0;t<600;++t) {
        sh2z_view_t v=random_view(true);
        const sh2z_vec3_t eye={random_range(-20000,20000)*65536+random_range(0,65535),random_range(-20000,20000)*65536,
            random_range(-20000,20000)*65536+random_range(0,65535)};
        check(sh2z_view_place(&v,eye),"place");
        /* batch, both forms, stride 6 and 10 */
        static sh2z_pos16_t pos[1024];static uint16_t wide_pos[1024][5];static sh2z_view_vertex_t a[1024],b[1024],o[1024];
        for(unsigned k=0;k<1024;++k) {
            const int32_t s16=random_word()%2u?64:4096;
            pos[k].x=(int16_t)clampw(eye.x/65536+random_range(-s16,s16),INT16_MIN,INT16_MAX);
            pos[k].y=(int16_t)clampw(eye.y/65536+random_range(-s16,s16),INT16_MIN,INT16_MAX);
            pos[k].z=(int16_t)clampw(eye.z/65536+random_range(-s16,s16),INT16_MIN,INT16_MAX);
            if(random_word()%8u==0){pos[k].x=(int16_t)random_word();pos[k].y=(int16_t)random_word();pos[k].z=(int16_t)random_word();}
            wide_pos[k][0]=(uint16_t)pos[k].x;wide_pos[k][1]=(uint16_t)pos[k].y;wide_pos[k][2]=(uint16_t)pos[k].z;
            wide_pos[k][3]=wide_pos[k][4]=0xBEEF;
        }
        check(sh2z_view_batch_trunc(a,&v,pos,6,1024)&&sh2z_view_batch_trunc_divu(b,&v,wide_pos,10,1024)&&
            sh2z_view_batch(o,&v,pos,6,1024),"batches");
        for(unsigned k=0;k<1024;++k) {
            check(a[k].x==o[k].x&&a[k].y==o[k].y&&a[k].z==o[k].z,"batch view space unchanged");
            check(!memcmp(&a[k],&b[k],sizeof(a[k])),"batch divu equals portable");
        }
        s->mismatches+=compare_points(a,&v,1024);s->batch+=1024;
        /* project, both forms, in place */
        for(unsigned k=0;k<512;++k){int32_t p[3];aimed_point(&v,p);
            if(random_word()%8u==0){p[0]=random_coord();p[1]=random_coord();p[2]=random_coord();}
            a[k]=(sh2z_view_vertex_t){p[0],p[1],p[2],(int16_t)random_word(),(int16_t)random_word(),(uint8_t)random_word(),{9,9,9}};b[k]=a[k];}
        check(sh2z_view_project_trunc(a,&v,a,512)&&sh2z_view_project_trunc_divu(o,&v,b,512),"projects");
        check(!memcmp(a,o,512*sizeof(a[0])),"project divu equals portable");
        s->mismatches+=compare_points(a,&v,512);s->project+=512;
        /* edge steps: same points as sh2z_view_edge_steps, projected by the oracle */
        for(unsigned e=0;e<8;++e) {
            int32_t p[3],q[3];aimed_point(&v,p);aimed_point(&v,q);
            if(e%4u==3){for(unsigned i=0;i<3;++i){p[i]=random_coord();q[i]=random_coord();}}
            const sh2z_view_vertex_t f={p[0],p[1],p[2],0,0,0,{0,0,0}},g={q[0],q[1],q[2],0,0,0,{0,0,0}};
            const uint16_t den=(uint16_t)random_range(1,65535);
            const size_t count=(size_t)random_range(1,64);
            const uint16_t first=(uint16_t)random_range(0,den/4);
            const uint16_t step=count==1?(uint16_t)random_range(0,65535):(uint16_t)random_range(0,(int32_t)((den-first)/(count-1)));
            sh2z_view_vertex_t x[64],y[64],z[64];
            check(sh2z_view_edge_steps_trunc(x,&v,&f,&g,first,step,den,count)&&sh2z_view_edge_steps_trunc_divu(y,&v,&f,&g,first,step,den,count)&&
                sh2z_view_edge_steps(z,&v,&f,&g,first,step,den,count),"edge steps");
            for(size_t k=0;k<count;++k)check(x[k].x==z[k].x&&x[k].y==z[k].y&&x[k].z==z[k].z,"stepped points unchanged");
            check(!memcmp(x,y,count*sizeof(x[0])),"edge divu equals portable");
            s->mismatches+=compare_points(x,&v,count);s->steps+=count;
        }
        /* near clip: corners from the _trunc projection, crossings by the oracle */
        for(unsigned e=0;e<16;++e) {
            sh2z_view_vertex_t in[4],x[6],y[6],z[6];
            for(unsigned i=0;i<4;++i){int32_t p[3];aimed_point(&v,p);if((e>>(i&3u))&1u)p[2]=random_range(v.near_z>INT32_MIN/2?-v.near_z*2:INT32_MIN,v.near_z-1);
                in[i]=(sh2z_view_vertex_t){p[0],p[1],p[2],0,0,0,{0,0,0}};}
            check(sh2z_view_project_trunc(in,&v,in,4),"clip corners");
            const unsigned n=sh2z_view_clip_near_trunc(x,&v,in),m=sh2z_view_clip_near_trunc_divu(y,&v,in),k=sh2z_view_clip_near(z,&v,in);
            check(n==m&&n==k,"near clip counts");
            for(unsigned i=0;i<n;++i)check(x[i].x==z[i].x&&x[i].y==z[i].y&&x[i].z==z[i].z,"near clip points unchanged");
            check(!memcmp(x,y,n*sizeof(x[0])),"near clip divu equals portable");
            s->mismatches+=compare_points(x,&v,n);s->clipped+=n;
            sh2z_view_vertex_t io[6];memcpy(io,in,sizeof(in));
            check(sh2z_view_clip_near_trunc(io,&v,io)==n&&!memcmp(io,x,n*sizeof(x[0])),"near clip in place");
        }
        /* guard clip: geometry of sh2z_view_clip_guard, oracle projection, clamp */
        for(unsigned e=0;e<16;++e) {
            int16_t g[4];
            switch(random_word()%3u) {
            case 0: g[0]=-900;g[1]=-900;g[2]=900;g[3]=900;break;
            case 1: g[0]=v.clip[0];g[1]=v.clip[1];g[2]=v.clip[2];g[3]=v.clip[3];break;
            default: g[0]=(int16_t)random_range(v.center_x-2000,v.center_x+200);g[2]=(int16_t)random_range(g[0],v.center_x+2000);
                g[1]=(int16_t)random_range(v.center_y-2000,v.center_y+200);g[3]=(int16_t)random_range(g[1],v.center_y+2000);
            }
            if(g[0]<v.center_x-16384||g[2]>v.center_x+16384||g[1]<v.center_y-16384||g[3]>v.center_y+16384)continue;
            sh2z_view_guard_vertex_t in[8],x[16],y[16],z[16];
            const unsigned n=3u+random_word()%6u,attrs=random_word()%5u;
            memset(in,0,sizeof(in));
            for(unsigned i=0;i<n;++i){int32_t p[3];aimed_point(&v,p);if(random_word()%6u==0)p[2]=random_range(-v.near_z,v.near_z);
                in[i].v.x=p[0];in[i].v.y=p[1];in[i].v.z=p[2];for(unsigned j=0;j<4;++j)in[i].attr[j]=(int32_t)random_word();}
            const int rx=sh2z_view_clip_guard_trunc(x,&v,g,in,n,attrs),ry=sh2z_view_clip_guard_trunc_divu(y,&v,g,in,n,attrs);
            const int rz=sh2z_view_clip_guard(z,&v,g,in,n,attrs);
            check(rx==rz&&ry==rz,"guard counts");
            if(rx<=0)continue;
            check(!memcmp(x,y,(size_t)rx*sizeof(x[0])),"guard divu equals portable");
            for(int i=0;i<rx;++i) {
                check(x[i].v.x==z[i].v.x&&x[i].v.y==z[i].v.y&&x[i].v.z==z[i].v.z&&!memcmp(x[i].attr,z[i].attr,sizeof(x[i].attr)),"guard geometry unchanged");
                sh2z_view_vertex_t w=oracle_point(&v,x[i].v.x,x[i].v.y,x[i].v.z);
                const int16_t sx=w.sx,sy=w.sy;
                if(!(mutation&MUT_NO_GUARD_CLAMP)) {
                    w.sx=(int16_t)clampw(w.sx,g[0],g[2]);w.sy=(int16_t)clampw(w.sy,g[1],g[3]);
                }
                w.outcode=(uint8_t)(codes(&v,w.sx,w.sy,0)&~SH2Z_OUT_FAR);
                s->mismatches+=!same_vertex(&x[i].v,&w);
                /* A point inside all six planes lands inside the rectangle unclamped. */
                const wide_t X=x[i].v.x,Y=x[i].v.y,Z=x[i].v.z;
                const bool inside=Z>=v.near_z&&Z<=v.far_z&&(wide_t)v.focal_x*X-(wide_t)65536*(g[0]-v.center_x)*Z>=0&&
                    (wide_t)65536*(g[2]-v.center_x)*Z-(wide_t)v.focal_x*X>=0&&(wide_t)65536*(v.center_y-g[1])*Z-(wide_t)v.focal_y*Y>=0&&
                    (wide_t)v.focal_y*Y-(wide_t)65536*(v.center_y-g[3])*Z>=0;
                if(inside&&!mutation){check(sx>=g[0]&&sx<=g[2]&&sy>=g[1]&&sy<=g[3],"inside all planes projects inside the rectangle");++s->inside_checked;}
                s->guard++;
            }
            sh2z_view_guard_vertex_t io[16];memcpy(io,in,sizeof(in));
            check(sh2z_view_clip_guard_trunc(io,&v,g,io,n,attrs)==rx&&!memcmp(io,x,(size_t)rx*sizeof(x[0])),"guard in place");
        }
    }
}

/* ---- argument rules: the same failures as view-contract-1, nothing written ---------- */
static void argument_rules(void) {
    sh2z_view_t v=random_view(false),bad=v;
    bad.focal_x=0;
    sh2z_pos16_t pos[4]={{1,2,3},{4,5,6},{7,8,9},{1,1,1}};
    sh2z_view_vertex_t out[8],pattern[8];
    memset(pattern,0x5A,sizeof(pattern));
    const sh2z_view_t *views[2]={&v,&bad};
    for(unsigned vi=0;vi<2;++vi)for(unsigned stride=4;stride<=8;++stride)for(unsigned misalign=0;misalign<2;++misalign) {
        const void *p=(const char *)pos+misalign;
        sh2z_view_vertex_t ref[8];
        const bool want=sh2z_view_batch(ref,views[vi],p,stride,3);
        memcpy(out,pattern,sizeof(out));
        const bool a=sh2z_view_batch_trunc(out,views[vi],p,stride,3);
        check(a==want,"batch rules match view-contract-1");
        if(!a)check(!memcmp(out,pattern,sizeof(out)),"failed batch writes nothing");
        memcpy(out,pattern,sizeof(out));
        check(sh2z_view_batch_trunc_divu(out,views[vi],p,stride,3)==want,"batch divu rules match");
        if(!want)check(!memcmp(out,pattern,sizeof(out)),"failed batch divu writes nothing");
    }
    check(sh2z_view_batch_trunc(NULL,&v,pos,6,0)&&!sh2z_view_batch_trunc(NULL,&v,pos,6,1),"null with count");
    check(!sh2z_view_batch_trunc((sh2z_view_vertex_t *)(void *)pos,&v,pos,6,1),"overlap rejected");
    memcpy(out,pattern,sizeof(out));
    check(!sh2z_view_project_trunc(out+1,&v,out,4)&&!sh2z_view_project_trunc_divu(out+1,&v,out,4)&&!memcmp(out,pattern,sizeof(out)),"partial overlap rejected");
    check(!sh2z_view_project_trunc(out,&bad,out,1)&&sh2z_view_project_trunc(NULL,&v,NULL,0),"project rules");
    const sh2z_view_vertex_t f={0,0,100000,0,0,0,{0,0,0}},g={1000,1000,200000,0,0,0,{0,0,0}};
    memcpy(out,pattern,sizeof(out));
    check(!sh2z_view_edge_steps_trunc(out,&v,&f,&g,10,0,0,1)&&!sh2z_view_edge_steps_trunc(out,&v,&f,&g,5,1,4,1)&&
        !sh2z_view_edge_steps_trunc(out,&v,&f,&g,0,2,4,4)&&!sh2z_view_edge_steps_trunc_divu(out,&bad,&f,&g,0,1,4,2)&&
        !memcmp(out,pattern,sizeof(out)),"edge rules write nothing");
    check(sh2z_view_edge_steps_trunc(out,&v,&f,&g,0,2,4,3),"edge bound met exactly");
    check(sh2z_view_clip_near_trunc(out,&bad,out)==0&&sh2z_view_clip_near_trunc_divu(NULL,&v,out)==0,"near clip rules");
    sh2z_view_guard_vertex_t gin[9],gout[16];memset(gin,0,sizeof(gin));
    const int16_t rect[4]={-900,-900,900,900},flipped[4]={900,-900,-900,900},far[4]={-20000,-900,900,900};
    check(sh2z_view_clip_guard_trunc(gout,&v,rect,gin,2,0)==SH2Z_VIEW_GUARD_INVALID&&
        sh2z_view_clip_guard_trunc(gout,&v,rect,gin,9,0)==SH2Z_VIEW_GUARD_INVALID&&
        sh2z_view_clip_guard_trunc(gout,&v,rect,gin,4,5)==SH2Z_VIEW_GUARD_INVALID&&
        sh2z_view_clip_guard_trunc_divu(gout,&v,flipped,gin,4,0)==SH2Z_VIEW_GUARD_INVALID&&
        sh2z_view_clip_guard_trunc_divu(gout,&bad,rect,gin,4,0)==SH2Z_VIEW_GUARD_INVALID&&
        sh2z_view_clip_guard_trunc(gout,&v,far,gin,4,0)==sh2z_view_clip_guard(gout,&v,far,gin,4,0),"guard rules");
}

int main(void) {
    hand_cases();
    relation_t r;memset(&r,0,sizeof(r));
    relation(&r);
    check(r.x_up>0&&r.y_down>0,"both differences from view-contract-1 occur");
    check(r.adapter_bad>0,"the centre-minus-one adapter is not exact (control)");
    check(r.mirror_bad>0&&r.mirror_bad_nonneg==0,"mirrored recipe: misses exist, none at non-negative positions");
    check(r.mirror_fixed_bad==0,"mirrored recipe plus reprojection is exact");
    sweep_t s;memset(&s,0,sizeof(s));
    sweep(&s);
    check(s.mismatches==0,"every path equals the oracle");
    check(s.inside_checked>0,"guard inside-planes property exercised");
    argument_rules();
    printf("view-round-1: %llu batch, %llu projected, %llu stepped, %llu near-clip, %llu guard vertices equal the oracle on both entry points\n",
        (unsigned long long)s.batch,(unsigned long long)s.project,(unsigned long long)s.steps,(unsigned long long)s.clipped,(unsigned long long)s.guard);
    printf("  relation over %llu points: sx +1 on %llu, sy -1 on %llu; inside-plane guard vertices checked unclamped: %llu\n",
        (unsigned long long)r.points,(unsigned long long)r.x_up,(unsigned long long)r.y_down,(unsigned long long)s.inside_checked);
    printf("  caller recipes on view-contract-1: centre-1 adapter %llu misses; mirrored y %llu misses (0 at non-negative positions); "
        "mirrored y + reprojection 0 misses with %llu extra projections\n",
        (unsigned long long)r.adapter_bad,(unsigned long long)r.mirror_bad,(unsigned long long)r.reprojected);
    /* Negative controls: each oracle mutation must be detected. */
    static const struct { unsigned m; const char *name; } controls[]={{MUT_FLOOR_X,"x floored"},{MUT_CEIL_Y,"y ceiled"},
        {MUT_ASYM_CLAMP,"quotient clamp asymmetric"},{MUT_IGNORE_FRACTION,"fraction test dropped"},{MUT_NO_GUARD_CLAMP,"guard clamp dropped"}};
    for(unsigned c=0;c<sizeof(controls)/sizeof(controls[0]);++c) {
        mutation=controls[c].m;state=UINT32_C(0x70c0ffee);
        sweep_t m;memset(&m,0,sizeof(m));
        sweep(&m);
        uint64_t extra=0;
        if(controls[c].m==MUT_ASYM_CLAMP) {
            /* The clamp only matters for quotients that saturate negative. */
            sh2z_view_t v;memset(&v,0,sizeof(v));v.r[0][0]=v.r[1][1]=v.r[2][2]=16384;v.focal_x=v.focal_y=65536;
            v.center_x=160;v.center_y=112;v.near_z=1;v.far_z=2;v.clip[2]=320;v.clip[3]=224;
            sh2z_view_vertex_t p={INT32_MIN,INT32_MIN,1,0,0,0,{0,0,0}};
            check(sh2z_view_project_trunc(&p,&v,&p,1),"clamp control projection");
            const sh2z_view_vertex_t w=oracle_point(&v,INT32_MIN,INT32_MIN,1);
            extra=!same_vertex(&p,&w);
        }
        printf("  control (%s): %llu mismatches\n",controls[c].name,(unsigned long long)(m.mismatches+extra));
        check(m.mismatches+extra>0,"negative control detected");
    }
    printf("view-round-1 PASS (%llu checks)\n",(unsigned long long)checks);
    return 0;
}
