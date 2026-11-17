/* view-guard-src-1: an independent oracle of a source renderer's guard clip
 * (written from that clipper, not from the library), how far the
 * view-round-1 guard clip is from it, exactness of the source-order clip,
 * edge cases and negative controls. MIT; SH2ZAM Contributors. */
#include "sh2zam/view_experimental.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

__extension__ typedef __int128 wide_t;

static uint64_t checks;
static uint32_t state=UINT32_C(0x5eedc11f);
static void check(bool value,const char *what) {
    ++checks;
    if(!value) {
        fprintf(stderr,"guard-src check %llu failed: %s\n",(unsigned long long)checks,what);
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
static wide_t toward_zero(wide_t n,wide_t d) {
    const wide_t a=n<0?-n:n,b=d<0?-d:d,q=a/b;
    return ((n<0)!=(d<0))?-q:q;
}
static wide_t floor_wide(wide_t n,wide_t d) {wide_t q=toward_zero(n,d);if(q*d!=n&&((n<0)!=(d<0)))--q;return q;}
static wide_t ceil_wide(wide_t n,wide_t d) {wide_t q=toward_zero(n,d);if(q*d!=n&&((n<0)==(d<0)))++q;return q;}
static wide_t clampw(wide_t a,wide_t lo,wide_t hi) {return a<lo?lo:a>hi?hi:a;}
static wide_t absw(wide_t a) {return a<0?-a:a;}

/* ---- the source clipper, as an oracle ----------------------------------------
 * Planes 0..5: near, far, left, right, top, bottom, each written as the source
 * writes it (centre Q16, clip edges in pixels). A pass walks previous ->
 * current starting from (last, first); a sign change emits the crossing from
 * previous towards current at the Q0.32 fraction |dp| / (|dp| + |dc|), which
 * is then moved inside the planes done; then the current corner if inside. */
enum {
    M_LIB_ORDER=1,      /* emit current, then the crossing towards next */
    M_START=2,          /* walk from (first, second) instead of (last, first) */
    M_PLANE_ORDER=4,    /* left, top, right, bottom */
    M_LIB_CROSS=8,      /* view-guard-1 crossing: from the inside end, Q2.30 sides */
    M_NO_SNAP=16,       /* crossings not moved inside */
    M_NO_ZSNAP=32,      /* no near/far clamp in the snap */
    M_ROUND_T=64,       /* fraction rounded to nearest */
    M_FROM_CURRENT=128, /* crossing measured from the current corner */
    M_NO_LIMIT=256,     /* pass limit ignored (16) */
    M_NO_FAIL=512       /* snap failures ignored */
};
typedef struct { wide_t c[7]; } opt_t;
typedef struct { const sh2z_view_t *v; wide_t cx,cy; int16_t g[4]; } ocam_t;
typedef struct { uint64_t small,mid,huge,full,snapped,zsnapped,snap_fail,range_fail,overflow; } ostats_t;
static ostats_t ostats;

static wide_t oplane(const ocam_t *o,const opt_t *p,unsigned plane) {
    const wide_t x=p->c[0],y=p->c[1],z=p->c[2];
    switch(plane) {
    case 0: return z-o->v->near_z;
    case 1: return o->v->far_z-z;
    case 2: return (wide_t)o->v->focal_x*x+(o->cx-(wide_t)o->g[0]*65536)*z;
    case 3: return ((wide_t)o->g[2]*65536-o->cx)*z-(wide_t)o->v->focal_x*x;
    case 4: return (o->cy-(wide_t)o->g[1]*65536)*z-(wide_t)o->v->focal_y*y;
    default: return (wide_t)o->v->focal_y*y+((wide_t)o->g[3]*65536-o->cy)*z;
    }
}
/* The source's inward step; done[k] marks planes clipped so far. */
static bool oinward(const ocam_t *o,opt_t *p,const bool done[6],unsigned flags) {
    if(!(flags&M_NO_ZSNAP)) {
        if(p->c[2]<o->v->near_z){p->c[2]=o->v->near_z;++ostats.zsnapped;}
        if(done[1]&&p->c[2]>o->v->far_z){p->c[2]=o->v->far_z;++ostats.zsnapped;}
    }
    for(unsigned k=2;k<6;++k) {
        if(!done[k])continue;
        const unsigned axis=k<4?0u:1u;
        const bool lower=k==2||k==5;
        const int16_t edge=o->g[k==2?0:k==3?2:k==4?1:3];
        const wide_t bound=(axis==0?(wide_t)edge*65536-o->cx:o->cy-(wide_t)edge*65536)*p->c[2];
        const wide_t f=axis==0?o->v->focal_x:o->v->focal_y;
        const wide_t n=lower?ceil_wide(bound,f):floor_wide(bound,f);
        if(n<INT32_MIN||n>INT32_MAX){++ostats.range_fail;if(!(flags&M_NO_FAIL))return false;continue;}
        if((lower&&p->c[axis]<n)||(!lower&&p->c[axis]>n)){p->c[axis]=n;++ostats.snapped;}
    }
    for(unsigned k=0;k<6;++k)if(done[k]&&oplane(o,p,k)<0){++ostats.snap_fail;if(!(flags&M_NO_FAIL))return false;}
    return true;
}
/* The crossing on the edge from -> to (from = previous corner). */
static opt_t ocross(const opt_t *from,const opt_t *to,wide_t df,wide_t dt,unsigned plane,unsigned coords,unsigned flags) {
    opt_t q;
    memset(&q,0,sizeof(q));
    if(flags&M_LIB_CROSS) {
        /* view-guard-1: from the inside end i towards the outside end o. */
        const opt_t *i=df>=0?from:to,*o=df>=0?to:from;
        const wide_t num=df>=0?df:dt,den=df>=0?df-dt:dt-df;
        wide_t t=0;
        if(plane>=2){unsigned s=0;while((den>>s)>=((wide_t)1<<31))++s;t=((num>>s)<<30)/(den>>s);}
        for(unsigned k=0;k<coords;++k) {
            const wide_t delta=o->c[k]-i->c[k],m=absw(delta);
            const wide_t off=plane>=2?(m*t)>>30:m*num/den;
            q.c[k]=i->c[k]+(delta<0?-off:off);
        }
        return q;
    }
    const opt_t *base=(flags&M_FROM_CURRENT)?to:from,*other=(flags&M_FROM_CURRENT)?from:to;
    const wide_t a=absw((flags&M_FROM_CURRENT)?dt:df),b=absw((flags&M_FROM_CURRENT)?df:dt);
    const wide_t den=a+b,one=(wide_t)1<<32;
    if(den<((wide_t)1<<31))++ostats.small;else if(den<((wide_t)1<<59))++ostats.mid;else ++ostats.huge;
    if(b==0)++ostats.full;
    const wide_t t=(flags&M_ROUND_T)?(2*a*one+den)/(2*den):a*one/den;
    for(unsigned k=0;k<coords;++k) {
        const wide_t delta=other->c[k]-base->c[k],step=(absw(delta)*t)>>32;
        q.c[k]=base->c[k]+(delta<0?-step:step);
    }
    return q;
}
/* Returns the count (0, 3..max) or -2 (overflow) / -3 (snap); out gets the
 * projected vertices. */
static int source_clip(sh2z_view_guard_vertex_t *out,const ocam_t *o,const sh2z_view_guard_vertex_t *in,
    unsigned count,unsigned attrs,unsigned max_out,unsigned flags) {
    static const unsigned normal[6]={0,1,2,3,4,5},swapped[6]={0,1,2,4,3,5};
    const unsigned *order=(flags&M_PLANE_ORDER)?swapped:normal;
    const unsigned limit=(flags&M_NO_LIMIT)?16u:max_out,coords=3u+attrs;
    opt_t poly[64],next[64];
    bool done[6]={false,false,false,false,false,false};
    for(unsigned k=0;k<count;++k) {
        memset(&poly[k],0,sizeof(poly[k]));
        poly[k].c[0]=in[k].v.x;poly[k].c[1]=in[k].v.y;poly[k].c[2]=in[k].v.z;
        for(unsigned a=0;a<attrs;++a)poly[k].c[3+a]=in[k].attr[a];
    }
    unsigned n=count;
    for(unsigned step=0;step<6&&n>=3;++step) {
        const unsigned plane=order[step];
        done[plane]=true;
        unsigned m=0;
        if(flags&M_LIB_ORDER) {
            for(unsigned i=0;i<n;++i) {
                const unsigned j=(i+1)%n;
                const wide_t di=oplane(o,&poly[i],plane),dj=oplane(o,&poly[j],plane);
                if(di>=0){if(m==limit){++ostats.overflow;return -2;}next[m++]=poly[i];}
                if((di<0)!=(dj<0)) {
                    if(m==limit){++ostats.overflow;return -2;}
                    next[m]=ocross(&poly[i],&poly[j],di,dj,plane,coords,flags);
                    if(!(flags&M_NO_SNAP)&&!oinward(o,&next[m],done,flags))return -3;
                    ++m;
                }
            }
        } else {
            const unsigned first=(flags&M_START)?1u:0u;
            unsigned prev=(first+n-1)%n;
            wide_t dp=oplane(o,&poly[prev],plane);
            for(unsigned s=0;s<n;++s) {
                const unsigned i=(first+s)%n;
                const wide_t dc=oplane(o,&poly[i],plane);
                if((dp<0)!=(dc<0)) {
                    if(m==limit){++ostats.overflow;return -2;}
                    next[m]=ocross(&poly[prev],&poly[i],dp,dc,plane,coords,flags);
                    if(!(flags&M_NO_SNAP)&&!oinward(o,&next[m],done,flags))return -3;
                    ++m;
                }
                if(dc>=0){if(m==limit){++ostats.overflow;return -2;}next[m++]=poly[i];}
                prev=i;dp=dc;
            }
        }
        n=m;
        memcpy(poly,next,n*sizeof(poly[0]));
    }
    if(n<3)return 0;
    for(unsigned k=0;k<n;++k) {
        memset(&out[k],0,sizeof(out[k]));
        const wide_t x=poly[k].c[0],y=poly[k].c[1],z=poly[k].c[2];
        check(x>=INT32_MIN&&x<=INT32_MAX&&y>=INT32_MIN&&y<=INT32_MAX&&z>0&&z<=INT32_MAX,"oracle point fits");
        wide_t sx=toward_zero(o->cx+toward_zero((wide_t)o->v->focal_x*x,z),65536);
        wide_t sy=toward_zero(o->cy-toward_zero((wide_t)o->v->focal_y*y,z),65536);
        if(!flags) {
            /* The source's own points lie inside every plane, so inside the
             * rectangle without any clamp. */
            for(unsigned p=0;p<6;++p)check(oplane(o,&poly[k],p)>=0,"source output inside every plane");
            check(sx>=o->g[0]&&sx<=o->g[2]&&sy>=o->g[1]&&sy<=o->g[3],"source output inside the rectangle");
        }
        sx=clampw(sx,o->g[0],o->g[2]);sy=clampw(sy,o->g[1],o->g[3]);
        out[k].v.x=(int32_t)x;out[k].v.y=(int32_t)y;out[k].v.z=(int32_t)z;
        out[k].v.sx=(int16_t)sx;out[k].v.sy=(int16_t)sy;
        const sh2z_view_t *v=o->v;
        out[k].v.outcode=(uint8_t)((sx<v->clip[0]?SH2Z_OUT_LEFT:0u)|(sx>v->clip[2]?SH2Z_OUT_RIGHT:0u)|
            (sy<v->clip[1]?SH2Z_OUT_TOP:0u)|(sy>v->clip[3]?SH2Z_OUT_BOTTOM:0u));
        for(unsigned a=0;a<attrs;++a)out[k].attr[a]=(int32_t)poly[k].c[3+a];
    }
    return (int)n;
}
static ocam_t ocam(const sh2z_view_t *v,const int16_t g[4]) {
    ocam_t o;
    o.v=v;o.cx=(wide_t)v->center_x*65536;o.cy=(wide_t)v->center_y*65536;
    memcpy(o.g,g,sizeof(o.g));
    return o;
}

/* ---- cameras and polygons ---------------------------------------------------- */
static int16_t q14(double x) {const double s=floor(x*16384.0+0.5);return (int16_t)(s>32767?32767:s<-32768?-32768:s);}
static sh2z_view_t random_view(void) {
    sh2z_view_t v;
    memset(&v,0,sizeof(v));
    const double a=(random_word()%65536)*6.283185307179586/65536.0,b=((double)(random_word()%65536)/65536.0-0.5)*1.5;
    const double ca=cos(a),sa=sin(a),cb=cos(b),sb=sin(b);
    const double m[3][3]={{ca,0,-sa},{sa*sb,cb,ca*sb},{sa*cb,-sb,ca*cb}};
    for(unsigned i=0;i<3;++i)for(unsigned j=0;j<3;++j)v.r[i][j]=q14(m[i][j]);
    switch(random_word()%4u) {
    case 0: v.focal_x=v.focal_y=160*65536;break;
    case 1: v.focal_x=65536*random_range(16,1024);v.focal_y=65536*random_range(16,1024);break;
    case 2: v.focal_x=random_range(65536,1024*65536);v.focal_y=random_range(65536,1024*65536);break;
    default: v.focal_x=random_range(1<<28,INT32_MAX);v.focal_y=random_range(1<<28,INT32_MAX);
    }
    switch(random_word()%3u) {
    case 0: v.center_x=160;v.center_y=112;break;
    case 1: v.center_x=(int16_t)random_range(-2000,2000);v.center_y=(int16_t)random_range(-2000,2000);break;
    default: v.center_x=(int16_t)random_range(0,352);v.center_y=(int16_t)random_range(0,256);
    }
    v.near_z=random_word()%8u==0?random_range(1,64):random_range(1,16*16384);
    v.far_z=v.near_z+(random_word()%2u?random_range(1,1<<24):random_range(1,INT32_MAX-v.near_z));
    v.clip[0]=(int16_t)random_range(-64,160);v.clip[2]=(int16_t)random_range(v.clip[0],400);
    v.clip[1]=(int16_t)random_range(-64,120);v.clip[3]=(int16_t)random_range(v.clip[1],300);
    check(sh2z_view_init(&v,&v),"random view valid");
    return v;
}
static void random_rect(const sh2z_view_t *v,int16_t g[4]) {
    switch(random_word()%4u) {
    case 0: g[0]=-900;g[1]=-900;g[2]=900;g[3]=900;break;
    case 1: g[0]=v->clip[0];g[1]=v->clip[1];g[2]=v->clip[2];g[3]=v->clip[3];break;
    case 2: g[0]=(int16_t)random_range(v->center_x-3,v->center_x+3);g[2]=(int16_t)(g[0]+random_range(0,2));
        g[1]=(int16_t)random_range(v->center_y-3,v->center_y+3);g[3]=(int16_t)(g[1]+random_range(0,2));break;
    default: g[0]=(int16_t)random_range(v->center_x-2000,v->center_x+200);g[2]=(int16_t)random_range(g[0],v->center_x+2000);
        g[1]=(int16_t)random_range(v->center_y-2000,v->center_y+200);g[3]=(int16_t)random_range(g[1],v->center_y+2000);
    }
}
/* A view-space corner at pixel (px, py) and depth z, exactly on the pixel when
 * the focal length divides evenly; sometimes nudged by a unit or more. */
static sh2z_view_guard_vertex_t at_pixel(const sh2z_view_t *v,int32_t px,int32_t py,int32_t z,int nudge) {
    sh2z_view_guard_vertex_t p;
    memset(&p,0,sizeof(p));
    p.v.x=(int32_t)clampw(toward_zero((wide_t)(px-v->center_x)*65536*z,v->focal_x)+nudge,INT32_MIN,INT32_MAX);
    p.v.y=(int32_t)clampw(toward_zero((wide_t)(v->center_y-py)*65536*z,v->focal_y)-nudge,INT32_MIN,INT32_MAX);
    p.v.z=z;
    for(unsigned i=0;i<4;++i)p.attr[i]=(int32_t)random_word();
    return p;
}
static int32_t random_depth(const sh2z_view_t *v) {
    switch(random_word()%6u) {
    case 0: return v->near_z;
    case 1: return v->far_z;
    case 2: return random_coord();
    case 3: return random_range(1,v->near_z+(1<<22)>0?v->near_z+(1<<22):INT32_MAX);
    case 4: return random_range(-v->near_z-(1<<20),v->near_z);
    default: return random_range(v->near_z,v->far_z);
    }
}
static int nudge(void) {
    switch(random_word()%4u){case 0: case 1: return 0;case 2: return random_range(-2,2);default: return random_range(-100000,100000);}
}
static void random_polygon(const sh2z_view_t *v,const int16_t g[4],sh2z_view_guard_vertex_t *in,unsigned n) {
    memset(in,0,n*sizeof(*in));
    const unsigned style=random_word()%8u;
    for(unsigned k=0;k<n;++k) {
        if(style==0||random_word()%8u==0) {
            in[k].v.x=random_coord();in[k].v.y=random_coord();in[k].v.z=random_coord();
            for(unsigned i=0;i<4;++i)in[k].attr[i]=(int32_t)random_word();
        } else if(style==1) {
            /* corners exactly on a guard edge at a depth the focal divides */
            /* (each random draw its own statement: argument order is unspecified) */
            const int16_t ex=g[random_word()%2u?0:2],ey=g[random_word()%2u?1:3];
            const int32_t px=random_word()%2u?ex:random_range(g[0]-500,g[2]+500);
            const int32_t py=random_word()%2u?ey:random_range(g[1]-500,g[3]+500);
            const int32_t pz=random_word()%2u?v->near_z:random_range(v->near_z,v->near_z+65536);
            const int unit=random_word()%4u?0:random_range(-1,1);
            in[k]=at_pixel(v,px,py,pz,unit);
        } else {
            const int32_t px=random_range(g[0]-3000,g[2]+3000);
            const int32_t py=random_range(g[1]-3000,g[3]+3000);
            const int32_t pz=random_depth(v);
            const int unit=nudge();
            in[k]=at_pixel(v,px,py,pz,unit);
        }
    }
    if(style==2&&n>=4) {
        /* degenerate: a repeated corner, and corners midway between 0 and 2 */
        in[1]=in[0];
        for(unsigned k=3;k<n;++k)if(random_word()%2u) {
            in[k].v.x=(int32_t)(((int64_t)in[0].v.x+in[2].v.x)/2);in[k].v.y=(int32_t)(((int64_t)in[0].v.y+in[2].v.y)/2);
            in[k].v.z=(int32_t)(((int64_t)in[0].v.z+in[2].v.z)/2);
        }
    }
    if(style==3) {
        /* edges symmetric about a plane: crossings on lattice points */
        const int32_t z=random_range(v->near_z,v->near_z+(1<<20));
        const int16_t ex=g[random_word()%2u?0:2],ey=g[random_word()%2u?1:3];
        const sh2z_view_guard_vertex_t c=at_pixel(v,ex,ey,z,0);
        for(unsigned k=0;k<n;++k){const int32_t dx=2*random_range(-(1<<20),1<<20),dy=2*random_range(-(1<<20),1<<20);
            in[k].v.x=c.v.x+((k&1u)?dx:-dx);in[k].v.y=c.v.y+((k&2u)?dy:-dy);in[k].v.z=z+((k&4u)?2*v->near_z:0);}
    }
}
/* A port-like camera: centre 160,112, screen 0..320 x 0..224, focal 140 to
 * 320 px, looking slightly down onto a road. */
static sh2z_view_t road_view(double *pitch,double *yaw) {
    sh2z_view_t v;
    memset(&v,0,sizeof(v));
    *pitch=((random_word()%65536)/65536.0)*0.35;
    *yaw=((random_word()%65536)/65536.0-0.5)*0.6;
    const double cp=cos(*pitch),sp=sin(*pitch);
    v.r[0][0]=v.r[1][1]=v.r[2][2]=16384;(void)sp;(void)cp;
    const int32_t f=random_range(140,320);
    v.focal_x=random_word()%2u?f*65536:random_range(140*65536,320*65536);v.focal_y=v.focal_x;
    v.center_x=160;v.center_y=112;
    v.near_z=random_range(4096,16*16384);v.far_z=random_range(1<<28,INT32_MAX);
    v.clip[0]=0;v.clip[1]=0;v.clip[2]=320;v.clip[3]=224;
    check(sh2z_view_init(&v,&v),"road view valid");
    return v;
}
/* A road quad on the ground plane under the camera, in view space. */
static void road_quad(sh2z_view_guard_vertex_t q[4],double pitch,double yaw) {
    memset(q,0,4*sizeof(*q));
    const double u=16384.0,h=(5.0+(random_word()%65536)/65536.0*60.0)*u;
    const double w=(20.0+(random_word()%65536)/65536.0*600.0)*u,len=(20.0+(random_word()%65536)/65536.0*1500.0)*u;
    const double x0=((random_word()%65536)/65536.0-0.5)*3000.0*u,z0=(-200.0+(random_word()%65536)/65536.0*3000.0)*u;
    const double corners[4][2]={{x0,z0},{x0+w,z0},{x0+w,z0+len},{x0,z0+len}};
    const double cp=cos(pitch),sp=sin(pitch),cy=cos(yaw),sy=sin(yaw);
    const bool flip=random_word()%2u!=0;
    for(unsigned k=0;k<4;++k) {
        const unsigned c=flip?3u-k:k;
        const double wx=corners[c][0],wy=-h,wz=corners[c][1];
        const double x1=cy*wx-sy*wz,z1=sy*wx+cy*wz;           /* yaw */
        const double y2=cp*wy+sp*z1,z2=-sp*wy+cp*z1;          /* pitch down */
        q[k].v.x=(int32_t)floor(x1+0.5);q[k].v.y=(int32_t)floor(y2+0.5);q[k].v.z=(int32_t)floor(z2+0.5);
        q[k].attr[0]=c&1u?64:0;q[k].attr[1]=c&2u?64:0;
    }
}
/* The port's trigger: a corner behind the camera or projecting outside +/-900. */
static bool road_unsafe(const sh2z_view_t *v,const sh2z_view_guard_vertex_t q[4]) {
    for(unsigned k=0;k<4;++k) {
        sh2z_view_vertex_t p=q[k].v;
        check(sh2z_view_project_trunc(&p,v,&p,1),"road projection");
        if((p.outcode&SH2Z_OUT_NEAR)||p.sx< -900||p.sx>900||p.sy< -900||p.sy>900)return true;
    }
    return false;
}

/* ---- comparisons -------------------------------------------------------------- */
static bool same_vertex(const sh2z_view_guard_vertex_t *a,const sh2z_view_guard_vertex_t *b) {
    return a->v.x==b->v.x&&a->v.y==b->v.y&&a->v.z==b->v.z&&a->v.sx==b->v.sx&&a->v.sy==b->v.sy&&
        a->v.outcode==b->v.outcode&&!a->v.reserved[0]&&!a->v.reserved[1]&&!a->v.reserved[2]&&!memcmp(a->attr,b->attr,sizeof(a->attr));
}
static bool same_polygon(const sh2z_view_guard_vertex_t *a,const sh2z_view_guard_vertex_t *b,int n) {
    for(int k=0;k<n;++k)if(!same_vertex(&a[k],&b[k]))return false;
    return true;
}
/* The rotation r (0 preferred on ties) that best aligns a[(k + r) % n] with
 * b[k]: least largest view-space difference. */
static int best_rotation(const sh2z_view_guard_vertex_t *a,const sh2z_view_guard_vertex_t *b,int n) {
    int best=0;
    wide_t best_d=-1;
    for(int r=0;r<n;++r) {
        wide_t d=0;
        for(int k=0;k<n;++k) {
            const sh2z_view_guard_vertex_t *p=&a[(k+r)%n],*q=&b[k];
            const wide_t e[3]={absw((wide_t)p->v.x-q->v.x),absw((wide_t)p->v.y-q->v.y),absw((wide_t)p->v.z-q->v.z)};
            for(unsigned i=0;i<3;++i)if(e[i]>d)d=e[i];
        }
        if(best_d<0||d<best_d){best=r;best_d=d;}
    }
    return best;
}
/* 0: a rotated by r equals b exactly, 1: in pixels only, 2: pixels differ. */
static int rotated_match(const sh2z_view_guard_vertex_t *a,const sh2z_view_guard_vertex_t *b,int n,int r) {
    int worst=0;
    for(int k=0;k<n;++k) {
        const sh2z_view_guard_vertex_t *p=&a[(k+r)%n],*q=&b[k];
        if(p->v.sx!=q->v.sx||p->v.sy!=q->v.sy)return 2;
        if(p->v.x!=q->v.x||p->v.y!=q->v.y||p->v.z!=q->v.z)worst=1;
    }
    return worst;
}

/* ---- (a) (b) (c): the view-round-1 clip against the source ---------------------- */
typedef struct {
    uint64_t polygons,clipped,lib_count,lib_kind[2][3];   /* [rotated][exact, pixels only, pixels differ] */
    uint64_t lib_oracle_bad;                     /* oracle in library mode vs the library */
    uint64_t b_polys,b_count,b_vertices,b_vertex_bad,b_pixel_bad,b_poly_vbad,b_poly_pbad; wide_t b_max_unit; int b_max_px;
    uint64_t s_polys,s_count,s_vertex_bad,s_pixel_bad; wide_t s_max_unit; int s_max_px;
    uint64_t c_vertices,c_bad,divu_bad,negative,overflow,snap,inplace;
} study_t;
static unsigned diff_stats(const sh2z_view_guard_vertex_t *a,const sh2z_view_guard_vertex_t *b,int n,
    uint64_t *vbad,uint64_t *pbad,wide_t *max_unit,int *max_px) {
    const uint64_t v0=*vbad,p0=*pbad;
    for(int k=0;k<n;++k) {
        const wide_t dx=absw((wide_t)a[k].v.x-b[k].v.x),dy=absw((wide_t)a[k].v.y-b[k].v.y),dz=absw((wide_t)a[k].v.z-b[k].v.z);
        const wide_t d=dx>dy?(dx>dz?dx:dz):(dy>dz?dy:dz);
        if(d){++*vbad;if(d>*max_unit)*max_unit=d;}
        const int px=abs(a[k].v.sx-b[k].v.sx),py=abs(a[k].v.sy-b[k].v.sy),p=px>py?px:py;
        if(p){++*pbad;if(p>*max_px)*max_px=p;}
    }
    return (*vbad!=v0?1u:0u)|(*pbad!=p0?2u:0u);
}
static unsigned mutation;
/* One polygon through every comparison. Returns mismatches of the library
 * against the oracle under the current mutation. */
static uint64_t study_one(study_t *s,const sh2z_view_t *v,const int16_t g[4],const sh2z_view_guard_vertex_t *in,
    unsigned n,unsigned attrs,unsigned max_out) {
    const ocam_t o=ocam(v,g);
    sh2z_view_guard_vertex_t want[16],got[16],gd[16],lib[16],lo[16],bv[16],sv[16];
    memset(got,0x5A,sizeof(got));memset(gd,0x5A,sizeof(gd));
    const int w=source_clip(want,&o,in,n,attrs,max_out,mutation);
    const int rp=sh2z_view_clip_guard_src_trunc(got,v,g,in,n,attrs,max_out);
    const int rd=sh2z_view_clip_guard_src_trunc_divu(gd,v,g,in,n,attrs,max_out);
    uint64_t bad=0;
    if(rp!=w)bad+=1;
    else if(w>0)for(int k=0;k<w;++k)bad+=!same_vertex(&got[k],&want[k]);
    if(mutation)return bad;
    ++s->polygons;
    check(rp==rd,"divu count equals portable");
    if(rp>0)check(!memcmp(got,gd,(size_t)rp*sizeof(got[0])),"divu bytes equal portable");
    else {
        unsigned char pattern[sizeof(got)];memset(pattern,0x5A,sizeof(pattern));
        check(!memcmp(got,pattern,sizeof(got))&&!memcmp(gd,pattern,sizeof(gd)),"nothing written on failure or empty");
    }
    s->c_bad+=bad;
    if(rp<0){++s->negative;s->overflow+=rp==SH2Z_VIEW_GUARD_OVERFLOW;s->snap+=rp==SH2Z_VIEW_GUARD_SNAP;}
    if(rp>0) {
        s->c_vertices+=(uint64_t)rp;
        for(int k=0;k<rp;++k) {
            check(got[k].v.sx>=g[0]&&got[k].v.sx<=g[2]&&got[k].v.sy>=g[1]&&got[k].v.sy<=g[3],"output inside the guard rectangle");
            sh2z_view_vertex_t p=got[k].v;
            check(sh2z_view_project_trunc(&p,v,&p,1)&&p.sx==got[k].v.sx&&p.sy==got[k].v.sy&&p.outcode==got[k].v.outcode,
                "projection is sh2z_view_project_trunc");
            for(unsigned a=attrs;a<4;++a)check(got[k].attr[a]==0,"unused attributes zero");
        }
        /* in place */
        sh2z_view_guard_vertex_t io[16];memset(io,0,sizeof(io));memcpy(io,in,n*sizeof(in[0]));
        check(sh2z_view_clip_guard_src_trunc(io,v,g,io,n,attrs,max_out)==rp&&!memcmp(io,got,(size_t)rp*sizeof(io[0])),"in place");
        ++s->inplace;
    }
    /* (a) the view-round-1 clip; the oracle in library mode must equal it. */
    const int rl=sh2z_view_clip_guard_trunc(lib,v,g,in,n,attrs);
    const int ro=source_clip(lo,&o,in,n,attrs,16,M_LIB_ORDER|M_LIB_CROSS|M_NO_SNAP);
    if(rl!=ro||(rl>0&&!same_polygon(lib,lo,rl)))++s->lib_oracle_bad;
    if(w<0||rl<0)return bad;
    bool clipped=false;
    for(unsigned p=0;p<6&&!clipped;++p)for(unsigned k=0;k<n;++k){opt_t q;memset(&q,0,sizeof(q));q.c[0]=in[k].v.x;q.c[1]=in[k].v.y;q.c[2]=in[k].v.z;
        if(oplane(&o,&q,p)<0){clipped=true;break;}}
    if(!clipped||w==0)return bad;
    ++s->clipped;
    if(rl!=w)++s->lib_count;
    else {const int r=best_rotation(lib,want,w);++s->lib_kind[r!=0][rotated_match(lib,want,w,r)];}
    /* (b) source order and planes, library crossings, no snap. */
    const int rb=source_clip(bv,&o,in,n,attrs,16,M_LIB_CROSS|M_NO_SNAP);
    ++s->b_polys;
    if(rb!=w)++s->b_count;
    else {
        s->b_vertices+=(uint64_t)w;
        const unsigned f=diff_stats(bv,want,w,&s->b_vertex_bad,&s->b_pixel_bad,&s->b_max_unit,&s->b_max_px);
        s->b_poly_vbad+=f&1u;s->b_poly_pbad+=f>>1;
    }
    /* source crossings without the snap */
    const int rs=source_clip(sv,&o,in,n,attrs,16,M_NO_SNAP);
    ++s->s_polys;
    if(rs!=w)++s->s_count;
    else (void)diff_stats(sv,want,w,&s->s_vertex_bad,&s->s_pixel_bad,&s->s_max_unit,&s->s_max_px);
    return bad;
}
static uint64_t run_road(study_t *s,unsigned trials) {
    uint64_t bad=0;
    for(unsigned t=0;t<trials;++t) {
        double pitch,yaw;
        const sh2z_view_t v=road_view(&pitch,&yaw);
        const int16_t g[4]={-900,-900,900,900};
        sh2z_view_guard_vertex_t q[4];
        unsigned tries=0;
        do road_quad(q,pitch,yaw);while(!road_unsafe(&v,q)&&++tries<64);
        bad+=study_one(s,&v,g,q,4,random_word()%3u,10);
    }
    return bad;
}
static uint64_t run_random(study_t *s,unsigned trials) {
    uint64_t bad=0;
    for(unsigned t=0;t<trials;++t) {
        const sh2z_view_t v=random_view();
        int16_t g[4];random_rect(&v,g);
        if(g[0]<v.center_x-16384||g[2]>v.center_x+16384||g[1]<v.center_y-16384||g[3]>v.center_y+16384)continue;
        sh2z_view_guard_vertex_t in[8];
        const unsigned n=3u+random_word()%6u,attrs=random_word()%5u;
        random_polygon(&v,g,in,n);
        const unsigned max_out=random_word()%4u==0?(unsigned)random_range(3,16):random_word()%2u?10u:16u;
        bad+=study_one(s,&v,g,in,n,attrs,max_out);
    }
    return bad;
}

/* ---- hand cases ------------------------------------------------------------------ */
static sh2z_view_t port_view(void) {
    sh2z_view_t v;
    memset(&v,0,sizeof(v));
    v.r[0][0]=v.r[1][1]=v.r[2][2]=16384;
    v.focal_x=v.focal_y=160*65536;v.center_x=160;v.center_y=112;
    v.near_z=16*16384;v.far_z=10000*16384;
    v.clip[0]=0;v.clip[1]=0;v.clip[2]=320;v.clip[3]=224;
    check(sh2z_view_init(&v,&v),"port view valid");
    return v;
}
static void hand_cases(void) {
    sh2z_view_t v=port_view();
    const int16_t g[4]={-900,-900,900,900};
    const ocam_t o=ocam(&v,g);
    sh2z_view_guard_vertex_t in[8],out[16],want[16],lib[16];
    /* A quad whose closing edge (last -> first) crosses the left plane:
     * the source starts with that crossing, view-round-1 with corner 0. */
    const int32_t z=160*16384;
    memset(in,0,sizeof(in));
    in[0]=at_pixel(&v,100,150,z,0);in[1]=at_pixel(&v,300,150,z,0);in[2]=at_pixel(&v,300,60,z,0);in[3]=at_pixel(&v,-2000,60,z,0);
    int n=sh2z_view_clip_guard_src_trunc(out,&v,g,in,4,0,10),w=source_clip(want,&o,in,4,0,10,0);
    const int nl=sh2z_view_clip_guard_trunc(lib,&v,g,in,4,0);
    check(n==5&&w==5&&same_polygon(out,want,5)&&nl==5,"closing-edge crossing: five corners");
    check(out[0].v.sx==-900&&out[1].v.sx==100&&out[4].v.sx==-899,"source order starts with the closing crossing");
    /* view-round-1 starts at corner 0: corners 0..3 are the source's 1..4,
     * and its closing crossing, measured from the inside end without the
     * snap, is one unit off the source's on x and y and one pixel off on
     * screen (-899, 107 against -900, 107). */
    for(int k=0;k<4;++k)check(same_vertex(&lib[k],&out[k+1]),"view-round-1 is the source rotated");
    check(lib[4].v.x==out[0].v.x+1&&lib[4].v.y==out[0].v.y-1&&lib[4].v.sx==-899&&out[0].v.sx==-900&&lib[4].v.sy==out[0].v.sy,
        "and its closing crossing is one unit and one pixel off");
    /* A corner exactly on the left plane, previous corner outside: the
     * crossing is that corner (t = 2^32) and it repeats. */
    in[0]=at_pixel(&v,-900,150,z,0);in[3]=at_pixel(&v,-2000,60,z,0);
    n=sh2z_view_clip_guard_src_trunc(out,&v,g,in,4,0,10);w=source_clip(want,&o,in,4,0,10,0);
    check(n==w&&n==5&&same_polygon(out,want,n)&&out[0].v.x==in[0].v.x&&out[1].v.x==in[0].v.x,"corner on the plane repeats");
    /* An edge symmetric about the left plane: the crossing is the midpoint. */
    memset(in,0,sizeof(in));
    const int32_t xl=(int32_t)toward_zero((wide_t)(-900-160)*65536*z,v.focal_x);
    in[0].v.x=xl-1000;in[0].v.y=0;in[0].v.z=z;in[1].v.x=xl+1000;in[1].v.y=0;in[1].v.z=z;in[2].v.x=xl+1000;in[2].v.y=-1000;in[2].v.z=z;
    n=sh2z_view_clip_guard_src_trunc(out,&v,g,in,3,0,10);w=source_clip(want,&o,in,3,0,10,0);
    check(n==w&&same_polygon(out,want,n),"lattice crossing");
    bool mid=false;for(int k=0;k<n;++k)mid=mid||(out[k].v.x==xl&&out[k].v.y==0);
    check(mid,"symmetric edge crosses at its midpoint");
    /* The pass limit: an octagon whose four axis corners poke out of the
     * rectangle clips to twelve corners. */
    memset(in,0,sizeof(in));
    static const int32_t octagon[8][2]={{1500,0},{600,600},{0,1500},{-600,600},{-1500,0},{-600,-600},{0,-1500},{600,-600}};
    for(unsigned k=0;k<8;++k) {
        in[k]=at_pixel(&v,160+octagon[k][0],112+octagon[k][1],z,0);
        in[k].attr[0]=(int32_t)k;
    }
    n=sh2z_view_clip_guard_src_trunc(out,&v,g,in,8,1,16);w=source_clip(want,&o,in,8,1,16,0);
    check(n==w&&n>10&&same_polygon(out,want,n),"more than ten corners within 16");
    check(sh2z_view_clip_guard_src_trunc(out,&v,g,in,8,1,10)==SH2Z_VIEW_GUARD_OVERFLOW&&
        source_clip(want,&o,in,8,1,10,0)==-2,"max_out 10 overflows like the source");
    printf("  hand cases: closing crossing first, repeated on-plane corner, lattice midpoint, %d corners (overflow at 10)\n",n);
    /* A lattice bound outside int32: focal_x is 1/65536 px, so at a top
     * crossing 1000 units deep the left bound is -2^30 * 1000. */
    sh2z_view_t flat=v;flat.focal_x=1;flat.near_z=1;flat.far_z=INT32_MAX;flat.center_x=0;flat.center_y=0;
    check(sh2z_view_init(&flat,&flat),"flat view valid");
    const int16_t wide_rect[4]={-16384,-16384,16384,16384};
    const ocam_t of=ocam(&flat,wide_rect);
    memset(in,0,sizeof(in));
    in[0].v.z=1000;in[1].v.y=200000;in[1].v.z=1000;in[2].v.x=10;in[2].v.z=1000;
    memset(&ostats,0,sizeof(ostats));
    w=source_clip(want,&of,in,3,0,16,0);
    n=sh2z_view_clip_guard_src_trunc(out,&flat,wide_rect,in,3,0,16);
    check(w==-3&&ostats.range_fail>0&&n==SH2Z_VIEW_GUARD_SNAP,"lattice bound outside int32 fails the snap");
    printf("  snap failures: empty lattice interval and int32 range, both SH2Z_VIEW_GUARD_SNAP like the source's -1\n");
}
static void argument_rules(void) {
    sh2z_view_t v=port_view(),bad=v;
    bad.focal_x=0;
    const int16_t rect[4]={-900,-900,900,900},flipped[4]={900,-900,-900,900},far_rect[4]={-20000,-900,900,900};
    sh2z_view_guard_vertex_t in[9],out[16],pattern[16];
    memset(in,0,sizeof(in));memset(pattern,0x3C,sizeof(pattern));
    for(unsigned k=0;k<9;++k){in[k].v.x=(int32_t)k*1000;in[k].v.z=100000;}
    struct { const sh2z_view_t *v; const int16_t *g; const sh2z_view_guard_vertex_t *in; unsigned n,attrs,max; } cases[]={
        {&bad,rect,in,4,0,10},{&v,flipped,in,4,0,10},{&v,far_rect,in,4,0,10},{&v,rect,in,2,0,10},{&v,rect,in,9,0,10},
        {&v,rect,in,4,5,10},{&v,rect,in,4,0,2},{&v,rect,in,4,0,17},{&v,NULL,in,4,0,10},{&v,rect,NULL,4,0,10},{NULL,rect,in,4,0,10}};
    for(unsigned c=0;c<sizeof(cases)/sizeof(cases[0]);++c)for(int d=0;d<2;++d) {
        memcpy(out,pattern,sizeof(out));
        const int r=d?sh2z_view_clip_guard_src_trunc_divu(out,cases[c].v,cases[c].g,cases[c].in,cases[c].n,cases[c].attrs,cases[c].max):
            sh2z_view_clip_guard_src_trunc(out,cases[c].v,cases[c].g,cases[c].in,cases[c].n,cases[c].attrs,cases[c].max);
        check(r==SH2Z_VIEW_GUARD_INVALID&&!memcmp(out,pattern,sizeof(out)),"invalid arguments rejected, nothing written");
    }
    check(sh2z_view_clip_guard_src_trunc(NULL,&v,rect,in,4,0,10)==SH2Z_VIEW_GUARD_INVALID,"null output");
    /* The same rectangle rules as view-guard-1: 16384 px from the centre is fine. */
    const int16_t edge_rect[4]={160-16384,112-16384,160+16384,112+16384},over[4]={160-16385,0,0,0};
    check(sh2z_view_clip_guard_src_trunc(out,&v,edge_rect,in,4,0,16)>=0&&
        sh2z_view_clip_guard_src_trunc(out,&v,over,in,4,0,16)==SH2Z_VIEW_GUARD_INVALID,"rectangle distance rule");
}

int main(void) {
    hand_cases();
    argument_rules();
    study_t road,rnd;
    memset(&road,0,sizeof(road));memset(&rnd,0,sizeof(rnd));
    memset(&ostats,0,sizeof(ostats));
    state=UINT32_C(0x0ad0ad01);
    uint64_t bad=run_road(&road,200000);
    state=UINT32_C(0x7a9d0b1e);
    bad+=run_random(&rnd,400000);
    check(bad==0&&road.c_bad==0&&rnd.c_bad==0,"source-order clip equals the source oracle");
    check(road.lib_oracle_bad==0&&rnd.lib_oracle_bad==0,"oracle in view-guard-1 mode equals sh2z_view_clip_guard_trunc");
    check(ostats.small>0&&ostats.mid>0&&ostats.huge>0&&ostats.full>0,"every fraction range exercised");
    check(ostats.snapped>0&&ostats.zsnapped>0&&ostats.snap_fail>0&&ostats.range_fail>0&&ostats.overflow>0,"snaps, both snap failures and overflow exercised");
    check(rnd.snap>0&&rnd.overflow>0,"library reports both failures");
    const study_t *sets[2]={&road,&rnd};
    static const char *const names[2]={"road quads (port camera, +/-900, max_out 10)","random polygons (3..8 corners)"};
    for(unsigned i=0;i<2;++i) {
        const study_t *s=sets[i];
        printf("view-guard-src-1 %s: %llu polygons, %llu clipped and non-empty; %llu output vertices exact on both entry points "
            "(%llu failures: %llu overflow, %llu snap; %llu in place)\n",names[i],(unsigned long long)s->polygons,(unsigned long long)s->clipped,
            (unsigned long long)s->c_vertices,(unsigned long long)s->negative,(unsigned long long)s->overflow,(unsigned long long)s->snap,
            (unsigned long long)s->inplace);
        printf("  (a) view-round-1 clip vs source, same start: identical %llu, same pixels %llu, pixels differ %llu; rotated: "
            "pure rotation %llu, rotation in pixels %llu, rotated and pixels differ %llu; count differs %llu\n",
            (unsigned long long)s->lib_kind[0][0],(unsigned long long)s->lib_kind[0][1],(unsigned long long)s->lib_kind[0][2],
            (unsigned long long)s->lib_kind[1][0],(unsigned long long)s->lib_kind[1][1],(unsigned long long)s->lib_kind[1][2],
            (unsigned long long)s->lib_count);
        printf("  (b) source order + planes, view-guard-1 crossings: count differs %llu; of %llu vertices %llu differ in view space (max %lld units), "
            "%llu in pixels (max %d px); polygons with a view-space difference %llu, with a pixel difference %llu\n",
            (unsigned long long)s->b_count,(unsigned long long)s->b_vertices,(unsigned long long)s->b_vertex_bad,(long long)s->b_max_unit,
            (unsigned long long)s->b_pixel_bad,s->b_max_px,(unsigned long long)s->b_poly_vbad,(unsigned long long)s->b_poly_pbad);
        printf("      source crossings without the snap: count differs %llu; %llu vertices differ (max %lld units), %llu in pixels (max %d px)\n",
            (unsigned long long)s->s_count,(unsigned long long)s->s_vertex_bad,(long long)s->s_max_unit,(unsigned long long)s->s_pixel_bad,s->s_max_px);
        printf("  (c) sh2z_view_clip_guard_src_trunc[_divu] vs source: %llu mismatches\n",(unsigned long long)s->c_bad);
    }
    printf("  oracle branches: fraction den < 2^31 %llu, < 2^59 %llu, >= 2^59 %llu, t = 2^32 %llu; snap moves %llu (z %llu); "
        "empty-interval failures %llu, int32 range failures %llu, overflows %llu\n",
        (unsigned long long)ostats.small,(unsigned long long)ostats.mid,(unsigned long long)ostats.huge,(unsigned long long)ostats.full,
        (unsigned long long)ostats.snapped,(unsigned long long)ostats.zsnapped,(unsigned long long)ostats.snap_fail,
        (unsigned long long)ostats.range_fail,(unsigned long long)ostats.overflow);
    static const struct { unsigned m; const char *name; } controls[]={
        {M_LIB_ORDER,"library emission order"},{M_START,"walk starts at the first corner"},{M_PLANE_ORDER,"planes left, top, right, bottom"},
        {M_LIB_CROSS,"view-guard-1 crossing rule"},{M_NO_SNAP,"no snap"},{M_NO_ZSNAP,"no near/far clamp in the snap"},
        {M_ROUND_T,"fraction rounded to nearest"},{M_FROM_CURRENT,"crossing from the current corner"},{M_NO_LIMIT,"pass limit ignored"},
        {M_NO_FAIL,"snap failures ignored"}};
    for(unsigned c=0;c<sizeof(controls)/sizeof(controls[0]);++c) {
        mutation=controls[c].m;
        study_t m;memset(&m,0,sizeof(m));
        state=UINT32_C(0x0ad0ad01);uint64_t mm=run_road(&m,20000);
        state=UINT32_C(0x7a9d0b1e);mm+=run_random(&m,40000);
        printf("  control (%s): %llu mismatches\n",controls[c].name,(unsigned long long)mm);
        check(mm>0,"negative control detected");
    }
    mutation=0;
    printf("view-guard-src-1 PASS (%llu checks)\n",(unsigned long long)checks);
    return 0;
}
