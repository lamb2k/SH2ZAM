/* Independent oracle, properties, edge cases and negative controls for the
 * view-guard-1 guard-band polygon clip. MIT; SH2ZAM Contributors. */
#include "sh2zam/view_experimental.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

__extension__ typedef __int128 wide_t;

static uint64_t checks;
static uint32_t state=UINT32_C(0x6a0d0001);
static void check(bool value,const char *what) {
    ++checks;
    if(!value) {
        fprintf(stderr,"guard check %llu failed: %s\n",(unsigned long long)checks,what);
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
static wide_t trunc_div(wide_t n,wide_t d) {
    const wide_t a=n<0?-n:n,b=d<0?-d:d,q=a/b;
    return ((n<0)!=(d<0))?-q:q;
}
static wide_t floor_div(wide_t n,wide_t d) {wide_t q=n/d;if(n%d!=0&&((n<0)!=(d<0)))--q;return q;}
static wide_t clampw(wide_t a,wide_t lo,wide_t hi) {return a<lo?lo:a>hi?hi:a;}

static sh2z_view_vertex_t oracle_project(const sh2z_view_t *v,wide_t x,wide_t y,wide_t z) {
    sh2z_view_vertex_t o;
    memset(&o,0,sizeof(o));
    o.x=(int32_t)x;o.y=(int32_t)y;o.z=(int32_t)z;
    if(z<v->near_z){o.outcode=SH2Z_OUT_NEAR;return o;}
    const wide_t qx=clampw(trunc_div((wide_t)v->focal_x*x,z),INT32_MIN,INT32_MAX);
    const wide_t qy=clampw(trunc_div((wide_t)v->focal_y*y,z),INT32_MIN,INT32_MAX);
    o.sx=(int16_t)clampw(v->center_x+floor_div(qx,65536),INT16_MIN,INT16_MAX);
    o.sy=(int16_t)clampw(v->center_y-floor_div(qy,65536),INT16_MIN,INT16_MAX);
    o.outcode=(uint8_t)((z>v->far_z?SH2Z_OUT_FAR:0)|(o.sx<v->clip[0]?SH2Z_OUT_LEFT:0)|(o.sx>v->clip[2]?SH2Z_OUT_RIGHT:0)|
        (o.sy<v->clip[1]?SH2Z_OUT_TOP:0)|(o.sy>v->clip[3]?SH2Z_OUT_BOTTOM:0));
    return o;
}
static sh2z_view_t random_view(void) {
    sh2z_view_t v;
    memset(&v,0,sizeof(v));
    v.r[0][0]=v.r[1][1]=v.r[2][2]=16384;
    /* Sometimes whole-pixel focal lengths, so corners can sit exactly on a side plane. */
    if(random_word()&1u){v.focal_x=65536*random_range(1,512);v.focal_y=65536*random_range(1,512);}
    else{v.focal_x=random_range(1,1024*65536);v.focal_y=random_range(1,1024*65536);}
    v.center_x=(int16_t)random_range(0,352);v.center_y=(int16_t)random_range(0,256);
    v.near_z=random_range(1,1<<20);v.far_z=v.near_z+random_range(1,INT32_MAX-v.near_z);
    v.clip[0]=(int16_t)random_range(-64,160);v.clip[2]=(int16_t)random_range(v.clip[0],400);
    v.clip[1]=(int16_t)random_range(-64,120);v.clip[3]=(int16_t)random_range(v.clip[1],300);
    check(sh2z_view_init(&v,&v),"random view valid");
    return v;
}
static void random_guard(const sh2z_view_t *v,int16_t g[4]) {
    switch(random_word()%4u) {
    case 0: g[0]=-900;g[1]=-900;g[2]=900;g[3]=900;break;              /* the port's band */
    case 1: g[0]=v->clip[0];g[1]=v->clip[1];g[2]=v->clip[2];g[3]=v->clip[3];break;
    case 2: g[0]=(int16_t)(v->center_x-16384);g[2]=(int16_t)(v->center_x+16384);
            g[1]=(int16_t)(v->center_y-16384);g[3]=(int16_t)(v->center_y+16384);break;
    default:
        g[0]=(int16_t)random_range(v->center_x-2000,v->center_x+200);g[2]=(int16_t)random_range(g[0],v->center_x+2000);
        g[1]=(int16_t)random_range(v->center_y-2000,v->center_y+200);g[3]=(int16_t)random_range(g[1],v->center_y+2000);
    }
}

/* ---- oracle -------------------------------------------------------------- */
enum { MUT_NONE=0, MUT_WALK=1, MUT_NO_CLAMP=2, MUT_STRICT=4 };
typedef struct { wide_t c[7]; } opt_t;
static uint64_t bound_checked;
static wide_t oracle_d(const sh2z_view_t *v,const int16_t g[4],unsigned plane,const opt_t *p) {
    const wide_t x=p->c[0],y=p->c[1],z=p->c[2];
    switch(plane) {
    case 0: return z-v->near_z;
    case 1: return v->far_z-z;
    case 2: return (wide_t)v->focal_x*x-(wide_t)65536*(g[0]-v->center_x)*z;
    case 3: return (wide_t)65536*(g[2]-v->center_x)*z-(wide_t)v->focal_x*x;
    case 4: return (wide_t)65536*(v->center_y-g[1])*z-(wide_t)v->focal_y*y;
    default: return (wide_t)v->focal_y*y-(wide_t)65536*(v->center_y-g[3])*z;
    }
}
static opt_t oracle_cross(const opt_t *a,const opt_t *b,wide_t da,wide_t db,unsigned plane,unsigned coords) {
    /* a is the end the crossing is measured from; da >= 0 > db unless mutated. */
    const wide_t num=da<0?-da:da,den=(da<0?-da:da)+(db<0?-db:db);
    opt_t p;memset(&p,0,sizeof(p));
    wide_t t=0;
    if(plane>=2) {
        unsigned s=0;
        while((den>>s)>=((wide_t)1<<31))++s;
        t=((num>>s)<<30)/(den>>s);
    }
    for(unsigned i=0;i<coords;++i) {
        const wide_t delta=b->c[i]-a->c[i],m=delta<0?-delta:delta;
        const wide_t off=plane>=2?(m*t)>>30:m*num/den;
        p.c[i]=a->c[i]+(delta<0?-off:off);
        if(plane>=2) {   /* documented bound against the exact crossing */
            const wide_t exact_num=a->c[i]*den+delta*num;
            const wide_t err=p.c[i]*den-exact_num;
            check((err<0?-err:err)*((wide_t)1<<30)<den*(((wide_t)1<<30)+3*m),"side crossing within 1 + 3m/2^30");
            ++bound_checked;
        }
    }
    return p;
}
static int oracle_clip(sh2z_view_guard_vertex_t *out,const sh2z_view_t *v,const int16_t g[4],
    const sh2z_view_guard_vertex_t *in,unsigned count,unsigned attrs,unsigned mutation) {
    opt_t poly[128],next[128];   /* 8 * 1.5^6 < 128: no cap in the oracle */
    unsigned n=count;
    const unsigned coords=3u+attrs;
    for(unsigned k=0;k<count;++k) {
        memset(&poly[k],0,sizeof(poly[k]));
        poly[k].c[0]=in[k].v.x;poly[k].c[1]=in[k].v.y;poly[k].c[2]=in[k].v.z;
        for(unsigned i=0;i<attrs;++i)poly[k].c[3+i]=in[k].attr[i];
    }
    bool overflow=false;
    for(unsigned plane=0;plane<6u;++plane) {
        unsigned m=0;
        for(unsigned i=0;i<n;++i) {
            const unsigned j=(i+1u)%n;
            const wide_t di=oracle_d(v,g,plane,&poly[i]),dj=oracle_d(v,g,plane,&poly[j]);
            const bool ii=(mutation&MUT_STRICT)?di>0:di>=0,ij=(mutation&MUT_STRICT)?dj>0:dj>=0;
            if(ii)next[m++]=poly[i];
            if(ii!=ij) {
                if((mutation&MUT_WALK))next[m++]=oracle_cross(&poly[i],&poly[j],di,dj,plane,coords);
                else next[m++]=ii?oracle_cross(&poly[i],&poly[j],di,dj,plane,coords):oracle_cross(&poly[j],&poly[i],dj,di,plane,coords);
            }
        }
        if(m>SH2Z_VIEW_GUARD_MAX_OUT)overflow=true;
        n=m;memcpy(poly,next,sizeof(opt_t)*m);
        if(!n)break;
    }
    if(overflow)return SH2Z_VIEW_GUARD_OVERFLOW;
    for(unsigned k=0;k<n;++k) {
        sh2z_view_vertex_t q=oracle_project(v,poly[k].c[0],poly[k].c[1],poly[k].c[2]);
        if(!(mutation&MUT_NO_CLAMP)) {
            q.sx=(int16_t)clampw(q.sx,g[0],g[2]);q.sy=(int16_t)clampw(q.sy,g[1],g[3]);
        }
        q.outcode=(uint8_t)((q.sx<v->clip[0]?SH2Z_OUT_LEFT:0)|(q.sx>v->clip[2]?SH2Z_OUT_RIGHT:0)|
            (q.sy<v->clip[1]?SH2Z_OUT_TOP:0)|(q.sy>v->clip[3]?SH2Z_OUT_BOTTOM:0));
        memset(&out[k],0,sizeof(out[k]));
        out[k].v=q;
        for(unsigned i=0;i<attrs;++i)out[k].attr[i]=(int32_t)poly[k].c[3+i];
    }
    return (int)n;
}

/* ---- polygon generators -------------------------------------------------- */
static sh2z_view_guard_vertex_t corner(int32_t x,int32_t y,int32_t z) {
    sh2z_view_guard_vertex_t c;memset(&c,0,sizeof(c));c.v.x=x;c.v.y=y;c.v.z=z;
    for(unsigned i=0;i<SH2Z_VIEW_GUARD_ATTRS;++i)c.attr[i]=(int32_t)random_word();
    c.v.sx=(int16_t)random_word();c.v.outcode=(uint8_t)random_word();   /* ignored on input */
    return c;
}
static int32_t sat32w(wide_t a){return (int32_t)clampw(a,INT32_MIN,INT32_MAX);}
/* A corner on or near the view volume's boundary, so every plane is crossed. */
static sh2z_view_guard_vertex_t near_volume(const sh2z_view_t *v,const int16_t g[4]) {
    int32_t z;
    switch(random_word()%5u) {
    case 0: z=v->near_z;break;
    case 1: z=v->far_z;break;
    case 2: z=random_range(v->near_z>(1<<22)?v->near_z-(1<<22):1,v->near_z+(1<<22)>v->far_z?v->far_z:v->near_z+(1<<22));break;
    case 3: z=random_coord();break;
    default: z=random_range(v->near_z,v->far_z);
    }
    if(z<1 && (random_word()&1u))z=random_range(1,1<<20);
    /* Screen positions spread around the guard; x, y = (s - c) * z / focal. */
    const int32_t sx=random_range(g[0]-3000,g[2]+3000),sy=random_range(g[1]-3000,g[3]+3000);
    wide_t x=trunc_div((wide_t)(sx-v->center_x)*65536*z,v->focal_x),y=trunc_div((wide_t)(v->center_y-sy)*65536*z,v->focal_y);
    if(random_word()%4u==0) {   /* exactly on a side plane when the focal length allows it */
        const int32_t fx=v->focal_x/65536,fy=v->focal_y/65536;
        if(v->focal_x%65536==0 && fx>0 && z%fx==0)x=(wide_t)(g[random_word()&1u?0:2]-v->center_x)*(z/fx);
        if(v->focal_y%65536==0 && fy>0 && z%fy==0)y=(wide_t)(v->center_y-g[random_word()&1u?1:3])*(z/fy);
    }
    return corner(sat32w(x),sat32w(y),z);
}
/* A planar convex polygon: corners on an ellipse in a random plane. */
static unsigned convex_polygon(sh2z_view_guard_vertex_t *p,const sh2z_view_t *v,const int16_t g[4]) {
    const unsigned n=3u+random_word()%6u;
    const sh2z_view_guard_vertex_t c=near_volume(v,g);
    double u[3],w[3];
    /* Keep every corner well inside int32 so none is clamped (that would bend the plane). */
    const double cc[3]={c.v.x,c.v.y,c.v.z};
    double room=2.0e9;
    for(unsigned i=0;i<3;++i){const double a=cc[i]<0?-cc[i]:cc[i];if(2.0e9-a<room)room=2.0e9-a;}
    double scale=(double)(1u<<(8u+random_word()%20u));
    if(scale>room/1.5)scale=room/1.5;
    if(scale<16.0)return convex_polygon(p,v,g);
    for(unsigned i=0;i<3;++i){u[i]=((double)(random_word()%65536u)/32768.0-1.0)*scale;w[i]=((double)(random_word()%65536u)/32768.0-1.0)*scale;}
    const double start=(double)(random_word()%65536u)/65536.0*6.283185307179586;
    for(unsigned k=0;k<n;++k) {
        const double a=start+6.283185307179586*k/n;
        double q[3];
        const double ca=__builtin_cos(a),sa=__builtin_sin(a);
        for(unsigned i=0;i<3;++i)q[i]=cc[i]+ca*u[i]+sa*w[i];
        p[k]=corner((int32_t)q[0],(int32_t)q[1],(int32_t)q[2]);
    }
    return n;
}
static unsigned random_polygon(sh2z_view_guard_vertex_t *p,const sh2z_view_t *v,const int16_t g[4],bool *convex) {
    *convex=random_word()%2u==0;
    if(*convex)return convex_polygon(p,v,g);
    const unsigned n=3u+random_word()%6u;
    for(unsigned k=0;k<n;++k)p[k]=random_word()%4u?near_volume(v,g):corner(random_coord(),random_coord(),random_coord());
    return n;
}

/* ---- helpers -------------------------------------------------------------- */
static int cmp_corner(const void *a,const void *b) {
    const sh2z_view_guard_vertex_t *x=a,*y=b;
    const int32_t ka[7]={x->v.x,x->v.y,x->v.z,x->attr[0],x->attr[1],x->attr[2],x->attr[3]};
    const int32_t kb[7]={y->v.x,y->v.y,y->v.z,y->attr[0],y->attr[1],y->attr[2],y->attr[3]};
    for(unsigned i=0;i<7;++i)if(ka[i]!=kb[i])return ka[i]<kb[i]?-1:1;
    return 0;
}
static bool same_points(const sh2z_view_guard_vertex_t *a,const sh2z_view_guard_vertex_t *b,int n) {
    sh2z_view_guard_vertex_t x[SH2Z_VIEW_GUARD_MAX_OUT],y[SH2Z_VIEW_GUARD_MAX_OUT];
    memcpy(x,a,sizeof(*a)*(size_t)n);memcpy(y,b,sizeof(*b)*(size_t)n);
    qsort(x,(size_t)n,sizeof(x[0]),cmp_corner);qsort(y,(size_t)n,sizeof(y[0]),cmp_corner);
    for(int k=0;k<n;++k)if(cmp_corner(&x[k],&y[k]))return false;
    return true;
}

static void sweep(void) {
    enum { TRIALS=300000 };
    uint64_t compared=0,overflows=0,convex_overflows=0,clipped=0,empty=0,max_convex_extra=0,near_equal=0;
    uint64_t control_walk=0,control_clamp=0,control_strict=0;
    int32_t max_clamp=0,max_clamp_fine=0;
    uint64_t by_count[SH2Z_VIEW_GUARD_MAX_OUT+1]={0};
    for(unsigned t=0;t<TRIALS;++t) {
        const sh2z_view_t v=random_view();
        int16_t g[4];random_guard(&v,g);
        sh2z_view_guard_vertex_t in[SH2Z_VIEW_GUARD_MAX_IN],rev[SH2Z_VIEW_GUARD_MAX_IN];
        bool convex;
        const unsigned n=random_polygon(in,&v,g,&convex);
        const unsigned attrs=random_word()%5u;
        sh2z_view_guard_vertex_t a[SH2Z_VIEW_GUARD_MAX_OUT],b[SH2Z_VIEW_GUARD_MAX_OUT],w[SH2Z_VIEW_GUARD_MAX_OUT],r[SH2Z_VIEW_GUARD_MAX_OUT];
        memset(a,0x5a,sizeof(a));memset(b,0x5a,sizeof(b));
        const int ra=sh2z_view_clip_guard(a,&v,g,in,n,attrs),rb=sh2z_view_clip_guard_divu(b,&v,g,in,n,attrs);
        const int rw=oracle_clip(w,&v,g,in,n,attrs,MUT_NONE);
        check(ra==rw && rb==rw,"result count equals the oracle");
        if(rw==SH2Z_VIEW_GUARD_OVERFLOW) {
            ++overflows;convex_overflows+=convex;
            sh2z_view_guard_vertex_t sentinel[SH2Z_VIEW_GUARD_MAX_OUT];memset(sentinel,0x5a,sizeof(sentinel));
            check(!memcmp(a,sentinel,sizeof(a)) && !memcmp(b,sentinel,sizeof(b)),"overflow writes nothing");
            continue;
        }
        check(rw==0 || (rw>=3 && rw<=SH2Z_VIEW_GUARD_MAX_OUT),"count is 0 or 3..16");
        ++by_count[rw];
        check(!memcmp(a,w,sizeof(a[0])*(size_t)rw) && !memcmp(b,w,sizeof(b[0])*(size_t)rw),"vertices equal the oracle on both entry points");
        compared+=(uint64_t)rw;
        if(!rw){++empty;continue;}
        if(convex && (uint64_t)rw>n && (uint64_t)rw-n>max_convex_extra)max_convex_extra=(uint64_t)rw-n;
        bool changed=(unsigned)rw!=n;
        for(int k=0;k<rw;++k) {
            const sh2z_view_guard_vertex_t *o=&a[k];
            check(o->v.sx>=g[0] && o->v.sx<=g[2] && o->v.sy>=g[1] && o->v.sy<=g[3],"inside the guard rectangle");
            check(o->v.z>=v.near_z && o->v.z<=v.far_z,"depth between near and far");
            check(!(o->v.outcode&(SH2Z_OUT_NEAR|SH2Z_OUT_FAR)) && !o->v.reserved[0] && !o->v.reserved[1] && !o->v.reserved[2],"outcode and reserved");
            for(unsigned i=attrs;i<SH2Z_VIEW_GUARD_ATTRS;++i)check(o->attr[i]==0,"unused attributes are zero");
            if((unsigned)k>=n || o->v.x!=in[k].v.x || o->v.y!=in[k].v.y || o->v.z!=in[k].v.z)changed=true;
        }
        clipped+=changed;
        /* Reversed winding: the same points. */
        for(unsigned k=0;k<n;++k)rev[k]=in[n-1u-k];
        const int rr=sh2z_view_clip_guard_divu(r,&v,g,rev,n,attrs);
        check(rr==rw && same_points(r,a,rw),"winding-independent crossings");
        /* In place. */
        sh2z_view_guard_vertex_t inplace[SH2Z_VIEW_GUARD_MAX_OUT];
        memcpy(inplace,in,sizeof(in[0])*n);
        check(sh2z_view_clip_guard(inplace,&v,g,inplace,n,attrs)==rw && !memcmp(inplace,a,sizeof(a[0])*(size_t)rw),"in place");
        /* Controls. */
        sh2z_view_guard_vertex_t m[SH2Z_VIEW_GUARD_MAX_OUT];
        const int rm=oracle_clip(m,&v,g,rev,n,attrs,MUT_WALK);
        if(rm!=rw || (rm>0 && !same_points(m,a,rw)))++control_walk;
        if(oracle_clip(m,&v,g,in,n,attrs,MUT_NO_CLAMP)==rw) {
            bool outside=false;
            for(int k=0;k<rw;++k) {
                const int32_t dx=m[k].v.sx<g[0]?g[0]-m[k].v.sx:m[k].v.sx>g[2]?m[k].v.sx-g[2]:0;
                const int32_t dy=m[k].v.sy<g[1]?g[1]-m[k].v.sy:m[k].v.sy>g[3]?m[k].v.sy-g[3]:0;
                const int32_t d=dx>dy?dx:dy;
                if(d)outside=true;
                /* Where one view unit in x, y or z moves the projection by at most
                 * 1/16 pixel anywhere in the guard band: dsx ~ (focal*dx + |sx-cx|*dz)/z. */
                const int64_t gx=g[0]-v.center_x<0?v.center_x-g[0]:g[0]-v.center_x,gx2=g[2]-v.center_x<0?v.center_x-g[2]:g[2]-v.center_x;
                const int64_t gy=g[1]-v.center_y<0?v.center_y-g[1]:g[1]-v.center_y,gy2=g[3]-v.center_y<0?v.center_y-g[3]:g[3]-v.center_y;
                const int64_t ex=v.focal_x+65536*(gx>gx2?gx:gx2),ey=v.focal_y+65536*(gy>gy2?gy:gy2);
                const bool fine=ex*16<=(int64_t)m[k].v.z*65536 && ey*16<=(int64_t)m[k].v.z*65536;
                if(fine && d>max_clamp_fine)max_clamp_fine=d;
                if(d>max_clamp)max_clamp=d;
            }
            control_clamp+=outside;
        }
        const int rs=oracle_clip(m,&v,g,in,n,attrs,MUT_STRICT);
        if(rs!=rw || (rs>0 && memcmp(m,a,sizeof(a[0])*(size_t)rw)))++control_strict;
        /* Near-only agreement: when sh2z_view_clip_near's polygon is inside the
         * other five planes, the guard clip returns exactly its vertices. */
        if(n==4) {
            sh2z_view_vertex_t q[4],cn[6];
            /* clip_near passes in-front corners through, so give it projected ones. */
            for(unsigned k=0;k<4;++k)q[k]=oracle_project(&v,in[k].v.x,in[k].v.y,in[k].v.z);
            const unsigned nn=sh2z_view_clip_near(cn,&v,q);
            bool inside=nn>0;
            for(unsigned k=0;k<nn && inside;++k) {
                opt_t p;memset(&p,0,sizeof(p));p.c[0]=cn[k].x;p.c[1]=cn[k].y;p.c[2]=cn[k].z;
                for(unsigned pl=1;pl<6;++pl)if(oracle_d(&v,g,pl,&p)<0)inside=false;
            }
            if(inside) {
                check((unsigned)rw==nn,"near-only count");
                for(unsigned k=0;k<nn;++k) {
                    if(memcmp(&a[k].v,&cn[k],sizeof(cn[k])))
                        fprintf(stderr,"k%u guard %d %d %d %d %d %d | near %d %d %d %d %d %d | near_z %d far %d g %d %d %d %d\n",k,
                            a[k].v.x,a[k].v.y,a[k].v.z,a[k].v.sx,a[k].v.sy,(int)a[k].v.outcode,cn[k].x,cn[k].y,cn[k].z,cn[k].sx,cn[k].sy,(int)cn[k].outcode,
                            v.near_z,v.far_z,g[0],g[1],g[2],g[3]);
                    check(!memcmp(&a[k].v,&cn[k],sizeof(cn[k])),"near crossings equal sh2z_view_clip_near");
                }
                ++near_equal;
            }
        }
    }
    printf("sweep: %u polygons, %llu output vertices exact on both entry points (%llu changed by clipping, %llu empty)\n",
        TRIALS,(unsigned long long)compared,(unsigned long long)clipped,(unsigned long long)empty);
    printf("  output sizes:");for(unsigned k=0;k<=SH2Z_VIEW_GUARD_MAX_OUT;++k)if(by_count[k])printf(" %u:%llu",k,(unsigned long long)by_count[k]);printf("\n");
    printf("  overflow: %llu polygons (%llu of them planar convex); planar convex max growth +%llu\n",
        (unsigned long long)overflows,(unsigned long long)convex_overflows,(unsigned long long)max_convex_extra);
    printf("  %llu quads equal sh2z_view_clip_near byte for byte; %llu side crossings within the documented bound\n",
        (unsigned long long)near_equal,(unsigned long long)bound_checked);
    printf("control 'crossing from walk order, not the inside end': %llu mismatches\n",(unsigned long long)control_walk);
    printf("control 'no clamp to the guard rectangle': %llu polygons outside it; the clamp moves a vertex at most %d px, %d px where a view unit is <= 1/16 px\n",
        (unsigned long long)control_clamp,(int)max_clamp,(int)max_clamp_fine);
    printf("control 'plane boundary exclusive': %llu mismatches\n",(unsigned long long)control_strict);
    check(clipped>TRIALS/4 && near_equal>1000 && bound_checked>100000,"sweep coverage");
    check(control_walk>0 && control_clamp>0 && control_strict>0,"every control is detected");
    check(max_clamp_fine<=1,"the clamp moves at most one pixel at fine resolution");
    check(convex_overflows==0,"no planar convex polygon overflows");
    check(overflows>0,"overflow path exercised");
}

/* The case from the port: identity view, focal 160, centre (160,112),
 * near 1, a face at depth 160 whose edge runs from screen (-2840,112) to
 * (160,12). VDP1 cannot draw -2840. Guard-band clipping keeps the edge's
 * line: where it crosses x = 0 moves by under a pixel. */
static void port_case(void) {
    sh2z_view_t v;memset(&v,0,sizeof(v));
    v.r[0][0]=v.r[1][1]=v.r[2][2]=16384;v.focal_x=v.focal_y=160*65536;
    v.center_x=160;v.center_y=112;v.near_z=1;v.far_z=1<<30;
    v.clip[0]=0;v.clip[1]=0;v.clip[2]=319;v.clip[3]=223;
    check(sh2z_view_init(&v,&v),"port view");
    const int16_t g[4]={-900,-900,900,900};
    sh2z_view_guard_vertex_t in[4]={corner(-3000,0,160),corner(0,100,160),corner(140,100,160),corner(140,0,160)};
    sh2z_view_guard_vertex_t out[SH2Z_VIEW_GUARD_MAX_OUT];
    const int n=sh2z_view_clip_guard_divu(out,&v,g,in,4,0);
    check(n==5,"port case: one corner cut off");
    /* Original edge (-2840,112)-(160,12) meets x = 0 at y = 112 - 100*2840/3000. */
    const double y_original=112.0-100.0*2840.0/3000.0;
    int found=0;
    for(int k=0;k<n;++k) {
        const sh2z_view_guard_vertex_t *p=&out[k],*q=&out[(k+1)%n];
        if(p->v.sx>=-900 && p->v.sx<-890 && q->v.sx==160 && q->v.sy==12) {   /* rounding is towards the inside end */
            const double y=p->v.sy+(double)(q->v.sy-p->v.sy)*(0.0-p->v.sx)/(double)(q->v.sx-p->v.sx);
            printf("port case: edge meets x=0 at y=%.2f (unclipped %.2f), new corner (%d,%d)\n",y,y_original,p->v.sx,p->v.sy);
            check(y-y_original<1.0 && y_original-y<1.0,"port case: within one pixel");
            ++found;
        }
    }
    check(found==1,"port case: clipped edge present");
}

static void argument_rules(void) {
    const sh2z_view_t v=random_view();
    const int16_t g[4]={-900,-900,900,900};
    sh2z_view_guard_vertex_t in[8],out[SH2Z_VIEW_GUARD_MAX_OUT],sentinel[SH2Z_VIEW_GUARD_MAX_OUT];
    for(int32_t k=0;k<8;++k)in[k]=corner(k,k,v.near_z+1);
    memset(out,0xa5,sizeof(out));memcpy(sentinel,out,sizeof(out));
    check(sh2z_view_clip_guard(out,NULL,g,in,4,0)==SH2Z_VIEW_GUARD_INVALID,"null view");
    check(sh2z_view_clip_guard(NULL,&v,g,in,4,0)==SH2Z_VIEW_GUARD_INVALID,"null out");
    check(sh2z_view_clip_guard_divu(out,&v,NULL,in,4,0)==SH2Z_VIEW_GUARD_INVALID,"null guard");
    check(sh2z_view_clip_guard_divu(out,&v,g,NULL,4,0)==SH2Z_VIEW_GUARD_INVALID,"null in");
    check(sh2z_view_clip_guard(out,&v,g,in,2,0)==SH2Z_VIEW_GUARD_INVALID && sh2z_view_clip_guard(out,&v,g,in,9,0)==SH2Z_VIEW_GUARD_INVALID,"count 3..8");
    check(sh2z_view_clip_guard(out,&v,g,in,4,5)==SH2Z_VIEW_GUARD_INVALID,"attrs <= 4");
    const int16_t flipped[4]={10,0,9,5},far_edge[4]={(int16_t)(v.center_x-16385),0,v.center_x,v.center_y};
    check(sh2z_view_clip_guard(out,&v,flipped,in,4,0)==SH2Z_VIEW_GUARD_INVALID,"left > right");
    check(sh2z_view_clip_guard_divu(out,&v,far_edge,in,4,0)==SH2Z_VIEW_GUARD_INVALID,"edge beyond 16384 of centre");
    sh2z_view_t bad=v;bad.near_z=0;
    check(sh2z_view_clip_guard(out,&bad,g,in,4,0)==SH2Z_VIEW_GUARD_INVALID,"invalid view");
    check(!memcmp(out,sentinel,sizeof(out)),"failed calls write nothing");
    const int16_t edge_ok[4]={(int16_t)(v.center_x-16384),(int16_t)(v.center_y-16384),(int16_t)(v.center_x+16384),(int16_t)(v.center_y+16384)};
    check(sh2z_view_clip_guard(out,&v,edge_ok,in,8,4)>=0,"edges at 16384 accepted, count 8, attrs 4");
}

int main(void) {
    port_case();
    argument_rules();
    sweep();
    printf("guard checks: %llu passed\n",(unsigned long long)checks);
    return EXIT_SUCCESS;
}
