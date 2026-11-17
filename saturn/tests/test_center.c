/* Independent oracle, hand cases, guard-plane property, argument rules, the
 * caller-only experiment and negative controls for view-center-1
 * (final-coordinate truncation with a Q16 centre). The equivalence with
 * view-round-1 at whole-pixel centres is test_center_equiv.c.
 * MIT; SH2ZAM Contributors. */
#include "sh2zam/view_experimental.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

__extension__ typedef __int128 wide_t;

static uint64_t checks;
static uint32_t state=UINT32_C(0x5eed0c16);
static void check(bool value,const char *what) {
    ++checks;
    if(!value) {
        fprintf(stderr,"center check %llu failed: %s\n",(unsigned long long)checks,what);
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
/* Division rounded toward zero and floor division, from magnitudes. */
static wide_t toward_zero(wide_t n,wide_t d) {
    const wide_t a=n<0?-n:n,b=d<0?-d:d,q=a/b;
    return ((n<0)!=(d<0))?-q:q;
}
static wide_t floor_wide(wide_t n,wide_t d) {wide_t q=toward_zero(n,d);if(q*d!=n&&((n<0)!=(d<0)))--q;return q;}
static wide_t clampw(wide_t a,wide_t lo,wide_t hi) {return a<lo?lo:a>hi?hi:a;}
static int32_t fit32(wide_t a) {return (int32_t)clampw(a,INT32_MIN,INT32_MAX);}

/* ---- oracle, from the header text ----------------------------------------------- */
enum { MUT_IGNORE_FRACTION=1, MUT_ROUND_CENTER=2, MUT_Y_SIGN=4, MUT_GUARD_INT=8, MUT_NO_CARRY=16 };
static unsigned mutation;
/* The Q16 centre the (possibly mutated) oracle projects with. */
static wide_t used_center(wide_t c,bool is_y) {
    const wide_t whole=floor_wide(c,65536)*65536;
    if(mutation&MUT_IGNORE_FRACTION)return whole;
    if(mutation&MUT_ROUND_CENTER)return floor_wide(c+32768,65536)*65536;
    if(is_y && (mutation&MUT_Y_SIGN)) {
        /* The residual from the nearest whole pixel, applied with the wrong sign. */
        const wide_t nearest=floor_wide(c+32768,65536)*65536;
        return nearest-(c-nearest);
    }
    return c;
}
/* trunc((c +/- q) / 65536), clamped to int16. */
static int16_t pixel(wide_t c,wide_t q,bool is_y) {
    const wide_t w=is_y?c-q:c+q;
    wide_t s=toward_zero(w,65536);
    if(mutation&MUT_NO_CARRY) {
        /* Floors of the halves added without the carry (x) or borrow (y). */
        const wide_t f=is_y?floor_wide(c,65536)-floor_wide(q,65536):floor_wide(c,65536)+floor_wide(q,65536);
        const wide_t frac=w-floor_wide(w,65536)*65536;
        s=f+((f<0&&frac!=0)?1:0);
    }
    return (int16_t)clampw(s,INT16_MIN,INT16_MAX);
}
static wide_t quotient(int32_t focal,wide_t a,wide_t z) {
    return clampw(toward_zero((wide_t)focal*a,z),-(wide_t)INT32_MAX,INT32_MAX);
}
static uint8_t side_codes(const sh2z_view_t *v,int16_t sx,int16_t sy) {
    return (uint8_t)((sx<v->clip[0]?SH2Z_OUT_LEFT:0u)|(sx>v->clip[2]?SH2Z_OUT_RIGHT:0u)|
        (sy<v->clip[1]?SH2Z_OUT_TOP:0u)|(sy>v->clip[3]?SH2Z_OUT_BOTTOM:0u));
}
static sh2z_view_vertex_t oracle_point(const sh2z_view_center_t *w,wide_t x,wide_t y,wide_t z) {
    sh2z_view_vertex_t o;
    memset(&o,0,sizeof(o));
    check(x>=INT32_MIN&&x<=INT32_MAX&&y>=INT32_MIN&&y<=INT32_MAX&&z>=INT32_MIN&&z<=INT32_MAX,"oracle input fits");
    o.x=(int32_t)x;o.y=(int32_t)y;o.z=(int32_t)z;
    if(z<w->view.near_z){o.outcode=SH2Z_OUT_NEAR;return o;}
    o.sx=pixel(used_center(w->center_x16,false),quotient(w->view.focal_x,x,z),false);
    o.sy=pixel(used_center(w->center_y16,true),quotient(w->view.focal_y,y,z),true);
    o.outcode=(uint8_t)((z>w->view.far_z?SH2Z_OUT_FAR:0u)|side_codes(&w->view,o.sx,o.sy));
    return o;
}
static bool same_vertex(const sh2z_view_vertex_t *a,const sh2z_view_vertex_t *b) {
    return a->x==b->x&&a->y==b->y&&a->z==b->z&&a->sx==b->sx&&a->sy==b->sy&&a->outcode==b->outcode&&
        !a->reserved[0]&&!a->reserved[1]&&!a->reserved[2];
}
/* Near-plane crossing from the inside end i towards o. */
static void near_cross(wide_t p[3],const sh2z_view_vertex_t *i,const sh2z_view_vertex_t *o,int32_t near_z) {
    const wide_t num=(wide_t)i->z-near_z,den=(wide_t)i->z-o->z;
    p[0]=i->x+toward_zero(((wide_t)o->x-i->x)*num,den);
    p[1]=i->y+toward_zero(((wide_t)o->y-i->y)*num,den);
    p[2]=near_z;
}
/* Guard planes with the Q16 centre (header text); MUT_GUARD_INT takes the
 * centre from view.center_x, center_y instead. */
static wide_t plane_d(const sh2z_view_center_t *w,const int16_t g[4],unsigned plane,const wide_t c[3]) {
    const wide_t cx=(mutation&MUT_GUARD_INT)?(wide_t)w->view.center_x*65536:(wide_t)w->center_x16;
    const wide_t cy=(mutation&MUT_GUARD_INT)?(wide_t)w->view.center_y*65536:(wide_t)w->center_y16;
    const wide_t x=c[0],y=c[1],z=c[2],fx=w->view.focal_x,fy=w->view.focal_y;
    switch(plane) {
    case 0: return z-w->view.near_z;
    case 1: return w->view.far_z-z;
    case 2: return fx*x-((wide_t)65536*g[0]-cx)*z;
    case 3: return ((wide_t)65536*g[2]-cx)*z-fx*x;
    case 4: return (cy-(wide_t)65536*g[1])*z-fy*y;
    default: return fy*y-(cy-(wide_t)65536*g[3])*z;
    }
}
typedef struct { wide_t c[7]; } opt_t;
static opt_t guard_cross(const opt_t *i,const opt_t *o,wide_t di,wide_t dout,unsigned plane,unsigned coords) {
    opt_t p;
    memset(&p,0,sizeof(p));
    const wide_t num=di,den=di-dout;
    wide_t t=0;
    if(plane>=2) {
        unsigned s=0;
        while((den>>s)>=((wide_t)1<<31))++s;
        t=((num>>s)<<30)/(den>>s);
    }
    for(unsigned k=0;k<coords;++k) {
        const wide_t delta=o->c[k]-i->c[k],m=delta<0?-delta:delta;
        const wide_t off=plane>=2?(m*t)>>30:m*num/den;
        p.c[k]=i->c[k]+(delta<0?-off:off);
    }
    return p;
}
/* Returns the count, -2 on overflow; out gets the clamped projections. */
static int oracle_guard(sh2z_view_guard_vertex_t *out,const sh2z_view_center_t *w,const int16_t g[4],
    const sh2z_view_guard_vertex_t *in,unsigned n,unsigned attrs,uint64_t *inside_checked) {
    opt_t poly[64],next[64];
    const unsigned coords=3u+attrs;
    for(unsigned k=0;k<n;++k) {
        memset(&poly[k],0,sizeof(poly[k]));
        poly[k].c[0]=in[k].v.x;poly[k].c[1]=in[k].v.y;poly[k].c[2]=in[k].v.z;
        for(unsigned a=0;a<attrs;++a)poly[k].c[3+a]=in[k].attr[a];
    }
    for(unsigned plane=0;plane<6u&&n;++plane) {
        unsigned m=0;
        for(unsigned i=0;i<n;++i) {
            const unsigned j=(i+1)%n;
            const wide_t di=plane_d(w,g,plane,poly[i].c),dj=plane_d(w,g,plane,poly[j].c);
            if(di>=0)next[m++]=poly[i];
            if((di>=0)!=(dj>=0))next[m++]=di>=0?guard_cross(&poly[i],&poly[j],di,dj,plane,coords):guard_cross(&poly[j],&poly[i],dj,di,plane,coords);
            if(m>SH2Z_VIEW_GUARD_MAX_OUT)return -2;
        }
        n=m;
        memcpy(poly,next,sizeof(poly[0])*m);
    }
    for(unsigned k=0;k<n;++k) {
        sh2z_view_vertex_t q=oracle_point(w,poly[k].c[0],poly[k].c[1],poly[k].c[2]);
        /* Inside all six planes (exactly, with the true Q16 centre): the
         * projection already lies inside the rectangle, before the clamp. */
        if(!mutation) {
            bool inside=true;
            for(unsigned plane=0;plane<6u;++plane)inside=inside&&plane_d(w,g,plane,poly[k].c)>=0;
            if(inside){check(q.sx>=g[0]&&q.sx<=g[2]&&q.sy>=g[1]&&q.sy<=g[3],"inside all planes projects inside the rectangle");++*inside_checked;}
        }
        q.sx=(int16_t)clampw(q.sx,g[0],g[2]);q.sy=(int16_t)clampw(q.sy,g[1],g[3]);
        q.outcode=side_codes(&w->view,q.sx,q.sy);
        memset(&out[k],0,sizeof(out[k]));
        out[k].v=q;
        for(unsigned a=0;a<attrs;++a)out[k].attr[a]=(int32_t)poly[k].c[3+a];
    }
    return (int)n;
}

/* ---- cameras, centres and points ------------------------------------------------- */
static int16_t q14(double x) {const double s=floor(x*16384.0+0.5);return (int16_t)(s>32767?32767:s<-32768?-32768:s);}
static int32_t random_center16(void) {
    static const int32_t special[]={INT32_MIN,INT32_MIN+1,INT32_MAX,INT32_MAX-65535,-65537,-65536,-65535,-1,0,1,65535,65536,
        12648447,12648449,10485760,12648448};
    const int32_t k=random_word()%4u?random_range(-64,400):random_range(-32768,32767);
    const wide_t base=(wide_t)k*65536;
    switch(random_word()%9u) {
    case 0: return fit32(base+1);
    case 1: return fit32(base-1);
    case 2: return fit32(base+32768);
    case 3: return fit32(base-32768);
    case 4: return (int32_t)random_word();
    case 5: return special[random_word()%(sizeof(special)/sizeof(special[0]))];
    case 6: return (random_word()&1u)?12648447:12648449;
    case 7: return fit32(base);
    default: return fit32(base+random_range(0,65535));
    }
}
static sh2z_view_center_t random_view(bool rotate) {
    sh2z_view_t v;
    memset(&v,0,sizeof(v));
    v.r[0][0]=v.r[1][1]=v.r[2][2]=16384;
    if(rotate) {
        const double a=(random_word()%65536)*6.283185307179586/65536.0,b=((double)(random_word()%65536)/65536.0-0.5)*1.5;
        const double ca=cos(a),sa=sin(a),cb=cos(b),sb=sin(b);
        const double m[3][3]={{ca,0,-sa},{sa*sb,cb,ca*sb},{sa*cb,-sb,ca*cb}};
        for(unsigned i=0;i<3;++i)for(unsigned j=0;j<3;++j)v.r[i][j]=q14(m[i][j]);
    }
    switch(random_word()%4u) {
    case 0: v.focal_x=v.focal_y=160*65536;break;
    case 1: v.focal_x=v.focal_y=65536*random_range(16,1024);break;
    case 2: v.focal_x=65536*random_range(16,1024);v.focal_y=65536*random_range(16,1024);break;
    default: v.focal_x=random_range(65536,1024*65536);v.focal_y=random_range(65536,1024*65536);
    }
    v.near_z=random_range(1,16*16384);v.far_z=v.near_z+random_range(1,INT32_MAX-v.near_z);
    v.clip[0]=(int16_t)random_range(-64,160);v.clip[2]=(int16_t)random_range(v.clip[0],400);
    v.clip[1]=(int16_t)random_range(-64,120);v.clip[3]=(int16_t)random_range(v.clip[1],300);
    sh2z_view_center_t w;
    check(sh2z_view_center_init(&w,&v,random_center16(),random_center16()),"random centre view valid");
    check(w.view.center_x==(int16_t)floor_wide(w.center_x16,65536)&&w.view.center_y==(int16_t)floor_wide(w.center_y16,65536),
        "init sets the floor of the centre");
    return w;
}
/* Aim at a Q16 screen position: on a row or column boundary, a unit either
 * side of it, or anywhere in the pixel. With z equal to the focal length the
 * quotient is the view coordinate itself, so boundaries are hit exactly. */
static void aimed_point(const sh2z_view_center_t *w,int32_t p[3]) {
    const sh2z_view_t *v=&w->view;
    const bool exact=random_word()%2u==0;
    int32_t z=exact?v->focal_x:random_range(v->near_z,(int64_t)v->near_z+(1<<22)>v->far_z?v->far_z:v->near_z+(1<<22));
    if(!exact&&random_word()%4u==0)z=random_range(v->near_z,v->far_z);
    if(random_word()%8u==0)z=random_range(1,v->near_z);
    const int32_t span=random_word()%4u==0?20000:1200;
    const wide_t target[2]={(wide_t)random_range(-span,span)*65536-w->center_x16,(wide_t)w->center_y16-(wide_t)random_range(-span,span)*65536};
    for(unsigned a=0;a<2;++a) {
        const int32_t f=a?v->focal_y:v->focal_x;
        wide_t c=z==f?target[a]:toward_zero(target[a]*z,f);
        switch(random_word()%5u) {
        case 0: case 1: break;
        case 2: c+=random_range(-1,1);break;
        case 3: c+=random_range(-2,2);break;
        default: c+=random_range(-(z<1000000?z:1000000),z<1000000?z:1000000);
        }
        p[a]=fit32(c);
    }
    p[2]=z;
}

/* ---- hand cases --------------------------------------------------------------------- */
static sh2z_view_vertex_t one(const sh2z_view_center_t *w,int32_t x,int32_t y,int32_t z,bool divu) {
    sh2z_view_vertex_t r={x,y,z,77,77,0xEE,{1,2,3}};
    check(divu?sh2z_view_project_center_divu(&r,w,&r,1):sh2z_view_project_center(&r,w,&r,1),"one-point projection");
    const sh2z_view_vertex_t o=oracle_point(w,x,y,z);
    check(same_vertex(&r,&o),"hand case equals the oracle");
    return r;
}
static int16_t trunc_sy(const sh2z_view_t *v,int32_t x,int32_t y,int32_t z) {
    sh2z_view_vertex_t r={x,y,z,0,0,0,{0,0,0}};
    check(sh2z_view_project_trunc(&r,v,&r,1),"trunc projection");
    return r.sy;
}
static void hand_cases(void) {
    sh2z_view_t v;
    memset(&v,0,sizeof(v));
    v.r[0][0]=v.r[1][1]=v.r[2][2]=16384;
    v.focal_x=v.focal_y=200*65536;v.center_x=160;v.center_y=193;
    v.near_z=16*16384;v.far_z=10000*16384;
    v.clip[0]=0;v.clip[1]=16;v.clip[2]=319;v.clip[3]=223;
    check(sh2z_view_init(&v,&v),"hand view valid");
    /* z equal to the focal length makes the quotient the coordinate itself:
     * the Q16 position is exactly center_x16 + x and center_y16 - y. */
    const int32_t z=v.focal_x,P=65536;
    sh2z_view_center_t lo,hi,half;
    check(sh2z_view_center_init(&lo,&v,10485760,12648447)&&sh2z_view_center_init(&hi,&v,10485760,12648449),"source centres");
    check(lo.view.center_y==192&&hi.view.center_y==193&&lo.view.center_x==160,"floors 192 and 193");
    check(sh2z_view_center_init(&half,&v,10485760+32768,-32768),"half-pixel centre");
    for(int divu=0;divu<2;++divu) {
        /* 193 - 1/65536: y = 0 sits a unit below row 193, so 192 (an integer
         * centre of 193 gives 193); y = -1 is exactly row 193. */
        check(one(&lo,0,0,z,divu).sy==192&&trunc_sy(&v,0,0,z)==193,"193-u, y=0: 192 (integer 193)");
        check(one(&lo,0,-1,z,divu).sy==193&&trunc_sy(&v,0,-1,z)==193,"193-u, y=-1: exactly 193");
        check(one(&lo,0,P-1,z,divu).sy==192&&one(&lo,0,P,z,divu).sy==191&&trunc_sy(&v,0,P,z)==192,"193-u, rows 192 and 191");
        /* Above the top of the screen the position is negative: truncation
         * rounds up. -7 - 1/65536 truncates to -7; -7 exactly is -7. */
        check(one(&lo,0,200*P,z,divu).sy==-7&&one(&lo,0,200*P-1,z,divu).sy==-7&&trunc_sy(&v,0,200*P-1,z)==-6,"193-u, row -7");
        /* 193 + 1/65536: y = 1 is exactly row 193 (integer centre: 192). */
        check(one(&hi,0,1,z,divu).sy==193&&trunc_sy(&v,0,1,z)==192,"193+u, y=1: exactly 193 (integer 192)");
        check(one(&hi,0,0,z,divu).sy==193&&one(&hi,0,P+1,z,divu).sy==192&&one(&hi,0,P+2,z,divu).sy==191,"193+u, rows");
        check(one(&hi,0,200*P,z,divu).sy==-6&&trunc_sy(&v,0,200*P,z)==-7&&one(&hi,0,200*P+1,z,divu).sy==-7,"193+u, -7 + u is -6");
        /* Outcode at the top clip row 16: position 16 - u is row 15 (TOP). */
        check(one(&lo,0,177*P,z,divu).outcode==SH2Z_OUT_TOP&&one(&lo,0,177*P-1,z,divu).outcode==0,"top clip edge from the residual");
        check(one(&hi,0,177*P+1,z,divu).outcode==0&&one(&hi,0,177*P+2,z,divu).outcode==SH2Z_OUT_TOP,"top clip edge, +u");
        /* x: 160.5 + q. */
        check(one(&half,-32768,0,z,divu).sx==160&&one(&half,-32769,0,z,divu).sx==159&&one(&half,32767,0,z,divu).sx==160&&
            one(&half,32768,0,z,divu).sx==161,"half-pixel x centre");
        check(one(&half,-160*P-32768,0,z,divu).sx==0&&one(&half,-160*P-32769,0,z,divu).sx==0&&
            one(&half,-161*P-32768,0,z,divu).sx==-1&&one(&half,-161*P-32767,0,z,divu).sx==0,"half-pixel x, either side of 0");
        /* Centre -1/2 in y: y = 0 truncates -0.5 to 0, y = 32768 is -1 exactly. */
        check(one(&half,0,0,z,divu).sy==0&&one(&half,0,32767,z,divu).sy==0&&one(&half,0,32768,z,divu).sy==-1&&
            one(&half,0,-32768,z,divu).sy==0&&one(&half,0,-32769,z,divu).sy==0&&one(&half,0,-98304,z,divu).sy==1,"half-pixel y centre");
    }
    /* Extreme centres: the 33-bit sums must not wrap. */
    sh2z_view_t u=v;u.focal_x=u.focal_y=65536;u.near_z=1;
    check(sh2z_view_init(&u,&u),"unit view");
    sh2z_view_center_t e;
    for(int divu=0;divu<2;++divu) {
        check(sh2z_view_center_init(&e,&u,INT32_MAX,INT32_MIN),"extreme centre");
        /* INT32_MAX + (2^31 - 1) = 2^32 - 2: 65535.99997 -> 32767 saturated. */
        check(one(&e,INT32_MAX,INT32_MAX,1,divu).sx==32767&&one(&e,INT32_MAX,INT32_MAX,1,divu).sy==-32768,"extremes saturate");
        /* INT32_MAX - (2^31 - 1) = 0 and INT32_MIN + (2^31 - 1) = -1 -> 0. */
        check(one(&e,INT32_MIN,INT32_MIN,1,divu).sx==0&&one(&e,-INT32_MAX,-INT32_MAX,1,divu).sx==0&&
            one(&e,-INT32_MAX,-INT32_MAX,1,divu).sy==0,"extremes cancel");
        check(sh2z_view_center_init(&e,&u,INT32_MIN,INT32_MAX),"extreme centre swapped");
        check(one(&e,0,0,1,divu).sx==-32768&&one(&e,0,0,1,divu).sy==32767&&one(&e,INT32_MAX,0,1,divu).sx==0&&
            one(&e,0,INT32_MAX,1,divu).sy==0,"extremes, other signs");
    }
}

/* ---- path sweeps against the oracle ----------------------------------------------------- */
/* The guard rule: |65536 * edge - centre16| <= 2^30. */
static bool edge_near(int16_t edge,int32_t center16) {
    const wide_t d=(wide_t)65536*edge-center16;
    return d>=-((wide_t)1<<30)&&d<=((wide_t)1<<30);
}
typedef struct { uint64_t batch,project,steps,clipped,guard,guard_polygons,overflows,inside_checked,boundary,mismatches; } sweep_t;
static uint64_t compare_points(const sh2z_view_vertex_t *got,const sh2z_view_center_t *w,wide_t (*want)[3],size_t n) {
    uint64_t bad=0;
    for(size_t k=0;k<n;++k){const sh2z_view_vertex_t o=oracle_point(w,want[k][0],want[k][1],want[k][2]);bad+=!same_vertex(&got[k],&o);}
    return bad;
}
static void sweep(sweep_t *s,unsigned views) {
    static wide_t want[1024][3];
    for(unsigned t=0;t<views;++t) {
        sh2z_view_center_t w=random_view(true);
        const sh2z_vec3_t eye={random_range(-20000,20000)*65536+random_range(0,65535),random_range(-20000,20000)*65536,
            random_range(-20000,20000)*65536+random_range(0,65535)};
        check(sh2z_view_place(&w.view,eye),"place");
        /* batch, both forms, stride 6 and 10 */
        static sh2z_pos16_t pos[1024];static uint16_t wide_pos[1024][5];static sh2z_view_vertex_t a[1024],b[1024];
        for(unsigned k=0;k<1024;++k) {
            const int32_t s16=random_word()%2u?64:4096;
            pos[k].x=(int16_t)clampw(eye.x/65536+random_range(-s16,s16),INT16_MIN,INT16_MAX);
            pos[k].y=(int16_t)clampw(eye.y/65536+random_range(-s16,s16),INT16_MIN,INT16_MAX);
            pos[k].z=(int16_t)clampw(eye.z/65536+random_range(-s16,s16),INT16_MIN,INT16_MAX);
            if(random_word()%8u==0){pos[k].x=(int16_t)random_word();pos[k].y=(int16_t)random_word();pos[k].z=(int16_t)random_word();}
            wide_pos[k][0]=(uint16_t)pos[k].x;wide_pos[k][1]=(uint16_t)pos[k].y;wide_pos[k][2]=(uint16_t)pos[k].z;
            wide_pos[k][3]=wide_pos[k][4]=0xBEEF;
            const int32_t pp[3]={pos[k].x,pos[k].y,pos[k].z};
            for(unsigned i=0;i<3;++i)want[k][i]=(wide_t)w.view.t[i]+(wide_t)w.view.r[i][0]*pp[0]+(wide_t)w.view.r[i][1]*pp[1]+(wide_t)w.view.r[i][2]*pp[2];
        }
        check(sh2z_view_batch_center(a,&w,pos,6,1024)&&sh2z_view_batch_center_divu(b,&w,wide_pos,10,1024),"batches");
        check(!memcmp(a,b,sizeof(a)),"batch divu equals portable");
        s->mismatches+=compare_points(a,&w,want,1024);s->batch+=1024;
        /* project, both forms, in place */
        for(unsigned k=0;k<512;++k) {
            int32_t p[3];aimed_point(&w,p);
            if(random_word()%8u==0){p[0]=random_coord();p[1]=random_coord();p[2]=random_coord();}
            a[k]=(sh2z_view_vertex_t){p[0],p[1],p[2],(int16_t)random_word(),(int16_t)random_word(),(uint8_t)random_word(),{9,9,9}};b[k]=a[k];
            for(unsigned i=0;i<3;++i)want[k][i]=p[i];
            if(p[2]>=w.view.near_z) {
                const wide_t W=(wide_t)w.center_y16-quotient(w.view.focal_y,p[1],p[2]);
                s->boundary+=(W%65536)==0;
            }
        }
        check(sh2z_view_project_center(a,&w,a,512)&&sh2z_view_project_center_divu(b,&w,b,512),"projects");
        check(!memcmp(a,b,512*sizeof(a[0])),"project divu equals portable");
        s->mismatches+=compare_points(a,&w,want,512);s->project+=512;
        /* edge steps: oracle points from the header formula */
        for(unsigned e=0;e<8;++e) {
            int32_t p[3],q[3];aimed_point(&w,p);aimed_point(&w,q);
            if(e%4u==3){for(unsigned i=0;i<3;++i){p[i]=random_coord();q[i]=random_coord();}}
            const sh2z_view_vertex_t f={p[0],p[1],p[2],0,0,0,{0,0,0}},g={q[0],q[1],q[2],0,0,0,{0,0,0}};
            const uint16_t den=(uint16_t)random_range(1,65535);
            const size_t count=(size_t)random_range(1,64);
            const uint16_t first=(uint16_t)random_range(0,den/4);
            const uint16_t step=count==1?(uint16_t)random_range(0,65535):(uint16_t)random_range(0,(int32_t)((den-first)/(count-1)));
            sh2z_view_vertex_t x[64],y[64];
            check(sh2z_view_edge_steps_center(x,&w,&f,&g,first,step,den,count)&&sh2z_view_edge_steps_center_divu(y,&w,&f,&g,first,step,den,count),"edge steps");
            check(!memcmp(x,y,count*sizeof(x[0])),"edge divu equals portable");
            for(size_t k=0;k<count;++k)for(unsigned i=0;i<3;++i) {
                const wide_t d=(wide_t)q[i]-p[i];
                want[k][i]=p[i]+toward_zero(toward_zero(d*first*65536,den)+(wide_t)k*toward_zero(d*step*65536,den),65536);
            }
            s->mismatches+=compare_points(x,&w,want,count);s->steps+=count;
        }
        /* near clip: corners from the _center projection, crossings by the oracle */
        for(unsigned e=0;e<16;++e) {
            sh2z_view_vertex_t in[4],x[6],y[6],r[6];
            for(unsigned i=0;i<4;++i){int32_t p[3];aimed_point(&w,p);if((e>>(i&3u))&1u)p[2]=random_range(w.view.near_z>INT32_MIN/2?-w.view.near_z*2:INT32_MIN,w.view.near_z-1);
                else if(p[2]<w.view.near_z)p[2]=w.view.near_z;
                in[i]=(sh2z_view_vertex_t){p[0],p[1],p[2],0,0,0,{0,0,0}};}
            check(sh2z_view_project_center(in,&w,in,4),"clip corners");
            unsigned n=0;
            for(unsigned i=0;i<4;++i) {
                const sh2z_view_vertex_t *cur=&in[i],*nx=&in[(i+1)&3u];
                const bool ci=cur->z>=w.view.near_z,ni=nx->z>=w.view.near_z;
                if(ci){const sh2z_view_vertex_t o=oracle_point(&w,cur->x,cur->y,cur->z);s->mismatches+=!same_vertex(cur,&o);r[n++]=*cur;}
                if(ci!=ni){wide_t p[3];near_cross(p,ci?cur:nx,ci?nx:cur,w.view.near_z);r[n++]=oracle_point(&w,p[0],p[1],p[2]);}
            }
            const unsigned np=sh2z_view_clip_near_center(x,&w,in),nd=sh2z_view_clip_near_center_divu(y,&w,in);
            check(np==nd,"near clip counts");
            if(np!=n){++s->mismatches;continue;}
            for(unsigned i=0;i<n;++i)s->mismatches+=!same_vertex(&x[i],&r[i]);
            check(!memcmp(x,y,n*sizeof(x[0])),"near clip divu equals portable");
            s->clipped+=n;
            sh2z_view_vertex_t io[6];memcpy(io,in,sizeof(in));
            check(sh2z_view_clip_near_center(io,&w,io)==n&&!memcmp(io,x,n*sizeof(x[0])),"near clip in place");
        }
        /* guard clip: full oracle with the Q16 planes */
        for(unsigned e=0;e<16;++e) {
            int16_t g[4];
            const int32_t cx=w.view.center_x,cy=w.view.center_y;
            switch(random_word()%3u) {
            case 0: g[0]=-900;g[1]=-900;g[2]=900;g[3]=900;break;
            case 1: g[0]=w.view.clip[0];g[1]=w.view.clip[1];g[2]=w.view.clip[2];g[3]=w.view.clip[3];break;
            default: g[0]=(int16_t)clampw(random_range(cx-2000,cx+200),INT16_MIN,INT16_MAX);g[2]=(int16_t)clampw(random_range(g[0],cx+2000),g[0],INT16_MAX);
                g[1]=(int16_t)clampw(random_range(cy-2000,cy+200),INT16_MIN,INT16_MAX);g[3]=(int16_t)clampw(random_range(g[1],cy+2000),g[1],INT16_MAX);
            }
            const bool ok=edge_near(g[0],w.center_x16)&&edge_near(g[2],w.center_x16)&&
                edge_near(g[1],w.center_y16)&&edge_near(g[3],w.center_y16);
            sh2z_view_guard_vertex_t in[8],x[16],y[16],r[16];
            const unsigned n=3u+random_word()%6u,attrs=random_word()%5u;
            memset(in,0,sizeof(in));
            for(unsigned i=0;i<n;++i){int32_t p[3];aimed_point(&w,p);if(random_word()%6u==0)p[2]=random_range(-w.view.near_z,w.view.near_z);
                in[i].v.x=p[0];in[i].v.y=p[1];in[i].v.z=p[2];for(unsigned j=0;j<4;++j)in[i].attr[j]=(int32_t)random_word();}
            const int rx=sh2z_view_clip_guard_center(x,&w,g,in,n,attrs),ry=sh2z_view_clip_guard_center_divu(y,&w,g,in,n,attrs);
            check(rx==ry,"guard counts portable and divu");
            if(!ok){check(rx==SH2Z_VIEW_GUARD_INVALID,"guard edge too far from the Q16 centre");continue;}
            const int want_n=oracle_guard(r,&w,g,in,n,attrs,&s->inside_checked);
            ++s->guard_polygons;
            if(rx!=want_n){++s->mismatches;continue;}
            if(rx<0){++s->overflows;continue;}
            check(!memcmp(x,y,(size_t)rx*sizeof(x[0])),"guard divu equals portable");
            for(int i=0;i<rx;++i)s->mismatches+=!(same_vertex(&x[i].v,&r[i].v)&&!memcmp(x[i].attr,r[i].attr,sizeof(r[i].attr)));
            s->guard+=(uint64_t)rx;
            sh2z_view_guard_vertex_t io[16];memcpy(io,in,sizeof(in));
            check(sh2z_view_clip_guard_center(io,&w,g,io,n,attrs)==rx&&!memcmp(io,x,(size_t)rx*sizeof(x[0])),"guard in place");
        }
    }
}

/* ---- argument rules -------------------------------------------------------------------------- */
static void argument_rules(void) {
    sh2z_view_center_t w=random_view(false),bad=w,stale=w;
    bad.view.focal_x=0;
    stale.view.center_y=(int16_t)(stale.view.center_y==INT16_MAX?INT16_MIN:stale.view.center_y+1);
    sh2z_pos16_t pos[4]={{1,2,3},{4,5,6},{7,8,9},{1,1,1}};
    sh2z_view_vertex_t out[8],pattern[8];
    memset(pattern,0x5A,sizeof(pattern));
    const sh2z_view_center_t *views[3]={&bad,&stale,NULL};
    const sh2z_view_vertex_t f={0,0,100000,0,0,0,{0,0,0}},g={1000,1000,200000,0,0,0,{0,0,0}};
    sh2z_view_guard_vertex_t gin[9],gout[16],gpat[16];memset(gin,0,sizeof(gin));memset(gpat,0x5A,sizeof(gpat));
    for(unsigned k=0;k<4;++k){gin[k].v.z=200000;gin[k].v.x=(int32_t)(k*1000);}
    const int16_t rect[4]={-900,-900,900,900};
    for(unsigned i=0;i<3;++i) {
        const sh2z_view_center_t *v=views[i];
        memcpy(out,pattern,sizeof(out));
        check(!sh2z_view_batch_center(out,v,pos,6,3)&&!sh2z_view_batch_center_divu(out,v,pos,6,3)&&
            !sh2z_view_project_center(out,v,out,4)&&!sh2z_view_project_center_divu(out,v,out,4)&&
            !sh2z_view_edge_steps_center(out,v,&f,&g,0,1,4,2)&&!sh2z_view_edge_steps_center_divu(out,v,&f,&g,0,1,4,2)&&
            sh2z_view_clip_near_center(out,v,out)==0&&sh2z_view_clip_near_center_divu(out,v,out)==0&&
            !memcmp(out,pattern,sizeof(out)),"invalid, stale or null centre view: false, nothing written");
        memcpy(gout,gpat,sizeof(gout));
        check(sh2z_view_clip_guard_center(gout,v,rect,gin,4,0)==SH2Z_VIEW_GUARD_INVALID&&
            sh2z_view_clip_guard_center_divu(gout,v,rect,gin,4,0)==SH2Z_VIEW_GUARD_INVALID&&!memcmp(gout,gpat,sizeof(gout)),"guard rejects it");
    }
    check(sh2z_view_batch_center(NULL,&w,pos,6,0)&&!sh2z_view_batch_center(NULL,&w,pos,6,1)&&!sh2z_view_batch_center(out,&w,pos,5,1)&&
        !sh2z_view_batch_center(out,&w,(const char *)pos+1,6,1),"batch rules");
    check(!sh2z_view_batch_center((sh2z_view_vertex_t *)(void *)pos,&w,pos,6,1),"overlap rejected");
    memcpy(out,pattern,sizeof(out));
    check(!sh2z_view_project_center(out+1,&w,out,4)&&!sh2z_view_project_center_divu(out+1,&w,out,4)&&!memcmp(out,pattern,sizeof(out)),"partial overlap");
    check(!sh2z_view_edge_steps_center(out,&w,&f,&g,5,1,4,1)&&!sh2z_view_edge_steps_center(out,&w,&f,&g,0,2,4,4)&&
        sh2z_view_edge_steps_center(out,&w,&f,&g,0,2,4,3),"edge bound rules");
    /* init: aliasing allowed; failures leave out unchanged */
    sh2z_view_center_t c=w,before;
    check(sh2z_view_center_init(&c,&c.view,-1,-65537)&&c.view.center_x==-1&&c.view.center_y==-2&&c.center_x16==-1&&c.center_y16==-65537,"init aliasing and floors");
    before=c;
    check(!sh2z_view_center_init(&c,&bad.view,5,5)&&!memcmp(&c,&before,sizeof(c))&&!sh2z_view_center_init(NULL,&w.view,0,0)&&
        !sh2z_view_center_init(&c,NULL,0,0),"init failures");
    /* Guard edges within exactly 2^30 of the Q16 centre are allowed. */
    sh2z_view_t u=w.view;u.focal_x=u.focal_y=65536*200;u.near_z=100;u.far_z=1<<30;
    check(sh2z_view_init(&u,&u),"guard view");
    const int16_t edge_l[4]={-16384,-10,10,10},edge_r[4]={-10,-10,16384,10};
    for(int divu=0;divu<2;++divu) {
        int (*clip)(sh2z_view_guard_vertex_t *,const sh2z_view_center_t *,const int16_t *,const sh2z_view_guard_vertex_t *,unsigned,unsigned)=
            divu?sh2z_view_clip_guard_center_divu:sh2z_view_clip_guard_center;
        check(sh2z_view_center_init(&c,&u,0,0)&&clip(gout,&c,edge_l,gin,4,0)>=0&&clip(gout,&c,edge_r,gin,4,0)>=0,"edges at 2^30");
        check(sh2z_view_center_init(&c,&u,1,0)&&clip(gout,&c,edge_l,gin,4,0)==SH2Z_VIEW_GUARD_INVALID&&clip(gout,&c,edge_r,gin,4,0)>=0,"left 2^30 + 1");
        check(sh2z_view_center_init(&c,&u,-1,0)&&clip(gout,&c,edge_r,gin,4,0)==SH2Z_VIEW_GUARD_INVALID&&clip(gout,&c,edge_l,gin,4,0)>=0,"right 2^30 + 1");
        const int16_t edge_t[4]={-10,-16384,10,10},edge_b[4]={-10,-10,10,16384};
        check(sh2z_view_center_init(&c,&u,0,1)&&clip(gout,&c,edge_t,gin,4,0)==SH2Z_VIEW_GUARD_INVALID&&clip(gout,&c,edge_b,gin,4,0)>=0,"top 2^30 + 1");
        check(sh2z_view_center_init(&c,&u,0,-1)&&clip(gout,&c,edge_b,gin,4,0)==SH2Z_VIEW_GUARD_INVALID&&clip(gout,&c,edge_t,gin,4,0)>=0,"bottom 2^30 + 1");
        const int16_t flipped[4]={10,-10,-10,10};
        check(clip(gout,&c,flipped,gin,4,0)==SH2Z_VIEW_GUARD_INVALID&&clip(gout,&c,rect,gin,2,0)==SH2Z_VIEW_GUARD_INVALID&&
            clip(gout,&c,rect,gin,9,0)==SH2Z_VIEW_GUARD_INVALID&&clip(gout,&c,rect,gin,4,5)==SH2Z_VIEW_GUARD_INVALID,"guard rules");
    }
}

/* ---- can a caller do it alone? ---------------------------------------------------------------- */
/* The source: sy = trunc((Cy16 - qy) / 65536) with Cy16 = 193 * 65536 -/+ 1.
 * Single-call caller tactics with whole-pixel centres:
 *   A  _trunc, centre rounded (193)          B  _trunc, centre floored
 *   C  view-contract-1, centre floored       D  view-contract-1, centre floored - 1
 * counted on (1) road-like views and (2) points aimed at row boundaries. */
typedef struct { uint64_t vertices,miss[4],ambiguous,onscreen,onscreen_miss; } caller_t;
static void caller_tally(caller_t *c,const sh2z_view_center_t *w,const sh2z_view_vertex_t *in,size_t n) {
    static sh2z_view_vertex_t truth[4096],got[4][4096];
    sh2z_view_t a=w->view,b=w->view,d=w->view;
    a.center_y=(int16_t)floor_wide((wide_t)w->center_y16+32768,65536);
    d.center_y=(int16_t)(b.center_y-1);
    check(sh2z_view_project_center(truth,w,in,n)&&sh2z_view_project_trunc(got[0],&a,in,n)&&sh2z_view_project_trunc(got[1],&b,in,n)&&
        sh2z_view_project(got[2],&b,in,n)&&sh2z_view_project(got[3],&d,in,n),"caller projections");
    /* Library output rows that the truth splits: no function of tactic A's
     * pixels can be exact when one of its rows maps to two true rows. */
    static uint8_t seen[65536];
    static int16_t first_truth[65536];
    memset(seen,0,sizeof(seen));
    for(size_t k=0;k<n;++k) {
        if(truth[k].outcode&SH2Z_OUT_NEAR)continue;
        ++c->vertices;
        for(unsigned t=0;t<4;++t)c->miss[t]+=got[t][k].sy!=truth[k].sy;
        if(!(truth[k].outcode&(SH2Z_OUT_LEFT|SH2Z_OUT_RIGHT|SH2Z_OUT_TOP|SH2Z_OUT_BOTTOM))){++c->onscreen;c->onscreen_miss+=got[0][k].sy!=truth[k].sy;}
        const unsigned key=(uint16_t)got[0][k].sy;
        if(!seen[key]){seen[key]=1;first_truth[key]=truth[k].sy;}
        else if(seen[key]==1&&first_truth[key]!=truth[k].sy){seen[key]=2;++c->ambiguous;}
    }
}
static void caller_only(void) {
    caller_t road[2],edge[2];
    memset(road,0,sizeof(road));memset(edge,0,sizeof(edge));
    static const int32_t residual[2]={12648447,12648449};
    static sh2z_view_vertex_t in[4096];
    for(unsigned r=0;r<2;++r)for(unsigned t=0;t<400;++t) {
        /* A road-like camera: a few model units above a flat road (below it
         * on alternate views, for a source whose view y points down), slight
         * pitch and yaw, focal 160..320 px (sometimes fractional), centre
         * x 160 and y 193 -/+ 1/65536. */
        sh2z_view_t v;
        memset(&v,0,sizeof(v));
        const double yaw=((double)(random_word()%65536)/65536.0-0.5)*0.6,pitch=((double)(random_word()%65536)/65536.0)*0.25;
        const double cy=cos(yaw),sy=sin(yaw),cp=cos(pitch),sp=sin(pitch);
        const double m[3][3]={{cy,0,-sy},{sy*sp,cp,cy*sp},{sy*cp,-sp,cy*cp}};
        for(unsigned i=0;i<3;++i)for(unsigned j=0;j<3;++j)v.r[i][j]=q14(m[i][j]);
        v.focal_x=v.focal_y=t%2u?random_range(160,320)*65536:random_range(160*65536,320*65536);
        v.near_z=4096;v.far_z=4000*16384;
        v.clip[0]=0;v.clip[1]=0;v.clip[2]=319;v.clip[3]=223;
        check(sh2z_view_init(&v,&v),"road view");
        const int32_t ex=random_range(-20000,20000),ez=random_range(-20000,20000),height=(t%4u<2?1:-1)*random_range(4,40);
        const sh2z_vec3_t eye={ex*65536,height*65536,ez*65536};
        check(sh2z_view_place(&v,eye),"road place");
        sh2z_view_center_t w;
        check(sh2z_view_center_init(&w,&v,10485760,residual[r]),"road centre");
        /* Road grid: rails across x, rows ahead in z, on the road plane y = 0
         * (and a kerb row one unit up), in model units. */
        static sh2z_pos16_t grid[4096];
        for(unsigned k=0;k<4096;++k) {
            const int32_t across=(int32_t)(k%16u)*8-60,ahead=(int32_t)(k/16u)*12+2;
            const double fx=ex+cy*across-sy*ahead,fz=ez+sy*across+cy*ahead;
            grid[k].x=(int16_t)clampw((wide_t)floor(fx+0.5),INT16_MIN,INT16_MAX);
            grid[k].y=(int16_t)((k%7u)==0?1:0);
            grid[k].z=(int16_t)clampw((wide_t)floor(fz+0.5),INT16_MIN,INT16_MAX);
        }
        check(sh2z_view_batch_center(in,&w,grid,6,4096),"road batch");
        caller_tally(&road[r],&w,in,4096);
        /* The same view, points aimed at row boundaries (z = focal: exact). */
        for(unsigned k=0;k<1024;++k) {
            const int32_t row=random_range(-200,420),d=random_range(-1,1);
            in[k]=(sh2z_view_vertex_t){random_range(-2000000,2000000),fit32((wide_t)w.center_y16-(wide_t)row*65536+d),v.focal_x,0,0,0,{0,0,0}};
        }
        caller_tally(&edge[r],&w,in,1024);
    }
    for(unsigned r=0;r<2;++r) {
        printf("  caller-only, centre y %ld (193 %s 1/65536):\n",(long)residual[r],r?"+":"-");
        const caller_t *set[2]={&road[r],&edge[r]};
        static const char *const names[2]={"road-like","row-boundary"};
        for(unsigned s=0;s<2;++s)
            printf("    %s, %llu vertices: misses A (_trunc, centre 193) %llu, B (_trunc, floor) %llu, C (view-contract-1, floor) %llu, "
                "D (view-contract-1, floor - 1) %llu; rows of A that map to two true rows %llu; on screen %llu, A misses %llu\n",
                names[s],(unsigned long long)set[s]->vertices,(unsigned long long)set[s]->miss[0],(unsigned long long)set[s]->miss[1],
                (unsigned long long)set[s]->miss[2],(unsigned long long)set[s]->miss[3],(unsigned long long)set[s]->ambiguous,
                (unsigned long long)set[s]->onscreen,(unsigned long long)set[s]->onscreen_miss);
        /* No single-call tactic is exact on boundary points, and tactic A's
         * output is ambiguous, so only redoing the y division would be. */
        for(unsigned t=0;t<4;++t)check(edge[r].miss[t]>0,"every caller tactic misses on row boundaries");
        check(edge[r].ambiguous>0,"centre-193 rows are ambiguous");
    }
}

int main(void) {
    hand_cases();
    argument_rules();
    sweep_t s;memset(&s,0,sizeof(s));
    sweep(&s,600);
    check(s.mismatches==0,"every path equals the oracle");
    check(s.inside_checked>0&&s.boundary>0,"guard property and boundary hits exercised");
    printf("view-center-1: %llu batch, %llu projected (%llu on an exact row boundary), %llu stepped, %llu near-clip, "
        "%llu guard vertices (%llu polygons, %llu overflows) equal the oracle on both entry points\n",
        (unsigned long long)s.batch,(unsigned long long)s.project,(unsigned long long)s.boundary,(unsigned long long)s.steps,
        (unsigned long long)s.clipped,(unsigned long long)s.guard,(unsigned long long)s.guard_polygons,(unsigned long long)s.overflows);
    printf("  guard vertices inside all six Q16 planes, inside the rectangle unclamped: %llu\n",(unsigned long long)s.inside_checked);
    caller_only();
    static const struct { unsigned m; const char *name; } controls[]={{MUT_IGNORE_FRACTION,"centre fraction ignored"},
        {MUT_ROUND_CENTER,"centre rounded to a pixel"},{MUT_Y_SIGN,"y residual sign flipped"},
        {MUT_GUARD_INT,"guard planes from the integer centre"},{MUT_NO_CARRY,"carry and borrow dropped"}};
    for(unsigned c=0;c<sizeof(controls)/sizeof(controls[0]);++c) {
        mutation=controls[c].m;state=UINT32_C(0x5eed0c16);
        sweep_t m;memset(&m,0,sizeof(m));
        sweep(&m,150);
        printf("  control (%s): %llu mismatches\n",controls[c].name,(unsigned long long)m.mismatches);
        check(m.mismatches>0,"negative control detected");
    }
    printf("view-center-1 PASS (%llu checks)\n",(unsigned long long)checks);
    return 0;
}
