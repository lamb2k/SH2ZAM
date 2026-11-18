/* Independent oracle for view-rails-1 (sh2z_view_edge_rails_center), shared
 * by tests/test_rails.c and the SH-2 model harness. Written from the header
 * texts with no code shared with the library: each boundary is classified on
 * its own, and every stepped point comes from view-edges-1's signed formula
 *     F = trunc(delta * first * 65536 / den), S = trunc(delta * step * 65536 / den),
 *     P(k) = from + trunc((F + k*S) / 65536)
 * in 128-bit arithmetic, then view-center-1's projection
 *     q = clamp(trunc(focal * a / z), -(2^31 - 1), 2^31 - 1),
 *     sx = sat16(trunc((center_x16 + qx) / 65536)), sy = sat16(trunc((center_y16 - qy) / 65536)).
 * Mutation flags select deliberately wrong rules (negative controls). Host
 * only. MIT; SH2ZAM Contributors. */
#ifndef SH2ZAM_RAILS_ORACLE_H
#define SH2ZAM_RAILS_ORACLE_H
#include "sh2zam/view_experimental.h"
#include <string.h>

__extension__ typedef __int128 ro_wide_t;

enum {
    RO_EXACT_POINT=1,      /* P(k) = from + trunc(delta * (first + k*step) / den), no fixed-point steps */
    RO_FLOOR_NEGATIVE=2,   /* floor instead of trunc on the way down (to < from) */
    RO_PREFIX_STRICT=4,    /* copy from only when b < low */
    RO_SUFFIX_STRICT=8,    /* copy to only when b > high */
    RO_CENTER_WHOLE=16,    /* the centre's fraction ignored */
    RO_STEP_CEIL=32        /* S rounded away from zero */
};

static ro_wide_t ro_trunc(ro_wide_t n,ro_wide_t d) {
    const ro_wide_t a=n<0?-n:n,b=d<0?-d:d,q=a/b;
    return ((n<0)!=(d<0))?-q:q;
}
static ro_wide_t ro_floor(ro_wide_t n,ro_wide_t d) {
    ro_wide_t q=ro_trunc(n,d);
    if(q*d!=n && ((n<0)!=(d<0)))--q;
    return q;
}
static ro_wide_t ro_clamp(ro_wide_t a,ro_wide_t lo,ro_wide_t hi) {return a<lo?lo:a>hi?hi:a;}

/* view-center-1 projection of one view-space point. */
static sh2z_view_vertex_t ro_project(const sh2z_view_center_t *w,ro_wide_t x,ro_wide_t y,ro_wide_t z,unsigned mut) {
    sh2z_view_vertex_t o;
    memset(&o,0,sizeof(o));
    o.x=(int32_t)x;o.y=(int32_t)y;o.z=(int32_t)z;
    if(z<w->view.near_z){o.outcode=SH2Z_OUT_NEAR;return o;}
    ro_wide_t cx=w->center_x16,cy=w->center_y16;
    if(mut&RO_CENTER_WHOLE){cx=ro_floor(cx,65536)*65536;cy=ro_floor(cy,65536)*65536;}
    const ro_wide_t qx=ro_clamp(ro_trunc((ro_wide_t)w->view.focal_x*x,z),-(ro_wide_t)INT32_MAX,INT32_MAX);
    const ro_wide_t qy=ro_clamp(ro_trunc((ro_wide_t)w->view.focal_y*y,z),-(ro_wide_t)INT32_MAX,INT32_MAX);
    o.sx=(int16_t)ro_clamp(ro_trunc(cx+qx,65536),INT16_MIN,INT16_MAX);
    o.sy=(int16_t)ro_clamp(ro_trunc(cy-qy,65536),INT16_MIN,INT16_MAX);
    const sh2z_view_t *v=&w->view;
    o.outcode=(uint8_t)((z>v->far_z?SH2Z_OUT_FAR:0u)|(o.sx<v->clip[0]?SH2Z_OUT_LEFT:0u)|(o.sx>v->clip[2]?SH2Z_OUT_RIGHT:0u)|
        (o.sy<v->clip[1]?SH2Z_OUT_TOP:0u)|(o.sy>v->clip[3]?SH2Z_OUT_BOTTOM:0u));
    return o;
}
/* One axis of point k of a stepped run. */
static ro_wide_t ro_axis(int32_t from,int32_t to,ro_wide_t first,ro_wide_t step,ro_wide_t den,ro_wide_t k,unsigned mut) {
    const ro_wide_t delta=(ro_wide_t)to-from;
    if(mut&RO_EXACT_POINT)return from+ro_trunc(delta*(first+k*step),den);
    const ro_wide_t f=ro_trunc(delta*first*65536,den);
    ro_wide_t s=ro_trunc(delta*step*65536,den);
    if((mut&RO_STEP_CEIL) && s*den!=delta*step*65536)s+=delta<0?-1:1;
    const ro_wide_t acc=f+k*s;
    return from+((mut&RO_FLOOR_NEGATIVE)?ro_floor(acc,65536):ro_trunc(acc,65536));
}
/* A rail's copied prefix [0, f) and suffix [e, count), from the header text. */
static void ro_split(const int32_t *b,unsigned count,int32_t low,int32_t high,unsigned mut,unsigned *fp,unsigned *ep) {
    unsigned f=0,e=count;
    for(;f<count;++f)if((mut&RO_PREFIX_STRICT)?!(b[f]<low):!(b[f]<=low))break;
    for(;e>f;--e)if((mut&RO_SUFFIX_STRICT)?!(b[e-1]>high):!(b[e-1]>=high))break;
    *fp=f;*ep=e;
}
/* One rail. Returns false where the contract says the call fails because
 * of this rail. */
static bool ro_rail(sh2z_view_vertex_t *out,const sh2z_view_center_t *w,const sh2z_view_vertex_t *from,
    const sh2z_view_vertex_t *to,const int32_t *b,unsigned count,int32_t low,int32_t high,unsigned mut) {
    unsigned f,e;
    ro_split(b,count,low,high,mut,&f,&e);
    const unsigned n=e-f;
    if(n) {
        const ro_wide_t den=(ro_wide_t)high-low,first=(ro_wide_t)b[f]-low;
        if(!(low<high) || den>65535 || first>=den)return false;
        if(n>=2) {
            const ro_wide_t step=(ro_wide_t)b[f+1]-b[f],last=(ro_wide_t)b[e-1]-low;
            if(!(step>0 && step<den && first<last && last<den && last-first==(ro_wide_t)(n-1)*step))return false;
        }
    }
    const ro_wide_t den=(ro_wide_t)high-low,first=n?(ro_wide_t)b[f]-low:0,step=n>1?(ro_wide_t)b[f+1]-b[f]:0;
    for(unsigned j=0;j<count;++j) {
        if(j<f)out[j]=*from;
        else if(j>=e)out[j]=*to;
        else {
            const ro_wide_t k=(ro_wide_t)j-f;
            out[j]=ro_project(w,ro_axis(from->x,to->x,first,step,den,k,mut),ro_axis(from->y,to->y,first,step,den,k,mut),
                ro_axis(from->z,to->z,first,step,den,k,mut),mut);
        }
    }
    return true;
}
/* Whole call on valid pointers and a valid view: false when the contract
 * rejects the arguments (then out_l, out_r are unspecified here). */
static bool ro_rails(sh2z_view_vertex_t *out_l,sh2z_view_vertex_t *out_r,const sh2z_view_center_t *w,
    const sh2z_view_vertex_t ends[4],const int32_t *b,unsigned count,
    int32_t l_low,int32_t l_high,int32_t r_low,int32_t r_high,unsigned mut) {
    if(count>SH2Z_VIEW_RAILS_MAX_COUNT)return false;
    const bool l=ro_rail(out_l,w,&ends[0],&ends[1],b,count,l_low,l_high,mut);
    const bool r=ro_rail(out_r,w,&ends[2],&ends[3],b,count,r_low,r_high,mut);
    return l && r;
}
static bool ro_same(const sh2z_view_vertex_t *a,const sh2z_view_vertex_t *b) {
    return a->x==b->x && a->y==b->y && a->z==b->z && a->sx==b->sx && a->sy==b->sy && a->outcode==b->outcode &&
        a->reserved[0]==b->reserved[0] && a->reserved[1]==b->reserved[1] && a->reserved[2]==b->reserved[2];
}
/* ---- fixtures: views, ends and strips (shared with the model harness) ---- */
static uint32_t ro_state=UINT32_C(0x7a11c0de);
static uint32_t ro_rnd(void) {ro_state^=ro_state<<13;ro_state^=ro_state>>17;ro_state^=ro_state<<5;return ro_state;}
static int32_t ro_range(int32_t lo,int32_t hi) {return (int32_t)((int64_t)lo+(int64_t)(ro_rnd()%(uint64_t)((int64_t)hi-lo+1)));}
/* A valid centre view (identity rotation: only the projection matters). */
static sh2z_view_center_t ro_view(void) {
    sh2z_view_center_t w;
    memset(&w,0,sizeof(w));
    sh2z_view_t *v=&w.view;
    v->r[0][0]=v->r[1][1]=v->r[2][2]=16384;
    switch(ro_rnd()%4u) {
    case 0: v->focal_x=v->focal_y=ro_range(1,INT32_MAX); break;
    case 1: v->focal_x=ro_range(1<<20,1<<26);v->focal_y=ro_range(1<<20,1<<26); break;
    case 2: v->focal_x=v->focal_y=256*65536; break;
    default: v->focal_x=ro_range(1,1000);v->focal_y=ro_range(1,INT32_MAX); break;
    }
    switch(ro_rnd()%3u) {
    case 0: v->near_z=ro_range(1,64); break;
    case 1: v->near_z=ro_range(1,1<<20); break;
    default: v->near_z=ro_range(1,INT32_MAX-1); break;
    }
    v->far_z=ro_range(v->near_z+1,INT32_MAX);
    const int16_t a=(int16_t)ro_range(-1200,1200),b=(int16_t)ro_range(-1200,1200),c=(int16_t)ro_range(-1200,1200),d=(int16_t)ro_range(-1200,1200);
    v->clip[0]=a<b?a:b;v->clip[2]=a<b?b:a;v->clip[1]=c<d?c:d;v->clip[3]=c<d?d:c;
    w.center_x16=(ro_rnd()%4u)?ro_range(-1000*65536,1000*65536):(int32_t)ro_rnd();
    w.center_y16=(ro_rnd()%4u)?ro_range(-1000*65536,1000*65536):(int32_t)ro_rnd();
    v->center_x=(int16_t)ro_floor(w.center_x16,65536);
    v->center_y=(int16_t)ro_floor(w.center_y16,65536);
    return w;
}
/* A road renderer's camera: 200 px focal length, a 320 x 224 clip, near a
 * quarter of a model unit, far 8000 units, centre (160, 112 + 1/65536). */
static sh2z_view_center_t ro_road_view(void) {
    sh2z_view_center_t w;
    memset(&w,0,sizeof(w));
    sh2z_view_t *v=&w.view;
    v->r[0][0]=v->r[1][1]=v->r[2][2]=16384;
    v->focal_x=v->focal_y=200*65536;v->near_z=16384/4;v->far_z=8000*16384;
    v->clip[0]=0;v->clip[1]=0;v->clip[2]=319;v->clip[3]=223;
    v->center_x=160;v->center_y=112;
    w.center_x16=160*65536;w.center_y16=112*65536+1;
    return w;
}
static int32_t ro_coord(void) {
    switch(ro_rnd()%6u) {
    case 0: return (int32_t)ro_rnd();
    case 1: return ro_range(-100000,100000);
    case 2: return (ro_rnd()&1u)?INT32_MAX-ro_range(0,3):INT32_MIN+ro_range(0,3);
    case 3: return ro_range(-(1<<16),1<<16);
    default: return ro_range(-(1<<24),1<<24);
    }
}
/* A rail end with the bytes a projection gives it. Road ends: model units of
 * 16384, a few hundred units around the camera, mostly in front. */
static sh2z_view_vertex_t ro_corner(const sh2z_view_center_t *w,bool road) {
    int32_t x,y,z;
    if(road) {
        x=ro_range(-400*16384,400*16384);y=ro_range(-20*16384,4*16384);
        const int64_t far=(int64_t)w->view.near_z+ro_range(1,600*16384);
        z=(ro_rnd()%8u)?ro_range(w->view.near_z,far>INT32_MAX?INT32_MAX:(int32_t)far):ro_range(-100*16384,w->view.near_z);
    } else {
        x=ro_coord();y=ro_coord();z=(ro_rnd()%3u)?ro_coord():ro_range(w->view.near_z-2,w->view.near_z+2);
    }
    return ro_project(w,x,y,z,0);
}
typedef struct {
    int32_t b[96];
    unsigned count;
    int32_t low[2],high[2],unit;
} ro_strip_t;
/* A road renderer's strip: boundary[0] = low, then each multiple of the
 * repetition length (256, 512 or 1024 texture rows) up to high; each rail's
 * low and high inside [low, high]. Repetitions per quad: mostly 1 to 3,
 * sometimes up to 8, rarely up to 64 (2 to 65 boundaries). */
static void ro_road_strip(ro_strip_t *s) {
    static const int32_t units[3]={256,512,1024};
    const int32_t unit=units[ro_rnd()%3u];
    const unsigned r=ro_rnd()%16u;
    const int32_t reps=r<10u?ro_range(1,3):r<15u?ro_range(3,8):ro_range(8,64);
    const int32_t low=ro_range(-40000,40000);
    int32_t high=low+ro_range(1,reps*unit);
    if(ro_rnd()%4u==0)high=low+reps*unit;
    s->unit=unit;s->count=0;s->b[s->count++]=low;
    for(int32_t start=low;start<high && s->count<65u;) {
        const int32_t phase=start&(unit-1);
        int32_t end=start+unit-phase;
        if(end>high)end=high;
        s->b[s->count++]=end;start=end;
    }
    if(s->b[s->count-1]!=high)s->b[s->count-1]=high;
    for(unsigned rail=0;rail<2;++rail) {
        int32_t a=ro_range(low,high),c=ro_range(low,high);
        if(ro_rnd()%3u==0){a=low;c=high;}
        else if(ro_rnd()%2u){a=(ro_rnd()&1u)?low:a;c=(ro_rnd()&1u)?high:c;}
        s->low[rail]=a<c?a:c;s->high[rail]=a<c?c:a;
    }
    if(ro_rnd()%3u==0){s->low[1]=s->low[0];s->high[1]=s->high[0];}
}
/* General strips: arbitrary start, equally spaced interior, ragged ends,
 * lows and highs anywhere (also outside the boundaries, reversed, equal). */
static void ro_general_strip(ro_strip_t *s) {
    const unsigned n=1u+ro_rnd()%70u;
    const int32_t step=(ro_rnd()%4u)?ro_range(1,300):ro_range(1,40000/(int32_t)(n+1));
    int32_t at=(ro_rnd()%8u)?ro_range(-100000,100000):((ro_rnd()&1u)?INT32_MIN+ro_range(0,5):INT32_MAX-(int32_t)(n+1)*step-10-ro_range(0,5));
    s->unit=step;s->count=0;
    if(ro_rnd()&1u){s->b[s->count++]=at;at+=ro_range(1,step+5);}
    for(unsigned k=0;k<n && s->count<90u;++k){s->b[s->count++]=at;if(k+1<n)at+=step;}
    if((ro_rnd()&1u) && at<INT32_MAX-step-10)s->b[s->count++]=at+ro_range(1,step+5);
    const int32_t b0=s->b[0],b1=s->b[s->count-1];
    for(unsigned rail=0;rail<2;++rail) {
        const int64_t span=(int64_t)b1-b0;
        int64_t a=b0+(int64_t)(ro_rnd()%(uint64_t)(span+3))-1,c=b0+(int64_t)(ro_rnd()%(uint64_t)(span+3))-1;
        if(ro_rnd()%5u==0)a=(int64_t)b0-ro_range(0,70000);
        if(ro_rnd()%5u==0)c=(int64_t)b1+ro_range(0,70000);
        a=a<INT32_MIN?INT32_MIN:a>INT32_MAX?INT32_MAX:a;
        c=c<INT32_MIN?INT32_MIN:c>INT32_MAX?INT32_MAX:c;
        if(ro_rnd()%9u==0){const int64_t t=a;a=c;c=t;}
        s->low[rail]=(int32_t)a;s->high[rail]=(int32_t)c;
    }
    /* Sometimes unsorted: swap two entries, or overwrite one in the middle. */
    if(s->count>3u && ro_rnd()%4u==0) {
        const unsigned i=ro_rnd()%s->count,j=ro_rnd()%s->count;
        if(ro_rnd()&1u){const int32_t t=s->b[i];s->b[i]=s->b[j];s->b[j]=t;}
        else s->b[1u+ro_rnd()%(s->count-2u)]=(ro_rnd()&1u)?(int32_t)ro_rnd():s->b[0];
    }
}
#endif
