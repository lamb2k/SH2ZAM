/* Independent oracle for pose-1 (sh2z_pose_from_f32): the reference
 * procedure of the header text written with the host's binary32 arithmetic
 * (float operations, as the caller computes its pose today) and 64-bit
 * integers, sharing no code with the library. Also the input generators used
 * by the host test and the SH-2 model harness, and mutated variants of the
 * procedure for the negative controls.
 *
 * The host must evaluate float expressions in binary32 (FLT_EVAL_METHOD 0,
 * no fast-math, no contraction across the products below), which x86-64 and
 * AArch64 GCC and Clang do by default. MIT; SH2ZAM Contributors. */
#ifndef SH2ZAM_POSE_ORACLE_H
#define SH2ZAM_POSE_ORACLE_H
#include "sh2zam/view_experimental.h"
#include <float.h>
#include <math.h>
#include <string.h>

#if !defined(FLT_EVAL_METHOD) || FLT_EVAL_METHOD!=0
#error "the pose oracle needs binary32 float evaluation"
#endif

enum {
    PO_EXACT=0,
    PO_EXACT_DELTA,       /* position - eye without binary32 rounding */
    PO_EXACT_PRODUCT,     /* basis * scale without binary32 rounding */
    PO_EXACT_RECIPROCAL,  /* 1 / scale without binary32 rounding */
    PO_FLOOR_VIEW,        /* view.r and view.t floored, not rounded */
    PO_FLOOR_BASIS,       /* basis Q14 floored, not rounded */
    PO_GRAM_511,          /* Gram tolerance 511 instead of 512 */
    PO_NO_LOCAL_ROUND,    /* eye without the intermediate R_30 */
    PO_AXES_NOT_REVERSED, /* ignores SH2Z_POSE_REVERSE_AXES */
    PO_MUTATIONS
};

static inline uint32_t po_bits(float f) {uint32_t u;memcpy(&u,&f,4);return u;}
static inline float po_float(uint32_t u) {float f;memcpy(&f,&u,4);return f;}
static inline bool po_finite(float x) {return (po_bits(x)&UINT32_C(0x7f800000))!=UINT32_C(0x7f800000);}
/* The caller's conversion: bound check, an exact power-of-two scaling and a
 * truncating conversion. */
static inline bool po_fixed(float x,float scale,float bound,int32_t *out) {
    if(!po_finite(x) || x< -bound || x>bound)return false;
    const float product=x*scale;
    if(product<=-2147483648.0f || product>=2147483648.0f)return false;
    *out=(int32_t)product;
    return true;
}
static inline int64_t po_rounded(int64_t x,int64_t divisor) {
    return (x>=0?x+divisor/2:-(-x+divisor/2))/divisor;
}
static inline int64_t po_floor_div(int64_t x,int64_t divisor) {
    return x>=0?x/divisor:-((-x+divisor-1)/divisor);
}
static inline bool po_view_ok(const sh2z_view_t *v) {
    if(v->focal_x<=0 || v->focal_y<=0 || v->near_z<=0 || v->far_z<=v->near_z ||
       v->clip[0]>v->clip[2] || v->clip[1]>v->clip[3])return false;
    for(unsigned i=0;i<3;++i) {
        int64_t sum=0;
        for(unsigned j=0;j<3;++j)sum+=v->r[i][j]<0?-(int64_t)v->r[i][j]:v->r[i][j];
        if(sum*32768+(v->t[i]<0?-(int64_t)v->t[i]:v->t[i])>INT32_MAX)return false;
    }
    return true;
}

/* The reference procedure. Returns false where the library must fail. */
static bool po_pose(sh2z_pose_t *out,const sh2z_view_t *base,const int32_t rotation[3][3],
    const float mbasis[3][3],const float mscale[3],const float position[3],const float eye[3],
    unsigned flags,int mutation) {
    if(flags&~SH2Z_POSE_REVERSE_AXES)return false;
    sh2z_pose_t next;
    memset(&next,0,sizeof next);
    next.view=*base;
    const bool reverse=(flags&SH2Z_POSE_REVERSE_AXES) && mutation!=PO_AXES_NOT_REVERSED;
    int32_t basis[3][3],model[3][3],delta[3],inverse_scale[3];
    for(unsigned i=0;i<3;++i) {
        if(!po_finite(eye[i]) || !po_finite(position[i]))return false;
        if(mutation==PO_EXACT_DELTA) {
            const double d=(double)position[i]-(double)eye[i];
            if(d< -4096.0 || d>4096.0)return false;
            delta[i]=(int32_t)(d*65536.0);
        } else if(!po_fixed(position[i]-eye[i],65536.0f,4096.0f,&delta[i]))return false;
        const unsigned ai=reverse?2u-i:i;
        const float s=mscale[ai];
        if(!po_finite(s) || s<0.0625f || s>1.0f)return false;
        if(mutation==PO_EXACT_RECIPROCAL)inverse_scale[i]=(int32_t)(65536.0/(double)s);
        else if(!po_fixed(1.0f/s,65536.0f,16.0f,&inverse_scale[i]))return false;
        for(unsigned j=0;j<3;++j) {
            const unsigned a=reverse?2u-j:j;
            if(rotation[i][j]< -1100000000 || rotation[i][j]>1100000000 ||
               !po_fixed(mbasis[a][i],1073741824.0f,1.00001f,&basis[i][j]))return false;
            if(mutation==PO_EXACT_PRODUCT) {
                /* scale[a] may not be range-checked yet; a bad scale fails the pose anyway */
                const double p=(double)mbasis[a][i]*(double)mscale[a]*65536.0;
                if(!(p> -2147483648.0 && p<2147483648.0))return false;
                model[i][j]=(int32_t)p;
            } else if(!po_fixed(mbasis[a][i]*mscale[a],65536.0f,1.00001f,&model[i][j]))return false;
            next.basis[i][j]=(int16_t)(mutation==PO_FLOOR_BASIS?po_floor_div(basis[i][j],65536):po_rounded(basis[i][j],65536));
        }
    }
    for(unsigned i=0;i<3;++i) {
        for(unsigned j=0;j<3;++j) {
            int64_t value=0;
            for(unsigned k=0;k<3;++k)value+=(int64_t)rotation[i][k]*model[k][j];
            const int64_t q=mutation==PO_FLOOR_VIEW?po_floor_div(value,INT64_C(4294967296)):po_rounded(value,INT64_C(4294967296));
            if(q<INT16_MIN || q>INT16_MAX)return false;
            next.view.r[i][j]=(int16_t)q;
        }
        int64_t value=0;
        for(unsigned k=0;k<3;++k)value+=(int64_t)rotation[i][k]*delta[k];
        const int64_t q=mutation==PO_FLOOR_VIEW?po_floor_div(value,INT64_C(4294967296)):po_rounded(value,INT64_C(4294967296));
        if(q<INT32_MIN || q>INT32_MAX)return false;
        next.view.t[i]=(int32_t)q;
    }
    next.cull_valid=true;next.eye_error=2;
    const int64_t tolerance=mutation==PO_GRAM_511?511:512;
    for(unsigned i=0;i<3;++i)for(unsigned j=0;j<3;++j) {
        int64_t product=0;
        for(unsigned k=0;k<3;++k)product+=(int64_t)basis[k][i]*basis[k][j];
        const int64_t error=po_rounded(product,INT64_C(1073741824))-(i==j?INT64_C(1073741824):0);
        if(error< -tolerance || error>tolerance)next.cull_valid=false;
    }
    for(unsigned i=0;i<3;++i) {
        int64_t dot=0;
        for(unsigned k=0;k<3;++k)dot-=(int64_t)basis[k][i]*delta[k];
        int64_t e;
        if(mutation==PO_NO_LOCAL_ROUND) {
            /* R_62(dot * I) with a 128-bit product. */
            __extension__ typedef __int128 po_i128;
            po_i128 p=(po_i128)dot*inverse_scale[i];
            const po_i128 h=(po_i128)1<<61;
            p=p>=0?(p+h)>>62:-((-p+h)>>62);
            e=(int64_t)p;
        } else {
            const int64_t local=po_rounded(dot,INT64_C(1073741824));
            e=po_rounded(local*inverse_scale[i],INT64_C(4294967296));
        }
        if(e< -30000 || e>30000)next.cull_valid=false;
        if(e<INT32_MIN || e>INT32_MAX)return false;
        next.eye[i]=(int32_t)e;
        if(e>=0)next.octant|=(uint8_t)(1u<<i);
    }
    if(!po_view_ok(&next.view))return false;
    *out=next;
    return true;
}

/* Field-by-field equality (no padding). */
static inline bool po_same(const sh2z_pose_t *a,const sh2z_pose_t *b) {
    for(unsigned i=0;i<3;++i) {
        if(a->view.t[i]!=b->view.t[i] || a->eye[i]!=b->eye[i])return false;
        for(unsigned j=0;j<3;++j)if(a->view.r[i][j]!=b->view.r[i][j] || a->basis[i][j]!=b->basis[i][j])return false;
    }
    for(unsigned i=0;i<4;++i)if(a->view.clip[i]!=b->view.clip[i])return false;
    return a->view.focal_x==b->view.focal_x && a->view.focal_y==b->view.focal_y &&
        a->view.center_x==b->view.center_x && a->view.center_y==b->view.center_y &&
        a->view.near_z==b->view.near_z && a->view.far_z==b->view.far_z &&
        a->eye_error==b->eye_error && a->octant==b->octant && a->cull_valid==b->cull_valid;
}

/* ---- generators ----------------------------------------------------------- */

typedef struct {
    int32_t rotation[3][3];
    float basis[3][3],scale[3],position[3],eye[3];
    unsigned flags;
} po_case_t;

static uint64_t po_state=UINT64_C(0x9e3779b97f4a7c15);
static inline uint64_t po_next(void) {
    po_state^=po_state>>12;po_state^=po_state<<25;po_state^=po_state>>27;
    return po_state*UINT64_C(0x2545f4914f6cdd1d);
}
static inline uint32_t po_rnd(void) {return (uint32_t)(po_next()>>32);}
static inline double po_unit(void) {return (double)(po_next()>>11)*(1.0/9007199254740992.0);}
static inline double po_uniform(double lo,double hi) {return lo+(hi-lo)*po_unit();}

/* A rotation matrix from a uniform random unit quaternion. */
static inline void po_rotation(double m[3][3]) {
    double q[4],n=0;
    do {n=0;for(unsigned i=0;i<4;++i){q[i]=po_uniform(-1,1);n+=q[i]*q[i];}} while(n<1e-3 || n>1);
    n=sqrt(n);for(unsigned i=0;i<4;++i)q[i]/=n;
    const double w=q[0],x=q[1],y=q[2],z=q[3];
    m[0][0]=1-2*(y*y+z*z);m[0][1]=2*(x*y-w*z);m[0][2]=2*(x*z+w*y);
    m[1][0]=2*(x*y+w*z);m[1][1]=1-2*(x*x+z*z);m[1][2]=2*(y*z-w*x);
    m[2][0]=2*(x*z-w*y);m[2][1]=2*(y*z+w*x);m[2][2]=1-2*(x*x+y*y);
}
static const float po_game_scales[]={0.0625f,0.1f,0.125f,0.15f,0.175f,0.2f,0.25f,0.3f,0.5f,0.75f,1.0f};
static inline float po_scale(void) {
    switch(po_rnd()%4u) {
    case 0:return po_game_scales[po_rnd()%(sizeof po_game_scales/sizeof po_game_scales[0])];
    case 1:return po_float(po_bits(po_game_scales[po_rnd()%(sizeof po_game_scales/sizeof po_game_scales[0])])+(po_rnd()%5u)-2u);
    default:return (float)po_uniform(0.0625,1.0);
    }
}
static inline float po_clamp_scale(float s) {return s<0.0625f?0.0625f:s>1.0f?1.0f:s;}
static inline double po_world(void) {
    switch(po_rnd()%8u) {
    case 0:return po_uniform(-1000,1000);
    case 1:return po_uniform(-1e5,1e5);
    case 2:return po_uniform(-16777216.0,16777216.0);
    case 3:return (double)(int32_t)(po_rnd()%65535u)-32767.0;   /* whole units */
    default:return po_uniform(-32767,32767);
    }
}
/* A visible model: camera rotation, a float model basis (sometimes slightly
 * scaled to straddle the Gram tolerance), position anywhere in the world and
 * an eye that puts the model in front of the camera within 4096 units. */
static inline void po_random_case(po_case_t *c) {
    double cam[3][3],model[3][3];
    po_rotation(cam);po_rotation(model);
    for(unsigned i=0;i<3;++i)for(unsigned j=0;j<3;++j)c->rotation[i][j]=(int32_t)llround(cam[i][j]*1073741824.0);
    const unsigned kind=po_rnd()%4u;
    for(unsigned a=0;a<3;++a) {
        const double stretch=kind==0?1.0+po_uniform(-6e-7,6e-7):kind==1?1.0+po_uniform(-2e-6,2e-6):1.0;
        for(unsigned k=0;k<3;++k) {
            double v=model[a][k]*stretch;
            if(v>1.00001)v=1.00001;
            if(v< -1.00001)v=-1.00001;
            c->basis[a][k]=(float)v;
        }
        c->scale[a]=po_clamp_scale(po_scale());
    }
    if(po_rnd()%3u==0u)c->scale[1]=c->scale[2]=c->scale[0];
    double view[3];
    view[2]=po_uniform(20,3000);
    view[0]=po_uniform(-0.9,0.9)*view[2];
    view[1]=po_uniform(-0.7,0.7)*view[2];
    if(po_rnd()%8u==0u)for(unsigned i=0;i<3;++i)view[i]=po_uniform(-2400,2400);
    for(unsigned k=0;k<3;++k) {
        double world=0;
        for(unsigned i=0;i<3;++i)world+=cam[i][k]*view[i];   /* R^T view */
        c->position[k]=(float)po_world();
        c->eye[k]=(float)((double)c->position[k]-world);
    }
    c->flags=po_rnd()%2u?SH2Z_POSE_REVERSE_AXES:0u;
}
/* The caller's base view: 320x224, focal 160, centre (160, 112) adjusted to
 * the library's raster rule, near 16 and far 2000 units (Q18.14). */
static inline void po_base(sh2z_view_t *v) {
    memset(v,0,sizeof *v);
    v->focal_x=v->focal_y=160*65536;
    v->center_x=160;v->center_y=111;
    v->near_z=16*4096*4;v->far_z=2000*4096*4;
    v->clip[0]=12;v->clip[1]=16;v->clip[2]=307;v->clip[3]=223;
}
#endif
