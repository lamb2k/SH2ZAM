/* SH-2 bridge for the executed-instruction pose model. MIT; Copyright (c) 2026
 * SH2ZAM Contributors. Every entry point is called directly by the model
 * harness with the SH-2 calling convention; inputs are written by the harness
 * before each call; nothing here uses libc.
 *
 *   fast_pose   sh2z_pose_from_f32 (the library)
 *   alias_base, alias_inputs
 *               the same with base as &out->view, or with every float input
 *               stored inside *out
 *   plain_pose  the same pose computed the way a renderer computes it today:
 *               binary32 arithmetic (libgcc soft-float on SH-2), a checked
 *               float-to-fixed conversion per value, 64-bit sums and
 *               ties-away rounding, then sh2z_view_init
 *   plain_base  that renderer's per-call base view: the camera rotation
 *               rounded from Q2.30 to Q1.14, the centre rounded to pixels and
 *               sh2z_view_place at the origin (the library takes this view
 *               as an input and reads only its focal, centre, depth and clip)
 * set_base and export_pose move data between word arrays and the structs
 * outside the timed calls; test_delta, test_inverse and test_element reach
 * the library's internal difference, reciprocal and basis-value routines. */
#include "sh2zam/view_experimental.h"

#define NOINLINE __attribute__((noinline))

int32_t rotation[3][3];
float basis[3][3],scale[3],position[3],eye[3];
int32_t base_words[10];       /* focal x, y; centre x, y; near, far; clip[4] */
int32_t source_view[16];      /* plain_base input: focal[2], centre16[2], near, far, clip[4] */
sh2z_view_t base,plain_base_view;
sh2z_pose_t pose_fast,pose_plain;
int32_t words[40];

NOINLINE void set_base(void) {
    base.focal_x=base_words[0];base.focal_y=base_words[1];
    base.center_x=(int16_t)base_words[2];base.center_y=(int16_t)base_words[3];
    base.near_z=base_words[4];base.far_z=base_words[5];
    for(unsigned i=0;i<4;++i)base.clip[i]=(int16_t)base_words[6+i];
    for(unsigned i=0;i<3;++i){base.t[i]=0;for(unsigned j=0;j<3;++j)base.r[i][j]=0;}
}
/* The aliasing the header allows: base as &out->view, and every float input
 * stored inside *out. */
union { sh2z_pose_t pose; float f[24]; } alias_out;
NOINLINE uint32_t alias_base(uint32_t flags) {
    sh2z_view_t *v=&alias_out.pose.view;
    v->focal_x=base.focal_x;v->focal_y=base.focal_y;
    v->center_x=base.center_x;v->center_y=base.center_y;
    v->near_z=base.near_z;v->far_z=base.far_z;
    for(unsigned i=0;i<4;++i)v->clip[i]=base.clip[i];
    return sh2z_pose_from_f32(&alias_out.pose,v,(const int32_t (*)[3])rotation,(const float (*)[3])basis,
        scale,position,eye,flags);
}
NOINLINE uint32_t alias_inputs(uint32_t flags) {
    float *f=alias_out.f;
    for(unsigned i=0;i<3;++i){f[i]=position[i];f[3+i]=eye[i];f[6+i]=scale[i];}
    for(unsigned i=0;i<9;++i)f[9+i]=basis[i/3][i%3];
    return sh2z_pose_from_f32(&alias_out.pose,&base,(const int32_t (*)[3])rotation,(const float (*)[3])(f+9),
        f+6,f,f+3,flags);
}

NOINLINE void export_pose(uint32_t which) {
    const sh2z_pose_t *p=which==2u?&alias_out.pose:which?&pose_plain:&pose_fast;
    unsigned n=0;
    for(unsigned i=0;i<3;++i)for(unsigned j=0;j<3;++j)words[n++]=p->view.r[i][j];
    for(unsigned i=0;i<3;++i)words[n++]=p->view.t[i];
    words[n++]=p->view.focal_x;words[n++]=p->view.focal_y;
    words[n++]=p->view.center_x;words[n++]=p->view.center_y;
    words[n++]=p->view.near_z;words[n++]=p->view.far_z;
    for(unsigned i=0;i<4;++i)words[n++]=p->view.clip[i];
    for(unsigned i=0;i<3;++i)words[n++]=p->eye[i];
    for(unsigned i=0;i<3;++i)for(unsigned j=0;j<3;++j)words[n++]=p->basis[i][j];
    words[n++]=p->eye_error;words[n++]=p->octant;words[n++]=p->cull_valid;
}
/* Fill both outputs with a pattern (a failed call must leave it). */
NOINLINE void poison(void) {
    unsigned char *a=(unsigned char *)&pose_fast,*b=(unsigned char *)&pose_plain;
    for(unsigned i=0;i<sizeof pose_fast;++i){a[i]=0xA5;b[i]=0xA5;}
}

/* The library's internal SH-2 routines, through its test entries. */
int32_t sh2zi_pose_test_delta(uint32_t a,uint32_t b);
uint32_t sh2zi_pose_test_inverse(uint32_t s);
uint32_t sh2zi_pose_test_element(uint32_t b,uint32_t s,int32_t out[2]);
int32_t element_out[2];
NOINLINE uint32_t test_delta(uint32_t a,uint32_t b) {return (uint32_t)sh2zi_pose_test_delta(a,b);}
NOINLINE uint32_t test_inverse(uint32_t s) {return sh2zi_pose_test_inverse(s);}
NOINLINE uint32_t test_element(uint32_t b,uint32_t s) {return sh2zi_pose_test_element(b,s,element_out);}

NOINLINE uint32_t fast_pose(uint32_t flags) {
    return sh2z_pose_from_f32(&pose_fast,&base,(const int32_t (*)[3])rotation,(const float (*)[3])basis,
        scale,position,eye,flags);
}

/* ---- the renderer's float pose ---- */
static bool plain_finite(float x) {
    union {float f;uint32_t u;} v;
    v.f=x;
    return (v.u&0x7f800000u)!=0x7f800000u;
}
static bool plain_fixed(float x,float scale_by,float bound,int32_t *out) {
    if(!plain_finite(x)||x< -bound||x>bound)return false;
    const float product=x*scale_by;
    if(product<=-2147483648.0f||product>=2147483648.0f)return false;
    *out=(int32_t)product;return true;
}
static int64_t plain_rounded(int64_t x,int64_t divisor) {
    return (x>=0?x+divisor/2:-(-x+divisor/2))/divisor;
}
NOINLINE uint32_t plain_pose(uint32_t flags) {
    if(flags&~SH2Z_POSE_REVERSE_AXES)return false;
    sh2z_pose_t next;
    {
        unsigned char *p=(unsigned char *)&next;
        for(unsigned i=0;i<sizeof next;++i)p[i]=0;
    }
    next.view.focal_x=base.focal_x;next.view.focal_y=base.focal_y;
    next.view.center_x=base.center_x;next.view.center_y=base.center_y;
    next.view.near_z=base.near_z;next.view.far_z=base.far_z;
    for(unsigned i=0;i<4;++i)next.view.clip[i]=base.clip[i];
    const bool reverse=(flags&SH2Z_POSE_REVERSE_AXES)!=0;
    int32_t b[3][3],model[3][3],delta[3],inverse_scale[3];
    for(unsigned i=0;i<3;++i) {
        if(!plain_finite(eye[i])||!plain_finite(position[i])||
           !plain_fixed(position[i]-eye[i],65536.0f,4096.0f,&delta[i]))return false;
        const float s=scale[reverse?2u-i:i];
        if(!plain_finite(s)||s<0.0625f||s>1.0f||
           !plain_fixed(1.0f/s,65536.0f,16.0f,&inverse_scale[i]))return false;
        for(unsigned j=0;j<3;++j) {
            const unsigned a=reverse?2u-j:j;
            if(rotation[i][j]<-1100000000||rotation[i][j]>1100000000||
               !plain_fixed(basis[a][i],1073741824.0f,1.00001f,&b[i][j])||
               !plain_fixed(basis[a][i]*scale[a],65536.0f,1.00001f,&model[i][j]))return false;
            next.basis[i][j]=(int16_t)plain_rounded(b[i][j],65536);
        }
    }
    for(unsigned i=0;i<3;++i) {
        for(unsigned j=0;j<3;++j) {
            int64_t value=0;
            for(unsigned k=0;k<3;++k)value+=(int64_t)rotation[i][k]*model[k][j];
            const int64_t q=plain_rounded(value,INT64_C(4294967296));
            if(q<INT16_MIN||q>INT16_MAX)return false;
            next.view.r[i][j]=(int16_t)q;
        }
        int64_t value=0;
        for(unsigned k=0;k<3;++k)value+=(int64_t)rotation[i][k]*delta[k];
        const int64_t q=plain_rounded(value,INT64_C(4294967296));
        if(q<INT32_MIN||q>INT32_MAX)return false;
        next.view.t[i]=(int32_t)q;
    }
    next.cull_valid=true;next.eye_error=2;
    for(unsigned i=0;i<3;++i)for(unsigned j=0;j<3;++j) {
        int64_t product=0;
        for(unsigned k=0;k<3;++k)product+=(int64_t)b[k][i]*b[k][j];
        const int64_t error=plain_rounded(product,INT64_C(1073741824))-(i==j?INT64_C(1073741824):0);
        if(error< -512||error>512)next.cull_valid=false;
    }
    for(unsigned i=0;i<3;++i) {
        int64_t dot=0;
        for(unsigned k=0;k<3;++k)dot-=(int64_t)b[k][i]*delta[k];
        const int64_t local=plain_rounded(dot,INT64_C(1073741824));
        const int64_t e=plain_rounded(local*inverse_scale[i],INT64_C(4294967296));
        if(e< -30000||e>30000)next.cull_valid=false;
        if(e<INT32_MIN||e>INT32_MAX)return false;
        next.eye[i]=(int32_t)e;
        if(e>=0)next.octant|=(uint8_t)(1u<<i);
    }
    if(!sh2z_view_init(&next.view,&next.view))return false;
    {
        const unsigned char *s=(const unsigned char *)&next;
        unsigned char *d=(unsigned char *)&pose_plain;
        for(unsigned i=0;i<sizeof next;++i)d[i]=s[i];
    }
    return true;
}

/* ---- the renderer's per-call base view ---- */
NOINLINE uint32_t plain_base(void) {
    if(source_view[4]<=0 || source_view[5]<=source_view[4] || source_view[5]>INT32_MAX/4 ||
       source_view[6]>=source_view[8] || source_view[7]>=source_view[9])return false;
    const int64_t x=source_view[2],y=source_view[3];
    const int32_t cx=(int32_t)((x>=0?x+32768:-(-x+32768))/65536);
    const int32_t cy=(int32_t)((y>=0?y+32768:-(-y+32768))/65536);
    if(cx<INT16_MIN || cx>INT16_MAX || cy<=INT16_MIN || cy>INT16_MAX)return false;
    sh2z_view_t next;
    for(unsigned i=0;i<3;++i)for(unsigned j=0;j<3;++j) {
        const int64_t n=rotation[i][j];
        const int64_t q=(n>=0?n+32768:-(-n+32768))/65536;
        if(q<INT16_MIN || q>INT16_MAX)return false;
        next.r[i][j]=(int16_t)q;
    }
    next.focal_x=source_view[0];next.focal_y=source_view[1];
    next.center_x=(int16_t)cx;next.center_y=(int16_t)(cy-1);
    next.near_z=source_view[4]*4;next.far_z=source_view[5]*4;
    for(unsigned i=0;i<4;++i)next.clip[i]=(int16_t)(source_view[6+i]-(i>=2));
    for(unsigned i=0;i<3;++i)next.t[i]=0;
    const sh2z_vec3_t origin={0,0,0};
    if(!sh2z_view_place(&next,origin))return false;
    plain_base_view.focal_x=next.focal_x;plain_base_view.focal_y=next.focal_y;
    plain_base_view.center_x=next.center_x;plain_base_view.center_y=next.center_y;
    plain_base_view.near_z=next.near_z;plain_base_view.far_z=next.far_z;
    for(unsigned i=0;i<4;++i)plain_base_view.clip[i]=next.clip[i];
    for(unsigned i=0;i<3;++i){plain_base_view.t[i]=next.t[i];for(unsigned j=0;j<3;++j)plain_base_view.r[i][j]=next.r[i][j];}
    return true;
}
