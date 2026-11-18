/* view-round-1: the view projections with the Q16 screen position truncated
 * toward zero instead of the quotient floored. Same paths as sh2zam_view.c,
 * whose code this file shares through sh2zam_view_internal.h and
 * sh2zam_view_paths.h; a separate file so that the view-contract-1 functions
 * compile exactly as before.
 *
 * Copyright (c) 2026 SH2ZAM Contributors. MIT License.
 * Inspired by SH4ZAM by Falco Girgis and SH4ZAM Contributors.
 */
#include "sh2zam_view_internal.h"

/* Clamp to +/-(2^31 - 1), the symmetric quotient range of view-round-1. */
static int32_t sat31(int64_t a) {
    return a<-INT32_MAX?-INT32_MAX:a>INT32_MAX?INT32_MAX:(int32_t)a;
}
/* trunc((65536*c + q) / 65536): the floor c + (q >> 16), plus one when the
 * position is negative and q has fraction bits. |f| < 2^17: no overflow. */
static SH2ZI_ALWAYS_INLINE int16_t trunc_sx(int32_t c,int32_t q) {
    int32_t f=c+shift16(q);
    if(f<0 && ((uint32_t)q&0xFFFFu))++f;
    return sat16(f);
}
/* trunc((65536*c - q) / 65536): the ceiling c - (q >> 16), less one when the
 * position is positive and q has fraction bits. */
static SH2ZI_ALWAYS_INLINE int16_t trunc_sy(int32_t c,int32_t q) {
    int32_t g=c-shift16(q);
    if(g>0 && ((uint32_t)q&0xFFFFu))--g;
    return sat16(g);
}
static SH2ZI_ALWAYS_INLINE uint8_t classify(const sh2z_view_t *c,int16_t sx,int16_t sy,int32_t z) {
    unsigned code=0;
    if(z>c->far_z)code|=SH2Z_OUT_FAR;
    if(sx<c->clip[0])code|=SH2Z_OUT_LEFT;
    if(sx>c->clip[2])code|=SH2Z_OUT_RIGHT;
    if(sy<c->clip[1])code|=SH2Z_OUT_TOP;
    if(sy>c->clip[3])code|=SH2Z_OUT_BOTTOM;
    return (uint8_t)code;
}
static SH2ZI_ALWAYS_INLINE sh2z_view_vertex_t project_point(const sh2z_view_t *c,int32_t x,int32_t y,int32_t z) {
    sh2z_view_vertex_t o={x,y,z,0,0,0,{0,0,0}};
    if(z<c->near_z) {
        o.outcode=SH2Z_OUT_NEAR;
        return o;
    }
    o.sx=trunc_sx(c->center_x,sat31((int64_t)c->focal_x*x/z));
    o.sy=trunc_sy(c->center_y,sat31((int64_t)c->focal_y*y/z));
    o.outcode=classify(c,o.sx,o.sy,z);
    return o;
}
static sh2z_view_vertex_t project(const sh2z_view_t *c,int32_t x,int32_t y,int32_t z) {
    return project_point(c,x,y,z);
}
static void batch_portable(sh2z_view_vertex_t *out,const sh2z_view_t *v,
    const void *positions,size_t in_stride,size_t count) {
    const sh2z_view_t c=*v;
    const unsigned char *p=positions;
    for(size_t k=0;k<count;++k,p+=in_stride) {
        /* positions and in_stride are 2-byte aligned by contract. */
        const int16_t *in=(const int16_t *)(const void *)p;
        int32_t view[3];
        for(unsigned i=0;i<3;++i)
            view[i]=(int32_t)c.r[i][0]*in[0]+(int32_t)c.r[i][1]*in[1]+(int32_t)c.r[i][2]*in[2]+c.t[i];
        out[k]=project_point(&c,view[0],view[1],view[2]);
    }
}
static void project_portable(sh2z_view_vertex_t *out,const sh2z_view_t *v,
    const sh2z_view_vertex_t *in,size_t count) {
    const sh2z_view_t c=*v;
    for(size_t k=0;k<count;++k) {
        const int32_t x=in[k].x,y=in[k].y,z=in[k].z;
        out[k]=project_point(&c,x,y,z);
    }
}
#if SH2Z_DIVU_NATIVE
/* project() through this CPU's divider. The divu helpers answer quotients
 * that do not fit with sat32; only INT32_MIN differs from the symmetric
 * clamp. */
static sh2z_view_vertex_t project_divu(const sh2z_view_t *c,int32_t x,int32_t y,int32_t z) {
    sh2z_view_vertex_t o={x,y,z,0,0,0,{0,0,0}};
    if(z<c->near_z) {
        o.outcode=SH2Z_OUT_NEAR;
        return o;
    }
    const uint64_t nx=(uint64_t)((int64_t)c->focal_x*x);
    sh2z_divu_t t;
    sh2z_divu_start64(&t,(int32_t)(uint32_t)(nx>>32),(uint32_t)nx,z);
    const uint64_t ny=(uint64_t)((int64_t)c->focal_y*y);
    int32_t qx=sh2z_divu_finish(&t,NULL);
    sh2z_divu_start64(&t,(int32_t)(uint32_t)(ny>>32),(uint32_t)ny,z);
    if(qx==INT32_MIN)qx=-INT32_MAX;
    o.sx=trunc_sx(c->center_x,qx);
    int32_t qy=sh2z_divu_finish(&t,NULL);
    if(qy==INT32_MIN)qy=-INT32_MAX;
    o.sy=trunc_sy(c->center_y,qy);
    o.outcode=classify(c,o.sx,o.sy,z);
    return o;
}
static void batch_divu_native(sh2z_view_vertex_t *out,const sh2z_view_t *v,
    const void *positions,size_t in_stride,size_t count) {
    if(in_stride-6u>(size_t)INT16_MAX) {
        /* The assembly loop keeps the stride step in 16 bits. */
        batch_portable(out,v,positions,in_stride,count);
        return;
    }
    divu_block_t b;
    fill_block(&b,v,(int16_t)(in_stride-6u));
    sh2zi_view_divu_run_trunc(out,&b,positions,count);
}
static void project_divu_native(sh2z_view_vertex_t *out,const sh2z_view_t *v,
    const sh2z_view_vertex_t *in,size_t count) {
    divu_block_t b;
    fill_block(&b,v,0);
    sh2zi_view_divu_project_trunc(out,&b,in,count);
}
#define PROJECT_DIVU project_divu
#else
#define PROJECT_DIVU project
#endif

#define SH2ZI_VIEW_POINT project
#define SH2ZI_VIEW_POINT_DIVU project_divu
#define SH2ZI_VIEW_RANGE project_portable
#define SH2ZI_VIEW_RANGE_DIVU project_divu_native
#include "sh2zam_view_paths.h"

bool sh2z_view_batch_trunc(sh2z_view_vertex_t *out,const sh2z_view_t *v,
    const void *positions,size_t in_stride,size_t count) {
    if(!view_valid(v))return false;
    if(!count)return true;
    if(!batch_args_ok(out,positions,in_stride,count))return false;
    batch_portable(out,v,positions,in_stride,count);
    return true;
}
bool sh2z_view_batch_trunc_divu(sh2z_view_vertex_t *out,const sh2z_view_t *v,
    const void *positions,size_t in_stride,size_t count) {
    if(!view_valid(v))return false;
    if(!count)return true;
    if(!batch_args_ok(out,positions,in_stride,count))return false;
#if SH2Z_DIVU_NATIVE
    batch_divu_native(out,v,positions,in_stride,count);
#else
    batch_portable(out,v,positions,in_stride,count);
#endif
    return true;
}
bool sh2z_view_project_trunc(sh2z_view_vertex_t *out,const sh2z_view_t *v,
    const sh2z_view_vertex_t *in,size_t count) {
    if(!view_valid(v))return false;
    if(!count)return true;
    if(!project_args_ok(out,in,count))return false;
    project_portable(out,v,in,count);
    return true;
}
bool sh2z_view_project_trunc_divu(sh2z_view_vertex_t *out,const sh2z_view_t *v,
    const sh2z_view_vertex_t *in,size_t count) {
    if(!view_valid(v))return false;
    if(!count)return true;
    if(!project_args_ok(out,in,count))return false;
#if SH2Z_DIVU_NATIVE
    project_divu_native(out,v,in,count);
#else
    project_portable(out,v,in,count);
#endif
    return true;
}
bool sh2z_view_edge_steps_trunc(sh2z_view_vertex_t *out,const sh2z_view_t *v,
    const sh2z_view_vertex_t *from,const sh2z_view_vertex_t *to,
    uint16_t first_num,uint16_t step_num,uint16_t den,size_t count) {
    return edge_steps(out,v,from,to,first_num,step_num,den,count,false);
}
bool sh2z_view_edge_steps_trunc_divu(sh2z_view_vertex_t *out,const sh2z_view_t *v,
    const sh2z_view_vertex_t *from,const sh2z_view_vertex_t *to,
    uint16_t first_num,uint16_t step_num,uint16_t den,size_t count) {
    return edge_steps(out,v,from,to,first_num,step_num,den,count,true);
}
unsigned sh2z_view_clip_near_trunc(sh2z_view_vertex_t out[6],const sh2z_view_t *v,
    const sh2z_view_vertex_t in[4]) {
    return clip_near(out,v,in,project,false);
}
unsigned sh2z_view_clip_near_trunc_divu(sh2z_view_vertex_t out[6],const sh2z_view_t *v,
    const sh2z_view_vertex_t in[4]) {
    return clip_near(out,v,in,PROJECT_DIVU,true);
}
int sh2z_view_clip_guard_trunc(sh2z_view_guard_vertex_t out[SH2Z_VIEW_GUARD_MAX_OUT],
    const sh2z_view_t *v,const int16_t guard[4],
    const sh2z_view_guard_vertex_t *in,unsigned count,unsigned attrs) {
    return clip_guard(out,v,guard,in,count,attrs,false);
}
int sh2z_view_clip_guard_trunc_divu(sh2z_view_guard_vertex_t out[SH2Z_VIEW_GUARD_MAX_OUT],
    const sh2z_view_t *v,const int16_t guard[4],
    const sh2z_view_guard_vertex_t *in,unsigned count,unsigned attrs) {
    return clip_guard(out,v,guard,in,count,attrs,true);
}
