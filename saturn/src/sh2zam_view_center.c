/* view-center-1: the view-round-1 projections with a centre that has 16
 * fraction bits. A separate file, so that the view-contract-1 and
 * view-round-1 functions compile exactly as before; it shares their argument
 * rules, stepper axes, near clip and guard passes through
 * sh2zam_view_internal.h and their SH-2 assembly macros through
 * sh2zam_view_asm.h. MIT; SH2ZAM Contributors. */
#include "sh2zam_view_internal.h"

_Static_assert(offsetof(sh2z_view_center_t,center_x16)==60,"center_x16 offset");
_Static_assert(sizeof(sh2z_view_center_t)==68,"sh2z_view_center_t is 68 bytes");

#include "sh2zam_view_center_internal.h"
/* Out of line, as in sh2zam_view_round.c, where the near clip's function
 * pointer keeps it so: the guard clip calls it a few times per polygon. */
#if defined(__GNUC__)
#define CENTER_NOINLINE __attribute__((noinline))
#else
#define CENTER_NOINLINE
#endif
static CENTER_NOINLINE sh2z_view_vertex_t project(const sh2z_view_center_t *c,int32_t x,int32_t y,int32_t z) {
    return project_point(c,x,y,z);
}
/* The near clip's projector type takes a sh2z_view_t; it is always given
 * &w->view of a centre view, the struct's first member, so the cast back to
 * the containing struct is well defined. */
static const sh2z_view_center_t *container(const sh2z_view_t *v) {
    return (const sh2z_view_center_t *)(const void *)v;
}
static sh2z_view_vertex_t project_near(const sh2z_view_t *v,int32_t x,int32_t y,int32_t z) {
    return project(container(v),x,y,z);
}
static void batch_portable(sh2z_view_vertex_t *out,const sh2z_view_center_t *v,
    const void *positions,size_t in_stride,size_t count) {
    const sh2z_view_center_t c=*v;
    const unsigned char *p=positions;
    for(size_t k=0;k<count;++k,p+=in_stride) {
        /* positions and in_stride are 2-byte aligned by contract. */
        const int16_t *in=(const int16_t *)(const void *)p;
        int32_t view[3];
        for(unsigned i=0;i<3;++i)
            view[i]=(int32_t)c.view.r[i][0]*in[0]+(int32_t)c.view.r[i][1]*in[1]+(int32_t)c.view.r[i][2]*in[2]+c.view.t[i];
        out[k]=project_point(&c,view[0],view[1],view[2]);
    }
}
static void project_portable(sh2z_view_vertex_t *out,const sh2z_view_center_t *v,
    const sh2z_view_vertex_t *in,size_t count) {
    const sh2z_view_center_t c=*v;
    for(size_t k=0;k<count;++k) {
        const int32_t x=in[k].x,y=in[k].y,z=in[k].z;
        out[k]=project_point(&c,x,y,z);
    }
}
#if SH2Z_DIVU_NATIVE
/* project() through this CPU's divider. The divu helpers answer quotients
 * that do not fit with sat32; only INT32_MIN differs from the symmetric
 * clamp. */
static CENTER_NOINLINE sh2z_view_vertex_t project_divu(const sh2z_view_center_t *c,int32_t x,int32_t y,int32_t z) {
    sh2z_view_vertex_t o={x,y,z,0,0,0,{0,0,0}};
    if(z<c->view.near_z) {
        o.outcode=SH2Z_OUT_NEAR;
        return o;
    }
    const uint64_t nx=(uint64_t)((int64_t)c->view.focal_x*x);
    sh2z_divu_t t;
    sh2z_divu_start64(&t,(int32_t)(uint32_t)(nx>>32),(uint32_t)nx,z);
    const uint64_t ny=(uint64_t)((int64_t)c->view.focal_y*y);
    int32_t qx=sh2z_divu_finish(&t,NULL);
    sh2z_divu_start64(&t,(int32_t)(uint32_t)(ny>>32),(uint32_t)ny,z);
    if(qx==INT32_MIN)qx=-INT32_MAX;
    o.sx=center_sx(c->view.center_x,c->center_x16,qx);
    int32_t qy=sh2z_divu_finish(&t,NULL);
    if(qy==INT32_MIN)qy=-INT32_MAX;
    o.sy=center_sy(c->view.center_y,c->center_y16,qy);
    o.outcode=classify(&c->view,o.sx,o.sy,z);
    return o;
}
static sh2z_view_vertex_t project_near_divu(const sh2z_view_t *v,int32_t x,int32_t y,int32_t z) {
    return project_divu(container(v),x,y,z);
}
#define PROJECT_NEAR_DIVU project_near_divu

/* The SH-2 kernels of sh2zam_view.c instantiated with center=1 (see
 * sh2zam_view_asm.h): the camera block, then fx << 16 and fy << 16. */
typedef struct {
    divu_block_t b;
    uint32_t fx16,fy16;
} divu_block_center_t;
_Static_assert(offsetof(divu_block_center_t,fx16)==84,"block fx16");
void sh2zi_view_divu_run_center(sh2z_view_vertex_t *out,const divu_block_center_t *block,
    const void *positions,size_t count);
void sh2zi_view_divu_project_center(sh2z_view_vertex_t *out,const divu_block_center_t *block,
    const sh2z_view_vertex_t *in,size_t count);

__asm__(
#include "sh2zam_view_asm.h"
"    .pushsection .text.sh2zi_view_divu_run_center,\"ax\",@progbits\n"
"    .align 2\n"
"    .global _sh2zi_view_divu_run_center\n"
"    .type _sh2zi_view_divu_run_center,@function\n"
"_sh2zi_view_divu_run_center:\n"
"    SH2ZI_VD_PROLOGUE 1\n"
".Lsh2zi_vd_loopTC:\n"
"    SH2ZI_VD_TRANSFORM\n"
"    SH2ZI_VD_REST TC, 0, 1\n"
"    .size _sh2zi_view_divu_run_center,.-_sh2zi_view_divu_run_center\n"
"    .popsection\n"
"    .pushsection .text.sh2zi_view_divu_project_center,\"ax\",@progbits\n"
"    .align 2\n"
"    .global _sh2zi_view_divu_project_center\n"
"    .type _sh2zi_view_divu_project_center,@function\n"
"_sh2zi_view_divu_project_center:\n"
"    SH2ZI_VD_PROLOGUE 1\n"
".Lsh2zi_vd_loopPC:\n"
"    SH2ZI_VD_LOAD\n"
"    SH2ZI_VD_REST PC, 0, 1\n"
"    .size _sh2zi_view_divu_project_center,.-_sh2zi_view_divu_project_center\n"
"    .popsection\n"
);

/* fill_block takes the floor of the centre from view.center_x, center_y,
 * which a valid centre view guarantees. */
static void fill_center_block(divu_block_center_t *b,const sh2z_view_center_t *v,int16_t skip) {
    fill_block(&b->b,&v->view,skip);
    b->fx16=(uint32_t)v->center_x16<<16;
    b->fy16=(uint32_t)v->center_y16<<16;
}
static void batch_divu_native(sh2z_view_vertex_t *out,const sh2z_view_center_t *v,
    const void *positions,size_t in_stride,size_t count) {
    if(in_stride-6u>(size_t)INT16_MAX) {
        /* The assembly loop keeps the stride step in 16 bits. */
        batch_portable(out,v,positions,in_stride,count);
        return;
    }
    divu_block_center_t b;
    fill_center_block(&b,v,(int16_t)(in_stride-6u));
    sh2zi_view_divu_run_center(out,&b,positions,count);
}
static void project_divu_native(sh2z_view_vertex_t *out,const sh2z_view_center_t *v,
    const sh2z_view_vertex_t *in,size_t count) {
    divu_block_center_t b;
    fill_center_block(&b,v,0);
    sh2zi_view_divu_project_center(out,&b,in,count);
}
#else
#define PROJECT_NEAR_DIVU project_near
#endif

/* sh2zam_view_paths.h's edge stepper with the centre view. */
static bool edge_steps(sh2z_view_vertex_t *out,const sh2z_view_center_t *v,
    const sh2z_view_vertex_t *from,const sh2z_view_vertex_t *to,
    uint16_t first_num,uint16_t step_num,uint16_t den,size_t count,bool divu) {
    if(!center_valid(v))return false;
    if(!count)return true;
    if(!out || !from || !to || !den || first_num>den)return false;
    if(step_num && (count-1)>(size_t)((uint32_t)(den-first_num)/step_num))return false;
    if(count>SIZE_MAX/sizeof(*out))return false;
    /* from and to are fully read here, before out is written. */
    edge_axis_t ex=edge_axis(from->x,to->x,first_num,step_num,den,divu);
    edge_axis_t ey=edge_axis(from->y,to->y,first_num,step_num,den,divu);
    edge_axis_t ez=edge_axis(from->z,to->z,first_num,step_num,den,divu);
    for(size_t k=0;k<count;++k) {
        out[k].x=edge_next(&ex);out[k].y=edge_next(&ey);out[k].z=edge_next(&ez);
    }
#if SH2Z_DIVU_NATIVE
    if(divu) {
        project_divu_native(out,v,out,count);
        return true;
    }
#endif
    project_portable(out,v,out,count);
    return true;
}

/* view-guard-1's plane constants with the centre in Q16:
 *     k = 65536 * edge - centre16      (left, right; top and bottom negate it)
 * and its edge rule, |k| <= 2^30, which keeps every plane distance inside
 * int64 (|focal * a| <= 2^62, |k * z| <= 2^61). In 32-bit arithmetic, with
 * centre16 = 65536 * whole + frac (0 <= frac < 65536) and h = edge - whole:
 * k = 65536 * h - frac, so |k| <= 2^30 exactly when
 * -16384 + (frac != 0) <= h <= 16384, and then k fits int32. */
static bool guard_k(int32_t *k,int32_t edge,int32_t whole,int32_t center16,bool negate) {
    (void)guard_edge_ok;   /* the integer-centre form of this rule, unused here */
    const int32_t h=edge-whole;
    const uint32_t frac=(uint32_t)center16&0xFFFFu;
    if(h>16384 || h<(frac?-16383:-16384))return false;
    const int32_t value=(int32_t)((uint32_t)h*65536u-frac);
    *k=negate?-value:value;
    return true;
}
/* sh2zam_view_paths.h's guard clip with the planes above. */
static int clip_guard(sh2z_view_guard_vertex_t *out,const sh2z_view_center_t *v,const int16_t guard[4],
    const sh2z_view_guard_vertex_t *in,unsigned count,unsigned attrs,bool divu) {
    if(!center_valid(v) || !out || !guard || !in || count<3u || count>SH2Z_VIEW_GUARD_MAX_IN ||
       attrs>SH2Z_VIEW_GUARD_ATTRS)return SH2Z_VIEW_GUARD_INVALID;
    const int16_t gl=guard[0],gt=guard[1],gr=guard[2],gb=guard[3];
    guard_planes_t g={v->view.near_z,v->view.far_z,v->view.focal_x,v->view.focal_y,0,0,0,0};
    const int32_t cx=v->view.center_x,cy=v->view.center_y;
    if(gl>gr || gt>gb || !guard_k(&g.kl,gl,cx,v->center_x16,false) || !guard_k(&g.kr,gr,cx,v->center_x16,false) ||
       !guard_k(&g.kt,gt,cy,v->center_y16,true) || !guard_k(&g.kb,gb,cy,v->center_y16,true))return SH2Z_VIEW_GUARD_INVALID;
    const unsigned coords=3u+attrs;
    guard_point_t buf[2][SH2Z_VIEW_GUARD_MAX_OUT];
    for(unsigned k=0;k<count;++k) {
        buf[0][k].c[0]=in[k].v.x;buf[0][k].c[1]=in[k].v.y;buf[0][k].c[2]=in[k].v.z;
        for(unsigned i=0;i<SH2Z_VIEW_GUARD_ATTRS;++i)buf[0][k].c[3+i]=i<attrs?in[k].attr[i]:0;
    }
    int n=(int)count;
    unsigned cur=0;
    for(unsigned plane=0;plane<6u;++plane) {
        int64_t d[SH2Z_VIEW_GUARD_MAX_OUT];
        const unsigned inside=guard_distances(d,buf[cur],(unsigned)n,&g,plane);
        if(inside==(unsigned)n)continue;
        if(!inside){n=0;break;}
        n=guard_pass(buf[cur^1u],buf[cur],(unsigned)n,d,plane,coords,divu);
        if(n<0)return SH2Z_VIEW_GUARD_OVERFLOW;
        cur^=1u;
    }
    const sh2z_view_center_t c=*v;
    for(int k=0;k<n;++k) {
        const guard_point_t *p=&buf[cur][k];
#if SH2Z_DIVU_NATIVE
        sh2z_view_vertex_t q=divu?project_divu(&c,p->c[0],p->c[1],p->c[2]):project(&c,p->c[0],p->c[1],p->c[2]);
#else
        sh2z_view_vertex_t q=project(&c,p->c[0],p->c[1],p->c[2]);
#endif
        if(q.sx<gl)q.sx=gl;
        if(q.sx>gr)q.sx=gr;
        if(q.sy<gt)q.sy=gt;
        if(q.sy>gb)q.sy=gb;
        q.outcode=(uint8_t)((q.sx<c.view.clip[0]?SH2Z_OUT_LEFT:0u)|(q.sx>c.view.clip[2]?SH2Z_OUT_RIGHT:0u)|
            (q.sy<c.view.clip[1]?SH2Z_OUT_TOP:0u)|(q.sy>c.view.clip[3]?SH2Z_OUT_BOTTOM:0u));
        out[k].v=q;
        for(unsigned i=0;i<SH2Z_VIEW_GUARD_ATTRS;++i)out[k].attr[i]=p->c[3+i];
    }
    return n;
}

bool sh2z_view_center_init(sh2z_view_center_t *out,const sh2z_view_t *view,
    int32_t center_x16,int32_t center_y16) {
    if(!out || !view_valid(view))return false;
    sh2z_view_t v=*view;
    v.center_x=(int16_t)shift16(center_x16);
    v.center_y=(int16_t)shift16(center_y16);
    out->view=v;
    out->center_x16=center_x16;
    out->center_y16=center_y16;
    return true;
}
bool sh2z_view_batch_center(sh2z_view_vertex_t *out,const sh2z_view_center_t *v,
    const void *positions,size_t in_stride,size_t count) {
    if(!center_valid(v))return false;
    if(!count)return true;
    if(!batch_args_ok(out,positions,in_stride,count))return false;
    batch_portable(out,v,positions,in_stride,count);
    return true;
}
bool sh2z_view_batch_center_divu(sh2z_view_vertex_t *out,const sh2z_view_center_t *v,
    const void *positions,size_t in_stride,size_t count) {
    if(!center_valid(v))return false;
    if(!count)return true;
    if(!batch_args_ok(out,positions,in_stride,count))return false;
#if SH2Z_DIVU_NATIVE
    batch_divu_native(out,v,positions,in_stride,count);
#else
    batch_portable(out,v,positions,in_stride,count);
#endif
    return true;
}
bool sh2z_view_project_center(sh2z_view_vertex_t *out,const sh2z_view_center_t *v,
    const sh2z_view_vertex_t *in,size_t count) {
    if(!center_valid(v))return false;
    if(!count)return true;
    if(!project_args_ok(out,in,count))return false;
    project_portable(out,v,in,count);
    return true;
}
bool sh2z_view_project_center_divu(sh2z_view_vertex_t *out,const sh2z_view_center_t *v,
    const sh2z_view_vertex_t *in,size_t count) {
    if(!center_valid(v))return false;
    if(!count)return true;
    if(!project_args_ok(out,in,count))return false;
#if SH2Z_DIVU_NATIVE
    project_divu_native(out,v,in,count);
#else
    project_portable(out,v,in,count);
#endif
    return true;
}
bool sh2z_view_edge_steps_center(sh2z_view_vertex_t *out,const sh2z_view_center_t *v,
    const sh2z_view_vertex_t *from,const sh2z_view_vertex_t *to,
    uint16_t first_num,uint16_t step_num,uint16_t den,size_t count) {
    return edge_steps(out,v,from,to,first_num,step_num,den,count,false);
}
bool sh2z_view_edge_steps_center_divu(sh2z_view_vertex_t *out,const sh2z_view_center_t *v,
    const sh2z_view_vertex_t *from,const sh2z_view_vertex_t *to,
    uint16_t first_num,uint16_t step_num,uint16_t den,size_t count) {
    return edge_steps(out,v,from,to,first_num,step_num,den,count,true);
}
unsigned sh2z_view_clip_near_center(sh2z_view_vertex_t out[6],const sh2z_view_center_t *v,
    const sh2z_view_vertex_t in[4]) {
    if(!center_valid(v))return 0;
    return clip_near(out,&v->view,in,project_near,false);
}
unsigned sh2z_view_clip_near_center_divu(sh2z_view_vertex_t out[6],const sh2z_view_center_t *v,
    const sh2z_view_vertex_t in[4]) {
    if(!center_valid(v))return 0;
    return clip_near(out,&v->view,in,PROJECT_NEAR_DIVU,true);
}
int sh2z_view_clip_guard_center(sh2z_view_guard_vertex_t out[SH2Z_VIEW_GUARD_MAX_OUT],
    const sh2z_view_center_t *v,const int16_t guard[4],
    const sh2z_view_guard_vertex_t *in,unsigned count,unsigned attrs) {
    return clip_guard(out,v,guard,in,count,attrs,false);
}
int sh2z_view_clip_guard_center_divu(sh2z_view_guard_vertex_t out[SH2Z_VIEW_GUARD_MAX_OUT],
    const sh2z_view_center_t *v,const int16_t guard[4],
    const sh2z_view_guard_vertex_t *in,unsigned count,unsigned attrs) {
    return clip_guard(out,v,guard,in,count,attrs,true);
}
