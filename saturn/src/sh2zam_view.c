/* Integer view transform, projection and clip codes. Portable definition of
 * the view-contract-1 results. MIT; SH2ZAM Contributors. */
#include "sh2zam/view_experimental.h"
#include <limits.h>
#include <string.h>

_Static_assert(sizeof(sh2z_pos16_t)==6,"sh2z_pos16_t is 6 bytes");
_Static_assert(sizeof(sh2z_view_vertex_t)==20,"sh2z_view_vertex_t is 20 bytes");

static uint32_t magnitude32(int32_t a) {
    return a<0?(uint32_t)0-(uint32_t)a:(uint32_t)a;
}
static bool row_fits(const int16_t r[3],int32_t t) {
    uint64_t sum=0;
    for(unsigned j=0;j<3;++j)sum+=magnitude32(r[j]);
    return sum*UINT64_C(32768)+magnitude32(t)<=(uint64_t)INT32_MAX;
}
static bool view_valid(const sh2z_view_t *v) {
    if(!v || v->focal_x<=0 || v->focal_y<=0 || v->near_z<=0 || v->far_z<=v->near_z ||
       v->clip[0]>v->clip[2] || v->clip[1]>v->clip[3])return false;
    for(unsigned i=0;i<3;++i)if(!row_fits(v->r[i],v->t[i]))return false;
    return true;
}
bool sh2z_view_init(sh2z_view_t *out,const sh2z_view_t *in) {
    if(!out || !view_valid(in))return false;
    if(out!=in)memmove(out,in,sizeof(*out));
    return true;
}
static int64_t floor_shift16(int64_t n) {
    /* Floor division by 65536 without relying on signed right shift. */
    int64_t q=n/65536;
    if(n%65536!=0 && n<0)--q;
    return q;
}
bool sh2z_view_place(sh2z_view_t *v,sh2z_vec3_t eye) {
    if(!view_valid(v))return false;
    const int64_t e[3]={eye.x,eye.y,eye.z};
    int32_t t[3];
    for(unsigned i=0;i<3;++i) {
        int64_t sum=0;
        for(unsigned j=0;j<3;++j)sum+=(int64_t)v->r[i][j]*e[j];
        const int64_t value=-floor_shift16(sum);
        if(value<INT32_MIN || value>INT32_MAX)return false;
        t[i]=(int32_t)value;
        if(!row_fits(v->r[i],t[i]))return false;
    }
    memcpy(v->t,t,sizeof(t));
    return true;
}
static int32_t sat32(int64_t a) {
    return a<INT32_MIN?INT32_MIN:a>INT32_MAX?INT32_MAX:(int32_t)a;
}
static int16_t sat16(int32_t a) {
    return (int16_t)(a<INT16_MIN?INT16_MIN:a>INT16_MAX?INT16_MAX:a);
}
static int32_t shift16(int32_t q) {
    /* floor(q / 65536) without implementation-defined signed shifts:
     * for negative q, floor(q / 2^16) = -((~q) >> 16) - 1. */
    return q>=0?(int32_t)((uint32_t)q>>16):-(int32_t)(~(uint32_t)q>>16)-1;
}
static bool overlaps(const void *a,size_t a_bytes,const void *b,size_t b_bytes) {
    const uintptr_t x=(uintptr_t)a,y=(uintptr_t)b;
    return x<y+b_bytes && y<x+a_bytes;
}
bool sh2z_view_batch(sh2z_view_vertex_t *out,const sh2z_view_t *v,
    const void *positions,size_t in_stride,size_t count) {
    if(!view_valid(v))return false;
    if(!count)return true;
    if(!out || !positions || in_stride<6 || (in_stride&1u) || ((uintptr_t)positions&1u))return false;
    if(count>SIZE_MAX/sizeof(*out) || (count-1)>(SIZE_MAX-6)/in_stride)return false;
    const size_t out_bytes=count*sizeof(*out),in_bytes=(count-1)*in_stride+6;
    if((uintptr_t)out>UINTPTR_MAX-out_bytes || (uintptr_t)positions>UINTPTR_MAX-in_bytes ||
       overlaps(out,out_bytes,positions,in_bytes))return false;
    const sh2z_view_t c=*v;
    const unsigned char *p=positions;
    for(size_t k=0;k<count;++k,p+=in_stride) {
        int16_t in[3];
        memcpy(in,p,sizeof(in));
        int32_t view[3];
        for(unsigned i=0;i<3;++i)
            view[i]=(int32_t)c.r[i][0]*in[0]+(int32_t)c.r[i][1]*in[1]+(int32_t)c.r[i][2]*in[2]+c.t[i];
        sh2z_view_vertex_t o;
        memset(&o,0,sizeof(o));
        o.x=view[0];o.y=view[1];o.z=view[2];
        if(view[2]<c.near_z) {
            o.outcode=SH2Z_OUT_NEAR;
        } else {
            const int32_t qx=sat32((int64_t)c.focal_x*view[0]/view[2]);
            const int32_t qy=sat32((int64_t)c.focal_y*view[1]/view[2]);
            o.sx=sat16((int32_t)c.center_x+shift16(qx));
            o.sy=sat16((int32_t)c.center_y-shift16(qy));
            unsigned code=0;
            if(view[2]>c.far_z)code|=SH2Z_OUT_FAR;
            if(o.sx<c.clip[0])code|=SH2Z_OUT_LEFT;
            if(o.sx>c.clip[2])code|=SH2Z_OUT_RIGHT;
            if(o.sy<c.clip[1])code|=SH2Z_OUT_TOP;
            if(o.sy>c.clip[3])code|=SH2Z_OUT_BOTTOM;
            o.outcode=(uint8_t)code;
        }
        out[k]=o;
    }
    return true;
}
