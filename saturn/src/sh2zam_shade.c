/* Experimental integer shading kernels (contract tag shade-1).
 * MIT; SH2ZAM Contributors. */
#include "sh2zam/shade_experimental.h"

static bool overlaps(const void *a,size_t a_bytes,const void *b,size_t b_bytes) {
    const uintptr_t x=(uintptr_t)a,y=(uintptr_t)b;
    return x<y+b_bytes && y<x+a_bytes;
}
/* Bytes spanned by count records of size bytes at stride; 0 on overflow
 * (count > 0 here). Counts and strides below 2^16 need no division (the SH-2
 * has no divide instruction; argument checks stay cheap for short calls). */
static size_t span(size_t count,size_t stride,size_t size) {
    const size_t n=count-1u;
    if(!((n|stride)>>16))return n*stride<=SIZE_MAX-size?n*stride+size:0;
    if(n>(SIZE_MAX-size)/stride)return 0;
    return n*stride+size;
}
static bool span_ok(const void *p,size_t bytes) {
    return bytes && (uintptr_t)p<=UINTPTR_MAX-bytes;
}

/* ---- light ---- */

bool sh2z_shade_light_prepare(sh2z_shade_light_t *out,sh2z_vec3_t light,
    sh2z_vec3_t reflection,const int16_t basis[3][3],const uint8_t ambient[3],
    const uint8_t color[3]) {
    if(!out || !basis || !ambient || !color)return false;
    for(unsigned i=0;i<3;++i)for(unsigned j=0;j<3;++j)
        if(basis[i][j]< -16384 || basis[i][j]>16384)return false;
    const sh2z_vec3_t l=sh2z_vec3_normalize(light),r=sh2z_vec3_normalize(reflection);
    if((!l.x && !l.y && !l.z) || (!r.x && !r.y && !r.z))return false;
    const int32_t lq[3]={l.x/4,l.y/4,l.z/4},rq[3]={r.x/4,r.y/4,r.z/4};
    int32_t a[3],b[3];
    for(unsigned j=0;j<3;++j) {
        a[j]=b[j]=0;
        for(unsigned k=0;k<3;++k){a[j]+=basis[k][j]*lq[k];b[j]+=basis[k][j]*rq[k];}
    }
    /* Field by field: no aggregate copy (freestanding builds have no memcpy). */
    for(unsigned j=0;j<3;++j) {
        out->light[j]=a[j]/16384;out->reflection[j]=b[j]/16384;
        out->ambient[j]=ambient[j];out->color[j]=color[j];
        out->scale[j]=(uint32_t)((((uint64_t)color[j]<<29)+126u)/127u);
    }
    out->reserved[0]=out->reserved[1]=0;
    return true;
}

/* d * c / D for 0 <= d <= D = 127 * 16384, c <= 255, with K = ceil(c * 2^43 / D)
 * = ceil(c * 2^29 / 127): d * K / 2^43 = d * c / D + d * e / 2^43 with
 * 0 <= e < 1, and d * e / 2^43 < D / 2^43 < 1 / D, smaller than the gap from
 * any non-integer d * c / D up to the next integer. So the floor is exact: one
 * 32 x 32 -> 64 multiply (dmulu.l on SH-2). Exhaustively checked by
 * tests/test_shade.c for every d and every c. */
static uint32_t light_scale(uint32_t d,uint32_t k) {
    return (uint32_t)(((uint64_t)d*k)>>43);
}

bool sh2z_shade_light_normals(uint8_t *out,size_t out_stride,const int8_t *normals,
    size_t normal_stride,size_t count,const sh2z_shade_light_t *light,
    const uint8_t reflection_lut[256]) {
    if(!count)return true;
    if(!out || !normals || !light || !reflection_lut || normal_stride<3 || out_stride<4)return false;
    const size_t out_bytes=span(count,out_stride,4),in_bytes=span(count,normal_stride,3);
    if(!span_ok(out,out_bytes) || !span_ok(normals,in_bytes) ||
       overlaps(out,out_bytes,normals,in_bytes) || overlaps(out,out_bytes,light,sizeof(*light)) ||
       overlaps(out,out_bytes,reflection_lut,256))return false;
    const int32_t lx=light->light[0],ly=light->light[1],lz=light->light[2];
    const int32_t rx=light->reflection[0],ry=light->reflection[1],rz=light->reflection[2];
    const uint32_t a0=light->ambient[0],a1=light->ambient[1],a2=light->ambient[2];
    const uint32_t c0=light->scale[0],c1=light->scale[1],c2=light->scale[2];
    const uint8_t *n=(const uint8_t *)normals;
    for(size_t i=0;i<count;++i,n+=normal_stride,out+=out_stride) {
        const int32_t x=(int8_t)n[0],y=(int8_t)n[1],z=(int8_t)n[2];
        const int32_t diffuse=x*lx+y*ly+z*lz,reflected=x*rx+y*ry+z*rz;
        uint32_t v0=a0,v1=a1,v2=a2;
        if(diffuse>0) {
            const uint32_t d=diffuse>127*16384?127u*16384u:(uint32_t)diffuse;
            v0+=light_scale(d,c0);v1+=light_scale(d,c1);v2+=light_scale(d,c2);
        }
        out[0]=(uint8_t)(v0>255u?255u:v0);
        out[1]=(uint8_t)(v1>255u?255u:v1);
        out[2]=(uint8_t)(v2>255u?255u:v2);
        /* reflected / 16384 truncates: shift the magnitude (the SH-2 has no
         * arithmetic shift by a count, only logical shifts by 1, 2, 8, 16). */
        int32_t index=reflected>=0?(int32_t)((uint32_t)reflected>>14):-(int32_t)((0u-(uint32_t)reflected)>>14);
        if(index< -128)index=-128;
        if(index>127)index=127;
        out[3]=reflection_lut[index+128];
    }
    return true;
}

/* ---- Gouraud ---- */

static uint32_t round255(uint32_t x) {return (x+(x>>8))>>8;}
/* Every factor is at most 255: 16-bit operands give the SH-2 its 16 x 16
 * multiply (mulu.w) instead of the 32-bit one. */
static uint32_t mul16(uint32_t a,uint32_t b) {return (uint32_t)(uint16_t)a*(uint16_t)b;}
static uint32_t lit_channel(uint32_t b,uint32_t l,uint32_t f,uint32_t t) {
    const uint32_t m=round255(mul16(b,l)+128u);
    return round255(mul16(m,t)+mul16(f,255u-t)+128u);
}

bool sh2z_shade_gouraud(uint16_t *out,uint32_t *saturated,const sh2z_shade_pair_t *pairs,
    size_t count,const uint8_t *base,size_t base_stride,const uint8_t *light,
    size_t light_stride,const uint8_t fog[3]) {
    if(!count){if(saturated)*saturated=0;return true;}
    if(!out || !pairs || !base || !light || !fog || base_stride<3 || light_stride<4 ||
       count>SIZE_MAX/sizeof(*pairs))return false;
    const size_t out_bytes=count*sizeof(*out);
    if(!span_ok(out,out_bytes) || !span_ok(pairs,count*sizeof(*pairs)) ||
       overlaps(out,out_bytes,pairs,count*sizeof(*pairs)) || overlaps(out,out_bytes,fog,3))return false;
    const uint32_t f0=fog[0],f1=fog[1],f2=fog[2];
    uint32_t clamped=0;
    for(size_t i=0;i<count;++i) {
        const uint8_t *b=base+(size_t)pairs[i].base*base_stride;
        const uint8_t *l=light+(size_t)pairs[i].light*light_stride;
        const uint32_t t=255u-l[3];
        /* Channels written out so every shift count is a constant. */
        int32_t v0=16+(int32_t)(lit_channel(b[0],l[0],f0,t)>>3)-(int32_t)(b[0]>>4u);
        int32_t v1=16+(int32_t)(lit_channel(b[1],l[1],f1,t)>>3)-(int32_t)(b[1]>>4u);
        int32_t v2=16+(int32_t)(lit_channel(b[2],l[2],f2,t)>>3)-(int32_t)(b[2]>>4u);
        if(v0<0){v0=0;++clamped;}else if(v0>31){v0=31;++clamped;}
        if(v1<0){v1=0;++clamped;}else if(v1>31){v1=31;++clamped;}
        if(v2<0){v2=0;++clamped;}else if(v2>31){v2=31;++clamped;}
        out[i]=(uint16_t)((uint32_t)v0|((uint32_t)v1<<5)|((uint32_t)v2<<10));
    }
    if(saturated)*saturated=clamped;
    return true;
}

uint16_t sh2z_shade_solid(const uint8_t base[3],const uint8_t light[4],const uint8_t fog[3]) {
    if(!base || !light || !fog)return 0;
    const uint32_t t=255u-light[3];
    return (uint16_t)(0x8000u|(lit_channel(base[0],light[0],fog[0],t)>>3)|
        ((lit_channel(base[1],light[1],fog[1],t)>>3)<<5)|((lit_channel(base[2],light[2],fog[2],t)>>3)<<10));
}

/* ---- culling ---- */

static uint32_t mag32(int32_t a) {return a<0?(uint32_t)0-(uint32_t)a:(uint32_t)a;}

bool sh2z_shade_planes_valid(const void *planes,size_t stride,size_t count,
    const uint16_t *order,size_t order_count) {
    if(count>65536u)return false;
    if(count) {
        if(!planes || stride<16 || (stride&3u) || ((uintptr_t)planes&3u) ||
           !span_ok(planes,span(count,stride,16)))return false;
        const char *p=(const char *)planes;
        for(size_t i=0;i<count;++i,p+=stride) {
            const sh2z_shade_plane_t *q=(const sh2z_shade_plane_t *)(const void *)p;
            for(unsigned k=0;k<3;++k)
                if(mag32(q->normal[k])>SH2Z_SHADE_NORMAL_MAX || mag32(q->first[k])>SH2Z_SHADE_FIRST_MAX)return false;
            if(mag32(q->plane)>(uint32_t)SH2Z_SHADE_PLANE_MAX)return false;
        }
    }
    if(order_count && !order)return false;
    for(size_t i=0;i<order_count;++i)if(order[i]>=count)return false;
    return true;
}

bool sh2z_shade_cull(uint16_t *kept,size_t *kept_count,const uint16_t *order,size_t count,
    const void *planes,size_t stride,const int32_t eye[3],uint32_t eye_error) {
    if(!kept_count)return false;
    *kept_count=0;
    if(!count)return true;
    if(!kept || !planes || !eye || count>65536u || stride<16 || (stride&3u) ||
       ((uintptr_t)planes&3u) || eye_error>SH2Z_SHADE_ERROR_MAX)return false;
    for(unsigned k=0;k<3;++k)if(mag32(eye[k])>SH2Z_SHADE_EYE_MAX)return false;
    const size_t kept_bytes=count*sizeof(*kept);
    if(!span_ok(kept,kept_bytes) || overlaps(kept,kept_bytes,eye,12) ||
       (order && order!=kept && overlaps(kept,kept_bytes,order,kept_bytes)))return false;
    /* Within the ranges of sh2z_shade_planes_valid every term below is under
     * 2^31 in magnitude; unsigned arithmetic keeps other inputs defined. */
    const uint32_t ex=(uint32_t)eye[0],ey=(uint32_t)eye[1],ez=(uint32_t)eye[2];
    const int32_t sx=eye[0],sy=eye[1],sz=eye[2];
    const char *base=(const char *)planes;
    size_t n=0;
    for(size_t i=0;i<count;++i) {
        const size_t id=order?order[i]:i;
        const sh2z_shade_plane_t *p=(const sh2z_shade_plane_t *)(const void *)(base+id*stride);
        const int32_t nx=p->normal[0],ny=p->normal[1],nz=p->normal[2];
        const uint32_t dot=(uint32_t)nx*ex+(uint32_t)ny*ey+(uint32_t)nz*ez-(uint32_t)p->plane;
        const uint32_t margin=3u+mag32(sx-p->first[0])+mag32(sy-p->first[1])+mag32(sz-p->first[2])+
            (mag32(nx)+mag32(ny)+mag32(nz))*eye_error;
        if(dot+margin<0x80000000u)kept[n++]=(uint16_t)id;
    }
    *kept_count=n;
    return true;
}

/* ---- fog ---- */

static uint8_t search(const uint32_t depth[256],uint32_t z,unsigned lo,unsigned hi) {
    /* max i in [lo, hi] with depth[i] <= z, given depth[lo] <= z. */
    while(lo<hi) {
        const unsigned mid=(lo+hi+1u)>>1;
        if(depth[mid]<=z)lo=mid;else hi=mid-1u;
    }
    return (uint8_t)lo;
}

bool sh2z_shade_fog_from_depth(sh2z_shade_fog_t *out,const uint32_t depth[256]) {
    if(!out || !depth || depth[0])return false;
    if(depth!=out->depth && overlaps(depth,256u*sizeof(*depth),out,sizeof(*out)))return false;
    for(unsigned i=1;i<256;++i)if(depth[i]<depth[i-1])return false;
    unsigned shift=0;
    while((depth[255]>>shift)>255u)++shift;
    /* Element by element: no aggregate copy (freestanding builds have no memcpy). */
    volatile uint32_t *d=out->depth;
    for(unsigned i=0;i<256;++i)d[i]=depth[i];
    /* bucket[b] = max i with depth[i] <= b << shift, in one merge pass. */
    const uint32_t step=(uint32_t)1u<<shift;
    uint32_t z=0;
    unsigned i=0;
    for(unsigned b=0;b<256;++b,z+=step) {
        while(i<255u && out->depth[i+1u]<=z)++i;
        out->bucket[b]=(uint8_t)i;
    }
    out->bucket[256]=255;
    out->shift=(uint8_t)shift;
    out->reserved[0]=out->reserved[1]=0;
    return true;
}

bool sh2z_shade_fog_build(sh2z_shade_fog_t *out,int32_t multiplier,int32_t offset,
    int32_t pa,int32_t pb,int32_t far_z) {
    if(!out || multiplier<=0 || pb>0 || far_z<=0)return false;
    uint32_t depth[256];
    const uint64_t numerator=(uint64_t)(-(int64_t)pb)*UINT64_C(1073741824);
    const int64_t one=INT64_C(1073741824);
    /* R(i) = ceil(n(i) / multiplier) with n(i) = (i - offset) * 2^30, stepped
     * exactly: n(i) = q * multiplier + r with 0 <= r < multiplier, and
     * 2^30 = step_q * multiplier + step_r, so moving to i + 1 adds step_q and
     * step_r with one carry. One 64-bit division in all, not two per entry. */
    const uint32_t step_q=UINT32_C(1073741824)/(uint32_t)multiplier,step_r=UINT32_C(1073741824)%(uint32_t)multiplier;
    const int64_t first=((int64_t)1-offset)*one;
    int64_t q=first/multiplier,r=first-q*multiplier;
    if(r<0){r+=multiplier;--q;}
    depth[0]=0;
    for(unsigned i=1;i<256;++i) {
        const int64_t required=q+(r>0);
        r+=step_r;q+=step_q;
        if(r>=multiplier){r-=multiplier;++q;}
        uint32_t d=(uint32_t)far_z+1u;
        if(required<=-one)d=0;
        else if(required<=one) {
            const int64_t m=(int64_t)pa-required;
            /* N / (m + 1) + 1 <= F (F = far_z / 4) exactly when N < F * (m + 1):
             * divide only then (m + 1 < 2^32 and F < 2^29, so no overflow). */
            if(m>=0 && numerator<(uint64_t)((uint32_t)far_z/4u)*((uint64_t)m+1u))
                d=(uint32_t)(numerator/((uint64_t)m+1u)+1u)*4u;
        }
        depth[i]=d;
    }
    return sh2z_shade_fog_from_depth(out,depth);
}

bool sh2z_shade_fog_lookup(uint8_t *fade,size_t fade_stride,const void *z,size_t z_stride,
    size_t count,const sh2z_shade_fog_t *fog) {
    if(!count)return true;
    if(!fade || !z || !fog || !fade_stride || !z_stride || (z_stride&3u) || ((uintptr_t)z&3u))return false;
    const size_t out_bytes=span(count,fade_stride,1),in_bytes=span(count,z_stride,4);
    if(!span_ok(fade,out_bytes) || !span_ok(z,in_bytes) || overlaps(fade,out_bytes,fog,sizeof(*fog)))return false;
    /* Bytes written must not be z bytes that are read (a fade field inside the
     * same records is fine). */
    if(overlaps(fade,out_bytes,z,in_bytes)) {
        const uintptr_t zb=(uintptr_t)z;
        for(size_t i=0;i<count;++i) {
            const uintptr_t w=(uintptr_t)fade+i*fade_stride;
            if(w>=zb && w<zb+in_bytes && (w-zb)%z_stride<4u)return false;
        }
    }
    const char *p=(const char *)z;
    /* u >> shift as the high word of u * 2^(32 - shift): one 32x32 multiply
     * instead of a shift by a variable count, which the SH-2 lacks. */
    const unsigned shift=fog->shift;
    const uint32_t scale=shift?(uint32_t)1u<<(32u-shift):0u;
    for(size_t i=0;i<count;++i,p+=z_stride,fade+=fade_stride) {
        const int32_t v=*(const int32_t *)(const void *)p;
        const uint32_t u=v>0?(uint32_t)v:0u;
        const uint32_t b=shift?(uint32_t)(((uint64_t)u*scale)>>32):u;
        if(b>255u){*fade=255;continue;}
        const unsigned lo=fog->bucket[b],hi=fog->bucket[b+1u];
        *fade=lo==hi?(uint8_t)lo:search(fog->depth,u,lo,hi);
    }
    return true;
}

/* ---- fade to 5:5:5 ---- */

static bool fade_args(const uint16_t *out,size_t out_stride,const uint8_t *fade,
    size_t fade_stride,size_t count,const void *p1,const void *p2,size_t param_bytes) {
    if(!out || !fade || !p1 || !p2 || !fade_stride || !out_stride || (out_stride&1u) ||
       ((uintptr_t)out&1u))return false;
    const size_t out_bytes=span(count,out_stride,2),in_bytes=span(count,fade_stride,1);
    return span_ok(out,out_bytes) && span_ok(fade,in_bytes) &&
        !overlaps(out,out_bytes,fade,in_bytes) && !overlaps(out,out_bytes,p1,param_bytes) &&
        !overlaps(out,out_bytes,p2,param_bytes);
}
/* trunc(x / 255) for |x| < 65536: floor(|x| / 255) == (|x| * 257 + 257) >> 16,
 * a 16 x 16 multiply. Exhaustively checked by tests/test_shade.c. */
static int32_t div255(int32_t x) {
    return x<0?-(int32_t)((mul16((uint32_t)-x,257u)+257u)>>16):(int32_t)((mul16((uint32_t)x,257u)+257u)>>16);
}

bool sh2z_shade_fade_offset(uint16_t *out,size_t out_stride,const uint8_t *fade,
    size_t fade_stride,size_t count,const int16_t slope[3],const int16_t bias[3]) {
    if(!count)return true;
    if(!fade_args(out,out_stride,fade,fade_stride,count,slope,bias,6))return false;
    for(unsigned k=0;k<3;++k)if(slope[k]< -127 || slope[k]>127 || bias[k]< -32767)return false;
    const int32_t s0=slope[0],s1=slope[1],s2=slope[2],b0=bias[0],b1=bias[1],b2=bias[2];
    char *o=(char *)out;
    for(size_t i=0;i<count;++i,fade+=fade_stride,o+=out_stride) {
        const int32_t f=*fade;
        int32_t v0=16+div255(f*s0-b0),v1=16+div255(f*s1-b1),v2=16+div255(f*s2-b2);
        v0=v0<0?0:v0>31?31:v0;
        v1=v1<0?0:v1>31?31:v1;
        v2=v2<0?0:v2>31?31:v2;
        *(uint16_t *)(void *)o=(uint16_t)(0x8000u|(uint32_t)v0|((uint32_t)v1<<5)|((uint32_t)v2<<10));
    }
    return true;
}

/* x / 65025 for x <= 2048287 (31 * 65025 + 32512): (x * 2113633) >> 37.
 * Exhaustively checked by tests/test_shade.c. */
static uint32_t div65025(uint32_t x) {
    return (uint32_t)(((uint64_t)x*2113633u)>>37);
}

bool sh2z_shade_fade_mix(uint16_t *out,size_t out_stride,const uint8_t *fade,
    size_t fade_stride,size_t count,const uint8_t near_rgb[3],const uint8_t far_rgb[3]) {
    if(!count)return true;
    if(!fade_args(out,out_stride,fade,fade_stride,count,near_rgb,far_rgb,3))return false;
    const uint32_t n0=near_rgb[0],n1=near_rgb[1],n2=near_rgb[2],f0=far_rgb[0],f1=far_rgb[1],f2=far_rgb[2];
    char *o=(char *)out;
    for(size_t i=0;i<count;++i,fade+=fade_stride,o+=out_stride) {
        const uint32_t f=*fade,g=255u-f;
        const uint32_t v0=div65025((n0*g+f0*f)*31u+32512u);
        const uint32_t v1=div65025((n1*g+f1*f)*31u+32512u);
        const uint32_t v2=div65025((n2*g+f2*f)*31u+32512u);
        *(uint16_t *)(void *)o=(uint16_t)(0x8000u|v0|(v1<<5)|(v2<<10));
    }
    return true;
}
