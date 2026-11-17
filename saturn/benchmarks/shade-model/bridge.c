/* SH-2 bridge for the executed-instruction shade model. MIT; Copyright (c) 2026
 * SH2ZAM Contributors. Every entry point is called directly by the model
 * harness with the SH-2 calling convention. "fast_" entries run the library;
 * "plain_" entries are straightforward C written from the same rules the way
 * a renderer computes them today (C division operators, a binary search, the
 * per-corner loop), so each pair is compared on identical inputs. Buffers are
 * written by the harness before each call; nothing here uses libc. */
#include "sh2zam/shade_experimental.h"

#define NOINLINE __attribute__((noinline))
#define MAXN 256u
#define MAXP 1024u
#define MAXF 256u

/* ---- inputs and outputs shared with the harness ---- */
sh2z_vec3_t prep_light,prep_reflection;
int16_t prep_basis[3][3];
uint8_t prep_ambient[4],prep_color[4];
sh2z_shade_light_t light;
int8_t normals[MAXN*4u];
uint8_t lut[256];
uint8_t lit[MAXN*4u];
uint8_t bases[64u*3u];
uint8_t fog_rgb[4];
sh2z_shade_pair_t pairs[MAXP];
uint16_t gouraud[MAXP];
volatile uint32_t saturated;
sh2z_shade_plane_t planes[MAXF];
uint16_t order[MAXF],kept[MAXF];
int32_t eye[3];
uint32_t eye_error;
volatile uint32_t kept_count;
sh2z_shade_fog_t fog_table;
int32_t fog_params[5];   /* multiplier, offset, pa, pb, far_z */
uint32_t plain_depth[256];
int32_t depths[MAXP];
uint8_t fades[MAXP];
int16_t fade_slope[3],fade_bias[3];
uint8_t near_rgb[4],far_rgb[4];
uint16_t colours[MAXP];
/* vehicle frame: faces with a palette and four corner normals, or four baked
 * pair ids into a per-model list of unique (palette, normal) pairs */
uint16_t face_palette[MAXF],corner_normal[MAXF*4u],corner_pair[MAXF*4u];
sh2z_shade_pair_t unique_pairs[MAXP];
uint16_t unique_gouraud[MAXP];
uint16_t frame_out[MAXF*4u];
sh2z_shade_pair_t frame_pairs[MAXF*4u];

/* ---- light ---- */
NOINLINE uint32_t fast_light(uint32_t count) {
    if(!sh2z_shade_light_prepare(&light,prep_light,prep_reflection,(const int16_t(*)[3])prep_basis,prep_ambient,prep_color))return 0;
    return sh2z_shade_light_normals(lit,4,normals,4,count,&light,lut);
}
NOINLINE uint32_t fast_light_normals(uint32_t count) {
    return sh2z_shade_light_normals(lit,4,normals,4,count,&light,lut);
}
/* The usual per-normal loop: normalise, quarter, rotate by the transposed
 * basis, then per normal an unsigned division per channel. */
NOINLINE uint32_t plain_light(uint32_t count) {
    const sh2z_vec3_t l=sh2z_vec3_normalize(prep_light),r=sh2z_vec3_normalize(prep_reflection);
    if((!l.x&&!l.y&&!l.z)||(!r.x&&!r.y&&!r.z))return 0;
    const int32_t li[3]={l.x/4,l.y/4,l.z/4},ri[3]={r.x/4,r.y/4,r.z/4};
    int32_t ll[3],lr[3];
    for(unsigned j=0;j<3;++j){int32_t a=0,b=0;for(unsigned k=0;k<3;++k){a+=prep_basis[k][j]*li[k];b+=prep_basis[k][j]*ri[k];}ll[j]=a/16384;lr[j]=b/16384;}
    for(uint32_t i=0;i<count;++i) {
        const int8_t *n=normals+4u*i;int32_t diffuse=0,reflected=0;
        for(unsigned k=0;k<3;++k){diffuse+=n[k]*ll[k];reflected+=n[k]*lr[k];}
        if(diffuse<0)diffuse=0;
        if(diffuse>127*16384)diffuse=127*16384;
        for(unsigned k=0;k<3;++k){unsigned v=prep_ambient[k]+(unsigned)diffuse*prep_color[k]/(127u*16384u);lit[4u*i+k]=(uint8_t)(v>255u?255u:v);}
        int32_t index=reflected/16384;if(index< -128)index=-128;if(index>127)index=127;
        lit[4u*i+3u]=lut[index+128];
    }
    return 1;
}

/* ---- Gouraud ---- */
static unsigned mul8(unsigned a,unsigned b){const unsigned x=a*b+128u;return (x+(x>>8))>>8;}
static unsigned blend(unsigned a,unsigned b,unsigned f){const unsigned x=a*f+b*(255u-f)+128u;return (x+(x>>8))>>8;}
static uint16_t shade(const uint8_t *base,const uint8_t *l,const uint8_t *fog,uint32_t *sat){
    uint16_t packed=0;
    for(unsigned k=0;k<3;++k){unsigned v=blend(mul8(base[k],l[k]),fog[k],255u-l[3]);
        int value=16+(int)(v>>3)-(int)(base[k]>>4);
        if(value<0){value=0;++*sat;}if(value>31){value=31;++*sat;}
        packed|=(uint16_t)((unsigned)value<<(5*k));}
    return packed;
}
NOINLINE uint32_t fast_gouraud(uint32_t count) {
    uint32_t s=0;const uint32_t ok=sh2z_shade_gouraud(gouraud,&s,pairs,count,bases,3,lit,4,fog_rgb);
    saturated=s;return ok;
}
NOINLINE uint32_t plain_gouraud(uint32_t count) {
    uint32_t s=0;
    for(uint32_t i=0;i<count;++i)gouraud[i]=shade(bases+3u*pairs[i].base,lit+4u*pairs[i].light,fog_rgb,&s);
    saturated=s;return 1;
}

/* ---- culling ---- */
NOINLINE uint32_t fast_cull(uint32_t count,uint32_t use_order) {
    size_t n=0;const uint32_t ok=sh2z_shade_cull(kept,&n,use_order?order:0,count,planes,sizeof(planes[0]),eye,eye_error);
    kept_count=(uint32_t)n;return ok;
}
/* The source's per-face test on the same 16-byte records. */
NOINLINE uint32_t plain_cull(uint32_t count,uint32_t use_order) {
    uint32_t n=0;
    for(uint32_t i=0;i<count;++i) {
        const unsigned id=use_order?order[i]:i;const sh2z_shade_plane_t *p=planes+id;
        int32_t dot=-p->plane,margin=3;
        for(unsigned k=0;k<3;++k){int32_t normal=p->normal[k],distance=eye[k]-p->first[k];dot+=normal*eye[k];
            margin+=(distance<0?-distance:distance)+(normal<0?-normal:normal)*(int32_t)eye_error;}
        if(dot< -margin)continue;
        kept[n++]=(uint16_t)id;
    }
    kept_count=n;return 1;
}

/* ---- fog ---- */
NOINLINE uint32_t fast_fog_build(void) {
    return sh2z_shade_fog_build(&fog_table,fog_params[0],fog_params[1],fog_params[2],fog_params[3],fog_params[4]);
}
NOINLINE uint32_t plain_fog_build(void) {
    const int32_t fm=fog_params[0],fo=fog_params[1],pa=fog_params[2],pb=fog_params[3],far_z=fog_params[4];
    const uint64_t numerator=(uint64_t)(-(int64_t)pb)*UINT64_C(1073741824);
    plain_depth[0]=0;
    for(unsigned i=1;i<256;++i){
        const int64_t n=((int64_t)i-fo)*INT64_C(1073741824);
        const int64_t required=n/fm+(n%fm>0);
        uint32_t depth=(uint32_t)far_z+1u;
        if(required<=-INT64_C(1073741824))depth=0;
        else if(required<=INT64_C(1073741824)){
            const int64_t m=(int64_t)pa-required;
            if(m>=0){const uint64_t q12=numerator/((uint64_t)m+1u)+1u;
                if(q12<=(uint32_t)far_z/4u)depth=(uint32_t)q12*4u;}
        }
        plain_depth[i]=depth;
    }
    return 1;
}
NOINLINE uint32_t fast_fog(uint32_t count) {
    return sh2z_shade_fog_lookup(fades,1,depths,4,count,&fog_table);
}
NOINLINE uint32_t plain_fog(uint32_t count) {
    for(uint32_t i=0;i<count;++i){
        unsigned lo=0,hi=256;const uint32_t depth=depths[i]>0?(uint32_t)depths[i]:0;
        while(lo+1<hi){const unsigned middle=(lo+hi)/2;if(depth>=fog_table.depth[middle])lo=middle;else hi=middle;}
        fades[i]=(uint8_t)lo;
    }
    return 1;
}
NOINLINE uint32_t fast_offset(uint32_t count) {
    return sh2z_shade_fade_offset(colours,2,fades,1,count,fade_slope,fade_bias);
}
NOINLINE uint32_t plain_offset(uint32_t count) {
    for(uint32_t i=0;i<count;++i){uint16_t packed=0x8000;
        for(unsigned c=0;c<3;++c){int color=16+((int)fades[i]*fade_slope[c]-fade_bias[c])/255;
            if(color<0)color=0;
            if(color>31)color=31;
            packed|=(uint16_t)((unsigned)color<<(c*5));}
        colours[i]=packed;}
    return 1;
}
NOINLINE uint32_t fast_mix(uint32_t count) {
    return sh2z_shade_fade_mix(colours,2,fades,1,count,near_rgb,far_rgb);
}
NOINLINE uint32_t plain_mix(uint32_t count) {
    for(uint32_t i=0;i<count;++i){uint16_t packed=0x8000;
        for(unsigned c=0;c<3;++c){int color=((near_rgb[c]*(255-(int)fades[i])+far_rgb[c]*(int)fades[i])*31+32512)/65025;
            if(color<0)color=0;
            if(color>31)color=31;
            packed|=(uint16_t)((unsigned)color<<(c*5));}
        colours[i]=packed;}
    return 1;
}

/* ---- one vehicle frame: light every normal, cull, colour four corners per
 * kept face. Output: kept faces' corner colours in draw order. ---- */
NOINLINE uint32_t plain_frame(uint32_t faces,uint32_t normal_count) {
    if(!plain_light(normal_count))return 0;
    uint32_t s=0,n=0;
    for(uint32_t i=0;i<faces;++i) {
        const unsigned id=order[i];const sh2z_shade_plane_t *p=planes+id;
        int32_t dot=-p->plane,margin=3;
        for(unsigned k=0;k<3;++k){int32_t normal=p->normal[k],distance=eye[k]-p->first[k];dot+=normal*eye[k];
            margin+=(distance<0?-distance:distance)+(normal<0?-normal:normal)*(int32_t)eye_error;}
        if(dot< -margin)continue;
        for(unsigned j=0;j<4;++j)frame_out[4u*n+j]=shade(bases+3u*face_palette[id],lit+4u*corner_normal[4u*id+j],fog_rgb,&s);
        ++n;
    }
    saturated=s;kept_count=n;return 1;
}
/* Kernels, corner pairs gathered per kept face. */
NOINLINE uint32_t fast_frame(uint32_t faces,uint32_t normal_count) {
    if(!fast_light(normal_count))return 0;
    size_t n=0;
    if(!sh2z_shade_cull(kept,&n,order,faces,planes,sizeof(planes[0]),eye,eye_error))return 0;
    for(size_t i=0;i<n;++i){const unsigned id=kept[i];
        for(unsigned j=0;j<4;++j){frame_pairs[4u*i+j].base=face_palette[id];frame_pairs[4u*i+j].light=corner_normal[4u*id+j];}}
    uint32_t s=0;
    if(!sh2z_shade_gouraud(frame_out,&s,frame_pairs,4u*n,bases,3,lit,4,fog_rgb))return 0;
    saturated=s;kept_count=(uint32_t)n;return 1;
}
/* Kernels, unique (palette, normal) pairs baked offline: one Gouraud per
 * pair per frame, corners copy from the table. The saturation count is per
 * unique pair, not per corner. */
NOINLINE uint32_t fast_frame_baked(uint32_t faces,uint32_t normal_count,uint32_t unique) {
    if(!fast_light(normal_count))return 0;
    size_t n=0;
    if(!sh2z_shade_cull(kept,&n,order,faces,planes,sizeof(planes[0]),eye,eye_error))return 0;
    uint32_t s=0;
    if(!sh2z_shade_gouraud(unique_gouraud,&s,unique_pairs,unique,bases,3,lit,4,fog_rgb))return 0;
    for(size_t i=0;i<n;++i){const uint16_t *c=corner_pair+4u*kept[i];uint16_t *o=frame_out+4u*i;
        o[0]=unique_gouraud[c[0]];o[1]=unique_gouraud[c[1]];o[2]=unique_gouraud[c[2]];o[3]=unique_gouraud[c[3]];}
    saturated=s;kept_count=(uint32_t)n;return 1;
}
