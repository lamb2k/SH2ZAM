/* SH-2 bridge for the executed-instruction model of the shade-2 kernels.
 * MIT; Copyright (c) 2026 SH2ZAM Contributors. Every entry point is called
 * directly by the model harness with the SH-2 calling convention. "fast_"
 * entries run the library's shade-2 kernels; "shade1_" entries run the
 * shade-1 batch calls per piece; "plain_" entries are the per-piece C a road
 * renderer runs today: an inlined bucket-indexed fog lookup, an integer mean
 * and (mean + 64) / 128 variant, then per corner the truncating / 255 as a
 * shift-add and / 65025 as a multiply. All three see identical inputs.
 * Buffers are written by the harness before each call; no libc. */
#include "sh2zam/shade_experimental.h"
#include "sh2zam/cdiv_experimental.h"
#include "sh2zam/view_experimental.h"

#define NOINLINE __attribute__((noinline))
#define MAXV 1024u
#define MAXP 256u

/* ---- inputs and outputs shared with the harness ---- */
int32_t fog_params[5];                 /* multiplier, offset, pa, pb, far_z */
sh2z_shade_fog_t fog_table;            /* shade-1 table and 257-bucket index */
sh2z_shade_fogp_t fogp;                /* shade-2 prepared fog */
sh2z_shade_ramp_t ramp_offset,ramp_mix;
uint8_t ramp_live[4],ramp_base[4],ramp_level[4],near_rgb[4],far_rgb[4];
uint16_t ramp_threshold[2];
int32_t live5[3],base5[3];             /* the plain caller's per-frame colours */
uint32_t plain_scale;                  /* the plain caller's bucket multiplier */
sh2z_view_vertex_t verts[MAXV];        /* piece corners, consecutive; z is read */
uint16_t piece_count[MAXP];
uint8_t piece_solid[MAXP];
uint8_t piece_mean[MAXP*4u];
uint16_t shades[MAXV];
uint8_t variants[MAXP];
uint32_t cdiv_in[MAXV],cdiv_out[MAXV];
sh2z_shade_ramp_t ramp_ident;          /* MIX words equal to the fade: reads fades back */
uint16_t args_out[8];
/* The same depths scattered, reached through one pointer per corner as a
 * caller's corner list (vref[i] -> logical vertex i). */
sh2z_view_vertex_t verts_scattered[MAXV];
const sh2z_view_vertex_t *vref[MAXV];

/* ---- per-frame preparation ---- */
NOINLINE uint32_t fast_prepare(uint32_t unused) {
    (void)unused;
    if(!sh2z_shade_fog_build(&fog_table,fog_params[0],fog_params[1],fog_params[2],fog_params[3],fog_params[4]))return 0;
    if(!sh2z_shade_fogp_prepare(&fogp,&fog_table))return 0;
    if(!sh2z_shade_ramp_offset(&ramp_offset,ramp_live,ramp_base,ramp_level,ramp_threshold))return 0;
    if(!sh2z_shade_ramp_mix(&ramp_mix,near_rgb,far_rgb,ramp_threshold))return 0;
    if(!sh2z_shade_ramp_mix(&ramp_ident,near_rgb,far_rgb,0))return 0;
    for(unsigned f=0;f<256u;++f)ramp_ident.mix[f]=(uint16_t)f;
    for(unsigned k=0;k<3;++k){live5[k]=ramp_live[k];base5[k]=ramp_base[k];}
    plain_scale=fog_table.shift?(uint32_t)1u<<(32u-fog_table.shift):0u;
    return 1;
}
/* The fades of count depths at verts[0..], through the identity MIX ramp. */
NOINLINE uint32_t fast_fades(uint32_t count) {
    return sh2z_shade_fog_gouraud(shades,&verts[0].z,sizeof(verts[0]),count,&fogp,&ramp_ident,0)>=0;
}
/* One argument set per case; returns the kernel's result as a word. */
NOINLINE uint32_t fast_args(uint32_t which) {
    const uint8_t hot[4]={5,128,7,0};
    sh2z_shade_ramp_t *r=&ramp_offset;
    const void *z=&verts[0].z;
    int got;
    switch(which) {
    case 0:got=sh2z_shade_fog_gouraud(0,z,20,4,&fogp,r,piece_mean);break;
    case 1:got=sh2z_shade_fog_gouraud(args_out,0,20,4,&fogp,r,piece_mean);break;
    case 2:got=sh2z_shade_fog_gouraud(args_out,z,20,4,0,r,piece_mean);break;
    case 3:got=sh2z_shade_fog_gouraud(args_out,z,20,4,&fogp,0,piece_mean);break;
    case 4:got=sh2z_shade_fog_gouraud(args_out,z,20,4,&fogp,r,0);break;
    case 5:got=sh2z_shade_fog_gouraud(args_out,z,20,4,&fogp,r,hot);break;
    case 6:got=sh2z_shade_fog_gouraud(args_out,z,20,65536,&fogp,r,piece_mean);break;
    case 7:got=sh2z_shade_fog_gouraud(args_out,z,22,4,&fogp,r,piece_mean);break;
    case 8:got=sh2z_shade_fog_gouraud(args_out,z,65536,4,&fogp,r,piece_mean);break;
    case 9:got=sh2z_shade_fog_gouraud(args_out,(const char *)z+2,20,4,&fogp,r,piece_mean);break;
    case 10:got=sh2z_shade_fog_gouraud((uint16_t *)(void *)((char *)args_out+1),z,20,4,&fogp,r,piece_mean);break;
    case 11:ramp_offset.form=0;got=sh2z_shade_fog_gouraud(args_out,z,20,4,&fogp,r,piece_mean);ramp_offset.form=1;break;
    case 12:ramp_offset.form=3;got=sh2z_shade_fog_gouraud(args_out,z,20,4,&fogp,r,piece_mean);ramp_offset.form=1;break;
    case 13:got=sh2z_shade_fog_gouraud(args_out,0,0,0,0,0,0);break;                 /* count 0: 0 */
    case 14:got=sh2z_shade_fog_gouraud(args_out,z,0,4,&fogp,r,piece_mean);break;     /* stride 0: accepted */
    case 15:got=sh2z_shade_fog_gouraud(args_out,z,20,4,&fogp,&ramp_mix,0);break;     /* MIX needs no mean */
    case 16:got=sh2z_shade_fog_gouraud_refs(args_out,0,8,4,&fogp,r,piece_mean);break;
    case 17:got=sh2z_shade_fog_gouraud_refs(args_out,(const void *const *)vref,10,4,&fogp,r,piece_mean);break;
    case 18:got=sh2z_shade_fog_gouraud_refs(args_out,(const void *const *)(const void *)((const char *)vref+2),8,4,&fogp,r,piece_mean);break;
    case 19:got=sh2z_shade_fog_gouraud_refs(args_out,(const void *const *)vref,8,4,&fogp,&ramp_offset,hot);break;
    case 20:got=sh2z_shade_fog_gouraud_refs(args_out,(const void *const *)vref,8,0,&fogp,r,piece_mean);break;  /* 0 */
    case 21:got=sh2z_shade_fog_gouraud_refs(args_out,(const void *const *)vref,8,4,&fogp,r,piece_mean);break;  /* accepted */
    default:got=-2;break;
    }
    return (uint32_t)got;
}
NOINLINE uint32_t fast_fogp_prepare(uint32_t unused) {(void)unused;return sh2z_shade_fogp_prepare(&fogp,&fog_table);}
NOINLINE uint32_t fast_ramp_offset(uint32_t unused) {(void)unused;return sh2z_shade_ramp_offset(&ramp_offset,ramp_live,ramp_base,ramp_level,ramp_threshold);}
NOINLINE uint32_t fast_ramp_mix(uint32_t unused) {(void)unused;return sh2z_shade_ramp_mix(&ramp_mix,near_rgb,far_rgb,ramp_threshold);}

/* ---- per piece: shade-2 fused call ---- */
NOINLINE uint32_t fast_pieces(uint32_t pieces) {
    const sh2z_view_vertex_t *v=verts;uint16_t *o=shades;
    for(uint32_t p=0;p<pieces;++p) {
        const unsigned n=piece_count[p];
        const int r=sh2z_shade_fog_gouraud(o,&v->z,sizeof(*v),n,&fogp,piece_solid[p]?&ramp_mix:&ramp_offset,piece_mean+4u*p);
        if(r<0)return 0;
        variants[p]=(uint8_t)r;v+=n;o+=n;
    }
    return 1;
}
/* The same pieces through the corner-pointer form. */
NOINLINE uint32_t fast_refs_pieces(uint32_t pieces) {
    const sh2z_view_vertex_t *const *v=vref;uint16_t *o=shades;
    for(uint32_t p=0;p<pieces;++p) {
        const unsigned n=piece_count[p];
        const int r=sh2z_shade_fog_gouraud_refs(o,(const void *const *)v,offsetof(sh2z_view_vertex_t,z),n,&fogp,
            piece_solid[p]?&ramp_mix:&ramp_offset,piece_mean+4u*p);
        if(r<0)return 0;
        variants[p]=(uint8_t)r;v+=n;o+=n;
    }
    return 1;
}
/* One batch of count vertices (a strip), textured with piece 0's mean. */
NOINLINE uint32_t fast_batch(uint32_t count) {
    const int r=sh2z_shade_fog_gouraud(shades,&verts[0].z,sizeof(verts[0]),count,&fogp,&ramp_offset,piece_mean);
    if(r<0)return 0;
    variants[0]=(uint8_t)r;return 1;
}

/* ---- per piece: shade-1 batch calls ---- */
NOINLINE uint32_t shade1_pieces(uint32_t pieces) {
    const sh2z_view_vertex_t *v=verts;uint16_t *o=shades;
    for(uint32_t p=0;p<pieces;++p) {
        const unsigned n=piece_count[p];
        uint8_t fade[16];
        if(!sh2z_shade_fog_lookup(fade,1,&v->z,sizeof(*v),n,&fog_table))return 0;
        unsigned middle=0;for(unsigned i=0;i<n;++i)middle+=fade[i];
        middle/=n;unsigned variant=(middle+64u)/128u;if(variant>2u)variant=2u;
        const int32_t selected=variant==2u?255:(int32_t)variant*128;
        if(piece_solid[p]) {
            if(!sh2z_shade_fade_mix(o,2,fade,1,n,near_rgb,far_rgb))return 0;
        } else {
            const uint8_t *m=piece_mean+4u*p;int16_t slope[3],bias[3];
            for(unsigned k=0;k<3;++k){slope[k]=(int16_t)(live5[k]-m[k]);bias[k]=(int16_t)(selected*(base5[k]-m[k]));}
            if(!sh2z_shade_fade_offset(o,2,fade,1,n,slope,bias))return 0;
        }
        variants[p]=(uint8_t)variant;v+=n;o+=n;
    }
    return 1;
}

/* ---- per piece: the caller's C today ---- */
static unsigned plain_fog_lookup(int32_t z) {
    const uint32_t depth=z>0?(uint32_t)z:0;
    const uint32_t b=fog_table.shift?(uint32_t)(((uint64_t)depth*plain_scale)>>32):depth;
    if(b>255u)return 255;
    unsigned lo=fog_table.bucket[b],hi=fog_table.bucket[b+1u];
    while(lo<hi){const unsigned middle=(lo+hi+1u)>>1;
        if(fog_table.depth[middle]<=depth)lo=middle;else hi=middle-1u;}
    return lo;
}
static int32_t plain_div255(int32_t n) {      /* trunc(n / 255), |n| <= 65534 */
    const uint32_t a=(uint32_t)(n<0?-n:n),q=(a+(a>>8)+1u)>>8;
    return n<0?-(int32_t)q:(int32_t)q;
}
static uint32_t plain_div65025(uint32_t n) {  /* n / 65025, n <= 2048287 */
    return (uint32_t)(((uint64_t)n*UINT32_C(2113633))>>37);
}
NOINLINE uint32_t plain_pieces(uint32_t pieces) {
    const sh2z_view_vertex_t *v=verts;uint16_t *o=shades;
    for(uint32_t p=0;p<pieces;++p) {
        const unsigned count=piece_count[p];
        unsigned fade[16],middle=0;
        for(unsigned i=0;i<count;++i){fade[i]=plain_fog_lookup(v[i].z);middle+=fade[i];}
        middle/=count;unsigned variant=(middle+64)/128;if(variant>2)variant=2;
        const int selected=variant==2?255:(int)variant*128;
        if(piece_solid[p]) {
            for(unsigned i=0;i<count;++i){uint16_t packed=0x8000;const uint32_t f=fade[i];
                for(unsigned c=0;c<3;++c){
                    const uint32_t color=plain_div65025((near_rgb[c]*(255u-f)+far_rgb[c]*f)*31u+32512u);
                    packed|=(uint16_t)((color>31u?31u:color)<<(c*5));
                }
                o[i]=packed;
            }
        } else {
            const uint8_t *means=piece_mean+4u*p;
            for(unsigned i=0;i<count;++i){uint16_t packed=0x8000;const int32_t f=(int32_t)fade[i];
                for(unsigned c=0;c<3;++c){const int32_t mean=means[c],live=live5[c],base=base5[c];int32_t color;
                    color=16+plain_div255(f*(live-mean)-selected*(base-mean));
                    if(color<0)color=0;
                    if(color>31)color=31;
                    packed|=(uint16_t)((unsigned)color<<(c*5));
                }
                o[i]=packed;
            }
        }
        variants[p]=(uint8_t)variant;v+=count;o+=count;
    }
    return 1;
}
/* The caller's per-piece code over one batch of count vertices (textured,
 * piece 0's mean). */
NOINLINE uint32_t plain_batch(uint32_t count) {
    static unsigned fade[MAXV];
    unsigned middle=0;
    for(unsigned i=0;i<count;++i){fade[i]=plain_fog_lookup(verts[i].z);middle+=fade[i];}
    middle/=count;unsigned variant=(middle+64)/128;if(variant>2)variant=2;
    const int selected=variant==2?255:(int)variant*128;
    for(unsigned i=0;i<count;++i){uint16_t packed=0x8000;const int32_t f=(int32_t)fade[i];
        for(unsigned c=0;c<3;++c){const int32_t mean=piece_mean[c],live=live5[c],base=base5[c];int32_t color;
            color=16+plain_div255(f*(live-mean)-selected*(base-mean));
            if(color<0)color=0;
            if(color>31)color=31;
            packed|=(uint16_t)((unsigned)color<<(c*5));
        }
        shades[i]=packed;
    }
    variants[0]=(uint8_t)variant;
    return 1;
}

/* ---- scalar constant divides over arrays (per-element cost is the slope) ---- */
NOINLINE uint32_t cdiv255_u(uint32_t n) {for(uint32_t i=0;i<n;++i)cdiv_out[i]=sh2z_cdiv255_u32(cdiv_in[i]);return 1;}
NOINLINE uint32_t cdiv255_s(uint32_t n) {for(uint32_t i=0;i<n;++i)cdiv_out[i]=(uint32_t)sh2z_cdiv255_s32((int32_t)cdiv_in[i]);return 1;}
NOINLINE uint32_t cdiv65025_u(uint32_t n) {for(uint32_t i=0;i<n;++i)cdiv_out[i]=sh2z_cdiv65025_u32(cdiv_in[i]);return 1;}
NOINLINE uint32_t cdiv_copy(uint32_t n) {for(uint32_t i=0;i<n;++i)cdiv_out[i]=cdiv_in[i];return 1;}
NOINLINE uint32_t c_div255_s(uint32_t n) {for(uint32_t i=0;i<n;++i)cdiv_out[i]=(uint32_t)((int32_t)cdiv_in[i]/255);return 1;}
NOINLINE uint32_t shiftadd_div255_s(uint32_t n) {for(uint32_t i=0;i<n;++i)cdiv_out[i]=(uint32_t)plain_div255((int32_t)cdiv_in[i]);return 1;}
NOINLINE uint32_t c_div65025(uint32_t n) {for(uint32_t i=0;i<n;++i)cdiv_out[i]=cdiv_in[i]/65025u;return 1;}
