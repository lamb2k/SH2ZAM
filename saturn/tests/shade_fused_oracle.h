/* Independent oracle for the shade-2 kernels (fused fade lookup and colour,
 * prepared fog and ramps), shared by tests/test_shade_fused.c and the SH-2
 * model harness. Written from the header text with plain C division and the
 * shade-1 oracle's source search (tests/shade_oracle.h): no bucket index, no
 * magic-number divides, no channel table, no code shared with the library.
 * Mutation flags select deliberately wrong rules (negative controls).
 * MIT; SH2ZAM Contributors. */
#ifndef SH2ZAM_SHADE_FUSED_ORACLE_H
#define SH2ZAM_SHADE_FUSED_ORACLE_H
#include "shade_oracle.h"

enum {
    FMUT_NONE=0,
    FMUT_BUCKET_ONLY=1,      /* fade = fade at the start of its 4096-bucket (no refinement) */
    FMUT_NEG_FAR=2,          /* negative depths fade like far ones (255) */
    FMUT_MEAN_ROUND=4,       /* variant from the rounded mean, not the floor */
    FMUT_STRICT=8,           /* variant needs mean > threshold */
    FMUT_EXACT_TRUNC=16,     /* shade-1's exact channel (no +1 at negative multiples of 255) */
    FMUT_FLOOR=32,           /* floor(x / 255) instead of trunc */
    FMUT_LEVEL0=64,          /* bias always uses level[0] */
    FMUT_SLOPE_SIGN=128      /* slope = m - live */
};

/* The ramp as the caller describes it (no prepared values). */
typedef struct {
    unsigned form;                       /* 1 offset, 2 mix */
    uint8_t live[3],base[3],level[3];    /* offset */
    uint8_t near_rgb[3],far_rgb[3];      /* mix */
    unsigned threshold[2];               /* 256 = never */
} oramp_t;

/* fade(z) through the source's binary search (shade-1 oracle). */
static inline unsigned ofused_fade(const uint32_t depth[256],int32_t z,unsigned mut) {
    if((mut&FMUT_NEG_FAR) && z<0)return 255;
    if(mut&FMUT_BUCKET_ONLY) {
        /* the bucket start, written from the header's shift rule */
        const uint64_t top=depth[255]>0x7FFFFFFFu?0x7FFFFFFFu:depth[255];
        unsigned shift=1;
        while((top>>shift)>=4096u)++shift;
        if(z<0)return oracle_fade(depth,0);
        const uint64_t start=((uint64_t)(uint32_t)z>>shift)<<shift;
        if(((uint64_t)(uint32_t)z>>shift)>=4096u)return 255;
        return oracle_fade(depth,(int32_t)start);
    }
    return oracle_fade(depth,z);
}
static inline int ofused_variant(uint64_t sum,uint64_t n,const unsigned t[2],unsigned mut) {
    /* mean = floor(sum / n) compared with each threshold */
    const uint64_t mean=(mut&FMUT_MEAN_ROUND)?(sum+n/2u)/n:sum/n;
    int v=0;
    for(int j=0;j<2;++j)v+=(mut&FMUT_STRICT)?mean>t[j]:mean>=t[j];
    return v;
}
/* One OFFSET channel for slope, bias and fade. */
static inline unsigned ofused_channel(int64_t fade,int64_t slope,int64_t bias,unsigned mut) {
    const int64_t x=fade*slope-bias;
    int64_t q=(mut&FMUT_FLOOR)?fdiv(x,255):x/255;          /* C: truncating */
    if(!(mut&(FMUT_EXACT_TRUNC|FMUT_FLOOR)) && x<0 && x%255==0)q+=1;
    return (unsigned)clamp64(16+q,0,31);
}
static inline uint16_t ofused_offset_word(unsigned fade,const oramp_t *r,const uint8_t mean[3],int v,unsigned mut) {
    unsigned w=0x8000u;
    const int lv=(mut&FMUT_LEVEL0)?r->level[0]:r->level[v];
    for(int k=0;k<3;++k) {
        const int64_t slope=(mut&FMUT_SLOPE_SIGN)?(int64_t)mean[k]-r->live[k]:(int64_t)r->live[k]-mean[k];
        const int64_t bias=(int64_t)lv*((int64_t)r->base[k]-mean[k]);
        w|=ofused_channel(fade,slope,bias,mut)<<(5*k);
    }
    return (uint16_t)w;
}
/* The whole call: returns the variant and writes count words. */
static inline int oracle_fused(uint16_t *out,const int32_t *z,size_t count,const uint32_t depth[256],
    const oramp_t *r,const uint8_t mean[3],unsigned mut) {
    if(!count)return 0;
    static unsigned fades[65536];
    uint64_t sum=0;
    for(size_t i=0;i<count;++i){fades[i]=ofused_fade(depth,z[i],mut);sum+=fades[i];}
    const int v=ofused_variant(sum,count,r->threshold,mut);
    for(size_t i=0;i<count;++i)
        out[i]=r->form==1u?ofused_offset_word(fades[i],r,mean,v,mut):oracle_mix(fades[i],r->near_rgb,r->far_rgb,0);
    return v;
}

#endif
