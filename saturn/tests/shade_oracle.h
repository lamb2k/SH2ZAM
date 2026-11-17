/* Independent oracle for the shade-1 kernels, shared by tests/test_shade.c and
 * the SH-2 model harness. Written from the rules in shade_experimental.h in
 * 64- and 128-bit arithmetic with plain C division: no reciprocal constants,
 * no bucket index, no code shared with the library. Mutation flags select
 * deliberately wrong rules (negative controls). MIT; SH2ZAM Contributors. */
#ifndef SH2ZAM_SHADE_ORACLE_H
#define SH2ZAM_SHADE_ORACLE_H
#include "sh2zam/shade_experimental.h"

__extension__ typedef __int128 wide_t;

static inline int64_t tdiv(int64_t a,int64_t b) {   /* truncating, written out */
    const int64_t q=(a<0?-a:a)/(b<0?-b:b);
    return (a<0)!=(b<0)?-q:q;
}
static inline int64_t fdiv(int64_t a,int64_t b) {   /* floor, b > 0 */
    return a>=0?a/b:-((-a+b-1)/b);
}
static inline int64_t clamp64(int64_t v,int64_t lo,int64_t hi) {return v<lo?lo:v>hi?hi:v;}

enum {
    MUT_NONE=0,
    MUT_NO_TRANSPOSE=1,      /* prepare multiplies by basis, not its transpose */
    MUT_NO_DIFFUSE_CAP=2,    /* diffuse not capped at 127 * 16384 */
    MUT_REFLECT_FLOOR=4,     /* reflection index floors instead of truncating */
    MUT_BLEND_FACTOR=8,      /* blend weight light[3] instead of 255 - light[3] */
    MUT_MUL8_FLOOR=16,       /* mul8 as a*b/255 floored */
    MUT_NO_MARGIN=32,        /* cull on the plain dot sign */
    MUT_NO_EYE_ERROR=64,     /* margin without the eye-error term */
    MUT_BUCKET_ONLY=128,     /* fog fade from bucket[z >> shift] alone (no refinement) */
    MUT_FLOOR255=256,        /* fade offset divides by 255 with floor */
    MUT_MIX_NO_ROUND=512     /* fade mix without the +32512 rounding */
};
typedef struct { int64_t light[3],reflection[3]; uint8_t ambient[3],color[3]; } olight_t;

static inline bool oracle_prepare(olight_t *o,sh2z_vec3_t l,sh2z_vec3_t r,const int16_t basis[3][3],
    const uint8_t ambient[3],const uint8_t color[3],unsigned mut) {
    const sh2z_vec3_t ln=sh2z_vec3_normalize(l),rn=sh2z_vec3_normalize(r);
    if((ln.x==0&&ln.y==0&&ln.z==0)||(rn.x==0&&rn.y==0&&rn.z==0))return false;
    const int64_t lq[3]={tdiv(ln.x,4),tdiv(ln.y,4),tdiv(ln.z,4)},rq[3]={tdiv(rn.x,4),tdiv(rn.y,4),tdiv(rn.z,4)};
    for(int j=0;j<3;++j) {
        int64_t a=0,b=0;
        for(int k=0;k<3;++k) {
            const int64_t m=(mut&MUT_NO_TRANSPOSE)?basis[j][k]:basis[k][j];
            a+=m*lq[k];b+=m*rq[k];
        }
        o->light[j]=tdiv(a,16384);o->reflection[j]=tdiv(b,16384);
        o->ambient[j]=ambient[j];o->color[j]=color[j];
    }
    return true;
}
static inline void oracle_normal(uint8_t out[4],const int8_t n[3],const olight_t *o,const uint8_t lut[256],unsigned mut) {
    int64_t d=0,r=0;
    for(int k=0;k<3;++k){d+=(int64_t)n[k]*o->light[k];r+=(int64_t)n[k]*o->reflection[k];}
    d=clamp64(d,0,(mut&MUT_NO_DIFFUSE_CAP)?INT64_MAX:127*16384);
    for(int k=0;k<3;++k)out[k]=(uint8_t)clamp64(o->ambient[k]+d*o->color[k]/(127*16384),0,255);
    const int64_t index=clamp64((mut&MUT_REFLECT_FLOOR)?fdiv(r,16384):tdiv(r,16384),-128,127);
    out[3]=lut[index+128];
}
static inline int64_t omul8(int64_t a,int64_t b,unsigned mut) {
    if(mut&MUT_MUL8_FLOOR)return a*b/255;
    const int64_t x=a*b+128;return (x+(x>>8))>>8;
}
static inline int64_t oblend(int64_t a,int64_t b,int64_t t) {const int64_t x=a*t+b*(255-t)+128;return (x+(x>>8))>>8;}
static inline int64_t olit(const uint8_t *b,const uint8_t *l,const uint8_t *f,int k,unsigned mut) {
    const int64_t t=(mut&MUT_BLEND_FACTOR)?l[3]:255-l[3];
    return oblend(omul8(b[k],l[k],mut),f[k],t);
}
static inline uint16_t oracle_gouraud(const uint8_t *b,const uint8_t *l,const uint8_t *f,uint32_t *sat,unsigned mut) {
    unsigned packed=0;
    for(int k=0;k<3;++k) {
        int64_t v=16+(olit(b,l,f,k,mut)>>3)-(b[k]>>4);
        if(v<0){v=0;++*sat;}
        if(v>31){v=31;++*sat;}
        packed|=(unsigned)v<<(5*k);
    }
    return (uint16_t)packed;
}
static inline uint16_t oracle_solid(const uint8_t *b,const uint8_t *l,const uint8_t *f) {
    unsigned packed=0x8000u;
    for(int k=0;k<3;++k)packed|=(unsigned)(olit(b,l,f,k,0)>>3)<<(5*k);
    return (uint16_t)packed;
}
static inline bool oracle_keep(const sh2z_shade_plane_t *p,const int32_t eye[3],uint32_t err,unsigned mut) {
    int64_t dot=-(int64_t)p->plane,margin=3;
    for(int k=0;k<3;++k) {
        const int64_t n=p->normal[k],dist=(int64_t)eye[k]-p->first[k];
        dot+=n*eye[k];
        margin+=(dist<0?-dist:dist)+((mut&MUT_NO_EYE_ERROR)?0:(n<0?-n:n)*(int64_t)err);
    }
    if(mut&MUT_NO_MARGIN)margin=0;
    return dot>=-margin;
}
static inline void oracle_fog_table(uint32_t depth[256],int32_t fm,int32_t fo,int32_t pa,int32_t pb,int32_t far_z) {
    const wide_t one=(wide_t)1<<30,numerator=-(wide_t)pb*one;
    depth[0]=0;
    for(int i=1;i<256;++i) {
        const wide_t n=((wide_t)i-fo)*one;
        /* ceil(n / fm), fm > 0, written with floor division */
        const wide_t fl=n>=0?n/fm:-((-n+fm-1)/fm);
        const wide_t required=fl*fm==n?fl:fl+1;
        wide_t d=(wide_t)far_z+1;
        if(required<=-one)d=0;
        else if(required<=one) {
            const wide_t m=(wide_t)pa-required;
            if(m>=0) {
                const wide_t q=numerator/(m+1)+1;
                if(q<=far_z/4)d=q*4;
            }
        }
        depth[i]=(uint32_t)d;
    }
}
static inline unsigned oracle_fade(const uint32_t depth[256],int32_t z) {
    /* the source's search: lo = 0, hi = 256, halve while lo + 1 < hi */
    unsigned lo=0,hi=256;const uint32_t u=z>0?(uint32_t)z:0u;
    while(lo+1u<hi){const unsigned mid=(lo+hi)/2u;if(u>=depth[mid])lo=mid;else hi=mid;}
    return lo;
}
static inline unsigned oracle_fade_mut(const sh2z_shade_fog_t *fog,int32_t z,unsigned mut) {
    if(!(mut&MUT_BUCKET_ONLY))return oracle_fade(fog->depth,z);
    const uint32_t u=z>0?(uint32_t)z:0u,b=u>>fog->shift;
    return b>255u?255u:fog->bucket[b];
}
static inline uint16_t oracle_offset(unsigned fade,const int16_t s[3],const int16_t b[3],unsigned mut) {
    unsigned packed=0x8000u;
    for(int k=0;k<3;++k) {
        const int64_t x=(int64_t)fade*s[k]-b[k];
        const int64_t v=clamp64(16+((mut&MUT_FLOOR255)?fdiv(x,255):x/255),0,31);
        packed|=(unsigned)v<<(5*k);
    }
    return (uint16_t)packed;
}
static inline uint16_t oracle_mix(unsigned fade,const uint8_t *nr,const uint8_t *fr,unsigned mut) {
    unsigned packed=0x8000u;
    for(int k=0;k<3;++k) {
        const int64_t v=(((int64_t)nr[k]*(255-(int64_t)fade)+(int64_t)fr[k]*fade)*31+((mut&MUT_MIX_NO_ROUND)?0:32512))/65025;
        packed|=(unsigned)clamp64(v,0,31)<<(5*k);
    }
    return (uint16_t)packed;
}

#endif
