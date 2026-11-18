/* Per-symbol variable-length decoding (vlc2_experimental.h, contract tag
 * vlc-2): the decoder's constant block, the table builder and the table
 * check. The decoder itself is inline in the header. No libc.
 * MIT; Copyright (c) 2026 SH2ZAM Contributors. */
#include "sh2zam/vlc2_experimental.h"
#include "sh2zam_sections.h"

const uint32_t sh2z_vlc2_consts[64] SH2ZI_RODATA(sh2z_vlc2_consts)={
    /* 0..15: 2^(32-i) (entry 0 unused) */
    0u,0x80000000u,0x40000000u,0x20000000u,0x10000000u,0x08000000u,0x04000000u,0x02000000u,
    0x01000000u,0x00800000u,0x00400000u,0x00200000u,0x00100000u,0x00080000u,0x00040000u,0x00020000u,
    /* 16..31: 1, selected by an escape field 64 + 4 * xb */
    1u,1u,1u,1u,1u,1u,1u,1u,1u,1u,1u,1u,1u,1u,1u,1u,
    /* 32..47: 2^xb - 1 */
    0x0000u,0x0001u,0x0003u,0x0007u,0x000Fu,0x001Fu,0x003Fu,0x007Fu,
    0x00FFu,0x01FFu,0x03FFu,0x07FFu,0x0FFFu,0x1FFFu,0x3FFFu,0x7FFFu,
    /* 48..63: 2^count, at byte 0xC0 + 4 * count */
    0x0001u,0x0002u,0x0004u,0x0008u,0x0010u,0x0020u,0x0040u,0x0080u,
    0x0100u,0x0200u,0x0400u,0x0800u,0x1000u,0x2000u,0x4000u,0x8000u,
};

/* 2^i: the SH-2 has no variable shift, and the builder and the check should
 * need no libgcc helper. */
static const uint16_t pow2[16] SH2ZI_RODATA(sh2zi_vlc2_pow2)={1,2,4,8,16,32,64,128,256,512,1024,2048,4096,8192,16384,32768};
static unsigned shr(unsigned v,unsigned n) {
    while(n--)v>>=1;
    return v;
}

/* The build plan shared by sh2z_vlc2_table_size and sh2z_vlc2_build. */
typedef struct {
    uint16_t code[256];   /* code, first stream bit in bit 0 */
    uint8_t xb[128];      /* extra bits of each first-level entry's group, 0 = none */
    uint8_t start[128];   /* group start, in units of 4 bytes from the table */
    unsigned only;        /* the symbol of a one-symbol code, or 256 */
    size_t size;          /* table bytes */
} vlc2_plan_t;

/* Canonical codes and group layout for valid lengths; false if the lengths
 * are not a complete code (or one length-1 symbol) or a group would start
 * past byte 1020. */
static bool plan(vlc2_plan_t *pl,const uint8_t *len,unsigned nsym,unsigned w) {
    if(!len||nsym<1u||nsym>256u||w<1u||w>SH2Z_VLC2_WMAX)return false;
    unsigned count[16],next[16],m=0,last=0;
    uint32_t kraft=0;
    for(unsigned l=0;l<16u;l++)count[l]=0;
    for(unsigned s=0;s<nsym;s++) {
        const unsigned l=len[s];
        if(!l)continue;
        if(l>SH2Z_VLC2_LMAX)return false;
        count[l]++;m++;last=s;
        kraft+=pow2[15u-l];
    }
    if(!m)return false;
    if(m==1u) {
        if(len[last]!=1u)return false;
        pl->only=last;
    } else {
        if(kraft!=UINT32_C(0x8000))return false;
        pl->only=256u;
    }
    for(unsigned l=1,c=0;l<16u;l++) {
        c=(c+count[l-1u])<<1;
        next[l]=c;
    }
    const unsigned size=pow2[w],mask=size-1u;
    for(unsigned e=0;e<size;e++)pl->xb[e]=0;
    for(unsigned s=0;s<nsym;s++) {
        const unsigned l=len[s];
        unsigned rev=0;
        if(l) {
            unsigned v=next[l]++;
            for(unsigned b=0;b<l;b++){rev=rev<<1|(v&1u);v>>=1;}
            if(l>w) {
                const unsigned e=rev&mask;
                if(l-w>pl->xb[e])pl->xb[e]=(uint8_t)(l-w);
            }
        }
        pl->code[s]=(uint16_t)rev;
    }
    /* Groups largest first, ties by ascending entry: each group starts at a
     * multiple of its size (at least 2 entries, 4 bytes). */
    unsigned at=size>>1;   /* 4-byte units: the pool follows 2^(w+1) bytes */
    for(unsigned x=SH2Z_VLC2_LMAX;x>=1u;x--) {
        for(unsigned e=0;e<size;e++) {
            if(pl->xb[e]!=x)continue;
            if(at>255u)return false;
            pl->start[e]=(uint8_t)at;
            at+=pow2[x-1u];
        }
    }
    pl->size=(size_t)at*4u;
    return true;
}

size_t sh2z_vlc2_table_size(const uint8_t *len,unsigned nsym,unsigned w) {
    vlc2_plan_t pl;
    return plan(&pl,len,nsym,w)?pl.size:0u;
}

size_t sh2z_vlc2_build(uint8_t *t,size_t capacity,const uint8_t *len,unsigned nsym,unsigned w,uint16_t *code_out) {
    vlc2_plan_t pl;
    if(!t||!plan(&pl,len,nsym,w)||pl.size>capacity)return 0;
    const unsigned size=pow2[w],mask=size-1u;
    if(code_out)for(unsigned s=0;s<nsym;s++)code_out[s]=len[s]?pl.code[s]:0u;
    if(pl.only<256u) {
        for(unsigned e=0;e<size;e++){t[e]=(uint8_t)pl.only;t[size+e]=4u;}
        return pl.size;
    }
    for(unsigned e=0;e<size;e++)if(pl.xb[e]){t[e]=pl.start[e];t[size+e]=(uint8_t)(64u+4u*pl.xb[e]);}
    for(unsigned s=0;s<nsym;s++) {
        const unsigned l=len[s],code=pl.code[s];
        if(!l)continue;
        if(l<=w) {
            for(unsigned e=code;e<size;e+=pow2[l]){t[e]=(uint8_t)s;t[size+e]=(uint8_t)(4u*l);}
        } else {
            const unsigned e=code&mask,group=4u*pl.start[e],n=pow2[pl.xb[e]];
            for(unsigned k=shr(code,w);k<n;k+=pow2[l-w]){t[group+2u*k]=(uint8_t)s;t[group+2u*k+1u]=(uint8_t)(4u*l);}
        }
    }
    return pl.size;
}

static bool field_ok(unsigned n) {
    return !(n&3u)&&n>=4u&&n<=4u*SH2Z_VLC2_LMAX;
}

unsigned sh2z_vlc2_check(const uint8_t *t,size_t size,unsigned w) {
    if(!t||w<1u||w>SH2Z_VLC2_WMAX)return 0;
    const unsigned first=pow2[w];
    if(size<2u*first)return 0;
    unsigned longest=0;
    for(unsigned e=0;e<first;e++) {
        const unsigned n=t[first+e];
        if(n<64u) {
            if(!field_ok(n))return 0;
            if(n>longest)longest=n;
            continue;
        }
        if(n&3u||n>124u)return 0;
        const unsigned xb=(n-64u)>>2;
        if(xb<1u||w+xb>SH2Z_VLC2_LMAX)return 0;
        const size_t group=4u*(size_t)t[e],entries=pow2[xb];
        if(group>size||2u*entries>size-group)return 0;
        for(size_t k=0;k<entries;k++) {
            const unsigned n2=t[group+2u*k+1u];
            if(!field_ok(n2))return 0;
            if(n2>longest)longest=n2;
        }
    }
    return longest>>2;
}
