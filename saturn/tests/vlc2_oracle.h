/* Independent oracle for the vlc-2 kernels (vlc2_experimental.h), shared by
 * tests/test_vlc2.c and the SH-2 model harness. Written from the header's
 * rules with no code shared with the library and with different structures:
 * canonical codes by walking lengths and symbols, the first level by
 * searching for the code that matches each index, groups ordered by an
 * explicit sort, decoding one bit at a time, the reader state by simulating
 * the refill rule, and the table check restated. Mutation flags select
 * deliberately wrong rules (negative controls). Host only. MIT; SH2ZAM
 * Contributors. */
#ifndef SH2ZAM_VLC2_ORACLE_H
#define SH2ZAM_VLC2_ORACLE_H
#include "sh2zam/vlc2_experimental.h"
#include <stdlib.h>
#include <string.h>

enum {
    VT_MSB=1,           /* bits taken most significant first within a byte */
    VT_CANON_DESC=2,    /* canonical order (length, -symbol) */
    VT_GROUP_ASC=4,     /* groups placed smallest first */
    VT_REFILL_LE16=8,   /* refill while 16 or fewer bits are left */
    VT_REACH_SHORT=16,  /* bound one unit short */
    VT_ESC_SHIFT=32,    /* group index taken one bit late */
    VT_ODD_NOALIGN=64,  /* odd start: read two bytes, do not align */
    VT_SINGLE_ZERO=128  /* one-symbol code consumes no bit */
};

/* ---- codes ---- */
/* 0 = invalid, 1 = complete code, 2 = one symbol of length 1. */
static inline int vt_kind(const uint8_t *len,unsigned nsym) {
    if(nsym<1||nsym>256)return 0;
    unsigned m=0,only=0;
    unsigned long long k=0;   /* in units of 2^-15 */
    for(unsigned s=0;s<nsym;s++){if(!len[s])continue;if(len[s]>15)return 0;m++;only=s;k+=1ull<<(15-len[s]);}
    if(m==1)return len[only]==1?2:0;
    return m>1&&k==(1ull<<15)?1:0;
}
static inline unsigned vt_rev(unsigned v,unsigned n){unsigned r=0;for(unsigned b=0;b<n;b++)r|=((v>>(n-1-b))&1u)<<b;return r;}
/* code[s]: first stream bit in bit 0. */
static inline void vt_codes(const uint8_t *len,unsigned nsym,unsigned *code,unsigned mut) {
    unsigned c=0;
    for(unsigned s=0;s<nsym;s++)code[s]=0;
    for(unsigned l=1;l<=15;l++) {
        for(unsigned i=0;i<nsym;i++) {
            const unsigned s=(mut&VT_CANON_DESC)?nsym-1-i:i;
            if(len[s]==l)code[s]=vt_rev(c++,l);
        }
        c<<=1;
    }
}
static inline int vt_match(unsigned bits,unsigned code,unsigned l){return (bits&((1u<<l)-1u))==code;}

/* ---- the expected table: returns its size, 0 = refuse ---- */
typedef struct {unsigned xb,e;} vt_group_t;
static inline int vt_group_cmp(const void *a,const void *b) {
    const vt_group_t *x=(const vt_group_t *)a,*y=(const vt_group_t *)b;
    if(x->xb!=y->xb)return x->xb>y->xb?-1:1;
    return x->e<y->e?-1:x->e>y->e;
}
static inline int vt_group_cmp_asc(const void *a,const void *b){return -vt_group_cmp(a,b);}
static inline size_t vt_table(uint8_t *t,size_t cap,const uint8_t *len,unsigned nsym,unsigned w,unsigned *code,unsigned mut) {
    if(w<1||w>7)return 0;
    const int kind=vt_kind(len,nsym);
    if(!kind)return 0;
    vt_codes(len,nsym,code,mut);
    const unsigned first=1u<<w;
    static uint8_t b[1024+65536];   /* a group may start at byte 1020; every byte used is written */
    if(kind==2) {
        unsigned only=0;for(unsigned s=0;s<nsym;s++)if(len[s])only=s;
        for(unsigned e=0;e<first;e++){b[e]=(uint8_t)only;b[first+e]=(mut&VT_SINGLE_ZERO)?0:4;}
    } else {
        vt_group_t g[128];unsigned ng=0;
        for(unsigned e=0;e<first;e++) {
            int direct=-1;unsigned xb=0;
            for(unsigned s=0;s<nsym;s++) {
                if(!len[s])continue;
                if(len[s]<=w){if(vt_match(e,code[s],len[s]))direct=(int)s;}
                else if((code[s]&(first-1))==e&&len[s]-w>xb)xb=len[s]-w;
            }
            if(direct>=0){b[e]=(uint8_t)direct;b[first+e]=(uint8_t)(4*len[direct]);}
            else{g[ng].xb=xb;g[ng].e=e;ng++;}
        }
        qsort(g,ng,sizeof g[0],(mut&VT_GROUP_ASC)?vt_group_cmp_asc:vt_group_cmp);
        size_t at=2u*first;
        for(unsigned i=0;i<ng;i++) {
            const unsigned e=g[i].e,xb=g[i].xb;
            if(at%4||at/4>255){return 0;}
            b[e]=(uint8_t)(at/4);b[first+e]=(uint8_t)(64+4*xb);
            for(unsigned k=0;k<(1u<<xb);k++) {
                const unsigned bits=k<<w|e;
                for(unsigned s=0;s<nsym;s++)if(len[s]>w&&vt_match(bits,code[s],len[s])){b[at+2*k]=(uint8_t)s;b[at+2*k+1]=(uint8_t)(4*len[s]);}
            }
            at+=2u<<xb;
        }
        if(at>cap){return 0;}
        memcpy(t,b,at);
        return at;
    }
    if(2u*first>cap){return 0;}
    memcpy(t,b,2u*first);
    return 2u*first;
}

/* ---- the table check, restated ---- */
static inline unsigned vt_check(const uint8_t *t,size_t size,unsigned w) {
    if(w<1||w>7||size<(2u<<w))return 0;
    unsigned longest=0;
    for(unsigned e=0;e<(1u<<w);e++) {
        const unsigned f=t[(1u<<w)+e];
        if(f<64){if(f%4||f/4<1||f/4>15)return 0;if(f/4>longest)longest=f/4;continue;}
        if(f>=128||f%4)return 0;
        const unsigned xb=(f-64)/4;
        if(xb==0||w+xb>15)return 0;
        const unsigned long long start=4ull*t[e],end=start+(2ull<<xb);
        if(end>size)return 0;
        for(unsigned k=0;k<(1u<<xb);k++){const unsigned g=t[start+2*k+1];if(g%4||g/4<1||g/4>15)return 0;if(g/4>longest)longest=g/4;}
    }
    return longest;
}

/* ---- streams ---- */
/* Bit i of the byte stream s (bits past `bytes` read as zero). */
static inline unsigned vt_bit(const uint8_t *s,size_t bytes,unsigned long long i,unsigned mut) {
    const unsigned long long by=i/8;
    if(by>=bytes)return 0;
    const unsigned b=(unsigned)(i%8);
    return (s[by]>>((mut&VT_MSB)?7-b:b))&1u;
}
/* Append a code, first bit first; returns the new bit position. */
static inline unsigned long long vt_put(uint8_t *s,unsigned long long at,unsigned code,unsigned l) {
    for(unsigned b=0;b<l;b++,at++)if((code>>b)&1u)s[at/8]|=(uint8_t)(1u<<(at%8));
    return at;
}
/* Decode one symbol at bit `at` by matching codes one bit at a time;
 * returns the symbol and advances *at. */
static inline unsigned vt_decode(const uint8_t *len,const unsigned *code,unsigned nsym,const uint8_t *s,size_t bytes,unsigned long long *at,unsigned mut) {
    if(vt_kind(len,nsym)==2) {
        unsigned only=0;for(unsigned x=0;x<nsym;x++)if(len[x])only=x;
        if(!(mut&VT_SINGLE_ZERO))(*at)++;
        return only;
    }
    unsigned v=0;
    for(unsigned l=1;l<=15;l++) {
        v|=vt_bit(s,bytes,(*at)++,mut)<<(l-1);
        for(unsigned x=0;x<nsym;x++)if(len[x]==l&&code[x]==v)return x;
    }
    return 0xFFFFu;   /* not a prefix code: cannot happen for valid lengths */
}
/* Interpret a table that passes vt_check, at bit `at`. */
static inline unsigned vt_decode_table(const uint8_t *t,unsigned w,const uint8_t *s,size_t bytes,unsigned long long *at,unsigned mut) {
    unsigned e=0;
    for(unsigned b=0;b<w;b++)e|=vt_bit(s,bytes,*at+b,mut)<<b;
    unsigned sym=t[e],f=t[(1u<<w)+e];
    if(f>=64) {
        const unsigned xb=(f-64)/4,skip=(mut&VT_ESC_SHIFT)?1u:0u;
        unsigned k=0;
        for(unsigned b=0;b<xb;b++)k|=vt_bit(s,bytes,*at+w+skip+b,mut)<<b;
        const unsigned g=4u*sym+2u*k;
        sym=t[g];f=t[g+1];
    }
    *at+=f/4;
    return sym;
}

/* ---- reader state after `consumed` bits from byte `offset` ---- */
typedef struct {uint32_t bits,cnt;size_t end;} vt_state_t;
/* `odd`: whether stream + offset is an odd address. */
static inline size_t vt_end(size_t offset,int odd,unsigned long long consumed,unsigned mut) {
    size_t end=offset+((odd&&!(mut&VT_ODD_NOALIGN))?3:2);
    const unsigned threshold=(mut&VT_REFILL_LE16)?17:16;
    while(8ull*(end-offset)<consumed+threshold)end+=2;
    return end;
}
static inline vt_state_t vt_state(const uint8_t *s,size_t bytes,size_t offset,int odd,unsigned long long consumed,unsigned mut) {
    vt_state_t st;
    st.end=vt_end(offset,odd,consumed,mut);
    const unsigned left=(unsigned)(8ull*(st.end-offset)-consumed);
    st.bits=0;
    for(unsigned b=0;b<left&&b<32;b++)st.bits|=(uint32_t)vt_bit(s,bytes,8ull*offset+consumed+b,mut)<<b;
    st.cnt=UINT32_C(0x7FFFFFC0)+4u*left;
    return st;
}
/* Bytes read (from a 2-byte-aligned base) once `bits` are consumed. */
static inline size_t vt_reach(size_t offset,unsigned long long bits,unsigned mut) {
    size_t r=vt_end(offset,(int)(offset&1),bits,0);
    if(mut&VT_REACH_SHORT)r-=2;
    return r;
}
#endif
