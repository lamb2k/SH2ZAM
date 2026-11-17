/* Independent oracle for the vlc-1 kernels (vlc_experimental.h), shared by
 * tests/test_vlc.c and the SH-2 model harness. Written from the header's
 * rules with no code shared with the library and with different structures:
 * Huffman by sorting the live nodes at every merge and relabelling leaf
 * groups, the length limit by explicit tuple comparisons over all symbols,
 * canonical codes by sorting (length, symbol), the table by searching for the
 * code that matches each index, decoding one bit at a time, and the reader
 * state from the number of bits consumed. Mutation flags select deliberately
 * wrong rules (negative controls). Host only. MIT; SH2ZAM Contributors. */
#ifndef SH2ZAM_VLC_ORACLE_H
#define SH2ZAM_VLC_ORACLE_H
#include "sh2zam/vlc_experimental.h"
#include <stdlib.h>
#include <string.h>

enum {
    VO_TIE_HIGH=1,         /* Huffman ties go to the higher id */
    VO_LENGTHEN_FREQ=2,    /* step 3 takes the greatest frequency */
    VO_SHORTEN_LEAST=4,    /* step 4 takes the smallest frequency */
    VO_CANON_DESC=8,       /* canonical order (length, -symbol) */
    VO_OVERRUN_EARLY=16,   /* ok only while fewer than 16*units bits are consumed */
    VO_MSB_STREAM=32,      /* bits taken most significant first within a unit */
    VO_PAD_NOSAT=64,       /* pad does not saturate */
    VO_SINGLE_HALF=128     /* one-symbol table: bit 1 decodes as symbol 0 */
};

/* ---- code lengths ---- */
typedef struct {uint64_t weight;unsigned id;} vo_node_t;
static inline int vo_node_cmp(const void *a,const void *b) {
    const vo_node_t *x=(const vo_node_t *)a,*y=(const vo_node_t *)b;
    if(x->weight!=y->weight)return x->weight<y->weight?-1:1;
    return x->id<y->id?-1:x->id>y->id;
}
static inline int vo_node_cmp_high(const void *a,const void *b) {
    const vo_node_t *x=(const vo_node_t *)a,*y=(const vo_node_t *)b;
    if(x->weight!=y->weight)return x->weight<y->weight?-1:1;
    return x->id>y->id?-1:x->id<y->id;
}
/* Step 1: plain Huffman; len[s] = depth (one symbol: 1). */
static inline void vo_huffman(const uint32_t freq[256],unsigned len[256],unsigned mut) {
    vo_node_t live[256];unsigned nlive=0,group[256],next_id=0;
    for(unsigned s=0;s<256;s++){len[s]=0;group[s]=~0u;}
    for(unsigned s=0;s<256;s++)if(freq[s]){live[nlive].weight=freq[s];live[nlive].id=next_id;group[s]=next_id++;nlive++;}
    if(nlive==1){for(unsigned s=0;s<256;s++)if(freq[s])len[s]=1;return;}
    while(nlive>1) {
        qsort(live,nlive,sizeof live[0],(mut&VO_TIE_HIGH)?vo_node_cmp_high:vo_node_cmp);
        const vo_node_t a=live[0],b=live[1];
        const unsigned id=next_id++;
        for(unsigned s=0;s<256;s++)if(group[s]==a.id||group[s]==b.id){len[s]++;group[s]=id;}
        live[0].weight=a.weight+b.weight;live[0].id=id;
        live[1]=live[nlive-1];nlive--;
    }
}
/* Steps 2-4. */
static inline void vo_limit(const uint32_t freq[256],unsigned len[256],unsigned lmax,unsigned mut) {
    const uint64_t full=(uint64_t)1<<lmax;uint64_t k=0;
    for(unsigned s=0;s<256;s++)if(len[s]){if(len[s]>lmax)len[s]=lmax;k+=full>>len[s];}
    while(k>full) {
        int best=-1;
        for(unsigned s=0;s<256;s++) {
            if(!len[s]||len[s]>=lmax)continue;
            if(best<0){best=(int)s;continue;}
            const unsigned b=(unsigned)best;
            const int freq_better=(mut&VO_LENGTHEN_FREQ)?freq[s]>freq[b]:freq[s]<freq[b];
            if(len[s]>len[b]||(len[s]==len[b]&&(freq_better||(freq[s]==freq[b]&&s<b))))best=(int)s;
        }
        if(best<0)return;
        len[best]++;k-=full>>len[best];
    }
    for(;;) {
        int best=-1;
        for(unsigned s=0;s<256;s++) {
            if(len[s]<=1||k+(full>>len[s])>full)continue;
            if(best<0){best=(int)s;continue;}
            const unsigned b=(unsigned)best;
            const int better=(mut&VO_SHORTEN_LEAST)?freq[s]<freq[b]:freq[s]>freq[b];
            if(better)best=(int)s;
        }
        if(best<0)return;
        k+=full>>len[best];len[best]--;
    }
}
/* ---- canonical codes, first stream bit in bit 0 ---- */
static inline unsigned vo_reverse(unsigned v,unsigned bits){unsigned r=0;for(unsigned b=0;b<bits;b++)if(v>>b&1u)r|=1u<<(bits-1u-b);return r;}
static inline void vo_canonical(const unsigned len[256],unsigned code[256],unsigned mut) {
    unsigned sorted[256],m=0;
    for(unsigned s=0;s<256;s++){code[s]=0;if(len[s])sorted[m++]=s;}
    for(unsigned i=1;i<m;i++)for(unsigned j=i;j>0;j--) {   /* insertion sort by (len, symbol) */
        const unsigned a=sorted[j-1],b=sorted[j];
        const int swap=len[a]>len[b]||(len[a]==len[b]&&((mut&VO_CANON_DESC)?a<b:a>b));
        if(!swap)break;
        sorted[j-1]=b;sorted[j]=a;
    }
    unsigned c=0,prev=m?len[sorted[0]]:0;
    for(unsigned i=0;i<m;i++){const unsigned s=sorted[i];c<<=len[s]-prev;prev=len[s];code[s]=vo_reverse(c,len[s]);c++;}
}
static inline uint64_t vo_kraft16(const unsigned len[256]){uint64_t k=0;for(unsigned s=0;s<256;s++)if(len[s])k+=(uint64_t)1<<(16-len[s]);return k;}
static inline unsigned vo_count(const unsigned len[256]){unsigned m=0;for(unsigned s=0;s<256;s++)m+=len[s]!=0;return m;}

/* ---- the expected table ---- */
/* Returns false where the builder must refuse (a group over 7 extra bits or
 * a pool over SH2Z_VLC8_POOL_MAX). t is filled byte for byte otherwise. */
static inline bool vo_table(sh2z_vlc8_t *t,const unsigned len[256],const unsigned code[256],unsigned w,unsigned lmax,unsigned mut) {
    memset(t,0,sizeof *t);
    t->w=(uint8_t)w;t->lmax=(uint8_t)lmax;
    const unsigned size=1u<<w;
    if(vo_count(len)==1) {
        unsigned only=0;for(unsigned s=0;s<256;s++)if(len[s])only=s;
        for(unsigned e=0;e<size;e++)t->s[e]=(uint8_t)(((mut&VO_SINGLE_HALF)&&(e&1u))?0u:only);
        return true;
    }
    unsigned pool=0;
    for(unsigned e=0;e<size;e++) {
        int direct=-1;unsigned xb=0;
        for(unsigned s=0;s<256;s++) {
            if(!len[s])continue;
            if(len[s]<=w){if((e&((1u<<len[s])-1u))==code[s])direct=(int)s;}
            else if((code[s]&(size-1u))==e&&len[s]-w>xb)xb=len[s]-w;
        }
        if(direct>=0){t->s[e]=(uint8_t)direct;t->n[e]=(uint8_t)(4u*(len[direct]-1u));continue;}
        if(!xb)continue;
        if(xb>7||pool+(1u<<xb)>SH2Z_VLC8_POOL_MAX)return false;
        t->s[e]=(uint8_t)(pool&255u);t->n[e]=(uint8_t)(0x80u|xb<<4|pool>>8);
        for(unsigned k=0;k<(1u<<xb);k++) {
            const unsigned bits=k<<w|e;
            for(unsigned s=0;s<256;s++)if(len[s]>w&&(bits&((1u<<len[s])-1u))==code[s])t->pool[pool+k]=(uint16_t)(s<<8|4u*(len[s]-1u));
        }
        pool+=1u<<xb;
    }
    t->pool_len=(uint16_t)pool;
    return true;
}
/* The whole builder: freq -> lengths, codes and table; false = refuse. */
static inline bool vo_build(sh2z_vlc8_t *t,const uint32_t freq[256],unsigned w,unsigned lmax,unsigned len[256],unsigned code[256],unsigned mut) {
    unsigned m=0;for(unsigned s=0;s<256;s++)m+=freq[s]!=0;
    if(w<1||w>8||lmax<w||lmax>16||!m||m>(1u<<lmax))return false;
    vo_huffman(freq,len,mut);vo_limit(freq,len,lmax,mut);vo_canonical(len,code,mut);
    return vo_table(t,len,code,w,lmax,mut);
}
/* Byte-for-byte comparison of the defined part of two tables. */
static inline bool vo_same_table(const sh2z_vlc8_t *a,const sh2z_vlc8_t *b) {
    if(a->w!=b->w||a->lmax!=b->lmax||a->pool_len!=b->pool_len)return false;
    if(memcmp(a->s,b->s,256)||memcmp(a->n,b->n,256))return false;
    return !memcmp(a->pool,b->pool,2u*a->pool_len);
}
/* Table safety rule, restated. */
static inline bool vo_check(const sh2z_vlc8_t *t) {
    if(t->w<1||t->w>8||t->pool_len>SH2Z_VLC8_POOL_MAX)return false;
    for(unsigned e=0;e<(1u<<t->w);e++) {
        const unsigned f=t->n[e];
        if(!(f&0x80u)){if(f%4u||f/4u+1u>16u)return false;continue;}
        const unsigned start=(f&15u)*256u+t->s[e],count=1u<<((f>>4)&7u);
        if(start+count>t->pool_len)return false;
        for(unsigned k=0;k<count;k++){const unsigned g=t->pool[start+k]%256u;if(g%4u||g/4u+1u>16u)return false;}
    }
    return true;
}

/* ---- streams ---- */
static inline unsigned vo_bit(const uint8_t *s,size_t units,uint64_t i,unsigned mut) {
    if(i>=(uint64_t)16*units)return 0;
    const uint64_t u=i/16;unsigned b=(unsigned)(i%16);
    if(mut&VO_MSB_STREAM)b=15u-b;
    return ((unsigned)s[2*u+b/8]>>(b%8))&1u;
}
/* Append code bits, first bit first; returns the new bit count. */
static inline uint64_t vo_put(uint8_t *s,uint64_t at,unsigned code,unsigned len) {
    for(unsigned b=0;b<len;b++,at++){if(code>>b&1u)s[2*(at/16)+(at%16)/8]|=(uint8_t)(1u<<(at%8));}
    return at;
}
/* Reference decoder: one bit at a time against the (length, code) pairs.
 * Returns the number of bits consumed (zeros past the end). */
static inline uint64_t vo_decode(const unsigned len[256],const unsigned code[256],const uint8_t *s,size_t units,uint64_t start,uint8_t *out,size_t n,unsigned mut) {
    static int16_t map[17][65536];static unsigned map_len[256],map_code[256];static int map_ok=0;
    if(!map_ok){for(unsigned l=0;l<17;l++)for(unsigned v=0;v<65536;v++)map[l][v]=-1;memset(map_len,0,sizeof map_len);}
    if(!map_ok||memcmp(map_len,len,sizeof map_len)||memcmp(map_code,code,sizeof map_code)) {
        for(unsigned x=0;x<256;x++)if(map_len[x])map[map_len[x]][map_code[x]]=-1;
        for(unsigned x=0;x<256;x++)if(len[x])map[len[x]][code[x]]=(int16_t)x;
        memcpy(map_len,len,sizeof map_len);memcpy(map_code,code,sizeof map_code);map_ok=1;
    }
    const unsigned single=vo_count(len)==1;
    uint64_t at=start;
    for(size_t i=0;i<n;i++) {
        if(single){unsigned only=0;for(unsigned x=0;x<256;x++)if(len[x])only=x;
            out[i]=(uint8_t)(((mut&VO_SINGLE_HALF)&&vo_bit(s,units,at,mut))?0u:only);at++;continue;}
        unsigned v=0,l=0;
        for(;;){v|=vo_bit(s,units,at++,mut)<<l;l++;if(map[l][v]>=0){out[i]=(uint8_t)map[l][v];break;}if(l>=16){out[i]=0;break;}}
    }
    return at;
}
/* Reference interpretation of a table that passes vo_check (any contents). */
static inline uint64_t vo_decode_table(const sh2z_vlc8_t *t,const uint8_t *s,size_t units,uint64_t start,uint8_t *out,size_t n) {
    uint64_t at=start;
    for(size_t i=0;i<n;i++) {
        unsigned e=0;for(unsigned b=0;b<t->w;b++)e|=vo_bit(s,units,at+b,0)<<b;
        unsigned f=t->n[e],sym=t->s[e];
        if(f&0x80u){const unsigned xb=(f>>4)&7u;unsigned k=0;for(unsigned b=0;b<xb;b++)k|=vo_bit(s,units,at+t->w+b,0)<<b;
            const unsigned v=t->pool[(f&15u)*256u+sym+k];sym=v/256u;f=v%256u;}
        out[i]=(uint8_t)sym;at+=f/4u+1u;
    }
    return at;
}
/* The reader state after `consumed` bits of a `units`-unit stream at s. */
typedef struct {size_t p;uint32_t bb;uint32_t pad;bool ok;} vo_state_t;
static inline vo_state_t vo_state(const uint8_t *s,size_t units,uint64_t consumed,unsigned mut) {
    const uint64_t fetched=(consumed+15)/16+1;   /* units fetched, zeros included */
    const uint64_t real=fetched<units?fetched:units;
    uint64_t pad=fetched-real;
    if(!(mut&VO_PAD_NOSAT)&&pad>2)pad=2;
    const unsigned left=(unsigned)(16*fetched-consumed);
    uint32_t bb=(uint32_t)1<<left;
    for(unsigned b=0;b<left;b++)bb|=(uint32_t)vo_bit(s,units,consumed+b,mut)<<b;
    vo_state_t st;st.p=(size_t)(2*real);st.bb=bb;st.pad=(uint32_t)pad;
    st.ok=(mut&VO_OVERRUN_EARLY)?consumed<(uint64_t)16*units:consumed<=(uint64_t)16*units;
    return st;
}

/* ---- optimal length-limited code (package-merge), for comparison ---- */
typedef struct {uint64_t weight;uint8_t count[256];} vo_item_t;
static inline int vo_item_cmp(const void *a,const void *b){const vo_item_t *x=(const vo_item_t *)a,*y=(const vo_item_t *)b;return x->weight<y->weight?-1:x->weight>y->weight;}
static inline void vo_package_merge(const uint32_t freq[256],unsigned lmax,unsigned len[256]) {
    unsigned m=0;for(unsigned s=0;s<256;s++){len[s]=0;m+=freq[s]!=0;}
    if(m==1){for(unsigned s=0;s<256;s++)if(freq[s])len[s]=1;return;}
    vo_item_t *leaves=(vo_item_t *)calloc(256,sizeof *leaves),*list=(vo_item_t *)calloc(1024,sizeof *list),*next=(vo_item_t *)calloc(1024,sizeof *next);
    unsigned nl=0;for(unsigned s=0;s<256;s++)if(freq[s]){leaves[nl].weight=freq[s];leaves[nl].count[s]=1;nl++;}
    qsort(leaves,nl,sizeof *leaves,vo_item_cmp);
    memcpy(list,leaves,nl*sizeof *list);unsigned n=nl;
    for(unsigned level=1;level<lmax;level++) {
        unsigned k=0;for(unsigned i=0;i<nl;i++)next[k++]=leaves[i];
        for(unsigned i=0;i+1<n;i+=2){vo_item_t p;p.weight=list[i].weight+list[i+1].weight;for(unsigned s=0;s<256;s++)p.count[s]=(uint8_t)(list[i].count[s]+list[i+1].count[s]);next[k++]=p;}
        qsort(next,k,sizeof *next,vo_item_cmp);
        vo_item_t *x=list;list=next;next=x;n=k;
    }
    for(unsigned i=0;i<2*(m-1)&&i<n;i++)for(unsigned s=0;s<256;s++)len[s]+=list[i].count[s];
    free(leaves);free(list);free(next);
}
static inline uint64_t vo_cost(const uint32_t freq[256],const unsigned len[256]){uint64_t c=0;for(unsigned s=0;s<256;s++)c+=(uint64_t)freq[s]*len[s];return c;}
#endif
