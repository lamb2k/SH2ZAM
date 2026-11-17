/* Variable-length decoding of byte symbols (vlc_experimental.h, contract tag
 * vlc-1): bounded marker-bit reader, length-limited canonical code builder,
 * two-level table decoder in C and in SH-2 assembly. No libc.
 * MIT; Copyright (c) 2026 SH2ZAM Contributors. */
#include "sh2zam/vlc_experimental.h"

#if defined(__sh__) && defined(__SH2__) && (!defined(SH2Z_FORCE_PORTABLE) || !SH2Z_FORCE_PORTABLE)
#define SH2ZI_VLC_NATIVE 1
#else
#define SH2ZI_VLC_NATIVE 0
#endif

/* ---- reader ---- */

void sh2z_vlc_reader_init(sh2z_vlc_reader_t *r,const void *stream,size_t units) {
    const uint8_t *p=(const uint8_t *)stream;
    if(units) {
        r->bb=(uint32_t)p[0]|(uint32_t)p[1]<<8|UINT32_C(1)<<16;
        r->p=p+2;
        r->end=p+2u*units;
        r->pad=0;
    } else {
        r->bb=UINT32_C(1)<<16;      /* one zero unit: nothing real to read */
        r->p=p;
        r->end=p;
        r->pad=1;
    }
}

bool sh2z_vlc_reader_ok(const sh2z_vlc_reader_t *r) {
    return r->pad<2u;
}

/* ---- portable decoder ---- */

/* bb in [1, 0xFFFF]: the marker bit, 2^floor(log2 bb). */
static uint32_t marker_bit(uint32_t bb) {
    bb|=bb>>1;bb|=bb>>2;bb|=bb>>4;bb|=bb>>8;
    return bb-(bb>>1);
}

bool sh2z_vlc8_decode_portable(const sh2z_vlc8_t *t,sh2z_vlc_reader_t *r,uint8_t *dst,size_t n) {
    const unsigned w=t->w;
    const uint32_t mask=(UINT32_C(1)<<w)-1u;
    const uint8_t *p=r->p,*const end=r->end;
    uint32_t bb=r->bb,pad=r->pad;
    for(size_t i=0;i<n;i++) {
        const uint32_t e=bb&mask;
        unsigned field=t->n[e],sym=t->s[e];
        if(field>=0x80u) {
            const unsigned group=(field&15u)<<8|sym;
            const unsigned v=t->pool[group+((bb>>w)&((1u<<((field>>4)&7u))-1u))];
            sym=v>>8;
            field=v&255u;
        }
        dst[i]=(uint8_t)sym;
        bb>>=(field>>2)+1u;
        if(bb<0x10000u) {
            /* fewer than 16 bits left: append one unit above them (move the
             * marker up 16 places); a zero unit past the end of the stream */
            const uint32_t h=marker_bit(bb);
            uint32_t unit=0;
            if(p!=end) {
                unit=(uint32_t)p[0]|(uint32_t)p[1]<<8;
                p+=2;
            } else if(pad<2u) {
                pad++;
            }
            bb+=h*(unit+0xFFFFu);
        }
    }
    r->p=p;r->bb=bb;r->pad=pad;
    return pad<2u;
}

/* ---- SH-2 decoder ---- */

#if SH2ZI_VLC_NATIVE
_Static_assert(offsetof(sh2z_vlc8_t,s)==4&&offsetof(sh2z_vlc8_t,n)==260&&offsetof(sh2z_vlc8_t,pool)==516,"vlc table layout");
_Static_assert(offsetof(sh2z_vlc_reader_t,bb)==4&&offsetof(sh2z_vlc_reader_t,end)==8&&offsetof(sh2z_vlc_reader_t,pad)==12,"vlc reader layout");
uint32_t sh2zi_vlc8_run(const sh2z_vlc8_t *t,sh2z_vlc_reader_t *r,uint8_t *dst,size_t n);
/* sh2zi_vlc8_run(t r4, r r5, dst r6, n r7 > 0), r0 = 1 if pad < 2 after the
 * call. Same bytes and reader state as sh2z_vlc8_decode_portable; the stream
 * pointer must be even (MOV.W).
 *
 * Loop registers: r1 bb, r2 symbol, r4 0x10000, r5 dst, r7 count, r8 s[],
 * r9 n[], r10 F, r11 stream pointer, r12 2^w - 1, r13 2^(32 - w); r0, r3, r6
 * scratch. Stack: @r15 reader, @(4,r15) pool, @(8,r15) end, @(12,r15) pad.
 * F[k] = 2^(31 - k): the high word of DMULU.L bb, F[k] is bb >> (k + 1), and
 * a length field 4 * (len - 1) is the byte offset of F[len - 1].
 *
 * Refill (bb < 0x10000, marker at bit c <= 15, H = 2^c from a table indexed
 * by bb's high or low byte): bb += H * (unit + 0xFFFF), that is, take the
 * marker out, put the unit at bit c and the marker at bit c + 16. MOV.W reads
 * the unit byte-swapped; SWAP.B restores the stream's little-endian order.
 * When the pointer has reached the end nothing is read, the unit is zero and
 * pad counts up to 2.
 *
 * Callee-saved r8-r13, MACH and MACL are saved and restored (DMULU.L and
 * MULU.W write them); r14, GBR and PR are not touched. */
__asm__(
"    .text\n"
"    .align 2\n"
"    .global _sh2zi_vlc8_run\n"
"    .type _sh2zi_vlc8_run,@function\n"
"_sh2zi_vlc8_run:\n"
"    mov.l   r8,@-r15\n"
"    mov.l   r9,@-r15\n"
"    mov.l   r10,@-r15\n"
"    mov.l   r11,@-r15\n"
"    mov.l   r12,@-r15\n"
"    mov.l   r13,@-r15\n"
"    sts.l   mach,@-r15\n"
"    sts.l   macl,@-r15\n"
"    add     #-16,r15\n"
"    mov.l   r5,@r15\n"            /* reader */
"    mov.l   @(8,r5),r0\n"
"    mov.l   r0,@(8,r15)\n"        /* end */
"    mov.l   @(12,r5),r0\n"
"    mov.l   r0,@(12,r15)\n"       /* pad */
"    mov.l   @r5,r11\n"            /* p */
"    mov.l   @(4,r5),r1\n"         /* bb */
"    mov     r6,r5\n"              /* dst */
"    mov     #1,r0\n"
"    shll8   r0\n"                 /* 256 */
"    mov     r4,r8\n"
"    add     #4,r8\n"              /* s = t + 4 */
"    mov     r8,r9\n"
"    add     r0,r9\n"              /* n = t + 260 */
"    mov     r9,r3\n"
"    add     r0,r3\n"
"    mov.l   r3,@(4,r15)\n"        /* pool = t + 516 */
"    mov.b   @r4,r0\n"
"    extu.b  r0,r0\n"
"    shll2   r0\n"                 /* 4 w */
"    mov.l   .Lsh2zi_vlc_mkp,r3\n"
"    mov.l   @(r0,r3),r12\n"       /* 2^w - 1 */
"    mov.l   .Lsh2zi_vlc_fp,r10\n"
"    add     #-4,r0\n"
"    mov.l   @(r0,r10),r13\n"      /* F[w - 1] = 2^(32 - w) */
"    mov     #1,r4\n"
"    shll16  r4\n"                 /* 0x10000 */
".Lsh2zi_vlc_loop:\n"
"    mov     r1,r0\n"
"    and     r12,r0\n"
"    mov.b   @(r0,r8),r2\n"
"    mov.b   @(r0,r9),r0\n"
"    cmp/pz  r0\n"
"    bf      .Lsh2zi_vlc_esc\n"
".Lsh2zi_vlc_cont:\n"
"    mov.l   @(r0,r10),r6\n"
"    mov.b   r2,@r5\n"
"    dmulu.l r1,r6\n"
"    add     #1,r5\n"
"    sts     mach,r1\n"            /* bb >> len */
"    cmp/hs  r4,r1\n"
"    bf      .Lsh2zi_vlc_refill\n"
".Lsh2zi_vlc_back:\n"
"    dt      r7\n"
"    bf      .Lsh2zi_vlc_loop\n"
"    mov.l   @r15,r3\n"
"    mov.l   r11,@r3\n"
"    mov.l   r1,@(4,r3)\n"
"    mov.l   @(12,r15),r0\n"
"    mov.l   r0,@(12,r3)\n"
"    mov     #1,r3\n"
"    cmp/hi  r3,r0\n"              /* T = pad >= 2 */
"    movt    r0\n"
"    xor     #1,r0\n"
"    add     #16,r15\n"
"    lds.l   @r15+,macl\n"
"    lds.l   @r15+,mach\n"
"    mov.l   @r15+,r13\n"
"    mov.l   @r15+,r12\n"
"    mov.l   @r15+,r11\n"
"    mov.l   @r15+,r10\n"
"    mov.l   @r15+,r9\n"
"    rts\n"
"    mov.l   @r15+,r8\n"
/* Second level: r0 = n[e] sign-extended (bit 7 set), r2 = s[e]. Leaves
 * r2 = symbol and r0 = 4 * (len - 1) for .Lsh2zi_vlc_cont. */
".Lsh2zi_vlc_esc:\n"
"    dmulu.l r1,r13\n"             /* MACH = bb >> w */
"    mov     r0,r3\n"
"    shlr2   r0\n"
"    shlr2   r0\n"
"    and     #7,r0\n"
"    shll2   r0\n"                 /* 4 xb */
"    mov.l   .Lsh2zi_vlc_mkp,r6\n"
"    mov.l   @(r0,r6),r6\n"        /* 2^xb - 1 */
"    mov     r3,r0\n"
"    and     #15,r0\n"
"    shll8   r0\n"
"    extu.b  r2,r2\n"
"    or      r2,r0\n"              /* group start */
"    sts     mach,r2\n"
"    and     r2,r6\n"
"    add     r6,r0\n"              /* entry */
"    shll    r0\n"
"    mov.l   @(4,r15),r3\n"
"    mov.w   @(r0,r3),r0\n"
"    extu.w  r0,r2\n"
"    shlr8   r2\n"                 /* symbol */
"    bra     .Lsh2zi_vlc_cont\n"
"    extu.b  r0,r0\n"              /* 4 * (len - 1) */
/* Refill: bb < 0x10000. */
".Lsh2zi_vlc_refill:\n"
"    mov.l   .Lsh2zi_vlc_hp,r6\n"
"    mov     r1,r0\n"
"    shlr8   r0\n"
"    tst     r0,r0\n"
"    bt      1f\n"
"    add     r0,r0\n"
"    mov.w   @(r0,r6),r3\n"
"    bra     2f\n"
"    shll8   r3\n"                 /* H = 2^c, c >= 8 */
"1:  mov     r1,r0\n"
"    add     r0,r0\n"
"    mov.w   @(r0,r6),r3\n"        /* H = 2^c, c < 8 */
"2:  mov.l   @(8,r15),r0\n"
"    cmp/eq  r0,r11\n"
"    bt      3f\n"
"    mov.w   @r11+,r0\n"
"    swap.b  r0,r0\n"
"    extu.w  r0,r0\n"              /* next unit */
"    mulu.w  r3,r0\n"
"    sts     macl,r0\n"
"    add     r0,r1\n"              /* + H * unit */
"    sub     r3,r1\n"              /* - marker */
"    shll16  r3\n"
"    bra     .Lsh2zi_vlc_back\n"
"    add     r3,r1\n"              /* + new marker */
"3:  sub     r3,r1\n"              /* end of stream: a zero unit */
"    shll16  r3\n"
"    add     r3,r1\n"
"    mov.l   @(12,r15),r0\n"
"    mov     #1,r3\n"
"    cmp/hi  r3,r0\n"
"    bt      .Lsh2zi_vlc_back\n"   /* pad saturates at 2 */
"    add     #1,r0\n"
"    bra     .Lsh2zi_vlc_back\n"
"    mov.l   r0,@(12,r15)\n"
"    .align 2\n"
".Lsh2zi_vlc_mkp: .long .Lsh2zi_vlc_mk\n"
".Lsh2zi_vlc_fp:  .long .Lsh2zi_vlc_f\n"
".Lsh2zi_vlc_hp:  .long .Lsh2zi_vlc_h\n"
"    .size _sh2zi_vlc8_run,.-_sh2zi_vlc8_run\n"
"    .pushsection .rodata\n"
"    .align 2\n"
".Lsh2zi_vlc_mk:\n"                /* 2^k - 1, k = 0..8 */
"    .long 0,1,3,7,15,31,63,127,255\n"
".Lsh2zi_vlc_f:\n"                 /* F[k] = 2^(31 - k), k = 0..15 */
"    .long 0x80000000,0x40000000,0x20000000,0x10000000\n"
"    .long 0x08000000,0x04000000,0x02000000,0x01000000\n"
"    .long 0x00800000,0x00400000,0x00200000,0x00100000\n"
"    .long 0x00080000,0x00040000,0x00020000,0x00010000\n"
".Lsh2zi_vlc_h:\n"                 /* largest power of two <= b, b = 0..255 */
"    .short 0,1,2,2,4,4,4,4\n"
"    .rept 8\n"
"    .short 8\n"
"    .endr\n"
"    .rept 16\n"
"    .short 16\n"
"    .endr\n"
"    .rept 32\n"
"    .short 32\n"
"    .endr\n"
"    .rept 64\n"
"    .short 64\n"
"    .endr\n"
"    .rept 128\n"
"    .short 128\n"
"    .endr\n"
"    .popsection\n"
);
#endif

bool sh2z_vlc8_decode(const sh2z_vlc8_t *t,sh2z_vlc_reader_t *r,uint8_t *dst,size_t n) {
#if SH2ZI_VLC_NATIVE
    if(n&&!((uintptr_t)r->p&1u))return sh2zi_vlc8_run(t,r,dst,n)!=0;
#endif
    return sh2z_vlc8_decode_portable(t,r,dst,n);
}

/* ---- table check ---- */

static bool length_field_ok(unsigned field) {
    return !(field&3u)&&field<=4u*(SH2Z_VLC8_LMAX_LIMIT-1u);
}

bool sh2z_vlc8_check(const sh2z_vlc8_t *t) {
    if(!t||t->w<1u||t->w>8u||t->pool_len>SH2Z_VLC8_POOL_MAX)return false;
    for(unsigned e=0;e<(1u<<t->w);e++) {
        const unsigned field=t->n[e];
        if(field<0x80u) {
            if(!length_field_ok(field))return false;
            continue;
        }
        const unsigned group=(field&15u)<<8|t->s[e],size=1u<<((field>>4)&7u);
        if(group+size>t->pool_len)return false;
        for(unsigned k=0;k<size;k++)if(!length_field_ok(t->pool[group+k]&255u))return false;
    }
    return true;
}

/* ---- builder ---- */

/* One stable counting-sort pass of m symbols from src to dst on the byte of
 * the frequency at `shift` (a constant after inlining: no variable shift). */
static inline void radix_pass(const uint32_t freq[256],const uint8_t *src,uint8_t *dst,unsigned m,unsigned shift) {
    uint16_t start[256];
    for(unsigned b=0;b<256u;b++)start[b]=0;
    for(unsigned i=0;i<m;i++)start[(freq[src[i]]>>shift)&255u]++;
    for(unsigned b=0,sum=0;b<256u;b++){const unsigned c=start[b];start[b]=(uint16_t)sum;sum+=c;}
    for(unsigned i=0;i<m;i++){const unsigned s=src[i];dst[start[(freq[s]>>shift)&255u]++]=(uint8_t)s;}
}
/* order[0..m) = the symbols with a non-zero frequency, ascending by
 * (frequency, symbol): a stable radix sort of the symbol-ordered list, one
 * pass per frequency byte that is not zero in every symbol (insertion sort
 * for up to 32 symbols). */
static unsigned sort_symbols(const uint32_t freq[256],uint8_t order[256]) {
    uint8_t other[256];
    unsigned m=0;
    uint32_t any=0;
    for(unsigned s=0;s<256u;s++)if(freq[s]){order[m++]=(uint8_t)s;any|=freq[s];}
    if(m<=32u) {   /* few symbols: stable insertion sort */
        for(unsigned i=1;i<m;i++) {
            const uint8_t x=order[i];
            unsigned j=i;
            for(;j&&freq[order[j-1u]]>freq[x];j--)order[j]=order[j-1u];
            order[j]=x;
        }
        return m;
    }
    radix_pass(freq,order,other,m,0);
    if(any>>8) {
        radix_pass(freq,other,order,m,8);
        if(any>>16) {
            radix_pass(freq,order,other,m,16);
            if(any>>24){radix_pass(freq,other,order,m,24);return m;}
            for(unsigned i=0;i<m;i++)order[i]=other[i];
        }
        return m;
    }
    for(unsigned i=0;i<m;i++)order[i]=other[i];
    return m;
}

/* Huffman lengths with the (weight, id) rule of the header, as a two-queue
 * merge: leaves in (frequency, symbol) order and merged nodes in creation
 * order (their weights never decrease) are each sorted by (weight, id), and a
 * leaf wins a tie with a merged node because its id is lower. m >= 2. */
static void huffman(const uint32_t freq[256],const uint8_t order[256],unsigned m,uint8_t len[256]) {
    uint64_t weight[255];
    uint8_t up[255],leaf_up[256],depth[255];
    unsigned leaf=0,node=0;
    for(unsigned k=0;k+1u<m;k++) {
        uint64_t sum=0;
        for(unsigned j=0;j<2u;j++) {
            if(leaf<m&&(node>=k||(uint64_t)freq[order[leaf]]<=weight[node])) {
                sum+=freq[order[leaf]];
                leaf_up[leaf++]=(uint8_t)k;
            } else {
                sum+=weight[node];
                up[node++]=(uint8_t)k;
            }
        }
        weight[k]=sum;
    }
    depth[m-2u]=0;   /* the last node made is the root */
    for(unsigned k=m-2u;k-->0;)depth[k]=(uint8_t)(depth[up[k]]+1u);
    for(unsigned i=0;i<m;i++)len[order[i]]=(uint8_t)(depth[leaf_up[i]]+1u);
}

/* Steps 2-4 of the header's length rule. Ties fall to the first symbol in
 * (frequency, symbol) order when lengthening and to the smallest symbol of
 * the greatest frequency when shortening, as written there. */
static bool limit_lengths(const uint32_t freq[256],const uint8_t order[256],unsigned m,uint8_t len[256],unsigned lmax) {
    const uint32_t full=UINT32_C(1)<<lmax;
    uint32_t kraft=0;
    unsigned count[17];
    for(unsigned l=0;l<17u;l++)count[l]=0;
    for(unsigned i=0;i<m;i++) {
        const unsigned s=order[i];
        if(len[s]>lmax)len[s]=(uint8_t)lmax;
        count[len[s]]++;
        kraft+=full>>len[s];
    }
    /* first[l]: no symbol before order[first[l]] has length l. A symbol that
     * leaves length l was the first there; one that arrives at l + 1 may be
     * earlier than first[l + 1]. */
    uint16_t first[17];
    for(unsigned l=0;l<17u;l++)first[l]=0;
    while(kraft>full) {
        unsigned l=lmax-1u;
        while(l&&!count[l])l--;
        if(!l)return false;
        unsigned i=first[l];
        while(len[order[i]]!=l)i++;
        len[order[i]]=(uint8_t)(l+1u);
        first[l]=(uint16_t)(i+1u);
        if(i<first[l+1u])first[l+1u]=(uint16_t)i;
        count[l]--;count[l+1u]++;
        kraft-=full>>(l+1u);
    }
    while(kraft<full) {
        const uint32_t room=full-kraft;
        unsigned best=256;
        for(unsigned i=m;i-->0;) {
            const unsigned s=order[i];
            if(best<256u&&freq[s]!=freq[best])break;
            if(len[s]>1u&&(full>>len[s])<=room)best=s;
        }
        if(best==256u)break;
        kraft+=full>>len[best];
        len[best]--;
    }
    return true;   /* with no room left no symbol can be shortened */
}

/* Canonical codes and the two-level table for valid lengths. */
static bool fill_table(sh2z_vlc8_t *t,const uint8_t len[256],unsigned w,unsigned lmax,uint16_t code_out[256]) {
    unsigned count[17],next[17];
    uint16_t code[256];
    uint8_t extra[256];
    unsigned nsym=0,single=0;
    for(unsigned l=0;l<17u;l++)count[l]=0;
    for(unsigned s=0;s<256u;s++)count[len[s]]++;
    count[0]=0;
    for(unsigned l=1,c=0;l<17u;l++) {
        c=(c+count[l-1u])<<1;
        next[l]=c;
    }
    for(unsigned s=0;s<256u;s++) {
        unsigned rev=0;
        if(len[s]) {
            unsigned v=next[len[s]]++;
            for(unsigned b=0;b<len[s];b++){rev=rev<<1|(v&1u);v>>=1;}
            nsym++;single=s;
        }
        code[s]=(uint16_t)rev;
        if(code_out)code_out[s]=(uint16_t)rev;
    }
    const unsigned size=1u<<w,mask=size-1u;
    t->w=(uint8_t)w;t->lmax=(uint8_t)lmax;t->pool_len=0;
    for(unsigned e=0;e<256u;e++){t->s[e]=0;t->n[e]=0;extra[e]=0;}
    if(nsym==1u) {
        /* one symbol, one bit: both bit values decode to it */
        for(unsigned e=0;e<size;e++)t->s[e]=(uint8_t)single;
        return true;
    }
    for(unsigned s=0;s<256u;s++) {
        const unsigned l=len[s];
        if(!l)continue;
        if(l<=w) {
            for(unsigned e=code[s];e<size;e+=1u<<l){t->s[e]=(uint8_t)s;t->n[e]=(uint8_t)(4u*(l-1u));}
        } else {
            const unsigned e=code[s]&mask;
            if(l-w>extra[e])extra[e]=(uint8_t)(l-w);
        }
    }
    unsigned pool=0;
    for(unsigned e=0;e<size;e++) {
        const unsigned x=extra[e];
        if(!x)continue;
        if(x>7u||pool+(1u<<x)>SH2Z_VLC8_POOL_MAX)return false;
        t->s[e]=(uint8_t)(pool&255u);
        t->n[e]=(uint8_t)(0x80u|x<<4|pool>>8);
        pool+=1u<<x;
    }
    t->pool_len=(uint16_t)pool;
    for(unsigned s=0;s<256u;s++) {
        const unsigned l=len[s];
        if(l<=w)continue;
        const unsigned e=code[s]&mask,group=(t->n[e]&15u)<<8|t->s[e],size2=1u<<extra[e];
        for(unsigned k=(unsigned)code[s]>>w;k<size2;k+=1u<<(l-w))t->pool[group+k]=(uint16_t)(s<<8|4u*(l-1u));
    }
    return true;
}

static bool args_ok(const sh2z_vlc8_t *t,const void *in,unsigned w,unsigned lmax) {
    return t&&in&&w>=1u&&w<=8u&&lmax>=w&&lmax<=SH2Z_VLC8_LMAX_LIMIT;
}

bool sh2z_vlc8_build(sh2z_vlc8_t *t,const uint32_t freq[256],unsigned w,unsigned lmax,uint8_t len_out[256],uint16_t code_out[256]) {
    uint8_t order[256],len[256];
    if(!args_ok(t,freq,w,lmax))return false;
    const unsigned m=sort_symbols(freq,order);
    if(!m||m>(1u<<lmax))return false;
    for(unsigned s=0;s<256u;s++)len[s]=0;
    if(m==1u)len[order[0]]=1;
    else huffman(freq,order,m,len);
    if(!limit_lengths(freq,order,m,len,lmax))return false;
    if(len_out)for(unsigned s=0;s<256u;s++)len_out[s]=len[s];
    return fill_table(t,len,w,lmax,code_out);
}

bool sh2z_vlc8_build_lengths(sh2z_vlc8_t *t,const uint8_t len[256],unsigned w,unsigned lmax,uint16_t code_out[256]) {
    if(!args_ok(t,len,w,lmax))return false;
    uint32_t kraft=0;
    unsigned nsym=0,first=0;
    for(unsigned s=0;s<256u;s++) {
        if(!len[s])continue;
        if(len[s]>lmax)return false;
        nsym++;first=len[s];
        kraft+=UINT32_C(0x10000)>>len[s];
    }
    if(nsym==1u?first!=1u:kraft!=UINT32_C(0x10000))return false;
    return fill_table(t,len,w,lmax,code_out);
}
