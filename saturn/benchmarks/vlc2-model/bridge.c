/* SH-2 bridge for the executed-instruction vlc-2 model. MIT; Copyright (c)
 * 2026 SH2ZAM Contributors. Every entry point is called directly by the
 * model harness with the SH-2 calling convention; buffers are written by the
 * harness before each call; nothing here uses libc.
 *
 * Verification entries:
 *   v2_build, v2_check          sh2z_vlc2_build / sh2z_vlc2_check on tbuf
 *   v2_asm_w<w>, v2_c_w<w>      decode n symbols with tbuf at window w
 *                               (sh2z_vlc2_decode, the inline assembly, and
 *                               sh2z_vlc2_decode_portable compiled for SH-2)
 *   v2_mixed_asm, v2_mixed_c    the table changes every symbol (w = 6)
 * Synthetic tile codec (benchmark only; a generic context-modelled 4 bpp
 * tile format, see VLC2.md):
 *   tile_v2, tile_v2c           one tile with sh2z_vlc2_decode / _portable
 *   tile_v2_packed              tile_v2 on byte-packed tiles (odd offsets)
 *   tile_v1                     one tile with one sh2z_vlc8_decode (vlc-1)
 *                               call per symbol, vlc-1 tables
 *   tile_mk                     tile_v2 with a marker-bit reader variant
 *                               (this file only), for comparison
 * Symbol loops (cycles per symbol by the slope between n = 1 and n = 4096):
 *   loop_v2, loop_v2c, loop_mk  one table, symbols stored to out[]
 *   loop_v1                     vlc-1, one call for all n symbols
 *   loop_v1_each                vlc-1, one call per symbol */
#include "sh2zam/vlc2_experimental.h"
#include "sh2zam/vlc_experimental.h"

#define NOINLINE __attribute__((noinline))
enum {TMAX=36864,VSTREAM=16384,OUT_MAX=4608,ARENA=24576,STREAM_MAX=32768,NTILES=128};

/* ---- verification ---- */
uint8_t tbuf[TMAX];
uint8_t lens[256];
uint16_t codes[256];
uint8_t vstream[VSTREAM+8] __attribute__((aligned(4)));
uint8_t out[OUT_MAX+32];
sh2z_vlc2_reader_t rd;
uint16_t mixed_off[OUT_MAX];

NOINLINE uint32_t v2_build(uint32_t nsym,uint32_t w) {
    return (uint32_t)sh2z_vlc2_build(tbuf,TMAX,lens,nsym,w,codes);
}
NOINLINE uint32_t v2_check(uint32_t size,uint32_t w) {
    return sh2z_vlc2_check(tbuf,size,w);
}
/* base 0 or 1: the stream pointer is vstream + base. Returns the bit position. */
#define DECODER(name,fn,W) \
NOINLINE uint32_t name(uint32_t offset,uint32_t n,uint32_t base) { \
    const uint8_t *s=vstream+base; \
    sh2z_vlc2_reader_t r; \
    sh2z_vlc2_begin(&r,s,offset); \
    for(uint32_t i=0;i<n;i++)out[i]=(uint8_t)fn(&r,tbuf,W); \
    rd=r; \
    return (uint32_t)sh2z_vlc2_bitpos(&r,s); \
}
DECODER(v2_asm_w1,sh2z_vlc2_decode,1)
DECODER(v2_asm_w2,sh2z_vlc2_decode,2)
DECODER(v2_asm_w3,sh2z_vlc2_decode,3)
DECODER(v2_asm_w4,sh2z_vlc2_decode,4)
DECODER(v2_asm_w5,sh2z_vlc2_decode,5)
DECODER(v2_asm_w6,sh2z_vlc2_decode,6)
DECODER(v2_asm_w7,sh2z_vlc2_decode,7)
DECODER(v2_c_w1,sh2z_vlc2_decode_portable,1)
DECODER(v2_c_w2,sh2z_vlc2_decode_portable,2)
DECODER(v2_c_w3,sh2z_vlc2_decode_portable,3)
DECODER(v2_c_w4,sh2z_vlc2_decode_portable,4)
DECODER(v2_c_w5,sh2z_vlc2_decode_portable,5)
DECODER(v2_c_w6,sh2z_vlc2_decode_portable,6)
DECODER(v2_c_w7,sh2z_vlc2_decode_portable,7)
/* Symbol i uses the table at tbuf + mixed_off[i]. */
#define MIXED(name,fn) \
NOINLINE uint32_t name(uint32_t offset,uint32_t n,uint32_t base) { \
    const uint8_t *s=vstream+base; \
    sh2z_vlc2_reader_t r; \
    sh2z_vlc2_begin(&r,s,offset); \
    for(uint32_t i=0;i<n;i++)out[i]=(uint8_t)fn(&r,tbuf+mixed_off[i],6); \
    rd=r; \
    return (uint32_t)sh2z_vlc2_bitpos(&r,s); \
}
MIXED(v2_mixed_asm,sh2z_vlc2_decode)
MIXED(v2_mixed_c,sh2z_vlc2_decode_portable)

/* ---- the synthetic tile codec ----
 * A tile is 16 rows of 8 bytes (16x16 pixels at 4 bits, the left pixel in
 * the high nibble), each row two half-rows of 4 bytes. Per row, one row
 * symbol from the table of the previous row symbol (0 before the first
 * row): bits 2h..2h+1 give half-row h's kind: 0 zero, 1 the half-row above
 * (zero above the first row), 2 a dictionary entry (one symbol from the
 * dictionary table, w = 7), 3 raw: four pair symbols, one byte each, each
 * from the pair table of context class16(byte above) | (low nibble of the
 * byte to the left, 0 at the start of the row), class16 = 16 * a class
 * 0..15 of the byte. Tables live in one arena at the offsets in rowmap,
 * dict_off and the context map.
 *
 * Caller-side tuning (not library API): the class table and the context map
 * are stored rotated by 128 entries and read through pointers to their
 * middle, so the SH-2's sign-extending MOV.B and MOV.W index them directly
 * (no EXTU): klass_rot[128 + (int8_t)b] is class16(b) as an int8_t, and
 * map_rot[128 + (int8_t)c] the arena offset of context c. */
uint8_t arena[ARENA];
uint16_t rowmap[16];
int16_t map_rot[256];
uint16_t dict_off;
int8_t klass_rot[256];
uint32_t dict[256];
uint32_t tile_off[NTILES],packed_off[NTILES];
uint8_t stream[STREAM_MAX] __attribute__((aligned(4)));
uint8_t packed[STREAM_MAX] __attribute__((aligned(4)));
/* Two zero words (the row above the first row), then the tile. */
uint32_t tile_words[2+32];
/* vlc-1 tables for the same codes; v1_ctx gives each context's pair table. */
sh2z_vlc8_t v1_row[16],v1_dict,v1_pair[32];
uint8_t v1_ctx[256];
/* The tile's symbols in decode order, for tile_glue. */
uint8_t symlist[256];

/* The caller's decode loop, for any reader R with BEGIN and DEC(r, t, w). */
#define TILE(name,R,BEGIN,DEC,RET,base_stream,offsets) \
NOINLINE uint32_t name(uint32_t index) { \
    R r; \
    BEGIN(&r,base_stream,offsets[index]); \
    uint32_t *o=tile_words+2; \
    const uint8_t *const a=arena; \
    const int8_t *const km=klass_rot+128; \
    const int16_t *const mm=map_rot+128; \
    uint32_t prev=0; \
    for(unsigned row=0;row<16;row++,o+=2) { \
        const uint32_t rs=DEC(&r,a+rowmap[prev],6); \
        prev=rs; \
        int left=0; \
        uint32_t kinds=rs; \
        for(unsigned h=0;h<2;h++,kinds>>=2) { \
            uint32_t *q=o+h; \
            const int8_t *above=(const int8_t *)(q-2); \
            switch(kinds&3u) { \
            case 0: *q=0; left=0; break; \
            case 1: *q=q[-2]; left=above[3]&15; break; \
            case 2: { const uint32_t d=dict[DEC(&r,a+dict_off,7)]; *q=d; left=(int)(d&15u); break; } \
            default: { \
                uint8_t *b=(uint8_t *)q; \
                _Pragma("GCC unroll 4") \
                for(unsigned j=0;j<4;j++) { \
                    const uint32_t v=DEC(&r,a+mm[km[above[j]]|left],6); \
                    b[j]=(uint8_t)v; left=(int)(v&15u); \
                } \
            } } \
        } \
    } \
    return RET(&r,base_stream); \
}
#define V2_RET(r,s) ((uint32_t)sh2z_vlc2_bitpos(r,s))
TILE(tile_v2,sh2z_vlc2_reader_t,sh2z_vlc2_begin,sh2z_vlc2_decode,V2_RET,stream,tile_off)
TILE(tile_v2c,sh2z_vlc2_reader_t,sh2z_vlc2_begin,sh2z_vlc2_decode_portable,V2_RET,stream,tile_off)
TILE(tile_v2_packed,sh2z_vlc2_reader_t,sh2z_vlc2_begin,sh2z_vlc2_decode,V2_RET,packed,packed_off)

/* The same caller with symbols taken from symlist (no decoding): what the
 * caller's own code costs. The asm statement makes the table pointer live. */
typedef struct {const uint8_t *p;} list_t;
static inline void list_begin(list_t *l,const uint8_t *s,uint32_t offset){(void)s;(void)offset;l->p=symlist;}
__attribute__((always_inline)) static inline uint32_t list_next(list_t *l,const uint8_t *t,unsigned w) {
    (void)w;
    __asm__ volatile("" : : "r" (t));
    return *l->p++;
}
#define LIST_RET(r,s) ((uint32_t)((r)->p-symlist))
TILE(tile_glue,list_t,list_begin,list_next,LIST_RET,stream,tile_off)

/* vlc-1: one call per symbol. Tiles start at even offsets. Returns the
 * reader's unit pointer offset (only checked for being in range). */
NOINLINE uint32_t tile_v1(uint32_t index) {
    sh2z_vlc_reader_t r;
    sh2z_vlc_reader_init(&r,stream+tile_off[index],(STREAM_MAX-tile_off[index])/2u);
    uint32_t *o=tile_words+2;
    const int8_t *const km=klass_rot+128;
    uint8_t sym[1];
    uint32_t prev=0;
    for(unsigned row=0;row<16;row++,o+=2) {
        sh2z_vlc8_decode(&v1_row[prev],&r,sym,1);
        const uint32_t rs=sym[0];
        prev=rs;
        uint32_t left=0,kinds=rs;
        for(unsigned h=0;h<2;h++,kinds>>=2) {
            uint32_t *q=o+h;
            const int8_t *above=(const int8_t *)(q-2);
            switch(kinds&3u) {
            case 0: *q=0; left=0; break;
            case 1: *q=q[-2]; left=(uint32_t)above[3]&15u; break;
            case 2: { sh2z_vlc8_decode(&v1_dict,&r,sym,1); const uint32_t d=dict[sym[0]]; *q=d; left=d&15u; break; }
            default: {
                uint8_t *b=(uint8_t *)q;
                for(unsigned j=0;j<4;j++) {
                    sh2z_vlc8_decode(&v1_pair[v1_ctx[(uint8_t)(km[above[j]]|(int)left)]],&r,b+j,1);
                    left=b[j]&15u;
                }
            } }
        }
    }
    return (uint32_t)(r.p-stream);
}

/* ---- marker-bit reader variant (comparison only, not library API) ----
 * The reader of vlc-1: bits with a marker bit above the last valid one
 * (bb = bits | 1 << count), so count >= 16 exactly when bb >= 0x10000 and no
 * count is kept, with the vlc-2 table format and constant block. The fast
 * path has the same 12 instructions (CMP/HS against 0x10000 replaces SUBV;
 * an escape multiplies by 1 and leaves bb = 0 < 0x10000). The refill must
 * find the marker: the highest set bit of bb < 0x10000, here from mk_log2
 * (a byte's highest set bit times 4, as an offset of 2^c in the constant
 * block) on bb's high or low byte. */
const uint8_t mk_log2[256]={
#define R4(x) x,x,x,x
#define R16(x) R4(x),R4(x),R4(x),R4(x)
    0xC0,0xC0,0xC4,0xC4,R4(0xC8),R4(0xCC),R4(0xCC),R16(0xD0),R16(0xD4),R16(0xD4),
    R16(0xD8),R16(0xD8),R16(0xD8),R16(0xD8),
    R16(0xDC),R16(0xDC),R16(0xDC),R16(0xDC),R16(0xDC),R16(0xDC),R16(0xDC),R16(0xDC)
#undef R16
#undef R4
};
typedef struct {uint32_t bb;const uint8_t *p;const uint32_t *k;} mk_reader_t;
static inline void mk_begin(mk_reader_t *r,const uint8_t *s,uint32_t offset) {
    const uint8_t *p=s+offset;
    if((uintptr_t)p&1u){r->bb=(uint32_t)p[0]|(uint32_t)p[1]<<8|(uint32_t)p[2]<<16|UINT32_C(1)<<24;p+=3;}
    else{r->bb=(uint32_t)p[0]|(uint32_t)p[1]<<8|UINT32_C(1)<<16;p+=2;}
    r->p=p;r->k=sh2z_vlc2_consts;
}
__attribute__((always_inline)) static inline uint32_t mk_decode(mk_reader_t *r,const uint8_t *t,unsigned w) {
    uint32_t sym,f,x;
    const uint32_t k16=0x10000u;
    __asm__(
        "mov     %[b],r0\n\t"
        "and     #%O[m],r0\n\t"
        "mov.b   @(r0,%[t]),%[s]\n\t"
        "or      #%O[h],r0\n\t"
        "mov.b   @(r0,%[t]),r0\n"
        "1:\n\t"
        "mov.l   @(r0,%[k]),%[f]\n\t"
        "dmulu.l %[f],%[b]\n\t"
        "sts     mach,%[b]\n\t"
        "cmp/hs  %[k16],%[b]\n\t"
        "bt/s    3f\n\t"
        "extu.b  %[s],%[s]\n\t"
        "tst     #64,r0\n\t"
        "bf      2f\n\t"
        /* refill: c = highest set bit of bb; offset 0xC0 + 4c */
        "mov     %[b],r0\n\t"
        "shlr8   r0\n\t"
        "tst     r0,r0\n\t"
        "bt/s    4f\n\t"
        "mov     %[b],%[f]\n\t"
        "mov.b   @(r0,%[lg]),r0\n\t"
        "bra     5f\n\t"
        "add     #32,r0\n"
        "4:\n\t"
        "extu.b  %[f],r0\n\t"
        "mov.b   @(r0,%[lg]),r0\n"
        "5:\n\t"
        "extu.b  r0,r0\n\t"
        "mov.l   @(r0,%[k]),%[f]\n\t"     /* H = 2^c */
        "sub     %[f],%[b]\n\t"           /* drop the marker */
        "mov.w   @%[p]+,r0\n\t"
        "swap.b  r0,r0\n\t"
        "mulu.w  %[f],r0\n\t"
        "shll16  %[f]\n\t"
        "add     %[f],%[b]\n\t"           /* new marker at c + 16 */
        "sts     macl,r0\n\t"
        "bra     3f\n\t"
        "or      r0,%[b]\n"
        "2:\n\t"
        "sts     macl,%[b]\n\t"
        "add     #64,r0\n\t"
        "mov.l   @(r0,%[k]),%[f]\n\t"
        "mov.l   @(%O[w4],%[k]),r0\n\t"
        "dmulu.l r0,%[b]\n\t"
        "mov     %[s],r0\n\t"
        "shll2   r0\n\t"
        "sts     mach,%[s]\n\t"
        "and     %[f],%[s]\n\t"
        "add     %[s],r0\n\t"
        "add     %[s],r0\n\t"
        "mov.b   @(r0,%[t]),%[s]\n\t"
        "add     #1,r0\n\t"
        "bra     1b\n\t"
        "mov.b   @(r0,%[t]),r0\n"
        "3:"
        : [b] "+r" (r->bb),[p] "+r" (r->p),[s] "=&r" (sym),[f] "=&r" (f),"=&z" (x)
        : [t] "r" (t),[k] "r" (r->k),[k16] "r" (k16),[lg] "r" (mk_log2),
          [m] "n" ((1u<<w)-1u),[h] "n" (1u<<w),[w4] "n" (4u*w)
        : "mach","macl","t","memory");
    (void)x;
    return sym;
}
#define MK_RET(r,s) ((uint32_t)((r)->p-(s)))
TILE(tile_mk,mk_reader_t,mk_begin,mk_decode,MK_RET,stream,tile_off)

/* ---- symbol loops over one table (tbuf, w = 6; vlc-1: v1_dict) ---- */
NOINLINE uint32_t loop_v2(uint32_t n) {
    sh2z_vlc2_reader_t r;sh2z_vlc2_begin(&r,vstream,0);
    for(uint32_t i=0;i<n;i++)out[i]=(uint8_t)sh2z_vlc2_decode(&r,tbuf,6);
    return (uint32_t)sh2z_vlc2_bitpos(&r,vstream);
}
NOINLINE uint32_t loop_v2c(uint32_t n) {
    sh2z_vlc2_reader_t r;sh2z_vlc2_begin(&r,vstream,0);
    for(uint32_t i=0;i<n;i++)out[i]=(uint8_t)sh2z_vlc2_decode_portable(&r,tbuf,6);
    return (uint32_t)sh2z_vlc2_bitpos(&r,vstream);
}
NOINLINE uint32_t loop_mk(uint32_t n) {
    mk_reader_t r;mk_begin(&r,vstream,0);
    for(uint32_t i=0;i<n;i++)out[i]=(uint8_t)mk_decode(&r,tbuf,6);
    return (uint32_t)(r.p-vstream);
}
NOINLINE uint32_t loop_v1(uint32_t n) {
    sh2z_vlc_reader_t r;sh2z_vlc_reader_init(&r,vstream,VSTREAM/2u);
    return sh2z_vlc8_decode(&v1_dict,&r,out,n);
}
NOINLINE uint32_t loop_v1_each(uint32_t n) {
    sh2z_vlc_reader_t r;sh2z_vlc_reader_init(&r,vstream,VSTREAM/2u);
    for(uint32_t i=0;i<n;i++)sh2z_vlc8_decode(&v1_dict,&r,out+i,1);
    return sh2z_vlc_reader_ok(&r);
}
