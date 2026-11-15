/* 68000-exact MULS.W/MULU.W/DIVS.W/DIVU.W against a straightforward model.
 * The library's SH-2 DIV1 path cannot run on a host; its sequence is checked
 * here with an instruction-level DIV1 emulation, and the compiled SH-2 object
 * is executed against the same specification by benchmarks/integer-model.
 * MIT; Copyright (c) 2026 SH2ZAM Contributors.
 *
 * Default depth runs in a few seconds. SH2Z_M68K_EXHAUSTIVE=1 runs all 2^32
 * 16x16 multiplies and 200 million random divides. */
#include "sh2zam/m68k.h"
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static uint64_t checks;
#define CHECK(e) do { ++checks; if(!(e)) { fprintf(stderr,"FAIL %s:%d: %s\n",__FILE__,__LINE__,#e); exit(EXIT_FAILURE); } } while(0)
static uint32_t rng=UINT32_C(0x2f6e1b93);
static uint32_t rnd(void) { rng^=rng<<13; rng^=rng>>17; rng^=rng<<5; return rng; }

/* Independent specification: bit-serial restoring long division, no '/' or '%'. */
static void bit_divide(uint64_t n,uint32_t d,uint64_t *q,uint64_t *r) {
    uint64_t quo=0,rem=0;
    for(int i=63;i>=0;i--) {
        rem=(rem<<1)|((n>>i)&1u);
        if(rem>=d) { rem-=d; quo|=UINT64_C(1)<<i; }
    }
    *q=quo; *r=rem;
}
/* The documented contract written out a third way, signed via magnitudes. */
static uint32_t spec_div(int is_signed,uint32_t dn,uint16_t src,uint32_t *out) {
    uint64_t un=dn,ud=src;
    int neg_n=0,neg_d=0;
    if(is_signed) {
        if(dn&0x80000000u) { un=(uint64_t)(0x100000000u-dn); neg_n=1; }
        if(src&0x8000u) { ud=(uint64_t)(0x10000u-src); neg_d=1; }
    }
    if(ud==0) { *out=dn; return SH2Z_M68K_DIV_ZERO; }
    uint64_t q,r; bit_divide(un,(uint32_t)ud,&q,&r);
    int neg_q=neg_n!=neg_d;
    int overflow=is_signed ? (neg_q ? q>32768 : q>32767) : q>65535;
    if(overflow) { *out=dn; return SH2Z_M68K_V|SH2Z_M68K_N; }
    uint32_t sq=(uint32_t)(neg_q ? (0x10000u-(uint32_t)q)&0xFFFFu : (uint32_t)q);
    uint32_t sr=(uint32_t)(neg_n ? (0x10000u-(uint32_t)r)&0xFFFFu : (uint32_t)r);
    *out=(sr<<16)|sq;
    return ((sq&0x8000u)?SH2Z_M68K_N:0u)|(sq==0?SH2Z_M68K_Z:0u);
}

static void div_case(int is_signed,uint32_t dn,uint16_t src) {
    uint32_t fast=dn,ref=dn,spec;
    const uint32_t cf=is_signed?sh2z_m68k_divs_w(&fast,src):sh2z_m68k_divu_w(&fast,src);
    const uint32_t cr=is_signed?sh2z_m68k_ref_divs_w(&ref,src):sh2z_m68k_ref_divu_w(&ref,src);
    const uint32_t cs=spec_div(is_signed,dn,src,&spec);
    if(cf!=cs||cr!=cs||fast!=spec||ref!=spec)
        fprintf(stderr,"%s dn=%08" PRIx32 " src=%04x fast=%08" PRIx32 "/%x ref=%08" PRIx32 "/%x spec=%08" PRIx32 "/%x\n",
            is_signed?"divs":"divu",dn,src,fast,cf,ref,cr,spec,cs);
    CHECK(cf==cs); CHECK(cr==cs); CHECK(fast==spec); CHECK(ref==spec);
}
static void div_both(uint32_t dn,uint16_t src) { div_case(0,dn,src); div_case(1,dn,src); }

static void mul_case(uint32_t dn,uint16_t src) {
    uint32_t a=dn,b=dn,c=dn,d=dn;
    const uint32_t cu=sh2z_m68k_mulu_w(&a,src),cur=sh2z_m68k_ref_mulu_w(&b,src);
    const uint32_t cs=sh2z_m68k_muls_w(&c,src),csr=sh2z_m68k_ref_muls_w(&d,src);
    const uint32_t eu=(dn&0xFFFFu)*(uint32_t)src;
    const int32_t es=(int32_t)(int16_t)(uint16_t)dn*(int32_t)(int16_t)src;
    CHECK(a==eu); CHECK(b==eu); CHECK(c==(uint32_t)es); CHECK(d==(uint32_t)es);
    CHECK(cu==cur); CHECK(cs==csr);
    CHECK(cu==(((eu>>31)?SH2Z_M68K_N:0u)|(eu==0?SH2Z_M68K_Z:0u)));
    CHECK(cs==(((es<0)?SH2Z_M68K_N:0u)|(es==0?SH2Z_M68K_Z:0u)));
    CHECK(!(cu&(SH2Z_M68K_V|SH2Z_M68K_C|SH2Z_M68K_DIV_ZERO)));
    CHECK(!(cs&(SH2Z_M68K_V|SH2Z_M68K_C|SH2Z_M68K_DIV_ZERO)));
}
static void multiplies(int exhaustive) {
    static const uint16_t edge[]={0,1,2,3,0x7FFE,0x7FFF,0x8000,0x8001,0xFFFE,0xFFFF,0x00FF,0x0100,0x5555,0xAAAA};
    for(unsigned i=0;i<sizeof edge/sizeof edge[0];i++)
        for(unsigned j=0;j<sizeof edge/sizeof edge[0];j++) {
            mul_case(edge[i],edge[j]);
            mul_case(0xFFFF0000u|edge[i],edge[j]); /* high word of dn must be ignored */
            mul_case(0x12340000u|edge[i],edge[j]);
        }
    const uint32_t step=exhaustive?1u:251u;
    for(uint32_t a=0;a<65536;a++)
        for(uint32_t b=0;b<65536;b+=step) mul_case((rnd()&0xFFFF0000u)|a,(uint16_t)b);
}

/* Instruction-level DIV0U/DIV1/ROTCL emulation per the SH-1/SH-2 Software Manual,
 * running the same 16-step unsigned 32/16 sequence the SH-2 path uses. */
static uint32_t emu_udiv32_16(uint32_t n,uint32_t divisor) {
    uint32_t d=divisor<<16; unsigned Q=0,M=0,T=0;
    for(int i=0;i<16;i++) {
        const unsigned old_q=Q; Q=n>>31; n=(n<<1)|T;
        const uint32_t tmp0=n; unsigned tmp1;
        if(!old_q) {
            if(!M) { n-=d; tmp1=n>tmp0; Q=Q?!tmp1:tmp1; }
            else   { n+=d; tmp1=n<tmp0; Q=Q?tmp1:!tmp1; }
        } else {
            if(!M) { n+=d; tmp1=n<tmp0; Q=Q?!tmp1:tmp1; }
            else   { n-=d; tmp1=n>tmp0; Q=Q?tmp1:!tmp1; }
        }
        T=(Q==M);
    }
    n=(n<<1)|T; /* ROTCL */
    return n&0xFFFFu;
}
static void check_sequence(uint32_t n,uint32_t d) {
    if(!d||(n>>16)>=d) return;
    CHECK(emu_udiv32_16(n,d)==n/d);
    CHECK(sh2z_m68k_udiv32_16(n,d)==n/d);
}
static void sequence(void) {
    for(uint32_t d=1;d<65536;d++) {
        const uint64_t base=(uint64_t)d*65536u;
        static const uint32_t ks[]={0,1,2,3,0x7FFE,0x7FFF,0x8000,0x8001,0xFFFE,0xFFFF};
        for(unsigned k=0;k<sizeof ks/sizeof ks[0];k++)
            for(int e=-2;e<=2;e++) {
                const int64_t n=(int64_t)d*ks[k]+e;
                if(n>=0&&n<(int64_t)base) check_sequence((uint32_t)n,d);
            }
        for(int e=1;e<=3;e++) check_sequence((uint32_t)(base-(uint64_t)e),d);
        for(unsigned i=0;i<8;i++) check_sequence((uint32_t)(((uint64_t)rnd()*(uint64_t)d)>>16),d);
    }
}

static void divides(uint64_t random_cases) {
    static const uint32_t dn_edge[]={0,1,2,0x7FFF,0x8000,0xFFFF,0x10000,0x7FFFFFFF,0x80000000u,0x80000001u,
                                     0xFFFF0000u,0xFFFFFFFFu,0x7FFF0000u,0x7FFF7FFFu,0x80008000u,0x00010000u,0xFFFF8000u};
    static const uint16_t src_edge[]={0,1,2,3,0x7FFE,0x7FFF,0x8000,0x8001,0xFFFE,0xFFFF,0x0100,0xFF00};
    for(unsigned i=0;i<sizeof dn_edge/sizeof dn_edge[0];i++)
        for(unsigned j=0;j<sizeof src_edge/sizeof src_edge[0];j++) div_both(dn_edge[i],src_edge[j]);
    /* Every divisor against every quotient/remainder boundary, both signs of dividend. */
    for(uint32_t d=0;d<65536;d++) {
        static const uint32_t qs[]={0,1,2,0x7FFE,0x7FFF,0x8000,0x8001,0xFFFE,0xFFFF,0x10000};
        const uint32_t ud=d?d:1u, sd=(d&0x8000u)?0x10000u-d:d, s=sd?sd:1u;
        for(unsigned k=0;k<sizeof qs/sizeof qs[0];k++) {
            for(int e=-1;e<=1;e++) {
                int64_t n=(int64_t)ud*qs[k]+((e>0)?(int64_t)ud-1:e<0?-1:0);
                if(n>=0&&n<=UINT32_MAX) { div_both((uint32_t)n,(uint16_t)d); div_both(0u-(uint32_t)n,(uint16_t)d); }
                n=(int64_t)s*qs[k]+((e>0)?(int64_t)s-1:e<0?-1:0);
                if(n>=0&&n<=UINT32_MAX) { div_both((uint32_t)n,(uint16_t)d); div_both(0u-(uint32_t)n,(uint16_t)d); }
            }
        }
    }
    /* Random: half fully random, half built to be in range (q*d+r with r<d). */
    for(uint64_t i=0;i<random_cases;i++) {
        uint16_t d=(uint16_t)rnd();
        if(i&1u) { div_both(rnd(),d); continue; }
        if(i&2u) d=(uint16_t)(d>>(rnd()&15u));
        const uint32_t m=d&0x8000u?0x10000u-d:d;
        uint32_t um=(i&4u)?(uint32_t)(uint16_t)d:(m?m:1u);
        if(!um) um=1;
        const uint32_t q=rnd()&0xFFFFu,r=rnd()%um;
        uint32_t n=q*um+r; if(rnd()&1u) n=0u-n;
        div_both(n,d);
    }
}
int main(void) {
    const char *x=getenv("SH2Z_M68K_EXHAUSTIVE");
    const int exhaustive=x&&*x=='1';
    CHECK(SH2Z_M68K_N==8&&SH2Z_M68K_Z==4&&SH2Z_M68K_V==2&&SH2Z_M68K_C==1);
    { uint32_t v=0x00040003u; CHECK(sh2z_m68k_divu_w(&v,0)==SH2Z_M68K_DIV_ZERO&&v==0x00040003u); }
    { uint32_t v=0x80000000u; CHECK(sh2z_m68k_divs_w(&v,0xFFFF)==(SH2Z_M68K_V|SH2Z_M68K_N)&&v==0x80000000u); }
    { uint32_t v=0xFFFF7FFFu; CHECK(sh2z_m68k_divs_w(&v,1)==(SH2Z_M68K_V|SH2Z_M68K_N));
      v=0xFFFF8000u; CHECK(sh2z_m68k_divs_w(&v,1)==(SH2Z_M68K_N)&&v==0x00008000u); }
    { uint32_t v=0x7FFF8000u; CHECK(sh2z_m68k_divs_w(&v,0xFFFF)==(SH2Z_M68K_V|SH2Z_M68K_N)); }
    { uint32_t v=7; CHECK(sh2z_m68k_divs_w(&v,0xFFFE)==SH2Z_M68K_N&&v==0x0001FFFDu); } /* 7 / -2 = -3 r 1 */
    { uint32_t v=0xFFFFFFF9u; CHECK(sh2z_m68k_divs_w(&v,2)==SH2Z_M68K_N&&v==0xFFFFFFFDu); } /* -7 / 2 = -3 r -1 */
    { uint32_t v=0xAAAA0005u; CHECK(sh2z_m68k_muls_w(&v,0xFFFF)==SH2Z_M68K_N&&v==0xFFFFFFFBu); }
    { uint32_t v=0x12340000u; CHECK(sh2z_m68k_mulu_w(&v,0xFFFF)==SH2Z_M68K_Z&&v==0); }
    { uint32_t v=0x0000FFFFu; CHECK(sh2z_m68k_mulu_w(&v,0xFFFF)==SH2Z_M68K_N&&v==0xFFFE0001u); }
    multiplies(exhaustive);
    sequence();
    divides(exhaustive?UINT64_C(200000000):UINT64_C(3000000));
    printf("m68k: %" PRIu64 " checks passed%s\n",checks,exhaustive?" (exhaustive)":"");
    return 0;
}
