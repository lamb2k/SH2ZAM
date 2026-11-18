/* SPDX-License-Identifier: GPL-2.0-or-later
 * Executed-instruction checks and cycle model for contract tag trig12-1:
 * 12-bit-angle sin/cos/tan (trig12_experimental.h) and the fixed-point
 * quotients through the divider (divu_experimental.h). Runs the actual
 * big-endian SH-2 object code in Yabause's unmodified interpreter, like
 * harness.c, whose loader, call wrapper and divider stand-in it reuses.
 * Flat RAM, no caches or wait states: MODELLED cycles, not Saturn timing.
 * Copyright (c) 2026 SH2ZAM Contributors. This optional model harness is
 * licensed separately from SH2ZAM's MIT core; see LICENSE.
 *
 * usage: trig12_harness model.elf verify-scale > results.json
 * The ELF must contain bridge.o from benchmarks/trig12-model and the library.
 * Every output of every verification entry is compared with the independent
 * oracle in tests/trig12_oracle.h before any timing is printed; the divider
 * stand-in traps any division the library promised never to issue. */
#define main sh2_model_main
#include "harness.c"
#undef main
#include "../../tests/trig12_oracle.h"

static void load_elf_symbols(const char *path) {
 FILE*f=fopen(path,"rb");if(!f){perror(path);exit(2);}
 fseek(f,0,SEEK_END);long length=ftell(f);rewind(f);
 unsigned char*b=malloc((size_t)length);if(!b||fread(b,1,(size_t)length,f)!=(size_t)length)exit(2);fclose(f);
 u32 shoff=be32(b+32);u16 shentsize=be16(b+46),shnum=be16(b+48);
 for(unsigned i=0;i<shnum;i++){
  const unsigned char*sh=b+shoff+(size_t)i*shentsize;
  if(be32(sh+4)!=2)continue; /* SHT_SYMTAB */
  u32 off=be32(sh+16),size=be32(sh+20),link=be32(sh+24);
  const unsigned char*str=b+be32(b+shoff+(size_t)link*shentsize+16);
  for(u32 p=off;p+16<=off+size;p+=16){
   const char*name=(const char*)str+be32(b+p);
   if(!*name||nsymbols>=2048)continue;
   if(strlen(name)>=sizeof symbols[0].name)continue;
   symbols[nsymbols].address=be32(b+p+4);strcpy(symbols[nsymbols++].name,name);
  }
 }
 free(b);
}
static struct cost call(const char *name,u32 a){char tmp[140];snprintf(tmp,sizeof tmp,"_%s",name);return invoke(tmp,a,0,0,0);}
static u32 symc(const char *name){char tmp[140];snprintf(tmp,sizeof tmp,"_%s",name);return sym(tmp);}
static void bad(const char *what,uint64_t it){fprintf(stderr,"trig12 model mismatch: %s (case %"PRIu64")\n",what,it);exit(3);}

static uint32_t rng=UINT32_C(0x7219d1c5);
static uint32_t rnd(void){rng^=rng<<13;rng^=rng>>17;rng^=rng<<5;return rng;}

enum {N=4096};
static void put_words(const char *s,const uint32_t *v,unsigned n){const u32 a=symc(s);for(unsigned i=0;i<n;i++)wl(&cpu,a+4*i,v[i]);}
static int32_t get_word(u32 a){return (int32_t)rl(&cpu,a);}
static u32 float_bits(float f){u32 b;memcpy(&b,&f,4);return b;}

/* ---- verification ---- */
static uint64_t v_angles,v_quotients,v_hw,v_sat;
static void verify_trig(uint64_t scale){
 static uint32_t a[N];
 const uint64_t rounds=1+scale/N;
 for(uint64_t r=0;r<rounds;r++){
  for(unsigned i=0;i<N;i++)a[i]=r==0?i:(r==1?i|(rnd()&~UINT32_C(4095)):rnd());
  put_words("angles",a,N);
  if(call("v_trig",N).result!=N)bad("v_trig count",r);
  const u32 s=symc("out_s"),c=symc("out_c"),sa=symc("out_a"),cb=symc("out_b"),t=symc("out_t");
  for(unsigned i=0;i<N;i++){
   const int32_t ws=t12_sin(a[i],T12_MUT_NONE),wc=t12_cos(a[i],T12_MUT_NONE),wt=t12_tan(a[i],T12_MUT_NONE);
   if(get_word(s+4*i)!=ws||get_word(sa+4*i)!=ws)bad("sin",a[i]);
   if(get_word(c+4*i)!=wc||get_word(cb+4*i)!=wc)bad("cos",a[i]);
   if(get_word(t+4*i)!=wt)bad("tan",a[i]);
  }
  v_angles+=N;
 }
}
/* Operands near each width's fit boundary, extremes, small divisors, or anything. */
static void operands(int32_t *n,int32_t *d){
 const unsigned mode=rnd()%6u,f=(rnd()&1u)?16u:30u;
 int32_t dd=(int32_t)(rnd()>>(rnd()%32u));if(rnd()&1u)dd=(int32_t)(0u-(uint32_t)dd);
 if(mode==5)dd=(int32_t)(rnd()%5u)-2;
 int32_t nn;
 if(mode==0||mode==5)nn=(int32_t)rnd();
 else if(mode==1)nn=(int32_t)(rnd()>>(rnd()%32u));
 else if(mode==2)nn=(rnd()&1u)?INT32_MIN:INT32_MAX-(int32_t)(rnd()%3u);
 else{
  const uint64_t ad=dd<0?(uint64_t)0-(uint64_t)(int64_t)dd:(uint64_t)dd;
  const uint64_t edge=ad<<(31u-f);uint64_t m=(uint64_t)((int64_t)edge+(int64_t)(rnd()%5u)-2);
  if(edge==0||m>UINT64_C(0x80000000))m=rnd()&0x7FFFFFFFu;
  nn=m==UINT64_C(0x80000000)?INT32_MIN:(int32_t)m;if(nn!=INT32_MIN&&(rnd()&1u))nn=-nn;
 }
 *n=nn;*d=dd;
}
static void verify_divq(uint64_t scale){
 static uint32_t nv[N],dv[N];
 static const unsigned widths[4]={16,30,0,31};
 const uint64_t rounds=1+scale/N;
 for(uint64_t r=0;r<rounds;r++){
  for(unsigned i=0;i<N;i++){int32_t a,b;operands(&a,&b);nv[i]=(uint32_t)a;dv[i]=(uint32_t)b;}
  if(r==0){ /* fixed edges first */
   static const int32_t e[][2]={{0,0},{5,0},{-5,0},{32767,1},{32768,1},{-32768,1},{3,2},{4,2},{-4,2},{INT32_MIN,INT32_MIN},
    {INT32_MIN,-1},{INT32_MIN,1},{1,INT32_MIN},{-1,1},{7,2},{-7,2},{1<<30,(1<<29)+1},{1<<30,1<<29},{0,-5},{INT32_MAX,INT32_MIN}};
   for(unsigned i=0;i<sizeof e/sizeof e[0];i++){nv[i]=(uint32_t)e[i][0];dv[i]=(uint32_t)e[i][1];}
  }
  put_words("num",nv,N);put_words("den",dv,N);
  divu_reset();
  if(call("v_divq",N).result!=N)bad("v_divq count",r);
  uint64_t expect_hw=0;
  const u32 q=symc("out_q"),o=symc("out_o");
  for(unsigned i=0;i<N;i++){
   const int32_t a=(int32_t)nv[i],b=(int32_t)dv[i];bool ov;
   for(unsigned k=0;k<4;k++){
    const int32_t want=t12_divq(a,b,widths[k],&ov,T12_MUT_NONE);
    if(get_word(q+4*(k*N+i))!=want)bad("start/finish quotient",r*N+i);
    if(rb(&cpu,o+k*N+i)!=(u8)ov)bad("overflow flag",r*N+i);
    expect_hw+=!ov;v_sat+=ov;
   }
   int32_t w;bool o2;
   w=t12_divq(a,b,16,&o2,T12_MUT_NONE);if(get_word(q+4*(4*N+i))!=w)bad("q16",i);expect_hw+=!o2;
   w=t12_divq(a,b,30,&o2,T12_MUT_NONE);if(get_word(q+4*(5*N+i))!=w)bad("q30",i);expect_hw+=!o2;
   w=t12_divq(1<<16,b,16,&o2,T12_MUT_NONE);if(get_word(q+4*(6*N+i))!=w)bad("recip_q16",i);expect_hw+=!o2;
   w=t12_divq(1<<30,b,30,&o2,T12_MUT_NONE);if(get_word(q+4*(7*N+i))!=w)bad("recip_q30",i);expect_hw+=!o2;
  }
  if(divu_divisions!=expect_hw)bad("hardware division count",r);
  v_hw+=divu_divisions;v_quotients+=8u*N;
 }
}

/* ---- cost ---- */
struct row {const char *name,*entry,*description;int inputs;};
static const struct row rows[]={
 {"loop","c_copy","the loop alone: load an angle, store it",0},
 {"sincos12","c_sincos12","sh2z_sincos12_u30: sin and cos, Q2.30",0},
 {"sin12","c_sin12","sh2z_sin12_u30",0},
 {"floatLookup","c_float_lookup","control: two binary32 table loads (float sin/cos table idiom)",0},
 {"floatToU30","c_float_to_u30","control: two binary32 loads converted to Q2.30 (libgcc soft-float)",0},
 {"sh2zSincos","c_sh2z_sincos","sh2z_sincos (16-bit angle, Q16.16, interpolated; out-of-line)",0},
 {"tan12","c_tan12","sh2z_tan12_q16 (out-of-line, CPU division steps)",0},
 {"floatTan","c_float_tan","control: binary32 sin / cos (libgcc __divsf3)",0},
 {"sh2zTan","c_sh2z_tan","sh2z_tan (16-bit angle, Q16.16)",0},
 {"qloop","c_qcopy","the loop alone: load two operands, store one word",1},
 {"divuQ16","c_q16","sh2z_divu_q16, blocking",1},
 {"divuQ30","c_q30","sh2z_divu_q30, blocking",2},
 {"recipQ30","c_recip30","sh2z_recip_q30, blocking",3},
 {"sh2zDiv","c_sh2z_div","sh2z_div (CPU division steps, out-of-line)",1},
 {"libgccQ16","c_libgcc_q16","control: clamp(((int64_t)num << 16) / den) (libgcc __divdi3)",1},
 {"floatDiv","c_float_div","control: binary32 num / den (libgcc __divsf3)",1},
 {"q16ThenSincos12","c_q16_then_sincos","blocking sh2z_divu_q16, then sh2z_sincos12_u30",1},
 {"q16OverlapSincos12","c_q16_overlap_sincos","sh2z_divu_q_start, sh2z_sincos12_u30, sh2z_divu_finish",1},
};
/* Inputs for the cost runs: random angles; quotients whose result fits
 * (num up to 2^14 times den for q16, |num| < 2|den| for q30, |den| > 2^29
 * for the reciprocal), so the divider path is what is timed. */
static void cost_inputs(int kind){
 static uint32_t a[N],nv[N],dv[N],fn[N],fd[N];
 for(unsigned i=0;i<N;i++){
  a[i]=rnd()&4095u;
  int32_t d=(int32_t)((rnd()>>(1u+rnd()%14u))|1u);if(rnd()&1u)d=-d;
  int32_t n;
  if(kind==2){n=(int32_t)(rnd()%(2u*(uint32_t)(d<0?-d:d)));if(rnd()&1u)n=-n;}
  else if(kind==3){d=(int32_t)((rnd()>>2)|(1u<<29))+1;if(rnd()&1u)d=-d;n=1<<30;}
  else{const uint32_t ad=(uint32_t)(d<0?-d:d);n=(int32_t)(rnd()%(ad<(1u<<16)?ad<<14:(1u<<30)));if(rnd()&1u)n=-n;}
  nv[i]=(uint32_t)n;dv[i]=(uint32_t)d;fn[i]=float_bits((float)n);fd[i]=float_bits((float)d);
 }
 put_words("angles",a,N);put_words("num",nv,N);put_words("den",dv,N);put_words("fnum",fn,N);put_words("fden",fd,N);
}
static void cost_rows(void){
 bool first=true;
 printf("\"rows\":{");
 for(size_t r=0;r<sizeof rows/sizeof rows[0];r++){
  rng=UINT32_C(0xc0570000)+(u32)r;
  cost_inputs(rows[r].inputs);
  divu_reset();
  const struct cost one=call(rows[r].entry,1),many=call(rows[r].entry,N);
  if(one.result!=1||many.result!=N)bad(rows[r].entry,r);
  const double per=(double)(many.cycles-one.cycles)/(N-1),iper=(double)(many.instructions-one.instructions)/(N-1);
  printf("%s\"%s\":{\"entry\":\"%s\",\"description\":\"%s\",\"cyclesPerElement\":%.2f,\"instructionsPerElement\":%.2f,\"call1Cycles\":%u,\"call%uCycles\":%u,\"dividerStallCycles\":%"PRIu64"}",
   first?"":",",rows[r].name,rows[r].entry,rows[r].description,per,iper,one.cycles,N,many.cycles,divu_stall_cycles);
  first=false;
 }
 printf("}");
}

int main(int argc,char**argv){
 if(argc<3){fprintf(stderr,"usage: trig12_harness model.elf verify-scale\n");return 2;}
 const uint64_t scale=strtoull(argv[2],0,10);
 load_elf_symbols(argv[1]);load_elf(argv[1]);
 cpu.model=SHMT_SH2;cpu.MappedMemoryReadByte=rb;cpu.MappedMemoryReadWord=rw;cpu.MappedMemoryReadLong=rl;cpu.MappedMemoryWriteByte=wb;cpu.MappedMemoryWriteWord=ww;cpu.MappedMemoryWriteLong=wl;
 for(unsigned i=0;i<65536;i++)cpu.opcodes[i]=decode(SHMT_SH2,(u16)i);for(unsigned i=0;i<256;i++)cpu.fetchlist[i]=fetch;
 {static uint32_t f[4096];for(int i=0;i<4096;i++)f[i]=float_bits((float)t12_series(i,T12_MUT_NONE));put_words("fsin",f,4096);}
 verify_trig(scale);verify_divq(scale);
 printf("{\"schema\":1,\"kind\":\"Yabause isolated SH2 interpreter flat-RAM model, not Saturn timing\",");
 printf("\"dividerLatencyCycles\":%u,",DIVU_LATENCY);
 printf("\"verified\":{\"angles\":%"PRIu64",\"quotients\":%"PRIu64",\"hardwareDivisions\":%"PRIu64",\"saturatedInSoftware\":%"PRIu64",\"entries\":\"v_trig: sh2z_sincos12_u30, sh2z_sin12_u30, sh2z_cos12_u30, sh2z_tan12_q16 at every angle and with random high bits; v_divq: sh2z_divu_q_start/finish at f = 16, 30, 0, 31 with overflow flags, sh2z_divu_q16, sh2z_divu_q30, sh2z_recip_q16, sh2z_recip_q30; against tests/trig12_oracle.h; hardware division count equal to the oracle's fitting count\"},",
  v_angles,v_quotients,v_hw,v_sat);
 cost_rows();
 printf("}\n");
 return 0;
}
