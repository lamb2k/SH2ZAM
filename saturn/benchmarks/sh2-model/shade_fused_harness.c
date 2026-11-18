/* SPDX-License-Identifier: GPL-2.0-or-later
 * Executed-instruction checks and cycle model for the shade-2 kernels
 * (shade_experimental.h, cdiv_experimental.h). Runs the actual big-endian
 * SH-2 object code in Yabause's unmodified interpreter, like harness.c, whose
 * loader and call wrapper it reuses. Flat RAM, no caches or wait states:
 * MODELLED cycles, not Saturn timing. Copyright (c) 2026 SH2ZAM Contributors.
 * This optional model harness is licensed separately from SH2ZAM's MIT core;
 * see LICENSE.
 *
 * usage: shade_fused_harness model.elf verify-scale > results.json
 * The ELF must contain bridge.o from benchmarks/shade-fused-model and the
 * library. Every output of every fast_, shade1_ and plain_ entry is checked
 * against the independent oracle in tests/shade_fused_oracle.h before any
 * timing is printed: fast_ against the shade-2 rule, shade1_ and plain_
 * against shade-1's exact rule. */
#define main sh2_model_main
#include "harness.c"
#undef main
#include "../../tests/shade_fused_oracle.h"

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
static struct cost call(const char *name,u32 a) {char tmp[140];snprintf(tmp,sizeof tmp,"_%s",name);return invoke(tmp,a,0,0,0);}
static u32 symc(const char *name) {char tmp[140];snprintf(tmp,sizeof tmp,"_%s",name);return sym(tmp);}
static void bad(const char *what,uint64_t it) {fprintf(stderr,"shade-2 model mismatch: %s (case %"PRIu64")\n",what,it);exit(3);}

static uint32_t rng=UINT32_C(0xfade5eed);
static uint32_t rnd(void) {rng^=rng<<13;rng^=rng>>17;rng^=rng<<5;return rng;}
static int32_t range(int32_t lo,int32_t hi) {return (int32_t)((int64_t)lo+(int64_t)(rnd()%(uint64_t)((int64_t)hi-lo+1)));}
static double unit(void) {return (double)rnd()/4294967296.0;}

struct stats {double mean,median;unsigned min,max;double instr;};
static int cmp_u(const void*a,const void*b){unsigned x=*(const unsigned*)a,y=*(const unsigned*)b;return x<y?-1:x>y;}
static struct stats summarize(unsigned *cycles,size_t n,uint64_t instr) {
 qsort(cycles,n,sizeof *cycles,cmp_u);
 double sum=0;for(size_t i=0;i<n;i++)sum+=cycles[i];
 return (struct stats){sum/(double)n,n&1?cycles[n/2]:(cycles[n/2-1]+cycles[n/2])/2.0,cycles[0],cycles[n-1],(double)instr/(double)n};
}
static void print_stats(struct stats s) {
 printf("{\"meanCycles\":%.2f,\"medianCycles\":%.1f,\"minCycles\":%u,\"maxCycles\":%u,\"meanInstructions\":%.2f}",s.mean,s.median,s.min,s.max,s.instr);
}
static void put8(const char *s,u32 off,uint8_t v){wb(&cpu,symc(s)+off,v);}
static void put16(const char *s,u32 off,uint16_t v){ww(&cpu,symc(s)+off,v);}
static void put32(const char *s,u32 off,uint32_t v){wl(&cpu,symc(s)+off,v);}
static uint8_t get8(const char *s,u32 off){return rb(&cpu,symc(s)+off);}
static uint16_t get16(const char *s,u32 off){return rw(&cpu,symc(s)+off);}
static uint32_t get32(const char *s,u32 off){return rl(&cpu,symc(s)+off);}

/* ---- host copies of the inputs ---- */
enum {MAXV=1024,MAXP=256,VSTRIDE=20,VZ=8};
static int32_t h_fogp[5];static uint32_t h_depth[256];static int32_t h_near_z;
static oramp_t h_off,h_mix;
static unsigned h_count[MAXP];static uint8_t h_solid[MAXP],h_mean[MAXP][3];static int32_t h_z[MAXV];
static unsigned h_pieces,h_vertices;

static void rand_frame(void) {
 const int32_t n=range(1,32),f=range(n+64,30000),fs=range(0,996);
 h_fogp[0]=128000/(1000-fs);h_fogp[1]=(500-fs)*256/(1000-fs);
 h_fogp[2]=(int32_t)(((int64_t)(f+n)<<30)/(f-n));h_fogp[3]=(int32_t)-((int64_t)2*f*n*4096/(f-n));
 int64_t fz=(int64_t)f*16384;h_fogp[4]=(int32_t)(fz>INT32_MAX-1?INT32_MAX-1:fz);h_near_z=n*16384;
 oracle_fog_table(h_depth,h_fogp[0],h_fogp[1],h_fogp[2],h_fogp[3],h_fogp[4]);
 memset(&h_off,0,sizeof h_off);memset(&h_mix,0,sizeof h_mix);h_off.form=1;h_mix.form=2;
 for(int k=0;k<3;k++){h_off.live[k]=(uint8_t)(rnd()%32u);h_off.base[k]=(uint8_t)(rnd()%32u);h_mix.near_rgb[k]=(uint8_t)rnd();h_mix.far_rgb[k]=(uint8_t)rnd();}
 h_off.level[0]=0;h_off.level[1]=128;h_off.level[2]=255;
 h_off.threshold[0]=h_mix.threshold[0]=64;h_off.threshold[1]=h_mix.threshold[1]=192;
 for(unsigned k=0;k<5;k++)put32("fog_params",4*k,(uint32_t)h_fogp[k]);
 for(unsigned k=0;k<3;k++){put8("ramp_live",k,h_off.live[k]);put8("ramp_base",k,h_off.base[k]);put8("ramp_level",k,h_off.level[k]);
  put8("near_rgb",k,h_mix.near_rgb[k]);put8("far_rgb",k,h_mix.far_rgb[k]);}
 put16("ramp_threshold",0,64);put16("ramp_threshold",2,192);
 if(call("fast_prepare",0).result!=1)bad("prepare",0);
}
/* Piece vertex i: consecutive in verts, and scattered in verts_scattered at a
 * fixed permutation reached through vref[i] (a caller's corner pointers). */
static void put_vertex(unsigned i,int32_t z) {
 const unsigned at=(i*389u+17u)&(MAXV-1u);
 put32("verts",VSTRIDE*i+VZ,(uint32_t)z);
 put32("verts_scattered",VSTRIDE*at+VZ,(uint32_t)z);
 put32("vref",4*i,symc("verts_scattered")+VSTRIDE*at);
}
/* Depth of a piece centre: uniform between the planes, or log-uniform (near-dense). */
static int32_t centre(bool near_dense) {
 if(!near_dense)return range(h_near_z,h_fogp[4]);
 return (int32_t)lround((double)h_near_z*pow((double)h_fogp[4]/h_near_z,unit()));
}
/* mode 0: textured 4-corner pieces; 1: solid 4-corner; 2: mixed (1 in 8 solid, 1 in 8 clipped to 3..10) */
static void rand_pieces(unsigned pieces,unsigned mode,bool near_dense) {
 h_pieces=pieces;h_vertices=0;
 for(unsigned p=0;p<pieces;p++){
  const unsigned n=mode==2&&rnd()%8u==0u?3u+rnd()%8u:4u;
  h_count[p]=n;h_solid[p]=(uint8_t)(mode==1||(mode==2&&rnd()%8u==0u));
  for(int k=0;k<3;k++)h_mean[p][k]=(uint8_t)(rnd()%32u);
  const int32_t c=centre(near_dense),spread=c/16>0?c/16:1;
  for(unsigned i=0;i<n;i++)h_z[h_vertices++]=(int32_t)clamp64((int64_t)c+range(-spread,spread),h_near_z,h_fogp[4]);
  put16("piece_count",2*p,(uint16_t)n);put8("piece_solid",p,h_solid[p]);
  for(unsigned k=0;k<3;k++)put8("piece_mean",4*p+k,h_mean[p][k]);
 }
 if(h_vertices>MAXV)bad("vertex capacity",h_vertices);
 for(unsigned i=0;i<h_vertices;i++)put_vertex(i,h_z[i]);
}
static void rand_batch(unsigned count,bool near_dense) {
 h_pieces=1;h_vertices=count;h_count[0]=count;h_solid[0]=0;
 for(int k=0;k<3;k++)h_mean[0][k]=(uint8_t)(rnd()%32u);
 for(unsigned k=0;k<3;k++)put8("piece_mean",k,h_mean[0][k]);
 for(unsigned i=0;i<count;i++){h_z[i]=centre(near_dense);put32("verts",VSTRIDE*i+VZ,(uint32_t)h_z[i]);}
}
/* Check shades and variants of the last call against the oracle. */
static void check_pieces(const char *entry,unsigned mut,uint64_t it) {
 unsigned at=0;
 for(unsigned p=0;p<h_pieces;p++){
  uint16_t want[MAXV];
  const int v=oracle_fused(want,h_z+at,h_count[p],h_depth,h_solid[p]?&h_mix:&h_off,h_mean[p],h_solid[p]?0:mut);
  if(get8("variants",p)!=(uint8_t)v)bad(entry,it);
  for(unsigned i=0;i<h_count[p];i++)if(get16("shades",2*(at+i))!=want[i])bad(entry,it);
  at+=h_count[p];
 }
}

/* ---- verification ---- */
static uint64_t v_words,v_pieces,v_batch,v_cdiv,v_exact,v_channels;
static void verify(uint64_t iterations) {
 for(uint64_t it=0;it<iterations;it++){
  if(it%16u==0u)rand_frame();
  const unsigned pieces=1u+rnd()%64u;rand_pieces(pieces,it%3u,it%2u==0u);
  if(call("fast_pieces",pieces).result!=1)bad("fast_pieces accept",it);
  check_pieces("fast_pieces",0,it);
  for(unsigned i=0;i<h_vertices;i++)put16("shades",2*i,0xA5A5);
  if(call("fast_refs_pieces",pieces).result!=1)bad("fast_refs_pieces accept",it);
  check_pieces("fast_refs_pieces",0,it);
  /* exact-match rate of the fused words against shade-1's exact rule */
  {unsigned at=0;for(unsigned p=0;p<h_pieces;p++){uint16_t exact[MAXV];
    oracle_fused(exact,h_z+at,h_count[p],h_depth,h_solid[p]?&h_mix:&h_off,h_mean[p],FMUT_EXACT_TRUNC);
    for(unsigned i=0;i<h_count[p];i++){const uint16_t g=get16("shades",2*(at+i));
     for(int k=0;k<3;k++){const unsigned a=(g>>(5*k))&31u,b=(exact[i]>>(5*k))&31u;if(a!=b&&a!=b+1u)bad("fused within +1",it);v_exact+=a==b;v_channels++;}}
    at+=h_count[p];}}
  for(int e=0;e<2;e++){const char*entry=e?"plain_pieces":"shade1_pieces";
   for(unsigned i=0;i<h_vertices;i++)put16("shades",2*i,0xA5A5);
   if(call(entry,pieces).result!=1)bad(entry,it);
   check_pieces(entry,FMUT_EXACT_TRUNC,it);}
  v_words+=h_vertices;v_pieces+=pieces;
  const unsigned n=1u+rnd()%MAXV;rand_batch(n,it%2u==1u);
  if(call("fast_batch",n).result!=1)bad("fast_batch accept",it);
  check_pieces("fast_batch",0,it);
  if(call("plain_batch",n).result!=1)bad("plain_batch accept",it);
  check_pieces("plain_batch",FMUT_EXACT_TRUNC,it);
  v_batch+=n;
  /* constant divides at random points of their domains, both ends included */
  static const char *const rn[]={"cdiv255_u","cdiv255_s","cdiv65025_u","shiftadd_div255_s"};
  for(int e=0;e<4;e++){
   for(unsigned i=0;i<256;i++){int64_t x;
    switch(e){case 0:x=i<2?(i?16909558:0):range(0,16909558);break;
     case 1:x=i<2?(i?16909558:-16909558):range(-16909558,16909558);break;
     case 2:x=i<2?(i?4291648:0):range(0,4291648);break;
     default:x=i<2?(i?65534:-65534):range(-65534,65534);break;}
    put32("cdiv_in",4*i,(uint32_t)x);}
   if(call(rn[e],256).result!=1)bad(rn[e],it);
   for(unsigned i=0;i<256;i++){const uint32_t x=get32("cdiv_in",4*i),y=get32("cdiv_out",4*i);
    const bool ok=e==0?y==x/255u:e==2?y==x/65025u:(int32_t)y==(int32_t)x/255;
    if(!ok)bad(rn[e],it);}}
  v_cdiv+=256;
 }
}

/* ---- adversarial fog tables, depths, ramps and counts (fast_ entries only) ---- */
static void rand_frame_adv(void) {
 h_fogp[0]=rnd()%5u==0u?(rnd()%2u?1:INT32_MAX):range(1,40000);
 h_fogp[1]=rnd()%5u==0u?(rnd()%2u?INT32_MIN:INT32_MAX):range(-40000,300);
 h_fogp[2]=rnd()%5u==0u?(rnd()%2u?INT32_MIN:INT32_MAX):(int32_t)rnd();
 h_fogp[3]=rnd()%5u==0u?(rnd()%2u?INT32_MIN:0):-(int32_t)(rnd()&0x7fffffffu);
 h_fogp[4]=rnd()%5u==0u?(rnd()%2u?1:INT32_MAX):range(1,INT32_MAX);h_near_z=1;
 oracle_fog_table(h_depth,h_fogp[0],h_fogp[1],h_fogp[2],h_fogp[3],h_fogp[4]);
 memset(&h_off,0,sizeof h_off);memset(&h_mix,0,sizeof h_mix);h_off.form=1;h_mix.form=2;
 for(int k=0;k<3;k++){h_off.live[k]=(uint8_t)(rnd()%4u==0u?(rnd()%2u?127u:0u):rnd()%128u);
  h_off.base[k]=(uint8_t)(rnd()%4u==0u?(rnd()%2u?127u:0u):rnd()%128u);
  h_off.level[k]=(uint8_t)(rnd()%3u==0u?(rnd()%2u?255u:0u):rnd());
  h_mix.near_rgb[k]=(uint8_t)rnd();h_mix.far_rgb[k]=(uint8_t)rnd();}
 unsigned a=rnd()%4u==0u?256u:rnd()%257u,b=rnd()%4u==0u?256u:rnd()%257u;if(a>b){unsigned t=a;a=b;b=t;}
 h_off.threshold[0]=h_mix.threshold[0]=a;h_off.threshold[1]=h_mix.threshold[1]=b;
 for(unsigned k=0;k<5;k++)put32("fog_params",4*k,(uint32_t)h_fogp[k]);
 for(unsigned k=0;k<3;k++){put8("ramp_live",k,h_off.live[k]);put8("ramp_base",k,h_off.base[k]);put8("ramp_level",k,h_off.level[k]);
  put8("near_rgb",k,h_mix.near_rgb[k]);put8("far_rgb",k,h_mix.far_rgb[k]);}
 put16("ramp_threshold",0,(uint16_t)a);put16("ramp_threshold",2,(uint16_t)b);
 if(call("fast_prepare",0).result!=1)bad("adversarial prepare",0);
}
static int32_t adv_z(void) {
 switch(rnd()%7u){
 case 0:{const int64_t z=(int64_t)h_depth[rnd()%256u]+range(-2,1);return (int32_t)clamp64(z,INT32_MIN,INT32_MAX);}
 case 1:return range(INT32_MIN,0);
 case 2:return (int32_t)rnd();
 case 3:return rnd()%2u?INT32_MAX:INT32_MIN;
 case 4:return range(h_fogp[4]-4<0?0:h_fogp[4]-4,h_fogp[4]<INT32_MAX-4?h_fogp[4]+4:INT32_MAX);
 default:return range(0,h_fogp[4]);
 }
}
static uint64_t a_words,a_fades,a_args;
static void verify_adv(uint64_t iterations) {
 for(uint64_t it=0;it<iterations;it++){
  if(it%8u==0u)rand_frame_adv();
  const unsigned pieces=1u+rnd()%48u;h_pieces=pieces;h_vertices=0;
  for(unsigned p=0;p<pieces;p++){
   const unsigned n=rnd()%2u?4u:1u+rnd()%16u;h_count[p]=n;h_solid[p]=(uint8_t)(rnd()%3u==0u);
   for(int k=0;k<3;k++)h_mean[p][k]=(uint8_t)(rnd()%4u==0u?(rnd()%2u?127u:0u):rnd()%128u);
   for(unsigned i=0;i<n;i++)h_z[h_vertices++]=adv_z();
   put16("piece_count",2*p,(uint16_t)n);put8("piece_solid",p,h_solid[p]);
   for(unsigned k=0;k<3;k++)put8("piece_mean",4*p+k,h_mean[p][k]);}
  for(unsigned i=0;i<h_vertices;i++)put_vertex(i,h_z[i]);
  if(call("fast_pieces",pieces).result!=1)bad("adversarial fast_pieces accept",it);
  check_pieces("adversarial fast_pieces",0,it);
  for(unsigned i=0;i<h_vertices;i++)put16("shades",2*i,0xA5A5);
  if(call("fast_refs_pieces",pieces).result!=1)bad("adversarial fast_refs_pieces accept",it);
  check_pieces("adversarial fast_refs_pieces",0,it);
  a_words+=h_vertices;
  const unsigned n=1u+rnd()%MAXV;
  for(unsigned i=0;i<n;i++){h_z[i]=adv_z();put32("verts",VSTRIDE*i+VZ,(uint32_t)h_z[i]);}
  if(call("fast_fades",n).result!=1)bad("fast_fades accept",it);
  for(unsigned i=0;i<n;i++)if(get16("shades",2*i)!=oracle_fade(h_depth,h_z[i]))bad("fade equals the source's binary search",it);
  a_fades+=n;
  h_pieces=1;h_vertices=n;h_count[0]=n;h_solid[0]=0;
  for(unsigned k=0;k<3;k++)put8("piece_mean",k,h_mean[0][k]);
  if(call("fast_batch",n).result!=1)bad("adversarial fast_batch accept",it);
  check_pieces("adversarial fast_batch",0,it);
  a_words+=n;
 }
 /* argument rules of the assembly entry: rejected calls return -1 and write nothing */
 for(unsigned which=0;which<22;which++){
  for(unsigned i=0;i<8;i++)put16("args_out",2*i,0xEEEE);
  const u32 r=call("fast_args",which).result;
  bool untouched=true;for(unsigned i=0;i<8;i++)untouched&=get16("args_out",2*i)==0xEEEE;
  const bool reject=which<13||(which>=16&&which<=19),zero=which==13||which==20;
  if(reject&&(r!=UINT32_C(0xFFFFFFFF)||!untouched))bad("argument rejected without writes",which);
  if(zero&&(r!=0||!untouched))bad("count 0 returns 0, writes nothing",which);
  if(!reject&&!zero&&(r>2u||untouched))bad("accepted argument set",which);
  a_args++;
 }
}

/* ---- cost ---- */
enum {CORPORA=48};
static unsigned cyc[5][CORPORA];static uint64_t ins[5];
static void row(const char *name,const char *unit_name,unsigned elements,const char *const *entries,unsigned ne,
                void (*setup)(unsigned),u32 arg,bool *first) {
 for(unsigned e=0;e<ne;e++)ins[e]=0;
 rng=UINT32_C(0xc0571234)+elements*7u+(unsigned)strlen(name);
 uint64_t verts=0;
 for(unsigned c=0;c<CORPORA;c++){setup(elements);verts+=h_vertices;
  for(unsigned e=0;e<ne;e++){struct cost r=call(entries[e],arg);if(r.result!=1)bad(entries[e],c);cyc[e][c]=r.cycles;ins[e]+=r.instructions;}}
 if(!*first)putchar(',');*first=false;
 printf("\"%s\":{\"%s\":%u,\"meanVertices\":%.2f,\"corpora\":%u",name,unit_name,elements,(double)verts/CORPORA,(unsigned)CORPORA);
 for(unsigned e=0;e<ne;e++){printf(",\"%s\":",entries[e]);print_stats(summarize(cyc[e],CORPORA,ins[e]));}
 printf("}");
}
static void s_tex_u(unsigned p){rand_frame();rand_pieces(p,0,false);}
static void s_tex_n(unsigned p){rand_frame();rand_pieces(p,0,true);}
static void s_sol_u(unsigned p){rand_frame();rand_pieces(p,1,false);}
static void s_mix_u(unsigned p){rand_frame();rand_pieces(p,2,false);}
static void s_batch_u(unsigned n){rand_frame();rand_batch(n,false);}
static void s_batch_n(unsigned n){rand_frame();rand_batch(n,true);}
static void s_prep(unsigned n){(void)n;rand_frame();h_vertices=0;}
static void s_cdiv(unsigned n){h_vertices=0;for(unsigned i=0;i<n;i++)put32("cdiv_in",4*i,rnd()%4000000u);}

int main(int argc,char**argv) {
 if(argc<3){fprintf(stderr,"usage: shade_fused_harness model.elf verify-scale\n");return 2;}
 const uint64_t scale=strtoull(argv[2],0,10);
 load_elf_symbols(argv[1]);load_elf(argv[1]);
 cpu.model=SHMT_SH2;cpu.MappedMemoryReadByte=rb;cpu.MappedMemoryReadWord=rw;cpu.MappedMemoryReadLong=rl;cpu.MappedMemoryWriteByte=wb;cpu.MappedMemoryWriteWord=ww;cpu.MappedMemoryWriteLong=wl;
 for(unsigned i=0;i<65536;i++)cpu.opcodes[i]=decode(SHMT_SH2,(u16)i);for(unsigned i=0;i<256;i++)cpu.fetchlist[i]=fetch;
 verify(scale);
 verify_adv(scale/2u+1u);
 printf("{\"schema\":1,\"kind\":\"Yabause isolated SH2 interpreter flat-RAM model, not Saturn timing\",");
 printf("\"verified\":{\"pieces\":%"PRIu64",\"pieceWords\":%"PRIu64",\"batchWords\":%"PRIu64",\"constantDivides\":%"PRIu64","
  "\"fusedChannelsEqualShade1\":%"PRIu64",\"fusedChannels\":%"PRIu64",\"adversarialWords\":%"PRIu64",\"adversarialFades\":%"PRIu64","
  "\"argumentSets\":%"PRIu64",\"entries\":\"every fast_, shade1_ and plain_ result against tests/shade_fused_oracle.h\"},",
  v_pieces,v_words,v_batch,v_cdiv*4u,v_exact,v_channels,a_words,a_fades,a_args);
 bool first=true;printf("\"rows\":{");
 static const char *const pc[]={"fast_pieces","fast_refs_pieces","shade1_pieces","plain_pieces"};
 row("textured_pieces_1","pieces",1,pc,4,s_tex_u,1,&first);
 row("textured_pieces_64","pieces",64,pc,4,s_tex_u,64,&first);
 row("textured_pieces_1_near_dense","pieces",1,pc,4,s_tex_n,1,&first);
 row("textured_pieces_64_near_dense","pieces",64,pc,4,s_tex_n,64,&first);
 row("solid_pieces_1","pieces",1,pc,4,s_sol_u,1,&first);
 row("solid_pieces_64","pieces",64,pc,4,s_sol_u,64,&first);
 row("mixed_pieces_64","pieces",64,pc,4,s_mix_u,64,&first);
 static const char *const bt[]={"fast_batch","plain_batch"};
 row("batch_4","vertices",4,bt,2,s_batch_u,4,&first);
 row("batch_64","vertices",64,bt,2,s_batch_u,64,&first);
 row("batch_256","vertices",256,bt,2,s_batch_u,256,&first);
 row("batch_1024","vertices",1024,bt,2,s_batch_u,1024,&first);
 row("batch_256_near_dense","vertices",256,bt,2,s_batch_n,256,&first);
 static const char *const pr[]={"fast_fogp_prepare","fast_ramp_offset","fast_ramp_mix"};
 row("prepare","calls",1,pr,3,s_prep,0,&first);
 static const char *const rc[]={"cdiv_copy","cdiv255_u","cdiv255_s","cdiv65025_u"};
 static const char *const rp[]={"shiftadd_div255_s","c_div255_s","c_div65025"};
 row("cdiv_1","values",1,rc,4,s_cdiv,1,&first);
 row("cdiv_256","values",256,rc,4,s_cdiv,256,&first);
 row("cdiv_plain_1","values",1,rp,3,s_cdiv,1,&first);
 row("cdiv_plain_256","values",256,rp,3,s_cdiv,256,&first);
 printf("}}\n");
 return 0;
}
