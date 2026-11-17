/* SPDX-License-Identifier: GPL-2.0-or-later
 * Executed-instruction checks and cycle model for the vlc-1 kernels
 * (vlc_experimental.h). Runs the actual big-endian SH-2 object code in
 * Yabause's unmodified interpreter, like harness.c, whose loader and call
 * wrapper it reuses. Flat RAM, no caches or wait states: MODELLED cycles, not
 * Saturn timing. Copyright (c) 2026 SH2ZAM Contributors. This optional model
 * harness is licensed separately from SH2ZAM's MIT core; see LICENSE.
 *
 * usage: vlc_harness model.elf verify-scale > results.json
 * The ELF must contain bridge.o from benchmarks/vlc-model and the library.
 * Every table, decoded symbol and reader state of every entry is checked
 * against the independent oracle in tests/vlc_oracle.h before any timing is
 * printed, and any read of stream bytes past the stream's end during a decode
 * stops the run. */
#define main sh2_model_main
#include "harness.c"
#undef main
#include "../../tests/vlc_oracle.h"

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
static struct cost call(const char *name,u32 a,u32 b){char tmp[140];snprintf(tmp,sizeof tmp,"_%s",name);return invoke(tmp,a,b,0,0);}
static u32 symc(const char *name){char tmp[140];snprintf(tmp,sizeof tmp,"_%s",name);return sym(tmp);}
static void bad(const char *what,uint64_t it){fprintf(stderr,"vlc model mismatch: %s (case %"PRIu64")\n",what,it);exit(3);}

static uint32_t rng=UINT32_C(0x71c0de55);
static uint32_t rnd(void){rng^=rng<<13;rng^=rng>>17;rng^=rng<<5;return rng;}
static unsigned below(unsigned n){return n?rnd()%n:0u;}

/* ---- reads past the stream stop the run ---- */
static u32 guard_lo,guard_hi;static int guard_on;
static int guarded(u32 a,u32 n){return guard_on&&a<guard_hi&&a+n>guard_lo;}
static u8 g_rb(SH2_struct*sh,u32 a){if(guarded(a,1))fail("Read past the end of the stream",a);return rb(sh,a);}
static u16 g_rw(SH2_struct*sh,u32 a){if(guarded(a,2))fail("Read past the end of the stream",a);return rw(sh,a);}
static u32 g_rl(SH2_struct*sh,u32 a){if(guarded(a,4))fail("Read past the end of the stream",a);return rl(sh,a);}

/* ---- target memory (big-endian SH-2 layouts) ---- */
enum {STREAM_MAX=49152,OUT_MAX=20480};
static void put_table(const char *s,const sh2z_vlc8_t *t){
 const u32 a=symc(s);wb(&cpu,a,t->w);wb(&cpu,a+1,t->lmax);ww(&cpu,a+2,t->pool_len);
 for(unsigned i=0;i<256;i++){wb(&cpu,a+4+i,t->s[i]);wb(&cpu,a+260+i,t->n[i]);}
 for(unsigned i=0;i<SH2Z_VLC8_POOL_MAX;i++)ww(&cpu,a+516+2*i,t->pool[i]);
}
static void get_table(const char *s,sh2z_vlc8_t *t){
 const u32 a=symc(s);t->w=rb(&cpu,a);t->lmax=rb(&cpu,a+1);t->pool_len=rw(&cpu,a+2);
 for(unsigned i=0;i<256;i++){t->s[i]=rb(&cpu,a+4+i);t->n[i]=rb(&cpu,a+260+i);}
 for(unsigned i=0;i<SH2Z_VLC8_POOL_MAX;i++)t->pool[i]=rw(&cpu,a+516+2*i);
}
static void put_freq(const uint32_t f[256]){const u32 a=symc("freq");for(unsigned i=0;i<256;i++)wl(&cpu,a+4*i,f[i]);}
static void put_lengths(const unsigned l[256]){const u32 a=symc("lengths");for(unsigned i=0;i<256;i++)wb(&cpu,a+i,(u8)l[i]);}
static void put_stream(const uint8_t *s,size_t bytes){const u32 a=symc("stream");for(size_t i=0;i<bytes;i++)wb(&cpu,a+(u32)i,s[i]);}

/* ---- alphabets: the host test's kinds ---- */
static void gen_freq(uint32_t f[256],unsigned kind,unsigned m){
 unsigned perm[256];for(unsigned i=0;i<256;i++)perm[i]=i;
 for(unsigned i=255;i>0;i--){unsigned j=below(i+1),x=perm[i];perm[i]=perm[j];perm[j]=x;}
 memset(f,0,1024);if(kind==5)m=1;if(kind==6)m=2;
 uint64_t a=1,b=1;
 for(unsigned r=0;r<m;r++){uint32_t v=1;
  switch(kind){case 0:v=1u+below(1000);break;case 1:v=1000000u/(1u+r)+1u;break;
  case 2:v=(uint32_t)(a>UINT32_MAX?UINT32_MAX:a);{uint64_t c=a+b;a=b;b=c;}break;case 3:v=7;break;case 4:v=1u<<(r%32u);break;
  case 5:v=1u+below(100000);break;case 6:v=1u+below(1u<<below(20));break;case 7:v=r?1u:UINT32_MAX;break;case 8:v=rnd()|1u;break;
  case 9:v=r<30u?(1u<<(r/2u+1u)):1u;break;default:v=1u<<(31u-(r*31u)/m);break;}
  f[perm[r]]=v;}
}
static unsigned rand_m(void){switch(rnd()%6u){case 0:return 1u+below(4);case 1:return 256;case 2:return 255;case 3:return 1u+below(40);default:return 1u+below(256);}}

/* ---- verification ---- */
static uint64_t v_builds,v_refusals,v_symbols,v_streams,v_truncated,v_random_tables,v_odd;
static void verify_build(uint64_t iterations){
 static sh2z_vlc8_t want,got;
 for(uint64_t it=0;it<iterations;it++){
  uint32_t f[256];gen_freq(f,below(11),rand_m());
  const unsigned lmax=1u+below(16),w=1u+below(lmax<8?lmax:8);
  unsigned len[256],code[256];
  const bool wok=vo_build(&want,f,w,lmax,len,code,0);
  put_freq(f);
  if(call("fast_build",w,lmax).result!=(u32)wok)bad("fast_build accept",it);
  if(call("plain_build",w,lmax).result!=(u32)wok)bad("plain_build accept",it);
  v_builds++;
  if(!wok){v_refusals++;continue;}
  get_table("table",&got);if(!vo_same_table(&got,&want))bad("fast_build table",it);
  get_table("plain_table",&got);
  if(vo_count(len)==1)for(unsigned e=1;e<(1u<<w);e+=2){if(got.s[e]||got.n[e])bad("plain_build single",it);got.s[e]=want.s[e];}
  if(!vo_same_table(&got,&want))bad("plain_build table",it);
  put_lengths(len);
  if(call("fast_lengths",w,lmax).result!=1)bad("fast_lengths accept",it);
  get_table("table",&got);if(!vo_same_table(&got,&want))bad("fast_lengths table",it);
 }
}
static const char *const decoders[3]={"fast_decode","portable_decode","plain_decode"};
static uint8_t host_stream[STREAM_MAX],ref[OUT_MAX];
/* Decode n symbols from `units` units with all three entries; compare symbols,
 * return value, (fast, portable) reader state and the bytes after out[n]. */
static void decode_all(const sh2z_vlc8_t *t,const unsigned *len,const unsigned *code,size_t units,size_t n,uint64_t it){
 const uint64_t used=len?vo_decode(len,code,host_stream,units,0,ref,n,0):vo_decode_table(t,host_stream,units,0,ref,n);
 const vo_state_t st=vo_state(host_stream,units,used,0);
 const u32 sa=symc("stream"),oa=symc("out"),ra=symc("reader");
 for(int k=0;k<3;k++){
  for(unsigned i=0;i<32;i++)wb(&cpu,oa+(u32)n+i,0xA5);
  guard_lo=sa+2*(u32)units;guard_hi=sa+STREAM_MAX;guard_on=1;
  struct cost c=call(decoders[k],(u32)units,(u32)n);
  guard_on=0;
  if(c.result!=(u32)st.ok)bad(k==0?"fast ok":k==1?"portable ok":"plain ok",it);
  for(size_t i=0;i<n;i++)if(rb(&cpu,oa+(u32)i)!=ref[i])bad(decoders[k],it);
  for(unsigned i=0;i<32;i++)if(rb(&cpu,oa+(u32)n+i)!=0xA5)bad("write past out[n]",it);
  if(k<2){
   if(rl(&cpu,ra)!=sa+(u32)st.p||rl(&cpu,ra+4)!=st.bb||rl(&cpu,ra+8)!=sa+2*(u32)units||rl(&cpu,ra+12)!=st.pad)bad(k?"portable reader state":"fast reader state",it);
  }
 }
 v_symbols+=n;
}
static void verify_decode(uint64_t iterations){
 static sh2z_vlc8_t t;
 for(uint64_t it=0;it<iterations;it++){
  uint32_t f[256];gen_freq(f,below(11),rand_m());
  const unsigned lmax=1u+below(16),w=1u+below(lmax<8?lmax:8);
  unsigned len[256],code[256];
  if(!vo_build(&t,f,w,lmax,len,code,0)){it--;continue;}
  put_table("table",&t);
  unsigned alphabet[256],m=0;for(unsigned s=0;s<256;s++)if(len[s])alphabet[m++]=s;
  const size_t n=it%8u==0u?2048u:below(400);
  memset(host_stream,0,sizeof host_stream);
  uint64_t at=0;for(size_t i=0;i<n;i++){const unsigned s=alphabet[below(m)];at=vo_put(host_stream,at,code[s],len[s]);}
  const size_t units=(size_t)((at+15)/16);
  put_stream(host_stream,2*units);
  decode_all(&t,len,code,units,n,it);                               /* the message */
  decode_all(&t,len,code,units,n+1u+below(40),it);                  /* past its end */
  const size_t cut=units?below((unsigned)units):0u;
  decode_all(&t,len,code,cut,n,it);v_truncated++;                   /* truncated */
  const size_t ru=below(64);for(size_t i=0;i<2*ru;i++)host_stream[i]=(uint8_t)rnd();
  put_stream(host_stream,2*ru);
  decode_all(&t,len,code,ru,below(600),it);                          /* random bytes */
  {const size_t n2=below(300);const uint64_t used=vo_decode(len,code,host_stream,ru,0,ref,n2,0);  /* odd address */
   const vo_state_t st=vo_state(host_stream,ru,used,0);const u32 sa=symc("stream")+1,oa=symc("out"),ra=symc("reader");
   for(size_t i=0;i<2*ru;i++)wb(&cpu,sa+(u32)i,host_stream[i]);
   guard_lo=sa+2*(u32)ru;guard_hi=symc("stream")+STREAM_MAX;guard_on=1;
   struct cost c=call("fast_decode_odd",(u32)ru,(u32)n2);guard_on=0;
   if(c.result!=(u32)st.ok)bad("odd ok",it);
   for(size_t i=0;i<n2;i++)if(rb(&cpu,oa+(u32)i)!=ref[i])bad("odd symbols",it);
   if(rl(&cpu,ra)!=sa+(u32)st.p||rl(&cpu,ra+4)!=st.bb||rl(&cpu,ra+12)!=st.pad)bad("odd reader state",it);
   v_odd++;}
  v_streams+=4;
 }
 /* random tables in the table format that pass the check */
 for(uint64_t it=0;it<iterations/4u+1u;it++){
  memset(&t,0,sizeof t);t.w=(uint8_t)(1u+below(8));t.pool_len=(uint16_t)below(SH2Z_VLC8_POOL_MAX+1u);
  for(unsigned i=0;i<SH2Z_VLC8_POOL_MAX;i++)t.pool[i]=(uint16_t)(below(256)<<8|4u*below(16));
  for(unsigned e=0;e<(1u<<t.w);e++){const unsigned xb=below(8);
   if(rnd()%3u||t.pool_len<(1u<<xb)){t.s[e]=(uint8_t)rnd();t.n[e]=(uint8_t)(4u*below(16));continue;}
   const unsigned start=below(t.pool_len-(1u<<xb)+1u);t.s[e]=(uint8_t)(start&255u);t.n[e]=(uint8_t)(0x80u|xb<<4|start>>8);}
  if(!vo_check(&t))bad("random table generator",it);
  put_table("table",&t);
  const size_t ru=below(48);for(size_t i=0;i<2*ru;i++)host_stream[i]=(uint8_t)rnd();
  put_stream(host_stream,2*ru);
  decode_all(&t,NULL,NULL,ru,below(500),it);
  v_random_tables++;
 }
}

/* ---- cost ---- */
enum {CORPORA=16,SYMBOLS=4096};
struct stats {double mean,median;unsigned min,max;double instr;};
static int cmp_u(const void*a,const void*b){unsigned x=*(const unsigned*)a,y=*(const unsigned*)b;return x<y?-1:x>y;}
static struct stats summarize(unsigned *cycles,size_t n,uint64_t instr){
 qsort(cycles,n,sizeof *cycles,cmp_u);double sum=0;for(size_t i=0;i<n;i++)sum+=cycles[i];
 return (struct stats){sum/(double)n,n&1?cycles[n/2]:(cycles[n/2-1]+cycles[n/2])/2.0,cycles[0],cycles[n-1],(double)instr/(double)n};
}
static void print_stats(struct stats s){printf("{\"meanCycles\":%.2f,\"medianCycles\":%.1f,\"minCycles\":%u,\"maxCycles\":%u,\"meanInstructions\":%.2f}",s.mean,s.median,s.min,s.max,s.instr);}
struct profile {const char *name,*description;unsigned kind,m,w,lmax;};
static const struct profile profiles[]={
 {"zipf256_w6_l12","256 symbols, frequency 1/rank (Zipf)",1,256,6,12},
 {"uniform256_w8_l8","256 symbols, near-uniform frequencies",0,256,8,8},
 {"longtail256_w6_l12","30 frequent symbols (doubling every two), 226 rare",9,256,6,12},
 {"few40_w5_l10","40 draws of random symbols with weights 2^0..2^13",12,40,5,10},
 {"geometric16_w8_l12","16 symbols, each half as frequent as the last",10,16,8,12},
};
static void profile_freq(uint32_t f[256],const struct profile *p){
 if(p->kind==12){memset(f,0,1024);for(unsigned i=0;i<p->m;i++)f[(rnd()>>3)&255u]+=1u<<below(14);return;}
 gen_freq(f,p->kind,p->m);
}
static void cost_profile(const struct profile *p,bool *first){
 static unsigned cyc[3][2][CORPORA],bcyc[3][CORPORA];static uint64_t ins[3][2],bins[3];
 memset(ins,0,sizeof ins);memset(bins,0,sizeof bins);
 rng=UINT32_C(0xc0571234)+p->kind*977u+p->w;
 uint64_t bits_total=0;static sh2z_vlc8_t t,got;
 for(unsigned c=0;c<CORPORA;c++){
  uint32_t f[256];unsigned len[256],code[256];
  do profile_freq(f,p);while(!vo_build(&t,f,p->w,p->lmax,len,code,0));
  /* build cost */
  put_freq(f);put_lengths(len);
  static const char *const builds[3]={"fast_build","plain_build","fast_lengths"};
  for(int k=0;k<3;k++){struct cost r=call(builds[k],p->w,p->lmax);if(r.result!=1)bad(builds[k],c);bcyc[k][c]=r.cycles;bins[k]+=r.instructions;}
  get_table("table",&got);if(!vo_same_table(&got,&t))bad("cost table",c);
  /* a message drawn by frequency */
  uint64_t total=0;for(unsigned s=0;s<256;s++)total+=f[s];
  memset(host_stream,0,sizeof host_stream);uint64_t at=0;
  for(unsigned i=0;i<SYMBOLS;i++){uint64_t x=((uint64_t)rnd()<<32|rnd())%total;unsigned s=0;while(x>=f[s]){x-=f[s];s++;}at=vo_put(host_stream,at,code[s],len[s]);}
  const size_t units=(size_t)((at+15)/16);bits_total+=at;
  put_stream(host_stream,2*units);
  const uint64_t used=vo_decode(len,code,host_stream,units,0,ref,SYMBOLS,0);if(used!=at)bad("cost message",c);
  for(int k=0;k<3;k++)for(int z=0;z<2;z++){
   const unsigned n=z?SYMBOLS:1u;
   struct cost r=call(decoders[k],(u32)units,n);if(r.result!=1)bad(decoders[k],c);
   for(unsigned i=0;i<n;i++)if(rb(&cpu,symc("out")+i)!=ref[i])bad("cost decode",c);
   cyc[k][z][c]=r.cycles;ins[k][z]+=r.instructions;}
 }
 const double bps=(double)bits_total/((double)CORPORA*SYMBOLS);
 if(!*first)putchar(',');*first=false;
 printf("\"%s\":{\"description\":\"%s\",\"w\":%u,\"lmax\":%u,\"corpora\":%u,\"symbols\":%u,\"meanBitsPerSymbol\":%.4f,\"decode\":{",p->name,p->description,p->w,p->lmax,CORPORA,SYMBOLS,bps);
 static const char *const names[3]={"fast","portable","plain"};
 for(int k=0;k<3;k++){
  struct stats one=summarize(cyc[k][0],CORPORA,ins[k][0]),many=summarize(cyc[k][1],CORPORA,ins[k][1]);
  const double per=(many.mean-one.mean)/(SYMBOLS-1),iper=(many.instr-one.instr)/(SYMBOLS-1);
  printf("%s\"%s\":{\"cyclesPerSymbol\":%.2f,\"cyclesPerBit\":%.2f,\"instructionsPerSymbol\":%.2f,\"call1\":",k?",":"",names[k],per,per/bps,iper);
  print_stats(one);printf(",\"call%u\":",SYMBOLS);print_stats(many);printf("}");
 }
 printf("},\"build\":{");
 static const char *const bnames[3]={"fast","plain","fromLengths"};
 for(int k=0;k<3;k++){printf("%s\"%s\":",k?",":"",bnames[k]);print_stats(summarize(bcyc[k],CORPORA,bins[k]));}
 printf("}}");
}

int main(int argc,char**argv){
 if(argc<3){fprintf(stderr,"usage: vlc_harness model.elf verify-scale\n");return 2;}
 const uint64_t scale=strtoull(argv[2],0,10);
 load_elf_symbols(argv[1]);load_elf(argv[1]);
 cpu.model=SHMT_SH2;cpu.MappedMemoryReadByte=g_rb;cpu.MappedMemoryReadWord=g_rw;cpu.MappedMemoryReadLong=g_rl;cpu.MappedMemoryWriteByte=wb;cpu.MappedMemoryWriteWord=ww;cpu.MappedMemoryWriteLong=wl;
 for(unsigned i=0;i<65536;i++)cpu.opcodes[i]=decode(SHMT_SH2,(u16)i);for(unsigned i=0;i<256;i++)cpu.fetchlist[i]=fetch;
 verify_build(scale);verify_decode(scale);
 printf("{\"schema\":1,\"kind\":\"Yabause isolated SH2 interpreter flat-RAM model, not Saturn timing\",");
 printf("\"verified\":{\"builds\":%"PRIu64",\"refusals\":%"PRIu64",\"streams\":%"PRIu64",\"truncatedStreams\":%"PRIu64",\"randomFormatTables\":%"PRIu64",\"oddAddressStreams\":%"PRIu64",\"symbolsPerEntry\":%"PRIu64",\"entries\":\"fast_build, plain_build, fast_lengths tables; fast_decode, portable_decode, plain_decode symbols and return values, fast and portable reader state, against tests/vlc_oracle.h; no read past any stream\"},",
  v_builds,v_refusals,v_streams,v_truncated,v_random_tables,v_odd,v_symbols);
 bool first=true;printf("\"rows\":{");
 for(size_t i=0;i<sizeof profiles/sizeof profiles[0];i++)cost_profile(&profiles[i],&first);
 printf("}}\n");
 return 0;
}
