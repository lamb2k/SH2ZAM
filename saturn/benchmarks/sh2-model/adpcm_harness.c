/* SPDX-License-Identifier: GPL-2.0-or-later
 * Executed-instruction checks and cycle model for the IMA ADPCM decoder (adpcm.h).
 * Runs the compiled big-endian SH-2 code of benchmarks/adpcm-model/bridge.c in
 * Yabause's unmodified interpreter, reusing harness.c's loader and call wrapper.
 * Flat RAM, no caches or wait states: MODELLED cycles, not Saturn or 32X timing.
 * Copyright (c) 2026 SH2ZAM Contributors. This optional model harness is licensed
 * separately from SH2ZAM's MIT core; see LICENSE.
 *
 * usage: adpcm_harness model.elf verify-count > results.json */
#define main sh2_model_main
#include "harness.c"
#undef main
#include "sh2zam/adpcm.h"

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
static u32 symc(const char *name) {char tmp[140];snprintf(tmp,sizeof tmp,"_%s",name);return sym(tmp);}
static struct cost call(const char *name,u32 a,u32 b) {char tmp[140];snprintf(tmp,sizeof tmp,"_%s",name);return invoke(tmp,a,b,0,0);}

static uint32_t rng_state=UINT32_C(0x51ed2701);
static uint32_t rnd(void) {rng_state^=rng_state<<13;rng_state^=rng_state>>17;rng_state^=rng_state<<5;return rng_state;}

#define SRC_BYTES 2048u
static uint8_t host_src[SRC_BYTES];
static u32 a_state,a_src,a_out;
static void put_stream(unsigned kind) {
 for(u32 i=0;i<SRC_BYTES;i++){
  uint32_t r=rnd();
  if(kind==1) r=(r&0x88u)?0x77u:0xFFu;     /* rails */
  else if(kind==2) r&=0x33u;               /* quiet: low step indexes */
  host_src[i]=(uint8_t)r; wb(&cpu,a_src+i,(u8)r);
 }
}
static void put_state(sh2z_ima_state_t s) {wl(&cpu,a_state,(u32)s.pred);wl(&cpu,a_state+4,(u32)s.index);}
static const char *entries[]={"fast_decode","plain_decode","step_decode"};

static uint64_t verify(uint64_t count) {
 uint64_t checks=0;static int16_t want[2048];
 for(uint64_t it=0;checks<count;it++){
  put_stream((unsigned)(it%3u));
  const u32 pos=rnd()%64u,n=1u+rnd()%600u;
  const sh2z_ima_state_t s0={(int32_t)(rnd()%65536u)-32768,(int32_t)(rnd()%89u)};
  sh2z_ima_state_t ref=s0;sh2z_ima_ref_decode(&ref,host_src,pos,want,n);
  for(unsigned e=0;e<3;e++){
   put_state(s0);call(entries[e],pos,n);
   for(u32 i=0;i<n;i++) if((int16_t)rw(&cpu,a_out+2*i)!=want[i]){fprintf(stderr,"%s: sample %u differs\n",entries[e],i);exit(1);}
   if(signed_word(a_state)!=ref.pred||signed_word(a_state+4)!=ref.index){fprintf(stderr,"%s: state differs\n",entries[e]);exit(1);}
   checks+=n+2u;
  }
 }
 return checks;
}

static void cost(unsigned count,int last) {
 enum {CORPUS=256};
 double sum[3]={0,0,0};uint64_t ins[3]={0,0,0};
 for(unsigned c=0;c<CORPUS;c++){
  put_stream(c%3u);
  const u32 pos=rnd()%2u;
  const sh2z_ima_state_t s0={(int32_t)(rnd()%65536u)-32768,(int32_t)(rnd()%89u)};
  for(unsigned e=0;e<3;e++){put_state(s0);struct cost k=call(entries[e],pos,count);sum[e]+=k.cycles;ins[e]+=k.instructions;}
 }
 printf("\"n%u\":{",count);
 for(unsigned e=0;e<3;e++)
  printf("\"%s\":{\"meanCycles\":%.1f,\"cyclesPerNibble\":%.2f,\"meanInstructions\":%.1f}%s",entries[e],sum[e]/CORPUS,sum[e]/CORPUS/count,(double)ins[e]/CORPUS,e<2?",":"");
 printf("}%s",last?"":",");
}

int main(int argc,char**argv) {
 if(argc<3){fprintf(stderr,"usage: adpcm_harness model.elf verify-count\n");return 2;}
 const uint64_t count=strtoull(argv[2],0,10);
 load_elf_symbols(argv[1]);load_elf(argv[1]);
 cpu.model=SHMT_SH2;cpu.MappedMemoryReadByte=rb;cpu.MappedMemoryReadWord=rw;cpu.MappedMemoryReadLong=rl;cpu.MappedMemoryWriteByte=wb;cpu.MappedMemoryWriteWord=ww;cpu.MappedMemoryWriteLong=wl;
 for(unsigned i=0;i<65536;i++)cpu.opcodes[i]=decode(SHMT_SH2,(u16)i);for(unsigned i=0;i<256;i++)cpu.fetchlist[i]=fetch;
 a_state=symc("bridge_state");a_src=symc("bridge_src");a_out=symc("bridge_out");
 const uint64_t v=verify(count);
 printf("{\"schema\":1,\"kind\":\"Yabause isolated SH2 interpreter flat-RAM model, not Saturn or 32X timing\",\"verified\":%" PRIu64 ",\"cost\":{",v);
 cost(1,0);cost(32,0);cost(512,1);
 printf("}}\n");
 return 0;
}
