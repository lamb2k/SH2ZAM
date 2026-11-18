/* SPDX-License-Identifier: GPL-2.0-or-later
 * Executed-instruction checks and cycle model for the memo-1 kernels
 * (memo_experimental.h). Runs the actual big-endian SH-2 object code in
 * Yabause's unmodified interpreter, like harness.c, whose loader, call wrapper
 * and placement model it reuses. Flat RAM, no caches or wait states unless
 * MODEL_PLACEMENT_LOG is set: MODELLED cycles, not Saturn timing.
 * Copyright (c) 2026 SH2ZAM Contributors. This optional model harness is
 * licensed separately from SH2ZAM's MIT core; see LICENSE.
 *
 * usage: memo_harness model.elf verify-scale > raw.json
 * The ELF must contain bridge.o from benchmarks/memo-model and the library.
 * Before any timing, every output of every entry is checked against the
 * independent oracle in tests/memo_oracle.h, and:
 *  - every write to the keyed-write destination is logged; it must be a
 *    16-bit store, the log must equal the oracle's write list in order, and
 *    the destination must never be read;
 *  - the marker copy must not read any source byte after the marker record;
 *  - guard bytes after every output must be untouched.
 * The cost fixtures are verified the same way on every timed call. */
#define main sh2_model_main
#include "harness.c"
#undef main
#include "../../tests/memo_oracle.h"

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
static u32 S(const char *name){char tmp[140];snprintf(tmp,sizeof tmp,"_%s",name);return sym(tmp);}
static struct cost call(const char *name,u32 a,u32 b,u32 c,u32 d){char tmp[140];snprintf(tmp,sizeof tmp,"_%s",name);return invoke(tmp,a,b,c,d);}
static void bad(const char *what,uint64_t it){fprintf(stderr,"memo model mismatch: %s (case %"PRIu64")\n",what,it);exit(3);}

static uint32_t rng=UINT32_C(0x3e3017a5);
static uint32_t rnd(void){rng^=rng<<13;rng^=rng>>17;rng^=rng<<5;return rng;}
static uint32_t below(uint32_t n){return n?rnd()%n:0u;}

/* ---- target memory ---- */
static void put_bytes(u32 a,const uint8_t *p,size_t n){for(size_t i=0;i<n;i++)wb(&cpu,a+(u32)i,p[i]);}
static void get_bytes(u32 a,uint8_t *p,size_t n){for(size_t i=0;i<n;i++)p[i]=rb(&cpu,a+(u32)i);}
static void fill_bytes(u32 a,uint8_t v,size_t n){for(size_t i=0;i<n;i++)wb(&cpu,a+(u32)i,v);}
static void put_words(u32 a,const uint16_t *p,size_t n){for(size_t i=0;i<n;i++)ww(&cpu,a+2u*(u32)i,p[i]);}
static void get_words(u32 a,uint16_t *p,size_t n){for(size_t i=0;i<n;i++)p[i]=rw(&cpu,a+2u*(u32)i);}

/* ---- destination write log and read guards ---- */
enum {MAXG=160,MAXR=640,MAXC=1024,MAXD=512};
static u32 dlo,dhi,dlog_n;static int dlog_on;
static struct {u32 off;u16 value;} dlog[4*MAXR+16];
static u32 glo,ghi;static int guard_on;
static int in_dst(u32 a,u32 n){return dlog_on&&a<dhi&&a+n>dlo;}
static int guarded(u32 a,u32 n){return guard_on&&a<ghi&&a+n>glo;}
static u8 h_rb(SH2_struct*sh,u32 a){if(in_dst(a,1))fail("Read of the destination",a);if(guarded(a,1))fail("Read past the end marker",a);return rb(sh,a);}
static u16 h_rw(SH2_struct*sh,u32 a){if(in_dst(a,2))fail("Read of the destination",a);if(guarded(a,2))fail("Read past the end marker",a);return rw(sh,a);}
static u32 h_rl(SH2_struct*sh,u32 a){if(in_dst(a,4))fail("Read of the destination",a);if(guarded(a,4))fail("Read past the end marker",a);return rl(sh,a);}
static void h_wb(SH2_struct*sh,u32 a,u8 v){if(in_dst(a,1))fail("Byte write to the destination",a);wb(sh,a,v);}
static void h_ww(SH2_struct*sh,u32 a,u16 v){
 if(in_dst(a,2)){if(dlog_n>=sizeof dlog/sizeof dlog[0])fail("Destination write log full",a);dlog[dlog_n].off=a-dlo;dlog[dlog_n++].value=v;}
 ww(sh,a,v);}
static void h_wl(SH2_struct*sh,u32 a,u32 v){if(in_dst(a,4))fail("Longword write to the destination",a);wl(sh,a,v);}

/* ---- placement declarations (MODEL_PLACEMENT_LOG) ----
 * A: shadow, mask and flags are output, src input.
 * B: the staged records and the descriptor are tables (high work RAM in
 *    every placement), the memo is input (low work RAM in typical, outVdp1
 *    and outVdp2), the destination is output (VDP1 or VDP2 there).
 * C: the snapshot is output, the table input. D: the runs are output. */
static const struct place_decl place_a[]={{"_a_shadow",PC_OUTPUT},{"_a_mask",PC_OUTPUT},{"_a_flag",PC_OUTPUT},{NULL,0}};
static const struct place_decl place_b[]={{"_b_rec",PC_TABLES},{"_gate",PC_TABLES},{"_b_dst",PC_OUTPUT},{NULL,0}};
/* The same with the memo declared as tables: high work RAM in outVdp1 and
 * outVdp2, to show what moving the memo out of low work RAM is worth. */
static const struct place_decl place_b_high[]={{"_b_rec",PC_TABLES},{"_gate",PC_TABLES},{"_b_memo",PC_TABLES},{"_b_dst",PC_OUTPUT},{NULL,0}};
static const struct place_decl place_c[]={{"_c_dst",PC_OUTPUT},{NULL,0}};
static const struct place_decl place_d[]={{"_d_runs",PC_OUTPUT},{NULL,0}};

/* ---- A: blocks ---- */
static uint8_t hs[MAXG*64+64],hd[MAXG*64+64],hw[MAXG*64+64],hg[MAXG*64+64];
static uint32_t mw[MAXG/32+2];
static void gen_pair(uint8_t *s,uint8_t *d,size_t bsize,uint32_t groups,unsigned mode){
 const size_t bytes=bsize*groups;
 for(size_t i=0;i<bytes;i++)s[i]=d[i]=(uint8_t)rnd();
 for(uint32_t g=0;g<groups;g++){
  uint8_t *b=s+bsize*g;
  switch(mode){
  case 0:break;                                                     /* all equal */
  case 1:for(size_t i=0;i<bsize;i++)b[i]=(uint8_t)~d[bsize*g+i];break; /* all bytes differ */
  case 2:if(below(2))b[below((uint32_t)bsize)]^=(uint8_t)(1u<<below(8));break;
  case 3:if(below(8)==0)b[bsize-1-below(4)]^=0x80u;break;           /* the last word */
  default:for(size_t i=0;i<bsize;i++)if(below(24)==0)b[i]=(uint8_t)rnd();break;
  }
 }
}
static uint64_t va_batches,va_singles,va_groups,va_unaligned;
static void verify_a_batch(int longs,u32 off,uint32_t groups,unsigned mode,uint64_t it){
 const size_t bsize=longs?64u:32u,bytes=bsize*groups;
 const u32 sa=S("a_shadow")+off,ra=S("a_src")+off,ma=S("a_mask");
 gen_pair(hs,hd,bsize,groups,mode);
 fill_bytes(S("a_shadow"),0xC5,MAXG*64+64);fill_bytes(S("a_src"),0xC5,MAXG*64+64);
 put_bytes(sa,hs,bytes);put_bytes(ra,hd,bytes);
 const uint32_t words=(groups+31u)/32u;
 for(uint32_t i=0;i<words+1u;i++)wl(&cpu,ma+4u*i,0xA5A5A5A5u);
 memcpy(hw,hs,bytes);
 const uint32_t want=mo_sync_batch(hw,hd,bsize,groups,mw,MO_MUT_NONE);
 const struct cost c=longs?call("l16_batch",groups,0,0,0):call("w16_batch",off,groups,0,0);
 if(c.result!=want)bad(longs?"l16 batch count":"w16 batch count",it);
 get_bytes(sa,hg,bytes+64u);
 if(memcmp(hg,hw,bytes))bad("batch shadow",it);
 for(size_t i=0;i<64u-off;i++)if(hg[bytes+i]!=0xC5u)bad("batch write past the shadow",it);
 get_bytes(ra,hg,bytes);if(memcmp(hg,hd,bytes))bad("batch src written",it);
 for(uint32_t i=0;i<words;i++)if(rl(&cpu,ma+4u*i)!=mw[i])bad("batch mask word",it);
 if(rl(&cpu,ma+4u*words)!=0xA5A5A5A5u)bad("batch mask written past the last word",it);
 va_batches++;va_groups+=groups;if(off&3u)va_unaligned++;
}
static void verify_a_single(int longs,u32 off,unsigned mode,uint64_t it){
 const size_t bsize=longs?64u:32u;const uint32_t g=below(MAXG);
 const u32 sa=S("a_shadow")+off+(u32)(bsize*g),ra=S("a_src")+off+(u32)(bsize*g);
 gen_pair(hs,hd,bsize,1,mode);if(mode==2&&below(2))hs[below((uint32_t)bsize)]^=1u;
 put_bytes(sa,hs,bsize);put_bytes(ra,hd,bsize);fill_bytes(sa+(u32)bsize,0xC5,8);
 memcpy(hw,hs,bsize);
 const bool want=mo_sync(hw,hd,bsize,MO_MUT_NONE);
 const struct cost c=longs?call("l16_one",g,0,0,0):call("w16_one",off,g,0,0);
 if(c.result!=(u32)want)bad("single changed flag",it);
 get_bytes(sa,hg,bsize+8u);if(memcmp(hg,hw,bsize))bad("single shadow",it);
 for(unsigned i=0;i<8u;i++)if(hg[bsize+i]!=0xC5u)bad("single write past the block",it);
 va_singles++;if(off&3u)va_unaligned++;
}
static void verify_a(uint64_t scale){
 for(uint64_t it=0;it<scale;it++){
  const int longs=(int)(it&1u);
  const u32 off=longs?0u:(below(3)==0?2u:0u);
  const uint32_t groups=it<2u*140u?(uint32_t)(it/2u):below(MAXG+1u);
  verify_a_batch(longs,off,groups,below(5),it);
  verify_a_single(longs,longs?0u:(below(3)==0?2u:0u),below(5),it);
 }
 /* the per-block loop: flags against the oracle */
 for(uint64_t it=0;it<scale/8u+1u;it++){
  const uint32_t groups=below(MAXG+1u);
  gen_pair(hs,hd,32,groups,below(5));
  put_bytes(S("a_shadow"),hs,32u*groups);put_bytes(S("a_src"),hd,32u*groups);
  memcpy(hw,hs,32u*groups);
  uint32_t want=0;uint8_t flag[MAXG];
  for(uint32_t g=0;g<groups;g++){flag[g]=mo_sync(hw+32u*g,hd+32u*g,32,MO_MUT_NONE);want+=flag[g];}
  if(call("w16_loop",groups,0,0,0).result!=want)bad("w16 loop count",it);
  get_bytes(S("a_shadow"),hg,32u*groups);if(memcmp(hg,hw,32u*groups))bad("w16 loop shadow",it);
  for(uint32_t g=0;g<groups;g++)if(rb(&cpu,S("a_flag")+g)!=flag[g])bad("w16 loop flag",it);
 }
}

/* ---- B: keyed writes ---- */
static uint32_t bmemo[MAXR+1],bwant[MAXR+1],bkey[MAXR];
static uint16_t bword[4*MAXR];
static mo_write_t wlist[4*MAXR];
static uint64_t vb_cases,vb_writes;
static void put_gate(const mo_gate_t *g){
 const u32 a=S("gate");
 for(unsigned k=0;k<4u;k++){wl(&cpu,a+4u*k,g->gate[k]);ww(&cpu,a+20u+2u*k,g->offset[k]);}
 wl(&cpu,a+16u,g->stride);wl(&cpu,a+28u,g->always);
}
static void put_recs(uint32_t n){
 const u32 r=S("b_rec"),m=S("b_memo");
 for(uint32_t i=0;i<n;i++){wl(&cpu,r+12u*i,bkey[i]);for(unsigned k=0;k<4u;k++)ww(&cpu,r+12u*i+4u+2u*k,bword[4u*i+k]);wl(&cpu,m+4u*i,bmemo[i]);}
 wl(&cpu,m+4u*n,0x5A5A5A5Au);
}
/* Calls entry over n records with the log on; compares with the oracle. */
static struct cost run_b(const char *entry,uint32_t n,const mo_gate_t *g,uint64_t it){
 memcpy(bwant,bmemo,4u*n);
 const uint32_t want=mo_put4(bwant,bkey,bword,n,g,wlist,MO_MUT_NONE);
 put_recs(n);
 dlo=S("b_dst");dhi=dlo+2u*(MAXR*16u+64u);dlog_n=0;dlog_on=1;
 const struct cost c=call(entry,n,0,0,0);
 dlog_on=0;
 if(c.result!=want)bad("keyed write count",it);
 if(dlog_n!=want)bad("keyed destination write count",it);
 for(uint32_t i=0;i<want;i++)if(dlog[i].off!=wlist[i].offset||dlog[i].value!=wlist[i].value)bad("keyed destination write",it);
 const u32 m=S("b_memo"),r=S("b_rec");
 for(uint32_t i=0;i<n;i++){
  if(rl(&cpu,m+4u*i)!=bwant[i])bad("keyed memo",it);
  if(rl(&cpu,r+12u*i)!=bkey[i])bad("keyed records written",it);
 }
 if(rl(&cpu,m+4u*n)!=0x5A5A5A5Au)bad("keyed memo written past n",it);
 vb_writes+=want;
 return c;
}
static uint32_t rand_gate(void){
 switch(below(7)){case 0:return 0;case 1:return 0xFFFFFFFFu;case 2:return 0xFFFF0000u;case 3:return 0x0000FFFFu;
 case 4:return UINT32_C(1)<<below(32);case 5:return 0xFF000000u>>(8u*below(4));default:return rnd();}
}
static void gen_records(uint32_t n,unsigned flip_pct){
 for(uint32_t i=0;i<n;i++){
  bmemo[i]=rnd();uint32_t k=bmemo[i];
  if(below(100)<flip_pct)k^=below(4)?(UINT32_C(1)<<below(32)):rnd();
  bkey[i]=k;for(unsigned j=0;j<4u;j++)bword[4u*i+j]=(uint16_t)rnd();
 }
}
static void verify_b(uint64_t scale){
 if(call("layout",0,0,0,0).result!=(32u<<16|12u<<8|28u))bad("descriptor or record layout",0);
 for(uint64_t it=0;it<scale;it++){
  mo_gate_t g;
  static const uint32_t strides[]={8,16,32,2,0};
  g.stride=below(3)?strides[below(5)]:2u*below(17);
  for(unsigned k=0;k<4u;k++){g.gate[k]=rand_gate();g.offset[k]=(uint16_t)(below(5)?2u*below(16):2u*k);}
  g.always=below(16);
  put_gate(&g);
  const uint32_t n=it<16u?(uint32_t)it:below(it%8u?80u:MAXR+1u);
  gen_records(n,below(4)?below(101):(below(2)?0u:100u));
  run_b(it&1u?"put4_one":"put4",n,&g,it);
  vb_cases++;
 }
}

/* ---- C: copy until a marker ---- */
static uint16_t csrc[4*MAXC+32],cwant[4*MAXC+32],cgot[4*MAXC+32];
static uint64_t vc_cases,vc_records,vc_unaligned;
static void gen_copy(uint32_t max,uint32_t l1,uint32_t l3,uint32_t at){
 for(uint32_t i=0;i<4u*max;i++)csrc[i]=(uint16_t)rnd();
 for(uint32_t i=0;i<max;i++){uint16_t *r=csrc+4u*i;
  if(i<at){if(!l1||!l3)break;if(r[1]>=l1)r[1]=(uint16_t)below(l1>0xFFFFu?0x10000u:l1);if(r[3]>=l3)r[3]=(uint16_t)below(l3>0xFFFFu?0x10000u:l3);}
  else if(i==at){if(l1<=0xFFFFu&&(below(2)||l3>0xFFFFu))r[1]=(uint16_t)(l1+below(0x10000u-l1));else if(l3<=0xFFFFu)r[3]=(uint16_t)(l3+below(0x10000u-l3));}}
}
/* Calls entry with the read guard after the marker record; returns its cost.
 * kind 0: copy8(off, max, l1, l3); 1: plain_copy8(max), limits 0x8000 and
 * 0xFF00; 2: full_copy(max), no marker test and no guard. */
static struct cost run_c(const char *entry,u32 off,uint32_t max,uint32_t l1,uint32_t l3,int kind,uint64_t it){
 const int verify_full=kind==2;
 const u32 sa=S("c_src")+off,da=S("c_dst");
 fill_bytes(S("c_src"),0x77,2u*(4u*MAXC+32u));put_words(sa,csrc,4u*max);
 fill_bytes(da,0xEE,2u*(4u*MAXC+32u));
 for(size_t i=0;i<sizeof cwant/2;i++)cwant[i]=0xEEEEu;
 const uint32_t want=mo_copy8_until(cwant+off/2u,csrc,max,l1,l3,MO_MUT_NONE);
 const uint32_t copied=want<max?want+1u:max;
 glo=sa+8u*copied;ghi=S("c_src")+2u*(4u*MAXC+32u);guard_on=!verify_full;
 const struct cost c=kind?call(entry,max,0,0,0):call(entry,off,max,l1,l3);
 guard_on=0;
 if(c.result!=(verify_full?max:want))bad("copy8 record count",it);
 get_words(da,cgot,4u*MAXC+32u);
 if(verify_full){if(memcmp(cgot,csrc,8u*max))bad("full copy",it);}
 else if(memcmp(cgot,cwant,sizeof cgot))bad("copy8 destination and guard",it);
 vc_records+=copied;
 return c;
}
static void verify_c(uint64_t scale){
 static const uint32_t lims[][2]={{0x8000u,0xFF00u},{0x10000u,0x10000u},{0u,0x10000u},{0x10000u,0u},{0xFFFFu,0xFFFFu},{1u,1u},{0x10000u,0xFF00u},{0x8000u,0x10000u}};
 uint64_t it=0;
 for(unsigned l=0;l<sizeof lims/sizeof lims[0];l++)
  for(uint32_t max=0;max<=24u;max++)for(uint32_t at=0;at<=max+1u;at+=1u+max/6u)
   for(u32 off=0;off<=2u;off+=2u){gen_copy(max,lims[l][0],lims[l][1],at);run_c("copy8",off,max,lims[l][0],lims[l][1],0,it++);vc_cases++;if(off)vc_unaligned++;}
 for(uint64_t k=0;k<scale;k++){
  const uint32_t max=k<4u?MAXC-(uint32_t)k:below(MAXC+1u);
  const uint32_t l1=below(3)?(below(2)?0x10000u:below(0x10001u)):0x8000u,l3=below(3)?(below(2)?0x10000u:below(0x10001u)):0xFF00u;
  const u32 off=below(4)==0?2u:0u;
  gen_copy(max,l1,l3,k<2u?max-1u:below(max+2u));run_c("copy8",off,max,l1,l3,0,it++);vc_cases++;if(off)vc_unaligned++;
 }
}

/* ---- D: run diff ---- */
static uint16_t da_[4*MAXD],db_[4*MAXD];
static mo_run_t rwant[MAXD+2];
static uint64_t vd_cases,vd_runs;
static void gen_diff(uint32_t n,unsigned equal_pct){
 for(uint32_t i=0;i<n;i++){for(unsigned j=0;j<4u;j++)da_[4u*i+j]=db_[4u*i+j]=(uint16_t)rnd();
  if(below(100)>=equal_pct){if(below(2))db_[4u*i+below(4)]^=(uint16_t)(1u<<below(16));else for(unsigned j=0;j<4u;j++)db_[4u*i+j]=(uint16_t)~da_[4u*i+j];}}
}
static struct cost run_d(const char *entry,uint32_t n,uint32_t max_runs,uint64_t it){
 put_words(S("d_a"),da_,4u*n);put_words(S("d_b"),db_,4u*n);
 const u32 ra=S("d_runs");for(uint32_t i=0;i<MAXD;i++)wl(&cpu,ra+4u*i,0xBEEFBEEFu);
 /* the oracle reads byte images in target (big-endian) order */
 uint8_t ia[8*MAXD],ib[8*MAXD];get_bytes(S("d_a"),ia,8u*n);get_bytes(S("d_b"),ib,8u*n);
 const uint32_t want=mo_diff8_runs(ia,ib,n,rwant,max_runs,MO_MUT_NONE);
 const struct cost c=call(entry,n,max_runs,0,0);
 if(c.result!=want)bad("diff run total",it);
 const uint32_t stored=want<max_runs?want:max_runs;
 for(uint32_t i=0;i<stored;i++)if(rw(&cpu,ra+4u*i)!=rwant[i].start||rw(&cpu,ra+4u*i+2u)!=rwant[i].count)bad("diff run",it);
 for(uint32_t i=stored;i<MAXD;i++)if(rl(&cpu,ra+4u*i)!=0xBEEFBEEFu)bad("diff run written past the stored ones",it);
 vd_runs+=want;
 return c;
}
static void verify_d(uint64_t scale){
 for(uint64_t it=0;it<scale;it++){
  const uint32_t n=it<64u?(uint32_t)it:below(MAXD+1u);
  gen_diff(n,it%5u==0u?(it%10u?0u:100u):below(101));
  run_d(it&1u?"diff8_c":"diff8",n,below(4)?MAXD:below(12),it);vd_cases++;
 }
}

/* ---- cost ---- */
/* Each point is called twice with the same inputs and the second call is
 * reported, so a placement's warm cache is that of the kernel called in a
 * loop. The placement log keeps both calls; the collector takes the last. */
static int first_row=1;
static void row_begin(const char *name,const char *entry,const char *unit,const char *fixture){
 printf("%s\"%s\":{\"entry\":\"%s\",\"unit\":\"%s\",\"fixture\":\"%s\",\"points\":{",first_row?"":",",name,entry,unit,fixture);first_row=0;
}
static void point(int first,uint32_t at,const char *tag,struct cost c,uint32_t result){
 printf("%s\"%u\":{\"tag\":\"%s\",\"cycles\":%u,\"instructions\":%"PRIu64",\"dataReads\":%"PRIu64",\"dataWrites\":%"PRIu64",\"result\":%u}",
  first?"":",",at,tag,c.cycles,c.instructions,c.reads,c.writes,result);
}
static void row_end(uint32_t from,uint32_t to){printf("},\"slope\":[%u,%u]}",from,to);}

/* A: states steady (no block changed), some (every tenth), all. Changes are
 * in the first 30 bytes, so the 15-word control sees them too. */
static void cost_a_point(const char *row,const char *entry,int longs,uint32_t groups,unsigned state,int first){
 const size_t bsize=longs?64u:32u,bytes=bsize*MAXG;
 rng=UINT32_C(0xa11ce5)+groups*7u+state;
 for(size_t i=0;i<bytes;i++)hs[i]=hd[i]=(uint8_t)rnd();
 for(uint32_t g=0;g<groups;g++)if(state==2u||(state==1u&&g%10u==3u))hs[bsize*g+below(30)]^=(uint8_t)(1u+below(255));
 memcpy(hw,hs,bytes);
 const uint32_t want=mo_sync_batch(hw,hd,bsize,groups,mw,MO_MUT_NONE);
 const bool plain=!strcmp(entry,"plain_w15");
 char tag[80];snprintf(tag,sizeof tag,"memo.%s.n%u",row,groups);
 struct cost c={0};
 for(int rep=0;rep<2;rep++){
  put_bytes(S("a_shadow"),hs,bsize*groups);put_bytes(S("a_src"),hd,bsize*groups);
  place_tag(tag,place_a);
  c=!strcmp(entry,"w16_batch")?call(entry,0,groups,0,0):call(entry,groups,0,0,0);
  if(c.result!=want)bad(tag,(uint64_t)rep);
  get_bytes(S("a_shadow"),hg,bsize*groups);
  if(plain){for(uint32_t g=0;g<groups;g++)if(memcmp(hg+bsize*g,hw+bsize*g,30))bad(tag,2);}
  else if(memcmp(hg,hw,bsize*groups))bad(tag,3);
  if(strstr(entry,"batch"))for(uint32_t i=0;i<(groups+31u)/32u;i++)if(rl(&cpu,S("a_mask")+4u*i)!=mw[i])bad(tag,4);
 }
 point(first,groups,tag,c,want);
}
static void cost_a(void){
 static const char *const states[3]={"steady","tenth","all"};
 static const struct {const char *entry;int longs;} entries[]={{"w16_batch",0},{"w16_loop",0},{"plain_w15",0},{"l16_batch",1}};
 static const uint32_t sizes[3]={1,75,90};
 for(unsigned s=0;s<3u;s++)for(unsigned e=0;e<sizeof entries/sizeof entries[0];e++){
  char row[64];snprintf(row,sizeof row,"A.%s.%s",states[s],entries[e].entry);
  row_begin(row,entries[e].entry,"group",states[s]);
  for(unsigned k=0;k<3u;k++)cost_a_point(row,entries[e].entry,entries[e].longs,sizes[k],s,k==0);
  row_end(1,90);
 }
}

/* B: the documented example layout. States: steady (no key changed),
 * typical (the two key bytes changed in 12% of records, the 16-bit key in
 * 50%), all (both changed in every record). */
static const mo_gate_t example_gate={{0xFFFF0000u,0xFFFF0000u,0x0000FFFFu,0u},32u,0x8u,{8u,10u,6u,12u}};
static void cost_b_point(const char *row,const char *entry,uint32_t n,unsigned state,int first,const struct place_decl *decl){
 rng=UINT32_C(0xb0b)+n*3u+state;
 for(uint32_t i=0;i<n;i++){
  bmemo[i]=rnd();uint32_t k=bmemo[i];
  if(state==2u)k^=0x00010001u|(rnd()&0xFFFEFFFEu);
  else if(state==1u){if(below(100)<12u)k^=UINT32_C(1)<<(16u+below(16));if(below(2))k^=UINT32_C(1)<<below(16);}
  bkey[i]=k;for(unsigned j=0;j<4u;j++)bword[4u*i+j]=(uint16_t)rnd();
 }
 mo_gate_t g=example_gate;if(!strcmp(entry,"plain_all"))g.always=0xFu;
 put_gate(&example_gate);
 char tag[80];snprintf(tag,sizeof tag,"memo.%s.n%u",row,n);
 struct cost c={0};
 for(int rep=0;rep<2;rep++){place_tag(tag,decl);c=run_b(entry,n,&g,(uint64_t)rep);}
 point(first,n,tag,c,c.result);
}
static void cost_b(void){
 static const char *const states[3]={"steady","typical","all"};
 static const char *const entries[]={"put4","put4_one","plain_memo","plain_all"};
 static const uint32_t sizes[3]={1,357,388};
 for(unsigned s=0;s<3u;s++)for(unsigned e=0;e<sizeof entries/sizeof entries[0];e++){
  if(!strcmp(entries[e],"plain_all")&&s)continue; /* no memo: the state does not matter */
  char row[64];snprintf(row,sizeof row,"B.%s.%s",states[s],entries[e]);
  row_begin(row,entries[e],"record",states[s]);
  for(unsigned k=0;k<3u;k++)cost_b_point(row,entries[e],sizes[k],s,k==0,place_b);
  row_end(1,388);
 }
 for(unsigned e=0;e<3u;e++){
  char row[64];snprintf(row,sizeof row,"B.typicalMemoHigh.%s",entries[e]);
  row_begin(row,entries[e],"record","typical, memo in high work RAM");
  for(unsigned k=0;k<3u;k++)cost_b_point(row,entries[e],sizes[k],1,k==0,place_b_high);
  row_end(1,388);
 }
}

/* C: 1,024 records, limits 0x8000 and 0xFF00; marker at 0, 209 or none. */
static void cost_c_point(const char *row,const char *entry,uint32_t at,int first){
 rng=UINT32_C(0xc0c0)+at;
 gen_copy(MAXC,0x8000u,0xFF00u,at);
 char tag[80];snprintf(tag,sizeof tag,"memo.%s.m%u",row,at>MAXC?MAXC:at);
 const int kind=!strcmp(entry,"plain_copy8")?1:0;
 struct cost c={0};
 for(int rep=0;rep<2;rep++){place_tag(tag,place_c);c=run_c(entry,0,MAXC,0x8000u,0xFF00u,kind,(uint64_t)rep);}
 point(first,at>MAXC?MAXC:at,tag,c,c.result);
}
static void cost_c(void){
 row_begin("C.copy8","copy8","record","marker at m of 1024");
 cost_c_point("C.copy8","copy8",0,1);cost_c_point("C.copy8","copy8",209,0);cost_c_point("C.copy8","copy8",MAXC+1u,0);
 row_end(0,209);
 row_begin("C.plain_copy8","plain_copy8","record","marker at m of 1024");
 cost_c_point("C.plain_copy8","plain_copy8",0,1);cost_c_point("C.plain_copy8","plain_copy8",209,0);cost_c_point("C.plain_copy8","plain_copy8",MAXC+1u,0);
 row_end(0,209);
}
/* the full copy has no marker test: its points are copy lengths */
static void cost_c_full(void){
 row_begin("C.full_copy","full_copy","record","no marker test");
 static const uint32_t lens[3]={1,210,MAXC};
 for(unsigned k=0;k<3u;k++){
  rng=UINT32_C(0xf011)+k;for(uint32_t i=0;i<4u*MAXC;i++)csrc[i]=(uint16_t)rnd();
  char tag[80];snprintf(tag,sizeof tag,"memo.C.full_copy.m%u",lens[k]);
  struct cost c={0};
  for(int rep=0;rep<2;rep++){place_tag(tag,place_c);c=run_c("full_copy",0,lens[k],0x8000u,0xFF00u,2,(uint64_t)rep);}
  point(k==0,lens[k],tag,c,c.result);
 }
 row_end(1,MAXC);
}

/* D: 208 records, 66% equal (states: also all equal, all different). */
static void cost_d(void){
 static const char *const states[3]={"equal66","equal100","equal0"};
 static const unsigned pct[3]={66,100,0};
 for(unsigned s=0;s<3u;s++)for(unsigned e=0;e<2u;e++){
  const char *entry=e?"diff8_c":"diff8";
  char row[64];snprintf(row,sizeof row,"D.%s.%s",states[s],entry);
  row_begin(row,entry,"record",states[s]);
  static const uint32_t sizes[2]={1,208};
  for(unsigned k=0;k<2u;k++){
   rng=UINT32_C(0xd1ff)+s*31u+k;gen_diff(sizes[k],pct[s]);
   char tag[80];snprintf(tag,sizeof tag,"memo.%s.n%u",row,sizes[k]);
   struct cost c={0};
   for(int rep=0;rep<2;rep++){place_tag(tag,place_d);c=run_d(entry,sizes[k],MAXD,(uint64_t)rep);}
   point(k==0,sizes[k],tag,c,c.result);
  }
  row_end(1,208);
 }
}

int main(int argc,char**argv){
 if(argc<3){fprintf(stderr,"usage: memo_harness model.elf verify-scale\n");return 2;}
 const uint64_t scale=strtoull(argv[2],0,10);
 load_elf_symbols(argv[1]);load_elf(argv[1]);place_init(argv[1]);
 cpu.model=SHMT_SH2;cpu.MappedMemoryReadByte=h_rb;cpu.MappedMemoryReadWord=h_rw;cpu.MappedMemoryReadLong=h_rl;cpu.MappedMemoryWriteByte=h_wb;cpu.MappedMemoryWriteWord=h_ww;cpu.MappedMemoryWriteLong=h_wl;
 for(unsigned i=0;i<65536;i++)cpu.opcodes[i]=decode(SHMT_SH2,(u16)i);for(unsigned i=0;i<256;i++)cpu.fetchlist[i]=fetch;
 verify_a(scale);verify_b(scale);verify_c(scale);verify_d(scale);
 printf("{\"schema\":1,\"kind\":\"Yabause isolated SH2 interpreter flat-RAM model, not Saturn timing\",");
 printf("\"verified\":{\"blockBatches\":%"PRIu64",\"blockSingles\":%"PRIu64",\"blocksInBatches\":%"PRIu64",\"blockCallsUnaligned\":%"PRIu64","
  "\"keyedCases\":%"PRIu64",\"keyedDestinationWrites\":%"PRIu64",\"copyCases\":%"PRIu64",\"copyRecords\":%"PRIu64",\"copyCasesUnaligned\":%"PRIu64","
  "\"diffCases\":%"PRIu64",\"diffRuns\":%"PRIu64",\"against\":\"tests/memo_oracle.h; keyed destination writes logged in order, 16-bit only, never read; no source read after the marker record; guards after every output\"},",
  va_batches,va_singles,va_groups,va_unaligned,vb_cases,vb_writes,vc_cases,vc_records,vc_unaligned,vd_cases,vd_runs);
 printf("\"rows\":{");
 cost_a();cost_b();cost_c();cost_c_full();cost_d();
 printf("}}\n");
 return 0;
}
