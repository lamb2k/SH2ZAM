/* SPDX-License-Identifier: GPL-2.0-or-later
 * Executed-instruction checks and cycle model for the cell-records-1 kernels
 * (cell_records_experimental.h). Runs the actual big-endian SH-2 object code in
 * Yabause's unmodified interpreter, like harness.c, whose loader and call
 * wrapper it reuses. Flat RAM, no caches or wait states: MODELLED cycles, not
 * Saturn timing. Copyright (c) 2026 SH2ZAM Contributors. This optional model
 * harness is licensed separately from SH2ZAM's MIT core; see LICENSE.
 *
 * usage: cell_records_harness model.elf verify-scale > results.json
 * The ELF must contain bridge.o from benchmarks/cell-records-model and the
 * library. Before any timing, every form of the trusted, the checked and the
 * plain entry is run on random lists and compared, entry for entry and in the
 * running position after the call, with the independent oracle in
 * tests/cell_records_oracle.h and with the host build of the library source.
 * Any mismatch exits non-zero. */
#define main sh2_model_main
#include "harness.c"
#undef main
#include "../../tests/cell_records_oracle.h"
#include "sh2zam/cell_records_experimental.h"

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
static u32 symc(const char *name){char tmp[140];snprintf(tmp,sizeof tmp,"_%s",name);return sym(tmp);}
static struct cost call_sr(const char *name,u32 n,u32 sr){char tmp[140];snprintf(tmp,sizeof tmp,"_%s",name);return invoke_sr(tmp,n,0,0,0,sr);}
static struct cost call(const char *name,u32 n){return call_sr(name,n,0);}

enum {CORPORA=16};
struct stats {double mean,median;unsigned min,max;double instr;};
static int cmp_u(const void*a,const void*b){unsigned x=*(const unsigned*)a,y=*(const unsigned*)b;return x<y?-1:x>y;}
static struct stats summarize(unsigned *cycles,size_t n,uint64_t instr){
 qsort(cycles,n,sizeof *cycles,cmp_u);double sum=0;for(size_t i=0;i<n;i++)sum+=cycles[i];
 return (struct stats){sum/(double)n,n&1?cycles[n/2]:(cycles[n/2-1]+cycles[n/2])/2.0,cycles[0],cycles[n-1],(double)instr/(double)n};
}
static void print_stats(struct stats s){printf("{\"meanCycles\":%.2f,\"medianCycles\":%.1f,\"minCycles\":%u,\"maxCycles\":%u,\"meanInstructions\":%.2f}",s.mean,s.median,s.min,s.max,s.instr);}

static void bad(const char *what,uint64_t it){fprintf(stderr,"cell-records model mismatch: %s (case %"PRIu64")\n",what,it);exit(3);}

enum {NMAX=64};
static uint64_t rs=UINT64_C(0x5eed0ce11c0ffee5);
static uint32_t rnd(void){rs^=rs<<13;rs^=rs>>7;rs^=rs<<17;return (uint32_t)(rs>>16);}
static void put_bytes(const char *s,const uint8_t *b,u32 n){for(u32 i=0;i<n;i++)wb(&cpu,symc(s)+i,b[i]);}
static void get_bytes(const char *s,uint8_t *b,u32 n){for(u32 i=0;i<n;i++)b[i]=rb(&cpu,symc(s)+i);}

/* one random case: parameters of the form, a list of n cells, and a starting position */
struct cs {sh2z_cellrec_params_t p;sh2z_cellrec_pos_t pos;uint8_t rec[NMAX*4],off[NMAX*4];unsigned n;};
static void make_case(struct cs *c,unsigned form,unsigned n,unsigned kind){
 c->n=n;c->p.form=form;
 c->p.pos_mask=kind==0?UINT32_C(0x03ff03ff):rnd();c->p.pos_or=kind==0?(rnd()&UINT32_C(0xfc00fc00)):rnd();
 c->p.mirror_bias=kind==0?(rnd()&0xffu):rnd();
 c->p.attr_keep=kind==2?rnd():UINT32_C(0xffffffe0);c->p.attr_or=kind==2?rnd():(rnd()&0x1fu);
 for(unsigned i=0;i<NMAX*4;i++){c->rec[i]=(uint8_t)rnd();c->off[i]=(kind==2||(form&1u))?(uint8_t)rnd():(uint8_t)((int)(rnd()%48u)-24);}
 if((form&1u)&&kind!=2)for(unsigned i=0;i<NMAX*4;i+=2){c->off[i]=(rnd()&1u)?0xff:0x00;c->off[i+1]=(uint8_t)rnd();}
 c->pos.x=rnd();c->pos.y=rnd();
}
static void load_case(const struct cs *c){
 put_bytes("rec",c->rec,NMAX*4);put_bytes("off",c->off,NMAX*4);
 const u32 pb=symc("prm");const uint32_t w[6]={c->p.form,c->p.pos_mask,c->p.pos_or,c->p.mirror_bias,c->p.attr_keep,c->p.attr_or};
 for(u32 i=0;i<6;i++)wl(&cpu,pb+4*i,w[i]);
 wl(&cpu,symc("pos"),c->pos.x);wl(&cpu,symc("pos")+4,c->pos.y);
 for(u32 i=0;i<NMAX*8;i++)wb(&cpu,symc("out")+i,0xa5);
}
static uint64_t v_cases,v_cells;
static void verify_one(uint64_t it,unsigned form,unsigned n,unsigned kind){
 struct cs c;make_case(&c,form,n,kind);
 uint8_t ex[NMAX*8],ho[NMAX*8],sh[NMAX*8];sh2z_cellrec_pos_t hp=c.pos;uint32_t ox=c.pos.x,oy=c.pos.y;
 memset(ex,0xa5,sizeof ex);memset(ho,0xa5,sizeof ho);
 cro_expand(&ox,&oy,form,c.p.pos_mask,c.p.pos_or,c.p.mirror_bias,c.p.attr_keep,c.p.attr_or,c.rec,c.off,ex,n,0);
 sh2z_cellrec_expand_trusted(&hp,&c.p,c.rec,c.off,ho,n);
 if(memcmp(ex,ho,n*8u)||hp.x!=(ox&0xffffu)||hp.y!=(oy&0xffffu))bad("host library against oracle",it);
 static const char *const names[3]={"fast_expand","checked_expand","plain_expand"};
 for(int e=0;e<3;e++){
  load_case(&c);
  struct cost k=call(names[e],n);if(k.result!=n)bad(names[e],it);
  get_bytes("out",sh,n*8u);
  for(u32 i=n*8u;i<NMAX*8u;i++)if(rb(&cpu,symc("out")+i)!=0xa5)bad("write past the entries",it);
  if(memcmp(sh,ex,n*8u))bad(names[e],it);
  if(rl(&cpu,symc("pos"))!=(ox&0xffffu)||rl(&cpu,symc("pos")+4)!=(oy&0xffffu))bad("running position",it);
 }
 v_cases++;v_cells+=n;
}
static void verify(uint64_t it){
 static const unsigned counts[]={1,2,3,4,5,8,16,17,40,63,64};
 for(unsigned f=0;f<12;f++)for(unsigned k=0;k<3;k++)verify_one(it,f,counts[(rnd())%11u],k);
}

/* ---- cost rows ---- */
static unsigned cur_form;
static void load_row(void){struct cs c;make_case(&c,cur_form,NMAX,0);load_case(&c);}
struct row {const char *name,*description;unsigned form,n;};
static const struct row rows[]={
 {"s8_keep_n40","byte offsets, no mirror, attribute kept, 40 cells",0,40},
 {"s8_replace_n40","byte offsets, no mirror, low field replaced, 40 cells",4,40},
 {"s8_add_n40","byte offsets, no mirror, byte added, 40 cells",8,40},
 {"s8_mirror_keep_n40","byte offsets, mirrored, attribute kept, 40 cells",2,40},
 {"s8_mirror_replace_n40","byte offsets, mirrored, low field replaced, 40 cells",6,40},
 {"s8_mirror_add_n40","byte offsets, mirrored, byte added, 40 cells",10,40},
 {"s16_keep_n40","word offsets, no mirror, attribute kept, 40 cells",1,40},
 {"s16_replace_n40","word offsets, no mirror, low field replaced, 40 cells",5,40},
 {"s16_add_n40","word offsets, no mirror, byte added, 40 cells",9,40},
 {"s16_mirror_keep_n40","word offsets, mirrored, attribute kept, 40 cells",3,40},
 {"s16_mirror_replace_n40","word offsets, mirrored, low field replaced, 40 cells",7,40},
 {"s16_mirror_add_n40","word offsets, mirrored, byte added, 40 cells",11,40},
 {"s8_keep_n4","byte offsets, no mirror, attribute kept, 4 cells",0,4},
 {"s8_mirror_keep_n4","byte offsets, mirrored, attribute kept, 4 cells",2,4},
 {"s8_add_n4","byte offsets, no mirror, byte added, 4 cells",8,4},
 {"s8_mirror_add_n4","byte offsets, mirrored, byte added, 4 cells",10,4},
};
static const char *const entries[]={"fast_expand","checked_expand","plain_expand",0};
static void cost_row(const struct row *r,bool *first){
 static unsigned cyc[2][CORPORA];static uint64_t ins[2];
 if(!*first)putchar(',');*first=false;
 printf("\"%s\":{\"description\":\"%s\",\"n\":%u,\"corpora\":%u,\"entries\":{",r->name,r->description,r->n,CORPORA);
 for(int e=0;entries[e];e++){
  memset(ins,0,sizeof ins);
  for(unsigned c=0;c<CORPORA;c++)for(int z=0;z<2;z++){
   rs=UINT64_C(0xce11c0571234)+(uint64_t)c*7919u+(uint64_t)(r-rows)*104729u;cur_form=r->form;load_row();
   const unsigned n=z?r->n:1u;struct cost k=call(entries[e],n);if(k.result!=n)bad(entries[e],c);
   cyc[z][c]=k.cycles;ins[z]+=k.instructions;
  }
  struct stats one=summarize(cyc[0],CORPORA,ins[0]),many=summarize(cyc[1],CORPORA,ins[1]);
  printf("%s\"%s\":{\"cyclesPerElement\":%.2f,\"instructionsPerElement\":%.2f,\"call1\":",e?",":"",entries[e],
   (many.mean-one.mean)/(r->n-1),(many.instr-one.instr)/(r->n-1));
  print_stats(one);printf(",\"call%u\":",r->n);print_stats(many);printf("}");
 }
 printf("}}");
}

int main(int argc,char**argv){
 if(argc<3){fprintf(stderr,"usage: cell_records_harness model.elf verify-scale\n");return 2;}
 const uint64_t scale=strtoull(argv[2],0,10);
 load_elf_symbols(argv[1]);load_elf(argv[1]);
 cpu.model=SHMT_SH2;cpu.MappedMemoryReadByte=rb;cpu.MappedMemoryReadWord=rw;cpu.MappedMemoryReadLong=rl;cpu.MappedMemoryWriteByte=wb;cpu.MappedMemoryWriteWord=ww;cpu.MappedMemoryWriteLong=wl;
 for(unsigned i=0;i<65536;i++)cpu.opcodes[i]=decode(SHMT_SH2,(u16)i);for(unsigned i=0;i<256;i++)cpu.fetchlist[i]=fetch;
 for(uint64_t it=0;it<scale;it++)verify(it);
 printf("{\"schema\":1,\"kind\":\"Yabause isolated SH2 interpreter flat-RAM model, not Saturn timing\",");
 printf("\"verified\":{\"lists\":%"PRIu64",\"cells\":%"PRIu64",\"entries\":\"fast_expand, checked_expand and plain_expand, all 12 forms, against tests/cell_records_oracle.h and the host build of the same library source: every entry word and the running position after the call, and nothing written past the entries\"},",v_cases,v_cells);
 bool first=true;printf("\"rows\":{");
 for(size_t i=0;i<sizeof rows/sizeof rows[0];i++){cost_row(&rows[i],&first);fflush(stdout);}
 printf("}}\n");
 return 0;
}
