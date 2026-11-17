/* SPDX-License-Identifier: GPL-2.0-or-later
 * Executed-instruction checks and cycle model for the integer kernels
 * (m68k.h, box2.h, cell16.h). Runs the actual big-endian SH-2 object code in
 * Yabause's unmodified interpreter, like harness.c, whose loader and call
 * wrapper it reuses. Flat RAM, no caches or wait states: MODELLED cycles, not
 * Saturn timing. Copyright (c) 2026 SH2ZAM Contributors. This optional model
 * harness is licensed separately from SH2ZAM's MIT core; see LICENSE.
 *
 * usage: integer_harness model.elf verify-count > results.json
 * The ELF must contain bridge.o from benchmarks/integer-model and the library
 * sources it calls; symbols are read from its symbol table. */
#define main sh2_model_main
#include "harness.c"
#undef main
#include "sh2zam/m68k.h"
#include "sh2zam/box2.h"
#include "sh2zam/cell16.h"
#include <assert.h>

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
/* Bridge symbols carry the sh-elf "_" prefix; callers pass them without it. */
static struct cost call(const char *name,u32 a,u32 b,u32 c,u32 d) {
 char tmp[140];snprintf(tmp,sizeof tmp,"_%s",name);return invoke(tmp,a,b,c,d);
}
static u32 symc(const char *name) {char tmp[140];snprintf(tmp,sizeof tmp,"_%s",name);return sym(tmp);}

static uint32_t rng_state=UINT32_C(0x2f6e1b93);
static uint32_t rnd(void) {rng_state^=rng_state<<13;rng_state^=rng_state>>17;rng_state^=rng_state<<5;return rng_state;}

/* ---- statistics over a corpus of per-call cycle counts ---- */
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

/* ---- 68000 arithmetic ---- */
enum {OP_MULU,OP_MULS,OP_DIVU,OP_DIVS};
static const char *op_names[]={"mulu_w","muls_w","divu_w","divs_w"};
typedef uint32_t (*ref_fn)(uint32_t*,uint16_t);
static const ref_fn refs[]={sh2z_m68k_ref_mulu_w,sh2z_m68k_ref_muls_w,sh2z_m68k_ref_divu_w,sh2z_m68k_ref_divs_w};
enum {KIND_ANY,KIND_INRANGE,KIND_OVERFLOW};
/* Draw (dn,src). KIND_INRANGE: divides that complete. KIND_OVERFLOW: divides that overflow. */
static void draw(int op,int kind,uint32_t *dn,uint16_t *src) {
 uint32_t r=rnd();*src=(uint16_t)rnd();*dn=rnd();
 if(op<OP_DIVU) {if(r&1)*dn&=0xFFFFu;return;}
 if(kind==KIND_ANY&&(r&1))return;
 if(*src==0)*src=1;
 const uint32_t mag=op==OP_DIVS&&(*src&0x8000u)?0x10000u-*src:*src;
 if(kind==KIND_OVERFLOW){ /* a quotient of at least 2^15 or 2^16 */
  const uint32_t lim=op==OP_DIVS?0x8001u:0x10000u;
  uint64_t n=(uint64_t)mag*(lim+(rnd()&0x7FFFu));if(n>UINT32_MAX)n=UINT32_MAX;
  *dn=(uint32_t)n;if(op==OP_DIVS&&(r&2))*dn=0u-*dn;return;
 }
 const uint32_t qmax=op==OP_DIVS?0x7FFFu:0xFFFFu;
 const uint32_t q=rnd()%(qmax+1u),rem=rnd()%mag;
 *dn=q*mag+rem;if(op==OP_DIVS&&(r&2))*dn=0u-*dn;
}
static uint64_t verify_m68k(uint64_t count) {
 uint64_t done=0;
 for(int op=0;op<4;op++) {
  char fast[32],plain[32];snprintf(fast,sizeof fast,"fast_%s",op_names[op]);snprintf(plain,sizeof plain,"plain_%s",op_names[op]);
  static const u32 edge_dn[]={0,1,0x7FFF,0x8000,0xFFFF,0x10000,0x7FFFFFFF,0x80000000u,0xFFFF0000u,0xFFFFFFFFu,0x7FFF7FFFu};
  static const uint16_t edge_src[]={0,1,2,0x7FFF,0x8000,0x8001,0xFFFF};
  for(uint64_t i=0;i<count+77;i++) {
   uint32_t dn;uint16_t src;
   if(i<77){dn=edge_dn[i%11];src=edge_src[i/11];} else draw(op,(int)(i%3),&dn,&src);
   uint32_t want=dn;const uint32_t want_ccr=refs[op](&want,src);
   struct cost c=call(fast,dn,src,0,0);
   if(c.result!=want||rl(&cpu,symc("ccr_out"))!=want_ccr){fprintf(stderr,"%s dn=%08x src=%04x got %08x/%x want %08x/%x\n",fast,dn,src,c.result,rl(&cpu,symc("ccr_out")),want,want_ccr);exit(3);}
   if(op<OP_DIVU&&(c.dmuls_ops||c.mac_ops)){fprintf(stderr,"unexpected MAC/DMULS\n");exit(3);}
   if(i%8==0){ /* the control must honour the same contract or the comparison is unfair */
    struct cost p=call(plain,dn,src,0,0);
    if(p.result!=want||rl(&cpu,symc("ccr_out"))!=want_ccr){fprintf(stderr,"%s dn=%08x src=%04x differs from the spec\n",plain,dn,src);exit(3);}
   }
   done++;
  }
 }
 return done;
}
static void cost_m68k(void) {
 enum {N=4096};
 static unsigned cf[N],cp[N];
 printf("\"m68k\":{");
 for(int op=0;op<4;op++) {
  char fast[32],plain[32];snprintf(fast,sizeof fast,"fast_%s",op_names[op]);snprintf(plain,sizeof plain,"plain_%s",op_names[op]);
  if(op)putchar(',');printf("\"%s\":{",op_names[op]);
  const int kinds=op<OP_DIVU?1:2;
  for(int k=0;k<kinds;k++) {
   const int kind=op<OP_DIVU?KIND_ANY:(k?KIND_OVERFLOW:KIND_INRANGE);
   uint64_t ifast=0,iplain=0;rng_state=UINT32_C(0x5eed1234)+(uint32_t)op;
   for(unsigned i=0;i<N;i++){uint32_t dn;uint16_t src;draw(op,kind,&dn,&src);
    struct cost a=call(fast,dn,src,0,0),b=call(plain,dn,src,0,0);cf[i]=a.cycles;cp[i]=b.cycles;ifast+=a.instructions;iplain+=b.instructions;}
   if(k)putchar(',');
   printf("\"%s\":{\"cases\":%u,\"fast\":",op<OP_DIVU?"all":(k?"overflow":"inRange"),(unsigned)N);print_stats(summarize(cf,N,ifast));
   printf(",\"plainC\":");print_stats(summarize(cp,N,iplain));printf("}");
  }
  printf("}");
 }
 printf("}");
}


/* ---- box overlap ---- */
#define BOX_A 64u
#define BOX_B 256u
static sh2z_ibox2_t host_q,host_a[BOX_A],host_b[BOX_B];
static sh2z_ibox2c_t host_cq,host_ca[BOX_A],host_cb[BOX_B];
static int32_t arena_coord(unsigned range) {
 switch(range){
 case 0:return (int32_t)(rnd()%9u)-4;                 /* tiny, lots of touching edges */
 case 1:return (int32_t)(rnd()%384u);                 /* screen-sized arena */
 case 2:return (int32_t)(rnd()%65536u)-32768;
 default:return (int32_t)rnd();                       /* full int32 */
 }
}
static void make_box(sh2z_ibox2_t *b,unsigned range,unsigned w) {
 int32_t x=arena_coord(range),y=range==1?(int32_t)(rnd()%224u):arena_coord(range);
 int32_t x1,y1;
 if(range>=3){x1=arena_coord(range);y1=arena_coord(range);if(x1<x){int32_t t=x;x=x1;x1=t;}if(y1<y){int32_t t=y;y=y1;y1=t;}}
 else{x1=x+(int32_t)(w?w:rnd()%72u);y1=y+(int32_t)(w?w:rnd()%72u);}
 b->min_x=x;b->min_y=y;b->max_x=x1;b->max_y=y1;
}
static void make_cbox(sh2z_ibox2c_t *b,unsigned range,unsigned w) {
 b->cx=arena_coord(range);b->cy=range==1?(int32_t)(rnd()%224u):arena_coord(range);
 uint32_t hx=range>=3?(rnd()&0x7FFFFFFFu):(w?w/2u:rnd()%36u),hy=range>=3?(rnd()&0x7FFFFFFFu):(w?w/2u:rnd()%36u);
 b->hx=(int32_t)hx;b->hy=(int32_t)hy;
}
static void put_box(u32 addr,const sh2z_ibox2_t *b){wl(&cpu,addr,(u32)b->min_x);wl(&cpu,addr+4,(u32)b->min_y);wl(&cpu,addr+8,(u32)b->max_x);wl(&cpu,addr+12,(u32)b->max_y);}
static void put_cbox(u32 addr,const sh2z_ibox2c_t *b){wl(&cpu,addr,(u32)b->cx);wl(&cpu,addr+4,(u32)b->cy);wl(&cpu,addr+8,(u32)b->hx);wl(&cpu,addr+12,(u32)b->hy);}
/* dense: every box overlaps the query (worst case for branchy rejection, best for the word builder). */
static void load_boxes(unsigned range,unsigned na,unsigned nb,int dense) {
 make_box(&host_q,range,range==1?32u:0u);make_cbox(&host_cq,range,range==1?32u:0u);
 for(unsigned i=0;i<na;i++){make_box(&host_a[i],range,range==1?32u:0u);make_cbox(&host_ca[i],range,range==1?32u:0u);}
 for(unsigned i=0;i<nb;i++){
  make_box(&host_b[i],range,0);make_cbox(&host_cb[i],range,0);
  if(dense){ /* centre every box on the query centre, any size */
   const int32_t cx=host_q.min_x+(host_q.max_x-host_q.min_x)/2,cy=host_q.min_y+(host_q.max_y-host_q.min_y)/2;
   const int32_t hx=(int32_t)(rnd()%36u),hy=(int32_t)(rnd()%36u);
   host_b[i]=(sh2z_ibox2_t){cx-hx,cy-hy,cx+hx,cy+hy};host_cb[i]=(sh2z_ibox2c_t){cx,cy,hx,hy};
  }
 }
 put_box(symc("box_query"),&host_q);put_cbox(symc("boxc_query"),&host_cq);
 for(unsigned i=0;i<na;i++){put_box(symc("box_a")+16*i,&host_a[i]);put_cbox(symc("boxc_a")+16*i,&host_ca[i]);}
 for(unsigned i=0;i<nb;i++){put_box(symc("box_b")+16*i,&host_b[i]);put_cbox(symc("boxc_b")+16*i,&host_cb[i]);}
}
static int64_t abs64(int64_t v){return v<0?-v:v;}
static int oracle_box(const sh2z_ibox2_t*a,const sh2z_ibox2_t*b,int ex){
 const int64_t x0=a->min_x,x1=a->max_x,y0=a->min_y,y1=a->max_y,u0=b->min_x,u1=b->max_x,v0=b->min_y,v1=b->max_y;
 return ex?(x0<u1&&u0<x1&&y0<v1&&v0<y1):(x0<=u1&&u0<=x1&&y0<=v1&&v0<=y1);
}
static int oracle_cbox(const sh2z_ibox2c_t*a,const sh2z_ibox2c_t*b,int ex){
 const int64_t dx=abs64((int64_t)a->cx-b->cx),dy=abs64((int64_t)a->cy-b->cy),sx=(int64_t)a->hx+b->hx,sy=(int64_t)a->hy+b->hy;
 return ex?(dx<sx&&dy<sy):(dx<=sx&&dy<=sy);
}
static void expect_bits(uint32_t *want,unsigned rows,unsigned cols,int centre,int many,int ex) {
 const unsigned rw=(cols+31u)/32u;memset(want,0,rows*rw*4);
 for(unsigned r=0;r<rows;r++)for(unsigned j=0;j<cols;j++){
  int hit=many?(centre?oracle_cbox(&host_ca[r],&host_cb[j],ex):oracle_box(&host_a[r],&host_b[j],ex))
              :(centre?oracle_cbox(&host_cq,&host_cb[j],ex):oracle_box(&host_q,&host_b[j],ex));
  if(hit)want[r*rw+(j>>5)]|=1u<<(j&31u);
 }
}
static uint64_t verify_box(uint64_t count) {
 static uint32_t want[BOX_A*(BOX_B/32)];uint64_t done=0;
 const u32 out=symc("box_out"),hitsym=symc("box_hits");
 for(uint64_t it=0;it<count;it++){
  const unsigned range=(unsigned)(it%4u),ex=(unsigned)((it/4u)&1u),centre=(unsigned)((it/8u)&1u),many=(unsigned)((it/16u)&1u);
  const unsigned nb=(it%7u==0)?(unsigned)(rnd()%(BOX_B+1u)):(unsigned[]){0,1,31,32,33,64,100,255,256}[rnd()%9u];
  const unsigned na=many?1u+rnd()%BOX_A:1u;
  load_boxes(range,na,nb,(it%5u)==4u);
  const char*fast=many?(centre?"fast_boxc_many_many":"fast_box_many_many"):(centre?"fast_boxc_one_many":"fast_box_one_many");
  const char*plain=many?(centre?"plain_boxc_many_many":"plain_box_many_many"):(centre?"plain_boxc_one_many":"plain_box_one_many");
  const unsigned rows=many?na:1u,rw=(nb+31u)/32u;expect_bits(want,rows,nb,(int)centre,(int)many,(int)ex);
  unsigned want_hits=0;for(unsigned w=0;w<rows*rw;w++)for(unsigned j=0;j<32;j++)want_hits+=(want[w]>>j)&1u;
  for(int k=0;k<2;k++){
   for(unsigned w=0;w<BOX_A*(BOX_B/32);w++)wl(&cpu,out+4*w,0xDEADBEEFu);
   struct cost c=many?call(k?plain:fast,na,nb,ex,0):call(k?plain:fast,nb,ex,0,0);
   if(c.result!=1){fprintf(stderr,"%s rejected valid arguments\n",k?plain:fast);exit(3);}
   for(unsigned w=0;w<rows*rw;w++)if(rl(&cpu,out+4*w)!=want[w]){fprintf(stderr,"%s mismatch word %u (it %"PRIu64")\n",k?plain:fast,w,it);exit(3);}
   if(rl(&cpu,hitsym)!=want_hits){fprintf(stderr,"%s hit count %u want %u\n",k?plain:fast,rl(&cpu,hitsym),want_hits);exit(3);}
   if(rl(&cpu,out+4*(rows*rw))!=0xDEADBEEFu&&rows*rw<BOX_A*(BOX_B/32)){fprintf(stderr,"%s wrote beyond the result words\n",k?plain:fast);exit(3);}
  }
  if(!many){ /* trusted-buffer entry: same words, same count */
   for(unsigned w=0;w<BOX_A*(BOX_B/32);w++)wl(&cpu,out+4*w,0xDEADBEEFu);
   struct cost c=call(centre?"fast_boxc_one_many_unchecked":"fast_box_one_many_unchecked",nb,ex,0,0);(void)c;
   for(unsigned w=0;w<rw;w++)if(rl(&cpu,out+4*w)!=want[w]){fprintf(stderr,"unchecked mismatch\n");exit(3);}
   if(rl(&cpu,hitsym)!=want_hits){fprintf(stderr,"unchecked hit count\n");exit(3);}
  }
  /* The host build of the same source agrees (guards the compiled-for-SH-2 object against compiler differences). */
  uint32_t host_out[BOX_A*(BOX_B/32)+1];size_t hc=0;bool ok;
  if(many)ok=centre?sh2z_ibox2c_overlap_many_many(host_out,sizeof host_out/4,&hc,host_ca,na,host_cb,nb,(sh2z_box_edges_t)ex)
                   :sh2z_ibox2_overlap_many_many(host_out,sizeof host_out/4,&hc,host_a,na,host_b,nb,(sh2z_box_edges_t)ex);
  else ok=centre?sh2z_ibox2c_overlap_one_many(host_out,sizeof host_out/4,&hc,&host_cq,host_cb,nb,(sh2z_box_edges_t)ex)
                :sh2z_ibox2_overlap_one_many(host_out,sizeof host_out/4,&hc,&host_q,host_b,nb,(sh2z_box_edges_t)ex);
  if(!ok||hc!=want_hits||memcmp(host_out,want,rows*rw*4)){fprintf(stderr,"host library disagrees with the oracle\n");exit(3);}
  done++;
 }
 return done;
}
static void cost_box(void) {
 enum {CORPORA=128};
 static unsigned cf[CORPORA],cp[CORPORA];
 static const unsigned counts[]={16,64,256};
 printf("\"box\":{");
 int first=1;
 for(int many=0;many<2;many++)for(int centre=0;centre<2;centre++)for(int ex=0;ex<2;ex++)for(int dense=0;dense<2;dense++){
  const unsigned sizes=many?2u:3u;
  for(unsigned variant=0;variant<(many?1u:2u);variant++)
  for(unsigned si=0;si<sizes;si++){
   unsigned na=many?(si?32u:16u):1u,nb=many?(si?64u:16u):counts[si];
   const char*fast=many?(centre?"fast_boxc_many_many":"fast_box_many_many"):variant?(centre?"fast_boxc_one_many_unchecked":"fast_box_one_many_unchecked"):(centre?"fast_boxc_one_many":"fast_box_one_many");
   const char*plain=many?(centre?"plain_boxc_many_many":"plain_box_many_many"):(centre?"plain_boxc_one_many":"plain_box_one_many");
   uint64_t ifast=0,iplain=0;rng_state=UINT32_C(0xb0c5eed)+si;
   for(unsigned c=0;c<CORPORA;c++){
    load_boxes(1,na,nb,dense);
    struct cost a=many?call(fast,na,nb,(u32)ex,0):call(fast,nb,(u32)ex,0,0);
    struct cost b=many?call(plain,na,nb,(u32)ex,0):call(plain,nb,(u32)ex,0,0);
    cf[c]=a.cycles;cp[c]=b.cycles;ifast+=a.instructions;iplain+=b.instructions;
   }
   if(!first)putchar(',');first=0;
   printf("\"%s_%s_%s_%s_%ux%u\":{\"queries\":%u,\"boxes\":%u,\"corpora\":%u,\"fast\":",many?"many_many":variant?"one_many_unchecked":"one_many",centre?"centre":"minmax",ex?"exclusive":"inclusive",dense?"dense":"sparse",na,nb,na,nb,(unsigned)CORPORA);
   print_stats(summarize(cf,CORPORA,ifast));printf(",\"plainC\":");print_stats(summarize(cp,CORPORA,iplain));printf("}");
  }
 }
 printf("}");
}

/* ---- 16x16 cell placement ---- */
#define CELL_MAX 512u
static sh2z_screen_t host_off[CELL_MAX];static sh2z_clip_rect_t host_clip={0,0,384,224};
static sh2z_cell16_xform_t host_xf;
/* kind 0: a block mostly on screen; kind 1: a block straddling the screen edge (about a quarter visible);
 * kind 2: random cells over a wide area (about a tenth visible). */
static void load_cells(unsigned n,int kind) {
 unsigned cols=n>=256?16u:n>=64?8u:4u;
 host_xf.block_w=(int32_t)(cols*16u);host_xf.block_h=(int32_t)((n/cols)*16u);
 for(unsigned i=0;i<n;i++){
  if(kind<2){host_off[i].x=(int16_t)((i%cols)*16u);host_off[i].y=(int16_t)((i/cols)*16u);}
  else{host_off[i].x=(int16_t)(rnd()%1024u);host_off[i].y=(int16_t)(rnd()%640u);}
 }
 host_xf.origin_x=kind==0?60+(int)(rnd()%40u):kind==1?-(int)(host_xf.block_w/2)+(int)(rnd()%8u):-100;
 host_xf.origin_y=kind==0?30+(int)(rnd()%30u):kind==1?-(int)(host_xf.block_h/2):-100;
 for(unsigned i=0;i<n;i++){wl(&cpu,symc("cell_off")+4*i,((u32)(uint16_t)host_off[i].x<<16)|(uint16_t)host_off[i].y);}
 wl(&cpu,symc("cell_xform"),(u32)host_xf.origin_x);wl(&cpu,symc("cell_xform")+4,(u32)host_xf.origin_y);
 wl(&cpu,symc("cell_xform")+8,(u32)host_xf.block_w);wl(&cpu,symc("cell_xform")+12,(u32)host_xf.block_h);
 wl(&cpu,symc("cell_clip"),((u32)(uint16_t)host_clip.min_x<<16)|(uint16_t)host_clip.min_y);
 wl(&cpu,symc("cell_clip")+4,((u32)(uint16_t)host_clip.max_x<<16)|(uint16_t)host_clip.max_y);
}
static uint64_t verify_cell(uint64_t count) {
 uint64_t done=0;
 for(uint64_t it=0;it<count;it++){
  const unsigned n=(unsigned[]){1,2,16,17,64,100,256,512}[rnd()%8u],flip=(unsigned)(rnd()&3u);int kind=(int)(it%3u);
  host_clip=(sh2z_clip_rect_t){0,0,384,224};
  if(it%11==5)host_clip=(sh2z_clip_rect_t){(int16_t)((int)(rnd()%60u)-30),(int16_t)((int)(rnd()%60u)-30),(int16_t)(100+(int)(rnd()%300u)),(int16_t)(60+(int)(rnd()%200u))};
  load_cells(n,kind);
  sh2z_screen_t hp[CELL_MAX];uint16_t hi[CELL_MAX];size_t hv=0;
  if(!(host_xf.flip=flip,sh2z_cell16_place(hp,hi,CELL_MAX,&hv,host_off,n,&host_xf,&host_clip))){fprintf(stderr,"host rejected cell case\n");exit(3);}
  for(int k=0;k<2;k++){
   const char*f=k?"plain_cell_place":"fast_cell_place";
   for(unsigned j=0;j<CELL_MAX;j++){wl(&cpu,symc("cell_pos")+4*j,0xDEADBEEFu);wb(&cpu,symc("cell_idx")+2*j,0xAA);wb(&cpu,symc("cell_idx")+2*j+1,0xAA);}
   struct cost c=call(f,n,flip,0,0);
   if(c.result!=1||rl(&cpu,symc("cell_visible"))!=hv){fprintf(stderr,"%s visible %u want %zu\n",f,rl(&cpu,symc("cell_visible")),hv);exit(3);}
   for(size_t j=0;j<hv;j++){
    u32 p=rl(&cpu,symc("cell_pos")+4*j);u16 id=rw(&cpu,symc("cell_idx")+2*j);
    if((int16_t)(p>>16)!=hp[j].x||(int16_t)p!=hp[j].y||id!=hi[j]){fprintf(stderr,"%s cell %zu differs\n",f,j);exit(3);}
   }
   if(hv<CELL_MAX&&rl(&cpu,symc("cell_pos")+4*hv)!=0xDEADBEEFu){fprintf(stderr,"%s wrote beyond visible\n",f);exit(3);}
  }
  done++;
 }
 return done;
}
static void cost_cell(void) {
 enum {CORPORA=128};
 static unsigned cf[CORPORA],cp[CORPORA];
 printf("\"cell16\":{");int first=1;
 static const unsigned counts[]={16,64,256};static const char*kinds[]={"onscreen","edge","scattered"};
 for(unsigned flip=0;flip<4;flip+=3)for(int kind=0;kind<3;kind++)for(unsigned si=0;si<3;si++){
  uint64_t ifast=0,iplain=0;rng_state=UINT32_C(0xce11)+si;unsigned vis_total=0;
  for(unsigned c=0;c<CORPORA;c++){
   host_clip=(sh2z_clip_rect_t){0,0,384,224};load_cells(counts[si],kind);
   struct cost a=call("fast_cell_place",counts[si],flip,0,0);vis_total+=rl(&cpu,symc("cell_visible"));
   struct cost b=call("plain_cell_place",counts[si],flip,0,0);
   cf[c]=a.cycles;cp[c]=b.cycles;ifast+=a.instructions;iplain+=b.instructions;
  }
  if(!first)putchar(',');first=0;
  printf("\"%s_%s_%u\":{\"cells\":%u,\"meanVisible\":%.1f,\"corpora\":%u,\"fast\":",flip?"flipxy":"noflip",kinds[kind],counts[si],counts[si],(double)vis_total/CORPORA,(unsigned)CORPORA);
  print_stats(summarize(cf,CORPORA,ifast));printf(",\"plainC\":");print_stats(summarize(cp,CORPORA,iplain));printf("}");
 }
 printf("}");
}
int main(int argc,char**argv) {
 if(argc<3){fprintf(stderr,"usage: integer_harness model.elf verify-count\n");return 2;}
 const uint64_t count=strtoull(argv[2],0,10);
 load_elf_symbols(argv[1]);load_elf(argv[1]);
 cpu.model=SHMT_SH2;cpu.MappedMemoryReadByte=rb;cpu.MappedMemoryReadWord=rw;cpu.MappedMemoryReadLong=rl;cpu.MappedMemoryWriteByte=wb;cpu.MappedMemoryWriteWord=ww;cpu.MappedMemoryWriteLong=wl;
 for(unsigned i=0;i<65536;i++)cpu.opcodes[i]=decode(SHMT_SH2,(u16)i);for(unsigned i=0;i<256;i++)cpu.fetchlist[i]=fetch;
 rng_state=UINT32_C(0x2f6e1b93);
 uint64_t m=verify_m68k(count),bx=verify_box(count/20+2000),cl=verify_cell(count/20+2000);
 printf("{\"schema\":1,\"kind\":\"Yabause isolated SH2 interpreter flat-RAM model, not Saturn timing\",\"verified\":{\"m68k\":%"PRIu64",\"box\":%"PRIu64",\"cell16\":%"PRIu64"},",m,bx,cl);
 cost_m68k();printf(",");cost_box();printf(",");cost_cell();
 printf("}\n");
 return 0;
}
