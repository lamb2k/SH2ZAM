/* SPDX-License-Identifier: GPL-2.0-or-later
 * Memory placement model for the SH-2 instruction harness. Included by
 * harness.c. Copyright (c) 2026 SH2ZAM Contributors. Harness code, licensed
 * like harness.c; see this directory's LICENSE.
 *
 * The interpreter's cycle counter (cpu.cycles, "CPU-only") is never touched.
 * When MODEL_PLACEMENT_LOG names a file, each tagged kernel call is also run
 * through an SH7604 cache model once per placement, and the stall cycles that
 * placement would add are written to that file, one JSON line per call.
 * Without the variable nothing here runs and every capture prints exactly
 * what it printed before.
 *
 * Placement: each 16-byte line of the image belongs to a class (code, tables,
 * input, output, stack) and each placement puts each class in a region (high
 * work RAM, low work RAM, cartridge, VDP1 VRAM, VDP2). The cache sees the
 * benchmark's own addresses, so set conflicts follow the image's layout; the
 * region decides only what a miss, a cache-through access or a write costs.
 *
 * Cache: SH7604, 4 KB, 4 ways, 64 sets of 16-byte lines, true LRU, shared by
 * instruction fetches and data, write-through with no write allocate. A read
 * or fetch that misses fills the whole line and stalls for the region's fill
 * cost. VDP1 and VDP2 are cache-through: every read stalls for the region's
 * access cost. Every write goes to the bus through a one-entry write buffer:
 * the CPU goes on, and the next bus access (fill, cache-through read or
 * write) first waits until that write has finished. Region costs are from
 * hardware notes of a Saturn port, not measured by this project; see
 * ../placement-model/README.md. */
#define PLACE_SETS 64u
#define PLACE_WAYS 4u
#define PLACE_LINES (RAM_SIZE/16u)
#define PLACE_TAGS 96u

enum {PC_CODE,PC_TABLES,PC_INPUT,PC_OUTPUT,PC_STACK,PC_N};
static const char *const place_class_names[PC_N]={"code","tables","input","output","stack"};

/* fill: cycles to fill one 16-byte line (0 = cache-through region).
 * read/write: cycles per access of 1, 2 and 4 bytes. */
struct place_region {const char *name;unsigned fill,read[3],write[3];};
enum {PR_HWRAM,PR_LWRAM,PR_CART6,PR_CART10,PR_VDP1,PR_VDP2,PR_N};
static const struct place_region place_regions[PR_N]={
 {"hwram",7,{0,0,0},{4,4,4}},
 {"lwram",56,{0,0,0},{7,7,14}},
 {"cart6",48,{0,0,0},{6,6,12}},
 {"cart10",80,{0,0,0},{10,10,20}},
 {"vdp1",0,{45,45,45},{45,45,45}},
 {"vdp2",0,{40,40,40},{40,40,40}},
};
struct place_placement {const char *name;unsigned char region[PC_N];};
static const struct place_placement place_set[]={
 {"best",{PR_HWRAM,PR_HWRAM,PR_HWRAM,PR_HWRAM,PR_HWRAM}},
 {"typical",{PR_HWRAM,PR_HWRAM,PR_LWRAM,PR_LWRAM,PR_HWRAM}},
 {"worst",{PR_CART10,PR_CART10,PR_CART10,PR_LWRAM,PR_HWRAM}},
 {"codeLow",{PR_LWRAM,PR_HWRAM,PR_HWRAM,PR_HWRAM,PR_HWRAM}},
 {"cartTables",{PR_HWRAM,PR_CART6,PR_LWRAM,PR_LWRAM,PR_HWRAM}},
 {"outVdp1",{PR_HWRAM,PR_HWRAM,PR_LWRAM,PR_VDP1,PR_HWRAM}},
 {"outVdp2",{PR_HWRAM,PR_HWRAM,PR_LWRAM,PR_VDP2,PR_HWRAM}},
};
#define PLACE_N (sizeof place_set/sizeof place_set[0])

struct place_cache {u32 line[PLACE_SETS][PLACE_WAYS];uint64_t stamp[PLACE_SETS][PLACE_WAYS],tick;};
/* Per call: stall = fill[code..stack] + through + wait. */
enum {PS_STALL,PS_FILL,PS_THROUGH=PS_FILL+PC_N,PS_WAIT,PS_N};
struct place_run {struct place_cache *cache;uint64_t bus_free,s[PS_N];};
struct place_decl {const char *symbol;unsigned char cls;};
struct place_tag {char label[80];struct place_cache warm[PLACE_N];};

static FILE *place_log;
static char place_pending[80];
static const struct place_decl *place_pending_decl;
static unsigned char place_base_map[PLACE_LINES],place_map[PLACE_LINES];
static const struct place_decl *place_map_decl;
static int place_map_ready,place_map_valid;
static struct place_tag place_tags[PLACE_TAGS];static unsigned place_ntags;
static struct place_cache place_cold[PLACE_N];
static struct place_run place_runs[PLACE_N][2];
static u32 place_image_end;

static int place_lookup(struct place_cache *c,u32 addr,int allocate) {
 const u32 tag=(addr>>4)+1u,set=(addr>>4)&(PLACE_SETS-1u);
 unsigned victim=0;
 for(unsigned w=0;w<PLACE_WAYS;w++){
  if(c->line[set][w]==tag){c->stamp[set][w]=++c->tick;return 1;}
  if(c->stamp[set][w]<c->stamp[set][victim])victim=w;
 }
 if(allocate){c->line[set][victim]=tag;c->stamp[set][victim]=++c->tick;}
 return 0;
}
/* kind: 0 instruction fetch, 1 data read, 2 data write; size 1, 2 or 4. */
static void place_one(struct place_run *r,const struct place_region *g,unsigned cls,u32 addr,int kind,unsigned size) {
 const unsigned z=size==4?2u:size==2?1u:0u;
 uint64_t now=(uint64_t)cpu.cycles+r->s[PS_STALL];
 if(kind==2){
  if(g->fill)(void)place_lookup(r->cache,addr,0);
  const uint64_t wait=r->bus_free>now?r->bus_free-now:0;
  r->s[PS_STALL]+=wait;r->s[PS_WAIT]+=wait;
  r->bus_free=now+wait+g->write[z];
  return;
 }
 unsigned cost;int slot;
 if(g->fill){if(place_lookup(r->cache,addr,1))return;cost=g->fill;slot=PS_FILL+(int)cls;}
 else{cost=g->read[z];slot=PS_THROUGH;}
 const uint64_t wait=r->bus_free>now?r->bus_free-now:0;
 r->s[PS_STALL]+=wait+cost;r->s[PS_WAIT]+=wait;r->s[slot]+=cost;
 r->bus_free=now+wait+cost;
}
static void place_access(u32 addr,int kind,unsigned size) {
 const unsigned cls=kind==0?(unsigned)PC_CODE:place_map[(addr-RAM_BASE)>>4];
 for(unsigned p=0;p<PLACE_N;p++){
  const struct place_region *g=&place_regions[place_set[p].region[cls]];
  if(kind==0&&!g->fill)fail("Placement puts code in a cache-through region",addr);
  place_one(&place_runs[p][0],g,cls,addr,kind,size);
  place_one(&place_runs[p][1],g,cls,addr,kind,size);
 }
}

/* Known sequences through the cache and bus rules, checked before any kernel
 * runs; the stall each must produce is worked out in its comment. One access
 * per cycle; op = kind << 28 | size << 24 | offset (reads are longs). */
static uint64_t place_check_run(const unsigned *ops,unsigned n,unsigned region) {
 static struct place_cache c;struct place_run r;
 memset(&c,0,sizeof c);memset(&r,0,sizeof r);r.cache=&c;cpu.cycles=0;
 for(unsigned i=0;i<n;i++){const unsigned op=ops[i];
  place_one(&r,&place_regions[region],PC_INPUT,RAM_BASE+(op&0xFFFFFu),(int)(op>>28),op>>28==1?4u:(op>>24)&7u);cpu.cycles++;}
 cpu.cycles=0;return r.s[PS_STALL];
}
static void place_selftest(void) {
 static unsigned ops[1100];unsigned n=0;
 /* 1. 4 KB read twice: 256 cold fills, then all hits. */
 for(unsigned pass=0;pass<2;pass++)for(unsigned a=0;a<4096;a+=16)ops[n++]=(1u<<28)|a;
 if(place_check_run(ops,n,PR_HWRAM)!=256u*7u||place_check_run(ops,n,PR_LWRAM)!=256u*56u)fail("Placement self-test: 4 KB working set",0);
 /* 2. Five lines in one set (1 KB apart), cycled twice: LRU misses all ten. */
 n=0;for(unsigned pass=0;pass<2;pass++)for(unsigned w=0;w<5;w++)ops[n++]=(1u<<28)|(w*1024u);
 if(place_check_run(ops,n,PR_HWRAM)!=10u*7u)fail("Placement self-test: LRU",0);
 /* 3. Four lines in one set fit: 4 misses, then 4 hits. */
 n=0;for(unsigned pass=0;pass<2;pass++)for(unsigned w=0;w<4;w++)ops[n++]=(1u<<28)|(w*1024u);
 if(place_check_run(ops,n,PR_HWRAM)!=4u*7u)fail("Placement self-test: four ways",0);
 /* 4. No write allocate: a long write to low RAM (14 bus cycles), then a read
  * of that line a cycle later waits 13 for the write and fills for 56. */
 n=0;ops[n++]=(2u<<28)|(4u<<24)|64u;ops[n++]=(1u<<28)|64u;
 if(place_check_run(ops,n,PR_LWRAM)!=13u+56u)fail("Placement self-test: no write allocate",0);
 /* 5. Write buffer: three long writes to low RAM a cycle apart; the second
  * and the third each wait 13. */
 n=0;for(unsigned k=0;k<3;k++)ops[n++]=(2u<<28)|(4u<<24)|(256u+4u*k);
 if(place_check_run(ops,n,PR_LWRAM)!=26u)fail("Placement self-test: write buffer",0);
 /* 6. VDP1 is cache-through: two reads of one address cost 45 each. */
 n=0;ops[n++]=(1u<<28)|512u;ops[n++]=(1u<<28)|512u;
 if(place_check_run(ops,n,PR_VDP1)!=90u)fail("Placement self-test: cache-through",0);
 /* 7. A write hit keeps the line (write-through): read, write, read costs
  * one fill, and neither later access waits. */
 n=0;ops[n++]=(1u<<28)|768u;ops[n++]=(2u<<28)|(2u<<24)|768u;ops[n++]=(1u<<28)|768u;
 if(place_check_run(ops,n,PR_HWRAM)!=7u)fail("Placement self-test: write hit",0);
 /* 8. LRU, not FIFO: A B C D A E A in one set misses A B C D E only
  * (E evicts B, the least recently used; FIFO would evict A). */
 n=0;{static const unsigned seq[7]={0,1,2,3,0,4,0};for(unsigned k=0;k<7;k++)ops[n++]=(1u<<28)|(seq[k]*1024u);}
 if(place_check_run(ops,n,PR_HWRAM)!=5u*7u)fail("Placement self-test: LRU order",0);
}

/* A symbol's bytes run to the next higher symbol, within the image. */
static void place_symbol_range(const char *name,u32 *start,u32 *end) {
 const u32 a=sym(name);u32 e=place_image_end;
 for(size_t i=0;i<nsymbols;i++)if(symbols[i].address>a&&symbols[i].address<e)e=symbols[i].address;
 if(a<RAM_BASE||a>=e)fail("Placement symbol outside the image",a);
 *start=a;*end=e;
}
static void place_build_map(const struct place_decl *d) {
 memcpy(place_map,place_base_map,sizeof place_map);
 for(;d&&d->symbol;d++){u32 a,e;place_symbol_range(d->symbol,&a,&e);
  for(u32 l=(a-RAM_BASE)>>4;l<=(e-1u-RAM_BASE)>>4;l++)place_map[l]=d->cls;}
}

/* Section flags give the default class of each line: executable sections are
 * code (literal pools included), other read-only sections tables, writable
 * sections input; above the image is the stack. */
static void place_init(const char *elf_path) {
 const char *path=getenv("MODEL_PLACEMENT_LOG");
 if(!path||!*path)return;
 FILE*f=fopen(elf_path,"rb");if(!f){perror(elf_path);exit(2);}
 fseek(f,0,SEEK_END);long length=ftell(f);rewind(f);
 unsigned char*b=malloc((size_t)length);if(!b||fread(b,1,(size_t)length,f)!=(size_t)length)exit(2);fclose(f);
 const u32 shoff=be32(b+32);const u16 shentsize=be16(b+46),shnum=be16(b+48),shstrndx=be16(b+50);
 if(!shoff||shentsize<40||(uint64_t)shoff+(uint64_t)shentsize*shnum>(uint64_t)length)fail("Placement: no section headers",0);
 const unsigned char*names=b+be32(b+shoff+(size_t)shstrndx*shentsize+16);
 place_log=fopen(path,"w");if(!place_log){perror(path);exit(2);}
 memset(place_base_map,PC_INPUT,sizeof place_base_map);
 u32 lo=RAM_BASE+RAM_SIZE;place_image_end=RAM_BASE;
 uint64_t bytes[PC_N]={0};
 fprintf(place_log,"{\"placementModel\":1,\"sections\":[");
 int first=1;
 for(unsigned i=0;i<shnum;i++){
  const unsigned char*sh=b+shoff+(size_t)i*shentsize;
  const u32 flags=be32(sh+8),size=be32(sh+20);u32 addr=be32(sh+12);
  if(!(flags&2u)||!size)continue; /* SHF_ALLOC */
  if(addr>=UINT32_C(0x26000000))addr-=UINT32_C(0x20000000);
  if(addr<RAM_BASE||addr-RAM_BASE>=RAM_SIZE||size>RAM_BASE+RAM_SIZE-addr)continue;
  const unsigned cls=(flags&4u)?PC_CODE:(flags&1u)?PC_INPUT:PC_TABLES;
  for(u32 l=(addr-RAM_BASE)>>4;l<=(addr+size-1u-RAM_BASE)>>4;l++)place_base_map[l]=(unsigned char)cls;
  if(addr<lo)lo=addr;if(addr+size>place_image_end)place_image_end=addr+size;
  bytes[cls]+=size;
  fprintf(place_log,"%s{\"name\":\"%s\",\"address\":%u,\"bytes\":%u,\"class\":\"%s\"}",first?"":",",(const char*)names+be32(sh),addr,size,place_class_names[cls]);first=0;
 }
 free(b);
 if(place_image_end<=lo)fail("Placement: no allocated sections in RAM",0);
 for(u32 l=(place_image_end+15u-RAM_BASE)>>4;l<PLACE_LINES;l++)place_base_map[l]=PC_STACK;
 fprintf(place_log,"],\"imageStart\":%u,\"imageEnd\":%u,\"classes\":[",lo,place_image_end);
 for(unsigned c=0;c<PC_N;c++)fprintf(place_log,"%s\"%s\"",c?",":"",place_class_names[c]);
 fprintf(place_log,"],\"regions\":[");
 for(unsigned r=0;r<PR_N;r++){const struct place_region *g=&place_regions[r];
  fprintf(place_log,"%s{\"name\":\"%s\",\"lineFill\":%u,\"read\":[%u,%u,%u],\"write\":[%u,%u,%u]}",r?",":"",g->name,g->fill,
   g->read[0],g->read[1],g->read[2],g->write[0],g->write[1],g->write[2]);}
 fprintf(place_log,"],\"placements\":[");
 for(unsigned p=0;p<PLACE_N;p++){fprintf(place_log,"%s{\"name\":\"%s\"",p?",":"",place_set[p].name);
  for(unsigned c=0;c<PC_N;c++)fprintf(place_log,",\"%s\":\"%s\"",place_class_names[c],place_regions[place_set[p].region[c]].name);
  fputc('}',place_log);}
 place_selftest();
 fprintf(place_log,"],\"selfTest\":\"passed\",\"fields\":[\"stall\",\"fillCode\",\"fillTables\",\"fillInput\",\"fillOutput\",\"fillStack\",\"through\",\"writeWait\"]}\n");
 place_map_ready=1;
}

/* Tag the next invoke: its stalls are logged under label, with the decl's
 * symbols (outputs, tables) overriding the section classes. */
static void place_tag(const char *label,const struct place_decl *decl) {
 if(!place_map_ready)return;
 if(strlen(label)>=sizeof place_pending)fail("Placement label length",0);
 strcpy(place_pending,label);place_pending_decl=decl;
}
static void place_begin(void) {
 struct place_tag *t=NULL;
 for(unsigned i=0;i<place_ntags;i++)if(!strcmp(place_tags[i].label,place_pending)){t=&place_tags[i];break;}
 if(!t){if(place_ntags>=PLACE_TAGS||strlen(place_pending)>=sizeof t->label)fail("Placement tags",place_ntags);
  t=&place_tags[place_ntags++];memset(t,0,sizeof *t);strcpy(t->label,place_pending);
  /* Log the classes this tag's declaration resolves to. */
  fprintf(place_log,"{\"declare\":\"%s\",\"symbols\":[",t->label);
  int first=1;
  for(const struct place_decl *d=place_pending_decl;d&&d->symbol;d++){u32 a,e;place_symbol_range(d->symbol,&a,&e);
   fprintf(place_log,"%s{\"symbol\":\"%s\",\"class\":\"%s\",\"address\":%u,\"bytes\":%u}",first?"":",",d->symbol,place_class_names[d->cls],a,e-a);first=0;}
  fprintf(place_log,"]}\n");}
 if(!place_map_valid||place_map_decl!=place_pending_decl){place_build_map(place_pending_decl);place_map_decl=place_pending_decl;place_map_valid=1;}
 for(unsigned p=0;p<PLACE_N;p++){
  memset(&place_cold[p],0,sizeof place_cold[p]);
  memset(place_runs[p],0,sizeof place_runs[p]);
  place_runs[p][0].cache=&t->warm[p];place_runs[p][1].cache=&place_cold[p];
 }
 place_active=1;
}
static void place_end(u32 cycles,uint64_t instructions) {
 place_active=0;
 fprintf(place_log,"{\"tag\":\"%s\",\"cycles\":%u,\"instructions\":%"PRIu64",\"placed\":[",place_pending,cycles,instructions);
 for(unsigned p=0;p<PLACE_N;p++){
  fputs(p?",[":"[",place_log);
  for(unsigned m=0;m<2;m++){
   const struct place_run *r=&place_runs[p][m];
   uint64_t sum=0;for(int k=PS_FILL;k<PS_N;k++)sum+=r->s[k];
   if(sum!=r->s[PS_STALL])fail("Placement stall accounting",p);
   fputs(m?",[":"[",place_log);
   for(int k=0;k<PS_N;k++)fprintf(place_log,"%s%"PRIu64,k?",":"",r->s[k]);
   fputc(']',place_log);
  }
  fputc(']',place_log);
 }
 fputs("]}\n",place_log);
 place_pending[0]=0;place_pending_decl=NULL;
}
