/* SPDX-License-Identifier: GPL-2.0-or-later
 * Executed-instruction checks and cycle model for view-rails-1
 * (sh2z_view_edge_rails_center[_divu], view_experimental.h). Runs the actual
 * big-endian SH-2 object code in Yabause's unmodified interpreter, like
 * harness.c, whose loader, call wrapper and divider model it reuses. Flat
 * RAM, no caches or wait states: MODELLED cycles, not Saturn timing.
 * Copyright (c) 2026 SH2ZAM Contributors. This optional model harness is
 * licensed separately from SH2ZAM's MIT core; see LICENSE.
 *
 * usage: rails_harness model.elf verify-scale > results.json
 * The ELF must contain bridge.o from benchmarks/rails-model and the library.
 * Every output record of rails_divu, rails_portable and (on road strips)
 * loop_divu is compared with the independent oracle in tests/rails_oracle.h
 * before any timing is printed; calls the contract rejects must return 0 and
 * leave both outputs untouched. Any mismatch exits non-zero. */
#define main sh2_model_main
#include "harness.c"
#undef main
#include "../../tests/rails_oracle.h"

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
static struct cost call(const char *name){char tmp[140];snprintf(tmp,sizeof tmp,"_%s",name);return invoke(tmp,0,0,0,0);}
static u32 symc(const char *name){char tmp[140];snprintf(tmp,sizeof tmp,"_%s",name);return sym(tmp);}
static void bad(const char *what,uint64_t it){fprintf(stderr,"rails model mismatch: %s (case %"PRIu64")\n",what,it);exit(3);}
enum {MAXB=96};

/* ---- target memory (big-endian SH-2 layouts) ---- */
static void put_view(const sh2z_view_center_t *w){
 const u32 a=symc("view");const sh2z_view_t *v=&w->view;
 for(unsigned i=0;i<3;i++)for(unsigned j=0;j<3;j++)ww(&cpu,a+2*(3*i+j),(u16)v->r[i][j]);
 ww(&cpu,a+18,0);
 for(unsigned i=0;i<3;i++)wl(&cpu,a+20+4*i,(u32)v->t[i]);
 wl(&cpu,a+32,(u32)v->focal_x);wl(&cpu,a+36,(u32)v->focal_y);
 ww(&cpu,a+40,(u16)v->center_x);ww(&cpu,a+42,(u16)v->center_y);
 wl(&cpu,a+44,(u32)v->near_z);wl(&cpu,a+48,(u32)v->far_z);
 for(unsigned i=0;i<4;i++)ww(&cpu,a+52+2*i,(u16)v->clip[i]);
 wl(&cpu,a+60,(u32)w->center_x16);wl(&cpu,a+64,(u32)w->center_y16);
}
static void put_vertex(u32 a,const sh2z_view_vertex_t *p){
 wl(&cpu,a,(u32)p->x);wl(&cpu,a+4,(u32)p->y);wl(&cpu,a+8,(u32)p->z);
 ww(&cpu,a+12,(u16)p->sx);ww(&cpu,a+14,(u16)p->sy);
 wb(&cpu,a+16,p->outcode);for(unsigned i=0;i<3;i++)wb(&cpu,a+17+i,p->reserved[i]);
}
static sh2z_view_vertex_t get_vertex(u32 a){
 sh2z_view_vertex_t p;
 p.x=(int32_t)rl(&cpu,a);p.y=(int32_t)rl(&cpu,a+4);p.z=(int32_t)rl(&cpu,a+8);
 p.sx=(int16_t)rw(&cpu,a+12);p.sy=(int16_t)rw(&cpu,a+14);
 p.outcode=rb(&cpu,a+16);for(unsigned i=0;i<3;i++)p.reserved[i]=rb(&cpu,a+17+i);
 return p;
}
static void put_strip(const ro_strip_t *s,const sh2z_view_vertex_t ends[4]){
 const u32 e=symc("ends"),b=symc("boundary"),g=symc("args");
 for(unsigned i=0;i<4;i++)put_vertex(e+20*i,&ends[i]);
 for(unsigned j=0;j<s->count;j++)wl(&cpu,b+4*j,(u32)s->b[j]);
 const int32_t args[6]={(int32_t)s->count,s->low[0],s->high[0],s->low[1],s->high[1],s->unit};
 for(unsigned i=0;i<6;i++)wl(&cpu,g+4*i,(u32)args[i]);
}
static void fill_outputs(u8 byte){const u32 l=symc("out_l"),r=symc("out_r");for(u32 i=0;i<20u*MAXB;i++){wb(&cpu,l+i,byte);wb(&cpu,r+i,byte);}}
static void prepare(const sh2z_view_center_t *w){put_view(w);if(call("rails_prepare").result!=1)bad("prepare",0);}

/* ---- verification ---- */
static uint64_t v_calls,v_rejected,v_points,v_copies,v_near,v_loop;
static const char *const entries[3]={"rails_divu","rails_portable","loop_divu"};
static void verify_one(const sh2z_view_center_t *w,const ro_strip_t *s,const sh2z_view_vertex_t ends[4],bool road,uint64_t it){
 static sh2z_view_vertex_t want[2][MAXB];
 const bool ok=ro_rails(want[0],want[1],w,ends,s->b,s->count,s->low[0],s->high[0],s->low[1],s->high[1],0);
 put_strip(s,ends);
 /* The loop is compared on the road renderer's strips only (it assumes
  * their shape and never rejects them except for den > 65535). */
 const unsigned entries_run=road&&ok?3u:2u;
 for(unsigned k=0;k<entries_run;k++){
  fill_outputs(0xA5);
  const struct cost c=call(entries[k]);
  if(c.result!=(u32)ok)bad(k==0?"divu accept":k==1?"portable accept":"loop accept",it);
  const u32 l=symc("out_l"),r=symc("out_r");
  for(unsigned j=0;j<MAXB;j++){
   const sh2z_view_vertex_t gl=get_vertex(l+20*j),gr=get_vertex(r+20*j);
   if(ok&&j<s->count){if(!ro_same(&gl,&want[0][j])||!ro_same(&gr,&want[1][j]))bad(entries[k],it);}
   else for(unsigned i=0;i<20;i++)if(rb(&cpu,l+20*j+i)!=0xA5||rb(&cpu,r+20*j+i)!=0xA5)bad("write outside the call's records",it);
  }
 }
 v_calls++;
 if(!ok){v_rejected++;return;}
 if(entries_run==3)v_loop++;
 for(unsigned rail=0;rail<2;rail++){
  unsigned f,e;ro_split(s->b,s->count,s->low[rail],s->high[rail],0,&f,&e);
  v_copies+=s->count-(e-f);v_points+=e-f;
  for(unsigned j=f;j<e;j++)if(want[rail][j].outcode&SH2Z_OUT_NEAR)v_near++;
 }
}
/* The pointer and handle rules, through rails_divu_ptrs and
 * rails_portable_ptrs (bridge.c), on one valid strip: each null pointer, an
 * unprepared handle, count 0 and above the maximum, overlapping outputs,
 * an output range that wraps, and boundary words inside an output. */
static uint64_t a_cases;
static void untouched(const char *what){const u32 l=symc("out_l"),r=symc("out_r");
 for(u32 i=0;i<20u*MAXB;i++)if(rb(&cpu,l+i)!=0xA5||rb(&cpu,r+i)!=0xA5)bad(what,a_cases);}
static void arg_rules(void){
 ro_state=UINT32_C(0xa11a5e75);
 const sh2z_view_center_t w=ro_road_view();prepare(&w);
 ro_strip_t s;s.count=6;s.unit=256;
 for(unsigned j=0;j<6;j++)s.b[j]=-256+256*(int32_t)j;
 s.low[0]=0;s.high[0]=1024;s.low[1]=-100;s.high[1]=1100;
 sh2z_view_vertex_t ends[4];for(unsigned i=0;i<4;i++)ends[i]=ro_corner(&w,true);
 static sh2z_view_vertex_t want[2][MAXB];
 if(!ro_rails(want[0],want[1],&w,ends,s.b,s.count,s.low[0],s.high[0],s.low[1],s.high[1],0))bad("argument strip",0);
 put_strip(&s,ends);
 const u32 base[8]={symc("out_l"),symc("out_r"),symc("handle"),symc("ends"),symc("ends")+20,symc("ends")+40,symc("ends")+60,symc("boundary")};
 const u32 p=symc("ptrs"),g=symc("args"),check=symc("handle")+108;
 static const char *const names[2]={"rails_divu_ptrs","rails_portable_ptrs"};
 for(unsigned k=0;k<2;k++){
  #define SETP(i,v) wl(&cpu,p+4*(i),(v))
  #define RESET() do{for(unsigned i_=0;i_<8;i_++)SETP(i_,base[i_]);wl(&cpu,g,s.count);fill_outputs(0xA5);}while(0)
  #define EXPECT(r) do{a_cases++;if(call(names[k]).result!=(r))bad(names[k],a_cases);}while(0)
  RESET();EXPECT(1u);
  for(unsigned j=0;j<s.count;j++){
   const sh2z_view_vertex_t gl=get_vertex(base[0]+20*j),gr=get_vertex(base[1]+20*j);
   if(!ro_same(&gl,&want[0][j])||!ro_same(&gr,&want[1][j]))bad("argument baseline",a_cases);}
  for(unsigned i=0;i<8;i++){RESET();SETP(i,0);EXPECT(0u);untouched("null pointer wrote");}
  RESET();const u32 c=rl(&cpu,check);wl(&cpu,check,c^1u);EXPECT(0u);untouched("unprepared handle wrote");wl(&cpu,check,c);
  RESET();wl(&cpu,g,0);EXPECT(1u);untouched("count 0 wrote");
  RESET();wl(&cpu,g,SH2Z_VIEW_RAILS_MAX_COUNT+1u);EXPECT(0u);untouched("count above the maximum wrote");
  RESET();SETP(1,base[0]+20*(s.count-1));EXPECT(0u);untouched("overlap wrote");
  RESET();SETP(1,base[0]+19);EXPECT(0u);untouched("overlap wrote");
  RESET();SETP(0,base[1]+20*(s.count-1));EXPECT(0u);untouched("overlap wrote");
  RESET();SETP(0,UINT32_C(0xFFFFFFFF)-20*s.count+2);EXPECT(0u);untouched("wrapping output wrote");
  RESET();SETP(1,UINT32_C(0xFFFFFFFF)-20*s.count+2);EXPECT(0u);untouched("wrapping output wrote");
  /* Adjacent outputs, then boundary words at the start of out_l. */
  RESET();SETP(1,base[0]+20*s.count);EXPECT(1u);
  for(unsigned j=0;j<s.count;j++){const sh2z_view_vertex_t gl=get_vertex(base[0]+20*j),gr=get_vertex(base[0]+20*(s.count+j));
   if(!ro_same(&gl,&want[0][j])||!ro_same(&gr,&want[1][j]))bad("adjacent outputs",a_cases);}
  RESET();for(unsigned j=0;j<s.count;j++)wl(&cpu,base[0]+4*j,(u32)s.b[j]);SETP(7,base[0]);EXPECT(1u);
  for(unsigned j=0;j<s.count;j++){const sh2z_view_vertex_t gl=get_vertex(base[0]+20*j),gr=get_vertex(base[1]+20*j);
   if(!ro_same(&gl,&want[0][j])||!ro_same(&gr,&want[1][j]))bad("boundary inside out_l",a_cases);}
  #undef SETP
  #undef RESET
  #undef EXPECT
 }
}
static void verify(uint64_t scale){
 ro_state=UINT32_C(0x5eedfa11);
 for(uint64_t view=0;view<scale;view++){
  const bool road=view&1u;
  const sh2z_view_center_t w=(view%4u==1u)?ro_road_view():ro_view();
  prepare(&w);
  for(unsigned q=0;q<16;q++){
   sh2z_view_vertex_t ends[4];for(unsigned i=0;i<4;i++)ends[i]=ro_corner(&w,road);
   if(ro_rnd()%8u==0)ends[1]=ends[0];
   ro_strip_t s;if(road)ro_road_strip(&s);else ro_general_strip(&s);
   verify_one(&w,&s,ends,road,view*16u+q);
  }
 }
}

/* ---- cost ---- */
struct stats {double mean;unsigned min,max;};
static unsigned cyc_of(const char *entry,uint64_t it){const struct cost c=call(entry);if(c.result!=1)bad(entry,it);return c.cycles;}
/* Least-squares line cycles = a + b * points over (points, cycles) pairs. */
struct line {double a,b;};
static struct line fit(const double *x,const double *y,size_t n){
 double sx=0,sy=0,sxx=0,sxy=0;for(size_t i=0;i<n;i++){sx+=x[i];sy+=y[i];sxx+=x[i]*x[i];sxy+=x[i]*y[i];}
 const double d=(double)n*sxx-sx*sx,b=((double)n*sxy-sx*sy)/d;return (struct line){(sy-b*sx)/(double)n,b};
}
/* One strip with n stepped points on each rail (n + 2 boundaries, unit 256),
 * both rails over the same span or not. */
static void sized_strip(ro_strip_t *s,unsigned n,bool shared){
 const int32_t unit=256,low=ro_range(-4000,4000)&~255;
 s->unit=unit;s->count=0;s->b[s->count++]=low;
 for(unsigned k=0;k<=n;k++)s->b[s->count++]=low+(int32_t)(k+1)*unit;
 s->low[0]=low;s->high[0]=s->b[s->count-1];
 if(shared){s->low[1]=s->low[0];s->high[1]=s->high[0];}
 else{s->low[1]=low+ro_range(1,unit-1);s->high[1]=s->b[s->count-1]-ro_range(0,unit-1);}
}
static void cost(void){
 ro_state=UINT32_C(0xc0571234);
 const sh2z_view_center_t w=ro_road_view();
 prepare(&w);
 /* 1. Per call and per point: n = 0 .. 63 stepped points per rail. */
 enum {SIZES=64,REPEAT=8};
 static double px[2][SIZES*REPEAT],py[3][2][SIZES*REPEAT];
 size_t m=0;
 for(unsigned n=0;n<SIZES;n++)for(unsigned r=0;r<REPEAT;r++,m++){
  for(unsigned shared=0;shared<2;shared++){
   ro_strip_t s;sized_strip(&s,n,shared);
   if(!shared&&n==0){s.low[1]=s.low[0];s.high[1]=s.high[0];}
   sh2z_view_vertex_t ends[4];for(unsigned i=0;i<4;i++){do ends[i]=ro_corner(&w,true);while(ends[i].outcode&SH2Z_OUT_NEAR);}
   static sh2z_view_vertex_t want[2][MAXB];
   if(!ro_rails(want[0],want[1],&w,ends,s.b,s.count,s.low[0],s.high[0],s.low[1],s.high[1],0))bad("sized strip",m);
   put_strip(&s,ends);
   unsigned points=0;for(unsigned j=0;j<s.count;j++)for(unsigned rail=0;rail<2;rail++)points+=s.b[j]>s.low[rail]&&s.b[j]<s.high[rail];
   px[shared][m]=points;
   for(unsigned k=0;k<3;k++)py[k][shared][m]=cyc_of(entries[k],m);
  }
 }
 printf("\"perCallAndPoint\":{\"camera\":\"road: focal 200 px, 320 x 224 clip, centre (160, 112 + 1/65536)\",\"pointsPerRail\":\"0 .. 63\",\"repeats\":%u,\"fit\":\"cycles per quad = perQuad + perPoint * stepped points (both rails), least squares\",",REPEAT);
 static const char *const names[3]={"railsDivu","railsPortable","loopDivu"};
 for(unsigned shared=0;shared<2;shared++){
  printf("%s\"%s\":{",shared?",":"",shared?"sharedSpan":"separateSpans");
  for(unsigned k=0;k<3;k++){
   const struct line f=fit(px[shared],py[k][shared],m);
   printf("%s\"%s\":{\"perQuad\":%.1f,\"perRail\":%.1f,\"perPoint\":%.2f}",k?",":"",names[k],f.a,f.a/2,f.b);
  }
  printf("}");
 }
 printf("},");
 /* 2. The road renderer's strips: boundaries per quad 2 .. 65. */
 enum {QUADS=4000};
 uint64_t tot[3]={0,0,0},points=0,bounds=0,stepped_rails=0;
 unsigned hist_max=0;
 for(unsigned q=0;q<QUADS;q++){
  ro_strip_t s;
  sh2z_view_vertex_t ends[4];
  static sh2z_view_vertex_t want[2][MAXB];
  do{ro_road_strip(&s);for(unsigned i=0;i<4;i++)ends[i]=ro_corner(&w,true);}
  while(!ro_rails(want[0],want[1],&w,ends,s.b,s.count,s.low[0],s.high[0],s.low[1],s.high[1],0));
  put_strip(&s,ends);
  for(unsigned k=0;k<3;k++)tot[k]+=cyc_of(entries[k],q);
  const u32 l=symc("out_l"),r=symc("out_r");
  for(unsigned j=0;j<s.count;j++){const sh2z_view_vertex_t gl=get_vertex(l+20*j),gr=get_vertex(r+20*j);
   if(!ro_same(&gl,&want[0][j])||!ro_same(&gr,&want[1][j]))bad("road cost strip",q);}
  bounds+=s.count;if(s.count>hist_max)hist_max=s.count;
  for(unsigned rail=0;rail<2;rail++){unsigned n=0;for(unsigned j=0;j<s.count;j++)n+=s.b[j]>s.low[rail]&&s.b[j]<s.high[rail];points+=n;stepped_rails+=n>0;}
 }
 printf("\"roadStrips\":{\"quads\":%u,\"meanBoundaries\":%.3f,\"maxBoundaries\":%u,\"meanPointsPerRail\":%.3f,\"railsWithPoints\":%"PRIu64",\"meanCyclesPerQuad\":{",
  QUADS,(double)bounds/QUADS,hist_max,(double)points/(2.0*QUADS),stepped_rails);
 for(unsigned k=0;k<3;k++)printf("%s\"%s\":%.1f",k?",":"",names[k],(double)tot[k]/QUADS);
 printf("},\"speedupOverLoop\":{\"railsDivu\":%.2f,\"railsPortable\":%.2f}}",(double)tot[2]/(double)tot[0],(double)tot[2]/(double)tot[1]);
}

int main(int argc,char**argv){
 if(argc<3){fprintf(stderr,"usage: rails_harness model.elf verify-scale\n");return 2;}
 const uint64_t scale=strtoull(argv[2],0,10);
 load_elf_symbols(argv[1]);load_elf(argv[1]);
 cpu.model=SHMT_SH2;cpu.MappedMemoryReadByte=rb;cpu.MappedMemoryReadWord=rw;cpu.MappedMemoryReadLong=rl;cpu.MappedMemoryWriteByte=wb;cpu.MappedMemoryWriteWord=ww;cpu.MappedMemoryWriteLong=wl;
 for(unsigned i=0;i<65536;i++)cpu.opcodes[i]=decode(SHMT_SH2,(u16)i);for(unsigned i=0;i<256;i++)cpu.fetchlist[i]=fetch;
 if(rl(&cpu,symc("rails_model_boundaries"))!=MAXB)bad("bridge buffer size",0);
 divu_reset();
 verify(scale);
 arg_rules(); verify(scale);
 const uint64_t divisions=divu_divisions;
 printf("{\"schema\":1,\"kind\":\"Yabause isolated SH2 interpreter flat-RAM model, not Saturn timing\",");
 printf("\"verified\":{\"calls\":%"PRIu64",\"rejected\":%"PRIu64",\"steppedPoints\":%"PRIu64",\"nearPoints\":%"PRIu64",\"copiedEnds\":%"PRIu64",\"loopCompared\":%"PRIu64",\"dividerDivisions\":%"PRIu64",\"entries\":\"rails_divu, rails_portable (every call) and loop_divu (road strips) against tests/rails_oracle.h, every record byte; rejected calls return 0 and write nothing\"},",
  v_calls,v_rejected,v_points,v_near,v_copies,v_loop,divisions);
 printf("\"argumentRules\":{\"cases\":%"PRIu64",\"checks\":\"each of 8 null pointers, unprepared handle, count 0 and maximum + 1, overlapping and wrapping outputs (nothing written); adjacent outputs and boundary inside out_l (every record against the oracle); rails_divu_ptrs and rails_portable_ptrs\"},",a_cases);
 cost();
 printf("}\n");
 return 0;
}
