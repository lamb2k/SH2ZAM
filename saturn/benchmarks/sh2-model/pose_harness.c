/* SPDX-License-Identifier: GPL-2.0-or-later
 * Executed-instruction checks and cycle model for pose-1 (sh2z_pose_from_f32,
 * view_experimental.h). Runs the actual big-endian SH-2 object code in
 * Yabause's unmodified interpreter, like harness.c, whose loader and call
 * wrapper it reuses. Flat RAM, no caches or wait states: MODELLED cycles, not
 * Saturn timing. Copyright (c) 2026 SH2ZAM Contributors. This optional model
 * harness is licensed separately from SH2ZAM's MIT core; see LICENSE.
 *
 * usage: pose_harness model.elf verify-count > results.json
 * The ELF must contain bridge.o from benchmarks/pose-model and the library.
 * Every output of fast_pose (with SR.S clear and set) and of plain_pose, and
 * every success flag, is checked against the binary32 oracle in
 * tests/pose_oracle.h before any timing is printed; a failed call must leave
 * its output untouched. */
#define main sh2_model_main
#include "harness.c"
#undef main
#include "../../tests/pose_oracle.h"

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
static struct cost call_sr(const char *name,u32 a,u32 sr){char tmp[140];snprintf(tmp,sizeof tmp,"_%s",name);return invoke_sr(tmp,a,0,0,0,sr);}
static void bad(const char *what,uint64_t it){fprintf(stderr,"pose model mismatch: %s (case %"PRIu64")\n",what,it);exit(3);}

/* ---- target memory ---- */
static void put_case(const po_case_t *c,const sh2z_view_t *v) {
 const u32 r=symc("rotation"),b=symc("basis"),s=symc("scale"),p=symc("position"),e=symc("eye"),w=symc("base_words");
 for(unsigned i=0;i<3;i++){
  wl(&cpu,s+4*i,po_bits(c->scale[i]));wl(&cpu,p+4*i,po_bits(c->position[i]));wl(&cpu,e+4*i,po_bits(c->eye[i]));
  for(unsigned j=0;j<3;j++){wl(&cpu,r+12*i+4*j,(u32)c->rotation[i][j]);wl(&cpu,b+12*i+4*j,po_bits(c->basis[i][j]));}
 }
 const int32_t words[10]={v->focal_x,v->focal_y,v->center_x,v->center_y,v->near_z,v->far_z,v->clip[0],v->clip[1],v->clip[2],v->clip[3]};
 for(unsigned i=0;i<10;i++)wl(&cpu,w+4*i,(u32)words[i]);
 invoke("_set_base",0,0,0,0);
}
static void get_pose(unsigned which,sh2z_pose_t *p) {
 invoke("_export_pose",which,0,0,0);
 const u32 w=symc("words");int32_t v[37];
 for(unsigned i=0;i<37;i++){u32 x=rl(&cpu,w+4*i);memcpy(&v[i],&x,4);}
 unsigned n=0;memset(p,0,sizeof *p);
 for(unsigned i=0;i<3;i++)for(unsigned j=0;j<3;j++)p->view.r[i][j]=(int16_t)v[n++];
 for(unsigned i=0;i<3;i++)p->view.t[i]=v[n++];
 p->view.focal_x=v[n++];p->view.focal_y=v[n++];p->view.center_x=(int16_t)v[n++];p->view.center_y=(int16_t)v[n++];
 p->view.near_z=v[n++];p->view.far_z=v[n++];
 for(unsigned i=0;i<4;i++)p->view.clip[i]=(int16_t)v[n++];
 for(unsigned i=0;i<3;i++)p->eye[i]=v[n++];
 for(unsigned i=0;i<3;i++)for(unsigned j=0;j<3;j++)p->basis[i][j]=(int16_t)v[n++];
 p->eye_error=(uint16_t)v[n++];p->octant=(uint8_t)v[n++];p->cull_valid=v[n++]!=0;
}
static bool untouched(const char *name) {
 const u32 a=symc(name);
 for(unsigned i=0;i<sizeof(sh2z_pose_t);i++)if(rb(&cpu,a+i)!=0xA5)return false;
 return true;
}

/* ---- verification ---- */
static const uint32_t specials[]={
 0x00000000u,0x80000000u,0x00000001u,0x80000001u,0x007FFFFFu,0x00800000u,0x7F800000u,0xFF800000u,0x7FC00000u,
 0xFFC00000u,0x7F800001u,0x7F7FFFFFu,0xFF7FFFFFu,0x3F800054u,0x3F800055u,0xBF800054u,0x3D800000u,0x3D7FFFFFu,
 0x3F800000u,0x3F800001u,0x45800000u,0xC5800000u,0x4B800000u,0x46FFFE00u};
static uint64_t v_cases,v_ok,v_cull_off,v_fields,v_alias;
static void verify_case(const po_case_t *c,const sh2z_view_t *v,uint64_t it) {
 sh2z_pose_t want,got;
 const bool ok=po_pose(&want,v,(const int32_t (*)[3])c->rotation,(const float (*)[3])c->basis,c->scale,c->position,c->eye,c->flags,PO_EXACT);
 put_case(c,v);
 static const char *const entries[3]={"fast_pose","fast_pose","plain_pose"};
 for(unsigned k=0;k<3;k++){
  invoke("_poison",0,0,0,0);
  const struct cost r=call_sr(entries[k],c->flags,k==1?2u:0u);
  if((r.result!=0)!=ok)bad(k==0?"fast success":k==1?"fast (SR.S set) success":"plain success",it);
  if(k==1 && r.mac_ops)bad("MAC.L with SR.S set",it);
  if(k==0 && ok && r.mac_ops!=63u)bad("MAC.L count",it);
  if(ok){get_pose(k==2,&got);if(!po_same(&got,&want))bad(k==0?"fast fields":k==1?"fast (SR.S set) fields":"plain fields",it);v_fields+=37;}
  else if(!untouched(k==2?"pose_plain":"pose_fast"))bad("output written on failure",it);
 }
 if(ok){ /* the aliasing the header allows, on both paths */
  static const char *const aliases[2]={"alias_base","alias_inputs"};
  for(unsigned k=0;k<4;k++){
   const struct cost r=call_sr(aliases[k&1u],c->flags,k>=2u?2u:0u);
   if(r.result!=1)bad("aliasing success",it);
   get_pose(2,&got);if(!po_same(&got,&want))bad(k&1u?"inputs aliasing out":"base aliasing out",it);
  }
  v_alias++;
 }
 v_cases++;v_ok+=ok;v_cull_off+=ok&&!want.cull_valid;
}
static void verify(uint64_t count) {
 sh2z_view_t v;po_base(&v);
 po_case_t c;
 for(uint64_t it=0;it<count;it++){
  po_random_case(&c);
  sh2z_view_t base=v;
  switch(it%16u){
  case 0:case 1:case 2:{ /* a special float in one slot */
   const unsigned slot=po_rnd()%18u;float *f=slot<9u?&c.basis[slot/3u][slot%3u]:slot<12u?&c.scale[slot-9u]:slot<15u?&c.position[slot-12u]:&c.eye[slot-15u];
   *f=po_float(specials[po_rnd()%(sizeof specials/sizeof specials[0])]);break;}
  case 3:c.rotation[po_rnd()%3u][po_rnd()%3u]=(po_rnd()&1u?1:-1)*(int32_t)(1100000000u+po_rnd()%2u);break;
  case 4:for(unsigned a=0;a<3;a++){c.scale[a]=1.0f;for(unsigned k=0;k<3;k++)c.basis[a][k]=po_rnd()&1u?1.00001f:-1.00001f;}
   for(unsigned i=0;i<3;i++)for(unsigned k=0;k<3;k++)c.rotation[i][k]=(po_rnd()&1u?1:-1)*(int32_t)(1000000000u+po_rnd()%100000001u);break;
  case 5:for(unsigned k=0;k<3;k++){c.position[k]=(float)po_uniform(-3e5,3e5);c.eye[k]=(float)((double)c.position[k]-po_uniform(-4100,4100));}break;
  case 6:if(po_rnd()%2u)base.focal_x=0;else base.far_z=base.near_z;break;
  case 7:c.flags=2u;break;
  default:break;
  }
  verify_case(&c,&base,it);
 }
}

/* ---- the internal routines, one value at a time ---- */
static uint64_t r_delta,r_inverse,r_element;
static uint32_t rnd_float_bits(void) {
 /* Mostly finite values near the game's range, some anywhere. */
 switch(po_rnd()%8u){
 case 0:return (uint32_t)po_rnd();
 case 1:return specials[po_rnd()%(sizeof specials/sizeof specials[0])];
 default:{const uint32_t e=96u+(uint32_t)(po_rnd()%64u);return (uint32_t)(po_rnd()&0x807FFFFFu)|e<<23;}
 }
}
static void verify_routines(uint64_t count) {
 for(uint64_t it=0;it<count;it++){
  /* difference: a random pair, or b within a few ulps or 4096 of a */
  uint32_t a=rnd_float_bits(),b;
  switch(po_rnd()%4u){
  case 0:b=rnd_float_bits();break;
  case 1:b=a+(uint32_t)(po_rnd()%64u)-32u;break;
  case 2:b=po_bits((float)((double)po_float(a)-po_uniform(-4100,4100)));break;
  default:b=po_bits(-po_float(a)+(float)po_uniform(-8,8));break;
  }
  int32_t want=INT32_MIN;
  {const float x=po_float(a),y=po_float(b);int32_t d;
   if(po_finite(x)&&po_finite(y)&&po_fixed(x-y,65536.0f,4096.0f,&d))want=d;}
  if((int32_t)invoke("_test_delta",a,b,0,0).result!=want)bad("delta",it);
  r_delta++;
  /* reciprocal: any scale in range, the ends included */
  uint32_t s=0x3D800000u+(uint32_t)(po_rnd()%0x02000001u);
  if(it<2u)s=it?0x3F800000u:0x3D800000u;
  {int32_t inv;if(!po_fixed(1.0f/po_float(s),65536.0f,16.0f,&inv))bad("inverse range",it);
   if((int32_t)invoke("_test_inverse",s,0,0,0).result!=inv)bad("inverse",it);}
  r_inverse++;
  /* basis value: any bits (the check included) times that scale */
  uint32_t v=po_rnd()%3u?(uint32_t)(po_rnd()&0x807FFFFFu)|(uint32_t)(100u+po_rnd()%28u)<<23:rnd_float_bits();
  if(po_rnd()%16u==0u)v=(po_rnd()&1u?0x80000000u:0u)|(0x3F800000u+(uint32_t)(po_rnd()%0x60u));
  const float bv=po_float(v);
  int32_t B,M;
  const bool ok=po_fixed(bv,1073741824.0f,1.00001f,&B)&&po_fixed(bv*po_float(s),65536.0f,1.00001f,&M);
  if((invoke("_test_element",v,s,0,0).result!=0)!=ok)bad("element check",it);
  if(ok){const u32 e=symc("element_out");
   if((int32_t)rl(&cpu,e)!=B||(int32_t)rl(&cpu,e+4)!=M)bad("element",it);}
  r_element++;
 }
}

/* ---- cost ---- */
struct stats {double mean,median;unsigned min,max;double instr;};
static int cmp_u(const void*a,const void*b){unsigned x=*(const unsigned*)a,y=*(const unsigned*)b;return x<y?-1:x>y;}
static struct stats summarize(unsigned *cycles,size_t n,uint64_t instr){
 qsort(cycles,n,sizeof *cycles,cmp_u);double sum=0;for(size_t i=0;i<n;i++)sum+=cycles[i];
 return (struct stats){sum/(double)n,n&1?cycles[n/2]:(cycles[n/2-1]+cycles[n/2])/2.0,cycles[0],cycles[n-1],(double)instr/(double)n};
}
static void print_stats(struct stats s){printf("{\"meanCycles\":%.2f,\"medianCycles\":%.1f,\"minCycles\":%u,\"maxCycles\":%u,\"meanInstructions\":%.2f}",s.mean,s.median,s.min,s.max,s.instr);}
enum {POSES=2000};
static void cost_row(const char *name,const char *description,int scales,bool *first) {
 static unsigned cyc[3][POSES];uint64_t ins[3]={0,0,0};
 po_state=UINT64_C(0x5eedc0de00000001)+(uint64_t)scales;
 sh2z_view_t v;po_base(&v);po_case_t c;
 for(unsigned n=0;n<POSES;n++){
  sh2z_pose_t want;
  do{
   po_random_case(&c);
   if(scales==0)c.scale[1]=c.scale[2]=c.scale[0];
   else {c.scale[0]=0.1f;c.scale[1]=0.125f;c.scale[2]=0.15f;}
  }while(!po_pose(&want,&v,(const int32_t (*)[3])c.rotation,(const float (*)[3])c.basis,c.scale,c.position,c.eye,c.flags,PO_EXACT));
  put_case(&c,&v);
  static const char *const entries[3]={"fast_pose","fast_pose","plain_pose"};
  for(unsigned k=0;k<3;k++){
   const struct cost r=call_sr(entries[k],c.flags,k==1?2u:0u);
   if(r.result!=1)bad("cost call",n);
   sh2z_pose_t got;get_pose(k==2,&got);if(!po_same(&got,&want))bad("cost fields",n);
   cyc[k][n]=r.cycles;ins[k]+=r.instructions;
  }
 }
 if(!*first)putchar(',');*first=false;
 printf("\"%s\":{\"description\":\"%s\",\"poses\":%u,\"fast\":",name,description,POSES);print_stats(summarize(cyc[0],POSES,ins[0]));
 printf(",\"fastSrS\":");print_stats(summarize(cyc[1],POSES,ins[1]));
 printf(",\"plain\":");print_stats(summarize(cyc[2],POSES,ins[2]));printf("}");
}
static void cost_base(void) {
 static unsigned cyc[POSES];uint64_t ins=0;
 po_state=UINT64_C(0xba5e000000000001);
 const u32 sv=symc("source_view");
 const int32_t words[10]={160*65536,160*65536,160*65536,112*65536,16*4096,2000*4096,12,16,308,224};
 for(unsigned i=0;i<10;i++)wl(&cpu,sv+4*i,(u32)words[i]);
 sh2z_view_t v;po_base(&v);po_case_t c;
 for(unsigned n=0;n<POSES;n++){
  po_random_case(&c);put_case(&c,&v);
  const struct cost r=invoke("_plain_base",0,0,0,0);
  if(r.result!=1)bad("plain_base",n);
  /* Its depth, centre and clip fields are the base the library reads. */
  const u32 b=symc("plain_base_view");
  if((int32_t)rl(&cpu,b+32)!=v.focal_x||(int32_t)rl(&cpu,b+44)!=v.near_z||(int32_t)rl(&cpu,b+48)!=v.far_z||
     (int16_t)rw(&cpu,b+40)!=v.center_x||(int16_t)rw(&cpu,b+42)!=v.center_y||(int16_t)rw(&cpu,b+58)!=v.clip[3])bad("plain_base fields",n);
  cyc[n]=r.cycles;ins+=r.instructions;
 }
 printf(",\"plainBase\":");print_stats(summarize(cyc,POSES,ins));
}

int main(int argc,char**argv){
 if(argc<3){fprintf(stderr,"usage: pose_harness model.elf verify-count\n");return 2;}
 const uint64_t count=strtoull(argv[2],0,10);
 load_elf_symbols(argv[1]);load_elf(argv[1]);
 cpu.model=SHMT_SH2;cpu.MappedMemoryReadByte=rb;cpu.MappedMemoryReadWord=rw;cpu.MappedMemoryReadLong=rl;cpu.MappedMemoryWriteByte=wb;cpu.MappedMemoryWriteWord=ww;cpu.MappedMemoryWriteLong=wl;
 for(unsigned i=0;i<65536;i++)cpu.opcodes[i]=decode(SHMT_SH2,(u16)i);for(unsigned i=0;i<256;i++)cpu.fetchlist[i]=fetch;
 verify(count);
 verify_routines(count*16u);
 printf("{\"schema\":1,\"kind\":\"Yabause isolated SH2 interpreter flat-RAM model, not Saturn timing\",");
 printf("\"verified\":{\"cases\":%"PRIu64",\"accepted\":%"PRIu64",\"acceptedCullInvalid\":%"PRIu64",\"fieldsPerEntry\":%"PRIu64",\"aliasedCases\":%"PRIu64",\"entries\":\"fast_pose (SR.S clear and set) and plain_pose: success, all 37 output fields, output untouched on failure, against tests/pose_oracle.h; each accepted case again with base as &out->view and with every float input inside *out, SR.S clear and set\"},",
  v_cases,v_ok,v_cull_off,v_fields/3u,v_alias);
 printf("\"routines\":{\"delta\":%"PRIu64",\"inverse\":%"PRIu64",\"element\":%"PRIu64",\"entries\":\"the internal SH-2 difference, reciprocal and basis-value routines through their test entries, one value per call, against the binary32 oracle\"},",
  r_delta,r_inverse,r_element);
 bool first=true;printf("\"rows\":{");
 cost_row("uniform_scale","visible poses, one scale on all three axes",0,&first);
 cost_row("three_scales","visible poses, scales 0.1, 0.125 and 0.15",1,&first);
 printf("}");
 cost_base();
 printf("}\n");
 return 0;
}
