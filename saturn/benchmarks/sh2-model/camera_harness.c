/* SPDX-License-Identifier: GPL-2.0-or-later
 * Executed-instruction checks and cycle model for view-camera-1
 * (sh2z_view_from_camera30, sh2z_view_place_q12). Runs the actual
 * big-endian SH-2 object code in Yabause's unmodified interpreter, like
 * harness.c, whose loader and call wrapper it reuses. Flat RAM, no caches or
 * wait states: MODELLED cycles, not Saturn timing. Copyright (c) 2026 SH2ZAM
 * Contributors. This optional model harness is licensed separately from
 * SH2ZAM's MIT core; see LICENSE.
 *
 * usage: camera_harness model.elf verify-scale > results.json
 * The ELF must contain bridge.o from benchmarks/camera-model and the library.
 * Every output of fast_camera and fast_place (return value and every field,
 * or untouched bytes on failure) is checked against the independent oracle
 * in tests/camera_oracle.h, and every output of plain_chain and
 * plain_adapter against the same float chain run on the host, before any
 * timing is printed. */
#define main sh2_model_main
#include "harness.c"
#undef main
#include "../../tests/camera_oracle.h"
#include "../../tests/camera_float_ref.h"
#include <stddef.h>

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
static void bad(const char *what,uint64_t it){fprintf(stderr,"camera model mismatch: %s (case %"PRIu64")\n",what,it);exit(3);}

static uint32_t rng=UINT32_C(0xca3e7a11);
static uint32_t rnd(void){rng^=rng<<13;rng^=rng>>17;rng^=rng<<5;return rng;}
static int32_t range(int32_t lo,int32_t hi){return (int32_t)((int64_t)lo+(int64_t)(rnd()%(uint64_t)((int64_t)hi-lo+1)));}
static long double unit(void){return (long double)rnd()/4294967296.0L;}

/* ---- target layouts (the same field offsets as the host; big-endian words) ---- */
static void put32(u32 a,int32_t v){wl(&cpu,a,(u32)v);}
static int32_t get32(u32 a){return (int32_t)rl(&cpu,a);}
static void put_view(u32 a,const sh2z_view_t *v){
 for(unsigned i=0;i<9;i++)ww(&cpu,a+2*i,(u16)v->r[i/3][i%3]);
 for(unsigned i=0;i<3;i++)put32(a+20+4*i,v->t[i]);
 put32(a+32,v->focal_x);put32(a+36,v->focal_y);ww(&cpu,a+40,(u16)v->center_x);ww(&cpu,a+42,(u16)v->center_y);
 put32(a+44,v->near_z);put32(a+48,v->far_z);for(unsigned i=0;i<4;i++)ww(&cpu,a+52+2*i,(u16)v->clip[i]);
}
static void get_view(u32 a,sh2z_view_t *v){
 for(unsigned i=0;i<9;i++)v->r[i/3][i%3]=(int16_t)rw(&cpu,a+2*i);
 for(unsigned i=0;i<3;i++)v->t[i]=get32(a+20+4*i);
 v->focal_x=get32(a+32);v->focal_y=get32(a+36);v->center_x=(int16_t)rw(&cpu,a+40);v->center_y=(int16_t)rw(&cpu,a+42);
 v->near_z=get32(a+44);v->far_z=get32(a+48);for(unsigned i=0;i<4;i++)v->clip[i]=(int16_t)rw(&cpu,a+52+2*i);
}
static void get_center(u32 a,sh2z_view_center_t *c){get_view(a,&c->view);c->center_x16=get32(a+60);c->center_y16=get32(a+64);}
static void put_lens(const sh2z_view_lens_t *l){
 const u32 a=symc("lens");
 put32(a,l->fov_q16);put32(a+4,l->scale_x16);put32(a+8,l->scale_y16);put32(a+12,l->center_x16);
 put32(a+16,l->center_y16);put32(a+20,l->near_q12);put32(a+24,l->far_q12);
 for(unsigned i=0;i<4;i++)ww(&cpu,a+28+2*i,(u16)l->clip[i]);
}
static void put_inputs(const int32_t b[9],const int32_t e[3],const sh2z_view_lens_t *l){
 const u32 ba=symc("basis"),ea=symc("eye");
 for(unsigned i=0;i<9;i++)put32(ba+4*i,b[i]);
 for(unsigned i=0;i<3;i++)put32(ea+4*i,e[i]);
 put_lens(l);
}
static void putf(u32 a,float f){wl(&cpu,a,cf_bits(f));}
static void put_camera(const cf_camera_t *c,const cf_frame_t *f){
 const u32 a=symc("cam"),fa=symc("frame");
 const float *v=&c->eye[0];
 _Static_assert(sizeof(cf_camera_t)==96,"camera layout");
 for(unsigned i=0;i<24;i++)putf(a+4*i,v[i]);
 for(unsigned i=0;i<9;i++)putf(fa+(u32)offsetof(cf_frame_t,basis)+4*i,f->basis[i/3][i%3]);
}

/* ---- inputs ---- */
static void rotation(long double r[3][3],const long double *toward){
 long double f[3],u[3],s[3],len;
 if(toward){for(unsigned i=0;i<3;i++)f[i]=toward[i];}
 else do{for(unsigned i=0;i<3;i++)f[i]=unit()*2-1;len=sqrtl(f[0]*f[0]+f[1]*f[1]+f[2]*f[2]);}while(len<0.1L||len>1);
 len=sqrtl(f[0]*f[0]+f[1]*f[1]+f[2]*f[2]);for(unsigned i=0;i<3;i++)f[i]/=len;
 do{for(unsigned i=0;i<3;i++)u[i]=unit()*2-1;s[0]=u[1]*f[2]-u[2]*f[1];s[1]=u[2]*f[0]-u[0]*f[2];s[2]=u[0]*f[1]-u[1]*f[0];
  len=sqrtl(s[0]*s[0]+s[1]*s[1]+s[2]*s[2]);}while(len<0.1L);
 for(unsigned i=0;i<3;i++)s[i]/=len;
 const long double up[3]={f[1]*s[2]-f[2]*s[1],f[2]*s[0]-f[0]*s[2],f[0]*s[1]-f[1]*s[0]};
 for(unsigned i=0;i<3;i++){r[0][i]=-s[i];r[1][i]=up[i];r[2][i]=f[i];}
}
static int32_t special(void){
 static const int32_t k[]={0,1,-1,32767,32768,-32768,-32769,65535,65536,-65536,INT32_MAX,INT32_MIN,
  INT32_MAX-32767,INT32_MAX-32768,INT32_MAX-32769,1073741824,-1073741824,5759,5760,5761,11790719,11790720,
  536870911,536870912,(int32_t)UINT32_C(0x80008000),0x7fff8000,0x7fff7fff};
 return k[rnd()%(sizeof k/sizeof k[0])];
}
static int32_t any(void){switch(rnd()%5u){case 0:return (int32_t)rnd();case 1:return special();case 2:return range(-65536*3,65536*3);default:return range(-(1<<30),1<<30);}}
static void race_lens(sh2z_view_lens_t *l){
 l->fov_q16=80*65536;l->scale_x16=160*65536;l->scale_y16=160*65536;l->center_x16=160*65536;l->center_y16=97*65536;
 l->near_q12=16*4096;l->far_q12=8192*4096;l->clip[0]=12;l->clip[1]=8;l->clip[2]=307;l->clip[3]=215;
}
static void unit_basis(int32_t b[9],bool jitter){
 long double r[3][3];rotation(r,NULL);
 for(unsigned i=0;i<9;i++)b[i]=(int32_t)(llroundl(r[i/3][i%3]*1073741824.0L)+(jitter?range(-8,8):0));
}
static void random_case(int32_t b[9],int32_t e[3],sh2z_view_lens_t *l){
 if(rnd()%3u){unit_basis(b,rnd()%2u);if(rnd()%8u==0)b[rnd()%9u]=any();}
 else for(unsigned i=0;i<9;i++)b[i]=any();
 for(unsigned i=0;i<3;i++)switch(rnd()%4u){case 0:e[i]=range(-32767*4096,32767*4096);break;case 1:e[i]=range(-75000*4096,75000*4096);break;
  case 2:e[i]=(int32_t)rnd();break;default:e[i]=special();break;}
 race_lens(l);
 switch(rnd()%8u){
 case 0:l->fov_q16=special();break;
 case 1:l->fov_q16=range(1,1023)*11520-5760+range(-2,2);break;
 case 2:l->scale_x16=rnd()%2u?special():(int32_t)rnd();l->scale_y16=rnd()%2u?l->scale_x16:range(-2,2);break;
 case 3:l->near_q12=any();l->far_q12=any();break;
 case 4:for(unsigned i=0;i<4;i++)l->clip[i]=(int16_t)range(-40,360);break;
 case 5:l->center_x16=any();l->center_y16=any();break;
 default:l->fov_q16=range(30*65536,120*65536);l->center_y16=range(17*65536,192*65536);break;
 }
}

/* ---- verification ---- */
static uint64_t v_camera,v_camera_fail,v_place,v_place_fail,v_chain,v_chain_fail;
static void verify_one(const int32_t b[9],const int32_t e[3],const sh2z_view_lens_t *l,uint64_t it){
 put_inputs(b,e,l);
 const u32 oa=symc("out");
 for(unsigned i=0;i<68;i++)wb(&cpu,oa+i,0xA5);
 const struct cost c=call("fast_camera");
 sh2z_view_center_t want;
 const int ok=co_from_camera30(&want,b,e,l,0);
 if((c.result&0xffu)!=(u32)ok)bad("fast_camera result",it);
 if(ok){sh2z_view_center_t got;get_center(oa,&got);if(!co_same_center(&got,&want))bad("fast_camera view",it);}
 else{for(unsigned i=0;i<68;i++)if(rb(&cpu,oa+i)!=0xA5)bad("fast_camera wrote on failure",it);v_camera_fail++;}
 v_camera++;
 /* the placement on its own, from the view (or an arbitrary one) */
 sh2z_view_t pv;
 if(ok)pv=want.view;else{memset(&pv,0,sizeof pv);
  for(unsigned i=0;i<9;i++)pv.r[i/3][i%3]=(int16_t)rnd();pv.focal_x=range(-1,3);pv.focal_y=range(-1,3);
  pv.near_z=range(-1,3);pv.far_z=range(-1,5);for(unsigned i=0;i<4;i++)pv.clip[i]=(int16_t)range(-2,2);}
 for(unsigned i=0;i<3;i++)if(rnd()%2u)pv.t[i]=any();
 put_view(symc("place_view"),&pv);
 const struct cost p=call("fast_place");
 sh2z_view_t wv=pv,gv;
 const int pok=co_place_q12(&wv,e,0);
 if((p.result&0xffu)!=(u32)pok)bad("fast_place result",it);
 get_view(symc("place_view"),&gv);
 if(!co_same_view(&gv,&wv))bad("fast_place view",it);
 v_place++;if(!pok)v_place_fail++;
}
static float table[4096];
/* A world pose for the float chain: eye in the int16 world, a target 50 to
 * 2000 units ahead, an up hint near the true up, a race-like lens. */
static void world_pose(cf_camera_t *c,cf_frame_t *f,int32_t b[9],int32_t e[3],sh2z_view_lens_t *l){
 long double r[3][3];rotation(r,NULL);
 int32_t at[3];const long double dist=50.0L+1950.0L*unit();
 for(unsigned i=0;i<3;i++){e[i]=range(-32767*4096,32767*4096);at[i]=(int32_t)llroundl((e[i]/4096.0L+r[2][i]*dist)*4096.0L);}
 const int32_t fov=range(40*65536,110*65536),fcx=rnd()%2u?range(-80*65536,80*65536):0,fcy=range(-80*65536,80*65536);
 for(unsigned i=0;i<3;i++){c->eye[i]=(float)(e[i]/4096.0L);c->at[i]=(float)(at[i]/4096.0L);c->up[i]=(float)(r[1][i]+(unit()-0.5L)*0.2L);}
 c->fov=(float)fov/65536.0f;c->near_plane=16.0f;c->far_plane=8192.0f;c->fov_scale_x=320.0f;c->fov_scale_y=240.0f;
 c->frustum_x=(float)fcx/65536.0f;c->frustum_y=(float)fcy/65536.0f;
 c->viewport[0]=160.0f;c->viewport[1]=120.0f;c->viewport[2]=160.0f;c->viewport[3]=120.0f;
 c->scissor[0]=12.0f;c->scissor[1]=16.0f;c->scissor[2]=308.0f;c->scissor[3]=224.0f;
 memset(f,0,sizeof *f);
 for(unsigned i=0;i<3;i++){f->basis[0][i]=c->at[i]-c->eye[i];f->basis[1][i]=c->up[i];}
 if(cf_orthonormalize_forward(f->basis))bad("float basis",0);
 /* the fixed-point camera's basis: rows {-side, up, forward} of the same basis */
 for(unsigned i=0;i<3;i++){b[i]=(int32_t)lroundf(-f->basis[2][i]*1073741824.0f);b[3+i]=(int32_t)lroundf(f->basis[1][i]*1073741824.0f);
  b[6+i]=(int32_t)lroundf(f->basis[0][i]*1073741824.0f);}
 l->fov_q16=fov;l->scale_x16=160*65536;l->scale_y16=160*65536;l->center_x16=160*65536-fcx;l->center_y16=112*65536+fcy;
 l->near_q12=16*4096;l->far_q12=8192*4096;l->clip[0]=12;l->clip[1]=8;l->clip[2]=307;l->clip[3]=215;
}
static void verify_chain(uint64_t n){
 const u32 pa=symc("plain_out");
 for(uint64_t it=0;it<n;it++){
  cf_camera_t c;cf_frame_t f,hf;int32_t b[9],e[3];sh2z_view_lens_t l;
  world_pose(&c,&f,b,e,&l);hf=f;
  sh2z_view_center_t want;const int ok=cf_chain(&want,&hf,&c,table);
  put_camera(&c,&f);
  for(const char *const*k=(const char*const[]){"plain_chain","plain_adapter",NULL};*k;k++){
   for(unsigned i=0;i<68;i++)wb(&cpu,pa+i,0);
   const struct cost r=call(*k);
   if((r.result&0xffu)!=(u32)ok)bad(*k,it);
   if(ok){sh2z_view_center_t got;get_center(pa,&got);if(!co_same_center(&got,&want))bad("plain view",it);}
  }
  v_chain++;if(!ok)v_chain_fail++;
 }
}

/* ---- cost ---- */
enum {POSES=64};
struct stats{double mean,median;unsigned min,max;double instr;};
static int cmp_u(const void*a,const void*b){unsigned x=*(const unsigned*)a,y=*(const unsigned*)b;return x<y?-1:x>y;}
static struct stats summarize(unsigned *cy,size_t n,uint64_t instr){
 qsort(cy,n,sizeof *cy,cmp_u);double s=0;for(size_t i=0;i<n;i++)s+=cy[i];
 return (struct stats){s/(double)n,n&1?cy[n/2]:(cy[n/2-1]+cy[n/2])/2.0,cy[0],cy[n-1],(double)instr/(double)n};
}
static void print_stats(struct stats s){printf("{\"meanCycles\":%.2f,\"medianCycles\":%.1f,\"minCycles\":%u,\"maxCycles\":%u,\"meanInstructions\":%.2f}",s.mean,s.median,s.min,s.max,s.instr);}
static void cost(void){
 static const char *const names[4]={"fast_camera","fast_place","plain_chain","plain_adapter"};
 static unsigned cy[4][POSES];uint64_t ins[4]={0,0,0,0};
 rng=UINT32_C(0xc0571234);
 for(unsigned p=0;p<POSES;p++){
  cf_camera_t c;cf_frame_t f;int32_t b[9],e[3];sh2z_view_lens_t l;
  world_pose(&c,&f,b,e,&l);
  put_inputs(b,e,&l);put_camera(&c,&f);
  sh2z_view_center_t want;if(!co_from_camera30(&want,b,e,&l,0))bad("cost pose",p);
  put_view(symc("place_view"),&want.view);
  for(unsigned k=0;k<4;k++){
   const struct cost r=call(names[k]);
   if((r.result&0xffu)!=1u)bad(names[k],p);
   cy[k][p]=r.cycles;ins[k]+=r.instructions;
  }
  sh2z_view_center_t got;get_center(symc("out"),&got);if(!co_same_center(&got,&want))bad("cost view",p);
 }
 printf("\"rows\":{\"world_poses\":{\"description\":\"%u camera poses in the int16 world (eye anywhere, 40 to 110 degree field of view, off-axis frustum centre), the source's 160 x 120 viewport and 16..8192 depth\",\"poses\":%u",POSES,POSES);
 for(unsigned k=0;k<4;k++){printf(",\"%s\":",names[k]);print_stats(summarize(cy[k],POSES,ins[k]));}
 printf("}}");
}

int main(int argc,char**argv){
 if(argc<3){fprintf(stderr,"usage: camera_harness model.elf verify-scale\n");return 2;}
 const uint64_t scale=strtoull(argv[2],0,10);
 load_elf_symbols(argv[1]);load_elf(argv[1]);
 cpu.model=SHMT_SH2;cpu.MappedMemoryReadByte=rb;cpu.MappedMemoryReadWord=rw;cpu.MappedMemoryReadLong=rl;cpu.MappedMemoryWriteByte=wb;cpu.MappedMemoryWriteWord=ww;cpu.MappedMemoryWriteLong=wl;
 for(unsigned i=0;i<65536;i++)cpu.opcodes[i]=decode(SHMT_SH2,(u16)i);for(unsigned i=0;i<256;i++)cpu.fetchlist[i]=fetch;
 cf_sin_table_init(table);
 {const u32 a=symc("sin_table");for(unsigned i=0;i<4096;i++)putf(a+4*i,table[i]);}
 /* every half-angle boundary and the range ends */
 {sh2z_view_lens_t l;race_lens(&l);int32_t b[9]={1073741824,0,0,0,1073741824,0,0,0,1073741824},e[3]={0,0,0};
  for(int32_t h=0;h<=1024;h++)for(int32_t d=-1;d<=1;d++){l.fov_q16=h*11520-5760+d;verify_one(b,e,&l,(uint64_t)h);}}
 for(uint64_t it=0;it<scale;it++){int32_t b[9],e[3];sh2z_view_lens_t l;random_case(b,e,&l);verify_one(b,e,&l,it);}
 verify_chain(scale/20u+16u);
 printf("{\"schema\":1,\"kind\":\"Yabause isolated SH2 interpreter flat-RAM model, not Saturn timing\",");
 printf("\"verified\":{\"cameraCalls\":%"PRIu64",\"cameraRefusals\":%"PRIu64",\"placeCalls\":%"PRIu64",\"placeRefusals\":%"PRIu64",\"floatChainPoses\":%"PRIu64",\"floatChainRefusals\":%"PRIu64",\"entries\":\"fast_camera and fast_place return values and every field (or untouched output on failure) against tests/camera_oracle.h; plain_chain and plain_adapter against the same binary32 chain run on the host\"},",
  v_camera,v_camera_fail,v_place,v_place_fail,v_chain,v_chain_fail);
 cost();
 printf("}\n");
 return 0;
}
