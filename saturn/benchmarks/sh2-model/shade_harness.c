/* SPDX-License-Identifier: GPL-2.0-or-later
 * Executed-instruction checks and cycle model for the shade-1 kernels
 * (shade_experimental.h). Runs the actual big-endian SH-2 object code in
 * Yabause's unmodified interpreter, like harness.c, whose loader and call
 * wrapper it reuses. Flat RAM, no caches or wait states: MODELLED cycles, not
 * Saturn timing. Copyright (c) 2026 SH2ZAM Contributors. This optional model
 * harness is licensed separately from SH2ZAM's MIT core; see LICENSE.
 *
 * usage: shade_harness model.elf verify-scale > results.json
 * The ELF must contain bridge.o from benchmarks/shade-model and the library
 * sources it calls; symbols are read from its symbol table. Every result of
 * every fast_ and plain_ entry is checked against the independent oracle in
 * tests/shade_oracle.h before any timing is printed. */
#define main sh2_model_main
#include "harness.c"
#undef main
#include "../../tests/shade_oracle.h"

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
static struct cost call(const char *name,u32 a,u32 b,u32 c,u32 d) {
 char tmp[140];snprintf(tmp,sizeof tmp,"_%s",name);return invoke(tmp,a,b,c,d);
}
static u32 symc(const char *name) {char tmp[140];snprintf(tmp,sizeof tmp,"_%s",name);return sym(tmp);}
static void bad(const char *what,uint64_t it) {fprintf(stderr,"shade model mismatch: %s (case %"PRIu64")\n",what,it);exit(3);}

static uint32_t rng=UINT32_C(0x5ade5eed);
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

/* ---- target memory helpers (big-endian, SH-2 struct layouts) ---- */
static void put8(const char *s,u32 off,uint8_t v){wb(&cpu,symc(s)+off,v);}
static void put16(const char *s,u32 off,uint16_t v){ww(&cpu,symc(s)+off,v);}
static void put32(const char *s,u32 off,uint32_t v){wl(&cpu,symc(s)+off,v);}
static uint8_t get8(const char *s,u32 off){return rb(&cpu,symc(s)+off);}
static uint16_t get16(const char *s,u32 off){return rw(&cpu,symc(s)+off);}
static uint32_t get32(const char *s,u32 off){return rl(&cpu,symc(s)+off);}

/* ---- host copies of the inputs ---- */
enum {MAXN=256,MAXP=1024,MAXF=256};
static sh2z_vec3_t h_l,h_r;static int16_t h_basis[3][3];static uint8_t h_ambient[3],h_color[3],h_lut[256];
static int8_t h_normals[MAXN*4];static uint8_t h_lit[MAXN*4],h_bases[64*3],h_fog[3];
static sh2z_shade_pair_t h_pairs[MAXP];
static sh2z_shade_plane_t h_planes[MAXF];static uint16_t h_order[MAXF];static int32_t h_eye[3];static uint32_t h_err;
static int32_t h_fogp[5];static int32_t h_depths[MAXP];static uint8_t h_fades[MAXP];
static int16_t h_slope[3],h_bias[3];static uint8_t h_near[3],h_far[3];

static void load_light(void) {
 const int32_t *lv=&h_l.x,*rv=&h_r.x;
 for(unsigned k=0;k<3;k++){put32("prep_light",4*k,(uint32_t)lv[k]);put32("prep_reflection",4*k,(uint32_t)rv[k]);
  put8("prep_ambient",k,h_ambient[k]);put8("prep_color",k,h_color[k]);
  for(unsigned j=0;j<3;j++)put16("prep_basis",2*(3*k+j),(uint16_t)h_basis[k][j]);}
 for(unsigned i=0;i<256;i++)put8("lut",i,h_lut[i]);
}
static void load_normals(unsigned n){for(unsigned i=0;i<4*n;i++)put8("normals",i,(uint8_t)h_normals[i]);}
static void load_lit(unsigned n){for(unsigned i=0;i<4*n;i++)put8("lit",i,h_lit[i]);}
static void load_bases(void){for(unsigned i=0;i<64*3;i++)put8("bases",i,h_bases[i]);for(unsigned k=0;k<3;k++)put8("fog_rgb",k,h_fog[k]);}
static void load_pairs(const char *s,const sh2z_shade_pair_t *p,unsigned n){for(unsigned i=0;i<n;i++){put16(s,4*i,p[i].base);put16(s,4*i+2,p[i].light);}}
static void load_planes(unsigned n){
 for(unsigned i=0;i<n;i++){for(unsigned k=0;k<3;k++){put16("planes",16*i+2*k,(uint16_t)h_planes[i].normal[k]);put16("planes",16*i+6+2*k,(uint16_t)h_planes[i].first[k]);}
  put32("planes",16*i+12,(uint32_t)h_planes[i].plane);put16("order",2*i,h_order[i]);}
 for(unsigned k=0;k<3;k++)put32("eye",4*k,(uint32_t)h_eye[k]);
 put32("eye_error",0,h_err);
}
static void load_fogp(void){for(unsigned k=0;k<5;k++)put32("fog_params",4*k,(uint32_t)h_fogp[k]);}
static void load_depths(unsigned n){for(unsigned i=0;i<n;i++)put32("depths",4*i,(uint32_t)h_depths[i]);}
static void load_fades(unsigned n){for(unsigned i=0;i<n;i++)put8("fades",i,h_fades[i]);}
static void load_fade_params(void){for(unsigned k=0;k<3;k++){put16("fade_slope",2*k,(uint16_t)h_slope[k]);put16("fade_bias",2*k,(uint16_t)h_bias[k]);put8("near_rgb",k,h_near[k]);put8("far_rgb",k,h_far[k]);}}

/* ---- generators ---- */
static int8_t rand_n8(void){static const int8_t e[]={-128,-127,-1,0,1,127};return rnd()%5u==0u?e[rnd()%6u]:(int8_t)range(-128,127);}
static uint8_t rand_u8(void){static const uint8_t e[]={0,1,15,16,127,128,240,254,255};return rnd()%3u==0u?e[rnd()%9u]:(uint8_t)rnd();}
static int32_t rand_dir(void){switch(rnd()%4u){case 0:return 0;case 1:return (int8_t)range(-128,127)*65536;case 2:return (int16_t)range(-32768,32767)*65536;default:return range(-2000000000,2000000000);}}
static void rand_rotation(int16_t b[3][3]) {
 const double a=unit()*6.283185307179586,c=unit()*6.283185307179586,d=unit()*3.141592653589793;
 const double ca=cos(a),sa=sin(a),cc=cos(c),sc=sin(c),cd=cos(d),sd=sin(d);
 const double m[3][3]={{ca*cc-sa*cd*sc,-ca*sc-sa*cd*cc,sa*sd},{sa*cc+ca*cd*sc,-sa*sc+ca*cd*cc,-ca*sd},{sd*sc,sd*cc,cd}};
 for(int i=0;i<3;i++)for(int j=0;j<3;j++)b[i][j]=(int16_t)lround(m[i][j]*16384.0);
}
static void rand_light_inputs(bool realistic) {
 if(realistic){
  h_l=(sh2z_vec3_t){(int8_t)range(-127,127)*65536,(int8_t)range(-127,127)*65536,(int8_t)range(1,127)*65536};
  h_r=(sh2z_vec3_t){(int16_t)range(-32767,32767)*65536,(int16_t)range(1,32767)*65536,(int16_t)range(-32767,32767)*65536};
  rand_rotation(h_basis);
  for(int k=0;k<3;k++){h_ambient[k]=(uint8_t)range(16,96);h_color[k]=(uint8_t)range(96,255);}
 } else {
  h_l=(sh2z_vec3_t){rand_dir(),rand_dir(),rand_dir()};h_r=(sh2z_vec3_t){rand_dir(),rand_dir(),rand_dir()};
  if(rnd()%2u)rand_rotation(h_basis);else for(int i=0;i<3;i++)for(int j=0;j<3;j++)h_basis[i][j]=(int16_t)range(-16384,16384);
  for(int k=0;k<3;k++){h_ambient[k]=rand_u8();h_color[k]=rand_u8();}
 }
 for(int i=0;i<256;i++)h_lut[i]=(uint8_t)rnd();
}
static void rand_normals(unsigned n,bool realistic) {
 for(unsigned i=0;i<n;i++){
  if(realistic){double x,y,z,l;do{x=unit()*2-1;y=unit()*2-1;z=unit()*2-1;l=sqrt(x*x+y*y+z*z);}while(l<0.1||l>1);
   h_normals[4*i]=(int8_t)lround(x/l*127);h_normals[4*i+1]=(int8_t)lround(y/l*127);h_normals[4*i+2]=(int8_t)lround(z/l*127);}
  else for(int k=0;k<3;k++)h_normals[4*i+k]=rand_n8();
  h_normals[4*i+3]=0;
 }
}
/* A closed convex-ish mesh: face normals on the sphere (Q14), a corner at radius
 * 600 along the normal with jitter, plane = n . corner; eye 1500..20000 away. */
static void rand_mesh(unsigned f,bool realistic) {
 for(unsigned i=0;i<f;i++){
  double x,y,z,l;do{x=unit()*2-1;y=unit()*2-1;z=unit()*2-1;l=sqrt(x*x+y*y+z*z);}while(l<0.1||l>1);
  sh2z_shade_plane_t *p=&h_planes[i];
  if(realistic){p->normal[0]=(int16_t)lround(x/l*16384);p->normal[1]=(int16_t)lround(y/l*16384);p->normal[2]=(int16_t)lround(z/l*16384);
   for(int k=0;k<3;k++)p->first[k]=(int16_t)(lround((k==0?x:k==1?y:z)/l*600)+range(-40,40));}
  else for(int k=0;k<3;k++){p->normal[k]=(int16_t)(rnd()%6u==0u?(rnd()%2u?16384:-16384):range(-16384,16384));p->first[k]=(int16_t)range(-2048,2048);}
  int64_t c=0;for(int k=0;k<3;k++)c+=(int64_t)p->normal[k]*p->first[k];
  if(!realistic&&rnd()%4u==0u)c+=range(-200000,200000);
  p->plane=(int32_t)(c>SH2Z_SHADE_PLANE_MAX?SH2Z_SHADE_PLANE_MAX:c< -SH2Z_SHADE_PLANE_MAX?-SH2Z_SHADE_PLANE_MAX:c);
  h_order[i]=(uint16_t)i;
 }
 for(unsigned i=f;i>1;i--){unsigned j=rnd()%i;uint16_t t=h_order[i-1];h_order[i-1]=h_order[j];h_order[j]=t;}
 if(realistic){double x,y,z,l;do{x=unit()*2-1;y=unit()*2-1;z=unit()*2-1;l=sqrt(x*x+y*y+z*z);}while(l<0.1||l>1);
  const double d=1500+unit()*18500;h_eye[0]=(int32_t)lround(x/l*d);h_eye[1]=(int32_t)lround(y/l*d);h_eye[2]=(int32_t)lround(z/l*d);h_err=2;}
 else{for(int k=0;k<3;k++)h_eye[k]=rnd()%6u==0u?(rnd()%2u?30000:-30000):range(-30000,30000);h_err=rnd()%3u==0u?SH2Z_SHADE_ERROR_MAX:rnd()%8u;}
}
static int32_t near_z_used;
static void rand_fogp(bool perspective) {
 if(perspective){const int32_t n=range(1,32),f=range(n+64,30000),fs=range(0,996);
  h_fogp[0]=128000/(1000-fs);h_fogp[1]=(500-fs)*256/(1000-fs);
  h_fogp[2]=(int32_t)(((int64_t)(f+n)<<30)/(f-n));h_fogp[3]=(int32_t)-((int64_t)2*f*n*4096/(f-n));
  int64_t fz=(int64_t)f*16384;h_fogp[4]=(int32_t)(fz>INT32_MAX-1?INT32_MAX-1:fz);near_z_used=n*16384;}
 else{h_fogp[0]=rnd()%5u==0u?(rnd()%2u?1:INT32_MAX):range(1,40000);h_fogp[1]=rnd()%5u==0u?(rnd()%2u?INT32_MIN:INT32_MAX):range(-40000,300);
  h_fogp[2]=rnd()%5u==0u?(rnd()%2u?INT32_MIN:INT32_MAX):(int32_t)rnd();h_fogp[3]=rnd()%5u==0u?(rnd()%2u?INT32_MIN:0):-(int32_t)(rnd()&0x7fffffffu);
  h_fogp[4]=rnd()%5u==0u?(rnd()%2u?1:INT32_MAX):range(1,INT32_MAX);near_z_used=1;}
}
static int32_t rand_depth(const uint32_t d[256],bool realistic) {
 if(realistic)return range(near_z_used,h_fogp[4]);
 switch(rnd()%5u){case 0:{int64_t z=(int64_t)d[rnd()%256u]+range(-2,1);return (int32_t)(z<INT32_MIN?INT32_MIN:z>INT32_MAX?INT32_MAX:z);}
 case 1:return range(INT32_MIN,0);case 2:return (int32_t)rnd();default:return range(0,h_fogp[4]);}
}

/* ---- verification ---- */
static uint64_t verify_light(uint64_t iterations) {
 uint64_t done=0;
 for(uint64_t it=0;it<iterations;it++){
  rand_light_inputs(it%3u==0u);const unsigned n=it%4u==0u?256u:1u+rnd()%256u;rand_normals(n,it%3u==0u);
  load_light();load_normals(n);
  olight_t o;const bool ok=oracle_prepare(&o,h_l,h_r,(const int16_t(*)[3])h_basis,h_ambient,h_color,0);
  for(int k=0;k<2;k++){
   for(unsigned i=0;i<4*n;i++)put8("lit",i,0xA5);
   struct cost c=call(k?"plain_light":"fast_light",n,0,0,0);
   if(c.result!=(u32)ok)bad(k?"plain_light accept":"fast_light accept",it);
   if(!ok)continue;
   if(!k)for(unsigned j=0;j<3;j++)if((int32_t)get32("light",4*j)!=o.light[j]||(int32_t)get32("light",12+4*j)!=o.reflection[j])bad("prepared light",it);
   for(unsigned i=0;i<n;i++){uint8_t w[4];oracle_normal(w,h_normals+4*i,&o,h_lut,0);
    for(unsigned b=0;b<4;b++)if(get8("lit",4*i+b)!=w[b])bad(k?"plain_light":"fast_light",it);}
  }
  if(ok)done+=n;
 }
 return done;
}
static uint64_t verify_gouraud(uint64_t iterations) {
 uint64_t done=0;
 for(uint64_t it=0;it<iterations;it++){
  for(int i=0;i<64*3;i++)h_bases[i]=rand_u8();for(int k=0;k<3;k++)h_fog[k]=rand_u8();
  for(int i=0;i<MAXN*4;i++)h_lit[i]=rand_u8();
  const unsigned n=it%4u==0u?MAXP:1u+rnd()%MAXP;
  for(unsigned i=0;i<n;i++)h_pairs[i]=(sh2z_shade_pair_t){(uint16_t)(rnd()%64u),(uint16_t)(rnd()%MAXN)};
  load_bases();load_lit(MAXN);load_pairs("pairs",h_pairs,n);
  uint32_t ws=0;static uint16_t want[MAXP];
  for(unsigned i=0;i<n;i++)want[i]=oracle_gouraud(h_bases+3*h_pairs[i].base,h_lit+4*h_pairs[i].light,h_fog,&ws,0);
  for(int k=0;k<2;k++){
   struct cost c=call(k?"plain_gouraud":"fast_gouraud",n,0,0,0);if(c.result!=1)bad("gouraud accept",it);
   for(unsigned i=0;i<n;i++)if(get16("gouraud",2*i)!=want[i])bad(k?"plain_gouraud":"fast_gouraud",it);
   if(get32("saturated",0)!=ws)bad("gouraud saturation count",it);
  }
  done+=n;
 }
 return done;
}
static uint64_t verify_cull(uint64_t iterations) {
 uint64_t done=0;
 for(uint64_t it=0;it<iterations;it++){
  const unsigned f=1u+rnd()%MAXF;rand_mesh(f,it%2u==0u);load_planes(f);
  for(int u=0;u<2;u++){
   static uint16_t want[MAXF];unsigned w=0;
   for(unsigned i=0;i<f;i++){unsigned id=u?h_order[i]:i;if(oracle_keep(&h_planes[id],h_eye,h_err,0))want[w++]=(uint16_t)id;}
   for(int k=0;k<2;k++){
    struct cost c=call(k?"plain_cull":"fast_cull",f,(u32)u,0,0);if(c.result!=1)bad("cull accept",it);
    if(get32("kept_count",0)!=w)bad(k?"plain_cull count":"fast_cull count",it);
    for(unsigned i=0;i<w;i++)if(get16("kept",2*i)!=want[i])bad(k?"plain_cull":"fast_cull",it);
   }
  }
  done+=f;
 }
 return done;
}
static uint64_t verify_fog(uint64_t iterations,uint64_t *tables) {
 uint64_t done=0;*tables=0;
 for(uint64_t it=0;it<iterations;it++){
  const bool persp=it%3u!=2u;rand_fogp(persp);load_fogp();
  uint32_t want[256];oracle_fog_table(want,h_fogp[0],h_fogp[1],h_fogp[2],h_fogp[3],h_fogp[4]);
  if(call("fast_fog_build",0,0,0,0).result!=1)bad("fog build accept",it);
  if(call("plain_fog_build",0,0,0,0).result!=1)bad("plain fog build",it);
  for(unsigned i=0;i<256;i++)if(get32("fog_table",4*i)!=want[i]||get32("plain_depth",4*i)!=want[i])bad("fog thresholds",it);
  const unsigned shift=get8("fog_table",1024+257);
  for(unsigned b=0;b<256;b++){uint32_t z=(uint32_t)b<<shift;if(z>0x7fffffffu)z=0x7fffffffu;
   if(get8("fog_table",1024+b)!=oracle_fade(want,(int32_t)z)&&((uint64_t)b<<shift)<=0x7fffffffu)bad("fog bucket",it);}
  if(get8("fog_table",1024+256)!=255)bad("fog bucket end",it);
  ++*tables;
  const unsigned n=it%4u==0u?MAXP:1u+rnd()%MAXP;
  for(unsigned i=0;i<n;i++)h_depths[i]=rand_depth(want,persp&&it%2u==0u);
  load_depths(n);
  for(int k=0;k<2;k++){
   for(unsigned i=0;i<n;i++)put8("fades",i,0xA5);
   if(call(k?"plain_fog":"fast_fog",n,0,0,0).result!=1)bad("fog accept",it);
   for(unsigned i=0;i<n;i++)if(get8("fades",i)!=oracle_fade(want,h_depths[i]))bad(k?"plain_fog":"fast_fog",it);
  }
  done+=n;
 }
 return done;
}
static uint64_t verify_fade(uint64_t iterations) {
 uint64_t done=0;
 for(uint64_t it=0;it<iterations;it++){
  const unsigned n=it%4u==0u?MAXP:1u+rnd()%MAXP;
  for(unsigned i=0;i<n;i++)h_fades[i]=(uint8_t)rnd();
  for(int k=0;k<3;k++){h_slope[k]=(int16_t)(rnd()%4u==0u?(rnd()%2u?127:-127):range(-31,31));
   h_bias[k]=(int16_t)(rnd()%4u==0u?(rnd()%2u?32767:-32767):range(0,255)*range(-31,31)+range(-1,1));
   h_near[k]=rand_u8();h_far[k]=rand_u8();}
  load_fades(n);load_fade_params();
  for(int k=0;k<4;k++){
   const char*e=(const char*[]){"fast_offset","plain_offset","fast_mix","plain_mix"}[k];
   if(call(e,n,0,0,0).result!=1)bad("fade accept",it);
   for(unsigned i=0;i<n;i++){const uint16_t w=k<2?oracle_offset(h_fades[i],h_slope,h_bias,0):oracle_mix(h_fades[i],h_near,h_far,0);
    if(get16("colours",2*i)!=w)bad(e,it);}
  }
  done+=n;
 }
 return done;
}

/* ---- one vehicle frame ---- */
enum {FRAME_F=112,FRAME_NN=128,FRAME_P=22};
static uint16_t h_face_palette[MAXF],h_corner_normal[MAXF*4],h_corner_pair[MAXF*4];static sh2z_shade_pair_t h_unique[MAXP];static unsigned h_unique_n;
static void rand_frame(bool realistic) {
 rand_light_inputs(realistic);rand_normals(FRAME_NN,realistic);rand_mesh(FRAME_F,true);
 for(int i=0;i<64*3;i++)h_bases[i]=rand_u8();for(int k=0;k<3;k++)h_fog[k]=rand_u8();
 h_unique_n=0;
 for(unsigned i=0;i<FRAME_F;i++){h_face_palette[i]=(uint16_t)(rnd()%FRAME_P);
  /* corners share normals with neighbours: draw from a small window */
  const unsigned base=rnd()%FRAME_NN;
  for(unsigned j=0;j<4;j++){h_corner_normal[4*i+j]=(uint16_t)((base+rnd()%6u)%FRAME_NN);
   unsigned u=0;while(u<h_unique_n&&!(h_unique[u].base==h_face_palette[i]&&h_unique[u].light==h_corner_normal[4*i+j]))u++;
   if(u==h_unique_n)h_unique[h_unique_n++]=(sh2z_shade_pair_t){h_face_palette[i],h_corner_normal[4*i+j]};
   h_corner_pair[4*i+j]=(uint16_t)u;}
 }
 load_light();load_normals(FRAME_NN);load_planes(FRAME_F);load_bases();
 for(unsigned i=0;i<FRAME_F;i++){put16("face_palette",2*i,h_face_palette[i]);
  for(unsigned j=0;j<4;j++){put16("corner_normal",2*(4*i+j),h_corner_normal[4*i+j]);put16("corner_pair",2*(4*i+j),h_corner_pair[4*i+j]);}}
 load_pairs("unique_pairs",h_unique,h_unique_n);
}
static uint64_t verify_frame(uint64_t iterations) {
 uint64_t done=0;
 for(uint64_t it=0;it<iterations;it++){
  rand_frame(it%2u==0u);
  olight_t o;if(!oracle_prepare(&o,h_l,h_r,(const int16_t(*)[3])h_basis,h_ambient,h_color,0)){it--;continue;}
  static uint8_t ol[FRAME_NN*4];for(unsigned i=0;i<FRAME_NN;i++)oracle_normal(ol+4*i,h_normals+4*i,&o,h_lut,0);
  static uint16_t want[MAXF*4];unsigned w=0;uint32_t ws=0,wu=0;
  for(unsigned i=0;i<FRAME_F;i++){unsigned id=h_order[i];if(!oracle_keep(&h_planes[id],h_eye,h_err,0))continue;
   for(unsigned j=0;j<4;j++)want[4*w+j]=oracle_gouraud(h_bases+3*h_face_palette[id],ol+4*h_corner_normal[4*id+j],h_fog,&ws,0);w++;}
  for(unsigned u=0;u<h_unique_n;u++)(void)oracle_gouraud(h_bases+3*h_unique[u].base,ol+4*h_unique[u].light,h_fog,&wu,0);
  for(int k=0;k<3;k++){
   const char*e=(const char*[]){"plain_frame","fast_frame","fast_frame_baked"}[k];
   for(unsigned i=0;i<4*FRAME_F;i++)put16("frame_out",2*i,0xA5A5);
   if(call(e,FRAME_F,FRAME_NN,h_unique_n,0).result!=1)bad("frame accept",it);
   if(get32("kept_count",0)!=w)bad("frame kept count",it);
   for(unsigned i=0;i<4*w;i++)if(get16("frame_out",2*i)!=want[i])bad(e,it);
   if(get32("saturated",0)!=(k==2?wu:ws))bad("frame saturation",it);
  }
  done+=w*4;
 }
 return done;
}

/* ---- cost ---- */
enum {CORPORA=64};
static unsigned cyc[4][CORPORA];static uint64_t ins[4];
static void row(const char *name,const char *unit_name,unsigned elements,const char *const *entries,unsigned ne,
                void (*setup)(unsigned,unsigned),u32 a1,u32 a2,u32 a3,bool *first) {
 if(ne>4)bad("too many entries",ne);
 for(unsigned e=0;e<ne;e++)ins[e]=0;
 rng=UINT32_C(0xc0571234)+elements;
 uint64_t kept_total=0,unique_total=0;
 for(unsigned c=0;c<CORPORA;c++){setup(elements,c);
  const u32 third=a3==UINT32_MAX?h_unique_n:a3;
  for(unsigned e=0;e<ne;e++){struct cost r=call(entries[e],a1,a2,third,0);if(r.result!=1)bad(entries[e],c);cyc[e][c]=r.cycles;ins[e]+=r.instructions;}
  kept_total+=get32("kept_count",0);unique_total+=h_unique_n;}
 if(!*first)putchar(',');*first=false;
 printf("\"%s\":{\"%s\":%u,\"corpora\":%u",name,unit_name,elements,(unsigned)CORPORA);
 if(a3==UINT32_MAX)printf(",\"meanKeptFaces\":%.1f,\"meanUniquePairs\":%.1f",(double)kept_total/CORPORA,(double)unique_total/CORPORA);
 for(unsigned e=0;e<ne;e++){printf(",\"%s\":",entries[e]);print_stats(summarize(cyc[e],CORPORA,ins[e]));}
 printf("}");
}
static void setup_light(unsigned n,unsigned c){(void)c;rand_light_inputs(true);
 rand_normals(n,true);load_light();load_normals(n);call("fast_light",0,0,0,0);}
static void setup_gouraud(unsigned n,unsigned c){(void)c;for(int i=0;i<64*3;i++)h_bases[i]=(uint8_t)rnd();for(int k=0;k<3;k++)h_fog[k]=(uint8_t)rnd();
 for(int i=0;i<MAXN*4;i++)h_lit[i]=(uint8_t)rnd();for(unsigned i=0;i<MAXN;i++)h_lit[4*i+3]=(uint8_t)range(0,255);
 for(unsigned i=0;i<n;i++)h_pairs[i]=(sh2z_shade_pair_t){(uint16_t)(rnd()%22u),(uint16_t)(rnd()%128u)};
 load_bases();load_lit(MAXN);load_pairs("pairs",h_pairs,n);}
static void setup_cull(unsigned n,unsigned c){(void)c;rand_mesh(n,true);load_planes(n);}
static void setup_fog_uniform(unsigned n,unsigned c){(void)c;rand_fogp(true);load_fogp();call("fast_fog_build",0,0,0,0);
 uint32_t d[256];oracle_fog_table(d,h_fogp[0],h_fogp[1],h_fogp[2],h_fogp[3],h_fogp[4]);for(unsigned i=0;i<n;i++)h_depths[i]=rand_depth(d,true);load_depths(n);}
static void setup_fog_near(unsigned n,unsigned c){(void)c;rand_fogp(true);load_fogp();call("fast_fog_build",0,0,0,0);
 for(unsigned i=0;i<n;i++){const double t=unit();h_depths[i]=(int32_t)lround((double)near_z_used*pow((double)h_fogp[4]/near_z_used,t));}load_depths(n);}
static void setup_fog_build(unsigned n,unsigned c){(void)n;(void)c;rand_fogp(true);load_fogp();}
static void setup_fade(unsigned n,unsigned c){(void)c;for(unsigned i=0;i<n;i++)h_fades[i]=(uint8_t)rnd();
 for(int k=0;k<3;k++){h_slope[k]=(int16_t)range(-31,31);h_bias[k]=(int16_t)(range(0,255)*range(-31,31));h_near[k]=(uint8_t)rnd();h_far[k]=(uint8_t)rnd();}
 load_fades(n);load_fade_params();}
static void setup_frame(unsigned n,unsigned c){(void)n;(void)c;rand_frame(true);}

int main(int argc,char**argv) {
 if(argc<3){fprintf(stderr,"usage: shade_harness model.elf verify-scale\n");return 2;}
 const uint64_t scale=strtoull(argv[2],0,10);
 load_elf_symbols(argv[1]);load_elf(argv[1]);
 cpu.model=SHMT_SH2;cpu.MappedMemoryReadByte=rb;cpu.MappedMemoryReadWord=rw;cpu.MappedMemoryReadLong=rl;cpu.MappedMemoryWriteByte=wb;cpu.MappedMemoryWriteWord=ww;cpu.MappedMemoryWriteLong=wl;
 for(unsigned i=0;i<65536;i++)cpu.opcodes[i]=decode(SHMT_SH2,(u16)i);for(unsigned i=0;i<256;i++)cpu.fetchlist[i]=fetch;
 uint64_t tables=0;
 const uint64_t vl=verify_light(scale),vg=verify_gouraud(scale),vc=verify_cull(scale),vf=verify_fog(scale,&tables),vd=verify_fade(scale),vv=verify_frame(scale/4+1);
 printf("{\"schema\":1,\"kind\":\"Yabause isolated SH2 interpreter flat-RAM model, not Saturn timing\",");
 printf("\"verified\":{\"normals\":%"PRIu64",\"gouraudPairs\":%"PRIu64",\"cullFaces\":%"PRIu64",\"fogTables\":%"PRIu64",\"fogLookups\":%"PRIu64",\"fadeColours\":%"PRIu64",\"frameCorners\":%"PRIu64",\"entries\":\"every fast_ and plain_ result against tests/shade_oracle.h\"},",
  vl,vg,vc,tables,vf,vd*2,vv);
 bool first=true;printf("\"rows\":{");
 static const char *const light[]={"fast_light","fast_light_normals","plain_light"};
 row("light_1","normals",1,light,3,setup_light,1,0,0,&first);
 row("light_128","normals",128,light,3,setup_light,128,0,0,&first);
 row("light_256","normals",256,light,3,setup_light,256,0,0,&first);
 static const char *const gour[]={"fast_gouraud","plain_gouraud"};
 row("gouraud_1","pairs",1,gour,2,setup_gouraud,1,0,0,&first);
 row("gouraud_448","pairs",448,gour,2,setup_gouraud,448,0,0,&first);
 static const char *const cull[]={"fast_cull","plain_cull"};
 row("cull_1","faces",1,cull,2,setup_cull,1,1,0,&first);
 row("cull_112","faces",112,cull,2,setup_cull,112,1,0,&first);
 row("cull_256","faces",256,cull,2,setup_cull,256,1,0,&first);
 static const char *const build[]={"fast_fog_build","plain_fog_build"};
 row("fog_build","tables",1,build,2,setup_fog_build,0,0,0,&first);
 static const char *const fog[]={"fast_fog","plain_fog"};
 row("fog_1","depths",1,fog,2,setup_fog_uniform,1,0,0,&first);
 row("fog_1024_uniform","depths",1024,fog,2,setup_fog_uniform,1024,0,0,&first);
 row("fog_1024_near_dense","depths",1024,fog,2,setup_fog_near,1024,0,0,&first);
 static const char *const fade[]={"fast_offset","plain_offset","fast_mix","plain_mix"};
 row("fade_1","fades",1,fade,4,setup_fade,1,0,0,&first);
 row("fade_1024","fades",1024,fade,4,setup_fade,1024,0,0,&first);
 static const char *const frame[]={"plain_frame","fast_frame","fast_frame_baked"};
 row("vehicle_frame_112_faces_128_normals","faces",FRAME_F,frame,3,setup_frame,FRAME_F,FRAME_NN,UINT32_MAX,&first);
 printf("}}\n");
 return 0;
}
