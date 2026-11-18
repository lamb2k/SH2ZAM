/* SPDX-License-Identifier: GPL-2.0-or-later
 * Executed-instruction checks and cycle model for the vehicle-kernels-1
 * kernels (screen facing, indexed lighting, approximate Gouraud, indexed view
 * batch). Runs the actual big-endian SH-2 object code in Yabause's unmodified
 * interpreter, like harness.c, whose loader, call wrapper and divider model it
 * reuses. Flat RAM, no caches or wait states: MODELLED cycles, not Saturn
 * timing. Copyright (c) 2026 SH2ZAM Contributors. This optional model harness
 * is licensed separately from SH2ZAM's MIT core; see LICENSE.
 *
 * usage: vehicle_harness model.elf verify-scale > results.json
 * Every result of every fast_ and port_ entry is checked before any timing is
 * printed: the shade kernels against the independent oracles in
 * tests/vehicle_oracle.h and tests/shade_oracle.h, the indexed view batch
 * against the host build of sh2z_view_batch (its definition). */
#define main sh2_model_main
#include "harness.c"
#undef main
#include "../../tests/vehicle_oracle.h"
#include <stddef.h>

static void load_elf_symbols(const char *path) {
 FILE*f=fopen(path,"rb");if(!f){perror(path);exit(2);}
 fseek(f,0,SEEK_END);long length=ftell(f);rewind(f);
 unsigned char*b=malloc((size_t)length);if(!b||fread(b,1,(size_t)length,f)!=(size_t)length)exit(2);fclose(f);
 u32 shoff=be32(b+32);u16 shentsize=be16(b+46),shnum=be16(b+48);
 for(unsigned i=0;i<shnum;i++){
  const unsigned char*sh=b+shoff+(size_t)i*shentsize;
  if(be32(sh+4)!=2)continue;
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
static struct cost call(const char *name,u32 a,u32 b,u32 c,u32 d) {char t[140];snprintf(t,sizeof t,"_%s",name);return invoke(t,a,b,c,d);}
static u32 symc(const char *name) {char t[140];snprintf(t,sizeof t,"_%s",name);return sym(t);}
static void bad(const char *what,uint64_t it) {fprintf(stderr,"vehicle model mismatch: %s (case %"PRIu64")\n",what,it);exit(3);}

static uint32_t rng=UINT32_C(0x7e41c1e5);
static uint32_t rnd(void) {rng^=rng<<13;rng^=rng>>17;rng^=rng<<5;return rng;}
static int32_t range(int32_t lo,int32_t hi) {return (int32_t)((int64_t)lo+(int64_t)(rnd()%(uint64_t)((int64_t)hi-lo+1)));}
static double unit(void) {return (double)rnd()/4294967296.0;}
static uint8_t rand_u8(void){static const uint8_t e[]={0,1,15,16,127,128,240,254,255};return rnd()%3u==0u?e[rnd()%9u]:(uint8_t)rnd();}

static void put8(const char *s,u32 off,uint8_t v){wb(&cpu,symc(s)+off,v);}
static void put16(const char *s,u32 off,uint16_t v){ww(&cpu,symc(s)+off,v);}
static void put32(const char *s,u32 off,uint32_t v){wl(&cpu,symc(s)+off,v);}
static uint8_t get8(const char *s,u32 off){return rb(&cpu,symc(s)+off);}
static uint16_t get16(const char *s,u32 off){return rw(&cpu,symc(s)+off);}
static uint32_t get32(const char *s,u32 off){return rl(&cpu,symc(s)+off);}

struct stats {double mean,median;unsigned min,max;double instr;};
static int cmp_u(const void*a,const void*b){unsigned x=*(const unsigned*)a,y=*(const unsigned*)b;return x<y?-1:x>y;}
static struct stats summarize(unsigned *c,size_t n,uint64_t instr) {
 qsort(c,n,sizeof *c,cmp_u);double sum=0;for(size_t i=0;i<n;i++)sum+=c[i];
 return (struct stats){sum/(double)n,n&1?c[n/2]:(c[n/2-1]+c[n/2])/2.0,c[0],c[n-1],(double)instr/(double)n};
}
static void print_stats(struct stats s) {
 printf("{\"meanCycles\":%.2f,\"medianCycles\":%.1f,\"minCycles\":%u,\"maxCycles\":%u,\"meanInstructions\":%.2f}",s.mean,s.median,s.min,s.max,s.instr);
}

/* ---- view: host definition and target layout ---- */
enum {MAXV=256,MAXF=256,MAXN=256,MAXP=1024};
static sh2z_view_t h_view;static sh2z_pos16_t h_pos[MAXV];static uint8_t h_vindex[MAXV*2];
static void load_view(void) {
 for(unsigned i=0;i<3;i++)for(unsigned j=0;j<3;j++)put16("view",(u32)(offsetof(sh2z_view_t,r)+2*(3*i+j)),(uint16_t)h_view.r[i][j]);
 for(unsigned i=0;i<3;i++)put32("view",(u32)(offsetof(sh2z_view_t,t)+4*i),(uint32_t)h_view.t[i]);
 put32("view",(u32)offsetof(sh2z_view_t,focal_x),(uint32_t)h_view.focal_x);put32("view",(u32)offsetof(sh2z_view_t,focal_y),(uint32_t)h_view.focal_y);
 put16("view",(u32)offsetof(sh2z_view_t,center_x),(uint16_t)h_view.center_x);put16("view",(u32)offsetof(sh2z_view_t,center_y),(uint16_t)h_view.center_y);
 put32("view",(u32)offsetof(sh2z_view_t,near_z),(uint32_t)h_view.near_z);put32("view",(u32)offsetof(sh2z_view_t,far_z),(uint32_t)h_view.far_z);
 for(unsigned i=0;i<4;i++)put16("view",(u32)(offsetof(sh2z_view_t,clip)+2*i),(uint16_t)h_view.clip[i]);
}
static void load_positions(unsigned n){for(unsigned i=0;i<n;i++){put16("positions",6*i,(uint16_t)h_pos[i].x);put16("positions",6*i+2,(uint16_t)h_pos[i].y);put16("positions",6*i+4,(uint16_t)h_pos[i].z);}}
static void get_vertex(const char *s,unsigned i,sh2z_view_vertex_t *v) {
 v->x=(int32_t)get32(s,20*i);v->y=(int32_t)get32(s,20*i+4);v->z=(int32_t)get32(s,20*i+8);
 v->sx=(int16_t)get16(s,20*i+12);v->sy=(int16_t)get16(s,20*i+14);v->outcode=get8(s,20*i+16);
 for(int k=0;k<3;k++)v->reserved[k]=get8(s,20*i+17+k);
}
static void put_vertex(const char *s,unsigned i,const sh2z_view_vertex_t *v) {
 put32(s,20*i,(uint32_t)v->x);put32(s,20*i+4,(uint32_t)v->y);put32(s,20*i+8,(uint32_t)v->z);
 put16(s,20*i+12,(uint16_t)v->sx);put16(s,20*i+14,(uint16_t)v->sy);put8(s,20*i+16,v->outcode);
 for(int k=0;k<3;k++)put8(s,20*i+17+k,v->reserved[k]);
}
static bool same_vertex(const sh2z_view_vertex_t *a,const sh2z_view_vertex_t *b) {
 return a->x==b->x&&a->y==b->y&&a->z==b->z&&a->sx==b->sx&&a->sy==b->sy&&a->outcode==b->outcode&&!a->reserved[0]&&!a->reserved[1]&&!a->reserved[2]&&!b->reserved[0];
}
static int16_t q14(double x){const double s=floor(x*16384.0+0.5);return (int16_t)(s>32767?32767:s<-32768?-32768:s);}
/* A camera looking at a vehicle-sized model near the origin from dist units
 * (realistic) or a random camera anywhere (adversarial). */
static void rand_view(bool realistic,double dist) {
 const double a=unit()*6.283185307179586,b=(unit()-0.5)*(realistic?1.2:3.0);
 const double r[3][3]={{cos(a),0,-sin(a)},{sin(a)*sin(b),cos(b),cos(a)*sin(b)},{sin(a)*cos(b),-sin(b),cos(a)*cos(b)}};
 sh2z_view_t v;memset(&v,0,sizeof v);
 for(int i=0;i<3;i++)for(int j=0;j<3;j++)v.r[i][j]=q14(r[i][j]);
 if(realistic){v.focal_x=range(200,320)*65536;v.focal_y=v.focal_x;v.center_x=176;v.center_y=112;
  v.near_z=16*16384;v.far_z=30000*16384;v.clip[0]=0;v.clip[1]=0;v.clip[2]=351;v.clip[3]=223;}
 else{v.focal_x=range(64,1024)*65536+range(0,65535);v.focal_y=range(64,1024)*65536+range(0,65535);
  v.center_x=(int16_t)range(0,352);v.center_y=(int16_t)range(0,256);v.near_z=range(1,16*16384);v.far_z=v.near_z+range(1,INT32_MAX-v.near_z);
  v.clip[0]=(int16_t)range(-64,160);v.clip[2]=(int16_t)range(v.clip[0],400);v.clip[1]=(int16_t)range(-64,120);v.clip[3]=(int16_t)range(v.clip[1],300);}
 if(!sh2z_view_init(&h_view,&v))bad("view init",0);
 /* eye at dist along the view axis (third row is the forward direction) */
 sh2z_vec3_t e;
 if(realistic){e.x=(int32_t)lround(-r[2][0]*dist*65536);e.y=(int32_t)lround(-r[2][1]*dist*65536);e.z=(int32_t)lround(-r[2][2]*dist*65536);}
 else{e.x=range(-3000,3000)*65536;e.y=range(-3000,3000)*65536;e.z=range(-3000,3000)*65536;}
 if(!sh2z_view_place(&h_view,e))bad("view place",0);
}
/* A closed vehicle-like mesh: a lat-long ellipsoid of quads (rows x cols), a
 * triangle fan cap at each pole stored as quads with a repeated corner. */
static unsigned mesh_faces,mesh_vertices;static uint8_t h_corners[MAXF][4];
static void rand_mesh(unsigned rows,unsigned cols) {
 const double ax=range(500,1000),ay=range(150,350),az=range(250,500);
 mesh_vertices=0;
 for(unsigned i=0;i<=rows;i++)for(unsigned j=0;j<cols;j++){
  if((i==0||i==rows)&&j)continue;
  const double th=3.141592653589793*i/rows,ph=6.283185307179586*j/cols;
  h_pos[mesh_vertices++]=(sh2z_pos16_t){(int16_t)lround(ax*sin(th)*cos(ph)),(int16_t)lround(ay*cos(th)),(int16_t)lround(az*sin(th)*sin(ph))};
 }
 const unsigned top=0,bottom=mesh_vertices-1;
 #define RING(i,j) (1u+((i)-1u)*cols+((j)%cols))
 mesh_faces=0;
 for(unsigned j=0;j<cols;j++){uint8_t *c=h_corners[mesh_faces++];c[0]=(uint8_t)top;c[1]=(uint8_t)RING(1,j+1);c[2]=(uint8_t)RING(1,j);c[3]=c[2];}
 for(unsigned i=1;i+1<rows;i++)for(unsigned j=0;j<cols;j++){uint8_t *c=h_corners[mesh_faces++];
  c[0]=(uint8_t)RING(i,j);c[1]=(uint8_t)RING(i,j+1);c[2]=(uint8_t)RING(i+1,j+1);c[3]=(uint8_t)RING(i+1,j);}
 for(unsigned j=0;j<cols;j++){uint8_t *c=h_corners[mesh_faces++];c[0]=(uint8_t)bottom;c[1]=(uint8_t)RING(rows-1,j);c[2]=(uint8_t)RING(rows-1,j+1);c[3]=c[2];}
 #undef RING
}
static void load_corners(unsigned n){for(unsigned i=0;i<n;i++)for(unsigned j=0;j<4;j++)put8("corners",4*i+j,h_corners[i][j]);}

static uint64_t verify_view(uint64_t iterations) {
 uint64_t done=0;static sh2z_view_vertex_t want[MAXV];
 for(uint64_t it=0;it<iterations;it++){
  const bool real=it%2u==0u;rand_view(real,range(1500,20000));
  unsigned count;
  if(real&&it%4u==0u){rand_mesh(8,14);count=mesh_vertices;}
  else{count=1u+rnd()%MAXV;for(unsigned i=0;i<count;i++)h_pos[i]=(sh2z_pos16_t){(int16_t)(rnd()%3u?range(-1024,1024):range(INT16_MIN,INT16_MAX)),(int16_t)(rnd()%3u?range(-1024,1024):range(INT16_MIN,INT16_MAX)),(int16_t)(rnd()%3u?range(-1024,1024):range(INT16_MIN,INT16_MAX))};}
  const unsigned n=it%5u==0u?2u*MAXV:rnd()%(count+1u);
  for(unsigned k=0;k<n;k++)h_vindex[k]=(uint8_t)(rnd()%count);
  load_view();load_positions(count);for(unsigned k=0;k<n;k++)put8("vindex",k,h_vindex[k]);
  sh2z_view_vertex_t sentinel;memset(&sentinel,0x3C,sizeof sentinel);
  for(unsigned i=0;i<count;i++){for(u32 b=0;b<20;b++)put8("projected",20*i+b,0x3C);}
  if(call("fast_view_indexed",count,n,0,0).result!=1)bad("view indexed accept",it);
  if(call("port_view_all",count,0,0,0).result!=1)bad("view batch accept",it);
  if(!sh2z_view_batch(want,&h_view,h_pos,6,count))bad("host batch",it);
  static bool listed[MAXV];memset(listed,0,sizeof listed);for(unsigned k=0;k<n;k++)listed[h_vindex[k]]=true;
  for(unsigned i=0;i<count;i++){sh2z_view_vertex_t g,a;get_vertex("projected",i,&g);get_vertex("projected_all",i,&a);
   if(!same_vertex(&a,&want[i]))bad("target batch vs host definition",it);
   if(listed[i]){if(!same_vertex(&g,&want[i]))bad("indexed vertex",it);done++;}
   else for(u32 b=0;b<20;b++)if(get8("projected",20*i+b)!=0x3C)bad("unlisted vertex written",it);}
 }
 return done;
}

/* ---- facing ---- */
static sh2z_view_vertex_t h_vert[MAXV];static uint16_t h_faces[MAXF];
static void rand_vertices(bool realistic) {
 for(unsigned i=0;i<MAXV;i++){
  sh2z_view_vertex_t *a=&h_vert[i];memset(a,0,sizeof *a);a->z=range(1,1000000);
  if(rnd()%(realistic?64u:12u)==0u){a->outcode=SH2Z_OUT_NEAR;continue;}
  if(realistic){a->sx=(int16_t)range(-40,390);a->sy=(int16_t)range(-40,260);}
  else switch(rnd()%6u){case 0:a->sx=(int16_t)range(-1025,-1021);a->sy=(int16_t)range(-1025,1025);break;case 1:a->sx=(int16_t)range(1021,1025);a->sy=(int16_t)range(-1025,1025);break;
   case 2:a->sx=(int16_t)range(INT16_MIN,INT16_MAX);a->sy=(int16_t)range(INT16_MIN,INT16_MAX);break;default:a->sx=(int16_t)range(-1023,1023);a->sy=(int16_t)range(-1023,1023);}
  unsigned code=rnd()%10u==0u?SH2Z_OUT_FAR:0u;
  if(a->sx<h_view.clip[0])code|=SH2Z_OUT_LEFT;if(a->sx>h_view.clip[2])code|=SH2Z_OUT_RIGHT;
  if(a->sy<h_view.clip[1])code|=SH2Z_OUT_TOP;if(a->sy>h_view.clip[3])code|=SH2Z_OUT_BOTTOM;
  a->outcode=(uint8_t)code;
 }
}
static uint64_t verify_facing(uint64_t iterations,uint64_t *kept_total) {
 uint64_t done=0;*kept_total=0;
 for(uint64_t it=0;it<iterations;it++){
  sh2z_view_t v;memset(&v,0,sizeof v);v.focal_x=v.focal_y=65536;v.near_z=1;v.far_z=2;
  if(it%3u==0u){v.clip[0]=(int16_t)range(-2000,0);v.clip[2]=(int16_t)range(300,2000);v.clip[1]=(int16_t)range(-2000,0);v.clip[3]=(int16_t)range(200,2000);}
  else{v.clip[0]=0;v.clip[1]=0;v.clip[2]=(int16_t)range(200,1023);v.clip[3]=(int16_t)range(200,1023);}
  h_view=v;load_view();
  unsigned n;
  if(it%4u==3u){ /* a projected mesh */
   rand_view(true,range(1500,9000));load_view();rand_mesh(8,14);load_positions(mesh_vertices);
   if(call("port_view_all",mesh_vertices,0,0,0).result!=1)bad("mesh projection",it);
   for(unsigned i=0;i<mesh_vertices;i++){get_vertex("projected_all",i,&h_vert[i]);put_vertex("projected",i,&h_vert[i]);}
   load_corners(mesh_faces);n=mesh_faces;for(unsigned i=0;i<n;i++)h_faces[i]=(uint16_t)i;
   for(unsigned i=0;i<MAXF;i++)for(int j=0;j<4;j++)if(i>=mesh_faces)h_corners[i][j]=0;
  } else {
   rand_vertices(it%2u==0u);for(unsigned i=0;i<MAXV;i++)put_vertex("projected",i,&h_vert[i]);
   for(unsigned i=0;i<MAXF;i++){for(int j=0;j<4;j++)h_corners[i][j]=(uint8_t)(rnd()%MAXV);if(rnd()%5u==0u)h_corners[i][3]=h_corners[i][2];}
   load_corners(MAXF);n=1u+rnd()%MAXF;for(unsigned i=0;i<n;i++)h_faces[i]=(uint16_t)(rnd()%MAXF);
  }
  for(unsigned i=0;i<n;i++)put16("faces",2*i,h_faces[i]);
  const int sign=rnd()%2u?1:-1;
  static uint16_t want[MAXF];unsigned w=0;
  for(unsigned i=0;i<n;i++){const uint8_t *c=h_corners[h_faces[i]];const sh2z_view_vertex_t *q[4]={h_vert+c[0],h_vert+c[1],h_vert+c[2],h_vert+c[3]};if(ovf_keep(q,sign,0))want[w++]=h_faces[i];}
  for(int k=0;k<2;k++){
   for(unsigned i=0;i<MAXF;i++)put16("kept",2*i,0xA5A5);
   if(call(k?"port_facing":"fast_facing",n,(u32)sign,0,0).result!=1)bad("facing accept",it);
   if(get32("kept_count",0)!=w)bad(k?"port facing count":"facing count",it);
   for(unsigned i=0;i<w;i++)if(get16("kept",2*i)!=want[i])bad(k?"port facing":"facing",it);
   if(w<MAXF&&get16("kept",2*w)!=0xA5A5)bad("facing wrote past the count",it);
  }
  done+=n;*kept_total+=w;
 }
 return done;
}

/* ---- lighting ---- */
static sh2z_vec3_t h_l,h_r;static int16_t h_basis[3][3];static uint8_t h_ambient[3],h_color[3],h_lut[256];
static int8_t h_normals[MAXN*4];static uint8_t h_nindex[MAXN*2];
static void rand_rotation(int16_t b[3][3]) {
 const double a=unit()*6.283185307179586,c=unit()*6.283185307179586,d=unit()*3.141592653589793;
 const double ca=cos(a),sa=sin(a),cc=cos(c),sc=sin(c),cd=cos(d),sd=sin(d);
 const double m[3][3]={{ca*cc-sa*cd*sc,-ca*sc-sa*cd*cc,sa*sd},{sa*cc+ca*cd*sc,-sa*sc+ca*cd*cc,-ca*sd},{sd*sc,sd*cc,cd}};
 for(int i=0;i<3;i++)for(int j=0;j<3;j++)b[i][j]=(int16_t)lround(m[i][j]*16384.0);
}
static void rand_light(bool realistic) {
 if(realistic){h_l=(sh2z_vec3_t){range(-127,127)*65536,range(-127,127)*65536,range(1,127)*65536};
  h_r=(sh2z_vec3_t){range(-32767,32767)*65536,range(1,32767)*65536,range(-32767,32767)*65536};rand_rotation(h_basis);
  for(int k=0;k<3;k++){h_ambient[k]=(uint8_t)range(16,96);h_color[k]=(uint8_t)range(96,255);}}
 else{h_l=(sh2z_vec3_t){range(INT32_MIN/2,INT32_MAX/2),range(-1,1),(int32_t)rnd()};h_r=(sh2z_vec3_t){(int32_t)rnd(),range(-128,127)*65536,(int32_t)rnd()};
  if(rnd()%2u)rand_rotation(h_basis);else for(int i=0;i<3;i++)for(int j=0;j<3;j++)h_basis[i][j]=(int16_t)range(-16384,16384);
  for(int k=0;k<3;k++){h_ambient[k]=rand_u8();h_color[k]=rand_u8();}}
 for(int i=0;i<256;i++)h_lut[i]=(uint8_t)rnd();
}
static void rand_normals(unsigned n,bool realistic) {
 for(unsigned i=0;i<n;i++){
  if(realistic){double x,y,z,l;do{x=unit()*2-1;y=unit()*2-1;z=unit()*2-1;l=sqrt(x*x+y*y+z*z);}while(l<0.1||l>1);
   h_normals[4*i]=(int8_t)lround(x/l*127);h_normals[4*i+1]=(int8_t)lround(y/l*127);h_normals[4*i+2]=(int8_t)lround(z/l*127);}
  else for(int k=0;k<3;k++)h_normals[4*i+k]=(int8_t)(rnd()%5u==0u?(rnd()%2u?-128:127):range(-128,127));
  h_normals[4*i+3]=0;
 }
}
static void load_light(void) {
 const int32_t *lv=&h_l.x,*rv=&h_r.x;
 for(unsigned k=0;k<3;k++){put32("prep_light",4*k,(uint32_t)lv[k]);put32("prep_reflection",4*k,(uint32_t)rv[k]);
  put8("prep_ambient",k,h_ambient[k]);put8("prep_color",k,h_color[k]);for(unsigned j=0;j<3;j++)put16("prep_basis",2*(3*k+j),(uint16_t)h_basis[k][j]);}
 for(unsigned i=0;i<256;i++)put8("lut",i,h_lut[i]);
}
static bool prepared(olight_t *o) {
 if(!oracle_prepare(o,h_l,h_r,(const int16_t(*)[3])h_basis,h_ambient,h_color,0))return false;
 return call("fast_prepare",0,0,0,0).result==1;
}
static uint64_t verify_light(uint64_t iterations) {
 uint64_t done=0;
 for(uint64_t it=0;it<iterations;it++){
  rand_light(it%3u!=2u);load_light();olight_t o;if(!prepared(&o)){it--;continue;}
  if(it%7u==6u){for(int k=0;k<3;k++){o.light[k]=range(-49152,49152);o.reflection[k]=range(-49152,49152);
   put32("light",4*(u32)k,(uint32_t)o.light[k]);put32("light",12+4*(u32)k,(uint32_t)o.reflection[k]);}}
  const unsigned count=1u+rnd()%MAXN,n=it%4u==0u?2u*MAXN:rnd()%(count+1u);
  rand_normals(count,it%2u==0u);for(unsigned i=0;i<4*count;i++)put8("normals",i,(uint8_t)h_normals[i]);
  for(unsigned k=0;k<n;k++){h_nindex[k]=(uint8_t)(rnd()%count);put8("nindex",k,h_nindex[k]);}
  for(unsigned i=0;i<4*MAXN;i++)put8("lit",i,0x5A);
  if(call("fast_light_indexed",count,n,0,0).result!=1)bad("indexed light accept",it);
  static bool listed[MAXN];memset(listed,0,sizeof listed);for(unsigned k=0;k<n;k++)listed[h_nindex[k]]=true;
  for(unsigned i=0;i<MAXN;i++){
   if(i<count&&listed[i]){uint8_t w[4];oracle_normal(w,h_normals+4*i,&o,h_lut,0);for(unsigned b=0;b<4;b++)if(get8("lit",4*i+b)!=w[b])bad("indexed light",it);done++;}
   else for(unsigned b=0;b<4;b++)if(get8("lit",4*i+b)!=0x5A)bad("unlisted normal written",it);
  }
  if(call("port_light_all",count,0,0,0).result!=1)bad("light_normals accept",it);
  for(unsigned i=0;i<count;i++){uint8_t w[4];oracle_normal(w,h_normals+4*i,&o,h_lut,0);for(unsigned b=0;b<4;b++)if(get8("lit",4*i+b)!=w[b])bad("light_normals",it);}
 }
 return done;
}

/* ---- Gouraud ---- */
static uint8_t h_bases[64*4],h_fog[3],h_lit[MAXN*4];static sh2z_shade_pair_t h_pairs[MAXP];static unsigned h_bstride=3;
static void load_gouraud(unsigned n) {
 for(unsigned i=0;i<64*4;i++)put8("bases",i,h_bases[i]);for(int k=0;k<3;k++)put8("fog_rgb",(u32)k,h_fog[k]);
 for(unsigned i=0;i<MAXN*4;i++)put8("lit",i,h_lit[i]);put32("base_stride",0,h_bstride);
 for(unsigned i=0;i<n;i++){put16("pairs",4*i,h_pairs[i].base);put16("pairs",4*i+2,h_pairs[i].light);}
}
static uint64_t verify_gouraud(uint64_t iterations,uint64_t *channels_exact,uint64_t *channels) {
 uint64_t done=0;*channels_exact=*channels=0;
 for(uint64_t it=0;it<iterations;it++){
  h_bstride=3u+(unsigned)(it%2u);
  for(int i=0;i<64*4;i++)h_bases[i]=rand_u8();for(int k=0;k<3;k++)h_fog[k]=rand_u8();for(int i=0;i<MAXN*4;i++)h_lit[i]=rand_u8();
  const unsigned n=it%4u==0u?MAXP:1u+rnd()%MAXP,nb=h_bstride==3u?64u:64u;
  for(unsigned i=0;i<n;i++)h_pairs[i]=(sh2z_shade_pair_t){(uint16_t)(rnd()%(nb-1u)),(uint16_t)(rnd()%MAXN)};
  load_gouraud(n);
  uint32_t ws=0,es=0;static uint16_t want[MAXP],exact[MAXP];
  for(unsigned i=0;i<n;i++){want[i]=oracle_gouraud_approx(h_bases+h_bstride*h_pairs[i].base,h_lit+4*h_pairs[i].light,h_fog,&ws,0);
   exact[i]=oracle_gouraud(h_bases+h_bstride*h_pairs[i].base,h_lit+4*h_pairs[i].light,h_fog,&es,0);}
  for(unsigned i=0;i<MAXP;i++)put16("gouraud",2*i,0xA5A5);
  if(call("fast_gouraud_approx",n,0,0,0).result!=1)bad("approx accept",it);
  for(unsigned i=0;i<n;i++){const uint16_t g=get16("gouraud",2*i);if(g!=want[i])bad("approx gouraud",it);
   for(int k=0;k<3;k++){const int a=(g>>(5*k))&31,e=(exact[i]>>(5*k))&31;if(a!=e&&a!=e-1)bad("approx bound",it);(*channels)++;*channels_exact+=a==e;}}
  if(n<MAXP&&get16("gouraud",2*n)!=0xA5A5)bad("approx wrote past the count",it);
  if(get32("saturated",0)!=ws)bad("approx saturation",it);
  if(call("port_gouraud",n,0,0,0).result!=1)bad("exact accept",it);
  for(unsigned i=0;i<n;i++)if(get16("gouraud",2*i)!=exact[i])bad("exact gouraud",it);
  if(get32("saturated",0)!=es)bad("exact saturation",it);
  done+=n;
 }
 return done;
}

/* ---- cost ---- */
enum {CORPORA=64};
static unsigned cyc[4][CORPORA];static uint64_t ins[4];
static bool first=true;
typedef void (*setup_fn)(unsigned,unsigned);
static void row(const char *name,const char *unit_name,unsigned elements,const char *const *entries,const u32 (*args)[2],unsigned ne,setup_fn setup) {
 for(unsigned e=0;e<ne;e++)ins[e]=0;
 rng=UINT32_C(0xc0571234)+elements*7u+ne;
 uint64_t kept_total=0;
 for(unsigned c=0;c<CORPORA;c++){setup(elements,c);
  for(unsigned e=0;e<ne;e++){struct cost r=call(entries[e],args[e][0],args[e][1],0,0);if(r.result!=1)bad(entries[e],c);cyc[e][c]=r.cycles;ins[e]+=r.instructions;}
  kept_total+=get32("kept_count",0);}
 if(!first)putchar(',');first=false;
 printf("\"%s\":{\"%s\":%u,\"corpora\":%u",name,unit_name,elements,(unsigned)CORPORA);
 if(!strncmp(name,"facing",6))printf(",\"meanKept\":%.1f",(double)kept_total/CORPORA);
 for(unsigned e=0;e<ne;e++){printf(",\"%s\":",entries[e]);print_stats(summarize(cyc[e],CORPORA,ins[e]));}
 printf("}");
}
/* The caller's sizes: a 68-position, 133-normal, 112-face level of detail
 * seen from 1,500 to 9,000 units. */
enum {POS=68,NORM=133};
static unsigned view_listed;
static void setup_view(unsigned n,unsigned c){(void)c;rand_view(true,range(1500,9000));load_view();
 for(unsigned i=0;i<POS;i++)h_pos[i]=(sh2z_pos16_t){(int16_t)range(-1000,1000),(int16_t)range(-350,350),(int16_t)range(-500,500)};load_positions(POS);
 /* n distinct ascending indices out of POS */
 unsigned k=0;for(unsigned i=0;i<POS&&k<n;i++)if(rnd()%(POS-i)<n-k)put8("vindex",k++,(uint8_t)i);view_listed=k;}
static unsigned facing_n;
static void setup_facing(unsigned n,unsigned c){(void)c;rand_view(true,range(1500,9000));load_view();rand_mesh(9,14);
 load_positions(mesh_vertices);call("port_view_all",mesh_vertices,0,0,0);
 for(unsigned i=0;i<mesh_vertices;i++){sh2z_view_vertex_t v;get_vertex("projected_all",i,&v);put_vertex("projected",i,&v);}
 load_corners(mesh_faces);facing_n=n<mesh_faces?n:mesh_faces;
 for(unsigned i=0;i<facing_n;i++)put16("faces",2*i,(uint16_t)((i*7u)%mesh_faces));}
static void setup_light(unsigned n,unsigned c){(void)c;rand_light(true);load_light();call("fast_prepare",0,0,0,0);
 rand_normals(NORM,true);for(unsigned i=0;i<4*NORM;i++)put8("normals",i,(uint8_t)h_normals[i]);
 unsigned k=0;for(unsigned i=0;i<NORM&&k<n;i++)if(rnd()%(NORM-i)<n-k)put8("nindex",k++,(uint8_t)i);}
static void setup_gouraud(unsigned n,unsigned c){(void)c;h_bstride=3;
 for(int i=0;i<64*4;i++)h_bases[i]=(uint8_t)rnd();for(int k=0;k<3;k++)h_fog[k]=(uint8_t)rnd();
 for(int i=0;i<MAXN*4;i++)h_lit[i]=(uint8_t)rnd();for(unsigned i=0;i<MAXN;i++)h_lit[4*i+3]=(uint8_t)range(0,255);
 for(unsigned i=0;i<n;i++)h_pairs[i]=(sh2z_shade_pair_t){(uint16_t)(rnd()%22u),(uint16_t)(rnd()%NORM)};load_gouraud(n);}

int main(int argc,char**argv) {
 if(argc<3){fprintf(stderr,"usage: vehicle_harness model.elf verify-scale\n");return 2;}
 const uint64_t scale=strtoull(argv[2],0,10);
 load_elf_symbols(argv[1]);load_elf(argv[1]);
 cpu.model=SHMT_SH2;cpu.MappedMemoryReadByte=rb;cpu.MappedMemoryReadWord=rw;cpu.MappedMemoryReadLong=rl;cpu.MappedMemoryWriteByte=wb;cpu.MappedMemoryWriteWord=ww;cpu.MappedMemoryWriteLong=wl;
 for(unsigned i=0;i<65536;i++)cpu.opcodes[i]=decode(SHMT_SH2,(u16)i);for(unsigned i=0;i<256;i++)cpu.fetchlist[i]=fetch;
 divu_reset();
 uint64_t kept=0,cx=0,cn=0;
 const uint64_t vv=verify_view(scale/4u+1u),vf=verify_facing(scale,&kept),vl=verify_light(scale),vg=verify_gouraud(scale,&cx,&cn);
 printf("{\"schema\":1,\"kind\":\"Yabause isolated SH2 interpreter flat-RAM model, not Saturn timing\",");
 printf("\"verified\":{\"viewVertices\":%"PRIu64",\"facingFaces\":%"PRIu64",\"facingKept\":%"PRIu64",\"normals\":%"PRIu64",\"gouraudPairs\":%"PRIu64",\"gouraudChannels\":%"PRIu64",\"gouraudChannelsExact\":%"PRIu64",\"divisions\":%"PRIu64","
  "\"entries\":\"every fast_ and port_ result against tests/vehicle_oracle.h, tests/shade_oracle.h or the host sh2z_view_batch\"},",vv,vf,kept,vl,vg,cn,cx,divu_divisions);
 printf("\"rows\":{");
 static const char *const view_e[]={"fast_view_indexed","port_view_all"};
 const u32 va1[2][2]={{POS,1},{POS,0}},va60[2][2]={{POS,60},{POS,0}},va68[2][2]={{POS,POS},{POS,0}};
 row("view_indexed_1","listed",1,view_e,va1,2,setup_view);
 row("view_indexed_60","listed",60,view_e,va60,2,setup_view);
 row("view_indexed_68","listed",68,view_e,va68,2,setup_view);
 static const char *const facing_e[]={"fast_facing","port_facing"};
 const u32 fa1[2][2]={{1,(u32)-1},{1,(u32)-1}},fa97[2][2]={{97,(u32)-1},{97,(u32)-1}};
 row("facing_1","faces",1,facing_e,fa1,2,setup_facing);
 row("facing_97","faces",97,facing_e,fa97,2,setup_facing);
 static const char *const light_e[]={"fast_light_indexed","port_light_all"};
 const u32 la1[2][2]={{NORM,1},{NORM,0}},la70[2][2]={{NORM,70},{NORM,0}},la133[2][2]={{NORM,NORM},{NORM,0}};
 row("light_indexed_1","listed",1,light_e,la1,2,setup_light);
 row("light_indexed_70","listed",70,light_e,la70,2,setup_light);
 row("light_indexed_133","listed",133,light_e,la133,2,setup_light);
 static const char *const gour_e[]={"fast_gouraud_approx","port_gouraud"};
 const u32 ga1[2][2]={{1,0},{1,0}},ga120[2][2]={{120,0},{120,0}},ga448[2][2]={{448,0},{448,0}};
 row("gouraud_1","pairs",1,gour_e,ga1,2,setup_gouraud);
 row("gouraud_120","pairs",120,gour_e,ga120,2,setup_gouraud);
 row("gouraud_448","pairs",448,gour_e,ga448,2,setup_gouraud);
 printf("}}\n");
 (void)view_listed;(void)facing_n;
 return 0;
}
