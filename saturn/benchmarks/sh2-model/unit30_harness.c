/* SPDX-License-Identifier: GPL-2.0-or-later
 * Executed-instruction checks and cycle model for the fixed-unit-1 kernels
 * (unit30_experimental.h). Runs the actual big-endian SH-2 object code in
 * Yabause's unmodified interpreter, like harness.c, whose loader and call
 * wrapper it reuses. Flat RAM, no caches or wait states: MODELLED cycles, not
 * Saturn timing. Copyright (c) 2026 SH2ZAM Contributors. This optional model
 * harness is licensed separately from SH2ZAM's MIT core; see LICENSE.
 *
 * usage: unit30_harness model.elf verify-scale > results.json
 * The ELF must contain bridge.o from benchmarks/unit30-model and the library.
 * Before any timing, every output of every fast_ and port_ entry is checked:
 * the exact K1 rules and the rounded square root against the 128-bit oracle
 * in tests/unit30_oracle.h; every K2/K3 result bit for bit against the same
 * library source compiled for this host (the portable C path) and against
 * the oracle's error bounds; MAC.L dot products again with SR.S set on entry.
 * Any mismatch exits non-zero. */
#define main sh2_model_main
#include "harness.c"
#undef main
#include "../../tests/unit30_oracle.h"

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
static void bad(const char *what,uint64_t it){fprintf(stderr,"unit30 model mismatch: %s (case %"PRIu64")\n",what,it);exit(3);}

enum {N=128,NB=32};
static void put(const char *s,u32 i,int32_t v){wl(&cpu,symc(s)+4*i,(u32)v);}
static int32_t get(const char *s,u32 i){return (int32_t)rl(&cpu,symc(s)+4*i);}
static void putf(const char *s,u32 i,float f){u32 v;memcpy(&v,&f,4);wl(&cpu,symc(s)+4*i,v);}
static float getf(const char *s,u32 i){u32 v=rl(&cpu,symc(s)+4*i);float f;memcpy(&f,&v,4);return f;}
static void put64(u32 i,uint64_t s){wl(&cpu,symc("ss")+8*i,(u32)(s>>32));wl(&cpu,symc("ss")+8*i+4,(u32)s);}
static void put_basis(u32 i,const sh2z_basis30_t *b){const int32_t *w=&b->x[0];for(u32 k=0;k<9;k++)wl(&cpu,symc("bout")+36*i+4*k,(u32)w[k]);}
static void get_basis(u32 i,sh2z_basis30_t *b){int32_t *w=&b->x[0];for(u32 k=0;k<9;k++)w[k]=(int32_t)rl(&cpu,symc("bout")+36*i+4*k);}

/* ---- generators (the host test's kinds) ---- */
static uint64_t rs=UINT64_C(0x5eed00c0ffee1234);
static uint32_t rnd(void){rs^=rs<<13;rs^=rs>>7;rs^=rs<<17;return (uint32_t)(rs>>16);}
static uint64_t rnd64(void){return (uint64_t)rnd()<<32|rnd();}
static double runit(void){return ((double)rnd()+0.5)/4294967296.0;}
static int32_t s32u(uint32_t u){int32_t v;memcpy(&v,&u,4);return v;}
static const int32_t edges[]={0,1,-1,2,-2,0x1fffffff,0x20000000,-0x20000000,0x3fffffff,0x40000000,-0x40000000,-0x3fffffff,
 0x40000001,-0x40000001,0x5a827999,-0x5a827999,0x7fffffff,-0x7fffffff,INT32_MIN,0x7ffffffe,4096,-4096,12345,-987654};
enum {NEDGE=(int)(sizeof edges/sizeof edges[0])};
static int32_t rand_word(void){
 switch(rnd()%8u){
 case 0:return edges[rnd()%NEDGE];
 case 1:return s32u(rnd());
 case 2:return s32u(rnd())>>(rnd()%31u);
 case 3:return (int32_t)(rnd()%0x80000001u)-0x40000000;
 case 4:{const int32_t w=s32u((rnd()&0x3fffffffu)|0x40000000u);return rnd()%2u?-w:w;}
 default:return s32u(rnd())>>(1+rnd()%6u);
 }
}
static void rand_dir(int32_t v[3],double len){
 double d[3],n;
 do{for(int k=0;k<3;k++)d[k]=2.0*runit()-1.0;n=sqrt(d[0]*d[0]+d[1]*d[1]+d[2]*d[2]);}while(n<0.05||n>1.0);
 for(int k=0;k<3;k++)v[k]=(int32_t)lrint(ldexp(d[k]/n*len,30));
}
static void rand_any(int32_t v[3]){
 const unsigned kind=rnd()%6u;
 if(kind==0){for(int k=0;k<3;k++)v[k]=rand_word();return;}
 const int bits=1+(int)(rnd()%31u);
 for(int k=0;k<3;k++){int32_t w=s32u(rnd())>>(32-bits);if(kind==1&&rnd()%3u==0u)w=0;v[k]=w;}
 if(kind==3)v[rnd()%3u]=rnd()%2u?INT32_MIN:INT32_MAX;
 if(kind==4){v[0]=v[1]=v[2];if(rnd()%2u)v[1]=-v[1];}
 if(kind==5&&rnd()%4u==0u)v[0]=v[1]=v[2]=0;
}
/* A camera-like Q20.12 offset: 50 .. 5000 units in a random direction. */
static void rand_q12_offset(int32_t v[3]){
 int32_t d[3];rand_dir(d,1.0);const double len=50.0*pow(100.0,runit());
 for(int k=0;k<3;k++)v[k]=(int32_t)lrint(ldexp((double)d[k],-30)*len*4096.0);
}
static void rand_basis(sh2z_basis30_t *b,int primary){
 int32_t *p=primary?b->y:b->x,*s=primary?b->x:b->y;
 switch(rnd()%4u){case 0:rand_any(p);break;case 1:rand_q12_offset(p);break;default:rand_dir(p,0.2+1.6*runit());break;}
 switch(rnd()%5u){
 case 0:{double d[3];for(int k=0;k<3;k++)d[k]=(double)p[k];const double n=sqrt(d[0]*d[0]+d[1]*d[1]+d[2]*d[2]);
  const double eps=ldexp(1.0,-(int)(rnd()%20u)-2);
  for(int k=0;k<3;k++)s[k]=(int32_t)lrint(ldexp((n>0?d[k]/n:0.0)+eps*(2.0*runit()-1.0),29));break;}
 case 1:memcpy(s,p,12);if(rnd()%2u)for(int k=0;k<3;k++)s[k]=(int32_t)(s[k]/(1+(int32_t)(rnd()%3u)));break;
 default:rand_dir(s,0.5+1.0*runit());break;
 }
 for(int k=0;k<3;k++)b->z[k]=s32u(rnd());
}

/* ---- verification ---- */
static uint64_t v_k1,v_dot_s,v_q31,v_rs64,v_norm,v_norm_zero,v_isqrt,v_bases,v_bases_fail;
static double e_q31_lo=1e9,e_q31_hi=-1e9,e_rs64=0,e_norm=0;
static void verify_k1(uint64_t it){
 for(u32 i=0;i<N;i++){put("ia",i,rand_word());put("ib",i,rand_word());}
 static const char *const mults[2]={"fast_mul","fast_q12"};
 for(int e=0;e<2;e++){if(call(mults[e],N).result!=N)bad(mults[e],it);
  for(u32 i=0;i<N;i++)if(get("iout",i)!=uo_mul(get("ia",i),get("ib",i)))bad(mults[e],it);}
 int32_t a[N][3],b[N][3];
 for(u32 i=0;i<N;i++){
  switch(rnd()%4u){
  case 0:for(int k=0;k<3;k++){a[i][k]=rand_word();b[i][k]=rand_word();}break;
  case 1:rand_dir(a[i],1.0);rand_dir(b[i],0.5+1.4*runit());break;
  case 2:for(int k=0;k<3;k++){a[i][k]=rnd()%2u?INT32_MIN:INT32_MAX;b[i][k]=rnd()%2u?INT32_MIN:INT32_MAX;}break;
  default:for(int k=0;k<3;k++){a[i][k]=edges[rnd()%NEDGE];b[i][k]=edges[rnd()%NEDGE];}break;
  }
  for(int k=0;k<3;k++){wl(&cpu,symc("va")+12*i+4*(u32)k,(u32)a[i][k]);wl(&cpu,symc("vb")+12*i+4*(u32)k,(u32)b[i][k]);}
 }
 static const char *const dots[2]={"fast_dot","port_dot"};
 for(int e=0;e<2;e++)for(u32 sr=0;sr<=2;sr+=2){
  for(u32 i=0;i<N;i++)put("iout",i,0x5a5a5a5a);
  struct cost c=call_sr(dots[e],N,sr);if(c.result!=N||(c.sr&2)!=sr)bad("dot SR.S",it);
  for(u32 i=0;i<N;i++)if(get("iout",i)!=uo_dot3(a[i],b[i]))bad(sr?"dot with SR.S set on entry":dots[e],it);
  if(sr)v_dot_s+=N;
 }
 if(call("fast_cross",N).result!=N)bad("fast_cross",it);
 for(u32 i=0;i<N;i++){int32_t w[3];uo_cross(w,a[i],b[i]);for(int k=0;k<3;k++)if((int32_t)rl(&cpu,symc("vout")+12*i+4*(u32)k)!=w[k])bad("fast_cross",it);}
 v_k1+=N;
}
static void verify_k2(uint64_t it){
 /* mantissa: random m plus interval ends */
 uint32_t m[N];
 for(u32 i=0;i<N;i++){const uint32_t j=rnd()%256u;const uint32_t lo=j<128?UINT32_C(0x40000000)+(j<<23):j<<24;
  const uint32_t len=j<128?(UINT32_C(1)<<23):(UINT32_C(1)<<24);
  m[i]=rnd()%3u==0u?lo+rnd()%len:(rnd()%2u?lo+rnd()%64u:lo+len-1u-rnd()%64u);put("mm",i,(int32_t)m[i]);}
 if(call("fast_q31",N).result!=N)bad("fast_q31",it);
 for(u32 i=0;i<N;i++){const uint32_t y=(uint32_t)get("uout",i);if(y!=sh2zi_u30_rsqrt_q31(m[i]))bad("fast_q31 vs host",it);
  const double e=(double)y-uo_rsqrt_q31_ref(m[i]);if(e<e_q31_lo)e_q31_lo=e;if(e>e_q31_hi)e_q31_hi=e;}
 if(e_q31_lo<-1.25||e_q31_hi>0.25)bad("fast_q31 bound",it);
 v_q31+=N;
 /* rsqrt64 */
 uint64_t s[N];int sc[N];
 for(u32 i=0;i<N;i++){s[i]=rnd()%16u?rnd64()>>(rnd()%64u):(rnd()%2u?0:UINT64_MAX);sc[i]=rnd()%8u?(int)(rnd()%128u):(int)(rnd()%140u)-6;put64(i,s[i]);put("sc",i,sc[i]);}
 if(call("fast_rsqrt64",N).result!=N)bad("fast_rsqrt64",it);
 for(u32 i=0;i<N;i++){const int32_t r=get("iout",i);if(r!=sh2z_rsqrt64_u30(s[i],sc[i]))bad("fast_rsqrt64 vs host",it);
  if(sc[i]>=0&&sc[i]<=127&&s[i]&&r!=INT32_MAX){const double e=fabs((double)r-uo_rsqrt_ref(s[i],sc[i]));if(e>e_rs64)e_rs64=e;if(e>2.0)bad("fast_rsqrt64 bound",it);}}
 v_rs64+=N;
 /* normalize */
 int32_t v[N][3];
 for(u32 i=0;i<N;i++){switch(rnd()%3u){case 0:rand_any(v[i]);break;case 1:rand_q12_offset(v[i]);break;default:rand_dir(v[i],0.25+1.5*runit());}
  for(int k=0;k<3;k++)wl(&cpu,symc("va")+12*i+4*(u32)k,(u32)v[i][k]);}
 static const char *const norms[2]={"fast_norm","port_norm"};
 for(int e=0;e<2;e++){
 for(u32 i=0;i<N;i++)for(u32 k=0;k<3;k++)wl(&cpu,symc("vout")+12*i+4*k,UINT32_C(0xa5a5a5a5));
 if(call(norms[e],N).result!=N)bad(norms[e],it);
 for(u32 i=0;i<N;i++){
  int32_t want[3]={s32u(0xa5a5a5a5u),s32u(0xa5a5a5a5u),s32u(0xa5a5a5a5u)};const bool ok=sh2z_vec3_normalize_u30(want,v[i]);
  if(get("brc",i)!=(int32_t)ok)bad(e?"port_norm result":"fast_norm result",it);
  for(int k=0;k<3;k++)if((int32_t)rl(&cpu,symc("vout")+12*i+4*(u32)k)!=want[k])bad(e?"port_norm vs host":"fast_norm vs host",it);
  if(!ok){if(v[i][0]||v[i][1]||v[i][2])bad("fast_norm refused",it);v_norm_zero++;continue;}
  double ref[3];uo_unit_ref(ref,v[i]);for(int k=0;k<3;k++){const double d=fabs((double)want[k]-ref[k]);if(d>e_norm)e_norm=d;if(d>2.0)bad("fast_norm bound",it);}
 }
 }
 /* the same vectors in place (out == v) */
 if(call("fast_norm_inplace",N).result!=N)bad("fast_norm_inplace",it);
 for(u32 i=0;i<N;i++){int32_t want[3];memcpy(want,v[i],12);(void)sh2z_vec3_normalize_u30(want,want);
  for(int k=0;k<3;k++)if((int32_t)rl(&cpu,symc("va")+12*i+4*(u32)k)!=want[k])bad("fast_norm in place",it);}
 v_norm+=N;
 /* isqrt */
 for(u32 i=0;i<N;i++){const uint64_t r=rnd64()>>(rnd()%33u+31u);
  s[i]=rnd()%2u?rnd64()>>(rnd()%64u):r*r+(rnd()%3u)*r-(rnd()%2u);put64(i,s[i]);}
 if(call("fast_isqrt",N).result!=N)bad("fast_isqrt",it);
 for(u32 i=0;i<N;i++)if((uint32_t)get("uout",i)!=uo_isqrt_round(s[i]))bad("fast_isqrt vs oracle",it);
 v_isqrt+=N;
}
static void verify_k3(uint64_t it){
 static const char *const fns[2]={"fast_forward","fast_up"};
 for(int primary=0;primary<2;primary++){
  sh2z_basis30_t in[NB];
  for(u32 i=0;i<NB;i++){rand_basis(&in[i],primary);put_basis(i,&in[i]);}
  if(call(fns[primary],NB).result!=NB)bad(fns[primary],it);
  for(u32 i=0;i<NB;i++){
   sh2z_basis30_t want=in[i],got;get_basis(i,&got);
   const int rc=primary?sh2z_basis30_orthonormalize_up(&want):sh2z_basis30_orthonormalize_forward(&want);
   if(get("brc",i)!=rc||memcmp(&want,&got,sizeof want))bad(primary?"fast_up vs host":"fast_forward vs host",it);
   if(rc)v_bases_fail++;
  }
  v_bases+=NB;
 }
}

/* ---- cost ---- */
enum {CORPORA=16};
struct stats {double mean,median;unsigned min,max;double instr;};
static int cmp_u(const void*a,const void*b){unsigned x=*(const unsigned*)a,y=*(const unsigned*)b;return x<y?-1:x>y;}
static struct stats summarize(unsigned *cycles,size_t n,uint64_t instr){
 qsort(cycles,n,sizeof *cycles,cmp_u);double sum=0;for(size_t i=0;i<n;i++)sum+=cycles[i];
 return (struct stats){sum/(double)n,n&1?cycles[n/2]:(cycles[n/2-1]+cycles[n/2])/2.0,cycles[0],cycles[n-1],(double)instr/(double)n};
}
static void print_stats(struct stats s){printf("{\"meanCycles\":%.2f,\"medianCycles\":%.1f,\"minCycles\":%u,\"maxCycles\":%u,\"meanInstructions\":%.2f}",s.mean,s.median,s.min,s.max,s.instr);}
typedef void (*loader_fn)(void);
/* Each loader writes one corpus for every entry of its row (fixed-point,
 * Q16.16 and binary32 forms of the same values). */
static void load_mul(void){for(u32 i=0;i<N;i++){int32_t a[3],b[3];rand_dir(a,1.0);rand_dir(b,1.0);
 put("ia",i,a[0]);put("ib",i,b[1]);putf("fa",i,(float)ldexp(a[0],-30));putf("fb",i,(float)ldexp(b[1],-30));}}
static void put_vecs(u32 i,const int32_t a[3],const int32_t b[3],int shift){
 for(u32 k=0;k<3;k++){wl(&cpu,symc("va")+12*i+4*k,(u32)a[k]);wl(&cpu,symc("vb")+12*i+4*k,(u32)b[k]);
  putf("fva",3*i+k,(float)ldexp(a[k],-30));putf("fvb",3*i+k,(float)ldexp(b[k],-30));
  wl(&cpu,symc("qa")+12*i+4*k,(u32)(shift>=0?a[k]>>shift:a[k]<<-shift));wl(&cpu,symc("qb")+12*i+4*k,(u32)(shift>=0?b[k]>>shift:b[k]<<-shift));}}
static void load_vec(void){for(u32 i=0;i<N;i++){int32_t a[3],b[3];rand_dir(a,1.0);rand_dir(b,1.0);put_vecs(i,a,b,14);}}
static void load_rsqrt(void){for(u32 i=0;i<N;i++){int32_t v[3];rand_q12_offset(v);
 const uint64_t s=(uint64_t)((int64_t)v[0]*v[0])+(uint64_t)((int64_t)v[1]*v[1])+(uint64_t)((int64_t)v[2]*v[2]);
 put64(i,s);put("sc",i,24);put("mm",i,(int32_t)(UINT32_C(0x40000000)+rnd()%UINT32_C(0xc0000000)));
 putf("fa",i,(float)ldexp((double)s,-24));const double q16=ldexp((double)s,-24+16);put("ia",i,q16>2147483647.0?INT32_MAX:(int32_t)q16);}}
static void load_norm_q12(void){for(u32 i=0;i<N;i++){int32_t v[3],z[3]={0,0,0};rand_q12_offset(v);put_vecs(i,v,z,-4);
 for(u32 k=0;k<3;k++)putf("fva",3*i+k,(float)ldexp(v[k],-12));}}
static void load_norm_u30(void){for(u32 i=0;i<N;i++){int32_t v[3],z[3]={0,0,0};rand_dir(v,1.0+ldexp(2.0*runit()-1.0,-12));put_vecs(i,v,z,14);}}
static void load_isqrt(void){for(u32 i=0;i<N;i++){int32_t v[3];rand_q12_offset(v);
 const uint64_t s=(uint64_t)((int64_t)v[0]*v[0])+(uint64_t)((int64_t)v[1]*v[1])+(uint64_t)((int64_t)v[2]*v[2]);
 put64(i,s);putf("fa",i,(float)ldexp((double)s,-24));}}
/* Camera-like bases: forward = a Q20.12 target - eye offset, up near +Y
 * (forward); or up = up + 0.3 tilt, x = a unit forward (up). */
static void load_basis(int primary){
 for(u32 i=0;i<NB;i++){
  sh2z_basis30_t b;int32_t t[3];
  if(!primary){rand_q12_offset(b.x);rand_dir(t,1.0);b.y[0]=t[0]/8;b.y[1]=0x40000000-abs(t[1]/16);b.y[2]=t[2]/8;}
  else{rand_dir(b.x,1.0);rand_dir(t,0.3);b.y[0]=t[0];b.y[1]=0x40000000+t[1];b.y[2]=t[2];}
  for(int k=0;k<3;k++)b.z[k]=0;
  put_basis(i,&b);
  const double xs=primary?-30.0:-12.0;
  for(u32 k=0;k<3;k++){
   putf("fbase",9*i+k,(float)ldexp(b.x[k],(int)xs));putf("fbase",9*i+3+k,(float)ldexp(b.y[k],-30));putf("fbase",9*i+6+k,0.0f);
   /* Q16.16 rows: forward x from Q20.12 (x16), unit rows from Q2.30 (/16384) */
   wl(&cpu,symc("qbase")+36*i+4*k,(u32)(primary?b.x[k]>>14:b.x[k]*16));
   wl(&cpu,symc("qbase")+36*i+12+4*k,(u32)(b.y[k]>>14));wl(&cpu,symc("qbase")+36*i+24+4*k,0);
  }
 }
}
static void load_forward(void){load_basis(0);}
static void load_up(void){load_basis(1);}
struct row {const char *name,*description;loader_fn load;unsigned n;const char *entries[5];};
static const struct row rows[]={
 {"mul","one Q2.30 product of unit-vector components",load_mul,N,{"fast_mul","fast_q12","float_mul","q16_mul",0}},
 {"dot","dot product of two unit vectors",load_vec,N,{"fast_dot","port_dot","float_dot","q16_dot",0}},
 {"cross","cross product of two unit vectors",load_vec,N,{"fast_cross","float_cross","q16_cross",0}},
 {"rsqrt","1/sqrt of the squared length of a Q20.12 offset (50 .. 5000 units); fast_q31 is the mantissa kernel alone",load_rsqrt,N,{"fast_q31","fast_rsqrt64","float_rsqrt","q16_rsqrt",0}},
 {"normalize_q12","unit vector of a Q20.12 offset of 50 .. 5000 units",load_norm_q12,N,{"fast_norm","port_norm","float_norm","q16_norm",0}},
 {"normalize_u30","renormalising a Q2.30 vector of length 1 +- 2^-12",load_norm_u30,N,{"fast_norm","port_norm","float_norm","q16_norm",0}},
 {"isqrt","length of a Q20.12 offset from its 64-bit sum of squares",load_isqrt,N,{"fast_isqrt","float_sqrt",0}},
 {"forward","orthonormalize around forward: x = Q20.12 target - eye, y = up near +Y",load_forward,NB,{"fast_forward","float_forward","q16_forward",0}},
 {"up","orthonormalize around up: y = up + 0.3 tilt, x = a unit forward",load_up,NB,{"fast_up","float_up",0}},
};
static void cost_row(const struct row *r,bool *first){
 static unsigned cyc[2][CORPORA];static uint64_t ins[2];
 if(!*first)putchar(',');*first=false;
 printf("\"%s\":{\"description\":\"%s\",\"n\":%u,\"corpora\":%u,\"entries\":{",r->name,r->description,r->n,CORPORA);
 for(int e=0;r->entries[e];e++){
  memset(ins,0,sizeof ins);
  for(unsigned c=0;c<CORPORA;c++)for(int z=0;z<2;z++){
   rs=UINT64_C(0xc0571234)+(uint64_t)c*7919u+(uint64_t)(r-rows)*104729u;r->load();
   const unsigned n=z?r->n:1u;struct cost k=call(r->entries[e],n);if(k.result!=n)bad(r->entries[e],c);
   cyc[z][c]=k.cycles;ins[z]+=k.instructions;
  }
  struct stats one=summarize(cyc[0],CORPORA,ins[0]),many=summarize(cyc[1],CORPORA,ins[1]);
  printf("%s\"%s\":{\"cyclesPerElement\":%.2f,\"instructionsPerElement\":%.2f,\"call1\":",e?",":"",r->entries[e],
   (many.mean-one.mean)/(r->n-1),(many.instr-one.instr)/(r->n-1));
  print_stats(one);printf(",\"call%u\":",r->n);print_stats(many);printf("}");
 }
 printf("}}");
}

int main(int argc,char**argv){
 if(argc<3){fprintf(stderr,"usage: unit30_harness model.elf verify-scale\n");return 2;}
 const uint64_t scale=strtoull(argv[2],0,10);
 load_elf_symbols(argv[1]);load_elf(argv[1]);
 cpu.model=SHMT_SH2;cpu.MappedMemoryReadByte=rb;cpu.MappedMemoryReadWord=rw;cpu.MappedMemoryReadLong=rl;cpu.MappedMemoryWriteByte=wb;cpu.MappedMemoryWriteWord=ww;cpu.MappedMemoryWriteLong=wl;
 for(unsigned i=0;i<65536;i++)cpu.opcodes[i]=decode(SHMT_SH2,(u16)i);for(unsigned i=0;i<256;i++)cpu.fetchlist[i]=fetch;
 for(uint64_t it=0;it<scale;it++){verify_k1(it);verify_k2(it);verify_k3(it);}
 printf("{\"schema\":1,\"kind\":\"Yabause isolated SH2 interpreter flat-RAM model, not Saturn timing\",");
 printf("\"verified\":{\"k1Elements\":%"PRIu64",\"dotWithSRSSet\":%"PRIu64",\"rsqrtMantissas\":%"PRIu64",\"rsqrt64\":%"PRIu64
  ",\"normalized\":%"PRIu64",\"normalizeRefusedZero\":%"PRIu64",\"isqrt\":%"PRIu64",\"bases\":%"PRIu64",\"basesReturningMinus1\":%"PRIu64
  ",\"mantissaErrorLsb\":[%.4f,%.4f],\"rsqrt64MaxErrorLsb\":%.4f,\"normalizeMaxErrorLsb\":%.4f,"
  "\"entries\":\"fast_mul, fast_q12, fast_dot, port_dot (also with SR.S set), fast_cross, fast_isqrt against tests/unit30_oracle.h; "
  "fast_q31, fast_rsqrt64, fast_norm (also in place), port_norm, fast_forward, fast_up bit for bit against the host build of the same source and within the oracle bounds\"},",
  v_k1,v_dot_s,v_q31,v_rs64,v_norm,v_norm_zero,v_isqrt,v_bases,v_bases_fail,e_q31_lo,e_q31_hi,e_rs64,e_norm);
 bool first=true;printf("\"rows\":{");
 for(size_t i=0;i<sizeof rows/sizeof rows[0];i++){cost_row(&rows[i],&first);fflush(stdout);}
 printf("}}\n");
 return 0;
}
