/* SPDX-License-Identifier: GPL-2.0-or-later
 * View batch capture, included by sh2-model/harness.c (--view).
 * Copyright (c) 2026 SH2ZAM Contributors.
 *
 * 1. Divider helper probes against an independent host rule.
 * 2. Exactness sweep: 512 random cameras x 2048 positions (the P0 host sweep's
 *    distribution, own generator and seed), both target paths, every output
 *    byte compared with an independent __int128 oracle.
 * 3. Timing: 64 cameras x 512 positions placed in front of the camera, so
 *    nearly every vertex costs two divisions. */
#define VIEW_SWEEP_POSES 512u
#define VIEW_SWEEP_COUNT 2048u
#define VIEW_TIME_POSES 64u
#define VIEW_TIME_COUNT 512u

struct view_cam {int16_t r[3][3];int32_t t[3],fx,fy;int16_t cx,cy;int32_t near_z,far_z;int16_t clip[4];};
struct view_vtx {int32_t x,y,z;int16_t sx,sy;uint8_t code;};
static uint32_t view_state;
static uint32_t view_word(void) {view_state^=view_state<<13;view_state^=view_state>>17;view_state^=view_state<<5;return view_state;}
static int32_t view_range(int32_t lo,int32_t hi) {return lo+(int32_t)(view_word()%(uint32_t)(hi-lo+1));}
static int16_t view_q14(double x) {double s=floor(x*16384.0+0.5);return (int16_t)(s>32767?32767:s<-32768?-32768:s);}
static __int128 view_floor(__int128 n,__int128 d) {__int128 q=n/d;if(n%d!=0&&((n<0)!=(d<0)))q--;return q;}
static __int128 view_clamp(__int128 a,__int128 lo,__int128 hi) {return a<lo?lo:a>hi?hi:a;}

/* Rotation rows from yaw/pitch/roll, Rz * Rx * Ry, then quantized. */
static void view_rotation(double m[3][3],double yaw,double pitch,double roll) {
 const double cy=cos(yaw),sy=sin(yaw),cp=cos(pitch),sp=sin(pitch),cr=cos(roll),sr=sin(roll);
 const double ry[3][3]={{cy,0,-sy},{0,1,0},{sy,0,cy}},rx[3][3]={{1,0,0},{0,cp,sp},{0,-sp,cp}},rz[3][3]={{cr,sr,0},{-sr,cr,0},{0,0,1}};
 double a[3][3];
 for(int i=0;i<3;i++)for(int j=0;j<3;j++)a[i][j]=rx[i][0]*ry[0][j]+rx[i][1]*ry[1][j]+rx[i][2]*ry[2][j];
 for(int i=0;i<3;i++)for(int j=0;j<3;j++)m[i][j]=rz[i][0]*a[0][j]+rz[i][1]*a[1][j]+rz[i][2]*a[2][j];
}
static int view_row_ok(const int16_t r[3],int64_t t) {
 int64_t s=0;for(int j=0;j<3;j++)s+=r[j]<0?-(int64_t)r[j]:r[j];
 return s*32768+(t<0?-t:t)<=INT32_MAX;
}
/* t[i] = -floor(r[i] . eye / 65536), eye in Q16.16: the contract's placement. */
static int view_place(struct view_cam *c,const int32_t eye[3]) {
 for(int i=0;i<3;i++){__int128 s=0;for(int j=0;j<3;j++)s+=(__int128)c->r[i][j]*eye[j];
  __int128 t=-view_floor(s,65536);if(t<INT32_MIN||t>INT32_MAX||!view_row_ok(c->r[i],(int64_t)t))return 0;c->t[i]=(int32_t)t;}
 return 1;
}
static struct view_cam view_random_cam(double m[3][3],double eye_units[3],int timing) {
 struct view_cam c;memset(&c,0,sizeof(c));
 for(;;) {
  view_rotation(m,(view_word()%65536)*6.283185307179586/65536.0,((double)(view_word()%65536)/65536.0-0.5)*(timing?0.8:3.0),
   ((double)(view_word()%65536)/65536.0-0.5)*(timing?0.6:1.2));
  for(int i=0;i<3;i++)for(int j=0;j<3;j++)c.r[i][j]=view_q14(m[i][j]);
  c.fx=timing?view_range(140,320)*65536+view_range(0,65535):view_range(64,1024)*65536+view_range(0,65535);
  c.fy=timing?c.fx:view_range(64,1024)*65536+view_range(0,65535);
  c.cx=(int16_t)(timing?160:view_range(0,352));c.cy=(int16_t)(timing?112:view_range(0,256));
  c.near_z=timing?4096:view_range(1,16*16384);c.far_z=timing?4000*16384:c.near_z+view_range(1,INT32_MAX-c.near_z);
  if(timing){c.clip[0]=0;c.clip[1]=0;c.clip[2]=319;c.clip[3]=223;}
  else{c.clip[0]=(int16_t)view_range(-64,160);c.clip[2]=(int16_t)view_range(c.clip[0],400);
   c.clip[1]=(int16_t)view_range(-64,120);c.clip[3]=(int16_t)view_range(c.clip[1],300);}
  const int32_t span=timing?20000:30000;
  int32_t eye[3];for(int j=0;j<3;j++){eye[j]=view_range(-span,span)*65536+view_range(0,65535);eye_units[j]=eye[j]/65536.0;}
  if(view_place(&c,eye))return c;
 }
}
static void view_positions(int16_t (*p)[3],unsigned n,const double m[3][3],const double eye[3],int timing,const struct view_cam *c) {
 for(unsigned k=0;k<n;k++) {
  int32_t q[3];
  if(timing) {
   /* A point inside the view volume, mapped back to the world. */
   const double z=2.0+(view_word()%65536)/65536.0*3000.0,half=170.0*65536.0/c->fx;
   const double v[3]={((view_word()%65536)/32768.0-1.0)*half*z,((view_word()%65536)/32768.0-1.0)*half*z*0.75,z};
   for(int j=0;j<3;j++){double w=eye[j]+m[0][j]*v[0]+m[1][j]*v[1]+m[2][j]*v[2];q[j]=(int32_t)floor(w+0.5);}
  } else {
   const unsigned mode=view_word()%4u;
   for(int j=0;j<3;j++){if(mode==0)q[j]=view_range(INT16_MIN,INT16_MAX);
    else{const int32_t s=mode==1?16:mode==2?2048:32768;q[j]=(int32_t)floor(eye[j])+view_range(-s,s);}}
  }
  for(int j=0;j<3;j++)p[k][j]=(int16_t)(q[j]<INT16_MIN?INT16_MIN:q[j]>INT16_MAX?INT16_MAX:q[j]);
 }
}
/* Projection of a view-space point, from the contract text. */
static struct view_vtx view_project(const struct view_cam *c,const __int128 v[3],uint64_t *saturated) {
 struct view_vtx o;memset(&o,0,sizeof(o));
 for(int i=0;i<3;i++)if(v[i]<INT32_MIN||v[i]>INT32_MAX)fail("View oracle bound",i);
 o.x=(int32_t)v[0];o.y=(int32_t)v[1];o.z=(int32_t)v[2];
 if(v[2]<c->near_z){o.code=1;return o;}
 __int128 n[2]={(__int128)c->fx*v[0],(__int128)c->fy*v[1]},q[2];
 for(int i=0;i<2;i++){__int128 a=n[i]<0?-n[i]:n[i];q[i]=(n[i]<0?-1:1)*(a/v[2]);
  if((q[i]<INT32_MIN||q[i]>INT32_MAX)&&saturated)(*saturated)++;q[i]=view_clamp(q[i],INT32_MIN,INT32_MAX);}
 o.sx=(int16_t)view_clamp(c->cx+view_floor(q[0],65536),INT16_MIN,INT16_MAX);
 o.sy=(int16_t)view_clamp(c->cy-view_floor(q[1],65536),INT16_MIN,INT16_MAX);
 o.code=(uint8_t)((v[2]>c->far_z?2:0)|(o.sx<c->clip[0]?4:0)|(o.sx>c->clip[2]?8:0)|(o.sy<c->clip[1]?16:0)|(o.sy>c->clip[3]?32:0));
 return o;
}
static struct view_vtx view_oracle(const struct view_cam *c,const int16_t p[3],uint64_t *saturated) {
 __int128 v[3];
 for(int i=0;i<3;i++){v[i]=c->t[i];for(int j=0;j<3;j++)v[i]+=(__int128)c->r[i][j]*p[j];}
 return view_project(c,v,saturated);
}
static void view_store_cam(const struct view_cam *c) {
 u32 a=sym("_view_camera");
 for(int i=0;i<9;i++)ww(&cpu,a+2*i,(u16)c->r[i/3][i%3]);
 ww(&cpu,a+18,0);
 for(int i=0;i<3;i++)wl(&cpu,a+20+4*i,(u32)c->t[i]);
 wl(&cpu,a+32,(u32)c->fx);wl(&cpu,a+36,(u32)c->fy);ww(&cpu,a+40,(u16)c->cx);ww(&cpu,a+42,(u16)c->cy);
 wl(&cpu,a+44,(u32)c->near_z);wl(&cpu,a+48,(u32)c->far_z);for(int i=0;i<4;i++)ww(&cpu,a+52+2*i,(u16)c->clip[i]);
}
static void view_store_positions(int16_t (*p)[3],unsigned n) {
 u32 a=sym("_view_input");for(unsigned k=0;k<n;k++)for(int j=0;j<3;j++)ww(&cpu,a+6*k+2*j,(u16)p[k][j]);
}
/* Compare one output buffer with the oracle; also checks reserved bytes. */
static uint64_t view_compare(u32 out,const struct view_vtx *want,unsigned n) {
 uint64_t bad=0;
 for(unsigned k=0;k<n;k++){u32 a=out+20*k;const struct view_vtx *w=&want[k];
  if(rl(&cpu,a)!=(u32)w->x||rl(&cpu,a+4)!=(u32)w->y||rl(&cpu,a+8)!=(u32)w->z||rw(&cpu,a+12)!=(u16)w->sx||
     rw(&cpu,a+14)!=(u16)w->sy||rb(&cpu,a+16)!=w->code||rb(&cpu,a+17)||rb(&cpu,a+18)||rb(&cpu,a+19))bad++;}
 return bad;
}
static void view_poison(u32 out,unsigned n) {for(unsigned i=0;i<5*n;i++)wl(&cpu,out+4*i,UINT32_C(0xA5A5A5A5));}
static int view_cmp_u32(const void *a,const void *b) {u32 x=*(const u32*)a,y=*(const u32*)b;return x<y?-1:x>y;}

#include "edges.h"
struct view_probe {int64_t n;int32_t d;};
static int capture_view(void) {
 if(rl(&cpu,sym("_view_capacity"))<VIEW_SWEEP_COUNT)fail("View capacity",0);
 printf("{\"schema\":1,\"kind\":\"SH2ZAM view batch, Yabause isolated SH2 flat-RAM model with harness divider model, not Saturn timing\",");
 printf("\"sourceCommit\":\"%s\",\"dividerLatencyAssumed\":%u,",(char*)ram+offset(sym("_view_source_commit"),1),DIVU_LATENCY);
 /* 1. Divider helpers. */
 const int64_t big=(int64_t)1<<62;
 struct view_probe probes[]={{100,7},{-100,7},{100,-7},{-100,-7},{0,5},{5,0},{-5,0},{0,0},
  {((int64_t)7<<31)-1,7},{(int64_t)7<<31,7},{-((int64_t)7<<31)+1,7},{-((int64_t)7<<31),7},{-((int64_t)7<<31)-1,7},
  {((int64_t)7<<31)-1,-7},{INT64_MIN,1},{INT64_MIN,-1},{INT64_MAX,INT32_MAX},{big,INT32_MIN},{-big,INT32_MIN},
  {(int64_t)INT32_MAX*INT32_MAX,INT32_MAX},{-(int64_t)INT32_MAX*INT32_MAX,INT32_MAX},{123456789012LL,-98765},{-1,INT32_MIN}};
 const unsigned nprobes=sizeof(probes)/sizeof(probes[0]);
 unsigned hw=0;
 view_state=UINT32_C(0x9e3779b9);  /* xorshift must not start at zero */
 printf("\"dividerProbes\":[");
 for(unsigned i=0;i<nprobes+2000;i++) {
  struct view_probe pr;
  if(i<nprobes)pr=probes[i];
  else{pr.d=(int32_t)(view_word()>>(view_word()%31));if(view_word()&1)pr.d=-pr.d;
   if(i%4==0){uint64_t b=((uint64_t)view_word()<<32)|view_word();pr.n=(int64_t)(b>>(view_word()%40));if(view_word()&1)pr.n=-pr.n;}
   else{/* q*d + r, mostly fitting, some at the boundary */
    int64_t q=i%4==1?(int64_t)(int32_t)view_word():i%4==2?(int64_t)INT32_MAX-(view_word()%3):-(int64_t)INT32_MAX-1+(view_word()%3);
    int64_t ad=pr.d<0?-(int64_t)pr.d:pr.d,r=ad?(int64_t)(view_word()%(uint64_t)ad):0;
    pr.n=q*pr.d+((view_word()&1)?r:-r);}}
  __int128 n=pr.n,d=pr.d,a=n<0?-n:n,ad=d<0?-d:d;
  int fits=d!=0&&a<(ad<<31);int32_t expect;
  if(d==0)expect=n>=0?INT32_MAX:INT32_MIN;
  else{__int128 q=((n<0)!=(d<0)?-1:1)*(a/ad);expect=(int32_t)view_clamp(q,INT32_MIN,INT32_MAX);}
  divu_reset();
  u32 hi=(u32)((uint64_t)pr.n>>32),lo=(u32)(uint64_t)pr.n;
  struct cost cst=invoke("_view_divu_probe",hi,lo,(u32)pr.d,0);
  if(cst.result!=1)fail("Divider probe call",i);
  u32 res=sym("_view_divu_result"),ovf=sym("_view_divu_overflow");
  for(int k=0;k<4;k++)if((int32_t)rl(&cpu,res+4*k)!=(k==1?-142857143:expect)){fprintf(stderr,"probe %u slot %d n=%lld d=%d got %d want %d\n",i,k,(long long)pr.n,pr.d,(int32_t)rl(&cpu,res+4*k),expect);fail("Divider probe result",i);}
  for(int k=0;k<2;k++)if(rl(&cpu,ovf+4*k)!=(u32)!fits)fail("Divider probe overflow flag",i);
  if(divu_divisions!=(fits?5u:2u))fail("Divider probe division count",(u32)divu_divisions);
  hw+=fits;
  if(i<nprobes){if(i)putchar(',');printf("{\"n\":\"%lld\",\"d\":%d,\"q\":%d,\"overflow\":%s}",(long long)pr.n,pr.d,expect,fits?"false":"true");}
 }
 printf("],\"dividerProbeCount\":%u,\"dividerProbeHardwarePath\":%u,",nprobes+2000,hw);
 /* 2. Exactness sweep. */
 static int16_t pos[VIEW_SWEEP_COUNT][3];static struct view_vtx want[VIEW_SWEEP_COUNT];
 uint64_t vertices=0,near=0,saturated=0,bad_portable=0,bad_divu=0,divisions=0,strided=0;
 view_state=UINT32_C(0x51ab0001);
 for(unsigned pose=0;pose<VIEW_SWEEP_POSES;pose++) {
  double m[3][3],eye[3];struct view_cam c=view_random_cam(m,eye,0);
  view_positions(pos,VIEW_SWEEP_COUNT,m,eye,0,&c);
  view_store_cam(&c);view_store_positions(pos,VIEW_SWEEP_COUNT);
  for(unsigned k=0;k<VIEW_SWEEP_COUNT;k++){want[k]=view_oracle(&c,pos[k],&saturated);near+=want[k].code&1;}
  view_poison(sym("_view_out_portable"),VIEW_SWEEP_COUNT);view_poison(sym("_view_out_divu"),VIEW_SWEEP_COUNT);
  divu_reset();
  if(invoke("_view_run_portable",VIEW_SWEEP_COUNT,0,0,0).result!=1)fail("Portable view batch",pose);
  if(divu_divisions)fail("Portable path used the divider",pose);
  if(invoke("_view_run_divu",VIEW_SWEEP_COUNT,0,0,0).result!=1)fail("Divider view batch",pose);
  divisions+=divu_divisions;
  bad_portable+=view_compare(sym("_view_out_portable"),want,VIEW_SWEEP_COUNT);
  bad_divu+=view_compare(sym("_view_out_divu"),want,VIEW_SWEEP_COUNT);
  vertices+=VIEW_SWEEP_COUNT;
  if(pose%8==0) {
   /* Stride 12: every other record of the same input. */
   static struct view_vtx want_even[VIEW_SWEEP_COUNT/2];
   for(unsigned k=0;k<VIEW_SWEEP_COUNT/2;k++)want_even[k]=want[2*k];
   view_poison(sym("_view_out_divu"),VIEW_SWEEP_COUNT);
   if(invoke("_view_run_divu_stride12",VIEW_SWEEP_COUNT/2,0,0,0).result!=1)fail("Strided divider view batch",pose);
   bad_divu+=view_compare(sym("_view_out_divu"),want_even,VIEW_SWEEP_COUNT/2);
   strided+=VIEW_SWEEP_COUNT/2;
  }
 }
 printf("\"sweep\":{\"poses\":%u,\"vertices\":%"PRIu64",\"near\":%"PRIu64",\"saturatedQuotients\":%"PRIu64",\"divisions\":%"PRIu64",\"stride12Vertices\":%"PRIu64",\"portableMismatches\":%"PRIu64",\"divuMismatches\":%"PRIu64"},",
  VIEW_SWEEP_POSES,vertices,near,saturated,divisions,strided,bad_portable,bad_divu);
 if(bad_portable||bad_divu)fail("View sweep mismatch",(u32)(bad_portable+bad_divu));
 /* 3. Timing. */
 static u32 cyc[2][VIEW_TIME_POSES],raw[VIEW_TIME_POSES],plus[VIEW_TIME_POSES];
 uint64_t tnear=0,tdiv=0,tstall=0,tinst[2]={0,0};
 view_state=UINT32_C(0x71e00002);
 printf("\"timing\":{\"poses\":%u,\"verticesPerBatch\":%u,\"batches\":[",VIEW_TIME_POSES,VIEW_TIME_COUNT);
 for(unsigned pose=0;pose<VIEW_TIME_POSES;pose++) {
  double m[3][3],eye[3];struct view_cam c=view_random_cam(m,eye,1);
  view_positions(pos,VIEW_TIME_COUNT,m,eye,1,&c);
  view_store_cam(&c);view_store_positions(pos,VIEW_TIME_COUNT);
  uint64_t sat=0,pnear=0;for(unsigned k=0;k<VIEW_TIME_COUNT;k++){want[k]=view_oracle(&c,pos[k],&sat);pnear+=want[k].code&1;}
  divu_reset();
  struct cost a=invoke("_view_run_portable",VIEW_TIME_COUNT,0,0,0);
  divu_reset();
  struct cost b=invoke("_view_run_divu",VIEW_TIME_COUNT,0,0,0);
  if(a.result!=1||b.result!=1)fail("Timing batch",pose);
  if(view_compare(sym("_view_out_portable"),want,VIEW_TIME_COUNT)||view_compare(sym("_view_out_divu"),want,VIEW_TIME_COUNT))fail("Timing mismatch",pose);
  cyc[0][pose]=a.cycles;cyc[1][pose]=b.cycles;raw[pose]=b.cycles-(u32)divu_stall_cycles;plus[pose]=raw[pose]+DIVU_LATENCY*(u32)divu_divisions;
  tnear+=pnear;tdiv+=divu_divisions;tstall+=divu_stall_cycles;tinst[0]+=a.instructions;tinst[1]+=b.instructions;
  if(pose)putchar(',');
  printf("{\"near\":%"PRIu64",\"divisions\":%"PRIu64",\"portableCycles\":%u,\"divuCycles\":%u,\"divuStallCycles\":%"PRIu64",\"divuInstructions\":%"PRIu64"}",pnear,divu_divisions,a.cycles,b.cycles,divu_stall_cycles,b.instructions);
 }
 qsort(cyc[0],VIEW_TIME_POSES,4,view_cmp_u32);qsort(cyc[1],VIEW_TIME_POSES,4,view_cmp_u32);
 qsort(raw,VIEW_TIME_POSES,4,view_cmp_u32);qsort(plus,VIEW_TIME_POSES,4,view_cmp_u32);
 const unsigned mid=VIEW_TIME_POSES/2;const double nv=VIEW_TIME_COUNT;
 printf("],\"near\":%"PRIu64",\"divisions\":%"PRIu64",\"stallCycles\":%"PRIu64",\"medianCyclesPerVertex\":{\"portable\":%.2f,\"divuRaw\":%.2f,\"divuStallModel\":%.2f,\"divuPlusLatencyPerDivision\":%.2f},",
  tnear,tdiv,tstall,cyc[0][mid]/nv,raw[mid]/nv,cyc[1][mid]/nv,plus[mid]/nv);
 printf("\"maxCyclesPerVertex\":{\"portable\":%.2f,\"divuStallModel\":%.2f,\"divuPlusLatencyPerDivision\":%.2f},\"instructionsPerVertex\":{\"portable\":%.2f,\"divu\":%.2f}},",
  cyc[0][VIEW_TIME_POSES-1]/nv,cyc[1][VIEW_TIME_POSES-1]/nv,plus[VIEW_TIME_POSES-1]/nv,tinst[0]/(nv*VIEW_TIME_POSES),tinst[1]/(nv*VIEW_TIME_POSES));
 return capture_edges();
}
