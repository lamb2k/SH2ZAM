/* SPDX-License-Identifier: GPL-2.0-or-later
 * Guard-band clip section of the view capture (view-guard-1), included by
 * edges.h. Copyright (c) 2026 SH2ZAM Contributors.
 *
 * Exactness: random polygons (3..8 corners, planar convex and twisted),
 * random cameras and guard rectangles; both entry points run on the SH-2 and
 * are compared with an oracle written here from the contract text.
 * Timing: the port's camera and its -900..900 band, quads with two
 * attributes in three shapes. Then merge.h (view-merge-1). */
#define GUARD_TRIALS 65536u
#define GUARD_TIME_TRIALS 64u
#define GUARD_MAX_OUT 16

#include "merge.h"
struct guard_vtx {struct view_vtx v;int32_t attr[4];};
struct guard_pt {__int128 c[7];};

static __int128 guard_plane_d(const struct view_cam *c,const int16_t g[4],unsigned plane,const struct guard_pt *p) {
 const __int128 x=p->c[0],y=p->c[1],z=p->c[2];
 switch(plane) {
  case 0: return z-c->near_z;
  case 1: return c->far_z-z;
  case 2: return (__int128)c->fx*x-(__int128)65536*(g[0]-c->cx)*z;
  case 3: return (__int128)65536*(g[2]-c->cx)*z-(__int128)c->fx*x;
  case 4: return (__int128)65536*(c->cy-g[1])*z-(__int128)c->fy*y;
  default: return (__int128)c->fy*y-(__int128)65536*(c->cy-g[3])*z;
 }
}
/* Crossing measured from the inside end i (di >= 0 > dout). */
static struct guard_pt guard_cross_oracle(const struct guard_pt *i,const struct guard_pt *o,__int128 di,__int128 dout,unsigned plane,unsigned coords) {
 const __int128 num=di,den=di-dout;
 struct guard_pt p;memset(&p,0,sizeof(p));
 __int128 t=0;
 if(plane>=2){unsigned s=0;while((den>>s)>=((__int128)1<<31))s++;t=((num>>s)<<30)/(den>>s);}
 for(unsigned k=0;k<coords;k++) {
  const __int128 delta=o->c[k]-i->c[k],m=delta<0?-delta:delta;
  const __int128 off=plane>=2?(m*t)>>30:m*num/den;
  p.c[k]=i->c[k]+(delta<0?-off:off);
 }
 return p;
}
static int guard_oracle(struct guard_vtx *out,const struct view_cam *c,const int16_t g[4],const struct guard_vtx *in,unsigned n,unsigned attrs) {
 struct guard_pt poly[128],next[128];
 const unsigned coords=3+attrs;
 int overflow=0;
 for(unsigned k=0;k<n;k++){memset(&poly[k],0,sizeof(poly[k]));poly[k].c[0]=in[k].v.x;poly[k].c[1]=in[k].v.y;poly[k].c[2]=in[k].v.z;
  for(unsigned a=0;a<attrs;a++)poly[k].c[3+a]=in[k].attr[a];}
 for(unsigned plane=0;plane<6&&n;plane++) {
  unsigned m=0;
  for(unsigned i=0;i<n;i++) {
   const unsigned j=(i+1)%n;
   const __int128 di=guard_plane_d(c,g,plane,&poly[i]),dj=guard_plane_d(c,g,plane,&poly[j]);
   if(di>=0)next[m++]=poly[i];
   if((di>=0)!=(dj>=0))next[m++]=di>=0?guard_cross_oracle(&poly[i],&poly[j],di,dj,plane,coords):guard_cross_oracle(&poly[j],&poly[i],dj,di,plane,coords);
  }
  if(m>GUARD_MAX_OUT)overflow=1;
  n=m;memcpy(poly,next,sizeof(poly[0])*m);
 }
 if(overflow)return -2;
 for(unsigned k=0;k<n;k++) {
  struct view_vtx q=view_project(c,poly[k].c,NULL);
  q.sx=(int16_t)view_clamp(q.sx,g[0],g[2]);q.sy=(int16_t)view_clamp(q.sy,g[1],g[3]);
  q.code=(uint8_t)((q.sx<c->clip[0]?4:0)|(q.sx>c->clip[2]?8:0)|(q.sy<c->clip[1]?16:0)|(q.sy>c->clip[3]?32:0));
  memset(&out[k],0,sizeof(out[k]));out[k].v=q;
  for(unsigned a=0;a<attrs;a++)out[k].attr[a]=(int32_t)poly[k].c[3+a];
 }
 return (int)n;
}
static void guard_store(const struct view_cam *c,const int16_t g[4],const struct guard_vtx *in,unsigned n,unsigned attrs) {
 view_store_cam(c);
 const u32 a=sym("_view_input"),r=sym("_view_guard_rect"),args=sym("_view_guard_args");
 for(unsigned k=0;k<n;k++) {
  const u32 p=a+36*k;
  wl(&cpu,p,(u32)in[k].v.x);wl(&cpu,p+4,(u32)in[k].v.y);wl(&cpu,p+8,(u32)in[k].v.z);
  wl(&cpu,p+12,view_word());wl(&cpu,p+16,view_word());          /* ignored on input */
  for(unsigned i=0;i<4;i++)wl(&cpu,p+20+4*i,(u32)in[k].attr[i]);
 }
 for(int i=0;i<4;i++)ww(&cpu,r+2*i,(u16)g[i]);
 wl(&cpu,args,n);wl(&cpu,args+4,attrs);
}
static uint64_t guard_compare(u32 out,const struct guard_vtx *w,int n) {
 uint64_t bad=0;
 for(int k=0;k<n;k++) {
  const u32 a=out+36*(u32)k;
  int ok=rl(&cpu,a)==(u32)w[k].v.x&&rl(&cpu,a+4)==(u32)w[k].v.y&&rl(&cpu,a+8)==(u32)w[k].v.z&&rw(&cpu,a+12)==(u16)w[k].v.sx&&
   rw(&cpu,a+14)==(u16)w[k].v.sy&&rb(&cpu,a+16)==w[k].v.code&&!rb(&cpu,a+17)&&!rb(&cpu,a+18)&&!rb(&cpu,a+19);
  for(int i=0;i<4;i++)ok=ok&&rl(&cpu,a+20+4*(u32)i)==(u32)w[k].attr[i];
  bad+=!ok;
 }
 return bad;
}
/* A corner at screen (sx, sy) and depth z for camera c. */
static struct guard_vtx guard_at(const struct view_cam *c,int32_t sx,int32_t sy,int32_t z) {
 struct guard_vtx p;memset(&p,0,sizeof(p));
 p.v.x=(int32_t)view_clamp(edge_trunc((__int128)(sx-c->cx)*65536*z,c->fx),INT32_MIN,INT32_MAX);
 p.v.y=(int32_t)view_clamp(edge_trunc((__int128)(c->cy-sy)*65536*z,c->fy),INT32_MIN,INT32_MAX);
 p.v.z=z;
 for(int i=0;i<4;i++)p.attr[i]=(int32_t)view_word();
 return p;
}
static void guard_random_rect(const struct view_cam *c,int16_t g[4]) {
 switch(view_word()%3u) {
  case 0: g[0]=-900;g[1]=-900;g[2]=900;g[3]=900;break;
  case 1: g[0]=c->clip[0];g[1]=c->clip[1];g[2]=c->clip[2];g[3]=c->clip[3];break;
  default: g[0]=(int16_t)view_range(c->cx-2000,c->cx+200);g[2]=(int16_t)view_range(g[0],c->cx+2000);
   g[1]=(int16_t)view_range(c->cy-2000,c->cy+200);g[3]=(int16_t)view_range(g[1],c->cy+2000);
 }
}
static int32_t guard_depth(const struct view_cam *c) {
 switch(view_word()%5u) {
  case 0: return c->near_z;
  case 1: return c->far_z;
  case 2: return edge_coord();
  case 3: return view_range(1,c->near_z+(1<<22)>0?c->near_z+(1<<22):INT32_MAX);
  default: return view_range(c->near_z,c->far_z);
 }
}
static int capture_guard(void) {
 const u32 op=sym("_view_out_portable"),od=sym("_view_out_divu");
 uint64_t vertices=0,bad=0,overflows=0,clipped=0,empty=0,sizes[GUARD_MAX_OUT+1]={0};
 for(unsigned t=0;t<GUARD_TRIALS;t++) {
  double m[3][3],eye[3];const struct view_cam c=view_random_cam(m,eye,0);
  int16_t g[4];guard_random_rect(&c,g);
  struct guard_vtx in[8],want[GUARD_MAX_OUT];
  const unsigned n=3+view_word()%6u,attrs=view_word()%5u;
  for(unsigned k=0;k<n;k++) {
   if(view_word()%4u==0){in[k].v.x=edge_coord();in[k].v.y=edge_coord();in[k].v.z=edge_coord();for(int i=0;i<4;i++)in[k].attr[i]=(int32_t)view_word();}
   else in[k]=guard_at(&c,view_range(g[0]-3000,g[2]+3000),view_range(g[1]-3000,g[3]+3000),guard_depth(&c));
  }
  guard_store(&c,g,in,n,attrs);
  for(unsigned i=0;i<9*GUARD_MAX_OUT;i++){wl(&cpu,op+4*i,UINT32_C(0xA5A5A5A5));wl(&cpu,od+4*i,UINT32_C(0xA5A5A5A5));}
  const int rw_=guard_oracle(want,&c,g,in,n,attrs);
  const int rp=(int32_t)invoke("_view_guard_portable",0,0,0,0).result,rd=(int32_t)invoke("_view_guard_divu",0,0,0,0).result;
  if(rp!=rw_||rd!=rw_)fail("Guard count",t);
  if(rw_<0) {
   overflows++;
   for(unsigned i=0;i<9*GUARD_MAX_OUT;i++)if(rl(&cpu,op+4*i)!=UINT32_C(0xA5A5A5A5)||rl(&cpu,od+4*i)!=UINT32_C(0xA5A5A5A5))fail("Guard overflow wrote",t);
   continue;
  }
  sizes[rw_]++;empty+=rw_==0;clipped+=(unsigned)rw_!=n;
  bad+=guard_compare(op,want,rw_)+guard_compare(od,want,rw_);
  vertices+=(uint64_t)rw_;
 }
 if(bad)fail("Guard mismatch",(u32)bad);
 printf("\"guard\":{\"trials\":%u,\"vertices\":%"PRIu64",\"mismatches\":%"PRIu64",\"overflows\":%"PRIu64",\"empty\":%"PRIu64",\"countChanged\":%"PRIu64",\"sizes\":[",
  GUARD_TRIALS,vertices,bad,overflows,empty,clipped);
 for(int k=0;k<=GUARD_MAX_OUT;k++)printf("%s%"PRIu64,k?",":"",sizes[k]);
 printf("],");

 /* Timing on the port's camera and band; quads carry two attributes (u, v).
  * inside: all four corners on screen (no crossing)
  * side:   one corner far left of the band, as in the port's case
  * near:   two corners behind the camera and one far left of the band */
 static u32 cyc[3][2][GUARD_TIME_TRIALS];
 uint64_t divs[3]={0};
 const char *const shape[3]={"inside","side","nearSide"};
 const int expect[3]={4,5,0};   /* nearSide: count depends on the camera */
 uint64_t outs[3]={0};
 for(unsigned t=0;t<GUARD_TIME_TRIALS;t++) {
  double m[3][3],eye[3];const struct view_cam c=view_random_cam(m,eye,1);
  const int16_t g[4]={-900,-900,900,900};
  for(int s=0;s<3;s++) {
   const int32_t z0=(int32_t)((20.0+(view_word()%65536)/65536.0*200.0)*16384.0);
   struct guard_vtx q[4]={guard_at(&c,40,40,z0),guard_at(&c,280,40,z0),guard_at(&c,280,180,z0),guard_at(&c,40,180,z0)};
   if(s>=1)q[0]=guard_at(&c,-2840,112,z0);
   if(s==2){q[2]=guard_at(&c,280,180,z0);q[2].v.z=-z0;q[3].v.z=-z0;q[3].v.x=-q[3].v.x;}
   guard_store(&c,g,q,4,2);
   struct guard_vtx want[GUARD_MAX_OUT];
   const int w=guard_oracle(want,&c,g,q,4,2);
   const struct cost cp=invoke("_view_guard_portable",0,0,0,0);divu_reset();
   const struct cost cd=invoke("_view_guard_divu",0,0,0,0);divs[s]+=divu_divisions;
   if((int32_t)cp.result!=w||(int32_t)cd.result!=w||guard_compare(op,want,w)||guard_compare(od,want,w))fail("Guard timing result",t);
   if(expect[s]?w!=expect[s]:w<3)fail("Guard timing shape",(u32)(s*1000+w));
   outs[s]+=(uint64_t)w;
   cyc[s][0][t]=cp.cycles;cyc[s][1][t]=cd.cycles;
  }
 }
 printf("\"timing\":{\"trials\":%u,\"band\":[-900,-900,900,900],\"attrs\":2",GUARD_TIME_TRIALS);
 for(int s=0;s<3;s++) {
  qsort(cyc[s][0],GUARD_TIME_TRIALS,4,view_cmp_u32);qsort(cyc[s][1],GUARD_TIME_TRIALS,4,view_cmp_u32);
  printf(",\"%s\":{\"meanOutputVertices\":%.2f,\"medianCycles\":{\"portable\":%u,\"divuStallModel\":%u},\"divisionsPerCall\":%.2f}",
   shape[s],(double)outs[s]/GUARD_TIME_TRIALS,cyc[s][0][GUARD_TIME_TRIALS/2],cyc[s][1][GUARD_TIME_TRIALS/2],(double)divs[s]/GUARD_TIME_TRIALS);
 }
 printf("}},");
 return capture_merge();
}
