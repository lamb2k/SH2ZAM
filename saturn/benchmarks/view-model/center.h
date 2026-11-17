/* SPDX-License-Identifier: GPL-2.0-or-later
 * Fractional-centre section of the view capture (view-center-1), included by
 * round.h after its own definitions. Copyright (c) 2026 SH2ZAM Contributors.
 *
 * Exactness: every _center entry point, portable and divider forms, run on
 * the SH-2 against an oracle written here from the contract text (the Q16
 * position center16 +/- q truncated toward zero, quotients clamped to
 * +/-(2^31 - 1), guard planes from the Q16 centre). Centres are whole pixels
 * plus or minus one unit, plus or minus half a pixel, random int32 values,
 * extremes and the requester's 193 -/+ 1/65536. Points are aimed at exact
 * row and column boundaries as well as between them.
 * Equivalence: at whole-pixel centres, each _center entry point and its
 * _trunc form run on the same inputs and must write the same bytes.
 * Timing: each view-round-1 timing fixture through the _trunc function
 * (centre 160, 112) and the _center function (centre 160, 112 + 1/65536)
 * back to back. Then guardsrc.h (view-guard-src-1), included by round.h
 * before its capture function, runs last. */
#define CENTER_BATCH_POSES 256u
#define CENTER_BATCH_COUNT 1024u
#define CENTER_PROJECT_TRIALS 256u
#define CENTER_EDGE_TRIALS 8192u
#define CENTER_CLIP_TRIALS 16384u
#define CENTER_GUARD_TRIALS 16384u
#define CENTER_EQUIV_TRIALS 1024u
#define CENTER_TIME_TRIALS 64u

struct center_cam {struct view_cam c;int32_t cx16,cy16;};

static __int128 center_floor(__int128 n) {return view_floor(n,65536);}
/* A Q16 centre near pixel k: k*65536 +/- 1, +/- 32768, whole, a random
 * fraction, a random int32, an extreme, or 193 -/+ 1/65536. */
static int32_t center_pick(int32_t k) {
 static const int32_t special[]={INT32_MIN,INT32_MIN+1,INT32_MAX,INT32_MAX-65535,-65537,-65536,-1,0,1,65535,65536,12648447,12648449};
 const __int128 b=(__int128)k*65536;
 switch(view_word()%9u) {
  case 0: return (int32_t)(b+1);
  case 1: return (int32_t)(b-1);
  case 2: return (int32_t)(b+32768);
  case 3: return (int32_t)(b-32768);
  case 4: return (int32_t)view_word();
  case 5: return special[view_word()%(sizeof(special)/sizeof(special[0]))];
  case 6: return (view_word()&1u)?12648447:12648449;
  case 7: return (int32_t)b;
  default: return (int32_t)(b+view_range(0,65535));
 }
}
static struct center_cam center_random(double m[3][3],double eye[3]) {
 struct center_cam cc;
 cc.c=view_random_cam(m,eye,0);
 cc.cx16=center_pick(cc.c.cx);cc.cy16=center_pick(cc.c.cy);
 cc.c.cx=(int16_t)center_floor(cc.cx16);cc.c.cy=(int16_t)center_floor(cc.cy16);
 return cc;
}
/* The camera block at _view_camera (for the _trunc forms) and the centre
 * view at _view_center: the same 60 bytes, then center_x16, center_y16. */
static void center_store(const struct center_cam *cc) {
 view_store_cam(&cc->c);
 const u32 a=sym("_view_camera"),b=sym("_view_center");
 for(u32 i=0;i<60;i++)wb(&cpu,b+i,rb(&cpu,a+i));
 wl(&cpu,b+60,(u32)cc->cx16);wl(&cpu,b+64,(u32)cc->cy16);
}
static struct view_vtx center_project(const struct center_cam *cc,const __int128 v[3]) {
 const struct view_cam *c=&cc->c;
 struct view_vtx o;memset(&o,0,sizeof(o));
 for(int i=0;i<3;i++)if(v[i]<INT32_MIN||v[i]>INT32_MAX)fail("Center oracle bound",i);
 o.x=(int32_t)v[0];o.y=(int32_t)v[1];o.z=(int32_t)v[2];
 if(v[2]<c->near_z){o.code=1;return o;}
 const __int128 qx=view_clamp(edge_trunc((__int128)c->fx*v[0],v[2]),-(__int128)INT32_MAX,INT32_MAX);
 const __int128 qy=view_clamp(edge_trunc((__int128)c->fy*v[1],v[2]),-(__int128)INT32_MAX,INT32_MAX);
 o.sx=(int16_t)view_clamp(edge_trunc((__int128)cc->cx16+qx,65536),INT16_MIN,INT16_MAX);
 o.sy=(int16_t)view_clamp(edge_trunc((__int128)cc->cy16-qy,65536),INT16_MIN,INT16_MAX);
 o.code=(uint8_t)((v[2]>c->far_z?2:0)|(o.sx<c->clip[0]?4:0)|(o.sx>c->clip[2]?8:0)|(o.sy<c->clip[1]?16:0)|(o.sy>c->clip[3]?32:0));
 return o;
}
/* Aim at a Q16 position on a pixel boundary, a unit off it or inside the
 * pixel. With z equal to the focal length the quotient is the coordinate
 * itself, so boundaries are hit exactly. */
static void center_aim(const struct center_cam *cc,__int128 v[3]) {
 const struct view_cam *c=&cc->c;
 const int exact=view_word()%2u==0;
 const int64_t top=(int64_t)c->near_z+(1<<22)>c->far_z?c->far_z:(int64_t)c->near_z+(1<<22);
 int32_t z=exact?c->fx:view_word()%4u?view_range(c->near_z,(int32_t)top):view_range(c->near_z,c->far_z);
 if(z<c->near_z)z=c->near_z;
 const __int128 target[2]={(__int128)view_range(-1200,1200)*65536-cc->cx16,(__int128)cc->cy16-(__int128)view_range(-1200,1200)*65536};
 const int32_t f[2]={c->fx,c->fy};
 for(int a=0;a<2;a++) {
  __int128 p=z==f[a]?target[a]:edge_trunc(target[a]*z,f[a]);
  switch(view_word()%5u){case 0:case 1:break;case 2:p+=view_range(-1,1);break;case 3:p+=view_range(-2,2);break;
   default:p+=view_range(-(z<1000000?z:1000000),z<1000000?z:1000000);}
  v[a]=view_clamp(p,INT32_MIN,INT32_MAX);
 }
 v[2]=z;
}
static unsigned center_clip_oracle(struct view_vtx out[6],const struct center_cam *cc,const struct view_vtx in[4]) {
 const struct view_cam *c=&cc->c;
 unsigned n=0;
 for(unsigned i=0;i<4;i++) {
  const struct view_vtx *cur=&in[i],*next=&in[(i+1)&3u];
  const int ci=cur->z>=c->near_z,ni=next->z>=c->near_z;
  if(ci)out[n++]=*cur;
  if(ci!=ni) {
   const struct view_vtx *p=ci?cur:next,*q=ci?next:cur;
   const __int128 num=(__int128)p->z-c->near_z,den=(__int128)p->z-q->z;
   const __int128 v[3]={p->x+edge_trunc(((__int128)q->x-p->x)*num,den),p->y+edge_trunc(((__int128)q->y-p->y)*num,den),c->near_z};
   out[n++]=center_project(cc,v);
  }
 }
 return n;
}
/* Guard planes with the Q16 centre: 65536 * center becomes center16. */
static __int128 center_plane_d(const struct center_cam *cc,const int16_t g[4],unsigned plane,const struct guard_pt *p) {
 const struct view_cam *c=&cc->c;
 const __int128 x=p->c[0],y=p->c[1],z=p->c[2];
 switch(plane) {
  case 0: return z-c->near_z;
  case 1: return c->far_z-z;
  case 2: return (__int128)c->fx*x-((__int128)65536*g[0]-cc->cx16)*z;
  case 3: return ((__int128)65536*g[2]-cc->cx16)*z-(__int128)c->fx*x;
  case 4: return ((__int128)cc->cy16-(__int128)65536*g[1])*z-(__int128)c->fy*y;
  default: return (__int128)c->fy*y-((__int128)cc->cy16-(__int128)65536*g[3])*z;
 }
}
static int center_edge_ok(int16_t e,int32_t c16) {
 const __int128 d=(__int128)65536*e-c16;
 return d>=-((__int128)1<<30)&&d<=((__int128)1<<30);
}
static int center_guard_oracle(struct guard_vtx *out,const struct center_cam *cc,const int16_t g[4],const struct guard_vtx *in,
 unsigned n,unsigned attrs,uint64_t *inside) {
 if(g[0]>g[2]||g[1]>g[3]||!center_edge_ok(g[0],cc->cx16)||!center_edge_ok(g[2],cc->cx16)||!center_edge_ok(g[1],cc->cy16)||!center_edge_ok(g[3],cc->cy16))
  return -1;
 struct guard_pt poly[128],next[128];
 const unsigned coords=3+attrs;
 int overflow=0;
 for(unsigned k=0;k<n;k++){memset(&poly[k],0,sizeof(poly[k]));poly[k].c[0]=in[k].v.x;poly[k].c[1]=in[k].v.y;poly[k].c[2]=in[k].v.z;
  for(unsigned a=0;a<attrs;a++)poly[k].c[3+a]=in[k].attr[a];}
 for(unsigned plane=0;plane<6&&n;plane++) {
  unsigned m=0;
  for(unsigned i=0;i<n;i++) {
   const unsigned j=(i+1)%n;
   const __int128 di=center_plane_d(cc,g,plane,&poly[i]),dj=center_plane_d(cc,g,plane,&poly[j]);
   if(di>=0)next[m++]=poly[i];
   if((di>=0)!=(dj>=0))next[m++]=di>=0?guard_cross_oracle(&poly[i],&poly[j],di,dj,plane,coords):guard_cross_oracle(&poly[j],&poly[i],dj,di,plane,coords);
  }
  if(m>GUARD_MAX_OUT)overflow=1;
  n=m;memcpy(poly,next,sizeof(poly[0])*m);
 }
 if(overflow)return -2;
 for(unsigned k=0;k<n;k++) {
  struct view_vtx q=center_project(cc,poly[k].c);
  int in_all=1;
  for(unsigned plane=0;plane<6;plane++)in_all=in_all&&center_plane_d(cc,g,plane,&poly[k])>=0;
  if(in_all){if(q.sx<g[0]||q.sx>g[2]||q.sy<g[1]||q.sy>g[3])fail("Center guard inside-planes property",k);(*inside)++;}
  q.sx=(int16_t)view_clamp(q.sx,g[0],g[2]);q.sy=(int16_t)view_clamp(q.sy,g[1],g[3]);
  q.code=(uint8_t)((q.sx<cc->c.clip[0]?4:0)|(q.sx>cc->c.clip[2]?8:0)|(q.sy<cc->c.clip[1]?16:0)|(q.sy>cc->c.clip[3]?32:0));
  memset(&out[k],0,sizeof(out[k]));out[k].v=q;
  for(unsigned a=0;a<attrs;a++)out[k].attr[a]=(int32_t)poly[k].c[3+a];
 }
 return (int)n;
}
/* Byte image of a target buffer, for the _trunc / _center comparison. */
static void center_read(unsigned char *dst,u32 a,u32 n) {for(u32 i=0;i<n;i++)dst[i]=rb(&cpu,a+i);}

static int capture_center(void) {
 const u32 op=sym("_view_out_portable"),od=sym("_view_out_divu"),pts=sym("_view_input");
 const u32 ends=sym("_view_edge_ends"),cin=sym("_view_clip_in");
 uint64_t bad=0,boundary=0,x_frac=0,y_frac=0;
 /* 1. Batch. */
 static int16_t pos[CENTER_BATCH_COUNT][3];static struct view_vtx want[CENTER_BATCH_COUNT];
 uint64_t batch=0,batch_near=0;
 view_state=UINT32_C(0xce17e001);
 for(unsigned pose=0;pose<CENTER_BATCH_POSES;pose++) {
  double m[3][3],eye[3];const struct center_cam cc=center_random(m,eye);
  view_positions(pos,CENTER_BATCH_COUNT,m,eye,0,&cc.c);
  center_store(&cc);view_store_positions(pos,CENTER_BATCH_COUNT);
  x_frac+=(cc.cx16&0xFFFF)!=0;y_frac+=(cc.cy16&0xFFFF)!=0;
  for(unsigned k=0;k<CENTER_BATCH_COUNT;k++) {
   __int128 v[3];for(int i=0;i<3;i++){v[i]=cc.c.t[i];for(int j=0;j<3;j++)v[i]+=(__int128)cc.c.r[i][j]*pos[k][j];}
   want[k]=center_project(&cc,v);batch_near+=want[k].code&1;
  }
  view_poison(op,CENTER_BATCH_COUNT);view_poison(od,CENTER_BATCH_COUNT);divu_reset();
  if(invoke("_view_center_batch_portable",CENTER_BATCH_COUNT,0,0,0).result!=1)fail("Center portable batch",pose);
  if(divu_divisions)fail("Center portable batch used the divider",pose);
  if(invoke("_view_center_batch_divu",CENTER_BATCH_COUNT,0,0,0).result!=1)fail("Center divider batch",pose);
  bad+=view_compare(op,want,CENTER_BATCH_COUNT)+view_compare(od,want,CENTER_BATCH_COUNT);
  batch+=CENTER_BATCH_COUNT;
 }
 /* 2. View-space projection: aimed and arbitrary points, junk in the tails. */
 uint64_t projected=0;
 for(unsigned t=0;t<CENTER_PROJECT_TRIALS;t++) {
  double m[3][3],eye[3];const struct center_cam cc=center_random(m,eye);center_store(&cc);
  for(unsigned k=0;k<512u;k++) {
   __int128 v[3];
   if(view_word()%4u)center_aim(&cc,v);else{v[0]=edge_coord();v[1]=edge_coord();v[2]=edge_coord();}
   struct view_vtx junk={(int32_t)v[0],(int32_t)v[1],(int32_t)v[2],(int16_t)view_word(),(int16_t)view_word(),(uint8_t)view_word()};
   edge_store_vertex(pts+20*k,&junk);want[k]=center_project(&cc,v);
   if(v[2]>=cc.c.near_z) {
    const __int128 qy=view_clamp(edge_trunc((__int128)cc.c.fy*v[1],v[2]),-(__int128)INT32_MAX,INT32_MAX);
    boundary+=(((__int128)cc.cy16-qy)%65536)==0;
   }
  }
  view_poison(op,512);view_poison(od,512);
  if(invoke("_view_center_project_portable",512,0,0,0).result!=1||invoke("_view_center_project_divu",512,0,0,0).result!=1)fail("Center project call",t);
  bad+=view_compare(op,want,512)+view_compare(od,want,512);
  projected+=512;
 }
 /* 3. Edge steps. */
 uint64_t stepped=0;
 for(unsigned t=0;t<CENTER_EDGE_TRIALS;t++) {
  double m[3][3],eye[3];const struct center_cam cc=center_random(m,eye);center_store(&cc);
  __int128 a[3],b[3];
  if(t%2u){center_aim(&cc,a);center_aim(&cc,b);}else for(int i=0;i<3;i++){a[i]=edge_coord();b[i]=edge_coord();}
  const unsigned den=(unsigned)view_range(1,65535),count=t%4u==0?1u:(unsigned)view_range(1,64);
  const unsigned first=(unsigned)view_range(0,(int32_t)den/4);
  const unsigned step=count==1?(unsigned)view_range(0,65535):(unsigned)view_range(0,(int32_t)((den-first)/(count-1)));
  struct view_vtx va={(int32_t)a[0],(int32_t)a[1],(int32_t)a[2],0,0,0},vb={(int32_t)b[0],(int32_t)b[1],(int32_t)b[2],0,0,0};
  edge_store_vertex(ends,&va);edge_store_vertex(ends+20,&vb);edge_set_args(first,step,den,count);
  const int32_t ia[3]={va.x,va.y,va.z},ib[3]={vb.x,vb.y,vb.z};
  for(unsigned k=0;k<count;k++) {
   __int128 p[3];
   for(int i=0;i<3;i++){const __int128 d=(__int128)ib[i]-ia[i];
    p[i]=ia[i]+edge_trunc(edge_trunc(d*first*65536,den)+(__int128)k*edge_trunc(d*step*65536,den),65536);}
   want[k]=center_project(&cc,p);
  }
  view_poison(op,count);view_poison(od,count);
  if(invoke("_view_center_edges_portable",0,0,0,0).result!=1||invoke("_view_center_edges_divu",0,0,0,0).result!=1)fail("Center edge call",t);
  bad+=view_compare(op,want,count)+view_compare(od,want,count);
  stepped+=count;
 }
 /* 4. Near clip: corners as a _center projection writes them. */
 uint64_t quads=0,clip_out=0;
 for(unsigned t=0;t<CENTER_CLIP_TRIALS;t++) {
  double m[3][3],eye[3];const struct center_cam cc=center_random(m,eye);center_store(&cc);
  struct view_vtx in[4],ref[6];
  for(unsigned i=0;i<4;i++) {
   __int128 v[3];center_aim(&cc,v);
   if(((t%16u)>>i)&1u)v[2]=view_range(cc.c.near_z-(1<<22)>INT32_MIN/2?cc.c.near_z-(1<<22):INT32_MIN/2,cc.c.near_z-1);
   in[i]=center_project(&cc,v);edge_store_vertex(cin+20*i,&in[i]);
  }
  const unsigned n=center_clip_oracle(ref,&cc,in);
  view_poison(op,6);view_poison(od,6);
  const u32 np=invoke("_view_center_clip_portable",0,0,0,0).result,nd=invoke("_view_center_clip_divu",0,0,0,0).result;
  if(np!=n||nd!=n)bad++;else bad+=view_compare(op,ref,n)+view_compare(od,ref,n);
  quads++;clip_out+=n;
 }
 /* 5. Guard clip, Q16 planes. */
 uint64_t guard_vertices=0,guard_overflows=0,guard_invalid=0,guard_inside=0;
 for(unsigned t=0;t<CENTER_GUARD_TRIALS;t++) {
  double m[3][3],eye[3];const struct center_cam cc=center_random(m,eye);
  int16_t g[4];guard_random_rect(&cc.c,g);
  struct guard_vtx in[8],gw[GUARD_MAX_OUT];
  const unsigned n=3+view_word()%6u,attrs=view_word()%5u;
  for(unsigned k=0;k<n;k++) {
   if(view_word()%4u==0){in[k].v.x=edge_coord();in[k].v.y=edge_coord();in[k].v.z=edge_coord();for(int i=0;i<4;i++)in[k].attr[i]=(int32_t)view_word();}
   else in[k]=guard_at(&cc.c,view_range(g[0]-3000,g[2]+3000),view_range(g[1]-3000,g[3]+3000),guard_depth(&cc.c));
  }
  guard_store(&cc.c,g,in,n,attrs);center_store(&cc);
  for(unsigned i=0;i<9*GUARD_MAX_OUT;i++){wl(&cpu,op+4*i,UINT32_C(0xA5A5A5A5));wl(&cpu,od+4*i,UINT32_C(0xA5A5A5A5));}
  const int w=center_guard_oracle(gw,&cc,g,in,n,attrs,&guard_inside);
  const int rp=(int32_t)invoke("_view_center_guard_portable",0,0,0,0).result,rd=(int32_t)invoke("_view_center_guard_divu",0,0,0,0).result;
  if(rp!=w||rd!=w)fail("Center guard count",t);
  if(w<0){if(w==-2)guard_overflows++;else guard_invalid++;
   for(unsigned i=0;i<9*GUARD_MAX_OUT;i++)if(rl(&cpu,op+4*i)!=UINT32_C(0xA5A5A5A5)||rl(&cpu,od+4*i)!=UINT32_C(0xA5A5A5A5))fail("Center guard failure wrote",t);
   continue;}
  bad+=guard_compare(op,gw,w)+guard_compare(od,gw,w);
  guard_vertices+=(uint64_t)w;
 }
 /* 6. Equivalence with view-round-1 at whole-pixel centres, byte for byte:
  * each path through its _trunc form, then its _center form, on the same
  * inputs, both outputs poisoned alike first. */
 static unsigned char img[2][2][CENTER_BATCH_COUNT*20];
 static const char *const eq_fn[5][4]={
  {"_view_round_batch_portable","_view_round_batch_divu","_view_center_batch_portable","_view_center_batch_divu"},
  {"_view_round_project_portable","_view_round_project_divu","_view_center_project_portable","_view_center_project_divu"},
  {"_view_round_edges_portable","_view_round_edges_divu","_view_center_edges_portable","_view_center_edges_divu"},
  {"_view_round_clip_portable","_view_round_clip_divu","_view_center_clip_portable","_view_center_clip_divu"},
  {"_view_round_guard_portable","_view_round_guard_divu","_view_center_guard_portable","_view_center_guard_divu"}};
 uint64_t eq_calls=0,eq_bytes=0,eq_bad=0;
 for(unsigned t=0;t<CENTER_EQUIV_TRIALS;t++) {
  double m[3][3],eye[3];struct center_cam cc;cc.c=view_random_cam(m,eye,0);
  cc.cx16=(int32_t)((__int128)cc.c.cx*65536);cc.cy16=(int32_t)((__int128)cc.c.cy*65536);
  const unsigned path=t%5u;
  u32 arg=0,bytes=0;
  if(path==0){view_positions(pos,CENTER_BATCH_COUNT,m,eye,0,&cc.c);center_store(&cc);view_store_positions(pos,CENTER_BATCH_COUNT);arg=CENTER_BATCH_COUNT;bytes=CENTER_BATCH_COUNT*20;}
  else if(path==1){center_store(&cc);for(unsigned k=0;k<512u;k++){__int128 v[3];center_aim(&cc,v);
    struct view_vtx p={(int32_t)v[0],(int32_t)v[1],(int32_t)v[2],0,0,0};edge_store_vertex(pts+20*k,&p);}arg=512;bytes=512*20;}
  else if(path==2){center_store(&cc);__int128 a[3],b[3];center_aim(&cc,a);center_aim(&cc,b);
    struct view_vtx va={(int32_t)a[0],(int32_t)a[1],(int32_t)a[2],0,0,0},vb={(int32_t)b[0],(int32_t)b[1],(int32_t)b[2],0,0,0};
    edge_store_vertex(ends,&va);edge_store_vertex(ends+20,&vb);edge_set_args(0,1,63,64);bytes=64*20;}
  else if(path==3){center_store(&cc);for(unsigned i=0;i<4;i++){__int128 v[3];center_aim(&cc,v);
    if((t>>i)&1u)v[2]=view_range(cc.c.near_z-(1<<22)>INT32_MIN/2?cc.c.near_z-(1<<22):INT32_MIN/2,cc.c.near_z-1);
    struct view_vtx q=center_project(&cc,v);edge_store_vertex(cin+20*i,&q);}bytes=6*20;}
  else{int16_t g[4];guard_random_rect(&cc.c,g);struct guard_vtx in[8];const unsigned n=3+view_word()%6u,attrs=view_word()%5u;
    for(unsigned k=0;k<n;k++)in[k]=guard_at(&cc.c,view_range(g[0]-3000,g[2]+3000),view_range(g[1]-3000,g[3]+3000),guard_depth(&cc.c));
    guard_store(&cc.c,g,in,n,attrs);center_store(&cc);bytes=GUARD_MAX_OUT*36;}
  u32 result[4];
  for(int k=0;k<4;k++) {
   const u32 out=(k&1)?od:op;
   for(u32 i=0;i<bytes/4;i++)wl(&cpu,out+4*i,UINT32_C(0xA5A5A5A5));
   result[k]=invoke(eq_fn[path][k],arg,0,0,0).result;
   center_read(img[k>>1][k&1],out,bytes);
  }
  eq_calls+=2;eq_bytes+=2*(uint64_t)bytes;
  eq_bad+=(result[0]!=result[2])+(result[1]!=result[3])+(memcmp(img[0][0],img[1][0],bytes)!=0)+(memcmp(img[0][1],img[1][1],bytes)!=0);
 }
 printf("\"center\":{\"batchVertices\":%"PRIu64",\"batchNear\":%"PRIu64",\"fractionalX\":%"PRIu64",\"fractionalY\":%"PRIu64","
  "\"projectedPoints\":%"PRIu64",\"rowBoundaryPoints\":%"PRIu64",\"steppedPoints\":%"PRIu64",\"clipQuads\":%"PRIu64",\"clipVertices\":%"PRIu64","
  "\"guardPolygons\":%u,\"guardVertices\":%"PRIu64",\"guardOverflows\":%"PRIu64",\"guardInvalid\":%"PRIu64",\"guardInsidePlanes\":%"PRIu64","
  "\"mismatches\":%"PRIu64",\"equivalence\":{\"calls\":%"PRIu64",\"bytes\":%"PRIu64",\"mismatches\":%"PRIu64"},",
  batch,batch_near,x_frac,y_frac,projected,boundary,stepped,quads,clip_out,CENTER_GUARD_TRIALS,guard_vertices,guard_overflows,
  guard_invalid,guard_inside,bad,eq_calls,eq_bytes,eq_bad);
 if(bad)fail("Center mismatch",(u32)bad);
 if(eq_bad)fail("Center equivalence mismatch",(u32)eq_bad);

 /* Timing: the view-round-1 fixtures, _trunc at centre (160, 112) and
  * _center at (160, 112 + 1/65536). cyc[row][0 trunc portable, 1 trunc
  * divu, 2 center portable, 3 center divu][trial]. */
 static u32 cyc[7][4][CENTER_TIME_TRIALS];
 static const char *const trunc_fn[7][2]={{"_view_round_batch_portable","_view_round_batch_divu"},{"_view_round_project_portable","_view_round_project_divu"},
  {"_view_round_edges_portable","_view_round_edges_divu"},{"_view_round_clip_portable","_view_round_clip_divu"},
  {"_view_round_guard_portable","_view_round_guard_divu"},{"_view_round_guard_portable","_view_round_guard_divu"},{"_view_round_guard_portable","_view_round_guard_divu"}};
 static const char *const center_fn[7][2]={{"_view_center_batch_portable","_view_center_batch_divu"},{"_view_center_project_portable","_view_center_project_divu"},
  {"_view_center_edges_portable","_view_center_edges_divu"},{"_view_center_clip_portable","_view_center_clip_divu"},
  {"_view_center_guard_portable","_view_center_guard_divu"},{"_view_center_guard_portable","_view_center_guard_divu"},{"_view_center_guard_portable","_view_center_guard_divu"}};
 uint64_t divisions[7][2]={{0}};
 view_state=UINT32_C(0x7e0dce17);
 for(unsigned t=0;t<CENTER_TIME_TRIALS;t++) {
  double m[3][3],eye[3];struct center_cam cc;cc.c=view_random_cam(m,eye,1);
  cc.cx16=(int32_t)cc.c.cx*65536;cc.cy16=(int32_t)cc.c.cy*65536+1;
  const struct view_cam c=cc.c;
  const double half=160.0*65536.0/c.fx;
  for(int row=0;row<7;row++) {
   u32 arg=0;
   if(row==0){view_positions(pos,512,m,eye,1,&cc.c);center_store(&cc);view_store_positions(pos,512);arg=512;}
   else center_store(&cc);
   if(row==1){for(unsigned k=0;k<512u;k++){const double z=(2.0+(view_word()%65536)/65536.0*3000.0)*16384.0;
     struct view_vtx p={(int32_t)(((view_word()%65536)/32768.0-1.0)*half*z),(int32_t)(((view_word()%65536)/32768.0-1.0)*half*z*0.7),(int32_t)z,0,0,0};
     edge_store_vertex(pts+20*k,&p);}arg=512;}
   if(row==2){for(int j=0;j<2;j++){const double z=(2.0+(view_word()%65536)/65536.0*3000.0)*16384.0;
     struct view_vtx e={(int32_t)(((view_word()%65536)/32768.0-1.0)*half*z),(int32_t)(((view_word()%65536)/32768.0-1.0)*half*z*0.7),(int32_t)z,0,0,0};
     edge_store_vertex(ends+20*j,&e);}edge_set_args(0,1,63,64);}
   if(row==3){for(int i=0;i<4;i++){const double z=i<2?(2.0+(view_word()%65536)/65536.0*40.0)*16384.0:-(1.0+(view_word()%65536)/65536.0*40.0)*16384.0;
     const double az=z<0?-z:z;
     const __int128 v[3]={(int32_t)(((view_word()%65536)/32768.0-1.0)*half*az),(int32_t)(((view_word()%65536)/32768.0-1.0)*half*az*0.7),(int32_t)z};
     struct view_vtx q=center_project(&cc,v);edge_store_vertex(cin+20*i,&q);}}
   if(row>=4){const int s=row-4;const int16_t g[4]={-900,-900,900,900};
    const int32_t z0=(int32_t)((20.0+(view_word()%65536)/65536.0*200.0)*16384.0);
    struct guard_vtx q[4]={guard_at(&c,40,40,z0),guard_at(&c,280,40,z0),guard_at(&c,280,180,z0),guard_at(&c,40,180,z0)};
    if(s>=1)q[0]=guard_at(&c,-2840,112,z0);
    if(s==2){q[2]=guard_at(&c,280,180,z0);q[2].v.z=-z0;q[3].v.z=-z0;q[3].v.x=-q[3].v.x;}
    guard_store(&c,g,q,4,2);center_store(&cc);}
   for(int k=0;k<4;k++) {
    const char *fn=k<2?trunc_fn[row][k]:center_fn[row][k-2];
    divu_reset();
    const struct cost cs=invoke(fn,arg,0,0,0);
    if(row==0&&cs.result!=1)fail("Center timing batch",t);
    if(row==3&&cs.result!=4)fail("Center timing clip shape",t);
    cyc[row][k][t]=cs.cycles;
    if(k&1)divisions[row][k>>1]+=divu_divisions;
   }
  }
 }
 static const char *const names[7]={"batch","project","edgeSteps","clipNear","guardInside","guardSide","guardNearSide"};
 static const double per[7]={512,512,64,1,1,1,1};
 printf("\"timing\":{\"trials\":%u,\"centerQ16\":[10485760,7340033]",CENTER_TIME_TRIALS);
 for(int row=0;row<7;row++) {
  for(int k=0;k<4;k++)qsort(cyc[row][k],CENTER_TIME_TRIALS,4,view_cmp_u32);
  const unsigned mid=CENTER_TIME_TRIALS/2;
  if(divisions[row][0]!=divisions[row][1])fail("Center timing division count differs",(u32)row);
  printf(",\"%s\":{\"per\":%.0f,\"divisions\":%"PRIu64",\"median\":{\"truncPortable\":%.2f,\"truncDivu\":%.2f,\"centerPortable\":%.2f,\"centerDivu\":%.2f}}",
   names[row],per[row],divisions[row][1],cyc[row][0][mid]/per[row],cyc[row][1][mid]/per[row],cyc[row][2][mid]/per[row],cyc[row][3][mid]/per[row]);
 }
 printf("}},");
 return capture_guard_src();
}
