/* SPDX-License-Identifier: GPL-2.0-or-later
 * Final-coordinate truncation section of the view capture (view-round-1),
 * included by guard.h after its own definitions. Copyright (c) 2026 SH2ZAM
 * Contributors.
 *
 * Exactness: every _trunc entry point, portable and divider forms, run on the
 * SH-2 against an oracle written here from the contract text (the Q16 screen
 * position truncated toward zero, quotients clamped to +/-(2^31 - 1)): batch,
 * view-space projection (points aimed exactly at pixels and between them),
 * edge steps, near clip and guard clip.
 * Timing: the same fixtures run through the view-contract-1 function and its
 * _trunc form back to back, so each row is a paired comparison. */
#define ROUND_BATCH_POSES 256u
#define ROUND_BATCH_COUNT 1024u
#define ROUND_PROJECT_TRIALS 256u
#define ROUND_EDGE_TRIALS 8192u
#define ROUND_CLIP_TRIALS 16384u
#define ROUND_GUARD_TRIALS 16384u
#define ROUND_TIME_TRIALS 64u

/* view_project with the view-round-1 pixel rule. */
static struct view_vtx round_project(const struct view_cam *c,const __int128 v[3]) {
 struct view_vtx o;memset(&o,0,sizeof(o));
 for(int i=0;i<3;i++)if(v[i]<INT32_MIN||v[i]>INT32_MAX)fail("Round oracle bound",i);
 o.x=(int32_t)v[0];o.y=(int32_t)v[1];o.z=(int32_t)v[2];
 if(v[2]<c->near_z){o.code=1;return o;}
 const __int128 qx=view_clamp(edge_trunc((__int128)c->fx*v[0],v[2]),-(__int128)INT32_MAX,INT32_MAX);
 const __int128 qy=view_clamp(edge_trunc((__int128)c->fy*v[1],v[2]),-(__int128)INT32_MAX,INT32_MAX);
 o.sx=(int16_t)view_clamp(edge_trunc((__int128)65536*c->cx+qx,65536),INT16_MIN,INT16_MAX);
 o.sy=(int16_t)view_clamp(edge_trunc((__int128)65536*c->cy-qy,65536),INT16_MIN,INT16_MAX);
 o.code=(uint8_t)((v[2]>c->far_z?2:0)|(o.sx<c->clip[0]?4:0)|(o.sx>c->clip[2]?8:0)|(o.sy<c->clip[1]?16:0)|(o.sy>c->clip[3]?32:0));
 return o;
}
/* How the rule differs from view-contract-1 on one point, for the counters. */
static void round_tally(const struct view_cam *c,const __int128 v[3],uint64_t *x_up,uint64_t *y_down) {
 const struct view_vtx a=round_project(c,v),b=view_project(c,v,NULL);
 *x_up+=a.sx!=b.sx;*y_down+=a.sy!=b.sy;
}
/* A view-space point aimed at a pixel: exactly on it when the focal length
 * divides, or a unit or up to a pixel off; both signs of screen position. */
static void round_aim(const struct view_cam *c,__int128 v[3]) {
 const int64_t top=(int64_t)c->near_z+(1<<22)>c->far_z?c->far_z:(int64_t)c->near_z+(1<<22);
 const int32_t z=view_word()%4u?view_range(c->near_z,(int32_t)top):view_range(c->near_z,c->far_z);
 const int32_t k[2]={view_range(-1200,1200)-c->cx,c->cy-view_range(-1200,1200)},f[2]={c->fx,c->fy};
 for(int a=0;a<2;a++) {
  __int128 p=edge_trunc((__int128)k[a]*65536*z,f[a]);
  switch(view_word()%4u){case 0:break;case 1:p+=view_range(-2,2);break;default:p+=view_range(-(z<1000000?z:1000000),z<1000000?z:1000000);}
  v[a]=view_clamp(p,INT32_MIN,INT32_MAX);
 }
 v[2]=z;
}
static unsigned round_clip_oracle(struct view_vtx out[6],const struct view_cam *c,const struct view_vtx in[4]) {
 unsigned n=0;
 for(unsigned i=0;i<4;i++) {
  const struct view_vtx *cur=&in[i],*next=&in[(i+1)&3u];
  const int ci=cur->z>=c->near_z,ni=next->z>=c->near_z;
  if(ci)out[n++]=*cur;
  if(ci!=ni) {
   const struct view_vtx *p=ci?cur:next,*q=ci?next:cur;
   const __int128 num=(__int128)p->z-c->near_z,den=(__int128)p->z-q->z;
   const __int128 v[3]={p->x+edge_trunc(((__int128)q->x-p->x)*num,den),p->y+edge_trunc(((__int128)q->y-p->y)*num,den),c->near_z};
   out[n++]=round_project(c,v);
  }
 }
 return n;
}
static int round_guard_oracle(struct guard_vtx *out,const struct view_cam *c,const int16_t g[4],const struct guard_vtx *in,unsigned n,unsigned attrs) {
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
  struct view_vtx q=round_project(c,poly[k].c);
  q.sx=(int16_t)view_clamp(q.sx,g[0],g[2]);q.sy=(int16_t)view_clamp(q.sy,g[1],g[3]);
  q.code=(uint8_t)((q.sx<c->clip[0]?4:0)|(q.sx>c->clip[2]?8:0)|(q.sy<c->clip[1]?16:0)|(q.sy>c->clip[3]?32:0));
  memset(&out[k],0,sizeof(out[k]));out[k].v=q;
  for(unsigned a=0;a<attrs;a++)out[k].attr[a]=(int32_t)poly[k].c[3+a];
 }
 return (int)n;
}
static void round_median(u32 *a,unsigned n) {qsort(a,n,4,view_cmp_u32);}

static int capture_round(void) {
 const u32 op=sym("_view_out_portable"),od=sym("_view_out_divu"),pts=sym("_view_input");
 const u32 ends=sym("_view_edge_ends"),cin=sym("_view_clip_in");
 uint64_t bad=0,x_up=0,y_down=0;
 /* 1. Batch. */
 static int16_t pos[ROUND_BATCH_COUNT][3];static struct view_vtx want[ROUND_BATCH_COUNT];
 uint64_t batch=0,batch_near=0;
 view_state=UINT32_C(0x40d1e001);
 for(unsigned pose=0;pose<ROUND_BATCH_POSES;pose++) {
  double m[3][3],eye[3];struct view_cam c=view_random_cam(m,eye,0);
  view_positions(pos,ROUND_BATCH_COUNT,m,eye,0,&c);
  view_store_cam(&c);view_store_positions(pos,ROUND_BATCH_COUNT);
  for(unsigned k=0;k<ROUND_BATCH_COUNT;k++) {
   __int128 v[3];for(int i=0;i<3;i++){v[i]=c.t[i];for(int j=0;j<3;j++)v[i]+=(__int128)c.r[i][j]*pos[k][j];}
   want[k]=round_project(&c,v);batch_near+=want[k].code&1;round_tally(&c,v,&x_up,&y_down);
  }
  view_poison(op,ROUND_BATCH_COUNT);view_poison(od,ROUND_BATCH_COUNT);divu_reset();
  if(invoke("_view_round_batch_portable",ROUND_BATCH_COUNT,0,0,0).result!=1)fail("Round portable batch",pose);
  if(divu_divisions)fail("Round portable batch used the divider",pose);
  if(invoke("_view_round_batch_divu",ROUND_BATCH_COUNT,0,0,0).result!=1)fail("Round divider batch",pose);
  bad+=view_compare(op,want,ROUND_BATCH_COUNT)+view_compare(od,want,ROUND_BATCH_COUNT);
  batch+=ROUND_BATCH_COUNT;
 }
 /* 2. View-space projection: aimed and arbitrary points, junk in the tails. */
 uint64_t projected=0;
 for(unsigned t=0;t<ROUND_PROJECT_TRIALS;t++) {
  double m[3][3],eye[3];struct view_cam c=view_random_cam(m,eye,0);view_store_cam(&c);
  for(unsigned k=0;k<512u;k++) {
   __int128 v[3];
   if(view_word()%4u)round_aim(&c,v);else{v[0]=edge_coord();v[1]=edge_coord();v[2]=edge_coord();}
   struct view_vtx junk={(int32_t)v[0],(int32_t)v[1],(int32_t)v[2],(int16_t)view_word(),(int16_t)view_word(),(uint8_t)view_word()};
   edge_store_vertex(pts+20*k,&junk);want[k]=round_project(&c,v);round_tally(&c,v,&x_up,&y_down);
  }
  view_poison(op,512);view_poison(od,512);
  if(invoke("_view_round_project_portable",512,0,0,0).result!=1||invoke("_view_round_project_divu",512,0,0,0).result!=1)fail("Round project call",t);
  bad+=view_compare(op,want,512)+view_compare(od,want,512);
  projected+=512;
 }
 /* 3. Edge steps. */
 uint64_t stepped=0;
 for(unsigned t=0;t<ROUND_EDGE_TRIALS;t++) {
  double m[3][3],eye[3];struct view_cam c=view_random_cam(m,eye,0);view_store_cam(&c);
  __int128 a[3],b[3];
  if(t%2u){round_aim(&c,a);round_aim(&c,b);}else for(int i=0;i<3;i++){a[i]=edge_coord();b[i]=edge_coord();}
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
   want[k]=round_project(&c,p);
  }
  view_poison(op,count);view_poison(od,count);
  if(invoke("_view_round_edges_portable",0,0,0,0).result!=1||invoke("_view_round_edges_divu",0,0,0,0).result!=1)fail("Round edge call",t);
  bad+=view_compare(op,want,count)+view_compare(od,want,count);
  stepped+=count;
 }
 /* 4. Near clip: corners as a _trunc projection writes them. */
 uint64_t quads=0,clip_out=0;
 for(unsigned t=0;t<ROUND_CLIP_TRIALS;t++) {
  double m[3][3],eye[3];struct view_cam c=view_random_cam(m,eye,0);view_store_cam(&c);
  struct view_vtx in[4],ref[6];
  for(unsigned i=0;i<4;i++) {
   __int128 v[3];round_aim(&c,v);
   if(((t%16u)>>i)&1u)v[2]=view_range(c.near_z-(1<<22)>INT32_MIN/2?c.near_z-(1<<22):INT32_MIN/2,c.near_z-1);
   in[i]=round_project(&c,v);edge_store_vertex(cin+20*i,&in[i]);
  }
  const unsigned n=round_clip_oracle(ref,&c,in);
  view_poison(op,6);view_poison(od,6);
  const u32 np=invoke("_view_round_clip_portable",0,0,0,0).result,nd=invoke("_view_round_clip_divu",0,0,0,0).result;
  if(np!=n||nd!=n)bad++;else bad+=view_compare(op,ref,n)+view_compare(od,ref,n);
  quads++;clip_out+=n;
 }
 /* 5. Guard clip. */
 uint64_t guard_vertices=0,guard_overflows=0;
 for(unsigned t=0;t<ROUND_GUARD_TRIALS;t++) {
  double m[3][3],eye[3];const struct view_cam c=view_random_cam(m,eye,0);
  int16_t g[4];guard_random_rect(&c,g);
  struct guard_vtx in[8],gw[GUARD_MAX_OUT];
  const unsigned n=3+view_word()%6u,attrs=view_word()%5u;
  for(unsigned k=0;k<n;k++) {
   if(view_word()%4u==0){in[k].v.x=edge_coord();in[k].v.y=edge_coord();in[k].v.z=edge_coord();for(int i=0;i<4;i++)in[k].attr[i]=(int32_t)view_word();}
   else in[k]=guard_at(&c,view_range(g[0]-3000,g[2]+3000),view_range(g[1]-3000,g[3]+3000),guard_depth(&c));
  }
  guard_store(&c,g,in,n,attrs);
  for(unsigned i=0;i<9*GUARD_MAX_OUT;i++){wl(&cpu,op+4*i,UINT32_C(0xA5A5A5A5));wl(&cpu,od+4*i,UINT32_C(0xA5A5A5A5));}
  const int w=round_guard_oracle(gw,&c,g,in,n,attrs);
  const int rp=(int32_t)invoke("_view_round_guard_portable",0,0,0,0).result,rd=(int32_t)invoke("_view_round_guard_divu",0,0,0,0).result;
  if(rp!=w||rd!=w)fail("Round guard count",t);
  if(w<0){guard_overflows++;
   for(unsigned i=0;i<9*GUARD_MAX_OUT;i++)if(rl(&cpu,op+4*i)!=UINT32_C(0xA5A5A5A5)||rl(&cpu,od+4*i)!=UINT32_C(0xA5A5A5A5))fail("Round guard overflow wrote",t);
   continue;}
  bad+=guard_compare(op,gw,w)+guard_compare(od,gw,w);
  guard_vertices+=(uint64_t)w;
 }
 printf("\"round\":{\"batchVertices\":%"PRIu64",\"batchNear\":%"PRIu64",\"projectedPoints\":%"PRIu64",\"steppedPoints\":%"PRIu64","
  "\"clipQuads\":%"PRIu64",\"clipVertices\":%"PRIu64",\"guardPolygons\":%u,\"guardVertices\":%"PRIu64",\"guardOverflows\":%"PRIu64","
  "\"xUp\":%"PRIu64",\"yDown\":%"PRIu64",\"mismatches\":%"PRIu64",",
  batch,batch_near,projected,stepped,quads,clip_out,ROUND_GUARD_TRIALS,guard_vertices,guard_overflows,x_up,y_down,bad);
 if(bad)fail("Round mismatch",(u32)bad);

 /* Timing: each fixture through the floor (view-contract-1) and _trunc entry
  * points, portable and divider. cyc[row][0 floor portable, 1 floor divu,
  * 2 trunc portable, 3 trunc divu][trial]. */
 static u32 cyc[7][4][ROUND_TIME_TRIALS];
 static const char *const floor_fn[7][2]={{"_view_run_portable","_view_run_divu"},{"_view_project_portable","_view_project_divu"},
  {"_view_edges_portable","_view_edges_divu"},{"_view_clip_portable","_view_clip_divu"},
  {"_view_guard_portable","_view_guard_divu"},{"_view_guard_portable","_view_guard_divu"},{"_view_guard_portable","_view_guard_divu"}};
 static const char *const trunc_fn[7][2]={{"_view_round_batch_portable","_view_round_batch_divu"},{"_view_round_project_portable","_view_round_project_divu"},
  {"_view_round_edges_portable","_view_round_edges_divu"},{"_view_round_clip_portable","_view_round_clip_divu"},
  {"_view_round_guard_portable","_view_round_guard_divu"},{"_view_round_guard_portable","_view_round_guard_divu"},{"_view_round_guard_portable","_view_round_guard_divu"}};
 uint64_t divisions[7][2]={{0}};
 view_state=UINT32_C(0x7e0d0004);
 for(unsigned t=0;t<ROUND_TIME_TRIALS;t++) {
  double m[3][3],eye[3];struct view_cam c=view_random_cam(m,eye,1);
  const double half=160.0*65536.0/c.fx;
  for(int row=0;row<7;row++) {
   u32 arg=0;
   if(row==0){view_positions(pos,512,m,eye,1,&c);view_store_cam(&c);view_store_positions(pos,512);arg=512;}
   else view_store_cam(&c);
   if(row==1){for(unsigned k=0;k<512u;k++){const double z=(2.0+(view_word()%65536)/65536.0*3000.0)*16384.0;
     struct view_vtx p={(int32_t)(((view_word()%65536)/32768.0-1.0)*half*z),(int32_t)(((view_word()%65536)/32768.0-1.0)*half*z*0.7),(int32_t)z,0,0,0};
     edge_store_vertex(pts+20*k,&p);}arg=512;}
   if(row==2){for(int j=0;j<2;j++){const double z=(2.0+(view_word()%65536)/65536.0*3000.0)*16384.0;
     struct view_vtx e={(int32_t)(((view_word()%65536)/32768.0-1.0)*half*z),(int32_t)(((view_word()%65536)/32768.0-1.0)*half*z*0.7),(int32_t)z,0,0,0};
     edge_store_vertex(ends+20*j,&e);}edge_set_args(0,1,63,64);}
   if(row==3){for(int i=0;i<4;i++){const double z=i<2?(2.0+(view_word()%65536)/65536.0*40.0)*16384.0:-(1.0+(view_word()%65536)/65536.0*40.0)*16384.0;
     const double az=z<0?-z:z;
     const __int128 v[3]={(int32_t)(((view_word()%65536)/32768.0-1.0)*half*az),(int32_t)(((view_word()%65536)/32768.0-1.0)*half*az*0.7),(int32_t)z};
     struct view_vtx q=round_project(&c,v);edge_store_vertex(cin+20*i,&q);}}
   if(row>=4){const int s=row-4;const int16_t g[4]={-900,-900,900,900};
    const int32_t z0=(int32_t)((20.0+(view_word()%65536)/65536.0*200.0)*16384.0);
    struct guard_vtx q[4]={guard_at(&c,40,40,z0),guard_at(&c,280,40,z0),guard_at(&c,280,180,z0),guard_at(&c,40,180,z0)};
    if(s>=1)q[0]=guard_at(&c,-2840,112,z0);
    if(s==2){q[2]=guard_at(&c,280,180,z0);q[2].v.z=-z0;q[3].v.z=-z0;q[3].v.x=-q[3].v.x;}
    guard_store(&c,g,q,4,2);}
   for(int k=0;k<4;k++) {
    const char *fn=k<2?floor_fn[row][k]:trunc_fn[row][k-2];
    divu_reset();
    const struct cost cs=invoke(fn,arg,0,0,0);
    if(row==0&&cs.result!=1)fail("Round timing batch",t);
    if(row==3&&cs.result!=4)fail("Round timing clip shape",t);
    cyc[row][k][t]=cs.cycles;
    if(k&1)divisions[row][k>>1]+=divu_divisions;
   }
  }
 }
 static const char *const names[7]={"batch","project","edgeSteps","clipNear","guardInside","guardSide","guardNearSide"};
 static const double per[7]={512,512,64,1,1,1,1};
 printf("\"timing\":{\"trials\":%u",ROUND_TIME_TRIALS);
 for(int row=0;row<7;row++) {
  for(int k=0;k<4;k++)round_median(cyc[row][k],ROUND_TIME_TRIALS);
  const unsigned mid=ROUND_TIME_TRIALS/2;
  if(divisions[row][0]!=divisions[row][1])fail("Round timing division count differs",(u32)row);
  printf(",\"%s\":{\"per\":%.0f,\"divisions\":%"PRIu64",\"median\":{\"floorPortable\":%.2f,\"floorDivu\":%.2f,\"truncPortable\":%.2f,\"truncDivu\":%.2f}}",
   names[row],per[row],divisions[row][1],cyc[row][0][mid]/per[row],cyc[row][1][mid]/per[row],cyc[row][2][mid]/per[row],cyc[row][3][mid]/per[row]);
 }
 printf("}}}\n");
 return 0;
}
