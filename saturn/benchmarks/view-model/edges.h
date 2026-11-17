/* SPDX-License-Identifier: GPL-2.0-or-later
 * Edge stepper and near-clip section of the view capture (view-edges-1),
 * included by capture.h. Copyright (c) 2026 SH2ZAM Contributors.
 *
 * Exactness: random view-space edges and quads, both entry points of each
 * function run on the SH-2 and compared with an oracle written here from the
 * contract text. Timing: 64-point edges in front of the camera, and quads
 * with two corners in front (two crossings each). */
#define EDGE_TRIALS 8192u
#define CLIP_TRIALS 65536u
#define EDGE_TIME_TRIALS 64u

static __int128 edge_trunc(__int128 n,__int128 d) {
 const __int128 a=n<0?-n:n,b=d<0?-d:d,q=a/b;
 return ((n<0)!=(d<0))?-q:q;
}
static int32_t edge_coord(void) {
 switch(view_word()%4u) {
  case 0: return (int32_t)view_word();
  case 1: return view_range(-100000,100000);
  case 2: return (view_word()&1u)?INT32_MAX-view_range(0,3):INT32_MIN+view_range(0,3);
  default: return view_range(-(1<<24),1<<24);
 }
}
static void edge_store_vertex(u32 a,const struct view_vtx *v) {
 wl(&cpu,a,(u32)v->x);wl(&cpu,a+4,(u32)v->y);wl(&cpu,a+8,(u32)v->z);
 ww(&cpu,a+12,(u16)v->sx);ww(&cpu,a+14,(u16)v->sy);wl(&cpu,a+16,(u32)v->code<<24);
}
static struct view_vtx edge_point(const struct view_cam *c,const int32_t a[3],const int32_t b[3],
 unsigned first,unsigned step,unsigned den,unsigned k) {
 __int128 p[3];
 for(int i=0;i<3;i++) {
  const __int128 delta=(__int128)b[i]-a[i];
  const __int128 f=edge_trunc(delta*first*65536,den),s=edge_trunc(delta*step*65536,den);
  p[i]=a[i]+edge_trunc(f+(__int128)k*s,65536);
 }
 return view_project(c,p,NULL);
}
static unsigned edge_clip_oracle(struct view_vtx out[6],const struct view_cam *c,const struct view_vtx in[4]) {
 unsigned n=0;
 for(unsigned i=0;i<4;i++) {
  const struct view_vtx *cur=&in[i],*next=&in[(i+1)&3u];
  const int ci=cur->z>=c->near_z,ni=next->z>=c->near_z;
  if(ci)out[n++]=*cur;
  if(ci!=ni) {
   const struct view_vtx *p=ci?cur:next,*q=ci?next:cur;
   const __int128 num=(__int128)p->z-c->near_z,den=(__int128)p->z-q->z;
   const __int128 v[3]={p->x+edge_trunc(((__int128)q->x-p->x)*num,den),p->y+edge_trunc(((__int128)q->y-p->y)*num,den),c->near_z};
   out[n++]=view_project(c,v,NULL);
  }
 }
 return n;
}
static void edge_set_args(unsigned first,unsigned step,unsigned den,unsigned count) {
 u32 a=sym("_view_edge_args");
 wl(&cpu,a,first);wl(&cpu,a+4,step);wl(&cpu,a+8,den);wl(&cpu,a+12,count);
}
static int capture_edges(void) {
 const u32 ends=sym("_view_edge_ends"),cin=sym("_view_clip_in");
 const u32 op=sym("_view_out_portable"),od=sym("_view_out_divu");
 static struct view_vtx want[64];
 uint64_t points=0,bad=0,quads=0,clip_bad=0;unsigned patterns[16]={0};
 view_state=UINT32_C(0xed6e5eed);
 for(unsigned t=0;t<EDGE_TRIALS;t++) {
  double m[3][3],eye[3];struct view_cam c=view_random_cam(m,eye,0);view_store_cam(&c);
  int32_t a[3],b[3];for(int i=0;i<3;i++){a[i]=edge_coord();b[i]=edge_coord();}
  const unsigned den=(unsigned)view_range(1,65535);
  const unsigned count=t%4u==0?1u:t%4u==1?64u:(unsigned)view_range(1,64);
  const unsigned first=t%8u==2?0u:(unsigned)view_range(0,(int32_t)den);
  const unsigned step=count==1?(unsigned)view_range(0,65535):t%16u==3?0u:(unsigned)view_range(0,(int32_t)((den-first)/(count-1)));
  struct view_vtx va={a[0],a[1],a[2],0,0,0},vb={b[0],b[1],b[2],0,0,0};
  edge_store_vertex(ends,&va);edge_store_vertex(ends+20,&vb);edge_set_args(first,step,den,count);
  for(unsigned k=0;k<count;k++)want[k]=edge_point(&c,a,b,first,step,den,k);
  view_poison(op,count);view_poison(od,count);divu_reset();
  if(invoke("_view_edges_portable",0,0,0,0).result!=1||invoke("_view_edges_divu",0,0,0,0).result!=1)fail("Edge steps call",t);
  bad+=view_compare(op,want,count)+view_compare(od,want,count);
  points+=count;
 }
 for(unsigned t=0;t<CLIP_TRIALS;t++) {
  double m[3][3],eye[3];struct view_cam c=view_random_cam(m,eye,0);view_store_cam(&c);
  const unsigned pattern=t%16u;
  struct view_vtx in[4],ref[6];
  for(unsigned i=0;i<4;i++) {
   const int inside=(pattern>>i)&1u;int32_t z;
   if(t%7u==0)z=inside?c.near_z:c.near_z-1;
   else if(inside)z=view_range(c.near_z,(t%5u==0)?INT32_MAX:(int32_t)((int64_t)c.near_z+(1<<22)>INT32_MAX?INT32_MAX:c.near_z+(1<<22)));
   else z=view_range((t%5u==0)?INT32_MIN:c.near_z-(1<<22),c.near_z-1);
   const __int128 v[3]={edge_coord(),edge_coord(),z};
   in[i]=view_project(&c,v,NULL);edge_store_vertex(cin+20*i,&in[i]);
  }
  const unsigned n=edge_clip_oracle(ref,&c,in);
  view_poison(op,6);view_poison(od,6);
  const u32 np=invoke("_view_clip_portable",0,0,0,0).result,nd=invoke("_view_clip_divu",0,0,0,0).result;
  if(np!=n||nd!=n)clip_bad++;
  else clip_bad+=view_compare(op,ref,n)+view_compare(od,ref,n);
  patterns[pattern]++;quads++;
 }
 /* sh2z_view_project[_divu]: random view-space records; the tail bytes hold
  * junk to show only x, y, z are read. */
 const u32 pts=sym("_view_input");
 uint64_t projected=0,project_bad=0;
 for(unsigned t=0;t<512u;t++) {
  double m[3][3],eye[3];struct view_cam c=view_random_cam(m,eye,0);view_store_cam(&c);
  for(unsigned k=0;k<512u;k++) {
   const int32_t z=k%3u==0?edge_coord():view_range(c.near_z-(1<<20)<INT32_MIN/2?INT32_MIN/2:c.near_z-(1<<20),INT32_MAX/2);
   const __int128 v[3]={edge_coord(),edge_coord(),z};
   struct view_vtx junk={(int32_t)v[0],(int32_t)v[1],(int32_t)v[2],(int16_t)view_word(),(int16_t)view_word(),(uint8_t)view_word()};
   edge_store_vertex(pts+20*k,&junk);
  }
  view_poison(op,512);view_poison(od,512);
  if(invoke("_view_project_portable",512,0,0,0).result!=1||invoke("_view_project_divu",512,0,0,0).result!=1)fail("Project call",t);
  for(unsigned k=0;k<512u;k++) {
   const __int128 v[3]={(int32_t)rl(&cpu,pts+20*k),(int32_t)rl(&cpu,pts+20*k+4),(int32_t)rl(&cpu,pts+20*k+8)};
   const struct view_vtx w=view_project(&c,v,NULL);
   project_bad+=view_compare(op+20*k,&w,1)+view_compare(od+20*k,&w,1);
  }
  projected+=512;
 }
 printf("\"edges\":{\"edgeTrials\":%u,\"points\":%"PRIu64",\"mismatches\":%"PRIu64",\"quads\":%"PRIu64",\"clipMismatches\":%"PRIu64",\"projectedPoints\":%"PRIu64",\"projectMismatches\":%"PRIu64",\"patterns\":[",
  EDGE_TRIALS,points,bad,quads,clip_bad,projected,project_bad);
 if(project_bad)fail("Project mismatch",(u32)project_bad);
 for(int i=0;i<16;i++)printf("%s%u",i?",":"",patterns[i]);
 printf("],");
 if(bad||clip_bad)fail("Edge or clip mismatch",(u32)(bad+clip_bad));
 /* Timing. */
 static u32 step_cyc[2][EDGE_TIME_TRIALS],clip_cyc[2][EDGE_TIME_TRIALS],one_cyc[2][EDGE_TIME_TRIALS],proj_cyc[2][EDGE_TIME_TRIALS];
 uint64_t step_div=0,clip_div=0,step_stall=0,proj_stall=0;
 view_state=UINT32_C(0x7e00e003);
 for(unsigned t=0;t<EDGE_TIME_TRIALS;t++) {
  double m[3][3],eye[3];struct view_cam c=view_random_cam(m,eye,1);view_store_cam(&c);
  const double half=160.0*65536.0/c.fx;
  struct view_vtx e[2];
  for(int j=0;j<2;j++) {
   const double z=(2.0+(view_word()%65536)/65536.0*3000.0)*16384.0;
   e[j].x=(int32_t)(((view_word()%65536)/32768.0-1.0)*half*z);e[j].y=(int32_t)(((view_word()%65536)/32768.0-1.0)*half*z*0.7);
   e[j].z=(int32_t)z;e[j].sx=e[j].sy=0;e[j].code=0;edge_store_vertex(ends+20*j,&e[j]);
  }
  edge_set_args(0,1,63,1);
  one_cyc[0][t]=invoke("_view_edges_portable",0,0,0,0).cycles;one_cyc[1][t]=invoke("_view_edges_divu",0,0,0,0).cycles;
  edge_set_args(0,1,63,64);
  struct cost a=invoke("_view_edges_portable",0,0,0,0);divu_reset();struct cost b=invoke("_view_edges_divu",0,0,0,0);
  step_div+=divu_divisions;step_stall+=divu_stall_cycles;step_cyc[0][t]=a.cycles;step_cyc[1][t]=b.cycles;
  /* Two corners in front, two behind. */
  struct view_vtx q[4];
  for(int i=0;i<4;i++) {
   const double z=i<2?(2.0+(view_word()%65536)/65536.0*40.0)*16384.0:-(1.0+(view_word()%65536)/65536.0*40.0)*16384.0;
   const double az=z<0?-z:z;
   const __int128 v[3]={(int32_t)(((view_word()%65536)/32768.0-1.0)*half*az),(int32_t)(((view_word()%65536)/32768.0-1.0)*half*az*0.7),(int32_t)z};
   q[i]=view_project(&c,v,NULL);edge_store_vertex(cin+20*i,&q[i]);
  }
  struct cost cp=invoke("_view_clip_portable",0,0,0,0);divu_reset();struct cost cd=invoke("_view_clip_divu",0,0,0,0);
  if(cp.result!=4||cd.result!=4)fail("Clip timing shape",t);
  clip_div+=divu_divisions;clip_cyc[0][t]=cp.cycles;clip_cyc[1][t]=cd.cycles;
  /* 512 view-space points inside the view volume. */
  for(unsigned k=0;k<512u;k++) {
   const double z=(2.0+(view_word()%65536)/65536.0*3000.0)*16384.0;
   struct view_vtx p={(int32_t)(((view_word()%65536)/32768.0-1.0)*half*z),(int32_t)(((view_word()%65536)/32768.0-1.0)*half*z*0.7),(int32_t)z,0,0,0};
   edge_store_vertex(pts+20*k,&p);
  }
  proj_cyc[0][t]=invoke("_view_project_portable",512,0,0,0).cycles;divu_reset();
  proj_cyc[1][t]=invoke("_view_project_divu",512,0,0,0).cycles;proj_stall+=divu_stall_cycles;
 }
 for(int k=0;k<2;k++){qsort(step_cyc[k],EDGE_TIME_TRIALS,4,view_cmp_u32);qsort(clip_cyc[k],EDGE_TIME_TRIALS,4,view_cmp_u32);
  qsort(one_cyc[k],EDGE_TIME_TRIALS,4,view_cmp_u32);qsort(proj_cyc[k],EDGE_TIME_TRIALS,4,view_cmp_u32);}
 const unsigned mid=EDGE_TIME_TRIALS/2;
 printf("\"timing\":{\"trials\":%u,\"medianCyclesPerProjectedPoint\":{\"portable\":%.2f,\"divuStallModel\":%.2f},\"projectStallCycles\":%"PRIu64",",
  EDGE_TIME_TRIALS,proj_cyc[0][mid]/512.0,proj_cyc[1][mid]/512.0,proj_stall);
 printf("\"pointsPerEdge\":64,\"edgeDivisions\":%"PRIu64",\"edgeStallCycles\":%"PRIu64","
  "\"medianCyclesPerPoint\":{\"portable\":%.2f,\"divuStallModel\":%.2f},\"medianCyclesOnePointCall\":{\"portable\":%u,\"divuStallModel\":%u},"
  "\"clipCrossingsPerQuad\":2,\"clipDivisions\":%"PRIu64",\"medianCyclesPerQuad\":{\"portable\":%u,\"divuStallModel\":%u}}}}\n",
  step_div,step_stall,step_cyc[0][mid]/64.0,step_cyc[1][mid]/64.0,one_cyc[0][mid],one_cyc[1][mid],
  clip_div,clip_cyc[0][mid],clip_cyc[1][mid]);
 return 0;
}
