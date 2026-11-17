/* SPDX-License-Identifier: GPL-2.0-or-later
 * Source-order guard clip section of the view capture (view-guard-src-1),
 * included by round.h before its capture function, which calls this one
 * last. Copyright (c) 2026 SH2ZAM Contributors.
 *
 * Exactness: random polygons (3..8 corners, random cameras, rectangles and
 * pass limits, thin rectangles for snap failures) and road quads on the
 * timing camera; both entry points run on the SH-2 and are compared with an
 * oracle written here from the contract text: previous -> current passes,
 * the Q0.32 crossing from the previous corner, the inward snap, the
 * view-round-1 projection. Failures must write nothing.
 * Timing: the view-round-1 guard fixtures (attrs 2) and a road quad passing
 * under the camera (attrs 0, pass limit 10), each through
 * sh2z_view_clip_guard_trunc and the source-order clip back to back. */
#define GSRC_TRIALS 32768u
#define GSRC_ROAD_TRIALS 16384u
#define GSRC_TIME_TRIALS 64u

/* ceil(n / d) for d > 0. */
static __int128 gsrc_ceil(__int128 n,__int128 d) {return -view_floor(-n,d);}
/* The snap: z into [near_z, far_z] (far once done), then the lattice
 * bounds of the side planes done, left, right, top, bottom; 0 on failure. */
static int gsrc_snap(struct guard_pt *p,unsigned plane,const struct view_cam *c,const int16_t g[4]) {
 if(p->c[2]<c->near_z)p->c[2]=c->near_z;
 if(plane>=1&&p->c[2]>c->far_z)p->c[2]=c->far_z;
 const __int128 z=p->c[2];
 const __int128 k[4]={(__int128)65536*(g[0]-c->cx),(__int128)65536*(g[2]-c->cx),(__int128)65536*(c->cy-g[1]),(__int128)65536*(c->cy-g[3])};
 const __int128 n[4]={gsrc_ceil(k[0]*z,c->fx),view_floor(k[1]*z,c->fx),view_floor(k[2]*z,c->fy),gsrc_ceil(k[3]*z,c->fy)};
 for(unsigned s=0;s<4;s++)if(plane>=2+s&&(n[s]<INT32_MIN||n[s]>INT32_MAX))return 0;
 if(plane>=2&&p->c[0]<n[0])p->c[0]=n[0];
 if(plane>=3&&p->c[0]>n[1])p->c[0]=n[1];
 if(plane>=4&&p->c[1]>n[2])p->c[1]=n[2];
 if(plane>=5&&p->c[1]<n[3])p->c[1]=n[3];
 for(unsigned q=0;q<=plane;q++)if(guard_plane_d(c,g,q,p)<0)return 0;
 return 1;
}
static int gsrc_oracle(struct guard_vtx *out,const struct view_cam *c,const int16_t g[4],const struct guard_vtx *in,unsigned n,
 unsigned attrs,unsigned max_out) {
 struct guard_pt poly[GUARD_MAX_OUT],next[GUARD_MAX_OUT];
 const unsigned coords=3+attrs;
 for(unsigned k=0;k<n;k++){memset(&poly[k],0,sizeof(poly[k]));poly[k].c[0]=in[k].v.x;poly[k].c[1]=in[k].v.y;poly[k].c[2]=in[k].v.z;
  for(unsigned a=0;a<attrs;a++)poly[k].c[3+a]=in[k].attr[a];}
 for(unsigned plane=0;plane<6&&n;plane++) {
  unsigned m=0,prev=n-1;
  for(unsigned i=0;i<n;i++) {
   const __int128 dp=guard_plane_d(c,g,plane,&poly[prev]),dc=guard_plane_d(c,g,plane,&poly[i]);
   if((dp>=0)!=(dc>=0)) {
    if(m==max_out)return -2;
    const __int128 a=dp<0?-dp:dp,b=dc<0?-dc:dc,t=(a<<32)/(a+b);
    struct guard_pt q;memset(&q,0,sizeof(q));
    for(unsigned j=0;j<coords;j++){const __int128 d=poly[i].c[j]-poly[prev].c[j],s=((d<0?-d:d)*t)>>32;q.c[j]=poly[prev].c[j]+(d<0?-s:s);}
    if(!gsrc_snap(&q,plane,c,g))return -3;
    next[m++]=q;
   }
   if(dc>=0){if(m==max_out)return -2;next[m++]=poly[i];}
   prev=i;
  }
  n=m;memcpy(poly,next,sizeof(poly[0])*m);
 }
 for(unsigned k=0;k<n;k++) {
  struct view_vtx q=round_project(c,poly[k].c);
  if(q.sx<g[0]||q.sx>g[2]||q.sy<g[1]||q.sy>g[3])fail("Source-order oracle left the rectangle",k);
  q.code=(uint8_t)((q.sx<c->clip[0]?4:0)|(q.sx>c->clip[2]?8:0)|(q.sy<c->clip[1]?16:0)|(q.sy>c->clip[3]?32:0));
  memset(&out[k],0,sizeof(out[k]));out[k].v=q;
  for(unsigned a=0;a<attrs;a++)out[k].attr[a]=(int32_t)poly[k].c[3+a];
 }
 return (int)n;
}
static void gsrc_store(const struct view_cam *c,const int16_t g[4],const struct guard_vtx *in,unsigned n,unsigned attrs,unsigned max_out) {
 guard_store(c,g,in,n,attrs);
 wl(&cpu,sym("_view_src_guard_args"),max_out);
}
/* A road quad on the ground under the timing camera, in view space: a level
 * camera h units up, the quad from z0 to z1 (z0 may be behind) and x0..x1. */
static void gsrc_road(struct guard_vtx q[4],double h,double x0,double x1,double z0,double z1,double pitch) {
 const double cp=cos(pitch),sp=sin(pitch);
 const double xz[4][2]={{x0,z0},{x1,z0},{x1,z1},{x0,z1}};
 for(int k=0;k<4;k++) {
  memset(&q[k],0,sizeof(q[k]));
  q[k].v.x=(int32_t)floor(xz[k][0]+0.5);q[k].v.y=(int32_t)floor(cp*-h+sp*xz[k][1]+0.5);q[k].v.z=(int32_t)floor(sp*h+cp*xz[k][1]+0.5);
 }
}
static void gsrc_random_road(struct guard_vtx q[4]) {
 const double u=16384.0,h=(5.0+(view_word()%65536)/65536.0*60.0)*u;
 const double x0=((view_word()%65536)/65536.0-0.5)*3000.0*u,w=(20.0+(view_word()%65536)/65536.0*600.0)*u;
 const double z0=(-200.0+(view_word()%65536)/65536.0*3000.0)*u,len=(20.0+(view_word()%65536)/65536.0*1500.0)*u;
 gsrc_road(q,h,x0,x0+w,z0,z0+len,(view_word()%65536)/65536.0*0.35);
}
static int capture_guard_src(void) {
 const u32 op=sym("_view_out_portable"),od=sym("_view_out_divu");
 uint64_t vertices=0,bad=0,overflows=0,snaps=0,empty=0,road=0,road_vertices=0,sizes[GUARD_MAX_OUT+1]={0};
 view_state=UINT32_C(0x5eed0c1f);
 for(unsigned t=0;t<GSRC_TRIALS+GSRC_ROAD_TRIALS;t++) {
  const int is_road=t>=GSRC_TRIALS;
  double m[3][3],eye[3];struct view_cam c=view_random_cam(m,eye,is_road);
  int16_t g[4];
  struct guard_vtx in[8],want[GUARD_MAX_OUT];
  unsigned n,attrs,max_out;
  if(is_road) {
   g[0]=g[1]=-900;g[2]=g[3]=900;n=4;attrs=view_word()%3u;max_out=10;
   gsrc_random_road(in);for(unsigned k=0;k<4;k++)for(int i=0;i<4;i++)in[k].attr[i]=(int32_t)view_word();
  } else {
   if(view_word()%4u==0) {
    /* thin rectangles near the centre and a shallow near plane: snap failures */
    g[0]=(int16_t)view_range(c.cx-3,c.cx+3);g[2]=(int16_t)(g[0]+view_range(0,2));
    g[1]=(int16_t)view_range(c.cy-3,c.cy+3);g[3]=(int16_t)(g[1]+view_range(0,2));
    c.near_z=view_range(1,64);
   } else guard_random_rect(&c,g);
   n=3+view_word()%6u;attrs=view_word()%5u;
   max_out=view_word()%4u==0?(unsigned)view_range(3,16):view_word()%2u?10u:16u;
   for(unsigned k=0;k<n;k++) {
    if(view_word()%4u==0){in[k].v.x=edge_coord();in[k].v.y=edge_coord();in[k].v.z=edge_coord();for(int i=0;i<4;i++)in[k].attr[i]=(int32_t)view_word();}
    else in[k]=guard_at(&c,view_range(g[0]-3000,g[2]+3000),view_range(g[1]-3000,g[3]+3000),guard_depth(&c));
   }
  }
  gsrc_store(&c,g,in,n,attrs,max_out);
  for(unsigned i=0;i<9*GUARD_MAX_OUT;i++){wl(&cpu,op+4*i,UINT32_C(0xA5A5A5A5));wl(&cpu,od+4*i,UINT32_C(0xA5A5A5A5));}
  const int w=gsrc_oracle(want,&c,g,in,n,attrs,max_out);
  const int rp=(int32_t)invoke("_view_src_guard_portable",0,0,0,0).result,rd=(int32_t)invoke("_view_src_guard_divu",0,0,0,0).result;
  if(rp!=w||rd!=w)fail("Source-order guard count",t);
  if(w<=0) {
   overflows+=w==-2;snaps+=w==-3;empty+=w==0;
   for(unsigned i=0;i<9*GUARD_MAX_OUT;i++)if(rl(&cpu,op+4*i)!=UINT32_C(0xA5A5A5A5)||rl(&cpu,od+4*i)!=UINT32_C(0xA5A5A5A5))fail("Source-order guard wrote on failure",t);
   continue;
  }
  sizes[w]++;
  bad+=guard_compare(op,want,w)+guard_compare(od,want,w);
  vertices+=(uint64_t)w;
  if(is_road){road++;road_vertices+=(uint64_t)w;}
 }
 printf("\"guardSrc\":{\"trials\":%u,\"roadTrials\":%u,\"vertices\":%"PRIu64",\"roadPolygons\":%"PRIu64",\"roadVertices\":%"PRIu64","
  "\"mismatches\":%"PRIu64",\"overflows\":%"PRIu64",\"snapFailures\":%"PRIu64",\"empty\":%"PRIu64",\"sizes\":[",
  GSRC_TRIALS,GSRC_ROAD_TRIALS,vertices,road,road_vertices,bad,overflows,snaps,empty);
 for(int k=0;k<=GUARD_MAX_OUT;k++)printf("%s%"PRIu64,k?",":"",sizes[k]);
 printf("],");
 if(bad)fail("Source-order guard mismatch",(u32)bad);

 /* Timing on the timing camera and the -900..900 band. Rows 0-2 are the
  * view-round-1 guard fixtures (attrs 2, pass limit 16); row 3 is a road
  * quad from behind the camera to 300 units ahead, 600 units wide (attrs 0,
  * pass limit 10). cyc[row][0 trunc portable, 1 trunc divu, 2 src portable,
  * 3 src divu][trial]. */
 static u32 cyc[4][4][GSRC_TIME_TRIALS];
 static const char *const fn[4]={"_view_round_guard_portable","_view_round_guard_divu","_view_src_guard_portable","_view_src_guard_divu"};
 static const char *const names[4]={"inside","side","nearSide","roadNear"};
 uint64_t divisions[4][2]={{0}},outs[4]={0};
 view_state=UINT32_C(0x5eed7140);
 for(unsigned t=0;t<GSRC_TIME_TRIALS;t++) {
  double m[3][3],eye[3];const struct view_cam c=view_random_cam(m,eye,1);
  const int16_t g[4]={-900,-900,900,900};
  for(int row=0;row<4;row++) {
   struct guard_vtx q[4];unsigned attrs=2,max_out=16;
   if(row<3) {
    const int32_t z0=(int32_t)((20.0+(view_word()%65536)/65536.0*200.0)*16384.0);
    q[0]=guard_at(&c,40,40,z0);q[1]=guard_at(&c,280,40,z0);q[2]=guard_at(&c,280,180,z0);q[3]=guard_at(&c,40,180,z0);
    if(row>=1)q[0]=guard_at(&c,-2840,112,z0);
    if(row==2){q[2]=guard_at(&c,280,180,z0);q[2].v.z=-z0;q[3].v.z=-z0;q[3].v.x=-q[3].v.x;}
   } else {
    const double u=16384.0,x0=((view_word()%65536)/65536.0-0.5)*200.0*u;
    gsrc_road(q,(10.0+(view_word()%65536)/65536.0*20.0)*u,x0-300.0*u,x0+300.0*u,-40.0*u,300.0*u,0.05);
    attrs=0;max_out=10;
   }
   gsrc_store(&c,g,q,4,attrs,max_out);
   struct guard_vtx want[GUARD_MAX_OUT];
   const int w=gsrc_oracle(want,&c,g,q,4,attrs,max_out);
   if(w<3)fail("Source-order timing shape",(u32)row);
   outs[row]+=(uint64_t)w;
   for(int k=0;k<4;k++) {
    divu_reset();
    const struct cost cs=invoke(fn[k],0,0,0,0);
    cyc[row][k][t]=cs.cycles;
    if(k==1)divisions[row][0]+=divu_divisions;
    if(k==3)divisions[row][1]+=divu_divisions;
    if(k>=2&&((int32_t)cs.result!=w||guard_compare(k==2?op:od,want,w)))fail("Source-order timing result",t);
   }
  }
 }
 printf("\"timing\":{\"trials\":%u,\"band\":[-900,-900,900,900]",GSRC_TIME_TRIALS);
 for(int row=0;row<4;row++) {
  for(int k=0;k<4;k++)qsort(cyc[row][k],GSRC_TIME_TRIALS,4,view_cmp_u32);
  const unsigned mid=GSRC_TIME_TRIALS/2;
  printf(",\"%s\":{\"attrs\":%u,\"maxOut\":%u,\"meanOutputVertices\":%.2f,\"divisionsPerCall\":{\"trunc\":%.2f,\"src\":%.2f},"
   "\"median\":{\"truncPortable\":%u,\"truncDivu\":%u,\"srcPortable\":%u,\"srcDivu\":%u},\"max\":{\"truncDivu\":%u,\"srcDivu\":%u}}",
   names[row],row<3?2u:0u,row<3?16u:10u,(double)outs[row]/GSRC_TIME_TRIALS,(double)divisions[row][0]/GSRC_TIME_TRIALS,
   (double)divisions[row][1]/GSRC_TIME_TRIALS,cyc[row][0][mid],cyc[row][1][mid],cyc[row][2][mid],cyc[row][3][mid],
   cyc[row][1][GSRC_TIME_TRIALS-1],cyc[row][3][GSRC_TIME_TRIALS-1]);
 }
 printf("}}}\n");
 return 0;
}
