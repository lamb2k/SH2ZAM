/* SPDX-License-Identifier: GPL-2.0-or-later
 * Texture-merge planner section of the view capture (view-merge-1),
 * included by guard.h. Copyright (c) 2026 SH2ZAM Contributors.
 *
 * Exactness: random strips (perspective road rails from a synthetic camera,
 * and adversarial int16 / 2^32-row strips with random clip flags), random
 * masks, end masks, modes and tolerances; the planner runs on the SH-2 and
 * its runs are compared with an oracle written here from the contract text.
 * Rejected calls must write nothing.
 * Timing: 64-boundary road strips, runs of 1, 2 and 4, in pixel mode at
 * 0.5, 1 and 2 px, with tier-aligned end masks at 1 px, and in rows mode at
 * one row. */
#define MERGE_TRIALS 16384u
#define MERGE_TIME_TRIALS 64u
#define MERGE_CAP 200u
#define MERGE_TIME_COUNT 64u

struct merge_pt {int16_t sx,sy;uint8_t code;};
struct merge_strip {unsigned n;struct merge_pt l[MERGE_CAP],r[MERGE_CAP];uint32_t rows[MERGE_CAP],ends[MERGE_CAP];};

/* Boundary k of the run a .. b, s rows, t rows in; from the header text. */
static int merge_point_ok(const struct merge_pt *a,const struct merge_pt *k,const struct merge_pt *b,
 __int128 s,__int128 t,uint32_t tol,unsigned mode) {
 const __int128 dx=b->sx-a->sx,dy=b->sy-a->sy,ex=s*(k->sx-a->sx)-t*dx,ey=s*(k->sy-a->sy)-t*dy;
 const __int128 mx=ex<0?-ex:ex,my=ey<0?-ey:ey,m=mx>my?mx:my;
 if(mode==0)return m*256<=(__int128)tol*s;
 if(m>s)return 0;
 const __int128 dot=ex*dx+ey*dy;
 return (dot<0?-dot:dot)*256<=(__int128)tol*(dx*dx+dy*dy);
}
static int merge_oracle(uint8_t *out,const struct merge_strip *st,uint32_t mask,int ends,uint32_t tol,unsigned mode) {
 unsigned i=0;int n=0;
 while(i+1<st->n) {
  unsigned best=1;
  for(unsigned r=31;r>=2&&best==1;r--) {
   if(!((mask>>r)&1u)||i+r>st->n-1||(ends&&!((st->ends[i+r]>>r)&1u)))continue;
   int ok=1;
   for(unsigned j=i;j<=i+r;j++)if((st->l[j].code|st->r[j].code)&0xc3u)ok=0;
   const __int128 s=(__int128)st->rows[i+r]-st->rows[i];
   for(unsigned k=i+1;k<i+r&&ok;k++) {
    const __int128 t=(__int128)st->rows[k]-st->rows[i];
    ok=merge_point_ok(&st->l[i],&st->l[k],&st->l[i+r],s,t,tol,mode)&&merge_point_ok(&st->r[i],&st->r[k],&st->r[i+r],s,t,tol,mode);
   }
   if(ok)best=r;
  }
  out[n++]=(uint8_t)best;i+=best;
 }
 return n;
}
static void merge_rows(struct merge_strip *st,int huge) {
 const uint32_t base=8u<<(view_word()%5u);
 uint64_t r=view_word()%4u==0?(view_word()&0x7fffffffu):0;
 if(huge) {
  r=view_word()%64u;
  const uint64_t each=(UINT64_C(0xffffffff)-r)/(st->n-1);
  st->rows[0]=(uint32_t)r;
  for(unsigned k=1;k<st->n;k++){r+=each-(view_word()%2u?view_word()%(uint32_t)(each/8+1):0);st->rows[k]=(uint32_t)r;}
  return;
 }
 st->rows[0]=(uint32_t)r;
 for(unsigned k=1;k<st->n;k++) {
  uint32_t span=base;
  if((k==1||k==st->n-1)&&view_word()%10u<3u)span=1+view_word()%base;   /* partial end repetition */
  else if(view_word()%10u==0)span=1+view_word()%(2*base);
  r+=span;st->rows[k]=(uint32_t)r;
 }
}
/* Screen point of a view-space position for a 320x224 screen: a synthetic
 * camera, not the library's projection. */
static struct merge_pt merge_project(double x,double y,double z,double f) {
 struct merge_pt p={0,0,0};
 if(z<0.25){p.code=1;return p;}
 double sx=floor(160.0+f*x/z),sy=floor(112.0-f*y/z);
 sx=sx>32767?32767:sx<-32768?-32768:sx;sy=sy>32767?32767:sy<-32768?-32768:sy;
 p.sx=(int16_t)sx;p.sy=(int16_t)sy;
 p.code=(uint8_t)((z>4000.0?2:0)|(p.sx<0?4:0)|(p.sx>319?8:0)|(p.sy<0?16:0)|(p.sy>223?32:0));
 return p;
}
static void merge_road(struct merge_strip *st,unsigned n,int timing) {
 st->n=n;
 const double f=140.0+(view_word()%65536)/65536.0*180.0,u=1.0/65536.0;
 const double yaw=((view_word()%65536)*u-0.5)*(timing?0.4:1.0),pitch=-(view_word()%65536)*u*0.3;
 const double h=0.5+(view_word()%65536)*u*(timing?3.0:7.0);
 const double s0=timing?1.0+(view_word()%65536)*u*20.0:(view_word()%6u==0?-(view_word()%65536)*u*20.0:exp(log(0.3)+(view_word()%65536)*u*log(1000.0)));
 const double rep=exp(log(0.2)+(view_word()%65536)*u*log(60.0)),w=0.5+(view_word()%65536)*u*(timing?5.0:11.0);
 const double off=((view_word()%65536)*u-0.5)*(timing?6.0:20.0),kappa=((view_word()%65536)*u-0.5)*0.12;
 const double slope=((view_word()%65536)*u-0.5)*0.3,bank=view_word()%2u?((view_word()%65536)*u-0.5)*0.8:0.0;
 const double cy=cos(yaw),sy=sin(yaw),cp=cos(pitch),sp=sin(pitch);
 for(unsigned k=0;k<n;k++) {
  const double s=s0+rep*k,th=kappa*s,cx=off+0.5*kappa*s*s,cz=s,ccy=-h+slope*s;
  for(int side=0;side<2;side++) {
   const double sg=side?-1.0:1.0;
   const double p[3]={cx-sg*w*cos(bank)*cos(th),ccy+sg*w*sin(bank),cz+sg*w*cos(bank)*sin(th)};
   const double a0=cy*p[0]-sy*p[2],a2=sy*p[0]+cy*p[2],b1=cp*p[1]+sp*a2,b2=-sp*p[1]+cp*a2;
   (side?st->r:st->l)[k]=merge_project(a0,b1,b2,f);
  }
 }
 merge_rows(st,!timing&&view_word()%16u==0);
}
static int16_t merge_extreme(void) {
 switch(view_word()%4u) {
  case 0: return (int16_t)view_word();
  case 1: return view_word()&1u?INT16_MAX:INT16_MIN;
  case 2: return (int16_t)(view_word()&1u?INT16_MAX-view_range(0,2):INT16_MIN+view_range(0,2));
  default: return (int16_t)view_range(-2000,2000);
 }
}
static void merge_adversarial(struct merge_strip *st) {
 st->n=2+view_word()%(MERGE_CAP-1);
 merge_rows(st,view_word()%2u==0);
 const int linear=view_word()%2u;
 for(int side=0;side<2;side++) {
  struct merge_pt *p=side?st->r:st->l;
  const double ax=merge_extreme(),ay=merge_extreme(),bx=merge_extreme(),by=merge_extreme();
  const double total=(double)st->rows[st->n-1]-st->rows[0];
  for(unsigned k=0;k<st->n;k++) {
   if(linear) {   /* on the line from a to b in proportion to the row, then 0 or +-1 */
    const double fr=((double)st->rows[k]-st->rows[0])/total;
    double x=floor(ax+(bx-ax)*fr+0.5)+(view_word()%3u==0?view_range(-1,1):0),y=floor(ay+(by-ay)*fr+0.5)+(view_word()%3u==0?view_range(-1,1):0);
    p[k].sx=(int16_t)(x>32767?32767:x<-32768?-32768:x);p[k].sy=(int16_t)(y>32767?32767:y<-32768?-32768:y);
   } else {p[k].sx=merge_extreme();p[k].sy=merge_extreme();}
   const uint32_t c=view_word()%100u;
   p[k].code=(uint8_t)(c<3?view_word():c<40?(view_word()&0x3cu):0);
  }
 }
}
static uint32_t merge_mask(void) {
 switch(view_word()%8u) {
  case 0: case 1: case 2: return 0x16u;
  case 3: return 0x06u;
  case 4: return 0x116u;
  case 5: return 0xfffffffeu;
  default: return view_word();
 }
}
static int merge_ends(struct merge_strip *st) {
 const unsigned c=view_word()%8u;
 if(c<4)return 0;
 const unsigned phase=view_word()%8u;
 for(unsigned j=0;j<st->n;j++) {
  if(c<7){st->ends[j]=0;for(unsigned r=2;r<32;r++)if((j+phase)%r==0)st->ends[j]|=UINT32_C(1)<<r;}
  else st->ends[j]=view_word()%4u?view_word():0;
 }
 return 1;
}
static void merge_store(const struct merge_strip *st,uint32_t mask,int ends,uint32_t tol,unsigned mode) {
 const u32 a=sym("_view_input"),args=sym("_view_merge_args");
 for(unsigned k=0;k<st->n;k++)for(int side=0;side<2;side++) {
  const struct merge_pt *p=side?&st->r[k]:&st->l[k];
  const u32 v=a+(side?20*MERGE_CAP:0)+20*k;
  wl(&cpu,v,view_word());wl(&cpu,v+4,view_word());wl(&cpu,v+8,view_word());   /* x, y, z: never read */
  ww(&cpu,v+12,(u16)p->sx);ww(&cpu,v+14,(u16)p->sy);wl(&cpu,v+16,(u32)p->code<<24|(view_word()&0xffffffu));
 }
 for(unsigned k=0;k<st->n;k++){wl(&cpu,a+40*MERGE_CAP+4*k,st->rows[k]);wl(&cpu,a+44*MERGE_CAP+4*k,st->ends[k]);}
 wl(&cpu,args,st->n);wl(&cpu,args+4,mask);wl(&cpu,args+8,tol);wl(&cpu,args+12,mode);wl(&cpu,args+16,(u32)ends);
}
/* Runs on the target equal want[0..n-1]; the rest of the poisoned area is untouched. */
static int merge_check(int n,const uint8_t *want,unsigned area) {
 const u32 out=sym("_view_out_portable");
 for(unsigned k=0;k<area;k++)if(rb(&cpu,out+k)!=((int)k<n?want[k]:0xA5))return 0;
 return 1;
}
static void merge_poison(unsigned area) {const u32 out=sym("_view_out_portable");for(unsigned k=0;k<area;k++)wb(&cpu,out+k,0xA5);}
static int capture_merge(void) {
 if(rl(&cpu,sym("_view_merge_capacity"))!=MERGE_CAP)fail("Merge capacity",0);
 static struct merge_strip st;
 static const uint32_t tols[2][4]={{128,256,512,0},{256,512,64,0}};
 uint64_t runs=0,merged=0,bad=0,huge=0,blocked=0,boundaries=0,rejected=0,by_len[32]={0};
 view_state=UINT32_C(0x3e7a5eed);
 for(unsigned t=0;t<MERGE_TRIALS;t++) {
  if(t%2)merge_adversarial(&st);else merge_road(&st,2+view_word()%(MERGE_CAP-1),0);
  const uint32_t mask=merge_mask();const int ends=merge_ends(&st);const unsigned mode=view_word()%2u;
  const uint32_t tol=view_word()%5u?tols[mode][view_word()%4u]:(view_word()%3u?view_word()%4096u:0x01000000u);
  if((uint64_t)st.rows[st.n-1]-st.rows[0]>UINT64_C(0x80000000))huge++;
  for(unsigned j=0;j<st.n;j++)blocked+=((st.l[j].code|st.r[j].code)&0xc3u)!=0;
  uint8_t want[MERGE_CAP];
  const int w=merge_oracle(want,&st,mask,ends,tol,mode);
  merge_store(&st,mask,ends,tol,mode);merge_poison(st.n+4);
  const int n=(int32_t)invoke("_view_merge",0,0,0,0).result;
  if(n!=w||!merge_check(n,want,st.n+4))bad++;
  runs+=(uint64_t)w;boundaries+=st.n;
  for(int k=0;k<w;k++){merged+=want[k]>1;by_len[want[k]]++;}
  /* Every fourth strip again with one argument broken: nothing written. */
  if(t%4==0&&st.n>=3) {
   const u32 args=sym("_view_merge_args"),rows=sym("_view_input")+40*MERGE_CAP;
   const unsigned which=(t/4)%4;
   if(which==0)wl(&cpu,rows+4*(st.n-1),st.rows[st.n-2]);          /* rows not increasing */
   if(which==1)wl(&cpu,args+8,0x01000001u);                       /* tolerance too large */
   if(which==2)wl(&cpu,args+12,2);                                /* no such mode */
   if(which==3)wl(&cpu,args,1);                                   /* one boundary */
   merge_poison(st.n+4);
   if((int32_t)invoke("_view_merge",0,0,0,0).result!=-1||!merge_check(0,want,st.n+4))bad++;
   rejected++;
  }
 }
 if(bad)fail("Merge mismatch",(u32)bad);
 printf("\"merge\":{\"trials\":%u,\"boundaries\":%"PRIu64",\"runs\":%"PRIu64",\"mergedRuns\":%"PRIu64",\"mismatches\":%"PRIu64",\"rejectedCalls\":%"PRIu64","
  "\"hugeRowStrips\":%"PRIu64",\"blockedBoundaries\":%"PRIu64",\"runLengths\":[",MERGE_TRIALS,boundaries,runs,merged,bad,rejected,huge,blocked);
 for(int k=1;k<32;k++)printf("%s%"PRIu64,k>1?",":"",by_len[k]);
 printf("],");

 /* Timing: one road strip per trial, every configuration on the same strip. */
 enum {CONFIGS=5};
 static const char *const names[CONFIGS]={"pixel0.5","pixel1","pixel2","pixel1Aligned","rows1"};
 static const uint32_t ctol[CONFIGS]={128,256,512,256,256};
 static const unsigned cmode[CONFIGS]={0,0,0,0,1};
 static u32 cyc[CONFIGS][MERGE_TIME_TRIALS];
 uint64_t quads[CONFIGS]={0},instr[CONFIGS]={0},lens[CONFIGS][5]={{0}};
 view_state=UINT32_C(0x71e05eed);
 for(unsigned t=0;t<MERGE_TIME_TRIALS;t++) {
  merge_road(&st,MERGE_TIME_COUNT,1);
  for(unsigned j=0;j<st.n;j++){st.ends[j]=0;for(unsigned r=2;r<=4;r+=2)if(j%r==0)st.ends[j]|=UINT32_C(1)<<r;}
  for(int c=0;c<CONFIGS;c++) {
   const int ends=c==3;
   uint8_t want[MERGE_CAP];
   const int w=merge_oracle(want,&st,0x16u,ends,ctol[c],cmode[c]);
   merge_store(&st,0x16u,ends,ctol[c],cmode[c]);merge_poison(st.n+4);
   const struct cost cs=invoke("_view_merge",0,0,0,0);
   if((int32_t)cs.result!=w||!merge_check(w,want,st.n+4))fail("Merge timing result",t);
   cyc[c][t]=cs.cycles;quads[c]+=(uint64_t)w;instr[c]+=cs.instructions;
   for(int k=0;k<w;k++)lens[c][want[k]]++;
  }
 }
 printf("\"timing\":{\"trials\":%u,\"boundaries\":%u,\"runMask\":[1,2,4],\"configs\":{",MERGE_TIME_TRIALS,MERGE_TIME_COUNT);
 for(int c=0;c<CONFIGS;c++) {
  qsort(cyc[c],MERGE_TIME_TRIALS,4,view_cmp_u32);
  printf("%s\"%s\":{\"mode\":\"%s\",\"tolQ8\":%u,\"endMasks\":%s,\"meanQuads\":%.2f,\"runs\":[%"PRIu64",%"PRIu64",%"PRIu64"],"
   "\"medianCycles\":%u,\"medianCyclesPerBoundary\":%.2f,\"maxCyclesPerBoundary\":%.2f,\"instructionsPerBoundary\":%.2f}",
   c?",":"",names[c],cmode[c]?"rows":"pixels",ctol[c],c==3?"true":"false",(double)quads[c]/MERGE_TIME_TRIALS,lens[c][1],lens[c][2],lens[c][4],
   cyc[c][MERGE_TIME_TRIALS/2],cyc[c][MERGE_TIME_TRIALS/2]/(double)MERGE_TIME_COUNT,cyc[c][MERGE_TIME_TRIALS-1]/(double)MERGE_TIME_COUNT,
   (double)instr[c]/(MERGE_TIME_TRIALS*(double)MERGE_TIME_COUNT));
 }
 printf("}}}}\n");
 return 0;
}
