/* SPDX-License-Identifier: GPL-2.0-or-later
 * Executed-instruction checks and cycle model for the view-merge-2 tier
 * texture merge planner (sh2z_view_merge_tier in view_experimental.h),
 * against view-merge-1 (sh2z_view_merge_runs in pixel mode), the planner a
 * road renderer calls today. Runs the actual big-endian SH-2 object code in
 * Yabause's unmodified interpreter, like harness.c, whose loader and call
 * wrapper it reuses. Flat RAM, no caches or wait states: MODELLED cycles, not
 * Saturn timing. Copyright (c) 2026 SH2ZAM Contributors. This optional model
 * harness is licensed separately from SH2ZAM's MIT core; see LICENSE.
 *
 * usage: merge_tier_harness model.elf verify-strips > results.json
 * The ELF must contain bridge.o from benchmarks/merge-model and the library.
 * Every plan is checked against an oracle written here from the header text
 * (and, where the contract says they agree, against view-merge-1 on the
 * target) before any timing is printed; any mismatch exits non-zero. */
#define main sh2_model_main
#include "harness.c"
#undef main

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
static struct cost call(const char *name){char tmp[140];snprintf(tmp,sizeof tmp,"_%s",name);return invoke(tmp,0,0,0,0);}
static u32 symc(const char *name){char tmp[140];snprintf(tmp,sizeof tmp,"_%s",name);return sym(tmp);}
static void bad(const char *what,uint64_t it){fprintf(stderr,"merge model mismatch: %s (case %"PRIu64")\n",what,it);exit(3);}

static uint32_t rng=UINT32_C(0x7e1e5eed);
static uint32_t rnd(void){rng^=rng<<13;rng^=rng>>17;rng^=rng<<5;return rng;}
static double unit01(void){return (double)rnd()/4294967296.0;}
static double frange(double lo,double hi){return lo+(hi-lo)*unit01();}
static int32_t range(int32_t lo,int32_t hi){return (int32_t)((int64_t)lo+(int64_t)(rnd()%(uint64_t)((int64_t)hi-lo+1)));}

enum {CAP=200};
struct pt {int16_t sx,sy;uint8_t code;};
struct strip {unsigned n;struct pt l[CAP],r[CAP];uint32_t rows[CAP],ends[CAP];};

/* ---- oracle, from the header text (view-merge-2) ---- */
static int point_ok(const struct pt *p,unsigned r,unsigned k,uint32_t tol){
 const __int128 fx=(__int128)r*(p[k].sx-p[0].sx)-(__int128)k*(p[r].sx-p[0].sx);
 const __int128 fy=(__int128)r*(p[k].sy-p[0].sy)-(__int128)k*(p[r].sy-p[0].sy);
 const __int128 ax=fx<0?-fx:fx,ay=fy<0?-fy:fy,m=ax>ay?ax:ay;
 return m*256<=(__int128)tol*r;
}
static int plan_oracle(uint8_t *out,const struct strip *st,uint32_t mask,int ends,uint32_t tol){
 static const unsigned lengths[4]={16,8,4,2};
 unsigned i=0;int n=0;
 while(i+1<st->n){
  unsigned best=1;
  for(unsigned c=0;c<4&&best==1;c++){
   const unsigned r=lengths[c];
   if(!((mask>>r)&1u)||i+r>st->n-1||(ends&&!((st->ends[i+r]>>r)&1u)))continue;
   int ok=1;
   for(unsigned j=i;j<=i+r&&ok;j++)if((st->l[j].code|st->r[j].code)&0xc3u)ok=0;
   for(unsigned k=1;k<r&&ok;k++)ok=point_ok(st->l+i,r,k,tol)&&point_ok(st->r+i,r,k,tol);
   if(ok)best=r;
  }
  out[n++]=(uint8_t)best;i+=best;
 }
 return n;
}

/* ---- strips ---- */
/* A road renderer's rows and end masks: boundaries on every whole period
 * between texture positions low and high (partial first and last
 * repetitions); a run of r <= factor may end at j when it starts on a period
 * boundary and covers r whole periods. at[j] is boundary j in periods. */
static void tier_rows(struct strip *st,unsigned n,uint32_t unit,unsigned factor,int whole,double *at){
 const uint32_t phase=whole?0u:rnd()%3u?rnd()%unit:0u,tail=whole?unit:rnd()%3u?1u+rnd()%unit:unit;
 st->n=n;st->rows[0]=0;
 for(unsigned j=1;j<n;j++)st->rows[j]=j+1<n?j*unit-phase:(n-2)*unit-phase+tail;
 if(n==2)st->rows[1]=tail;
 for(unsigned j=0;j<n;j++){
  at[j]=((double)st->rows[j]+phase)/unit;st->ends[j]=0;
  for(unsigned r=2;r<=16;r*=2)if(r<=factor&&j>=r&&(st->rows[j-r]+phase)%unit==0&&st->rows[j]-st->rows[j-r]==unit*r)st->ends[j]|=UINT32_C(1)<<r;
 }
}
/* Screen point for a 320x224 screen from a synthetic camera (not the
 * library's projection); corners outside a +-900 band get 0x80, which a
 * road renderer sets on corners it will clip instead. */
static struct pt project(double x,double y,double z,double f){
 struct pt p={0,0,0};
 if(z<0.25){p.code=1;return p;}
 double sx=floor(160.0+f*x/z),sy=floor(112.0-f*y/z);
 sx=sx>32767?32767:sx<-32768?-32768:sx;sy=sy>32767?32767:sy<-32768?-32768:sy;
 p.sx=(int16_t)sx;p.sy=(int16_t)sy;
 p.code=(uint8_t)((z>4000.0?2:0)|(p.sx<0?4:0)|(p.sx>319?8:0)|(p.sy<0?16:0)|(p.sy>223?32:0));
 if(p.sx<-900||p.sx>900||p.sy<-900||p.sy>900)p.code|=0x80;
 return p;
}
/* The view-merge-1 capture's road: curvature, hills, banking; boundaries at
 * at[k] repetitions along it. */
static void road(struct strip *st,const double *at,int timing){
 const double f=140.0+unit01()*180.0;
 const double yaw=(unit01()-0.5)*(timing?0.4:1.0),pitch=-unit01()*0.3,h=0.5+unit01()*(timing?3.0:7.0);
 const double s0=timing?1.0+unit01()*20.0:(rnd()%6u==0?-unit01()*20.0:exp(log(0.3)+unit01()*log(1000.0)));
 const double rep=exp(log(0.2)+unit01()*log(60.0)),w=0.5+unit01()*(timing?5.0:11.0);
 const double off=(unit01()-0.5)*(timing?6.0:20.0),kappa=(unit01()-0.5)*0.12;
 const double slope=(unit01()-0.5)*0.3,bank=rnd()%2u?(unit01()-0.5)*0.8:0.0;
 const double cy=cos(yaw),sy=sin(yaw),cp=cos(pitch),sp=sin(pitch);
 for(unsigned k=0;k<st->n;k++){
  const double s=s0+rep*at[k],th=kappa*s,cx=off+0.5*kappa*s*s,cz=s,ccy=-h+slope*s;
  for(int side=0;side<2;side++){
   const double sg=side?-1.0:1.0;
   const double p[3]={cx-sg*w*cos(bank)*cos(th),ccy+sg*w*sin(bank),cz+sg*w*cos(bank)*sin(th)};
   const double a0=cy*p[0]-sy*p[2],a2=sy*p[0]+cy*p[2],b1=cp*p[1]+sp*a2,b2=-sp*p[1]+cp*a2;
   (side?st->r:st->l)[k]=project(a0,b1,b2,f);
  }
 }
}
static int16_t extreme(void){
 switch(rnd()%4u){
  case 0: return (int16_t)rnd();
  case 1: return rnd()&1u?INT16_MAX:INT16_MIN;
  case 2: return (int16_t)(rnd()&1u?INT16_MAX-range(0,2):INT16_MIN+range(0,2));
  default: return (int16_t)range(-2000,2000);
 }
}
/* int16 extremes or rails linear in the boundary index with +-1 noise and a
 * bow; equal row steps; random flags and end masks. */
static void adversarial(struct strip *st){
 st->n=2+rnd()%(CAP-1);
 const uint32_t step=1+rnd()%4096u;
 for(unsigned k=0;k<st->n;k++){st->rows[k]=step*k;st->ends[k]=rnd()%4u?rnd():0;}
 const int linear=rnd()%2u;
 for(int side=0;side<2;side++){
  struct pt *p=side?st->r:st->l;
  const double ax=extreme(),ay=extreme(),bx=extreme(),by=extreme(),bow=frange(-3.0,3.0);
  for(unsigned k=0;k<st->n;k++){
   if(linear){
    const double fr=(double)k/(st->n-1);
    double x=floor(ax+(bx-ax)*fr+bow*fr*(1-fr)*4+0.5)+(rnd()%3u==0?range(-1,1):0),y=floor(ay+(by-ay)*fr+0.5)+(rnd()%3u==0?range(-1,1):0);
    p[k].sx=(int16_t)(x>32767?32767:x<-32768?-32768:x);p[k].sy=(int16_t)(y>32767?32767:y<-32768?-32768:y);
   } else {p[k].sx=extreme();p[k].sy=extreme();}
   const uint32_t c=rnd()%100u;
   p[k].code=(uint8_t)(c<3?rnd():c<40?(rnd()&0x3cu):0);
  }
 }
}
static uint32_t factor_mask(unsigned factor){return factor>=16?0x10116u:factor==8?0x116u:factor==4?0x16u:factor==2?0x6u:0x2u;}

/* ---- target memory ---- */
static void store(const struct strip *st,uint32_t mask,int ends,uint32_t tol){
 const u32 l=symc("merge_left"),r=symc("merge_right"),rows=symc("merge_rows"),e=symc("merge_ends"),a=symc("merge_args");
 for(unsigned k=0;k<st->n;k++)for(int side=0;side<2;side++){
  const struct pt *p=side?&st->r[k]:&st->l[k];
  const u32 v=(side?r:l)+20*k;
  wl(&cpu,v,rnd());wl(&cpu,v+4,rnd());wl(&cpu,v+8,rnd());   /* x, y, z: never read */
  ww(&cpu,v+12,(u16)p->sx);ww(&cpu,v+14,(u16)p->sy);wl(&cpu,v+16,(u32)p->code<<24|(rnd()&0xffffffu));
 }
 for(unsigned k=0;k<st->n;k++){wl(&cpu,rows+4*k,st->rows[k]);wl(&cpu,e+4*k,st->ends[k]);}
 wl(&cpu,a,st->n);wl(&cpu,a+4,mask);wl(&cpu,a+8,tol);wl(&cpu,a+12,(u32)ends);
}
static void poison(unsigned area){const u32 o=symc("merge_runs");for(unsigned k=0;k<area;k++)wb(&cpu,o+k,0xA5);}
static int same(int n,const uint8_t *want,unsigned area){
 const u32 o=symc("merge_runs");
 for(unsigned k=0;k<area;k++)if(rb(&cpu,o+k)!=((int)k<n?want[k]:0xA5))return 0;
 return 1;
}

/* ---- verification ---- */
static uint64_t v_strips,v_boundaries,v_runs,v_merged,v_len[17],v_equal,v_rejected;
static void verify(uint64_t strips){
 static struct strip st;double at[CAP];
 for(uint64_t t=0;t<strips;t++){
  uint32_t mask;int ends;
  const int is_road=t%3u!=2u;
  if(is_road){
   static const uint32_t units[3]={256,512,1024};
   const unsigned factor=rnd()%8u==0?16:rnd()%8u==1?8:(unsigned[]){4,4,2,1}[rnd()%4u];
   tier_rows(&st,rnd()%2u?64:2+rnd()%(CAP-1),units[rnd()%3u],factor,rnd()%8u==0,at);
   road(&st,at,0);mask=factor_mask(factor);ends=rnd()%8u!=0;
  } else {
   adversarial(&st);
   switch(rnd()%6u){case 0:case 1:mask=0x16u;break;case 2:mask=0x6u;break;case 3:mask=0x10117u;break;case 4:mask=0x2u;break;default:mask=rnd()&0x10117u;}
   ends=rnd()%2u;
  }
  static const uint32_t tols[8]={256,256,128,512,0,3,1024,255};
  const uint32_t tol=rnd()%6u?tols[rnd()%8u]:rnd()%5u?rnd()%4096u:0x01000000u;
  uint8_t want[CAP];
  const int w=plan_oracle(want,&st,mask,ends,tol);
  store(&st,mask,ends,tol);poison(st.n+8);
  if((int32_t)call("tier_plan").result!=w||!same(w,want,st.n+8))bad("tier_plan against the oracle",t);
  /* Equal-height repetitions in every permitted run: view-merge-1 agrees. */
  if(!is_road||ends){
   poison(st.n+8);
   if((int32_t)call("runs_plan").result!=w||!same(w,want,st.n+8))bad("runs_plan against tier_plan",t);
   v_equal++;
  }
  v_strips++;v_boundaries+=st.n;v_runs+=(uint64_t)w;
  for(int k=0;k<w;k++){v_merged+=want[k]>1;v_len[want[k]]++;}
  /* Every fourth strip again with one argument broken: -1, nothing written. */
  if(t%4u==0&&st.n>=3){
   const u32 a=symc("merge_args");const unsigned which=(unsigned)(t/4u)%4u;
   if(which==0)wl(&cpu,a,1);
   if(which==1)wl(&cpu,a+8,0x01000001u);
   if(which==2)wl(&cpu,a+4,mask|0x8u);
   if(which==3)wl(&cpu,a+4,mask|0x80000000u);
   poison(st.n+8);
   if((int32_t)call("tier_plan").result!=-1||!same(0,want,st.n+8))bad("rejected call",t);
   v_rejected++;
  }
 }
}

/* ---- texel drift of a plan, host side ---- */
enum {BINS=16*1024};
struct drift {uint64_t quads,samples,hist[BINS+1];double max;};
static double cheb(double ax,double ay,double bx,double by){const double dx=fabs(ax-bx),dy=fabs(ay-by);return dx>dy?dx:dy;}
static void drift_plan(struct drift *d,const struct strip *st,const uint8_t *runs,int n){
 enum {Q=12,U=8};
 unsigned i=0;
 for(int k=0;k<n;k++){
  const unsigned r=runs[k];
  if(r>1){
   d->quads++;
   const struct pt *L=st->l+i,*R=st->r+i;
   for(unsigned j=0;j<r;j++)for(unsigned q=(j?1u:0u);q<=Q;q++){
    const double f=(double)q/Q,v=(j+f)/r;
    const double lmx=L[0].sx+(L[r].sx-L[0].sx)*v,lmy=L[0].sy+(L[r].sy-L[0].sy)*v,rmx=R[0].sx+(R[r].sx-R[0].sx)*v,rmy=R[0].sy+(R[r].sy-R[0].sy)*v;
    const double lrx=L[j].sx+(L[j+1].sx-L[j].sx)*f,lry=L[j].sy+(L[j+1].sy-L[j].sy)*f,rrx=R[j].sx+(R[j+1].sx-R[j].sx)*f,rry=R[j].sy+(R[j+1].sy-R[j].sy)*f;
    for(unsigned uq=0;uq<=U;uq++){
     const double u=(double)uq/U;
     const double e=cheb(lmx+(rmx-lmx)*u,lmy+(rmy-lmy)*u,lrx+(rrx-lrx)*u,lry+(rry-lry)*u);
     if(e>d->max)d->max=e;
     const double b=e*1024.0;d->hist[b>=BINS?BINS:(unsigned)b]++;d->samples++;
    }
   }
  }
  i+=r;
 }
}
static double quantile(const struct drift *d,double q){
 if(!d->samples)return 0.0;   /* no merged quad: no drift */
 const uint64_t want=(uint64_t)ceil(q*(double)d->samples);uint64_t seen=0;
 for(unsigned b=0;b<=BINS;b++){seen+=d->hist[b];if(seen>=want)return (b+1u)/1024.0;}
 return 16.0;
}

/* ---- cost ---- */
/* Placement model (MODEL_PLACEMENT_LOG): the runs are output; the rails, rows,
 * end masks and arguments are input. */
static const struct place_decl place_merge[]={{"_merge_runs",PC_OUTPUT},{NULL,0}};
enum {TRIALS=64};
static int cmp_u(const void*a,const void*b){unsigned x=*(const unsigned*)a,y=*(const unsigned*)b;return x<y?-1:x>y;}
struct config {const char *name,*description;unsigned n;uint32_t mask,tol;int ends,whole;};
static void cost_config(const struct config *c,int first){
 static struct strip st;double at[CAP];
 static unsigned cyc[2][TRIALS];uint64_t ins[2]={0,0},quads=0,lens[17]={0};
 static struct drift d;memset(&d,0,sizeof d);
 rng=UINT32_C(0x71e05eed)+c->n*31u+c->tol;
 for(unsigned t=0;t<TRIALS;t++){
  static const uint32_t units[3]={256,512,1024};
  tier_rows(&st,c->n,units[t%3u],4,c->whole,at);road(&st,at,1);
  uint8_t want[CAP];
  const int w=plan_oracle(want,&st,c->mask,c->ends,c->tol);
  store(&st,c->mask,c->ends,c->tol);
  static const char *const entry[2]={"tier_plan","runs_plan"};
  for(int e=0;e<2;e++){
   poison(st.n+8);
   {char label[80];snprintf(label,sizeof label,"merge.%s.%s",c->name,entry[e]);place_tag(label,place_merge);}
   const struct cost r=call(entry[e]);
   if((int32_t)r.result!=w||!same(w,want,st.n+8))bad(entry[e],t);
   cyc[e][t]=r.cycles;ins[e]+=r.instructions;
  }
  quads+=(uint64_t)w;for(int k=0;k<w;k++)lens[want[k]]++;
  drift_plan(&d,&st,want,w);
 }
 printf("%s\"%s\":{\"description\":\"%s\",\"boundaries\":%u,\"trials\":%u,\"runMask\":%u,\"tolQ8\":%u,\"endMasks\":%s,"
  "\"meanQuads\":%.3f,\"runLengths\":{\"1\":%"PRIu64",\"2\":%"PRIu64",\"4\":%"PRIu64"},",
  first?"":",",c->name,c->description,c->n,TRIALS,c->mask,c->tol,c->ends?"true":"false",(double)quads/TRIALS,lens[1],lens[2],lens[4]);
 double med[2];
 static const char *const key[2]={"tier","viewMerge1"};
 for(int e=0;e<2;e++){
  double sum=0;for(unsigned t=0;t<TRIALS;t++)sum+=cyc[e][t];
  qsort(cyc[e],TRIALS,sizeof cyc[e][0],cmp_u);
  med[e]=(cyc[e][TRIALS/2-1]+cyc[e][TRIALS/2])/2.0;
  printf("\"%s\":{\"medianCyclesPerCall\":%.1f,\"medianCyclesPerBoundary\":%.2f,\"meanCyclesPerBoundary\":%.2f,\"maxCyclesPerBoundary\":%.2f,\"instructionsPerBoundary\":%.2f},",
   key[e],med[e],med[e]/c->n,sum/TRIALS/c->n,cyc[e][TRIALS-1]/(double)c->n,(double)ins[e]/TRIALS/c->n);
 }
 printf("\"speedup\":%.2f,\"drift\":{\"mergedQuads\":%"PRIu64",\"texelSamples\":%"PRIu64",\"p50Px\":%.4f,\"p99Px\":%.4f,\"maxPx\":%.4f,\"boundPx\":%.4f}}",
  med[1]/med[0],d.quads,d.samples,quantile(&d,0.5),quantile(&d,0.99),d.max,c->tol/256.0);
 if(d.max>c->tol/256.0+1e-12)bad("drift above the tolerance",0);
}

int main(int argc,char**argv){
 if(argc<3){fprintf(stderr,"usage: merge_tier_harness model.elf verify-strips\n");return 2;}
 const uint64_t strips=strtoull(argv[2],0,10);
 load_elf_symbols(argv[1]);load_elf(argv[1]);place_init(argv[1]);
 cpu.model=SHMT_SH2;cpu.MappedMemoryReadByte=rb;cpu.MappedMemoryReadWord=rw;cpu.MappedMemoryReadLong=rl;cpu.MappedMemoryWriteByte=wb;cpu.MappedMemoryWriteWord=ww;cpu.MappedMemoryWriteLong=wl;
 for(unsigned i=0;i<65536;i++)cpu.opcodes[i]=decode(SHMT_SH2,(u16)i);for(unsigned i=0;i<256;i++)cpu.fetchlist[i]=fetch;
 if(rl(&cpu,symc("merge_capacity"))!=CAP)fail("Merge capacity",0);
 verify(strips);
 printf("{\"schema\":1,\"kind\":\"Yabause isolated SH2 interpreter flat-RAM model, not Saturn timing\",");
 printf("\"verified\":{\"strips\":%"PRIu64",\"boundaries\":%"PRIu64",\"runs\":%"PRIu64",\"mergedRuns\":%"PRIu64",\"runLengths\":{\"1\":%"PRIu64",\"2\":%"PRIu64",\"4\":%"PRIu64",\"8\":%"PRIu64",\"16\":%"PRIu64"},"
  "\"plansEqualToViewMerge1\":%"PRIu64",\"rejectedCalls\":%"PRIu64",\"entries\":\"tier_plan against the oracle in this harness; runs_plan (view-merge-1, pixel mode) against it where every permitted run has equal-height repetitions; rejected calls return -1 and write nothing\"},",
  v_strips,v_boundaries,v_runs,v_merged,v_len[1],v_len[2],v_len[4],v_len[8],v_len[16],v_equal,v_rejected);
 static const struct config configs[]={
  {"pixel1","64 boundaries, runs of 1, 2, 4 on whole periods, 1 px",64,0x16u,256,1,0},
  {"pixel0.5","64 boundaries, runs of 1, 2, 4 on whole periods, 0.5 px",64,0x16u,128,1,0},
  {"pixel2","64 boundaries, runs of 1, 2, 4 on whole periods, 2 px",64,0x16u,512,1,0},
  {"pixel1Runs2","64 boundaries, runs of 1, 2 on whole periods, 1 px",64,0x6u,256,1,0},
  {"pixel1Single","64 boundaries, runs of 1 only (nothing to merge)",64,0x2u,256,1,0},
  {"pixel1NoEndMasks","64 boundaries of whole periods, no end masks, runs of 1, 2, 4, 1 px",64,0x16u,256,0,1},
  {"short8Pixel1","8 boundaries, runs of 1, 2, 4 on whole periods, 1 px",8,0x16u,256,1,0},
  {"short8Single","8 boundaries, runs of 1 only",8,0x2u,256,1,0},
 };
 printf("\"configs\":{");
 for(size_t i=0;i<sizeof configs/sizeof configs[0];i++)cost_config(&configs[i],i==0);
 printf("}}\n");
 return 0;
}
