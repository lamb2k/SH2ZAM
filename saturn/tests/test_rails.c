/* view-rails-1 host tests: both entry points against the independent oracle
 * in rails_oracle.h and against the loop they replace (two calls of
 * sh2z_view_edge_steps_center with the ends copied around them), on
 * road-like strips with the caller's boundary pattern, on general strips,
 * on hand cases at the limits of every step of the setup's proof, on the
 * argument rules (nothing written on failure), on aliasing, and with
 * negative controls. SH2Z_RAILS_SCALE (default 1) multiplies the random
 * counts. MIT; SH2ZAM Contributors. */
#include "rails_oracle.h"
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>

static uint64_t checks;
static void check(bool value,const char *what,unsigned line) {
    ++checks;
    if(!value) {
        fprintf(stderr,"rails check %" PRIu64 " failed at line %u: %s\n",checks,line,what);
        exit(EXIT_FAILURE);
    }
}
#define CHECK(e) check((e),#e,__LINE__)
static uint32_t rnd(void) {return ro_rnd();}
static int32_t range(int32_t lo,int32_t hi) {return ro_range(lo,hi);}

/* ---- views and ends: the oracle's fixtures, checked against the library -- */
static sh2z_view_center_t random_view(void) {
    sh2z_view_center_t w=ro_view(),c;
    sh2z_view_t v=w.view;
    CHECK(sh2z_view_init(&v,&v));
    CHECK(sh2z_view_center_init(&c,&v,w.center_x16,w.center_y16) && !memcmp(&c,&w,sizeof(c)));
    return w;
}
static sh2z_view_vertex_t corner(const sh2z_view_center_t *w,bool road) {
    const sh2z_view_vertex_t v=ro_corner(w,road);
    sh2z_view_vertex_t p=v;
    CHECK(sh2z_view_project_center(&p,w,&p,1) && !memcmp(&p,&v,sizeof(p)));
    return v;
}

/* ---- the loop view-rails-1 replaces -------------------------------------- */
static bool caller_rail(sh2z_view_vertex_t *out,const sh2z_view_center_t *w,const sh2z_view_vertex_t *from,
    const sh2z_view_vertex_t *to,const int32_t *b,unsigned count,int32_t low,int32_t high) {
    unsigned first=0,end=count;
    while(first<count && b[first]<=low)out[first++]=*from;
    while(end>first && b[end-1]>=high)out[--end]=*to;
    if(end==first)return true;
    const int64_t den=(int64_t)high-low;
    if(den<1 || den>65535)return false;
    const uint32_t step=end-first>1u?(uint32_t)b[first+1]-(uint32_t)b[first]:0u;
    return sh2z_view_edge_steps_center(out+first,w,from,to,(uint16_t)((uint32_t)b[first]-(uint32_t)low),
        (uint16_t)step,(uint16_t)den,end-first);
}

/* ---- strips ------------------------------------------------------------- */
typedef ro_strip_t strip_t;
#define road_strip ro_road_strip
#define general_strip ro_general_strip

/* ---- one comparison ------------------------------------------------------ */
typedef struct {
    uint64_t calls,ok,rejected,points,copies,near,equal_ref,mism,shared;
    uint64_t hist[66];
} tally_t;
static unsigned mutation;
static void one(tally_t *t,const sh2z_view_center_t *w,const sh2z_view_rails_t *h,const sh2z_view_vertex_t ends[4],
    const strip_t *s) {
    static sh2z_view_vertex_t ol[96],orr[96],pl[96],pr[96],dl[96],dr[96],cl[96],cr[96];
    const unsigned count=s->count;
    const bool want=ro_rails(ol,orr,w,ends,s->b,count,s->low[0],s->high[0],s->low[1],s->high[1],mutation);
    memset(pl,0x5a,sizeof(pl));memset(pr,0x5a,sizeof(pr));memset(dl,0xa5,sizeof(dl));memset(dr,0xa5,sizeof(dr));
    const bool got=sh2z_view_edge_rails_center(pl,pr,h,&ends[0],&ends[1],&ends[2],&ends[3],s->b,count,
        s->low[0],s->high[0],s->low[1],s->high[1]);
    const bool gotd=sh2z_view_edge_rails_center_divu(dl,dr,h,&ends[0],&ends[1],&ends[2],&ends[3],s->b,count,
        s->low[0],s->high[0],s->low[1],s->high[1]);
    ++t->calls;
    if(mutation) {
        if(got!=want){++t->mism;return;}
        if(!got)return;
        for(unsigned j=0;j<count;++j)if(!ro_same(&pl[j],&ol[j])||!ro_same(&pr[j],&orr[j]))++t->mism;
        return;
    }
    CHECK(got==want);CHECK(gotd==want);
    if(!got) {
        ++t->rejected;
        for(size_t i=0;i<sizeof(pl);++i)CHECK(((unsigned char *)pl)[i]==0x5a && ((unsigned char *)pr)[i]==0x5a &&
            ((unsigned char *)dl)[i]==0xa5 && ((unsigned char *)dr)[i]==0xa5);
        return;
    }
    ++t->ok;
    if(count<66u)++t->hist[count];
    if(s->low[0]==s->low[1] && s->high[0]==s->high[1])++t->shared;
    CHECK(!memcmp(pl,dl,count*sizeof(pl[0])) && !memcmp(pr,dr,count*sizeof(pr[0])));
    const bool cok=caller_rail(cl,w,&ends[0],&ends[1],s->b,count,s->low[0],s->high[0])&&
        caller_rail(cr,w,&ends[2],&ends[3],s->b,count,s->low[1],s->high[1]);
    CHECK(cok);
    for(unsigned j=0;j<count;++j) {
        CHECK(ro_same(&pl[j],&ol[j]) && ro_same(&pr[j],&orr[j]));
        CHECK(!memcmp(&pl[j],&cl[j],sizeof(pl[j])) && !memcmp(&pr[j],&cr[j],sizeof(pr[j])));
        t->equal_ref+=2;
        for(unsigned rail=0;rail<2;++rail) {
            const int32_t b=s->b[j];
            if(b<=s->low[rail]||b>=s->high[rail])++t->copies;
            else {++t->points;if((rail?pr:pl)[j].outcode&SH2Z_OUT_NEAR)++t->near;}
        }
    }
}
static void sweep(tally_t *t,unsigned views,unsigned per_view,bool road) {
    for(unsigned v=0;v<views;++v) {
        /* Half the road strips through the road camera. */
        const sh2z_view_center_t w=(road && (v&1u))?ro_road_view():random_view();
        sh2z_view_rails_t h;
        CHECK(sh2z_view_rails_prepare(&h,&w));
        for(unsigned q=0;q<per_view;++q) {
            sh2z_view_vertex_t ends[4];
            for(unsigned i=0;i<4;++i)ends[i]=corner(&w,road);
            if(rnd()%8u==0)ends[1]=ends[0];
            strip_t s;
            if(road)road_strip(&s);else general_strip(&s);
            one(t,&w,&h,ends,&s);
        }
    }
}

/* ---- hand cases: the limits of the setup's proof -------------------------- */
static void hand_cases(tally_t *t) {
    sh2z_view_t v;
    memset(&v,0,sizeof(v));
    v.r[0][0]=v.r[1][1]=v.r[2][2]=16384;v.focal_x=v.focal_y=200*65536;v.near_z=16;v.far_z=INT32_MAX;
    v.clip[0]=-160;v.clip[1]=-112;v.clip[2]=159;v.clip[3]=111;
    CHECK(sh2z_view_init(&v,&v));
    static const int32_t centres[4][2]={{0,0},{10485760,7340033},{-1,65535},{INT32_MIN,INT32_MAX}};
    static const int32_t dens[]={2,3,4,5,7,255,256,257,1023,1024,4095,4096,32767,32768,32769,46340,46341,65534,65535};
    static const int32_t vals[]={INT32_MIN,INT32_MIN+1,-65536,-1,0,1,15,16,65535,65536,INT32_MAX-1,INT32_MAX};
    for(unsigned c=0;c<4;++c) {
        sh2z_view_center_t w;
        CHECK(sh2z_view_center_init(&w,&v,centres[c][0],centres[c][1]));
        sh2z_view_rails_t h;
        CHECK(sh2z_view_rails_prepare(&h,&w));
        for(unsigned d=0;d<sizeof(dens)/sizeof(dens[0]);++d) {
            const int32_t den=dens[d];
            for(unsigned a=0;a<sizeof(vals)/sizeof(vals[0]);++a)for(unsigned e=0;e<sizeof(vals)/sizeof(vals[0]);e+=(a&1u)?1u:3u) {
                sh2z_view_vertex_t ends[4];
                memset(ends,0,sizeof(ends));
                ends[0].x=vals[a];ends[0].y=vals[e];ends[0].z=vals[(a+e)%12u];
                ends[1].x=vals[e];ends[1].y=vals[a];ends[1].z=vals[(a*7u+e)%12u];
                ends[2].x=vals[a];ends[2].y=-1-vals[e]/3;ends[2].z=16+(int32_t)(rnd()%100000u);
                ends[3].x=-1-vals[a]/5;ends[3].y=vals[e];ends[3].z=16+(int32_t)(rnd()%(1u<<30));
                for(unsigned i=0;i<4;++i)CHECK(sh2z_view_project_center(&ends[i],&w,&ends[i],1));
                /* first = 1, step = 1 up to den - 1 (as many points as fit,
                 * capped), and first = den - 1 alone. */
                strip_t s;
                const int32_t low=range(-1000,1000);
                s.count=0;s.b[s.count++]=low;
                const int32_t n=den-1<60?den-1:60;
                const int32_t step=n>1?(den-2)/(n-1)>0?(den-2)/(n-1):1:1;
                for(int32_t k=0;k<n && 1+k*step<den;++k)s.b[s.count++]=low+1+k*step;
                s.b[s.count++]=low+den;
                s.low[0]=low;s.high[0]=low+den;
                s.low[1]=s.b[s.count-2]-1;s.high[1]=s.low[1]+den;   /* first = 1 near the end */
                one(t,&w,&h,ends,&s);
            }
        }
    }
}

/* ---- argument rules --------------------------------------------------------- */
static void argument_rules(void) {
    const sh2z_view_center_t w=random_view();
    sh2z_view_rails_t h,blank;
    memset(&blank,0,sizeof(blank));
    CHECK(sh2z_view_rails_prepare(&h,&w));
    CHECK(!sh2z_view_rails_prepare(NULL,&w) && !sh2z_view_rails_prepare(&blank,NULL));
    sh2z_view_center_t bad=w;bad.center_x16+=65536;
    sh2z_view_rails_t keep;memset(&keep,0x33,sizeof(keep));
    CHECK(!sh2z_view_rails_prepare(&keep,&bad));
    for(size_t i=0;i<sizeof(keep);++i)CHECK(((unsigned char *)&keep)[i]==0x33);
    bad=w;bad.view.focal_x=0;CHECK(!sh2z_view_rails_prepare(&keep,&bad));
    sh2z_view_vertex_t ends[4];
    for(unsigned i=0;i<4;++i)ends[i]=corner(&w,true);
    static sh2z_view_vertex_t out[2][300];
    const int32_t b[5]={0,256,512,768,800};
    typedef bool (*fn_t)(sh2z_view_vertex_t *,sh2z_view_vertex_t *,const sh2z_view_rails_t *,const sh2z_view_vertex_t *,
        const sh2z_view_vertex_t *,const sh2z_view_vertex_t *,const sh2z_view_vertex_t *,const int32_t *,unsigned,int32_t,int32_t,int32_t,int32_t);
    static const fn_t fns[2]={sh2z_view_edge_rails_center,sh2z_view_edge_rails_center_divu};
    for(unsigned f=0;f<2;++f) {
        const fn_t fn=fns[f];
#define UNTOUCHED(expr) do{memset(out,0x77,sizeof(out));CHECK(!(expr));for(size_t i_=0;i_<sizeof(out);++i_)CHECK(((unsigned char *)out)[i_]==0x77);}while(0)
        UNTOUCHED(fn(out[0],out[1],&blank,&ends[0],&ends[1],&ends[2],&ends[3],b,5,0,800,0,800));
        UNTOUCHED(fn(out[0],out[1],NULL,&ends[0],&ends[1],&ends[2],&ends[3],b,5,0,800,0,800));
        UNTOUCHED(fn(NULL,out[1],&h,&ends[0],&ends[1],&ends[2],&ends[3],b,5,0,800,0,800));
        UNTOUCHED(fn(out[0],NULL,&h,&ends[0],&ends[1],&ends[2],&ends[3],b,5,0,800,0,800));
        UNTOUCHED(fn(out[0],out[1],&h,NULL,&ends[1],&ends[2],&ends[3],b,5,0,800,0,800));
        UNTOUCHED(fn(out[0],out[1],&h,&ends[0],NULL,&ends[2],&ends[3],b,5,0,800,0,800));
        UNTOUCHED(fn(out[0],out[1],&h,&ends[0],&ends[1],NULL,&ends[3],b,5,0,800,0,800));
        UNTOUCHED(fn(out[0],out[1],&h,&ends[0],&ends[1],&ends[2],NULL,b,5,0,800,0,800));
        UNTOUCHED(fn(out[0],out[1],&h,&ends[0],&ends[1],&ends[2],&ends[3],NULL,5,0,800,0,800));
        UNTOUCHED(fn(out[0],out[1],&h,&ends[0],&ends[1],&ends[2],&ends[3],b,SH2Z_VIEW_RAILS_MAX_COUNT+1u,0,800,0,800));
        /* overlapping outputs, by one record or by a byte */
        UNTOUCHED(fn(out[0],out[0]+4,&h,&ends[0],&ends[1],&ends[2],&ends[3],b,5,0,800,0,800));
        UNTOUCHED(fn(out[0]+4,out[0],&h,&ends[0],&ends[1],&ends[2],&ends[3],b,5,0,800,0,800));
        UNTOUCHED(fn(out[0],(sh2z_view_vertex_t *)(void *)((char *)out[0]+99),&h,&ends[0],&ends[1],&ends[2],&ends[3],b,5,0,800,0,800));
        /* a zero step; unsorted boundaries are fine when no run needs them */
        const int32_t eq[5]={0,256,256,768,800},down[5]={0,512,256,768,800};
        UNTOUCHED(fn(out[0],out[1],&h,&ends[0],&ends[1],&ends[2],&ends[3],eq,5,0,800,0,800));
        CHECK(fn(out[0],out[1],&h,&ends[0],&ends[1],&ends[2],&ends[3],down,5,-10,-5,1000,2000));
        for(unsigned j=0;j<5;++j)CHECK(!memcmp(&out[0][j],&ends[1],20) && !memcmp(&out[1][j],&ends[2],20));
        /* unequal spacing (the last entry is off the step); a reversed span */
        UNTOUCHED(fn(out[0],out[1],&h,&ends[0],&ends[1],&ends[2],&ends[3],b,5,-1,801,0,800));
        const int32_t rev[2]={700,100};
        UNTOUCHED(fn(out[0],out[1],&h,&ends[0],&ends[1],&ends[2],&ends[3],rev,2,600,300,1000,2000));
        {   /* only boundary[f], [f+1] and [e-1] of a run are read */
            const int32_t odd[6]={0,256,512,77,1024,1280};
            sh2z_view_vertex_t ref[4];
            CHECK(fn(out[0],out[1],&h,&ends[0],&ends[1],&ends[2],&ends[3],odd,6,0,1280,1280,0));
            CHECK(sh2z_view_edge_steps_center(ref,&w,&ends[0],&ends[1],256,256,1280,4) && !memcmp(ref,&out[0][1],sizeof(ref)));
            const int32_t past[6]={0,256,512,77,1100,1280};
            UNTOUCHED(fn(out[0],out[1],&h,&ends[0],&ends[1],&ends[2],&ends[3],past,6,0,1280,1280,0));
        }
        CHECK(fn(out[0],out[1],&h,&ends[0],&ends[1],&ends[2],&ends[3],b,5,0,800,256,800));
        CHECK(fn(out[0],out[1],&h,&ends[0],&ends[1],&ends[2],&ends[3],b,5,0,780,0,800));
        /* den 65535 accepted, 65536 rejected; a span without points is free */
        const int32_t wide[3]={0,1,65536};
        CHECK(fn(out[0],out[1],&h,&ends[0],&ends[1],&ends[2],&ends[3],wide,3,0,65535,1,65536));
        UNTOUCHED(fn(out[0],out[1],&h,&ends[0],&ends[1],&ends[2],&ends[3],wide,3,0,65536,0,1));
        CHECK(fn(out[0],out[1],&h,&ends[0],&ends[1],&ends[2],&ends[3],wide,3,1,65536,INT32_MIN,0));
        const int32_t ext[2]={INT32_MIN,INT32_MAX};
        CHECK(fn(out[0],out[1],&h,&ends[0],&ends[1],&ends[2],&ends[3],ext,2,INT32_MIN,INT32_MAX,INT32_MAX,INT32_MIN));
        CHECK(!memcmp(&out[0][0],&ends[0],20) && !memcmp(&out[0][1],&ends[1],20) && !memcmp(&out[1][0],&ends[2],20) && !memcmp(&out[1][1],&ends[2],20));
        /* count 0 succeeds with null pointers, but not with an unprepared handle */
        CHECK(fn(NULL,NULL,&h,NULL,NULL,NULL,NULL,NULL,0,0,0,0,0));
        CHECK(!fn(NULL,NULL,&blank,NULL,NULL,NULL,NULL,NULL,0,0,0,0,0));
        /* SH2Z_VIEW_RAILS_MAX_COUNT boundaries */
        static int32_t many[SH2Z_VIEW_RAILS_MAX_COUNT];
        static sh2z_view_vertex_t big[2][SH2Z_VIEW_RAILS_MAX_COUNT];
        for(unsigned j=0;j<SH2Z_VIEW_RAILS_MAX_COUNT;++j)many[j]=(int32_t)j-30000;
        CHECK(fn(big[0],big[1],&h,&ends[0],&ends[1],&ends[2],&ends[3],many,SH2Z_VIEW_RAILS_MAX_COUNT,-30000,35535,-1,1));
        CHECK(!memcmp(&big[0][0],&ends[0],20) && !memcmp(&big[0][65535],&ends[1],20) && !memcmp(&big[1][29999],&ends[2],20));
        static sh2z_view_vertex_t ref[65535];
        CHECK(sh2z_view_edge_steps_center(ref,&w,&ends[0],&ends[1],1,1,65535,65534));
        CHECK(!memcmp(ref,&big[0][1],65534*sizeof(ref[0])));
    }
}

/* ---- aliasing: boundary inside the outputs -------------------------------- */
static void aliasing(void) {
    const sh2z_view_center_t w=random_view();
    sh2z_view_rails_t h;
    CHECK(sh2z_view_rails_prepare(&h,&w));
    for(unsigned t=0;t<2000;++t) {
        strip_t s;
        road_strip(&s);
        sh2z_view_vertex_t ends[4];
        for(unsigned i=0;i<4;++i)ends[i]=corner(&w,true);
        static sh2z_view_vertex_t want_l[96],want_r[96];
        if(!ro_rails(want_l,want_r,&w,ends,s.b,s.count,s.low[0],s.high[0],s.low[1],s.high[1],0))continue;
        /* Boundary words at the start of either output. */
        static sh2z_view_vertex_t buf[2][96];
        memset(buf,0,sizeof(buf));
        int32_t *bb=(int32_t *)(void *)&buf[t&2u?1:0][0];
        memcpy(bb,s.b,s.count*sizeof(int32_t));
        const bool divu=t&1u;
        CHECK((divu?sh2z_view_edge_rails_center_divu:sh2z_view_edge_rails_center)(buf[0],buf[1],&h,
            &ends[0],&ends[1],&ends[2],&ends[3],bb,s.count,s.low[0],s.high[0],s.low[1],s.high[1]));
        CHECK(!memcmp(buf[0],want_l,s.count*sizeof(want_l[0])) && !memcmp(buf[1],want_r,s.count*sizeof(want_r[0])));
    }
}

int main(void) {
    unsigned scale=1;
    const char *e=getenv("SH2Z_RAILS_SCALE");
    if(e && atoi(e)>0)scale=(unsigned)atoi(e);
    tally_t hand,road,gen;
    memset(&hand,0,sizeof(hand));memset(&road,0,sizeof(road));memset(&gen,0,sizeof(gen));
    hand_cases(&hand);
    argument_rules();
    aliasing();
    sweep(&road,1500u*scale,200,true);
    sweep(&gen,1500u*scale,200,false);
    const tally_t *all[3]={&hand,&road,&gen};
    static const char *const names[3]={"hand cases","road strips","general strips"};
    uint64_t points=0;
    for(unsigned i=0;i<3;++i) {
        const tally_t *t=all[i];
        points+=t->points;
        printf("  %s: %" PRIu64 " calls (%" PRIu64 " accepted, %" PRIu64 " rejected, %" PRIu64 " with shared low/high), "
            "%" PRIu64 " stepped points (%" PRIu64 " NEAR), %" PRIu64 " copied ends; every byte equals the oracle and the "
            "two-call loop, portable = divu\n",names[i],t->calls,t->ok,t->rejected,t->shared,t->points,t->near,t->copies);
    }
    CHECK(road.points>100000u && gen.points>100000u && gen.rejected>0 && road.near>0 && gen.near>0);
    uint64_t sum=0,n=0;
    for(unsigned c=0;c<66u;++c){sum+=c*road.hist[c];n+=road.hist[c];}
    printf("  road strips: mean %.2f boundaries per quad (2 .. 65)\n",(double)sum/(double)n);
    static const struct { unsigned m; const char *name; } controls[]={
        {RO_EXACT_POINT,"exact rational point, no fixed-point steps"},{RO_FLOOR_NEGATIVE,"floor on the way down"},
        {RO_PREFIX_STRICT,"from copied only below low"},{RO_SUFFIX_STRICT,"to copied only above high"},
        {RO_CENTER_WHOLE,"centre fraction ignored"},{RO_STEP_CEIL,"step rounded away from zero"}};
    for(unsigned c=0;c<sizeof(controls)/sizeof(controls[0]);++c) {
        mutation=controls[c].m;ro_state=UINT32_C(0x5eed7a11);
        tally_t m;memset(&m,0,sizeof(m));
        sweep(&m,60,100,true);
        sweep(&m,60,100,false);
        printf("  control (%s): %" PRIu64 " mismatching records\n",controls[c].name,m.mism);
        CHECK(m.mism>0);
    }
    printf("view-rails-1 PASS (%" PRIu64 " stepped points, %" PRIu64 " checks)\n",points,checks);
    return 0;
}
