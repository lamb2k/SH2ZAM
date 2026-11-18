/* Independent oracle, properties, drift measurement, edge cases and negative
 * controls for the view-merge-2 tier texture merge planner. MIT; SH2ZAM
 * Contributors. */
#include "sh2zam/view_experimental.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

__extension__ typedef __int128 wide_t;
enum { MAXB=160 };

static uint64_t checks;
static uint32_t state=UINT32_C(0x7e1e0002);
static void check(bool value,const char *what) {
    ++checks;
    if(!value) {
        fprintf(stderr,"merge-tier check %llu failed: %s\n",(unsigned long long)checks,what);
        exit(EXIT_FAILURE);
    }
}
static uint32_t random_word(void) {state^=state<<13;state^=state>>17;state^=state<<5;return state;}
static int32_t random_range(int32_t lo,int32_t hi) {
    return (int32_t)((int64_t)lo+(int64_t)(random_word()%(uint64_t)((int64_t)hi-lo+1)));
}
static double frand(void) {return (double)random_word()/4294967296.0;}
static double frange(double lo,double hi) {return lo+(hi-lo)*frand();}
static wide_t absw(wide_t a) {return a<0?-a:a;}

typedef struct {
    unsigned count;
    sh2z_view_vertex_t left[MAXB],right[MAXB];
    uint32_t rows[MAXB],ends[MAXB];
    bool equal;   /* every run the end masks permit has equal-height repetitions */
} strip_t;

/* ---- oracle, written from the header text in 128-bit arithmetic ---------- */
enum { MUT_NONE=0, MUT_MIDDLE_ONLY=1, MUT_NO_RIGHT=2, MUT_STRICT=4, MUT_SMALLEST=8, MUT_ENDS=16,
       MUT_START_MASK=32, MUT_MIRROR=64, MUT_CONTROLS=7 };
#define BLOCKING (~(unsigned)(SH2Z_OUT_LEFT|SH2Z_OUT_RIGHT|SH2Z_OUT_TOP|SH2Z_OUT_BOTTOM)&0xffu)

/* F = r * (P[i+k] - P[i]) - k * (P[i+r] - P[i]) per axis; the boundary passes
 * when max(|F.x|, |F.y|) * 256 <= tol * r. */
static bool oracle_point(const sh2z_view_vertex_t *p,unsigned r,unsigned k,uint32_t tol,unsigned mutation) {
    const wide_t kk=(mutation&MUT_MIRROR)?(wide_t)(r-k):(wide_t)k;
    const wide_t fx=absw((wide_t)r*(p[k].sx-p[0].sx)-kk*(p[r].sx-p[0].sx));
    const wide_t fy=absw((wide_t)r*(p[k].sy-p[0].sy)-kk*(p[r].sy-p[0].sy));
    const wide_t m=fx>fy?fx:fy,limit=(wide_t)tol*r;
    return (mutation&MUT_STRICT)?m*256<limit:m*256<=limit;
}
static bool oracle_run(const strip_t *st,unsigned i,unsigned r,uint32_t tol,unsigned mutation) {
    for(unsigned k=1;k<r;++k) {
        if((mutation&MUT_MIDDLE_ONLY) && k!=r/2u)continue;
        if(!oracle_point(st->left+i,r,k,tol,mutation))return false;
        if(!(mutation&MUT_NO_RIGHT) && !oracle_point(st->right+i,r,k,tol,mutation))return false;
    }
    return true;
}
static bool oracle_allowed(const strip_t *st,unsigned i,unsigned r,uint32_t mask,const uint32_t *ends,unsigned mutation) {
    if(i+r>st->count-1u || !((mask>>r)&1u))return false;
    if(ends && !((ends[(mutation&MUT_START_MASK)?i:i+r]>>r)&1u))return false;
    const unsigned from=(mutation&MUT_ENDS)?i+1u:i,to=(mutation&MUT_ENDS)?i+r-1u:i+r;
    for(unsigned j=from;j<=to;++j)if((st->left[j].outcode|st->right[j].outcode)&BLOCKING)return false;
    return true;
}
static int oracle_plan(uint8_t *out,const strip_t *st,uint32_t mask,const uint32_t *ends,uint32_t tol,unsigned mutation) {
    static const unsigned longest[4]={16,8,4,2},shortest[4]={2,4,8,16};
    unsigned i=0;
    int n=0;
    while(i<st->count-1u) {
        unsigned best=1;
        for(unsigned c=0;c<4u;++c) {
            const unsigned r=(mutation&MUT_SMALLEST)?shortest[c]:longest[c];
            if(oracle_allowed(st,i,r,mask,ends,mutation) && oracle_run(st,i,r,tol,mutation)){best=r;break;}
        }
        out[n++]=(uint8_t)best;
        i+=best;
    }
    return n;
}

/* ---- strip generators ----------------------------------------------------- */
static sh2z_view_t screen_view(double focal) {
    sh2z_view_t v;
    memset(&v,0,sizeof(v));
    v.r[0][0]=v.r[1][1]=v.r[2][2]=16384;
    v.focal_x=v.focal_y=(int32_t)(focal*65536.0);
    v.center_x=160;v.center_y=112;
    v.near_z=4096;v.far_z=4000*16384;
    v.clip[0]=0;v.clip[1]=0;v.clip[2]=319;v.clip[3]=223;
    check(sh2z_view_init(&v,&v),"screen view valid");
    return v;
}
static int32_t q14(double a) {
    const double s=floor(a*16384.0+0.5);
    return (int32_t)(s>2147483647.0?2147483647.0:s<-2147483648.0?-2147483648.0:s);
}
/* Rows and end masks the way a road renderer with tier textures makes them:
 * the strip starts at texture position low and ends at high; boundaries
 * fall on every whole period (unit rows) between them, so the first and last
 * repetitions may be partial. A run of r (r <= factor) may end at boundary j
 * when it starts on a period boundary and covers r whole periods. Returns the
 * position of each boundary in periods, for the geometry. */
static void tier_rows(strip_t *st,unsigned count,uint32_t unit,unsigned factor,double *at) {
    const uint32_t phase=random_word()%3u?random_word()%unit:0u;
    const uint32_t tail=random_word()%3u?1u+random_word()%unit:unit;
    st->count=count;
    st->rows[0]=0;
    for(unsigned j=1;j<count;++j)st->rows[j]=j+1u<count?(uint32_t)j*unit-phase:(uint32_t)(count-2u)*unit-phase+tail;
    if(count==2u)st->rows[1]=tail;
    for(unsigned j=0;j<count;++j) {
        at[j]=((double)st->rows[j]+phase)/unit;
        st->ends[j]=0;
        for(unsigned r=2;r<=16u;r*=2u)
            if(r<=factor && j>=r && (st->rows[j-r]+phase)%unit==0u && st->rows[j]-st->rows[j-r]==unit*r)st->ends[j]|=UINT32_C(1)<<r;
    }
    st->equal=true;
}
/* A curved, hilly, banked road seen from a synthetic camera, projected by the
 * library, with boundaries at the positions at[] (in repetitions). */
static void road_geometry(strip_t *st,const double *at,bool timing) {
    const double focal=frange(120.0,320.0);
    const sh2z_view_t v=screen_view(focal);
    const double yaw=frange(-0.5,0.5)*(timing?0.4:1.0),pitch=frange(-0.35,0.05),roll=random_word()%5u<2u?frange(-0.3,0.3):0.0;
    const double h=frange(0.3,timing?3.5:8.0);
    const double s0=timing?frange(1.0,21.0):random_word()%7u==0u?frange(-20.0,0.0):exp(frange(log(0.3),log(300.0)));
    const double rep=exp(frange(log(0.2),log(12.0))),w=frange(0.5,timing?5.5:12.0),off=frange(-10.0,10.0)*(timing?0.3:1.0);
    const bool straight=random_word()%3u==0u;
    const double kappa=straight?0.0:frange(-0.06,0.06),dk=straight?0.0:frange(-0.002,0.002);
    const double slope=frange(-0.15,0.15),hill_a=straight?0.0:frange(0.0,3.0),hill_f=frange(0.01,0.2);
    const double bank=random_word()%2u?frange(-0.4,0.4):0.0;
    const double cy=cos(yaw),sy=sin(yaw),cp=cos(pitch),sp=sin(pitch),cr=cos(roll),sr=sin(roll);
    static sh2z_view_vertex_t lv[MAXB],rv[MAXB];
    memset(lv,0,sizeof(lv));memset(rv,0,sizeof(rv));
    double cx=off,cz=0.0,s=0.0;
    for(unsigned k=0;k<st->count;++k) {
        const double target=s0+rep*at[k];
        const unsigned steps=k?16u:64u;
        const double ds=(target-s)/steps;
        for(unsigned q=0;q<steps;++q) {
            const double m=s+ds*(q+0.5),th=kappa*m+0.5*dk*m*m;
            cx+=ds*sin(th);cz+=ds*cos(th);
        }
        s=target;
        const double th=kappa*s+0.5*dk*s*s,ccy=-h+slope*s+hill_a*sin(hill_f*s);
        for(unsigned side=0;side<2u;++side) {
            const double sign=side?-1.0:1.0;
            const double p[3]={cx-sign*w*cos(bank)*cos(th),ccy+sign*w*sin(bank),cz+sign*w*cos(bank)*sin(th)};
            const double a0=cy*p[0]-sy*p[2],a2=sy*p[0]+cy*p[2];
            const double b1=cp*p[1]+sp*a2,b2=-sp*p[1]+cp*a2;
            const double vx=cr*a0+sr*b1,vy=-sr*a0+cr*b1;
            sh2z_view_vertex_t *o=side?&rv[k]:&lv[k];
            o->x=q14(vx);o->y=q14(vy);o->z=q14(b2);
        }
    }
    check(sh2z_view_project(st->left,&v,lv,st->count) && sh2z_view_project(st->right,&v,rv,st->count),"road projection");
}
/* One straight quad as a road renderer steps it: both rails straight
 * view-space segments, stepped at the boundaries by sh2z_view_edge_steps. */
static void quad_geometry(strip_t *st) {
    const sh2z_view_t v=screen_view(frange(120.0,320.0));
    const double z0=exp(frange(log(0.5),log(200.0))),len=exp(frange(log(0.5),log(80.0))),x0=frange(-15.0,15.0),y0=frange(-6.0,-0.3);
    const double w=frange(1.0,12.0),turn=frange(-0.4,0.4),climb=frange(-0.1,0.1);
    sh2z_view_vertex_t ends[4];
    memset(ends,0,sizeof(ends));
    for(unsigned side=0;side<2u;++side)for(unsigned e=0;e<2u;++e) {
        sh2z_view_vertex_t *o=&ends[side*2u+e];
        const double x=x0+(side?w:0.0)+(e?len*turn:0.0),y=y0+(e?len*climb:0.0),z=z0+(e?len:0.0)+(side?w*0.1:0.0);
        o->x=q14(x);o->y=q14(y);o->z=q14(z);
    }
    const uint32_t den=st->rows[st->count-1];
    check(den>0u && den<=65535u,"quad row span fits the edge stepper");
    for(unsigned side=0;side<2u;++side) {
        sh2z_view_vertex_t *p=side?st->right:st->left;
        check(sh2z_view_project(&p[0],&v,&ends[side*2u],1) && sh2z_view_project(&p[st->count-1],&v,&ends[side*2u+1u],1),"quad corners");
        for(unsigned j=1;j+1u<st->count;++j)
            check(sh2z_view_edge_steps(&p[j],&v,&ends[side*2u],&ends[side*2u+1u],(uint16_t)st->rows[j],0,(uint16_t)den,1),"edge step");
    }
}
/* Corners a renderer will clip instead: far outside a +-900 band get 0x80. */
static void mark_band(strip_t *st) {
    for(unsigned j=0;j<st->count;++j)for(unsigned side=0;side<2u;++side) {
        sh2z_view_vertex_t *p=side?&st->right[j]:&st->left[j];
        if(p->sx< -900 || p->sx>900 || p->sy< -900 || p->sy>900)p->outcode|=0x80u;
    }
}
static unsigned tier_strip(strip_t *st,bool straight_quad) {
    static const uint32_t units[3]={256,512,1024};
    static const unsigned factors[4]={4,4,2,1};
    const uint32_t unit=units[random_word()%3u];
    const unsigned factor=random_word()%8u==0u?16u:random_word()%8u==1u?8u:factors[random_word()%4u];
    unsigned count=straight_quad?2u+random_word()%(factor>4u?40u:20u):(random_word()%2u?64u:2u+random_word()%(MAXB-1u));
    if(straight_quad && (uint64_t)count*unit>65535u)count=(unsigned)(65535u/unit);
    double at[MAXB];
    tier_rows(st,count,unit,factor,at);
    if(straight_quad)quad_geometry(st);else road_geometry(st,at,false);
    mark_band(st);
    return factor;
}
static int16_t extreme16(void) {
    switch(random_word()%4u) {
    case 0: return (int16_t)random_word();
    case 1: return random_word()&1u?INT16_MAX:INT16_MIN;
    case 2: return (int16_t)(random_word()&1u?INT16_MAX-random_range(0,2):INT16_MIN+random_range(0,2));
    default: return (int16_t)random_range(-2000,2000);
    }
}
/* Adversarial strips: int16 extremes, rails linear in the boundary index
 * plus 0 or +-1 (every test sits at its threshold), random clip flags,
 * random end masks. Rows are equal steps, so the cross-check against
 * sh2z_view_merge_runs applies without end masks. */
static void adversarial_strip(strip_t *st) {
    st->count=2u+random_word()%(MAXB-1u);
    st->equal=true;
    memset(st->left,0,sizeof(st->left));memset(st->right,0,sizeof(st->right));
    const unsigned kind=random_word()%3u;
    const uint32_t step=1u+random_word()%4096u;
    for(unsigned k=0;k<st->count;++k)st->rows[k]=step*k;
    for(unsigned side=0;side<2u;++side) {
        sh2z_view_vertex_t *p=side?st->right:st->left;
        if(kind==0u) {
            for(unsigned k=0;k<st->count;++k){p[k].sx=extreme16();p[k].sy=extreme16();}
        } else {
            const double ax=extreme16(),ay=extreme16(),bx=extreme16(),by=extreme16();
            const double bow=kind==2u?frange(-3.0,3.0):0.0;
            for(unsigned k=0;k<st->count;++k) {
                const double f=(double)k/(st->count-1u);
                double x=floor(ax+(bx-ax)*f+bow*f*(1-f)*4+0.5),y=floor(ay+(by-ay)*f+0.5);
                if(random_word()%3u==0u)x+=(double)random_range(-1,1);
                if(random_word()%3u==0u)y+=(double)random_range(-1,1);
                p[k].sx=(int16_t)(x>32767.0?32767.0:x<-32768.0?-32768.0:x);
                p[k].sy=(int16_t)(y>32767.0?32767.0:y<-32768.0?-32768.0:y);
            }
        }
        for(unsigned k=0;k<st->count;++k) {
            const uint32_t roll_=random_word()%100u;
            p[k].outcode=(uint8_t)(roll_<3u?random_word():roll_<40u?(random_word()&0x3cu):0u);
            p[k].x=(int32_t)random_word();p[k].z=(int32_t)random_word();   /* never read */
        }
    }
    for(unsigned j=0;j<st->count;++j)st->ends[j]=random_word()%4u?random_word():0u;
}
static uint32_t random_mask(void) {
    switch(random_word()%10u) {
    case 0: case 1: case 2: case 3: return 0x16u;
    case 4: return 0x06u;
    case 5: return 0x12u;
    case 6: return 0x116u;
    case 7: return SH2Z_VIEW_MERGE_TIER_RUNS|1u;
    case 8: return 0x02u;
    default: return random_word()&(SH2Z_VIEW_MERGE_TIER_RUNS|1u);
    }
}
static uint32_t random_tol(void) {
    static const uint32_t pixels[]={256,256,128,512,64,1024,0,3,255,257};
    const uint32_t r=random_word()%12u;
    if(r<10u)return pixels[r];
    return r==10u?SH2Z_VIEW_MERGE_MAX_TOL-(random_word()%2u):random_word()%4096u;
}

/* ---- the documented bound, in floating point ------------------------------ */
typedef struct {double x,y;} pt_t;
static pt_t ptv(const sh2z_view_vertex_t *a) {pt_t p={a->sx,a->sy};return p;}
static pt_t lerp(pt_t a,pt_t b,double f) {pt_t p={a.x+(b.x-a.x)*f,a.y+(b.y-a.y)*f};return p;}
static double cheb(pt_t a,pt_t b) {const double dx=fabs(a.x-b.x),dy=fabs(a.y-b.y);return dx>dy?dx:dy;}

/* Largest boundary displacement of a run on both rails, in pixels, with the
 * reference's equal-height repetitions. */
static double run_boundary_error(const strip_t *st,unsigned i,unsigned r) {
    double e=0;
    for(unsigned side=0;side<2u;++side) {
        const sh2z_view_vertex_t *p=side?st->right:st->left;
        for(unsigned k=1;k<r;++k) {
            const double d=cheb(ptv(&p[i+k]),lerp(ptv(&p[i]),ptv(&p[i+r]),(double)k/r));
            if(d>e)e=d;
        }
    }
    return e;
}

/* Texel drift: every sampled texel (u across, v along) of a merged quad,
 * merged position against the per-repetition reference, in pixels. The
 * histogram has 1/1024 px bins up to 16 px. */
enum { BINS=16*1024 };
typedef struct { uint64_t runs,samples,hist[BINS+1]; double max,max_vs_boundary,max_ratio; } drift_t;
static void drift_run(drift_t *d,const strip_t *st,unsigned i,unsigned r,uint32_t tol,bool check_bound) {
    enum { Q=12, U=8 };
    const double bmax=run_boundary_error(st,i,r);
    const pt_t l0=ptv(&st->left[i]),ln=ptv(&st->left[i+r]),r0=ptv(&st->right[i]),rn=ptv(&st->right[i+r]);
    double worst=0;
    for(unsigned j=0;j<r;++j) {
        const pt_t lj=ptv(&st->left[i+j]),lj1=ptv(&st->left[i+j+1]),rj=ptv(&st->right[i+j]),rj1=ptv(&st->right[i+j+1]);
        for(unsigned q=(j?1u:0u);q<=Q;++q) {
            const double f=(double)q/Q,v=(j+f)/r;
            const pt_t lm=lerp(l0,ln,v),rm=lerp(r0,rn,v),lr=lerp(lj,lj1,f),rr=lerp(rj,rj1,f);
            for(unsigned uq=0;uq<=U;++uq) {
                const double u=(double)uq/U,e=cheb(lerp(lm,rm,u),lerp(lr,rr,u));
                if(e>worst)worst=e;
                const double b=e*1024.0;
                d->hist[b>=BINS?BINS:(unsigned)b]++;
                d->samples++;
            }
        }
    }
    d->runs++;
    if(worst>d->max)d->max=worst;
    if(worst-bmax>d->max_vs_boundary)d->max_vs_boundary=worst-bmax;
    if(tol && worst/(tol/256.0)>d->max_ratio)d->max_ratio=worst/(tol/256.0);
    if(check_bound) {
        check(worst<=bmax+1e-9*(1.0+bmax),"texel drift is largest at a boundary vertex");
        check(worst>=bmax-1e-9*(1.0+bmax),"the boundary maximum is attained");
        check(worst<=tol/256.0+1e-9,"texel drift within the tolerance");
    }
}
static double drift_quantile(const drift_t *d,double q) {
    const uint64_t want=(uint64_t)ceil(q*(double)d->samples);
    uint64_t seen=0;
    for(unsigned b=0;b<=BINS;++b){seen+=d->hist[b];if(seen>=want)return (b+1u)/1024.0;}
    return 16.0;
}
static void plan_drift(drift_t *d,const strip_t *st,const uint8_t *runs,int n,uint32_t tol,bool check_bound) {
    unsigned i=0;
    for(int k=0;k<n;++k){if(runs[k]>1u)drift_run(d,st,i,runs[k],tol,check_bound);i+=runs[k];}
}

/* ---- sweep ---------------------------------------------------------------- */
static void plan_properties(const strip_t *st,const uint8_t *runs,int n,uint32_t mask,const uint32_t *ends,uint32_t tol) {
    unsigned i=0;
    for(int k=0;k<n;++k) {
        const unsigned r=runs[k];
        check(r==1u || r==2u || r==4u || r==8u || r==16u,"run length is a tier length");
        check(i+r<=st->count-1u,"run inside the strip");
        if(r>1u) {
            check(((mask>>r)&1u)!=0,"run allowed by the mask");
            check(!ends || ((ends[i+r]>>r)&1u),"run allowed by its end mask");
            for(unsigned j=i;j<=i+r;++j)check(!((st->left[j].outcode|st->right[j].outcode)&BLOCKING),"no blocked boundary in a merged run");
            check(run_boundary_error(st,i,r)<=tol/256.0*(1.0+1e-12)+1e-12,"pixel bound holds");
        }
        i+=r;
    }
    check(i==st->count-1u,"runs cover every repetition");
}

static void sweep(void) {
    enum { TIER=40000, QUADS=40000, ADVERSARIAL=60000, CONTROLS=7 };
    static strip_t st;
    uint64_t compared=0,merged=0,boundaries=0,equal_v1=0,equal_v1_runs=0,blocked_strips=0;
    uint64_t control[CONTROLS]={0},lengths[17]={0};
    static const unsigned controls[CONTROLS]={MUT_MIDDLE_ONLY,MUT_NO_RIGHT,MUT_STRICT,MUT_SMALLEST,MUT_ENDS,MUT_START_MASK,MUT_MIRROR};
    static const char *const names[CONTROLS]={"only the middle boundary tested","right rail not checked",
        "strict < at the threshold","greedy takes the smallest passing run","NEAR/FAR checked only inside the run",
        "end mask read at the run's start","interior boundary k tested as r - k"};
    static drift_t drift;
    for(unsigned trial=0;trial<TIER+QUADS+ADVERSARIAL;++trial) {
        const unsigned kind=trial<TIER?0u:trial<TIER+QUADS?1u:2u;
        unsigned factor=16;
        if(kind<2u)factor=tier_strip(&st,kind==1u);else adversarial_strip(&st);
        const uint32_t mask=kind<2u?(factor>=16u?0x10116u:factor==8u?0x116u:factor==4u?0x16u:factor==2u?0x6u:0x2u):random_mask();
        const uint32_t *ends=kind<2u?(random_word()%8u?st.ends:NULL):(random_word()%2u?st.ends:NULL);
        const uint32_t tol=random_tol();
        uint8_t got[MAXB+8],want[MAXB+8],sentinel[MAXB+8];
        memset(got,0xa5,sizeof(got));memcpy(sentinel,got,sizeof(got));
        const int n=sh2z_view_merge_tier(got,st.left,st.right,ends,st.count,mask,tol);
        const int w=oracle_plan(want,&st,mask,ends,tol,MUT_NONE);
        if(n!=w || memcmp(got,want,(size_t)n)) {
            fprintf(stderr,"trial %u kind %u count %u mask %08x ends %d tol %u n %d w %d\n",trial,kind,st.count,mask,ends!=NULL,tol,n,w);
            for(int k=0;k<n||k<w;++k)fprintf(stderr,"  %d: got %d want %d\n",k,k<n?got[k]:-1,k<w?want[k]:-1);
        }
        check(n==w,"run count equals the oracle");
        check(!memcmp(got,want,(size_t)n),"runs equal the oracle");
        check(!memcmp(got+n,sentinel+n,sizeof(got)-(size_t)n),"nothing written past the runs");
        compared+=(uint64_t)n;boundaries+=st.count;
        plan_properties(&st,got,n,mask,ends,tol);
        for(int k=0;k<n;++k){merged+=got[k]>1u;lengths[got[k]]++;}
        for(unsigned c=0;c<CONTROLS;++c) {
            uint8_t m[MAXB+8];
            const int nm=oracle_plan(m,&st,mask,ends,tol,controls[c]);
            if(nm!=n || memcmp(m,got,(size_t)n))++control[c];
        }
        /* The same plan as view-merge-1 in pixel mode whenever every
         * permitted run has equal-height repetitions: tier strips with their
         * end masks, and equal-step strips with or without them. */
        if(kind==2u || ends) {
            uint8_t v1[MAXB+8];
            const int n1=sh2z_view_merge_runs(v1,st.left,st.right,st.rows,ends,st.count,mask,tol,SH2Z_VIEW_MERGE_PIXELS);
            check(n1==n && !memcmp(v1,got,(size_t)n),"equals sh2z_view_merge_runs on equal-height runs");
            equal_v1++;equal_v1_runs+=(uint64_t)n;
        }
        if(kind==2u) {
            static strip_t clean;
            clean=st;
            for(unsigned j=0;j<st.count;++j){clean.left[j].outcode&=0x3cu;clean.right[j].outcode&=0x3cu;}
            uint8_t m[MAXB+8];
            blocked_strips+=oracle_plan(m,&clean,mask,ends,tol,MUT_NONE)!=n;
        }
        if(kind<2u && tol<=1024u)plan_drift(&drift,&st,got,n,tol,true);
    }
    printf("sweep: %u tier road strips, %u stepped quads, %u adversarial strips; %llu boundaries; %llu runs equal the oracle (%llu merged: x2 %llu, x4 %llu, x8 %llu, x16 %llu)\n",
        TIER,QUADS,ADVERSARIAL,(unsigned long long)boundaries,(unsigned long long)compared,(unsigned long long)merged,
        (unsigned long long)lengths[2],(unsigned long long)lengths[4],(unsigned long long)lengths[8],(unsigned long long)lengths[16]);
    printf("  %llu plans (%llu runs) equal sh2z_view_merge_runs in pixel mode; %llu adversarial strips whose plan changes when the NEAR/FAR/undefined flags are cleared\n",
        (unsigned long long)equal_v1,(unsigned long long)equal_v1_runs,(unsigned long long)blocked_strips);
    for(unsigned c=0;c<CONTROLS;++c)printf("control '%s': %llu plans differ\n",names[c],(unsigned long long)control[c]);
    printf("drift: %llu merged quads, %llu texel samples; max %.4g x tol, never above the boundary maximum (+%.3g px)\n",
        (unsigned long long)drift.runs,(unsigned long long)drift.samples,drift.max_ratio,drift.max_vs_boundary);
    check(merged>100000u && lengths[8]>100u && lengths[16]>100u && blocked_strips>1000u,"sweep coverage");
    for(unsigned c=0;c<CONTROLS;++c)check(control[c]>0,"every control is detected");
    check(drift.runs>50000u && drift.max_vs_boundary<=1e-9 && drift.max_ratio<=1.0+1e-12,"drift within the bound");
}

/* ---- the road renderer's case: commands and drift at 0.5, 1 and 2 px ------- */
static void caller_case(void) {
    enum { STRIPS=4000 };
    static const uint32_t tols[3]={128,256,512};
    static strip_t st;
    static drift_t drift[3],middle[3];
    uint64_t quads_tier[3]={0},quads_v1[3]={0},quads_free[3]={0},quads_middle[3]={0};
    double strips=0;
    state=UINT32_C(0x71e0c311);
    for(unsigned t=0;t<STRIPS;++t) {
        double at[MAXB];
        static const uint32_t units[3]={256,512,1024};
        tier_rows(&st,64,units[random_word()%3u],4,at);
        road_geometry(&st,at,true);
        mark_band(&st);
        strips++;
        for(unsigned c=0;c<3u;++c) {
            uint8_t a[MAXB],b[MAXB],f[MAXB],m[MAXB];
            const int na=sh2z_view_merge_tier(a,st.left,st.right,st.ends,64,0x16u,tols[c]);
            const int nb=sh2z_view_merge_runs(b,st.left,st.right,st.rows,st.ends,64,0x16u,tols[c],SH2Z_VIEW_MERGE_PIXELS);
            const int nf=sh2z_view_merge_runs(f,st.left,st.right,st.rows,NULL,64,0x16u,tols[c],SH2Z_VIEW_MERGE_PIXELS);
            const int nm=oracle_plan(m,&st,0x16u,st.ends,tols[c],MUT_MIDDLE_ONLY);
            check(na==nb && !memcmp(a,b,(size_t)na),"caller case equals view-merge-1");
            quads_tier[c]+=(uint64_t)na;quads_v1[c]+=(uint64_t)nb;quads_free[c]+=(uint64_t)nf;quads_middle[c]+=(uint64_t)nm;
            plan_drift(&drift[c],&st,a,na,tols[c],true);
            plan_drift(&middle[c],&st,m,nm,tols[c],false);
        }
    }
    for(unsigned c=0;c<3u;++c) {
        printf("road renderer case, %.1f px, runs {1,2,4} on whole periods, 64 boundaries: %.3f quads per 63 repetitions "
               "(view-merge-1 with the same end masks %.3f; without end masks %.3f; middle-boundary-only test %.3f)\n",
            tols[c]/256.0,(double)quads_tier[c]/strips,(double)quads_v1[c]/strips,(double)quads_free[c]/strips,(double)quads_middle[c]/strips);
        printf("  texel drift on %llu merged quads (%llu samples): p50 %.3f, p99 %.3f, max %.4f px (bound %.3f px); "
               "middle-only test: p99 %.3f, max %.3f px\n",
            (unsigned long long)drift[c].runs,(unsigned long long)drift[c].samples,drift_quantile(&drift[c],0.5),
            drift_quantile(&drift[c],0.99),drift[c].max,tols[c]/256.0,drift_quantile(&middle[c],0.99),middle[c].max);
        check(quads_tier[c]==quads_v1[c],"no more commands than view-merge-1");
        check(drift[c].max<=tols[c]/256.0+1e-12,"drift within the tolerance");
    }
    check(middle[1].max>1.0,"a middle-only test would exceed the tolerance");
}

/* ---- hand cases ----------------------------------------------------------- */
static void set_vertex(sh2z_view_vertex_t *p,int32_t x,int32_t y) {memset(p,0,sizeof(*p));p->sx=(int16_t)x;p->sy=(int16_t)y;}
static void hand_cases(void) {
    static strip_t st;
    uint8_t runs[MAXB];
    /* Linear on screen: every merge is exact, even at tolerance 0. */
    st.count=64;
    for(unsigned k=0;k<64u;++k){set_vertex(&st.left[k],100,(int32_t)(10+8*k));set_vertex(&st.right[k],200,(int32_t)(10+8*k));}
    int n=sh2z_view_merge_tier(runs,st.left,st.right,NULL,64,SH2Z_VIEW_MERGE_TIER_RUNS,0);
    check(n==7 && runs[0]==16 && runs[2]==16 && runs[3]==8 && runs[4]==4 && runs[5]==2 && runs[6]==1,"linear strip: 16 x 3, 8, 4, 2, 1");
    n=sh2z_view_merge_tier(runs,st.left,st.right,NULL,64,0x16u,0);
    check(n==17 && runs[14]==4 && runs[15]==2 && runs[16]==1,"linear strip, runs {1,2,4}: 15 x 4, 2, 1");
    n=sh2z_view_merge_tier(runs,st.left,st.right,NULL,64,0x2u,256);
    check(n==63 && runs[0]==1 && runs[62]==1,"no run longer than 1 allowed: singles");
    n=sh2z_view_merge_tier(runs,st.left,st.right,NULL,64,0,256);
    check(n==63,"empty mask: singles");
    /* Blocking flags. */
    st.left[9].outcode=SH2Z_OUT_NEAR;st.right[20].outcode=SH2Z_OUT_FAR;
    n=sh2z_view_merge_tier(runs,st.left,st.right,NULL,64,0x16u,0);
    {
        static const uint8_t want[]={4,4,1,1,4,4,1,1,1,4,4,4,4,4,4,4,4,4,4,2};
        check(n==(int)sizeof(want) && !memcmp(runs,want,sizeof(want)),"blocked boundaries are never inside or at the end of a merged run");
    }
    st.left[9].outcode=SH2Z_OUT_LEFT|SH2Z_OUT_TOP;st.right[20].outcode=SH2Z_OUT_BOTTOM;
    check(sh2z_view_merge_tier(runs,st.left,st.right,NULL,64,0x16u,0)==17,"side bits do not block");
    st.left[9].outcode=0x40;
    check(sh2z_view_merge_tier(runs,st.left,st.right,NULL,64,0x16u,0)==18,"undefined bits block");
    st.left[9].outcode=0x80;
    check(sh2z_view_merge_tier(runs,st.left,st.right,NULL,64,0x16u,0)==18,"bit 0x80 blocks");
    st.left[9].outcode=0;st.right[20].outcode=0;
    /* End masks: runs of 4 only on multiples of 4 from 2, runs of 2 on even boundaries. */
    for(unsigned j=0;j<64u;++j)st.ends[j]=((j%2u==0u)?4u:0u)|(((j+2u)%4u==0u)?16u:0u);
    n=sh2z_view_merge_tier(runs,st.left,st.right,st.ends,64,0x16u,0);
    check(n==17 && runs[0]==2 && runs[1]==4 && runs[15]==4 && runs[16]==1,"end masks align the runs");
    /* Thresholds. Run of 2: F = 2 P1 - P0 - P2; 2 px bow -> F = 4 (x) needs
     * tol * 2 >= 4 * 256, i.e. tol >= 512. */
    st.count=3;
    set_vertex(&st.left[0],0,0);set_vertex(&st.left[1],2,5);set_vertex(&st.left[2],0,10);
    set_vertex(&st.right[0],50,0);set_vertex(&st.right[1],50,5);set_vertex(&st.right[2],50,10);
    check(sh2z_view_merge_tier(runs,st.left,st.right,NULL,3,0x6u,512)==1 && runs[0]==2,"run of 2 passes at 2 px");
    check(sh2z_view_merge_tier(runs,st.left,st.right,NULL,3,0x6u,511)==2,"and fails just under");
    /* Run of 4, middle boundary 1 px off the chord: F2 = 4 -> tol >= 256;
     * the floor: tol 383 gives |m| <= 2 (F2 <= 5) and passes a 2 px middle
     * error only from tol 512. */
    st.count=5;
    for(unsigned k=0;k<5u;++k){set_vertex(&st.left[k],0,(int32_t)(4*k));set_vertex(&st.right[k],60,(int32_t)(4*k));}
    st.left[2].sx=1;
    check(sh2z_view_merge_tier(runs,st.left,st.right,NULL,5,0x16u,256)==1 && runs[0]==4,"middle 1 px off passes at 1 px");
    check(sh2z_view_merge_tier(runs,st.left,st.right,NULL,5,0x16u,255)==2 && runs[0]==2 && runs[1]==2,"and fails just under");
    st.left[2].sx=2;
    check(sh2z_view_merge_tier(runs,st.left,st.right,NULL,5,0x16u,511)==2 && sh2z_view_merge_tier(runs,st.left,st.right,NULL,5,0x16u,512)==1,"2 px middle at 512");
    /* Quarter boundary: 1 px off -> F1 = 4 -> 1 px; 3/4 boundary on the right rail. */
    st.left[2].sx=0;st.left[1].sy=5;
    check(sh2z_view_merge_tier(runs,st.left,st.right,NULL,5,0x16u,256)==1 && sh2z_view_merge_tier(runs,st.left,st.right,NULL,5,0x16u,255)==3,"quarter boundary threshold");
    st.left[1].sy=4;st.right[3].sx=59;
    check(sh2z_view_merge_tier(runs,st.left,st.right,NULL,5,0x16u,256)==1 && sh2z_view_merge_tier(runs,st.left,st.right,NULL,5,0x16u,255)==3,"three-quarter boundary threshold, right rail");
    /* Runs of 8 and 16 with one boundary 1 px off: passes at exactly 1 px. */
    for(unsigned r=8;r<=16u;r*=2u) {
        st.count=r+1u;
        for(unsigned k=0;k<=r;++k){set_vertex(&st.left[k],-7,(int32_t)(3*k));set_vertex(&st.right[k],40,(int32_t)(3*k));}
        st.right[r-1u].sy=(int16_t)(st.right[r-1u].sy+1);
        const uint32_t m=UINT32_C(1)<<r|0x6u;
        check(sh2z_view_merge_tier(runs,st.left,st.right,NULL,r+1u,m,256)==1 && runs[0]==r,"long run passes at 1 px");
        check(sh2z_view_merge_tier(runs,st.left,st.right,NULL,r+1u,m,255)>1,"and fails just under");
    }
    /* int16 extremes: a run of 16 from -32768 to 32767 with every interior
     * boundary at an extreme; |F| close to 2^21 must not wrap. */
    st.count=17;
    for(unsigned k=0;k<=16u;++k){set_vertex(&st.left[k],k==0u?INT16_MIN:INT16_MAX,k%2u?INT16_MIN:INT16_MAX);st.right[k]=st.left[k];}
    check(sh2z_view_merge_tier(runs,st.left,st.right,NULL,17,0x10000u,SH2Z_VIEW_MERGE_MAX_TOL)==1,"maximum tolerance passes the worst run");
    check(sh2z_view_merge_tier(runs,st.left,st.right,NULL,17,0x10000u,SH2Z_VIEW_MERGE_MAX_TOL/32u)==16,"and a smaller one fails it");
    uint8_t m[20];
    const int w=oracle_plan(m,&st,0x10000u,NULL,SH2Z_VIEW_MERGE_MAX_TOL/32u,MUT_NONE);
    check(w==16,"oracle agrees on the extreme strip");
    printf("hand cases: linear strips, blocking flags, end masks, thresholds for runs of 2, 4, 8, 16 and int16 extremes\n");
}

static void argument_rules(void) {
    enum { N=8 };
    sh2z_view_vertex_t l[N],r[N];
    uint32_t ends[N];
    memset(l,0,sizeof(l));memset(r,0,sizeof(r));
    for(unsigned k=0;k<N;++k){ends[k]=~0u;l[k].sy=(int16_t)k;r[k].sy=(int16_t)k;}
    uint8_t out[N],sentinel[N];
    memset(out,0x5a,sizeof(out));memcpy(sentinel,out,sizeof(out));
    const int bad=SH2Z_VIEW_MERGE_INVALID;
    check(sh2z_view_merge_tier(NULL,l,r,NULL,N,0x16u,256)==bad,"null out");
    check(sh2z_view_merge_tier(out,NULL,r,NULL,N,0x16u,256)==bad,"null left");
    check(sh2z_view_merge_tier(out,l,NULL,NULL,N,0x16u,256)==bad,"null right");
    check(sh2z_view_merge_tier(out,l,r,NULL,0,0x16u,256)==bad && sh2z_view_merge_tier(out,l,r,NULL,1,0x16u,256)==bad,"count >= 2");
    check(sh2z_view_merge_tier(out,l,r,NULL,SH2Z_VIEW_MERGE_MAX_COUNT+1u,0x16u,256)==bad,"count <= max");
    check(sh2z_view_merge_tier(out,l,r,NULL,N,0x16u,SH2Z_VIEW_MERGE_MAX_TOL+1u)==bad,"tolerance <= max");
    check(sh2z_view_merge_tier(out,l,r,NULL,N,0x1eu,256)==bad && sh2z_view_merge_tier(out,l,r,NULL,N,0x80000016u,256)==bad &&
          sh2z_view_merge_tier(out,l,r,NULL,N,0x20000u,256)==bad,"run lengths other than 1, 2, 4, 8, 16");
    check(!memcmp(out,sentinel,sizeof(out)),"rejected calls write nothing");
    check(sh2z_view_merge_tier(out,l,r,NULL,N,0x17u,256)>0 && sh2z_view_merge_tier(out,l,r,NULL,N,0x10117u,256)>0,"bit 0 is ignored");
    /* Overlap with any input array. */
    static _Alignas(8) uint8_t arena[4096];
    sh2z_view_vertex_t *al=(sh2z_view_vertex_t *)(void *)arena,*arr=(sh2z_view_vertex_t *)(void *)(arena+3072);
    uint32_t *ae=(uint32_t *)(void *)(arena+2048);
    memcpy(al,l,sizeof(l));memcpy(ae,ends,sizeof(ends));memcpy(arr,r,sizeof(r));
    uint8_t before[4096];memcpy(before,arena,sizeof(arena));
    check(sh2z_view_merge_tier(arena+sizeof(l)-1,al,arr,NULL,N,0x16u,256)==bad,"out overlaps left");
    check(sh2z_view_merge_tier(arena+3072-(N-2),al,arr,NULL,N,0x16u,256)==bad,"out overlaps right");
    check(sh2z_view_merge_tier(arena+2048+sizeof(ends)-1,al,arr,ae,N,0x16u,256)==bad,"out overlaps end masks");
    check(!memcmp(before,arena,sizeof(arena)),"overlapping calls write nothing");
    check(sh2z_view_merge_tier(arena+2048+sizeof(ends),al,arr,ae,N,0x16u,256)>0,"adjacent buffers are fine");
    check(sh2z_view_merge_tier(arena+3072-(N-1),al,arr,ae,N,0x16u,256)>0,"out ending just before right is fine");
    check(sh2z_view_merge_tier(arena+2048+4,al,arr,NULL,N,0x16u,256)>0,"end masks not read when NULL");
    /* Smallest and largest counts. */
    check(sh2z_view_merge_tier(out,l,r,NULL,2,SH2Z_VIEW_MERGE_TIER_RUNS,0)==1 && out[0]==1,"count 2: one run");
    static sh2z_view_vertex_t bl[SH2Z_VIEW_MERGE_MAX_COUNT],br[SH2Z_VIEW_MERGE_MAX_COUNT];
    static uint8_t bout[SH2Z_VIEW_MERGE_MAX_COUNT];
    for(unsigned k=0;k<SH2Z_VIEW_MERGE_MAX_COUNT;++k){bl[k].sx=(int16_t)(k/4u);br[k].sx=(int16_t)(k/4u+50u);}
    const int nb=sh2z_view_merge_tier(bout,bl,br,NULL,SH2Z_VIEW_MERGE_MAX_COUNT,0x16u,64);
    unsigned sum=0;for(int k=0;k<nb;++k)sum+=bout[k];
    check(nb>0 && sum==SH2Z_VIEW_MERGE_MAX_COUNT-1u,"largest count");
    printf("argument rules: null pointers, count, tolerance, run lengths, overlap; nothing written on rejection\n");
}

int main(void) {
    argument_rules();
    hand_cases();
    sweep();
    caller_case();
    printf("merge-tier checks: %llu passed\n",(unsigned long long)checks);
    return EXIT_SUCCESS;
}
