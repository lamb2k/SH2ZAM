/* view-camera-1: sh2z_view_place_q12 and sh2z_view_from_camera30 against the
 * independent oracle (tests/camera_oracle.h) over every field of view,
 * random and boundary inputs, negative controls, and an accuracy study
 * against the reference float pipeline (tests/camera_float_ref.h) and the
 * exact projection. MIT; SH2ZAM Contributors. */
#include "sh2zam/view_experimental.h"
#include "camera_oracle.h"
#include "camera_float_ref.h"
#include <string.h>

static uint64_t checks;
static void check(bool value,const char *what) {
    ++checks;
    if(!value) {
        fprintf(stderr,"camera check %llu failed: %s\n",(unsigned long long)checks,what);
        exit(EXIT_FAILURE);
    }
}
static uint32_t state=UINT32_C(0x5eedca3e);
static uint32_t random_word(void) {state^=state<<13;state^=state>>17;state^=state<<5;return state;}
static int32_t random_range(int32_t lo,int32_t hi) {
    return (int32_t)((int64_t)lo+(int64_t)(random_word()%(uint64_t)((int64_t)hi-lo+1)));
}
static long double random_unit(void) {return (long double)random_word()/4294967296.0L;}

/* ---- calling both sides ------------------------------------------------------ */

/* Compare library and (possibly mutated) oracle on one input. Returns 1 when
 * they agree; a failed call must leave every output byte unchanged. */
static int agree(const int32_t basis[9],const int32_t eye[3],const sh2z_view_lens_t *lens,unsigned mutation) {
    sh2z_view_center_t got,want;
    memset(&got,0xA5,sizeof got);memset(&want,0xA5,sizeof want);
    const bool ok=sh2z_view_from_camera30(&got,basis,eye,lens);
    const int wok=co_from_camera30(&want,basis,eye,lens,mutation);
    if(ok!=(wok!=0))return 0;
    if(!ok) {
        const unsigned char *b=(const unsigned char *)&got;
        for(size_t i=0;i<sizeof got;++i)if(b[i]!=0xA5)return 0;
        return 1;
    }
    return co_same_center(&got,&want);
}

/* ---- input generators ------------------------------------------------------------ */

/* Rows {right, up, forward}: a uniform unit forward (or the given one) and a
 * random up, Gram-Schmidt in long double. */
static void random_rotation_toward(long double r[3][3],const long double *toward) {
    long double f[3],u[3],len;
    if(toward) {
        len=sqrtl(toward[0]*toward[0]+toward[1]*toward[1]+toward[2]*toward[2]);
        for(unsigned i=0;i<3;++i)f[i]=toward[i];
    } else do {
        for(unsigned i=0;i<3;++i)f[i]=random_unit()*2-1;
        len=sqrtl(f[0]*f[0]+f[1]*f[1]+f[2]*f[2]);
    } while(len<0.1L || len>1);
    for(unsigned i=0;i<3;++i)f[i]/=len;
    long double s[3];
    do {
        for(unsigned i=0;i<3;++i)u[i]=random_unit()*2-1;
        s[0]=u[1]*f[2]-u[2]*f[1];s[1]=u[2]*f[0]-u[0]*f[2];s[2]=u[0]*f[1]-u[1]*f[0];
        len=sqrtl(s[0]*s[0]+s[1]*s[1]+s[2]*s[2]);
    } while(len<0.1L);
    for(unsigned i=0;i<3;++i)s[i]/=len;
    const long double up[3]={f[1]*s[2]-f[2]*s[1],f[2]*s[0]-f[0]*s[2],f[0]*s[1]-f[1]*s[0]};
    for(unsigned i=0;i<3;++i){r[0][i]=-s[i];r[1][i]=up[i];r[2][i]=f[i];}
}
static void random_rotation(long double r[3][3]) {random_rotation_toward(r,NULL);}
static int32_t to_u30(long double v,int32_t jitter) {
    return (int32_t)(llroundl(v*1073741824.0L)+jitter);
}
static int32_t special_word(void) {
    static const int32_t k[]={0,1,-1,32767,32768,-32768,-32769,65535,65536,-65536,
        INT32_MAX,INT32_MIN,INT32_MAX-32767,INT32_MAX-32768,INT32_MAX-32769,
        INT32_MIN+32767,INT32_MIN+32768,1073741824,-1073741824,
        5759,5760,5761,11790719,11790720,536870911,536870912,
        (int32_t)UINT32_C(0x80008000),0x7fff8000,0x7fff7fff};
    return k[random_word()%(sizeof k/sizeof k[0])];
}
static int32_t random_word_any(void) {
    switch(random_word()%6u) {
    case 0: return (int32_t)random_word();
    case 1: return special_word();
    case 2: return random_range(-65536*3,65536*3);
    case 3: return random_range(-1,1)*65536*random_range(0,32768)+random_range(-32769,32769);
    default: return random_range(-(1<<30),1<<30);
    }
}
static void random_basis(int32_t b[9]) {
    if(random_word()%3u) {
        long double r[3][3];random_rotation(r);
        const int32_t j=random_word()%2u?(int32_t)random_range(-8,8):0;
        for(unsigned i=0;i<3;++i)for(unsigned k=0;k<3;++k)b[3*i+k]=to_u30(r[i][k],j?random_range(-j,j):0);
        if(random_word()%8u==0)b[random_word()%9u]=random_word_any();
    } else for(unsigned i=0;i<9;++i)b[i]=random_word_any();
}
static void random_eye(int32_t e[3]) {
    for(unsigned i=0;i<3;++i) switch(random_word()%5u) {
        case 0: e[i]=random_range(-32767*4096,32767*4096);break;
        case 1: e[i]=random_range(-75000*4096,75000*4096);break;
        case 2: e[i]=(int32_t)random_word();break;
        case 3: e[i]=special_word();break;
        default: e[i]=random_range(-4096*64,4096*64);break;
    }
}
static int32_t random_fov(void) {
    switch(random_word()%6u) {
    case 0: return special_word();
    case 1: return random_range(5700,11790800);
    case 2: return random_range(1,1023)*11520-5760+random_range(-2,2);
    default: return random_range(30*65536,120*65536);
    }
}
static int32_t random_scale(void) {
    switch(random_word()%6u) {
    case 0: return special_word();
    case 1: return random_range(-3,3);
    case 2: return (int32_t)random_word();
    default: return random_range(65536,320*65536);
    }
}
static void random_lens(sh2z_view_lens_t *l) {
    l->fov_q16=random_fov();
    l->scale_x16=random_scale();
    l->scale_y16=random_word()%2u?l->scale_x16:random_scale();
    l->center_x16=random_word()%4u?random_range(0,320*65536):random_word_any();
    l->center_y16=random_word()%4u?random_range(0,224*65536):random_word_any();
    switch(random_word()%5u) {
    case 0: l->near_q12=random_word_any();l->far_q12=random_word_any();break;
    case 1: l->near_q12=random_range(-2,2);l->far_q12=l->near_q12+random_range(-1,1);break;
    case 2: l->near_q12=random_range(1,1000);l->far_q12=INT32_MAX/4+random_range(-1,1);break;
    default: l->near_q12=16*4096;l->far_q12=random_word()%2u?8192*4096:16384*4096;break;
    }
    for(unsigned i=0;i<4;++i)l->clip[i]=(int16_t)random_range(-40,360);
    if(random_word()%4u) {l->clip[0]=12;l->clip[1]=8;l->clip[2]=307;l->clip[3]=215;}
    if(random_word()%16u==0)l->clip[2]=l->clip[0];
    if(random_word()%16u==0)l->clip[3]=(int16_t)(l->clip[1]-1);
}
static void race_lens(sh2z_view_lens_t *l) {
    l->fov_q16=80*65536;l->scale_x16=160*65536;l->scale_y16=160*65536;
    l->center_x16=160*65536;l->center_y16=97*65536;
    l->near_q12=16*4096;l->far_q12=8192*4096;
    l->clip[0]=12;l->clip[1]=8;l->clip[2]=307;l->clip[3]=215;
}

/* ---- 1. table and every field of view ---------------------------------------------- */

static uint64_t table_and_fov(unsigned mutation) {
    uint64_t bad=0;
    static const int32_t basis[9]={1073741824,0,0,0,1073741824,0,0,0,1073741824};
    const int32_t eye[3]={0,0,0};
    sh2z_view_lens_t l;race_lens(&l);
    l.scale_x16=1<<20;l.scale_y16=1<<20;  /* focal = C[h] exactly */
    if(!mutation) for(unsigned h=1;h<1024;++h) {
        l.fov_q16=(int32_t)(h*11520u);
        sh2z_view_center_t v;
        check(sh2z_view_from_camera30(&v,basis,eye,&l),"table entry call");
        check(v.view.focal_x==co_cot_q20(h) && v.view.focal_y==co_cot_q20(h),"table entry");
        const long double exact=cosl(3.141592653589793238462643383279502884L*(long double)h/2048.0L)/
            sinl(3.141592653589793238462643383279502884L*(long double)h/2048.0L)*1048576.0L;
        check(fabsl((long double)v.view.focal_x-exact)<=0.5L,"table within half an LSB");
    }
    /* every fov from below the range to above it */
    for(int32_t fov=5000;fov<=11791000;fov+=(mutation?97:1)) {
        l.fov_q16=fov;
        if(!agree(basis,eye,&l,mutation))++bad;
    }
    return bad;
}

/* ---- 2. random inputs ---------------------------------------------------------------- */

static uint64_t random_cases(uint64_t n,unsigned mutation) {
    uint64_t bad=0;
    for(uint64_t it=0;it<n;++it) {
        int32_t b[9],e[3];sh2z_view_lens_t l;
        random_basis(b);random_eye(e);
        if(random_word()%3u)random_lens(&l);
        else {race_lens(&l);l.fov_q16=random_range(30*65536,120*65536);
              l.center_y16=random_range(17*65536,192*65536);}
        if(!agree(b,e,&l,mutation))++bad;
    }
    return bad;
}

/* ---- 3. boundaries --------------------------------------------------------------- */

/* Row 0 = (r0, 1, 0) and eye chosen so that floor(r . eye / 4096) = -target
 * + k for small k: t lands exactly on and around the row bound. */
static uint64_t row_bound_cases(unsigned mutation) {
    uint64_t bad=0;
    sh2z_view_lens_t l;race_lens(&l);
    for(unsigned it=0;it<4000;++it) {
        const int32_t r0=random_range(1,32767)*(random_word()%2u?1:-1);
        const uint32_t sum=(uint32_t)(r0<0?-r0:r0)+1u;
        const int64_t limit=(int64_t)INT32_MAX-(int64_t)sum*32768;
        const int64_t target=(limit+random_range(-2,2))*(random_word()%2u?1:-1);   /* desired t */
        /* t = -floor(s / 4096) = target  <=>  s in [-target*4096, -target*4096 + 4095] */
        const int64_t s=-target*4096+random_range(0,4095);
        const int64_t e0=s/r0;
        const int64_t e1=s-(int64_t)r0*e0;
        if(e0<INT32_MIN || e0>INT32_MAX || e1<INT32_MIN || e1>INT32_MAX)continue;
        const int32_t b[9]={r0*65536,65536,0,0,0,1073741824,0,0,1073741824};
        const int32_t e[3]={(int32_t)e0,(int32_t)e1,0};
        if(!agree(b,e,&l,mutation))++bad;
    }
    /* t = INT32_MIN (q = 2^31) and its neighbours */
    for(int32_t d=-3;d<=3;++d) {
        static const int32_t b[9]={1073741824,0,0,0,1073741824,0,0,0,1073741824};
        const int32_t e[3]={-536870912+d,0,0},f[3]={536870911+d,0,0};
        if(!agree(b,e,&l,mutation))++bad;
        if(!agree(b,f,&l,mutation))++bad;
    }
    return bad;
}
/* Scales around the largest focal (INT32_MAX) and the smallest (1). */
static uint64_t focal_bound_cases(unsigned mutation) {
    uint64_t bad=0;
    static const int32_t b[9]={1073741824,0,0,0,1073741824,0,0,0,1073741824};
    const int32_t e[3]={0,0,0};
    sh2z_view_lens_t l;race_lens(&l);
    for(unsigned h=1;h<1024;h+=(mutation?37u:1u)) {
        const int64_t c=co_cot_q20(h);
        l.fov_q16=(int32_t)(h*11520u);
        /* largest: (scale * c + 2^19) >> 20 <= INT32_MAX */
        const co_wide_t top=((co_wide_t)INT32_MAX+1)*1048576-524288;   /* scale * c < top */
        const co_wide_t smax=(top-1)/c;
        const co_wide_t smin=(524288+c-1)/c;                             /* scale * c >= 2^19 */
        for(int d=-2;d<=2;++d) {
            if(smax+d>0 && smax+d<=INT32_MAX) {
                l.scale_x16=(int32_t)(smax+d);l.scale_y16=65536;
                if(!agree(b,e,&l,mutation))++bad;
                l.scale_y16=(int32_t)(smax+d);l.scale_x16=65536;
                if(!agree(b,e,&l,mutation))++bad;
            }
            if(smin+d>0) {
                l.scale_x16=(int32_t)(smin+d);l.scale_y16=(int32_t)(smin+d);
                if(!agree(b,e,&l,mutation))++bad;
            }
        }
    }
    return bad;
}
/* Rotation rounding boundaries: b = k * 65536 +- 32768 +- 1. */
static uint64_t rounding_cases(unsigned mutation) {
    uint64_t bad=0;
    sh2z_view_lens_t l;race_lens(&l);
    const int32_t e[3]={4096*100,-4096*7,4096*3+5};
    for(unsigned it=0;it<20000;++it) {
        int32_t b[9]={1073741824,0,0,0,1073741824,0,0,0,1073741824};
        const int64_t k=random_range(-32768,32767);
        const int64_t v=k*65536+(random_word()%2u?32768:-32768)+random_range(-1,1);
        if(v<INT32_MIN || v>INT32_MAX)continue;
        b[random_word()%9u]=(int32_t)v;
        if(!agree(b,e,&l,mutation))++bad;
    }
    return bad;
}
/* Negative fractional centres (floor, not truncation) and centre extremes. */
static uint64_t centre_cases(unsigned mutation) {
    uint64_t bad=0;
    static const int32_t b[9]={1073741824,0,0,0,1073741824,0,0,0,1073741824};
    const int32_t e[3]={0,0,0};
    sh2z_view_lens_t l;race_lens(&l);
    for(unsigned it=0;it<4000;++it) {
        l.center_x16=it<8?special_word():random_range(-65536*40,65536*40);
        l.center_y16=random_word_any();
        if(!agree(b,e,&l,mutation))++bad;
    }
    return bad;
}
/* Depth and clip rules at their edges. */
static uint64_t depth_clip_cases(unsigned mutation) {
    uint64_t bad=0;
    static const int32_t b[9]={1073741824,0,0,0,1073741824,0,0,0,1073741824};
    const int32_t e[3]={4096,8192,-4096};
    sh2z_view_lens_t l;race_lens(&l);
    const int32_t nears[]={INT32_MIN,-1,0,1,2,16*4096,INT32_MAX/4-1,INT32_MAX/4,INT32_MAX/4+1};
    for(unsigned i=0;i<9;++i)for(unsigned j=0;j<9;++j) {
        l.near_q12=nears[i];l.far_q12=nears[j];
        if(!agree(b,e,&l,mutation))++bad;
    }
    race_lens(&l);
    for(int a=-1;a<=1;++a)for(int c=-1;c<=1;++c) {
        l.clip[0]=(int16_t)(100+a);l.clip[2]=100;l.clip[1]=(int16_t)(50+c);l.clip[3]=50;
        if(!agree(b,e,&l,mutation))++bad;
    }
    l.clip[0]=INT16_MIN;l.clip[1]=INT16_MIN;l.clip[2]=INT16_MAX;l.clip[3]=INT16_MAX;
    if(!agree(b,e,&l,mutation))++bad;
    return bad;
}
static uint64_t all_cases(uint64_t random_count,unsigned mutation) {
    return table_and_fov(mutation)+random_cases(random_count,mutation)+row_bound_cases(mutation)+
        focal_bound_cases(mutation)+rounding_cases(mutation)+centre_cases(mutation)+depth_clip_cases(mutation);
}

/* ---- 4. sh2z_view_place_q12 ----------------------------------------------------------- */

static void place_cases(uint64_t n) {
    uint64_t equivalent=0;
    for(uint64_t it=0;it<n;++it) {
        int32_t b[9],e[3];sh2z_view_lens_t l;
        random_basis(b);race_lens(&l);
        sh2z_view_center_t c;
        if(!co_from_camera30(&c,b,(const int32_t[3]){0,0,0},&l,0)) {
            /* an arbitrary view, valid or not */
            for(unsigned i=0;i<3;++i)for(unsigned j=0;j<3;++j)c.view.r[i][j]=(int16_t)random_word();
            c.view.focal_x=random_range(-1,3);c.view.focal_y=random_range(-1,3);
            c.view.near_z=random_range(-1,3);c.view.far_z=random_range(-1,5);
            for(unsigned i=0;i<4;++i)c.view.clip[i]=(int16_t)random_range(-2,2);
        }
        for(unsigned i=0;i<3;++i)c.view.t[i]=random_word()%2u?0:random_word_any();
        random_eye(e);
        sh2z_view_t got=c.view,want=c.view;
        const bool ok=sh2z_view_place_q12(&got,e);
        const int wok=co_place_q12(&want,e,0);
        check(ok==(wok!=0),"place_q12 result");
        check(co_same_view(&got,&want),"place_q12 view (or unchanged)");
        bool fits=true;
        for(unsigned i=0;i<3;++i)fits=fits && e[i]>=INT32_MIN/16 && e[i]<=INT32_MAX/16;
        if(fits) {
            sh2z_view_t q16=c.view;
            const sh2z_vec3_t eye16={e[0]*16,e[1]*16,e[2]*16};
            const bool ok16=sh2z_view_place(&q16,eye16);
            check(ok16==ok && co_same_view(&q16,&got),"place_q12 equals place with 16 * eye");
            ++equivalent;
        }
    }
    sh2z_view_t v;
    check(!sh2z_view_place_q12(NULL,(const int32_t[3]){0,0,0}),"place_q12 null view");
    memset(&v,0,sizeof v);
    check(!sh2z_view_place_q12(&v,NULL),"place_q12 null eye");
    printf("place_q12: %llu random views, %llu equal to sh2z_view_place(16 * eye)\n",
        (unsigned long long)n,(unsigned long long)equivalent);
}
static void null_cases(void) {
    static const int32_t b[9]={1073741824,0,0,0,1073741824,0,0,0,1073741824};
    const int32_t e[3]={0,0,0};
    sh2z_view_lens_t l;race_lens(&l);
    sh2z_view_center_t c;
    check(!sh2z_view_from_camera30(NULL,b,e,&l),"null out");
    check(!sh2z_view_from_camera30(&c,NULL,e,&l),"null basis");
    check(!sh2z_view_from_camera30(&c,b,NULL,&l),"null eye");
    check(!sh2z_view_from_camera30(&c,b,e,NULL),"null lens");
    check(sh2z_view_from_camera30(&c,b,e,&l),"identity camera");
    /* It is a valid centre view: the view-center-1 batch accepts it. */
    const sh2z_pos16_t p={0,0,100};
    sh2z_view_vertex_t o;
    check(sh2z_view_batch_center(&o,&c,&p,sizeof p,1),"batch accepts the view");
    check(o.z==100*16384 && o.sx==160 && o.sy==97,"axis point projects to the centre");
}

/* ---- 5. accuracy against the float pipeline and the exact projection --------------- */

typedef struct { double *v; size_t n,cap; } series_t;
static void push(series_t *s,double x) {
    if(s->n==s->cap) {s->cap=s->cap?s->cap*2:4096;s->v=realloc(s->v,s->cap*sizeof *s->v);if(!s->v)exit(2);}
    s->v[s->n++]=x;
}
static int cmp_double(const void *a,const void *b) {
    const double x=*(const double *)a,y=*(const double *)b;
    return x<y?-1:x>y;
}
static double quantile(series_t *s,double q) {
    if(!s->n)return 0;
    qsort(s->v,s->n,sizeof *s->v,cmp_double);
    size_t i=(size_t)(q*(double)(s->n-1)+0.5);
    return s->v[i];
}
static double maxof(const series_t *s) {double m=0;for(size_t i=0;i<s->n;++i)if(s->v[i]>m)m=s->v[i];return m;}

typedef struct {
    const char *name;
    long double eye_lo,eye_hi;    /* |eye coordinate| range, units */
    unsigned poses;
    bool outside;                 /* one coordinate in the range, the others in the world; aimed at the world */
} pose_class_t;

typedef struct {
    series_t mine,port,diff;      /* continuous projected error, px (max of x, y) */
    uint64_t points,int_diff,int_diff_max,mine_int_wrong,port_int_wrong;
    uint64_t poses,port_failed,mine_failed,points_bound;
    double r_lsb_port,r_lsb_exact,t_lsb_port,focal_lsb_port,center_lsb_port,focal_px_exact;
    double worst_ratio;           /* largest (mine error) / (per-point bound) */
} study_t;

static float sin_table[4096];

static long double dot3(const long double a[3],const long double b[3]) {return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];}

/* One pose: compare both views over points spread in the guard-extended screen. */
static void study_pose(study_t *st,const pose_class_t *pc,bool jitter) {
    /* camera */
    int32_t eye[3];
    const unsigned axis=random_word()%3u;
    for(unsigned i=0;i<3;++i) {
        const long double m=(pc->outside && i!=axis)?32767.0L*random_unit():pc->eye_lo+(pc->eye_hi-pc->eye_lo)*random_unit();
        eye[i]=(int32_t)llroundl((random_word()%2u?m:-m)*4096.0L);
    }
    const long double ex[3]={eye[0]/4096.0L,eye[1]/4096.0L,eye[2]/4096.0L};
    long double rot[3][3];
    if(pc->outside) {
        long double toward[3];
        for(unsigned i=0;i<3;++i)toward[i]=(random_unit()*2-1)*20000.0L-ex[i];
        random_rotation_toward(rot,toward);
    } else random_rotation(rot);
    const long double dist=50.0L+1950.0L*random_unit();
    int32_t at[3];
    for(unsigned i=0;i<3;++i)at[i]=(int32_t)llroundl((ex[i]+rot[2][i]*dist)*4096.0L);
    /* the camera's up hint: the true up, tilted a little */
    float upf[3];
    for(unsigned i=0;i<3;++i)upf[i]=(float)(rot[1][i]+(random_unit()-0.5L)*0.2L);
    /* exact basis from (at - eye, up hint), as the fixed-point camera would hold it */
    long double fw[3],len=0;
    for(unsigned i=0;i<3;++i){fw[i]=(at[i]-eye[i])/4096.0L;len+=fw[i]*fw[i];}
    len=sqrtl(len);for(unsigned i=0;i<3;++i)fw[i]/=len;
    long double sd[3]={upf[1]*fw[2]-upf[2]*fw[1],upf[2]*fw[0]-upf[0]*fw[2],upf[0]*fw[1]-upf[1]*fw[0]};
    len=sqrtl(dot3(sd,sd));for(unsigned i=0;i<3;++i)sd[i]/=len;
    const long double un[3]={fw[1]*sd[2]-fw[2]*sd[1],fw[2]*sd[0]-fw[0]*sd[2],fw[0]*sd[1]-fw[1]*sd[0]};
    const long double R[3][3]={{-sd[0],-sd[1],-sd[2]},{un[0],un[1],un[2]},{fw[0],fw[1],fw[2]}};
    int32_t basis[9];
    for(unsigned i=0;i<3;++i)for(unsigned j=0;j<3;++j)basis[3*i+j]=to_u30(R[i][j],jitter?random_range(-8,8):0);
    /* lens */
    const int32_t fov=random_word()%4u?random_range(40*65536,110*65536):(int32_t[]){60*65536,80*65536,85*65536,100*65536}[random_word()%4u];
    const int32_t fcx=random_word()%2u?random_range(-80*65536,80*65536):0;
    const int32_t fcy=random_word()%2u?random_range(-80*65536,80*65536):random_range(-80,80)*65536;
    const bool deep=pc->outside || random_word()%4u==0;
    const float near_f=deep?512.0f:16.0f,far_f=deep?16384.0f:8192.0f;
    sh2z_view_lens_t lens={fov,160*65536,160*65536,160*65536-fcx,112*65536+fcy,
        (int32_t)(near_f*4096),(int32_t)(far_f*4096),{12,8,307,215}};
    ++st->poses;
    sh2z_view_center_t mine,port;
    if(!sh2z_view_from_camera30(&mine,basis,eye,&lens)) {++st->mine_failed;return;}
    /* float chain */
    cf_camera_t cam;
    for(unsigned i=0;i<3;++i){cam.eye[i]=(float)(eye[i]/4096.0L);cam.at[i]=(float)(at[i]/4096.0L);cam.up[i]=upf[i];}
    cam.fov=(float)fov/65536.0f;cam.near_plane=near_f;cam.far_plane=far_f;
    cam.fov_scale_x=320.0f;cam.fov_scale_y=240.0f;cam.frustum_x=(float)fcx/65536.0f;cam.frustum_y=(float)fcy/65536.0f;
    cam.viewport[0]=160.0f;cam.viewport[1]=120.0f;cam.viewport[2]=160.0f;cam.viewport[3]=120.0f;
    cam.scissor[0]=12.0f;cam.scissor[1]=16.0f;cam.scissor[2]=308.0f;cam.scissor[3]=224.0f;
    cf_frame_t fr;memset(&fr,0,sizeof fr);
    for(unsigned i=0;i<3;++i){fr.basis[0][i]=cam.at[i]-cam.eye[i];fr.basis[1][i]=cam.up[i];fr.basis[2][i]=0.0f;}
    check(cf_orthonormalize_forward(fr.basis)==0,"float basis");
    const bool port_ok=cf_chain(&port,&fr,&cam,sin_table)!=0;
    if(!port_ok)++st->port_failed;
    /* exact lens */
    const unsigned h=co_half_angle(fov,0);
    const long double pi=3.141592653589793238462643383279502884L;
    const long double cot=cosl(pi*h/2048.0L)/sinl(pi*h/2048.0L);
    const long double fx=160.0L*cot,fy=160.0L*cot;
    const long double cx=lens.center_x16/65536.0L,cy=lens.center_y16/65536.0L;
    /* matrix elements */
    for(unsigned i=0;i<3;++i)for(unsigned j=0;j<3;++j) {
        const double d=fabs((double)(mine.view.r[i][j]-R[i][j]*16384.0L));
        if(d>st->r_lsb_exact)st->r_lsb_exact=d;
        if(port_ok) {
            const double q=fabs((double)(mine.view.r[i][j]-port.view.r[i][j]));
            if(q>st->r_lsb_port)st->r_lsb_port=q;
        }
    }
    {
        const double fe=fabs((double)(mine.view.focal_x/65536.0L-fx));
        if(fe>st->focal_px_exact)st->focal_px_exact=fe;
    }
    if(port_ok) {
        for(unsigned i=0;i<3;++i) {
            const double d=fabs((double)mine.view.t[i]-(double)port.view.t[i]);
            if(d>st->t_lsb_port)st->t_lsb_port=d;
        }
        const double f=fmax(fabs((double)mine.view.focal_x-(double)port.view.focal_x),fabs((double)mine.view.focal_y-(double)port.view.focal_y));
        if(f>st->focal_lsb_port)st->focal_lsb_port=f;
        const double c=fmax(fabs((double)mine.center_x16-(double)port.center_x16),fabs((double)mine.center_y16-(double)port.center_y16));
        if(c>st->center_lsb_port)st->center_lsb_port=c;
    }
    /* points */
    enum { POINTS=96 };
    sh2z_pos16_t pos[POINTS];long double ideal[POINTS][2];long double zs[POINTS],ks[POINTS];
    unsigned n=0;
    for(unsigned tries=0;tries<POINTS*4 && n<POINTS;++tries) {
        const long double u=-4.0L+332.0L*random_unit(),w=-8.0L+240.0L*random_unit();
        const long double z=near_f*expl(logl(far_f/near_f)*random_unit());
        const long double vx=(u-cx)*z/fx,vy=(cy-w)*z/fy;
        long double p[3];bool inside=true;
        for(unsigned i=0;i<3;++i) {
            p[i]=roundl(ex[i]+vx*R[0][i]+vy*R[1][i]+z*R[2][i]);
            inside=inside && p[i]>=-32768.0L && p[i]<=32767.0L;
        }
        if(!inside)continue;
        const long double d[3]={p[0]-ex[0],p[1]-ex[1],p[2]-ex[2]};
        const long double qx=dot3(R[0],d),qy=dot3(R[1],d),qz=dot3(R[2],d);
        if(qz<near_f+1 || qz>far_f-1)continue;
        pos[n].x=(int16_t)p[0];pos[n].y=(int16_t)p[1];pos[n].z=(int16_t)p[2];
        ideal[n][0]=cx+fx*qx/qz;ideal[n][1]=cy-fy*qy/qz;
        zs[n]=qz;ks[n]=sqrtl(dot3(d,d))/qz;
        ++n;
    }
    sh2z_view_vertex_t vm[POINTS],vp[POINTS];
    if(!n)return;
    check(sh2z_view_batch_center(vm,&mine,pos,sizeof pos[0],n),"batch mine");
    if(port_ok)check(sh2z_view_batch_center(vp,&port,pos,sizeof pos[0],n),"batch port");
    const long double J=8.5L/1073741824.0L;
    for(unsigned k=0;k<n;++k) {
        const long double mx=(mine.center_x16+mine.view.focal_x*(long double)vm[k].x/vm[k].z)/65536.0L;
        const long double my=(mine.center_y16-mine.view.focal_y*(long double)vm[k].y/vm[k].z)/65536.0L;
        const long double em=fmaxl(fabsl(mx-ideal[k][0]),fabsl(my-ideal[k][1]));
        push(&st->mine,(double)em);
        /* per-point bound (VIEW_BATCH.md): (|X - c| + f)(3 (2^-15 + J) k + 2^-14 / z) + |X - c| df / f */
        const long double rot_err=3.0L*(1.0L/32768.0L+J)*ks[k]+1.0L/(16384.0L*zs[k]);
        const long double df=fabsl(mine.view.focal_x/65536.0L-fx)+1e-12L;
        const long double ax=fabsl(ideal[k][0]-cx),ay=fabsl(ideal[k][1]-cy);
        const long double bx=(ax+fx)*rot_err+ax*df/fx,by=(ay+fy)*rot_err+ay*df/fy;
        const long double bound=1.001L*fmaxl(bx,by)+1e-9L;
        const double ratio=(double)(em/bound);
        if(ratio>st->worst_ratio)st->worst_ratio=ratio;
        check(em<=bound,"projected point within the stated bound");
        ++st->points_bound;
        /* integer pixels: the truncating rule of view-center-1 applied to the exact point */
        const int32_t ix=(int32_t)truncl(ideal[k][0]),iy=(int32_t)truncl(ideal[k][1]);
        if(vm[k].sx!=ix || vm[k].sy!=iy)++st->mine_int_wrong;
        if(port_ok) {
            const long double px=(port.center_x16+port.view.focal_x*(long double)vp[k].x/vp[k].z)/65536.0L;
            const long double py=(port.center_y16-port.view.focal_y*(long double)vp[k].y/vp[k].z)/65536.0L;
            push(&st->port,(double)fmaxl(fabsl(px-ideal[k][0]),fabsl(py-ideal[k][1])));
            push(&st->diff,(double)fmaxl(fabsl(px-mx),fabsl(py-my)));
            const uint64_t dx=(uint64_t)llabs((long long)vm[k].sx-vp[k].sx),dy=(uint64_t)llabs((long long)vm[k].sy-vp[k].sy);
            if(dx||dy)++st->int_diff;
            if(dx>st->int_diff_max)st->int_diff_max=dx;
            if(dy>st->int_diff_max)st->int_diff_max=dy;
            if(vp[k].sx!=ix || vp[k].sy!=iy)++st->port_int_wrong;
        }
        ++st->points;
    }
}

static void accuracy_study(void) {
    cf_sin_table_init(sin_table);
    /* the correctly rounded square root of the reference matches libm */
    for(unsigned i=0;i<2000000;++i) {
        float x=cf_float(random_word()&0x7fffffffu);
        if(!cf_finite(x))continue;
        check(cf_bits(cf_sqrtf(x))==cf_bits(sqrtf(x)),"reference sqrtf is correctly rounded");
    }
    static const pose_class_t classes[]={
        {"near origin |e| <= 2048",0,2048,3000,false},
        {"world |e| <= 32767",0,32767,6000,false},
        {"world edge 30000..32767",30000,32767,3000,false},
        {"outside 32768..42000",32768,42000,3000,true},
    };
    printf("accuracy: projected error in pixels (max of x, y), continuous before the final truncation\n");
    printf("%-26s %6s %8s | %-27s | %-27s | %-27s | %s\n","class","poses","points",
        "mine vs exact p50/p99/max","float vs exact p50/p99/max","mine vs float p50/p99/max","integer px: mine!=float (max), mine!=exact, float!=exact");
    double worst_mine=0,worst_diff=0;
    for(size_t c=0;c<sizeof classes/sizeof classes[0];++c) {
        study_t st;memset(&st,0,sizeof st);
        for(unsigned p=0;p<classes[c].poses;++p)study_pose(&st,&classes[c],(p&1u)!=0);
        const double mm=maxof(&st.mine),dm=maxof(&st.diff),pm=maxof(&st.port);
        printf("%-26s %6llu %8llu | %8.5f %8.5f %8.5f | %8.5f %8.5f %8.5f | %8.5f %8.5f %8.5f | %llu (%llu), %llu, %llu\n",
            classes[c].name,(unsigned long long)st.poses,(unsigned long long)st.points,
            quantile(&st.mine,0.5),quantile(&st.mine,0.99),mm,
            quantile(&st.port,0.5),quantile(&st.port,0.99),pm,
            quantile(&st.diff,0.5),quantile(&st.diff,0.99),dm,
            (unsigned long long)st.int_diff,(unsigned long long)st.int_diff_max,
            (unsigned long long)st.mine_int_wrong,(unsigned long long)st.port_int_wrong);
        printf("  elements: r vs exact %.4f LSB (Q1.14), r vs float %.0f LSB, t vs float %.0f LSB (Q18.14), focal vs float %.0f LSB, "
               "centre vs float %.0f LSB (Q16), focal vs exact %.2e px; float chain failed on %llu poses, mine on %llu; "
               "largest error/bound %.3f over %llu points\n",
            st.r_lsb_exact,st.r_lsb_port,st.t_lsb_port,st.focal_lsb_port,st.center_lsb_port,st.focal_px_exact,
            (unsigned long long)st.port_failed,(unsigned long long)st.mine_failed,st.worst_ratio,(unsigned long long)st.points_bound);
        check(st.mine_failed==0,"no pose fails inside |eye| <= 74,000");
        check(st.int_diff_max<=1,"integer pixels differ from the float pipeline by at most one");
        if(mm>worst_mine)worst_mine=mm;
        if(dm>worst_diff)worst_diff=dm;
        if(classes[c].eye_lo>=32768)check(st.port_failed==st.poses,"the Q16.16 eye overflows beyond 32767");
        else check(st.port_failed==0,"float chain defined inside the world");
        free(st.mine.v);free(st.port.v);free(st.diff.v);
    }
    check(worst_mine<=0.05,"mine within 0.05 px of the exact projection");
    check(worst_diff<=0.06,"mine within 0.06 px of the float pipeline");
}

int main(void) {
    const uint64_t random_count=600000;
    check(all_cases(random_count,0)==0,"library equals the oracle");
    null_cases();
    place_cases(300000);
    static const struct { unsigned m; const char *name; } controls[]={
        {CO_MUT_R_FLOOR,"rotation by floor"},{CO_MUT_T_TRUNC,"t by truncation"},
        {CO_MUT_H_TRUNC,"half angle truncated"},{CO_MUT_FOCAL_TRUNC,"focal truncated"},
        {CO_MUT_COT_NEXT,"next table entry"},{CO_MUT_CENTER_TRUNC,"centre floor by truncation"},
        {CO_MUT_ROW_STRICT,"strict row bound"},{CO_MUT_DEPTH_Q16,"depth scaled as Q16"},
    };
    for(size_t c=0;c<sizeof controls/sizeof controls[0];++c) {
        state=UINT32_C(0x0c0ffee5)+(uint32_t)c;
        const uint64_t bad=all_cases(20000,controls[c].m);
        printf("negative control %-28s %llu mismatches\n",controls[c].name,(unsigned long long)bad);
        check(bad>0,"negative control detected");
    }
    state=UINT32_C(0x5eedacc0);
    accuracy_study();
    printf("camera: %llu checks passed\n",(unsigned long long)checks);
    return 0;
}
