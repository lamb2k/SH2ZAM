/* Host evidence for fixed-unit-1 (unit30_experimental.h): every result
 * against the independent oracle in unit30_oracle.h (128-bit integers for the
 * exact rules, float64 for the approximate kernels), edge and random corpora,
 * the stated error bounds, the overflow counter, and mutation controls that
 * the same checks must reject. Set SH2Z_UNIT30_EXHAUSTIVE=1 to check the
 * reciprocal-square-root mantissa for every m in [2^30, 2^32).
 * MIT; SH2ZAM Contributors. */
#define SH2Z_U30_COUNT_OVERFLOWS 1
#include "unit30_oracle.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static uint64_t checks;
static void check(bool value,const char *what) {
    ++checks;
    if(!value) {
        fprintf(stderr,"unit30 check %llu failed: %s\n",(unsigned long long)checks,what);
        exit(EXIT_FAILURE);
    }
}
static uint64_t state=UINT64_C(0x0123456789abcdef);
static uint32_t rnd(void) {
    state^=state<<13;state^=state>>7;state^=state<<17;
    return (uint32_t)(state>>16);
}
static uint64_t rnd64(void) { return (uint64_t)rnd()<<32|rnd(); }
static double rnd_unit(void) { return ((double)rnd()+0.5)/4294967296.0; }
static int32_t s32(uint32_t u) { int32_t v; memcpy(&v,&u,4); return v; }

static const int32_t edges[]={0,1,-1,2,-2,0x1fffffff,0x20000000,-0x20000000,0x3fffffff,0x40000000,
    -0x40000000,-0x3fffffff,0x40000001,-0x40000001,0x5a827999,-0x5a827999,0x7fffffff,-0x7fffffff,
    INT32_MIN,0x7ffffffe,4096,-4096,12345,-987654};
enum { NEDGE=(int)(sizeof(edges)/sizeof(edges[0])) };
/* Words drawn from several scales, with edges mixed in. */
static int32_t rand_word(void) {
    switch(rnd()%8u) {
    case 0: return edges[rnd()%NEDGE];
    case 1: return s32(rnd());
    case 2: return s32(rnd())>>(rnd()%31u);
    case 3: return (int32_t)(rnd()%0x80000001u)-0x40000000;          /* [-1, 1] */
    case 4: { const int32_t w=s32((rnd()&0x3fffffffu)|0x40000000u); return rnd()%2u ? -w : w; } /* |v| in [1, 2) */
    default: return s32(rnd())>>(1+rnd()%6u);
    }
}
static void rand_vec(int32_t v[3]) { for(int k=0;k<3;k++) v[k]=rand_word(); }
/* A direction scaled to length `len` (Q2.30 units). */
static void rand_dir(int32_t v[3],double len) {
    double d[3],n;
    do { for(int k=0;k<3;k++) d[k]=2.0*rnd_unit()-1.0; n=sqrt(d[0]*d[0]+d[1]*d[1]+d[2]*d[2]); } while(n<0.05||n>1.0);
    for(int k=0;k<3;k++) v[k]=(int32_t)lrint(ldexp(d[k]/n*len,30));
}
/* A vector of any scale for normalisation. */
static void rand_any(int32_t v[3]) {
    const unsigned kind=rnd()%6u;
    if(kind==0) { rand_vec(v); return; }
    const int bits=1+(int)(rnd()%31u);
    for(int k=0;k<3;k++) {
        int32_t w=s32(rnd())>>(32-bits);
        if(kind==1 && rnd()%3u==0u) w=0;
        if(kind==2 && rnd()%4u==0u) w=w<0 ? -(1<<(bits-1)) : (1<<(bits-1))-1;
        v[k]=w;
    }
    if(kind==3) v[rnd()%3u]=rnd()%2u ? INT32_MIN : INT32_MAX;
    if(kind==4) { v[0]=v[1]=v[2]; if(rnd()%2u) v[1]=-v[1]; }
}

/* ---- K1: exact rules --------------------------------------------------------- */
static uint64_t saturations;
static void k1_one(int32_t a,int32_t b) {
    const int32_t m=sh2z_u30_mul(a,b);
    check(m==uo_mul(a,b),"u30_mul equals the oracle");
    check(sh2z_q12_mul_u30(a,b)==m,"q12_mul_u30 equals the oracle");
    const uo_i128 p=(uo_i128)a*b;
    if(uo_saturates(p)) saturations+=2;
    else {
        const uo_i128 d=(uo_i128)m*((uo_i128)1<<30)-p;
        check(d>-((uo_i128)1<<29) && d<=((uo_i128)1<<29),"u30_mul within half an LSB");
    }
}
static void k1_vec(const int32_t a[3],const int32_t b[3]) {
    const int32_t d=sh2z_u30_dot3(a,b);
    check(d==uo_dot3(a,b),"u30_dot3 equals the oracle");
    check(sh2zi_u30_dot3_c(a,b)==d,"portable dot3 equals dot3");
    if(uo_saturates(uo_dot_exact(a,b))) saturations+=2;             /* dot3 and the portable form */
    int32_t c[3],w[3],t[3];
    sh2z_u30_cross(c,a,b);
    uo_cross(w,a,b);
    check(!memcmp(c,w,sizeof c),"u30_cross equals the oracle");
    uo_i128 e[3]; uo_cross_exact(e,a,b);
    for(int k=0;k<3;k++) if(uo_saturates(e[k])) saturations+=3;     /* three cross calls below */
    memcpy(t,a,sizeof t); sh2z_u30_cross(t,t,b); check(!memcmp(t,w,sizeof t),"cross out aliasing a");
    memcpy(t,b,sizeof t); sh2z_u30_cross(t,a,t); check(!memcmp(t,w,sizeof t),"cross out aliasing b");
}
static void test_k1(void) {
    const uint32_t before=sh2z_u30_overflows;
    saturations=0;
    for(int i=0;i<NEDGE;i++) for(int j=0;j<NEDGE;j++) k1_one(edges[i],edges[j]);
    for(int n=0;n<2000000;n++) k1_one(rand_word(),rand_word());
    /* Every edge vector pair, then random. */
    int32_t a[3],b[3];
    for(int i=0;i<NEDGE;i++) for(int j=0;j<NEDGE;j++) for(int k=0;k<NEDGE;k+=3) {
        a[0]=edges[i];a[1]=edges[j];a[2]=edges[k];
        b[0]=edges[k];b[1]=edges[i];b[2]=edges[(j+5)%NEDGE];
        k1_vec(a,b);
    }
    for(int n=0;n<1000000;n++) { rand_vec(a); rand_vec(b); k1_vec(a,b); }
    for(int n=0;n<200000;n++) { rand_dir(a,1.0); rand_dir(b,0.5+1.4*rnd_unit()); k1_vec(a,b); }
    /* The 65-bit sums: all-minimum and mixed extremes, where the 64-bit sum wraps. */
    const int32_t mn[3]={INT32_MIN,INT32_MIN,INT32_MIN},mx[3]={INT32_MAX,INT32_MAX,INT32_MAX};
    const int32_t mix[3]={INT32_MIN,INT32_MAX,INT32_MIN};
    k1_vec(mn,mn); k1_vec(mn,mx); k1_vec(mx,mx); k1_vec(mix,mn); k1_vec(mix,mx);
    check(sh2z_u30_dot3(mn,mn)==INT32_MAX,"3 * 2^62 saturates up");
    check(sh2z_u30_dot3(mn,mx)==INT32_MIN,"-3 * (2^62 - 2^31) saturates down");
    saturations+=2;
    const uint32_t counted=sh2z_u30_overflows-before;
    check(counted==saturations,"overflow counter equals the oracle's saturation count");
    printf("K1: mul/q12_mul/dot3/cross equal the 128-bit oracle (%llu saturations, all counted)\n",
        (unsigned long long)saturations);
}

/* ---- K2 ----------------------------------------------------------------------- */
static double q31_lo=1e9,q31_hi=-1e9;
static void q31_one(uint32_t m) {
    const double e=(double)sh2zi_u30_rsqrt_q31(m)-uo_rsqrt_q31_ref(m);
    if(e<q31_lo) q31_lo=e;
    if(e>q31_hi) q31_hi=e;
}
static void test_rsqrt_mantissa(void) {
    uint64_t n=0;
    const char *env=getenv("SH2Z_UNIT30_EXHAUSTIVE");
    if(env && *env=='1') {
        for(uint64_t m=UINT64_C(0x40000000);m<=UINT64_C(0xffffffff);m++) q31_one((uint32_t)m);
        n=UINT64_C(0xc0000000);
    } else {
        for(uint64_t m=UINT64_C(0x40000000);m<=UINT64_C(0xffffffff);m+=4099u) { q31_one((uint32_t)m); n++; }
        for(uint32_t i=0;i<256;i++) {                  /* both ends of every seed interval */
            const uint32_t lo=i<128 ? UINT32_C(0x40000000)+(i<<23) : i<<24;
            const uint32_t hi=lo+(i<128 ? (UINT32_C(1)<<23) : (UINT32_C(1)<<24))-1u;
            for(uint32_t d=0;d<512;d++) { q31_one(lo+d); q31_one(hi-d); n+=2; }
        }
    }
    check(q31_lo>=-1.25 && q31_hi<=0.25,"mantissa within [-1.25, 0.25] Q1.31 LSB");
    printf("K2 mantissa: %llu m checked, Y - 2^47/sqrt(m) in [%.4f, %.4f] LSB%s\n",(unsigned long long)n,
        q31_lo,q31_hi,env&&*env=='1' ? " (exhaustive)" : " (strided + interval ends)");
}

static double rs_max;
static bool rsqrt_ok(int32_t got,uint64_t s,int scale) {
    if(scale<0||scale>127) return got==0;
    if(!s) return got==INT32_MAX;
    const double ref=uo_rsqrt_ref(s,scale);
    if(got==INT32_MAX) return ref>=2147483647.0-2.0;
    const double e=fabs((double)got-ref);
    if(e>rs_max) rs_max=e;
    return got>=0 && e<=2.0;
}
static void test_rsqrt64(void) {
    uint64_t n=0;
    for(int scale=-2;scale<=129;scale++) {
        for(int b=0;b<=64;b++) {
            const uint64_t base=b ? UINT64_C(1)<<(b-1) : 0;
            const uint64_t ss[4]={base,base-1,base+1,base|(base>>1)};
            for(int k=0;k<4;k++) { check(rsqrt_ok(sh2z_rsqrt64_u30(ss[k],scale),ss[k],scale),"rsqrt64 edge within 2 LSB"); n++; }
        }
        for(int i=0;i<3000;i++) {
            const uint64_t s=rnd64()>>(rnd()%64u);
            check(rsqrt_ok(sh2z_rsqrt64_u30(s,scale),s,scale),"rsqrt64 within 2 LSB"); n++;
        }
    }
    printf("K2 rsqrt64: %llu values, max |error| %.4f LSB (bound 2)\n",(unsigned long long)n,rs_max);
}

static double nm_max;
static uint64_t nm_count;
/* The checks applied to every normalisation result (also to mutants). */
static bool normalize_ok(bool ok,const int32_t u[3],const int32_t v[3],double *worst) {
    if(!v[0]&&!v[1]&&!v[2]) return !ok;
    if(!ok) return false;
    double ref[3]; uo_unit_ref(ref,v);
    for(int k=0;k<3;k++) {
        const double e=fabs((double)u[k]-ref[k]);
        if(e>*worst) *worst=e;
        if(e>2.0) return false;
        if(u[k]>0x40000002||u[k]<-0x40000002) return false;
    }
    return true;
}
static void normalize_one(const int32_t v[3]) {
    int32_t u[3]={0x11111111,0x22222222,0x33333333},w[3];
    const bool ok=sh2z_vec3_normalize_u30(u,v);
    check(normalize_ok(ok,u,v,&nm_max),"normalize within 2 LSB per component");
    if(!ok) { check(u[0]==0x11111111&&u[1]==0x22222222&&u[2]==0x33333333,"zero vector leaves out unchanged"); return; }
    if(v[0]!=INT32_MIN&&v[1]!=INT32_MIN&&v[2]!=INT32_MIN) {
        const int32_t nv[3]={-v[0],-v[1],-v[2]};
        check(sh2z_vec3_normalize_u30(w,nv) && w[0]==-u[0]&&w[1]==-u[1]&&w[2]==-u[2],"normalize is odd-symmetric");
    }
    memcpy(w,v,sizeof w);
    check(sh2z_vec3_normalize_u30(w,w) && !memcmp(w,u,sizeof w),"normalize in place");
    nm_count++;
}
static void test_normalize(void) {
    int32_t v[3];
    for(int i=0;i<NEDGE;i++) for(int j=0;j<NEDGE;j++) for(int k=0;k<NEDGE;k++) {
        v[0]=edges[i];v[1]=edges[j];v[2]=edges[k]; normalize_one(v);
    }
    for(int n=0;n<1500000;n++) { rand_any(v); normalize_one(v); }
    for(int n=0;n<300000;n++) { rand_dir(v,0.25+1.5*rnd_unit()); normalize_one(v); }
    for(int s=0;s<31;s++) for(int c=0;c<3;c++) {            /* single axes at every scale */
        v[0]=v[1]=v[2]=0; v[c]=1<<s; normalize_one(v); v[c]=-(1<<s); normalize_one(v);
    }
    printf("K2 normalize: %llu vectors, max |error| %.4f LSB (bound 2), odd-symmetric, in place\n",
        (unsigned long long)nm_count,nm_max);
}

static void isqrt_one(uint64_t s) { check(sh2z_isqrt64_q12(s)==uo_isqrt_round(s),"isqrt64 equals the rounded root"); }
static void test_isqrt(void) {
    uint64_t n=0;
    for(uint64_t s=0;s<70000;s++,n++) isqrt_one(s);
    for(int i=0;i<200000;i++) {
        const uint64_t r=rnd64()>>(rnd()%33u+31u);       /* roots up to 2^33 - 1 */
        const uint64_t base=r*r;
        if(r<=UINT32_MAX) {
            const uint64_t c[6]={base,base-1,base+1,base+r,base+r+1,base+r-1};
            for(int k=0;k<6;k++) { isqrt_one(c[k]); n++; }
        }
        isqrt_one(rnd64()>>(rnd()%64u)); n++;
    }
    const uint64_t top[6]={UINT64_MAX,UINT64_MAX-1,UINT64_C(0xfffffffe00000001),UINT64_C(0xfffffffe00000001)+UINT64_C(0xfffffffe),
        UINT64_C(0xfffffffe00000001)+UINT64_C(0xffffffff),UINT64_C(0xfffffffe00000001)+UINT64_C(0xffffffff)-1u};
    for(int k=0;k<6;k++) { isqrt_one(top[k]); n++; }
    check(sh2z_isqrt64_q12(UINT64_MAX)==UINT32_MAX,"root of 2^64 - 1 saturates");
    printf("K2 isqrt64: %llu values equal floor(sqrt(s) + 1/2)\n",(unsigned long long)n);
}

/* ---- K3 ----------------------------------------------------------------------- */
typedef int (*ortho_fn)(sh2z_basis30_t *);
typedef int (*ortho64_fn)(uo_basis64_t *);
typedef int (*ortho32_fn)(uo_basis32_t *);
/* Report bins by c = |cross| of the normalised primary row and the other row:
 * [0.5, 2), [0.1, 0.5), [0.01, 0.1), [2^-16, 0.01), below 2^-16. */
enum { BINS=5 };
static const double bin_lo[BINS]={0.5,0.1,0.01,1.0/65536.0,0.0};
static double k3_max[BINS],k3_scaled[BINS],k3_f32[BINS],k3_ortho[BINS],k3_len[BINS];
static uint64_t k3_count,k3_fail,k3_bin[BINS];
static int k3_last_bin;
static int bin_of(double c) { int b=0; while(b<BINS-1 && c<bin_lo[b]) b++; return b; }
/* The deviation bound against the float64 oracle, per output word in LSB:
 * 3 + 1.5 / c, with c = |cross| (real units) of the normalised primary row
 * and the secondary row. The same bound holds for the orthogonality of the
 * output rows (|row . row| in LSB) when c >= 2^-16. */
static double k3_bound(double c) { return 3.0+1.5/c; }
static double k3_last_c;
/* Shared checks: rc, partial-update semantics, accuracy. primary is the row
 * normalised first (0 = x for forward, 1 = y for up). */
static bool ortho_ok(int rc,const sh2z_basis30_t *in,const sh2z_basis30_t *out,int primary,bool record) {
    const int32_t *p_in=primary ? in->y : in->x,*s_in=primary ? in->x : in->y;
    const int32_t *p_out=primary ? out->y : out->x,*s_out=primary ? out->x : out->y;
    if(!p_in[0]&&!p_in[1]&&!p_in[2]) return rc==-1 && !memcmp(in,out,sizeof *in);
    int32_t pn[3]; if(!sh2z_vec3_normalize_u30(pn,p_in)) return false;
    int32_t z[3]; sh2z_u30_cross(z,primary ? pn : s_in,primary ? s_in : pn);
    if(!z[0]&&!z[1]&&!z[2])                               /* -1, primary normalised, z = 0, other row unchanged */
        return rc==-1 && !memcmp(p_out,pn,12) && !memcmp(out->z,z,12) && !memcmp(s_out,s_in,12);
    if(rc!=0) return false;
    uo_basis64_t r; uo_load64(&r,in);
    const int rc64=primary ? uo_up64(&r) : uo_forward64(&r);
    if(rc64!=0) return false;
    double pr[3]; for(int k=0;k<3;k++) pr[k]=primary ? r.y[k] : r.x[k];
    double sv[3]; for(int k=0;k<3;k++) sv[k]=ldexp((double)s_in[k],-30);
    double cc[3]; uo_cross64(cc,primary ? pr : sv,primary ? sv : pr);
    const double c=sqrt(cc[0]*cc[0]+cc[1]*cc[1]+cc[2]*cc[2]);
    const double bound=k3_bound(c);
    const int32_t *o=&out->x[0]; const double *e64=&r.x[0];
    double w=0;
    for(int k=0;k<9;k++) { const double e=fabs((double)o[k]-ldexp(e64[k],30)); if(e>w) w=e; }
    if(record) {
        const int b=bin_of(c); k3_last_bin=b; k3_bin[b]++; k3_last_c=c;
        if(w>k3_max[b]) k3_max[b]=w;
        if(w*c>k3_scaled[b]) k3_scaled[b]=w*c;
    }
    return w<=bound;
}
static void ortho_case(const sh2z_basis30_t *in,int primary) {
    sh2z_basis30_t out=*in;
    const int rc=primary ? sh2z_basis30_orthonormalize_up(&out) : sh2z_basis30_orthonormalize_forward(&out);
    k3_last_bin=-1;
    check(ortho_ok(rc,in,&out,primary,true),"orthonormalize matches the float64 oracle within its bound");
    k3_count++;
    if(rc) { k3_fail++; return; }
    const int bin=k3_last_bin;
    /* binary32 reference deviation from float64, and orthonormality of ours */
    uo_basis64_t r; uo_load64(&r,in); (void)(primary ? uo_up64(&r) : uo_forward64(&r));
    uo_basis32_t f; uo_load32(&f,in);
    if((primary ? uo_up32(&f) : uo_forward32(&f))==0) {
        const float *fp=&f.x[0]; const double *dp=&r.x[0];
        for(int k=0;k<9;k++) {
            const double e=fabs(ldexp((double)fp[k]-dp[k],30));
            if(e>k3_f32[bin]) k3_f32[bin]=e;
        }
    }
    /* Orthonormality of our output, from exact (oracle) dot products. */
    const double d[3]={fabs(ldexp((double)uo_dot_exact(out.x,out.y),-30)),fabs(ldexp((double)uo_dot_exact(out.x,out.z),-30)),
        fabs(ldexp((double)uo_dot_exact(out.y,out.z),-30))};
    for(int k=0;k<3;k++) if(d[k]>k3_ortho[bin]) k3_ortho[bin]=d[k];
    const double l[3]={fabs(ldexp((double)uo_dot_exact(out.x,out.x),-30)-1073741824.0),fabs(ldexp((double)uo_dot_exact(out.y,out.y),-30)-1073741824.0),
        fabs(ldexp((double)uo_dot_exact(out.z,out.z),-30)-1073741824.0)};
    for(int k=0;k<3;k++) if(l[k]>k3_len[bin]) k3_len[bin]=l[k];
    if(k3_last_c>=1.0/65536.0) {
        check(d[0]<=k3_bound(k3_last_c)&&d[1]<=k3_bound(k3_last_c)&&d[2]<=k3_bound(k3_last_c),"output rows orthogonal within the bound");
        check(l[0]<=8.0&&l[1]<=8.0&&l[2]<=8.0,"output rows unit to 8 LSB in len^2");
    }
}
static void rand_basis(sh2z_basis30_t *b,int primary) {
    int32_t *p=primary ? b->y : b->x,*s=primary ? b->x : b->y;
    switch(rnd()%5u) {
    case 0: rand_any(p); break;                           /* any scale */
    case 1: for(int k=0;k<3;k++) p[k]=((int32_t)(rnd()%2000001u)-1000000)*41; break;  /* Q20.12, +-10000 units */
    default: rand_dir(p,0.2+1.6*rnd_unit()); break;
    }
    const unsigned kind=rnd()%6u;
    if(kind==0) rand_dir(s,0.5+1.0*rnd_unit());           /* a general secondary row */
    else if(kind==1) {                                     /* nearly parallel to the primary */
        double d[3]; for(int k=0;k<3;k++) d[k]=(double)p[k];
        const double n=sqrt(d[0]*d[0]+d[1]*d[1]+d[2]*d[2]);
        const double eps=ldexp(1.0,-(int)(rnd()%20u)-2);
        for(int k=0;k<3;k++) s[k]=(int32_t)lrint(ldexp((n>0 ? d[k]/n : 0.0)+eps*(2.0*rnd_unit()-1.0),29));
    } else if(kind==2) for(int k=0;k<3;k++) s[k]=(int32_t)(rnd()%0x60000001u)-0x30000000;
    else rand_dir(s,1.0);
    for(int k=0;k<3;k++) b->z[k]=s32(rnd());
}
static void test_k3(void) {
    sh2z_basis30_t b;
    for(int primary=0;primary<2;primary++) {
        for(int n=0;n<400000;n++) { rand_basis(&b,primary); ortho_case(&b,primary); }
        /* -1 cases: zero primary, zero secondary, exactly parallel rows */
        memset(&b,0,sizeof b); b.z[0]=77; ortho_case(&b,primary);
        int32_t *p=primary ? b.y : b.x,*s=primary ? b.x : b.y;
        rand_dir(p,1.0); memset(s,0,12); ortho_case(&b,primary);
        for(int n=0;n<2000;n++) {
            rand_dir(p,1.0); int32_t pn[3]; check(sh2z_vec3_normalize_u30(pn,p),"unit primary");
            const int32_t f=(int32_t)(rnd()%3u)-1;
            for(int k=0;k<3;k++) s[k]=f ? f*pn[k] : pn[k]/2;
            ortho_case(&b,primary);
            p[0]=4096*(int32_t)(rnd()%100u+1u); p[1]=0; p[2]=0; s[0]=0x40000000; s[1]=0; s[2]=0; ortho_case(&b,primary);
        }
    }
    printf("K3: %llu bases, %llu returned -1 (bound per word: 3 + 1.5/|cross| LSB)\n",(unsigned long long)k3_count,(unsigned long long)k3_fail);
    printf("K3:  |cross| from   bases    ours-f64 max  max*|cross|  binary32-f64 max  |row dot| max  |len^2-1| max (LSB)\n");
    for(int i=0;i<BINS;i++)
        printf("K3:  %-12.6g %8llu %13.3f %12.3f %17.1f %14.1f %14.1f\n",bin_lo[i],(unsigned long long)k3_bin[i],
            k3_max[i],k3_scaled[i],k3_f32[i],k3_ortho[i],k3_len[i]);
}

/* ---- mutation controls: the same checks must reject each mutant ---------------- */
static int32_t mut_mul_trunc(int32_t a,int32_t b) { return uo_sat(uo_floor30((uo_i128)a*b)); }
static int32_t mut_mul_wrap(int32_t a,int32_t b) { return (int32_t)(uint32_t)(uo_floor30((uo_i128)a*b+((uo_i128)1<<29))); }
static int32_t mut_dot_three_roundings(const int32_t a[3],const int32_t b[3]) {
    return uo_sat((uo_i128)uo_mul(a[0],b[0])+uo_mul(a[1],b[1])+uo_mul(a[2],b[2]));
}
static int32_t mut_dot_wrap64(const int32_t a[3],const int32_t b[3]) {  /* sign from the wrapped 64-bit sum */
    const uint64_t q=(uint64_t)((int64_t)a[0]*b[0])+(uint64_t)((int64_t)a[1]*b[1])+(uint64_t)((int64_t)a[2]*b[2])+UINT64_C(0x20000000);
    const uint32_t hi=(uint32_t)(q>>32);
    if(hi+UINT32_C(0x20000000)>=UINT32_C(0x40000000)) return (hi>>31) ? INT32_MIN : INT32_MAX;
    return s32((uint32_t)(q>>30));
}
static bool mut_normalize_binary32(int32_t u[3],const int32_t v[3]) {   /* the float reference, in Q2.30 */
    float f[3]={(float)v[0],(float)v[1],(float)v[2]};
    if(!uo_norm32(f)) return false;
    for(int k=0;k<3;k++) u[k]=(int32_t)lrintf(ldexpf(f[k],30));
    return true;
}
static bool mut_normalize_truncate(int32_t u[3],const int32_t v[3]) {  /* truncated output words */
    if(!sh2z_vec3_normalize_u30(u,v)) return false;
    double r[3]; uo_unit_ref(r,v);
    for(int k=0;k<3;k++) u[k]=(int32_t)(r[k]>0 ? floor(r[k]-1.5) : ceil(r[k]+1.5)); /* 1.5 .. 2.5 LSB short */
    return true;
}
static uint32_t mut_isqrt_floor(uint64_t s) { return (uint32_t)uo_isqrt128((uo_i128)s); }
static int mut_forward_swapped(sh2z_basis30_t *b) {   /* z = x cross y */
    if(!sh2z_vec3_normalize_u30(b->x,b->x)) return -1;
    sh2z_u30_cross(b->z,b->x,b->y);
    if(!sh2z_vec3_normalize_u30(b->z,b->z)) return -1;
    sh2z_u30_cross(b->y,b->x,b->z);
    return 0;
}
static int mut_up_no_partial(sh2z_basis30_t *b) {     /* restores the basis on the second -1 */
    const sh2z_basis30_t keep=*b;
    const int rc=sh2z_basis30_orthonormalize_up(b);
    if(rc) *b=keep;
    return rc;
}
static void test_controls(void) {
    unsigned caught[9]={0};
    static const char *const names[9]={"mul truncates","mul wraps","dot rounds three times","dot sign from wrapped sum",
        "normalize in binary32","normalize 1.5 LSB short","isqrt floors","forward z = x cross y","up without partial update"};
    for(int n=0;n<200000;n++) {
        const int32_t a=rand_word(),b=rand_word();
        if(mut_mul_trunc(a,b)!=uo_mul(a,b)) caught[0]++;
        if(mut_mul_wrap(a,b)!=uo_mul(a,b)) caught[1]++;
        int32_t va[3],vb[3]; rand_vec(va); rand_vec(vb);
        if(mut_dot_three_roundings(va,vb)!=uo_dot3(va,vb)) caught[2]++;
        rand_any(va); int32_t u[3]; double w=0;
        if(!normalize_ok(mut_normalize_binary32(u,va),u,va,&w)) caught[4]++;
        if(!normalize_ok(mut_normalize_truncate(u,va),u,va,&w)) caught[5]++;
        const uint64_t s=rnd64()>>(rnd()%64u);
        if(mut_isqrt_floor(s)!=uo_isqrt_round(s)) caught[6]++;
    }
    const int32_t mn[3]={INT32_MIN,INT32_MIN,INT32_MIN};
    if(mut_dot_wrap64(mn,mn)!=uo_dot3(mn,mn)) caught[3]++;
    for(int n=0;n<20000;n++) {
        sh2z_basis30_t in,out;
        rand_basis(&in,0); out=in;
        if(!ortho_ok(mut_forward_swapped(&out),&in,&out,0,false)) caught[7]++;
        rand_basis(&in,1); rand_dir(in.x,1.0);
        if(n%2) { int32_t pn[3]; rand_dir(in.y,1.0); check(sh2z_vec3_normalize_u30(pn,in.y),"unit"); for(int k=0;k<3;k++) in.x[k]=pn[k]; }
        out=in;
        if(!ortho_ok(mut_up_no_partial(&out),&in,&out,1,false)) caught[8]++;
    }
    for(int c=0;c<9;c++) {
        printf("control '%s': rejected %u times\n",names[c],caught[c]);
        check(caught[c]>0,"mutation control rejected");
    }
}

/* The seed table is the library's stated rule: round(2^15 * harmonic mean of
 * 1/sqrt at the two ends of the interval). */
extern const uint16_t sh2zi_u30_seed[256];
static void test_seed_table(void) {
    for(int i=0;i<256;i++) {
        const double a=i<128 ? (128.0+i)/512.0 : i/256.0,b=i<128 ? (129.0+i)/512.0 : (i+1.0)/256.0;
        const double fa=1.0/sqrt(a),fb=1.0/sqrt(b),h=2.0*fa*fb/(fa+fb);
        check(sh2zi_u30_seed[i]==(uint16_t)lrint(h*32768.0),"seed entry follows the rule");
        check(sh2zi_u30_seed[i]>32768u,"seed entry above 2^15");
    }
    printf("K2 seed: 256 entries follow round(2^15 * harmonic mean of 1/sqrt at the interval ends)\n");
}

int main(void) {
    test_seed_table();
    test_k1();
    test_rsqrt_mantissa();
    test_rsqrt64();
    test_normalize();
    test_isqrt();
    test_k3();
    test_controls();
    printf("unit30: %llu checks passed\n",(unsigned long long)checks);
    return 0;
}
