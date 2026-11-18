/* Independent oracle, exhaustive domains, a road-piece pipeline, argument
 * rules and negative controls for the shade-2 kernels (fused fade lookup and
 * colour, prepared fog and ramps) and the constant-divide helpers. The oracle
 * (shade_fused_oracle.h) uses plain C division and the source's binary
 * search and shares no code with the library. MIT; SH2ZAM Contributors. */
#include "shade_fused_oracle.h"
#include "sh2zam/cdiv_experimental.h"
#include "sh2zam/view_experimental.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* The library's OFFSET channel table (internal; checked here for every x). */
extern const uint16_t sh2zi_offset_clamp[1536];

static uint64_t checks;
static uint32_t state=UINT32_C(0xfade0002);
static void check(bool value,const char *what) {
    ++checks;
    if(!value) {
        fprintf(stderr,"shade-2 check %llu failed: %s\n",(unsigned long long)checks,what);
        exit(EXIT_FAILURE);
    }
}
static uint32_t rnd(void) {state^=state<<13;state^=state>>17;state^=state<<5;return state;}
static int32_t range(int32_t lo,int32_t hi) {
    return (int32_t)((int64_t)lo+(int64_t)(rnd()%(uint64_t)((int64_t)hi-lo+1)));
}

/* ---- constant-divide helpers --------------------------------------------------- */

static void cdiv(void) {
    uint64_t n_checked=0;
    for(uint32_t n=0;n<=16909558u;++n)if(sh2z_cdiv255_u32(n)!=n/255u)check(false,"cdiv255_u32 exact on its domain");
    n_checked+=16909559u;
    check(sh2z_cdiv255_u32(16909559u)!=16909559u/255u,"cdiv255_u32 first wrong input is the stated one");
    for(int32_t n=-16909558;n<=16909558;++n)if(sh2z_cdiv255_s32(n)!=n/255)check(false,"cdiv255_s32 exact on its domain");
    n_checked+=2u*16909558u+1u;
    for(uint32_t n=0;n<=4291648u;++n)if(sh2z_cdiv65025_u32(n)!=n/65025u)check(false,"cdiv65025_u32 exact on its domain");
    n_checked+=4291649u;
    check(sh2z_cdiv65025_u32(4291649u)!=4291649u/65025u,"cdiv65025_u32 first wrong input is the stated one");
    check(SH2Z_CDIV_M(255,0)==16843010u && SH2Z_CDIV_M(65025,5)==2113633u,"named constants are the family's");
    check(SH2Z_CDIV_N(255,0)<=16909558u && SH2Z_CDIV_N(65025,5)<=4291648u,"guaranteed domains inside the true ones");
    check(SH2Z_CDIV_N(65025,5)>=2048287u,"cdiv65025 covers the fade mix (31 * 65025 + 32512)");
    /* The family: random divisors and post-shifts, every guaranteed bound
     * checked at its end, at random points and (small d) exhaustively. */
    uint64_t family=0;
    for(unsigned t=0;t<200000;++t) {
        const unsigned s=rnd()%32u;
        const uint64_t lo=(UINT64_C(1)<<s)+1u;
        uint64_t d=lo+(t%3u==0u?rnd()%64u:((uint64_t)rnd()<<1|(rnd()&1u))%(UINT64_C(0x100000000)-lo));
        if(d>0xFFFFFFFFu)d=0xFFFFFFFFu;
        const uint32_t m=SH2Z_CDIV_M(d,s),lim=SH2Z_CDIV_N(d,s);
        const uint64_t e=(uint64_t)m*d-(UINT64_C(1)<<(32u+s));
        check(e<d,"m = ceil(2^(32+s) / d)");
        const uint32_t pts[4]={lim,lim-(lim>0u),rnd()%(lim+(lim<0xFFFFFFFFu)),(uint32_t)(lim/2u)};
        for(unsigned i=0;i<4;++i){check(sh2z_cdiv_u32(pts[i],m,s)==pts[i]/d,"family exact on the guaranteed domain");++family;}
        const int32_t sn=(int32_t)(rnd()%(lim>0x7FFFFFFFu?0x7FFFFFFFu:lim+1u))*(rnd()%2u?1:-1);
        check(sh2z_cdiv_s32(sn,m,s)==(int32_t)((int64_t)sn/(int64_t)d),"signed family truncates");++family;
    }
    for(uint32_t d=2;d<=300;++d)for(unsigned s=0;(1u<<s)<d && s<8u;++s) {
        const uint32_t m=SH2Z_CDIV_M(d,s),lim=SH2Z_CDIV_N(d,s),top=lim<(1u<<16)?lim:(1u<<16);
        for(uint32_t n=0;n<=top;++n)if(sh2z_cdiv_u32(n,m,s)!=n/d)check(false,"family exhaustive, small divisors");
        family+=(uint64_t)top+1u;
    }
    printf("cdiv: cdiv255_u32, cdiv255_s32, cdiv65025_u32 equal C division on every input of their domains (%llu inputs),\n"
           "  wrong one past them; family exact on %llu checked points of guaranteed domains\n",
        (unsigned long long)n_checked,(unsigned long long)family);
}

/* ---- the OFFSET channel table, every x -------------------------------------- */

static uint64_t table_differs_from_shade1;
static void channel_table(void) {
    for(unsigned k=0;k<3;++k)for(int32_t x=-64770;x<=64770;++x) {
        const int64_t X=257*(int64_t)x+128+65536*(256+512*(int64_t)k);
        check(X>0 && X<INT64_C(0x80000000),"channel index positive, below 2^31");
        const uint32_t idx=(uint32_t)X>>16;
        check(idx<1536u,"channel index inside the table");
        const unsigned want=ofused_channel(1,x,0,0),exact=ofused_channel(1,x,0,FMUT_EXACT_TRUNC);
        const unsigned word=(want<<(5u*k))|(k==0u?0x8000u:0u);
        if(sh2zi_offset_clamp[idx]!=word)check(false,"channel table equals the header rule for every x");
        ++checks;
        if(want!=exact) {
            check(x<0 && x%255==0 && want==exact+1u,"differs from shade-1 only at negative multiples of 255, by +1");
            ++table_differs_from_shade1;
        }
    }
    printf("offset channel: every x in [-64770, 64770] for each channel equals the header rule; %llu of %u x differ\n"
           "  from shade-1's exact channel (negative multiples of 255 in the unclamped range, +1 LSB)\n",
        (unsigned long long)table_differs_from_shade1,3u*129541u);
    check(table_differs_from_shade1==3u*16u,"16 differing x per channel: -255 q for q = 1..16");
}

/* ---- fog tables ------------------------------------------------------------- */

typedef struct { int32_t fm,fo,pa,pb,far_z; } fogp_t;
static int32_t last_near_z;
static fogp_t perspective_fog(void) {
    const int32_t near_u=range(1,32),far_u=range(near_u+64,30000),fs=range(0,996);
    fogp_t p;
    p.fm=128000/(1000-fs);p.fo=(500-fs)*256/(1000-fs);
    p.pa=(int32_t)(((int64_t)(far_u+near_u)<<30)/(far_u-near_u));
    p.pb=(int32_t)-((int64_t)2*far_u*near_u*4096/(far_u-near_u));
    last_near_z=near_u*16384;
    const int64_t f=(int64_t)far_u*16384;
    p.far_z=(int32_t)(f>INT32_MAX-1?INT32_MAX-1:f);
    return p;
}
static fogp_t adversarial_fog(void) {
    fogp_t p;
    p.fm=rnd()%5u==0u?(rnd()%2u?1:INT32_MAX):range(1,40000);
    p.fo=rnd()%5u==0u?(rnd()%2u?INT32_MIN:INT32_MAX):range(-40000,300);
    p.pa=rnd()%5u==0u?(rnd()%2u?INT32_MIN:INT32_MAX):(int32_t)rnd();
    p.pb=rnd()%5u==0u?(rnd()%2u?INT32_MIN:0):-(int32_t)(rnd()&0x7fffffffu);
    p.far_z=rnd()%5u==0u?(rnd()%2u?1:INT32_MAX):range(1,INT32_MAX);
    last_near_z=1;
    return p;
}
static void caller_table(uint32_t d[256],unsigned kind) {
    d[0]=0;
    for(unsigned i=1;i<256;++i) {
        if(kind==0)d[i]=d[i-1]+(rnd()%4u==0u?0u:rnd()%(1u<<(rnd()%24u)));
        else if(kind==1)d[i]=0;
        else if(kind==2)d[i]=0xffffffffu;
        else if(kind==3)d[i]=i<128u?d[i-1]+rnd()%3u:d[i-1]+(rnd()&0xffffu);
        else d[i]=d[i-1]+(rnd()%3u==0u?0u:1u+rnd()%9u);   /* every few depths */
        if(d[i]<d[i-1])d[i]=d[i-1];
    }
}
static int32_t rand_z(const uint32_t depth[256],int32_t far_z) {
    switch(rnd()%7u) {
    case 0:{const uint32_t d=depth[rnd()%256u];const int64_t z=(int64_t)d+range(-2,1);return (int32_t)clamp64(z,INT32_MIN,INT32_MAX);}
    case 1:return range(INT32_MIN,0);
    case 2:return (int32_t)rnd();
    case 3:return range(far_z-4<0?0:far_z-4,far_z<INT32_MAX-4?far_z+4:INT32_MAX);
    case 4:return rnd()%2u?INT32_MAX:INT32_MIN;
    default:return range(0,far_z);
    }
}
/* Prepared handle from thresholds; checks its documented fields. */
static void prepare(sh2z_shade_fogp_t *fp,const uint32_t d[256]) {
    static sh2z_shade_fog_t fog;
    check(sh2z_shade_fog_from_depth(&fog,d),"shade-1 table accepted");
    check(sh2z_shade_fogp_prepare(fp,&fog),"prepared fog accepted");
    const uint32_t top=d[255]>0x7FFFFFFFu?0x7FFFFFFFu:d[255];
    check(fp->shift>=1u && fp->shift<=19u && (top>>fp->shift)<4096u && (fp->shift==1u || (top>>(fp->shift-1u))>=4096u),"smallest shift >= 1");
    check(fp->scale==(uint32_t)(UINT64_C(1)<<(32u-fp->shift)),"scale = 2^(32 - shift)");
    check(!memcmp(fp->depth,d,1024),"thresholds copied");
    for(uint32_t b=0;b<=4096u;++b) {
        const uint64_t u=(uint64_t)b<<fp->shift;
        unsigned w=0;while(w<255u && d[w+1u]<=u)++w;
        if(fp->bucket[b]!=w)check(false,"bucket[b] = fade of b << shift");
    }
    check(!fp->bucket[4097] && !fp->bucket[4098] && !fp->bucket[4099] && !fp->reserved[0],"padding zero");
}

/* ---- ramps -------------------------------------------------------------------- */

static void make_ramp(sh2z_shade_ramp_t *r,const oramp_t *o) {
    const uint16_t t[2]={(uint16_t)o->threshold[0],(uint16_t)o->threshold[1]};
    if(o->form==1u)check(sh2z_shade_ramp_offset(r,o->live,o->base,o->level,t),"offset ramp accepted");
    else check(sh2z_shade_ramp_mix(r,o->near_rgb,o->far_rgb,t),"mix ramp accepted");
}
static void rand_oramp(oramp_t *o,unsigned form,bool road) {
    memset(o,0,sizeof(*o));
    o->form=form;
    for(int k=0;k<3;++k) {
        o->live[k]=(uint8_t)(road?rnd()%32u:rnd()%4u==0u?(rnd()%2u?127u:0u):rnd()%128u);
        o->base[k]=(uint8_t)(road?rnd()%32u:rnd()%4u==0u?(rnd()%2u?127u:0u):rnd()%128u);
        o->near_rgb[k]=(uint8_t)rnd();o->far_rgb[k]=(uint8_t)rnd();
    }
    if(road){o->level[0]=0;o->level[1]=128;o->level[2]=255;o->threshold[0]=64;o->threshold[1]=192;}
    else {
        for(int k=0;k<3;++k)o->level[k]=(uint8_t)(rnd()%3u==0u?(rnd()%2u?255u:0u):rnd());
        unsigned a=rnd()%4u==0u?256u:rnd()%257u,b=rnd()%4u==0u?256u:rnd()%257u;
        if(a>b){const unsigned t=a;a=b;b=t;}
        o->threshold[0]=a;o->threshold[1]=b;
    }
}
static void rand_mean(uint8_t m[3],bool road) {
    for(int k=0;k<3;++k)m[k]=(uint8_t)(road?rnd()%32u:rnd()%4u==0u?(rnd()%2u?127u:0u):rnd()%128u);
}

/* ---- every reachable (slope, d = base - m) at every fade and level -------------- */

static void offset_exhaustive(void) {
    /* depth[i] = 4 i: the depth 4 f has fade f, so one call of 256 depths
     * covers every fade. */
    uint32_t d[256];d[0]=0;for(unsigned i=1;i<256;++i)d[i]=4u*i;
    static sh2z_shade_fogp_t fp;prepare(&fp,d);
    static int32_t z[256];for(unsigned i=0;i<256;++i)z[i]=(int32_t)(4u*i);
    static const uint8_t levels[]={0,1,127,128,254,255,77};
    static const unsigned tsel[3][2]={{256,256},{0,256},{0,0}};    /* variant 0, 1, 2 */
    uint64_t words=0,exact=0,pairs=0;
    int pending=0;uint8_t live[3],base[3],mean[3];
    for(int s=-127;s<=127;++s)for(int dd=-127;dd<=127;++dd) {
        int lo=0;if(-s>lo)lo=-s;if(-dd>lo)lo=-dd;
        int hi=127;if(127-s<hi)hi=127-s;if(127-dd<hi)hi=127-dd;
        if(lo>hi)continue;                       /* not reachable with values in 0..127 */
        const int m=lo+(int)(rnd()%(unsigned)(hi-lo+1));
        mean[pending]=(uint8_t)m;live[pending]=(uint8_t)(m+s);base[pending]=(uint8_t)(m+dd);
        ++pairs;
        if(++pending<3 && !(s==127 && dd==127))continue;
        for(;pending<3;++pending){mean[pending]=0;live[pending]=0;base[pending]=0;}
        pending=0;
        for(unsigned li=0;li<sizeof(levels);++li) {
            const unsigned v=(unsigned)(li%3u);
            oramp_t o;memset(&o,0,sizeof(o));o.form=1;
            for(int k=0;k<3;++k){o.live[k]=live[k];o.base[k]=base[k];o.level[k]=levels[li];}
            o.threshold[0]=tsel[v][0];o.threshold[1]=tsel[v][1];
            sh2z_shade_ramp_t r;make_ramp(&r,&o);
            uint16_t out[256],want[256];
            const int got=sh2z_shade_fog_gouraud(out,z,4,256,&fp,&r,mean);
            const int wv=oracle_fused(want,z,256,d,&o,mean,0);
            check(got==wv && got==(int)v,"variant");
            for(unsigned i=0;i<256;++i) {
                if(out[i]!=want[i])check(false,"offset word equals the oracle for every fade");
                const int lv=o.level[v];
                for(int k=0;k<3;++k) {
                    const unsigned e=ofused_channel(i,(int64_t)live[k]-mean[k],(int64_t)lv*((int64_t)base[k]-mean[k]),FMUT_EXACT_TRUNC);
                    const unsigned g=(out[i]>>(5*k))&31u;
                    check(g==e || g==e+1u,"within +1 LSB of shade-1's exact channel");
                    exact+=g==e;
                }
            }
            words+=256;
        }
    }
    printf("offset sweep: %llu reachable (slope, base - mean) pairs x 256 fades x 7 levels: %llu words equal the oracle;\n"
           "  %.4f%% of %llu channels equal shade-1's exact form, the rest +1\n",
        (unsigned long long)pairs,(unsigned long long)words,100.0*(double)exact/(3.0*(double)words),(unsigned long long)(3u*words));
}

/* ---- random fog, depths, ramps, counts and strides ------------------------------- */

static void fused_sweep(void) {
    enum { CASES=30000, MAXN=600 };
    static int32_t zbuf[MAXN*9];static int32_t zs[MAXN];
    static uint16_t out[MAXN+8],want[MAXN];
    static sh2z_shade_fogp_t fp;
    uint64_t words=0,fades=0,vcount[3]={0,0,0},tables=0,refs_words=0;
    for(unsigned c=0;c<CASES;++c) {
        uint32_t d[256];int32_t far_z;
        const unsigned kind=c%5u;
        if(kind<3u){const fogp_t p=kind==2u?adversarial_fog():perspective_fog();oracle_fog_table(d,p.fm,p.fo,p.pa,p.pb,p.far_z);far_z=p.far_z;}
        else {caller_table(d,rnd()%5u);far_z=(int32_t)(d[255]>0x7fffffffu?0x7fffffff:d[255]);}
        if(c%8u==0u){prepare(&fp,d);++tables;}
        else {static sh2z_shade_fog_t fog;check(sh2z_shade_fog_from_depth(&fog,d) && sh2z_shade_fogp_prepare(&fp,&fog),"prepared");}
        const unsigned n=c%4u==0u?4u:c%4u==1u?1u+rnd()%16u:1u+rnd()%MAXN;
        const unsigned stride=4u*(1u+rnd()%9u);
        for(unsigned i=0;i<n;++i){zs[i]=kind<2u&&rnd()%2u?range(last_near_z,far_z):rand_z(d,far_z);zbuf[i*stride/4u]=zs[i];}
        oramp_t o;rand_oramp(&o,rnd()%3u==0u?2u:1u,c%2u==0u);
        sh2z_shade_ramp_t r;make_ramp(&r,&o);
        uint8_t mean[3];rand_mean(mean,c%2u==0u);
        for(unsigned i=0;i<n+8u;++i)out[i]=0xA5A5;
        const int got=sh2z_shade_fog_gouraud(out,zbuf,stride,n,&fp,&r,o.form==2u&&rnd()%2u?NULL:mean);
        const int wv=oracle_fused(want,zs,n,d,&o,mean,0);
        if(got!=wv){fprintf(stderr,"case %u kind %u n %u form %u got %d want %d t %u %u\n",c,kind,n,o.form,got,wv,o.threshold[0],o.threshold[1]);
            for(unsigned i=0;i<n&&i<8;++i)fprintf(stderr," z %d fade %u\n",zs[i],oracle_fade(d,zs[i]));}
        check(got==wv,"variant equals the oracle");
        ++vcount[got];
        for(unsigned i=0;i<n;++i)if(out[i]!=want[i])check(false,"word equals the oracle");
        for(unsigned i=n;i<n+8u;++i)check(out[i]==0xA5A5,"nothing written past count");
        words+=n;
        /* The refs form over the same depths, scattered behind pointers. */
        {static const void *refs[MAXN];static int32_t scatter[MAXN*4];
         const size_t z_offset=4u*(rnd()%3u);
         for(unsigned i=0;i<n;++i){const unsigned at=(i*389u+17u)%MAXN;
             scatter[4u*at+z_offset/4u]=zs[i];refs[i]=&scatter[4u*at];}
         for(unsigned i=0;i<n+8u;++i)out[i]=0xA5A5;
         check(sh2z_shade_fog_gouraud_refs(out,refs,z_offset,n,&fp,&r,mean)==wv,"refs variant equals the oracle");
         for(unsigned i=0;i<n;++i)if(out[i]!=want[i])check(false,"refs word equals the oracle");
         for(unsigned i=n;i<n+8u;++i)check(out[i]==0xA5A5,"refs: nothing written past count");
         refs_words+=n;}
        /* Fades read back directly: a MIX ramp whose words are the fades. */
        sh2z_shade_ramp_t id;make_ramp(&id,&(oramp_t){.form=2,.threshold={256,256}});
        for(unsigned f=0;f<256;++f)id.mix[f]=(uint16_t)f;
        check(sh2z_shade_fog_gouraud(out,zbuf,stride,n,&fp,&id,NULL)==0,"identity ramp");
        for(unsigned i=0;i<n;++i)if(out[i]!=oracle_fade(d,zs[i]))check(false,"fade equals the source's binary search");
        fades+=n;
    }
    printf("fused sweep: %u calls (perspective, adversarial and caller tables; counts 1..%u; strides 4..36): %llu words\n"
           "  and %llu fades read back equal the oracle; variants %llu / %llu / %llu; %llu handles checked field by field;\n"
           "  refs form: %llu words through scattered corner pointers equal the oracle\n",
        CASES,MAXN,(unsigned long long)words,(unsigned long long)fades,(unsigned long long)vcount[0],(unsigned long long)vcount[1],
        (unsigned long long)vcount[2],(unsigned long long)tables,(unsigned long long)refs_words);
}

/* ---- the road's per-piece rule, as a caller computes it today ------------------ */

static void road_pipeline(void) {
    enum { PIECES=200000 };
    static sh2z_shade_fogp_t fp;
    uint64_t channels=0,same=0,pieces=0,solid=0,solid_words=0,lookups=0,searched=0;
    for(unsigned t=0;t<PIECES/500u;++t) {
        const fogp_t p=perspective_fog();
        uint32_t d[256];oracle_fog_table(d,p.fm,p.fo,p.pa,p.pb,p.far_z);
        prepare(&fp,d);
        oramp_t o;rand_oramp(&o,1,true);sh2z_shade_ramp_t r;make_ramp(&r,&o);
        oramp_t s;rand_oramp(&s,2,true);sh2z_shade_ramp_t rs;make_ramp(&rs,&s);
        for(unsigned q=0;q<500u;++q) {
            /* corners of one piece: 4, or a clipped 3..10, nearby depths */
            const unsigned n=q%8u==0u?3u+rnd()%8u:4u;
            sh2z_view_vertex_t v[10];memset(v,0,sizeof(v));
            const int32_t zc=range(last_near_z,p.far_z),spread=(int32_t)(rnd()%2u?zc/8:zc/64);
            for(unsigned i=0;i<n;++i){v[i].z=(int32_t)clamp64((int64_t)zc+range(-spread,spread),last_near_z,p.far_z);
                const uint32_t b=(uint32_t)v[i].z>>fp.shift;searched+=b<4096u&&fp.bucket[b]!=fp.bucket[b+1];}
            lookups+=n;
            uint8_t mean[3];rand_mean(mean,true);
            /* the caller's rule: source search, integer mean, (mean + 64) / 128 */
            unsigned fade[10],sum=0;
            for(unsigned i=0;i<n;++i){fade[i]=oracle_fade(d,v[i].z);sum+=fade[i];}
            unsigned variant=(sum/n+64u)/128u;if(variant>2u)variant=2u;
            const int selected=variant==2u?255:(int)variant*128;
            uint16_t out[10];
            const int got=sh2z_shade_fog_gouraud(out,&v[0].z,sizeof(v[0]),n,&fp,&r,mean);
            check(got==(int)variant,"road variant equals the caller's rule");
            for(unsigned i=0;i<n;++i)for(int k=0;k<3;++k) {
                const int32_t x=(int32_t)fade[i]*((int32_t)o.live[k]-mean[k])-selected*((int32_t)o.base[k]-mean[k]);
                int32_t c=16+x/255;c=c<0?0:c>31?31:c;
                const unsigned g=(out[i]>>(5*k))&31u;
                check(g==(unsigned)c || g==(unsigned)c+1u,"road channel within +1 of the caller's");
                same+=g==(unsigned)c;++channels;
            }
            check(sh2z_shade_fog_gouraud(out,&v[0].z,sizeof(v[0]),n,&fp,&rs,NULL)==(int)variant,"solid variant");
            for(unsigned i=0;i<n;++i) {
                unsigned w=0x8000u;
                for(int k=0;k<3;++k){const uint32_t f=fade[i];
                    const uint32_t c=((s.near_rgb[k]*(255u-f)+s.far_rgb[k]*f)*31u+32512u)/65025u;w|=(c>31u?31u:c)<<(5*k);}
                check(out[i]==w,"solid word equals the caller's");
                ++solid_words;
            }
            ++solid;++pieces;
        }
    }
    printf("road pieces: %llu textured and %llu solid pieces (4 corners, 1 in 8 clipped to 3..10) against the caller's rule:\n"
           "  variants equal; %.4f%% of %llu textured channels equal (the rest +1); %llu solid words equal;\n"
           "  %.2f%% of %llu lookups searched a bucket\n",
        (unsigned long long)pieces,(unsigned long long)solid,100.0*(double)same/(double)channels,(unsigned long long)channels,
        (unsigned long long)solid_words,100.0*(double)searched/(double)lookups,(unsigned long long)lookups);
}

/* ---- negative controls ------------------------------------------------------------ */

static void controls(void) {
    static const struct { unsigned mut; const char *name; } m[]={
        {FMUT_BUCKET_ONLY,"fade from the bucket start alone"},{FMUT_NEG_FAR,"negative depths fade as far"},
        {FMUT_MEAN_ROUND,"variant from the rounded mean"},{FMUT_STRICT,"variant needs mean > threshold"},
        {FMUT_EXACT_TRUNC,"shade-1's exact channel (no +1)"},{FMUT_FLOOR,"floor instead of trunc"},
        {FMUT_LEVEL0,"bias always at level[0]"},{FMUT_SLOPE_SIGN,"slope sign flipped"}};
    static sh2z_shade_fogp_t fp;
    static int32_t zs[64];static uint16_t out[64],want[64];
    for(unsigned j=0;j<sizeof(m)/sizeof(m[0]);++j) {
        uint64_t calls=0,caught=0;
        for(unsigned c=0;c<4000u;++c) {
            const fogp_t p=perspective_fog();uint32_t d[256];oracle_fog_table(d,p.fm,p.fo,p.pa,p.pb,p.far_z);
            if(c%50u==0u){static sh2z_shade_fog_t fog;check(sh2z_shade_fog_from_depth(&fog,d) && sh2z_shade_fogp_prepare(&fp,&fog),"prepared");}
            else continue;
            for(unsigned q=0;q<50u;++q) {
                const unsigned n=1u+rnd()%64u;
                for(unsigned i=0;i<n;++i)zs[i]=rnd()%8u==0u?range(INT32_MIN,0):range(last_near_z,p.far_z);
                oramp_t o;rand_oramp(&o,1,false);sh2z_shade_ramp_t r;make_ramp(&r,&o);uint8_t mean[3];rand_mean(mean,false);
                const int got=sh2z_shade_fog_gouraud(out,zs,4,n,&fp,&r,mean);
                const int wv=oracle_fused(want,zs,n,d,&o,mean,m[j].mut);
                bool differs=got!=wv;
                for(unsigned i=0;i<n;++i)differs|=out[i]!=want[i];
                caught+=differs;++calls;
            }
        }
        printf("control '%s': %llu of %llu calls differ\n",m[j].name,(unsigned long long)caught,(unsigned long long)calls);
        check(caught>0,"negative control detected");
    }
}

/* ---- argument rules ---------------------------------------------------------------- */

static void argument_rules(void) {
    uint32_t d[256];d[0]=0;for(unsigned i=1;i<256;++i)d[i]=100u*i;
    static sh2z_shade_fog_t fog;check(sh2z_shade_fog_from_depth(&fog,d),"table");
    static sh2z_shade_fogp_t fp,fp2;
    check(!sh2z_shade_fogp_prepare(NULL,&fog) && !sh2z_shade_fogp_prepare(&fp,NULL),"prepare nulls");
    memset(&fp,0x5A,sizeof(fp));
    fog.depth[3]=fog.depth[2]-1u;check(!sh2z_shade_fogp_prepare(&fp,&fog),"prepare rejects a decreasing table");
    fog.depth[3]=300u;fog.depth[0]=1u;check(!sh2z_shade_fogp_prepare(&fp,&fog),"prepare rejects depth[0] != 0");
    fog.depth[0]=0;
    {static unsigned char both[sizeof(sh2z_shade_fogp_t)+sizeof(sh2z_shade_fog_t)];
     memcpy(both+64,&fog,sizeof(fog));
     check(!sh2z_shade_fogp_prepare((sh2z_shade_fogp_t *)(void *)both,(const sh2z_shade_fog_t *)(const void *)(both+64)),"prepare rejects overlap");}
    for(size_t i=0;i<sizeof(fp);++i)if(((unsigned char *)&fp)[i]!=0x5A)check(false,"rejected prepare writes nothing");
    check(sh2z_shade_fogp_prepare(&fp,&fog),"prepare accepted");
    memcpy(&fp2,&fp,sizeof(fp));

    const uint8_t live[3]={10,20,30},base[3]={1,2,3},level[3]={0,128,255},near_rgb[3]={1,2,3},far_rgb[3]={200,100,50};
    const uint16_t th[2]={64,192},bad_th[2]={192,64},big_th[2]={64,257};
    sh2z_shade_ramp_t r,rm,keep;memset(&keep,0x77,sizeof(keep));r=keep;
    check(!sh2z_shade_ramp_offset(NULL,live,base,level,th) && !sh2z_shade_ramp_offset(&r,NULL,base,level,th) &&
          !sh2z_shade_ramp_offset(&r,live,NULL,level,th) && !sh2z_shade_ramp_offset(&r,live,base,NULL,th),"ramp offset nulls");
    check(!sh2z_shade_ramp_offset(&r,live,base,level,bad_th) && !sh2z_shade_ramp_offset(&r,live,base,level,big_th),"ramp thresholds");
    {const uint8_t hot[3]={10,128,30};check(!sh2z_shade_ramp_offset(&r,hot,base,level,th) && !sh2z_shade_ramp_offset(&r,live,hot,level,th),"ramp values <= 127");}
    check(!sh2z_shade_ramp_mix(NULL,near_rgb,far_rgb,th) && !sh2z_shade_ramp_mix(&r,NULL,far_rgb,th) &&
          !sh2z_shade_ramp_mix(&r,near_rgb,NULL,th) && !sh2z_shade_ramp_mix(&r,near_rgb,far_rgb,bad_th),"ramp mix rejects");
    check(!memcmp(&r,&keep,sizeof(r)),"rejected ramps write nothing");
    check(sh2z_shade_ramp_offset(&r,live,base,level,NULL) && r.threshold[0]==256u && r.threshold[1]==256u,"null threshold never selects");
    check(sh2z_shade_ramp_offset(&r,live,base,level,th) && r.form==SH2Z_SHADE_RAMP_OFFSET && r.live[2]==257*30 && r.base[0]==257,"offset ramp fields");
    check(sh2z_shade_ramp_mix(&rm,near_rgb,far_rgb,th) && rm.form==SH2Z_SHADE_RAMP_MIX,"mix ramp");
    for(unsigned f=0;f<256;++f)check(rm.mix[f]==oracle_mix(f,near_rgb,far_rgb,0),"mix table equals shade-1's mix for every fade");

    int32_t z[16]={0};for(unsigned i=0;i<16;++i)z[i]=(int32_t)(1000u*i);
    uint16_t out[20];const uint8_t mean[3]={5,6,7},hot[3]={5,128,7};
    #define REJECT(call,what) do{for(unsigned i=0;i<20;++i)out[i]=0xEEEE;check((call)==-1,what);\
        for(unsigned i=0;i<20;++i)check(out[i]==0xEEEE,"rejected call writes nothing");}while(0)
    REJECT(sh2z_shade_fog_gouraud(NULL,z,4,4,&fp,&r,mean),"null out");
    REJECT(sh2z_shade_fog_gouraud(out,NULL,4,4,&fp,&r,mean),"null z");
    REJECT(sh2z_shade_fog_gouraud(out,z,4,4,NULL,&r,mean),"null fog");
    REJECT(sh2z_shade_fog_gouraud(out,z,4,4,&fp,NULL,mean),"null ramp");
    REJECT(sh2z_shade_fog_gouraud(out,z,4,4,&fp,&r,NULL),"offset needs mean");
    REJECT(sh2z_shade_fog_gouraud(out,z,4,4,&fp,&r,hot),"mean > 127");
    REJECT(sh2z_shade_fog_gouraud(out,z,4,65536,&fp,&r,mean),"count > 65535");
    REJECT(sh2z_shade_fog_gouraud(out,z,6,4,&fp,&r,mean),"stride not a multiple of 4");
    REJECT(sh2z_shade_fog_gouraud(out,z,65536,4,&fp,&r,mean),"stride > 65532");
    REJECT(sh2z_shade_fog_gouraud(out,(const char *)z+2,4,4,&fp,&r,mean),"z misaligned");
    REJECT(sh2z_shade_fog_gouraud((uint16_t *)(void *)((char *)out+1),z,4,4,&fp,&r,mean),"out odd");
    {sh2z_shade_ramp_t bad=r;bad.form=0;REJECT(sh2z_shade_fog_gouraud(out,z,4,4,&fp,&bad,mean),"form 0");
     bad.form=3;REJECT(sh2z_shade_fog_gouraud(out,z,4,4,&fp,&bad,mean),"form 3");}
    for(unsigned i=0;i<20;++i)out[i]=0xEEEE;
    check(sh2z_shade_fog_gouraud(out,NULL,4,0,NULL,NULL,NULL)==0 && out[0]==0xEEEE,"count 0 returns 0, writes nothing");
    check(sh2z_shade_fog_gouraud(out,z,0,4,&fp,&rm,NULL)>=0,"stride 0 repeats one depth; mix needs no mean");
    check(out[0]==out[3],"stride 0 gives one colour");
    check(sh2z_shade_fog_gouraud(out,z,65532,1,&fp,&r,mean)>=0,"stride 65532, one vertex");
    {const void *refs[4]={&z[3],&z[0],&z[9],&z[5]};
     REJECT(sh2z_shade_fog_gouraud_refs(NULL,refs,0,4,&fp,&r,mean),"refs: null out");
     REJECT(sh2z_shade_fog_gouraud_refs(out,NULL,0,4,&fp,&r,mean),"refs: null refs");
     REJECT(sh2z_shade_fog_gouraud_refs(out,refs,0,4,NULL,&r,mean),"refs: null fog");
     REJECT(sh2z_shade_fog_gouraud_refs(out,refs,0,4,&fp,NULL,mean),"refs: null ramp");
     REJECT(sh2z_shade_fog_gouraud_refs(out,refs,0,4,&fp,&r,hot),"refs: mean > 127");
     REJECT(sh2z_shade_fog_gouraud_refs(out,refs,2,4,&fp,&r,mean),"refs: offset not a multiple of 4");
     REJECT(sh2z_shade_fog_gouraud_refs(out,refs,65536,4,&fp,&r,mean),"refs: offset > 65532");
     REJECT(sh2z_shade_fog_gouraud_refs(out,refs,0,65536,&fp,&r,mean),"refs: count > 65535");
     REJECT(sh2z_shade_fog_gouraud_refs(out,(const void *const *)(const void *)((const char *)refs+2),0,4,&fp,&r,mean),"refs misaligned");
     for(unsigned i=0;i<20;++i)out[i]=0xEEEE;
     check(sh2z_shade_fog_gouraud_refs(out,NULL,0,0,NULL,NULL,NULL)==0 && out[0]==0xEEEE,"refs: count 0 returns 0, writes nothing");
     uint16_t a[4];const int32_t zz[4]={z[3],z[0],z[9],z[5]};
     check(sh2z_shade_fog_gouraud_refs(out,refs,0,4,&fp,&r,mean)==sh2z_shade_fog_gouraud(a,zz,4,4,&fp,&r,mean) &&
           !memcmp(out,a,sizeof(a)),"refs equals the strided form");}
    check(!memcmp(&fp,&fp2,sizeof(fp)),"handles are read only");
    #undef REJECT
    printf("arguments: prepare and ramp rejects write nothing; the fused calls reject 13 + 9 bad argument sets writing nothing\n");
}

int main(void) {
    argument_rules();
    channel_table();
    cdiv();
    offset_exhaustive();
    fused_sweep();
    road_pipeline();
    controls();
    printf("shade-2 checks: %llu passed\n",(unsigned long long)checks);
    return EXIT_SUCCESS;
}
