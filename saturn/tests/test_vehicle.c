/* Independent oracle, full-domain bound proof, edge cases and negative
 * controls for the vehicle-kernels-1 kernels: screen-space facing, indexed
 * lighting, approximate Gouraud and the indexed view batch.
 * MIT; SH2ZAM Contributors. */
#include "vehicle_oracle.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static uint64_t checks;
static uint32_t state=UINT32_C(0x7e41c1e5);
static void check(bool value,const char *what) {
    ++checks;
    if(!value) {
        fprintf(stderr,"vehicle check %llu failed: %s\n",(unsigned long long)checks,what);
        exit(EXIT_FAILURE);
    }
}
static uint32_t rnd(void) {state^=state<<13;state^=state>>17;state^=state<<5;return state;}
static int32_t range(int32_t lo,int32_t hi) {
    return (int32_t)((int64_t)lo+(int64_t)(rnd()%(uint64_t)((int64_t)hi-lo+1)));
}
static uint8_t rand_u8(void) {
    static const uint8_t e[]={0,1,15,16,127,128,239,240,254,255};
    return rnd()%3u==0u?e[rnd()%10u]:(uint8_t)rnd();
}
static int64_t r255(int64_t x) {return (x+(x>>8))>>8;}

/* ---- approximate Gouraud: the bound, proven over the whole domain ---------- */
static void gouraud_bound_proof(void) {
    /* Stage 1: x >> 8 against (x + (x >> 8)) >> 8 for every x a product or a
     * blend sum can take (x <= 255 * 255 + 128). */
    for(int64_t x=0;x<=255*255+128;++x) {
        const int64_t d=r255(x)-(x>>8);
        check(d==0 || d==1,"x >> 8 is the exact rounding or one less");
    }
    /* Stage 2: every (m, m', t, f) with m' in {m - 1, m}: the blended value
     * lit' is lit or up to two less, so lit' >> 3 is lit >> 3 or one less. */
    uint64_t stage2=0;
    for(int64_t m=0;m<=255;++m)for(int64_t mm=(m?m-1:0);mm<=m;++mm)
        for(int64_t t=0;t<=255;++t)for(int64_t f=0;f<=255;++f) {
            const int64_t lit=r255(m*t+f*(255-t)+128),lit2=(mm*t+f*(255-t)+128)>>8;
            if(lit-lit2<0 || lit-lit2>2 || (lit>>3)-(lit2>>3)>1){check(false,"blend stage bound");}
            ++stage2;
        }
    checks+=stage2;
    /* Whole domain, every (b, l, t, f) per channel (2^32 tuples), counted
     * through the structure above: the channel differs from the exact one
     * exactly when lit' >> 3 = (lit >> 3) - 1 and the exact value is not
     * clamped above 31 - 1 (L <= 15 + (b >> 4)); weight[q][m][dm] counts the
     * (b, l) with b >> 4 = q, exact m and m - m' = dm. */
    static uint32_t weight[16][256][2];
    memset(weight,0,sizeof(weight));
    for(int64_t b=0;b<256;++b)for(int64_t l=0;l<256;++l) {
        const int64_t m=r255(b*l+128),mm=(b*l+128)>>8;
        check(m-mm==0 || m-mm==1,"m' is m or one less");
        ++weight[b>>4][m][m-mm];
    }
    uint64_t total=0,differ=0,below=0;
    for(int64_t m=0;m<256;++m)for(int64_t dm=0;dm<2;++dm) {
        uint64_t w[16],ws=0;
        for(int q=0;q<16;++q){w[q]=weight[q][m][dm];ws+=w[q];}
        if(!ws)continue;
        for(int64_t t=0;t<=255;++t)for(int64_t f=0;f<=255;++f) {
            const int64_t L=r255(m*t+f*(255-t)+128)>>3,L2=(((m-dm)*t+f*(255-t)+128)>>8)>>3;
            total+=ws;
            if(L-L2==1)for(int q=0;q<16;++q)if(L<=15+q){differ+=w[q];below+=w[q];}
        }
    }
    check(total==UINT64_C(4294967296),"whole domain counted");
    /* Cross-check the counting against a direct evaluation on a sample. */
    uint64_t sample=0,sample_differ=0;
    for(unsigned i=0;i<4000000u;++i) {
        const int64_t b=rnd()&255u,l=rnd()&255u,t=rnd()&255u,f=rnd()&255u;
        const int64_t m=r255(b*l+128);
        int64_t e=16+(r255(m*t+f*(255-t)+128)>>3)-(b>>4);e=e>31?31:e;
        int64_t a=ova_channel(b,l,f,t,0);a=a>31?31:a;
        check(a==e || a==e-1,"sampled channel within the bound");
        ++sample;sample_differ+=a!=e;
    }
    printf("gouraud approx: bound proven (stages 1-2: %llu cases); whole domain %llu channel tuples, "
        "%llu differ by -1 (exact match %.4f%%); sample %.4f%%\n",(unsigned long long)stage2,
        (unsigned long long)total,(unsigned long long)differ,100.0*(double)(total-differ)/(double)total,
        100.0*(double)(sample-sample_differ)/(double)sample);
    (void)below;
}

/* ---- approximate Gouraud: the library against the oracle ------------------- */
enum { PMAX=1024, LMAX=256 };
static void gouraud_sweep(void) {
    static uint8_t bases[64*5],lights[LMAX*6];
    static sh2z_shade_pair_t pairs[PMAX];
    static uint16_t got[PMAX+2],exact[PMAX];
    uint64_t n=0,differ=0;uint64_t control[2]={0,0};
    static const unsigned controls[2]={VMUT_NO_ROUND_M,VMUT_EXACT_ROUND};
    for(unsigned trial=0;trial<6000u;++trial) {
        const size_t bstride=3u+rnd()%3u,lstride=4u+rnd()%3u;
        const size_t count=trial%5u==0u?PMAX:1u+rnd()%PMAX;
        for(size_t i=0;i<sizeof(bases);++i)bases[i]=rand_u8();
        for(size_t i=0;i<sizeof(lights);++i)lights[i]=rand_u8();
        uint8_t fog[3];for(int k=0;k<3;++k)fog[k]=rand_u8();
        const size_t nb=sizeof(bases)/bstride-1u,nl=sizeof(lights)/lstride-1u;
        for(size_t i=0;i<count;++i)pairs[i]=(sh2z_shade_pair_t){(uint16_t)(rnd()%nb),(uint16_t)(rnd()%nl)};
        got[count]=0xA5A5;
        uint32_t sat=0xFFFFFFFFu,esat=0;
        check(sh2z_shade_gouraud_approx(got,&sat,pairs,count,bases,bstride,lights,lstride,fog),"approx accepts");
        check(got[count]==0xA5A5,"approx writes count words");
        check(sh2z_shade_gouraud(exact,&esat,pairs,count,bases,bstride,lights,lstride,fog),"exact accepts");
        uint32_t want_sat=0;
        for(size_t i=0;i<count;++i) {
            const uint8_t *b=bases+pairs[i].base*bstride,*l=lights+pairs[i].light*lstride;
            check(got[i]==oracle_gouraud_approx(b,l,fog,&want_sat,0),"approx equals the oracle");
            for(int k=0;k<3;++k) {
                const int a=(got[i]>>(5*k))&31,e=(exact[i]>>(5*k))&31;
                check(a==e || a==e-1,"approx within -1 of the exact channel");
                ++n;differ+=a!=e;
            }
            check(!(got[i]&0x8000u),"bit 15 clear");
            for(unsigned c=0;c<2u;++c){uint32_t s=0;if(oracle_gouraud_approx(b,l,fog,&s,controls[c])!=got[i])++control[c];}
        }
        check(sat==want_sat,"approx saturation count");
    }
    /* Every base x light byte pair per channel for a set of fogs and weights. */
    uint64_t grid=0;
    for(unsigned g=0;g<12u;++g) {
        const uint8_t fog[3]={(uint8_t)(g*23u),(uint8_t)(255u-g*21u),(uint8_t)(g&1u?255u:0u)};
        const uint8_t w=(uint8_t)(g*22u+1u);
        for(unsigned b=0;b<256;++b) {
            uint8_t base[3]={(uint8_t)b,(uint8_t)(255u-b),(uint8_t)(b*7u)};
            static uint8_t lt[256*4];
            static sh2z_shade_pair_t pp[256];
            for(unsigned l=0;l<256;++l){lt[4*l]=(uint8_t)l;lt[4*l+1]=(uint8_t)(l*3u);lt[4*l+2]=(uint8_t)(255u-l);lt[4*l+3]=w;pp[l]=(sh2z_shade_pair_t){0,(uint16_t)l};}
            uint32_t sat=0,ws=0;
            check(sh2z_shade_gouraud_approx(got,&sat,pp,256,base,3,lt,4,fog),"grid accepts");
            for(unsigned l=0;l<256;++l)check(got[l]==oracle_gouraud_approx(base,lt+4*l,fog,&ws,0),"grid equals the oracle");
            check(sat==ws,"grid saturation");
            grid+=256;
        }
    }
    printf("gouraud approx: %llu random channels (%.4f%% exact), %llu grid pairs, equal to the oracle\n",
        (unsigned long long)n,100.0*(double)(n-differ)/(double)n,(unsigned long long)grid);
    printf("control 'm without + 128': %llu pairs differ\ncontrol 'exact rounding of lit': %llu pairs differ\n",
        (unsigned long long)control[0],(unsigned long long)control[1]);
    check(control[0]>0 && control[1]>0,"gouraud controls detected");
    /* Argument rules: same as sh2z_shade_gouraud. */
    uint8_t fog[3]={1,2,3};uint32_t s=7;
    check(sh2z_shade_gouraud_approx(got,&s,pairs,0,0,0,0,0,0) && s==0,"count 0 succeeds");
    check(!sh2z_shade_gouraud_approx(0,&s,pairs,1,bases,3,lights,4,fog),"null out");
    check(!sh2z_shade_gouraud_approx(got,&s,pairs,1,bases,2,lights,4,fog),"base stride < 3");
    check(!sh2z_shade_gouraud_approx(got,&s,pairs,1,bases,3,lights,3,fog),"light stride < 4");
    check(!sh2z_shade_gouraud_approx((uint16_t *)(void *)pairs,&s,pairs,4,bases,3,lights,4,fog),"out overlaps pairs");
    check(sh2z_shade_gouraud_approx(got,0,pairs,1,bases,3,lights,4,fog),"saturated may be null");
}

/* ---- indexed lighting ------------------------------------------------------ */
static void light_sweep(void) {
    static int8_t normals[256*6];
    static uint8_t got[256*8],want[256*8],lut[256],index[600];
    uint64_t lit=0,trials=0,untouched=0,control=0;
    for(unsigned trial=0;trial<20000u;++trial) {
        int16_t basis[3][3];
        const int kind=(int)(rnd()%3u);
        for(int i=0;i<3;++i)for(int j=0;j<3;++j)
            basis[i][j]=(int16_t)(kind==0?(i==j?16384:0):kind==1?range(-16384,16384):(i==j?range(15000,16384):range(-4000,4000)));
        const sh2z_vec3_t l={range(-127,127)*65536,range(-127,127)*65536,range(-127,127)*65536+(rnd()%2u?range(-2000000000,2000000000):0)};
        const sh2z_vec3_t r={range(-32767,32767)*65536,range(-32767,32767)*65536,range(-2000000000,2000000000)};
        uint8_t ambient[3],color[3];for(int k=0;k<3;++k){ambient[k]=rand_u8();color[k]=rand_u8();}
        sh2z_shade_light_t L;
        if(!sh2z_shade_light_prepare(&L,l,r,(const int16_t (*)[3])basis,ambient,color))continue;
        if(trial%7u==0u)for(int k=0;k<3;++k){L.light[k]=range(-49152,49152);L.reflection[k]=range(-49152,49152);}
        const size_t nstride=3u+rnd()%3u,ostride=4u+rnd()%3u,count=1u+rnd()%256u;
        for(size_t i=0;i<count*nstride;++i)normals[i]=(int8_t)(rnd()%5u==0u?(rnd()%2u?-128:127):range(-128,127));
        for(int i=0;i<256;++i)lut[i]=(uint8_t)rnd();
        const size_t ni=trial%5u==0u?600u:rnd()%300u;
        for(size_t k=0;k<ni;++k)index[k]=(uint8_t)(rnd()%count);
        memset(got,0x5A,sizeof(got));
        memset(want,0x5A,sizeof(want));
        check(sh2z_shade_light_normals(want,ostride,normals,nstride,count,&L,lut),"light_normals accepts");
        check(sh2z_shade_light_normals_indexed(got,ostride,normals,nstride,count,index,ni,&L,lut),"indexed accepts");
        static bool listed[256];memset(listed,0,sizeof(listed));
        for(size_t k=0;k<ni;++k)listed[index[k]]=true;
        bool differs_if_all=false;
        for(size_t i=0;i<count;++i)for(size_t b=0;b<ostride;++b) {
            const uint8_t g=got[i*ostride+b];
            if(listed[i] && b<4u)check(g==want[i*ostride+b],"listed normal equals light_normals");
            else {check(g==0x5A,"other bytes untouched");++untouched;if(b<4u && want[i*ostride+b]!=0x5A)differs_if_all=true;}
        }
        for(size_t i=0;i<count;++i)if(listed[i]) {
            uint8_t o[4];olight_t ol;
            for(int k=0;k<3;++k){ol.light[k]=L.light[k];ol.reflection[k]=L.reflection[k];ol.ambient[k]=L.ambient[k];ol.color[k]=L.color[k];}
            oracle_normal(o,normals+i*nstride,&ol,lut,0);
            check(!memcmp(o,got+i*ostride,4),"indexed equals the oracle");
            ++lit;
        }
        control+=differs_if_all;++trials;
    }
    printf("indexed light: %llu trials, %llu listed normals equal light_normals and the oracle, %llu other bytes untouched\n",
        (unsigned long long)trials,(unsigned long long)lit,(unsigned long long)untouched);
    printf("control 'every normal written': %llu trials differ\n",(unsigned long long)control);
    check(control>0,"index control detected");
    /* Argument rules. */
    sh2z_shade_light_t L;memset(&L,0,sizeof(L));
    check(sh2z_shade_light_normals_indexed(0,0,0,0,0,0,0,0,0),"index_count 0 succeeds");
    check(!sh2z_shade_light_normals_indexed(got,4,normals,4,0,index,1,&L,lut),"count 0 with indices");
    check(!sh2z_shade_light_normals_indexed(got,4,normals,4,4,0,1,&L,lut),"null index");
    check(!sh2z_shade_light_normals_indexed(got,3,normals,4,4,index,1,&L,lut),"out stride < 4");
    check(!sh2z_shade_light_normals_indexed(got,4,normals,2,4,index,1,&L,lut),"normal stride < 3");
    check(!sh2z_shade_light_normals_indexed(got,4,(const int8_t *)got,4,4,index,1,&L,lut),"out overlaps normals");
    check(!sh2z_shade_light_normals_indexed(got,4,normals,4,4,got+8,1,&L,lut),"out overlaps index");
    check(!sh2z_shade_light_normals_indexed(got,4,normals,4,4,index,1,&L,got),"out overlaps table");
}

/* ---- screen facing --------------------------------------------------------- */
enum { FV=200, FF=400 };
static sh2z_view_vertex_t verts[FV];
static int16_t rand_coord(void) {
    switch(rnd()%7u) {
    case 0:return (int16_t)range(-1025,-1021);
    case 1:return (int16_t)range(1021,1025);
    case 2:return (int16_t)range(INT16_MIN,INT16_MAX);
    case 3:return (int16_t)range(-60,400);
    default:return (int16_t)range(-1023,1023);
    }
}
static void rand_vertices(const sh2z_view_t *v) {
    for(unsigned i=0;i<FV;++i) {
        sh2z_view_vertex_t *a=&verts[i];
        memset(a,0,sizeof(*a));
        a->z=range(1,1000000);
        if(rnd()%12u==0u){a->outcode=SH2Z_OUT_NEAR;continue;}
        a->sx=rand_coord();a->sy=rand_coord();
        unsigned code=rnd()%10u==0u?SH2Z_OUT_FAR:0u;
        if(a->sx<v->clip[0])code|=SH2Z_OUT_LEFT;
        if(a->sx>v->clip[2])code|=SH2Z_OUT_RIGHT;
        if(a->sy<v->clip[1])code|=SH2Z_OUT_TOP;
        if(a->sy>v->clip[3])code|=SH2Z_OUT_BOTTOM;
        a->outcode=(uint8_t)code;
    }
}
static void facing_sweep(void) {
    static uint8_t corners[FF][4] __attribute__((aligned(4)));
    static uint16_t faces[FF],kept[FF+1],want[FF];
    uint64_t faces_n=0,kept_n=0,simple_n=0,slow_n=0;
    uint64_t control[5]={0};
    static const unsigned controls[5]={VMUT_ONE_TRIANGLE,VMUT_NO_SIMPLE,VMUT_NO_ALL,VMUT_KEEP_EDGE_ON,VMUT_LIMIT_1024};
    static const char *const names[5]={"area of triangle 0-1-2 only","no simple gate","no AND-outcode reject",
        "edge-on faces kept","limit 1024"};
    for(unsigned trial=0;trial<20000u;++trial) {
        sh2z_view_t v;memset(&v,0,sizeof(v));
        if(trial%3u==0u){v.clip[0]=(int16_t)range(-2000,0);v.clip[2]=(int16_t)range(300,2000);v.clip[1]=(int16_t)range(-2000,0);v.clip[3]=(int16_t)range(200,2000);}
        else{v.clip[0]=0;v.clip[1]=0;v.clip[2]=(int16_t)range(200,1023);v.clip[3]=(int16_t)range(200,1023);}
        rand_vertices(&v);
        /* Small coordinates make edge-on and degenerate quads common. */
        if(trial%4u==1u)for(unsigned i=0;i<FV;++i)if(!(verts[i].outcode&SH2Z_OUT_NEAR)){verts[i].sx=(int16_t)range(0,6);verts[i].sy=(int16_t)range(0,6);verts[i].outcode=verts[i].outcode&SH2Z_OUT_FAR;}
        const unsigned nf=1u+rnd()%FF;
        for(unsigned i=0;i<FF;++i){for(int j=0;j<4;++j)corners[i][j]=(uint8_t)(rnd()%FV);if(rnd()%5u==0u)corners[i][3]=corners[i][2];}
        for(unsigned i=0;i<nf;++i)faces[i]=(uint16_t)(rnd()%FF);
        const int sign=rnd()%2u?1:-1;
        unsigned w=0;
        for(unsigned i=0;i<nf;++i) {
            const uint8_t *c=corners[faces[i]];
            const sh2z_view_vertex_t *q[4]={verts+c[0],verts+c[1],verts+c[2],verts+c[3]};
            if(ovf_keep(q,sign,0))want[w++]=faces[i];
            bool s=true;for(int j=0;j<4;++j){if(q[j]->outcode)s=false;}
            if(s)++simple_n;else ++slow_n;
            for(unsigned c2=0;c2<5u;++c2)if(ovf_keep(q,sign,controls[c2])!=ovf_keep(q,sign,0))++control[c2];
        }
        size_t n=99;kept[nf]=0xA5A5;
        const bool inplace=trial%2u==0u;
        uint16_t *dst=inplace?faces:kept;
        check(sh2z_shade_screen_facing(dst,&n,faces,nf,(const uint8_t (*)[4])corners,verts,&v,sign),"facing accepts");
        check(n==w,"kept count");
        for(unsigned i=0;i<w;++i)check(dst[i]==want[i],"kept ids in order");
        if(!inplace)check(kept[nf]==0xA5A5,"no write past count");
        faces_n+=nf;kept_n+=w;
    }
    printf("facing: %llu faces (%llu with zero outcodes, %llu others), %llu kept, equal to the oracle\n",
        (unsigned long long)faces_n,(unsigned long long)simple_n,(unsigned long long)slow_n,(unsigned long long)kept_n);
    for(unsigned c=0;c<5u;++c){printf("control '%s': %llu faces differ\n",names[c],(unsigned long long)control[c]);check(control[c]>0,"facing control detected");}
    /* Argument rules. */
    sh2z_view_t v;memset(&v,0,sizeof(v));size_t n=5;
    check(!sh2z_shade_screen_facing(kept,0,faces,1,(const uint8_t (*)[4])corners,verts,&v,1),"null count");
    check(sh2z_shade_screen_facing(0,&n,0,0,0,0,0,0) && n==0,"count 0 succeeds");
    n=5;check(!sh2z_shade_screen_facing(kept,&n,faces,1,(const uint8_t (*)[4])corners,verts,&v,0) && n==0,"sign 0");
    check(!sh2z_shade_screen_facing(kept,&n,faces,1,(const uint8_t (*)[4])corners,verts,&v,2),"sign 2");
    check(!sh2z_shade_screen_facing(kept,&n,faces,1,(const uint8_t (*)[4])corners,verts,0,1),"null view");
    check(!sh2z_shade_screen_facing(faces+1,&n,faces,4,(const uint8_t (*)[4])corners,verts,&v,1),"partial overlap");
    /* A hand case: unit square, counter-clockwise on screen (y down) has
     * negative area; sign -1 keeps it, sign +1 drops it. */
    sh2z_view_vertex_t sq[4];memset(sq,0,sizeof(sq));
    sq[1].sx=10;sq[2].sx=10;sq[2].sy=-10;sq[3].sy=-10;
    for(int j=0;j<4;++j){sq[j].sx=(int16_t)(sq[j].sx+100);sq[j].sy=(int16_t)(sq[j].sy+100);}
    v.clip[2]=300;v.clip[3]=300;
    static uint8_t one[1][4] __attribute__((aligned(4)))={{0,1,2,3}};
    uint16_t id=0,out=9;
    check(sh2z_shade_screen_facing(&out,&n,&id,1,(const uint8_t (*)[4])one,sq,&v,-1) && n==1 && out==0,"square kept with sign -1");
    check(sh2z_shade_screen_facing(&out,&n,&id,1,(const uint8_t (*)[4])one,sq,&v,1) && n==0,"square dropped with sign +1");
}

/* ---- indexed view batch ---------------------------------------------------- */
static int16_t q14(double x) {
    const double s=floor(x*16384.0+0.5);
    return (int16_t)(s>32767?32767:s<-32768?-32768:s);
}
static sh2z_view_t random_view(void) {
    const double a=(rnd()%65536)*6.283185307179586/65536.0,b=((double)(rnd()%65536)/65536.0-0.5)*3.0;
    const double r[3][3]={{cos(a),0,-sin(a)},{sin(a)*sin(b),cos(b),cos(a)*sin(b)},{sin(a)*cos(b),-sin(b),cos(a)*cos(b)}};
    sh2z_view_t v;memset(&v,0,sizeof(v));
    for(int i=0;i<3;++i)for(int j=0;j<3;++j)v.r[i][j]=q14(r[i][j]);
    v.focal_x=range(64,1024)*65536+range(0,65535);v.focal_y=range(64,1024)*65536+range(0,65535);
    v.center_x=(int16_t)range(0,352);v.center_y=(int16_t)range(0,256);
    v.near_z=range(1,16*16384);v.far_z=v.near_z+range(1,INT32_MAX-v.near_z);
    v.clip[0]=(int16_t)range(-64,160);v.clip[2]=(int16_t)range(v.clip[0],400);
    v.clip[1]=(int16_t)range(-64,120);v.clip[3]=(int16_t)range(v.clip[1],300);
    sh2z_view_t valid;
    check(sh2z_view_init(&valid,&v),"random view valid");
    const sh2z_vec3_t e={range(-3000,3000)*65536,range(-3000,3000)*65536,range(-3000,3000)*65536};
    check(sh2z_view_place(&valid,e),"eye placeable");
    return valid;
}
static void view_sweep(void) {
    enum { VC=256 };
    static int16_t pos[VC*4];
    static sh2z_view_vertex_t got[VC+1],gotd[VC+1],want[VC];
    static uint8_t index[700];
    uint64_t done=0,control=0;
    for(unsigned trial=0;trial<4000u;++trial) {
        const sh2z_view_t v=random_view();
        const size_t stride=6u+2u*(rnd()%2u),count=1u+rnd()%VC;
        for(size_t i=0;i<count*stride/2u;++i)pos[i]=(int16_t)(rnd()%3u==0u?range(INT16_MIN,INT16_MAX):range(-4000,4000));
        const size_t ni=trial%4u==0u?700u:rnd()%200u;
        for(size_t k=0;k<ni;++k)index[k]=(uint8_t)(rnd()%count);
        check(sh2z_view_batch(want,&v,pos,stride,count),"batch accepts");
        memset(got,0x3C,sizeof(got));memset(gotd,0x3C,sizeof(gotd));
        check(sh2z_view_batch_indexed(got,&v,pos,stride,count,index,ni),"indexed accepts");
        check(sh2z_view_batch_divu_indexed(gotd,&v,pos,stride,count,index,ni),"divu indexed accepts");
        static bool listed[VC];memset(listed,0,sizeof(listed));
        for(size_t k=0;k<ni;++k)listed[index[k]]=true;
        sh2z_view_vertex_t sentinel;memset(&sentinel,0x3C,sizeof(sentinel));
        bool would=false;
        for(size_t i=0;i<=count;++i) {
            if(i<count && listed[i]) {
                check(!memcmp(&got[i],&want[i],sizeof(want[i])),"indexed equals batch");
                check(!memcmp(&gotd[i],&want[i],sizeof(want[i])),"divu indexed equals batch");
                ++done;
            } else {
                check(!memcmp(&got[i],&sentinel,sizeof(sentinel)) && !memcmp(&gotd[i],&sentinel,sizeof(sentinel)),"unlisted records untouched");
                if(i<count)would=true;
            }
        }
        control+=would;
    }
    printf("indexed view: %llu listed vertices equal sh2z_view_batch (both forms); others untouched\n",(unsigned long long)done);
    printf("control 'every vertex written': %llu trials differ\n",(unsigned long long)control);
    check(control>0,"view index control detected");
    const sh2z_view_t v=random_view();
    sh2z_view_t bad=v;bad.focal_x=0;
    check(sh2z_view_batch_divu_indexed(0,0,0,0,0,0,0),"index_count 0 succeeds");
    check(!sh2z_view_batch_divu_indexed(got,&bad,pos,6,4,index,1),"invalid view");
    check(!sh2z_view_batch_divu_indexed(got,&v,pos,6,0,index,1),"count 0");
    check(!sh2z_view_batch_divu_indexed(got,&v,pos,6,4,0,1),"null index");
    check(!sh2z_view_batch_divu_indexed(got,&v,pos,5,4,index,1),"odd stride");
    check(!sh2z_view_batch_divu_indexed(got,&v,got,6,4,index,1),"out overlaps positions");
    check(!sh2z_view_batch_divu_indexed(got,&v,pos,6,4,(const uint8_t *)(const void *)(got+1),1),"out overlaps index");
    check(!sh2z_view_batch_indexed(got,&v,pos,6,4,(const uint8_t *)(const void *)(got+1),1),"portable: out overlaps index");
}

int main(void) {
    gouraud_bound_proof();
    gouraud_sweep();
    light_sweep();
    facing_sweep();
    view_sweep();
    printf("vehicle kernels: %llu checks passed\n",(unsigned long long)checks);
    return EXIT_SUCCESS;
}
