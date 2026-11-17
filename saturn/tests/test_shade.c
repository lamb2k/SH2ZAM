/* Independent oracle, exhaustive domains, edge cases and negative controls
 * for the shade-1 kernels. The oracle is written from the rules in the header
 * text in 64- and 128-bit arithmetic with plain C division (no reciprocal
 * constants, no bucket index) and shares no code with the library.
 * MIT; SH2ZAM Contributors. */
#include "shade_oracle.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static uint64_t checks;
static uint32_t state=UINT32_C(0x5ade0001);
static void check(bool value,const char *what) {
    ++checks;
    if(!value) {
        fprintf(stderr,"shade check %llu failed: %s\n",(unsigned long long)checks,what);
        exit(EXIT_FAILURE);
    }
}
static uint32_t rnd(void) {state^=state<<13;state^=state>>17;state^=state<<5;return state;}
static int32_t range(int32_t lo,int32_t hi) {
    return (int32_t)((int64_t)lo+(int64_t)(rnd()%(uint64_t)((int64_t)hi-lo+1)));
}
/* ---- generators ------------------------------------------------------------ */
static const int8_t edge8[]={-128,-127,-1,0,1,63,64,126,127};
static int8_t rand_n8(void) {
    if(rnd()%5u==0u)return edge8[rnd()%9u];
    return (int8_t)(int32_t)range(-128,127);
}
static int32_t rand_q16_dir(void) {
    switch(rnd()%6u) {
    case 0:return 0;
    case 1:return range(-1,1);
    case 2:return (int32_t)(int8_t)(int32_t)range(-128,127)*65536;   /* int8 source */
    case 3:return (int32_t)(int16_t)(int32_t)range(-32768,32767)*65536; /* int16 source */
    default:return range(-2000000000,2000000000);
    }
}
static void rand_basis(int16_t b[3][3]) {
    const int kind=(int)(rnd()%4u);
    for(int i=0;i<3;++i)for(int j=0;j<3;++j) {
        if(kind==0)b[i][j]=(int16_t)(i==j?16384:0);
        else if(kind==1)b[i][j]=(int16_t)range(-16384,16384);
        else if(kind==2)b[i][j]=(int16_t)(rnd()%2u?16384:-16384);
        else b[i][j]=(int16_t)(i==j?range(16000,16384):range(-3000,3000));
    }
    if(kind==0 && rnd()%2u) {   /* an exact permutation with signs */
        int16_t t[3][3];memcpy(t,b,sizeof(t));
        for(int i=0;i<3;++i)for(int j=0;j<3;++j)b[i][j]=t[(i+1)%3][j];
        b[1][2]=(int16_t)-b[1][2];
    }
}
static uint8_t rand_u8(void) {
    static const uint8_t e[]={0,1,15,16,127,128,239,240,254,255};
    return rnd()%3u==0u?e[rnd()%10u]:(uint8_t)rnd();
}

static uint32_t scale_of(uint8_t c) {   /* the header's ceil(c * 2^29 / 127) */
    const uint64_t n=(uint64_t)c<<29;
    return (uint32_t)(n/127u+(n%127u!=0u));
}

/* ---- light and Gouraud ----------------------------------------------------- */
enum { NMAX=512 };
static void light_sweep(void) {
    enum { TRIALS=40000 };
    uint64_t normals=0,prepared=0,rejected=0,capped=0,saturated=0,pairs_n=0;
    uint64_t control[5]={0};
    static const unsigned controls[5]={MUT_NO_TRANSPOSE,MUT_NO_DIFFUSE_CAP,MUT_REFLECT_FLOOR,MUT_BLEND_FACTOR,MUT_MUL8_FLOOR};
    static const char *const names[5]={"basis not transposed","diffuse not capped","reflection index floored",
        "blend weight light[3]","mul8 floored"};
    static int8_t nbuf[NMAX*6];
    static uint8_t got[NMAX*8],want[NMAX*4],lut[256],bases[64*5];
    static uint16_t gg[NMAX*2],gw[NMAX*2];
    static sh2z_shade_pair_t pairs[NMAX*2];
    for(unsigned trial=0;trial<TRIALS;++trial) {
        int16_t basis[3][3];rand_basis(basis);
        const sh2z_vec3_t l={rand_q16_dir(),rand_q16_dir(),rand_q16_dir()},r={rand_q16_dir(),rand_q16_dir(),rand_q16_dir()};
        uint8_t ambient[3],color[3];
        for(int k=0;k<3;++k){ambient[k]=rand_u8();color[k]=rand_u8();}
        for(int k=0;k<256;++k)lut[k]=(uint8_t)rnd();
        sh2z_shade_light_t L;memset(&L,0xa5,sizeof(L));
        const sh2z_shade_light_t before=L;
        olight_t O;
        const bool ok=sh2z_shade_light_prepare(&L,l,r,(const int16_t(*)[3])basis,ambient,color);
        const bool ook=oracle_prepare(&O,l,r,(const int16_t(*)[3])basis,ambient,color,MUT_NONE);
        check(ok==ook,"prepare accepts exactly when both directions normalise to non-zero");
        if(!ok){check(!memcmp(&L,&before,sizeof(L)),"rejected prepare leaves out unchanged");++rejected;continue;}
        ++prepared;
        for(int k=0;k<3;++k) {
            check(L.light[k]==O.light[k] && L.reflection[k]==O.reflection[k],"prepared light equals the oracle");
            check(L.light[k]>=-49152 && L.light[k]<=49152,"prepared light bounded");
            check(L.scale[k]==scale_of(color[k]) && L.color[k]==color[k] && L.ambient[k]==ambient[k],"prepared scale and colours");
        }
        const unsigned count=1u+rnd()%NMAX,stride=3u+rnd()%4u,ostride=4u+rnd()%5u;
        for(unsigned i=0;i<count*stride;++i)nbuf[i]=rand_n8();
        memset(got,0xee,sizeof(got));
        check(sh2z_shade_light_normals(got,ostride,nbuf,stride,count,&L,lut),"light normals accepted");
        for(unsigned i=0;i<count;++i) {
            oracle_normal(want+4*i,nbuf+i*stride,&O,lut,MUT_NONE);
            check(!memcmp(got+i*ostride,want+4*i,4),"lit normal equals the oracle");
            for(unsigned g=4;g<ostride;++g)check(got[i*ostride+g]==0xee,"gap bytes untouched");
            int64_t d=0;for(int k=0;k<3;++k)d+=(int64_t)nbuf[i*stride+(unsigned)k]*O.light[k];
            if(d>127*16384)++capped;
        }
        normals+=count;
        /* Gouraud over random pairs drawn from these lit normals. */
        const unsigned nb=1u+rnd()%64u,bstride=3u+rnd()%3u,np=1u+rnd()%(2u*NMAX);
        for(unsigned i=0;i<nb*bstride;++i)bases[i]=rand_u8();
        for(unsigned i=0;i<np;++i)pairs[i]=(sh2z_shade_pair_t){(uint16_t)(rnd()%nb),(uint16_t)(rnd()%count)};
        uint8_t fog[3]={rand_u8(),rand_u8(),rand_u8()};
        uint32_t sat_got=0xdeadu,sat_want=0;
        check(sh2z_shade_gouraud(gg,&sat_got,pairs,np,bases,bstride,got,ostride,fog),"gouraud accepted");
        for(unsigned i=0;i<np;++i)gw[i]=oracle_gouraud(bases+pairs[i].base*bstride,got+pairs[i].light*ostride,fog,&sat_want,MUT_NONE);
        check(!memcmp(gg,gw,np*2u),"gouraud equals the oracle");
        check(sat_got==sat_want,"saturation count equals the oracle");
        saturated+=sat_want;pairs_n+=np;
        for(unsigned i=0;i<np;i+=7u)
            check(sh2z_shade_solid(bases+pairs[i].base*bstride,got+pairs[i].light*ostride,fog)==
                  oracle_solid(bases+pairs[i].base*bstride,got+pairs[i].light*ostride,fog),"solid equals the oracle");
        /* Controls: each mutated oracle must disagree somewhere. */
        for(unsigned c=0;c<5u;++c) {
            olight_t M;
            if(!oracle_prepare(&M,l,r,(const int16_t(*)[3])basis,ambient,color,controls[c]))continue;
            bool differs=false;
            for(unsigned i=0;i<count&&!differs;++i) {
                uint8_t m[4];oracle_normal(m,nbuf+i*stride,&M,lut,controls[c]);
                if(memcmp(m,want+4*i,4))differs=true;
            }
            for(unsigned i=0;i<np&&!differs;++i) {
                uint32_t s=0;
                if(oracle_gouraud(bases+pairs[i].base*bstride,got+pairs[i].light*ostride,fog,&s,controls[c])!=gw[i])differs=true;
            }
            if(differs)++control[c];
        }
    }
    printf("light: %u trials, %llu prepared (%llu rejected zero directions), %llu normals equal the oracle (%llu above the diffuse cap)\n",
        TRIALS,(unsigned long long)prepared,(unsigned long long)rejected,(unsigned long long)normals,(unsigned long long)capped);
    printf("gouraud: %llu pairs equal the oracle, %llu channels saturated\n",(unsigned long long)pairs_n,(unsigned long long)saturated);
    for(unsigned c=0;c<5u;++c){printf("control '%s': %llu trials differ\n",names[c],(unsigned long long)control[c]);check(control[c]>0,"light/gouraud control detected");}
}

/* Every diffuse value 0 .. 2^21 - 1 against every channel colour: normal
 * (x, y, z) in [0, 127]^3 with light (16384, 128, 1) gives d = 16384x+128y+z. */
static void light_exhaustive(void) {
    enum { N=128*128*128 };
    static int8_t n[N*3];
    static uint8_t out[N*4],lut[256];
    for(unsigned i=0;i<N;++i){n[3*i]=(int8_t)(i>>14);n[3*i+1]=(int8_t)((i>>7)&127u);n[3*i+2]=(int8_t)(i&127u);}
    for(unsigned k=0;k<256;++k)lut[k]=(uint8_t)k;
    sh2z_shade_light_t L;memset(&L,0,sizeof(L));
    L.light[0]=16384;L.light[1]=128;L.light[2]=1;
    uint64_t compared=0;
    for(unsigned c=0;c<256;c+=3u) {
        L.color[0]=(uint8_t)c;L.color[1]=(uint8_t)(c+1u<256u?c+1u:0u);L.color[2]=(uint8_t)(c+2u<256u?c+2u:1u);
        for(unsigned k=0;k<3;++k)L.scale[k]=scale_of(L.color[k]);
        check(sh2z_shade_light_normals(out,4,n,3,N,&L,lut),"exhaustive light accepted");
        for(unsigned i=0;i<N;++i) {
            const uint64_t d=i>127u*16384u?127u*16384u:i;
            for(unsigned k=0;k<3;++k)if(out[4*i+k]!=(uint8_t)(d*L.color[k]/(127u*16384u)))check(false,"exhaustive diffuse scale");
            if(out[4*i+3]!=128u)check(false,"zero reflection index");
        }
        compared+=(uint64_t)N*3u;
    }
    printf("light exhaustive: every diffuse 0 .. 2^21-1 x every colour: %llu channels exact\n",(unsigned long long)compared);
    /* Saturation of the ambient sum and extreme reflection indices. */
    L.ambient[0]=255;L.ambient[1]=128;L.ambient[2]=0;L.color[0]=L.color[1]=L.color[2]=255;
    for(unsigned k=0;k<3;++k)L.scale[k]=scale_of(255);
    L.reflection[0]=49152;L.reflection[1]=-49152;L.reflection[2]=16383;
    for(unsigned i=0;i<4096;++i){n[3*i]=(int8_t)(int32_t)range(-128,127);n[3*i+1]=(int8_t)(int32_t)range(-128,127);n[3*i+2]=(int8_t)(int32_t)range(-128,127);}
    check(sh2z_shade_light_normals(out,4,n,3,4096,&L,lut),"extreme light accepted");
    olight_t O;for(int k=0;k<3;++k){O.light[k]=L.light[k];O.reflection[k]=L.reflection[k];O.ambient[k]=L.ambient[k];O.color[k]=L.color[k];}
    unsigned low=0,high=0;
    for(unsigned i=0;i<4096;++i) {
        uint8_t w[4];oracle_normal(w,n+3*i,&O,lut,MUT_NONE);
        check(!memcmp(w,out+4*i,4),"extreme light equals the oracle");
        low+=w[3]==0u;high+=w[3]==255u;
    }
    check(low>0 && high>0,"both reflection clamps reached");
}

/* All 65536 (base, light) channel pairs for edge fogs and weights. */
static void gouraud_exhaustive(void) {
    static uint8_t bases[256*3],lights[256*4];
    static sh2z_shade_pair_t pairs[65536];
    static uint16_t got[65536];
    static const uint8_t fogs[]={0,1,127,128,254,255},weights[]={0,1,127,128,200,254,255};
    for(unsigned i=0;i<256;++i){bases[3*i]=(uint8_t)i;bases[3*i+1]=(uint8_t)(255u-i);bases[3*i+2]=(uint8_t)(i*7u);}
    for(unsigned i=0;i<65536;++i)pairs[i]=(sh2z_shade_pair_t){(uint16_t)(i>>8),(uint16_t)(i&255u)};
    uint64_t compared=0,sat_total=0;
    for(unsigned f=0;f<6;++f)for(unsigned w=0;w<7;++w) {
        for(unsigned i=0;i<256;++i){lights[4*i]=(uint8_t)i;lights[4*i+1]=(uint8_t)(i*3u);lights[4*i+2]=(uint8_t)(255u-i);lights[4*i+3]=weights[w];}
        const uint8_t fog[3]={fogs[f],fogs[5-f],fogs[(f+2)%6]};
        uint32_t sat=0,want_sat=0;
        check(sh2z_shade_gouraud(got,&sat,pairs,65536,bases,3,lights,4,fog),"exhaustive gouraud accepted");
        for(unsigned i=0;i<65536;++i)
            if(got[i]!=oracle_gouraud(bases+3*(i>>8),lights+4*(i&255u),fog,&want_sat,MUT_NONE))check(false,"exhaustive gouraud");
        check(sat==want_sat,"exhaustive saturation count");
        compared+=65536;sat_total+=sat;
    }
    printf("gouraud exhaustive: all 65536 base x light per channel, 6 fogs x 7 weights: %llu pairs, %llu saturations exact\n",
        (unsigned long long)compared,(unsigned long long)sat_total);
}

/* ---- culling ---------------------------------------------------------------- */
static void rand_plane(sh2z_shade_plane_t *p,const int32_t eye[3],int kind) {
    for(int k=0;k<3;++k) {
        p->normal[k]=(int16_t)(rnd()%6u==0u?(rnd()%2u?16384:-16384):range(-16384,16384));
        p->first[k]=(int16_t)(rnd()%6u==0u?(rnd()%2u?2048:-2048):range(-1024,1024));
    }
    int64_t c=0;for(int k=0;k<3;++k)c+=(int64_t)p->normal[k]*p->first[k];
    if(kind==1) {   /* land dot + margin on -1, 0 or +1 */
        int64_t dot=0,margin=3;
        for(int k=0;k<3;++k){const int64_t d=(int64_t)eye[k]-p->first[k];dot+=(int64_t)p->normal[k]*eye[k];margin+=(d<0?-d:d);}
        c=dot+margin+range(-1,1);   /* eye_error term added by the caller below */
    }
    if(kind==2)c=rnd()%2u?SH2Z_SHADE_PLANE_MAX:-SH2Z_SHADE_PLANE_MAX;
    if(c>SH2Z_SHADE_PLANE_MAX)c=SH2Z_SHADE_PLANE_MAX;
    if(c< -SH2Z_SHADE_PLANE_MAX)c=-SH2Z_SHADE_PLANE_MAX;
    p->plane=(int32_t)c;
}
static void cull_sweep(void) {
    enum { TRIALS=30000, FMAX=300 };
    static sh2z_shade_plane_t planes[FMAX];
    static uint16_t order[FMAX],kept[FMAX],want[FMAX];
    uint64_t faces=0,kept_total=0,boundary=0;
    uint64_t control[2]={0};
    for(unsigned trial=0;trial<TRIALS;++trial) {
        int32_t eye[3];
        for(int k=0;k<3;++k)eye[k]=rnd()%8u==0u?(rnd()%2u?30000:-30000):range(-30000,30000);
        if(rnd()%3u==0u)for(int k=0;k<3;++k)eye[k]=range(-2000,2000);
        const uint32_t err=rnd()%4u==0u?SH2Z_SHADE_ERROR_MAX:rnd()%4u==0u?0u:rnd()%8u;
        const unsigned count=1u+rnd()%FMAX;
        for(unsigned i=0;i<count;++i) {
            const int kind=rnd()%4u==0u?1:rnd()%20u==0u?2:0;
            rand_plane(&planes[i],eye,kind);
            if(kind==1) {
                int64_t c=(int64_t)planes[i].plane;
                for(int k=0;k<3;++k)c+=(int64_t)(planes[i].normal[k]<0?-planes[i].normal[k]:planes[i].normal[k])*err;
                planes[i].plane=(int32_t)clamp64(c,-SH2Z_SHADE_PLANE_MAX,SH2Z_SHADE_PLANE_MAX);
            }
            order[i]=(uint16_t)i;
        }
        for(unsigned i=count;i>1u;--i){const unsigned j=rnd()%i;const uint16_t t=order[i-1u];order[i-1u]=order[j];order[j]=t;}
        const bool use_order=rnd()%4u!=0u,in_place=use_order&&rnd()%3u==0u;
        check(sh2z_shade_planes_valid(planes,sizeof(planes[0]),count,order,count),"generated planes valid");
        size_t n=99;unsigned w=0,wm[2]={0,0};bool d[2]={false,false};
        for(unsigned i=0;i<count;++i) {
            const unsigned id=use_order?order[i]:i;
            if(oracle_keep(&planes[id],eye,err,MUT_NONE))want[w++]=(uint16_t)id;
            int64_t dot=-(int64_t)planes[id].plane,margin=3;
            for(int k=0;k<3;++k){const int64_t dd=(int64_t)eye[k]-planes[id].first[k];const int64_t nk=planes[id].normal[k];
                dot+=nk*eye[k];margin+=(dd<0?-dd:dd)+(nk<0?-nk:nk)*(int64_t)err;}
            if(dot+margin>=-1 && dot+margin<=1)++boundary;
            for(unsigned c=0;c<2u;++c) {
                const bool a=oracle_keep(&planes[id],eye,err,c?MUT_NO_EYE_ERROR:MUT_NO_MARGIN);
                if(a!=oracle_keep(&planes[id],eye,err,MUT_NONE))d[c]=true;
                wm[c]+=a;
            }
        }
        uint16_t *dst=in_place?order:kept;
        check(sh2z_shade_cull(dst,&n,use_order?order:NULL,count,planes,sizeof(planes[0]),eye,err),"cull accepted");
        check(n==w && !memcmp(dst,want,w*2u),"kept faces equal the oracle");
        faces+=count;kept_total+=w;
        for(unsigned c=0;c<2u;++c)control[c]+=d[c];
    }
    /* Larger stride: planes embedded in 24-byte records. */
    static uint32_t rec[FMAX*6];
    for(unsigned i=0;i<FMAX;++i){memcpy(&rec[6*i],&planes[i],16);rec[6*i+4]=0xdeadbeefu;rec[6*i+5]=i;}
    const int32_t eye[3]={100,-200,300};size_t n;
    check(sh2z_shade_cull(kept,&n,NULL,FMAX,rec,24,eye,2),"strided cull accepted");
    unsigned w=0;for(unsigned i=0;i<FMAX;++i)if(oracle_keep(&planes[i],eye,2,MUT_NONE))check(kept[w++]==i,"strided cull equals the oracle");
    check(n==w,"strided cull count");
    printf("cull: %u trials, %llu faces, %llu kept equal the oracle, %llu within one of the boundary\n",
        TRIALS,(unsigned long long)faces,(unsigned long long)kept_total,(unsigned long long)boundary);
    printf("control 'plain dot sign (no margin)': %llu trials differ\n",(unsigned long long)control[0]);
    printf("control 'margin without the eye-error term': %llu trials differ\n",(unsigned long long)control[1]);
    check(control[0]>0 && control[1]>0,"cull controls detected");
}

/* ---- fog -------------------------------------------------------------------- */
typedef struct { int32_t fm,fo,pa,pb,far_z; } fogp_t;
static int32_t last_near_z;
static fogp_t perspective_fog(void) {
    /* A perspective depth ratio: ratio = A - B / w with A = 2^30 (f+n)/(f-n),
     * B = 2fn/(f-n) in Q12, w = z / 4; fade = multiplier * ratio + offset
     * with the classic fog-range rule for a start in [0, 996] of 1000. */
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
    return p;
}
static int32_t rand_z(const uint32_t depth[256],int32_t far_z) {
    switch(rnd()%6u) {
    case 0:{const uint32_t d=depth[rnd()%256u];const int64_t z=(int64_t)d+range(-2,1);return (int32_t)clamp64(z,INT32_MIN,INT32_MAX);}
    case 1:return range(INT32_MIN,0);
    case 2:return (int32_t)rnd();
    case 3:return range(far_z-4<0?0:far_z-4,far_z<INT32_MAX-4?far_z+4:INT32_MAX);
    default:return range(0,far_z);
    }
}
static void fog_sweep(void) {
    enum { TABLES=6000, ZS=4000 };
    static int32_t z[ZS*3];
    static uint8_t fade[ZS*2];
    uint64_t looked=0,bucket_only_wrong=0,bucket_only_max=0,direct_cmp=0,direct_diff=0,searched=0,widest=0;
    uint64_t control[1]={0};
    uint64_t real_n=0,real_wrong=0,real_max=0,real_search=0,real_levels=0,real_widest=0;
    for(unsigned t=0;t<TABLES;++t) {
        const bool persp=t%3u!=2u;
        const fogp_t p=persp?perspective_fog():adversarial_fog();
        uint32_t want[256];oracle_fog_table(want,p.fm,p.fo,p.pa,p.pb,p.far_z);
        sh2z_shade_fog_t fog;
        check(sh2z_shade_fog_build(&fog,p.fm,p.fo,p.pa,p.pb,p.far_z),"fog build accepted");
        check(!memcmp(fog.depth,want,sizeof(want)),"fog thresholds equal the oracle");
        for(unsigned i=1;i<256;++i)check(want[i]>=want[i-1],"thresholds non-decreasing");
        for(unsigned b=0;b<256;++b){const unsigned w=(unsigned)fog.bucket[b+1]-fog.bucket[b];if(w>widest)widest=w;}
        const unsigned zstride=4u*(1u+rnd()%3u),fstride=1u+rnd()%2u,count=1u+rnd()%ZS;
        for(unsigned i=0;i<count;++i)z[i*zstride/4u]=rand_z(want,p.far_z);
        memset(fade,0xcc,sizeof(fade));
        check(sh2z_shade_fog_lookup(fade,fstride,z,zstride,count,&fog),"fog lookup accepted");
        bool differs=false;
        for(unsigned i=0;i<count;++i) {
            const int32_t zi=z[i*zstride/4u];
            const unsigned w=oracle_fade(want,zi);
            check(fade[i*fstride]==w,"fade equals the source's binary search");
            if(fstride==2u)check(fade[i*2u+1u]==0xcc,"fade gap bytes untouched");
            const unsigned b=oracle_fade_mut(&fog,zi,MUT_BUCKET_ONLY);
            if(b!=w){differs=true;++bucket_only_wrong;if(w-b>bucket_only_max)bucket_only_max=w-b;}
            const uint32_t u=zi>0?(uint32_t)zi:0u;
            if(u>>fog.shift<=255u && fog.bucket[u>>fog.shift]!=fog.bucket[(u>>fog.shift)+1u])++searched;
        }
        looked+=count;
        /* Depths spread evenly between the near and far planes: how often the
         * plain (z >> shift) table is wrong, and how much searching is left. */
        if(persp)for(unsigned i=0;i<512;++i) {
            const int32_t zi=range(last_near_z,p.far_z);
            const uint32_t b=(uint32_t)zi>>fog.shift;
            const unsigned w=oracle_fade(want,zi),only=b>255u?255u:fog.bucket[b],span=b>255u?0u:(unsigned)fog.bucket[b+1u]-only;
            ++real_n;real_levels+=span;real_search+=span!=0u;if(span>real_widest)real_widest=span;
            if(only!=w){++real_wrong;if(w-only>real_max)real_max=w-only;}
        }
        control[0]+=differs;
        /* The direct per-vertex rule, for perspective parameters inside the far plane. */
        if(persp)for(unsigned i=0;i<64;++i) {
            const int32_t zi=range(4,p.far_z);
            const wide_t N=-(wide_t)p.pb*((wide_t)1<<30),ratio=(wide_t)p.pa-N/(zi/4),x=(wide_t)p.fm*ratio;
            const wide_t fl=x>=0?x>>30:-((-x+((wide_t)1<<30)-1)>>30),v=fl+p.fo;
            const int64_t direct=v<0?0:v>255?255:(int64_t)v;
            ++direct_cmp;if((unsigned)direct!=oracle_fade(want,zi))++direct_diff;
        }
    }
    printf("fog: %u tables (2/3 perspective, 1/3 adversarial) equal the oracle; %llu lookups equal the source's binary search\n",
        TABLES,(unsigned long long)looked);
    printf("  %llu lookups needed a search inside a bucket; widest bucket spans %llu fade levels\n",(unsigned long long)searched,(unsigned long long)widest);
    printf("  direct per-vertex rule vs table, perspective, 0 < z <= far: %llu of %llu differ\n",(unsigned long long)direct_diff,(unsigned long long)direct_cmp);
    printf("control 'bucket[z >> shift] alone': %llu tables differ, %llu of %llu lookups wrong, by up to %llu levels\n",
        (unsigned long long)control[0],(unsigned long long)bucket_only_wrong,(unsigned long long)looked,(unsigned long long)bucket_only_max);
    check(control[0]>0,"bucket-only control detected");
    check(direct_diff==0,"threshold table encodes the direct rule for perspective parameters");
    printf("  perspective, depth uniform in [near, far]: %llu lookups; plain (z >> shift) table wrong for %llu (%.3f%%), by up to %llu levels;\n"
           "  %llu (%.2f%%) land in a bucket holding a threshold, mean %.3f levels per bucket searched, widest %llu\n",
        (unsigned long long)real_n,(unsigned long long)real_wrong,100.0*(double)real_wrong/(double)real_n,(unsigned long long)real_max,
        (unsigned long long)real_search,100.0*(double)real_search/(double)real_n,(double)real_levels/(double)real_n,(unsigned long long)real_widest);
    check(real_wrong>0,"the plain bucket table is not exact for perspective fog");
    /* Caller tables: random monotone, flat, all zero, all maximum. */
    for(unsigned t=0;t<4000;++t) {
        uint32_t d[256];d[0]=0;const unsigned kind=t%4u;
        for(unsigned i=1;i<256;++i) {
            if(kind==0)d[i]=d[i-1]+(rnd()%4u==0u?0u:rnd()%(1u<<(rnd()%24u)));
            else if(kind==1)d[i]=0;
            else if(kind==2)d[i]=0xffffffffu;
            else d[i]=i<128u?d[i-1]+rnd()%3u:d[i-1]+(rnd()&0xffffu);
            if(d[i]<d[i-1])d[i]=d[i-1];
        }
        sh2z_shade_fog_t fog;check(sh2z_shade_fog_from_depth(&fog,d),"caller table accepted");
        check((d[255]>>fog.shift)<=255u && (fog.shift==0u || (d[255]>>(fog.shift-1u))>255u),"smallest shift");
        for(unsigned i=0;i<512;++i){const int32_t zi=rand_z(d,(int32_t)(d[255]>0x7fffffffu?0x7fffffff:d[255]));uint8_t f;
            check(sh2z_shade_fog_lookup(&f,1,&zi,4,1,&fog) && f==oracle_fade(d,zi),"caller table lookup");}
        for(unsigned i=0;i<256;++i)if(d[i]<=0x7fffffffu){const int32_t zi=(int32_t)d[i];uint8_t f;
            check(sh2z_shade_fog_lookup(&f,1,&zi,4,1,&fog) && f==oracle_fade(d,zi),"lookup at every threshold");}
    }
    printf("fog caller tables: 4000 monotone tables (random, flat zero, flat maximum, dense then sparse) exact at every threshold\n");
}

/* ---- fade to 5:5:5 ---------------------------------------------------------- */
static void fade_sweep(void) {
    static uint8_t fades[256];
    static uint16_t got[256];
    for(unsigned i=0;i<256;++i)fades[i]=(uint8_t)i;
    uint64_t offset_n=0,mix_n=0,control[2]={0};
    /* Every fade x every slope, biases at the boundaries and at random. */
    for(int s=-127;s<=127;++s)for(unsigned bi=0;bi<24u;++bi) {
        int16_t slope[3]={(int16_t)s,(int16_t)-s,(int16_t)(s/2)},bias[3];
        for(int k=0;k<3;++k) {
            int32_t b;
            switch(bi%6u) {
            case 0:b=range(-32767,32767);break;
            case 1:b=bi%12u<6u?32767:-32767;break;
            case 2:b=range(-31,31)*255+range(-1,1);break;   /* near multiples of 255 */
            case 3:b=s*range(0,255)+range(-17,16)*255+range(-1,1);break;
            case 4:b=0;break;
            default:b=range(-8000,8000);break;
            }
            bias[k]=(int16_t)clamp64(b,-32767,32767);
        }
        check(sh2z_shade_fade_offset(got,2,fades,1,256,slope,bias),"fade offset accepted");
        bool d=false;
        for(unsigned f=0;f<256;++f) {
            const uint16_t w=oracle_offset(f,slope,bias,MUT_NONE);
            check(got[f]==w,"fade offset equals the oracle");
            if(oracle_offset(f,slope,bias,MUT_FLOOR255)!=w)d=true;
        }
        offset_n+=256;control[0]+=d;
    }
    /* fade mix: every near x far x fade per channel (exhaustive). */
    for(unsigned a=0;a<256;++a)for(unsigned b=0;b<256;++b) {
        const uint8_t nr[3]={(uint8_t)a,(uint8_t)b,(uint8_t)(a^b)},fr[3]={(uint8_t)b,(uint8_t)a,(uint8_t)(255u-a)};
        check(sh2z_shade_fade_mix(got,2,fades,1,256,nr,fr),"fade mix accepted");
        bool d=false;
        for(unsigned f=0;f<256;++f) {
            const uint16_t w=oracle_mix(f,nr,fr,MUT_NONE);
            if(got[f]!=w)check(false,"fade mix equals the oracle");
            if(oracle_mix(f,nr,fr,MUT_MIX_NO_ROUND)!=w)d=true;
        }
        mix_n+=256;control[1]+=d;
    }
    /* Strided output into 8-byte records, strided fades. */
    static uint8_t frec[256*3];static uint16_t orec[256*4];
    for(unsigned i=0;i<256;++i){frec[3*i+1]=(uint8_t)i;}
    for(unsigned i=0;i<256*4;++i)orec[i]=0x5555u;
    const int16_t s3[3]={31,-31,7},b3[3]={255*16,-255*16,1000};
    check(sh2z_shade_fade_offset(orec,8,frec+1,3,256,s3,b3),"strided offset accepted");
    for(unsigned i=0;i<256;++i){check(orec[4*i]==oracle_offset(i,s3,b3,0),"strided offset");for(unsigned g=1;g<4;++g)check(orec[4*i+g]==0x5555u,"strided gaps untouched");}
    printf("fade offset: every fade x every slope in [-127,127] x 24 bias sets: %llu colours exact\n",(unsigned long long)offset_n);
    printf("fade mix: every near x far x fade: %llu colours exact\n",(unsigned long long)mix_n);
    printf("control 'floor instead of truncating division by 255': %llu cases differ\n",(unsigned long long)control[0]);
    printf("control 'mix without +32512 rounding': %llu cases differ\n",(unsigned long long)control[1]);
    check(control[0]>0 && control[1]>0,"fade controls detected");
}

/* ---- the reciprocal constants, over their whole stated domains ------------ */
static void constants(void) {
    for(uint32_t x=0;x<=32385u;++x)check(((x*33027u)>>22)==x/127u,"x/127 constant");
    for(uint32_t x=0;x<65536u;++x)check(((x*257u+257u)>>16)==x/255u,"x/255 constant");
    for(uint32_t c=0;c<256u;++c){const uint64_t k=(((uint64_t)c<<29)+126u)/127u;
        check(k<=UINT32_MAX && (k==0u || (k-1u)*127u<((uint64_t)c<<29)) && k*127u>=((uint64_t)c<<29),"light scale is the ceiling");}
    for(uint32_t x=0;x<=2048287u;++x)if((uint32_t)(((uint64_t)x*2113633u)>>37)!=x/65025u)check(false,"x/65025 constant");
    for(uint64_t d=0;d<=127u*16384u;d+=1u+(d&7u))for(uint32_t c=0;c<256u;c+=5u)
        if(((d*c)>>14)/127u!=d*c/(127u*16384u))check(false,"nested floor identity");
    printf("constants: x/127 for x <= 32385, x/255 for x < 65536, x/65025 for x <= 2048287 exhaustive; light scales; nested floor identity sampled\n");
}

/* ---- a whole model, as a caller would chain it ------------------------------ */
/* The source draws each kept face's corners with shade(representative,
 * light[normal], fog); the kernels chain cull -> light -> gouraud(pairs). */
static void pipeline(void) {
    enum { F=112, NN=96, P=22, T=600 };
    static sh2z_shade_plane_t planes[F];
    static int8_t normals[NN*4];
    static uint16_t order[8][F],corner_normal[F][4],face_palette[F];
    static uint8_t rep[P*3],lit[NN*4],lut[256];
    uint64_t corners=0,faces_kept=0;
    for(unsigned t=0;t<T;++t) {
        for(unsigned i=0;i<NN*4;++i)normals[i]=(i&3u)==3u?0:rand_n8();
        for(unsigned i=0;i<P*3;++i)rep[i]=rand_u8();
        for(unsigned i=0;i<256;++i)lut[i]=(uint8_t)rnd();
        const int32_t eye[3]={range(-3000,3000),range(-3000,3000),range(-3000,3000)};
        for(unsigned i=0;i<F;++i){rand_plane(&planes[i],eye,rnd()%3u==0u?1:0);face_palette[i]=(uint16_t)(rnd()%P);
            for(unsigned j=0;j<4;++j)corner_normal[i][j]=(uint16_t)(rnd()%NN);}
        for(unsigned o=0;o<8;++o){for(unsigned i=0;i<F;++i)order[o][i]=(uint16_t)i;
            for(unsigned i=F;i>1u;--i){const unsigned j=rnd()%i;const uint16_t x=order[o][i-1u];order[o][i-1u]=order[o][j];order[o][j]=x;}}
        int16_t basis[3][3];rand_basis(basis);
        const sh2z_vec3_t l={(int32_t)(int8_t)rand_n8()*65536,(int32_t)(int8_t)rand_n8()*65536,65536},r={rand_q16_dir(),rand_q16_dir(),65536};
        const uint8_t ambient[3]={rand_u8(),rand_u8(),rand_u8()},color[3]={rand_u8(),rand_u8(),rand_u8()},fog[3]={rand_u8(),rand_u8(),rand_u8()};
        const unsigned oc=rnd()%8u;const uint32_t err=2;
        /* Reference: the source's loop. */
        olight_t O;check(oracle_prepare(&O,l,r,(const int16_t(*)[3])basis,ambient,color,0),"pipeline light");
        uint16_t want[F*4];unsigned wn=0;uint32_t want_sat=0;
        for(unsigned i=0;i<F;++i) {
            const unsigned id=order[oc][i];
            if(!oracle_keep(&planes[id],eye,err,0))continue;
            for(unsigned j=0;j<4;++j){uint8_t lw[4];oracle_normal(lw,normals+4*corner_normal[id][j],&O,lut,0);
                want[wn++]=oracle_gouraud(rep+3*face_palette[id],lw,fog,&want_sat,0);}
        }
        /* Kernels. */
        sh2z_shade_light_t L;uint16_t kept[F];size_t kn;
        check(sh2z_shade_light_prepare(&L,l,r,(const int16_t(*)[3])basis,ambient,color),"pipeline prepare");
        check(sh2z_shade_light_normals(lit,4,normals,4,NN,&L,lut),"pipeline normals");
        check(sh2z_shade_cull(kept,&kn,order[oc],F,planes,sizeof(planes[0]),eye,err),"pipeline cull");
        sh2z_shade_pair_t pairs[F*4];
        for(size_t i=0;i<kn;++i)for(unsigned j=0;j<4;++j)pairs[4*i+j]=(sh2z_shade_pair_t){face_palette[kept[i]],corner_normal[kept[i]][j]};
        uint16_t got[F*4];uint32_t sat;
        check(sh2z_shade_gouraud(got,&sat,pairs,kn*4u,rep,3,lit,4,fog),"pipeline gouraud");
        check(kn*4u==wn && !memcmp(got,want,wn*2u) && sat==want_sat,"pipeline equals the source loop");
        corners+=wn;faces_kept+=kn;
    }
    printf("pipeline: %u synthetic %u-face models, %llu kept faces, %llu corner colours and saturation counts equal the per-corner source loop\n",
        T,F,(unsigned long long)faces_kept,(unsigned long long)corners);
}

/* ---- argument rules ----------------------------------------------------------- */
static void argument_rules(void) {
    static uint8_t buf[4096];static uint8_t lut[256];
    sh2z_shade_light_t L;memset(&L,0,sizeof(L));
    const int16_t ident[3][3]={{16384,0,0},{0,16384,0},{0,0,16384}},bad[3][3]={{16385,0,0},{0,16384,0},{0,0,16384}};
    const uint8_t a3[3]={1,2,3};const sh2z_vec3_t x={65536,0,0},zero={0,0,0};
    check(sh2z_shade_light_prepare(&L,x,x,ident,a3,a3),"identity prepare");
    check(L.light[0]==16384 && L.light[1]==0 && L.reflection[0]==16384,"identity light");
    sh2z_shade_light_t keep=L;
    check(!sh2z_shade_light_prepare(&L,zero,x,ident,a3,a3) && !sh2z_shade_light_prepare(&L,x,zero,ident,a3,a3),"zero direction rejected");
    check(!sh2z_shade_light_prepare(&L,x,x,bad,a3,a3),"basis entry out of range");
    check(!sh2z_shade_light_prepare(NULL,x,x,ident,a3,a3) && !sh2z_shade_light_prepare(&L,x,x,NULL,a3,a3) &&
          !sh2z_shade_light_prepare(&L,x,x,ident,NULL,a3) && !sh2z_shade_light_prepare(&L,x,x,ident,a3,NULL),"null prepare arguments");
    check(!memcmp(&L,&keep,sizeof(L)),"rejected prepare writes nothing");
    int8_t *n=(int8_t *)(void *)(buf+2048);
    memset(buf,0x77,sizeof(buf));uint8_t before[4096];memcpy(before,buf,sizeof(buf));
    check(sh2z_shade_light_normals(NULL,4,NULL,3,0,NULL,NULL),"count 0 with nulls");
    check(!sh2z_shade_light_normals(buf,4,n,2,8,&L,lut) && !sh2z_shade_light_normals(buf,3,n,3,8,&L,lut),"normal/out stride");
    check(!sh2z_shade_light_normals(buf,4,n,3,8,NULL,lut) && !sh2z_shade_light_normals(buf,4,n,3,8,&L,NULL) &&
          !sh2z_shade_light_normals(NULL,4,n,3,8,&L,lut) && !sh2z_shade_light_normals(buf,4,NULL,3,8,&L,lut),"null normals arguments");
    check(!sh2z_shade_light_normals(buf+2048-31,4,n,3,8,&L,lut),"out overlaps normals");
    check(!sh2z_shade_light_normals(buf+2048+20,4,n,3,8,&L,lut),"out overlaps normals from above");
    check(!sh2z_shade_light_normals((uint8_t *)(void *)&L,4,n,3,2,&L,lut),"out overlaps light");
    check(!sh2z_shade_light_normals(lut+250,4,n,3,2,&L,lut),"out overlaps table");
    check(!memcmp(before,buf,sizeof(buf)),"rejected normals write nothing");
    check(sh2z_shade_light_normals(buf+2048-32,4,n,3,8,&L,lut),"adjacent normals fine");
    /* gouraud */
    sh2z_shade_pair_t pr[4]={{0,0},{0,0},{0,0},{0,0}};uint16_t go[8];uint32_t s=7;const uint8_t fog[3]={0,0,0};
    check(sh2z_shade_gouraud(NULL,&s,NULL,0,NULL,0,NULL,0,NULL) && s==0,"gouraud count 0");
    check(sh2z_shade_gouraud(go,NULL,pr,4,a3,3,buf,4,fog),"saturated may be null");
    check(!sh2z_shade_gouraud(go,&s,pr,4,a3,2,buf,4,fog) && !sh2z_shade_gouraud(go,&s,pr,4,a3,3,buf,3,fog),"gouraud strides");
    check(!sh2z_shade_gouraud(go,&s,NULL,4,a3,3,buf,4,fog) && !sh2z_shade_gouraud(go,&s,pr,4,a3,3,buf,4,NULL),"gouraud nulls");
    check(!sh2z_shade_gouraud((uint16_t *)(void *)pr+1,&s,pr,4,a3,3,buf,4,fog),"gouraud out overlaps pairs");
    check(sh2z_shade_solid(NULL,buf,fog)==0,"solid null");
    /* planes and cull */
    sh2z_shade_plane_t pl[3]={{{1,2,3},{4,5,6},32},{{16384,-16384,0},{2048,-2048,0},1<<28},{{0,0,0},{0,0,0},0}};
    const uint16_t ord[3]={2,0,1},ord_bad[3]={2,0,3};
    check(sh2z_shade_planes_valid(pl,16,3,ord,3) && sh2z_shade_planes_valid(NULL,16,0,NULL,0),"valid planes");
    check(!sh2z_shade_planes_valid(pl,16,3,ord_bad,3) && !sh2z_shade_planes_valid(pl,16,3,NULL,2),"order ids checked");
    pl[2].normal[1]=16385;check(!sh2z_shade_planes_valid(pl,16,3,NULL,0),"normal range");pl[2].normal[1]=-16385;check(!sh2z_shade_planes_valid(pl,16,3,NULL,0),"normal range low");pl[2].normal[1]=0;
    pl[2].first[2]=2049;check(!sh2z_shade_planes_valid(pl,16,3,NULL,0),"first range");pl[2].first[2]=0;
    pl[2].plane=(1<<28)+1;check(!sh2z_shade_planes_valid(pl,16,3,NULL,0),"plane range");pl[2].plane=0;
    check(!sh2z_shade_planes_valid(pl,12,3,NULL,0) && !sh2z_shade_planes_valid(pl,18,2,NULL,0) &&
          !sh2z_shade_planes_valid((const char *)(const void *)pl+2,16,2,NULL,0) && !sh2z_shade_planes_valid(pl,16,65537,NULL,0),"plane layout");
    uint16_t kept[4]={9,9,9,9};size_t kn=5;const int32_t eye[3]={0,0,0},far_eye[3]={30001,0,0};
    check(!sh2z_shade_cull(kept,NULL,ord,3,pl,16,eye,2),"kept_count required");
    check(sh2z_shade_cull(NULL,&kn,NULL,0,NULL,0,NULL,0) && kn==0,"cull count 0");
    kn=5;check(!sh2z_shade_cull(kept,&kn,ord,3,pl,16,far_eye,2) && kn==0,"eye range");
    check(!sh2z_shade_cull(kept,&kn,ord,3,pl,16,eye,SH2Z_SHADE_ERROR_MAX+1u),"eye error range");
    check(!sh2z_shade_cull(kept,&kn,ord,3,pl,18,eye,2) && !sh2z_shade_cull(kept,&kn,ord,3,pl,8,eye,2),"cull stride");
    uint16_t ov[5]={2,0,1,0,0};
    check(!sh2z_shade_cull(ov+1,&kn,ov,3,pl,16,eye,2),"kept partly overlaps order");
    check(kept[0]==9 && kept[3]==9,"rejected cull writes nothing");
    check(sh2z_shade_cull(ov,&kn,ov,3,pl,16,eye,2),"in-place compaction allowed");
    check(!sh2z_shade_cull(kept,&kn,ord,65537,pl,16,eye,2),"cull count limit");
    /* fog */
    sh2z_shade_fog_t fog_t;uint32_t d[256];memset(d,0,sizeof(d));
    check(sh2z_shade_fog_from_depth(&fog_t,d) && fog_t.shift==0u && fog_t.bucket[0]==255u,"all-zero table");
    d[0]=1;check(!sh2z_shade_fog_from_depth(&fog_t,d),"depth[0] must be 0");d[0]=0;
    d[10]=5;check(!sh2z_shade_fog_from_depth(&fog_t,d),"decreasing table rejected");
    check(!sh2z_shade_fog_build(&fog_t,0,0,0,0,100) && !sh2z_shade_fog_build(&fog_t,1,0,0,1,100) &&
          !sh2z_shade_fog_build(&fog_t,1,0,0,0,0) && !sh2z_shade_fog_build(NULL,1,0,0,0,1),"fog build arguments");
    check(sh2z_shade_fog_build(&fog_t,128,128,1<<30,-8192,1<<24),"fog build");
    int32_t zs[4]={1,2,3,4};uint8_t f8[8];
    check(sh2z_shade_fog_lookup(NULL,0,NULL,0,0,NULL),"lookup count 0");
    check(!sh2z_shade_fog_lookup(f8,1,(const char *)(const void *)zs+2,4,2,&fog_t) && !sh2z_shade_fog_lookup(f8,1,zs,6,2,&fog_t) &&
          !sh2z_shade_fog_lookup(f8,0,zs,4,2,&fog_t) && !sh2z_shade_fog_lookup(f8,1,zs,0,2,&fog_t),"lookup layout");
    check(!sh2z_shade_fog_lookup((uint8_t *)(void *)zs,1,zs,4,4,&fog_t),"fade over read z rejected");
    check(sh2z_shade_fog_lookup((uint8_t *)(void *)zs+4,8,zs,8,2,&fog_t),"fade into a non-z byte of a record");
    check(!sh2z_shade_fog_lookup((uint8_t *)(void *)&fog_t,1,zs,4,4,&fog_t),"fade over the table rejected");
    /* fade */
    const int16_t sl[3]={0,0,0},bi[3]={0,0,0},sl_bad[3]={128,0,0},bi_bad[3]={0,-32768,0};
    uint16_t o16[4];uint8_t fd[4]={0,1,2,3};
    check(sh2z_shade_fade_offset(NULL,0,NULL,0,0,NULL,NULL),"offset count 0");
    check(!sh2z_shade_fade_offset(o16,2,fd,1,4,sl_bad,bi) && !sh2z_shade_fade_offset(o16,2,fd,1,4,sl,bi_bad),"offset parameter range");
    check(!sh2z_shade_fade_offset(o16,3,fd,1,4,sl,bi) && !sh2z_shade_fade_offset((uint16_t *)(void *)((char *)o16+1),2,fd,1,2,sl,bi) &&
          !sh2z_shade_fade_offset(o16,2,fd,0,4,sl,bi),"offset layout");
    check(!sh2z_shade_fade_offset((uint16_t *)(void *)fd,2,fd,1,2,sl,bi),"offset overlaps fades");
    check(!sh2z_shade_fade_mix(o16,2,fd,1,4,NULL,a3) && !sh2z_shade_fade_mix((uint16_t *)(void *)fd,2,fd,1,2,a3,a3),"mix arguments");
    printf("argument rules: nulls, ranges, strides, alignment, overlap; nothing written on rejection\n");
}

int main(void) {
    argument_rules();
    constants();
    light_exhaustive();
    gouraud_exhaustive();
    light_sweep();
    cull_sweep();
    fog_sweep();
    fade_sweep();
    pipeline();
    printf("shade checks: %llu passed\n",(unsigned long long)checks);
    return EXIT_SUCCESS;
}
