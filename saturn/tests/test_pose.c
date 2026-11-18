/* pose-1 (sh2z_pose_from_f32) against the binary32 oracle in pose_oracle.h:
 * exhaustive and stratified tests of the three emulated binary32 operations,
 * randomised whole poses over the world range, every special float in every
 * input, the projection gate, argument rules and negative controls.
 *
 * The library source is included so the private decoders can be tested
 * directly; the archive's copy of the same object is then not linked.
 * MIT; SH2ZAM Contributors. */
#include "../src/sh2zam_pose.c"
#include "pose_oracle.h"
#include <stdio.h>
#include <stdlib.h>

/* The two values of pose_element separately. */
static int32_t pose_q30(uint32_t b) {
    int32_t B,M;
    pose_element(b,UINT32_C(0x80000000),pose_g+4,&B,&M);
    return B;
}
static int32_t pose_model(uint32_t b,uint32_t ms,uint32_t es) {
    int32_t B,M;
    pose_element(b,ms<<8,pose_g+es-123u,&B,&M);
    return M;
}

static uint64_t checks;
static void check(bool value,const char *what) {
    ++checks;
    if(!value) {
        fprintf(stderr,"pose check %llu failed: %s\n",(unsigned long long)checks,what);
        exit(EXIT_FAILURE);
    }
}

/* ---- the three binary32 operations ---------------------------------------- */

/* Every scale the pose accepts: 33,554,433 values. */
static void test_reciprocal(void) {
    uint64_t n=0;
    for(uint32_t s=UINT32_C(0x3D800000);s<=UINT32_C(0x3F800000);++s) {
        const float f=po_float(s);
        const int32_t want=(int32_t)((1.0f/f)*65536.0f);
        const uint32_t got=pose_inverse((s&UINT32_C(0x7FFFFF))|UINT32_C(0x800000),s>>23);
        if((int64_t)got!=want){fprintf(stderr,"reciprocal %08x: %u want %d\n",s,got,want);check(false,"reciprocal");}
        ++n;
    }
    check(n==UINT64_C(33554433),"reciprocal count");
    printf("reciprocal: all %llu accepted scales exact\n",(unsigned long long)n);
}

/* trunc(2^30 * b): every pattern of eight exponents in full, random
 * significands for every other exponent, both signs. */
static void test_q30(void) {
    uint64_t n=0;
    static const uint32_t full[]={0,1,96,97,119,120,126,127};
    for(uint32_t sign=0;sign<2u;++sign)for(uint32_t e=0;e<128u;++e) {
        bool all=false;
        for(unsigned i=0;i<sizeof full/sizeof full[0];++i)all=all||full[i]==e;
        const uint32_t count=all?(e==127u?0x55u:UINT32_C(0x800000)):65536u;
        for(uint32_t m=0;m<count;++m) {
            const uint32_t frac=all?m:(m<4u?(m==0?0u:m==1?1u:m==2?UINT32_C(0x7FFFFF):UINT32_C(0x400000)):po_rnd()&UINT32_C(0x7FFFFF));
            const uint32_t b=sign<<31|e<<23|frac;
            if((b&UINT32_C(0x7FFFFFFF))>UINT32_C(0x3F800054))continue;
            int32_t want;
            check(po_fixed(po_float(b),1073741824.0f,1.00001f,&want),"q30 domain");
            if(pose_q30(b)!=want){fprintf(stderr,"q30 %08x: %d want %d\n",b,pose_q30(b),want);check(false,"q30");}
            ++n;
        }
    }
    printf("basis Q30: %llu values exact\n",(unsigned long long)n);
}

/* trunc(65536 * fl(b * s)): scales that occur, their neighbours, scales
 * with short significands (many exact ties) and random scales, against
 * random basis values and, for three scales, every significand of the
 * largest exponent. */
static void test_model(void) {
    float scales[96];unsigned ns=0;
    for(unsigned i=0;i<sizeof po_game_scales/sizeof po_game_scales[0];++i) {
        scales[ns++]=po_game_scales[i];
        if(po_game_scales[i]>0.0625f)scales[ns++]=po_float(po_bits(po_game_scales[i])-1u);
        if(po_game_scales[i]<1.0f)scales[ns++]=po_float(po_bits(po_game_scales[i])+1u);
    }
    static const float short_scales[]={0.5625f,0.53125f,0.625f,0.6875f,0.9375f,0.96875f,0.0703125f,0.1875f};
    for(unsigned i=0;i<sizeof short_scales/sizeof short_scales[0];++i)scales[ns++]=short_scales[i];
    while(ns<96u)scales[ns++]=(float)po_uniform(0.0625,1.0);
    uint64_t n=0,ties=0;
    for(unsigned si=0;si<ns;++si) {
        const uint32_t s=po_bits(scales[si]),ms=(s&UINT32_C(0x7FFFFF))|UINT32_C(0x800000),es=s>>23;
        const uint32_t exhaustive=si<3u?UINT32_C(0x800000):0u;
        for(uint32_t it=0;it<300000u+exhaustive;++it) {
            uint32_t b;
            if(it<exhaustive)b=UINT32_C(126)<<23|it;
            else {
                const uint32_t e=po_rnd()%4u==0u?po_rnd()%128u:88u+po_rnd()%40u;
                b=(po_rnd()&1u)<<31|e<<23|(po_rnd()&UINT32_C(0x7FFFFF));
                if(po_rnd()%4u==0u)b&=~UINT32_C(0xFFFF);  /* short significands: ties */
            }
            if((b&UINT32_C(0x7FFFFFFF))>UINT32_C(0x3F800054))continue;
            int32_t want;
            check(po_fixed(po_float(b)*scales[si],65536.0f,1.00001f,&want),"model domain");
            const int32_t got=pose_model(b,ms,es);
            if(got!=want){fprintf(stderr,"model %08x * %08x: %d want %d\n",b,s,got,want);check(false,"model");}
            const uint64_t P=(uint64_t)(((b&UINT32_C(0x7FFFFF))|UINT32_C(0x800000)))*ms;
            if((b>>23&255u) && (P&((P>>47)?UINT64_C(0xFFFFFF):UINT64_C(0x7FFFFF)))==((P>>47)?UINT64_C(0x800000):UINT64_C(0x400000)))++ties;
            ++n;
        }
    }
    check(ties>1000u,"model ties exercised");
    printf("model product: %llu values exact over %u scales, %llu exact rounding ties\n",
        (unsigned long long)n,ns,(unsigned long long)ties);
}

/* trunc(65536 * fl(a - b)) and its acceptance, over structured pairs. */
static uint32_t rand_float_bits(uint32_t emin,uint32_t emax) {
    uint32_t frac=po_rnd()&UINT32_C(0x7FFFFF);
    switch(po_rnd()%8u){case 0:frac=0;break;case 1:frac=UINT32_C(0x7FFFFF);break;case 2:frac&=~UINT32_C(0xFFF);break;default:break;}
    return (po_rnd()&1u)<<31|(emin+po_rnd()%(emax-emin+1u))<<23|frac;
}
static uint64_t delta_n,delta_ok,delta_ties;
static void delta_pair(uint32_t a,uint32_t b) {
    const float fa=po_float(a),fb=po_float(b);
    if(!po_finite(fa) || !po_finite(fb))return;
    const float diff=fa-fb;
    int32_t want=0;
    const bool ok=po_fixed(diff,65536.0f,4096.0f,&want);
    int32_t got=0x5A5A5A5A;
    const bool gok=pose_delta(a,b,&got);
    if(gok!=ok || (ok && got!=want)) {
        fprintf(stderr,"delta %08x - %08x: %d %d want %d %d\n",a,b,gok,got,ok,want);
        check(false,"delta");
    }
    /* An exact tie: the double difference sits halfway between two floats. */
    const double exact=(double)fa-(double)fb;
    if(ok && exact!=(double)diff) {
        const float other=nextafterf(diff,exact>diff?INFINITY:-INFINITY);
        if(fabs(exact-(double)diff)==fabs((double)other-exact))++delta_ties;
    }
    ++delta_n;delta_ok+=ok;
}
static void test_delta(void) {
    for(uint32_t it=0;it<6000000u;++it) {
        uint32_t a,b;
        switch(it%8u) {
        case 0:a=rand_float_bits(100,160);b=rand_float_bits(100,160);break;     /* anything near the range */
        case 1:a=rand_float_bits(127,150);b=(a&UINT32_C(0xFF800000))|(po_rnd()&UINT32_C(0x7FFFFF));break; /* same exponent */
        case 2:a=rand_float_bits(127,150);b=a+(po_rnd()%2u?1u:0u)*UINT32_C(0x800000)-(po_rnd()&UINT32_C(0x3FFF));break; /* near cancellation */
        case 3:{ /* world position, eye a few thousand units away */
            const float pa=(float)po_world();const float pb=(float)((double)pa-po_uniform(-4200,4200));
            a=po_bits(pa);b=po_bits(pb);break;}
        case 4:{ /* half an ulp of a, and its neighbours: ties and near-ties */
            a=rand_float_bits(127,139);
            const uint32_t ea=a>>23&255u;
            b=(po_rnd()&1u)<<31|(ea-24u)<<23|(po_rnd()%3u==0u?0u:po_rnd()%2u?1u:UINT32_C(0x7FFFFF));
            if(po_rnd()%2u)b=(b&UINT32_C(0x80000000))|((ea-24u-po_rnd()%3u)<<23)|(b&UINT32_C(0x7FFFFF));
            break;}
        case 5:a=rand_float_bits(0,2);b=rand_float_bits(0,2);break;              /* subnormal and tiny */
        case 6:a=rand_float_bits(127,139);b=rand_float_bits(0,127);break;        /* big minus small */
        default:a=rand_float_bits(140,254);b=(a&UINT32_C(0x7F800000))?a-(po_rnd()&UINT32_C(0x3FF)):a;break; /* huge, close */
        }
        if(po_rnd()%2u){const uint32_t t=a;a=b;b=t;}
        delta_pair(a,b);
    }
    static const uint32_t special[]={0,UINT32_C(0x80000000),1,UINT32_C(0x80000001),UINT32_C(0x7FFFFF),UINT32_C(0x800000),
        UINT32_C(0x45800000),UINT32_C(0xC5800000),UINT32_C(0x45800001),UINT32_C(0x457FFFFF),UINT32_C(0x7F7FFFFF),UINT32_C(0xFF7FFFFF),
        UINT32_C(0x3F800000),UINT32_C(0x47000000),UINT32_C(0x4B800000),UINT32_C(0x4B800001),UINT32_C(0x37800000),UINT32_C(0x37800001),
        UINT32_C(0x377FFFFF),UINT32_C(0x43800000),UINT32_C(0x437FFFFF),UINT32_C(0x3F000000)};
    const unsigned ns=sizeof special/sizeof special[0];
    for(unsigned i=0;i<ns;++i)for(unsigned j=0;j<ns;++j)for(int da=-2;da<=2;++da)for(int db=-2;db<=2;++db)
        delta_pair(special[i]+(uint32_t)da,special[j]+(uint32_t)db);
    check(delta_ok>delta_n/3u && delta_ties>10000u,"delta coverage");
    printf("difference: %llu pairs (%llu in range, %llu exact rounding ties) exact\n",
        (unsigned long long)delta_n,(unsigned long long)delta_ok,(unsigned long long)delta_ties);
}

/* ---- whole poses ------------------------------------------------------------ */

static uint64_t controls[PO_MUTATIONS];
static uint64_t pose_n,pose_ok,pose_cull_off;
static void compare_case(const po_case_t *c,const sh2z_view_t *base) {
    sh2z_pose_t want,got;
    memset(&got,0xA5,sizeof got);
    const sh2z_pose_t before=got;
    const bool ok=po_pose(&want,base,(const int32_t (*)[3])c->rotation,(const float (*)[3])c->basis,c->scale,c->position,c->eye,c->flags,PO_EXACT);
    const bool gok=sh2z_pose_from_f32(&got,base,(const int32_t (*)[3])c->rotation,(const float (*)[3])c->basis,c->scale,c->position,c->eye,c->flags);
    check(gok==ok,"pose success");
    if(ok)check(po_same(&got,&want),"pose fields");
    else check(!memcmp(&got,&before,sizeof got),"pose output unchanged on failure");
    for(int m=1;m<PO_MUTATIONS;++m) {
        sh2z_pose_t mut;
        const bool mok=po_pose(&mut,base,(const int32_t (*)[3])c->rotation,(const float (*)[3])c->basis,c->scale,c->position,c->eye,c->flags,m);
        if(mok!=ok || (ok && !po_same(&mut,&want)))++controls[m];
    }
    ++pose_n;pose_ok+=ok;pose_cull_off+=ok && !want.cull_valid;
}

/* Projection through the pose against the caller's gate: a double model of
 * the source transform, 160 * x / z from the centre (160, 112), truncated.
 * Points behind the near plane or off the clip rectangle are skipped. */
static double gate_max;
static uint64_t gate_points,gate_poses;
static void gate_case(const po_case_t *c,const sh2z_view_t *base) {
    sh2z_pose_t want,got;
    if(!po_pose(&want,base,(const int32_t (*)[3])c->rotation,(const float (*)[3])c->basis,c->scale,c->position,c->eye,c->flags,PO_EXACT))return;
    check(sh2z_pose_from_f32(&got,base,(const int32_t (*)[3])c->rotation,(const float (*)[3])c->basis,c->scale,c->position,c->eye,c->flags),"gate pose");
    sh2z_pos16_t p[32];
    sh2z_view_vertex_t a[32],b[32];
    for(unsigned i=0;i<32;++i)p[i]=(sh2z_pos16_t){(int16_t)(int32_t)(po_rnd()%2049u-1024u),(int16_t)(int32_t)(po_rnd()%2049u-1024u),(int16_t)(int32_t)(po_rnd()%2049u-1024u)};
    check(sh2z_view_batch(a,&got.view,p,sizeof p[0],32) && sh2z_view_batch(b,&want.view,p,sizeof p[0],32),"gate batch");
    check(!memcmp(a,b,sizeof a),"gate projection identical to the reference pose");
    for(unsigned i=0;i<32;++i) {
        const double local[3]={p[i].x,p[i].y,p[i].z};
        double cam[3];
        for(unsigned r=0;r<3;++r) {
            cam[r]=0;
            for(unsigned k=0;k<3;++k) {
                double world=(double)c->position[k]-(double)c->eye[k];
                for(unsigned j=0;j<3;++j) {
                    const unsigned s=(c->flags&SH2Z_POSE_REVERSE_AXES)?2u-j:j;
                    world+=local[j]*(double)c->basis[s][k]*(double)c->scale[s];
                }
                cam[r]+=(double)c->rotation[r][k]/1073741824.0*world;
            }
        }
        if(cam[2]<16.0+1.0/16)continue;
        const double x=160.0+160.0*cam[0]/cam[2],y=112.0-160.0*cam[1]/cam[2];
        if(x<12 || x>=308 || y<16 || y>=224 || (a[i].outcode&SH2Z_OUT_NEAR))continue;
        const double e=fmax(fabs(a[i].sx-trunc(x)),fabs(a[i].sy-trunc(y)));
        if(e>gate_max)gate_max=e;
        ++gate_points;
    }
    ++gate_poses;
}

static const uint32_t specials[]={
    UINT32_C(0x00000000),UINT32_C(0x80000000),UINT32_C(0x00000001),UINT32_C(0x80000001),UINT32_C(0x007FFFFF),
    UINT32_C(0x00800000),UINT32_C(0x80800000),UINT32_C(0x0DA24260),UINT32_C(0x7F800000),UINT32_C(0xFF800000),
    UINT32_C(0x7FC00000),UINT32_C(0xFFC00000),UINT32_C(0x7F800001),UINT32_C(0x7FBFFFFF),UINT32_C(0x7F7FFFFF),
    UINT32_C(0xFF7FFFFF),UINT32_C(0x3F800054),UINT32_C(0x3F800055),UINT32_C(0xBF800054),UINT32_C(0xBF800055),
    UINT32_C(0x3D800000),UINT32_C(0x3D7FFFFF),UINT32_C(0x3F800000),UINT32_C(0x3F800001),UINT32_C(0xBF800000),
    UINT32_C(0x3DCCCCCD),UINT32_C(0x30800000),UINT32_C(0x45800000),UINT32_C(0xC5800000),UINT32_C(0x4B800000),
    UINT32_C(0x46FFFE00),UINT32_C(0xC6FFFE00),UINT32_C(0x4F000000)};
#define NSPECIAL (sizeof specials/sizeof specials[0])

static void test_poses(void) {
    sh2z_view_t base;po_base(&base);
    po_case_t c;
    for(uint32_t it=0;it<1500000u;++it){po_random_case(&c);compare_case(&c,&base);}
    printf("random poses: %llu, %llu accepted (%llu with cull_valid false), all equal to the oracle\n",
        (unsigned long long)pose_n,(unsigned long long)pose_ok,(unsigned long long)pose_cull_off);
    check(pose_ok>pose_n*9u/10u && pose_cull_off>1000u,"pose coverage");
    /* Every special float in every input slot of random valid poses. */
    const uint64_t before=pose_n,ok_before=pose_ok;
    for(unsigned base_case=0;base_case<300u;++base_case) {
        po_random_case(&c);
        for(unsigned slot=0;slot<18u;++slot)for(unsigned s=0;s<NSPECIAL;++s) {
            po_case_t d=c;
            float *f=slot<9u?&d.basis[slot/3u][slot%3u]:slot<12u?&d.scale[slot-9u]:slot<15u?&d.position[slot-12u]:&d.eye[slot-15u];
            *f=po_float(specials[s]);
            compare_case(&d,&base);
            if(slot>=12u) {  /* the same value in both position and eye */
                if(slot<15u)d.eye[slot-12u]=*f;else d.position[slot-15u]=*f;
                compare_case(&d,&base);
            }
        }
    }
    printf("special floats: %llu poses, %llu accepted, all equal to the oracle\n",
        (unsigned long long)(pose_n-before),(unsigned long long)(pose_ok-ok_before));
    /* Differences at the 4096 bound and positions of any magnitude. */
    static const double far[]={0,1,4095.5,32767,1e5,8388608,16777216,1e9,1e20,3.0e38};
    for(unsigned i=0;i<sizeof far/sizeof far[0];++i)for(int sign=-1;sign<=1;sign+=2)for(int step=-3;step<=3;++step) {
        po_random_case(&c);
        const float p=(float)(sign*far[i]);
        for(unsigned k=0;k<3;++k) {
            c.position[k]=p;
            const float d=po_float(po_bits(4096.0f)+(uint32_t)step);
            c.eye[k]=(float)((double)p-(k==1?-1:1)*(double)d);
        }
        compare_case(&c,&base);
        for(unsigned k=0;k<3;++k)c.eye[k]=p;
        compare_case(&c,&base);
    }
    /* Rotation bounds and int16 overflow of view.r. */
    static const int32_t rot_edge[]={1100000000,1100000001,-1100000000,-1100000001,INT32_MIN,INT32_MAX,0};
    for(unsigned e=0;e<sizeof rot_edge/sizeof rot_edge[0];++e)for(unsigned slot=0;slot<9u;++slot) {
        po_random_case(&c);c.rotation[slot/3u][slot%3u]=rot_edge[e];compare_case(&c,&base);
    }
    for(unsigned it=0;it<20000u;++it) {
        po_random_case(&c);
        for(unsigned i=0;i<3;++i)for(unsigned k=0;k<3;++k)c.rotation[i][k]=(po_rnd()&1u?1:-1)*(int32_t)(1000000000u+po_rnd()%100000001u);
        for(unsigned a=0;a<3;++a){c.scale[a]=1.0f;for(unsigned k=0;k<3;++k)c.basis[a][k]=po_rnd()&1u?1.00001f:-1.00001f;}
        compare_case(&c,&base);
    }
    /* Base views: each field invalid in turn. */
    for(unsigned f=0;f<7u;++f) {
        sh2z_view_t bad=base;
        switch(f){case 0:bad.focal_x=0;break;case 1:bad.focal_y=-1;break;case 2:bad.near_z=0;break;
            case 3:bad.far_z=bad.near_z;break;case 4:bad.clip[0]=(int16_t)(bad.clip[2]+1);break;
            case 5:bad.clip[1]=(int16_t)(bad.clip[3]+1);break;default:bad.clip[0]=bad.clip[2];break;}
        for(unsigned it=0;it<100u;++it){po_random_case(&c);compare_case(&c,&bad);}
    }
    printf("whole poses: %llu compared, %llu accepted\n",(unsigned long long)pose_n,(unsigned long long)pose_ok);
}

static void test_arguments(void) {
    sh2z_view_t base;po_base(&base);
    po_case_t c;
    do po_random_case(&c); while(!po_pose(&(sh2z_pose_t){0},&base,(const int32_t (*)[3])c.rotation,(const float (*)[3])c.basis,c.scale,c.position,c.eye,c.flags,PO_EXACT));
    sh2z_pose_t out,want;
    check(po_pose(&want,&base,(const int32_t (*)[3])c.rotation,(const float (*)[3])c.basis,c.scale,c.position,c.eye,c.flags,PO_EXACT),"argument case");
    const int32_t (*rot)[3]=(const int32_t (*)[3])c.rotation;const float (*bas)[3]=(const float (*)[3])c.basis;
    check(!sh2z_pose_from_f32(NULL,&base,rot,bas,c.scale,c.position,c.eye,c.flags),"null out");
    check(!sh2z_pose_from_f32(&out,NULL,rot,bas,c.scale,c.position,c.eye,c.flags),"null base");
    check(!sh2z_pose_from_f32(&out,&base,NULL,bas,c.scale,c.position,c.eye,c.flags),"null rotation");
    check(!sh2z_pose_from_f32(&out,&base,rot,NULL,c.scale,c.position,c.eye,c.flags),"null basis");
    check(!sh2z_pose_from_f32(&out,&base,rot,bas,NULL,c.position,c.eye,c.flags),"null scale");
    check(!sh2z_pose_from_f32(&out,&base,rot,bas,c.scale,NULL,c.eye,c.flags),"null position");
    check(!sh2z_pose_from_f32(&out,&base,rot,bas,c.scale,c.position,NULL,c.flags),"null eye");
    check(!sh2z_pose_from_f32(&out,&base,rot,bas,c.scale,c.position,c.eye,2u),"unknown flag");
    check(!sh2z_pose_from_f32(&out,&base,rot,bas,c.scale,c.position,c.eye,~0u),"unknown flags");
    /* base may be the output's own view. */
    out.view=base;
    check(sh2z_pose_from_f32(&out,&out.view,rot,bas,c.scale,c.position,c.eye,c.flags) && po_same(&out,&want),"base aliasing out");
    /* Inputs inside the output's storage are read before it is written. */
    union {sh2z_pose_t pose;float f[24];} u;
    memset(&u,0,sizeof u);
    for(unsigned k=0;k<3;++k){u.f[k]=c.position[k];u.f[3+k]=c.eye[k];u.f[6+k]=c.scale[k];for(unsigned j=0;j<3;++j)u.f[9+3*k+j]=c.basis[k][j];}
    check(sh2z_pose_from_f32(&u.pose,&base,rot,(const float (*)[3])(u.f+9),u.f+6,u.f,u.f+3,c.flags) && po_same(&u.pose,&want),"inputs aliasing out");
    printf("arguments: null pointers, unknown flags and aliasing as documented\n");
}

int main(void) {
    test_reciprocal();
    test_q30();
    test_model();
    test_delta();
    test_poses();
    test_arguments();
    sh2z_view_t base;po_base(&base);
    po_case_t c;
    for(uint32_t it=0;it<200000u;++it){po_random_case(&c);gate_case(&c,&base);}
    printf("projection gate: %llu poses, %llu on-screen points; identical to the reference pose; max %.0f px from the double model\n",
        (unsigned long long)gate_poses,(unsigned long long)gate_points,gate_max);
    check(gate_points>1000000u && gate_max<=1.0,"projection gate");
    static const char *const names[PO_MUTATIONS]={"exact",
        "difference without binary32 rounding","product without binary32 rounding","reciprocal without binary32 rounding",
        "view floored","basis Q14 floored","Gram tolerance 511","eye without the intermediate R_30","axes not reversed"};
    for(int m=1;m<PO_MUTATIONS;++m) {
        printf("control '%s': %llu poses differ\n",names[m],(unsigned long long)controls[m]);
        check(controls[m]>0u,"control detected");
    }
    printf("pose-1: %llu checks passed\n",(unsigned long long)checks);
    return 0;
}
