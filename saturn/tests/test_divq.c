/* Fixed-point quotients through the divider (contract tag trig12-1):
 * sh2z_divu_q and its start/finish, q16, q30 and reciprocal forms against an
 * independent 128-bit oracle, every fraction width, boundary tables on both
 * signs, agreement with sh2z_div / sh2z_recip, and negative controls. On a
 * host these run the portable path; the SH-2 path runs in the model.
 * MIT; Copyright (c) 2026 SH2ZAM Contributors. */
#include "sh2zam/divu_experimental.h"
#include "trig12_oracle.h"
#include <stdio.h>
#include <stdlib.h>

static uint64_t checks;
static void check(bool value,const char *what) {
    ++checks;
    if(!value) {
        fprintf(stderr,"divq check %llu failed: %s\n",(unsigned long long)checks,what);
        exit(EXIT_FAILURE);
    }
}
static uint32_t state;
static uint32_t random_word(void) {state^=state<<13;state^=state>>17;state^=state<<5;return state;}

/* Operands near the fit boundary of width f, near zero, at the extremes, or
 * anything. */
static void operands(unsigned f,int32_t *num,int32_t *den) {
    const uint32_t mode=random_word()%6u;
    int32_t d=(int32_t)(random_word()>>(random_word()%32u));
    if(random_word()&1u)d=(int32_t)(0u-(uint32_t)d);
    if(mode==5)d=(int32_t)(random_word()%5u)-2;
    int32_t n;
    if(mode==0||mode==5) n=(int32_t)random_word();
    else if(mode==1) n=(int32_t)(random_word()>>(random_word()%32u));
    else if(mode==2) n=(random_word()&1u)?INT32_MIN:INT32_MAX-(int32_t)(random_word()%3u);
    else {
        /* |num| = |den| * 2^(31-f) + small: the first non-fitting value. */
        const uint64_t ad=d<0?(uint64_t)0-(uint64_t)(int64_t)d:(uint64_t)d;
        const uint64_t edge=ad<<(31u-f);
        const int64_t k=(int64_t)(random_word()%5u)-2;
        uint64_t m=(uint64_t)((int64_t)edge+k);
        if(edge==0||m>UINT64_C(0x80000000)) m=random_word()&0x7FFFFFFFu;
        n=m==UINT64_C(0x80000000)?INT32_MIN:(int32_t)m;
        if(n!=INT32_MIN&&(random_word()&1u))n=-n;
    }
    *num=n;*den=d;
}

/* The library form under test for width f. A variable f exercises the
 * generic inline on a host; the SH-2 model uses constants. */
static int32_t library(int32_t num,int32_t den,unsigned f,bool *overflow) {
    sh2z_divu_t t;
    sh2z_divu_q_start(&t,num,den,f);
    const int32_t split=sh2z_divu_finish(&t,overflow);
    check(sh2z_divu_q(num,den,f)==split,"blocking equals start/finish");
    return split;
}

static uint64_t sweep(unsigned mutation,uint64_t per_width,uint64_t *fitted) {
    uint64_t bad=0;
    for(unsigned f=0;f<32u;f++) {
        state=UINT32_C(0xd1d1d1d1)+f;
        for(uint64_t i=0;i<per_width;i++) {
            int32_t num,den;
            operands(f,&num,&den);
            bool ow=false,og=false;
            const int32_t want=t12_divq(num,den,f,&ow,mutation);
            const int32_t got=library(num,den,f,&og);
            if(got!=want||og!=ow)++bad;
            if(fitted&&!og)++*fitted;
        }
    }
    return bad;
}

int main(void) {
    /* Edge table: every row checked for f = 16 and f = 30 explicitly. */
    static const struct {int32_t num,den;unsigned f;int32_t q;bool overflow;} cases[]={
        {7,2,0,3,false},{-7,2,0,-3,false},{7,-2,0,-3,false},{-7,-2,0,3,false},
        {1,3,16,21845,false},{-1,3,16,-21845,false},{0,5,16,0,false},{0,-5,30,0,false},
        {5,0,16,INT32_MAX,true},{-5,0,16,INT32_MIN,true},{0,0,16,INT32_MAX,true},{0,0,30,INT32_MAX,true},
        {65536,65536,16,65536,false},{-65536,65536,16,-65536,false},
        {1<<30,1<<30,30,1<<30,false},{-(1<<30),1<<30,30,-(1<<30),false},
        /* |num| * 2^16 against |den| * 2^31: num = den * 2^15 is the first that saturates. */
        {32767,1,16,INT32_MAX-65535,false},{32768,1,16,INT32_MAX,true},{-32768,1,16,INT32_MIN,true},
        {-32767,1,16,-(INT32_MAX-65535),false},{65535,2,16,INT32_MAX-32767,false},{65536,2,16,INT32_MAX,true},
        /* f = 30: |num| < 2 |den| fits. */
        {3,2,30,1610612736,false},{4,2,30,INT32_MAX,true},{-4,2,30,INT32_MIN,true},{-3,-2,30,1610612736,false},
        {INT32_MIN,INT32_MIN,16,65536,false},{INT32_MIN,INT32_MIN,30,1<<30,false},
        {INT32_MIN,-1,0,INT32_MAX,true},{INT32_MIN,1,0,INT32_MIN,true},{INT32_MAX,1,0,INT32_MAX,false},
        {INT32_MIN,2,0,-(1<<30),false},{1,INT32_MIN,31,-1,false},{-1,INT32_MIN,31,1,false},
        {INT32_MAX,INT32_MIN,31,-INT32_MAX,false},{INT32_MIN,INT32_MIN,31,INT32_MAX,true},
        {-1,1,31,INT32_MIN,true},{1,2,31,1<<30,false},{-3,2,31,INT32_MIN,true},
    };
    for(unsigned i=0;i<sizeof cases/sizeof cases[0];i++) {
        bool o=!cases[i].overflow,ow=false;
        check(library(cases[i].num,cases[i].den,cases[i].f,&o)==cases[i].q,"edge quotient");
        check(o==cases[i].overflow,"edge overflow flag");
        check(t12_divq(cases[i].num,cases[i].den,cases[i].f,&ow,T12_MUT_NONE)==cases[i].q&&ow==cases[i].overflow,"oracle agrees with the edge table");
        if(cases[i].f==16u)check(sh2z_divu_q16(cases[i].num,cases[i].den)==cases[i].q,"q16 form");
        if(cases[i].f==30u)check(sh2z_divu_q30(cases[i].num,cases[i].den)==cases[i].q,"q30 form");
    }
    /* null overflow pointer */
    {sh2z_divu_t t;sh2z_divu_q_start(&t,1,0,16);check(sh2z_divu_finish(&t,NULL)==INT32_MAX,"null overflow pointer");}

    uint64_t fitted=0;
    const uint64_t per_width=125000u;
    check(sweep(T12_MUT_NONE,per_width,&fitted)==0,"every width equals the oracle");
    printf("divq sweep: %llu quotients over f = 0..31 exact, %llu on the divider path\n",
        (unsigned long long)(per_width*32u),(unsigned long long)fitted);
    check(fitted>per_width*8u&&fitted<per_width*28u,"sweep covers both paths");

    /* Named forms, reciprocals and agreement with sh2z_div / sh2z_recip. */
    state=UINT32_C(0x0dd5eed5);
    uint64_t same_div=0,zero_zero=0;
    for(unsigned i=0;i<2000000u;i++) {
        int32_t num,den;
        operands(i&1u?16u:30u,&num,&den);
        bool o=false;
        check(sh2z_divu_q16(num,den)==t12_divq(num,den,16,&o,T12_MUT_NONE),"q16 equals the oracle");
        check(sh2z_divu_q30(num,den)==t12_divq(num,den,30,&o,T12_MUT_NONE),"q30 equals the oracle");
        check(sh2z_recip_q16(den)==t12_divq(1<<16,den,16,&o,T12_MUT_NONE),"recip_q16 equals the oracle");
        check(sh2z_recip_q30(den)==t12_divq(1<<30,den,30,&o,T12_MUT_NONE),"recip_q30 equals the oracle");
        check(sh2z_recip_q16(den)==sh2z_recip(den),"recip_q16 equals sh2z_recip");
        if(num==0&&den==0) {++zero_zero;check(sh2z_div(num,den)==0,"sh2z_div 0/0");continue;}
        check(sh2z_divu_q16(num,den)==sh2z_div(num,den),"q16 equals sh2z_div");
        ++same_div;
    }
    check(sh2z_divu_q16(0,0)==INT32_MAX&&sh2z_div(0,0)==0,"0 / 0 is the one difference from sh2z_div");
    for(int32_t d=-4;d<=4;d++)check(sh2z_recip_q16(d)==sh2z_recip(d),"small reciprocals");
    check(sh2z_recip_q30((1<<29)+1)==2147483644&&sh2z_recip_q30(1<<29)==INT32_MAX&&sh2z_recip_q30(-(1<<29))==INT32_MIN,"recip_q30 domain edge");
    check(sh2z_recip_q30(1<<30)==1<<30&&sh2z_recip_q30(-(1<<30))==-(1<<30)&&sh2z_recip_q30(INT32_MIN)==-(1<<29),"recip_q30 values");
    printf("divq named forms: 2000000 operand pairs; q16 equals sh2z_div on %llu (0 / 0 drawn %llu times)\n",
        (unsigned long long)same_div,(unsigned long long)zero_zero);

    static const struct {unsigned mutation;const char *name;} controls[]={
        {T12_MUT_Q_FLOOR,"floor instead of truncation"},
        {T12_MUT_Q_FITS_EQUAL,"exact -2^31 treated as fitting"},
        {T12_MUT_Q_ZERO_ZERO,"0 / 0 = 0"},
        {T12_MUT_Q_SHIFT,"dividend num * 2^(f-1)"},
    };
    unsigned caught=0;
    for(unsigned k=0;k<sizeof controls/sizeof controls[0];k++) {
        uint64_t bad=sweep(controls[k].mutation,4000u,NULL);
        for(unsigned i=0;i<sizeof cases/sizeof cases[0];i++) {
            bool o=false;
            bad+=t12_divq(cases[i].num,cases[i].den,cases[i].f,&o,controls[k].mutation)!=cases[i].q||o!=cases[i].overflow;
        }
        printf("divq control '%s': %llu mismatches\n",controls[k].name,(unsigned long long)bad);
        check(bad>0,"negative control detected");
        caught+=bad>0;
    }
    printf("divq: %u of %u negative controls caught\n",caught,(unsigned)(sizeof controls/sizeof controls[0]));
    printf("divq checks: %llu passed\n",(unsigned long long)checks);
    return 0;
}
