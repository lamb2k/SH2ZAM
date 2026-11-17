/* Independent oracle and negative controls for the explicit divider helpers.
 * On a host these exercise the portable results; the SH-2 hardware path is
 * exercised by the SH-2 model capture. MIT; SH2ZAM Contributors. */
#include "sh2zam/divu_experimental.h"
#include <stdio.h>
#include <stdlib.h>

__extension__ typedef __int128 wide_t;

static uint64_t checks;
static uint32_t state=UINT32_C(0xd1d1d1d1);
static void check(bool value,const char *what) {
    ++checks;
    if(!value) {
        fprintf(stderr,"divu check %llu failed: %s\n",(unsigned long long)checks,what);
        exit(EXIT_FAILURE);
    }
}
static uint32_t random_word(void) {state^=state<<13;state^=state>>17;state^=state<<5;return state;}

enum { MUT_NONE=0, MUT_FLOOR=1, MUT_BOUNDARY_FITS=2 };
/* The rule from divu_experimental.h, from magnitudes, in 128 bits. */
static int32_t oracle(int64_t n,int32_t d,bool *overflow,unsigned mutation) {
    const wide_t a=n<0?-(wide_t)n:(wide_t)n,ad=d<0?-(wide_t)d:(wide_t)d;
    const wide_t limit=ad*((wide_t)1<<31);
    *overflow=d==0 || ((mutation&MUT_BOUNDARY_FITS)?a>limit:a>=limit);
    if(d==0)return n>=0?INT32_MAX:INT32_MIN;
    wide_t q=((n<0)!=(d<0)?-1:1)*(a/ad);
    if((mutation&MUT_FLOOR) && (n<0)!=(d<0) && a%ad!=0)--q;
    return q>INT32_MAX?INT32_MAX:q<INT32_MIN?INT32_MIN:(int32_t)q;
}
static int64_t random_dividend(int32_t *d) {
    const uint32_t mode=random_word()%4u;
    *d=(int32_t)(random_word()>>(random_word()%32u));
    if(random_word()&1u)*d=(int32_t)(0u-(uint32_t)*d);
    if(mode==0) {      /* anything */
        return (int64_t)(((uint64_t)random_word()<<32)|random_word());
    }
    /* q*d + r near the fit boundary or well inside it. */
    const int64_t q=mode==1?(int64_t)(int32_t)random_word():
        mode==2?(int64_t)INT32_MAX-(random_word()%4u):-(int64_t)INT32_MAX-1+(random_word()%4u);
    const int64_t r=*d?(int64_t)(random_word()%(uint32_t)(*d<0?0u-(uint32_t)*d:(uint32_t)*d)):0;
    return q*(int64_t)*d+((random_word()&1u)?r:-r);
}

static uint64_t sweep(unsigned mutation,uint64_t *fits) {
    state=UINT32_C(0xd1d1d1d1);
    uint64_t bad=0;
    for(unsigned i=0;i<2000000u;++i) {
        int32_t d;
        const int64_t n=random_dividend(&d);
        bool ow,og,os;
        const int32_t want=oracle(n,d,&ow,mutation);
        const int32_t got=sh2z_divu_div64(n,d,&og);
        sh2z_divu_t t;
        sh2z_divu_start64(&t,(int32_t)(uint32_t)((uint64_t)n>>32),(uint32_t)(uint64_t)n,d);
        const int32_t split=sh2z_divu_finish(&t,&os);
        if(got!=want || og!=ow || split!=got || os!=og)++bad;
        if(fits && !og)++*fits;
    }
    return bad;
}

int main(void) {
    struct { int64_t n; int32_t d; int32_t q; bool overflow; } cases[]={
        {100,7,14,false},{-100,7,-14,false},{100,-7,-14,false},{-100,-7,14,false},
        {0,5,0,false},{5,0,INT32_MAX,true},{-5,0,INT32_MIN,true},{0,0,INT32_MAX,true},
        {((int64_t)7<<31)-1,7,INT32_MAX,false},{(int64_t)7<<31,7,INT32_MAX,true},
        {-((int64_t)7<<31)+1,7,-INT32_MAX,false},
        {-((int64_t)7<<31),7,INT32_MIN,true},          /* exact -2^31: exact value, flagged */
        {INT64_MIN,1,INT32_MIN,true},{INT64_MIN,-1,INT32_MAX,true},
        {INT64_MAX,INT32_MAX,INT32_MAX,true},{(int64_t)1<<62,INT32_MIN,INT32_MIN,true},
        {-1,INT32_MIN,0,false},{(int64_t)INT32_MIN,-1,INT32_MAX,true},
    };
    for(unsigned i=0;i<sizeof(cases)/sizeof(cases[0]);++i) {
        bool overflow=!cases[i].overflow;
        check(sh2z_divu_div64(cases[i].n,cases[i].d,&overflow)==cases[i].q,"table quotient");
        check(overflow==cases[i].overflow,"table overflow flag");
        check(sh2z_divu_div64(cases[i].n,cases[i].d,NULL)==cases[i].q,"null overflow pointer");
        bool o2;
        check(oracle(cases[i].n,cases[i].d,&o2,MUT_NONE)==cases[i].q && o2==cases[i].overflow,"oracle agrees with table");
    }
    /* fits is exact at the boundary on both signs and both divisor signs. */
    /* 7 * 2^31 is hi 3, lo 0x80000000. */
    check(sh2z_divu_fits(3,UINT32_C(0x7FFFFFFF),7) && !sh2z_divu_fits(3,UINT32_C(0x80000000),7),"positive boundary");
    check(sh2z_divu_fits(-4,UINT32_C(0x80000001),-7) && !sh2z_divu_fits(-4,UINT32_C(0x80000000),-7),"negative boundary");
    /* save/restore are no-ops on a host but must be callable. */
    sh2z_divu_state_t s;sh2z_divu_save(&s);sh2z_divu_restore(&s);
    check(s.dvsr==0 && s.dvdnth==0 && s.dvdntl==0,"portable save is zero");

    uint64_t fits=0;
    check(sweep(MUT_NONE,&fits)==0,"helpers equal the oracle");
    printf("divu sweep: 2000000 divisions exact, %llu sent to the hardware path\n",(unsigned long long)fits);
    check(fits>500000u && fits<1900000u,"sweep covers both paths");
    const unsigned mutations[2]={MUT_FLOOR,MUT_BOUNDARY_FITS};
    const char *names[2]={"floor instead of trunc","exact -2^31 treated as fitting"};
    for(unsigned m=0;m<2;++m) {
        uint64_t bad=sweep(mutations[m],NULL);
        bool o;
        bad+=oracle(-((int64_t)7<<31),7,&o,mutations[m])!=INT32_MIN || o!=true;
        printf("control '%s': %llu mismatches\n",names[m],(unsigned long long)bad);
        check(bad>0,"mutated oracle is detected");
    }
    printf("divu checks: %llu passed\n",(unsigned long long)checks);
    return 0;
}
