/* 12-bit-angle sine, cosine and tangent (contract tag trig12-1): the table
 * against its binary64 definition, every angle against an independent
 * oracle, measured deviations from the binary64 and binary32 series tables
 * and from the true functions, and negative controls.
 * MIT; Copyright (c) 2026 SH2ZAM Contributors. */
#include "sh2zam/trig12_experimental.h"
#include "trig12_oracle.h"
#include <stdio.h>
#include <stdlib.h>

static uint64_t checks;
static void check(bool value,const char *what) {
    ++checks;
    if(!value) {
        fprintf(stderr,"trig12 check %llu failed: %s\n",(unsigned long long)checks,what);
        exit(EXIT_FAILURE);
    }
}
static uint32_t state=UINT32_C(0x5eed1201);
static uint32_t random_word(void) {state^=state<<13;state^=state>>17;state^=state<<5;return state;}

static double series[4096];   /* S(a) in binary64 at every angle */
static float series32[4096];  /* the same table rounded to binary32 */
static const double two_pi=6.283185307179586476925286766559;

static void table_against_definition(void) {
    double worst=0;
    for(int i=0;i<=1024;i++) {
        const int32_t t=sh2z_sin12_quarter_u30[i];
        const double scaled=series[i]*1073741824.0;     /* exact scaling */
        const double error=fabs((double)t-scaled);      /* exact: Sterbenz */
        check(t==t12_entry(i,T12_MUT_NONE),"entry is the nearest Q2.30 value of S(i)");
        check(error<=0.5,"entry within 0.5 LSB of S(i)");
        if(error>worst)worst=error;
        if(i)check(t>=sh2z_sin12_quarter_u30[i-1],"table nondecreasing");
    }
    check(sh2z_sin12_quarter_u30[0]==0 && sh2z_sin12_quarter_u30[1024]==SH2Z_U30_ONE,"endpoints 0 and 2^30");
    printf("trig12 table: 1025 entries equal round(S(i) * 2^30); max |entry - S(i)| = %.9f LSB\n",worst);
}

static void every_angle(void) {
    for(uint32_t a=0;a<4096u;a++) {
        const int32_t ws=t12_sin(a,T12_MUT_NONE),wc=t12_cos(a,T12_MUT_NONE);
        int32_t s=0,c=0;
        sh2z_sincos12_u30(a,&s,&c);
        check(s==ws && c==wc,"sincos12 equals the oracle");
        check(sh2z_sin12_u30(a)==ws && sh2z_cos12_u30(a)==wc,"sin12 and cos12 equal the oracle");
        check(sh2z_tan12_q16(a)==t12_tan(a,T12_MUT_NONE),"tan12 equals the oracle");
        check(sh2z_cos12_u30(a)==sh2z_sin12_u30(a+1024u),"cos(a) == sin(a + quarter)");
        check(sh2z_sin12_u30(a+2048u)==-sh2z_sin12_u30(a),"sin(a + half) == -sin(a)");
        check(sh2z_sin12_u30(4096u-a)==-sh2z_sin12_u30(a),"sin(-a) == -sin(a)");
        check(sh2z_cos12_u30(4096u-a)==sh2z_cos12_u30(a),"cos(-a) == cos(a)");
        if((a&1023u)||!(a&1024u))check(sh2z_tan12_q16(a+2048u)==sh2z_tan12_q16(a),"tan has period half a turn");
        if(a!=1024u && a!=3072u)check(sh2z_tan12_q16(4096u-a)==-sh2z_tan12_q16(a),"tan(-a) == -tan(a)");
        /* Only the low 12 bits matter, including signed angles. */
        const uint32_t high=random_word()&~UINT32_C(4095);
        int32_t s2=0,c2=0;
        sh2z_sincos12_u30(a|high,&s2,&c2);
        check(s2==ws && c2==wc && sh2z_tan12_q16(a|high)==sh2z_tan12_q16(a),"high bits ignored");
        const int32_t negative=-(int32_t)a;
        check(sh2z_sin12_u30((uint32_t)negative)==t12_sin(4096u-a,T12_MUT_NONE),"negative angles wrap");
        /* s == c: c is written last. */
        int32_t both=0;
        sh2z_sincos12_u30(a,&both,&both);
        check(both==wc,"aliased outputs hold the cosine");
    }
    static const struct {uint32_t a;int32_t s,c,t;} fixed[]={
        {0,0,SH2Z_U30_ONE,0},{1024,SH2Z_U30_ONE,0,INT32_MAX},{2048,0,-SH2Z_U30_ONE,0},
        {3072,-SH2Z_U30_ONE,0,INT32_MIN},{512,759250125,759250125,65536},
        {1536,759250125,-759250125,-65536},{UINT32_MAX,-1647099,1073740561,-101},
        {1023,1073740561,1647099,42722788},{1025,1073740561,-1647099,-42722788},
    };
    for(unsigned i=0;i<sizeof fixed/sizeof fixed[0];i++) {
        int32_t s=0,c=0;
        sh2z_sincos12_u30(fixed[i].a,&s,&c);
        check(s==fixed[i].s && c==fixed[i].c && sh2z_tan12_q16(fixed[i].a)==fixed[i].t,"fixed values");
    }
}

/* Deviation of sin12 (cos12 is the same values shifted a quarter) from the
 * binary64 table, the binary32 table and the true sine, by quadrant, in LSB
 * of Q2.30. */
static void sine_deviation(void) {
    double d64[4]={0},d32[4]={0},dtrue[4]={0},cos64=0,cos32=0;
    unsigned over_one=0;
    for(uint32_t a=0;a<4096u;a++) {
        const double v=(double)sh2z_sin12_u30(a),w=(double)sh2z_cos12_u30(a);
        const unsigned q=a>>10;
        const double e64=fabs(v-series[a]*1073741824.0),e32=fabs(v-(double)series32[a]*1073741824.0);
        const double et=fabs(v-sin(two_pi*a/4096.0)*1073741824.0);
        if(e64>d64[q])d64[q]=e64;
        if(e32>d32[q])d32[q]=e32;
        if(et>dtrue[q])dtrue[q]=et;
        if(e64>1.0)++over_one;
        const double c64=fabs(w-series[(a+1024u)&4095u]*1073741824.0);
        const double c32=fabs(w-(double)series32[(a+1024u)&4095u]*1073741824.0);
        if(c64>cos64)cos64=c64;
        if(c32>cos32)cos32=c32;
    }
    printf("trig12 sin deviation, LSB of Q2.30, quadrants 0..3:\n");
    printf("  from the binary64 series table: %.4f %.4f %.4f %.4f\n",d64[0],d64[1],d64[2],d64[3]);
    printf("  from the binary32 series table: %.4f %.4f %.4f %.4f\n",d32[0],d32[1],d32[2],d32[3]);
    printf("  from sin(2 pi a / 4096):        %.4f %.4f %.4f %.4f\n",dtrue[0],dtrue[1],dtrue[2],dtrue[3]);
    printf("  cos max from binary64 %.4f, from binary32 %.4f; angles more than 1 LSB from binary64: %u\n",cos64,cos32,over_one);
    check(d64[0]<=0.5,"quadrant 0 within 0.5 LSB of the binary64 table");
    check(d64[1]<=0.59 && d64[2]<=0.6,"quadrants 1 and 2 within 0.6 LSB");
    check(d64[3]<=332.2 && cos64<=332.2,"near a full turn within 332.2 LSB");
    check(d32[0]<=32.0 && d32[1]<=32.0 && d32[2]<=32.0 && d32[3]<=332.2 && cos32<=332.2,"binary32 bounds");
    for(unsigned q=0;q<4;q++)check(dtrue[q]<=0.514,"within 0.514 LSB of the true sine");
}

/* tan12 deviation in LSB of Q16.16, by distance from the nearest zero of the
 * tangent (|t| in 12-bit units), from the exact quotient of the two table
 * values, the binary64 series quotient, the binary32 quotient (binary32
 * division of binary32 values) and the true tangent. */
static void tangent_deviation(void) {
    static const unsigned limits[]={512,683,910,967,1012,1014,1023};
    static const char *const labels[]={"45","60","80","85","89","89.12","89.91"};
    enum {R=sizeof limits/sizeof limits[0]};
    double dq[R]={0},d64[R]={0},d32[R]={0},dt[R]={0};
    for(uint32_t a=0;a<4096u;a++) {
        if(a==1024u||a==3072u)continue;
        const int32_t h=(int32_t)(a&2047u),t=h>=1024?2048-h:h;
        const double v=(double)sh2z_tan12_q16(a);
        const double s=(double)sh2z_sin12_u30(a),c=(double)sh2z_cos12_u30(a);
        const double eq=fabs(v-s/c*65536.0);
        const double e64=fabs(v-series[a]/series[(a+1024u)&4095u]*65536.0);
        const float f=series32[a]/series32[(a+1024u)&4095u];
        const double e32=fabs(v-(double)f*65536.0);
        const double et=fabs(v-tan(two_pi*a/4096.0)*65536.0);
        for(unsigned r=0;r<R;r++)if((unsigned)t<=limits[r]) {
            if(eq>dq[r])dq[r]=eq;
            if(e64>d64[r])d64[r]=e64;
            if(e32>d32[r])d32[r]=e32;
            if(et>dt[r])dt[r]=et;
        }
    }
    printf("trig12 tan deviation, LSB of Q16.16, for |angle| up to (degrees):\n  ");
    for(unsigned r=0;r<R;r++)printf("%10s",labels[r]);
    printf("\n  table quotient: ");for(unsigned r=0;r<R;r++)printf(" %9.4f",dq[r]);
    printf("\n  binary64 table: ");for(unsigned r=0;r<R;r++)printf(" %9.4f",d64[r]);
    printf("\n  binary32 table: ");for(unsigned r=0;r<R;r++)printf(" %9.4f",d32[r]);
    printf("\n  true tangent:   ");for(unsigned r=0;r<R;r++)printf(" %9.4f",dt[r]);
    printf("\n");
    check(dq[R-1]<=0.5000001,"tan within 0.5 LSB of the table quotient");
    check(dt[R-2]<=0.5 && dt[R-1]<=8.3,"tan within 0.5 LSB of the true tangent to 89.12 degrees, 8.3 overall");
}

static unsigned differences(unsigned mutation) {
    unsigned n=0;
    for(uint32_t a=0;a<4096u;a++) {
        int32_t s=0,c=0;
        sh2z_sincos12_u30(a,&s,&c);
        n+=s!=t12_sin(a,mutation);
        n+=c!=t12_cos(a,mutation);
        n+=sh2z_tan12_q16(a)!=t12_tan(a,mutation);
    }
    return n;
}
static void negative_controls(void) {
    static const struct {unsigned mutation;const char *name;} controls[]={
        {T12_MUT_TABLE_TRUNC,"table truncated instead of rounded"},
        {T12_MUT_TABLE_PI,"series at the libm pi"},
        {T12_MUT_TABLE_B32,"table from the binary32 values"},
        {T12_MUT_COS_BACKWARD,"cos(a) as sin(a - quarter)"},
        {T12_MUT_SIGN_BIT,"sine sign from the quarter bit"},
        {T12_MUT_TAN_TRUNC,"tangent truncated"},
        {T12_MUT_TAN_POLE,"poles saturate the other way"},
    };
    unsigned caught=0;
    for(unsigned k=0;k<sizeof controls/sizeof controls[0];k++) {
        const unsigned n=differences(controls[k].mutation);
        printf("trig12 control '%s': %u results differ\n",controls[k].name,n);
        check(n>0,"negative control detected");
        caught+=n>0;
    }
    printf("trig12: %u of %u negative controls caught\n",caught,(unsigned)(sizeof controls/sizeof controls[0]));
}

int main(void) {
    for(int i=0;i<4096;i++) {series[i]=t12_series(i,T12_MUT_NONE);series32[i]=(float)series[i];}
    check(differences(T12_MUT_NONE)==0,"unmutated oracle agrees");
    table_against_definition();
    every_angle();
    sine_deviation();
    tangent_deviation();
    negative_controls();
    printf("trig12 checks: %llu passed\n",(unsigned long long)checks);
    return 0;
}
