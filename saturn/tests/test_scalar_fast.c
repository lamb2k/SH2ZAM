/* Exact scalar/vector regression coverage. The public comparisons use the
 * frozen pre-optimization source, plus independent int128 arithmetic and
 * integer-root properties. No floating-point tolerance is permitted.
 * MIT; Copyright (c) 2026 SH2ZAM Contributors. */
#include "../src/sh2zam_internal.h"
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>

#if !defined(__SIZEOF_INT128__)
#error "The host oracle suite requires GCC/Clang int128; the library does not."
#endif
__extension__ typedef __int128 wide_t;
__extension__ typedef unsigned __int128 uwide_t;

sh2z_fx_t legacy_sh2z_div(sh2z_fx_t,sh2z_fx_t);
sh2z_fx_t legacy_sh2z_muldiv(sh2z_fx_t,sh2z_fx_t,sh2z_fx_t);
sh2z_fx_t legacy_sh2z_sqrt(sh2z_fx_t);
sh2z_fx_t legacy_sh2z_recip(sh2z_fx_t);
sh2z_fx_t legacy_sh2z_rsqrt(sh2z_fx_t);
sh2z_vec2_t legacy_sh2z_vec2_normalize(sh2z_vec2_t);
sh2z_vec3_t legacy_sh2z_vec3_normalize(sh2z_vec3_t);
sh2z_vec4_t legacy_sh2z_vec4_normalize(sh2z_vec4_t);

static uint64_t checks;
#define CHECK(e) do { ++checks; if(!(e)) { fprintf(stderr,"FAIL %s:%d: %s\n",__FILE__,__LINE__,#e); exit(EXIT_FAILURE); } } while(0)
static int32_t sat(wide_t x) { return x>INT32_MAX ? INT32_MAX : (x<INT32_MIN ? INT32_MIN : (int32_t)x); }
static wide_t floor16(wide_t x) { return x>=0 ? x/65536 : -((-x+65535)/65536); }
static uint32_t rng=UINT32_C(0x7a32b19d);
static uint32_t random_u32(void) { rng^=rng<<13; rng^=rng>>17; rng^=rng<<5; return rng; }
static int32_t random_i32(void) {
    uint32_t x=random_u32();
    return x<=INT32_MAX ? (int32_t)x : (int32_t)((int64_t)x-INT64_C(4294967296));
}
static int64_t random_i64(void) {
    return (int64_t)random_i32()*INT64_C(4294967296)+random_u32();
}
static int32_t div_oracle(int64_t numerator,int32_t denominator) {
    return denominator ? sat((wide_t)numerator/denominator) :
        (numerator>0 ? INT32_MAX : (numerator<0 ? INT32_MIN : 0));
}
static void root_property(uint64_t x) {
    uint64_t root=sh2zi_isqrt(x);
    CHECK((uwide_t)root*root<=x);
    CHECK((uwide_t)(root+1)*(root+1)>x);
}
static void scalars(int32_t a,int32_t b,int32_t c) {
    CHECK(sh2z_div(a,b)==div_oracle((int64_t)a*65536,b));
    CHECK(sh2z_div(a,b)==legacy_sh2z_div(a,b));
    CHECK(sh2z_muldiv(a,b,c)==div_oracle((int64_t)a*b,c));
    CHECK(sh2z_muldiv(a,b,c)==legacy_sh2z_muldiv(a,b,c));
    CHECK(sh2z_recip(a)==legacy_sh2z_recip(a));
    CHECK(sh2z_sqrt(a)==legacy_sh2z_sqrt(a));
    CHECK(sh2z_rsqrt(a)==legacy_sh2z_rsqrt(a));
    CHECK(sh2z_mul(a,b)==sat(floor16((wide_t)a*b)));
}
static void vectors(const int32_t *a,const int32_t *b) {
    const sh2z_vec2_t a2={a[0],a[1]},b2={b[0],b[1]};
    const sh2z_vec3_t a3={a[0],a[1],a[2]},b3={b[0],b[1],b[2]};
    const sh2z_vec4_t a4={a[0],a[1],a[2],a[3]},b4={b[0],b[1],b[2],b[3]};
    wide_t total=(wide_t)a[0]*b[0]+(wide_t)a[1]*b[1];
    CHECK(sh2z_vec2_dot(a2,b2)==sat(floor16(total)));
    total+=(wide_t)a[2]*b[2];
    CHECK(sh2z_vec3_dot(a3,b3)==sat(floor16(total)));
    total+=(wide_t)a[3]*b[3];
    CHECK(sh2z_vec4_dot(a4,b4)==sat(floor16(total)));
    CHECK(sh2z_vec2_cross(a2,b2)==sat(floor16((wide_t)a[0]*b[1]-(wide_t)a[1]*b[0])));
    const sh2z_vec3_t cross=sh2z_vec3_cross(a3,b3);
    CHECK(cross.x==sat(floor16((wide_t)a[1]*b[2]-(wide_t)a[2]*b[1])));
    CHECK(cross.y==sat(floor16((wide_t)a[2]*b[0]-(wide_t)a[0]*b[2])));
    CHECK(cross.z==sat(floor16((wide_t)a[0]*b[1]-(wide_t)a[1]*b[0])));
    const sh2z_vec2_t n2=sh2z_vec2_normalize(a2),r2=legacy_sh2z_vec2_normalize(a2);
    const sh2z_vec3_t n3=sh2z_vec3_normalize(a3),r3=legacy_sh2z_vec3_normalize(a3);
    const sh2z_vec4_t n4=sh2z_vec4_normalize(a4),r4=legacy_sh2z_vec4_normalize(a4);
    CHECK(n2.x==r2.x && n2.y==r2.y);
    CHECK(n3.x==r3.x && n3.y==r3.y && n3.z==r3.z);
    CHECK(n4.x==r4.x && n4.y==r4.y && n4.z==r4.z && n4.w==r4.w);
    const int32_t lengths[]={sh2z_vec2_length(a2),sh2z_vec3_length(a3),sh2z_vec4_length(a4)};
    uwide_t squares=(uwide_t)((wide_t)a[0]*a[0]);
    for(unsigned i=1;i<4;i++) {
        squares+=(uwide_t)((wide_t)a[i]*a[i]);
        uint64_t length=(uint32_t)lengths[i-1];
        if(squares>=(uwide_t)INT32_MAX*INT32_MAX) CHECK(length==INT32_MAX);
        else {
            CHECK((uwide_t)length*length<=squares);
            CHECK((uwide_t)(length+1)*(length+1)>squares);
        }
    }
}
static void boundaries(void) {
    const int32_t edge[]={INT32_MIN,INT32_MIN+1,-1073741824,-65537,-65536,-32768,-1,0,1,
        32767,32768,65535,65536,65537,1073741823,1073741824,INT32_MAX-1,INT32_MAX};
    const size_t count=sizeof(edge)/sizeof(edge[0]);
    for(size_t i=0;i<count;i++) for(size_t j=0;j<count;j++) for(size_t k=0;k<count;k++)
        scalars(edge[i],edge[j],edge[k]);
    /* Exercise cancellation, all four signs, mixed scales, the vec2-dot
     * unique overflow case, and the four-square uint64 overflow. */
    for(size_t i=0;i<count;i++) for(size_t j=0;j<count;j++) {
        const int32_t a[]={edge[i],edge[j],edge[i],edge[j]};
        const int32_t b[]={edge[j],edge[i],edge[count-1-i],edge[count-1-j]};
        vectors(a,b);
        for(unsigned n=2;n<=4;n++) for(unsigned minus=0;minus<(1u<<n);minus++) {
            wide_t total=0;
            for(unsigned k=0;k<n;k++) total+=((minus>>k)&1u) ? -(wide_t)a[k]*b[k] : (wide_t)a[k]*b[k];
            CHECK(sh2zi_dot(a,b,n,minus,edge[i])==floor16(total)+edge[i]);
        }
    }
    const int64_t wide_edge[]={INT64_MIN,INT64_MIN+1,-4294967296LL,-65537,-65536,-1,0,1,65535,65536,
        4294967295LL,4294967296LL,INT64_MAX-1,INT64_MAX};
    for(size_t i=0;i<sizeof(wide_edge)/sizeof(wide_edge[0]);i++) {
        CHECK(sh2zi_floor16(wide_edge[i])==floor16(wide_edge[i]));
    }
    root_property(0); root_property(UINT64_MAX);
    for(unsigned bit=0;bit<64;bit++) {
        uint64_t n=UINT64_C(1)<<bit;
        root_property(n-1); root_property(n); root_property(n+1);
    }
    for(unsigned bit=0;bit<32;bit++) {
        uint64_t root=UINT64_C(1)<<bit;
        uint64_t square=root*root;
        root_property(square-1); root_property(square); root_property(square+1);
        square=(root+1)*(root+1);
        root_property(square-1); root_property(square); root_property(square+1);
    }
    /* Every possible fractional numerator for a representative denominator;
     * and all sixteen output-bit boundaries for the largest denominator. */
    for(uint32_t n=0;n<=65536u;n++) CHECK(sh2zi_fraction16(n,65536u)==n);
    for(uint32_t q=0;q<=65536u;q++) {
        uint32_t n=q<<15;
        CHECK(sh2zi_fraction16(n,UINT32_C(1)<<31)==q);
        if(n) CHECK(sh2zi_fraction16(n-1,UINT32_C(1)<<31)==q-1);
    }
}
int main(void) {
    boundaries();
    for(unsigned i=0;i<300000;i++) {
        int32_t a=random_i32(),b=random_i32(),c=random_i32();
        if(i%3u==1u) { a/=4096; b/=4096; c/=4096; }
        else if(i%3u==2u) { a/=65536; b/=65536; c/=65536; }
        scalars(a,b,c);
        int64_t numerator=random_i64();
        CHECK(sh2zi_floor16(numerator)==floor16(numerator));
        uint64_t bits=((uint64_t)random_u32()<<32)|random_u32();
        root_property(bits);
        uint64_t root=random_u32(),square=root*root;
        root_property(square); if(square) root_property(square-1);
        if(square<UINT64_MAX) root_property(square+1);
        int32_t va[4],vb[4];
        for(unsigned j=0;j<4;j++) {
            va[j]=random_i32(); vb[j]=random_i32();
            if(i%3u==1u) { va[j]/=4096; vb[j]/=4096; }
            else if(i%3u==2u) { va[j]/=268435456; vb[j]/=268435456; }
        }
        vectors(va,vb);
        unsigned n=2u+i%3u,minus=random_u32()&15u;
        wide_t total=0;
        for(unsigned j=0;j<n;j++) total+=((minus>>j)&1u) ? -(wide_t)va[j]*vb[j] : (wide_t)va[j]*vb[j];
        CHECK(sh2zi_dot(va,vb,n,minus,a)==floor16(total)+a);
    }
    printf("scalar/vector exact checks: %" PRIu64 "\n",checks);
    return EXIT_SUCCESS;
}
