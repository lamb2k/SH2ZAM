/* Independent wide-oracle tests for the automatic batch kernel and fallback.
 * MIT; Copyright (c) 2026 SH2ZAM Contributors. */
#include "sh2zam/sh2zam.h"
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if !defined(__SIZEOF_INT128__)
#error "The host oracle suite requires GCC/Clang int128; the library does not."
#endif
__extension__ typedef __int128 wide_t;
static unsigned long checks;
#define CHECK(e) do { ++checks; if(!(e)) { fprintf(stderr,"FAIL %s:%d: %s\n",__FILE__,__LINE__,#e); exit(1); } } while(0)

static int32_t oracle_row(const int32_t row[4],sh2z_vec3_t v) {
    const wide_t sum=(wide_t)row[0]*v.x+(wide_t)row[1]*v.y+(wide_t)row[2]*v.z;
    /* C truncates toward zero; explicitly correct negative fractional values. */
    wide_t q=sum/65536;
    if(sum<0 && sum%65536) --q;
    q+=row[3];
    return q>INT32_MAX?INT32_MAX:(q<INT32_MIN?INT32_MIN:(int32_t)q);
}
static sh2z_vec3_t oracle(const sh2z_mat34_t *m,sh2z_vec3_t v) {
    return (sh2z_vec3_t){oracle_row(m->m[0],v),oracle_row(m->m[1],v),oracle_row(m->m[2],v)};
}
static void equal(sh2z_vec3_t a,sh2z_vec3_t b) {
    CHECK(a.x==b.x); CHECK(a.y==b.y); CHECK(a.z==b.z);
}

enum { CAPACITY=37 };
static const sh2z_vec3_t sentinel={123456789,-987654321,192837465};

/* Offset arrays by one vertex: retain natural 4-byte alignment, but exercise
 * buffers that are not cache-line or 16-byte aligned and odd batch tails. */
static void check_batch(const sh2z_mat34_t *m,const sh2z_vec3_t *in,size_t count) {
    sh2z_vec3_t optimized[CAPACITY+2],general[CAPACITY+2],inplace[CAPACITY+2];
    CHECK(count<=CAPACITY);
    for(size_t i=0;i<CAPACITY+2;i++) optimized[i]=general[i]=inplace[i]=sentinel;
    CHECK(sh2z_mat34_transform_points(optimized+1,m,in,count));
    CHECK(sh2z_mat34_transform_points_general(general+1,m,in,count));
    for(size_t i=0;i<count;i++) {
        const sh2z_vec3_t expected=oracle(m,in[i]);
        equal(optimized[i+1],expected);
        equal(general[i+1],expected);
        equal(sh2z_mat34_transform_point(m,in[i]),expected);
        inplace[i+1]=in[i];
    }
    equal(optimized[0],sentinel); equal(general[0],sentinel);
    equal(optimized[count+1],sentinel); equal(general[count+1],sentinel);
    CHECK(sh2z_mat34_transform_points(inplace+1,m,inplace+1,count));
    for(size_t i=0;i<count;i++) equal(inplace[i+1],optimized[i+1]);
    equal(inplace[0],sentinel); equal(inplace[count+1],sentinel);
    for(size_t i=0;i<count;i++) inplace[i+1]=in[i];
    CHECK(sh2z_mat34_transform_points_general(inplace+1,m,inplace+1,count));
    for(size_t i=0;i<count;i++) equal(inplace[i+1],optimized[i+1]);
    equal(inplace[0],sentinel); equal(inplace[count+1],sentinel);
}

static void contracts(void) {
    sh2z_mat34_t m=sh2z_mat34_identity();
    sh2z_vec3_t in={1,2,3},out=sentinel;
    CHECK(sh2z_mat34_transform_points(NULL,NULL,NULL,0));
    CHECK(sh2z_mat34_transform_points_general(NULL,NULL,NULL,0));
    CHECK(!sh2z_mat34_transform_points(NULL,&m,&in,1));
    CHECK(!sh2z_mat34_transform_points(&out,NULL,&in,1));
    CHECK(!sh2z_mat34_transform_points(&out,&m,NULL,1));
    equal(out,sentinel);
    CHECK(!sh2z_mat34_transform_points_general(NULL,&m,&in,1));
    CHECK(!sh2z_mat34_transform_points_general(&out,NULL,&in,1));
    CHECK(!sh2z_mat34_transform_points_general(&out,&m,NULL,1));
    equal(out,sentinel);
    check_batch(&m,&in,0); check_batch(&m,&in,1);
}

static void boundaries(void) {
    static const int32_t edges[]={INT32_MIN,INT32_MIN+1,-65537,-65536,-32769,-32768,-1,
        0,1,32767,32768,65535,65536,65537,INT32_MAX-1,INT32_MAX};
    sh2z_vec3_t in[CAPACITY];
    /* The first row is at the fast-path gate exactly; the next matrices cross
     * it by one raw unit, then force multiple products beyond signed int64_t.
     * Identical edge coordinates cover cancellation, saturation and INT32_MIN. */
    const sh2z_mat34_t matrices[]={
        {{{INT32_MIN,INT32_MAX,0,INT32_MIN},{INT32_MAX,0,INT32_MIN,INT32_MAX},{0,INT32_MIN,INT32_MAX,0}}},
        {{{INT32_MIN,INT32_MAX,1,0},{1,0,0,-1},{0,1,0,1}}},
        {{{INT32_MIN,INT32_MIN,INT32_MIN,INT32_MAX},{INT32_MIN,INT32_MIN,INT32_MAX,0},{INT32_MAX,INT32_MAX,INT32_MAX,INT32_MIN}}},
        {{{1,1,1,0},{-1,-1,-1,0},{1,-1,1,-1}}},
        {{{SH2Z_ONE,0,0,INT32_MAX},{0,SH2Z_ONE,0,INT32_MIN},{0,0,SH2Z_ONE,0}}},
        {{{-SH2Z_ONE,0,0,-1},{0,-SH2Z_ONE,0,1},{0,0,-SH2Z_ONE,0}}},
        {{{0,0,0,INT32_MAX},{0,0,0,INT32_MIN},{0,0,0,-1}}}
    };
    for(size_t n=0;n<sizeof(matrices)/sizeof(matrices[0]);n++) {
        for(size_t x=0;x<sizeof(edges)/sizeof(edges[0]);x++) {
            for(size_t y=0;y<sizeof(edges)/sizeof(edges[0]);y++) {
                for(size_t z=0;z<sizeof(edges)/sizeof(edges[0]);z++)
                    in[z]=(sh2z_vec3_t){edges[x],edges[y],edges[z]};
                check_batch(&matrices[n],in,sizeof(edges)/sizeof(edges[0]));
            }
        }
    }
    /* Half-LSB products accumulate before the one floor operation. */
    const sh2z_mat34_t fractional={{{1,1,1,0},{-1,-1,-1,0},{1,1,1,7}}};
    sh2z_vec3_t input={32768,32768,32768},out;
    CHECK(sh2z_mat34_transform_points(&out,&fractional,&input,1));
    equal(out,(sh2z_vec3_t){1,-2,8});
    input=(sh2z_vec3_t){-1,-1,-1};
    CHECK(sh2z_mat34_transform_points(&out,&fractional,&input,1));
    equal(out,(sh2z_vec3_t){-1,0,6});

    /* Put the over-bound row in every position, including the final row. */
    for(unsigned row=0;row<3;row++) {
        sh2z_mat34_t m=sh2z_mat34_identity();
        m.m[row][0]=INT32_MIN; m.m[row][1]=INT32_MIN; m.m[row][2]=INT32_MAX;
        for(size_t i=0;i<CAPACITY;i++) in[i]=(sh2z_vec3_t){INT32_MIN,INT32_MIN,INT32_MIN};
        check_batch(&m,in,CAPACITY);
    }
}

static uint32_t rng=UINT32_C(0xc2d1b479);
static uint32_t random_u32(void) { rng^=rng<<13; rng^=rng>>17; rng^=rng<<5; return rng; }
static int32_t random_i32(void) {
    const uint32_t v=random_u32();
    return v<=INT32_MAX?(int32_t)v:(int32_t)((int64_t)v-INT64_C(4294967296));
}
static void randomized(void) {
    sh2z_mat34_t m;
    sh2z_vec3_t in[CAPACITY];
    for(unsigned pass=0;pass<5000;pass++) {
        for(unsigned row=0;row<3;row++) for(unsigned col=0;col<4;col++) {
            m.m[row][col]=random_i32();
            /* Full-range matrices exercise fallback; bounded matrices include
             * non-saturating geometry as well as arbitrary full-range inputs. */
            if(col<3 && pass%3) m.m[row][col]/=4096;
            if(col==3 && pass%3==2) m.m[row][col]/=8192;
        }
        for(size_t i=0;i<CAPACITY;i++) {
            in[i]=(sh2z_vec3_t){random_i32(),random_i32(),random_i32()};
            if(pass%3==2) { in[i].x/=4096; in[i].y/=4096; in[i].z/=4096; }
        }
        check_batch(&m,in,1+pass%CAPACITY);
    }
}

int main(void) {
    contracts(); boundaries(); randomized();
    printf("batch transform: %lu checks; general/optimized/int128 oracle are bit-identical\n",checks);
    return 0;
}
