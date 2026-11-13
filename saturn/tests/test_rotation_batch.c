/* Exact shared-quaternion batch contracts and full-range output checks.
 * Host execution does not measure or execute the native MAC.L kernel.
 * MIT; Copyright (c) 2026 SH2ZAM Contributors. */
#include "sh2zam/rotation_batch.h"
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if !defined(__SIZEOF_INT128__)
#error "The test oracle needs signed128; the library does not."
#endif
__extension__ typedef __int128 wide_t;
static unsigned long checks;
#define CHECK(e) do { ++checks; if(!(e)) { \
    fprintf(stderr,"FAIL %s:%d: %s\n",__FILE__,__LINE__,#e);exit(1); \
} } while(0)
static uint32_t state=UINT32_C(0x45fa7329);
static uint32_t random_word(void) {
    state^=state<<13;state^=state>>17;state^=state<<5;return state;
}
static int32_t signed_word(uint32_t value) {
    return value<=INT32_MAX?(int32_t)value:
        INT32_MIN+(int32_t)(value-UINT32_C(0x80000000));
}
static int32_t component(unsigned profile) {
    const uint32_t value=random_word();
    if(profile==0) return (int32_t)(value%2097153u)-1048576;
    if(profile==1) return (int32_t)(value%65u)-32;
    return signed_word(value);
}
static const sh2z_vec3_t sentinel={INT32_C(0x134579bd),-123456789,987654321};
static void same(sh2z_vec3_t a,sh2z_vec3_t b) {
    CHECK(a.x==b.x);CHECK(a.y==b.y);CHECK(a.z==b.z);
}
static int32_t row_oracle(const int32_t row[4],sh2z_vec3_t v) {
    wide_t raw=(wide_t)row[0]*v.x+(wide_t)row[1]*v.y+(wide_t)row[2]*v.z;
    wide_t result=raw/65536;
    if(raw<0&&raw%65536) --result;
    result+=row[3];
    return result>INT32_MAX?INT32_MAX:result<INT32_MIN?INT32_MIN:(int32_t)result;
}
typedef bool (*batch_fn)(sh2z_vec3_t*,sh2z_quat_t,const sh2z_vec3_t*,size_t);
static const batch_fn functions[]={sh2z_quat_rotate_vectors,sh2z_quat_rotate_vectors_mac};

static void contracts(void) {
    const sh2z_quat_t q={32768,-23456,34567,56789};
    sh2z_vec3_t buffer[12],saved[12];
    for(size_t i=0;i<12;i++) buffer[i]=sentinel;
    memcpy(saved,buffer,sizeof buffer);
    for(size_t f=0;f<sizeof functions/sizeof functions[0];f++) {
        CHECK(functions[f](NULL,q,NULL,0));
        CHECK(functions[f](buffer,q,NULL,0));
        CHECK(functions[f](NULL,q,buffer,0));
        CHECK(!functions[f](NULL,q,buffer,1));
        CHECK(!functions[f](buffer,q,NULL,1));
        CHECK(!functions[f](buffer,q,buffer,SIZE_MAX/sizeof(*buffer)+1));
        CHECK(!functions[f](buffer,q,buffer+1,4));
        CHECK(!functions[f](buffer+1,q,buffer,4));
        CHECK(!memcmp(saved,buffer,sizeof buffer));
        /* Adjacent half-open regions do not overlap. */
        CHECK(functions[f](buffer+3,q,buffer,3));
        for(size_t i=0;i<3;i++) same(buffer[i+3],sh2z_quat_rotate(q,saved[i]));
        memcpy(buffer,saved,sizeof buffer);
    }
}

enum { CAPACITY=512 };
static void check_batch(sh2z_quat_t q,unsigned profile,size_t count) {
    static const sh2z_vec3_t edges[]={
        {0,0,0},{1,-1,2},{INT32_MIN,INT32_MAX,INT32_MIN},
        {INT32_MAX,INT32_MIN,INT32_MAX},{65535,-65537,32768},
        {-32768,65536,-65536},{INT32_MIN,0,0},{0,INT32_MIN,0},
        {0,0,INT32_MIN},{INT32_MAX,INT32_MAX,INT32_MAX}
    };
    sh2z_vec3_t input[CAPACITY],saved[CAPACITY],expected[CAPACITY];
    sh2z_vec3_t output[CAPACITY+2],inplace[CAPACITY+2];
    const sh2z_mat34_t matrix=sh2z_quat_to_mat34(q);
    for(size_t i=0;i<count;i++) {
        input[i]=(sh2z_vec3_t){component(profile),component(profile),component(profile)};
        if(profile==2&&i<sizeof edges/sizeof edges[0]) input[i]=edges[i];
        expected[i]=sh2z_quat_rotate(q,input[i]);
        const sh2z_vec3_t wide={row_oracle(matrix.m[0],input[i]),
            row_oracle(matrix.m[1],input[i]),row_oracle(matrix.m[2],input[i])};
        same(expected[i],wide);
    }
    memcpy(saved,input,count*sizeof(*input));
    for(size_t f=0;f<sizeof functions/sizeof functions[0];f++) {
        for(size_t i=0;i<count+2;i++) output[i]=inplace[i]=sentinel;
        memcpy(inplace+1,input,count*sizeof(*input));
        CHECK(functions[f](output+1,q,input,count));
        CHECK(functions[f](inplace+1,q,inplace+1,count));
        CHECK(!memcmp(input,saved,count*sizeof(*input)));
        for(size_t i=0;i<count;i++) {
            same(output[i+1],expected[i]);same(inplace[i+1],expected[i]);
        }
        same(output[0],sentinel);same(output[count+1],sentinel);
        same(inplace[0],sentinel);same(inplace[count+1],sentinel);
    }
}

static void edge_quaternions(void) {
    static const sh2z_quat_t q[]={
        {0,0,0,0},{0,0,0,65536},{0,0,0,-65536},{0,0,0,INT32_MIN},
        {1,0,0,0},{0,-1,0,0},{0,0,INT32_MIN,0},{1,1,1,1},
        {46341,0,0,46341},{-46341,0,0,46341},{16384,-32768,65536,131072},
        {INT32_MIN,INT32_MIN,INT32_MIN,INT32_MIN},
        {INT32_MAX,INT32_MIN,INT32_MAX,INT32_MIN},
        {INT32_MAX,INT32_MAX,INT32_MAX,INT32_MAX}
    };
    static const size_t counts[]={1,8,32,128,512};
    for(size_t i=0;i<sizeof q/sizeof q[0];i++) for(unsigned profile=0;profile<3;profile++)
        for(size_t n=0;n<sizeof counts/sizeof counts[0];n++) check_batch(q[i],profile,counts[n]);
}

static void random_quaternions(void) {
    for(unsigned pass=0;pass<1800;pass++) {
        const unsigned profile=pass%3;
        sh2z_quat_t q={component(profile),component(profile),component(profile),component(profile)};
        check_batch(q,profile,1u+pass%37u);
    }
}

int main(void) {
    contracts();edge_quaternions();random_quaternions();
    printf("rotation batch: %lu exact/contract checks; host fallback, not native timing\n",checks);
    return 0;
}
