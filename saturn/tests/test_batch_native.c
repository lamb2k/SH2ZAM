/* Host instruction-model checks plus experimental-wrapper contract tests.
 * This is not execution or timing evidence for SH-2 assembly.
 * MIT; Copyright (c) 2026 SH2ZAM Contributors. */
#include "sh2zam/batch_experimental.h"
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

static uint32_t state=UINT32_C(0x53af071b);
static uint32_t random_u32(void) {
    state^=state<<13; state^=state>>17; state^=state<<5; return state;
}
static int32_t signed_word(uint32_t value) {
    return value<=INT32_MAX?(int32_t)value:(int32_t)((int64_t)value-INT64_C(4294967296));
}
static int32_t random_i32(void) { return signed_word(random_u32()); }
static wide_t magnitude(int32_t value) { return value<0?-(wide_t)value:value; }
static wide_t floor16(wide_t value) {
    wide_t result=value/65536;
    if(value<0 && value%65536) --result;
    return result;
}
static int32_t oracle(const int32_t row[4],sh2z_vec3_t point) {
    wide_t result=floor16((wide_t)row[0]*point.x+(wide_t)row[1]*point.y+(wide_t)row[2]*point.z)+row[3];
    return result>INT32_MAX?INT32_MAX:(result<INT32_MIN?INT32_MIN:(int32_t)result);
}
static int32_t xtrct(uint32_t hi,uint32_t lo) {
    return signed_word((hi<<16)|(lo>>16));
}
static void check_extract(int64_t raw) {
    const uint64_t bits=(uint64_t)raw;
    CHECK(xtrct((uint32_t)(bits>>32),(uint32_t)bits)==floor16(raw));
}
static void extraction_edges(void) {
    static const int64_t edges[]={
        -INT64_C(140737488355328),-INT64_C(140737488355327),
        -INT64_C(4294967297),-INT64_C(4294967296),-INT64_C(4294967295),
        -65537,-65536,-65535,-32768,-1,0,1,32768,65535,65536,65537,
        INT64_C(4294967295),INT64_C(4294967296),INT64_C(4294967297),
        INT64_C(140737488355326),INT64_C(140737488355327)
    };
    for(size_t i=0;i<sizeof(edges)/sizeof(edges[0]);i++) check_extract(edges[i]);
    for(unsigned i=0;i<100000;i++) {
        const uint64_t raw=((uint64_t)(random_u32()&UINT32_C(0xffff))<<32)|random_u32();
        const int64_t value=raw<UINT64_C(140737488355328)?(int64_t)raw:
            (int64_t)raw-INT64_C(281474976710656);
        check_extract(value);
    }
}

/* Model the exact register operations, independently of the portable dot.
 * Keep the low-word carry explicit so sign combinations and 32-bit boundaries
 * are checked against the wide oracle after each product accumulation. */
static void check_row_model(const int32_t row[4],sh2z_vec3_t point) {
    const int32_t coordinates[3]={point.x,point.y,point.z};
    wide_t budget=magnitude(row[3])*65536;
    for(unsigned j=0;j<3;j++) budget+=magnitude(row[j])*magnitude(coordinates[j]);
    if(budget>INT64_C(140737488355327)) return;
    uint32_t hi=0,lo=0;
    uint64_t mac=0;
    wide_t expected=0;
    for(unsigned j=0;j<3;j++) {
        const int64_t product=(int64_t)row[j]*coordinates[j];
        const uint64_t product_bits=(uint64_t)product;
        const uint32_t product_hi=(uint32_t)(product_bits>>32);
        const uint32_t product_lo=(uint32_t)product_bits;
        if(!j) { hi=product_hi; lo=product_lo; }
        else {
            const uint64_t low_sum=(uint64_t)lo+product_lo;
            lo=(uint32_t)low_sum;
            hi=hi+product_hi+(uint32_t)(low_sum>>32);
        }
        mac+=product_bits;
        expected+=(wide_t)row[j]*coordinates[j];
        CHECK(expected>=-INT64_C(140737488355327));
        CHECK(expected<=INT64_C(140737488355327));
        const uint64_t expected_bits=(uint64_t)expected;
        CHECK(lo==(uint32_t)expected_bits);
        CHECK(hi==(uint32_t)(expected_bits>>32));
        CHECK(mac==expected_bits);
    }
    const int64_t dmuls_result=(int64_t)xtrct(hi,lo)+row[3];
    const int64_t mac_result=(int64_t)xtrct((uint32_t)(mac>>32),(uint32_t)mac)+row[3];
    CHECK(dmuls_result>=INT32_MIN && dmuls_result<=INT32_MAX);
    CHECK(dmuls_result==oracle(row,point));
    CHECK(mac_result==dmuls_result);
}

static const sh2z_vec3_t sentinel={147258369,-963852741,192837465};
static void same(sh2z_vec3_t actual,sh2z_vec3_t expected) {
    CHECK(actual.x==expected.x); CHECK(actual.y==expected.y); CHECK(actual.z==expected.z);
}
static void contracts(void) {
    sh2z_vec3_t input[4]={{1,-1,65535},{-65537,32768,-32768},{INT32_MIN,7,INT32_MAX},{0,0,0}};
    sh2z_vec3_t output[6];
    sh2z_mesh_view_t mesh;
    sh2z_transform_plan_t plan;
    sh2z_mat34_t matrix=sh2z_mat34_identity();
    CHECK(!sh2z_transform_plan_mac_available(NULL));
    CHECK(!sh2z_transform_plan_dmuls_available(NULL));
    CHECK(sh2z_mesh_view_init(&mesh,input,4));
    CHECK(sh2z_transform_plan_init(&plan,&mesh,&matrix));
    for(size_t i=0;i<6;i++) output[i]=sentinel;
    CHECK(!sh2z_transform_plan_execute_mac(output,NULL,0,0));
    CHECK(!sh2z_transform_plan_execute_dmuls(output,NULL,0,0));
    CHECK(sh2z_transform_plan_execute_mac(NULL,&plan,4,0));
    CHECK(sh2z_transform_plan_execute_dmuls(NULL,&plan,4,0));
    CHECK(!sh2z_transform_plan_execute_mac(NULL,&plan,0,1));
    CHECK(!sh2z_transform_plan_execute_dmuls(NULL,&plan,0,1));
    CHECK(!sh2z_transform_plan_execute_mac(output,&plan,5,0));
    CHECK(!sh2z_transform_plan_execute_dmuls(output,&plan,3,2));
    CHECK(!sh2z_transform_plan_execute_mac(output,&plan,SIZE_MAX,1));
    CHECK(!sh2z_transform_plan_execute_dmuls(output,&plan,0,SIZE_MAX));
    CHECK(!sh2z_transform_plan_execute_mac(input,&plan,0,4));
    CHECK(!sh2z_transform_plan_execute_dmuls(input+2,&plan,0,1));
    CHECK(!sh2z_transform_plan_execute_mac((sh2z_vec3_t *)&plan,&plan,0,1));
    CHECK(!sh2z_transform_plan_execute_dmuls((sh2z_vec3_t *)&plan,&plan,0,1));
    for(size_t i=0;i<6;i++) same(output[i],sentinel);
    CHECK(sh2z_transform_plan_execute_mac(output+1,&plan,1,2));
    same(output[1],input[1]); same(output[2],input[2]);
    same(output[0],sentinel); same(output[3],sentinel);
    CHECK(sh2z_transform_plan_execute_dmuls(output+1,&plan,1,2));
    same(output[1],input[1]); same(output[2],input[2]);
    same(output[0],sentinel); same(output[3],sentinel);
#if !defined(__sh__) || !defined(__SH2__) || (defined(SH2Z_FORCE_PORTABLE) && SH2Z_FORCE_PORTABLE)
    CHECK(!sh2z_transform_plan_mac_available(&plan));
    CHECK(!sh2z_transform_plan_dmuls_available(&plan));
#endif
}

static void randomized(void) {
    enum { CAPACITY=35 };
    sh2z_vec3_t input[CAPACITY],mac[CAPACITY+2],dmuls[CAPACITY+2];
    sh2z_mat34_t matrix;
    unsigned eligible=0,fallback=0;
    for(unsigned pass=0;pass<3000;pass++) {
        for(unsigned row=0;row<3;row++) for(unsigned col=0;col<4;col++) {
            int32_t value=random_i32();
            if(pass%4==0) value/=4096;
            else if(pass%4==1 && col==3) value/=65536;
            else if(pass%4==2) value/=INT32_C(536870912);
            matrix.m[row][col]=value;
        }
        for(size_t i=0;i<CAPACITY;i++) {
            input[i]=(sh2z_vec3_t){random_i32(),random_i32(),random_i32()};
            if(pass%4==0) { input[i].x/=4096; input[i].y/=4096; input[i].z/=4096; }
            else if(pass%4==1) { input[i].x/=65536; input[i].y/=65536; input[i].z/=65536; }
        }
        const size_t count=1+pass%CAPACITY;
        sh2z_mesh_view_t mesh;
        sh2z_transform_plan_t plan;
        CHECK(sh2z_mesh_view_init(&mesh,input,count));
        CHECK(sh2z_transform_plan_init(&plan,&mesh,&matrix));
        if(plan.fast_path) ++eligible; else ++fallback;
        for(size_t i=0;i<CAPACITY+2;i++) mac[i]=dmuls[i]=sentinel;
        CHECK(sh2z_transform_plan_execute_mac(mac+1,&plan,0,count));
        CHECK(sh2z_transform_plan_execute_dmuls(dmuls+1,&plan,0,count));
        for(size_t i=0;i<count;i++) {
            const sh2z_vec3_t expected={oracle(matrix.m[0],input[i]),oracle(matrix.m[1],input[i]),oracle(matrix.m[2],input[i])};
            same(mac[i+1],expected); same(dmuls[i+1],expected);
            for(unsigned row=0;row<3;row++) check_row_model(matrix.m[row],input[i]);
        }
        same(mac[0],sentinel); same(dmuls[0],sentinel);
        same(mac[count+1],sentinel); same(dmuls[count+1],sentinel);
    }
    CHECK(eligible>1000); CHECK(fallback>500);
}

int main(void) {
    extraction_edges(); contracts(); randomized();
    printf("native batch host models/wrappers: %lu checks; SH-2 assembly not executed\n",checks);
    return 0;
}
