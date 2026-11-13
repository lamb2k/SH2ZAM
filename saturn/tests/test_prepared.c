/* Independent wide-oracle tests for immutable prepared mesh transforms.
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
    const wide_t raw=(wide_t)row[0]*v.x+(wide_t)row[1]*v.y+(wide_t)row[2]*v.z;
    wide_t result=raw/65536;
    if(raw<0&&raw%65536!=0)--result;
    result+=row[3];
    if(result<INT32_MIN)return INT32_MIN;
    if(result>INT32_MAX)return INT32_MAX;
    return (int32_t)result;
}

static sh2z_vec3_t oracle(const sh2z_mat34_t *matrix,sh2z_vec3_t v) {
    return (sh2z_vec3_t){oracle_row(matrix->m[0],v),
                         oracle_row(matrix->m[1],v),
                         oracle_row(matrix->m[2],v)};
}

static void equal(sh2z_vec3_t a,sh2z_vec3_t b) {
    CHECK(a.x==b.x); CHECK(a.y==b.y); CHECK(a.z==b.z);
}

enum { CAPACITY=65 };
static const sh2z_vec3_t sentinel={123456789,-987654321,192837465};

/* Separate, naturally aligned arrays with odd offsets exercise scalar tails
 * and unaligned-to-16-byte batches. The independent int128 oracle does not
 * reuse the library's bound proof, floor helper, or saturation implementation. */
static void check_batch(const sh2z_mat34_t *matrix,const sh2z_vec3_t *positions,
                        size_t count,int expected_fast) {
    sh2z_mesh_view_t view;
    sh2z_transform_plan_t plan;
    sh2z_vec3_t out[CAPACITY+2],general[CAPACITY+2],chunks[CAPACITY+2];
    CHECK(count<=CAPACITY);
    CHECK(sh2z_mesh_view_init(&view,positions,count));
    CHECK(view.positions==positions); CHECK(view.count==count);
    CHECK(sh2z_transform_plan_init(&plan,&view,matrix));
    CHECK(plan.positions==positions); CHECK(plan.count==count);
    if(expected_fast>=0)CHECK(plan.fast_path==(expected_fast!=0));
    for(size_t i=0;i<CAPACITY+2;++i)out[i]=general[i]=chunks[i]=sentinel;
    CHECK(sh2z_transform_plan_execute(out+1,&plan));
    CHECK(sh2z_mat34_transform_points_general(general+1,matrix,positions,count));
    for(size_t i=0;i<count;++i) {
        const sh2z_vec3_t expected=oracle(matrix,positions[i]);
        equal(out[i+1],expected); equal(general[i+1],expected);
    }
    equal(out[0],sentinel); equal(out[count+1],sentinel);
    equal(general[0],sentinel); equal(general[count+1],sentinel);
    /* Range execution must use original cached bounds even for 32-vertex
     * benchmark chunks and short tails. Its output starts at out[0]. */
    const size_t widths[]={1,3,32};
    for(size_t w=0;w<sizeof(widths)/sizeof(widths[0]);++w) {
        for(size_t first=0;first<count;first+=widths[w]) {
            const size_t remaining=count-first;
            const size_t n=remaining<widths[w]?remaining:widths[w];
            CHECK(sh2z_transform_plan_execute_range(chunks+first+1,&plan,first,n));
        }
        for(size_t i=0;i<count;++i)equal(chunks[i+1],out[i+1]);
        equal(chunks[0],sentinel); equal(chunks[count+1],sentinel);
    }
    CHECK(sh2z_transform_plan_execute_range(NULL,&plan,count,0));
}

static void contracts(void) {
    sh2z_vec3_t positions[5]={{1,2,3},{-4,5,-6},{7,-8,9},
                             {INT32_MIN,INT32_MAX,0},{10,11,12}};
    sh2z_vec3_t out[5];
    for(size_t i=0;i<5;++i)out[i]=sentinel;
    sh2z_mat34_t matrix=sh2z_mat34_identity();
    sh2z_mesh_view_t view;
    memset(&view,0xa5,sizeof(view));
    unsigned char original_view[sizeof(view)];
    memcpy(original_view,&view,sizeof(view));
    CHECK(!sh2z_mesh_view_init(NULL,positions,5));
    CHECK(!sh2z_mesh_view_init(&view,NULL,1));
    CHECK(!sh2z_mesh_view_init(&view,positions,SIZE_MAX/sizeof(*positions)+1));
    CHECK(memcmp(original_view,&view,sizeof(view))==0);
    CHECK(sh2z_mesh_view_init(&view,positions,5));
    CHECK(view.max_abs[0]==UINT32_C(2147483648));
    CHECK(view.max_abs[1]==UINT32_C(2147483647)); CHECK(view.max_abs[2]==12);

    sh2z_transform_plan_t plan;
    memset(&plan,0x5a,sizeof(plan));
    unsigned char original_plan[sizeof(plan)];
    memcpy(original_plan,&plan,sizeof(plan));
    CHECK(!sh2z_transform_plan_init(NULL,&view,&matrix));
    CHECK(!sh2z_transform_plan_init(&plan,NULL,&matrix));
    CHECK(!sh2z_transform_plan_init(&plan,&view,NULL));
    CHECK(memcmp(original_plan,&plan,sizeof(plan))==0);
    CHECK(sh2z_transform_plan_init(&plan,&view,&matrix));
    CHECK(!sh2z_transform_plan_execute(NULL,&plan));
    CHECK(!sh2z_transform_plan_execute(out,NULL));
    CHECK(!sh2z_transform_plan_execute_range(out,NULL,0,0));
    CHECK(!sh2z_transform_plan_execute_range(out,&plan,6,0));
    CHECK(!sh2z_transform_plan_execute_range(out,&plan,5,1));
    CHECK(!sh2z_transform_plan_execute_range(out,&plan,1,SIZE_MAX));
    CHECK(!sh2z_transform_plan_execute_range(out,&plan,SIZE_MAX,1));
    for(size_t i=0;i<5;++i)equal(out[i],sentinel);
    CHECK(sh2z_transform_plan_execute_range(NULL,&plan,5,0));
    CHECK(sh2z_transform_plan_execute_range(NULL,&plan,2,0));

    const sh2z_vec3_t saved[5]={{1,2,3},{-4,5,-6},{7,-8,9},
                               {INT32_MIN,INT32_MAX,0},{10,11,12}};
    CHECK(!sh2z_transform_plan_execute(positions,&plan));
    CHECK(!sh2z_transform_plan_execute_range(positions+1,&plan,0,4));
    /* This output is outside the selected input range but inside the full
     * cached mesh. Allowing it would invalidate bounds for later executions. */
    CHECK(!sh2z_transform_plan_execute_range(positions+4,&plan,0,1));
    CHECK(!sh2z_transform_plan_execute_range(positions,&plan,4,1));
    CHECK(!sh2z_transform_plan_execute_range((sh2z_vec3_t *)&plan,&plan,0,1));
    for(size_t i=0;i<5;++i)equal(positions[i],saved[i]);

    /* Constructor storage must not overwrite the immutable array it scans. */
    union { sh2z_vec3_t positions[8]; sh2z_mesh_view_t view; } view_alias;
    for(size_t i=0;i<8;++i)view_alias.positions[i]=sentinel;
    CHECK(!sh2z_mesh_view_init(&view_alias.view,view_alias.positions,8));
    for(size_t i=0;i<8;++i)equal(view_alias.positions[i],sentinel);
    union { sh2z_vec3_t positions[8]; sh2z_transform_plan_t plan; } plan_alias;
    for(size_t i=0;i<8;++i)plan_alias.positions[i]=sentinel;
    CHECK(sh2z_mesh_view_init(&view,plan_alias.positions,8));
    CHECK(!sh2z_transform_plan_init(&plan_alias.plan,&view,&matrix));
    for(size_t i=0;i<8;++i)equal(plan_alias.positions[i],sentinel);

    CHECK(sh2z_mesh_view_init(&view,NULL,0));
    CHECK(view.positions==NULL); CHECK(view.count==0);
    CHECK(view.max_abs[0]==0&&view.max_abs[1]==0&&view.max_abs[2]==0);
    CHECK(sh2z_transform_plan_init(&plan,&view,&matrix));
    CHECK(sh2z_transform_plan_execute(NULL,&plan));
    CHECK(sh2z_transform_plan_execute_range(NULL,&plan,0,0));
    CHECK(!sh2z_transform_plan_execute_range(NULL,&plan,1,0));
    CHECK(!sh2z_transform_plan_execute_range(NULL,&plan,0,1));

    /* Adjacent input/output storage is legal in either address order. */
    sh2z_vec3_t adjacent[4]={{65536,-65537,1},{-1,32768,2},sentinel,sentinel};
    CHECK(sh2z_mesh_view_init(&view,adjacent,2));
    CHECK(sh2z_transform_plan_init(&plan,&view,&matrix));
    CHECK(sh2z_transform_plan_execute(adjacent+2,&plan));
    equal(adjacent[2],adjacent[0]); equal(adjacent[3],adjacent[1]);
    CHECK(sh2z_mesh_view_init(&view,adjacent+2,2));
    CHECK(sh2z_transform_plan_init(&plan,&view,&matrix));
    CHECK(sh2z_transform_plan_execute(adjacent,&plan));
    equal(adjacent[0],adjacent[2]); equal(adjacent[1],adjacent[3]);
}

static void lifetimes(void) {
    sh2z_vec3_t positions[2]={{32768,-65537,131073},{INT32_MAX,0,1}};
    sh2z_vec3_t other[1]={{4,5,6}},out[2];
    sh2z_mesh_view_t view;
    sh2z_transform_plan_t plan;
    sh2z_mat34_t matrix={{{1,2,3,4},{-5,6,-7,8},{9,-10,11,-12}}};
    const sh2z_mat34_t saved=matrix;
    CHECK(sh2z_mesh_view_init(&view,positions,2));
    CHECK(sh2z_transform_plan_init(&plan,&view,&matrix));
    CHECK(plan.fast_path);
    /* A plan snapshots metadata and matrix; neither original object needs to
     * retain its value after preparation, only the vertex storage does. */
    matrix=sh2z_mat34_scale((sh2z_vec3_t){INT32_MAX,INT32_MIN,INT32_MAX});
    CHECK(sh2z_mesh_view_init(&view,other,1));
    CHECK(sh2z_transform_plan_execute(out,&plan));
    equal(out[0],oracle(&saved,positions[0]));
    equal(out[1],oracle(&saved,positions[1]));
    /* After mutation, rebuild the view and plan before using their bounds. */
    positions[0]=(sh2z_vec3_t){INT32_MIN,INT32_MAX,INT32_MIN};
    CHECK(sh2z_mesh_view_init(&view,positions,2));
    CHECK(sh2z_transform_plan_init(&plan,&view,&matrix));
    CHECK(!plan.fast_path);
    CHECK(sh2z_transform_plan_execute(out,&plan));
    equal(out[0],oracle(&matrix,positions[0]));
    equal(out[1],oracle(&matrix,positions[1]));
}

static void bounds(void) {
    const sh2z_vec3_t point={65536,1,0};
    for(unsigned row=0;row<3;++row) {
        sh2z_mat34_t matrix=sh2z_mat34_identity();
        /* INT32_MAX*65536 + 65535 = 2^47-1, the exact budget. */
        matrix.m[row][0]=INT32_MAX; matrix.m[row][1]=65535;
        matrix.m[row][2]=0; matrix.m[row][3]=0;
        check_batch(&matrix,&point,1,1);
        matrix.m[row][1]=65536;
        check_batch(&matrix,&point,1,0);
        matrix.m[row][0]=-INT32_MAX; matrix.m[row][1]=-65535;
        check_batch(&matrix,&point,1,1);
        matrix.m[row][1]=-65536;
        check_batch(&matrix,&point,1,0);
        /* Translation must take part in the proof rather than being clamped
         * or allowed to overflow after a separately bounded dot product. */
        matrix=sh2z_mat34_identity();
        matrix.m[row][0]=0; matrix.m[row][1]=65535; matrix.m[row][2]=0;
        matrix.m[row][3]=INT32_MAX;
        check_batch(&matrix,&point,1,1);
        matrix.m[row][1]=65536;
        check_batch(&matrix,&point,1,0);
    }
    const sh2z_vec3_t zero={0,0,0};
    const sh2z_mat34_t upper={{{0,0,0,INT32_MAX},{0,0,0,0},{0,0,0,-1}}};
    const sh2z_mat34_t lower={{{0,0,0,INT32_MIN},{0,0,0,0},{0,0,0,-1}}};
    check_batch(&upper,&zero,1,1);
    check_batch(&lower,&zero,1,0);

    /* High coefficients can still qualify for a scanned tiny mesh even when
     * the previous matrix-only bound would reject the matrix outright. */
    const sh2z_vec3_t tiny[]={{1,-1,1},{-1,1,-1},{0,0,0}};
    const sh2z_mat34_t large={{{INT32_MIN,INT32_MIN,INT32_MIN,0},
                              {INT32_MAX,INT32_MAX,INT32_MAX,0},
                              {INT32_MIN,INT32_MAX,INT32_MIN,0}}};
    check_batch(&large,tiny,3,1);

    static const int32_t edges[]={INT32_MIN,INT32_MIN+1,-65537,-65536,
        -32769,-32768,-1,0,1,32767,32768,65535,65536,65537,INT32_MAX-1,INT32_MAX};
    const sh2z_mat34_t matrices[]={
        {{{1,1,1,0},{-1,-1,-1,0},{1,1,1,7}}},
        {{{INT32_MIN,INT32_MIN,INT32_MIN,INT32_MAX},
          {INT32_MIN,INT32_MIN,INT32_MAX,0},
          {INT32_MAX,INT32_MAX,INT32_MAX,INT32_MIN}}},
        {{{INT32_MIN,INT32_MAX,1,0},{1,0,0,-1},{0,1,0,1}}},
        {{{65536,0,0,INT32_MAX},{0,65536,0,INT32_MIN},{0,0,65536,0}}},
        {{{-65536,0,0,-1},{0,-65536,0,1},{0,0,-65536,0}}}
    };
    sh2z_vec3_t positions[CAPACITY];
    for(size_t n=0;n<sizeof(matrices)/sizeof(matrices[0]);++n)
        for(size_t x=0;x<sizeof(edges)/sizeof(edges[0]);++x)
            for(size_t y=0;y<sizeof(edges)/sizeof(edges[0]);++y) {
                for(size_t z=0;z<sizeof(edges)/sizeof(edges[0]);++z)
                    positions[z]=(sh2z_vec3_t){edges[x],edges[y],edges[z]};
                check_batch(&matrices[n],positions,sizeof(edges)/sizeof(edges[0]),-1);
            }
    /* Fractional products must be accumulated before a single floor. */
    const sh2z_vec3_t fractional[]={{32768,32768,32768},{-1,-1,-1},
                                   {65535,1,0},{-65535,-1,0}};
    check_batch(&matrices[0],fractional,4,1);
}

static uint32_t rng=UINT32_C(0x61c7e93b);
static uint32_t random_u32(void) {
    rng^=rng<<13; rng^=rng>>17; rng^=rng<<5; return rng;
}
static int32_t random_i32(void) {
    const uint32_t value=random_u32();
    return value<=INT32_MAX?(int32_t)value:
        (int32_t)((int64_t)value-INT64_C(4294967296));
}

static void randomized(void) {
    sh2z_vec3_t positions[CAPACITY];
    sh2z_mat34_t matrix;
    for(unsigned pass=0;pass<3000;++pass) {
        const unsigned mode=pass%3;
        for(unsigned row=0;row<3;++row)for(unsigned col=0;col<4;++col) {
            matrix.m[row][col]=random_i32();
            if(mode==1)matrix.m[row][col]/=4096;
            if(mode==2)matrix.m[row][col]/=65536;
        }
        for(size_t i=0;i<CAPACITY;++i) {
            positions[i]=(sh2z_vec3_t){random_i32(),random_i32(),random_i32()};
            if(mode==1) {
                positions[i].x/=2048; positions[i].y/=2048; positions[i].z/=2048;
            }
        }
        /* mode 1 is guaranteed bounded and nonsaturating; modes 0 and 2
         * exercise full-range fallback and data-dependent eligibility. */
        check_batch(&matrix,positions,pass%(CAPACITY+1),mode==1?1:-1);
    }
}

int main(void) {
    contracts(); lifetimes(); bounds(); randomized();
    printf("prepared transform: %lu checks; prepared/range/general/int128 oracle are bit-identical\n",checks);
    return 0;
}
