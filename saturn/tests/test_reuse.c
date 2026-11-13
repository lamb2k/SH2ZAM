/* Independent exact oracle and workspace contracts. MIT; SH2ZAM Contributors. */
#include "sh2zam/reuse_experimental.h"
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#if !defined(__SIZEOF_INT128__)
#error "Host oracle requires int128; the library does not."
#endif
__extension__ typedef __int128 wide_t;
static unsigned long checks;
#define CHECK(e) do {++checks;if(!(e)){fprintf(stderr,"FAIL %s:%d: %s\n",__FILE__,__LINE__,#e);exit(1);}}while(0)
enum { N=385,K=256 };
static sh2z_vec3_t input[N],output[N+2],chunks[N+2];
static sh2z_reuse_index_t indices[N];
static sh2z_fx_t xs[K];
static sh2z_reuse_yz_t yz[K];
static sh2z_reuse_term_t xt[K],yzt[K];
static sh2z_reuse_mesh_t mesh;
static sh2z_reuse_plan_t plan;
static const sh2z_vec3_t sentinel={123456789,-987654321,192837465};
static uint32_t rng=UINT32_C(0xabc781e3);
static uint32_t random_word(void){rng^=rng<<13;rng^=rng>>17;rng^=rng<<5;return rng;}
static int32_t signed_word(uint32_t u){return u<=INT32_MAX?(int32_t)u:(int32_t)((int64_t)u-INT64_C(4294967296));}
static int32_t oracle_row(const int32_t row[4],sh2z_vec3_t v){
    wide_t raw=(wide_t)row[0]*v.x+(wide_t)row[1]*v.y+(wide_t)row[2]*v.z;
    wide_t q=raw/65536;if(raw<0&&raw%65536)--q;q+=row[3];
    return q<INT32_MIN?INT32_MIN:q>INT32_MAX?INT32_MAX:(int32_t)q;
}
static void equal(sh2z_vec3_t a,sh2z_vec3_t b){CHECK(a.x==b.x);CHECK(a.y==b.y);CHECK(a.z==b.z);}
static void init_mesh(size_t count){CHECK(sh2z_reuse_mesh_init(&mesh,input,count,indices,N,xs,K,yz,K));}
static void check_output(const sh2z_mat34_t *matrix,size_t count,int expected_enabled){
    CHECK(sh2z_reuse_plan_init(&plan,&mesh,matrix,xt,K,yzt,K));
    if(expected_enabled>=0)CHECK(plan.enabled==(expected_enabled!=0));
    for(size_t i=0;i<N+2;i++)output[i]=chunks[i]=sentinel;
    CHECK(sh2z_reuse_plan_execute(output+1,&plan));
    for(size_t i=0;i<count;i++){
        const sh2z_vec3_t expected={oracle_row(matrix->m[0],input[i]),oracle_row(matrix->m[1],input[i]),oracle_row(matrix->m[2],input[i])};
        equal(output[i+1],expected);
    }
    equal(output[0],sentinel);equal(output[count+1],sentinel);
    for(size_t first=0;first<count;first+=31){size_t n=count-first;if(n>31)n=31;CHECK(sh2z_reuse_plan_execute_range(chunks+first+1,&plan,first,n));}
    for(size_t i=0;i<count;i++)equal(chunks[i+1],output[i+1]);
    equal(chunks[0],sentinel);equal(chunks[count+1],sentinel);
    CHECK(sh2z_reuse_plan_execute_range(NULL,&plan,count,0));
}

static void contracts(void){
    for(size_t i=0;i<N;i++)input[i]=(sh2z_vec3_t){(int32_t)(i%2),-7,9};
    memset(&mesh,0xa5,sizeof(mesh));memset(indices,0x5a,sizeof(indices));
    unsigned char mesh_before[sizeof(mesh)],index_before[sizeof(indices)];
    memcpy(mesh_before,&mesh,sizeof(mesh));memcpy(index_before,indices,sizeof(indices));
    CHECK(!sh2z_reuse_mesh_init(NULL,input,128,indices,N,xs,K,yz,K));
    CHECK(!sh2z_reuse_mesh_init(&mesh,NULL,128,indices,N,xs,K,yz,K));
    CHECK(!sh2z_reuse_mesh_init(&mesh,input,128,indices,127,xs,K,yz,K));
    CHECK(!sh2z_reuse_mesh_init(&mesh,input,128,NULL,N,xs,K,yz,K));
    CHECK(!sh2z_reuse_mesh_init(&mesh,input,128,indices,N,NULL,K,yz,K));
    CHECK(!sh2z_reuse_mesh_init(&mesh,input,128,indices,N,xs,K,NULL,K));
    CHECK(!sh2z_reuse_mesh_init(&mesh,input,SIZE_MAX,indices,SIZE_MAX,xs,K,yz,K));
    CHECK(!sh2z_reuse_mesh_init(&mesh,input,128,indices,SIZE_MAX,xs,K,yz,K));
    CHECK(!sh2z_reuse_mesh_init(&mesh,input,128,indices,N,xs,UINT32_MAX/sizeof(sh2z_reuse_term_t)+1u,yz,K));
    CHECK(!sh2z_reuse_mesh_init(&mesh,input,128,(sh2z_reuse_index_t *)input,128,xs,K,yz,K));
    CHECK(!sh2z_reuse_mesh_init(&mesh,input,128,indices,N,(sh2z_fx_t *)indices,K,yz,K));
    CHECK(memcmp(mesh_before,&mesh,sizeof(mesh))==0);
    CHECK(memcmp(index_before,indices,sizeof(indices))==0);
    init_mesh(128);CHECK(mesh.reusable);

    sh2z_mat34_t matrix=sh2z_mat34_identity();
    memset(&plan,0x5a,sizeof(plan));memset(xt,0x3c,sizeof(xt));
    unsigned char plan_before[sizeof(plan)],terms_before[sizeof(xt)];
    memcpy(plan_before,&plan,sizeof(plan));memcpy(terms_before,xt,sizeof(xt));
    CHECK(!sh2z_reuse_plan_init(NULL,&mesh,&matrix,xt,K,yzt,K));
    CHECK(!sh2z_reuse_plan_init(&plan,NULL,&matrix,xt,K,yzt,K));
    CHECK(!sh2z_reuse_plan_init(&plan,&mesh,NULL,xt,K,yzt,K));
    CHECK(!sh2z_reuse_plan_init(&plan,&mesh,&matrix,NULL,K,yzt,K));
    CHECK(!sh2z_reuse_plan_init(&plan,&mesh,&matrix,xt,K,NULL,K));
    CHECK(!sh2z_reuse_plan_init(&plan,&mesh,&matrix,xt,SIZE_MAX,yzt,K));
    CHECK(!sh2z_reuse_plan_init(&plan,&mesh,&matrix,xt,K,xt,K));
    CHECK(!sh2z_reuse_plan_init(&plan,&mesh,&matrix,(sh2z_reuse_term_t *)&plan,1,yzt,K));
    CHECK(!sh2z_reuse_plan_init(&plan,&mesh,&matrix,(sh2z_reuse_term_t *)&mesh,1,yzt,K));
    CHECK(!sh2z_reuse_plan_init(&plan,&mesh,&matrix,(sh2z_reuse_term_t *)input,1,yzt,K));
    CHECK(!sh2z_reuse_plan_init(&plan,&mesh,&matrix,(sh2z_reuse_term_t *)indices,1,yzt,K));
    CHECK(!sh2z_reuse_plan_init(&plan,&mesh,&matrix,(sh2z_reuse_term_t *)xs,1,yzt,K));
    CHECK(!sh2z_reuse_plan_init(&plan,&mesh,&matrix,(sh2z_reuse_term_t *)yz,1,yzt,K));
    CHECK(!sh2z_reuse_plan_init(&plan,&mesh,(const sh2z_mat34_t *)&plan,xt,K,yzt,K));
    CHECK(!sh2z_reuse_plan_init(&plan,&mesh,(const sh2z_mat34_t *)xt,xt,K,yzt,K));
    CHECK(memcmp(plan_before,&plan,sizeof(plan))==0);
    CHECK(memcmp(terms_before,xt,sizeof(xt))==0);
    CHECK(sh2z_reuse_plan_init(&plan,&mesh,&matrix,xt,K,yzt,K));CHECK(plan.enabled);
    for(size_t i=0;i<N+2;i++)output[i]=sentinel;
    CHECK(!sh2z_reuse_plan_execute(NULL,&plan));CHECK(!sh2z_reuse_plan_execute(output,NULL));
    CHECK(!sh2z_reuse_plan_execute_range(output,&plan,129,0));
    CHECK(!sh2z_reuse_plan_execute_range(output,&plan,128,1));
    CHECK(!sh2z_reuse_plan_execute_range(output,&plan,1,SIZE_MAX));
    CHECK(!sh2z_reuse_plan_execute_range(input+100,&plan,0,1));
    CHECK(!sh2z_reuse_plan_execute_range((sh2z_vec3_t *)&plan,&plan,0,1));
    CHECK(!sh2z_reuse_plan_execute_range((sh2z_vec3_t *)&mesh,&plan,0,1));
    CHECK(!sh2z_reuse_plan_execute_range((sh2z_vec3_t *)indices,&plan,0,1));
    CHECK(!sh2z_reuse_plan_execute_range((sh2z_vec3_t *)(indices+300),&plan,0,1));
    CHECK(!sh2z_reuse_plan_execute_range((sh2z_vec3_t *)(xs+200),&plan,0,1));
    CHECK(!sh2z_reuse_plan_execute_range((sh2z_vec3_t *)(yz+200),&plan,0,1));
    CHECK(!sh2z_reuse_plan_execute_range((sh2z_vec3_t *)(xt+200),&plan,0,1));
    CHECK(!sh2z_reuse_plan_execute_range((sh2z_vec3_t *)(yzt+200),&plan,0,1));
    for(size_t i=0;i<N+2;i++)equal(output[i],sentinel);

    /* Borrowed workspace is read-only during execute; matrix is snapshotted. */
    memcpy(terms_before,xt,sizeof(xt));matrix=sh2z_mat34_scale((sh2z_vec3_t){0,0,0});
    CHECK(sh2z_reuse_plan_execute(output,&plan));
    for(size_t i=0;i<128;i++)equal(output[i],input[i]);
    CHECK(memcmp(terms_before,xt,sizeof(xt))==0);
    CHECK(sh2z_reuse_plan_init(&plan,&mesh,&matrix,xt,K,yzt,K));
    CHECK(sh2z_reuse_plan_execute(output,&plan));
    for(size_t i=0;i<128;i++)equal(output[i],(sh2z_vec3_t){0,0,0});

    /* Lack of table space is a valid exact prepared fallback. */
    matrix=sh2z_mat34_identity();
    CHECK(sh2z_reuse_plan_init(&plan,&mesh,&matrix,NULL,0,NULL,0));CHECK(!plan.enabled);
    CHECK(sh2z_reuse_plan_execute(output,&plan));
    for(size_t i=0;i<128;i++)equal(output[i],input[i]);
    CHECK(sh2z_reuse_mesh_init(&mesh,input,128,indices,N,NULL,0,NULL,0));CHECK(!mesh.reusable);
    check_output(&matrix,128,0);
    CHECK(sh2z_reuse_mesh_init(&mesh,NULL,0,NULL,0,NULL,0,NULL,0));
    CHECK(sh2z_reuse_plan_init(&plan,&mesh,&matrix,NULL,0,NULL,0));
    CHECK(sh2z_reuse_plan_execute(NULL,&plan));
    CHECK(sh2z_reuse_plan_execute_range(NULL,&plan,0,0));
}

static void boundaries(void){
    sh2z_mat34_t matrix=sh2z_mat34_identity();
    for(size_t i=0;i<128;i++)input[i]=(sh2z_vec3_t){(int32_t)(i%2),(int32_t)(i%31),0};
    init_mesh(128);CHECK(mesh.reusable);CHECK(mesh.x_count==2);CHECK(mesh.yz_count==31);
    check_output(&matrix,128,1);
    for(size_t i=0;i<128;i++)input[i].y=(int32_t)(i%32);
    init_mesh(128);CHECK(!mesh.reusable);check_output(&matrix,128,0);
    for(size_t i=0;i<128;i++)input[i]=(sh2z_vec3_t){(int32_t)(i%2),0,0};
    init_mesh(127);CHECK(!mesh.reusable);check_output(&matrix,127,0);
    CHECK(sh2z_reuse_mesh_init(&mesh,input,128,indices,N,xs,1,yz,K));CHECK(!mesh.reusable);check_output(&matrix,128,0);
    for(size_t i=0;i<128;i++)input[i]=(sh2z_vec3_t){0,(int32_t)(i%2),0};
    CHECK(sh2z_reuse_mesh_init(&mesh,input,128,indices,N,xs,K,yz,1));CHECK(!mesh.reusable);check_output(&matrix,128,0);
    /* INT32_MIN magnitudes, cancellation and negative fractional floors. */
    for(size_t i=0;i<129;i++)input[i]=(sh2z_vec3_t){i&1?INT32_MIN:INT32_MAX,-1,1};
    init_mesh(129);CHECK(mesh.reusable);
    matrix=(sh2z_mat34_t){{{1,65535,-1,17},{-1,1,-65535,-23},{0,INT32_MAX,INT32_MAX,-1}}};
    check_output(&matrix,129,1);
    matrix.m[0][0]=INT32_MAX;matrix.m[0][1]=INT32_MIN;matrix.m[0][2]=INT32_MAX;
    matrix.m[1][3]=INT32_MIN;check_output(&matrix,129,0);
    for(size_t i=0;i<129;i++)input[i]=(sh2z_vec3_t){1,0,0};
    init_mesh(129);
    matrix=sh2z_mat34_identity();matrix.m[0][3]=INT32_MAX;matrix.m[0][0]=65535;
    check_output(&matrix,129,1); /* abs translated raw sum is exactly 2^47-1. */
    matrix.m[0][0]=65536;check_output(&matrix,129,0);
}

static void randomized(void){
    unsigned fast=0,fallback=0;
    for(unsigned trial=0;trial<700;trial++){
        const size_t count=128+random_word()%(N-128);
        int32_t xpool[6],ypool[4],zpool[4];
        for(unsigned i=0;i<6;i++)xpool[i]=trial%3?((int32_t)(random_word()%1048577)-524288):signed_word(random_word());
        for(unsigned i=0;i<4;i++){
            ypool[i]=trial%3?((int32_t)(random_word()%1048577)-524288):signed_word(random_word());
            zpool[i]=trial%3?((int32_t)(random_word()%1048577)-524288):signed_word(random_word());
        }
        for(size_t i=0;i<count;i++)input[i]=(sh2z_vec3_t){xpool[random_word()%6],ypool[random_word()%4],zpool[random_word()%4]};
        init_mesh(count);CHECK(mesh.reusable);
        sh2z_mat34_t matrix;
        for(unsigned r=0;r<3;r++)for(unsigned c=0;c<4;c++)matrix.m[r][c]=trial%3?((int32_t)(random_word()%524289)-262144):signed_word(random_word());
        check_output(&matrix,count,-1);if(plan.enabled)fast++;else fallback++;
        /* Reuse the identical metadata with a freshly changed matrix/table. */
        matrix.m[0][0]=-1;matrix.m[2][3]=12345;check_output(&matrix,count,-1);
    }
    CHECK(fast>400);CHECK(fallback>200);
}

int main(void){contracts();boundaries();randomized();printf("PASS: %lu coordinate-reuse oracle/contract checks; exact original-order output, fresh matrix tables, guarded fallback\n",checks);return 0;}
