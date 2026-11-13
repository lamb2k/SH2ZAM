/* Independent spatial-query oracle and buffer/lifetime contracts. MIT. */
#include "sh2zam/spatial.h"
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_BOXES 512u
#define MAX_NODES (2u*MAX_BOXES-1u)
#define MAX_WORDS ((MAX_BOXES+31u)/32u)
__extension__ typedef __int128 wide_t;
static uint64_t checks;
static uint32_t state=UINT32_C(0x3267ab15);
static sh2z_aabb_t boxes[MAX_BOXES],saved_boxes[MAX_BOXES];
static sh2z_aabb_node_t nodes[MAX_NODES],saved_nodes[MAX_NODES];
static uint32_t scratch[MAX_BOXES],saved_scratch[MAX_BOXES];
static uint32_t hits[MAX_WORDS+2],expected[MAX_WORDS+2];
static bool seen[MAX_BOXES];

static void check(bool value) {
    ++checks;
    if(!value) {
        fprintf(stderr,"spatial check %llu failed\n",(unsigned long long)checks);
        exit(EXIT_FAILURE);
    }
}
static uint32_t random_word(void) {
    state^=state<<13;state^=state>>17;state^=state<<5;return state;
}
static int32_t signed_bits(uint32_t x) {
    return x<=INT32_MAX?(int32_t)x:INT32_MIN+(int32_t)(x-UINT32_C(0x80000000));
}
static int32_t bounded(void) {return (int32_t)(random_word()%UINT32_C(16777217))-8388608;}
static bool valid(sh2z_aabb_t b) {
    return b.min.x<=b.max.x&&b.min.y<=b.max.y&&b.min.z<=b.max.z;
}
static int32_t clamp_coordinate(int32_t p,int32_t lo,int32_t hi) {
    return p<lo?lo:p>hi?hi:p;
}
static bool oracle(sh2z_sphere_t s,sh2z_aabb_t b) {
    if(s.radius<0||!valid(b)) return false;
    const int64_t x=(int64_t)s.center.x-clamp_coordinate(s.center.x,b.min.x,b.max.x);
    const int64_t y=(int64_t)s.center.y-clamp_coordinate(s.center.y,b.min.y,b.max.y);
    const int64_t z=(int64_t)s.center.z-clamp_coordinate(s.center.z,b.min.z,b.max.z);
    return (wide_t)x*x+(wide_t)y*y+(wide_t)z*z<=(wide_t)s.radius*s.radius;
}
static sh2z_aabb_t make_box(sh2z_vec3_t a,sh2z_vec3_t b) {
    return (sh2z_aabb_t){{a.x<b.x?a.x:b.x,a.y<b.y?a.y:b.y,a.z<b.z?a.z:b.z},
        {a.x>b.x?a.x:b.x,a.y>b.y?a.y:b.y,a.z>b.z?a.z:b.z}};
}
static bool contains(sh2z_aabb_t a,sh2z_aabb_t b) {
    return a.min.x<=b.min.x&&a.max.x>=b.max.x&&a.min.y<=b.min.y&&
           a.max.y>=b.max.y&&a.min.z<=b.min.z&&a.max.z>=b.max.z;
}
static size_t check_node(const sh2z_aabb_index_t *index,size_t n) {
    check(n<index->node_count);
    const sh2z_aabb_node_t *node=&index->nodes[n];
    check(node->escape>n&&node->escape<=index->node_count);
    check(valid(node->bounds));
    if(node->object!=UINT32_MAX) {
        check(node->object<index->count);
        check(valid(index->boxes[node->object]));
        check(!seen[node->object]);seen[node->object]=true;
        check(!memcmp(&node->bounds,&index->boxes[node->object],sizeof(node->bounds)));
        check(node->escape==n+1);
        return 1;
    }
    const size_t left=n+1,right=index->nodes[left].escape;
    check(right<node->escape);
    check(index->nodes[right].escape==node->escape);
    check(contains(node->bounds,index->nodes[left].bounds));
    check(contains(node->bounds,index->nodes[right].bounds));
    const size_t a=check_node(index,left),b=check_node(index,right);
    check(b==a||b==a+1);
    check(node->escape-n==a+b+(a+b-1));
    return a+b;
}
static sh2z_aabb_index_t build(size_t count) {
    sh2z_aabb_index_t index;
    memset(nodes,0xa5,sizeof nodes);memset(scratch,0xa5,sizeof scratch);
    memcpy(saved_boxes,boxes,sizeof boxes);
    check(sh2z_aabb_index_build(&index,boxes,count,nodes,MAX_NODES,scratch,MAX_BOXES));
    size_t valid_count=0;
    for(size_t i=0;i<count;i++) if(valid(boxes[i])) ++valid_count;
    check(index.count==count);
    check(index.node_count==(valid_count?valid_count+(valid_count-1):0));
    check(index.boxes==boxes);
    check(index.nodes==(valid_count?nodes:NULL));
    memset(seen,0,sizeof seen);
    if(valid_count) check(check_node(&index,0)==valid_count);
    for(size_t i=0;i<count;i++) check(seen[i]==valid(boxes[i]));
    for(size_t i=index.node_count;i<MAX_NODES;i++) {
        const unsigned char *p=(const unsigned char*)&nodes[i];
        for(size_t j=0;j<sizeof(nodes[i]);j++) check(p[j]==0xa5);
    }
    for(size_t i=valid_count;i<MAX_BOXES;i++) check(scratch[i]==UINT32_C(0xa5a5a5a5));
    check(!memcmp(boxes,saved_boxes,sizeof boxes));
    memcpy(saved_nodes,nodes,sizeof nodes);
    return index;
}
static void query(const sh2z_aabb_index_t *index,sh2z_sphere_t s) {
    memset(hits,0x53,sizeof hits);memset(expected,0x53,sizeof expected);
    const size_t words=index->count/32+(index->count%32!=0);
    for(size_t i=0;i<words;i++) expected[i]=0;
    size_t total=0,actual=SIZE_MAX;
    for(size_t i=0;i<index->count;i++) {
        const bool wanted=oracle(s,index->boxes[i]);
        check(wanted==sh2z_sphere_intersects_aabb(s,index->boxes[i]));
        if(wanted) {expected[i>>5]|=UINT32_C(1)<<(i&31u);++total;}
    }
    check(sh2z_aabb_index_query_sphere(hits,MAX_WORDS+2,&actual,index,s));
    check(total==actual);
    for(size_t i=0;i<MAX_WORDS+2;i++) check(hits[i]==expected[i]);
    check(!memcmp(boxes,saved_boxes,sizeof boxes));
    check(!memcmp(nodes,saved_nodes,sizeof nodes));
    memset(hits,0x53,sizeof hits);
    check(sh2z_aabb_index_query_sphere(hits,MAX_WORDS+2,NULL,index,s));
    check(!memcmp(hits,expected,sizeof hits));
}
static void edge_cases(void) {
    const int32_t e[]={INT32_MIN,INT32_MIN+1,-65536,-1,0,1,65536,INT32_MAX-1,INT32_MAX};
    for(size_t i=0;i<9;i++) for(size_t j=0;j<9;j++) {
        boxes[0]=make_box((sh2z_vec3_t){e[i],e[j],e[8-i]},(sh2z_vec3_t){e[j],e[8-i],e[i]});
        boxes[1]=(sh2z_aabb_t){{INT32_MIN,INT32_MIN,INT32_MIN},{INT32_MAX,INT32_MAX,INT32_MAX}};
        boxes[2]=(sh2z_aabb_t){{0,0,0},{0,0,0}};
        boxes[3]=(sh2z_aabb_t){{1,1,1},{-1,1,1}};
        sh2z_aabb_index_t index=build(4);
        for(size_t k=0;k<9;k++) {
            query(&index,(sh2z_sphere_t){{e[k],e[8-k],e[(k+i)%9]},e[j]});
            query(&index,(sh2z_sphere_t){{e[k],e[k],e[k]},INT32_MAX});
        }
    }
    /* Tangency and the farthest-corner containment boundary. */
    for(size_t count=1;count<=MAX_BOXES;count+=17) {
        for(size_t i=0;i<count;i++) boxes[i]=(sh2z_aabb_t){{-3,-4,0},{3,4,0}};
        sh2z_aabb_index_t index=build(count);
        query(&index,(sh2z_sphere_t){{0,0,0},5});
        query(&index,(sh2z_sphere_t){{0,0,0},4});
        query(&index,(sh2z_sphere_t){{8,0,0},5});
        query(&index,(sh2z_sphere_t){{9,0,0},5});
        query(&index,(sh2z_sphere_t){{0,0,0},-1});
        boxes[count/2]=(sh2z_aabb_t){{1,0,0},{0,0,0}};
        index=build(count);
        query(&index,(sh2z_sphere_t){{0,0,0},5});
    }
    /* Three bounded squares can exceed INT64_MAX; the narrow phase must use
     * their exact unsigned sum. Broad direct scans also preserve tail bits. */
    for(size_t i=0;i<65;i++) boxes[i]=(sh2z_aabb_t){{0,0,0},{0,0,0}};
    sh2z_aabb_index_t index=build(65);
    query(&index,(sh2z_sphere_t){{INT32_MAX,INT32_MAX,INT32_MAX},INT32_MAX});
    query(&index,(sh2z_sphere_t){{INT32_MAX,0,0},INT32_MAX});
    query(&index,(sh2z_sphere_t){{INT32_MAX,1,0},INT32_MAX});
    query(&index,(sh2z_sphere_t){{INT32_MIN,0,0},INT32_MAX});
    boxes[32]=(sh2z_aabb_t){{1,0,0},{0,0,0}};
    index=build(65);
    query(&index,(sh2z_sphere_t){{INT32_MAX,INT32_MAX,INT32_MAX},INT32_MAX});
    query(&index,(sh2z_sphere_t){{0,0,0},0});
}
static void distribution_cases(void) {
    const size_t counts[]={0,1,2,3,7,16,31,32,33,63,64,65,127,128,255,256,511,512};
    for(unsigned mode=0;mode<7;mode++) for(size_t k=0;k<sizeof(counts)/sizeof(counts[0]);k++) {
        const size_t count=counts[k];
        for(size_t i=0;i<count;i++) {
            int32_t x=(int32_t)i*1048576-268435456;
            if(mode==1) x=268435456-(int32_t)i*1048576;
            if(mode==2) x=(int32_t)(i<count/2?i:count-i)*1048576-134217728;
            if(mode==3) x=0;
            const int32_t half=(mode==4)?134217728:4096;
            boxes[i]=(sh2z_aabb_t){{x-half,-half,-half},{x+half,half,half}};
            if(mode==5&&i%3==0) boxes[i].min.y=boxes[i].max.y+1;
            if(mode==6) boxes[i]=(sh2z_aabb_t){{1,0,0},{0,0,0}};
        }
        sh2z_aabb_index_t index=build(count);
        for(unsigned q=0;q<48;q++) {
            const int32_t x=q<16?(int32_t)q*16777216-134217728:signed_bits(random_word());
            const int32_t radius=q<16?8192:q<32?134217728:INT32_MAX;
            query(&index,(sh2z_sphere_t){{x,0,0},radius});
        }
    }
}
static void random_cases(void) {
    for(unsigned mode=0;mode<3;mode++) for(unsigned batch=0;batch<32;batch++) {
        const size_t count=1+random_word()%MAX_BOXES;
        for(size_t i=0;i<count;i++) {
            sh2z_vec3_t a={0,0,0},b={0,0,0};
            if(mode==0) {
                a=(sh2z_vec3_t){bounded(),bounded(),bounded()};
                b=(sh2z_vec3_t){a.x+(int32_t)(random_word()%65537u),a.y+(int32_t)(random_word()%65537u),a.z+(int32_t)(random_word()%65537u)};
            } else {
                a=(sh2z_vec3_t){signed_bits(random_word()),signed_bits(random_word()),signed_bits(random_word())};
                b=(sh2z_vec3_t){signed_bits(random_word()),signed_bits(random_word()),signed_bits(random_word())};
            }
            boxes[i]=make_box(a,b);
            if(mode==2&&i%7==0) boxes[i]=(sh2z_aabb_t){{1,0,0},{0,0,0}};
        }
        const sh2z_aabb_index_t index=build(count);
        for(unsigned q=0;q<64;q++) {
            const sh2z_sphere_t s=mode==0?(sh2z_sphere_t){{bounded(),bounded(),bounded()},(int32_t)(random_word()%131073u)}
                :(sh2z_sphere_t){{signed_bits(random_word()),signed_bits(random_word()),signed_bits(random_word())},signed_bits(random_word())};
            query(&index,s);
        }
    }
}
static void failed_build(sh2z_aabb_index_t *out,const sh2z_aabb_t *in,size_t count,
                         sh2z_aabb_node_t *storage,size_t capacity,uint32_t *ids,size_t id_capacity) {
    sh2z_aabb_index_t saved;
    if(out) memcpy(&saved,out,sizeof saved);
    memcpy(saved_nodes,nodes,sizeof nodes);memcpy(saved_scratch,scratch,sizeof scratch);
    memcpy(saved_boxes,boxes,sizeof boxes);
    check(!sh2z_aabb_index_build(out,in,count,storage,capacity,ids,id_capacity));
    if(out) check(!memcmp(out,&saved,sizeof saved));
    check(!memcmp(nodes,saved_nodes,sizeof nodes));
    check(!memcmp(scratch,saved_scratch,sizeof scratch));
    check(!memcmp(boxes,saved_boxes,sizeof boxes));
}
static void compact_capacity_cases(void) {
    for(size_t i=0;i<65;i++) boxes[i]=(sh2z_aabb_t){{1,0,0},{0,0,0}};
    sh2z_aabb_index_t index;
    memcpy(saved_boxes,boxes,sizeof boxes);memcpy(saved_nodes,nodes,sizeof nodes);
    check(sh2z_aabb_index_build(&index,boxes,65,NULL,0,NULL,0));
    check(index.count==65&&index.node_count==0&&index.nodes==NULL);
    query(&index,(sh2z_sphere_t){{0,0,0},INT32_MAX});
    boxes[0]=(sh2z_aabb_t){{-3,-4,0},{3,4,0}};
    boxes[32]=boxes[0];boxes[64]=boxes[0];
    memset(nodes,0xa5,sizeof nodes);memset(scratch,0xa5,sizeof scratch);
    memcpy(saved_boxes,boxes,sizeof boxes);
    check(sh2z_aabb_index_build(&index,boxes,65,nodes,5,scratch,3));
    check(index.count==65&&index.node_count==5);
    check(scratch[3]==UINT32_C(0xa5a5a5a5));
    const unsigned char *tail=(const unsigned char*)&nodes[5];
    for(size_t i=0;i<sizeof(nodes[5]);i++) check(tail[i]==0xa5);
    memcpy(saved_nodes,nodes,sizeof nodes);
    query(&index,(sh2z_sphere_t){{0,0,0},5});
    query(&index,(sh2z_sphere_t){{8,0,0},5});
    failed_build(&index,boxes,65,nodes,4,scratch,3);
    failed_build(&index,boxes,65,nodes,5,scratch,2);
}
static void failure_cases(void) {
    boxes[0]=(sh2z_aabb_t){{-1,-2,-3},{1,2,3}};
    boxes[1]=(sh2z_aabb_t){{4,5,6},{7,8,9}};
    sh2z_aabb_index_t index=build(2);
    failed_build(NULL,boxes,2,nodes,MAX_NODES,scratch,MAX_BOXES);
    failed_build(&index,NULL,2,nodes,MAX_NODES,scratch,MAX_BOXES);
    failed_build(&index,boxes,SIZE_MAX,nodes,MAX_NODES,scratch,MAX_BOXES);
    failed_build(&index,boxes,2,nodes,2,scratch,MAX_BOXES);
    failed_build(&index,boxes,2,nodes,MAX_NODES,scratch,1);
    failed_build(&index,boxes,2,NULL,3,scratch,MAX_BOXES);
    failed_build(&index,boxes,2,nodes,MAX_NODES,NULL,2);
    failed_build(&index,boxes,2,nodes,SIZE_MAX,scratch,MAX_BOXES);
    failed_build(&index,boxes,2,nodes,MAX_NODES,scratch,SIZE_MAX);
    failed_build(&index,boxes,2,(sh2z_aabb_node_t*)(void*)boxes,3,scratch,MAX_BOXES);
    failed_build(&index,boxes,2,nodes,MAX_NODES,(uint32_t*)(void*)boxes,2);
    failed_build(&index,boxes,2,nodes,MAX_NODES,(uint32_t*)(void*)nodes,2);
    failed_build(&index,boxes,2,(sh2z_aabb_node_t*)(void*)&index,3,scratch,MAX_BOXES);
    failed_build(&index,boxes,2,nodes,MAX_NODES,(uint32_t*)(void*)&index,2);
    /* Check rejection output preservation before inspecting borrowed storage. */
    const sh2z_sphere_t s={{0,0,0},5};
    memset(hits,0x4a,sizeof hits);memcpy(expected,hits,sizeof hits);
    size_t count=123;
    check(!sh2z_aabb_index_query_sphere(hits,0,&count,&index,s));
    check(count==123&&!memcmp(hits,expected,sizeof hits));
    check(!sh2z_aabb_index_query_sphere(NULL,1,&count,&index,s));
    check(count==123);
    check(!sh2z_aabb_index_query_sphere(hits,SIZE_MAX,&count,&index,s));
    check(count==123&&!memcmp(hits,expected,sizeof hits));
    check(!sh2z_aabb_index_query_sphere(hits,MAX_WORDS+2,&count,NULL,s));
    check(count==123&&!memcmp(hits,expected,sizeof hits));
    check(!sh2z_aabb_index_query_sphere((uint32_t*)(void*)boxes,1,&count,&index,s));
    check(!memcmp(boxes,saved_boxes,sizeof boxes));
    check(!sh2z_aabb_index_query_sphere((uint32_t*)(void*)nodes,1,&count,&index,s));
    check(!memcmp(nodes,saved_nodes,sizeof nodes));
    sh2z_aabb_index_t saved=index;
    check(!sh2z_aabb_index_query_sphere((uint32_t*)(void*)&index,1,&count,&index,s));
    check(!memcmp(&index,&saved,sizeof saved));
    check(!sh2z_aabb_index_query_sphere(hits,MAX_WORDS+2,(size_t*)(void*)hits,&index,s));
    check(!memcmp(hits,expected,sizeof hits));
    check(!sh2z_aabb_index_query_sphere(hits,MAX_WORDS+2,(size_t*)(void*)boxes,&index,s));
    check(!memcmp(hits,expected,sizeof hits)&&!memcmp(boxes,saved_boxes,sizeof boxes));
    check(!sh2z_aabb_index_query_sphere(hits,MAX_WORDS+2,(size_t*)(void*)nodes,&index,s));
    check(!memcmp(hits,expected,sizeof hits)&&!memcmp(nodes,saved_nodes,sizeof nodes));
    check(!sh2z_aabb_index_query_sphere(hits,MAX_WORDS+2,(size_t*)(void*)&index,&index,s));
    check(!memcmp(hits,expected,sizeof hits)&&!memcmp(&index,&saved,sizeof saved));
    check(sh2z_aabb_index_build(&index,NULL,0,NULL,0,NULL,0));
    check(sh2z_aabb_index_query_sphere(NULL,0,&count,&index,s)&&count==0);
    check(sh2z_aabb_index_query_sphere(NULL,0,NULL,&index,s));
}
int main(void) {
    edge_cases();distribution_cases();random_cases();compact_capacity_cases();failure_cases();
    printf("spatial: %llu exact tree/query/contract checks passed\n",(unsigned long long)checks);
    return 0;
}
