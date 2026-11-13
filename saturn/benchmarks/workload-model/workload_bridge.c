/* Exact SH-2 workload comparisons with setup included. MIT. */
#include "suite.h"
#include "prior_api.h"
#include "sh2zam/batch_experimental.h"
#include <limits.h>
#include <string.h>
#include "standalone.c"
#ifndef BENCH_COMMIT
#define BENCH_COMMIT "host-test-only"
#endif
const char workload_source_commit[]=BENCH_COMMIT;
const char workload_compiler[]=__VERSION__;
const char workload_compile_flags[]="-m2 -mb -O2 -std=c11 -fno-fast-math -ffp-contract=off -fno-lto";
const struct workload_case workload_cases[WORKLOAD_CASES]={
#include "cases.inc"
};
const uint32_t workload_case_count=WORKLOAD_CASES;
const uint32_t workload_input_bytes=sizeof(struct workload_input);
const uint32_t workload_max_words=WORKLOAD_WORDS;
struct workload_input workload_inputs;
union workload_output workload_outputs[WORKLOAD_PATHS];
_Static_assert(sizeof(union workload_output)==WORKLOAD_WORDS*sizeof(uint32_t),"model output stride");
static sh2z_aabb_node_t nodes[2*WORKLOAD_BOXES-1];
static uint32_t scratch[WORKLOAD_BOXES];
static sh2z_aabb_index_t index_view;
static uint32_t active_case,state;

static uint32_t random_word(void) {
 state^=state<<13;state^=state>>17;state^=state<<5;return state;
}
static int32_t signed_bits(uint32_t x) {
 return x<=INT32_MAX?(int32_t)x:INT32_MIN+(int32_t)(x-UINT32_C(0x80000000));
}
static int32_t value(uint32_t profile,uint32_t i) {
 static const int32_t edge[8]={INT32_MIN,INT32_MAX,0,1,-1,65536,-65536,INT32_MIN+1};
 uint32_t x=random_word();
 if(profile==1) return (int32_t)(x%65u)-32;
 if(profile==2) return i<8?edge[x&7u]:signed_bits(x);
 return (int32_t)(x%1048577u)-524288;
}
static int32_t jitter(uint32_t span) {return (int32_t)(random_word()%(2*span+1))- (int32_t)span;}
static sh2z_vec3_t box_center(const sh2z_aabb_t *b) {
 return (sh2z_vec3_t){(int32_t)(((int64_t)b->min.x+b->max.x)/2),
  (int32_t)(((int64_t)b->min.y+b->max.y)/2),(int32_t)(((int64_t)b->min.z+b->max.z)/2)};
}
static void spatial_inputs(uint32_t profile,uint32_t count,uint32_t sample) {
 for(uint32_t i=0;i<count;i++) {
  int32_t x,y,z,extent;
  if(profile==2) {
   x=jitter(32768);y=jitter(32768);z=jitter(32768);extent=16384+(int32_t)(random_word()%16385u);
  } else if(profile==3) {
   x=jitter(262144);y=jitter(262144);z=jitter(262144);extent=32768+(int32_t)(random_word()%32769u);
  } else if(profile==4) {
   x=((int32_t)(i&7u)*2-7)*INT32_C(268435456);
   y=((int32_t)((i>>3)&7u)*2-7)*INT32_C(268435456);
   z=((int32_t)((i>>6)&7u)*2-7)*INT32_C(268435456);extent=INT32_C(4194304);
  } else {
   int32_t step=profile==0?1048576:262144;
   x=((int32_t)(i&7u)-4)*step+jitter(4096);
   y=((int32_t)((i>>3)&7u)-4)*step+jitter(4096);
   z=((int32_t)((i>>6)&7u)-4)*step+jitter(4096);extent=32768+(int32_t)(random_word()%32769u);
  }
  workload_inputs.boxes[i]=(sh2z_aabb_t){{x-extent,y-extent,z-extent},{x+extent,y+extent,z+extent}};
  if(profile==4 && i%31u==0) workload_inputs.boxes[i].min.x=workload_inputs.boxes[i].max.x+1;
 }
 if(profile==4) {
  workload_inputs.boxes[0]=(sh2z_aabb_t){{INT32_MIN,INT32_MIN,INT32_MIN},{INT32_MIN+1024,INT32_MIN+1024,INT32_MIN+1024}};
  workload_inputs.boxes[1]=(sh2z_aabb_t){{INT32_MAX-1024,INT32_MAX-1024,INT32_MAX-1024},{INT32_MAX,INT32_MAX,INT32_MAX}};
 }
 for(uint32_t q=0;q<WORKLOAD_QUERIES;q++) {
  uint32_t selected=(q*37u+sample*19u)%count;
  sh2z_vec3_t c=box_center(&workload_inputs.boxes[selected]);int32_t radius;
  if(profile==2) {c=(sh2z_vec3_t){jitter(8192),jitter(8192),jitter(8192)};radius=262144+(int32_t)(random_word()%65536u);}
  else if(profile==3) {c=(sh2z_vec3_t){jitter(65536),jitter(65536),jitter(65536)};radius=262144+(int32_t)(random_word()%65536u);}
  else if(profile==4) {radius=INT32_C(8388608)+(int32_t)(random_word()%UINT32_C(4194304));}
  else {c.x+=jitter(131072);c.y+=jitter(131072);c.z+=jitter(131072);radius=profile==0?98304:65536+(int32_t)(random_word()%262145u);}
  if(profile!=2 && q%31u==0) radius=-1;
  workload_inputs.queries[q]=(sh2z_sphere_t){c,radius};
 }
}
uint32_t workload_prepare(uint32_t case_id,uint32_t sample) {
 if(case_id>=WORKLOAD_CASES || sample>=WORKLOAD_SAMPLES) return 0;
 const struct workload_case *c=&workload_cases[case_id];active_case=case_id;
 state=UINT32_C(0x6d2b79f5)^(sample+1)*UINT32_C(0x85ebca6b)^(c->profile+1)*UINT32_C(0x9e3779b9);
 memset(&workload_inputs,0,sizeof workload_inputs);memset(workload_outputs,0x5a,sizeof workload_outputs);
 if(c->group==W_SPATIAL) spatial_inputs(c->profile,c->size,sample);
 else {
  uint32_t vector_profile=c->profile<3?c->profile:0;
  for(uint32_t i=0;i<c->size;i++) workload_inputs.vectors[i]=(sh2z_vec3_t){value(vector_profile,i),value(vector_profile,i),value(vector_profile,i)};
  workload_inputs.rotation=(sh2z_quat_t){jitter(1048576),jitter(1048576),jitter(1048576),jitter(1048576)};
  if(c->profile==2) workload_inputs.rotation=(sh2z_quat_t){signed_bits(random_word()),signed_bits(random_word()),signed_bits(random_word()),signed_bits(random_word())};
  if(c->profile==0 && sample==0) workload_inputs.rotation=(sh2z_quat_t){32768,32768,32768,32768};
  if(c->profile==3) workload_inputs.rotation=(sh2z_quat_t){0,0,0,65536};
  if(c->profile==4) workload_inputs.rotation=(sh2z_quat_t){0,0,0,0};
 }
 return 1;
}
bool workload_plain_sphere_aabb(const sh2z_sphere_t *sphere,const sh2z_aabb_t *box) {
 if(sphere->radius<0)return false;
 struct plain_sphere prepared=plain_prepare(*sphere);return plain_sphere_aabb(&prepared,box);
}
uint32_t workload_run(uint32_t path) {
 const struct workload_case *c=&workload_cases[active_case];
 if(path>=WORKLOAD_PATHS) return 0;
 uint32_t *out=workload_outputs[path].words;uint32_t n=c->size;
 if(c->group==W_ROTATION) {
  sh2z_vec3_t *vout=workload_outputs[path].vectors;
  sh2z_quat_t q=workload_inputs.rotation;
  if(path==0) {for(uint32_t i=0;i<n;i++) vout[i]=prior_sh2z_quat_rotate(q,workload_inputs.vectors[i]);return 1;}
  if(path==1) {sh2z_mat34_t m=prior_sh2z_quat_to_mat34(q);return prior_sh2z_mat34_transform_points(vout,&m,workload_inputs.vectors,n);}
  if(path==2) {
   sh2z_mat34_t m=prior_sh2z_quat_to_mat34(q);sh2z_mesh_view_t mesh;sh2z_transform_plan_t plan;
   if(!sh2z_mesh_view_init(&mesh,workload_inputs.vectors,n)||!sh2z_transform_plan_init(&plan,&mesh,&m)) return 0;
   if(sh2z_transform_plan_mac_available(&plan)) return sh2z_transform_plan_execute_mac(vout,&plan,0,n);
   return prior_sh2z_mat34_transform_points(vout,&m,workload_inputs.vectors,n);
  }
  if(path==3) return sh2z_quat_rotate_vectors(vout,q,workload_inputs.vectors,n);
  return sh2z_quat_rotate_vectors_mac(vout,q,workload_inputs.vectors,n);
 }
 if(c->group==W_NORMALIZE) {
  if(path>1)return 0;
  for(uint32_t i=0;i<n;i++) {
   sh2z_vec3_t r=path==0?prior_sh2z_vec3_normalize(workload_inputs.vectors[i]):sh2z_vec3_normalize(workload_inputs.vectors[i]);
   memcpy(out+3*i,&r,sizeof r);
  }
  return 1;
 }
 if(c->group==W_ATAN2||c->group==W_ATAN) {
  if(path>1)return 0;
  for(uint32_t i=0;i<n;i++) {
   sh2z_vec3_t v=workload_inputs.vectors[i];
   out[i]=c->group==W_ATAN2?(path==0?prior_sh2z_atan2(v.y,v.x):sh2z_atan2(v.y,v.x))
                           :(path==0?prior_sh2z_atan(v.x):sh2z_atan(v.x));
  }
  return 1;
 }
 if(c->group==W_SPATIAL) {
  if(path>2)return 0;
  uint32_t words=(n+31u)/32u;
  if(path==2 && !sh2z_aabb_index_build(&index_view,workload_inputs.boxes,n,nodes,2*WORKLOAD_BOXES-1,scratch,WORKLOAD_BOXES)) return 0;
  for(uint32_t query=0;query<c->queries;query++) {
   uint32_t *bits=out+query*(words+1u);size_t matches=0;
   if(path==2) {
    if(!sh2z_aabb_index_query_sphere(bits,words,&matches,&index_view,workload_inputs.queries[query])) return 0;
   } else {
    if(workload_inputs.queries[query].radius<0) {
     for(uint32_t w=0;w<words;w++) bits[w]=0;
    } else {
     if(path==0) {
      for(uint32_t w=0;w<words;w++) {
       uint32_t mask=0,bit=1,end=32u*(w+1u);if(end>n)end=n;
       for(uint32_t i=32u*w;i<end;i++,bit<<=1) if(prior_sh2z_sphere_intersects_aabb(workload_inputs.queries[query],workload_inputs.boxes[i])) {
        mask|=bit;matches++;
       }
       bits[w]=mask;
      }
     } else {
      const struct plain_sphere sphere=plain_prepare(workload_inputs.queries[query]);
      for(uint32_t w=0;w<words;w++) {
       uint32_t mask=0,bit=1,end=32u*(w+1u);if(end>n)end=n;
       for(uint32_t i=32u*w;i<end;i++,bit<<=1) if(plain_sphere_aabb(&sphere,&workload_inputs.boxes[i])) {
        mask|=bit;matches++;
       }
       bits[w]=mask;
      }
     }
    }
   }
   bits[words]=(uint32_t)matches;
  }
  return 1;
 }
 return 0;
}
