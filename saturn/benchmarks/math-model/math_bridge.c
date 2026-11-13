/* SH2ZAM math suite: identical, deterministic inputs for old/new native code.
 * MIT; see root LICENSE. Fixture construction is outside measured ranges. */
#include "suite.h"
#include "legacy_api.h"
#include <limits.h>
#include <string.h>

#ifndef BENCH_COMMIT
#define BENCH_COMMIT "host-test-only"
#endif
const char math_source_commit[]=BENCH_COMMIT;
const char math_compiler[]=__VERSION__;
const char math_compile_flags[]="-m2 -mb -O2 -std=c11 -fno-fast-math -ffp-contract=off -fno-lto";

const struct math_case math_cases[M_COUNT]={
#include "cases.inc"
};
const uint32_t math_case_count=M_COUNT;
const uint32_t math_input_bytes=sizeof(struct math_input)*MATH_BATCH;
struct math_input math_inputs[MATH_BATCH];
uint32_t math_before_output[MATH_BATCH][MATH_WORDS];
uint32_t math_after_output[MATH_BATCH][MATH_WORDS];
static uint32_t math_operation;
static uint32_t state;
static uint32_t random_word(void) {
 state^=state<<13; state^=state>>17; state^=state<<5; return state;
}
/* Defined conversion even when unsigned -> signed would be implementation
 * defined. The resulting bit sequence is independent of host/target ABI. */
static int32_t signed_bits(uint32_t x) {
 return x<=INT32_MAX?(int32_t)x:INT32_MIN+(int32_t)(x-UINT32_C(0x80000000));
}
static int32_t fixture_value(uint32_t profile,uint32_t index) {
 static const int32_t edges[16]={0,1,-1,INT32_MIN,INT32_MAX,65536,-65536,
  32768,-32768,INT32_MIN+1,INT32_MAX-1,65535,-65535,2,-2,17};
 uint32_t r=random_word();
 if(profile==0) return (int32_t)(r%UINT32_C(1048577))-524288;
 if(profile==1) return (int32_t)(r%65u)-32;
 return index<8?edges[(r>>28)&15u]:signed_bits(r);
}
static int32_t positive(int32_t x) {return x==INT32_MIN?INT32_MAX:x<0?-x:x;}
static sh2z_vec3_t sorted_min(sh2z_vec3_t a,sh2z_vec3_t b) {
 return (sh2z_vec3_t){a.x<b.x?a.x:b.x,a.y<b.y?a.y:b.y,a.z<b.z?a.z:b.z};
}
static sh2z_vec3_t sorted_max(sh2z_vec3_t a,sh2z_vec3_t b) {
 return (sh2z_vec3_t){a.x>b.x?a.x:b.x,a.y>b.y?a.y:b.y,a.z>b.z?a.z:b.z};
}

uint32_t math_prepare(uint32_t operation,uint32_t profile,uint32_t sample) {
 if(operation>=M_COUNT || profile>=MATH_PROFILES || sample>=MATH_SAMPLES) return 0;
 math_operation=operation;
 state=UINT32_C(0x6d2b79f5) ^ (profile+1)*UINT32_C(0x9e3779b9)
       ^ (sample+1)*UINT32_C(0x85ebca6b);
 /* Initialize padding too, so the raw input checksum is reproducible. */
 memset(math_inputs,0,sizeof math_inputs);
 memset(math_before_output,0x5a,sizeof math_before_output);
 memset(math_after_output,0x5a,sizeof math_after_output);
 for(uint32_t i=0;i<MATH_BATCH;i++) {
  struct math_input *p=&math_inputs[i];
  p->a=fixture_value(profile,i); p->b=fixture_value(profile,i);
  p->c=fixture_value(profile,i); p->t=(int32_t)(random_word()%131073u)-32768;
  if((i&15u)==0) p->b=0;
  if((i&15u)==1) p->c=0;
  p->root=positive(fixture_value(profile,i));
  if(i==0) p->root=0;
  if(i==1) p->root=-1;
  p->unit=profile==2?fixture_value(profile,i):(int32_t)(random_word()%131073u)-65536;
  p->angle=(sh2z_angle_t)random_word();
  if(i<4) p->angle=(sh2z_angle_t)(i*16384u);
  p->v2=(sh2z_vec2_t){fixture_value(profile,i),fixture_value(profile,i)};
  p->w2=(sh2z_vec2_t){fixture_value(profile,i),fixture_value(profile,i)};
  p->v3=(sh2z_vec3_t){fixture_value(profile,i),fixture_value(profile,i),fixture_value(profile,i)};
  p->w3=(sh2z_vec3_t){fixture_value(profile,i),fixture_value(profile,i),fixture_value(profile,i)};
  p->v4=(sh2z_vec4_t){fixture_value(profile,i),fixture_value(profile,i),fixture_value(profile,i),fixture_value(profile,i)};
  p->w4=(sh2z_vec4_t){fixture_value(profile,i),fixture_value(profile,i),fixture_value(profile,i),fixture_value(profile,i)};
  p->q=(sh2z_quat_t){p->v4.x,p->v4.y,p->v4.z,p->v4.w};
  p->r=(sh2z_quat_t){p->w4.x,p->w4.y,p->w4.z,p->w4.w};
  if(i==0) {
   p->v2=(sh2z_vec2_t){0,0}; p->v3=(sh2z_vec3_t){0,0,0};
   p->v4=(sh2z_vec4_t){0,0,0,0}; p->q=(sh2z_quat_t){0,0,0,0};
  }
  for(unsigned r=0;r<2;r++) for(unsigned c=0;c<2;c++) {
   p->a22.m[r][c]=fixture_value(profile,i); p->b22.m[r][c]=fixture_value(profile,i);
  }
  for(unsigned r=0;r<3;r++) for(unsigned c=0;c<3;c++) {
   p->a33.m[r][c]=fixture_value(profile,i); p->b33.m[r][c]=fixture_value(profile,i);
  }
  for(unsigned r=0;r<3;r++) for(unsigned c=0;c<4;c++) {
   p->a34.m[r][c]=fixture_value(profile,i); p->b34.m[r][c]=fixture_value(profile,i);
  }
  for(unsigned r=0;r<4;r++) for(unsigned c=0;c<4;c++) {
   p->a44.m[r][c]=fixture_value(profile,i); p->b44.m[r][c]=fixture_value(profile,i);
  }
  /* Cardinal rotations are exactly orthonormal. One quarter of inputs is
   * deliberately invalid, exercising the unchanged failure contract. */
  int32_t c=(i&1u)?0:((i&2u)?-65536:65536);
  int32_t s=(i&1u)?((i&2u)?-65536:65536):0;
  p->rigid=(sh2z_mat34_t){{{c,-s,0,p->v3.x},{s,c,0,p->v3.y},{0,0,65536,p->v3.z}}};
  if((i&3u)==3) p->rigid.m[0][0]=p->a34.m[0][0];
  int32_t w=profile==1?(int32_t)(1u+random_word()%32u)
             :profile==2?INT32_MAX-(int32_t)(random_word()&65535u)
             :65536+(int32_t)(random_word()%524288u);
  uint32_t span=(uint32_t)w;
  int32_t x=(int32_t)(random_word()%span),y=(int32_t)(random_word()%span);
  if(i&1u) x=-x;
  if(i&2u) y=-y;
  p->clip=(sh2z_vec4_t){x,y,w/2,w};
  if((i&7u)==0) p->clip.w=0;
  if((i&7u)==1) p->clip.z=-w;
  p->fov=(sh2z_angle_t)(4096u+random_word()%24576u);
  p->aspect=65536+(int32_t)(random_word()%65536u);
  p->near_z=256+(int32_t)(random_word()%65536u);
  p->far_z=p->near_z+65536+(int32_t)(random_word()%6553600u);
  if((i&7u)==0) p->aspect=0;
  p->s1=(sh2z_sphere_t){p->v3,positive(fixture_value(profile,i))};
  p->s2=(sh2z_sphere_t){p->w3,positive(fixture_value(profile,i))};
  if(i==1) p->s1.radius=-1;
  p->box=(sh2z_aabb_t){sorted_min(p->v3,p->w3),sorted_max(p->v3,p->w3)};
  /* Place most sphere/AABB probes away from the box's defining corner. */
  p->s1.center=(sh2z_vec3_t){p->v4.x,p->v4.y,p->v4.z};
 }
 return 1;
}

#define CALL(name) legacy_sh2z_##name
#define RANGE math_before_range
#define OUTPUT math_before_output
#include "workloads.inc"
#undef OUTPUT
#undef RANGE
#undef CALL
#define CALL(name) sh2z_##name
#define RANGE math_after_range
#define OUTPUT math_after_output
#include "workloads.inc"
