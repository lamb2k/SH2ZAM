/* Deterministic SH-2 model workloads. MIT; see root LICENSE. */
#ifndef SH2Z_MATH_SUITE_H
#define SH2Z_MATH_SUITE_H
#include "sh2zam/sh2zam.h"
#define MATH_BATCH 32u
#define MATH_SAMPLES 7u
#define MATH_PROFILES 3u
#define MATH_WORDS 17u
enum math_operation {
 M_MUL,
 M_DIV,
 M_MULDIV,
 M_RECIP,
 M_SQRT,
 M_RSQRT,
 M_LERP,
 M_SMOOTHSTEP,
 M_VEC2_DOT,
 M_VEC3_DOT,
 M_VEC4_DOT,
 M_VEC2_CROSS,
 M_VEC3_CROSS,
 M_VEC2_LENGTH,
 M_VEC2_NORMALIZE,
 M_VEC3_LENGTH,
 M_VEC3_NORMALIZE,
 M_VEC4_LENGTH,
 M_VEC4_NORMALIZE,
 M_MAT22_MULTIPLY,
 M_MAT33_MULTIPLY,
 M_MAT34_MULTIPLY,
 M_MAT44_MULTIPLY,
 M_MAT22_TRANSFORM,
 M_MAT33_TRANSFORM,
 M_MAT34_TRANSFORM_POINT,
 M_MAT34_TRANSFORM_DIRECTION,
 M_MAT44_TRANSFORM,
 M_QUAT_MULTIPLY,
 M_QUAT_NORMALIZE,
 M_QUAT_NLERP,
 M_QUAT_TO_MAT34,
 M_QUAT_ROTATE,
 M_QUAT_AXIS_ANGLE,
 M_SIN,
 M_COS,
 M_SINCOS,
 M_TAN,
 M_ATAN2,
 M_ATAN,
 M_ASIN,
 M_ACOS,
 M_PROJECT_CLIP,
 M_MAT34_INVERSE_RIGID,
 M_MAT34_LOOK_AT,
 M_MAT44_PERSPECTIVE,
 M_SPHERE_INTERSECTS,
 M_SPHERE_INTERSECTS_AABB,
 M_COUNT
};
struct math_input {
 sh2z_fx_t a,b,c,t,root,unit;
 sh2z_angle_t angle,fov;
 sh2z_fx_t aspect,near_z,far_z;
 sh2z_vec2_t v2,w2;
 sh2z_vec3_t v3,w3;
 sh2z_vec4_t v4,w4,clip;
 sh2z_mat22_t a22,b22;
 sh2z_mat33_t a33,b33;
 sh2z_mat34_t a34,b34,rigid;
 sh2z_mat44_t a44,b44;
 sh2z_quat_t q,r;
 sh2z_sphere_t s1,s2;
 sh2z_aabb_t box;
};
struct math_case { const char *name; uint32_t words; };
extern const struct math_case math_cases[M_COUNT];
extern const uint32_t math_case_count,math_input_bytes;
extern struct math_input math_inputs[MATH_BATCH];
extern uint32_t math_before_output[MATH_BATCH][MATH_WORDS];
extern uint32_t math_after_output[MATH_BATCH][MATH_WORDS];
uint32_t math_prepare(uint32_t operation,uint32_t profile,uint32_t sample);
uint32_t math_before_range(uint32_t first,uint32_t count);
uint32_t math_after_range(uint32_t first,uint32_t count);
#endif
