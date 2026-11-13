/* Freestanding integer-only hardware smoke test. Zero means all checks passed.
 * Link this SAME source into a Saturn app, and display/log the returned bitmask.
 * No hardware results are implied by the host invocation.
 * MIT; Copyright (c) 2026 SH2ZAM Contributors. */
#include "math_examples.h"
#include <limits.h>
uint32_t sh2z_saturn_smoke(void) {
    uint32_t failures=0;
    if(sh2z_mul(-1,SH2Z_HALF)!=-1||sh2z_mul(INT32_MIN,SH2Z_ONE)!=INT32_MIN||sh2z_div(SH2Z_ONE,2*SH2Z_ONE)!=SH2Z_HALF) failures|=1;
    sh2z_vec3_t small=sh2z_vec3_normalize((sh2z_vec3_t){1,1,0});
    if(small.x<46340||small.x>46341||small.y<46340||small.y>46341) failures|=2;
    if(sh2z_vec3_length((sh2z_vec3_t){300*SH2Z_ONE,400*SH2Z_ONE,0})!=500*SH2Z_ONE) failures|=4;
    if(sh2z_vec2_dot((sh2z_vec2_t){INT32_MAX,INT32_MAX},(sh2z_vec2_t){INT32_MAX,-INT32_MAX})!=0) failures|=8;
    sh2z_mat34_t rotation=sh2z_mat34_rotation_z(SH2Z_ANGLE_QUARTER);
    sh2z_vec3_t rotated=sh2z_mat34_transform_point(&rotation,(sh2z_vec3_t){SH2Z_ONE,0,0});
    if(rotated.x||rotated.y!=SH2Z_ONE) failures|=16;
    sh2z_screen_t screen;
    if(!sh2z_project_clip(&screen,(sh2z_vec4_t){0,0,0,SH2Z_ONE},320,224)||screen.x!=159||screen.y!=111) failures|=32;
    if(!sh2z_example_batch()) failures|=64;
    sh2z_quat_t q=sh2z_quat_axis_angle((sh2z_vec3_t){0,SH2Z_ONE,0},SH2Z_ANGLE_HALF);
    rotated=sh2z_quat_rotate(q,(sh2z_vec3_t){SH2Z_ONE,0,0});
    if(sh2z_abs(sh2z_add(rotated.x,SH2Z_ONE))>8||sh2z_abs(rotated.z)>8) failures|=128;
    return failures;
}
