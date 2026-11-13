/* Frozen benchmark declarations; no change to the public API. MIT. */
#ifndef SH2Z_MATH_LEGACY_API_H
#define SH2Z_MATH_LEGACY_API_H
#include "sh2zam/sh2zam.h"
sh2z_fx_t legacy_sh2z_mul(sh2z_fx_t,sh2z_fx_t);
sh2z_fx_t legacy_sh2z_div(sh2z_fx_t,sh2z_fx_t);
sh2z_fx_t legacy_sh2z_muldiv(sh2z_fx_t,sh2z_fx_t,sh2z_fx_t);
sh2z_fx_t legacy_sh2z_recip(sh2z_fx_t);
sh2z_fx_t legacy_sh2z_sqrt(sh2z_fx_t);
sh2z_fx_t legacy_sh2z_rsqrt(sh2z_fx_t);
sh2z_fx_t legacy_sh2z_lerp(sh2z_fx_t,sh2z_fx_t,sh2z_fx_t);
sh2z_fx_t legacy_sh2z_smoothstep(sh2z_fx_t);
sh2z_fx_t legacy_sh2z_vec2_dot(sh2z_vec2_t,sh2z_vec2_t);
sh2z_fx_t legacy_sh2z_vec3_dot(sh2z_vec3_t,sh2z_vec3_t);
sh2z_fx_t legacy_sh2z_vec4_dot(sh2z_vec4_t,sh2z_vec4_t);
sh2z_fx_t legacy_sh2z_vec2_cross(sh2z_vec2_t,sh2z_vec2_t);
sh2z_vec3_t legacy_sh2z_vec3_cross(sh2z_vec3_t,sh2z_vec3_t);
sh2z_fx_t legacy_sh2z_vec2_length(sh2z_vec2_t);
sh2z_vec2_t legacy_sh2z_vec2_normalize(sh2z_vec2_t);
sh2z_fx_t legacy_sh2z_vec3_length(sh2z_vec3_t);
sh2z_vec3_t legacy_sh2z_vec3_normalize(sh2z_vec3_t);
sh2z_fx_t legacy_sh2z_vec4_length(sh2z_vec4_t);
sh2z_vec4_t legacy_sh2z_vec4_normalize(sh2z_vec4_t);
void legacy_sh2z_mat22_multiply(sh2z_mat22_t*,const sh2z_mat22_t*,const sh2z_mat22_t*);
void legacy_sh2z_mat33_multiply(sh2z_mat33_t*,const sh2z_mat33_t*,const sh2z_mat33_t*);
void legacy_sh2z_mat34_multiply(sh2z_mat34_t*,const sh2z_mat34_t*,const sh2z_mat34_t*);
void legacy_sh2z_mat44_multiply(sh2z_mat44_t*,const sh2z_mat44_t*,const sh2z_mat44_t*);
sh2z_vec2_t legacy_sh2z_mat22_transform(const sh2z_mat22_t*,sh2z_vec2_t);
sh2z_vec3_t legacy_sh2z_mat33_transform(const sh2z_mat33_t*,sh2z_vec3_t);
sh2z_vec3_t legacy_sh2z_mat34_transform_point(const sh2z_mat34_t*,sh2z_vec3_t);
sh2z_vec3_t legacy_sh2z_mat34_transform_direction(const sh2z_mat34_t*,sh2z_vec3_t);
sh2z_vec4_t legacy_sh2z_mat44_transform(const sh2z_mat44_t*,sh2z_vec4_t);
sh2z_quat_t legacy_sh2z_quat_multiply(sh2z_quat_t,sh2z_quat_t);
sh2z_quat_t legacy_sh2z_quat_normalize(sh2z_quat_t);
sh2z_quat_t legacy_sh2z_quat_nlerp(sh2z_quat_t,sh2z_quat_t,sh2z_fx_t);
sh2z_mat34_t legacy_sh2z_quat_to_mat34(sh2z_quat_t);
sh2z_vec3_t legacy_sh2z_quat_rotate(sh2z_quat_t,sh2z_vec3_t);
sh2z_quat_t legacy_sh2z_quat_axis_angle(sh2z_vec3_t,sh2z_angle_t);
sh2z_fx_t legacy_sh2z_sin(sh2z_angle_t);
sh2z_fx_t legacy_sh2z_cos(sh2z_angle_t);
void legacy_sh2z_sincos(sh2z_angle_t,sh2z_fx_t*,sh2z_fx_t*);
sh2z_fx_t legacy_sh2z_tan(sh2z_angle_t);
sh2z_angle_t legacy_sh2z_atan2(sh2z_fx_t,sh2z_fx_t);
sh2z_angle_t legacy_sh2z_atan(sh2z_fx_t);
sh2z_angle_t legacy_sh2z_asin(sh2z_fx_t);
sh2z_angle_t legacy_sh2z_acos(sh2z_fx_t);
bool legacy_sh2z_project_clip(sh2z_screen_t*,sh2z_vec4_t,uint16_t,uint16_t);
bool legacy_sh2z_mat34_inverse_rigid(sh2z_mat34_t*,const sh2z_mat34_t*);
bool legacy_sh2z_mat34_look_at(sh2z_mat34_t*,sh2z_vec3_t,sh2z_vec3_t,sh2z_vec3_t);
bool legacy_sh2z_mat44_perspective(sh2z_mat44_t*,sh2z_angle_t,sh2z_fx_t,sh2z_fx_t,sh2z_fx_t);
bool legacy_sh2z_sphere_intersects(sh2z_sphere_t,sh2z_sphere_t);
bool legacy_sh2z_sphere_intersects_aabb(sh2z_sphere_t,sh2z_aabb_t);
#endif
