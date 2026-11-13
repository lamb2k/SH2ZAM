/* Request-start library symbols are renamed only in benchmark wrappers. MIT. */
#ifndef SH2Z_WORKLOAD_PRIOR_API_H
#define SH2Z_WORKLOAD_PRIOR_API_H
#include "sh2zam/sh2zam.h"
sh2z_vec3_t prior_sh2z_quat_rotate(sh2z_quat_t q,sh2z_vec3_t v);
sh2z_mat34_t prior_sh2z_quat_to_mat34(sh2z_quat_t q);
bool prior_sh2z_mat34_transform_points(sh2z_vec3_t *out,const sh2z_mat34_t *m,const sh2z_vec3_t *in,size_t count);
sh2z_vec3_t prior_sh2z_vec3_normalize(sh2z_vec3_t v);
sh2z_angle_t prior_sh2z_atan2(sh2z_fx_t y,sh2z_fx_t x);
sh2z_angle_t prior_sh2z_atan(sh2z_fx_t x);
bool prior_sh2z_sphere_intersects_aabb(sh2z_sphere_t sphere,sh2z_aabb_t box);
#endif
