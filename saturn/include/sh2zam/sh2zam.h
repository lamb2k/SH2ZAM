/* SH2ZAM: Saturn-native fixed-point math (MIT; see root LICENSE).
 * Independent SH-2 implementation inspired by SH4ZAM's public API categories.
 * Copyright (c) 2026 SH2ZAM Contributors.
 */
#ifndef SH2ZAM_H
#define SH2ZAM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef int32_t sh2z_fx_t;       /* Signed Q16.16, 1.0 == 65536 */
typedef uint16_t sh2z_angle_t;   /* Unsigned turns: 65536 == 360 degrees */
#define SH2Z_ONE ((sh2z_fx_t)65536)
#define SH2Z_HALF ((sh2z_fx_t)32768)
#define SH2Z_ZERO ((sh2z_fx_t)0)
#define SH2Z_ANGLE_QUARTER ((sh2z_angle_t)16384)
#define SH2Z_ANGLE_HALF ((sh2z_angle_t)32768)

typedef struct { sh2z_fx_t x, y; } sh2z_vec2_t;
typedef struct { sh2z_fx_t x, y, z; } sh2z_vec3_t;
typedef struct { sh2z_fx_t x, y, z, w; } sh2z_vec4_t;
typedef struct { sh2z_fx_t m[3][4]; } sh2z_mat34_t; /* row-major affine, column vectors */
typedef struct { sh2z_fx_t m[4][4]; } sh2z_mat44_t; /* row-major, column vectors */
typedef struct { sh2z_fx_t x, y, z, w; } sh2z_quat_t; /* x,y,z,w */

/* Arithmetic saturates on overflow. Multiply rounds toward -infinity;
 * division rounds toward zero. Division by zero saturates by sign (0/0 = 0).
 * No runtime floating-point, allocations, or global mutable state. */
sh2z_fx_t sh2z_from_int(int32_t number);
int32_t sh2z_to_int(sh2z_fx_t number); /* truncates toward zero */
sh2z_fx_t sh2z_add(sh2z_fx_t a, sh2z_fx_t b);
sh2z_fx_t sh2z_sub(sh2z_fx_t a, sh2z_fx_t b);
sh2z_fx_t sh2z_mul(sh2z_fx_t a, sh2z_fx_t b);
sh2z_fx_t sh2z_div(sh2z_fx_t numerator, sh2z_fx_t denominator);
sh2z_fx_t sh2z_sqrt(sh2z_fx_t value); /* negative -> zero */
sh2z_angle_t sh2z_angle_from_degrees(int32_t degrees);
sh2z_fx_t sh2z_sin(sh2z_angle_t angle);
sh2z_fx_t sh2z_cos(sh2z_angle_t angle);

sh2z_fx_t sh2z_vec2_dot(sh2z_vec2_t a, sh2z_vec2_t b);
sh2z_fx_t sh2z_vec3_dot(sh2z_vec3_t a, sh2z_vec3_t b);
sh2z_fx_t sh2z_vec4_dot(sh2z_vec4_t a, sh2z_vec4_t b);
sh2z_vec3_t sh2z_vec3_add(sh2z_vec3_t a, sh2z_vec3_t b);
sh2z_vec3_t sh2z_vec3_scale(sh2z_vec3_t a, sh2z_fx_t scale);
sh2z_vec3_t sh2z_vec3_cross(sh2z_vec3_t a, sh2z_vec3_t b);
sh2z_fx_t sh2z_vec3_length(sh2z_vec3_t vector);
sh2z_vec3_t sh2z_vec3_normalize(sh2z_vec3_t vector);

sh2z_mat34_t sh2z_mat34_identity(void);
void sh2z_mat34_multiply(sh2z_mat34_t *out, const sh2z_mat34_t *a,
                          const sh2z_mat34_t *b); /* out may alias a/b */
sh2z_vec3_t sh2z_mat34_transform_point(const sh2z_mat34_t *matrix, sh2z_vec3_t point);
sh2z_vec3_t sh2z_mat34_transform_direction(const sh2z_mat34_t *matrix, sh2z_vec3_t direction);
sh2z_mat44_t sh2z_mat44_identity(void);
void sh2z_mat44_multiply(sh2z_mat44_t *out, const sh2z_mat44_t *a,
                          const sh2z_mat44_t *b); /* out may alias a/b */
sh2z_vec4_t sh2z_mat44_transform(const sh2z_mat44_t *matrix, sh2z_vec4_t vector);

sh2z_quat_t sh2z_quat_identity(void);
sh2z_quat_t sh2z_quat_multiply(sh2z_quat_t a, sh2z_quat_t b);
sh2z_quat_t sh2z_quat_normalize(sh2z_quat_t q);

#ifdef __cplusplus
}
#endif
#endif
