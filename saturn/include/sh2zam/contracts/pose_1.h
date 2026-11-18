/* SPDX-License-Identifier: MIT */
/* ---- Contract tag pose-1 ------------------------------------------------- */
/* A model's pose (view, model-space eye, Q1.14 basis, plane-culling data)
 * from binary32 model state with no floating-point arithmetic. Reached
 * through sh2zam/view_experimental.h or included directly. */
#ifndef SH2ZAM_CONTRACTS_POSE_1_H
#define SH2ZAM_CONTRACTS_POSE_1_H

#include "../view_experimental.h"

#ifdef __cplusplus
extern "C" {
#endif

/** A model's view and culling data, built by sh2z_pose_from_f32.
 *
 *  view        model-to-view: r in Q1.14 (camera rotation times the scaled
 *              model basis), t in Q18.14 model units; every other field is
 *              the base view's.
 *  eye         the camera position in whole model units, on model axes.
 *  basis       [world][model] unscaled model-to-world basis, Q1.14.
 *  eye_error   always 2: model units per axis that eye may be off by, for
 *              plane culling (meaningful only when cull_valid).
 *  octant      bit j set when eye[j] >= 0.
 *  cull_valid  the basis is orthonormal to 512/2^30 per Gram entry and every
 *              |eye[j]| <= 30000. */
typedef struct {
    sh2z_view_t view;
    int32_t eye[3];
    int16_t basis[3][3];
    uint16_t eye_error;
    uint8_t octant;
    bool cull_valid;
} sh2z_pose_t;

/** Model axis j reads basis row 2-j and scale 2-j (a model stored Z, Y, X). */
#define SH2Z_POSE_REVERSE_AXES 1u

/** Build a model's pose from binary32 model state with no floating-point
 *  arithmetic: every float is decoded with integer operations. The result
 *  equals, field for field and in success or failure, the procedure below
 *  evaluated with IEEE 754 binary32 operations (round to nearest even,
 *  subnormals kept) and exact integer arithmetic elsewhere. Every output is
 *  therefore within 0 LSB of that procedure, and vertices projected through
 *  view are identical.
 *
 *  Inputs. rotation: the camera's world-to-view rotation rows, Q2.30.
 *  basis[a][k]: world component k of stored model axis a. scale[a]: the scale
 *  of stored model axis a. position, eye: the model origin and the camera, in
 *  world units. a(j) = j, or 2 - j with SH2Z_POSE_REVERSE_AXES. base: any
 *  valid view; only its focal, centre, near/far and clip fields are read.
 *
 *  With fl() one binary32 operation and trunc() toward zero:
 *      D[k]    = trunc(65536 * fl(position[k] - eye[k]))
 *      B[k][j] = trunc(2^30 * basis[a(j)][k])
 *      M[k][j] = trunc(65536 * fl(basis[a(j)][k] * scale[a(j)]))
 *      I[j]    = trunc(65536 * fl(1 / scale[a(j)]))
 *      R_n(x)  = sign(x) * floor((|x| + 2^(n-1)) / 2^n)   (ties away from 0)
 *  then
 *      view.r[i][j] = R_32(sum_k rotation[i][k] * M[k][j])
 *      view.t[i]    = R_32(sum_k rotation[i][k] * D[k])
 *      basis[k][j]  = R_16(B[k][j])
 *      eye[j]       = R_32(R_30(-sum_k B[k][j] * D[k]) * I[j])
 *      octant bit j = (eye[j] >= 0),  eye_error = 2
 *      cull_valid   = |R_30(sum_k B[k][i] * B[k][j]) - 2^30 [i == j]| <= 512
 *                     for all i, j, and |eye[j]| <= 30000 for all j.
 *
 *  It fails, leaving *out unchanged, unless all of these hold: no pointer is
 *  null and flags has no bit but SH2Z_POSE_REVERSE_AXES;
 *  |rotation[i][j]| <= 1,100,000,000; every basis value is finite with
 *  |basis| <= 1.00001f (bits 0x3F800054); every scale is in [0.0625, 1], so
 *  NaN, infinite, zero, subnormal and negative scales fail; position and eye
 *  are finite and |fl(position[k] - eye[k])| <= 4096, so an overflowing
 *  difference fails; every view.r fits int16; and the resulting view passes
 *  sh2z_view_init. A position or eye of any finite magnitude is accepted when
 *  the difference is in range. Zero and subnormal inputs follow binary32
 *  exactly: a subnormal basis value gives B = M = 0, subnormal positions
 *  subtract as binary32 does, and -0 behaves as +0. Every input but base is
 *  read before out is written, so out may overlap any of them; base may be
 *  &out->view (the fields it supplies are copied onto themselves) but must
 *  not otherwise overlap *out. */
bool sh2z_pose_from_f32(sh2z_pose_t *out,const sh2z_view_t *base,
    const int32_t rotation[3][3],const float basis[3][3],const float scale[3],
    const float position[3],const float eye[3],unsigned flags);

#ifdef __cplusplus
}
#endif

#endif /* SH2ZAM_CONTRACTS_POSE_1_H */
