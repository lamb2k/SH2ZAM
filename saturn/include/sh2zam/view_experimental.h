/* Experimental integer view transform, projection and clip codes for SH-2
 * renderers. Not selected by any default dispatch. MIT; SH2ZAM Contributors.
 *
 * Contract tag: view-contract-1. Struct layouts and the numeric definitions
 * below are frozen at that tag. A faster path may replace the implementation
 * only if it produces identical bytes; new behaviour gets a new function. */
#ifndef SH2ZAM_VIEW_EXPERIMENTAL_H
#define SH2ZAM_VIEW_EXPERIMENTAL_H

#include "sh2zam.h"

#ifdef __cplusplus
extern "C" {
#endif

/** A model-space position in whole model units. Host byte order. */
typedef struct { int16_t x,y,z; } sh2z_pos16_t;

/** Camera for sh2z_view_batch.
 *
 *  r       rotation rows, Q1.14 (16384 is one). Any values are accepted if the
 *          bound below holds; an orthonormal Q1.14 rotation always does.
 *  t       translation, Q18.14 model units. Compute it with sh2z_view_place
 *          from the same quantized r, so that view = r * (p - eye) exactly up
 *          to the eye's rounding and error grows with distance from the
 *          camera rather than from the world origin.
 *  focal_x, focal_y  pixels per unit of x/z and y/z, Q16.16, both > 0.
 *  center_x, center_y  pixel position of the view axis.
 *  near_z, far_z     Q18.14, 0 < near_z < far_z.
 *  clip    inclusive pixel rectangle {left, top, right, bottom}; left <= right
 *          and top <= bottom.
 *
 *  Validity also requires, for every row i:
 *      32768 * (|r[i][0]| + |r[i][1]| + |r[i][2]|) + |t[i]| <= INT32_MAX
 *  which proves the 32-bit transform below cannot wrap for any int16 input. */
typedef struct {
    int16_t r[3][3];
    int32_t t[3];
    int32_t focal_x,focal_y;
    int16_t center_x,center_y;
    int32_t near_z,far_z;
    int16_t clip[4];
} sh2z_view_t;

#define SH2Z_OUT_NEAR   0x01u
#define SH2Z_OUT_FAR    0x02u
#define SH2Z_OUT_LEFT   0x04u
#define SH2Z_OUT_RIGHT  0x08u
#define SH2Z_OUT_TOP    0x10u
#define SH2Z_OUT_BOTTOM 0x20u

/** One transformed vertex (20 bytes).
 *
 *  x, y, z  view space, Q18.14; z > 0 is in front of the camera.
 *  sx, sy   pixels; both 0 when SH2Z_OUT_NEAR is set.
 *  outcode  SH2Z_OUT_* bits. A NEAR vertex carries no side bits, so the AND
 *           of a polygon's outcodes is a conservative trivial reject.
 *  reserved always written as zero. */
typedef struct {
    int32_t x,y,z;
    int16_t sx,sy;
    uint8_t outcode;
    uint8_t reserved[3];
} sh2z_view_vertex_t;

/** Copy *in to *out if it is a valid view (see sh2z_view_t). On failure out is
 *  unchanged. in and out may alias. */
bool sh2z_view_init(sh2z_view_t *out,const sh2z_view_t *in);

/** Set v->t from an eye position given in Q16.16 model units, using v's
 *  current rotation:
 *      t[i] = -floor((r[i][0]*eye.x + r[i][1]*eye.y + r[i][2]*eye.z) / 65536)
 *  computed exactly in 64 bits. Fails, leaving v unchanged, if a result does
 *  not fit int32 or the resulting view would be invalid. Other fields of v
 *  must already be valid. */
bool sh2z_view_place(sh2z_view_t *v,sh2z_vec3_t eye);

/** Transform, project and classify count positions.
 *
 *  Input: position k is three int16 values at
 *  (const char *)positions + k * in_stride. positions must be 2-byte aligned,
 *  in_stride even and >= 6. Output: out[0..count-1].
 *
 *  Definition, for each position p:
 *      view[i] = r[i][0]*p.x + r[i][1]*p.y + r[i][2]*p.z + t[i]   (exact, 32-bit)
 *      if view.z < near_z:
 *          outcode = NEAR, sx = sy = 0
 *      else:
 *          qx = sat32(trunc((int64)focal_x * view.x / view.z))
 *          qy = sat32(trunc((int64)focal_y * view.y / view.z))
 *          sx = sat16(center_x + (qx >> 16))     (>> is arithmetic: floor)
 *          sy = sat16(center_y - (qy >> 16))
 *          outcode = FAR if view.z > far_z
 *                  | LEFT if sx < clip[0] | RIGHT if sx > clip[2]
 *                  | TOP if sy < clip[1]  | BOTTOM if sy > clip[3]
 *  sat32/sat16 clamp to the type's range.
 *
 *  Returns false, writing nothing, if v is invalid, count > 0 with a null
 *  pointer, positions or in_stride violate the rules above, or the output
 *  overlaps the input bytes. count == 0 succeeds and permits null pointers.
 *  No other failure exists: every valid input has a defined result.
 *  No global state, allocation, floating point or hardware divider use;
 *  reentrant. */
bool sh2z_view_batch(sh2z_view_vertex_t *out,const sh2z_view_t *v,
    const void *positions,size_t in_stride,size_t count);

#ifdef __cplusplus
}
#endif
#endif
