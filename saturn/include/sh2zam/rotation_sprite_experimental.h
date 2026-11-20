/* Experimental rotation sprite sets for SH-2: draw a 3D object as a pre-rendered
 * view of itself. A set holds cells, each an image of the object seen from one
 * direction (a yaw x pitch grid around its own up axis). At run time the
 * caller picks the cell nearest the actual viewing direction, places it at the
 * object's projected origin and emits one VDP1 distorted sprite that scales
 * the cell with depth and rotates it with the object's on-screen roll.
 *
 * Contract tag: rotation-sprites-2 (2 adds the sub-pixel anchor: placement keeps
 * the fraction it used to drop and emission rounds each corner once, so an
 * object held still on screen no longer steps a pixel with rounding noise).
 * Contract tag rotation-sprites-3 (at the end) adds faster forms with
 * identical output: a yaw climb for convex tables, the world pose from a
 * caller's camera-space terms and the emission of one cell.
 * Every function below is a pure function of
 * its inputs as stated. Selection is exact for the stated rule; placement and
 * emission round as stated (no floating point). Nothing here knows what the
 * object is: the set format, the angle bins and the pixel data belong to the
 * caller's offline baker.
 *
 * No function uses floating point, the hardware divider, global state, libc
 * or allocation; all are reentrant. sh2z_rspr_place divides 64-bit integers
 * with the compiler's own routine (four divisions per call). All integers are in the CPU's own byte
 * order: a set read from a big-endian file is used in place on the SH-2.
 *
 * Model frame. The object's own axes: +y is its up axis, x and z span its
 * horizontal plane. The viewing direction is the camera position minus the
 * object origin, in that frame and in any consistent unit.
 *   yaw   = atan2(x, z): 0 when the camera is on +z, 90 degrees on +x.
 *   pitch = atan2(y, sqrt(x*x + z*z)): positive when the camera is above.
 * A mirrored set (SH2Z_RSPR_MIRROR) stores yaw 0..180 degrees only; a camera
 * at negative x uses the cell for |x| drawn mirrored left to right. This is
 * exact only for objects whose image is symmetric about the y-z plane.
 *
 * Set layout. One contiguous block, so it can be copied to VDP1 VRAM as is:
 *   sh2z_rspr_set_t header, then yaw_count*pitch_count sh2z_rspr_cell_t
 *   (pitch-major: cell index = pitch*yaw_count + yaw), then CLUTs and pixels
 *   at the offsets the header and cells give. Pixels are 4-bit indexed
 *   (VDP1 colour lookup table mode), row-major, two pixels per byte with the
 *   left pixel in the high nibble; index 0 is transparent.
 *
 * Copyright (c) 2026 SH2ZAM Contributors. MIT License.
 * Inspired by SH4ZAM by Falco Girgis and SH4ZAM Contributors.
 */
#ifndef SH2ZAM_ROTATION_SPRITE_EXPERIMENTAL_H
#define SH2ZAM_ROTATION_SPRITE_EXPERIMENTAL_H

#include "sh2zam.h"
#include "sh2zam/view_experimental.h"
#include "sh2zam/divu_experimental.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SH2Z_RSPR_MAGIC      0x52535031u /* "RSP1" */
#define SH2Z_RSPR_MAX_YAW    32u
#define SH2Z_RSPR_MAX_PITCH  4u
#define SH2Z_RSPR_MIRROR     0x0001u

/** One cell: a 4-bit indexed image of the object seen from one direction.
 *
 *  width, height    pixels; width is a multiple of 8, 8..504; height 1..255
 *                   (the VDP1 character size limits).
 *  origin_x, origin_y  cell pixel (from the top left) of the object origin.
 *  pixel_offset     bytes from the start of the set; a multiple of 8.
 *  clut             CLUT index; the CLUT is at clut_offset + clut*32.
 */
typedef struct {
    uint16_t width,height;
    int16_t origin_x,origin_y;
    uint32_t pixel_offset;
    uint16_t clut,reserved;
} sh2z_rspr_cell_t;

/** Set header, followed directly by the cells.
 *
 *  magic            SH2Z_RSPR_MAGIC.
 *  bytes            size of the whole set, header included.
 *  yaw_count        1..SH2Z_RSPR_MAX_YAW yaw bins.
 *  pitch_count      1..SH2Z_RSPR_MAX_PITCH pitch bins.
 *  flags            SH2Z_RSPR_MIRROR or 0.
 *  clut_count       1..1024 CLUTs of 16 RGB555 words (32 bytes) each.
 *  clut_offset      bytes from the start of the set; a multiple of 8.
 *  pixels_per_unit  cell pixels per model unit at the bake distance, Q16.16, > 0.
 *  yaw              bin centre direction {sin, cos} of each yaw, Q1.14.
 *  pitch            bin centre direction {sin, cos} of each pitch, Q1.14.
 */
typedef struct {
    uint32_t magic,bytes;
    uint16_t yaw_count,pitch_count,flags,clut_count;
    uint32_t clut_offset;
    int32_t pixels_per_unit;
    int16_t yaw[SH2Z_RSPR_MAX_YAW][2];
    int16_t pitch[SH2Z_RSPR_MAX_PITCH][2];
} sh2z_rspr_set_t;

/** The chosen cell: index into the cell array and whether to mirror it. */
typedef struct {
    uint16_t cell;
    uint8_t mirror,reserved;
} sh2z_rspr_pick_t;

/** Where and how large to draw a cell.
 *
 *  x, y     screen position of the object origin (VDP1 coordinates): the
 *           whole pixel at or left of / above it (floor).
 *  frac     the rest of that position, {x, y}, in 1/65536 pixel (0..65535):
 *           the origin is at x + frac[0] / 65536, y + frac[1] / 65536.
 *           A place built by hand may leave it 0.
 *  scale    screen pixels per cell pixel, Q16.16, > 0.
 *  up       the object's up axis as a unit screen direction {x, y}, Q1.14,
 *           y pointing down the screen ({0, -16384} is upright).
 */
typedef struct {
    int16_t x,y;
    int32_t scale;
    int16_t up[2];
    uint16_t frac[2];
} sh2z_rspr_place_t;

/** Check a set before use: magic, counts, flags, offsets, sizes, alignment and
 *  that every cell's pixels and every CLUT lie inside bytes. True if valid. */
bool sh2z_rspr_valid(const sh2z_rspr_set_t *set);

/** Return the cell array that follows the header. */
static inline const sh2z_rspr_cell_t *sh2z_rspr_cells(const sh2z_rspr_set_t *set) {
    return (const sh2z_rspr_cell_t *)(const void *)(set+1);
}

/** Pick the cell for a viewing direction {x, y, z} (camera minus origin, model
 *  frame, any unit; not all zero).
 *
 *  The inputs are first shifted right arithmetically by the smallest s >= 0
 *  that brings every |component| below 2^15. With mirror, x is replaced by |x|
 *  and pick.mirror = (x < 0). Then
 *      yaw   = the bin k maximising yaw[k].sin*x + yaw[k].cos*z
 *      h     = (yaw[yaw].sin*x + yaw[yaw].cos*z) >> 14
 *      pitch = the bin k maximising pitch[k].sin*y + pitch[k].cos*h
 *  (ties go to the lower bin) and pick.cell = pitch*yaw_count + yaw. h is the
 *  horizontal length seen through the chosen yaw bin, so no square root is
 *  needed. Returns false, leaving *out unchanged, for an invalid set, a zero
 *  direction or a null pointer. */
bool sh2z_rspr_select(sh2z_rspr_pick_t *out,const sh2z_rspr_set_t *set,
                      int32_t x,int32_t y,int32_t z);

/** Place a set drawn for model view v (v->r includes the object's model
 *  scale; v->t is the object origin in view space; see sh2z_view_t):
 *      X     = v->center_x * 65536 + trunc(focal_x * t.x / t.z)
 *      Y     = v->center_y * 65536 - trunc(focal_y * t.y / t.z)
 *      x, y  = floor(X / 65536), floor(Y / 65536); frac = X, Y mod 65536
 *  (X, Y: the origin in 1/65536 pixel; x is sh2z_view_batch's whole pixel). With U = the model up column
 *  {r[0][1], r[1][1]} and |U| = isqrt(U.x^2 + U.y^2) (floor):
 *      up    = {16384 * U.x / |U|, -16384 * U.y / |U|}       (truncated)
 *      scale = trunc(trunc(focal_y * L / t.z) * 65536 / pixels_per_unit)
 *  where L = isqrt(r[0][1]^2 + r[1][1]^2 + r[2][1]^2) (floor) is the length of
 *  one model unit in view units, Q1.14. Returns false, leaving *out unchanged, when
 *  t.z <= v->near_z, when the origin's screen position or scale does not fit
 *  (|x|,|y| > 1023 or scale outside 1..2^24), when the up axis points along
 *  the view direction (|U| == 0), or for null pointers. */
bool sh2z_rspr_place(sh2z_rspr_place_t *out,const sh2z_view_t *v,const sh2z_rspr_set_t *set);

/** Write one VDP1 distorted sprite command for the picked cell as sixteen
 *  16-bit words in VDP1 order (CTRL, LINK, PMOD, COLR, SRCA, SIZE, XA, YA,
 *  XB, YB, XC, YC, XD, YD, GRDA, 0).
 *
 *  vram     byte address of the set in VDP1 VRAM (0..0x7ffff, multiple of 8).
 *  pmod     the draw mode word; the caller sets colour mode 1 (16-colour
 *           lookup table) and any clipping, mesh or end code bits.
 *
 *  Corners. With c = the cell, R = {-up.y, up.x} and a cell pixel (px, py)
 *  measured from the origin (px = column - origin_x, py = row - origin_y;
 *  px is negated when mirrored), the screen point is
 *      {x, y} + floor((frac * 2^14 + scale * (px * R - py * up)) / 2^30 + 1/2)
 *  (one rounding from the sub-pixel origin)
 *  for the four cell corners A = (0, 0), B = (width, 0), C = (width, height)
 *  and D = (0, height) in column/row pixels. CTRL = 0x0002 (distorted
 *  sprite, next command follows), LINK = 0, GRDA = 0, COLR = (vram +
 *  clut_offset + clut*32) / 8, SRCA = (vram + pixel_offset) / 8 and SIZE =
 *  (width / 8) << 8 | height. Returns false, writing nothing, for a bad pick,
 *  a vram address that is misaligned or puts the cell outside VDP1 VRAM, a
 *  corner outside -1024..1023, or null pointers. */
bool sh2z_rspr_emit(uint16_t command[16],const sh2z_rspr_set_t *set,const sh2z_rspr_pick_t *pick,
                    const sh2z_rspr_place_t *place,uint32_t vram,uint16_t pmod);

/** sh2z_rspr_emit for a set the caller has already accepted with
 *  sh2z_rspr_valid (once, at load): identical output and rejections except
 *  that the set itself is not re-checked on each call. */
bool sh2z_rspr_emit_valid(uint16_t command[16],const sh2z_rspr_set_t *set,const sh2z_rspr_pick_t *pick,
                          const sh2z_rspr_place_t *place,uint32_t vram,uint16_t pmod);

/* ---- World-pose fast path (same contract tag) ----
 *
 * For many objects per frame: one camera set up per frame, then one call per
 * object from a fixed-point world pose. Per object: 32 x 32 -> 64-bit
 * multiplies, a 32-bit integer square root and five divisions on the
 * hardware divider (sh2z_divu_div64); no 64-bit division by software and no
 * 64-bit square root. */

/** Per-frame camera.
 *
 *  r        world-to-view rotation rows, Q1.14 (as sh2z_view_t).
 *  eye      camera position, Q16.16 world units.
 *  focal_x, focal_y, center_x, center_y, near_z  as sh2z_view_t (near_z in
 *           Q18.14 view units: one world unit is one view unit).
 *  scale_num  set by sh2z_rspr_camera_scale. */
typedef struct {
    int16_t r[3][3];
    int32_t eye[3];
    int32_t focal_x,focal_y;
    int16_t center_x,center_y;
    int32_t near_z;
    int64_t scale_num;
} sh2z_rspr_camera_t;

/** Set c->scale_num for objects whose model unit is unit_q14 view units
 *  (Q1.14, > 0; the model scale) drawn with set:
 *      scale_num = trunc(focal_y * unit_q14 * 65536 / pixels_per_unit)
 *  so that scale = trunc(scale_num / t.z) matches sh2z_rspr_place for a
 *  model unit of the same length. Once per frame and set (64-bit division).
 *  Returns false for null pointers, an invalid set or unit_q14 <= 0. */
bool sh2z_rspr_camera_scale(sh2z_rspr_camera_t *c,const sh2z_rspr_set_t *set,int32_t unit_q14);

/** Pick and place one object from its world pose. The set must already have
 *  been accepted by sh2z_rspr_valid; it is not re-checked per call.
 *
 *  pos      object origin, Q16.16 world units, within 2^31 of the eye.
 *  basis    model axes in world space, Q1.14, nine values row-major:
 *           basis[0..2] = model +z, basis[3..5] = model +y (up),
 *           basis[6..8] = model +x.
 *
 *  With d = pos - eye (int32, wrapping is the caller's error):
 *      t[i]  = floor(sum_k r[i][k] * d[k] / 2^16)                (Q18.14)
 *      view  = {-floor(B2.d / 2^16), -floor(B1.d / 2^16), -floor(B0.d / 2^16)}
 *              with Bk = basis[3k..3k+2]                   (camera in model)
 *      *pick = sh2z_rspr_select(set, view)
 *      X     = center_x * 2^16 + Q(focal_x * t.x, t.z)
 *      Y     = center_y * 2^16 - Q(focal_y * t.y, t.z)
 *      x, y  = floor(X / 2^16), floor(Y / 2^16); frac = X, Y mod 2^16
 *      scale = Q(scale_num, t.z)
 *      U     = {floor(r[0].B1 / 2^14), floor(r[1].B1 / 2^14)}
 *      up    = {Q(16384 * U.x, |U|), Q(-16384 * U.y, |U|)},  |U| = isqrt(U.x^2 + U.y^2)
 *  where Q(n, d) = sh2z_divu_div64(n, d) (truncating; saturating on
 *  overflow) and *depth = t.z. Returns false, leaving the outputs
 *  unchanged, for t.z <= near_z, |x| or |y| > 1023, scale outside 1..2^24,
 *  |U| == 0, a selection failure or null pointers. */
bool sh2z_rspr_pose(sh2z_rspr_pick_t *pick,sh2z_rspr_place_t *place,int32_t *depth,
                    const sh2z_rspr_camera_t *c,const sh2z_rspr_set_t *set,
                    const int32_t pos[3],const int16_t basis[9]);

/* ---- Contract tag rotation-sprites-3 ----
 *
 * Faster forms of the selection, the world pose and the emission above, each
 * with output identical to the rotation-sprites-2 function it names, for a
 * caller that already holds part of the work (a culling pass's camera-space
 * terms) or can prove a property of its set once (a convex yaw table). The
 * rotation-sprites-2 functions and their object code are unchanged. */

/** True when set's yaw bin centres, in index order, are the corners of a
 *  strictly convex polygon wound once around the origin, and set is not
 *  mirrored. Every turn and every step around the origin then has the same
 *  strict sign and the polar angle wraps once. For such a table the products
 *  yaw[k].sin*x + yaw[k].cos*z (exact integers) rise to one maximum (a corner,
 *  or one edge of two equal corners) and fall to one minimum around the
 *  polygon, so sh2z_rspr_yaw_climb finds sh2z_rspr_select's yaw bin. Check
 *  once per set (no division). False for a null or invalid-count set. */
bool sh2z_rspr_yaw_convex(const sh2z_rspr_set_t *set);

/** The yaw bin sh2z_rspr_select's rule gives for {x, z} (after its shift:
 *  |x|, |z| < 2^15; not mirrored), found by climbing from bin start (any
 *  value; out of range means 0) to a bin no neighbour exceeds, ties to the
 *  lower bin. *best receives that bin's product. Only for a set accepted by
 *  sh2z_rspr_yaw_convex; otherwise the result is unspecified. A caller that
 *  passes the object's previous bin usually evaluates three bins, not
 *  yaw_count. */
unsigned sh2z_rspr_yaw_climb(const sh2z_rspr_set_t *set,unsigned start,int32_t x,int32_t z,int32_t *best);

/** An object's camera-space terms for sh2z_rspr_pose_terms, as sh2z_rspr_pose
 *  computes them from pos (a culling pass usually has them already):
 *      d   = pos - eye (int32, wrapping)
 *      tz  = floor(r[2].d / 2^16)
 *      nx  = focal_x * floor(r[0].d / 2^16)
 *      ny  = focal_y * floor(r[1].d / 2^16) */
typedef struct {
    int32_t d[3];
    int32_t tz;
    int64_t nx,ny;
} sh2z_rspr_terms_t;

/** sh2z_rspr_pose from terms t (as sh2z_rspr_terms_t states them for the
 *  object's pos): the same outputs, *depth and return value as
 *  sh2z_rspr_pose(pick, place, depth, c, set, pos, basis). Each division is
 *  started on the hardware divider before independent work (the selection,
 *  the up products, the square root) and collected after it, so the same
 *  caller promise as sh2z_divu_start64 applies for the whole call.
 *  yaw_hint (nullable): when non-null, the yaw bin is found by
 *  sh2z_rspr_yaw_climb from *yaw_hint and the chosen bin is written back;
 *  pass it only for a set accepted by sh2z_rspr_yaw_convex. The set must
 *  already have been accepted by sh2z_rspr_valid. */
bool sh2z_rspr_pose_terms(sh2z_rspr_pick_t *pick,sh2z_rspr_place_t *place,int32_t *depth,
                          const sh2z_rspr_camera_t *c,const sh2z_rspr_set_t *set,
                          const sh2z_rspr_terms_t *t,const int16_t basis[9],uint8_t *yaw_hint);

/** sh2z_rspr_emit_valid for one cell, with the COLR and SRCA words given by
 *  the caller (a cell held in a slot pool, a CLUT kept elsewhere): the same
 *  sixteen words as sh2z_rspr_emit_valid(command, set, pick, place, vram,
 *  pmod) followed by command[3] = colr and command[4] = srca, where cell is
 *  the picked cell and mirror the pick's mirror flag. The set-level checks of
 *  sh2z_rspr_emit_valid (the cell index, vram) are the caller's. Each corner
 *  offset px*R - py*up is below 2^26 in magnitude (|px|, |py| <= 504,
 *  |up| <= 2^15), so its product with the scale is one 32 x 32 -> 64-bit
 *  multiply; the rounding is the same. Returns false, writing nothing, for a
 *  scale outside 1..2^24, mirror > 1, a corner outside -1024..1023 or null
 *  pointers. */
bool sh2z_rspr_emit_cell(uint16_t command[16],const sh2z_rspr_cell_t *cell,uint32_t mirror,
                         const sh2z_rspr_place_t *place,uint16_t pmod,uint16_t colr,uint16_t srca);

#ifdef __cplusplus
}
#endif
#endif
