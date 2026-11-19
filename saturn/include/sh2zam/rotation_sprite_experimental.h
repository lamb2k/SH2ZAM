/* Experimental rotation sprite sets for SH-2: draw a 3D object as a pre-rendered
 * view of itself. A set holds cells, each an image of the object seen from one
 * direction (a yaw x pitch grid around its own up axis). At run time the
 * caller picks the cell nearest the actual viewing direction, places it at the
 * object's projected origin and emits one VDP1 distorted sprite that scales
 * the cell with depth and rotates it with the object's on-screen roll.
 *
 * Contract tag: rotation-sprites-1. Every function below is a pure function of
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
 *  x, y     screen pixel of the object origin (VDP1 coordinates).
 *  scale    screen pixels per cell pixel, Q16.16, > 0.
 *  up       the object's up axis as a unit screen direction {x, y}, Q1.14,
 *           y pointing down the screen ({0, -16384} is upright).
 */
typedef struct {
    int16_t x,y;
    int32_t scale;
    int16_t up[2];
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
 *      x     = v->center_x + floor(trunc(focal_x * t.x / t.z) / 65536)
 *      y     = v->center_y - floor(trunc(focal_y * t.y / t.z) / 65536)
 *  as sh2z_view_batch projects the origin. With U = the model up column
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
 *      {x, y} + floor((scale * (px * R - py * up)) / 2^30 + 1/2)
 *  for the four cell corners A = (0, 0), B = (width, 0), C = (width, height)
 *  and D = (0, height) in column/row pixels. CTRL = 0x0002 (distorted
 *  sprite, next command follows), LINK = 0, GRDA = 0, COLR = (vram +
 *  clut_offset + clut*32) / 8, SRCA = (vram + pixel_offset) / 8 and SIZE =
 *  (width / 8) << 8 | height. Returns false, writing nothing, for a bad pick,
 *  a vram address that is misaligned or puts the cell outside VDP1 VRAM, a
 *  corner outside -1024..1023, or null pointers. */
bool sh2z_rspr_emit(uint16_t command[16],const sh2z_rspr_set_t *set,const sh2z_rspr_pick_t *pick,
                    const sh2z_rspr_place_t *place,uint32_t vram,uint16_t pmod);

#ifdef __cplusplus
}
#endif
#endif
