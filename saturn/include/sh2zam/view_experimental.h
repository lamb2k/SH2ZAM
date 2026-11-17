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

/** Identical arguments, failures and output bytes to sh2z_view_batch, but on
 *  SH-2 it uses this CPU's hardware divider (see divu_experimental.h for the
 *  caller promise about interrupt handlers). Every division sent to the
 *  divider is proven in software to fit, so no hardware overflow or
 *  divide-by-zero result is ever used. Portable builds compute in C. */
bool sh2z_view_batch_divu(sh2z_view_vertex_t *out,const sh2z_view_t *v,
    const void *positions,size_t in_stride,size_t count);

/* ---- Contract tag view-edges-1 ------------------------------------------- */

/** Project and classify view-space points: for each in[k], only x, y, z are
 *  read, and out[k] is exactly what sh2z_view_batch writes after its
 *  transform for that view position. in == out is allowed (projection in
 *  place); otherwise the arrays must not overlap. Returns false, writing
 *  nothing, if v is invalid, count > 0 with a null pointer, or the arrays
 *  partly overlap. count == 0 succeeds. The _divu form uses this CPU's
 *  hardware divider on SH-2 (see divu_experimental.h). */
bool sh2z_view_project(sh2z_view_vertex_t *out,const sh2z_view_t *v,
    const sh2z_view_vertex_t *in,size_t count);
bool sh2z_view_project_divu(sh2z_view_vertex_t *out,const sh2z_view_t *v,
    const sh2z_view_vertex_t *in,size_t count);

/** Points along the view-space segment from -> to, projected.
 *
 *  Only the x, y, z of from and to are read. For each axis, with
 *  delta = to - from (exact, 64-bit) and 16 extra fraction bits:
 *      F = trunc(delta * first_num * 65536 / den)
 *      S = trunc(delta * step_num  * 65536 / den)
 *      P(k) = from + trunc((F + k * S) / 65536),   k = 0 .. count-1
 *  Each P(k) is then projected and classified exactly as sh2z_view_batch does
 *  after its transform. Six divisions per call; none per point beyond the
 *  projection. |P(k) - exact| < 1 + (k + 1) / 65536 units per axis, where
 *  exact = from + delta * (first_num + k * step_num) / den.
 *
 *  Requires den > 0 and first_num + (count - 1) * step_num <= den, so every
 *  point lies between from and to. Points are a function of the direction:
 *  stepping a shared edge from its other end can differ by one unit, so step
 *  shared edges in one canonical direction.
 *
 *  from and to are read before anything is written; they may point into out.
 *  Returns false, writing nothing, if v is invalid, count > 0 with a null
 *  pointer, den == 0 or the bound above fails. count == 0 succeeds. */
bool sh2z_view_edge_steps(sh2z_view_vertex_t *out,const sh2z_view_t *v,
    const sh2z_view_vertex_t *from,const sh2z_view_vertex_t *to,
    uint16_t first_num,uint16_t step_num,uint16_t den,size_t count);

/** Clip a quad against the near plane only; side clipping is left to VDP1.
 *
 *  in[0..3] are the quad's vertices in order, as written by sh2z_view_batch
 *  with the same view. A vertex is inside when z >= near_z. Walking the edges
 *  in[i] -> in[(i+1) % 4] from i = 0, each inside vertex is copied unchanged
 *  and each edge with exactly one inside end adds its crossing point:
 *      with i the inside end and o the outside end,
 *      z = near_z,
 *      a = i.a + trunc((o.a - i.a) * (i.z - near_z) / (i.z - o.z))  for x, y,
 *  projected as sh2z_view_batch would (so never NEAR). The crossing depends
 *  only on the unordered pair of ends, so two quads sharing an edge produce
 *  the same point.
 *
 *  Returns the number of vertices written:
 *      0  all behind, or v invalid / a pointer null (nothing written)
 *      3  one in front          4  two adjacent in front, or all four
 *      5  three in front        6  two opposite in front (only possible for a
 *                                  non-planar or non-convex quad)
 *  in is read before anything is written; it may alias out. */
unsigned sh2z_view_clip_near(sh2z_view_vertex_t out[6],const sh2z_view_t *v,
    const sh2z_view_vertex_t in[4]);

/** The same results as sh2z_view_edge_steps and sh2z_view_clip_near, but on
 *  SH-2 each projection uses this CPU's hardware divider (see
 *  divu_experimental.h for the caller promise). Portable builds compute in C. */
bool sh2z_view_edge_steps_divu(sh2z_view_vertex_t *out,const sh2z_view_t *v,
    const sh2z_view_vertex_t *from,const sh2z_view_vertex_t *to,
    uint16_t first_num,uint16_t step_num,uint16_t den,size_t count);
unsigned sh2z_view_clip_near_divu(sh2z_view_vertex_t out[6],const sh2z_view_t *v,
    const sh2z_view_vertex_t in[4]);

/* ---- Contract tag view-guard-1 ------------------------------------------- */

#define SH2Z_VIEW_GUARD_ATTRS    4
#define SH2Z_VIEW_GUARD_MAX_IN   8
#define SH2Z_VIEW_GUARD_MAX_OUT  16
#define SH2Z_VIEW_GUARD_INVALID  (-1)
#define SH2Z_VIEW_GUARD_OVERFLOW (-2)

/** A polygon corner for the guard-band clip (36 bytes): v.x, v.y, v.z are
 *  view space; attr holds caller-owned values (texture coordinates, shade,
 *  colour) that are interpolated with the coordinates. On input only v.x,
 *  v.y, v.z and attr[0..attrs-1] are read. */
typedef struct {
    sh2z_view_vertex_t v;
    int32_t attr[SH2Z_VIEW_GUARD_ATTRS];
} sh2z_view_guard_vertex_t;

/** Clip a view-space polygon in[0..count-1] (3 <= count <= 8) against six
 *  planes before projection, so that every output vertex projects inside the
 *  caller's guard rectangle guard = {left, top, right, bottom} (inclusive
 *  pixels). Choose the rectangle inside the target's representable range;
 *  VDP1 command coordinates must lie in -1024..1023.
 *
 *  Planes, in this order; P is inside a plane when d(P) >= 0, all exact in
 *  64 bits:
 *      1 near    d = z - near_z
 *      2 far     d = far_z - z
 *      3 left    d = focal_x*x - 65536*(left - center_x)*z
 *      4 right   d = 65536*(right - center_x)*z - focal_x*x
 *      5 top     d = 65536*(center_y - top)*z - focal_y*y
 *      6 bottom  d = focal_y*y - 65536*(center_y - bottom)*z
 *  A point inside all six projects (by the sh2z_view_batch definition) to
 *  left <= sx <= right and top <= sy <= bottom, with near_z <= z <= far_z.
 *
 *  Each plane is one Sutherland-Hodgman pass over the current polygon,
 *  starting at its vertex 0: for each edge cur -> next, emit cur if inside,
 *  then emit the crossing if exactly one end is inside. The crossing is
 *  computed from the inside end I towards the outside end O, for x, y, z and
 *  attr[0..attrs-1]:
 *      a = I.a + sign(O.a - I.a) * offset(|O.a - I.a|)
 *  with num = d(I) >= 0 and den = d(I) - d(O) > 0:
 *      near, far:  offset(m) = trunc(m * num / den)       (exact)
 *      sides:      offset(m) = floor(m * t / 2^30), where
 *                  s = the least shift with (den >> s) < 2^31,
 *                  t = floor((num >> s) * 2^30 / (den >> s))   (t <= 2^30)
 *  Near crossings are therefore the same points sh2z_view_clip_near makes,
 *  and every crossing depends only on the unordered pair of ends, so
 *  polygons sharing an edge get identical points on it in either winding.
 *  Each a lies between I.a and O.a; a side crossing is within
 *  1 + 3*|O.a - I.a|/2^30 of the exact one.
 *
 *  Each output vertex is then projected as sh2z_view_batch would, and sx, sy
 *  are clamped to the guard rectangle. Vertices inside all six planes already
 *  project inside it; only crossings whose rounded position projects just
 *  outside are moved. outcode is computed from the clamped sx, sy against
 *  v->clip (never NEAR or FAR); reserved bytes and attr[attrs..3] are zero.
 *
 *  Returns:
 *      0                         every point outside some plane
 *      3 .. 16                   vertices written to out; corners may repeat
 *                                when the polygon touches a plane
 *      SH2Z_VIEW_GUARD_OVERFLOW  some pass would exceed 16 vertices
 *      SH2Z_VIEW_GUARD_INVALID   v invalid, a null pointer, count outside
 *                                3..8, attrs > 4, left > right, top > bottom,
 *                                or a guard edge more than 16384 pixels from
 *                                the centre
 *  Nothing is written on a negative return. A planar convex polygon needs at
 *  most count + 6 vertices, so it overflows only if rounding leaves corners
 *  within a unit of a plane in a near-tangent arrangement; twisted or
 *  non-convex polygons can need more. in is read before out is written, so
 *  they may overlap. Output order follows the input winding. */
int sh2z_view_clip_guard(sh2z_view_guard_vertex_t out[SH2Z_VIEW_GUARD_MAX_OUT],
    const sh2z_view_t *v,const int16_t guard[4],
    const sh2z_view_guard_vertex_t *in,unsigned count,unsigned attrs);

/** The same result as sh2z_view_clip_guard; on SH-2 the side-plane fractions,
 *  near and far crossings and projections use this CPU's hardware divider
 *  (see divu_experimental.h for the caller promise). */
int sh2z_view_clip_guard_divu(sh2z_view_guard_vertex_t out[SH2Z_VIEW_GUARD_MAX_OUT],
    const sh2z_view_t *v,const int16_t guard[4],
    const sh2z_view_guard_vertex_t *in,unsigned count,unsigned attrs);

/* ---- Contract tag view-merge-1 ------------------------------------------- */

#define SH2Z_VIEW_MERGE_PIXELS    0u
#define SH2Z_VIEW_MERGE_ROWS      1u
#define SH2Z_VIEW_MERGE_MAX_COUNT 65536u
#define SH2Z_VIEW_MERGE_MAX_TOL   0x01000000u
#define SH2Z_VIEW_MERGE_INVALID   (-1)

/** Plan which consecutive texture repetitions of a strip to draw as one
 *  distorted sprite, with every merged quad provably within a tolerance of
 *  the one-quad-per-repetition reference.
 *
 *  Boundaries j = 0 .. count-1 cut the strip into count-1 repetitions; the
 *  reference draws repetition j as the quad left[j], right[j], right[j+1],
 *  left[j+1] with texture rows rows[j] .. rows[j+1]. Only sx, sy and
 *  outcode of left and right are read. rows[] holds cumulative source
 *  texture rows and must be strictly increasing (uint32 compare).
 *
 *  A merged run of r repetitions from boundary i is the single quad
 *  left[i], right[i], right[i+r], left[i+r] with rows rows[i] .. rows[i+r].
 *  On each rail P (left and right), with S = rows[i+r] - rows[i], and for
 *  each interior boundary k = i+1 .. i+r-1 with t = rows[k] - rows[i]:
 *      D = P[i+r] - P[i],  E = S * (P[k] - P[i]) - t * D   (per axis, exact)
 *  E / S is how far, in pixels, the reference puts row t from where the
 *  merged quad puts it. The run passes when every interior boundary passes
 *  on both rails:
 *      SH2Z_VIEW_MERGE_PIXELS: max(|E.x|, |E.y|) * 256 <= tol_q8 * S
 *          (each boundary row within tol_q8 / 256 pixels, Chebyshev)
 *      SH2Z_VIEW_MERGE_ROWS:   max(|E.x|, |E.y|) <= S   (within 1 pixel) and
 *          |E.x * D.x + E.y * D.y| * 256 <= tol_q8 * (D.x^2 + D.y^2)
 *          (the boundary point, projected onto the merged rail, lies within
 *          tol_q8 / 256 merged texture rows of row t; a rail whose two ends
 *          coincide on screen passes this half)
 *  Between boundaries both rails' errors are linear in the row, and the quad
 *  interpolates between its rails, so in pixel mode every texel of a merged
 *  quad lies within the tolerance of its reference position. VIEW_BATCH.md
 *  has the proof and what rows mode bounds.
 *
 *  Greedy plan, from boundary i = 0 until count-1: take the largest r with
 *  i + r <= count-1 and r <= 31 such that r == 1 (a single repetition is the
 *  reference, always allowed), or all of:
 *      bit r of run_mask is set;
 *      end_masks is NULL, or bit r of end_masks[i+r] is set;
 *      no boundary i .. i+r (ends included) has an outcode bit other than
 *          LEFT, RIGHT, TOP, BOTTOM on either rail (NEAR, FAR and undefined
 *          bits stop merging: such corners have no usable screen position);
 *      the run passes;
 *  then runs_out[n++] = r and i += r. Bits 0 and 1 of run_mask and of
 *  end_masks[] are ignored. Bit r of end_masks[j] permits the run j-r .. j,
 *  so one word per boundary expresses any start and end alignment per run
 *  length (for tier textures that line up only at some boundaries).
 *
 *  Returns n >= 1, the number of runs written (they sum to count-1), or
 *  SH2Z_VIEW_MERGE_INVALID, writing nothing, if a pointer other than
 *  end_masks is null, count < 2 or count > SH2Z_VIEW_MERGE_MAX_COUNT, mode
 *  is neither of the two above, tol_q8 > SH2Z_VIEW_MERGE_MAX_TOL, rows is
 *  not strictly increasing, or runs_out[0 .. count-2] overlaps an input
 *  array. Every product is exact in 64 bits; no division, floating point,
 *  global state or allocation. At most 30 candidate runs per output run,
 *  typically one to three. */
int sh2z_view_merge_runs(uint8_t *runs_out,const sh2z_view_vertex_t *left,
    const sh2z_view_vertex_t *right,const uint32_t *rows,const uint32_t *end_masks,
    unsigned count,uint32_t run_mask,uint32_t tol_q8,unsigned mode);

/* ---- Contract tag view-round-1 ------------------------------------------- */

/** Final-coordinate truncation (opt-in). For a renderer that matches a source
 *  which computes a Q16.16 screen position and then truncates it toward zero
 *  (C division by 65536), instead of rounding the quotient first.
 *
 *  Each _trunc function takes the same arguments, has the same failures and
 *  writes the same bytes as the function of the same name without _trunc,
 *  except that every projection uses these sx, sy (and the outcode bits that
 *  follow from them):
 *      qx = clamp(trunc(focal_x * x / z), -(2^31 - 1), 2^31 - 1)
 *      qy = clamp(trunc(focal_y * y / z), -(2^31 - 1), 2^31 - 1)
 *      sx = sat16(trunc((65536 * center_x + qx) / 65536))
 *      sy = sat16(trunc((65536 * center_y - qy) / 65536))
 *  computed exactly; trunc rounds toward zero. The quotient clamp is
 *  symmetric, unlike view-contract-1's sat32, so the projection is odd:
 *  negating x and center_x negates sx before saturation (same for y).
 *
 *  Compared with view-contract-1, sx is one larger when the Q16 position
 *  65536 * center_x + qx is negative and not a whole pixel, and sy is one
 *  smaller when 65536 * center_y - qy is positive and not a whole pixel;
 *  otherwise both are equal.
 *
 *  Paths: the edge stepper and both clips compute the same view-space points
 *  as before. sh2z_view_clip_near_trunc copies inside corners unchanged, so
 *  give it corners from a _trunc batch or projection. sh2z_view_clip_guard_trunc
 *  keeps the six planes of sh2z_view_clip_guard: truncation, like the old
 *  rounding, never moves a value out of an interval with whole-pixel ends, so a
 *  point inside all six still lands inside the guard rectangle; sx and sy are
 *  then clamped to the rectangle as before. The _divu forms give the same
 *  bytes and use this CPU's hardware divider on SH-2 (see divu_experimental.h). */
bool sh2z_view_batch_trunc(sh2z_view_vertex_t *out,const sh2z_view_t *v,
    const void *positions,size_t in_stride,size_t count);
bool sh2z_view_batch_trunc_divu(sh2z_view_vertex_t *out,const sh2z_view_t *v,
    const void *positions,size_t in_stride,size_t count);
bool sh2z_view_project_trunc(sh2z_view_vertex_t *out,const sh2z_view_t *v,
    const sh2z_view_vertex_t *in,size_t count);
bool sh2z_view_project_trunc_divu(sh2z_view_vertex_t *out,const sh2z_view_t *v,
    const sh2z_view_vertex_t *in,size_t count);
bool sh2z_view_edge_steps_trunc(sh2z_view_vertex_t *out,const sh2z_view_t *v,
    const sh2z_view_vertex_t *from,const sh2z_view_vertex_t *to,
    uint16_t first_num,uint16_t step_num,uint16_t den,size_t count);
bool sh2z_view_edge_steps_trunc_divu(sh2z_view_vertex_t *out,const sh2z_view_t *v,
    const sh2z_view_vertex_t *from,const sh2z_view_vertex_t *to,
    uint16_t first_num,uint16_t step_num,uint16_t den,size_t count);
unsigned sh2z_view_clip_near_trunc(sh2z_view_vertex_t out[6],const sh2z_view_t *v,
    const sh2z_view_vertex_t in[4]);
unsigned sh2z_view_clip_near_trunc_divu(sh2z_view_vertex_t out[6],const sh2z_view_t *v,
    const sh2z_view_vertex_t in[4]);
int sh2z_view_clip_guard_trunc(sh2z_view_guard_vertex_t out[SH2Z_VIEW_GUARD_MAX_OUT],
    const sh2z_view_t *v,const int16_t guard[4],
    const sh2z_view_guard_vertex_t *in,unsigned count,unsigned attrs);
int sh2z_view_clip_guard_trunc_divu(sh2z_view_guard_vertex_t out[SH2Z_VIEW_GUARD_MAX_OUT],
    const sh2z_view_t *v,const int16_t guard[4],
    const sh2z_view_guard_vertex_t *in,unsigned count,unsigned attrs);

/* ---- Contract tag view-center-1 ------------------------------------------ */

/** A view whose centre has 16 fraction bits (68 bytes), for a source renderer
 *  whose centre is fractional. center_x16 and center_y16 are the centre in
 *  Q16.16 pixels; any int32 values are allowed. view holds every other field.
 *
 *  Valid when view is valid (see sh2z_view_t) and
 *      view.center_x == floor(center_x16 / 65536)
 *      view.center_y == floor(center_y16 / 65536)
 *  so a stale or forgotten centre is rejected instead of used. The _center
 *  functions read the centre only from center_x16 and center_y16.
 *  sh2z_view_center_init sets all four fields consistently. */
typedef struct {
    sh2z_view_t view;
    int32_t center_x16,center_y16;
} sh2z_view_center_t;

/** Copy *view to out->view, set out->view.center_x and center_y to the floor
 *  of the Q16 centre and store center_x16 and center_y16. Fails, leaving out
 *  unchanged, if out is null or *view is invalid. view may be &out->view.
 *  Call sh2z_view_place on the plain view first (it ignores the centre). */
bool sh2z_view_center_init(sh2z_view_center_t *out,const sh2z_view_t *view,
    int32_t center_x16,int32_t center_y16);

/** Final-coordinate truncation with a fractional centre (opt-in). Each
 *  _center function takes a sh2z_view_center_t where the _trunc function of
 *  the same name takes a view. It has the same other arguments and failures,
 *  plus failure on an invalid centre (see above). It writes the same bytes,
 *  except that every projection uses these sx, sy and the outcode bits that
 *  follow from them:
 *      qx = clamp(trunc(focal_x * x / z), -(2^31 - 1), 2^31 - 1)
 *      qy = clamp(trunc(focal_y * y / z), -(2^31 - 1), 2^31 - 1)
 *      sx = sat16(trunc((center_x16 + qx) / 65536))
 *      sy = sat16(trunc((center_y16 - qy) / 65536))
 *  The sums are computed exactly (33 bits; they never wrap), so every int32
 *  centre is valid. With center_x16 = 65536 * center_x and
 *  center_y16 = 65536 * center_y, each function writes exactly what its
 *  _trunc form writes.
 *
 *  Edge steps compute the same points as sh2z_view_edge_steps. The near clip
 *  computes the same crossings as sh2z_view_clip_near and copies inside
 *  corners unchanged, so give it corners from a _center batch or projection
 *  with the same view.
 *
 *  The guard clip uses sh2z_view_clip_guard's six planes with the centre
 *  taken in Q16 (65536 * center_x becomes center_x16):
 *      3 left    d = focal_x*x - (65536*left - center_x16)*z
 *      4 right   d = (65536*right - center_x16)*z - focal_x*x
 *      5 top     d = (center_y16 - 65536*top)*z - focal_y*y
 *      6 bottom  d = focal_y*y - (center_y16 - 65536*bottom)*z
 *  and its rule "a guard edge more than 16384 pixels from the centre" becomes
 *  |65536*edge - centre16| > 2^30 (then SH2Z_VIEW_GUARD_INVALID). Passes,
 *  crossings and the order of the output are as before. A point inside all
 *  six planes lands, by the projection above, inside the guard rectangle;
 *  sx and sy are then clamped to the rectangle as before, and outcode is
 *  computed from the clamped sx, sy against view.clip (never NEAR or FAR).
 *  The _divu forms give the same bytes and use this CPU's hardware divider
 *  on SH-2 (see divu_experimental.h). */
bool sh2z_view_batch_center(sh2z_view_vertex_t *out,const sh2z_view_center_t *v,
    const void *positions,size_t in_stride,size_t count);
bool sh2z_view_batch_center_divu(sh2z_view_vertex_t *out,const sh2z_view_center_t *v,
    const void *positions,size_t in_stride,size_t count);
bool sh2z_view_project_center(sh2z_view_vertex_t *out,const sh2z_view_center_t *v,
    const sh2z_view_vertex_t *in,size_t count);
bool sh2z_view_project_center_divu(sh2z_view_vertex_t *out,const sh2z_view_center_t *v,
    const sh2z_view_vertex_t *in,size_t count);
bool sh2z_view_edge_steps_center(sh2z_view_vertex_t *out,const sh2z_view_center_t *v,
    const sh2z_view_vertex_t *from,const sh2z_view_vertex_t *to,
    uint16_t first_num,uint16_t step_num,uint16_t den,size_t count);
bool sh2z_view_edge_steps_center_divu(sh2z_view_vertex_t *out,const sh2z_view_center_t *v,
    const sh2z_view_vertex_t *from,const sh2z_view_vertex_t *to,
    uint16_t first_num,uint16_t step_num,uint16_t den,size_t count);
unsigned sh2z_view_clip_near_center(sh2z_view_vertex_t out[6],const sh2z_view_center_t *v,
    const sh2z_view_vertex_t in[4]);
unsigned sh2z_view_clip_near_center_divu(sh2z_view_vertex_t out[6],const sh2z_view_center_t *v,
    const sh2z_view_vertex_t in[4]);
int sh2z_view_clip_guard_center(sh2z_view_guard_vertex_t out[SH2Z_VIEW_GUARD_MAX_OUT],
    const sh2z_view_center_t *v,const int16_t guard[4],
    const sh2z_view_guard_vertex_t *in,unsigned count,unsigned attrs);
int sh2z_view_clip_guard_center_divu(sh2z_view_guard_vertex_t out[SH2Z_VIEW_GUARD_MAX_OUT],
    const sh2z_view_center_t *v,const int16_t guard[4],
    const sh2z_view_guard_vertex_t *in,unsigned count,unsigned attrs);

#ifdef __cplusplus
}
#endif
#endif
