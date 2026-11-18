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

/* ---- Contract tag view-merge-2 ------------------------------------------- */

/** Run lengths sh2z_view_merge_tier accepts in run_mask: 1, 2, 4, 8, 16. */
#define SH2Z_VIEW_MERGE_TIER_RUNS 0x00010116u

/** Tier texture merge planner: the pixel-mode plan of sh2z_view_merge_runs
 *  for strips whose merged repetitions have equal texture heights, such as
 *  tier textures that hold 2 or 4 whole periods. It reads no rows, uses no
 *  multiplication and costs far less.
 *
 *  Boundaries j = 0 .. count-1 cut the strip into count-1 repetitions, and
 *  the reference draws one quad per repetition, as in sh2z_view_merge_runs.
 *  Only sx, sy and outcode of left and right are read. The repetitions of a
 *  run are taken to have equal texture height, so the reference draws the
 *  row at fraction k/r of a run of r repetitions from boundary i at
 *  P[i+k]. Permit a run, through run_mask and end_masks, only where that
 *  holds (whole periods of one texture, for example). The bound below is
 *  then against the true reference. If every permitted run has equal-height
 *  repetitions under some rows[], the plan equals sh2z_view_merge_runs with
 *  those rows, the same masks and SH2Z_VIEW_MERGE_PIXELS.
 *
 *  A run of r repetitions from boundary i passes when, on each rail P (left
 *  and right) and for each interior boundary k = 1 .. r-1,
 *      F = r * (P[i+k] - P[i]) - k * (P[i+r] - P[i])   (per axis, exact)
 *      max(|F.x|, |F.y|) * 256 <= tol_q8 * r
 *  F / r is how far, in pixels, the reference puts that boundary from where
 *  the merged quad puts its row. Between boundaries both rails' errors are
 *  linear in the row, and the quad interpolates between its rails, so every
 *  texel of a merged quad lies within tol_q8 / 256 pixels (Chebyshev) of
 *  its reference position. VIEW_BATCH.md has the proof.
 *
 *  Greedy plan, from boundary i = 0 until count-1: take the largest r of
 *  16, 8, 4, 2 with i + r <= count-1 and all of:
 *      bit r of run_mask is set;
 *      end_masks is NULL, or bit r of end_masks[i+r] is set;
 *      no boundary i .. i+r (ends included) has an outcode bit other than
 *          LEFT, RIGHT, TOP, BOTTOM on either rail;
 *      the run passes;
 *  or r = 1 if there is none (a single repetition is the reference). Then
 *  runs_out[n++] = r and i += r. Only bit r of end_masks[i+r] is read, for
 *  the r above, so end_masks words mean what they mean for
 *  sh2z_view_merge_runs.
 *
 *  Returns n >= 1, the number of runs written (they sum to count-1), or
 *  SH2Z_VIEW_MERGE_INVALID, writing nothing, if runs_out, left or right is
 *  null, count < 2 or count > SH2Z_VIEW_MERGE_MAX_COUNT,
 *  tol_q8 > SH2Z_VIEW_MERGE_MAX_TOL, run_mask has a bit other than bit 0
 *  and those of SH2Z_VIEW_MERGE_TIER_RUNS, or runs_out[0 .. count-2]
 *  overlaps left[], right[] or end_masks[] (count words each). Every value
 *  is exact in 32 bits: |F| < 2^22. No multiplication, division, floating
 *  point, global state or allocation; reentrant. */
int sh2z_view_merge_tier(uint8_t *runs_out,const sh2z_view_vertex_t *left,
    const sh2z_view_vertex_t *right,const uint32_t *end_masks,unsigned count,
    uint32_t run_mask,uint32_t tol_q8);

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

/* ---- Contract tag view-guard-src-1 --------------------------------------- */

#define SH2Z_VIEW_GUARD_SNAP (-3)

/** Guard-band clip in a source clipper's order (opt-in). A renderer that
 *  draws each clipped polygon as a fan with the whole texture mapped onto
 *  every fan piece shows the polygon's starting vertex, its order and every
 *  coordinate. sh2z_view_clip_guard_trunc makes the same cyclic polygon up
 *  to rounding, but often rotated. These functions reproduce a source
 *  clipper vertex for vertex.
 *
 *  Planes, their order and the inside test (d >= 0) are those of
 *  sh2z_view_clip_guard, written with a Q16 centre (Cx, Cy):
 *      left    d = focal_x*x - KL*z     KL = 65536*left - Cx
 *      right   d = KR*z - focal_x*x     KR = 65536*right - Cx
 *      top     d = KT*z - focal_y*y     KT = Cy - 65536*top
 *      bottom  d = focal_y*y - KB*z     KB = Cy - 65536*bottom
 *  These entry points use Cx = 65536*center_x and Cy = 65536*center_y.
 *  Three things differ from sh2z_view_clip_guard_trunc:
 *
 *  1. Order. Each pass walks the edges previous -> current, starting with
 *     (last, first), then (first, second), and so on. For each edge it emits
 *     the crossing if exactly one end is inside, then the current corner if
 *     it is inside. A pass with every corner inside changes nothing.
 *  2. Crossing, measured from the previous corner P towards the current
 *     corner Q, with dP and dQ their distances to the plane:
 *         t = floor(|dP| * 2^32 / (|dP| + |dQ|))        (exact; 2^32 if dQ = 0)
 *         a = P.a + sign(Q.a - P.a) * floor(|Q.a - P.a| * t / 2^32)
 *     for x, y, z and attr[0..attrs-1]. The crossing depends on the edge's
 *     direction: a shared edge walked the other way can differ by a unit.
 *  3. Snap. Then x, y and z (not the attributes) are moved inside every
 *     plane clipped so far, this pass's included, in this order:
 *         z = max(z, near_z);  once far is done:  z = min(z, far_z)
 *         left done:    x = max(x, ceil(KL*z / focal_x))
 *         right done:   x = min(x, floor(KR*z / focal_x))
 *         top done:     y = min(y, floor(KT*z / focal_y))
 *         bottom done:  y = max(y, ceil(KB*z / focal_y))
 *     computed exactly. The call fails with SH2Z_VIEW_GUARD_SNAP if one of
 *     those ceil or floor values for a plane done falls outside int32, used
 *     or not, or if the moved point is outside a plane done. That happens
 *     only when no integer x (or y) lies between left and right (top and
 *     bottom) at that depth.
 *
 *  Every output vertex then lies inside all six planes and is projected by
 *  the view-round-1 rule with the Q16 centre:
 *      sx = trunc((Cx + trunc(focal_x*x/z)) / 65536)
 *      sy = trunc((Cy - trunc(focal_y*y/z)) / 65536)
 *  which here is exactly sh2z_view_project_trunc. sx and sy fall inside the
 *  guard rectangle without a clamp. outcode is computed from sx and sy
 *  against v->clip (never NEAR or FAR); reserved bytes and attr[attrs..3]
 *  are zero.
 *
 *  max_out (3..16) is the most vertices any pass may produce. A pass that
 *  would produce more fails with SH2Z_VIEW_GUARD_OVERFLOW, so passing a
 *  source's own buffer size makes the call fail exactly where it fails. A
 *  pass that keeps every corner counts too: a polygon of more than max_out
 *  corners fails unless the near plane empties it or cuts it to max_out
 *  corners or fewer.
 *
 *  Returns 0 (every point outside some plane), 3..max_out vertices written,
 *  or, with nothing written: SH2Z_VIEW_GUARD_INVALID under the argument
 *  rules of sh2z_view_clip_guard or with max_out outside 3..16,
 *  SH2Z_VIEW_GUARD_OVERFLOW, or SH2Z_VIEW_GUARD_SNAP. Of these two, the
 *  first one the passes meet in the order above is returned. in is read
 *  before out is written, so they may overlap. The _divu form gives the same
 *  bytes and uses this CPU's hardware divider on SH-2 (see
 *  divu_experimental.h). */
int sh2z_view_clip_guard_src_trunc(sh2z_view_guard_vertex_t out[SH2Z_VIEW_GUARD_MAX_OUT],
    const sh2z_view_t *v,const int16_t guard[4],
    const sh2z_view_guard_vertex_t *in,unsigned count,unsigned attrs,unsigned max_out);
int sh2z_view_clip_guard_src_trunc_divu(sh2z_view_guard_vertex_t out[SH2Z_VIEW_GUARD_MAX_OUT],
    const sh2z_view_t *v,const int16_t guard[4],
    const sh2z_view_guard_vertex_t *in,unsigned count,unsigned attrs,unsigned max_out);

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

/** The view-guard-src-1 clip (sh2z_view_clip_guard_src_trunc) with a
 *  fractional centre: the same arguments, with a sh2z_view_center_t in place
 *  of the view. Its contract text already writes the planes, the snap
 *  bounds and the projection with a Q16 centre (Cx, Cy); here
 *      Cx = center_x16,  Cy = center_y16
 *  exactly, with nothing rounded. So the planes are those of
 *  sh2z_view_clip_guard_center above, the snap bounds are
 *  ceil(KL*z/focal_x), floor(KR*z/focal_x), floor(KT*z/focal_y) and
 *  ceil(KB*z/focal_y) with KL = 65536*left - center_x16,
 *  KR = 65536*right - center_x16, KT = center_y16 - 65536*top,
 *  KB = center_y16 - 65536*bottom, and every output vertex gets
 *      sx = trunc((center_x16 + trunc(focal_x*x/z)) / 65536)
 *      sy = trunc((center_y16 - trunc(focal_y*y/z)) / 65536)
 *  which lies inside the guard rectangle without a clamp (it is what
 *  sh2z_view_project_center writes). outcode is from sx, sy against
 *  view.clip. Order, crossings, max_out, the snap failure and every return
 *  value are as for sh2z_view_clip_guard_src_trunc. The call fails with
 *  SH2Z_VIEW_GUARD_INVALID on an invalid centre view (see above) or when
 *  |65536*edge - centre16| > 2^30 for an edge. With
 *  center_x16 = 65536 * center_x and center_y16 = 65536 * center_y each
 *  writes exactly what its _src_trunc form writes. The _divu form gives the
 *  same bytes and uses this CPU's hardware divider on SH-2. */
int sh2z_view_clip_guard_src_center(sh2z_view_guard_vertex_t out[SH2Z_VIEW_GUARD_MAX_OUT],
    const sh2z_view_center_t *v,const int16_t guard[4],
    const sh2z_view_guard_vertex_t *in,unsigned count,unsigned attrs,unsigned max_out);
int sh2z_view_clip_guard_src_center_divu(sh2z_view_guard_vertex_t out[SH2Z_VIEW_GUARD_MAX_OUT],
    const sh2z_view_center_t *v,const int16_t guard[4],
    const sh2z_view_guard_vertex_t *in,unsigned count,unsigned attrs,unsigned max_out);

/* ---- Contract tag pose-1 ------------------------------------------------- */

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
#endif
