/* Experimental integer shading kernels for SH-2 renderers: per-normal
 * directional light, Gouraud packing, conservative face culling and
 * threshold depth fog. Not selected by any default dispatch.
 * MIT; SH2ZAM Contributors.
 *
 * Contract tag: shade-1. Struct layouts and the numeric definitions below are
 * frozen at that tag. A faster path may replace an implementation only if it
 * produces identical bytes; new behaviour gets a new function.
 *
 * Every definition is written in exact integer arithmetic. C operators have
 * their C meaning: / and % truncate toward zero. A 5:5:5 colour packs channel
 * k (k = 0, 1, 2) at bits 5k .. 5k+4. No function uses floating point, the
 * hardware divider, global state or allocation; all are reentrant. */
#ifndef SH2ZAM_SHADE_EXPERIMENTAL_H
#define SH2ZAM_SHADE_EXPERIMENTAL_H

#include "sh2zam.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Contract tag shade-1 ------------------------------------------------- */

/* ---- Directional light per normal ---- */

/** A directional light and a reflection direction, rotated into model space.
 *  Built by sh2z_shade_light_prepare; callers may read but should not write
 *  it. Components are bounded by 49152 in magnitude. scale[k] is
 *  ceil(color[k] * 2^29 / 127), with which d * color[k] / (127 * 16384) is
 *  (d * scale[k]) >> 43 exactly for 0 <= d <= 127 * 16384. 44 bytes. */
typedef struct {
    int32_t light[3];
    int32_t reflection[3];
    uint32_t scale[3];
    uint8_t ambient[3];
    uint8_t color[3];
    uint8_t reserved[2];
} sh2z_shade_light_t;

/** Prepare a light for sh2z_shade_light_normals.
 *
 *  light and reflection are directions of any length (Q16.16, as given to
 *  sh2z_vec3_normalize). basis[k][j] is a model-to-world rotation in Q1.14
 *  (16384 is one), entries in [-16384, 16384]. For each direction D:
 *      u = sh2z_vec3_normalize(D)
 *      q[k] = u[k] / 4                                  (truncating)
 *      out[j] = (basis[0][j]*q[0] + basis[1][j]*q[1] + basis[2][j]*q[2]) / 16384
 *  (the transpose of basis applied to q; every product and sum is exact in
 *  32 bits). ambient and color are copied and scale is filled in.
 *
 *  Returns false, leaving *out unchanged, if a pointer is null, a basis entry
 *  is outside [-16384, 16384], or either normalised direction is zero. */
bool sh2z_shade_light_prepare(sh2z_shade_light_t *out,sh2z_vec3_t light,
    sh2z_vec3_t reflection,const int16_t basis[3][3],const uint8_t ambient[3],
    const uint8_t color[3]);

/** Light count int8 normals.
 *
 *  Normal i is three int8 values at (const char *)normals + i * normal_stride;
 *  its four output bytes are at out + i * out_stride. With n the normal and
 *  L the prepared light:
 *      d = clamp(n . L.light, 0, 127 * 16384)
 *      out[k] = min(255, L.ambient[k] + d * L.color[k] / (127 * 16384))   k < 3
 *      r = clamp((n . L.reflection) / 16384, -128, 127)
 *      out[3] = reflection_lut[r + 128]
 *  (the products d * color are below 2^29, so the division is exact unsigned
 *  32-bit arithmetic). L must come from sh2z_shade_light_prepare, or at least
 *  keep its bounds (components within +/-49152, scale matching color).
 *
 *  Returns false, writing nothing, if count > 0 with a null pointer,
 *  normal_stride < 3, out_stride < 4, or the output bytes overlap the normals,
 *  the light or the table. count == 0 succeeds. */
bool sh2z_shade_light_normals(uint8_t *out,size_t out_stride,const int8_t *normals,
    size_t normal_stride,size_t count,const sh2z_shade_light_t *light,
    const uint8_t reflection_lut[256]);

/* ---- Gouraud packing from a lit colour ---- */

/** One Gouraud evaluation: which base colour and which lit normal. */
typedef struct { uint16_t base,light; } sh2z_shade_pair_t;

/** For each pair i, with b = base + pairs[i].base * base_stride (three bytes),
 *  l = light + pairs[i].light * light_stride (four bytes, as written by
 *  sh2z_shade_light_normals) and the fog colour f:
 *      mul8(a, c)     = (x + (x >> 8)) >> 8   with x = a*c + 128
 *      blend(a, c, t) = (x + (x >> 8)) >> 8   with x = a*t + c*(255 - t) + 128
 *      lit[k] = blend(mul8(b[k], l[k]), f[k], 255 - l[3])
 *      v[k]   = 16 + (lit[k] >> 3) - (b[k] >> 4)
 *      out[i] = sum over k of clamp(v[k], 0, 31) << 5k            (bit 15 clear)
 *  *saturated (if not null) receives the number of channels clamped, summed
 *  over all pairs. Indices are not bounds-checked: the caller promises every
 *  referenced record lies in its array.
 *
 *  Returns false, writing nothing, if count > 0 with a null pointer other than
 *  saturated, base_stride < 3, light_stride < 4, or out[0 .. count-1]
 *  overlaps the pairs or the fog colour. count == 0 succeeds (and sets
 *  *saturated to 0). */
bool sh2z_shade_gouraud(uint16_t *out,uint32_t *saturated,const sh2z_shade_pair_t *pairs,
    size_t count,const uint8_t *base,size_t base_stride,const uint8_t *light,
    size_t light_stride,const uint8_t fog[3]);

/** The same lit colour as a solid 5:5:5 colour with bit 15 set:
 *      return 0x8000 | sum over k of (lit[k] >> 3) << 5k
 *  with lit[k] as in sh2z_shade_gouraud. Null pointers return 0. */
uint16_t sh2z_shade_solid(const uint8_t base[3],const uint8_t light[4],const uint8_t fog[3]);

/* ---- Conservative model-space face culling ---- */

/** A face plane in model units: normal n (int16, any scale), a point A on the
 *  face, and the plane constant c, normally n . A. 16 bytes. */
typedef struct {
    int16_t normal[3];
    int16_t first[3];
    int32_t plane;
} sh2z_shade_plane_t;

#define SH2Z_SHADE_NORMAL_MAX 16384
#define SH2Z_SHADE_FIRST_MAX  2048
#define SH2Z_SHADE_PLANE_MAX  (INT32_C(1) << 28)
#define SH2Z_SHADE_EYE_MAX    30000
#define SH2Z_SHADE_ERROR_MAX  1024u

/** Load-time check of plane records (plane i at (const char *)planes +
 *  i * stride) and, if order is not null, of order_count face ids. True when
 *  every |normal[k]| <= SH2Z_SHADE_NORMAL_MAX, |first[k]| <= SH2Z_SHADE_FIRST_MAX,
 *  |plane| <= SH2Z_SHADE_PLANE_MAX and every order[i] < count; the pointer is
 *  4-byte aligned and stride a multiple of 4, at least 16; count <= 65536.
 *  count == 0 with no order is valid. */
bool sh2z_shade_planes_valid(const void *planes,size_t stride,size_t count,
    const uint16_t *order,size_t order_count);

/** Keep the faces whose plane the eye might see.
 *
 *  For i = 0 .. count-1, face id = order ? order[i] : i, and its plane P at
 *  (const char *)planes + id * stride:
 *      dot    = n . eye - P.plane
 *      margin = 3 + sum over k of (|eye[k] - P.first[k]| + |n[k]| * eye_error)
 *      kept   if dot >= -margin
 *  Kept ids are written in order to kept[0 .. *kept_count - 1]. kept may be
 *  the same array as order (compaction in place); no other overlap is allowed
 *  (overlap with the planes is not detected: the caller must avoid it).
 *  With eye the integer eye position in model units, and every coordinate of
 *  the true eye within eye_error of it, a face is only dropped when the true
 *  eye is strictly behind its plane by more than the error of the integer
 *  normal (one unit per component) can explain.
 *
 *  Exact for planes that pass sh2z_shade_planes_valid (the 32-bit sums cannot
 *  wrap then); other planes give an unspecified but memory-safe keep set.
 *  Returns false, writing nothing and setting *kept_count (if not null) to 0,
 *  if kept_count is null, count > 0 with another null pointer (order may be
 *  null), count > 65536, stride or alignment break the rules above, some
 *  |eye[k]| > SH2Z_SHADE_EYE_MAX, eye_error > SH2Z_SHADE_ERROR_MAX, or kept
 *  partly overlaps order or eye. Ids in order are not checked here; validate
 *  them once with sh2z_shade_planes_valid. */
bool sh2z_shade_cull(uint16_t *kept,size_t *kept_count,const uint16_t *order,size_t count,
    const void *planes,size_t stride,const int32_t eye[3],uint32_t eye_error);

/* ---- Threshold depth fog ---- */

/** A non-decreasing 256-entry fade threshold table and its bucket index.
 *      fade(z) = max { i : depth[i] <= max(z, 0) }        (depth[0] == 0)
 *  bucket[b] = fade(b << shift) for b < 256 and bucket[256] = 255, where
 *  shift is the smallest value with depth[255] >> shift <= 255. */
typedef struct {
    uint32_t depth[256];
    uint8_t bucket[257];
    uint8_t shift;
    uint8_t reserved[2];
} sh2z_shade_fog_t;

/** Build the table from caller thresholds (depth may be out->depth itself,
 *  but may not otherwise overlap *out). Returns false, leaving *out
 *  unchanged, if a pointer is null, depth[0] != 0, depth decreases or depth
 *  partly overlaps *out. */
bool sh2z_shade_fog_from_depth(sh2z_shade_fog_t *out,const uint32_t depth[256]);

/** Build the thresholds of a fog that is linear in a projective depth ratio,
 *  then the index as sh2z_shade_fog_from_depth does.
 *
 *  With N = -pb * 2^30 (as uint64) and, for i = 1 .. 255,
 *      R = ceil((i - offset) * 2^30 / multiplier)            (exact, int64)
 *      depth[i] = 0                      if R <= -2^30
 *               = 4 * (N / (m + 1) + 1)  if -2^30 < R <= 2^30, m = pa - R >= 0
 *                                        and N / (m + 1) + 1 <= far_z / 4
 *               = far_z + 1              otherwise
 *  and depth[0] = 0. These are the depths at which
 *  clamp(floor(multiplier * ratio / 2^30) + offset, 0, 255), with
 *  ratio = pa - floor(N / (z / 4)), reaches i, for perspective-style
 *  parameters (SHADE.md gives the measured agreement); the table, not that
 *  formula, is the contract.
 *
 *  Returns false, leaving *out unchanged, if out is null, multiplier <= 0,
 *  pb > 0 or far_z <= 0. */
bool sh2z_shade_fog_build(sh2z_shade_fog_t *out,int32_t multiplier,int32_t offset,
    int32_t pa,int32_t pb,int32_t far_z);

/** fade[i * fade_stride] = fade(z_i) for count depths, z_i the int32 at
 *  (const char *)z + i * z_stride (for example the z of view vertices).
 *  Identical to a binary search of depth[] for every int32 z; the bucket
 *  index bounds the search to the thresholds inside one bucket.
 *
 *  Returns false, writing nothing, if count > 0 with a null pointer, z is not
 *  4-byte aligned, z_stride is not a multiple of 4 or is 0, fade_stride is 0,
 *  or a written byte overlaps a read z or the table. count == 0 succeeds. */
bool sh2z_shade_fog_lookup(uint8_t *fade,size_t fade_stride,const void *z,size_t z_stride,
    size_t count,const sh2z_shade_fog_t *fog);

/** Per-fade 5:5:5 offsets with bit 15 set:
 *      v[k] = 16 + (fade * slope[k] - bias[k]) / 255          (truncating)
 *      out  = 0x8000 | sum over k of clamp(v[k], 0, 31) << 5k
 *  for count fades (fade[i * fade_stride]) into out at out_stride bytes.
 *  |slope[k]| <= 127 and |bias[k]| <= 32767.
 *
 *  Returns false, writing nothing, if count > 0 with a null pointer, a
 *  parameter is out of range, out is not 2-byte aligned, out_stride is odd or
 *  0, or written bytes overlap the fades or parameters. */
bool sh2z_shade_fade_offset(uint16_t *out,size_t out_stride,const uint8_t *fade,
    size_t fade_stride,size_t count,const int16_t slope[3],const int16_t bias[3]);

/** Per-fade 5:5:5 mix of two colours with bit 15 set:
 *      v[k] = ((near[k] * (255 - fade) + far[k] * fade) * 31 + 32512) / 65025
 *      out  = 0x8000 | sum over k of v[k] << 5k               (v[k] <= 31)
 *  Same argument rules as sh2z_shade_fade_offset. */
bool sh2z_shade_fade_mix(uint16_t *out,size_t out_stride,const uint8_t *fade,
    size_t fade_stride,size_t count,const uint8_t near_rgb[3],const uint8_t far_rgb[3]);

/* ---- Contract tag shade-2 ------------------------------------------------- */

/* A fused fade lookup and 5:5:5 colour per vertex batch, with the per-frame
 * work moved into two prepared handles. Everything above is unchanged by this
 * tag. Scalar divides by constants for the same arithmetic are in
 * cdiv_experimental.h. */

/* ---- Prepared fade lookup ---- */

#define SH2Z_SHADE_FOGP_BUCKETS 4096

/** A threshold fog prepared for constant-time lookups (5,132 bytes). Built by
 *  sh2z_shade_fogp_prepare; callers may read but should not write it.
 *  fade(z) is shade-1's: max { i : depth[i] <= max(z, 0) }. shift is the
 *  smallest value >= 1 with min(depth[255], 2^31 - 1) >> shift below
 *  SH2Z_SHADE_FOGP_BUCKETS, scale = 2^(32 - shift), and
 *  bucket[b] = max { i : depth[i] <= b << shift } (as uint32) for
 *  b <= SH2Z_SHADE_FOGP_BUCKETS, so a depth in bucket b has its fade in
 *  bucket[b] .. bucket[b + 1]; equal bounds need no search. */
typedef struct {
    uint32_t scale;
    uint8_t shift;
    uint8_t reserved[3];
    uint32_t depth[256];
    uint8_t bucket[SH2Z_SHADE_FOGP_BUCKETS+4];
} sh2z_shade_fogp_t;

/** Prepare *out from a shade-1 table. Only fog->depth is read, so the table
 *  may come from sh2z_shade_fog_build, sh2z_shade_fog_from_depth or the
 *  caller. Once per fog change. Returns false, leaving *out unchanged, if a
 *  pointer is null, *out overlaps *fog, depth[0] != 0 or depth decreases. */
bool sh2z_shade_fogp_prepare(sh2z_shade_fogp_t *out,const sh2z_shade_fog_t *fog);

/* ---- Prepared colour ramp ---- */

#define SH2Z_SHADE_RAMP_OFFSET 1u
#define SH2Z_SHADE_RAMP_MIX    2u

/** The per-frame colour rule of sh2z_shade_fog_gouraud (532 bytes). Built by
 *  sh2z_shade_ramp_offset or sh2z_shade_ramp_mix; callers may read but should
 *  not write it.
 *
 *  Both forms choose a variant v from the mean fade of the batch:
 *      v = [sum >= threshold[0] * count] + [sum >= threshold[1] * count]
 *  with sum the sum of the batch's fades (floor(sum / count) >= t exactly when
 *  sum >= t * count), so a threshold of 256 never selects.
 *
 *  OFFSET form: a texture pre-blended toward a fog colour base at fade level
 *  level[v], corrected per vertex toward the live fog colour, relative to the
 *  texture's mean colour m (all three in the same units):
 *      slope[k] = live[k] - m[k],  bias[k] = level[v] * (base[k] - m[k])
 *      x        = fade * slope[k] - bias[k]
 *      c[k]     = clamp(16 + trunc(x / 255) + e, 0, 31)
 *                 with e = 1 if x is a negative multiple of 255, else e = 0.
 *  That is sh2z_shade_fade_offset's channel for this slope and bias, except
 *  that a negative multiple of 255 is one higher before the clamp: at most
 *  1 LSB per channel, and the only difference. The ramp stores 257 * live,
 *  257 * base, level and threshold.
 *
 *  MIX form: mix[f] is sh2z_shade_fade_mix's word for fade f (exact); mean is
 *  ignored and level is zero. */
typedef struct {
    uint8_t form;
    uint8_t level[3];
    uint16_t threshold[2];
    int16_t live[3],base[3];
    uint16_t mix[256];
} sh2z_shade_ramp_t;

/** OFFSET ramp. Every live[k] and base[k] <= 127 (with m[k] <= 127 this keeps
 *  shade-1's |slope| <= 127 and |bias| <= 32767). threshold may be null
 *  (variant always 0); otherwise threshold[0] <= threshold[1] <= 256.
 *  Returns false, leaving *out unchanged, if out, live, base or level is null
 *  or a value is out of range. */
bool sh2z_shade_ramp_offset(sh2z_shade_ramp_t *out,const uint8_t live[3],const uint8_t base[3],
    const uint8_t level[3],const uint16_t threshold[2]);

/** MIX ramp: mix[f] = sh2z_shade_fade_mix's word for (near_rgb, far_rgb) and
 *  fade f, for all 256 fades (once per colour change). threshold as above.
 *  Returns false, leaving *out unchanged, on a null pointer other than
 *  threshold or a threshold out of range. */
bool sh2z_shade_ramp_mix(sh2z_shade_ramp_t *out,const uint8_t near_rgb[3],const uint8_t far_rgb[3],
    const uint16_t threshold[2]);

/* ---- Fused fade and colour per batch ---- */

/** For count vertices with depth z_i, the int32 at (const char *)z + i * z_stride
 *  (for example &v[0].z of view vertices, stride sizeof(v[0])):
 *      fade_i = fade(z_i)                                (exact, as shade-1)
 *      v      = the ramp's variant for these fades
 *      out[i] = 0x8000 | c[0] | c[1] << 5 | c[2] << 10   (the ramp's form)
 *  with m = mean for the OFFSET form (mean[k] <= 127); the MIX form ignores
 *  mean, which may be null. Returns v (0, 1 or 2). count == 0 returns 0 and
 *  writes nothing.
 *
 *  Returns -1, writing nothing, if count > 0 and out, z, fog or ramp is null,
 *  ramp->form is not a form, the OFFSET form has a null mean or a
 *  mean[k] > 127, count > 65535, out is odd, z is not 4-byte aligned,
 *  or z_stride is not a multiple of 4 or exceeds 65532. Not checked, to keep
 *  the per-call cost small at about four vertices a call: out[0 .. count-1]
 *  must not overlap the depths, *fog, *ramp or mean, and fog and ramp must
 *  come from their prepare functions. */
int sh2z_shade_fog_gouraud(uint16_t *out,const void *z,size_t z_stride,size_t count,
    const sh2z_shade_fogp_t *fog,const sh2z_shade_ramp_t *ramp,const uint8_t mean[3]);

/** sh2z_shade_fog_gouraud with z_i the int32 at (const char *)refs[i] + z_offset:
 *  the corners of a piece given as pointers to their vertex records (for
 *  example an array of const sh2z_view_vertex_t * cast to const void *const *,
 *  z_offset offsetof(sh2z_view_vertex_t, z)), with no gather. Same results,
 *  returns and checks, with refs in place of z and z_offset in place of
 *  z_stride. Also not checked: every refs[i] is non-null and
 *  refs[i] + z_offset is 4-byte aligned. */
int sh2z_shade_fog_gouraud_refs(uint16_t *out,const void *const *refs,size_t z_offset,size_t count,
    const sh2z_shade_fogp_t *fog,const sh2z_shade_ramp_t *ramp,const uint8_t mean[3]);

#ifdef __cplusplus
}
#endif

#endif

/* ---- Contract tag vehicle-kernels-1 -------------------------------------- */
/* Appended section with its own guard: per-draw kernels for an indexed mesh
 * renderer (screen-space quad facing, lighting of referenced normals only,
 * and an approximate Gouraud packing with a proven error bound). */
#ifndef SH2ZAM_SHADE_VEHICLE_KERNELS_1
#define SH2ZAM_SHADE_VEHICLE_KERNELS_1

#include "view_experimental.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Screen-space facing of projected quads ---- */

/** Largest |sx| or |sy| for which a quad is tested (see below). */
#define SH2Z_SHADE_FACING_LIMIT 1023

/** Compact a list of quads to the ones that may face the viewer.
 *
 *  For i = 0 .. count-1, face id = faces[i] has corners
 *  c[j] = corners[id][j] (j = 0..3, vertex indices) and V[j] = vertices[c[j]].
 *  With x[j] = V[j].sx and y[j] = V[j].sy:
 *      all    = V[0].outcode & V[1].outcode & V[2].outcode & V[3].outcode
 *      simple = no V[j].outcode has SH2Z_OUT_NEAR or SH2Z_OUT_FAR, and every
 *               |x[j]|, |y[j]| <= SH2Z_SHADE_FACING_LIMIT
 *      area   = (x1-x0)(y2-y0) - (y1-y0)(x2-x0)
 *             + (x2-x0)(y3-y0) - (y2-y0)(x3-x0)
 *               (twice the signed area of triangles 0-1-2 plus 0-2-3)
 *      dropped when all != 0, or when simple and sign * area <= 0
 *  The other ids are written in order to kept[0 .. *kept_count - 1]. kept may
 *  be the same array as faces (compaction in place); no other overlap is
 *  allowed. A triangle stored as a quad with a repeated corner works as is.
 *  A face that is not simple is kept unless all != 0: its corners need a
 *  clip, which may not preserve the orientation.
 *
 *  sign is +1 or -1: the sign of area for faces that face the viewer
 *  (with y down, counter-clockwise front faces have negative area).
 *  view is the view the vertices were projected with: only its clip
 *  rectangle is read, to skip the coordinate limit test for quads whose
 *  outcodes are all zero, so each vertex's LEFT, RIGHT, TOP and BOTTOM bits
 *  must be those of its sx, sy against view->clip (as every sh2z_view
 *  projection writes them). Corner indices and face ids are not
 *  bounds-checked: the caller promises every referenced record exists.
 *
 *  Returns false, writing nothing and setting *kept_count (if not null) to
 *  0, if kept_count is null, count > 0 with another null pointer,
 *  sign is not +1 or -1, or kept partly overlaps faces. count == 0
 *  succeeds. */
bool sh2z_shade_screen_facing(uint16_t *kept,size_t *kept_count,const uint16_t *faces,
    size_t count,const uint8_t (*corners)[4],const sh2z_view_vertex_t *vertices,
    const sh2z_view_t *view,int32_t sign);

/* ---- Lighting of referenced normals only ---- */

/** For k = 0 .. index_count-1, with i = index[k] (i < count, not checked):
 *  the four bytes at out + i * out_stride are set exactly as
 *  sh2z_shade_light_normals(out, out_stride, normals, normal_stride, count,
 *  light, reflection_lut) sets them; other output bytes are not written.
 *  An index may repeat. Argument rules are those of sh2z_shade_light_normals
 *  for count normals, and the output bytes may also not overlap the index
 *  list. index_count == 0 succeeds. */
bool sh2z_shade_light_normals_indexed(uint8_t *out,size_t out_stride,const int8_t *normals,
    size_t normal_stride,size_t count,const uint8_t *index,size_t index_count,
    const sh2z_shade_light_t *light,const uint8_t reflection_lut[256]);

/* ---- Approximate Gouraud packing ---- */

/** sh2z_shade_gouraud with plain >> 8 in place of the (x + (x >> 8)) >> 8
 *  rounding. With b, l, f and t = 255 - l[3] as there, for each pair:
 *      m[k]   = (b[k]*l[k] + 128) >> 8
 *      lit[k] = (m[k]*t + f[k]*(255 - t) + 128) >> 8
 *      v[k]   = 16 + (lit[k] >> 3) - (b[k] >> 4)
 *      out[i] = sum over k of min(v[k], 31) << 5k               (bit 15 clear)
 *  (v[k] >= 1 always). *saturated (if not null) receives the number of
 *  channels with v[k] > 31. Same arguments and failures as
 *  sh2z_shade_gouraud.
 *
 *  Error bound: every 5-bit channel equals the sh2z_shade_gouraud channel
 *  or is one less. (x >> 8 is the exact rounding or one less, so m[k] is;
 *  then lit[k] is the exact value or up to two less, and lit >> 3 moves by
 *  at most one; the clamp keeps that.) VEHICLE_KERNELS.md gives the
 *  measured match rate. */
bool sh2z_shade_gouraud_approx(uint16_t *out,uint32_t *saturated,const sh2z_shade_pair_t *pairs,
    size_t count,const uint8_t *base,size_t base_stride,const uint8_t *light,
    size_t light_stride,const uint8_t fog[3]);

#ifdef __cplusplus
}
#endif

#endif /* SH2ZAM_SHADE_VEHICLE_KERNELS_1 */

/* ---- Per-contract headers -------------------------------------------------
 * A new shade contract goes in its own header, sh2zam/contracts/<tag>.h
 * (the tag with '_' for '-', e.g. contracts/shade_example_1.h), with its own
 * include guard and extern "C" block, starting with
 * #include "../shade_experimental.h". List it below: one #include per line,
 * sorted by file name. The sections above stay as they are, and every
 * existing include keeps working. See saturn/CONTRIBUTING-KERNELS.md. */
