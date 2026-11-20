/* Experimental segmented strip mesh: integer visibility pick and far-to-near run
 * ordering for a closed (circular) chain of mesh segments. Not selected by any
 * default dispatch.
 *
 * Contract tags: segmesh-1 (pick, below) and segmesh-2 (cluster bounds culling,
 * at the end). segmesh-1: A mesh is a ring of N segments, each represented for
 * visibility by one anchor point. Per frame the caller supplies four fixed-point
 * clip rows. The pick classifies every anchor, marks each segment's traversal
 * direction from the depth slope to its successor, splits the ring into runs of
 * similar depth and returns the runs furthest first, ready for painter-order
 * drawing. All arithmetic is 32-bit integer (one 64-bit sum per run average);
 * no float, no libc, no hardware divider state, no allocation.
 *
 * Copyright (c) 2026 SH2ZAM Contributors. MIT License.
 * Inspired by SH4ZAM by Falco Girgis and SH4ZAM Contributors.
 */
#ifndef SH2ZAM_SEGMESH_EXPERIMENTAL_H
#define SH2ZAM_SEGMESH_EXPERIMENTAL_H

#include "sh2zam.h"
#include "view_experimental.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Contract tag segmesh-1 ---------------------------------------------- */

/** \brief Largest coefficient magnitude for columns 0..2 of a clip row. */
#define SH2Z_SEGMESH_COEF_MAX  INT32_C(16383)
/** \brief Largest translation (column 3) magnitude of a clip row. */
#define SH2Z_SEGMESH_TRANS_MAX INT32_C(0x1FFFFFFF)
/** \brief Largest ring length. */
#define SH2Z_SEGMESH_MAX_SEGMENTS 4096u

/** \brief One segment anchor in mesh units. Components are full int16. */
typedef struct {
    int16_t x, y, z;
} sh2z_segmesh_anchor_t;

/** \brief Per-frame clip rows and thresholds.
 *
 * For an anchor p, each row r evaluates r[0]*p.x + r[1]*p.y + r[2]*p.z + r[3].
 * Rows x, y and w share one scale; row d gives depth in caller units. With
 * |r[0..2]| <= SH2Z_SEGMESH_COEF_MAX and |r[3]| <= SH2Z_SEGMESH_TRANS_MAX every
 * row value fits int32 (3 * 16383 * 32768 + 2^29 - 1 < 2^31).
 *
 * A segment is visible when 0 <= d <= far, |x| <= |w| and |y| <= |w| (the
 * divide-free form of x/w and y/w in [-1, 1]; w == 0 admits only x == y == 0).
 * slope, split and floor are depth thresholds in the same units as d. */
typedef struct {
    int32_t x[4], y[4], w[4], d[4];
    int32_t far;    /**< \brief Furthest visible depth; >= 0. */
    int32_t slope;  /**< \brief Depth step that sets a traversal direction; >= 0. */
    int32_t split;  /**< \brief Depth change from a run's first segment that ends the run; >= 0. */
    int32_t floor;  /**< \brief Runs whose average is <= floor are not returned. */
} sh2z_segmesh_view_t;

/** \brief One run of consecutive ring segments.
 *
 * direction +1: draw backward from end to start; -1: draw forward from start
 * to end (both far to near). start == end with direction +1 and average 0 is
 * the whole ring. average is the run's mean depth, truncated toward zero. */
typedef struct {
    uint16_t start, end;
    int16_t direction;
    int16_t reserved;
    int32_t average;
} sh2z_segmesh_run_t;

/** \brief Result codes. */
enum {
    SH2Z_SEGMESH_OK = 0,        /**< \brief Runs written (possibly zero visible). */
    SH2Z_SEGMESH_BAD_ARGS = -1, /**< \brief Null pointer, n < 3 or n too large, or a row/threshold outside the contract. */
    SH2Z_SEGMESH_TOO_MANY = -2  /**< \brief More runs than max_runs; the output is incomplete. */
};

/** \brief Scratch the caller provides: one depth and one state per segment, plus
 * one sentinel slot each (n + 1 entries). */
typedef struct {
    int32_t *depth;  /**< \brief n + 1 entries. */
    int8_t *state;   /**< \brief n + 1 entries. */
} sh2z_segmesh_scratch_t;

/** \brief Classify a ring and return its runs furthest first.
 *
 * \param anchors     n anchors, ring order (segment n-1 is followed by segment 0).
 * \param n           Ring length, 3..SH2Z_SEGMESH_MAX_SEGMENTS.
 * \param view        Clip rows and thresholds within the contract above.
 * \param scratch     Caller storage, n + 1 entries each; contents are undefined afterward.
 * \param runs        Output, at most max_runs entries, furthest first.
 * \param max_runs    Capacity of runs (also the run pool size).
 * \param run_count   Number of runs written.
 * \param marked      Number of segments marked for drawing (visible, or the predecessor of a visible one).
 * \returns SH2Z_SEGMESH_OK, or a negative code; on BAD_ARGS nothing is written.
 *
 * Semantics, in order:
 * 1. depth[i] = d(p_i); visible[i] as above.
 * 2. With the ring closed (segment n acts as segment 0), for i = 0..n-1: if
 *    segment i or i+1 is visible, step = depth[i+1] - depth[i]; state[i] = -1 if
 *    step < -slope, +1 if step > slope, otherwise the previous marked state
 *    (initially +1); else state[i] = 0.
 * 3. If no segment is marked, no runs. If all n are marked, one whole-ring run.
 *    Otherwise runs start at the first marked segment after the last unmarked
 *    one; a run extends while the depth stays within split of its first
 *    segment, the state is unchanged and the start is not reached again. The
 *    segment that ends a run is included in that run (as its end and in its
 *    average) and starts the next run when it is marked.
 * 4. Runs are returned by decreasing average; ties keep ring order; runs with
 *    average <= floor are dropped.
 */
int sh2z_segmesh_pick(const sh2z_segmesh_anchor_t *anchors, uint32_t n,
                         const sh2z_segmesh_view_t *view,
                         const sh2z_segmesh_scratch_t *scratch,
                         sh2z_segmesh_run_t *runs, uint32_t max_runs,
                         uint32_t *run_count, uint32_t *marked);


/* ---- Contract tag segmesh-2: cluster bounds culling ----------------------- */

/** \brief Bounds of one cluster of a segment's faces, baked offline (20 bytes).
 *
 * centre and radius: a sphere, in the view batch's int16 model units, that
 * holds every vertex of the cluster. axis (Q1.14, unit length to within
 * 2^-13), cos_q14 and sin_q14: a cone of half angle t holding every face
 * normal, with cos_q14 <= 16384 cos t and sin_q14 >= 16384 sin t. A face
 * normal is (b - a) x (c - a) for corners a, b, c in drawing order; a quad
 * a, b, c, d contributes the normals of a, b, c and of a, c, d. Set
 * SH2Z_SEGMESH_NO_CONE in flags when no cone applies. Cones need t < 80
 * degrees: the test's rounding allowance assumes cos t >= 0.17. */
typedef struct {
    int16_t centre[3];
    uint16_t radius;
    int16_t axis[3];
    int16_t cos_q14, sin_q14;
    uint16_t flags;
} sh2z_segmesh_bound_t;

#define SH2Z_SEGMESH_NO_CONE 1u
/** \brief Most bounds per sh2z_segmesh_cull call (one result bit each). */
#define SH2Z_SEGMESH_CULL_MAX 32u
/** \brief Largest side-plane margin, in pixels. */
#define SH2Z_SEGMESH_MARGIN_MAX 64

/** \brief Per-view state from sh2z_segmesh_cull_prepare (treat as opaque). */
typedef struct {
    int16_t r[3][3];
    int16_t facing;            /**< \brief +1, or -1 when the rotation reflects */
    int32_t t[3];
    int32_t near_z, far_z;
    int32_t focal_x, focal_y;
    int32_t k[4];              /**< \brief left, right, top, bottom plane offsets (Q16 pixels) */
    int32_t length[4];         /**< \brief their normal lengths, rounded up (see the source) */
} sh2z_segmesh_cull_t;

/** \brief Prepare culling for one centre view.
 *
 * margin (whole pixels, 0..SH2Z_SEGMESH_MARGIN_MAX) moves the side planes
 * outward. Returns false, leaving out unchanged, on a null pointer, a view
 * outside the view batch's contract (sh2z_view_t, sh2z_view_center_t) or a
 * margin outside its range. */
bool sh2z_segmesh_cull_prepare(sh2z_segmesh_cull_t *out, const sh2z_view_center_t *view,
                               int32_t margin);

/** \brief Classify up to SH2Z_SEGMESH_CULL_MAX bounds. Bit i of the result is
 * clear only when every face of bound i is culled for the prepared view.
 *
 * With p = r * centre + t (Q18.14, the view batch's transform) and the sphere
 * scaled by the same factor, bound i is culled when
 *  - the sphere lies wholly beyond the near plane (z < near_z) or the far plane
 *    (z > far_z), or wholly outside one of the four side planes of the view's
 *    clip rectangle moved outward by margin pixels, as the half spaces of the
 *    view guard clip (sh2z_view_clip_guard, _center form). Then every vertex
 *    in front of the near plane projects outside that side, and the six-plane
 *    clip of every face is empty; or
 *  - its cone applies and dot(q, n) <= 0 for every point q of the sphere and
 *    every normal n of the cone, with n in view space (negated when the
 *    rotation reflects). The projected signed area
 *    sum(sx_i * sy_{i+1} - sy_i * sx_{i+1}) of every face (sy grows downward),
 *    and of any part of it in front of the near plane, is then >= 0, up to
 *    the projection's rounding of corners.
 * Integer arithmetic, every rounding against culling: the cone test runs in
 * 1/16-unit steps; no float and no divide. Bits n..31 are zero; a null
 * pointer or n > SH2Z_SEGMESH_CULL_MAX gives all n low bits set (nothing
 * culled).
 *
 * \warning Not exact. The cluster path is an approximation of the per-face
 * path: the facing test holds only up to the projection's rounding of
 * corners, so a culled bound can still hold a face that the per-face path
 * would draw (a near edge-on face a pixel wide). On one instrumented road
 * renderer it skipped 0.08 such faces per frame. Callers that need the
 * per-face result exactly keep cluster culling off and use sh2z_segmesh_pick
 * alone. */
uint32_t sh2z_segmesh_cull(const sh2z_segmesh_cull_t *cull, const sh2z_segmesh_bound_t *bounds,
                           uint32_t n);

#ifdef __cplusplus
}
#endif

#endif /* SH2ZAM_SEGMESH_EXPERIMENTAL_H */
