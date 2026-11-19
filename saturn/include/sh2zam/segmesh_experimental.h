/* Experimental segmented strip mesh: integer visibility pick and far-to-near run
 * ordering for a closed (circular) chain of mesh segments. Not selected by any
 * default dispatch.
 *
 * Contract tag: segmesh-1. A mesh is a ring of N segments, each represented for
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

#ifdef __cplusplus
}
#endif

#endif /* SH2ZAM_SEGMESH_EXPERIMENTAL_H */
