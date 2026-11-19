/* Experimental segmented strip mesh visibility pick and run ordering (segmesh-1).
 *
 * Copyright (c) 2026 SH2ZAM Contributors. MIT License.
 * Inspired by SH4ZAM by Falco Girgis and SH4ZAM Contributors.
 */
#include "sh2zam/segmesh_experimental.h"

static bool segmesh_row_ok(const int32_t r[4]) {
    for (unsigned i = 0; i < 3u; ++i)
        if (r[i] > SH2Z_SEGMESH_COEF_MAX || r[i] < -SH2Z_SEGMESH_COEF_MAX) return false;
    return r[3] <= SH2Z_SEGMESH_TRANS_MAX && r[3] >= -SH2Z_SEGMESH_TRANS_MAX;
}

/* Within the row contract the sum fits int32 (see the header). */
static inline int32_t segmesh_eval(const int32_t r[4], const sh2z_segmesh_anchor_t *p) {
    return r[0] * (int32_t)p->x + r[1] * (int32_t)p->y + r[2] * (int32_t)p->z + r[3];
}

static inline int32_t segmesh_abs(int32_t v) { return v < 0 ? -v : v; }

/* Mean truncated toward zero. The 64-bit divide only runs for sums outside int32. */
static int32_t segmesh_mean(int64_t sum, uint32_t count) {
    if (sum >= INT32_MIN && sum <= INT32_MAX) return (int32_t)sum / (int32_t)count;
    return (int32_t)(sum / (int64_t)count);
}

int sh2z_segmesh_pick(const sh2z_segmesh_anchor_t *anchors, uint32_t n,
                      const sh2z_segmesh_view_t *view,
                      const sh2z_segmesh_scratch_t *scratch,
                      sh2z_segmesh_run_t *runs, uint32_t max_runs,
                      uint32_t *run_count, uint32_t *marked) {
    if (!anchors || !view || !scratch || !scratch->depth || !scratch->state || !runs ||
        !run_count || !marked || n < 3u || n > SH2Z_SEGMESH_MAX_SEGMENTS || max_runs == 0u ||
        !segmesh_row_ok(view->x) || !segmesh_row_ok(view->y) || !segmesh_row_ok(view->w) ||
        !segmesh_row_ok(view->d) || view->far < 0 || view->slope < 0 || view->split < 0)
        return SH2Z_SEGMESH_BAD_ARGS;

    int32_t *const depth = scratch->depth;
    int8_t *const state = scratch->state;
    *run_count = 0u;
    *marked = 0u;

    /* 1. Depth and visibility per anchor. */
    for (uint32_t i = 0; i < n; ++i) {
        const sh2z_segmesh_anchor_t *p = &anchors[i];
        const int32_t d = segmesh_eval(view->d, p);
        depth[i] = d;
        if (d < 0 || d > view->far) { state[i] = 0; continue; }
        const int32_t w = segmesh_abs(segmesh_eval(view->w, p));
        state[i] = (int8_t)(segmesh_abs(segmesh_eval(view->x, p)) <= w &&
                            segmesh_abs(segmesh_eval(view->y, p)) <= w);
    }
    depth[n] = depth[0];
    state[n] = state[0];

    /* 2. Traversal direction from the depth step to the successor. */
    int8_t last = 1;
    uint32_t count = 0;
    for (uint32_t i = 0; i < n; ++i) {
        if (state[i] != 0 || state[i + 1u] != 0) {
            const int64_t step = (int64_t)depth[i + 1u] - depth[i];
            if (step < -view->slope) last = -1;
            else if (step > view->slope) last = 1;
            state[i] = last;
            ++count;
        } else {
            state[i] = 0;
        }
    }
    state[n] = state[0];
    *marked = count;
    if (count == 0u) return SH2Z_SEGMESH_OK;

    if (count == n) {
        if (0 <= view->floor) return SH2Z_SEGMESH_OK;
        runs[0] = (sh2z_segmesh_run_t){ 0u, 0u, 1, 0, 0 };
        *run_count = 1u;
        return SH2Z_SEGMESH_OK;
    }

    /* 3. Runs, starting after the last unmarked segment. The run pool is the
     * output array; step 4 reorders it in place. */
    uint32_t start;
    if (state[n - 1u] == 0) {
        start = 0;
        while (state[start] == 0) ++start;
    } else {
        start = n - 2u;
        while (state[start] != 0) --start;
        ++start;
    }
    uint32_t c = start, pool = 0;
    do {
        sh2z_segmesh_run_t *run = &runs[pool];
        const int32_t first = depth[c];
        const int8_t s = state[c];
        int64_t sum = first;
        uint32_t members = 1;
        run->start = (uint16_t)c;
        run->direction = s;
        run->reserved = 0;
        for (;;) {
            if (++c == n) c = 0;
            ++members;
            sum += depth[c];
            const int64_t delta = (int64_t)first - depth[c];
            if (delta < -view->split || delta > view->split || c == start || s != state[c]) break;
        }
        run->end = (uint16_t)c;
        run->average = segmesh_mean(sum, members);
        if (++pool == max_runs) { *run_count = pool; return SH2Z_SEGMESH_TOO_MANY; }
        while (state[c] == 0) if (++c == n) c = 0;
    } while (c != start);

    /* 4. Furthest first: selection by strictly greater average, so ties keep
     * creation order; runs at or below floor are dropped. */
    uint32_t out = 0;
    while (out < pool) {
        uint32_t best = pool;
        int32_t best_average = view->floor;
        for (uint32_t i = out; i < pool; ++i)
            if (runs[i].average > best_average) { best_average = runs[i].average; best = i; }
        if (best == pool) break;
        /* Keep the remaining runs in creation order: rotate best down to out. */
        const sh2z_segmesh_run_t chosen = runs[best];
        for (uint32_t i = best; i > out; --i) runs[i] = runs[i - 1u];
        runs[out++] = chosen;
    }
    *run_count = out;
    return SH2Z_SEGMESH_OK;
}
