/* Segmented strip mesh pick (segmesh-1): oracle and contract tests.
 *
 * The oracle follows the reference planner's structure independently: a ring
 * stored with a sentinel slot after its last segment, pointer walks, int64
 * arithmetic, a fixed run pool and repeated furthest-first selection that marks
 * chosen runs as drawn.
 *
 * Copyright (c) 2026 SH2ZAM Contributors. MIT License.
 * Inspired by SH4ZAM by Falco Girgis and SH4ZAM Contributors.
 */
#include "sh2zam/segmesh_experimental.h"
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static uint64_t checks;
#define CHECK(e) do { ++checks; if(!(e)) { fprintf(stderr,"FAIL %s:%d: %s\n",__FILE__,__LINE__,#e); exit(EXIT_FAILURE); } } while(0)
static uint32_t rng = UINT32_C(0x9e3779b9);
static uint32_t rnd(void) { rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5; return rng; }
static int32_t rrange(int32_t lo, int32_t hi) { return lo + (int32_t)(rnd() % (uint32_t)(hi - lo + 1)); }

#define MAXN 600u
#define POOL 64u

typedef struct { int64_t depth; int state; } node_t;
typedef struct { node_t *start, *end; int state; int64_t sum, members; int64_t average; } group_t;

static int64_t row(const int32_t r[4], const sh2z_segmesh_anchor_t *p) {
    return (int64_t)r[0] * p->x + (int64_t)r[1] * p->y + (int64_t)r[2] * p->z + r[3];
}
static int64_t abs64(int64_t v) { return v < 0 ? -v : v; }

/* Returns 0 ok, -2 pool overflow. out[] receives runs in draw order. */
static int oracle(const sh2z_segmesh_anchor_t *a, uint32_t n, const sh2z_segmesh_view_t *v,
                  sh2z_segmesh_run_t *out, uint32_t *out_count, uint32_t *marked) {
    static node_t ring[MAXN + 1];
    static group_t pool[POOL];
    node_t *const first = ring, *const sentinel = ring + n;
    for (uint32_t i = 0; i < n; ++i) {
        ring[i].depth = row(v->d, &a[i]);
        if (ring[i].depth < 0 || ring[i].depth > v->far) { ring[i].state = 0; continue; }
        const int64_t w = abs64(row(v->w, &a[i]));
        ring[i].state = abs64(row(v->x, &a[i])) <= w && abs64(row(v->y, &a[i])) <= w;
    }
    *sentinel = ring[0];
    int last = 1; uint32_t count = 0;
    for (node_t *c = first, *next = first + 1; next <= sentinel; c = next, ++next) {
        if (c->state || next->state) {
            const int64_t step = next->depth - c->depth;
            if (step < -v->slope) last = -1; else if (step > v->slope) last = 1;
            c->state = last; ++count;
        } else c->state = 0;
    }
    *marked = count; *out_count = 0;
    if (!count) return 0;
    group_t *end;
    if (count == n) {
        pool[0] = (group_t){ first, first, 1, 0, 1, 0 };
        end = pool + 1;
    } else {
        node_t *c = sentinel - 1;
        if (!c->state) { c = first; while (!c->state) ++c; }
        else { --c; while (c->state) --c; ++c; }
        node_t *const parse = c;
        group_t *g = pool;
        do {
            g->start = c; g->state = c->state; g->sum = c->depth; g->members = 1;
            const int64_t f = c->depth;
            for (;;) {
                if (++c == sentinel) c = first;
                ++g->members; g->sum += c->depth;
                const int64_t delta = f - c->depth;
                if (delta < -v->split || delta > v->split || c == parse || g->state != c->state) break;
            }
            g->end = c;
            g->average = g->sum / g->members;
            if (++g == pool + POOL) return -2;
            while (!c->state) if (++c == sentinel) c = first;
        } while (c != parse);
        end = g;
    }
    const int64_t drawn = (int64_t)INT32_MIN - 1;
    for (;;) {
        int64_t best = v->floor; group_t *pick = NULL;
        for (group_t *g = pool; g != end; ++g) if (best < g->average) { best = g->average; pick = g; }
        if (!pick) break;
        sh2z_segmesh_run_t *r = &out[(*out_count)++];
        r->start = (uint16_t)(pick->start - first); r->end = (uint16_t)(pick->end - first);
        r->direction = (int16_t)pick->state; r->reserved = 0; r->average = (int32_t)pick->average;
        pick->average = drawn;
    }
    return 0;
}

/* A ring of anchors along a closed, wobbling loop, so depth runs rise and fall. */
static void make_ring(sh2z_segmesh_anchor_t *a, uint32_t n, int32_t radius, int32_t wobble) {
    for (uint32_t i = 0; i < n; ++i) {
        /* Integer octagon-ish loop: avoid libm in the generator. */
        const uint32_t phase = (i * 4096u) / n;  /* 0..4095 */
        const int32_t q = (int32_t)(phase & 1023u);
        int32_t x, z;
        switch (phase >> 10) {
            case 0: x = radius - q * radius / 1024; z = q * radius / 1024; break;
            case 1: x = -q * radius / 1024; z = radius - q * radius / 1024; break;
            case 2: x = -radius + q * radius / 1024; z = -q * radius / 1024; break;
            default: x = q * radius / 1024; z = -radius + q * radius / 1024; break;
        }
        a[i].x = (int16_t)x;
        a[i].y = (int16_t)(wobble ? rrange(-wobble, wobble) : 0);
        a[i].z = (int16_t)z;
    }
}

static void random_view(sh2z_segmesh_view_t *v) {
    memset(v, 0, sizeof(*v));
    for (unsigned i = 0; i < 3u; ++i) {
        v->x[i] = rrange(-SH2Z_SEGMESH_COEF_MAX, SH2Z_SEGMESH_COEF_MAX);
        v->y[i] = rrange(-SH2Z_SEGMESH_COEF_MAX, SH2Z_SEGMESH_COEF_MAX);
        v->w[i] = rrange(-SH2Z_SEGMESH_COEF_MAX, SH2Z_SEGMESH_COEF_MAX);
        v->d[i] = rrange(-SH2Z_SEGMESH_COEF_MAX, SH2Z_SEGMESH_COEF_MAX);
    }
    v->x[3] = rrange(-(1 << 26), 1 << 26); v->y[3] = rrange(-(1 << 26), 1 << 26);
    v->w[3] = rrange(-(1 << 26), 1 << 26); v->d[3] = rrange(-(1 << 28), 1 << 28);
    v->far = rrange(0, 1 << 29);
    v->slope = rrange(0, 1 << 22);
    v->split = rrange(0, 1 << 25);
    v->floor = rrange(-(1 << 29), 0);
}

static void compare(const sh2z_segmesh_anchor_t *a, uint32_t n, const sh2z_segmesh_view_t *v) {
    static int32_t depth[MAXN + 1]; static int8_t state[MAXN + 1];
    static sh2z_segmesh_run_t got[POOL], want[POOL];
    const sh2z_segmesh_scratch_t s = { depth, state };
    uint32_t gc = 0, gm = 0, wc = 0, wm = 0;
    const int rc = sh2z_segmesh_pick(a, n, v, &s, got, POOL, &gc, &gm);
    const int orc = oracle(a, n, v, want, &wc, &wm);
    if (orc == -2) { CHECK(rc == SH2Z_SEGMESH_TOO_MANY); CHECK(gm == wm); return; }
    CHECK(rc == SH2Z_SEGMESH_OK);
    CHECK(gm == wm);
    CHECK(gc == wc);
    for (uint32_t i = 0; i < gc; ++i) {
        CHECK(got[i].start == want[i].start);
        CHECK(got[i].end == want[i].end);
        CHECK(got[i].direction == want[i].direction);
        CHECK(got[i].average == want[i].average);
        CHECK(got[i].reserved == 0);
    }
}

/* A camera-like view: depth along +z from a viewer at (ex, ez) looking along
 * the ring, with a symmetric frustum (|x| <= w, |y| <= w with w = depth). */
static void camera_view(sh2z_segmesh_view_t *v, int32_t ex, int32_t ez, int32_t far) {
    memset(v, 0, sizeof(*v));
    const int32_t k = 4096;
    v->x[0] = k; v->x[3] = -k * ex;
    v->y[1] = k;
    v->w[2] = k; v->w[3] = -k * ez;
    v->d[2] = k; v->d[3] = -k * ez;
    v->far = far * k;
    v->slope = 30 * k / 16;
    v->split = 400 * k / 16;
    v->floor = -10000 * (k / 16);
}

int main(void) {
    static sh2z_segmesh_anchor_t a[MAXN];
    static int32_t depth[MAXN + 1]; static int8_t state[MAXN + 1];
    static sh2z_segmesh_run_t runs[POOL];
    const sh2z_segmesh_scratch_t s = { depth, state };
    uint32_t rc_count, marked;
    sh2z_segmesh_view_t v;

    /* Arguments and contract bounds. */
    make_ring(a, 300, 8000, 0);
    camera_view(&v, 0, -9000, 20000);
    CHECK(sh2z_segmesh_pick(NULL, 300, &v, &s, runs, POOL, &rc_count, &marked) == SH2Z_SEGMESH_BAD_ARGS);
    CHECK(sh2z_segmesh_pick(a, 2, &v, &s, runs, POOL, &rc_count, &marked) == SH2Z_SEGMESH_BAD_ARGS);
    CHECK(sh2z_segmesh_pick(a, SH2Z_SEGMESH_MAX_SEGMENTS + 1u, &v, &s, runs, POOL, &rc_count, &marked) == SH2Z_SEGMESH_BAD_ARGS);
    CHECK(sh2z_segmesh_pick(a, 300, &v, &s, runs, 0, &rc_count, &marked) == SH2Z_SEGMESH_BAD_ARGS);
    {
        sh2z_segmesh_view_t bad = v; bad.x[1] = SH2Z_SEGMESH_COEF_MAX + 1;
        CHECK(sh2z_segmesh_pick(a, 300, &bad, &s, runs, POOL, &rc_count, &marked) == SH2Z_SEGMESH_BAD_ARGS);
        bad = v; bad.d[3] = SH2Z_SEGMESH_TRANS_MAX + 1;
        CHECK(sh2z_segmesh_pick(a, 300, &bad, &s, runs, POOL, &rc_count, &marked) == SH2Z_SEGMESH_BAD_ARGS);
        bad = v; bad.far = -1;
        CHECK(sh2z_segmesh_pick(a, 300, &bad, &s, runs, POOL, &rc_count, &marked) == SH2Z_SEGMESH_BAD_ARGS);
        bad = v; bad.split = -1;
        CHECK(sh2z_segmesh_pick(a, 300, &bad, &s, runs, POOL, &rc_count, &marked) == SH2Z_SEGMESH_BAD_ARGS);
    }

    /* Extreme contract corner: full int16 anchors at maximum coefficients. */
    for (uint32_t i = 0; i < 8; ++i) { a[i].x = (i & 1) ? INT16_MIN : INT16_MAX; a[i].y = (i & 2) ? INT16_MIN : INT16_MAX; a[i].z = (i & 4) ? INT16_MIN : INT16_MAX; }
    memset(&v, 0, sizeof(v));
    for (unsigned i = 0; i < 3u; ++i) { v.x[i] = SH2Z_SEGMESH_COEF_MAX; v.y[i] = -SH2Z_SEGMESH_COEF_MAX; v.w[i] = SH2Z_SEGMESH_COEF_MAX; v.d[i] = SH2Z_SEGMESH_COEF_MAX; }
    v.x[3] = SH2Z_SEGMESH_TRANS_MAX; v.y[3] = -SH2Z_SEGMESH_TRANS_MAX; v.w[3] = SH2Z_SEGMESH_TRANS_MAX; v.d[3] = -SH2Z_SEGMESH_TRANS_MAX;
    v.far = INT32_MAX; v.floor = INT32_MIN;
    compare(a, 8, &v);

    /* Nothing visible: no runs, nothing marked. */
    make_ring(a, 300, 8000, 0);
    camera_view(&v, 0, -9000, 20000);
    v.d[3] = -SH2Z_SEGMESH_TRANS_MAX;
    CHECK(sh2z_segmesh_pick(a, 300, &v, &s, runs, POOL, &rc_count, &marked) == SH2Z_SEGMESH_OK);
    CHECK(rc_count == 0 && marked == 0);

    /* Everything visible: one whole-ring run, unless the floor drops it. */
    memset(&v, 0, sizeof(v));
    v.w[3] = SH2Z_SEGMESH_TRANS_MAX; v.d[3] = 1000; v.far = 2000; v.floor = -1;
    CHECK(sh2z_segmesh_pick(a, 300, &v, &s, runs, POOL, &rc_count, &marked) == SH2Z_SEGMESH_OK);
    CHECK(marked == 300 && rc_count == 1 && runs[0].start == 0 && runs[0].end == 0 && runs[0].direction == 1 && runs[0].average == 0);
    v.floor = 0;
    CHECK(sh2z_segmesh_pick(a, 300, &v, &s, runs, POOL, &rc_count, &marked) == SH2Z_SEGMESH_OK);
    CHECK(marked == 300 && rc_count == 0);

    /* Depth steps exactly at +-slope keep the previous direction; one past it
     * flips. Segments 0 and 1 are hidden so the ring splits into runs. */
    for (int32_t bump = 0; bump <= 1; ++bump) {
        static const int16_t z[8] = { -500, -500, 1000, 970, 940, 910, 880, 850 };
        for (uint32_t i = 0; i < 8; ++i) { a[i].x = 0; a[i].y = 0; a[i].z = z[i]; }
        memset(&v, 0, sizeof(v));
        v.w[3] = SH2Z_SEGMESH_TRANS_MAX; v.d[2] = 1; v.far = 5000; v.slope = 30 - bump; v.split = 100000; v.floor = INT32_MIN;
        compare(a, 8, &v);
        uint32_t c2, m2;
        CHECK(sh2z_segmesh_pick(a, 8, &v, &s, runs, POOL, &c2, &m2) == SH2Z_SEGMESH_OK);
        /* Without the flip one +1 run covers segments 1..7; with it, segment 2
         * starts a -1 run. */
        uint32_t seen = 0;
        for (uint32_t i = 0; i < c2; ++i) {
            if (!bump && runs[i].start == 1u) { CHECK(runs[i].direction == 1 && runs[i].end == 7u); ++seen; }
            if (bump && runs[i].start == 2u) { CHECK(runs[i].direction == -1); ++seen; }
        }
        CHECK(seen == 1u);
    }

    /* Camera walks round the ring: oracle agreement, monotone draw order. */
    make_ring(a, 300, 8000, 300);
    uint32_t walked = 0;
    for (int32_t step = 0; step < 400; ++step) {
        camera_view(&v, rrange(-9000, 9000), rrange(-12000, 12000), rrange(500, 30000));
        compare(a, 300, &v);
        uint32_t c2, m2;
        const int rc = sh2z_segmesh_pick(a, 300, &v, &s, runs, POOL, &c2, &m2);
        CHECK(rc == SH2Z_SEGMESH_OK || rc == SH2Z_SEGMESH_TOO_MANY);
        if (rc == SH2Z_SEGMESH_OK) { ++walked; for (uint32_t i = 1; i < c2; ++i) CHECK(runs[i - 1].average >= runs[i].average); }
    }

    CHECK(walked > 100u);

    /* Random rows, rings and pool sizes, including pool overflow. */
    for (int t = 0; t < 20000; ++t) {
        const uint32_t n = 3u + rnd() % (MAXN - 3u);
        for (uint32_t i = 0; i < n; ++i) { a[i].x = (int16_t)rrange(INT16_MIN, INT16_MAX); a[i].y = (int16_t)rrange(INT16_MIN, INT16_MAX); a[i].z = (int16_t)rrange(INT16_MIN, INT16_MAX); }
        random_view(&v);
        if (t & 1) { make_ring(a, n, rrange(100, 30000), rrange(0, 2000)); camera_view(&v, rrange(-20000, 20000), rrange(-20000, 20000), rrange(0, 40000)); }
        compare(a, n, &v);
    }

    /* Pool overflow is reported, never written past max_runs. */
    {
        uint32_t n = 400, c2 = 0, m2 = 0;
        for (uint32_t i = 0; i < n; ++i) { a[i].x = 0; a[i].y = 0; a[i].z = (int16_t)((i & 1) ? 0 : 1000); }
        memset(&v, 0, sizeof(v));
        v.w[3] = SH2Z_SEGMESH_TRANS_MAX; v.d[2] = 1; v.far = 100000; v.slope = 1000; v.split = 10; v.floor = INT32_MIN;
        a[0].z = a[1].z = -32768; /* segment 0 and its successor hidden: one unmarked segment, so runs split */
        sh2z_segmesh_run_t small[4]; memset(small, 0xA5, sizeof(small));
        const int rc = sh2z_segmesh_pick(a, n, &v, &s, small, 3, &c2, &m2);
        CHECK(rc == SH2Z_SEGMESH_TOO_MANY && c2 == 3);
        CHECK(small[3].start == 0xA5A5u);
    }

    printf("segmesh: %" PRIu64 " checks passed\n", checks);
    return 0;
}
