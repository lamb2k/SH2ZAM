/* Cluster bounds culling (segmesh-2): property and contract tests.
 *
 * Random views (some reflecting) and random clusters of near-planar quads.
 * Bounds are built as an offline baker would (sphere around the vertices,
 * cone around both triangle normals of every quad, half a degree of margin).
 * Whenever sh2z_segmesh_cull culls a bound, every quad of it must be rejected
 * by the view batch's outcodes, emptied by the six-plane guard clip, or have a
 * non-negative signed area (computed exactly from the view-space corners, and
 * counted separately when only the projection's rounding makes it negative).
 *
 * Copyright (c) 2026 SH2ZAM Contributors. MIT License.
 * Inspired by SH4ZAM by Falco Girgis and SH4ZAM Contributors.
 */
#include "sh2zam/segmesh_experimental.h"
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static uint64_t checks;
#define CHECK(e) do { ++checks; if(!(e)) { fprintf(stderr,"FAIL %s:%d: %s\n",__FILE__,__LINE__,#e); exit(EXIT_FAILURE); } } while(0)
static uint32_t rng = UINT32_C(0x2545f491);
static uint32_t rnd(void) { rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5; return rng; }
static int32_t rrange(int32_t lo, int32_t hi) { return lo + (int32_t)(rnd() % (uint32_t)(hi - lo + 1)); }
static double frand(double lo, double hi) { return lo + (hi - lo) * (double)rnd() / 4294967295.0; }

#define QUADS 6
static double g_eye[3], g_rows[3][3];   /* the last random view's eye (units) and rows */
#define PI 3.14159265358979323846

static void random_view(sh2z_view_center_t *out) {
    for (;;) {
        sh2z_view_t v;
        memset(&v, 0, sizeof v);
        const double yaw = frand(-3.2, 3.2), pitch = frand(-1.2, 1.2), roll = frand(-0.6, 0.6);
        const double cy = cos(yaw), sy = sin(yaw), cp = cos(pitch), sp = sin(pitch), cr = cos(roll), sr = sin(roll);
        double m[3][3] = {{cy * cr + sy * sp * sr, -cy * sr + sy * sp * cr, sy * cp},
                          {cp * sr, cp * cr, -sp},
                          {-sy * cr + cy * sp * sr, sy * sr + cy * sp * cr, cy * cp}};
        /* rows right, up, forward */
        double rows[3][3] = {{m[0][0], m[1][0], m[2][0]}, {m[0][1], m[1][1], m[2][1]}, {m[0][2], m[1][2], m[2][2]}};
        if (rnd() % 4u == 0u) for (unsigned j = 0; j < 3u; ++j) rows[0][j] = -rows[0][j];   /* reflecting */
        for (unsigned i = 0; i < 3u; ++i)
            for (unsigned j = 0; j < 3u; ++j) v.r[i][j] = (int16_t)lround(rows[i][j] * 16384.0);
        v.focal_x = rrange(150, 450) * 65536 + rrange(0, 65535);
        v.focal_y = v.focal_x + rrange(-20000, 20000);
        v.near_z = rrange(1, 64) * 16384;
        v.far_z = v.near_z + rrange(2000, 40000) * 16384;
        v.clip[0] = (int16_t)rrange(-170, -100); v.clip[1] = (int16_t)rrange(-120, -80);
        v.clip[2] = (int16_t)rrange(100, 170); v.clip[3] = (int16_t)rrange(80, 120);
        const sh2z_vec3_t eye = {rrange(-12000, 12000) * 65536 + rrange(0, 65535),
                                 rrange(-3000, 3000) * 65536 + rrange(0, 65535),
                                 rrange(-12000, 12000) * 65536 + rrange(0, 65535)};
        v.center_x = 0; v.center_y = 0;
        g_eye[0] = eye.x / 65536.0; g_eye[1] = eye.y / 65536.0; g_eye[2] = eye.z / 65536.0;
        for (unsigned i = 0; i < 3u; ++i) for (unsigned j = 0; j < 3u; ++j) g_rows[i][j] = rows[i][j];
        if (!sh2z_view_place(&v, eye)) continue;
        const int32_t cx16 = rrange(-3, 2) * 65536 + rrange(0, 65535), cy16 = rrange(-3, 2) * 65536 + rrange(0, 65535);
        if (sh2z_view_center_init(out, &v, cx16, cy16)) return;
    }
}

static void normal(const double a[3], const double b[3], const double c[3], double n[3]) {
    const double u[3] = {b[0] - a[0], b[1] - a[1], b[2] - a[2]}, w[3] = {c[0] - a[0], c[1] - a[1], c[2] - a[2]};
    n[0] = u[1] * w[2] - u[2] * w[1]; n[1] = u[2] * w[0] - u[0] * w[2]; n[2] = u[0] * w[1] - u[1] * w[0];
}

/* Signed area of view-space corners projected without rounding (sy downward). */
static double exact_area(const sh2z_view_center_t *v, const int32_t (*p)[3], unsigned n) {
    double s = 0, x[16], y[16];
    for (unsigned i = 0; i < n; ++i) {
        x[i] = (double)v->view.focal_x * p[i][0] / p[i][2];
        y[i] = -(double)v->view.focal_y * p[i][1] / p[i][2];
    }
    for (unsigned i = 0; i < n; ++i) { const unsigned j = (i + 1u) % n; s += x[i] * y[j] - y[i] * x[j]; }
    return s;
}
static int32_t screen_area(const sh2z_view_vertex_t *v, unsigned n, unsigned stride) {
    int64_t s = 0;
    for (unsigned i = 0; i < n; ++i) {
        const sh2z_view_vertex_t *a = (const sh2z_view_vertex_t *)((const char *)v + i * stride);
        const sh2z_view_vertex_t *b = (const sh2z_view_vertex_t *)((const char *)v + ((i + 1u) % n) * stride);
        s += (int64_t)a->sx * b->sy - (int64_t)a->sy * b->sx;
    }
    return s > 0 ? 1 : s < 0 ? -1 : 0;
}

static uint64_t culled, culled_frustum_only, kept, rounding_only;

static void one_cluster(const sh2z_view_center_t *view, const sh2z_segmesh_cull_t *cull) {
    /* A cluster: QUADS quads around a centre, near one orientation. */
    /* Mostly in or near the view volume: ahead of the eye, across the view. */
    double centre[3];
    {
        const double d = frand(20.0, 9000.0), x = frand(-0.9, 0.9) * d, y = frand(-0.6, 0.6) * d;
        const double back = rnd() % 8u == 0u ? -1.0 : 1.0;
        for (unsigned i = 0; i < 3u; ++i)
            centre[i] = g_eye[i] + back * d * g_rows[2][i] + x * g_rows[0][i] + y * g_rows[1][i];
        for (unsigned i = 0; i < 3u; ++i) if (fabs(centre[i]) > 30000.0) return;
    }
    const double size = frand(4.0, 900.0), tilt = frand(0.0, 0.7);
    double axis[3] = {frand(-1, 1), frand(-1, 1), frand(-1, 1)};
    double m = sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]);
    if (m < 1e-3) return;
    for (unsigned i = 0; i < 3u; ++i) axis[i] /= m;
    double e1[3] = {axis[1], -axis[0], 0}, e2[3];
    if (fabs(axis[2]) > 0.9) { e1[0] = 0; e1[1] = axis[2]; e1[2] = -axis[1]; }
    m = sqrt(e1[0] * e1[0] + e1[1] * e1[1] + e1[2] * e1[2]);
    for (unsigned i = 0; i < 3u; ++i) e1[i] /= m;
    e2[0] = axis[1] * e1[2] - axis[2] * e1[1]; e2[1] = axis[2] * e1[0] - axis[0] * e1[2]; e2[2] = axis[0] * e1[1] - axis[1] * e1[0];
    int16_t corners[QUADS][4][3];
    for (unsigned q = 0; q < QUADS; ++q) {
        const double u0 = frand(-size, size), v0 = frand(-size, size), du = frand(size * 0.1, size), dv = frand(size * 0.1, size);
        const double lift[4] = {frand(-tilt, tilt) * size, frand(-tilt, tilt) * size, frand(-tilt, tilt) * size, frand(-tilt, tilt) * size};
        /* counter-clockwise about axis: (u,v) (u+du,v) (u+du,v+dv) (u,v+dv) gives normal +axis */
        const double uv[4][2] = {{u0, v0}, {u0 + du, v0}, {u0 + du, v0 + dv}, {u0, v0 + dv}};
        for (unsigned k = 0; k < 4u; ++k)
            for (unsigned i = 0; i < 3u; ++i)
                corners[q][k][i] = (int16_t)lround(centre[i] + uv[k][0] * e1[i] + uv[k][1] * e2[i] + lift[k] * axis[i]);
    }
    /* Bound, as a baker would. */
    int32_t lo[3] = {INT32_MAX, INT32_MAX, INT32_MAX}, hi[3] = {INT32_MIN, INT32_MIN, INT32_MIN};
    for (unsigned q = 0; q < QUADS; ++q)
        for (unsigned k = 0; k < 4u; ++k)
            for (unsigned i = 0; i < 3u; ++i) {
                if (corners[q][k][i] < lo[i]) lo[i] = corners[q][k][i];
                if (corners[q][k][i] > hi[i]) hi[i] = corners[q][k][i];
            }
    sh2z_segmesh_bound_t b;
    memset(&b, 0, sizeof b);
    double rmax = 0;
    for (unsigned i = 0; i < 3u; ++i) b.centre[i] = (int16_t)((lo[i] + hi[i]) / 2);
    for (unsigned q = 0; q < QUADS; ++q)
        for (unsigned k = 0; k < 4u; ++k) {
            double d2 = 0;
            for (unsigned i = 0; i < 3u; ++i) d2 += ((double)corners[q][k][i] - b.centre[i]) * ((double)corners[q][k][i] - b.centre[i]);
            if (sqrt(d2) > rmax) rmax = sqrt(d2);
        }
    b.radius = (uint16_t)(ceil(rmax) + 1);
    double normals[2 * QUADS][3], sum[3] = {0, 0, 0};
    unsigned nn = 0;
    for (unsigned q = 0; q < QUADS; ++q)
        for (unsigned t = 0; t < 2u; ++t) {
            double a[3], bb[3], c[3], n[3];
            for (unsigned i = 0; i < 3u; ++i) { a[i] = corners[q][0][i]; bb[i] = corners[q][t ? 2 : 1][i]; c[i] = corners[q][t ? 3 : 2][i]; }
            normal(a, bb, c, n);
            const double l = sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
            if (l == 0) continue;
            for (unsigned i = 0; i < 3u; ++i) { normals[nn][i] = n[i] / l; sum[i] += n[i] / l; }
            ++nn;
        }
    const double sl = sqrt(sum[0] * sum[0] + sum[1] * sum[1] + sum[2] * sum[2]);
    double worst = 0;
    if (nn && sl > 1e-6) {
        int16_t aq[3];
        double al = 0;
        for (unsigned i = 0; i < 3u; ++i) { aq[i] = (int16_t)lround(sum[i] / sl * 16384.0); al += (double)aq[i] * aq[i]; }
        al = sqrt(al);
        for (unsigned k = 0; k < nn; ++k) {
            const double c = (normals[k][0] * aq[0] + normals[k][1] * aq[1] + normals[k][2] * aq[2]) / al;
            const double ang = acos(c > 1 ? 1 : c < -1 ? -1 : c) * 180.0 / PI;
            if (ang > worst) worst = ang;
        }
        worst += 0.5;
        for (unsigned i = 0; i < 3u; ++i) b.axis[i] = aq[i];
    }
    if (!nn || sl <= 1e-6 || worst >= 80.0) b.flags = SH2Z_SEGMESH_NO_CONE;
    else {
        b.cos_q14 = (int16_t)floor(cos(worst * PI / 180.0) * 16384.0);
        b.sin_q14 = (int16_t)ceil(sin(worst * PI / 180.0) * 16384.0);
    }
    const uint32_t keep = sh2z_segmesh_cull(cull, &b, 1);
    CHECK(keep <= 1u);
    if (keep) { ++kept; return; }
    ++culled;
    bool any_face = false;
    for (unsigned q = 0; q < QUADS; ++q) {
        sh2z_pos16_t pos[4];
        sh2z_view_vertex_t out[4];
        for (unsigned k = 0; k < 4u; ++k) { pos[k].x = corners[q][k][0]; pos[k].y = corners[q][k][1]; pos[k].z = corners[q][k][2]; }
        CHECK(sh2z_view_batch_center(out, view, pos, sizeof pos[0], 4));
        if (out[0].outcode & out[1].outcode & out[2].outcode & out[3].outcode) continue;
        sh2z_view_guard_vertex_t in[4], clipped[SH2Z_VIEW_GUARD_MAX_OUT];
        memset(in, 0, sizeof in);
        for (unsigned k = 0; k < 4u; ++k) in[k].v = out[k];
        const int n = sh2z_view_clip_guard_src_center(clipped, view, view->view.clip, in, 4, 0, SH2Z_VIEW_GUARD_MAX_OUT);
        CHECK(n >= 0);
        if (n < 3) continue;
        any_face = true;
        int32_t p[16][3];
        for (int k = 0; k < n; ++k) { p[k][0] = clipped[k].v.x; p[k][1] = clipped[k].v.y; p[k][2] = clipped[k].v.z; }
        const double exact = exact_area(view, (const int32_t (*)[3])p, (unsigned)n);
        CHECK(exact >= -1e-6 * fabs(exact) - 1e-9);
        if (screen_area(&clipped[0].v, (unsigned)n, sizeof clipped[0]) < 0) ++rounding_only;
        bool front = true;
        for (unsigned k = 0; k < 4u; ++k) if (out[k].outcode & SH2Z_OUT_NEAR) front = false;
        if (front) {
            int32_t c4[4][3];
            for (unsigned k = 0; k < 4u; ++k) { c4[k][0] = out[k].x; c4[k][1] = out[k].y; c4[k][2] = out[k].z; }
            CHECK(exact_area(view, (const int32_t (*)[3])c4, 4) >= 0);
            if (screen_area(out, 4, sizeof out[0]) < 0) ++rounding_only;
        }
    }
    if (!any_face) ++culled_frustum_only;
}

int main(void) {
    /* Contract: arguments. */
    sh2z_segmesh_cull_t c;
    sh2z_view_center_t v;
    random_view(&v);
    CHECK(!sh2z_segmesh_cull_prepare(NULL, &v, 2));
    CHECK(!sh2z_segmesh_cull_prepare(&c, NULL, 2));
    CHECK(!sh2z_segmesh_cull_prepare(&c, &v, -1));
    CHECK(!sh2z_segmesh_cull_prepare(&c, &v, SH2Z_SEGMESH_MARGIN_MAX + 1));
    CHECK(sh2z_segmesh_cull_prepare(&c, &v, 2));
    CHECK(sh2z_segmesh_cull(NULL, NULL, 5) == 31u);
    CHECK(sh2z_segmesh_cull(&c, NULL, 32) == UINT32_C(0xFFFFFFFF));
    CHECK(sh2z_segmesh_cull(&c, NULL, 0) == 0u);
    /* A bound behind the eye is culled; one straight ahead is kept. */
    {
        sh2z_view_t s;
        memset(&s, 0, sizeof s);
        s.r[0][0] = 16384; s.r[1][1] = 16384; s.r[2][2] = 16384;
        s.focal_x = s.focal_y = 300 * 65536; s.near_z = 16384; s.far_z = 30000 * 16384;
        s.clip[0] = -160; s.clip[1] = -112; s.clip[2] = 159; s.clip[3] = 111;
        sh2z_view_center_t sc;
        CHECK(sh2z_view_center_init(&sc, &s, 0, 0));
        CHECK(sh2z_segmesh_cull_prepare(&c, &sc, 2));
        sh2z_segmesh_bound_t b[3];
        memset(b, 0, sizeof b);
        b[0].centre[2] = -500; b[0].radius = 100; b[0].flags = SH2Z_SEGMESH_NO_CONE;
        b[1].centre[2] = 500; b[1].radius = 100; b[1].flags = SH2Z_SEGMESH_NO_CONE;
        /* Ahead with normal (b - a) x (c - a) = +z: counter-clockwise on screen
         * (negative signed area with sy downward), so drawn. */
        b[2].centre[2] = 500; b[2].radius = 10; b[2].axis[2] = 16384; b[2].cos_q14 = 16000; b[2].sin_q14 = 3600;
        CHECK(sh2z_segmesh_cull(&c, b, 3) == 6u);
        b[2].axis[2] = -16384;   /* normal toward the eye: non-negative area, culled */
        CHECK(sh2z_segmesh_cull(&c, b, 3) == 2u);
    }
    /* Properties. */
    for (unsigned views = 0; views < 400u; ++views) {
        random_view(&v);
        CHECK(sh2z_segmesh_cull_prepare(&c, &v, (int32_t)(rnd() % 3u)));
        for (unsigned k = 0; k < 400u; ++k) one_cluster(&v, &c);
    }
    printf("segmesh-2: %" PRIu64 " checks; %" PRIu64 " bounds culled (%" PRIu64 " with no face left by outcode or clip), "
           "%" PRIu64 " kept, %" PRIu64 " faces negative only after rounding\n",
           checks, culled, culled_frustum_only, kept, rounding_only);
    CHECK(culled > 1000u && kept > 1000u && culled - culled_frustum_only > 1000u);
    return 0;
}
