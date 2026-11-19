/* Experimental cluster bounds culling (segmesh-2). See segmesh_experimental.h.
 *
 * Copyright (c) 2026 SH2ZAM Contributors. MIT License.
 * Inspired by SH4ZAM by Falco Girgis and SH4ZAM Contributors.
 */
#include "sh2zam/segmesh_experimental.h"

/* Largest focal length and side-plane offset accepted (Q16): every plane
 * product then fits 2^61 and every plane sum 2^62. */
#define CULL_LIMIT INT32_C(0x40000000)

static uint32_t cull_mag(int32_t a) { return a < 0 ? (uint32_t)0 - (uint32_t)a : (uint32_t)a; }

static bool cull_view_ok(const sh2z_view_center_t *w) {
    if (!w) return false;
    const sh2z_view_t *v = &w->view;
    if (v->focal_x <= 0 || v->focal_y <= 0 || v->focal_x > CULL_LIMIT || v->focal_y > CULL_LIMIT ||
        v->near_z <= 0 || v->far_z <= v->near_z || v->clip[0] > v->clip[2] || v->clip[1] > v->clip[3])
        return false;
    for (unsigned i = 0; i < 3u; ++i) {
        uint64_t sum = 0;
        for (unsigned j = 0; j < 3u; ++j) sum += cull_mag(v->r[i][j]);
        if (sum * UINT64_C(32768) + cull_mag(v->t[i]) > (uint64_t)INT32_MAX) return false;
    }
    return true;
}

/* Smallest s with s * s >= a. */
static uint32_t cull_sqrt_up(uint64_t a) {
    uint64_t r = 0, bit = UINT64_C(1) << 62;
    uint64_t x = a;
    while (bit > x) bit >>= 2;
    while (bit) {
        if (x >= r + bit) { x -= r + bit; r = (r >> 1) + bit; }
        else r >>= 1;
        bit >>= 2;
    }
    return (uint32_t)(r * r < a ? r + 1u : r);
}

/* floor(a / 2^n) and ceil(a / 2^n) without signed shifts. */
static int64_t cull_floor(int64_t a, unsigned n) {
    return a >= 0 ? (int64_t)((uint64_t)a >> n) : -(int64_t)((~(uint64_t)a) >> n) - 1;
}
static int64_t cull_ceil(int64_t a, unsigned n) { return -cull_floor(-a, n); }

bool sh2z_segmesh_cull_prepare(sh2z_segmesh_cull_t *out, const sh2z_view_center_t *view,
                               int32_t margin) {
    if (!out || !cull_view_ok(view) || margin < 0 || margin > SH2Z_SEGMESH_MARGIN_MAX) return false;
    const sh2z_view_t *v = &view->view;
    const int64_t left = (int64_t)v->clip[0] - margin, right = (int64_t)v->clip[2] + margin;
    const int64_t top = (int64_t)v->clip[1] - margin, bottom = (int64_t)v->clip[3] + margin;
    /* The guard clip's side planes (half spaces, inside >= 0):
     *   left   focal_x*x - k0*z      k0 = 65536*left - cx16
     *   right  k1*z - focal_x*x      k1 = 65536*right - cx16
     *   top    k2*z - focal_y*y      k2 = cy16 - 65536*top
     *   bottom focal_y*y - k3*z      k3 = cy16 - 65536*bottom */
    const int64_t k[4] = {65536 * left - view->center_x16, 65536 * right - view->center_x16,
                          view->center_y16 - 65536 * top, view->center_y16 - 65536 * bottom};
    sh2z_segmesh_cull_t c;
    for (unsigned i = 0; i < 4u; ++i) {
        if (k[i] > CULL_LIMIT || k[i] < -CULL_LIMIT) return false;
        const int64_t f = i < 2u ? v->focal_x : v->focal_y;
        c.k[i] = (int32_t)k[i];
        c.length[i] = (int32_t)cull_sqrt_up((uint64_t)(f * f) + (uint64_t)(k[i] * k[i]));
    }
    int64_t det = 0;
    for (unsigned j = 0; j < 3u; ++j) {
        const unsigned a = (j + 1u) % 3u, b = (j + 2u) % 3u;
        det += (int64_t)v->r[0][j] * ((int64_t)v->r[1][a] * v->r[2][b] - (int64_t)v->r[1][b] * v->r[2][a]);
    }
    for (unsigned i = 0; i < 3u; ++i) {
        for (unsigned j = 0; j < 3u; ++j) c.r[i][j] = v->r[i][j];
        c.t[i] = v->t[i];
    }
    c.facing = (int16_t)(det < 0 ? -1 : 1);
    c.near_z = v->near_z; c.far_z = v->far_z;
    c.focal_x = v->focal_x; c.focal_y = v->focal_y;
    *out = c;
    return true;
}

static bool cull_one(const sh2z_segmesh_cull_t *c, const sh2z_segmesh_bound_t *b) {
    /* Centre in view space; the view contract keeps each row within int32. */
    int32_t p[3];
    for (unsigned i = 0; i < 3u; ++i)
        p[i] = c->r[i][0] * (int32_t)b->centre[0] + c->r[i][1] * (int32_t)b->centre[1] +
               c->r[i][2] * (int32_t)b->centre[2] + c->t[i];
    /* Radius in Q18.14, allowing rows up to 1 + 2^-9 long and 3 units of rounding. */
    const int64_t radius = b->radius;
    const int64_t r14 = radius * 16384 + radius * 32 + 3 * 16384;
    if ((int64_t)p[2] + r14 < c->near_z || (int64_t)p[2] - r14 > c->far_z) return true;
    const int64_t d[4] = {
        (int64_t)c->focal_x * p[0] - (int64_t)c->k[0] * p[2],
        (int64_t)c->k[1] * p[2] - (int64_t)c->focal_x * p[0],
        (int64_t)c->k[2] * p[2] - (int64_t)c->focal_y * p[1],
        (int64_t)c->focal_y * p[1] - (int64_t)c->k[3] * p[2]};
    for (unsigned i = 0; i < 4u; ++i)
        if (d[i] < -(r14 * c->length[i])) return true;
    if (b->flags & SH2Z_SEGMESH_NO_CONE) return false;
    /* Cone: culled when dot(centre, a) cos t + |centre| sin t + R <= 0 (a the
     * view-space axis), the bound of dot(q, n) over the sphere and the cone.
     * In 1/16 units; sin is raised by 32/16384 for the axis and rotation
     * rounding; each rounding raises the bound. */
    int32_t q[3], n[3];
    for (unsigned i = 0; i < 3u; ++i) {
        q[i] = (int32_t)cull_floor(p[i], 10);
        const int32_t s = c->r[i][0] * (int32_t)b->axis[0] + c->r[i][1] * (int32_t)b->axis[1] +
                          c->r[i][2] * (int32_t)b->axis[2];
        n[i] = c->facing * (int32_t)cull_floor(s, 14);
    }
    const int64_t dot = (int64_t)q[0] * n[0] + (int64_t)q[1] * n[1] + (int64_t)q[2] * n[2];
    const int32_t dot4 = (int32_t)cull_ceil(dot, 14);           /* 1/16 units, <= 2^25 */
    const int64_t x = cull_floor(-(int64_t)dot4 * b->cos_q14, 14) - (radius * 16 + radius / 512 + 48);
    if (x <= 0) return false;
    const uint64_t length2 = (uint64_t)((int64_t)q[0] * q[0] + (int64_t)q[1] * q[1] + (int64_t)q[2] * q[2]);
    const uint32_t s = (uint32_t)(b->sin_q14 < 0 ? 0 : b->sin_q14) + 32u;
    const uint64_t rhs = ((((length2 + 16383u) >> 14) * (uint64_t)(s * s)) + 16383u) >> 14;
    return (uint64_t)(x * x) > rhs;
}

uint32_t sh2z_segmesh_cull(const sh2z_segmesh_cull_t *cull, const sh2z_segmesh_bound_t *bounds,
                           uint32_t n) {
    const uint32_t all = n >= 32u ? UINT32_C(0xFFFFFFFF) : (UINT32_C(1) << n) - 1u;
    if (!cull || !bounds || n > SH2Z_SEGMESH_CULL_MAX) return all;
    uint32_t keep = 0;
    for (uint32_t i = 0; i < n; ++i)
        if (!cull_one(cull, bounds + i)) keep |= UINT32_C(1) << i;
    return keep;
}
