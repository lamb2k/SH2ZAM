/* Track-bound vehicle dynamics (contract tags vehicle-dynamics-1 and -2).
 * Integer only; see vehicle_dynamics_experimental.h and VEHICLE_DYNAMICS.md.
 *
 * This file is the vehicle-dynamics-1 unit. sh2zam_vehicle_dynamics2.c builds
 * it a second time with SH2Z_VD_TUBES set: that unit holds only sh2z_vd_step2,
 * which adds the pipe and cylinder shapes and calls this unit's search. The
 * tube code below is preprocessed out of this unit, so vehicle-dynamics-1's
 * object code does not depend on it.
 *
 * Copyright (c) 2026 SH2ZAM Contributors. MIT License.
 * Inspired by SH4ZAM by Falco Girgis and SH4ZAM Contributors.
 */
#include "sh2zam/vehicle_dynamics_experimental.h"
#include "sh2zam/divu_experimental.h"
#include "sh2zam/unit30_experimental.h"

#ifndef SH2Z_VD_TUBES
#define SH2Z_VD_TUBES 0
#endif

#define ONE30 INT32_C(0x40000000)
#define ONE16 INT32_C(0x10000)
/* vehicle-dynamics-3: sh2zam_vehicle_contact.c builds this file a second time
 * with VD_CONTACT set to apply a contact at the start of the tick. Here it
 * expands to no tokens at all, so sh2z_vd_step keeps its vehicle-dynamics-1
 * code, under LTO too. */
#ifndef VD_CONTACT
#define VD_CONTACT(b, v, r)
#endif
/* vehicle-dynamics-6: hooks for the host tests, empty statements unless a build defines them.
 * FAR_HIT is invoked once per sh2z_vd_locate call that stops on the SH2Z_VD_LOCATE_FAR test. */
#ifndef SH2Z_VD_LOCATE_FAR_HIT
#define SH2Z_VD_LOCATE_FAR_HIT()
#endif
/* Invoked at the top of every pass with the residual (int32_t[3], Q16); empty unless defined. */
#ifndef SH2Z_VD_LOCATE_PASS
#define SH2Z_VD_LOCATE_PASS(d)
#endif
/* The squared residual (Q32) that SH2Z_VD_LOCATE_FAR units make. */
#define VD_FAR_SQ (((int64_t)SH2Z_VD_LOCATE_FAR * SH2Z_VD_LOCATE_FAR) << 32)

/* ---- Arithmetic --------------------------------------------------------- */

/* floor((x + 2^(s-1)) / 2^s) for 1 <= s <= 62, without shifting a negative
 * value (round to nearest, ties up). */
static int64_t vd_r64(int64_t x, unsigned s) {
    const int64_t half = (int64_t)1 << (s - 1u);
    if(x >= -half) return (int64_t)((uint64_t)(x + half) >> s);
    return -(int64_t)(((uint64_t)(-(x + half)) + (((uint64_t)1 << s) - 1u)) >> s);
}

static int32_t vd_sat(int64_t x) {
    if(x > INT32_MAX) return INT32_MAX;
    if(x < INT32_MIN) return INT32_MIN;
    return (int32_t)x;
}

static int32_t vd_round(int64_t x, unsigned s) { return vd_sat(vd_r64(x, s)); }

/* Q16 x Q16 -> Q16; Qn x Q30 -> Qn. */
static int32_t m16(int32_t a, int32_t b) { return vd_round((int64_t)a * b, 16); }
static int32_t mu(int32_t a, int32_t u) { return vd_round((int64_t)a * u, 30); }

/* u . v rounded to v's format (u Q30). */
static int32_t dotu(const int32_t u[3], const int32_t v[3]) {
    return vd_round((int64_t)u[0] * v[0] + (int64_t)u[1] * v[1] + (int64_t)u[2] * v[2], 30);
}

static int64_t sumsq(const int32_t v[3]) {
    return (int64_t)v[0] * v[0] + (int64_t)v[1] * v[1] + (int64_t)v[2] * v[2];
}

/* |v| in v's format. */
static int32_t vlen(const int32_t v[3]) {
    const uint32_t r = sh2z_isqrt64_q12((uint64_t)sumsq(v));
    return r > (uint32_t)INT32_MAX ? INT32_MAX : (int32_t)r;
}

/* round(n / d), ties away from zero, saturated; d != 0. */
static int32_t vd_div(int64_t n, int32_t d) {
    const int64_t h = (d < 0 ? -(int64_t)d : (int64_t)d) >> 1;
    return sh2z_divu_div64(n >= 0 ? n + h : n - h, d, NULL);
}

/* a * 2^s / d, rounded. */
static int32_t vd_divq(int32_t a, int32_t d, unsigned s) { return vd_div((int64_t)a * ((int64_t)1 << s), d); }

static void vcopy(int32_t o[3], const int32_t a[3]) { o[0] = a[0]; o[1] = a[1]; o[2] = a[2]; }

/* o += k * u (k Q16, u Q30) */
static void vadd_ku(int32_t o[3], int32_t k, const int32_t u[3]) {
    o[0] += mu(k, u[0]); o[1] += mu(k, u[1]); o[2] += mu(k, u[2]);
}

/* (a x b) . f, with a and b Q16 and f Q30; exact sign, magnitude in Q16 units^2. */
static int64_t triple(const int32_t a[3], const int32_t b[3], const int32_t f[3]) {
    const int64_t c0 = vd_r64((int64_t)a[1] * b[2] - (int64_t)a[2] * b[1], 16);
    const int64_t c1 = vd_r64((int64_t)a[2] * b[0] - (int64_t)a[0] * b[2], 16);
    const int64_t c2 = vd_r64((int64_t)a[0] * b[1] - (int64_t)a[1] * b[0], 16);
    return c0 * f[0] + c1 * f[1] + c2 * f[2];
}

static bool normalize(int32_t o[3], const int32_t v[3]) { return sh2z_vec3_normalize_u30(o, v); }

/* ---- Spline ------------------------------------------------------------- */

/* n < 3 * count: no division (the SH-2 has no fast integer divide outside the divider). */
static uint32_t wrap(const sh2z_vd_track_t *k, uint32_t n) {
    if(n >= k->count) n -= k->count;
    if(n >= k->count) n -= k->count;
    return n;
}

static const sh2z_vd_node_t *node_at(const sh2z_vd_track_t *k, uint32_t n) { return &k->nodes[wrap(k, n)]; }

#if !SH2Z_VD_TUBES
void sh2z_vd_spline(const sh2z_vd_track_t *k, uint32_t n, int32_t t32, int32_t pos[3], int32_t fwd[3]) {
    const sh2z_vd_node_t *p0 = node_at(k, n + k->count - 1u), *p1 = node_at(k, n);
    const sh2z_vd_node_t *p2 = node_at(k, n + 1u), *p3 = node_at(k, n + 2u);
    /* |t| <= 1.5 keeps every product below 2^63; the search only evaluates
     * t a little outside [0, 1). */
    const int64_t T = p1->tension, t = t32 > ONE30 / 2 * 3 ? ONE30 / 2 * 3 : t32 < -(ONE30 / 2 * 3) ? -(ONE30 / 2 * 3) : t32;
    const int64_t t2 = vd_r64(t * t, 30), t3 = vd_r64(t2 * t, 30);
    const int64_t one = ONE30;
    /* Position weights (the node's own weight is 1 minus the others). */
    const int64_t c0 = vd_r64((2 * t2 - t - t3) * T, 30);
    const int64_t c2 = vd_r64((T - 2 * one) * t3, 30) - vd_r64((2 * T - 3 * one) * t2, 30) + vd_r64(T * t, 30);
    const int64_t c3 = vd_r64((t3 - t2) * T, 30);
    /* Tangent weights (they sum to zero). */
    const int64_t d0 = vd_r64((4 * t - one - 3 * t2) * T, 30);
    const int64_t d2 = vd_r64((3 * T - 6 * one) * t2, 30) + vd_r64((6 * one - 4 * T) * t, 30) + T;
    const int64_t d3 = vd_r64((3 * t2 - 2 * t) * T, 30);
    int i;
    for(i = 0; i < 3; i++) {
        const int64_t a = (int64_t)p0->pos[i] - p1->pos[i];
        const int64_t c = (int64_t)p2->pos[i] - p1->pos[i];
        const int64_t d = (int64_t)p3->pos[i] - p1->pos[i];
        if(pos) pos[i] = vd_sat(p1->pos[i] + vd_r64(c0 * a + c2 * c + c3 * d, 30));
        if(fwd) fwd[i] = vd_round(d0 * a + d2 * c + d3 * d, 30);
    }
}

static int32_t tangent_mag(const sh2z_vd_track_t *k, uint32_t n, int32_t t) {
    int32_t f[3];
    sh2z_vd_spline(k, n, t, NULL, f);
    return vlen(f);
}

int sh2z_vd_locate(sh2z_vd_body_t *b, const sh2z_vd_track_t *k, const int32_t target[3], int iterations,
                   int64_t tolerance) {
    int32_t d[3];
    int i;
#if SH2Z_VD_LOCATE_FAR
    int far_run = 0;
#endif
    for(i = 0; i < 3; i++) d[i] = target[i] - b->ground[i];
    while(sumsq(d) > tolerance && --iterations != 0) {
        SH2Z_VD_LOCATE_PASS(d);
#if SH2Z_VD_LOCATE_FAR
        /* vehicle-dynamics-6: a residual that stays above SH2Z_VD_LOCATE_FAR units for
         * SH2Z_VD_LOCATE_FAR_RUN passes in a row belongs to a search that circles (a body far
         * above a gap has no closest point to reach); it keeps the state it has. A search that
         * overshoots and then recovers, as on a jump, comes back under the line and is not
         * stopped. */
        far_run = sumsq(d) > VD_FAR_SQ ? far_run + 1 : 0;
        if(far_run >= SH2Z_VD_LOCATE_FAR_RUN) {
            SH2Z_VD_LOCATE_FAR_HIT();
            break;
        }
#endif
        const int64_t dot = (int64_t)d[0] * b->fwd[0] + (int64_t)d[1] * b->fwd[1] + (int64_t)d[2] * b->fwd[2];
        const int32_t along = vd_div(dot, b->fwd_mag);              /* Q16 */
        int64_t t = (int64_t)b->t + vd_divq(along, b->fwd_mag, 30);
        int32_t dist, fhat[3], perp[3], n[3];
        if(t >= ONE30) {
            if(t > (int64_t)ONE30 * 19 / 10) return -1;
            b->node = wrap(k, b->node + 1u);
            t = (int64_t)mu(vd_sat(t - ONE30), vd_divq(b->fwd_mag, tangent_mag(k, b->node, 0), 30));
        } else if(t < 0) {
            if(t < -(int64_t)ONE30 * 9 / 10) return -1;
            b->node = wrap(k, b->node + k->count - 1u);
            t = (int64_t)mu(vd_sat(t), vd_divq(b->fwd_mag, tangent_mag(k, b->node, ONE30), 30)) + ONE30;
        }
        b->t = vd_sat(t);
        sh2z_vd_spline(k, b->node, b->t, b->spos, b->fwd);
        b->fwd_mag = vlen(b->fwd);
        for(i = 0; i < 3; i++) b->disp[i] = target[i] - b->spos[i];
        dist = vlen(b->disp);
        if(dist < 66) break;                                         /* 0.001 */
        /* Ground point: the displacement turned into the plane across the
         * spline, keeping its length. */
        if(!normalize(fhat, b->fwd)) return -1;
        {
            const int32_t a = dotu(fhat, b->disp);
            for(i = 0; i < 3; i++) perp[i] = b->disp[i] - mu(a, fhat[i]);
        }
        if(!normalize(n, perp)) {
            for(i = 0; i < 3; i++) b->disp[i] = 0;
        } else {
            for(i = 0; i < 3; i++) b->disp[i] = mu(dist, n[i]);
        }
        for(i = 0; i < 3; i++) {
            b->ground[i] = b->disp[i] + b->spos[i];
            d[i] = target[i] - b->ground[i];
        }
    }
    for(i = 0; i < 3; i++) {
        b->disp[i] = target[i] - b->spos[i];
        b->pos[i] = target[i];
    }
    return 0;
}

int32_t sh2z_vd_lap_distance(sh2z_vd_body_t *b, const sh2z_vd_track_t *k) {
    const sh2z_vd_node_t *n = node_at(k, b->node);
    const int32_t *mark;
    int32_t len, d[3];
    int i;
    if(b->t < ONE30 / 4) { len = 0; mark = n->pos; }
    else if(b->t < ONE30 / 2) { len = n->mark_len[0]; mark = n->mark_pos[0]; }
    else if(b->t < ONE30 / 4 * 3) { len = n->mark_len[1]; mark = n->mark_pos[1]; }
    else { len = n->mark_len[2]; mark = n->mark_pos[2]; }
    for(i = 0; i < 3; i++) d[i] = b->spos[i] - mark[i];
    len += vd_round(vlen(d), 4);                                     /* Q16 -> Q12 */
    b->prop = vd_divq(len, n->length, 30);
    return n->start + len;
}
#endif

/* ---- Contact ------------------------------------------------------------ */

static void damage(sh2z_vd_body_t *b, const sh2z_vd_rules_t *r, int32_t amount) {
    if(b->flags & (SH2Z_VD_CRASHED | SH2Z_VD_FINISHED)) return;
    b->energy -= amount;
    b->flags |= SH2Z_VD_DAMAGED;
    if(b->energy < 0) {
        int32_t s = mu(amount, r->spin_steer);
        b->energy = 0;
        if(s > ONE16) s = ONE16;
        b->spin_steer = (b->race_ticks & 1) ? -s : s;  /* the side is arbitrary */
        if(!(b->flags & (SH2Z_VD_CRASHED | SH2Z_VD_SPINNING))) {
            b->flags = (b->flags | SH2Z_VD_SPINNING) & ~SH2Z_VD_CONTROLLABLE;
            b->spin_timer = 1;
            b->wall_bounce = r->spin_bounce;
        }
    }
}

static void move_by(sh2z_vd_body_t *b, int32_t k, const int32_t u[3]) {
    int i;
    for(i = 0; i < 3; i++) {
        const int32_t m = mu(k, u[i]);
        b->disp[i] -= m;
        b->pos[i] -= m;
        b->ground[i] -= m;
    }
}

static void hit_wall(sh2z_vd_body_t *b, const sh2z_vd_rules_t *r, int32_t side_vel, int32_t dir, int32_t edge) {
    const int32_t into = dir > 0 ? side_vel : -side_vel;
    int i;
    move_by(b, edge, b->road[2]);
    if(into <= 0) return;
    if(into > r->wall_min) damage(b, r, mu(into, r->wall_damage));
    {
        const int32_t k = m16(b->wall_bounce, side_vel);
        for(i = 0; i < 3; i++) b->vel[i] = mu(b->vel[i] - mu(k, b->road[2][i]), r->wall_keep);
    }
    b->flags |= SH2Z_VD_RECOIL;
}

/* Pitch input tilts the up vector; gravity eases toward world up. */
static void airborne_up(sh2z_vd_body_t *b, const sh2z_vd_rules_t *r) {
    const int32_t k = sh2z_u30_mul(sh2z_u30_mul(sh2z_u30_dot3(b->gravity_up, b->tilt_up), r->tilt_input), b->tilt_in);
    int32_t v[3];
    int i;
    for(i = 0; i < 3; i++) v[i] = b->body[1][i] + sh2z_u30_mul(k, b->body[0][i]);
    if(normalize(v, v)) vcopy(b->tilt_up, v);
    b->gravity_up[0] -= vd_round(b->gravity_up[0], 1);
    b->gravity_up[1] += vd_round(ONE30 - b->gravity_up[1], 1);
    b->gravity_up[2] -= vd_round(b->gravity_up[2], 1);
}

static void set_ground_up(sh2z_vd_body_t *b) {
    vcopy(b->ground_up, b->road[1]);
}

static void clamp_to_ground(sh2z_vd_body_t *b) {
    int32_t vn;
    int i;
    move_by(b, b->height, b->road[1]);
    b->height = 0;
    vn = dotu(b->road[1], b->vel);
    if(vn < 0) for(i = 0; i < 3; i++) b->vel[i] -= mu(vn, b->road[1][i]);
}

static void contact_band(sh2z_vd_body_t *b, const sh2z_vd_rules_t *r) {
    int i;
    if(b->height <= r->ground_band) {
        b->flags &= ~SH2Z_VD_AIRBORNE;
        vcopy(b->gravity_up, b->road[1]);
        vcopy(b->tilt_up, b->road[1]);
        return;
    }
    b->flags |= SH2Z_VD_AIRBORNE;
    airborne_up(b, r);
    if(b->height < r->blend_band) {
        const int32_t s = vd_divq(b->height - r->ground_band, r->blend_band - r->ground_band, 30);
        int32_t v[3];
        for(i = 0; i < 3; i++) {
            b->gravity_up[i] = sh2z_u30_mul(b->gravity_up[i] - b->road[1][i], s) + b->road[1][i];
            v[i] = sh2z_u30_mul(b->tilt_up[i] - b->road[1][i], s) + b->tilt_up[i];
        }
        if(normalize(v, v)) vcopy(b->tilt_up, v);
    }
}

static void shape_gap(sh2z_vd_body_t *b, const sh2z_vd_rules_t *r) {
    b->flags |= SH2Z_VD_OFF_ROAD | SH2Z_VD_AIRBORNE;
    b->height = dotu(b->road[1], b->disp);
    set_ground_up(b);
    airborne_up(b, r);
}

static void shape_edgeless(sh2z_vd_body_t *b, const sh2z_vd_rules_t *r) {
    const int32_t side = dotu(b->road[2], b->disp);
    b->height = dotu(b->road[1], b->disp);
    set_ground_up(b);
    if(b->flags & SH2Z_VD_OFF_ROAD) {
        if(b->height < r->edge_low || b->height > r->blend_band || side < -b->half_right || side > b->half_left) {
            airborne_up(b, r);
            return;
        }
        b->flags &= ~SH2Z_VD_OFF_ROAD;
    } else if(b->height > r->blend_band || side < -b->half_right || side > b->half_left) {
        b->flags |= SH2Z_VD_OFF_ROAD | SH2Z_VD_AIRBORNE;
        airborne_up(b, r);
        return;
    }
    if(b->height < 0) clamp_to_ground(b);
    contact_band(b, r);
}

static void shape_road(sh2z_vd_body_t *b, const sh2z_vd_rules_t *r, int32_t air) {
    int32_t side;
    if(b->flags & SH2Z_VD_OFF_ROAD) {
        b->half_left += r->edge_widen;
        b->half_right += r->edge_widen;
        shape_edgeless(b, r);
        b->half_left -= r->edge_widen;
        b->half_right -= r->edge_widen;
        if(b->flags & SH2Z_VD_OFF_ROAD) return;
        side = dotu(b->road[2], b->disp);
        if(side < -b->half_right || side > b->half_left) {
            b->flags |= SH2Z_VD_OFF_ROAD;
            damage(b, r, r->edge_damage);
        }
        return;
    }
    side = dotu(b->road[2], b->disp);
    b->height = dotu(b->road[1], b->disp);
    set_ground_up(b);
    if(b->height > air) {
        b->flags |= SH2Z_VD_OFF_ROAD | SH2Z_VD_AIRBORNE;
        airborne_up(b, r);
        return;
    }
    if(b->height < 0) clamp_to_ground(b);
    contact_band(b, r);
    if(side < -b->half_right) hit_wall(b, r, dotu(b->road[2], b->vel), -1, b->half_right + side);
    else if(side > b->half_left) hit_wall(b, r, dotu(b->road[2], b->vel), 1, side - b->half_left);
}

#if SH2Z_VD_TUBES
/* ---- Tubes (vehicle-dynamics-2) ----------------------------------------- */

/* Where the span joins a neighbour: the join share (Q30, 0 at the seam's far
 * end, 1 where the tube is whole) and the side of the seam (+1 previous, -1
 * next, 0 outside the joins). */
static int join_at(const sh2z_vd_body_t *b, const sh2z_vd_track2_t *k2, int32_t *share) {
    const sh2z_vd_join_t *j;
    *share = ONE30;
    if(!k2->joins) return 0;
    j = &k2->joins[b->node];
    if(b->t < j->prev_end) {
        *share = vd_divq(b->prop, j->scale, 30);
        return 1;
    }
    if(j->next_start < b->t) {
        *share = vd_divq(ONE30 - b->prop, j->scale, 30);
        return -1;
    }
    return 0;
}

/* Contact on the tube's round surface: the body is put back on the radius r
 * along the radial direction and loses its speed into the surface. */
static void radial_clamp(sh2z_vd_body_t *b, int32_t dist, int32_t radius) {
    const int32_t s = vd_divq(radius, dist, 30);
    int32_t vn;
    int i;
    for(i = 0; i < 3; i++) {
        b->disp[i] = mu(b->disp[i], s);
        b->pos[i] = b->ground[i] = b->disp[i] + b->spos[i];
    }
    b->height = 0;
    vn = dotu(b->ground_up, b->vel);
    if(vn < 0) for(i = 0; i < 3; i++) b->vel[i] -= mu(vn, b->ground_up[i]);
}

/* Airborne above the ground band, in contact at or below it. */
static void tube_band(sh2z_vd_body_t *b, const sh2z_vd_rules_t *r) {
    if(b->height <= r->ground_band) b->flags &= ~SH2Z_VD_AIRBORNE;
    else b->flags |= SH2Z_VD_AIRBORNE;
}

static void leave_surface(sh2z_vd_body_t *b, const sh2z_vd_rules_t *r) {
    b->flags |= SH2Z_VD_OFF_ROAD | SH2Z_VD_AIRBORNE;
    airborne_up(b, r);
}

/* Inside a pipe: the ground is the circle of radius half_left round the
 * line; up points to the axis. In a join the road plane (lifted by the
 * remaining radius) takes over toward the seam. No walls. */
static void shape_pipe(sh2z_vd_body_t *b, const sh2z_vd_track2_t *k2, const sh2z_vd_rules_t *r) {
    const int32_t radius = b->half_left;
    int32_t share, floor_r, dist = vlen(b->disp);
    const int side = join_at(b, k2, &share);
    int i;
    floor_r = side ? mu(radius, share) : radius;
    b->height = radius - dist;
    if(dist > k2->tube.min_dist) {
        int32_t u[3];
        if(normalize(u, b->disp)) for(i = 0; i < 3; i++) b->ground_up[i] = -u[i];
    }
    if(side) {
        const int32_t above = dotu(b->road[1], b->disp) - k2->tube.pipe_rim;
        if(b->flags & SH2Z_VD_OFF_ROAD) {
            if(above >= 0 || radius < dist) { airborne_up(b, r); return; }
            b->flags &= ~SH2Z_VD_OFF_ROAD;
        } else if(above >= 0) {
            leave_surface(b, r);
            return;
        }
    } else if(b->flags & SH2Z_VD_OFF_ROAD) {
        if(radius < dist) { airborne_up(b, r); return; }
        b->flags &= ~SH2Z_VD_OFF_ROAD;
    }
    if(side) {
        const int32_t lean = side > 0 ? radius : -radius;
        const int32_t plane = floor_r + dotu(b->road[1], b->disp);
        int32_t v[3];
        if(plane < b->height) {
            b->height = plane;
            vcopy(b->ground_up, b->road[1]);
            for(i = 0; i < 3; i++) v[i] = mu(k2->tube.join_lever, b->road[1][i]) + mu(lean, b->road[0][i]);
            if(normalize(v, v)) vcopy(b->tilt_up, v);
            if(b->height < 0) {
                int32_t vn;
                move_by(b, b->height, b->road[1]);
                dist = vlen(b->disp);
                b->height = 0;
                vn = dotu(b->road[1], b->vel);
                if(vn < 0) for(i = 0; i < 3; i++) b->vel[i] -= mu(vn, b->road[1][i]);
            }
        } else {
            vcopy(b->tilt_up, b->ground_up);
        }
        for(i = 0; i < 3; i++) v[i] = sh2z_u30_mul(b->tilt_up[i] - b->road[1][i], share) + b->road[1][i];
        if(vlen(v) > k2->tube.min_unit && normalize(v, v)) vcopy(b->tilt_up, v);
        for(i = 0; i < 3; i++) b->gravity_up[i] = sh2z_u30_mul(b->ground_up[i] - b->road[1][i], share) + b->road[1][i];
    } else {
        vcopy(b->gravity_up, b->ground_up);
        vcopy(b->tilt_up, b->ground_up);
    }
    if(radius < dist) radial_clamp(b, dist, radius);
    tube_band(b, r);
}

/* Half-pipe rim gain: sin(22.5 degrees) of the (join-tapered) radius is added to the rim test. */
#define HALF_PIPE_RIM_Q30 INT32_C(0x187DE284)

/* Inside a half-pipe (vehicle-dynamics-5): the pipe's ground, circle of radius
 * half_left round the line with up to the line, but a trough, open above. The
 * body leaves it when its height over the line, less tube.pipe_rim, plus
 * HALF_PIPE_RIM_Q30 of the tapered radius reaches zero: that test applies on the
 * whole span, not only over a join. Gravity always acts along the road's up; the
 * pipe's joins taper in the same way, with the same road plane. */
static void shape_half_pipe(sh2z_vd_body_t *b, const sh2z_vd_track2_t *k2, const sh2z_vd_rules_t *r) {
    const int32_t radius = b->half_left;
    int32_t share, floor_r, above, dist = vlen(b->disp);
    const int side = join_at(b, k2, &share);
    int i;
    floor_r = side ? mu(radius, share) : radius;
    b->height = radius - dist;
    if(dist > k2->tube.min_dist) {
        int32_t u[3];
        if(normalize(u, b->disp)) for(i = 0; i < 3; i++) b->ground_up[i] = -u[i];
    }
    above = dotu(b->road[1], b->disp) - k2->tube.pipe_rim + mu(floor_r, HALF_PIPE_RIM_Q30);
    if(b->flags & SH2Z_VD_OFF_ROAD) {
        if(above >= 0 || radius < dist) { airborne_up(b, r); return; }
        b->flags &= ~SH2Z_VD_OFF_ROAD;
    } else if(above >= 0) {
        leave_surface(b, r);
        return;
    }
    if(side) {
        const int32_t lean = side > 0 ? radius : -radius;
        const int32_t plane = floor_r + dotu(b->road[1], b->disp);
        int32_t v[3];
        if(plane < b->height) {
            b->height = plane;
            vcopy(b->ground_up, b->road[1]);
            for(i = 0; i < 3; i++) v[i] = mu(k2->tube.join_lever, b->road[1][i]) + mu(lean, b->road[0][i]);
            if(normalize(v, v)) vcopy(b->tilt_up, v);
            if(b->height < 0) {
                int32_t vn;
                move_by(b, b->height, b->road[1]);
                dist = vlen(b->disp);
                b->height = 0;
                vn = dotu(b->road[1], b->vel);
                if(vn < 0) for(i = 0; i < 3; i++) b->vel[i] -= mu(vn, b->road[1][i]);
            }
        } else {
            vcopy(b->tilt_up, b->ground_up);
        }
        for(i = 0; i < 3; i++) v[i] = sh2z_u30_mul(b->tilt_up[i] - b->road[1][i], share) + b->road[1][i];
        if(vlen(v) > k2->tube.min_unit && normalize(v, v)) vcopy(b->tilt_up, v);
    } else {
        vcopy(b->tilt_up, b->ground_up);
    }
    vcopy(b->gravity_up, b->road[1]);
    if(radius < dist) radial_clamp(b, dist, radius);
    tube_band(b, r);
}

/* Outside a cylinder: the ground is the circle of radius half_left (tapered
 * to the line in a join); up points away from the axis. Beyond the radius
 * plus tube.cyl_reach the body has left the surface. */
static void shape_cylinder(sh2z_vd_body_t *b, const sh2z_vd_track2_t *k2, const sh2z_vd_rules_t *r) {
    const int32_t radius = b->half_left;
    int32_t share, cyl_r, dist = vlen(b->disp);
    const int side = join_at(b, k2, &share);
    int i;
    cyl_r = side ? mu(radius, share) : radius;
    b->height = dist - cyl_r;
    if(dist > k2->tube.min_dist) {
        int32_t u[3];
        if(normalize(u, b->disp)) vcopy(b->ground_up, u);
    }
    if(b->flags & SH2Z_VD_OFF_ROAD) {
        if(radius + k2->tube.cyl_reach < dist) { airborne_up(b, r); return; }
        b->flags &= ~SH2Z_VD_OFF_ROAD;
    } else if(radius + k2->tube.cyl_reach < dist) {
        leave_surface(b, r);
        return;
    }
    if(side) {
        const int32_t rest = ONE30 - share, rest2 = sh2z_u30_mul(rest, rest);
        int32_t v[3], plane;
        for(i = 0; i < 3; i++) v[i] = sh2z_u30_mul(b->ground_up[i] - b->road[1][i], share) + b->road[1][i];
        if(vlen(v) > k2->tube.min_unit && normalize(v, v)) vcopy(b->tilt_up, v);
        for(i = 0; i < 3; i++) {
            const int32_t base = sh2z_u30_mul(b->road[1][i], rest2);
            b->gravity_up[i] = sh2z_u30_mul(b->ground_up[i] - base, share) + base;
        }
        plane = mu(k2->tube.cyl_lift, share) + cyl_r + dotu(b->road[1], b->disp);
        if(plane < b->height) {
            b->height = plane;
            vcopy(b->ground_up, b->road[1]);
            if(b->height < 0) {
                move_by(b, b->height, b->road[1]);
                dist = vlen(b->disp);
                b->height = 0;
            }
        }
    } else {
        vcopy(b->gravity_up, b->ground_up);
        vcopy(b->tilt_up, b->ground_up);
    }
    if(dist < cyl_r && dist > 0) radial_clamp(b, dist, cyl_r);
    tube_band(b, r);
}
#endif

static void road_contact(sh2z_vd_body_t *b, const sh2z_vd_track_t *k, const sh2z_vd_rules_t *r, int shape) {
    (void)k;
    switch(shape) {
    case SH2Z_VD_SHAPE_WALLED: shape_road(b, r, r->air_walled); break;
    case SH2Z_VD_SHAPE_TUNNEL: shape_road(b, r, r->air_tunnel); break;
    case SH2Z_VD_SHAPE_GAP: shape_gap(b, r); break;
    case SH2Z_VD_SHAPE_EDGELESS: shape_edgeless(b, r); break;
#if SH2Z_VD_TUBES
    /* k is the base of a sh2z_vd_track2_t (sh2z_vd_step2). */
    case SH2Z_VD_SHAPE_PIPE: shape_pipe(b, (const sh2z_vd_track2_t *)(const void *)k, r); break;
    case SH2Z_VD_SHAPE_CYLINDER: shape_cylinder(b, (const sh2z_vd_track2_t *)(const void *)k, r); break;
    case SH2Z_VD_SHAPE_HALF_PIPE: shape_half_pipe(b, (const sh2z_vd_track2_t *)(const void *)k, r); break;
#endif
    default: shape_road(b, r, r->air_road); break;
    }
}

static void fall_off(sh2z_vd_body_t *b) {
    b->flags = (b->flags & ~SH2Z_VD_CONTROLLABLE) | SH2Z_VD_FALLING;
}

/* ---- Step pieces -------------------------------------------------------- */

static int32_t stick_q30(int s) {
    int32_t q = sh2z_divu_div64((int64_t)s * ONE30 + (s >= 0 ? 31 : -31), 63, NULL);
    if(q > ONE30) q = ONE30;
    if(q < -ONE30) q = -ONE30;
    return q;
}

static uint32_t zone_under(const sh2z_vd_body_t *b, const sh2z_vd_track_t *k) {
    const sh2z_vd_node_t *n = node_at(k, b->node);
    uint32_t i;
    if(n->zone_count == 0 || (b->flags & SH2Z_VD_AIRBORNE)) return 0;
    for(i = n->zone_first; i < (uint32_t)n->zone_first + n->zone_count; i++) {
        const sh2z_vd_zone_t *z = &k->zones[i];
        if(n->zones_by_side) {
            const int32_t side = dotu(b->road[2], b->disp) + (b->half_right - b->half_left) / 2;
            if(side < z->lo[0] || z->hi[0] < side) continue;
            if(b->t < z->t0 || z->t1 < b->t) continue;
        } else {
            if(b->t < z->t0 || z->t1 < b->t) continue;
            if(triple(b->disp, z->lo, b->road[0]) < 0) continue;
            if(triple(z->hi, b->disp, b->road[0]) < 0) continue;
        }
        return z->kind & SH2Z_VD_ZONE_MASK;
    }
    return 0;
}

/* Side force that fades with the side speed in its direction. */
static int32_t fade(int32_t into, int32_t force, int32_t rate, int32_t cut) {
    if(into < 0) return force;
    if(into < cut) return force - m16(rate, into);
    return 0;
}

/* acc +=/-= f * left, with or without its component along gravity_up. */
static void push_side(sh2z_vd_body_t *b, int32_t f, int dir, bool flat) {
    int32_t w[3], g;
    int i;
    for(i = 0; i < 3; i++) w[i] = mu(f, b->body[2][i]);
    g = flat ? dotu(b->gravity_up, w) : 0;
    for(i = 0; i < 3; i++) {
        const int32_t c = w[i] - mu(g, b->gravity_up[i]);
        b->acc[i] += dir > 0 ? c : -c;
    }
}

static void steer(sh2z_vd_body_t *b, const sh2z_vd_vehicle_t *v, const sh2z_vd_rules_t *r, uint16_t held,
                  int32_t stick, int32_t *change) {
    int32_t rate, old[3], z[3], x[3], dx[3];
    int i;
    *change = 0;
    if(!(stick > r->dead_zone || stick < -r->dead_zone)) return;
    if(b->spin_timer) rate = r->turn_spin;
    else if(held & (SH2Z_VD_BTN_LEAN_L | SH2Z_VD_BTN_LEAN_R))
        rate = b->lean_force ? sh2z_u30_mul(v->turn_free - v->turn_grip, r->lean_turn_mix) + v->turn_grip : v->turn_free;
    else if(b->flags & SH2Z_VD_GRIPPING)
        rate = (held & SH2Z_VD_BTN_ACCEL) ? v->turn_grip
                                          : sh2z_u30_mul(v->turn_free - v->turn_grip, r->grip_turn_mix) + v->turn_grip;
    else rate = v->turn_free;
    rate = sh2z_u30_mul(rate, stick);
    vcopy(old, b->body[0]);
    for(i = 0; i < 3; i++) b->body[0][i] -= sh2z_u30_mul(rate, b->body[2][i]);
    sh2z_u30_cross(z, b->body[1], b->body[0]);
    if(!normalize(z, z)) { vcopy(b->body[0], old); return; }
    vcopy(b->body[2], z);
    sh2z_u30_cross(x, z, b->body[1]);
    vcopy(b->body[0], x);
    for(i = 0; i < 3; i++) dx[i] = old[i] - x[i];
    *change = vd_round(vlen(dx), 14);                                /* Q30 -> Q16 */
}

static void damp(sh2z_vd_body_t *b, const sh2z_vd_rules_t *r, uint16_t held) {
    int32_t ks, kf, s, u, f;
    int i;
    if(b->spin_timer) ks = kf = r->damp_spin;
    else if(b->flags & SH2Z_VD_CRASHED) ks = kf = r->damp_crashed;
    else { ks = r->damp_side; kf = r->damp_fwd; }
    if(b->boost_timer == 0 && (b->flags & (SH2Z_VD_AIRBORNE | SH2Z_VD_ZONE_MASK)) == SH2Z_VD_ZONE_ROUGH) {
        ks = sh2z_u30_mul(ks, r->damp_rough);
        kf = sh2z_u30_mul(kf, r->damp_rough);
    }
    if(held & SH2Z_VD_BTN_BRAKE) {
        ks = sh2z_u30_mul(ks, r->damp_brake);
        kf = sh2z_u30_mul(kf, r->damp_brake);
    }
    s = mu(dotu(b->body[2], b->vel), ks);
    u = mu(dotu(b->body[1], b->vel), r->damp_up);
    f = mu(dotu(b->body[0], b->vel), kf);
    for(i = 0; i < 3; i++) b->vel[i] = mu(s, b->body[2][i]) + mu(u, b->body[1][i]) + mu(f, b->body[0][i]);
}

static void mines_and_ramps(sh2z_vd_body_t *b, const sh2z_vd_track_t *k, const sh2z_vd_rules_t *r) {
    const sh2z_vd_node_t *n = node_at(k, b->node);
    uint32_t i;
    int j;
    if(!(b->flags & SH2Z_VD_MINE_HIT)) {
        const int64_t rr = (int64_t)r->mine_radius * r->mine_radius;
        for(i = n->mine_first; i < (uint32_t)n->mine_first + n->mine_count; i++) {
            int32_t d[3];
            for(j = 0; j < 3; j++) d[j] = k->mines[i].pos[j] - b->pos[j];
            if(sumsq(d) < rr) {
                vadd_ku(b->acc, r->mine_kick, b->tilt_up);
                b->flags |= SH2Z_VD_MINE_HIT;
                damage(b, r, r->mine_damage);
                break;
            }
        }
    }
    if(!(b->flags & SH2Z_VD_LAUNCHED)) {
        for(i = n->ramp_first; i < (uint32_t)n->ramp_first + n->ramp_count; i++) {
            const sh2z_vd_ramp_t *p = &k->ramps[i];
            int32_t d[3], a;
            for(j = 0; j < 3; j++) d[j] = b->pos[j] - p->pos[j];
            a = dotu(p->axis[0], d);
            if(a < 0 || p->dim[2] < a) continue;
            a = dotu(p->axis[2], d);
            if(a < 0 || p->dim[0] < a) continue;
            a = dotu(p->axis[1], d);
            if(a < 0 || p->dim[1] < a) continue;
            b->flags |= SH2Z_VD_LAUNCHED;
            b->ramp_boost = mu(b->speed, r->ramp_gain) + r->ramp_base;
            break;
        }
    }
}

static void leans_and_attacks(sh2z_vd_body_t *b, const sh2z_vd_rules_t *r, uint16_t *held, uint16_t *pressed) {
    b->tap_l = (int16_t)(uint16_t)(uint32_t)(b->tap_l + 1);  /* 16-bit counters: they wrap */
    b->tap_r = (int16_t)(uint16_t)(uint32_t)(b->tap_r + 1);
    switch(*pressed & (SH2Z_VD_BTN_LEAN_L | SH2Z_VD_BTN_LEAN_R)) {
    case SH2Z_VD_BTN_LEAN_L:
        if(b->tap_l < r->double_tap && b->attack_angle == 0) {
            b->attack_angle = 0x80;
            b->attack_rate = 0x100;
            b->attack = (*held & SH2Z_VD_BTN_LEAN_R) ? SH2Z_VD_ATTACK_SPIN : SH2Z_VD_ATTACK_SIDE;
        }
        b->tap_l = 0;
        break;
    case SH2Z_VD_BTN_LEAN_R:
        if(b->tap_r < r->double_tap && b->attack_angle == 0) {
            b->attack_angle = 0xF80;
            b->attack_rate = -0x100;
            b->attack = (*held & SH2Z_VD_BTN_LEAN_L) ? SH2Z_VD_ATTACK_SPIN : SH2Z_VD_ATTACK_SIDE;
        }
        b->tap_r = 0;
        break;
    default:
        break;
    }
    if(b->attack != SH2Z_VD_ATTACK_NONE) {
        const int dir = b->attack_rate < 0 ? -1 : 1;
        const int32_t into = dir < 0 ? -b->bvel[0] : b->bvel[0];
        b->attack_dir = dir;
        if(b->attack == SH2Z_VD_ATTACK_SIDE) {
            b->lean_force = fade(into, r->side_force, r->side_fade, r->side_cut);
            push_side(b, b->lean_force, dir, true);
        } else {
            b->lean_force = fade(into, r->spin_force, r->spin_fade, r->spin_cut);
            push_side(b, b->lean_force, dir, false);
        }
        return;
    }
    switch(*held & (SH2Z_VD_BTN_LEAN_L | SH2Z_VD_BTN_LEAN_R)) {
    case 0:
    case SH2Z_VD_BTN_LEAN_L | SH2Z_VD_BTN_LEAN_R:
        b->lean_force = 0;
        break;
    default: {
        const int dir = (*held & SH2Z_VD_BTN_LEAN_L) ? 1 : -1;
        b->attack_dir = dir;
        b->lean_force = fade(dir > 0 ? b->bvel[0] : -b->bvel[0], r->lean_force, r->lean_fade, r->lean_cut);
        if((b->flags & SH2Z_VD_ZONE_MASK) == SH2Z_VD_ZONE_ICE) b->lean_force = mu(b->lean_force, r->lean_ice);
        push_side(b, b->lean_force, dir, true);
        break;
    }
    }
}

static void grip(sh2z_vd_body_t *b, const sh2z_vd_vehicle_t *v, const sh2z_vd_rules_t *r, uint16_t held,
                 uint16_t pressed) {
    int32_t side, sign;
    int i;
    if((b->flags & (SH2Z_VD_CRASHED | SH2Z_VD_SPINNING)) || (b->flags & SH2Z_VD_ZONE_MASK) == SH2Z_VD_ZONE_ICE) {
        b->slide_count = 0;
        return;
    }
    side = dotu(b->body[2], b->vel);
    sign = side < 0 ? -1 : 1;
    if(side < 0) side = -side;
    if(pressed & SH2Z_VD_BTN_ACCEL) {
        b->free_timer = r->grip_tap;
        b->tap_timer = r->tap_window;
    } else if(b->tap_timer) {
        b->tap_timer--;
    }
    if((b->flags & (SH2Z_VD_AIRBORNE | SH2Z_VD_DAMAGED)) || b->hit_timer || (held & (SH2Z_VD_BTN_LEAN_L | SH2Z_VD_BTN_LEAN_R))) {
        if(b->hit_timer) b->hit_timer--;
        if(b->free_timer) b->free_timer--;
        b->flags &= ~SH2Z_VD_GRIPPING;
        if(v->slide_limit < side) side = v->slide_limit;
    } else if(b->free_timer || !(held & SH2Z_VD_BTN_ACCEL)) {
        if(b->free_timer) b->free_timer--;
        b->flags |= SH2Z_VD_GRIPPING;
        if(v->grip_limit < side) side = v->grip_limit;
    } else if(b->flags & SH2Z_VD_GRIPPING) {
        if(v->grip_limit < side) {
            b->flags &= ~SH2Z_VD_GRIPPING;
            side = v->slide_limit;
        }
    } else if(v->slide_limit < side) {
        side = v->slide_limit;
    } else {
        b->flags |= SH2Z_VD_GRIPPING;
    }
    if(sign < 0) side = -side;
    for(i = 0; i < 3; i++) b->vel[i] -= mu(side, b->body[2][i]);
    if((b->bvel[0] < -r->slide_speed || b->bvel[0] > r->slide_speed) && !(b->flags & SH2Z_VD_AIRBORNE)) {
        int32_t share;
        b->slide_count++;
        share = vd_divq(b->slide_count, r->slide_ticks, 16);
        if(share >= ONE16) b->slide_boost = v->slide_max;
        else if(b->slide_boost < share) b->slide_boost = m16(v->slide_max, share);
    } else {
        b->slide_count = 0;
    }
}

#if SH2Z_VD_FORCE_FRAC == 16

static void engine(sh2z_vd_body_t *b, const sh2z_vd_vehicle_t *v, const sh2z_vd_rules_t *r, uint16_t held,
                   uint16_t released, int32_t change) {
    int32_t lo, hi, mul, fv, x;
    int i;
    if(b->slide_boost != 0) {
        lo = v->force_slide;
        hi = v->force_high;
        mul = m16(v->slide_mul, b->slide_boost) + ONE16;
        b->slide_boost -= v->slide_decay;
        if(b->slide_boost < 0) b->slide_boost = 0;
    } else {
        lo = v->force_low;
        hi = v->force_high;
        mul = ONE16;
    }
    if(b->boost_timer != 0) {
        if(b->flags & SH2Z_VD_PLATE_BOOST) {
            if(mul < v->plate_mul) mul = v->plate_mul;
        } else {
            if(mul < v->boost_mul) mul = v->boost_mul;
            b->energy -= v->boost_cost;
            if(b->energy < 0) {
                b->boost_timer = 1;
                b->energy = 0;
            }
        }
        if(--b->boost_timer == 0) b->flags &= ~SH2Z_VD_PLATE_BOOST;
    }
    lo = m16(lo, mul);
    hi = m16(hi, mul);
    x = ((b->flags & SH2Z_VD_ZONE_MASK) == SH2Z_VD_ZONE_ICE) ? b->speed : b->bvel[2];
    if(!(((held | released) & SH2Z_VD_BTN_ACCEL) && b->speed < v->accel_cutoff)) {
        b->accel_force = 0;
        return;
    }
    if(x <= 0) {
        x = 0;
        fv = lo;
    } else if(x < v->crossover) {
        const int32_t a = vd_divq(x, v->crossover, 16);
        const int32_t e = 2 * a - ONE16;
        int32_t bump = ONE16 - m16(e, e);
        bump = m16(bump, m16(lo < hi ? hi - lo : lo - hi, v->bump));
        fv = m16(hi - lo, a) + lo + bump;
    } else {
        fv = v->top_mix == 0 ? hi : m16(v->top_force - hi, v->top_mix) + hi;
    }
    fv -= mu(change, r->turn_cost) + mu(b->lean_force, r->lean_cost);
    if(fv < 0) {
        b->accel_force = 0;
        return;
    }
    if(fv <= b->accel_force) {
        b->accel_force = fv;
    } else {
        x = m16(x, v->slew_gain);
        x = x < ONE16 ? m16(v->slew_slope, x) + v->slew_base : ONE16;
        b->accel_force += m16(fv - b->accel_force, x);
    }
    {
        int32_t w[3], g;
        for(i = 0; i < 3; i++) w[i] = mu(b->accel_force, b->body[0][i]);
        g = dotu(b->gravity_up, w);
        for(i = 0; i < 3; i++) b->acc[i] += w[i] - mu(g, b->gravity_up[i]);
    }
}

#else /* SH2Z_VD_FORCE_FRAC > 16 (vehicle-dynamics-4) */

/* The engine force chain in QF = Q(SH2Z_VD_FORCE_FRAC): force_low, force_slide,
 * force_high, top_force and accel_force carry SH2Z_VD_FORCE_FRAC fraction bits,
 * the multipliers and shape factors stay Q16 and Q30. Products are int64 and
 * rounded once (ties up, saturating). The only conversion back to Q16 is the
 * acceleration the force adds along the body's forward axis. */
#define VD_FX (SH2Z_VD_FORCE_FRAC - 16)

/* QF x Q16 -> QF. */
static int32_t mf(int32_t a, int32_t b) { return vd_round((int64_t)a * b, 16); }
/* Q16 x Q30 -> QF. */
static int32_t muf(int32_t a, int32_t u) { return vd_round((int64_t)a * u, 30 - VD_FX); }

static void engine(sh2z_vd_body_t *b, const sh2z_vd_vehicle_t *v, const sh2z_vd_rules_t *r, uint16_t held,
                   uint16_t released, int32_t change) {
    int32_t lo, hi, mul, fv, x;
    int i;
    if(b->slide_boost != 0) {
        lo = v->force_slide;
        hi = v->force_high;
        mul = m16(v->slide_mul, b->slide_boost) + ONE16;
        b->slide_boost -= v->slide_decay;
        if(b->slide_boost < 0) b->slide_boost = 0;
    } else {
        lo = v->force_low;
        hi = v->force_high;
        mul = ONE16;
    }
    if(b->boost_timer != 0) {
        if(b->flags & SH2Z_VD_PLATE_BOOST) {
            if(mul < v->plate_mul) mul = v->plate_mul;
        } else {
            if(mul < v->boost_mul) mul = v->boost_mul;
            b->energy -= v->boost_cost;
            if(b->energy < 0) {
                b->boost_timer = 1;
                b->energy = 0;
            }
        }
        if(--b->boost_timer == 0) b->flags &= ~SH2Z_VD_PLATE_BOOST;
    }
    lo = mf(lo, mul);
    hi = mf(hi, mul);
    x = ((b->flags & SH2Z_VD_ZONE_MASK) == SH2Z_VD_ZONE_ICE) ? b->speed : b->bvel[2];
    if(!(((held | released) & SH2Z_VD_BTN_ACCEL) && b->speed < v->accel_cutoff)) {
        b->accel_force = 0;
        return;
    }
    if(x <= 0) {
        x = 0;
        fv = lo;
    } else if(x < v->crossover) {
        const int32_t a = vd_divq(x, v->crossover, 16);
        const int32_t e = 2 * a - ONE16;
        int32_t bump = ONE16 - m16(e, e);
        bump = mf(mf(lo < hi ? hi - lo : lo - hi, v->bump), bump);
        fv = mf(hi - lo, a) + lo + bump;
    } else {
        fv = v->top_mix == 0 ? hi : mf(v->top_force - hi, v->top_mix) + hi;
    }
    fv -= muf(change, r->turn_cost) + muf(b->lean_force, r->lean_cost);
    if(fv < 0) {
        b->accel_force = 0;
        return;
    }
    if(fv <= b->accel_force) {
        b->accel_force = fv;
    } else {
        x = m16(x, v->slew_gain);
        x = x < ONE16 ? m16(v->slew_slope, x) + v->slew_base : ONE16;
        b->accel_force += mf(fv - b->accel_force, x);
    }
    {
        int32_t w[3], g;
        for(i = 0; i < 3; i++) w[i] = vd_round((int64_t)b->accel_force * b->body[0][i], 30 + VD_FX);
        g = dotu(b->gravity_up, w);
        for(i = 0; i < 3; i++) b->acc[i] += w[i] - mu(g, b->gravity_up[i]);
    }
}

#endif

static void race_clock(sh2z_vd_body_t *b, const sh2z_vd_track_t *k, const sh2z_vd_vehicle_t *v,
                       const sh2z_vd_rules_t *r, int32_t prev) {
    int32_t diff;
    if((b->flags & (SH2Z_VD_CRASHED | SH2Z_VD_FINISHED | SH2Z_VD_FALLING)) || b->intro >= r->clock_tick) return;
    diff = prev - b->lap_dist;
    b->race_ticks++;
    b->race_time = sh2z_divu_div64((int64_t)b->race_ticks * r->clock_num, r->clock_den, NULL);
    if(diff > k->length / 2) {
        b->laps_dist += k->length;
        b->lap++;
        if(b->lap == b->laps_done + 1 && b->lap <= k->laps + 1) {
            int32_t at;
            b->laps_done = b->lap;
            b->flags |= SH2Z_VD_CAN_BOOST;
            at = b->race_time - sh2z_divu_div64((int64_t)r->clock_num * b->lap_dist,
                                                r->clock_den * (k->length - diff), NULL);
            if(b->lap >= 2 && b->lap - 2 < 8) b->lap_times[b->lap - 2] = at - b->done_time;
            b->done_time = at;
            if(b->lap == k->laps + 1) {
                b->race_time = at;
                b->flags |= SH2Z_VD_FINISHED;
                b->energy = v->max_energy;
            }
        }
    } else if(diff < -(k->length / 2)) {
        b->lap--;
        b->laps_dist -= k->length;
    }
    b->race_dist = b->laps_dist + b->lap_dist;
}

static void attack_motion(sh2z_vd_body_t *b, const sh2z_vd_rules_t *r) {
    if(b->attack_angle == 0) return;
    b->attack_angle += b->attack_rate;
    if(b->attack == SH2Z_VD_ATTACK_SIDE) {
        if(b->attack_angle == -0x80 || b->attack_angle == 0x1080) {
            b->free_timer = r->grip_tap;
            b->attack_angle = 0;
            b->attack = SH2Z_VD_ATTACK_NONE;
        }
    } else if(b->attack_rate < 0) {
        b->attack_rate += 2;
        if(b->attack_angle <= 0) {
            b->attack_angle += 0x1000;
            if(b->attack_rate > -50) {
                b->attack_angle = 0;
                b->attack = SH2Z_VD_ATTACK_NONE;
            }
        }
    } else {
        b->attack_rate -= 2;
        if(b->attack_angle >= 0x1000) {
            b->attack_angle -= 0x1000;
            if(b->attack_rate < 50) {
                b->attack_angle = 0;
                b->attack = SH2Z_VD_ATTACK_NONE;
            }
        }
    }
}

#if !SH2Z_VD_TUBES
/* ---- Line-space view ---------------------------------------------------- */

int32_t sh2z_vd_body_lateral(const sh2z_vd_body_t *b) { return dotu(b->road[2], b->disp); }

void sh2z_vd_line_view(const sh2z_vd_body_t *b, const sh2z_vd_track_t *k, sh2z_vd_line_view_t *out) {
    int32_t d = b->lap_dist;
    while(d >= k->length) d -= k->length;
    while(d < 0) d += k->length;
    out->dist = vd_round(d, 4);                                      /* Q12 -> Q8 */
    out->lateral = -vd_round(dotu(b->road[2], b->disp), 8);          /* left Q16 -> right Q8 */
    out->speed = vd_round(dotu(b->road[0], b->vel), 8);
    out->lat_vel = -vd_round(dotu(b->road[2], b->vel), 8);
    out->lap = b->laps_done > 0 ? b->laps_done - 1 : 0;
}

void sh2z_vd_apply_kick(sh2z_vd_body_t *b, int32_t kick_lat, int32_t kick_speed) {
    int i;
    for(i = 0; i < 3; i++)
        b->vel[i] += mu(-kick_lat * 256, b->road[2][i]) + mu(kick_speed * 256, b->body[0][i]);
}
#endif

/* ---- The tick ----------------------------------------------------------- */

#if SH2Z_VD_TUBES
void sh2z_vd_step2(sh2z_vd_body_t *b, const sh2z_vd_track2_t *track2, const sh2z_vd_vehicle_t *v,
                   const sh2z_vd_rules_t *r, const sh2z_vd_input_t *in) {
    const sh2z_vd_track_t *const k = &track2->base;
#else
void sh2z_vd_step(sh2z_vd_body_t *b, const sh2z_vd_track_t *k, const sh2z_vd_vehicle_t *v,
                  const sh2z_vd_rules_t *r, const sh2z_vd_input_t *in) {
#endif
    uint16_t held, pressed, released;
    int32_t stick, change, saved[3][3], before[3];
    int i;

    /* Intro and per-tick resets. */
    if(b->intro != 0) {
        b->intro--;
        if(b->intro == r->control_tick) b->flags |= SH2Z_VD_CONTROLLABLE;
    }
    if(!(b->flags & SH2Z_VD_ACTIVE)) return;
    b->flags &= ~(SH2Z_VD_DAMAGED | SH2Z_VD_RECOIL);
    b->acc[0] = b->acc[1] = b->acc[2] = 0;
    VD_CONTACT(b, v, r)

    /* 1. Pad. */
    if(!(b->flags & SH2Z_VD_CONTROLLABLE)) {
        held = pressed = released = 0;
        b->tilt_in = 0;
        stick = vd_sat((int64_t)b->spin_steer * 16384);              /* Q16 -> Q30 */
        b->spin_steer = mu(b->spin_steer, INT32_C(1063004406));        /* x0.99 */
    } else {
        held = in->held;
        pressed = in->pressed;
        released = in->released;
        b->tilt_in = stick_q30(in->stick_y);
        stick = stick_q30(in->stick_x);
    }
    for(i = 0; i < 3; i++) {
        saved[0][i] = b->body[0][i]; saved[1][i] = b->body[1][i]; saved[2][i] = b->body[2][i];
        before[i] = b->pos[i] + b->vel[i];
    }

    /* 2. Zone under the vehicle (from the last tick's place). */
    b->flags = (b->flags & ~SH2Z_VD_ZONE_MASK) | zone_under(b, k);

    /* 3. Steering. 4. Damping. */
    steer(b, v, r, held, stick, &change);
    damp(b, r, held);

    /* 5. Boost. */
    if((pressed & SH2Z_VD_BTN_BOOST) && b->boost_timer == 0 && b->energy != 0 && (b->flags & SH2Z_VD_CAN_BOOST)) {
        b->boost_timer = r->boost_ticks;
    } else if((b->flags & (SH2Z_VD_CRASHED | SH2Z_VD_ZONE_MASK)) == SH2Z_VD_ZONE_BOOST_PLATE) {
        held |= SH2Z_VD_BTN_ACCEL;
        b->flags |= SH2Z_VD_PLATE_BOOST;
        b->boost_timer = r->boost_ticks;
    }

    /* 6. Mines and ramps. 7. Gravity, launch impulse, hover spring. */
    mines_and_ramps(b, k, r);
    for(i = 0; i < 3; i++) b->acc[i] -= mu(r->gravity, b->gravity_up[i]);
    if(b->flags & SH2Z_VD_LAUNCHED) {
        vadd_ku(b->acc, b->ramp_boost, b->tilt_up);
        b->ramp_boost = mu(b->ramp_boost, r->ramp_decay);
    }
    b->spring = 0;
    if(!(b->flags & SH2Z_VD_OFF_ROAD)) {
        int32_t gap = b->hover - b->height;
        if(gap > 0) {
            gap -= dotu(b->ground_up, b->vel);
            b->spring = mu(gap, r->hover_spring);
            vadd_ku(b->acc, b->spring, b->ground_up);
            if(b->ramp_boost < r->ramp_release) b->flags &= ~(SH2Z_VD_LAUNCHED | SH2Z_VD_MINE_HIT);
        }
    }

    /* 8. Leans and attacks. 9-10. Grip and slide turbo. 11. Engine. */
    leans_and_attacks(b, r, &held, &pressed);
    grip(b, v, r, held, pressed);
    engine(b, v, r, held, released, change);

    /* 12. Integrate. */
    for(i = 0; i < 3; i++) b->vel[i] += b->acc[i];
    b->speed = vlen(b->vel);
    if(b->flags & SH2Z_VD_FALLING) {
        b->vel[0] = mu(b->vel[0], r->fall_damp);
        b->vel[2] = mu(b->vel[2], r->fall_damp);
        for(i = 0; i < 3; i++) {
            b->pos[i] += b->vel[i];
            b->disp[i] += b->vel[i];
        }
        shape_gap(b, r);
        if(b->pos[1] < r->fall_floor) b->flags = (b->flags & ~SH2Z_VD_ACTIVE) | SH2Z_VD_CRASHED;
    } else {
        int32_t target[3];
        if(b->speed > r->speed_cap) {
            const int32_t s = vd_divq(r->speed_cap, b->speed, 30);
            b->speed = r->speed_cap;
            for(i = 0; i < 3; i++) b->vel[i] = mu(b->vel[i], s);
        }
        for(i = 0; i < 3; i++) target[i] = b->pos[i] + b->vel[i];
        /* 13. Closest point on the spline, road frame and widths. */
        if(sh2z_vd_locate(b, k, target, 100, (int64_t)1 << 32) == 0) {
            const sh2z_vd_node_t *n = node_at(k, b->node), *m = node_at(k, b->node + 1u);
            const int32_t prev = b->lap_dist;
            sh2z_basis30_t f;
            b->lap_dist = sh2z_vd_lap_distance(b, k);
            b->half_left = mu(m->half_left - n->half_left, b->prop) + n->half_left;
            b->half_right = mu(m->half_right - n->half_right, b->prop) + n->half_right;
            vcopy(f.x, b->fwd);
            for(i = 0; i < 3; i++) f.y[i] = sh2z_u30_mul(b->prop, m->up[i] - n->up[i]) + n->up[i];
            if(sh2z_basis30_orthonormalize_forward(&f) == 0) {
                vcopy(b->road[0], f.x); vcopy(b->road[1], f.y); vcopy(b->road[2], f.z);
            }
            /* 14. Race clock and laps. */
            race_clock(b, k, v, r, prev);
            /* 15. Road shape. */
            road_contact(b, k, r, n->shape);
            if(b->flags & SH2Z_VD_OFF_ROAD) {
                if(b->pos[0] < -r->world_edge || b->pos[0] > r->world_edge || b->pos[2] < -r->world_edge ||
                   b->pos[2] > r->world_edge ||
                   (b->vel[1] < 0 && b->disp[1] < -b->half_left &&
                    (int64_t)b->disp[0] * b->vel[0] + (int64_t)b->disp[1] * b->vel[1] + (int64_t)b->disp[2] * b->vel[2] >= 0))
                    fall_off(b);
            }
        } else {
            for(i = 0; i < 3; i++) b->pos[i] = before[i];
            fall_off(b);
        }
    }

    /* 16. Up vector, body-frame velocity, recharge, attack motion, spin-out, hover. */
    {
        sh2z_basis30_t f;
        vcopy(f.x, b->body[0]);
        for(i = 0; i < 3; i++) f.y[i] = b->body[1][i] + sh2z_u30_mul(r->tilt_follow, b->tilt_up[i]);
        if(sh2z_basis30_orthonormalize_up(&f) == 0) {
            vcopy(b->body[0], f.x); vcopy(b->body[1], f.y); vcopy(b->body[2], f.z);
        } else {
            for(i = 0; i < 3; i++) {
                b->body[0][i] = saved[0][i]; b->body[1][i] = saved[1][i]; b->body[2][i] = saved[2][i];
            }
        }
    }
    b->bvel[0] = dotu(b->body[2], b->vel);
    b->bvel[1] = dotu(b->body[1], b->vel);
    b->bvel[2] = dotu(b->body[0], b->vel);
    if((b->flags & (SH2Z_VD_CRASHED | SH2Z_VD_AIRBORNE | SH2Z_VD_SPINNING | SH2Z_VD_ZONE_MASK)) == SH2Z_VD_ZONE_RECHARGE) {
        b->energy += v->recharge;
        if(v->max_energy < b->energy) b->energy = v->max_energy;
    }
    attack_motion(b, r);
    if(b->spin_timer != 0 && ++b->spin_timer > r->spin_ticks) {
        b->spin_timer = 0;
        b->flags = (b->flags & ~SH2Z_VD_SPINNING) | SH2Z_VD_CRASHED;
        b->hover = r->hover_crashed;
    }
    if(!(b->flags & (SH2Z_VD_CRASHED | SH2Z_VD_HOVER_SET))) {
        b->hover += mu(r->hover_top - b->hover, r->hover_rise_gain);
        if(b->hover > r->hover_top) {
            b->hover = r->hover_top;
            b->flags |= SH2Z_VD_HOVER_SET;
        }
    } else if(b->hover == v->hover_base && in->held != 0 && b->intro < r->hover_tick) {
        b->flags &= ~SH2Z_VD_HOVER_SET;
    }
}
