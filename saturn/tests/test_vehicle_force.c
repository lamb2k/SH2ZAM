/* Host tests for vehicle-dynamics-4: the engine force carried with more fraction bits
 * (SH2Z_VD_FORCE_FRAC). This file builds the vehicle model a second time, as the
 * contact entry points do, with the force scale at 24 under private names, and
 * calls its engine force function (engine) on prepared states against an
 * independent long-double oracle. Checks: the stored constant is the delivered
 * force (no Q16 rounding), the per-case error bound over randomised states
 * (boost, slide turbo, curve, bump, turn and lean costs, slew), saturation without
 * wrap, the single conversion to Q16 acceleration, and negative controls (a Q16
 * constant, a wrong shift, truncation, a Q16-only accumulator) that the same bounds
 * must reject. The default build's objects are compared byte for byte against the
 * previous library by the objcmp step of the capture.
 *
 * Copyright (c) 2026 SH2ZAM Contributors. MIT License.
 * Inspired by SH4ZAM by Falco Girgis and SH4ZAM Contributors.
 */
#include "sh2zam/vehicle_dynamics_experimental.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* The wide copy of the model: SH2Z_VD_FORCE_FRAC 24, every public name private. */
static void vdw_step(sh2z_vd_body_t *b, const sh2z_vd_track_t *track, const sh2z_vd_vehicle_t *v,
                     const sh2z_vd_rules_t *r, const sh2z_vd_input_t *in);
static int vdw_locate(sh2z_vd_body_t *b, const sh2z_vd_track_t *track, const int32_t target[3], int iterations,
                      int64_t tolerance_q32);
static int32_t vdw_lap_distance(sh2z_vd_body_t *b, const sh2z_vd_track_t *track);
static __attribute__((unused)) int32_t vdw_body_lateral(const sh2z_vd_body_t *b);
static __attribute__((unused)) void vdw_line_view(const sh2z_vd_body_t *b, const sh2z_vd_track_t *track,
                                                  sh2z_vd_line_view_t *out);
static __attribute__((unused)) void vdw_apply_kick(sh2z_vd_body_t *b, int32_t kick_lat, int32_t kick_speed);
static void vdw_spline(const sh2z_vd_track_t *track, uint32_t n, int32_t t, int32_t pos[3], int32_t fwd[3]);
#define sh2z_vd_step vdw_step
#define sh2z_vd_locate vdw_locate
#define sh2z_vd_lap_distance vdw_lap_distance
#define sh2z_vd_body_lateral vdw_body_lateral
#define sh2z_vd_line_view vdw_line_view
#define sh2z_vd_apply_kick vdw_apply_kick
#define sh2z_vd_spline vdw_spline
#undef SH2Z_VD_FORCE_FRAC
#define SH2Z_VD_FORCE_FRAC 24
#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "../src/sh2zam_vehicle_dynamics.c"
#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif
#undef sh2z_vd_step
#undef sh2z_vd_locate
#undef sh2z_vd_lap_distance
#undef sh2z_vd_body_lateral
#undef sh2z_vd_line_view
#undef sh2z_vd_apply_kick
#undef sh2z_vd_spline

#define FX 24
#define BOUND_STEPS 5.0   /* accel_force: at most ten roundings of half a QF step each in one call */

static unsigned checks;
static void check(bool ok, const char *what) {
    ++checks;
    if(!ok) {
        fprintf(stderr, "vehicle-force check %u failed: %s\n", checks, what);
        exit(EXIT_FAILURE);
    }
}

static uint64_t rng_state = 0x243F6A8885A308D3ull;
static uint32_t rnd(void) {
    rng_state = rng_state * 6364136223846793005ull + 1442695040888963407ull;
    return (uint32_t)(rng_state >> 33);
}
static double urand(double lo, double hi) { return lo + (hi - lo) * ((double)rnd() / 2147483648.0); }

static int32_t qf(double x) { return (int32_t)lround(x * (double)(1 << FX)); }   /* ties away from zero */
static int32_t q16(double x) { return (int32_t)lround(x * 65536.0); }
static int32_t q30(double x) { return (int32_t)lround(x * 1073741824.0); }

static void unit_vec(double a, double b, int32_t out[3]) {
    const double x = cos(a) * cos(b), y = sin(b), z = sin(a) * cos(b);
    out[0] = q30(x); out[1] = q30(y); out[2] = q30(z);
}

typedef struct {
    sh2z_vd_body_t b;
    sh2z_vd_vehicle_t v;
    sh2z_vd_rules_t r;
    uint16_t held, released;
    int32_t change;
} Case;

typedef struct { double accel_force; double acc[3]; bool zeroed; } Expect;

/* The oracle: the engine chain on the exact real values of the integer inputs, long double. mode: 0 the contract; 1 the force
 * constants and accel_force rounded to Q16 (a Q16 carrier); 2 the carrier truncated to four fewer bits;
 * 3 the acceleration taken at one bit less (a wrong shift). */
static Expect oracle(const Case *c, int mode) {
    const double F = (double)(1 << FX);
    const sh2z_vd_body_t *b = &c->b;
    const sh2z_vd_vehicle_t *v = &c->v;
    const sh2z_vd_rules_t *r = &c->r;
    Expect e = {0, {0, 0, 0}, false};
    long double lo, hi, mul = 1.0L, fv, x;
    int32_t slide = b->slide_boost, timer = b->boost_timer;
    long double flo = v->force_low, fhi = v->force_high, fsl = v->force_slide;
    if(mode == 1) { flo = roundl(flo / 256.0L) * 256.0L; fhi = roundl(fhi / 256.0L) * 256.0L; fsl = roundl(fsl / 256.0L) * 256.0L; }
    if(slide != 0) { lo = fsl; hi = fhi; mul = (long double)(m16(v->slide_mul, slide) + 65536) / 65536.0L; }
    else { lo = flo; hi = fhi; }
    if(timer != 0) {
        if(b->flags & SH2Z_VD_PLATE_BOOST) { if(mul < (long double)v->plate_mul / 65536.0L) mul = (long double)v->plate_mul / 65536.0L; }
        else if(mul < (long double)v->boost_mul / 65536.0L) mul = (long double)v->boost_mul / 65536.0L;
    }
    lo *= mul; hi *= mul;
    x = ((b->flags & SH2Z_VD_ZONE_MASK) == SH2Z_VD_ZONE_ICE) ? b->speed : b->bvel[2];
    if(!(((c->held | c->released) & SH2Z_VD_BTN_ACCEL) && b->speed < v->accel_cutoff)) { e.zeroed = true; return e; }
    if(x <= 0) { x = 0; fv = lo; }
    else if(x < v->crossover) {
        const int32_t a = vd_divq((int32_t)x, v->crossover, 16);
        const int32_t ei = 2 * a - 65536;
        const long double shape = (long double)(65536 - m16(ei, ei)) / 65536.0L;   /* the Q16 shape factor, as the Q16 model rounds it */
        const long double bumpc = fabsl(hi - lo) * ((long double)v->bump / 65536.0L);
        fv = (hi - lo) * (long double)a / 65536.0L + lo + bumpc * shape;
    } else fv = v->top_mix == 0 ? hi : (v->top_force - hi) * ((long double)v->top_mix / 65536.0L) + hi;
    fv -= ((long double)c->change * r->turn_cost / 1073741824.0L + (long double)b->lean_force * r->lean_cost / 1073741824.0L) * 256.0L;
    if(fv < 0) { e.zeroed = true; return e; }
    {
        long double af = b->accel_force;
        if(fv <= af) af = fv;
        else {
            int32_t xs = m16((int32_t)x, v->slew_gain);
            xs = xs < 65536 ? m16(v->slew_slope, xs) + v->slew_base : 65536;
            af += (fv - af) * (long double)xs / 65536.0L;
        }
        if(mode == 1) af = roundl(af / 256.0L) * 256.0L;
        if(mode == 2) af = floorl(af / 16.0L) * 16.0L;   /* a carrier of four fewer bits, truncating */
        e.accel_force = (double)af;
        {
            long double w[3], g = 0;
            const int sh = mode == 3 ? 37 : 38;
            int i;
            for(i = 0; i < 3; i++) w[i] = af * b->body[0][i] / powl(2.0L, sh);
            for(i = 0; i < 3; i++) g += (long double)b->gravity_up[i] * (long double)roundl(w[i]);
            g = roundl(g / 1073741824.0L);
            for(i = 0; i < 3; i++) e.acc[i] = (double)(roundl(w[i]) - (long double)b->gravity_up[i] * g / 1073741824.0L);
        }
    }
    (void)F;
    return e;
}

static void random_case(Case *c) {
    memset(c, 0, sizeof(*c));
    c->v = (sh2z_vd_vehicle_t){
        .force_low = qf(urand(1.0, 2.5)), .force_slide = qf(urand(1.0, 2.2)), .force_high = qf(urand(0.08, 0.14)),
        .crossover = q16(urand(25, 40)), .boost_mul = q16(urand(1.5, 3.0)), .plate_mul = q16(urand(1.5, 3.0)),
        .slide_max = q16(0.065), .slide_decay = q16(0.005), .slew_base = q16(urand(0.1, 1.0)), .top_mix = rnd() & 1 ? 0 : q16(urand(0.0, 1.0)),
        .accel_cutoff = q16(92.0), .bump = q16(urand(0.0, 0.5)), .slew_gain = q16(urand(0.005, 0.03)), .slew_slope = q16(urand(0.0, 0.9)),
        .slide_mul = q16(urand(0.5, 2.0)), .max_energy = q16(178), .boost_cost = q16(0.267), .top_force = qf(urand(0.1, 0.3))};
    c->r.turn_cost = q30(0.5);
    c->r.lean_cost = q30(0.16);
    c->held = SH2Z_VD_BTN_ACCEL;
    c->change = q16(urand(-0.4, 0.4));
    c->b.flags = SH2Z_VD_ACTIVE | SH2Z_VD_GRIPPING | SH2Z_VD_CONTROLLABLE;
    if(rnd() % 5 == 0) c->b.flags |= SH2Z_VD_PLATE_BOOST;
    c->b.slide_boost = (rnd() % 3 == 0) ? q16(urand(0.001, 0.065)) : 0;
    c->b.boost_timer = (rnd() % 4 == 0) ? 1 + (int32_t)(rnd() % 90) : 0;
    c->b.energy = q16(100);
    c->b.speed = q16(urand(0, 60));
    c->b.bvel[2] = (rnd() % 7 == 0) ? -q16(urand(0, 3)) : q16(urand(0, 60));
    c->b.lean_force = q16(urand(0, 1.5));
    c->b.accel_force = (rnd() % 3 == 0) ? 0 : qf(urand(0.0, 2.5));
    unit_vec(urand(-3.1, 3.1), urand(-0.2, 0.2), c->b.body[0]);
    unit_vec(urand(-3.1, 3.1), urand(1.2, 1.5), c->b.gravity_up);
}

static double force_err_steps(const Case *c, const Expect *e, int32_t got) { (void)c; return fabs((double)got - e->accel_force); }

int main(void) {
    unsigned i, cases = 0, worst_case = 0;
    double worst = 0, worst_acc = 0;
    /* 1. The constant is the delivered force: slew 1 (above the crossover with the slew pinned), no costs, mul 1. */
    {
        Case c;
        const double exact16 = 6571.245;   /* a force_high of 0.1002701... in Q16 steps */
        double err24, err16;
        random_case(&c);
        c.v.force_high = qf(exact16 / 65536.0);
        c.v.crossover = q16(1.0);
        c.v.top_mix = 0;
        c.b.slide_boost = 0; c.b.boost_timer = 0; c.b.flags &= ~SH2Z_VD_PLATE_BOOST;
        c.b.bvel[2] = q16(30.0); c.b.speed = q16(30.0);
        c.v.slew_gain = q16(10.0);          /* the slew share saturates at 1: accel_force = fv */
        c.change = 0; c.b.lean_force = 0; c.b.accel_force = 0;
        engine(&c.b, &c.v, &c.r, c.held, c.released, c.change);
        err24 = fabs((double)c.b.accel_force / 256.0 - exact16);
        err16 = fabs(round(exact16) - exact16);
        check(c.b.accel_force == c.v.force_high, "above the crossover with the slew saturated the force is the stored constant, exactly");
        check(err24 <= 0.5 / 256.0 + 1e-12, "the Q24 constant is within half a QF step (0.002 Q16 steps) of the exact force");
        check(err16 > 0.2, "control: the Q16 constant of the same force is 0.245 Q16 steps off");
        check(err16 / (err24 + 1e-12) > 50.0, "the Q24 constant is far closer than the Q16 one");
    }
    /* 2. Randomised states against the oracle. */
    for(i = 0; i < 200000; i++) {
        Case c;
        Expect e;
        int j;
        random_case(&c);
        e = oracle(&c, 0);
        engine(&c.b, &c.v, &c.r, c.held, c.released, c.change);
        cases++;
        if(e.zeroed) {
            check(c.b.accel_force == 0, "zeroed force");
            for(j = 0; j < 3; j++) check(c.b.acc[j] == 0, "a zeroed force adds no acceleration");
            continue;
        }
        {
            const double d = force_err_steps(&c, &e, c.b.accel_force);
            double da = 0;
            for(j = 0; j < 3; j++) { const double t = fabs((double)c.b.acc[j] - e.acc[j]); if(t > da) da = t; }
            if(d > worst) { worst = d; worst_case = i; }
            if(da > worst_acc) worst_acc = da;
        }
    }
    printf("vehicle-force: %u cases, worst accel_force error %.3f QF steps (case %u), worst acceleration error %.3f Q16 steps\n", cases, worst,
           worst_case, worst_acc);
    check(worst <= BOUND_STEPS, "accel_force within the documented bound (5 QF steps) of the exact chain");
    check(worst_acc <= 2.0, "acceleration within 2 Q16 steps of the exact chain");
    /* 3. Negative controls: each planted model must break the same bounds on this case set. */
    {
        int mode;
        for(mode = 1; mode <= 3; mode++) {
            double w = 0, wa = 0;
            unsigned n;
            rng_state = 0x243F6A8885A308D3ull;
            for(n = 0; n < 20000; n++) {
                Case c;
                Expect e;
                int j;
                random_case(&c);
                e = oracle(&c, mode);
                engine(&c.b, &c.v, &c.r, c.held, c.released, c.change);
                if(e.zeroed) continue;
                {
                    const double d = fabs((double)c.b.accel_force - e.accel_force);
                    double da = 0;
                    for(j = 0; j < 3; j++) { const double t = fabs((double)c.b.acc[j] - e.acc[j]); if(t > da) da = t; }
                    if(d > w) w = d;
                    if(da > wa) wa = da;
                }
            }
            check(w > BOUND_STEPS || wa > 2.0, mode == 1 ? "control: a Q16 carrier breaks the bound" : mode == 2 ? "control: truncating the carrier breaks the bound" : "control: a wrong force-application shift breaks the bound");
        }
    }
    /* 4. Saturation: a force beyond the Q(16+8) range saturates, it does not wrap. */
    {
        Case c;
        random_case(&c);
        c.v.force_high = INT32_MAX; c.v.force_low = INT32_MAX; c.v.boost_mul = q16(3.0); c.v.top_mix = 0;
        c.b.slide_boost = 0; c.b.boost_timer = 5; c.b.flags &= ~SH2Z_VD_PLATE_BOOST;
        c.b.bvel[2] = q16(50.0); c.b.speed = q16(50.0); c.change = 0; c.b.lean_force = 0; c.b.accel_force = 0; c.v.slew_gain = q16(10.0);
        engine(&c.b, &c.v, &c.r, c.held, c.released, c.change);
        check(c.b.accel_force == INT32_MAX, "an overflowing force saturates at INT32_MAX");
        random_case(&c);
        c.v.force_high = INT32_MIN; c.v.force_low = INT32_MIN; c.v.top_mix = 0; c.b.slide_boost = 0; c.b.boost_timer = 0;
        c.b.bvel[2] = q16(50.0); c.b.speed = q16(50.0); c.change = 0; c.b.lean_force = 0; c.b.accel_force = 0;
        engine(&c.b, &c.v, &c.r, c.held, c.released, c.change);
        check(c.b.accel_force == 0, "a negative force is dropped (accel_force 0), as in the Q16 model");
    }
    printf("vehicle-force: %u checks passed\n", checks);
    return 0;
}
