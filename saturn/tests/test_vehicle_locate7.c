/* Host tests for vehicle-dynamics-7: the secant damping (SH2Z_VD_LOCATE_DAMP) and the node flip-flop stop
 * (SH2Z_VD_LOCATE_FLIP) of the closest-point search.
 * The model is built a second time under private names, as the contact entry points do, with
 * SH2Z_VD_LOCATE_DAMP 8 and SH2Z_VD_LOCATE_FLIP 3 (B_DAMP and B_FLIP override them for the controls) and the
 * vehicle-dynamics-6 stop off. The linked library (both off, the 99-pass search of every earlier tag) is the oracle.
 * Over randomised bodies on a stadium and on rough roads, and targets from level with the line to 2000 units off it:
 *  - a search in which neither change acted equals the library's, the whole body, word for word;
 *  - every damped step is not longer than the step it replaces and has its sign (the hook sees both);
 *  - a damped step is taken only after DAMP passes of the call;
 *  - a search the flip stop ends ends once, after at least 5 passes, on a pass of the parity the 99th pass has, and
 *    keeps the target as the body position;
 *  - where the library's search converged (residual within 1 unit) within 700 units of the line and a change acted
 *    on it, the copy converged too and ends within B_BOUND units (distance between the spline points) of the
 *    library's end state; where the library did not converge, the numbers are printed (there is no right end state);
 *  - the set holds damped searches, flip-ended searches and searches that neither change touched.
 * The controls (tests/mutate_vehicle_locate7.sh) plant a step twice as long, a reversed secant, a slope test the wrong
 * way, a damping from the second pass, a flip stop without the parity, without the return test, ending a search as
 * off the line, and both hooks removed, and build with DAMP 1 and FLIP 1 and 2; each must fail.
 * The default build's objects are compared byte for byte against the previous library by the objcmp step of the capture.
 *
 * Copyright (c) 2026 SH2ZAM Contributors. MIT License.
 * Inspired by SH4ZAM by Falco Girgis and SH4ZAM Contributors.
 */
#include "sh2zam/vehicle_dynamics_experimental.h"
#include "sh2zam/unit30_experimental.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef VD_SRC
#define VD_SRC "../src/sh2zam_vehicle_dynamics.c"
#endif

#ifndef B_DAMP
#define B_DAMP 8
#endif
#ifndef B_FLIP
#define B_FLIP 3
#endif
#ifndef B_BOUND
#define B_BOUND 1.0
#endif
static unsigned b_passes, b_damp_hits, b_flip_hits, b_damp_bad, b_damp_early;
static void b_note_pass(const int32_t d[3]);
static void b_damp(int32_t old_dt, int32_t new_dt);

static void vdb_step(sh2z_vd_body_t *, const sh2z_vd_track_t *, const sh2z_vd_vehicle_t *, const sh2z_vd_rules_t *,
                     const sh2z_vd_input_t *);
static int vdb_locate(sh2z_vd_body_t *, const sh2z_vd_track_t *, const int32_t[3], int, int64_t);
static int32_t vdb_lap_distance(sh2z_vd_body_t *, const sh2z_vd_track_t *);
static __attribute__((unused)) int32_t vdb_body_lateral(const sh2z_vd_body_t *);
static __attribute__((unused)) void vdb_line_view(const sh2z_vd_body_t *, const sh2z_vd_track_t *, sh2z_vd_line_view_t *);
static __attribute__((unused)) void vdb_apply_kick(sh2z_vd_body_t *, int32_t, int32_t);
static void vdb_spline(const sh2z_vd_track_t *, uint32_t, int32_t, int32_t[3], int32_t[3]);
#define sh2z_vd_step vdb_step
#define sh2z_vd_locate vdb_locate
#define sh2z_vd_lap_distance vdb_lap_distance
#define sh2z_vd_body_lateral vdb_body_lateral
#define sh2z_vd_line_view vdb_line_view
#define sh2z_vd_apply_kick vdb_apply_kick
#define sh2z_vd_spline vdb_spline
#undef SH2Z_VD_LOCATE_FAR
#define SH2Z_VD_LOCATE_FAR 0
#undef SH2Z_VD_LOCATE_DAMP
#define SH2Z_VD_LOCATE_DAMP B_DAMP
#undef SH2Z_VD_LOCATE_FLIP
#define SH2Z_VD_LOCATE_FLIP B_FLIP
#define SH2Z_VD_LOCATE_PASS(d) b_note_pass(d)
#define SH2Z_VD_LOCATE_DAMP_HIT(old_dt, new_dt) b_damp(old_dt, new_dt)
#define SH2Z_VD_LOCATE_FLIP_HIT() (b_flip_hits++)
#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include VD_SRC
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

static unsigned checks;
static void check(bool ok, const char *what) {
    ++checks;
    if(!ok) {
        fprintf(stderr, "vehicle-locate check %u failed: %s\n", checks, what);
        exit(EXIT_FAILURE);
    }
}

static int32_t q16(double x) { return (int32_t)lround(x * 65536.0); }
static int32_t q30(double x) { return (int32_t)lround(x * 1073741824.0); }
static int32_t q12(double x) { return (int32_t)lround(x * 4096.0); }
static double f16(int32_t v) { return v / 65536.0; }

/* The copy's per-pass bookkeeping: passes of the call, and the damped steps against the steps they replace. */
static void b_note_pass(const int32_t d[3]) {
    (void)d;
    b_passes++;
}
static void b_damp(int32_t old_dt, int32_t new_dt) {
    b_damp_hits++;
    if((new_dt < 0) != (old_dt < 0) || llabs((long long)new_dt) > llabs((long long)old_dt)) b_damp_bad++;
    if(b_passes <= (unsigned)B_DAMP) b_damp_early++;
}

/* A stadium: two straights along x joined by half circles; 16 nodes (the vehicle-dynamics tests' track). */
#define N 16
static sh2z_vd_node_t nodes[N];
static sh2z_vd_track_t track;

static double rough[3];   /* per-node scatter of the rough track (0: the stadium) */
static uint32_t rnd(void);
static double urand(double lo, double hi);
static void point(int i, double p[3]) {
    const double half = 6000.0, r = 3000.0, pi = 3.14159265358979323846;
    if(i < 4) { p[0] = -half + i * 3000.0; p[2] = -r; }
    else if(i < 8) { double a = -pi / 2 + (i - 3) * pi / 5; p[0] = half + r * cos(a); p[2] = r * sin(a); }
    else if(i < 12) { p[0] = half - (i - 8) * 3000.0; p[2] = r; }
    else { double a = pi / 2 + (i - 11) * pi / 5; p[0] = -half + r * cos(a); p[2] = r * sin(a); }
    p[1] = 0.0;
    if(rough[0] != 0.0) {
        p[0] += urand(-rough[0], rough[0]);
        p[1] += urand(-rough[1], rough[1]);
        p[2] += urand(-rough[2], rough[2]);
    }
}

static void build_track(void) {
    double total = 0.0;
    int i, k;
    memset(nodes, 0, sizeof(nodes));
    for(i = 0; i < N; i++) {
        double p[3];
        point(i, p);
        for(k = 0; k < 3; k++) nodes[i].pos[k] = q16(p[k]);
        nodes[i].up[1] = q30(1.0);
        nodes[i].tension = q30(0.5);
        nodes[i].half_left = nodes[i].half_right = q16(300.0);
        nodes[i].shape = SH2Z_VD_SHAPE_WALLED;
        nodes[i].zones_by_side = 1;
    }
    track.nodes = nodes;
    track.count = N;
    track.laps = 3;
    for(i = 0; i < N; i++) {
        double len = 0.0, prev[3] = {0, 0, 0};
        int s;
        for(s = 0; s <= 256; s++) {
            int32_t pos[3];
            sh2z_vd_spline(&track, (uint32_t)i, (int32_t)((int64_t)s * (1 << 30) / 256), pos, NULL);
            if(s) len += sqrt(pow(f16(pos[0]) - prev[0], 2) + pow(f16(pos[1]) - prev[1], 2) + pow(f16(pos[2]) - prev[2], 2));
            prev[0] = f16(pos[0]); prev[1] = f16(pos[1]); prev[2] = f16(pos[2]);
        }
        nodes[i].length = q12(len);
        nodes[i].start = q12(total);
        total += len;
    }
    track.length = q12(total);
}

/* A body on the line at node n, parameter t, with a search state consistent with its spline point. */
static sh2z_vd_body_t on_line(uint32_t n, int32_t t) {
    sh2z_vd_body_t b;
    int k;
    memset(&b, 0, sizeof(b));
    b.node = n;
    b.t = t;
    sh2z_vd_spline(&track, n, t, b.spos, b.fwd);
    b.fwd_mag = (int32_t)lround(65536.0 * sqrt(pow(f16(b.fwd[0]), 2) + pow(f16(b.fwd[1]), 2) + pow(f16(b.fwd[2]), 2)));
    for(k = 0; k < 3; k++) b.pos[k] = b.ground[k] = b.spos[k];
    return b;
}

static uint64_t rng_state = 0x243F6A8885A308D3ull;
static uint32_t rnd(void) {
    rng_state = rng_state * 6364136223846793005ull + 1442695040888963407ull;
    return (uint32_t)(rng_state >> 33);
}
static double urand(double lo, double hi) { return lo + (hi - lo) * ((double)rnd() / 2147483648.0); }

static double dist3(const int32_t a[3], const int32_t b[3]) {
    double s = 0.0;
    int k;
    for(k = 0; k < 3; k++) s += pow(f16(a[k]) - f16(b[k]), 2);
    return sqrt(s);
}

int main(void) {
    unsigned i, pass_no, cases = 0, same = 0, damped = 0, flipped = 0, both = 0, lib_off = 0, copy_off = 0;
    unsigned conv_n = 0, conv_lost = 0, conv_over = 0, conv_gained = 0, worse = 0, over1 = 0;
    double conv_max = 0.0, max_excess = 0.0, max_diff = 0.0;
    for(pass_no = 0; pass_no < 6; pass_no++) {
    /* pass 0 is the stadium; passes 1 to 5 are rough roads (nodes scattered by up to 1400 units, 450 vertically). */
    rough[0] = pass_no ? 1400.0 : 0.0; rough[1] = 450.0; rough[2] = 1400.0;
    build_track();
    for(i = 0; i < 20000; i++) {
        const uint32_t n = rnd() % N;
        const int32_t t = q30(urand(0.0, 0.99));
        sh2z_vd_body_t a = on_line(n, t), b = a;
        int32_t target[3];
        const double dist = (i % 3 == 0) ? urand(0.0, 60.0) : urand(0.0, 2000.0);
        double dir[3], len;
        int k, ra, rb;
        do {
            for(k = 0; k < 3; k++) dir[k] = urand(-1.0, 1.0);
            len = sqrt(dir[0] * dir[0] + dir[1] * dir[1] + dir[2] * dir[2]);
        } while(len < 0.2);
        for(k = 0; k < 3; k++) target[k] = a.spos[k] + q16(dir[k] / len * dist);
        b_passes = 0;
        b_damp_hits = b_flip_hits = 0;
        ra = sh2z_vd_locate(&a, &track, target, 100, (int64_t)1 << 32);
        rb = vdb_locate(&b, &track, target, 100, (int64_t)1 << 32);
        cases++;
        if(ra != rb) {
            if(ra < 0) lib_off++; else copy_off++;
            if(b_damp_hits == 0 && b_flip_hits == 0) check(false, "an untouched search has the library's return value");
            continue;
        }
        check(b_damp_bad == 0, "every damped step is not longer than the step it replaces and has its sign");
        check(b_damp_early == 0, "a damped step is taken only after DAMP passes");
        check(b_flip_hits <= 1, "the flip stop ends a call once");
        if(b_damp_hits == 0 && b_flip_hits == 0) {
            check(memcmp(&a, &b, sizeof(a)) == 0, "a search neither change acted on equals the library's, word for word");
            same++;
            continue;
        }
        damped += b_damp_hits > 0;
        flipped += b_flip_hits > 0;
        both += b_damp_hits > 0 && b_flip_hits > 0;
        if(b_flip_hits) {
            check(b_passes >= 5 && b_passes <= 99, "a flip-ended search ran 5 to 99 passes");
            check(((99u - (b_passes - 1u)) & 1u) == 0, "a flip-ended search ends on a pass of the parity the 99th pass has");
        }
        if(rb == 0) for(k = 0; k < 3; k++) check(b.pos[k] == target[k], "the body position is the target");
        check(b.node < N, "the end state has a valid node");
        {
            const double diff = dist3(a.spos, b.spos), lr = dist3(a.ground, target), cr = dist3(b.ground, target);
            if(diff > max_diff) max_diff = diff;
            if(diff > 1.0) over1++;
            if(cr > lr + 1.0 && dist <= 700.0) {
                worse++;
                if(cr - lr > max_excess) max_excess = cr - lr;
            }
            if(lr <= 1.0 && dist <= 700.0) {
                conv_n++;
                if(cr > 1.0) conv_lost++;
                if(diff > B_BOUND) conv_over++;
                if(diff > conv_max) conv_max = diff;
            } else if(cr <= 1.0) {
                conv_gained++;
            }
        }
    }
    }
    printf("vehicle-locate7: %u cases, %u untouched (equal to the library), %u damped, %u flip-ended (%u both)\n", cases, same, damped, flipped, both);
    printf("vehicle-locate7: the earlier search converged (residual within 1 unit, within 700 units of the line) in %u of the changed searches; the changed search did not converge in %u, ended more than %.1f unit from its end state in %u, longest %.3f units\n", conv_n, conv_lost, (double)B_BOUND, conv_over, conv_max);
    printf("vehicle-locate7: where the earlier search did not converge, the changed one converged in %u; within 700 units its residual was larger by over 1 unit in %u (the largest by %.1f units); end states more than 1 unit apart in %u (the longest %.1f units)\n", conv_gained, worse, max_excess, over1, max_diff);
    printf("vehicle-locate7: return value differs in %u cases (%u where only the library reports -1, %u where only the copy does)\n", lib_off + copy_off, lib_off, copy_off);
    check(copy_off <= 10, "the copy reports an off-line search where the library does not in at most 10 of the cases");
    check(conv_n >= 100, "the set holds changed searches that the library converged");
    check(conv_lost * 100 <= conv_n && conv_over * 100 <= conv_n, "where the library converged, the changed search converged within the bound of its end state in all but 1 in 100");
    check(conv_max <= 16.0, "and the exceptions end within 16 units of the library's end state");
    check(damped >= 500, "the set holds damped searches");
    check(flipped >= 100, "the set holds flip-ended searches");
    check(same >= 10000, "the set holds searches neither change touched");
    printf("vehicle-locate7: %u checks passed\n", checks);
    return EXIT_SUCCESS;
}
