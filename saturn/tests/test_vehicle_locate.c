/* Host tests for vehicle-dynamics-6: the stop for a search that circles (SH2Z_VD_LOCATE_FAR).
 * The model is built a second time under private names, as the contact entry points do, with
 * SH2Z_VD_LOCATE_FAR 128 and SH2Z_VD_LOCATE_FAR_RUN 8 (B_FAR and B_RUN override them for the
 * controls), and calls its search through the SH2Z_VD_LOCATE_PASS hook that reads every pass's
 * residual. The linked library (the stop off, the search of every earlier tag) is the reference.
 * Over randomised bodies and targets from level with the line to 700 units off it:
 *  - a search that the copy does not stop equals the library's, the whole body, word for word, and
 *    never had 8 passes in a row above 128 units (the stop is complete);
 *  - a search that it stops had its residual above 128 units for 8 passes in a row at that point
 *    (and the run was not shorter), stopped once, and keeps the target as the body position;
 *  - the set holds searches that overshot above 128 units for 1 to 7 passes and came back (the case
 *    the run length exists for, not stopped) and circling ones (stopped);
 *  - a build with FAR 3 and RUN 1 (the controls, compiled from this file with -DB_FAR=3 -DB_RUN=1)
 *    changes searches that converge, so the same comparison must fail.
 * The default build's objects are compared byte for byte against the previous library by the
 * objcmp step of the capture.
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

#ifndef B_FAR
#define B_FAR 128
#endif
#ifndef B_RUN
#define B_RUN 8
#endif
static unsigned b_hits, b_passes;
static int b_run, b_maxrun;
static int64_t b_far_sq;
static void b_note_pass(const int32_t d[3]);

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
#define SH2Z_VD_LOCATE_FAR B_FAR
#undef SH2Z_VD_LOCATE_FAR_RUN
#define SH2Z_VD_LOCATE_FAR_RUN B_RUN
#define SH2Z_VD_LOCATE_PASS(d) b_note_pass(d)
#define SH2Z_VD_LOCATE_FAR_HIT() (b_hits++)
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

/* The copy's per-pass bookkeeping: passes, the current and the longest run of residuals above 128 units. */
static void b_note_pass(const int32_t d[3]) {
    const int64_t sq = (int64_t)d[0] * d[0] + (int64_t)d[1] * d[1] + (int64_t)d[2] * d[2];
    b_passes++;
    b_run = sq > b_far_sq ? b_run + 1 : 0;
    if(b_run > b_maxrun) b_maxrun = b_run;
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

int main(void) {
    unsigned i, cases = 0, same = 0, recovered = 0, stopped = 0, long_runs = 0, pass_no, runs37 = 0;
    b_far_sq = ((int64_t)128 * 128) << 32;
    for(pass_no = 0; pass_no < 6; pass_no++) {
    /* pass 0 is the stadium; passes 1 to 5 are rough roads (nodes scattered by up to 1400 units, 450 vertically). */
    rough[0] = pass_no ? 1400.0 : 0.0; rough[1] = 450.0; rough[2] = 1400.0;
    build_track();
    for(i = 0; i < 20000; i++) {
        const uint32_t n = rnd() % N;
        const int32_t t = q30(urand(0.0, 0.99));
        sh2z_vd_body_t a = on_line(n, t), b = a;
        int32_t target[3];
        const double dist = (i % 3 == 0) ? urand(0.0, 60.0) : urand(0.0, 700.0);
        double dir[3], len;
        int k, ra, rb;
        do {
            for(k = 0; k < 3; k++) dir[k] = urand(-1.0, 1.0);
            len = sqrt(dir[0] * dir[0] + dir[1] * dir[1] + dir[2] * dir[2]);
        } while(len < 0.2);
        for(k = 0; k < 3; k++) target[k] = a.spos[k] + q16(dir[k] / len * dist);
        b_passes = 0;
        b_run = b_maxrun = 0;
        b_hits = 0;
        ra = sh2z_vd_locate(&a, &track, target, 100, (int64_t)1 << 32);
        rb = vdb_locate(&b, &track, target, 100, (int64_t)1 << 32);
        check(ra == rb, "the return value does not depend on the stop");
        cases++;
        if(b_hits == 0) {
            check(memcmp(&a, &b, sizeof(a)) == 0, "an unstopped search equals the library's, word for word");
            check(b_maxrun < 8, "the stop is complete: no search with 8 far passes in a row runs on");
            same++;
            if(b_maxrun > 0) recovered++;
            if(b_maxrun >= 3) runs37++;
        } else {
            check(b_hits == 1, "one stop per call");
            check(b_run == 8 && b_maxrun == 8, "a stopped search had 8 far passes in a row at the stop, and not more");
            check(b_passes >= 8 && b_passes <= 99, "a stopped search ran 8 to 99 passes");
            for(k = 0; k < 3; k++) check(b.pos[k] == target[k], "the body position is the target");
            check(b.node < N, "a stopped search keeps a valid node");
            stopped++;
            if(b_passes > 20) long_runs++;
        }
    }
    }
    printf("vehicle-locate: %u cases, %u equal to the library (%u overshot above 128 units and recovered), %u stopped (%u after more than 20 passes)\n",
           cases, same, recovered, stopped, long_runs);
    check(stopped >= 50, "the set holds circling searches");
    check(runs37 >= 20, "the set holds searches that stayed above 128 units for 3 to 7 passes in a row and came back");
    printf("vehicle-locate: %u checks passed\n", checks);
    return EXIT_SUCCESS;
}
