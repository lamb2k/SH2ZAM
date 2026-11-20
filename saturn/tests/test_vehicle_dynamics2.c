/* Host tests for vehicle-dynamics-2 (sh2z_vd_step2) on a synthetic stadium:
 * road spans give vehicle-dynamics-1's bits, a body keeps contact on any side
 * of a pipe and of a cylinder, the radial clamp, leaving a cylinder, a drive
 * through a pipe that joins open road at both ends, and a golden state hash.
 * Each property has a negative control (vehicle-dynamics-1's road rule, or
 * the tube without its joins).
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

#define PI 3.14159265358979323846

static unsigned checks;
static void check(bool ok, const char *what) {
    ++checks;
    if(!ok) {
        fprintf(stderr, "vehicle-dynamics-2 check %u failed: %s\n", checks, what);
        exit(EXIT_FAILURE);
    }
}

static int32_t q16(double x) { return (int32_t)lround(x * 65536.0); }
static int32_t q30(double x) { return (int32_t)lround(x * 1073741824.0); }
static int32_t q12(double x) { return (int32_t)lround(x * 4096.0); }
static double f16(int32_t v) { return v / 65536.0; }
static double f30(int32_t v) { return v / 1073741824.0; }

/* A stadium: two straights along x joined by half circles; 16 nodes. */
#define N 16
#define RADIUS 300.0
static sh2z_vd_node_t nodes[N];
static sh2z_vd_join_t joins[N];
static sh2z_vd_track2_t track2;
static sh2z_vd_track_t *const track = &track2.base;
static sh2z_vd_vehicle_t veh;
static sh2z_vd_rules_t rules;

static void point(int i, double p[3]) {
    const double half = 6000.0, r = 3000.0;
    if(i < 4) { p[0] = -half + i * 3000.0; p[2] = -r; }
    else if(i < 8) { double a = -PI / 2 + (i - 3) * PI / 5; p[0] = half + r * cos(a); p[2] = r * sin(a); }
    else if(i < 12) { p[0] = half - (i - 8) * 3000.0; p[2] = r; }
    else { double a = PI / 2 + (i - 11) * PI / 5; p[0] = -half + r * cos(a); p[2] = r * sin(a); }
    p[1] = 0.0;
}

/* Every span WALLED except those listed in tube[] (shape per span, 0 = none). */
static void build_track(const uint8_t *tube) {
    double total = 0.0;
    int i, k;
    memset(nodes, 0, sizeof(nodes));
    memset(&track2, 0, sizeof(track2));
    for(i = 0; i < N; i++) {
        double p[3];
        point(i, p);
        for(k = 0; k < 3; k++) nodes[i].pos[k] = q16(p[k]);
        nodes[i].up[1] = q30(1.0);
        nodes[i].tension = q30(0.5);
        nodes[i].half_left = nodes[i].half_right = q16(RADIUS);
        nodes[i].shape = tube && tube[i] ? tube[i] : SH2Z_VD_SHAPE_WALLED;
        nodes[i].zones_by_side = 1;
        joins[i] = (sh2z_vd_join_t){0, q30(1.0), q30(1.0)};
    }
    track->nodes = nodes;
    track->count = N;
    track->laps = 3;
    for(i = 0; i < N; i++) {
        double len = 0.0, prev[3] = {0, 0, 0}, marks[3] = {0, 0, 0};
        int s;
        for(s = 0; s <= 256; s++) {
            int32_t pos[3];
            sh2z_vd_spline(track, (uint32_t)i, (int32_t)((int64_t)s * (1 << 30) / 256), pos, NULL);
            if(s) len += sqrt(pow(f16(pos[0]) - prev[0], 2) + pow(f16(pos[1]) - prev[1], 2) + pow(f16(pos[2]) - prev[2], 2));
            prev[0] = f16(pos[0]); prev[1] = f16(pos[1]); prev[2] = f16(pos[2]);
            if(s == 64 || s == 128 || s == 192) {
                marks[s / 64 - 1] = len;
                for(k = 0; k < 3; k++) nodes[i].mark_pos[s / 64 - 1][k] = pos[k];
            }
        }
        nodes[i].length = q12(len);
        nodes[i].start = q12(total);
        for(k = 0; k < 3; k++) nodes[i].mark_len[k] = q12(marks[k]);
        total += len;
    }
    track->length = q12(total);
    track2.joins = NULL;
    track2.tube = (sh2z_vd_tube_rules_t){q16(1000.0), q16(23.0), q16(200.0), q16(200.0), q16(0.01), q30(0.01)};
}

static void build_vehicle(void) {
    veh = (sh2z_vd_vehicle_t){
        .force_low = q16(2.0), .force_slide = q16(1.7), .force_high = q16(0.1), .crossover = q16(33.0),
        .boost_mul = q16(2.0), .plate_mul = q16(2.0), .slide_max = q16(0.065), .slide_decay = q16(0.005),
        .slew_base = q16(1.0), .turn_free = q30(0.052), .turn_grip = q30(0.028), .top_mix = 0,
        .accel_cutoff = q16(92.0), .grip_limit = q16(1.25), .slide_limit = q16(0.38), .wall_bounce = q16(1.7),
        .slide_mul = q16(1.0), .bump = q16(0.25), .slew_gain = q16(0.015), .slew_slope = 0,
        .max_energy = q16(178.0), .boost_cost = q16(0.267), .recharge = q16(1.424), .hover_base = q16(5.0),
        .top_force = q16(0.198)};
    rules = (sh2z_vd_rules_t){
        .dead_zone = q30(0.1), .turn_spin = q30(0.2), .lean_turn_mix = q30(0.4), .grip_turn_mix = q30(0.125),
        .damp_side = q30(0.996), .damp_up = q30(0.94), .damp_fwd = q30(0.997), .damp_spin = q30(0.997),
        .damp_crashed = q30(0.983), .damp_rough = q30(0.987), .damp_brake = q30(0.987),
        .mine_radius = q16(30), .mine_kick = q16(15), .mine_damage = q16(12.5), .ramp_gain = q30(0.3),
        .ramp_base = q16(0.5), .ramp_decay = q30(0.8), .ramp_release = q16(0.5), .gravity = q16(1.0),
        .hover_spring = q30(0.2), .hover_top = q16(13), .hover_rise_gain = q30(0.05), .hover_crashed = q16(10),
        .double_tap = 15, .side_force = q16(5), .side_fade = q16(0.8333333), .side_cut = q16(6),
        .spin_force = q16(0.5), .spin_fade = q16(0.25), .spin_cut = q16(2), .lean_force = q16(1),
        .lean_fade = q16(0.125), .lean_cut = q16(8), .lean_ice = q30(0.2), .grip_tap = 30, .tap_window = 27,
        .contact_grip = 60, .slide_speed = q16(8), .slide_ticks = 100, .turn_cost = q30(0.5),
        .lean_cost = q30(0.16), .speed_cap = q16(138.9), .boost_ticks = 100, .air_road = q16(40),
        .air_walled = q16(145), .air_tunnel = q16(210), .ground_band = q16(15), .blend_band = q16(30),
        .edge_widen = q16(57.5), .edge_low = q16(-100), .edge_damage = q16(2), .tilt_follow = q30(0.19),
        .tilt_input = q30(0.15), .wall_damage = q30(0.7), .wall_min = q16(0.01), .wall_keep = q30(0.998),
        .spin_bounce = q16(2), .spin_steer = q30(0.2), .spin_ticks = 150, .fall_damp = q30(0.94),
        .fall_floor = q16(-2750), .world_edge = q16(20000), .control_tick = 40, .clock_tick = 40,
        .hover_tick = 250, .clock_num = 50, .clock_den = 3};
}

/* A body at node 1 (a straight span), t = 0.05, at height h above the
 * surface on the side given by the unit vector dir (in the road frame:
 * 0 left, 1 up; sign selects the side), facing along the line. For a pipe
 * the surface is at radius RADIUS inside, for a cylinder outside, for a road
 * at the line. */
enum { ON_ROAD, ON_PIPE, ON_CYLINDER };
static sh2z_vd_body_t start_body(int surface, int axis, int sign, double h, double t) {
    sh2z_vd_body_t b;
    sh2z_basis30_t f;
    double off;
    int k;
    memset(&b, 0, sizeof(b));
    b.flags = SH2Z_VD_ACTIVE | SH2Z_VD_GRIPPING | SH2Z_VD_HOVER_SET | SH2Z_VD_CONTROLLABLE | SH2Z_VD_CAN_BOOST;
    b.node = 1;
    b.t = q30(t);
    sh2z_vd_spline(track, 1, b.t, b.spos, b.fwd);
    b.fwd_mag = (int32_t)lround(65536.0 * sqrt(pow(f16(b.fwd[0]), 2) + pow(f16(b.fwd[1]), 2) + pow(f16(b.fwd[2]), 2)));
    for(k = 0; k < 3; k++) { f.x[k] = b.fwd[k]; f.y[k] = nodes[1].up[k]; }
    check(sh2z_basis30_orthonormalize_forward(&f) == 0, "start basis");
    off = surface == ON_PIPE ? RADIUS - h : surface == ON_CYLINDER ? RADIUS + h : h;
    for(k = 0; k < 3; k++) {
        const int32_t d = axis == 0 ? f.z[k] : f.y[k];
        const int32_t up = surface == ON_PIPE ? -sign * d : sign * d;
        b.road[0][k] = f.x[k]; b.road[1][k] = f.y[k]; b.road[2][k] = f.z[k];
        b.disp[k] = q16(off * sign * f30(d));
        b.pos[k] = b.ground[k] = b.spos[k] + b.disp[k];
        b.body[0][k] = f.x[k];
        b.body[1][k] = b.gravity_up[k] = b.tilt_up[k] = b.ground_up[k] = up;
    }
    sh2z_u30_cross(b.body[2], b.body[1], b.body[0]);
    b.lap_dist = sh2z_vd_lap_distance(&b, track);
    b.half_left = b.half_right = q16(RADIUS);
    b.energy = veh.max_energy;
    b.hover = q16(13.0);
    b.wall_bounce = veh.wall_bounce;
    b.lap = 1;
    b.laps_done = 1;
    b.height = q16(h);
    return b;
}

static sh2z_vd_input_t pad(uint16_t held, uint16_t prev, int sx) {
    sh2z_vd_input_t in = {held, (uint16_t)(held & ~prev), (uint16_t)(prev & ~held), (int8_t)sx, 0};
    return in;
}

static uint64_t fnv(uint64_t h, const sh2z_vd_body_t *b) {
    const int32_t *w = (const int32_t *)(const void *)b;
    size_t i;
    for(i = 0; i < sizeof(*b) / 4; i++) {
        uint32_t v = (uint32_t)w[i];
        int k;
        for(k = 24; k >= 0; k -= 8) h = (h ^ ((v >> k) & 0xFFu)) * UINT64_C(0x100000001B3);
    }
    return h;
}

static double dist_from_line(const sh2z_vd_body_t *b) {
    return sqrt(pow(f16(b->disp[0]), 2) + pow(f16(b->disp[1]), 2) + pow(f16(b->disp[2]), 2));
}

/* Cosine between the body's ground up and the radial direction. */
static double radial_cos(const sh2z_vd_body_t *b) {
    const double d = dist_from_line(b);
    return (f30(b->ground_up[0]) * f16(b->disp[0]) + f30(b->ground_up[1]) * f16(b->disp[1]) +
            f30(b->ground_up[2]) * f16(b->disp[2])) / d;
}

typedef void (*stepper)(sh2z_vd_body_t *, const sh2z_vd_input_t *);
static void step_vd2(sh2z_vd_body_t *b, const sh2z_vd_input_t *in) { sh2z_vd_step2(b, &track2, &veh, &rules, in); }
static void step_vd1(sh2z_vd_body_t *b, const sh2z_vd_input_t *in) { sh2z_vd_step(b, track, &veh, &rules, in); }

/* Settled hover height on an open road (the spring against gravity). */
static double settled = 0.0;

/* Drive straight with ACCEL for n ticks (all on the first straight). Returns
 * the worst |height - settled| after 60 ticks and the worst radial cosine
 * departure from want (+1 cylinder, -1 pipe). */
static void drive(sh2z_vd_body_t *b, stepper step, int n, double want, double *height_err, double *cos_err) {
    sh2z_vd_input_t in = pad(SH2Z_VD_BTN_ACCEL, 0, 0);
    int i;
    *height_err = *cos_err = 0.0;
    for(i = 0; i < n; i++) {
        step(b, &in);
        in = pad(SH2Z_VD_BTN_ACCEL, SH2Z_VD_BTN_ACCEL, 0);
        if(i >= 60) {
            const double he = fabs(f16(b->height) - settled), ce = fabs(radial_cos(b) - want);
            if(he > *height_err) *height_err = he;
            if(ce > *cos_err) *cos_err = ce;
        }
    }
}

static void test_road_identity(void) {
    sh2z_vd_body_t a, b;
    sh2z_vd_input_t in;
    uint16_t prev = 0;
    int i, same = 1;
    build_track(NULL);
    a = b = start_body(ON_ROAD, 1, 1, 13.0, 0.05);
    for(i = 0; i < 2400; i++) {
        uint16_t held = (uint16_t)(SH2Z_VD_BTN_ACCEL | ((i % 300) == 250 ? SH2Z_VD_BTN_BOOST : 0) |
                                   ((i / 70) % 5 == 3 ? SH2Z_VD_BTN_LEAN_R : 0));
        in = pad(held, prev, ((i / 40) % 3 - 1) * 50);
        prev = held;
        sh2z_vd_step(&a, track, &veh, &rules, &in);
        sh2z_vd_step2(&b, &track2, &veh, &rules, &in);
        if(memcmp(&a, &b, sizeof(a)) != 0) same = 0;
    }
    check(same, "road spans: sh2z_vd_step2 gives sh2z_vd_step's state bits every tick (2400 ticks, walls)");
    check(a.flags & SH2Z_VD_RECOIL || a.laps_done >= 1, "the identity run exercised the road rules");
    a = start_body(ON_ROAD, 1, 1, 13.0, 0.05);
    {
        sh2z_vd_input_t acc = pad(SH2Z_VD_BTN_ACCEL, 0, 0);
        for(i = 0; i < 240; i++) {
            sh2z_vd_step(&a, track, &veh, &rules, &acc);
            acc = pad(SH2Z_VD_BTN_ACCEL, SH2Z_VD_BTN_ACCEL, 0);
        }
    }
    settled = f16(a.height);
    printf("settled hover height on the road: %.4f\n", settled);
    check(settled > 1.0 && settled < 13.0, "the road hover settles below the hover target");
}

static void test_pipe(void) {
    static const uint8_t pipe[N] = {SH2Z_VD_SHAPE_PIPE, SH2Z_VD_SHAPE_PIPE, SH2Z_VD_SHAPE_PIPE, SH2Z_VD_SHAPE_PIPE};
    sh2z_vd_body_t b;
    double he, ce;
    int side;
    build_track(pipe);
    /* Bottom, left wall, ceiling: gravity is toward the surface wherever the body is. */
    for(side = 0; side < 3; side++) {
        const int axis = side == 1 ? 0 : 1, sign = side == 2 ? 1 : -1;
        b = start_body(ON_PIPE, axis, sign, 13.0, 0.05);
        drive(&b, step_vd2, 240, -1.0, &he, &ce);
        if(he > 1.0 || ce > 1e-4) fprintf(stderr, "pipe side %d: height err %f cos err %g\n", side, he, ce);
        check(he < 1.0, "pipe: the body settles at the road hover height on every side");
        check(ce < 1e-4, "pipe: up points to the line");
        check(!(b.flags & (SH2Z_VD_AIRBORNE | SH2Z_VD_OFF_ROAD | SH2Z_VD_FALLING)), "pipe: in contact");
        check(dist_from_line(&b) <= RADIUS + 0.01, "pipe: inside the radius");
    }
    /* Control: vehicle-dynamics-1 treats the pipe as a road, so on the left wall up stays the road's. */
    b = start_body(ON_PIPE, 0, -1, 13.0, 0.05);
    drive(&b, step_vd1, 240, -1.0, &he, &ce);
    check(ce > 0.5, "control: the road rule does not hold a body on the pipe wall");
    /* Radial clamp: a body past the radius, moving outward, is put back on it and loses that speed. */
    b = start_body(ON_PIPE, 0, -1, -40.0, 0.05);
    b.vel[2] = q16(-6.0);
    {
        sh2z_vd_input_t in = pad(0, 0, 0);
        sh2z_vd_step2(&b, &track2, &veh, &rules, &in);
    }
    check(fabs(dist_from_line(&b) - RADIUS) < 0.01, "pipe: a body past the radius is put back on it");
    check(f30(b.ground_up[0]) * f16(b.vel[0]) + f30(b.ground_up[1]) * f16(b.vel[1]) + f30(b.ground_up[2]) * f16(b.vel[2]) > -1e-3,
          "pipe: no speed left into the surface");
    check(f16(b.height) == 0.0, "pipe: clamped to height 0");
}

static void test_cylinder(void) {
    static const uint8_t cyl[N] = {SH2Z_VD_SHAPE_CYLINDER, SH2Z_VD_SHAPE_CYLINDER, SH2Z_VD_SHAPE_CYLINDER,
                                   SH2Z_VD_SHAPE_CYLINDER};
    sh2z_vd_body_t b;
    double he, ce;
    int side;
    build_track(cyl);
    for(side = 0; side < 3; side++) {
        const int axis = side == 1 ? 0 : 1, sign = side == 2 ? -1 : 1;
        b = start_body(ON_CYLINDER, axis, sign, 13.0, 0.05);
        drive(&b, step_vd2, 240, 1.0, &he, &ce);
        if(he > 1.0 || ce > 1e-4) fprintf(stderr, "cylinder side %d: height err %f cos err %g\n", side, he, ce);
        check(he < 1.0, "cylinder: the body settles at the road hover height on every side");
        check(ce < 1e-4, "cylinder: up points away from the line");
        check(!(b.flags & (SH2Z_VD_AIRBORNE | SH2Z_VD_OFF_ROAD | SH2Z_VD_FALLING)), "cylinder: in contact");
    }
    b = start_body(ON_CYLINDER, 0, 1, 13.0, 0.05);
    drive(&b, step_vd1, 240, 1.0, &he, &ce);
    check(ce > 0.5 || (b.flags & (SH2Z_VD_OFF_ROAD | SH2Z_VD_FALLING)), "control: the road rule does not hold a body on the cylinder's side");
    /* Leaving: beyond radius + reach (200) the body is off the surface; inside it, it is not. */
    {
        sh2z_vd_input_t in = pad(0, 0, 0);
        b = start_body(ON_CYLINDER, 1, 1, 230.0, 0.05);
        b.vel[1] = q16(2.0);
        sh2z_vd_step2(&b, &track2, &veh, &rules, &in);
        check((b.flags & SH2Z_VD_OFF_ROAD) && (b.flags & SH2Z_VD_AIRBORNE), "cylinder: off the surface beyond radius + 200");
        b = start_body(ON_CYLINDER, 1, 1, 150.0, 0.05);
        sh2z_vd_step2(&b, &track2, &veh, &rules, &in);
        check(!(b.flags & SH2Z_VD_OFF_ROAD) && (b.flags & SH2Z_VD_AIRBORNE), "cylinder: airborne but on it within radius + 200");
    }
    /* Clamp: below the radius the body is put back on it. */
    {
        sh2z_vd_input_t in = pad(0, 0, 0);
        b = start_body(ON_CYLINDER, 1, 1, -30.0, 0.05);
        sh2z_vd_step2(&b, &track2, &veh, &rules, &in);
        check(fabs(dist_from_line(&b) - RADIUS) < 0.01, "cylinder: a body below the radius is put back on it");
    }
}

/* Spans 1 and 2 are a pipe that joins open road (span 0 before, span 3 after);
 * each join covers 30% of its span. Driving along the bottom from the road,
 * the floor drops into the tube and climbs out again with the body in contact. */
static int drive_through(int with_joins, double *worst_height, uint64_t *hash) {
    static const uint8_t pipe[N] = {0, SH2Z_VD_SHAPE_PIPE, SH2Z_VD_SHAPE_PIPE};
    sh2z_vd_body_t b;
    sh2z_vd_input_t in = pad(SH2Z_VD_BTN_ACCEL, 0, 0);
    uint64_t h = UINT64_C(0xCBF29CE484222325);
    int i, through = 0;
    build_track(pipe);
    nodes[0].shape = SH2Z_VD_SHAPE_ROAD;
    nodes[3].shape = SH2Z_VD_SHAPE_ROAD;
    joins[1] = (sh2z_vd_join_t){q30(0.3), q30(1.0), q30(0.3)};
    joins[2] = (sh2z_vd_join_t){0, q30(0.7), q30(0.3)};
    track2.joins = with_joins ? joins : NULL;
    b = start_body(ON_ROAD, 1, 1, 13.0, 0.05);
    b.node = 0;
    b.t = q30(0.3);
    sh2z_vd_spline(track, 0, b.t, b.spos, b.fwd);
    { int k; for(k = 0; k < 3; k++) b.pos[k] = b.ground[k] = b.spos[k] + b.disp[k]; }
    *worst_height = 0.0;
    for(i = 0; i < 400; i++) {
        sh2z_vd_step2(&b, &track2, &veh, &rules, &in);
        in = pad(SH2Z_VD_BTN_ACCEL, SH2Z_VD_BTN_ACCEL, 0);
        h = fnv(h, &b);
        if(i > 30 && f16(b.height) > *worst_height) *worst_height = f16(b.height);
        if(b.node == 3) through = 1;
        if(b.flags & SH2Z_VD_FALLING) return -1;
    }
    *hash = h;
    return through;
}

static void test_joins(void) {
    double worst, worst_open;
    uint64_t h, h_open;
    const int r_join = drive_through(1, &worst, &h), r_open = drive_through(0, &worst_open, &h_open);
    printf("join drive: worst height %.3f with joins, %.3f without; hash %016llx\n", worst, worst_open,
           (unsigned long long)h);
    check(r_join == 1, "join: the body drives from road into the pipe and out again without falling");
    check(worst < 0.5 * worst_open, "join: the tapered floor keeps the body far closer to the surface");
    check(r_open != 1 || worst_open > 100.0, "control: without its joins the body drops into the whole pipe");
    check(h == UINT64_C(0xd2659d3902905498), "golden state hash of the join drive (fixed by the contract)");
}

int main(void) {
    build_vehicle();
    test_road_identity();
    test_pipe();
    test_cylinder();
    test_joins();
    printf("vehicle-dynamics-2: %u checks passed\n", checks);
    return 0;
}
