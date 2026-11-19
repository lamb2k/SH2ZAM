/* Host tests for vehicle-dynamics-1 on a synthetic oval: spline identities, the
 * closest-point search, launch, steering, grip, a signed wall response, surface
 * zones, lap counting and a golden state hash, each with a negative control.
 * MIT; SH2ZAM Contributors. */
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
        fprintf(stderr, "vehicle-dynamics check %u failed: %s\n", checks, what);
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
static sh2z_vd_node_t nodes[N];
static sh2z_vd_zone_t zones[2];
static sh2z_vd_track_t track;
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
    /* Arc lengths by fine sampling of the same spline. */
    track.nodes = nodes;
    track.zones = zones;
    track.count = N;
    track.laps = 3;
    for(i = 0; i < N; i++) {
        double len = 0.0, prev[3] = {0, 0, 0}, marks[3] = {0, 0, 0};
        int s;
        for(s = 0; s <= 256; s++) {
            int32_t pos[3];
            sh2z_vd_spline(&track, (uint32_t)i, (int32_t)((int64_t)s * (1 << 30) / 256), pos, NULL);
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
    track.length = q12(total);
    /* A boost plate in the middle of node 2 and a recharge strip on node 9. */
    zones[0] = (sh2z_vd_zone_t){SH2Z_VD_ZONE_BOOST_PLATE, q30(0.4), q30(0.6), {q16(-100), 0, 0}, {q16(100), 0, 0}};
    zones[1] = (sh2z_vd_zone_t){SH2Z_VD_ZONE_RECHARGE, q30(0.0), q30(1.0), {q16(-300), 0, 0}, {q16(300), 0, 0}};
    nodes[2].zone_first = 0; nodes[2].zone_count = 1;
    nodes[9].zone_first = 1; nodes[9].zone_count = 1;
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

/* A body on the line at node 1 (a straight span), facing along the line. */
static sh2z_vd_body_t start_body(void) {
    sh2z_vd_body_t b;
    sh2z_basis30_t f;
    int k;
    memset(&b, 0, sizeof(b));
    b.flags = SH2Z_VD_ACTIVE | SH2Z_VD_GRIPPING | SH2Z_VD_HOVER_SET | SH2Z_VD_CONTROLLABLE | SH2Z_VD_CAN_BOOST;
    b.node = 1;
    b.t = q30(0.05);
    sh2z_vd_spline(&track, 1, b.t, b.spos, b.fwd);
    b.fwd_mag = (int32_t)lround(65536.0 * sqrt(pow(f16(b.fwd[0]), 2) + pow(f16(b.fwd[1]), 2) + pow(f16(b.fwd[2]), 2)));
    for(k = 0; k < 3; k++) { b.pos[k] = b.ground[k] = b.spos[k]; f.x[k] = b.fwd[k]; f.y[k] = nodes[1].up[k]; }
    check(sh2z_basis30_orthonormalize_forward(&f) == 0, "start basis");
    for(k = 0; k < 3; k++) {
        b.body[0][k] = b.road[0][k] = f.x[k];
        b.body[1][k] = b.road[1][k] = b.gravity_up[k] = b.tilt_up[k] = b.ground_up[k] = f.y[k];
        b.body[2][k] = b.road[2][k] = f.z[k];
    }
    b.lap_dist = sh2z_vd_lap_distance(&b, &track);
    b.half_left = b.half_right = q16(300);
    b.energy = veh.max_energy;
    b.hover = q16(8.0);
    b.wall_bounce = veh.wall_bounce;
    b.lap = 1;
    b.laps_done = 1;
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

static void test_spline(void) {
    int32_t p[3], f[3];
    int k;
    sh2z_vd_spline(&track, 3, 0, p, f);
    for(k = 0; k < 3; k++) check(p[k] == nodes[3].pos[k], "spline passes through its node");
    for(k = 0; k < 3; k++) {
        const double want = 0.5 * (f16(nodes[4].pos[k]) - f16(nodes[2].pos[k]));
        check(fabs(f16(f[k]) - want) < 0.01, "tangent at t = 0 is tension x (next - previous)");
    }
    sh2z_vd_spline(&track, 3, q30(1.0) - 1, p, NULL);
    for(k = 0; k < 3; k++) check(abs(p[k] - nodes[4].pos[k]) < 64, "spline reaches the next node");
}

static void test_locate(void) {
    sh2z_vd_body_t b = start_body();
    int32_t target[3];
    int k;
    /* 1000 units ahead and 120 to the left on the first straight (forward +x, left -z). */
    for(k = 0; k < 3; k++) target[k] = b.pos[k];
    target[0] += q16(1000.0);
    target[2] -= q16(120.0);
    check(sh2z_vd_locate(&b, &track, target, 100, (int64_t)1 << 32) == 0, "locate succeeds");
    check(fabs(f16(sh2z_vd_body_lateral(&b)) - 120.0) < 0.5, "lateral offset found (positive left)");
    check(fabs(f16(b.spos[0]) - f16(target[0])) < 0.5, "closest point is level with the target");
    for(k = 0; k < 3; k++) target[k] = b.pos[k];
    target[0] += q16(9000.0);
    check(sh2z_vd_locate(&b, &track, target, 100, (int64_t)1 << 32) == -1, "control: a jump of 3 spans is off the line");
}

static void test_launch_and_balance(void) {
    sh2z_vd_body_t b = start_body();
    sh2z_vd_input_t in = pad(SH2Z_VD_BTN_ACCEL, 0, 0);
    double v10 = 0, v300 = 0, lat100 = 0;
    int i;
    for(i = 1; i <= 200; i++) {
        sh2z_vd_step(&b, &track, &veh, &rules, &in);
        in = pad(SH2Z_VD_BTN_ACCEL, SH2Z_VD_BTN_ACCEL, 0);
        if(i == 10) v10 = f16(b.speed);
        if(i == 100) lat100 = f16(sh2z_vd_body_lateral(&b));
    }
    v300 = f16(b.speed);
    check(v10 > 10.0 && v10 < v300, "launch: fast start, still rising");
    /* Balance: forward force ~ (1 - damping) x speed, 0.1 / 0.003 = 33, plus the plate on node 2. */
    if(!(v300 > 30.0 && v300 < 45.0)) fprintf(stderr, "v10 %f v200 %f\n", v10, v300);
    check(v300 > 30.0 && v300 < 45.0, "speed after 200 ticks near the force / damping balance");
    check(fabs(lat100) < 1.0, "straight line stays on the line along the straight span");
    {
        /* Control: no ACCEL, no speed. */
        sh2z_vd_body_t c = start_body();
        sh2z_vd_input_t none = pad(0, 0, 0);
        for(i = 0; i < 60; i++) sh2z_vd_step(&c, &track, &veh, &rules, &none);
        check(f16(c.speed) < 0.5, "control: coasting from rest stays at rest");
    }
}

static double yaw_rate(int stick, uint16_t extra) {
    sh2z_vd_body_t b = start_body();
    sh2z_vd_input_t in;
    double a0, a1;
    int i;
    uint16_t prev = 0;
    for(i = 0; i < 152; i++) {
        uint16_t held = (uint16_t)(SH2Z_VD_BTN_ACCEL | (i >= 150 ? extra : 0));
        in = pad(held, prev, i >= 150 ? stick : 0);
        prev = held;
        sh2z_vd_step(&b, &track, &veh, &rules, &in);
    }
    a0 = atan2(f30(b.body[0][2]), f30(b.body[0][0]));
    for(i = 0; i < 10; i++) {
        uint16_t held = (uint16_t)(SH2Z_VD_BTN_ACCEL | extra);
        in = pad(held, prev, stick);
        prev = held;
        sh2z_vd_step(&b, &track, &veh, &rules, &in);
    }
    a1 = atan2(f30(b.body[0][2]), f30(b.body[0][0]));
    return (a1 - a0) / 10.0 * 180.0 / PI;
}

static void test_steering(void) {
    const double r32 = yaw_rate(32, 0), r64 = yaw_rate(64, 0), r6 = yaw_rate(6, 0);
    printf("yaw rate, degrees per tick: stick 6 %.4f, 32 %.4f, 64 %.4f\n", r6, r32, r64);
    check(r32 > 0.0, "positive stick turns right (heading toward +z from +x)");
    check(r64 / r32 > 1.95 && r64 / r32 < 2.0, "yaw rate near linear in the stick (atan of the rate)");
    check(fabs(r64 - atan(0.028) * 180.0 / PI) < 0.01, "grip turn rate: atan(rate) per tick");
    check(fabs(r6) < 1e-9, "stick inside the dead zone does not turn");
    check(fabs(yaw_rate(80, 0) - r64) < 1e-12, "stick saturates at 63");
    check(yaw_rate(64, SH2Z_VD_BTN_LEAN_R) > r64 * 1.2, "leaning into the turn turns faster");
}

/* Drive into the right-hand wall and measure the response along its normal. */
static void test_wall(void) {
    sh2z_vd_body_t b = start_body();
    sh2z_vd_input_t in;
    uint16_t prev = 0;
    int i, contact = -1;
    double before_n = 0, after_n = 0, e0 = 0, e1 = 0;
    for(i = 0; i < 400 && contact < 0; i++) {
        const int stick = (i >= 120 && i < 136) ? 64 : 0;
        sh2z_vd_body_t last = b;
        in = pad(SH2Z_VD_BTN_ACCEL, prev, stick);
        prev = SH2Z_VD_BTN_ACCEL;
        sh2z_vd_step(&b, &track, &veh, &rules, &in);
        if(b.flags & SH2Z_VD_RECOIL) {
            contact = i;
            /* Normal = road right = -road left; positive into the wall. */
            before_n = -(f30(last.road[2][0]) * f16(last.vel[0]) + f30(last.road[2][2]) * f16(last.vel[2]));
            after_n = -(f30(b.road[2][0]) * f16(b.vel[0]) + f30(b.road[2][2]) * f16(b.vel[2]));
            e0 = f16(last.energy);
            e1 = f16(b.energy);
        }
    }
    check(contact > 0, "the vehicle reaches the right wall");
    check(before_n > 1.0, "approaching the wall");
    check(after_n < 0.0, "signed response: the velocity along the wall normal reverses");
    check(fabs(after_n + 0.7 * before_n * 0.998) < 0.35 * before_n, "restitution about 0.7 (1 - 1.7)");
    check(fabs((e0 - e1) - 0.7 * before_n) < 0.15 * before_n + 0.1, "damage 0.7 x speed into the wall");
    check(fabs(f16(sh2z_vd_body_lateral(&b)) + 300.0) < 1.0, "pushed back inside the road edge");
}

static void test_zones_and_laps(void) {
    sh2z_vd_body_t b = start_body();
    sh2z_vd_input_t in;
    uint16_t prev = 0;
    int i, plate = -1, laps_at = -1, recharge = 0, finish = -1;
    b.energy = q16(100.0);
    for(i = 0; i < 9000 && finish < 0; i++) {
        /* A simple line-holding driver from the previous tick's state. */
        double s = 0.004 * f16(sh2z_vd_body_lateral(&b)) +
                   2.5 * (f30(b.body[0][0]) * f30(b.road[2][0]) + f30(b.body[0][1]) * f30(b.road[2][1]) +
                          f30(b.body[0][2]) * f30(b.road[2][2]));
        if(s > 1) s = 1;
        if(s < -1) s = -1;
        in = pad(SH2Z_VD_BTN_ACCEL, prev, (int)lround(s * 63.0));
        prev = SH2Z_VD_BTN_ACCEL;
        sh2z_vd_step(&b, &track, &veh, &rules, &in);
        if(plate < 0 && (b.flags & SH2Z_VD_PLATE_BOOST)) plate = i;
        if((b.flags & SH2Z_VD_ZONE_MASK) == SH2Z_VD_ZONE_RECHARGE) recharge++;
        if(laps_at < 0 && b.laps_done == 2) laps_at = i;
        if(b.flags & SH2Z_VD_FINISHED) finish = i;
    }
    check(plate > 0, "the boost plate starts a free boost");
    check(recharge > 0 && f16(b.energy) > 170.0, "the recharge strip refills energy");
    check(laps_at > 0, "the first crossing after the start completes lap 1");
    check(finish > laps_at && b.laps_done == 4, "three laps finish the race");
    check(b.race_time == b.lap_times[0] + b.lap_times[1] + b.lap_times[2], "lap times sum to the race time");
    check(b.lap_times[0] > 0 && b.lap_times[1] > 0, "lap times recorded");
}

static void test_line_view(void) {
    sh2z_vd_body_t b = start_body();
    sh2z_vd_line_view_t v;
    int32_t target[3];
    int k;
    for(k = 0; k < 3; k++) target[k] = b.pos[k];
    target[2] -= q16(50.0);  /* 50 to the left */
    check(sh2z_vd_locate(&b, &track, target, 100, (int64_t)1 << 32) == 0, "locate");
    sh2z_vd_line_view(&b, &track, &v);
    check(abs(v.lateral + 50 * 256) <= 2, "line view: lateral is positive to the right, Q8");
    check(v.dist >= 0 && v.dist < (track.length >> 4), "line view: distance inside the lap");
}

static void test_golden(void) {
    sh2z_vd_body_t b = start_body();
    sh2z_vd_input_t in;
    uint64_t h = UINT64_C(0xCBF29CE484222325);
    uint16_t prev = 0;
    int i;
    for(i = 0; i < 900; i++) {
        uint16_t held = (uint16_t)(SH2Z_VD_BTN_ACCEL | ((i % 240) == 200 ? SH2Z_VD_BTN_BOOST : 0) |
                                   ((i / 60) % 5 == 3 ? SH2Z_VD_BTN_LEAN_L : 0));
        in = pad(held, prev, ((i / 45) % 3 - 1) * 40);
        prev = held;
        sh2z_vd_step(&b, &track, &veh, &rules, &in);
        h = fnv(h, &b);
    }
    printf("golden hash %016llx\n", (unsigned long long)h);
    check(h == UINT64_C(0x9509eedd2aaf382f), "golden state hash over 900 ticks (fixed by the contract)");
}

int main(void) {
    build_track();
    build_vehicle();
    test_spline();
    test_locate();
    test_launch_and_balance();
    test_steering();
    test_wall();
    test_zones_and_laps();
    test_line_view();
    test_golden();
    printf("vehicle-dynamics: %u checks passed\n", checks);
    return 0;
}
