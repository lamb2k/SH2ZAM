/* Host tests for vehicle-dynamics-3 (sh2z_vd_step_contact, sh2z_vd_step2_contact) on the
 * vehicle-dynamics-1 test oval: with a zero contact it is sh2z_vd_step bit
 * for bit; a push lands in the tick's velocity and position; damage marks
 * the tick, empties into the spin-out before the pad is read, and a hit
 * takes grip away; the regain stops at the maximum and not while spinning
 * out. Two wrong placements of the contact are controls that the same
 * checks must reject.
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
        fprintf(stderr, "vehicle-contact check %u failed: %s\n", checks, what);
        exit(EXIT_FAILURE);
    }
}

static int32_t q16(double x) { return (int32_t)lround(x * 65536.0); }
static int32_t q30(double x) { return (int32_t)lround(x * 1073741824.0); }
static int32_t q12(double x) { return (int32_t)lround(x * 4096.0); }
static double f16(int32_t v) { return v / 65536.0; }
static int32_t mu_test(int32_t a, int32_t u) { return (int32_t)(((int64_t)a * u + (INT64_C(1) << 29)) >> 30); }

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


/* A scripted drive: throttle, steering waves, side and spin attacks, boosts, walls. */
static sh2z_vd_input_t drive(int i, uint16_t *prev) {
    uint16_t held = SH2Z_VD_BTN_ACCEL;
    int sx = (int)(40.0 * sin(i / 37.0));
    const int ph = i % 300;
    if(ph == 100 || ph == 104) held |= SH2Z_VD_BTN_LEAN_L;                        /* side attack */
    if(ph >= 200 && ph < 212) held |= SH2Z_VD_BTN_LEAN_L;                         /* held for the spin */
    if(ph == 203 || ph == 207) held |= SH2Z_VD_BTN_LEAN_R;                        /* spin attack */
    if(i > 1200 && i % 400 == 0) held |= SH2Z_VD_BTN_BOOST;
    if(i % 900 > 700) sx = 64;                                                     /* into the walls */
    sh2z_vd_input_t in = pad(held, *prev, sx);
    *prev = held;
    return in;
}

static void test_zero_contact(void) {
    sh2z_vd_body_t a = start_body(), b = start_body();
    const sh2z_vd_contact_t zero = {{0, 0, 0}, 0, 0, 0};
    uint16_t pa = 0, pb = 0;
    uint64_t ha = UINT64_C(0xcbf29ce484222325), hb = ha;
    int walls = 0, attacks = 0, spins = 0;
    for(int i = 0; i < 4000; i++) {
        sh2z_vd_input_t ia = drive(i, &pa), ib = drive(i, &pb);
        sh2z_vd_step(&a, &track, &veh, &rules, &ia);
        sh2z_vd_step_contact(&b, &track, &veh, &rules, &ib, &zero);
        ha = fnv(ha, &a);
        hb = fnv(hb, &b);
        check(memcmp(&a, &b, sizeof a) == 0, "zero contact: the same body as sh2z_vd_step every tick");
        walls += (a.flags & SH2Z_VD_RECOIL) != 0;
        attacks += a.attack == SH2Z_VD_ATTACK_SIDE;
        spins += a.attack == SH2Z_VD_ATTACK_SPIN;
    }
    check(ha == hb && walls > 0 && attacks > 0 && spins > 0, "zero contact: walls, side and spin attacks exercised");
    printf("zero contact: 4000 ticks identical (%d wall, %d side-attack, %d spin-attack ticks)\n", walls, attacks, spins);
}

/* Where the contact goes: the contract's place, or one of two wrong places (controls). */
enum { PLACE_CONTRACT, PLACE_PUSH_BEFORE, PLACE_DAMAGE_BEFORE };

static void contact_step(int place, sh2z_vd_body_t *b, const sh2z_vd_input_t *in, const sh2z_vd_contact_t *c) {
    sh2z_vd_contact_t rest = *c;
    if(place == PLACE_PUSH_BEFORE) {
        /* the push as a velocity change before the tick: damping and grip act on it */
        for(int k = 0; k < 3; k++) b->vel[k] += c->push[k];
        rest.push[0] = rest.push[1] = rest.push[2] = 0;
    } else if(place == PLACE_DAMAGE_BEFORE && c->damage > 0) {
        /* the damage taken before the tick's resets: its mark is cleared */
        b->energy -= c->damage;
        if(b->energy < 0) {
            b->energy = 0;
            b->flags = (b->flags | SH2Z_VD_SPINNING) & ~SH2Z_VD_CONTROLLABLE;
            b->spin_timer = 1;
        }
        b->flags |= SH2Z_VD_DAMAGED;
        rest.damage = 0;
    }
    sh2z_vd_step_contact(b, &track, &veh, &rules, in, &rest);
}

/* 0 when every claim holds for this placement, else the number of the first failed claim. */
static int contact_claims(int place) {
    sh2z_vd_body_t warm = start_body();
    uint16_t prev = 0;
    for(int i = 0; i < 100; i++) {   /* straight throttle: at speed, clear of the walls */
        sh2z_vd_input_t in = pad(SH2Z_VD_BTN_ACCEL, prev, 0);
        prev = SH2Z_VD_BTN_ACCEL;
        sh2z_vd_step(&warm, &track, &veh, &rules, &in);
    }
    if(warm.flags & SH2Z_VD_RECOIL) return 99;
    const sh2z_vd_input_t in = pad(SH2Z_VD_BTN_ACCEL, SH2Z_VD_BTN_ACCEL, 10);
    /* 1-2. Push: lands in this tick's velocity and position, sideways and backwards. */
    {
        sh2z_vd_body_t a = warm, b = warm;
        sh2z_vd_contact_t c = {{0, 0, 0}, 0, 0, 0};
        for(int k = 0; k < 3; k++) c.push[k] = mu_test(q16(3.0), warm.body[2][k]) + mu_test(q16(-1.5), warm.body[0][k]);
        sh2z_vd_step(&a, &track, &veh, &rules, &in);
        contact_step(place, &b, &in, &c);
        for(int k = 0; k < 3; k++) {
            if(llabs((long long)(b.vel[k] - a.vel[k]) - c.push[k]) > 2) return 1;
            if(llabs((long long)(b.pos[k] - a.pos[k]) - c.push[k]) > 64) return 2;
        }
    }
    /* 3-5. Damage: energy, the damage mark (no grip this tick), no spin-out above zero. */
    {
        sh2z_vd_body_t a = warm, b = warm;
        sh2z_vd_contact_t c = {{0, 0, 0}, q16(20.0), 0, 0};
        sh2z_vd_step(&a, &track, &veh, &rules, &in);
        contact_step(place, &b, &in, &c);
        if(b.energy != a.energy - q16(20.0)) return 3;
        if(!(b.flags & SH2Z_VD_DAMAGED) || (b.flags & SH2Z_VD_GRIPPING)) return 4;
        if(b.flags & SH2Z_VD_SPINNING) return 5;
    }
    /* 6-7. Damage past zero: spin-out before the pad is read (the pad changes nothing). */
    {
        sh2z_vd_body_t b = warm, s = warm;
        sh2z_vd_contact_t c = {{0, 0, 0}, warm.energy + q16(4.0), 0, 0};
        const sh2z_vd_input_t hard = pad(SH2Z_VD_BTN_ACCEL, SH2Z_VD_BTN_ACCEL, 63), soft = pad(0, 0, 0);
        contact_step(place, &b, &hard, &c);
        contact_step(place, &s, &soft, &c);
        if(!(b.flags & SH2Z_VD_SPINNING) || (b.flags & SH2Z_VD_CONTROLLABLE) || b.energy != 0 || b.spin_timer != 2) return 6;
        if(memcmp(&b, &s, sizeof b) != 0) return 7;
    }
    /* 8. Hit: grip off for contact_grip ticks from this one. */
    {
        sh2z_vd_body_t b = warm;
        sh2z_vd_contact_t c = {{0, 0, 0}, 0, 0, 1};
        contact_step(place, &b, &in, &c);
        if(b.hit_timer != rules.contact_grip - 1 || (b.flags & SH2Z_VD_GRIPPING)) return 8;
    }
    /* 9-10. Regain: up to the maximum; not while spinning out. */
    {
        sh2z_vd_body_t a = warm, b = warm;
        a.energy = b.energy = veh.max_energy - q16(10.0);
        sh2z_vd_contact_t c = {{0, 0, 0}, 0, q16(22.25), 0};
        contact_step(place, &b, &in, &c);
        sh2z_vd_step(&a, &track, &veh, &rules, &in);
        if(b.energy != veh.max_energy) return 9;
        b = warm;
        b.flags |= SH2Z_VD_SPINNING;
        b.energy = q16(5.0);
        contact_step(place, &b, &in, &c);
        if(b.energy != q16(5.0)) return 10;
    }
    return 0;
}

/* Tubes: the oval made a pipe all round (radius 300 round the line, no joins), the body started on
 * the pipe's floor. Zero contact gives sh2z_vd_step2 bit for bit; damage and the hit mark as on
 * the road; road shapes give sh2z_vd_step_contact bit for bit. */
static void test_tubes(void) {
    static sh2z_vd_node_t tnodes[N];
    sh2z_vd_track2_t t2;
    memcpy(tnodes, nodes, sizeof(tnodes));
    for(int i = 0; i < N; i++) tnodes[i].shape = SH2Z_VD_SHAPE_PIPE;
    t2.base = track;
    t2.base.nodes = tnodes;
    t2.joins = NULL;
    t2.tube = (sh2z_vd_tube_rules_t){q16(1000.0), q16(23.0), q16(200.0), q16(200.0), q16(0.01), q30(0.01)};
    sh2z_vd_body_t a = start_body(), b;
    for(int k = 0; k < 3; k++) { a.pos[k] -= mu_test(q16(292.0), a.road[1][k]); a.ground[k] = a.pos[k]; }
    b = a;
    const sh2z_vd_contact_t zero = {{0, 0, 0}, 0, 0, 0};
    uint16_t pa = 0;
    int active = 0, pushes = 0, pushes_ok = 1;
    double share_min = 1e9, share_max = -1e9;
    for(int i = 0; i < 3000; i++) {
        const int sx = (int)(30.0 * sin(i / 53.0));
        sh2z_vd_input_t in = pad(SH2Z_VD_BTN_ACCEL, pa, sx);
        pa = SH2Z_VD_BTN_ACCEL;
        if(!(a.flags & SH2Z_VD_ACTIVE)) break;
        if(i % 97 == 50) {
            /* A push in the pipe moves the body its way: on the curved floor the radial clamp takes a
             * share, so the claim is that at least 90% of it lands along it, and the damage and hit
             * marks behave as on the road. */
            sh2z_vd_body_t c0 = a, c1 = a, c2 = a;
            const sh2z_vd_contact_t marks = {{0, 0, 0}, q16(5.0), 0, 1};
            sh2z_vd_step2_contact(&c2, &t2, &veh, &rules, &in, &marks);
            sh2z_vd_contact_t c = {{0, 0, 0}, 0, 0, 0};
            for(int k = 0; k < 3; k++) c.push[k] = mu_test(q16(-1.5), a.body[0][k]);   /* along the tube */
            sh2z_vd_step2(&c0, &t2, &veh, &rules, &in);
            sh2z_vd_step2_contact(&c1, &t2, &veh, &rules, &in, &c);
            double along = 0, len2 = 0;
            for(int k = 0; k < 3; k++) { along += f16(c1.vel[k] - c0.vel[k]) * f16(c.push[k]); len2 += f16(c.push[k]) * f16(c.push[k]); }
            /* The pipe's radial projection after the integration reshapes the velocity, so the share of
             * the push that remains is reported, and only its sign is a claim. */
            const double share = along / len2;
            if(share < share_min) share_min = share;
            if(share > share_max) share_max = share;
            check(share > 0.0, "tubes: a push moves the body its way");
            if(!(c0.flags & SH2Z_VD_ZONE_MASK))   /* off the recharge strip, which refills after the damage */
                check(c2.energy == c0.energy - q16(5.0), "tubes: damage as on the road");
            check(c2.hit_timer >= rules.contact_grip - 1, "tubes: the hit sets the grip-off timer");
            pushes++;
        }
        sh2z_vd_step2(&a, &t2, &veh, &rules, &in);
        sh2z_vd_step2_contact(&b, &t2, &veh, &rules, &in, &zero);
        check(memcmp(&a, &b, sizeof a) == 0, "tubes, zero contact: the same body as sh2z_vd_step2 every tick");
        active++;
    }
    check(active > 1000 && pushes > 10, "tubes: the drive stays in the pipe");
    check(pushes_ok, "tubes: a push moves the body its way in the pipe; damage and hit as on the road");
    /* On road shapes the tube step with contact is the road step with contact, bit for bit. */
    {
        sh2z_vd_track2_t tr;
        tr.base = track; tr.joins = NULL; tr.tube = t2.tube;
        sh2z_vd_body_t r0 = start_body(), r1 = start_body();
        uint16_t p0 = 0, p1 = 0;
        for(int i = 0; i < 3000; i++) {
            sh2z_vd_input_t i0 = drive(i, &p0), i1 = drive(i, &p1);
            sh2z_vd_contact_t c = {{0, 0, 0}, 0, 0, 0};
            if(i % 41 == 7) { c.push[0] = q16(1.5); c.push[2] = -q16(2.0); c.hit = (i % 82 == 7); }
            if(i % 157 == 3) c.damage = q16(9.0);
            if(i % 300 == 5) c.regain = q16(20.0);
            sh2z_vd_step_contact(&r0, &track, &veh, &rules, &i0, &c);
            sh2z_vd_step2_contact(&r1, &tr, &veh, &rules, &i1, &c);
            check(memcmp(&r0, &r1, sizeof r0) == 0, "road shapes: sh2z_vd_step2_contact is sh2z_vd_step_contact bit for bit");
        }
    }
    printf("tubes: %d ticks in a pipe identical to sh2z_vd_step2; %d pushes, share kept along the push %.3f..%.3f\n", active, pushes, share_min, share_max);
}

int main(void) {
    build_track();
    build_vehicle();
    test_zero_contact();
    test_tubes();
    const int c0 = contact_claims(PLACE_CONTRACT);
    printf("contract placement: first failed claim %d (0 none)\n", c0);
    check(c0 == 0, "contact claims hold");
    const int c1 = contact_claims(PLACE_PUSH_BEFORE), c2 = contact_claims(PLACE_DAMAGE_BEFORE);
    printf("controls: push before the tick fails claim %d, damage before the tick fails claim %d\n", c1, c2);
    check(c1 != 0 && c2 != 0, "both wrong placements are rejected");
    printf("vehicle-dynamics-3: %u checks passed\n", checks);
    return 0;
}
