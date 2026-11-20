/* Line field (contract tags line-field-1 to line-field-4) checks.
 * Invariants over long random races, an independent grid formula, exact
 * contact arithmetic on hand-placed pairs, determinism, level of detail, and
 * a control for every behavioural claim (the same scene with the feature off
 * must fail the claim).
 *
 * Copyright (c) 2026 SH2ZAM Contributors. MIT License.
 * Inspired by SH4ZAM by Falco Girgis and SH4ZAM Contributors.
 */
#include "sh2zam/line_field_experimental.h"
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static uint64_t checks;
#define CHECK(e) do { ++checks; if(!(e)) { fprintf(stderr,"FAIL %s:%d: %s\n",__FILE__,__LINE__,#e); exit(EXIT_FAILURE); } } while(0)
#define U(x) ((int32_t)((x) * 256))
#define NODES 96u
#define MAXV 32u

static sh2z_line_node_t nodes[NODES];
static sh2z_line_t line;
static sh2z_vehicle_t veh[MAXV];
static sh2z_field_brain_t brain[MAXV];
static uint8_t order[MAXV], slot[MAXV], rank_order[MAXV];
static sh2z_field_params_t P;
static uint32_t rng = UINT32_C(0x9e3779b9);
static uint32_t rnd(void) { rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5; return rng; }

/* A closed line of NODES x 200 units whose width and racing line vary. */
static void make_line(void) {
    const int32_t span = U(200);
    for(uint32_t i = 0; i < NODES; ++i) {
        nodes[i].start = (int32_t)i * span;
        nodes[i].lateral = (int16_t)(((int32_t)(i % 12u) - 6) * 15);
        nodes[i].speed_cap = (uint16_t)(60 * 16);
        nodes[i].half_width = (uint16_t)(220 + (int32_t)((i * 7u) % 5u) * 40);
        nodes[i].flags = 0;
        nodes[i].inv_span = sh2z_line_node_inv_span(span);
    }
    line.nodes = nodes; line.count = NODES; line.length = (int32_t)NODES * span;
}

static void params(void) {
    memset(&P, 0, sizeof P);
    P.pace_gain = 3277; P.pace_max = 81920; P.chase_gain = 655; P.steer_gain = 9830; P.margin = U(6);
    P.avoid_long = U(92); P.avoid_lat = U(69); P.avoid_offset = U(92); P.avoid_ticks = 40; P.pass_hold = 60;
    P.contact_long = U(46); P.contact_lat = U(40); P.bump_keep = 16384;
    P.hit_energy = 32768; P.side_energy = U(2); P.attack_energy = U(10); P.wall_energy = U(3); P.wall_keep = 62259;
    P.attack_long = U(184); P.attack_lat = U(92);
    P.boost_speed = U(6); P.boost_energy = U(10); P.boost_min_share = 19661; P.boost_min_cap = 0; P.boost_behind = U(150);
    P.cruise = U(30); P.rank_lead = U(8);
    P.lod_near[0] = INT32_MAX; P.lod_near[1] = INT32_MAX; P.lod_near[2] = INT32_MAX;
    P.boost_ticks = 90; P.laps = 2; P.boost_from_lap = 1;
}

static void vehicle(uint32_t i, uint32_t grid, int32_t lap_ticks) {
    memset(&veh[i], 0, sizeof veh[i]); memset(&brain[i], 0, sizeof brain[i]);
    sh2z_field_grid_slot(&line, grid, U(146), U(100), U(50), 4, &veh[i].t);
    veh[i].energy = veh[i].max_energy = U(160);
    veh[i].accel = 40; veh[i].brake = 96; veh[i].top_speed = U(60); veh[i].grip = U(6); veh[i].armor = 256;
    brain[i].pace = line.length / lap_ticks;
    brain[i].lane = ((int32_t)(rnd() % 61u) - 30) * 256;
    brain[i].chase = -1;
}

static sh2z_field_t F;
static void field(uint32_t count, uint32_t seed) {
    sh2z_field_init(&F, &line, &P, veh, brain, order, slot, rank_order, count, seed);
}

static void test_grid(void) {
    for(uint32_t columns = 1; columns <= 4; ++columns) for(uint32_t p = 0; p < 40; ++p) {
        sh2z_track_state_t t;
        const int32_t back = U(146), step = U(100), half = U(50);
        sh2z_field_grid_slot(&line, p, back, step, half, columns, &t);
        int64_t total = -(int64_t)back - (int64_t)p * step;                 /* relative to the line start */
        int64_t d = total; int32_t lap = 0;
        while(d < 0) { d += line.length; --lap; }
        CHECK(t.dist == (int32_t)d && t.lap == lap);
        CHECK(t.lateral == ((int32_t)columns - 1 - 2 * (int32_t)(p % columns)) * half);
        CHECK(t.speed == 0 && t.lat_vel == 0 && t.node == sh2z_line_locate(&line, t.dist, 0));
    }
}

/* Every invariant the plan lists as exact, checked on every tick. */
static void invariants(uint32_t count, const sh2z_track_state_t *ext_before) {
    uint32_t seen = 0, fin = 0, ret = 0;
    for(uint32_t i = 0; i < count; ++i) {
        const sh2z_vehicle_t *v = &veh[i];
        sh2z_line_sample_t at;
        sh2z_line_sample(&line, v->t.node, v->t.dist, &at);
        if(v->flags & SH2Z_VEH_EXTERNAL) { CHECK(memcmp(&v->t, ext_before, sizeof v->t) == 0); }
        else CHECK(v->t.lateral >= -at.half_width && v->t.lateral <= at.half_width);
        CHECK(v->t.speed >= 0 && v->t.dist >= 0 && v->t.dist < line.length);
        CHECK(v->energy >= 0 && v->energy <= v->max_energy);
        CHECK(order[slot[i]] == i && v->rank < count && rank_order[v->rank] == i);
        seen |= UINT32_C(1) << v->rank;
        fin += (v->flags & SH2Z_VEH_FINISHED) != 0; ret += (v->flags & SH2Z_VEH_RETIRED) != 0;
        if(v->flags & SH2Z_VEH_RETIRED) CHECK(v->energy == 0 && brain[i].retire_tick >= 0);
        if(v->flags & SH2Z_VEH_FINISHED) CHECK(brain[i].finish_tick >= 0 && v->t.lap >= (int32_t)P.laps);
    }
    CHECK(seen == (count == 32u ? UINT32_MAX : (UINT32_C(1) << count) - 1u));
    CHECK(fin == F.finished && ret == F.retired);
    /* Finishers lead in finishing order; retired vehicles trail. */
    for(uint32_t r = 1; r < count; ++r) {
        const sh2z_vehicle_t *a = &veh[rank_order[r - 1]], *b = &veh[rank_order[r]];
        if(b->flags & SH2Z_VEH_FINISHED) CHECK((a->flags & SH2Z_VEH_FINISHED) && brain[a->id].finish_seq < brain[b->id].finish_seq);
        if(a->flags & SH2Z_VEH_RETIRED) CHECK((b->flags & SH2Z_VEH_RETIRED) != 0);
    }
}

static uint32_t race(uint32_t count, uint32_t seed, int external, uint32_t ticks, int check) {
    rng = seed * UINT32_C(2654435761) + 1u;
    for(uint32_t i = 0; i < count; ++i) vehicle(i, i, 1500 + (int32_t)i * 20);
    for(uint32_t i = 4; i < count; i += 5) { brain[i].chase = (int16_t)(i - 1u); brain[i].chase_lead = 30; brain[i].aggression = 60; }
    if(external) veh[0].flags |= SH2Z_VEH_EXTERNAL;
    field(count, seed);
    F.focus = external ? 0 : -1;
    for(uint32_t t = 0; t < ticks; ++t) {
        if(t == 60) sh2z_field_start(&F);
        if(external) {   /* an erratic outside driver: weaves and changes speed */
            veh[0].t.speed = t < 60 ? 0 : U(30) + (int32_t)(rnd() % 4096u);
            veh[0].t.lateral = (int32_t)((t / 40u) % 7u) * U(50) - U(150);
            sh2z_track_advance(&line, &veh[0].t, 1);
            veh[0].kick_lat = veh[0].kick_speed = 0; veh[0].energy = veh[0].max_energy;
        }
        const sh2z_track_state_t before = veh[0].t;
        sh2z_field_step(&F);
        if(check) invariants(count, &before);
    }
    return sh2z_field_hash(&F);
}

static void test_races(void) {
    for(uint32_t seed = 1; seed <= 6; ++seed) {
        P.lod_near[0] = INT32_MAX; P.lod_near[1] = INT32_MAX; P.lod_near[2] = INT32_MAX;
        const uint32_t h1 = race(24, seed, 0, 5000, 1);
        /* line-field-1 results, pinned: with no finish targets line-field-2 races step for step as before. */
        static const uint32_t lf1[6] = { 0x7c56497eu, 0x9c2d952fu, 0x12afee5bu, 0xccfb9880u, 0x080ae758u, 0x244b0332u };
        CHECK(h1 == lf1[seed - 1u]);
        CHECK(F.finished + F.retired == 24u);                       /* nobody is stuck: the race ends */
        CHECK(F.rank_changes > 0u);                                 /* places change hands */
        const uint32_t h2 = race(24, seed, 0, 5000, 0);
        CHECK(h1 == h2);                                            /* same seed, same race */
        CHECK(race(24, seed + 100u, 0, 5000, 0) != h1);             /* control: another seed is another race */
        (void)race(30, seed, 1, 5000, 1);                           /* with an outside vehicle among them */
        P.lod_near[0] = U(1500); P.lod_near[1] = U(4000); P.lod_near[2] = U(8000);
        (void)race(30, seed, 1, 5000, 1);                           /* and with level of detail */
    }
    P.lod_near[0] = INT32_MAX; P.lod_near[1] = INT32_MAX; P.lod_near[2] = INT32_MAX;
}

/* A faster agent behind a slower one on the same lateral gets by untouched. */
static uint32_t pass_scene(int avoid) {
    params();
    if(!avoid) { P.avoid_long = 0; P.avoid_ticks = 0; }
    rng = 7u;
    vehicle(0, 0, 2400); vehicle(1, 3, 1500);
    veh[0].t.lateral = veh[1].t.lateral = 0; brain[0].lane = brain[1].lane = 0;
    veh[1].t.dist = veh[0].t.dist - U(300); veh[1].t.node = sh2z_line_locate(&line, veh[1].t.dist, 0);
    field(2, 3u);
    sh2z_field_start(&F);
    for(uint32_t t = 0; t < 1500; ++t) sh2z_field_step(&F);
    return F.contacts;
}
static void test_pass(void) {
    CHECK(pass_scene(1) == 0u);
    CHECK(sh2z_track_gap(&line, &veh[1].t, &veh[0].t) > U(200));   /* and it is clearly ahead */
    CHECK(veh[1].rank == 0u && veh[0].rank == 1u);
    CHECK(pass_scene(0) > 0u);                                      /* control: without avoiding they collide */
    params();
}

/* Exact contact arithmetic on a hand-placed pair. */
static void test_contact(void) {
    params();
    rng = 9u;
    vehicle(0, 0, 2000); vehicle(1, 1, 2000);
    veh[0].t.dist = U(5000); veh[0].t.lateral = U(10); veh[0].t.speed = U(30);
    veh[1].t.dist = U(5000) - U(20); veh[1].t.lateral = -U(10); veh[1].t.speed = U(34);
    veh[0].t.node = sh2z_line_locate(&line, veh[0].t.dist, 0); veh[1].t.node = sh2z_line_locate(&line, veh[1].t.dist, 0);
    field(2, 1u);
    /* Not started: the step only orders, so drive the contact through one started tick with frozen agents. */
    sh2z_field_start(&F);
    veh[0].accel = veh[1].accel = 0; veh[0].brake = veh[1].brake = 0; veh[0].grip = veh[1].grip = 0;
    brain[0].pace = brain[1].pace = 0;
    const int32_t e0 = veh[0].energy, e1 = veh[1].energy;
    sh2z_field_step(&F);
    CHECK(F.contacts == 1u);
    CHECK(veh[0].t.lateral - veh[1].t.lateral >= P.contact_lat);    /* pushed apart sideways */
    CHECK(veh[1].t.speed <= veh[0].t.speed + (U(4) >> 2) + 64);     /* trailing speed limited to a quarter of the difference */
    const int32_t hit = P.side_energy + (int32_t)(((int64_t)U(4) * P.hit_energy) >> 16);
    CHECK(e0 - veh[0].energy == hit && e1 - veh[1].energy == hit);  /* both pay the impact once */
    const int32_t after0 = veh[0].energy;
    veh[1].t.lateral = veh[0].t.lateral - U(5);                     /* still touching on the next tick */
    sh2z_field_step(&F);
    CHECK(veh[0].energy == after0 && F.contacts == 1u);             /* same incident: no second charge */
    /* An outside vehicle is never moved: it is kicked instead, and the agent takes the whole push. */
    vehicle(0, 0, 2000); vehicle(1, 1, 2000);
    veh[0].flags |= SH2Z_VEH_EXTERNAL;
    veh[0].t.dist = U(5000); veh[0].t.lateral = U(10); veh[0].t.speed = U(30);
    veh[1].t.dist = U(5000) - U(20); veh[1].t.lateral = -U(10); veh[1].t.speed = U(34);
    veh[0].t.node = sh2z_line_locate(&line, veh[0].t.dist, 0); veh[1].t.node = sh2z_line_locate(&line, veh[1].t.dist, 0);
    field(2, 1u);
    sh2z_field_start(&F);
    veh[1].accel = 0; veh[1].brake = 0; veh[1].grip = 0; brain[1].pace = 0;
    const sh2z_track_state_t ext = veh[0].t;
    sh2z_field_step(&F);
    CHECK(memcmp(&ext, &veh[0].t, sizeof ext) == 0 && veh[0].kick_lat > 0 && F.contacts == 1u);
    params();
}

/* contact_lat 0 turns the field's own contact off: an overlapping pair is neither pushed, bumped nor
 * charged, and no incident is counted. Control: the same scene with contact_lat 40 (test_contact). */
static uint32_t contact_off_scene(int32_t contact_lat) {
    params();
    P.contact_lat = contact_lat;
    rng = 9u;
    vehicle(0, 0, 2000); vehicle(1, 1, 2000);
    veh[0].t.dist = U(5000); veh[0].t.lateral = U(10); veh[0].t.speed = U(30);
    veh[1].t.dist = U(5000) - U(20); veh[1].t.lateral = -U(10); veh[1].t.speed = U(34);
    veh[0].t.node = sh2z_line_locate(&line, veh[0].t.dist, 0); veh[1].t.node = sh2z_line_locate(&line, veh[1].t.dist, 0);
    field(2, 1u);
    sh2z_field_start(&F);
    veh[0].accel = veh[1].accel = 0; veh[0].brake = veh[1].brake = 0; veh[0].grip = veh[1].grip = 0;
    brain[0].pace = brain[1].pace = 0;
    const int32_t e0 = veh[0].energy, e1 = veh[1].energy;
    sh2z_field_step(&F);
    const uint32_t untouched = veh[0].t.lateral == U(10) && veh[1].t.lateral == -U(10) && veh[0].energy == e0 &&
                               veh[1].energy == e1 && !((veh[0].flags | veh[1].flags) & SH2Z_VEH_TOUCH) &&
                               !brain[0].touch_cool && !brain[1].touch_cool;
    params();
    return F.contacts == 0u && untouched;
}
static void test_contact_off(void) {
    CHECK(contact_off_scene(0) == 1u);
    CHECK(contact_off_scene(-U(1)) == 1u);
    CHECK(contact_off_scene(U(40)) == 0u);    /* control: the same pair in reach collides */
}

/* Level of detail: far agents take fewer steps and still arrive on time. */
static void test_lod(void) {
    params();
    int32_t lap_full[30];
    (void)race(30, 5u, 1, 6000, 0);
    const uint32_t steps_full = F.steps;
    for(uint32_t i = 0; i < 30; ++i) lap_full[i] = brain[i].lap_tick[0];
    P.lod_near[0] = U(1500); P.lod_near[1] = U(4000); P.lod_near[2] = U(8000);
    (void)race(30, 5u, 1, 6000, 0);
    CHECK(F.steps * 10u < steps_full * 8u);                         /* at least a fifth fewer agent steps */
    uint32_t compared = 0;
    for(uint32_t i = 1; i < 30; ++i) {
        if(lap_full[i] < 0 || brain[i].lap_tick[0] < 0) continue;
        CHECK(abs(lap_full[i] - brain[i].lap_tick[0]) * 100 <= lap_full[i] * 6);   /* first lap within 6% */
        ++compared;
    }
    CHECK(compared >= 20u);
    params();
}

/* Energy runs out: the vehicle retires, leaves contact, stops and stays put. */
static void test_retire(void) {
    params();
    rng = 11u;
    vehicle(0, 0, 2000); vehicle(1, 1, 2000);
    veh[1].energy = 1;
    veh[0].t.dist = U(5000); veh[0].t.lateral = U(10); veh[0].t.speed = U(30);
    veh[1].t.dist = U(5000) - U(20); veh[1].t.lateral = -U(10); veh[1].t.speed = U(34);
    veh[0].t.node = sh2z_line_locate(&line, veh[0].t.dist, 0); veh[1].t.node = sh2z_line_locate(&line, veh[1].t.dist, 0);
    field(2, 1u);
    sh2z_field_start(&F);
    sh2z_field_step(&F);
    CHECK((veh[1].flags & SH2Z_VEH_RETIRED) && F.retired == 1u && veh[1].rank == 1u);
    for(uint32_t t = 0; t < 600; ++t) sh2z_field_step(&F);
    CHECK((veh[1].flags & SH2Z_VEH_STOPPED) && veh[1].t.speed == 0);
    const sh2z_track_state_t rest = veh[1].t;
    for(uint32_t t = 0; t < 100; ++t) sh2z_field_step(&F);
    CHECK(memcmp(&rest, &veh[1].t, sizeof rest) == 0 && F.contacts == 1u);
    params();
}

/* line-field-2: one agent, three laps of 1,500-tick pace; from its first lap it re-targets a finish
 * at FINISH ticks. Returns the finish tick. */
static int32_t lone_race(int32_t finish) {
    P.laps = 3;
    rng = 7u;
    vehicle(0, 0, 1500);
    brain[0].lane = 0;
    brain[0].finish_ticks = finish;
    field(1, 3u);
    for(uint32_t t = 0; t < 9000 && !(veh[0].flags & SH2Z_VEH_FINISHED); ++t) {
        if(t == 60) sh2z_field_start(&F);
        sh2z_field_step(&F);
    }
    CHECK(veh[0].flags & SH2Z_VEH_FINISHED);
    return brain[0].finish_tick;
}

static void test_retarget(void) {
    const int32_t keep = lone_race(0);
    const int32_t lap1 = brain[0].lap_tick[0];
    CHECK(keep > 4300 && keep < 4700);                            /* about three laps of 1,500 ticks */
    /* Faster and slower finish targets are met within 1%; the control (0) misses each by far more. */
    static const int32_t targets[] = { 3900, 4100, 4800, 5200 };
    for(unsigned k = 0; k < sizeof targets / sizeof targets[0]; ++k) {
        const int32_t got = lone_race(targets[k]);
        CHECK(brain[0].lap_tick[0] == lap1);                      /* the first lap is unchanged */
        CHECK(got * 100 >= targets[k] * 99 && got * 100 <= targets[k] * 101);
        CHECK((keep - targets[k]) * 100 > targets[k] * 4 || (targets[k] - keep) * 100 > targets[k] * 4);
    }
    /* A target already behind the first lap is ignored: the agent keeps its pace. */
    CHECK(lone_race(lap1 - 10) == keep);
    params();
}

/* line-field-3: a chaser starting behind a fast leader. With a first-lap cap its target never passes the
 * cap before its first lap is done, and it does after; the control (no cap) passes it on the first lap. */
static int32_t chase_race(int32_t cap, int32_t *after) {
    P.laps = 3;
    rng = 11u;
    vehicle(0, 0, 1300);
    vehicle(1, 3, 1500);
    brain[0].lane = brain[1].lane = 0;
    brain[1].chase = 0; brain[1].chase_lead = 0; brain[1].first_lap_cap = cap;
    field(2, 5u);
    int32_t first = 0;
    *after = 0;
    for(uint32_t t = 0; t < 9000 && !(veh[1].flags & SH2Z_VEH_FINISHED); ++t) {
        if(t == 60) sh2z_field_start(&F);
        sh2z_field_step(&F);
        if(veh[1].t.lap < 1) { if(brain[1].target_speed > first) first = brain[1].target_speed; }
        else if(brain[1].target_speed > *after) *after = brain[1].target_speed;
    }
    return first;
}

static void test_first_lap_cap(void) {
    int32_t after, after_free;
    const int32_t cap = U(12);               /* under the leader's pace of about 14.8 per tick */
    const int32_t capped = chase_race(cap, &after);
    const int32_t free_run = chase_race(0, &after_free);
    CHECK(capped <= cap);                     /* held on the first lap */
    CHECK(after > cap);                       /* and free from the second */
    CHECK(free_run > cap);                    /* control: without the cap the first lap passes it */
    params();
}

/* line-field-4: an empty surface, or a surface whose zones and tubes nobody reaches, is line-field-3. */
static sh2z_field_zone_t zone_buf[8];
static sh2z_field_surface_t surf;
static void surface_reset(void) {
    memset(&surf, 0, sizeof surf);
    surf.zones = zone_buf; surf.plate_ticks = 60; surf.slip_steer = 16384; surf.rough_keep = 58982; surf.recharge = 512;
}
static void test_surface_identity(void) {
    static const uint32_t lf1[6] = { 0x7c56497eu, 0x9c2d952fu, 0x12afee5bu, 0xccfb9880u, 0x080ae758u, 0x244b0332u };
    params();
    surface_reset();
    P.surface = &surf;                                              /* no zones, no tubes */
    for(uint32_t seed = 1; seed <= 6; ++seed) CHECK(race(24, seed, 0, 5000, 1) == lf1[seed - 1u]);
    surf.wrap_mask = 0x0180u; surf.wrap_value = 0x0080u;            /* tubes that no node is */
    zone_buf[0] = (sh2z_field_zone_t){ U(100), U(200), U(5000), U(6000), SH2Z_FIELD_ZONE_BOOST };   /* out of reach sideways */
    surf.zone_count = 1;
    for(uint32_t seed = 1; seed <= 6; ++seed) CHECK(race(24, seed, 0, 5000, 1) == lf1[seed - 1u]);
    params();
}

/* One agent, alone, from distance `at` and lateral `lat` with lateral target `want`, for `ticks`. */
static void lone(int32_t at, int32_t lat, int32_t lap_ticks) {
    rng = 5u;
    vehicle(0, 0, lap_ticks);
    brain[0].lane = 0;
    veh[0].t.dist = at; veh[0].t.lap = 0; veh[0].t.lateral = lat; veh[0].t.speed = U(20);
    veh[0].t.node = sh2z_line_locate(&line, at, 0);
    field(1, 3u);
    sh2z_field_start(&F);
}

/* A plate starts a free boost for agents inside its band only. */
static int plate_scene(int32_t lat_lo, int32_t lat_hi, int32_t *energy_lost) {
    params(); surface_reset(); P.surface = &surf;
    zone_buf[0] = (sh2z_field_zone_t){ U(4000), U(4200), lat_lo, lat_hi, SH2Z_FIELD_ZONE_BOOST };
    surf.zone_count = 1;
    lone(U(3800), 0, 2000);
    const int32_t e0 = veh[0].energy;
    int boosted = 0;
    for(uint32_t t = 0; t < 40; ++t) {
        sh2z_field_step(&F);
        if(veh[0].flags & SH2Z_VEH_BOOST) { boosted = 1; CHECK(veh[0].boost_ticks <= surf.plate_ticks); }
    }
    *energy_lost = e0 - veh[0].energy;
    return boosted;
}
static void test_plate(void) {
    int32_t lost;
    CHECK(plate_scene(-U(400), U(400), &lost) == 1 && lost == 0);   /* a free boost */
    CHECK(plate_scene(U(300), U(400), &lost) == 0);                 /* control: the band misses the agent */
    params();
}

/* Rough ground damps the target speed unless boosting; low grip slows the lateral correction;
 * a recharge strip refills energy. Each against the same scene without the zone. */
static int32_t speed_after(uint32_t kind, int boost) {
    params(); surface_reset(); P.surface = &surf;
    P.pace_gain = 0;                                                /* the target is the pace itself */
    zone_buf[0] = (sh2z_field_zone_t){ 0, line.length, -U(1000), U(1000), kind };
    surf.zone_count = kind ? 1u : 0u;
    lone(U(1000), 0, 900);
    veh[0].t.speed = U(40);
    if(boost) { veh[0].flags |= SH2Z_VEH_BOOST; veh[0].boost_ticks = 30000; P.boost_speed = 0; }
    for(uint32_t t = 0; t < 300; ++t) sh2z_field_step(&F);
    return veh[0].t.speed;
}
static int32_t lateral_after(uint32_t kind) {
    params(); surface_reset(); P.surface = &surf;
    zone_buf[0] = (sh2z_field_zone_t){ 0, line.length, -U(1000), U(1000), kind };
    surf.zone_count = kind ? 1u : 0u;
    for(uint32_t i = 0; i < NODES; ++i) nodes[i].lateral = 0;
    lone(U(1000), U(150), 2000);
    for(uint32_t t = 0; t < 12; ++t) sh2z_field_step(&F);
    make_line();
    return veh[0].t.lateral;
}
static void test_zones(void) {
    const int32_t free_run = speed_after(0u, 0), rough = speed_after(SH2Z_FIELD_ZONE_ROUGH, 0);
    CHECK(rough < free_run);                                        /* damped */
    CHECK(abs(rough - (int32_t)(((int64_t)free_run * surf.rough_keep) >> 16)) <= U(1));   /* by rough_keep */
    CHECK(speed_after(SH2Z_FIELD_ZONE_ROUGH, 1) == speed_after(0u, 1));                    /* not while boosting */
    const int32_t grip = lateral_after(0u), slip = lateral_after(SH2Z_FIELD_ZONE_SLIP);
    CHECK(grip < U(150) && slip < U(150));                          /* both steer back toward the line */
    CHECK(slip > grip);                                             /* low grip steers back more slowly */
    /* Recharge: energy refills by recharge per tick, up to the maximum; control: no zone, no refill. */
    for(int on = 0; on < 2; ++on) {
        params(); surface_reset(); P.surface = &surf;
        zone_buf[0] = (sh2z_field_zone_t){ 0, line.length, -U(1000), U(1000), SH2Z_FIELD_ZONE_RECHARGE };
        surf.zone_count = (uint32_t)on;
        lone(U(1000), 0, 2000);
        veh[0].energy = U(100);
        sh2z_field_step(&F);
        CHECK(veh[0].energy == (on ? U(100) + surf.recharge : U(100)));
        for(uint32_t t = 0; t < 400; ++t) sh2z_field_step(&F);
        CHECK(veh[0].energy == (on ? veh[0].max_energy : U(100)));
    }
    params();
}

/* Tubes: nodes 20..59 are a tube of radius 100 (flags 0x80). An agent asked to run 250 units off the line
 * there is never charged at an edge on a step from a tube node (leaving it included), goes round past the
 * radius and keeps within half the round; the control (no wrap mask) never passes the half width. */
static int tube_line(void) {
    for(uint32_t i = 20; i < 60; ++i) { nodes[i].flags = 0x0080u; nodes[i].half_width = 100; nodes[i].lateral = 250; }
    nodes[60].half_width = 120;                                     /* a narrow road after it: it leaves wide */
    return 1;
}
/* Within half the round of a tube whose half width (radius) is hw here: [-c/2, c - c/2). */
static int in_round(int32_t lat, int32_t hw) {
    const int32_t c = (int32_t)(((int64_t)hw * 411775) >> 16);
    return lat >= -(c >> 1) && lat < c - (c >> 1);
}
static int32_t tube_scene(int wrap, int32_t *walls) {
    params(); surface_reset();
    if(wrap) { surf.wrap_mask = 0x0180u; surf.wrap_value = 0x0080u; }
    P.surface = &surf;
    P.boost_from_lap = 4;                                           /* energy moves by the edge only */
    make_line(); (void)tube_line();
    lone(U(200 * 19), U(90), 1000);
    *walls = 0;
    int32_t widest = INT32_MIN;   /* most past the half width on a tube node */
    for(uint32_t t = 0; t < 1200; ++t) {
        sh2z_line_sample_t from;
        sh2z_line_sample(&line, veh[0].t.node, veh[0].t.dist, &from);
        sh2z_field_step(&F);
        sh2z_line_sample_t at;
        sh2z_line_sample(&line, veh[0].t.node, veh[0].t.dist, &at);
        if((veh[0].flags & SH2Z_VEH_WALL) && (from.flags & 0x0080u)) ++*walls;   /* steps from a tube node */
        if(at.flags & 0x0080u) {
            if(wrap) CHECK(in_round(veh[0].t.lateral, at.half_width));
            if(abs(veh[0].t.lateral) - at.half_width > widest) widest = abs(veh[0].t.lateral) - at.half_width;
        } else CHECK(veh[0].t.lateral >= -at.half_width && veh[0].t.lateral <= at.half_width);
    }
    make_line();
    return widest;
}
static void test_tubes(void) {
    int32_t walls_wrap, walls_edge;
    const int32_t e0 = U(160);
    const int32_t widest = tube_scene(1, &walls_wrap);
    CHECK(walls_wrap == 0 && veh[0].energy == e0);                  /* no edge in the tube, none on leaving it */
    CHECK(widest > U(50));                                          /* well past the radius: round the tube */
    const int32_t widest_edge = tube_scene(0, &walls_edge);
    CHECK(widest_edge <= 0);                                        /* control: with edges it cannot go round */
    /* The short way round: from just left of the seam to a target just right of it, the agent crosses
     * the seam instead of sweeping back through 0. */
    params(); surface_reset(); surf.wrap_mask = 0x0180u; surf.wrap_value = 0x0080u; P.surface = &surf;
    make_line(); (void)tube_line();
    const int32_t round = (int32_t)(((int64_t)U(100) * 411775) >> 16), h = round >> 1;
    for(uint32_t i = 20; i < 60; ++i) nodes[i].lateral = (int16_t)(-(h >> 8) + 10);   /* just right of the seam */
    lone(U(200 * 25), round - h - U(20), 2000);                  /* just left of it */
    int crossed = 0, through_middle = 0;
    for(uint32_t t = 0; t < 40; ++t) {
        sh2z_field_step(&F);
        if(veh[0].t.lateral < 0) crossed = 1;
        if(abs(veh[0].t.lateral) < U(100)) through_middle = 1;
    }
    CHECK(crossed && !through_middle);
    make_line();
    /* Random races with tubes and level of detail: tube laterals stay within half the round, road
     * laterals within the half width, energies in range, every vehicle ranked once. */
    for(uint32_t seed = 1; seed <= 4; ++seed) {
        params(); surface_reset(); surf.wrap_mask = 0x0180u; surf.wrap_value = 0x0080u; P.surface = &surf;
        make_line(); (void)tube_line();
        P.lod_near[0] = U(1500); P.lod_near[1] = U(4000); P.lod_near[2] = U(8000);
        rng = seed * UINT32_C(2654435761) + 1u;
        for(uint32_t i = 0; i < 30; ++i) vehicle(i, i, 1500 + (int32_t)i * 20);
        for(uint32_t i = 4; i < 30; i += 5) { brain[i].chase = (int16_t)(i - 1u); brain[i].chase_lead = 30; brain[i].aggression = 60; }
        field(30, seed);
        for(uint32_t t = 0; t < 4000; ++t) {
            if(t == 60) sh2z_field_start(&F);
            sh2z_field_step(&F);
            uint32_t seen = 0;
            for(uint32_t i = 0; i < 30; ++i) {
                sh2z_line_sample_t at;
                sh2z_line_sample(&line, veh[i].t.node, veh[i].t.dist, &at);
                if(at.flags & 0x0080u) CHECK(in_round(veh[i].t.lateral, at.half_width));
                else CHECK(veh[i].t.lateral >= -at.half_width && veh[i].t.lateral <= at.half_width);
                CHECK(veh[i].energy >= 0 && veh[i].energy <= veh[i].max_energy);
                seen |= UINT32_C(1) << veh[i].rank;
            }
            CHECK(seen == (UINT32_C(1) << 30) - 1u);
        }
        CHECK(F.finished + F.retired > 0u);
    }
    make_line();
    params();
}

/* The final dash: from dash_from on the final lap an agent targets at least dash_pace of its pace and
 * boosts on energy. The laps before are unchanged; the control (dash_from 0) runs the last lap as before. */
static int32_t dash_race(int32_t from, int32_t *lap2) {
    params();
    P.laps = 3; P.dash_from = from; P.dash_pace = 78643;            /* 1.2 */
    rng = 7u;
    vehicle(0, 0, 1500);
    brain[0].lane = 0;
    field(1, 3u);
    for(uint32_t t = 0; t < 9000 && !(veh[0].flags & SH2Z_VEH_FINISHED); ++t) {
        if(t == 60) sh2z_field_start(&F);
        sh2z_field_step(&F);
    }
    CHECK(veh[0].flags & SH2Z_VEH_FINISHED);
    *lap2 = brain[0].lap_tick[1];
    return brain[0].finish_tick;
}
static void test_dash(void) {
    int32_t lap2_free, lap2_dash;
    const int32_t free_run = dash_race(0, &lap2_free);
    const int32_t dash = dash_race(line.length / 2, &lap2_dash);
    CHECK(lap2_dash == lap2_free);                                  /* nothing changes before the final lap */
    CHECK(dash < free_run - 100);                                   /* the last half lap is raced */
    params();
}

int main(void) {
    make_line();
    params();
    test_grid();
    test_races();
    test_pass();
    test_contact();
    test_contact_off();
    test_lod();
    test_retire();
    test_retarget();
    test_first_lap_cap();
    test_surface_identity();
    test_plate();
    test_zones();
    test_tubes();
    test_dash();
    printf("line field: %" PRIu64 " checks passed\n", checks);
    return 0;
}
