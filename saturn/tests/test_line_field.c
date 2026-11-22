/* Line field (contract tags line-field-1 to line-field-5 and line-field-7 to line-field-9) checks.
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
/* The step the races take (line-field-7 runs them through each entry point). */
static void (*field_step)(sh2z_field_t *) = sh2z_field_step;
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

/* line-field-8: pairs of stepped agents left inside each other after a step (a quarter of the contact box, so a
 * pair the contact passes resolved is never counted). */
static uint32_t inside;
static void count_inside(uint32_t count) {
    for(uint32_t a = 0; a < count; ++a) for(uint32_t b = a + 1u; b < count; ++b) {
        if((veh[a].flags | veh[b].flags) & (SH2Z_VEH_STOPPED | SH2Z_VEH_RETIRED | SH2Z_VEH_EXTERNAL)) continue;
        if(brain[a].last_tick != F.tick || brain[b].last_tick != F.tick) continue;
        const int32_t g = sh2z_track_near_gap(&line, &veh[a].t, &veh[b].t), dl = veh[a].t.lateral - veh[b].t.lateral;
        if(abs(g) < U(46) / 4 && abs(dl) < U(40) / 4) ++inside;   /* a fixed box, whatever the contact settings */
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
        field_step(&F);
        if(check) invariants(count, &before);
    }
    return sh2z_field_hash(&F);
}

static void test_races(void) {
    for(uint32_t seed = 1; seed <= 6; ++seed) {
        P.lod_near[0] = INT32_MAX; P.lod_near[1] = INT32_MAX; P.lod_near[2] = INT32_MAX;
        const uint32_t h1 = race(24, seed, 0, 5000, 1);
        /* line-field-1 results, pinned: with no finish targets line-field-2 races step for step as before.
         * line-field-8 orders again before the contact passes: seeds 2, 5 and 6 changed there (line-field-1 to
         * line-field-7 gave 0x9c2d952f, 0x080ae758 and 0x244b0332). line-field-9 drops three lateral holds: every
         * seed changed (line-field-8 gave 0x7c56497e, 0xb6a98eab, 0x12afee5b, 0xccfb9880, 0xe86c77ef, 0x36df557b). */
        static const uint32_t lf1[6] = { 0x26258cb9u, 0xae50d21fu, 0x9821ea57u, 0xc1fc223fu, 0xe9d006cfu, 0xa428c63bu };
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

/* line-field-8: an agent that passes another within one step still meets the vehicle it lands on. A (frozen) leads,
 * B (frozen, wide) sits 50 behind it, C closes at 58 a step from 10 behind B and lands 2 behind A on its lateral, and
 * D (frozen) is far behind. In the step's starting order A, B, C, D, a scan from A ends at B and a scan from C at D,
 * so only an order taken after the movement checks A against C. Control: without contact, C stays inside A. */
static uint32_t overtake_scene(int contact) {
    params();
    if(!contact) P.contact_long = 0;
    rng = 11u;
    for(uint32_t i = 0; i < 4u; ++i) {
        vehicle(i, i, 2000);
        veh[i].accel = veh[i].brake = 0; veh[i].grip = 0; brain[i].pace = 0; brain[i].lane = 0;
    }
    static const int32_t at[4] = { 5000, 4950, 4940, 4000 }, lat[4] = { 0, 150, 0, 0 };
    for(uint32_t i = 0; i < 4u; ++i) {
        veh[i].t.dist = U(at[i]); veh[i].t.lateral = U(lat[i]); veh[i].t.speed = 0;
        veh[i].t.node = sh2z_line_locate(&line, veh[i].t.dist, 0);
    }
    veh[2].t.speed = U(58);
    field(4, 1u);
    sh2z_field_start(&F);
    inside = 0u;
    sh2z_field_step(&F);
    count_inside(4u);
    CHECK(sh2z_track_near_gap(&line, &veh[0].t, &veh[2].t) < P.contact_long || !contact);   /* C ends next to A */
    CHECK(sh2z_track_near_gap(&line, &veh[2].t, &veh[1].t) > 0);                           /* and ahead of B */
    return F.contacts;
}
static void test_overtake_contact(void) {
    CHECK(overtake_scene(1) == 1u && inside == 0u);                 /* the pair is found and pushed apart */
    CHECK(abs(veh[0].t.lateral - veh[2].t.lateral) >= P.contact_lat);
    CHECK(overtake_scene(0) == 0u && inside == 1u);                 /* control: unresolved, it is inside */
    params();
}

/* line-field-10: the contact passes of a step share the line samples and, while nobody retires, the list of pairs in
 * reach. A pile of five frozen agents stacked on one spot: the first pass pushes several pairs, the second pass runs, and
 * a vehicle that retires in the first pass (its energy is nearly spent) is left out of the later passes, as a scan of the
 * order leaves it out. `LF10_PRINT` prints the scene's result for recording it against another build. */
static uint64_t pile_scene(uint32_t low_energy_mask, int print) {
    params();
    rng = 21u;
    static const int32_t at[5] = { 5000, 4996, 4992, 4988, 4984 }, lat[5] = { 0, 12, -14, 22, -6 };
    for(uint32_t i = 0; i < 5u; ++i) {
        vehicle(i, i, 2000);
        veh[i].accel = veh[i].brake = 0; veh[i].grip = 0; brain[i].pace = 0; brain[i].lane = 0;
        veh[i].t.dist = U(at[i]); veh[i].t.lateral = U(lat[i]); veh[i].t.speed = U(20 + (int32_t)i * 3);
        veh[i].t.node = sh2z_line_locate(&line, veh[i].t.dist, 0);
        if(low_energy_mask & (UINT32_C(1) << i)) veh[i].energy = U(3);
    }
    field(5, 1u);
    sh2z_field_start(&F);
    sh2z_field_step(&F);
    uint64_t h = 1469598103934665603ull;
    for(uint32_t i = 0; i < 5u; ++i) {
        const int32_t w[5] = { veh[i].t.lateral, veh[i].t.speed, veh[i].energy, (int32_t)veh[i].flags, 0 };
        for(int k = 0; k < 4; ++k) h = (h ^ (uint32_t)w[k]) * 1099511628211ull;
        if(print) printf("pile %u: lat %d speed %d energy %d flags %x\n", i, w[0], w[1], w[2], (unsigned)w[3]);
    }
    h = (h ^ F.contacts) * 1099511628211ull; h = (h ^ F.retired) * 1099511628211ull;
    if(print) printf("pile: contacts %u retired %u hash %llx\n", (unsigned)F.contacts, (unsigned)F.retired, (unsigned long long)h);
    params();
    return h;
}

/* Many random piles (4..8 agents within 60 of each other, low energy so that some retire in the first or a later pass):
 * one hash over every agent's lateral, speed, energy and flags after a started step, and the field's counters. */
static uint64_t pile_fuzz(uint32_t scenes) {
    uint64_t h = 1469598103934665603ull;
    for(uint32_t sc = 1; sc <= scenes; ++sc) {
        params();
        rng = sc * 7919u + 3u;
        const uint32_t n = (sc % 11u == 0u) ? 28u + rnd() % 5u : 4u + rnd() % 7u;   /* every eleventh pile overflows the pair list */
        const int32_t base = (sc % 5u == 0u) ? U(8) : U(5000);   /* every fifth pile straddles the line's start */
        const uint32_t spread = (sc % 3u == 0u || n > 10u) ? (uint32_t)U(25) : (uint32_t)U(60);
        for(uint32_t i = 0; i < n; ++i) {
            vehicle(i, i, 2000);
            veh[i].accel = veh[i].brake = 0; veh[i].grip = 0; brain[i].pace = 0; brain[i].lane = 0;
            int32_t d = base - (int32_t)(rnd() % spread);
            if(d < 0) d += line.length;
            veh[i].t.dist = d; veh[i].t.lateral = (int32_t)(rnd() % (uint32_t)U(120)) - U(60);
            veh[i].t.speed = U(20) + (int32_t)(rnd() % (uint32_t)U(20));
            veh[i].t.node = sh2z_line_locate(&line, veh[i].t.dist, 0);
            veh[i].energy = U(2) + (int32_t)(rnd() % (uint32_t)U(12));
            if(i && rnd() % 9u == 0u) veh[i].flags |= SH2Z_VEH_EXTERNAL;
        }
        field(n, 1u);
        sh2z_field_start(&F);
        sh2z_field_step(&F);
        for(uint32_t i = 0; i < n; ++i) {
            const int32_t w[6] = { veh[i].t.lateral, veh[i].t.speed, veh[i].energy, (int32_t)veh[i].flags, veh[i].kick_lat, veh[i].kick_speed };
            for(int k = 0; k < 6; ++k) h = (h ^ (uint32_t)w[k]) * 1099511628211ull;
        }
        h = (h ^ F.contacts) * 1099511628211ull; h = (h ^ F.retired) * 1099511628211ull;
    }
    params();
    return h;
}
#define PILE_FUZZ_HASH UINT64_C(0xed014f8596b4f52d)   /* recorded from the line-field-9 step */
static void test_pile_passes(void) {
    /* The results are those of the line-field-9 step (recorded from it): every pass scans the order in full there. */
    CHECK(pile_scene(0u, 0) == UINT64_C(0x44dbf3bf4ac40a53));     /* nobody retires: the later passes walk the list */
    CHECK(pile_scene(2u, 0) == UINT64_C(0x73952fec64b830e2));     /* one retires in the first pass: later passes leave it out */
    CHECK(pile_scene(6u, 0) == UINT64_C(0xbe7725040c9854c7));     /* two retire */
    CHECK(pile_scene(31u, 0) == UINT64_C(0xd15443e4a3aa6366));    /* all five retire (the scan finds nobody) */
    CHECK(pile_scene(0u, 0) != pile_scene(2u, 0));                /* control: a retirement changes the result */
    CHECK(pile_fuzz(3000u) == PILE_FUZZ_HASH);                    /* retirements in the first and in a later pass */
    CHECK(pile_fuzz(2999u) != PILE_FUZZ_HASH);                    /* control: one scene fewer is another hash */
}

/* line-field-9 scenes: frozen agents (no speed or lateral change of their own), lane 0, placed relative to their
 * racing line at dist `at`. Returns that line's lateral there. */
static int32_t frozen_pair(int32_t at, int32_t lat0, int32_t speed0, int32_t gap1, int32_t lat1, int32_t speed1) {
    rng = 13u;
    vehicle(0, 0, 2000); vehicle(1, 1, 2000);
    sh2z_line_sample_t s;
    sh2z_line_sample(&line, sh2z_line_locate(&line, at, 0), at, &s);
    for(uint32_t i = 0; i < 2u; ++i) {
        veh[i].accel = veh[i].brake = 0; veh[i].grip = 0; brain[i].pace = 0; brain[i].lane = 0;
        veh[i].t.dist = i ? at + gap1 : at; veh[i].t.lateral = s.lateral + (i ? lat1 : lat0);
        veh[i].t.speed = i ? speed1 : speed0; veh[i].t.node = sh2z_line_locate(&line, veh[i].t.dist, 0);
    }
    field(2, 1u);
    sh2z_field_start(&F);
    return s.lateral;
}
/* Four steps: each agent decides once (decisions are staggered over four ticks). */
static void think_steps(void) { for(uint32_t t = 0; t < 4u; ++t) sh2z_field_step(&F); }

/* line-field-9: no post-contact room hold. A pair 20 apart and 20 across touches: both are pushed and charged once,
 * and neither is left holding a side (the reference contact is a push and an impact only). Control: the same pair
 * with avoidance on shows the probe sees a held side (the trailing agent picks one around the agent ahead). */
static uint32_t room_scene(int avoid) {
    params();
    if(!avoid) { P.avoid_long = 0; P.avoid_ticks = 0; }
    (void)frozen_pair(U(5000), U(10), U(30), -U(20), -U(10), U(34));
    think_steps();
    CHECK(F.contacts == 1u);
    const uint32_t held = (uint32_t)(brain[0].pass_ticks != 0u) + (uint32_t)(brain[1].pass_ticks != 0u);
    CHECK(held == (uint32_t)(brain[0].pass_dir != 0) + (uint32_t)(brain[1].pass_dir != 0));
    params();
    return held;
}
static void test_no_room_hold(void) {
    CHECK(room_scene(0) == 0u);
    CHECK(room_scene(1) == 1u);    /* control: the avoiding agent holds its side */
}

/* line-field-9: no hold against cutting across. The agent sits 100 left of its line with a neighbour 50 left of the
 * line, 10 ahead or 10 behind: its target is its line, whether or not the neighbour is in between (the reference
 * steers to the side it chose; contact settles any overlap). Control: with avoidance on, the neighbour ahead moves
 * the target off the line (the probe sees a lateral decision). */
static int32_t cut_scene(int32_t gap1, int32_t lat1, int avoid, int32_t *line_lat) {
    params();
    if(!avoid) { P.avoid_long = 0; P.avoid_ticks = 0; }
    *line_lat = frozen_pair(U(5000), -U(100), 0, gap1, lat1, 0);
    think_steps();
    CHECK(F.contacts == 0u);
    params();
    return brain[0].target_lat;
}
static void test_no_cut_hold(void) {
    int32_t base;
    CHECK(cut_scene(U(10), -U(50), 0, &base) == base);      /* neighbour ahead, in between */
    CHECK(cut_scene(-U(10), -U(50), 0, &base) == base);     /* neighbour behind, in between */
    CHECK(cut_scene(U(10), -U(150), 0, &base) == base);     /* neighbour ahead, outside */
    CHECK(cut_scene(U(10), -U(50), 1, &base) != base);      /* control: avoiding the neighbour ahead */
}

/* line-field-9: no blocking hold. The agent stands on its line with a stale target 80 off it, and an agent 80
 * behind on the same line closes at 4 a tick: the target is its line again (the reference reads the vehicle behind
 * only for a side attack). Control: with a pass side held, the stale target stands (the probe sees a held target). */
static int32_t block_scene(int32_t speed1, int hold, int32_t *line_lat) {
    params();
    *line_lat = frozen_pair(U(5000), 0, 0, -U(80), 0, speed1);   /* the agent stands; the one behind closes slowly */
    brain[0].target_lat = *line_lat + U(80);
    if(hold) { brain[0].pass_dir = 1; brain[0].pass_ticks = 10u; }
    think_steps();
    CHECK(F.contacts == 0u);
    params();
    return brain[0].target_lat;
}
static void test_no_block_hold(void) {
    int32_t base;
    CHECK(block_scene(U(4), 0, &base) == base);             /* faster agent close behind */
    CHECK(block_scene(0, 0, &base) == base);                /* agent behind not faster */
    CHECK(block_scene(U(4), 1, &base) == base + U(80));     /* control: a held pass side keeps the stale target */
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
    static const uint32_t lf1[6] = { 0x26258cb9u, 0xae50d21fu, 0x9821ea57u, 0xc1fc223fu, 0xe9d006cfu, 0xa428c63bu };
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

/* ---- line-field-5 ---------------------------------------------------------------------------------- */
static sh2z_field_behave_t B;
/* Every behaviour off but the ones a scene sets. */
static void behave_reset(void) {
    memset(&B, 0, sizeof B);
    B.roll_min = 20; B.roll_span = 16; B.fall_ticks = 95; B.ko_floor = 26;
}
/* Nodes lo..hi-1 get flags and a half width (and the line's lateral 0). */
static void mark(uint32_t lo, uint32_t hi, uint16_t flags, uint16_t hw) {
    for(uint32_t i = lo; i < hi; ++i) { nodes[i].flags = flags; nodes[i].half_width = hw; nodes[i].lateral = 0; }
}

/* A behaviour block whose tubes and air no node is moves nobody differently: the pinned line-field-1 hashes
 * of the line-field-4 state (the behaviour state hashes only with a block). */
static void test_behave_identity(void) {
    static const uint32_t lf1[6] = { 0x26258cb9u, 0xae50d21fu, 0x9821ea57u, 0xc1fc223fu, 0xe9d006cfu, 0xa428c63bu };
    params(); behave_reset();
    B.roll_chance = 65535; B.roll_lead = 13107;
    B.air_mask = 0x01c0u; B.air_value = 0x0180u; B.open_mask = 0x01c0u; B.open_value = 0x0100u; B.air_spread = 65535;
    P.behave = &B;
    surface_reset(); surf.wrap_mask = 0x0180u; surf.wrap_value = 0x0080u; P.surface = &surf;
    for(uint32_t seed = 1; seed <= 6; ++seed) {
        (void)race(24, seed, 0, 5000, 1);
        P.behave = 0;                                               /* the line-field-4 state alone */
        CHECK(sh2z_field_hash(&F) == lf1[seed - 1u]);
        CHECK(F.falls == 0u && F.kos == 0u);
        P.behave = &B;
    }
    params();
}

/* Roll: on a hold tube (radius 100, nodes 10..89) one agent that always rolls goes round and round;
 * not rolling it keeps its angle (hold_lead 0); the control without the hold mask steers home. */
static int32_t roll_scene(int32_t chance, int hold, int32_t start_lat, int32_t *travel) {
    params(); behave_reset(); surface_reset();
    surf.wrap_mask = 0x0180u; surf.wrap_value = 0x0080u; P.surface = &surf;
    B.roll_chance = chance; B.roll_latch = 65536; B.roll_lead = 131072;
    if(hold) { B.hold_mask = 0x01c0u; B.hold_value = 0x00c0u; }
    P.behave = &B; P.boost_from_lap = 4;
    make_line(); mark(10, 90, 0x00c0u, 100);
    lone(U(200 * 11), start_lat, 3000);
    int32_t last = veh[0].t.lateral; *travel = 0;
    for(uint32_t t = 0; t < 1000; ++t) {
        sh2z_field_step(&F);
        sh2z_line_sample_t at; sh2z_line_sample(&line, veh[0].t.node, veh[0].t.dist, &at);
        if(!(at.flags & 0x0080u)) break;
        const int32_t c = (int32_t)(((int64_t)at.half_width * 411775) >> 16);
        int32_t d = veh[0].t.lateral - last;
        while(d >= c / 2) d -= c;
        while(d < -c / 2) d += c;
        *travel += d; last = veh[0].t.lateral;
    }
    const int32_t end = veh[0].t.lateral;
    make_line(); params();
    return end;
}
static void test_roll(void) {
    int32_t travel;
    (void)roll_scene(65536, 1, 0, &travel);
    CHECK(travel > U(2 * 628));                                     /* more than twice round (radius 100) */
    const int32_t held = roll_scene(0, 1, U(120), &travel);
    CHECK(held == U(120) && travel == 0);                           /* not rolling: keeps its angle */
    const int32_t home = roll_scene(0, 0, U(120), &travel);
    CHECK(abs(home) < U(10));                                       /* control: no hold mask, back to its line */
    (void)roll_scene(0, 1, 0, &travel);
    CHECK(travel == 0);                                             /* control: no rolls, no travel */
}

/* A held roll ends in the exit stretch of an exit tube (pipe 0x80, nodes 10..49, look-ahead 600);
 * the control (no exit mask) keeps it to the tube's end. */
static int exit_scene(int exit_mask) {
    params(); behave_reset(); surface_reset();
    surf.wrap_mask = 0x0180u; surf.wrap_value = 0x0080u; P.surface = &surf;
    B.roll_chance = 65536; B.roll_latch = 65536; B.roll_lead = 6554; B.roll_exit_ahead = U(600);
    if(exit_mask) { B.exit_mask = 0x01c0u; B.exit_value = 0x0080u; }
    P.behave = &B; P.boost_from_lap = 4;
    make_line(); mark(10, 50, 0x0080u, 100);
    lone(U(200 * 11), 0, 3000);
    int ended = 0, rolled = 0;
    for(uint32_t t = 0; t < 2000 && veh[0].t.dist < U(200 * 50); ++t) {
        sh2z_field_step(&F);
        if(brain[0].roll < 0) rolled = 1;
        if(rolled && !ended && brain[0].roll == 0) {
            ended = 1;
            CHECK(veh[0].t.dist + U(600) + U(4 * 60) >= U(200 * 50));   /* within the look-ahead (a decision's travel) */
        }
    }
    make_line(); params();
    return rolled && ended;
}
static void test_roll_exit(void) {
    CHECK(exit_scene(1) == 1);
    CHECK(exit_scene(0) == 0);                                      /* control: no exit stretch, the roll holds */
}

/* Lateral inertia: the lateral velocity changes by at most lat_accel a tick and an agent sent across
 * overshoots its target; the control (lat_accel 0, line-field-4 steering) does not. */
static int32_t inertia_scene(int32_t accel) {
    params(); behave_reset(); B.lat_accel = accel; P.behave = &B; P.boost_from_lap = 4;
    make_line(); mark(0, NODES, 0x0000u, 300);
    lone(U(200 * 3), -U(150), 3000);
    brain[0].lane = U(150);
    int32_t most = INT32_MIN, v0 = veh[0].t.lat_vel;
    for(uint32_t t = 0; t < 400; ++t) {
        sh2z_field_step(&F);
        if(accel) CHECK(abs(veh[0].t.lat_vel - v0) <= accel);
        v0 = veh[0].t.lat_vel;
        if(veh[0].t.lateral - U(150) > most) most = veh[0].t.lateral - U(150);
    }
    make_line(); params();
    return most;
}
static void test_inertia(void) {
    CHECK(inertia_scene(48) > U(4));                                /* overshoots */
    CHECK(inertia_scene(0) <= 0);                                   /* control */
}

/* line-field-7: an agent under lateral inertia still keeps the line's speed cap and its own accel and brake (its speed
 * and distance come from the step's sample). Control: without the cap it runs faster. */
static int32_t inertia_cap_scene(uint16_t cap_q4, int32_t *slowest) {
    params(); behave_reset(); B.lat_accel = 48; P.behave = &B;
    make_line();
    for(uint32_t i = 40; i < 60; ++i) nodes[i].speed_cap = cap_q4;
    lone(U(200 * 10), 0, 400);
    veh[0].t.speed = U(50);
    int32_t fastest = 0;
    *slowest = INT32_MAX;
    for(uint32_t t = 0; t < 400; ++t) {
        sh2z_field_step(&F);
        const uint32_t n = veh[0].t.node;
        if(n >= 53u && n < 59u) {
            if(veh[0].t.speed > fastest) fastest = veh[0].t.speed;
            if(veh[0].t.speed < *slowest) *slowest = veh[0].t.speed;
        }
    }
    params(); make_line();
    return fastest;
}
static void test_inertia_cap(void) {
    int32_t slow;
    CHECK(inertia_cap_scene(30 * 16, &slow) <= U(30) && slow > U(20));   /* held to the cap, braking at its rate */
    CHECK(inertia_cap_scene(60 * 16, &slow) > U(30));                    /* control: no cap */
}

/* Air and falls: nodes 0..29 walled road (0x40), 30..39 air (0x180), 40.. open road (0x000) of half width 60.
 * An agent leaving the air beyond 60 + margin falls: out of contact, no laps, energy kept until it retires
 * exactly fall_ticks later with energy 0. The control (no open mask) lands and stays on the line. */
static int fall_scene(int open, int32_t *retire_at, int32_t *fall_at) {
    params(); behave_reset();
    B.air_mask = 0x01c0u; B.air_value = 0x0180u; B.fall_margin = U(10); B.fall_ticks = 95;
    if(open) { B.open_mask = 0x01c0u; B.open_value = 0x0000u; }
    P.behave = &B; P.boost_from_lap = 4;
    make_line(); mark(0, 30, 0x0040u, 200); mark(30, 40, 0x0180u, 200); mark(40, NODES, 0x0000u, 60);
    lone(U(200 * 25), U(150), 3000);
    brain[0].lane = U(150);
    int fell = 0;
    for(uint32_t t = 0; t < 900; ++t) {
        const uint16_t was = veh[0].flags;
        sh2z_field_step(&F);
        if((veh[0].flags & SH2Z_VEH_FALLING) && !(was & SH2Z_VEH_FALLING)) { fell = 1; CHECK(F.falls == 1u); }
        if(veh[0].flags & SH2Z_VEH_FALLING) CHECK(veh[0].energy > 0 && brain[0].laps_seen == 0);
        if((veh[0].flags & SH2Z_VEH_RETIRED) && !(was & SH2Z_VEH_RETIRED)) CHECK(veh[0].energy == 0);
    }
    *retire_at = brain[0].retire_tick; *fall_at = brain[0].fall_tick;
    make_line(); params();
    return fell;
}
static void test_falls(void) {
    int32_t r, f;
    CHECK(fall_scene(1, &r, &f) == 1);
    CHECK(f >= 0 && r == f + 95);                                   /* retires exactly fall_ticks later */
    CHECK(fall_scene(0, &r, &f) == 0 && r < 0 && f < 0);            /* control: no open edge, no fall */
}

/* Knock-out credit at the fall, the source predicate: credited when the age after the falling step's count
 * is at most ko_window (120), once. The age is set before the step that falls, as the reference probe does. */
static uint32_t credit_scene(int32_t age_before, int attacker_done, int32_t window) {
    params(); behave_reset();
    B.air_mask = 0x01c0u; B.air_value = 0x0180u; B.open_mask = 0x01c0u; B.open_value = 0x0000u;
    B.fall_margin = U(10); B.fall_ticks = 95; B.ko_window = window;
    P.behave = &B; P.boost_from_lap = 4;
    make_line(); mark(0, 30, 0x0040u, 200); mark(30, 40, 0x0180u, 200); mark(40, NODES, 0x0000u, 60);
    rng = 5u;
    vehicle(0, 0, 3000); vehicle(1, 0, 3000);
    veh[1].flags = SH2Z_VEH_EXTERNAL; veh[1].t.dist = U(10); veh[1].t.lap = 0; veh[1].t.lateral = 0; veh[1].t.node = 0;
    if(attacker_done) veh[1].flags |= SH2Z_VEH_FINISHED;
    brain[0].lane = U(150);
    veh[0].t.dist = U(200 * 25); veh[0].t.lap = 0; veh[0].t.lateral = U(150); veh[0].t.speed = U(20);
    veh[0].t.node = sh2z_line_locate(&line, veh[0].t.dist, 0);
    field(2, 3u); sh2z_field_start(&F);
    uint32_t kos_at_fall = 0u;
    int found = 0;
    static sh2z_vehicle_t sv[2]; static sh2z_field_brain_t sb[2]; static sh2z_field_t sf; static uint8_t so[2], ss[2], sr[2];
    for(uint32_t t = 0; t < 600 && !found; ++t) {
        /* Step a copy; on the step that falls, restore and set the age before it. */
        memcpy(sv, veh, sizeof sv); memcpy(sb, brain, sizeof sb); sf = F; memcpy(so, order, 2); memcpy(ss, slot, 2); memcpy(sr, rank_order, 2);
        sh2z_field_step(&F);
        if(!(veh[0].flags & SH2Z_VEH_FALLING)) continue;
        found = 1;
        memcpy(veh, sv, sizeof sv); memcpy(brain, sb, sizeof sb); F = sf; memcpy(order, so, 2); memcpy(slot, ss, 2); memcpy(rank_order, sr, 2);
        brain[0].ko_by = 1; brain[0].ko_age = (uint16_t)age_before;
        sh2z_field_step(&F);
        CHECK(veh[0].flags & SH2Z_VEH_FALLING);
        CHECK(brain[0].ko_age == age_before + 1);
        kos_at_fall = F.kos;
        for(uint32_t k = 0; k < 200; ++k) sh2z_field_step(&F);
        CHECK(F.kos == kos_at_fall);                                /* once */
        CHECK(veh[0].energy == 0 && (veh[0].flags & SH2Z_VEH_RETIRED));
        CHECK(brain[0].ko_credit == (kos_at_fall ? 1 : -1));
    }
    CHECK(found);
    make_line(); params();
    return kos_at_fall;
}
static void test_ko_credit(void) {
    CHECK(credit_scene(118, 0, 120) == 1u);                        /* age 119 */
    CHECK(credit_scene(119, 0, 120) == 1u);                        /* age 120: the last eligible */
    CHECK(credit_scene(120, 0, 120) == 0u);                        /* age 121: the first ineligible */
    CHECK(credit_scene(10, 1, 120) == 0u);                         /* the attacker has finished */
    CHECK(credit_scene(10, 0, 0) == 0u);                           /* control: no window, no credit */
}

/* Near-focus gate: an agent emptied by contact far from the focus keeps ko_floor; near it, or with no gate
 * (the control), it retires with energy 0. */
static void test_ko_near(void) {
    for(int far = 0; far < 2; ++far) for(int gate = 0; gate < 2; ++gate) {
        params(); behave_reset(); B.ko_near = gate ? U(1000) : 0; P.behave = &B; P.boost_from_lap = 4;
        P.side_energy = U(500);                                     /* one hit empties anyone */
        rng = 5u;
        vehicle(0, 0, 3000); vehicle(1, 0, 3000); vehicle(2, 0, 3000);
        const int32_t at = far ? U(200 * 40) : U(200 * 2);
        veh[0].t.dist = U(200 * 2) - U(30); veh[0].t.lateral = U(100);   /* the focus, beside them */
        veh[1].t.dist = at; veh[1].t.lateral = 0; veh[2].t.dist = at + U(20); veh[2].t.lateral = U(5);
        for(int i = 0; i < 3; ++i) { veh[i].t.lap = 0; veh[i].t.speed = U(10); veh[i].t.node = sh2z_line_locate(&line, veh[i].t.dist, 0); brain[i].lane = 0; }
        field(3, 3u); F.focus = 0; sh2z_field_start(&F);
        for(uint32_t t = 0; t < 3; ++t) sh2z_field_step(&F);
        const int retired = (veh[1].flags & SH2Z_VEH_RETIRED) != 0;
        if(far && gate) CHECK(!retired && veh[1].energy == 26);    /* far and gated: the floor */
        else CHECK(retired && veh[1].energy == 0);                  /* near, or no gate (control) */
    }
    params();
}

/* Drops off an outside tube: a held agent round a drop cylinder (radius 100) beyond drop_lat drops off with
 * drop_chance; controls: not beyond drop_lat, no drop flag, a pipe (not a hold tube), drop_chance 0. */
static int drop_scene(uint16_t flags, int32_t start_lat, int32_t chance) {
    params(); behave_reset(); surface_reset();
    surf.wrap_mask = 0x0180u; surf.wrap_value = 0x0080u; P.surface = &surf;
    B.hold_mask = 0x01c0u; B.hold_value = 0x00c0u;
    B.drop_mask = 0x0200u; B.drop_value = 0x0200u; B.drop_lat = 117965; B.drop_chance = chance;
    P.behave = &B; P.boost_from_lap = 4;
    make_line(); mark(10, 90, flags, 100);
    lone(U(200 * 11), start_lat, 3000);
    brain[0].lane = start_lat;
    int fell = 0;
    for(uint32_t t = 0; t < 300 && !fell; ++t) { sh2z_field_step(&F); fell = (veh[0].flags & SH2Z_VEH_FALLING) != 0; }
    if(fell) CHECK(F.falls == 1u && brain[0].fall_tick >= 0);
    make_line(); params();
    return fell;
}
static void test_drops(void) {
    CHECK(drop_scene(0x02c0u, U(200), 65536) == 1);                 /* 2.0 of the radius round: drops off */
    CHECK(drop_scene(0x02c0u, U(150), 65536) == 0);                 /* control: 1.5, inside drop_lat */
    CHECK(drop_scene(0x00c0u, U(200), 65536) == 0);                 /* control: not a drop node */
    CHECK(drop_scene(0x0280u, U(200), 65536) == 0);                 /* control: a pipe (inside surface) */
    CHECK(drop_scene(0x02c0u, U(200), 0) == 0);                     /* control: no chance */
    CHECK(drop_scene(0x02c0u, U(200), 1) == 0);                     /* 1 / 65536 a step: not within 300 steps here */
}

/* Launch out of a pipe (radius 100, nodes 10..39) onto an open drop road (half width 60): beyond the edge plus
 * fall_margin the agent leaves the line with launch_chance; controls: launch_chance 0, no drop flag, inside the edge. */
static int launch_scene(uint16_t road, int32_t lane, int32_t chance) {
    params(); behave_reset(); surface_reset();
    surf.wrap_mask = 0x0180u; surf.wrap_value = 0x0080u; P.surface = &surf;
    B.open_mask = 0x01c0u; B.open_value = 0x0000u; B.fall_margin = U(10);
    B.drop_mask = 0x0200u; B.drop_value = 0x0200u; B.launch_chance = chance;
    P.behave = &B; P.boost_from_lap = 4;
    make_line(); mark(10, 40, 0x0080u, 100); mark(40, NODES, road, 60);
    lone(U(200 * 30), lane, 3000);
    brain[0].lane = lane;
    int fell = 0;
    for(uint32_t t = 0; t < 600 && !fell; ++t) { sh2z_field_step(&F); fell = (veh[0].flags & SH2Z_VEH_FALLING) != 0; }
    if(!fell) CHECK(abs(veh[0].t.lateral) <= U(60));                /* landed on the road */
    make_line(); params();
    return fell;
}
static void test_launch(void) {
    CHECK(launch_scene(0x0200u, U(150), 65536) == 1);
    CHECK(launch_scene(0x0200u, U(150), 0) == 0);                   /* control: no chance */
    CHECK(launch_scene(0x0000u, U(150), 65536) == 0);               /* control: not a drop node */
    CHECK(launch_scene(0x0200u, U(40), 65536) == 0);                /* control: inside the edge */
}

/* Impact damage (contact_norm): both pay contact_norm of the closing speed along the line between them, no flat
 * side energy; an attacker's victim pays twice and the attacker nothing. Control: contact_norm 0 charges side_energy. */
static void impact_scene(int32_t norm, int attack, int32_t *l0, int32_t *l1) {
    params(); behave_reset(); B.contact_norm = norm; P.behave = &B;
    P.side_energy = U(500); P.attack_energy = U(500);
    rng = 9u;
    vehicle(0, 0, 2000); vehicle(1, 1, 2000);
    veh[0].t.dist = U(5000); veh[0].t.lateral = U(10); veh[0].t.speed = U(30);
    veh[1].t.dist = U(5000) - U(20); veh[1].t.lateral = -U(10); veh[1].t.speed = U(34);
    veh[0].t.node = sh2z_line_locate(&line, veh[0].t.dist, 0); veh[1].t.node = sh2z_line_locate(&line, veh[1].t.dist, 0);
    field(2, 1u);
    sh2z_field_start(&F);
    veh[0].accel = veh[1].accel = 0; veh[0].brake = veh[1].brake = 0; veh[0].grip = veh[1].grip = 0;
    brain[0].pace = brain[1].pace = 0;
    if(attack) { brain[1].attack_ticks = 30; brain[1].attack_target = 0; }
    const int32_t e0 = veh[0].energy, e1 = veh[1].energy;
    sh2z_field_step(&F);
    CHECK(F.contacts == 1u);
    *l0 = e0 - veh[0].energy; *l1 = e1 - veh[1].energy;
    params();
}
static void test_impact(void) {
    int32_t a, b;
    impact_scene(36045, 0, &a, &b);
    CHECK(a == b && a > 0 && a < U(4));                             /* 0.55 of a closing speed under 4: no flat term */
    const int32_t share = a;
    impact_scene(36045, 1, &a, &b);
    CHECK(b == 0 && a >= 2 * share - 1 && a <= 2 * share + 1);      /* the victim pays the whole, the attacker nothing */
    impact_scene(0, 0, &a, &b);
    CHECK(a > U(50) && b > U(50));                                  /* control: the flat side energy empties both */
}

/* line-field-7: the contact test samples only tube nodes for the seam; a pair either side of a tube's seam still
 * touches (its lateral difference wraps). Control: the same pair on a node that is not a tube does not. */
static uint32_t seam_scene(uint16_t flags) {
    params(); surface_reset(); surf.wrap_mask = 0x0180u; surf.wrap_value = 0x0080u; P.surface = &surf;
    make_line(); mark(20, 60, flags, 100);
    const int32_t round = (int32_t)(((int64_t)U(100) * 411775) >> 16), h = round >> 1;
    rng = 5u;
    vehicle(0, 0, 2000); vehicle(1, 1, 2000);
    veh[0].t.dist = U(200 * 30); veh[0].t.lateral = round - h - U(10); veh[0].t.speed = U(30);
    veh[1].t.dist = U(200 * 30) - U(20); veh[1].t.lateral = -h + U(10); veh[1].t.speed = U(30);
    veh[0].t.node = sh2z_line_locate(&line, veh[0].t.dist, 0); veh[1].t.node = sh2z_line_locate(&line, veh[1].t.dist, 0);
    field(2, 1u);
    sh2z_field_start(&F);
    veh[0].accel = veh[1].accel = 0; veh[0].brake = veh[1].brake = 0; veh[0].grip = veh[1].grip = 0;
    brain[0].pace = brain[1].pace = 0;
    sh2z_field_step(&F);
    const uint32_t c = F.contacts;
    params(); make_line();
    return c;
}
static void test_seam_contact(void) {
    CHECK(seam_scene(0x0080u) == 1u);                                /* across the seam of a tube */
    CHECK(seam_scene(0x0000u) == 0u);                                /* control: no tube, no seam */
}

/* line-field-7: sh2z_line_locate_far gives sh2z_line_locate's node from every hint: on the even line, on a line of
 * uneven spans with zero-length nodes, and on a one-node line; at every node start and either side of it, below the
 * start and past the length, and for look-aheads of 3000 units from every node (crossing the start line). Control:
 * a search that takes the first of equal starts differs on the zero-length nodes. */
static uint32_t locate_first(const sh2z_line_t *l, int32_t dist) {   /* control */
    uint32_t h = sh2z_line_locate_far(l, dist);
    while(h > 0u && l->nodes[h - 1u].start == l->nodes[h].start) --h;
    return h;
}
static uint32_t locate_check(const sh2z_line_t *l) {
    uint32_t differ = 0u;
    for(uint32_t k = 0; k <= l->count; ++k) {
        const int32_t base = k < l->count ? l->nodes[k].start : l->length;
        for(int32_t e = -2; e <= 2; ++e) {
            const int32_t d = base + e;
            const uint32_t far = sh2z_line_locate_far(l, d);
            for(uint32_t h = 0; h < l->count + 2u; ++h) CHECK(sh2z_line_locate(l, d, h) == far);
            differ += locate_first(l, d) != far;
        }
    }
    for(uint32_t k = 0; k < l->count; ++k) {
        int32_t d = l->nodes[k].start + U(3000) + (int32_t)(rnd() % 256u);
        while(d >= l->length) d -= l->length;
        CHECK(sh2z_line_locate(l, d, k) == sh2z_line_locate_far(l, d));
    }
    for(uint32_t r = 0; r < 20000; ++r) {
        const int32_t d = (int32_t)(rnd() % (uint32_t)(l->length + 512)) - 256;
        CHECK(sh2z_line_locate(l, d, rnd() % l->count) == sh2z_line_locate_far(l, d));
    }
    return differ;
}
static void test_locate_far(void) {
    make_line();
    CHECK(locate_check(&line) == 0u);                                /* no equal starts: the control agrees */
    sh2z_line_node_t un[40];
    sh2z_line_t ul = { un, 40u, 0 };
    int32_t at = 0;
    for(uint32_t i = 0; i < 40u; ++i) {
        un[i] = nodes[i];
        un[i].start = at;
        at += (i % 7u == 3u) ? 0 : U(20 + (int32_t)(rnd() % 300u));   /* every 7th node has zero length */
    }
    ul.length = at + U(50);
    CHECK(locate_check(&ul) > 0u);                                   /* control: first-of-equal differs */
    sh2z_line_t one = { nodes, 1u, U(200) };
    CHECK(locate_check(&one) == 0u);
    make_line();
}

/* line-field-7: the entry points split by behaviour block. Each gives sh2z_field_step's state step for step:
 * the plain step the pinned line-field-1 hashes; the behaviour step, on a line with a hold tube, a drop tube, an
 * air stretch with an open edge and an outside driver, with every behaviour on, the hashes line-field-5's
 * sh2z_field_step gave (pinned from main 7f99eb7) and sh2z_field_step's own. Controls: one behaviour value
 * changed changes the race; the scene has falls and contact damage to tell apart. line-field-9 moved all six
 * behaviour pins (line-field-8 gave 0x68687a12, 0xbaccf237, 0xabc03fa3, 0x29f21133, 0x12e650c6, 0xbb9479cd). */
static void behave_scene(void) {
    params(); behave_reset(); surface_reset();
    surf.wrap_mask = 0x0180u; surf.wrap_value = 0x0080u; P.surface = &surf;
    make_line(); mark(10, 40, 0x00c0u, 100); mark(40, 46, 0x02c0u, 100); mark(60, 66, 0x0180u, 220); mark(66, 70, 0x0100u, 220);
    B.roll_chance = 2000; B.roll_latch = 30000; B.roll_lead = 13107; B.hold_lead = 7864; B.roll_well = 52429;
    B.hold_mask = 0x01c0u; B.hold_value = 0x00c0u; B.exit_mask = 0x01c0u; B.exit_value = 0x0080u; B.roll_exit_ahead = U(400);
    B.lat_accel = 64; B.air_mask = 0x01c0u; B.air_value = 0x0180u; B.air_spread = 6554;
    B.open_mask = 0x01c0u; B.open_value = 0x0100u; B.fall_margin = 0;
    B.ko_window = 120; B.ko_near = U(1000); B.contact_norm = 36045;
    B.drop_mask = 0x0200u; B.drop_value = 0x0200u; B.drop_lat = 117965; B.drop_chance = 4000; B.launch_chance = 4096;
    P.behave = &B;
}
#ifdef LF7_PIN_PRINT   /* pin mode: built against line-field-5's sources, which have one step; prints lf5[] */
#define sh2z_field_step_plain sh2z_field_step
#define sh2z_field_step_behave sh2z_field_step
#define LF5_PINS 0u
#else
#define LF5_PINS 0xc2a25804u, 0x47960c34u, 0xcdea24ddu, 0x864b559cu, 0x9c64a595u, 0x68fe14f3u
#endif
static void test_step_entries(void) {
    static const uint32_t lf1[6] = { 0x26258cb9u, 0xae50d21fu, 0x9821ea57u, 0xc1fc223fu, 0xe9d006cfu, 0xa428c63bu };
    static const uint32_t lf5[6] = { LF5_PINS };
    uint32_t falls = 0u;
    for(uint32_t seed = 1; seed <= 6; ++seed) {
        params(); make_line();
        field_step = sh2z_field_step_plain;
        CHECK(race(24, seed, 0, 5000, 1) == lf1[seed - 1u]);
        behave_scene();
        field_step = sh2z_field_step;
        const uint32_t h = race(30, seed, 1, 5000, 0);
        falls += F.falls;
#ifdef LF7_PIN_PRINT
        printf("lf5 pin seed %" PRIu32 ": 0x%08" PRIx32 "u\n", seed, h); (void)lf5;
#else
        CHECK(h == lf5[seed - 1u]);
#endif
        behave_scene();
        field_step = sh2z_field_step_behave;
        CHECK(race(30, seed, 1, 5000, 0) == h);
        behave_scene(); B.roll_chance = 0;                          /* control: one behaviour off */
        CHECK(race(30, seed, 1, 5000, 0) != h);
    }
    CHECK(falls > 0u);                                              /* the scene reaches the behaviour code */
    field_step = sh2z_field_step;
    make_line(); params();
}

int main(void) {
    make_line();
    params();
    test_grid();
    test_races();
    test_pass();
    test_contact();
    test_contact_off();
    test_overtake_contact();
    test_pile_passes();
    test_no_room_hold();
    test_no_cut_hold();
    test_no_block_hold();
    test_lod();
    test_retire();
    test_retarget();
    test_first_lap_cap();
    test_surface_identity();
    test_plate();
    test_zones();
    test_tubes();
    test_dash();
    test_behave_identity();
    test_roll();
    test_roll_exit();
    test_inertia();
    test_inertia_cap();
    test_falls();
    test_ko_credit();
    test_ko_near();
    test_drops();
    test_launch();
    test_impact();
    test_step_entries();
    test_locate_far();
    test_seam_contact();
    printf("line field: %" PRIu64 " checks passed\n", checks);
    return 0;
}
