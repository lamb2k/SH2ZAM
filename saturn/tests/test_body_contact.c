/* Body contact (contract tag body-contact-1) checks.
 * An independent double-precision transcription of the rule judges every
 * output of sh2z_bc_pair on random and edge-case pairs within the stated
 * bound; the field walk is judged against a brute-force walk over every pair
 * with its own history; contact history, credit and the walk's single visit
 * per pair are checked on scripted scenes. Every rule has a control: the
 * oracle with that rule changed must disagree with the library.
 *
 * Copyright (c) 2026 SH2ZAM Contributors. MIT License.
 * Inspired by SH4ZAM by Falco Girgis and SH4ZAM Contributors.
 */
#include "sh2zam/body_contact_experimental.h"
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static uint64_t checks;
#define CHECK(e) do { ++checks; if(!(e)) { fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #e); exit(EXIT_FAILURE); } } while(0)
#define Q16(x) ((int32_t)((x) * 65536.0))
#define BOUND 256.0  /* Q16 LSB: the contract's 2^-8 */

static uint32_t rng = UINT32_C(0x2545f491);
static uint32_t rnd(void) { rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5; return rng; }
static int32_t rnd_range(int32_t lo, int32_t hi) { return lo + (int32_t)(rnd() % (uint32_t)(hi - lo + 1)); }

static sh2z_bc_rules_t R;
static void rules(void) {
    R.reach_long = 92 * 256;           /* along units: Q8 in these tests */
    R.reach = Q16(46.0);
    R.near = Q16(0.01);
    R.impact_gain = Q16(1.1);
    R.attack_push = Q16(3.0);
    R.attack_min = Q16(3.0);
    R.weak_share = 21823;              /* 0.333 */
    R.heavy_gain = Q16(10.0);
    R.credit_window = 60u;
}

/* ---- Oracle: the rule in doubles, written from the contract text -------- */

typedef struct {
    double impact_gain, attack_push, attack_min, weak_share, heavy_gain, reach, near;
    int swap_shares, closing_sign, credit_window;
} orules_t;

static orules_t orules(void) {
    orules_t o;
    o.impact_gain = R.impact_gain / 65536.0; o.attack_push = R.attack_push / 65536.0;
    o.attack_min = R.attack_min / 65536.0; o.weak_share = R.weak_share / 65536.0;
    o.heavy_gain = R.heavy_gain / 65536.0; o.reach = R.reach / 65536.0; o.near = R.near / 65536.0;
    o.swap_shares = 0; o.closing_sign = 1; o.credit_window = (int)R.credit_window;
    return o;
}

typedef struct { double damage, push[3]; int hit; } oout_t;

/* Decisions taken on the same integers as the contract states. */
static uint32_t oracle(const sh2z_bc_body_t *a, const sh2z_bc_body_t *b, int before, const orules_t *o,
                       oout_t *oa, oout_t *ob) {
    int64_t dxi[3], d2 = 0, cl = 0;
    double dx[3], dv[3], n[3], d, wa, wb, sa, sb, len;
    int atk_a = (a->flags & SH2Z_BC_ATTACK) != 0, atk_b = (b->flags & SH2Z_BC_ATTACK) != 0;
    uint32_t ev = SH2Z_BC_EV_TOUCH;
    memset(oa, 0, sizeof *oa); memset(ob, 0, sizeof *ob);
    for(int i = 0; i < 3; i++) {
        dxi[i] = (int64_t)b->pos[i] - a->pos[i];
        if(dxi[i] > R.reach || dxi[i] < -R.reach) return 0u;
        d2 += dxi[i] * dxi[i];
    }
    if(d2 > (int64_t)R.reach * R.reach) return 0u;
    for(int i = 0; i < 3; i++) {
        dx[i] = (double)dxi[i] / 65536.0;
        dv[i] = ((double)b->vel[i] - a->vel[i]) / 65536.0;
        cl += dxi[i] * ((int64_t)b->vel[i] - a->vel[i]);
    }
    d = sqrt((double)d2) / 65536.0;
    const double d_round = floor(sqrt((double)d2) + 0.5);  /* the integer decision below near */
    if(d_round < R.near) {
        double m = sqrt(dv[0] * dv[0] + dv[1] * dv[1] + dv[2] * dv[2]);
        for(int i = 0; i < 3; i++) n[i] = m > 0 ? dv[i] / m : 0.0;
    } else {
        for(int i = 0; i < 3; i++) n[i] = dx[i] / d;
    }
    wa = (double)a->strength * ((a->flags & SH2Z_BC_WEAK) ? o->weak_share : 1.0);
    wb = (double)b->strength * ((b->flags & SH2Z_BC_WEAK) ? o->weak_share : 1.0);
    sa = wa / (wa + wb); sb = 1.0 - sa;
    if(o->swap_shares) { double t = sa; sa = sb; sb = t; }
    if(!before) {
        double impact = fabs(n[0] * dv[0] + n[1] * dv[1] + n[2] * dv[2]) * o->impact_gain;
        if(atk_a == atk_b) { oa->damage = sb * impact; ob->damage = sa * impact; }
        else if(atk_b) oa->damage = impact * ((b->flags & SH2Z_BC_HEAVY) ? o->heavy_gain : 1.0);
        else ob->damage = impact * ((a->flags & SH2Z_BC_HEAVY) ? o->heavy_gain : 1.0);
        ev |= SH2Z_BC_EV_FIRST;
    }
    if((o->closing_sign > 0) ? cl <= 0 : cl >= 0) {
        if(d_round < R.near) len = -o->reach;
        else { len = o->reach - d; if((atk_a || atk_b) && len < o->attack_min) len = o->attack_min; }
        if(atk_a != atk_b) {
            oout_t *v = atk_a ? ob : oa;
            const double s = atk_a ? 1.0 : -1.0;
            for(int i = 0; i < 3; i++) v->push[i] = s * len * o->attack_push * n[i];
            v->hit = 1;
            ev |= SH2Z_BC_EV_HIT;
        } else {
            for(int i = 0; i < 3; i++) { oa->push[i] = -len * sb * n[i]; ob->push[i] = len * sa * n[i]; }
        }
        ev |= SH2Z_BC_EV_PUSH;
    }
    return ev;
}

static double worst_damage, worst_push;

/* 1 when the library output agrees with the oracle within the bound. */
static int agrees(const sh2z_bc_body_t *a, const sh2z_bc_body_t *b, uint32_t ev, uint32_t oev,
                  const oout_t *oa, const oout_t *ob, int record) {
    if(ev != oev) return 0;
    const sh2z_bc_body_t *lb[2] = { a, b };
    const oout_t *ob2[2] = { oa, ob };
    for(int k = 0; k < 2; k++) {
        double e = fabs(lb[k]->damage - ob2[k]->damage * 65536.0);
        if(record && e > worst_damage) worst_damage = e;
        if(e > BOUND) return 0;
        if(lb[k]->hit != ob2[k]->hit) return 0;
        for(int i = 0; i < 3; i++) {
            e = fabs(lb[k]->push[i] - ob2[k]->push[i] * 65536.0);
            if(record && e > worst_push) worst_push = e;
            if(e > BOUND) return 0;
        }
    }
    return 1;
}

static void clear_out(sh2z_bc_body_t *b) {
    b->push[0] = b->push[1] = b->push[2] = 0; b->damage = 0; b->hit = 0; b->first = 0;
}

static void random_body(sh2z_bc_body_t *b) {
    memset(b, 0, sizeof *b);
    for(int i = 0; i < 3; i++) {
        b->pos[i] = rnd_range(-Q16(4000.0) / 4, Q16(4000.0) / 4) * 4;
        b->vel[i] = rnd_range(-Q16(120.0), Q16(120.0));
    }
    switch(rnd() % 4u) {
    case 0: b->strength = 1u + rnd() % (1u << 30); break;
    default: b->strength = 600000u + rnd() % 5000000u; break;   /* squared weights of real machines */
    }
    b->flags = SH2Z_BC_ACTIVE | (rnd() & (SH2Z_BC_ATTACK | SH2Z_BC_WEAK | SH2Z_BC_HEAVY | SH2Z_BC_CREDIT));
    b->toucher = -1;
    b->since = UINT32_MAX;
}

/* A pair placed in or near reach. */
static void random_pair(sh2z_bc_body_t *a, sh2z_bc_body_t *b) {
    random_body(a); random_body(b);
    const int32_t spread = (rnd() & 7u) == 0 ? Q16(0.02) : Q16(50.0);
    for(int i = 0; i < 3; i++) b->pos[i] = a->pos[i] + rnd_range(-spread, spread);
    if((rnd() & 15u) == 0) for(int i = 0; i < 3; i++) b->vel[i] = a->vel[i] + rnd_range(-2, 2); /* tiny dv */
    /* Keep shares above 2^-12 (the bound's condition). */
    const double wa = a->strength, wb = b->strength;
    if(wa / (wa + wb) < 1.0 / 4096.0) a->strength = b->strength / 1000u + 1u;
    if(wb / (wa + wb) < 1.0 / 4096.0) b->strength = a->strength / 1000u + 1u;
}

#define PAIRS 400000

static void test_pairs(void) {
    orules_t o = orules();
    uint64_t in_reach = 0, first = 0, pushed = 0, hits = 0, degenerate = 0;
    for(int t = 0; t < PAIRS; t++) {
        sh2z_bc_body_t a, b;
        oout_t oa, ob;
        random_pair(&a, &b);
        const int before = (int)(rnd() & 1u);
        const uint32_t oev = oracle(&a, &b, before, &o, &oa, &ob);
        const uint32_t ev = sh2z_bc_pair(&a, &b, 3u, 7u, before != 0, &R);
        if(!agrees(&a, &b, ev, oev, &oa, &ob, 0)) {
            fprintf(stderr, "case %d ev %x oev %x before %d flags %x %x str %u %u\n", t, (unsigned)ev, (unsigned)oev, before,
                    (unsigned)a.flags, (unsigned)b.flags, (unsigned)a.strength, (unsigned)b.strength);
            fprintf(stderr, " dmg %d %d vs %.1f %.1f hit %d %d vs %d %d\n", (int)a.damage, (int)b.damage,
                    oa.damage * 65536, ob.damage * 65536, a.hit, b.hit, oa.hit, ob.hit);
            for(int i = 0; i < 3; i++)
                fprintf(stderr, " push %d %d vs %.1f %.1f dx %d dv %d\n", (int)a.push[i], (int)b.push[i], oa.push[i] * 65536,
                        ob.push[i] * 65536, (int)(b.pos[i] - a.pos[i]), (int)(b.vel[i] - a.vel[i]));
        }
        CHECK(agrees(&a, &b, ev, oev, &oa, &ob, 1));
        if(ev & SH2Z_BC_EV_TOUCH) in_reach++;
        if(ev & SH2Z_BC_EV_FIRST) {
            first++;
            CHECK(a.first == 1u && b.first == 1u);
            CHECK(b.toucher == ((a.flags & SH2Z_BC_CREDIT) ? 3 : -1));
            CHECK(a.toucher == ((b.flags & SH2Z_BC_CREDIT) ? 7 : -1));
        } else {
            CHECK(a.first == 0u && b.first == 0u && a.toucher == -1 && b.toucher == -1);
        }
        if(ev & SH2Z_BC_EV_PUSH) pushed++;
        if(ev & SH2Z_BC_EV_HIT) hits++;
        if((ev & SH2Z_BC_EV_TOUCH) && llabs((long long)a.pos[0] - b.pos[0]) < Q16(0.02)) degenerate++;
    }
    /* The random set must exercise every branch. */
    CHECK(in_reach > PAIRS / 4 && first > PAIRS / 10 && pushed > PAIRS / 10 && hits > PAIRS / 50 && degenerate > 100);
    printf("pairs: %d (in reach %" PRIu64 ", first %" PRIu64 ", pushed %" PRIu64 ", hits %" PRIu64 "); worst damage %.1f LSB, worst push %.1f LSB\n",
           PAIRS, in_reach, first, pushed, hits, worst_damage, worst_push);
}

/* Edge cases: exactly at reach, one LSB beyond, coincident bodies, zero dv. */
static void test_edges(void) {
    orules_t o = orules();
    sh2z_bc_body_t a, b;
    oout_t oa, ob;
    memset(&a, 0, sizeof a); memset(&b, 0, sizeof b);
    a.strength = b.strength = 1000000u; a.flags = b.flags = SH2Z_BC_ACTIVE; a.toucher = b.toucher = -1;
    b.pos[0] = R.reach;                      /* d == reach: in reach */
    b.vel[0] = -Q16(1.0);
    CHECK(sh2z_bc_pair(&a, &b, 0, 1, false, &R) & SH2Z_BC_EV_TOUCH);
    clear_out(&a); clear_out(&b);
    b.pos[0] = R.reach + 1;                  /* one LSB beyond */
    CHECK(sh2z_bc_pair(&a, &b, 0, 1, false, &R) == 0u && a.damage == 0 && b.damage == 0);
    b.pos[0] = Q16(30.0); b.pos[1] = Q16(30.0); b.pos[2] = Q16(30.0);   /* each axis in reach, distance not */
    CHECK(sh2z_bc_pair(&a, &b, 0, 1, false, &R) == 0u);
    /* Coincident with dv: the normal comes from dv, the push is -reach n. */
    b.pos[0] = b.pos[1] = b.pos[2] = 0; b.vel[0] = Q16(2.0); b.vel[1] = 0; b.vel[2] = 0;
    clear_out(&a); clear_out(&b);
    uint32_t ev = sh2z_bc_pair(&a, &b, 0, 1, false, &R);
    CHECK(agrees(&a, &b, ev, oracle(&a, &b, 0, &o, &oa, &ob), &oa, &ob, 0));
    CHECK(b.push[0] < 0 && a.push[0] > 0);
    /* Coincident and at rest: no direction, no push, no damage, still in reach. */
    b.vel[0] = 0; clear_out(&a); clear_out(&b);
    ev = sh2z_bc_pair(&a, &b, 0, 1, false, &R);
    CHECK((ev & SH2Z_BC_EV_TOUCH) && a.damage == 0 && b.damage == 0 && a.push[0] == 0 && b.push[0] == 0);
    /* Separating pair: in reach, impact on the first tick, no push. */
    b.pos[0] = Q16(40.0); b.vel[0] = Q16(3.0); clear_out(&a); clear_out(&b);
    ev = sh2z_bc_pair(&a, &b, 0, 1, false, &R);
    CHECK(ev == (SH2Z_BC_EV_TOUCH | SH2Z_BC_EV_FIRST) && a.push[0] == 0 && b.damage > 0);
    /* Equal weights, head-on at 4 units/tick, 10 apart: each takes 2.2, pushed 18 apart in halves. */
    b.pos[0] = Q16(36.0); b.vel[0] = -Q16(4.0); clear_out(&a); clear_out(&b);
    ev = sh2z_bc_pair(&a, &b, 0, 1, false, &R);
    CHECK(llabs((long long)a.damage - Q16(2.2)) <= 2 && llabs((long long)b.damage - Q16(2.2)) <= 2);
    CHECK(llabs((long long)a.push[0] + Q16(5.0)) <= 2 && llabs((long long)b.push[0] - Q16(5.0)) <= 2);
    /* Heavy attacker: the victim takes 10x, the attacker nothing; victim pushed 3 x 10 = 30. */
    a.flags |= SH2Z_BC_ATTACK | SH2Z_BC_HEAVY; clear_out(&a); clear_out(&b);
    ev = sh2z_bc_pair(&a, &b, 0, 1, false, &R);
    CHECK(a.damage == 0 && llabs((long long)b.damage - Q16(44.0)) <= 64 && b.hit == 1u && a.push[0] == 0);
    CHECK(llabs((long long)b.push[0] - Q16(30.0)) <= 4);
    /* Attack with the pair nearly at reach: the push is at least attack_min long (3 x 3 = 9). */
    b.pos[0] = Q16(45.5); clear_out(&a); clear_out(&b);
    ev = sh2z_bc_pair(&a, &b, 0, 1, true, &R);
    CHECK(ev == (SH2Z_BC_EV_TOUCH | SH2Z_BC_EV_PUSH | SH2Z_BC_EV_HIT) && llabs((long long)b.push[0] - Q16(9.0)) <= 4);
}

/* ---- The field walk against a brute-force walk ------------------------- */

#define NB 30u
#define NPAIR (NB * (NB - 1u) / 2u)

static void sort_order(uint8_t *order, const int32_t *along, uint32_t n) {
    for(uint32_t i = 0; i < n; i++) order[i] = (uint8_t)i;
    for(uint32_t i = 1; i < n; i++) {
        uint8_t k = order[i]; uint32_t j = i;
        while(j > 0 && (along[order[j - 1]] < along[k] || (along[order[j - 1]] == along[k] && order[j - 1] > k))) { order[j] = order[j - 1]; --j; }
        order[j] = k;
    }
}

static void test_field(void) {
    static sh2z_bc_body_t fb[NB], bb[NB];
    static uint32_t last[NPAIR];
    static uint8_t touching_before[NB][NB];
    static int32_t along[NB];
    uint8_t order[NB];
    sh2z_bc_field_t f;
    const int32_t length = 4000 * 256;   /* a short closed line: the walk wraps often */
    uint64_t total_tested = 0, total_touch = 0, wraps = 0;
    for(int race = 0; race < 40; race++) {
        for(uint32_t i = 0; i < NB; i++) {
            random_body(&fb[i]);
            fb[i].flags &= ~SH2Z_BC_ATTACK;
            if((rnd() & 15u) == 0) fb[i].flags &= ~SH2Z_BC_ACTIVE;
            /* along the line on x, sideways on z */
            fb[i].pos[0] = rnd_range(0, length / 256 - 1) * 65536 / 1;
            fb[i].pos[1] = 0;
            fb[i].pos[2] = rnd_range(-Q16(40.0), Q16(40.0));
            fb[i].vel[0] = rnd_range(Q16(20.0), Q16(40.0));
            fb[i].vel[2] = rnd_range(-Q16(2.0), Q16(2.0));
        }
        /* bunch them so contacts happen */
        for(uint32_t i = 0; i < NB; i++) fb[i].pos[0] = (int32_t)(((int64_t)fb[i].pos[0] % (Q16(400.0))) );
        sh2z_bc_field_init(&f, fb, last, NB);
        memcpy(bb, fb, sizeof bb);
        memset(touching_before, 0, sizeof touching_before);
        for(int tick = 0; tick < 300; tick++) {
            sh2z_bc_field_begin(&f);
            for(uint32_t i = 0; i < NB; i++) {
                clear_out(&bb[i]);
                if(bb[i].since != UINT32_MAX) bb[i].since++;
                CHECK(bb[i].since == fb[i].since && bb[i].toucher == fb[i].toucher);
                along[i] = (int32_t)((((int64_t)fb[i].pos[0] >> 8) % length + length) % length);
                const uint32_t atk = (rnd() % 7u) == 0 ? SH2Z_BC_ATTACK : 0u;
                fb[i].flags = (fb[i].flags & ~SH2Z_BC_ATTACK) | atk;
                bb[i].flags = fb[i].flags;
            }
            sort_order(order, along, NB);
            const uint32_t touching = sh2z_bc_field_resolve(&f, order, along, length, &R);
            total_tested += f.tested;
            /* brute force: every pair, both orders of the along test */
            uint32_t bt = 0;
            for(uint32_t j = 1; j < NB; j++) for(uint32_t i = 0; i < j; i++) {
                if(!(bb[i].flags & SH2Z_BC_ACTIVE) || !(bb[j].flags & SH2Z_BC_ACTIVE)) { touching_before[i][j] = 0; continue; }
                int32_t gap = along[i] - along[j];
                if(gap > length / 2) gap -= length; else if(gap <= -length / 2) gap += length;
                if(gap > R.reach_long || gap < -R.reach_long) { touching_before[i][j] = 0; continue; }
                { const int32_t raw = along[i] - along[j]; if(raw > length / 2 || raw < -length / 2) wraps++; }
                const uint32_t ev = sh2z_bc_pair(&bb[i], &bb[j], i, j, touching_before[i][j] != 0, &R);
                touching_before[i][j] = (ev & SH2Z_BC_EV_TOUCH) ? 1u : 0u;
                bt += touching_before[i][j];
            }
            CHECK(touching == bt);
            total_touch += bt;
            for(uint32_t i = 0; i < NB; i++) {
                CHECK(fb[i].damage == bb[i].damage && fb[i].hit == bb[i].hit && fb[i].first == bb[i].first);
                CHECK(fb[i].push[0] == bb[i].push[0] && fb[i].push[1] == bb[i].push[1] && fb[i].push[2] == bb[i].push[2]);
                CHECK(fb[i].toucher == bb[i].toucher && fb[i].since == bb[i].since);
            }
            /* move: own velocity plus the push, kept on the short line */
            for(uint32_t i = 0; i < NB; i++) {
                for(int c = 0; c < 3; c++) {
                    fb[i].vel[c] += fb[i].push[c] / 4;
                    fb[i].pos[c] += fb[i].vel[c];
                }
                if(fb[i].pos[2] > Q16(60.0)) { fb[i].pos[2] = Q16(60.0); fb[i].vel[2] = -fb[i].vel[2] / 2; }
                if(fb[i].pos[2] < -Q16(60.0)) { fb[i].pos[2] = -Q16(60.0); fb[i].vel[2] = -fb[i].vel[2] / 2; }
                bb[i].pos[0] = fb[i].pos[0]; bb[i].pos[1] = fb[i].pos[1]; bb[i].pos[2] = fb[i].pos[2];
                bb[i].vel[0] = fb[i].vel[0]; bb[i].vel[1] = fb[i].vel[1]; bb[i].vel[2] = fb[i].vel[2];
            }
        }
    }
    CHECK(total_touch > 1000 && wraps > 0);
    printf("field: 40 races x 300 ticks x %u bodies; pairs tested %" PRIu64 " (all pairs %u per tick), in reach %" PRIu64 "\n",
           NB, total_tested, NPAIR, total_touch);
    CHECK(total_tested < (uint64_t)40 * 300 * NPAIR / 2);
}

/* History and credit on a scripted approach, stay, separation and return. */
static void test_history(void) {
    sh2z_bc_body_t b[3];
    uint32_t last[3];
    sh2z_bc_field_t f;
    int32_t along[3];
    uint8_t order[3];
    memset(b, 0, sizeof b);
    for(int i = 0; i < 3; i++) { b[i].strength = 1000000u; b[i].flags = SH2Z_BC_ACTIVE; }
    b[0].flags |= SH2Z_BC_CREDIT;              /* body 0 is the credited one */
    b[2].pos[0] = Q16(5000.0);                 /* far away all along */
    sh2z_bc_field_init(&f, b, last, 3);
    int firsts = 0, credited_at = -1, uncredited_at = -1;
    for(int t = 0; t < 400; t++) {
        /* body 1 alongside body 0 between ticks 10..39 and 200..209, apart otherwise */
        const int touch = (t >= 10 && t < 40) || (t >= 200 && t < 210);
        b[1].pos[2] = touch ? Q16(30.0) : Q16(200.0);
        b[1].vel[2] = touch ? -Q16(0.5) : 0;
        sh2z_bc_field_begin(&f);
        for(int i = 0; i < 3; i++) along[i] = b[i].pos[0] >> 8;
        sort_order(order, along, 3);
        sh2z_bc_field_resolve(&f, order, along, 100000 * 256, &R);
        firsts += b[1].first;
        CHECK(b[2].first == 0u && b[2].toucher == -1);
        if(t == 10 || t == 200) CHECK(b[1].first == 1u && b[1].toucher == 0 && b[1].since == 0u);
        if(t > 10 && t < 40) CHECK(b[1].first == 0u);
        const int32_t c = sh2z_bc_credit(&f, 1, &R);
        if(t > 40 && t < 200 && c < 0 && uncredited_at < 0) uncredited_at = t;
        if(t == 70) credited_at = c;
        CHECK(sh2z_bc_credit(&f, 0, &R) == -1);  /* body 1 is not CREDIT */
    }
    CHECK(firsts == 2 && f.contacts == 2u);
    CHECK(credited_at == 0 && uncredited_at == 71);  /* the window counts from the first tick of contact (10) */
    printf("history: 2 contacts, credit from tick 10 through 70\n");
}

/* ---- Controls: each rule changed in the oracle must be caught ---------- */

static int control_caught(void (*mut)(orules_t *), const char *name) {
    orules_t o = orules();
    mut(&o);
    uint32_t seed = rng;
    int caught = 0;
    for(int t = 0; t < 20000 && !caught; t++) {
        sh2z_bc_body_t a, b;
        oout_t oa, ob;
        random_pair(&a, &b);
        const int before = (int)(rnd() & 1u);
        const uint32_t oev = oracle(&a, &b, before, &o, &oa, &ob);
        const uint32_t ev = sh2z_bc_pair(&a, &b, 0, 1, before != 0, &R);
        if(!agrees(&a, &b, ev, oev, &oa, &ob, 0)) caught = 1;
    }
    (void)seed;
    printf("control %-22s %s\n", name, caught ? "caught" : "MISSED");
    return caught;
}
static void m_impact(orules_t *o) { o->impact_gain = 1.2; }
static void m_attack_push(orules_t *o) { o->attack_push = 2.5; }
static void m_shares(orules_t *o) { o->swap_shares = 1; }
static void m_weak(orules_t *o) { o->weak_share = 0.5; }
static void m_heavy(orules_t *o) { o->heavy_gain = 9.0; }
static void m_closing(orules_t *o) { o->closing_sign = -1; }
static void m_attack_min(orules_t *o) { o->attack_min = 2.0; }
static void m_reach(orules_t *o) { o->reach = 45.0; }

static void test_controls(void) {
    CHECK(control_caught(m_impact, "impact gain 1.2"));
    CHECK(control_caught(m_attack_push, "attack push 2.5"));
    CHECK(control_caught(m_shares, "shares swapped"));
    CHECK(control_caught(m_weak, "weak share 0.5"));
    CHECK(control_caught(m_heavy, "heavy gain 9"));
    CHECK(control_caught(m_closing, "push while separating"));
    CHECK(control_caught(m_attack_min, "attack min 2"));
    CHECK(control_caught(m_reach, "push reach 45"));
    /* The credit window: one tick longer must change the history scene's answer. */
    sh2z_bc_rules_t r2 = R;
    r2.credit_window = 61u;
    sh2z_bc_body_t b[2];
    uint32_t last[1];
    sh2z_bc_field_t f;
    memset(b, 0, sizeof b);
    b[0].strength = b[1].strength = 1u; b[0].flags = SH2Z_BC_ACTIVE | SH2Z_BC_CREDIT; b[1].flags = SH2Z_BC_ACTIVE;
    sh2z_bc_field_init(&f, b, last, 2);
    sh2z_bc_field_begin(&f);
    CHECK(sh2z_bc_pair(&b[0], &b[1], 0, 1, false, &R) & SH2Z_BC_EV_FIRST);
    for(int t = 0; t < 61; t++) sh2z_bc_field_begin(&f);
    CHECK(sh2z_bc_credit(&f, 1, &R) == -1 && sh2z_bc_credit(&f, 1, &r2) == 0);
    printf("control credit window 61   caught\n");
}

int main(void) {
    rules();
    test_edges();
    test_pairs();
    test_field();
    test_history();
    test_controls();
    printf("body-contact-1: %" PRIu64 " checks passed\n", checks);
    return 0;
}
