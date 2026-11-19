/* Racing-line follower (contract tag line-follower-1) oracles. MIT; SH2ZAM Contributors.
 * Oracles use int64 arithmetic on the documented formulas and brute-force
 * searches; the library never divides at run time. */
#include "sh2zam/line_follower_experimental.h"
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static uint64_t checks;
#define CHECK(e) do { ++checks; if(!(e)) { fprintf(stderr,"FAIL %s:%d: %s\n",__FILE__,__LINE__,#e); exit(EXIT_FAILURE); } } while(0)
static uint32_t rng = UINT32_C(0x2545f491);
static uint32_t rnd(void) { rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5; return rng; }
static int32_t rnd_range(int32_t lo, int32_t hi) { return lo + (int32_t)(rnd() % (uint32_t)(hi - lo + 1)); }

#define MAXN 64
static sh2z_line_node_t nodes[MAXN];

/* A random closed line with count nodes over length. */
static sh2z_line_t make_line(uint32_t count, int32_t length) {
    sh2z_line_t l;
    int32_t starts[MAXN];
    starts[0] = 0;
    for(uint32_t i = 1; i < count; ++i) starts[i] = (int32_t)((int64_t)length * i / count) + rnd_range(-8, 8);
    for(uint32_t i = 0; i < count; ++i) {
        const int32_t next = i + 1 < count ? starts[i + 1] : length;
        nodes[i].start = starts[i];
        nodes[i].lateral = (int16_t)rnd_range(-300, 300);
        nodes[i].speed_cap = (uint16_t)rnd_range(200, 2000);
        nodes[i].half_width = (uint16_t)rnd_range(200, 400);
        nodes[i].flags = (uint16_t)(rnd() & 0xffffu);
        nodes[i].inv_span = sh2z_line_node_inv_span(next - starts[i]);
    }
    l.nodes = nodes; l.count = count; l.length = length;
    return l;
}

static uint32_t oracle_locate(const sh2z_line_t *l, int32_t dist) {
    uint32_t best = 0;
    for(uint32_t i = 0; i < l->count; ++i) if(l->nodes[i].start <= dist) best = i;
    return best;
}

static void test_inv_span(void) {
    CHECK(sh2z_line_node_inv_span(0) == 0u);
    CHECK(sh2z_line_node_inv_span(-5) == 0u);
    CHECK(sh2z_line_node_inv_span(65537) == 0u);
    CHECK(sh2z_line_node_inv_span(1) == 0u);
    for(int i = 0; i < 20000; ++i) {
        const int32_t s = rnd_range(2, 65536);
        CHECK(sh2z_line_node_inv_span(s) == (uint32_t)((UINT64_C(1) << 32) / (uint64_t)s));
    }
}

static void test_locate_sample(void) {
    for(int round = 0; round < 200; ++round) {
        const uint32_t count = (uint32_t)rnd_range(1, MAXN);
        const int32_t length = rnd_range(4096 * (int32_t)count, 65000 * (int32_t)count);
        const sh2z_line_t l = make_line(count, length);
        uint32_t hint = 0;
        for(int k = 0; k < 400; ++k) {
            const int32_t d = rnd_range(0, length - 1);
            const uint32_t n = sh2z_line_locate(&l, d, (k & 3) ? hint : rnd() % (count + 3));
            CHECK(n == oracle_locate(&l, d));
            hint = n;
            sh2z_line_sample_t s;
            sh2z_line_sample(&l, n, d, &s);
            const sh2z_line_node_t *a = &l.nodes[n], *b = n + 1 < count ? a + 1 : &l.nodes[0];
            const int64_t span = (n + 1 < count ? b->start : length) - a->start;
            const int64_t dd = d - a->start;
            /* Documented formula, exactly. */
            int64_t f = (int64_t)(((uint64_t)dd * a->inv_span) >> 16);
            if(f > 65535) f = 65535;
            CHECK(s.lateral == (int32_t)((int64_t)a->lateral * 256 + (((int64_t)(b->lateral - a->lateral) * f) >> 8)));
            CHECK(s.speed_cap == (int32_t)((int64_t)a->speed_cap * 16 + (((int64_t)(b->speed_cap - a->speed_cap) * f) >> 12)));
            CHECK(s.half_width == (int32_t)((int64_t)a->half_width * 256 + (((int64_t)(b->half_width - a->half_width) * f) >> 8)));
            CHECK(s.flags == a->flags);
            /* And close to true linear interpolation (within 2 Q8 units). */
            const int64_t exact = (int64_t)a->lateral * 256 + (int64_t)(b->lateral - a->lateral) * 256 * dd / span;
            CHECK(llabs(exact - s.lateral) <= 2 + llabs((int64_t)(b->lateral - a->lateral)) / 128);
        }
    }
}

static void test_advance(void) {
    const sh2z_line_t l = make_line(16, 16 * 65000);
    sh2z_track_state_t s;
    memset(&s, 0, sizeof s);
    int64_t total = 0;
    for(int i = 0; i < 100000; ++i) {
        s.speed = rnd_range(0, 9000);
        const uint32_t n = (uint32_t)rnd_range(1, 8);
        sh2z_track_advance(&l, &s, n);
        total += (int64_t)s.speed * n;
        CHECK(s.dist >= 0 && s.dist < l.length);
        CHECK((int64_t)s.lap * l.length + s.dist == total);
        CHECK(s.node == oracle_locate(&l, s.dist));
        if(total > (INT64_C(1) << 30)) { memset(&s, 0, sizeof s); total = 0; }
    }
}

static void test_follow(void) {
    const sh2z_follow_params_t p = { 24, 48, 6554, 96, 512 };
    for(int round = 0; round < 300; ++round) {
        const uint32_t nc = (uint32_t)rnd_range(4, MAXN);
        const sh2z_line_t l = make_line(nc, rnd_range(4096 * (int32_t)nc, 65000 * (int32_t)nc));
        sh2z_track_state_t s;
        memset(&s, 0, sizeof s);
        s.lateral = rnd_range(-20000, 20000);
        s.speed = rnd_range(0, 30000);
        for(int t = 0; t < 600; ++t) {
            const uint32_t n = (uint32_t)(1u << (rnd() % 4u));
            const int32_t target_speed = rnd_range(-100, 40000);
            const int32_t target_lat = rnd_range(-120000, 120000);
            sh2z_line_sample_t at;
            sh2z_line_sample(&l, s.node, s.dist, &at);
            const sh2z_track_state_t before = s;
            sh2z_follow_step(&l, &p, &s, target_speed, target_lat, n);
            const int32_t want = target_speed < at.speed_cap ? (target_speed < 0 ? 0 : target_speed) : at.speed_cap;
            CHECK(s.speed >= 0);
            CHECK(s.speed - before.speed <= p.accel * (int32_t)n);
            CHECK(before.speed - s.speed <= p.brake * (int32_t)n);
            if(before.speed <= want) CHECK(s.speed <= want); else CHECK(s.speed >= want);
            CHECK(abs(s.lat_vel) <= p.steer_max);
            const int32_t limit = at.half_width - p.margin > 0 ? at.half_width - p.margin : 0;
            CHECK(s.lateral >= -limit && s.lateral <= limit);
        }
    }
    /* Convergence: a constant target inside the width is reached and held. */
    const sh2z_line_t l = make_line(8, 8 * 65000);
    for(int i = 0; i < 8; ++i) nodes[i].half_width = 400, nodes[i].speed_cap = 1600;
    sh2z_track_state_t s;
    memset(&s, 0, sizeof s);
    for(int t = 0; t < 2000; ++t) sh2z_follow_step(&l, &p, &s, 20000, 25600, 1);
    CHECK(abs(s.lateral - 25600) <= 16);
    CHECK(s.speed == 20000);
}

static void test_pace_avoid_lod(void) {
    for(int i = 0; i < 100000; ++i) {
        const int32_t base = rnd_range(0, 40000), e = rnd_range(0, INT32_C(1) << 29), a = rnd_range(0, INT32_C(1) << 29);
        const int32_t g = rnd_range(0, 65536), lo = rnd_range(0, 20000), hi = lo + rnd_range(0, 40000);
        int64_t t = (int64_t)base + ((((int64_t)e - a) * g) >> 16);
        if(t < lo) t = lo;
        if(t > hi) t = hi;
        CHECK(sh2z_pace_target(base, e, a, g, lo, hi) == (int32_t)t);
    }
    const sh2z_line_t l = make_line(8, 8 * 65000);
    sh2z_track_state_t self, ahead;
    memset(&self, 0, sizeof self); memset(&ahead, 0, sizeof ahead);
    self.dist = 1000; ahead.dist = 1000 + 5000; ahead.lateral = 2000;
    CHECK(sh2z_follow_avoid(&l, &self, &ahead, 1000, 8000, 4000, 6000) == 2000 - 6000);
    CHECK(sh2z_follow_avoid(&l, &self, &ahead, 3000, 8000, 4000, 6000) == 2000 + 6000);
    CHECK(sh2z_follow_avoid(&l, &self, &ahead, 9000, 8000, 4000, 6000) == 9000);     /* clear sideways */
    ahead.dist = 1000 + 9000;
    CHECK(sh2z_follow_avoid(&l, &self, &ahead, 1000, 8000, 4000, 6000) == 1000);     /* too far ahead */
    ahead.dist = 500;
    CHECK(sh2z_follow_avoid(&l, &self, &ahead, 1000, 8000, 4000, 6000) == 1000);     /* behind */
    /* Wrap: ahead just past the line start. */
    self.dist = l.length - 2000; ahead.dist = 1000; ahead.lateral = 0;
    CHECK(sh2z_follow_avoid(&l, &self, &ahead, 0, 8000, 4000, 6000) == -6000);
    const int32_t near[3] = { 100, 200, 400 };
    CHECK(sh2z_lod_period(0, near) == 1u && sh2z_lod_period(-99, near) == 1u && sh2z_lod_period(100, near) == 2u);
    CHECK(sh2z_lod_period(-399, near) == 4u && sh2z_lod_period(400, near) == 8u);
    CHECK(sh2z_lod_due(10, 7, 4) == 0u && sh2z_lod_due(11, 7, 4) == 4u && sh2z_lod_due(20, 7, 4) == 13u);
    CHECK(sh2z_lod_due(3, UINT32_MAX - 1u, 4) == 5u); /* tick counter wrap */
}

static sh2z_track_state_t g_states[255];
static const sh2z_line_t *g_line;
static int cmp_total(const void *x, const void *y) {
    const uint8_t a = *(const uint8_t *)x, b = *(const uint8_t *)y;
    const int32_t ta = sh2z_track_total(g_line, &g_states[a]), tb = sh2z_track_total(g_line, &g_states[b]);
    if(ta != tb) return ta > tb ? -1 : 1;
    return a < b ? -1 : 1;
}
static int cmp_dist(const void *x, const void *y) {
    const uint8_t a = *(const uint8_t *)x, b = *(const uint8_t *)y;
    if(g_states[a].dist != g_states[b].dist) return g_states[a].dist > g_states[b].dist ? -1 : 1;
    return a < b ? -1 : 1;
}

static void test_sort(void) {
    const sh2z_line_t l = make_line(64, 64 * 65000);
    g_line = &l;
    for(int round = 0; round < 2000; ++round) {
        const uint32_t count = (uint32_t)rnd_range(1, 255);
        uint8_t order[255], want[255];
        for(uint32_t i = 0; i < count; ++i) {
            g_states[i].dist = rnd_range(0, 64) * 4096;        /* many ties */
            g_states[i].lap = rnd_range(0, 3);
            order[i] = (uint8_t)i;
        }
        for(uint32_t i = count; i > 1; --i) { const uint32_t j = rnd() % i; const uint8_t t = order[i - 1]; order[i - 1] = order[j]; order[j] = t; }
        memcpy(want, order, count);
        qsort(want, count, 1, cmp_total);
        sh2z_rank_sort(order, &l, g_states, count);
        CHECK(memcmp(order, want, count) == 0);
        qsort(want, count, 1, cmp_dist);
        sh2z_order_by_dist(order, g_states, count);
        CHECK(memcmp(order, want, count) == 0);
    }
}

/* Independent implementation of the documented contact rule. */
static int32_t o_clamp(int32_t v, int32_t lo, int32_t hi) { return v < lo ? lo : v > hi ? hi : v; }
static uint32_t oracle_contacts(const sh2z_line_t *l, const sh2z_contact_params_t *p, sh2z_track_state_t *st,
                                const uint8_t *order, uint32_t count) {
    uint32_t c = 0;
    if(count < 2) return 0;
    for(uint32_t i = 0; i < count; ++i) {
        sh2z_track_state_t *a = &st[order[i]];
        for(uint32_t k = 1; k < count; ++k) {
            sh2z_track_state_t *b = &st[order[(i + k) % count]];
            int64_t gap = (int64_t)a->dist - b->dist;
            if(gap > l->length / 2) gap -= l->length; else if(gap <= -(l->length / 2)) gap += l->length;
            if(gap < 0 || gap >= p->reach_long) break;
            const int32_t dl = a->lateral - b->lateral, adl = dl < 0 ? -dl : dl;
            if(adl >= p->reach_lat) continue;
            const int32_t ov = p->reach_lat - adl, push = (ov + 1) / 2, dir = dl >= 0 ? 1 : -1;
            sh2z_line_sample_t at;
            sh2z_line_sample(l, a->node, a->dist, &at);
            a->lateral = o_clamp(a->lateral + dir * push, -at.half_width, at.half_width);
            sh2z_line_sample(l, b->node, b->dist, &at);
            b->lateral = o_clamp(b->lateral - dir * (ov - push), -at.half_width, at.half_width);
            const int32_t keep = a->speed + (int32_t)((((int64_t)b->speed - a->speed) * p->bump_keep) >> 16);
            if(b->speed > keep) b->speed = keep;
            ++c;
        }
    }
    return c;
}

static void test_contacts(void) {
    const sh2z_contact_params_t p = { 92 * 256, 69 * 256, 16384 };
    static sh2z_track_state_t a[255], b[255];
    for(int round = 0; round < 3000; ++round) {
        const sh2z_line_t l = make_line(16, rnd_range(16 * 4096, 16 * 65000));
        const uint32_t count = (uint32_t)rnd_range(1, 60);
        const int32_t base = rnd_range(0, l.length - 1);
        uint8_t order[255];
        for(uint32_t i = 0; i < count; ++i) {
            a[i].dist = (base + rnd_range(0, 400 * 256)) % l.length;   /* a bunched pack, maybe across the wrap */
            a[i].lateral = rnd_range(-190, 190) * 256;  /* inside every node half width (>= 200) */
            a[i].speed = rnd_range(0, 30000);
            a[i].lat_vel = 0; a[i].lap = 0;
            a[i].node = sh2z_line_locate(&l, a[i].dist, 0);
            order[i] = (uint8_t)i;
        }
        sh2z_order_by_dist(order, a, count);
        memcpy(b, a, sizeof(a[0]) * count);
        const uint32_t got = sh2z_contact_resolve(&l, &p, a, order, count);
        const uint32_t want = oracle_contacts(&l, &p, b, order, count);
        CHECK(got == want);
        CHECK(memcmp(a, b, sizeof(a[0]) * count) == 0);
        for(uint32_t i = 0; i < count; ++i) {
            sh2z_line_sample_t at;
            sh2z_line_sample(&l, a[i].node, a[i].dist, &at);
            CHECK(a[i].lateral >= -at.half_width && a[i].lateral <= at.half_width);
            CHECK(a[i].speed >= 0);
        }
        /* External body: never moved, agents pushed out of its reach where the width allows. */
        sh2z_track_state_t body = a[0];
        body.lateral = 0;
        const sh2z_track_state_t keep = body;
        (void)sh2z_contact_external(&l, &p, a, count, &body);
        CHECK(memcmp(&body, &keep, sizeof body) == 0);
    }
}

int main(void) {
    test_inv_span();
    test_locate_sample();
    test_advance();
    test_follow();
    test_pace_avoid_lod();
    test_sort();
    test_contacts();
    printf("line follower: %" PRIu64 " checks passed\n", checks);
    return 0;
}
