/* Sprite slot cache and cell4z decoder (sprite-cache-1): oracle and contract tests.
 *
 * The cache is checked against an independent reference model of the
 * replacement rule on every call of long random and racer-like sequences.
 * Each rule (key match, smallest fit, free first, least recently used,
 * live-frame protection, filling slots kept) has a control: the same
 * comparison against a reference with that one rule broken must fail.
 * The decoder is checked against a test-side encoder on generated cells,
 * and against truncated and corrupted streams placed at the very end of an
 * exact-size allocation, so any read past the buffer is caught by the
 * sanitizers.
 *
 * Copyright (c) 2026 SH2ZAM Contributors. MIT License.
 * Inspired by SH4ZAM by Falco Girgis and SH4ZAM Contributors.
 */
#include "sh2zam/sprite_cache_experimental.h"
#include "sh2zam/vlc_experimental.h"
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static uint64_t checks;
#define CHECK(e) do { ++checks; if (!(e)) { fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #e); exit(EXIT_FAILURE); } } while (0)
static uint32_t rng = UINT32_C(0x2545f491);
static uint32_t rnd(void) { rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5; return rng; }

/* ---- Reference model of the replacement rule ---- */
enum { BREAK_NONE, BREAK_KEY_SIZE, BREAK_SMALLEST, BREAK_FREE_FIRST, BREAK_LRU, BREAK_LIVE, BREAK_FILLING, BREAK_COUNT };
static const char *const break_name[BREAK_COUNT] = {
    "none", "key needs size", "smallest fit", "free first", "least recently used", "live protection", "filling kept"
};

typedef struct { uint32_t key, last, bytes, state; } ref_slot_t;
typedef struct { ref_slot_t s[256]; uint32_t n, frame, live; int brk; } ref_t;

static int ref_get(ref_t *m, uint32_t key, uint32_t bytes, bool pre, uint32_t *slot) {
    const uint32_t stamp = m->frame + (pre ? 1u : 0u);
    for (uint32_t i = 0; i < m->n; ++i) {
        ref_slot_t *s = &m->s[i];
        const bool size_ok = m->brk == BREAK_KEY_SIZE ? true : s->bytes >= bytes;
        if (s->state != 0u && s->key == key && size_ok) {
            if (s->last < stamp) s->last = stamp;
            *slot = i;
            return s->state == 2u ? SH2Z_SCACHE_HIT : SH2Z_SCACHE_PENDING;
        }
    }
    int best = -1;
    for (uint32_t i = 0; i < m->n; ++i) {
        const ref_slot_t *s = &m->s[i];
        if (s->bytes < bytes) continue;
        if (s->state == 1u && m->brk != BREAK_FILLING) continue;
        const bool live = s->last + m->live > m->frame;
        if (s->state != 0u && live && m->brk != BREAK_LIVE) continue;
        if (best < 0) { best = (int)i; continue; }
        const ref_slot_t *b = &m->s[best];
        if (m->brk != BREAK_SMALLEST && s->bytes != b->bytes) { if (s->bytes < b->bytes) best = (int)i; continue; }
        const bool sf = s->state == 0u, bf = b->state == 0u;
        if (m->brk != BREAK_FREE_FIRST && sf != bf) { if (sf) best = (int)i; continue; }
        if (m->brk == BREAK_LRU) { if (s->last > b->last) best = (int)i; continue; }
        if (!sf && !bf && s->last < b->last) best = (int)i;
    }
    if (best < 0) return SH2Z_SCACHE_REFUSED;
    ref_slot_t *s = &m->s[best];
    s->key = key; s->last = stamp; s->state = 1u;
    *slot = (uint32_t)best;
    return SH2Z_SCACHE_MISS;
}

/* One scene: n slots in up to 3 size classes, a stream of requests. Returns
 * true when the cache and the reference agree on every result and slot, and
 * no live slot was ever replaced. Fills complete after a random delay. */
static bool scene(uint32_t seed, int brk, uint32_t live, uint32_t frames, bool racer_like, uint64_t *hash) {
    static sh2z_scache_slot_t slots[256];
    const uint32_t sizes[3] = {512u, 1024u, 2048u}, counts[3] = {12u, 10u, 6u};
    sh2z_scache_t c;
    ref_t m;
    rng = seed;
    const uint32_t n = sh2z_scache_layout(slots, 256u, 0x1000u, 0x40000u, sizes, counts, 3u, 0);
    CHECK(n == 28u);
    CHECK(sh2z_scache_init(&c, slots, n, live));
    memset(&m, 0, sizeof m);
    m.n = n; m.frame = live; m.live = live; m.brk = brk;
    for (uint32_t i = 0; i < n; ++i) { m.s[i].bytes = slots[i].bytes; m.s[i].key = SH2Z_SCACHE_NO_KEY; }
    uint32_t pending[64], pend_n = 0;
    uint32_t prev_used[256], prev_n = 0;   /* slots referenced by the previous frame's list */
    uint64_t h = 1469598103934665603u;
    bool agree = true;
    /* Racer-like: 30 racers, each with a slowly drifting cell index, sizes by profile. */
    uint32_t cell[30], prof[30];
    for (uint32_t r = 0; r < 30; ++r) { cell[r] = rnd() % 60u; prof[r] = rnd() % 3u; }
    for (uint32_t f = 0; f < frames; ++f) {
        sh2z_scache_frame(&c);
        m.frame += 1u;
        /* finish some fills */
        for (uint32_t i = 0; i < pend_n;) {
            if (rnd() % 3u == 0u) {
                CHECK(sh2z_scache_ready(&c, pending[i]));
                m.s[pending[i]].state = 2u;
                pending[i] = pending[--pend_n];
            } else ++i;
        }
        uint32_t used[256], used_n = 0;
        const uint32_t reqs = racer_like ? 30u : 1u + rnd() % 24u;
        for (uint32_t q = 0; q < reqs; ++q) {
            uint32_t key, bytes;
            bool pre;
            if (racer_like) {
                if (rnd() % 8u == 0u) cell[q] = (cell[q] + 1u + rnd() % 2u) % 60u;
                if (rnd() % 64u == 0u) prof[q] = rnd() % 3u;
                key = q * 256u + prof[q] * 64u + cell[q];
                bytes = prof[q] == 2u ? 1500u + (cell[q] % 5u) * 100u : prof[q] == 1u ? 700u + (cell[q] % 3u) * 80u : 200u + cell[q] * 4u;
                pre = rnd() % 4u == 0u;
            } else {
                key = rnd() % 90u;
                bytes = 100u + (key * 37u) % 1500u + (rnd() % 10u == 0u ? 400u : 0u);
                pre = rnd() % 5u == 0u;
            }
            uint32_t a = 999u, b = 999u;
            const int ra = sh2z_scache_get(&c, key, bytes, pre, &a);
            const int rb = ref_get(&m, key, bytes, pre, &b);
            if (ra != rb || (ra != SH2Z_SCACHE_REFUSED && a != b)) agree = false;
            if (ra == SH2Z_SCACHE_MISS) {
                /* A replaced slot must not be in the list VDP1 may still read. */
                for (uint32_t i = 0; i < prev_n && live >= 2u; ++i) if (prev_used[i] == a) agree = false;
                for (uint32_t i = 0; i < used_n; ++i) if (used[i] == a) agree = false;
                if (pend_n < 64u) pending[pend_n++] = a;
                else { CHECK(sh2z_scache_abandon(&c, a)); m.s[a].state = 0u; m.s[a].key = SH2Z_SCACHE_NO_KEY; }
            }
            if (ra == SH2Z_SCACHE_HIT || ra == SH2Z_SCACHE_PENDING || ra == SH2Z_SCACHE_MISS) {
                CHECK(slots[a].key == key && slots[a].bytes >= bytes);
                if (used_n < 256u) used[used_n++] = a;
            }
            h = (h ^ (uint64_t)(uint32_t)(ra * 1000 + (int)(ra == SH2Z_SCACHE_REFUSED ? 0u : a))) * 1099511628211u;
        }
        memcpy(prev_used, used, used_n * sizeof used[0]);
        prev_n = used_n;
        if (!agree) break;
    }
    if (hash) *hash = h;
    return agree;
}

static void cache_tests(void) {
    /* layout */
    sh2z_scache_slot_t s[8];
    const uint32_t sz[2] = {100u, 30u}, ct[2] = {2u, 3u};
    uint32_t end = 0;
    CHECK(sh2z_scache_layout(s, 8u, 0x13u, 1000u, sz, ct, 2u, &end) == 5u);
    CHECK(s[0].addr == 0x18u && s[1].addr == 0x80u && s[2].addr == 0xE8u && s[3].addr == 0x108u && s[4].addr == 0x128u);
    CHECK(end == 0x148u);
    for (uint32_t i = 0; i < 5u; ++i) CHECK((s[i].addr & 7u) == 0u && s[i].state == SH2Z_SCACHE_FREE && s[i].key == SH2Z_SCACHE_NO_KEY);
    CHECK(sh2z_scache_layout(s, 4u, 0u, 1000u, sz, ct, 2u, 0) == 0u);   /* too many slots */
    CHECK(sh2z_scache_layout(s, 8u, 0u, 300u, sz, ct, 2u, 0) == 0u);    /* does not fit */
    const uint32_t zero[1] = {0u}, one[1] = {1u};
    CHECK(sh2z_scache_layout(s, 8u, 0u, 300u, zero, one, 1u, 0) == 0u);
    /* init */
    sh2z_scache_t c;
    CHECK(!sh2z_scache_init(&c, s, 5u, 0u) && !sh2z_scache_init(&c, s, 5u, SH2Z_SCACHE_LIVE_MAX + 1u) && !sh2z_scache_init(&c, s, 0u, 1u));
    /* a small worked scene: two slots, live 2 */
    sh2z_scache_slot_t two[2];
    const uint32_t sz2[1] = {64u}, ct2[1] = {2u};
    CHECK(sh2z_scache_layout(two, 2u, 0u, 4096u, sz2, ct2, 1u, 0) == 2u);
    CHECK(sh2z_scache_init(&c, two, 2u, 2u));
    uint32_t a, b;
    sh2z_scache_frame(&c);
    CHECK(sh2z_scache_get(&c, 10u, 64u, false, &a) == SH2Z_SCACHE_MISS && a == 0u);
    CHECK(sh2z_scache_get(&c, 10u, 64u, false, &b) == SH2Z_SCACHE_PENDING && b == 0u);
    CHECK(sh2z_scache_get(&c, 11u, 64u, false, &b) == SH2Z_SCACHE_MISS && b == 1u);
    CHECK(sh2z_scache_get(&c, 12u, 64u, false, &b) == SH2Z_SCACHE_REFUSED);   /* both filling */
    CHECK(sh2z_scache_ready(&c, 0u) && sh2z_scache_ready(&c, 1u) && !sh2z_scache_ready(&c, 1u));
    CHECK(sh2z_scache_get(&c, 10u, 65u, false, &b) == SH2Z_SCACHE_REFUSED);   /* too big for any slot */
    sh2z_scache_frame(&c);   /* both slots still live: referenced last frame */
    CHECK(sh2z_scache_live(&c, 0u) && sh2z_scache_live(&c, 1u));
    CHECK(sh2z_scache_get(&c, 12u, 64u, false, &b) == SH2Z_SCACHE_REFUSED);
    CHECK(sh2z_scache_get(&c, 11u, 64u, false, &b) == SH2Z_SCACHE_HIT && b == 1u);
    sh2z_scache_frame(&c);   /* slot 0 now free to replace, slot 1 still live */
    CHECK(!sh2z_scache_live(&c, 0u) && sh2z_scache_live(&c, 1u));
    CHECK(sh2z_scache_get(&c, 12u, 64u, false, &b) == SH2Z_SCACHE_MISS && b == 0u);
    CHECK(c.evictions == 1u && c.hits == 1u && c.misses == 3u && c.refused == 3u);
    CHECK(sh2z_scache_abandon(&c, 0u) && !sh2z_scache_abandon(&c, 0u) && two[0].state == SH2Z_SCACHE_FREE);
    /* prefetch protects from now on */
    CHECK(sh2z_scache_get(&c, 13u, 64u, true, &b) == SH2Z_SCACHE_MISS && b == 0u && sh2z_scache_live(&c, 0u));
    CHECK(sh2z_scache_ready(&c, 0u));
    sh2z_scache_clear(&c);
    CHECK(two[0].state == SH2Z_SCACHE_FREE && two[1].state == SH2Z_SCACHE_FREE);
    /* aliases */
    const uint16_t map[4] = {0u, 0x8000u | 3u, 2u, 3u};
    uint32_t flip = 7u;
    CHECK(sh2z_scache_alias(map, 1u, &flip) == 3u && flip == 1u);
    CHECK(sh2z_scache_alias(map, 2u, &flip) == 2u && flip == 0u);
    CHECK(sh2z_scache_alias(0, 9u, &flip) == 9u && flip == 0u);

    /* Oracle agreement, determinism, and one failing control per rule. */
    for (uint32_t live = 1u; live <= 3u; ++live) {
        for (uint32_t seed = 1u; seed <= 40u; ++seed) {
            uint64_t h1, h2;
            CHECK(scene(seed * 7919u, BREAK_NONE, live, 400u, false, &h1));
            CHECK(scene(seed * 7919u, BREAK_NONE, live, 400u, false, &h2));
            CHECK(h1 == h2);
            CHECK(scene(seed * 104729u, BREAK_NONE, live, 600u, true, 0));
        }
    }
    for (int brk = 1; brk < BREAK_COUNT; ++brk) {
        bool caught = false;
        for (uint32_t seed = 1u; seed <= 40u && !caught; ++seed)
            for (uint32_t live = 1u; live <= 3u && !caught; ++live)
                caught = !scene(seed * 7919u, brk, live, 400u, (seed & 1u) != 0u, 0);
        if (!caught) fprintf(stderr, "control not caught: %s\n", break_name[brk]);
        CHECK(caught);
    }
}

/* ---- cell4z: test-side encoder ---- */
typedef struct { uint16_t *u; size_t cap, bits; } bw_t;
static void bw_put(bw_t *b, uint32_t code, uint32_t len) {
    for (uint32_t i = 0; i < len; ++i) {
        const size_t k = b->bits >> 4;
        CHECK(k < b->cap);
        if ((b->bits & 15u) == 0u) b->u[k] = 0u;
        b->u[k] |= (uint16_t)(((code >> i) & 1u) << (b->bits & 15u));
        ++b->bits;
    }
}

typedef struct { int ctx, sym, px; } tok_t;
static uint32_t tokenize(const uint8_t *px, uint32_t w, uint32_t h, tok_t *out) {
    uint32_t n = 0;
    for (uint32_t y = 0; y < h; ++y) {
        const uint8_t *row = px + y * w;
        uint32_t x = 0, left = 0;
        while (x < w) {
            if (row[x] == 0u) {
                uint32_t z = 0;
                while (x + z < w && row[x + z] == 0u) ++z;
                if (z >= 2u) {
                    int best = -1;
                    for (int r = 13; r >= 0; --r) if (sh2z_cell4z_runs[r] <= z) { best = r; break; }
                    out[n].ctx = (int)left; out[n].sym = 16 + best; out[n].px = sh2z_cell4z_runs[best]; ++n;
                    x += sh2z_cell4z_runs[best];
                    left = 0;
                    continue;
                }
            }
            out[n].ctx = (int)left; out[n].sym = row[x]; out[n].px = 1; ++n;
            left = row[x];
            ++x;
        }
    }
    return n;
}

/* A sprite-like cell: an opaque blob with smooth bands, transparent around it. */
static void gen_cell(uint8_t *px, uint32_t w, uint32_t h) {
    const int32_t cx = (int32_t)(w / 2u), cy = (int32_t)(h / 2u);
    const int32_t rx = (int32_t)(w / 2u) - 1 - (int32_t)(rnd() % (w / 4u + 1u)), ry = (int32_t)(h / 2u) - (int32_t)(rnd() % (h / 3u + 1u));
    for (uint32_t y = 0; y < h; ++y)
        for (uint32_t x = 0; x < w; ++x) {
            const int32_t dx = (int32_t)x - cx, dy = (int32_t)y - cy;
            const int64_t e = (int64_t)dx * dx * ry * ry + (int64_t)dy * dy * rx * rx;
            const int64_t lim = (int64_t)rx * rx * ry * ry;
            uint8_t v = 0u;
            if (rx > 0 && ry > 0 && e <= lim) v = (uint8_t)(1u + ((x / 3u + y / 2u + (rnd() % 5u == 0u)) % 15u));
            if (rnd() % 97u == 0u) v = (uint8_t)(rnd() % 16u);
            px[y * w + x] = v;
        }
}

static void decoder_tests(void) {
    enum { CELLS = 160 };
    static uint8_t px[CELLS][64 * 48];
    static uint32_t cw[CELLS], ch[CELLS];
    static tok_t tok[64 * 48];
    uint32_t freq[SH2Z_CELL4Z_CONTEXTS][256];
    memset(freq, 0, sizeof freq);
    rng = 12345u;
    for (int i = 0; i < CELLS; ++i) {
        cw[i] = 8u * (1u + rnd() % 8u);
        ch[i] = 1u + rnd() % 48u;
        gen_cell(px[i], cw[i], ch[i]);
        const uint32_t n = tokenize(px[i], cw[i], ch[i], tok);
        for (uint32_t k = 0; k < n; ++k) freq[tok[k].ctx][tok[k].sym] += 1u;
    }
    uint8_t len[SH2Z_CELL4Z_CONTEXTS][SH2Z_CELL4Z_SYMBOLS];
    uint16_t code[SH2Z_CELL4Z_CONTEXTS][256];
    for (uint32_t k = 0; k < SH2Z_CELL4Z_CONTEXTS; ++k) {
        for (uint32_t s = 0; s < SH2Z_CELL4Z_SYMBOLS; ++s) freq[k][s] += 1u;   /* every symbol codable */
        sh2z_vlc8_t t8;
        uint8_t l8[256];
        CHECK(sh2z_vlc8_build(&t8, freq[k], 8u, 15u, l8, code[k]));
        for (uint32_t s = 0; s < SH2Z_CELL4Z_SYMBOLS; ++s) len[k][s] = l8[s];
        for (uint32_t s = SH2Z_CELL4Z_SYMBOLS; s < 256u; ++s) CHECK(l8[s] == 0u);
    }
    static uint8_t arena[16384];
    sh2z_cell4z_tables_t tt;
    const size_t used = sh2z_cell4z_build(arena, sizeof arena, &len[0][0], &tt);
    CHECK(used > 0u && tt.lmax >= 1u && tt.lmax <= 15u);
    for (uint32_t k = 0; k < SH2Z_CELL4Z_CONTEXTS; ++k) CHECK(sh2z_vlc2_check(arena + tt.offset[k], tt.size[k], SH2Z_CELL4Z_W) != 0u);
    CHECK(sh2z_cell4z_build(arena, used - 1u, &len[0][0], &tt) == 0u);   /* arena too small */
    /* vlc-2 codes for each context table (canonical, first stream bit in bit 0) */
    uint16_t c2[SH2Z_CELL4Z_CONTEXTS][SH2Z_CELL4Z_SYMBOLS];
    static uint8_t scratch[2048];
    for (uint32_t k = 0; k < SH2Z_CELL4Z_CONTEXTS; ++k)
        CHECK(sh2z_vlc2_build(scratch, sizeof scratch, len[k], SH2Z_CELL4Z_SYMBOLS, SH2Z_CELL4Z_W, c2[k]) != 0u);
    uint64_t total_bits = 0, total_raw = 0;
    for (int i = 0; i < CELLS; ++i) {
        static uint16_t units[8192];
        bw_t b = {units, 8192u, 0u};
        const uint32_t n = tokenize(px[i], cw[i], ch[i], tok);
        for (uint32_t k = 0; k < n; ++k) bw_put(&b, c2[tok[k].ctx][tok[k].sym], len[tok[k].ctx][tok[k].sym]);
        const size_t units_n = (b.bits + 15u) >> 4;
        const size_t own = units_n * 2u;
        const size_t bytes = own + SH2Z_CELL4Z_SLACK(cw[i], tt.lmax);
        total_bits += b.bits;
        total_raw += cw[i] * ch[i] * 4u;
        uint8_t *buf = (uint8_t *)malloc(bytes);
        CHECK(buf != 0);
        for (size_t u = 0; u < units_n; ++u) { buf[2u * u] = (uint8_t)units[u]; buf[2u * u + 1u] = (uint8_t)(units[u] >> 8); }
        for (size_t u = own; u < bytes; ++u) buf[u] = (uint8_t)rnd();   /* whatever follows in a pack */
        uint16_t out[64 * 48 / 4 + 4];
        out[cw[i] * ch[i] / 4u] = 0xBEEFu;
        size_t used_bits = 0;
        CHECK(sh2z_cell4z_decode(out, cw[i], ch[i], buf, bytes, arena, &tt, &used_bits));
        CHECK(used_bits == b.bits);
        CHECK(out[cw[i] * ch[i] / 4u] == 0xBEEFu);   /* wrote exactly w*h/4 words */
        for (uint32_t p = 0; p < cw[i] * ch[i]; p += 4u) {
            const uint16_t want = (uint16_t)(px[i][p] << 12 | px[i][p + 1] << 8 | px[i][p + 2] << 4 | px[i][p + 3]);
            CHECK(out[p / 4u] == want);
        }
        /* Truncated: the last row cannot fit, so the decode refuses before reading past the buffer. */
        CHECK(!sh2z_cell4z_decode(out, cw[i], ch[i], buf, 2u, arena, &tt, 0));
        /* Garbage streams at the end of an exact allocation: any outcome, no out-of-bounds read. */
        for (int g = 0; g < 4; ++g) {
            for (size_t u = 0; u < bytes; ++u) buf[u] = (uint8_t)rnd();
            (void)sh2z_cell4z_decode(out, cw[i], ch[i], buf, bytes, arena, &tt, 0);
        }
        free(buf);
    }
    /* Bad sizes */
    uint16_t o[8];
    const uint8_t z[4] = {0, 0, 0, 0};
    CHECK(!sh2z_cell4z_decode(o, 12u, 1u, z, 4u, arena, &tt, 0));
    CHECK(!sh2z_cell4z_decode(o, 512u, 1u, z, 4u, arena, &tt, 0));
    CHECK(!sh2z_cell4z_decode(o, 8u, 0u, z, 4u, arena, &tt, 0));
    CHECK(!sh2z_cell4z_decode(o, 8u, 256u, z, 4u, arena, &tt, 0));
    CHECK(!sh2z_cell4z_decode(o, 8u, 1u, z, 1u, arena, &tt, 0));
    /* A run past the row end: an 8-wide row, 6 pixels then a run of 3 must fail. */
    {
        static uint16_t units[16];
        bw_t b = {units, 16u, 0u};
        for (int p = 0; p < 6; ++p) bw_put(&b, c2[p ? 1 : 0][1], len[p ? 1 : 0][1]);
        bw_put(&b, c2[1][17], len[1][17]);   /* run of 3 */
        uint8_t buf[40] = {0};
        for (size_t u = 0; u < 16u; ++u) { buf[2u * u] = (uint8_t)units[u]; buf[2u * u + 1u] = (uint8_t)(units[u] >> 8); }
        CHECK(!sh2z_cell4z_decode(o, 8u, 1u, buf, sizeof buf, arena, &tt, 0));
    }
    printf("cell4z: %d generated cells, %.3f of raw 4-bit size\n", CELLS, (double)total_bits / (double)total_raw);
}

int main(void) {
    cache_tests();
    decoder_tests();
    printf("sprite-cache-1: %" PRIu64 " checks passed\n", checks);
    return 0;
}
