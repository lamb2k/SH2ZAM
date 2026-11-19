/* VDP1 double-bank bookkeeping (vdp1-banks-1) against a direct model of
 * frames, draws and end-of-draw uploads, plus argument contracts.
 *
 * Copyright (c) 2026 SH2ZAM Contributors. MIT License.
 * Inspired by SH4ZAM by Falco Girgis and SH4ZAM Contributors.
 */
#include "sh2zam/vdp1_banks_experimental.h"
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static uint64_t checks;
#define CHECK(e) do { ++checks; if(!(e)) { fprintf(stderr,"FAIL %s:%d: %s\n",__FILE__,__LINE__,#e); exit(EXIT_FAILURE); } } while(0)
static uint32_t rng = UINT32_C(0x2545f491);
static uint32_t rnd(void) { rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5; return rng; }

/* A frame loop: each frame builds into a bank, may overlap the previous draw,
 * and the previous draw's tables are read until its end. The model tracks
 * which bank VDP1 is reading and asserts no build writes it. */
static void frame_loop(bool hold, uint32_t frames, uint32_t *conflicts_seen) {
    sh2z_vdp1_banks_t b;
    sh2z_vdp1_banks_init(&b, hold);
    CHECK(b.build == 0u && b.inflight == SH2Z_VDP1_BANK_NONE && !b.prev_ok);
    uint32_t reading = SH2Z_VDP1_BANK_NONE; /* bank VDP1 reads now */
    uint32_t serial = 0, overlap = 0, bad = 0;
    for (uint32_t f = 0; f < frames; ++f) {
        const bool busy = reading != SH2Z_VDP1_BANK_NONE && (rnd() & 3u) != 0u;
        if (!busy) reading = SH2Z_VDP1_BANK_NONE;
        const bool required = (rnd() % 11u) == 0u;
        const bool must_wait = sh2z_vdp1_banks_begin(&b, required, busy);
        CHECK(must_wait == (required || f == 0u || b.prev_ok == 0u));
        if (must_wait) { ++serial; reading = SH2Z_VDP1_BANK_NONE; }
        else if (busy) { ++overlap; if (sh2z_vdp1_banks_bank(&b) == reading) ++bad; }
        const uint32_t writes = sh2z_vdp1_banks_bank(&b);
        CHECK(writes <= 1u);
        CHECK(hold ? writes == 0u : true);
        const bool filled = (rnd() % 7u) != 0u;
        sh2z_vdp1_banks_submitted(&b, filled);
        CHECK(b.inflight == writes && b.prev_ok == (filled ? 1u : 0u));
        CHECK(hold ? b.build == 0u : b.build == (writes ^ 1u));
        reading = writes;
    }
    CHECK(b.frames == frames && b.serial == serial && b.overlap == overlap && b.conflicts == bad);
    *conflicts_seen = bad;
}

static void test_bank_state(void) {
    uint32_t c = 0;
    frame_loop(false, 5000u, &c);
    CHECK(c == 0u);                 /* alternating banks never write the drawn bank */
    frame_loop(true, 5000u, &c);
    CHECK(c > 0u);                  /* control: a held bank must conflict */
}

static void test_guards(void) {
    uint32_t w[8];
    sh2z_vdp1_guard_fill(w, 8u, 0xdb4b5a17u, 8u);
    for (uint32_t i = 0; i < 8u; ++i) CHECK(w[i] == (0xdb4b5a17u ^ (i << 8)));
    CHECK(sh2z_vdp1_guard_check(w, 8u, 0xdb4b5a17u, 8u));
    CHECK(!sh2z_vdp1_guard_check(w, 8u, 0xdb4b5a17u, 9u));
    w[5] ^= 0x100u;
    CHECK(!sh2z_vdp1_guard_check(w, 8u, 0xdb4b5a17u, 8u)); /* control: one bit */
    CHECK(sh2z_vdp1_guard_check(w, 0u, 0u, 0u));
}

static void test_carve(void) {
    const uint32_t sizes[3] = {1600u, 4096u, 448u}, aligns[3] = {32u, 8u, 4u};
    uintptr_t out[3] = {0, 0, 0}, end = 0;
    CHECK(sh2z_vdp1_carve(0x25c7c000u, 8192u, 32u, sizes, aligns, 3u, out, &end));
    CHECK(out[0] == 0x25c7c020u && out[1] == 0x25c7c660u && out[2] == 0x25c7d660u && end == 0x25c7d820u);
    CHECK(end + 32u - 0x25c7c000u <= 8192u);
    /* Exactly fits, then one byte too many. */
    const uint32_t s1[1] = {8192u - 64u}, a1[1] = {4u};
    CHECK(sh2z_vdp1_carve(0x1000u, 8192u, 32u, s1, a1, 1u, out, &end) && end == 0x1000u + 8192u - 32u);
    const uint32_t s2[1] = {8192u - 63u};
    uintptr_t keep[1] = {7u}, keep_end = 9u;
    CHECK(!sh2z_vdp1_carve(0x1000u, 8192u, 32u, s2, a1, 1u, keep, &keep_end) && keep[0] == 7u && keep_end == 9u);
    /* Alignment from a base that is only 4-byte aligned. */
    const uint32_t s3[2] = {3u, 10u}, a3[2] = {4u, 32u};
    CHECK(sh2z_vdp1_carve(0x2004u, 256u, 8u, s3, a3, 2u, out, &end) && out[0] == 0x200cu && out[1] == 0x2020u && end == 0x202au);
    const uint32_t bad_align[1] = {3u};
    CHECK(!sh2z_vdp1_carve(0x2000u, 256u, 8u, s3, bad_align, 1u, out, &end));
    CHECK(!sh2z_vdp1_carve(0x2002u, 256u, 8u, s3, a3, 1u, out, &end));
    CHECK(!sh2z_vdp1_carve(0x2000u, 256u, 6u, s3, a3, 1u, out, &end));
    CHECK(sh2z_vdp1_carve(0x2000u, 64u, 32u, NULL, NULL, 0u, NULL, &end) && end == 0x2020u);
    CHECK(!sh2z_vdp1_carve(0x2000u, 63u, 32u, NULL, NULL, 0u, NULL, &end));
}

/* Model: builds push in order, each completed draw pops the oldest; the
 * popped entry must always be the frame whose draw completed. */
static void test_queue(void) {
    sh2z_vdp1_queue_t q;
    sh2z_vdp1_queue_init(&q);
    uint32_t bank = 9, tag = 9;
    CHECK(!sh2z_vdp1_queue_pop(&q, &bank, &tag) && bank == 9u && tag == 9u);
    CHECK(sh2z_vdp1_queue_front(&q) == 0u);
    CHECK(!sh2z_vdp1_queue_push(&q, 0u, 0u));                 /* zero tag refused */
    CHECK(!sh2z_vdp1_queue_push(&q, 2u, 1u));                 /* bad bank */
    uint32_t drawn[64], nd = 0, next_tag = 1, b = 0;
    for (uint32_t step = 0; step < 20000u; ++step) {
        if ((rnd() & 1u) && sh2z_vdp1_queue_free(&q, b)) {
            CHECK(sh2z_vdp1_queue_push(&q, b, next_tag));
            drawn[nd++ % 64u] = next_tag++;
            b ^= 1u;
        } else if (q.count) {
            const uint32_t expect = q.tag[0];
            CHECK(sh2z_vdp1_queue_front(&q) == expect);
            CHECK(sh2z_vdp1_queue_pop(&q, &bank, &tag) && tag == expect && bank <= 1u);
        }
        CHECK(q.count <= 2u);
        if (q.count == 2u) CHECK(q.bank[0] != q.bank[1] && q.tag[0] < q.tag[1]);
    }
    (void)drawn;
    /* The same bank cannot be queued twice; the queue holds two at most. */
    sh2z_vdp1_queue_init(&q);
    CHECK(sh2z_vdp1_queue_push(&q, 1u, 5u) && !sh2z_vdp1_queue_push(&q, 1u, 6u) && sh2z_vdp1_queue_push(&q, 0u, 7u));
    CHECK(!sh2z_vdp1_queue_free(&q, 0u) && !sh2z_vdp1_queue_free(&q, 1u) && !sh2z_vdp1_queue_push(&q, 0u, 8u));
    CHECK(sh2z_vdp1_queue_pop(&q, &bank, &tag) && bank == 1u && tag == 5u && sh2z_vdp1_queue_front(&q) == 7u);
}

/* Model: a value per entry; each bank holds a copy refreshed only for stale
 * entries. After a bank is built, its copy must equal the values. */
static void test_stale(void) {
    sh2z_vdp1_stale64_t s;
    sh2z_vdp1_stale_init(&s);
    uint32_t value[64], copy[2][64];
    for (uint32_t i = 0; i < 64u; ++i) { value[i] = rnd(); copy[0][i] = copy[1][i] = 0xdeadbeefu; }
    for (uint32_t frame = 0; frame < 4000u; ++frame) {
        for (uint32_t k = rnd() % 4u; k; --k) { const uint32_t i = rnd() % 64u; value[i] = rnd(); sh2z_vdp1_stale_mark(&s, i); }
        const uint32_t bank = frame & 1u;
        for (uint32_t i = 0; i < 64u; ++i) if (sh2z_vdp1_stale_take(&s, bank, i)) copy[bank][i] = value[i];
        for (uint32_t i = 0; i < 64u; ++i) CHECK(copy[bank][i] == value[i]);
    }
    /* Control: a single shared flag (clearing both banks) leaves the other bank stale. */
    sh2z_vdp1_stale_init(&s);
    CHECK(sh2z_vdp1_stale_take(&s, 0u, 3u) && !sh2z_vdp1_stale_take(&s, 0u, 3u) && sh2z_vdp1_stale_take(&s, 1u, 3u));
    sh2z_vdp1_stale_mark(&s, 3u);
    CHECK(sh2z_vdp1_stale_take(&s, 1u, 3u) && sh2z_vdp1_stale_take(&s, 0u, 3u));
    CHECK(!sh2z_vdp1_stale_take(&s, 2u, 0u) && !sh2z_vdp1_stale_take(&s, 0u, 64u));
    sh2z_vdp1_stale_mark(&s, 64u); /* ignored */
}

static void test_jump_skip(void) {
    uint16_t r[16];
    for (uint32_t i = 0; i < 16u; ++i) r[i] = 0xffffu;
    CHECK(sh2z_vdp1_jump_skip(r, 130u));
    CHECK(r[0] == 0x5000u && r[1] == 520u);
    for (uint32_t i = 2; i < 16u; ++i) CHECK(r[i] == 0u);
    CHECK(((r[0] >> 12) & 7u) == 5u);           /* skip + assign */
    CHECK((uint32_t)r[1] * 8u == 130u * 32u);   /* link is the slot's byte address / 8 */
    for (uint32_t i = 0; i < 16u; ++i) r[i] = 0x1234u;
    CHECK(!sh2z_vdp1_jump_skip(r, 0x4000u) && r[0] == 0x1234u && r[15] == 0x1234u);
    CHECK(sh2z_vdp1_jump_skip(r, 0x3fffu) && r[1] == 0xfffcu);
    CHECK(!sh2z_vdp1_jump_skip(NULL, 1u));
}

int main(void) {
    test_bank_state();
    test_guards();
    test_carve();
    test_queue();
    test_stale();
    test_jump_skip();
    printf("vdp1_banks: %" PRIu64 " checks passed\n", checks);
    return 0;
}
