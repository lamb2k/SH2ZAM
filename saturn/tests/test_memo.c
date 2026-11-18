/* Block memo kernels (contract tag memo-1) against the independent oracle in
 * memo_oracle.h: exhaustive single-difference sweeps, random and edge-case
 * sets (zero counts, all equal, all different, mask-word boundaries, markers
 * at 0 and at the end, 2-byte alignment), canaries around every output, and
 * negative controls (planted oracle bugs that the comparison must catch).
 * Under ASan the marker copy reads from a source sized to end right after the
 * marker, so a read past it would stop the test.
 * MIT; Copyright (c) 2026 SH2ZAM Contributors. */
#include "sh2zam/memo_experimental.h"
#include "memo_oracle.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static uint64_t checks;
static void check(bool value, const char *what) {
    ++checks;
    if (!value) {
        fprintf(stderr, "memo check %llu failed: %s\n", (unsigned long long)checks, what);
        exit(EXIT_FAILURE);
    }
}
static uint32_t state = UINT32_C(0x3e3017a5);
static uint32_t rnd(void) { state ^= state << 13; state ^= state >> 17; state ^= state << 5; return state; }
static uint32_t below(uint32_t n) { return n ? rnd() % n : 0u; }

#define CANARY 0xC5u
enum { MAXG = 1100, PAD = 64 };

/* ---- A. Block compare-and-copy ------------------------------------------- */
/* 4-aligned arenas; `shift` = 2 moves both blocks off 4-byte alignment. */
static uint32_t arena_shadow[(MAXG * 64 + 2 * PAD) / 4 + 1], arena_src[(MAXG * 64 + 2 * PAD) / 4 + 1];
static uint8_t want_shadow[MAXG * 64], want_src[MAXG * 64];
static uint32_t mask_got[MAXG / 32 + 4], mask_want[MAXG / 32 + 4];

static void fill_pair(uint8_t *s, uint8_t *d, size_t bytes, unsigned mode) {
    /* mode 0 equal, 1 all different, 2 random density, 3 one byte, 4 random */
    for (size_t i = 0; i < bytes; i++) s[i] = (uint8_t)rnd();
    memcpy(d, s, bytes);
    const size_t blocks = bytes / 32u;
    switch (mode) {
    case 0: break;
    case 1: for (size_t i = 0; i < bytes; i++) d[i] = (uint8_t)~s[i]; break;
    case 2: {
        const uint32_t p = below(101);
        for (size_t b = 0; b < blocks; b++)
            if (below(100) < p) d[32u * b + below(32)] ^= (uint8_t)(1u << below(8));
        break;
    }
    case 3: if (bytes) d[below((uint32_t)bytes)] ^= (uint8_t)(1u + below(255)); break;
    default: for (size_t i = 0; i < bytes; i++) if (below(16) == 0) d[i] = (uint8_t)rnd(); break;
    }
}

static uint32_t a_cases;
/* One batch call (and the per-block calls over the same data) against the oracle. */
static void batch_case(bool longs, uint32_t groups, unsigned mode, unsigned shift) {
    const size_t bsize = longs ? 64u : 32u, bytes = bsize * groups;
    uint8_t *sh = (uint8_t *)arena_shadow, *sr = (uint8_t *)arena_src;
    memset(sh, CANARY, bytes + 2 * PAD + 4);
    memset(sr, CANARY, bytes + 2 * PAD + 4);
    uint8_t *s = sh + PAD + shift, *d = sr + PAD + shift;
    fill_pair(s, d, bytes, mode);
    memcpy(want_shadow, s, bytes);
    memcpy(want_src, d, bytes);
    const uint32_t words = (groups + 31u) / 32u;
    for (uint32_t i = 0; i < words + 2u; i++) mask_got[i] = 0xA5A5A5A5u;
    const uint32_t want = mo_sync_batch(want_shadow, d, bsize, groups, mask_want, MO_MUT_NONE);
    const uint32_t got = longs ? sh2z_memo_sync16l_batch((uint32_t *)(void *)s, (const uint32_t *)(const void *)d, groups, mask_got)
                               : sh2z_memo_sync16w_batch((uint16_t *)(void *)s, (const uint16_t *)(const void *)d, groups, mask_got);
    check(got == want, "batch changed count");
    check(memcmp(s, want_shadow, bytes) == 0, "batch shadow");
    check(memcmp(s, d, bytes) == 0, "batch shadow equals src");
    check(memcmp(d, want_src, bytes) == 0, "batch src unchanged");
    for (uint32_t i = 0; i < words; i++) check(mask_got[i] == mask_want[i], "batch mask word");
    check(mask_got[words] == 0xA5A5A5A5u && mask_got[words + 1u] == 0xA5A5A5A5u, "batch writes no mask word past the last");
    for (size_t i = 0; i < PAD + shift; i++) check(sh[i] == CANARY && sr[i] == CANARY, "batch canary before");
    for (size_t i = 0; i < PAD; i++) check(s[bytes + i] == CANARY && d[bytes + i] == CANARY, "batch canary after");
    /* the same data again: nothing changed now */
    const uint32_t again = longs ? sh2z_memo_sync16l_batch((uint32_t *)(void *)s, (const uint32_t *)(const void *)d, groups, mask_got)
                                 : sh2z_memo_sync16w_batch((uint16_t *)(void *)s, (const uint16_t *)(const void *)d, groups, mask_got);
    check(again == 0u, "batch steady state");
    for (uint32_t i = 0; i < words; i++) check(mask_got[i] == 0u, "batch steady mask");
    a_cases++;
}

static void single_case(bool longs, unsigned mode, unsigned shift, int same_pointer) {
    const size_t bytes = longs ? 64u : 32u;
    uint8_t *sh = (uint8_t *)arena_shadow, *sr = (uint8_t *)arena_src;
    memset(sh, CANARY, bytes + 2 * PAD + 4);
    memset(sr, CANARY, bytes + 2 * PAD + 4);
    uint8_t *s = sh + PAD + shift, *d = same_pointer ? s : sr + PAD + shift;
    if (same_pointer) { for (size_t i = 0; i < bytes; i++) s[i] = (uint8_t)rnd(); }
    else fill_pair(s, d, bytes, mode);
    memcpy(want_shadow, s, bytes);
    const bool want = mo_sync(want_shadow, d, bytes, MO_MUT_NONE);
    const bool got = longs ? sh2z_memo_sync16l((uint32_t *)(void *)s, (const uint32_t *)(const void *)d)
                           : sh2z_memo_sync16w((uint16_t *)(void *)s, (const uint16_t *)(const void *)d);
    check(got == want, "single changed flag");
    check(memcmp(s, want_shadow, bytes) == 0, "single shadow");
    for (size_t i = 0; i < PAD; i++) check(s[bytes + i] == CANARY && s[-1 - (ptrdiff_t)i] == CANARY, "single canary");
    a_cases++;
}

static void test_blocks(void) {
    for (int longs = 0; longs < 2; longs++) {
        const size_t bytes = longs ? 64u : 32u;
        /* exhaustive: every byte position and every bit of it differing */
        for (size_t pos = 0; pos < bytes; pos++)
            for (unsigned bit = 0; bit < 8u; bit++) {
                uint8_t *s = (uint8_t *)arena_shadow + PAD, *d = (uint8_t *)arena_src + PAD;
                for (size_t i = 0; i < bytes; i++) s[i] = d[i] = (uint8_t)rnd();
                s[pos] ^= (uint8_t)(1u << bit);
                memcpy(want_shadow, d, bytes);
                const bool got = longs ? sh2z_memo_sync16l((uint32_t *)(void *)s, (const uint32_t *)(const void *)d)
                                       : sh2z_memo_sync16w((uint16_t *)(void *)s, (const uint16_t *)(const void *)d);
                check(got, "single-bit difference detected");
                check(memcmp(s, want_shadow, bytes) == 0, "single-bit difference copied");
                a_cases++;
            }
        for (unsigned mode = 0; mode < 5u; mode++)
            for (unsigned it = 0; it < 400u; it++) {
                single_case(longs != 0, mode, 0, 0);
                if (!longs) single_case(false, mode, 2, 0);
            }
        single_case(longs != 0, 0, 0, 1);
        /* every group count across the first mask words, every mode */
        for (uint32_t g = 0; g <= 130u; g++)
            for (unsigned mode = 0; mode < 5u; mode++) {
                batch_case(longs != 0, g, mode, 0);
                if (!longs) batch_case(false, g, mode, 2);
            }
        for (unsigned it = 0; it < 300u; it++) {
            const uint32_t g = it < 4u ? MAXG - it : below(MAXG + 1u);
            batch_case(longs != 0, g, below(5), longs ? 0u : 2u * below(2));
        }
    }
    printf("memo blocks: %u cases (single and batch, 16 words and 16 longwords)\n", a_cases);
}

/* ---- B. Keyed write-through ---------------------------------------------- */
enum { MAXR = 700, DSTB = MAXR * 64 + 256 };
static sh2z_memo_rec4_t recs[MAXR], recs_copy[MAXR];
static uint32_t memo_got[MAXR + 2], memo_want[MAXR + 2], keys[MAXR];
static uint16_t bwords[4 * MAXR];
static uint16_t dst_got[DSTB / 2], dst_want[DSTB / 2];
static mo_write_t wlist[4 * MAXR];
static uint32_t b_cases, b_writes;

static uint32_t rand_gate(void) {
    switch (below(7)) {
    case 0: return 0;
    case 1: return 0xFFFFFFFFu;
    case 2: return 0xFFFF0000u;
    case 3: return 0x0000FFFFu;
    case 4: return UINT32_C(1) << below(32);
    case 5: return 0xFF000000u >> (8u * below(4));
    default: return rnd();
    }
}
static void rand_desc(sh2z_memo_gate4_t *g, mo_gate_t *o) {
    static const uint32_t strides[] = {8, 16, 32, 2, 0};
    g->stride = below(3) ? strides[below(5)] : 2u * below(33);
    for (unsigned k = 0; k < 4u; k++) {
        g->gate[k] = rand_gate();
        const uint32_t span = g->stride > 8u ? g->stride : 8u;
        g->offset[k] = (uint16_t)(below(5) ? 2u * below(span / 2u) : 2u * k);
    }
    g->always = below(16);
    for (unsigned k = 0; k < 4u; k++) { o->gate[k] = g->gate[k]; o->offset[k] = g->offset[k]; }
    o->stride = g->stride;
    o->always = g->always;
}
static void put_case(const sh2z_memo_gate4_t *g, const mo_gate_t *o, uint32_t n, unsigned flip_pct, int inline_form) {
    for (uint32_t i = 0; i < n; i++) {
        memo_got[i] = memo_want[i] = rnd();
        uint32_t k = memo_got[i];
        if (below(100) < flip_pct) k ^= below(4) ? (UINT32_C(1) << below(32)) : rnd();
        keys[i] = k;
        recs[i].key = k;
        for (unsigned j = 0; j < 4u; j++) bwords[4u * i + j] = recs[i].word[j] = (uint16_t)rnd();
    }
    memo_got[n] = memo_want[n] = 0x5A5A5A5Au;
    memcpy(recs_copy, recs, sizeof recs[0] * n);
    const uint32_t want = mo_put4(memo_want, keys, bwords, n, o, wlist, MO_MUT_NONE);
    /* two sentinels, so a write of a value equal to the sentinel cannot hide */
    for (int pass = 0; pass < 2; pass++) {
        const uint8_t fill = pass ? 0x3Cu : 0xC3u;
        memset(dst_got, fill, sizeof dst_got);
        memset(dst_want, fill, sizeof dst_want);
        uint32_t *memo = memo_got;
        uint32_t saved[MAXR + 2];
        memcpy(saved, memo_got, sizeof saved[0] * (n + 1u));
        uint32_t got = 0;
        if (inline_form) {
            for (uint32_t i = 0; i < n; i++)
                got += sh2z_memo_put4_one((volatile uint16_t *)(void *)((uint8_t *)dst_got + (size_t)i * g->stride),
                                          memo + i, recs[i].key, recs[i].word, g);
        } else {
            got = sh2z_memo_put4((volatile uint16_t *)dst_got, memo, recs, n, g);
        }
        mo_apply((uint8_t *)dst_want, wlist, want);
        check(got == want, "put4 write count");
        check(memcmp(dst_got, dst_want, sizeof dst_got) == 0, "put4 destination image");
        check(memcmp(memo_got, memo_want, sizeof memo_got[0] * (n + 1u)) == 0, "put4 memo");
        check(memcmp(recs, recs_copy, sizeof recs[0] * n) == 0, "put4 records unchanged");
        if (!pass) memcpy(memo_got, saved, sizeof saved[0] * (n + 1u));
    }
    b_writes += want;
    b_cases++;
}
static void test_put4(void) {
    sh2z_memo_gate4_t g;
    mo_gate_t o;
    /* the documented example layout */
    const sh2z_memo_gate4_t example = {{0xFFFF0000u, 0xFFFF0000u, 0x0000FFFFu, 0u}, 32u, {8u, 10u, 6u, 12u}, 0x8u};
    for (unsigned k = 0; k < 4u; k++) { o.gate[k] = example.gate[k]; o.offset[k] = example.offset[k]; }
    o.stride = example.stride;
    o.always = example.always;
    for (unsigned flip = 0; flip <= 100u; flip += 10u) {
        put_case(&example, &o, 388, flip, 0);
        put_case(&example, &o, 357, flip, 1);
    }
    put_case(&example, &o, 0, 50, 0);
    /* random descriptors, sizes and change rates */
    for (unsigned it = 0; it < 3000u; it++) {
        rand_desc(&g, &o);
        const uint32_t n = it < 20u ? it : below(it % 10u ? 64u : MAXR);
        const unsigned flip = below(4) ? below(101) : (below(2) ? 0u : 100u);
        put_case(&g, &o, n, flip, it % 7u == 0u);
    }
    printf("memo put4: %u cases, %u destination writes\n", b_cases, b_writes);
}

/* ---- C. Copy until an end marker ------------------------------------------ */
enum { MAXC = 1100 };
static uint16_t csrc[4 * MAXC + 8], cdst[4 * MAXC + 64], cwant[4 * MAXC + 64];
static uint32_t c_cases;
static void copy_case(uint32_t max, uint32_t lim1, uint32_t lim3, uint32_t marker_at, unsigned shift_words) {
    uint16_t *s = csrc + shift_words, *d = cdst + shift_words, *w = cwant + shift_words;
    for (uint32_t i = 0; i < 4u * max; i++) s[i] = (uint16_t)rnd();
    /* records before marker_at are not markers; the one at marker_at is */
    for (uint32_t i = 0; i < max; i++) {
        uint16_t *r = s + 4u * i;
        if (i < marker_at) {
            if (lim1 == 0u || lim3 == 0u) break;
            if (r[1] >= lim1) r[1] = (uint16_t)below(lim1 > 0xFFFFu ? 0x10000u : lim1);
            if (r[3] >= lim3) r[3] = (uint16_t)below(lim3 > 0xFFFFu ? 0x10000u : lim3);
        } else if (i == marker_at) {
            if (lim1 <= 0xFFFFu && (below(2) || lim3 > 0xFFFFu)) r[1] = (uint16_t)(lim1 + below(0x10000u - lim1));
            else if (lim3 <= 0xFFFFu) r[3] = (uint16_t)(lim3 + below(0x10000u - lim3));
        }
    }
    memset(cdst, 0xEE, sizeof cdst);
    memset(cwant, 0xEE, sizeof cwant);
    const uint32_t want = mo_copy8_until(w, s, max, lim1, lim3, MO_MUT_NONE);
    const uint32_t got = sh2z_memo_copy8_until(d, s, max, lim1, lim3);
    check(got == want, "copy8 record count");
    check(memcmp(cdst, cwant, sizeof cdst) == 0, "copy8 destination and canaries");
    c_cases++;
}
static void test_copy8(void) {
    static const uint32_t lims[][2] = {{0x8000u, 0xFF00u}, {0x10000u, 0x10000u}, {0u, 0x10000u}, {0x10000u, 0u},
                                       {0xFFFFu, 0xFFFFu}, {1u, 1u}, {0x10000u, 0xFF00u}, {0x8000u, 0x10000u}};
    for (unsigned l = 0; l < sizeof lims / sizeof lims[0]; l++)
        for (uint32_t max = 0; max <= 40u; max++)
            for (uint32_t at = 0; at <= max + 1u; at++)
                for (unsigned sh = 0; sh < 2u; sh++) copy_case(max, lims[l][0], lims[l][1], at, sh ? 1u : 0u);
    copy_case(1024, 0x8000u, 0xFF00u, 209, 0);
    copy_case(1024, 0x8000u, 0xFF00u, 1023, 0);
    copy_case(1024, 0x8000u, 0xFF00u, 2000, 0);
    for (unsigned it = 0; it < 4000u; it++) {
        const uint32_t max = below(MAXC + 1u);
        const uint32_t lim1 = below(3) ? (below(2) ? 0x10000u : below(0x10001u)) : 0x8000u;
        const uint32_t lim3 = below(3) ? (below(2) ? 0x10000u : below(0x10001u)) : 0xFF00u;
        copy_case(max, lim1, lim3, below(max + 2u), below(2));
    }
    /* reads stop at the marker: a source that ends right after it */
    for (uint32_t m = 0; m < 300u; m++) {
        uint16_t *s = (uint16_t *)malloc(8u * (m + 1u));
        check(s != NULL, "allocation");
        for (uint32_t i = 0; i < 4u * (m + 1u); i++) s[i] = (uint16_t)(rnd() & 0x7FFFu);
        s[4u * m + 1u] = 0x8000u;
        memset(cdst, 0, sizeof cdst);
        check(sh2z_memo_copy8_until(cdst, s, MAXC, 0x8000u, 0xFF00u) == m, "copy8 exact-size source");
        check(memcmp(cdst, s, 8u * (m + 1u)) == 0 && cdst[4u * (m + 1u)] == 0u, "copy8 exact-size copy");
        free(s);
        c_cases++;
    }
    printf("memo copy8: %u cases\n", c_cases);
}

/* ---- D. Record-run diff -------------------------------------------------- */
enum { MAXD = 65535 };
static uint16_t ta[4 * MAXD + 4], tb[4 * MAXD + 4];
static sh2z_memo_run_t runs_got[MAXD + 4];
static mo_run_t runs_want[MAXD + 4];
static uint32_t d_cases;
static void diff_case(uint32_t n, unsigned equal_pct, uint32_t max_runs, unsigned shift_words) {
    uint16_t *a = ta + shift_words, *b = tb + shift_words;
    for (uint32_t i = 0; i < n; i++) {
        for (unsigned j = 0; j < 4u; j++) a[4u * i + j] = b[4u * i + j] = (uint16_t)rnd();
        if (below(100) >= equal_pct) {
            if (below(2)) b[4u * i + below(4)] ^= (uint16_t)(1u << below(16));
            else for (unsigned j = 0; j < 4u; j++) b[4u * i + j] = (uint16_t)~a[4u * i + j];
        }
    }
    const uint32_t want = mo_diff8_runs((const uint8_t *)a, (const uint8_t *)b, n, runs_want, max_runs, MO_MUT_NONE);
    if (max_runs > want + 2u) max_runs = want + 2u;
    for (uint32_t i = 0; i < max_runs + 2u; i++) runs_got[i].start = runs_got[i].count = 0xBEEFu;
    const uint32_t got = sh2z_memo_diff8_runs(a, b, n, runs_got, max_runs);
    check(got == want, "diff run total");
    const uint32_t stored = want < max_runs ? want : max_runs;
    for (uint32_t i = 0; i < stored; i++)
        check(runs_got[i].start == runs_want[i].start && runs_got[i].count == runs_want[i].count, "diff run");
    for (uint32_t i = stored; i < max_runs + 2u; i++)
        check(runs_got[i].start == 0xBEEFu && runs_got[i].count == 0xBEEFu, "diff writes no run past the stored ones");
    d_cases++;
}
static void test_diff(void) {
    for (uint32_t n = 0; n <= 64u; n++)
        for (unsigned p = 0; p <= 100u; p += 25u) {
            diff_case(n, p, MAXD, 0);
            diff_case(n, p, below(8), 1);
        }
    /* every equal/differ pattern of 12 records */
    for (uint32_t pat = 0; pat < 4096u; pat++) {
        for (uint32_t i = 0; i < 12u; i++) {
            for (unsigned j = 0; j < 4u; j++) ta[4u * i + j] = tb[4u * i + j] = (uint16_t)rnd();
            if (pat >> i & 1u) tb[4u * i + below(4)] ^= 1u;
        }
        const uint32_t want = mo_diff8_runs((const uint8_t *)ta, (const uint8_t *)tb, 12, runs_want, 12, MO_MUT_NONE);
        check(sh2z_memo_diff8_runs(ta, tb, 12, runs_got, 12) == want, "diff pattern total");
        for (uint32_t i = 0; i < want; i++)
            check(runs_got[i].start == runs_want[i].start && runs_got[i].count == runs_want[i].count, "diff pattern run");
        d_cases++;
    }
    for (unsigned it = 0; it < 2000u; it++) diff_case(below(1000), below(101), below(4) ? MAXD : below(20), below(2));
    diff_case(208, 66, MAXD, 0);
    diff_case(MAXD, 50, MAXD, 0);
    diff_case(MAXD, 0, MAXD, 0);
    diff_case(MAXD, 100, MAXD, 0);
    printf("memo diff8: %u cases\n", d_cases);
}

/* ---- negative controls --------------------------------------------------- */
/* Count the cases of a fixed random sweep where the mutated oracle disagrees
 * with the library. */
static unsigned mutant_differences(unsigned mut) {
    unsigned found = 0;
    state = UINT32_C(0x900d5eed) + mut;
    for (unsigned it = 0; it < 200u; it++) {
        /* A */
        {
            const bool longs = it & 1u;
            const size_t bsize = longs ? 64u : 32u;
            const uint32_t groups = 1u + below(80);
            uint8_t *s = (uint8_t *)arena_shadow + PAD, *d = (uint8_t *)arena_src + PAD;
            fill_pair(s, d, bsize * groups, 2u + below(3));
            memcpy(want_shadow, s, bsize * groups);
            const uint32_t w = mo_sync_batch(want_shadow, d, bsize, groups, mask_want, mut);
            const uint32_t words = (groups + 31u) / 32u;
            for (uint32_t i = 0; i < words; i++) mask_got[i] = 0xA5A5A5A5u;
            /* the oracle leaves a skipped tail word as is: start it at the sentinel too */
            if (mut == MO_MUT_MASK_TAIL && groups % 32u) mask_want[words - 1u] = 0xA5A5A5A5u;
            const uint32_t g = longs ? sh2z_memo_sync16l_batch((uint32_t *)(void *)s, (const uint32_t *)(const void *)d, groups, mask_got)
                                     : sh2z_memo_sync16w_batch((uint16_t *)(void *)s, (const uint16_t *)(const void *)d, groups, mask_got);
            bool bad = g != w || memcmp(s, want_shadow, bsize * groups) != 0;
            for (uint32_t i = 0; i < words; i++) bad |= mask_got[i] != mask_want[i];
            found += bad;
        }
        /* B */
        {
            sh2z_memo_gate4_t g;
            mo_gate_t o;
            rand_desc(&g, &o);
            const uint32_t n = 1u + below(60);
            for (uint32_t i = 0; i < n; i++) {
                memo_got[i] = memo_want[i] = rnd();
                keys[i] = recs[i].key = below(2) ? memo_got[i] : memo_got[i] ^ (UINT32_C(1) << below(32));
                for (unsigned j = 0; j < 4u; j++) bwords[4u * i + j] = recs[i].word[j] = (uint16_t)rnd();
            }
            const uint32_t w = mo_put4(memo_want, keys, bwords, n, &o, wlist, mut);
            memset(dst_got, 0xC3, sizeof dst_got);
            memset(dst_want, 0xC3, sizeof dst_want);
            const uint32_t got = sh2z_memo_put4((volatile uint16_t *)dst_got, memo_got, recs, n, &g);
            mo_apply((uint8_t *)dst_want, wlist, w);
            found += got != w || memcmp(dst_got, dst_want, sizeof dst_got) != 0 ||
                     memcmp(memo_got, memo_want, sizeof memo_got[0] * n) != 0;
        }
        /* C */
        {
            const uint32_t max = 1u + below(40);
            for (uint32_t i = 0; i < 4u * max; i++) csrc[i] = (uint16_t)rnd();
            const uint32_t lim1 = 0x8000u + below(3) - 1u, lim3 = 0xFF00u;
            if (below(2)) csrc[4u * below(max) + 1u] = (uint16_t)lim1;
            memset(cdst, 0xEE, sizeof cdst);
            memset(cwant, 0xEE, sizeof cwant);
            const uint32_t w = mo_copy8_until(cwant, csrc, max, lim1, lim3, mut);
            found += sh2z_memo_copy8_until(cdst, csrc, max, lim1, lim3) != w || memcmp(cdst, cwant, sizeof cdst) != 0;
        }
        /* D */
        {
            const uint32_t n = below(200), max_runs = below(2) ? 1000u : below(6);
            for (uint32_t i = 0; i < n; i++)
                for (unsigned j = 0; j < 4u; j++) {
                    ta[4u * i + j] = (uint16_t)rnd();
                    tb[4u * i + j] = (uint16_t)(below(3) ? ta[4u * i + j] : rnd());
                }
            const uint32_t w = mo_diff8_runs((const uint8_t *)ta, (const uint8_t *)tb, n, runs_want, max_runs, mut);
            const uint32_t got = sh2z_memo_diff8_runs(ta, tb, n, runs_got, max_runs);
            bool bad = got != w;
            for (uint32_t i = 0; !bad && i < w && i < max_runs; i++)
                bad = runs_got[i].start != runs_want[i].start || runs_got[i].count != runs_want[i].count;
            found += bad;
        }
    }
    return found;
}
static void negative_controls(void) {
    static const char *const names[MO_MUT_COUNT] = {
        "none", "compare 15 words", "last longword not copied", "mask bit shifted", "tail mask word not written",
        "always flags ignored", "whole-key change", "memo not updated", "words written in reverse order",
        "marker not copied", "> for >=", "stored count returned", "long runs split"};
    check(mutant_differences(MO_MUT_NONE) == 0u, "unmutated oracle agrees");
    unsigned caught = 0;
    for (unsigned m = 1; m < MO_MUT_COUNT; m++) {
        const unsigned n = mutant_differences(m);
        printf("memo control '%s': %u cases differ\n", names[m], n);
        check(n > 0u, "negative control detected");
        caught++;
    }
    printf("memo: %u of %u negative controls caught\n", caught, (unsigned)(MO_MUT_COUNT - 1));
}

int main(void) {
    test_blocks();
    test_put4();
    test_copy8();
    test_diff();
    negative_controls();
    printf("memo: %llu checks passed\n", (unsigned long long)checks);
    return 0;
}
