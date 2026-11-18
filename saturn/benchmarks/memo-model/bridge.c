/* SH-2 bridge for the executed-instruction memo model (contract tag memo-1).
 * MIT; Copyright (c) 2026 SH2ZAM Contributors. Every entry point is called
 * directly by the model harness with the SH-2 calling convention; the harness
 * writes the buffers before each call and reads them after it; nothing here
 * uses libc.
 *
 * Library entries (one call each):
 *   w16_batch(off, groups)       sh2z_memo_sync16w_batch, blocks at byte offset off
 *   l16_batch(groups)            sh2z_memo_sync16l_batch
 *   w16_one(off, g), l16_one(g)  sh2z_memo_sync16w / sync16l on block g
 *   w16_loop(groups)             sh2z_memo_sync16w called once per block
 *   put4(n)                      sh2z_memo_put4 with the descriptor in gate
 *   put4_one(n)                  sh2z_memo_put4_one in the caller's loop
 *   copy8(off, max, l1, l3)      sh2z_memo_copy8_until
 *   diff8(n, max_runs)           sh2z_memo_diff8_runs
 * Controls (plain C, not library code), written the way the shapes describe
 * today's code:
 *   plain_w15(groups)   per-block call; compare 7 longwords and 1 word, copy
 *                       the 15 words and set a flag on a difference
 *   plain_memo(n)       per record: two key bytes and a 16-bit key compared
 *                       with a 4-byte memo record, words 0 and 1, word 2 and
 *                       word 3 written at fixed offsets (the documented
 *                       example layout: offsets 8, 10, 6, 12, stride 32)
 *   plain_all(n)        the same four words written for every record, no memo
 *   plain_copy8(max)    word copy, testing word 1 >= 0x8000 and word 3 >= 0xFF00
 *   full_copy(max)      longword copy of max records with no test
 *   diff8_c(n, max_runs) the library's portable C diff (sh2zi_memo_diff8_runs_c,
 *                       the path 2-byte-aligned tables take) */
#include "sh2zam/memo_experimental.h"

#define NOINLINE __attribute__((noinline))
#define MAXG 160u
#define MAXR 640u
#define MAXC 1024u
#define MAXD 512u

typedef uint32_t alias32 __attribute__((may_alias));
typedef uint16_t alias16 __attribute__((may_alias));

/* A: 64 bytes per block covers both shapes, plus 64 bytes of guard */
uint32_t a_shadow[MAXG * 16u + 16u] __attribute__((aligned(16)));
uint32_t a_src[MAXG * 16u + 16u] __attribute__((aligned(16)));
uint32_t a_mask[MAXG / 32u + 2u];
uint8_t a_flag[MAXG];
/* B */
sh2z_memo_rec4_t b_rec[MAXR];
uint32_t b_memo[MAXR];
uint16_t b_dst[MAXR * 16u + 64u] __attribute__((aligned(16)));
sh2z_memo_gate4_t gate;
/* C */
uint16_t c_src[4u * MAXC + 32u] __attribute__((aligned(16)));
uint16_t c_dst[4u * MAXC + 32u] __attribute__((aligned(16)));
/* D */
uint16_t d_a[4u * MAXD + 2u] __attribute__((aligned(16)));
uint16_t d_b[4u * MAXD + 2u] __attribute__((aligned(16)));
sh2z_memo_run_t d_runs[MAXD];

NOINLINE uint32_t layout(void) {
    return (uint32_t)sizeof(sh2z_memo_gate4_t) << 16 | (uint32_t)sizeof(sh2z_memo_rec4_t) << 8 |
           (uint32_t)__builtin_offsetof(sh2z_memo_gate4_t, always);
}

/* ---- A ---- */
NOINLINE uint32_t w16_batch(uint32_t off, uint32_t groups) {
    return sh2z_memo_sync16w_batch((uint16_t *)((uint8_t *)a_shadow + off),
                                   (const uint16_t *)((const uint8_t *)a_src + off), groups, a_mask);
}
NOINLINE uint32_t l16_batch(uint32_t groups) {
    return sh2z_memo_sync16l_batch(a_shadow, a_src, groups, a_mask);
}
NOINLINE uint32_t w16_one(uint32_t off, uint32_t g) {
    return sh2z_memo_sync16w((uint16_t *)((uint8_t *)a_shadow + off) + 16u * g,
                             (const uint16_t *)((const uint8_t *)a_src + off) + 16u * g);
}
NOINLINE uint32_t l16_one(uint32_t g) {
    return sh2z_memo_sync16l(a_shadow + 16u * g, a_src + 16u * g);
}
NOINLINE uint32_t w16_loop(uint32_t groups) {
    uint32_t count = 0;
    for (uint32_t g = 0; g < groups; g++) {
        const bool c = sh2z_memo_sync16w((uint16_t *)a_shadow + 16u * g, (const uint16_t *)a_src + 16u * g);
        a_flag[g] = c;
        count += c;
    }
    return count;
}
static NOINLINE int plain_group(alias32 *s, const alias32 *r) {
    for (unsigned k = 0; k < 7u; k++)
        if (s[k] != r[k]) goto differ;
    if (((alias16 *)s)[14] != ((const alias16 *)r)[14]) goto differ;
    return 0;
differ:
    for (unsigned k = 0; k < 7u; k++) s[k] = r[k];
    ((alias16 *)s)[14] = ((const alias16 *)r)[14];
    return 1;
}
NOINLINE uint32_t plain_w15(uint32_t groups) {
    uint32_t count = 0;
    for (uint32_t g = 0; g < groups; g++) {
        const int c = plain_group(a_shadow + 8u * g, a_src + 8u * g);
        a_flag[g] = (uint8_t)c;
        count += (uint32_t)c;
    }
    return count;
}

/* ---- B ---- */
NOINLINE uint32_t put4(uint32_t n) {
    return sh2z_memo_put4(b_dst, b_memo, b_rec, n, &gate);
}
NOINLINE uint32_t put4_one(uint32_t n) {
    uint32_t writes = 0;
    volatile uint8_t *d = (volatile uint8_t *)b_dst;
    for (uint32_t i = 0; i < n; i++, d += gate.stride)
        writes += sh2z_memo_put4_one((volatile uint16_t *)(volatile void *)d, b_memo + i, b_rec[i].key, b_rec[i].word, &gate);
    return writes;
}
NOINLINE uint32_t plain_memo(uint32_t n) {
    volatile uint16_t *d = b_dst;
    uint8_t *m = (uint8_t *)b_memo;
    uint32_t writes = 0;
    for (uint32_t i = 0; i < n; i++, d += 16, m += 4) {
        const uint32_t key = b_rec[i].key;
        const uint8_t k0 = (uint8_t)(key >> 24), k1 = (uint8_t)(key >> 16);
        const uint16_t kb = (uint16_t)key;
        if (m[0] != k0 || m[1] != k1) {
            d[4] = b_rec[i].word[0];
            d[5] = b_rec[i].word[1];
            m[0] = k0;
            m[1] = k1;
            writes += 2;
        }
        if (*(alias16 *)(m + 2) != kb) {
            d[3] = b_rec[i].word[2];
            *(alias16 *)(m + 2) = kb;
            writes++;
        }
        d[6] = b_rec[i].word[3];
        writes++;
    }
    return writes;
}
NOINLINE uint32_t plain_all(uint32_t n) {
    volatile uint16_t *d = b_dst;
    for (uint32_t i = 0; i < n; i++, d += 16) {
        d[4] = b_rec[i].word[0];
        d[5] = b_rec[i].word[1];
        d[3] = b_rec[i].word[2];
        d[6] = b_rec[i].word[3];
    }
    return 4u * n;
}

/* ---- C ---- */
NOINLINE uint32_t copy8(uint32_t off, uint32_t max, uint32_t l1, uint32_t l3) {
    return sh2z_memo_copy8_until(c_dst + off / 2u, c_src + off / 2u, max, l1, l3);
}
NOINLINE uint32_t plain_copy8(uint32_t max) {
    for (uint32_t i = 0; i < max; i++) {
        const uint16_t *s = c_src + 4u * i;
        uint16_t *d = c_dst + 4u * i;
        d[0] = s[0];
        d[1] = s[1];
        d[2] = s[2];
        d[3] = s[3];
        if (s[1] >= 0x8000u || s[3] >= 0xFF00u)
            return i;
    }
    return max;
}
/* GCC would turn this loop into a memcpy call, which the model image lacks. */
__attribute__((optimize("no-tree-loop-distribute-patterns")))
NOINLINE uint32_t full_copy(uint32_t max) {
    alias32 *d = (alias32 *)c_dst;
    const alias32 *s = (const alias32 *)c_src;
    for (uint32_t i = 0; i < 2u * max; i++)
        d[i] = s[i];
    return max;
}

/* ---- D ---- */
NOINLINE uint32_t diff8(uint32_t n, uint32_t max_runs) {
    return sh2z_memo_diff8_runs(d_a, d_b, n, d_runs, max_runs);
}
uint32_t sh2zi_memo_diff8_runs_c(const uint16_t *a, const uint16_t *b, uint32_t n,
                                 sh2z_memo_run_t *runs, uint32_t max_runs);
NOINLINE uint32_t diff8_c(uint32_t n, uint32_t max_runs) {
    return sh2zi_memo_diff8_runs_c(d_a, d_b, n, d_runs, max_runs);
}
