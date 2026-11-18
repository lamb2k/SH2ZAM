/* Independent oracle for the memo-1 kernels (memo_experimental.h), shared by
 * the host tests and the SH-2 model harness. It works on byte images in the
 * caller's byte order and is written from the contract text, not from the
 * library: blocks compare byte by byte, masks are built from a flag array,
 * keyed writes are produced as an ordered write list, and runs come from
 * flag transitions. Each MO_MUT_* value plants one deliberate bug; the tests
 * require every mutant to disagree with the library (negative controls).
 * MIT; Copyright (c) 2026 SH2ZAM Contributors. */
#ifndef SH2ZAM_MEMO_ORACLE_H
#define SH2ZAM_MEMO_ORACLE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

enum {
    MO_MUT_NONE = 0,
    MO_MUT_SYNC_15_WORDS,      /* block compare skips the last word */
    MO_MUT_SYNC_NO_COPY_LAST,  /* changed block: last longword not copied */
    MO_MUT_MASK_SHIFT,         /* mask bit g + 1 instead of g */
    MO_MUT_MASK_TAIL,          /* last partial mask word not written */
    MO_MUT_PUT_NO_ALWAYS,      /* always flags ignored */
    MO_MUT_PUT_WHOLE_KEY,      /* any key change rewrites every gated word */
    MO_MUT_PUT_NO_MEMO,        /* memo never updated */
    MO_MUT_PUT_ORDER,          /* words written 3, 2, 1, 0 */
    MO_MUT_COPY_EXCLUSIVE,     /* marker record not copied */
    MO_MUT_COPY_GREATER,       /* > in place of >= */
    MO_MUT_RUNS_STORED,        /* returns the stored count, not the total */
    MO_MUT_RUNS_SPLIT,         /* a run longer than 3 is reported as two */
    MO_MUT_COUNT
};

/* ---- A ---- */
/* bytes: 32 or 64. Returns changed; copies when changed. */
static bool mo_sync(uint8_t *shadow, const uint8_t *src, size_t bytes, unsigned mut) {
    size_t cmp = bytes;
    if (mut == MO_MUT_SYNC_15_WORDS) cmp = bytes - 2u;
    bool changed = false;
    for (size_t i = 0; i < cmp; i++)
        if (shadow[i] != src[i]) changed = true;
    if (changed) {
        size_t cp = bytes;
        if (mut == MO_MUT_SYNC_NO_COPY_LAST) cp = bytes - 4u;
        for (size_t i = 0; i < cp; i++) shadow[i] = src[i];
    }
    return changed;
}
/* mask_words receives (groups + 31) / 32 words; returns the changed count. */
static uint32_t mo_sync_batch(uint8_t *shadow, const uint8_t *src, size_t bytes, uint32_t groups,
                              uint32_t *mask_words, unsigned mut) {
    static bool flag[1u << 16];
    uint32_t count = 0;
    for (uint32_t g = 0; g < groups; g++) {
        flag[g] = mo_sync(shadow + (size_t)g * bytes, src + (size_t)g * bytes, bytes, mut);
        count += flag[g];
    }
    const uint32_t words = (groups + 31u) / 32u;
    for (uint32_t w = 0; w < words; w++) {
        if (mut == MO_MUT_MASK_TAIL && w + 1u == words && groups % 32u) continue;
        uint64_t v = 0;
        for (uint32_t b = 0; b < 32u; b++) {
            const uint32_t g = 32u * w + b;
            if (g < groups && flag[g]) v |= (uint64_t)1 << (mut == MO_MUT_MASK_SHIFT ? b + 1u : b);
        }
        mask_words[w] = (uint32_t)v;
    }
    return count;
}

/* ---- B ---- */
typedef struct { uint32_t offset; uint16_t value; } mo_write_t;
typedef struct {
    uint32_t gate[4], stride, always;
    uint16_t offset[4];
} mo_gate_t;
/* Appends the writes of n records to w[] (capacity 4 n) and updates memo;
 * returns the write count. Record i: key[i], word[4 i .. 4 i + 3]. */
static uint32_t mo_put4(uint32_t *memo, const uint32_t *key, const uint16_t *word, uint32_t n,
                        const mo_gate_t *g, mo_write_t *w, unsigned mut) {
    uint32_t count = 0;
    for (uint32_t i = 0; i < n; i++) {
        const uint32_t before = memo[i], after = key[i];
        bool write[4];
        for (unsigned k = 0; k < 4u; k++) {
            bool gated = false;
            for (unsigned bit = 0; bit < 32u; bit++) {
                const uint32_t m = UINT32_C(1) << bit;
                if ((g->gate[k] & m) && ((before & m) != (after & m))) gated = true;
            }
            if (mut == MO_MUT_PUT_WHOLE_KEY) gated = g->gate[k] != 0u && before != after;
            const bool always = (g->always >> k & 1u) && mut != MO_MUT_PUT_NO_ALWAYS;
            write[k] = gated || always;
        }
        for (unsigned j = 0; j < 4u; j++) {
            const unsigned k = mut == MO_MUT_PUT_ORDER ? 3u - j : j;
            if (!write[k]) continue;
            w[count].offset = i * g->stride + g->offset[k];
            w[count].value = word[4u * i + k];
            count++;
        }
        if (mut != MO_MUT_PUT_NO_MEMO) memo[i] = after;
    }
    return count;
}
/* Applies a write list to a byte image of 16-bit words (native order). */
static void mo_apply(uint8_t *dst, const mo_write_t *w, uint32_t n) {
    for (uint32_t i = 0; i < n; i++) memcpy(dst + w[i].offset, &w[i].value, 2);
}

/* ---- C ---- */
/* Records of 4 words. Returns m; copies records 0 .. min(m, max - 1). */
static uint32_t mo_copy8_until(uint16_t *dst, const uint16_t *src, uint32_t max, uint32_t lim1,
                               uint32_t lim3, unsigned mut) {
    uint32_t m = max;
    for (uint32_t i = 0; i < max; i++) {
        const uint32_t w1 = src[4u * i + 1u], w3 = src[4u * i + 3u];
        const bool marker = mut == MO_MUT_COPY_GREATER ? (w1 > lim1 || w3 > lim3)
                                                        : (w1 >= lim1 || w3 >= lim3);
        if (marker) { m = i; break; }
    }
    uint32_t copied = m < max ? m + 1u : max;
    if (mut == MO_MUT_COPY_EXCLUSIVE && m < max) copied = m;
    for (uint32_t i = 0; i < 4u * copied; i++) dst[i] = src[i];
    return m;
}

/* ---- D ---- */
typedef struct { uint32_t start, count; } mo_run_t;
static uint32_t mo_diff8_runs(const uint8_t *a, const uint8_t *b, uint32_t n, mo_run_t *runs,
                              uint32_t max_runs, unsigned mut) {
    uint32_t total = 0;
    bool prev = false;
    uint32_t start = 0;
    for (uint32_t i = 0; i <= n; i++) {
        bool diff = false;
        if (i < n) diff = memcmp(a + 8u * (size_t)i, b + 8u * (size_t)i, 8) != 0;
        if (diff && !prev) start = i;
        if (!diff && prev) {
            uint32_t len = i - start;
            if (mut == MO_MUT_RUNS_SPLIT && len > 3u) {
                if (total < max_runs) runs[total] = (mo_run_t){start, 2u};
                total++;
                start += 2u;
                len -= 2u;
            }
            if (total < max_runs) runs[total] = (mo_run_t){start, len};
            total++;
        }
        prev = diff;
    }
    if (mut == MO_MUT_RUNS_STORED && total > max_runs) total = max_runs;
    return total;
}

#endif
