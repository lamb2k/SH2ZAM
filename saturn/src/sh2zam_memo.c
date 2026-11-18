/* Block memo kernels (memo_experimental.h, contract tag memo-1): block
 * compare-and-copy with changed flags, keyed write-through to a 16-bit
 * destination, a record copy that stops at an end marker and a record-run
 * diff. Portable C, and SH-2 assembly for the block and copy kernels. No libc.
 * MIT; Copyright (c) 2026 SH2ZAM Contributors. */
#include "sh2zam/memo_experimental.h"

#if defined(__sh__) && defined(__SH2__) && (!defined(SH2Z_FORCE_PORTABLE) || !SH2Z_FORCE_PORTABLE)
#define SH2ZI_MEMO_NATIVE 1
#else
#define SH2ZI_MEMO_NATIVE 0
#endif

/* GCC turns some copy loops into memcpy calls even when freestanding; the
 * library has no memcpy. */
#if defined(__GNUC__) && !defined(__clang__)
#define MEMO_NO_LIBCALL __attribute__((optimize("no-tree-loop-distribute-patterns")))
#else
#define MEMO_NO_LIBCALL
#endif

/* Portable paths. On SH-2 they serve the 2-byte-aligned cases the kernels
 * leave out; the kernels jump to them by name. */
bool sh2zi_memo_sync16w_c(uint16_t *shadow, const uint16_t *src);
uint32_t sh2zi_memo_sync16w_batch_c(uint16_t *shadow, const uint16_t *src,
                                    uint32_t groups, uint32_t *changed);
uint32_t sh2zi_memo_copy8_until_c(uint16_t *dst, const uint16_t *src,
                                  uint32_t max_records,
                                  uint32_t w1_limit, uint32_t w3_limit);
uint32_t sh2zi_memo_put4_c(volatile uint16_t *dst, uint32_t *memo,
                           const sh2z_memo_rec4_t *rec, uint32_t n,
                           const sh2z_memo_gate4_t *gate);
uint32_t sh2zi_memo_diff8_runs_c(const uint16_t *a, const uint16_t *b, uint32_t n,
                                 sh2z_memo_run_t *runs, uint32_t max_runs);

/* ---- A. Block compare-and-copy ---- */

MEMO_NO_LIBCALL bool sh2zi_memo_sync16w_c(uint16_t *shadow, const uint16_t *src) {
    uint32_t d = 0;
    for (unsigned k = 0; k < 16u; k++)
        d |= (uint32_t)(shadow[k] ^ src[k]);
    if (d == 0u)
        return false;
    for (unsigned k = 0; k < 16u; k++)
        shadow[k] = src[k];
    return true;
}

uint32_t sh2zi_memo_sync16w_batch_c(uint16_t *shadow, const uint16_t *src,
                                    uint32_t groups, uint32_t *changed) {
    /* A running bit, not 1 << (g & 31): the SH-2 has no variable shift. */
    uint32_t count = 0, word = 0, bit = 1;
    for (uint32_t g = 0; g < groups; g++) {
        if (sh2zi_memo_sync16w_c(shadow + 16u * g, src + 16u * g)) {
            word |= bit;
            count++;
        }
        bit <<= 1;
        if (bit == 0u || g + 1u == groups) {
            changed[g >> 5] = word;
            word = 0;
            bit = 1;
        }
    }
    return count;
}

#if !SH2ZI_MEMO_NATIVE
MEMO_NO_LIBCALL static bool memo_sync16l_c(uint32_t *shadow, const uint32_t *src) {
    uint32_t d = 0;
    for (unsigned k = 0; k < 16u; k++)
        d |= shadow[k] ^ src[k];
    if (d == 0u)
        return false;
    for (unsigned k = 0; k < 16u; k++)
        shadow[k] = src[k];
    return true;
}

static uint32_t memo_sync16l_batch_c(uint32_t *shadow, const uint32_t *src,
                                     uint32_t groups, uint32_t *changed) {
    /* A running bit, not 1 << (g & 31): the SH-2 has no variable shift. */
    uint32_t count = 0, word = 0, bit = 1;
    for (uint32_t g = 0; g < groups; g++) {
        if (memo_sync16l_c(shadow + 16u * g, src + 16u * g)) {
            word |= bit;
            count++;
        }
        bit <<= 1;
        if (bit == 0u || g + 1u == groups) {
            changed[g >> 5] = word;
            word = 0;
            bit = 1;
        }
    }
    return count;
}
#endif

/* ---- C. Copy until an end marker ---- */

MEMO_NO_LIBCALL uint32_t sh2zi_memo_copy8_until_c(uint16_t *dst, const uint16_t *src,
                                                  uint32_t max_records,
                                                  uint32_t w1_limit, uint32_t w3_limit) {
    for (uint32_t i = 0; i < max_records; i++) {
        const uint16_t *s = src + 4u * i;
        uint16_t *d = dst + 4u * i;
        const uint16_t w0 = s[0], w1 = s[1], w2 = s[2], w3 = s[3];
        d[0] = w0;
        d[1] = w1;
        d[2] = w2;
        d[3] = w3;
        if ((uint32_t)w1 >= w1_limit || (uint32_t)w3 >= w3_limit)
            return i;
    }
    return max_records;
}

/* ---- B. Keyed write-through ---- */

uint32_t sh2zi_memo_put4_c(volatile uint16_t *dst, uint32_t *memo,
                           const sh2z_memo_rec4_t *rec, uint32_t n,
                           const sh2z_memo_gate4_t *gate) {
    /* The descriptor is read once into locals (field by field: a struct
     * copy could become a memcpy call). */
    const uint32_t g0 = gate->gate[0], g1 = gate->gate[1];
    const uint32_t g2 = gate->gate[2], g3 = gate->gate[3];
    const uint32_t a = gate->always, stride = gate->stride;
    const bool a0 = (a & 1u) != 0u, a1 = (a & 2u) != 0u;
    const bool a2 = (a & 4u) != 0u, a3 = (a & 8u) != 0u;
    const uint32_t o0 = gate->offset[0], o1 = gate->offset[1];
    const uint32_t o2 = gate->offset[2], o3 = gate->offset[3];
    volatile uint8_t *base = (volatile uint8_t *)dst;
    uint32_t writes = 0;
    for (uint32_t i = 0; i < n; i++, base += stride) {
        const uint32_t key = rec[i].key, d = memo[i] ^ key;
        if ((d & g0) != 0u || a0) {
            *(volatile uint16_t *)(volatile void *)(base + o0) = rec[i].word[0];
            writes++;
        }
        if ((d & g1) != 0u || a1) {
            *(volatile uint16_t *)(volatile void *)(base + o1) = rec[i].word[1];
            writes++;
        }
        if ((d & g2) != 0u || a2) {
            *(volatile uint16_t *)(volatile void *)(base + o2) = rec[i].word[2];
            writes++;
        }
        if ((d & g3) != 0u || a3) {
            *(volatile uint16_t *)(volatile void *)(base + o3) = rec[i].word[3];
            writes++;
        }
        if (d != 0u)
            memo[i] = key;
    }
    return writes;
}

/* ---- D. Record-run diff ---- */

static bool memo_same8(const uint16_t *a, const uint16_t *b) {
    return ((uint32_t)(a[0] ^ b[0]) | (uint32_t)(a[1] ^ b[1]) |
            (uint32_t)(a[2] ^ b[2]) | (uint32_t)(a[3] ^ b[3])) == 0u;
}

uint32_t sh2zi_memo_diff8_runs_c(const uint16_t *a, const uint16_t *b, uint32_t n,
                                 sh2z_memo_run_t *runs, uint32_t max_runs) {
    uint32_t total = 0, i = 0;
    for (;;) {
        while (i < n && memo_same8(a + 4u * i, b + 4u * i))
            i++;
        if (i == n)
            return total;
        const uint32_t start = i;
        while (i < n && !memo_same8(a + 4u * i, b + 4u * i))
            i++;
        if (total < max_runs) {
            runs[total].start = (uint16_t)start;
            runs[total].count = (uint16_t)(i - start);
        }
        total++;
    }
}

#if SH2ZI_MEMO_NATIVE
/* ---- SH-2 kernels ----
 * Block kernels. A block of L longwords (L = 8 for sync16w, 16 for sync16l)
 * is compared as L longword pairs: r0 = OR of (shadow[k] ^ src[k]), each
 * loaded register used two instructions after its load (the SH-2 stalls a
 * cycle on an immediate use). An unchanged block costs 4 L - 1 instructions
 * for the compare plus 8 for the loop; a changed one is copied from src (now
 * in the cache) with 2 L moves.
 *
 * The batch kernels keep the current mask bit in r8 and the mask word in r9.
 * A changed block ORs r8 into r9; SHLL r8 shifts the bit out after block 31
 * of a word, which stores r9 and starts the next word. After the loop a
 * partial word (r8 != 1) is stored.
 *
 * The 16-word kernels need both pointers 4-byte aligned (MOV.L) and jump to
 * the portable path otherwise; uint32_t blocks are aligned by type.
 * Callee-saved r8-r10 are saved and restored by the batch kernels; MACH,
 * MACL, GBR, r14 and PR are not touched. */
/* ---- generated: block kernels ---- */
__asm__(
"    .text\n"
"    .align 2\n"
"    .global _sh2z_memo_sync16w\n"
"    .type _sh2z_memo_sync16w,@function\n"
"_sh2z_memo_sync16w:\n"
"    mov     r4,r0\n"
"    or      r5,r0\n"
"    tst     #3,r0\n"
"    bf      .Lsh2zi_memo_s8_portable\n"
"    mov.l   @r4,r0\n"
"    mov.l   @r5,r1\n"
"    mov.l   @(4,r4),r2\n"
"    xor     r1,r0\n"
"    mov.l   @(4,r5),r1\n"
"    mov.l   @(8,r4),r3\n"
"    xor     r2,r1\n"
"    or      r1,r0\n"
"    mov.l   @(8,r5),r2\n"
"    mov.l   @(12,r4),r1\n"
"    xor     r3,r2\n"
"    or      r2,r0\n"
"    mov.l   @(12,r5),r3\n"
"    mov.l   @(16,r4),r2\n"
"    xor     r1,r3\n"
"    or      r3,r0\n"
"    mov.l   @(16,r5),r1\n"
"    mov.l   @(20,r4),r3\n"
"    xor     r2,r1\n"
"    or      r1,r0\n"
"    mov.l   @(20,r5),r2\n"
"    mov.l   @(24,r4),r1\n"
"    xor     r3,r2\n"
"    or      r2,r0\n"
"    mov.l   @(24,r5),r3\n"
"    mov.l   @(28,r4),r2\n"
"    xor     r1,r3\n"
"    or      r3,r0\n"
"    mov.l   @(28,r5),r1\n"
"    xor     r2,r1\n"
"    or      r1,r0\n"
"    tst     r0,r0\n"
"    bt      .Lsh2zi_memo_s8_same\n"
"    mov.l   @r5,r0\n"
"    mov.l   @(4,r5),r1\n"
"    mov.l   @(8,r5),r2\n"
"    mov.l   @(12,r5),r3\n"
"    mov.l   r0,@r4\n"
"    mov.l   r1,@(4,r4)\n"
"    mov.l   r2,@(8,r4)\n"
"    mov.l   r3,@(12,r4)\n"
"    mov.l   @(16,r5),r0\n"
"    mov.l   @(20,r5),r1\n"
"    mov.l   @(24,r5),r2\n"
"    mov.l   @(28,r5),r3\n"
"    mov.l   r0,@(16,r4)\n"
"    mov.l   r1,@(20,r4)\n"
"    mov.l   r2,@(24,r4)\n"
"    mov.l   r3,@(28,r4)\n"
"    rts\n"
"    mov     #1,r0\n"
".Lsh2zi_memo_s8_same:\n"
"    rts\n"
"    mov     #0,r0\n"
".Lsh2zi_memo_s8_portable:\n"
"    mov.l   .Lsh2zi_memo_s8_c,r0\n"
"    jmp     @r0\n"
"    nop\n"
"    .align 2\n"
".Lsh2zi_memo_s8_c:\n"
"    .long   _sh2zi_memo_sync16w_c\n"
"    .size _sh2z_memo_sync16w,.-_sh2z_memo_sync16w\n"
);
__asm__(
"    .text\n"
"    .align 2\n"
"    .global _sh2z_memo_sync16l\n"
"    .type _sh2z_memo_sync16l,@function\n"
"_sh2z_memo_sync16l:\n"
"    mov.l   @r4,r0\n"
"    mov.l   @r5,r1\n"
"    mov.l   @(4,r4),r2\n"
"    xor     r1,r0\n"
"    mov.l   @(4,r5),r1\n"
"    mov.l   @(8,r4),r3\n"
"    xor     r2,r1\n"
"    or      r1,r0\n"
"    mov.l   @(8,r5),r2\n"
"    mov.l   @(12,r4),r1\n"
"    xor     r3,r2\n"
"    or      r2,r0\n"
"    mov.l   @(12,r5),r3\n"
"    mov.l   @(16,r4),r2\n"
"    xor     r1,r3\n"
"    or      r3,r0\n"
"    mov.l   @(16,r5),r1\n"
"    mov.l   @(20,r4),r3\n"
"    xor     r2,r1\n"
"    or      r1,r0\n"
"    mov.l   @(20,r5),r2\n"
"    mov.l   @(24,r4),r1\n"
"    xor     r3,r2\n"
"    or      r2,r0\n"
"    mov.l   @(24,r5),r3\n"
"    mov.l   @(28,r4),r2\n"
"    xor     r1,r3\n"
"    or      r3,r0\n"
"    mov.l   @(28,r5),r1\n"
"    mov.l   @(32,r4),r3\n"
"    xor     r2,r1\n"
"    or      r1,r0\n"
"    mov.l   @(32,r5),r2\n"
"    mov.l   @(36,r4),r1\n"
"    xor     r3,r2\n"
"    or      r2,r0\n"
"    mov.l   @(36,r5),r3\n"
"    mov.l   @(40,r4),r2\n"
"    xor     r1,r3\n"
"    or      r3,r0\n"
"    mov.l   @(40,r5),r1\n"
"    mov.l   @(44,r4),r3\n"
"    xor     r2,r1\n"
"    or      r1,r0\n"
"    mov.l   @(44,r5),r2\n"
"    mov.l   @(48,r4),r1\n"
"    xor     r3,r2\n"
"    or      r2,r0\n"
"    mov.l   @(48,r5),r3\n"
"    mov.l   @(52,r4),r2\n"
"    xor     r1,r3\n"
"    or      r3,r0\n"
"    mov.l   @(52,r5),r1\n"
"    mov.l   @(56,r4),r3\n"
"    xor     r2,r1\n"
"    or      r1,r0\n"
"    mov.l   @(56,r5),r2\n"
"    mov.l   @(60,r4),r1\n"
"    xor     r3,r2\n"
"    or      r2,r0\n"
"    mov.l   @(60,r5),r3\n"
"    xor     r1,r3\n"
"    or      r3,r0\n"
"    tst     r0,r0\n"
"    bt      .Lsh2zi_memo_s16_same\n"
"    mov.l   @r5,r0\n"
"    mov.l   @(4,r5),r1\n"
"    mov.l   @(8,r5),r2\n"
"    mov.l   @(12,r5),r3\n"
"    mov.l   r0,@r4\n"
"    mov.l   r1,@(4,r4)\n"
"    mov.l   r2,@(8,r4)\n"
"    mov.l   r3,@(12,r4)\n"
"    mov.l   @(16,r5),r0\n"
"    mov.l   @(20,r5),r1\n"
"    mov.l   @(24,r5),r2\n"
"    mov.l   @(28,r5),r3\n"
"    mov.l   r0,@(16,r4)\n"
"    mov.l   r1,@(20,r4)\n"
"    mov.l   r2,@(24,r4)\n"
"    mov.l   r3,@(28,r4)\n"
"    mov.l   @(32,r5),r0\n"
"    mov.l   @(36,r5),r1\n"
"    mov.l   @(40,r5),r2\n"
"    mov.l   @(44,r5),r3\n"
"    mov.l   r0,@(32,r4)\n"
"    mov.l   r1,@(36,r4)\n"
"    mov.l   r2,@(40,r4)\n"
"    mov.l   r3,@(44,r4)\n"
"    mov.l   @(48,r5),r0\n"
"    mov.l   @(52,r5),r1\n"
"    mov.l   @(56,r5),r2\n"
"    mov.l   @(60,r5),r3\n"
"    mov.l   r0,@(48,r4)\n"
"    mov.l   r1,@(52,r4)\n"
"    mov.l   r2,@(56,r4)\n"
"    mov.l   r3,@(60,r4)\n"
"    rts\n"
"    mov     #1,r0\n"
".Lsh2zi_memo_s16_same:\n"
"    rts\n"
"    mov     #0,r0\n"
"    .size _sh2z_memo_sync16l,.-_sh2z_memo_sync16l\n"
);
__asm__(
"    .text\n"
"    .align 2\n"
"    .global _sh2z_memo_sync16w_batch\n"
"    .type _sh2z_memo_sync16w_batch,@function\n"
"_sh2z_memo_sync16w_batch:\n"
"    mov     r4,r0\n"
"    or      r5,r0\n"
"    tst     #3,r0\n"
"    bf      .Lsh2zi_memo_8_portable\n"
"    tst     r6,r6\n"
"    bt/s    .Lsh2zi_memo_8_zero\n"
"    mov     #0,r0\n"
"    mov.l   r8,@-r15\n"
"    mov.l   r9,@-r15\n"
"    mov.l   r10,@-r15\n"
"    mov     #1,r8\n"
"    mov     #0,r9\n"
"    mov     #0,r3\n"
".Lsh2zi_memo_8_loop:\n"
"    mov.l   @r4,r0\n"
"    mov.l   @r5,r1\n"
"    mov.l   @(4,r4),r2\n"
"    xor     r1,r0\n"
"    mov.l   @(4,r5),r1\n"
"    mov.l   @(8,r4),r10\n"
"    xor     r2,r1\n"
"    or      r1,r0\n"
"    mov.l   @(8,r5),r2\n"
"    mov.l   @(12,r4),r1\n"
"    xor     r10,r2\n"
"    or      r2,r0\n"
"    mov.l   @(12,r5),r10\n"
"    mov.l   @(16,r4),r2\n"
"    xor     r1,r10\n"
"    or      r10,r0\n"
"    mov.l   @(16,r5),r1\n"
"    mov.l   @(20,r4),r10\n"
"    xor     r2,r1\n"
"    or      r1,r0\n"
"    mov.l   @(20,r5),r2\n"
"    mov.l   @(24,r4),r1\n"
"    xor     r10,r2\n"
"    or      r2,r0\n"
"    mov.l   @(24,r5),r10\n"
"    mov.l   @(28,r4),r2\n"
"    xor     r1,r10\n"
"    or      r10,r0\n"
"    mov.l   @(28,r5),r1\n"
"    xor     r2,r1\n"
"    or      r1,r0\n"
"    tst     r0,r0\n"
"    bf      .Lsh2zi_memo_8_changed\n"
".Lsh2zi_memo_8_next:\n"
"    shll    r8\n"
"    bt      .Lsh2zi_memo_8_flush\n"
".Lsh2zi_memo_8_cont:\n"
"    add     #32,r4\n"
"    dt      r6\n"
"    bf/s    .Lsh2zi_memo_8_loop\n"
"    add     #32,r5\n"
"    mov     r8,r0\n"
"    cmp/eq  #1,r0\n"
"    bt      .Lsh2zi_memo_8_done\n"
"    mov.l   r9,@r7\n"
".Lsh2zi_memo_8_done:\n"
"    mov     r3,r0\n"
"    mov.l   @r15+,r10\n"
"    mov.l   @r15+,r9\n"
"    rts\n"
"    mov.l   @r15+,r8\n"
".Lsh2zi_memo_8_zero:\n"
"    rts\n"
"    nop\n"
".Lsh2zi_memo_8_flush:\n"
"    mov.l   r9,@r7\n"
"    add     #4,r7\n"
"    mov     #0,r9\n"
"    bra     .Lsh2zi_memo_8_cont\n"
"    mov     #1,r8\n"
".Lsh2zi_memo_8_changed:\n"
"    or      r8,r9\n"
"    add     #1,r3\n"
"    mov.l   @r5,r0\n"
"    mov.l   @(4,r5),r1\n"
"    mov.l   @(8,r5),r2\n"
"    mov.l   @(12,r5),r10\n"
"    mov.l   r0,@r4\n"
"    mov.l   r1,@(4,r4)\n"
"    mov.l   r2,@(8,r4)\n"
"    mov.l   r10,@(12,r4)\n"
"    mov.l   @(16,r5),r0\n"
"    mov.l   @(20,r5),r1\n"
"    mov.l   @(24,r5),r2\n"
"    mov.l   @(28,r5),r10\n"
"    mov.l   r0,@(16,r4)\n"
"    mov.l   r1,@(20,r4)\n"
"    mov.l   r2,@(24,r4)\n"
"    bra     .Lsh2zi_memo_8_next\n"
"    mov.l   r10,@(28,r4)\n"
".Lsh2zi_memo_8_portable:\n"
"    mov.l   .Lsh2zi_memo_8_c,r0\n"
"    jmp     @r0\n"
"    nop\n"
"    .align 2\n"
".Lsh2zi_memo_8_c:\n"
"    .long   _sh2zi_memo_sync16w_batch_c\n"
"    .size _sh2z_memo_sync16w_batch,.-_sh2z_memo_sync16w_batch\n"
);
__asm__(
"    .text\n"
"    .align 2\n"
"    .global _sh2z_memo_sync16l_batch\n"
"    .type _sh2z_memo_sync16l_batch,@function\n"
"_sh2z_memo_sync16l_batch:\n"
"    tst     r6,r6\n"
"    bt/s    .Lsh2zi_memo_16_zero\n"
"    mov     #0,r0\n"
"    mov.l   r8,@-r15\n"
"    mov.l   r9,@-r15\n"
"    mov.l   r10,@-r15\n"
"    mov     #1,r8\n"
"    mov     #0,r9\n"
"    mov     #0,r3\n"
".Lsh2zi_memo_16_loop:\n"
"    mov.l   @r4,r0\n"
"    mov.l   @r5,r1\n"
"    mov.l   @(4,r4),r2\n"
"    xor     r1,r0\n"
"    mov.l   @(4,r5),r1\n"
"    mov.l   @(8,r4),r10\n"
"    xor     r2,r1\n"
"    or      r1,r0\n"
"    mov.l   @(8,r5),r2\n"
"    mov.l   @(12,r4),r1\n"
"    xor     r10,r2\n"
"    or      r2,r0\n"
"    mov.l   @(12,r5),r10\n"
"    mov.l   @(16,r4),r2\n"
"    xor     r1,r10\n"
"    or      r10,r0\n"
"    mov.l   @(16,r5),r1\n"
"    mov.l   @(20,r4),r10\n"
"    xor     r2,r1\n"
"    or      r1,r0\n"
"    mov.l   @(20,r5),r2\n"
"    mov.l   @(24,r4),r1\n"
"    xor     r10,r2\n"
"    or      r2,r0\n"
"    mov.l   @(24,r5),r10\n"
"    mov.l   @(28,r4),r2\n"
"    xor     r1,r10\n"
"    or      r10,r0\n"
"    mov.l   @(28,r5),r1\n"
"    mov.l   @(32,r4),r10\n"
"    xor     r2,r1\n"
"    or      r1,r0\n"
"    mov.l   @(32,r5),r2\n"
"    mov.l   @(36,r4),r1\n"
"    xor     r10,r2\n"
"    or      r2,r0\n"
"    mov.l   @(36,r5),r10\n"
"    mov.l   @(40,r4),r2\n"
"    xor     r1,r10\n"
"    or      r10,r0\n"
"    mov.l   @(40,r5),r1\n"
"    mov.l   @(44,r4),r10\n"
"    xor     r2,r1\n"
"    or      r1,r0\n"
"    mov.l   @(44,r5),r2\n"
"    mov.l   @(48,r4),r1\n"
"    xor     r10,r2\n"
"    or      r2,r0\n"
"    mov.l   @(48,r5),r10\n"
"    mov.l   @(52,r4),r2\n"
"    xor     r1,r10\n"
"    or      r10,r0\n"
"    mov.l   @(52,r5),r1\n"
"    mov.l   @(56,r4),r10\n"
"    xor     r2,r1\n"
"    or      r1,r0\n"
"    mov.l   @(56,r5),r2\n"
"    mov.l   @(60,r4),r1\n"
"    xor     r10,r2\n"
"    or      r2,r0\n"
"    mov.l   @(60,r5),r10\n"
"    xor     r1,r10\n"
"    or      r10,r0\n"
"    tst     r0,r0\n"
"    bf      .Lsh2zi_memo_16_changed\n"
".Lsh2zi_memo_16_next:\n"
"    shll    r8\n"
"    bt      .Lsh2zi_memo_16_flush\n"
".Lsh2zi_memo_16_cont:\n"
"    add     #64,r4\n"
"    dt      r6\n"
"    bf/s    .Lsh2zi_memo_16_loop\n"
"    add     #64,r5\n"
"    mov     r8,r0\n"
"    cmp/eq  #1,r0\n"
"    bt      .Lsh2zi_memo_16_done\n"
"    mov.l   r9,@r7\n"
".Lsh2zi_memo_16_done:\n"
"    mov     r3,r0\n"
"    mov.l   @r15+,r10\n"
"    mov.l   @r15+,r9\n"
"    rts\n"
"    mov.l   @r15+,r8\n"
".Lsh2zi_memo_16_zero:\n"
"    rts\n"
"    nop\n"
".Lsh2zi_memo_16_flush:\n"
"    mov.l   r9,@r7\n"
"    add     #4,r7\n"
"    mov     #0,r9\n"
"    bra     .Lsh2zi_memo_16_cont\n"
"    mov     #1,r8\n"
".Lsh2zi_memo_16_changed:\n"
"    or      r8,r9\n"
"    add     #1,r3\n"
"    mov.l   @r5,r0\n"
"    mov.l   @(4,r5),r1\n"
"    mov.l   @(8,r5),r2\n"
"    mov.l   @(12,r5),r10\n"
"    mov.l   r0,@r4\n"
"    mov.l   r1,@(4,r4)\n"
"    mov.l   r2,@(8,r4)\n"
"    mov.l   r10,@(12,r4)\n"
"    mov.l   @(16,r5),r0\n"
"    mov.l   @(20,r5),r1\n"
"    mov.l   @(24,r5),r2\n"
"    mov.l   @(28,r5),r10\n"
"    mov.l   r0,@(16,r4)\n"
"    mov.l   r1,@(20,r4)\n"
"    mov.l   r2,@(24,r4)\n"
"    mov.l   r10,@(28,r4)\n"
"    mov.l   @(32,r5),r0\n"
"    mov.l   @(36,r5),r1\n"
"    mov.l   @(40,r5),r2\n"
"    mov.l   @(44,r5),r10\n"
"    mov.l   r0,@(32,r4)\n"
"    mov.l   r1,@(36,r4)\n"
"    mov.l   r2,@(40,r4)\n"
"    mov.l   r10,@(44,r4)\n"
"    mov.l   @(48,r5),r0\n"
"    mov.l   @(52,r5),r1\n"
"    mov.l   @(56,r5),r2\n"
"    mov.l   @(60,r5),r10\n"
"    mov.l   r0,@(48,r4)\n"
"    mov.l   r1,@(52,r4)\n"
"    mov.l   r2,@(56,r4)\n"
"    bra     .Lsh2zi_memo_16_next\n"
"    mov.l   r10,@(60,r4)\n"
"    .size _sh2z_memo_sync16l_batch,.-_sh2z_memo_sync16l_batch\n"
);
/* ---- end of generated block kernels ---- */

/* Copy until an end marker. Big-endian: the longword at a record's byte 0 is
 * w0 << 16 | w1 and at byte 4 is w2 << 16 | w3, so EXTU.W gives w1 and w3.
 * Each longword is stored as soon as it is loaded, before the record's tests
 * (the marker record is copied too), which spaces the stores so that none
 * waits behind the previous one in the write buffer. Records go in pairs,
 * after one single record when max_records is odd; nothing after the marker
 * record is read. m is (dst pointer - dst) / 8 at the exit.
 * r4 dst, r5 src, r6 pairs left, r7 w1_limit, r3 w3_limit (the fifth
 * argument, on the stack), r8 dst at entry (callee-saved, restored). */
/* ---- generated: copy kernel ---- */
__asm__(
"    .text\n"
"    .align 2\n"
"    .global _sh2z_memo_copy8_until\n"
"    .type _sh2z_memo_copy8_until,@function\n"
"_sh2z_memo_copy8_until:\n"
"    mov     r4,r0\n"
"    or      r5,r0\n"
"    tst     #3,r0\n"
"    bf      .Lsh2zi_memo_c8_portable\n"
"    tst     r6,r6\n"
"    bt/s    .Lsh2zi_memo_c8_zero\n"
"    mov     #0,r0\n"
"    mov.l   @r15,r3\n"
"    mov.l   r8,@-r15\n"
"    mov     r4,r8\n"
"    shlr    r6\n"
"    bf      .Lsh2zi_memo_c8_even\n"
"    mov.l   @r5+,r0\n"
"    mov.l   @r5+,r2\n"
"    mov.l   r0,@r4\n"
"    extu.w  r0,r1\n"
"    cmp/hs  r7,r1\n"
"    bt      .Lsh2zi_memo_c8_ma1\n"
"    extu.w  r2,r1\n"
"    mov.l   r2,@(4,r4)\n"
"    cmp/hs  r3,r1\n"
"    bt      .Lsh2zi_memo_c8_ma\n"
"    add     #8,r4\n"
".Lsh2zi_memo_c8_even:\n"
"    tst     r6,r6\n"
"    bt      .Lsh2zi_memo_c8_none\n"
".Lsh2zi_memo_c8_pair:\n"
"    mov.l   @r5+,r0\n"
"    mov.l   @r5+,r2\n"
"    mov.l   r0,@r4\n"
"    extu.w  r0,r1\n"
"    cmp/hs  r7,r1\n"
"    bt      .Lsh2zi_memo_c8_ma1\n"
"    extu.w  r2,r1\n"
"    mov.l   r2,@(4,r4)\n"
"    cmp/hs  r3,r1\n"
"    bt      .Lsh2zi_memo_c8_ma\n"
"    mov.l   @r5+,r0\n"
"    mov.l   @r5+,r2\n"
"    mov.l   r0,@(8,r4)\n"
"    extu.w  r0,r1\n"
"    cmp/hs  r7,r1\n"
"    bt      .Lsh2zi_memo_c8_mb1\n"
"    extu.w  r2,r1\n"
"    mov.l   r2,@(12,r4)\n"
"    cmp/hs  r3,r1\n"
"    bt      .Lsh2zi_memo_c8_mb\n"
"    dt      r6\n"
"    bf/s    .Lsh2zi_memo_c8_pair\n"
"    add     #16,r4\n"
".Lsh2zi_memo_c8_none:\n"
"    bra     .Lsh2zi_memo_c8_ma\n"
"    nop\n"
".Lsh2zi_memo_c8_mb1:\n"
"    mov.l   r2,@(12,r4)\n"
".Lsh2zi_memo_c8_mb:\n"
"    bra     .Lsh2zi_memo_c8_ma\n"
"    add     #8,r4\n"
".Lsh2zi_memo_c8_ma1:\n"
"    mov.l   r2,@(4,r4)\n"
".Lsh2zi_memo_c8_ma:\n"
"    mov     r4,r0\n"
"    sub     r8,r0\n"
"    shlr2   r0\n"
"    shlr    r0\n"
"    rts\n"
"    mov.l   @r15+,r8\n"
".Lsh2zi_memo_c8_zero:\n"
"    rts\n"
"    nop\n"
".Lsh2zi_memo_c8_portable:\n"
"    mov.l   .Lsh2zi_memo_c8_c,r0\n"
"    jmp     @r0\n"
"    nop\n"
"    .align 2\n"
".Lsh2zi_memo_c8_c:\n"
"    .long   _sh2zi_memo_copy8_until_c\n"
"    .size _sh2z_memo_copy8_until,.-_sh2z_memo_copy8_until\n"
);
/* ---- end of generated copy kernel ---- */

/* Record-run diff. A record is two longword compares; an equal record costs
 * 12 cycles. r6 counts the records left (the current one included), so the
 * current index is n - r6; r11 is the start of the open run. A run is stored
 * as one longword, start << 16 | count (big-endian sh2z_memo_run_t), while
 * total < max_runs. Callee-saved r8-r11 are saved and restored. */
/* ---- generated: diff kernel ---- */
__asm__(
"    .text\n"
"    .align 2\n"
"    .global _sh2z_memo_diff8_runs\n"
"    .type _sh2z_memo_diff8_runs,@function\n"
"_sh2z_memo_diff8_runs:\n"
"    mov     r4,r0\n"
"    or      r5,r0\n"
"    tst     #3,r0\n"
"    bf      .Lsh2zi_memo_d8_portable\n"
"    mov.l   @r15,r1\n"
"    mov.l   r8,@-r15\n"
"    mov.l   r9,@-r15\n"
"    mov.l   r10,@-r15\n"
"    mov.l   r11,@-r15\n"
"    mov     r1,r8\n"
"    mov     #0,r9\n"
"    mov     r6,r10\n"
"    tst     r6,r6\n"
"    bt      .Lsh2zi_memo_d8_done\n"
".Lsh2zi_memo_d8_eq:\n"
"    mov.l   @r4+,r0\n"
"    mov.l   @r5+,r1\n"
"    cmp/eq  r0,r1\n"
"    bf      .Lsh2zi_memo_d8_f0\n"
"    mov.l   @r4+,r0\n"
"    mov.l   @r5+,r1\n"
"    cmp/eq  r0,r1\n"
"    bf      .Lsh2zi_memo_d8_f1\n"
"    dt      r6\n"
"    bf      .Lsh2zi_memo_d8_eq\n"
"    bra     .Lsh2zi_memo_d8_done\n"
"    nop\n"
".Lsh2zi_memo_d8_f0:\n"
"    add     #4,r4\n"
"    add     #4,r5\n"
".Lsh2zi_memo_d8_f1:\n"
"    mov     r10,r11\n"
"    sub     r6,r11\n"
"    dt      r6\n"
"    bt      .Lsh2zi_memo_d8_end\n"
".Lsh2zi_memo_d8_df:\n"
"    mov.l   @r4+,r0\n"
"    mov.l   @r5+,r1\n"
"    cmp/eq  r0,r1\n"
"    bf      .Lsh2zi_memo_d8_n0\n"
"    mov.l   @r4+,r0\n"
"    mov.l   @r5+,r1\n"
"    cmp/eq  r0,r1\n"
"    bf      .Lsh2zi_memo_d8_n1\n"
"    mov     r10,r0\n"
"    sub     r6,r0\n"
"    sub     r11,r0\n"
"    cmp/hs  r8,r9\n"
"    bt      .Lsh2zi_memo_d8_s1\n"
"    shll16  r11\n"
"    or      r11,r0\n"
"    mov.l   r0,@r7\n"
"    add     #4,r7\n"
".Lsh2zi_memo_d8_s1:\n"
"    add     #1,r9\n"
"    dt      r6\n"
"    bf      .Lsh2zi_memo_d8_eq\n"
"    bra     .Lsh2zi_memo_d8_done\n"
"    nop\n"
".Lsh2zi_memo_d8_n0:\n"
"    add     #4,r4\n"
"    add     #4,r5\n"
".Lsh2zi_memo_d8_n1:\n"
"    dt      r6\n"
"    bf      .Lsh2zi_memo_d8_df\n"
".Lsh2zi_memo_d8_end:\n"
"    mov     r10,r0\n"
"    sub     r11,r0\n"
"    cmp/hs  r8,r9\n"
"    bt      .Lsh2zi_memo_d8_s2\n"
"    shll16  r11\n"
"    or      r11,r0\n"
"    mov.l   r0,@r7\n"
"    add     #4,r7\n"
".Lsh2zi_memo_d8_s2:\n"
"    add     #1,r9\n"
".Lsh2zi_memo_d8_done:\n"
"    mov     r9,r0\n"
"    mov.l   @r15+,r11\n"
"    mov.l   @r15+,r10\n"
"    mov.l   @r15+,r9\n"
"    rts\n"
"    mov.l   @r15+,r8\n"
".Lsh2zi_memo_d8_portable:\n"
"    mov.l   .Lsh2zi_memo_d8_c,r0\n"
"    jmp     @r0\n"
"    nop\n"
"    .align 2\n"
".Lsh2zi_memo_d8_c:\n"
"    .long   _sh2zi_memo_diff8_runs_c\n"
"    .size _sh2z_memo_diff8_runs,.-_sh2z_memo_diff8_runs\n"
);
/* ---- end of generated diff kernel ---- */

/* Keyed write-through. One loop per position of the always-written words:
 * variant G (0..4) tests words 0 .. G-1 against their gates and writes words
 * G .. 3 unconditionally, which covers always = 0, 0x8, 0xC, 0xE and 0xF;
 * other always masks take the portable path. Per record: d = memo ^ key; a
 * gated word costs TST and a not-taken BF when unchanged, its write is out of
 * line. A write is MOV.W of the record word into r3, then MOV.W r3 to
 * @(r0,r4) with r0 the word's offset (kept on the stack), so one base
 * pointer advances by the stride. When d != 0 the memo is stored before the
 * record's destination writes, so the CPU work after the last (slow) write
 * of a record overlaps that write instead of waiting behind a memo store.
 * r4 dst record, r5 write count, r6 stride, r8-r11 gates, r12 memo, r13
 * records, r14 records left; r8-r14 are saved and restored. */
/* ---- generated: keyed write kernel ---- */
__asm__(
"    .text\n"
"    .align 2\n"
"    .global _sh2z_memo_put4\n"
"    .type _sh2z_memo_put4,@function\n"
"_sh2z_memo_put4:\n"
"    mov.l   @r15,r1\n"
"    mov.l   @(28,r1),r0\n"
"    cmp/eq  #0,r0\n"
"    bt/s    .Lsh2zi_memo_p4_setup\n"
"    mov     #4,r3\n"
"    cmp/eq  #8,r0\n"
"    bt/s    .Lsh2zi_memo_p4_setup\n"
"    mov     #3,r3\n"
"    cmp/eq  #12,r0\n"
"    bt/s    .Lsh2zi_memo_p4_setup\n"
"    mov     #2,r3\n"
"    cmp/eq  #14,r0\n"
"    bt/s    .Lsh2zi_memo_p4_setup\n"
"    mov     #1,r3\n"
"    cmp/eq  #15,r0\n"
"    bt/s    .Lsh2zi_memo_p4_setup\n"
"    mov     #0,r3\n"
"    mov.l   .Lsh2zi_memo_p4_c,r0\n"
"    jmp     @r0\n"
"    nop\n"
"    .align 2\n"
".Lsh2zi_memo_p4_c:\n"
"    .long   _sh2zi_memo_put4_c\n"
".Lsh2zi_memo_p4_setup:\n"
"    tst     r7,r7\n"
"    bt/s    .Lsh2zi_memo_p4_zero\n"
"    mov     #0,r0\n"
"    mov.l   r8,@-r15\n"
"    mov.l   r9,@-r15\n"
"    mov.l   r10,@-r15\n"
"    mov.l   r11,@-r15\n"
"    mov.l   r12,@-r15\n"
"    mov.l   r13,@-r15\n"
"    mov.l   r14,@-r15\n"
"    add     #-16,r15\n"
"    mov     r5,r12\n"
"    mov     r6,r13\n"
"    mov     r7,r14\n"
"    mov.l   @r1,r8\n"
"    mov.l   @(4,r1),r9\n"
"    mov.l   @(8,r1),r10\n"
"    mov.l   @(12,r1),r11\n"
"    mov.l   @(16,r1),r6\n"
"    mov.w   @(20,r1),r0\n"
"    extu.w  r0,r0\n"
"    mov.l   r0,@r15\n"
"    mov.w   @(22,r1),r0\n"
"    extu.w  r0,r0\n"
"    mov.l   r0,@(4,r15)\n"
"    mov.w   @(24,r1),r0\n"
"    extu.w  r0,r0\n"
"    mov.l   r0,@(8,r15)\n"
"    mov.w   @(26,r1),r0\n"
"    extu.w  r0,r0\n"
"    mov.l   r0,@(12,r15)\n"
"    mov     #0,r5\n"
"    mova    .Lsh2zi_memo_p4_tab,r0\n"
"    shll2   r3\n"
"    mov.l   @(r0,r3),r3\n"
"    jmp     @r3\n"
"    nop\n"
"    .align 2\n"
".Lsh2zi_memo_p4_tab:\n"
"    .long   .Lsh2zi_memo_p4_0_loop\n"
"    .long   .Lsh2zi_memo_p4_1_loop\n"
"    .long   .Lsh2zi_memo_p4_2_loop\n"
"    .long   .Lsh2zi_memo_p4_3_loop\n"
"    .long   .Lsh2zi_memo_p4_4_loop\n"
".Lsh2zi_memo_p4_zero:\n"
"    rts\n"
"    nop\n"
".Lsh2zi_memo_p4_done:\n"
"    mov     r5,r0\n"
"    add     #16,r15\n"
"    mov.l   @r15+,r14\n"
"    mov.l   @r15+,r13\n"
"    mov.l   @r15+,r12\n"
"    mov.l   @r15+,r11\n"
"    mov.l   @r15+,r10\n"
"    mov.l   @r15+,r9\n"
"    rts\n"
"    mov.l   @r15+,r8\n"
".Lsh2zi_memo_p4_0_loop:\n"
"    mov.l   @r13,r2\n"
"    mov.l   @r12,r1\n"
"    xor     r2,r1\n"
"    tst     r1,r1\n"
"    bf      .Lsh2zi_memo_p4_0_m\n"
".Lsh2zi_memo_p4_0_mn:\n"
"    mov.w   @(4,r13),r0\n"
"    mov     r0,r3\n"
"    mov.l   @r15,r0\n"
"    mov.w   r3,@(r0,r4)\n"
"    add     #1,r5\n"
"    mov.w   @(6,r13),r0\n"
"    mov     r0,r3\n"
"    mov.l   @(4,r15),r0\n"
"    mov.w   r3,@(r0,r4)\n"
"    add     #1,r5\n"
"    mov.w   @(8,r13),r0\n"
"    mov     r0,r3\n"
"    mov.l   @(8,r15),r0\n"
"    mov.w   r3,@(r0,r4)\n"
"    add     #1,r5\n"
"    mov.w   @(10,r13),r0\n"
"    mov     r0,r3\n"
"    mov.l   @(12,r15),r0\n"
"    mov.w   r3,@(r0,r4)\n"
"    add     #1,r5\n"
"    add     #4,r12\n"
"    add     #12,r13\n"
"    dt      r14\n"
"    bf/s    .Lsh2zi_memo_p4_0_loop\n"
"    add     r6,r4\n"
"    bra     .Lsh2zi_memo_p4_done\n"
"    nop\n"
".Lsh2zi_memo_p4_0_m:\n"
"    bra     .Lsh2zi_memo_p4_0_mn\n"
"    mov.l   r2,@r12\n"
".Lsh2zi_memo_p4_1_loop:\n"
"    mov.l   @r13,r2\n"
"    mov.l   @r12,r1\n"
"    xor     r2,r1\n"
"    tst     r1,r1\n"
"    bf      .Lsh2zi_memo_p4_1_m\n"
".Lsh2zi_memo_p4_1_mn:\n"
"    tst     r8,r1\n"
"    bf      .Lsh2zi_memo_p4_1_w0\n"
".Lsh2zi_memo_p4_1_n0:\n"
"    mov.w   @(6,r13),r0\n"
"    mov     r0,r3\n"
"    mov.l   @(4,r15),r0\n"
"    mov.w   r3,@(r0,r4)\n"
"    add     #1,r5\n"
"    mov.w   @(8,r13),r0\n"
"    mov     r0,r3\n"
"    mov.l   @(8,r15),r0\n"
"    mov.w   r3,@(r0,r4)\n"
"    add     #1,r5\n"
"    mov.w   @(10,r13),r0\n"
"    mov     r0,r3\n"
"    mov.l   @(12,r15),r0\n"
"    mov.w   r3,@(r0,r4)\n"
"    add     #1,r5\n"
"    add     #4,r12\n"
"    add     #12,r13\n"
"    dt      r14\n"
"    bf/s    .Lsh2zi_memo_p4_1_loop\n"
"    add     r6,r4\n"
"    bra     .Lsh2zi_memo_p4_done\n"
"    nop\n"
".Lsh2zi_memo_p4_1_w0:\n"
"    mov.w   @(4,r13),r0\n"
"    mov     r0,r3\n"
"    mov.l   @r15,r0\n"
"    mov.w   r3,@(r0,r4)\n"
"    bra     .Lsh2zi_memo_p4_1_n0\n"
"    add     #1,r5\n"
".Lsh2zi_memo_p4_1_m:\n"
"    bra     .Lsh2zi_memo_p4_1_mn\n"
"    mov.l   r2,@r12\n"
".Lsh2zi_memo_p4_2_loop:\n"
"    mov.l   @r13,r2\n"
"    mov.l   @r12,r1\n"
"    xor     r2,r1\n"
"    tst     r1,r1\n"
"    bf      .Lsh2zi_memo_p4_2_m\n"
".Lsh2zi_memo_p4_2_mn:\n"
"    tst     r8,r1\n"
"    bf      .Lsh2zi_memo_p4_2_w0\n"
".Lsh2zi_memo_p4_2_n0:\n"
"    tst     r9,r1\n"
"    bf      .Lsh2zi_memo_p4_2_w1\n"
".Lsh2zi_memo_p4_2_n1:\n"
"    mov.w   @(8,r13),r0\n"
"    mov     r0,r3\n"
"    mov.l   @(8,r15),r0\n"
"    mov.w   r3,@(r0,r4)\n"
"    add     #1,r5\n"
"    mov.w   @(10,r13),r0\n"
"    mov     r0,r3\n"
"    mov.l   @(12,r15),r0\n"
"    mov.w   r3,@(r0,r4)\n"
"    add     #1,r5\n"
"    add     #4,r12\n"
"    add     #12,r13\n"
"    dt      r14\n"
"    bf/s    .Lsh2zi_memo_p4_2_loop\n"
"    add     r6,r4\n"
"    bra     .Lsh2zi_memo_p4_done\n"
"    nop\n"
".Lsh2zi_memo_p4_2_w0:\n"
"    mov.w   @(4,r13),r0\n"
"    mov     r0,r3\n"
"    mov.l   @r15,r0\n"
"    mov.w   r3,@(r0,r4)\n"
"    bra     .Lsh2zi_memo_p4_2_n0\n"
"    add     #1,r5\n"
".Lsh2zi_memo_p4_2_w1:\n"
"    mov.w   @(6,r13),r0\n"
"    mov     r0,r3\n"
"    mov.l   @(4,r15),r0\n"
"    mov.w   r3,@(r0,r4)\n"
"    bra     .Lsh2zi_memo_p4_2_n1\n"
"    add     #1,r5\n"
".Lsh2zi_memo_p4_2_m:\n"
"    bra     .Lsh2zi_memo_p4_2_mn\n"
"    mov.l   r2,@r12\n"
".Lsh2zi_memo_p4_3_loop:\n"
"    mov.l   @r13,r2\n"
"    mov.l   @r12,r1\n"
"    xor     r2,r1\n"
"    tst     r1,r1\n"
"    bf      .Lsh2zi_memo_p4_3_m\n"
".Lsh2zi_memo_p4_3_mn:\n"
"    tst     r8,r1\n"
"    bf      .Lsh2zi_memo_p4_3_w0\n"
".Lsh2zi_memo_p4_3_n0:\n"
"    tst     r9,r1\n"
"    bf      .Lsh2zi_memo_p4_3_w1\n"
".Lsh2zi_memo_p4_3_n1:\n"
"    tst     r10,r1\n"
"    bf      .Lsh2zi_memo_p4_3_w2\n"
".Lsh2zi_memo_p4_3_n2:\n"
"    mov.w   @(10,r13),r0\n"
"    mov     r0,r3\n"
"    mov.l   @(12,r15),r0\n"
"    mov.w   r3,@(r0,r4)\n"
"    add     #1,r5\n"
"    add     #4,r12\n"
"    add     #12,r13\n"
"    dt      r14\n"
"    bf/s    .Lsh2zi_memo_p4_3_loop\n"
"    add     r6,r4\n"
"    bra     .Lsh2zi_memo_p4_done\n"
"    nop\n"
".Lsh2zi_memo_p4_3_w0:\n"
"    mov.w   @(4,r13),r0\n"
"    mov     r0,r3\n"
"    mov.l   @r15,r0\n"
"    mov.w   r3,@(r0,r4)\n"
"    bra     .Lsh2zi_memo_p4_3_n0\n"
"    add     #1,r5\n"
".Lsh2zi_memo_p4_3_w1:\n"
"    mov.w   @(6,r13),r0\n"
"    mov     r0,r3\n"
"    mov.l   @(4,r15),r0\n"
"    mov.w   r3,@(r0,r4)\n"
"    bra     .Lsh2zi_memo_p4_3_n1\n"
"    add     #1,r5\n"
".Lsh2zi_memo_p4_3_w2:\n"
"    mov.w   @(8,r13),r0\n"
"    mov     r0,r3\n"
"    mov.l   @(8,r15),r0\n"
"    mov.w   r3,@(r0,r4)\n"
"    bra     .Lsh2zi_memo_p4_3_n2\n"
"    add     #1,r5\n"
".Lsh2zi_memo_p4_3_m:\n"
"    bra     .Lsh2zi_memo_p4_3_mn\n"
"    mov.l   r2,@r12\n"
".Lsh2zi_memo_p4_4_loop:\n"
"    mov.l   @r13,r2\n"
"    mov.l   @r12,r1\n"
"    xor     r2,r1\n"
"    tst     r1,r1\n"
"    bf      .Lsh2zi_memo_p4_4_m\n"
".Lsh2zi_memo_p4_4_mn:\n"
"    tst     r8,r1\n"
"    bf      .Lsh2zi_memo_p4_4_w0\n"
".Lsh2zi_memo_p4_4_n0:\n"
"    tst     r9,r1\n"
"    bf      .Lsh2zi_memo_p4_4_w1\n"
".Lsh2zi_memo_p4_4_n1:\n"
"    tst     r10,r1\n"
"    bf      .Lsh2zi_memo_p4_4_w2\n"
".Lsh2zi_memo_p4_4_n2:\n"
"    tst     r11,r1\n"
"    bf      .Lsh2zi_memo_p4_4_w3\n"
".Lsh2zi_memo_p4_4_n3:\n"
"    add     #4,r12\n"
"    add     #12,r13\n"
"    dt      r14\n"
"    bf/s    .Lsh2zi_memo_p4_4_loop\n"
"    add     r6,r4\n"
"    bra     .Lsh2zi_memo_p4_done\n"
"    nop\n"
".Lsh2zi_memo_p4_4_w0:\n"
"    mov.w   @(4,r13),r0\n"
"    mov     r0,r3\n"
"    mov.l   @r15,r0\n"
"    mov.w   r3,@(r0,r4)\n"
"    bra     .Lsh2zi_memo_p4_4_n0\n"
"    add     #1,r5\n"
".Lsh2zi_memo_p4_4_w1:\n"
"    mov.w   @(6,r13),r0\n"
"    mov     r0,r3\n"
"    mov.l   @(4,r15),r0\n"
"    mov.w   r3,@(r0,r4)\n"
"    bra     .Lsh2zi_memo_p4_4_n1\n"
"    add     #1,r5\n"
".Lsh2zi_memo_p4_4_w2:\n"
"    mov.w   @(8,r13),r0\n"
"    mov     r0,r3\n"
"    mov.l   @(8,r15),r0\n"
"    mov.w   r3,@(r0,r4)\n"
"    bra     .Lsh2zi_memo_p4_4_n2\n"
"    add     #1,r5\n"
".Lsh2zi_memo_p4_4_w3:\n"
"    mov.w   @(10,r13),r0\n"
"    mov     r0,r3\n"
"    mov.l   @(12,r15),r0\n"
"    mov.w   r3,@(r0,r4)\n"
"    bra     .Lsh2zi_memo_p4_4_n3\n"
"    add     #1,r5\n"
".Lsh2zi_memo_p4_4_m:\n"
"    bra     .Lsh2zi_memo_p4_4_mn\n"
"    mov.l   r2,@r12\n"
"    .size _sh2z_memo_put4,.-_sh2z_memo_put4\n"
);
/* ---- end of generated keyed write kernel ---- */
#else
uint32_t sh2z_memo_put4(volatile uint16_t *dst, uint32_t *memo,
                        const sh2z_memo_rec4_t *rec, uint32_t n,
                        const sh2z_memo_gate4_t *gate) {
    return sh2zi_memo_put4_c(dst, memo, rec, n, gate);
}
bool sh2z_memo_sync16w(uint16_t *shadow, const uint16_t *src) {
    return sh2zi_memo_sync16w_c(shadow, src);
}
bool sh2z_memo_sync16l(uint32_t *shadow, const uint32_t *src) {
    return memo_sync16l_c(shadow, src);
}
uint32_t sh2z_memo_sync16w_batch(uint16_t *shadow, const uint16_t *src,
                                 uint32_t groups, uint32_t *changed) {
    return sh2zi_memo_sync16w_batch_c(shadow, src, groups, changed);
}
uint32_t sh2z_memo_sync16l_batch(uint32_t *shadow, const uint32_t *src,
                                 uint32_t groups, uint32_t *changed) {
    return memo_sync16l_batch_c(shadow, src, groups, changed);
}
uint32_t sh2z_memo_copy8_until(uint16_t *dst, const uint16_t *src,
                               uint32_t max_records,
                               uint32_t w1_limit, uint32_t w3_limit) {
    return sh2zi_memo_copy8_until_c(dst, src, max_records, w1_limit, w3_limit);
}
uint32_t sh2z_memo_diff8_runs(const uint16_t *a, const uint16_t *b, uint32_t n,
                              sh2z_memo_run_t *runs, uint32_t max_runs) {
    return sh2zi_memo_diff8_runs_c(a, b, n, runs, max_runs);
}
#endif

