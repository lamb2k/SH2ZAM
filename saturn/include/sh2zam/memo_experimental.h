/* Experimental block memo kernels for SH-2: compare-and-copy of fixed-size
 * blocks with a changed flag, keyed write-through of 16-bit words to a slow
 * destination, a record copy that stops at an end marker, and a record-run
 * diff. Hand-written SH-2 kernels for the block and copy forms, portable C
 * everywhere. Not selected by any default dispatch.
 * MIT; SH2ZAM Contributors.
 *
 * Contract tag: memo-1. Every function below is exact: its outputs are a
 * pure function of its inputs, as stated, with no approximation. A faster
 * path may replace an implementation only if it gives the same results and
 * performs the same destination writes; new behaviour gets a new function.
 *
 * The data and its meaning belong to the caller: nothing here knows a file
 * format, a palette layout or a command layout. No function uses floating
 * point, the hardware divider, global state, libc or allocation; all are
 * reentrant (the caller owns the memory and the synchronisation).
 *
 * Words. A "word" is a uint16_t and a "longword" a uint32_t, each in the
 * CPU's own byte order. Record word k of an 8-byte record is element k of a
 * uint16_t[4] view of it. Nothing depends on byte order: the SH-2 kernels
 * read two words as one longword only where the result is the same. */
#ifndef SH2ZAM_MEMO_EXPERIMENTAL_H
#define SH2ZAM_MEMO_EXPERIMENTAL_H

#include "sh2zam.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Contract tag memo-1 -------------------------------------------------- */

/* ---- A. Block compare-and-copy ----
 * A block is 16 words (32 bytes, sh2z_memo_sync16w*) or 16 longwords
 * (64 bytes, sh2z_memo_sync16l*). A block has changed when any of its bytes
 * differs between shadow and src. For each changed block the kernel copies src
 * into shadow; an unchanged block is only read. After the call every shadow
 * block equals its src block.
 *
 * sh2z_memo_sync16w / sh2z_memo_sync16l: one block; returns true iff it had
 * changed.
 *
 * The batch forms process `groups` consecutive blocks and return the number
 * that had changed. changed[] receives a bit mask: bit (g % 32) of
 * changed[g / 32] is set iff block g had changed. Exactly the words
 * changed[0 .. (groups + 31) / 32) are written, and the bits of the last one
 * at or after `groups` are zero. groups == 0 writes nothing and returns 0.
 *
 * Alignment: sh2z_memo_sync16w* need 2-byte aligned blocks and take the SH-2
 * kernel when both shadow and src are 4-byte aligned (otherwise the portable
 * path, same results). sh2z_memo_sync16l* need 4-byte aligned blocks.
 * Aliasing: shadow and src must be the same pointer or must not overlap;
 * changed[] must not overlap either.
 * Every word of each block is compared, including words the caller does not
 * use: keep unused words equal (for example zero) in both. */
bool sh2z_memo_sync16w(uint16_t *shadow, const uint16_t *src);
bool sh2z_memo_sync16l(uint32_t *shadow, const uint32_t *src);
uint32_t sh2z_memo_sync16w_batch(uint16_t *shadow, const uint16_t *src,
                                 uint32_t groups, uint32_t *changed);
uint32_t sh2z_memo_sync16l_batch(uint32_t *shadow, const uint32_t *src,
                                 uint32_t groups, uint32_t *changed);

/* ---- B. Keyed write-through to a 16-bit destination ----
 * Each record carries a 32-bit key and 4 words. The memo holds, per record
 * slot, the key whose words the destination holds now. For record i with
 * old = memo[i], new = rec[i].key and d = old ^ new, word k is written to
 *     dst + i * gate->stride + gate->offset[k]     (byte addresses)
 * iff (d & gate->gate[k]) != 0 or bit k of gate->always is set. Words are
 * written in order k = 0, 1, 2, 3, records in order i = 0 .. n - 1, each as
 * one 16-bit store through a volatile pointer; no destination byte is read or
 * written otherwise. memo[i] = new afterwards; it is written only when
 * d != 0 (the SH-2 kernel writes it before the record's destination
 * writes). Returns the number of 16-bit destination writes.
 *
 * The gate descriptor describes the key layout without knowing it. For a
 * key made of two key bytes in bits 31..16 that select words 0 and 1, and a
 * 16-bit key in bits 15..0 that selects word 2, with word 3 always written:
 *     gate = {0xFFFF0000, 0xFFFF0000, 0x0000FFFF, 0}, always = 0x8.
 * A key bit no gate covers only updates the memo.
 *
 * Requirements: dst, stride and every offset even; offsets below 65536; the
 * memo and the records 4-byte aligned; n * stride within the address space.
 * To force every word out once (for example on the first picture), call with
 * a descriptor whose always = 0xF, or fill the memo with keys that differ
 * from every new key in every gated bit.
 * Aliasing: the destination must not overlap rec, memo or the descriptor. */
typedef struct sh2z_memo_rec4 {
    uint32_t key;
    uint16_t word[4];
} sh2z_memo_rec4_t;                       /* 12 bytes */

typedef struct sh2z_memo_gate4 {
    uint32_t gate[4];     /* key bits that, when changed, rewrite word k */
    uint32_t stride;      /* destination bytes per record */
    uint16_t offset[4];   /* destination byte offset of word k in a record */
    uint32_t always;      /* bit k: write word k for every record */
} sh2z_memo_gate4_t;

uint32_t sh2z_memo_put4(volatile uint16_t *dst, uint32_t *memo,
                        const sh2z_memo_rec4_t *rec, uint32_t n,
                        const sh2z_memo_gate4_t *gate);

/* One record of sh2z_memo_put4 for callers that build their words in their
 * own loop (no staging array): the same writes, in the same order, for one
 * record with memo slot *memo and destination record dst. */
static inline uint32_t sh2z_memo_put4_one(volatile uint16_t *dst, uint32_t *memo,
                                          uint32_t key, const uint16_t word[4],
                                          const sh2z_memo_gate4_t *gate) {
    /* Everything is read before the first store: a volatile store could
     * alias the descriptor, the memo or the words, so a load after it would
     * be repeated for every record. */
    const uint32_t d = *memo ^ key, a = gate->always;
    const uint32_t g0 = gate->gate[0], g1 = gate->gate[1], g2 = gate->gate[2], g3 = gate->gate[3];
    const uint32_t o0 = gate->offset[0], o1 = gate->offset[1], o2 = gate->offset[2], o3 = gate->offset[3];
    const uint16_t w0 = word[0], w1 = word[1], w2 = word[2], w3 = word[3];
    volatile uint8_t *const b = (volatile uint8_t *)dst;
    uint32_t writes = 0;
    if ((d & g0) != 0u || (a & 1u) != 0u) { *(volatile uint16_t *)(volatile void *)(b + o0) = w0; writes++; }
    if ((d & g1) != 0u || (a & 2u) != 0u) { *(volatile uint16_t *)(volatile void *)(b + o1) = w1; writes++; }
    if ((d & g2) != 0u || (a & 4u) != 0u) { *(volatile uint16_t *)(volatile void *)(b + o2) = w2; writes++; }
    if ((d & g3) != 0u || (a & 8u) != 0u) { *(volatile uint16_t *)(volatile void *)(b + o3) = w3; writes++; }
    if (d != 0u)
        *memo = key;
    return writes;
}

/* ---- C. Copy 8-byte records until an end marker ----
 * Records are 4 words. Record i is an end marker iff
 *     src[4i + 1] >= w1_limit  or  src[4i + 3] >= w3_limit
 * (word 1 or word 3, compared as unsigned; a limit of 0x10000 or more turns
 * its test off, 0 makes every record a marker). Let m be the index of the
 * first marker among records 0 .. max_records - 1, or max_records if there is
 * none. Records 0 .. min(m, max_records - 1) are copied to dst, so the marker
 * itself is copied and the copy is terminated like its source. Returns m.
 * max_records == 0 copies nothing and returns 0. Nothing at or after record
 * m + 1 of src is read.
 * Alignment: 2-byte; the SH-2 kernel runs when src and dst are both 4-byte
 * aligned. src and dst must not overlap. */
uint32_t sh2z_memo_copy8_until(uint16_t *dst, const uint16_t *src,
                               uint32_t max_records,
                               uint32_t w1_limit, uint32_t w3_limit);

/* ---- D. Record-run diff ----
 * Records are 8 bytes (4 words). Record i differs iff any of its bytes
 * differs between a and b. A run is a maximal range of consecutive differing
 * records; runs are reported in increasing order as (start, count). The
 * first min(total, max_runs) runs are written to runs[]; the return value is
 * the total number of runs. n must be at most 65535. Records 2-byte aligned
 * (4-byte aligned is faster on SH-2). Neither table is written. */
typedef struct sh2z_memo_run {
    uint16_t start;
    uint16_t count;
} sh2z_memo_run_t;

uint32_t sh2z_memo_diff8_runs(const uint16_t *a, const uint16_t *b, uint32_t n,
                              sh2z_memo_run_t *runs, uint32_t max_runs);

#ifdef __cplusplus
}
#endif

#endif
