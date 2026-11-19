/* VDP1 double-bank bookkeeping (vdp1_banks_experimental.h, contract tag
 * vdp1-banks-1): bank state, guards, carving, the end-of-draw upload queue,
 * per-bank staleness and the jump-skip record. Portable C, no hardware
 * access, no libc.
 *
 * Copyright (c) 2026 SH2ZAM Contributors. MIT License.
 * Inspired by SH4ZAM by Falco Girgis and SH4ZAM Contributors.
 */
#include "sh2zam/vdp1_banks_experimental.h"

void sh2z_vdp1_banks_init(sh2z_vdp1_banks_t *b, bool hold) {
    b->build = 0u; b->inflight = SH2Z_VDP1_BANK_NONE; b->prev_ok = 0u; b->hold = hold ? 1u : 0u;
    b->frames = 0u; b->serial = 0u; b->overlap = 0u; b->conflicts = 0u;
}

bool sh2z_vdp1_banks_begin(sh2z_vdp1_banks_t *b, bool serial_required, bool vdp1_busy) {
    ++b->frames;
    if (serial_required || !b->prev_ok) { ++b->serial; return true; }
    if (vdp1_busy) {
        ++b->overlap;
        if (b->build == b->inflight) ++b->conflicts;
    }
    return false;
}

void sh2z_vdp1_banks_submitted(sh2z_vdp1_banks_t *b, bool filled) {
    b->inflight = b->build;
    b->prev_ok = filled ? 1u : 0u;
    if (!b->hold) b->build ^= 1u;
}

void sh2z_vdp1_guard_fill(volatile uint32_t *words, uint32_t count, uint32_t seed, uint32_t shift) {
    for (uint32_t i = 0; i < count; ++i) words[i] = seed ^ (i << (shift & 31u));
}

bool sh2z_vdp1_guard_check(const volatile uint32_t *words, uint32_t count, uint32_t seed, uint32_t shift) {
    for (uint32_t i = 0; i < count; ++i)
        if (words[i] != (seed ^ (i << (shift & 31u)))) return false;
    return true;
}

bool sh2z_vdp1_carve(uintptr_t base, uint32_t limit, uint32_t guard_bytes,
                     const uint32_t *sizes, const uint32_t *aligns, uint32_t count,
                     uintptr_t *out, uintptr_t *end) {
    if ((base & 3u) || (guard_bytes & 3u) || !end || (count && (!sizes || !aligns || !out))) return false;
    /* Absolute addresses in 64 bits, so no step can wrap. */
    const uint64_t start = (uint64_t)base;
    uint64_t at = start + guard_bytes;
    for (uint32_t i = 0; i < count; ++i) {
        const uint32_t a = aligns[i];
        if (!a || a > 256u || (a & (a - 1u))) return false;
        at = (at + (a - 1u)) & ~(uint64_t)(a - 1u);
        at += sizes[i];
    }
    if (at + guard_bytes - start > limit) return false;
    at = start + guard_bytes;
    for (uint32_t i = 0; i < count; ++i) {
        const uint32_t a = aligns[i];
        at = (at + (a - 1u)) & ~(uint64_t)(a - 1u);
        out[i] = (uintptr_t)at;
        at += sizes[i];
    }
    *end = (uintptr_t)at;
    return true;
}

void sh2z_vdp1_queue_init(sh2z_vdp1_queue_t *q) {
    q->count = 0u; q->bank[0] = q->bank[1] = 0u; q->tag[0] = q->tag[1] = 0u;
}

bool sh2z_vdp1_queue_free(const sh2z_vdp1_queue_t *q, uint32_t bank) {
    if (bank > 1u || q->count >= 2u) return false;
    return !(q->count == 1u && q->bank[0] == bank);
}

bool sh2z_vdp1_queue_push(sh2z_vdp1_queue_t *q, uint32_t bank, uint32_t tag) {
    if (!tag || !sh2z_vdp1_queue_free(q, bank)) return false;
    q->bank[q->count] = bank; q->tag[q->count] = tag; ++q->count;
    return true;
}

bool sh2z_vdp1_queue_pop(sh2z_vdp1_queue_t *q, uint32_t *bank, uint32_t *tag) {
    if (!q->count || !bank || !tag) return false;
    *bank = q->bank[0]; *tag = q->tag[0];
    q->bank[0] = q->bank[1]; q->tag[0] = q->tag[1];
    q->bank[1] = 0u; q->tag[1] = 0u; --q->count;
    return true;
}

uint32_t sh2z_vdp1_queue_front(const sh2z_vdp1_queue_t *q) { return q->count ? q->tag[0] : 0u; }

void sh2z_vdp1_stale_init(sh2z_vdp1_stale64_t *s) {
    s->bits[0][0] = s->bits[0][1] = s->bits[1][0] = s->bits[1][1] = 0xffffffffu;
}

void sh2z_vdp1_stale_mark(sh2z_vdp1_stale64_t *s, uint32_t i) {
    if (i >= 64u) return;
    const uint32_t bit = 1u << (i & 31u);
    s->bits[0][i >> 5] |= bit; s->bits[1][i >> 5] |= bit;
}

bool sh2z_vdp1_stale_take(sh2z_vdp1_stale64_t *s, uint32_t bank, uint32_t i) {
    if (i >= 64u || bank > 1u) return false;
    const uint32_t bit = 1u << (i & 31u);
    if (!(s->bits[bank][i >> 5] & bit)) return false;
    s->bits[bank][i >> 5] &= ~bit;
    return true;
}

bool sh2z_vdp1_jump_skip(volatile uint16_t *record, uint32_t target) {
    if (!record || target >= 0x4000u) return false;
    record[0] = 0x5000u;                       /* jump mode: skip assign */
    record[1] = (uint16_t)(target << 2);       /* CMDLINK: slot * 32 bytes / 8 */
    for (uint32_t i = 2; i < 16u; ++i) record[i] = 0u;
    return true;
}
