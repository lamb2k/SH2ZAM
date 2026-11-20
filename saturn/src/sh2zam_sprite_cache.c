/* Experimental sprite slot cache and 4-bit cell decoder (sprite-cache-1).
 *
 * Copyright (c) 2026 SH2ZAM Contributors. MIT License.
 * Inspired by SH4ZAM by Falco Girgis and SH4ZAM Contributors.
 */
#include "sh2zam/sprite_cache_experimental.h"

const uint8_t sh2z_cell4z_runs[SH2Z_CELL4Z_SYMBOLS - 16u] = {
    2u, 3u, 4u, 5u, 6u, 8u, 10u, 12u, 16u, 20u, 24u, 32u, 48u, 64u
};

/* ---- A. Slot pool ---- */

uint32_t sh2z_scache_layout(sh2z_scache_slot_t *slots, uint32_t max, uint32_t base, uint32_t limit,
                            const uint32_t *sizes, const uint32_t *counts, uint32_t classes,
                            uint32_t *end) {
    uint32_t n = 0u, at = (base + 7u) & ~UINT32_C(7);
    const uint32_t stop = base + limit;
    if (stop < base) return 0u;
    for (uint32_t c = 0u; c < classes; ++c) {
        if (sizes[c] == 0u) return 0u;
        for (uint32_t i = 0u; i < counts[c]; ++i) {
            if (n >= max || at > stop || sizes[c] > stop - at) return 0u;
            at += sizes[c];
            at = (at + 7u) & ~UINT32_C(7);
            ++n;
        }
    }
    at = (base + 7u) & ~UINT32_C(7);
    n = 0u;
    for (uint32_t c = 0u; c < classes; ++c)
        for (uint32_t i = 0u; i < counts[c]; ++i) {
            slots[n].key = SH2Z_SCACHE_NO_KEY;
            slots[n].last_use = 0u;
            slots[n].addr = at;
            slots[n].bytes = sizes[c];
            slots[n].state = SH2Z_SCACHE_FREE;
            at = (at + sizes[c] + 7u) & ~UINT32_C(7);
            ++n;
        }
    if (end) *end = at;
    return n;
}

bool sh2z_scache_init(sh2z_scache_t *c, sh2z_scache_slot_t *slots, uint32_t count, uint32_t live) {
    if (!slots || count == 0u || live == 0u || live > SH2Z_SCACHE_LIVE_MAX) return false;
    c->slots = slots;
    c->count = count;
    c->live = live;
    c->frame = live;
    c->hits = c->misses = c->evictions = c->refused = 0u;
    return true;
}

static inline bool scache_live_at(const sh2z_scache_t *c, const sh2z_scache_slot_t *s) {
    return s->last_use + c->live > c->frame;
}

bool sh2z_scache_live(const sh2z_scache_t *c, uint32_t slot) {
    return slot < c->count && c->slots[slot].state != SH2Z_SCACHE_FREE && scache_live_at(c, &c->slots[slot]);
}

int sh2z_scache_get(sh2z_scache_t *c, uint32_t key, uint32_t bytes, bool prefetch, uint32_t *slot) {
    const uint32_t stamp = c->frame + (prefetch ? 1u : 0u);
    uint32_t best = c->count;
    for (uint32_t i = 0u; i < c->count; ++i) {
        sh2z_scache_slot_t *s = &c->slots[i];
        if (s->state != SH2Z_SCACHE_FREE && s->key == key && s->bytes >= bytes) {
            if (s->last_use < stamp) s->last_use = stamp;
            *slot = i;
            if (s->state == SH2Z_SCACHE_READY) { ++c->hits; return SH2Z_SCACHE_HIT; }
            return SH2Z_SCACHE_PENDING;
        }
    }
    /* Claim: smallest fitting size; within it free first, then least recently used. */
    for (uint32_t i = 0u; i < c->count; ++i) {
        const sh2z_scache_slot_t *s = &c->slots[i];
        if (s->bytes < bytes || s->state == SH2Z_SCACHE_FILLING) continue;
        if (s->state != SH2Z_SCACHE_FREE && scache_live_at(c, s)) continue;
        if (best == c->count) { best = i; continue; }
        const sh2z_scache_slot_t *b = &c->slots[best];
        if (s->bytes != b->bytes) { if (s->bytes < b->bytes) best = i; continue; }
        const bool sf = s->state == SH2Z_SCACHE_FREE, bf = b->state == SH2Z_SCACHE_FREE;
        if (sf != bf) { if (sf) best = i; continue; }
        if (!sf && s->last_use < b->last_use) best = i;
    }
    if (best == c->count) { ++c->refused; return SH2Z_SCACHE_REFUSED; }
    sh2z_scache_slot_t *s = &c->slots[best];
    if (s->state != SH2Z_SCACHE_FREE) ++c->evictions;
    s->key = key;
    s->last_use = stamp;
    s->state = SH2Z_SCACHE_FILLING;
    ++c->misses;
    *slot = best;
    return SH2Z_SCACHE_MISS;
}

bool sh2z_scache_ready(sh2z_scache_t *c, uint32_t slot) {
    if (slot >= c->count || c->slots[slot].state != SH2Z_SCACHE_FILLING) return false;
    c->slots[slot].state = SH2Z_SCACHE_READY;
    return true;
}

bool sh2z_scache_abandon(sh2z_scache_t *c, uint32_t slot) {
    if (slot >= c->count || c->slots[slot].state != SH2Z_SCACHE_FILLING) return false;
    c->slots[slot].state = SH2Z_SCACHE_FREE;
    c->slots[slot].key = SH2Z_SCACHE_NO_KEY;
    return true;
}

void sh2z_scache_clear(sh2z_scache_t *c) {
    for (uint32_t i = 0u; i < c->count; ++i) {
        if (c->slots[i].state == SH2Z_SCACHE_FILLING) continue;
        c->slots[i].state = SH2Z_SCACHE_FREE;
        c->slots[i].key = SH2Z_SCACHE_NO_KEY;
    }
}

/* ---- D. Cell stream (cell4z) ---- */

size_t sh2z_cell4z_build(uint8_t *arena, size_t capacity, const uint8_t *len, sh2z_cell4z_tables_t *t) {
    size_t at = 0u;
    uint32_t lmax = 0u;
    uint16_t offset[SH2Z_CELL4Z_CONTEXTS], size[SH2Z_CELL4Z_CONTEXTS];
    for (uint32_t k = 0u; k < SH2Z_CELL4Z_CONTEXTS; ++k) {
        const uint8_t *row = len + k * SH2Z_CELL4Z_SYMBOLS;
        const size_t need = sh2z_vlc2_table_size(row, SH2Z_CELL4Z_SYMBOLS, SH2Z_CELL4Z_W);
        if (need == 0u || need > capacity - at || at > 0xFFFFu) return 0u;
        if (sh2z_vlc2_build(arena + at, capacity - at, row, SH2Z_CELL4Z_SYMBOLS, SH2Z_CELL4Z_W, 0) != need)
            return 0u;
        for (uint32_t s = 0u; s < SH2Z_CELL4Z_SYMBOLS; ++s)
            if (row[s] > lmax) lmax = row[s];
        offset[k] = (uint16_t)at;
        size[k] = (uint16_t)need;
        at += need;
    }
    for (uint32_t k = 0u; k < SH2Z_CELL4Z_CONTEXTS; ++k) { t->offset[k] = offset[k]; t->size[k] = size[k]; }
    t->lmax = lmax;
    return at;
}

bool sh2z_cell4z_decode(volatile uint16_t *dst, uint32_t w, uint32_t h,
                        const uint8_t *stream, size_t readable,
                        const uint8_t *arena, const sh2z_cell4z_tables_t *t, size_t *bits_used) {
    if (w == 0u || (w & 7u) || w > SH2Z_CELL4Z_WMAX || h == 0u || h > SH2Z_CELL4Z_HMAX) return false;
    if (t->lmax == 0u || t->lmax > SH2Z_VLC2_LMAX || readable < 2u) return false;
    const uint8_t *tab[SH2Z_CELL4Z_CONTEXTS];
    for (uint32_t k = 0u; k < SH2Z_CELL4Z_CONTEXTS; ++k) tab[k] = arena + t->offset[k];
    const size_t row_bits = (size_t)w * t->lmax;
    sh2z_vlc2_reader_t r;
    sh2z_vlc2_begin(&r, stream, 0u);
    /* acc holds up to four pending nibbles under a marker bit: it starts at 1 and holds a
     * full word once it passes 0xFFFF. Only constant shifts, since the SH-2 has no
     * variable shift instruction. */
    for (uint32_t y = 0u; y < h; ++y) {
        /* Every symbol covers at least one pixel, so a row consumes at most w * lmax bits. */
        if (sh2z_vlc2_reach(0u, sh2z_vlc2_bitpos(&r, stream) + row_bits) > readable) return false;
        uint32_t left_px = w, acc = 1u;
        const uint8_t *table = tab[0];
        do {
            const uint32_t s = sh2z_vlc2_decode(&r, table, SH2Z_CELL4Z_W);
            if (s < 16u) {
                acc = (acc << 4) | s;
                table = tab[s];
                --left_px;
                if (acc > 0xFFFFu) { *dst++ = (uint16_t)acc; acc = 1u; }
                continue;
            }
            if (s >= SH2Z_CELL4Z_SYMBOLS) return false;
            uint32_t run = sh2z_cell4z_runs[s - 16u];
            if (run > left_px) return false;
            left_px -= run;
            table = tab[0];
            while (acc != 1u && run) {
                acc <<= 4;
                --run;
                if (acc > 0xFFFFu) { *dst++ = (uint16_t)acc; acc = 1u; }
            }
            for (; run >= 4u; run -= 4u) *dst++ = 0u;
            while (run--) acc <<= 4;
        } while (left_px);
    }
    if (bits_used) *bits_used = sh2z_vlc2_bitpos(&r, stream);
    return true;
}
