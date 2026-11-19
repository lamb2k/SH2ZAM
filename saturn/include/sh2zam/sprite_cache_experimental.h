/* Experimental sprite slot cache and 4-bit cell decoder for streaming sprite
 * art into a fixed VDP1 VRAM pool. Not selected by any default dispatch.
 *
 * Contract tag: sprite-cache-1. Struct layouts, the replacement rule, the
 * cell stream format and the run table below are frozen at that tag. A
 * faster path may replace an implementation only if it produces identical
 * results; new behaviour gets a new function.
 *
 * A. Slot pool. The caller carves a fixed set of slots out of VRAM (any
 *    sizes, 8-byte aligned addresses). Each slot holds one cell under a
 *    caller key. A frame stamp records when a slot was last referenced; a
 *    slot is "live" while it may still be read by VDP1: referenced in the
 *    frame being built or in the `live - 1` frames before it (live = 1 for
 *    a single command bank, 2 with vdp1-banks-1 double banks). Live slots
 *    are never replaced.
 * B. Lookup and replacement. sh2z_scache_get finds the key, or claims a
 *    slot for it: the smallest slot that fits, free slots first, else the
 *    least recently used slot that is neither live nor still filling, ties
 *    to the lowest index. A claimed slot is FILLING until the caller (for
 *    example the other CPU, after decoding into it) calls
 *    sh2z_scache_ready. A request for the next frame (a prefetch) claims in
 *    the same way and protects the slot from the current frame on.
 * C. Aliases. A cell may be drawn as the horizontal flip of another cell.
 *    sh2z_scache_alias maps a cell to the cell to fetch and the flip bit to
 *    set in the VDP1 command, from a caller table.
 * D. Cell stream (cell4z). A 4-bit cell of w x h pixels (w a multiple of 8,
 *    8..504; h 1..255; pixel 0 transparent) is coded row by row, rows
 *    left to right, as vlc-2 symbols (vlc2_experimental.h): 0..15 is one
 *    pixel, 16..29 a run of transparent pixels of length
 *    sh2z_cell4z_runs[symbol - 16]. The table for each symbol is chosen by
 *    the context, the previous pixel of the same row (0 at a row start and
 *    after a run): one table per context, 16 tables, window
 *    SH2Z_CELL4Z_W. A run never crosses the end of a row. Each cell's
 *    stream starts on a 2-byte boundary. The decoder writes the cell as
 *    16-bit words, four pixels per word, first pixel in the high nibble,
 *    which is the VDP1 4-bit format, straight into the slot.
 *
 * No function uses floating point, the hardware divider, global state, libc
 * or allocation. Synchronisation between CPUs, cache coherency (decode to a
 * cache-through address or purge) and VRAM access timing belong to the
 * caller.
 *
 * Copyright (c) 2026 SH2ZAM Contributors. MIT License.
 * Inspired by SH4ZAM by Falco Girgis and SH4ZAM Contributors.
 */
#ifndef SH2ZAM_SPRITE_CACHE_EXPERIMENTAL_H
#define SH2ZAM_SPRITE_CACHE_EXPERIMENTAL_H

#include "sh2zam.h"
#include "sh2zam/vlc2_experimental.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Contract tag sprite-cache-1 ----------------------------------------- */

/* ---- A. Slot pool ---- */
#define SH2Z_SCACHE_NO_KEY    UINT32_C(0xFFFFFFFF)
#define SH2Z_SCACHE_FREE      0u
#define SH2Z_SCACHE_FILLING   1u
#define SH2Z_SCACHE_READY     2u
#define SH2Z_SCACHE_LIVE_MAX  4u

typedef struct {
    uint32_t key;       /* caller key, SH2Z_SCACHE_NO_KEY when free */
    uint32_t last_use;  /* frame of the last reference */
    uint32_t addr;      /* fixed address of the slot (caller units, 8-aligned in bytes) */
    uint32_t bytes;     /* capacity */
    uint32_t state;     /* SH2Z_SCACHE_FREE, _FILLING or _READY */
} sh2z_scache_slot_t;

typedef struct {
    sh2z_scache_slot_t *slots;
    uint32_t count;
    uint32_t frame;     /* the frame being built; starts at live, so stamp 0 is never live */
    uint32_t live;      /* 1..SH2Z_SCACHE_LIVE_MAX */
    uint32_t hits, misses, evictions, refused;
} sh2z_scache_t;

/* Lays out slots from base: counts[c] slots of sizes[c] bytes for each class
 * c < classes, each at the next 8-byte boundary, into slots[0..max). Every
 * slot starts FREE. Returns the slot count, or 0 (nothing written) if a size
 * is 0, the layout passes base + limit, or there are more than max slots.
 * *end, if not null, receives the address after the last slot. */
uint32_t sh2z_scache_layout(sh2z_scache_slot_t *slots, uint32_t max, uint32_t base, uint32_t limit,
                            const uint32_t *sizes, const uint32_t *counts, uint32_t classes,
                            uint32_t *end);

/* live: frames a reference stays protected (1..SH2Z_SCACHE_LIVE_MAX); false
 * and nothing changed if out of range or count is 0. */
bool sh2z_scache_init(sh2z_scache_t *c, sh2z_scache_slot_t *slots, uint32_t count, uint32_t live);

/* Start building the next frame. */
static inline void sh2z_scache_frame(sh2z_scache_t *c) { c->frame += 1u; }

/* ---- B. Lookup and replacement ---- */
#define SH2Z_SCACHE_HIT      0  /* slot is READY: draw from it */
#define SH2Z_SCACHE_PENDING  1  /* slot is FILLING: not drawable yet */
#define SH2Z_SCACHE_MISS     2  /* slot claimed for key, now FILLING: fill it, then ready() */
#define SH2Z_SCACHE_REFUSED  3  /* no slot of at least `bytes` is free or replaceable */

/* Reference key for the frame being built (prefetch false) or for the next
 * frame (prefetch true; the reference counts from the next frame, and the
 * slot is protected from now on). bytes is the cell's size; a slot is
 * reused for a key only if it is at least that large. *slot receives the
 * slot index for every result except REFUSED. key must not be
 * SH2Z_SCACHE_NO_KEY. */
int sh2z_scache_get(sh2z_scache_t *c, uint32_t key, uint32_t bytes, bool prefetch, uint32_t *slot);

/* Mark a FILLING slot READY; false if it is not FILLING. */
bool sh2z_scache_ready(sh2z_scache_t *c, uint32_t slot);
/* Drop a FILLING slot's claim (a failed fill): the slot becomes FREE. */
bool sh2z_scache_abandon(sh2z_scache_t *c, uint32_t slot);
/* Forget every key (for example at a race change). Stamps stay. */
void sh2z_scache_clear(sh2z_scache_t *c);
/* true while slot may still be read by VDP1. */
bool sh2z_scache_live(const sh2z_scache_t *c, uint32_t slot);

/* ---- C. Aliases ----
 * map[cell] is the cell to fetch, with bit 15 set when it is drawn
 * horizontally flipped; map null means no aliases. Returns the cell to fetch
 * and writes the flip bit (0 or 1). */
static inline uint32_t sh2z_scache_alias(const uint16_t *map, uint32_t cell, uint32_t *flip) {
    if (!map) { *flip = 0u; return cell; }
    *flip = (uint32_t)(map[cell] >> 15);
    return (uint32_t)(map[cell] & 0x7FFFu);
}

/* ---- D. Cell stream (cell4z) ---- */
#define SH2Z_CELL4Z_W        6u   /* vlc-2 window of every context table */
#define SH2Z_CELL4Z_SYMBOLS  30u  /* 16 pixels + 14 run lengths */
#define SH2Z_CELL4Z_CONTEXTS 16u
#define SH2Z_CELL4Z_WMAX     504u
#define SH2Z_CELL4Z_HMAX     255u
extern const uint8_t sh2z_cell4z_runs[SH2Z_CELL4Z_SYMBOLS - 16u]; /* 2,3,4,5,6,8,10,12,16,20,24,32,48,64 */

typedef struct {
    uint16_t offset[SH2Z_CELL4Z_CONTEXTS]; /* table of context k at arena + offset[k] */
    uint16_t size[SH2Z_CELL4Z_CONTEXTS];
    uint32_t lmax;                         /* longest code of any table */
} sh2z_cell4z_tables_t;

/* Build the 16 context tables from code lengths into arena[0..capacity).
 * len holds SH2Z_CELL4Z_CONTEXTS rows of SH2Z_CELL4Z_SYMBOLS lengths, row k
 * (context k) at len + k * SH2Z_CELL4Z_SYMBOLS; each row is a complete prefix
 * code with lengths <= SH2Z_VLC2_LMAX. Returns the arena bytes used, or 0
 * (with *t unchanged) if a table is refused or the arena is too small. */
size_t sh2z_cell4z_build(uint8_t *arena, size_t capacity, const uint8_t *len, sh2z_cell4z_tables_t *t);

/* Readable bytes a w-pixel-wide row may need past any point of a stream:
 * the slack a pack must carry after its last cell stream. */
#define SH2Z_CELL4Z_SLACK(w, lmax) ((((uint32_t)(w) * (uint32_t)(lmax) + 31u) >> 4) * 2u)

/* Decode one w x h cell whose stream starts at stream[0] (2-byte aligned),
 * writing w / 4 words per row to dst. readable is the number of bytes that
 * may be read from stream[0] on: the cell's own stream and whatever follows
 * it in the same buffer (for example the rest of a pack). Before each row
 * the decoder checks that the row cannot read past readable, so a malformed
 * stream never reads outside it; a buffer's last stream needs
 * SH2Z_CELL4Z_SLACK(w, lmax) readable bytes after it. *bits_used, if not
 * null, receives the bits the cell consumed (compare it with the cell's own
 * stream length to detect corruption). Returns true when the cell decoded
 * completely. Returns false on a bad size, a run past a row end or a row
 * that might read past readable; output written before the failure stays. */
bool sh2z_cell4z_decode(volatile uint16_t *dst, uint32_t w, uint32_t h,
                        const uint8_t *stream, size_t readable,
                        const uint8_t *arena, const sh2z_cell4z_tables_t *t, size_t *bits_used);

#ifdef __cplusplus
}
#endif

#endif
