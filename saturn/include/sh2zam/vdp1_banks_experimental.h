/* Experimental VDP1 double-bank bookkeeping: lets the CPU build frame N+1
 * while VDP1 still draws frame N. Not selected by any default dispatch.
 *
 * Contract tag: vdp1-banks-1. Pure bookkeeping, no hardware access: the
 * caller reads the VDP1 state, waits, transfers and uploads; these functions
 * only decide and record. Every function is exact and deterministic.
 *
 * Model. Every table the CPU rewrites per frame and VDP1 (or an end-of-draw
 * upload) reads during the draw gets two banks. The build of each frame
 * writes one bank; submission hands that bank to VDP1 and the next build
 * takes the other one. A frame may overlap the previous draw only when that
 * previous frame filled every banked table (otherwise a table could still be
 * in use from an older layout) and the caller has no reason of its own to
 * wait. The caller then waits for the previous draw only after the build.
 *
 * A. Bank state (sh2z_vdp1_banks_t): which bank the next build writes, which
 *    bank the submitted frame uses, and counters. begin() says whether this
 *    build must wait for the previous draw first (serial); it counts frames
 *    that start while VDP1 is still busy (overlap) and, of those, frames that
 *    would write the in-flight bank (conflicts, never nonzero unless the bank
 *    is held). submitted() records the frame and alternates the bank.
 * B. Guards: fixed words around a carved region, checked every frame.
 * C. Carving: lays out guard, items, guard inside one address range.
 * D. Upload queue: tables uploaded at the end of a draw (for example by a
 *    VBlank callback) are queued in build order; each completed draw pops the
 *    oldest. At most two entries, one per bank.
 * E. Per-bank staleness for tables uploaded only when they change: a change
 *    marks the entry stale in both banks; each bank rewrites its own stale
 *    entries when it is next built.
 * F. Jump-skip record: a 32-byte VDP1 command that skips to a reserved slot,
 *    so a block written elsewhere (for example by the other CPU) keeps a
 *    fixed position in the list while the commands before it vary in number.
 *
 * Synchronisation, cache coherency and interrupt safety belong to the
 * caller: the queue is modified by the build and by the end-of-draw handler,
 * which must not run concurrently with the same queue operation.
 *
 * Copyright (c) 2026 SH2ZAM Contributors. MIT License.
 * Inspired by SH4ZAM by Falco Girgis and SH4ZAM Contributors.
 */
#ifndef SH2ZAM_VDP1_BANKS_EXPERIMENTAL_H
#define SH2ZAM_VDP1_BANKS_EXPERIMENTAL_H

#include "sh2zam.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Contract tag vdp1-banks-1 -------------------------------------------- */

/* ---- A. Bank state ---- */
#define SH2Z_VDP1_BANK_NONE 2u
typedef struct {
    uint32_t build;     /* bank the next build writes: 0 or 1 */
    uint32_t inflight;  /* bank of the last submitted frame, SH2Z_VDP1_BANK_NONE before any */
    uint32_t prev_ok;   /* the last submitted frame filled every banked table */
    uint32_t hold;      /* nonzero: never alternate (negative control only) */
    uint32_t frames;    /* begin() calls */
    uint32_t serial;    /* builds told to wait for the previous draw first */
    uint32_t overlap;   /* non-serial builds that started while VDP1 was busy */
    uint32_t conflicts; /* overlap builds that would write the in-flight bank */
} sh2z_vdp1_banks_t;

/* Bank 0 first, nothing in flight, the first build serial. hold != 0 keeps
 * the bank at 0 (a control that must raise conflicts). */
void sh2z_vdp1_banks_init(sh2z_vdp1_banks_t *b, bool hold);
/* Before a build. serial_required: the caller's own reasons to wait (for
 * example a scene that rewrites tables in place). vdp1_busy: VDP1 is still
 * drawing the submitted frame now. Returns true when the build must wait for
 * the previous draw before writing anything: serial_required, or the previous
 * frame did not fill every banked table. */
bool sh2z_vdp1_banks_begin(sh2z_vdp1_banks_t *b, bool serial_required, bool vdp1_busy);
/* After the frame's command list is submitted. filled: this frame wrote every
 * banked table, so the next frame may overlap its draw. */
void sh2z_vdp1_banks_submitted(sh2z_vdp1_banks_t *b, bool filled);
/* The bank the next build writes. */
static inline uint32_t sh2z_vdp1_banks_bank(const sh2z_vdp1_banks_t *b) { return b->build; }

/* ---- B. Guards ----
 * Word i of a guard is seed ^ (i << shift); shift < 32. */
void sh2z_vdp1_guard_fill(volatile uint32_t *words, uint32_t count, uint32_t seed, uint32_t shift);
bool sh2z_vdp1_guard_check(const volatile uint32_t *words, uint32_t count, uint32_t seed, uint32_t shift);

/* ---- C. Carving ----
 * Lays out [guard_bytes][item 0]...[item count-1][guard_bytes] from base:
 * each item starts at the next multiple of its alignment (a power of two,
 * 1..256) and out[i] receives its address. *end receives the address just
 * after the last item (where the trailing guard starts). Returns false, with
 * out[] and *end unchanged, when base is not 4-byte aligned, guard_bytes is
 * not a multiple of 4, an alignment is invalid or the layout does not fit in
 * limit bytes. count 0 lays out the two guards only. */
bool sh2z_vdp1_carve(uintptr_t base, uint32_t limit, uint32_t guard_bytes,
                     const uint32_t *sizes, const uint32_t *aligns, uint32_t count,
                     uintptr_t *out, uintptr_t *end);

/* ---- D. Upload queue ---- */
typedef struct {
    uint32_t count;     /* 0, 1 or 2 queued entries */
    uint32_t bank[2];   /* oldest first */
    uint32_t tag[2];    /* caller's nonzero tag per entry (for example a tick) */
} sh2z_vdp1_queue_t;
void sh2z_vdp1_queue_init(sh2z_vdp1_queue_t *q);
/* true when bank may be filled: it is not queued and the queue is not full. */
bool sh2z_vdp1_queue_free(const sh2z_vdp1_queue_t *q, uint32_t bank);
/* Queues bank (0 or 1) with a nonzero tag; false, unchanged, if it is not free. */
bool sh2z_vdp1_queue_push(sh2z_vdp1_queue_t *q, uint32_t bank, uint32_t tag);
/* Removes the oldest entry into *bank and *tag; false, outputs unchanged, if empty. */
bool sh2z_vdp1_queue_pop(sh2z_vdp1_queue_t *q, uint32_t *bank, uint32_t *tag);
/* Tag of the oldest entry, 0 when empty. */
uint32_t sh2z_vdp1_queue_front(const sh2z_vdp1_queue_t *q);

/* ---- E. Per-bank staleness, up to 64 entries ---- */
typedef struct { uint32_t bits[2][2]; } sh2z_vdp1_stale64_t;
/* Every entry stale in both banks. */
void sh2z_vdp1_stale_init(sh2z_vdp1_stale64_t *s);
/* Entry i (< 64) changed: stale in both banks. */
void sh2z_vdp1_stale_mark(sh2z_vdp1_stale64_t *s, uint32_t i);
/* true if entry i (< 64) is stale in bank; clears it there. */
bool sh2z_vdp1_stale_take(sh2z_vdp1_stale64_t *s, uint32_t bank, uint32_t i);

/* ---- F. Jump-skip record ----
 * Writes the 16 halfwords of a VDP1 command record that skips to command
 * slot target (CMDCTRL jump mode "skip assign", CMDLINK = target * 4, every
 * other word zero). Returns false, writing nothing, if target >= 0x4000. */
bool sh2z_vdp1_jump_skip(volatile uint16_t *record, uint32_t target);

#ifdef __cplusplus
}
#endif

#endif
