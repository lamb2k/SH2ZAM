/* Scene sequencing (scene_experimental.h, contract tag scene-1): arenas, the
 * scene sequencer with fades, confirmed retirement and preload, requests by
 * rank. Portable C, no hardware access, no libc.
 *
 * Copyright (c) 2026 SH2ZAM Contributors. MIT License.
 * Inspired by SH4ZAM by Falco Girgis and SH4ZAM Contributors.
 */
#include "sh2zam/scene_experimental.h"

_Static_assert(SH2Z_SCENE_FADE_MAX == 32u, "the fade turnaround shifts by 5");

void *sh2z_arena_alloc(sh2z_arena_t *arena, uint32_t bytes, uint32_t align) {
    if (align == 0u || align > 4096u || (align & (align - 1u)) != 0u) { ++arena->refused; return NULL; }
    const uintptr_t at = arena->base + arena->used;
    const uintptr_t aligned = (at + (uintptr_t)(align - 1u)) & ~(uintptr_t)(align - 1u);
    const uint32_t pad = (uint32_t)(aligned - at);
    const uint32_t room = arena->size - arena->used;
    if (aligned < at || pad > room || bytes > room - pad) { ++arena->refused; return NULL; }
    arena->used += pad + bytes;
    if (arena->used > arena->high) arena->high = arena->used;
    return (void *)aligned;
}

static void reset_arenas(const sh2z_scene_t *scene) {
    if (!scene->arenas) return;
    for (uint32_t i = 0; i < scene->arena_count; ++i)
        if (scene->arenas[i]) sh2z_arena_reset(scene->arenas[i]);
}

static void set_phase(sh2z_scene_seq_t *seq, uint32_t phase) {
    seq->phase = phase;
    seq->phase_frame = 0u;
}

/* The destination's preload is done (or absent): enter it. */
static void enter_next(sh2z_scene_seq_t *seq) {
    const uint32_t target = seq->next;
    seq->current = target;
    seq->next = SH2Z_SCENE_NONE;
    ++seq->epoch;
    ++seq->switches;
    if (!seq->scenes[target].enter(seq->user, target)) { set_phase(seq, SH2Z_SCENE_FAULT); return; }
    set_phase(seq, SH2Z_SCENE_ENTER);
}

bool sh2z_scene_init(sh2z_scene_seq_t *seq, const sh2z_scene_t *scenes, uint32_t count, uint32_t first,
                     uint32_t fade_frames, uint32_t retire_frames, uint32_t preempt_rank, void *user) {
    seq->scenes = scenes;
    seq->count = count;
    seq->user = user;
    seq->current = first;
    seq->phase = SH2Z_SCENE_FAULT;
    seq->phase_frame = 0u;
    seq->fade_frames = fade_frames;
    seq->retire_frames = retire_frames;
    seq->retired = 0u;
    seq->next = SH2Z_SCENE_NONE;
    seq->preempt_rank = preempt_rank;
    seq->pending = SH2Z_SCENE_NONE;
    seq->pending_rank = 0u;
    seq->epoch = 0u;
    seq->frames = 0u;
    seq->switches = 0u;
    seq->redirects = 0u;
    if (!scenes || count == 0u || first >= count || fade_frames == 0u || fade_frames > SH2Z_SCENE_FADE_MAX)
        return false;
    for (uint32_t i = 0; i < count; ++i)
        if (!scenes[i].enter || !scenes[i].tick || !scenes[i].leave || !scenes[i].release) return false;
    seq->epoch = 1u;
    if (!scenes[first].enter(user, first)) return false;
    set_phase(seq, SH2Z_SCENE_ENTER);
    return true;
}

bool sh2z_scene_request(sh2z_scene_seq_t *seq, uint32_t target, uint32_t rank) {
    if (seq->phase == SH2Z_SCENE_FAULT || target >= seq->count) return false;
    if (seq->phase == SH2Z_SCENE_ENTER || seq->phase == SH2Z_SCENE_RUN) {
        if (seq->pending != SH2Z_SCENE_NONE && rank <= seq->pending_rank) return false;
        seq->pending = target;
        seq->pending_rank = rank;
        return true;
    }
    /* The transition has begun: only a preempting request redirects it. */
    if (rank < seq->preempt_rank || rank <= seq->pending_rank) return false;
    seq->pending_rank = rank;
    seq->pending = target;
    if (seq->next != target) {
        seq->next = target;
        ++seq->redirects;
        if (seq->phase == SH2Z_SCENE_PRELOAD) seq->phase_frame = 0u;
    }
    return true;
}

/* n * SH2Z_SCENE_FADE_MAX / d, rounded down, for n <= d <= SH2Z_SCENE_FADE_MAX,
 * without a divide (at most SH2Z_SCENE_FADE_MAX steps). */
static uint32_t scale(uint32_t n, uint32_t d) {
    uint32_t q = 0u;
    for (uint32_t r = n * SH2Z_SCENE_FADE_MAX; r >= d; r -= d) ++q;
    return q;
}

/* The fade level for the phase and frame count reached. */
static uint32_t level(const sh2z_scene_seq_t *seq) {
    const uint32_t fade = seq->fade_frames;
    switch (seq->phase) {
    case SH2Z_SCENE_ENTER:
        return seq->phase_frame >= fade ? 0u : SH2Z_SCENE_FADE_MAX - scale(seq->phase_frame, fade);
    case SH2Z_SCENE_RUN:
        return 0u;
    case SH2Z_SCENE_LEAVE:
        return seq->phase_frame >= fade ? SH2Z_SCENE_FADE_MAX : scale(seq->phase_frame, fade);
    default:
        return SH2Z_SCENE_FADE_MAX;
    }
}

static uint32_t leave_step(sh2z_scene_seq_t *seq) {
    ++seq->phase_frame;
    const uint32_t out = level(seq);
    if (seq->phase_frame >= seq->fade_frames) {
        set_phase(seq, SH2Z_SCENE_RETIRE);
        seq->retired = 0u;
    }
    return out;
}

uint32_t sh2z_scene_frame(sh2z_scene_seq_t *seq) {
    ++seq->frames;
    const sh2z_scene_t *const scenes = seq->scenes;
    switch (seq->phase) {
    case SH2Z_SCENE_ENTER:
    case SH2Z_SCENE_RUN:
        if (seq->pending != SH2Z_SCENE_NONE) {
            /* A request made by the last tick (or by the caller): fade out from
             * the level reached, so an interrupted fade in turns around. */
            const uint32_t from = level(seq);
            seq->next = seq->pending;
            scenes[seq->current].leave(seq->user, seq->current);
            set_phase(seq, SH2Z_SCENE_LEAVE);
            seq->phase_frame = (from * seq->fade_frames) >> 5; /* / SH2Z_SCENE_FADE_MAX (32) */
            return leave_step(seq);
        }
        scenes[seq->current].tick(seq->user, seq->current);
        ++seq->phase_frame;
        {
            const uint32_t out = level(seq);
            if (seq->phase == SH2Z_SCENE_ENTER && seq->phase_frame >= seq->fade_frames) set_phase(seq, SH2Z_SCENE_RUN);
            return out;
        }
    case SH2Z_SCENE_LEAVE:
        return leave_step(seq);
    case SH2Z_SCENE_RETIRE:
        ++seq->phase_frame;
        if (!seq->retired || seq->phase_frame < seq->retire_frames) return SH2Z_SCENE_FADE_MAX;
        scenes[seq->current].release(seq->user, seq->current);
        reset_arenas(&scenes[seq->current]);
        set_phase(seq, SH2Z_SCENE_PRELOAD);
        /* The destination's preload starts on this frame. */
        /* fallthrough */
    case SH2Z_SCENE_PRELOAD: {
        const uint32_t target = seq->next;
        ++seq->phase_frame;
        if (!scenes[target].preload || scenes[target].preload(seq->user, target)) {
            seq->pending = SH2Z_SCENE_NONE;
            seq->pending_rank = 0u;
            enter_next(seq);
        }
        return SH2Z_SCENE_FADE_MAX;
    }
    default:
        return SH2Z_SCENE_FADE_MAX;
    }
}
