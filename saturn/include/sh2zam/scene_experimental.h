/* Experimental scene sequencing: a fixed table of scenes (title, menus,
 * loading, game, results ...), fade transitions, scene-owned memory arenas
 * and press-edge filtering that survives a transition. Not selected by any
 * default dispatch.
 *
 * Contract tag: scene-1. Pure bookkeeping, no hardware access: the caller
 * reads the pad, draws, fades and waits; these functions only decide and
 * record. Every function is exact and deterministic.
 *
 * A. Arena (sh2z_arena_t): a bump allocator over one caller range. Each
 *    allocation is aligned to a power of two; reset frees everything at
 *    once. The high-water mark survives resets, so a caller can prove that a
 *    scene uses the same memory on every visit.
 * B. Sequencer (sh2z_scene_seq_t). The caller owns a table of scenes, each
 *    with up to five callbacks:
 *      preload optional; one bounded chunk of loading before enter, called
 *              once per frame until it returns true;
 *      enter   the scene starts; false is a fault (the sequencer stops);
 *      tick    one presented frame while the scene is entering or running;
 *      leave   the transition away from the scene has begun (once);
 *      release every displayed list that referenced the scene has been
 *              replaced; it must give back every resource it took (the
 *              sequencer then resets its arenas, if any).
 *    sh2z_scene_frame() runs once per presented frame and moves through the
 *    phases ENTER (fade in), RUN, LEAVE (fade out), RETIRE, PRELOAD and back
 *    to ENTER for the next scene. It returns the fade level for the frame, 0
 *    (fully shown) to SH2Z_SCENE_FADE_MAX (black).
 *    RETIRE ends only when the caller has confirmed, with
 *    sh2z_scene_retired(), that the last displayed list referencing the
 *    leaving scene has been replaced (the caller knows its display pipeline;
 *    elapsed frames alone prove nothing), and at least retire_frames frames
 *    have passed in RETIRE. A confirmation given before RETIRE began does
 *    not count.
 * C. Requests. A scene, or the caller, asks for a scene change with a rank.
 *    At most one request is pending; a request replaces the pending one only
 *    with a higher rank. Once the fade out has begun, only a request of rank
 *    preempt_rank or more can still change the destination, and it does so
 *    without restarting the fade (a reset during a transition); during
 *    PRELOAD the preload starts again for the new destination. A request
 *    for the current scene restarts it (leave, retire, release, preload,
 *    enter).
 * D. Press edges (sh2z_scene_input_t). Edges are presses not yet consumed;
 *    consuming latches the buttons held at that moment, and a latched
 *    button gives no edge again until it has been released. So the press
 *    that confirmed one screen cannot confirm the next one, however long
 *    the transition between them.
 *
 * No function uses floating point, division, global state, libc or
 * allocation.
 *
 * Copyright (c) 2026 SH2ZAM Contributors. MIT License.
 * Inspired by SH4ZAM by Falco Girgis and SH4ZAM Contributors.
 */
#ifndef SH2ZAM_SCENE_EXPERIMENTAL_H
#define SH2ZAM_SCENE_EXPERIMENTAL_H

#include "sh2zam.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Contract tag scene-1 ------------------------------------------------- */

/* ---- A. Arena ---- */
typedef struct {
    uintptr_t base;  /**< first byte of the range */
    uint32_t size;   /**< bytes in the range */
    uint32_t used;   /**< bytes allocated since the last reset, padding included */
    uint32_t high;   /**< largest `used` ever reached */
    uint32_t refused;/* allocations refused for lack of room */
} sh2z_arena_t;

/** Takes the range [base, base + size). */
static inline void sh2z_arena_init(sh2z_arena_t *arena, void *base, uint32_t size) {
    arena->base = (uintptr_t)base;
    arena->size = size;
    arena->used = 0u;
    arena->high = 0u;
    arena->refused = 0u;
}

/** `bytes` from the arena at a multiple of `align` (a power of two, 1..4096,
 * counted from address 0). NULL, and nothing changes but `refused`, when the
 * range has no room or `align` is not a power of two in range. A request of
 * zero bytes returns the aligned position without taking room. */
void *sh2z_arena_alloc(sh2z_arena_t *arena, uint32_t bytes, uint32_t align);

/** Frees everything; `high` and `refused` are kept. */
static inline void sh2z_arena_reset(sh2z_arena_t *arena) { arena->used = 0u; }

/* ---- B. Sequencer ---- */
#define SH2Z_SCENE_FADE_MAX 32u
#define SH2Z_SCENE_NONE     UINT32_C(0xFFFFFFFF)

enum {
    SH2Z_SCENE_ENTER = 0,   /**< fading in */
    SH2Z_SCENE_RUN = 1,
    SH2Z_SCENE_LEAVE = 2,   /**< fading out */
    SH2Z_SCENE_RETIRE = 3,  /**< black; waiting until the leaving scene's lists are replaced */
    SH2Z_SCENE_PRELOAD = 4, /**< black; the next scene's preload chunks */
    SH2Z_SCENE_FAULT = 5    /**< an enter returned false; nothing runs */
};

typedef struct {
    bool (*preload)(void *user, uint32_t scene); /**< NULL: nothing to preload */
    bool (*enter)(void *user, uint32_t scene);
    void (*tick)(void *user, uint32_t scene);
    void (*leave)(void *user, uint32_t scene);
    void (*release)(void *user, uint32_t scene);
    sh2z_arena_t *const *arenas; /**< NULL, or `arena_count` arenas reset after release */
    uint32_t arena_count;
} sh2z_scene_t;

typedef struct {
    const sh2z_scene_t *scenes;
    uint32_t count;
    void *user;
    uint32_t current;       /**< the scene shown */
    uint32_t phase;         /**< SH2Z_SCENE_ENTER ... SH2Z_SCENE_FAULT */
    uint32_t phase_frame;   /**< frames spent in the phase */
    uint32_t fade_frames;   /**< frames of a fade in or out, 1..SH2Z_SCENE_FADE_MAX */
    uint32_t retire_frames; /**< minimum frames in RETIRE */
    uint32_t retired;       /**< the caller confirmed replacement during this RETIRE */
    uint32_t next;          /**< destination of the transition in progress */
    uint32_t preempt_rank;  /**< rank that may still redirect a transition */
    uint32_t pending;       /**< requested scene, SH2Z_SCENE_NONE when none */
    uint32_t pending_rank;
    uint32_t epoch;         /**< increments each time a scene is entered */
    uint32_t frames;        /**< sh2z_scene_frame() calls */
    uint32_t switches;      /**< scenes entered after the first */
    uint32_t redirects;     /**< transitions redirected by a preempting request */
} sh2z_scene_seq_t;

/** Sets up the sequencer and enters scene `first` (fading in). False when the
 * table is empty, `first` is out of range, `fade_frames` is not in
 * 1..SH2Z_SCENE_FADE_MAX or the first enter fails (phase FAULT). */
bool sh2z_scene_init(sh2z_scene_seq_t *seq, const sh2z_scene_t *scenes, uint32_t count, uint32_t first,
                     uint32_t fade_frames, uint32_t retire_frames, uint32_t preempt_rank, void *user);

/* ---- C. Requests ---- */
/** Asks for scene `target` with `rank`. True if the request is now the one
 * that will be carried out. False when `target` is out of range, a pending
 * request has an equal or higher rank, the transition has begun and `rank`
 * is below preempt_rank, or the sequencer has faulted. */
bool sh2z_scene_request(sh2z_scene_seq_t *seq, uint32_t target, uint32_t rank);

/** One presented frame: calls the callbacks the phase needs (tick while
 * entering and running; leave, release and enter at the transitions) and
 * returns the fade level of the frame, 0..SH2Z_SCENE_FADE_MAX. A tick that
 * makes a request starts the fade out on the next frame. */
uint32_t sh2z_scene_frame(sh2z_scene_seq_t *seq);

/** The caller confirms that the last displayed list referencing the leaving
 * scene has been replaced by one built after the fade out. Ignored outside
 * RETIRE. */
static inline void sh2z_scene_retired(sh2z_scene_seq_t *seq) {
    if (seq->phase == SH2Z_SCENE_RETIRE) seq->retired = 1u;
}

/** True while the shown scene should draw: entering, running or leaving. */
static inline bool sh2z_scene_drawing(const sh2z_scene_seq_t *seq) {
    return seq->phase <= SH2Z_SCENE_LEAVE;
}

/* ---- D. Press edges ---- */
typedef struct {
    uint32_t held;         /**< buttons down this step */
    uint32_t edges;        /**< buttons pressed this step and not latched */
    uint32_t repeat;       /**< edges, plus auto-repeats of repeat_mask buttons */
    uint32_t previous;     /**< buttons down the step before */
    uint32_t latched;      /**< buttons consumed and not yet released */
    uint32_t repeat_mask;  /**< buttons that auto-repeat (for example the directions) */
    uint32_t repeat_delay; /**< steps held before the first repeat, 1 or more */
    uint32_t repeat_rate;  /**< steps between later repeats, 1 or more */
    uint32_t repeat_held;  /**< steps the current repeat_mask set has stayed down unchanged */
    uint32_t repeat_due;   /**< value of repeat_held at which the next repeat falls */
} sh2z_scene_input_t;

/** Clears the state and sets the auto-repeat (a delay or rate of 0 is taken
 * as 1). */
static inline void sh2z_scene_input_init(sh2z_scene_input_t *in, uint32_t repeat_mask, uint32_t delay, uint32_t rate) {
    in->held = 0u; in->edges = 0u; in->repeat = 0u; in->previous = 0u; in->latched = 0u; in->repeat_held = 0u;
    in->repeat_mask = repeat_mask;
    in->repeat_delay = delay ? delay : 1u;
    in->repeat_rate = rate ? rate : 1u;
    in->repeat_due = in->repeat_delay;
}

/** Records one step's buttons (bit set = down) and returns the edges. Call it
 * once per step; when several steps share one pad reading, pass the same
 * buttons each time and the edge is reported on the first step only. A
 * repeat falls repeat_delay steps after the press and then every
 * repeat_rate steps, while the same unlatched repeat_mask buttons stay down. */
static inline uint32_t sh2z_scene_input_update(sh2z_scene_input_t *in, uint32_t held) {
    in->latched &= held;
    in->edges = held & ~in->previous & ~in->latched;
    const uint32_t dir = held & in->repeat_mask & ~in->latched;
    const uint32_t before = in->previous & in->repeat_mask & ~in->latched;
    in->repeat = in->edges;
    if (dir == 0u || dir != before) {
        in->repeat_held = 0u;
        in->repeat_due = in->repeat_delay;
    } else if (++in->repeat_held == in->repeat_due) {
        in->repeat |= dir;
        in->repeat_due += in->repeat_rate;
    }
    in->previous = held;
    in->held = held;
    return in->edges;
}

/** Latches every button held now: none of them gives an edge again until it
 * has been released. Call when a press starts a transition. */
static inline void sh2z_scene_input_consume(sh2z_scene_input_t *in) {
    in->latched |= in->held;
    in->edges = 0u;
    in->repeat = 0u;
    in->repeat_held = 0u;
    in->repeat_due = in->repeat_delay;
}

#ifdef __cplusplus
}
#endif

#endif /* SH2ZAM_SCENE_EXPERIMENTAL_H */
