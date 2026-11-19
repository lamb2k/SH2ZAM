/* Experimental racing-line follower: fixed-point kinematics for agents that
 * travel along a closed line (a lap course, a patrol loop, a rail).
 * Game-neutral: the library knows nothing about what an agent is.
 *
 * Contract tag line-follower-1.
 *
 * Model. An agent's state is a distance along the line plus a lateral offset
 * from it, with a forward speed and a lateral velocity (sh2z_track_state_t).
 * The line is a list of nodes baked offline: each node starts at a distance
 * and carries a target lateral offset, a speed cap and a usable half width,
 * linearly interpolated to the next node. The same state is meant for any
 * vehicle that moves in line space, so a later full vehicle model can reuse
 * the line sampling and the integration below.
 *
 * Units (all two's-complement int32 unless stated):
 *   distance, lateral, half width     Q8 units (1/256 of a whole unit)
 *   speed, lateral velocity           Q8 units per tick
 *   gains                             Q16 fractions (65536 = 1.0)
 * The line length must lie in (0, 2^30) Q8 units, node starts must be
 * strictly increasing from 0 and every node span must lie in [2, 65536]. Distances wrap at the length; a wrap counts a
 * lap. Totals (lap * length + distance) must stay below 2^31.
 *
 * Determinism. Every function is integer-only, branch-for-branch identical on
 * every target, allocation-free, reentrant and without global state. Steps of
 * n ticks at once (the level-of-detail scheduler) integrate the same formulas
 * with n-scaled limits; they approximate n single steps, they are not equal to
 * them. Callers that need bit-identical histories must keep the same schedule.
 *
 * SH-2: compiled C. The only multiplies are 32x32->64 (DMULS) for gains; there
 * is no division at run time (nodes carry a baked reciprocal of their span).
 *
 * Copyright (c) 2026 SH2ZAM Contributors. MIT License.
 * Inspired by SH4ZAM by Falco Girgis and SH4ZAM Contributors.
 */
#ifndef SH2ZAM_LINE_FOLLOWER_EXPERIMENTAL_H
#define SH2ZAM_LINE_FOLLOWER_EXPERIMENTAL_H
#include "sh2zam.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Contract tag line-follower-1 -------------------------------------- */

/** One baked line node (16 bytes). span is the Q8 distance to the next node
 *  (the last node spans to the line length) and must lie in [2, 65536]: bakers
 *  split longer spans into several nodes, which keeps the interpolation within
 *  one Q16 step of exact. inv_span = floor(2^32 / span), computed offline by
 *  sh2z_line_node_inv_span. */
typedef struct {
    int32_t start;        /**< Q8 distance of the node start. */
    int16_t lateral;      /**< Target lateral offset, whole units (+ = right). */
    uint16_t speed_cap;   /**< Speed cap, Q4 units per tick. */
    uint16_t half_width;  /**< Usable half width, whole units. */
    uint16_t flags;       /**< Caller-defined node flags, passed through. */
    uint32_t inv_span;    /**< floor(2^32 / span), span in Q8 (offline). */
} sh2z_line_node_t;

typedef struct {
    const sh2z_line_node_t *nodes;
    uint32_t count;       /**< >= 1 */
    int32_t length;       /**< Q8 line length. */
} sh2z_line_t;

/** Interpolated line values at a distance. */
typedef struct {
    int32_t lateral;      /**< Q8 */
    int32_t speed_cap;    /**< Q8 per tick */
    int32_t half_width;   /**< Q8 */
    uint16_t flags;       /**< flags of the node containing the distance */
} sh2z_line_sample_t;

/** Line-space vehicle state, shared by every user of the line. */
typedef struct {
    int32_t dist;         /**< Q8, in [0, length). */
    int32_t lateral;      /**< Q8 */
    int32_t speed;        /**< Q8 per tick, >= 0. */
    int32_t lat_vel;      /**< Q8 per tick. */
    int32_t lap;          /**< Completed wraps. */
    uint32_t node;        /**< Cached node index (hint for the next lookup). */
} sh2z_track_state_t;

/** Follower tuning; limits are per tick and scale with the step count. */
typedef struct {
    int32_t accel;        /**< Q8 per tick^2, max speed gain. */
    int32_t brake;        /**< Q8 per tick^2, max speed loss. */
    int32_t steer_gain;   /**< Q16 share of the lateral error turned into lateral velocity. */
    int32_t steer_max;    /**< Q8 per tick, max |lateral velocity|. */
    int32_t margin;       /**< Q8 kept clear of the half width. */
} sh2z_follow_params_t;

/** Close-range contact tuning (line space). */
typedef struct {
    int32_t reach_long;   /**< Q8 longitudinal contact distance. */
    int32_t reach_lat;    /**< Q8 lateral contact distance. */
    int32_t bump_keep;    /**< Q16 share of the trailing speed kept above the leader's on contact. */
} sh2z_contact_params_t;

/** Offline helper: floor(2^32 / span) for 2 <= span <= 65536 (Q8); 0 for any other span. */
uint32_t sh2z_line_node_inv_span(int32_t span);

/** Node containing dist (0 <= dist < length), searched from hint. */
uint32_t sh2z_line_locate(const sh2z_line_t *line, int32_t dist, uint32_t hint);

/** Interpolated sample at dist inside node. */
void sh2z_line_sample(const sh2z_line_t *line, uint32_t node, int32_t dist, sh2z_line_sample_t *out);

/** Total distance travelled: lap * length + dist. */
static inline int32_t sh2z_track_total(const sh2z_line_t *line, const sh2z_track_state_t *s) {
    return s->lap * line->length + s->dist;
}

/** Signed race gap from b to a on totals (a ahead of b is positive). */
static inline int32_t sh2z_track_gap(const sh2z_line_t *line, const sh2z_track_state_t *a, const sh2z_track_state_t *b) {
    return sh2z_track_total(line, a) - sh2z_track_total(line, b);
}

/** Signed physical gap from b to a along the closed line, ignoring laps,
 *  in (-length/2, length/2]: what proximity tests use. */
static inline int32_t sh2z_track_near_gap(const sh2z_line_t *line, const sh2z_track_state_t *a, const sh2z_track_state_t *b) {
    const int32_t half = line->length >> 1;
    int32_t d = a->dist - b->dist;
    if(d > half) d -= line->length;
    else if(d <= -half) d += line->length;
    return d;
}

/** Moves dist by speed * steps, wrapping and counting laps, and refreshes node. */
void sh2z_track_advance(const sh2z_line_t *line, sh2z_track_state_t *s, uint32_t steps);

/** Steers toward target_lateral (Q8) and target_speed (Q8 per tick), capped by
 *  the line's speed cap and half width, for steps ticks (1..64), then advances. */
void sh2z_follow_step(const sh2z_line_t *line, const sh2z_follow_params_t *params, sh2z_track_state_t *s,
                      int32_t target_speed, int32_t target_lateral, uint32_t steps);

/** Pace controller: base + gain * (expected - actual), clamped to [lo, hi]. All Q8. */
int32_t sh2z_pace_target(int32_t base, int32_t expected_total, int32_t actual_total, int32_t gain_q16,
                         int32_t lo, int32_t hi);

/** Lateral target that avoids an agent ahead: if ahead is within reach_long in
 *  front and within reach_lat sideways of own_target, returns own_target moved
 *  to ahead->lateral -/+ offset (the side own_target is already on). */
int32_t sh2z_follow_avoid(const sh2z_line_t *line, const sh2z_track_state_t *self, const sh2z_track_state_t *ahead,
                          int32_t own_target, int32_t reach_long, int32_t reach_lat, int32_t offset);

/** Distance level of detail: period 1, 2, 4 or 8 ticks for |gap| below
 *  near[0], near[1], near[2], or beyond. */
uint32_t sh2z_lod_period(int32_t gap, const int32_t near[3]);

/** Ticks due now for an agent last stepped at last_tick: 0 if fewer than
 *  period ticks have passed, otherwise now - last_tick (catch-up keeps time exact). */
static inline uint32_t sh2z_lod_due(uint32_t now, uint32_t last_tick, uint32_t period) {
    const uint32_t elapsed = now - last_tick;
    return elapsed >= period ? elapsed : 0u;
}

/** Orders count (<= 255) agents by total distance (race rank), leader first,
 *  ties by lower index. order[] holds a permutation of 0..count-1 and is
 *  updated in place (insertion sort: cheap when nearly sorted). */
void sh2z_rank_sort(uint8_t *order, const sh2z_line_t *line, const sh2z_track_state_t *states, uint32_t count);

/** Orders count (<= 255) agents by physical distance along the line (dist,
 *  largest first, ties by lower index), for proximity scans. In place. */
void sh2z_order_by_dist(uint8_t *order, const sh2z_track_state_t *states, uint32_t count);

/** Resolves contacts between agents near each other on the line. order[] must
 *  come from sh2z_order_by_dist. Every pair whose physical gap is below
 *  reach_long and whose lateral distance is below reach_lat is pushed apart
 *  sideways by half the overlap each, clamped to the half width at each agent,
 *  and the trailing agent's speed is limited to the leader's plus bump_keep of
 *  the difference. Pairs are visited in a fixed order (deterministic).
 *  Returns the number of contacts. */
uint32_t sh2z_contact_resolve(const sh2z_line_t *line, const sh2z_contact_params_t *params,
                              sh2z_track_state_t *states, const uint8_t *order, uint32_t count);

/** Same rule against one external body that is not moved (for example a
 *  separately simulated vehicle): only the agents are pushed. Returns contacts. */
uint32_t sh2z_contact_external(const sh2z_line_t *line, const sh2z_contact_params_t *params,
                               sh2z_track_state_t *states, uint32_t count, const sh2z_track_state_t *body);

#ifdef __cplusplus
}
#endif
#endif
