/* Experimental line field: a field of vehicles racing along one closed line,
 * built on the racing-line follower. Start grid, pace and chase targets,
 * lane choice, avoiding and overtaking, boost, contact with energy loss and
 * retirement, distance level of detail, rank and lap bookkeeping.
 * Game-neutral: the library knows nothing about what a vehicle is.
 *
 * Contract tag line-field-1.
 *
 * Shared vehicle state. sh2z_vehicle_t is the line-space state of ANY vehicle
 * on the line: agents the field drives, and vehicles driven elsewhere (a
 * player's own dynamics) that set SH2Z_VEH_EXTERNAL. External vehicles take
 * part in ordering, contact, rank and lap bookkeeping; the field never moves
 * them. Contact against them is reported in kick_lat / kick_speed (Q8 deltas
 * the owner applies and clears) and charged to their energy.
 *
 * Units as line_follower_experimental.h: Q8 distance, lateral and speed per
 * tick; Q16 gains. Energy is Q8. Totals (lap * length + dist) must stay
 * inside (-2^29, 2^29). At most 255 vehicles.
 *
 * Level of detail. With a focus vehicle set, agents farther from it than
 * lod_near[] step every 2, 4 or 8 ticks (catching up the ticks they skipped).
 * Agents on the same 512-unit stretch of the line share a phase. Contact is
 * resolved between vehicles that are both current on a tick, so everything
 * near the focus is exact and far pairs on different phases do not touch.
 * Rank extrapolates a skipped agent to the current tick.
 *
 * Determinism. One xorshift32 state in the field drives every random choice;
 * the same initial state and the same external inputs give the same result
 * on every target. No division, no floating point, no allocation, no global
 * state. sh2z_field_step is not reentrant on one field.
 *
 * Copyright (c) 2026 SH2ZAM Contributors. MIT License.
 * Inspired by SH4ZAM by Falco Girgis and SH4ZAM Contributors.
 */
#ifndef SH2ZAM_LINE_FIELD_EXPERIMENTAL_H
#define SH2ZAM_LINE_FIELD_EXPERIMENTAL_H
#include "line_follower_experimental.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Contract tag line-field-1 ------------------------------------------ */

#define SH2Z_VEH_EXTERNAL 0x0001u /**< Driven outside the field. */
#define SH2Z_VEH_RETIRED  0x0002u /**< Out of energy: out of contact, coasting to a stop. */
#define SH2Z_VEH_FINISHED 0x0004u /**< Completed the race distance. */
#define SH2Z_VEH_BOOST    0x0008u /**< Boost active. */
#define SH2Z_VEH_STOPPED  0x0010u /**< Retired and at rest: no longer stepped. */
#define SH2Z_VEH_WALL     0x0020u /**< Touched the line's edge this step. */
#define SH2Z_VEH_TOUCH    0x0040u /**< In contact with a vehicle this step. */

/** Line-space vehicle: shared by field agents and externally driven vehicles. */
typedef struct {
    sh2z_track_state_t t;      /**< Distance, lateral, speed, lap. */
    int32_t energy;            /**< Q8, 0 .. max_energy. */
    int32_t max_energy;        /**< Q8 */
    int32_t accel;             /**< Q8 per tick^2 */
    int32_t brake;             /**< Q8 per tick^2 */
    int32_t top_speed;         /**< Q8 per tick, without boost. */
    int32_t grip;              /**< Q8 per tick, max lateral speed. */
    int32_t kick_lat;          /**< Q8 contact lateral delta (external vehicles). */
    int32_t kick_speed;        /**< Q8 contact speed delta, <= 0 (external vehicles). */
    uint16_t armor;            /**< Q8 damage taken multiplier, 256 = normal. */
    uint16_t flags;            /**< SH2Z_VEH_* */
    int16_t boost_ticks;       /**< Remaining boost ticks. */
    uint8_t rank;              /**< Place, 0 = leader. */
    uint8_t id;                /**< Index in the field. */
} sh2z_vehicle_t;

/** Per-agent decision state. */
typedef struct {
    int32_t pace;              /**< Q8 per tick: line length / target lap ticks. */
    int32_t expect;            /**< Q8 expected total distance at this tick. */
    int32_t lane;              /**< Q8 personal offset from the line's lateral. */
    int32_t target_lat;        /**< Q8 current lateral target. */
    int32_t target_speed;      /**< Q8 current speed target. */
    uint32_t last_tick;        /**< Field tick of the last step (level of detail). */
    int32_t lap_tick[4];       /**< Race tick at which laps 1..4 ended, -1 before. */
    int32_t finish_tick;       /**< Race tick of the finish, -1 before. */
    int32_t retire_tick;       /**< Race tick of retirement, -1 before. */
    int16_t chase;             /**< Vehicle index to pace against, or -1. */
    int16_t chase_lead;        /**< Whole units to hold ahead (+) of the chased vehicle. */
    int16_t laps_seen;         /**< Lap counter at the last bookkeeping pass. */
    uint8_t aggression;        /**< Chance in 1/256, per decision, of a side attack on the chased vehicle when alongside it. */
    uint8_t pass_ticks;        /**< Ticks left holding a pass or avoid offset. */
    int8_t pass_dir;           /**< -1 left, +1 right, 0 none. */
    uint8_t attack_ticks;      /**< Ticks left in a side attack. */
    uint8_t attack_target;     /**< Vehicle index under attack. */
    uint8_t finish_seq;        /**< Finishing order, 0 = winner. */
    uint8_t touch_cool;        /**< Ticks left of the current contact incident. */
} sh2z_field_brain_t;

/** Field tuning. */
typedef struct {
    int32_t pace_gain;         /**< Q16 speed per Q8 of distance behind expectation. */
    int32_t pace_max;          /**< Q16 cap on target speed as a share of pace (for example 1.25). */
    int32_t chase_gain;        /**< Q16 speed per Q8 of gap to the chase point. */
    int32_t steer_gain;        /**< Q16 share of lateral error steered per tick. */
    int32_t margin;            /**< Q8 kept clear of the half width. */
    int32_t avoid_long;        /**< Q8 look-ahead for the vehicle ahead. */
    int32_t avoid_lat;         /**< Q8 lateral clearance wanted. */
    int32_t avoid_ticks;       /**< Look-ahead added per Q8 of closing speed, in ticks. */
    int32_t avoid_offset;      /**< Q8 offset taken around the vehicle ahead. */
    int32_t pass_hold;         /**< Ticks a pass side is held. */
    int32_t contact_long;      /**< Q8 */
    int32_t contact_lat;       /**< Q8 */
    int32_t bump_keep;         /**< Q16 */
    int32_t hit_energy;        /**< Q16 energy per Q8 of closing speed. */
    int32_t side_energy;       /**< Q8 energy per contact. */
    int32_t attack_energy;     /**< Q8 extra energy taken by an attacked vehicle. */
    int32_t wall_energy;       /**< Q8 energy per tick on the edge. */
    int32_t wall_keep;         /**< Q16 speed kept per tick on the edge. */
    int32_t attack_long;       /**< Q8 */
    int32_t attack_lat;        /**< Q8 */
    int32_t boost_speed;       /**< Q8 per tick added to the target and top speed. */
    int32_t boost_energy;      /**< Q8 energy per boost. */
    int32_t boost_min_share;   /**< Q16 share of max energy needed to boost. */
    int32_t boost_min_cap;     /**< Q8 per tick line speed cap needed to boost. */
    int32_t boost_behind;      /**< Q8 distance behind expectation that triggers a boost. */
    int32_t cruise;            /**< Q8 per tick after the finish. */
    int32_t rank_lead;         /**< Q8 lead needed to take a place (>= 0; 0 swaps on any lead). */
    int32_t lod_near[3];       /**< Q8 gaps to the focus for periods 1, 2, 4 (else 8). */
    int16_t boost_ticks;       /**< Boost length. */
    uint8_t laps;              /**< Race length in laps (1..4). */
    uint8_t boost_from_lap;    /**< Completed laps needed before boosting. */
} sh2z_field_params_t;

/** The field. The caller owns every array (count entries each). */
typedef struct {
    const sh2z_line_t *line;
    const sh2z_field_params_t *params;
    sh2z_vehicle_t *veh;
    sh2z_field_brain_t *brain;
    uint8_t *order;            /**< Physical order (largest dist first). */
    uint8_t *slot;             /**< slot[i] = position of vehicle i in order. */
    uint8_t *rank_order;       /**< Race order, leader first. */
    uint32_t count;            /**< 1..255 */
    uint32_t tick;             /**< Field ticks since init. */
    uint32_t start_tick;       /**< Field tick of the race start. */
    uint32_t rng;              /**< xorshift32 state, never 0. */
    int32_t focus;             /**< Vehicle whose surroundings run every tick, or -1. */
    uint32_t started;          /**< 0 before sh2z_field_start. */
    uint32_t finished;         /**< Vehicles finished. */
    uint32_t retired;          /**< Vehicles retired. */
    uint32_t contacts;         /**< Contact incidents so far (a pair touching counts once). */
    uint32_t rank_changes;     /**< Ticks on which the race order changed. */
    uint32_t steps;            /**< Agent steps executed (level-of-detail accounting). */
} sh2z_field_t;

/** Start-grid slot for grid position p (0 = pole): rows of `columns` (1..4)
 *  staggered across the line, one position per `row_step`.
 *  dist = length - first_back - p * row_step (wrapped, lap -1);
 *  lateral = (columns - 1 - 2 * (p % columns)) * column_half. All Q8. */
void sh2z_field_grid_slot(const sh2z_line_t *line, uint32_t p, int32_t first_back, int32_t row_step,
                          int32_t column_half, uint32_t columns, sh2z_track_state_t *out);

/** Binds the arrays, zeroes the counters, seeds the generator (seed 0 becomes
 *  1) and resets every brain's bookkeeping. Vehicles and brain tuning (pace,
 *  lane, chase, aggression) are the caller's to fill before or after. */
void sh2z_field_init(sh2z_field_t *f, const sh2z_line_t *line, const sh2z_field_params_t *params,
                     sh2z_vehicle_t *veh, sh2z_field_brain_t *brain, uint8_t *order, uint8_t *slot,
                     uint8_t *rank_order, uint32_t count, uint32_t seed);

/** Starts the race: from the next step agents drive and expectations run. */
void sh2z_field_start(sh2z_field_t *f);

/** One tick: orders, decides and moves the agents that are due, resolves
 *  contacts (external vehicles included), applies energy, retirement, laps,
 *  finish and rank. External vehicles must be updated by their owner first. */
void sh2z_field_step(sh2z_field_t *f);

/** FNV-1a hash of the whole field state, vehicles in index order (determinism checks). */
uint32_t sh2z_field_hash(const sh2z_field_t *f);

#ifdef __cplusplus
}
#endif
#endif
