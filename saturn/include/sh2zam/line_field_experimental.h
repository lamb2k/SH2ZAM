/* Experimental line field: a field of vehicles racing along one closed line,
 * built on the racing-line follower. Start grid, pace and chase targets,
 * lane choice, avoiding and overtaking, boost, contact with energy loss and
 * retirement, distance level of detail, rank and lap bookkeeping.
 * Game-neutral: the library knows nothing about what a vehicle is.
 *
 * Contract tags line-field-1, line-field-2 (the second-lap re-target),
 * line-field-3 (the first-lap chase cap), line-field-4 (surface zones and
 * tube nodes) and line-field-5 (tube roll, lateral inertia, falls with a
 * knock-out credit, a near-focus knock-out gate).
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
 * Second-lap re-target (line-field-2). An agent whose brain has a finish
 * target (finish_ticks > 0) changes pace when it completes its first lap:
 * from then on it expects to cover the remaining laps evenly by that race
 * tick, pace = (laps - 1) * length / (finish_ticks - first lap tick), and its
 * expectation restarts at one lap. finish_ticks = 0 keeps line-field-1's one
 * pace for the whole race, step for step, and its hashes.
 *
 * First-lap chase cap (line-field-3). A chasing agent with first_lap_cap > 0
 * holds its target speed to that cap until it completes its first lap:
 * racing games often keep chasers at their machine's own top speed while the
 * field forms, and let them run faster later. 0 = no cap (line-field-2).
 *
 * Surface (line-field-4). params->surface, when set, gives the line surface
 * zones and tube nodes; NULL is line-field-3 step for step, with its hashes.
 * - Zones: a stretch of distance and a band of lateral where the ground acts
 *   on agents: a boost plate starts a free boost, low grip scales steering,
 *   rough ground damps the target speed unless boosting, a recharge strip
 *   refills energy. An agent takes the zone it is in when its step starts.
 * - Tube nodes: nodes whose flags match the surface's wrap mask have no edge.
 *   Lateral there is an arc length round a circle of radius half_width: it
 *   wraps at -pi to +pi times the radius, agents steer the short way round,
 *   are never charged at an edge, and contact pushes wrap. An agent leaving
 *   a tube for a node with an edge is put inside the new half width without
 *   an edge charge.
 * Final dash (line-field-4). With dash_from > 0, an agent on the final lap
 * past dash_from drops its pace and chase targets and races for the line:
 * it targets at least dash_pace of its pace whatever its expectation, and
 * boosts whenever its energy allows, as racing games often let their field
 * do over the last stretch.
 *
 * Behaviour (line-field-5). params->behave, when set, adds the behaviours
 * below; NULL is line-field-4 step for step, with its hashes.
 * - Tube roll: on a tube node an agent decides now and then (roll_chance per
 *   decision) to roll round the tube, moving round it by roll_lead of its
 *   speed. A roll either lasts roll_min .. roll_min + roll_span - 1
 *   decisions or, for roll_latch of the rolls, holds until the agent reaches
 *   an exit stretch: a node matching the exit mask whose node roll_exit_ahead
 *   ahead is not a tube. There the roll ends and the agent steers back to
 *   its line. Not rolling, an agent on a node matching the hold mask keeps
 *   its angle round the tube (drifting by hold_lead) instead of steering back. On other tubes, with
 *   roll_well > 0, a rolling agent instead settles roll_well of the radius
 *   round from its line (a tube that pulls back toward its line).
 * - Lateral inertia: with lat_accel > 0 the lateral velocity changes by at
 *   most lat_accel per tick toward the steering demand, and the agent may
 *   reach the edge (the margin bounds its target, not its position). An
 *   agent on the edge is charged as at a wall and its lateral velocity
 *   reverses at half its size.
 * - Air and falls: on a node matching the air mask an agent does not steer;
 *   it keeps its lateral velocity plus a drift drawn at take-off, uniform in
 *   +-air_spread of its speed. An agent in the air that is beyond the half
 *   width plus fall_margin of a node matching the open mask leaves the line: SH2Z_VEH_FALLING, out of contact, no laps. fall_ticks
 *   later it retires with energy exactly 0 (retire_tick that tick).
 * - Drops: on a node matching the drop mask, an agent that comes out of a tube beyond the half width plus
 *   fall_margin of an open-edge node is launched (launch_chance; otherwise it lands on the edge as before) and
 *   leaves the line the same way, and an agent on a drop tube that is also a hold tube (an outside surface)
 *   more than drop_lat of the radius round from the line drops off with drop_chance per step (one draw per
 *   step while beyond). Falls and drops share the knock-out credit below.
 * - Knock-out credit: an agent's ko_age counts its steps since a contact
 *   incident began with an external vehicle (1 on the contact's own tick,
 *   ko_by that vehicle). When it starts to fall with ko_age <= ko_window and
 *   ko_by not finished or retired, the fall is credited once: ko_credit is
 *   set to ko_by, the field's kos counts it and the owner applies its reward.
 * - Near-focus knock-out gate: with ko_near > 0 and a focus, contact or wall
 *   energy cannot retire an agent farther than ko_near from the focus; its
 *   energy stays at ko_floor.
 *
 * Determinism. One xorshift32 state in the field drives every random choice;
 * the same initial state and the same external inputs give the same result
 * on every target. One integer division per agent and race (the re-target);
 * no other division, no floating point, no allocation, no global
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
#define SH2Z_VEH_FALLING  0x0080u /**< line-field-5: left the line; retires fall_ticks later. */
#define SH2Z_VEH_AIR      0x0100u /**< line-field-5: in the air this step. */

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
    int32_t finish_ticks;      /**< line-field-2: race tick to finish by, from the second lap on; 0 keeps the pace. Caller's tuning. */
    int32_t first_lap_cap;     /**< line-field-3: Q8 per tick cap on a chase target before the first lap is done; 0 = none. Caller's tuning. */
    int32_t fall_tick;         /**< line-field-5: race tick the agent left the line, -1 before. */
    int32_t air_drift;         /**< line-field-5: Q16 share of speed drifted per tick in the air (drawn at take-off). */
    int16_t roll;              /**< line-field-5: > 0 decisions of roll left, < 0 rolling until an exit, 0 none. */
    int16_t ko_by;             /**< line-field-5: external vehicle of the last contact incident, -1 none. */
    int16_t ko_credit;         /**< line-field-5: vehicle credited with this agent's fall, -1 none. */
    uint16_t ko_age;           /**< line-field-5: steps since that incident began (saturates at 65535). */
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
    const struct sh2z_field_surface *surface; /**< line-field-4: zones and tube nodes, or NULL for none. */
    int32_t dash_from;         /**< line-field-4: Q8 distance into the final lap from which agents race for the line; 0 = never. */
    int32_t dash_pace;         /**< line-field-4: Q16 share of its pace an agent targets at least in the final dash. */
    const struct sh2z_field_behave *behave; /**< line-field-5: tube roll, inertia, falls, knock-out credit; NULL for none. */
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
    uint32_t falls;            /**< line-field-5: agents that left the line. */
    uint32_t kos;              /**< line-field-5: falls credited to an external vehicle. */
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

/* ---- Contract tag line-field-4 ------------------------------------------ */

#define SH2Z_FIELD_ZONE_BOOST    1u /**< Boost plate: a free boost (no energy) of plate_ticks. */
#define SH2Z_FIELD_ZONE_SLIP     2u /**< Low grip: steering gain scaled by slip_steer. */
#define SH2Z_FIELD_ZONE_ROUGH    3u /**< Rough ground: target speed scaled by rough_keep unless boosting. */
#define SH2Z_FIELD_ZONE_RECHARGE 4u /**< Recharge strip: energy refills by recharge per tick. */

/** A surface zone: distances [start, end) and laterals [lat_lo, lat_hi], Q8.
 *  A zone across the line's start is baked as two zones. */
typedef struct {
    int32_t start;             /**< Q8 distance where the zone starts. */
    int32_t end;               /**< Q8 distance where it ends, > start. */
    int32_t lat_lo;            /**< Q8 lowest lateral inside. */
    int32_t lat_hi;            /**< Q8 highest lateral inside. */
    uint32_t kind;             /**< SH2Z_FIELD_ZONE_* */
} sh2z_field_zone_t;

/** The line's surface (params->surface). */
typedef struct sh2z_field_surface {
    const sh2z_field_zone_t *zones; /**< Sorted by start; the first zone that holds an agent applies. */
    uint32_t zone_count;
    int32_t plate_ticks;       /**< Boost ticks a plate gives (at least; a longer boost is kept). */
    int32_t slip_steer;        /**< Q16 share of the steering gain kept on low grip. */
    int32_t rough_keep;        /**< Q16 share of the target speed kept on rough ground. */
    int32_t recharge;          /**< Q8 energy per tick on a recharge strip. */
    uint16_t wrap_mask;        /**< A node is a tube when wrap_mask != 0 and (flags & wrap_mask) == wrap_value. */
    uint16_t wrap_value;
} sh2z_field_surface_t;

/* ---- Contract tag line-field-5 ------------------------------------------ */

/** Behaviour tuning (params->behave). Every mask pair selects the nodes whose
 *  flags satisfy (flags & mask) == value; a zero mask selects none. */
typedef struct sh2z_field_behave {
    int32_t roll_chance;       /**< Q16 chance per decision on a tube that a roll starts (a new roll replaces one under way). */
    int32_t roll_latch;        /**< Q16 share of rolls that hold until an exit stretch. */
    int32_t roll_lead;         /**< Q16 share of speed targeted ahead of the agent round the tube while rolling (signed: + is right). */
    int32_t hold_lead;         /**< Q16 share of speed targeted ahead round a hold-mask tube when not rolling (a slow drift; 0 = keep the angle). */
    int32_t roll_well;         /**< Q16 of the radius: on tubes outside the hold mask a roll settles this far round from the line; 0 = it circles. */
    int16_t roll_min;          /**< Decisions a timed roll lasts at least (>= 1). */
    int16_t roll_span;         /**< Spread of a timed roll's length in decisions (>= 1). */
    uint16_t hold_mask, hold_value;   /**< Tubes where an agent that is not rolling keeps its angle. */
    uint16_t exit_mask, exit_value;   /**< Tubes whose exit stretch ends rolls. */
    int32_t roll_exit_ahead;   /**< Q8 look-ahead for an exit (a non-tube node). */
    int32_t lat_accel;         /**< Q8 per tick^2 change of lateral velocity; 0 = line-field-4 steering. */
    uint16_t air_mask, air_value;     /**< Nodes where agents are in the air. */
    uint16_t open_mask, open_value;   /**< Nodes with an open edge (no wall to stop a flying agent). */
    int32_t air_spread;        /**< Q16 share of speed: the take-off drift is uniform in +-air_spread. */
    int32_t fall_margin;       /**< Q8 beyond the half width before an agent falls. */
    int32_t fall_ticks;        /**< Ticks from leaving the line to retirement (>= 1). */
    int32_t ko_window;         /**< Steps since a contact incident within which a fall is credited; 0 = no credit. */
    int32_t ko_near;           /**< Q8 distance from the focus within which energy can retire an agent; 0 = anywhere. */
    int32_t ko_floor;          /**< Q8 energy an agent beyond ko_near keeps (> 0). */
    uint16_t drop_mask, drop_value;   /**< Drop nodes: launches out of a tube and drops off a tube (zero mask: none). */
    int32_t drop_lat;          /**< Q16 of the radius: round a drop tube beyond this, an agent may drop off. */
    int32_t drop_chance;       /**< Q16 chance per step that an agent beyond drop_lat drops off (0 = never). */
    int32_t launch_chance;     /**< Q16 chance that an agent out of a tube beyond the open edge of a drop node is launched off (65536 = always). */
} sh2z_field_behave_t;

#ifdef __cplusplus
}
#endif
#endif
