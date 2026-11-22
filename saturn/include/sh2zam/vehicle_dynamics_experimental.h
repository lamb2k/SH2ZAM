/* Experimental track-bound vehicle dynamics: one hovering vehicle on a closed
 * road laid along a cardinal spline, stepped one fixed tick at a time in
 * integer arithmetic. Not selected by any default dispatch. Game-neutral: the
 * road, the vehicle's constants and the rule constants all come from the
 * caller.
 *
 * Contract tag vehicle-dynamics-1 (experimental: the numeric definitions may
 * still change until the tag is published). Contract tag vehicle-dynamics-2
 * adds sh2z_vd_step2: the same tick with pipe and cylinder road shapes and the
 * joins that taper them into their neighbours. Contract tag vehicle-dynamics-5
 * models the half-pipe shape in sh2z_vd_step2.
 *
 * Model. Each tick (sh2z_vd_step) runs, in order: the pad, the surface zone
 * under the vehicle, steering, body-frame damping, boost, mines and launch
 * ramps, gravity and the hover spring, leans and attacks, grip, slide turbo,
 * the engine force curve, integration with a speed cap, the closest-point
 * search on the spline, lap and race clock, the road shape (ground clamp,
 * airborne thresholds, walls), the up vector easing toward the road normal
 * and recharge strips. VEHICLE_DYNAMICS.md gives each step.
 *
 * Formats (int32_t unless stated):
 *   Q16   positions, velocities, accelerations, energies, widths: 1.0 = 2^16
 *   Q30   unit vectors, spline parameter t, factors below 2: 1.0 = 2^30
 *   Q12   arc lengths along the line (lap distance, race distance): 1.0 = 2^12
 * World up is +y. A road frame's z axis points to the road's left.
 *
 * Determinism. Every function is integer-only (32x32->64 products rounded to
 * nearest, ties up; quotients from sh2z_divu_div64), allocation-free and
 * reentrant, so a host build and an SH-2 build give identical bits for
 * identical inputs. The SH-2 build uses the on-chip divider through
 * divu_experimental.h; the caller keeps the divider promise stated there.
 *
 * Copyright (c) 2026 SH2ZAM Contributors. MIT License.
 * Inspired by SH4ZAM by Falco Girgis and SH4ZAM Contributors.
 */
#ifndef SH2ZAM_VEHICLE_DYNAMICS_EXPERIMENTAL_H
#define SH2ZAM_VEHICLE_DYNAMICS_EXPERIMENTAL_H

#include "sh2zam.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Contract tag vehicle-dynamics-1 ------------------------------------ */

/** Road cross-section of a span; selects the contact rule. */
enum {
    SH2Z_VD_SHAPE_ROAD = 0,       /**< open road; airborne above rules.air_road */
    SH2Z_VD_SHAPE_WALLED = 1,     /**< walled road; airborne above rules.air_walled */
    SH2Z_VD_SHAPE_PIPE = 2,       /**< inside a tube (vehicle-dynamics-2); road in sh2z_vd_step */
    SH2Z_VD_SHAPE_CYLINDER = 3,   /**< outside a tube (vehicle-dynamics-2); road in sh2z_vd_step */
    SH2Z_VD_SHAPE_HALF_PIPE = 4,  /**< a trough (vehicle-dynamics-5); road in sh2z_vd_step */
    SH2Z_VD_SHAPE_TUNNEL = 5,     /**< tunnel; airborne above rules.air_tunnel */
    SH2Z_VD_SHAPE_GAP = 6,        /**< no road: always airborne */
    SH2Z_VD_SHAPE_EDGELESS = 7    /**< road without walls: leaving its width is falling */
};

/** Surface zone kinds (low four bits of the body flags). */
enum {
    SH2Z_VD_ZONE_NONE = 0,
    SH2Z_VD_ZONE_RECHARGE = 1,    /**< energy refills */
    SH2Z_VD_ZONE_ROUGH = 2,       /**< extra damping unless boosting */
    SH2Z_VD_ZONE_BOOST_PLATE = 3, /**< free boost */
    SH2Z_VD_ZONE_ICE = 4          /**< no grip, no lean force, engine reads total speed */
};

/** Body flags. */
#define SH2Z_VD_ZONE_MASK        UINT32_C(0x0000000F)
#define SH2Z_VD_RECOIL           UINT32_C(0x00002000) /**< wall contact this tick */
#define SH2Z_VD_SPINNING         UINT32_C(0x00004000) /**< out of energy, spinning out */
#define SH2Z_VD_HOVER_SET        UINT32_C(0x00008000) /**< hover height reached (or not yet started) */
#define SH2Z_VD_MINE_HIT         UINT32_C(0x00010000)
#define SH2Z_VD_DAMAGED          UINT32_C(0x00020000) /**< took damage this tick */
#define SH2Z_VD_FALLING          UINT32_C(0x00080000) /**< left the course */
#define SH2Z_VD_CAN_BOOST        UINT32_C(0x00100000) /**< boost unlocked */
#define SH2Z_VD_LAUNCHED         UINT32_C(0x00200000) /**< on a ramp impulse */
#define SH2Z_VD_CONTROLLABLE     UINT32_C(0x00400000) /**< pad input is read */
#define SH2Z_VD_PLATE_BOOST      UINT32_C(0x01000000) /**< current boost came from a plate */
#define SH2Z_VD_FINISHED         UINT32_C(0x02000000)
#define SH2Z_VD_AIRBORNE         UINT32_C(0x04000000)
#define SH2Z_VD_CRASHED          UINT32_C(0x08000000)
#define SH2Z_VD_GRIPPING         UINT32_C(0x20000000) /**< grip state of the grip state machine */
#define SH2Z_VD_ACTIVE           UINT32_C(0x40000000)
#define SH2Z_VD_OFF_ROAD         UINT32_C(0x80000000) /**< beyond the road's contact range */

/** Pad buttons, as the caller maps them. */
#define SH2Z_VD_BTN_ACCEL  UINT16_C(0x8000)
#define SH2Z_VD_BTN_BOOST  UINT16_C(0x4000)
#define SH2Z_VD_BTN_LEAN_L UINT16_C(0x2000) /**< lean / attack to the left */
#define SH2Z_VD_BTN_LEAN_R UINT16_C(0x0010) /**< lean / attack to the right */
#define SH2Z_VD_BTN_BRAKE  UINT16_C(0x0004)
#define SH2Z_VD_BTN_OTHER  UINT16_C(0x0001) /**< any other button (starts the hover) */

/** One spline node and the span from it to the next node (record per node). */
typedef struct {
    int32_t pos[3];          /**< Q16 control point */
    int32_t up[3];           /**< Q30 road up at the node */
    int32_t tension;         /**< Q30 cardinal tension of the span */
    int32_t half_left;       /**< Q16 road half width to the left at the node */
    int32_t half_right;      /**< Q16 road half width to the right */
    int32_t length;          /**< Q12 arc length of the span, > 0 */
    int32_t start;           /**< Q12 arc length from the line start to the node */
    int32_t mark_pos[3][3];  /**< Q16 points at t = 1/4, 1/2, 3/4 */
    int32_t mark_len[3];     /**< Q12 arc length from the node to each mark */
    uint8_t shape;           /**< SH2Z_VD_SHAPE_* */
    uint8_t zones_by_side;   /**< 1: zones bound by side offset, 0: by two edge vectors */
    uint16_t zone_first, zone_count;
    uint16_t ramp_first, ramp_count;
    uint16_t mine_first, mine_count;
    uint16_t reserved;
} sh2z_vd_node_t;

/** Surface zone on a span: t range, then side bounds. By side, lo[0] <= the
 *  offset from the road middle <= hi[0]; by edges, the road displacement d
 *  satisfies (d x lo) . f >= 0 and (hi x d) . f >= 0 with f the road forward. */
typedef struct {
    uint32_t kind;           /**< SH2Z_VD_ZONE_* */
    int32_t t0, t1;          /**< Q30 */
    int32_t lo[3], hi[3];    /**< Q16 */
} sh2z_vd_zone_t;

/** Launch ramp: a box in its own frame (rows x, y, z), from its corner. */
typedef struct {
    int32_t pos[3];          /**< Q16 corner */
    int32_t axis[3][3];      /**< Q30 rows: along x up to dim[2], y up to dim[1], z up to dim[0] */
    int32_t dim[3];          /**< Q16 width (z), height (y), depth (x) */
} sh2z_vd_ramp_t;

typedef struct { int32_t pos[3]; } sh2z_vd_mine_t; /**< Q16 */

typedef struct {
    const sh2z_vd_node_t *nodes;
    const sh2z_vd_zone_t *zones;
    const sh2z_vd_ramp_t *ramps;
    const sh2z_vd_mine_t *mines;
    uint32_t count;          /**< nodes, >= 4 */
    int32_t length;          /**< Q12 line length */
    int32_t laps;            /**< race laps */
} sh2z_vd_track_t;

/** Per-vehicle constants (Q16 unless stated). */
typedef struct {
    int32_t force_low;       /**< engine force at rest */
    int32_t force_slide;     /**< engine force at rest while slide turbo runs */
    int32_t force_high;      /**< engine force from the crossover speed */
    int32_t crossover;       /**< forward speed where the curve reaches force_high */
    int32_t boost_mul;       /**< force multiplier while boosting */
    int32_t plate_mul;       /**< force multiplier on a plate boost */
    int32_t slide_max;       /**< slide turbo level */
    int32_t slide_decay;     /**< slide turbo loss per tick */
    int32_t slew_base;       /**< force slew at rest */
    int32_t turn_free;       /**< Q30 turn rate, free state */
    int32_t turn_grip;       /**< Q30 turn rate, grip state with ACCEL held */
    int32_t top_mix;         /**< share of top_force above the crossover (usually 0) */
    int32_t accel_cutoff;    /**< no engine force at or above this speed */
    int32_t grip_limit;      /**< side speed removed per tick, grip state */
    int32_t slide_limit;     /**< side speed removed per tick, free state */
    int32_t wall_bounce;     /**< side speed reflection factor at a wall */
    int32_t slide_mul;       /**< force gain per unit of slide turbo */
    int32_t bump;            /**< curve bump height factor */
    int32_t slew_gain;       /**< force slew per unit of forward speed */
    int32_t slew_slope;      /**< force slew slope */
    int32_t max_energy;
    int32_t boost_cost;      /**< energy per boost tick */
    int32_t recharge;        /**< energy per tick on a recharge zone */
    int32_t hover_base;      /**< hover height before the hover starts */
    int32_t top_force;       /**< force target mixed in by top_mix */
} sh2z_vd_vehicle_t;

/** Rule constants shared by every vehicle (Q16 unless stated). */
typedef struct {
    int32_t dead_zone;       /**< Q30 stick dead zone */
    int32_t turn_spin;       /**< Q30 turn rate while spinning out */
    int32_t lean_turn_mix;   /**< Q30 grip-to-free turn mix with lean force */
    int32_t grip_turn_mix;   /**< Q30 grip-to-free turn mix without ACCEL */
    int32_t damp_side, damp_up, damp_fwd; /**< Q30 per tick */
    int32_t damp_spin;       /**< Q30 side and forward, spinning out */
    int32_t damp_crashed;    /**< Q30 side and forward, crashed */
    int32_t damp_rough;      /**< Q30 extra on rough zones */
    int32_t damp_brake;      /**< Q30 extra with BRAKE */
    int32_t mine_radius, mine_kick, mine_damage;
    int32_t ramp_gain;       /**< Q30 launch impulse per unit of speed */
    int32_t ramp_base;       /**< launch impulse at rest */
    int32_t ramp_decay;      /**< Q30 per tick */
    int32_t ramp_release;    /**< impulse below which the launch ends on the ground */
    int32_t gravity;
    int32_t hover_spring;    /**< Q30 */
    int32_t hover_top, hover_rise_gain; /**< hover height, Q30 rise share per tick */
    int32_t hover_crashed;
    int32_t double_tap;      /**< ticks */
    int32_t side_force, side_fade, side_cut;    /**< side attack */
    int32_t spin_force, spin_fade, spin_cut;    /**< spin attack */
    int32_t lean_force, lean_fade, lean_cut;    /**< lean */
    int32_t lean_ice;        /**< Q30 lean share on ice */
    int32_t grip_tap;        /**< ticks of free state after an ACCEL press or an attack */
    int32_t tap_window;      /**< ticks an ACCEL press counts as a tap */
    int32_t contact_grip;    /**< ticks of free state after a body hit */
    int32_t slide_speed;     /**< side speed that builds slide turbo */
    int32_t slide_ticks;     /**< ticks to full slide turbo */
    int32_t turn_cost;       /**< Q30 force lost per unit of heading change */
    int32_t lean_cost;       /**< Q30 force lost per unit of lean force */
    int32_t speed_cap;
    int32_t boost_ticks;
    int32_t air_road, air_walled, air_tunnel; /**< heights where contact ends */
    int32_t ground_band;     /**< full contact at or below this height */
    int32_t blend_band;      /**< contact blends out up to this height */
    int32_t edge_widen;      /**< extra half width while off the road */
    int32_t edge_low;        /**< off an edgeless road below this height */
    int32_t edge_damage;
    int32_t tilt_follow;     /**< Q30 up vector easing per tick */
    int32_t tilt_input;      /**< Q30 pitch input gain */
    int32_t wall_damage;     /**< Q30 per unit of side speed */
    int32_t wall_min;        /**< side speed below which a wall does no damage */
    int32_t wall_keep;       /**< Q30 velocity kept at a wall */
    int32_t spin_bounce;     /**< wall_bounce while spinning out */
    int32_t spin_steer;      /**< Q30 steering per unit of the damage that spun out, max 1 */
    int32_t spin_ticks;
    int32_t fall_damp;       /**< Q30 horizontal damping while falling */
    int32_t fall_floor;      /**< retire below this height */
    int32_t world_edge;      /**< off-road beyond |x| or |z| of this is falling */
    int32_t control_tick;    /**< intro timer value that hands control over */
    int32_t clock_tick;      /**< intro timer below which the clock runs */
    int32_t hover_tick;      /**< intro timer below which a button starts the hover */
    int32_t clock_num, clock_den; /**< race clock = ticks * num / den */
} sh2z_vd_rules_t;

/** Pad input for one tick. stick values are signed, full deflection = 63. */
typedef struct {
    uint16_t held, pressed, released;
    int8_t stick_x, stick_y; /**< x positive right; y positive pitches the nose down */
} sh2z_vd_input_t;

/** Body state (Q16 unless stated). */
typedef struct {
    uint32_t flags;
    int32_t pos[3], vel[3], acc[3];
    int32_t body[3][3];      /**< Q30 rows: forward, up, left */
    int32_t road[3][3];      /**< Q30 road frame rows: forward, up, left */
    int32_t gravity_up[3], tilt_up[3], ground_up[3]; /**< Q30 */
    int32_t tilt_in;         /**< Q30 pitch input */
    int32_t height;          /**< above the road surface */
    int32_t half_left, half_right; /**< road half widths here */
    /* closest-point search */
    uint32_t node;
    int32_t t;               /**< Q30 */
    int32_t prop;            /**< Q30 share of the span's arc length */
    int32_t fwd[3];          /**< spline tangent (units per unit t) */
    int32_t fwd_mag;
    int32_t spos[3], disp[3], ground[3];
    /* race */
    int32_t lap_dist, laps_dist, race_dist; /**< Q12 */
    int32_t lap, laps_done;
    int32_t race_ticks, race_time, done_time, lap_times[8];
    /* motion */
    int32_t speed;
    int32_t bvel[3];         /**< body-frame velocity: left, up, forward */
    int32_t energy;
    int32_t accel_force, lean_force, ramp_boost, slide_boost;
    int32_t hover, spring;
    int32_t spin_steer;      /**< steering while not controllable */
    int32_t wall_bounce;
    int32_t boost_timer, spin_timer;
    int32_t attack, attack_dir, attack_angle, attack_rate;
    int32_t tap_l, tap_r;    /**< ticks since the last LEAN_L / LEAN_R press */
    int32_t free_timer, hit_timer, slide_count, tap_timer;
    int32_t intro;           /**< intro timer; counts down to 0 */
} sh2z_vd_body_t;

enum { SH2Z_VD_ATTACK_NONE = 0, SH2Z_VD_ATTACK_SIDE = 1, SH2Z_VD_ATTACK_SPIN = 2 };

/** One tick. Reads input only while the body is controllable. */
void sh2z_vd_step(sh2z_vd_body_t *b, const sh2z_vd_track_t *track, const sh2z_vd_vehicle_t *v,
                  const sh2z_vd_rules_t *r, const sh2z_vd_input_t *in);

/** Closest-point search: moves b's node, t and spline frame to the point of
 *  the spline nearest target (Q16), as the step does. Returns 0, or -1 when
 *  the target is off the line (more than about one span away). */
int sh2z_vd_locate(sh2z_vd_body_t *b, const sh2z_vd_track_t *track, const int32_t target[3],
                   int iterations, int64_t tolerance_q32);

/** Lap distance (Q12) of the body's located point; also sets b->prop. */
int32_t sh2z_vd_lap_distance(sh2z_vd_body_t *b, const sh2z_vd_track_t *track);

/** Offset of the body from the line, along the road's left axis (Q16). */
int32_t sh2z_vd_body_lateral(const sh2z_vd_body_t *b);

/** Line-space view for a line-space field (rank, gaps, contact with agents
 *  that move along the line): distance along the line and lateral offset in
 *  Q8 units with lateral positive to the right, along-line speed and lateral
 *  velocity in Q8 per tick, and completed laps. */
typedef struct {
    int32_t dist, lateral, speed, lat_vel, lap;
} sh2z_vd_line_view_t;

void sh2z_vd_line_view(const sh2z_vd_body_t *b, const sh2z_vd_track_t *track, sh2z_vd_line_view_t *out);

/** Applies a contact kick from a line-space field before the next step: kick_lat
 *  (Q8 per tick, positive right) along the road's right axis and kick_speed (Q8
 *  per tick) along the body's forward axis. */
void sh2z_vd_apply_kick(sh2z_vd_body_t *b, int32_t kick_lat, int32_t kick_speed);

/** Spline point (Q16) and tangent (Q16 units per unit t) of node n at t (Q30). */
void sh2z_vd_spline(const sh2z_vd_track_t *track, uint32_t n, int32_t t, int32_t pos[3], int32_t fwd[3]);

/* ---- Contract tag vehicle-dynamics-2 ------------------------------------ */

/** Joins of a span (one record per node). A tube span tapers into an open
 *  neighbour over a join: below t = prev_end the span joins the previous span,
 *  above t = next_start the next one. Over a join the share
 *  s = prop / scale (or (1 - prop) / scale at the far end) runs from 0 at the
 *  seam to 1 where the tube is whole. A span without joins has prev_end <= 0
 *  and next_start >= 1 (Q30). */
typedef struct {
    int32_t prev_end;        /**< Q30 t where the join with the previous span ends */
    int32_t next_start;      /**< Q30 t where the join with the next span starts */
    int32_t scale;           /**< Q30 share of the span's arc length one join covers, > 0 */
} sh2z_vd_join_t;

/** Rule constants of the tube shapes (Q16 unless stated). */
typedef struct {
    int32_t join_lever;      /**< pipe join: up-vector lever of the road plane against the seam */
    int32_t pipe_rim;        /**< pipe join: off the surface at this height above the line */
    int32_t cyl_reach;       /**< cylinder: off the surface beyond radius + cyl_reach */
    int32_t cyl_lift;        /**< cylinder join: road plane lift per unit of join share */
    int32_t min_dist;        /**< radial direction kept below this distance from the line */
    int32_t min_unit;        /**< Q30 blended up vectors shorter than this are not renormalised */
} sh2z_vd_tube_rules_t;

/** A track with tube shapes: the vehicle-dynamics-1 track (first member), its
 *  joins (one per node, or NULL for none) and the tube rules. */
typedef struct {
    sh2z_vd_track_t base;
    const sh2z_vd_join_t *joins;
    sh2z_vd_tube_rules_t tube;
} sh2z_vd_track2_t;

/** One tick, as sh2z_vd_step, with the pipe and cylinder shapes:
 *  - SH2Z_VD_SHAPE_PIPE: the ground is the inside of the circle of radius
 *    half_left round the line. Height is the radius less the distance from
 *    the line; the up vector points to the line; gravity and the hover act
 *    along it. There are no walls: past the radius the body is put back on
 *    it. Over a join, the road plane lifted by the tapered radius takes over
 *    toward the seam, and the up vectors blend toward the road's.
 *  - SH2Z_VD_SHAPE_CYLINDER: the ground is the outside of that circle, the
 *    radius tapering to the line over a join. Height is the distance less the
 *    radius; the up vector points away from the line. Beyond radius plus
 *    tube.cyl_reach the body has left the surface (it falls).
 *  - SH2Z_VD_SHAPE_HALF_PIPE (contract tag vehicle-dynamics-5): the pipe's
 *    ground with the top open. The body leaves the surface when its height
 *    over the line, less tube.pipe_rim, plus sin(22.5 degrees) of the radius
 *    (tapered by the share over a join) reaches zero, on the whole span; the
 *    joins are the pipe's, and gravity acts along the road's up everywhere.
 *  Road shapes give the same bits as sh2z_vd_step. */
void sh2z_vd_step2(sh2z_vd_body_t *b, const sh2z_vd_track2_t *track, const sh2z_vd_vehicle_t *v,
                   const sh2z_vd_rules_t *r, const sh2z_vd_input_t *in);

/* ---- Contract tag vehicle-dynamics-3 ------------------------------------ */

/** Contact from other bodies for one tick (for example from
 *  body_contact_experimental.h). */
typedef struct {
    int32_t push[3];         /**< Q16 acceleration, integrated with the tick's own forces */
    int32_t damage;          /**< Q16 energy lost; empties into the spin-out as wall damage does */
    int32_t regain;          /**< Q16 energy regained first, up to the maximum (not while spinning out or crashed) */
    int32_t hit;             /**< nonzero: no grip for rules.contact_grip ticks, from this tick */
} sh2z_vd_contact_t;

/** sh2z_vd_step with contact: after the per-tick resets and before the pad
 *  (so damage that starts a spin-out takes the pad away this tick), the
 *  regain, the damage and the grip-off timer; the push starts the tick's
 *  acceleration. With a zero contact it is sh2z_vd_step, bit for bit. One
 *  call at a time per CPU (the contact in progress is held in a file
 *  static); see BODY_CONTACT.md. */
void sh2z_vd_step_contact(sh2z_vd_body_t *b, const sh2z_vd_track_t *track, const sh2z_vd_vehicle_t *v,
                          const sh2z_vd_rules_t *r, const sh2z_vd_input_t *in, const sh2z_vd_contact_t *c);

/** sh2z_vd_step2 with contact (the tube shapes of vehicle-dynamics-2), the
 *  contact placed as in sh2z_vd_step_contact. With a zero contact it is
 *  sh2z_vd_step2, bit for bit. One call at a time per CPU. */
void sh2z_vd_step2_contact(sh2z_vd_body_t *b, const sh2z_vd_track2_t *track, const sh2z_vd_vehicle_t *v,
                           const sh2z_vd_rules_t *r, const sh2z_vd_input_t *in, const sh2z_vd_contact_t *c);

#ifdef __cplusplus
}
#endif

#endif
