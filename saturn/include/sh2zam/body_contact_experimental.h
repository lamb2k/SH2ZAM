/* Experimental body contact: bodies that touch as spheres of one reach,
 * share the impact energy by weight, push each other apart while they close,
 * and remember which credited body last started a contact with them. An
 * "attacking" body pushes its victim harder, takes no damage from the hit and
 * may hurt more. Not selected by any default dispatch. Game-neutral: what a
 * body is, what damage does and who is credited are the caller's.
 *
 * Contract tag body-contact-1 (experimental: the numeric definitions may
 * still change until the tag is published). body-contact-2 adds the sorted
 * sweep and the pair-list entry points below, and makes the field walk test
 * the pair's position box in 32 bits first; no result of body-contact-1
 * changes.
 *
 * The rule for one pair at separation d = |pB - pA| <= reach, relative
 * velocity dv = vB - vA, unit separation n = (pB - pA) / d, weights wA, wB
 * (a WEAK body counts weak_share of its strength) and shares sA = wA / (wA +
 * wB), sB = 1 - sA:
 *   - first tick in reach (the pair was not in reach the tick before): the
 *     impact I = impact_gain * |n . dv|; with no attacker or two, A takes
 *     sB * I and B takes sA * I; with one attacker only the other takes I,
 *     times heavy_gain when the attacker is HEAVY. A CREDIT body becomes the
 *     other's toucher (its since counter restarts at 0);
 *   - every tick in reach while closing ((pB - pA) . dv <= 0): the push P =
 *     (reach - d) n, at least attack_min long when either attacks; A's
 *     acceleration gets -sB P and B's +sA P, except that an attacker against
 *     a non-attacker gets nothing and its victim gets attack_push P and the
 *     hit mark.
 * Below d = near the separation is degenerate and n is the unit of dv; the
 * push is then -reach n.
 *
 * Formats (int32_t unless stated):
 *   Q16   positions, velocities, accelerations, damage, lengths: 1.0 = 2^16
 *   Q30   unit vectors (internal)
 * Strength is an unsigned weight of any scale up to 2^30; only ratios count.
 *
 * Error bound (against the exact real-number rule on the same inputs): each
 * damage and each push component within 2^-8 (256 Q16 LSB) for separations
 * from near to reach, speeds below 2^10 per tick and shares above 2^-12.
 * The decisions (in reach, first, closing, attack cases) are exact on the
 * integer inputs.
 *
 * Determinism. Integer only (64-bit products, rounded to nearest, quotients
 * from sh2z_divu_div64, square roots from sh2z_isqrt64_q12), allocation-free
 * and reentrant on distinct fields, so a host build and an SH-2 build give
 * identical bits. The SH-2 build uses the on-chip divider; the caller keeps
 * the divider promise of divu_experimental.h.
 *
 * Copyright (c) 2026 SH2ZAM Contributors. MIT License.
 * Inspired by SH4ZAM by Falco Girgis and SH4ZAM Contributors.
 */
#ifndef SH2ZAM_BODY_CONTACT_EXPERIMENTAL_H
#define SH2ZAM_BODY_CONTACT_EXPERIMENTAL_H

#include "sh2zam.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Contract tag body-contact-1 ---------------------------------------- */

/** Body flags (inputs). */
#define SH2Z_BC_ACTIVE UINT32_C(0x01) /**< takes part in contact */
#define SH2Z_BC_ATTACK UINT32_C(0x02) /**< attacking this tick */
#define SH2Z_BC_WEAK   UINT32_C(0x04) /**< counts rules.weak_share of its strength */
#define SH2Z_BC_HEAVY  UINT32_C(0x08) /**< its attacks hurt rules.heavy_gain times more */
#define SH2Z_BC_CREDIT UINT32_C(0x10) /**< a contact it starts is recorded on the other body */

/** Pair events returned by sh2z_bc_pair. */
#define SH2Z_BC_EV_TOUCH UINT32_C(0x01) /**< in reach this tick */
#define SH2Z_BC_EV_FIRST UINT32_C(0x02) /**< first tick in reach: impact charged */
#define SH2Z_BC_EV_PUSH  UINT32_C(0x04) /**< closing: pushed */
#define SH2Z_BC_EV_HIT   UINT32_C(0x08) /**< an attack pushed its victim */

typedef struct {
    int32_t pos[3];          /**< Q16 */
    int32_t vel[3];          /**< Q16 per tick */
    uint32_t strength;       /**< weight, 1 .. 2^30 */
    uint32_t flags;          /**< SH2Z_BC_* */
    int32_t push[3];         /**< out: Q16 acceleration from contacts this tick */
    int32_t damage;          /**< out: Q16 damage from contacts this tick */
    uint8_t hit;             /**< out: 1 when an attack pushed it this tick */
    uint8_t first;           /**< out: contacts that started this tick */
    int16_t toucher;         /**< index of the last CREDIT body to start a contact with it, -1 none */
    uint32_t since;          /**< ticks since that contact (sh2z_bc_field_begin ages it; saturates) */
} sh2z_bc_body_t;

typedef struct {
    int32_t reach_long;      /**< along-line gap, in the caller's along units, beyond which no pair is tested */
    int32_t reach;           /**< Q16 contact distance, > near, at most 2^24 */
    int32_t near;            /**< Q16 separation below which n comes from dv */
    int32_t impact_gain;     /**< Q16 */
    int32_t attack_push;     /**< Q16 victim's push share */
    int32_t attack_min;      /**< Q16 shortest push with an attacker in the pair */
    int32_t weak_share;      /**< Q16 share of strength a WEAK body counts */
    int32_t heavy_gain;      /**< Q16 damage factor of a HEAVY attacker's hit */
    uint32_t credit_window;  /**< ticks a toucher keeps its credit */
} sh2z_bc_rules_t;

/** One pair. touching_before is whether the pair was in reach on the previous
 *  tick; ia and ib are the bodies' indices (recorded as toucher). Adds to
 *  both bodies' outputs. Returns SH2Z_BC_EV_* bits (0: out of reach). */
uint32_t sh2z_bc_pair(sh2z_bc_body_t *a, sh2z_bc_body_t *b, uint32_t ia, uint32_t ib, bool touching_before,
                      const sh2z_bc_rules_t *r);

/** A field of bodies with their pair history. The caller owns both arrays:
 *  body[count] and last[count * (count - 1) / 2] (zeroed before the first
 *  tick). At most 255 bodies. */
typedef struct {
    sh2z_bc_body_t *body;
    uint32_t *last;          /**< per pair (i < j at j (j - 1) / 2 + i): tick last in reach, 0 never */
    uint32_t count;
    uint32_t tick;           /**< ticks begun, from 0 */
    uint32_t contacts;       /**< contacts started so far */
    uint32_t tested;         /**< pairs tested in the last resolve (cost accounting) */
} sh2z_bc_field_t;

/** Binds the arrays, clears the history and every body's outputs and toucher. */
void sh2z_bc_field_init(sh2z_bc_field_t *f, sh2z_bc_body_t *body, uint32_t *last, uint32_t count);

/** Starts a tick: clears every body's outputs and ages every since counter. */
void sh2z_bc_field_begin(sh2z_bc_field_t *f);

/** Resolves every pair of ACTIVE bodies whose along-line gap is at most
 *  rules.reach_long. order lists the bodies by along[] descending (largest
 *  first) on a closed line of the given length (along units, more than four
 *  times reach_long); gaps wrap. Each pair is tested once, from the body
 *  ahead. A stale order only costs missed pairs, never a pair twice. Returns
 *  the pairs in reach. */
uint32_t sh2z_bc_field_resolve(sh2z_bc_field_t *f, const uint8_t *order, const int32_t *along, int32_t length,
                               const sh2z_bc_rules_t *r);

/* ---- Contract tag body-contact-2 ---------------------------------------- */

/** The pairs of a sorted sweep within a window, in the order sh2z_bc_field_resolve visits them. order lists the bodies by along[]
 *  descending on a closed line of the given length (as sh2z_bc_field_resolve takes it); live[i] is nonzero for a body that takes
 *  part; across[i] is a second coordinate in the along units. A pair is listed when both bodies are live, their along gap is at most
 *  reach_long (each pair once, from the body ahead, ties before the wrap) and their across difference is at most reach_across.
 *  pa[] and pb[] receive the bodies of each pair (room for count * (count - 1) / 2 each), marked[i] (count bytes) is 1 for a body
 *  in a listed pair, else 0. Returns the number of pairs. Pairs left out because of reach_across are the caller's claim: bodies in
 *  reach are within reach_across across. */
uint32_t sh2z_bc_sweep(const uint8_t *order, const int32_t *along, const int32_t *across, const uint8_t *live, uint32_t count,
                       int32_t length, int32_t reach_long, int32_t reach_across, uint8_t *pa, uint8_t *pb, uint8_t *marked);

/** The pairs of a list whose two bodies are ACTIVE and within the reach on every axis (the first test of sh2z_bc_pair, done in
 *  32 bits), in list order, into qa[] and qb[] (room for as many as the list). Sets field->tested to the pairs with both bodies
 *  ACTIVE. Returns the number in range. A caller that computes a body's velocity lazily does it for the bodies of this list. */
uint32_t sh2z_bc_range(sh2z_bc_field_t *f, const uint8_t *pa, const uint8_t *pb, uint32_t pairs, const sh2z_bc_rules_t *r,
                       uint8_t *qa, uint8_t *qb);

/** sh2z_bc_pair for each pair of a list, with the field's history as sh2z_bc_field_resolve keeps it (a pair in reach this tick is
 *  recorded, a first contact counts). For a list from sh2z_bc_range the result equals sh2z_bc_field_resolve's on the same bodies.
 *  Returns the pairs in reach. */
uint32_t sh2z_bc_field_resolve_list(sh2z_bc_field_t *f, const uint8_t *pa, const uint8_t *pb, uint32_t pairs, const sh2z_bc_rules_t *r);

/** The credited body for a victim that has just been put out by damage: its
 *  toucher when the contact is at most credit_window ticks old, else -1. */
int32_t sh2z_bc_credit(const sh2z_bc_field_t *f, uint32_t victim, const sh2z_bc_rules_t *r);

#ifdef __cplusplus
}
#endif

#endif
