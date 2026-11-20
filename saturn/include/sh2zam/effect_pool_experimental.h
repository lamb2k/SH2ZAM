/* Experimental effect pool: fixed rings of short-lived world particles
 * (sparks, puffs, smoke, debris) stepped once per simulation tick, projected
 * as camera-facing billboards and written as VDP1 commands. Not selected by
 * any default dispatch. Game-neutral: what a particle looks like, when it is
 * spawned and which art it draws with are the caller's.
 *
 * Contract tag effect-pool-1 (experimental: the numeric definitions may still
 * change until the tag is published).
 *
 * A pool is a ring of capacity slots (a power of two) whose particles share
 * one lifetime, one scale growth, one velocity damping and one acceleration.
 * Spawning takes the slot after the newest and is refused when the ring is
 * full; because every particle of a pool lives equally long, particles end
 * oldest first and the live ones are always the count newest slots. One step:
 *   timer = (timer + 1) mod life; a particle whose timer returns to 0 ends;
 *   otherwise scale = scale * growth, vel = vel * damp + accel, pos += vel.
 * A spawned particle starts at timer 0 and position pos + vel.
 *
 * Projection (sh2z_ep_project) uses the same camera terms as the rotation
 * sprites' world-pose path: with d = pos - eye,
 *     t[i] = floor(sum_k r[i][k] * d[k] / 2^16)                (Q18.14)
 *     X = center_x * 2^16 + Q(focal_x * t.x, t.z),  Y = center_y * 2^16 - Q(focal_y * t.y, t.z)
 *     a world length L (Q16) is Q(focal_y * floor(L / 4), t.z) / 2^16 pixels
 * where Q(n, d) = sh2z_divu_div64(n, d) (truncating, saturating).
 *
 * VDP1 commands are the 16 big-endian words of a command table entry:
 * CMDCTRL, CMDLINK, CMDPMOD, CMDCOLR, CMDSRCA, CMDSIZE, XA, YA, XB, YB, XC,
 * YC, XD, YD, CMDGRDA, (unused). Jump and zoom bits are 0 (next command).
 *
 * Formats (int32_t unless stated):
 *   Q16    positions, velocities, scales, accelerations: 1.0 = 2^16
 *   Q16    growth and damping factors (65536 = 1.0)
 *   Q1.14  camera rows and spin rows (int16_t)
 *   angle12  one turn = 4096 (trig12_experimental.h)
 *
 * Determinism. Integer only, allocation-free, no libc; host and SH-2 builds
 * give identical bits. Projection uses the on-chip divider; the caller keeps
 * the divider promise of divu_experimental.h.
 *
 * Copyright (c) 2026 SH2ZAM Contributors. MIT License.
 * Inspired by SH4ZAM by Falco Girgis and SH4ZAM Contributors.
 */
#ifndef SH2ZAM_EFFECT_POOL_EXPERIMENTAL_H
#define SH2ZAM_EFFECT_POOL_EXPERIMENTAL_H

#include "sh2zam.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Contract tag effect-pool-1 ----------------------------------------- */

/** One particle. user and seed are the caller's (art, owner, colour). */
typedef struct {
    int32_t pos[3];          /**< Q16 */
    int32_t vel[3];          /**< Q16 per tick */
    int32_t scale;           /**< Q16 world half size */
    uint16_t timer;          /**< ticks lived, 0 at spawn and when ended */
    uint8_t user;            /**< the caller's */
    uint8_t seed;            /**< the caller's */
} sh2z_ep_particle_t;

/** A ring of particles with shared rules. */
typedef struct {
    sh2z_ep_particle_t *items;
    uint32_t mask;           /**< capacity - 1 (capacity a power of two, 1 .. 256) */
    uint32_t head;           /**< slot the next spawn takes */
    uint32_t count;          /**< live particles: the count slots before head */
    uint32_t life;           /**< ticks a particle lives, 2 .. 65535 */
    int32_t growth;          /**< Q16 scale factor per tick */
    int32_t damp;            /**< Q16 velocity factor per tick */
    int32_t accel[3];        /**< Q16 added to the velocity per tick */
} sh2z_ep_pool_t;

/** Binds items[capacity] (capacity a power of two, 1 .. 256) and clears every
 *  slot. damp 65536 and accel 0 give the original's plain drift. Returns
 *  false for a bad capacity or life, or null pointers. */
bool sh2z_ep_init(sh2z_ep_pool_t *p, sh2z_ep_particle_t *items, uint32_t capacity, uint32_t life,
                  int32_t growth, int32_t damp);

/** Spawns at pos + vel with velocity vel and half size scale. Returns the
 *  slot, or -1 when the ring is full. */
int32_t sh2z_ep_spawn(sh2z_ep_pool_t *p, const int32_t pos[3], const int32_t vel[3], int32_t scale,
                      uint8_t user, uint8_t seed);

/** One tick for every live particle (see the rule above). */
void sh2z_ep_step(sh2z_ep_pool_t *p);

/** The k-th newest live particle, k < count. */
static inline sh2z_ep_particle_t *sh2z_ep_at(const sh2z_ep_pool_t *p, uint32_t k) {
    return &p->items[(p->head - 1u - k) & p->mask];
}

/** Drops every particle. */
void sh2z_ep_clear(sh2z_ep_pool_t *p);

/** Per-frame camera (the rotation sprites' world-pose terms).
 *  r        world-to-view rows, Q1.14: [0] screen right, [1] screen up, [2] forward.
 *  eye      Q16 world position.
 *  focal_x, focal_y  Q16 pixels; center_x, center_y whole pixels.
 *  near_z   Q14: points nearer are not projected.
 *  guard    whole pixels: centres farther than this outside the screen
 *           rectangle 0..width-1 x 0..height-1 are not projected. */
typedef struct {
    int16_t r[3][3];
    int32_t eye[3];
    int32_t focal_x, focal_y;
    int16_t center_x, center_y;
    int32_t near_z;
    int16_t width, height, guard;
} sh2z_ep_view_t;

/** A projected point: Q16 screen position and Q14 depth. */
typedef struct {
    int32_t x, y;            /**< Q16 pixels */
    int32_t depth;           /**< Q14 */
} sh2z_ep_point_t;

/** Projects pos. Returns false for a depth at or below near_z, a centre
 *  outside the guard rectangle or null pointers (out unchanged). */
bool sh2z_ep_project(const sh2z_ep_view_t *v, const int32_t pos[3], sh2z_ep_point_t *out);

/** Q16 pixels of a world length (Q16, >= 0) at a projected depth (> 0). */
int32_t sh2z_ep_pixels(const sh2z_ep_view_t *v, int32_t length, int32_t depth);

/** A spot: a whole-pixel screen position, the Q14 depth and the point's
 *  vertical scale (focal_y * 2^16 / depth, through the view's ratio), from
 *  one divide (sh2z_ep_spot). */
typedef struct {
    int32_t x, y;            /**< whole pixels */
    int32_t depth;           /**< Q14 */
    int32_t scale;           /**< focal_y (Q16) * 2^16 / depth, approximately */
} sh2z_ep_spot_t;

/** focal_y / focal_x in Q16, once per view, for sh2z_ep_spot. */
int32_t sh2z_ep_ratio(const sh2z_ep_view_t *v);

/** The view depth of pos (Q14, the projection's t.z) with no divide. */
int32_t sh2z_ep_depth(const sh2z_ep_view_t *v, const int32_t pos[3]);

/** One-divide projection for billboards. With the projection's t (Q14):
 *     s = Q(focal_x * 2^16, t.z),  scale = floor(s * ratio / 2^16)
 *     x = center_x + floor(t.x * s / 2^32),  y = center_y + floor(-t.y * scale / 2^32)
 *  The position is within one pixel of sh2z_ep_project's (its Q16 result
 *  rounded down to whole pixels); the near and guard rules are the same, the
 *  guard applied to the whole-pixel position. Returns false (out unchanged)
 *  as sh2z_ep_project does. */
bool sh2z_ep_spot(const sh2z_ep_view_t *v, int32_t ratio, const int32_t pos[3], sh2z_ep_spot_t *out);

/** Whole pixels of a world half size (Q16, >= 0) at a spot:
 *  floor(floor(length / 4) * scale / 2^32); within one pixel of
 *  sh2z_ep_pixels(v, length, depth) >> 16. */
int32_t sh2z_ep_spot_pixels(const sh2z_ep_spot_t *s, int32_t length);

/** VDP1 command codes used here (CMDCTRL bits 3..0). */
#define SH2Z_EP_DISTORTED 2u
#define SH2Z_EP_POLYGON   4u
#define SH2Z_EP_LINE      6u

/** A camera-facing diamond: the texture's (0,0), (w,0), (w,h), (0,h) corners
 *  at the centre plus (r,0), (0,r), (-r,0), (0,-r) (screen y down), as a
 *  distorted sprite. x, y, r whole pixels (r >= 1). vram, colr: texture and
 *  colour table byte addresses in VDP1 VRAM (multiples of 8; 32 for colr).
 *  w a multiple of 8 (8 .. 504), h 1 .. 255. Returns false, writing nothing,
 *  for corners outside -1024 .. 1023, bad sizes or addresses. */
bool sh2z_ep_emit_diamond(uint16_t cmd[16], int32_t x, int32_t y, int32_t r, uint32_t vram, uint32_t w,
                          uint32_t h, uint32_t colr, uint16_t pmod);

/** A distorted sprite through four corners {x, y} (A, B, C, D in texture
 *  order), whole pixels. Same checks as sh2z_ep_emit_diamond. */
bool sh2z_ep_emit_quad(uint16_t cmd[16], const int16_t xy[4][2], uint32_t vram, uint32_t w, uint32_t h,
                       uint32_t colr, uint16_t pmod);

/** An untextured polygon (code SH2Z_EP_POLYGON) or a line (SH2Z_EP_LINE,
 *  corners A and B only) in an RGB colour, with an optional Gouraud table
 *  byte address (multiple of 8; 0 none). Corners outside -1024 .. 1023 are
 *  refused. */
bool sh2z_ep_emit_shape(uint16_t cmd[16], uint32_t code, const int16_t xy[4][2], uint16_t colour,
                        uint16_t pmod, uint32_t gouraud);

/** Turns the forward and side rows of a Q1.14 basis about its up row by
 *  angle12 (one turn 4096): f' = f cos + s sin, s' = s cos - f sin, each
 *  product rounded to nearest. */
void sh2z_ep_spin_rows(int16_t fwd[3], int16_t side[3], uint32_t angle12);

/** Orders n <= 256 items far to near: order[] receives indices sorted by
 *  depth descending; equal depths keep index order (stable). */
void sh2z_ep_sort(uint8_t *order, const int32_t *depth, uint32_t n);

#ifdef __cplusplus
}
#endif

#endif
