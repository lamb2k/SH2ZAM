# Effect pool (experimental, contract tag `effect-pool-1`)

[`effect_pool_experimental.h`](include/sh2zam/effect_pool_experimental.h) keeps short-lived
world particles (sparks, puffs, smoke, debris) in fixed rings, steps them once per simulation
tick, projects them as camera-facing billboards and writes them as VDP1 commands. It is meant for
the effects of a racing or action game drawn with VDP1 sprites over a 3D scene. What an effect
looks like, when it is spawned and which art it draws with stay with the caller.

Nothing selects it by default. The tag is experimental: its numeric definitions may change until
it is published.

## Rings

A pool is a ring of `capacity` slots (a power of two, at most 256) whose particles share one
lifetime, one scale growth, one velocity damping and one acceleration. Because every particle of a
pool lives equally long, particles end oldest first and the live ones are always the `count`
newest slots, so a step visits only live particles and a spawn is one store.

| Call | What |
| --- | --- |
| `sh2z_ep_spawn` | Takes the slot after the newest; refused (`-1`) when the ring is full. The particle starts at `pos + vel`, timer 0. |
| `sh2z_ep_step` | Per particle: `timer = (timer + 1) mod life`; at 0 it ends; otherwise `scale *= growth`, `vel = vel * damp + accel`, `pos += vel`. |
| `sh2z_ep_at(p, k)` | The `k`-th newest live particle. |

## Drawing

`sh2z_ep_view_t` holds the per-frame camera with the same terms as the rotation sprites'
world-pose path (rows Q1.14, eye Q16, focal Q16, centre, near depth) plus a guard rectangle.
`sh2z_ep_project` gives a Q16 screen position and Q14 depth with the on-chip divider, and
`sh2z_ep_pixels` the screen size of a world length there.

For billboards a whole-pixel position is enough. `sh2z_ep_spot` takes one division instead of
three: it divides `focal_x` by the depth once and reaches the vertical scale through
`sh2z_ep_ratio` (`focal_y / focal_x`, once per view). The position is within one pixel of
`sh2z_ep_project`'s and `sh2z_ep_spot_pixels` within one pixel of `sh2z_ep_pixels`.
`sh2z_ep_depth` gives the depth alone, with no division, for distance tests before projecting.

| Call | VDP1 command |
| --- | --- |
| `sh2z_ep_emit_diamond` | A distorted sprite whose texture corners sit at the centre plus (r,0), (0,r), (-r,0), (0,-r): a texture turned 45 degrees, as many N64 effect quads are drawn. |
| `sh2z_ep_emit_quad` | A distorted sprite through four given corners (stretched trails, a triangle with D = C). |
| `sh2z_ep_emit_shape` | An untextured polygon or a line, with an optional Gouraud table. |

Commands are the 16 words of a command table entry, jump bits 0; corners outside -1024..1023,
misaligned addresses and bad sizes are refused without writing. `sh2z_ep_sort` orders up to 256
items far to near (stable), and `sh2z_ep_spin_rows` turns a Q1.14 basis' forward and side rows
about its up row by a 4096-step angle (a spinning sprite picks its cell from the turned basis).

## Using it

```c
static sh2z_ep_particle_t sparks[32];
static sh2z_ep_pool_t pool;
sh2z_ep_init(&pool, sparks, 32u, 16u, growth_q16, 65536);   /* life 16 ticks */

/* each tick */
sh2z_ep_spawn(&pool, pos, vel, scale, owner, seed);
sh2z_ep_step(&pool);

/* each frame */
for(uint32_t k = 0; k < pool.count; k++) {
    const sh2z_ep_particle_t *e = sh2z_ep_at(&pool, k);
    sh2z_ep_spot_t s;                       /* ratio = sh2z_ep_ratio(&view), once a frame */
    if(!sh2z_ep_spot(&view, ratio, e->pos, &s)) continue;
    const int32_t r = sh2z_ep_spot_pixels(&s, e->scale);
    sh2z_ep_emit_diamond(cmd, s.x, s.y, r, frame_vram, 32u, 32u, clut_vram, pmod);
}
```

## Determinism and cost

Integer only, allocation-free, no libc. The step is two Q16 products per axis and one for the
scale; projection is three 32x16 dot products and two divisions on the on-chip divider (a spot: one,
pixel size included). The SH-2
object is about 3 KB (`-O2`) and needs `sh2z_divu_div64` and the trig12 table. See the
[validation record](validation/effect-pool-1.md).
