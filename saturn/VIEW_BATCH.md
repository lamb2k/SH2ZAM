# View batch: transform, project and classify (experimental)

`sh2zam/view_experimental.h` turns 16-bit model positions into view-space
coordinates, integer screen positions and clip outcodes in one call. It is
built for SH-2 renderers that would otherwise transform, divide and test six
planes per vertex in 64-bit arithmetic. Contract tag: `view-contract-1`.

## Status

| Part | State |
| --- | --- |
| Header, numeric definition, portable C (`sh2z_view_batch`) | Landed, `view-contract-1` |
| SH-2 path with the hardware divider (`sh2z_view_batch_divu`) | Landed, `view-sh2-1` |
| SH-2 model timing (MODELLED tier) | 134.7 cycles per vertex at 512 vertices |
| View-space projection, edge stepper, near clip | Landed, `view-edges-1` (see below) |
| Guard-band polygon clip (near, far, sides; attributes) | Landed, `view-guard-1` (see below) |
| Console or emulator timing | Unrun; the [calibration image](benchmarks/yaul/calibration/README.md) times the batch on a console |

`sh2z_view_batch` is the definition of correct and never touches the hardware
divider (its contract says so). On SH-2 it calls the 64-bit division helper
twice per front vertex. `sh2z_view_batch_divu` takes the same arguments and
writes the same bytes, and on SH-2 uses the CPU's own divider; see
[DIVU.md](DIVU.md) for the promise it needs about interrupt handlers.

## Formats

| Field | Format | Notes |
| --- | --- | --- |
| Position | `int16` model units | Any stride from 6 bytes, 2-byte aligned |
| Rotation `r` | Q1.14 | 16384 is one |
| Translation `t`, view `x y z`, `near_z`, `far_z` | Q18.14 model units | |
| `focal_x`, `focal_y` | Q16.16 pixels | Both positive |
| `sx`, `sy` | `int16` pixels | Saturated |

The transform is exact in 32 bits. `sh2z_view_init` accepts a view only when
`32768 * sum|r[i][j]| + |t[i]| <= INT32_MAX` for every row, which proves no
int16 position can wrap the sum. Any Q1.14 rotation with an eye inside the int16
world passes.

Compute `t` with `sh2z_view_place` from the already quantized rotation. Then a
vertex's view position is the quantized rotation applied to `p - eye`, and the
rotation error scales with distance from the camera, not from the world origin.

## Definition

See the header for the full text. In short, per vertex: the view position; then
`NEAR` if `z < near_z` (screen 0,0, no other bits); otherwise both quotients
truncated toward zero and saturated to 32 bits, floor-shifted to pixels around
the center (`y` up in view, down on screen), and `FAR`, `LEFT`, `RIGHT`, `TOP`,
`BOTTOM` from the inclusive clip rectangle. A `NEAR` vertex has no side bits,
so ANDing a polygon's outcodes is a conservative trivial reject.

## Evidence (HOST)

`saturn/tests/test_view.c`, built under GCC and Clang Release and Clang
ASan/UBSan:

- 1,048,576 vertices over 512 random poses equal an independent 128-bit oracle
  byte for byte, including 592,340 `NEAR` and 1,773 saturated results.
- Edge cases: exactly on `near_z`, behind the camera, one unit deep, each clip
  edge one pixel either side, `far_z`, quotient and screen saturation on both
  signs, truncation of negative quotients, the bound met exactly and exceeded by
  one, every argument rule, failure without writes, and record strides of 10.
- Error study against double precision with the unquantized rotation and eye,
  focal lengths 64 to 1,024 px, vertices aimed inside the clip rectangle:

| Depth (model units) | Vertices | Off by one pixel | Off-by-one distance from a pixel edge |
| --- | ---: | ---: | ---: |
| 1 to 16 | 115,367 | 2,471 | at most 0.104 px |
| 16 to 256 | 239,507 | 4,588 | at most 0.131 px |
| 256 to 4,096 | 255,758 | 4,856 | at most 0.104 px |
| 4,096 to 65,536 | 142,518 | 2,685 | at most 0.098 px |

  Every difference is a value lying within about a tenth of a pixel of a pixel
  boundary, independent of depth: that is the Q1.14 rotation's angular error at
  1,024 px focal length. The test fails above a quarter pixel.

- Negative controls, each required to be detected: truncation replaced by floor
  (5 mismatches), LEFT and RIGHT swapped (397,166), `NEAR` made inclusive (2).

The same sweep and every argument-rule case also run through
`sh2z_view_batch_divu`, which must equal `sh2z_view_batch` byte for byte (on a
host both take the portable route).

## SH-2 path (`sh2z_view_batch_divu`)

Hand-scheduled SH-2 assembly in `src/sh2zam_view.c`, selected only on SH-2
builds. Per vertex it costs nine `MULS.W`, two `DMULS.L` and two hardware
divisions, and overlaps the CPU work with the divider:

    transform k, store x y z       (y division of vertex k-1 still running)
    collect y quotient of k-1
    near test; start x division of k
    finish k-1: sy, top/bottom bits, store its last two words
    y numerator of k; collect x quotient; start y division of k
    sx of k, far/left/right bits

A division goes to the divider only when a software test proves its quotient
fits strictly (`|n| < z * 2^31`); otherwise the defined result is `INT32_MAX`
or `INT32_MIN` by sign. Only `q >> 16` is ever used, and
`INT32_MIN >> 16 == (INT32_MIN + 1) >> 16`, so the kernel replaces such a
division by `(±(2^31 - 1) * z) / z`, which fits and gives the same pixel. The
main path therefore has a single shape and never needs a software division.
Strides above 32,773 bytes fall back to the portable loop.

### Evidence (MODELLED: SH-2 instruction model, not a console)

The benchmark ELF is executed by the pinned Yabause SH-2 interpreter with a
harness-side divider model (39-cycle latency; a read before completion holds
the CPU — an assumption, see [DIVU.md](DIVU.md)). Capture:
`benchmarks/view-model/`, run in CI by `sh2-cross.yml`.

- **Exactness:** 512 random cameras x 2,048 positions (1,048,576 vertices,
  589,846 `NEAR`, 6,796 saturated quotients) plus 65,536 vertices read with a
  12-byte stride. Both `sh2z_view_batch` and `sh2z_view_batch_divu`, run on
  the SH-2, equal an independent 128-bit oracle in the harness byte for byte,
  reserved bytes included.
- **Timing:** 64 cameras x 512 positions inside the view volume, so every
  vertex costs two divisions. Median modelled cycles per vertex, including
  argument checks and camera setup:

| Path | Cycles per vertex |
| --- | ---: |
| `sh2z_view_batch` (libgcc 64-bit division) | 681.0 |
| `sh2z_view_batch_divu`, instructions only (no divider latency) | 130.2 |
| `sh2z_view_batch_divu`, divider latency with overlap (stall model) | **134.7** |
| `sh2z_view_batch_divu`, plus a full 39 cycles per division (no overlap credit) | 208.2 |

  126 instructions per vertex. The plan's target was 160 (stop above 220).

- **Designs tried**, same capture: straight C with the divider, 207.4 (stall
  model); C software-pipelined across vertices, 199.0 (GCC spilled the
  camera to the stack and used 32-bit multiplies); the assembly above, 134.7.

### Not established

- No console or emulator run: the cycle counts assume flat memory, no cache
  misses, no multiplier latency and the divider assumptions in DIVU.md.
- The 39-cycle latency and the read-holds behaviour come from the hardware
  manual's description and are modelled, not measured.

## Edges and near clip (`view-edges-1`)

Three more entry points for renderers that cut and clip in view space, each
with a `_divu` form that gives the same bytes and uses the hardware divider on
SH-2. Full definitions are in the header.

- `sh2z_view_project`: project and classify view-space records (only `x y z`
  read). Same result as `sh2z_view_batch` after its transform. May run in place.
- `sh2z_view_edge_steps`: points along `from -> to` at fractions
  `(first_num + k * step_num) / den`, then projected. A road quad is cut where
  a repeating texture restarts, which VDP1 cannot repeat by itself. Six
  divisions per edge set up 16-bit-extended fixed-point steps, so each point
  costs additions, not divisions. `first_num + (count - 1) * step_num <= den`
  is required, keeping every point on the segment.
- `sh2z_view_clip_near`: clip a quad against `z = near_z` only (side clipping is
  left to VDP1). Returns 0, 3, 4, 5 or 6 vertices; each crossing point is
  computed from the in-front end of its edge, so two quads sharing an edge get
  the same point.

### Contract notes

- **Accuracy of stepped points.** Point `k` is within `1 + (k + 1) / 65536`
  units per axis of the exact rational point on the segment: just over one
  unit, not "within one unit". Without the 16 extra fraction bits the error
  would grow by up to one unit per step. Points depend on the direction, so
  step shared edges in one canonical direction.
- **Six outputs, not five.** A quad with two *opposite* corners in front clips
  to a hexagon. That only happens for a non-planar or non-convex quad, but the
  function is total, so `out` holds six vertices.

### Evidence

**HOST** (`tests/test_edges.c`, GCC, Clang, ASan/UBSan): 1,297,747 stepped
points over 40,000 random edges (counts 1 to 64, `first_num == 0`, step 0,
full-int32 spans) equal an independent 128-bit oracle and lie within the bound
above. 200,000 quads over all 16 in-front/behind patterns, including corners
exactly on the plane, equal the oracle, keep their crossing points when the
winding is reversed, and clip in place. 1,024,000 projected view-space points
equal the oracle, in place included. Every argument rule writes nothing on
failure. Controls, each detected: steps without the first offset (1,138,922
mismatches), crossings computed from the walk order instead of the in-front
end (149,989), near plane exclusive (26,786).

**MODELLED** (SH-2 model capture, both entry points run on the SH-2 and
compared with the harness's own oracle): 268,025 stepped points, 65,536
clipped quads (4,096 per pattern) and 262,144 projected points, all exact.
Median cycles:

| Function | Portable | `_divu` |
| --- | ---: | ---: |
| `sh2z_view_project`, per point (512 in front) | 612.4 | 101.4 |
| `sh2z_view_edge_steps`, per point (64-point edges) | 713.3 | 177.3 |
| `sh2z_view_edge_steps`, one-point call (setup) | 4,327 | 2,148 |
| `sh2z_view_clip_near`, per quad (two crossings) | 3,099 | 1,647 |

The `_divu` edge setup does its six exact fractions as twelve hardware
divisions (spans of 2^31 units or more fall back to 32-bit software long
division). Projection reuses the batch kernel's assembly from a second entry
point. No console or emulator run.

## Guard-band clip (`view-guard-1`)

`sh2z_view_clip_near` leaves side clipping to VDP1, but VDP1 command
coordinates must lie in -1024..1023, and its clipping rectangle cannot make an
out-of-range vertex legal. Clamping such a vertex bends the polygon's edges.
`sh2z_view_clip_guard` (and `_divu`) clips a view-space polygon of 3 to 8
corners against near, far and four side planes *before* projection. Every
output vertex then projects inside a caller-chosen guard rectangle, and the
edges stay on their original lines. Full definition in the header.

- **Planes** are exact 64-bit linear tests derived from the projection. A
  corner inside all six projects inside the rectangle, by the batch's own
  rounding.
- **Crossings** are computed from the inside end, so shared edges match in
  either winding. Near and far crossings are exact, and near crossings are
  the same points `sh2z_view_clip_near` makes. Side crossings use one
  hardware-sized division per crossing (a Q30 fraction) and lie within
  `1 + 3|O.a - I.a|/2^30` units of the exact point.
- **Clamp.** Projected `sx`, `sy` are clamped to the rectangle. That moves
  only crossing vertices whose rounded position projects just outside it:
  by at most one pixel wherever one view unit is at most 1/16 pixel across
  the band (measured).
- **Attributes.** Up to four caller-owned `int32` attributes per corner
  (texture u, v, shade, colour) are interpolated with exactly the
  coordinates' rule, so they also agree across shared edges.
- **Capacity.** The output holds up to 16 vertices. A planar convex
  polygon needs at most `count + 6`; a twisted or non-convex one can need
  more, and then the call returns `SH2Z_VIEW_GUARD_OVERFLOW` and writes
  nothing. `SH2Z_VIEW_GUARD_INVALID` likewise writes nothing.

**Routing.** Only polygons that need it should go through the clipper: any
corner `NEAR` or `FAR`, or with `sx`/`sy` outside the band in the batch
output. A quad already inside the band costs about 2,750 modelled cycles here
for nothing.

### Evidence

**HOST** (`tests/test_guard.c`, GCC, Clang, ASan/UBSan):
- 300,000 random polygons (planar convex and twisted; corners exactly on
  planes; whole-pixel and arbitrary focal lengths; the port's ±900 band, the
  screen, and bands up to 16,384 pixels).
- 1,025,373 output vertices equal an independent 128-bit oracle on both entry
  points, in place included, and are identical in reversed winding.
- 4,575 quads equal `sh2z_view_clip_near` byte for byte.
- 19,473,780 side crossings are within the documented bound.
- No planar convex polygon overflowed; the largest growth was +5. 19 twisted
  polygons overflowed, and none wrote anything.
- The port's case (edge from screen (-2840,112) to (160,12), band ±900): the
  new corner is (-899,47), and the edge meets x = 0 at y = 17.29 against 17.33
  unclipped.
- Controls detected: crossings from the walk order (160,494 mismatches), no
  clamp (30,504 polygons outside the band), plane boundary exclusive (54,013).

**MODELLED** (SH-2 model capture): 65,536 random polygons, 261,395 output
vertices exact on both entry points; 3 overflows, nothing written. Median
cycles on the port's camera with the ±900 band, quads with two attributes:

| Quad | Output vertices | Portable | `_divu` |
| --- | ---: | ---: | ---: |
| All corners inside the band | 4 | 4,484 | 2,752 |
| One corner far left (the port's case) | 5 | 6,809 | 4,371 |
| Two corners behind the camera, one far left | 5 | 11,556 | 7,498 |

No console or emulator run.
