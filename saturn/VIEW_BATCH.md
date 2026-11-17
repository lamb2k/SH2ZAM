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
| Bounded texture-merge planner (pixel or row tolerance) | Landed, `view-merge-1` (see below): HOST and MODELLED; 223 modelled cycles per boundary at 1 px |
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

## Texture-merge planner (`view-merge-1`)

A road renderer that draws one VDP1 distorted sprite per texture repetition
can save commands by drawing 2 or 4 consecutive repetitions as one quad. VDP1
maps the texture linearly between a quad's corners, while the
one-quad-per-repetition reference has a perspective-projected vertex at every
repetition boundary, so a merged quad drifts away from the reference between
its ends. `sh2z_view_merge_runs` chooses the merges so that the drift stays
within a tolerance you set, and checks it exactly in integers. The full
definition is in the header.

### Contract

```c
int sh2z_view_merge_runs(uint8_t *runs_out, const sh2z_view_vertex_t *left,
    const sh2z_view_vertex_t *right, const uint32_t *rows, const uint32_t *end_masks,
    unsigned count, uint32_t run_mask, uint32_t tol_q8, unsigned mode);
```

- **Strip.** `count` boundaries (2 to 65,536). Boundary `j` has screen points
  `left[j]` and `right[j]` (only `sx`, `sy` and `outcode` are read) and
  `rows[j]`, the cumulative source-texture row there. `rows` must be strictly
  increasing, so partial and odd-sized repetitions are fine.
- **Runs.** Bit `r` of `run_mask` allows runs of `r` repetitions (2 to 31).
  A single repetition is the reference and is always allowed. Runs of 1, 2
  and 4 are `(1u<<1)|(1u<<2)|(1u<<4)`, which is `0x16`.
- **Alignment.** If `end_masks` is not NULL, a run of `r` may end at boundary
  `j` only if bit `r` of `end_masks[j]` is set. The end and the length fix
  the start, so one word per boundary expresses any start and end alignment
  per run length. For example, if the ×4 texture lines up only every fourth
  boundary from 0, set `1u<<4` in `end_masks[j]` for `j % 4 == 0`. This
  replaces the `can_end` byte the request suggested: it covers that case
  (all bits or none) and also tier textures with different alignments.
- **Blocking.** A boundary whose outcode has any bit other than LEFT, RIGHT,
  TOP or BOTTOM on either rail is never part of a merged run, ends included.
  NEAR corners have no screen position, and FAR ones are culled anyway. Bits
  `0x40` and `0x80` are unused by the view functions, so a caller can set
  one of them to keep a boundary out of every merge (for example, a corner
  that needs the guard clip).
- **Plan.** Greedy, from boundary 0: take the longest allowed run whose
  every interior boundary passes the test on both rails, emit its length, and
  continue from its end. Returns the number of runs (their lengths sum to
  `count - 1`). On invalid arguments it returns `SH2Z_VIEW_MERGE_INVALID` and
  writes nothing: a null pointer, `count` out of range, an unknown mode,
  `tol_q8 > 2^24`, rows not strictly increasing, or `runs_out` overlapping
  an input.

**The test.** For a run over boundaries `0..n` with `S = rows[n] - rows[0]`,
on each rail `P`, each interior boundary `k` with `t = rows[k] - rows[0]`:

```
D = P[n] - P[0]        E = S * (P[k] - P[0]) - t * D      (per axis, exact)
```

The merged quad draws row `t` at `P[0] + D * t / S`, and the reference draws
it at `P[k]`, so `E / S` is the drift of that row in pixels.

| Mode | Passes when | Meaning |
| --- | --- | --- |
| `SH2Z_VIEW_MERGE_PIXELS` (primary) | `max(\|E.x\|, \|E.y\|) * 256 <= tol_q8 * S` | boundary row within `tol_q8/256` px (Chebyshev) |
| `SH2Z_VIEW_MERGE_ROWS` | `max(\|E.x\|, \|E.y\|) <= S` and `\|E . D\| * 256 <= tol_q8 * \|D\|^2` | within 1 px, and within `tol_q8/256` texture rows along the merged rail |

### Why the boundary test bounds the whole quad (pixel mode)

Put `u` in [0, 1] across the strip and `v` in [0, S] in rows. VDP1 steps both
rails and interpolates between them, so the merged quad draws texel `(u, v)`
at

```
M(u, v) = (1 - u) Lm(v) + u Rm(v),   Lm(v) = L[0] + (L[n] - L[0]) v / S   (same for R)
```

The reference draws it at `F(u, v) = (1 - u) Lf(v) + u Rf(v)`, where `Lf` is
linear on each repetition `[t_k, t_k+1]` and passes through `L[k]`. Then:

1. `M - F = (1 - u) dL(v) + u dR(v)`, where `dL = Lm - Lf`.
2. `dL` is linear between boundaries and zero at both ends of the run. Each
   coordinate therefore reaches its extremes at a boundary, where
   `|dL(t_k)| = |E_k| / S`. The same holds for `dR`.
3. The Chebyshev norm is convex, so
   `|M - F| <= (1 - u)|dL(v)| + u|dR(v)| <= max over rails and k of |E_k| / S`.

So in pixel mode **every texel of a merged quad, on the rails and inside, is
drawn within `tol_q8/256` px of its reference position**, and the maximum is
reached at a boundary vertex. The same statement read per pixel: at any pixel
the merged quad covers, the texel it shows is drawn by the reference within
the tolerance of that pixel. This is the screen-pixel error, the texel error
divided by the local texel density. Because the rails move by at most the
tolerance, the quad's outline changes by at most the tolerance too.

The bound is for the ideal mapping. VDP1's own stepping and rounding apply
equally to the reference, and the test uses the same integer corners the
quads are drawn with.

### Rows mode (secondary)

Rows mode bounds two things. First, the 1 px pixel test, so the proof above
caps the drift at 1 px everywhere. Second, at each interior boundary, the
boundary vertex projected onto the merged rail must be within the tolerance
of row `t`. The merged quad draws row `S ((P[k] - P[0]) . D) / |D|^2` at that
projection; subtracting `t` gives `(E . D) / |D|^2`.

So the condition is `|E . D| * 256 <= tol_q8 * |D|^2`. The request's
`tol_q8 * S * |D|^2` had an extra factor `S`, because `E` already carries
`S`. A rail whose two ends coincide on screen passes this half, and the
1 px guard still applies.

For a straight rail (boundary vertices on one line), this is the exact row
error along the rail, again largest at a boundary. Rows mode does **not**
bound the row error at every screen pixel. Inside a thin or slanted reference
quad, a drift of up to 1 px can cross many texture rows. The host property
test measures this (below). For a texel-accurate gate, use pixel mode with a
tolerance chosen from the texel density.

### Ranges

`sx`, `sy` differences are within ±65,535 and `S`, `t` < 2^32, so
`|E| < 2^49`. Each test compares `max|E|` with `floor(tol_q8 * S / 256)`,
which is exact for integers and below 2^48. In rows mode the guard first
gives `|E.x|, |E.y| <= S`, so `|E . D| < 2^49` and
`floor(tol_q8 |D|^2 / 256) < 2^49`. Everything fits 64 bits, and there is no
division.

When `S < 2^14` (any realistic run), `|E| < 2^31`, so the products are plain
32-bit `mul.l`; longer spans use 32 × 32 → 64-bit products. Candidate run
lengths are walked with constant shifts, because SH-2 has no barrel shifter.
There is no `_divu` variant because there is no division.

### Evidence

**HOST** (`tests/test_merge.c`, GCC and Clang strict, ASan/UBSan):
- **Oracle.** An independent 128-bit oracle, written from the header text,
  uses the expanded form `E . D = S ((P[k] - P[0]) . D) - t |D|^2` and a
  brute-force blocking scan.
- **Strips.** 60,000 road strips: perspective rails from a synthetic camera
  with yaw, pitch and roll, depths from behind the camera to 300 units,
  repetitions 0.2 to 12 units, curvature, hills and banking, partial and odd
  row spans. Plus 60,000 adversarial strips: int16 extremes, rails linear in
  the row ±1 px with huge row counts (23,884 strips with `S > 2^31`), and
  random clip flags.
- **Settings.** Masks {1,2}, {1,4}, {1,2,4}, {1,2,4,8}, all lengths and
  random; end masks absent, tier-aligned or random; both modes; tolerances
  0, 3, 64 to 2,048 and up to the maximum.
- **Exactness.** 6,507,825 runs (742,275 merged) equal the oracle, and
  nothing is written past them. Every plan obeys the mask, the end masks and
  the blocking rule, covers every repetition and meets its bound recomputed
  in floating point.
- **Controls detected.** Interior loop one boundary short: 85,045 plans
  differ. Right rail not checked: 56,004. Rows mode without the pixel guard:
  21,166. Greedy taking the smallest passing run: 35,404. NEAR/FAR checked
  only inside the run: 23,611.
- **Hand cases.** Exact thresholds in both modes, including `S = 2^32 - 1`
  with int16 corners (0.4999 px: 128 passes, 127 fails), blocking and side
  flags, end-mask alignment, and the bowed rail that only the rows-mode guard
  rejects.
- **Rejected calls.** Null pointers, count, mode, tolerance, equal, falling
  and wrapping rows, and overlap with each input array: all write nothing.
- **Property, pixel mode.** 24,000 merged quads, 10,120,500 texel samples
  (13 rows per repetition by 9 positions across, rails included). The drift
  never exceeded the boundary maximum by more than 9.1e-13 px (floating-point
  rounding; the test allows 1e-9). It reached the boundary maximum on every
  quad and never exceeded the tolerance. **The boundary-maximum claim
  holds.**
- **Property, rows mode.** The drift is again bounded by its boundary maximum
  and the 1 px guard. The row error at screen points was found by inverting
  the reference's bilinear quads, on 9,341 quads where every quad is
  convex, with 2,641,275 samples. 2,623 quads exceed the row tolerance
  somewhere. Measured in pixels at the reference's local row density, the
  excess reached 26.9 px. **The along-rail row claim holds at the boundary
  vertices; it does not hold at every screen point.**

**Plan quality** on host road strips (64 boundaries, runs {1,2,4}, 1 px or
1 row): 40.23 quads per 63 repetitions in pixel mode, against 56.57 in rows
mode.

**MODELLED** (SH-2 model capture, `merge.h`):
- **Exactness.** 16,384 random strips, mixing road and adversarial, up to
  200 boundaries. All 1,349,576 runs (132,486 merged) equal the harness
  oracle. 4,078 calls with one argument broken returned -1 and wrote
  nothing.
- **Timing.** 64 road strips in front of the camera, 64 boundaries, runs of
  1, 2 and 4:

| Mode, tolerance | Quads per 63 repetitions | Median cycles per boundary | Max |
| --- | ---: | ---: | ---: |
| Pixels, 0.5 px | 33.98 | 308.1 | 414.6 |
| **Pixels, 1 px** | **25.16** | **223.1** | 389.0 |
| Pixels, 2 px | 22.20 | 201.3 | 366.0 |
| Pixels, 1 px, ×2/×4 aligned to even/fourth boundaries | 27.52 | 210.2 | 276.5 |
| Rows, 1 row | 52.12 | 527.1 | 698.3 |

These cycles include argument validation and the row-order pass. No console
or emulator run.

### Caller guidance

- **Use pixel mode** with `tol_q8 = 256` (1 px), or 128 or 512 for 0.5 or
  2 px. The tolerance bounds every texel's drift. A p99 gate over pixels
  is therefore met by construction for the ideal mapping. What remains is
  VDP1 rasterization, which the reference shares.
- **Plan on projected corners.** Call the planner on the corners you will
  draw: after `sh2z_view_batch` or `sh2z_view_project`, and before any guard
  clip. Boundaries with NEAR or FAR stay single repetitions. Route those
  quads through `sh2z_view_clip_guard` as before.
- **Coordinates are int16.** The test uses `sx`, `sy` as given. Corners that
  the projection saturated are compared as saturated, so set bit `0x80` on
  them if you clip those quads instead.
- **Rows** are cumulative texture rows at each boundary, so a partial last
  repetition simply has a smaller step. Pass the rows the quad's texture
  coordinates come from.
- **Tier textures.** Use `end_masks` to allow a ×4 run only where the ×4
  texture lines up.
- **Cost.** Pixel mode at 1 px costs about 223 modelled cycles per boundary
  (14,300 per 64-boundary strip), less than projecting the boundary's two
  corners with `sh2z_view_batch_divu` (2 × 134.7). Rows
  mode costs 2.4 times as much and merges far less, because the row test is
  strict where the road is minified.
