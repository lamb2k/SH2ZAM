# View batch: transform, project and classify (experimental)

`sh2zam/view_experimental.h` turns 16-bit model positions into view-space
coordinates, integer screen positions and clip outcodes in one call. It is
built for SH-2 renderers that would otherwise transform, divide and test six
planes per vertex in 64-bit arithmetic. Contract tag: `view-contract-1`.

## Status

<!-- One row per contract. After the first three rows, rows are sorted by contract tag; the console row stays last. -->

| Part | State |
| --- | --- |
| Header, numeric definition, portable C (`sh2z_view_batch`) | Landed, `view-contract-1` |
| SH-2 path with the hardware divider (`sh2z_view_batch_divu`) | Landed, `view-sh2-1` |
| SH-2 model timing (MODELLED tier) | 134.7 cycles per vertex at 512 vertices |
| View from a Q2.30 camera basis, Q20.12 eye and lens; `sh2z_view_place` for a Q20.12 eye | `view-camera-1` (see below): HOST and MODELLED; 691 modelled cycles per view, within 0.05 px of exact |
| Fractional (Q16) centre (`_center` forms of every `_trunc` path and of the source-order guard clip) | `view-center-1` (see below): HOST and MODELLED; +4.3 modelled cycles per batch vertex over `_trunc`, +6 to +9 per source-order clip (divider) |
| View-space projection, edge stepper, near clip | Landed, `view-edges-1` (see below) |
| Guard-band polygon clip (near, far, sides; attributes) | Landed, `view-guard-1` (see below) |
| Guard clip in a source clipper's vertex order (fan-mapped textures) | `view-guard-src-1` (see below): HOST and MODELLED; −10% to +6% modelled cycles against the `view-round-1` guard clip |
| Bounded texture-merge planner (pixel or row tolerance) | Landed, `view-merge-1` (see below): HOST and MODELLED; 223 modelled cycles per boundary at 1 px |
| Tier texture-merge planner (equal-height runs, no rows) | `view-merge-2` (see below): HOST and MODELLED; 38.7 modelled cycles per boundary at 1 px, 64 boundaries (65.5 at 8); same plans as `view-merge-1` |
| Final-coordinate truncation (`_trunc` forms of every projecting path) | `view-round-1` (see below): HOST and MODELLED; +8.4 modelled cycles per batch vertex |
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

## Tier texture-merge planner (`view-merge-2`)

`sh2z_view_merge_runs` costs about 223 modelled cycles per boundary at 1 px.
A road renderer calls it about 51 times a frame for about 397 boundaries,
and the caller measured 4.8 ms mean and 11.0 ms p99 a frame for its merge
planning. Its strips use tier textures that hold 2 or 4 whole periods, and
it permits a run only where the run covers whole periods (its end masks).
The repetitions of every permitted run then have the same texture height,
and the general test collapses to a few subtractions per boundary.
`sh2z_view_merge_tier` is that planner. It reads no rows and uses no
multiplication, and on SH-2 runs of 2 and 4 go through a hand-written
kernel. For such strips its plan is the plan of `sh2z_view_merge_runs` in
pixel mode, byte for byte.

### Contract

```c
#define SH2Z_VIEW_MERGE_TIER_RUNS 0x00010116u   /* runs of 1, 2, 4, 8, 16 */
int sh2z_view_merge_tier(uint8_t *runs_out, const sh2z_view_vertex_t *left,
    const sh2z_view_vertex_t *right, const uint32_t *end_masks, unsigned count,
    uint32_t run_mask, uint32_t tol_q8);
```

- **Strip, end masks, blocking, plan, output.** As for `sh2z_view_merge_runs`:
  `count` boundaries (2 to 65,536), only `sx`, `sy` and `outcode` read; bit
  `r` of `end_masks[j]` lets a run of `r` end at boundary `j`; a boundary with
  an outcode bit other than LEFT, RIGHT, TOP or BOTTOM on either rail is never
  part of a merged run; greedy longest run first; `runs_out` gets the run
  lengths, which sum to `count - 1`.
- **Run lengths.** 1, 2, 4, 8 and 16 (`run_mask` bits in
  `SH2Z_VIEW_MERGE_TIER_RUNS`). The road renderer's masks (`0x16`, `0x6`,
  `0x2`) pass unchanged.
- **No rows.** The repetitions of a run are taken to have equal texture
  height. Permit a run (through `run_mask` and `end_masks`) only where that
  holds. With equal heights the plan equals `sh2z_view_merge_runs(...,
  rows, ..., SH2Z_VIEW_MERGE_PIXELS)` for any such `rows`.
- **Invalid arguments** return `SH2Z_VIEW_MERGE_INVALID` and write nothing:
  a null `runs_out`, `left` or `right`, `count` out of range,
  `tol_q8 > 2^24`, a `run_mask` bit other than bit 0 and the five lengths, or
  `runs_out` overlapping an input.

**The test.** For a run of `r` repetitions from boundary `i`, on each rail
`P` and for each interior boundary `k = 1 .. r-1`:

```
F = r * (P[i+k] - P[i]) - k * (P[i+r] - P[i])     (per axis, exact)
passes when  max(|F.x|, |F.y|) * 256 <= tol_q8 * r
```

Because `F` is an integer, this is `|F| <= tol_q8 >> (8 - log2 r)`: limit
`tol_q8 >> 7` for runs of 2, `>> 6` for 4, `>> 5` for 8, `>> 4` for 16.

### The bound and its proof

**Every texel of a merged quad, on its rails and inside, is drawn within
`tol_q8 / 256` px (Chebyshev) of where the one-quad-per-repetition
reference draws it**, provided the run's repetitions have equal texture
height. The maximum is reached at a boundary vertex.

Proof. Give each repetition of the run `h` rows, so `S = r h` and boundary
`k` sits at row `t = k h`. The `view-merge-1` error of boundary `k` is then

```
E = S (P[i+k] - P[i]) - t (P[i+r] - P[i]) = h F,     E / S = F / r.
```

`F / r` is the distance in pixels from the reference's boundary vertex to the
point where the merged quad draws the same row. The test is
`max |F| / r <= tol_q8 / 256`, which is exactly the pixel-mode test of
`view-merge-1` with these rows. The `view-merge-1` proof
([above](#why-the-boundary-test-bounds-the-whole-quad-pixel-mode)) then
applies unchanged. Each rail's error is linear between boundaries and zero at
the run's ends, the quad interpolates its rails, and the Chebyshev norm is
convex. So no texel drifts further than the worst boundary, and the worst
boundary is within the tolerance. ∎

If a permitted run has repetitions of unequal height, the bound holds against
a reference with equal heights, not against the true one. The caller's end
masks are what make the heights equal.

A cheaper test that checks only the middle boundary of a run of 4 does **not**
bound the drift. On the host road strips below it let boundaries 1 and 3
drift by up to 41 px at 0.5 px, 120 px at 1 px and 166 px at 2 px.

### Ranges

`sx`, `sy` differences are below 2^16 and `r <= 16`, so `|F| < 2^21`. With
the limit (at most 2^20), every value fits 32 bits. Each comparison is one
unsigned compare: `|f| <= L` exactly when `f + L` lies in `0 .. 2L` as an
unsigned value.

### SH-2 kernel

On SH-2, masks with runs of {2, 4} or {2} (the road renderer's ×4 and ×2
tiers) go to `sh2zi_merge_tier24`, hand-written assembly in
`sh2zam_view_merge_tier.c`. Runs of 8 and 16 and host builds use the
library's C, which follows the same contract.

- A run of 4 tests the middle boundary first, then the outer two. With
  `K = 2 (P[0] + P[4]) - L4` and `D = P[4] - P[0]`, the shifted errors are
  `F2 + L4 = 4 P[2] - K`, `F1 + L4 = 4 P[1] - K + D` and
  `F3 + L4 = 4 P[3] - K - D`. Each is one `CMP/HI` against `2 L4`. A rail
  costs 53 instructions when it passes, fewer when it fails early.
- A run of 2 tests `4 P[1] - (2 (P[0] + P[2]) - L4)` against `2 L4`, which is
  `2 F + L4`. `|2 F| <= L4` exactly when `|F| <= L4 >> 1`, the run-of-2 limit.
  This costs 22 instructions per rail.
- Blocked boundaries are found by a scan that tests four boundaries a step,
  one long load per rail each (the outcode is the long's top byte), and stops
  the fast loop before the next blocked boundary.

### Evidence

**HOST** (`tests/test_merge_tier.c`; GCC strict, Clang strict, ASan/UBSan):
- **Oracle.** An independent oracle, written from the header text, uses
  128-bit arithmetic and a brute-force blocking scan.
- **Strips.** 40,000 road strips with rows and end masks as a road renderer
  builds them (×1, ×2, ×4, ×8, ×16 tiers, partial first and last
  repetitions, corners outside ±900 px flagged `0x80`). 40,000 strips of
  straight quads stepped by `sh2z_view_edge_steps`. 60,000 adversarial strips
  (int16 extremes, near-linear rails, random outcodes, random masks).
  Tolerances 0 to the maximum.
- **Exactness.** 6,387,501 runs (839,811 merged: 497,948 ×2, 294,882 ×4,
  37,056 ×8, 9,925 ×16) equal the oracle. Nothing is written past them.
- **Against view-merge-1.** 130,022 plans (6,076,461 runs) equal
  `sh2z_view_merge_runs` in pixel mode: every strip whose permitted runs
  have equal-height repetitions.
- **Blocking exercised.** On 23,659 adversarial strips the plan changes when
  the NEAR, FAR and undefined outcode bits are cleared.
- **Controls detected.** Only the middle boundary tested: 24,993 plans
  differ. Right rail not checked: 43,167. Strict `<` at the threshold:
  25,074. Greedy takes the smallest passing run: 40,698. NEAR/FAR checked
  only inside the run: 24,920. End mask read at the run's start: 37,950.
  Interior boundary `k` tested as `r - k`: 32,964.
- **Hand cases.** Linear strips, blocking flags, end masks, exact thresholds
  for runs of 2, 4, 8 and 16, and int16 extremes. Rejected calls (null
  pointers, count, tolerance, run lengths, overlap with each input) write
  nothing.
- **Drift property.** 335,915 merged quads, 113,284,539 texel samples. The
  drift never exceeded the tolerance or the quad's boundary maximum (by more
  than 2.3e-13 px of floating-point rounding). **The bound holds.**
- **Total.** 22,147,066 checks.

**Commands and drift** on host road strips (4,000 strips of 64 boundaries,
runs {1, 2, 4} on whole periods, a road renderer's end masks):

| Tolerance | Quads per 63 repetitions, view-merge-2 | view-merge-1, same end masks | Drift p50 | Drift p99 | Drift max |
| --- | ---: | ---: | ---: | ---: | ---: |
| 0.5 px | 38.092 | 38.092 | 0.126 px | 0.501 px | 0.5000 px |
| 1 px | 33.095 | 33.095 | 0.209 px | 0.896 px | 1.0000 px |
| 2 px | 31.316 | 31.316 | 0.229 px | 1.501 px | 2.0000 px |

The command count is identical to `view-merge-1` at every tolerance, because
the plans are identical. Percentiles are upper edges of 1/1024 px bins.

**MODELLED** (SH-2 model capture, [`merge-model`](benchmarks/merge-model/README.md),
[results](benchmarks/merge-model/results.json), captured by
[CI run 44206041141](https://github.com/lamb2k/SH2ZAM/actions/runs/44206041141)):
- **Exactness.** 12,000 strips (road and adversarial, 2 to 200 boundaries),
  1,065,023 boundaries. All 806,730 runs (114,985 merged: 68,721 ×2,
  39,745 ×4, 4,931 ×8, 1,588 ×16) equal the harness oracle. 10,969 plans also
  equal `sh2z_view_merge_runs` on the target. 2,987 calls with one argument
  broken returned -1 and wrote nothing.
- **Timing.** 64 road strips per row, the same strips for both planners. Both
  return the same plan. Whole calls, argument checks included:

| Strip | Quads | view-merge-2, median cycles per boundary | Max | view-merge-1 | Ratio | Drift p99 / max |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 64 boundaries, 0.5 px | 32.23 | 50.02 | 60.02 | 261.21 | 5.22 | 0.501 / 0.500 px |
| **64 boundaries, 1 px** | **24.73** | **38.74** | 45.52 | 188.60 | 4.87 | 0.876 / 1.000 px |
| 64 boundaries, 2 px | 22.09 | 38.30 | 42.80 | 179.95 | 4.70 | 1.292 / 2.000 px |
| 64 boundaries, 1 px, runs {1, 2} | 36.53 | 40.92 | 42.92 | 214.23 | 5.24 | 0.501 / 1.000 px |
| 64 boundaries, 1 px, no end masks | 23.42 | 37.77 | 47.25 | 186.54 | 4.94 | 0.806 / 1.000 px |
| 64 boundaries, runs of 1 only | 63.00 | 6.89 | 6.89 | 67.33 | 9.77 | — |
| **8 boundaries, 1 px** | 5.03 | 65.50 (524 a call) | 117.12 | 248.88 | 3.80 | 0.917 / 1.000 px |
| 8 boundaries, runs of 1 only | 7.00 | 20.12 (161 a call) | 20.12 | 83.62 | 4.16 | — |

A call costs about 200 modelled cycles before its first boundary: argument
checks, the overlap test, saved registers and the first blocking scan. At the
road renderer's mean of about 8 boundaries a call, that overhead is why the
cost per boundary is 65.5 rather than 39. No console or emulator run.

### Caller guidance

- **Switch.** Replace the `sh2z_view_merge_runs` call with
  `sh2z_view_merge_tier(runs, left, right, end_masks, count, mask, 256u)`.
  Keep `runs`, the end masks, the mask and the tolerance. The rows are no
  longer needed. The output is the same, so the drawing code does not change.
- **Equal heights.** Keep the end masks that permit a run only where it covers
  whole periods (a span of `r` periods from an aligned boundary). Those runs
  have equal-height repetitions, which the bound needs. Partial first and
  last repetitions stay single quads.
- **Runs of 1 only.** With mask `0x2` there is nothing to merge. The call
  fills `runs` with 1 in 6.9 modelled cycles per boundary, or the caller can
  skip it.
- **Tolerance.** `tol_q8 = 256` bounds every texel's drift by 1 px, as with
  `view-merge-1` in pixel mode. `128` and `512` give 0.5 and 2 px.

## Final-coordinate truncation (`view-round-1`)

Every `view-contract-1` projection rounds the Q16 quotient to a pixel first:
`sx = center_x + (qx >> 16)` and `sy = center_y - (qy >> 16)`. Some source
renderers keep the screen position in Q16.16 and truncate it at the end,
`s = (65536 * center +/- q) / 65536` with C division. For a port that must
reproduce such a renderer's coordinates exactly, the `_trunc` forms of every
projecting entry point apply that rule instead:

```c
bool sh2z_view_batch_trunc(...);        bool sh2z_view_batch_trunc_divu(...);
bool sh2z_view_project_trunc(...);      bool sh2z_view_project_trunc_divu(...);
bool sh2z_view_edge_steps_trunc(...);   bool sh2z_view_edge_steps_trunc_divu(...);
unsigned sh2z_view_clip_near_trunc(...); unsigned sh2z_view_clip_near_trunc_divu(...);
int sh2z_view_clip_guard_trunc(...);    int sh2z_view_clip_guard_trunc_divu(...);
```

Each takes the same arguments, has the same failures and writes the same
bytes as the function without `_trunc`, except for `sx`, `sy` and the outcode
bits that follow from them. The old functions and their output are
unchanged.

### Matching sources that truncate the final coordinate

**The source rule.** With an integer centre `c`, focal length `F` (Q16.16)
and view-space `a`, `z`:

```
Wx = 65536 * c_x + trunc(F_x * x / z)      sx = trunc(Wx / 65536)
Wy = 65536 * c_y - trunc(F_y * y / z)      sy = trunc(Wy / 65536)
```

`view-contract-1` gives `sx = floor(Wx / 65536)` and `sy = ceil(Wy / 65536)`,
because the centre is whole and `>>` floors. Truncation is floor for a
positive position and ceiling for a negative one. So `view-contract-1`
already matches the source for `x` right of column 0 and for `y` above row 0.
It is one pixel off where the position is not a whole pixel and lies left of
column 0 (`sx` one lower) or below row 0 (`sy` one higher). The second case
covers almost the whole screen.

**Caller adaptations, and why they are not enough.**

- *Centre minus one.* Passing `center_y - 1` gives `ceil(Wy) - 1`. That is
  `floor(Wy)` only when `Wy` is not a whole pixel. On whole pixels it is one
  too low, for example source centre 112, focal 160 px, `y = -600`,
  `z = 1000`: the source gives 208, the adapter 207. It is also one too low
  everywhere above row 0.
- *Mirrored y.* Negate the rotation's y row and `t[1]` after
  `sh2z_view_place` (placing a negated row rounds the other way), pass
  `center_y' = -center_y`, reflect the clip rectangle's top and bottom, swap
  the TOP and BOTTOM bits, and take `sy = -sy'` (clamped to 32,767). The
  quotient truncation is odd, so `-sy' = floor(Wy / 65536)` exactly, which
  is the source's value for every `y` at or below row 0, whole pixel or not.
  The edge stepper's and near clip's arithmetic is odd too, so their points
  mirror exactly. But the rule is now floor on both axes, so every vertex
  left of column 0 or above row 0 that is not on a whole pixel is still one
  pixel off. A road quad drawn into a guard band has such vertices. No
  single view can do better, since truncation switches between floor and
  ceiling at the source's zero, not the library's centre.
- *Exact, with a second projection.* After the mirrored-y projection,
  project every vertex with a negative result again with x mirrored
  (`(-x, y, z)`, `center_x' = -center_x`) and take the ceiling from it on
  the negative axes. That is exact, but it costs a second projection (about
  100 modelled cycles with the divider, plus a call) for each vertex left of
  or above the screen. The guard clip is worse. A mirrored view runs the
  top and bottom planes in the opposite order, which changes the clipped
  polygon when both planes cut it. So the caller must clip with the
  unmirrored view, reproject every output vertex itself and redo the clamp
  and outcodes.

The host test measures all three on the same 448,122 points. Centre minus
one misses 229,270 of them. Mirrored y misses 329,663, none at a
non-negative position, each by one pixel. Mirrored y plus the second
projection misses none, at the cost of 334,824 extra projections. Because no
cheap caller-side route is exact, the rule became a library contract.

### Contract

```
qx = clamp(trunc(focal_x * x / z), -(2^31 - 1), 2^31 - 1)      (same for qy)
sx = sat16(trunc((65536 * center_x + qx) / 65536))
sy = sat16(trunc((65536 * center_y - qy) / 65536))
```

- **Relation to `view-contract-1`.** Before saturation, `sx` is one larger
  exactly when `65536 * center_x + qx` is negative and `qx` is not a
  multiple of 65536. `sy` is one smaller exactly when
  `65536 * center_y - qy` is positive and `qy` is not a multiple. Otherwise
  both are equal. The kernels compute it that way: the old floor, then a
  fix-up from the sign and the quotient's low 16 bits.
- **Symmetric clamp.** `view-contract-1` saturates quotients to
  `INT32_MIN..INT32_MAX`. Its floor cannot tell `-2^31` from `-2^31 + 1`, but
  truncation can, so `view-round-1` clamps to `+/-(2^31 - 1)`. The
  projection is then odd: negating `x` and `center_x` negates `sx` before
  saturation, and likewise for `y`. The SH-2 kernel's stand-in division for
  quotients that do not fit already yields `+/-(2^31 - 1)`. The clamp only
  matters 32,768 pixels from the centre.
- **Guard clip.** The six planes of `sh2z_view_clip_guard` are unchanged. A
  point inside them has its exact position inside the rectangle. Truncating
  the quotient and then the position are both monotone and keep a value
  between two whole-pixel bounds, as floor and ceiling do, so the point
  still projects inside. Crossings are then clamped to the rectangle as
  before.
- **Near clip.** Inside corners are copied unchanged, so pass corners
  written by a `_trunc` batch or projection.
- **Geometry.** Stepped points, near crossings, guard crossings and
  attributes are byte-identical to the `view-contract-1` functions (the host
  test checks this).

### Implementation

The `_trunc` functions live in `src/sh2zam_view_round.c`. The argument
rules, stepper axes, near clip and guard passes that both files need moved
verbatim into the private headers `src/sh2zam_view_internal.h` and
`src/sh2zam_view_paths.h`. `sh2zam_view_paths.h` holds the edge stepper and
the guard clip, instantiated once per projection rule. Keeping the new code
in its own file leaves the old functions' compiled code alone. In the
benchmark build, every function section of `sh2zam_view.o` disassembles,
relocations included, exactly as on `main`, and the capture below reproduces
every earlier `results.json` number.

On SH-2 the batch and projection kernels are the same assembly macros,
instantiated with `trunc=1` as two more entry points. At each of the three
places a quotient becomes a pixel, the fix-up keeps the quotient's low 16
bits in `r10` and adds four to seven instructions. With `trunc=0` the macros
expand to the original instructions.

### Evidence

**HOST** (`tests/test_round.c`, GCC and Clang strict, ASan/UBSan):
- **Oracle.** An independent 128-bit oracle, written from the header text,
  covers 614,400 batch vertices (random rotations and eyes, strides 6 and
  10). It also covers 307,200 projected points, aimed exactly at pixels, one
  unit off them and between them, on both sides of zero. Then 156,633
  stepped points, 35,731 near-clip vertices and 35,491 guard-clip vertices.
  All equal the oracle on both entry points, with the `_divu` bytes equal to
  the portable bytes.
- **Geometry and in-place calls.** View-space points, crossings and
  attributes equal the `view-contract-1` functions', in place included.
- **Guard rectangle.** 28,075 guard outputs inside all six planes project
  inside the rectangle without the clamp.
- **Relation to `view-contract-1`.** On 448,122 points, `sx` and `sy` differ
  exactly as stated above: `sx` +1 on 217,793 points and `sy` -1 on 218,852.
- **Hand cases.** The requester's two examples (208 and 192) and whole
  pixels on both sides of zero. Fractions just either side of zero. The
  symmetric clamp: a quotient of `-2^31` gives -32,607 at centre 160, not
  -32,608. Odd symmetry on random points. Argument rules: each failure
  returns what `view-contract-1` returns and writes nothing.
- **Controls detected.** Oracle with x floored: 319,954 mismatches. y
  ceiled: 394,191. Asymmetric quotient clamp: 10,663. Fraction test dropped:
  13,813. Guard clamp dropped: 897.

**MODELLED** (SH-2 model capture, `round.h`): every `_trunc` entry point,
portable and divider forms, runs on the SH-2 against the harness's own
oracle:
- 262,144 batch vertices (145,841 `NEAR`), 131,072 projected points, 201,769
  stepped points, 16,384 near-clip quads and 16,384 guard polygons (64,752
  output vertices). All are exact.
- On these points the rule differs from `view-contract-1` 93,389 times in
  `sx` and 126,405 times in `sy`.

Each timing fixture ran through the `view-contract-1` function and its
`_trunc` form back to back. Medians of 64 trials on the earlier sections'
timing camera (centre 160, 112; focal length 140 to 320 px). Those sections
use their own fixtures, so their medians differ slightly:

| Path | Unit | Portable, old / `_trunc` | `_divu`, old / `_trunc` |
| --- | --- | ---: | ---: |
| batch (512 vertices) | per vertex | 682.10 / 691.33 | 134.68 / **143.11** |
| project (512 points) | per point | 612.40 / 617.52 | 101.44 / 103.96 |
| edge steps (64 points) | per point | 713.23 / 719.69 | 177.22 / 179.50 |
| near clip (2 crossings) | per quad | 3,101 / 3,133 | 1,648 / 1,699 |
| guard, quad inside | per quad | 4,486 / 4,520 | 2,752 / 2,834 |
| guard, one corner far left | per quad | 6,818 / 6,891 | 4,389 / 4,499 |
| guard, near plus side | per quad | 11,556 / 11,620 | 7,494 / 7,595 |

Each path does the same divisions in both forms. No console or emulator
run.

### Caller guidance

- **Use the source's real centre.** The rule rounds the Q16 position itself,
  so drop any centre offset adapter and pass the source centre unchanged.
  The centre must be a whole pixel; for a source centre with a fraction,
  use the `view-center-1` forms below.
- **Use one rule per frame.** Use the `_trunc` form for the batch, edge
  steps and both clips together. The near clip copies corners as given, and
  the merge planner reads whatever `sx`, `sy` it is given.
- **What exactness covers.** The pixel rule only. Points match a source's
  pixels exactly when the view-space points match. A different rotation
  precision, edge-split rule or clip-crossing rule upstream can still move
  a point by a unit.

## Source-order guard clip (`view-guard-src-1`)

A renderer with fan-mapped textures draws each clipped polygon as a fan of
quads from vertex 0 and maps the whole texture onto every fan piece. Then the
polygon's starting vertex and order show on screen, not just its outline.
`sh2z_view_clip_guard_trunc` makes the same cyclic polygon as a source
renderer's clipper, up to rounding, but usually starts it somewhere else.
Rotating its output afterwards does not fix that, because the crossing points
also differ. The opt-in `sh2z_view_clip_guard_src_trunc` (and `_divu`)
reproduces that clipper vertex for vertex.

```c
int sh2z_view_clip_guard_src_trunc(sh2z_view_guard_vertex_t out[16], const sh2z_view_t *v,
    const int16_t guard[4], const sh2z_view_guard_vertex_t *in, unsigned count,
    unsigned attrs, unsigned max_out);
int sh2z_view_clip_guard_src_trunc_divu(/* same */);
#define SH2Z_VIEW_GUARD_SNAP (-3)
```

### The source clipper

The planes, their order (near, far, left, right, top, bottom) and the inside
test `d >= 0` are those of `view-guard-1`. Three things differ:

- **Order.** Each pass walks previous -> current, starting with the closing
  edge (last, first). It emits the crossing, then the current corner.
  `view-guard-1` walks current -> next from (first, second) and emits the
  current corner, then the crossing. When the closing edge crosses a plane,
  the source's output starts with that crossing; `view-guard-1` puts it last.
- **Crossing.** The point is measured from the previous corner P towards
  the current one Q, whichever is inside, at the Q0.32 fraction
  `t = floor(|dP| * 2^32 / (|dP| + |dQ|))`. Each coordinate moves
  `floor(|Q.a - P.a| * t / 2^32)` from P. `view-guard-1` measures from the
  inside end, exactly for near and far and with a Q2.30 fraction for sides.
- **Snap.** The rounded crossing is moved back inside every plane clipped so
  far. z is clamped to `[near_z, far_z]`, then x is moved to
  `ceil(KL*z / focal_x)` or `floor(KR*z / focal_x)` and y likewise, with the
  plane offsets `KL = 65536*left - Cx`, and so on. The source fails the whole
  call if one of those bounds does not fit int32 (used or not), or if no
  lattice point lies between two opposite planes at that depth.

The source also fails once a pass would produce more corners than its buffer
holds (10), so the new function takes `max_out`. Its failures return
`SH2Z_VIEW_GUARD_OVERFLOW` or `SH2Z_VIEW_GUARD_SNAP` and write nothing. Output
vertices are projected by the `view-round-1` rule. They lie inside all six
planes, so they land inside the guard rectangle without the clamp. The full
contract is in the header.

### How far `view-round-1` is from the source (HOST)

`tests/test_guard_src.c` runs an oracle of the source clipper, written from
the clipper itself in 128-bit arithmetic, next to `sh2z_view_clip_guard_trunc`.
It uses 200,000 synthetic road quads and 400,000 random polygons. The road
quads lie on a ground plane under a camera like the port's: centre 160, 112,
focal 140 to 320 px, the ±900 band, pass limit 10. Each quad was drawn until
one corner was behind the camera or outside the band.

| Compared with the source | Road quads | Random polygons |
| --- | ---: | ---: |
| Clipped, non-empty polygons | 176,752 | 169,041 |
| (a) `view-round-1`, same start, identical | 0 | 1,721 |
| (a) same start, same pixels, view space differs | 19,349 | 25,013 |
| (a) same start, pixels differ | 8,707 | 21,307 |
| (a) **rotated**, identical otherwise | 0 | 2,037 |
| (a) **rotated**, same pixels | 123,625 | 51,933 |
| (a) **rotated**, pixels differ | 25,071 | 60,930 |
| (a) different vertex count | 0 | 6,100 |
| (b) source order and planes, `view-guard-1` crossings: vertices differing in view space | 228,807 of 751,420 (≤ 172 units) | 592,312 of 842,273 |
| (b) the same: vertices on a different pixel | 35,432, all one pixel off (in 33,778 polygons) | 142,621 |
| source crossings without the snap: vertices on a different pixel | 179,909, all one pixel off | 344,446 |
| (c) `sh2z_view_clip_guard_src_trunc[_divu]` mismatches | **0** | **0** |

On the road, 84% of clipped quads come out rotated. A rotation changes the
fan's hub and pieces, and so where the whole texture lands. With the order
matched, the crossing coordinates still differ by up to 172 units in every
clipped polygon, and by one pixel in 19% of them. The source's crossing
without its snap leaves a pixel error on about a quarter of the vertices.
Only all three together are exact. The random polygons have corners up to
2^31 units away, so their differences in the last three rows reach the int32
range and thousands of pixels; no bound is given for them.

### Implementation

`src/sh2zam_view_guard_src.c`, a new file: every object of the earlier
library builds byte for byte as on `main` (SH-2 `objdump -dr`, all 16 files).
The core takes the centre as Q16 (`Cx`, `Cy`); the two entry points pass
`65536 * center`.

- **Fraction.** For `den = |dP| + |dQ| < 2^31`, one division:
  `a*2^32/den = 2^31 + (2a - den)*2^31/den` when `2a >= den`, which keeps the
  quotient below 2^31. For `den < 2^59`, the same on `den` and `a` shifted
  until `den < 2^31`, then at most nine exact corrections from the 64-bit
  remainder (proof in the source). Larger distances, which need a focal
  length times a coordinate near 2^58, use a 32-step restoring division.
  Each coordinate's step is then the high word of a 32 x 32 product.
- **Snap.** A bound is divided only when the point is outside it; whether a
  bound fits int32 is two comparisons with `focal * 2^31`.
- **Projection.** Inside all six planes, `65536*left <= Cx + qx <=
  65536*right` (and likewise for y), so the quotients fit the divider and the
  pixels need no clamp. The two divisions overlap.

### Evidence

**HOST** (`tests/test_guard_src.c`, GCC and Clang strict, GCC ASan/UBSan):
- **Oracle.** 751,464 road and 884,634 random output vertices equal the
  source oracle on both entry points, with the `_divu` bytes equal to the
  portable bytes. In-place calls are included. The random set adds 48,432
  failures (38,678 overflow, 9,754 snap), each matched and writing nothing.
  GCC and Clang draw the same fixtures, so every count here is the same
  under both.
- **Coverage.** Every oracle branch runs: 1,574,203 fractions with
  `den < 2^31`, 4,081,294 below 2^59, 50,707 above, 121,774 with a corner on
  the plane (`t = 2^32`), 1,189,953 snap moves, 1,074 empty-lattice failures
  and 8,680 int32 range failures.
- **Properties.** Every source output lies inside all six planes and the
  guard rectangle unclamped. Every output projects exactly as
  `sh2z_view_project_trunc`, and unused attributes are zero. The oracle run
  in `view-guard-1` mode equals `sh2z_view_clip_guard_trunc` on every polygon.
- **Hand cases.** A closing-edge crossing comes first; `view-round-1` gives
  the same corners rotated, with that crossing one unit and one pixel off
  (-899 against -900). A corner exactly on a plane repeats. An edge
  symmetric about a plane crosses at its lattice midpoint. An octagon clips
  to 12 corners and overflows at `max_out` 10. A lattice bound outside int32
  fails with `SH2Z_VIEW_GUARD_SNAP`. Every invalid argument writes nothing.
- **Controls**, each detected (mismatches over 20,000 road quads and 40,000
  random polygons):

| Oracle mutation | Mismatches |
| --- | ---: |
| `view-guard-1` emission order (current, then crossing) | 126,787 |
| Walk starts at (first, second) | 139,264 |
| Planes left, top, right, bottom | 27,273 |
| `view-guard-1` crossing rule | 89,479 |
| No snap | 80,815 |
| No near/far clamp in the snap | 33,353 |
| Fraction rounded to nearest | 33,805 |
| Crossing measured from the current corner | 109,913 |
| Pass limit ignored | 3,840 |
| Snap failures ignored | 1,026 |

**MODELLED** (SH-2 model capture, `guardsrc.h`): both entry points run on
the SH-2 against the harness's own oracle, written from the header text, on
32,768 random polygons (thin rectangles and shallow near planes included) and
16,384 road quads. All 134,608 output vertices are exact; 3,645 overflows and
3,055 snap failures write nothing. Medians of 64 trials on the timing camera
and the ±900 band, each fixture through `sh2z_view_clip_guard_trunc` and the
new clip back to back:

| Quad | Output vertices | `_trunc` portable / `_divu` | `_src_trunc` portable / `_divu` | Divisions, `_trunc` / `_src_trunc` |
| --- | ---: | ---: | ---: | ---: |
| All corners inside (attrs 2) | 4 | 4,518 / 2,834 | 4,277 / **2,873** | 8 / 8 |
| One corner far left (attrs 2) | 5 | 6,896 / 4,496 | 6,774 / **4,577** | 12 / 13 |
| Two corners behind, one far left (attrs 2) | 5 | 11,617 / 7,598 | 9,790 / **6,865** | 23.1 / 18 |
| Road quad from behind the camera (attrs 0, `max_out` 10) | 6 | 12,636 / 8,347 | 12,544 / **8,856** | 24 / 22.6 |

So the `_divu` form costs within about −10% to +6% of
`sh2z_view_clip_guard_trunc_divu`. Near crossings are cheaper: one division
for the fraction, against one per coordinate. Side crossings cost a second
division when the snap moves them. No console or emulator run.

### Caller guidance

- **Swap the call where the source clips.** Pass the source's buffer size as
  `max_out` (10 for a buffer of 10 corners) and attrs 0 if the texture comes
  from the fan, not from per-corner coordinates. Treat both negative results
  as the source's failure.
- **Same inputs.** Give the corners in the source's order, as view-space
  points equal to the source's. Exactness covers order, count and every
  coordinate given equal view-space corners and an integer centre.
- **Q16 centre.** The clip computes with a Q16 centre internally. A source
  whose centre has a fraction needs an entry point that takes it, which can
  pass it through unchanged; these two functions take `sh2z_view_t`'s whole
  pixels. `sh2z_view_clip_guard_src_center` (`view-center-1`, below) is that
  entry point.
- **Attributes**, which the source does not have, are interpolated with the
  same fraction from the previous corner and never snapped.

## Fractional centre (`view-center-1`)

`view-round-1` takes the centre from `sh2z_view_t`, whose `center_x` and
`center_y` are whole pixels. A source renderer whose centre is fractional
keeps it as a Q16.16 value and adds the quotient to it before truncating:

```
X16 = Cx16 + trunc(F_x * x / z)      sx = trunc(X16 / 65536)
Y16 = Cy16 - trunc(F_y * y / z)      sy = trunc(Y16 / 65536)
```

Such centres arise when the source computes the centre in floating point.
A residual of one unit either side of a whole pixel (for example
`193 * 65536 - 1` and `193 * 65536 + 1`) is enough to move `sy` by a row
whenever `Y16` lands on, or a unit beside, a row boundary. The `_center`
forms take the Q16 centre itself:

```c
typedef struct {
    sh2z_view_t view;
    int32_t center_x16,center_y16;
} sh2z_view_center_t;                       /* 68 bytes */

bool sh2z_view_center_init(sh2z_view_center_t *out,const sh2z_view_t *view,
    int32_t center_x16,int32_t center_y16);

bool sh2z_view_batch_center(...);          bool sh2z_view_batch_center_divu(...);
bool sh2z_view_project_center(...);        bool sh2z_view_project_center_divu(...);
bool sh2z_view_edge_steps_center(...);     bool sh2z_view_edge_steps_center_divu(...);
unsigned sh2z_view_clip_near_center(...);  unsigned sh2z_view_clip_near_center_divu(...);
int sh2z_view_clip_guard_center(...);      int sh2z_view_clip_guard_center_divu(...);
int sh2z_view_clip_guard_src_center(...);  int sh2z_view_clip_guard_src_center_divu(...);
```

Each takes a `const sh2z_view_center_t *` where the `_trunc` function takes
a `const sh2z_view_t *`; every other argument is the same.

### Can a caller do it with `view-round-1` alone?

No. The library rounds at multiples of 65536 in its own position
`65536 * c - qy`, and the source rounds at multiples of 65536 in
`Cy16 - qy`. With `Cy16 = 65536 * c +/- 1` the two differ exactly when the
position is at, or one unit beside, a row boundary. Which side it is on
depends on `qy mod 65536`, which the library never returns. Two points can
therefore get the same `sy` from the library and need different rows. No
choice of integer centre and no function of the library's pixels can tell
them apart. A caller could only redo the y division for every vertex, which
is the projection again.

The host test counts single-call tactics against the oracle for both
residuals, on 400 synthetic road-like cameras per residual (a flat road a
few units above or below the eye, focal 160 to 320 px, centre x 160) and on
points aimed at row boundaries in the same views (GCC build):

| Centre y | Tactic | Road-like misses | Row-boundary misses |
| --- | --- | ---: | ---: |
| `193 * 65536 - 1` | `_trunc`, centre 193 (rounded) | 38 of 1,634,277 | 136,951 of 409,600 |
| | `_trunc`, centre 192 (floor) | 1,634,179 | 272,187 |
| | `view-contract-1`, centre 192 | 9,047 | 87,698 |
| | `view-contract-1`, centre 191 | 1,634,276 | 409,600 |
| `193 * 65536 + 1` | `_trunc`, centre 193 | 22 of 1,634,270 | 136,635 of 409,600 |
| | `view-contract-1`, centre 193 | 1,624,611 | 136,049 |
| | `view-contract-1`, centre 192 | 9,658 | 317,688 |

Every tactic misses, so the claim holds. On road-like views, rounding the
centre to 193 misses one vertex in 40,000 to 75,000 (35 and 17 of the misses
are on screen), which an exactness gate still catches. Its misses are not
recoverable either: 37 and 21 of its output rows each map to two different
source rows on the road-like views, and about 70,000 do on the
boundary-aimed points.

One near-miss is worth knowing. With `Cy16 = 65536 * c - 1`,
`view-contract-1` at centre `c - 1` computes `ceil((Y16 + 1 - 65536) / 65536)`,
which is exactly `floor(Y16 / 65536)`. That is the source's row for every
position at or below row 0, and wrong only above it (the 9,047 road-like
misses are vertices above the top of the screen). It does not help with
`+1`, it floors `x`, and a guard band reaches above row 0, so it is not a
substitute.

### Contract

```
qx = clamp(trunc(focal_x * x / z), -(2^31 - 1), 2^31 - 1)      (same for qy)
sx = sat16(trunc((center_x16 + qx) / 65536))
sy = sat16(trunc((center_y16 - qy) / 65536))
```

- **Centre fields.** A centre view is valid when `view` is valid and
  `view.center_x == floor(center_x16 / 65536)` and
  `view.center_y == floor(center_y16 / 65536)` (an arithmetic shift by 16).
  The `_center` functions read the centre only from `center_x16` and
  `center_y16`; the rule makes a stale or forgotten centre fail instead of
  projecting around the wrong point. `sh2z_view_center_init` sets all four
  fields. Every other failure is the `_trunc` function's.
- **Range.** Every int32 centre is valid. `center16 +/- q` needs 33 bits; the
  kernels never form it. They add (x) or subtract (y) the low 16 bits of
  the centre and the quotient apart, fold the carry or borrow into the sum
  of the high halves, and use the low 16 bits of the result as the fraction
  that truncation tests. The floor before saturation lies within
  -65,536 .. 65,536, so it fits any register.
- **Equivalence.** With `center_x16 = 65536 * center_x` and
  `center_y16 = 65536 * center_y`, every `_center` function writes exactly
  what its `_trunc` form writes, return value included.
- **Guard clip.** The six planes of `sh2z_view_clip_guard` with the centre
  taken in Q16 (`65536 * center` becomes `center16`):

  ```
  3 left    d = focal_x*x - (65536*left - center_x16)*z
  4 right   d = (65536*right - center_x16)*z - focal_x*x
  5 top     d = (center_y16 - 65536*top)*z - focal_y*y
  6 bottom  d = focal_y*y - (center_y16 - 65536*bottom)*z
  ```

  The edge rule "more than 16,384 pixels from the centre" becomes
  `|65536 * edge - center16| > 2^30`, so the plane constants fit int32 and
  every distance fits int64 as before. Passes, crossings, clamp and output
  order are those of `view-guard-1`.
  **Why every output still lands inside the rectangle.** Inside the left
  plane, `focal_x * x / z >= 65536 * left - center_x16`, which is a whole
  number. Truncating a quotient keeps it at or above a whole number below it,
  and the symmetric clamp keeps it there too (the bound is within `2^30`).
  So `center_x16 + qx >= 65536 * left`. Truncating that position by 65536
  again keeps it at or above `left`. The other three planes are the same
  argument. Outcodes are computed from the final (clamped) `sx`, `sy`
  against `view.clip`, so they always agree with the pixels.
- **Near clip and edge steps.** The near clip computes the same crossings
  as `sh2z_view_clip_near` and copies inside corners unchanged, so pass
  corners written by a `_center` batch or projection with the same view.
  The edge stepper computes the same points as `sh2z_view_edge_steps` and
  projects each with the rule above.

### Implementation

The functions live in `src/sh2zam_view_center.c`, a new file, so every
earlier object is unchanged. Compiled for SH-2 with the CI flags, with and
without function sections, every other library object is byte-identical to
`main`'s, `sh2zam_view.o` and `sh2zam_view_round.o` included. The SH-2
kernel's assembler macros moved verbatim from `sh2zam_view.c` into the
private header `src/sh2zam_view_asm.h`, so the new file can instantiate them.

The batch and projection kernels are those macros with `center=1`. The
camera block gains two words, the centre fractions shifted into the top
half. The prologue pushes them, so `@r15` and `@(4,r15)` reach them without
a free register. At each place a quotient becomes a pixel, the quotient's
low half, shifted up, is added to (`ADDC`) or subtracted from (`SUBC`) the
centre fraction. The carry or borrow lands in `T`, and the next
`ADDC`/`SUBC` folds it into the floor. `T` is already clear there on the main path
(right after a `CMP/HS` that did not branch); the stand-in-quotient stubs and
the finish subroutine clear it. With `center=0` the macros expand to the
original instructions. The guard clip, near clip and stepper reuse the
private passes; their per-point projections are C.

### Evidence

**HOST** (`tests/test_center.c`, `tests/test_center_equiv.c`; GCC and Clang
strict, ASan/UBSan; counts from the GCC build):
- **Oracle.** An independent 128-bit oracle, written from the header text,
  covers 614,400 batch vertices, 307,200 projected points (39,595 exactly on
  a row boundary), 155,313 stepped points, 38,400 near-clip vertices and
  27,344 guard-clip vertices from 7,476 polygons, the guard clip included
  (its own Sutherland-Hodgman passes with the Q16 planes). Centres are drawn
  from whole pixels plus or minus one unit, plus or minus half a pixel,
  random int32 values, the int32 extremes, and 193 -/+ 1/65536. Points are
  aimed at exact row and column boundaries (with z equal to the focal length
  the quotient is the coordinate itself) and between them. All equal the
  oracle on both entry points, with the `_divu` bytes equal to the portable
  bytes and in-place calls equal to separate ones.
- **Guard rectangle.** 21,888 guard outputs inside all six Q16 planes
  project inside the rectangle without the clamp.
- **Hand cases.** Both of the requester's centres with vertices placed on
  rows 193, 192, 191 and -7, a unit either side of each, and at the top clip
  row (where the residual flips the `TOP` bit). Half-pixel centres on both
  axes and either side of zero. Extreme centres whose sums would wrap 32
  bits.
- **Equivalence with `view-round-1`.** `test_center_equiv.c` runs the whole
  `view-round-1` test suite, unchanged, with every `_trunc` call routed
  through a wrapper. Each wrapper also calls the `_center` form with
  `center16 = 65536 * center` on the same memory and requires the same
  return value and identical bytes. That covers every input of that suite:
  hand cases, relation sweep, path sweeps, argument-rule failures, in-place
  calls and the sweeps of its five negative controls. 471,417 paired calls,
  372,685,880 output bytes, no difference. Control: the centre one unit
  lower makes 18,936 of 72,953 paired calls differ.
- **Argument rules.** Stale centre fields, invalid views and null views
  fail on all ten entry points and write nothing. Guard edges exactly
  `2^30` from the Q16 centre are accepted, `2^30 + 1` rejected, on each
  side. Initialization from the view's own embedded copy is allowed.
- **Controls detected** (oracle mutations, 150 cameras each): centre
  fraction ignored, 117,155 mismatches; centre rounded to a pixel, 66,064;
  y residual sign flipped, 64,774; guard planes from the integer centre,
  4,283; carry and borrow dropped, 134,384.

**MODELLED** (SH-2 model capture, `center.h`): every `_center` entry point,
portable and divider forms, runs on the SH-2 against the harness's own
oracle (fractional centres in 219 of the 256 batch poses on x and 217 on y):
- 262,144 batch vertices (147,352 `NEAR`), 131,072 projected points (7,044
  on an exact row boundary), 202,714 stepped points, 16,384 near-clip quads
  and 16,384 guard polygons (55,282 output vertices, 46,663 of them checked
  inside all six planes, 1 overflow and 2,210 rejected rectangles writing
  nothing). All are exact.
- Equivalence at whole-pixel centres: 2,048 paired `_trunc`/`_center`
  calls over all five paths and both forms, 13,404,208 output bytes,
  identical.

Each timing fixture of `view-round-1` ran through the `_trunc` function
(centre 160, 112) and the `_center` function (centre 160, 112 + 1/65536)
back to back. Medians of 64 trials:

| Path | Unit | Portable, `_trunc` / `_center` | `_divu`, `_trunc` / `_center` |
| --- | --- | ---: | ---: |
| batch (512 vertices) | per vertex | 691.27 / 699.03 | 143.12 / **147.45** |
| project (512 points) | per point | 617.53 / 625.05 | 103.96 / 106.04 |
| edge steps (64 points) | per point | 720.44 / 729.50 | 180.02 / 182.64 |
| near clip (2 crossings) | per quad | 3,128 / 3,206 | 1,696 / 1,776 |
| guard, quad inside | per quad | 4,522 / 4,633 | 2,834 / 2,955 |
| guard, one corner far left | per quad | 6,905 / 7,053 | 4,501 / 4,669 |
| guard, near plus side | per quad | 11,620 / 11,770 | 7,592 / 7,758 |

Each path does the same divisions in both forms. The batch and projection
kernels add three instructions at each of the two places a quotient becomes
a pixel. The clips' per-point
C projections and the centre check cost 20 to 40 cycles per projected point.
No console or emulator run.

### Source-order guard clip with a fractional centre

`sh2z_view_clip_guard_src_center[_divu]` is the `view-guard-src-1` clip
taking a `const sh2z_view_center_t *` where `sh2z_view_clip_guard_src_trunc`
takes a view; every other argument (`max_out` included) is the same. The
`view-guard-src-1` contract is already written with a Q16 centre `(Cx, Cy)`;
here `Cx = center_x16` and `Cy = center_y16`, unrounded:

```
planes       KL = 65536*left - center_x16     KR = 65536*right - center_x16
             KT = center_y16 - 65536*top      KB = center_y16 - 65536*bottom
snap bounds  x >= ceil(KL*z/focal_x)   x <= floor(KR*z/focal_x)
             y <= floor(KT*z/focal_y)  y >= ceil(KB*z/focal_y)
pixels       sx = trunc((center_x16 + trunc(focal_x*x/z)) / 65536)
             sy = trunc((center_y16 - trunc(focal_y*y/z)) / 65536)
```

These are the planes of `sh2z_view_clip_guard_center`, and the pixels are
what `sh2z_view_project_center` writes. They land inside the rectangle
without a clamp, by the argument above. Outcodes come from `sx`, `sy`
against `view.clip`. The validity rules are those of the other `_center`
functions (stale centre fields fail), plus `|65536 * edge - center16| <= 2^30`
for each edge. Order, crossings, snap, `max_out`, the snap failure and every
return value are `view-guard-src-1`'s.

**Implementation.** The clip's static functions moved verbatim from
`sh2zam_view_guard_src.c` into the private header
`src/sh2zam_view_guard_src_core.h`. `sh2zam_view_guard_src.c` includes it,
and its object is byte-identical to `main`'s. The new file
`src/sh2zam_view_guard_src_center.c` includes it too, checks the centre fields
and passes `center_x16`, `center_y16` to the clip unchanged. Every other
object is unchanged, `sh2zam_view_center.o` included.

**HOST** (`tests/test_guard_src_center.c`; GCC and Clang strict,
ASan/UBSan; counts from the GCC build):
- **Equivalence.** The whole `view-guard-src-1` suite (`test_guard_src.c`,
  unchanged) runs with every `_src_trunc[_divu]` call routed through a
  wrapper. Each wrapper also calls the `_src_center` form with
  `center16 = 65536 * center` on the same memory, and requires the same
  return value and identical bytes over the whole 16-vertex buffer. That
  covers its hand cases, argument rules, 600,000 polygons, in-place calls and
  the sweeps of its ten negative controls: 2,746,894 paired calls and
  1,582,210,368 bytes, no difference. Control: the centre 1/65536 lower makes
  38,318 of 154,736 paired calls differ.
- **Oracle.** That suite's own source-clipper oracle, which takes a Q16
  centre for its planes, snap and projection, is compared with both entry
  points at fractional centres. 200,000 road quads (port-like camera, ±900,
  `max_out` 10, centre y `12648447` or `12648449` in half of them) give
  752,026 exact output vertices. 400,000 random polygons (3 to 8 corners,
  half aimed through the Q16 centre at exact pixel boundaries) give
  1,308,231 exact vertices, 250,254 of them on a pixel boundary, with
  43,398 overflows and 7,899 snap failures writing nothing. Divider bytes
  equal portable bytes, and in-place calls equal separate ones. 158,720 road
  quads and 214,130 random polygons differ from the whole-pixel clip of the
  floor view.
- **Hand cases.** At `12648447` and `12648449` a corner on the y axis lands
  on rows 192 and 193; `_src_trunc` at 193 puts it on 193 both times. Left
  crossings snap to `x >= 65536 * left - center_x16` for x fractions 0,
  1/65536 and 1/2.
- **Argument rules.** A stale floor, a null view, `max_out` 2 or 17 and an
  edge `2^30 + 1` from the Q16 centre are rejected with nothing written
  (`2^30` exactly, and `2^30 - 1` with a fraction, are accepted). Centres at
  the int32 extremes clip exactly.
- **Controls detected** (oracle mutations, 60,000 polygons each): fraction
  dropped, 131,610 mismatches; centre rounded to a pixel, 104,693; y fraction
  subtracted, 108,731.

**MODELLED** (`center.h`): both entry points ran on the SH-2 against the
harness's own oracle (the `guardsrc.h` oracle rewritten with the Q16
planes, snap and projection). 16,384 random polygons and 8,192 road quads
(centre y `12648447` or `12648449` on three in four) gave 69,946 exact output
vertices (26,795 on road quads; 14,447 on a pixel boundary). 1,716 overflows,
986 snap failures and 2,852 rejected rectangles wrote nothing. 12,712
polygons differ from the whole-pixel clip of the floor view. At whole-pixel
centres, 1,024 paired `_src_trunc`/`_src_center` calls wrote identical bytes.

`guardsrc.h`'s timing fixtures ran through `_src_trunc` (centre 160, 112) and
`_src_center` (centre 160, 112 + 1/65536) back to back. Medians of 64 trials,
cycles per call:

| Fixture | Portable, `_src_trunc` / `_src_center` | `_divu`, `_src_trunc` / `_src_center` |
| --- | ---: | ---: |
| quad inside (attrs 2, `max_out` 16) | 4,281 / 4,287 | 2,873 / 2,879 |
| one corner far left | 6,763 / 6,770 | 4,576 / 4,583 |
| near plus side | 9,780 / 9,789 | 6,866 / 6,875 |
| road quad from behind the camera (attrs 0, `max_out` 10) | 12,542 / 12,537 | 8,864 / 8,870 |

Both forms do the same divisions. The divider forms cost 6 to 9 cycles more
per call (the centre check); the portable medians differ by -5 to +9. No
console or emulator run.

### Caller guidance

- **Pass the source centre as it is.** Build the plain view as for
  `_trunc` (rotation, `sh2z_view_place`, focal, clip), then call
  `sh2z_view_center_init` with the source's Q16 centre. Do not round it.
- **Use one rule per frame.** Use the `_center` forms for the batch, edge
  steps and both clips together, with the same centre view. Where the source
  clips in its own order, use `sh2z_view_clip_guard_src_center[_divu]` with
  the source's pass limit instead of `sh2z_view_clip_guard_center`. The near clip
  copies corners as given, and the merge planner reads whatever `sx`, `sy`
  it is given.
- **Guard rectangle.** Pass the same rectangle as before. Its edges are
  whole pixels, and the planes come from the Q16 centre. Use
  `view.clip` of the centre view when the rectangle is the clip rectangle.
- **What exactness covers.** The pixel rule only, as for `view-round-1`.

## View from a camera (`view-camera-1`)

A renderer that keeps its camera in fixed point can build the view for
these kernels without floating point. `sh2z_view_from_camera30` builds a
`view-center-1` centre view from a camera basis in Q2.30, an eye in Q20.12
and a lens. It replaces the usual per-frame float chain in one integer
pass: a look-at matrix, a frustum, their product and packing, a
float-to-fixed adapter, and `sh2z_view_place` with a Q16.16 eye.
`sh2z_view_place_q12` is `sh2z_view_place` with a Q20.12 eye.

```c
bool sh2z_view_place_q12(sh2z_view_t *v, const int32_t eye_q12[3]);

typedef struct {
    int32_t fov_q16;                /* full horizontal fov, degrees, Q16.16 */
    int32_t scale_x16, scale_y16;   /* pixels per unit of tangent, Q16.16 */
    int32_t center_x16, center_y16; /* view axis on screen, Q16.16 pixels */
    int32_t near_q12, far_q12;      /* Q20.12 model units */
    int16_t clip[4];                /* inclusive pixel rectangle */
} sh2z_view_lens_t;

bool sh2z_view_from_camera30(sh2z_view_center_t *out, const int32_t basis_u30[9],
    const int32_t eye_q12[3], const sh2z_view_lens_t *lens);
```

### Why a Q20.12 eye

`sh2z_view_place` takes the eye in Q16.16 model units, so it overflows at
32,768 units. A world whose geometry fills the int16 range has no headroom
there: an eye computed as `eye * 65536` from a float is undefined at the
edge of the world. In Q20.12 the eye can be 16 times further out. The
translation is still computed exactly in 64 bits from the quantized
rotation:

    t[i] = -floor((r[i][0]*eye[0] + r[i][1]*eye[1] + r[i][2]*eye[2]) / 4096)

That is the same value `sh2z_view_place` computes from `16 * eye` wherever
`16 * eye` fits. The host test checks that equality on 66,156 random views.
The only limit left is the view's own row bound
(`32768 * sum|r| + |t| <= INT32_MAX`). For an orthonormal rotation, it
holds for every eye within 74,000 units of the origin. The corner of the
int16 cube is 56,755 units away.

### Contract

- **Basis.** Three rows of three, row-major, Q2.30. Row 0 is screen +x
  (right), row 1 screen +y (up), row 2 the viewing direction. A camera that
  keeps {forward, up, side = up x forward} passes {-side, up, forward}.
  `r = round(basis / 65536)`, ties away from zero, to Q1.14. A value at or
  above `2^31 - 32768` would round to 32768, so the call fails.
- **Field of view.** The half angle is rounded to a 4096-step turn, ties
  away from zero, the way a float frustum built from a 4096-entry sine table
  rounds it: `h = floor((fov_q16 + 5760) / 11520)`. `1 <= h <= 1023` is
  required.
- **Focal length.** `focal = floor((scale * C[h] + 2^19) / 2^20)`, Q16.16.
  `C[h] = round(cot(pi h / 2048) * 2^20)` is a 1,024-entry, 4 KB table,
  generated exactly by `saturn/tools/view_camera_cot.py`. There is no
  division. An off-axis frustum, or a vertical extent different from the
  horizontal one, goes into `scale_y` and the centre. For a viewport of
  half-size `(sx, sy)` and a frustum of `fov_scale_x` by `fov_scale_y`
  units whose centre is `(fx, fy)`:

      scale_x  = sx                         centre_x = translate_x - 2 sx fx / fov_scale_x
      scale_y  = sy fov_scale_x / fov_scale_y   centre_y = translate_y + 2 sy fy / fov_scale_y

- **Centre, depth, clip.** The centre is kept as given, any int32 (as in
  `view-center-1`). `near_z = 4 * near_q12` and `far_z = 4 * far_q12`, with
  `0 < near < far <= INT32_MAX / 4`. The clip rectangle is inclusive.
- **Failure.** On failure the call returns false and leaves `*out`
  unchanged. It fails on:
  - a null pointer;
  - `h` out of range;
  - a scale that is not positive;
  - a focal length of 0 or above `INT32_MAX`;
  - a bad depth or clip;
  - a rotation value that does not fit;
  - an eye too far for the view (`t` does not fit int32, or the row bound
    fails).

  There is no other failure. The result is a valid centre view: the
  `_center` batch, projection, clips and edge steps accept it.

No floating point, divider, libc or global state is used. Host and SH-2
builds write identical bytes.

### Accuracy (HOST)

**Analytic bound.** Take a unit orthonormal basis given to within `J` per
element (Q2.30 rounding plus caller error). Then each rotation element is
within `2^-15 + J` of the true basis. The translation adds less than
`2^-14` units. The focal length is within `scale * 2^-21 + 2^-17` pixels.

For a point at depth `z`, distance `k z` from the eye, projecting `X` pixels
from the centre, the continuous projection (before the final truncation)
is within

    (|X| + f) (3 (2^-15 + J) k + 2^-14 / z) + |X| df / f

of the exact one. `tests/test_camera.c` checks this bound for every point
it projects, with `J = 8.5 * 2^-30`. The largest error measured was 0.512
of the bound.

**Measured against the float pipeline.** `tests/camera_float_ref.h` writes
out the float chain in binary32, every operation in source order:
- a 4096-entry sine table from a Taylor series;
- an orthonormalised camera basis;
- a look-at matrix and an off-axis frustum;
- a float-to-fixed adapter (Q2.30 rotation, Q16.16 focal and centre);
- `sh2z_view_place` with `eye * 65536` truncated.

15,000 synthetic poses were run:
- eye anywhere in the world or outside it, a random forward and up;
- field of view 40 to 110 degrees;
- frustum centre up to ±80 pixels, fractional;
- near/far 16/8192 or 512/16384;
- 160 x 120 viewport;
- half the poses with ±8 LSB of basis error.

Each pose has up to 96 int16 points spread over the screen plus a margin of
about 16 to 21 pixels, at log-uniform depth. The tables give the continuous
projected error, the larger of x and y, in pixels, as p50 / p99 / max.

| Eye | Points | This view vs exact | Float pipeline vs exact | This view vs float |
| --- | ---: | --- | --- | --- |
| near origin, \|e\| <= 2,048 | 288,000 | 0.0051 / 0.0155 / 0.0278 | 0.0051 / 0.0155 / 0.0278 | 0.00002 / 0.00008 / 0.0129 |
| world, \|e\| <= 32,767 | 574,140 | 0.0052 / 0.0152 / 0.0302 | 0.0053 / 0.0165 / 0.0392 | 0.0002 / 0.0144 / 0.0415 |
| world edge, 30,000..32,767 | 274,124 | 0.0052 / 0.0155 / 0.0275 | 0.0055 / 0.0189 / 0.0466 | 0.0006 / 0.0173 / 0.0494 |
| outside, one axis 32,768..42,000 | 235,182 | 0.0051 / 0.0148 / 0.0276 | fails (Q16.16 eye overflows) | — |

| Eye | Integer pixels differing from the float pipeline | This view vs truncated exact point | Float pipeline vs truncated exact point |
| --- | --- | --- | --- |
| near origin | 12 of 288,000, by 1 | 2,359 | 2,359 |
| world | 1,007 of 574,140, by 1 | 4,691 | 4,850 |
| world edge | 888 of 274,124, by 1 | 2,255 | 2,489 |
| outside | — | 1,870 of 235,182 | — |

The integer columns use the `view-center-1` rule.

The test asserts:
- at most 0.05 px from exact;
- at most 0.06 px from the float pipeline;
- integer pixels differ from the float pipeline by at most 1;
- no failure inside 74,000 units;
- the float pipeline fails on every pose beyond 32,767.

Toward the edge of the world this view is closer to exact than the float
pipeline. A binary32 eye there has 1/512-unit steps, and its look-at
translation rounds.

**Matrix elements against the float pipeline.**
- Rotation: within 1 LSB (Q1.14), and within 0.5 LSB of exact plus the basis
  error.
- Focal length: within 10 LSB (Q16.16, 1.5e-4 px). It is within 7.6e-5 px
  of `160 cot(h)`.
- Centre: within 2 LSB (Q16.16).
- Translation: differs by up to 63,147 LSB (3.9 units) at the world edge.
  This is not an error. A 1-LSB rotation difference times a 32,767-unit eye
  moves `t` by up to 2 units per element, and the rotation moves every point
  by the same amount. The projections above include both.

### Implementation and evidence

`src/sh2zam_view_camera.c` is portable C on both host and SH-2. It is a
separate object, so no existing object changes. Each row is branch-free:
- three roundings;
- a 64-bit dot product, as `DMULS.L`;
- a floor and fit test on the two halves;
- the row bound, computed in 32 bits with the carry.

A single failure word is tested once. There are no 64-bit shifts or libgcc
calls (`nm -u` is empty).

- **HOST.** `camera_exact` passes under GCC and Clang with `SH2Z_STRICT=ON`
  and under ASan/UBSan, with 4,074,767 checks:
  - every table entry, re-derived from `cosl`/`sinl`;
  - every field of view from 5,000 to 11,791,000 (Q16.16);
  - 600,000 random calls mixing valid and invalid inputs against the
    independent oracle `tests/camera_oracle.h` (128-bit integers, no shared
    code or table);
  - row-bound, `t = -2^31`, focal-overflow, rounding-tie, centre, depth and
    clip boundaries;
  - 300,000 `sh2z_view_place_q12` calls;
  - the accuracy study above.

  Eight negative controls (mutated oracles) are each detected: floor
  rounding, truncated `t`, truncated half angle, truncated focal, the next
  table entry, truncated centre floor, a strict row bound, and Q16-scaled
  depth.
- **MODELLED.** [`benchmarks/camera-model`](benchmarks/camera-model/README.md)
  ([results](benchmarks/camera-model/results.json), captured by
  [CI run 44224191560](https://github.com/lamb2k/SH2ZAM/actions/runs/44224191560)
  on `3da9af1`) runs the SH-2 object code in the Yabause interpreter and checks it before
  timing:
  - `sh2z_view_from_camera30` and `sh2z_view_place_q12` on 23,075 calls
    each (every field, or untouched output on failure), against the oracle;
  - the float chain compiled for SH-2 with GCC soft-float, against the same
    chain on the host, bit for bit (1,016 poses).

  Over 64 world poses, mean modelled cycles per call:

| Entry | Cycles | Instructions |
| --- | ---: | ---: |
| `sh2z_view_from_camera30` | 691 | 655 |
| `sh2z_view_place_q12` | 411 | 394 |
| float chain: frustum, look-at, product, packing, adapter, centre view (soft-float) | 87,395 | 75,645 |
| float-to-fixed adapter and centre view only | 25,762 | 22,431 |

The float chain is 126 times the cost. Its soft-float is libgcc's, not any
particular port's float library, so treat the ratio as indicative.

The planned cost was 250 to 350 cycles, and 691 is above it. The work is
mostly the exact contract:
- 9 roundings;
- three 64-bit dot products;
- the per-row validity bound with `sum|r|`.

A hand-scheduled SH-2 version of the rows is estimated at about 400 cycles.
That saves about 10 µs per frame for one call, so it was not written. No
console or emulator run.

### Caller guidance

- **Keep the eye in Q20.12** and pass it as is. Do not convert to Q16.16.
- **Basis order.** Pass rows {right, up, forward}. For a basis {x forward,
  y up, z = y × x}, that is {-z, y, x}. A basis from a fixed-point
  orthonormalisation within a few LSB of Q2.30 is far inside the bound.
- **Lens from a frustum.** Use the formulas above. For a 160 x 120 viewport
  of a 320 x 240 frustum, `scale_x = scale_y = 160`. The centre is then
  `(translate_x - fx, translate_y + fy)`, plus any line crop of the target
  screen, kept fractional.
- **Depth fog or perspective constants** that a frustum matrix also
  provides depend only on near and far. Compute them once per change.
