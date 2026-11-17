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
| Console or emulator timing | Unrun |

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
