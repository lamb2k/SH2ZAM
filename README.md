<div align="center">
  <h1><strong>SH2ZAM!</strong></h1>
  <p>
    Fixed-point math for the Sega Saturn's SH-2 CPUs
    <br />
    <a href="saturn/README.md"><strong>Explore the docs »</strong></a>
    · <a href="exhibits/README.md"><strong>Interactive math lab »</strong></a>
    <br /><br />
    Saturn-native · Fixed-point · Portable
  </p>
  <p>
    <a href="https://github.com/lamb2k/SH2ZAM/actions/workflows/sh2zam-host.yml"><img src="https://github.com/lamb2k/SH2ZAM/actions/workflows/sh2zam-host.yml/badge.svg?branch=main" alt="Host correctness"></a>
    <a href="https://github.com/lamb2k/SH2ZAM/actions/workflows/sh2-cross.yml"><img src="https://github.com/lamb2k/SH2ZAM/actions/workflows/sh2-cross.yml/badge.svg?branch=main" alt="SH-2 cross build"></a>
    <a href="https://github.com/lamb2k/SH2ZAM/actions/workflows/exhibits.yml"><img src="https://github.com/lamb2k/SH2ZAM/actions/workflows/exhibits.yml/badge.svg?branch=main" alt="Interactive math lab"></a>
  </p>
</div>
<hr>

# Overview

<p align="center">
  <img src="doc/sh2zam-gainz.webp" width="480" alt="Sega Saturn with turquoise flexing arms above GAINZ">
</p>

`SH2ZAM` is a general-purpose fixed-point math and linear algebra library for the Sega Saturn. Inspired by [SH4ZAM](https://github.com/gyrovorbis/sh4zam), it brings the same emphasis on practical game mathematics to a different machine: Q16.16 arithmetic, turn-based angles, vectors, matrices and quaternions for the Saturn's SH-2 processors.

**Original project credit:** SH4ZAM by **Falco Girgis and SH4ZAM Contributors**. Their Git history, source, [MIT license](LICENSE), copyright notices and [acknowledgements](README.SH4ZAM.md#acknowledgements) are preserved. SH2ZAM is an independent adaptation, not an official SH4ZAM release or a claim of upstream/Sega endorsement.

> [!IMPORTANT]
> **Experimental Saturn foundation, not full SH4ZAM parity.** Host correctness tests, sanitizers, big-endian SH-2 builds and isolated execution of the compiled SH-2 kernels have passed. Physical-Saturn performance and production qualification remain unverified. See the [validation record](saturn/VALIDATION.md).

# Features

- C11 API with optional C++11 conveniences and C/C++ interoperability.
- Signed Q16.16 arithmetic with explicit saturation and rounding contracts.
- An SH-2 `dmuls.l` multiplication path and a portable reference backend.
- An exact batch-transform fast path with one rounding step per row and an automatic full-range fallback.
- Exact quaternion rotation batches that share normalization and matrix construction across all vectors.
- Caller-owned static AABB indexes for exact moving-sphere queries, with conservative pruning and bulk acceptance.
- Prepared immutable meshes with cached coordinate bounds and a clamp-free batch kernel when the matrix proof succeeds.
- Opt-in exact coordinate reuse for repetitive meshes, with caller-owned tables rebuilt for each matrix.
- A checked eight-vertex loop that shares coordinate-table loads while preserving original output order.
- Opt-in fused SH-2 `MAC.L` and `DMULS.L` candidates with audited loops and a five-kernel console benchmark.
- No heap allocation, runtime floating point or mutable global state in the math core.
- Wide-intermediate products and normalization that handle small and large inputs.
- Exact square-root, normalization, matrix, quaternion, trig, projection and collision optimizations measured as compiled SH-2 code.
- Caller-owned transform stacks and batch transforms without hidden CPU scheduling.
- Installable CMake targets, a Yaul cross-toolchain file and runnable examples.
- Host regression/oracle tests, sanitizer checks and actual SH-2 build checks.

# APIs

- **Scalar:** add/subtract/multiply/divide, muldiv, rounding, reciprocal, square root, reciprocal square root, lerp and smoothstep.
- **Trigonometry:** sine/cosine, sincos, tangent, atan/atan2, asin/acos and shortest-angle interpolation.
- **Vectors:** 2D, 3D and 4D arithmetic, dot products, lengths, normalization and interpolation; 2D/3D cross products.
- **Matrices:** 2×2, 3×3, affine 3×4 and 4×4, transformations, transpose and alias-safe multiplication.
- **Camera:** right-handed look-at, rigid inverse, perspective and clip-to-screen helpers.
- **Quaternions:** multiplication, normalization, conjugation, axis-angle, nlerp, matrix conversion, vector rotation and exact rotation batches.
- **Game helpers:** AABB/sphere intersection, static AABB indexes for sphere queries, transform stacks and batch-work partitioning.

Complex/FFT, a general libm replacement, accelerated memory routines and the SH-4 XMTRX register bank are **not** implemented. The C++ facade is smaller than the C API. See the [capability comparison](saturn/PARITY.md).

# Usage

## C11

Include `sh2zam/sh2zam.h` and link the `sh2zam` library:

```c
#include <sh2zam/sh2zam.h>

int main(void) {
    sh2z_vec3_t position = {sh2z_from_int(3), sh2z_from_int(4), 0};
    sh2z_vec3_t direction = sh2z_vec3_normalize(position);
    sh2z_mat34_t world = sh2z_mat34_rotation_y(sh2z_angle_from_degrees(30));
    world.m[0][3] = sh2z_from_int(10);
    sh2z_vec3_t moved = sh2z_mat34_transform_point(&world, position);
    (void)direction;
    (void)moved;
    return 0;
}
```

## C++11

Include `sh2zam/sh2zam.hpp` for the small fixed-point/vector facade. The full C API remains available:

```cpp
#include <sh2zam/sh2zam.hpp>

int main() {
    using sh2z::fixed;
    sh2z::vec3 position(fixed::from_int(3), fixed::from_int(4), fixed{});
    sh2z::vec3 direction = position.direction();
    auto world = sh2z_mat34_rotation_y(sh2z_angle_from_degrees(30));
    auto moved = position.transformed(world);
    (void)direction;
    (void)moved;
    return 0;
}
```

# Installation

## CMake

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DSH2Z_STRICT=ON
cmake --build build --parallel
ctest --test-dir build --output-on-failure
cmake --install build --prefix /your/install/prefix
```

The root build builds **SH2ZAM**, not the inherited Dreamcast library. For library-only builds, set `-DSH2Z_BUILD_TESTS=OFF -DSH2Z_BUILD_EXAMPLES=OFF`. `cmake -S saturn ...` is also supported. Installed consumers can use `find_package(SH2ZAM 0.2 CONFIG REQUIRED)` and `SH2ZAM::sh2zam`.

## Transform a vertex batch

```c
sh2z_mat34_t world = sh2z_mat34_rotation_y(SH2Z_ANGLE_QUARTER);
sh2z_vec3_t vertices[3] = {
    {SH2Z_ONE, 0, 0}, {0, SH2Z_ONE, 0}, {0, 0, SH2Z_ONE}
};
sh2z_vec3_t transformed[3];
sh2z_mat34_transform_points(transformed, &world, vertices, 3);
```

The batch API checks the matrix once, reuses its coefficients and accumulates each
row directly before rounding. Its bound covers every Q16.16 input coordinate, so
there is no vertex prescan or caller-supplied range promise. Matrices outside the
bound use the general implementation. Both paths return identical fixed-point
results, including saturation and negative rounding. Exact in-place transforms are
supported. See [the bound, API contract and implementation](saturn/BATCH_TRANSFORMS.md).

For a mesh reused across frames, cache its coordinate bounds once and qualify
each new matrix before executing the prepared batch:

```c
sh2z_mesh_view_t mesh;
sh2z_transform_plan_t plan;
if (sh2z_mesh_view_init(&mesh, vertices, 3) &&
    sh2z_transform_plan_init(&plan, &mesh, &world)) {
    sh2z_transform_plan_execute(transformed, &plan);
}
```

Keep the source vertices immutable until the view and its plans are rebuilt.
Prepared output must be separate from the entire source mesh. The plan snapshots
the matrix; qualifying a new matrix does not rescan the vertices. Eligible plans
prove that products, partial sums and outputs fit, so the hot loop can omit
clamps. Other plans preserve the exact general fallback. Experimental native
accumulator kernels are explicit alternatives, pending physical-console
qualification; see [the fused-kernel audit](saturn/benchmarks/FUSED_SH2.md).

Meshes with repeated coordinates can also opt into
[`sh2zam/reuse_experimental.h`](saturn/include/sh2zam/reuse_experimental.h).
It computes distinct coordinate contributions once per matrix and combines them
for every original output vertex. The prepared proof preserves exact rounding
and full-range fallback. This trades caller-owned table/index memory for less
repeated multiplication; the [coordinate-reuse guide](saturn/COORDINATE_REUSE.md)
documents selection, setup cost and lifetimes.

## Share rotation setup and index static geometry

[`sh2zam/rotation_batch.h`](saturn/include/sh2zam/rotation_batch.h) adds
`sh2z_quat_rotate_vectors`: apply one quaternion to a complete vector batch,
sharing normalization and matrix construction while preserving every scalar
rotation result. Exact in-place operation is supported. The explicit `_mac`
alternative includes input scanning and qualification for the existing optional
native kernel, with the ordinary exact batch as its fallback.

[`sh2zam/spatial.h`](saturn/include/sh2zam/spatial.h) adds
`sh2z_aabb_index_build` and `sh2z_aabb_index_query_sphere`. Build into caller-owned
storage, then query unchanged boxes with different spheres to obtain exact
original-index hit bitsets. Keep boxes and built nodes immutable until rebuilding;
build scratch can be reused afterward. Pruning and bulk acceptance depend on
the query pattern, and index construction must be included when evaluating cost.
See the [API examples, memory budget and workload comparisons](saturn/benchmarks/workload-model/README.md).

## Integer kernels for lifted 16-bit game code

[`sh2zam/m68k.h`](saturn/include/sh2zam/m68k.h) provides 68000-exact `MULS.W`/`MULU.W`/`DIVS.W`/`DIVU.W`
(condition codes, overflow rule, divide-by-zero report; the divides use `DIV1`, not the on-chip divider).
[`sh2zam/box2.h`](saturn/include/sh2zam/box2.h) does exact 2D integer box overlap with packed bitset results,
and [`sh2zam/cell16.h`](saturn/include/sh2zam/cell16.h) mirrors, offsets and clips batches of 16x16 cells.
All are host-exact tested and costed in the SH-2 instruction model (modelled, no console run). See the
[integer kernels guide](saturn/INTEGER_KERNELS.md).

## Shading kernels

[`sh2zam/shade_experimental.h`](saturn/include/sh2zam/shade_experimental.h) (contract tag `shade-1`)
lights int8 normals with a directional light and a reflection table, packs 5:5:5 Gouraud colours,
culls faces against an integer eye with a conservative margin, and looks up a threshold depth fog
through a bucket index, each reproducing an existing renderer's integer rules exactly with
reciprocal multiplies in place of its divisions. Host-exact tested against an independent oracle
and costed in the SH-2 instruction model (modelled, no console run). See
[the shading guide](saturn/SHADE.md).

Contract tag `shade-2` adds `sh2z_shade_fog_gouraud`: a road renderer's per-piece fade lookup,
fog variant and 5:5:5 colour in one hand-written SH-2 call with prepared fog and colour handles
(fades, variants and solid colours exact; textured colours within +1 LSB of shade-1), and
[`sh2zam/cdiv_experimental.h`](saturn/include/sh2zam/cdiv_experimental.h), division by constants
through magic-number multiplies with exhaustively checked domains. See
[the fused shade guide](saturn/SHADE_FUSED.md).

Contract tag `vehicle-kernels-1` adds per-draw kernels for an indexed mesh renderer:
`sh2z_shade_screen_facing` (exact projected-quad facing), `sh2z_shade_light_normals_indexed`
and `sh2z_view_batch_divu_indexed` (only the referenced normals and positions, byte-identical
to the full batches) and `sh2z_shade_gouraud_approx` (each channel equal to shade-1's or one
less, proven), all with hand-written SH-2 loops. See
[the vehicle kernels guide](saturn/VEHICLE_KERNELS.md).

## Variable-length decoding

[`sh2zam/vlc_experimental.h`](saturn/include/sh2zam/vlc_experimental.h) (contract tag `vlc-1`)
decodes prefix-coded byte symbols from a 16-bit-unit bit stream: a bounded marker-bit reader
that never reads past the stream and reports truncation, a length-limited canonical code
builder (from frequencies or from code lengths), a table check for untrusted tables, and a
two-level table decoder with a hand-written SH-2 kernel (`DMULU.L` in place of variable
shifts). Host-exact tested against an independent oracle under ASan/UBSan and costed in the
SH-2 instruction model (modelled, no console run). See [the vlc guide](saturn/VLC.md).

## 12-bit-angle trig and divider quotients

[`sh2zam/trig12_experimental.h`](saturn/include/sh2zam/trig12_experimental.h) (contract tag `trig12-1`)
gives sine and cosine in Q2.30 for 12-bit angles from a 1,025-entry quarter table, generated
offline from a common series construction for 4096-entry sine tables and checked in CI against
its generator, and a rounded Q16.16 tangent with saturating poles. The `trig12-1` section of
[`sh2zam/divu_experimental.h`](saturn/include/sh2zam/divu_experimental.h) adds inline
`num * 2^f / den` quotients (Q16.16, Q2.30, any constant `f`) and reciprocals through the SH-2
division unit, with start/finish forms that overlap the division with other work.
Host-exact tested against an independent oracle under ASan/UBSan and costed in the SH-2
instruction model (modelled, no console run). See [the trig12 guide](saturn/TRIG12.md).

## Unit vectors and bases

[`sh2zam/unit30_experimental.h`](saturn/include/sh2zam/unit30_experimental.h) (contract tag
`fixed-unit-1`) keeps directions in Q2.30. It provides:
- an inline rounded multiply, a single-rounding `MAC.L` dot product and a cross product,
  all saturating;
- a reciprocal square root from a seed table and two multiply-only Newton steps (within
  2 LSB);
- a normalisation of any `int32_t` scale in hand-written SH-2 assembly (within 2 LSB);
- an exactly rounded 64-bit integer square root;
- the forward-first and up-first cross-product orthonormalisations of a 3x3 basis.

Host and SH-2 results are bit-identical. Host-tested against an independent oracle and
costed in the SH-2 instruction model (modelled, no console run). See
[the unit-vector guide](saturn/UNIT30.md).

## View from a fixed-point camera

`sh2z_view_from_camera30` in [`sh2zam/view_experimental.h`](saturn/include/sh2zam/view_experimental.h)
(contract tag `view-camera-1`) builds a fractional-centre view for the view kernels from a Q2.30
camera basis, a Q20.12 eye and a lens (field of view, scales, centre, depth, clip) in one integer
pass, replacing a float look-at, frustum and float-to-fixed adapter. `sh2z_view_place_q12` places a
view at a Q20.12 eye, which does not overflow at the edge of an int16 world as a Q16.16 eye does.
Host-exact tested against an independent oracle, within 0.05 px of the exact projection over
15,000 synthetic poses, and costed in the SH-2 instruction model (modelled, no console run). See
[View from a camera](saturn/VIEW_BATCH.md#view-from-a-camera-view-camera-1).

## Model pose from binary32 state

`sh2z_pose_from_f32` in [`sh2zam/view_experimental.h`](saturn/include/sh2zam/view_experimental.h)
(contract tag `pose-1`) builds a model's view, model-space camera position, Q1.14 basis and
plane-culling data from float model state (basis, scales, position, camera) with no
floating-point arithmetic: each float is decoded with integer instructions and the binary32
difference, product and reciprocal are emulated exactly, so every output equals the float
procedure to 0 LSB. Hand-written SH-2 path (`MAC.L`, `DMULU.L` shifts, `DIV1`), portable C
elsewhere. Host-exact tested against an independent binary32 oracle under ASan/UBSan and
costed in the SH-2 instruction model (modelled, no console run). See
[the pose guide](saturn/POSE.md).

## Git submodule

```sh
git submodule add https://github.com/lamb2k/SH2ZAM.git external/SH2ZAM
```

```cmake
add_subdirectory(external/SH2ZAM)
target_link_libraries(your_game PRIVATE SH2ZAM::sh2zam)
```

## Sega Saturn / Yaul

Use a bare-metal SH-2 toolchain such as Yaul's `sh2eb-elf-gcc`:

```sh
cmake -S . -B build-sh2 \
  -DCMAKE_TOOLCHAIN_FILE="$PWD/saturn/toolchains/yaul-sh2.cmake" \
  -DSH2Z_BUILD_TESTS=OFF -DSH2Z_BUILD_EXAMPLES=OFF
cmake --build build-sh2 --parallel
```

The library does not replace Yaul, SGL, Jo Engine or a renderer. SDK/runtime compatibility outside the documented Yaul build must be checked separately. See [hardware integration](saturn/HARDWARE.md).

# Examples

| Example | Description |
|---|---|
| [Interactive math lab](exhibits/README.md) | Three looping scenes with SH-2 model comparisons against independent fixed-point C or software float, precision checks and evidence export. |
| [Sprite orbit](saturn/examples/orbit.c) | Integer-only orbit coordinates for driving a sprite. |
| [Cube transforms](saturn/examples/cube.c) | Eight vertices through model, view, perspective and screen-coordinate transforms. |
| [Batch jobs](saturn/examples/batch.c) | Two disjoint vertex jobs compared with serial execution; no slave-CPU launch. |
| [Yaul diagnostic](saturn/examples/yaul-smoke/) | Buildable Saturn diagnostic application and disc image. |

Host example commands and Saturn-disc build instructions are in [the examples guide](saturn/examples/README.md). The cube example does not submit VDP1 polygons or clip polygons.

# Showcase

## F-Zero X Saturn

[F-Zero X Saturn](https://github.com/lamb2k/fzero-x-saturn) is an early native
Saturn port foundation with a shared C renderer using SH2ZAM for Saturn and the
web. It includes a synthetic test course and an importer for user-supplied
course data. Original gameplay and retail assets remain to be integrated.

[Open the web workbench](https://lamb2k.github.io/fzero-x-saturn/).

## Math lab

The [math lab](https://lamb2k.github.io/SH2ZAM/) shows what changes when a
Saturn developer adopts SH2ZAM. Choose **Ring world**, **Cube field** or
**Twisted orbit**, then change mesh detail to compare the same workload with
and without the library. Every performance figure comes from executing compiled
SH-2 code in the [instruction model](saturn/benchmarks/sh2-model/README.md).
The browser renders a live precision comparison; it does not time the kernels.

Choose an independent **plain C Q16.16** baseline for existing fixed-point code,
or **software float** for a floating-point port. Compare it with the prepared
batch API or the optional native MAC.L kernel. The lab reports modeled cycles
per mesh and per vertex, speedup and the percentage of transform cost saved.
Matrix qualification is included once per batch, and every output is checked
against an independent integer oracle. Slower results remain visible.

The native MAC.L path is an explicit experimental API, not the default prepared
dispatcher. It requires proven bounds and SR.S clear; the library preserves its
fallback when those conditions do not hold. The standalone fixed-point baseline
is intentionally bounded and reports unsupported inputs instead of silently
overflowing. The same quantized vertices, matrices and output writes are used
for every measured path.

The model excludes cache, bus and pipeline effects. Its recorded cycle counts
are not converted to Saturn milliseconds or game FPS. Physical-console results
remain unmeasured until a capture exists. Source, build and fixture identities
are included in the lab's downloadable evidence. See [the measurement
contract](exhibits/README.md) and [validation record](saturn/VALIDATION.md).

The **Math library improvements** section covers 48 existing APIs across bounded,
tiny and full-range inputs. It compares SH2ZAM at the historical `10786b6` baseline
with the optimized library, using the same SH-2 compiler, fixtures and output
checks. This is a separate library-revision comparison; it does not relabel the
old library as independent engine code. Select a math family and operation to
inspect all three profiles, then export the raw samples and provenance. See
[the exact math changes and results](saturn/MATH_OPTIMIZATIONS.md).

The **Optimize the whole workload** section covers 44 cases with seven changing
input samples each. Rotation comparisons include repeated scalar calls, an
existing matrix batch that already shares setup, and the existing prepared
MAC.L composition. Spatial comparisons include the prior predicate API and two
independent C loops: packed output words and per-hit bit writes. The default
comparison uses each case's fastest recorded control. Quaternion setup, scans
and qualification, or a complete index build plus all 128 sphere queries are
included in their respective totals. Exact outputs, small cases and slower
results remain visible; a 10× goal is not a universal performance promise.
This suite uses request-start `3374d77` separately from the historical math
baseline. See the [workload contracts and reproduction guide](saturn/benchmarks/workload-model/README.md).

There are no independently verified production adopters or Saturn speedup claims listed yet. Share measured integrations through this repository's issues.

# Conventions

| Convention | SH2ZAM default |
|---|---|
| Scalars | Signed Q16.16; one is 65536; saturating arithmetic |
| Angles | Unsigned 16-bit turns; quarter-turn is 16384 |
| Coordinates | Right-handed; camera looks along −Z; NDC depth [−1, 1] |
| Matrices | Row-major, column vectors; `A * B` applies B first |
| Quaternions | Stored in x, y, z, w order |
| Rounding | Multiply/dot round down once; division truncates toward zero |
| Edge cases | Division by zero saturates by sign; `0/0 = 0`; zero vector normalizes to zero; zero quaternion to identity |

These differ from SH4ZAM's float types, layouts and API contracts. Port calls explicitly; changing an include path is not sufficient. The application owns interrupt preservation, shared-memory/cache synchronization, DMA and VDP1 command submission.

# Community

Use [SH2ZAM issues](https://github.com/lamb2k/SH2ZAM/issues) for Saturn-specific bugs, examples and hardware measurements. Read [CONTRIBUTING.md](CONTRIBUTING.md) before contributing. Dreamcast users should continue to use [upstream SH4ZAM](https://github.com/gyrovorbis/sh4zam); submit reusable upstream improvements there.

# Acknowledgements

- **Falco Girgis and SH4ZAM Contributors:** the original project, its mathematical scope and the inspiration for this library and README structure. The [full inherited acknowledgements](README.SH4ZAM.md#acknowledgements) remain intact.
- **Roberto Daniel Alves Duarte / SaturnMath++** and **Israel Jacquez / Yaul:** research and tooling references for Saturn development; no claim that their routines were copied into SH2ZAM.
- **Yabause Contributors:** the unmodified, pinned SH-2 interpreter used by the separately licensed optional [instruction-model harness](saturn/benchmarks/sh2-model/README.md).
- **SH2ZAM Contributors:** the independent Saturn implementation, tests, integration examples and interactive exhibit.

Original MIT notices are preserved. See [LICENSE](LICENSE) and [NOTICE.md](NOTICE.md). New Saturn code lives in `saturn/`; the inherited SH4ZAM sources remain as provenance/reference, not active Saturn dependencies.
The optional `saturn/benchmarks/sh2-model/` harness and the GPL-marked
`capture.h` / `collect.py` extensions under `saturn/benchmarks/math-model/` and
`saturn/benchmarks/workload-model/` are GPL-2.0-or-later and download Yabause
separately. They are not linked into the MIT math library or browser module.
