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

[`sh2zam/vlc2_experimental.h`](saturn/include/sh2zam/vlc2_experimental.h) (contract tag `vlc-2`)
decodes one symbol at a time with a table the caller picks per symbol, for context-modelled
decoders such as a tile codec. The decode step is inline (hand-written SH-2 assembly on SH-2,
with the bit reader kept in the caller's registers), tables are compact byte arrays in one
arena, the reader starts at any byte offset, and stream bounds are checked once per stream.
A builder and a table check for untrusted tables are included; vlc-1 is unchanged.
Host-exact tested against an independent oracle and costed in the SH-2 instruction model
(modelled, no console run). See [the vlc-2 guide](saturn/VLC2.md).

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

Contract tag `view-rails-1` steps both edges ("rails") of a textured strip at shared
boundaries in one call, `sh2z_view_edge_rails_center` (and `_divu`), with the centre view
validated once per frame through a prepared `sh2z_view_rails_t` handle instead of once per
rail. Each rail's points are exactly what `sh2z_view_edge_steps_center` writes (0 px error),
and the call writes nothing when an argument breaks its rules. See
[the rails guide](saturn/VIEW_RAILS.md).

Contract tag `pose-1`, [`sh2zam/contracts/pose_1.h`](saturn/include/sh2zam/contracts/pose_1.h)
(included by `view_experimental.h`), adds `sh2z_pose_from_f32`: a model's view, camera
position in model space, Q1.14 basis and plane-culling data, built from model state stored as
IEEE 754 binary32 floats without any floating-point arithmetic. It reads each float's bits and
reproduces the binary32 operations involved exactly with integer instructions, so a renderer
that keeps its model state as floats does not pay soft-float per model. See
[the pose guide](saturn/POSE.md).

## Block memo kernels

[`sh2zam/memo_experimental.h`](saturn/include/sh2zam/memo_experimental.h) (contract tag `memo-1`)
gives exact kernels for per-picture work on data that has mostly not changed: compare 32- or
64-byte blocks with their shadows and copy and report only changed ones (singly or as a bit
mask over many blocks), write a record's words to a slow 16-bit destination only when their
key bits changed, copy 8-byte records up to an end marker, and diff two record tables into runs.
Hand-written SH-2 kernels and portable C give the same results and the same destination
writes, with no floating point, divider, libc or allocation. Host-exact tested against an
independent oracle and costed in the SH-2 instruction model (modelled, no console run). See
[the memo guide](saturn/MEMO.md).

## VDP1 double banks

[`sh2zam/vdp1_banks_experimental.h`](saturn/include/sh2zam/vdp1_banks_experimental.h)
(contract tag `vdp1-banks-1`) is bookkeeping for building frame N+1 while VDP1 still draws
frame N:
- bank state with serial, overlap and conflict counters;
- guard words;
- carving a guarded region;
- an in-order queue for tables uploaded at the end of a draw;
- per-bank staleness for tables uploaded only when they change;
- a jump-skip command record that keeps a block at a fixed slot.

It makes decisions and records them; the caller reads VDP1 state and does every wait, transfer
and upload. Portable C, no libc. Host-tested against a frame model with negative controls (a
held bank must conflict, a changed guard bit must fail). See
[the VDP1 banks guide](saturn/VDP1_BANKS.md).

## SCU DSP

[`sh2zam/dsp_experimental.h`](saturn/include/sh2zam/dsp_experimental.h) (contract tag `dsp-1`)
drives the Saturn's SCU DSP, a fixed-point coprocessor that runs while the SH-2s do other work:
- program and data RAM access, start, poll and wait;
- D0-bus addresses and cache purge for the DSP's own DMA;
- a batched affine transform (three rows of coefficients and a translation, exact modulo 2^32);
- a Q16 outer product, `floor(a[i] * b[k] / 65536)` from the exact 48-bit product.

Kernels are DSP assembly in `saturn/src/dsp/`, assembled by `saturn/tools/sh2z_dspasm.py` (MIT)
and committed as data; CI checks them against their sources. Host tests run the committed words
on a reference DSP simulator against the CPU form, with mutation controls. Portable builds run
the CPU form. See [the DSP guide](saturn/DSP.md).

## Two-CPU try-claim

[`sh2zam/dual_claim_experimental.h`](saturn/include/sh2zam/dual_claim_experimental.h)
(contract tag `dual-claim-1`) shares an optional job between the two SH-2s so that exactly one
runs it:
- no atomic instruction: two claim words and a decision word in cache-through memory;
- side A (the master) may wait a few instructions for B's decision; side B (the slave)
  never waits;
- a new tag for every job, so the record needs no reset.

Host-tested over every interleaving of the two sides' operations, with four broken
protocols as negative controls. See [the try-claim guide](saturn/DUAL_CLAIM.md).

## Segmented strip meshes

[`sh2zam/segmesh_experimental.h`](saturn/include/sh2zam/segmesh_experimental.h)
(contract tag `segmesh-1`) picks what is visible in a closed ring of mesh segments, such as a
circuit built from strips. `sh2z_segmesh_pick` tests one anchor point per segment against four
fixed-point clip rows without dividing, groups the visible segments into runs of similar depth
and returns the runs furthest first, ready for painter-order drawing. Integer arithmetic only,
no float, libc or allocation. Host-exact tested against an independent oracle with mutation
controls (no console run). Contract tag `segmesh-2` adds `sh2z_segmesh_cull`, which drops whole
clusters of faces by a baked sphere and normal cone. It is an approximation: it can skip a face a
pixel wide that the per-face path would draw. See [the segmesh guide](saturn/SEGMESH.md).

## Sprite slot cache

[`sh2zam/sprite_cache_experimental.h`](saturn/include/sh2zam/sprite_cache_experimental.h)
(contract tag `sprite-cache-1`) streams pre-rendered sprite cells into a fixed pool of VDP1
VRAM slots. Least-recently-used replacement never replaces a slot VDP1 may still read (one or
two command banks), requests run a frame ahead, and mirrored cells draw from one slot with the
VDP1 flip bit. cell4z, a 4-bit cell codec on vlc-2 symbols, decodes straight into a slot at
0.37 to 0.44 of raw size and about 29 modelled cycles per pixel. Portable C, no libc or
allocation. Host-tested against a reference model with one negative control per rule (no
console run). See [the sprite cache guide](saturn/SPRITE_CACHE.md).

## Scene sequencing

[`sh2zam/scene_experimental.h`](saturn/include/sh2zam/scene_experimental.h)
(contract tag `scene-1`) runs a fixed table of scenes, such as title, menus, loading, game and
results:
- fades in and out, with an interrupted fade turning around from the level it reached;
- requests by rank, so a reset can still redirect a transition that has begun;
- release only after the caller confirms that the last displayed frame of the leaving scene
  has been replaced (elapsed frames alone prove nothing);
- bounded preload chunks before a scene is entered;
- bump arenas with a high-water mark, reset when their scene is released;
- press edges that a transition consumes, with directional auto-repeat.

Pure bookkeeping: the caller reads the pad, draws, fades and waits. Portable C, no libc.
Host-tested against an independent timeline model with a negative control per rule. See
[the scene guide](saturn/SCENE.md).

## Sound events and CD audio (experimental)

[`sh2zam/snd_experimental.h`](saturn/include/sh2zam/snd_experimental.h)
(contract tag `snd-1`) drives Saturn sound from the SH-2 with no 68000 program:
- an epoch-stamped event queue that delivers each event once per frame and refuses posts from a
  scene that has left;
- SCSP slot playback of 8- or 16-bit PCM clips, with priority stealing, level to TL and pitch
  scaling helpers;
- a CD audio player that plays a track range once and then a loop range forever, and fades and
  pauses without waiting on the CD block.

Register bases are the caller's, so it is host-tested against reference models with negative
controls (no console run). See [the sound guide](saturn/SND.md).

## Rotation sprites

[`sh2zam/rotation_sprite_experimental.h`](saturn/include/sh2zam/rotation_sprite_experimental.h)
(contract tags `rotation-sprites-2` and `rotation-sprites-3`) draws a 3D object as one VDP1
distorted sprite, using a cell from a set baked over a grid of viewing directions:
- picks the cell nearest the viewing direction, by dot products, with mirrored sets;
- places the object's origin with sub-pixel precision and its on-screen up axis;
- emits one command that scales the cell with depth and rotates it with the object's roll.

The `-3` forms give identical output faster: a yaw climb from the last cell, the pose from a
camera's own terms, and emission of one cell. Integer only, with no libc. Host-tested against
float references and the general forms (no console run). See
[the rotation sprite guide](saturn/ROTATION_SPRITES.md).

## Racing lines and vehicle dynamics

- [`sh2zam/line_follower_experimental.h`](saturn/include/sh2zam/line_follower_experimental.h)
  (`line-follower-1`) moves a vehicle along a baked closed line, as a distance plus a lateral
  offset, with speed and steering limits.
- [`sh2zam/line_field_experimental.h`](saturn/include/sh2zam/line_field_experimental.h)
  (`line-field-1`) runs up to 255 such vehicles, with contact, rank and level of detail;
  `line-field-2` adds a per-vehicle finish-time re-target from the second lap.
- [`sh2zam/vehicle_dynamics_experimental.h`](saturn/include/sh2zam/vehicle_dynamics_experimental.h)
  (`vehicle-dynamics-1`) steps one hovering vehicle on a spline road, one fixed tick at a time.
  A host build and an SH-2 build give the same bits.
- [`sh2zam/body_contact_experimental.h`](saturn/include/sh2zam/body_contact_experimental.h)
  (`body-contact-1`) resolves contact between vehicles that touch as spheres: shared impact damage,
  a push while they close, attackers and contact credit. `vehicle-dynamics-2` feeds it into the
  vehicle step (`sh2z_vd_step_contact`).

All four are integer only and deterministic. See [the line guide](saturn/LINE_FOLLOWER.md),
[the vehicle dynamics guide](saturn/VEHICLE_DYNAMICS.md) and [the body contact guide](saturn/BODY_CONTACT.md).

## Link only what you call

From contract tag `sections-1`, every hand-written SH-2 entry and every read-only table has its
own section. Compile with `-ffunction-sections -fdata-sections` and link with
`-Wl,--gc-sections`, and a program keeps only the entries and tables it calls, whether it
builds the sources itself or links `libsh2zam.a`. For example, a program that calls only
`sh2z_memo_put4` links 820 bytes of the library instead of 2,400. No API or result changed.
A linker script can also place a cold table by name, for example in low work RAM. See
[the validation record](saturn/validation/sections-1.md) and
[the link probe](saturn/benchmarks/gc-probe/README.md).

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

# Releases

Every tag has a [GitHub release](https://github.com/lamb2k/SH2ZAM/releases) with notes: what it adds, its evidence tiers, and the merge train it came in with.

- **Kernel tags** such as `view-rails-1` or `vlc-2` are SH2ZAM contracts. Each release links the contract's guide at that tag.
- **`v0.5` to `v0.9.1`** are upstream SH4ZAM releases, kept with the Git history.
- Downloads are GitHub's source archives; there are no binary assets.

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
