# Saturn three-way transform benchmark

Status: the harness compares **software float → general SH2ZAM → optimized SH2ZAM**.
**No physical-console measurements are recorded.** Browser/WASM timing is shown
separately in the interactive lab. Saturn values remain unmeasured until an evidenced
capture is committed.

## Workload and kernels

`exhibits/meshes.js` supplies both the browser and the fixture generator: Ring world,
Cube field and Twisted orbit, each at detail 1–5. `generate-fixtures.cjs` freezes the
exact Q16.16 vertex inputs and 21 poses across the same 12-second loop. Its manifest
records vertex counts and a SHA256 of the complete inputs and poses.

Both environments compile `exhibits/bridge.c`. All three transform kernels receive
the same vertices and the same matrix coefficients. Setup constructs the Q16.16
matrix once, then converts those exact coefficients and vertices to float32 outside
timing. This isolates transform arithmetic; it does not compare different trig
approximations or matrix construction methods.

| Path | Function | What is measured |
|---|---|---|
| Software float | `before_range` | Float32 affine transform; SH-2 compiler software-float helpers |
| General SH2ZAM | `general_range` | Existing full-range `sh2z_mat34_transform_points_general` |
| Optimized SH2ZAM | `after_range` | Safe dispatch through `sh2z_mat34_transform_points`, including its matrix bound check |

The general and optimized paths must match **bit for bit** at every output coordinate.
The benchmark also measures their error against float32. Separate output buffers and
checksums keep all three results observable. The optimized path retains the general
fallback for matrices outside its proven accumulator bound.

The browser uses WebAssembly hardware floating-point operations and the portable
fixed-point backend. On Saturn, float32 uses software helpers; fixed point uses
native `dmuls.l`. CI checks `__mulsf3` and `__addsf3` dependencies and exports native
multiplication disassembly, relocations and symbol sizes for inspection. These checks
do not establish a hardware speedup.

## Build and run

Use the Yaul SDK/toolchain pinned in `.github/workflows/sh2-cross.yml`:

```sh
node saturn/benchmarks/generate-fixtures.cjs
make -C saturn/benchmarks/yaul
```

CI builds the bootable `sh2zam-benchmark` image and uploads it in
`sh2-compile-evidence`, with the compiler version, image checksum, manifest, linked
outputs and instruction/helper audit. Build success does not establish that the image
has booted on a console.

Boot the ISO/CUE using your usual Saturn loader. The app captures all 15 workloads,
then displays **30 result pages: two pages per workload**. Each page holds up to 11
poses so four raw timing columns remain readable at 320×224. Pages advance every 600
display frames and repeat. Photograph all pages, including build/fixture identifiers,
all four checksums, maximum error, mismatch count, and all 21 float/general/optimized
sample triples with their overhead samples. Any `INVALID` result is unusable.

Keep the image digest, complete source commit and photographs/log as public capture
evidence. An emulator can test boot and correctness; emulator timings must be marked
`emulator` and are not shown as Saturn performance in the lab.

Record console model, PAL/NTSC region, actual CPU clock, video mode (this app selects
320 horizontal), compiler/flags, cache policy and memory placement. SDK startup cache
settings are not automatically certified by this harness. The app warms all paths;
it does not flush caches between samples. It does not start slave-CPU work. Inspect
the SDK startup and linked map when documenting those settings. No clock is guessed
by the browser or converter.

## Measurement

- 21 deterministic poses, one sample per pose for each kernel. The three timing
  orders rotate so every kernel occupies each position seven times.
- All three kernels are warmed twice at each pose. Their inputs, matrix coefficients,
  buffer layout, compiler flags and vertex counts match.
- CPU FRT clock /128; 32-vertex chunks prevent long intervals from wrapping the
  16-bit timer. Each chunk resets the count. The overflow flag invalidates a capture
  rather than silently wrapping. FRT control follows Yaul's `cpu/frt.h` and
  `cpu_frt.c` APIs. This timer integration still needs first-console qualification.
- Interrupt mask 15 only during timed chunks, restored to 0 afterward. Rendering,
  matrix construction, input conversion, checksums and precision checks are outside
  the measured interval. This measures isolated CPU transform work, not game FPS.
- Raw whole-mesh tick sums include the normal range wrapper and dispatch cost for
  every chunk. The optimized matrix bound check is inside timing. A full-mesh call
  amortizes that check over more vertices; the fixed 32-vertex chunks therefore do
  not measure the best possible full-batch throughput. Keep chunk size in provenance.
- A matching empty-call measurement is retained per pose. The converter subtracts
  each overhead sample, then takes the median of 21 adjusted whole-mesh totals for
  each kernel. Nonpositive adjusted samples are rejected. Cache effects are not
  removed by subtraction.
- Cycles = adjusted ticks × 128. Milliseconds = cycles / recorded CPU clock × 1000.
  `referenceOverGeneral`, `referenceOverOptimized` and `generalOverOptimized` are the
  numerator's time divided by the denominator's time; values above 1 favor the
  denominator. Slower optimized results remain valid measurements.
- `fixedMismatches` counts general/optimized component differences and must be zero.
  Both fixed-point checksums must also agree. `maxErrorLSB` is the maximum float32
  error, rounded up to Q16 LSB units over every vertex and pose. Capture validation
  requires at most 66 LSB (about 0.00101 world units). It is not a screen-pixel error.
- Every output contributes to its kernel checksum and a combined checksum. Separate
  translation units, memory barriers and `-fno-lto` prevent benchmark elimination.
  Hardware and browser builds use `-O2` and `-O3` respectively; compare paired kernels
  within each environment, not absolute times across environments.

## Publish a capture

Edit `results/captures.json`. Leave `run: null` and `records: []` until evidence exists.
The data uses **schema 2**. Schema 1 two-kernel captures are rejected; they cannot be
relabeled as measurements of the optimized path. A real `run` must contain:

| Field | Required value |
|---|---|
| environment | `physical-console` or `emulator` |
| kernelSet | `float-general-batch-v1` |
| backend | `sh2-dmuls` |
| timerDivisor / chunkVertices | `128` / `32` |
| sourceCommit / imageSHA256 | Full source SHA and boot ISO digest |
| clockHz | Validated CPU frequency for the console/mode |
| compiler / flags | Exact toolchain and compilation options |
| console / videoMode | Console identity, region and selected video mode |
| cachePolicy / memoryPlacement | Observed/configured cache and linked memory placement |
| capturedAt / evidenceUrl | ISO timestamp and HTTPS capture evidence link |

Each record identifies `scene`, `detail`, `vertices`, `invalid: false`,
`fixedMismatches: 0`, integer `maxErrorLSB`, and four eight-digit hexadecimal checksums:
`referenceChecksum`, `generalChecksum`, `optimizedChecksum`, and combined `checksum`.
It also contains **21 integers in each** of `referenceTicks`, `generalTicks`,
`optimizedTicks`, and `overheadTicks`. Scene names are `saturn`, `cubes`, `torus`.
Keep the manifest's `fixtureSHA256`; changing geometry or poses invalidates old data.
Partial captures are allowed: missing scene/detail combinations remain unmeasured.

```sh
node saturn/benchmarks/generate-fixtures.cjs
node saturn/benchmarks/compile-results.cjs
node saturn/benchmarks/test-results.cjs
```

The converter produces `referenceCycles`, `generalCycles`, `optimizedCycles`, their
`*Ms` equivalents and the three time ratios. Generated `exhibits/saturn-results.js`
is bundled by the site build and offline artifact. The UI shows only physical-console
records matching the selected scene/detail, with evidence and recorded build provenance.
The validator checks consistency, not authenticity: review capture evidence and source
before publication. A measurement from older math code remains a measurement of that
recorded build, never a new measurement.

## Correctness check

```sh
node saturn/benchmarks/generate-fixtures.cjs
cc -O2 -std=c11 -Wall -Wextra -Werror -ffp-contract=off \
  -DSH2Z_FORCE_PORTABLE=1 -Isaturn/include \
  saturn/benchmarks/test-fixtures.c exhibits/bridge.c saturn/src/*.c \
  -lm -o /tmp/sh2zam-fixtures
/tmp/sh2zam-fixtures
```

This runs all 315 mesh/pose combinations, validates full-batch/chunk output identity
for all three kernels, requires exact general/optimized equality and checks a 0.001
world-unit error bound against float32. It produces correctness evidence only.
