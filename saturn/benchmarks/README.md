# Saturn float32 vs SH2ZAM benchmark

Status: benchmark harness implemented; **no physical-console measurements recorded**.
The interactive lab shows browser/WASM timing separately. Saturn values remain unmeasured
until an evidenced capture is committed. No emulator result or estimated multiplier is
substituted for hardware data.

## Workload

`exhibits/meshes.js` supplies the browser and the fixture generator: Ring world, Cube
field and Twisted orbit, each at detail 1–5. `generate-fixtures.cjs` freezes the exact
Q16.16 vertex inputs and 21 poses across the same 12-second loop. Its manifest records
vertex counts and a SHA256 of the full inputs and poses. The reference converts these
same input integers to float32 once before measurement.

Both builds compile `exhibits/bridge.c`. On the browser it targets WebAssembly with the
portable backend. On Saturn, the float32 kernel uses the compiler's software float
helpers; SH2ZAM uses its native `dmuls.l` path. CI checks the reference object's
`__mulsf3` and `__addsf3` dependencies and audits native multiplication instructions.
This is not a claim that fixed point will win: the full safe arithmetic implementation
is measured as it exists today.

## Build and run

Use the Yaul SDK/toolchain pinned in `.github/workflows/sh2-cross.yml`:

```sh
node saturn/benchmarks/generate-fixtures.cjs
make -C saturn/benchmarks/yaul
```

CI also builds the bootable `sh2zam-benchmark` image and uploads it in
`sh2-compile-evidence`, along with the compiler version, image checksum, manifest,
linked build outputs and instruction/helper audit. Build success does not mean the
image has booted successfully on a console.

Boot the ISO/CUE using your usual Saturn loader. The app captures all 15 workloads,
then cycles through their result pages every 600 display frames. Photograph all pages,
including the build/fixture identifiers, checksum, maximum error, 21 raw float/fixed
sample pairs and 21 overhead samples. Pages repeat. Any `INVALID` result is unusable.
Keep the original image digest, complete source commit and the photographs/log as
public capture evidence. An emulator can test boot/correctness, but its timings must
be marked `emulator` and are not shown as Saturn performance in the lab.

Record console model, PAL/NTSC region, actual CPU clock, video mode (this app selects
320 horizontal), compiler/flags, cache policy and memory placement. SDK startup cache
settings are not automatically certified by this harness. The app warms both paths;
it does not flush caches between samples. It does not start slave-CPU work. Inspect
the SDK startup and linked map when documenting those settings. No clock is guessed
by the browser or converter.

## Measurement

- 21 deterministic poses, one sample per pose, alternating float/fixed order.
- Both kernels warmed twice at each pose; identical inputs, output buffers and flags.
- CPU FRT clock /128; 32-vertex chunks prevent long intervals from wrapping the
  16-bit timer. Each chunk resets the count. The overflow flag invalidates the capture
  rather than silently wrapping. FRT register control follows Yaul's `cpu/frt.h` and
  `cpu_frt.c` APIs. This timer integration still needs first-console qualification.
- Interrupt mask 15 only during timed chunks, restored to 0 afterward. Rendering,
  matrix construction, input conversion, checksums and error evaluation are outside
  the measured interval. This measures isolated CPU work, not game-frame FPS.
- Raw whole-mesh tick sums include chunk call/timer overhead. A matching empty-call
  measurement is retained per pose. The converter subtracts each overhead sample,
  then takes the median of 21 adjusted whole-mesh totals. It rejects nonpositive
  adjusted samples. Instruction/data cache interactions are not removed by subtraction.
- Cycles = adjusted ticks × 128. Milliseconds = cycles / recorded CPU clock × 1000.
  Float/fixed time >1 means SH2ZAM is faster; <1 means it is slower.
- Maximum error is conservatively rounded up in Q16 LSB units, checked across all
  vertices and poses, outside timing. It is world-coordinate error, not screen pixels.
- Every output contributes to a retained checksum. Separate translation units,
  memory barriers and `-fno-lto` prevent benchmark elimination. Hardware and browser
  builds use different optimization levels (`-O2` and `-O3` respectively); compare
  the paired kernels within each environment, not absolute times across environments.

## Publish a capture

Edit `results/captures.json`. Leave `run: null` and `records: []` until evidence exists.
For a real capture, `run` must contain:

| Field | Value |
|---|---|
| environment | `physical-console` or `emulator` |
| backend | `sh2-dmuls` |
| timerDivisor / chunkVertices | `128` / `32` |
| sourceCommit / imageSHA256 | Full source SHA and boot ISO digest |
| clockHz | Validated CPU frequency for the console/mode |
| compiler / flags | Exact toolchain and compilation options |
| console / videoMode | Console identity, region and selected video mode |
| cachePolicy / memoryPlacement | Observed/configured cache and linked memory placement |
| capturedAt / evidenceUrl | ISO timestamp and HTTPS capture evidence link |

Each record identifies `scene`, `detail`, `vertices`, `invalid: false`, an eight-digit
hex `checksum`, integer `maxErrorLSB`, and 21 integers in each of `referenceTicks`,
`fixedTicks`, `overheadTicks`. Scene names are `saturn`, `cubes`, `torus`. Keep the
manifest's `fixtureSHA256`; a change to geometry or poses invalidates old data.
Partial captures are allowed: missing scene/detail combinations remain unmeasured.

```sh
node saturn/benchmarks/generate-fixtures.cjs
node saturn/benchmarks/compile-results.cjs
node saturn/benchmarks/test-results.cjs
```

The generated `exhibits/saturn-results.js` is bundled by the site build, including the
offline artifact. The UI only shows physical-console records and selects the record
matching the active scene and mesh detail. Evidence is linked beside the measurements.
The validator checks structure and consistency, not authenticity: review the raw
capture, build and hardware provenance before publishing. A capture from older math
code must be identified as that recorded build, never presented as a new measurement.

## Correctness check

```sh
node saturn/benchmarks/generate-fixtures.cjs
cc -O2 -std=c11 -Wall -Wextra -Werror -ffp-contract=off \
  -DSH2Z_FORCE_PORTABLE=1 -Isaturn/include \
  saturn/benchmarks/test-fixtures.c exhibits/bridge.c saturn/src/*.c \
  -lm -o /tmp/sh2zam-fixtures
/tmp/sh2zam-fixtures
```

This runs all 315 mesh/pose combinations, validates chunk/full output identity and
checks a 0.001 world-unit error bound. It produces correctness evidence only.
