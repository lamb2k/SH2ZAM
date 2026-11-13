# Saturn prepared-transform benchmark

The harness compares **software float → the prior safe batch → a prepared batch**,
then measures two experimental SH-2 assembly candidates against the same inputs.
**The compiled SH-2 kernels have executed and passed exact output checks in an
isolated instruction model. No physical-console measurements are recorded.**
The [model evidence](sh2-model/README.md), browser/WASM timing and console capture
schema are separate. A successful model run checks target instruction behavior;
boot, timer integration and elapsed time on a Saturn still require console evidence.

## Workload and kernels

`exhibits/meshes.js` supplies the browser and fixture generator: Ring world, Cube
field and Twisted orbit, each at detail 1–5. `generate-fixtures.cjs` freezes the exact
Q16.16 vertices and 21 poses across the same 12-second loop. Its manifest records
vertex counts and a SHA256 of all inputs and poses.

Both environments compile `exhibits/bridge.c`. Matrix construction quantizes once;
the float kernel receives float32 conversions of those same matrix coefficients
and vertices. Matrix construction, input conversion, and the one-time scan that
prepares the immutable mesh view are outside transform timing. Per-matrix plan
qualification is measured separately and included once in each prepared whole-mesh
result. Input arrays remain unchanged throughout all poses.

| Path | Bridge function | Measured operation |
|---|---|---|
| Software float | `before_range` | Float32 affine transform; SH-2 software-float helpers |
| Prior safe batch | `general_range` | `sh2z_mat34_transform_points`, including its matrix-only dispatch |
| Portable prepared | `after_range` | `sh2z_transform_plan_execute_range` on a qualified plan |
| Experimental MAC.L | `mac_range` | Prepared wrapper using fused native accumulation when available |
| Experimental DMULS.L | `dmuls_range` | Prepared wrapper using native products and ADDC accumulation when available |

The legacy JSON keys `general` and `optimized` now mean **prior safe batch** and
**portable prepared**, respectively. Schema 3 makes that change explicit. The
original full-range scalar routine remains the correctness fallback and is covered
by the host oracle suites; it is no longer the middle timing baseline.

The portable prepared implementation uses ordinary C arithmetic, so GCC generates
its SH instructions. The two assembly candidates are optional entry points, not
the default prepared dispatcher. MAC.L requires SR.S to be clear and does not alter
it. Both candidates require a successful prepared bounds proof and a native SH-2
build. Otherwise they execute the ordinary prepared fallback. Availability is
recorded for every pose; a fallback timing is never published as an active native
candidate result. See `include/sh2zam/batch_experimental.h` for the contracts.

All four fixed-point outputs must match bit for bit at every coordinate. Separate
buffers and checksums keep all five results observable. Host tests compare the
prepared implementation and instruction models against an independent 128-bit
integer oracle. The shared-fixture test additionally checks float error and
whole-mesh/chunk parity; executing a host fallback does not test SH-2 assembly.

The browser uses WebAssembly hardware floating point and the portable fixed-point
backend. Saturn float32 uses software helpers. CI checks the bridge's `__mulsf3`
and `__addsf3` dependencies and exports native disassembly, relocations, symbol
sizes and fused-loop audits. Paired results within one environment are comparable;
browser timings cannot establish a Saturn speedup.

## Executed SH-2 instruction model

[The reproducible model harness](sh2-model/README.md) executes the actual
big-endian benchmark ELF, including libgcc software-float helpers, through
unmodified Yabause SH-2 opcode handlers pinned at
[`7e38821dbac265490f115e163c523a939acda759`](https://github.com/Yabause/yabause/tree/7e38821dbac265490f115e163c523a939acda759).
The committed [results.json](sh2-model/results.json) uses the ELF from SH2ZAM
commit [`26a935eea2c6fb9a7dfba6a81d273ab81562a2eb`](https://github.com/lamb2k/SH2ZAM/commit/26a935eea2c6fb9a7dfba6a81d273ab81562a2eb),
[CI artifact 14068406384](https://github.com/lamb2k/SH2ZAM/actions/runs/43487712557/artifacts/14068406384),
with ELF SHA256
`38a7c875c749c0348f886405f30224d7e4ea8846c9e0f3341f173e7fe12a89a6`.

All **315 mesh/pose combinations** executed, checking **1,267,560 coordinates per
fixed path** against an independent `__int128` oracle with **zero mismatches**.
The maximum software-float difference was **0.000015497207641601562 world units**
(**1.015625 Q16.16 units**, within the two-unit comparison bound). Every ordinary
pose selected availability mask 7, and opcode counts confirmed execution of the
native candidates. Additional SR.S=1 and full-range checks verified exact fallback
outputs; every target call checked the stack, callee-saved registers and SR.S.

For Ring world detail 4, **1,584 vertices**, the inclusive medians across 21 poses
were:

| Path | Modeled instruction cycles per whole batch |
| --- | ---: |
| Software float | 6,751,532 |
| Prior safe batch | 316,178 |
| Prepared C, including plan | 152,597 |
| Fused MAC.L, including plan | 93,956 |
| Fused DMULS.L, including plan | 136,725 |

Across the 15 workloads, the model reports **43.65–45.41× float/prepared**,
**70.67–73.80× float/MAC**, and **48.71–50.68× float/DMULS**. Prepared and native
samples include one plan preparation per full batch, added per pose before taking
the median. Range wrappers and all output stores are included; one-time input
conversion and mesh scanning, matrix construction, and precision checks are
excluded. Raw guest instruction counts are retained separately from modeled cycles.

This is a flat-RAM CPU interpreter model with **no cache, bus wait states, pipeline
or multiplier contention, DMA, interrupts or peripheral integration**. It does not
run BIOS startup or boot the display/timer application. Whole-batch calls also
differ from the console harness's 32-vertex timing chunks. The model ratios are
not physical Saturn speedups or full-system emulator timings and are not converted
to console milliseconds or FPS. Keep this data in its separate model schema;
physical-console captures remain empty until measured.

The [cross workflow](../../.github/workflows/sh2-cross.yml) now runs this optional
model harness against the freshly built ELF and uploads `raw.json` and validated
`results.json` alongside native build evidence. Each capture records its own
source, fixture, ELF, interpreter and harness provenance. Earlier compile-only CI
results in [the validation record](../VALIDATION.md) retain their historical scope.

## Build and capture

Use the Yaul SDK/toolchain pinned in `.github/workflows/sh2-cross.yml`:

```sh
node saturn/benchmarks/generate-fixtures.cjs
make -C saturn/benchmarks/yaul
```

CI builds the bootable `sh2zam-benchmark` image and uploads it in
`sh2-compile-evidence`, with the compiler version, image digest, manifest, linked
outputs, instruction/helper audits and isolated-model results. The boot program
and console timer integration remain unqualified until the image has actually
booted and produced valid output on the recorded console or full-system emulator.

Boot the ISO/CUE using your usual Saturn loader. The application captures all 15
workloads, then shows **60 pages: four per workload**. The first two pages, marked
A, cover poses 0–10 and 11–20; the next two, marked B, cover those poses again with
the remaining fields. Each page fits at most 11 sample rows at 320×224. Pages
advance every 600 display frames and repeat.

| Page | Display columns | Capture fields |
|---|---|---|
| A | `float`, `safe`, `empty`, `prep64` | `referenceTicks`, `generalTicks`, `overheadTicks`, `prepareTicks` |
| B | `prepared`, `mac`, `dmuls`, `pe64`, `A` | `optimizedTicks`, `macTicks`, `dmulsTicks`, `prepareOverheadTicks`, `candidateMasks` |

`prep64` and `pe64` are raw FRT ticks for 64 plan-preparation calls and 64 matching
empty calls. The `A` column is a bit mask: 1 = prepared proof succeeded, 2 = MAC.L
available, 4 = DMULS.L available. For example, 7 means all three; 5 means the
prepared and DMULS.L paths are available while MAC.L falls back. A zero mask means
the plan's full-range fallback is required.

Photograph all pages, including the commit/fixture prefixes, six checksums,
maximum error, mismatch count, every raw timing sample and every availability
mask. The checksum labels map to `referenceChecksum`, `generalChecksum`,
`optimizedChecksum`, `macChecksum`, `dmulsChecksum`, and combined `checksum`.
Any `INVALID` result is unusable. Keep the full source commit, image digest and
photographs/log as public evidence; displayed prefixes alone are insufficient.

Record console model, PAL/NTSC region, actual CPU clock, video mode (320×224 here),
compiler/flags, cache policy and memory placement. SDK startup cache settings are
not certified by this harness. Inspect startup code and the linked map when
recording those settings. The app warms paths without flushing caches and does
not start slave-CPU work. An emulator can test boot and behavior; emulator timing
must be labeled `emulator` and is not shown as physical Saturn performance.

## Timing and interpretation

There are 21 deterministic poses and five kernels. The start of the timing order
rotates by one path per pose, so each kernel occupies each position **four or five
times**, rather than an exactly balanced count. All kernels are warmed twice at
each pose. The preparation and preparation-empty measurement order alternates.

The CPU FRT runs at clock /128. Each kernel is timed in 32-vertex chunks, resetting
the 16-bit counter for each chunk and summing the ticks. Any overflow flag
invalidates the capture instead of guessing a wrap count. The interrupt mask is
15 during the timed interval and restored to 0 afterward. Rendering, matrix
construction, input conversion, mesh scanning, checksums and precision checks are
outside these intervals. This measures isolated CPU transform work, not game FPS.
The timer integration itself still needs first-console qualification.

Every chunk includes the path's range wrapper. The safe batch checks its matrix
for every chunk; prepared paths validate their output/range and reuse the prepared
plan. Those range costs remain in the measurements. A direct full-mesh call can
amortize wrapper work over more vertices, so chunked FRT timing is not a claim of
the best achievable full-batch throughput. Keep chunk size in provenance.

Plan qualification is measured as a separate block of 64 `plan_prepare()` calls,
with a matching 64-call empty-loop measurement. For each pose, the converter uses:

```text
setup ticks per batch = (prepareTicks - prepareOverheadTicks) / 64
kernel ticks         = raw kernel ticks - overheadTicks
prepared total ticks = prepared kernel ticks + setup ticks per batch
MAC/DMULS total ticks = candidate kernel ticks + setup ticks per batch
```

It adds setup **once per whole mesh**, not once per 32-vertex chunk. All adjustment
and addition happens per pose before taking the median of the 21 samples. Separate
medians must not be added together. The converter also exports kernel-only and
setup medians, so setup amortization can be inspected. A resolved zero setup cost
is allowed and means the timer did not resolve a positive net value; it is not
replaced with an epsilon. Negative setup or nonpositive adjusted kernel samples
are rejected. Empty-loop subtraction does not remove cache effects or uncertainty
from FRT quantization.

Cycles are adjusted ticks × 128; milliseconds are cycles / the recorded CPU clock
× 1000. `referenceOverGeneral`, `referenceOverOptimized` and
`generalOverOptimized` divide the numerator's total by the denominator's total.
Values above 1 favor the denominator; slower prepared results remain valid.
Native-candidate summaries are null if any pose used a fallback. Their raw samples
and availability masks remain in the output for inspection.

`fixedMismatches` counts coordinates where any fixed-point path differs from the
safe batch and must be zero. All four fixed-point checksums must agree.
`maxErrorLSB` is the maximum float32 error, rounded up to Q16 LSB units across all
vertices and poses, and must not exceed 66 LSB (about 0.00101 world units). This is
not a screen-pixel error. Every output contributes to its own checksum and the
combined checksum. Separate translation units, memory barriers and `-fno-lto`
keep the calls and output consumption observable. Hardware and browser builds use
`-O2` and `-O3` respectively; compare kernels within each environment.

## Capture schema and publication

Keep `results/captures.json` at `run: null`, `records: []` until evidence exists.
The data uses **schema 3** and kernel set `float-safe-prepared-fused-v1`. Schema 1
and 2 records are rejected because they measured different kernel sets and did
not include prepared setup or native-candidate availability.

A real `run` contains:

| Field | Required value |
|---|---|
| `environment` | `physical-console` or `emulator` |
| `kernelSet` / `backend` | `float-safe-prepared-fused-v1` / `sh2-dmuls` |
| `timerDivisor` / `chunkVertices` | 128 / 32 |
| `prepareIterations` | 64 |
| `sourceCommit` / `imageSHA256` | Full source commit and boot ISO digest |
| `clockHz` | Validated CPU frequency for the console and video mode |
| `compiler` / `flags` | Exact toolchain and compilation options |
| `console` / `videoMode` | Console identity, region and selected video mode |
| `cachePolicy` / `memoryPlacement` | Observed settings and linked memory placement |
| `capturedAt` / `evidenceUrl` | ISO timestamp and HTTPS evidence link |

Each record identifies `scene`, `detail`, `vertices`, `invalid: false`,
`fixedMismatches: 0`, integer `maxErrorLSB`, and the six eight-digit hexadecimal
checksums listed above. It has **21 integers in each** of the nine arrays
`referenceTicks`, `generalTicks`, `optimizedTicks`, `macTicks`, `dmulsTicks`,
`overheadTicks`, `prepareTicks`, `prepareOverheadTicks`, and `candidateMasks`.
Scene names are `saturn`, `cubes`, `torus`. Keep the manifest's `fixtureSHA256`;
changing geometry or poses invalidates earlier data. Partial captures are allowed:
missing scene/detail combinations remain unmeasured.

```sh
node saturn/benchmarks/generate-fixtures.cjs
node saturn/benchmarks/compile-results.cjs
node saturn/benchmarks/test-results.cjs
```

The converter retains all raw data and derives:

- `referenceCycles`, `generalCycles`, `optimizedCycles`: headline float, safe and
  prepared totals; prepared includes one plan setup per mesh.
- `prepareCycles`, `optimizedKernelCycles`: separately observable setup and
  prepared kernel medians.
- `macCycles`, `dmulsCycles`, `macKernelCycles`, `dmulsKernelCycles`: candidate
  totals/kernel medians, or null when unavailable for any pose.
- Matching `*Ms` fields, ratios, and `preparedAvailable`, `macAvailable`,
  `dmulsAvailable` flags.

Generated `exhibits/saturn-results.js` is bundled in the site and offline artifact.
The initial UI presents the three headline columns for physical-console records
matching the selected scene/detail; candidate details remain in the data export.
The validator checks consistency, not authenticity. Review source and capture
evidence before publication. A result belongs to its recorded build, and cannot
be relabeled as a measurement of later code.

## Host correctness check

```sh
node saturn/benchmarks/generate-fixtures.cjs
cc -O2 -std=c11 -Wall -Wextra -Werror -ffp-contract=off \
  -DSH2Z_FORCE_PORTABLE=1 -Isaturn/include \
  saturn/benchmarks/test-fixtures.c exhibits/bridge.c saturn/src/*.c \
  -lm -o /tmp/sh2zam-fixtures
/tmp/sh2zam-fixtures
```

The fixture test runs all 315 mesh/pose combinations, validates whole-mesh/chunk
identity for five wrappers, checks prepared-kernel/inclusive-call parity, requires
exact equality among all four fixed-point outputs and verifies a 0.001 world-unit
float error bound. It also checks that source vertices remain unchanged. Assertions
remain active in Release builds. Host candidate calls exercise their portable
fallbacks; the independent oracle and native instruction-model suites are CTest
`prepared_transform_oracles` and `native_batch_host_models`.
