# Validation record — 2026-11-13

## Saturn adoption comparison against independent fixed-point C

The site now reports recorded SH-2 instruction-model costs and has no browser
transform timing or sampling. Its default comparison is independent fixed-point
C against the optional native MAC.L API. Software float and prepared C remain
selectable. The browser only renders the live precision check.

[Native CI 43510305134](https://github.com/lamb2k/SH2ZAM/actions/runs/43510305134)
executed the compiled ELF from
[`29a4a5b43bd4fd83cd09f64b2662e819d1a1df8d`](https://github.com/lamb2k/SH2ZAM/commit/29a4a5b43bd4fd83cd09f64b2662e819d1a1df8d).
The [evidence artifact](https://github.com/lamb2k/SH2ZAM/actions/runs/43510305134/artifacts/14077414299)
contains the target ELF, symbols, raw capture and validated schema-2 result.
The ELF SHA256 is
`d3eb9af325c03f142711507177fc4b6f9cbdb756fadc2ebc3afbf28cb84e4c46`.
The committed [model record](benchmarks/sh2-model/results.json) retains all raw
samples, checksums and provenance for this capture.

The standalone baseline calls no SH2ZAM arithmetic, bounds-proof or execution
routine. It caches the matrix, unrolls three rows, accumulates products in 64 bits
and floors each result once. Its own per-matrix proof and snapshot are included,
just as library plan preparation is included. All paths use the same target
compiler and optimization flags. Unsupported inputs make the bounded baseline
fail without writing outputs.

Inclusive medians across 21 poses, in modeled cycles per whole batch:

| Detail 4 workload | Vertices | Plain fixed-point C | SH2ZAM prepared C | SH2ZAM native MAC.L | MAC cost saved vs plain |
| --- | ---: | ---: | ---: | ---: | ---: |
| Ring world | 1,584 | 123,881 | 152,597 | 93,956 | 24.16% |
| Cube field | 1,728 | 135,113 | 166,421 | 102,452 | 24.17% |
| Twisted orbit | 2,016 | 157,577 | 194,069 | 119,444 | 24.20% |

Across all 15 workloads, native MAC.L saves **22.90–24.24%** of modeled transform
cost against plain fixed-point C (**1.297–1.320×**). Prepared C costs
**23.14–23.82% more** than plain C. These results support a measured benefit for
the explicit native API; they do not establish a universal library speedup.
The software-float comparison is a separate migration result.

All **315 poses** passed, with **1,267,560 coordinates checked per fixed path**
and zero mismatches against the independent 128-bit oracle. The plain kernel
executed nine DMULS.L instructions per vertex and zero MAC.L. Native availability,
SR.S preservation, ABI preservation and full-range library fallback checks also
passed; the plain baseline rejected the full-range fixture without output writes.
The maximum float difference remained **1.015625 Q16.16 units**.

The interpreter has flat RAM and no cache, bus wait-state, pipeline, multiplier
contention or peripheral timing model. These are modeled transform costs, not
physical-console milliseconds or game FPS. Physical-console results remain
unmeasured. The browser captures below are historical and are no longer displayed
or collected by the site.

## Historical: coordinate reuse beats the browser float reference on the cube field

Implementation:
[`43f98abaacd074150efc04c64cb4087723ddf5f4`](https://github.com/lamb2k/SH2ZAM/commit/43f98abaacd074150efc04c64cb4087723ddf5f4).
[Chromium CI 43495362538](https://github.com/lamb2k/SH2ZAM/actions/runs/43495362538)
captured the actual browser application, with the unchanged float reference and
the three paths compiled together using the same flags. Each path retained 21
adaptive samples lasting at least 5 ms, with rotating measurement order.

The following medians are **microseconds per complete batch**, including one
matrix qualification and all contribution-table rebuilding for the optimized
path. Immutable mesh indexing, workspace binding and float input conversion
are one-time preparation outside the measured calls. Every output write and
the JS-to-WASM call remain included.

| Detail 4 workload | Vertices | Float | Prior safe batch | Optimized | Float / optimized |
| --- | ---: | ---: | ---: | ---: | ---: |
| Ring world | 1,584 | 4.167 µs | 7.227 µs | 5.469 µs | 0.76× |
| Cube field | 1,728 | 4.557 µs | 7.910 µs | **2.686 µs** | **1.70×** |
| Twisted orbit | 2,016 | 5.273 µs | 9.277 µs | 6.934 µs | 0.76× |

The cube capture reports `optimizedStrategy=coordinate-reuse` and
`coordinateReuseBlockSize=8`. It is **2.95× faster than the prior safe batch**.
Ring and torus retain the prepared scalar kernel and still lose to browser float.
These results describe this browser capture, not a universal fixed-point win or
a physical-Saturn speedup. The earlier scalar-table prototype tied float on one
CI runner and lost on another; performance claims apply to the measured block
kernel and workload, and remain sensitive to the host/browser environment.

The WASM SHA256 is
`bf02355225fe2d4bfbd398233efe7c991fd5f4740f129c5f01dd3bf9ca2c67e0`.
The CI artifact includes the complete distributions, component timings,
precision checks, module hash and browser metadata in the three exported
`test-result*.json` files. All three captures had zero fixed mismatches and
maximum float error of one Q16.16 LSB at their captured pose.

[Host CI 43495362743](https://github.com/lamb2k/SH2ZAM/actions/runs/43495362743)
passed all 11 targets in GCC, Clang and the sanitizer configuration. The new
reuse suite performs **2,395,331 independent oracle/contract checks**, including
broken coordinate patterns, every range alignment and tail length, exact output
order, matrix updates, numeric/capacity fallback and forbidden overlap. The
integrated WASM tests checked 737,280 random coordinates plus all **1,267,560
shared-fixture coordinates across 315 mesh/pose combinations**, with zero fixed
mismatches and maximum float error **0.000015497207641601562 world units**.
[SH-2 CI 43495362707](https://github.com/lamb2k/SH2ZAM/actions/runs/43495362707)
also passed the native build, instruction audits and executed-model checks.

The optional API and proof are documented in [COORDINATE_REUSE.md](COORDINATE_REUSE.md).
Browser captures include the selected strategy and block size; no scene-name
test or transformed-output cache participates in selection or timing.

## Historical: original five-path SH-2 instruction-model evidence

The actual compiled SH-2 software-float, safe, prepared, fused MAC.L and fused
DMULS.L paths have now executed through the unmodified opcode handlers of
[Yabause `7e38821dbac265490f115e163c523a939acda759`](https://github.com/Yabause/yabause/tree/7e38821dbac265490f115e163c523a939acda759).
The isolated harness loads the benchmark ELF and invokes its target functions
directly. **This is target instruction execution with modeled cycles; physical
Saturn execution and elapsed time remain unmeasured.**

The recorded ELF is from
[`26a935eea2c6fb9a7dfba6a81d273ab81562a2eb`](https://github.com/lamb2k/SH2ZAM/commit/26a935eea2c6fb9a7dfba6a81d273ab81562a2eb),
[SH-2 CI 43487712557](https://github.com/lamb2k/SH2ZAM/actions/runs/43487712557),
[artifact 14068406384](https://github.com/lamb2k/SH2ZAM/actions/runs/43487712557/artifacts/14068406384).
Its SHA256 is
`38a7c875c749c0348f886405f30224d7e4ea8846c9e0f3341f173e7fe12a89a6`.
[The historical model record](https://github.com/lamb2k/SH2ZAM/blob/741bcbf12442dd6b073525641a2d318708e2a6b2/saturn/benchmarks/sh2-model/results.json) retains that provenance,
fixture and harness hashes, all raw samples, opcode counts, output checksums,
availability masks and precision checks.

- All **15 workloads × 21 poses = 315 combinations** executed. Each of the four
  fixed paths matched an independent `__int128` floor-and-saturation oracle at
  **1,267,560 coordinates**, with **zero mismatches**. The maximum difference
  from software float was **0.000015497207641601562 world units**, exactly
  **1.015625 Q16.16 units**, within the two-unit comparison bound.
- All ordinary poses had availability mask **7**. Fetched-opcode counts verified
  nine native MAC.L or DMULS.L multiplications per vertex for the respective
  assembly candidate. With SR.S=1, the MAC wrapper reported mask **5**, executed
  zero MAC.L instructions, retained SR.S and produced exact output. A full-range
  fixture reported mask **0** and all four fixed wrappers matched the oracle.
- Every target call verified stack restoration, preservation of distinct nonzero
  values in R8–R14, and preservation of SR.S.

For Ring world detail 4 (**1,584 vertices**), the median modeled instruction-cycle
counts across 21 poses were:

| Path | Modeled cycles per whole batch |
| --- | ---: |
| Software float | 6,751,532 |
| Prior safe batch | 316,178 |
| Prepared C, including plan | 152,597 |
| Fused MAC.L, including plan | 93,956 |
| Fused DMULS.L, including plan | 136,725 |

Across all 15 workloads, inclusive float/prepared ratios were **43.65–45.41×**,
float/MAC **70.67–73.80×**, and float/DMULS **48.71–50.68×** in this model. Each
prepared or native sample includes one measured plan preparation before its
median is computed. Normal range wrappers and output writes are included;
one-time input conversion and mesh scanning, matrix construction, and precision
checks are excluded. These are full-mesh calls; the console FRT harness uses
32-vertex chunks and has a separate measurement boundary.

The model uses flat RAM and upstream per-opcode cycle increments. It has no
cache, bus wait-state, pipeline or multiplier-contention model, DMA, interrupts,
second CPU or peripheral execution. It does not boot the console image. These
values are not physical-console timings, full-system emulator timings, browser
timings or an FPS result, and are not entered in the console capture schema.
See [the model method, limits and reproduction instructions](benchmarks/sh2-model/README.md).

The [cross workflow](../.github/workflows/sh2-cross.yml) now reruns this optional
model harness on its freshly built ELF and uploads both `raw.json` and validated
`results.json` in the build evidence artifact. The committed capture above remains
attributed to its recorded build; later CI captures have their own ELF and source
hashes. The older CI records below retain their original provenance and describe
the evidence available at that time.

[SH-2 CI 43493549643](https://github.com/lamb2k/SH2ZAM/actions/runs/43493549643)
repeated the execution on the freshly built
[`b79f83c81896bedbf58fc695203af2296c8ea854`](https://github.com/lamb2k/SH2ZAM/commit/b79f83c81896bedbf58fc695203af2296c8ea854)
ELF. All 315 poses, 1,267,560 components per fixed path and both fallback guards
passed again, and all 15 inclusive median cycle counts matched the recorded
capture above. Its raw and validated results are in
[artifact 14069829911](https://github.com/lamb2k/SH2ZAM/actions/runs/43493549643/artifacts/14069829911).

## Historical prepared bounds and fused native candidates

Implementation and shared benchmark:
[`8943e8297f41158796191527639ddc35510c6d4c`](https://github.com/lamb2k/SH2ZAM/commit/8943e8297f41158796191527639ddc35510c6d4c).
This update compares the **prior safe batch** with the new prepared batch. The
historical comparison below used the original full-range scalar implementation
as its baseline; those ratios describe different changes.

- [Host CI 43486731594](https://github.com/lamb2k/SH2ZAM/actions/runs/43486731594) passed GCC Release, Clang Release and Clang ASan/UBSan, with ten CTest targets per configuration and installed-consumer checks. Local strict GCC 14.2, Clang 14 and a GCC sanitizer build also passed all ten targets.
- The prepared oracle executes **2,099,539 checks**. It covers the exact output bound and adjacent fallback, negative fractions, cancellation, saturation, immutable-source overlap rejection, source subranges, matrix snapshots and null/empty/failure contracts. Native instruction-model and wrapper tests execute **2,651,515 checks**; host execution uses portable fallbacks and does not execute SH-2 assembly.
- [Interactive lab CI 43486731606](https://github.com/lamb2k/SH2ZAM/actions/runs/43486731606) passed WASM math, five-kernel capture-schema validation and Chromium UI tests. The WASM suite checked **737,280 coordinates** with no safe/prepared difference, plus identity, kernel-only/inclusive parity and full-range fallback selection. It has no host imports. The shared native-fixture suite checked **1,267,560 coordinates across 315 mesh/pose combinations**, including all five wrappers and full/chunk parity. Maximum float error in those suites remained **0.000015497207641601562 world units**.
- Browser capture now retains **at least 5 ms per sample**, rotates timing order and includes matrix qualification once per prepared batch. Mesh scanning is excluded as one-time input preparation; inputs remain immutable. Separate component timings diagnose setup and kernel cost without constructing the inclusive result from separate medians. Browser tests also cover live updates, all scenes, one slider, mobile width, reduced motion, offline use and nonfinite-output rejection.
- [SH-2 CI 43486731579](https://github.com/lamb2k/SH2ZAM/actions/runs/43486731579) compiled and linked the prepared library and five-kernel benchmark with the pinned Yaul GCC 13.2 toolchain. Both fused loops have **zero hot-loop stack references and zero helper calls**. The MAC kernel saves four registers (16 bytes); DMULS saves six (24 bytes); neither needs an additional local frame. Inspection removed the initial DMULS row-pointer spill. Full arithmetic and memory-load details are in [the fused-kernel audit](benchmarks/FUSED_SH2.md).
- The [native evidence artifact](https://github.com/lamb2k/SH2ZAM/actions/runs/43486731579/artifacts/14067602856) contains disassembly, opcode/spill audits, compiler metadata and boot images. The float bridge still calls software-float helpers. Native candidates remain opt-in and unqualified by console execution. MAC checks SR.S and falls back when saturation mode is enabled; no SR write is introduced.

### Prepared browser measurement

The Chromium capture in run 43486731606 used Ring world detail 4, **1,584
vertices**, and 21 adaptive samples at one paused pose. The three median batch
times include JS-to-WASM calls; prepared includes matrix qualification:

| Path | Median batch time | Meaning |
| --- | ---: | --- |
| Float32 C / WASM | 0.004167 ms | Same quantized inputs and coefficients |
| Prior safe batch | 0.007227 ms | Existing automatic matrix-bound path |
| Prepared batch | 0.005469 ms | New plan qualification + complete transform |

This is **1.32× safe/prepared throughput**, or **24.3% less batch time** than the
prior safe path. The prepared path is still **31.3% slower than float** in this
capture. It has not beaten the browser float reference. These are observed
results from one shared CI environment, not a timing guarantee, FPS result or
Saturn speedup.

The separately measured qualification median was 0.0000404 ms; the kernel-only
median was 0.005469 ms. Independently measured medians need not add exactly to
the inclusive median, particularly at this timer resolution. The full exported
distributions and iteration counts are retained in the lab artifact.

Safe/prepared checksums both were `cc6caa5f`; float was `4f4c1aa8`. Maximum
float error in this capture was one Q16.16 LSB, **0.0000152587890625 world
units**. Module SHA256:
`a141942fe27b1751eed476aa70a7a80bb916a874dd211c873d4d904ac3d5cb55`.

### SIMD experiments and remaining evidence

Private [WASM experiments](benchmarks/wasm-experiments/) preserve three wide
SIMD strategies and two packed signed-16 decomposition strategies. They use
the same optimization flags for float and fixed paths and check exact outputs.
None established a float win, so no experimental SIMD kernel is selected in
the library or production lab. Their Node host distributions are noisy and
are exploratory evidence only.

**Still unmeasured:** execution/correctness and elapsed time on a physical
Saturn, batch-size/alignment/cache effects and dual-CPU scaling. Native schema
3 records software float, safe batch, prepared C, fused MAC and fused DMULS,
adds measured plan qualification once per mesh, and suppresses candidate
summary timings when an availability mask reports fallback. Hardware records
remain empty until a genuine capture exists.

## Historical exact batch-transform update

Implementation: [`31bbf95a6cb8d7562155e061759f346f428ee2fd`](https://github.com/lamb2k/SH2ZAM/commit/31bbf95a6cb8d7562155e061759f346f428ee2fd).

- [Host CI run 43482808025](https://github.com/lamb2k/SH2ZAM/actions/runs/43482808025): GCC Release, Clang Release and Clang Debug with ASan/UBSan passed. Each configuration executes eight CTest targets, including the new independent 128-bit batch oracle. Standalone/installed consumer checks passed. Local strict GCC 14.2 and Clang 14 Release builds, and a GCC sanitizer build, also passed all eight tests.
- The dedicated batch suite executes **2,052,695 checks** against the independent oracle. The unchanged foundation suite executes **1,073,727 checks**. The new suite covers the exact matrix-bound threshold and one unit over it, full-range coordinates, extreme cancellation, negative fractional flooring, saturation, exact in-place operation, odd counts and buffer boundaries. These counts are checks, not coverage percentages.
- [Interactive lab CI run 43482808208](https://github.com/lamb2k/SH2ZAM/actions/runs/43482808208): WASM math, capture-schema validation and Chromium browser tests passed. WASM checks compared **737,280 coordinates**, with zero general/optimized differences and zero module imports. The native shared-fixture test checked **1,267,560 coordinates across 315 mesh/pose combinations**, including full-batch/chunk parity.
- Maximum float comparison error in both fixed-point paths was **0.000015497207641601562 world units** in the WASM suite. Inputs and matrix coefficients are now shared exactly; this is transform rounding error and excludes common matrix/trigonometry quantization. The two fixed-point paths were bit-identical.
- The browser tests exercised continued live sampling, all scenes, detail changes, exact output agreement, the single-slider layout, capture/export, mobile width, reduced motion, offline HTML and rejection of a deliberately injected nonfinite float result.
- [SH-2 CI run 43482808084](https://github.com/lamb2k/SH2ZAM/actions/runs/43482808084): pinned Yaul `sh2eb-elf-gcc 13.2.0`, `-m2 -mb -O2 -ffreestanding`, compiled the library and linked both the diagnostic and three-way benchmark ISO/CUE images. The float bridge retained `__mulsf3` and `__addsf3` dependencies. Existing RWX LOAD-segment linker warnings remain recorded.
- [Native evidence artifact](https://github.com/lamb2k/SH2ZAM/actions/runs/43482808084/artifacts/14066480838) contains compiler metadata, object/archive outputs, disassembly, the static batch audit, float-helper audit and boot images. No boot or console timing is implied by build success.

### Browser measurement example

The Chromium CI capture above used Ring world, detail 4, **1,584 vertices**, 21
samples of 64 batches per path. Median batch times include the JS-to-WASM call:

| Kernel | Browser median per batch |
|---|---:|
| Float32 C / WASM | 0.003125 ms |
| General SH2ZAM | 0.018750 ms |
| Optimized SH2ZAM | 0.009375 ms |

This capture reports **2.00× general/optimized** and **0.33× float/optimized**.
Thus the optimization halved fixed-point transform time in this run, while browser
float remained faster. It is one browser measurement on a shared CI runner, not a
Saturn speedup, a timing guarantee or an FPS result. The live page measures its
current environment rather than displaying these fixed values.

General and optimized output checksums both were `7054995f`; float was `08e695c6`.
Module SHA256: `37a6111ffd5e9064617bb7a30960ca46094254b71583286511e5eb9662c91749`.

### Generated SH-2 code

The specialized function contains the nine native `dmuls.l` instructions for its
three unrolled rows, with **no function/helper calls in the fast path**. Its only
external relocation is the tail fallback to the general batch implementation.
The old implementation also performs nine products per vertex through loops; the
optimization removes repeated quotient/remainder bookkeeping and scalar-call setup,
not mathematical products.

The compiler copies 12 matrix values into a **48-byte local stack area** and saves
seven registers (76 bytes total stack adjustment). Coefficients are loaded from that
area in the vertex loop. They are not all retained in registers. This is a concrete
remaining register-allocation/memory cost to investigate with target measurements.
Static instruction counts and stack references do not predict elapsed cycles.

**Still unmeasured:** execution/correctness and timings on a physical Saturn,
native batch-size/alignment/cache effects and dual-CPU scaling. Capture schema 2
keeps all hardware times empty until evidenced measurements exist.

## Original foundation validation

Implementation tested: `fbb6f3d67554316d13a3219ace3a6c3eae71c7b9`. Later documentation/image-pin commits do not change these math sources.

## Completed

- [Host CI run 43465864295](https://github.com/lamb2k/SH2ZAM/actions/runs/43465864295): GCC Release, Clang Release, and Clang Debug with AddressSanitizer/UndefinedBehaviorSanitizer all passed. Each executes seven CTest targets. Release builds also passed standalone-library build and installed-package consumer checks.
- `foundation_oracles`: 1,073,727 executed checks per run. Includes 50,000 randomized scalar samples, 20,000 vector samples, 1,000 matrix products, full 65,536-angle sweep, complete Q16.16 asin input grid in [-1,1], overflow/cancellation/invalid/aliasing regressions. Counts describe checks, not line or branch coverage.
- Local GCC 14.2 and Clang 17 builds passed, including Clang ASan/UBSan. The local full-turn sine sweep differed from the rounded host reference by at most one Q16.16 raw unit; round-tripping the library's own sin/cos through atan2 differed by zero angle units. These observations are not global error proofs or hardware timings.
- [SH-2 CI run 43465864349](https://github.com/lamb2k/SH2ZAM/actions/runs/43465864349): all five library C sources and the freestanding examples compiled with `sh2eb-elf-gcc 13.2.0`, `-m2 -mb -O2 -ffreestanding`; ELF headers identify big-endian SH-2 and disassembly contains `dmuls.l`.
- The same SH-2 job linked the Yaul diagnostic app and generated `sh2zam-smoke.elf`, `.bin`, `.iso` and `.cue`. It emitted RWX LOAD-segment linker warnings; these are recorded, not hidden or claimed resolved.
- [Build evidence artifact](https://github.com/lamb2k/SH2ZAM/actions/runs/43465864349/artifacts/14058036876) contains the archive, compiler/image metadata, ELF/disassembly/undefined-symbol reports and diagnostic build outputs. Artifacts may expire.

Toolchain image used: `ijacquez/yaul@sha256:b6d272204767cd0a734139dd3a2ab298ef5912fb7b7224b78e6d9fac1a7825a6`. The cross workflow is pinned to this observed digest for repeatability.

## Not completed / not claimed in the foundation record

At the time of this foundation record, no emulator or physical Saturn execution,
hardware timing, master/slave concurrency stress, ISR register-preservation audit,
or production game integration had been performed. The isolated SH-2 execution
recorded above was added later. Physical console execution and timing, concurrency
stress, ISR integration and production game integration remain unmeasured. The
foundation diagnostic mask was zero on the host; no console mask is reported.
No superiority to Yaul, SaturnMath++ or existing game code is asserted.

## Observed runtime helpers

The archive references integer/runtime helpers such as `___divdi3`, `___ashlsi3_r0`, `___lshrsi3_r0`, `___movmem` and `___udivsi3`. Yaul successfully resolves the demo's dependencies. A static archive alone does not prove that a different SDK's runtime or ABI is compatible. Hardware DIVU is deliberately not enabled.
