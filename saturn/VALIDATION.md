# Validation record — 2026-11-13

## Exact batch-transform update

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

## Not completed / not claimed

No emulator or physical Saturn execution, hardware timing, master/slave concurrency stress, ISR register-preservation audit, or production game integration has been performed here. The diagnostic mask was zero on the host; no console mask is reported. No superiority to Yaul, SaturnMath++ or existing game code is asserted.

## Observed runtime helpers

The archive references integer/runtime helpers such as `___divdi3`, `___ashlsi3_r0`, `___lshrsi3_r0`, `___movmem` and `___udivsi3`. Yaul successfully resolves the demo's dependencies. A static archive alone does not prove that a different SDK's runtime or ABI is compatible. Hardware DIVU is deliberately not enabled.
