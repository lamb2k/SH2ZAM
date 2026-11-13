# Executed SH-2 instruction model

This harness executes the **actual big-endian SH-2 benchmark ELF**, including its
software-float runtime helpers and native MAC/DMULS instructions, through the
unmodified Yabause SH-2 interpreter. Against an independent plain C fixed-point
baseline, the optional native MAC.L kernel uses **22.90–24.24% fewer modeled
cycles** across all 15 workloads. The standard prepared C path uses
**23.14–23.82% more cycles** than that baseline. Every fixed path beats software
float, but adopting the library does not produce a universal performance win.
**These are not physical Saturn measurements, full-system emulator timings,
or browser timings.**

The pinned source is
[Yabause/yabause `7e38821dbac265490f115e163c523a939acda759`](https://github.com/Yabause/yabause/tree/7e38821dbac265490f115e163c523a939acda759),
specifically [`yabause/src/sh2int.c`](https://github.com/Yabause/yabause/blob/7e38821dbac265490f115e163c523a939acda759/yabause/src/sh2int.c).
No opcode implementation, branch behavior, arithmetic operation or per-opcode
cycle increment is changed. The interpreter source is fetched during the optional
model build; it is not vendored into the math library.

## What is executed

The recorded ELF comes from source commit
[`29a4a5b43bd4fd83cd09f64b2662e819d1a1df8d`](https://github.com/lamb2k/SH2ZAM/commit/29a4a5b43bd4fd83cd09f64b2662e819d1a1df8d),
[SH-2 build 43510305134](https://github.com/lamb2k/SH2ZAM/actions/runs/43510305134),
[artifact 14077414299](https://github.com/lamb2k/SH2ZAM/actions/runs/43510305134/artifacts/14077414299).
It was built with the project's pinned Yaul GCC 13.2 toolchain. Its SHA256 is
`d3eb9af325c03f142711507177fc4b6f9cbdb756fadc2ebc3afbf28cb84e4c46`.

The harness loads the ELF's RAM segments, then invokes its compiled bridge
functions using the SH-2 calling convention. The data, poses, matrices, input
conversion, bound proofs, output writes and libgcc software-float routines are
all target instructions. BIOS startup and the display/timer capture program are
not executed. The bridge functions are called directly, with a separate stack in
unused RAM and a sentinel return address. Stack balance, callee-saved registers,
and preservation of SR.S are checked after every call.

The workloads are the same 15 exhibit meshes and 21 poses embedded in the ELF.
The collector verifies the source commit and fixture SHA against strings in that
ELF, and checks the workload metadata against the fixture manifest. Changing
geometry or the recorded build invalidates old evidence.

Each pose executes software float, independent plain C fixed point, the SH2ZAM
safe batch, prepared C, native MAC.L and native DMULS.L. All five fixed outputs
are compared coordinate by
coordinate with an independent host `__int128` oracle using the actual input and
quantized matrix read from target RAM. Float outputs are checked separately.
The final capture checked **1,267,560 coordinates per fixed path**, with zero
fixed mismatches and maximum float error **0.000015497207641601562 world units**
(about **1.016 Q16.16 units**, within the two-unit comparison bound).
All 315 benchmark poses report availability mask 7, and instruction instrumentation
confirms nine MAC.L or DMULS.L multiplications per vertex in the respective native
candidate, rather than a portable fallback. The independent plain kernel also
executes exactly nine DMULS.L instructions per vertex and zero MAC.L instructions.

Two additional actual target-execution checks exercise the dispatch guards:

- **SR.S = 1:** MAC availability is false, the MAC wrapper executes zero MAC.L
  instructions, output remains exact, and SR.S is preserved.
- **Full-range input and matrix:** the prepared proof fails, all native candidate
  availability bits are clear, and all four fixed wrappers agree with the
  independent saturation/rounding oracle. No MAC.L instruction executes. The
  independent bounded baseline rejects the input without modifying its output;
  it does not borrow the library fallback.

These checks execute the assembly through the CPU interpreter. They complement
host arithmetic models; they do not certify a console's interrupt handlers,
cache coherency, startup settings or bus behavior.

## Results

### Independent fixed-point adoption baseline

Schema 2 adds a **plain C Q16.16** transform alongside software float and the
four SH2ZAM paths. This is a separate implementation in `exhibits/bridge.c`:
`plain_prepare()` and `plain_range()` call no SH2ZAM arithmetic, bounds-proof,
or transform routine. The benchmark shares only vector/matrix data containers.
It caches all 12 matrix coefficients, unrolls the three output rows, uses
64-bit products and accumulation, floors each row once, and writes every
coordinate. GCC's arithmetic signed right shift is explicitly required with
a compile-time assertion. It uses the same SH-2 optimization flags as the
library's C implementation.

An independent immutable mesh scan caches the maximum coordinate magnitudes
outside timing. Each measured batch includes a new independent per-matrix
proof and matrix snapshot. The proof requires each row's sum of absolute
products, including translation, to remain below `2^47`; this matches the
bounded numerical domain of the library's prepared path. Full-range inputs
are rejected without writing. They are **not** silently delegated to SH2ZAM
and cannot become published plain-C timings. The SH2ZAM paths continue to
exercise their general fallback independently.

The baseline is intentionally competent: no deliberately repeated matrix
setup, per-product rounding, avoidable general-range bookkeeping, or disabled
compiler optimizations. Its wrapper specializes to the benchmark's known
separate input/output arrays, while the library API retains its general
argument and overlap checks. A ratio below 1 is a valid result and must remain
visible; the experiment does not assume a library call beats bespoke C.

Schema 2 records `plainPreparationCycles`, `plainPreparationInstructions`,
`kernelCycles.plain`, exact plain output checksums, and inclusive
`medianCycles.plain`. `plainOver` compares this independent baseline with each
library path. The source and ELF hashes identify each capture. Historical
schema 1 captures have no independent fixed-point baseline; their `safe` row
is already SH2ZAM and must never be labeled "without SH2ZAM."

The complete machine-readable evidence is [results.json](results.json). Values
below are medians across the 21 poses, in **Yabause's modeled instruction-cycle
units per complete batch**. Plain C includes one measured `plain_prepare()`
call. Prepared, MAC and DMULS each include one measured `plan_prepare()` call.
Setup is added to each pose before taking the median.

| Path | Ring world d4, 1,584 vertices | Cubes d4, 1,728 vertices | Torus d4, 2,016 vertices |
| --- | ---: | ---: | ---: |
| Plain C fixed point, including proof | 123,881 | 135,113 | 157,577 |
| SH2ZAM prepared C, including plan | 152,597 | 166,421 | 194,069 |
| SH2ZAM optional MAC.L, including plan | 93,956 | 102,452 | 119,444 |
| SH2ZAM optional DMULS.L, including plan | 136,725 | 149,109 | 173,877 |
| SH2ZAM prior safe batch | 316,178 | 344,885 | 402,344 |
| Software float | 6,751,532 | 7,556,934 | 8,804,596 |
| Plain C / MAC | 1.319× | 1.319× | 1.319× |
| MAC cycle reduction versus plain C | 24.16% | 24.17% | 24.20% |

Across all 15 workloads, the optional MAC.L kernel uses **22.90–24.24% fewer
cycles than independent plain C fixed point** (plain/MAC ratio
**1.297–1.320×**). Standard prepared C uses **23.14–23.82% more cycles**, and the
optional DMULS.L kernel uses **10.32–11.06% more cycles**. These results include
each path's own per-matrix preparation. MAC.L is an optional experimental entry
point requiring its documented bounds and SR.S conditions; it is not the
standard prepared dispatcher.

For migration from software float, the inclusive float/prepared ratio is
**43.65–45.41×**, float/MAC is **70.67–73.80×**, and float/DMULS is
**48.71–50.68×**. That larger gain includes changing from software floating-point
to fixed-point arithmetic and must not be presented as the benefit over an
existing fixed-point implementation. None of these ratios establishes a
physical-console speedup.

Guest instruction counts are recorded independently from the cycle counter.
They count every fetched opcode, including delay slots, without changing
instruction semantics. For Ring world d4, median counts including plan
preparation are 108,001 for independent plain C, 136,703 for prepared, 63,813
for MAC, 120,838 for DMULS, 278,932 for safe and 5,893,934 for float. Memory reads/writes and native multiplication
opcode counts are also retained per pose. None is a host wall-clock measurement.

## Measurement boundary and limits

The core increments its cycle counter according to each decoded instruction.
For example, this interpreter assigns three cycles to MAC.L and two to DMULS.L.
The harness adds **no made-up latency** for memory or other hardware. It records
that upstream counter as a modeled value rather than converting it to claimed
Saturn milliseconds or FPS.

This model deliberately has:

- Flat, big-endian high-work RAM and immediate instruction/data access.
- **No cache model, bus wait states, pipeline or multiplier contention.**
- No DMA, timer, interrupt, second CPU, renderer or other peripheral activity.
- No BIOS, boot sequence, SDK timer integration or full-console execution.

Unsupported hardware access, an invalid opcode, an out-of-RAM access, a runaway
function, an ABI violation or any output mismatch causes failure. The benchmark
bypasses only the outer scheduler, debugger and idle-loop detector; opcode
handlers and their internal delay-slot execution are unchanged.

Every measured path includes its normal range wrapper and all output stores.
Whole-mesh calls are used here; the separate console FRT harness uses 32-vertex
chunks. Model timings therefore must not be inserted into the console capture
schema or relabeled as results from that timer harness. One-time input conversion
and mesh scanning, matrix construction and float coefficient conversion are
outside steady-state transform costs. Plan qualification is included once for
prepared/MAC/DMULS and reported separately. Plain C includes its independent
bound proof and matrix snapshot once per batch, also reported separately.
Oracle checks run after measurement.

## Reproduce

Prerequisites: a host C compiler with `__int128`, Python 3, Git, and the two
benchmark files from the native CI artifact. No Saturn BIOS, game image,
commercial ROM or local SH crosscompiler is needed.

From the repository root, after extracting the native evidence artifact:

```sh
MODEL_EVIDENCE_URL=https://github.com/lamb2k/SH2ZAM/actions/runs/43510305134/artifacts/14077414299 \
  bash saturn/benchmarks/sh2-model/run.sh \
  /path/to/saturn/benchmarks/yaul/build/sh2zam-benchmark.elf \
  /path/to/saturn/benchmarks/yaul/build/sh2zam-benchmark.sym \
  build/sh2-model/results.json
```

`MODEL_BUILD_DIR` changes the temporary build directory. `YABAUSE_SOURCE` may
point to an existing clean checkout of the exact pinned upstream commit. The
script verifies the commit and tracked source before compiling. It builds the
host interpreter harness with `-O2 -fwrapv -fno-strict-aliasing`, runs all 315
poses and both guard checks, and validates/packages the data with `collect.py`.
Normal operation leaves all downloaded source, host executables, raw results and
ELFs under ignored build storage. It does not overwrite this committed capture
unless its output path is explicitly selected.

The result uses an independent schema and kernel set
`float-plain-fixed-safe-prepared-fused-sh2-model-v2`. `kernelCycles` and
`kernelInstructions` hold 21 raw values per path. `planPreparationCycles` and
`planPreparationInstructions` hold 21 library setup values;
`plainPreparationCycles` and `plainPreparationInstructions` hold 21 independent
baseline setup values. `medianCycles` and `medianInstructions` include the
applicable setup for plain/prepared/MAC/DMULS before the median.
Per-pose checksums, availability, numerical error, metadata and all source/ELF
hashes remain available to audit the result.

## License separation

This optional harness includes Yabause's GPL interpreter when built. Accordingly,
`harness.c` and the scripts in this directory are distributed under
**GPL-2.0-or-later**, with the license text in [LICENSE](LICENSE). Yabause's own
copyright and license notices remain in its fetched source. The SH2ZAM math
library remains MIT-licensed; this harness is not linked into it, installed with
it, or included in the browser module. The native ELF is loaded as guest data.
