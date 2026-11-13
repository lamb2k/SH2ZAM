# Executed SH-2 instruction model

This harness executes the **actual big-endian SH-2 benchmark ELF**, including its
software-float runtime helpers and native MAC/DMULS instructions, through the
unmodified Yabause SH-2 interpreter. The fixed kernels beat the software-float
reference in this isolated instruction model. **These are not physical Saturn
measurements, full-system emulator timings, or browser timings.**

The pinned source is
[Yabause/yabause `7e38821dbac265490f115e163c523a939acda759`](https://github.com/Yabause/yabause/tree/7e38821dbac265490f115e163c523a939acda759),
specifically [`yabause/src/sh2int.c`](https://github.com/Yabause/yabause/blob/7e38821dbac265490f115e163c523a939acda759/yabause/src/sh2int.c).
No opcode implementation, branch behavior, arithmetic operation or per-opcode
cycle increment is changed. The interpreter source is fetched during the optional
model build; it is not vendored into the math library.

## What is executed

The recorded ELF comes from main commit
[`26a935eea2c6fb9a7dfba6a81d273ab81562a2eb`](https://github.com/lamb2k/SH2ZAM/commit/26a935eea2c6fb9a7dfba6a81d273ab81562a2eb),
[SH-2 build 43487712557](https://github.com/lamb2k/SH2ZAM/actions/runs/43487712557),
[artifact 14068406384](https://github.com/lamb2k/SH2ZAM/actions/runs/43487712557/artifacts/14068406384).
It was built with the project's pinned Yaul GCC 13.2 toolchain. Its SHA256 is
`38a7c875c749c0348f886405f30224d7e4ea8846c9e0f3341f173e7fe12a89a6`.

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

Each pose executes software float, the safe batch, portable prepared C, native
MAC.L and native DMULS.L. All four fixed outputs are compared coordinate by
coordinate with an independent host `__int128` oracle using the actual input and
quantized matrix read from target RAM. Float outputs are checked separately.
The final capture checked **1,267,560 coordinates per fixed path**, with zero
fixed mismatches and maximum float error **0.000015497207641601562 world units**
(about **1.016 Q16.16 units**, within the two-unit comparison bound).
All 315 benchmark poses report availability mask 7, and instruction instrumentation
confirms nine MAC.L or DMULS.L multiplications per vertex in the respective native
candidate, rather than a portable fallback.

Two additional actual target-execution checks exercise the dispatch guards:

- **SR.S = 1:** MAC availability is false, the MAC wrapper executes zero MAC.L
  instructions, output remains exact, and SR.S is preserved.
- **Full-range input and matrix:** the prepared proof fails, all native candidate
  availability bits are clear, and all four fixed wrappers agree with the
  independent saturation/rounding oracle. No MAC.L instruction executes.

These checks execute the assembly through the CPU interpreter. They complement
host arithmetic models; they do not certify a console's interrupt handlers,
cache coherency, startup settings or bus behavior.

## Results

The complete machine-readable evidence is [results.json](results.json). Values
below are medians across the 21 poses, in **Yabause's modeled instruction-cycle
units per complete batch**. Prepared, MAC and DMULS include one measured
`plan_prepare()` call per batch, added before taking the median.

| Path | Ring world d4, 1,584 vertices | Cubes d4, 1,728 vertices | Torus d4, 2,016 vertices |
| --- | ---: | ---: | ---: |
| Software float | 6,751,532 | 7,556,934 | 8,804,596 |
| Prior safe batch | 316,178 | 344,885 | 402,344 |
| Prepared C, including plan | 152,597 | 166,421 | 194,069 |
| Fused MAC.L, including plan | 93,956 | 102,452 | 119,444 |
| Fused DMULS.L, including plan | 136,725 | 149,109 | 173,877 |
| Software float / MAC | 71.86× | 73.76× | 73.71× |

Across all 15 workloads, the model's inclusive float/prepared ratio is
**43.65–45.41×**, float/MAC is **70.67–73.80×**, and float/DMULS is
**48.71–50.68×**. This answers a target-instruction question: the actual SH-2
software-float helper path performs far more work than these integer kernels.
It does not establish those speedups on a real console.

Guest instruction counts are recorded independently from the cycle counter.
They count every fetched opcode, including delay slots, without changing
instruction semantics. For Ring world d4, median counts including plan
preparation are 5,893,934 for float, 278,932 for safe, 136,703 for prepared,
63,813 for MAC and 120,838 for DMULS. Memory reads/writes and native multiplication
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
prepared/MAC/DMULS and reported separately. Oracle checks run after measurement.

## Reproduce

Prerequisites: a host C compiler with `__int128`, Python 3, Git, and the two
benchmark files from the native CI artifact. No Saturn BIOS, game image,
commercial ROM or local SH crosscompiler is needed.

From the repository root, after extracting the native evidence artifact:

```sh
MODEL_EVIDENCE_URL=https://github.com/lamb2k/SH2ZAM/actions/runs/43487712557/artifacts/14068406384 \
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
`float-safe-prepared-fused-sh2-model-v1`. `kernelCycles` and `kernelInstructions`
hold 21 raw values per path. `planPreparationCycles` and
`planPreparationInstructions` hold 21 setup values. `medianCycles` and
`medianInstructions` include setup for prepared/MAC/DMULS before the median.
Per-pose checksums, availability, numerical error, metadata and all source/ELF
hashes remain available to audit the result.

## License separation

This optional harness includes Yabause's GPL interpreter when built. Accordingly,
`harness.c` and the scripts in this directory are distributed under
**GPL-2.0-or-later**, with the license text in [LICENSE](LICENSE). Yabause's own
copyright and license notices remain in its fetched source. The SH2ZAM math
library remains MIT-licensed; this harness is not linked into it, installed with
it, or included in the browser module. The native ELF is loaded as guest data.
