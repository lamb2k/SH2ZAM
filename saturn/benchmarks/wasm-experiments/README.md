# Private WASM SIMD experiments

These experiments did **not beat the float reference**. They remain separate
from the public library, the exhibit build, and default dispatch. The production
prepared executor remains the simpler portable implementation.

The purpose is to preserve the actual candidates, exactness checks, and timing
data so another compiler or engine can repeat the comparison. This is Node/V8
on an x86 host. WASM float executes using the host's floating-point facilities;
it is **not a measurement of software float on an SH-2**.

## Reproduce

From the repository root, with a Clang/lld installation supporting WASM SIMD:

```sh
CLANG=clang bash saturn/benchmarks/wasm-experiments/build.sh
COMPILER_ID="$(clang --version | head -n 1)" SAMPLE_MS=10 \
  node saturn/benchmarks/wasm-experiments/run.cjs \
  > build/wasm-experiments/results.json
```

`OUT_DIR` changes the build directory. Pass the resulting `math.wasm` path as
the runner's first argument if it is outside the default directory. `DETAILS`
selects mesh detail levels, for example `DETAILS=1,2,3,4,5`. The saved capture
uses detail 4 for the ring world, cubes, and torus, and 21 rotation poses per
mesh. `SAMPLE_MS` must be at least 5; the capture uses 10.

The recorded run used Debian Clang 14.0.6/lld 14, Node v24.22.1, V8
13.6.233.17-node.58, and an x64 Linux host reporting an Intel Xeon Platinum
8573C and nine logical CPUs. The local lld 14 wrapper removed
`-Wl,--export-memory`, because that linker exports the memory without this
newer flag. It did not change optimization or floating-point flags.

Every translation unit uses the same `-O3 -msimd128 -fno-fast-math
-ffp-contract=off` settings. LLVM may automatically vectorize the float
reference and ordinary fixed kernels. The reference is the normal affine
float loop, with no added checks or conversion work in its timed body.
`SH2Z_FORCE_PORTABLE=1` excludes SH-2 instructions. There are no host imports.

## Candidates

| Runner name | Timed operation | Extra input storage |
| --- | --- | --- |
| `before` | Normal float affine transform | Float copy, 12 bytes/vertex |
| `after` | Existing safe `sh2z_mat34_transform_points` | None |
| `prepared` | Prepared portable transform kernel | None |
| `prepared_inclusive` | Plan initialization plus prepared portable transform | None |
| `wide` | Four vertices, unrolled rows, signed 32-to-64 widening multiply, AoS input | None |
| `packed` | Same widening arithmetic with prepacked four-vertex input blocks | 12 bytes/vertex |
| `rows` | Three matrix rows in vector lanes for each vertex | None |
| `split` | Exact signed16 product decomposition using `i32x4.dot_i16x8` | 16 bytes/vertex |
| `narrow` | Signed16 decomposition with a cheaper residual sum when its bound qualifies | 16 bytes/vertex |

All results are written as ordinary contiguous xyz structures. Wide candidates
use `i64x2.extmul_low_i32x4` and its high counterpart. Byte shuffles extract
bits 16 through 47, avoiding the especially unattractive `i64x2.mul` and signed
64-bit vector-shift lowering. The packed candidates eliminate input
transposition but still pay for ordinary AoS output. The row-oriented variant
avoids transposing four input vertices at the cost of an unused fourth row.

The signed16 candidate writes each input or coefficient as
`high * 65536 + signed_low`. It uses exact partial products, adds the integer
terms, and floors the residual once. The general split variant computes a
carry between two residual sums. The narrow variant can omit that carry
bookkeeping when each row's sum of absolute signed-low coefficients is at most
65535. That narrower condition qualified for 15 of the 21 exhibit poses.
These are exact decompositions; they do not reduce coordinate or matrix
precision.

Every candidate first requires the prepared plan's output-fit proof. The
signed16 variants also reject rounded high halves which do not fit int16, and
reject the signed16 pair-overflow corner case. Ineligible cases use the
ordinary prepared executor, whose own full-range fallback remains intact.
Tails of fewer than four vertices use the prepared executor. The runner
deliberately exercises these branches.

## Recorded exploratory results

The complete capture is [results-node-v8.json](results-node-v8.json). It includes
567 retained timing samples, source/dependency and WASM hashes, all per-pose
eligibility results, one-time preparation costs, matrix preparation costs,
precision checks, and distributions. Every retained transform sample lasted
at least 10.006 ms. Shorter calibration attempts are discarded. Path order
rotates at every pose, and each pose receives warm-up calls before timing.

Median microseconds per complete batch:

| Operation | Ring world, 1,584 vertices | Cubes, 1,728 vertices | Torus, 2,016 vertices |
| --- | ---: | ---: | ---: |
| Float reference | 5.022 | 3.089 | 6.767 |
| Current safe fixed batch | 45.191 | 29.215 | 70.208 |
| Prepared portable, kernel only | 13.491 | 6.795 | 18.123 |
| **Prepared portable, including plan** | **11.905** | **7.086** | **13.411** |
| Wide SIMD, AoS | 28.638 | 17.211 | 29.271 |
| Wide SIMD, packed input | 24.487 | 15.358 | 30.480 |
| Wide SIMD, rows per vertex | 44.839 | 24.958 | 53.660 |
| Signed16 split, packed input | 11.298 | 5.867 | 13.623 |
| Signed16 narrow, packed input | 8.964 | 5.226 | 13.802 |

**The host was noisy even while our other builds were paused.** The table
reports this capture, rather than a stable machine ranking. For example,
including plan preparation sometimes has a lower observed median than the
kernel alone; that is timing variability, not negative preparation cost.
The p10–p90 ranges below make the uncertainty visible:

| Operation | Ring world, µs | Cubes, µs | Torus, µs |
| --- | ---: | ---: | ---: |
| Float | 3.672–12.011 | 2.575–5.860 | 4.154–13.116 |
| Prepared including plan | 8.013–37.514 | 5.763–14.984 | 8.515–80.371 |
| Signed16 narrow | 6.752–11.773 | 4.316–6.882 | 6.894–39.786 |

The inclusive prepared medians are 3.80×, 4.12× and 5.24× faster than the
current safe batch **in this SIMD-enabled build**, but remain 1.98–2.37× slower
than float. The packed narrow medians remain 1.69–2.04× slower than float.
These ratios should not be transferred to the production exhibit's default
non-SIMD build, to other engines, or to Saturn hardware. Broad timing
distributions also preclude a precise claim about the smaller differences
between prepared portable and the narrower packed candidates.

### Preparation and packing accounting

Input vertices and matrix coefficients are quantized identically before any
transform. Float input conversion, mesh-bound scanning, and optional input
packing happen once for an immutable mesh. Their medians were measured
separately:

| One-time operation | Ring world, µs | Cubes, µs | Torus, µs |
| --- | ---: | ---: | ---: |
| Float input conversion | 3.838 | 4.963 | 5.095 |
| Prepared mesh-bound scan | 1.601 | 1.334 | 2.276 |
| Wide input packing | 3.457 | 2.518 | 2.648 |
| Signed16 input packing and validation | 19.111 | 11.910 | 12.494 |

Those one-time costs are excluded from the steady-state transform table. A
deforming or otherwise edited mesh must pay them again; the packed input may
not be reused after its source changes. Setup neither changes nor compresses
the original coordinates. The extra packed storage is retained alongside the
original mesh so scalar tails and fallbacks can read it.

Plan initialization is per matrix, not per vertex. Its separately measured
medians were about 0.045, 0.022 and 0.048 µs. `prepared_inclusive` measures it
inside every timed transform invocation. The experimental SIMD transform
rows include their candidate-specific eligibility checks, but exclude common
plan initialization; add the measured plan preparation cost when comparing
their full per-matrix work. Matrix construction/trigonometry and conversion
of quantized coefficients to float are outside the kernel table for all
paths. Float matrix conversion is also recorded separately in the JSON.
No rendering, data upload or JS-array conversion is timed.

## Precision evidence and decision

The runner passes 103 independent BigInt oracle cases, including zero counts,
vector tails, negative fractions, signed16 pair overflow, rounded-high
overflow, extreme int32 values, saturation and fallback. Every candidate is
checked immediately after execution. Every one of the 63 timed exhibit poses
also checks all fixed outputs against the original full-range implementation.
The run checked 339,093 output components per fixed path, with no fixed-output
disagreement. Maximum observed float/fixed difference was
`0.000015497207641601562` coordinate units, within the two-Q16-unit check.

Widening multiplication, packed layouts, and explicit vectors did not produce
a float win. Signed16 dot products recover some of the gap but require extra
packing, memory, eligibility rules and more complicated arithmetic. Keeping
those experiments private avoids burdening the public library with a
backend that has no measured advantage over the reference. A future engine
or hardware result can revisit the decision using the same reproducible
checks and complete preparation accounting.
