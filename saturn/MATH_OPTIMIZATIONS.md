# Exact math optimizations for SH-2

The optimized library improves **40 of the 48 measured operations** across all
three recorded input profiles. The other eight operations retain their previous
cost. All **144 operation/profile medians** and **1,008 individual batch samples**
are faster or unchanged in this capture, with **114,912 output words matching
exactly**. These results compare previous SH2ZAM with optimized SH2ZAM; the
[independent plain-C and software-float adoption comparison](benchmarks/sh2-model/README.md)
remains a separate vertex workload.

[Native CI evidence](https://github.com/lamb2k/SH2ZAM/actions/runs/43518264229) executes the compiled big-endian SH-2
ELF from [`fbbd984b`](https://github.com/lamb2k/SH2ZAM/commit/fbbd984b184ad329e772ec519cdcdf0a427bc690).
The [complete dataset](benchmarks/math-model/results.json) retains every raw
sample, input/output checksum, toolchain/fixture identity and limitation.
ELF SHA-256: `47acbcd28b46ffbdede2c61b4d88e3768434b9cc3901fe369e4e440911993ec6`.

## What changes for adopters

Existing calls use the optimized implementations automatically after rebuilding
or relinking the library. Public types, signatures, saturation, rounding,
zero/invalid-input behavior and matrix aliasing contracts remain unchanged.
There is no new setup object, allocation, approximate mode or caller-supplied
range promise for these operations.

| Area | Exact implementation change |
| --- | --- |
| Square root and vector length | Run integer-root digit loops with 32-bit state; use native 32×32-to-64 products for component squares. |
| Vector normalization | Preserve the existing rescaling and truncation; obtain each normalized component with 16 exact fractional quotient steps. |
| Vec2 dot and 2D/3D cross | Accumulate the raw products directly and floor once; avoid temporary vectors and nested calls in cross3. |
| Matrix multiplication | Check each left row once, reuse the proof across output columns, and accumulate products directly on the proved path. |
| Scalar matrix transforms | Prove the input vector once, reuse it across explicit output rows, and avoid intermediate component/output arrays. |
| Quaternion multiplication | Prove one four-component magnitude sum for all four output components; preserve the original fallback. |
| Sine/cosine | Keep the same 257-entry table and interpolation outputs, use one interpolation multiply, and share work in sincos. |
| Inverse trig | Search the existing quarter-wave table directly, then refine within one cached cell; preserve the exact comparison and tie rule. |
| Projection | Compute both bounded screen-coordinate quotients in one restoring-division loop instead of two general 64-bit divisions. |
| Rigid inverse | Check six unique row pairs instead of repeating all nine symmetric orthonormality checks. |
| Sphere collision | Use exact unsigned32 coordinate distances and 32×32 squares; preserve overflow-safe radius subtraction. |

Camera construction, quaternion normalization/nlerp/rotation, and perspective
inherit improvements from the shared routines they call. The table below
reports their full API costs, including the unchanged portions.

## Recorded costs

Both versions are compiled with the same pinned Yaul GCC 13.2 toolchain for
`-m2 -mb -O2`, with fast-math and LTO disabled. Each cell is the median of seven
deterministic batches of 32 API calls, including wrappers and output writes.
Fixtures cover ordinary bounded inputs, tiny raw values, and signed32 full-range
values/edge cases. See the [exact fixture definitions and reproduction steps](benchmarks/math-model/README.md).

Positive percentages below mean fewer **modeled instruction cycles**. Zero
means unchanged. These are results for the recorded fixtures, not a guarantee
that every possible input or game will improve.

| API (sh2z_ prefix omitted) | Prior bounded cycles/call | Optimized bounded cycles/call | Bounded saved | Tiny saved | Full-range saved |
| --- | ---: | ---: | ---: | ---: | ---: |
| `mul` | 41.34 | 41.34 | 0.00% | 0.00% | 0.00% |
| `div` | 266.09 | 266.09 | 0.00% | 0.00% | 0.00% |
| `muldiv` | 269.62 | 269.62 | 0.00% | 0.00% | 0.00% |
| `recip` | 258.44 | 258.44 | 0.00% | 0.00% | 0.00% |
| `sqrt` | 798.34 | 445.28 | 44.22% | 76.90% | 42.05% |
| `rsqrt` | 1,025.09 | 689.34 | 32.75% | 57.98% | 32.05% |
| `lerp` | 60.84 | 60.84 | 0.00% | 0.00% | 0.00% |
| `smoothstep` | 74.75 | 74.75 | 0.00% | 0.00% | 0.00% |
| `vec2_dot` | 89.78 | 73.59 | 18.03% | 18.62% | 16.38% |
| `vec3_dot` | 158.72 | 158.72 | 0.00% | 0.00% | 0.00% |
| `vec4_dot` | 198.88 | 198.88 | 0.00% | 0.00% | 0.00% |
| `vec2_cross` | 94.19 | 58.53 | 37.86% | 37.97% | 37.88% |
| `vec3_cross` | 384.56 | 237.91 | 38.14% | 38.20% | 37.51% |
| `vec2_length` | 898.62 | 538.09 | 40.12% | 76.05% | 40.87% |
| `vec2_normalize` | 1,857.66 | 1,159.91 | 37.56% | 40.03% | 46.09% |
| `vec3_length` | 960.59 | 597.81 | 37.77% | 70.30% | 36.83% |
| `vec3_normalize` | 2,195.56 | 1,400.19 | 36.23% | 38.95% | 47.81% |
| `vec4_length` | 984.00 | 621.91 | 36.80% | 68.12% | 34.79% |
| `vec4_normalize` | 2,540.09 | 1,647.06 | 35.16% | 37.42% | 48.29% |
| `mat22_multiply` | 434.12 | 341.25 | 21.39% | 21.60% | 20.85% |
| `mat33_multiply` | 1,283.66 | 778.62 | 39.34% | 39.41% | 31.47% |
| `mat34_multiply` | 1,743.84 | 1,020.06 | 41.50% | 42.06% | 34.44% |
| `mat44_multiply` | 2,780.84 | 1,609.25 | 42.13% | 42.09% | 22.23% |
| `mat22_transform` | 246.34 | 181.66 | 26.26% | 26.36% | 23.53% |
| `mat33_transform` | 467.75 | 305.06 | 34.78% | 34.96% | 29.11% |
| `mat34_transform_point` | 474.78 | 343.81 | 27.59% | 29.40% | 22.14% |
| `mat34_transform_direction` | 467.53 | 305.12 | 34.74% | 34.94% | 29.13% |
| `mat44_transform` | 719.09 | 465.41 | 35.28% | 35.26% | 13.74% |
| `quat_multiply` | 856.22 | 486.03 | 43.24% | 43.17% | 19.49% |
| `quat_normalize` | 2,564.69 | 1,671.72 | 34.82% | 37.06% | 47.91% |
| `quat_nlerp` | 8,309.50 | 5,504.59 | 33.76% | 35.33% | 43.47% |
| `quat_to_mat34` | 2,964.72 | 2,072.03 | 30.11% | 32.19% | 42.48% |
| `quat_rotate` | 3,339.25 | 2,284.25 | 31.59% | 33.45% | 42.08% |
| `quat_axis_angle` | 5,012.25 | 3,253.81 | 35.08% | 36.53% | 41.32% |
| `sin` | 68.62 | 64.19 | 6.47% | 6.56% | 6.59% |
| `cos` | 69.81 | 69.28 | 0.76% | 0.71% | 0.80% |
| `sincos` | 157.91 | 109.38 | 30.73% | 30.88% | 30.84% |
| `tan` | 384.47 | 360.56 | 6.22% | 6.26% | 6.20% |
| `atan2` | 2,298.84 | 774.81 | 66.30% | 65.82% | 67.71% |
| `atan` | 2,443.44 | 817.97 | 66.52% | 66.33% | 67.27% |
| `asin` | 3,346.56 | 1,064.72 | 68.18% | 68.18% | 69.91% |
| `acos` | 3,359.56 | 1,077.72 | 67.92% | 67.92% | 69.12% |
| `project_clip` | 554.25 | 368.47 | 33.52% | 32.52% | 37.32% |
| `mat34_inverse_rigid` | 1,345.34 | 1,095.34 | 18.58% | 19.97% | 19.12% |
| `mat34_look_at` | 7,926.66 | 4,593.88 | 42.05% | 42.88% | 44.84% |
| `mat44_perspective` | 1,309.91 | 1,305.53 | 0.33% | 0.33% | 0.33% |
| `sphere_intersects` | 170.53 | 107.25 | 37.11% | 34.30% | 36.86% |
| `sphere_intersects_aabb` | 261.62 | 129.38 | 50.55% | 48.50% | 50.49% |

Scalar multiply/divide/muldiv/reciprocal, scalar lerp/smoothstep, and vec3/vec4
dot retain their previous measured costs. Trial changes to generic division,
floor extraction and repeated dot-product bounds checks were removed after the
native model exposed regressions. The final capture also passes the camera
case that rejected an earlier candidate; failed exploratory captures are not
published as performance evidence.

## Why the arithmetic remains exact

For matrix/quaternion accumulation, checking that one side's raw magnitudes
sum to at most `UINT32_MAX` proves the absolute product sum is at most
`2^31 * (2^32 - 1)`, below `INT64_MAX`. Every partial sum fits, so the routine
can floor the final raw sum once and apply the same saturation. Inputs outside
that bound retain the original per-product quotient/remainder decomposition.
Matrix multiplication keeps output temporaries so either legal alias direction
still works.

Two signed32 products can overflow a sum only when both are exactly `2^62`;
vec2 dot detects the corresponding all-`INT32_MIN` case and saturates. Their
difference always fits signed64, which permits an unconditional direct cross
component. These are full-domain arithmetic proofs rather than assumed mesh
limits.

The root, normalization and trig changes preserve existing discrete outputs.
They do not substitute an approximate reciprocal/Newton iteration or a coarser
trig table. The tests exhaust all 65,536 angle inputs and the complete in-domain
asin/acos Q16 grid, with additional angle-boundary and full-range cases.

[Host CI](https://github.com/lamb2k/SH2ZAM/actions/runs/43518264323) passes all 14
CTest targets under GCC, Clang, and Clang ASan/UBSan, including independent
signed128 arithmetic oracles, frozen-source comparisons, matrix aliases,
failure-output preservation, C++ interop and the examples. The three new suites
perform 10,320,432 scalar/vector checks, 3,939,231 transform/collision/projection
checks and 1,683,296 trig checks. The target capture checks every output and
verifies stack, callee-saved registers and SR.S, including separate SR.S=1 runs.

## Generated instructions and remaining tradeoffs

The actual generated SH-2 code confirms that normalization no longer calls the
general signed64 division helper. Its exact fractional quotient and root loops
are inlined; the remaining variable-shift helpers operate on 32-bit values.
The cross3 routine is a leaf with six native DMULS.L products and direct output
stores, removing three nested cross2 calls.

For the recorded bounded 32-call batch, normalization's data accesses fall from
3,393 reads / 1,926 writes to 1,314 / 901, and cross3 falls from 1,072 / 840 to
816 / 616. Improvements are based on measured whole calls, not a prediction
from multiplication counts alone.

Some routines grow: sqrt uses 256 bytes of target code versus 148 previously,
and cross3 inlines work previously shared with cross2. Matrix/quaternion fast
branches retain their general fallback, increasing static size. Quaternion
multiply grows from 848 to 1,636 bytes and its own stack frame from 132 to 164
bytes; several product halves still spill. Matrix 4×4 reloads two row
coefficients from stack and uses a runtime copy for alias-safe output. Projection
and both sphere functions are leaves with no arithmetic helper calls; projection's
quotient loop has no stack accesses. These observations come from the archived
native disassembly, not a prediction of physical elapsed time.

The model cannot assess the instruction-cache tradeoff. It uses flat RAM with no cache, bus wait-state,
pipeline, multiplier-contention, DMA, interrupt, peripheral or second-CPU model.
Physical Saturn timing, alignment/batch-size tuning and dual-CPU scaling remain
unmeasured. No browser timings, console milliseconds, FPS, or whole-game speedup
are inferred from these counts.
