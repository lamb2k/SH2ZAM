# Exact math optimizations for SH-2

The current capture improves **40 of the 48 measured operations** across all
three recorded input profiles relative to SH2ZAM `10786b6`. The other eight
operations retain their previous cost: **120 operation/profile medians are
faster, 24 are unchanged, and none are slower**. All **1,008 individual batch
samples** are faster or unchanged, with **114,912 output words matching exactly**.
These figures describe the cumulative improvement from the frozen original
library used by the [48-operation math suite](benchmarks/math-model/README.md).

The latest changes also improve on the already optimized library at `3374d77`:
**39 profile medians are faster and 105 are unchanged**, covering **13 improved
APIs across all three profiles**, with no median regression. The improvement
comes from native fractional division and an exact inverse-tangent search
replacement. The [whole-workload suite](benchmarks/workload-model/README.md)
uses the separate `3374d77` baseline for the new rotation and spatial APIs and
selected scalar calls. Its setup-sharing and indexing gains must be assessed
against its recorded controls. The [independent plain-C and software-float
adoption comparison](benchmarks/sh2-model/README.md) remains a separate vertex
workload.

[Native CI evidence](https://github.com/lamb2k/SH2ZAM/actions/runs/43525421998) executes
the compiled big-endian SH-2 ELF from
[`598a6247`](https://github.com/lamb2k/SH2ZAM/commit/598a6247fe5a75320f2648e716a21db48f0eaa4c).
The [native artifact](https://github.com/lamb2k/SH2ZAM/actions/runs/43525421998/artifacts/14084553596)
contains the ELF, disassembly and captures. The
[complete dataset](benchmarks/math-model/results.json) retains every raw sample,
input/output checksum, toolchain/fixture identity and limitation.
ELF SHA-256: `4bed1cd10ee00f3e3b9db71ad454b136a5e24d1ef8823f47eb0b46ed43e9d3a8`.

## What changes for adopters

Existing calls use the optimized implementations automatically after rebuilding
or relinking the library. Public types, signatures, saturation, rounding,
zero/invalid-input behavior and matrix aliasing contracts remain unchanged.
There is no new setup object, allocation, approximate mode or caller-supplied
range promise for these existing operations. The separately documented batch
rotation and spatial-index APIs are optional additions.

| Area | Exact implementation change |
| --- | --- |
| Square root and vector length | Run integer-root digit loops with 32-bit state; use native 32×32-to-64 products for component squares. |
| Vector normalization | Preserve the existing rescaling and truncation; use `DIV0U`, sixteen unrolled `ROTCL`/`DIV1` pairs and a final `ROTCL` for each native fractional quotient. The portable C fallback stays exact. |
| Vec2 dot and 2D/3D cross | Accumulate the raw products directly and floor once; avoid temporary vectors and nested calls in cross3. |
| Matrix multiplication | Check each left row once, reuse the proof across output columns, and accumulate products directly on the proved path. |
| Scalar matrix transforms | Prove the input vector once, reuse it across explicit output rows, and avoid intermediate component/output arrays. |
| Quaternion multiplication | Prove one four-component magnitude sum for all four output components; preserve the original fallback. |
| Sine/cosine | Keep the same 257-entry sine table and interpolation outputs, use one interpolation multiply, and share work in sincos. |
| Inverse trig | Nominate an angle with a 514-byte first-octant table and the exact fractional quotient, then test the original neighboring decision boundaries. This replaces the prior fourteen binary-search comparisons without changing outputs or ties. |
| Projection | Compute both bounded screen-coordinate quotients in one restoring-division loop instead of two general 64-bit divisions. |
| Rigid inverse | Check six unique row pairs instead of repeating all nine symmetric orthonormality checks. |
| Sphere collision | Use exact unsigned32 coordinate distances and 32×32 squares; preserve overflow-safe radius subtraction. |

Camera construction, quaternion normalization/nlerp/rotation, and perspective
inherit improvements from the shared routines they call. The tables report
full API costs, including unchanged portions.

## Latest changes relative to optimized `3374d77`

The following bounded-input medians use the identical math-suite fixtures and
32-call wrappers from the [previous native capture](https://github.com/lamb2k/SH2ZAM/actions/runs/43518264229)
and the current capture. Input and output checksums agree. This isolates the
incremental improvement from the cumulative comparison in the next section.

| API (sh2z_ prefix omitted) | Optimized `3374d77` cycles/call | Current cycles/call | Further speedup |
| --- | ---: | ---: | ---: |
| `vec3_normalize` | 1,400.19 | 1,115.44 | 1.26× |
| `atan2` | 774.81 | 285.06 | 2.72× |
| `atan` | 817.97 | 302.75 | 2.70× |
| `asin` | 1,064.72 | 524.31 | 2.03× |
| `acos` | 1,077.72 | 537.31 | 2.01× |

The other affected APIs are `vec2_normalize`, `vec4_normalize`,
`quat_normalize`, `quat_nlerp`, `quat_to_mat34`, `quat_rotate`,
`quat_axis_angle`, and `mat34_look_at`. Their three input profiles also improve.
The other 35 APIs retain their prior profile medians. The measured improvements
above do not establish a universal 10× gain for individual math calls.

## Cumulative recorded costs relative to `10786b6`

Both versions in this suite are compiled with the same pinned Yaul GCC 13.2.0
toolchain for `-m2 -mb -O2`, with fast-math and LTO disabled. Each cell is the
median of seven deterministic batches of 32 API calls, including wrappers and
output writes. Fixtures cover ordinary bounded inputs, tiny raw values, and
signed32 full-range values/edge cases. See the
[exact fixture definitions and reproduction steps](benchmarks/math-model/README.md).

Positive percentages below mean fewer **modeled instruction cycles**. Zero
means unchanged. These results apply to the recorded fixtures; they do not
guarantee that every possible input or game will improve. The table is
generated from the current dataset's stored medians and derived percentages.

| API (sh2z_ prefix omitted) | Prior `10786b6` bounded cycles/call | Current bounded cycles/call | Bounded saved | Tiny saved | Full-range saved |
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
| `vec2_normalize` | 1,857.66 | 983.66 | 47.05% | 49.13% | 53.48% |
| `vec3_length` | 960.59 | 597.81 | 37.77% | 70.30% | 36.83% |
| `vec3_normalize` | 2,195.56 | 1,115.44 | 49.20% | 51.15% | 57.64% |
| `vec4_length` | 984.00 | 621.91 | 36.80% | 68.12% | 34.79% |
| `vec4_normalize` | 2,540.09 | 1,254.91 | 50.60% | 52.23% | 60.15% |
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
| `quat_normalize` | 2,564.69 | 1,279.53 | 50.11% | 51.74% | 59.67% |
| `quat_nlerp` | 8,309.50 | 4,304.75 | 48.19% | 49.35% | 55.22% |
| `quat_to_mat34` | 2,964.72 | 1,679.97 | 43.33% | 44.95% | 52.90% |
| `quat_rotate` | 3,339.25 | 1,891.66 | 43.35% | 44.80% | 51.32% |
| `quat_axis_angle` | 5,012.25 | 2,581.84 | 48.49% | 49.48% | 52.77% |
| `sin` | 68.62 | 64.19 | 6.47% | 6.56% | 6.59% |
| `cos` | 69.81 | 69.28 | 0.76% | 0.71% | 0.80% |
| `sincos` | 157.91 | 109.38 | 30.73% | 30.88% | 30.84% |
| `tan` | 384.47 | 360.56 | 6.22% | 6.26% | 6.20% |
| `atan2` | 2,298.84 | 285.06 | 87.60% | 87.73% | 88.74% |
| `atan` | 2,443.44 | 302.75 | 87.61% | 88.56% | 90.71% |
| `asin` | 3,346.56 | 524.31 | 84.33% | 84.33% | 79.59% |
| `acos` | 3,359.56 | 537.31 | 84.01% | 84.00% | 78.69% |
| `project_clip` | 554.25 | 368.47 | 33.52% | 32.52% | 37.32% |
| `mat34_inverse_rigid` | 1,345.34 | 1,095.34 | 18.58% | 19.97% | 19.12% |
| `mat34_look_at` | 7,926.66 | 4,177.06 | 47.30% | 47.91% | 48.71% |
| `mat44_perspective` | 1,309.91 | 1,305.53 | 0.33% | 0.33% | 0.33% |
| `sphere_intersects` | 170.53 | 107.25 | 37.11% | 34.30% | 36.86% |
| `sphere_intersects_aabb` | 261.62 | 129.38 | 50.55% | 48.50% | 50.49% |

Scalar multiply/divide/muldiv/reciprocal, scalar lerp/smoothstep, and vec3/vec4
dot retain their previous measured costs. Earlier trial changes to general
division, generic floor extraction and repeated dot-product bounds checks were
removed after the native model exposed regressions. The retained native
`DIV1` sequence specializes a proven fractional quotient; it does not replace
the general signed division contract. The current capture passes all camera
cases and publishes only successful exact-output runs.

## Why the arithmetic remains exact

### Accumulation and full-range fallbacks

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

### Native fractional quotient

The helper computes `floor(value * 65536 / denominator)` for
`0 <= value <= denominator` and `0 < denominator <= 2^31`, with zero and equality handled directly.
For a proper fraction, `DIV0U` initializes the division state, and sixteen
`ROTCL`/`DIV1` pairs consume the sixteen zero fractional numerator bits while
collecting the quotient. A final rotation collects the last quotient bit.
The quotient stays below 65536, so its rotations supply zero numerator bits.
This is the same integer quotient as the portable loop.

One assembly block owns the division flags throughout the sequence and declares
the T-bit clobber. Early-clobber register constraints keep the divisor, remainder
and quotient separate. SR.S is preserved. This uses CPU division instructions;
no memory-mapped divider or approximate reciprocal is introduced. Instruction
semantics are documented in the
[Renesas SH-1/SH-2/SH-DSP Software Manual](https://www.renesas.com/en/document/mas/sh-1sh-2sh-dsp-software-manual),
sections 6.1.18, 6.1.19 and 6.1.47.

### First-octant estimate with exact correction

The added table has 257 unsigned16 entries, totaling **514 bytes**, generated as
`legacy_sh2z_atan2(i, 256)`. It estimates an angle from the sixteen-bit ratio.
The original sine table still determines the final output.

For every one of the 65,536 fractional-ratio bins, tests check the exact answer
at both inclusive endpoints in both octants. The original first-quadrant answer
is monotonic in the ratio, so these endpoints bound every intervening rational
input. The estimated angle differs from the exact answer by at most one unit.
The transition from `k` to `k+1` occurs exactly when
`ay * (cos(k) + cos(k+1)) > ax * (sin(k) + sin(k+1))`.
Testing the two neighboring original boundaries therefore recovers the former
answer. Equality retains the lower angle, and the original quadrant mapping is
preserved. Unsigned magnitudes also cover `INT32_MIN` inputs.

The root, normalization and trig changes preserve existing discrete outputs.
Tests exhaust all 65,536 sine/cosine angle inputs and the complete in-domain
asin/acos Q16 grid, plus every original angle-transition tie, nearby full-range
directions, quotient boundaries and randomized signed inputs.

## Verification

[Host CI](https://github.com/lamb2k/SH2ZAM/actions/runs/43525422024) passes all
**17 CTest targets** under GCC, Clang, and Clang ASan/UBSan, including independent
signed128 arithmetic oracles, frozen-source comparisons, matrix aliases,
failure-output preservation, C++ interop and the examples. Focused suites report:

| Suite | Exact checks |
| --- | ---: |
| Scalar/vector | 10,619,477 |
| Transform/collision/projection | 3,939,231 |
| Trigonometry | 2,469,728 |
| Quaternion rotation batches | 1,015,148 |
| Spatial index | 12,445,504 |

The native math capture compares every output word and verifies stack,
callee-saved registers and SR.S, including separate SR.S=1 runs. The workload
suite independently retains its own fixtures, output comparisons and API
contracts; see its [measurement details](benchmarks/workload-model/README.md).

## Generated instructions and remaining tradeoffs

The current archived SH-2 disassembly contains the unrolled sixteen-`DIV1`
sequence inside normalization and both ratio branches of `atan2`. Only the
selected ratio branch executes for a call. Normalization's root work remains
inlined, and its variable-shift helpers operate on 32-bit values. The cross3
routine remains a leaf with six native DMULS.L products and direct output
stores. The trig object still references 32-bit arithmetic-shift helpers;
the recorded full-call costs include their work.

The following are medians of **data reads / data writes per bounded 32-call
batch**, recomputed from the raw sample counters. They are instruction-model
access counts, not cache misses or bus transactions.

| API | Original `10786b6` | Optimized `3374d77` | Current capture |
| --- | ---: | ---: | ---: |
| `vec3_normalize` | 3,393 / 1,926 | 1,314 / 901 | 1,314 / 901 |
| `vec3_cross` | 1,072 / 840 | 816 / 616 | 816 / 616 |
| `atan2` | 8,308 / 1,840 | 1,317 / 390 | 1,017 / 416 |
| `atan` | 8,866 / 1,960 | 1,378 / 392 | 1,086 / 424 |

The native quotient improvement reduces normalization's executed arithmetic
while retaining its memory-access counts. The new `atan2` path reduces reads
relative to `3374d77` but increases writes from 390 to 416 in these batches;
its measured cycle reduction includes that tradeoff. Multiplication counts
alone would not reveal the result.

The additional 514-byte angle table and unrolled division instructions increase
static storage. Matrix/quaternion fast branches also retain their general
fallbacks. The model cannot assess the instruction-cache tradeoff: it uses flat
RAM with no cache, bus wait-state, pipeline, multiplier-contention, DMA,
interrupt, peripheral or second-CPU model. Physical Saturn timing,
alignment/batch-size tuning and dual-CPU scaling remain unmeasured. No browser
timings, console milliseconds, FPS, or whole-game speedup are inferred from
these counts.
