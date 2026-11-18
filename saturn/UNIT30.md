# Q2.30 unit-vector kernels (experimental, contract tag `fixed-unit-1`)

[`unit30_experimental.h`](include/sh2zam/unit30_experimental.h) adds fixed-point
arithmetic for directions and bases, so that code which today keeps a camera or a body
frame in soft-float can keep it in integers instead:

- inline Q2.30 multiply, dot product and cross product, and a Q20.12 x Q2.30 scale;
- a reciprocal square root built from a 256-entry seed and two multiply-only Newton steps;
- a vector normalisation that accepts any `int32_t` scale;
- a rounded 64-bit integer square root;
- the two cross-product orthonormalisations of a 3x3 basis, in a fixed operation order.

Nothing selects them by default. There is no floating point, divider, libc or
allocation; the only global state is an optional overflow counter.

## Status

- Contract tag `fixed-unit-1`. The numeric definitions in the header are frozen at the tag.
  A faster path may replace an implementation only if it produces identical bits.
- **Host and SH-2 agree bit for bit.** Every function is a fixed integer function of its
  inputs. On SH-2, GCC compiles each 32 x 32 -> 64 product of the C to one `DMULS.L` or
  `DMULU.L`. Two pieces are hand-written:
  - `sh2z_u30_dot3`, a `MAC.L` sequence;
  - `sh2z_vec3_normalize_u30`, an assembly function.

  The instruction model runs both and their portable C on SH-2, and compares them bit for
  bit with each other and with the host build of the same source.
- Evidence tiers: **HOST** (an independent oracle, exhaustive and randomised sets, GCC
  and Clang strict, ASan+UBSan) and **MODELLED** (the compiled SH-2 code in the pinned
  Yabause interpreter, flat RAM). Nothing has run on an emulator system or a console.
- The cycle targets of the plan were not all met; see [Targets](#targets).

## API and error bounds

Q2.30 is `int32_t` with 1.0 = 2^30 and range [-2, 2). Q20.12 is `int32_t` with 1.0 = 4096.
"LSB" means one unit of the result word.

| Function | Definition (exact text in the header) | Error |
| --- | --- | --- |
| `sh2z_u30_mul(a, b)` | `floor((a*b + 2^29) / 2^30)`, saturated | <= 0.5 LSB, exact rule |
| `sh2z_q12_mul_u30(q12, u)` | The same arithmetic, Q20.12 x Q2.30 -> Q20.12 | <= 0.5 LSB, exact rule |
| `sh2z_u30_dot3(a, b)` | `floor((a0*b0 + a1*b1 + a2*b2 + 2^29) / 2^30)`; the sum is exact (65 bits), rounded once, saturated | <= 0.5 LSB, exact rule |
| `sh2z_u30_cross(out, a, b)` | Each component `floor((a1*b2 - a2*b1 + 2^29) / 2^30)` (and cyclically), saturated; `out` may alias `a` or `b` | <= 0.5 LSB per component, exact rule |
| `sh2z_rsqrt64_u30(s, scale)` | `2^(30 + scale/2) / sqrt(s)` in Q2.30, for `scale` 0..127. Results of 2 or more, and `s == 0`, give `INT32_MAX`; an out-of-range `scale` gives 0 | <= 2 LSB (measured <= 1.36) |
| `sh2z_vec3_normalize_u30(out, v)` | `2^30 v / |v|` for `v` of any `int32_t` scale; false (out untouched) for `v == 0`; odd-symmetric; `out` may be `v` | <= 2 LSB per component (measured <= 1.03) |
| `sh2z_isqrt64_q12(s)` | `floor(sqrt(s) + 1/2)`; `UINT32_MAX` when that is 2^32 | exact |
| `sh2z_basis30_orthonormalize_forward(b)` | See [Orthonormalisation](#orthonormalisation) | see below |
| `sh2z_basis30_orthonormalize_up(b)` | See [Orthonormalisation](#orthonormalisation) | see below |

**When saturation happens.**
- The multiplies saturate only when `|a*b| >= 2^61 - 2^29`, which needs both factors at
  least 1 in magnitude.
- `dot3` saturates only when `|a| |b| >= 2`.
- `cross` saturates only when a component of `a x b` reaches 2.
- With a unit vector on either side, none of them saturates.

**The overflow counter.** Define `SH2Z_U30_COUNT_OVERFLOWS` to non-zero before including
the header, and every saturation in the inline operations adds one to
`sh2z_u30_overflows`. Build the library with the same definition to count the
out-of-line functions as well. Without the definition nothing is written, and results
never depend on the counter. It is not synchronised between the two CPUs.

**`dot3` and SR.S.** `MAC.L` accumulates 64 bits only while SR.S is clear. `dot3`
therefore saves SR, clears S, preloads MACH:MACL with the rounding term 2^29, runs the
three `MAC.L` and restores SR. The model checks it with S set on entry as well.

**`dot3` saturation.** The modular 64-bit sum is exact whenever the result does not
saturate (the exact sum is at most 3 * 2^62). The rare saturating case calls
`sh2zi_u30_dot3_saturate`, which takes the exact sign from a split sum. For example,
(INT32_MIN, INT32_MIN, INT32_MIN) dotted with itself gives `INT32_MAX`, even though the
64-bit sum has wrapped negative.

### Arithmetic of K2

**Mantissa kernel** (`sh2zi_u30_rsqrt_q31`). For m in [2^30, 2^32) (x = m / 2^32 in
[1/4, 1)) it returns Y, about 2^31 / sqrt(x).

- **Seed.** `seed[i] << 16`, where
  - i = m >> 24 on [2^31, 2^32);
  - i = (m >> 23) - 128 on [2^30, 2^31).

  Every interval is therefore 2^-7 of its x. The entry is round(2^15 h), with h the
  harmonic mean of 1/sqrt at the interval ends, so the seed's relative error is at most
  2^-9 + 2^-16.
- **Newton step 1.** e = -(high(high(m y) y) << 2) is (1 - x y^2) 2^32. Then
  y += high_signed((y >> 1) e). The relative error after this step is at most 2^-17.4.
- **Newton step 2.**
  1. The residual (1 - x y^2) 2^47 is formed modulo 2^32 from bits 48..79 of the exact
     96-bit m y^2. That takes three `DMULU.L` and one `XTRCT`.
  2. y += high_signed(((y >> 16) + 1) e).

Over **every** m in [2^30, 2^32) (3,221,225,472 values, host), Y - 2^47/sqrt(m) lies in
[-1.2004, +0.2243] Q1.31 LSB. The model's sample gives [-1.1576, +0.2066].

**`sh2z_vec3_normalize_u30`**
1. Take the magnitudes a_k = |v_k|.
2. Shift all three left by the one amount that puts the largest in [2^30, 2^31]. This is
   exact; nothing is lost.
3. Form the exact sum of squares s, which lies in [2^60, 3 * 2^62].
4. If s >= 2^62, take m = s >> 32. Otherwise take m = s >> 30 and double every a_k (each
   is below 2^31 then). Either way 1/sqrt(s) = Y / 2^63.
5. Each output word is ±((high(a_k Y) + 1) >> 1) = ±round(a_k Y / 2^33), with the sign
   of v_k.

The error budget is half an LSB from the final rounding plus 2^30 times the mantissa's
relative error. Measured, it stays at or below 1.03 LSB.

**`sh2z_isqrt64_q12`**
1. Shift s left by an even amount so it lies in [2^62, 2^64).
2. Estimate q = (S y) >> 95 from the mantissa kernel of the top word. This is within -2
   to +1 of floor(sqrt(S)).
3. Correct q exactly with q^2 and (q+1)^2.
4. Round while shifting back: `(q >> (j-1)) + 1) >> 1` (the variable shift is one
   `DMULU.L` by a power of two). When j = 0, round with the remainder test S - q^2 > q.

## Orthonormalisation

Both functions follow the binary32 routines they replace, in the same operation order,
with the same partial updates and the same -1 cases:

```text
forward:  if x == 0: return -1                 (nothing written)
          x = normalize(x)
          z = y cross x                         (sh2z_u30_cross)
          if z == 0: return -1                  (x and z written, y unchanged)
          z = normalize(z)
          y = x cross z
          return 0
up:       the same with x and y exchanged in the roles: y normalised first,
          z = y cross x, x = z cross y
```

**Input scales.**
- The row that is normalised first (x for forward, y for up) may have any `int32_t` scale,
  because only its direction is used. A Q20.12 difference such as target - eye can be
  stored directly.
- The other input row is Q2.30. While it is shorter than 2, nothing saturates.
- The last cross product takes two normalised rows. It cannot saturate, so it skips the
  test and still gives the same words.

**Zero cases.** The -1 case after the cross product applies when the Q2.30-rounded cross
product is exactly zero. That is, the two rows are parallel to within about 2^-31. The
binary32 routine tests its sum of squares against 0.0 instead.

**Accuracy against a float64 oracle of the same algorithm** (host, 808,004 bases). Let c be
the length of the cross product of the normalised first row and the other row: the sine
of the angle between them, times the other row's length. The bound per output word is

    |ours - float64| <= 3 + 1.5 / c  LSB

and it holds on every base tested. The same bound holds for the dot product of any two
output rows (orthogonality) when c >= 2^-16. Every output row has |len^2 - 1| <= 8 LSB in
those cases.

| c from | Bases | Ours - float64, max (LSB) | max x c | binary32 - float64, max (LSB) | Max row dot (LSB) | Max \|len^2 - 1\| (LSB) |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 0.5 | 528,802 | 2.86 | 2.45 | 315.8 | 2.2 | 4.7 |
| 0.1 | 133,771 | 10.09 | 1.40 | 778.8 | 6.8 | 4.4 |
| 0.01 | 25,871 | 88.2 | 1.06 | 4,412.7 | 73.7 | 4.2 |
| 2^-16 | 62,616 | 55,364 | 0.91 | 2,630,417 | 43,975 | 5.1 |
| 0 | 46,524 | 6.5e7 | 0.91 | 2.0e9 | (degenerate) | (degenerate) |

**How it compares with binary32.** Where the rows are well apart (c >= 0.5), the result
is within 2.9 LSB of float64, while the binary32 reference is up to 316 LSB away. Both
degrade as 1/c, because the cross product is a difference of nearly equal products.
Ours degrades about 100 times less: the products are exact and are rounded once at
2^-31, where binary32 rounds at 2^-24.

## Evidence (HOST)

[`tests/test_unit30.c`](tests/test_unit30.c) with the independent oracle
[`tests/unit30_oracle.h`](tests/unit30_oracle.h). The oracle uses 128-bit integers for the
exact rules and for `isqrt`, and float64 for the approximate kernels and for both
orthonormalisations. It shares no code with the library. GCC strict, Clang strict and GCC
ASan+UBSan all pass: 21,452,989 checks per run, about 2 s.

| Set | Size | Result |
| --- | --- | --- |
| `mul`, `q12_mul`: every pair of 24 edge words, plus random words of mixed scales | 2,000,576 pairs | exact; within half an LSB when not saturated |
| `dot3` (and its portable form), `cross` (and both aliasings): edge vectors, random, unit vectors, the 65-bit extremes | 1,204,613 pairs | exact |
| Saturations counted by `sh2z_u30_overflows` | 977,938 | equal to the oracle's count |
| Seed table rebuilt from its rule | 256 entries | equal |
| Mantissa kernel: strided, plus 512 values at both ends of every seed interval | 1,048,001 m | [-1.1776, 0.2220] LSB |
| Mantissa kernel, exhaustive (`SH2Z_UNIT30_EXHAUSTIVE=1`, run once locally) | 3,221,225,472 m | [-1.2004, 0.2243] LSB |
| `rsqrt64`: every power of two ± 1 for scale -2..129, plus random | 430,320 | max 1.3534 LSB; saturation and range rules exact |
| `normalize`: every triple of edge words, random scales, unit vectors, single axes at every scale | 1,791,127 vectors | max 1.0267 LSB; zero refused with out untouched; odd-symmetric; in place |
| `isqrt`: 0..69,999, squares and half-way points around roots up to 2^33, random, the top of the range | 1,451,994 | exact |
| Orthonormalisations (forward and up): any-scale and Q20.12 primary rows; general, nearly parallel and exactly parallel secondary rows; zero rows | 808,004 bases, 10,420 returning -1 | within 3 + 1.5/c LSB of float64; -1 and partial updates exact |

**Mutation controls.** The same checks must reject each mutant, and all are rejected:

| Mutant | Times rejected |
| --- | ---: |
| `mul` truncating | 92,384 |
| `mul` wrapping instead of saturating | 6,609 |
| `dot` rounding each product | 63,903 |
| `dot` saturating by the sign of the wrapped 64-bit sum | 1 (the all-minimum case) |
| `normalize` in binary32 | 167,783 |
| `normalize` 1.5 LSB short | 158,565 |
| `isqrt` flooring | 95,090 |
| forward computing z = x cross y | 19,948 |
| up without the partial update on its second -1 | 10,000 |

## Evidence (MODELLED)

[`benchmarks/unit30-model`](benchmarks/unit30-model/README.md) runs the library compiled
for `-m2 -mb -O2` (GCC 13.2 from the pinned Yaul image) in the pinned Yabause SH-2
interpreter. That is flat RAM with no caches or wait states, not Saturn timing. The
committed capture [`results.json`](benchmarks/unit30-model/results.json) is the CI
capture named in the commit that records it.

**Verification before any timing.**
- **Against the oracle:** `mul`, `q12_mul`, `dot3` (`MAC.L`, again with SR.S set on
  entry), the portable `dot3`, `cross` and `isqrt`.
- **Bit for bit against the host build of the same source**, and within the oracle's
  bounds: the mantissa kernel, `rsqrt64`, the assembly `normalize` (out of place and in
  place), its portable C on SH-2, and both orthonormalisations.
- **Counts per run:** 51,200 elements per kernel; 102,400 `MAC.L` dots with SR.S set;
  25,600 bases, 2,828 of which return -1.

**Columns.**
- **Cycles per element:** the slope between the one-element call and the 128-element call
  (32 for bases). It includes the bridge loop's load, store and branch, a few cycles per
  element.
- **binary32 soft-float:** the same algorithm in C on libgcc's `__mulsf3`/`__addsf3`/
  `__divsf3`, with a correctly rounded integer `sqrtf`.
- **SH2ZAM Q16.16:** the straight translation onto today's out-of-line calls
  (`sh2z_mul`, `sh2z_vec3_dot`, `sh2z_vec3_cross`, `sh2z_rsqrt`, `sh2z_vec3_normalize`).

| Operation | fixed-unit-1 | binary32 soft-float | SH2ZAM Q16.16 | Speed-up over Q16.16 |
| --- | ---: | ---: | ---: | ---: |
| multiply (inline, per element) | 23 | 255 | 33.5 | 1.5x |
| dot of two unit vectors (`MAC.L`; portable C: 43) | 37 | 1,322 | 157 | 4.2x |
| cross of two unit vectors | 89 | 2,381 | 179 | 2.0x |
| rsqrt mantissa kernel alone | 60.5 | | | |
| `rsqrt64` of a Q20.12 squared length | 142 | 896 | 639 | 4.5x |
| normalize a Q20.12 offset (assembly; portable C: 207) | 182 | 3,113 | 1,084 | 6.0x |
| renormalise a Q2.30 vector (assembly; portable C: 180) | 155 | 3,112 | 1,094 | 7.0x |
| `isqrt` of a Q20.12 squared length | 213 | 407 (`sqrtf`) | | |
| orthonormalize forward (target - eye in Q20.12, up near +Y) | 519 | 10,948 | 2,507 | 4.8x |
| orthonormalize up (up + 0.3 tilt, a unit forward) | 494 | 10,969 | | |

On the same model, a caller measured the binary32 forward routine at 3,843 cycles with
its own soft-float library, and the straight SH2ZAM translation at 3,076. The kernel
here is 7.4 times faster than the former.

### Targets

| Kernel | Target (cycles) | Modelled | Why |
| --- | ---: | ---: | --- |
| `mul` / `q12_mul` | 8-11 / 9 | about 14 in a loop (23 with two loads, a store and the loop) | The rounding add and the saturation test cost about 7 beyond `DMULS.L` and its two `STS`. |
| `dot3` | 14 | about 28 (37 in a loop) | The three `MAC.L` cost 9. Saving, clearing and restoring SR costs 5. The saturation test and extraction cost 9. |
| `cross` | 30 | about 75 (89 in a loop) | Each component needs two `DMULS.L`, four `STS`, a 64-bit subtract and the round-and-saturate (about 21). |
| rsqrt kernel | about 50 | about 48 (60.5 per call in a loop) | met |
| normalize | 90-110 | about 145-170 | Mantissa kernel 48, sums of squares 17, outputs 15, magnitudes and signs 27, prescale 5-30. |
| `isqrt` | 80-110 | about 200 | The 64-bit alignment search, two estimate products and the exact correction. |
| orthonormalize | 250-300 | about 495-520 | Two normalisations (about 145 each) and two cross products (about 60-70 each). |

Getting K3 to 300 needs a normalisation near 90, which this arithmetic does not reach. It
would need cheaper sign handling (a signed final product, giving up odd symmetry) and a
hand-written cross product. Both would be new functions, not changes under this tag.

## Caller guidance

- Keep directions and basis rows in Q2.30 and positions in Q20.12.
- `sh2z_q12_mul_u30(length_or_coordinate, direction_component)` scales a position by a
  direction.
- `sh2z_u30_dot3` projects onto a basis row.
- For a camera basis, store target - eye (Q20.12) directly in `basis.x` and the up
  direction (Q2.30) in `basis.y`, then call `sh2z_basis30_orthonormalize_forward`. The
  result rows are Q2.30 unit vectors.
- `sh2z_vec3_normalize_u30` accepts the Q20.12 offset directly. Do not convert it to
  Q2.30 first.
- `sh2z_isqrt64_q12` gives a Q20.12 length from the 64-bit sum of squares of Q20.12
  components. Form that sum exactly with `(int64_t)` products.

## Not established

- Any measurement on a console or an emulator system. The model has no caches, wait
  states, bus contention or real multiplier latency. Real `DMULS.L`/`MAC.L` timing
  depends on the following instruction.
- The effect on a whole tick or frame. That needs the caller's camera or body update
  measured with and without the kernels.
- Accuracy of a whole caller pipeline (for example accumulated drift over a race). The
  bounds here are per call.
