# Validation record — 2026-11-17 (Q2.30 unit vectors, fixed-unit-1)

`sh2zam/unit30_experimental.h` ([guide](../UNIT30.md)) adds:
- inline Q2.30 multiply, `MAC.L` dot and cross products, all rounded once and saturating;
- a seeded multiply-only reciprocal square root;
- a normalisation of any `int32_t` scale, in hand-written SH-2 assembly;
- an exactly rounded 64-bit square root;
- the two cross-product orthonormalisations of a 3x3 basis.

Host and SH-2 give the same bits. **No console or emulator run was done.** Evidence by tier:

- **HOST:** `unit30` passes under GCC and Clang Release with `SH2Z_STRICT=ON` and under
  ASan/UBSan. It is checked against an independent oracle (128-bit integers, float64):
  - the K1 operations are exact on 3.2M pairs, and 977,938 saturations are all counted;
  - the reciprocal-square-root mantissa is in [-1.2004, +0.2243] LSB over all 3.2e9 inputs
    (exhaustive run);
  - `rsqrt64` is within 1.36 LSB and normalisation within 1.03 LSB (bound 2);
  - `isqrt` is exact on 1.45M values;
  - 808,004 bases are within 3 + 1.5/|cross| LSB of a float64 run of the same algorithm,
    against up to 316 LSB for binary32 when the rows are well apart;
  - 9 negative controls are all caught (21,452,989 checks).
- **MODELLED:** in the Yabause interpreter, before timing, every output is checked against
  the oracle or equals the host build bit for bit:
  - the assembly normalisation equals its portable C on SH-2;
  - the `MAC.L` dot product is also checked with SR.S set on entry.

  ([results](../benchmarks/unit30-model/results.json), captured by [CI run 44224191560](https://github.com/lamb2k/SH2ZAM/actions/runs/44224191560).)

  Modelled cycles per element: multiply 23, dot 37, cross 89, normalisation 155-182. A
  basis orthonormalisation costs 494-519, against 10,948 for the same algorithm on
  binary32 soft-float and 2,507 for SH2ZAM Q16.16. Cycle numbers are modelled, not Saturn
  timings. The plan's cycle targets for dot, cross, normalisation, `isqrt` and the
  orthonormalisation (250-300) are not met; the guide lists why.
- **Not done:** console or emulator execution, cache and memory timing, a whole camera or
  body update.
