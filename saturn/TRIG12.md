# 12-bit-angle trig and divider quotients (experimental, `trig12-1`)

Two small kernel families for code that keeps its angles as 12-bit indices (4096 per turn)
and wants fixed-point results:

- [`sh2zam/trig12_experimental.h`](include/sh2zam/trig12_experimental.h): sine and cosine in
  Q2.30 from a 1,025-entry quarter table, and a Q16.16 tangent.
- The `trig12-1` section of [`sh2zam/divu_experimental.h`](include/sh2zam/divu_experimental.h):
  inline `num * 2^f / den` quotients and reciprocals through the SH-2 division unit, with
  start/finish forms so the division can overlap other work.

Nothing here is selected by a default dispatch. The existing 16-bit-angle trig
(`sh2zam_trig.c`, 257 entries, Q16.16) and the existing divider calls are unchanged: their SH-2
objects are byte-identical to `main`.

## Sine and cosine

```c
#include <sh2zam/trig12_experimental.h>

int32_t s, c;
sh2z_sincos12_u30(angle12, &s, &c);   /* Q2.30: 1.0 is 1 << 30 */
int32_t y = sh2z_sin12_u30(angle12);
int32_t x = sh2z_cos12_u30(angle12);  /* == sh2z_sin12_u30(angle12 + 1024) */
```

Only the low 12 bits of `angle12` are used, so a signed angle converted to `uint32_t` wraps
the way `table[angle & 0xFFF]` does. All three are `static inline`; the table
`sh2z_sin12_quarter_u30[1025]` (4,100 bytes) is in `sh2zam_trig12.c`.

### The table

Entry `i` (0..1024) is the nearest Q2.30 integer to `S(i)`, the binary64 value of the
truncated Taylor series that is commonly used to build a 4096-entry sine table at start-up:

```c
double x = (i * (2 * 3.1415926535)) / 4096;     /* note the 10-digit pi */
double term = -x * x * x, minus_square = -x * x, denominator = 1 * 2 * 3;
for (int j = 2; j < 14; j++) {                 /* x - x^3/3! + ... - x^27/27! */
    x += term / denominator;
    term *= minus_square;
    denominator *= j * ((j << 2) + 2);
}
```

[`tools/sin12_table.py`](tools/sin12_table.py) generates it and, with `--check`, verifies the
committed table. CI runs the check. The script evaluates the same statements in binary64 (Python
floats are IEEE binary64, with no fused operations) and once more in exact rational
arithmetic. The host test re-evaluates the series in C.

| Property | Value |
| --- | --- |
| \|entry − S(i)\|, max over 1,025 entries | 0.499884 LSB (2^-31 bound) |
| \|entry − exact series\|, max | 0.499884 LSB; binary64 rounding is at most 4·10^-7 LSB |
| Closest S(i)·2^30 to a rounding tie | 1.2·10^-4 LSB, so the binary64 rounding never changes an entry |
| Endpoints, monotonic | entry 0 = 0, entry 1024 = 2^30; nondecreasing |

### Quadrants

With `o = angle12 & 1023` and `T` the table:

| Quadrant | sin | cos |
| --- | --- | --- |
| 0 | `T[o]` | `T[1024 − o]` |
| 1 | `T[1024 − o]` | `−T[o]` |
| 2 | `−T[o]` | `−T[1024 − o]` |
| 3 | `−T[1024 − o]` | `T[o]` |

This is exact symmetry: `cos(a) == sin(a + 1024)`, `sin(a + 2048) == −sin(a)` and
`sin(−a) == −sin(a)` hold for every angle. The quarter turns are exact (0 and ±2^30), and no
interpolation is done.

### Deviation from a full 4096-entry series table

A table built from the same series at all 4096 angles is not exactly symmetric. Without
range reduction, the truncated series loses accuracy as `x` approaches 2π, and the 10-digit π
shifts the samples slightly. Measured over every angle (HOST, `trig12_exact`), in LSB of Q2.30
(2^-30 ≈ 9.3·10^-10):

| Reference | Quadrant 0 | 1 | 2 | 3 |
| --- | ---: | ---: | ---: | ---: |
| binary64 series table | 0.4999 | 0.5887 | 0.5936 | 332.16 |
| the same table rounded to binary32 | 32.0 | 32.0 | 32.0 | 332.13 |
| true `sin(2πa/4096)` | 0.5137 | 0.5137 | 0.5137 | 0.5137 |

- **Cosine:** the deviation is the same, because cosine is the same values shifted by a quarter turn.
- **Binary32 rows:** the 32 LSB is the binary32 rounding of values near 1.0 (half an ulp is
  2^-25).
- **Quadrant 3:** the 332 LSB (3.1·10^-7) is the series' own error at the last angles before a
  full turn. 787 of the 4096 angles, all in quadrant 3, are more than 1 LSB from the binary64
  table.
- **Accuracy against the true sine:** these results are closer to the true sine than the
  4096-entry table is.

## Tangent

```c
int32_t t = sh2z_tan12_q16(angle12);   /* Q16.16 */
```

- **Formula:** in quadrants 0 and 2 the result is `T[o] / T[1024 − o]`; in quadrants 1 and 3 it
  is `−T[1024 − o] / T[o]`.
- **Rounding:** to nearest, ties away from zero.
- **Domain:** every angle. Near ±90° the largest finite value is ±651.898
  (`42722788`), one step from a pole.
- **Poles:** the poles themselves (1024 for +90° and 3072 for −90°) saturate by the sign of the
  sine, to `INT32_MAX` and `INT32_MIN`.
- **Division:** uses the CPU's division steps (`DIV1`), not the memory-mapped divider. This
  keeps the divider promise in `divu_experimental.h`, under which only `_divu` functions and
  that header use the divider. At about 100 cycles, it is meant for once-per-frame work such as
  a projection's field of view.

Accuracy over every angle except the poles (HOST), in LSB of Q16.16 (2^-16), for
\|angle\| from zero up to the given limit:

| Reference | 45° | 60° | 80° | 85° | 89° | 89.12° | 89.91° |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| exact quotient of the two table values | 0.499 | 0.499 | 0.499 | 0.499 | 0.499 | 0.499 | 0.499 |
| true `tan(2πa/4096)` | 0.499 | 0.499 | 0.499 | 0.499 | 0.499 | 0.499 | 8.21 |
| binary64 series sin / cos | 0.511 | 0.511 | 0.744 | 2.09 | 55.3 | 81.0 | 8617 |
| binary32 sin / cos (binary32 division) | 0.511 | 0.511 | 0.750 | 2.13 | 55.3 | 81.0 | 8616 |

- **True tangent:** the result is within half an LSB up to 89.12° (1014 units). Beyond that the
  table's half-LSB rounding of a small sine is amplified, to at most 8.2 LSB (relative 3·10^-7)
  at 89.91°.
- **Series-table rows:** the larger deviations come from that table's own error near a full
  turn, which the series quotient divides by.

## Divider quotients

```c
#include <sh2zam/divu_experimental.h>

int32_t r  = sh2z_divu_q16(num, den);     /* num * 2^16 / den: ratio in Q16.16 */
int32_t u  = sh2z_divu_q30(num, den);     /* num * 2^30 / den: ratio in Q2.30  */
int32_t q  = sh2z_divu_q(num, den, 12);   /* any constant 0 <= f <= 31 */
int32_t i1 = sh2z_recip_q16(x16);         /* 2^32 / x16: 1/x in Q16.16 */
int32_t i2 = sh2z_recip_q30(x30);         /* 2^60 / x30: 1/x in Q2.30, fits for |x| > 0.5 */

sh2z_divu_t t;                            /* overlap: start, work, finish */
sh2z_divu_q_start(&t, num, den, 16);
/* ... up to ~39 cycles of independent work ... */
bool overflow;
int32_t r2 = sh2z_divu_finish(&t, &overflow);
```

The result rule is the one `divu_experimental.h` already defines, applied to the dividend
`num · 2^f`:

| Case | Result | `overflow` |
| --- | --- | --- |
| `den != 0`, `abs(num)·2^f < abs(den)·2^31` | `trunc(num·2^f / den)`, toward zero on both signs | false |
| `den != 0`, otherwise | `INT32_MIN` if the signs differ, else `INT32_MAX` | true |
| `den == 0` (0/0 included) | `num >= 0 ? INT32_MAX : INT32_MIN` | true |

- **Exact −2^31:** an exact quotient of −2^31 is returned exactly and flagged, as for
  `sh2z_divu_div64`.
- **Zero numerator:** `num = 0` with `den != 0` returns 0.
- **Agreement with existing calls:** `sh2z_divu_q16` equals `sh2z_div` everywhere except
  `0/0`, which `sh2z_div` answers with 0. `sh2z_recip_q16` equals `sh2z_recip` everywhere.
- **Fit test:** `abs(num) >> (31 − f) < abs(den)` on unsigned 32-bit magnitudes. It is exact,
  and a zero divisor never passes it. Only quotients that pass it reach the hardware; the rest
  are answered in software before the divider is touched.
- **Constant `f`:** pass a constant. The SH-2 has no variable shift, so a variable `f` compiles
  to libgcc shift calls.
- **Caller promise:** the promise at the top of the header applies between start and finish.
  No other code may use that CPU's divider unless its interrupt handlers save and restore it.

## Cost (MODELLED)

These are cycles per element in a loop in the Yabause SH-2 interpreter: flat RAM, no caches or
wait states, and a divider latency of 39 cycles (assumed). The figures are not Saturn timings.
"Net" subtracts the bare loop, which loads the inputs and stores one word. Results:
[`benchmarks/trig12-model/results.json`](benchmarks/trig12-model/results.json), captured by
[CI run 44208800045](https://github.com/lamb2k/SH2ZAM/actions/runs/44208800045) on `ad5eae2`.

| Kernel | Net cycles | Control | Net cycles |
| --- | ---: | --- | ---: |
| `sh2z_sincos12_u30` (both values, two stores) | 19.0 | two binary32 table loads | 11.0 |
| `sh2z_sin12_u30` | 11.0 | two binary32 loads converted to Q2.30 (libgcc soft-float) | 704 |
| | | `sh2z_sincos` (16-bit angle, interpolated, a call) | 100 |
| `sh2z_tan12_q16` (a call) | 103.5 | binary32 sin / cos (`__divsf3`) | 496 |
| | | `sh2z_tan` (16-bit angle) | 212.5 |
| `sh2z_divu_q16`, blocking | 56.0 | `sh2z_div` | 98.5 |
| `sh2z_divu_q30`, blocking | 57.5 | `((int64_t)num << 16) / den`, clamped (`__divdi3`) | 253 |
| `sh2z_recip_q30`, blocking | 45.5 | binary32 `num / den` (`__divsf3`) | 485 |
| q16, then sincos12 (blocking) | 84.5 | | |
| q16 started, sincos12, then finished | 59.5 | | |

- **Divider wait:** the blocking quotients spend 38 of their cycles waiting for the divider.
- **Overlap:** overlapping a sincos with the division saves 25 cycles per pair. The quotient
  then costs about 41 cycles beyond the sincos.

Against the K-plan targets:
- **Single values:** the sine or cosine alone (11) is within the 8–12-cycle target.
- **sin/cos pair:** the pair costs 19. Its two conditional quadrant fix-ups (swap and negate)
  dominate; two alternative C formulations (a sign-then-store form and a
  four-way switch) were slower, at 23.5 and 25.
- **Blocking quotients:** these cost 45.5–57.5 against a target of about 45, set by the 39-cycle
  latency plus a software fit test of about 10 instructions.

## Evidence

- **HOST:** `trig12_exact` and `divq_exact` pass under GCC and Clang Release with
  `SH2Z_STRICT=ON` and under ASan/UBSan. Both are checked against the independent oracle
  [`tests/trig12_oracle.h`](tests/trig12_oracle.h): the series rewritten in C, and 128-bit
  quotients.
  - **Table:** every entry equals the nearest Q2.30 value of `S(i)`.
  - **Trig functions:** every angle, plus random high bits and negative angles, for sin, cos,
    sincos (aliased outputs too) and tan. Symmetry identities and fixed values are also checked.
  - **Quotients:** 4,000,000 quotients over every `f` from 0 to 31 (2,157,275 on the divider
    path), boundary-biased, plus an edge table. 2,000,000 more pairs cover the named forms and
    the equalities with `sh2z_div` and `sh2z_recip`.
  - **Negative controls:** 11, all caught.
    - Table: truncation instead of rounding, the libm π, rounding from binary32.
    - Sine and cosine: cosine taken backward, the wrong sign bit.
    - Tangent: truncation, pole signs swapped.
    - Quotients: floor instead of truncation, exact −2^31 treated as fitting, `0/0 = 0`, a
      dividend shifted one bit short.
- **MODELLED:** [`benchmarks/trig12-model`](benchmarks/trig12-model/README.md) checks every
  output of the compiled SH-2 code against the same oracle before timing:
  - every angle and random angles for all four trig functions;
  - start/finish at `f` = 16, 30, 0 and 31 with overflow flags, and the four named quotient
    forms.

  The harness's divider stand-in traps any division the library promised never to issue. It also
  checks that the number of hardware divisions equals the oracle's count of fitting quotients.
  This shows that SH-2 and host results are identical. It is not evidence about the console's
  divider.
- **Not done:** console or emulator runs, cache and memory timing, and confirmation of the
  divider's latency and register behaviour on hardware (see [DIVU.md](DIVU.md)).
