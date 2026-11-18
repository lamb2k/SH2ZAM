# Validation record — 2026-11-17 (12-bit-angle trig and divider quotients, trig12-1)

`sh2zam/trig12_experimental.h` and the `trig12-1` section of `sh2zam/divu_experimental.h`
([guide](../TRIG12.md)) add the following:
- Q2.30 sine and cosine for 12-bit angles, from a generated 1,025-entry quarter table.
- A rounded Q16.16 tangent.
- Inline `num * 2^f / den` quotients and reciprocals through the divider, with start/finish forms.

Existing SH-2 objects are byte-identical to `8d2c43b`. **No console or emulator run was done.**
Evidence by tier:

- **HOST:** `trig12_exact` and `divq_exact` pass under GCC and Clang Release with
  `SH2Z_STRICT=ON` and under ASan/UBSan, against the independent oracle `tests/trig12_oracle.h`.
  - **Table:** every entry is the nearest Q2.30 value of its binary64 series value (max
    0.499884 LSB). CI also checks the table against `tools/sin12_table.py`, which verifies it
    with exact rationals.
  - **Trig functions:** every angle for sin, cos, sincos and tan, plus symmetry identities.
  - **Measured deviations:** within 32 LSB of the binary32 4096-entry series table in quadrants
    0–2, and 332 LSB near a full turn, where the series itself is inaccurate. Within 0.514 LSB
    of the true sine. Tangent within 0.5 LSB of the true tangent up to 89.12°, and 8.2 LSB at
    most.
  - **Quotients:** 4,000,000 quotients over every `f` from 0 to 31, with flags. A further
    2,000,000 pairs cover the named forms and their equalities with `sh2z_div` and `sh2z_recip`.
  - **Negative controls:** 11, all caught.
- **MODELLED:** the compiled SH-2 code matches the oracle in the Yabause interpreter before
  timing ([results](../benchmarks/trig12-model/results.json)):
  - every angle, plus random angles, for all four trig functions;
  - start/finish quotients at `f` = 16, 30, 0 and 31, and the four named forms, with the
    hardware division count equal to the oracle's.

  Modelled cycles per element: sincos12 19, sin12 11, tan12 103.5. Blocking q16 is 56, against
  98.5 for `sh2z_div`; recip_q30 is 45.5. Overlapped with a sincos, the quotient adds about 41.
  Cycle numbers are modelled, not Saturn timings, and the divider latency is assumed.
- **Not done:** console or emulator execution, cache and memory timing, divider confirmation on
  hardware.
