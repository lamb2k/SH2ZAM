# Validation record - body contact, body-contact-2

`sh2zam/body_contact_experimental.h` ([guide](../BODY_CONTACT.md)) gains `sh2z_bc_sweep`, `sh2z_bc_range` and `sh2z_bc_field_resolve_list`,
the field walk split into a pair list, an exact range filter and the pair calls, and `sh2z_bc_field_resolve` tests the position box in 32
bits before calling the pair function. Every `body-contact-1` result is unchanged. **No console run was done in this repository.**

- **HOST:** `body_contact_exact` passes under GCC and Clang Release with `SH2Z_STRICT=ON` and under ASan/UBSan.
  - the existing checks (random pairs against the double-precision transcription, the field walk against a brute-force walk over all
    pairs, history and credit) pass with the new 32-bit box test in `sh2z_bc_field_resolve`;
  - the list path against the field walk: 20 races of 300 ticks of 30 bodies on a short wrapping line with a window covering every pair in
    reach: outputs, history (`last`), `toucher`, `since`, contacts and pairs in reach equal every tick;
  - the 32-bit box test at the ends of the int32 range: bodies 2^32 - 10 and 2^32 - 20 apart in a coordinate pass it and the pair function
    rejects them with no effect; a pair 10 apart is resolved; the library's own walk gives the same damage.
  - controls: a sweep window of 4 units across and a range filter with half the reach each change the answer and are caught.
- **SH-2 build:** compiles with `-m2 -mb -O2 -Wall -Wextra -Werror`; the object needs `___ashiftrt_r4_30`, `sh2z_divu_div64`,
  `sh2z_isqrt64_q12` and `sh2z_vec3_normalize_u30` and no libc function.
- **Not done:** an instruction-model capture and console execution. The caller's measurement is quoted in its own report.
