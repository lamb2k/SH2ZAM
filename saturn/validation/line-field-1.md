# Validation record — 2026-11-19 (line field, line-field-1)

`sh2zam/line_field_experimental.h` ([guide](../LINE_FOLLOWER.md)) runs up to 255 vehicles on one
line-follower line:
- contact between neighbours;
- rank and gaps;
- level of detail by distance from a focus;
- one xorshift state for every random choice, and a state hash.

Integer only, with no divide at run time. **No console run was done in this repository.** Evidence by tier:

- **HOST:** `line_field_exact` passes 11,621,899 checks under GCC and Clang Release with `SH2Z_STRICT=ON` and under ASan/UBSan:
  - invariants over long random races;
  - an independent grid formula;
  - exact contact arithmetic on hand-placed pairs;
  - determinism and level of detail.
  Every behavioural claim has a control: the same scene with the feature off must fail the
  claim, and does. A multi-tick step approximates that many single steps, as the guide states.
- **SH-2 build:** the source compiles with the CI flags and leaves nothing in plain sections
  under the consumer flags. The object is 6,412 bytes (`-m2 -mb -O2`); it calls only
  line-follower-1 functions.
- **Not done:** an instruction-model capture, console execution and any frame measurement.
