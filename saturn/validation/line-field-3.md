# Validation record — 2026-11-20 (line field first-lap chase cap, line-field-3)

`sh2zam/line_field_experimental.h` ([guide](../LINE_FOLLOWER.md)) adds `first_lap_cap` to the brain: a
chasing agent holds its target speed to it until its first lap is done. 0 = no cap (line-field-2).
**No console run was done in this repository.** Evidence by tier:

- **HOST:** `line_field_exact` passes 11,621,928 checks under GCC and Clang (`-O2 -Wall -Wextra -Werror
  -pedantic`, ASan and UBSan): a chaser behind a faster leader never targets above a 12-unit cap on
  its first lap and does from its second; the no-cap control passes the cap on the first lap, and a
  tightened assertion (cap minus one unit) fails, as it must. The line-field-1 pinned hashes and the
  line-field-2 re-target checks are unchanged.
- **Consumer:** F Zero S rival bands (6 seeds, original code against the native field) pass on four
  courses where native first-lap chasers had run about 11% fast.
- **Not done:** an instruction-model capture, console execution and any frame measurement.
