# Validation record — 2026-11-20 (line field second-lap re-target, line-field-2)

`sh2zam/line_field_experimental.h` ([guide](../LINE_FOLLOWER.md)) adds one brain field,
`finish_ticks`. An agent with a finish target re-paces when it completes its first lap: it covers
the remaining laps evenly by that race tick, and its expectation restarts at one lap. With
`finish_ticks = 0` the field is line-field-1, step for step. **No console run was done in this
repository.** Evidence by tier:

- **HOST:** `line_field_exact` passes 11,621,925 checks under GCC and Clang (`-O2 -Wall -Wextra
  -Werror -pedantic`, ASan and UBSan):
  - the line-field-1 race hashes of seeds 1 to 6 are pinned and unchanged with no finish targets,
    and the control (a changed pin) fails;
  - a lone agent with a 1,500-tick lap pace meets finish targets of 3,900, 4,100, 4,800 and 5,200
    ticks within 1% over three laps, its first lap unchanged; the control (no target) misses each
    by more than 4%;
  - a target already behind the first lap is ignored.
- **SH-2 build:** the source compiles with `-m2 -mb -O2 -Wall -Wextra -Werror`; the object is
  6,668 bytes (6,412 for line-field-1). The re-target's one division per agent and race is a
  libgcc `__sdivsi3` call; the step has no other division.
- **Not done:** an instruction-model capture, console execution and any frame measurement.
