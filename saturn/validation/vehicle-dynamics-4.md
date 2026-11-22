# Validation record — 2026-11-22 (engine force carried in Q(16+n), vehicle-dynamics-4)

`SH2Z_VD_FORCE_FRAC` in `sh2zam/vehicle_dynamics_experimental.h` ([guide](../VEHICLE_DYNAMICS.md)) carries the engine
force constants and the body's `accel_force` with 16 to 28 fraction bits (build time, default 16). At 16 every earlier
tag's object code is unchanged.
**No console run was done in this repository.** Evidence by tier:

- **Default build unchanged (SH-2, `sh2eb-elf-gcc -m2 -mb -O2 -ffreestanding -std=c11 -Wall -Wextra -Werror`):** all 51
  objects of `saturn/src` (disassembly, relocations, section contents and symbols) are identical to main's.
- **HOST:** `vehicle_force_exact` passes 134,467 checks under GCC and Clang with `-Wall -Wextra -Wconversion -Wshadow
  -Wpedantic -Werror`, under Clang ASan and UBSan, and through the CMake `SH2Z_STRICT=ON` build:
  - the stored constant is the delivered force: with the slew saturated above the crossover `accel_force` equals the
    stored constant exactly, within half a step of 2^-24 of the exact force (0.002 Q16 steps), where the Q16 constant of
    the same force is 0.245 Q16 steps off;
  - 200,000 randomised states (boost, plate boost, slide turbo, curve and bump, top mix, turn and lean costs, slew, ice
    and reverse speed) against a long-double oracle of the same chain: `accel_force` within 2.7 force steps (bound 5) and
    the acceleration within 1.5 Q16 steps (bound 2);
  - a force beyond the range saturates at INT32_MAX, a negative one is dropped.
- **Controls (3 of 3 caught):** the oracle with a Q16 carrier, with a carrier truncated to four fewer bits, and with the
  force application one bit short each break the same bounds on the same cases.
- **SH-2 build at 24:** `sh2zam_vehicle_dynamics`, `sh2zam_vehicle_dynamics2`, `sh2zam_vehicle_contact` and
  `sh2zam_vehicle_contact2` compile with the CI flags; no `memcpy`, `memset` or `memmove` symbols; each object is about
  100 to 150 bytes smaller than the default.
- **Not done:** an instruction-model capture and console execution; a caller built at one scale and linked with a
  library at another is not detected (the caller's loader names the scale).
