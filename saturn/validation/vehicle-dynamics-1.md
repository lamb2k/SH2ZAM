# Validation record — 2026-11-19 (track-bound vehicle dynamics, vehicle-dynamics-1)

`sh2zam/vehicle_dynamics_experimental.h` ([guide](../VEHICLE_DYNAMICS.md)) steps one hovering
vehicle on a closed spline road, one fixed tick at a time. It covers the pad, surface zones,
steering, damping, boost, ramps, the hover spring, grip, the engine curve, the closest-point
search, laps, walls and the up vector. It is integer only and promises identical host and SH-2
bits. **No console run was done in this repository.** Evidence by tier:

- **HOST:** `vehicle_dynamics_exact` passes 51 checks under GCC and Clang Release with
  `SH2Z_STRICT=ON` and under ASan/UBSan. It runs on a synthetic stadium and covers:
  - spline identities and the closest-point search;
  - launch and the force/damping balance;
  - steering (dead zone, linearity, saturation, lean);
  - a signed wall response, surface zones, laps and the race clock;
  - the line-space view;
  - a golden state hash.

  Each check has a negative control.
- **MODELLED:** the `vehicle-dynamics-model` suite runs the SH-2 object code in the pinned Yabause
  interpreter: flat RAM, no caches and no wait states, so these are not Saturn timings. It first
  checks every tick and probe word for word against the host build:
  - 5,808 ticks over five input scripts, half of them with SR.S set;
  - 1,366 closest-point searches, plus 86 that report the target off the line.

  Mean cost: `sh2z_vd_step` takes 9,420.7 cycles per tick (median 9,045, max 16,187).
  `sh2z_vd_locate` takes 6,411.8 cycles, and `sh2z_vd_lap_distance` takes 394.7. The capture
  is from CI run 44660318294, ELF SHA-256 `72836f49...`.
- **SH-2 build:** the source compiles with the CI flags and leaves nothing in plain sections
  under the consumer flags. The object is 24,148 bytes (`-m2 -mb -O2`). It needs
  `sh2z_divu_div64`, the `fixed-unit-1` and unit30 helpers, and `___ashiftrt_r4_14`.
- **Not done:** console execution and any frame measurement.
