# Validation record — 2026-11-17 (view from a camera, view-camera-1)

`sh2zam/view_experimental.h` adds `sh2z_view_from_camera30` and `sh2z_view_place_q12`
([guide](../VIEW_BATCH.md#view-from-a-camera-view-camera-1)). **No console or emulator run was done.**
Evidence by tier:

- **HOST:** `camera_exact` passes under GCC and Clang Release with `SH2Z_STRICT=ON` and under
  ASan/UBSan, against the independent oracle `tests/camera_oracle.h`: every table entry, every
  field of view, 600,000 random calls, boundary sets, 300,000 placements and 8 negative controls
  (4,074,767 checks). Against a binary32 reference pipeline over 15,000 synthetic poses (1.37
  million projected points), every point is within its stated bound and within 0.05 px of the
  exact projection; integer pixels differ from the float pipeline by at most one.
- **MODELLED:** the compiled SH-2 object code matches the oracle on 23,075 calls of each
  function, and the float chain compiled for SH-2 matches the host bit for bit
  ([results](../benchmarks/camera-model/results.json), captured by [CI run 44224191560](https://github.com/lamb2k/SH2ZAM/actions/runs/44224191560)). 691 modelled cycles per view against
  87,395 for the soft-float chain. Cycle numbers are modelled, not Saturn timings.
- **Not done:** console or emulator execution, cache and memory timing, a port's own float
  library.
