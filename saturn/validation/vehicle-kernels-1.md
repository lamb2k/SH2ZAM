# Validation record — 2026-11-17 (vehicle kernels, vehicle-kernels-1)

`sh2z_shade_screen_facing`, `sh2z_shade_light_normals_indexed` and `sh2z_shade_gouraud_approx`
in `sh2zam/shade_experimental.h`, and `sh2z_view_batch_indexed` / `_divu_indexed` in
`sh2zam/view_experimental.h` ([guide](../VEHICLE_KERNELS.md)), each with a hand-written SH-2
path. Facing, indexed light and indexed view are exact; every approximate Gouraud channel
equals shade-1's or is one less (proven exhaustively; 94.5443% equal over the whole domain).
**No console or emulator run was done.** Evidence by tier:

- **HOST:** `vehicle` passes under GCC and Clang Release with `SH2Z_STRICT=ON` and under
  ASan/UBSan, against the independent oracles `tests/vehicle_oracle.h` and
  `tests/shade_oracle.h`: the Gouraud bound proven over every case, 1,618,022 indexed normals,
  3,976,307 faces, 295,190 indexed vertices, and 9 negative controls (75,743,762 checks).
- **MODELLED:** the compiled SH-2 code matches the oracles on 30,555 view vertices, 251,325
  faces, 130,563 normals and 1,293,261 Gouraud pairs in the Yabause interpreter before timing
  ([results](../benchmarks/vehicle-model/results.json), captured by [CI run 44224191560](https://github.com/lamb2k/SH2ZAM/actions/runs/44224191560)). Modelled cycles per element,
  kernel / today: facing 53.0 / 228.7, indexed light 72.7 / 101 (all normals), approximate
  Gouraud 91.4 / 134.2, indexed view 146.5 / 142.4 (all positions). Cycle numbers are
  modelled, not Saturn timings.
- **Not done:** console or emulator execution, cache and memory timing, a whole frame.
