# Validation record — 2026-11-15 (integer kernels)

Branch `cps2-kernels-20261115` adds `sh2zam/m68k.h`, `sh2zam/box2.h` and `sh2zam/cell16.h`
([guide](../INTEGER_KERNELS.md)). **No console run was done.** Evidence by tier:

- **Host-exact:** `m68k_exact`, `box2_exact`, `cell16_exact` pass under GCC and Clang Release and
  Clang Debug with ASan/UBSan (`SH2Z_STRICT=ON`). `SH2Z_M68K_EXHAUSTIVE=1` checked all 2^32 16x16
  multiplies and 200 million random divides (44,618,231,011 checks).
- **Executed SH-2 object code (modelled):** 16,000,308 68000-arithmetic calls, 202,000 box and
  202,000 cell cases verified exact in the Yabause interpreter before any timing
  ([results](../benchmarks/integer-model/results.json), captured by [CI run 44071367448](https://github.com/lamb2k/SH2ZAM/actions/runs/44071367448)
  with the pinned Yaul GCC 13.2). Cycle numbers are modelled, not Saturn timings.
- **Cross build:** every new source compiles for `-m2 -mb` with `-Wall -Wextra -Werror`
  locally (GCC 9.3); the `sh2-cross` workflow builds them with the Yaul image and runs the model.
- **Not done:** console execution, whole-frame measurement, silicon check of the 68000 overflow N/Z
  convention, evaluation of the on-chip divider.
