# Validation record — 2026-11-17 (tier texture-merge planner, view-merge-2)

`sh2z_view_merge_tier` in `sh2zam/view_experimental.h` ([guide](../VIEW_BATCH.md#tier-texture-merge-planner-view-merge-2))
plans texture merges for strips whose permitted runs have equal-height repetitions, without rows
or multiplication, with a hand-written SH-2 kernel for runs of 2 and 4. Every texel of a merged
quad stays within `tol_q8 / 256` px of the one-quad-per-repetition reference (proof in the
guide). **No console or emulator run was done.** Evidence by tier:

- **HOST:** `merge_tier` passes under GCC and Clang Release with `SH2Z_STRICT=ON` and under
  ASan/UBSan, against an independent 128-bit oracle: 140,000 road, stepped-quad and adversarial
  strips, 6,387,501 runs (839,811 merged) equal, 130,022 plans equal `sh2z_view_merge_runs` in
  pixel mode, drift never above the tolerance on 113,284,539 texel samples, and 7 negative
  controls (22,147,066 checks). On road strips the command count equals `view-merge-1`'s at
  0.5, 1 and 2 px.
- **MODELLED:** the compiled planner matches the harness oracle on 806,730 runs of 12,000 strips
  in the Yabause interpreter before timing ([results](../benchmarks/merge-model/results.json), captured by [CI run 44206041141](https://github.com/lamb2k/SH2ZAM/actions/runs/44206041141)).
  38.7 modelled cycles per boundary at 1 px on 64-boundary road strips (65.5 at 8 boundaries),
  4.9 (3.8) times fewer than `view-merge-1`, with the same plans. Cycle numbers are modelled,
  not Saturn timings.
- **Not done:** console or emulator execution, cache and memory timing, a whole frame.
