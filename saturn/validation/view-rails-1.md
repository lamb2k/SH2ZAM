# Validation record — 2026-11-18 (strip rails in one call, view-rails-1)

`sh2z_view_rails_prepare`, `sh2z_view_edge_rails_center` and `sh2z_view_edge_rails_center_divu`
in `sh2zam/view_experimental.h` ([guide](../VIEW_RAILS.md)) step and project both rails of a strip
quad, with its copied ends, in one call. The view is validated once by a prepared handle, and the
SH-2 `_divu` entry is hand-written assembly. Every stepped run is byte for byte what
`sh2z_view_edge_steps_center` writes, so the error is 0 px (proof in the guide). **No console or
emulator run was done.** Evidence by tier:

- **HOST:** `rails_exact` passes under GCC and Clang Release with `SH2Z_STRICT=ON`, under GCC
  ASan/UBSan, and under Clang Debug ASan/UBSan. It checks against an independent 128-bit oracle
  written from the header text:
  - 7,296 hand cases, 300,000 road strips (2 to 65 boundaries) and 300,000 general strips
    (reversed, unsorted, unequally spaced, spans above 65,535; 119,104 rejected as the contract
    says);
  - 4,242,122 stepped points and 250,753,743 checks, every byte equal to the oracle and to the
    two-call loop with `sh2z_view_edge_steps_center`;
  - the pointer, handle, count, overlap and aliasing rules;
  - 6 mutation controls, all caught.
- **MODELLED:**
  - Verification before timing: in the Yabause interpreter, the assembly entry, the C path and
    the loop it replaces match the oracle on 128,000 calls (25,158 rejected) and 837,840 stepped
    points, plus 38 argument-rule cases ([model](../benchmarks/rails-model/README.md),
    [results](../benchmarks/rails-model/results.json), captured by
    [CI run 44247175665](https://github.com/lamb2k/SH2ZAM/actions/runs/44247175665)).
  - Cost: 941 modelled cycles per quad (both rails) plus 130.6 per stepped point, against 4,073
    plus 154.5 for the loop. On road strips that is 1,273 cycles per quad against 3,536.
  - At a road renderer's edge load (103 rail calls, 408 points per frame) that is 3.55 ms per
    frame against 9.53 ms.
  - Cycle numbers are modelled, not Saturn timings.
- **Object identity:** every other `saturn/src` object is `cmp`-identical to main's under the CI
  flags.
- **Not done:** the setup target of 150 cycles per rail (471 modelled); console or emulator
  execution; cache and memory timing; a whole frame.
