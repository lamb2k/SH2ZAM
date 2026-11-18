# Merge-planner instruction model

Executes the compiled big-endian SH-2 object code of the tier texture-merge planner
`sh2z_view_merge_tier` ([`view_experimental.h`](../../include/sh2zam/view_experimental.h),
contract tag `view-merge-2`) and, for comparison, `sh2z_view_merge_runs` in pixel mode
(`view-merge-1`) in the same unmodified Yabause SH-2 interpreter as
[`sh2-model`](../sh2-model/README.md) (pinned commit
`7e38821dbac265490f115e163c523a939acda759`, flat RAM, no caches, no wait states, no bus
contention). **Cycle numbers are MODELLED. They are not Saturn measurements, not whole-frame
timings, and not emulator-system timings.** No console or emulator run of these planners has
been done. The numbers here are CPU-only; what memory placement adds (cache fills, slow
regions) is in the [placement model](../placement-model/README.md), which leaves them
unchanged.

## What runs

- [`bridge.c`](bridge.c) is compiled for `-m2 -mb` and linked with the library objects and
  `libgcc` by the integer model's [`model.ld`](../integer-model/model.ld). `tier_plan` calls
  `sh2z_view_merge_tier` (the hand-written SH-2 kernel for runs of {2, 4} and {2}, the
  library's C for 8 and 16). `runs_plan` calls `sh2z_view_merge_runs` in pixel mode with the
  rows and end masks a road renderer passes it today.
- [`merge_tier_harness.c`](../sh2-model/merge_tier_harness.c) loads the ELF, writes the
  strip into target RAM (with random bytes in every field the planners must not read), calls
  the entry points with the SH-2 calling convention and checks the stack, callee-saved
  registers and SR.S after every call, like `sh2-model/harness.c`, whose loader and call
  wrapper it reuses.
- **Verification before timing.** Every plan is compared byte for byte, with the bytes after
  it, against an oracle written in the harness from the header text. Strips: road strips
  of 2 to 200 boundaries in front of a synthetic camera, with ×2, ×4, ×8 and ×16 tier
  textures, rows and end masks the way a road renderer builds them, partial first and last
  repetitions, and corners outside a �900 px band flagged `0x80`; and adversarial strips with int16
  extremes, near-linear rails and random outcodes. Tolerances 0, 3, 128 to 1,024, random and
  the maximum. Where every permitted run has equal-height repetitions, `runs_plan` must
  return the same plan on the target. Every fourth strip is called again with one argument
  broken and must return -1 and write nothing. Any mismatch exits non-zero before any timing
  is printed. `MERGE_VERIFY` (default 12000) sets the number of strips.
- **Cost.** 64 road strips per configuration, the same strips for both planners, both of
  which must return the oracle's plan. Cycles are whole calls through the bridge, argument
  checks included. `speedup` is view-merge-1's median over the tier planner's.
- **Drift.** For every merged quad of the timed plans, the harness samples texels (rails and
  inside) and measures the Chebyshev distance between the merged quad's position and the
  one-quad-per-repetition reference's, in pixels. `p50Px` and `p99Px` are upper edges of
  1/1024 px histogram bins; `maxPx` is exact. The run fails if `maxPx` exceeds `boundPx`
  (`tol_q8 / 256`).

## Reproduce

CI builds the ELF with the Yaul project's pinned toolchain image (see
`.github/workflows/sh2-cross.yml`) and runs the model on the Linux runner:

```sh
$CC -m2 -mb -O2 -ffreestanding -std=c11 -Wall -Wextra -Werror -DSH2Z_REQUIRE_SH2=1 \
    -Isaturn/include -c saturn/benchmarks/merge-model/bridge.c -o merge-bridge.o
$CC -m2 -mb -nostdlib -nostartfiles -Wl,-T,saturn/benchmarks/integer-model/model.ld -Wl,-e,0 \
    -o merge-model.elf merge-bridge.o libsh2zam.a -lgcc     # libsh2zam.a: saturn/src/*.c, -O2 -m2 -mb
MERGE_VERIFY=12000 bash saturn/benchmarks/merge-model/run.sh merge-model.elf results.json
```

[`results.json`](results.json) is the committed capture. It records the source commit, the
ELF SHA-256, the toolchain and the Yabause commit. It is the `merge-results.json` from the
`sh2-compile-evidence` artifact of the CI run named in
[VIEW_BATCH.md](../../VIEW_BATCH.md#tier-texture-merge-planner-view-merge-2). Every CI run
uploads a fresh capture there.

## Not covered

Real memory timing (cache, SDRAM, bus arbitration with the second CPU or DMA), load-use
interlocks, interrupt latency, and the effect on a whole frame. A caller's adoption decision
needs a measurement of its own frame on the target.
