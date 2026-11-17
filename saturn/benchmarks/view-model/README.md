# View batch capture (SH-2 instruction model)

Runs `sh2z_view_batch` and `sh2z_view_batch_divu` from the real big-endian
benchmark ELF in the pinned Yabause SH-2 interpreter, with the harness-side
divider model described in [../../DIVU.md](../../DIVU.md). Evidence tier:
MODELLED. Not console timing, not emulator timing.

- `view_bridge.c` (MIT) is linked into the benchmark ELF: input, camera and
  output buffers plus small entry points the harness calls directly.
- `capture.h` (GPL-2.0-or-later, harness side) generates cameras and positions
  on the host, writes them into target RAM, calls each path and compares every
  output byte with its own 128-bit oracle. It runs:
  1. divider probes: 23 fixed cases and 2,000 random ones, each also
     interrupted by a saved-and-restored second division;
  2. an exactness sweep of 512 cameras x 2,048 positions, plus a 12-byte
     stride run on every eighth camera;
  3. timing on 64 cameras x 512 positions placed inside the view volume.
  4. `edges.h`, view-edges-1: edge steps on 8,192 random edges, the near clip
     on 65,536 quads (every in-front/behind pattern), view-space projection
     on 262,144 points, each against its own oracle, then timing.
  5. `guard.h`, view-guard-1: the guard-band clip on 65,536 random polygons
     against its own oracle, then timing on the port's camera and band.
  6. `merge.h`, view-merge-1: the texture-merge planner on 16,384 random
     strips (road and adversarial) against its own oracle, rejected calls
     writing nothing, then cycles per boundary on 64-boundary road strips.
  7. `round.h`, view-round-1: every `_trunc` entry point (batch, projection,
     edge steps, near clip, guard clip; portable and divider forms) against
     its own oracle, then each timing fixture through the old function and
     its `_trunc` form back to back.
  8. `center.h`, view-center-1: every `_center` entry point against its own
     oracle with fractional centres (points aimed at exact row boundaries),
     the `_center` and `_trunc` forms byte for byte at whole-pixel centres,
     then each timing fixture through the `_trunc` form and its `_center`
     form back to back.
- `collect.py` checks the invariants (zero mismatches, two divisions per
  timing vertex, merge runs covering every repetition, both view-round-1
  differences occurring, view-center-1 exact and equal to view-round-1 at
  whole-pixel centres, the ELF's embedded
  source commit, the interpreter pin) and
  writes the results file.

Reproduce from the repository root with the benchmark ELF built as in
`.github/workflows/sh2-cross.yml`:

```sh
MODEL_SUITE=view bash saturn/benchmarks/sh2-model/run.sh \
  saturn/benchmarks/yaul/build/sh2zam-benchmark.elf \
  saturn/benchmarks/yaul/build/sh2zam-benchmark.sym \
  build/sh2-model/view-results.json
```

[results.json](results.json) is the capture from
[SH-2 cross compile run 43989876956](https://github.com/lamb2k/SH2ZAM/actions/runs/43989876956)
(artifact `sh2-compile-evidence`, id 14286877894), whose ELF embeds source commit
`5f015510520acadefc4f162cd98bdcd61d1f3018`. Median cycles per vertex at 512
vertices: portable 681.02; divider path 130.15 raw, 134.68 stall model, 208.15
with a full 39 cycles per division.
