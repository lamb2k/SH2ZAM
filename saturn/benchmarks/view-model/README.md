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
- `collect.py` checks the invariants (zero mismatches, two divisions per
  timing vertex, the ELF's embedded source commit, the interpreter pin) and
  writes the results file.

Reproduce from the repository root with the benchmark ELF built as in
`.github/workflows/sh2-cross.yml`:

```sh
MODEL_SUITE=view bash saturn/benchmarks/sh2-model/run.sh \
  saturn/benchmarks/yaul/build/sh2zam-benchmark.elf \
  saturn/benchmarks/yaul/build/sh2zam-benchmark.sym \
  build/sh2-model/view-results.json
```

[results.json](results.json) is the capture from the CI run it names.
