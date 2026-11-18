# view-rails instruction model

Executes the compiled big-endian SH-2 object code of the strip-rail functions
([`view_experimental.h`](../../include/sh2zam/view_experimental.h), contract tag
`view-rails-1`) in the same unmodified Yabause SH-2 interpreter as
[`sh2-model`](../sh2-model/README.md) (pinned commit
`7e38821dbac265490f115e163c523a939acda759`, flat RAM, no caches, no wait states, no bus
contention). **Cycle numbers are MODELLED. They are not Saturn measurements, not whole-frame
timings, and not emulator-system timings.** No console or emulator run of these functions
has been done.

## What runs

- [`bridge.c`](bridge.c) is compiled for `-m2 -mb` and linked with the library objects and
  `libgcc` by the integer model's [`model.ld`](../integer-model/model.ld). Entry points:
  `rails_prepare` (`sh2z_view_rails_prepare`), `rails_divu`
  (`sh2z_view_edge_rails_center_divu`, the hand-written SH-2 entry and loop),
  `rails_portable` (`sh2z_view_edge_rails_center`, the C path), `rails_divu_ptrs` and
  `rails_portable_ptrs` (the same with every pointer taken from a table, for the argument
  rules), and `loop_divu`: the loop the functions replace, as a road renderer writes it
  (copy each rail's ends, then one `sh2z_view_edge_steps_center_divu` call per rail with the
  renderer's repetition length as the step).
- [`rails_harness.c`](../sh2-model/rails_harness.c) loads the ELF, writes the centre view,
  the four rail ends, the boundaries and the arguments into target RAM, and calls the entry
  points with the SH-2 calling convention, checking the stack, callee-saved registers and
  SR.S after every call, like `sh2-model/harness.c`, whose loader, call wrapper and divider
  model it reuses.
- **Verification before timing.** Every output record of `rails_divu` and `rails_portable`
  (and of `loop_divu` on road strips) is compared byte for byte with the independent oracle
  in [`tests/rails_oracle.h`](../../tests/rails_oracle.h), for random valid views and a road
  camera, road strips (2 to 65 boundaries on a 256, 512 or 1024 repetition length) and
  general strips (any spans, reversed spans, unequal spacing, unsorted boundaries, spans
  above 65,535). Calls the contract rejects must return 0 and leave both outputs untouched;
  nothing outside the call's records may change. Then the argument rules: each null pointer,
  an unprepared handle, count 0 and the maximum + 1, overlapping and wrapping outputs,
  adjacent outputs and boundary words inside an output. Any mismatch exits non-zero before
  any timing is printed. `RAILS_VERIFY` (default 4000) sets the number of views (16 strips
  each).
- **Cost.** `perCallAndPoint`: 0 to 63 stepped points per rail, 8 repeats, separate and
  shared spans, on the road camera with every point in front of the near plane; a least
  squares line gives cycles per quad (one call, both rails) and per stepped point.
  `roadStrips`: 4,000 road strips, mean cycles per quad of each entry. `frameEstimateMs`
  (added by [`run.sh`](run.sh)) applies the line to a road renderer's measured edge load:
  103 rail calls (51.5 quads) and 408 stepped points per frame on average, 809 at p99, at
  28.6364 MHz.

## Reproduce

CI builds the ELF with the Yaul project's pinned toolchain image (see
`.github/workflows/sh2-cross.yml`) and runs the model on the Linux runner:

```sh
$CC -m2 -mb -O2 -ffreestanding -std=c11 -Wall -Wextra -Werror -DSH2Z_REQUIRE_SH2=1 \
    -Isaturn/include -c saturn/benchmarks/rails-model/bridge.c -o rails-bridge.o
$CC -m2 -mb -nostdlib -nostartfiles -Wl,-T,saturn/benchmarks/integer-model/model.ld -Wl,-e,0 \
    -o rails-model.elf rails-bridge.o libsh2zam.a -lgcc     # libsh2zam.a: saturn/src/*.c, -O2 -m2 -mb
RAILS_VERIFY=4000 bash saturn/benchmarks/rails-model/run.sh rails-model.elf results.json
```

[`results.json`](results.json) is the committed capture. It records the source commit, the
ELF SHA-256, the toolchain and the Yabause commit. It is the `rails-results.json` from the
`sh2-compile-evidence` artifact of the CI run named in [VIEW_BATCH.md](../../VIEW_BATCH.md).
Every CI run uploads a fresh capture there.

## Not covered

Real memory timing (cache, SDRAM, bus arbitration with the second CPU or VDP1), interrupts
during a division, and the renderer around the call. The frame estimate is the model's line
at the renderer's counts, not a frame measurement.
