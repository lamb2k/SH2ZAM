# cell-records instruction model

Executes the compiled big-endian SH-2 object code of the delta coded sprite record kernel
([`cell_records_experimental.h`](../../include/sh2zam/cell_records_experimental.h), contract tag
`cell-records-1`) in the same unmodified Yabause SH-2 interpreter as
[`sh2-model`](../sh2-model/README.md) (pinned commit
`7e38821dbac265490f115e163c523a939acda759`, flat RAM, no caches, no wait states, no bus
contention). **Cycle numbers are MODELLED. They are not Saturn measurements, not
whole-frame timings, and not emulator-system timings.** No console or emulator run of
this kernel has been done.

## What runs

- [`bridge.c`](bridge.c) is compiled for `-m2 -mb` and linked with the library objects and
  `libgcc` by the integer model's [`model.ld`](../integer-model/model.ld). Three entry points
  work on arrays the harness writes before each call:
  - `fast_expand`: the library, trusted entry;
  - `checked_expand`: the library, checked entry;
  - `plain_expand`: an independent straightforward C control with the same contract (one
    cell at a time, every decision taken per cell, every stream value read a byte at a time).
- [`cell_records_harness.c`](../sh2-model/cell_records_harness.c) reuses the loader and call
  wrapper of `sh2-model/harness.c`, which checks the stack, callee-saved registers and SR.S
  after every call.
- **Verification before timing.** Each mismatch exits non-zero before any timing is printed.
  `CELL_RECORDS_VERIFY` (default 300) sets the number of rounds of all 12 forms with random
  list lengths, parameters and offsets. Every entry of all three entry points is compared with
  the independent oracle [`tests/cell_records_oracle.h`](../../tests/cell_records_oracle.h) and
  with the host build of the same library source, as is the running position after the call;
  bytes after the last entry must stay untouched.
- **Cost.** Each row runs 16 corpora with one element and with the row's list length (40 or
  4 cells). The figure per element is the slope between them, which removes the per-call
  cost; `speedupOf_fast_expand` divides the other entries' slope by the library's.

## Reproduce

CI builds the ELF with the Yaul project's pinned toolchain image (see
`.github/workflows/sh2-cross.yml`) and runs the model on the Linux runner:

```sh
$CC -m2 -mb -O2 -ffreestanding -std=c11 -Wall -Wextra -Werror -DSH2Z_REQUIRE_SH2=1 \
    -Isaturn/include -c saturn/benchmarks/cell-records-model/bridge.c -o cell-records-bridge.o
$CC -m2 -mb -nostdlib -nostartfiles -Wl,-T,saturn/benchmarks/integer-model/model.ld -Wl,-e,0 \
    -o cell-records-model.elf cell-records-bridge.o libsh2zam.a -lgcc     # libsh2zam.a: saturn/src/*.c, -O2 -m2 -mb
bash saturn/benchmarks/cell-records-model/run.sh cell-records-model.elf results.json
```

[`results.json`](results.json) is the committed capture once the "Record the cell-records-1
capture" commit lands: it records the source commit, the ELF SHA-256, the toolchain and the
Yabause commit, from the `sh2-compile-evidence` artifact of the CI run named in that commit.
