# Integer-kernel instruction model

Executes the compiled big-endian SH-2 object code of the integer kernels
([`m68k.h`](../../include/sh2zam/m68k.h), [`box2.h`](../../include/sh2zam/box2.h),
[`cell16.h`](../../include/sh2zam/cell16.h)) in the same unmodified Yabause SH-2
interpreter as [`sh2-model`](../sh2-model/README.md) (pinned commit
`7e38821dbac265490f115e163c523a939acda759`, flat RAM, no caches, no wait states,
no bus contention). **Cycle numbers are MODELLED. They are not Saturn
measurements, not whole-frame timings, and not emulator-system timings.** No
console run of these kernels has been done.

## What runs

- [`bridge.c`](bridge.c) is compiled for `-m2 -mb` and linked with the library
  objects and `libgcc` by [`model.ld`](model.ld) (code and data in the harness's
  1 MiB RAM). Each kernel has two entry points: `fast_*` calls the library;
  `plain_*` is an independent straightforward C control with the same inputs,
  outputs and flag nibble (division and remainder operators for the 68000 divides,
  a nested loop that sets `1 << (i & 31)` for box overlap, per-cell branches for
  cell placement). Controls share only the library's N/Z helper so the comparison
  isolates the arithmetic.
- [`integer_harness.c`](../sh2-model/integer_harness.c) loads the ELF, writes inputs
  into target RAM, calls the entry points with the SH-2 calling convention and checks
  the stack, callee-saved registers and SR.S after every call, exactly like
  `sh2-model/harness.c`, whose loader and call wrapper it reuses.
- **Verification before timing.** Every fast and plain result (output words, hit
  counts, flag nibble) is compared with an independent host model: the 68000 contract
  written out with bit-serial division (`m68k`), int64 oracles over small, 16-bit and
  full-int32 ranges (`box`), and a per-cell model (`cell16`). Any mismatch exits
  non-zero before any timing is printed. `INTEGER_VERIFY` (default 4,000,000) sets the
  per-operation case count; box and cell kernels run one twentieth of it plus 2,000.
- **Cost.** `meanCycles` is the mean modelled cycles per call over a fixed
  seeded corpus (4,096 draws for `m68k`; 128 corpora per box/cell row), including the
  `rts` and the bridge's one store of the flag nibble or hit count. Box and cell rows
  are whole-call costs including argument validation.

## Reproduce

CI builds the ELF with the Yaul project's pinned toolchain image (see
`.github/workflows/sh2-cross.yml`) and runs the model on the Linux runner:

```sh
$CC -m2 -mb -O2 -ffreestanding -std=c11 -Wall -Wextra -Werror -DSH2Z_REQUIRE_SH2=1 \
    -Isaturn/include -c saturn/benchmarks/integer-model/bridge.c -o integer-bridge.o
$CC -m2 -mb -nostdlib -nostartfiles -Wl,-T,saturn/benchmarks/integer-model/model.ld -Wl,-e,0 \
    -o integer-model.elf integer-bridge.o libsh2zam.a -lgcc      # libsh2zam.a: saturn/src/*.c, -O2 -m2 -mb
INTEGER_VERIFY=4000000 bash saturn/benchmarks/integer-model/run.sh integer-model.elf results.json
```

[`results.json`](results.json) is the committed capture. It records the source commit,
the ELF SHA-256, the toolchain and the Yabause commit. The committed capture was
produced locally with GCC 9.3.0 (compile to assembly) and GNU binutils 2.40
(assemble and link) rather than the CI image's GCC 13.2; CI's own capture is uploaded
as the `sh2-compile-evidence` artifact and may differ in individual cycle counts.

## Not covered

Real memory timing (cache, SDRAM, bus arbitration with the second CPU or DMA),
interrupt latency, and the whole-frame effect of any kernel. A game-level adoption
decision needs a console measurement of the actual frame.
