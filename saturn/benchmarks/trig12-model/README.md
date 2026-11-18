# trig12 instruction model

Executes the compiled big-endian SH-2 object code of the 12-bit-angle trig kernels
([`trig12_experimental.h`](../../include/sh2zam/trig12_experimental.h)) and the divider
quotients (the `trig12-1` section of [`divu_experimental.h`](../../include/sh2zam/divu_experimental.h))
in the same unmodified Yabause SH-2 interpreter as [`sh2-model`](../sh2-model/README.md):
- pinned commit `7e38821dbac265490f115e163c523a939acda759`;
- flat RAM, no caches, no wait states, no bus contention;
- the harness's divider stand-in, [`divider_model.h`](../sh2-model/divider_model.h), with a
  39-cycle latency.

**Cycle numbers are MODELLED. They are not Saturn measurements, and the divider latency is an
assumption from the hardware manual.** No console or emulator run of these kernels has been
done.

## What runs

- **Bridge:** [`bridge.c`](bridge.c) is compiled for `-m2 -mb` and linked with the library
  objects and `libgcc` by the integer model's [`model.ld`](../integer-model/model.ld).
  - `v_trig` writes `sh2z_sincos12_u30`, `sh2z_sin12_u30`, `sh2z_cos12_u30` and
    `sh2z_tan12_q16` for an array of angles.
  - `v_divq` writes `sh2z_divu_q_start`/`sh2z_divu_finish` at `f` = 16, 30, 0 and 31, with
    overflow flags, plus `sh2z_divu_q16`, `sh2z_divu_q30`, `sh2z_recip_q16` and
    `sh2z_recip_q30`.
  - The `c_*` entries are one loop each over `n` inputs.
- **Harness:** [`trig12_harness.c`](../sh2-model/trig12_harness.c) loads the ELF, writes the
  inputs into target RAM and calls the entry points with the SH-2 calling convention. After every
  call it checks the stack, the callee-saved registers and SR.S. It reuses the loader, call wrapper
  and divider stand-in of `sh2-model/harness.c`.
- **Verification before timing.** Every output is compared with the independent oracle in
  [`tests/trig12_oracle.h`](../../tests/trig12_oracle.h):
  - trig: all 4096 angles, then angles with random high bits, then random words;
  - quotients: boundary-biased operands and an edge table.

  The divider stand-in traps any division the library promised never to issue: a zero
  divisor, a quotient that does not fit, or a write during a division. The harness also
  requires the number of hardware divisions to equal the oracle's count of fitting quotients.
  Any mismatch exits non-zero before timing. `TRIG12_VERIFY` (default 40000) sets the number
  of cases of each kind.
- **Cost.** Each row is the whole-call cycle difference between `n = 4096` and `n = 1`, divided
  by 4095, which removes the call cost.
  - `netCyclesPerElement` also subtracts the bare loop (`loop` for angles, `qloop` for
    quotients).
  - Cost inputs are random angles, and quotients that fit (so the divider path is timed).
  - Controls: binary32 table loads, libgcc soft-float conversion and division, libgcc 64-bit
    division, and the existing `sh2z_sincos`, `sh2z_tan` and `sh2z_div`.

## Reproduce

CI builds the ELF with the Yaul project's pinned toolchain image (see
`.github/workflows/sh2-cross.yml`) and runs the model on the Linux runner:

```sh
$CC -m2 -mb -O2 -ffreestanding -std=c11 -Wall -Wextra -Werror -DSH2Z_REQUIRE_SH2=1 \
    -Isaturn/include -c saturn/benchmarks/trig12-model/bridge.c -o trig12-bridge.o
$CC -m2 -mb -nostdlib -nostartfiles -Wl,-T,saturn/benchmarks/integer-model/model.ld -Wl,-e,0 \
    -o trig12-model.elf trig12-bridge.o libsh2zam.a -lgcc     # libsh2zam.a: saturn/src/*.c, -O2 -m2 -mb
TRIG12_VERIFY=40000 bash saturn/benchmarks/trig12-model/run.sh trig12-model.elf results.json
```

[`results.json`](results.json) is the committed capture. It records the source commit, the
ELF SHA-256, the toolchain and the Yabause commit. It is the `trig12-results.json` from the
`sh2-compile-evidence` artifact of the CI run named in [TRIG12.md](../../TRIG12.md). Every CI
run uploads a fresh capture there.

## Not covered

- Real memory timing: cache, SDRAM, bus arbitration with the second CPU or DMA.
- The divider's real latency and its register behaviour.
- Load-use interlocks.
- Interrupts during a division.
