# pose instruction model

Executes the compiled big-endian SH-2 object code of `sh2z_pose_from_f32`
([`view_experimental.h`](../../include/sh2zam/view_experimental.h), contract tag `pose-1`) in
the same unmodified Yabause SH-2 interpreter as [`sh2-model`](../sh2-model/README.md)
(pinned commit `7e38821dbac265490f115e163c523a939acda759`, flat RAM, no caches, no wait
states, no bus contention). **Cycle numbers are MODELLED. They are not Saturn measurements,
not whole-frame timings, and not emulator-system timings.** No console or emulator run of
this function has been done.

## What runs

- [`bridge.c`](bridge.c) is compiled for `-m2 -mb` and linked with the library objects and
  `libgcc` by the integer model's [`model.ld`](../integer-model/model.ld). Entry points:
  `fast_pose` (`sh2z_pose_from_f32`; the harness calls it with SR.S clear, which takes the
  hand-written SH-2 path, and with SR.S set, which takes the C path), `alias_base` and
  `alias_inputs` (the same call with `base` as `&out->view`, or with every float input stored
  inside `*out`), `plain_pose` (the same pose computed with binary32 float arithmetic,
  libgcc soft-float, a checked conversion per value and `sh2z_view_init`, as a renderer that
  keeps float model state computes it), `plain_base` (that renderer's per-call base view) and
  `test_delta`, `test_inverse`, `test_element` (the library's internal SH-2 difference,
  reciprocal and basis-value routines, through test entries).
- [`pose_harness.c`](../sh2-model/pose_harness.c) loads the ELF, writes inputs into target
  RAM, calls the entry points with the SH-2 calling convention and checks the stack, the
  callee-saved registers and SR.S after every call, like `sh2-model/harness.c`, whose loader
  and call wrapper it reuses.
- **Verification before timing.** For `POSE_VERIFY` cases (default 20,000): random poses in
  front of the camera (world positions to ±2^24, any orientation, typical and random
  scales, bases straddling the orthonormality tolerance, both axis orders), a special float
  (zero, subnormal, NaN, infinity, the bounds) in one input, rotations at the ±1.1e9 bound,
  int16 overflow, large positions, invalid base views and unknown flags. Each case runs
  `fast_pose` with SR.S clear and set and `plain_pose`; the success flag and all 37 output
  fields must equal the independent oracle in [`tests/pose_oracle.h`](../../tests/pose_oracle.h),
  a failed call must leave its output untouched, the SH-2 path must execute exactly 63
  `MAC.L` and the C path none. Each accepted case runs again through both aliasing entries
  on both paths. Then 16 × `POSE_VERIFY` values each through the difference, reciprocal and
  basis-value routines, against the oracle's binary32 arithmetic. Any mismatch exits non-zero
  before any timing is printed.
- **Cost.** Two rows of 2,000 visible poses each: one scale on all three axes, and the
  scales 0.1, 0.125 and 0.15. Whole-call cycles for `fast` (SR.S clear), `fastSrS` (SR.S
  set: the C path) and `plain`; `fastSpeedup` is plain over fast. `plainBase` is the
  float procedure's per-call base view, which the library takes as an input instead.

## Reproduce

CI builds the ELF with the Yaul project's pinned toolchain image (see
`.github/workflows/sh2-cross.yml`) and runs the model on the Linux runner:

```sh
$CC -m2 -mb -O2 -ffreestanding -std=c11 -Wall -Wextra -Werror -DSH2Z_REQUIRE_SH2=1 \
    -Isaturn/include -c saturn/benchmarks/pose-model/bridge.c -o pose-bridge.o
$CC -m2 -mb -nostdlib -nostartfiles -Wl,-T,saturn/benchmarks/integer-model/model.ld -Wl,-e,0 \
    -o pose-model.elf pose-bridge.o libsh2zam.a -lgcc       # libsh2zam.a: saturn/src/*.c, -O2 -m2 -mb
POSE_VERIFY=20000 bash saturn/benchmarks/pose-model/run.sh pose-model.elf results.json
```

[`results.json`](results.json) is the committed capture. It records the source commit, the
ELF SHA-256, the toolchain and the Yabause commit. It is the `pose-results.json` from the
`sh2-compile-evidence` artifact of the CI run named in [VALIDATION.md](../../VALIDATION.md).
Every CI run uploads a fresh capture there.

## Not covered

Real memory timing (cache, SDRAM, bus arbitration with the second CPU or DMA), the
multiplier's and divider's real latency and load-use interlocks, interrupt latency, and the
effect on a whole frame. A caller's adoption decision needs a measurement of its own
renderer on the target.
