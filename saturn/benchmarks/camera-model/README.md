# View-from-camera instruction model

This model runs the compiled big-endian SH-2 object code of `sh2z_view_from_camera30` and
`sh2z_view_place_q12` ([`view_experimental.h`](../../include/sh2zam/view_experimental.h),
contract tag `view-camera-1`). It uses the same unmodified Yabause SH-2 interpreter as
[`sh2-model`](../sh2-model/README.md): pinned commit
`7e38821dbac265490f115e163c523a939acda759`, flat RAM, no caches, no wait states and no bus
contention.

**Cycle numbers are MODELLED.** They are not Saturn measurements, whole-frame timings or
emulator-system timings. No console or emulator run of these functions has been done.

## What runs

- [`bridge.c`](bridge.c) is compiled for `-m2 -mb` and linked with the library objects and
  `libgcc` by the integer model's [`model.ld`](../integer-model/model.ld). It has four entries:
  - `fast_camera` calls `sh2z_view_from_camera30`.
  - `fast_place` calls `sh2z_view_place_q12`.
  - `plain_chain` runs the float chain that a view from a camera replaces
    ([`tests/camera_float_ref.h`](../../tests/camera_float_ref.h)), in binary32 with GCC's
    soft-float:
    - the frustum and the look-at matrix, each also packed to 16.16;
    - their product;
    - the float-to-fixed adapter;
    - the centre view through `sh2z_view_place` with `eye * 65536`.
  - `plain_adapter` runs only the adapter and the centre view.
- [`camera_harness.c`](../sh2-model/camera_harness.c) loads the ELF and writes the inputs into
  target RAM. It calls each entry with the SH-2 calling convention and checks the stack,
  callee-saved registers and SR.S after every call. It reuses the loader and call wrapper of
  `sh2-model/harness.c`.
- **Verification before timing.** Any mismatch exits non-zero before any timing is printed.
  - `fast_camera` and `fast_place` are compared with the independent oracle
    [`tests/camera_oracle.h`](../../tests/camera_oracle.h) on every half-angle boundary and on
    random valid and invalid inputs. The harness compares the return value and every field, or
    checks that every output byte is untouched after a failure.
  - `plain_chain` and `plain_adapter` are compared, bit for bit, with the same binary32 chain
    run on the host.
  - `CAMERA_VERIFY` (default 20000) sets the number of random calls.
- **Cost.** 64 camera poses in the int16 world:
  - eye anywhere, with a random forward and an up hint;
  - field of view 40 to 110 degrees;
  - off-axis frustum centre;
  - a 160 x 120 viewport, depth 16 to 8192.

  Whole-call cycles include argument checks and the `rts`. The float chain's soft-float is
  libgcc's, not any particular port's float library, so its ratio is indicative.

## Reproduce

CI builds the ELF with the Yaul project's pinned toolchain image (see
`.github/workflows/sh2-cross.yml`) and runs the model on the Linux runner:

```sh
$CC -m2 -mb -O2 -ffreestanding -std=c11 -Wall -Wextra -Werror -DSH2Z_REQUIRE_SH2=1 \
    -Isaturn/include -c saturn/benchmarks/camera-model/bridge.c -o camera-bridge.o
$CC -m2 -mb -nostdlib -nostartfiles -Wl,-T,saturn/benchmarks/integer-model/model.ld -Wl,-e,0 \
    -o camera-model.elf camera-bridge.o libsh2zam.a -lgcc      # libsh2zam.a: saturn/src/*.c, -O2 -m2 -mb
CAMERA_VERIFY=20000 bash saturn/benchmarks/camera-model/run.sh camera-model.elf results.json
```

[`results.json`](results.json) is the committed capture. It records the source commit, the
ELF SHA-256, the toolchain and the Yabause commit. It is the `camera-results.json` from the
`sh2-compile-evidence` artifact of the CI run named in [VIEW_BATCH.md](../../VIEW_BATCH.md).
Every CI run uploads a fresh capture there.

## Not covered

- Real memory timing: cache, SDRAM, and bus arbitration with the second CPU or DMA.
- The multiplier's real latency.
- The cost of a port's own float library.
