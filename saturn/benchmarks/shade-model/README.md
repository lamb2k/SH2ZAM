# Shading-kernel instruction model

Executes the compiled big-endian SH-2 object code of the shading kernels
([`shade_experimental.h`](../../include/sh2zam/shade_experimental.h), contract tag `shade-1`)
in the same unmodified Yabause SH-2 interpreter as [`sh2-model`](../sh2-model/README.md)
(pinned commit `7e38821dbac265490f115e163c523a939acda759`, flat RAM, no caches, no wait
states, no bus contention). **Cycle numbers are MODELLED. They are not Saturn
measurements, not whole-frame timings, and not emulator-system timings.** No console or
emulator run of these kernels has been done. The numbers here are CPU-only; what
memory placement adds (cache fills, slow regions) is in the
[placement model](../placement-model/README.md), which leaves them unchanged.

## What runs

- [`bridge.c`](bridge.c) is compiled for `-m2 -mb` and linked with the library objects and
  `libgcc` by the integer model's [`model.ld`](../integer-model/model.ld) (code and data in
  the harness's 1 MiB RAM). Each kernel has a `fast_*` entry that calls the library and a
  `plain_*` entry: straightforward C written from the same rules the way a renderer computes
  them today (an unsigned division per light channel, the per-corner `shade` routine, the
  per-face plane test, a 256-entry binary search per depth, signed division by 255 and
  65025 per fade channel). Both see identical inputs and must produce identical outputs.
- `plain_frame`, `fast_frame` and `fast_frame_baked` time one synthetic vehicle frame
  (112 faces, 128 normals, 22 palettes; corners drawn from small windows of neighbouring
  normals): light every normal, cull with an octant order, colour the four corners of
  every kept face. `fast_frame` gathers (palette, normal) pairs per kept corner and calls
  `sh2z_shade_gouraud` once; `fast_frame_baked` colours a per-model list of unique pairs
  baked offline and copies four table entries per kept face.
- [`shade_harness.c`](../sh2-model/shade_harness.c) loads the ELF, writes inputs into
  target RAM, calls the entry points with the SH-2 calling convention and checks the
  stack, callee-saved registers and SR.S after every call, exactly like
  `sh2-model/harness.c`, whose loader and call wrapper it reuses.
- **Verification before timing.** Every output of every `fast_` and `plain_` entry (lit
  normals and the prepared light, Gouraud words and saturation counts, kept face lists,
  fog thresholds, bucket index, fades, fade colours, whole frames) is compared with the
  independent oracle in [`tests/shade_oracle.h`](../../tests/shade_oracle.h) (64- and
  128-bit arithmetic, plain C division, no reciprocal constants, no bucket index) over
  random and edge-case inputs, perspective and adversarial fog parameters. Any mismatch
  exits non-zero before any timing is printed. `SHADE_VERIFY` (default 4000) sets the
  number of random calls per kernel; `verified` in the results counts elements.
- **Cost.** Each row reports whole-call cycles over 64 seeded corpora per entry point,
  including argument checks and the `rts`. `cyclesPerElement` is the slope between the
  one-element row and the largest row, which removes the per-call cost. Inputs are
  realistic: unit-length int8 normals and a rotation basis for the light, a closed mesh
  seen from 1,500 to 20,000 units for culling, perspective fog parameters with depths
  uniform between the near and far planes (and a near-dense, log-uniform set).

## Reproduce

CI builds the ELF with the Yaul project's pinned toolchain image (see
`.github/workflows/sh2-cross.yml`) and runs the model on the Linux runner:

```sh
$CC -m2 -mb -O2 -ffreestanding -std=c11 -Wall -Wextra -Werror -DSH2Z_REQUIRE_SH2=1 \
    -Isaturn/include -c saturn/benchmarks/shade-model/bridge.c -o shade-bridge.o
$CC -m2 -mb -nostdlib -nostartfiles -Wl,-T,saturn/benchmarks/integer-model/model.ld -Wl,-e,0 \
    -o shade-model.elf shade-bridge.o libsh2zam.a -lgcc      # libsh2zam.a: saturn/src/*.c, -O2 -m2 -mb
SHADE_VERIFY=4000 bash saturn/benchmarks/shade-model/run.sh shade-model.elf results.json
```

[`results.json`](results.json) is the committed capture. It records the source commit,
the ELF SHA-256, the toolchain and the Yabause commit. It is the `shade-results.json`
from the `sh2-compile-evidence` artifact of the CI run named in [SHADE.md](../../SHADE.md).
Every CI run uploads a fresh capture there.

## Not covered

Real memory timing (cache, SDRAM, bus arbitration with the second CPU or DMA), the
multiplier's real latency, interrupt latency, and the whole-frame effect of any kernel.
A game-level adoption decision needs a measurement of the actual frame.
