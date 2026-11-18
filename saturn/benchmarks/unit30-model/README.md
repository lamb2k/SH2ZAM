# unit30 instruction model

Executes the compiled big-endian SH-2 object code of the Q2.30 unit-vector kernels
([`unit30_experimental.h`](../../include/sh2zam/unit30_experimental.h), contract tag
`fixed-unit-1`) in the same unmodified Yabause SH-2 interpreter as
[`sh2-model`](../sh2-model/README.md) (pinned commit
`7e38821dbac265490f115e163c523a939acda759`, flat RAM, no caches, no wait states, no bus
contention). **Cycle numbers are MODELLED. They are not Saturn measurements, not
whole-frame timings, and not emulator-system timings.** No console or emulator run of
these kernels has been done.

## What runs

- [`bridge.c`](bridge.c) is compiled for `-m2 -mb` and linked with the library objects
  and `libgcc` by the integer model's [`model.ld`](../integer-model/model.ld). Each entry
  point loops over arrays the harness writes. There are four families of entry point:
  - `fast_*`: the library. K1 is inline. `sh2z_u30_dot3` is the `MAC.L` sequence and
    `sh2z_vec3_normalize_u30` is the hand-written assembly.
  - `port_*`: the portable C of the two assembly paths, compiled for SH-2.
  - `float_*`: the same algorithm in binary32 C on libgcc soft-float, with a correctly
    rounded integer `sqrtf`.
  - `q16_*`: the straight translation onto today's SH2ZAM Q16.16 calls.
- [`unit30_harness.c`](../sh2-model/unit30_harness.c) reuses the loader and call wrapper
  of `sh2-model/harness.c`. That wrapper checks the stack, callee-saved registers and SR.S
  after every call. The dot product is also called with SR.S set on entry, and must
  restore it.
- **Verification before timing.** Each mismatch below exits non-zero before any timing is
  printed. `UNIT30_VERIFY` (default 400) sets the number of rounds of 128 elements; CI
  uses the default.
  - Against the independent oracle [`tests/unit30_oracle.h`](../../tests/unit30_oracle.h):
    multiply, Q12 multiply, `MAC.L` dot product, portable dot product, cross product and
    `isqrt`.
  - Bit for bit against the host build of the same library source, and within the
    oracle's bounds:
    - the mantissa kernel;
    - `rsqrt64`;
    - the assembly normalisation (also in place), and its portable C on SH-2;
    - both orthonormalisations (results, return codes and partial updates).
- **Cost.** Each row runs 16 corpora with one element and with 128 elements (32 for
  bases). The figure per element is the slope between them, which removes the per-call
  cost; `speedupOf_fast_*` divides the other entries' slope by the library's.

## Reproduce

CI builds the ELF with the Yaul project's pinned toolchain image (see
`.github/workflows/sh2-cross.yml`) and runs the model on the Linux runner:

```sh
$CC -m2 -mb -O2 -ffreestanding -std=c11 -Wall -Wextra -Werror -DSH2Z_REQUIRE_SH2=1 \
    -Isaturn/include -c saturn/benchmarks/unit30-model/bridge.c -o unit30-bridge.o
$CC -m2 -mb -nostdlib -nostartfiles -Wl,-T,saturn/benchmarks/integer-model/model.ld -Wl,-e,0 \
    -o unit30-model.elf unit30-bridge.o libsh2zam.a -lgcc     # libsh2zam.a: saturn/src/*.c, -O2 -m2 -mb
bash saturn/benchmarks/unit30-model/run.sh unit30-model.elf results.json
```

[`results.json`](results.json) is the committed capture. It records the source commit,
the ELF SHA-256, the toolchain and the Yabause commit. It is the `unit30-results.json`
from the `sh2-compile-evidence` artifact of the CI run named in the commit that records
it. Every CI run uploads a fresh capture there.

## Not covered

- Real memory timing: cache, SDRAM, bus arbitration with the second CPU or DMA.
- The multiplier's real latency and the interlocks of `STS` after `DMULS.L`/`MAC.L`.
- The effect on a whole camera or body update.
