# ADPCM instruction model

Executes the compiled big-endian SH-2 code of the IMA ADPCM decoder
([`adpcm.h`](../../include/sh2zam/adpcm.h)) in the same pinned Yabause SH-2 interpreter
as [`integer-model`](../integer-model/README.md) (commit
`7e38821dbac265490f115e163c523a939acda759`, flat RAM, no caches, no wait states, no bus
contention). **Cycle numbers are MODELLED. They are not Saturn or 32X measurements.**

## What runs

[`bridge.c`](bridge.c) has three entry points over the same stream and state:

- `fast_decode`: the library, `sh2z_ima_decode`;
- `plain_decode`: straightforward C, the bitwise rule inline, one nibble per iteration;
- `step_decode`: a non-inlined step function called once per nibble on a voice record,
  the shape the Altered Beast 32X port's speech lane used before it adopted `adpcm-1`.

[`adpcm_harness.c`](../sh2-model/adpcm_harness.c) loads the ELF, verifies every entry
point's samples and final state against the host reference `sh2z_ima_ref_decode` on
random streams, positions and lengths (any mismatch exits before timing), then reports
mean cycles per call and per nibble for runs of 1, 32 and 512 nibbles over 256 streams.

## Reproduce

The suite follows [Adding a kernel suite](../../CONTRIBUTING-KERNELS.md) and links with the
shared flat-RAM map, `integer-model/model.ld`. In the Yaul toolchain, after the library
objects and archive are in `build/sh2/`:

```sh
SH2Z_CC=sh2eb-elf-gcc SH2Z_OBJDUMP=sh2eb-elf-objdump SH2Z_NM=sh2eb-elf-nm \
  bash saturn/benchmarks/adpcm-model/ci.sh build
```

Then on the host (`ADPCM_VERIFY` sets the verification count, default 2,000,000):

```sh
bash saturn/benchmarks/adpcm-model/ci.sh model
```

[`results.json`](results.json) is a local capture (Yaul image GCC 13.2.0, 400,704 values
verified). CI writes a fresh `build/sh2-model/adpcm-results.json` and uploads it with the
other model outputs.
