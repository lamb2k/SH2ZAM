# vlc instruction model

Executes the compiled big-endian SH-2 object code of the variable-length decoding kernels
([`vlc_experimental.h`](../../include/sh2zam/vlc_experimental.h), contract tag `vlc-1`) in
the same unmodified Yabause SH-2 interpreter as [`sh2-model`](../sh2-model/README.md)
(pinned commit `7e38821dbac265490f115e163c523a939acda759`, flat RAM, no caches, no wait
states, no bus contention). **Cycle numbers are MODELLED. They are not Saturn measurements,
not whole-frame timings, and not emulator-system timings.** No console or emulator run of
these kernels has been done. The numbers here are CPU-only; what memory placement adds
(cache fills, slow regions) is in the [placement model](../placement-model/README.md),
which leaves them unchanged.

## What runs

- [`bridge.c`](bridge.c) is compiled for `-m2 -mb` and linked with the library objects and
  `libgcc` by the integer model's [`model.ld`](../integer-model/model.ld). Decoders over one
  table and stream: `fast_decode` (`sh2z_vlc8_decode`, the hand-written SH-2 kernel),
  `fast_decode_odd` (an odd stream address, which must take the portable path),
  `portable_decode` (`sh2z_vlc8_decode_portable`) and `plain_decode` (straightforward C with
  a bit count and variable shifts). Builders: `fast_build` (`sh2z_vlc8_build`),
  `plain_build` (the builder as contributed, with its O(n²) Huffman and full scans) and
  `fast_lengths` (`sh2z_vlc8_build_lengths`).
- [`vlc_harness.c`](../sh2-model/vlc_harness.c) loads the ELF, writes inputs into target RAM,
  calls the entry points with the SH-2 calling convention and checks the stack, callee-saved
  registers and SR.S after every call, like `sh2-model/harness.c`, whose loader and call
  wrapper it reuses.
- **Verification before timing.** Every table of every builder, and every decoded symbol,
  return value and (for the library decoders) reader state, is compared with the independent
  oracle in [`tests/vlc_oracle.h`](../../tests/vlc_oracle.h): random alphabets of 1 to 256
  symbols for every `w` and `lmax`, encoded messages, the same messages decoded past their
  end, truncated streams, random bytes, random tables in the table format, and odd stream
  addresses. During every decode the harness traps any read of the stream buffer past the
  stream's `2 × units` bytes, and checks that nothing after `out[n]` was written. Any
  mismatch exits non-zero before any timing is printed. `VLC_VERIFY` (default 2000) sets the
  number of random cases.
- **Cost.** Five synthetic alphabets, 16 corpora each. Decoders: whole-call cycles for 1 and
  for 4,096 symbols drawn by frequency; `cyclesPerSymbol` is the slope between them, which
  removes the per-call cost, and `cyclesPerBit` divides it by the mean code length.
  Builders: whole-call cycles per table. `fastSpeedup` and `buildSpeedup` are the C controls'
  cycles over the library's.

## Reproduce

CI builds the ELF with the Yaul project's pinned toolchain image (see
`.github/workflows/sh2-cross.yml`) and runs the model on the Linux runner:

```sh
$CC -m2 -mb -O2 -ffreestanding -std=c11 -Wall -Wextra -Werror -DSH2Z_REQUIRE_SH2=1 \
    -Isaturn/include -c saturn/benchmarks/vlc-model/bridge.c -o vlc-bridge.o
$CC -m2 -mb -nostdlib -nostartfiles -Wl,-T,saturn/benchmarks/integer-model/model.ld -Wl,-e,0 \
    -o vlc-model.elf vlc-bridge.o libsh2zam.a -lgcc        # libsh2zam.a: saturn/src/*.c, -O2 -m2 -mb
VLC_VERIFY=2000 bash saturn/benchmarks/vlc-model/run.sh vlc-model.elf results.json
```

[`results.json`](results.json) is the committed capture. It records the source commit, the
ELF SHA-256, the toolchain and the Yabause commit. It is the `vlc-results.json` from the
`sh2-compile-evidence` artifact of the CI run named in [VLC.md](../../VLC.md). Every CI run
uploads a fresh capture there.

## Not covered

Real memory timing (cache, SDRAM, cartridge RAM, bus arbitration with the second CPU or
DMA), the multiplier's real latency and load-use interlocks, interrupt latency, and the
effect on a whole tile decode. A caller's adoption decision needs a measurement of its own
decoder on the target.
