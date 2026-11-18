# vlc-2 instruction model

Executes the compiled big-endian SH-2 object code of the per-symbol variable-length decoder
([`vlc2_experimental.h`](../../include/sh2zam/vlc2_experimental.h), contract tag `vlc-2`)
in the same unmodified Yabause SH-2 interpreter as [`sh2-model`](../sh2-model/README.md)
(pinned commit `7e38821dbac265490f115e163c523a939acda759`, flat RAM, no caches, no wait
states, no bus contention). **Cycle numbers are MODELLED. They are not Saturn measurements,
not whole-frame timings, and not emulator-system timings.** No console or emulator run of
these kernels has been done. Placed figures add the
[placement model](../placement-model/README.md)'s cache and region stalls beside the CPU-only
count; its region costs come from hardware notes, not from measurements by this project.

## What runs

- [`bridge.c`](bridge.c) is compiled for `-m2 -mb` and linked with the library objects and
  `libgcc` by the integer model's [`model.ld`](../integer-model/model.ld). It holds:
  - verification entries: `v2_build` and `v2_check`, and decode loops at every window
    `w` = 1..7 with `sh2z_vlc2_decode` (the inline SH-2 assembly) and
    `sh2z_vlc2_decode_portable` (the C path compiled for SH-2), plus loops whose table changes
    every symbol;
  - a synthetic, generic tile codec (described in [VLC2.md](../../VLC2.md)) decoded by one
    caller loop with five decoders: `tile_v2` (the inline assembly), `tile_v2_packed` (the
    same on byte-packed tiles, so half the tiles start at odd offsets), `tile_v2c` (the
    portable C), `tile_mk` (a marker-bit reader variant of the same assembly, defined in the
    bridge only, for comparison) and `tile_v1` (one vlc-1 `sh2z_vlc8_decode` call per symbol
    on vlc-1 tables of the same codes). `tile_glue` runs the same caller with the symbols read
    from a list, so the caller's own cost can be subtracted;
  - symbol loops over one table for the cost per symbol.
- [`vlc2_harness.c`](../sh2-model/vlc2_harness.c) loads the ELF, writes inputs into target
  RAM, calls the entry points with the SH-2 calling convention and checks the stack,
  callee-saved registers and SR.S after every call, like `sh2-model/harness.c`, whose loader
  and call wrapper it reuses.
- **Verification before timing.** Every table `v2_build` makes and every code it reports is
  compared with the independent oracle [`tests/vlc2_oracle.h`](../../tests/vlc2_oracle.h):
  every length vector of up to 4 symbols with lengths 0..4 at every `w`, then random
  alphabets of 1 to 256 symbols with length limits 1..15. Each accepted table decodes
  encoded messages, the same messages past their end, and random bytes, at random offsets of
  an even and an odd base address; random tables in the format go through `v2_check` and,
  when accepted, decode random bytes; 32 tables in one arena decode a stream whose table
  changes every symbol. For both decoders every symbol, the reader state and the bit
  position must equal the oracle's. During every decode the harness traps any read of
  stream bytes at or past the bound `sh2z_vlc2_reach` gives for the message (or for
  symbols × longest code), and any read of a checked table past its size. Every tile of both
  mixes is decoded by every variant and compared with the generator's tile, with the same
  stream bound for the vlc-2 variants. Any mismatch exits non-zero before any timing.
  `VLC2_VERIFY` (default 1500) sets the number of random cases.
- **Cost.** Two tile mixes of 128 tiles each, every tile one call; per-tile statistics,
  cycles per symbol (per-tile mean over symbols per tile) and the decoder's share
  (a variant's mean minus `tile_glue`'s). Symbol loops: whole-call cycles for 1 and 4,096
  symbols; `cyclesPerSymbol` is the slope between them. With `MODEL_PLACEMENT_LOG` set (as
  CI sets it) every tile call is also run through the placement model, and `run.sh` adds the
  placed cycles per tile for each placement, warm (the cache carries over from the previous
  tile, as when tiles are decoded in a row) and cold.

## Reproduce

CI builds the ELF with the Yaul project's pinned toolchain image and runs the model on the
Linux runner through [`ci.sh`](ci.sh). By hand, with the library objects and archive in
`build/sh2/`:

```sh
SH2Z_CC=sh2eb-elf-gcc SH2Z_OBJDUMP=sh2eb-elf-objdump SH2Z_NM=sh2eb-elf-nm \
  bash saturn/benchmarks/vlc2-model/ci.sh build
YABAUSE_SOURCE=/path/to/pinned/yabause bash saturn/benchmarks/vlc2-model/ci.sh model
```

The capture is `build/sh2-model/vlc2-results.json` in the `sh2-compile-evidence` artifact of
each CI run, with the source commit, the ELF SHA-256, the toolchain and the Yabause commit.

## Not covered

Real memory timing beyond the placement model (SDRAM page effects, bus arbitration with the
second CPU or DMA), the multiplier's real latency and load-use interlocks, and interrupt
latency. The synthetic tile codec is generic; a caller's own decoder and data decide its
real cost, so an adoption decision needs a measurement of that decoder.
