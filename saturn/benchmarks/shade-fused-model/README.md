# Fused shade instruction model

Executes the compiled big-endian SH-2 object code of the shade-2 kernels
([`shade_experimental.h`](../../include/sh2zam/shade_experimental.h) and
[`cdiv_experimental.h`](../../include/sh2zam/cdiv_experimental.h), contract tag
`shade-2`) in the same unmodified Yabause SH-2 interpreter as
[`sh2-model`](../sh2-model/README.md) (pinned commit
`7e38821dbac265490f115e163c523a939acda759`, flat RAM, no caches, no wait states, no bus
contention). **Cycle numbers are MODELLED. They are not Saturn measurements, not
whole-frame timings, and not emulator-system timings.** No console or emulator run of
these kernels has been done.

## What runs

- [`bridge.c`](bridge.c) (MIT) is compiled for `-m2 -mb` and linked with the library
  objects and `libgcc` by the integer model's [`model.ld`](../integer-model/model.ld).
  Entries:
  - `fast_pieces`, `fast_refs_pieces`: one `sh2z_shade_fog_gouraud` (strided) or
    `sh2z_shade_fog_gouraud_refs` (corner pointers into scattered records) call per piece;
  - `shade1_pieces`: per piece, shade-1's `sh2z_shade_fog_lookup`, the variant, then
    `sh2z_shade_fade_offset` or `sh2z_shade_fade_mix`;
  - `plain_pieces`: a renderer's per-piece C today (a 257-bucket lookup with a search per
    corner, the variant, signed shift-add division by 255 per textured channel, a
    multiply for /65025 per solid channel);
  - `fast_batch`, `plain_batch`: one call, or the same C, over a whole strip;
  - `fast_fades`, `fast_args`: fades read back through an identity ramp, and the argument
    rules of the assembly entry points;
  - `cdiv*`, `shiftadd_div255_s`, `c_div*`: the scalar divides over arrays.
- [`shade_fused_harness.c`](../sh2-model/shade_fused_harness.c) (GPL-2.0-or-later) loads
  the ELF, writes inputs into target RAM, calls the entries with the SH-2 calling
  convention and checks the stack, callee-saved registers and SR.S after every call,
  reusing `sh2-model/harness.c`'s loader and call wrapper.
- **Verification before timing.** Every output of every `fast_`, `shade1_` and `plain_`
  entry is compared with the independent oracle in
  [`tests/shade_fused_oracle.h`](../../tests/shade_fused_oracle.h): `fast_` against the
  shade-2 rule, `shade1_` and `plain_` against shade-1's exact rule; the fused words are
  also checked to be within +1 of the exact rule, and the exact-match rate is reported
  (`fusedChannelsEqualShade1` of `fusedChannels`). A second pass uses adversarial tables
  (extreme parameters, thresholds at 0 and past `INT32_MAX`), depths at and next to every
  threshold, `INT32_MIN`/`INT32_MAX`, random levels and thresholds up to 256, colours at 0
  and 127 and counts 1..16, and checks
  every fade against the source's binary search. The divides are checked at both ends and
  random points of their domains. Any mismatch exits non-zero before any timing is
  printed. `SHADE_FUSED_VERIFY` (default 3000) sets the iterations.
- **Cost.** Each row reports whole-call cycles over 48 seeded corpora per entry point.
  `cyclesPerVertex` gives whole-row cycles per vertex for piece rows (the caller's loop
  included) and whole-call cycles per vertex for strip rows, plus per-vertex slopes;
  `constantDivideCyclesPerValue` is the slope over 255 values minus a plain copy loop.

## Reproduce

```sh
# build libsh2zam.a and the bridge as in .github/workflows/sh2-cross.yml, then:
bash saturn/benchmarks/shade-fused-model/run.sh build/sh2/shade-fused-model.elf \
  build/sh2-model/shade-fused-results.json
```

`run.sh` fetches the pinned Yabause source (sparse checkout) when `YABAUSE_SOURCE` is not
given, refuses a modified tree, builds the harness with the host compiler and writes the
raw and summarised JSON. The committed [`results.json`](results.json) is a CI capture;
see [SHADE_FUSED.md](../../SHADE_FUSED.md) for the numbers and what they do not show.
