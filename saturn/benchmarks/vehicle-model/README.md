# Vehicle-kernel instruction model

Executes the compiled big-endian SH-2 object code of the vehicle-kernels-1 kernels
([`shade_experimental.h`](../../include/sh2zam/shade_experimental.h) and
[`view_experimental.h`](../../include/sh2zam/view_experimental.h), contract tag
`vehicle-kernels-1`) in the same unmodified Yabause SH-2 interpreter as
[`sh2-model`](../sh2-model/README.md) (pinned commit
`7e38821dbac265490f115e163c523a939acda759`, flat RAM, no caches, no wait states, no bus
contention, the divider model of `sh2-model/divider_model.h`). **Cycle numbers are
MODELLED. They are not Saturn measurements, not whole-frame timings, and not
emulator-system timings.** No console or emulator run of these kernels has been done.

## What runs

- [`bridge.c`](bridge.c) (MIT) is compiled for `-m2 -mb` and linked with the library
  objects and `libgcc` by the integer model's [`model.ld`](../integer-model/model.ld).
  Entries, each pair on identical inputs:
  - `fast_view_indexed` (`sh2z_view_batch_divu_indexed`) and `port_view_all`
    (`sh2z_view_batch_divu` over every position, what an indexed mesh renderer calls
    today);
  - `fast_facing` (`sh2z_shade_screen_facing`) and `port_facing` (the same rule written
    as a renderer's per-face C loop: gather four corners, AND the outcodes, the
    near/far/limit test, two triangle areas);
  - `fast_light_indexed` (`sh2z_shade_light_normals_indexed`) and `port_light_all`
    (`sh2z_shade_light_normals` over every normal), with `fast_prepare`;
  - `fast_gouraud_approx` (`sh2z_shade_gouraud_approx`) and `port_gouraud`
    (`sh2z_shade_gouraud`).
- [`vehicle_harness.c`](../sh2-model/vehicle_harness.c) (GPL-2.0-or-later) loads the ELF,
  writes inputs into target RAM, calls the entries with the SH-2 calling convention and
  checks the stack, callee-saved registers and SR.S after every call, reusing
  `sh2-model/harness.c`'s loader, call wrapper and divider model.
- **Verification before timing.** Every output of every entry is compared before any
  timing is printed:
  - view: every listed record against the host build of `sh2z_view_batch` (the
    definition), every unlisted record untouched, and the full batch against the host;
    random views and positions over the whole int16 range and projected ellipsoid meshes;
  - facing: kept lists and counts of both entries against `ovf_keep` of
    [`tests/vehicle_oracle.h`](../../tests/vehicle_oracle.h), nothing written past the
    count; vertices near and outside the +/-1023 limit, near/far codes, wide clip
    rectangles (the C path) and projected meshes;
  - light: listed normals against `oracle_normal` of
    [`tests/shade_oracle.h`](../../tests/shade_oracle.h), unlisted bytes untouched,
    repeated indices, prepared lights outside int16 (the C path), saturating ambient plus
    colour, and the full batch against the oracle;
  - Gouraud: every word of both entries against `oracle_gouraud_approx` and
    `oracle_gouraud`, saturation counts, nothing written past the count, and each
    approximate channel equal to the exact one or one less (`gouraudChannelsExact` of
    `gouraudChannels` counts the equal ones on random inputs).
  Any mismatch exits non-zero. `VEHICLE_VERIFY` (default 2000) sets the iterations.
- **Cost.** Each row reports whole-call cycles over 64 seeded corpora at the sizes of a
  68-position, 133-normal, 112-face level of detail seen from 1,500 to 9,000 units:
  view 1, 60 and 68 listed of 68; facing 1 and 97 faces of a projected 9 x 14 ellipsoid;
  light 1, 70 and 133 listed of 133; Gouraud 1, 120 and 448 pairs.
  `cyclesPerElement` is the slope between the 1-element row and the largest row, which
  removes the per-call cost; the whole-mesh `port_view_all` and `port_light_all` do the
  same work in every row, so their whole-call cycles are the comparison.

## Reproduce

```sh
# build libsh2zam.a and the bridge as in .github/workflows/sh2-cross.yml, then:
bash saturn/benchmarks/vehicle-model/run.sh build/sh2/vehicle-model.elf \
  build/sh2-model/vehicle-results.json
```

`run.sh` fetches the pinned Yabause source (sparse checkout) when `YABAUSE_SOURCE` is not
given, refuses a modified tree, builds the harness with the host compiler and writes the
raw and summarised JSON. The committed [`results.json`](results.json) is a CI capture;
see [VEHICLE_KERNELS.md](../../VEHICLE_KERNELS.md) for the numbers and what they do not
show.
