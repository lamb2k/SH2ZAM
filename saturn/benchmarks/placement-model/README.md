# Memory placement model (SH-2 instruction model)

The SH-2 instruction model ([`../sh2-model`](../sh2-model/README.md), pinned
Yabause) charges every memory access the same. On a Saturn, where code and data
sit changes what a kernel costs: a cache line fills in about 7 clocks from high
work RAM and in about 56 from low work RAM, and every VDP access waits for the
bus. This harness option adds those costs **beside** the interpreter's cycle
counter. It never changes that counter, so every CPU-only number in every
suite's `results.json` is unchanged.

**Evidence tier: MODELLED.** The region costs come from another project's
hardware notes. This project has not measured them (see
[Calibration status](#calibration-status)).

## What each report gives

For each kernel, [results.json](results.json) gives:

- **CPU-only cycles**, the statistic its own capture publishes. The collector
  recomputes it from the placement log and requires it to equal the published
  value exactly. Each entry names the file and JSON path it matches, for
  example `saturn/benchmarks/view-model/results.json`
  `center.timing.batch.median.centerDivu`.
- **Placed cycles** for seven placements, each warm and cold, computed with the
  same statistic over placed cycles. Placed cycles are CPU-only cycles plus the
  stall cycles that placement adds.
- **Stall breakdown** for each placement and mode: the percentage of stall
  cycles spent on line fills for code, tables, input, output and stack, on
  cache-through reads, and on waits behind a pending write.

The existing captures (`view-model`, `shade-model`, `vlc-model`, `merge-model`) print exactly
what they printed before, because the model never writes the interpreter's
cycle counter. This is checked three ways:

- Locally, each capture's raw output was byte-identical with and without
  `MODEL_PLACEMENT_LOG`.
- In CI, the collector recomputes every CPU-only figure from the log and fails
  unless it equals the value that capture published in the same run.
- The collector also records whether each figure equals the committed capture
  (`matchesCommitted`). All 76 do.

At start-up the model runs a self-test: eight access sequences whose stalls
are worked out by hand. They cover the 4 KB working set, LRU order, four ways,
no write allocate, the write buffer, cache-through reads and write hits. If
any sequence gives the wrong stall, the harness stops. Seven deliberate
mutants of the model were each stopped by the self-test: two ways, 32 sets,
FIFO in place of LRU, write allocate, no write-buffer wait, a cached VDP and
fills that skip the write wait.

## How it works

Turn it on by setting `MODEL_PLACEMENT_LOG=<file>` when you run a capture. The
harness then runs each **tagged** kernel call through an SH7604 cache model,
once per placement, and writes one JSON line per call to that file. The timing
loops tag their calls with `place_tag(label, declaration)` just before
`invoke`. Untagged calls, such as the exactness sweeps, run without the model.

### Buffers and placements

Each 16-byte line of the benchmark image belongs to one of five **classes**:

| Class | What it covers |
| --- | --- |
| code | Every instruction fetch, plus data reads from executable sections (literal pools). |
| tables | Read-only sections, and buffers a kernel declares as tables. |
| input | Writable data the kernel does not declare. This includes caller structs such as the camera. |
| output | Buffers the kernel declares as output, including working arrays it writes. |
| stack | Everything above the image, which is where the harness stack lives. |

A kernel declares its buffers by symbol. Only outputs and tables need a
declaration, because anything else writable defaults to input. A buffer can
change class between kernels. For example, `lit` is output for the light kernel
and tables for Gouraud. The log records each tag's declaration with resolved
addresses and sizes.

| Kernel | tables | output |
| --- | --- | --- |
| view (`center.h`, `merge.h` timing) | library read-only data | `view_out_portable`, `view_out_divu` |
| shade light | `lut`, `light` | `lit` |
| shade Gouraud | `bases`, `lit`, `fog_rgb` | `gouraud`, `saturated` |
| shade cull | none | `kept`, `kept_count` |
| shade fog build | none | `fog_table`, `plain_depth` |
| shade fog lookup | `fog_table` | `fades` |
| shade fade | `fade_slope`, `fade_bias`, `near_rgb`, `far_rgb` | `colours` |
| shade vehicle frame | `lut`, `light`, `bases`, `fog_rgb` | `frame_out`, `saturated`, `kept_count`, and the working arrays `lit`, `kept`, `frame_pairs`, `unique_gouraud` |
| vlc decode | `table` | `out` (the stream and reader state are input) |
| merge-model tier and runs planners | none | `merge_runs` (rails, rows, end masks are input) |

A **placement** assigns each class to a region. The stack always stays in high
work RAM.

| Placement | code | tables | input | output |
| --- | --- | --- | --- | --- |
| `best` | high WRAM | high WRAM | high WRAM | high WRAM |
| `typical` | high WRAM | high WRAM | low WRAM | low WRAM |
| `worst` | cart (10/beat) | cart (10/beat) | cart (10/beat) | low WRAM |
| `codeLow` | low WRAM | high WRAM | high WRAM | high WRAM |
| `cartTables` | high WRAM | cart (6/beat) | low WRAM | low WRAM |
| `outVdp1` | high WRAM | high WRAM | low WRAM | VDP1 VRAM |
| `outVdp2` | high WRAM | high WRAM | low WRAM | VDP2 |

`typical` is library code linked into high work RAM with the working arrays in
low work RAM, which is how a road renderer we serve lays out its data. `worst`
keeps the code and everything the kernel reads in the cartridge at the slow end
of its assumed range.

### Region costs

All costs are in SH-2 clocks. Cited figures are hardware measurements from a
Saturn port, 2026-11-12. That port's notes take the work RAM figures from
Sega's Saturn programming FAQ and the VDP figures from an emulator's commented
bus table. The port checked the resulting model only against whole-frame
readings on a console. Rows marked **ASSUMED** have no published figure.

| Region | Line fill (16 bytes) | Read, cache-through | Write (byte, word, long) | Basis |
| --- | ---: | ---: | --- | --- |
| high WRAM | 7 | (not used) | 4, 4, 4 | cited: fill about 7, write 4 a longword. Byte and word writes **ASSUMED** the same. |
| low WRAM | 56 | (not used) | 7, 7, 14 | cited: 14 a longword (4 × 14 a line), 7 a word. Byte **ASSUMED** to cost as a word. |
| cart, 6 clocks a beat | 48 | (not used) | 6, 6, 12 | **ASSUMED**: low end of the 48–80 range |
| cart, 10 clocks a beat | 80 | (not used) | 10, 10, 20 | **ASSUMED**: high end of the 48–80 range |
| VDP1 VRAM | (cache-through) | 45 | 45, 45, 45 | cited: 45 per access, any size |
| VDP2 | (cache-through) | 40 | 40, 40, 40 | cited: 40 per access, any size |

### The cache model

The cache is modelled in full rather than simplified. It follows the SH7604:

- 4 KB, 4 ways × 64 sets × 16-byte lines, indexed by address bits 9:4, with
  true LRU replacement.
- One cache shared by instruction fetches and data.
- Write-through, with no write allocate.

Costs are charged as follows:

- **Read or fetch miss.** The whole line is filled, and the CPU stalls for the
  region's fill cost. Instruction fetch misses are charged.
- **VDP1 and VDP2.** These are cache-through, so every read stalls for the
  region's access cost.
- **Writes.** Every write goes to the bus through a one-entry write buffer. The
  CPU carries on, and the next bus access (fill, cache-through read or write)
  waits until the previous write has finished. A write that hits the cache
  updates the line and its LRU age.

These are the stated simplifications:

- The cache sees the benchmark image's own addresses, so set conflicts follow
  that image's layout. The region only sets the price of a miss or a write.
- Classes are assigned per 16-byte line.
- Stalls are added after the instruction's own cycles. The interpreter's
  1-cycle access is not split out, and critical-word-first refill is not
  modelled: a fill costs its whole line.
- These are not modelled:
  - the second SH-2, DMA or the 68000 sharing the bus;
  - SDRAM refresh and page effects;
  - cache-through reads of work RAM or the cartridge (there is no figure for a
    single read, so no placement uses them);
  - the bus cost of the on-chip divider (its own harness model applies);
  - two-way cache mode;
  - interrupts.

### Warm and cold

- **warm**: the cache as the previous call of the same kernel in the same
  fixture left it. This models the kernel called in a loop with nothing in
  between. Code and reused scratch buffers stay cached. Streaming inputs and
  outputs still miss. The first call starts with an empty cache.
- **cold**: the cache is empty at entry, so every line the call touches costs
  one fill.

A real caller sits between the two. Per-element slopes (shade, vlc) subtract
the 1-element call, so they exclude one-off code fills even when cold.

## Results

The tables below come from the CI capture recorded in [results.json](results.json):
[CI run 44513686872](https://github.com/lamb2k/SH2ZAM/actions/runs/44513686872) (merge train 3, push run on `eac28d8`).
Its values equal, value for value, those of
[SH-2 cross compile run 44256084857](https://github.com/lamb2k/SH2ZAM/actions/runs/44256084857),
source commit `b9ae01dbe4866b58b1faf2a50577ca3427a73839` (sections-1). Its CPU-only values
equal those of the earlier captures ([run 44213194513](https://github.com/lamb2k/SH2ZAM/actions/runs/44213194513)
on `bd63fdd`, and the first, [run 44209972735](https://github.com/lamb2k/SH2ZAM/actions/runs/44209972735)
on `f013630`, which matches local runs). Placed values of the view and vlc kernels moved
when sections-1 put each hand-written kernel in its own section: the images' layouts
changed, and the cache's set conflicts follow the image's addresses.
Fixture ids name the `results.json` entry, and each entry names the published
value it matches. CPU-only numbers are the published ones, unchanged.

Best, typical and worst placement. Each cell is warm / cold, then the warm ratio to CPU-only.

| Kernel | Fixture | Unit | CPU-only | best | typical | worst |
| --- | --- | --- | ---: | ---: | ---: | ---: |
| `sh2z_view_batch_center_divu` | center.batch | cycles per vertex | 147.45 | 156.99 / 158.79 (1.06x) | 198.20 / 207.78 (1.34x) | 210.91 / 229.99 (1.43x) |
| `sh2z_view_project_center_divu` | center.project | cycles per point | 106.04 | 122.83 / 123.18 (1.16x) | 214.58 / 214.94 (2.02x) | 251.90 / 254.83 (2.38x) |
| `sh2z_view_edge_steps_center_divu` | center.edgeSteps | cycles per point | 182.64 | 202.59 / 222.11 (1.11x) | 256.05 / 330.44 (1.40x) | 316.47 / 481.33 (1.73x) |
| `sh2z_view_clip_near_center_divu` | center.clipNear | cycles per quad | 1,776 | 1,967 / 2,917 (1.11x) | 2,145 / 3,596 (1.21x) | 2,145 / 11,063 (1.21x) |
| `sh2z_view_clip_guard_center_divu` | center.guardInside | cycles per polygon | 2,955 | 3,606 / 4,767 (1.22x) | 3,983 / 5,830 (1.35x) | 7,584 / 18,430 (2.57x) |
| `sh2z_view_clip_guard_center_divu` | center.guardSide | cycles per polygon | 4,669 | 6,055 / 7,150 (1.30x) | 6,657 / 8,291 (1.43x) | 15,442 / 25,490 (3.31x) |
| `sh2z_view_clip_guard_center_divu` | center.guardNearSide | cycles per polygon | 7,758 | 9,619 / 10,613 (1.24x) | 10,221 / 11,754 (1.32x) | 22,676 / 31,581 (2.92x) |
| `sh2z_view_clip_guard_src_center_divu` | srcGuard.inside | cycles per polygon | 2,879 | 3,086 / 4,452 (1.07x) | 3,417 / 5,524 (1.19x) | 3,417 / 16,664 (1.19x) |
| `sh2z_view_clip_guard_src_center_divu` | srcGuard.side | cycles per polygon | 4,583 | 5,710 / 6,977 (1.25x) | 6,616 / 8,179 (1.44x) | 13,821 / 25,695 (3.02x) |
| `sh2z_view_clip_guard_src_center_divu` | srcGuard.nearSide | cycles per polygon | 6,875 | 8,334 / 9,544 (1.21x) | 9,343 / 10,749 (1.36x) | 18,367 / 29,797 (2.67x) |
| `sh2z_view_clip_guard_src_center_divu` | srcGuard.roadNear | cycles per polygon | 8,870 | 11,165 / 12,034 (1.26x) | 12,213 / 13,176 (1.38x) | 27,073 / 34,469 (3.05x) |
| `sh2z_view_merge_runs` | merge.pixel1 | cycles per boundary | 223.11 | 259.67 / 278.56 (1.16x) | 328.17 / 431.28 (1.47x) | 400.16 / 595.59 (1.79x) |
| `sh2z_view_merge_runs` | merge.rows1 | cycles per boundary | 527.14 | 591.78 / 609.27 (1.12x) | 657.41 / 754.19 (1.25x) | 734.55 / 922.39 (1.39x) |
| `sh2z_view_merge_tier` | mergeTier.pixel1.tier | cycles per boundary | 38.74 | 46.84 / 66.25 (1.21x) | 78.09 / 202.53 (2.02x) | 123.73 / 337.19 (3.19x) |
| `sh2z_view_merge_tier` | mergeTier.short8Pixel1.tier | cycles per boundary | 65.50 | 69.50 / 143.88 (1.06x) | 69.50 / 290.12 (1.06x) | 69.50 / 853.38 (1.06x) |
| `sh2z_shade_light_normals` | shade.fast_light_normals | cycles per normal | 99.43 | 99.48 / 101.69 (1.00x) | 105.13 / 119.90 (1.06x) | 105.22 / 130.85 (1.06x) |
| `sh2z_shade_gouraud` | shade.fast_gouraud | cycles per pair | 134.19 | 134.53 / 136.54 (1.00x) | 136.04 / 148.84 (1.01x) | 137.86 / 161.05 (1.03x) |
| `sh2z_shade_cull` | shade.fast_cull | cycles per face | 82.81 | 89.81 / 93.09 (1.08x) | 124.88 / 154.79 (1.51x) | 147.71 / 185.86 (1.78x) |
| `sh2z_shade_fog_build` | shade.fast_fog_build | cycles per table | 90,646 | 96,203.50 / 97,276.50 (1.06x) | 99,762.50 / 102,577.50 (1.10x) | 105,332.50 / 112,771 (1.16x) |
| `sh2z_shade_fog_lookup` | shade.fast_fog | cycles per depth | 41.79 | 43.56 / 43.89 (1.04x) | 53.71 / 56.40 (1.29x) | 62.00 / 66.05 (1.48x) |
| `sh2z_shade_fade_offset` | shade.fast_offset | cycles per fade | 82.50 | 82.50 / 82.94 (1.00x) | 82.55 / 86.07 (1.00x) | 82.58 / 87.62 (1.00x) |
| `sh2z_shade_fade_mix` | shade.fast_mix | cycles per fade | 80.00 | 80.01 / 80.44 (1.00x) | 80.06 / 83.69 (1.00x) | 80.08 / 85.19 (1.00x) |
| light + cull + gouraud kernels | shade.fast_frame | cycles per vehicle | 55,683 | 60,020 / 60,205 (1.08x) | 76,392.50 / 77,670.50 (1.37x) | 98,929.50 / 100,992 (1.78x) |
| kernels, baked unique pairs | shade.fast_frame_baked | cycles per vehicle | 70,955.50 | 75,783.50 / 75,909.50 (1.07x) | 94,951 / 95,517.50 (1.34x) | 119,220 / 120,375 (1.68x) |
| port C | shade.plain_frame | cycles per vehicle | 84,697 | 90,012.50 / 90,752 (1.06x) | 101,586.50 / 105,030 (1.20x) | 113,957.50 / 121,516.50 (1.35x) |
| `sh2z_vlc8_decode` | vlc.zipf256_w6_l12.fast | cycles per symbol | 40.51 | 40.82 / 40.98 (1.01x) | 42.65 / 43.37 (1.05x) | 43.94 / 45.88 (1.08x) |
| `sh2z_vlc8_decode` | vlc.uniform256_w8_l8.fast | cycles per symbol | 31.00 | 31.40 / 31.49 (1.01x) | 33.94 / 34.54 (1.09x) | 35.56 / 36.61 (1.15x) |
| `sh2z_vlc8_decode` | vlc.few40_w5_l10.fast | cycles per symbol | 25.61 | 25.64 / 25.88 (1.00x) | 25.78 / 27.26 (1.01x) | 25.92 / 28.71 (1.01x) |
| `sh2z_vlc8_decode_portable` | vlc.zipf256_w6_l12.portable | cycles per symbol | 73.92 | 74.16 / 74.34 (1.00x) | 75.57 / 76.73 (1.02x) | 76.55 / 78.74 (1.04x) |
| contributed C | vlc.zipf256_w6_l12.plain | cycles per symbol | 75.31 | 75.45 / 75.73 (1.00x) | 76.29 / 78.12 (1.01x) | 76.83 / 80.14 (1.02x) |

Other placements, warm cache (ratio to CPU-only):

| Fixture | CPU-only | codeLow | cartTables | outVdp1 | outVdp2 |
| --- | ---: | ---: | ---: | ---: | ---: |
| center.batch | 147.45 | 161.87 (1.10x) | 198.20 (1.34x) | 292.62 (1.98x) | 276.12 (1.87x) |
| center.project | 106.04 | 127.62 (1.20x) | 214.58 (2.02x) | 309.20 (2.92x) | 292.68 (2.76x) |
| center.edgeSteps | 182.64 | 241.64 (1.32x) | 256.05 (1.40x) | 552.16 (3.02x) | 505.56 (2.77x) |
| center.clipNear | 1,776 | 1,967 (1.11x) | 2,145 (1.21x) | 2,734 (1.54x) | 2,639 (1.49x) |
| center.guardInside | 2,955 | 6,007 (2.03x) | 3,983 (1.35x) | 5,183 (1.75x) | 4,993 (1.69x) |
| center.guardSide | 4,669 | 11,888 (2.55x) | 6,657 (1.43x) | 8,133 (1.74x) | 7,898 (1.69x) |
| center.guardNearSide | 7,758 | 17,873 (2.30x) | 10,262 (1.32x) | 11,697 (1.51x) | 11,462 (1.48x) |
| srcGuard.inside | 2,879 | 3,086 (1.07x) | 3,417 (1.19x) | 4,592 (1.59x) | 4,407 (1.53x) |
| srcGuard.side | 4,583 | 10,390 (2.27x) | 6,616 (1.44x) | 8,024 (1.75x) | 7,789 (1.70x) |
| srcGuard.nearSide | 6,875 | 14,214 (2.07x) | 9,343 (1.36x) | 10,765 (1.57x) | 10,530 (1.53x) |
| srcGuard.roadNear | 8,870 | 20,965 (2.36x) | 12,213 (1.38x) | 13,920 (1.57x) | 13,640 (1.54x) |
| merge.pixel1 | 223.11 | 288.75 (1.29x) | 328.17 (1.47x) | 340.64 (1.53x) | 339.00 (1.52x) |
| merge.rows1 | 527.14 | 622.41 (1.18x) | 657.41 (1.25x) | 691.47 (1.31x) | 686.94 (1.30x) |
| mergeTier.pixel1.tier | 38.74 | 68.22 (1.76x) | 78.09 (2.02x) | 87.21 (2.25x) | 86.12 (2.22x) |
| mergeTier.short8Pixel1.tier | 65.50 | 69.50 (1.06x) | 69.50 (1.06x) | 81.19 (1.24x) | 79.69 (1.22x) |
| shade.fast_light_normals | 99.43 | 99.48 (1.00x) | 105.13 (1.06x) | 204.12 (2.05x) | 189.07 (1.90x) |
| shade.fast_gouraud | 134.19 | 135.18 (1.01x) | 136.11 (1.01x) | 137.59 (1.03x) | 137.38 (1.02x) |
| shade.fast_cull | 82.81 | 93.79 (1.13x) | 124.88 (1.51x) | 130.11 (1.57x) | 128.83 (1.56x) |
| shade.fast_fog_build | 90,646 | 99,660.50 (1.10x) | 100,105 (1.10x) | 132,075 (1.46x) | 127,359.50 (1.41x) |
| shade.fast_fog | 41.79 | 44.77 (1.07x) | 54.63 (1.31x) | 67.35 (1.61x) | 62.90 (1.51x) |
| shade.fast_offset | 82.50 | 82.50 (1.00x) | 82.55 (1.00x) | 82.59 (1.00x) | 82.58 (1.00x) |
| shade.fast_mix | 80.00 | 80.01 (1.00x) | 80.06 (1.00x) | 80.10 (1.00x) | 80.09 (1.00x) |
| shade.fast_frame | 55,683 | 70,284 (1.26x) | 77,469.50 (1.39x) | 159,047 (2.86x) | 148,161 (2.66x) |
| shade.fast_frame_baked | 70,955.50 | 86,207.50 (1.21x) | 96,026 (1.35x) | 194,016.50 (2.73x) | 181,037.50 (2.55x) |
| shade.plain_frame | 84,697 | 94,122.50 (1.11x) | 102,283.50 (1.21x) | 184,073.50 (2.17x) | 173,258.50 (2.05x) |
| vlc.zipf256_w6_l12.fast | 40.51 | 41.03 (1.01x) | 42.69 (1.05x) | 52.64 (1.30x) | 50.61 (1.25x) |
| vlc.uniform256_w8_l8.fast | 31.00 | 31.64 (1.02x) | 33.95 (1.10x) | 49.08 (1.58x) | 45.87 (1.48x) |
| vlc.few40_w5_l10.fast | 25.61 | 25.66 (1.00x) | 25.80 (1.01x) | 45.61 (1.78x) | 41.77 (1.63x) |
| vlc.zipf256_w6_l12.portable | 73.92 | 74.33 (1.01x) | 75.59 (1.02x) | 77.69 (1.05x) | 76.04 (1.03x) |
| vlc.zipf256_w6_l12.plain | 75.31 | 75.51 (1.00x) | 76.31 (1.01x) | 77.33 (1.03x) | 76.73 (1.02x) |

### What the model says

These are MODELLED results; read them for the ratios, not the absolute
values.

- **Working arrays in low work RAM** (`typical`) cost the view and cull
  kernels 1.2–2.0× warm:

  | Kernel | Cost, warm |
  | --- | ---: |
  | batch | 1.34× |
  | projection | 2.02× |
  | edge steps | 1.40× |
  | merge | 1.47× |
  | cull | 1.51× |
  | vehicle frame | 1.37× |
  | road guard clip | 1.38× |

  Most of the stall is input line fills (56 clocks a line) and waits behind
  14-clock long writes. For a road path whose arrays sit in low work RAM,
  the `typical` column is the one to quote:

  | Kernel | CPU-only | `typical`, warm |
  | --- | ---: | ---: |
  | `sh2z_view_batch_center_divu` | 147.45 a vertex | 198.20 |
  | `sh2z_view_edge_steps_center_divu` | 182.64 a point | 256.05 |
  | `sh2z_view_merge_runs` (1 px) | 223.11 a boundary | 328.17 |
  | `sh2z_view_clip_guard_src_center_divu` (road quad) | 8,870 a quad | 12,213 |

- **The tier merge planner is now memory-bound.** `sh2z_view_merge_tier`
  costs 38.74 CPU-only cycles a boundary, but reading its rails from low work
  RAM brings it to 78.09 warm (2.02×), and to 123.73 (3.19×) in the cart.
  Placing its rails matters more than its arithmetic.
- **The guard clips' code does not fit the cache.**
  - The body of `sh2z_view_clip_guard_src_center[_divu]` (the static
    `clip_guard_src`) is 4,324 bytes on its own, more than the whole 4 KB
    cache.
  - With a warm cache and everything in high work RAM, code fills are still
    about 60% of the stall for the crossing cases.
  - With code in low work RAM (`codeLow`), the side, near-side and road
    cases cost 2.1–2.6× warm. A source-order clip whose quad crosses nothing
    costs 1.07×.
  - Keep guard-clip code in high work RAM. Calling it in a tight loop does
    not keep it cached.
- **Writes dominate the cheap placements.** With everything in high work
  RAM, waits behind 4-clock long writes are most of the stall for the batch
  (75%) and edge steps (58%). This is because their 20-byte records are
  stored back to back.
- **Writing straight to VRAM** (`outVdp1`, `outVdp2`) costs 1.5–3.0× for
  kernels that write a lot: edge steps 3.02×, projection 2.92×, batch 1.98×,
  vehicle frame 2.86×. Build in work RAM and copy by DMA instead.
- **Compute-bound kernels barely notice placement when warm**: fade
  offset/mix, Gouraud, vlc decode and the plain decoders, at up to 1.09×
  `typical`. Their tables fit in the cache: the vlc table is 2.5 KB, the
  fog table and `bases`/`lit` are smaller. The cold columns show what the
  first call pays, for example a vlc table in the cartridge: `cartTables`
  costs 1.05× warm.

## Calibration status

- These figures are not this project's own measurements. They are hardware
  measurements from a Saturn port, 2026-11-12, and the cartridge rows are
  ASSUMED. Placed cycles are therefore a **model** of what placement costs, not
  console timings. Treat the ratios between placements as the result and the
  absolute values as provisional.
- This project already has a calibration disc image,
  [`../yaul/calibration`](../yaul/calibration/README.md) (P4). It times 32-bit
  loads from high work RAM, low work RAM and a RAM cartridge, each through
  cached and cache-through addresses, and times `sh2z_view_batch[_divu]`.
  Those rows would check the fill costs here, and the view rows would check one
  placed figure end to end. **The image has not been run** on an emulator or a
  console. Every one of its rows is unrun.
- When the image has been run, set the region table in `placement_model.h` from
  its memory rows, re-run CI, and record the change here.

## Reproduce

Build the ELFs as in [`.github/workflows/sh2-cross.yml`](../../../.github/workflows/sh2-cross.yml),
then run the four captures with the log on and collect the results:

```sh
MODEL_PLACEMENT_LOG=$PWD/build/sh2-model/view-placement.jsonl MODEL_SUITE=view \
  bash saturn/benchmarks/sh2-model/run.sh \
  saturn/benchmarks/yaul/build/sh2zam-benchmark.elf \
  saturn/benchmarks/yaul/build/sh2zam-benchmark.sym build/sh2-model/view-results.json
MODEL_PLACEMENT_LOG=$PWD/build/sh2-model/shade-placement.jsonl \
  bash saturn/benchmarks/shade-model/run.sh build/sh2/shade-model.elf build/sh2-model/shade-results.json
MODEL_PLACEMENT_LOG=$PWD/build/sh2-model/vlc-placement.jsonl \
  bash saturn/benchmarks/vlc-model/run.sh build/sh2/vlc-model.elf build/sh2-model/vlc-results.json
MODEL_PLACEMENT_LOG=$PWD/build/sh2-model/merge-placement.jsonl \
  bash saturn/benchmarks/merge-model/run.sh build/sh2/merge-model.elf build/sh2-model/merge-results.json
python3 saturn/benchmarks/placement-model/collect.py \
  --view-log build/sh2-model/view-placement.jsonl --view-results build/sh2-model/view-results.json \
  --shade-log build/sh2-model/shade-placement.jsonl --shade-results build/sh2-model/shade-results.json \
  --vlc-log build/sh2-model/vlc-placement.jsonl --vlc-results build/sh2-model/vlc-results.json \
  --merge-log build/sh2-model/merge-placement.jsonl --merge-results build/sh2-model/merge-results.json \
  --committed-root . --evidence-url local --output build/sh2-model/placement-results.json \
  --markdown build/sh2-model/placement-tables.md
```

To add a kernel:

1. Put `place_tag("<suite>.<row>.<entry>", decl)` just before its timed
   `invoke` call. `decl` is a `{symbol, PC_OUTPUT or PC_TABLES}` list ending in
   `{NULL,0}`.
2. Add a spec to `collect.py` that gives the published JSON path and the
   capture's statistic.

## Licence

`placement_model.h` and `collect.py` are harness code, GPL-2.0-or-later like
the rest of [`../sh2-model`](../sh2-model/LICENSE). The library sources are
unchanged.
