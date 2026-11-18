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

The existing captures (`view-model`, `shade-model`, `vlc-model`) print exactly
what they printed before, because the model never writes the interpreter's
cycle counter. This is checked three ways:

- Locally, each capture's raw output was byte-identical with and without
  `MODEL_PLACEMENT_LOG`.
- In CI, the collector recomputes every CPU-only figure from the log and fails
  unless it equals the value that capture published in the same run.
- The collector also records whether each figure equals the committed capture
  (`matchesCommitted`). All 60 do.

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

The tables come from the CI capture recorded in [results.json](results.json).

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
then run the three captures with the log on and collect the results:

```sh
MODEL_PLACEMENT_LOG=$PWD/build/sh2-model/view-placement.jsonl MODEL_SUITE=view \
  bash saturn/benchmarks/sh2-model/run.sh \
  saturn/benchmarks/yaul/build/sh2zam-benchmark.elf \
  saturn/benchmarks/yaul/build/sh2zam-benchmark.sym build/sh2-model/view-results.json
MODEL_PLACEMENT_LOG=$PWD/build/sh2-model/shade-placement.jsonl \
  bash saturn/benchmarks/shade-model/run.sh build/sh2/shade-model.elf build/sh2-model/shade-results.json
MODEL_PLACEMENT_LOG=$PWD/build/sh2-model/vlc-placement.jsonl \
  bash saturn/benchmarks/vlc-model/run.sh build/sh2/vlc-model.elf build/sh2-model/vlc-results.json
python3 saturn/benchmarks/placement-model/collect.py \
  --view-log build/sh2-model/view-placement.jsonl --view-results build/sh2-model/view-results.json \
  --shade-log build/sh2-model/shade-placement.jsonl --shade-results build/sh2-model/shade-results.json \
  --vlc-log build/sh2-model/vlc-placement.jsonl --vlc-results build/sh2-model/vlc-results.json \
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
