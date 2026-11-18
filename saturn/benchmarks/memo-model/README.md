# memo instruction model

Executes the compiled big-endian SH-2 object code of the block memo kernels
([`memo_experimental.h`](../../include/sh2zam/memo_experimental.h), contract tag `memo-1`) in
the same unmodified Yabause SH-2 interpreter as [`sh2-model`](../sh2-model/README.md), pinned
at commit `7e38821dbac265490f115e163c523a939acda759`.

Each call is costed two ways:
- **CPU-only**: the interpreter's own count. Flat RAM, no caches and no wait states.
- **Placed**: the same call through the
  [memory placement model](../placement-model/README.md) (`model-placement-1`, an SH7604
  cache, a write buffer and cited region costs). The interpreter's count is never changed.

**Cycle numbers are MODELLED. They are not Saturn measurements.** No console or emulator run
of these kernels has been done.

## What runs

### Bridge

[`bridge.c`](bridge.c) is compiled for `-m2 -mb` and linked with the library objects and
`libgcc` by the integer model's [`model.ld`](../integer-model/model.ld). It has one entry per
library form:
- `w16_batch`, `l16_batch`, `w16_one`, `l16_one`;
- `w16_loop`: the single form called once per block;
- `put4`, and `put4_one` called in a loop;
- `copy8`, `diff8`.

It also has plain C controls written the way the requested shapes describe today's code:
- `plain_w15`: a call per block that compares 7 longwords and 1 word;
- `plain_memo`: a fixed-layout memo with two key bytes and a 16-bit key;
- `plain_all`: every word written, no memo;
- `plain_copy8`: a word copy with the marker tests;
- `full_copy`: a longword copy with no test;
- `diff8_c`: the library's portable C diff.

### Harness

[`memo_harness.c`](../sh2-model/memo_harness.c) loads the ELF, writes the inputs into target
RAM and calls the entries with the SH-2 calling convention. It reuses the loader, the call
wrapper (stack, callee-saved registers and SR.S checked after every call) and the placement
model of `sh2-model/harness.c`. Its memory hooks:
- **Destination log.** Every write to the keyed-write destination is logged. A byte or
  longword write, or any read of the destination, stops the run. The log must equal the
  oracle's write list in order.
- **Marker guard.** During a marker copy, a read of any source byte after the marker record
  stops the run.

### Verification before timing

Every output is compared with the independent oracle in
[`tests/memo_oracle.h`](../../tests/memo_oracle.h): shadows, masks and counts, flags, keyed
writes, the memo, copies and their guards, and runs and their guards. `MEMO_VERIFY` (default
3000) sets the number of cases of each kind. The cases include:
- every block count from 0 to 139;
- random counts up to 160;
- 2-byte-aligned blocks and copies (the portable path);
- random gate descriptors;
- markers at every position;
- `max_runs` truncation.

Any mismatch exits non-zero. Every timed call is verified the same way.

### Negative controls

Eleven planted bugs in the SH-2 kernels each stopped the harness with a mismatch:
- a mask bit that restarts at 2;
- a 64-byte block copied without its last longword;
- an unchanged block reported changed;
- the last longword pair not compared;
- `>` in place of `≥` in the marker test;
- half the marker record copied;
- the memo never stored;
- word 3 taken from word 2;
- `always = 0x8` dispatched to the all-gated loop;
- one run stored past `max_runs`;
- a final run one record too long.

The unmutated build passes. See [MEMO.md](../../MEMO.md).

## Cost

Each point is called twice with the same inputs, and the second call is reported. Its
placed figure is therefore:
- **warm**: the cache as the first identical call left it;
- **cold**: an empty cache.

Per-unit figures are slopes between two points, which removes the call cost:
- A: groups 1 → 90;
- B: records 1 → 388;
- C: marker at record 0 → 209;
- D: records 1 → 208.

The fixtures follow the requested shapes:
- **A**: 75 and 90 groups, with none, every tenth or all changed.
- **B**: 357 and 388 commands in the example layout (stride 32; offsets 8, 10, 6, 12; word
  3 always). Three key mixes:
  - none changed;
  - typical: the two key bytes change in 12% of records and the 16-bit key in 50%;
  - all changed.
- **C**: 1,024 records with limits 0x8000 and 0xFF00, and the marker at 0, at 209 or absent.
- **D**: 208 records, 66%, 100% or 0% equal.

Each kernel declares its buffers to the placement model:
- A: the shadow, mask and flags are output.
- B: the staged records and the descriptor are tables (high work RAM), the memo is input (low
  work RAM) and the destination is output. In `outVdp1` and `outVdp2` the destination is VDP1
  VRAM (45 a write) or VDP2 (40). The `typicalMemoHigh` rows declare the memo as tables, to
  show it in high work RAM.
- C: the snapshot is output.
- D: the runs are output.

`run.sh` adds the placed cycles of each point from `MODEL_PLACEMENT_LOG`. For each point it
requires the log's CPU-only cycles to equal the capture's.

## Reproduce

CI builds the ELF with the Yaul project's pinned toolchain image and runs the model on the Linux
runner through [`ci.sh`](ci.sh) (`build` and `model` phases, run by `saturn/benchmarks/ci/suites.sh`).
By hand:

```sh
$CC -m2 -mb -O2 -ffreestanding -std=c11 -Wall -Wextra -Werror -DSH2Z_REQUIRE_SH2=1 \
    -Isaturn/include -c saturn/benchmarks/memo-model/bridge.c -o memo-bridge.o
$CC -m2 -mb -nostdlib -nostartfiles -Wl,-T,saturn/benchmarks/integer-model/model.ld -Wl,-e,0 \
    -o memo-model.elf memo-bridge.o libsh2zam.a -lgcc     # libsh2zam.a: saturn/src/*.c, -O2 -m2 -mb
MODEL_PLACEMENT_LOG=$PWD/memo-placement.jsonl bash saturn/benchmarks/memo-model/run.sh memo-model.elf results.json
```

[`results.json`](results.json) is the committed capture, from
[CI run 44231934774](https://github.com/lamb2k/SH2ZAM/actions/runs/44231934774) on `d401348`. It
records the source commit, the ELF SHA-256, the toolchain, the Yabause commit and the placement
model's regions. Every CI run
uploads a fresh capture in the `sh2-compile-evidence` artifact.

## Not covered

- Console timing. The region costs are cited hardware notes, and the calibration image is
  unrun.
- The second CPU, DMA and bus arbitration.
- SDRAM refresh.
- Interrupts.
- Load-use interlocks in the CPU-only count.
