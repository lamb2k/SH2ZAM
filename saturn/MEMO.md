# Block memo kernels (contract tag `memo-1`)

[`sh2zam/memo_experimental.h`](include/sh2zam/memo_experimental.h) gives four kernels for
work a Saturn program repeats every picture on data that mostly has not changed:

| Kernel | What it does |
| --- | --- |
| `sh2z_memo_sync16w`, `sh2z_memo_sync16l` and their `_batch` forms | Compare a 16-word (32-byte) or 16-longword (64-byte) block with its shadow. When the block has changed, copy it into the shadow and report it, one block at a time or as a bit mask over N blocks. |
| `sh2z_memo_put4`, `sh2z_memo_put4_one` | Write a record's 4 words to a slow 16-bit destination, each only when the key bits that select it have changed since the destination was last written. |
| `sh2z_memo_copy8_until` | Copy 8-byte records up to and including the first end marker, and return the record count. |
| `sh2z_memo_diff8_runs` | Diff two tables of 8-byte records and return the runs (start, count) that differ. |

A Saturn port asked for these shapes. The kernels know nothing of its formats:
- a palette is just a list of blocks;
- a command table is a destination with a stride and four word offsets;
- the end marker is two limits.

**Evidence tiers: HOST and MODELLED only.** No emulator or console run has been done.

## Contract

Every kernel is **exact**: its outputs are a pure function of its inputs, with no
approximation, so there is no error bound to state. The SH-2 kernels and the portable C give
the same results and the same destination writes. None of the kernels uses floating point,
the divider, global state, libc or allocation.

- **Blocks** (`sync16w`, `sync16l`):
  - A block has changed iff any of its bytes differs from the shadow. Only changed blocks are
    copied. After the call every shadow block equals its source.
  - The batch forms return the changed count. Bit `g % 32` of `changed[g / 32]` is set iff
    block `g` changed. Exactly `(groups + 31) / 32` mask words are written, with zero bits
    past `groups`.
  - Every word of a block is compared, so keep unused words equal (for example zero) in both
    the source and the shadow.
  - Blocks must be 2-byte aligned. The SH-2 kernel runs when both pointers are 4-byte
    aligned; otherwise the portable path runs, with the same results.
- **Keyed writes** (`put4`):
  - Record `i` has a 32-bit key and 4 words. Let `d = memo[i] ^ key`. Word `k` is written
    to `dst + i * stride + offset[k]` iff `d & gate[k]` is non-zero or bit `k` of `always`
    is set.
  - Writes go in word order, then record order, each as a single 16-bit volatile store. The
    destination is never read.
  - Afterwards `memo[i] = key`. The memo is written only when `d != 0`.
  - The function returns the number of destination writes.
  - The SH-2 kernel handles `always` = 0, 0x8, 0xC, 0xE and 0xF, where the always-written
    words come last; any other mask takes the portable path. Reorder your words and offsets
    to put the always-written ones last.
- **Copy until a marker** (`copy8_until`):
  - Record `i` is a marker iff word 1 ≥ `w1_limit` or word 3 ≥ `w3_limit`, compared
    unsigned. A limit of 0x10000 or more turns its test off.
  - Records up to and including the first marker are copied. The return value `m` is the
    marker's index, or `max_records` if there is none.
  - Nothing after the marker record is read.
- **Run diff** (`diff8_runs`):
  - A run is a maximal range of consecutive records that differ. Runs are reported in
    order.
  - The return value is the total number of runs. Only the first `max_runs` are stored.
  - `n` must be at most 65,535.

The header gives the full aliasing and alignment rules.

## How to call them

**Palette groups (A).**
- Keep a shadow of the groups, 4-byte aligned in high work RAM.
- Each picture, build the source groups and call:
  ```c
  uint32_t mask[(90 + 31) / 32];
  uint32_t n = sh2z_memo_sync16w_batch(shadow, src, groups, mask);
  ```
- If `n` is non-zero, send the shadow group of every set bit to colour RAM (by CPU or DMA).
  The colour RAM write is not part of the kernel.
- Keep the unused 16th word equal in the source and the shadow.
- The batch form saves about 14 cycles a group over calling `sh2z_memo_sync16w` once per
  group.

**Command words (B).**
- Stage one `sh2z_memo_rec4_t` per command in high work RAM:
  - `key = byte_a << 24 | byte_b << 16 | key16`;
  - the four words in the order: words 0 and 1, word 2, then the always-written word.
- Describe the layout once:
  ```c
  static const sh2z_memo_gate4_t gate = {
      {0xFFFF0000u, 0xFFFF0000u, 0x0000FFFFu, 0u},  /* gates of words 0..3 */
      32u,                                          /* bytes per command */
      {8u, 10u, 6u, 12u},                           /* each word's byte offset in a command */
      0x8u };                                       /* word 3 always */
  writes = sh2z_memo_put4(vdp1_commands, memo, records, n, &gate);
  ```
- Keep one memo per destination buffer: with double-buffered command tables, the memo of a
  buffer describes what that buffer holds.
- Before a buffer's first use, fill its memo with keys that cannot match, or make one call
  with `always = 0xF`.
- Without a staging array, `sh2z_memo_put4_one` does the same for one record inside your own
  loop. It is slower (see the table).
- Placing the memo in high work RAM rather than low saves about 14 cycles a command (see the
  `memo in high work RAM` rows).

**Copy until an end marker (C).**
```c
uint32_t m = sh2z_memo_copy8_until(snapshot, table, 1024, 0x8000, 0xFF00);
/* m records before the marker; the snapshot also holds the marker record when m < 1024 */
```

**Run diff (D).**
```c
uint32_t total = sh2z_memo_diff8_runs(current, previous, 208, runs, 64);
/* send runs[0 .. min(total, 64)) */
```

## Evidence

### HOST

`ctest -R memo_exact` runs `tests/test_memo.c` against the independent oracle
[`tests/memo_oracle.h`](tests/memo_oracle.h):
- The oracle compares bytes, builds masks from a flag array, lists keyed writes in order and
  finds runs from flag transitions.
- The suite runs under GCC and Clang with `-Werror -Wpedantic`, and under ASan/UBSan.
- It makes **1,011,260 checks**:
  - **Blocks, 9,335 cases.** A single-bit difference at every byte and bit of both block
    sizes. Every group count from 0 to 130 (across the mask-word boundaries 31/32/33 and
    63/64/65) in five difference patterns. Group counts up to 1,100. 2-byte-aligned blocks.
    A shadow equal to its source. Each case checks canaries around every output and a
    second, steady call.
  - **Keyed writes, 3,023 cases with 492,952 destination writes.** Random descriptors
    (gates 0, all, high half, low half, one bit, one byte, random), strides 0 to 64, any
    `always` mask, change rates from 0 to 100%, both the batch and the one-record form. The
    destination is checked against two different fill patterns, so a write of a value equal
    to the fill cannot hide.
  - **Copy, 18,735 cases.** Every `max_records` from 0 to 40 with the marker at every
    position, at the end and absent; eight limit pairs including 0 and 0x10000; both
    alignments; random sizes up to 1,100. Under ASan, the source ends right after the
    marker in 300 cases.
  - **Diff, 6,750 cases.** Every equal/differ pattern of 12 records, sizes 0 to 64 and up
    to 65,535, all equal, all different, and `max_runs` truncation with a canary past the
    stored runs.
- **12 of 12 negative controls caught.** Each control is the oracle with one planted bug,
  which the comparison with the library must detect:
  - compare only 15 words;
  - skip copying the last longword;
  - shift the mask bit;
  - drop the last mask word;
  - ignore `always`;
  - treat any key change as a change;
  - never update the memo;
  - write the words in reverse order;
  - leave the marker uncopied;
  - use `>` in place of `≥`;
  - return the stored run count instead of the total;
  - split long runs.

### MODELLED

[`benchmarks/memo-model`](benchmarks/memo-model/README.md) runs the compiled SH-2 objects in
the pinned Yabause interpreter. Before any timing, every output is checked against the same
oracle:
- 3,000 block batches (235,017 blocks) and 3,000 single blocks;
- 3,000 keyed cases, with 547,013 destination writes logged and matched in order;
- 5,192 copy cases (873,444 records);
- 3,000 diff cases (102,024 runs).

The checks also cover:
- the unaligned paths;
- that every destination write is one 16-bit store and the destination is never read;
- that no source byte after the marker record is read;
- guards after every output.

Every timed call is verified the same way. **11 of 11 planted bugs in the SH-2 kernels**
stopped the harness:
- a mask bit restarted at 2;
- a 64-byte block copied short;
- the wrong single-block flag;
- the last longword pair skipped;
- `>` for `≥` in the marker test;
- half a marker record copied;
- the memo never stored;
- the wrong word written;
- the wrong `always` dispatch;
- a run stored past `max_runs`;
- a final run one record long.

Cycles are given two ways:
- **CPU-only:** the interpreter's flat-RAM count.
- **Placed:** the same calls through the `model-placement-1` cache and bus model
  ([placement model](benchmarks/placement-model/README.md)). Each figure is warm (the
  previous identical call left the cache) or cold (empty cache).

Its region costs are cited hardware notes, not this project's measurements:
- high work RAM: 7 a line fill;
- low work RAM: 56 a line fill;
- VDP1 VRAM: 45 a write;
- VDP2: 40 a write.

The tables come from the committed capture
[`benchmarks/memo-model/results.json`](benchmarks/memo-model/results.json):
[SH-2 cross compile run 44231934774](https://github.com/lamb2k/SH2ZAM/actions/runs/44231934774),
source commit `d401348`. It is identical to a local run with the same pinned Yaul GCC
(the same ELF SHA-256).

<!-- tables -->
**A. Block compare-and-copy**, cycles per group (slope from 1 to 90 groups), and whole calls at 75 and 90 groups. Placed = `best` (all in high work RAM), warm / cold.

| Groups changed | Entry | CPU-only per group | Placed per group | 75 groups: CPU / placed cold | 90 groups: CPU / placed cold |
| --- | --- | ---: | ---: | ---: | ---: |
| none (steady state) | `sh2z_memo_sync16w_batch` | 40.18 | 67.45 / 68.28 | 3,047 / 5,249 | 3,647 / 6,269 |
| none (steady state) | `sh2z_memo_sync16w`, one call per group | 54 | 80.38 / 82 | 4,075 / 6,309 | 4,885 / 7,539 |
| none (steady state) | plain C: 7 longwords + 1 word, a call per group | 83 | 110.48 / 111 | 6,250 / 8,454 | 7,495 / 10,119 |
| none (steady state) | `sh2z_memo_sync16l_batch` (64-byte groups) | 72.18 | 129.61 / 128.47 | 5,442 / 9,789 | 6,522 / 11,709 |
| every tenth | `sh2z_memo_sync16w_batch` | 42.40 | 71.18 / 72.26 | 3,223 / 5,565 | 3,845 / 6,623 |
| every tenth | `sh2z_memo_sync16w`, one call per group | 55.42 | 83.62 / 85.31 | 4,187 / 6,572 | 5,011 / 7,834 |
| every tenth | plain C: 7 longwords + 1 word, a call per group | 84.75 | 111.92 / 112.44 | 6,345 / 8,514 | 7,651 / 10,247 |
| every tenth | `sh2z_memo_sync16l_batch` (64-byte groups) | 76.02 | 136.66 / 135.96 | 5,746 / 10,383 | 6,864 / 12,375 |
| all | `sh2z_memo_sync16w_batch` | 62.18 | 105.34 / 106.17 | 4,697 / 8,112 | 5,627 / 9,702 |
| all | `sh2z_memo_sync16w`, one call per group | 68 | 112.38 / 114 | 5,125 / 8,716 | 6,145 / 10,426 |
| all | plain C: 7 longwords + 1 word, a call per group | 95.85 | 115.87 / 119.84 | 7,467 / 9,406 | 8,660 / 10,928 |
| all | `sh2z_memo_sync16l_batch` (64-byte groups) | 110.18 | 201.38 / 200.25 | 8,292 / 15,208 | 9,942 / 18,208 |

**B. Keyed write-through**, the example layout (stride 32, offsets 8, 10, 6, 12, word 3 always). Cycles per record (slope from 1 to 388 records) and whole calls at 357 and 388 records. Placed: `outVdp1` (destination VDP1 VRAM, 45 a write) and `outVdp2` (VDP2, 40), memo in low work RAM, staged records in high work RAM; cold.

| Keys changed | Entry | CPU-only per record | VDP1 per record | VDP2 per record | 357 records: VDP1 | 388 records: VDP1 | 388 records: VDP2 |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| none | `sh2z_memo_put4` | 22 | 69.15 | 64.15 | 25,048 | 27,142 | 25,202 |
| none | `sh2z_memo_put4_one` in the caller's loop | 51 | 93.36 | 89.61 | 33,749 | 36,595 | 35,140 |
| none | plain C memo (fixed layout) | 36 | 72.83 | 67.83 | 26,232 | 28,429 | 26,489 |
| none | plain C, every word written (no memo) | 13 | 186.49 | 166.49 | 66,594 | 72,373 | 64,618 |
| typical: key bytes 12%, 16-bit key 50% | `sh2z_memo_put4` | 31.71 | 110.86 | 102.03 | 39,709 | 43,285 | 39,860 |
| typical: key bytes 12%, 16-bit key 50% | `sh2z_memo_put4_one` in the caller's loop | 54.14 | 135.31 | 126.88 | 48,401 | 52,828 | 49,563 |
| typical: key bytes 12%, 16-bit key 50% | plain C memo (fixed layout) | 38.01 | 111.16 | 102.33 | 39,772 | 43,264 | 39,839 |
| all | `sh2z_memo_put4` | 54 | 216.91 | 196.91 | 77,820 | 84,498 | 76,738 |
| all | `sh2z_memo_put4_one` in the caller's loop | 63 | 230.87 | 210.87 | 82,836 | 89,951 | 82,191 |
| all | plain C memo (fixed layout) | 41.02 | 219.11 | 199.11 | 78,441 | 85,179 | 77,419 |
| typical, memo in high work RAM | `sh2z_memo_put4` | 31.71 | 96.27 | 87.44 | 34,450 | 37,590 | 34,165 |
| typical, memo in high work RAM | `sh2z_memo_put4_one` in the caller's loop | 54.14 | 121.08 | 112.66 | 43,299 | 47,274 | 44,009 |
| typical, memo in high work RAM | plain C memo (fixed layout) | 38.01 | 97.12 | 88.28 | 34,696 | 37,779 | 34,354 |

**C. Copy until an end marker**, 1,024-record table, limits 0x8000 and 0xFF00. Per record = slope from a marker at 0 to a marker at 209. Placed = `best`, warm / cold.

| Entry | CPU-only per record | Placed per record | Marker at 209: CPU / placed warm / cold | No marker (1,024): CPU / placed warm / cold |
| --- | ---: | ---: | ---: | ---: |
| `sh2z_memo_copy8_until` | 12 | 12 / 15.53 | 2,563 / 2,564 / 3,392 | 12,332 / 15,987 / 16,031 |
| plain C word copy with the tests | 16 | 16 / 19.48 | 3,362 / 3,362 / 4,132 | 16,396 / 19,994 / 20,022 |
| plain C longword copy, no test | 12 | 15.52 / 15.50 | n/a | 12,304 / 15,902 / 15,916 |

**D. Record-run diff**, 208 records. Per record = slope from 1 to 208 records. Placed = `best`, warm / cold.

| Records equal | Entry | CPU-only per record | Placed per record | 208 records: CPU / placed warm / cold |
| --- | --- | ---: | ---: | ---: |
| 66% | `sh2z_memo_diff8_runs` | 14.04 | 15.37 / 21.21 | 2,956 / 3,241 / 4,544 |
| 66% | its portable C path | 46.14 | 47.41 / 53.86 | 9,630 / 9,902 / 11,369 |
| all | `sh2z_memo_diff8_runs` | 12 | 13.20 / 19.03 | 2,534 / 2,791 / 4,094 |
| all | its portable C path | 35 | 35.69 / 42 | 7,323 / 7,475 / 8,914 |
| none | `sh2z_memo_diff8_runs` | 12.46 | 13.86 / 19.53 | 2,642 / 2,941 / 4,223 |
| none | its portable C path | 28 | 28.69 / 35 | 5,914 / 6,069 / 7,529 |
<!-- end tables -->

<!-- targets -->
### Against the figures quoted for today's code

The port quoted these figures from its own cost model (CPU plus memory). This project has not
measured them, so compare them with the placed columns.

| Shape | Today (quoted) | `memo-1`, placed (cold) | CPU-only |
| --- | ---: | ---: | ---: |
| A: per 16-word group, steady state | about 107 | **68.28** (67.45 warm) | 40.18 |
| A: 90 groups, steady state | about 9,630 (90 × 107) | **6,269** | 3,647 |
| B: 357 to 388 commands, typical mix, VDP1 | 47,000 to 50,000 | **39,709 to 43,285** (34,450 to 37,590 with the memo in high work RAM) | 11,320 to 12,370 |
| C: copy with the stop at about 209 records | 6,467 | **3,392** (2,564 warm) | 2,563 |
| C: full 1,024-record copy | 17,940 | **16,031** (no marker) | 12,332 |

What the model says:
- **A meets its target.** The batch costs 40 CPU cycles a group in steady state. Placed, it
  is 68 a group, against the 107 quoted. About 28 of those cycles are the four line fills of
  the two 32-byte blocks, which every compare must pay. The plain C control (a call per
  group, 7 longwords and 1 word) comes out at 111 placed, close to the 107 quoted, which
  suggests the two cost models agree.
- **B is bound by the destination.** Each VDP1 write takes 45 bus cycles, and a low work
  RAM memo line takes 56 to fill. `sh2z_memo_put4` needs 22 to 32 CPU cycles a record, and
  it stores the memo before the record's slow writes so the next record's CPU work overlaps
  the last write. Placed, it is 69 a record in steady state against 73 for the fixed-layout
  plain C, and 111 against 111 in the typical mix. The kernel saves CPU time
  (14,772 → 12,370 cycles for 388 commands in the typical mix) but not bus time.
  - The larger gain is placement: with the memo in high work RAM the typical mix costs 96 a
    record (37,590 for 388 commands).
  - The typical mix is an assumption. The two key bytes change in 12% of records (the port
    quotes 7 to 17%) and the 16-bit key in 50% (no figure was quoted). The steady and
    all-changed rows bracket it.
- **C is about half of today's figure** with the stop at 209 records: 3,392 cycles cold and
  2,564 warm. Each longword is stored as soon as it is loaded, so no store waits behind
  another. A full copy adds the 512 line fills of the 8 KB table to 12 CPU cycles a record.
- **D** costs 12 to 14 CPU cycles a record. Its portable C path, which 2-byte-aligned tables
  take, costs 28 to 46.
<!-- end targets -->

## Not covered

- Console or emulator timing. The placed figures use cited region costs, and the
  calibration image is unrun.
- DMA, the second CPU, and interrupts during a call.
- The colour RAM write after a changed group. It belongs to the caller.
