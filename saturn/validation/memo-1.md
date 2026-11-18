# Validation record — 2026-11-17 (block memo kernels, memo-1)

`sh2zam/memo_experimental.h` ([guide](../MEMO.md)) adds:
- block compare-and-copy with a changed flag, over 16 words or 16 longwords, singly or as a
  batch with a changed-block bit mask;
- keyed write-through of 4 words a record to a slow 16-bit destination, gated by a key
  descriptor;
- a copy of 8-byte records up to and including the first end marker;
- a record-run diff between two tables.

All four are exact, with hand-written SH-2 kernels and portable C paths that give the same
results and destination writes. **No console or emulator run was done.** Evidence by tier:

- **HOST:** `memo_exact` passes under GCC and Clang Release with `SH2Z_STRICT=ON` and under
  ASan/UBSan, against the independent oracle `tests/memo_oracle.h` (1,011,260 checks):
  - single-bit differences at every byte and bit of both block sizes;
  - every group count from 0 to 130;
  - random gate descriptors, with the destination checked under two fill patterns;
  - markers at every position, including sources that end right after the marker;
  - every equal/differ pattern of 12 records;
  - 12 of 12 negative controls caught.
- **MODELLED:** the compiled SH-2 object code matches the oracle before any timing:
  - 6,000 block calls (235,017 batched blocks);
  - 3,000 keyed cases, with 547,013 destination writes logged in order, each a 16-bit store
    and none a read;
  - 5,192 copy cases, with no read after the marker;
  - 3,000 diff cases.

  11 of 11 planted bugs in the SH-2 kernels stop the harness. Cycles are CPU-only and placed
  through `model-placement-1`
  ([results](../benchmarks/memo-model/results.json), captured by
  [CI run 44247175665](https://github.com/lamb2k/SH2ZAM/actions/runs/44247175665)):

  | Shape | Today (the port's own model) | Placed (cold) | CPU-only |
  | --- | ---: | ---: | ---: |
  | A: 16-word group, steady state | about 107 a group | 68.28 a group | 40.18 a group |
  | B: 388 commands, typical mix, VDP1 | about 47,000 to 50,000 (357 to 388 commands) | 43,285 (37,590 with the memo in high work RAM) | 12,370 |
  | C: copy stopping at record 209 | 6,467 | 3,392 | 2,563 |
  | D: 208 records, 66% equal | not quoted | 4,544 | 2,956 |

  B is bound by the 45-cycle destination writes and ties a fixed-layout plain C memo there.
  Cycle numbers are modelled, not Saturn timings.
- **Byte identity:** every existing SH-2 library object is byte-identical to main under the
  CI flags; no existing `results.json` changed.
- **Not done:** console or emulator execution, DMA and second-CPU contention, interrupts.
