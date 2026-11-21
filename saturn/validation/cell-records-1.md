# Validation record — 2026-11-21 (delta coded sprite records, cell-records-1)

`sh2zam/cell_records_experimental.h` ([guide](../CELL_RECORDS.md)) expands delta coded multi tile sprite records into
cell list entries: attribute rules (keep, replace, add to the low byte), a mirrored or plain running position, byte or
word offset pairs, and a masked and flagged position word. Integer only. **No console run was done in this repository.**
Evidence by tier:

- **HOST:** `cell_records_exact` passes under GCC and Clang Release with `SH2Z_STRICT=ON` and under ASan/UBSan. It
  compares all 12 forms, list lengths 0 to 72, both record alignments and random and edge parameters, with an
  independent per-cell oracle (`tests/cell_records_oracle.h`): every entry byte, the running position, guard bytes around
  the output, trusted and checked entries, and two calls in a row against one call over the whole list. The argument
  contract (nulls, odd addresses, count above the limit, unknown or contradictory form bits, overlaps) leaves every
  output untouched.
- **Mutation controls (10 of 10 caught).** Each plants one bug in the oracle, and the library must disagree: the flip
  bit moved, the width read from the wrong attribute bits, the mirrored x moving the wrong way, the mirror bias dropped,
  the added byte carrying out of its byte, the replace rule applied in the wrong order, byte offsets read unsigned, the
  position OR applied before the mask, the mirrored dy subtracted, and the running x not carried to the next call.
- **MODELLED:** the SH-2 instruction model ([README](../benchmarks/cell-records-model/README.md)) runs the compiled
  object code of the trusted and checked entries and of an independent plain C control over 10,800 random lists
  (215,604 cells, all 12 forms): every entry word and the running position equal the oracle and the host build, and
  nothing is written past the entries. The cell cost is 18 to 35 cycles for the library and 47 to 74 for the control
  (the control costs 1.5 to 3.2 times as much); the checked entry adds about 155 cycles a call.
- **SH-2 build:** compiled C with the CI flags; the object needs no symbol (no libc, no libgcc helper).
- **Not done:** console execution, any frame measurement and a hand-written assembly path (the loops are compiled C).
