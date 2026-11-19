# Validation record — 2026-11-19 (VDP1 double-bank bookkeeping, vdp1-banks-1)

`sh2zam/vdp1_banks_experimental.h` ([guide](../VDP1_BANKS.md)) is bookkeeping for building
frame N+1 while VDP1 still draws frame N: bank state with serial, overlap and conflict counters,
guard words, carving a guarded region, an in-order queue for end-of-draw uploads, per-bank
staleness and a jump-skip command record. It is portable C with no hardware access.
**No console or emulator run was done in this repository.** Evidence by tier:

- **HOST:** `vdp1_banks_exact` passes 354,224 checks under GCC and Clang Release with
  `SH2Z_STRICT=ON` and under ASan/UBSan. A frame model tracks the bank VDP1 is reading over
  random busy, serial and unfilled frames, and no build writes it. Negative controls: a held
  bank raises conflicts, and a changed guard bit fails the guard check. Rejected calls write
  nothing.
- **SH-2 build:** the source compiles with the CI flags and passes
  `sh2z_require_per_symbol_sections`. The object is 900 bytes (`-m2 -mb -O2`).
- **Not done:** an instruction-model capture (the functions are constant-time bookkeeping,
  not kernels), console or emulator execution, and any measurement of a frame.
