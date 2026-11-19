# VDP1 double banks (vdp1-banks-1, experimental)

`sh2zam/vdp1_banks_experimental.h` is bookkeeping for building frame N+1 while
VDP1 still draws frame N. It decides and records. The caller reads the VDP1
state and does every wait, transfer and upload.

## Why two banks

VDP1 reads its command list and the tables the list points at (Gouraud tables,
colour lookup tables) for the whole draw. A program with one copy of those
tables cannot start the next frame until the draw ends. With two banks, each
frame's build writes one bank, submission hands that bank to VDP1, and the next
build takes the other one.

## Contract

- **A. Bank state** (`sh2z_vdp1_banks_t`):
  - `sh2z_vdp1_banks_begin` returns true when this build must wait for the
    previous draw first: the caller asked for it, or the previous frame did not
    fill every banked table.
  - It counts frames that start while VDP1 is busy (`overlap`) and, of those,
    frames that would write the bank in flight (`conflicts`).
  - `sh2z_vdp1_banks_submitted` records the frame and alternates the bank.
  - `hold` keeps the bank at 0. It is a negative control and must raise
    conflicts.
- **B. Guards:** `sh2z_vdp1_guard_fill` and `sh2z_vdp1_guard_check` write and
  check fixed words around a region. Word `i` is `seed ^ (i << shift)`.
- **C. Carving:** `sh2z_vdp1_carve` lays out guard, items and guard from a base
  address with per-item power-of-two alignment. It writes nothing when the
  layout does not fit or an argument is invalid.
- **D. Upload queue** (`sh2z_vdp1_queue_t`): up to two entries, oldest first,
  for tables uploaded at the end of a draw. A bank is free to fill only when it
  is not queued.
- **E. Staleness** (`sh2z_vdp1_stale64_t`): up to 64 entries per bank, for
  tables uploaded only when they change. A change marks the entry stale in both
  banks; taking it clears one bank.
- **F. Jump-skip record:** `sh2z_vdp1_jump_skip` writes a command record that
  skips to a command slot, so a block built elsewhere can stay at a fixed slot.

Portable C. No hardware access, no libc, no allocation, no float.

## Use

1. After the display sync and the previous list transfer, call
   `sh2z_vdp1_banks_begin`. Wait for the previous draw only if it returns true.
2. Build the frame into bank `sh2z_vdp1_banks_bank`.
3. Submit the list, then call `sh2z_vdp1_banks_submitted`. Pass `filled` as
   false when the frame did not write every banked table; the next build is
   then serial.
4. Check the guards every frame. `conflicts` must stay at 0.

## Tests (`tests/test_vdp1_banks.c`)

- A frame model tracks which bank VDP1 is reading and checks that no build
  writes it, over random busy, serial and unfilled frames.
- Negative controls: a held bank must conflict, and a changed guard bit must
  fail the guard check.
- Argument contracts for carving, the queue, staleness and the jump-skip
  record, including that a rejected call writes nothing.
- GCC and Clang, with ASan, UBSan, `-Wconversion` and `-Werror -Wpedantic`.

## Cost

- **SH-2 object:** 900 bytes (`-m2 -mb -O2`).
- Every call is constant time except the guard fill and check (one pass over
  the guard words) and carving (one pass over the items).
