# Validation record — 2026-11-20 (effect pool, effect-pool-1)

`sh2zam/effect_pool_experimental.h` ([guide](../EFFECT_POOL.md)) keeps particle rings, steps
them per tick, projects them as billboards and writes VDP1 commands. It is integer only and
promises identical host and SH-2 bits. **No console run was done in this repository.** Evidence
by tier:

- **HOST:** `effect_pool_exact` passes 454,070 checks under GCC and Clang Release with
  `SH2Z_STRICT=ON` and under Clang ASan/UBSan:
  - the step against an independent double-precision transcription of the contract on the pool's
    own Q16 constants (growth 1.05, damping 0.98, an acceleration), random spawns over 4,000 ticks
    for capacities 1, 8, 32 and 128: the live set and every timer match exactly; the worst
    position, velocity or scale error is 33 Q16 LSB against the stated bound of 64 (half an LSB of
    velocity rounding a tick, summed over a 15-tick life);
  - the ring on a scripted run: refusal when full, the oldest ending first, `count` newest live,
    `clear`, and the refused capacities and lifetimes;
  - 200,000 projections from 200 random cameras against the double transcription: equal in
    every visibility decision and bit-exact in position, depth and pixel size (about 40,000 on
    screen);
  - 200,000 spots (`sh2z_ep_spot`, one division) against `sh2z_ep_project`: within one pixel,
    the same visibility decisions except at the guard edge (one pixel from it), the depth equal,
    sizes within one pixel; `sh2z_ep_depth` equal to the oracle's depth;
  - every emitted command decoded word by word (diamond corner order, sizes, addresses, Gouraud
    table, line words), and each refusal writing nothing;
  - the spin of 100,000 random bases within 1 Q14 LSB of the exact rotation;
  - the far-to-near order of 2,000 random sets of up to 256 items against a stable reference.

  Controls: the oracle with one rule changed must disagree with the library, and does, for each of
  growth 1.02, no damping, the acceleration's sign, position before velocity, and life 15; a
  mirrored projection disagrees on every off-centre point, a spot without its vertical ratio is
  off by more than a pixel on most points, and a reversed spin on most rows.
- **SH-2 build:** the source compiles with the CI flags (`-m2 -mb -O2 -Werror`, plus
  `-ffunction-sections -fdata-sections`). The object is 3,092 bytes of text and no data. It needs
  `sh2z_divu_div64` and `sh2z_sin12_quarter_u30`; `nm -u` lists no `memcpy`, `memset` or `memmove`.
- **Consumer:** a racing engine's race effects, run on an emulator (EMULATOR tier, modelled
  timing).
- **Not done:** an instruction-model capture, console execution and any frame measurement in this
  repository.
