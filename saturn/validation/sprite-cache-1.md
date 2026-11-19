# Validation record — 2026-11-19 (sprite slot cache and cell4z decoder, sprite-cache-1)

`sh2zam/sprite_cache_experimental.h` ([guide](../SPRITE_CACHE.md)) adds a fixed VDP1 slot
pool with least-recently-used replacement under live-frame protection, per-cell flip aliases
and a vlc-2 based 4-bit cell decoder (cell4z). No existing contract changed.
**No console or emulator run was done.** Evidence by tier:

- **HOST:** `sprite_cache_exact` passes under GCC and Clang with `-Wall -Wextra -Werror`, and
  under ASan/UBSan (3,404,860 checks):
  - layout: alignment, overflow, too many slots and zero sizes;
  - a worked two-slot scene: claim, pending, refusal while filling, live protection over two
    frames, prefetch protection, abandon and clear, and the four counters;
  - every result and slot of 240 random scene runs (120 distinct) and 120 racer-like scenes
    (30 sprites swapping between near, far and player sets, 400 to 600 frames, live 1, 2
    and 3) equals an independent reference model of the replacement rule; a replaced slot
    is never in the list being built or, with live 2 or more, in the previous frame's list;
  - determinism: the same scene twice gives the same result hash;
  - six negative controls, one per rule (key needs size, smallest fit, free first, least
    recently used, live protection, filling kept): the same comparison against a reference
    with that rule broken must fail, and each does;
  - cell4z: 160 generated cells round-trip through a test-side encoder; each writes exactly
    `w * h / 4` words, `bits_used` equals the encoded length, a stream cut to 2 bytes is
    refused, and 640 random-byte streams decode at the very end of an exact-size heap
    allocation without a read outside it; a run past a row end and bad sizes are refused.
- **HOST (local data, not in the repository):** on a roster of 105 pre-rendered sprite sets
  (3,080 cells, 5,219,384 pixels) every cell decodes bit-exact at 1.98 ns per pixel. Bytes
  against raw 4-bit: 0.374 (60-cell near sets), 0.395 (player sets), 0.441 (8-cell far
  sets); plain vlc-2 with one table over the raw bytes gives 0.557.
- **MODELLED:** the `-O2` SH-2 code (`sh2eb-elf-gcc -m2 -mb`, 1,626 bytes of text for the
  whole component) runs in an SH7604 pipeline model with flat memory on 308 of those cells,
  every output checked against the source: 29.1 cycles per pixel for near cells (53,221
  cycles, 1.86 ms at 28.6 MHz per cell of 1,822 pixels on average, worst 85,845), 31.4 for
  far cells. Plain vlc-2 on raw bytes in the same model: 15.1 cycles per pixel. Cycle
  numbers are modelled, not Saturn timings.
- **Not done:** console or emulator execution; cartridge wait states; a multi-CPU run of the
  fill job.
