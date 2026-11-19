# Validation record — 2026-11-19 (sound events and CD audio, snd-1)

`sh2zam/snd_experimental.h` ([guide](../SND.md)) adds an epoch-stamped sound event queue, SCSP
slot playback with priority stealing, pitch and level helpers, and a CD audio player (play once,
loop forever, fade and pause) over caller-named register bases. No existing contract changed.
**No console run was done.** Evidence by tier:

- **HOST:** `test_snd` passes under GCC and Clang with `-Wall -Wextra -Wpedantic -Wconversion
  -Werror`, and under ASan/UBSan (3,911,593 checks):
  - **Queue:** a worked example covering the closed start, another epoch, a post after the close,
    exactly-once delivery and a full queue. 3,000 random scripts of 400 operations match a
    reference queue.
  - **Voices:** 2,000 random scripts of 600 operations match a reference allocator. Every play's
    slot registers are decoded back and checked against the clip, level and pan. Out-of-range
    slot ranges are refused without writes.
  - **Level and pitch:** every level 1..255 gives a TL within 0.91 steps of the exact
    attenuation. 20,000 random pitch ratios (0.25x to 4x) give a rate within one FNS step of the
    exact one, against double precision.
  - **CD player:** 500 runs against a model CD block with random busy times: the play and loop
    commands are exact, the fade never rises and reaches silence by its last frame, the drive is
    paused and quiet after it, and a command is never written while the block is busy.
  - **Negative controls:** eight, one per rule: post after close, another epoch, drop at close,
    delivery order, stealing a higher priority, youngest among equals, free slots not first,
    writing without CMOK. Each must fail, and each does.
- **SH-2 cross build:** `sh2eb-elf-gcc -m2 -O2 -ffreestanding` with the same warnings: 2,668 bytes
  of text, no data; undefined only libgcc shift helpers.
- **MODELLED:** none; the component runs a few register writes per frame.
- **Not done:** console execution. A caller's emulator capture is that caller's evidence.
