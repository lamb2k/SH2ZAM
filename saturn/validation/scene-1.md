# Validation record — 2026-11-19 (scene sequencing, scene-1)

`sh2zam/scene_experimental.h` ([guide](../SCENE.md)) adds a scene sequencer with fades, ranked
requests, retirement only on caller confirmation, preload chunks, bump arenas and press edges with
consumption and auto-repeat. No existing contract changed. **No console or emulator run was
done.** Evidence by tier:

- **HOST:** `scene_exact` passes under GCC and Clang with `-Wall -Wextra -Wpedantic -Wconversion
  -Werror`, and under ASan/UBSan (3,249,067 checks):
  - **Arena:** a worked example covering alignment, bad alignments, overflow, the exact fit and a
    zero-byte request. 2,000 random runs of 64 operations match a reference allocator, and a
    range ending at the top of the address space refuses without wrapping.
  - **Sequencer, worked example:** the exact fade levels in and out, an equal rank refused,
    preempt refused below its rank, RETIRE held for 50 frames without confirmation, a
    confirmation before RETIRE not counted, and a preempting redirect.
  - **Sequencer, random scripts:** 4,000 scripts of 600 frames, with random table sizes, fade
    lengths, retire minimums, preempt ranks and preload lengths, and enters that fail. Each
    script makes random requests (several per frame at times) and random confirmations. The
    callback trace, fade level, phase and current scene match an independent timeline model on
    every frame.
  - **Negative controls:** five, one per rule. Each is the same comparison against a reference
    with that one rule broken (retire waits for confirmation, preempt rank, higher rank
    replaces, the fade turns around, preload before enter); each must fail, and each does.
  - **Press edges:** a worked example (one edge across repeated steps, latching and release, and
    the repeat timeline: delay 20, then every 6), plus 3,000 random runs of 200 steps matching
    a reference model.
- **MODELLED:** none; the component is bookkeeping run once per frame.
- **Not done:** console or emulator execution.
