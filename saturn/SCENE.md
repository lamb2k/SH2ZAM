# Scene sequencing (scene-1)

`sh2zam/scene_experimental.h` sequences a fixed table of scenes (title, menus, loading, game,
results) with fades, ranked requests, confirmed retirement, preload chunks, scene arenas and
press edges that survive a transition. It's pure bookkeeping: the caller reads the pad, draws,
applies the fade (for example with a VDP2 colour offset) and knows when VDP1 has displayed what.

## Phases

```
ENTER (fade in) -> RUN -> LEAVE (fade out) -> RETIRE -> PRELOAD -> ENTER (next scene)
```

- `sh2z_scene_frame()` runs once per presented frame. It returns the fade level, from 0 (fully
  shown) to `SH2Z_SCENE_FADE_MAX` (black).
- `tick` runs while the scene is entering or running.
- `leave` runs once, when the fade out starts.
- A request that arrives while the scene is still fading in turns the fade around from the level
  it reached.
- **RETIRE** waits for two things:
  - the caller calls `sh2z_scene_retired()` to confirm that the last displayed list
    referencing the leaving scene has been replaced by one built after the fade out;
  - at least `retire_frames` frames have passed in RETIRE.

  Only then does `release` run and the scene's arenas reset. Elapsed frames alone are never
  taken as proof, and a confirmation given before RETIRE began doesn't count.
- **PRELOAD** calls the next scene's `preload` once per frame until it returns true, then
  `enter`. An `enter` that fails stops the sequencer (`SH2Z_SCENE_FAULT`).

## Requests

- `sh2z_scene_request(seq, target, rank)`. At most one request is pending, and a higher rank
  replaces it.
- After the fade out has begun, only `rank >= preempt_rank` can change the destination, for
  example a reset chord during a transition. The fade carries on without restarting, and a
  preload in progress starts again for the new destination.
- A request for the current scene restarts it.

## Press edges

- `sh2z_scene_input_update()` takes one 60 Hz step's buttons and reports presses.
- **Multiple steps per frame:** when a frame runs several steps from one pad reading, pass the
  same buttons each time and the press is reported on the first step only.
- **Consuming:** `sh2z_scene_input_consume()` latches every button held at that moment, so the
  press that confirmed one screen can't confirm the next, however long the transition between
  them.
- **Auto-repeat:** the `repeat_mask` buttons (for example the directions) repeat after
  `repeat_delay` steps, then every `repeat_rate` steps.

## Arenas

- `sh2z_arena_alloc()` is a bump allocator with power-of-two alignment (1..4096).
- A failed allocation changes nothing but the `refused` counter.
- `high` keeps the largest use across resets, so a caller can show that a scene uses the same
  memory on every visit.

## Example

```c
static const sh2z_scene_t scenes[] = {
    [TITLE] = { NULL, title_enter, title_tick, title_leave, title_release, title_arenas, 1 },
    [LOAD]  = { load_chunk, load_enter, load_tick, load_leave, load_release, NULL, 0 },
    /* ... */
};
sh2z_scene_seq_t seq;
sh2z_scene_init(&seq, scenes, COUNT, TITLE, 16u, 0u, RANK_RESET, &game);
for (;;) {
    if (reset_chord) sh2z_scene_request(&seq, TITLE, RANK_RESET);
    const uint32_t fade = sh2z_scene_frame(&seq);
    if (sh2z_scene_drawing(&seq)) draw(seq.current);
    set_colour_offset(fade);
    submit_and_wait();
    if (last_list_with_scene_commands_replaced()) sh2z_scene_retired(&seq);
}
```

Host-tested only. See [the validation record](validation/scene-1.md).
