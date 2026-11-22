# Validation record — 2026-11-22 (line field contact passes, less work per step, line-field-13)

`sh2zam/line_field_experimental.h` ([guide](../LINE_FOLLOWER.md)) adds no entry point and no parameter. The
results are the line-field-12 step's, bit for bit; only the work changes.

A step's contact phase (at most three passes) cost 40 to 47 us a touching pair and about 10 us a pair in reach on
the console's SH-2 clock when a pack bunches. Reading the compiled code showed where the time went: every pair
was evaluated by a call that took the sample of the line under both vehicles, a variable shift and a 92-byte
indexed brain, and the passes after the first looked at every listed pair again although only pushes change what
a pair does. The cuts, each exact:

- **Packed rows.** The step reads each row of the order once: the vehicle's distance, or -1 for a vehicle that
  does not take part. The scan, the list and the early exit all work from that array, which is refreshed after
  a retirement (the only thing that changes who takes part).
- **No neighbours in reach, no pairs.** Rows are sorted by distance along the line, so a pair in reach has a
  cyclic neighbour in reach. When no two neighbouring rows are within the contact reach, the passes do nothing
  (the common step, and every step of a field that is not bunched).
- **First pass lists, later passes skip.** The first pass lists the pairs in reach (up to 192) from the packed
  rows. A pair whose vehicles are not within the lateral reach changes nothing, and that depends on the two
  laterals alone, which only a push changes. A later pass skips such a pair until a push has moved one of its two
  vehicles (a counter a vehicle, the pair's stamp is the sum of the two). Pairs are visited in scan order, so
  pushes happen in the same order as before. A retirement during a pass resumes as a scan from the position after
  the pair it was on; a list over 192 pairs is not kept and the passes scan in full.
- **Cheap rejection.** A pair is rejected on the signed lateral difference (with the tube's wrap) before the
  push code; the tube circumference and the line's half width are taken where a pair needs them, without the
  sample's speed cap and lateral. The push code is moved out of line unchanged.

The step's stack grows by about 4.5 KB (per-vehicle arrays for 255 vehicles, the packed rows and a 192-pair list).

**No console run was done in this repository.** Evidence by tier:

- **HOST:** `line_field_exact` passes 26,143,380 checks under GCC and Clang (`-O2 -Wall -Wextra -Werror -pedantic`)
  and under GCC with ASan and UBSan; the whole CTest set passes under GCC and Clang (68 of 68). Every pin of the
  line-field-12 suite is unchanged. Two new checks record the line-field-12 step's results: 3000 random scenes
  (8 to 96 nodes, tube nodes, behaviours, 2 to 60 agents, external vehicles, retired, stopped and falling starts,
  vehicles on the node next to the one holding their distance, several contact reaches; one hash over every tick's agents and counters: 38,053 contacts, 8,483 retirements,
  19 falls), and two agents half a line apart with the signed gap on the edge of its wrap, both ways round.
  The same generator over 20000 scenes gives the same hash on the line-field-12 and line-field-13 sources.
- **Controls:** a scene fewer hashes differently; the half-line pair differs from a shifted one.
- **Mutation controls (31 applied, 30 caught, the other equivalent):** the stamp skip ignoring a pushed vehicle,
  counting only one of the two, never counting, or starting at 0; an overlapping pair keeping its stamp;
  the neighbour test needing half the reach, missing the wrap pair or strict on ties; the row start's distance
  taken from the packed row; no refresh of the rows after a retirement (three places); the half width with a
  clamp off by one or without its floor; the wrap taken from the vehicle behind; the pair's two vehicles
  swapped, or the gap's sign; the resume one column early or missing; the retirement checks on the list removed;
  a truncated list kept; the width of a non-tube node not taken; the lateral reach loosened; the seam wrap
  removed; a fourth pass; the full scan skipped; the signed gap's wrap off by one at its upper edge. The one that
  is equivalent changes the signed gap's lower edge to a strict test; it changes nothing the contact code reads.
- **Instruction model (MODELLED, flat-RAM Yabause model of the compiled SH-2 object, no caches):** the contact
  phase of four scenes (a loose field to a pile), cycles before and after: 32,387 to 30,056 (loose field, -7%), 67,065 to
  51,891 (-23%), 84,188 to 61,165 (-27%) and 97,038 to 68,705 (-29%, a pile). The hash chains of the model
  runs are identical.
- **Consumer (not this repository):** a Saturn racing engine's native field.
  - HOST: six courses, seeds 1 to 6, behaviours on and off, built on the line-field-12 and line-field-13 sources:
    72 of 72 reports equal (final state hash, contacts, falls, knock-outs and every counter).
  - EMULATOR (Ymir, timing MODELLED): the engine's six-round Grand Prix run with the library the only difference
    (SH2ZAM main and this branch overlaid, not the engine's pin): the field's state hash and contact count are equal at
    all 21,307 race ticks; the HOST checkpoints of its Mute City, Sand Ocean and Big Blue races are equal (36/36,
    24/24, 32/32) and its stack margin is 101 KB or more. The frame's critical-path 99th percentile on round 3 (a tube
    course) fell from 28.14 to 27.01 ms (master 27.94 to 25.93) and on round 5 (a tube course) from 26.06 to 25.11 ms
    (master 26.04 to 25.02); the other rounds moved by 0.0 to 0.5 ms. Every interval stayed at two fields.
- **Not done:** console execution.
