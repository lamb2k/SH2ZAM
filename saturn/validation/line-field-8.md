# Validation record — 2026-11-21 (line field contact order within a step, line-field-8)

`sh2zam/line_field_experimental.h` ([guide](../LINE_FOLLOWER.md)) adds no entry point. Every step
(`sh2z_field_step`, `sh2z_field_step_plain`, `sh2z_field_step_behave`) now orders the field again from the
step's own positions before its contact passes. The contact scan from each vehicle ends at the first vehicle
out of reach. With the order from the start of the step, a vehicle that passed another within the step could
sit behind it in the order, so the scan ended at the passed vehicle and never reached the pair that touched.

**No console run was done in this repository.** Evidence by tier:

- **HOST:** `line_field_exact` passes 26,152,565 checks under GCC and Clang (`-O2 -Wall -Wextra -Werror
  -pedantic`, ASan and UBSan):
  - a hand-placed overtake: an agent closing at 58 a step passes a vehicle 50 behind the leader and lands
    2 behind the leader on its lateral, with a fourth vehicle far behind; the pair is found, charged once and
    pushed apart, and no pair is left inside a quarter of the contact box. Against the line-field-7 step the
    same scene fails: no contact, the pair left inside each other;
  - every other check as before, with three plain-race pins moved (below).
- **Pins that moved:** the plain races of seeds 2, 5 and 6 (24 agents, 5000 steps), pinned since line-field-1
  as `0x9c2d952f`, `0x080ae758`, `0x244b0332`, are now `0xb6a98eab`, `0xe86c77ef`, `0x36df557b`. Seeds 1, 3
  and 4 and the six line-field-5 behaviour pins are unchanged. The order is a strict total order of the
  positions, so the step differs only in its contact passes. In a diagnostic build of the line-field-7 step
  (not in this repository), seed 6 had a touching pair the stale order never scanned. Seeds 2 and 5 had none;
  the contact passes there met the same pairs in another sequence, and a pile-up settled differently.
- **Controls:** without contact the overtake pair stays inside (the check can fail).
- **Mutation controls (5 of 6 caught):** the second ordering removed, the ordering moved after the contact
  passes, the slot table not rebuilt, the order ascending, and the order taken only on the first step. Not
  caught: ties broken the other way (no two agents share a distance in these races).
- **Consumer (not this repository):** a Saturn racing engine's native field on six courses, seeds 1 to 6, with
  closer-racing settings: without the re-order, races on two of the six courses left a vehicle inside another
  after a step (one step each race); with it, no such step in the same 36 races.
- **Not done:** an instruction-model capture, console execution. The re-order is one insertion pass over an
  order that is nearly sorted (one comparison a vehicle when nobody passed); its frame cost was measured only
  in the consumer's emulator runs (MODELLED there).
