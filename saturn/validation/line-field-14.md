# Validation record — 2026-11-22 (line field, a gap too long to clear, line-field-14)

`sh2zam/line_field_experimental.h` ([guide](../LINE_FOLLOWER.md)) adds one parameter, `behave->gap_reach_ticks`, appended
to the behaviour block. With it 0 the step is line-field-13's, bit for bit, and nothing else changes: no new brain
field (the brain stays 92 bytes), no new entry point, no state kept between steps.

What the field did not have: an agent that crosses a stretch of road with no surface under it (a run of air nodes) and
falls short of the far side. The line field's air rule lets an agent fly over the whole run and land on the far
side, and only an open edge drops it; a field that must reproduce a reference in which every agent falls into such a
gap (and so retires) had no way to say so.

- **Rule.** With `gap_reach_ticks` > 0, an agent in the air (on a node matching the air mask, not in a tube) has the
  reach `speed * gap_reach_ticks` (speed is a distance a tick, both Q8, a 64-bit product). It is dropped on the step
  in which it has covered that reach from the start of its air run while still in the run. The run is the contiguous
  nodes that match the air mask, back from the agent's own to its start (at most 64 nodes back, not across the
  line's start). The drop is the open-edge fall: `SH2Z_VEH_FALLING`, one fall counted, out of contact, no laps, and
  `fall_ticks` later the agent retires with energy 0 through the ordinary path (the knock-out credit is the open-edge
  fall's).
- **Boundary.** The agent is dropped only while inside the run, so a run exactly as long as the reach, or shorter, is
  cleared: it is past the end by the time it has covered the reach. A stopped agent (speed not above 0) does not fly and
  is never dropped by the rule.
- **Cost.** One scan back along the run (at most 64 node flags) a step for an agent in the air with the field set; none
  otherwise (the field is tested first).

**No console run was done in this repository.** Evidence by tier:

- **HOST:** `line_field_exact` passes 26,365,473 checks under GCC and Clang (`-O2 -Wall -Wextra -Wconversion -Wshadow
  -Werror -pedantic`) and under GCC with ASan and UBSan; the whole CTest set passes under GCC, Clang and the
  sanitizers (68 of 68 each). Every pin of the line-field-13 suite is unchanged. New checks (`test_gap_fall`,
  `test_gap_fuzz`):
  - a 3-node run, 153,600 long, at a constant 10,240 a tick (accel and brake 0): reach off, 16 ticks and exactly 15
    ticks are cleared (no fall, no retirement); 14 ticks (one tick of flight short) drops the agent once, on the
    first tick with 143,360 covered, and it retires exactly `fall_ticks` later with energy 0 and no laps; a reach of 1
    tick drops it as soon as the run starts. A start one unit short of the reach on the previous tick does not drop it
    early, and an exact reach on a tick drops it on that tick;
  - no air nodes: never dropped. A ground node between two runs makes two runs, each cleared; the same nodes together
    are one run and drop the agent. A 90-node run (past the 64-node scan) drops. At the end of the line a run is cut
    by the line's length (exactly reachable: cleared; one tick short: dropped). The same run at half the speed drops,
    a stopped agent does not;
  - an oracle over 3000 random lone scenes (run 1 to 40 nodes, speed 2048 to 22,000, reach 1 to 60 ticks, first tick in
    the run anywhere in a tick; every tenth with the agent landing on the run's start and a run exactly the reach
    long): the rule's outcome equals the oracle's in 3000 of 3000, with 250 or more flights ending exactly on the
    run's end (the boundary), and both outcomes plentiful;
  - inert at zero, against main: the 3000 line-field-13 random scenes (8 to 96 nodes, tubes, behaviours, 2 to 60
    agents, external, retired, stopped and falling starts) hash as the line-field-12 step did (`X13_FUZZ_HASH`),
    with the new field 0; the same generator over 20000 scenes (differential harness, the test's generator
    compiled against main `0aba730` and against this branch) gives one hash for both,
    `e4e11a6ee678110b` (253,445 contacts, 55,893 retirements, 134 falls, 7,420,514 steps);
  - the rule on: the 3000 scenes with `gap_reach_ticks` 6 hash to a new pinned value (recorded from this step), with
    3672 falls against 19 with the field 0; 20000 scenes: 26,249 falls (reach 6) and 19,897 (reach 40) against 134.
- **Controls (mutation, 13 applied, 12 caught, the other equivalent):** the covered test `>` for `>=`; no scan back
  along the run; a scan limit of 1; reach without the speed; reach halved; a stopped agent that flies; the agent not
  required to be on an air node; the scan crossing a ground node; the reach one unit more, and one unit less; the rule
  applied when the agent is not in the air; the field's zero guard `<` 0 (the rule on at zero). The one that is
  equivalent adds a redundant `>= 0` test to a field already guarded.
- **Instruction model:** none; there is no instruction model for the line field, so no cycle count is claimed. The
  added code is an integer scan of at most 64 node flags, and only for an agent in the air with the field set.
- **Objects:** no libc `memcpy`, `memset` or `memmove` in the SH-2 objects (CI checks).
- **Consumer (not this repository):** a Saturn racing engine's native field on a reference course where every reference agent
  falls short of one gap and retires. HOST, the harness built from this branch's sources, a band and a rule frozen before the
  runs (reach 19 ticks, fall 127 ticks; every one of 29 agents retires, sorted retirement ticks within 40 rank by rank, at
  least 24 of 29 within 60 tick by agent): on 8 fresh seeds, each against its own reference run of the original code and
  with its pace taken from that run, 8 of 8 pass with a worst sorted error of 10 to 17 ticks and 29 of 29 agents within 60
  ticks on every seed; the same field with the rule off or at a reach of 40 ticks retires 0 or 1 of 29 (every run clears
  the gap), with a fall of 60 ticks the sorted error is 74 to 81 ticks, and with one pace for all agents 41 to 50 ticks, so
  each control fails. The development prototype and this branch retire every agent on the same tick (two seeds compared).
- **Not done:** console execution.
