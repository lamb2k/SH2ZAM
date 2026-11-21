# Validation record — 2026-11-21 (line field without the extra lateral holds, line-field-9)

`sh2zam/line_field_experimental.h` ([guide](../LINE_FOLLOWER.md)) adds no entry point and no parameter. Every
step (`sh2z_field_step`, `sh2z_field_step_plain`, `sh2z_field_step_behave`) drops three lateral holds that the
reference driver the field models does not have:

- **Post-contact room hold.** After a contact, both vehicles were set to hold the side they were pushed to for
  `pass_hold` ticks. The reference's contact is a push and an impact only.
- **No-cut hold.** A target lateral with a vehicle between it and the current lateral was replaced by the
  current lateral. The reference steers to the side it picked around the vehicle ahead and nudges away from it
  when close. Nothing holds the lateral for a vehicle in between.
- **Blocking hold.** A vehicle kept its previous target while a faster vehicle was close behind on its line.
  The reference reads the vehicle behind only for a side attack.

Avoiding the vehicle ahead, holding that side for `pass_hold` ticks and the side attack are unchanged.
`pass_hold` now only times the side chosen around the vehicle ahead.

**No console run was done in this repository.** Evidence by tier:

- **HOST:** `line_field_exact` passes 26,143,316 checks under GCC and Clang (`-O2 -Wall -Wextra -Werror
  -pedantic`) and under Clang with ASan and UBSan. Three new scenes, each on frozen agents over four steps
  (every agent decides once):
  - room: a pair 20 apart and 20 across touches. It is charged once, and neither agent holds a side.
  - no-cut: an agent 100 off its line, with a neighbour 50 off the line, 10 ahead or 10 behind, targets its
    line whether or not the neighbour is in between.
  - blocking: an agent on its line with a stale target 80 off it, and an agent 80 behind closing at 4 a tick.
    It targets its line again, with the agent behind faster or not.
- **Controls:** each scene can fail.
  - With avoidance on, the room pair shows one held side: the trailing agent picks a side around the agent
    ahead.
  - With avoidance on, the no-cut target moves off the line.
  - With a pass side held, the blocking agent keeps its stale target.
- **Against the line-field-8 step:** the same tests fail five checks (room 2, no-cut 2, blocking 1) and all
  pins.
- **Mutation controls (7 of 7 caught).** Each mutant restores one removed hold or part of one: the room hold
  on both agents, on the agent ahead only, or on the trailing agent only; the no-cut hold on both neighbours,
  the one ahead only, or the one behind only; and the blocking hold. Each fails its own scene and the pins.
  - In a two-agent field the neighbour behind is also the next agent ahead round the line. So the ahead-only
    and behind-only no-cut mutants each fail both no-cut checks.
- **Pins that moved:** the scenes with close racing change, as intended.
  - The six plain-race pins (24 agents, 5000 steps) are now `0x26258cb9`, `0xae50d21f`, `0x9821ea57`,
    `0xc1fc223f`, `0xe9d006cf`, `0xa428c63b`. They were `0x7c56497e`, `0xb6a98eab`, `0x12afee5b`, `0xccfb9880`,
    `0xe86c77ef`, `0x36df557b`.
  - The six behaviour pins are now `0xc2a25804`, `0x47960c34`, `0xcdea24dd`, `0x864b559c`, `0x9c64a595`,
    `0x68fe14f3`. They were `0x68687a12`, `0xbaccf237`, `0xabc03fa3`, `0x29f21133`, `0x12e650c6`, `0xbb9479cd`.
  - Every other check is unchanged, including "nobody is stuck: the race ends" on all six plain races.
- **Consumer (not this repository):** a Saturn racing engine's native field ran a HOST lab build with the same
  three removals on six courses, seeds 1 to 6. Its CPU-pair contact damage came inside the consumer's frozen
  band (0.67 to 1.50 of its reference) on all six courses. With the room hold, one course fell below the band.
  The consumer rescores on this library build before merging.
- **Not done:** an instruction-model capture and console execution. The change removes work from the step: one
  neighbour lookup and two lateral checks for each deciding agent, and two brain writes for each touching pair.
