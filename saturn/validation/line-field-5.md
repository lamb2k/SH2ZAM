# Validation record — 2026-11-20 (line field behaviours, line-field-5)

`sh2zam/line_field_experimental.h` ([guide](../LINE_FOLLOWER.md)) adds `params->behave`
(`sh2z_field_behave_t`): tube roll and hold, lateral inertia, air and falls, drops on drop nodes, a
knock-out credit, a near-focus knock-out gate and impact damage. NULL is line-field-4 step for step.
**No console run was done in this repository.** Evidence by tier:

- **HOST:** `line_field_exact` passes 22,675,873 checks under GCC and Clang (`-O2 -Wall -Wextra -Werror
  -pedantic`, ASan and UBSan), and the CMake strict build:
  - a behaviour block whose tubes and air no agent reaches gives the pinned line-field-1 hashes on six
    seeds, with no falls and no knock-outs;
  - an agent that always rolls goes more than twice round a hold tube; one that does not roll keeps its
    angle there; without the hold mask it steers back to its line; a latched roll ends at an exit stretch;
  - with lateral inertia the lateral velocity changes by at most `lat_accel` a tick and the agent can
    reach the edge;
  - in the air beyond an open edge an agent falls and retires exactly `fall_ticks` later with energy 0;
    the control without an open edge never falls;
  - the knock-out credit follows the original's predicate: credited when the age after the falling step
    is at most 120 (119 and 120 credited, 121 not), once, never to a finished attacker; no window, no
    credit;
  - far from the focus, contact cannot empty an agent (it keeps `ko_floor`); near it, or with no gate,
    it retires with energy 0;
  - drops: an agent 2.0 of the radius round a drop node of an outside tube drops off; the controls at
    1.5 of the radius, off a drop node, on an inside tube (a pipe), with a chance of 0 and with a
    chance of 1 / 65536 do not;
  - launches: out of a pipe beyond the open edge of a drop road an agent is launched off; the controls
    with a chance of 0, without a drop node and inside the edge land on the road;
  - impact: with `contact_norm` both vehicles pay the same share of the closing speed and no flat
    energy; an attacker's victim pays twice that and the attacker nothing; the control without
    `contact_norm` charges the flat side energy.
- **Mutation controls (21 of 21 caught):** retained energy at retirement, the knock-out boundary moved
  either way, the wrong attacker, a duplicate credit, the retirement tick shifted, a finished attacker
  credited, no roll, no hold, no exit, no inertia, no gate, walls in the air, drops off inside tubes,
  `drop_lat` ignored, `drop_chance` ignored, launches off any node, `launch_chance` ignored, a flat term
  added to the impact, the victim paying a single share, and the attacker paying.
- **Consumer (not this repository):** a Saturn racing engine's HOST checks against its original code on
  six courses, two of them with pipes and cylinders: rival bands 13 of 13 on each course; the share of
  rival time 60 and 90 degrees round its tubes within 5 points on held-out seeds; falls on the tube
  courses at the original's rate within a factor of 2 over 18 races; impact damage per race inside or
  near the original's range.
- **Not done:** an instruction-model capture, console execution. Frame cost was measured only inside the
  consumer's emulator runs (MODELLED there).
