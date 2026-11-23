# Validation record — 2026-11-22 (line field, per-machine pit recharge and an airborne exclusion, line-field-15)

`sh2zam/line_field_experimental.h` ([guide](../LINE_FOLLOWER.md)) adds one parameter, `surface->recharge_share`,
appended to the surface block, and one function, `sh2z_field_recharge`. With the share 0 the step is
line-field-14's, bit for bit (the one global `recharge` a tick, in the air or not), and nothing else changes: no new
brain field, no new state, no change to any other entry point.

What the field did not have: a recharge strip refilled every vehicle by the one global rate, whatever its capacity,
and in the air as on the ground. A field that has to reproduce a reference in which each vehicle on a strip refills by
a share of its own energy capacity a tick, and not while it is off the ground, could say neither. And a vehicle the
owner drives itself (an external vehicle) was never touched by the strip at all.

- **Rule (step).** With `recharge_share` > 0 (Q16; a negative value is read as 0), a vehicle on a recharge zone
  gains `(max_energy * recharge_share + 0x8000) >> 16` (Q8 energy, a 64-bit product, rounded to nearest) a tick,
  for the ticks the step was due, clamped at `max_energy`, and `recharge` is not used. A vehicle with `SH2Z_VEH_AIR`
  set gains nothing; the flag is the one the previous step left (a step sets it after it has looked at the strip),
  so a vehicle in its first air step has still the ground flag. Retired and finished vehicles gain nothing, as
  before; a falling one is not stepped.
- **Function.** `sh2z_field_recharge(f, v, ticks)` finds the zone under `v->t` with the same static function the step
  uses, and when it is a recharge zone and the vehicle is not RETIRED, FINISHED or FALLING applies the same gain for
  `ticks`. The arithmetic is one static inline, called from both, so the two cannot differ. With the share 0 the
  function applies the global rate in the air, as the step does (the air test belongs to the share). Does nothing
  with no surface, no zones or no ticks; only `v->energy` changes.
- **Cost.** One compare of the share on a recharge zone; nothing elsewhere.

**No console run was done in this repository.** Evidence by tier:

- **HOST:** `line_field_exact` passes 26,367,628 checks (main's line-field-14 tests included) under GCC and Clang (`-O2 -Wall -Wextra -Wconversion -Wshadow
  -Werror -pedantic`) and under Clang with ASan and UBSan; the whole CTest set passes under all three (68 of 68
  each). Every pin of the line-field-14 suite is unchanged (the existing tests are not edited). New checks
  (`test_recharge_identity`, `test_recharge_share`, `test_recharge_external`), each with its control:
  - (a) share 0 is the old step: 400 random scenes (a closed line of 96 nodes with five air runs and two tubes, 3 to 7
    zones of which most are recharge strips, 2 to 23 agents with random capacities and energies, behaviour block on
    in three scenes of four, 260 ticks, every agent's place, energy and flags and the field hash mixed in each
    tick) hash to `0x6f29a14e3fe9f895`, the value main at `0aba730`
    (no share field, before line-field-14) gives for the same generator, compiled against its library with GCC -O2,
    GCC -O0 and Clang -O2 (three equal values); the test still passes on main with line-field-14 merged, whose step is
    the same at its zero parameter. 80,548
    agent-steps in the air on a recharge strip are in the scenes, the case the share changes. Controls: the same
    scenes with a share of 328 and of 1200 hash to two other values; a negative share hashes as 0;
  - (b) share on: two agents far apart on the strip, capacities 160 and 80 (Q8 units of 256), start energy 5, share
    328: after one tick they hold 5 + 205 and 5 + 103 (204.99 and 102.5 round to nearest), after 50 ticks 50 times
    that; capacities 150 and 37 at a share of 1000 after 7 ticks equal 7 times the independent oracle
    `floor((max * share + 32768) / 65536)`. Control: share 0 gives both 512 a tick, the claim above fails on it;
  - (c) air: an agent flagged `SH2Z_VEH_AIR` gains 0 (share 328) and gains 205 when the flag is clear; control: at
    share 0, and at a negative share, it gains the global 512 in the air. A real flight, an agent over 10 air
    nodes with the behaviour block, 300 ticks: every step is checked against the oracle, at least 40 steps start in
    the air and at least 40 on the ground; with the share on the air steps change nothing; control: at share 0 the
    air steps gain the global rate (4 a tick) exactly;
  - (d) the function equals the step: over 1500 random states (position, lateral, energy, capacity, a third of them
    flagged in the air, share 0 for a third and 1 to 2000 for the rest, over three zones of which two are recharge
    strips and one a boost plate), one field step on a lone agent gives the energy that the function gives for one tick
    on a copy of the vehicle as the step found it (more than 200 states in a recharge zone and more than 200 out, more
    than 100 gains); and for a far agent on a level-of-detail period above 1, the function with the ticks the step was due.
    The function alone: on a strip, on a boost plate, beside the strip, 0 ticks, a retired, finished, airborne and
    falling vehicle (each gains nothing), an external vehicle (gains), no surface and no zones (nothing). Controls: one
    more tick differs unless the vehicle is full;
  - (e) clamp: three below the maximum, at the maximum, a share of 1.0 from 0 energy, share 0 three below the
    maximum, and the function with 2,147,483,647 ticks, all end at `max_energy`.
- **Controls (mutation, 11 applied, 11 caught):** the air test removed; the share path using the global rate; the
  rounding offset dropped; no clamp in the share path; the function accepting every zone kind; the function without
  its state exclusion; the function applying one tick too many; the air test applied at share 0 too; the share path
  taken at a share of 0; the step ignoring the share; the capacity rounded down to a multiple of 1024 before the
  gain.
- **Instruction model:** none; there is no instruction model for the line field, so no cycle count is claimed. The
  added code is one compare and, with the share set, a 64-bit multiply, on a step that is on a recharge strip.
- **Objects:** no libc `memcpy`, `memset` or `memmove` in the SH-2 objects (CI checks).
- **Not done:** console execution; a consumer measurement (the consuming engine has not yet run this branch).
