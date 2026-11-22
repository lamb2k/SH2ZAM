# Validation record — 2026-11-22 (line field pace ranks, the driver's slowing and wall impacts, line-field-12)

`sh2zam/line_field_experimental.h` ([guide](../LINE_FOLLOWER.md)) adds `params->ranks` (pace ranks, coast, floor,
after-drive), `brain->pace_rank` and `behave->wall_impact`. With `ranks` NULL and `wall_impact` 0 the step is
line-field-10's, bit for bit, and its hashes are unchanged.

What a field of racing agents does that the line field did not:

- **Pace ranks that move.** A finish target that follows each agent's pace rank. A chaser released in the last lap joins
  the ranking at the rank of the nearest ranked agent, the list is renumbered (rank, then the larger distance covered
  first) and each agent behind it moves down a place: its target moves later by a step. A retiring agent leaves the
  ranking (rank 254) and the places close up at the next release. One division per re-target.
- **Slowing is coasting.** A vehicle has no brake on a pace. With `coast` an agent whose target holds it below its
  speed loses `coast` of its speed a tick (at least 1) in place of the vehicle's brake; a road cap that holds it is
  still a braking limit. `floor` is the least target speed, applied after the cap. `after_speed` is every agent's
  target once the focus vehicle has finished (no paces, no chases, no new boosts).
- **A wall costs by its impact.** With `wall_impact` a wall costs that share of the speed the agent hits the edge at
  (how far a contact push or a step took it past the edge, at most its steering speed), once, and scrubs speed only
  then; an agent resting on the edge pays nothing. Line-field-5 charged a flat `wall_energy` for every wall call, up
  to four a tick, so a pile pressed on the edge lost its energy within a few ticks.

**No console run was done in this repository.** Evidence by tier:

- **HOST:** `line_field_exact` passes 26,143,376 checks under GCC (`-std=gnu11` and `-std=c11`, `-O2 -Wall -Wextra
  -Werror -pedantic`), Clang, and GCC with ASan and UBSan; the library object has no libc `memcpy`, `memset` or
  `memmove`. Every earlier pin is unchanged. New checks:
  - the nearest rank: three ranked agents on paces 1300, 1400 and 1600 and a chaser that keeps beside the third.
    Released, the chaser takes the rank of the nearest ranked agent (3 or 4, never 1), the live ranked agents hold
    exactly 1..n, and with the second agent retired on the wall it holds 254 and the others close up;
  - retirement: a ranked agent that retires holds 254; with ranks NULL its rank is untouched;
  - the hash includes the rank only with ranks set;
  - coast: ten ticks of an agent at 50 units a tick lose exactly the coast share each (and 96 a tick with coast 0);
    floor: it holds the floor at 700 ticks (and falls below 15 units without one); after-drive: it reaches
    `after_speed` once the focus has finished, and not before it, and not with `after_speed` 0;
  - wall impact: a pair pressed on the wall pays 0.7 of its steering speed when pushed 15 units into it and 0.7 of
    3 units when pushed 3 (exact), the flat charge is the same for both with `wall_impact` 0, an agent resting on the
    edge for 400 ticks pays nothing and keeps its speed (with `wall_impact` 0 it empties).
- **Mutation controls (13 of 13 caught):** no renumber after a release; a release at rank 1 in place of the nearest;
  no re-target on a rank change; a retirement that keeps its rank; a retired agent left in the renumbering; no
  coast; no floor; no after-drive; after-drive before the focus has finished; the wall charged flat; a resting agent
  charged or scrubbed; no cap at the steering speed; the hash without the rank.
- **Consumer (not this repository):** a Saturn racing engine's native field, six courses, seeds 1 to 6, built on
  the line-field-10 and line-field-12 sources with every new parameter at zero: 72 of 72 reports equal. With the
  parameters set, the two behaviours the original driver shows and the field lacked are present: the slowest
  racers coast down to the floor in the last lap and then drive on once the focus has finished (the last finisher
  was 300 to 1000 ticks late and is now 0 to 300), and racers pressed on a wall in the start crowd are no longer
  retired in the first 700 ticks (9 to 13 early retirements in 6 races on two courses before; none after).
- **Not done:** an instruction-model capture and console execution.
