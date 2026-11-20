# Racing-line follower and line field (`line-follower-1`, `line-field-1` to `line-field-4`, experimental)

Contracts: [`line_follower_experimental.h`](include/sh2zam/line_follower_experimental.h),
section "Contract tag line-follower-1", and
[`line_field_experimental.h`](include/sh2zam/line_field_experimental.h), section
"Contract tags line-field-1 to line-field-4". Tests: `tests/test_line_follower.c` and
`tests/test_line_field.c`. Validation records:
[line-follower-1](validation/line-follower-1.md) and
[line-field-1](validation/line-field-1.md), [line-field-2](validation/line-field-2.md),
[line-field-3](validation/line-field-3.md) and [line-field-4](validation/line-field-4.md). No
instruction model yet: both are compiled C and have only been measured as part of a larger program.

Two layers for vehicles that travel along one closed, baked line: a lap
course, a patrol loop, a rail.

## Layer 1: the follower (`line-follower-1`)

A vehicle is a distance along the line plus a lateral offset from it
(`sh2z_track_state_t`). The line is a list of 16-byte nodes baked offline;
each carries a target lateral offset, a speed cap, a usable half width, and
the reciprocal of its span, so sampling never divides.

| Function | What it does |
| --- | --- |
| `sh2z_line_locate`, `sh2z_line_sample` | node lookup from a hint, and interpolated lateral, speed cap and half width |
| `sh2z_track_advance` | moves along the line, wraps, counts laps |
| `sh2z_follow_step` | speed toward a target under acceleration and braking limits, proportional steering kept inside the half width, for 1 to 64 ticks at once |
| `sh2z_pace_target` | a rubber band to an expected distance |
| `sh2z_follow_avoid` | a lateral target around the vehicle ahead |
| `sh2z_lod_period`, `sh2z_lod_due` | 1/2/4/8-tick periods by distance, with exact catch-up |
| `sh2z_rank_sort`, `sh2z_order_by_dist` | race order and physical order (insertion sorts: cheap when nearly sorted) |
| `sh2z_contact_resolve`, `sh2z_contact_external` | sideways push and speed limit for touching pairs |

## Layer 2: the field (`line-field-1`)

`sh2z_field_step` runs a whole field for one tick. `sh2z_vehicle_t` wraps the
track state with energy, limits and flags, and is meant to be shared with
vehicles driven elsewhere: one with `SH2Z_VEH_EXTERNAL` set is never moved by
the field, but takes part in order, contact, rank and laps. Contact against
it arrives as `kick_lat` and `kick_speed`.

- **Start grid:** `sh2z_field_grid_slot` places grid position `p` in staggered
  rows of 1 to 4 columns.
- **Pace:** each agent follows its own lap-time pace with a rubber band, or
  chases another vehicle at a set lead. When the chased vehicle retires or
  finishes, the agent carries on at its own pace.
- **Second-lap re-target (`line-field-2`):** an agent given a finish target
  (`finish_ticks`) re-paces when it completes its first lap, to cover the
  remaining laps evenly by that race tick, and its expectation restarts at one
  lap. Racing games often pace the opening lap on a lap time and the rest on a
  finish time; with `finish_ticks = 0` the field runs exactly as `line-field-1`
  (same steps, same hashes).
- **First-lap chase cap (`line-field-3`):** a chasing agent given `first_lap_cap`
  holds its target speed to that cap until its first lap is done, as games keep
  chasers at their machine's own top speed while the field forms; 0 is no cap.
- **Surface (`line-field-4`):** `params->surface` gives the line zones and tube nodes; NULL keeps
  `line-field-3` step for step (same hashes).
  - Zones are distance stretches with a lateral band, sorted by start: a boost plate gives a free
    boost of `plate_ticks`, low grip scales the steering gain by `slip_steer`, rough ground scales
    the target speed by `rough_keep` unless boosting, and a recharge strip refills `recharge` energy
    per tick. An agent takes the zone it is in when its step starts.
  - Tube nodes (`(flags & wrap_mask) == wrap_value`) have no edge: lateral is an arc length round a
    circle of radius `half_width` that wraps at plus or minus pi times it. Agents steer the short way
    round, are never charged at an edge there, and contact pushes wrap. Leaving a tube for a node
    with an edge puts an agent inside the new half width without a charge.
- **Final dash (`line-field-4`):** with `dash_from` > 0, an agent on the final lap past that
  distance drops its pace and chase targets, targets at least `dash_pace` of its pace and boosts
  whenever its energy allows, as racing games often let the field race for the line.
- **Lanes:** an agent takes the side with more room around a slower vehicle
  ahead, looking further ahead the faster it closes. It holds that side for a
  while, never cuts across a vehicle alongside, and does not move across a
  faster vehicle close behind on its line (the blocking limit).
- **Boost:** after a set number of laps, with enough energy, when behind its
  pace.
- **Contact:** touching pairs are pushed apart. What one cannot take (the
  line's edge, an external vehicle) goes to the other. Energy is charged once
  per incident, scaled by each vehicle's armor. A side attack on the chased
  vehicle charges the victim extra.
- **Retirement:** at zero energy a vehicle leaves contact at once and coasts
  to a stop.
- **Rank and laps:** finishers first in finishing order, then by distance,
  retired vehicles last. A place changes hands only on a clear lead
  (`rank_lead`), so vehicles side by side do not flicker.
- **Level of detail:** with a focus vehicle set, far agents step every 2, 4 or
  8 ticks. Agents on the same 512-unit stretch share a phase.

## Determinism and limits

- Integer only, no allocation, no global state, and no division at run time
  except one per re-targeting agent and race (`line-field-2`). One
  xorshift32 state in the field drives every random choice.
- The same initial state and the same external inputs give the same
  `sh2z_field_hash` on every target.
- Node spans lie in [2, 65536] Q8 units; totals stay inside (-2^29, 2^29); at
  most 255 vehicles.
- Contact is exact near the focus. Far pairs on different level-of-detail
  phases do not touch.
- A multi-tick step approximates that many single steps; it is not equal to
  them.
