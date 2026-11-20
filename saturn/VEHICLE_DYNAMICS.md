# Track-bound vehicle dynamics (experimental, contract tag `vehicle-dynamics-1`)

[`vehicle_dynamics_experimental.h`](include/sh2zam/vehicle_dynamics_experimental.h) steps one
hovering vehicle on a closed road laid along a cardinal spline, one fixed tick at a time, in
integer arithmetic. It is meant for a racing game's player vehicle: the caller supplies the road,
the vehicle's constants and the rule constants, so the library holds the algorithm and no game's
numbers.

Nothing selects it by default. The tag is experimental: its numeric definitions may change until
the tag is published.

## Status

- **Host and SH-2 agree bit for bit.** Every function is integer only: 32x32 to 64 products
  rounded to nearest, quotients from `sh2z_divu_div64`, square roots and normalisation from
  `fixed-unit-1`. A caller's HOST build of the same C gives the same state words as its SH-2
  build, tick by tick.
- HOST tests (`tests/test_vehicle_dynamics.c`) on a synthetic stadium cover the spline identities,
  the closest-point search, launch and the force/damping balance, steering (dead zone, linearity,
  saturation, lean), a signed wall response, surface zones, laps and the race clock, the line-space
  view and a golden state hash, each with a negative control.
- Evidence tiers: HOST and MODELLED (below). Callers record their own qualification against a
  reference.

## MODELLED cost

The [instruction model](benchmarks/vehicle-dynamics-model/README.md) runs the SH-2 object code in
the pinned Yabause interpreter: flat RAM, no caches, no wait states. Its figures are not Saturn
timing. It checks every tick and every probe against the host build, word for word, before it
times anything:

- 5,808 ticks over five input scripts (straight, steering and boost, a wall, airborne, a three-lap
  race), half of them with SR.S set;
- 1,366 closest-point searches and 86 that report the target off the line.

Capture: [results.json](benchmarks/vehicle-dynamics-model/results.json), CI run
[44660318294](https://github.com/lamb2k/SH2ZAM/actions/runs/44660318294), ELF SHA-256
`72836f49...`.

| Call | Mean cycles | Median | Max | Mean instructions |
| --- | ---: | ---: | ---: | ---: |
| `sh2z_vd_step`, one tick (all scripts) | 9,420.7 | 9,045 | 16,187 | 7,991 |
| `sh2z_vd_locate`, 100 iterations, 2^32 tolerance, target within 600 units | 6,411.8 | 5,870 | 92,046 | 5,433 |
| `sh2z_vd_locate`, target off the line | 872.3 | 342 | 14,499 | 711 |
| `sh2z_vd_lap_distance` | 394.7 | 387 | 468 | 325 |

The step includes its own closest-point search. A tick costs about 9,400 modelled cycles, or about
0.33 ms at 28.6 MHz before memory waits.

## Formats

| Quantity | Format |
| --- | --- |
| Positions, velocities, accelerations, energies, widths, heights | Q16.16 |
| Unit vectors, the spline parameter `t`, factors below 2 | Q2.30 |
| Arc lengths along the line (node starts, lap and race distance) | Q20.12 |
| Timers, counters, flags | integers |

World up is +y. A road frame's z axis points to the road's left; positive stick steers right.

## The tick

`sh2z_vd_step(body, track, vehicle, rules, input)` runs, in order:

1. Pad: buttons and stick, read only while the body is controllable (an intro timer hands control over).
2. The surface zone under the vehicle (recharge, rough, boost plate, ice), by spline `t` and side offset.
3. Steering: forward -= rate x stick x left, then re-orthonormalised; the rate depends on the grip state and lean.
4. Damping in the body frame (side, up, forward), with extra damping on rough ground and when braking.
5. Boost from the pad (when unlocked and energy remains) or a free boost on a plate.
6. Mines and launch ramps.
7. Gravity along the vehicle's gravity-up vector, the launch impulse, and the hover spring.
8. Leans and attacks (double tap), each a side force that fades with side speed.
9. Grip: side speed removed up to a limit picked by a small state machine.
10. Slide turbo from sustained side speed.
11. Engine: a force target from a speed curve, less turning and lean costs, slewed toward.
12. Integration and the speed cap.
13. The closest point on the spline (`sh2z_vd_locate`), lap distance, half widths, road frame.
14. Race clock, lap crossing (forward and backward) and finish, with the crossing interpolated.
15. Road shape: height, ground clamp, airborne thresholds, walls (push back, damage, reflected side speed).
16. The up vector eases toward the road normal; body-frame velocity; recharge; attack motion; spin-out; hover rise.

`sh2z_vd_locate` is an iterative search: each step moves `t` by the projection of the remaining
offset on the tangent over the tangent's squared length, crosses to the next or previous node when
`t` leaves [0, 1) (rescaling `t` by the ratio of tangent lengths), and turns the offset into the
plane across the spline while keeping its length. It stops below the caller's squared tolerance,
after the iteration limit, or reports the target as off the line.

## Sharing with a line-space field

`sh2z_vd_line_view` gives the body's distance along the line and lateral offset (Q8, lateral
positive to the right), along-line speed and lateral velocity, and completed laps, which is what a
line-space field of other vehicles needs to rank, pace and collide with it.
`sh2z_vd_apply_kick` applies a contact kick from such a field (lateral and speed deltas) before
the next step.

## Not covered

- Pipe, cylinder and half-pipe road shapes are treated as an open road.
- Vehicle-to-vehicle contact inside the model; the line-space field resolves contact and passes kicks.
- Real memory timing (cache, SDRAM, bus contention) and a whole-frame measurement on hardware.
