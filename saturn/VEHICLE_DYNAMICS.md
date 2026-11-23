# Track-bound vehicle dynamics (experimental, contract tags `vehicle-dynamics-1`, `-2`, `-4` and `-5`)

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
[44675995095](https://github.com/lamb2k/SH2ZAM/actions/runs/44675995095), ELF SHA-256
`82234b60...` (first recorded from run 44660318294, ELF `72836f49...`, with the same values).

| Call | Mean cycles | Median | Max | Mean instructions |
| --- | ---: | ---: | ---: | ---: |
| `sh2z_vd_step`, one tick (all scripts) | 9,420.7 | 9,045 | 16,187 | 7,991 |
| `sh2z_vd_locate`, 100 iterations, 2^32 tolerance, target within 600 units | 6,411.8 | 5,870 | 92,046 | 5,433 |
| `sh2z_vd_locate`, target off the line | 872.3 | 342 | 14,499 | 711 |
| `sh2z_vd_lap_distance` | 394.7 | 387 | 468 | 325 |

The step includes its own closest-point search. A tick costs about 9,400 modelled cycles, or about
0.33 ms at 28.6 MHz before memory waits.

**vehicle-dynamics-2.** The same suite runs `sh2z_vd_step2` on the tube stadium of
`tests/test_vehicle_dynamics2.c` and checks 3,080 ticks against the host build word for word:
- walled road, where the host step2 body also equals `sh2z_vd_step`'s on every tick;
- a pipe and a cylinder, with the body on three sides of each;
- a drive from road through a joined pipe and out;
- the half-pipe (`vehicle-dynamics-5`): 240 ticks on its floor and a drive from road through a joined trough and out.

Half the ticks run with SR.S set. Same capture; the `sh2z_vd_step` rows above are unchanged value for
value. With `vehicle-dynamics-5` the step2 object takes the half-pipe case, so the earlier step2
scripts (same ticks, same results word for word) move by about 0.2% at most in cycles; the two
half-pipe scripts are new rows.

| Call | Mean cycles | Median | Max | Mean instructions |
| --- | ---: | ---: | ---: | ---: |
| `sh2z_vd_step2`, one tick (all tube scripts) | 8,942.4 | 8,752 | 16,304 | 7,578 |
| `sh2z_vd_step2` on the road | 9,284.3 | | | |
| `sh2z_vd_step2` in a pipe (three sides) | 8,766.6 to 8,802.4 | | | |
| `sh2z_vd_step2` on a cylinder (three sides) | 8,763.0 to 8,805.3 | | | |
| `sh2z_vd_step2` through a joined pipe | 8,954.4 | | | |
| `sh2z_vd_step2` on a half-pipe floor | 8,940.4 | | | |
| `sh2z_vd_step2` through a joined half-pipe | 8,981.6 | | | |

## Engine force scale (`vehicle-dynamics-4`, `SH2Z_VD_FORCE_FRAC`)

The engine force constants (`force_low`, `force_slide`, `force_high`, `top_force`) and the body's `accel_force`
are Q16 by default, so a constant such as 0.10027 is stored as the nearest multiple of 2^-16 (a stored 6571 for an
exact 6571.245). Held at top speed the vehicle settles at force / (1 - damp), so the error is a standing speed offset
(damp 0.997: 333 times the constant's error). `SH2Z_VD_FORCE_FRAC` (build time, 16 to 28, default 16) carries those
five words with that many fraction bits instead:

- Default 16: the object code of `vehicle-dynamics-1` to `-3` is unchanged (51 of 51 SH-2 objects byte-identical).
- Above 16 (24 is the tested scale): the force chain (selection by boost and slide turbo, the crossover curve and
  bump, the turn and lean costs, the slew) is evaluated in that scale, each product in int64 rounded once to nearest
  with ties up, saturating to int32; the force added to the acceleration is converted to Q16 once, at the force
  application (`accel_force x body axis`, rounded once at `30 + F - 16` bits), so the acceleration stays Q16.
- Range: Q(16+8) holds forces up to 128 (units of the caller's speed); a larger force saturates at INT32_MAX
  and never wraps.
- Every translation unit of one image must use the same value (the header sets the default); the caller's constants
  and the body's `accel_force` are in that scale. A caller that reads packs converts at load.
- The cost is one slightly smaller object (the chain has fewer conversions): 23,996 against 24,148 bytes for
  `sh2zam_vehicle_dynamics.o` at 24 (SH-2, `-m2 -mb -O2`).

## Search stop for a body far off the line (`vehicle-dynamics-6`, `SH2Z_VD_LOCATE_FAR`)

The closest-point search (`sh2z_vd_locate`, 100 passes from `sh2z_vd_step` and `sh2z_vd_step2`) moves along the spline by the
projection of its residual on the tangent. Next to the line that converges in a few passes (a mean of 2 to 3 on the tracks it
was measured on, at most about 55 on the worst tick of any of them). A body hundreds of units off the line, such as one in the air
over a gap, has no closest point to reach: the pass overshoots a span, crosses a node and settles into a circle of two to ten
states with a residual of hundreds of units, and the search runs all 99 passes every tick (about 20 ms on an SH-2 at 28.6 MHz).

`SH2Z_VD_LOCATE_FAR` (build time, whole units 0 to 32767, default 0 = off) with `SH2Z_VD_LOCATE_FAR_RUN` (default 8) stops a
search whose residual (the distance between the target and its ground point, the value the tolerance tests) stays above FAR
units for RUN passes in a row. It keeps the state it has; `sh2z_vd_locate` then ends as it does at the pass limit.

- Off (0): the object code of every earlier tag is unchanged (51 of 51 SH-2 objects identical).
- On: only a search that circles is stopped. A search that overshoots above FAR units and comes back within RUN passes is not
  stopped, and its result is the same word for word. The host test builds the model twice (stop on, and the library as the
  reference) and compares them over 120,000 randomised searches.
- The stopped state is one point of the circle the search was running round, as the state after 99 passes is. It is not the
  same point, so a body's path can differ from the 99-pass path from the tick of the stop on. The caller checks its outcome
  (the evidence record gives the one made on a road with a jump).
- A stop that is too eager changes searches that converge (FAR 3, RUN 1 is the control in the test).
- Cost: 48 bytes of text in `sh2zam_vehicle_dynamics.o` at FAR 128 (the contact units, which compile the search again, 240).
- Hooks for host builds, empty by default: `SH2Z_VD_LOCATE_FAR_HIT()` (a call that was stopped) and `SH2Z_VD_LOCATE_PASS(d)`
  (every pass, with the residual).
- Every translation unit of one image must use the same values.

## Search damping and flip stop (`vehicle-dynamics-7`, `SH2Z_VD_LOCATE_DAMP`, `SH2Z_VD_LOCATE_FLIP`)

The search steps the spline parameter by the projection of the residual on the tangent, `t += d.f / |f|^2`, and leaves out
the curvature term of the Newton step, `|f|^2 + (p - x).p''`. A body hundreds of units off the line, such as one over a gap,
runs the whole 99 passes in three ways, two of them under the `vehicle-dynamics-6` line (128 units) and so not stopped by it:

1. **Swing.** Near the centre of curvature (`-disp.p'' / |f|^2` near 1) the step is about twice the Newton step: the residual
   alternates in sign and closes by about 0.94 a pass (36 units to 1 in 54 passes, 883 units off the line).
2. **Join flip.** The closest point is the join of two nodes, with a different tangent length either side; the search flips
   between them, back to the same parameter every second pass, with a residual that does not fall (47 units, drifting by
   0.01 a pass). There is no exact repeat, so the cycle cannot be skipped exactly.
3. **Past the centre of curvature.** The curvature term passes -1, the minimum is gone, and the residual falls and rises
   again. Not changed by this tag.

`SH2Z_VD_LOCATE_DAMP` (build time, passes 0 to 99, default 0 = off): after that many passes of the call, the secant step
through the last two passes, `-along * prev_dt / (along - prev_along)`, replaces the pass's step when it is not longer. It has
the step's sign (the slope is negative), so the search is only ever damped, never pushed further than before. A residual that
falls one way takes the longer step. A node change forgets the last pass. A search of DAMP passes or fewer is unchanged.

`SH2Z_VD_LOCATE_FLIP` (build time, flips 1 or more, default 0 = off): the search ends once it has flipped between two nodes
that many times in a row, each time back to the parameter it had two passes before (within 2^-10 of a node), and on a pass
of the parity the 99th pass has. The state is then the one the 99-pass search would end on, to the drift of the cycle: the
join is on one side or the other of the closest point, and which side is the parity of the pass.

- Off: the object code of every earlier tag is unchanged (the three vehicle units compared with `cmp`, CI flags).
- Both on (DAMP 8, FLIP 3): +504 bytes of text in `sh2zam_vehicle_dynamics.o`. A secant step costs one division
  (`sh2z_divu_div64`) on a pass; the flip test, three compares.
- A search neither change acted on is the earlier search word for word. Where the earlier search converged within 700 units
  of the line and a change acted on it, the changed search converged too, within 1 unit of its end state, in 339 of 340
  cases in the host test; the exception ended 12.5 units off it (a flip stop at 40 passes; the earlier search closed in 59 more).
  Where the earlier search did not converge there is no right end state: the changed one may end elsewhere. The caller
  checks its outcome (the validation record gives the one made on a road with a jump).
- Hooks for host builds, empty by default: `SH2Z_VD_LOCATE_DAMP_HIT(old_dt, new_dt)` and `SH2Z_VD_LOCATE_FLIP_HIT()`.
- Every translation unit of one image must use the same values.

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

## Tubes (`vehicle-dynamics-2`)

`sh2z_vd_step2(body, track2, vehicle, rules, input)` is the same tick with two more road shapes.
`sh2z_vd_track2_t` holds the vehicle-dynamics-1 track as its first member, one join record per
node (`sh2z_vd_join_t`, or NULL) and the tube rule constants (`sh2z_vd_tube_rules_t`).

- **Pipe** (`SH2Z_VD_SHAPE_PIPE`): the ground is the inside of the circle of radius `half_left`
  round the line.
  - Height is the radius less the distance from the line.
  - The ground up vector points to the line, and gravity, the hover and the up vector follow it,
    so the vehicle can run all the way round.
  - There are no walls: past the radius the body is put back on it and loses its speed into the
    surface.
- **Cylinder** (`SH2Z_VD_SHAPE_CYLINDER`): the ground is the outside of that circle.
  - The up vector points away from the line.
  - Beyond the radius plus `cyl_reach` the body has left the surface and falls as off a road.
- **Half-pipe** (`SH2Z_VD_SHAPE_HALF_PIPE`, `vehicle-dynamics-5`): a trough, the pipe's ground with the
  top open.
  - Height, the up vector, the radial clamp and the joins are the pipe's.
  - The body leaves the surface when its height over the line, less `pipe_rim`, plus
    sin(22.5 degrees) of the radius (tapered over a join) reaches zero. The test applies on the whole
    span, not only over a join.
  - Gravity acts along the road's up everywhere, also over a join, where the pipe blends it.
  - The constant sin(22.5 degrees) is part of the rule, not a parameter; `pipe_rim` is the same
    record field as the pipe's.
- **Joins:** a tube span tapers into an open neighbour over a join.
  - The share `s` is the span's arc-length share over the join's `scale`: 0 at the seam, 1 where
    the tube is whole.
  - In a pipe join, the road plane lifted by `s` times the radius takes over where it is
    higher. Off-road starts `pipe_rim` above the line, and the up vectors blend toward the road's
    with `s`.
  - In a cylinder join, the radius tapers with `s`. Gravity blends from the road up through
    `(1 - s)^2`.
- **Road shapes:** `sh2z_vd_step2` gives `sh2z_vd_step`'s bits on road shapes (a host test
  checks 2,400 ticks).

Build: `sh2zam_vehicle_dynamics2.c` compiles `sh2zam_vehicle_dynamics.c` a second time with the
tube shapes and keeps only `sh2z_vd_step2`; the search, spline and line view come from the
vehicle-dynamics-1 unit. A program that never calls `sh2z_vd_step2` links none of it, and the
vehicle-dynamics-1 object code is unchanged by the tube code (checked with `cmp` under the CI
flags).

## Not covered

- `sh2z_vd_step` (without tubes) treats the half-pipe as an open road; `sh2z_vd_step2` models it.
- Vehicle-to-vehicle contact inside the model; the line-space field resolves contact and passes kicks.
- Real memory timing (cache, SDRAM, bus contention) and a whole-frame measurement on hardware.
