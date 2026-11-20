# Vehicle-dynamics instruction model

Executes the compiled big-endian SH-2 object code of `sh2z_vd_step`, `sh2z_vd_locate` and
`sh2z_vd_lap_distance` ([`vehicle_dynamics_experimental.h`](../../include/sh2zam/vehicle_dynamics_experimental.h),
contract tag `vehicle-dynamics-1`) and of `sh2z_vd_step2` (contract tag `vehicle-dynamics-2`) in the same unmodified Yabause SH-2 interpreter as
[`sh2-model`](../sh2-model/README.md) (pinned commit `7e38821dbac265490f115e163c523a939acda759`,
flat RAM, no caches, no wait states, no bus contention, the divider model of
`sh2-model/divider_model.h`). **Cycle numbers are MODELLED. They are not Saturn measurements, not
whole-frame timings, and not emulator-system timings.** No console or emulator run of these
functions has been done.

## What runs

- [`bridge.c`](bridge.c) (MIT) is compiled for `-m2 -mb` and linked with the library objects and
  `libgcc` by the integer model's [`model.ld`](../integer-model/model.ld). It holds the track
  records, the vehicle and rule constants, the body, a probe body and the pad in target RAM.
  Entries: `set_track`, `step` (`sh2z_vd_step` on the body: one tick), `locate`
  (`sh2z_vd_locate` on the probe, 100 iterations, tolerance 2^32) and `lap_distance`
  (`sh2z_vd_lap_distance` on the probe). The measured cost includes each entry's thin wrapper.
- [`vehicle_dynamics_harness.c`](../sh2-model/vehicle_dynamics_harness.c) (GPL-2.0-or-later) builds
  a synthetic stadium on the host: 16 nodes, two straights joined by half circles, walled, with a
  boost plate, a recharge strip, a mine on the line and a launch ramp. It writes the records into
  target RAM after checking that every record has the host layout, calls the entries with the SH-2
  calling convention and checks the stack, the callee-saved registers and SR.S after every call,
  reusing `sh2-model/harness.c`'s loader, call wrapper and divider model.
- **Verification before timing.** The contract promises the same bits on the host and the SH-2,
  so the oracle is the host build of the same source. Five input scripts run from the start line:
  - `straight`: ACCEL held for 600 ticks;
  - `steer_boost`: 900 ticks with boosts, lean bursts and a stick sweep;
  - `wall`: 400 ticks, steering into the right-hand wall and on;
  - `airborne`: 300 ticks from 300 units above the road;
  - `race`: a line-holding driver over three laps to the finish.

  Every other tick runs with SR.S set. After every tick, the step's return and all 99 words of
  the SH-2 body must equal the host body. Every fourth tick probes `sh2z_vd_locate` from a copy
  of the body: toward a point within ±600 units, or one time in 16 toward a point 9,000 units away,
  which must be reported off the line. The result and every probe word must match the host, and
  so must `sh2z_vd_lap_distance` after each successful search. `coverage` counts the ticks
  that were airborne, on a wall, launched, mine-hit, plate-boosted, boosting, recharging,
  spinning out or falling, and the race's finishing tick. Any mismatch exits non-zero before any
  timing is printed.
- **vehicle-dynamics-2.** After those scripts, the bridge's `set_track2` and `step2` entries run
  `sh2z_vd_step2` on the stadium of `tests/test_vehicle_dynamics2.c` (300-unit tubes):
  - `road`: the walled stadium for 600 ticks with boosts, lean bursts and a stick sweep. The host
    step2 body must also equal `sh2z_vd_step`'s on every tick.
  - `pipe_bottom`, `pipe_wall`, `pipe_ceiling`: a pipe on spans 0-3, 240 ticks of ACCEL on each side.
  - `cylinder_top`, `cylinder_side`, `cylinder_bottom`: the same on a cylinder.
  - `pipe_join`: a pipe on spans 1-2 that joins road at both ends, 400 ticks from the road through
    the pipe and out.

  Every other tick runs with SR.S set. Every tick's return and all 99 body words must equal the
  host build's. `step2Verified` counts the checks, `step2Rows` splits the cost by script and
  `step2` is every step2 tick's whole-call cycles.
- **Cost.** `step` is every tick's whole-call cycles across all scripts; `rows` splits them by
  script. `locate` covers searches that succeed; `locateOffLine` covers searches that report the
  target off the line; `lapDistance` is one call after each successful search.
  `VEHICLE_DYNAMICS_LOCATE_EVERY` (default 4) sets the probe interval.

## Reproduce

CI builds the ELF with the Yaul project's pinned toolchain image (see
`.github/workflows/sh2-cross.yml`) and runs the model on the Linux runner:

```sh
SH2Z_CC=$CC SH2Z_OBJDUMP=$OBJDUMP SH2Z_NM=$NM \
  bash saturn/benchmarks/vehicle-dynamics-model/ci.sh build   # after build/sh2/libsh2zam.a
bash saturn/benchmarks/vehicle-dynamics-model/ci.sh model     # writes build/sh2-model/vehicle-dynamics-results.json
```

[`results.json`](results.json) is the committed capture. It records the source commit, the ELF
SHA-256, the toolchain and the Yabause commit. It is the `vehicle-dynamics-results.json` from the
`sh2-compile-evidence` artifact of the CI run named in its `evidenceUrl`. Every CI run uploads a
fresh capture there.

## Not covered

Real memory timing (cache, SDRAM, bus arbitration with the second CPU or DMA), the multiplier's
and divider's real latency and load-use interlocks, interrupt latency, and the effect on a whole
frame. A caller's adoption decision needs a measurement of its own game loop on the target.
