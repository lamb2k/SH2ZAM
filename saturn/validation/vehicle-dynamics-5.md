# Validation record — 2026-11-22 (half-pipe road shape, vehicle-dynamics-5)

`sh2z_vd_step2` ([guide](../VEHICLE_DYNAMICS.md#tubes-vehicle-dynamics-2)) now models
`SH2Z_VD_SHAPE_HALF_PIPE`: a trough, the pipe's ground with the top open. **No console run was
done in this repository.** Evidence by tier:

- **HOST:** `vehicle_dynamics2_exact` passes 438 checks under GCC and Clang Release with
  `SH2Z_STRICT=ON` and under ASan/UBSan (52 earlier checks plus the half-pipe ones). It adds:
  - a rim oracle in double precision: a sweep of 360 body positions at half-degree steps round the
    trough; the body leaves the surface exactly when
    `road_up . disp - 23 + sin(22.5 deg) * radius >= 0`, with a 0.5 unit band around the boundary
    left out (no mismatch in the other positions);
  - the floor holds the hover height with up toward the line, and the wall at 60 degrees is held;
  - gravity acts along the road's up on the wall and through a join, where the pipe blends it
    (worst deviation 0 against 0.268 for the pipe on the same drive);
  - an off-road flag is taken back inside the rim and the radius, and kept above the rim and
    past the radius;
  - a drive from open road through a joined trough and out again, with a golden state hash.
- **Mutation controls (10 of 10 caught):** rim gain removed; rim constant removed; rim only over
  joins; gravity blended as the pipe's; the rim ignoring the join taper; radial clamp removed; road
  plane in the join removed; off-road flag never cleared; the radius test dropped; the tilt not
  blended in the join. One first survived (the radius test) and got its own check.
- **Negative controls:** the pipe holds the bodies the half-pipe lets go; the pipe's gravity follows
  the wall.
- **SH-2 build:** all 51 sources compile with the CI flags (`-m2 -mb -O2 -Werror`). 49 objects are
  byte-identical (objdump, and on the CI artifacts against main `46cd07f`), including `sh2zam_vehicle_dynamics.o`. The two that
  contain the tube build differ, as they must: `sh2zam_vehicle_dynamics2.o` goes from 22,168 to
  23,916 bytes of text and `sh2zam_vehicle_contact2.o` follows. No new undefined symbol; no memcpy or
  memset.
- **MODELLED:** the `vehicle-dynamics-model` suite gains `half_pipe_floor` (240 ticks) and
  `half_pipe_join` (400 ticks from road through the trough and out), each checked word for word
  (all 99 body words per tick) against the host build in the pinned Yabause interpreter. The CI
  capture (the commit that follows the first CI run on the merged head) gives a mean of 8,940.4
  cycles on the floor and 8,981.6 in the join, against 8,802.4 for the pipe floor and 8,954.4 for
  the pipe join. The earlier step2 scripts keep their ticks and results word for word, and their
  cycles move by about 0.2% at most because the step2 object changed (all-script mean 8,942.8 to
  8,942.4). Flat RAM, no caches or wait states, not Saturn timing.
- **Caller qualification (recorded by the caller, HOST):** see the caller's report; the library's own
  tests do not depend on it.
- **Known, not changed here:** `airborne_up` overflows a signed integer when the gravity up vector is
  exactly (0, -1, 0) (an off-road body upside down in a tube). That code predates this change and
  is shared by every tube shape; the half-pipe sweep stops at 179.5 degrees for that reason.
- **Not done:** console execution and any frame measurement.
