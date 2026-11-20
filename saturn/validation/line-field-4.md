# Validation record — 2026-11-20 (line field surface zones, tube nodes and final dash, line-field-4)

`sh2zam/line_field_experimental.h` ([guide](../LINE_FOLLOWER.md)) adds the line's surface to the field
parameters (`surface`: zones and tube nodes) and a final dash (`dash_from`, `dash_pace`). NULL and 0 are
line-field-3. **No console run was done in this repository.** Evidence by tier:

- **HOST:** `line_field_exact` passes 19,316,790 checks under GCC and Clang (`-O2 -Wall -Wextra -Werror
  -pedantic`, ASan and UBSan):
  - an empty surface, and one whose zone and tubes no agent reaches, give the pinned line-field-1 hashes
    on six seeds;
  - a plate boosts an agent inside its band at no energy cost, and not one outside it;
  - rough ground damps the target speed by `rough_keep` (within one unit), not while boosting; low grip
    steers back more slowly; a recharge strip refills `recharge` per tick up to the maximum;
  - an agent asked to run 250 units off the line in a tube of radius 100 goes past the radius, stays
    within half the round and is never charged at an edge on a step from a tube node (leaving it onto a
    narrower road included); the control without the wrap mask never passes the half width; across the
    seam it takes the short way round;
  - random races of 30 with tubes and level of detail keep tube laterals within half the local round,
    road laterals within the half width, energies in range and every place held once;
  - the final dash leaves the laps before it unchanged and shortens the last.
- **Mutation controls (8 of 8 caught):** a plate that costs energy, tubes ignored, rough ignored, low grip
  ignored, recharge ignored, the dash ignored, steering the long way round, and no clamp on leaving a tube.
- **Consumer (not this repository):** a Saturn racing engine's rival bands (6 seeds, its original code with
  the course's surface set up against the native field) pass 14 of 14 on six courses, two of them with
  pipes and cylinders.
- **Not done:** an instruction-model capture, console execution and any frame measurement.
