# Validation record — 2026-11-19 (vehicle dynamics with contact, vehicle-dynamics-3)

`sh2z_vd_step_contact` and `sh2z_vd_step2_contact` (tube shapes) in
`sh2zam/vehicle_dynamics_experimental.h` ([guide](../BODY_CONTACT.md))
feeds one tick's contact (push, damage, regain, hit) into the vehicle-dynamics-1 tick.
**No console run was done in this repository.** Evidence by tier:

- **vehicle-dynamics-1 unchanged:** the step's source gains a contact hook that is empty in its own
  file; `sh2zam_vehicle_contact.c` builds the same source a second time with the hook set. The
  SH-2 object `sh2zam_vehicle_dynamics.o` (`-m2 -mb -O2`, with and without
  `-ffunction-sections`) is byte-identical to main `93b3774`'s.
- **HOST:** `vehicle_contact_exact` passes 4,008 checks under GCC and Clang Release with
  `SH2Z_STRICT=ON` and under ASan/UBSan, on the vehicle-dynamics-1 test stadium:
  - with a zero contact, 4,000 ticks of a scripted drive (walls, side and spin attacks, boosts)
    give the same body as `sh2z_vd_step` every tick;
  - a push lands in that tick's velocity (within 2 LSB) and position;
  - damage takes energy, marks the tick (no grip), and does not spin out above zero;
  - damage past zero spins out before the pad is read (the pad then changes nothing);
  - a hit takes grip away for `contact_grip` ticks from the same tick;
  - the regain stops at the maximum and does nothing while spinning out.

  Controls: the push given as a velocity change before the tick fails the push claim; damage taken
  before the tick's resets fails the damage-mark claim.
- **SH-2 build:** `sh2zam_vehicle_contact.o` compiles with the CI flags; 25,788 bytes (the second
  build of the model). A caller built with `-ffunction-sections` and linked with `--gc-sections`
  keeps only the step it calls.
- **Tubes (`sh2z_vd_step2_contact`, `sh2zam_vehicle_contact2.c`, the vehicle-dynamics-2 tick built
  with the contact hook):** with a zero contact, 3,000 ticks driven on a pipe's floor give
  `sh2z_vd_step2`'s body every tick; damage and the hit mark land as on the road; on road shapes,
  3,000 ticks with a stream of pushes, damage, hits and regains give `sh2z_vd_step_contact`'s body bit
  for bit. In the pipe the radial projection after the integration keeps 0.60 to 1.00 of a push
  along the tube (reported; its sign is the claim). `sh2zam_vehicle_dynamics.o` and
  `sh2zam_vehicle_dynamics2.o` are byte-identical to main `accd505`; `sh2zam_vehicle_contact2.o` is
  24,464 bytes.
- **Not done:** an instruction-model capture of `sh2z_vd_step_contact`, console execution and any
  frame measurement.
