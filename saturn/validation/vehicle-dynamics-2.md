# Validation record — 2026-11-20 (track-bound vehicle dynamics with tubes, vehicle-dynamics-2)

`sh2z_vd_step2` ([guide](../VEHICLE_DYNAMICS.md#tubes-vehicle-dynamics-2)) adds the pipe and
cylinder road shapes and their joins to the vehicle-dynamics-1 tick. **No console run was done
in this repository.** Evidence by tier:

- **HOST:** `vehicle_dynamics2_exact` passes 52 checks under GCC and Clang Release with
  `SH2Z_STRICT=ON` and under ASan/UBSan. It covers:
  - road shapes: `sh2z_vd_step2` equals `sh2z_vd_step` bit for bit over 2,400 ticks with walls,
    boosts, leans and steering;
  - a pipe: the body holds the road's settled hover height on the floor, the side wall and the
    ceiling, with up toward the line. Past the radius it is put back on it with no speed into
    the surface;
  - a cylinder: the same on top, at the side and underneath, with up away from the line. It
    leaves the surface beyond radius plus reach and not before; below the radius it is put back;
  - joins: driving from open road into a joined pipe and out again, the worst height is 85
    against 257 without the joins;
  - a golden state hash of the join drive.

  Each property has a negative control: the vehicle-dynamics-1 road rule on the same tube, or the
  tube without its joins.
- **SH-2 build:** both units compile with the CI flags (`-m2 -mb -O2 -Werror`). All 44 library
  objects of main are byte-identical (objdump) with this branch, `sh2zam_vehicle_dynamics.o`
  included. `sh2zam_vehicle_dynamics2.o` is 22,168 bytes of text. It needs the
  vehicle-dynamics-1 unit's `sh2z_vd_locate` and `sh2z_vd_lap_distance`, the divider, the
  unit30 helpers and `___ashiftrt_r4_14`; no memcpy or memset.
- **MODELLED:** the `vehicle-dynamics-model` suite runs `sh2z_vd_step2`'s SH-2 object code in the
  pinned Yabause interpreter: flat RAM, no caches or wait states, not Saturn timing. It first
  checks 2,440 ticks against the host build word for word: walled road (also equal to
  `sh2z_vd_step` on the host), a pipe and a cylinder on three sides each, and a joined-pipe
  drive. Mean cost: 8,942.8 cycles per tick (median 8,743, max 16,317). The capture is CI run
  44675995095, ELF SHA-256 `82234b60...`. The vehicle-dynamics-1 rows of the same suite are unchanged.
- **Caller qualification (recorded by the caller, HOST):** a racing game's pipe and cylinder
  courses against its original code. Reference-seeded windows across every tube span and join,
  and whole races, agree within its bands.
- **Not done:** the half-pipe shape, console execution and any frame measurement.
