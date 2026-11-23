# Validation record — 2026-11-23 (search damping and flip stop, vehicle-dynamics-7)

`SH2Z_VD_LOCATE_DAMP` and `SH2Z_VD_LOCATE_FLIP` in `sh2zam/vehicle_dynamics_experimental.h`
([guide](../VEHICLE_DYNAMICS.md#search-damping-and-flip-stop-vehicle-dynamics-7-sh2z_vd_locate_damp-sh2z_vd_locate_flip))
change the closest-point search of `sh2z_vd_step` and `sh2z_vd_step2` for a body far off the line, in the two ways its
99 passes run under the `vehicle-dynamics-6` line. Off (the default, 0), nothing changes. **No console run was done in this
repository; the caller's numbers below are HOST and EMULATOR (MODELLED timing).** This is an approximation, not an exact change
(the end state of a changed search differs from the 99-pass one within the bounds below).

- **Default build unchanged (SH-2, `sh2eb-elf-gcc -m2 -mb -O2 -ffreestanding -std=c11 -Wall -Wextra -Werror`):** the objects
  of `sh2zam_vehicle_dynamics`, `sh2zam_vehicle_dynamics2` and `sh2zam_vehicle_contact` are identical to main `e83dae5` (`cmp`).
  With DAMP 8 and FLIP 3 (and FAR 128) `sh2zam_vehicle_dynamics.o` grows from 24,148 to 24,652 bytes of text, with no new
  undefined symbol and no `memcpy`, `memset` or `memmove`. The model suite is unchanged (the objects it links are identical,
  so its ELF and `results.json` are too; the CI model job confirms).
- **HOST, library test (`tests/test_vehicle_locate7.c`):** 506,265 checks pass under GCC and Clang with `-Wall -Wextra
  -Wconversion -Wshadow -Wpedantic -Werror` and under ASan and UBSan. The model is built a second time under private names with
  DAMP 8 and FLIP 3, the linked library (both off) being the oracle, over 120,000 randomised searches (a stadium and five rough
  roads, targets up to 2,000 units off the line):
  - 112,514 are searches neither change acted on and equal the library word for word; 5,501 were damped, 1,968 ended by the
    flip stop (2 both);
  - every damped step is not longer than the step it replaces and has its sign; none is taken before DAMP passes; a flip stop
    ends a call once, after 5 to 99 passes, on a pass of the parity the 99th pass has, with the body position the target;
  - the library converged (residual within 1 unit) within 700 units of the line in 340 of the changed searches: the changed
    search converged too in 339 and ended within 0.853 units of the library's end state; the one exception was a flip stop at
    40 passes that ended 12.5 units off the library's end state (the library closed in 59 more). The test allows 1 in 100;
  - where the library did not converge there is no right end state: the changed search converged in 5,157 of them; within 700
    units of the line its residual was larger by over 1 unit in 14 (the largest by 21.8 units); end states more than 1 unit
    apart in 2,400;
  - the return value differs in 19 searches, all of them where only the library reports -1 (off the line);
  - the set holds damped, flip-ended and untouched searches (the test requires 500, 100 and 10,000).
- **Controls (`tests/mutate_vehicle_locate7.sh`, 12 of 12 caught, the unmutated source passes):** a step twice as long; a
  reversed secant; a slope test the wrong way; damping from the second pass; a flip stop without the parity; without the
  return test; a flip stop returning -1; both hooks removed; builds with DAMP 1 and FLIP 1 and FLIP 2.
- **Caller qualification (HOST and EMULATOR, by the port's engine lane; not part of the library's tests).** Fire Field's gap:
  the default driver ran 48 passes on its worst tick, a line with kl 0.0044, kh 2.75, target 100 ran 99 passes on five ticks in
  a row (86 ticks over 24 passes). The earlier search is the baseline throughout:
  - `fzx-phys` over 325 driver settings (kl, kh, target) on that course: finishers 134 to 135 (lost 1, gained 2), the finishes
    that both complete within 0.10%, ticks over 12 passes 15,376 to 1,521, passes 2.885 M to 2.414 M, longest 99 to 83. The
    noise controls of the same set (the tolerance halved or doubled) lose 3 to 9 and gain 5 to 6;
  - the default driver's worst tick 48 to 22 passes, the 0.0044/2.75/100 line's 99 to 44; stick noise on the default driver
    (AR(1), 60 seeds each at two amplitudes): runs with a 99-pass search 56 and 54 to 0 and 0, the median of each run's longest
    search 99 to 34 and 23, the mean ticks over 24 passes 40.5 and 20.2 to 2.1 and 0.5, finishes 30 and 3 to 30 and 5;
  - the 24 courses with the gate's default drivers: 20 have identical checkpoint hashes; Fire Field finishes at 100,490 ms as
    before (its checkpoints differ), White Land 1 and Devils Forest 3 keep their finish (checkpoints differ), Big Blue 2
    finishes at 80,037 ms against 80,008 (+0.036%);
  - EMULATOR (Ymir, engine on the earlier prototype of the flip rule, which compared nodes only; the final rule is a subset
    of its stops, the engine gate on the merged pin repeats it): the 0.0044/2.75/100 line failed the cadence gate with the
    earlier search (10 intervals over two fields, worst 4F, critical maximum 57.06 ms) and passed with the change (0 over two,
    worst 2F, 22.25 ms), the engine equal to HOST at 26 of 26 checkpoints; the default driver was unchanged (critical maximum
    21.06 to 21.07 ms).
- **Not done:** a console run; a Saturn timing capture of the changed call (the model suite is unchanged, as the default object
  is); the third kind of long search (a residual that rises past the centre of curvature), which this tag leaves as it was
  (Port Town 1 and Red Canyon 1 keep their 55 and 57 passes).
