# Validation record — 2026-11-22 (search stop for a body far off the line, vehicle-dynamics-6)

`SH2Z_VD_LOCATE_FAR` and `SH2Z_VD_LOCATE_FAR_RUN` in `sh2zam/vehicle_dynamics_experimental.h`
([guide](../VEHICLE_DYNAMICS.md#search-stop-for-a-body-far-off-the-line-vehicle-dynamics-6-sh2z_vd_locate_far))
stop the closest-point search of `sh2z_vd_step` and `sh2z_vd_step2` when its residual stays above FAR units for RUN
passes in a row. Off (the default, FAR 0), nothing changes. **No console run was done in this repository.**
Evidence by tier:

- **Default build unchanged (SH-2, `sh2eb-elf-gcc -m2 -mb -O2 -ffreestanding -std=c11 -Wall -Wextra -Werror`):** all 51
  objects of `saturn/src` (disassembly, relocations, section contents) are identical to main `0aba730`. With FAR 128
  the four vehicle objects compile with no new undefined symbol and no `memcpy`, `memset` or `memmove`;
  `sh2zam_vehicle_dynamics.o` grows from 24,148 to 24,196 bytes of text, `sh2zam_vehicle_contact.o` (which compiles the
  search again) from 23,800 to 24,040; the two tube units do not change (they call the search of the first).
- **HOST, library test:** `vehicle_locate_exact` passes 363,897 checks under GCC and Clang with `-Wall -Wextra -Wconversion
  -Wshadow -Wpedantic -Werror`, under ASan and UBSan, and through the CMake `SH2Z_STRICT=ON` build. The model is built a
  second time with FAR 128 and RUN 8 and compared with the library over 120,000 randomised searches (a stadium road and
  five rough roads, targets from level with the line to 700 units off it):
  - 119,221 are not stopped and equal the library word for word (the whole body), 64,519 of them with the residual above
    128 units at some pass (an overshoot that came back); none had 8 such passes in a row;
  - 779 are stopped (27 of them only after more than 20 passes); each had exactly 8 passes in a row above 128 units at
    the stop, ran 8 to 99 passes, stopped once and kept the target as the body position;
  - 318 of the unstopped searches stayed above 128 units for 3 to 7 passes in a row and came back.
- **Controls (9 of 9 caught, `tests/mutate_vehicle_locate.sh`):** a stop of 3 units over 1 pass and one of 2 units over 8
  passes (the test's `B_FAR` and `B_RUN`); the run test off by one; the run not reset when the residual falls; the squared
  threshold shifted by one bit; the stop returning -1; the run starting from -1; the hook never called; the stop testing
  against the tolerance. The unmutated source passes.
- **Caller qualification (HOST and EMULATOR, by the port's engine lane; the library's tests do not depend on it).** Fire
  Field's jump (a body 380 units over the line) ran the search through all 99 passes on 20 ticks of the race (18,134
  passes in the whole race; 16,126 with FAR 128 and RUN 8). `fzx-phys`, the port's HOST driver, over all 24 courses at
  FAR 128, RUN 8, against the same build at FAR 0:
  - 23 courses: no search stopped, the trace of every tick and the 256-tick checkpoint hashes byte for byte equal, finish
    equal to the millisecond (or the same retirement tick);
  - the jump course: 25 stops, finish 100,490 ms against 100,521 (-0.03%), lap starts 0 to 3 ticks apart;
  - 63 driver settings (gain, look, lateral target) on the jump course, 22 of which finish at FAR 0: at FAR 128, RUN 8 all
    22 finish within 0.5% (FAR 96, RUN 8: 22; FAR 112, RUN 9: 22; FAR 192, RUN 8: 19; a plain pass limit of 12, 20, 30 or 40:
    21 of 22, and each of those limits stops a search on 3 to 6 of the other 23 courses);
  - the control (FAR 3, RUN 1) stops 993 to 11,999 searches on each of the other 23 courses and changes every trace;
  - the outcome depends on where the stop ends the search: with the default driver, RUN 4 to 7 at FAR 128, and several
    settings from FAR 144 to 176, ended the first jump before the search had settled into its circle and lost the finish.
    FAR 128 with RUN 8 is in the range that holds on the 63-driver set; a retune re-runs it.
  - EMULATOR (Ymir, timing MODELLED, the engine image with the stop on, the course's driver): 0 intervals over two fields in
    3,151 (the same image with FAR 0: 5 of 3,148, worst 4F), critical-path maximum 31.17 ms against 66.57, the 26 HOST
    checkpoints equal; Mute City 1: 27 of 27 checkpoints equal, 0 over two.
- **Not done:** a console run; a Saturn timing capture of the library call (the model suite is unchanged, as the default
  object is).
