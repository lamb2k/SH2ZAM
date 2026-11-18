# Validation record — 2026-11-17 (model pose from binary32 state, pose-1)

`sh2z_pose_from_f32` in `sh2zam/contracts/pose_1.h` (through `sh2zam/view_experimental.h`, [guide](../POSE.md)) builds a model's view,
model-space eye, Q1.14 basis and plane-culling data from binary32 model state with no
floating-point arithmetic. It equals the header's binary32 reference procedure field for field
(0 LSB, identical projected vertices). Existing SH-2 objects are byte-identical to `198848f`.
**No console or emulator run was done.** Evidence by tier:

- **HOST:** `pose_exact` passes under GCC and Clang Release with `SH2Z_STRICT=ON` and under
  ASan/UBSan, against the independent binary32 oracle `tests/pose_oracle.h` (190,657,419
  checks):
  - **Operations:** the reciprocal for all 33,554,433 accepted scales; the Q30 basis value for
    133,169,322 values; the model product for 53,370,921 values over 96 scales (392,144 exact
    rounding ties); the difference for 6,010,000 pairs (341,974 exact ties).
  - **Whole poses:** 1,500,000 random poses (world positions to ±2^24, any orientation) and
    237,600 with a zero, subnormal, NaN, infinity or bound in each input; argument, flag and
    aliasing rules.
  - **Projection gate:** 199,955 poses, 3,934,362 on-screen points identical to the reference
    pose and at most 1 px from a double-precision model.
  - **Negative controls:** 8, all caught.
- **MODELLED:** the compiled SH-2 path (SR.S clear), the C path (SR.S set) and a soft-float
  control match the oracle on all 37 fields of 20,000 cases (13,075 accepted, each also run
  with the allowed aliasing), and the internal difference, reciprocal and basis-value routines
  on 320,000 values each, in the Yabause interpreter before timing
  ([results](../benchmarks/pose-model/results.json)). Modelled cycles per call: 1,440 with one
  scale on all three axes and 1,574 with three scales, against 23,853 and 23,879 for the
  binary32 procedure in soft-float (16.6× and 15.2×). Cycle numbers are modelled, not Saturn
  timings.
- **Not done:** console or emulator execution, cache and memory timing, a whole frame.
