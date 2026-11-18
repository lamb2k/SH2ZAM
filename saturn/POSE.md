# Model pose from binary32 state (experimental, contract tag `pose-1`)

[`contracts/pose_1.h`](include/sh2zam/contracts/pose_1.h), included by `view_experimental.h`, adds `sh2z_pose_from_f32`: it
builds a model's view (rotation and translation for `sh2z_view_*`), its camera position in
model space, its basis in Q1.14 and its plane-culling data from model state stored as IEEE 754
binary32 floats — a rotation basis, per-axis scales, a position and the camera position —
without any floating-point arithmetic. The SH-2 has no FPU, so a renderer that keeps its model
state as floats pays libgcc soft-float for every model every frame; this function reads each
float's bits and emulates the three binary32 operations involved exactly with integer
instructions. It came from a contributing Claude session's Saturn port work, which built the
same pose with float arithmetic; every test here uses synthetic poses.

**Evidence tiers: HOST and MODELLED only.** No emulator or console run of this function has
been done. Cycle numbers come from the Yabause SH-2 interpreter with flat RAM (no cache, no
wait states); they are not Saturn timings.

## API

| Item | What it is |
| --- | --- |
| `sh2z_pose_t` | `view` (an `sh2z_view_t`), `eye[3]` (model units), `basis[3][3]` (Q1.14), `eye_error`, `octant`, `cull_valid`. |
| `SH2Z_POSE_REVERSE_AXES` | Model axis `j` reads stored basis row `2 - j` and scale `2 - j` (state stored Z, Y, X). |
| `sh2z_pose_from_f32(out, base, rotation, basis, scale, position, eye, flags)` | Build the pose; false (with `*out` unchanged) when an input is out of range. |

The header is the contract. In short:

- **Exactly the float procedure.** The result equals, field for field and in success or
  failure, the reference procedure in the header evaluated with binary32 operations (round to
  nearest even, subnormals kept) and exact integer arithmetic elsewhere: the difference
  `position - eye`, the products `basis * scale` and the reciprocals `1 / scale` are each one
  binary32 operation, then truncated to fixed point; the sums are exact 64-bit sums rounded
  ties away from zero. Every output is therefore within **0 LSB** of that procedure and every
  vertex projected through `view` is identical. Against a double-precision model the
  projected vertices are within 1 pixel (tested, see below).
- **Inputs.** `rotation`: the camera's world-to-view rows, Q2.30, each in ±1,100,000,000.
  `basis`: finite, |value| ≤ 1.00001f. `scale`: in [0.0625, 1]. `position`, `eye`: any finite
  magnitude, with |fl(position − eye)| ≤ 4096 per axis. `base`: any valid view; only its
  focal, centre, near/far and clip fields are read (build it once per camera).
- **Outputs.** `view.r` = rotation × scaled basis (Q1.14), `view.t` (Q18.14); `eye` in whole
  model units on model axes; `basis[world][model]` unscaled, Q1.14; `octant` bit `j` when
  `eye[j] >= 0`; `eye_error = 2`; `cull_valid` when the basis is orthonormal to 512/2^30 per
  Gram entry and every |eye| ≤ 30000.
- **Edge cases.** NaN and infinity anywhere fail. A zero, subnormal, negative or over-1 scale
  fails. A subnormal basis value gives zero; subnormal and zero positions subtract as binary32
  does; −0 behaves as +0. An overflowing difference fails. A `view.r` outside int16 fails, as
  does a view that `sh2z_view_init` would refuse.
- **Aliasing.** Every input but `base` may overlap `*out`; `base` may be `&out->view`.

## How it is exact

Each float is split into sign, exponent and 24-bit significand. Two facts keep the emulation
short:

1. Rounding to 24 significant bits and then truncating at a coarser bit is one floor:
   `floor(RNE_p(x) / 2^f) = floor((x + 2^(p-1)) / 2^f)` for every `x >= 0` when `f > p`, ties
   included. The product and the reciprocal both truncate above their rounding bit, so each
   is one multiply (or one division) and one add.
2. A binary32 difference rounds at most once, on an aligned 32-bit sum whose leading bit is
   29, 30 or 31 whenever any bit can be lost. When the two operands share sign and exponent
   (the usual case for a model near the camera) the difference is exact (Sterbenz) and is the
   difference of the two bit patterns in units of one ulp.

The SH-2 has no barrel shifter: every variable shift is the high word of a 32 × 32-bit
`DMULU.L` with a power of two from one 145-entry table, indexed by a float exponent. The
reciprocal `floor(2^(47-g) / significand)` takes 21 `DIV1` steps. The 21 three-term dot
products use `MAC.L`; the rounded results come from `CMP/PZ` and two `ADDC`, and the
orthonormality test preloads `MACH:MACL` so that one 64-bit `SUBC` borrow decides each Gram
entry.

## Implementation

- **SH-2 path.** On SH-2 builds (without `SH2Z_FORCE_PORTABLE`), `sh2z_pose_from_f32` is
  hand-written SH-2 code in [`src/sh2zam_pose.c`](src/sh2zam_pose.c). It follows the GCC SH
  calling convention (arguments r4–r7 and the stack, result r0), preserves r8–r14, PR, MACH
  and MACL, and uses 188 bytes of stack. It needs no other symbol: no libc, no soft-float or
  other libgcc helper.
- **C path.** Every other build, and the SH-2 build when SR.S is set (`MAC.L` would then
  saturate at 48 bits), runs the portable C in the same file, which gives the same results.
- **Table.** 580 bytes of read-only data.

## Performance (MODELLED)

Whole calls from C in the Yabause interpreter, 2,000 visible poses per row (world positions to
±2^24, both axis orders, bases straddling the orthonormality tolerance):

| Row | Library (SR.S clear) | Library C path (SR.S set) | Float procedure (soft-float) | Speed-up |
| --- | ---: | ---: | ---: | ---: |
| one scale on all three axes | 1,440 (max 1,629) | 2,698 | 23,853 | 16.6× |
| scales 0.1, 0.125, 0.15 | 1,574 (max 1,759) | 2,832 | 23,879 | 15.2× |

Each distinct scale costs one exact reciprocal (about 64 cycles of `DIV1`); equal scales share
it. The float procedure also rebuilt its base view per call (1,355 modelled cycles,
`plainBase`); the library takes the base view as an input, built once per camera. Numbers and
the CI capture are in [`benchmarks/pose-model`](benchmarks/pose-model/README.md).

## Use

```c
#include "sh2zam/view_experimental.h"

/* Once per camera: any view with the right focal, centre, depth and clip. */
sh2z_view_t base;   /* e.g. from sh2z_view_init / sh2z_view_place at the origin */

/* Per model: Q2.30 camera rotation rows and the model's float state. */
sh2z_pose_t pose;
if(!sh2z_pose_from_f32(&pose,&base,camera_rotation_q30,model_basis,model_scale,
                       model_position,camera_eye,SH2Z_POSE_REVERSE_AXES)) {
    /* out of range: pose unchanged */
}
/* pose.view projects model vertices; pose.eye/octant/eye_error/cull_valid cull planes. */
```

## Evidence

- **HOST** ([`tests/test_pose.c`](tests/test_pose.c), against the independent binary32
  oracle [`tests/pose_oracle.h`](tests/pose_oracle.h), which evaluates the reference
  procedure with the host's float arithmetic and shares no code with the library): the
  reciprocal over all 33,554,433 accepted scales; the Q30 basis over 133,169,322 values; the
  model product over 53,370,921 values (392,144 exact rounding ties); the difference over
  6,010,000 pairs (341,974 ties); 1,500,000 random poses and 237,600 poses with a special
  float (zero, subnormal, NaN, infinity, bounds) in each input; argument and aliasing rules;
  a projection gate (199,955 poses, 3,934,362 on-screen points identical to the reference
  pose, at most 1 pixel from a double model); and 8 negative controls (the oracle without
  each binary32 rounding, floored instead of rounded outputs, a 511 tolerance, no
  intermediate R_30, the axis order ignored), each caught. GCC and Clang with
  `SH2Z_STRICT=ON` and ASan/UBSan.
- **MODELLED** ([`benchmarks/pose-model`](benchmarks/pose-model/README.md)): the compiled
  SH-2 path, the C path with SR.S set and the float procedure match the oracle on every field
  of every case before timing, and the internal difference, reciprocal and basis-value
  routines match it value by value; see [validation/pose-1.md](validation/pose-1.md).
- **Not done:** console or emulator execution, cache and memory timing, a whole frame.
