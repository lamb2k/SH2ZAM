# Validation record — 2026-11-19 (rotation sprite sets: select, place, emit, world pose, rotation-sprites-2)

`sh2zam/rotation_sprite_experimental.h` ([guide](../ROTATION_SPRITES.md)) draws a 3D object as
one VDP1 distorted sprite picked from a baked yaw x pitch set:
- `sh2z_rspr_valid`, `sh2z_rspr_select`, `sh2z_rspr_place` and `sh2z_rspr_emit`;
- the validated forms `sh2z_rspr_emit_valid` and `sh2z_rspr_pose`, the world-pose fast path with
  `sh2z_rspr_camera_scale`.

rotation-sprites-2 supersedes the unpublished rotation-sprites-1. Placement keeps the sub-pixel
fraction, and emission rounds each corner once, so an object held still no longer steps a pixel.
Integer only. **No console run was done in this repository.** Evidence by tier:

- **HOST:** `rotation_sprite_exact` passes under GCC and Clang Release with `SH2Z_STRICT=ON` and under ASan/UBSan:
  - selection against a floating-point reference over the whole sphere: 56,591 mirrored and
    58,584 full-set directions, with exact ties skipped;
  - placement and corners against references: 1,352 cases;
  - a still object: 1,617 poses with 0 corner steps;
  - the world pose against the general path: 20,000 poses, with 1 cell difference at a bin
    edge;
  - command words, mirroring and every rejection path.
- **SH-2 build:** the source compiles with the CI flags and leaves nothing in plain sections
  under the consumer flags. The object is 3,472 bytes (`-m2 -mb -O2`). It needs `___divdi3`,
  `___ashiftrt_r4_14`, `___ashiftrt_r4_30` and `sh2z_divu_div64`.
- **Merge train 4:** `sh2z_rspr_pose` copied its 4-byte pick with a struct assignment, which GCC
  compiled to a `memcpy` call. It now stores the fields one by one. Only that function's code
  changed (463 to 457 instructions), and the host checks above pass unchanged.
- **Not done:** an instruction-model capture, console execution and any frame measurement.
