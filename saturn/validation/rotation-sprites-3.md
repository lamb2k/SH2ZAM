# Validation record — 2026-11-19 (rotation sprites: yaw climb, pose from terms, one-cell emission, rotation-sprites-3)

rotation-sprites-3 adds faster forms to `sh2zam/rotation_sprite_experimental.h`
([guide](../ROTATION_SPRITES.md)), each with output identical to a rotation-sprites-2 function:
- `sh2z_rspr_yaw_convex` and `sh2z_rspr_yaw_climb`;
- `sh2z_rspr_pose_terms`, the world pose from a caller's camera-space terms;
- `sh2z_rspr_emit_cell`.

**No console run was done in this repository.** Evidence by tier:

- **HOST:** `rotation_sprite3_exact` passes under GCC and Clang Release with `SH2Z_STRICT=ON` and under ASan/UBSan, comparing each form bit for bit against the
  function it replaces:
  - 16,525,250 climbs equal the scan of `sh2z_rspr_select`. Two controls (no tie rule; climbing one
    way) differ 5,525 and 7,559,280 times, so the comparison can fail.
  - 194,970 poses placed and 45,030 rejected exactly as `sh2z_rspr_pose`, 120,000 of them also
    through the climb.
  - 160,696 commands and 239,304 rejections exactly as `sh2z_rspr_emit_valid`.
  - The convexity test covers convex, reversed, swapped, mirrored, star, collinear and dented
    tables.
- **SH-2 build:** the source compiles with the CI flags and leaves nothing in plain sections
  under the consumer flags. The object is 3,196 bytes (`-m2 -mb -O2`); it needs
  `___ashiftrt_r4_14`.
- **Not done:** an instruction-model capture, console execution and any frame measurement.
