# Rotation sprite sets (contract tags rotation-sprites-2 and -3)

Draw a 3D object as a pre-rendered view of itself. An offline baker renders the object from a grid of
viewing directions around its own up axis and packs the images as one set. At run time:

1. `sh2z_rspr_select` picks the cell nearest the viewing direction (camera position minus object origin,
   in the object's own frame).
2. `sh2z_rspr_place` projects the object origin with the object's model view (`sh2z_view_t`) and works out
   the screen scale and the on-screen direction of the object's up axis.
3. `sh2z_rspr_emit` writes one VDP1 distorted sprite command: the four corners scale the cell with depth
   and rotate it with the object's roll.

One command per object, whatever its polygon count. Header: `sh2zam/rotation_sprite_experimental.h`.

## Set layout

One contiguous block (big-endian in files; used in place on the SH-2), so it can be copied to VDP1 VRAM as is:

| Part | Size | Notes |
| --- | --- | --- |
| `sh2z_rspr_set_t` | 168 bytes | magic `RSP1`, total size, yaw/pitch counts, mirror flag, CLUT count and offset, pixels per model unit (Q16.16), yaw and pitch bin centres as {sin, cos} Q1.14 |
| `sh2z_rspr_cell_t` x cells | 16 bytes each | width (multiple of 8), height, origin pixel, pixel offset (multiple of 8), CLUT index; pitch-major |
| CLUTs | 32 bytes each | 16 RGB555 entries; index 0 is transparent |
| pixels | width x height / 2 per cell | 4-bit indices, left pixel in the high nibble |

## Conventions

- Object frame: +y up. yaw = atan2(x, z) of the viewing direction; pitch = atan2(y, horizontal length).
- A mirrored set stores yaw 0..180 degrees; negative x draws the |x| cell mirrored left to right. Exact only
  for objects whose image is symmetric about the y-z plane (decals may not be).
- Selection uses dot products with the bin centres, with no square root and no divide.
- Placement projects the origin exactly as `sh2z_view_batch` does and divides 64-bit integers four times.
- The origin keeps its sub-pixel position (`sh2z_rspr_place_t.frac`, 1/65536 pixel), and each corner is
  rounded once from it.

## Faster forms with identical output (rotation-sprites-3)

For a caller drawing many objects a frame. Each function equals a rotation-sprites-2 function bit for bit:

| Function | Equals | Saves |
| --- | --- | --- |
| `sh2z_rspr_yaw_convex`, `sh2z_rspr_yaw_climb` | the yaw rule of `sh2z_rspr_select` | the scan over every yaw bin, for a table that is a strictly convex polygon wound once around the origin; climbing from the object's previous bin usually reads three bins |
| `sh2z_rspr_pose_terms` | `sh2z_rspr_pose` | the camera-space terms a culling pass already has (`sh2z_rspr_terms_t`); each division is started before independent work and collected after it; optional climb |
| `sh2z_rspr_emit_cell` | `sh2z_rspr_emit_valid`, then the caller's COLR and SRCA | 64-bit corner products: each corner offset is below 2^26, so one 32 x 32 -> 64 multiply per coordinate; one pass |

Why the climb is exact: the products `yaw[k].sin*x + yaw[k].cos*z` are exact integers, the linear function of a
direction over the corners of a strictly convex polygon rises to one maximum (a corner, or one edge of two equal
corners) and falls to one minimum, so a climb that stops where neither neighbour is larger is on the maximum; an
equal neighbour with a lower index is taken, as the scan's tie rule. `sh2z_rspr_yaw_convex` checks every turn and
every step around the origin for one strict sign and the polar angle for one wrap; a mirrored set is refused.

## Contract history

- **rotation-sprites-4:** `sh2z_rspr_emit_cell` refuses, writing nothing, a lookup-table colour mode (pmod bits
  5..3 = 1) with `colr & 3` nonzero. VDP1 reads the table from `(COLR & ~3) * 8`, so such a command would draw
  with the 16 bytes before the table. Colour-bank modes keep their low COLR bits (they are bank code).
  `sh2z_rspr_emit` and `sh2z_rspr_emit_valid` only document the rule, since callers may emit with vram 0 and
  write their own COLR. Only `sh2z_rspr_emit_cell`'s object changes.
- **rotation-sprites-3:** adds the three faster forms above in their own unit (`sh2zam_rotation_sprite3.c`); the
  rotation-sprites-2 functions and objects are unchanged.
- **rotation-sprites-2:** the sub-pixel origin. rotation-sprites-1 floored the projected origin, then rounded
  each corner offset. An object held at a fixed screen position (a chase-camera player at the screen centre,
  with rounding noise either side of a pixel boundary) then stepped one whole pixel left and right from frame to
  frame. Now `x`/`y` stay the floor, `frac` carries the rest, and the corners round once. The struct grows by
  4 bytes; code that only passes it between `pose`/`place` and `emit` is unchanged.
- **rotation-sprites-1:** first contract.

## Validation

`saturn/tests/test_rotation_sprite.c` checks selection against a floating-point reference over the whole
sphere (mirrored and full sets, magnitudes up to 2e9), placement and corners against references (within half
a pixel of the sub-pixel origin), the command words, mirroring and every rejection path. A stillness test
holds an object a hair either side of a pixel boundary and requires identical corners. Its control: on the
rotation-sprites-1 code the same test finds 924 corner steps in 1,617 poses. It passes under gcc and clang with
UBSan and ASan. Not yet run on Saturn hardware.

`saturn/tests/test_rotation_sprite3.c` checks each faster form against its rotation-sprites-2 counterpart: the
convex check on convex, reversed, swapped, mirrored, star, collinear and dented tables; 16.5 million climbs on
four tables (dense, grid, random and edge-perpendicular tie directions, five starts each) equal the scan, and two
controls (no tie rule; climbing one way) differ; 240,000 poses (placed and rejected, with and without the climb)
equal `sh2z_rspr_pose`; 400,000 cells and places (widths 8..504, origins outside the cell, scales to 2^24 + 2,
any `up`) equal `sh2z_rspr_emit_valid`. gcc and clang strict, and ASan and UBSan, pass. Seven mutations of the new
unit (tie rule, winding check, rounding, mirror, projection numerator, corner range, near plane) are each caught.
