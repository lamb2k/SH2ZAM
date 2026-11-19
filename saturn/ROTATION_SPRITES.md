# Rotation sprite sets (contract tag rotation-sprites-1)

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

## Validation

`saturn/tests/test_rotation_sprite.c` checks selection against a floating-point reference over the whole
sphere (mirrored and full sets, magnitudes up to 2e9), placement and corners against references (within one
pixel), the command words, mirroring and every rejection path. It passes under gcc and clang with
UBSan and ASan. Not yet run on Saturn hardware.
