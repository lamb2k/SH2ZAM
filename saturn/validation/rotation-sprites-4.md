# Validation record — 2026-11-20 (rotation sprites: lookup-table COLR alignment in one-cell emission, rotation-sprites-4)

rotation-sprites-4 changes one rule in `sh2zam/rotation_sprite_experimental.h`
([guide](../ROTATION_SPRITES.md)): `sh2z_rspr_emit_cell` refuses, writing nothing, a lookup-table
colour mode (pmod bits 5..3 = 1) with `colr & 3` nonzero, because VDP1 ignores those two bits and
would read the table from the 32-byte address below. Every other input gives the rotation-sprites-3
output. **No console run was done in this repository.** Evidence by tier:

- **HOST:** `rotation_sprite3_exact` passes under GCC and Clang Release with `SH2Z_STRICT=ON` and
  under ASan/UBSan:
  - over 400,000 random cells and places, 145,585 commands and 217,055 rejections equal
    `sh2z_rspr_emit_valid` plus the caller's COLR and SRCA; the 37,360 misaligned lookup-table
    cases are refused with the command untouched;
  - every colour mode (8) and COLR low-bit pattern (4) on one otherwise accepted placement: 29
    accepted with COLR written as given, 3 refused (lookup-table mode, low bits 1 to 3).
  - Control: with the check removed, `rotation_sprite3_exact` fails.
- **SH-2 build:** only `sh2zam_rotation_sprite3.o` changes; `sh2z_rspr_emit` and
  `sh2z_rspr_emit_valid` keep their objects.
- **Not done:** an instruction-model capture, console execution and any frame measurement.
