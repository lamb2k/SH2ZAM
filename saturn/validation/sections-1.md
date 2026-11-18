# Validation record — 2026-11-18 (one section per hand-written entry and table, sections-1)

Every global SH-2 assembly entry is now assembled in its own `.text.<symbol>` section, the
vlc-1 tables in `.rodata.sh2zi_vlc8_run`, and every read-only C table is placed in
`.rodata.<name>` explicitly (`SH2ZI_RODATA`, `src/sh2zam_sections.h`). A consumer that
compiles with `-ffunction-sections -fdata-sections` and links with `-Wl,--gc-sections` gets
only the entries and tables it calls, transitively, whether it compiles the sources into its
program or links `libsh2zam.a`. No API, contract or result changed.
**No console or emulator run was done.** Evidence by tier:

- **Per-symbol identity** (`tools/symbol_identity.py`, every library object, main `65b81d5`
  against the branch, pinned Yaul GCC 13.2):
  - CI flags: 278 symbols byte-identical; 50 identical except the relocated words, which
    reach the same targets (C code whose `.text` lost the assembly placed before it); 2
    (`sh2z_shade_fog_gouraud`, `sh2z_shade_fog_gouraud_refs`) identical in every reachable
    instruction and branch target, with only PC-relative displacements changed.
  - Consumer flags (`-ffunction-sections -fdata-sections`): 327 byte-identical, 1
    relocated-words only (`sh2z_memo_put4`, its jump table), the same 2 as above.
  - The two fog entries shared their exits and bounded search by branching. An SH-2 branch
    cannot leave its section, so each entry now has its own copy (a gas macro instantiated
    per section): 64 bytes more when both are linked, no instruction more at run time.
- **Link probe** (`benchmarks/gc-probe`, CI): `sh2z_require_per_symbol_sections` passes for
  every source, and 20 one-call programs linked with `--gc-sections`, from objects and from
  the archive, keep exactly the expected entries and tables. Calling one entry, before and
  after: `sh2z_shade_fog_gouraud_refs` 5,144 to 4,148 bytes, `sh2z_memo_put4` 2,400 to 820,
  `sh2z_view_batch_divu` 4,856 to 3,512, `sh2z_shade_gouraud_approx` 2,276 to 932.
- **HOST:** every ctest passes under GCC and Clang Release with `SH2Z_STRICT=ON` and under
  ASan/UBSan, including two new header tests that include every public header in one
  translation unit, in both orders (`SH2Z_U30_ONE` is now defined identically in
  `unit30_experimental.h` and `trig12_experimental.h`, behind `#ifndef`).
- **MODELLED:** every capture's CPU-only cycles, instruction counts and checked outputs are
  unchanged (CI run
  [44256084857](https://github.com/lamb2k/SH2ZAM/actions/runs/44256084857) against main's
  [44250671574](https://github.com/lamb2k/SH2ZAM/actions/runs/44250671574)). The model ELFs'
  layouts changed: the cache model's set conflicts follow the image's addresses, so placed
  figures moved in the memo, vlc-2 (vlc-1 rows) and placement captures, which are re-recorded
  from that run. The moves are cache aliasing, not instruction timing: `best` and `typical`
  figures moved by -18% to +6% (the tier merge planner's short fixtures most), the
  other placements by -30% to +27%, and one memo C-path
  `worst` figure by +58%. The memo and vlc-2 guides and the placement README were updated
  from the new capture.
- **Not split:** the three pose test hooks (`sh2zi_pose_test_*`, about 220 bytes with their
  literals, called only by the pose model's bridge) stay in `sh2z_pose_from_f32`'s section,
  because they reach its helpers by `BSR`.
- **Not done:** console or emulator execution.
