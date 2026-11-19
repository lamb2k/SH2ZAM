# Validation record — 2026-11-19 (segmented strip mesh visibility pick, segmesh-1)

`sh2z_segmesh_pick` in `sh2zam/segmesh_experimental.h` ([guide](../SEGMESH.md)) classifies a
closed ring of mesh segments against four fixed-point clip rows without dividing, groups the
visible segments into runs of similar depth and returns the runs furthest first. Integer
arithmetic only. **No console or emulator run was done in this repository.** Evidence by tier:

- **HOST:** `segmesh_exact` passes 1,098,800 checks under GCC and Clang Release with
  `SH2Z_STRICT=ON` and under ASan/UBSan, against an independent oracle compared field by field:
  argument and contract rejection, the full-int16 contract corner, nothing visible, everything
  visible with and without the floor, depth steps at the slope and one past it, 400 camera
  positions around a wobbling ring, 20,000 random rings and rows, and pool overflow, which
  writes nothing past `max_runs`. Mutation controls on tie order, the slope boundary and the
  far boundary all fail. One mutant is equivalent and is explained in the guide.
- **SH-2 build:** the source compiles with the CI flags and passes
  `sh2z_require_per_symbol_sections`. The object is 1,376 bytes (`-m2 -mb -O2`).
- **Not done:** an instruction-model capture and cycle figures, console or emulator
  execution, and any measurement of a frame.
