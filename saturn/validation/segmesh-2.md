# Validation record — 2026-11-19 (cluster bounds culling, segmesh-2)

`sh2z_segmesh_cull_prepare` and `sh2z_segmesh_cull` in `sh2zam/segmesh_experimental.h`
([guide](../SEGMESH.md)) classify up to 32 baked cluster bounds (a sphere and a normal cone,
20 bytes each) against one centre view. Integer only, with no divide. **The cluster path is not
exact:** its facing test holds only up to the projection's rounding of corners. **No console run was done in this repository.** Evidence
by tier:

- **HOST:** `segmesh_cull_exact` passes 640,896 checks under GCC and Clang Release with `SH2Z_STRICT=ON` and under ASan/UBSan, over random views (reflecting ones
  included) and clusters. 74,409 bounds were culled and 85,591 kept. Every culled bound's faces
  are either:
  - rejected by the view batch's outcodes;
  - emptied by the guard clip;
  - or have a non-negative exact projected area.

  One face was negative only after the projection's rounding.
- **EMULATOR, measured by a caller:** on one instrumented road renderer image the cluster path
  skipped 0.08 faces per frame that the per-face path draws. That renderer keeps cluster culling
  off.
- **SH-2 build:** the source compiles with the CI flags and leaves nothing in plain sections
  under the consumer flags. The object is 2,128 bytes (`-m2 -mb -O2`); it needs `___movmem`,
  `___ashlsi3_r0` and `___ashiftrt_r4_9`.
- **Not done:** an instruction-model capture and console execution.
