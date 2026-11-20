# Validation records

Each contract's validation record is its own file in [`validation/`](validation/),
named after its contract tag. This index is generated: after adding or renaming
a record, run `python3 saturn/tools/validation_index.py --write`; host CI runs
`--check`. Records are listed by file name; each line gives the record's date
and scope.

- [dual-claim-1](validation/dual-claim-1.md): 2026-11-19 (two-CPU try-claim, dual-claim-1)
- [fixed-unit-1](validation/fixed-unit-1.md): 2026-11-17 (Q2.30 unit vectors, fixed-unit-1)
- [foundation-2026-11-13](validation/foundation-2026-11-13.md): 2026-11-13
- [integer-kernels-1](validation/integer-kernels-1.md): 2026-11-15 (integer kernels)
- [memo-1](validation/memo-1.md): 2026-11-17 (block memo kernels, memo-1)
- [pose-1](validation/pose-1.md): 2026-11-17 (model pose from binary32 state, pose-1)
- [scene-1](validation/scene-1.md): 2026-11-19 (scene sequencing, scene-1)
- [sections-1](validation/sections-1.md): 2026-11-18 (one section per hand-written entry and table, sections-1)
- [segmesh-1](validation/segmesh-1.md): 2026-11-19 (segmented strip mesh visibility pick, segmesh-1)
- [snd-1](validation/snd-1.md): 2026-11-19 (sound events and CD audio, snd-1)
- [sprite-cache-1](validation/sprite-cache-1.md): 2026-11-19 (sprite slot cache and cell4z decoder, sprite-cache-1)
- [trig12-1](validation/trig12-1.md): 2026-11-17 (12-bit-angle trig and divider quotients, trig12-1)
- [vdp1-banks-1](validation/vdp1-banks-1.md): 2026-11-19 (VDP1 double-bank bookkeeping, vdp1-banks-1)
- [vehicle-kernels-1](validation/vehicle-kernels-1.md): 2026-11-17 (vehicle kernels, vehicle-kernels-1)
- [view-camera-1](validation/view-camera-1.md): 2026-11-17 (view from a camera, view-camera-1)
- [view-merge-2](validation/view-merge-2.md): 2026-11-17 (tier texture-merge planner, view-merge-2)
- [view-rails-1](validation/view-rails-1.md): 2026-11-18 (strip rails in one call, view-rails-1)
- [vlc-1](validation/vlc-1.md): 2026-11-17 (variable-length decoding, vlc-1)
- [vlc-2](validation/vlc-2.md): 2026-11-18 (per-symbol variable-length decoding, vlc-2)
