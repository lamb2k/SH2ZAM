# SH2ZAM Saturn library

The maintained overview, build commands, API scope and upstream credits are in the [root README](../README.md).

- [C API and numeric contracts](include/sh2zam/sh2zam.h)
- [C++11 convenience facade](include/sh2zam/sh2zam.hpp)
- [Runnable examples and Yaul diagnostic app](examples/README.md)
- [SH4ZAM capability comparison](PARITY.md)
- [Hardware integration and qualification](HARDWARE.md)
- [Exact batch-transform optimization and fallback](BATCH_TRANSFORMS.md)
- [Exact coordinate-reuse API](COORDINATE_REUSE.md)
- [Five-kernel Saturn transform benchmark](benchmarks/README.md)
- [Executed SH-2 instruction-model results](benchmarks/sh2-model/README.md)
- [Exact math optimizations and SH-2 model results](MATH_OPTIMIZATIONS.md)
- [48-operation native math comparison and reproduction](benchmarks/math-model/README.md)
- [Exact rotation batches, static spatial queries and workload comparison](benchmarks/workload-model/README.md)
- [Remaining work](PORTING_PLAN.md)
- [How to add a kernel suite](CONTRIBUTING-KERNELS.md)

Both `cmake -S .` from the repository root and `cmake -S saturn` build the Saturn library. Batch transforms now select a specialized exact kernel when the matrix proves direct accumulation safe, with a general fallback for the remaining matrices. Full SH4ZAM API parity and physical-console performance qualification remain incomplete.

For one quaternion applied to many vectors, include
[`sh2zam/rotation_batch.h`](include/sh2zam/rotation_batch.h) and call
`sh2z_quat_rotate_vectors` to share normalization and matrix construction across
the batch. It preserves the scalar rotation's exact results and supports exact
in-place operation. The experimental `_mac` entry point adds the existing native
MAC.L qualification and fallback. Compare against an application that already
constructs one matrix per batch when evaluating adoption benefits.

For many sphere queries against unchanged AABBs, include
[`sh2zam/spatial.h`](include/sh2zam/spatial.h), build a caller-owned index, then
reuse it for distinct moving spheres. Queries return exact original-index hit
bitsets; source boxes and built nodes must remain unchanged until rebuilding.
At 512 boxes the index uses 32,736 bytes of nodes plus 2,048 bytes of temporary
build scratch. The [workload guide](benchmarks/workload-model/README.md) covers
buffer contracts, full setup costs, useful and adverse query patterns, and
comparisons against both prior public APIs and competent composed/inline code.
The workload suite uses `3374d77` as its prior baseline; the historical
48-operation suite retains its separate `10786b6` baseline.

## Experimental kernel guides

One line per guide, sorted by guide file name; a kernel lane adds its own line
(its guide, then its instruction model).

- [Experimental explicit SH-2 divider](DIVU.md)
- [Experimental SCU DSP access, batched affine transform, Q16 outer product](DSP.md)
- [Integer kernels: 68000 arithmetic, 2D box overlap, 16x16 cell placement](INTEGER_KERNELS.md); [Integer-kernel instruction model](benchmarks/integer-model/README.md)
- [Experimental racing-line follower and line field](LINE_FOLLOWER.md)
- [Experimental model pose from binary32 state, with no floating-point arithmetic](POSE.md); [pose instruction model](benchmarks/pose-model/README.md)
- [Experimental block memo kernels: block compare-and-copy, keyed write-through, copy until a marker, run diff](MEMO.md); [memo instruction model](benchmarks/memo-model/README.md)
- [Experimental rotation sprite sets: cell pick, placement, one VDP1 distorted sprite per object](ROTATION_SPRITES.md)
- [Experimental scene sequencer, arenas and press edges](SCENE.md)
- [Experimental segmented strip meshes: visibility pick, run ordering, cluster bounds culling](SEGMESH.md)
- [Experimental shading kernels: per-normal light, Gouraud packing, face culling, depth fog](SHADE.md); [Shading-kernel instruction model](benchmarks/shade-model/README.md)
- [Experimental fused fade and colour per piece, prepared fog, divides by constants](SHADE_FUSED.md); [Fused shade instruction model](benchmarks/shade-fused-model/README.md)
- [Experimental sound events, SCSP slot playback and CD audio](SND.md)
- [Experimental sprite slot cache and cell4z decoder](SPRITE_CACHE.md)
- [Experimental 12-bit-angle sin/cos/tan and inline divider quotients](TRIG12.md); [trig12 instruction model](benchmarks/trig12-model/README.md)
- [Experimental Q2.30 unit vectors: inline multiply, dot and cross, reciprocal square root, normalisation, basis orthonormalisation](UNIT30.md); [unit30 instruction model](benchmarks/unit30-model/README.md)
- [Experimental VDP1 double-bank bookkeeping](VDP1_BANKS.md)
- [Experimental track-bound vehicle dynamics: one hovering vehicle on a spline road, stepped in integers](VEHICLE_DYNAMICS.md); [Vehicle-dynamics instruction model](benchmarks/vehicle-dynamics-model/README.md)
- [Experimental vehicle kernels: screen facing, indexed light and view batch, approximate Gouraud](VEHICLE_KERNELS.md); [Vehicle-kernel instruction model](benchmarks/vehicle-model/README.md)
- [Experimental view batch: transform, project, clip codes, guard clip, texture-merge planners, final-coordinate truncation, source-order guard clip, fractional centre](VIEW_BATCH.md); [Merge-planner instruction model](benchmarks/merge-model/README.md)
- [Experimental strip rails in one call: both rails of a strip quad, view prepared once](VIEW_RAILS.md); [View-rails instruction model](benchmarks/rails-model/README.md)
- [Experimental variable-length decoding: bounded bit reader, length-limited canonical codes, SH-2 table decoder](VLC.md); [vlc instruction model](benchmarks/vlc-model/README.md)
- [Experimental per-symbol variable-length decoding: register-resident reader, per-symbol tables, packed table arena](VLC2.md); [vlc-2 instruction model](benchmarks/vlc2-model/README.md)
