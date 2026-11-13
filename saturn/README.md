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
