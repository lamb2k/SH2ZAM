# SH2ZAM Saturn library

The maintained overview, build commands, API scope and upstream credits are in the [root README](../README.md).

- [C API and numeric contracts](include/sh2zam/sh2zam.h)
- [C++11 convenience facade](include/sh2zam/sh2zam.hpp)
- [Runnable examples and Yaul diagnostic app](examples/README.md)
- [SH4ZAM capability comparison](PARITY.md)
- [Hardware integration and qualification](HARDWARE.md)
- [Exact batch-transform optimization and fallback](BATCH_TRANSFORMS.md)
- [Three-way Saturn transform benchmark](benchmarks/README.md)
- [Remaining work](PORTING_PLAN.md)

Both `cmake -S .` from the repository root and `cmake -S saturn` build the Saturn library. Batch transforms now select a specialized exact kernel when the matrix proves direct accumulation safe, with a general fallback for the remaining matrices. Full SH4ZAM API parity and physical-console performance qualification remain incomplete.
