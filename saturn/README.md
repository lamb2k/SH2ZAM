# SH2ZAM Saturn library

The maintained overview, build commands, API scope and upstream credits are in the [root README](../README.md).

- [C API and numeric contracts](include/sh2zam/sh2zam.h)
- [C++11 convenience facade](include/sh2zam/sh2zam.hpp)
- [Runnable examples and Yaul diagnostic app](examples/README.md)
- [SH4ZAM capability comparison](PARITY.md)
- [Hardware integration and qualification](HARDWARE.md)
- [Remaining work](PORTING_PLAN.md)

Both `cmake -S .` from the repository root and `cmake -S saturn` build the Saturn library. This is a correctness-first fixed-point foundation, not full SH4ZAM API parity or a console-benchmarked performance release.
