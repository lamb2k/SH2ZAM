# SH2ZAM: Saturn-native experimental backend

This branch adds an independent, native SH-2 fixed-point math library under [`saturn/`](saturn/README.md), inspired by the capabilities of SH4ZAM. The **original SH4ZAM files, history, and MIT license are unchanged**. The project remains a fork of [Falco Girgis and SH4ZAM Contributors' original](https://github.com/gyrovorbis/sh4zam).

Use `cmake -S saturn -B build/sh2zam-host` and `ctest --test-dir build/sh2zam-host --output-on-failure` for host tests. See [`saturn/PORTING_PLAN.md`](saturn/PORTING_PLAN.md) for the hardware qualification path and remaining parity work.

**Status:** early working C11 math core with host tests; no verified Saturn-console benchmarks yet. Do not replace SH4ZAM/KallistiOS code in Dreamcast projects with this backend.
