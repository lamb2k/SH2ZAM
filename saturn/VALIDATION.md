# Validation record — 2026-11-13

Implementation tested: `fbb6f3d67554316d13a3219ace3a6c3eae71c7b9`. Later documentation/image-pin commits do not change these math sources.

## Completed

- [Host CI run 43465864295](https://github.com/lamb2k/SH2ZAM/actions/runs/43465864295): GCC Release, Clang Release, and Clang Debug with AddressSanitizer/UndefinedBehaviorSanitizer all passed. Each executes seven CTest targets. Release builds also passed standalone-library build and installed-package consumer checks.
- `foundation_oracles`: 1,073,727 executed checks per run. Includes 50,000 randomized scalar samples, 20,000 vector samples, 1,000 matrix products, full 65,536-angle sweep, complete Q16.16 asin input grid in [-1,1], overflow/cancellation/invalid/aliasing regressions. Counts describe checks, not line or branch coverage.
- Local GCC 14.2 and Clang 17 builds passed, including Clang ASan/UBSan. The local full-turn sine sweep differed from the rounded host reference by at most one Q16.16 raw unit; round-tripping the library's own sin/cos through atan2 differed by zero angle units. These observations are not global error proofs or hardware timings.
- [SH-2 CI run 43465864349](https://github.com/lamb2k/SH2ZAM/actions/runs/43465864349): all five library C sources and the freestanding examples compiled with `sh2eb-elf-gcc 13.2.0`, `-m2 -mb -O2 -ffreestanding`; ELF headers identify big-endian SH-2 and disassembly contains `dmuls.l`.
- The same SH-2 job linked the Yaul diagnostic app and generated `sh2zam-smoke.elf`, `.bin`, `.iso` and `.cue`. It emitted RWX LOAD-segment linker warnings; these are recorded, not hidden or claimed resolved.
- [Build evidence artifact](https://github.com/lamb2k/SH2ZAM/actions/runs/43465864349/artifacts/14058036876) contains the archive, compiler/image metadata, ELF/disassembly/undefined-symbol reports and diagnostic build outputs. Artifacts may expire.

Toolchain image used: `ijacquez/yaul@sha256:b6d272204767cd0a734139dd3a2ab298ef5912fb7b7224b78e6d9fac1a7825a6`. The cross workflow is pinned to this observed digest for repeatability.

## Not completed / not claimed

No emulator or physical Saturn execution, hardware timing, master/slave concurrency stress, ISR register-preservation audit, or production game integration has been performed here. The diagnostic mask was zero on the host; no console mask is reported. No superiority to Yaul, SaturnMath++ or existing game code is asserted.

## Observed runtime helpers

The archive references integer/runtime helpers such as `___divdi3`, `___ashlsi3_r0`, `___lshrsi3_r0`, `___movmem` and `___udivsi3`. Yaul successfully resolves the demo's dependencies. A static archive alone does not prove that a different SDK's runtime or ABI is compatible. Hardware DIVU is deliberately not enabled.
