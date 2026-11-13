# SH2ZAM — Saturn implementation notes (experimental)

The **Saturn-native, fixed-point core** of [SH2ZAM](../README.md), an independent adaptation of the mathematical capabilities of [SH4ZAM](https://github.com/gyrovorbis/sh4zam). Full API parity and a validated Saturn release are not claimed. The inherited Dreamcast source baseline and MIT LICENSE are preserved; SH2ZAM builds from this directory.

**Upstream credit:** SH4ZAM by **Falco Girgis and SH4ZAM Contributors**, MIT-licensed. SH2ZAM draws on its public functional goals and API categories; it does not copy SH4-specific optimized routines. SH4ZAM's maintainers prefer upstream changes; please keep this fork synchronized and propose any applicable improvements upstream. [Original source](https://github.com/gyrovorbis/sh4zam), [MIT license](../LICENSE), [upstream agent/contribution guidance](https://github.com/gyrovorbis/sh4zam/blob/master/AGENTS.md).

**Additional research credit:** [SaturnMath++ by Roberto Daniel Alves Duarte](https://github.com/robertoduarte/SaturnMathPP) and [libyaul](https://github.com/yaul-org/libyaul) document useful Saturn math and toolchain patterns. No code from those repositories has been copied into this implementation.

## Why SH2ZAM differs

Dreamcast SH-4 has a hardware FPU and XMTRX register bank. Saturn's dual SH-2 CPUs have no FPU, so direct `float` API / XMTRX instruction parity would regress performance. SH2ZAM instead uses a deterministic, zero-allocation Q16.16 math core, unsigned 16-bit turn angles, and an SH-2 `dmuls.l` backend selected only for SH-2 builds. No global mutable data makes the routines safe to call from either CPU (ordinary application synchronization still required for shared buffers).

## Present features (prototype)

- Saturating Q16.16 add, subtract, multiply, divide, square root; host reference implementation.
- SH-2 signed 32×32-to-64 multiply path using `dmuls.l` + MACH/MACL extraction; **not yet compiled or tested on Saturn hardware**.
- 1 KiB quarter-wave sine/cosine table with linear interpolation (turn-based 16-bit angle).
- 2D/3D/4D vector dot products; 3D cross, length, normalization, scale/add.
- Row-major affine 3×4 and 4×4 matrices, transformations and alias-safe multiplication.
- Quaternion Hamilton multiplication and normalization.
- C11 implementation with C++ `extern "C"` interop, standalone CMake target `sh2zam`.

Not implemented: SH4ZAM's float API, XMTRX register bank, SIMD/FPU instructions, complex/FFT, advanced trig, optimized copy/move routines, all matrix layouts, C++ ergonomic facade. See `PORTING_PLAN.md`.

## Host build and correctness checks

```bash
cmake -S saturn -B build/sh2zam-host -DSH2Z_BUILD_TESTS=ON \
  -DSH2Z_FORCE_PORTABLE=ON
cmake --build build/sh2zam-host -j
ctest --test-dir build/sh2zam-host --output-on-failure
```

## SH-2 cross-build (libyaul-compatible toolchain)

Requires `sh2eb-elf-gcc` on PATH. If using an existing libyaul make-based game, the emitted `libsh2zam.a` can be linked alongside Yaul; SH2ZAM does **not** replace Yaul's hardware layer.

```bash
cmake -S saturn -B build/sh2zam-sh2 \
  -DCMAKE_TOOLCHAIN_FILE="$PWD/saturn/toolchains/yaul-sh2.cmake" \
  -DSH2Z_BUILD_TESTS=OFF
cmake --build build/sh2zam-sh2 -j
```

For a cross-build from a separate directory, specify the toolchain file as an absolute path if CMake cannot resolve it. The `-m2 -mb` flags target big-endian SH-2. Cross-compilation and console/emulator correctness are **still unverified**.

## Usage

```c
#include <sh2zam/sh2zam.h>
sh2z_vec3_t p = {sh2z_from_int(3), sh2z_from_int(4), 0};
sh2z_vec3_t unit = sh2z_vec3_normalize(p);
sh2z_mat34_t world = sh2z_mat34_identity();
world.m[0][3] = sh2z_from_int(10);
sh2z_vec3_t moved = sh2z_mat34_transform_point(&world, unit);
```

### Numeric contract

- `sh2z_fx_t` is **signed 16.16 fixed-point**, represented range −32768 to 32767 + 65535/65536. Addition/subtraction/multiplication/division saturate at 32-bit bounds; multiplication uses floor for negative fractional products, division truncates toward zero. Division by zero returns the signed limit, with `0/0 = 0`. Square root of negative is zero.
- Repeated, per-product fixed-point rounding can introduce precision differences from a fused dot product. Existing SH4ZAM floats are **not bit-identical**. Angle input uses unsigned 0..65535 turns, with a quarter-turn = 16384. Long vectors can saturate in dot products; choose scene scale carefully.
- Non-reentrant SH-2 hardware divider is **not yet enabled**; divide uses a correct baseline likely requiring a libgcc helper on hardware. Benchmark before enabling DIVU or SIMD-like techniques.
- No evidence yet supports a faster-than-Yaul or production-ready performance claim. Measure cycles, generated assembly, and console behavior first.

## Project relationship

SH2ZAM is intended for a **separate `lamb2k/SH2ZAM` repository**, not a rename of
the existing Dreamcast fork. Preserve the inherited Git ancestry, the original
MIT notices, and the [upstream README and acknowledgements](../README.SH4ZAM.md).
The [project README](../README.md) is the Saturn-facing entry point.

Do not push Saturn work into the Dreamcast fork's default branch, rename that
repository, or rewrite its existing branches. The separate project should have
its own `main` branch, issues, releases, and qualification criteria.
