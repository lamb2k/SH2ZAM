# SH2ZAM

**Saturn-native fixed-point math, inspired by SH4ZAM.**

SH2ZAM is an independent Sega Saturn / SH-2 adaptation of the mathematical
capabilities offered by [SH4ZAM](https://github.com/gyrovorbis/sh4zam). It targets
Saturn development with a C11 fixed-point core, a portable reference path, and
an experimental SH-2 multiplication backend.

**Original project credit:** SH4ZAM was created by **Falco Girgis and SH4ZAM
Contributors**. Their Git history, copyright notices, and [MIT license](LICENSE)
are preserved. SH2ZAM is not an official SH4ZAM release, and no endorsement by
the upstream maintainers is implied.

> **Status: experimental prototype.** Portable host tests exist. The SH-2
> assembly path has not yet been cross-compiled or validated on a Saturn.
> Full SH4ZAM feature parity, drop-in API compatibility, and performance gains
> are not claimed.

## Why a separate project?

SH2ZAM has a different hardware target, numeric representation, API, build
configuration, and validation path. Its goal is equivalent *math capabilities*
implemented appropriately for Saturn, not a rename of Dreamcast code.

The native API uses `sh2z_*` names and Q16.16 fixed-point values. Existing
`shz_*` floating-point calls require an explicit port; simply replacing an
include or library name is not sufficient.

Keep Dreamcast projects on SH4ZAM. SH2ZAM is intended to have its own repository,
release cycle, issues, and Saturn-focused documentation.

## Implemented so far

| Area | Prototype coverage |
| --- | --- |
| Scalars | Saturating Q16.16 arithmetic, division, and square root |
| Trigonometry | Sine and cosine with a quarter-wave lookup table and interpolation |
| Vectors | 2D/3D/4D dot products; 3D cross, length, normalization, add, and scale |
| Matrices | Row-major affine 3x4 and 4x4 matrices, transforms, and multiplication |
| Quaternions | Hamilton multiplication and normalization |
| Integration | C11 API, C++ linkage, CMake target, portable tests, SH-2 toolchain configuration |

The SH-2 multiply path uses `dmuls.l` and MACH/MACL. Hardware division,
complex/FFT, advanced trigonometry, additional matrix types, optimized memory
routines, and a C++ convenience API remain future work. XMTRX hardware-register
compatibility is not provided.

## Build and test on a host

Run these commands from the repository root:

```sh
cmake -S saturn -B build/sh2zam-host \
  -DSH2Z_BUILD_TESTS=ON -DSH2Z_FORCE_PORTABLE=ON
cmake --build build/sh2zam-host --parallel
ctest --test-dir build/sh2zam-host --output-on-failure
```

**Use `-S saturn`.** The inherited root CMake project still builds SH4ZAM;
its presence preserves the upstream baseline, not a Saturn compatibility claim.

## Integrate with a project

```cmake
set(SH2Z_BUILD_TESTS OFF CACHE BOOL "Build SH2ZAM host tests")
add_subdirectory(external/SH2ZAM/saturn)
target_link_libraries(your_game PRIVATE SH2ZAM::sh2zam)
```

```c
#include <sh2zam/sh2zam.h>

int main(void) {
    sh2z_vec3_t position = {sh2z_from_int(3), sh2z_from_int(4), 0};
    sh2z_vec3_t direction = sh2z_vec3_normalize(position);
    sh2z_mat34_t world = sh2z_mat34_identity();
    world.m[0][3] = sh2z_from_int(10);
    sh2z_vec3_t transformed = sh2z_mat34_transform_point(&world, direction);
    (void)transformed;
    return 0;
}
```

SH2ZAM supplies math, not a Saturn hardware SDK or renderer. Integration with a
particular SDK must be built and tested rather than assumed.

## Experimental Saturn cross-build

With `sh2eb-elf-gcc` and its matching toolchain programs installed:

```sh
cmake -S saturn -B build/sh2zam-sh2 \
  -DCMAKE_TOOLCHAIN_FILE="$PWD/saturn/toolchains/yaul-sh2.cmake" \
  -DSH2Z_BUILD_TESTS=OFF
cmake --build build/sh2zam-sh2 --parallel
```

This configuration targets big-endian SH-2. It is a starting point, **not a
verified cross-build or a hardware-tested release**. Portable host results do
not validate the SH-2 assembly path.

## Numerical and compatibility boundaries

Q16.16 is not floating point. Read the
[numeric contract](saturn/README.md#numeric-contract) before porting a caller:
range, saturation, rounding, zero division, and scene scale all matter.
SH2ZAM's row-major matrices also differ from SH4ZAM's documented column-major
convention. There is no promise of bit-identical results or identical layouts.

## Development priorities

1. Cross-compile and inspect the SH-2 machine code.
2. Validate numerical behavior in an emulator and on a physical Saturn.
3. Benchmark representative workloads before enabling further optimizations.
4. Expand the API against an explicit SH4ZAM capability-parity checklist.

See the [porting and qualification plan](saturn/PORTING_PLAN.md) and
[Saturn implementation notes](saturn/README.md).

## Attribution and upstream relationship

- **Falco Girgis and SH4ZAM Contributors:** original SH4ZAM library, design,
  and inherited source/history. The original
  [README and contributor acknowledgements](README.SH4ZAM.md) are preserved.
- **Roberto Daniel Alves Duarte (SaturnMath++)** and **libyaul contributors:**
  additional research references credited in the Saturn implementation notes.

Retain copyright notices and license text with redistributed code. Keep the
upstream history available, review upstream changes, and propose reusable fixes
back to SH4ZAM. Changes here do not imply that the upstream project supports
Saturn.

**License:** [MIT](LICENSE). Existing upstream copyright notices remain intact.
