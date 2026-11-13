<div align="center">
  <h1><strong>SH2ZAM!</strong></h1>
  <p>
    Fixed-point math for the Sega Saturn's SH-2 CPUs
    <br />
    <a href="saturn/README.md"><strong>Explore the docs »</strong></a>
    · <a href="exhibits/README.md"><strong>Interactive math lab »</strong></a>
    <br /><br />
    Saturn-native · Fixed-point · Portable
  </p>
  <p>
    <a href="https://github.com/lamb2k/SH2ZAM/actions/workflows/sh2zam-host.yml"><img src="https://github.com/lamb2k/SH2ZAM/actions/workflows/sh2zam-host.yml/badge.svg?branch=main" alt="Host correctness"></a>
    <a href="https://github.com/lamb2k/SH2ZAM/actions/workflows/sh2-cross.yml"><img src="https://github.com/lamb2k/SH2ZAM/actions/workflows/sh2-cross.yml/badge.svg?branch=main" alt="SH-2 cross build"></a>
    <a href="https://github.com/lamb2k/SH2ZAM/actions/workflows/exhibits.yml"><img src="https://github.com/lamb2k/SH2ZAM/actions/workflows/exhibits.yml/badge.svg?branch=main" alt="Interactive math lab"></a>
  </p>
</div>
<hr>

# Overview

<p align="center">
  <img src="doc/sh2zam-gainz.webp" width="480" alt="Sega Saturn with turquoise flexing arms above GAINZ">
</p>

`SH2ZAM` is a general-purpose fixed-point math and linear algebra library for the Sega Saturn. Inspired by [SH4ZAM](https://github.com/gyrovorbis/sh4zam), it brings the same emphasis on practical game mathematics to a different machine: Q16.16 arithmetic, turn-based angles, vectors, matrices and quaternions for the Saturn's SH-2 processors.

**Original project credit:** SH4ZAM by **Falco Girgis and SH4ZAM Contributors**. Their Git history, source, [MIT license](LICENSE), copyright notices and [acknowledgements](README.SH4ZAM.md#acknowledgements) are preserved. SH2ZAM is an independent adaptation, not an official SH4ZAM release or a claim of upstream/Sega endorsement.

> [!IMPORTANT]
> **Experimental Saturn foundation, not full SH4ZAM parity.** Host correctness tests, sanitizers, big-endian SH-2 cross-compilation and a Yaul diagnostic-disc build have passed. Compilation is not console execution: physical-Saturn performance and production qualification remain unverified. See the [validation record](saturn/VALIDATION.md).

# Features

- C11 API with optional C++11 conveniences and C/C++ interoperability.
- Signed Q16.16 arithmetic with explicit saturation and rounding contracts.
- An SH-2 `dmuls.l` multiplication path and a portable reference backend.
- No heap allocation, runtime floating point or mutable global state in the math core.
- Wide-intermediate products and normalization that handle small and large inputs.
- Caller-owned transform stacks and batch transforms without hidden CPU scheduling.
- Installable CMake targets, a Yaul cross-toolchain file and runnable examples.
- Host regression/oracle tests, sanitizer checks and actual SH-2 build checks.

# APIs

- **Scalar:** add/subtract/multiply/divide, muldiv, rounding, reciprocal, square root, reciprocal square root, lerp and smoothstep.
- **Trigonometry:** sine/cosine, sincos, tangent, atan/atan2, asin/acos and shortest-angle interpolation.
- **Vectors:** 2D, 3D and 4D arithmetic, dot products, lengths, normalization and interpolation; 2D/3D cross products.
- **Matrices:** 2×2, 3×3, affine 3×4 and 4×4, transformations, transpose and alias-safe multiplication.
- **Camera:** right-handed look-at, rigid inverse, perspective and clip-to-screen helpers.
- **Quaternions:** multiplication, normalization, conjugation, axis-angle, nlerp, matrix conversion and vector rotation.
- **Game helpers:** AABB/sphere intersection, transform stacks and batch-work partitioning.

Complex/FFT, a general libm replacement, accelerated memory routines and the SH-4 XMTRX register bank are **not** implemented. The C++ facade is smaller than the C API. See the [capability comparison](saturn/PARITY.md).

# Usage

## C11

Include `sh2zam/sh2zam.h` and link the `sh2zam` library:

```c
#include <sh2zam/sh2zam.h>

int main(void) {
    sh2z_vec3_t position = {sh2z_from_int(3), sh2z_from_int(4), 0};
    sh2z_vec3_t direction = sh2z_vec3_normalize(position);
    sh2z_mat34_t world = sh2z_mat34_rotation_y(sh2z_angle_from_degrees(30));
    world.m[0][3] = sh2z_from_int(10);
    sh2z_vec3_t moved = sh2z_mat34_transform_point(&world, position);
    (void)direction;
    (void)moved;
    return 0;
}
```

## C++11

Include `sh2zam/sh2zam.hpp` for the small fixed-point/vector facade. The full C API remains available:

```cpp
#include <sh2zam/sh2zam.hpp>

int main() {
    using sh2z::fixed;
    sh2z::vec3 position(fixed::from_int(3), fixed::from_int(4), fixed{});
    sh2z::vec3 direction = position.direction();
    auto world = sh2z_mat34_rotation_y(sh2z_angle_from_degrees(30));
    auto moved = position.transformed(world);
    (void)direction;
    (void)moved;
    return 0;
}
```

# Installation

## CMake

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DSH2Z_STRICT=ON
cmake --build build --parallel
ctest --test-dir build --output-on-failure
cmake --install build --prefix /your/install/prefix
```

The root build builds **SH2ZAM**, not the inherited Dreamcast library. For library-only builds, set `-DSH2Z_BUILD_TESTS=OFF -DSH2Z_BUILD_EXAMPLES=OFF`. `cmake -S saturn ...` is also supported. Installed consumers can use `find_package(SH2ZAM 0.2 CONFIG REQUIRED)` and `SH2ZAM::sh2zam`.

## Git submodule

```sh
git submodule add https://github.com/lamb2k/SH2ZAM.git external/SH2ZAM
```

```cmake
add_subdirectory(external/SH2ZAM)
target_link_libraries(your_game PRIVATE SH2ZAM::sh2zam)
```

## Sega Saturn / Yaul

Use a bare-metal SH-2 toolchain such as Yaul's `sh2eb-elf-gcc`:

```sh
cmake -S . -B build-sh2 \
  -DCMAKE_TOOLCHAIN_FILE="$PWD/saturn/toolchains/yaul-sh2.cmake" \
  -DSH2Z_BUILD_TESTS=OFF -DSH2Z_BUILD_EXAMPLES=OFF
cmake --build build-sh2 --parallel
```

The library does not replace Yaul, SGL, Jo Engine or a renderer. SDK/runtime compatibility outside the documented Yaul build must be checked separately. See [hardware integration](saturn/HARDWARE.md).

# Examples

| Example | Description |
|---|---|
| [Interactive math lab](exhibits/README.md) | Three looping 3D scenes with a before/after reveal slider, actual C-to-WASM math, precision comparison and browser timing export. |
| [Sprite orbit](saturn/examples/orbit.c) | Integer-only orbit coordinates for driving a sprite. |
| [Cube transforms](saturn/examples/cube.c) | Eight vertices through model, view, perspective and screen-coordinate transforms. |
| [Batch jobs](saturn/examples/batch.c) | Two disjoint vertex jobs compared with serial execution; no slave-CPU launch. |
| [Yaul diagnostic](saturn/examples/yaul-smoke/) | Buildable Saturn diagnostic application and disc image. |

Host example commands and Saturn-disc build instructions are in [the examples guide](saturn/examples/README.md). The cube example does not submit VDP1 polygons or clip polygons.

# Showcase

The browser lab includes **Ring world**, **Cube field** and **Twisted orbit**. Drag the divider to compare the same image before and after changing the math implementation. The “after” path executes SH2ZAM's actual portable C backend compiled to WebAssembly; the “before” path is a conventional float32 C reference, **not SH4ZAM**.

Both sides use the same scene, camera and renderer. Similar pictures are the desired outcome. Browser timings are measured, not scripted; a slower result is reported as slower. **They are not Saturn benchmarks.** See [how to run the exhibits and the measurement contract](exhibits/README.md).

There are no independently verified production adopters or Saturn speedup claims listed yet. Share measured integrations through this repository's issues.

# Conventions

| Convention | SH2ZAM default |
|---|---|
| Scalars | Signed Q16.16; one is 65536; saturating arithmetic |
| Angles | Unsigned 16-bit turns; quarter-turn is 16384 |
| Coordinates | Right-handed; camera looks along −Z; NDC depth [−1, 1] |
| Matrices | Row-major, column vectors; `A * B` applies B first |
| Quaternions | Stored in x, y, z, w order |
| Rounding | Multiply/dot round down once; division truncates toward zero |
| Edge cases | Division by zero saturates by sign; `0/0 = 0`; zero vector normalizes to zero; zero quaternion to identity |

These differ from SH4ZAM's float types, layouts and API contracts. Port calls explicitly; changing an include path is not sufficient. The application owns interrupt preservation, shared-memory/cache synchronization, DMA and VDP1 command submission.

# Community

Use [SH2ZAM issues](https://github.com/lamb2k/SH2ZAM/issues) for Saturn-specific bugs, examples and hardware measurements. Read [CONTRIBUTING.md](CONTRIBUTING.md) before contributing. Dreamcast users should continue to use [upstream SH4ZAM](https://github.com/gyrovorbis/sh4zam); submit reusable upstream improvements there.

# Acknowledgements

- **Falco Girgis and SH4ZAM Contributors:** the original project, its mathematical scope and the inspiration for this library and README structure. The [full inherited acknowledgements](README.SH4ZAM.md#acknowledgements) remain intact.
- **Roberto Daniel Alves Duarte / SaturnMath++** and **Israel Jacquez / Yaul:** research and tooling references for Saturn development; no claim that their routines were copied into SH2ZAM.
- **SH2ZAM Contributors:** the independent Saturn implementation, tests, integration examples and interactive exhibit.

Original MIT notices are preserved. See [LICENSE](LICENSE) and [NOTICE.md](NOTICE.md). New Saturn code lives in `saturn/`; the inherited SH4ZAM sources remain as provenance/reference, not active Saturn dependencies.
