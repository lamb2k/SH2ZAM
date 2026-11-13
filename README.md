# SH2ZAM

**A Saturn-native fixed-point math foundation inspired by SH4ZAM.**

SH2ZAM 0.2 targets the Sega Saturn's SH-2 CPUs with a C11 Q16.16 core, an SH-2 multiplication backend, a portable reference path, and optional C++11 conveniences. It is a math library, not a renderer or a replacement for your Saturn SDK.

**Credit:** [SH4ZAM](https://github.com/gyrovorbis/sh4zam) by **Falco Girgis and SH4ZAM Contributors**. The original Git history, source, copyright notices, [MIT license](LICENSE), and [upstream README/acknowledgements](README.SH4ZAM.md) are preserved. This is an independent adaptation; no upstream or Sega endorsement is implied. See [NOTICE](NOTICE.md).

> **Status: experimental Saturn foundation, not full SH4ZAM parity.** Host correctness and sanitizer tests are implemented, along with actual big-endian SH-2 cross-compilation and a Yaul diagnostic-disc build in CI. Compilation is not console execution. No physical-Saturn performance advantage or production qualification is claimed. [Recorded validation and build artifacts](saturn/VALIDATION.md).

## What is implemented

| Area | Saturn implementation |
|---|---|
| Scalar | Saturating add/subtract/multiply/divide, wide-intermediate muldiv, floor/ceil/round, reciprocal, square root, reciprocal square root, lerp and smoothstep |
| Angles and trig | Unsigned 16-bit turns, sine/cosine table, sincos, tan, atan/atan2, asin/acos, shortest-angle interpolation |
| Vectors | 2D/3D/4D arithmetic, dot, length, robust normalization, lerp; 2D/3D cross products |
| Matrices | 2x2, 3x3, affine 3x4 and 4x4; alias-safe multiplication, transforms, transpose, translation/scale/axis rotations |
| Camera | Right-handed look-at, validated rigid inverse, perspective and clip-space-to-screen projection |
| Quaternions | Multiply, normalize, conjugate, axis-angle, shortest-path nlerp, matrix conversion and vector rotation |
| Game helpers | AABB/sphere intersections, caller-owned transform stack, in-place batch transforms and overflow-safe work partitioning |
| Integration | C11 API, small C++11 facade, CMake install/export, Yaul cross-toolchain file and diagnostic app |

The core does not allocate memory, use runtime floating point, or keep mutable global math state. Inverse trig and division prioritize correctness; they are not advertised as fastest-in-class kernels.

### Fixes beyond the initial prototype

Vector length no longer takes the square root of an already saturated dot product. Normalization handles one-LSB vectors as well as full-range inputs. Dot products and matrix/quaternion products retain cancellation and round once before final saturation, without requiring 128-bit arithmetic on SH-2. Regression checks run in Release builds too.

## Build and run on a host

Requires CMake 3.16+, a C11 compiler and a C++11 compiler for the optional interop test. The host oracle suite uses GCC/Clang's `__int128`; the library does not.

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DSH2Z_STRICT=ON
cmake --build build --parallel
ctest --test-dir build --output-on-failure
./build/saturn/sh2zam_orbit
./build/saturn/sh2zam_cube
./build/saturn/sh2zam_batch
./build/saturn/sh2zam_smoke_main
```

The foundation test performs **1,073,727 checks**: full-range integer oracles, overflow/cancellation regressions, all 65,536 turn angles, the full representable asin domain, random vector/matrix cases, invalid inputs, and aliasing. These are checks within a suite, not a million independent test programs. Seven CTest targets also cover legacy regressions, C++ linkage and four examples.

For sanitizers, use a separate host build with `-DSH2Z_SANITIZE=ON -DCMAKE_BUILD_TYPE=Debug`. For library-only integration set `-DSH2Z_BUILD_TESTS=OFF -DSH2Z_BUILD_EXAMPLES=OFF`. `cmake -S saturn ...` remains supported.

## Use in a project

```cmake
add_subdirectory(external/SH2ZAM)
target_link_libraries(your_game PRIVATE SH2ZAM::sh2zam)
```

Or install with `cmake --install build --prefix <prefix>`, then use `find_package(SH2ZAM 0.2 CONFIG REQUIRED)`. Embedded builds default to library-only.

```c
#include <sh2zam/sh2zam.h>

void transform_actor(void) {
    sh2z_vec3_t point = {sh2z_from_int(3), sh2z_from_int(4), 0};
    sh2z_vec3_t direction = sh2z_vec3_normalize(point);
    sh2z_mat34_t world = sh2z_mat34_rotation_y(sh2z_angle_from_degrees(30));
    world.m[0][3] = sh2z_from_int(10);
    sh2z_vec3_t moved = sh2z_mat34_transform_point(&world, point);
    (void)direction;
    (void)moved;
}
```

## Saturn build and examples

Use a **bare-metal SH-2** compiler such as Yaul's `sh2eb-elf-gcc`; Ubuntu's SH-4 Linux compiler is not a substitute.

```sh
cmake -S . -B build-sh2 \
  -DCMAKE_TOOLCHAIN_FILE="$PWD/saturn/toolchains/yaul-sh2.cmake" \
  -DSH2Z_BUILD_TESTS=OFF -DSH2Z_BUILD_EXAMPLES=OFF
cmake --build build-sh2 --parallel
```

[Examples and instructions](saturn/examples/README.md) include sprite-orbit coordinates, an eight-vertex model/view/projection pipeline, two disjoint batch jobs, and a Yaul app displaying a diagnostic bitmask. The batch example does **not** launch the slave CPU; the cube example does **not** submit VDP1 polygons or implement polygon clipping.

See [hardware integration and qualification](saturn/HARDWARE.md), [capability comparison](saturn/PARITY.md), and [remaining work](saturn/PORTING_PLAN.md).

## Numeric and integration contracts

Values are signed Q16.16: approximately -32768 to +32768, with 1/65536 resolution. Arithmetic saturates; multiply/dot round down, divide truncates toward zero. Division by zero returns the signed limit (`0/0 = 0`). A zero vector normalizes to zero; a zero quaternion normalizes to identity.

Matrices are **row-major with column vectors**; `A*B` applies B first. Quaternions are **x,y,z,w**. Angles are turns, not radians. The camera looks down -Z; NDC depth is [-1,1]. These are not SH4ZAM's binary/layout/API contracts: port calls explicitly. Camera constructors reject invalid/unrepresentable results rather than silently returning unusable matrices.

Division currently uses integer compiler helpers, not the SH-2 DIVU. Applications retain responsibility for interrupt register preservation, shared-memory ownership, cache synchronization, VDP1 command lists and DMA. The library performs none of those operations behind the caller's back.

## Repository layout

New Saturn work lives in `saturn/`. The root build now builds SH2ZAM. Inherited SH4ZAM sources remain as provenance/reference, not active Saturn dependencies. Original build files are retained as `CMakeLists.SH4ZAM.txt` and `Makefile.SH4ZAM`; Dreamcast users should use upstream SH4ZAM instead.
