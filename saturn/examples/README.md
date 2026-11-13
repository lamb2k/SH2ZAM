# SH2ZAM example implementations

All math kernels in `math_examples.c` and `saturn_smoke.c` are freestanding and integer-only. Host programs print results; the Yaul app displays diagnostic text on Saturn. These examples deliberately separate math from rendering and CPU scheduling.

## Host previews

From the repository root:

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
ctest --test-dir build --output-on-failure
./build/saturn/sh2zam_orbit
./build/saturn/sh2zam_cube
./build/saturn/sh2zam_batch
./build/saturn/sh2zam_smoke_main
```

**Orbit:** `orbit.c` calls `sh2z_example_orbit()`. Expected quarter-cycle positions are (240,112), (160,168), (80,112), (160,56). Use the resulting integer position in your sprite command construction; the sample does not write VRAM.

**Cube transform:** `cube.c` composes rotations, look-at and perspective matrices, transforms eight cube vertices, rejects outside clip-space and outputs 320x224 screen coordinates. It is a math pipeline, not a filled-polygon renderer. A real renderer must handle polygon/near-plane clipping and VDP1 command ordering.

**Two-job batch:** `batch.c` partitions 17 vertices into disjoint ranges and verifies identical results to the serial transform. It executes those jobs sequentially. To dispatch on two SH-2 CPUs, use your engine's scheduler and cache/ownership protocol; do not share writable matrix-stack state between jobs.

**Hardware smoke kernel:** `saturn_smoke.c` exports `sh2z_saturn_smoke()`. The host runner prints a mask; zero is expected. The same C function can be called inside an existing Saturn game. Mask meanings are in [HARDWARE.md](../HARDWARE.md).

## Yaul diagnostic disc

With an installed and configured Yaul SDK:

```sh
make -C saturn/examples/yaul-smoke
```

Or use the image documented by the Yaul project:

```sh
docker pull ijacquez/yaul:latest
docker run --rm -v "$PWD:/work" -w /work ijacquez/yaul:latest \
  bash -c 'export PATH="$YAUL_INSTALL_ROOT/bin:$PATH"; make -C saturn/examples/yaul-smoke'
```

The cross workflow builds the library using `-m2 -mb`, records compiler/image metadata and disassembly, compiles the freestanding examples and invokes the Yaul app build. Its artifacts are build evidence, not proof of a successful boot.

On a successful console run the app should show backend `sh2-dmuls`, a zero failure mask, changing orbit coordinates and a cube vertex. It displays text; it does not draw an orbiting sprite or shaded cube. Record real boot results before claiming hardware qualification.
