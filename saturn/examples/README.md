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
./build/saturn/sh2zam_view
./build/saturn/sh2zam_edges
./build/saturn/sh2zam_merge
./build/saturn/sh2zam_shade
```

**Orbit:** `orbit.c` calls `sh2z_example_orbit()`. Expected quarter-cycle positions are (240,112), (160,168), (80,112), (160,56). Use the resulting integer position in your sprite command construction; the sample does not write VRAM.

**Cube transform:** `cube.c` composes rotations, look-at and perspective matrices, transforms eight cube vertices, rejects outside clip-space and outputs 320x224 screen coordinates. It is a math pipeline, not a filled-polygon renderer. A real renderer must handle polygon/near-plane clipping and VDP1 command ordering.

**Two-job batch:** `batch.c` partitions 17 vertices into disjoint ranges and verifies identical results to the serial transform. It executes those jobs sequentially. To dispatch on two SH-2 CPUs, use your engine's scheduler and cache/ownership protocol; do not share writable matrix-stack state between jobs.

**View batch:** `view.c` places a camera with `sh2z_view_place`, projects two road quads with `sh2z_view_batch`, draws the one in front and rejects the one behind the camera by ANDing outcodes. It then checks that `sh2z_view_batch_divu`, the SH-2 hardware-divider entry point, writes the same bytes. See [VIEW_BATCH.md](../VIEW_BATCH.md) and [DIVU.md](../DIVU.md); the API is experimental.

**Edges and near clip:** `edges.c` cuts both side edges of a road quad where a 6-unit texture restarts with `sh2z_view_edge_steps`, and clips a quad that passes the camera with `sh2z_view_clip_near`. See [VIEW_BATCH.md](../VIEW_BATCH.md).

**Texture merges:** `merge.c` projects the boundaries of a curving road strip whose texture repeats every 2 units and plans with `sh2z_view_merge_runs` which repetitions to draw as one quad (runs of 1, 2 or 4) so that every texel stays within 1 pixel of the one-quad-per-repetition drawing. It prints the plan: 32 repetitions as 20 quads at 1 px, 24 at 0.5 px. See [VIEW_BATCH.md](../VIEW_BATCH.md).

**Shading:** `shade.c` culls the faces of a cube against an integer eye with `sh2z_shade_cull`, lights its six normals with `sh2z_shade_light_normals`, packs one Gouraud colour per kept face with `sh2z_shade_gouraud`, then builds a perspective fog table with `sh2z_shade_fog_build` and fades and colours a row of depths with `sh2z_shade_fog_lookup` and `sh2z_shade_fade_mix`. It keeps 3 of 6 faces. See [SHADE.md](../SHADE.md).

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
