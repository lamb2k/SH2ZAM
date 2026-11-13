# SH2ZAM interactive math lab

Three looping 3D exhibits with a draggable before/after reveal, pause/play, timeline scrubbing, mesh-detail control, numeric error display, browser measurements and JSON export. The supplied GAINZ artwork appears in the lab and repository overview.

| Exhibit | Demonstration |
|---|---|
| Ring world | Low-poly planet and concentric rings |
| Cube field | Many cubes in one transformed vertex batch |
| Twisted orbit | Continuous torus-knot mesh |

## Run it

The **Interactive math lab** GitHub Actions workflow produces a ready-to-run `sh2zam-interactive-lab` artifact. Download it, extract it, and open `index.html`. All scripts, the compiled module and the artwork are local; no CDN, account or third-party analytics is used. Alternatively serve the directory with `python3 -m http.server --directory exhibits 8000`.

To build from source, install Clang with its WebAssembly target, LLD, Python 3 and Node.js, then run from the repository root:

```sh
bash exhibits/build.sh
node exhibits/test-math.cjs
python3 -m http.server --directory exhibits 8000
```

Open `http://localhost:8000`. The build also emits a base64 JS module so the downloaded site can run without a local server, in browsers that allow local scripts. Build products are intentionally ignored by Git; CI builds them from the current library sources.

## What before and after mean

**Before:** a conventional float32 C affine transform. It is not SH4ZAM and is not deliberately slowed down.

**After:** this repository's actual `sh2z_mat34_rotation_x/y`, `sh2z_mat34_multiply` and `sh2z_mat34_transform_points` routines, linked into a WebAssembly module with `SH2Z_FORCE_PORTABLE=1`. This is not a JavaScript imitation and not an SH-2 emulator.

The input vertices, angle grid, camera, triangle colors, lighting and Canvas renderer are shared. Matching pictures are the point. The projected-difference metric measures the maximum per-vertex CSS-pixel displacement; it is not an image-difference metric. Projection itself is shared JavaScript, not the integer-pixel Saturn projection API.

## Timing contract

Both measured transform kernels are C compiled with the same compiler/optimization settings into the same WASM module. Neither allocates memory during measurement. Matrix construction, trigonometry, scene generation, output conversion and rendering are excluded from timing. Both paths receive the same Q16.16 source vertices; float conversion happens once during preparation.

The loop pauses for a benchmark. Each kernel is warmed for 32 calls, then measured in 21 samples of 64 calls, alternating which kernel is measured first. Displayed values are medians per batch, including call overhead. Exported JSON records all samples, the geometry, pose, browser user agent, module hash and measurement boundaries.

**A desktop browser often favors native floating point. SH2ZAM may be slower here. The exhibit reports the result without artificially slowing the reference, changing animation rates, or implying Saturn FPS/performance.** Console performance requires separate SH-2 hardware measurements.

## Accessibility and tests

The reveal, detail and timeline controls are native keyboard-accessible sliders. The handle also supports mouse/touch dragging. Reduced-motion preferences start paused. Hidden tabs do not advance the simulation clock. Load failures show an error rather than substituting fake math.

`test-math.cjs` checks buffer bounds, no imports, fixed-point identity and 737,280 component comparisons on bounded scene inputs. This is an exhibit regression suite, not a global numeric proof. `test-browser.py` covers scenes, high detail, reveal, scrubbing, pause, measurements, export, narrow layout and reduced motion in Chromium.

## GitHub Pages

The workflow attempts Pages deployment from `main` using the standard Pages actions, but only after a successful build/test job. When Pages is not enabled or the token cannot enable it, the tested artifact is still available. A repository administrator can select **Settings → Pages → Source: GitHub Actions**, then rerun the workflow. Do not advertise a live URL until deployment succeeds.

## Credits

SH2ZAM is inspired by SH4ZAM by Falco Girgis and SH4ZAM Contributors. Original licensing and acknowledgements remain in the repository. No Sega or upstream endorsement is implied. See the root `LICENSE` and `NOTICE.md`.
