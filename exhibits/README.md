# SH2ZAM interactive math lab

Three looping 3D exhibits with live float32, general SH2ZAM and optimized SH2ZAM
transform times. Float and optimized images run together; the mesh-detail slider
changes the vertex count for every path. Pause/play, precision checks, source code
panels and JSON measurement export are included. The supplied GAINZ artwork appears
in the lab and repository overview.

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

## The three paths

| Path | Implementation |
|---|---|
| Float32 | Conventional C affine transform; browser floating point in WASM, compiler software-float helpers on SH-2. |
| Current SH2ZAM | `sh2z_mat34_transform_points_general`: the original full-range implementation, preserving quotient/remainder accumulation for each product. |
| Optimized SH2ZAM | `sh2z_mat34_transform_points`: checks the matrix bound once, reuses coefficients, unrolls all three rows and rounds once after direct raw-product accumulation. Outside the bound, it uses the general path. |

The actual C kernels are linked into one WebAssembly module with
`SH2Z_FORCE_PORTABLE=1`. No SH-2 emulation or JavaScript replacement is used.
The float reference is not SH4ZAM. Both fixed-point outputs must match bit for bit;
the live view checks that on every displayed pose.

All paths receive the same quantized vertices **and matrix coefficients**. Matrix
construction and conversion happen outside measurement. Consequently, the reported
float error measures transform rounding, excluding common matrix/trigonometry
quantization. This differs from the earlier lab's independently built float matrix;
do not compare those earlier error figures as if the reference were unchanged.

The camera, triangle colors, lighting and Canvas renderer are shared. The screen
offset is the maximum per-vertex CSS-pixel displacement, not an image-difference
metric. Projection is shared JavaScript, not the integer-pixel Saturn projection API.
See [the optimized kernel's proof and contract](../saturn/BATCH_TRANSFORMS.md).

## Timing contract

All three kernels use the same compiler and optimization settings. They allocate no
memory during measurement. Matrix construction, trigonometry, input conversion,
scene generation, rendering, checksums and precision checks are outside timing.
The optimized matrix-bound check remains inside its measured call.

Live timings update every 500 ms while the exhibit runs. Each update averages 64
calls per path, and the display uses a rolling median of five updates. Measurement
order rotates across the three paths to distribute first/second/third position.
The ratio `current / optimized` isolates the batch optimization; `float / optimized`
compares it with float. Above 1 means optimized is faster, below 1 means slower.
These are transform batch times, not frame rate.

**Capture benchmark** pauses at the current pose, warms all paths for 32 calls,
then captures 21 samples of 64 calls per kernel with rotating order. Results include
JS-to-WASM call overhead. Schema-2 JSON export records every distribution, medians,
ratios, independent output checksums, precision checks, geometry, pose, browser user
agent, module hash and measurement boundaries. **Play** resumes live sampling.

Desktop browsers use their host floating-point facilities; SH2ZAM may still lose to
float there. Browser results establish browser behavior only. The separate
[Saturn benchmark](../saturn/benchmarks/README.md) runs the same shared bridge with
SH-2 software float and native fixed-point kernels. Its results section stays
unmeasured until an evidenced physical-console capture is recorded; emulator data
is never displayed as physical-console speed.

## Accessibility and tests

Mesh detail is the only slider and is keyboard accessible. Reduced-motion
preferences start paused. Hidden tabs stop advancing the simulation clock and
collecting live samples. Load failures show an error rather than substituting math.

`test-math.cjs` checks module imports, buffer isolation/bounds, exact identity,
general/optimized equality and float precision. `test-browser.py` covers scene and
detail changes, continued live sampling, one-slider layout, pause/play, capture,
three-way export, narrow layout and reduced motion in Chromium. The shared native
fixture suite checks all 15 scene/detail combinations at 21 poses and verifies
full-batch versus chunked output identity. The library's separate batch oracle
suite covers full-range inputs and fallback boundaries.

## GitHub Pages

The workflow attempts Pages deployment from `main` using the standard Pages actions, but only after a successful build/test job. When Pages is not enabled or the token cannot enable it, the tested artifact is still available. A repository administrator can select **Settings → Pages → Source: GitHub Actions**, then rerun the workflow. Do not advertise a live URL until deployment succeeds.

## Credits

SH2ZAM is inspired by SH4ZAM by Falco Girgis and SH4ZAM Contributors. Original licensing and acknowledgements remain in the repository. No Sega or upstream endorsement is implied. See the root `LICENSE` and `NOTICE.md`.
