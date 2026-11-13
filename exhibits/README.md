# SH2ZAM interactive math lab

Three looping 3D exhibits with live float32, safe SH2ZAM and prepared SH2ZAM
transform times. Float and prepared images run together; the mesh-detail slider
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
| Safe batch | `sh2z_mat34_transform_points`: the previous optimization, using one matrix bound check, direct raw-product sums, one floor and one clamp per output. |
| Prepared batch | `sh2z_transform_plan_init` followed by `sh2z_transform_plan_execute_range`: cached mesh bounds prove that sums and outputs fit. The hot loop uses a defined branchless floor and omits clamps. Matrix qualification is included once per measured batch; failed proofs use the exact general fallback. |

The actual C kernels are linked into one WebAssembly module with
`SH2Z_FORCE_PORTABLE=1`. No SH-2 emulation or JavaScript replacement is used.
The float reference is not SH4ZAM. The original full-range scalar function remains
in the library and correctness tests; the displayed baseline is the prior safe
batch, so improvements are not measured only against the oldest implementation.
Both fixed-point outputs must match bit for bit;
the live view checks that on every displayed pose.

All paths receive the same quantized vertices **and matrix coefficients**. Matrix
construction and conversion happen outside measurement. Consequently, the reported
float error measures transform rounding, excluding common matrix/trigonometry
quantization. This differs from the earlier lab's independently built float matrix;
do not compare those earlier error figures as if the reference were unchanged.

The camera, triangle colors, lighting and Canvas renderer are shared. The screen
offset is the maximum per-vertex CSS-pixel displacement, not an image-difference
metric. Projection is shared JavaScript, not the integer-pixel Saturn projection API.
See [the prepared kernel's proof and contract](../saturn/BATCH_TRANSFORMS.md).

## Timing contract

All three kernels use the same compiler and optimization settings. They allocate no
memory during measurement. Matrix construction, trigonometry, input conversion,
the one-time mesh-bound scan, scene generation, rendering, checksums and precision
checks are outside timing. Source meshes remain immutable after preparation.
Changing a mesh requires rerunning preparation. The safe matrix gate and the
prepared matrix qualification remain inside their respective measured calls.

Live timings update every 500 ms while the exhibit runs. Repetition counts adapt
until each update spans at least 0.75 ms per path; the display uses a rolling
median of five updates. Measurement
order rotates across the three paths to distribute first/second/third position.
The ratio `safe / prepared` isolates this optimization; `float / prepared`
compares it with float. Above 1 means prepared is faster, below 1 means slower.
These are transform batch times, not frame rate.

**Capture benchmark** pauses at the current pose, warms all paths for 32 calls,
then captures 21 samples at that same pose with rotating order. Each retained
sample spans at least 5 ms; the iteration count adapts separately for each path.
Results include JS-to-WASM call overhead and **one matrix qualification per
prepared batch**. Separate diagnostic samples report qualification alone and
the already-qualified kernel. Their independently measured medians need not add
up to the inclusive median; they are not used to construct the headline time.

Schema-3 JSON export (`float-safe-prepared-v1`) records every distribution,
iteration count and elapsed duration, component timings, medians, ratios,
independent output checksums, precision checks, geometry, pose, browser user
agent, module hash and measurement boundaries. Internal JSON keys `general` and
`optimized` now mean safe batch and prepared batch, respectively. Schema 2 used
different baselines and is not interchangeable. **Play** resumes live sampling.

Desktop browsers use their host floating-point facilities; SH2ZAM may still lose to
float there. Browser results establish browser behavior only. The separate
[Saturn benchmark](../saturn/benchmarks/README.md) runs the same shared bridge with
SH-2 software float and four fixed-point paths, including explicit fused
`MAC.L`/`DMULS.L` candidates. The native capture samples 21 distinct poses, unlike
the browser's 21 repetitions at one paused pose. Its results section stays
unmeasured until an evidenced physical-console capture is recorded; emulator data
is never displayed as physical-console speed.

## Accessibility and tests

Mesh detail is the only slider and is keyboard accessible. Reduced-motion
preferences start paused. Hidden tabs stop advancing the simulation clock and
collecting live samples. Load failures show an error rather than substituting math.

`test-math.cjs` checks module imports, buffer isolation/bounds, exact identity,
safe/prepared equality, prepared fallback selection and float precision. `test-browser.py` covers scene and
detail changes, continued live sampling, one-slider layout, pause/play, capture,
adaptive capture, inclusive setup, three-way export, narrow layout and reduced motion in Chromium. The shared native
fixture suite checks all 15 scene/detail combinations at 21 poses and verifies
full-batch versus chunked output identity across all five wrappers. Host candidate
wrappers use the portable fallback and do not execute SH-2 assembly. The library's
separate prepared oracle suite covers full-range inputs, fallback boundaries,
matrix snapshots, range and immutable-source overlap contracts.

Manual WASM SIMD experiments are kept under
[`saturn/benchmarks/wasm-experiments`](../saturn/benchmarks/wasm-experiments/).
They compare identical compiler flags with float, retain exact-output checks,
and are not selected by the production lab.

## GitHub Pages

The workflow attempts Pages deployment from `main` using the standard Pages actions, but only after a successful build/test job. When Pages is not enabled or the token cannot enable it, the tested artifact is still available. A repository administrator can select **Settings → Pages → Source: GitHub Actions**, then rerun the workflow. Do not advertise a live URL until deployment succeeds.

## Credits

SH2ZAM is inspired by SH4ZAM by Falco Girgis and SH4ZAM Contributors. Original licensing and acknowledgements remain in the repository. No Sega or upstream endorsement is implied. See the root `LICENSE` and `NOTICE.md`.
