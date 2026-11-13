# SH2ZAM interactive math lab

Three looping 3D exhibits with live float32, safe SH2ZAM and optimized SH2ZAM
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
| Safe batch | `sh2z_mat34_transform_points`: the previous optimization, using one matrix bound check, direct raw-product sums, one floor and one clamp per output. |
| Optimized batch | Actual coordinate counts select an exact coordinate-reuse plan for sufficiently repetitive meshes, otherwise the prepared scalar kernel. Every measured batch includes matrix qualification, all contribution-table rebuilding when selected, and every output write. Failed numeric proofs use the exact general fallback. |

The actual C kernels are linked into one WebAssembly module with
`SH2Z_FORCE_PORTABLE=1`. Live browser measurements execute WebAssembly directly.
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

The coordinate-reuse path scans quantized vertices once, recording distinct X
values, distinct (Y,Z) pairs and two indices per original vertex. For each new
matrix it rebuilds raw signed-64-bit contribution tables, then adds and floors
once per output row. It preserves input order, all Q16.16 coordinate bits and all
output stores. The selection rule requires at least 128 vertices and
`2 * unique_YZ + unique_X <= vertex_count / 2`, with a 256-key budget for each
table. Every current cube-field detail qualifies; the ring and torus use the
prepared scalar path. Selection depends on the data, never the scene name.
A second proof recognizes blocks of eight vertices sharing two X contributions
and four paired YZ contributions. Eligible complete blocks load 18 terms instead
of 48 and reuse six indices instead of 16; arbitrary range prefixes and tails
keep the scalar loop. The proof checks actual indices, and adds no per-vertex
storage. This work heuristic does not guarantee a float win. See the
[coordinate-reuse API, proof and memory costs](../saturn/COORDINATE_REUSE.md).

## Timing contract

All three kernels use the same compiler and optimization settings. They allocate no
memory during measurement. Matrix construction, trigonometry, input conversion,
the one-time mesh-bound scan, coordinate indexing and workspace binding, scene
generation, rendering, checksums and precision
checks are outside timing. Source meshes remain immutable after preparation.
Changing a mesh requires rerunning preparation. The safe matrix gate and the
optimized matrix qualification and contribution-table building remain inside
their respective measured calls. Repeated matrices still rebuild every table;
only immutable workspace bindings retain their validation. Float receives the same immutable source and
quantized coefficients and retains the conventional affine reference kernel.

Live timings update every 500 ms while the exhibit runs. Repetition counts adapt
until each update spans at least 0.75 ms per path; the display uses a rolling
median of five updates. Measurement
order rotates across the three paths to distribute first/second/third position.
The ratio `safe / optimized` isolates this optimization; `float / optimized`
compares it with float. Above 1 means optimized is faster, below 1 means slower.
These are transform batch times, not frame rate.

**Capture benchmark** pauses at the current pose, warms all paths for 32 calls,
then captures 21 samples at that same pose with rotating order. Each retained
sample spans at least 5 ms; the iteration count adapts separately for each path.
Results include JS-to-WASM call overhead and **one matrix qualification per
optimized batch, including contribution-table building when selected**.
Separate diagnostic samples report plan preparation and the already-prepared
kernel. Their independently measured medians need not add
up to the inclusive median; they are not used to construct the headline time.

Schema-4 JSON export (`float-safe-mesh-v1`) records every distribution,
iteration count and elapsed duration, component timings, medians, ratios,
independent output checksums, precision checks, geometry, pose, browser user
agent, module hash and measurement boundaries. `optimizedStrategy` records
`coordinate-reuse`, `prepared-scalar` or `general-fallback`.
`coordinateReuseBlockSize` records 8 for the unrolled blocks, 1 for scalar
coordinate reuse or 0 when reuse is inactive. Internal JSON keys
`general` and `optimized` mean safe batch and the selected optimized batch,
respectively. Schema 3 always used prepared scalar; schema 2 used different
baselines. **Play** resumes live sampling.

Desktop browsers use their host floating-point facilities; SH2ZAM may still lose to
float there. Browser results establish browser behavior only. The separate
[Saturn benchmark](../saturn/benchmarks/README.md) runs the same shared bridge with
SH-2 software float and four fixed-point paths, including explicit fused
`MAC.L`/`DMULS.L` candidates. The native capture samples 21 distinct poses, unlike
the browser's 21 repetitions at one paused pose. Its results section stays
unmeasured until an evidenced physical-console capture is recorded; emulator data
is never displayed as physical-console speed.

The separate **SH-2 / instruction model** table displays a recorded execution of
the actual compiled SH-2 ELF in the unmodified, pinned Yabause CPU interpreter.
It includes software-float helpers and native MAC.L/DMULS.L instructions. Its
five paths retain the native prepared/fused implementations; the new browser
coordinate tables are not substituted into that capture. Model values are
guest cycles, with one plan preparation included per batch, across 21 poses.
They exclude cache, bus, pipeline, peripheral and second-CPU effects and cannot
be converted into claimed console timings. Provenance, raw samples, exact-output
checks and reproduction instructions are in the
[executed SH-2 model record](../saturn/benchmarks/sh2-model/README.md).
The browser and physical-console tables keep their own separate measurement
contracts and schemas.

## Accessibility and tests

Mesh detail is the only slider and is keyboard accessible. Reduced-motion
preferences start paused. Hidden tabs stop advancing the simulation clock and
collecting live samples. Load failures show an error rather than substituting math.

`test-math.cjs` checks module imports, buffer isolation/bounds, exact identity,
safe/optimized equality, prepared fallback selection and float precision. It
also checks all 315 shared mesh/pose combinations, including original output
order and actual coordinate-reuse selection. `test-browser.py` covers scene and
detail changes, continued live sampling, one-slider layout, pause/play, capture,
adaptive capture in all three scenes, inclusive setup, three-way export,
recorded-model labels, narrow layout and reduced motion in Chromium. The shared native
fixture suite checks all 15 scene/detail combinations at 21 poses and verifies
full-batch versus chunked output identity across all five wrappers. Host candidate
wrappers use the portable fallback and do not execute SH-2 assembly. The library's
separate prepared oracle suite covers full-range inputs, fallback boundaries,
matrix snapshots, range and immutable-source overlap contracts.
The coordinate-reuse suite adds independent 128-bit oracles, capacity fallback,
table lifetime and overlap tests. Model-data validation recomputes inclusive
medians from raw samples and checks native guard execution.

Manual WASM SIMD experiments are kept under
[`saturn/benchmarks/wasm-experiments`](../saturn/benchmarks/wasm-experiments/).
They compare identical compiler flags with float, retain exact-output checks,
and are not selected by the production lab.

## GitHub Pages

The workflow attempts Pages deployment from `main` using the standard Pages actions, but only after a successful build/test job. When Pages is not enabled or the token cannot enable it, the tested artifact is still available. A repository administrator can select **Settings → Pages → Source: GitHub Actions**, then rerun the workflow. Do not advertise a live URL until deployment succeeds.

## Credits

SH2ZAM is inspired by SH4ZAM by Falco Girgis and SH4ZAM Contributors. Original licensing and acknowledgements remain in the repository. No Sega or upstream endorsement is implied. See the root `LICENSE` and `NOTICE.md`.
