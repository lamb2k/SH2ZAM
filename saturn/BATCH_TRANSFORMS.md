# Batch transforms

SH2ZAM provides two exact batch APIs. The existing
`sh2z_mat34_transform_points` checks the matrix once per call and supports the full
Q16.16 input range, including in-place operation. The new prepared API scans an
immutable mesh once, then qualifies each matrix against that mesh's bounds. When
that stronger proof succeeds, its loop can omit saturation checks as well as
per-product overflow bookkeeping. Both APIs preserve the original floor-once,
translate, saturate result bit for bit.

Experimental native MAC.L and DMULS.L kernels are separate opt-in entry points.
They are not selected by the default prepared dispatcher. Physical-console timing
and execution remain unmeasured; cross-compilation, host tests and browser timing
do not qualify an SH-2 performance claim.

## Prepared API: cache a mesh, qualify each matrix

```c
#include <sh2zam/sh2zam.h>

static const sh2z_vec3_t vertices[2] = {
    {SH2Z_ONE, 0, 0},
    {0, SH2Z_ONE, 0}
};
sh2z_vec3_t transformed[2];
sh2z_mesh_view_t mesh;
sh2z_transform_plan_t plan;

/* Once when this immutable mesh is loaded. */
bool ok = sh2z_mesh_view_init(&mesh, vertices, 2);

/* Once for each new model matrix; the plan copies the matrix. */
sh2z_mat34_t model = sh2z_mat34_rotation_y(SH2Z_ANGLE_QUARTER);
if (ok) ok = sh2z_transform_plan_init(&plan, &mesh, &model);
if (ok) ok = sh2z_transform_plan_execute(transformed, &plan);

/* plan.fast_path reports eligibility, not whether initialization succeeded.
 * A valid plan with fast_path == false executes the full-range fallback. */
```

The view constructor scans the actual positions and records the largest raw
coordinate magnitude on each axis. Callers do not supply an assumed bounding box.
The plan constructor copies the matrix, source pointer and count, and proves the
bounds once. It returns `true` for both eligible and fallback plans. Its source
view and original matrix object can then be changed or discarded without changing
the plan; the borrowed source vertex array must remain alive and unchanged.

A subrange can reuse the same plan without a new scan or matrix qualification:

```c
/* Reads vertices[1], writes transformed[0]. */
if (ok) ok = sh2z_transform_plan_execute_range(transformed, &plan, 1, 1);
```

The range output begins at `out[0]`, not `out[first]`. The executor checks
`first <= plan.count` and `count <= plan.count - first`, avoiding overflowing
`first + count`. This supports benchmark chunks and application partitions while
preserving the proof for the complete mesh.

### Ownership, mutation and failure

The public structs are caller-owned storage with read-only fields after their
constructors return. Initialize them through their API functions rather than
filling in bounds or changing `fast_path` manually. There is no allocation,
vertex copy, hidden cache/DMA work, scheduler or global matrix state.

Prepared output must be disjoint from the **entire source mesh**, even when only
one subrange is being read. It must also be disjoint from the plan object.
Nonempty exact and partial overlaps are rejected before any write. This prevents
a transform from invalidating cached bounds for a later call. Use the existing
batch API when in-place transformation is required. After changing any source
vertex, rebuild the mesh view and its plans before executing them again.

`sh2z_mesh_view_init` accepts a null source for zero vertices; it rejects a null
view, a nonempty null source, a byte length that cannot be represented, and
view/source overlap. `sh2z_transform_plan_init` requires nonnull plan, view and
matrix even for an empty mesh and rejects plan/source overlap. Failed constructors
leave their output object unchanged. The input view must have been initialized.

Execution requires a valid initialized plan even for zero vertices. A valid empty
range accepts null output; nonempty null output and invalid ranges return `false`
without writing. Array storage must cover the declared counts. Use naturally
aligned contiguous `sh2z_vec3_t` arrays; extra cache-line alignment is not required.
Independent calls can share an immutable plan and input when their outputs do not
overlap. CPU scheduling and shared-memory/cache coherence remain application work.
The exact declarations and contracts are in the
[public header](include/sh2zam/sh2zam.h).

### Why the prepared proof permits a no-clamp loop

Let `B[0..2]` be the largest absolute **raw integer** coordinate on each scanned
axis. For each matrix row, preparation checks:

```text
abs(m0) * B[0] + abs(m1) * B[1] + abs(m2) * B[2]
    + abs(translation) * 65536 <= 2^47 - 1
```

Magnitudes use unsigned arithmetic, including for `INT32_MIN`. Each individual
magnitude product fits `uint64_t`. The check consumes a decreasing unsigned budget
rather than adding arbitrarily large products, so even an ineligible full-range
matrix cannot overflow the proof calculation.

Every actual product, partial dot sum, and dot sum plus scaled translation has
absolute value at most `2^47 - 1`. Consequently the raw accumulator fits signed
64-bit storage, and the final Q16.16 integer fits `int32_t`. Translation is integral
in raw output units, so:

```text
floor(dot_raw / 65536) + translation
    == floor((dot_raw + translation * 65536) / 65536)
```

The right side lies within the signed 32-bit output range. Omitting the per-vertex
clamp therefore changes no results. This conservative proof can reject a transform
whose actual vertices would fit through cancellation; rejection selects the exact
full-range implementation and loses only the optimization.

The [portable prepared kernel](src/sh2zam_prepared.c) expands all three rows,
captures the 12 matrix values outside the vertex loop, loads each vertex's xyz
once, accumulates three full-precision products per output, floors once, and adds
translation. No product is rounded early. Its
[unsigned-bit floor helper](src/sh2zam_prepared_internal.h) implements division by
65536 rounded down, including negative fractions, without a signed right shift
or signed negation of `INT64_MIN`. The proven output range makes the final cast
to `int32_t` defined. Normal C multiplication lets the target compiler generate
instructions; this portable kernel contains no SH assembly or WASM intrinsics.

## Prior safe batch: automatic matrix-only gate

Existing callers continue to use the prior safe batch without creating a view or
plan:

```c
#include <sh2zam/sh2zam.h>

sh2z_mat34_t model = sh2z_mat34_rotation_y(SH2Z_ANGLE_QUARTER);
sh2z_vec3_t vertices[2] = {{SH2Z_ONE, 0, 0}, {0, SH2Z_ONE, 0}};
sh2z_vec3_t transformed[2];
bool ok = sh2z_mat34_transform_points(transformed, &model, vertices, 2);
/* Exact in-place operation is supported by this API. */
ok = sh2z_mat34_transform_points(vertices, &model, vertices, 2) && ok;
```

Both `sh2z_mat34_transform_points` and the original
`sh2z_mat34_transform_points_general` accept every valid Q16.16 input and matrix
value. Zero count returns `true` even with null pointers; a nonzero count with any
null pointer returns `false` without writing output. Their input/output arrays
must be disjoint or exactly the same array; other overlap is unsupported. Keep the
matrix separate from writable output. The general entry point remains available
for direct full-range reference comparisons.

For each basis row, the automatic dispatcher checks:

```text
abs(m[row][0]) + abs(m[row][1]) + abs(m[row][2]) <= UINT32_MAX
```

Every signed 32-bit coordinate has magnitude at most `2^31`, so the sum of absolute
raw products is at most:

```text
2^31 * (2^32 - 1) = 2^63 - 2^31 < INT64_MAX
```

Every partial and complete dot sum therefore fits signed 64-bit storage, including
products involving `INT32_MIN`. The gate uses checked unsigned additions. It
requires no source scan, immutable-mesh lifetime or caller-supplied bounds.
Translation is added after Q16 extraction, where both it and the quotient fit
comfortably in `int64_t`. This weaker proof does not guarantee an `int32_t` output,
so each output still needs a clamp:

```c
int64_t raw = product(m0, x) + product(m1, y) + product(m2, z);
int32_t result = saturate_int32(floor_divide_by_65536(raw) + translation);
```

The [safe batch implementation](src/sh2zam_transform.c) expands the three rows
and snapshots the matrix outside the vertex loop. It loads all input coordinates
before writing output to preserve exact in-place use. Its native build uses the
existing `dmuls.l` product helper. Matrices outside the matrix-only bound select
the original general implementation, which keeps quotient/remainder bookkeeping
to handle full-range sums safely.

## Experimental SH-2 fused kernels

Include [sh2zam/batch_experimental.h](include/sh2zam/batch_experimental.h) to use
`sh2z_transform_plan_execute_mac` or `sh2z_transform_plan_execute_dmuls`. They take
`(out, plan, first, count)` and preserve the prepared range, overlap, rounding,
fallback and failure contracts. Their availability queries report whether a
nonempty valid call would use the native candidate; they do not validate buffers.

The [MAC.L candidate](src/sh2zam_batch_sh2.c) clears the accumulator for each row,
accumulates its three products in MACH:MACL and uses XTRCT to extract bits 47:16
before adding translation. The prepared bound makes those bits the exact signed
Q16 result. The DMULS.L candidate uses native products with ADDC carry propagation,
then the same extraction. The register and instruction semantics are specified in
the [Renesas SH-1/SH-2/SH-DSP Software Manual, rev. 5](https://www.renesas.com/en/document/mas/sh-1sh-2sh-dsp-software-manual),
sections 6.1.3 (ADDC), 6.1.20 (DMULS.L), 6.1.31 (MAC.L), and 6.1.72 (XTRCT).

MAC.L additionally requires the executing CPU's SR.S bit to be clear. The wrapper
reads the bit and falls back when it is set; it never changes saturation or
interrupt controls. Both candidates fall back on non-SH-2 builds, with
`SH2Z_FORCE_PORTABLE`, or when the prepared bound fails. Interrupt handlers that
use MACH/MACL must preserve them, as with the existing native product backend.
No dual-CPU scheduling is introduced. Consult the experimental header and source
before integrating the candidates; neither is selected by default dispatch.

## Validate and measure

The host CTest suites provide independent correctness evidence:

| Test | Coverage |
|---|---|
| [`batch_transform_oracles`](tests/test_batch.c) | Prior safe/general/scalar equivalence to a 128-bit oracle, full-range sums, cancellation, rounding, saturation, in-place use and tails |
| [`prepared_transform_oracles`](tests/test_prepared.c) | Prepared/range/general equivalence to a 128-bit oracle, exact and just-over budget edges in every row, translation, full-range fallback, matrix/view snapshots, immutable source, empty/null/range/overlap contracts |
| [`native_batch_host_models`](tests/test_batch_native.c) | Host models of native product/carry/extraction arithmetic plus wrapper availability and fallback contracts; no SH-2 instructions executed |

The shared [fixture test](benchmarks/test-fixtures.c) checks all 15 meshes and 21
poses, five wrapper outputs, fixed-point equality, float error, full/range parity,
and source immutability. GCC/Clang Release checks and ASan/UBSan run the meaningful
numeric assertions. Actual SH-2 cross-builds and linked-image audits are separate
from those host checks.

The benchmark's headline comparison is now **software float → prior safe batch →
portable prepared**, with separate MAC.L and DMULS.L candidate measurements on
Saturn. Native schema 3 records all five timings, four fixed-point checksums,
availability masks and plan preparation cost. Preparation is amortized over 64
calls and added once per whole mesh before taking the median; kernel-only values
remain available. Browser capture directly includes one plan qualification per
prepared batch and also reports component timings. The one-time mesh scan and
float conversion are outside transform timing and are declared exclusions.

Use [benchmarks/README.md](benchmarks/README.md) for the raw capture format,
32-vertex FRT chunk methodology, 21-pose timing order and provenance requirements.
Only physical-console captures can establish Saturn speedups or justify later
alignment, cache, batch-size and dual-CPU changes. Generated instruction audits
and current evidence are tracked in [VALIDATION.md](VALIDATION.md).
