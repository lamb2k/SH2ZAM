# Batch transforms

`sh2z_mat34_transform_points` has an automatic fast path for affine vertex
batches. It returns exactly the same Q16.16 results as the original scalar
implementation, including negative rounding and saturation. Native Saturn speed
still requires a physical-console capture; desktop and browser timings do not
establish SH-2 performance.

## Use

```c
#include <sh2zam/sh2zam.h>

sh2z_mat34_t model = sh2z_mat34_rotation_y(SH2Z_ANGLE_QUARTER);
sh2z_vec3_t vertices[2] = {
    {SH2Z_ONE, 0, 0},
    {0, SH2Z_ONE, 0}
};
sh2z_vec3_t transformed[2];

bool ok = sh2z_mat34_transform_points(transformed, &model, vertices, 2);
/* Exact in-place operation is also supported. */
ok = sh2z_mat34_transform_points(vertices, &model, vertices, 2) && ok;
```

Existing callers automatically receive the new batch implementation. For direct
comparisons, `sh2z_mat34_transform_points_general` exposes the original loop over
`sh2z_mat34_transform_point`. The automatic dispatcher also uses that function
when the matrix does not satisfy the fast-path bound.

Both entry points accept every valid Q16.16 input and matrix value. Zero count
returns `true` even with null pointers; a nonzero count with any null pointer
returns `false` without writing output. A `true` result describes a successful
transform, not which internal path ran.

Input and output arrays must be disjoint or exactly the same array. Partial
overlap is unsupported. Keep the matrix separate from writable output storage.
Use contiguous `sh2z_vec3_t` arrays with their natural alignment; no extra
cache-line alignment is required by the API. Callers retain responsibility for
buffer lengths, CPU scheduling, and cache/DMA coherence.

## Why the gate is sufficient

For each of the three basis rows, the dispatcher checks the sum of absolute
**raw integer coefficients**:

```text
abs(m[row][0]) + abs(m[row][1]) + abs(m[row][2]) <= UINT32_MAX
```

Every signed 32-bit coordinate has magnitude at most `2^31`. Consequently,
the sum of the absolute raw products is bounded by:

```text
2^31 * (2^32 - 1) = 2^63 - 2^31 < INT64_MAX
```

Every partial sum and the complete sum therefore fit signed 64-bit storage,
including products involving `INT32_MIN`. This proof covers the entire input
coordinate range. It avoids an additional vertex scan and requires no
caller-supplied coordinate promise.

The gate computes magnitudes with unsigned arithmetic, which handles
`INT32_MIN` without signed negation. It checks each unsigned addition before
performing it. A row exceeding the bound selects the general implementation
for the entire batch. Translation is excluded from the raw-product sum and is
added after Q16 extraction; at that point both the sum and translation fit
comfortably in signed 64-bit storage.

## Kernel

For each output coordinate, the optimized path evaluates:

```c
/* The matrix gate proves these additions cannot overflow. */
int64_t raw = product(m0, x) + product(m1, y) + product(m2, z);
int32_t result = saturate_int32(floor_divide_by_65536(raw) + translation);
```

This floors once after accumulation. It does not floor each product, truncate
negative fractions toward zero, change precision, or drop saturation. The
full-range implementation remains necessary for matrices whose raw products
could overflow before cancellation.

The batch kernel captures matrix coefficients outside the vertex loop, expands
the three rows explicitly, and loads each vertex's three coordinates before any
output write. That removes per-vertex row loops, temporary component arrays, and
scalar transform calls while preserving in-place operation. Each row is stored
as it is completed to keep accumulator lifetimes short. SH-2 builds reuse the
existing native `dmuls.l` product helper. No new hand-written assembly or
dual-CPU dispatch is introduced.

## Validate and measure

`tests/test_batch.c` compares both entry points and the scalar transform against
an independent 128-bit host oracle. It covers exact and just-over-bound matrices,
extreme inputs, cancellation, negative fractions, saturation, exact in-place
operation, odd batch tails, buffer boundaries, and pointer/count contracts.
It runs as the CMake/CTest `batch_transform_oracles` test and is compatible with
AddressSanitizer and UndefinedBehaviorSanitizer.

The shared exhibit benchmark compares software float, the general SH2ZAM path,
and this optimized path on the same inputs and matrix coefficients. Matrix
construction and input conversion stay outside transform timing. Record both
32-vertex chunks and useful whole-mesh sizes when checking setup overhead and
batch scaling. General and optimized fixed-point outputs must match exactly;
float error is checked separately.

Review generated SH-2 instructions for helper calls, register spills, and
scheduling before considering additional assembly. Use the bootable benchmark
and capture procedure in [benchmarks/README.md](benchmarks/README.md) for native
measurements. Only console measurements can establish the Saturn speedup or
justify subsequent alignment, cache, batching, or dual-CPU changes.
