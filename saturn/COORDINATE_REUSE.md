# Exact coordinate reuse

`sh2zam/reuse_experimental.h` provides an opt-in batch path for immutable meshes
with many repeated X coordinates and (Y,Z) coordinate pairs. It computes each
distinct contribution once per matrix, then writes every original output
vertex using two table reads and one exact sum per row. It preserves ordinary
contiguous xyz output and the existing Q16.16 rounding/saturation contract.

The ordinary batch and prepared APIs do not select this path automatically.
The browser bridge can opt in based on scanned coordinate counts; no scene name,
mesh generator or matrix-specific visual shortcut is part of this API.

## Work selection

Preparation scans the actual quantized coordinates and builds caller-owned
keys and per-vertex byte offsets. Reuse is enabled only when:

```text
vertex_count >= 128
2 * unique_YZ_pairs + unique_X_values <= vertex_count / 2
```

The second comparison is evaluated with subtraction/division so its own
integer arithmetic cannot overflow. Counts are monotonic, allowing preparation
to stop as soon as reuse cannot qualify. A fixed key budget also limits one-time
work and storage. Exhausting that budget yields a valid ordinary prepared
fallback. A false `mesh.reusable` does not mean the input is invalid.

The cost gate is a conservative work heuristic, not a mathematical speed proof.
It caps contribution-building multiplication work at roughly one sixth of the
nine-multiply-per-vertex transform. Table loads, metadata traffic, setup checks,
cache placement, batch size and the execution engine still determine speed.
Measurements must include plan preparation and complete output writes.

## Exact arithmetic

For each output row, plan preparation builds:

```text
X contribution[x_key] = m0 * x + translation * 65536
YZ contribution[yz_key] = m1 * y + m2 * z
```

The executor adds the two signed 64-bit contributions and floors once by 65536.
Translation is integral in output Q16 units, so moving it into the raw sum is
exact. The existing prepared bound includes translation:

```text
abs(m0)*max_abs_x + abs(m1)*max_abs_y + abs(m2)*max_abs_z
    + abs(translation)*65536 <= 2^47 - 1
```

This proves that both table entries, every intermediate and the translated
sum fit, and that the final floor fits `int32_t`. The kernel therefore needs no
per-output clamp. A matrix failing that proof uses the ordinary prepared
executor and its full-range general fallback. No coordinate precision is lost,
and no transformed result is reused across different matrices.

## Caller-owned storage

```c
#include <sh2zam/reuse_experimental.h>

enum { MAX_VERTICES = 4096, KEY_BUDGET = 256 };
static sh2z_reuse_index_t indices[MAX_VERTICES];
static sh2z_fx_t x_keys[KEY_BUDGET];
static sh2z_reuse_yz_t yz_keys[KEY_BUDGET];
static sh2z_reuse_term_t x_terms[KEY_BUDGET], yz_terms[KEY_BUDGET];
static sh2z_reuse_mesh_t mesh;
static sh2z_reuse_plan_t plan;

/* Once for an immutable source mesh. */
bool ok = sh2z_reuse_mesh_init(&mesh, positions, count,
    indices, MAX_VERTICES, x_keys, KEY_BUDGET, yz_keys, KEY_BUDGET);

/* Repeat for each new matrix. Include both calls in timing. */
ok = ok && sh2z_reuse_plan_init(&plan, &mesh, &matrix,
    x_terms, KEY_BUDGET, yz_terms, KEY_BUDGET);
ok = ok && sh2z_reuse_plan_execute(output, &plan);
```

`plan.enabled` identifies actual table execution; a successful call may instead
use the prepared fallback. Insufficient term capacity produces that fallback
without modifying the tables. Range execution takes `(out, plan, first, count)`
and writes the selected source range starting at `out[0]`.

Per-vertex metadata occupies eight bytes: two 32-bit byte offsets, avoiding a
per-vertex multiplication by the 24-byte term stride. With equally sized key
budgets K, the complete workspace is `8*N + 60*K` bytes, excluding the mesh/plan
objects and existing source/output arrays. N=4096 and K=256 use 48,128 bytes.
Choosing smaller capacities is valid; reducing them can cause fallback.

The implementation allocates no heap memory and has no mutable global state.
Key interning uses bounded linear searches during one-time mesh preparation.

## Lifetimes and overlap

The source positions, mesh object, keys and indices must remain immutable and
alive throughout a plan's lifetime. Source edits require a fresh mesh build.
The matrix is copied by plan initialization, so later changes to the original
matrix do not change an initialized plan.

Contribution tables belong exclusively to the plan using them. Rebuilding
tables for another matrix changes their contents, so separate simultaneously
active plans need separate table buffers. The caller synchronizes setup and
execution, including cross-CPU access. This API does not launch CPUs or alter
cache/DMA policy.

Constructors reject null buffers with nonzero capacities, unrepresentable byte
lengths and forbidden overlap before writing. Key-budget exhaustion is a
successful fallback preparation; partially populated keys/indices are then
ignored. Output must be disjoint from the entire source, mesh/plan objects and
all borrowed workspace capacity, including unused entries. Invalid ranges and
overlap fail before any output write. Natural alignment and actual backing
allocation lengths remain caller obligations.

## Verification

`coordinate_reuse_oracles` uses an independent signed 128-bit host oracle. It
covers full-range cancellation and saturation, negative fractional floors,
the exact prepared output bound, work-gate thresholds, key/term capacity
fallback, changed matrices, original output order, chunk tails, protected
unused workspace, and poisoned constructor failure state. GCC, Clang and
ASan/UBSan runs validate the portable implementation; they do not establish
physical Saturn performance.
