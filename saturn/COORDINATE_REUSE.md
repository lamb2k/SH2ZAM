# Exact coordinate reuse

`sh2zam/reuse_experimental.h` provides an opt-in batch path for immutable meshes
with many repeated X coordinates and (Y,Z) coordinate pairs. It computes each
distinct contribution once per matrix, then writes every original output
vertex using one exact sum per row. Repeated index patterns can share table
loads across eight adjacent outputs. It preserves ordinary
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
Measurements must include matrix qualification, all contribution-table
rebuilds and complete output writes. The workspace binding and immutable-mesh
indexing can be prepared once, with those setup costs reported separately.

## Blocks of eight

After building eligible mesh indices, preparation checks every complete
eight-vertex block starting at source indices 0, 8, 16, and so on. It sets
`mesh.blocks8` only when each block's actual indices have these patterns:

```text
X keys:    a b b a a b b a
YZ keys:   u u v v w w t t
```

The key values can differ between blocks, and keys within a block can coincide.
There is no scene identifier, generator assumption or source reordering.
Incomplete final blocks have no pattern requirement.

When the matrix proof also succeeds, the executor loads two X entries and four
YZ entries for each such block. That reduces table component loads from 48 to
18 and index-offset loads from 16 to 6 for eight vertices. Each of the 24 output
coordinates still receives its own exact sum, floor and store. No transformed
output is cached, and every matrix update continues to rebuild the contribution
tables in full.

Range execution processes up to seven scalar vertices to reach a source index
divisible by eight, uses complete proved blocks, then finishes any scalar tail.
It preserves the caller's original output order and works for short, odd or
unaligned ranges. A mesh that fails the block pattern keeps the ordinary
single-vertex reuse loop. Numeric or capacity fallback continues to use the
ordinary prepared executor.

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

/* Bind and validate workspace once, with the initial matrix. */
ok = ok && sh2z_reuse_plan_init(&plan, &mesh, &matrix,
    x_terms, KEY_BUDGET, yz_terms, KEY_BUDGET);

/* Repeat for each matrix. Include update and execution in timing. */
ok = ok && sh2z_reuse_plan_update_matrix(&plan, &matrix);
ok = ok && sh2z_reuse_plan_execute(output, &plan);
```

Initialization validates the complete source/workspace binding. Matrix updates
reuse that validation while proving the numeric bounds again and rebuilding
every used table entry. This includes updates whose matrix values equal the
previous matrix; the operation does not cache completed transforms or skip
matrix preparation. Call `plan_init` again when any binding or capacity changes.

`plan.enabled` identifies actual table execution; a successful call may instead
use the prepared fallback. Insufficient term capacity produces that fallback
without modifying the tables. Range execution takes `(out, plan, first, count)`
and writes the selected source range starting at `out[0]`.
When `plan.enabled` is true, `mesh.blocks8` identifies whether complete aligned
groups use the eight-vertex kernel; short prefixes/tails still use one vertex.

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
The matrix is copied by plan initialization and by each explicit matrix update,
so later changes to the original matrix do not change the plan.

Contribution tables belong exclusively to the plan using them. Rebuilding
tables for another matrix changes their contents, so separate simultaneously
active plans need separate table buffers. Plan bindings, capacities and workspace
allocations must remain unchanged between initialization calls; table contents
must remain unchanged between initialization or matrix-update calls. The caller
synchronizes setup, updates and execution, including cross-CPU access. This API
does not launch CPUs or alter cache/DMA policy.

Constructors reject null buffers with nonzero capacities, unrepresentable byte
lengths and forbidden overlap before writing. Key-budget exhaustion is a
successful fallback preparation; partially populated keys/indices are then
ignored. Output must be disjoint from the entire source, mesh/plan objects and
all borrowed workspace capacity, including unused entries. Invalid ranges and
overlap fail before any output write. Natural alignment and actual backing
allocation lengths remain caller obligations.

`plan_update_matrix` requires a successfully initialized plan. It rejects null
arguments and matrices overlapping the plan or either complete term allocation
before writing, preserving the prior plan and tables. Aliasing between read-only
matrix and source/metadata storage does not introduce a write hazard. A matrix
that fails the numeric bound still updates successfully into the exact fallback;
a later bounded matrix can re-enable reuse. Unused tables remain untouched while
fallback is active.

## Verification

`coordinate_reuse_oracles` uses an independent signed 128-bit host oracle. It
covers full-range cancellation and saturation, negative fractional floors,
the exact prepared output bound, work-gate thresholds, key/term capacity
fallback, changed and restored matrices, matrix snapshots, enabled/fallback
transitions, original output order, chunk tails, protected unused workspace,
poisoned constructor/update failure state, every block alignment and tail
length, and individual broken X/YZ equalities. GCC, Clang and
ASan/UBSan runs validate the portable implementation; they do not establish
physical Saturn performance.
