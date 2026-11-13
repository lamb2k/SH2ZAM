# Exact workload optimizations on the SH-2 model

This suite measures complete workloads: rotating many vectors by one quaternion,
querying static boxes with many moving spheres, and calling selected scalar math
functions. Each comparison includes the setup, validation, fallbacks and result
writes required by that path. The benchmark is intended to identify useful
workload gains with exact results; it does not assume a universal 10× speedup.

Native measurements for this suite are pending. Use the API and measurement
contracts below to evaluate a capture; no speedup is established by the host
tests or by adding a batch entry point.

## Choose the API for the workload

| Workload | Entry point | What can be shared or avoided |
| --- | --- | --- |
| One quaternion rotates many vectors | `sh2z_quat_rotate_vectors` | Normalize the quaternion and construct its matrix once for the whole batch. |
| The same batch can use the experimental native MAC kernel | `sh2z_quat_rotate_vectors_mac` | Share quaternion setup, then qualify the existing exact MAC.L path once. |
| Many spheres query an unchanged set of AABBs | `sh2z_aabb_index_build`, then `sh2z_aabb_index_query_sphere` | Build one index, prune disjoint subtrees, and accept fully contained subtrees. |
| Independent normalization or inverse-tangent calls | Existing `sh2z_vec3_normalize`, `sh2z_atan2`, `sh2z_atan` | Keep the existing API and compare any arithmetic changes against the version at the start of this work. |

### Quaternion rotation batches

Include [`sh2zam/rotation_batch.h`](../../include/sh2zam/rotation_batch.h):

```c
#include <sh2zam/rotation_batch.h>

bool rotate_mesh(sh2z_vec3_t *out, sh2z_quat_t rotation,
                 const sh2z_vec3_t *in, size_t count)
{
    return sh2z_quat_rotate_vectors(out, rotation, in, count);
}
```

The output is bit-identical to calling `sh2z_quat_rotate(rotation, in[i])`
for every vector, including full-range input, zero quaternions, flooring and
saturation. The quaternion is normalized and converted to a matrix once per
call. The matrix has zero translation, so the ordinary batch implementation is
the exact existing composition of `sh2z_quat_to_mat34` followed by
`sh2z_mat34_transform_points`.

That composition was already available to callers. A gain over repeated scalar
rotation therefore measures the benefit of sharing setup. Compare against the
`composed` control below to determine whether the new API also improves on
an application that already shares setup.

Exact in-place operation (`out == in`) is supported. Other input/output overlap,
a byte count overflowing `size_t`, and null buffers for a nonempty batch return
`false` before any output write. An empty batch accepts null buffers and performs
no math. There is no persistent cache, allocation or retained input lifetime.

`sh2z_quat_rotate_vectors_mac` is an explicit experimental alternative. Its cost
includes matrix construction, input-bound scanning and plan qualification. It
uses the existing fused native kernel only for an out-of-place batch of at least
eight vectors whose
range proof succeeds, with `SR.S` clear on the current SH-2 CPU. In-place calls,
unproved output ranges, `SR.S=1`, non-SH-2 builds and forced-portable builds use
the ordinary exact matrix batch. The API reads SR without changing it. Interrupt
handlers that use MACH/MACL must preserve them, as with the existing native
multiplication backend. Both APIs use a direct scalar rotation for one vector;
the MAC API skips input scanning and qualification below eight vectors. Eight
is the smallest measured profitable bounded MAC batch in the initial capture,
not a universally optimal break-even point. Compare against `composed_mac`, which already makes
the prepared MAC path available to the caller.

Small batches may not amortize qualification or wrapper costs. A new quaternion
requires new setup; dividing a single batch into many calls repeats that work.
Both entry points preserve the general numeric contract rather than requiring
the application to limit coordinates silently.

### Static AABB index and moving-sphere queries

Include [`sh2zam/spatial.h`](../../include/sh2zam/spatial.h). Build once after
loading or changing the scene's boxes, then query it with different spheres:

```c
#include <sh2zam/spatial.h>

enum { BOX_CAPACITY = 512, HIT_WORDS = (BOX_CAPACITY + 31) / 32 };
static sh2z_aabb_node_t scene_nodes[2 * BOX_CAPACITY - 1];
static uint32_t build_ids[BOX_CAPACITY];
static sh2z_aabb_index_t scene_index;

bool rebuild_scene_index(const sh2z_aabb_t *boxes, size_t count)
{
    if (count > BOX_CAPACITY) return false;
    return sh2z_aabb_index_build(&scene_index, boxes, count,
        scene_nodes, 2 * BOX_CAPACITY - 1, build_ids, BOX_CAPACITY);
}

bool query_scene(sh2z_sphere_t sphere, uint32_t hits[HIT_WORDS],
                 size_t *hit_count)
{
    return sh2z_aabb_index_query_sphere(hits, HIT_WORDS, hit_count,
                                      &scene_index, sphere);
}
```

Call `query_scene` only after a successful build. `true` means the operation
succeeded; an empty intersection set is a successful query with zero set bits.
For original box index `i`, test `hits[i >> 5] & (UINT32_C(1) << (i & 31))`.
The optional `hit_count` receives the number of set bits.

The caller owns every buffer. Source boxes and populated nodes must stay alive
and unchanged while the index is queried. Rebuild after changing any box;
moving the query sphere does not require rebuilding. Scratch IDs are only
needed during construction and can be reused after the build. Separate indexes
are reentrant; shared mutable buffers require caller synchronization.

For `V` valid boxes, construction needs `2*V-1` nodes and `V` scratch IDs, with
no nodes or IDs needed when `V` is zero. A box with `min > max` on any axis is
invalid and is omitted, matching the primitive predicate's false result. Its
original bit remains zero; the output ID space is never compacted. Reserving
`2*N-1` nodes and `N` IDs for `N` input boxes is sufficient without a preliminary
validity pass. The count limit is 2³¹, additionally constrained by representable
buffer lengths and available address space.

Build buffers must be naturally aligned and disjoint from the source boxes,
index and one another. Query outputs must be naturally aligned and disjoint
from the index, its boxes/nodes and the optional hit count. Invalid pointers,
capacities, byte spans or overlaps return `false` before any output or workspace
write; this check cannot establish whether an arbitrary pointer refers to live
allocated storage. The caller must provide valid buffers and an intact index.

Queries write exactly `ceil(N/32)` words. Unused bits in the last word are zero;
additional capacity is untouched. Empty input permits null buffers where no
storage is required. Negative sphere radii produce an empty hit set. Bounding
and containment calculations cover the complete signed 32-bit coordinate range
without overflowing, and every reported bit agrees with the exact
`sh2z_sphere_intersects_aabb` contract.

#### Memory budget

Each node is 32 bytes. For all-valid input, the caller's storage is:

| Boxes | Node storage | Build scratch | Nodes + scratch | Source boxes | One output bitset |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 128 | 8,160 B | 512 B | 8,672 B | 3,072 B | 16 B |
| 512 | 32,736 B | 2,048 B | 34,784 B | 12,288 B | 64 B |

The table excludes the small index descriptor, optional hit count, call stack
and a shared 128-byte read-only bit-mask table. Scratch is reusable after
construction. A streaming caller can reuse one output bitset for successive
queries; the benchmark retains every mask and hit
count so it can compare all outputs. Source boxes are existing scene data, but
their storage still belongs in the application's total memory budget.

#### Which query patterns help?

The balanced tree uses conservative AABB bounds. Disjoint subtrees are skipped
without running the exact sphere/AABB predicate. If the farthest corner of a
subtree's bounds is inside the sphere, all leaves in that subtree are accepted.
Full containment of an all-valid root fills the original-index bitset directly.
When invalid input boxes exist, accepted leaves are enumerated to preserve
their original indices. Each sphere's squared radius is prepared once and reused
by the exact pointer-based narrow phase. Direct scans assemble a whole output
word in a register before writing it.

Separated objects and local queries offer useful pruning. Fully contained
scenes offer bulk acceptance. Crowded objects crossing a query's boundary can
force many tree visits and narrow tests; a small number of queries may also
fail to recover construction cost. After testing full spherical containment,
the implementation selects a direct scan when the query AABB covers at least
three quarters of the root interval on every axis. Clipped unsigned interval
spans make this selection safe at the signed coordinate endpoints; no volume
product is needed. This is a cost heuristic with no efficiency guarantee for
an arbitrary distribution. Both the scan and the tree produce exact results.
A query AABB containing the entire root always meets the scan criterion, and
its traversal could not reject any descendants. The fallback still pays the
already incurred build and query setup costs. Frequently moving boxes require
rebuilding and should be evaluated with those rebuilds included.

## Controls and baseline versions

The frozen prior library in [`prior/`](prior/) comes from
`3374d77c4625749cc932cafa4385ad18db811e9b`, the version at the start of this
optimization request. `prior/manifest.json` records each source hash, and thin
wrappers rename its public symbols so both versions can execute in the same
ELF. The historical [48-operation math suite](../math-model/README.md) instead
compares against `10786b6`. These are different comparisons: a cumulative gain
over `10786b6` does not establish a new gain over `3374d77`.

### Rotation controls

| Path | Work performed in the measured range |
| --- | --- |
| `scalar` | Call the frozen prior `sh2z_quat_rotate` for every vector, repeating its quaternion setup. |
| `composed` | Construct the matrix once with the frozen prior API, then call the prior matrix batch. |
| `composed_mac` | Construct the matrix once with the frozen prior API; scan the input and qualify the existing prepared MAC path; execute it when available, otherwise use the prior matrix batch. |
| `batch` | Call `sh2z_quat_rotate_vectors`, including its one-time setup and validation. |
| `mac` | Call `sh2z_quat_rotate_vectors_mac`, including its scan, qualification and exact fallback. |

The prepared MAC implementation is the existing library facility, not a newly
invented baseline. The three controls distinguish shared setup, an already
competent matrix composition, and the existing optional native kernel. Publish
the relevant comparisons together; a large ratio against `scalar` alone does
not demonstrate a similar gain over `composed` or `composed_mac`.

### Spatial controls

| Path | Work performed in the measured range |
| --- | --- |
| `api_brute` | For every query, loop over all boxes with the frozen prior public sphere/AABB predicate; write the complete mask and count. |
| `inline_brute` | Run an independent ordinary-C predicate inside the same loop, hoisting the sphere's radius square once per query and allowing the compiler to inline the predicate; write the same mask and count. |
| `index` | Build the complete index once, then run all 128 queries and write every mask and count. |

Both brute controls skip a negative-radius query before the inner loop. The
independent implementation in [`standalone.c`](standalone.c) uses full-range,
overflow-safe integer arithmetic and calls no SH2ZAM geometry or arithmetic
routines. It provides a competent direct-scan comparison without inflating the
reference through a mandatory external predicate call for every pair. It is
still a brute-force algorithm; this suite does not claim superiority over all
other spatial indexes.

Normalization and inverse-tangent records use `prior` and `current` loops over
the same inputs. They do not reuse results from previous samples or queries.

## Workloads and accounting

[`manifest.json`](manifest.json) declares 44 cases, each with seven deterministic
samples. [`workload_bridge.c`](workload_bridge.c) defines the exact input recipes.

| Group | Sizes | Profiles |
| --- | --- | --- |
| Quaternion rotation | 1, 8, 32, 128, 512 vectors | Bounded, tiny, full-range, identity quaternion, zero quaternion |
| Vector normalization | 32 vectors | Bounded, tiny, full-range |
| `atan2` and `atan` | 32 calls each | Bounded, tiny, full-range |
| Spatial queries | 128 or 512 boxes; 128 queries | Sparse, mixed, dense, crowded, full-range |

Bounded vector components are within approximately ±8 in Q16.16; tiny
components are within ±32 raw units. Full-range math profiles combine signed
32-bit inputs with explicit extreme/zero/one values. The rotation identity and
zero profiles use bounded vectors.

Spatial `sparse` inputs place boxes on a separated grid with moving local
queries. `mixed` uses a closer grid and varied radii. `dense` uses clustered
boxes fully covered by the query spheres, exercising root acceptance. `crowded`
uses partial coverage and overlapping bounds, where pruning can be weak.
`full-range` includes near-endpoint coordinates and invalid boxes. Profiles
other than dense also include negative-radius queries. Query centers/radii and
sample data vary; the index stores scene geometry rather than memoizing answers.

The measured interval includes the range wrapper, normal validation and exact
fallbacks, quaternion setup, any input scan/plan qualification, index
construction, all queries, and all observable result writes. Fixture generation,
output equality checks, SR.S guard reruns and host execution overhead are
outside the interval. Each spatial sample starts with a new build; build cost
is not amortized across the seven samples. Keeping an index across more
application frames can change the economics, but that is a separate workload.

Report median total modeled cycles per complete sample. `cyclesPerItem` divides
by vector/call count for math and by 128 for spatial queries; the latter still
includes one 128th of the build cost. `speedup` is baseline median cycles divided
by candidate median cycles. A value below 1 is a slowdown and must remain in
the results. Inspect the raw samples, instruction counts, reads/writes and
MAC/DMULS instruction counts alongside ratios. Do not combine unrelated APIs
into a single speedup claim.

## Reproduce and validate

Build the benchmark ELF with the pinned Yaul toolchain used by the
[`SH-2 cross compile` workflow](../../../.github/workflows/sh2-cross.yml), then
run the workload suite from the repository root:

```sh
MODEL_SUITE=workloads bash saturn/benchmarks/sh2-model/run.sh \
  saturn/benchmarks/yaul/build/sh2zam-benchmark.elf \
  saturn/benchmarks/yaul/build/sh2zam-benchmark.sym \
  build/sh2-model/workload-results.json
node saturn/benchmarks/workload-model/validate.cjs \
  build/sh2-model/workload-results.json
```

The capture produces `workload-raw.json`; [`collect.py`](collect.py) verifies
coverage, exact equality and provenance, and packages `workload-results.json`.
[`validate.cjs`](validate.cjs) independently recomputes derived metrics and
rejects missing cases, mismatched output records, altered frozen sources,
unsupported model metadata and a universal 10× claim. These gates do not reject
a valid slowdown or impose a performance target.

Keep the ELF, symbol table, disassembly, raw capture and packaged JSON together.
The result binds the compiler flags, source commit, input manifest, frozen
baseline, benchmark sources and interpreter to hashes. Compare actual native
outputs, including every spatial mask and count; matching a timing number or
checksum by itself is insufficient. Host tests provide independent numeric
oracles and buffer-contract checks, while native SR.S reruns check exact output
and status preservation without contributing to the measured medians.

## What the model establishes

The suite executes the actual big-endian SH-2 ELF with the pinned, unmodified
Yabause interpreter at `7e38821dbac265490f115e163c523a939acda759`. Reported costs
are the interpreter's per-opcode modeled cycles. Browser or host elapsed time
does not enter the comparison.

This is a one-CPU instruction model with flat RAM. It does not model Saturn
cache behavior, bus wait states, pipeline or multiplier contention, DMA,
interrupts, peripherals, or scheduling and shared-memory traffic from the
second CPU. It cannot establish physical Saturn timings, frame rate, or
whole-game gains. In particular, the index's additional memory traffic and
footprint need hardware measurement before choosing a scene's batch size,
alignment or placement.

## Licensing

The library, frozen comparison sources, fixtures and target bridge are MIT
licensed. The optional model capture/collection tooling follows the
GPL-2.0-or-later model harness and is not part of the shipping MIT library.
Yabause is fetched separately and remains unmodified; see
[`../sh2-model/LICENSE`](../sh2-model/LICENSE).
