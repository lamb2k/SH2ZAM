# Exact workload optimizations on the SH-2 model

This suite measures complete workloads: rotating many vectors by one quaternion,
querying static boxes with many moving spheres, and calling selected scalar math
functions. Each comparison includes the setup, validation, fallbacks and result
writes required by that path. The benchmark is intended to identify useful
workload gains with exact results; it does not assume a universal 10× speedup.

## Recorded results

[Native SH-2 CI 43525421998](https://github.com/lamb2k/SH2ZAM/actions/runs/43525421998)
executes source [`598a6247`](https://github.com/lamb2k/SH2ZAM/commit/598a6247fe5a75320f2648e716a21db48f0eaa4c).
The [evidence artifact](https://github.com/lamb2k/SH2ZAM/actions/runs/43525421998/artifacts/14084553596)
contains the target ELF, symbols, disassembly and raw observations. The
[complete dataset](results.json) retains all 44 cases and all seven samples per
case. ELF SHA-256: `4bed1cd10ee00f3e3b9db71ad454b136a5e24d1ef8823f47eb0b46ed43e9d3a8`.

The 512-vector bounded rotation batch uses **47,134 modeled cycles**,
including quaternion setup, input scanning and native qualification. This is
**24.79×** faster than repeated prior scalar calls
(1,168,257 cycles), **2.21×** versus an existing
matrix-plus-C batch (104,382), and **1.0072×**
versus the existing matrix-plus-MAC composition (47,475). The
last comparison is only **0.72%** less cost.
The large repeated-call gain comes from sharing setup; it does not establish
that the underlying MAC kernel became 24× faster.

Spatial results below include **one complete index build, all 128 distinct
queries, and every result mask and hit count**. Each row compares against the
faster of the two independent C output strategies in that same compiled ELF.
All figures are medians of seven samples; ratios divide the displayed medians.

| Profile | Boxes | Faster independent C | C cycles | Index + queries | Speedup |
| --- | ---: | --- | ---: | ---: | ---: |
| sparse | 128 | Scatter | 505,756 | 258,961 | 1.95× |
| sparse | 512 | Scatter | 1,977,201 | 750,200 | 2.64× |
| mixed | 128 | Packed | 563,641 | 367,307 | 1.53× |
| mixed | 512 | Scatter | 2,192,272 | 926,749 | 2.37× |
| dense | 128 | Packed | 1,581,743 | 168,536 | 9.39× |
| dense | 512 | Packed | 6,291,196 | 626,075 | 10.05× |
| crowded | 128 | Packed | 1,449,410 | 1,381,000 | 1.05× |
| crowded | 512 | Packed | 5,762,615 | 5,434,385 | 1.06× |
| full-range | 128 | Scatter | 498,541 | 253,712 | 1.96× |
| full-range | 512 | Scatter | 1,947,715 | 668,186 | 2.91× |

The **10.05×** result applies to the 512-box dense/contained fixture, where each
sphere contains the entire scene and the index can accept the root. It is a
median for that workload, not a minimum over every sample or a universal
collision-query gain. Sparse, mixed, crowded and full-range cases remain in
the record. All ten spatial medians improve on their faster independent C
control, with substantially smaller gains in crowded partial-overlap scenes.

Using the new MAC wrapper for rotation, the index for spatial queries, and the
new scalar kernels, the site's strongest-control overview reports **30 faster
cases and 14 slower cases**. Only **one of those 44 cases reaches 10× against
its fastest recorded control**. The slower cases are rotation configurations
where an existing composition or scalar call is cheaper; every ordinary-batch
alternative also remains visible. For example, the full-range 512-vector
ordinary batch takes 104,086 cycles, while the optional MAC wrapper's rejected
qualification adds work and takes 118,986 cycles.

Native comparison verifies **173,425 reference output words and 585,060 exact
path comparisons**, with zero mismatches, unchanged inputs, preserved output
padding, ABI checks and SR.S=1 fallback checks. The existing 48-operation math
suite and 315-pose vertex suite also pass. [Host CI](https://github.com/lamb2k/SH2ZAM/actions/runs/43525422024)
passes all 17 CTest targets under GCC, Clang and Clang ASan/UBSan.

These are single-CPU, flat-RAM SH-2 instruction-model results. Physical Saturn
performance, cache/bus effects, alignment and dual-CPU scaling remain unmeasured.
No browser execution time or modeled-cycle-to-FPS conversion is used.

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
| `api_brute` | For every query, loop over all boxes with the frozen prior public sphere/AABB predicate; assemble each output word with a progressing bit mask and write the complete mask and count. |
| `inline_brute` | Use the independent ordinary-C predicate, with its radius square prepared once per query and compiler inlining enabled; pack each output word in a register using a progressing bit mask. |
| `inline_scatter` | Use the same independent predicate and per-query preparation; clear the output mask, then set the original-index bit in memory for each hit. |
| `index` | Build the complete index once, then run all 128 queries and write every mask and count. |

All three brute controls skip a negative-radius query before the inner loop. The
independent implementation in [`standalone.c`](standalone.c) uses full-range,
overflow-safe integer arithmetic and calls no SH2ZAM geometry or arithmetic
routines. Both independent variants allow the compiler to inline the predicate
and hoist invariant sphere work; neither requires an external predicate call
for each pair.

The packed and scatter variants execute in the same ELF on identical inputs
and produce the same masks and counts. Packing advances the bit mask for every
box and writes once per output word; scattering performs indexed bit-setting
only for hits after clearing the output. Their costs can differ with hit
density, so neither is presumed fastest for every workload. Retain both
comparisons. When reporting one adoption ratio for a case, use the lower of
their median cycle counts as the independent C reference. Both controls still
use brute-force search; this suite does not claim superiority over all other
spatial indexes.

Normalization and inverse-tangent records use `prior` and `current` loops over
the same inputs. They do not reuse results from previous samples or queries.

## Workloads and accounting

[`manifest.json`](manifest.json) declares 44 cases, each with seven deterministic
samples. [`workload_bridge.c`](workload_bridge.c) defines the exact input recipes.
With both independent spatial controls, complete fixture coverage requires
1,281 path executions, 173,425 reference output words and 585,060 exact
comparisons against those reference words. These counts describe coverage,
not a performance result.

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
