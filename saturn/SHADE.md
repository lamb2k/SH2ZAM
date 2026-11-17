# Shading kernels (experimental, contract tag `shade-1`)

[`shade_experimental.h`](include/sh2zam/shade_experimental.h) adds integer kernels for the
per-frame shading work of a polygon renderer: directional light per model normal, 5:5:5
Gouraud packing from a lit colour, a conservative model-space back-face test, and a depth
fog that is a threshold table rather than a per-vertex divide. Nothing selects them by
default. Every function is batched over arrays with byte strides, has no callbacks, no
global state, no allocation, no floating point, no libc and no hardware-divider use.

## Status

- Contract tag `shade-1`. Struct layouts and numeric definitions are frozen at the tag; a
  faster path may replace an implementation only if it writes identical bytes.
- Every kernel reproduces an existing renderer's integer rules **exactly** (the next
  section lists them). The point is that a caller can adopt a kernel with no image change.
- Evidence tiers: **HOST** (independent oracle, exhaustive domains and randomised sets on
  GCC, Clang and sanitizers) and **MODELLED** (the compiled SH-2 code executed in the
  pinned Yabause interpreter, flat RAM). Nothing here has run on an emulator as a whole
  program or on a console.
- Portable C only. No hand assembly: the modelled C is within about 1.2 to 1.7 times of a
  hand-scheduled estimate (see [SH-2 code](#sh-2-code)), less than the per-call and
  data-layout gains below, so no `_divu` or assembly variant is part of this tag.

## API and the rule each one reproduces

| Function | Semantics (exact definitions in the header) | Rule reproduced |
| --- | --- | --- |
| `sh2z_shade_light_prepare` | Normalise a light and a reflection direction (`sh2z_vec3_normalize`), quarter them to Q14, rotate into model space by the transposed Q14 basis with truncating `/ 16384`; precompute one exact reciprocal per colour channel. | Vehicle light setup: per frame, before the per-normal loop. |
| `sh2z_shade_light_normals` | Per int8 normal: diffuse `= clamp(n . L, 0, 127*16384)`; channel `= min(255, ambient + diffuse*color/(127*16384))`; reflection index `= clamp((n . R) / 16384, -128, 127)` into a 256-byte table. Four bytes per normal. | Vehicle per-normal lighting with the 32-bit unsigned division and truncating signed division of the source. |
| `sh2z_shade_gouraud` | Per (base colour, lit normal) pair: `lit = blend(mul8(base, light), fog, 255 - light[3])` with the source's `(x + (x >> 8)) >> 8` rounding, then `16 + (lit >> 3) - (base >> 4)` clamped to 0..31 per channel, packed 5:5:5, plus a count of clamped channels. | Vehicle Gouraud corner colour (`shade`), including the saturation counter. |
| `sh2z_shade_solid` | The same lit colour as `0x8000 | (lit >> 3)` per channel. | Vehicle solid colour for clipped faces. |
| `sh2z_shade_planes_valid` | Load-time range check of plane records and face-id order tables. | (validation) |
| `sh2z_shade_cull` | Keep face `id` if `n . eye - plane >= -(3 + sum(abs(eye - first)) + sum(abs(n)) * eye_error)`; ids written in order (optionally through an order table, in place allowed). | Vehicle model-space plane cull with the conservative margin for an integer eye. |
| `sh2z_shade_fog_build` | The 256 fade thresholds of a fog linear in a projective depth ratio (`multiplier`, `offset`, `pa`, `pb`, `far_z`), then a bucket index. | Road fog table preparation (threshold per fade level, `far_z + 1` for never). |
| `sh2z_shade_fog_from_depth` | Bucket index for any caller table of non-decreasing thresholds. | (any threshold fog) |
| `sh2z_shade_fog_lookup` | Per depth: `fade = max { i : depth[i] <= max(z, 0) }`, through the bucket index. | Road fog lookup: identical to the source's 8-step binary search for every int32 depth. |
| `sh2z_shade_fade_offset` | Per fade: `16 + (fade*slope - bias) / 255` (truncating) clamped to 0..31 per channel, bit 15 set. | Road textured-piece Gouraud offsets (`slope = live - mean`, `bias = selected * (base - mean)`). |
| `sh2z_shade_fade_mix` | Per fade: `((near*(255 - fade) + far*fade)*31 + 32512) / 65025` per channel, bit 15 set. | Road solid-piece colour. |

Nothing in the listed source rules was left out or approximated. The one rule that could
not be met as first sketched is the plan's "256-entry table indexed by `z >> shift`": it
is not exact, and the measured difference is below.

### Reciprocals instead of divisions

The SH-2 has no divide instruction; C division calls a library routine. Each kernel
replaces the source's divisions with an exact multiply, proved for its whole domain and
also checked exhaustively by [`tests/test_shade.c`](tests/test_shade.c):

- **light channel** `d * c / (127 * 16384)` for `0 <= d <= 127 * 16384`, `c <= 255`:
  with `K = ceil(c * 2^29 / 127)` (prepared once per call), `(d * K) >> 43` is exact,
  because `d * K / 2^43` exceeds `d * c / D` by less than `D / 2^43 < 1 / D`, smaller
  than any non-zero distance from `d * c / D` to the next integer. One `dmulu.l`.
  Checked for every `d` from 0 to 2^21 - 1 and every `c`.
- **reflection index** `r / 16384` truncating: the magnitude is shifted (the SH-2 has only
  logical shifts by fixed counts).
- **`/ 255`** for `|x| < 65536`: `(|x| * 257 + 257) >> 16`, a 16 x 16 multiply.
- **`/ 65025`** for `x <= 2,048,287`: `(x * 2113633) >> 37`.
- **fog bucket** `u >> shift` (a variable shift, which the SH-2 lacks): the high word of
  `u * 2^(32 - shift)`.

### Depth fog: why the plain table is not exact

The source's thresholds `depth[i]` are multiples of 4 anywhere in `[0, far_z + 1]`: each is
`4 * (N / (m + 1) + 1)` for an integer `m` that falls with `i`. A table indexed by
`z >> shift` gives one fade per bucket of `2^shift` depths, so any bucket that contains a
threshold returns the wrong fade for part of its range, and a projective fog puts many
thresholds into the first buckets (fade climbs fastest near the camera). Measured on HOST
over 4,000 perspective tables (near plane 1..32 units, far plane up to 30,000 units, fog
start 0..996 of 1000) with depths uniform between the planes:

- the plain `bucket[z >> shift]` table is wrong for **9.18%** of depths, by up to **252**
  fade levels;
- **13.94%** of depths land in a bucket that holds a threshold; there the exact kernel
  searches the thresholds of that bucket only (mean 1.07 levels per searched bucket,
  widest 252, never more than the source's 8 steps).

`sh2z_shade_fog_lookup` therefore keeps the 256 thresholds and adds a 257-byte bucket
index (`bucket[b]` is the fade at `b << shift`, so the answer for a depth in bucket `b` lies
in `bucket[b] .. bucket[b + 1]`): one multiply, two byte loads and, for 14% of depths, a
short search. It equals the source's binary search for every int32 depth, including
negative depths and depths beyond the far plane.

The threshold builder `sh2z_shade_fog_build` reproduces the source's table bit for bit. It
steps the ceiling `(i - offset) * 2^30 / multiplier` exactly (one 64-bit division instead
of two per level) and divides `N / (m + 1)` only when the result can fall inside the far
plane, so it costs half the source's preparation.

For perspective parameters the thresholds are exactly where the direct per-vertex rule
`fade = clamp(floor(multiplier * ratio / 2^30) + offset, 0, 255)`,
`ratio = pa - floor(N / (z / 4))`, changes value: on HOST the table agrees with that rule at
256,000 sampled depths in `(0, far_z]`.

## Evidence (HOST)

[`tests/test_shade.c`](tests/test_shade.c) with the independent oracle
[`tests/shade_oracle.h`](tests/shade_oracle.h) (64- and 128-bit arithmetic, plain C
division, written from the rules above; it shares no code with the library). One run,
GCC/Clang strict and GCC ASan+UBSan, all passing, 57.7 million checks:

| Set | Size | Result |
| --- | --- | --- |
| Light channel, every diffuse 0 .. 2^21 - 1 x every colour | 541,065,216 channels | exact |
| Light, random lights/bases/normals incl. int8 extremes, zero directions | 40,000 trials, 10,061,408 normals (764,028 above the diffuse cap) | exact, prepared light exact |
| Gouraud, every base x light per channel, 6 fogs x 7 weights | 2,752,512 pairs, 1,214,682 saturations | exact, counts exact |
| Gouraud, random pairs over lit normals | 20,023,453 pairs | exact |
| Cull, random planes at full ranges, eyes at +/-30,000, eye errors 0..1024 | 4,532,240 faces, 777,578 within 1 of the boundary | exact |
| Fog tables, 2/3 perspective, 1/3 adversarial (extreme `multiplier`, `offset`, `pa`, `pb`, `far_z`) | 6,000 tables, 12,085,042 lookups | exact (table and the source's search) |
| Fog, caller tables (random, flat 0, flat max, dense then sparse) | 4,000 tables, every threshold and random depths | exact |
| Fade offset, every fade x every slope -127..127 x 24 bias sets | 1,566,720 colours | exact |
| Fade mix, every near x far x fade | 16,777,216 colours | exact |
| Whole vehicle pipeline (light -> cull through an order table -> Gouraud) vs the per-corner source loop | 600 models of 112 faces, 178,972 corner colours | exact, saturation counts exact |
| Reciprocal constants over their whole domains | `/127`, `/255`, `/65025`, light scales | exact |
| Argument rules | nulls, ranges, strides, alignment, overlap | rejected calls write nothing |

Negative controls (each must make the oracle disagree somewhere; all are detected):
basis not transposed (32,941 trials), diffuse not capped (25,594), reflection index
floored (34,210), blend weight `light[3]` instead of `255 - light[3]` (39,104), `mul8`
floored (38,366), cull on the plain dot sign (29,205), margin without the eye-error term
(21,717), fog from `bucket[z >> shift]` alone (5,464 tables), floor instead of truncating
division by 255 (4,496), mix without the `+32512` rounding (65,535).

## Evidence (MODELLED)

[`benchmarks/shade-model`](benchmarks/shade-model/README.md): the library compiled for
`-m2 -mb -O2` (GCC 13.2), executed in the pinned Yabause SH-2 interpreter (flat RAM, no
caches or wait states; not Saturn timing). Before timing, every output of every library
entry and every plain-C control is checked against the oracle: 632,538 normals, 2,574,879
Gouraud pairs, 509,311 faces, 4,000 fog tables, 2,555,263 fog lookups, 5,172,842 fade
colours and 205,072 frame corners.

"Plain C" is the same rule written the way a renderer computes it today (an unsigned
division per light channel, the per-corner routine, the per-face plane test, the binary
search, signed division per fade channel), on identical inputs. Cycles per element are the
slope between the one-element and the largest row (per-call cost removed):

| Kernel | Kernel cycles / element | Plain C cycles / element | Ratio |
| --- | ---: | ---: | ---: |
| Light, per normal | 99.4 | 169.9 | 1.71 |
| Gouraud, per pair | 134.2 | 236.4 | 1.76 |
| Cull, per face | 82.8 | 101.4 | 1.22 |
| Fog lookup, per depth (uniform depths) | 41.8 | 129.9 | 3.11 |
| Fog lookup, per depth (near-dense, log-uniform depths) | 79.7 | 133.8 | 1.68 |
| Fade offset, per fade | 82.5 | 165.6 | 2.01 |
| Fade mix, per fade | 80.0 | 154.0 | 1.93 |

| Whole call | Kernel cycles | Plain C cycles |
| --- | ---: | ---: |
| Fog threshold table build (once per fog change) | 90,658 | 191,492 |
| Light prepare + 1 normal | 3,170 | 2,711 |
| One-element call: cull / fog / fade offset / fade mix | 246 / 206 / 311 / 267 | 128 / 152 / 193 / 182 |
| Vehicle frame, 112 faces, 128 normals (light, cull, corner colours) | 55,258 (1.93 ms at 28.64 MHz) | 83,830 (2.93 ms) |
| Same frame with unique (palette, normal) pairs baked offline | 70,958 (2.48 ms) | |

The per-call cost is argument checking (strides, alignment, overlap). The kernel is
already cheaper per call from two depths for the fog lookup, three fades for the fade
kernels, eight faces for the cull and eight normals for the light (whose prepare also
computes the three channel reciprocals). In the synthetic frame the baked-pair variant is slower because its corners come
from windows of six neighbouring normals, giving more unique pairs (328 on average) than
kept corners (51 kept faces, 204 corners); it wins when a model's unique pairs are fewer
than the corners drawn.

## SH-2 code

The library is portable C. The compiled loops are close to what hand scheduling would
give, measured by modelled instructions per element (kernel / hand estimate): Gouraud
124 / ~100, light 82 / ~70, cull 73 / ~45, fog 38 / ~22, fade offset 78 / ~55. The
remaining cost is the arithmetic itself (nine 16 x 16 multiplies per Gouraud pair; six
32-bit and up to three 64-bit multiplies per normal), not compiler overhead. The hand
figures are estimates from counting a straight-line schedule, not measurements. A hand-written variant would win
1.2 to 1.7 times on these loops; it is not part of `shade-1`, and can be added under a new
name if a measured frame shows one of these loops dominating.

The CI build checks that the SH-2 object needs no `memcpy`, `memset` or `memmove` (the
freestanding and wasm builds have none) and that it uses `dmulu.l` and `mulu.w`.

## Caller guidance

**Vehicle, per frame.** `sh2z_shade_light_prepare` once; `sh2z_shade_light_normals` over
all normals (four bytes each, the source's layout); `sh2z_shade_cull` over the octant's
order table into a kept list (validate planes and orders once at load with
`sh2z_shade_planes_valid`); then `sh2z_shade_gouraud` once over the (palette, normal)
pairs of the kept faces' corners. The kernels read native-endian 16-byte plane records
(`normal[3]`, `first[3]`, `plane`); on the big-endian SH-2 a baked big-endian record of
that layout is native, so a pack can embed them with a stride. Things that would make it
cheaper on the caller's side:

- store per face its plane record (normal, first corner position, plane) contiguously, and
  per corner the normal index, so the pair list is four 16-bit loads per kept face rather
  than byte-wise decoding of vertex records;
- if a model's unique (palette, normal) pairs are fewer than the corners drawn per frame,
  bake that list and four pair ids per face, colour the list once per frame and copy;
- light only when the light, the reflection direction or the basis changes; Gouraud only
  when the light or the fog colour changes; otherwise reuse the words.

**Road, per piece.** `sh2z_shade_fog_build` (or `sh2z_shade_fog_from_depth` with the
caller's own thresholds) when fog changes. Per piece, `sh2z_shade_fog_lookup` over the
corners' `z` (stride `sizeof(sh2z_view_guard_vertex_t)` or `sizeof(sh2z_view_vertex_t)`
from `&v[0].z`), then choose the variant from the fades, then `sh2z_shade_fade_offset`
(textured) or `sh2z_shade_fade_mix` (solid). Calling over all corners of a strip or of the
frame at once instead of per piece removes most of the per-call cost.

## Not established

- Any measurement on a console or an emulator system; the instruction model has no
  caches, wait states, bus contention or real multiplier latency.
- The whole-frame effect of adopting these kernels; that needs the caller's frame
  measured with and without them.
- Whether real models have fewer unique (palette, normal) pairs than drawn corners.
