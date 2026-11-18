# Fused fade and colour (experimental, contract tag `shade-2`)

[`shade_experimental.h`](include/sh2zam/shade_experimental.h) gains, under its "Contract
tag shade-2" section, one call that does a road renderer's whole per-piece fog step: the
depth-fog fade of every corner, the piece's fog variant from the mean fade, and the 5:5:5
Gouraud word of every corner. [`cdiv_experimental.h`](include/sh2zam/cdiv_experimental.h)
adds scalar division by constants as inline magic-number multiplies. Nothing selects
either by default. Everything under `shade-1` ([SHADE.md](SHADE.md)) is unchanged: same
declarations, same structs and byte-identical SH-2 objects.

## Status

- Contract tag `shade-2`. Struct layouts and numeric definitions are frozen at the tag.
- Fades and variants are **exact** (shade-1's fade and the caller's variant rule). The MIX
  (solid) colour is **exact**. The OFFSET (textured) colour is within **+1 LSB per
  channel** of shade-1's `sh2z_shade_fade_offset`, and the only difference is stated
  exactly (below).
- Evidence tiers: **HOST** (independent oracle, exhaustive domains, randomised and
  adversarial sets, negative controls, on GCC, Clang and sanitizers) and **MODELLED** (the
  SH-2 code executed in the pinned Yabause interpreter, flat RAM). Nothing here has run
  on an emulator as a whole program or on a console.
- The fused kernel is hand-written SH-2 assembly with a portable C twin; the tests hold
  both to the same oracle.

## API

| Call | When | What it does |
| --- | --- | --- |
| `sh2z_shade_fogp_prepare(&fogp, &fog)` | once per fog change | copies the 256 thresholds of a shade-1 table and builds a 4,096-bucket index (5,132 bytes) |
| `sh2z_shade_ramp_offset(&ramp, live, base, level, threshold)` | once per frame | textured rule: live and base fog colours, the three pre-blend levels, the variant thresholds |
| `sh2z_shade_ramp_mix(&ramp, near_rgb, far_rgb, threshold)` | once per colour change | solid rule: the 256 mixed words, shade-1's `sh2z_shade_fade_mix` |
| `sh2z_shade_fog_gouraud(out, z, z_stride, count, &fogp, &ramp, mean)` | per piece or per strip | fades, variant and words for `count` depths at a byte stride; returns the variant |
| `sh2z_shade_fog_gouraud_refs(out, refs, z_offset, count, &fogp, &ramp, mean)` | per piece | the same, with each depth at `refs[i] + z_offset` (a list of corner pointers, no gather) |

The rules, word for word in the header:

- `fade(z) = max { i : depth[i] <= max(z, 0) }`, shade-1's fade, for every int32 z
  (negative z has the fade of depth 0).
- `v = [sum >= threshold[0] * count] + [sum >= threshold[1] * count]`, `sum` the sum of
  the call's fades; this is `floor(sum / count) >= t` without a division. Thresholds
  `{64, 192}` give the common `min(2, (mean + 64) / 128)`; a null threshold never selects.
- OFFSET: `slope = live - m`, `bias = level[v] * (base - m)`, `x = fade * slope - bias`,
  `c = clamp(16 + trunc(x / 255) + e, 0, 31)` with `e = 1` exactly when `x` is a negative
  multiple of 255. Without `e` this is `sh2z_shade_fade_offset`. Inputs: `live`, `base`
  and `mean` at most 127 (5-bit colours fit easily).
- MIX: `mix[fade]`, shade-1's mixed word, exact.

Checked once per call: null pointers, form, mean range, count at most 65,535, stride at
most 65,532 and a multiple of 4, alignment of `z` (or `refs`) and `out`. Not checked, to
keep the call small: `out` must not overlap the depths or the handles, the handles must
come from their prepare functions, and for the refs form every pointer is valid and
4-byte aligned at `z_offset`. `count == 0` returns 0 and writes nothing; a rejected call
returns -1 and writes nothing.

### Why +1 LSB on the textured form

The kernel forms `X = fade * (257 * slope) + C` with one 16 x 16 multiply and reads a
1,536-entry table at `X >> 16`. Because `257 * x / 65536` is `x / 255` to within a
fraction that never crosses an integer except at exact negative multiples of 255, the
table gives `trunc(x / 255)` everywhere but there, where truncation toward zero and the
table's floor-plus-one disagree. Over the whole input domain (every x in
`[-64770, 64770]`) 48 of 388,623 x per channel differ, by +1. They matter in practice at
full fog: at `fade = 255` with `level = 255`, `x = 255 * (slope - (base - m))` is always a
multiple of 255, so fully fogged textured corners whose live colour is below the
pre-blend base come out 1 LSB brighter in that channel. Removing `e` needs a wider
multiply and a remainder test per channel, which this tag does not pay for.

## Scalar division by constants

`cdiv_experimental.h` (tag `shade-2`): the family `SH2Z_CDIV_M(d, s)` (magic number),
`SH2Z_CDIV_E` (its error), `SH2Z_CDIV_N(d, s)` (a guaranteed domain, proven in the header),
`sh2z_cdiv_u32(n, m, s)` and `sh2z_cdiv_s32` (truncating), and three named helpers with
domains found exhaustively:

| Helper | Exact for | First wrong | Modelled cycles / value |
| --- | --- | --- | ---: |
| `sh2z_cdiv255_u32` | n <= 16,909,558 | 16,909,559 | 3 |
| `sh2z_cdiv255_s32` | \|n\| <= 16,909,558 | +-16,909,559 | 5 |
| `sh2z_cdiv65025_u32` | n <= 4,291,648 | 4,291,649 | 6 |

For comparison in the same model: a shift-add signed /255 valid for |n| <= 65,534 costs
12 cycles a value, C's signed `/ 255` costs 18 (a library call), and GCC already turns an
unsigned `/ 65025` into a multiply (6).

## Evidence (HOST)

[`tests/test_shade_fused.c`](tests/test_shade_fused.c) against the independent oracle
[`tests/shade_fused_oracle.h`](tests/shade_fused_oracle.h) (shade-1's source binary search,
64-bit arithmetic and plain C division; no bucket index, no magic numbers, no channel
table). 94,558,852 checks pass on GCC strict, Clang strict and GCC with ASan/UBSan:

- every x of the channel table domain equals the header rule;
- each named divide equals C division on every input of its domain (55,020,325 inputs) and
  is wrong one past it; the family is exact on 141,576,865 checked points of its
  guaranteed domains, exhaustively for small divisors;
- every reachable (slope, base - mean) pair x 256 fades x 7 levels (29,132,544 words)
  equals the oracle; 99.886% of those channels equal shade-1's exact form, the rest +1;
- 30,000 fused calls over perspective, adversarial and caller-made tables, counts 1..600
  and strides 4..36: every word, every fade (read back through an identity ramp) and
  every variant equal the oracle, and the refs form gives the same words through
  scattered pointers;
- 200,000 textured and 200,000 solid road pieces against the caller's per-piece rule:
  variants equal, solid words equal, textured channels within +1 (about 90% equal; the
  rest are the full-fog case above);
- 8 negative controls (bucket start only, negative depths as far, rounded mean, strict
  threshold, shade-1's exact channel, floor division, level 0 bias, slope sign) are each
  detected;
- 22 bad argument sets are rejected without writes; handles are read only.

## Evidence (MODELLED)

[`benchmarks/shade-fused-model`](benchmarks/shade-fused-model/README.md): the library
compiled for `-m2 -mb -O2` (GCC 13.2 from the pinned Yaul image) in the pinned Yabause
interpreter (flat RAM, no caches or wait states; not Saturn timing). The committed capture
is [`results.json`](benchmarks/shade-fused-model/results.json), from
[CI run 44207683875](https://github.com/lamb2k/SH2ZAM/actions/runs/44207683875) (source
commit `10fce21`), and equals a local run of the same source. Every output of every
entry is checked against the oracle before timing, including adversarial tables, depths
at and next to every threshold, `INT32_MIN`/`INT32_MAX`, and the argument rules of the
assembly entry points.

"Port C" (`plain_*`) is a renderer's per-piece C today: the 257-bucket lookup with a
search per corner, the variant, then signed shift-add division by 255 per channel
(textured) or a multiply for /65025 (solid). "shade-1" is the same work through
`sh2z_shade_fog_lookup` and `sh2z_shade_fade_offset`/`_mix` per piece. Rows include the
caller's loop around the call; 64 pieces of 4 corners per row:

| Cycles per vertex | shade-2 strided | shade-2 refs | shade-1 calls | Port C |
| --- | ---: | ---: | ---: | ---: |
| Textured pieces, uniform depths | 79.5 | 80.5 | 264.1 | 261.6 |
| Textured pieces, near-dense depths | 96.3 | 97.3 | 303.3 | 300.2 |
| Solid pieces | 54.7 | 55.7 | 240.8 | 248.4 |
| Mixed (1 in 8 solid, 1 in 8 clipped to 3..10 corners) | 75.6 | 76.6 | 255.2 | 259.4 |

| One call over a strip (textured) | 4 | 64 | 256 | 1,024 vertices |
| --- | ---: | ---: | ---: | ---: |
| shade-2, cycles per vertex | 76.6 | 48.9 | 46.7 | 46.1 |
| Port C over the same strip | 258.6 | 229.8 | 228.2 | 228.7 |

Per-vertex slope of a long call: 46.0 cycles textured (about 13 for the fade, 22 for the
three channels, the rest the stored fade and loops). Once per change: `fogp_prepare`
100,223 cycles, `ramp_offset` 1,433, `ramp_mix` 8,842.

A 4-corner textured piece costs about 318 cycles including the caller's loop. Counting
the unrolled path's instructions, roughly 45 are argument checks, 30 register saves and
restores, 55 the four fade lookups, 15 the variant, 40 the per-piece channel constants
and 80 the four words. The 40
cycle-per-vertex target is **not met**: about 80 at four corners a call and 46 per vertex
for whole strips. Reaching 40 at four corners would need the per-piece constant set-up and
argument checks gone (a call over many pieces with one prepared constant set per texture),
and a whole strip would still need about 6 cycles less per vertex than the present floor.

## Caller guidance

**Road, per frame.** When fog changes: build the shade-1 table
(`sh2z_shade_fog_build`, or `sh2z_shade_fog_from_depth` with the caller's thresholds) and
`sh2z_shade_fogp_prepare` it. Per frame: `sh2z_shade_ramp_offset` with the live and base
fog colours in 5-bit units, levels `{0, 128, 255}` and thresholds `{64, 192}`;
`sh2z_shade_ramp_mix` with the primitive and fog colours when they change. Per piece: one
`sh2z_shade_fog_gouraud_refs(shades, corners, offsetof(sh2z_view_vertex_t, z), count,
&fogp, solid ? &mix : &offset, image_mean)`; use its return value to choose the textured
variant. If corners are consecutive records, `sh2z_shade_fog_gouraud(shades, &v[0].z,
sizeof(v[0]), count, ...)` is 1 cycle a vertex cheaper. Calling once per strip rather
than per piece removes most of the per-call cost when the mean texture is shared.

## Not established

- Any measurement on a console or emulator system, and the whole-frame effect.
- Whether a renderer accepts the +1 LSB at full fog on textured corners.
