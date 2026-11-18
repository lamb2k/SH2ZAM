# Vehicle kernels (contract tag `vehicle-kernels-1`)

Per-draw kernels for an indexed mesh renderer: a few dozen to a few hundred textured
quads whose corners index shared positions and normals, drawn with Gouraud shading. After
back-face culling by plane, such a renderer projects the positions the kept faces use,
lights the normals they use, packs one Gouraud word per (palette, normal) pair and drops
the projected quads that face away. The four kernels below do those steps.

| API | Header | Output | SH-2 path |
|---|---|---|---|
| `sh2z_shade_screen_facing` | `shade_experimental.h` | exact | hand-written loop |
| `sh2z_view_batch_indexed`, `sh2z_view_batch_divu_indexed` | `view_experimental.h` | byte-identical to `sh2z_view_batch` | view-sh2-1 divider pipeline with an indexed load |
| `sh2z_shade_light_normals_indexed` | `shade_experimental.h` | byte-identical to `sh2z_shade_light_normals` | hand-written loop |
| `sh2z_shade_gouraud_approx` | `shade_experimental.h` | each 5-bit channel equal to `sh2z_shade_gouraud`'s or one less | hand-written loop |

Each header section is appended with its own include guard
(`SH2ZAM_SHADE_VEHICLE_KERNELS_1`, `SH2ZAM_VIEW_VEHICLE_KERNELS_1`). The sources are new
files (`src/sh2zam_vehicle.c`, `src/sh2zam_view_indexed.c`), so every earlier SH-2 object
is byte-identical. Portable builds (`SH2Z_FORCE_PORTABLE`, other CPUs) compute the same
results in C.

## Screen-space facing

`sh2z_shade_screen_facing(kept, &kept_count, faces, count, corners, vertices, view, sign)`
compacts a list of face ids. Face `id` has corners `corners[id][0..3]`, indices into
`vertices` (records from any `sh2z_view` projection). With `x[j]`, `y[j]` the corners'
`sx`, `sy`:

```
all    = AND of the four outcodes
simple = no corner has SH2Z_OUT_NEAR or SH2Z_OUT_FAR, every |x[j]|, |y[j]| <= 1023
area   = (x1-x0)(y2-y0) - (y1-y0)(x2-x0) + (x2-x0)(y3-y0) - (y2-y0)(x3-x0)
dropped when all != 0, or when simple and sign * area <= 0
```

`area` is twice the signed area of triangles 0-1-2 and 0-2-3, so a triangle stored with a
repeated corner works unchanged. A face that is not simple is kept (its corners need a
clip, which may not preserve orientation). `kept` may be `faces` itself. The kernel is
exact: there is no approximation.

The SH-2 loop uses the identity `area = (x2-x0)(y3-y1) - (x3-x1)(y2-y0)` (both are the
shoelace sum of the quad) and reads each corner's `sx:sy` as one long word. On the
diagonals, the low halves of the long-word differences are the coordinate differences
modulo 2^16, which are exact as int16 when every coordinate is within +/-1023. `MULS.W`
reads exactly those halves. The loop takes a face whose four outcodes are zero through
this path. That is valid because such a quad lies inside the clip rectangle, and the
wrapper only enters the loop when `view->clip` is inside +/-1023. For that reason the
vertices' LEFT/RIGHT/TOP/BOTTOM bits must be those of their `sx`, `sy` against
`view->clip`, which every projection writes. Faces with a non-zero outcode go to the
definition in C.

Fast-path conditions:
- `corners` and `vertices` are 4-byte aligned, and `faces` and `kept` are 2-byte aligned;
- the clip rectangle is within +/-1023.

Otherwise the call runs in C with the same results.

## Indexed view batch

`sh2z_view_batch_divu_indexed(out, v, positions, stride, count, index, n)` writes
`out[index[k]]` for `k < n`, each exactly the record `sh2z_view_batch(out, v, positions,
stride, count)` writes there. It leaves every other record untouched. Indices may repeat
and come in any order; they are not bounds-checked. The SH-2 form runs the view-sh2-1
transform and hardware-divider pipeline unchanged after an indexed load (index ->
`out + 20 i` and `positions + stride i`), so the per-vertex cost is that pipeline's plus
about 12 cycles. With 68 positions it saves time only below about 61 listed. Its use is
removing a gather (copying the referenced positions into a compact array and remapping
corner indices), not reducing the division work.

## Indexed lighting

`sh2z_shade_light_normals_indexed(out, 4, normals, 4, count, index, n, light, lut)` writes
the four bytes of each listed normal exactly as `sh2z_shade_light_normals` would. It
writes no other bytes. The SH-2 loop applies when both strides are 4 and `out` is 4-byte
aligned. It also needs the prepared light and reflection components to fit int16, which
is always true when the prepare basis is a rotation. The loop computes the channel
`d * c / (127 * 16384)` as `(d * K) >> 42` with `K = ceil(c * 2^28 / 127)`. This is exact
for every `0 <= d <= 127 * 16384` and every colour `c`, because the excess `d * K / 2^42 - d c / D`
is below `D / 2^42 < 1 / D`. `tests/test_vehicle.c` checks every `d` and every `c`. A
normal facing away stores the ambient bytes and the reflection byte with one long store.

## Approximate Gouraud packing

`sh2z_shade_gouraud_approx` takes the arguments of `sh2z_shade_gouraud` and computes, per
channel:

```
m   = (b*l + 128) >> 8
lit = (m*t + f*(255 - t) + 128) >> 8
v   = 16 + (lit >> 3) - (b >> 4)        (1 <= v <= 47), clamped to 31
```

The exact kernel divides by 255 with `(x + (x >> 8)) >> 8`; this one uses `>> 8`.

**Bound: every channel equals the exact channel or is one less.** For `0 <= x <= 65280`,
`x >> 8` is `round(x / 255)` or one less, so `m' in {m - 1, m}`. Then `lit'` is in
`[lit - 2, lit]`: one from `m`, since `t <= 255` adds at most `t / 256 < 1` before the
final shift, and one from the final shift. So `lit' >> 3` is in `{L - 1, L}`, and the
clamp is monotone. `tests/test_vehicle.c` proves this in two exhaustive stages: every
`x` for the shift, then every `(m, m', t, f)` for the second rounding (33.5 million cases).

**Match rate.** Over the whole domain (all 2^32 `(b, l, f, t)` channel tuples, counted
exactly by weights), 94.5443% of channels equal the exact kernel's; the rest are one less.
Without the `+ 128` in `m`, only 85.736% would. A test control shows that this is a different
rule. On the model's random inputs, 3,690,324 of 3,879,783 channels (95.12%) are equal.
The SH-2 loop needs a light stride of 4 with 4-byte-aligned light records and pairs, and
a base stride of at most 65535. It keeps `m - f` in the low half of one register, so each
channel costs two 16-bit multiplies.

## Evidence

**HOST** (`tests/test_vehicle.c` against the independent 64-bit oracle
`tests/vehicle_oracle.h` and shade-1's `tests/shade_oracle.h`) passes under GCC and Clang
Release with `SH2Z_STRICT=ON` and under ASan/UBSan.

| Kernel | What was checked | Negative controls |
|---|---|---|
| Gouraud | Bound proven exhaustively; 6,000 random trials plus a fog grid against the oracle and the bound; argument rules | 2 |
| Indexed light | 1,618,022 listed normals equal `light_normals` and the oracle; unlisted bytes untouched | 1 |
| Facing | 3,976,307 faces, in place and out of place, random vertices consistent with the clip | 5 |
| Indexed view | 295,190 listed vertices equal `sh2z_view_batch`; sentinels untouched | 1 |

The negative controls are:
- Gouraud: no `+ 128`; the exact rounding.
- Indexed light: writing every record.
- Facing: one triangle; no simple gate; no `all` reject; keep edge-on; limit 1024.
- Indexed view: writing every record.

Every control is caught.

**MODELLED** ([results](benchmarks/vehicle-model/results.json), captured by
[CI run 44215071056](https://github.com/lamb2k/SH2ZAM/actions/runs/44215071056) on source commit `c8674d4`, ELF SHA-256 `3f911d42...`; the
[vehicle-model](benchmarks/vehicle-model/README.md) harness). The compiled SH-2 code
matches the oracles before timing on 30,555 view vertices, 251,325 faces,
130,563 normals and 1,293,261 Gouraud pairs. The table gives cycles per element (the
slope between the 1-element and the largest row):

| Work | Kernel | Today | Whole call at caller sizes, kernel / today |
|---|---:|---:|---|
| facing, 97 faces | 53.0 | 228.7 (per-face C) | 5,333 / 22,228 |
| light, 70 of 133 normals | 72.7 | 101 (`light_normals`, all 133) | 5,461 / 13,431 |
| light, all 133 | 72.7 | 101 | 10,063 / 13,443 |
| Gouraud, 120 pairs | 91.4 | 134.2 (`sh2z_shade_gouraud`) | 11,266 / 16,246 |
| Gouraud, 448 pairs | 91.4 | 134.2 | 41,211 / 60,245 |
| view, 60 of 68 positions | 146.5 | 142.4 (`batch_divu`, all 68) | 9,418 / 9,681 |
| view, all 68 | 146.5 | 142.4 | 10,591 / 9,681 |

Cycle numbers are modelled (flat RAM, no caches or wait states), not Saturn timings.

**Not established:** console or emulator execution, cache and memory timing, and a whole
frame. The facing, view and Gouraud loops are bound by their memory gathers and
multiplies. For reference, the per-element costs that informed this work were 20 for
facing, 90 for the indexed view, 60 for indexed light and 50 for Gouraud. They are not
reached: the four random corner gathers alone take about 36 cycles per face; one
exact perspective division per vertex dominates the view; and the Gouraud kernel needs
two multiplies per channel.
