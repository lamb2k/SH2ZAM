# Integer and 2D kernels for lifted-arcade-style game code

Three game-neutral modules for code that carries 16-bit integer arithmetic and
screen-space bookkeeping to the Saturn: 68000-exact multiplies and divides, exact 2D
integer box overlap with packed bitmask results, and batch placement of 16x16 sprite
cells. They are plain integer kernels (no Q16.16, no floating point), reentrant,
allocation-free and independent of any renderer, title or data format.

**Evidence tiers.** *Host-exact* means checked against an independent oracle on a host
build. *Modelled* means executed SH-2 instructions in the Yabause interpreter with flat
RAM ([`benchmarks/integer-model`](benchmarks/integer-model/README.md)); it is not Saturn
timing. *Console* means measured on hardware. **Nothing here has a console
measurement; no console run was done.** Controls are plain C compiled with the same flags
and verified to honour the same contract.

| Module | Header | What it provides |
|---|---|---|
| 68000 arithmetic | [`sh2zam/m68k.h`](include/sh2zam/m68k.h) | `MULS.W`, `MULU.W`, `DIVS.W`, `DIVU.W` with the 68000's N/Z/V/C nibble, overflow rule and a divide-by-zero report |
| Box overlap | [`sh2zam/box2.h`](include/sh2zam/box2.h) | min/max and centre+half-size integer rectangles; inclusive or exclusive edges; one-vs-many and many-vs-many packed bitsets |
| Cell placement | [`sh2zam/cell16.h`](include/sh2zam/cell16.h) | mirror, offset, clip and compact a block of 16x16 cells |

## 68000 arithmetic

```c
uint32_t dn = ...;                         /* the full 32-bit data register */
uint32_t ccr = sh2z_m68k_divu_w(&dn, src); /* dn = remainder << 16 | quotient */
if (ccr & SH2Z_M68K_DIV_ZERO) { /* dn unchanged; raise your own trap */ }
else if (ccr & SH2Z_M68K_V)   { /* quotient > 16 bits; dn unchanged   */ }
```

- `sh2z_m68k_mulu_w` / `_muls_w`: `dn = (dn & 0xFFFF) * src`, unsigned or signed,
  returning N/Z (V = C = 0). On SH-2 these compile to `MULU.W` / `MULS.W` plus `STS MACL`;
  an ISR that uses MACL must preserve it per your SDK ABI, as for the rest of the library.
- `sh2z_m68k_divu_w` / `_divs_w`: 32-bit dividend, 16-bit divisor. Quotient truncates toward zero;
  the signed remainder takes the dividend's sign. Overflow (quotient outside 16 bits,
  signed range -32768..32767) leaves `dn` unchanged and sets V. Divide by zero leaves
  `dn` unchanged and returns only `SH2Z_M68K_DIV_ZERO`. The 68000 manual lists N and Z as
  undefined after an overflow; the library returns N = 1, Z = 0 there, identically on every
  path, and that choice is **not checked against silicon**.
- On SH-2 the divides run the manual's 16-step `DIV0U`/`DIV1` sequence plus one `MULU.W`
  to recover the remainder. No libgcc 64-bit divide. **The memory-mapped on-chip divider is
  deliberately not used:** its registers and interrupt rules are caller-owned, and the project
  avoids implicit divider access ([HARDWARE.md](HARDWARE.md)). The divider's real cost is
  not modelled by the instruction model, so no claim is made about it. The sequence
  clobbers `SR.T/Q/M` inside one asm block (T is declared; DIV0U resets Q/M on entry) and leaves `SR.S`
  alone.
- `sh2z_m68k_ref_*` are straightforward portable C and serve as the specification.

**Host-exact.** [`test_m68k.c`](tests/test_m68k.c) checks the fast and reference versions against a
third, independent bit-serial long-division model. It also checks an instruction-level emulation of
`DIV0U`/`DIV1`/`ROTCL` running the same 16-step sequence, every divisor against its quotient
and remainder boundaries (both dividend signs), edge dividends, and random in-range and
out-of-range cases (3M by default). `SH2Z_M68K_EXHAUSTIVE=1` runs all 2^32 16x16 multiplies
(signed and unsigned) and 200 million random divides; that run passed 44,618,231,011 checks.
**Executed SH-2 code.** The harness runs the actual compiled `DIV1` object code against the same
specification: see the verified counts below.

**Modelled cycles per call** (mean over 4,096 draws; plain C control with the same contract):

| Operation | Case | Library | Plain C | Fewer cycles |
|---|---|---:|---:|---:|
| `mulu_w`, `muls_w` | any | 12 | 12 | 0% (the compiler already emits `MULU.W`/`MULS.W`; the API adds exact flags and register semantics, not speed) |
| `divu_w` | quotient fits | 48 | 170 | 71.8% |
| `divu_w` | overflow | 16 | 88 | 81.8% |
| `divs_w` | quotient fits | 73.1 | 200 | 63.5% |
| `divs_w` | overflow | 58.0 | 105 | 44.8% |

## Box overlap

Rectangles are `int32`: `sh2z_ibox2_t {min_x,min_y,max_x,max_y}` or `sh2z_ibox2c_t {cx,cy,hx,hy}`.
`SH2Z_BOX_INCLUSIVE` treats rectangles as closed (touching edges overlap); `SH2Z_BOX_EXCLUSIVE`
requires interiors to intersect. The formulas are in the header. They are exact over the full
`int32` range for valid boxes (min <= max, half-sizes >= 0); the centre form compares an unsigned
absolute distance with the unsigned half-size sum, so nothing overflows.

- `sh2z_ibox2_overlap_one_many`, `sh2z_ibox2c_overlap_one_many`: one query against `count` boxes into a
  bitset (bit `i & 31` of word `i >> 5`, unused bits zero, extra capacity untouched) with an optional hit count.
- `sh2z_ibox2_overlap_many_many`, `sh2z_ibox2c_overlap_many_many`: one bitset row per `a` box
  (`sh2z_box_bitset_words(b_count)` words per row) with an optional pair count.
- `sh2z_ibox2_overlap_one_many_unchecked` (and `c`): trusted-buffer variant. Argument validation costs
  about as much as testing a dozen boxes on SH-2, so tiny batches that are already known to be valid use this.
- Checked functions validate counts (<= 2^31), alignment, representable lengths, capacity and overlap of the
  output with every input before any write; failure returns false and changes nothing.

**Host-exact.** [`test_box2.c`](tests/test_box2.c): int64 oracles for both forms and edge modes over tiny
(many touching edges), screen-sized, 16-bit and full-`int32` coordinate ranges, counts around every word
boundary, an interval-based second oracle for the centre form, scalar-vs-batch and unchecked-vs-checked
identity, and the argument and buffer contracts.

**Modelled** (selected rows; whole call including validation; mean of 128 corpora; "sparse" is boxes up to 71 px (query 32 px)
scattered over a 384x224 arena with about 7% hits, "dense" has every box overlapping the query; full table in
[results.json](benchmarks/integer-model/results.json)):

| Case (inclusive edges) | Library | Plain C | Fewer cycles |
|---|---:|---:|---:|
| min/max, 1 vs 16, sparse | 423 | 317 | -33% (slower: validation dominates) |
| min/max, 1 vs 16, sparse, unchecked | 265 | 317 | 16% |
| min/max, 1 vs 64, sparse | 993 | 1,130 | 12% |
| min/max, 1 vs 256, sparse | 3,267 | 4,384 | 26% |
| min/max, 1 vs 256, dense | 5,789 | 12,181 | 52% |
| centre, 1 vs 64, sparse | 1,449 | 2,424 | 40% |
| centre, 1 vs 256, sparse | 5,077 | 9,585 | 47% |
| min/max, 16 x 16, sparse | 4,406 | 4,397 | 0% |
| min/max, 32 x 64, sparse | 26,567 | 32,880 | 19% |
| centre, 32 x 64, sparse | 41,270 | 77,170 | 46% |

Honest reading: the min/max form is only a modest win when most boxes miss at the first test, and a checked
call on a handful of boxes does not beat a bare loop. Centre form and hit-heavy batches gain the most. Edge
mode (exclusive) costs within 1% of inclusive.

## Cell placement

`sh2z_cell16_place` takes cell offsets inside a block (`sh2z_screen_t`), a `sh2z_cell16_xform_t` (origin,
block size in pixels, `SH2Z_CELL16_FLIP_X|Y`) and a half-open `sh2z_clip_rect_t`. Visible 16x16 cells are written in input
order as screen positions plus (optionally) their input indices. Flipping mirrors the block (`x = block_w - 16 - x`); pixel
contents are the renderer's concern. Exact int32 arithmetic; origin and block size are limited to +/-2^20 and
clip minima to >= -32752 so every written position fits `int16`. The clip test runs in offset space (one unsigned
compare per axis), so mirroring costs nothing before the test.

**Host-exact.** [`test_cell16.c`](tests/test_cell16.c): a per-cell branching model over tile-grid, wide
and full-range offsets, random clips including empty and extreme ones, all flip combinations, output-extent
checks and the argument and overlap contracts.
**Modelled** (mean of 128 corpora, no flip and X+Y flip are within 3% of each other except where noted):

| Case | Visible cells | Library | Plain C | Fewer cycles |
|---|---:|---:|---:|---:|
| 16 cells, on screen | 16 | 799 | 718 | -11% |
| 64 cells, on screen | 64 | 2,383 | 2,734 | 13% |
| 256 cells, on screen | 185 | 7,589 | 10,162 | 25% |
| 256 cells, edge | 71 | 5,196 | 7,560 | 31% |
| 256 cells, scattered | 37 | 4,442 | 7,188 | 38% |
| 256 cells, scattered, flipped X+Y | 10 | 3,697 | 6,659 | 45% |

Batches below roughly 40 cells do not beat an unchecked plain loop because the library validates its arguments.

## Verification on the executed object code

The harness first verifies the SH-2 object code against independent models: **16,000,308 `m68k` calls**
(plus the plain controls on every eighth), **202,000 box** and **202,000 cell16** randomized cases
(`INTEGER_VERIFY=4000000`), all exact. See [benchmarks/integer-model](benchmarks/integer-model/README.md).

## Not done

No console run. No whole-frame measurement. The 68000 overflow N/Z choice is unchecked against silicon. The
memory-mapped divider was not evaluated. The numbers above are from a local GCC 9.3 build; CI produces its own
capture with the Yaul GCC.
