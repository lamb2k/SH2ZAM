# Delta coded sprite records (experimental, contract tag `cell-records-1`)

[`cell_records_experimental.h`](include/sh2zam/cell_records_experimental.h) expands a list of delta coded multi tile
sprite records into cell list entries. A sprite is a short chain: each record carries a tile and attribute word and one
pair of offsets that moves the running position, so the list is read once, in order, and each cell costs a few
instructions. It sits next to [`cell16.h`](include/sh2zam/cell16.h): `sh2z_cell16_place` places a block of identical
16x16 cells from absolute offsets and clips them; this kernel builds the entries themselves (a position word and an
attribute word) from records whose cells have their own sizes, palette rules and flips, and does not clip.

Nothing selects it by default. It is integer-only, reentrant, allocation-free and has no libc, divide or multiply.

## Status

- Contract tag `cell-records-1`. The per-cell definition in the header is frozen at the tag. A faster path may replace
  the implementation only if it produces identical entries and positions.
- **Exact.** Every entry and the running position after a call equal an independent per-cell oracle for all 12 forms,
  on every host and on SH-2 object code.
- **Evidence tiers:** HOST and MODELLED only (see [the validation record](validation/cell-records-1.md)). No console run
  was done.

## Contract in one page

A call turns `count` records into `count` entries of 8 bytes, a position word and an attribute word. Per cell, in order
(position arithmetic wraps modulo 2^16):

| Step | Rule |
| --- | --- |
| Attribute | `KEEP`: as read. `ATTR_REPLACE`: `(record & attr_keep) \| attr_or`. `ATTR_ADD_BYTE`: the low byte plus `attr_or`, no carry out of the byte. With `MIRROR_X`, then `^ 0x20`. |
| Position | No mirror: `x += dx`, `y += dy`; the entry's x is `x`. `MIRROR_X`: `x -= dx`, `y += dy`; the entry's x is `x - w - mirror_bias`, with `w = (attribute >> 4) & 0xf0` the cell's own width in pixels (attribute bits 8 to 11, in units of 16). |
| Entry | position word `((x << 16 \| y) & pos_mask) \| pos_or`, then the attribute. |

Offsets are pairs of signed bytes, or signed words with `SH2Z_CELLREC_S16`. All streams are big-endian bytes (the
layout the SH-2 and the 68000 data share); loads are byte-order neutral, so a little-endian host gives the same entries.
Records and offsets are 2-byte aligned, entries 4-byte aligned. The running position (`sh2z_cellrec_pos_t`) is read
before the first cell and written after the last, so a long list can be expanded in several calls.

`sh2z_cellrec_expand` checks its arguments and changes nothing on a refusal; `sh2z_cellrec_expand_trusted` checks
nothing and is the entry point for callers that validate one parameter block for many calls. Batches of 4 to 40 cells
are the design point.

### Folding a masked, flagged running position

A caller whose own loop keeps the running position masked and flagged between cells (the position register is
`(position & mask) | flags` after every cell) gets the same entries from this kernel when, per 16-bit half, the mask
is `2^k - 1` and the flags have no bit under the mask: the bits above the mask never feed back into the bits below it.
The caller checks that once per call.

## Not provided

Vertical mirroring and clipping. Callers that need them keep their own loop for those cells.

## Modelled cost

SH-2 instruction model ([instruction model](benchmarks/cell-records-model/README.md), Yabause interpreter, flat RAM, no
caches or wait states; **MODELLED, not Saturn timing**). Cycles for one list of 40 cells, compiled C against an independent
straightforward C control (one cell at a time, every decision taken per cell, bytes read one at a time):

| Form | Cycles a cell, library | Cycles a cell, control | Control / library | List of 40, trusted | List of 40, checked | List of 40, control |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| byte offsets, no mirror, kept | 20 | 48 | 2.4 | 865 | 1020 | 1983 |
| byte offsets, no mirror, replaced | 22 | 47 | 2.1 | 948 | 1103 | 1943 |
| byte offsets, no mirror, byte added | 25 | 55 | 2.2 | 1068 | 1223 | 2263 |
| byte offsets, mirrored, kept | 29 | 56 | 1.9 | 1225 | 1380 | 2303 |
| byte offsets, mirrored, byte added | 35 | 63 | 1.8 | 1466 | 1621 | 2583 |
| word offsets, no mirror, kept | 18 | 61 | 3.4 | 787 | 942 | 2503 |
| word offsets, no mirror, byte added | 23 | 68 | 3.0 | 987 | 1142 | 2783 |
| word offsets, mirrored, byte added | 32 | 74 | 2.3 | 1349 | 1504 | 3023 |

Lists of 4 cells (whole call, cycles):

| Form | Trusted | Checked | Control |
| --- | ---: | ---: | ---: |
| byte offsets, no mirror, kept | 145 | 300 | 255 |
| byte offsets, mirrored, kept | 181 | 336 | 287 |
| byte offsets, no mirror, byte added | 168 | 323 | 283 |
| byte offsets, mirrored, byte added | 206 | 361 | 315 |

The checked entry costs about 155 cycles more a call than the trusted one (argument and overlap checks); for lists
of 4 to 40 cells the trusted entry is the one to call after validating the parameter block once.
