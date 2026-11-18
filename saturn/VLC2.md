# Per-symbol variable-length decoding (experimental, contract tag `vlc-2`)

[`vlc2_experimental.h`](include/sh2zam/vlc2_experimental.h) decodes prefix-coded byte
symbols one at a time, with the table chosen per symbol by the caller. It is meant for
context-modelled decoders, such as a tile codec whose next table depends on pixels already
decoded. There, vlc-1's `sh2z_vlc8_decode` (one call per run of symbols from one table) pays
its call and reader set-up on almost every symbol. vlc-2 is an inline decode step: on SH-2
it is hand-written assembly, and the bit reader stays in the caller's registers. Tables are
compact byte arrays packed back to back in one arena. The reader starts at any byte offset of
the caller's stream. Bounds are checked once per stream, and the caller writes the output.
vlc-1 ([VLC.md](VLC.md)) is unchanged. The request came from a contributing Saturn port's tile
decoder; every test here uses synthetic, generated data.

**Evidence tiers: HOST and MODELLED only.** No emulator or console run of these kernels has
been done. Cycle numbers come from the Yabause SH-2 interpreter with flat RAM; placed numbers
add the [placement model](benchmarks/placement-model/README.md)'s cache and region costs,
which come from hardware notes. None of them are Saturn timings.

## API

| Function | What it does |
| --- | --- |
| `sh2z_vlc2_begin(r, stream, offset)` | Start a reader at `stream[offset]` (any offset; nothing is copied). Inline. |
| `sh2z_vlc2_decode(r, t, w)` | Decode one symbol (0..255) with table `t` of window `w` (a constant). Inline; SH-2 assembly on SH-2. |
| `sh2z_vlc2_decode_portable(r, t, w)` | The C definition: same symbols and reader state on every target. |
| `sh2z_vlc2_reach(offset, bits)` | One past the last stream byte read once `bits` bits are consumed: the once-per-stream bound. |
| `sh2z_vlc2_bitpos(r, stream)` | Bits consumed since `stream[0]`, for a truncation test after the stream. |
| `sh2z_vlc2_build(t, capacity, len, nsym, w, code_out)` | Table bytes for canonical code lengths; returns its size (0 = refused). |
| `sh2z_vlc2_table_size(len, nsym, w)` | The size `sh2z_vlc2_build` would return. |
| `sh2z_vlc2_check(t, size, w)` | Is a loaded table safe? Returns its longest code length, or 0. |
| `sh2z_vlc2_consts` | The 256-byte constant block the decoder reads. |

The header is the contract. In short:

- **Stream.** The same bits as vlc-1: bytes consumed least significant bit first, each code
  written first bit first (the canonical code bit-reversed, as in DEFLATE). vlc-1's 16-bit
  little-endian units are the same stream, so an existing encoder is unchanged.
- **Codes.** Lengths 1..15 forming a complete prefix code, or one symbol of length 1. Codes
  are canonical in (length, symbol) order, like vlc-1. Lengths can come from an encoder or
  from vlc-1's `sh2z_vlc8_build` (`len_out`), which applies a length limit.
- **Tables.** For window `w` (1..7): `t[e]` is the symbol and `t[2^w + e]` the field for the
  next `w` bits `e`. The field is `4 × len`, or `64 + 4 × xb` for an escape to a group of
  `2^xb` two-byte entries (symbol, `4 × len`) at byte `4 × t[e]` of the table. Groups follow
  the first level, largest first. A table has no alignment requirement and is
  `2^(w+1) + 2 × (pool entries)` bytes: 128 bytes at `w` = 6 when no code is longer than 6
  bits. `sh2z_vlc2_build` refuses a table whose last group would start past byte 1020; at
  `w` = 6 that leaves about 450 pool entries.
- **Reader.** `bits` holds the unread bits and `cnt` is `0x7FFFFFC0 + 4 × count`. Between
  symbols 16..31 bits are buffered; one aligned 16-bit unit is appended after a symbol that
  leaves fewer than 16. Keep the reader in a local variable.
- **Bounds, once per stream.** A reader begun at `offset` has read exactly the bytes
  `[offset, sh2z_vlc2_reach(offset, B))` after consuming `B` bits. A symbol consumes at
  most `L` bits, the longest code of the tables used, which `sh2z_vlc2_check` returns. So if
  `sh2z_vlc2_reach(offset, S × L)` is within the buffer, decoding at most `S` symbols reads
  nothing past it, whatever the stream holds: there is no test per symbol. A well-formed
  stream reads at most 2 bytes past its last byte. For the last block of a buffer that has
  no such slack, the caller pads the buffer or copies that block into a padded buffer.
  `sh2z_vlc2_bitpos` tells after the stream whether it was truncated.
- **Safety.** A table that passes `sh2z_vlc2_check` decodes without reading outside itself,
  whatever the stream. Every built table passes. A table loaded from untrusted data must be
  checked; an unchecked table can make the SH-2 path loop or fault.
- **SH-2 path.** It is used on SH-2 builds without `SH2Z_FORCE_PORTABLE`. `w` must be a
  constant expression (it is an immediate operand), so compile with optimisation. The step
  needs `r0` and two scratch registers besides the reader's three and the table pointer, and
  clobbers MACH, MACL and T. It uses no stack, calls nothing, and is a `"memory"` barrier.

## How the SH-2 step works

```
mov     bits,r0          ; e = bits & (2^w - 1)
and     #63,r0
mov.b   @(r0,t),s        ; symbol
or      #64,r0
mov.b   @(r0,t),r0       ; field n = 4 * len
mov.l   @(r0,k),f        ; 2^(32 - len)
dmulu.l f,bits
sts     mach,bits        ; bits >>= len (the SH-2 has no variable shift)
subv    r0,cnt           ; T = fewer than 16 bits left
bf/s    done
extu.b  s,s              ; delay slot, both paths
```

- **12 instructions, 13 modelled cycles** for a code of at most `w` bits.
- **The count doubles as the refill test.** `cnt` is biased so that "fewer than 16 bits
  left" is exactly a signed underflow of `SUBV`. The refill test costs no instruction beyond
  the count update, and no constant register.
- **Refill (+11 cycles, once per 16 bits).** After the underflow, the low byte of `cnt` is
  `0xC0 + 4 × count`, the offset of `2^count` in the constant block. The refill is `MOV.W`,
  `SWAP.B` (stream order), `MULU.W` by that power, `OR`, then add 64 to `cnt`.
- **Escape (about +29 cycles).** An escape field selects a constant of 1, so `DMULU.L`
  leaves the bits in MACL. It also always underflows, so it shares the refill's branch; bit 6
  of the field tells the two apart. The escape path restores the bits and the count, indexes
  the group with the next `xb` bits and redoes the step with the group entry.
- **Out of line without a call.** The slow paths sit inside the same asm statement, after
  the `bf/s`, so the fast path's only branch is one 2-cycle taken branch with a useful delay
  slot. A slow path in another section or subsection was rejected: once the caller's function
  is larger than the 256-byte reach of a conditional branch, the assembler turns `bt` into
  `bf` + `bra`, which costs the fast path 2 cycles.

### Reader variant: a count, not vlc-1's marker bit

vlc-1 marks the end of the valid bits with a marker bit, so the refill test is
`bits >= 0x10000`. That variant is measured here too (`marker` below, defined in the model
bridge only, with the same tables and fast path). Its fast path is the same 12 instructions
(`CMP/HS` against a constant register instead of `SUBV`). But its refill must find the
marker first: a 256-byte log table on the high or low byte, about 10 more instructions. On the
synthetic tiles it costs 28.2 cycles per symbol against vlc-2's 24.0 (+9% per tile). It also
needs one more register (0x10000) and 256 more bytes of hot table. vlc-2 therefore keeps a
count. The bits on the stream are the same either way.

## How a caller writes its decode loop

```c
#include <sh2zam/vlc2_experimental.h>

/* Tables: build each from its code lengths into one arena, back to back. */
size_t at = 0;
for (unsigned k = 0; k < ntables; k++) {
    const size_t n = sh2z_vlc2_build(arena + at, sizeof arena - at, lengths[k], 256, 6, NULL);
    if (!n) return false;                       /* refused: see the header */
    table_at[k] = (uint16_t)at;                 /* the caller's context map points here */
    at += n;
}

/* One block (say a tile) whose next table depends on what is already decoded.
 * `stream` is 2-byte aligned; `offset` is the block's byte offset in it. */
if (sh2z_vlc2_reach(offset, MAX_SYMBOLS * 12) > stream_bytes)
    return false;                               /* once per block: pad, or copy the tail */
sh2z_vlc2_reader_t r;                           /* a local: its fields stay in registers */
sh2z_vlc2_begin(&r, stream, offset);
unsigned ctx = 0;
for (unsigned i = 0; i < count; i++) {
    const uint8_t *t = arena + context_map[ctx];   /* any table, chosen per symbol */
    const uint32_t s = sh2z_vlc2_decode(&r, t, 6); /* 13 cycles for a short code */
    out[i] = (uint8_t)s;                           /* the caller writes the output */
    ctx = next_context(ctx, s, out, i);            /* from output already decoded */
}
if (sh2z_vlc2_bitpos(&r, stream) > 8 * block_end) /* optional truncation test */
    return false;
```

Several call sites in one loop are fine. Each inlines its own copy of the step, about
90 bytes, and the hot part is its first 24. A call site with another window (say `w` = 7 for
a large dictionary alphabet) uses that window's tables. In the model bridge, two caller
tricks save about 3 cycles per context lookup. They are caller code, not library API. The
class table and the context map are stored rotated by 128 entries and read through a pointer
to their middle, so the SH-2's sign-extending byte and word loads index them with no `EXTU`.

## The synthetic workload

The model decodes a generic, context-modelled 4 bpp tile format, generated by the harness
(no game data):

- A tile is 16x16 pixels at 4 bits: 16 rows of 8 bytes, each two half-rows of 4 bytes.
- Each row has one **row symbol** (16 tables, chosen by the previous row symbol). It gives
  each half-row a kind: zero, the half-row above, a **dictionary** entry (one symbol from a
  256-entry table, `w` = 7, giving 4 bytes) or raw.
- A raw half-row is four **pair symbols**, one byte (two pixels) each. Each pair symbol's
  table is one of 32, chosen per symbol through a 256-entry `u16` map. The map is indexed by
  a context: the class (16 classes) of the byte above, and the nibble to the left.
- All codes are canonical with a length limit of 12, and the pair and row tables use
  `w` = 6. Pair tables are Zipf-like over 64 to 256 byte values. Escapes (codes longer than
  `w`) are about 20% of symbols, which matches the bit budget the contributor describes
  (300 to 450 bits per tile).
- The 50 tables take 17.5 KB in one arena. The hot part (first levels, maps, class table and
  dictionary) is 8.2 KB.

There are two mixes. **shape** follows the contributor's per-part counts: 16 row, 3.8
dictionary and 48.7 pair symbols, so 68.5 symbols and 305 bits per tile, with 14.0 escapes.
**dense** matches its total of about 105 symbols: 16 row, 4.2 dictionary and 84.3 pair, so
104.5 symbols and 458 bits per tile, with 23.1 escapes. The contributor's description gives
both "about 105 symbols" and parts that add up to about 70, so both are reported.

## Evidence

### HOST (`saturn/tests/test_vlc2.c`, `ctest -R vlc2_exact`)

The test runs under GCC and Clang Release with `SH2Z_STRICT=ON`, and under ASan/UBSan. The
independent oracle [`tests/vlc2_oracle.h`](tests/vlc2_oracle.h) shares no code with the
library. It walks lengths and symbols for canonical codes and searches for the code that
matches each table index. It orders groups by an explicit sort, decodes one bit at a time,
derives the reader state by simulating the refill rule, and restates the table check.

- **Every build for small alphabets.** Every length vector of 1 to 6 symbols with lengths
  0..7, at every `w` 1..7: 2,097,144 builds, 10,388 accepted. Each matches the oracle's
  bytes, size, codes and refusal, and `sh2z_vlc2_check` returns the longest length. Exact
  capacity is accepted; one byte less is refused without writing.
- **Random alphabets.** 4,000 builds (3,509 accepted) of 1 to 256 symbols with limits
  1..15, plus every-length (1..15) codes and deep escapes at every `w`.
- **Round trips.** 9,853 messages and 943,800 symbols. Each goes into a heap buffer of
  exactly `sh2z_vlc2_reach` bytes (so ASan sees any read past the bound), at offsets 0..8 of
  an even and an odd base. Symbols, reader state and bit position match the oracle.
- **Under the once-per-stream bound.** 1,500 random-byte and 1,500 truncated streams of up
  to 200 symbols, each in a buffer of exactly `sh2z_vlc2_reach(offset, S × L)` bytes,
  decoded against the oracle's table walker.
- **Hostile tables.** 40,000 garbage tables through `sh2z_vlc2_check`, matching the
  oracle's restated rule. The 5,202 accepted ones decode random streams in bounds.
- **Negative controls (8, all caught):** most-significant-first bits; canonical order
  (length, −symbol); groups smallest first; refill at 16 bits left instead of 15; the bound
  one unit short; the group index one bit late; an odd start that does not align; a
  one-symbol code that consumes no bit.

### MODELLED ([`benchmarks/vlc2-model`](benchmarks/vlc2-model/README.md))

The compiled big-endian SH-2 objects run in the pinned Yabause interpreter. All checks
below run against the oracle before any timing, and a read past a bound stops the run:

- **The SH-2 builder:** 6,960 builds (5,297 refused), with tables, codes and check results
  compared.
- **Decoders, every `w`, inline assembly and SH-2-compiled C:** 7,011 streams (3,536 of them
  at odd starts) and 2,772,366 symbols per decoder. Messages, messages decoded past their
  end, and 1,327 random-byte streams under the bound.
- **Format tables:** 6,000 random tables through the check, of which the 2,535 accepted
  ones decode.
- **Table switching:** 376 streams whose table changes every symbol.
- **Tiles:** 1,536 tile decodes, every variant against the generator's tiles. Any read of
  stream bytes at or past `sh2z_vlc2_reach`, or of a table past its size, stops the run.

**Tile cost, cycles per tile** (mean of 128 tiles; placed figures are warm, with the cache
carried over from the previous tile):

| Mix | Decoder | CPU only | Decoder share (per symbol) | All high WRAM | Code/tables high, stream/output low | Tables in cart (6) | All in cart (10) |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| shape (68.5 symbols) | **vlc-2** | **3,081** | 1,643 (24.0) | **3,972** | 4,233 | **8,825** | 12,855 |
| | vlc-2, byte-packed tiles | 3,078 | 1,640 (24.0) | 3,962 | 4,218 | 8,793 | 12,790 |
| | vlc-2 portable C | 4,732 | 3,294 (48.1) | 5,712 | 5,977 | 10,750 | 15,443 |
| | marker-bit variant | 3,365 | 1,928 (28.2) | 4,366 | 4,622 | 9,729 | 14,397 |
| | vlc-1, one call per symbol | 9,478 | 8,040 (117.4) | 12,866 | 13,178 | 18,988 | 24,281 |
| | caller only (no decoding) | 1,438 | | 1,459 | 1,466 | 1,501 | 1,541 |
| dense (104.5 symbols) | **vlc-2** | **4,356** | 2,503 (24.0) | **5,723** | 6,125 | **13,229** | 19,509 |
| | vlc-2 portable C | 6,836 | 4,984 (47.7) | 8,332 | 8,730 | 16,121 | 23,365 |
| | marker-bit variant | 4,789 | 2,937 (28.1) | 6,346 | 6,741 | 14,678 | 22,095 |
| | vlc-1, one call per symbol | 14,234 | 12,382 (118.5) | 19,371 | 19,827 | 28,564 | 36,560 |
| | caller only (no decoding) | 1,853 | | 1,874 | 1,881 | 1,917 | 1,958 |

The column placements are the placement model's `best`, `typical`, `cartTables` (stream and
output in low WRAM) and `worst`. The decoder share is a variant's cycles minus the caller
only. The contributor quotes its own decoder at 3,396 cycles per tile CPU only, about 4,960
with its tables in high WRAM and about 9,590 with them in the cartridge. The vlc-2 tiles are
3.1 (shape) and 3.3 (dense) times faster than one vlc-1 call per symbol.

**One table, cycles per symbol** (slope from 1 to 4,096 symbols, 8 corpora, each symbol
stored to memory):

| Alphabet | bits/symbol | escapes | vlc-2 | vlc-2 C | marker variant | vlc-1 batch | vlc-1 per call |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 16 symbols, geometric | 2.00 | 1.5% | 19.82 | 38.01 | 21.18 | 21.66 | 104.66 |
| 256 symbols, Zipf 1.3 | 4.84 | 27.1% | 29.19 | 50.17 | 32.22 | 33.18 | 116.18 |

Per symbol, the loop itself (store, count, branch) is about 5 cycles. From the two rows, an
escape costs about 29 cycles more than a first-level symbol. vlc-2 decoding one symbol at a
time is faster than vlc-1 decoding a whole run from one table.

### Against the targets

| Target | Result |
| --- | --- |
| Per-symbol decode about 20 to 25 cycles | **Met**: 13 cycles for a first-level code, plus 11 per refill. On the tiles the decoder averages 24.0 cycles per symbol, including about 20% escapes. |
| Reader in registers, table pointer per symbol | **Met**: an inline step, with no stack and no call. |
| Two byte arrays, escapes out of line, compact tables packed in one arena | **Met**: a table is 128 bytes at `w` = 6 plus its pool, with no alignment. |
| Start at a byte offset, 16-bit refills, no copy | **Met**: odd and even starts; byte-packed tiles cost the same. |
| Bounds once per stream; output with the caller | **Met**: `sh2z_vlc2_reach` and `sh2z_vlc2_bitpos`. |
| 3,396 cycles per tile CPU only | **Met for shape (3,081); missed for dense (4,356).** |
| About 4,960 with the tables in high WRAM | **Met for shape (3,972, or 4,233 with the stream and output in low WRAM); missed for dense (5,723).** |
| About 9,590 with the tables in the cartridge | **Met for shape (8,825); missed for dense (13,229).** |

The dense mix misses because it decodes 53% more symbols for the same budget. The decoder's
share stays 24.0 cycles per symbol; the synthetic caller's own code is 1,438 to 1,853 cycles
per tile. Whether the contributor's tile is closer to shape or to dense, and how its own
caller code compares, needs a measurement of its decoder with this step. The cartridge
figures stay high for any decoder whose hot tables (8.2 KB here) exceed the 4 KB cache.

The numbers above are from the committed capture
[`benchmarks/vlc2-model/results.json`](benchmarks/vlc2-model/results.json), from
[CI run 44247175665](https://github.com/lamb2k/SH2ZAM/actions/runs/44247175665), source `769d1a4`,
pinned Yaul GCC 13.2, ELF SHA-256 `d04deb11...`; apart from its provenance it equals the capture of
[CI run 44243332134](https://github.com/lamb2k/SH2ZAM/actions/runs/44243332134) on `254b725`. A local run with the same toolchain gave identical
values. Every CI run uploads a fresh capture there.

## Not included

There is no batch API: vlc-1 remains the batch decoder. The library has no context
modelling or tile format; the tile codec above is the model's synthetic workload. There is
no table builder from frequencies: use an encoder's lengths or vlc-1's `sh2z_vlc8_build`. No
32-bit refill, and no interleaved symbol/length layout. An interleaved layout would touch one
cache line per lookup instead of two, at the cost of an index instruction; it was not asked
for and was not measured.
