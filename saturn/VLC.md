# VLC: variable-length symbol decoding for byte alphabets

Game-neutral kernels for decoding prefix-coded byte symbols from a bit stream on the SH-2. Candidate for SH2ZAM; written in lane `fastcodec` of a Saturn port (the port's data never enters this folder: every test here uses generated, synthetic alphabets).

## What is in it

| File | What |
| --- | --- |
| `include/sh2zam/vlc.h` | API, stream and table format (documented in the header comment) |
| `src/sh2zam_vlc.c` | marker-bit bit reader, length-limited canonical code builder, portable decoder (C reference) |
| `src/sh2zam_vlc_sh2.S` | SH-2 assembly decoder, same bytes as the C reference |
| `tests/test_vlc.c` | host tests: 758 round trips over 7 alphabet shapes, every first level window 1..8 and every `lmax` up to 16, builder invariants (complete, prefix free), chunked decoding, a negative control, argument contracts |

Build: `-DSH2Z_VLC_ASM=ON` adds the assembly kernel to the library for an SH-2 toolchain; the C parts and the test build everywhere (`ctest -R vlc_exact`, also with `SH2Z_STRICT=ON`, ASan and UBSan).

## The ideas

- **Marker-bit buffer.** The unread bits sit in one 32-bit register with a marker bit above the last valid bit (`bb = data | 1 << nb`). `bb >= 0x10000` means 16 or more valid bits, so no bit count is kept and the refill test is a single compare. A refill appends one 16-bit unit.
- **No variable shift.** The SH-2 shifts by 1, 2, 8 or 16 only. `bb >> len` is the high word of `dmulu.l bb, 2^(32-len)`; the factor comes from a 16-entry table indexed by the length byte.
- **Two byte arrays, not one table of structs.** The first level is `s[2^w]` (symbol) and `n[2^w]` (`4 * (len - 1)`, the table offset of the shift factor). Each is one `mov.b`; `n` goes straight into the multiplier lookup. A code longer than `w` bits has `n >= 0x80` and sends the decoder to a small second level pool (`xb` more bits index a group of `u16` entries). One compare and one branch separate the paths.
- **Small first level.** `w` = 5 or 6 keeps the table in 64 to 128 bytes per array, so it stays in the 4 KB cache beside the code; the long codes are rare by construction.
- **Builder** limits Huffman lengths to `lmax` by rebalancing the Kraft sum (clamp, lengthen the rarest short code until it fits, then spend leftover room on the most frequent symbol). Valid and prefix free (checked), close to optimal, not guaranteed optimal.

## Numbers (MODELLED, not console measurements)

Yabause `sh2int.c` per-opcode cycles, plus a 4 KB 4-way unified cache (line fill: high RAM 7, cart 80 clocks) and load-use and multiply interlocks. 20,000 symbols decoded from a generated alphabet (`SH2Z_VLC_DUMP`, `SH2Z_VLC_DUMP_CASE=kind,w,lmax`), table in the cart or in high RAM, stream in the cart, cold cache. Cycles per symbol, `tab` = where the table lives:

| Alphabet (kind, w, lmax) | stream B / 20,000 | asm CPU only | asm cache, table cart | asm cache, table high | C ref CPU only (GCC 9.3 -O2) |
| --- | --- | --- | --- | --- | --- |
| zipf, 256 symbols (1, 6, 12) | 15,652 | 44.8 (37 instr) | 55.8 | 55.6 | 191.6 |
| near uniform, 256 symbols (0, 8, 8) | 20,000 | 33.0 (27 instr) | 42.6 | 42.5 | 156.5 |
| long tail, 30 + 226 rare (3, 6, 12) | 8,204 | 26.3 (22 instr) | 32.5 | 32.4 | 115.1 |
| few, wide range (2, 5, 10) | 9,800 | 28.0 (23 instr) | 34.7 | 34.7 | 133.3 |

All runs decode every symbol bit for bit; a flipped stream bit changes the output (negative control). The C reference is written for clarity (a bit-by-bit refill), so the 4x gap is the cost of not having the marker-bit and multiply tricks, not a tuned C baseline.

Where the cycles go (zipf): about 14 instructions for a first level symbol, plus 18 for a refill (one every 2 to 3 symbols), plus 30 for a second level symbol. A 32-bit refill (two units at once) is the next step; it was not needed for the port this came from, where a context-selected table per symbol dominates.

## Not included

No rANS or tANS kernel and no two-symbol table: they were not built. A table-driven tANS step needs a table of 2^R entries that competes with the 4 KB cache, and the SH-2 cannot hide a dependent table load behind the multiply, so they were judged on the size side only and not carried further. No SCU DSP kernel: it has no indexed table lookup, a 256-word program RAM and 1 KB of data RAM.

## Hardware assumptions

Big-endian SH-2 with `dmulu.l`, `mov.w` on 2-byte aligned stream addresses. The stream is little-endian bit order (low byte of each unit first) on any host. The caller pads the stream by 4 bytes; the reader may read 4 bytes past the bits it needs. No hidden DMA, cache flush or global state.
