# IMA ADPCM decoding (contract tag `adpcm-1`)

[`adpcm.h`](include/sh2zam/adpcm.h) decodes 4-bit IMA/DVI ADPCM on the SH-2. It is exact:
for every input it writes the same samples and leaves the same state as the common
bitwise IMA rule, truncations included. A caller can swap it in for an existing
decoder with no audio change.

```c
sh2z_ima_state_t st = {0, 0};              /* pred, step index */
int16_t pcm[64];
sh2z_ima_decode(&st, bank, pos, pcm, 64);  /* nibbles pos .. pos+63 */
pos += 64;                                  /* continue later from st */
```

- Nibble `p` is the low nibble of `src[p >> 1]` when `p` is even, the high nibble when odd.
- Any split works: decoding a stream in pieces gives the same output as one call.
- No allocation, no global mutable state, no floating point, no libgcc or `memcpy`
  calls, no hardware divider. Reentrant.
- `sh2z_ima_ref_decode` is the specification. It lives in `sh2zam_adpcm_ref.c` with the
  step table, so a program that only decodes links neither.

## How it is faster

The common decoder, per nibble, loads the step, tests three bits, shifts twice, applies
the sign, clamps both ways, then adjusts and clamps the index. Usually it is also a
function called once per nibble on a voice record. `sh2z_ima_decode`:

- reads one precomputed 89 x 8 table of deltas (uint16, 1,424 bytes) instead of the step
  table and the bit tests;
- clamps each value only on the side the nibble moves it: an add to the predictor can only
  overflow upward, nibbles 4..7 only raise the index and 0..3 only lower it;
- decodes a run per call, two nibbles per byte load.

A next-index table would save one compare per nibble for 712 more bytes. It is left out:
on the targets this was written for, the 4 KB SH-2 cache and cartridge space are scarcer.

## Evidence

**HOST** ([`test_adpcm.c`](tests/test_adpcm.c)): the table is recomputed from the step
table; every (index, nibble, nibble parity) transition is checked at 11 predictor values
around both rails; random streams (full range, rail-driving, quiet) are decoded in random
pieces against one reference call, with guard words after the output. GCC with
AddressSanitizer and UndefinedBehaviorSanitizer, `-Werror -Wconversion -Wpedantic`:
432,040 checks at `SH2Z_ADPCM_ITERS=200000`.

**MODELLED** ([`benchmarks/adpcm-model`](benchmarks/adpcm-model/README.md)): the compiled
SH-2 code runs in the pinned Yabause interpreter (flat RAM, no caches or wait states; not
Saturn or 32X timing). All three entry points are verified against the reference first
(2,001,036 values, [CI run 44311978373](https://github.com/lamb2k/SH2ZAM/actions/runs/44311978373)), then timed over 256 streams per size:

| Decoder | 1 nibble | 32 nibbles | 512 nibbles |
|---|---:|---:|---:|
| Per-nibble step function on a voice record | 128.4 | 85.9 | 84.6 |
| Plain C loop, bitwise rule inline | 92.8 | 61.6 | 60.2 |
| **`sh2z_ima_decode`** | 102.3 | **33.7** | **31.6** |

Cycles per nibble. From 32 nibbles up, the library is 45% to 48% faster than the plain
loop and 61% to 63% faster than the per-nibble function. For single nibbles the call setup
dominates; use it on runs.

**IN A GAME, EMULATED**: the Sega 32X port of Altered Beast
([sega16-2-32x](https://github.com/mholzinger/sega16-2-32x)) decodes its speech and
effects with this kernel. On ares (cycle-modelled SH-2 with bus waits; not hardware),
level 1, frames 1800 to 3600, the decode work on the slave CPU fell from 16,721 to 11,273
cycles per frame (-33%; it reads the bank and table from cartridge memory). Its output was
checked byte for byte against the port's old decoder on 102.4 million samples.

**CONSOLE**: none yet.
