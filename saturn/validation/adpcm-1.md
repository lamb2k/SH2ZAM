# Validation record — 2026-11-18 (IMA ADPCM decoding, adpcm-1)

`sh2zam/adpcm.h` ([guide](../ADPCM.md)) adds `sh2z_ima_decode`, an exact 4-bit IMA/DVI
ADPCM decoder that works from any nibble position and in pieces of any length, and the
reference decoder `sh2z_ima_ref_decode` in its own file. Evidence by tier:

- **HOST:** `adpcm_exact` checks the decoder against the reference:
  - the 89 x 8 delta table recomputed from the step table;
  - every (index, nibble, nibble parity) transition at 11 predictor values around both
    rails;
  - random streams (full range, rail-driving, quiet) decoded in random pieces against one
    reference call, with guard words after the output.

  432,040 checks at `SH2Z_ADPCM_ITERS=200000` under GCC with ASan/UBSan and
  `-Werror -Wconversion -Wpedantic`.
- **MODELLED:** the compiled SH-2 code of all three bridge entry points matches the
  reference on 2,001,036 values in the Yabause interpreter before any timing
  ([model](../benchmarks/adpcm-model/README.md),
  [results](../benchmarks/adpcm-model/results.json), captured by
  [CI run 44311978373](https://github.com/lamb2k/SH2ZAM/actions/runs/44311978373)). Cycles per nibble,
  CPU-only flat RAM:

  | Decoder | 1 nibble | 32 nibbles | 512 nibbles |
  | --- | ---: | ---: | ---: |
  | Per-nibble step function on a voice record | 128.4 | 85.9 | 84.6 |
  | Plain C loop, bitwise rule inline | 92.8 | 61.6 | 60.2 |
  | `sh2z_ima_decode` | 102.3 | 33.7 | 31.6 |

- **EMULATOR:** the Altered Beast 32X port decodes its speech with this kernel. On ares,
  level 1, frames 1800 to 3600, the slave CPU's decode work fell from 16,721 to 11,273
  cycles per frame (-33%); its output equals the port's old decoder on 102.4 million
  samples. Frame rate did not change.
- **Not done:** console execution; cache and bus placement through `model-placement-1`.
