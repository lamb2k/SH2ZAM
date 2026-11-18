# Validation record — 2026-11-18 (per-symbol variable-length decoding, vlc-2)

`sh2zam/vlc2_experimental.h` ([guide](../VLC2.md)) adds an inline per-symbol decode step:
SH-2 assembly with the bit reader in the caller's registers and a table pointer per symbol.
It also adds compact byte tables that pack together in one arena, a builder and a check, and
a reader that starts at any byte offset and is bounded once per stream. vlc-1 is unchanged.
**No console or emulator run was done.** Evidence by tier:

- **HOST:** `vlc2_exact` passes under GCC and Clang Release with `SH2Z_STRICT=ON` and under
  ASan/UBSan, against the independent oracle `tests/vlc2_oracle.h`:
  - 2,097,144 builds, covering every length vector of up to 6 symbols at every window (10,388
    accepted), and 4,000 random builds, all equal to the oracle byte for byte;
  - 943,800 round-trip symbols in heap buffers of exactly the stated bound, at even and odd
    starts;
  - 3,000 random-byte and truncated streams under the once-per-stream bound;
  - 40,000 garbage tables through the check;
  - 8 negative controls caught.
- **MODELLED:** the compiled SH-2 builder (6,960 builds), the inline assembly and the
  SH-2-compiled C path (2,772,366 symbols each, every window, odd starts, random bytes,
  tables switched every symbol) and 1,536 synthetic tile decodes match the oracle and the
  generator in the Yabause interpreter, with any read past the stream bound or a table
  trapped ([model](../benchmarks/vlc2-model/README.md), captured by
  [CI run 44243332134](https://github.com/lamb2k/SH2ZAM/actions/runs/44243332134)). On a generic context-modelled 4 bpp
  tile workload the decoder averages 24.0 modelled cycles per symbol (13 for a first-level
  code). Tiles of 68.5 symbols take 3,081 cycles CPU only, 3,972 placed in high WRAM and
  8,825 with the tables in the cartridge. That is 3.1 times faster than one vlc-1 call per
  symbol. Tiles of 104.5 symbols take 4,356, 5,723 and 13,229. Cycle numbers are modelled,
  not Saturn timings.
- **Not done:** console or emulator execution; a measurement of the contributor's own tile
  decoder using this step.
