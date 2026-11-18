# Validation record — 2026-11-17 (variable-length decoding, vlc-1)

`sh2zam/vlc_experimental.h` ([guide](../VLC.md)) adds a bounded bit reader, a length-limited
canonical code builder and a two-level table decoder with a hand-written SH-2 kernel.
**No console or emulator run was done.** Evidence by tier:

- **HOST:** `vlc_exact` passes under GCC and Clang Release with `SH2Z_STRICT=ON` and under
  ASan/UBSan, against the independent oracle `tests/vlc_oracle.h`: 3,300 builds over every
  `w` and `lmax` equal to the oracle and to the contributed builder, 3,000 length sets,
  4,154,640 round-trip symbols through exact-size heap buffers, 10,121 truncated streams,
  random streams and tables, 20,000 garbage tables through the check, and 8 negative
  controls (2,153,117 checks).
- **MODELLED:** the compiled SH-2 kernel, the library's C path and a plain C decoder match
  the oracle on 3,355,101 symbols each (tables, return values, reader state, no read past
  any stream) in the Yabause interpreter before timing
  ([results](../benchmarks/vlc-model/results.json), captured by [CI run 44179557524](https://github.com/lamb2k/SH2ZAM/actions/runs/44179557524)). The kernel decodes in 20.5 to 40.5 modelled
  cycles per symbol, 1.7 to 2.0 times fewer than C. Cycle numbers are modelled, not Saturn
  timings.
- **Not done:** console or emulator execution, cache and memory timing, a whole tile decode.
