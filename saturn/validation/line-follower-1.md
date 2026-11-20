# Validation record — 2026-11-19 (racing-line follower, line-follower-1)

`sh2zam/line_follower_experimental.h` ([guide](../LINE_FOLLOWER.md)) moves a vehicle along a baked
closed line, as a distance plus a lateral offset:
- node lookup and sampling;
- advance with lap counting;
- speed and steering steps for 1 to 64 ticks;
- a pace target.

Integer only, with no divide at run time. **No console run was done in this repository.** Evidence by tier:

- **HOST:** `line_follower_exact` passes 2,173,420 checks under GCC and Clang Release with `SH2Z_STRICT=ON` and under ASan/UBSan. The oracles use int64
  arithmetic on the documented formulas and brute-force searches over random lines.
- **SH-2 build:** the source compiles with the CI flags and leaves nothing in plain sections
  under the consumer flags. The object is 2,020 bytes (`-m2 -mb -O2`); it needs `___divdi3` and
  `___udivdi3`.
- **Not done:** an instruction-model capture, console execution and any frame measurement. The
  follower has been measured only inside a larger program.
