# SH2ZAM roadmap and acceptance gates

**Principle:** match the *jobs* SH4ZAM's API performs, not its SH-4 float or XMTRX implementation. Preserve upstream MIT attribution and fork history. Ship measurable benefits instead of assuming hand assembly is faster.

| SH4ZAM category | Saturn-specific strategy | Initial status |
|---|---|---|
| Scalar float and trig | Q16.16, 16-bit turn angles, quarter-wave LUT | Implemented; host-tested |
| Vec2/3/4 | fixed-point dot, length, normalize, cross | Partial; host-tested |
| Matrix 2x2/3x2/3x3/3x4/4x3/4x4 | affine 3x4 + general 4x4 first | Partial; host-tested |
| Quaternions | Hamilton mul/normalize; then interpolation and to/from matrix | Partial; host-tested |
| XMTRX | no Saturn equivalent; software matrix stack / SH-2 MAC pipeline | Not implemented |
| Complex/FFT | fixed-point complex + scale-safe radix-2 FFT | Not implemented |
| `memcpy`/`memset`/`memmove` | benchmark against libyaul/libc before custom loops; alignment-safe | Not implemented |
| Transcendentals | LUT/CORDIC per operation, measured error bounds | Not implemented |

## Gate 1 — build / numerical parity

1. Compile with both a native host compiler and `sh2eb-elf-gcc -m2 -mb` (C11).
2. Test portable and SH-2-specific multiply on real hardware for signed, extreme, and random operands against a 64-bit reference. Check compiler clobbers and register allocation.
3. Add accuracy envelopes for sin/cos, matrix composition, quat normalization, and fixed-point overflow behavior. Verify big-endian assumptions and data alignment.
4. Run Saturn emulator tests (Mednafen/Kronos) and a console smoke test; neither has been performed yet.

## Gate 2 — benchmark before optimizing

1. Capture cycles/function and complete frame time on target Saturn scenes; compare Yaul gamemath/fix16 and SaturnMath++ using equivalent algorithms and numeric formats.
2. Separate cost of math from cache, CD load, VDP1 command building, slave CPU synchronization, and CPU stalls.
3. Test hardware DIVU only with a reentrancy/interrupt contract; compare long division fallback and software reciprocal approximations.
4. Try MAC-based dot products, matrix transpose/layout experiments, compile-time trig tables, direct DMA-friendly transform output. Keep only wins on physical hardware.

## Gate 3 — API breadth and adoption

1. Add 2x2, 3x3 and 4x3 matrices, quaternion-to-matrix, angle interpolation, cross-format Q8.24/Q24.8 utilities.
2. Add complex arithmetic and fixed-point FFT with automatic scaling to prevent saturation.
3. Provide an ergonomic C++ interface as an *adapter* over the tested C core; don't require C++23 in Saturn toolchains.
4. Integrate with Yaul as a `libsh2zam.a` and example scene demonstrating correctness, profiling and two-CPU safe use.
5. Send upstream a respectful proposal to link/reference SH2ZAM and contribute cross-platform improvements rather than forking individual SH4 routines.

**Release definition:** no "Saturn-optimized" performance claim or semantic parity claim until a real SH-2 benchmark and functional parity matrix pass. `SH2ZAM` is an experimental project name, not an official upstream SH4ZAM release.
