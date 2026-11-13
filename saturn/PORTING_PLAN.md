# SH2ZAM qualification and next steps

## Landed foundation

Scalar/trig and 2/3/4D vectors; cancellation-safe dot/matrix products; 2x2/3x3/3x4/4x4 transforms; camera/projection; quaternion rotation/nlerp; bounds; caller-owned stack; batch partitioning; C++11 conveniences; installable CMake package; host regression/oracle/sanitizer coverage; freestanding example kernels; Yaul diagnostic app and cross-build CI.

## Next gate: execution, not more API names

1. Build the Yaul smoke image and boot it on a Saturn emulator and a real console. Record commit, compiler/image digest, boot method, video mode and the displayed mask. Expected mask is zero and backend is `sh2-dmuls`. A successful build alone does not pass this gate.
2. Run the same kernels with `SH2Z_FORCE_PORTABLE=1`; compare outputs and exact edge-case behavior. Test interrupt-heavy workloads, verify MACH/MACL preservation, and stress separate contexts on both CPUs with application-owned synchronization.
3. Measure batched transforms, scalar multiply/divide, normalization and trig in an actual engine. Record cycles per operation, code/data size, compiler flags, cache state and whole-frame timing. Compare against current engine code and Yaul; keep a switchable baseline. Reject optimizations that slow the real workload.

## Then optimize measured bottlenecks

DIVU needs a per-CPU ownership/preemption contract, saturation/zero handling and explicit protection against interrupt/nested use. Do not silently enable MMIO division. Batched MAC assembly needs range/rounding proofs and register-preserving ISR validation. Approximate normalization/trig variants need documented error envelopes and separate names. Exact internal optimizations keep the existing API and numeric contracts. LTO is opt-in until measured.

## Extend only for concrete consumers

General inverses, additional matrix shapes, quaternion SLERP, complex/FFT and advanced scalar functions remain tracked in [PARITY.md](PARITY.md). They are not stubs or claimed as implemented. Existing SDK memory functions remain the default.

## Release criteria

Require host correctness and sanitizers, reproducible SH-2 compile/link evidence, emulator and console zero-mask runs, real-workload timing evidence, retained licensing/attribution, and examples matching the documented API. Do not label a release production-ready or faster-than alternatives before those measurements exist.
