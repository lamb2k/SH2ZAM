# Capability comparison with SH4ZAM

This is a functional-category comparison, not a percentage-complete claim. Scope is based on the upstream README/API categories; implementation and numeric contracts differ. SH4ZAM source snapshot inherited by this repo is retained for attribution, not linked into the Saturn core.

| Category | SH2ZAM 0.2 | Explicit gap / decision |
|---|---|---|
| Scalars | Q16.16 arithmetic, rounding, sqrt/rsqrt, reciprocal, interpolation | No float ABI, NaN/Inf, exp/log/pow/hyperbolic families |
| Trigonometry | sin/cos/sincos/tan, atan/atan2/asin/acos, turn angles | No float-radian API; inverse trig is a bounded-search baseline, not a performance claim |
| Vectors | 2/3/4 arithmetic, dot, length, normalize, lerp; cross2/3 | Not every upstream convenience/swizzle/utility |
| Matrices | 2x2/3x3/3x4/4x4; multiplication/transforms; 3x3/4x4 transpose; rigid inverse; camera helpers | No 3x2/4x3 distinct layouts, general inverse/decomposition/determinant API |
| Quaternions | Product, normalize, conjugate, axis-angle, nlerp, matrix and vector rotation | No SLERP, quaternion exp/log or full upstream facade |
| XMTRX | Explicit caller-owned affine transform stack | No SH4 floating-point register bank emulation or implicit mutable globals |
| Complex/FFT | Not implemented | Defer until a concrete audio/signal workload and overflow/scaling contract exist |
| Memory routines | Use platform SDK/libc | No speculative memcpy/memmove replacement, store queues or cache side effects |
| C++ | C interop plus fixed/vec3 C++11 conveniences | Not complete upstream C++ feature parity |
| Saturn game helpers | AABB/sphere collision, screen projection, batch and balanced partition helpers | No renderer, polygon clipping, DMA or master/slave scheduler |
| Validation | Host oracle/sanitizer suites; SH-2 compile/audit and Yaul-disc CI | Hardware execution, dual-CPU stress and console timing still required |

## Why this is a useful stopping point

A game can now compute 2D motion, transform 3D entities, construct a camera/projection, update orientations, perform basic collision broad-phase tests and partition a transform batch without runtime floating point or allocation. There are examples and documented failure contracts for those workflows. Adding FFT and advanced scalar functions simply to tick feature boxes is lower priority than validating the common paths on Saturn.

## Compatibility cautions

Matrices: row-major, column vectors. Quaternions: x,y,z,w. Angles: unsigned 16-bit turns. Scalars: signed Q16.16, saturating. Dot products now round once and saturate last; some results intentionally differ from the initial per-product-rounded prototype. Normalize handles tiny/large vectors without first saturating squared length. SH4ZAM float data cannot be reinterpreted as these types.
