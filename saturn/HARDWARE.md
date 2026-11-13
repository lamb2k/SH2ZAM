# Saturn integration and qualification

## Build target

Use C11, `-m2 -mb`, and a bare-metal Saturn-compatible compiler/runtime. The supplied toolchain uses `sh2eb-elf-gcc`. `SH2Z_REQUIRE_SH2=1` rejects accidental non-SH2 builds; little-endian SuperH builds are rejected. `SH2Z_FORCE_PORTABLE=1` selects the C multiplication reference. Runtime `sh2z_backend()` identifies the selected multiplication implementation, not whether a console test occurred.

The SH-2 path uses signed 32x32 multiplication (`dmuls.l`) and reads MACH/MACL in one compiler-visible assembly block. It does not use SH-4 FPU instructions, XMTRX, store queues, or hardware divider registers.

## Runtime dependencies and concurrency

No allocation, float math, TLS, pthread dependency or mutable global math context exists in the Saturn core. Compiler-generated integer helpers (including 64-bit division/shifts) and ordinary runtime memory helpers may be required; link through the matching SDK/compiler, not an unrelated SH4 Linux libgcc. CI records undefined archive symbols and links the diagnostic app through Yaul.

Every interrupt handler that uses MACH/MACL must preserve interrupted values according to your SDK ABI. Separate CPUs/contexts may call the library, but callers own shared input/output buffers, completion signaling, cache coherency and DMA synchronization. `volatile` alone is not a cross-CPU ownership protocol. Batch jobs must write disjoint ranges. There is no hidden second-CPU startup or cache flush.

## Numeric limits

Q16.16 has approximately +/-32768 range. Dot/matrix operations round down once and saturate their final scalar result; later composition cannot recover information lost to earlier saturation. Rescale world units and intermediates. Vector normalization rescales internally to retain tiny and full-range directions. Length saturates only its final result.

Perspective/view constructors report false for degenerate or unrepresentable results and preserve their output on failure. Rigid inverse accepts only orthonormal bases within its documented tolerance; it is not a general affine inverse. Projected positions use a top-left origin and range 0..width-1 / 0..height-1. Vertex rejection is not polygon clipping: implement near-plane and polygon clipping before VDP1 submission in a real renderer.

## Diagnostic app

See [examples](examples/README.md). The Yaul app displays the backend, an eight-category failure bitmask, an orbit position and one transformed cube vertex. The **expected** result is `Failures: 00000000`; this document does not report a physical-console run.

| Mask bit | What failed |
|---|---|
| 0x01 | Scalar multiplication/division |
| 0x02 | One-LSB normalization |
| 0x04 | Large-vector length |
| 0x08 | Cancellation in a full-range dot |
| 0x10 | Quarter-turn matrix rotation |
| 0x20 | Clip-space projection |
| 0x40 | Partitioned batch vs serial result |
| 0x80 | Quaternion rotation |

Record the actual result in an issue with commit hash, compiler version, image digest, hardware/emulator, boot setup and a photograph/log. An emulator result must not be relabeled as a console result.

## Performance measurement plan

Use the engine's validated timer/profiler. Warm up caches, measure repeated batches, subtract harness overhead, retain output checksums and repeat at representative batch sizes. Compare the portable and SH-2 paths under identical flags and memory placement. Measure interrupt overhead and whole-frame time too. No FPS/cycle gain is asserted by this foundation.

SDK/toolchain references: [Yaul](https://github.com/yaul-org/libyaul), [documented Docker image](https://github.com/yaul-org/libyaul-docker), and [GCC SH target flags](https://gcc.gnu.org/onlinedocs/gcc/SH-Options.html).
