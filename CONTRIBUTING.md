# Contributing to SH2ZAM

Keep the root MIT notice, upstream credits and file-specific provenance. Original SH4ZAM guidance is preserved in CONTRIBUTING.SH4ZAM.md; use upstream SH4ZAM for Dreamcast improvements.

For Saturn changes, document public numeric ranges, rounding/saturation, aliasing, failure behavior, memory ownership and hardware assumptions. Add regression/oracle tests for correctness bugs. Tests must still execute in Release builds. Keep portable reference behavior available.

Run root CMake tests with GCC and Clang, then ASan/UBSan. SH-2 changes must cross-compile and link with the Saturn toolchain. Console performance claims require console measurements, not desktop benchmarks or instruction counts. Treat the Yaul diagnostic image as a test artifact until it has actually booted.

Avoid implicit hardware-divider access, mutable global transforms, hidden cache/DMA work or undocumented approximation modes. Add examples that use the real API and keep their instructions current. Use separate feature branches for unqualified optimizations.
