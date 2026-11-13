# Exact math improvements on the SH-2 model

This suite compares **previous SH2ZAM at `10786b6` with the optimized
SH2ZAM library**, compiled together for the same big-endian SH-2 target.
It is a library optimization comparison. The site's separate vertex suite
contains the independent plain-C fixed-point adoption baseline and software
float comparison; the frozen implementation here must not be presented as
an independent non-SH2ZAM implementation.

## What runs

The suite covers 48 operations: scalar arithmetic, square roots, vector
dot/cross/length/normalization, matrix multiplication and transforms,
quaternions, trigonometry, camera/projection functions, and sphere collision.
Each operation runs on three deterministic profiles:

| Profile | Inputs |
| --- | --- |
| `bounded` | Ordinary signed Q16.16 values, generally within ±8; all trig quadrants; useful in-domain inverse trig and projection inputs. |
| `tiny` | Vector/scalar components within ±32 raw units, including zero and one-LSB behavior; small positive projection `w`. |
| `full-range` | Signed32 random values and explicit extreme/zero/one cases; matrices can exceed the direct accumulation bound; inverse trig includes out-of-domain clamping. |

The names describe input generation, not a promise that every call takes a
particular branch. Operations also include documented failure/zero cases.
Angles cover all quadrants in every profile. Camera, perspective and clip
fixtures use operation-appropriate construction rather than interpreting
arbitrary bytes as valid geometry. See `math_bridge.c` for the exact recipe.

There are seven deterministic batches of 32 calls per operation/profile.
Fixture preparation and verification are outside the measured range. Both
versions execute the same range wrapper, API calls and result writes.
Matrices include their normal alias-safe output temporaries; no extra
preparation is silently charged only to one side. Reported cycles per call
are the median batch cycle count divided by 32, including amortized wrapper
overhead. The suite retains raw cycle, instruction, read and write counts,
including every regression. It does not average unrelated APIs into a single
marketing speedup.

Every output word must match the frozen implementation. Output padding must
remain untouched, input buffers must remain unchanged, and native entry/exit
checks verify the stack, callee-saved registers, and saturation-mode bit.
Every operation/profile is also rerun with `SR.S=1`; guard reruns do not enter
the measured medians. Independent host oracles, exhaustive trig comparisons,
and sanitizer tests are separate correctness gates.

## Reproduce

The `SH-2 cross compile` workflow builds both versions with the same pinned
Yaul GCC toolchain and runs the compiled ELF through the existing unmodified
Yabause SH-2 interpreter. The workflow artifact contains the ELF, symbols,
raw observations, packaged results, and disassembly.

```sh
# Build the benchmark with the documented Yaul image first.
MODEL_SUITE=math bash saturn/benchmarks/sh2-model/run.sh \
  saturn/benchmarks/yaul/build/sh2zam-benchmark.elf \
  saturn/benchmarks/yaul/build/sh2zam-benchmark.sym \
  build/sh2-model/math-results.json
node saturn/benchmarks/math-model/validate.cjs build/sh2-model/math-results.json
```

`legacy/manifest.json` records the original commit and SHA-256 of every
frozen source file. The wrappers rename public symbols without changing
the frozen implementations. The collector verifies those hashes and binds
the result to the source commit, compiler/flags, ELF, symbols, fixture code,
raw observations, and interpreter source. New captures do not rewrite the
provenance of older captures.

## Limits

These are **modeled SH-2 instruction costs**, not browser timings, physical
Saturn timings, or full-system emulator timings. The model has flat RAM and
no cache, bus wait states, pipeline or multiplier contention, DMA, interrupt,
peripheral, BIOS, or second-CPU simulation. No browser/host elapsed time is
used as a console-performance proxy. Do not convert these results to Saturn
milliseconds, FPS, or claimed whole-game gains.

Changes are retained based on this model with exact outputs. Real hardware
benchmarking remains necessary to assess code-size/cache effects, scheduling,
memory alignment, batch sizes, and integration with other Saturn work.

## Licensing

The math library, frozen benchmark source, fixtures and target wrappers are
MIT licensed. `capture.h` and `collect.py` extend the optional
GPL-2.0-or-later model tooling and are not part of the shipping MIT library
or browser build. Yabause remains
an unmodified, separately fetched GPL dependency; see `../sh2-model/LICENSE`.
