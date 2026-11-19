# Adding a kernel suite

A kernel lane adds one contract (tag `<tag>`, for example `view-example-1`) with
its library code, host tests, SH-2 instruction-model suite and documents. Lanes
run in parallel, so a lane should **create new files only** and touch at most
one line in each of a few sorted lists. Nothing below needs an edit to the CI
workflows, `saturn/CMakeLists.txt` or `saturn/benchmarks/yaul/Makefile`.

In the paths below, `<name>` is the suite name (`example`), `<family>` the host
test family, and `<tag_>` the tag with `_` for `-` (`view_example_1`).

## New files

| File | What picks it up |
| --- | --- |
| `saturn/src/sh2zam_<name>.c` (and any `*_internal.h`) | CMake builds every `saturn/src/*.c` (sorted glob); the SH-2 cross build compiles every `saturn/src/*.c` into `build/sh2/` and `libsh2zam.a`. |
| `saturn/include/sh2zam/contracts/<tag_>.h`, or a new `saturn/include/sh2zam/<family>_experimental.h` | Your public API, in a "Contract tag `<tag>`" section with its own include guard and `extern "C"` block. A `contracts/` header starts with `#include "../view_experimental.h"` (or `shade_`) and gets one line in that umbrella header (below). |
| `saturn/tests/test_<family>.c` (and `<family>_oracle.h`) | CMake registers every `tests/test_<family>.c` as `sh2zam_<family>_tests` / ctest `<family>_exact`, linked with the library and `-lm`, with the strict warnings. Host CI runs it under GCC and Clang strict and ASan/UBSan. |
| `saturn/benchmarks/<name>-model/bridge.c`, `run.sh`, `README.md` | Your instruction-model suite, as in `camera-model/`. |
| `saturn/benchmarks/<name>-model/ci.sh` | The SH-2 cross workflow finds it (`saturn/benchmarks/ci/suites.sh`) and runs its phases. Template below. |
| `saturn/benchmarks/sh2-model/<name>_harness.c` | Compiled by your `run.sh`. |
| `saturn/benchmarks/<name>-model/results.json` | Your capture, in the "Record the `<tag>` capture from CI run …" commit. |
| `saturn/<GUIDE>.md` | Your guide. Write a new guide file; do not append sections to `VIEW_BATCH.md`, `SHADE.md` or other existing guides. |
| `saturn/validation/<tag>.md` | Your validation record. Its first line is `# Validation record — <date> (<scope>, <tag>)`. |
| `saturn/benchmarks/yaul/parts/<tag>.mk` (only if the contract adds code to the Yaul benchmark ELF, as the view-model capture does) | Included by the benchmark Makefile in name order after the existing lists. It appends to `SH_SRCS` (paths relative to `saturn/benchmarks/yaul/`) and to `SH_LDFLAGS` (`-Wl,--undefined=...` roots). |

Audit text files go at the repository root as `sh2-<name>-*.txt`. Model outputs
go in `build/sh2-model/` as `<name>-results.json`, `<name>-raw.json` and
`<name>-placement.jsonl`. The workflow uploads them by glob, with `build/sh2/`.

## Sections: one per entry point and per table

Consumers link with `-ffunction-sections -fdata-sections` and
`-Wl,--gc-sections`, so that an entry point they never call costs no RAM.
The compiler splits C by itself; hand-written assembly and tables must do it
explicitly:

- Each global assembly entry goes in its own section, named after the symbol
  without the leading underscore, and leaves it again:
  ```c
  "    .pushsection .text.sh2zi_example_run,\"ax\",@progbits\n"
  "    .align 2\n"
  "    .global _sh2zi_example_run\n"
  ...
  "    .size _sh2zi_example_run,.-_sh2zi_example_run\n"
  "    .popsection\n"
  ```
  Never write a bare `.text` in a top-level `__asm__`.
- An SH-2 branch (`BRA`, `BSR`, `BT`, `BF`) cannot reach another section: gas
  resolves it in place or fails. Code that two entries share by branching
  must be split by instantiating it once per entry (a gas macro, as
  `SH2ZI_FG_SEARCH` in `sh2zam_shade_fused.c`), or be reached by `JSR` or
  `JMP` through a literal (`.long _symbol`), which is a normal relocation and
  keeps the helper (in its own section) whenever any caller is kept.
- A table that assembly defines goes in `.rodata.<symbol>` (for example
  `.pushsection .rodata.sh2zi_vlc8_run,"a",@progbits`). A read-only C table
  takes `SH2ZI_RODATA(<name>)` from `src/sh2zam_sections.h` on its definition
  (`<name>` is its symbol, or `sh2zi_<name>` for a file-static one), which
  places it in `.rodata.<name>` with or without `-fdata-sections`. A consumer's
  linker script can then place a cold table by name, for example in low work
  RAM: `.lwram_tables : { *(.rodata.sh2z_sin12_quarter_u30) } > lwram`.
- `sh2z_require_per_symbol_sections saturn/src/sh2zam_<name>.c` (in
  `ci/lib.sh`) compiles a source with the consumer flags and fails if any
  global symbol, or anything at all, is left in plain `.text`, `.data`,
  `.rodata` or `.bss`. The `gc-probe` suite runs it over every source and
  links one-entry programs with `--gc-sections`; add a probe there for a new
  contract (`benchmarks/gc-probe/README.md`).

## Single lines you touch

| File | Line |
| --- | --- |
| `saturn/VALIDATION.md` | Generated: run `python3 saturn/tools/validation_index.py --write` after adding your record. Host CI runs `--check`. |
| `saturn/README.md`, "Experimental kernel guides" | One line, sorted by guide file name: `- [Guide title](GUIDE.md); [instruction model](benchmarks/<name>-model/README.md)`. |
| `saturn/include/sh2zam/view_experimental.h` or `shade_experimental.h` | Only if your contract extends that header: one `#include "contracts/<tag_>.h"` in the list at the end, sorted by file name. Leave the existing sections alone. |
| `saturn/VIEW_BATCH.md`, Status table | View contracts only: one row, placed in contract-tag order. |

Kernel lanes do not add sections to the root `README.md`. Its overview is
updated when lanes land.

## `ci.sh` template

Copy `saturn/benchmarks/camera-model/ci.sh` and change the names. Each phase
runs from the repository root, under `set -euo pipefail`, so any failing
`grep -q` fails the suite.

```bash
#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (c) 2026 SH2ZAM Contributors.
# Example bridge (view-example-1) for the executed-instruction model. The library
# object must need nothing external and must contain the hand-written kernel.
# Phases and environment: saturn/benchmarks/ci/suites.sh and ci/lib.sh.
source "$(dirname "$0")/../ci/lib.sh"
case "${1:?usage: ci.sh phases|build|model}" in
  phases) echo build model ;;
  build)
    sh2z_bridge saturn/benchmarks/example-model/bridge.c build/sh2/example-bridge.o
    "$SH2Z_OBJDUMP" -dr build/sh2/sh2zam_example.o > sh2-example-disassembly.txt
    "$SH2Z_NM" -u build/sh2/sh2zam_example.o | tee sh2-example-undefined.txt
    sh2z_require_no_undefined sh2-example-undefined.txt
    sh2z_require_per_symbol_sections saturn/src/sh2zam_example.c
    grep -q "<_sh2z_example>:" sh2-example-disassembly.txt
    sh2z_model_elf build/sh2/example-bridge.o build/sh2/example-model.elf
    ;;
  model)
    sh2z_model_env
    bash saturn/benchmarks/example-model/run.sh \
      build/sh2/example-model.elf \
      build/sh2-model/example-results.json
    ;;
  *) echo "ci.sh: unknown phase $1" >&2; exit 2 ;;
esac
```

The phases are:

- **`build`** runs inside the pinned Yaul image, after every library object is
  in `build/sh2/` and `libsh2zam.a` is archived. `lib.sh` gives
  `sh2z_bridge` and `sh2z_model_elf` (the flags every bridge uses),
  `sh2z_require_no_undefined`, `sh2z_require_no_mem_calls` and
  `sh2z_require_per_symbol_sections`. The tools are
  `$SH2Z_CC`, `$SH2Z_OBJDUMP` and `$SH2Z_NM`.
- **`model`** runs on the runner. `sh2z_model_env` exports `MODEL_EVIDENCE_URL`,
  `MODEL_TOOLCHAIN` and `MODEL_SOURCE_COMMIT` for your `run.sh`. Export
  `MODEL_PLACEMENT_LOG="$PWD/build/sh2-model/<name>-placement.jsonl"` if your
  harness writes one.
- **`report`** runs after every suite's `model` phase. Only
  `placement-model` uses it.

Suites run in C-locale order of their directory names. `suites.sh list` prints
each suite and its phases. The workflow's step summary gives a table of every
suite and phase that ran.

## Running a suite locally

In the Yaul toolchain, after the library objects and archive are in
`build/sh2/` (built with the same commands as `sh2-cross.yml`):

```sh
SH2Z_CC=sh2eb-elf-gcc SH2Z_OBJDUMP=sh2eb-elf-objdump SH2Z_NM=sh2eb-elf-nm \
  bash saturn/benchmarks/example-model/ci.sh build
```

Then, on the host, with the pinned Yabause in `YABAUSE_SOURCE`:

```sh
bash saturn/benchmarks/example-model/ci.sh model
```

`bash saturn/benchmarks/ci/suites.sh build` (or `model`) runs every suite.

## What is still shared

These still need coordinating when two lanes change them:

- the view-model capture chain in the Yaul benchmark ELF (`view_bridge.c`,
  the `round.h`, `center.h` and `guardsrc.h` chain, `view-model/collect.py`);
- `saturn/benchmarks/sh2-model/harness.c` and `run.sh`, when a suite needs a new
  harness mode;
- the placement model (`placement-model/collect.py` and its `ci.sh`), when a
  suite adds a placement log to its report;
- an existing guide or header section that a lane extends instead of adding its
  own.

## Merge trains and releases

Contracts that are ready land together in a merge train, as #25 and #30 did:

1. One branch (for example `claude/merge-train-2`) merges each car's PR with
   `--no-ff`, in a fixed order.
2. One CI run on that branch produces every car's model capture.
3. One commit records them: "Record the train-<n> captures from CI run …".
4. The train merges to `main`, and every car's tag goes on the train's merge
   commit.
5. **Each train's tags get a GitHub release with notes, in the same landing.**
   Create them in car order once the tags are pushed:

   ```sh
   gh release create <tag> --verify-tag \
     --title "<tag>: <short description>" --notes-file <tag>.md --latest=false
   ```

   - `--verify-tag` stops if the tag is not on the remote, so a release never
     creates or moves a tag.
   - Pass `--latest` instead of `--latest=false` on the train's last car, so the
     newest train is marked Latest.
   - Notes say what the contract adds, its evidence tiers (HOST, MODELLED,
     HARDWARE) and any targets not met. They link the guide at the tag, and
     name the train and the other tags on the same commit. Write only what the
     PRs and docs show.
   - No binary assets; GitHub's source archives are enough.

A contract that lands on its own follows steps 4 and 5 for its single tag.
