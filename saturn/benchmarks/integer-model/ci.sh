#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (c) 2026 SH2ZAM Contributors.
# Integer-kernel bridge (integer-kernels-1): 68000 arithmetic, box overlap and cell
# placement, executed in the SH-2 instruction model.
# Phases and environment: saturn/benchmarks/ci/suites.sh and ci/lib.sh.
source "$(dirname "$0")/../ci/lib.sh"
case "${1:?usage: ci.sh phases|build|model}" in
  phases) echo build model ;;
  build)
    sh2z_bridge saturn/benchmarks/integer-model/bridge.c build/sh2/integer-bridge.o
    # The m68k kernels are header-inline: audit the bridge object that instantiates them.
    "$SH2Z_OBJDUMP" -d build/sh2/integer-bridge.o > sh2-m68k-disassembly.txt
    grep -q "div1" sh2-m68k-disassembly.txt
    grep -q "muls.w" sh2-m68k-disassembly.txt
    sh2z_model_elf build/sh2/integer-bridge.o build/sh2/integer-model.elf
    ;;
  model)
    sh2z_model_env
    bash saturn/benchmarks/integer-model/run.sh \
      build/sh2/integer-model.elf \
      build/sh2-model/integer-results.json
    ;;
  *) echo "ci.sh: unknown phase $1" >&2; exit 2 ;;
esac
