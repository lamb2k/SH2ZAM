#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (c) 2026 SH2ZAM Contributors.
# Fused shade bridge (shade-2) for the executed-instruction model. The library
# object must need no external symbol and must contain the hand-written kernel.
# Phases and environment: saturn/benchmarks/ci/suites.sh and ci/lib.sh.
source "$(dirname "$0")/../ci/lib.sh"
case "${1:?usage: ci.sh phases|build|model}" in
  phases) echo build model ;;
  build)
    sh2z_bridge saturn/benchmarks/shade-fused-model/bridge.c build/sh2/shade-fused-bridge.o
    "$SH2Z_OBJDUMP" -dr build/sh2/sh2zam_shade_fused.o > sh2-shade-fused-disassembly.txt
    "$SH2Z_NM" -u build/sh2/sh2zam_shade_fused.o | tee sh2-shade-fused-undefined.txt
    sh2z_require_no_undefined sh2-shade-fused-undefined.txt
    grep -q "<_sh2z_shade_fog_gouraud>:" sh2-shade-fused-disassembly.txt
    grep -q "dmulu.l" sh2-shade-fused-disassembly.txt
    grep -q "muls.w" sh2-shade-fused-disassembly.txt
    sh2z_model_elf build/sh2/shade-fused-bridge.o build/sh2/shade-fused-model.elf
    ;;
  model)
    sh2z_model_env
    bash saturn/benchmarks/shade-fused-model/run.sh \
      build/sh2/shade-fused-model.elf \
      build/sh2-model/shade-fused-results.json
    ;;
  *) echo "ci.sh: unknown phase $1" >&2; exit 2 ;;
esac
