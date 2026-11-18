#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (c) 2026 SH2ZAM Contributors.
# unit30 bridge (fixed-unit-1) for the executed-instruction model. The library
# object must need nothing undefined (no memcpy/memset, no libgcc helper) and
# must contain the hand-written normalisation; the inline MAC.L dot product
# is instantiated in the bridge.
# Phases and environment: saturn/benchmarks/ci/suites.sh and ci/lib.sh.
source "$(dirname "$0")/../ci/lib.sh"
case "${1:?usage: ci.sh phases|build|model}" in
  phases) echo build model ;;
  build)
    sh2z_bridge saturn/benchmarks/unit30-model/bridge.c build/sh2/unit30-bridge.o
    "$SH2Z_OBJDUMP" -dr build/sh2/sh2zam_unit30.o > sh2-unit30-disassembly.txt
    "$SH2Z_OBJDUMP" -dr build/sh2/unit30-bridge.o > sh2-unit30-bridge-disassembly.txt
    "$SH2Z_NM" -u build/sh2/sh2zam_unit30.o | tee sh2-unit30-undefined.txt
    sh2z_require_no_undefined sh2-unit30-undefined.txt
    grep -q "<_sh2z_vec3_normalize_u30>:" sh2-unit30-disassembly.txt
    grep -q "dmuls.l" sh2-unit30-disassembly.txt
    grep -q "mac.l" sh2-unit30-bridge-disassembly.txt
    sh2z_model_elf build/sh2/unit30-bridge.o build/sh2/unit30-model.elf
    ;;
  model)
    sh2z_model_env
    bash saturn/benchmarks/unit30-model/run.sh \
      build/sh2/unit30-model.elf \
      build/sh2-model/unit30-results.json
    ;;
  *) echo "ci.sh: unknown phase $1" >&2; exit 2 ;;
esac
