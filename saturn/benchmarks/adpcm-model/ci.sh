#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (c) 2026 SH2ZAM Contributors.
# ADPCM bridge (adpcm-1) for the executed-instruction model. The decoder object
# must need no external symbol (no libgcc shifts or division, no memcpy/memset).
# Phases and environment: saturn/benchmarks/ci/suites.sh and ci/lib.sh.
source "$(dirname "$0")/../ci/lib.sh"
case "${1:?usage: ci.sh phases|build|model}" in
  phases) echo build model ;;
  build)
    sh2z_bridge saturn/benchmarks/adpcm-model/bridge.c build/sh2/adpcm-bridge.o
    "$SH2Z_OBJDUMP" -dr build/sh2/sh2zam_adpcm.o > sh2-adpcm-disassembly.txt
    "$SH2Z_NM" -u build/sh2/sh2zam_adpcm.o | tee sh2-adpcm-undefined.txt
    sh2z_require_no_undefined sh2-adpcm-undefined.txt
    grep -q "<_sh2z_ima_decode>:" sh2-adpcm-disassembly.txt
    sh2z_model_elf build/sh2/adpcm-bridge.o build/sh2/adpcm-model.elf
    ;;
  model)
    sh2z_model_env
    bash saturn/benchmarks/adpcm-model/run.sh \
      build/sh2/adpcm-model.elf \
      build/sh2-model/adpcm-results.json
    ;;
  *) echo "ci.sh: unknown phase $1" >&2; exit 2 ;;
esac
