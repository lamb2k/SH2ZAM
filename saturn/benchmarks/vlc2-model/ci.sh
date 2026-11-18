#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (c) 2026 SH2ZAM Contributors.
# Per-symbol vlc bridge (vlc-2) for the executed-instruction model. The
# library object must need nothing external; the bridge must contain the
# inline decoder (SUBV, DMULU.L) in its tile loop.
# Phases and environment: saturn/benchmarks/ci/suites.sh and ci/lib.sh.
source "$(dirname "$0")/../ci/lib.sh"
case "${1:?usage: ci.sh phases|build|model}" in
  phases) echo build model ;;
  build)
    sh2z_bridge saturn/benchmarks/vlc2-model/bridge.c build/sh2/vlc2-bridge.o
    "$SH2Z_OBJDUMP" -dr build/sh2/sh2zam_vlc2.o > sh2-vlc2-disassembly.txt
    "$SH2Z_OBJDUMP" -dr build/sh2/vlc2-bridge.o > sh2-vlc2-bridge-disassembly.txt
    "$SH2Z_NM" -u build/sh2/sh2zam_vlc2.o | tee sh2-vlc2-undefined.txt
    sh2z_require_no_undefined sh2-vlc2-undefined.txt
    grep -q "<_sh2z_vlc2_build>:" sh2-vlc2-disassembly.txt
    grep -q "subv" sh2-vlc2-bridge-disassembly.txt
    grep -q "dmulu.l" sh2-vlc2-bridge-disassembly.txt
    sh2z_model_elf build/sh2/vlc2-bridge.o build/sh2/vlc2-model.elf
    ;;
  model)
    sh2z_model_env
    export MODEL_PLACEMENT_LOG="$PWD/build/sh2-model/vlc2-placement.jsonl"
    bash saturn/benchmarks/vlc2-model/run.sh \
      build/sh2/vlc2-model.elf \
      build/sh2-model/vlc2-results.json
    ;;
  *) echo "ci.sh: unknown phase $1" >&2; exit 2 ;;
esac
