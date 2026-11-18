#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (c) 2026 SH2ZAM Contributors.
# vlc bridge (vlc-1) for the executed-instruction model. The library object
# must not need memcpy/memset and must contain the hand-written SH-2 decoder.
# Phases and environment: saturn/benchmarks/ci/suites.sh and ci/lib.sh.
source "$(dirname "$0")/../ci/lib.sh"
case "${1:?usage: ci.sh phases|build|model}" in
  phases) echo build model ;;
  build)
    sh2z_bridge saturn/benchmarks/vlc-model/bridge.c build/sh2/vlc-bridge.o
    "$SH2Z_OBJDUMP" -dr build/sh2/sh2zam_vlc.o > sh2-vlc-disassembly.txt
    "$SH2Z_NM" -u build/sh2/sh2zam_vlc.o | tee sh2-vlc-undefined.txt
    sh2z_require_no_mem_calls sh2-vlc-undefined.txt
    grep -q "<_sh2zi_vlc8_run>:" sh2-vlc-disassembly.txt
    grep -q "dmulu.l" sh2-vlc-disassembly.txt
    sh2z_model_elf build/sh2/vlc-bridge.o build/sh2/vlc-model.elf
    ;;
  model)
    sh2z_model_env
    export MODEL_PLACEMENT_LOG="$PWD/build/sh2-model/vlc-placement.jsonl"
    bash saturn/benchmarks/vlc-model/run.sh \
      build/sh2/vlc-model.elf \
      build/sh2-model/vlc-results.json
    ;;
  *) echo "ci.sh: unknown phase $1" >&2; exit 2 ;;
esac
