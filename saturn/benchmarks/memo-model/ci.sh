#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (c) 2026 SH2ZAM Contributors.
# Block memo bridge (memo-1) for the executed-instruction model. The library
# object must need no external symbol and must contain the hand-written kernels.
# The model phase also writes the placement log the suite's run.sh costs from.
# Phases and environment: saturn/benchmarks/ci/suites.sh and ci/lib.sh.
source "$(dirname "$0")/../ci/lib.sh"
case "${1:?usage: ci.sh phases|build|model}" in
  phases) echo build model ;;
  build)
    sh2z_bridge saturn/benchmarks/memo-model/bridge.c build/sh2/memo-bridge.o
    "$SH2Z_OBJDUMP" -dr build/sh2/sh2zam_memo.o > sh2-memo-disassembly.txt
    "$SH2Z_NM" -u build/sh2/sh2zam_memo.o | tee sh2-memo-undefined.txt
    sh2z_require_no_undefined sh2-memo-undefined.txt
    grep -q "<_sh2z_memo_sync16w_batch>:" sh2-memo-disassembly.txt
    grep -q "<_sh2z_memo_put4>:" sh2-memo-disassembly.txt
    grep -q "<_sh2z_memo_copy8_until>:" sh2-memo-disassembly.txt
    grep -q "<_sh2z_memo_diff8_runs>:" sh2-memo-disassembly.txt
    sh2z_model_elf build/sh2/memo-bridge.o build/sh2/memo-model.elf
    ;;
  model)
    sh2z_model_env
    export MODEL_PLACEMENT_LOG="$PWD/build/sh2-model/memo-placement.jsonl"
    bash saturn/benchmarks/memo-model/run.sh \
      build/sh2/memo-model.elf \
      build/sh2-model/memo-results.json
    ;;
  *) echo "ci.sh: unknown phase $1" >&2; exit 2 ;;
esac
