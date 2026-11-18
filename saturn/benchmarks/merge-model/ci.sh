#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (c) 2026 SH2ZAM Contributors.
# Tier merge planner (view-merge-2) for the executed-instruction model. The library
# object must not need memcpy/memset and must contain the hand-written SH-2 kernel.
# Phases and environment: saturn/benchmarks/ci/suites.sh and ci/lib.sh.
source "$(dirname "$0")/../ci/lib.sh"
case "${1:?usage: ci.sh phases|build|model}" in
  phases) echo build model ;;
  build)
    sh2z_bridge saturn/benchmarks/merge-model/bridge.c build/sh2/merge-bridge.o
    "$SH2Z_OBJDUMP" -dr build/sh2/sh2zam_view_merge_tier.o > sh2-merge-tier-disassembly.txt
    "$SH2Z_NM" -u build/sh2/sh2zam_view_merge_tier.o | tee sh2-merge-tier-undefined.txt
    sh2z_require_no_mem_calls sh2-merge-tier-undefined.txt
    grep -q "<_sh2zi_merge_tier24>:" sh2-merge-tier-disassembly.txt
    sh2z_model_elf build/sh2/merge-bridge.o build/sh2/merge-model.elf
    ;;
  model)
    sh2z_model_env
    export MODEL_PLACEMENT_LOG="$PWD/build/sh2-model/merge-placement.jsonl"
    bash saturn/benchmarks/merge-model/run.sh \
      build/sh2/merge-model.elf \
      build/sh2-model/merge-results.json
    ;;
  *) echo "ci.sh: unknown phase $1" >&2; exit 2 ;;
esac
