#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (c) 2026 SH2ZAM Contributors.
# The view batch and divider kernel executed in the SH-2 instruction model, from
# the Yaul benchmark ELF (saturn/benchmarks/yaul), with the placement log.
# Phases and environment: saturn/benchmarks/ci/suites.sh and ci/lib.sh.
source "$(dirname "$0")/../ci/lib.sh"
case "${1:?usage: ci.sh phases|model}" in
  phases) echo model ;;
  model)
    export MODEL_SUITE=view MODEL_EVIDENCE_URL="${SH2Z_EVIDENCE_URL:-}"
    export MODEL_PLACEMENT_LOG="$PWD/build/sh2-model/view-placement.jsonl"
    bash saturn/benchmarks/sh2-model/run.sh \
      saturn/benchmarks/yaul/build/sh2zam-benchmark.elf \
      saturn/benchmarks/yaul/build/sh2zam-benchmark.sym \
      build/sh2-model/view-results.json
    ;;
  *) echo "ci.sh: unknown phase $1" >&2; exit 2 ;;
esac
