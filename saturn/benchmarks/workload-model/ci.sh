#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (c) 2026 SH2ZAM Contributors.
# Complete workloads executed in the SH-2 instruction model, from the Yaul
# benchmark ELF (saturn/benchmarks/yaul).
# Phases and environment: saturn/benchmarks/ci/suites.sh and ci/lib.sh.
source "$(dirname "$0")/../ci/lib.sh"
case "${1:?usage: ci.sh phases|model}" in
  phases) echo model ;;
  model)
    export MODEL_SUITE=workloads MODEL_EVIDENCE_URL="${SH2Z_EVIDENCE_URL:-}"
    bash saturn/benchmarks/sh2-model/run.sh \
      saturn/benchmarks/yaul/build/sh2zam-benchmark.elf \
      saturn/benchmarks/yaul/build/sh2zam-benchmark.sym \
      build/sh2-model/workload-results.json
    node saturn/benchmarks/workload-model/validate.cjs build/sh2-model/workload-results.json
    ;;
  *) echo "ci.sh: unknown phase $1" >&2; exit 2 ;;
esac
