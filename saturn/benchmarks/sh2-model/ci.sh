#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (c) 2026 SH2ZAM Contributors.
# Compiled SH-2 kernels (the five-path vertex suite) executed in the isolated
# instruction model, from the Yaul benchmark ELF (saturn/benchmarks/yaul).
# Phases and environment: saturn/benchmarks/ci/suites.sh and ci/lib.sh.
source "$(dirname "$0")/../ci/lib.sh"
case "${1:?usage: ci.sh phases|model}" in
  phases) echo model ;;
  model)
    export MODEL_EVIDENCE_URL="${SH2Z_EVIDENCE_URL:-}"
    bash saturn/benchmarks/sh2-model/run.sh \
      saturn/benchmarks/yaul/build/sh2zam-benchmark.elf \
      saturn/benchmarks/yaul/build/sh2zam-benchmark.sym \
      build/sh2-model/results.json
    node saturn/benchmarks/compile-model-results.cjs build/sh2-model/results.json
    ;;
  *) echo "ci.sh: unknown phase $1" >&2; exit 2 ;;
esac
