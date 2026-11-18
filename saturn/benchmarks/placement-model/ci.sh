#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (c) 2026 SH2ZAM Contributors.
# Memory placement costs for the view, shade, vlc and merge kernels, from the
# placement logs their model phases wrote. Runs after every model phase.
# Phases and environment: saturn/benchmarks/ci/suites.sh and ci/lib.sh.
source "$(dirname "$0")/../ci/lib.sh"
case "${1:?usage: ci.sh phases|report}" in
  phases) echo report ;;
  report)
    python3 saturn/benchmarks/placement-model/collect.py \
      --view-log build/sh2-model/view-placement.jsonl --view-results build/sh2-model/view-results.json \
      --shade-log build/sh2-model/shade-placement.jsonl --shade-results build/sh2-model/shade-results.json \
      --vlc-log build/sh2-model/vlc-placement.jsonl --vlc-results build/sh2-model/vlc-results.json \
      --merge-log build/sh2-model/merge-placement.jsonl --merge-results build/sh2-model/merge-results.json \
      --committed-root . \
      --evidence-url "${SH2Z_EVIDENCE_URL:-}" \
      --output build/sh2-model/placement-results.json \
      --markdown build/sh2-model/placement-tables.md
    cat build/sh2-model/placement-tables.md
    ;;
  *) echo "ci.sh: unknown phase $1" >&2; exit 2 ;;
esac
