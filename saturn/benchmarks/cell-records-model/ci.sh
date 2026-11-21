#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (c) 2026 SH2ZAM Contributors.
# cell-records bridge (cell-records-1) for the executed-instruction model. The
# library object must need nothing undefined (no memcpy, no libgcc helper).
# Phases and environment: saturn/benchmarks/ci/suites.sh and ci/lib.sh.
source "$(dirname "$0")/../ci/lib.sh"
case "${1:?usage: ci.sh phases|build|model}" in
  phases) echo build model ;;
  build)
    sh2z_bridge saturn/benchmarks/cell-records-model/bridge.c build/sh2/cell-records-bridge.o
    "$SH2Z_OBJDUMP" -dr build/sh2/sh2zam_cell_records.o > sh2-cell-records-disassembly.txt
    "$SH2Z_NM" -u build/sh2/sh2zam_cell_records.o | tee sh2-cell-records-undefined.txt
    sh2z_require_no_undefined sh2-cell-records-undefined.txt
    grep -q "<_sh2z_cellrec_expand_trusted>:" sh2-cell-records-disassembly.txt
    grep -q "<_sh2z_cellrec_expand>:" sh2-cell-records-disassembly.txt
    sh2z_model_elf build/sh2/cell-records-bridge.o build/sh2/cell-records-model.elf
    ;;
  model)
    sh2z_model_env
    bash saturn/benchmarks/cell-records-model/run.sh \
      build/sh2/cell-records-model.elf \
      build/sh2-model/cell-records-results.json
    ;;
  *) echo "ci.sh: unknown phase $1" >&2; exit 2 ;;
esac
