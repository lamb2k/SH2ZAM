#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (c) 2026 SH2ZAM Contributors.
# View-rails bridge (view-rails-1) for the executed-instruction model. The
# library object must not call memcpy, memset or memmove, and must contain the
# hand-written SH-2 entry and its stepping loop.
# Phases and environment: saturn/benchmarks/ci/suites.sh and ci/lib.sh.
source "$(dirname "$0")/../ci/lib.sh"
case "${1:?usage: ci.sh phases|build|model}" in
  phases) echo build model ;;
  build)
    sh2z_bridge saturn/benchmarks/rails-model/bridge.c build/sh2/rails-bridge.o
    "$SH2Z_OBJDUMP" -dr build/sh2/sh2zam_view_rails.o > sh2-rails-disassembly.txt
    "$SH2Z_NM" -u build/sh2/sh2zam_view_rails.o | tee sh2-rails-undefined.txt
    sh2z_require_no_mem_calls sh2-rails-undefined.txt
    if grep -q "movmem" sh2-rails-undefined.txt; then
      echo "sh2-rails-undefined.txt: the object calls libgcc's block copy" >&2
      exit 1
    fi
    grep -q "<_sh2zi_view_divu_rails_center>:" sh2-rails-disassembly.txt
    grep -q "<_sh2z_view_edge_rails_center_divu>:" sh2-rails-disassembly.txt
    sh2z_model_elf build/sh2/rails-bridge.o build/sh2/rails-model.elf
    ;;
  model)
    sh2z_model_env
    bash saturn/benchmarks/rails-model/run.sh \
      build/sh2/rails-model.elf \
      build/sh2-model/rails-results.json
    ;;
  *) echo "ci.sh: unknown phase $1" >&2; exit 2 ;;
esac
