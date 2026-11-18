#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (c) 2026 SH2ZAM Contributors.
# trig12 bridge (trig12-1) for the executed-instruction model. The table object
# needs nothing external; the tangent uses CPU division steps (div1), and the
# bridge instantiates the header-inline sincos and divider quotients.
# Phases and environment: saturn/benchmarks/ci/suites.sh and ci/lib.sh.
source "$(dirname "$0")/../ci/lib.sh"
case "${1:?usage: ci.sh phases|build|model}" in
  phases) echo build model ;;
  build)
    sh2z_bridge saturn/benchmarks/trig12-model/bridge.c build/sh2/trig12-bridge.o
    "$SH2Z_OBJDUMP" -dr build/sh2/sh2zam_trig12.o > sh2-trig12-disassembly.txt
    "$SH2Z_OBJDUMP" -dr build/sh2/trig12-bridge.o > sh2-trig12-bridge-disassembly.txt
    "$SH2Z_NM" -u build/sh2/sh2zam_trig12.o | tee sh2-trig12-undefined.txt
    sh2z_require_no_undefined sh2-trig12-undefined.txt
    grep -q "div1" sh2-trig12-disassembly.txt
    sh2z_model_elf build/sh2/trig12-bridge.o build/sh2/trig12-model.elf
    ;;
  model)
    sh2z_model_env
    bash saturn/benchmarks/trig12-model/run.sh \
      build/sh2/trig12-model.elf \
      build/sh2-model/trig12-results.json
    ;;
  *) echo "ci.sh: unknown phase $1" >&2; exit 2 ;;
esac
