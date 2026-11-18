#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (c) 2026 SH2ZAM Contributors.
# Vehicle kernels (vehicle-kernels-1) for the executed-instruction model. The library
# objects must not need memcpy/memset and must contain the hand-written SH-2 loops.
# Phases and environment: saturn/benchmarks/ci/suites.sh and ci/lib.sh.
source "$(dirname "$0")/../ci/lib.sh"
case "${1:?usage: ci.sh phases|build|model}" in
  phases) echo build model ;;
  build)
    sh2z_bridge saturn/benchmarks/vehicle-model/bridge.c build/sh2/vehicle-bridge.o
    "$SH2Z_OBJDUMP" -dr build/sh2/sh2zam_vehicle.o build/sh2/sh2zam_view_indexed.o > sh2-vehicle-disassembly.txt
    "$SH2Z_NM" -u build/sh2/sh2zam_vehicle.o build/sh2/sh2zam_view_indexed.o | tee sh2-vehicle-undefined.txt
    sh2z_require_no_mem_calls sh2-vehicle-undefined.txt
    grep -q "<_sh2zi_facing_pos>:" sh2-vehicle-disassembly.txt
    grep -q "<_sh2zi_light_run_sat>:" sh2-vehicle-disassembly.txt
    grep -q "<_sh2zi_gouraud_run>:" sh2-vehicle-disassembly.txt
    grep -q "<_sh2zi_view_divu_indexed>:" sh2-vehicle-disassembly.txt
    sh2z_model_elf build/sh2/vehicle-bridge.o build/sh2/vehicle-model.elf
    ;;
  model)
    sh2z_model_env
    bash saturn/benchmarks/vehicle-model/run.sh \
      build/sh2/vehicle-model.elf \
      build/sh2-model/vehicle-results.json
    ;;
  *) echo "ci.sh: unknown phase $1" >&2; exit 2 ;;
esac
