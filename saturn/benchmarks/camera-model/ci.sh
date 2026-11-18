#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (c) 2026 SH2ZAM Contributors.
# View-from-camera bridge (view-camera-1) for the executed-instruction model. The
# library object must need nothing from libgcc or libc (no 64-bit shift helpers).
# Phases and environment: saturn/benchmarks/ci/suites.sh and ci/lib.sh.
source "$(dirname "$0")/../ci/lib.sh"
case "${1:?usage: ci.sh phases|build|model}" in
  phases) echo build model ;;
  build)
    sh2z_bridge saturn/benchmarks/camera-model/bridge.c build/sh2/camera-bridge.o
    "$SH2Z_OBJDUMP" -dr build/sh2/sh2zam_view_camera.o > sh2-camera-disassembly.txt
    "$SH2Z_NM" -u build/sh2/sh2zam_view_camera.o | tee sh2-camera-undefined.txt
    sh2z_require_no_undefined sh2-camera-undefined.txt
    grep -q "dmuls.l" sh2-camera-disassembly.txt
    sh2z_model_elf build/sh2/camera-bridge.o build/sh2/camera-model.elf
    ;;
  model)
    sh2z_model_env
    bash saturn/benchmarks/camera-model/run.sh \
      build/sh2/camera-model.elf \
      build/sh2-model/camera-results.json
    ;;
  *) echo "ci.sh: unknown phase $1" >&2; exit 2 ;;
esac
