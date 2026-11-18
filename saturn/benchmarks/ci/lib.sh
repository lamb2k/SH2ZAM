# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (c) 2026 SH2ZAM Contributors.
# Helpers for saturn/benchmarks/<suite>/ci.sh. Source it first; it moves to the
# repository root, because each object records its source path as given and the
# model ELFs must stay byte-identical. See saturn/CONTRIBUTING-KERNELS.md.
#
# build phase (inside the Yaul SH-2 toolchain, after build/sh2/libsh2zam.a and
# every library object in build/sh2/ exist):
#   SH2Z_CC, SH2Z_OBJDUMP, SH2Z_NM   the sh-elf tools; sh2-cross.yml passes them.
# model and report phases (on the runner, after the build phase):
#   SH2Z_EVIDENCE_URL, SH2Z_TOOLCHAIN, SH2Z_SOURCE_COMMIT   provenance; optional
#   locally (each suite's run.sh then falls back to its own defaults).
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../../.."

# Compile a model bridge with the flags every bridge uses.
#   sh2z_bridge <saturn/benchmarks/<suite>/bridge.c> <build/sh2/<name>-bridge.o>
sh2z_bridge() {
  "$SH2Z_CC" -m2 -mb -O2 -ffreestanding -std=c11 -Wall -Wextra -Werror -DSH2Z_REQUIRE_SH2=1 \
    -Isaturn/include -c "$1" -o "$2"
}

# Link a bridge with the library for the flat RAM map of the instruction model.
#   sh2z_model_elf <build/sh2/<name>-bridge.o> <build/sh2/<name>-model.elf>
sh2z_model_elf() {
  "$SH2Z_CC" -m2 -mb -nostdlib -nostartfiles -Wl,-T,saturn/benchmarks/integer-model/model.ld -Wl,-e,0 \
    -o "$2" "$1" build/sh2/libsh2zam.a -lgcc
}

# Fail unless an `nm -u` listing is empty (the object needs no symbol at all).
sh2z_require_no_undefined() {
  if [ -s "$1" ]; then
    echo "$1: the object needs external symbols" >&2
    exit 1
  fi
}

# Fail if an `nm -u` listing names memcpy, memset or memmove.
sh2z_require_no_mem_calls() {
  if grep -qE "memcpy|memset|memmove" "$1"; then
    echo "$1: the object calls memcpy, memset or memmove" >&2
    exit 1
  fi
}

# Export the provenance a bridge suite's run.sh records (MODEL_EVIDENCE_URL,
# MODEL_TOOLCHAIN, MODEL_SOURCE_COMMIT). An empty value means "use the default".
sh2z_model_env() {
  export MODEL_EVIDENCE_URL="${SH2Z_EVIDENCE_URL:-}"
  export MODEL_TOOLCHAIN="${SH2Z_TOOLCHAIN:-}"
  export MODEL_SOURCE_COMMIT="${SH2Z_SOURCE_COMMIT:-}"
}
