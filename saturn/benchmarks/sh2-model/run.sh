#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (c) 2026 SH2ZAM Contributors.
set -euo pipefail
project_root=$(cd "$(dirname "$0")/../../.." && pwd)
model_dir="$project_root/saturn/benchmarks/sh2-model"
model_build=${MODEL_BUILD_DIR:-"$project_root/build/sh2-model"}
if [ "$#" -lt 2 ]; then
  echo 'Usage: run.sh benchmark.elf benchmark.sym [results.json]' >&2
  exit 2
fi
model_elf=$(realpath "$1")
model_symbols=$(realpath "$2")
model_output=${3:-"$model_build/results.json"}
model_core=${YABAUSE_SOURCE:-"$model_build/yabause"}
model_commit=7e38821dbac265490f115e163c523a939acda759
mkdir -p "$model_build"
if [ ! -d "$model_core/.git" ]; then
  git init "$model_core"
  git -C "$model_core" remote add origin https://github.com/Yabause/yabause.git
  git -C "$model_core" sparse-checkout init --cone
  git -C "$model_core" sparse-checkout set yabause/src
  git -C "$model_core" fetch --depth 1 origin "$model_commit"
  git -C "$model_core" checkout --detach FETCH_HEAD
fi
if [ "$(git -C "$model_core" rev-parse HEAD)" != "$model_commit" ]; then
  echo 'YABAUSE_SOURCE must be the pinned upstream commit; source was not changed.' >&2
  exit 2
fi
git -C "$model_core" diff --exit-code HEAD -- yabause/src
"${CC:-cc}" -O2 -fwrapv -fno-strict-aliasing -ffunction-sections -fdata-sections \
  -DHAVE_STDINT_H -I"$model_core/yabause/src" -Wl,--gc-sections \
  "$model_dir/harness.c" -lm -o "$model_build/harness"
"$model_build/harness" "$model_elf" "$model_symbols" 15 21 > "$model_build/raw.json"
python3 "$model_dir/collect.py" --raw "$model_build/raw.json" \
  --elf "$model_elf" --symbols "$model_symbols" --core "$model_core" \
  --manifest "$project_root/saturn/benchmarks/manifest.json" --output "$model_output" \
  --evidence-url "${MODEL_EVIDENCE_URL:-https://github.com/lamb2k/SH2ZAM/actions/workflows/sh2-cross.yml}"
