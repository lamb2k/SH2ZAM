#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (c) 2026 SH2ZAM Contributors.
# Executes the vlc bridge ELF in the pinned Yabause SH-2 interpreter:
# verification against the independent oracle first (any mismatch, or any
# read past a stream, exits non-zero), then the cycle model.
set -euo pipefail
project_root=$(cd "$(dirname "$0")/../../.." && pwd)
model_build=${MODEL_BUILD_DIR:-"$project_root/build/sh2-model"}
if [ "$#" -lt 1 ]; then
  echo 'Usage: run.sh vlc-model.elf [results.json]' >&2
  exit 2
fi
model_elf=$(realpath "$1")
model_output=${2:-"$model_build/vlc-results.json"}
model_core=${YABAUSE_SOURCE:-"$model_build/yabause"}
model_commit=7e38821dbac265490f115e163c523a939acda759
verify_scale=${VLC_VERIFY:-2000}
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
  -DHAVE_STDINT_H -I"$model_core/yabause/src" -I"$project_root/saturn/include" -Wl,--gc-sections \
  "$project_root/saturn/benchmarks/sh2-model/vlc_harness.c" -lm -o "$model_build/vlc-harness"
"$model_build/vlc-harness" "$model_elf" "$verify_scale" > "$model_build/vlc-raw.json"
MODEL_ELF="$model_elf" MODEL_RAW="$model_build/vlc-raw.json" MODEL_OUT="$model_output" \
MODEL_EVIDENCE_URL="${MODEL_EVIDENCE_URL:-}" MODEL_TOOLCHAIN="${MODEL_TOOLCHAIN:-}" \
MODEL_SOURCE_COMMIT="${MODEL_SOURCE_COMMIT:-$(git -C "$project_root" rev-parse HEAD 2>/dev/null || echo unknown)}" \
python3 - <<'PY'
import hashlib, json, os
raw = json.load(open(os.environ["MODEL_RAW"]))
elf = open(os.environ["MODEL_ELF"], "rb").read()
raw["label"] = "MODELLED: executed SH-2 instructions in the Yabause interpreter; flat RAM, no caches or wait states; not Saturn hardware"
raw["sourceCommit"] = os.environ["MODEL_SOURCE_COMMIT"]
raw["elfSha256"] = hashlib.sha256(elf).hexdigest()
raw["toolchain"] = os.environ["MODEL_TOOLCHAIN"]
raw["evidenceUrl"] = os.environ["MODEL_EVIDENCE_URL"]
raw["yabauseCommit"] = "7e38821dbac265490f115e163c523a939acda759"
# Speed-ups of the SH-2 kernel and the library builder over the C controls.
for row in raw["rows"].values():
    d, b = row["decode"], row["build"]
    row["fastSpeedup"] = {k: round(d[k]["cyclesPerSymbol"] / d["fast"]["cyclesPerSymbol"], 2) for k in ("portable", "plain")}
    row["buildSpeedup"] = round(b["plain"]["meanCycles"] / b["fast"]["meanCycles"], 2)
json.dump(raw, open(os.environ["MODEL_OUT"], "w"), indent=1)
print("wrote", os.environ["MODEL_OUT"])
PY
