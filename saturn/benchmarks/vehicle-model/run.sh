#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (c) 2026 SH2ZAM Contributors.
# Executes the vehicle-kernels-1 bridge ELF in the pinned Yabause SH-2
# interpreter: verification against the independent oracles first (any
# mismatch exits non-zero), then the cycle model.
set -euo pipefail
project_root=$(cd "$(dirname "$0")/../../.." && pwd)
model_build=${MODEL_BUILD_DIR:-"$project_root/build/sh2-model"}
if [ "$#" -lt 1 ]; then
  echo 'Usage: run.sh vehicle-model.elf [results.json]' >&2
  exit 2
fi
model_elf=$(realpath "$1")
model_output=${2:-"$model_build/vehicle-results.json"}
model_core=${YABAUSE_SOURCE:-"$model_build/yabause"}
model_commit=7e38821dbac265490f115e163c523a939acda759
verify_scale=${VEHICLE_VERIFY:-2000}
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
src="$project_root/saturn/src"
"${CC:-cc}" -O2 -fwrapv -fno-strict-aliasing -ffunction-sections -fdata-sections \
  -DHAVE_STDINT_H -I"$model_core/yabause/src" -I"$project_root/saturn/include" -Wl,--gc-sections \
  "$project_root/saturn/benchmarks/sh2-model/vehicle_harness.c" \
  "$src/sh2zam.c" "$src/sh2zam_trig.c" "$src/sh2zam_angles.c" "$src/sh2zam_transform.c" \
  "$src/sh2zam_geometry.c" "$src/sh2zam_view.c" "$src/sh2zam_divu.c" -lm -o "$model_build/vehicle-harness"
"$model_build/vehicle-harness" "$model_elf" "$verify_scale" > "$model_build/vehicle-raw.json"
MODEL_ELF="$model_elf" MODEL_RAW="$model_build/vehicle-raw.json" MODEL_OUT="$model_output" \
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
rows = raw["rows"]
def per(small, big, entry, field="meanCycles"):
    a, b = rows[small], rows[big]
    n = next(v for k, v in b.items() if isinstance(v, int) and k != "corpora")
    return round((b[entry][field] - a[entry][field]) / (n - 1), 2)
# New kernels: slope between the 1-element row and the caller-sized row, which
# removes the per-call cost. The whole-mesh kernels the caller runs today
# ("port_") do the same work at every row: their whole-call cost is quoted.
raw["cyclesPerElement"] = {
    "fast_view_indexed": per("view_indexed_1", "view_indexed_68", "fast_view_indexed"),
    "fast_facing": per("facing_1", "facing_97", "fast_facing"),
    "port_facing": per("facing_1", "facing_97", "port_facing"),
    "fast_light_indexed": per("light_indexed_1", "light_indexed_133", "fast_light_indexed"),
    "fast_gouraud_approx": per("gouraud_1", "gouraud_448", "fast_gouraud_approx"),
    "port_gouraud": per("gouraud_1", "gouraud_448", "port_gouraud"),
}
raw["instructionsPerElement"] = {
    "fast_view_indexed": per("view_indexed_1", "view_indexed_68", "fast_view_indexed", "meanInstructions"),
    "fast_facing": per("facing_1", "facing_97", "fast_facing", "meanInstructions"),
    "fast_light_indexed": per("light_indexed_1", "light_indexed_133", "fast_light_indexed", "meanInstructions"),
    "fast_gouraud_approx": per("gouraud_1", "gouraud_448", "fast_gouraud_approx", "meanInstructions"),
}
json.dump(raw, open(os.environ["MODEL_OUT"], "w"), indent=1)
print("wrote", os.environ["MODEL_OUT"])
PY
