#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (c) 2026 SH2ZAM Contributors.
# Executes the shade bridge ELF in the pinned Yabause SH-2 interpreter:
# verification against the independent oracle first (any mismatch exits
# non-zero), then the cycle model.
set -euo pipefail
project_root=$(cd "$(dirname "$0")/../../.." && pwd)
model_build=${MODEL_BUILD_DIR:-"$project_root/build/sh2-model"}
if [ "$#" -lt 1 ]; then
  echo 'Usage: run.sh shade-model.elf [results.json]' >&2
  exit 2
fi
model_elf=$(realpath "$1")
model_output=${2:-"$model_build/shade-results.json"}
model_core=${YABAUSE_SOURCE:-"$model_build/yabause"}
model_commit=7e38821dbac265490f115e163c523a939acda759
verify_scale=${SHADE_VERIFY:-4000}
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
  "$project_root/saturn/benchmarks/sh2-model/shade_harness.c" \
  "$project_root/saturn/src/sh2zam.c" "$project_root/saturn/src/sh2zam_trig.c" \
  "$project_root/saturn/src/sh2zam_angles.c" "$project_root/saturn/src/sh2zam_transform.c" \
  "$project_root/saturn/src/sh2zam_geometry.c" -lm -o "$model_build/shade-harness"
"$model_build/shade-harness" "$model_elf" "$verify_scale" > "$model_build/shade-raw.json"
MODEL_ELF="$model_elf" MODEL_RAW="$model_build/shade-raw.json" MODEL_OUT="$model_output" \
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
# Per-element slopes between the 1-element row and the largest row of each kernel.
rows = raw["rows"]
def slope(small, big, entry, field="meanCycles"):
    a, b = rows[small], rows[big]
    n = next(v for k, v in b.items() if isinstance(v, int) and k != "corpora")
    return round((b[entry][field] - a[entry][field]) / (n - 1), 2)
per = {}
ins = {}
for small, big, entries in (("light_1", "light_256", ("fast_light", "fast_light_normals", "plain_light")),
                            ("gouraud_1", "gouraud_448", ("fast_gouraud", "plain_gouraud")),
                            ("cull_1", "cull_256", ("fast_cull", "plain_cull")),
                            ("fog_1", "fog_1024_uniform", ("fast_fog", "plain_fog")),
                            ("fade_1", "fade_1024", ("fast_offset", "plain_offset", "fast_mix", "plain_mix"))):
    per[big] = {e: slope(small, big, e) for e in entries}
    ins[big] = {e: slope(small, big, e, "meanInstructions") for e in entries}
raw["cyclesPerElement"] = per
raw["instructionsPerElement"] = ins
json.dump(raw, open(os.environ["MODEL_OUT"], "w"), indent=1)
print("wrote", os.environ["MODEL_OUT"])
PY
