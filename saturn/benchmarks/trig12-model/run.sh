#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (c) 2026 SH2ZAM Contributors.
# Executes the trig12 bridge ELF in the pinned Yabause SH-2 interpreter:
# verification against the independent oracle first (any mismatch, or any
# division the library promised never to issue, exits non-zero), then the
# cycle model.
set -euo pipefail
project_root=$(cd "$(dirname "$0")/../../.." && pwd)
model_build=${MODEL_BUILD_DIR:-"$project_root/build/sh2-model"}
if [ "$#" -lt 1 ]; then
  echo 'Usage: run.sh trig12-model.elf [results.json]' >&2
  exit 2
fi
model_elf=$(realpath "$1")
model_output=${2:-"$model_build/trig12-results.json"}
model_core=${YABAUSE_SOURCE:-"$model_build/yabause"}
model_commit=7e38821dbac265490f115e163c523a939acda759
verify_scale=${TRIG12_VERIFY:-40000}
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
  "$project_root/saturn/benchmarks/sh2-model/trig12_harness.c" -lm -o "$model_build/trig12-harness"
"$model_build/trig12-harness" "$model_elf" "$verify_scale" > "$model_build/trig12-raw.json"
MODEL_ELF="$model_elf" MODEL_RAW="$model_build/trig12-raw.json" MODEL_OUT="$model_output" \
MODEL_EVIDENCE_URL="${MODEL_EVIDENCE_URL:-}" MODEL_TOOLCHAIN="${MODEL_TOOLCHAIN:-}" \
MODEL_SOURCE_COMMIT="${MODEL_SOURCE_COMMIT:-$(git -C "$project_root" rev-parse HEAD 2>/dev/null || echo unknown)}" \
python3 - <<'PY'
import hashlib, json, os
raw = json.load(open(os.environ["MODEL_RAW"]))
elf = open(os.environ["MODEL_ELF"], "rb").read()
raw["label"] = "MODELLED: executed SH-2 instructions in the Yabause interpreter; flat RAM, no caches or wait states; divider latency assumed; not Saturn hardware"
raw["sourceCommit"] = os.environ["MODEL_SOURCE_COMMIT"]
raw["elfSha256"] = hashlib.sha256(elf).hexdigest()
raw["toolchain"] = os.environ["MODEL_TOOLCHAIN"]
raw["evidenceUrl"] = os.environ["MODEL_EVIDENCE_URL"]
raw["yabauseCommit"] = "7e38821dbac265490f115e163c523a939acda759"
rows = raw["rows"]
# Net of the bare loop, and speed-ups over the controls.
for r in rows.values():
    base = rows["qloop" if r["entry"].startswith("c_q") or r["entry"] in
                ("c_recip30", "c_sh2z_div", "c_libgcc_q16", "c_float_div") else "loop"]
    r["netCyclesPerElement"] = round(r["cyclesPerElement"] - base["cyclesPerElement"], 2)
def ratio(a, b):
    return round(rows[a]["netCyclesPerElement"] / rows[b]["netCyclesPerElement"], 2)
raw["speedups"] = {
    "sincos12OverFloatToU30": ratio("floatToU30", "sincos12"),
    "sincos12OverSh2zSincos": ratio("sh2zSincos", "sincos12"),
    "tan12OverFloatTan": ratio("floatTan", "tan12"),
    "divuQ16OverSh2zDiv": ratio("sh2zDiv", "divuQ16"),
    "divuQ16OverLibgcc": ratio("libgccQ16", "divuQ16"),
    "divuQ16OverFloatDiv": ratio("floatDiv", "divuQ16"),
    "overlapSavedCyclesPerElement": round(rows["q16ThenSincos12"]["cyclesPerElement"] - rows["q16OverlapSincos12"]["cyclesPerElement"], 2),
}
json.dump(raw, open(os.environ["MODEL_OUT"], "w"), indent=1)
print("wrote", os.environ["MODEL_OUT"])
PY
