#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (c) 2026 SH2ZAM Contributors.
# Executes the memo bridge ELF in the pinned Yabause SH-2 interpreter:
# verification against the independent oracle first (any mismatch, any
# destination write that is not the oracle's, any destination read or any
# source read after an end marker exits non-zero), then the cycle model.
# With MODEL_PLACEMENT_LOG set, the placed cycles of every timed call are
# added from the placement model (model-placement-1).
set -euo pipefail
project_root=$(cd "$(dirname "$0")/../../.." && pwd)
model_build=${MODEL_BUILD_DIR:-"$project_root/build/sh2-model"}
if [ "$#" -lt 1 ]; then
  echo 'Usage: run.sh memo-model.elf [results.json]' >&2
  exit 2
fi
model_elf=$(realpath "$1")
model_output=${2:-"$model_build/memo-results.json"}
model_core=${YABAUSE_SOURCE:-"$model_build/yabause"}
model_commit=7e38821dbac265490f115e163c523a939acda759
verify_scale=${MEMO_VERIFY:-3000}
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
  "$project_root/saturn/benchmarks/sh2-model/memo_harness.c" -lm -o "$model_build/memo-harness"
"$model_build/memo-harness" "$model_elf" "$verify_scale" > "$model_build/memo-raw.json"
MODEL_ELF="$model_elf" MODEL_RAW="$model_build/memo-raw.json" MODEL_OUT="$model_output" \
MODEL_EVIDENCE_URL="${MODEL_EVIDENCE_URL:-}" MODEL_TOOLCHAIN="${MODEL_TOOLCHAIN:-}" \
MODEL_PLACEMENT_LOG="${MODEL_PLACEMENT_LOG:-}" \
MODEL_SOURCE_COMMIT="${MODEL_SOURCE_COMMIT:-$(git -C "$project_root" rev-parse HEAD 2>/dev/null || echo unknown)}" \
python3 - <<'PY'
import hashlib, json, os
raw = json.load(open(os.environ["MODEL_RAW"]))
elf = open(os.environ["MODEL_ELF"], "rb").read()
raw["label"] = "MODELLED: executed SH-2 instructions in the Yabause interpreter; CPU-only cycles are flat RAM with no caches or wait states; placed cycles add the model-placement-1 cache and bus model; not Saturn hardware"
raw["sourceCommit"] = os.environ["MODEL_SOURCE_COMMIT"]
raw["elfSha256"] = hashlib.sha256(elf).hexdigest()
raw["toolchain"] = os.environ["MODEL_TOOLCHAIN"]
raw["evidenceUrl"] = os.environ["MODEL_EVIDENCE_URL"]
raw["yabauseCommit"] = "7e38821dbac265490f115e163c523a939acda759"
raw["targets"] = {
    "note": "Figures for today's code as quoted by the contributing port, from its own cost model (CPU plus memory); not measured here",
    "A": "about 107 cycles per 16-word group, steady state; 75 to 90 groups a picture",
    "B": "357 to 388 commands a picture; the memo saves 42 to 80 cycles a command; 64,000 to about 47,000-50,000 cycles a picture",
    "C": "6,467 cycles for about 209 records with the early stop; 17,940 for the full 1,024-record copy",
}

# Placement: the last logged call of each tag (the second of two identical
# calls, so "warm" is the kernel called in a loop).
log = os.environ["MODEL_PLACEMENT_LOG"]
placements = None
last = {}
if log:
    for line in open(log):
        rec = json.loads(line)
        if "placementModel" in rec:
            placements = [p["name"] for p in rec["placements"]]
            raw["placementModel"] = {"regions": rec["regions"], "placements": rec["placements"], "selfTest": rec["selfTest"]}
        elif "tag" in rec:
            last[rec["tag"]] = rec
    if placements is None:
        raise SystemExit("placement log has no header")

def slope(a, b, ka, kb):
    return round((b - a) / (kb - ka), 2)

for name, row in raw["rows"].items():
    pts = row["points"]
    ka, kb = (str(k) for k in row["slope"])
    na, nb = row["slope"]
    row["cyclesPerUnit"] = slope(pts[ka]["cycles"], pts[kb]["cycles"], na, nb)
    if not last:
        continue
    placed = {}
    for i, p in enumerate(placements):
        placed[p] = {}
        for m, mode in enumerate(("warm", "cold")):
            vals = {}
            for k, pt in pts.items():
                rec = last[pt["tag"]]
                if rec["cycles"] != pt["cycles"]:
                    raise SystemExit(f"{pt['tag']}: placement log cycles {rec['cycles']} != capture {pt['cycles']}")
                vals[k] = pt["cycles"] + rec["placed"][i][m][0]
            placed[p][mode] = {"call": vals, "perUnit": slope(vals[ka], vals[kb], na, nb)}
    row["placed"] = placed

json.dump(raw, open(os.environ["MODEL_OUT"], "w"), indent=1)
print("wrote", os.environ["MODEL_OUT"])
PY
