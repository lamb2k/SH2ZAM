#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (c) 2026 SH2ZAM Contributors.
# Executes the vlc-2 bridge ELF in the pinned Yabause SH-2 interpreter:
# verification against the independent oracle first (any mismatch, or any
# read past a stream bound or a table, exits non-zero), then the cycle model.
# With MODEL_PLACEMENT_LOG set, the placement model's stalls for every tile
# decode are summarised into the results (placed cycles per tile).
set -euo pipefail
project_root=$(cd "$(dirname "$0")/../../.." && pwd)
model_build=${MODEL_BUILD_DIR:-"$project_root/build/sh2-model"}
if [ "$#" -lt 1 ]; then
  echo 'Usage: run.sh vlc2-model.elf [results.json]' >&2
  exit 2
fi
model_elf=$(realpath "$1")
model_output=${2:-"$model_build/vlc2-results.json"}
model_core=${YABAUSE_SOURCE:-"$model_build/yabause"}
model_commit=7e38821dbac265490f115e163c523a939acda759
verify_scale=${VLC2_VERIFY:-1500}
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
  "$project_root/saturn/benchmarks/sh2-model/vlc2_harness.c" -lm -o "$model_build/vlc2-harness"
"$model_build/vlc2-harness" "$model_elf" "$verify_scale" > "$model_build/vlc2-raw.json"
MODEL_ELF="$model_elf" MODEL_RAW="$model_build/vlc2-raw.json" MODEL_OUT="$model_output" \
MODEL_PLACEMENT_LOG="${MODEL_PLACEMENT_LOG:-}" \
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
# The contributor's figures for its own decoder on the same tile shape (CPU
# only; tables in high work RAM; tables in the cartridge), for comparison.
raw["targetCyclesPerTile"] = {"cpuOnly": 3396, "tablesHighWram": 4960, "tablesCartridge": 9590}
for mix in raw["tiles"].values():
    v = mix["variants"]
    for name, row in v.items():
        if name != "glue":
            row["speedupOverVlc1"] = round(v["vlc1"]["perTile"]["meanCycles"] / row["perTile"]["meanCycles"], 2)
    # The decoder's share: a variant's cycles minus the caller's own (glue).
    glue = v["glue"]["perTile"]["meanCycles"]
    mix["decodeCyclesPerTile"] = {k: round(v[k]["perTile"]["meanCycles"] - glue, 1) for k in v if k != "glue"}
    mix["decodeCyclesPerSymbol"] = {k: round((v[k]["perTile"]["meanCycles"] - glue) / mix["symbolsPerTile"], 2) for k in v if k != "glue"}
log = os.environ.get("MODEL_PLACEMENT_LOG")
if log and os.path.exists(log):
    lines = [json.loads(l) for l in open(log) if l.strip()]
    head = lines[0]
    assert head["placementModel"] == 1 and head["selfTest"] == "passed"
    names = [p["name"] for p in head["placements"]]
    regions = {p["name"]: {c: p[c] for c in head["classes"]} for p in head["placements"]}
    calls = {}
    for l in lines:
        if "tag" in l and l["tag"].startswith("vlc2."):
            calls.setdefault(l["tag"], []).append(l)
    raw["placement"] = {"placements": regions, "note": "placed cycles = CPU-only cycles + stalls of the SH7604 cache and region model (placement_model.h); warm: the cache carries over from the previous tile of the same run, cold: empty cache per tile; region costs from hardware notes, not measured by this project"}
    for mixname, mix in raw["tiles"].items():
        for vname, row in mix["variants"].items():
            c = calls["vlc2.%s.%s" % (mixname, vname)]
            cpu = sum(x["cycles"] for x in c) / len(c)
            assert abs(cpu - row["perTile"]["meanCycles"]) < 0.01, "placement log and capture disagree"
            row["placedPerTile"] = {}
            for i, p in enumerate(names):
                row["placedPerTile"][p] = {mode: round(sum(x["cycles"] + x["placed"][i][m][0] for x in c) / len(c), 1)
                                          for m, mode in enumerate(("warm", "cold"))}
json.dump(raw, open(os.environ["MODEL_OUT"], "w"), indent=1)
print("wrote", os.environ["MODEL_OUT"])
PY
