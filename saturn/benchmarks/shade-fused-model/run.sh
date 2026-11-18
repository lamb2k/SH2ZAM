#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (c) 2026 SH2ZAM Contributors.
# Executes the shade-2 bridge ELF in the pinned Yabause SH-2 interpreter:
# verification against the independent oracle first (any mismatch exits
# non-zero), then the cycle model.
set -euo pipefail
project_root=$(cd "$(dirname "$0")/../../.." && pwd)
model_build=${MODEL_BUILD_DIR:-"$project_root/build/sh2-model"}
if [ "$#" -lt 1 ]; then
  echo 'Usage: run.sh shade-fused-model.elf [results.json]' >&2
  exit 2
fi
model_elf=$(realpath "$1")
model_output=${2:-"$model_build/shade-fused-results.json"}
model_core=${YABAUSE_SOURCE:-"$model_build/yabause"}
model_commit=7e38821dbac265490f115e163c523a939acda759
verify_scale=${SHADE_FUSED_VERIFY:-3000}
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
  "$project_root/saturn/benchmarks/sh2-model/shade_fused_harness.c" \
  "$project_root/saturn/src/sh2zam.c" "$project_root/saturn/src/sh2zam_trig.c" \
  "$project_root/saturn/src/sh2zam_angles.c" "$project_root/saturn/src/sh2zam_transform.c" \
  "$project_root/saturn/src/sh2zam_geometry.c" -lm -o "$model_build/shade-fused-harness"
"$model_build/shade-fused-harness" "$model_elf" "$verify_scale" > "$model_build/shade-fused-raw.json"
MODEL_ELF="$model_elf" MODEL_RAW="$model_build/shade-fused-raw.json" MODEL_OUT="$model_output" \
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
def mean(row, entry):
    return rows[row][entry]["meanCycles"]
def verts(row):
    return rows[row]["meanVertices"]
# Whole-call cycles per vertex at the caller's batch sizes (per-call cost
# included), and per-element slopes between a small and a large row (per-call
# cost removed). Piece rows include the caller's loop around the call.
per_vertex = {}
for small, big, entries in (("textured_pieces_1", "textured_pieces_64", ("fast_pieces", "fast_refs_pieces", "shade1_pieces", "plain_pieces")),
                            ("textured_pieces_1_near_dense", "textured_pieces_64_near_dense", ("fast_pieces", "fast_refs_pieces", "shade1_pieces", "plain_pieces")),
                            ("solid_pieces_1", "solid_pieces_64", ("fast_pieces", "fast_refs_pieces", "shade1_pieces", "plain_pieces"))):
    per_vertex[big] = {e: {"wholeRowPerVertex": round(mean(big, e) / verts(big), 2),
                           "slopePerVertex": round((mean(big, e) - mean(small, e)) / (verts(big) - verts(small)), 2)} for e in entries}
per_vertex["mixed_pieces_64"] = {e: {"wholeRowPerVertex": round(mean("mixed_pieces_64", e) / verts("mixed_pieces_64"), 2)}
                                 for e in ("fast_pieces", "fast_refs_pieces", "shade1_pieces", "plain_pieces")}
for r in ("batch_4", "batch_64", "batch_256", "batch_1024", "batch_256_near_dense"):
    per_vertex[r] = {e: {"wholeCallPerVertex": round(mean(r, e) / verts(r), 2)} for e in ("fast_batch", "plain_batch")}
per_vertex["batch_slope_4_to_1024"] = {e: round((mean("batch_1024", e) - mean("batch_4", e)) / 1020, 2) for e in ("fast_batch", "plain_batch")}
cdiv = {}
copy = (mean("cdiv_256", "cdiv_copy") - mean("cdiv_1", "cdiv_copy")) / 255
for e in ("cdiv255_u", "cdiv255_s", "cdiv65025_u"):
    cdiv[e] = round((mean("cdiv_256", e) - mean("cdiv_1", e)) / 255 - copy, 2)
for e in ("shiftadd_div255_s", "c_div255_s", "c_div65025"):
    cdiv[e] = round((mean("cdiv_plain_256", e) - mean("cdiv_plain_1", e)) / 255 - copy, 2)
raw["cyclesPerVertex"] = per_vertex
raw["constantDivideCyclesPerValue"] = {"note": "slope over 255 values minus the slope of a plain copy loop", **cdiv}
json.dump(raw, open(os.environ["MODEL_OUT"], "w"), indent=1)
print("wrote", os.environ["MODEL_OUT"])
PY
