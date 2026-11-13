#!/usr/bin/env bash
set -euo pipefail
project_root=$(cd "$(dirname "$0")/../../.." && pwd)
out_dir=${OUT_DIR:-"$project_root/build/wasm-experiments"}
mkdir -p "$out_dir"
"${CLANG:-clang}" --target=wasm32 -O3 -msimd128 -std=c11 \
  -ffreestanding -fno-builtin -fno-fast-math -ffp-contract=off -nostdlib \
  -DSH2Z_FORCE_PORTABLE=1 -I"$project_root/saturn/include" \
  "$project_root/saturn/benchmarks/wasm-experiments/bridge.c" \
  "$project_root/saturn/src/sh2zam.c" \
  "$project_root/saturn/src/sh2zam_trig.c" \
  "$project_root/saturn/src/sh2zam_angles.c" \
  "$project_root/saturn/src/sh2zam_transform.c" \
  "$project_root/saturn/src/sh2zam_prepared.c" \
  "$project_root/saturn/src/sh2zam_geometry.c" \
  -Wl,--no-entry -Wl,--export-memory -Wl,--initial-memory=4194304 \
  -Wl,--max-memory=4194304 -Wl,-z,stack-size=65536 \
  -o "$out_dir/math.wasm"
printf '%s\n' "$out_dir/math.wasm"
