#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (c) 2026 SH2ZAM Contributors.
# Section garbage-collection probe (sections-1). Compiles the library as a
# consumer does (-ffunction-sections -fdata-sections), links one ELF per
# probe in probe.c with that probe as its only root and --gc-sections, once
# from the objects and once from an archive, and checks that exactly the
# expected hand-written entries and read-only tables survive. Writes the
# per-probe sizes to sh2-gc-probe.txt. See README.md.
# Phases and environment: saturn/benchmarks/ci/suites.sh and ci/lib.sh.
#
# GC_PROBE_REPORT_ONLY=1 skips the checks and only reports (to measure an
# older tree); GC_PROBE_ENTRIES and GC_PROBE_TABLES then name the hand-written
# entries and tables to look for, which an older tree cannot tell apart.
source "$(dirname "$0")/../ci/lib.sh"

# probe: the hand-written entries and the tables that must survive (sorted).
expect() {
  case "$1" in
    fog_gouraud_refs) echo "_sh2z_shade_fog_gouraud_refs|.rodata.sh2zi_offset_clamp" ;;
    fog_gouraud) echo "_sh2z_shade_fog_gouraud|.rodata.sh2zi_offset_clamp" ;;
    memo_put4) echo "_sh2z_memo_put4|" ;;
    memo_sync16w) echo "_sh2z_memo_sync16w|" ;;
    memo_diff8_runs) echo "_sh2z_memo_diff8_runs|" ;;
    view_batch_divu) echo "_sh2zi_view_divu_run|" ;;
    view_batch_trunc_divu) echo "_sh2zi_view_divu_run_trunc|" ;;
    view_batch_center_divu) echo "_sh2zi_view_divu_run_center|" ;;
    view_batch_divu_indexed) echo "_sh2zi_view_divu_indexed|" ;;
    view_merge_tier) echo "_sh2zi_merge_tier24|" ;;
    view_edge_rails_center_divu) echo "_sh2z_view_edge_rails_center_divu _sh2zi_view_divu_rails_center|" ;;
    view_from_camera30) echo "|.rodata.sh2zi_cot_q20" ;;
    shade_gouraud_approx) echo "_sh2zi_gouraud_run|" ;;
    shade_light_normals_indexed) echo "_sh2zi_light_run _sh2zi_light_run_sat|" ;;
    shade_screen_facing) echo "_sh2zi_facing_neg _sh2zi_facing_pos|" ;;
    vlc8_decode) echo "_sh2zi_vlc8_run|.rodata.sh2zi_vlc8_run" ;;
    vlc2_build) echo "|.rodata.sh2zi_vlc2_pow2" ;;
    vec3_normalize_u30) echo "_sh2z_vec3_normalize_u30|.rodata.sh2zi_u30_seed" ;;
    tan12_q16) echo "|.rodata.sh2z_sin12_quarter_u30" ;;
    pose_from_f32) echo "_sh2z_pose_from_f32 _sh2zi_pose_test_delta _sh2zi_pose_test_element _sh2zi_pose_test_inverse|.rodata.sh2zi_pose_g" ;;
    *) echo "gc-probe: no expectation for $1" >&2; exit 2 ;;
  esac
}

# Sum of the allocated output sections of an ELF, in bytes.
elf_bytes() {
  "$SH2Z_OBJDUMP" -h "$1" | awk '$2 ~ /^\.(text|rodata|data|bss)$/ {print $3}' |
    { t=0; while read -r h; do t=$((t + 0x$h)); done; echo "$t"; }
}

case "${1:?usage: ci.sh phases|build}" in
  phases) echo build ;;
  build)
    out=build/sh2/gc-probe
    rm -rf "$out"; mkdir -p "$out/obj"
    flags=(-m2 -mb -O2 -ffreestanding -std=c11 -Wall -Wextra -Werror -DSH2Z_REQUIRE_SH2=1
      -ffunction-sections -fdata-sections -Isaturn/include)
    if [ -z "${GC_PROBE_REPORT_ONLY:-}" ]; then
      # Every library source: nothing in a shared section.
      sh2z_require_per_symbol_sections saturn/src/*.c
    fi
    for src in saturn/src/*.c; do
      "$SH2Z_CC" "${flags[@]}" -c "$src" -o "$out/obj/$(basename "${src%.c}").o"
    done
    "${SH2Z_CC%gcc}ar" rcs "$out/libsh2zam-sections.a" "$out"/obj/*.o
    "$SH2Z_CC" "${flags[@]}" -c saturn/benchmarks/gc-probe/probe.c -o "$out/probe.o"
    # The hand-written entries are the global functions that the CI objects
    # (compiled without -ffunction-sections) hold outside plain .text; the
    # tables are their .rodata.<name> sections.
    entries=${GC_PROBE_ENTRIES:-$("$SH2Z_OBJDUMP" -t build/sh2/sh2zam*.o |
      awk '$2 == "g" && $3 == "F" && $4 ~ /^\.text\./ {print $6}' | LC_ALL=C sort -u | tr '\n' ' ')}
    tables=${GC_PROBE_TABLES:-$("$SH2Z_OBJDUMP" -h build/sh2/sh2zam*.o |
      awk '$2 ~ /^\.rodata\./ {print $2}' | LC_ALL=C sort -u | tr '\n' ' ')}
    [ -n "$entries" ] && [ -n "$tables" ] || { echo "gc-probe: no entries or tables found" >&2; exit 1; }
    report=sh2-gc-probe.txt
    {
      echo "gc-probe: one root per ELF, --gc-sections, library built with -ffunction-sections -fdata-sections"
      echo "bytes = .text + .rodata + .data + .bss of the linked ELF (libgcc helpers included), less the probe function"
      printf '%-30s %8s %8s  %s\n' probe objects archive "hand-written entries | tables linked"
    } > "$report"
    failed=0
    for probe in $(grep -oE '^void probe_[a-z0-9_]+\(void\) \{' saturn/benchmarks/gc-probe/probe.c | sed 's/^void probe_//; s/(void) {$//'); do
      want=$(expect "$probe")
      sizes=()
      got_first=
      for mode in objects archive; do
        if [ "$mode" = objects ]; then libs=("$out"/obj/*.o); else libs=("$out/libsh2zam-sections.a"); fi
        elf="$out/$probe-$mode.elf"
        "$SH2Z_CC" -m2 -mb -nostdlib -nostartfiles -Wl,-T,saturn/benchmarks/integer-model/model.ld \
          -Wl,-e,_probe_"$probe" -Wl,--gc-sections -Wl,-Map,"${elf%.elf}.map" \
          -o "$elf" "$out/probe.o" "${libs[@]}" -lgcc
        syms=$("$SH2Z_NM" "$elf" | awk '{print $3}' | LC_ALL=C sort -u)
        got_e=$(comm -12 <(echo "$syms") <(tr ' ' '\n' <<< "$entries" | LC_ALL=C sort -u) | tr '\n' ' ' | sed 's/ $//')
        kept=$(awk '/^Linker script and memory map/ {m = 1} m && $1 ~ /^\.rodata\./ {print $1}' "${elf%.elf}.map" | LC_ALL=C sort -u)
        got_t=$(comm -12 <(echo "$kept") <(tr ' ' '\n' <<< "$tables" | LC_ALL=C sort -u) | tr '\n' ' ' | sed 's/ $//')
        probe_size=$("$SH2Z_NM" -S "$elf" | awk -v s="_probe_$probe" '$4 == s {print $2}')
        sizes+=($(( $(elf_bytes "$elf") - 0x${probe_size:-0} )))
        got="$got_e|$got_t"
        if [ -z "$got_first" ]; then got_first=$got; fi
        if [ -z "${GC_PROBE_REPORT_ONLY:-}" ] && { [ "$got" != "$want" ] || [ "$got" != "$got_first" ]; }; then
          echo "gc-probe $probe ($mode): linked [$got], expected [$want]" >&2
          failed=1
        fi
      done
      printf '%-30s %8d %8d  %s\n' "$probe" "${sizes[0]}" "${sizes[1]}" "$got_first" >> "$report"
    done
    cat "$report"
    if [ "$failed" != 0 ]; then
      exit 1
    fi
    ;;
  *) echo "ci.sh: unknown phase $1" >&2; exit 2 ;;
esac
