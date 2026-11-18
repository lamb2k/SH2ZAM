#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (c) 2026 SH2ZAM Contributors.
# Runs one phase of every model suite, for .github/workflows/sh2-cross.yml and
# for local runs. A suite is a directory saturn/benchmarks/<suite>/ holding a
# ci.sh; suites run in C-locale order of their directory names, so adding a
# suite needs no edit here or in the workflow (saturn/CONTRIBUTING-KERNELS.md).
#
# `ci.sh phases` prints the phases a suite takes part in, from:
#   build   inside the Yaul SH-2 toolchain, once build/sh2/libsh2zam.a exists:
#           compile and audit the suite's bridge and link its model ELF.
#   model   on the runner: run the suite in the SH-2 instruction model and write
#           build/sh2-model/<name>-results.json (and any *-raw.json, *.jsonl).
#   report  on the runner, after every suite's model phase: reports that read
#           several suites' outputs (the memory placement model).
# The first failing suite stops the phase with a non-zero exit, as a failing
# workflow step did.
#
# usage: suites.sh list | build | model | report
set -euo pipefail
cd "$(dirname "$0")/../../.."
phase=${1:-}
case "$phase" in
  list|build|model|report) ;;
  *) echo 'usage: suites.sh list|build|model|report' >&2; exit 2 ;;
esac
mapfile -t scripts < <(printf '%s\n' saturn/benchmarks/*/ci.sh | LC_ALL=C sort)
if [ ! -f "${scripts[0]}" ]; then
  echo 'suites.sh: no saturn/benchmarks/*/ci.sh found' >&2
  exit 1
fi
summary=${GITHUB_STEP_SUMMARY:-/dev/null}
if [ "$phase" = list ]; then
  printf '| Suite | Phases |\n| --- | --- |\n' >> "$summary"
else
  printf '| Suite | Phase | Result | Seconds |\n| --- | --- | --- | ---: |\n' >> "$summary"
fi
ran=()
for script in "${scripts[@]}"; do
  suite=$(basename "$(dirname "$script")")
  phases=$(bash "$script" phases)
  if [ "$phase" = list ]; then
    printf '%-20s %s\n' "$suite" "$phases"
    printf '| %s | %s |\n' "$suite" "$phases" >> "$summary"
    continue
  fi
  case " $phases " in
    *" $phase "*) ;;
    *) continue ;;
  esac
  echo "::group::$suite: $phase"
  start=$SECONDS
  if ! bash "$script" "$phase"; then
    echo "::endgroup::"
    echo "::error::model suite $suite failed its $phase phase ($script)"
    printf '| %s | %s | **failed** | %d |\n' "$suite" "$phase" $((SECONDS - start)) >> "$summary"
    exit 1
  fi
  echo "::endgroup::"
  printf '| %s | %s | passed | %d |\n' "$suite" "$phase" $((SECONDS - start)) >> "$summary"
  ran+=("$suite")
done
if [ "$phase" != list ]; then
  echo "suites.sh $phase: ${#ran[@]} suites passed: ${ran[*]-}"
fi
