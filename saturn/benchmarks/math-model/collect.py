#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (c) 2026 SH2ZAM Contributors.
"""Package exact old/new native SH-2 model evidence, including regressions."""
import argparse
import hashlib
import json
import math
import re
import shlex
import statistics
import subprocess
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
KERNEL_SET = "previous-optimized-math-sh2-model-v1"
RAW_KIND = "SH2ZAM previous-versus-optimized math, Yabause isolated SH2 flat-RAM model"
BASELINE_COMMIT = "10786b64f4593d647d06d2a4a73e752c0c413dd0"
CORE_COMMIT = "7e38821dbac265490f115e163c523a939acda759"
PROFILES = ["bounded", "tiny", "full-range"]
BASELINE_FILES = ["sh2zam.c", "sh2zam_angles.c", "sh2zam_trig.c",
                  "sh2zam_transform.c", "sh2zam_geometry.c", "sh2zam_internal.h"]
COST_KEYS = ["cycles", "instructions", "dataReads", "dataWrites",
             "macInstructions", "dmulsInstructions"]
FLAGS = ["-m2", "-mb", "-O2", "-fno-fast-math", "-ffp-contract=off", "-fno-lto"]
LIMITATIONS = [
    "Not physical Saturn timing or full-system emulator timing",
    "No cache or bus wait-state model",
    "No pipeline or multiplier-contention model",
    "No DMA, interrupts, timers or peripheral integration",
    "One SH-2 CPU; no second-CPU scheduling or shared-memory contention",
    "No conversion to milliseconds, frame rate or clock-frequency estimates",
]


def require(condition, message):
    if not condition:
        raise ValueError(message)


def digest(data):
    return hashlib.sha256(data).hexdigest()


def file_hash(path):
    return digest(path.read_bytes())


def exact_keys(value, keys, label):
    require(isinstance(value, dict) and set(value) == set(keys),
            f"Unexpected/missing {label} fields")


def integer(value, minimum, label):
    require(type(value) is int and minimum <= value <= 9007199254740991,
            f"Invalid {label}: expected a safe integer >= {minimum}")


def checked_object(pairs):
    result = {}
    for key, value in pairs:
        require(key not in result, f"Duplicate JSON key: {key}")
        result[key] = value
    return result


def load_json(path):
    def nonfinite(value):
        raise ValueError(f"Nonfinite JSON number: {value}")
    return json.loads(path.read_text(), object_pairs_hook=checked_object,
                      parse_constant=nonfinite)


def family(operation):
    if operation.startswith("vec"):
        return "vector"
    if operation.startswith("quat"):
        return "quaternion"
    if operation.startswith("mat"):
        return "matrix"
    if operation in {"sin", "cos", "sincos", "tan", "atan2", "atan", "asin", "acos"}:
        return "trig"
    if operation == "project_clip" or operation.startswith("sphere_"):
        return "geometry"
    return "scalar"


def cases():
    found = [(name, int(words)) for name, words in re.findall(
        r'\{"([a-z0-9_]+)",(\d+)u\}', (HERE / "cases.inc").read_text())]
    require(len(found) == 48 and len(dict(found)) == 48, "Expected 48 unique operations")
    return found


def validate_raw(raw, expected):
    exact_keys(raw, ["schema", "kind", "sourceCommit", "compiler", "compileFlags",
                    "batchSize", "samplesPerProfile", "records", "checkedOutputWords",
                    "mismatches"], "raw capture")
    require(type(raw["schema"]) is int and raw["schema"] == 1 and raw["kind"] == RAW_KIND,
            "Wrong native math capture schema")
    require(type(raw["batchSize"]) is int and raw["batchSize"] == 32 and
            type(raw["samplesPerProfile"]) is int and raw["samplesPerProfile"] == 7,
            "Expected complete 32-call batches and seven samples per profile")
    require(isinstance(raw["records"], list) and len(raw["records"]) == 144,
            "All 48 operations and three profiles are required")
    integer(raw["mismatches"], 0, "mismatches")
    require(raw["mismatches"] == 0, "Native outputs disagree")
    seen, records = set(), []
    widths = dict(expected)
    for record in raw["records"]:
        exact_keys(record, ["operation", "profile", "outputWords", "samples",
                            "srSRetained", "srSOutputMatch"], "raw record")
        operation, profile = record["operation"], record["profile"]
        require(isinstance(operation, str) and operation in widths and profile in PROFILES,
                "Unknown operation/profile")
        key = (operation, profile)
        require(key not in seen, f"Duplicate workload: {key}")
        seen.add(key)
        integer(record["outputWords"], 1, "outputWords")
        require(record["outputWords"] == widths[operation], f"Wrong output width: {key}")
        require(record["srSRetained"] is True and record["srSOutputMatch"] is True,
                f"SR.S guard did not pass: {key}")
        samples = record["samples"]
        require(isinstance(samples, list) and len(samples) == 7, f"Incomplete samples: {key}")
        for sample in samples:
            exact_keys(sample, ["inputChecksum", "before", "after", "outputChecksum", "mismatches"],
                       "sample")
            for name in ["inputChecksum", "outputChecksum"]:
                require(isinstance(sample[name], str) and re.fullmatch(r"[a-f0-9]{8}", sample[name]),
                        f"Invalid {name}: {key}")
            integer(sample["mismatches"], 0, "sample mismatches")
            require(sample["mismatches"] == 0, f"Native sample differs: {key}")
            for path in ["before", "after"]:
                exact_keys(sample[path], COST_KEYS, f"{path} cost")
                for metric in COST_KEYS:
                    integer(sample[path][metric], 1 if metric in ["cycles", "instructions"] else 0,
                            f"{key} {path} {metric}")
                require(sample[path]["macInstructions"] + sample[path]["dmulsInstructions"] <=
                        sample[path]["instructions"], f"Impossible instruction counters: {key}")
        medians = {path: statistics.median(s[path]["cycles"] for s in samples)
                   for path in ["before", "after"]}
        records.append(dict(record, family=family(operation), sampleCount=7,
                            medianCycles=medians,
                            cyclesPerCall={path: value / 32 for path, value in medians.items()},
                            speedup=medians["before"] / medians["after"],
                            cyclesSavedPercent=(1 - medians["after"] / medians["before"]) * 100,
                            checkedOutputWords=32 * 7 * widths[operation]))
    require(seen == {(name, profile) for name in widths for profile in PROFILES},
            "Missing operation/profile coverage")
    expected_words = sum(widths.values()) * 32 * 7 * 3
    integer(raw["checkedOutputWords"], 1, "checkedOutputWords")
    require(raw["checkedOutputWords"] == expected_words, "Incomplete native exactness checks")
    order = {name: i for i, (name, _) in enumerate(expected)}
    records.sort(key=lambda r: (order[r["operation"]], PROFILES.index(r["profile"])))
    return records


def collect(args):
    raw = load_json(args.raw)
    records = validate_raw(raw, cases())
    elf, symbols = args.elf.read_bytes(), args.symbols.read_bytes()
    require(len(elf) >= 52 and elf[:6] == b"\x7fELF\x01\x02" and
            int.from_bytes(elf[18:20], "big") == 42, "Expected ELF32 big-endian SuperH binary")
    head = subprocess.check_output(["git", "-C", str(ROOT), "rev-parse", "HEAD"], text=True).strip()
    source_commit = args.source_commit or raw["sourceCommit"]
    require(isinstance(source_commit, str) and re.fullmatch(r"[a-f0-9]{40}", source_commit) and
            source_commit == raw["sourceCommit"] == head, "ELF source commit must match current git HEAD")
    for key in ["sourceCommit", "compiler", "compileFlags"]:
        value = raw[key]
        require(isinstance(value, str) and value and "\x00" not in value and
                value.encode() + b"\x00" in elf, f"{key} not verified inside the ELF")
    flags = shlex.split(raw["compileFlags"])
    require(all(flag in flags for flag in FLAGS) and "-ffast-math" not in flags and
            "-flto" not in flags, "Unexpected compiler configuration")
    for name in ["math_source_commit", "math_compiler", "math_compile_flags", "math_prepare",
                 "math_before_range", "math_after_range", "math_cases", "math_case_count"]:
        require(re.search(rb"(?m)^\s*[0-9a-fA-F]+\s+\S\s+_?" + name.encode() + rb"\s*$", symbols),
                f"Missing retained ELF symbol: {name}")
    require(re.fullmatch(r"https://github\.com/[A-Za-z0-9_.-]+/[A-Za-z0-9_.-]+/actions/(?:runs/[0-9]+|workflows/[^\s?#]+)",
                         args.evidence_url), "Expected public GitHub Actions evidence URL")
    core_commit = subprocess.check_output(["git", "-C", str(args.core), "rev-parse", "HEAD"], text=True).strip()
    require(core_commit == CORE_COMMIT, "Unexpected interpreter version")
    subprocess.run(["git", "-C", str(args.core), "diff", "--exit-code", "HEAD", "--", "yabause/src"], check=True)
    manifest = load_json(args.manifest)
    exact_keys(manifest, ["sourceCommit", "files"], "frozen baseline manifest")
    require(manifest["sourceCommit"] == BASELINE_COMMIT, "Wrong frozen baseline commit")
    exact_keys(manifest["files"], BASELINE_FILES, "frozen baseline source")
    require(args.manifest.read_bytes() == (HERE / "legacy/manifest.json").read_bytes(),
            "Manifest differs from the checked-in frozen baseline")
    for name in BASELINE_FILES:
        require(file_hash(HERE / "legacy" / name) == manifest["files"][name],
                f"Frozen baseline bytes were changed: {name}")
    library_hashes = {name: file_hash(ROOT / "saturn/src" / name) for name in BASELINE_FILES}
    wrapper_names = ["legacy_" + name for name in BASELINE_FILES if name.endswith(".c")]
    wrapper_hashes = {name: file_hash(HERE / name) for name in wrapper_names}
    run = dict(
        environment="isolated-sh2-interpreter", sourceCommit=source_commit,
        compiler=raw["compiler"], compileFlags=raw["compileFlags"], toolchainVerifiedInELF=True,
        elfSHA256=digest(elf), symbolSHA256=digest(symbols), rawCaptureSHA256=file_hash(args.raw),
        evidenceUrl=args.evidence_url,
        interpreterRepository="https://github.com/Yabause/yabause", interpreterCommit=core_commit,
        interpreterSource="yabause/src/sh2int.c",
        interpreterSourceSHA256=file_hash(args.core / "yabause/src/sh2int.c"), interpreterModified=False,
        memoryModel="flat RAM", cpuCount=1, wholeBatch=True,
        cycleUnit="upstream interpreter per-opcode modeled cycles",
        hardwareMeasured=False, browserTimingUsed=False, hostResults=False,
        includes=["32 public API calls per batch", "identical batch dispatch and input loads",
                  "all result output writes", "normal numeric range checks and fallbacks"],
        excludes=["fixture construction", "output equality and SR.S checks", "host execution overhead"],
        limitations=LIMITATIONS,
        bridgeSourceSHA256=file_hash(HERE / "math_bridge.c"),
        workloadsSourceSHA256=file_hash(HERE / "workloads.inc"),
        casesSourceSHA256=file_hash(HERE / "cases.inc"), suiteSourceSHA256=file_hash(HERE / "suite.h"),
        harnessSourceSHA256=file_hash(HERE.parent / "sh2-model/harness.c"),
        captureSourceSHA256=file_hash(HERE / "capture.h"),
        collectorSourceSHA256=file_hash(Path(__file__)), validatorSourceSHA256=file_hash(HERE / "validate.cjs"),
        librarySourceSHA256=library_hashes, baselineWrapperSHA256=wrapper_hashes,
        baselineRenameSHA256=file_hash(HERE / "legacy/rename.h"),
    )
    result = dict(
        schema=1, kernelSet=KERNEL_SET,
        comparison=dict(before="previous SH2ZAM", after="optimized SH2ZAM", independentNonShazamBaseline=False),
        baseline=dict(sourceCommit=BASELINE_COMMIT, manifestSHA256=file_hash(args.manifest), files=manifest["files"]),
        run=run, batchSize=32, samplesPerProfile=7, profiles=PROFILES,
        checkedOutputWords=raw["checkedOutputWords"], mismatches=0, records=records,
    )
    # The independent Node validator is also the publication gate. It checks
    # all derived fields and local baseline bytes before this file is written.
    subprocess.run(["node", str(HERE / "validate.cjs"), "-"],
                   input=json.dumps(result, allow_nan=False), text=True, check=True)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, separators=(",", ":"), allow_nan=False) + "\n")
    for record in records:
        print(f'{record["operation"]:26s} {record["profile"]:10s} '
              f'{record["cyclesPerCall"]["before"]:10.3f} -> {record["cyclesPerCall"]["after"]:10.3f} '
              f'modeled cycles/call; {record["speedup"]:.3f}x; {record["cyclesSavedPercent"]:+.3f}% saved')
    print(f'PASS: {len(records)} workloads; {raw["checkedOutputWords"]} exact native output words; '
          'SR.S preserved; every improvement and regression retained')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ["raw", "elf", "symbols", "core", "manifest", "output"]:
        parser.add_argument("--" + name, type=Path, required=True)
    parser.add_argument("--evidence-url", required=True)
    parser.add_argument("--source-commit")
    args = parser.parse_args()
    try:
        collect(args)
    except (ValueError, OSError, subprocess.CalledProcessError) as error:
        parser.exit(1, f"Math model collection rejected: {error}\n")


if __name__ == "__main__":
    main()
