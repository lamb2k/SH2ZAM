#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (c) 2026 SH2ZAM Contributors.
"""Package exact SH-2 workload comparisons without imposing a speedup target."""
import argparse
import hashlib
import json
import re
import shlex
import statistics
import subprocess
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
KERNEL_SET = "exact-workloads-sh2-model-v1"
RAW_KIND = "SH2ZAM exact workload optimizations"
BASELINE_COMMIT = "3374d77c4625749cc932cafa4385ad18db811e9b"
CORE_COMMIT = "7e38821dbac265490f115e163c523a939acda759"
BASELINE_FILES = ["sh2zam.c", "sh2zam_angles.c", "sh2zam_trig.c",
                  "sh2zam_transform.c", "sh2zam_geometry.c", "sh2zam_internal.h"]
COST_KEYS = ["cycles", "instructions", "dataReads", "dataWrites", "macInstructions", "dmulsInstructions"]
META_KEYS = ["id", "group", "profile", "size", "queries", "outputWords", "paths"]
FLAGS = ["-m2", "-mb", "-O2", "-fno-fast-math", "-ffp-contract=off", "-fno-lto"]
LIMITATIONS = [
    "Not physical Saturn timing or full-system emulator timing",
    "No cache or bus wait-state model",
    "No pipeline or multiplier-contention model",
    "No DMA, interrupts, timers or peripheral integration",
    "One SH-2 CPU; no second-CPU scheduling or shared-memory contention",
    "No conversion to milliseconds, frame rate or clock-frequency estimates",
]
SPECS = {
    "rotation": ([1, 8, 32, 128, 512], ["bounded", "tiny", "full-range", "identity", "zero"],
                 ["scalar", "composed", "composed_mac", "batch", "mac"]),
    "normalize": ([32], ["bounded", "tiny", "full-range"], ["prior", "current"]),
    "atan2": ([32], ["bounded", "tiny", "full-range"], ["prior", "current"]),
    "atan": ([32], ["bounded", "tiny", "full-range"], ["prior", "current"]),
    "spatial": ([128, 512], ["sparse", "mixed", "dense", "crowded", "full-range"],
                ["api_brute", "inline_brute", "index"]),
}


def require(condition, message):
    if not condition:
        raise ValueError(message)


def exact_keys(value, keys, label):
    require(isinstance(value, dict) and set(value) == set(keys), f"Unexpected/missing {label} fields")


def integer(value, minimum, label):
    require(type(value) is int and minimum <= value <= 9007199254740991, f"Invalid {label} integer")


def digest(data):
    return hashlib.sha256(data).hexdigest()


def file_hash(path):
    return digest(path.read_bytes())


def load_json(path):
    def pairs(items):
        result = {}
        for key, value in items:
            require(key not in result, "Duplicate JSON key: " + key)
            result[key] = value
        return result
    def nonfinite(value):
        raise ValueError("Nonfinite JSON number: " + value)
    return json.loads(path.read_text(), object_pairs_hook=pairs, parse_constant=nonfinite)


def baseline_paths(group):
    return ["scalar", "composed", "composed_mac"] if group == "rotation" else \
        ["api_brute", "inline_brute"] if group == "spatial" else ["prior"]


def validate_manifest(manifest):
    exact_keys(manifest, ["schema", "samplesPerCase", "cases"], "fixture manifest")
    require(type(manifest["schema"]) is int and manifest["schema"] == 1 and
            type(manifest["samplesPerCase"]) is int and manifest["samplesPerCase"] == 7,
            "Wrong fixture manifest schema/sample count")
    cases = manifest["cases"]
    require(isinstance(cases, list) and len(cases) == 44, "Expected all 44 workload cases")
    ids, keys = set(), set()
    for case in cases:
        exact_keys(case, META_KEYS, "fixture case")
        require(isinstance(case["id"], str) and re.fullmatch(r"[a-z][a-z0-9_.:-]{0,95}", case["id"]),
                "Invalid case ID")
        require(case["id"] not in ids, "Duplicate case ID")
        ids.add(case["id"])
        group = case["group"]
        require(group in SPECS, "Unknown workload group")
        sizes, profiles, paths = SPECS[group]
        integer(case["size"], 1, "size")
        integer(case["queries"], 0, "queries")
        integer(case["outputWords"], 1, "outputWords")
        require(case["size"] in sizes and case["profile"] in profiles and case["paths"] == paths,
                "Unexpected fixture sizes, profiles or path order")
        require(case["queries"] == (128 if group == "spatial" else 0), "Unexpected query count")
        words = 128 * ((case["size"] + 31) // 32 + 1) if group == "spatial" else \
            case["size"] * (3 if group in ["rotation", "normalize"] else 1)
        require(case["outputWords"] == words, "Fixture output width differs")
        key = (group, case["profile"], case["size"])
        require(key not in keys, "Duplicate fixture coverage")
        keys.add(key)
    expected = {(group, profile, size) for group, (sizes, profiles, _) in SPECS.items()
                for size in sizes for profile in profiles}
    require(keys == expected, "Incomplete fixture coverage")
    return cases


def validate_raw(raw, cases):
    exact_keys(raw, ["schema", "kind", "sourceCommit", "compiler", "compileFlags", "records",
                     "checkedOutputWords", "checkedComparisons", "mismatches"], "raw capture")
    require(type(raw["schema"]) is int and raw["schema"] == 1 and raw["kind"] == RAW_KIND,
            "Wrong native workload capture")
    require(type(raw["mismatches"]) is int and raw["mismatches"] == 0, "Native outputs disagree")
    require(isinstance(raw["records"], list) and len(raw["records"]) == len(cases), "Missing native records")
    by_id, seen, records = {case["id"]: case for case in cases}, set(), []
    for record in raw["records"]:
        exact_keys(record, META_KEYS + ["samples", "guards"], "raw record")
        key = record["id"]
        require(key in by_id and key not in seen, "Unknown/duplicate native case")
        seen.add(key)
        require(all(record[name] == by_id[key][name] for name in META_KEYS), "Native metadata differs: " + key)
        for name in ["size", "queries", "outputWords"]:
            integer(record[name], 0 if name == "queries" else 1, name)
        exact_keys(record["guards"], ["srSRetained", "srSOutputMatch", "noMacWithS"], "native guards")
        require(all(value is True for value in record["guards"].values()), "Native SR.S guards failed: " + key)
        samples, paths = record["samples"], record["paths"]
        require(isinstance(samples, list) and len(samples) == 7, "Incomplete samples: " + key)
        for sample in samples:
            exact_keys(sample, ["inputChecksum", "paths", "mismatches"], "native sample")
            require(isinstance(sample["inputChecksum"], str) and re.fullmatch(r"[a-f0-9]{8}", sample["inputChecksum"]),
                    "Invalid native input checksum")
            require(type(sample["mismatches"]) is int and sample["mismatches"] == 0, "Native sample differs")
            exact_keys(sample["paths"], paths, "sample paths")
            checksums = []
            for name in paths:
                cost = sample["paths"][name]
                exact_keys(cost, COST_KEYS + ["checksum"], "native path counters")
                for metric in COST_KEYS:
                    integer(cost[metric], 1 if metric in ["cycles", "instructions"] else 0, metric)
                require(cost["macInstructions"] + cost["dmulsInstructions"] <= cost["instructions"],
                        "Impossible opcode counters")
                require(isinstance(cost["checksum"], str) and re.fullmatch(r"[a-f0-9]{8}", cost["checksum"]),
                        "Invalid native output checksum")
                checksums.append(cost["checksum"])
            require(len(set(checksums)) == 1, "Native path outputs differ: " + key)
        medians = {name: statistics.median(sample["paths"][name]["cycles"] for sample in samples) for name in paths}
        denominator = record["queries"] if record["group"] == "spatial" else record["size"]
        ratios = {baseline: {name: {"speedup": medians[baseline] / medians[name],
                                   "cyclesSavedPercent": (1 - medians[name] / medians[baseline]) * 100}
                             for name in paths} for baseline in baseline_paths(record["group"])}
        records.append(dict(record, sampleCount=7, setupIncluded=True, medianCycles=medians,
                            cyclesPerItem={name: cycles / denominator for name, cycles in medians.items()},
                            relativeTo=ratios, checkedOutputWords=7 * record["outputWords"],
                            checkedComparisons=7 * record["outputWords"] * (len(paths) - 1)))
    checked = sum(record["checkedOutputWords"] for record in records)
    comparisons = sum(record["checkedComparisons"] for record in records)
    for name, expected in [("checkedOutputWords", checked), ("checkedComparisons", comparisons)]:
        integer(raw[name], 1, name)
        require(raw[name] == expected, "Incomplete exact native comparisons: " + name)
    order = {case["id"]: i for i, case in enumerate(cases)}
    return sorted(records, key=lambda record: order[record["id"]])


def collect(args):
    manifest = load_json(args.manifest)
    cases = validate_manifest(manifest)
    require(args.manifest.read_bytes() == (HERE / "manifest.json").read_bytes(), "Fixture manifest differs")
    raw = load_json(args.raw)
    records = validate_raw(raw, cases)
    elf, symbols = args.elf.read_bytes(), args.symbols.read_bytes()
    require(len(elf) >= 52 and elf[:6] == b"\x7fELF\x01\x02" and int.from_bytes(elf[18:20], "big") == 42,
            "Expected ELF32 big-endian SuperH binary")
    head = subprocess.check_output(["git", "-C", str(ROOT), "rev-parse", "HEAD"], text=True).strip()
    source_commit = args.source_commit or raw["sourceCommit"]
    require(isinstance(source_commit, str) and re.fullmatch(r"[a-f0-9]{40}", source_commit) and
            source_commit == raw["sourceCommit"] == head, "ELF source commit must match git HEAD")
    for key in ["sourceCommit", "compiler", "compileFlags"]:
        value = raw[key]
        require(isinstance(value, str) and value and "\x00" not in value and value.encode() + b"\x00" in elf,
                "Metadata not verified inside ELF: " + key)
    flags = shlex.split(raw["compileFlags"])
    require(all(flag in flags for flag in FLAGS) and "-ffast-math" not in flags and "-flto" not in flags,
            "Unexpected compiler configuration")
    for suffix in ["source_commit", "compiler", "compile_flags", "prepare", "run", "cases", "case_count",
                   "inputs", "input_bytes", "outputs", "max_words"]:
        name = "workload_" + suffix
        require(re.search(rb"(?m)^\s*[0-9a-fA-F]+\s+\S\s+_?" + name.encode() + rb"\s*$", symbols),
                "Missing retained native symbol: " + name)
    require(re.fullmatch(r"https://github\.com/[A-Za-z0-9_.-]+/[A-Za-z0-9_.-]+/actions/(?:runs/[0-9]+|workflows/[^\s?#]+)",
                         args.evidence_url), "Expected public GitHub Actions evidence URL")
    core_commit = subprocess.check_output(["git", "-C", str(args.core), "rev-parse", "HEAD"], text=True).strip()
    require(core_commit == CORE_COMMIT, "Unexpected interpreter version")
    subprocess.run(["git", "-C", str(args.core), "diff", "--exit-code", "HEAD", "--", "yabause/src"], check=True)
    prior_path = HERE / "prior/manifest.json"
    prior = load_json(prior_path)
    exact_keys(prior, ["sourceCommit", "files"], "prior manifest")
    require(prior["sourceCommit"] == BASELINE_COMMIT, "Wrong frozen prior commit")
    exact_keys(prior["files"], BASELINE_FILES, "prior source files")
    for name in BASELINE_FILES:
        require(file_hash(HERE / "prior" / name) == prior["files"][name], "Frozen prior source differs: " + name)
    library_paths = sorted([* (ROOT / "saturn/src").glob("*.c"), * (ROOT / "saturn/src").glob("*.h"),
                            * (ROOT / "saturn/include/sh2zam").glob("*.h")])
    library_hashes = {str(path.relative_to(ROOT)): file_hash(path) for path in library_paths}
    wrappers = ["prior_" + name for name in BASELINE_FILES if name.endswith(".c")]
    run = dict(
        environment="isolated-sh2-interpreter", sourceCommit=source_commit,
        compiler=raw["compiler"], compileFlags=raw["compileFlags"], toolchainVerifiedInELF=True,
        elfSHA256=digest(elf), symbolSHA256=digest(symbols), rawCaptureSHA256=file_hash(args.raw), evidenceUrl=args.evidence_url,
        interpreterRepository="https://github.com/Yabause/yabause", interpreterCommit=core_commit,
        interpreterSource="yabause/src/sh2int.c", interpreterSourceSHA256=file_hash(args.core / "yabause/src/sh2int.c"),
        interpreterModified=False, memoryModel="flat RAM", cpuCount=1, wholeBatch=True, setupIncluded=True,
        cycleUnit="upstream interpreter per-opcode modeled cycles", hardwareMeasured=False, browserTimingUsed=False, hostResults=False,
        includes=["identical inputs and all observable result writes", "normal validation and exact fallbacks",
                  "one quaternion normalization/matrix construction per composed or batch rotation",
                  "input scan and plan qualification for native rotation composition",
                  "one spatial index build plus all 128 queries per spatial sample"],
        excludes=["fixture construction", "output equality and SR.S checks", "host execution overhead"], limitations=LIMITATIONS,
        fixtureManifestSHA256=file_hash(args.manifest),
        bridgeSourceSHA256=file_hash(HERE / "workload_bridge.c"), standaloneSourceSHA256=file_hash(HERE / "standalone.c"),
        casesSourceSHA256=file_hash(HERE / "cases.inc"), suiteSourceSHA256=file_hash(HERE / "suite.h"),
        harnessSourceSHA256=file_hash(HERE.parent / "sh2-model/harness.c"), captureSourceSHA256=file_hash(HERE / "capture.h"),
        collectorSourceSHA256=file_hash(Path(__file__)), validatorSourceSHA256=file_hash(HERE / "validate.cjs"),
        librarySourceSHA256=library_hashes, baselineWrapperSHA256={name: file_hash(HERE / name) for name in wrappers},
        baselineRenameSHA256=file_hash(HERE / "prior/rename.h"), baselineApiSHA256=file_hash(HERE / "prior_api.h"),
    )
    result = dict(schema=1, kernelSet=KERNEL_SET,
                  comparison=dict(priorLibraryCommit=BASELINE_COMMIT,
                      rotationBaselines=["scalar", "composed", "composed_mac"],
                      spatialBaselines=["api_brute", "inline_brute"],
                      scalarBaseline="prior", claimsUniversalTenfoldGain=False),
                  baseline=dict(sourceCommit=BASELINE_COMMIT, manifestSHA256=file_hash(prior_path), files=prior["files"]),
                  run=run, samplesPerCase=7, checkedOutputWords=raw["checkedOutputWords"],
                  checkedComparisons=raw["checkedComparisons"], mismatches=0, records=records)
    subprocess.run(["node", str(HERE / "validate.cjs"), "-"],
                   input=json.dumps(result, allow_nan=False), text=True, check=True)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, separators=(",", ":"), allow_nan=False) + "\n")
    for record in records:
        print(record["id"], record["medianCycles"], record["relativeTo"])
    print(f'PASS: {len(records)} workload cases; {raw["checkedOutputWords"]} output words; '
          f'{raw["checkedComparisons"]} exact path comparisons; setup included; regressions retained')


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
        parser.exit(1, f"Workload model collection rejected: {error}\n")


if __name__ == "__main__":
    main()
