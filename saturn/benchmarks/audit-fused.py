"""Inspect experimental SH-2 loops. Static counts are not cycle measurements.

Usage: python3 audit-fused.py sh2-fused-disassembly.txt
Generate input with sh2eb-elf-objdump -dr sh2zam_batch_sh2.o, without LTO.
"""
from pathlib import Path
import json
import re
import sys

source = Path(sys.argv[1]).read_text()
functions = {}
name = None
for line in source.splitlines():
    match = re.match(r"^[0-9a-f]+ <([^>]+)>:$", line)
    if match:
        name = match.group(1).lstrip("_")
        functions[name] = []
    elif name is not None:
        functions[name].append(line)


def find_function(symbol):
    matches = [key for key in functions if key == symbol or key.startswith(symbol + ".")]
    if len(matches) != 1:
        raise SystemExit(f"Expected one SH-2 symbol for {symbol}, found {matches}")
    return matches[0], "\n".join(functions[matches[0]])


expected = {
    "sh2zi_batch_mac_kernel": {"mac.l": 9, "dmuls.l": 0, "sts": 6, "xtrct": 3},
    "sh2zi_batch_dmuls_kernel": {"mac.l": 0, "dmuls.l": 9, "sts": 18, "addc": 12, "xtrct": 3},
}
report = {}
loops = {}
for symbol, wanted in expected.items():
    actual_name, text = find_function(symbol)
    loops[actual_name] = text
    instructions = {
        opcode: len(re.findall(r"\b" + re.escape(opcode) + r"\b", text))
        for opcode in ["mac.l", "dmuls.l", "clrmac", "clrt", "sts", "addc", "xtrct"]
    }
    calls = len(re.findall(r"\b(?:jsr|bsr|bsrf)\b", text))
    report[actual_name] = {
        "staticInstructions": instructions,
        "callInstructions": calls,
        "stackReferencesIncludingPrologueEpilogue": len(
            re.findall(r"@(?:-r15|r15\+?|\([^)]*,r15\))", text)
        ),
        "relocationTargets": sorted(set(re.findall(r"R_SH_\w+\s+(\S+)", text))),
    }
    if calls:
        raise SystemExit(f"Inspect {actual_name}: the vertex loop contains a call")
    for opcode, count in wanted.items():
        if instructions[opcode] != count:
            raise SystemExit(
                f"Inspect {actual_name}: expected {count} {opcode}, got {instructions[opcode]}"
            )

guard_names = ["sh2z_transform_plan_mac_available", "sh2z_transform_plan_execute_mac"]
guard_text = "\n".join(find_function(symbol)[1] for symbol in guard_names)
if not re.search(r"\bstc\s+sr\s*,", guard_text):
    raise SystemExit("Missing SR read in MAC availability/execution guard")
if re.search(r"\bldc(?:\.l)?\s+[^\n]*\bsr\b", source):
    raise SystemExit("Experimental object unexpectedly writes SR")

print("Experimental SH-2 static audit; no native execution or timing is implied")
print(json.dumps(report, indent=2))
print("MAC guard reads SR; experimental object contains no write to SR.")
print("Stack-reference counts include saves/restores; inspect each loop for spills.")
for name, text in loops.items():
    print(f"\n{name}:\n{text}")
