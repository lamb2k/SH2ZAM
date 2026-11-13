"""Report the actual SH-2 batch code; instruction counts are not cycle timings."""
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

names = ["sh2z_mat34_transform_point", "sh2z_mat34_transform_points_general",
         "sh2z_mat34_transform_points"]
report = {}
for name in names:
    if name not in functions:
        raise SystemExit("Missing SH-2 symbol: " + name)
    text = "\n".join(functions[name])
    report[name] = {
        "dmulsInstructions": len(re.findall(r"\bdmuls\.l\b", text)),
        "callInstructions": len(re.findall(r"\b(?:jsr|bsr|bsrf)\b", text)),
        "stackReferences": len(re.findall(r"@(?:-r15|r15\+?|\([^)]*,r15\))", text)),
        "relocationTargets": sorted(set(re.findall(r"R_SH_\w+\s+(\S+)", text))),
    }

print("SH-2 static instruction audit (not runtime/cycle measurements)")
print(json.dumps(report, indent=2))
fast = report["sh2z_mat34_transform_points"]
if fast["dmulsInstructions"] != 9:
    raise SystemExit("Inspect batch code: expected three native multiplies per unrolled row")
print("\nOptimized batch function, including matrix gate and general fallback:")
print("\n".join(functions["sh2z_mat34_transform_points"]))
