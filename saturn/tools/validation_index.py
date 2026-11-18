#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 SH2ZAM Contributors.
"""Generate or check saturn/VALIDATION.md, the index of saturn/validation/*.md.

Each contract's validation record is its own file, saturn/validation/<tag>.md,
whose first line is "# Validation record — <date> (<scope>)". The index lists
every record once, sorted by file name, so two lanes adding records touch
different lines.

usage: validation_index.py --write   rewrite saturn/VALIDATION.md
       validation_index.py --check   fail if saturn/VALIDATION.md is stale
"""
import os
import sys

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
INDEX = os.path.join(ROOT, "VALIDATION.md")
RECORDS = os.path.join(ROOT, "validation")
PREFIX = "# Validation record — "

HEAD = """# Validation records

Each contract's validation record is its own file in [`validation/`](validation/),
named after its contract tag. This index is generated: after adding or renaming
a record, run `python3 saturn/tools/validation_index.py --write`; host CI runs
`--check`. Records are listed by file name; each line gives the record's date
and scope.
"""


def build():
    lines = [HEAD]
    names = sorted(n for n in os.listdir(RECORDS) if n.endswith(".md"))
    if not names:
        raise SystemExit("no records in saturn/validation/")
    for name in names:
        with open(os.path.join(RECORDS, name), encoding="utf-8") as f:
            first = f.readline().rstrip("\r\n")
        if not first.startswith(PREFIX):
            raise SystemExit(f"validation/{name}: first line must start with {PREFIX!r}")
        lines.append(f"- [{name[:-3]}](validation/{name}): {first[len(PREFIX):]}\n")
    return lines[0] + "\n" + "".join(lines[1:])


def main(argv):
    if len(argv) != 2 or argv[1] not in ("--write", "--check"):
        print(__doc__, file=sys.stderr)
        return 2
    text = build()
    if argv[1] == "--write":
        with open(INDEX, "w", encoding="utf-8", newline="\n") as f:
            f.write(text)
        print(f"wrote {os.path.normpath(INDEX)}: {text.count(chr(10) + '- ')} records")
        return 0
    with open(INDEX, encoding="utf-8") as f:
        current = f.read().replace("\r\n", "\n")
    if current != text:
        raise SystemExit("saturn/VALIDATION.md is stale: run python3 saturn/tools/validation_index.py --write")
    print(f"validation index: {text.count(chr(10) + '- ')} records listed, in file-name order")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
