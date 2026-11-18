#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 SH2ZAM Contributors.
"""Generate or check the 12-bit-angle quarter sine table (contract tag trig12-1).

The table holds round(sin12(i) * 2^30) for i = 0..1024, where sin12(i) is the
float64 value of a common series construction for a 4096-entry sine table:

    x = (i * (2 * 3.1415926535)) / 4096          (binary64, as written)
    term_k = -x^3 / 3!, then each term times -x^2 / ((2j)(2j + 1)), j = 2..13
    sin12(i) = x + the 12 terms                   (binary64, in that order)

Python floats are IEEE binary64 with round-to-nearest-even and no fused
operations, so this reproduces a C double evaluation of the same statements.
The same expression is evaluated once more in exact rational arithmetic to
show that the binary64 rounding never changes the nearest Q2.30 integer.

usage: sin12_table.py            print the C table
       sin12_table.py --check F  verify the table in F and print the bounds
"""
import re
import sys
from fractions import Fraction

N = 4096
QUARTER = 1024
ONE = 1 << 30


def series64(i):
    x = (i * (2 * 3.1415926535)) / N
    numerator = -x * x * x
    minus_square = -x * x
    denominator = 1.0 * 2 * 3
    for j in range(2, 14):
        x += numerator / denominator
        numerator *= minus_square
        denominator *= j * ((j << 2) + 2)
    return x


def series_exact(i):
    x = Fraction((i * (2 * 3.1415926535)) / N)
    numerator = -x * x * x
    minus_square = -x * x
    denominator = Fraction(6)
    s = x
    for j in range(2, 14):
        s += numerator / denominator
        numerator *= minus_square
        denominator *= j * ((j << 2) + 2)
    return s


def nearest(v):
    """Nearest integer to a Fraction; ties never occur (checked)."""
    f = v.numerator // v.denominator
    return f + 1 if 2 * (v - f) > 1 else f


def build():
    table, worst_ref, worst_exact, tie_gap = [], Fraction(0), Fraction(0), Fraction(1)
    for i in range(QUARTER + 1):
        ref = Fraction(series64(i)) * ONE
        exact = series_exact(i) * ONE
        q = nearest(ref)
        if nearest(exact) != q:
            raise SystemExit(f"entry {i}: binary64 rounding changes the Q2.30 value")
        frac = ref - (ref.numerator // ref.denominator)
        tie_gap = min(tie_gap, abs(frac - Fraction(1, 2)))
        worst_ref = max(worst_ref, abs(ref - q))
        worst_exact = max(worst_exact, abs(exact - q))
        table.append(q)
    if table[0] != 0 or table[QUARTER] != ONE:
        raise SystemExit("endpoints must be 0 and 2^30")
    if any(b < a for a, b in zip(table, table[1:])):
        raise SystemExit("table must be nondecreasing")
    return table, worst_ref, worst_exact, tie_gap


def emit(table):
    lines = []
    for k in range(0, len(table), 6):
        lines.append("    " + ", ".join(str(v) for v in table[k:k + 6]) + ",")
    return "\n".join(lines)


def main(argv):
    table, worst_ref, worst_exact, tie_gap = build()
    if len(argv) == 3 and argv[1] == "--check":
        text = open(argv[2], encoding="ascii").read()
        m = re.search(r"sh2z_sin12_quarter_u30\[1025\]\s*=\s*\{([^}]*)\}", text)
        if not m:
            raise SystemExit("table not found")
        got = [int(v) for v in re.findall(r"-?\d+", m.group(1))]
        if got != table:
            raise SystemExit("table differs from the generator")
        print(f"sin12 table: 1025 entries match; max |entry - binary64 reference| = "
              f"{float(worst_ref):.9f} LSB, max |entry - exact series| = {float(worst_exact):.9f} LSB, "
              f"closest reference to a rounding tie: {float(tie_gap):.9f} LSB")
        return 0
    if len(argv) != 1:
        print(__doc__, file=sys.stderr)
        return 2
    print(emit(table))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
