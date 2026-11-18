#!/usr/bin/env python3
# Generates the cotangent table of saturn/src/sh2zam_view_camera.c
# (contract tag view-camera-1): entry h, h = 1..1023, is
#     round_half_up(cot(pi * h / 2048) * 2^20)
# (a 4096-step turn, Q12.20); entry 0 is 0 and never used.
# Exact decimal arithmetic at 60 digits, so no entry depends on binary
# floating point. The host test re-derives every entry from long double.
# MIT; SH2ZAM Contributors.
from decimal import Decimal, getcontext, ROUND_FLOOR

getcontext().prec = 60
PI = Decimal("3.14159265358979323846264338327950288419716939937510582097494459")


def sin_cos(x):
    s, c = Decimal(0), Decimal(0)
    term_s, term_c = x, Decimal(1)
    for n in range(1, 80):
        s += term_s
        c += term_c
        term_s = -term_s * x * x / ((2 * n) * (2 * n + 1))
        term_c = -term_c * x * x / ((2 * n - 1) * (2 * n))
    return s, c


def entry(h):
    if h == 0:
        return 0
    s, c = sin_cos(PI * h / 2048)
    v = c / s * (1 << 20)
    frac = v - v.to_integral_value(rounding=ROUND_FLOOR)
    assert abs(frac - Decimal("0.5")) > Decimal("1e-20"), h
    return int((v + Decimal("0.5")).to_integral_value(rounding=ROUND_FLOOR))


values = [entry(h) for h in range(1024)]
assert values[1023] > 0 and all(values[h] > values[h + 1] for h in range(1, 1023))
assert values[1] < (1 << 30)
print("static const int32_t cot_q20[1024] = {")
for i in range(0, 1024, 8):
    print("    " + ", ".join(str(v) for v in values[i:i + 8]) + ",")
print("};")
