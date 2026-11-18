# Explicit SH-2 divider (experimental)

`sh2zam/divu_experimental.h` gives explicit, caller-controlled access to the
SH-2's on-chip division unit. Nothing else in SH2ZAM touches the divider:
only these calls and functions whose names end in `_divu`
(`sh2z_view_batch_divu`) do.

## API

```c
sh2z_divu_t t;
sh2z_divu_start64(&t, hi, lo, divisor);   /* starts hi:lo / divisor */
/* ... up to ~39 cycles of independent work ... */
bool overflow;
int32_t q = sh2z_divu_finish(&t, &overflow);

int32_t q2 = sh2z_divu_div64(n, divisor, NULL);   /* blocking form */
```

The result rule is the same on every build:

| Case | Result | `overflow` |
| --- | --- | --- |
| `d != 0`, `abs(n) < abs(d) * 2^31` | `trunc(n / d)` | false |
| `d != 0`, otherwise | `sat32(trunc(n / d))` | true |
| `d == 0` | `n >= 0 ? INT32_MAX : INT32_MIN` | true |

An exact quotient of `-2^31` is returned exactly but still flagged, because
it is the one value whose hardware handling is not established. Only divisions
in the first row ever reach the hardware; the rest are answered in software
before the divider is touched, so no result depends on the divider's overflow
or divide-by-zero behaviour. Portable builds compute the same results in C.

## Fixed-point quotients (contract tag `trig12-1`)

```c
int32_t r = sh2z_divu_q16(num, den);       /* num * 2^16 / den */
int32_t u = sh2z_divu_q30(num, den);       /* num * 2^30 / den */
int32_t q = sh2z_divu_q(num, den, f);      /* constant 0 <= f <= 31 */
int32_t i = sh2z_recip_q30(den);           /* 2^60 / den; also sh2z_recip_q16 */
sh2z_divu_q_start(&t, num, den, 16);       /* then sh2z_divu_finish(&t, &overflow) */
```

These are the division above with the dividend `num * 2^f`. The result rule and the overflow
flag are the same, and rounding is toward zero. The fit test runs on 32-bit magnitudes
(`abs(num) >> (31 - f) < abs(den)`). All forms are inline. `sh2z_divu_q16` equals `sh2z_div`
except at `0/0`, and `sh2z_recip_q16` equals `sh2z_recip`. Edge cases, accuracy and modelled
costs are in [TRIG12.md](TRIG12.md): 56 cycles for a blocking q16, against 98.5 for
`sh2z_div`; about 41 cycles beyond the overlapped work.

## Caller promise

Each SH-2 has its own divider. Between `sh2z_divu_start64` and
`sh2z_divu_finish`, and for the whole of a `_divu` batch call, no other code on
that CPU may use the divider, unless every interrupt handler that uses it does
this:

```c
void my_vblank_handler(void) {
    sh2z_divu_state_t saved;
    sh2z_divu_save(&saved);      /* waits for a running division */
    /* ... handler work that uses the divider ... */
    sh2z_divu_restore(&saved);
}
```

`sh2z_divu_restore` leaves `DVSR`, `DVDNTH` and `DVDNTL` exactly as saved, so an
interruption anywhere inside a start/finish pair is invisible: it rebuilds the
saved quotient by dividing it by one, waits, then rewrites the divisor and the
high word. It costs about two divider latencies. Leave `DVCR` (and its
overflow-interrupt enable) at its reset value; SH2ZAM never writes it.

## Hardware assumptions (not yet checked on a console)

From the SH7604 hardware manual's division-unit chapter:

- Registers: `DVSR` at `0xFFFFFF00`, `DVDNTH` at `0xFFFFFF10`, `DVDNTL` at
  `0xFFFFFF14`, accessed as 32-bit words.
- Writing `DVDNTL` starts a signed 64/32 division of `DVDNTH:DVDNTL` by
  `DVSR`, truncating toward zero; the quotient lands in `DVDNTL`.
- It takes 39 cycles, and reading `DVDNTL` or `DVDNTH` earlier holds the CPU
  until it completes.

The SH-2 model harness implements exactly these assumptions on its own side
(`benchmarks/sh2-model/divider_model.h`, labelled harness code), and traps on
any division SH2ZAM promised never to issue: a zero divisor, a quotient that
does not fit strictly, or a write while a division runs. Agreement with that
model shows SH2ZAM keeps its promises; it is not evidence about the console.

## Evidence

- **HOST:** `tests/test_divu.c` — 2,000,000 random divisions (1,401,707 on the
  hardware path), boundary tables on both signs, against an independent
  128-bit oracle under GCC, Clang and ASan/UBSan. Controls: floor instead of
  truncation and "exact `-2^31` fits" are each detected.
- **MODELLED:** the view capture's probe step runs `start64`/`finish`,
  `div64`, and an interrupted division saved and restored around a second
  division, on the SH-2 interpreter: 2,023 probes, 1,673 of them through the
  hardware path, all equal to the host rule, with the expected number of
  hardware divisions each.
- **HARDWARE:** none. A console run should confirm the register behaviour,
  the latency and the read-holds behaviour before the cycle figures in
  [VIEW_BATCH.md](VIEW_BATCH.md) are trusted.

The [calibration image](benchmarks/yaul/calibration/README.md) checks truncation and the fit rule
on a console (4,096 random divisions against C) and times one division. Unrun so far.
