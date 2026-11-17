# Calibration image

A Saturn disc image that times single SH-2 operations, memory reads, and
`sh2z_view_batch`, then shows everything on one screen. It contains no game
data. Burn it, run it from an optical drive emulator, or load it in an
emulator, then photograph the screen.

The numbers relate the instruction model (`../../sh2-model`), emulators, and a
real console to each other. Until someone runs the image, every row is
**unrun**: CI only proves that it builds.

## Getting it

The `sh2-compile-evidence` artifact of the **SH-2 cross compile** workflow holds
`saturn/benchmarks/yaul/calibration/sh2zam-calibration.iso` (plus `.cue`), and
`sh2-calibration-image-sha256.txt` records its hash. To build it locally,
inside the `ijacquez/yaul` image:

```sh
make -C saturn/benchmarks/yaul/calibration
```

## Reading the screen

The first line shows the source commit. Columns: row, cycles per unit (two
decimals), raw timer ticks, unit.

- **Timer.** The free-running timer runs at CPU clock / 8, so cycles =
  ticks × 8. All interrupts are masked while timing. Each timed span clears the
  timer's overflow flag first and checks it afterwards; if the counter wrapped,
  the row shows `OVF` instead of a number. Every row is the median of 7 runs.
- **Arithmetic rows** (`nop`, `dmuls.l`, `dmuls+sts mach`, `mac.l`) run 16
  operations per loop iteration. Each value is the difference between 512 and
  256 iterations, divided by 4,096 operations, so call and timer overhead cancel
  out. The loop's own `dt`/`bf` remains, spread over the 16 operations; the
  `nop` row shows how much that is.
  - `dmuls+sts mach` reads the result straight away, which exposes the
    multiplier latency.
  - `mac.l` reads both operands from cached RAM.
- **Call rows** use the same 512 − 256 difference, over 256 calls:
  - `sh2z_div` (the DIV1 loop)
  - one hardware-divider 64/32 division through `sh2z_divu_start64` / `finish`
  - GCC's 64-bit multiply and divide
- **Memory rows** time 64 KB of sequential 32-bit loads, as four 16 KB spans
  (`mov.l @r+` unrolled 16 times), and report cycles per word. The regions
  are:
  - high work RAM (a buffer in `.bss`)
  - low work RAM from `0x00200000`
  - a RAM cartridge, if Yaul's `dram_cart_id_get` finds one

  Each region is read through its cached address (area 0) and its
  cache-through address (+`0x20000000`).
- **View rows** time `sh2z_view_batch` and `sh2z_view_batch_divu` on 512
  vertices, as four calls of 128. Every vertex is in front of the camera and
  inside the far plane, so each one costs two divisions.

Checks at the bottom:

| Line | Meaning |
| --- | --- |
| `mem sums a = b` | Cached and cache-through reads of the same RAM returned the same words. |
| `view fnv` | FNV-1a of the portable batch output. It should match on every machine that runs this commit. |
| `divu==portable YES` | The hardware-divider batch produced exactly the portable bytes on this machine. |
| `DIVU vs C: 0/4096 bad` | 4,096 random signed 64/32 divisions that fit the divider: its quotients equal C's truncating division. |
| `cart id` | `5a` = 1 MB, `5c` = 4 MB, `00` = none. |

On a console, the last two lines are the first hardware evidence for the
assumptions the divider code relies on (`../../../DIVU.md`): truncation toward
zero and the fit rule.

The CPU clock depends on the video mode. This image uses 320-pixel NTSC (about
26.87 MHz), and cycle counts do not depend on it.
