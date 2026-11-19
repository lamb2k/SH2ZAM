# SCU DSP (`dsp-1`, experimental)

Contract: [`dsp_experimental.h`](include/sh2zam/dsp_experimental.h), section "Contract tag
dsp-1". Tests: `tests/test_dsp.c` with the reference simulator `tests/dsp_sim.h`. Kernels:
`src/dsp/*.dsp`, assembled by [`tools/sh2z_dspasm.py`](tools/sh2z_dspasm.py) into
`src/sh2zam_dsp_*.inc`.

The SCU DSP runs one instruction per SCU cycle (half the SH-2 clock), with a 32 x 32 multiplier,
a 48-bit ALU, four 64-word data banks and its own DMA. It has no divider and no indexed data
access. Per operation it is slower than an SH-2. Its value is parallel work: a job started by
one CPU runs while that CPU does something else.

## What it provides

| Part | Functions | Notes |
| --- | --- | --- |
| Machine | `sh2z_dsp_load`, `_data_write`, `_data_read`, `_start`, `_busy`, `_wait` | the CPU touches program and data RAM only while the DSP is stopped |
| Addresses | `sh2z_dsp_d0`, `sh2z_dsp_purge` | DMA uses word addresses; DSP writes bypass the SH-2 cache |
| Affine job | `sh2z_dsp_xform_prepare`, `_start`, `_finish`, `_cpu` | `out = m * v + t` per vector, exact modulo 2^32; output in row-major blocks of 20 |
| Q16 outer product | `sh2z_dsp_scale16_prepare`, `_start`, `_finish`, `_cpu` | `floor(a[i] * b[k] / 65536)`, 16 x 16 at most, exact while `|a b| < 2^47` |

A job is a small block in work RAM. Starting it costs one data RAM word and one control write;
the DSP fetches the block, its inputs and writes its outputs by DMA. Portable builds have no DSP:
`_start` runs the CPU form, so host programs get the same results.

## Kernel rules

- Every store of an ALU result happens on an instruction whose ALU field is NOP, so the stored
  value is the finished sum whether the D1 bus samples this instruction's ALU result or the
  previous one.
- Every DMA is followed by a `JMP T0` wait before its bank is touched; D0 addresses are set
  explicitly before every transfer (no reliance on address auto-advance).
- Loops use `BTM` with the last body instruction in its delay slot. `LPS`, `MVI` to `PC` and
  the end interrupt are not used.

## Evidence

**HOST.** The committed kernel words run on the reference simulator, which completes each DMA up
to 80 instructions late, treats any access to a bank under DMA as a fault and leaves RA0/WA0
unchanged after a transfer. Results equal the CPU form for 60,180 transform vectors (random full
range, Q1.14 rotations on int16 positions, extremes; 1 to 1,000 vectors) and 215,147 Q16
products, with guard words intact. 15 mutation controls (an ALU op changed, a clear or wait
removed, a DMA count or loop condition changed, a constant altered) are all caught. GCC strict,
Clang strict and ASan/UBSan clean. The assembler is checked against 38 hand-derived encodings and
10 rejected inputs; a control that alters one expectation must fail.

**Cycles (exact, DSP instructions).** Affine: 20 per vector, 414 for the first block of 20 and
400 per further block. Q16 outer product: 78 for 1 x 8, 463 for 8 x 8.

**EMULATOR (Ymir, timing modelled; Ymir charges nothing for DSP DMA or bus sharing).**
Affine job against the same transform in plain C on the master SH-2, 3 x 3 + translation:

| N | SH-2 C | DSP wall time | master cost to start + collect | master time freed |
| ---: | ---: | ---: | ---: | ---: |
| 16 | 48 us | 34 us | 12 us | 36 us |
| 64 | 189 us | 119 us | 25 us | 164 us |
| 256 | 750 us | 370 us | 62 us | 688 us |

Break-even is about 4 vectors. The collect cost is mostly the cache purge (about 0.2 us per
vector); reading results cache-through avoids it. With the master running 937 us of its own
work during a 320-vector job, the total was 1,007 us: the job was fully hidden.

**Not established.** No console run. DMA time and its contention with the SH-2s and VDP access
are not modelled by the emulator; the hardware figures will be worse.
