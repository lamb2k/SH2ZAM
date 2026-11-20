# Validation record — 2026-11-19 (SCU DSP access, affine transform, Q16 outer product, dsp-1)

`sh2zam/dsp_experimental.h` ([guide](../DSP.md)) drives the SCU DSP:
- program and data RAM access, start, poll and wait, and cache purge;
- a batched affine transform;
- a Q16 outer product.

The kernels are DSP assembly in `saturn/src/dsp/`, assembled by `saturn/tools/sh2z_dspasm.py`
and committed as data; CI checks them against their sources. **No console run was done in this repository.** Evidence by tier:

- **HOST:** `dsp_exact` passes 7,776 checks under GCC and Clang Release with `SH2Z_STRICT=ON` and under ASan/UBSan:
  - the committed kernel words run on a reference DSP simulator (late DMA, bank faults);
  - 60,180 transform vectors and 215,147 Q16 products equal the CPU form, with guard words
    intact;
  - 15 mutation controls are all caught;
  - the assembler passes 38 hand-derived encodings and 10 rejected inputs, and its control fails.
- **Cycles, exact in DSP instructions:** the transform costs 20 per vector (414 for the first
  block of 20). The product costs 78 for 1 x 8 and 463 for 8 x 8.
- **EMULATOR, timing modelled (Ymir; no charge for DSP DMA or bus sharing):** break-even against
  plain C on the master SH-2 is about 4 vectors. At 256 vectors the job frees 688 us of the
  master's time.
- **SH-2 build:** the source compiles with the CI flags and leaves nothing in plain sections
  under the consumer flags. The object is 2,164 bytes (`-m2 -mb -O2`); it needs `___udivdi3`.
- **Not done:** console execution. DMA time and its contention on hardware are not modelled.
