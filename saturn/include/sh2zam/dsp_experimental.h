/* Experimental access to the Saturn SCU DSP, and a batched affine transform
 * that runs on it. Not used by any default dispatch.
 *
 * Contract tag: dsp-1.
 *
 * The SCU DSP is a separate fixed-point processor in the system control
 * unit: one instruction per SCU cycle (about 14.3 MHz, half the SH-2
 * clock), 256 words of program RAM, four banks of 64 words of data RAM, a
 * 32 x 32 multiplier with a 48-bit ALU, no divider, and its own DMA over the
 * D0 bus (high work RAM, A-bus, B-bus). It is never faster than an SH-2 per
 * operation; its value is that it runs while the SH-2 does other work.
 *
 * A. Machine: load a program, write and read data RAM, start, poll, wait.
 *    The CPU may touch program and data RAM only while the DSP is stopped.
 * B. D0 addresses and caches: the DSP's DMA addresses work RAM by 32-bit
 *    word (byte address >> 2). Its writes bypass the SH-2 cache, so the CPU
 *    purges the lines it will read back (or reads them cache-through). SH-2
 *    stores are write-through, so data the CPU wrote is visible to the DSP.
 * C. Affine job (sh2z_dsp_xform_*): for each input vector v = (x, y, z),
 *        out[r] = m[r][0] * x + m[r][1] * y + m[r][2] * z + m[r][3]
 *    for r = 0, 1, 2, every product and sum taken modulo 2^32 (two's
 *    complement wrap). Exact: the DSP and the CPU form give the same bits
 *    for every input. When the true sum fits int32 (for example a Q1.14
 *    rotation applied to int16 positions under sh2z_view_t's bound), the
 *    result is the exact sum.
 *    Layout. Input: count vectors of three int32 words, packed. Output:
 *    blocks of SH2Z_DSP_XFORM_BATCH vectors, row-major inside a block:
 *        out[(k / B) * 3B + r * B + k % B],  B = SH2Z_DSP_XFORM_BATCH
 *    (sh2z_dsp_xform_at). Both arrays are padded to whole blocks
 *    (sh2z_dsp_xform_padded): the DSP reads and writes whole blocks, so the
 *    padding vectors are read and their results written.
 *
 * D. Q16 outer product (sh2z_dsp_scale16_*): for a[0..n-1], b[0..m-1]
 *    (1 <= n, m <= 16),
 *        out[i * m + k] = floor(a[i] * b[k] / 65536)
 *    from the exact 64-bit product, valid when |a[i] * b[k]| < 2^47 (the
 *    DSP's 48-bit ALU). For a[i] >= 0 and b[k] >= 0 floor is also trunc,
 *    so a caller wanting trunc(d * f / 65536) passes |d| and restores the
 *    sign. Exact: the DSP and the CPU form give the same bits.
 *
 * Machine facts used, from the public SCU description and checked against
 * emulator behaviour, not yet on a console: the four ports at 0x25FE0080
 * (PPAF, PPD, PDA, PDD); PPAF bit 15 loads the program counter from bits
 * 0-7, bit 16 starts execution and reads back as "executing"; END stops
 * execution; DMA on the D0 bus to and from high work RAM; the T0 flag is set
 * while a DSP DMA runs. The kernel never relies on the end interrupt.
 *
 * Synchronisation belongs to the caller: one job at a time, and nothing
 * else uses the DSP between sh2z_dsp_xform_start and _finish. Portable
 * builds (anything that is not SH-2) have no DSP: the machine calls report
 * failure and the job runs on the CPU with the same results.
 *
 * Copyright (c) 2026 SH2ZAM Contributors. MIT License.
 * Inspired by SH4ZAM by Falco Girgis and SH4ZAM Contributors.
 */
#ifndef SH2ZAM_DSP_EXPERIMENTAL_H
#define SH2ZAM_DSP_EXPERIMENTAL_H

#include "sh2zam.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Contract tag dsp-1 ---------------------------------------------------- */

#if defined(__sh__) && defined(__SH2__) && (!defined(SH2Z_FORCE_PORTABLE) || !SH2Z_FORCE_PORTABLE)
#define SH2Z_DSP_NATIVE 1
#else
#define SH2Z_DSP_NATIVE 0
#endif

#define SH2Z_DSP_PPAF_ADDRESS UINT32_C(0x25FE0080)
#define SH2Z_DSP_PPD_ADDRESS  UINT32_C(0x25FE0084)
#define SH2Z_DSP_PDA_ADDRESS  UINT32_C(0x25FE0088)
#define SH2Z_DSP_PDD_ADDRESS  UINT32_C(0x25FE008C)
#define SH2Z_DSP_PPAF_LOAD_PC UINT32_C(0x00008000)
#define SH2Z_DSP_PPAF_EXECUTE UINT32_C(0x00010000)
#define SH2Z_DSP_PROGRAM_WORDS 256u
#define SH2Z_DSP_BANK_WORDS    64u

/* ---- A. Machine ---- */

/** Stop the DSP and copy words (at most 256) into program RAM from address
 *  0. Returns false, writing nothing, in portable builds, when words is
 *  null with words > 0, or when words > 256. */
bool sh2z_dsp_load(const uint32_t *program, uint32_t words);
/** Copy count words into data RAM bank (0..3) from index (0..63). The DSP
 *  must be stopped. False, writing nothing, when the range leaves the bank,
 *  an argument is invalid, or in portable builds. */
bool sh2z_dsp_data_write(uint32_t bank, uint32_t index, const uint32_t *words, uint32_t count);
/** Read count words of data RAM into words; the same rules. */
bool sh2z_dsp_data_read(uint32_t bank, uint32_t index, uint32_t *words, uint32_t count);
/** Start execution at pc (0..255). No-op in portable builds. */
void sh2z_dsp_start(uint32_t pc);
/** True while the DSP executes. Always false in portable builds. */
bool sh2z_dsp_busy(void);
/** Poll until the DSP stops, at most polls reads of PPAF (0: no limit).
 *  True when it has stopped. */
bool sh2z_dsp_wait(uint32_t polls);

/* ---- B. D0 addresses and caches ---- */

/** The D0-bus word address of a work RAM byte address: (p & 0x07FFFFFF) >> 2,
 *  so cached and cache-through views give the same word. Low two bits of p
 *  are ignored. */
static inline uint32_t sh2z_dsp_d0(const void *p) {
    return (uint32_t)(((uintptr_t)p&UINT32_C(0x07FFFFFF))>>2);
}
/** Purge the SH-2 cache lines covering [p, p + bytes) so the next reads
 *  fetch what the DSP wrote. No-op in portable builds. */
void sh2z_dsp_purge(const void *p, uint32_t bytes);

/* ---- C. Affine job ---- */

#define SH2Z_DSP_XFORM_BATCH 20u
#define SH2Z_DSP_XFORM_MAX   20480u

/** One job, read by the DSP's DMA: keep it in work RAM, 4-byte aligned, and
 *  unchanged until sh2z_dsp_xform_finish. 16 words. */
typedef struct {
    int32_t m[3][4];    /* rows: three coefficients, then the translation */
    uint32_t in_d0;     /* D0 word address of the input */
    uint32_t out_d0;    /* D0 word address of the output */
    uint32_t blocks;    /* number of blocks of SH2Z_DSP_XFORM_BATCH */
    uint32_t count;     /* vectors requested (CPU side only) */
    const int32_t *in;  /* CPU side only */
    int32_t *out;       /* CPU side only */
} sh2z_dsp_xform_t;

/** Vectors in whole blocks: count rounded up to a multiple of the batch. */
static inline uint32_t sh2z_dsp_xform_padded(uint32_t count) {
    return (count+SH2Z_DSP_XFORM_BATCH-1u)/SH2Z_DSP_XFORM_BATCH*SH2Z_DSP_XFORM_BATCH;
}
/** Output word index of row r (0..2) of vector k. */
static inline uint32_t sh2z_dsp_xform_at(uint32_t k, uint32_t r) {
    return k/SH2Z_DSP_XFORM_BATCH*(3u*SH2Z_DSP_XFORM_BATCH)+r*SH2Z_DSP_XFORM_BATCH+k%SH2Z_DSP_XFORM_BATCH;
}

/** The assembled kernel (generated from src/dsp/xform.dsp). */
extern const uint32_t sh2z_dsp_xform_program[];
extern const uint32_t sh2z_dsp_xform_program_words;
/** Data RAM bank and index of the job's D0 address word, written by
 *  sh2z_dsp_xform_start before it starts the kernel at address 0. */
#define SH2Z_DSP_XFORM_JOB_BANK  3u
#define SH2Z_DSP_XFORM_JOB_INDEX 1u

/** Load the kernel into program RAM (once; it stays until another program
 *  is loaded). False in portable builds. */
bool sh2z_dsp_xform_load(void);
/** Fill job for count (1..SH2Z_DSP_XFORM_MAX) vectors of in, results to
 *  out. in holds sh2z_dsp_xform_padded(count) * 3 words, out the same
 *  number; both 4-byte aligned, not overlapping. False, job unchanged, on
 *  a bad argument. */
bool sh2z_dsp_xform_prepare(sh2z_dsp_xform_t *job, const int32_t m[3][4],
    const int32_t *in, int32_t *out, uint32_t count);
/** Start the job. SH-2: the kernel must be loaded and the DSP idle; the
 *  call writes one data word and starts the DSP, then returns. Portable:
 *  computes the result on the CPU. */
void sh2z_dsp_xform_start(const sh2z_dsp_xform_t *job);
/** Wait for the job (at most polls reads of PPAF, 0: no limit), then purge
 *  the output's cache lines. True when the results are in out. */
bool sh2z_dsp_xform_finish(const sh2z_dsp_xform_t *job, uint32_t polls);
/** The same results computed on the calling CPU, for the padded count. */
void sh2z_dsp_xform_cpu(const int32_t m[3][4], const int32_t *in, int32_t *out, uint32_t count);

/* ---- D. Q16 outer product ---- */

#define SH2Z_DSP_SCALE16_MAX 16u

/** One job, read by the DSP's DMA (header and a, then b): keep it in work
 *  RAM, 4-byte aligned, and unchanged until sh2z_dsp_scale16_finish. */
typedef struct {
    uint32_t b_d0;      /* D0 word address of b (filled by prepare) */
    uint32_t out_d0;    /* D0 word address of the output */
    uint32_t n, m;
    int32_t a[SH2Z_DSP_SCALE16_MAX];
    int32_t b[SH2Z_DSP_SCALE16_MAX];
    int32_t *out;       /* CPU side only */
} sh2z_dsp_scale16_t;

extern const uint32_t sh2z_dsp_scale16_program[];
extern const uint32_t sh2z_dsp_scale16_program_words;
/** Data RAM bank and index of the job's D0 address word. */
#define SH2Z_DSP_SCALE16_JOB_BANK  3u
#define SH2Z_DSP_SCALE16_JOB_INDEX 0u

/** Load the kernel into program RAM. False in portable builds. */
bool sh2z_dsp_scale16_load(void);
/** Fill job: copies a (n values) and b (m values); out receives n * m
 *  words, 4-byte aligned, not overlapping job. False, job unchanged, when
 *  n or m is 0 or above SH2Z_DSP_SCALE16_MAX, a pointer is null, out is
 *  misaligned, or some |a[i] * b[k]| >= 2^47. */
bool sh2z_dsp_scale16_prepare(sh2z_dsp_scale16_t *job, const int32_t *a, uint32_t n,
    const int32_t *b, uint32_t m, int32_t *out);
/** Start the job (SH-2: kernel loaded, DSP idle; portable: computes it). */
void sh2z_dsp_scale16_start(const sh2z_dsp_scale16_t *job);
/** Wait (at most polls reads, 0: no limit) and purge the output's cache
 *  lines. True when the results are in out. */
bool sh2z_dsp_scale16_finish(const sh2z_dsp_scale16_t *job, uint32_t polls);
/** The same results on the calling CPU. */
void sh2z_dsp_scale16_cpu(const int32_t *a, uint32_t n, const int32_t *b, uint32_t m, int32_t *out);

#ifdef __cplusplus
}
#endif

#endif
