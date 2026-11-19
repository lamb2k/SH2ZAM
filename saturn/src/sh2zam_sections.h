/* Private to the library sources: section placement of read-only tables.
 * SH2ZI_RODATA(name) on a table's definition puts it in its own section,
 * .rodata.<name>, with or without -fdata-sections, so that --gc-sections can
 * drop a table nothing calls into, and a consumer's linker script can place a
 * cold table, for example in low work RAM:
 *     .lwram_tables : { *(.rodata.sh2z_sin12_quarter_u30) } > lwram
 * Hand-written assembly does the same for its code and tables with
 * .pushsection .text.<symbol> and .rodata.<symbol> (CONTRIBUTING-KERNELS.md).
 *
 * Copyright (c) 2026 SH2ZAM Contributors. MIT License.
 * Inspired by SH4ZAM by Falco Girgis and SH4ZAM Contributors.
 */
#ifndef SH2ZAM_SECTIONS_H
#define SH2ZAM_SECTIONS_H
#if defined(__GNUC__) && defined(__ELF__)
#define SH2ZI_RODATA(name) __attribute__((section(".rodata." #name)))
#else
#define SH2ZI_RODATA(name)
#endif
#endif
