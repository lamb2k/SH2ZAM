/* Experimental expansion of delta coded multi tile sprite records into cell
 * list entries. Game-neutral: the library knows nothing about the title, the
 * renderer or the memory map the records came from.
 *
 * Contract tag cell-records-1.
 *
 * Model. A sprite is a list of count records. Each record is one 32-bit word
 * (a tile code in the high half, an attribute in the low half) plus one pair
 * of offsets. The offsets are deltas: every cell moves the running position
 * (x, y) by its pair, so a sprite is a short chain, not a table of absolute
 * positions. The kernel turns the chain into count entries of 8 bytes, a
 * position word and an attribute word, which a caller hands on to whatever
 * draws cells. Calls cover 4 to 40 cells; both entry points have no setup cost
 * beyond loading the parameter block, and the trusted one checks nothing.
 *
 * Per cell, in this order (all position arithmetic wraps modulo 2^16):
 *   attribute   KEEP: a = record.
 *               ATTR_REPLACE: a = (record & attr_keep) | attr_or.
 *               ATTR_ADD_BYTE: the low byte of the record plus attr_or, carrying
 *               nowhere (the upper 24 bits are kept).
 *               MIRROR_X: then a ^= 0x20 (the horizontal flip bit).
 *   position    no mirror: x += dx, y += dy; the entry's x is x.
 *               MIRROR_X:  x -= dx, y += dy; the entry's x is x - w - mirror_bias,
 *               where w = (a >> 4) & 0xf0 is the cell's own width in pixels
 *               (attribute bits 8 to 11, in units of 16).
 *   entry       position word = ((x << 16 | y) & pos_mask) | pos_or, then a.
 * The running position pos is read and written back after the last cell; only
 * its low 16 bits matter. Offsets are pairs of signed bytes (dx, dy) or, with
 * SH2Z_CELLREC_S16, of signed 16-bit words.
 *
 * Byte order. Records, offsets and entries are big-endian byte streams, the
 * layout the SH-2 and the 68000 data they come from share, read and written
 * with byte-order-neutral loads; every host gives the same entries. Records and
 * offsets must be 2-byte aligned and entries 4-byte aligned.
 *
 * Exactness. The kernel is integer-only and exact: the host tests compare every
 * entry with a per-cell model and the position state after every call, and the
 * SH-2 instruction model checks the same on executed object code.
 *
 * SH-2: compiled C, no multiplies, no divides.
 *
 * Copyright (c) 2026 SH2ZAM Contributors. MIT License.
 * Inspired by SH4ZAM by Falco Girgis and SH4ZAM Contributors.
 */
#ifndef SH2ZAM_CELL_RECORDS_EXPERIMENTAL_H
#define SH2ZAM_CELL_RECORDS_EXPERIMENTAL_H
#include "sh2zam.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Contract tag cell-records-1 --------------------------------------- */

/** Form bits of sh2z_cellrec_params_t.form. ATTR_REPLACE and ATTR_ADD_BYTE
 *  exclude each other; no bit outside this set may be given. */
#define SH2Z_CELLREC_S16 1u
#define SH2Z_CELLREC_MIRROR_X 2u
#define SH2Z_CELLREC_ATTR_REPLACE 4u
#define SH2Z_CELLREC_ATTR_ADD_BYTE 8u
#define SH2Z_CELLREC_MAX_COUNT 65536u

/** The running position: the low 16 bits of each field are used, the upper
 *  bits are ignored on input and written as zero. */
typedef struct { uint32_t x, y; } sh2z_cellrec_pos_t;

/** What to do with a list; see the file comment. 20 bytes. */
typedef struct {
    uint32_t form;          /* SH2Z_CELLREC_* bits */
    uint32_t pos_mask;      /* mask of the packed position word */
    uint32_t pos_or;        /* bits OR-ed into the packed position word */
    uint32_t mirror_bias;   /* subtracted from the mirrored x (low 16 bits) */
    uint32_t attr_keep;     /* ATTR_REPLACE: bits of the record kept */
    uint32_t attr_or;       /* ATTR_REPLACE: bits OR-ed in; ATTR_ADD_BYTE: the byte added */
} sh2z_cellrec_params_t;

/** Expand count records into count entries. offsets holds count pairs (2 or 4
 *  bytes a pair). Returns false and changes nothing when an argument breaks the
 *  contract: a null pointer (for count > 0), a misaligned stream, count above
 *  SH2Z_CELLREC_MAX_COUNT, an unknown or contradictory form bit, or an output
 *  that overlaps an input or the position. count 0 succeeds and writes nothing. */
bool sh2z_cellrec_expand(sh2z_cellrec_pos_t *pos, const sh2z_cellrec_params_t *params,
    const uint8_t *records, const uint8_t *offsets, uint8_t *out, size_t count);

/** The same without any check: the caller guarantees the contract above (one
 *  validated parameter block for many calls). count may be 0. */
void sh2z_cellrec_expand_trusted(sh2z_cellrec_pos_t *pos, const sh2z_cellrec_params_t *params,
    const uint8_t *records, const uint8_t *offsets, uint8_t *out, size_t count);

#ifdef __cplusplus
}
#endif
#endif
