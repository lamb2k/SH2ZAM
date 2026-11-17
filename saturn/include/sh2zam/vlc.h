/* Variable-length symbol decoding for byte alphabets: marker-bit bit reader and a two-level table decoder.
 * MIT; Copyright (c) 2026 SH2ZAM Contributors. Game-neutral: the tables, the stream and the meaning of the
 * symbols belong to the caller. Nothing here knows a file format, a tile or a palette.
 *
 * Stream: bits are packed least significant first in 16-bit units; unit k is the two bytes p[2k] (bits 0..7)
 * and p[2k+1] (bits 8..15), so the stream is a little-endian bit stream whatever the CPU byte order. The reader
 * keeps the unread bits in a 32-bit buffer `bb` with a marker bit above the last valid bit (bb = data | 1 << nb):
 * bb >= 0x10000 means at least 16 valid bits are left, so no bit count is stored and the refill test is one
 * compare. A refill appends one 16-bit unit. After init the reader has read 4 bytes; a decode of the last symbol
 * may read up to 4 bytes beyond the bits it needs, so the caller pads the stream with 4 zero bytes (the encoder
 * helper in the tests does the same).
 *
 * Table: canonical prefix code, code length 1..lmax (lmax <= 16), codes written least significant bit first
 * (the first stream bit is the first code bit, i.e. the code is bit-reversed, the usual order for LSB-first
 * streams). First level: the low `w` bits of the buffer index two byte arrays, s[] and n[]. n[e] < 0x80:
 * the symbol is s[e] and its code length is n[e] / 4 + 1. n[e] >= 0x80: the code is longer than w bits;
 * xb = (n[e] >> 4) & 7 more bits (taken after the first w bits) index a group of 2^xb u16 entries of pool[]
 * starting at ((n[e] & 15) << 8 | s[e]); a pool entry is (symbol << 8 | 4 * (length - 1)), the length counted
 * from the first bit. The split of the symbol and the length into two byte arrays lets an SH-2 load each with
 * one mov.b and turn the length into a multiplier for a right shift (the SH-2 has no variable shift: bb >> len
 * is the high word of the product bb * 2^(32 - len), dmulu.l with a table of 16 multipliers).
 *
 * All functions are allocation-free and reentrant; the caller owns the memory and the synchronisation. */
#ifndef SH2ZAM_VLC_H
#define SH2ZAM_VLC_H
#include "sh2zam.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SH2Z_VLC8_POOL_MAX 1024u

typedef struct {
    uint8_t w;                           /* first level window, 1..8 */
    uint8_t lmax;                        /* longest code, w..16 */
    uint16_t pool_len;                   /* entries of pool[] in use */
    uint8_t s[256];                      /* first level, 2^w entries used */
    uint8_t n[256];
    uint16_t pool[SH2Z_VLC8_POOL_MAX];
} sh2z_vlc8_t;

typedef struct { const uint8_t *p; uint32_t bb; } sh2z_bitr_t;

/* Start reading at p (at least 4 readable bytes). */
void sh2z_bitr_init(sh2z_bitr_t *r, const uint8_t *p);

/* Build a code for the symbol frequencies freq[0..255] (a symbol with frequency 0 gets no code and must not be
 * encoded; at least one symbol must have a non-zero frequency; with exactly one the code is one bit long).
 * Code lengths: Huffman lengths limited to lmax by rebalancing the Kraft sum (valid and prefix free, close to
 * optimal, not guaranteed optimal). len_out[256] (0 = no code) and code_out[256] (the code, least significant bit
 * first, for the encoder) are optional (may be NULL). Returns false if the arguments are out of range or the
 * second level pool would exceed SH2Z_VLC8_POOL_MAX; the table is then unspecified. */
bool sh2z_vlc8_build(sh2z_vlc8_t *t, const uint32_t freq[256], unsigned w, unsigned lmax, uint8_t len_out[256], uint16_t code_out[256]);

/* Decode n symbols to dst. Portable C reference (the SH-2 kernel in sh2zam_vlc_sh2.S gives the same bytes). */
void sh2z_vlc8_decode(const sh2z_vlc8_t *t, sh2z_bitr_t *r, uint8_t *dst, size_t n);

#ifdef __cplusplus
}
#endif
#endif
