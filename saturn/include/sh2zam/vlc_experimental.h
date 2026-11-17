/* Experimental variable-length decoding of byte symbols for SH-2: a bounded
 * marker-bit reader over a 16-bit-unit stream, a length-limited canonical code
 * builder and a two-level table decoder with a hand-written SH-2 kernel.
 * Not selected by any default dispatch. MIT; SH2ZAM Contributors.
 *
 * Contract tag: vlc-1. The stream format, the table layout and the code
 * lengths sh2z_vlc8_build chooses are frozen at that tag. A faster path may
 * replace an implementation only if it produces identical bytes and reader
 * state; new behaviour gets a new function.
 *
 * The tables, the stream and the meaning of the symbols belong to the caller:
 * nothing here knows a file format. No function uses floating point, the
 * hardware divider, global state, libc or allocation; all are reentrant (the
 * caller owns the memory and the synchronisation).
 *
 * ---- Stream ----
 * A stream is `units` 16-bit units; unit k is the bytes s[2k] (bits 0..7) and
 * s[2k+1] (bits 8..15), so it is a little-endian bit stream on any CPU. Bits
 * are consumed least significant first, unit after unit. A code is written
 * first bit first: a code is the canonical (MSB-first) code word bit-reversed,
 * the usual order for LSB-first streams (as in DEFLATE).
 *
 * The decoder reads only the 2 * units bytes of the stream. Bits past the end
 * read as zero. A decode that consumes any such bit has overrun: the reader
 * remembers it and sh2z_vlc8_decode returns false from then on.
 *
 * ---- Code lengths (sh2z_vlc8_build) ----
 * Exactly, for the m symbols with a non-zero frequency:
 *  1. Huffman: repeatedly merge the two live nodes that are smallest by
 *     (weight, id); leaves have id = rank of the symbol among the m (so the
 *     lower symbol wins a tie, and any leaf beats any merged node), merged
 *     nodes get ids m, m+1, ... in creation order. A symbol's length is its
 *     depth (m = 1: length 1).
 *  2. Clamp every length to lmax. K = sum of 2^(lmax - len).
 *  3. While K > 2^lmax: of the symbols with len < lmax, take the one with the
 *     greatest len, then the smallest frequency, then the smallest symbol;
 *     len += 1.
 *  4. Repeat: of the symbols with len > 1 whose shortening keeps
 *     K + 2^(lmax - len) <= 2^lmax, take the one with the greatest frequency,
 *     then the smallest symbol; len -= 1. Stop when there is none.
 * The result is a complete code (K == 2^lmax) for m >= 2. When step 1 already
 * fits lmax it is a Huffman code (optimal); otherwise it is close to, but not
 * always, the optimal length-limited code (package-merge). VLC.md gives the
 * measured gap.
 *
 * ---- Canonical codes ----
 * Codes are assigned in order of (length, symbol), each length's first code
 * being (previous first + previous count) << 1, as in DEFLATE.
 *
 * ---- Table ----
 * w (1..8) bits index the first level. For entry e < 2^w:
 *  n[e] < 0x80: the code is s[e], its length n[e] / 4 + 1 (n[e] = 4*(len-1)).
 *  n[e] >= 0x80: the code is longer than w bits. xb = (n[e] >> 4) & 7 more
 *     bits (after the first w) index a group of 2^xb entries of pool[] that
 *     starts at ((n[e] & 15) << 8 | s[e]); a pool entry is
 *     symbol << 8 | 4 * (length - 1), the length counted from the first bit.
 * The lengths are stored as 4*(len-1) because the SH-2 kernel uses them as
 * byte offsets into a table of 2^(32-len) multipliers: the SH-2 has no
 * variable shift, and bb >> len is the high word of DMULU.L bb, 2^(32-len).
 * Entries s[e], n[e] for e >= 2^w are zero; pool[] entries at or after
 * pool_len are unspecified. The table is 2,564 bytes.
 *
 * ---- Reader ----
 * The unread bits sit in one 32-bit word with a marker bit above the last
 * valid bit (bb = bits | 1 << count), so at least 16 bits are left exactly
 * when bb >= 0x10000 and no count is kept. Between calls at least 16 bits are
 * buffered; when fewer are left after a symbol, one unit is appended (a zero
 * unit once the stream is exhausted). The reader therefore reads one unit
 * ahead of the last bit it has consumed, but never past the stream. */
#ifndef SH2ZAM_VLC_EXPERIMENTAL_H
#define SH2ZAM_VLC_EXPERIMENTAL_H

#include "sh2zam.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Contract tag vlc-1 --------------------------------------------------- */

/** Longest code length the table format holds. */
#define SH2Z_VLC8_LMAX_LIMIT 16u
/** Second-level pool capacity, in entries. */
#define SH2Z_VLC8_POOL_MAX 1024u

/** Decoding table for byte symbols (layout above; 2,564 bytes, 2-byte
 *  aligned). Built by sh2z_vlc8_build or sh2z_vlc8_build_lengths, or loaded
 *  by the caller and accepted by sh2z_vlc8_check. The SH-2 kernel reads it at
 *  these offsets: w 0, lmax 1, pool_len 2, s 4, n 260, pool 516. */
typedef struct {
    uint8_t w;          /* first-level window, 1..8 */
    uint8_t lmax;       /* length limit the table was built with, w..16 */
    uint16_t pool_len;  /* pool[] entries in use */
    uint8_t s[256];
    uint8_t n[256];
    uint16_t pool[SH2Z_VLC8_POOL_MAX];
} sh2z_vlc8_t;

/** Bit reader over one stream. Set up by sh2z_vlc_reader_init and then only
 *  advanced by the decoders; callers may read but must not write it. pad
 *  counts zero units appended past the end of the stream, saturating at 2.
 *  On SH-2 the kernel reads it at offsets p 0, bb 4, end 8, pad 12. */
typedef struct {
    const uint8_t *p;    /* next unit to read */
    uint32_t bb;         /* unread bits with the marker bit above them */
    const uint8_t *end;  /* one past the last unit */
    uint32_t pad;
} sh2z_vlc_reader_t;

/** Start reading a stream of `units` 16-bit units at `stream` (2 * units
 *  readable bytes; stream may be null when units is 0). Reads the first unit
 *  (none when units is 0). For the SH-2 kernel the stream should be 2-byte
 *  aligned; an odd address is correct but takes the portable path.
 *  units must not exceed SIZE_MAX / 2. */
void sh2z_vlc_reader_init(sh2z_vlc_reader_t *r,const void *stream,size_t units);

/** True while no bit past the end of the stream has been consumed. Once
 *  false it stays false (until the reader is initialised again). */
bool sh2z_vlc_reader_ok(const sh2z_vlc_reader_t *r);

/** Build a code and its table from symbol frequencies freq[0..255].
 *
 *  A symbol with frequency 0 gets no code and must not be encoded. Code
 *  lengths follow the exact rule at the top of this header; codes are
 *  canonical. len_out[256] (0 = no code) and code_out[256] (the code, first
 *  stream bit in bit 0, for an encoder) may be null.
 *
 *  Requires 1 <= w <= 8, w <= lmax <= 16, and between 1 and 2^lmax symbols
 *  with a non-zero frequency. Returns false if an argument is out of range,
 *  or if a second-level group would need more than 7 extra bits (a code
 *  longer than w + 7, so a table never holds a 16-bit code; pass lmax <= 15
 *  or lmax <= w + 7 to avoid that refusal) or the pool would exceed
 *  SH2Z_VLC8_POOL_MAX entries;
 *  *t, len_out and code_out are then unspecified. With exactly one symbol the
 *  code is one bit long and every bit pattern decodes to that symbol.
 *  Uses about 5 KB of stack. */
bool sh2z_vlc8_build(sh2z_vlc8_t *t,const uint32_t freq[256],unsigned w,
    unsigned lmax,uint8_t len_out[256],uint16_t code_out[256]);

/** Build the table for given code lengths len[0..255] (0 = no code), for
 *  example lengths read from a stream. Codes are canonical; code_out may be
 *  null. Accepts only lengths <= lmax that form a complete prefix code
 *  (sum of 2^-len == 1), or exactly one symbol of length 1. Returns false
 *  otherwise, with the conditions of sh2z_vlc8_build; *t and code_out are
 *  then unspecified. sh2z_vlc8_build(freq) builds the same table as this
 *  function on the lengths it reports. */
bool sh2z_vlc8_build_lengths(sh2z_vlc8_t *t,const uint8_t len[256],unsigned w,
    unsigned lmax,uint16_t code_out[256]);

/** Check that a table is safe to decode with: 1 <= w <= 8,
 *  pool_len <= SH2Z_VLC8_POOL_MAX, every first-level and reachable pool
 *  length field is a multiple of 4 and at most 60 (length 1..16), and every
 *  group lies inside pool[0..pool_len). Every built table passes. A table
 *  that passes decodes without reading or writing out of bounds, but need
 *  not be a prefix code. Use it on tables loaded from untrusted data. */
bool sh2z_vlc8_check(const sh2z_vlc8_t *t);

/** Decode n symbols to dst[0..n).
 *
 *  t must have been built by sh2z_vlc8_build[_lengths] or accepted by
 *  sh2z_vlc8_check; r must have been set up by sh2z_vlc_reader_init (and may
 *  have been advanced by earlier calls: a decode in several calls gives the
 *  same bytes and reader state as one call). dst must not overlap t, r or the
 *  stream; it may be null when n is 0.
 *
 *  For any stream contents and any n: reads only t, *r and the stream's
 *  2 * units bytes, writes exactly dst[0..n) and *r, and takes O(n) time.
 *  Returns sh2z_vlc_reader_ok(r) after the call: false if this or an earlier
 *  call consumed a bit past the end of the stream (a truncated or malformed
 *  stream). All n bytes are written either way, each a symbol of the table.
 *
 *  On SH-2 this runs the hand-written kernel (same bytes, same reader state)
 *  unless SH2Z_FORCE_PORTABLE is set or the stream address is odd. It
 *  preserves r8-r14, PR, MACH and MACL and does not touch r14 or GBR. */
bool sh2z_vlc8_decode(const sh2z_vlc8_t *t,sh2z_vlc_reader_t *r,uint8_t *dst,
    size_t n);

/** The portable C decoder: the same contract and results as
 *  sh2z_vlc8_decode on every target. */
bool sh2z_vlc8_decode_portable(const sh2z_vlc8_t *t,sh2z_vlc_reader_t *r,
    uint8_t *dst,size_t n);

#ifdef __cplusplus
}
#endif
#endif
