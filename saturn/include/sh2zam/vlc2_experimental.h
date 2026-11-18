/* Experimental per-symbol variable-length decoding for SH-2: a decode step
 * that keeps the bit reader in the caller's registers and takes a table
 * pointer per symbol, compact byte tables that pack together in one arena,
 * and a reader that starts at any byte offset of the caller's stream.
 * Not selected by any default dispatch. MIT; SH2ZAM Contributors.
 *
 * Contract tag: vlc-2. The stream format, the table layout, the reader state
 * and the constant block are frozen at that tag. A faster path may replace an
 * implementation only if it gives identical symbols and reader state; new
 * behaviour gets a new function. vlc-1 (vlc_experimental.h) is unchanged and
 * independent of this header.
 *
 * Use: a decoder whose table changes from symbol to symbol, chosen from
 * output it has already decoded (context modelling), where one call per
 * symbol of a batch decoder would pay the call and the reader set-up every
 * time. The caller keeps an sh2z_vlc2_reader_t in a local variable, computes
 * a table pointer for each symbol and calls sh2z_vlc2_decode, which is
 * inlined; on SH-2 it is hand-written assembly and the reader stays in
 * registers. The caller writes the output. VLC2.md shows a decode loop.
 *
 * Nothing here knows a file format. No function uses floating point, the
 * hardware divider, global state, libc or allocation; all are reentrant.
 *
 * ---- Stream ----
 * The same bits as vlc-1: a byte stream whose bits are consumed least
 * significant bit first, byte after byte (so vlc-1's 16-bit little-endian
 * units are the same stream). A code is written first bit first: the
 * canonical (MSB-first) code word bit-reversed, as in DEFLATE. A stream may
 * start at any byte offset of the caller's buffer; nothing is copied.
 *
 * ---- Codes and tables ----
 * Symbols are 0..nsym-1 (nsym <= 256) with code lengths 0 (no code) or
 * 1..SH2Z_VLC2_LMAX. The lengths must form a complete prefix code (the sum of
 * 2^-len is 1), or be exactly one symbol of length 1 (both bit values decode
 * to it). Codes are canonical: assigned in order of (length, symbol), each
 * length's first code being (previous first + previous count) << 1.
 *
 * A table for window w (1..SH2Z_VLC2_WMAX) is a byte array with no alignment
 * requirement, so tables pack back to back in one arena:
 *   t[e]          symbol of first-level entry e, e < 2^w (the next w bits)
 *   t[2^w + e]    its field: 4 * len for a code of len <= w bits, or
 *                 64 + 4 * xb when the codes with these first w bits are
 *                 longer: then t[e] = g and their group of 2^xb entries
 *                 starts at byte 4 * g; entry k of the group (k = the xb
 *                 bits after the first w) is 2 bytes: symbol, then 4 * len,
 *                 the length counted from the first bit.
 *   t[2^(w+1) ..] the groups (the second-level pool), largest groups first,
 *                 ties by ascending e; within a group, entries are filled as
 *                 in the first level (a code of len < w + xb fills every
 *                 2^(len-w)-th entry).
 * sh2z_vlc2_build makes exactly these bytes. A table is
 * 2^(w+1) + 2 * (pool entries) bytes; with w = 6 a code without long codes
 * is 128 bytes.
 *
 * ---- Reader ----
 * bits holds the unread bits (next bit in bit 0) and cnt = 0x7FFFFFC0 +
 * 4 * count (mod 2^32) for count unread bits. Between symbols
 * 16 <= count <= 31. After a symbol, if fewer than 16 bits are left, one
 * aligned 16-bit unit is appended (the reader is 2-byte aligned after
 * sh2z_vlc2_begin). No end-of-stream test is made per symbol: see
 * sh2z_vlc2_reach. p is the next byte to read; k points at
 * sh2z_vlc2_consts. Callers may read but must not write the fields.
 *
 * ---- Bounds, once per stream ----
 * Started at byte `offset` of a buffer, a reader that has consumed B bits
 * has read exactly bytes [offset, sh2z_vlc2_reach(offset, B)) of it. A
 * symbol consumes at most L bits, where L is the longest code length of the
 * tables used (sh2z_vlc2_check returns it). So decoding at most S symbols
 * reads nothing at or past sh2z_vlc2_reach(offset, S * L), whatever the
 * stream contains: the caller tests that once per stream against its buffer
 * size, and then needs no test per symbol. A well-formed stream of E bits
 * reads at most 2 bytes past its last byte. sh2z_vlc2_bitpos tells how many
 * bits were consumed, for a truncation test after the stream. */
#ifndef SH2ZAM_VLC2_EXPERIMENTAL_H
#define SH2ZAM_VLC2_EXPERIMENTAL_H

#include "sh2zam.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Contract tag vlc-2 --------------------------------------------------- */

/** Longest code length the table format holds. */
#define SH2Z_VLC2_LMAX 15u
/** Largest first-level window. */
#define SH2Z_VLC2_WMAX 7u

/** The constant block the decoder reads (256 bytes, 4-byte aligned): entry i
 *  is 2^(32-i) for i = 1..15 (bits >> len is the high word of the 64-bit
 *  product bits * 2^(32-len)), 1 for i = 16..31 (an escape field selects one
 *  of these), 2^(i-32) - 1 for i = 32..47 and 2^(i-48) for i = 48..63;
 *  entry 0 is 0. */
extern const uint32_t sh2z_vlc2_consts[64];

/** Bit reader (layout and invariants above). Keep it in a local variable so
 *  the compiler can hold its fields in registers. */
typedef struct {
    uint32_t bits;          /* unread bits, next bit in bit 0 */
    uint32_t cnt;           /* 0x7FFFFFC0 + 4 * (unread bit count) */
    const uint8_t *p;       /* next byte to read; even after sh2z_vlc2_begin */
    const uint32_t *k;      /* sh2z_vlc2_consts */
} sh2z_vlc2_reader_t;

/** Start reading at stream[offset]. Reads bytes offset, offset + 1 and, when
 *  stream + offset is odd, offset + 2 (that is, up to
 *  sh2z_vlc2_reach(offset, 0) when stream is 2-byte aligned). */
static inline void sh2z_vlc2_begin(sh2z_vlc2_reader_t *r,const void *stream,size_t offset) {
    const uint8_t *p=(const uint8_t *)stream+offset;
    uint32_t count=16u;
    uint32_t bits=(uint32_t)p[0]|(uint32_t)p[1]<<8;
    if((uintptr_t)p&1u) {
        bits|=(uint32_t)p[2]<<16;
        count=24u;
        p+=3;
    } else {
        p+=2;
    }
    r->bits=bits;
    r->cnt=UINT32_C(0x7FFFFFC0)+4u*count;
    r->p=p;
    r->k=sh2z_vlc2_consts;
}

/** One past the last byte (counted from the start of a 2-byte-aligned
 *  buffer) that a reader begun at `offset` has read once it has consumed
 *  `bits` bits: 2 * ceil((8 * offset + bits + 16) / 16). For a buffer that
 *  is not 2-byte aligned, count offset from the aligned address below it.
 *  Requires 8 * offset + bits + 31 to fit in size_t. */
static inline size_t sh2z_vlc2_reach(size_t offset,size_t bits) {
    return ((8u*offset+bits+31u)>>4)*2u;
}

/** The number of bits consumed since stream[0]: 8 * (bytes read) - unread,
 *  with stream the pointer given to sh2z_vlc2_begin. A stream that should
 *  have ended at bit E was truncated or malformed if this exceeds E. */
static inline size_t sh2z_vlc2_bitpos(const sh2z_vlc2_reader_t *r,const void *stream) {
    return 8u*(size_t)(r->p-(const uint8_t *)stream)-(size_t)((r->cnt-UINT32_C(0x7FFFFFC0))>>2);
}

/** Portable definition of sh2z_vlc2_decode: same symbols and reader state
 *  on every target. */
static inline uint32_t sh2z_vlc2_decode_portable(sh2z_vlc2_reader_t *r,const uint8_t *t,unsigned w) {
    const uint32_t bits=r->bits;
    const uint32_t e=bits&((UINT32_C(1)<<w)-1u);
    uint32_t sym=t[e],n=t[e|(UINT32_C(1)<<w)];
    if(n>=64u) {
        const uint32_t xb=(n-64u)>>2;
        const uint32_t g=4u*sym+2u*((bits>>w)&((UINT32_C(1)<<xb)-1u));
        sym=t[g];
        n=t[g+1u];
    }
    uint32_t c4=r->cnt-UINT32_C(0x7FFFFFC0)-n;   /* 4 * bits left */
    uint32_t left=bits>>(n>>2);
    if(c4<64u) {
        const uint8_t *p=r->p;
        left|=((uint32_t)p[0]|(uint32_t)p[1]<<8)<<(c4>>2);
        r->p=p+2;
        c4+=64u;
    }
    r->bits=left;
    r->cnt=c4+UINT32_C(0x7FFFFFC0);
    return sym;
}

#if defined(__sh__) && defined(__SH2__) && (!defined(SH2Z_FORCE_PORTABLE) || !SH2Z_FORCE_PORTABLE)
#define SH2ZI_VLC2_NATIVE 1
#else
#define SH2ZI_VLC2_NATIVE 0
#endif

/** Decode one symbol with table t (window w) and return it (0..255).
 *
 *  t must have been built by sh2z_vlc2_build with this w, or accepted by
 *  sh2z_vlc2_check(t, size, w); r must have been set up by sh2z_vlc2_begin
 *  and only advanced by sh2z_vlc2_decode[_portable] since. Reads t, the
 *  constant block and the stream (see the bounds rule above); writes only
 *  *r. w must be a constant expression (an immediate operand on SH-2;
 *  compile with optimisation). Uses no stack and calls nothing.
 *
 *  On SH-2 (unless SH2Z_FORCE_PORTABLE) this is inline assembly: a symbol
 *  whose code is at most w bits takes 12 instructions (13 cycles in the
 *  instruction model) plus 11 more when it leaves fewer than 16 bits; a
 *  longer code takes a second-level step of about 30 more cycles. It needs
 *  r0 and two scratch registers besides the reader's three and t, and
 *  clobbers MACH, MACL and T. */
#if defined(__GNUC__)
__attribute__((always_inline))
#endif
static inline uint32_t sh2z_vlc2_decode(sh2z_vlc2_reader_t *r,const uint8_t *t,unsigned w) {
#if SH2ZI_VLC2_NATIVE
    uint32_t sym,f,x;
    /* Fast path: index the first level with the low w bits, load the symbol
     * and the field n = 4 * len, shift the bits right by len (the high word
     * of DMULU.L bits, 2^(32-len)) and subtract n from cnt with SUBV, which
     * sets T exactly when fewer than 16 bits are left (cnt is biased so that
     * count < 16 is a signed underflow). The delay slot zero-extends the
     * symbol on both paths.
     * Slow path, T = 1: an escape field (64 + 4 * xb) selects a constant of
     * 1 and always underflows, so bit 6 of n tells the two cases apart.
     * Refill: the low byte of cnt is now 0xC0 + 4 * count, the offset of
     * 2^count in the constant block; append the next 16-bit unit (MOV.W reads
     * it byte-swapped, SWAP.B restores stream order) at bit count.
     * Escape: undo the step (MACL still holds the bits, cnt gets n back),
     * index the group with the next xb bits and redo the step with the
     * group entry's symbol and field. */
    __asm__(
        "mov     %[b],r0\n\t"
        "and     #%O[m],r0\n\t"
        "mov.b   @(r0,%[t]),%[s]\n\t"
        "or      #%O[h],r0\n\t"
        "mov.b   @(r0,%[t]),r0\n"
        "1:\n\t"
        "mov.l   @(r0,%[k]),%[f]\n\t"
        "dmulu.l %[f],%[b]\n\t"
        "sts     mach,%[b]\n\t"
        "subv    r0,%[c]\n\t"
        "bf/s    3f\n\t"
        "extu.b  %[s],%[s]\n\t"
        "tst     #64,r0\n\t"
        "bf      2f\n\t"
        "extu.b  %[c],r0\n\t"
        "mov.l   @(r0,%[k]),%[f]\n\t"
        "mov.w   @%[p]+,r0\n\t"
        "swap.b  r0,r0\n\t"
        "mulu.w  %[f],r0\n\t"
        "add     #64,%[c]\n\t"
        "sts     macl,r0\n\t"
        "bra     3f\n\t"
        "or      r0,%[b]\n"
        "2:\n\t"
        "sts     macl,%[b]\n\t"
        "add     r0,%[c]\n\t"
        "add     #64,r0\n\t"
        "mov.l   @(r0,%[k]),%[f]\n\t"
        "mov.l   @(%O[w4],%[k]),r0\n\t"
        "dmulu.l r0,%[b]\n\t"
        "mov     %[s],r0\n\t"
        "shll2   r0\n\t"
        "sts     mach,%[s]\n\t"
        "and     %[f],%[s]\n\t"
        "add     %[s],r0\n\t"
        "add     %[s],r0\n\t"
        "mov.b   @(r0,%[t]),%[s]\n\t"
        "add     #1,r0\n\t"
        "bra     1b\n\t"
        "mov.b   @(r0,%[t]),r0\n"
        "3:"
        : [b] "+r" (r->bits),[c] "+r" (r->cnt),[p] "+r" (r->p),
          [s] "=&r" (sym),[f] "=&r" (f),"=&z" (x)
        : [t] "r" (t),[k] "r" (r->k),
          [m] "n" ((1u<<w)-1u),[h] "n" (1u<<w),[w4] "n" (4u*w)
        : "mach","macl","t","memory");
    (void)x;
    return sym;
#else
    return sh2z_vlc2_decode_portable(r,t,w);
#endif
}

/** Bytes of the table sh2z_vlc2_build would make for these lengths, or 0
 *  if it would refuse them (same conditions). */
size_t sh2z_vlc2_table_size(const uint8_t *len,unsigned nsym,unsigned w);

/** Build the table for code lengths len[0..nsym) into t[0..capacity).
 *
 *  Requires 1 <= nsym <= 256, 1 <= w <= SH2Z_VLC2_WMAX and lengths that are
 *  a complete prefix code with every length <= SH2Z_VLC2_LMAX, or exactly
 *  one symbol of length 1. Refuses (returns 0, writes nothing) otherwise, or
 *  when a group would start past byte 1020 (4 * 255; at w = 6 that leaves
 *  about 450 pool entries) or the table is larger than capacity. Returns the
 *  table's size in bytes. code_out[0..nsym), if not null, receives each
 *  symbol's code with its first stream bit in bit 0 (for an encoder; 0 for
 *  symbols without a code). Uses about 800 bytes of stack. */
size_t sh2z_vlc2_build(uint8_t *t,size_t capacity,const uint8_t *len,unsigned nsym,
    unsigned w,uint16_t *code_out);

/** Check that the size bytes at t are safe to decode with window w, and
 *  return the longest length any decode step can consume (1..15), or 0 if
 *  the table is unsafe: w out of range, size < 2^(w+1), a first-level field
 *  that is neither 4 * len (len 1..15) nor 64 + 4 * xb (w + xb <= 15), a
 *  group not inside t[0..size), or a group entry field that is not
 *  4 * len (len 1..15). Every built table passes and returns its longest
 *  code length. A table that passes decodes without reading outside
 *  t[0..size) but need not be a prefix code. Use it on tables loaded from
 *  untrusted data. */
unsigned sh2z_vlc2_check(const uint8_t *t,size_t size,unsigned w);

#ifdef __cplusplus
}
#endif
#endif
