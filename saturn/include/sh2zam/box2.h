/* Exact 2D integer box overlap, scalar and batched with packed bitmask results.
 * MIT; Copyright (c) 2026 SH2ZAM Contributors. Game-neutral: the library knows
 * nothing about what a box means (hitbox, hurtbox, trigger, tile range).
 *
 * Two box layouts, both int32 per field and both exact over the full int32 range
 * (no hidden overflow; every comparison is done on exact values):
 *   sh2z_ibox2_t   min/max:       covers x in [min_x,max_x], y in [min_y,max_y]
 *   sh2z_ibox2c_t  centre + half: covers x in [cx-hx,cx+hx], y in [cy-hy,cy+hy]
 * Preconditions: min <= max (resp. half-sizes >= 0) on both axes of every box.
 * Boxes that violate them produce deterministic results defined only by the
 * formulas below, identical on every path, with no further meaning.
 *
 * Edge rule (sh2z_box_edges_t), applied to both axes:
 *   SH2Z_BOX_INCLUSIVE  closed rectangles: boxes that only share an edge or a
 *                       corner DO overlap.
 *     min/max: a.min_x <= b.max_x && b.min_x <= a.max_x && same for y
 *     centre:  |a.cx - b.cx| <= a.hx + b.hx && same for y
 *   SH2Z_BOX_EXCLUSIVE  interiors must intersect: boxes that only touch do NOT
 *                       overlap (the usual rule for half-open pixel ranges).
 *     min/max: a.min_x <  b.max_x && b.min_x <  a.max_x && same for y
 *     centre:  |a.cx - b.cx| <  a.hx + b.hx && same for y
 *   (min/max form: a zero-width box is overlapped by exclusive boxes that
 *   strictly contain its position, as the formula says. The centre form
 *   evaluates |dc| and the half-size sum as unsigned 32-bit values, which is
 *   exact for any int32 centres and half-sizes >= 0.)
 *
 * Bitset results: bit (i & 31) of word (i >> 5) is the result for box i; the
 * first ceil(count/32) words are written, unused bits of the last word are 0,
 * additional capacity is untouched (same layout as sh2z_aabb_index_query_sphere).
 * Many-vs-many writes one row per a-box: row r starts at word r*row_words, where
 * row_words = sh2z_box_bitset_words(b_count); bit j of row r is a[r] vs b[j].
 * hit_count / pair_count are optional (null to skip) and receive the number of
 * set bits on success.
 *
 * Argument rules (checked before any output write; failure returns false and
 * leaves every output unchanged): counts <= 2^31; non-empty arrays need a
 * nonnull, 4-byte-aligned pointer with a representable byte length; the output
 * needs enough capacity and must not overlap an input array or the count
 * output; empty inputs permit null pointers. All functions are reentrant,
 * allocation-free and use no global state; the caller owns synchronization and
 * cache/DMA coherency of the buffers. SH-2: compiled C, no MAC registers, no
 * hardware divider, no SR bits other than T. */
#ifndef SH2ZAM_BOX2_H
#define SH2ZAM_BOX2_H
#include "sh2zam.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct { int32_t min_x,min_y,max_x,max_y; } sh2z_ibox2_t;
typedef struct { int32_t cx,cy,hx,hy; } sh2z_ibox2c_t;
typedef enum { SH2Z_BOX_INCLUSIVE=0, SH2Z_BOX_EXCLUSIVE=1 } sh2z_box_edges_t;

/** Words needed for a bitset of count boxes: ceil(count/32). */
static inline size_t sh2z_box_bitset_words(size_t count) { return count/32u+((count&31u)!=0u); }

/** Scalar tests; see the edge rule above. */
static inline bool sh2z_ibox2_overlap(const sh2z_ibox2_t *a,const sh2z_ibox2_t *b,sh2z_box_edges_t edges) {
    if(edges==SH2Z_BOX_EXCLUSIVE)
        return a->min_x<b->max_x&&b->min_x<a->max_x&&a->min_y<b->max_y&&b->min_y<a->max_y;
    return a->min_x<=b->max_x&&b->min_x<=a->max_x&&a->min_y<=b->max_y&&b->min_y<=a->max_y;
}
static inline bool sh2z_ibox2c_overlap(const sh2z_ibox2c_t *a,const sh2z_ibox2c_t *b,sh2z_box_edges_t edges) {
    uint32_t dx=(uint32_t)a->cx-(uint32_t)b->cx,dy=(uint32_t)a->cy-(uint32_t)b->cy;
    if(a->cx<b->cx) dx=0u-dx;
    if(a->cy<b->cy) dy=0u-dy;
    const uint32_t sx=(uint32_t)a->hx+(uint32_t)b->hx,sy=(uint32_t)a->hy+(uint32_t)b->hy;
    return edges==SH2Z_BOX_EXCLUSIVE ? dx<sx&&dy<sy : dx<=sx&&dy<=sy;
}

/** One query box against count boxes. */
bool sh2z_ibox2_overlap_one_many(uint32_t *hits,size_t hit_word_capacity,size_t *hit_count,
    const sh2z_ibox2_t *query,const sh2z_ibox2_t *boxes,size_t count,sh2z_box_edges_t edges);
bool sh2z_ibox2c_overlap_one_many(uint32_t *hits,size_t hit_word_capacity,size_t *hit_count,
    const sh2z_ibox2c_t *query,const sh2z_ibox2c_t *boxes,size_t count,sh2z_box_edges_t edges);
/** Trusted-buffer form of one_many for hot loops over small batches: no argument
 *  validation (which costs about as much as testing a dozen boxes on SH-2) and no
 *  result-count check. The caller guarantees: query and boxes are valid, aligned
 *  and non-null for count >= 1; hits points to sh2z_box_bitset_words(count)
 *  writable, aligned words that do not overlap query or boxes; edges is one of the
 *  two enumerators. Writes exactly those words, returns the number of set bits.
 *  Results are identical to the checked function. */
size_t sh2z_ibox2_overlap_one_many_unchecked(uint32_t *hits,const sh2z_ibox2_t *query,
    const sh2z_ibox2_t *boxes,size_t count,sh2z_box_edges_t edges);
size_t sh2z_ibox2c_overlap_one_many_unchecked(uint32_t *hits,const sh2z_ibox2c_t *query,
    const sh2z_ibox2c_t *boxes,size_t count,sh2z_box_edges_t edges);
/** a_count x b_count matrix, one bitset row per a box (see the layout above). */
bool sh2z_ibox2_overlap_many_many(uint32_t *bits,size_t bit_word_capacity,size_t *pair_count,
    const sh2z_ibox2_t *a,size_t a_count,const sh2z_ibox2_t *b,size_t b_count,sh2z_box_edges_t edges);
bool sh2z_ibox2c_overlap_many_many(uint32_t *bits,size_t bit_word_capacity,size_t *pair_count,
    const sh2z_ibox2c_t *a,size_t a_count,const sh2z_ibox2c_t *b,size_t b_count,sh2z_box_edges_t edges);

#ifdef __cplusplus
}
#endif
#endif
