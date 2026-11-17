/* Batch placement of 16x16 sprite cells: mirror, offset, clip, compact.
 * MIT; Copyright (c) 2026 SH2ZAM Contributors. Renderer-neutral: the output is
 * screen positions and source indices; the caller builds whatever command or
 * quad list its renderer needs. Nothing here knows a VDP, a palette or a tile
 * format, and no screen size is assumed.
 *
 * A "block" is a rectangular group of cells (a sprite built from tiles). The
 * caller gives each cell's top-left pixel offset inside the block
 * (offsets[i]) and the block's size in pixels. For each cell i:
 *   x = offsets[i].x, y = offsets[i].y
 *   FLIP_X:  x = block_w - 16 - x     (mirror the block horizontally)
 *   FLIP_Y:  y = block_h - 16 - y
 *   screen   = origin + (x, y)        (exact integer arithmetic, no wrap)
 * The cell covers the half-open pixel rectangle [sx, sx+16) x [sy, sy+16).
 * It is visible iff that rectangle intersects the half-open clip rectangle
 * [min_x, max_x) x [min_y, max_y), i.e. sx+16 > min_x && sx < max_x, same for y.
 * Visible cells are written in input order to out_pos (and their input index
 * to out_index when non-null); invisible cells write nothing. Pixel contents of
 * a flipped cell are the renderer's business: this function only moves the cell.
 *
 * Preconditions (checked before any output write; failure returns false and
 * leaves all outputs unchanged): count <= 65536 (indices are 16-bit); arrays
 * non-null and naturally aligned when needed; out_capacity >= count (worst
 * case: every cell visible); |origin|, |block_w|, |block_h| <= 2^20; clip min_x/min_y >=
 * -32752 and max > min on both axes (an empty clip rectangle returns true with
 * zero visible cells), which guarantees every visible position fits int16; flip
 * only uses the two defined bits; outputs do not overlap each other or the
 * input. All arithmetic is exact in int32. Reentrant, allocation-free, no global
 * state; the caller owns synchronization and cache/DMA coherency. SH-2: compiled
 * C, no MAC registers, no hardware divider. */
#ifndef SH2ZAM_CELL16_H
#define SH2ZAM_CELL16_H
#include "sh2zam.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SH2Z_CELL16_FLIP_X 1u
#define SH2Z_CELL16_FLIP_Y 2u
#define SH2Z_CELL16_SIZE 16

/** Half-open clip rectangle in screen pixels. */
typedef struct { int16_t min_x,min_y,max_x,max_y; } sh2z_clip_rect_t;

/** Placement of one block: where its origin lies on screen, its size, and its
 *  mirroring (SH2Z_CELL16_FLIP_*). Passed by pointer; not modified. */
typedef struct { int32_t origin_x,origin_y,block_w,block_h; uint32_t flip; } sh2z_cell16_xform_t;

/** Mirror, offset and clip count cells; see the file comment. visible receives
 *  the number of cells written (required). */
bool sh2z_cell16_place(sh2z_screen_t *out_pos,uint16_t *out_index,size_t out_capacity,size_t *visible,
    const sh2z_screen_t *offsets,size_t count,const sh2z_cell16_xform_t *xform,const sh2z_clip_rect_t *clip);

#ifdef __cplusplus
}
#endif
#endif
