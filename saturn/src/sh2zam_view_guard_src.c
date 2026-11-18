/* view-guard-src-1: the guard-band clip in a source renderer's order, for
 * ports whose fan-mapped textures make the clipped polygon's starting vertex
 * and order visible. Previous-to-current Sutherland-Hodgman passes, crossings
 * measured along the edge with a Q0.32 fraction, each crossing snapped
 * inward to the integer lattice of the planes clipped so far, then the
 * view-round-1 truncating projection. The centre enters as Q16 internally.
 * A separate file, so every earlier function compiles exactly as before.
 * The clip itself is in sh2zam_view_guard_src_core.h, shared with the
 * view-center-1 forms in sh2zam_view_guard_src_center.c.
 *
 * Copyright (c) 2026 SH2ZAM Contributors. MIT License.
 * Inspired by SH4ZAM by Falco Girgis and SH4ZAM Contributors.
 */
#include "sh2zam_view_guard_src_core.h"

int sh2z_view_clip_guard_src_trunc(sh2z_view_guard_vertex_t out[SH2Z_VIEW_GUARD_MAX_OUT],
    const sh2z_view_t *v,const int16_t guard[4],
    const sh2z_view_guard_vertex_t *in,unsigned count,unsigned attrs,unsigned max_out) {
    if(!v)return SH2Z_VIEW_GUARD_INVALID;
    return clip_guard_src(out,v,(int32_t)v->center_x*65536,(int32_t)v->center_y*65536,guard,in,count,attrs,max_out,false);
}
int sh2z_view_clip_guard_src_trunc_divu(sh2z_view_guard_vertex_t out[SH2Z_VIEW_GUARD_MAX_OUT],
    const sh2z_view_t *v,const int16_t guard[4],
    const sh2z_view_guard_vertex_t *in,unsigned count,unsigned attrs,unsigned max_out) {
    if(!v)return SH2Z_VIEW_GUARD_INVALID;
    return clip_guard_src(out,v,(int32_t)v->center_x*65536,(int32_t)v->center_y*65536,guard,in,count,attrs,max_out,true);
}
