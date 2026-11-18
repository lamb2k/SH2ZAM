/* view-center-1 forms of the view-guard-src-1 clip: the source-order guard
 * clip with the Q16 centre of sh2z_view_center_t passed through unchanged.
 * A separate file with its own copy of the clip, so sh2zam_view_guard_src.c
 * and sh2zam_view_center.c compile exactly as before.
 *
 * Copyright (c) 2026 SH2ZAM Contributors. MIT License.
 * Inspired by SH4ZAM by Falco Girgis and SH4ZAM Contributors.
 */
#include "sh2zam_view_guard_src_core.h"

/* floor(q / 65536) without implementation-defined signed shifts. */
static int32_t src_center_floor(int32_t q) {
    return q>=0?(int32_t)((uint32_t)q>>16):-(int32_t)(~(uint32_t)q>>16)-1;
}
/* The view-center-1 validity rule: the whole-pixel centre is the floor of
 * the Q16 one. The clip checks the view itself, under the same rules. */
static int clip_guard_src_center(sh2z_view_guard_vertex_t *out,const sh2z_view_center_t *v,const int16_t guard[4],
    const sh2z_view_guard_vertex_t *in,unsigned count,unsigned attrs,unsigned max_out,bool divu) {
    if(!v || v->view.center_x!=src_center_floor(v->center_x16) ||
       v->view.center_y!=src_center_floor(v->center_y16))return SH2Z_VIEW_GUARD_INVALID;
    return clip_guard_src(out,&v->view,v->center_x16,v->center_y16,guard,in,count,attrs,max_out,divu);
}

int sh2z_view_clip_guard_src_center(sh2z_view_guard_vertex_t out[SH2Z_VIEW_GUARD_MAX_OUT],
    const sh2z_view_center_t *v,const int16_t guard[4],
    const sh2z_view_guard_vertex_t *in,unsigned count,unsigned attrs,unsigned max_out) {
    return clip_guard_src_center(out,v,guard,in,count,attrs,max_out,false);
}
int sh2z_view_clip_guard_src_center_divu(sh2z_view_guard_vertex_t out[SH2Z_VIEW_GUARD_MAX_OUT],
    const sh2z_view_center_t *v,const int16_t guard[4],
    const sh2z_view_guard_vertex_t *in,unsigned count,unsigned attrs,unsigned max_out) {
    return clip_guard_src_center(out,v,guard,in,count,attrs,max_out,true);
}
