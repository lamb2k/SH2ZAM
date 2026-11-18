/* SH-2 bridge for the executed-instruction view-rails-1 model. MIT; Copyright
 * (c) 2026 SH2ZAM Contributors. Every entry point is called directly by the
 * model harness with the SH-2 calling convention; the harness writes the
 * centre view, the four rail ends, the boundaries and the arguments before
 * each call; nothing here uses libc.
 *
 *   rails_prepare      sh2z_view_rails_prepare(&handle, &view)
 *   rails_divu         sh2z_view_edge_rails_center_divu: both rails, one call
 *   rails_portable     sh2z_view_edge_rails_center (the C path)
 *   rails_divu_ptrs    rails_divu and rails_portable with every pointer
 *   rails_portable_ptrs  taken from ptrs[], for the argument rules
 *   loop_divu          the loop it replaces, as a road renderer writes it:
 *                      for each rail, copy the end to the boundaries at or
 *                      below low and at or above high, then one
 *                      sh2z_view_edge_steps_center_divu call for the rest
 *                      with the renderer's repetition length as the step. */
#include "sh2zam/view_experimental.h"

#define NOINLINE __attribute__((noinline))
#define RAILS_MODEL_BOUNDARIES 96u

sh2z_view_center_t view;
sh2z_view_rails_t handle;
sh2z_view_vertex_t ends[4];
int32_t boundary[RAILS_MODEL_BOUNDARIES];
/* count, l_low, l_high, r_low, r_high, step (loop_divu only) */
int32_t args[6];
sh2z_view_vertex_t out_l[RAILS_MODEL_BOUNDARIES],out_r[RAILS_MODEL_BOUNDARIES];
const uint32_t rails_model_boundaries=RAILS_MODEL_BOUNDARIES;
/* out_l, out_r, handle, l_from, l_to, r_from, r_to, boundary */
void *ptrs[8];

NOINLINE uint32_t rails_prepare(void) {
    return sh2z_view_rails_prepare(&handle,&view);
}
NOINLINE uint32_t rails_divu(void) {
    return sh2z_view_edge_rails_center_divu(out_l,out_r,&handle,&ends[0],&ends[1],&ends[2],&ends[3],
        boundary,(unsigned)args[0],args[1],args[2],args[3],args[4]);
}
NOINLINE uint32_t rails_portable(void) {
    return sh2z_view_edge_rails_center(out_l,out_r,&handle,&ends[0],&ends[1],&ends[2],&ends[3],
        boundary,(unsigned)args[0],args[1],args[2],args[3],args[4]);
}
NOINLINE uint32_t rails_divu_ptrs(void) {
    return sh2z_view_edge_rails_center_divu(ptrs[0],ptrs[1],ptrs[2],ptrs[3],ptrs[4],ptrs[5],ptrs[6],ptrs[7],
        (unsigned)args[0],args[1],args[2],args[3],args[4]);
}
NOINLINE uint32_t rails_portable_ptrs(void) {
    return sh2z_view_edge_rails_center(ptrs[0],ptrs[1],ptrs[2],ptrs[3],ptrs[4],ptrs[5],ptrs[6],ptrs[7],
        (unsigned)args[0],args[1],args[2],args[3],args[4]);
}
static NOINLINE bool edge(sh2z_view_vertex_t *out,const sh2z_view_vertex_t *from,const sh2z_view_vertex_t *to,
    int32_t low,int32_t high,unsigned count,unsigned step) {
    unsigned first=0,end=count;
    while(first<count && boundary[first]<=low)out[first++]=*from;
    while(end>first && boundary[end-1]>=high)out[--end]=*to;
    if(end==first)return true;
    return sh2z_view_edge_steps_center_divu(out+first,&view,from,to,
        (uint16_t)(boundary[first]-low),(uint16_t)step,(uint16_t)(high-low),end-first);
}
NOINLINE uint32_t loop_divu(void) {
    const unsigned count=(unsigned)args[0],step=(unsigned)args[5];
    return edge(out_l,&ends[0],&ends[1],args[1],args[2],count,step) &&
        edge(out_r,&ends[2],&ends[3],args[3],args[4],count,step);
}
