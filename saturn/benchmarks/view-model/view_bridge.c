/* View batch entry points for the SH-2 instruction model. The harness writes
 * the camera and positions into these buffers, calls each path and compares
 * the outputs with its own oracle. MIT; SH2ZAM Contributors. */
#include "sh2zam/view_experimental.h"
#include "sh2zam/divu_experimental.h"
#include <stddef.h>
#ifndef BENCH_COMMIT
#define BENCH_COMMIT "host-test-only"
#endif
#define VIEW_CAPACITY 1024u

/* The harness writes these layouts as big-endian bytes; keep them pinned. */
_Static_assert(offsetof(sh2z_view_t,t)==20,"view t offset");
_Static_assert(offsetof(sh2z_view_t,focal_x)==32,"view focal offset");
_Static_assert(offsetof(sh2z_view_t,center_x)==40,"view center offset");
_Static_assert(offsetof(sh2z_view_t,near_z)==44,"view near offset");
_Static_assert(offsetof(sh2z_view_t,clip)==52,"view clip offset");
_Static_assert(sizeof(sh2z_view_t)==60,"view size");
_Static_assert(sizeof(sh2z_view_vertex_t)==20,"vertex size");

const char view_source_commit[]=BENCH_COMMIT;
const uint32_t view_capacity=VIEW_CAPACITY;
sh2z_view_t view_camera;
/* Positions for the batch, or view-space records for sh2z_view_project:
 * one buffer, so the ELF still fits the benchmark RAM region. */
#define VIEW_MERGE_CAPACITY 200u
union {
    sh2z_pos16_t pos[VIEW_CAPACITY];
    sh2z_view_vertex_t pts[512];
    sh2z_view_guard_vertex_t guard[SH2Z_VIEW_GUARD_MAX_IN];
    struct {
        sh2z_view_vertex_t left[VIEW_MERGE_CAPACITY],right[VIEW_MERGE_CAPACITY];
        uint32_t rows[VIEW_MERGE_CAPACITY],ends[VIEW_MERGE_CAPACITY];
    } merge;
} view_input;
sh2z_view_vertex_t view_out_portable[VIEW_CAPACITY];
sh2z_view_vertex_t view_out_divu[VIEW_CAPACITY];

uint32_t view_run_portable(uint32_t count) {
    return sh2z_view_batch(view_out_portable,&view_camera,view_input.pos,sizeof(view_input.pos[0]),count);
}
uint32_t view_run_divu(uint32_t count) {
    return sh2z_view_batch_divu(view_out_divu,&view_camera,view_input.pos,sizeof(view_input.pos[0]),count);
}

uint32_t view_run_divu_stride12(uint32_t count) {
    return sh2z_view_batch_divu(view_out_divu,&view_camera,view_input.pos,2*sizeof(view_input.pos[0]),count);
}

/* view-edges-1: the harness writes the two edge ends, the four quad corners
 * and {first_num, step_num, den, count}. */
sh2z_view_vertex_t view_edge_ends[2];
sh2z_view_vertex_t view_clip_in[4];
uint32_t view_edge_args[4];
uint32_t view_edges_portable(void) {
    return sh2z_view_edge_steps(view_out_portable,&view_camera,&view_edge_ends[0],&view_edge_ends[1],
        (uint16_t)view_edge_args[0],(uint16_t)view_edge_args[1],(uint16_t)view_edge_args[2],view_edge_args[3]);
}
uint32_t view_edges_divu(void) {
    return sh2z_view_edge_steps_divu(view_out_divu,&view_camera,&view_edge_ends[0],&view_edge_ends[1],
        (uint16_t)view_edge_args[0],(uint16_t)view_edge_args[1],(uint16_t)view_edge_args[2],view_edge_args[3]);
}
uint32_t view_clip_portable(void) {
    return sh2z_view_clip_near(view_out_portable,&view_camera,view_clip_in);
}
uint32_t view_clip_divu(void) {
    return sh2z_view_clip_near_divu(view_out_divu,&view_camera,view_clip_in);
}
/* View-space points for sh2z_view_project[_divu], in view_input. */
uint32_t view_project_portable(uint32_t count) {
    return sh2z_view_project(view_out_portable,&view_camera,view_input.pts,count);
}
uint32_t view_project_divu(uint32_t count) {
    return sh2z_view_project_divu(view_out_divu,&view_camera,view_input.pts,count);
}

/* view-guard-1: polygon in view_input.guard, rectangle and {count, attrs}
 * set by the harness; outputs reuse the start of the batch output buffers. */
int16_t view_guard_rect[4];
uint32_t view_guard_args[2];
_Static_assert(sizeof(sh2z_view_guard_vertex_t)==36,"guard vertex size");
_Static_assert(sizeof(view_out_portable)>=SH2Z_VIEW_GUARD_MAX_OUT*sizeof(sh2z_view_guard_vertex_t),"guard output room");
int32_t view_guard_portable(void) {
    return sh2z_view_clip_guard((sh2z_view_guard_vertex_t *)(void *)view_out_portable,&view_camera,view_guard_rect,
        view_input.guard,view_guard_args[0],view_guard_args[1]);
}
int32_t view_guard_divu(void) {
    return sh2z_view_clip_guard_divu((sh2z_view_guard_vertex_t *)(void *)view_out_divu,&view_camera,view_guard_rect,
        view_input.guard,view_guard_args[0],view_guard_args[1]);
}

/* view-merge-1: rails, rows and end masks in view_input.merge (left at 0,
 * right at 4000, rows at 8000, end masks at 8800) and
 * {count, run_mask, tol_q8, mode, use end masks} set by the harness; the
 * runs go to the start of view_out_portable. No new buffers: the merge
 * layout fits inside the existing input union. */
const uint32_t view_merge_capacity=VIEW_MERGE_CAPACITY;
uint32_t view_merge_args[5];
_Static_assert(sizeof(view_input.merge)<=sizeof(view_input.pts),"merge input fits the existing union");
_Static_assert(offsetof(sh2z_view_vertex_t,outcode)==16,"outcode offset");
int32_t view_merge(void) {
    return sh2z_view_merge_runs((uint8_t *)(void *)view_out_portable,view_input.merge.left,view_input.merge.right,
        view_input.merge.rows,view_merge_args[4]?view_input.merge.ends:NULL,view_merge_args[0],view_merge_args[1],
        view_merge_args[2],view_merge_args[3]);
}

/* Divider helper checks: start64/finish and an interrupted division saved and
 * restored around a second one. Arguments are the dividend halves and
 * divisor; results land in view_divu_result. */
int32_t view_divu_result[4];
uint32_t view_divu_overflow[2];
uint32_t view_divu_probe(int32_t hi,uint32_t lo,int32_t divisor) {
    sh2z_divu_t t;
    bool overflow=false;
    sh2z_divu_start64(&t,hi,lo,divisor);
    view_divu_result[0]=sh2z_divu_finish(&t,&overflow);
    view_divu_overflow[0]=overflow;
    /* Same division, interrupted: a handler saves, divides, restores. */
    sh2z_divu_start64(&t,hi,lo,divisor);
    sh2z_divu_state_t saved;
    sh2z_divu_save(&saved);
    view_divu_result[1]=sh2z_divu_div64(-1000000007,7,NULL);
    sh2z_divu_restore(&saved);
    view_divu_result[2]=sh2z_divu_finish(&t,&overflow);
    view_divu_overflow[1]=overflow;
    view_divu_result[3]=sh2z_divu_div64((int64_t)(((uint64_t)(uint32_t)hi<<32)|lo),divisor,NULL);
    return 1;
}
