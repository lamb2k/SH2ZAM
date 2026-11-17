/* View batch entry points for the SH-2 instruction model. The harness writes
 * the camera and positions into these buffers, calls each path and compares
 * the outputs with its own oracle. MIT; SH2ZAM Contributors. */
#include "sh2zam/view_experimental.h"
#include "sh2zam/divu_experimental.h"
#include <stddef.h>
#ifndef BENCH_COMMIT
#define BENCH_COMMIT "host-test-only"
#endif
#define VIEW_CAPACITY 2048u

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
sh2z_pos16_t view_input[VIEW_CAPACITY];
sh2z_view_vertex_t view_out_portable[VIEW_CAPACITY];
sh2z_view_vertex_t view_out_divu[VIEW_CAPACITY];

uint32_t view_run_portable(uint32_t count) {
    return sh2z_view_batch(view_out_portable,&view_camera,view_input,sizeof(view_input[0]),count);
}
uint32_t view_run_divu(uint32_t count) {
    return sh2z_view_batch_divu(view_out_divu,&view_camera,view_input,sizeof(view_input[0]),count);
}

uint32_t view_run_divu_stride12(uint32_t count) {
    return sh2z_view_batch_divu(view_out_divu,&view_camera,view_input,2*sizeof(view_input[0]),count);
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
