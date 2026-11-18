/* SH-2 bridge for the executed-instruction model of the texture-merge
 * planners. MIT; Copyright (c) 2026 SH2ZAM Contributors. Every entry point is
 * called directly by the model harness with the SH-2 calling convention; the
 * harness writes the buffers and arguments before each call; nothing here
 * uses libc.
 *
 *   tier_plan  sh2z_view_merge_tier (view-merge-2)
 *   runs_plan  sh2z_view_merge_runs in pixel mode (view-merge-1), with the
 *              rows and end masks a road renderer passes it today
 *
 * Arguments in merge_args: {count, run_mask, tol_q8, use end masks}. The
 * runs go to merge_runs. */
#include "sh2zam/view_experimental.h"

#define NOINLINE __attribute__((noinline))
#define MERGE_CAP 200u

_Static_assert(sizeof(sh2z_view_vertex_t)==20,"vertex size");

const uint32_t merge_capacity=MERGE_CAP;
sh2z_view_vertex_t merge_left[MERGE_CAP],merge_right[MERGE_CAP];
uint32_t merge_rows[MERGE_CAP],merge_ends[MERGE_CAP];
uint8_t merge_runs[MERGE_CAP+16u];
uint32_t merge_args[4];

NOINLINE int32_t tier_plan(void) {
    return sh2z_view_merge_tier(merge_runs,merge_left,merge_right,merge_args[3]?merge_ends:(const uint32_t *)0,
        merge_args[0],merge_args[1],merge_args[2]);
}
NOINLINE int32_t runs_plan(void) {
    return sh2z_view_merge_runs(merge_runs,merge_left,merge_right,merge_rows,merge_args[3]?merge_ends:(const uint32_t *)0,
        merge_args[0],merge_args[1],merge_args[2],SH2Z_VIEW_MERGE_PIXELS);
}
