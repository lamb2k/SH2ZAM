/* SH-2 bridge for the executed-instruction view-camera-1 model. MIT;
 * Copyright (c) 2026 SH2ZAM Contributors. Every entry point is called
 * directly by the model harness with the SH-2 calling convention; inputs are
 * written by the harness before each call; nothing here uses libc.
 *
 *   fast_camera     sh2z_view_from_camera30 (basis, eye, lens -> out)
 *   fast_place      sh2z_view_place_q12 (place_view, eye)
 *   plain_chain     the float chain a view from a camera replaces, in binary32
 *                   with GCC's soft-float (tests/camera_float_ref.h): frustum
 *                   and look-at with their 16.16 packing, the projection-view
 *                   product, the float-to-fixed adapter and the centre view
 *                   with a Q16.16 eye (sh2z_view_place, sh2z_view_center_init)
 *   plain_adapter   the adapter and centre view alone, from the matrices that
 *                   plain_chain left in frame */
#include "sh2zam/view_experimental.h"
#include "../../tests/camera_float_ref.h"

#define NOINLINE __attribute__((noinline))

int32_t basis[9];
int32_t eye[3];
sh2z_view_lens_t lens;
sh2z_view_center_t out;
sh2z_view_t place_view;
float sin_table[4096];
cf_camera_t cam;
cf_frame_t frame;
sh2z_view_center_t plain_out;

NOINLINE uint32_t fast_camera(void) {
    return sh2z_view_from_camera30(&out,basis,eye,&lens);
}
NOINLINE uint32_t fast_place(void) {
    return sh2z_view_place_q12(&place_view,eye);
}
NOINLINE uint32_t plain_chain(void) {
    return (uint32_t)cf_chain(&plain_out,&frame,&cam,sin_table);
}
NOINLINE uint32_t plain_adapter(void) {
    cf_view_t v;
    if(!cf_view_from(&v,(const float(*)[4])frame.view.m,(const float(*)[4])frame.projection.m,
        cam.viewport,cam.scissor,cam.near_plane,cam.far_plane))return 0;
    int32_t e[3];
    for(unsigned i=0;i<3;++i)e[i]=(int32_t)(cam.eye[i]*65536.0f);
    return (uint32_t)cf_view16_center(&plain_out,&v,e);
}
