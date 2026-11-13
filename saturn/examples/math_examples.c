/* Practical examples shared by host executables and the Yaul disc demo.
 * MIT; Copyright (c) 2026 SH2ZAM Contributors. */
#include "math_examples.h"

void sh2z_example_orbit(uint32_t frame,sh2z_screen_t *out) {
    sh2z_angle_t phase=(sh2z_angle_t)(frame*256u);
    out->x=(int16_t)(160+sh2z_to_int(sh2z_mul(sh2z_from_int(80),sh2z_cos(phase))));
    out->y=(int16_t)(112+sh2z_to_int(sh2z_mul(sh2z_from_int(56),sh2z_sin(phase))));
}
bool sh2z_example_cube(sh2z_angle_t angle,sh2z_screen_t out[8],uint8_t *visible) {
    if(!out||!visible) return false;
    sh2z_mat34_t rx=sh2z_mat34_rotation_x((sh2z_angle_t)(angle/2));
    sh2z_mat34_t ry=sh2z_mat34_rotation_y(angle),model,view,model_view;
    sh2z_mat34_multiply(&model,&ry,&rx);
    if(!sh2z_mat34_look_at(&view,(sh2z_vec3_t){0,0,6*SH2Z_ONE},(sh2z_vec3_t){0,0,0},(sh2z_vec3_t){0,SH2Z_ONE,0})) return false;
    sh2z_mat34_multiply(&model_view,&view,&model);
    sh2z_mat44_t mv=sh2z_mat44_from_mat34(&model_view),projection,mvp;
    if(!sh2z_mat44_perspective(&projection,sh2z_angle_from_degrees(60),sh2z_div(320,224),SH2Z_ONE,20*SH2Z_ONE)) return false;
    sh2z_mat44_multiply(&mvp,&projection,&mv);
    *visible=0;
    for(unsigned i=0;i<8;i++) {
        sh2z_vec4_t p={(i&1)?SH2Z_ONE:-SH2Z_ONE,(i&2)?SH2Z_ONE:-SH2Z_ONE,(i&4)?SH2Z_ONE:-SH2Z_ONE,SH2Z_ONE};
        out[i]=(sh2z_screen_t){0,0};
        if(sh2z_project_clip(&out[i],sh2z_mat44_transform(&mvp,p),320,224)) *visible|=(uint8_t)(1u<<i);
    }
    return true;
}
bool sh2z_example_batch(void) {
    sh2z_vec3_t input[17],output[17];
    sh2z_mat34_t transform=sh2z_mat34_rotation_y(sh2z_angle_from_degrees(30));
    transform.m[0][3]=4*SH2Z_ONE;
    for(unsigned i=0;i<17;i++) input[i]=(sh2z_vec3_t){sh2z_from_int((int32_t)i-8),SH2Z_ONE,0};
    /* Demonstrate two DISJOINT jobs sequentially. Your engine dispatches these
     * ranges to master/slave SH-2 and handles ownership, caches and completion.
     * This function deliberately does not pretend to start a second CPU. */
    for(size_t worker=0;worker<2;worker++) {
        sh2z_range_t range;
        if(!sh2z_partition(17,worker,2,&range)) return false;
        if(!sh2z_mat34_transform_points(output+range.begin,&transform,input+range.begin,range.end-range.begin)) return false;
    }
    for(unsigned i=0;i<17;i++) {
        sh2z_vec3_t ref=sh2z_mat34_transform_point(&transform,input[i]);
        if(output[i].x!=ref.x||output[i].y!=ref.y||output[i].z!=ref.z) return false;
    }
    return true;
}
