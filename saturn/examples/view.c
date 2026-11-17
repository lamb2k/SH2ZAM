/* View batch example: place a camera, project four corners of a road quad and
 * trivially reject a quad behind the camera. Host demo of the real API. */
#include "sh2zam/view_experimental.h"
#include <stdio.h>
#include <string.h>

int main(void) {
    /* Camera looking down +z from (0, 2, -10) model units: identity rotation. */
    sh2z_view_t view;
    memset(&view,0,sizeof(view));
    view.r[0][0]=view.r[1][1]=view.r[2][2]=16384;
    view.focal_x=view.focal_y=160*65536;          /* 90 degree field of view */
    view.center_x=160;view.center_y=112;
    view.near_z=16384/4;view.far_z=4000*16384;
    view.clip[0]=0;view.clip[1]=0;view.clip[2]=319;view.clip[3]=223;
    if(!sh2z_view_init(&view,&view) ||
       !sh2z_view_place(&view,(sh2z_vec3_t){0,2*65536,-10*65536}))return 1;

    const sh2z_pos16_t road[8]={{-4,0,0},{4,0,0},{4,0,20},{-4,0,20},
                                {-4,0,-30},{4,0,-30},{4,0,-20},{-4,0,-20}};
    sh2z_view_vertex_t out[8];
    if(!sh2z_view_batch(out,&view,road,sizeof(road[0]),8))return 1;
    /* Same bytes from the hardware-divider entry point (portable on a host). */
    sh2z_view_vertex_t fast[8];
    if(!sh2z_view_batch_divu(fast,&view,road,sizeof(road[0]),8) ||
       memcmp(out,fast,sizeof(out)))return 1;

    for(unsigned q=0;q<2;++q) {
        unsigned all=0x3f;
        for(unsigned k=0;k<4;++k)all&=out[q*4+k].outcode;
        printf("quad %u:",q);
        for(unsigned k=0;k<4;++k)
            printf(" (%d,%d)%s",out[q*4+k].sx,out[q*4+k].sy,
                (out[q*4+k].outcode&SH2Z_OUT_NEAR)?"near":"");
        printf(" -> %s\n",all?"rejected":"drawn");
        if((q==0)!=(all==0))return 1;
    }
    return 0;
}
