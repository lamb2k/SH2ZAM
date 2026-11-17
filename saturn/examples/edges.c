/* Edge steps and near clip example: cut a road quad's side edges where a
 * repeating texture restarts, and clip a quad that passes the camera.
 * Host demo of the real API. */
#include "sh2zam/view_experimental.h"
#include <stdio.h>
#include <string.h>

int main(void) {
    sh2z_view_t view;
    memset(&view,0,sizeof(view));
    view.r[0][0]=view.r[1][1]=view.r[2][2]=16384;
    view.focal_x=view.focal_y=160*65536;
    view.center_x=160;view.center_y=112;
    view.near_z=16384/4;view.far_z=4000*16384;
    view.clip[0]=0;view.clip[1]=0;view.clip[2]=319;view.clip[3]=223;
    if(!sh2z_view_init(&view,&view) ||
       !sh2z_view_place(&view,(sh2z_vec3_t){0,2*65536,-10*65536}))return 1;

    /* A quad 8 units wide running from z = 0 to z = 20; the texture repeats
     * every 6 units and its first restart is 2 units in: cut points at 2, 8
     * and 14 of 20 units, i.e. first 2/20, then every 6/20. */
    const sh2z_pos16_t road[4]={{-4,0,0},{4,0,0},{4,0,20},{-4,0,20}};
    sh2z_view_vertex_t corner[4],left[3],right[3];
    if(!sh2z_view_batch(corner,&view,road,sizeof(road[0]),4))return 1;
    if(!sh2z_view_edge_steps(left,&view,&corner[0],&corner[3],2,6,20,3) ||
       !sh2z_view_edge_steps(right,&view,&corner[1],&corner[2],2,6,20,3))return 1;
    for(unsigned k=0;k<3;++k)
        printf("cut %u at z=%d units: left (%d,%d) right (%d,%d)\n",k,
            left[k].z/16384,left[k].sx,left[k].sy,right[k].sx,right[k].sy);
    if(left[0].z!=12*16384 || left[1].z!=18*16384 || left[2].z!=24*16384)return 1;

    /* A quad from 5 units behind the camera to 10 in front: clip at near_z. */
    const sh2z_pos16_t passing[4]={{-4,0,-15},{4,0,-15},{4,0,0},{-4,0,0}};
    sh2z_view_vertex_t in[4],out[6];
    if(!sh2z_view_batch(in,&view,passing,sizeof(passing[0]),4))return 1;
    const unsigned n=sh2z_view_clip_near(out,&view,in);
    printf("clipped quad: %u vertices:",n);
    for(unsigned k=0;k<n;++k)printf(" (%d,%d)",out[k].sx,out[k].sy);
    printf("\n");
    return n==4 && out[0].z==view.near_z && out[3].z==view.near_z?0:1;
}
