/* Texture-merge planner example: a curving road strip whose texture repeats
 * every 2 units, drawn as one quad per repetition or merged two or four at a
 * time wherever every merged quad stays within 1 pixel of the per-repetition
 * drawing. Host demo of the real API. */
#include "sh2zam/view_experimental.h"
#include <stdio.h>
#include <string.h>

enum { BOUNDARIES=33 };

int main(void) {
    sh2z_view_t view;
    memset(&view,0,sizeof(view));
    view.r[0][0]=view.r[1][1]=view.r[2][2]=16384;
    view.focal_x=view.focal_y=160*65536;
    view.center_x=160;view.center_y=112;
    view.near_z=16384/4;view.far_z=4000*16384;
    view.clip[0]=0;view.clip[1]=0;view.clip[2]=319;view.clip[3]=223;
    if(!sh2z_view_init(&view,&view) ||
       !sh2z_view_place(&view,(sh2z_vec3_t){0,2*65536,-4*65536}))return 1;

    /* Rails 8 units apart, bending right as the road goes away; a boundary
     * every 2 units, 32 texture rows per repetition. */
    sh2z_pos16_t rail[2][BOUNDARIES];
    uint32_t rows[BOUNDARIES];
    for(int k=0;k<BOUNDARIES;++k) {
        const int z=2*k,bend=z*z/48;
        rail[0][k]=(sh2z_pos16_t){(int16_t)(bend-4),0,(int16_t)z};
        rail[1][k]=(sh2z_pos16_t){(int16_t)(bend+4),0,(int16_t)z};
        rows[k]=32u*(uint32_t)k;
    }
    sh2z_view_vertex_t left[BOUNDARIES],right[BOUNDARIES];
    if(!sh2z_view_batch(left,&view,rail[0],sizeof(rail[0][0]),BOUNDARIES) ||
       !sh2z_view_batch(right,&view,rail[1],sizeof(rail[1][0]),BOUNDARIES))return 1;

    /* Runs of 1, 2 or 4 repetitions (bits 1, 2, 4), every boundary row within
     * 1 pixel (256 in Q8) of where the reference draws it. */
    uint8_t runs[BOUNDARIES-1];
    const int n=sh2z_view_merge_runs(runs,left,right,rows,NULL,BOUNDARIES,
        (1u<<1)|(1u<<2)|(1u<<4),256,SH2Z_VIEW_MERGE_PIXELS);
    if(n<1)return 1;
    printf("%d repetitions as %d quads at 1 px:",BOUNDARIES-1,n);
    unsigned covered=0,i=0;
    for(int k=0;k<n;++k) {
        /* Quad k: left[i], right[i], right[i+runs[k]], left[i+runs[k]]. */
        printf(" %u",(unsigned)runs[k]);
        covered+=runs[k];i+=runs[k];
    }
    printf("\n");
    /* The same strip at 0.5 px, and in rows mode at one texture row. */
    const int half=sh2z_view_merge_runs(runs,left,right,rows,NULL,BOUNDARIES,0x16u,128,SH2Z_VIEW_MERGE_PIXELS);
    const int by_rows=sh2z_view_merge_runs(runs,left,right,rows,NULL,BOUNDARIES,0x16u,256,SH2Z_VIEW_MERGE_ROWS);
    printf("0.5 px: %d quads; rows mode, 1 row: %d quads\n",half,by_rows);
    return covered==BOUNDARIES-1 && i==covered && half>=n && by_rows>=1 && n<BOUNDARIES-1?0:1;
}
