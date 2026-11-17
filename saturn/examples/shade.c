/* Shading kernels example: a lit, fogged, culled cube (vehicle-style per-normal
 * light and Gouraud packing), then a road-style depth fog for a row of
 * vertices. Host demo of the real API. */
#include "sh2zam/shade_experimental.h"
#include <stdio.h>

int main(void) {
    /* Six faces of a cube of half-size 64: normal, a corner, plane n . corner. */
    sh2z_shade_plane_t planes[6];
    static const int16_t n[6][3]={{16384,0,0},{-16384,0,0},{0,16384,0},{0,-16384,0},{0,0,16384},{0,0,-16384}};
    for(int f=0;f<6;++f) {
        for(int k=0;k<3;++k){planes[f].normal[k]=n[f][k];planes[f].first[k]=(int16_t)(n[f][k]?(n[f][k]>0?64:-64):64);}
        planes[f].plane=n[f][0]*planes[f].first[0]+n[f][1]*planes[f].first[1]+n[f][2]*planes[f].first[2];
    }
    if(!sh2z_shade_planes_valid(planes,sizeof(planes[0]),6,NULL,0))return 1;

    /* Eye at (300, 200, -500) in model units, within 2 units per axis. */
    const int32_t eye[3]={300,200,-500};
    uint16_t kept[6];size_t kept_count;
    if(!sh2z_shade_cull(kept,&kept_count,NULL,6,planes,sizeof(planes[0]),eye,2))return 1;
    printf("cull: %zu of 6 faces kept:",kept_count);
    for(size_t i=0;i<kept_count;++i)printf(" %u",(unsigned)kept[i]);
    printf("\n");

    /* Light from above-left, reflection straight up; identity model basis. */
    const int16_t basis[3][3]={{16384,0,0},{0,16384,0},{0,0,16384}};
    const uint8_t ambient[3]={40,40,48},color[3]={200,190,170},fog[3]={90,100,140};
    sh2z_shade_light_t light;
    if(!sh2z_shade_light_prepare(&light,(sh2z_vec3_t){-65536,65536*2,-65536},
        (sh2z_vec3_t){0,65536,0},basis,ambient,color))return 1;
    int8_t normals[6][4];
    for(int f=0;f<6;++f){for(int k=0;k<3;++k)normals[f][k]=(int8_t)(n[f][k]/129);normals[f][3]=0;}
    uint8_t lut[256];
    for(int i=0;i<256;++i)lut[i]=(uint8_t)(i<128?255:255-2*(i-128));   /* more fog when facing away */
    uint8_t lit[6][4];
    if(!sh2z_shade_light_normals(&lit[0][0],4,&normals[0][0],4,6,&light,lut))return 1;

    /* One Gouraud colour per kept face (all corners share the face normal). */
    const uint8_t base[1][3]={{220,40,30}};
    sh2z_shade_pair_t pairs[6];
    for(size_t i=0;i<kept_count;++i)pairs[i]=(sh2z_shade_pair_t){0,kept[i]};
    uint16_t gouraud[6];uint32_t saturated;
    if(!sh2z_shade_gouraud(gouraud,&saturated,pairs,kept_count,&base[0][0],3,&lit[0][0],4,fog))return 1;
    for(size_t i=0;i<kept_count;++i)
        printf("face %u: light %3u %3u %3u reflect %3u -> gouraud %04x\n",(unsigned)kept[i],
            lit[kept[i]][0],lit[kept[i]][1],lit[kept[i]][2],lit[kept[i]][3],gouraud[i]);
    printf("saturated channels: %u\n",(unsigned)saturated);

    /* Road fog: thresholds of a perspective ratio (near 4, far 4000 units,
     * fog starting at 600 of 1000), then fades and colours for a row of depths. */
    const int32_t near_u=4,far_u=4000,start=600;
    const int32_t pa=(int32_t)(((int64_t)(far_u+near_u)<<30)/(far_u-near_u));
    const int32_t pb=(int32_t)-((int64_t)2*far_u*near_u*4096/(far_u-near_u));
    sh2z_shade_fog_t road;
    if(!sh2z_shade_fog_build(&road,128000/(1000-start),(500-start)*256/(1000-start),pa,pb,far_u*16384))return 1;
    int32_t z[8];uint8_t fade[8];uint16_t mix[8];
    for(int i=0;i<8;++i)z[i]=(int32_t)(16384*(8<<i));
    const uint8_t road_rgb[3]={120,120,120};
    if(!sh2z_shade_fog_lookup(fade,1,z,sizeof(z[0]),8,&road) ||
       !sh2z_shade_fade_mix(mix,sizeof(mix[0]),fade,1,8,road_rgb,fog))return 1;
    for(int i=0;i<8;++i)printf("z %5d units: fade %3u colour %04x\n",z[i]/16384,fade[i],mix[i]);
    return kept_count==3 && fade[7]>=fade[0]?0:1;
}
