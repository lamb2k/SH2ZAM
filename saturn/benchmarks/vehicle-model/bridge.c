/* SH-2 bridge for the executed-instruction model of the vehicle-kernels-1
 * kernels. MIT; Copyright (c) 2026 SH2ZAM Contributors. Every entry point is
 * called directly by the model harness with the SH-2 calling convention.
 * "fast_" entries run the new kernels; "port_" entries run what an indexed
 * mesh renderer calls today for the same work (the whole-mesh shade-1 and
 * view-sh2-1 kernels, and its per-face facing loop written the usual way), so
 * each pair is compared on identical inputs. Buffers are written by the
 * harness before each call; nothing here uses libc. */
#include "sh2zam/shade_experimental.h"

#define NOINLINE __attribute__((noinline))
#define MAXV 256u
#define MAXN 256u
#define MAXP 1024u
#define MAXF 256u

/* ---- view ---- */
sh2z_view_t view;
sh2z_pos16_t positions[MAXV];
sh2z_view_vertex_t projected[MAXV],projected_all[MAXV];
uint8_t vindex[MAXV*2u];

NOINLINE uint32_t fast_view_indexed(uint32_t count,uint32_t n) {
    return sh2z_view_batch_divu_indexed(projected,&view,positions,sizeof(positions[0]),count,vindex,n);
}
NOINLINE uint32_t port_view_all(uint32_t count) {
    return sh2z_view_batch_divu(projected_all,&view,positions,sizeof(positions[0]),count);
}

/* ---- screen facing ---- */
uint8_t corners[MAXF][4] __attribute__((aligned(4)));
uint16_t faces[MAXF],kept[MAXF];
volatile uint32_t kept_count;

NOINLINE uint32_t fast_facing(uint32_t n,int32_t sign) {
    size_t k=0;const uint32_t ok=sh2z_shade_screen_facing(kept,&k,faces,n,(const uint8_t (*)[4])corners,projected,&view,sign);
    kept_count=(uint32_t)k;return ok;
}
/* The renderer's per-face test, as its emission loop does it: gather the four
 * corners, AND the outcodes, decide "simple" (no near/far, every coordinate
 * within +/-1023), then twice the signed area of triangles 0-1-2 and 0-2-3. */
NOINLINE uint32_t port_facing(uint32_t n,int32_t sign) {
    uint32_t k=0;
    for(uint32_t i=0;i<n;++i) {
        const unsigned id=faces[i];unsigned all=63;bool simple=true;int16_t x[4],y[4];
        for(unsigned j=0;j<4;++j){const sh2z_view_vertex_t *a=projected+corners[id][j];all&=a->outcode;
            x[j]=a->sx;y[j]=a->sy;
            if((a->outcode&3u)||a->sx< -1023||a->sx>1023||a->sy< -1023||a->sy>1023)simple=false;}
        if(all)continue;
        if(simple) {
            const int32_t x0=x[0],y0=y[0];
            const int32_t area=(x[1]-x0)*(y[2]-y0)-(y[1]-y0)*(x[2]-x0)+(x[2]-x0)*(y[3]-y0)-(y[2]-y0)*(x[3]-x0);
            if(sign*area<=0)continue;
        }
        kept[k++]=(uint16_t)id;
    }
    kept_count=k;return 1;
}

/* ---- lighting ---- */
sh2z_vec3_t prep_light,prep_reflection;
int16_t prep_basis[3][3];
uint8_t prep_ambient[4],prep_color[4];
sh2z_shade_light_t light;
int8_t normals[MAXN*4u];
uint8_t lut[256];
uint8_t lit[MAXN*4u];
uint8_t nindex[MAXN*2u];

NOINLINE uint32_t fast_prepare(void) {
    return sh2z_shade_light_prepare(&light,prep_light,prep_reflection,(const int16_t(*)[3])prep_basis,prep_ambient,prep_color);
}
NOINLINE uint32_t fast_light_indexed(uint32_t count,uint32_t n) {
    return sh2z_shade_light_normals_indexed(lit,4,normals,4,count,nindex,n,&light,lut);
}
NOINLINE uint32_t port_light_all(uint32_t count) {
    return sh2z_shade_light_normals(lit,4,normals,4,count,&light,lut);
}

/* ---- Gouraud ---- */
uint8_t bases[64u*4u];
uint8_t fog_rgb[4];
sh2z_shade_pair_t pairs[MAXP];
uint16_t gouraud[MAXP];
volatile uint32_t saturated;
uint32_t base_stride=3;

NOINLINE uint32_t fast_gouraud_approx(uint32_t count) {
    uint32_t s=0;const uint32_t ok=sh2z_shade_gouraud_approx(gouraud,&s,pairs,count,bases,base_stride,lit,4,fog_rgb);
    saturated=s;return ok;
}
NOINLINE uint32_t port_gouraud(uint32_t count) {
    uint32_t s=0;const uint32_t ok=sh2z_shade_gouraud(gouraud,&s,pairs,count,bases,base_stride,lit,4,fog_rgb);
    saturated=s;return ok;
}
