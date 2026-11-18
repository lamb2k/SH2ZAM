/* Independent oracle for the vehicle-kernels-1 shade kernels, shared by
 * tests/test_vehicle.c and the SH-2 model harness. Written from the header
 * text in 64-bit arithmetic; shares no code with the library. Mutation flags
 * select deliberately wrong rules (negative controls). The lighting oracle is
 * tests/shade_oracle.h's (light_normals defines the indexed form).
 * MIT; SH2ZAM Contributors. */
#ifndef SH2ZAM_VEHICLE_ORACLE_H
#define SH2ZAM_VEHICLE_ORACLE_H
#include "shade_oracle.h"

enum {
    VMUT_NONE=0,
    VMUT_ONE_TRIANGLE=1,     /* facing area of triangle 0-1-2 only */
    VMUT_NO_SIMPLE=2,        /* facing tested on every quad (no near/far/limit gate) */
    VMUT_NO_ALL=4,           /* no trivial reject on the AND of the outcodes */
    VMUT_KEEP_EDGE_ON=8,     /* keep sign * area == 0 */
    VMUT_LIMIT_1024=16,      /* coordinate limit 1024 instead of 1023 */
    VMUT_NO_ROUND_M=32,      /* approximate m without the + 128 */
    VMUT_EXACT_ROUND=64,     /* approximate lit with the exact (x + (x >> 8)) >> 8 */
    VMUT_INDEX_ALL=128       /* indexed kernels write every record, not only listed ones */
};

/* The facing rule for one quad: 1 kept, 0 dropped. */
static inline int ovf_keep(const sh2z_view_vertex_t *v[4],int sign,unsigned mut) {
    if(!(mut&VMUT_NO_ALL) && (v[0]->outcode&v[1]->outcode&v[2]->outcode&v[3]->outcode))return 0;
    const int64_t limit=(mut&VMUT_LIMIT_1024)?1024:1023;
    int simple=1;
    for(int j=0;j<4;++j) {
        if(v[j]->outcode&(SH2Z_OUT_NEAR|SH2Z_OUT_FAR))simple=0;
        const int64_t x=v[j]->sx,y=v[j]->sy;
        if(x<-limit || x>limit || y<-limit || y>limit)simple=0;
    }
    if(!simple && !(mut&VMUT_NO_SIMPLE))return 1;
    int64_t x[4],y[4];
    for(int j=0;j<4;++j){x[j]=v[j]->sx;y[j]=v[j]->sy;}
    int64_t area=(x[1]-x[0])*(y[2]-y[0])-(y[1]-y[0])*(x[2]-x[0]);
    if(!(mut&VMUT_ONE_TRIANGLE))area+=(x[2]-x[0])*(y[3]-y[0])-(y[2]-y[0])*(x[3]-x[0]);
    const int64_t s=sign*area;
    return (mut&VMUT_KEEP_EDGE_ON)?s>=0:s>0;
}

/* The approximate Gouraud channel value before the clamp (1 .. 47). */
static inline int64_t ova_channel(int64_t b,int64_t l,int64_t f,int64_t t,unsigned mut) {
    const int64_t m=(mut&VMUT_NO_ROUND_M)?(b*l)>>8:(b*l+128)>>8;
    const int64_t x=m*t+f*(255-t)+128;
    const int64_t lit=(mut&VMUT_EXACT_ROUND)?(x+(x>>8))>>8:x>>8;
    return 16+(lit>>3)-(b>>4);
}
static inline uint16_t oracle_gouraud_approx(const uint8_t *b,const uint8_t *l,const uint8_t *f,uint32_t *sat,unsigned mut) {
    unsigned packed=0;
    const int64_t t=255-(int64_t)l[3];
    for(int k=0;k<3;++k) {
        int64_t v=ova_channel(b[k],l[k],f[k],t,mut);
        if(v<0){v=0;++*sat;}
        if(v>31){v=31;++*sat;}
        packed|=(unsigned)v<<(5*k);
    }
    return (uint16_t)packed;
}

#endif
