/* IMA/DVI ADPCM: the step table and the reference decoder (the specification).
 * Separate from the fast path so a program that only decodes links neither.
 * MIT; Copyright (c) 2026 SH2ZAM Contributors. */
#include "sh2zam/adpcm.h"

const uint16_t sh2z_ima_step[SH2Z_IMA_ROWS]={
    7,8,9,10,11,12,13,14,16,17,19,21,23,25,28,31,
    34,37,41,45,50,55,60,66,73,80,88,97,107,118,130,143,
    157,173,190,209,230,253,279,307,337,371,408,449,494,544,
    598,658,724,796,876,963,1060,1166,1282,1411,1552,1707,
    1878,2066,2272,2499,2749,3024,3327,3660,4026,4428,4871,
    5358,5894,6484,7132,7845,8630,9493,10442,11487,12635,13899,
    15289,16818,18500,20350,22385,24623,27086,29794,32767};

void sh2z_ima_ref_decode(sh2z_ima_state_t *state,const uint8_t *src,uint32_t pos,
                         int16_t *out,size_t count) {
    static const int8_t adjust[16]={-1,-1,-1,-1,2,4,6,8,-1,-1,-1,-1,2,4,6,8};
    int32_t pred=state->pred,index=state->index;
    for(size_t i=0;i<count;i++,pos++) {
        const uint8_t b=src[pos>>1];
        const int nib=(pos&1u) ? (b>>4) : (b&0x0F);
        const int32_t step=sh2z_ima_step[index];
        int32_t delta=step>>3;
        if(nib&4) delta+=step;
        if(nib&2) delta+=step>>1;
        if(nib&1) delta+=step>>2;
        pred+=(nib&8) ? -delta : delta;
        if(pred>32767) pred=32767; else if(pred<-32768) pred=-32768;
        index+=adjust[nib];
        if(index<0) index=0; else if(index>88) index=88;
        out[i]=(int16_t)pred;
    }
    state->pred=pred;
    state->index=index;
}
