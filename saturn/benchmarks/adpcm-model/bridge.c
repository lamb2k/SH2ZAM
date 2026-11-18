/* SH-2 bridge for the executed-instruction ADPCM model. MIT; Copyright (c) 2026
 * SH2ZAM Contributors. The harness writes the stream into bridge_src and the state
 * into bridge_state, calls one entry point, then reads bridge_out and bridge_state.
 *  fast_decode   the library (sh2z_ima_decode)
 *  plain_decode  straightforward C: the bitwise rule inline, one nibble per iteration
 *  step_decode   a per-nibble step function called once per nibble on a voice record,
 *                the shape the Altered Beast 32X port's speech lane used before adpcm-1 */
#include "sh2zam/adpcm.h"

#define NOINLINE __attribute__((noinline))
sh2z_ima_state_t bridge_state;
uint8_t bridge_src[2048];
int16_t bridge_out[2048];

NOINLINE void fast_decode(uint32_t pos,uint32_t count) {
    sh2z_ima_decode(&bridge_state,bridge_src,pos,bridge_out,count);
}

static const uint16_t plain_step[89]={7,8,9,10,11,12,13,14,16,17,19,21,23,25,28,31,34,37,41,45,50,55,60,66,73,80,88,97,107,118,130,143,157,173,190,209,230,253,279,307,337,371,408,449,494,544,598,658,724,796,876,963,1060,1166,1282,1411,1552,1707,1878,2066,2272,2499,2749,3024,3327,3660,4026,4428,4871,5358,5894,6484,7132,7845,8630,9493,10442,11487,12635,13899,15289,16818,18500,20350,22385,24623,27086,29794,32767};
static const int8_t plain_adjust[16]={-1,-1,-1,-1,2,4,6,8,-1,-1,-1,-1,2,4,6,8};

NOINLINE void plain_decode(uint32_t pos,uint32_t count) {
    int32_t pred=bridge_state.pred,index=bridge_state.index;
    for(uint32_t i=0;i<count;i++,pos++) {
        const uint8_t b=bridge_src[pos>>1];
        const int nib=(pos&1u) ? (b>>4) : (b&0x0F);
        const int32_t step=plain_step[index];
        int32_t delta=step>>3;
        if(nib&4) delta+=step;
        if(nib&2) delta+=step>>1;
        if(nib&1) delta+=step>>2;
        pred+=(nib&8) ? -delta : delta;
        if(pred>32767) pred=32767; else if(pred<-32768) pred=-32768;
        index+=plain_adjust[nib];
        if(index<0) index=0; else if(index>88) index=88;
        bridge_out[i]=(int16_t)pred;
    }
    bridge_state.pred=pred;
    bridge_state.index=index;
}

struct voice { const uint8_t *adp; uint32_t nibbles,pos; int32_t pred,idx; uint8_t active; };
static struct voice v;
NOINLINE static int voice_step(struct voice *x) {
    uint8_t b=x->adp[x->pos>>1];
    int nib=(x->pos&1) ? (b>>4) : (b&0x0F);
    int step=plain_step[x->idx];
    int delta=step>>3;
    if(nib&4) delta+=step;
    if(nib&2) delta+=step>>1;
    if(nib&1) delta+=step>>2;
    x->pred+=(nib&8) ? -delta : delta;
    if(x->pred>32767) x->pred=32767; else if(x->pred<-32768) x->pred=-32768;
    x->idx+=plain_adjust[nib];
    if(x->idx<0) x->idx=0; else if(x->idx>88) x->idx=88;
    if(++x->pos>=x->nibbles) { x->active=0; return 0; }
    return 1;
}
NOINLINE void step_decode(uint32_t pos,uint32_t count) {
    v.adp=bridge_src; v.nibbles=pos+count; v.pos=pos; v.pred=bridge_state.pred; v.idx=bridge_state.index; v.active=1;
    for(uint32_t i=0;i<count;i++) { voice_step(&v); bridge_out[i]=(int16_t)v.pred; }
    bridge_state.pred=v.pred;
    bridge_state.index=v.idx;
}
