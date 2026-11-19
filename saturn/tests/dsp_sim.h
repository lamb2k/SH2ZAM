/* Reference SCU DSP simulator for the host tests of dsp_experimental.h.
 * Test code only, not part of the library.
 *
 * Models the instruction set from the public SCU DSP description: a
 * one-instruction prefetch (so JMP and BTM have one delay slot), the ALU
 * computed first in an operation word, then the X, Y and D1 buses, with
 * MOV MUL,P taking the product of the RX and RY held before the word, D1
 * reads of ALL and ALH seeing this word's ALU result, CT post-increments
 * applied at the end of the word (a D1 write of CTn overrides its
 * increment). Choices that make the tests stricter than the hardware is
 * known to need:
 *   - a DMA completes dma_delay instructions after it starts (T0 set until
 *     then), and any access to the DMA's bank before then is a fault;
 *   - RA0 and WA0 are left unchanged by a DMA (no auto-advance), so a
 *     kernel must set them for every transfer;
 *   - LPS, MVI to PC, DMA to program RAM and a D1 write into a bank read
 *     by the same word are faults (no kernel needs them).
 * Each executed instruction counts one cycle (SCU clock); DMA transfer
 * time is not modelled.
 *
 * MIT; Copyright (c) 2026 SH2ZAM Contributors. */
#ifndef SH2ZAM_TEST_DSP_SIM_H
#define SH2ZAM_TEST_DSP_SIM_H

#include <stdint.h>
#include <string.h>

#define DSIM_MASK48 UINT64_C(0xFFFFFFFFFFFF)

typedef struct {
    uint32_t prog[256], ram[4][64];
    uint32_t ct[4], inc[4];
    uint32_t pc, next, top, lop;
    int running, fault;
    int s, z, c, v;
    uint64_t a, p, alu;
    int32_t rx, ry;
    uint32_t ra0, wa0;          /* byte addresses */
    /* DMA in flight */
    int dma, dma_to_d0, dma_bank;
    uint32_t dma_count, dma_add, dma_left;
    uint32_t dma_delay;
    /* D0 memory window: word w of mem is D0 byte address base + 4w */
    uint32_t *mem, mem_base, mem_words;
    uint64_t cycles, dma_words;
} dsim_t;

static uint64_t dsim_sext48(int64_t v) { return (uint64_t)v&DSIM_MASK48; }

static void dsim_init(dsim_t *d,uint32_t *mem,uint32_t mem_base,uint32_t mem_words) {
    memset(d,0,sizeof(*d));
    d->mem=mem;d->mem_base=mem_base;d->mem_words=mem_words;d->dma_delay=3;
}

static uint32_t *dsim_word(dsim_t *d,uint32_t addr) {
    if(addr&3u || addr<d->mem_base || (addr-d->mem_base)/4u>=d->mem_words){d->fault=10;return NULL;}
    return &d->mem[(addr-d->mem_base)/4u];
}

static void dsim_dma_finish(dsim_t *d) {
    uint32_t addr=d->dma_to_d0?d->wa0:d->ra0;
    for(uint32_t i=0;i<d->dma_count && !d->fault;++i) {
        uint32_t *w=dsim_word(d,addr);
        if(!w)return;
        uint32_t *r=&d->ram[d->dma_bank][d->ct[d->dma_bank]];
        if(d->dma_to_d0)*w=*r;else *r=*w;
        d->ct[d->dma_bank]=(d->ct[d->dma_bank]+1u)&63u;
        addr+=d->dma_add;++d->dma_words;
    }
    d->dma=0;
}

static void dsim_bank_use(dsim_t *d,uint32_t bank) {
    if(d->dma && d->dma_bank==(int)bank)d->fault=11;
}

static uint32_t dsim_read(dsim_t *d,uint32_t src) {
    if(src<8u) {
        const uint32_t n=src&3u;
        dsim_bank_use(d,n);
        if(src&4u)d->inc[n]=1u;
        return d->ram[n][d->ct[n]];
    }
    if(src==9u)return (uint32_t)d->alu;
    if(src==10u)return (uint32_t)(d->alu>>16);
    d->fault=12;return 0;
}

static void dsim_write(dsim_t *d,uint32_t dst,uint32_t v,int immediate_inc) {
    switch(dst) {
    case 0: case 1: case 2: case 3:
        dsim_bank_use(d,dst);
        d->ram[dst][d->ct[dst]]=v;
        if(immediate_inc)d->ct[dst]=(d->ct[dst]+1u)&63u;else d->inc[dst]=1u;
        break;
    case 4: d->rx=(int32_t)v;break;
    case 5: d->p=dsim_sext48((int32_t)v);break;
    case 6: d->ra0=(v<<2)&UINT32_C(0x07FFFFFC);break;
    case 7: d->wa0=(v<<2)&UINT32_C(0x07FFFFFC);break;
    case 10: d->lop=v&0xFFFu;break;
    case 11: d->top=v&0xFFu;break;
    case 12: case 13: case 14: case 15:
        dsim_bank_use(d,dst&3u);
        d->ct[dst&3u]=v&63u;d->inc[dst&3u]=0;break;
    default: d->fault=13;
    }
}

static int dsim_cond(const dsim_t *d,uint32_t cond) {
    int any=((cond&1u)&&d->z)||((cond&2u)&&d->s)||((cond&4u)&&d->c)||((cond&8u)&&d->dma);
    return any==((cond&0x20u)!=0);
}

static void dsim_operation(dsim_t *d,uint32_t w) {
    const uint32_t aluop=(w>>26)&15u,xop=(w>>23)&7u,xs=(w>>20)&7u,yop=(w>>17)&7u,ys=(w>>14)&7u;
    const uint32_t d1op=(w>>12)&3u,dst=(w>>8)&15u,imm=w&255u;
    uint32_t read_banks=0;
    const uint32_t al=(uint32_t)d->a,pl=(uint32_t)d->p;
    d->alu=d->a;
    switch(aluop) {
    case 0: break;
    case 1: case 2: case 3: {
        const uint32_t r=aluop==1?al&pl:aluop==2?al|pl:al^pl;
        d->alu=(d->a&~UINT64_C(0xFFFFFFFF))|r;d->z=r==0;d->s=(int32_t)r<0;d->c=0;break;
    }
    case 4: case 5: {
        const uint64_t r=aluop==4?(uint64_t)al+pl:(uint64_t)al-pl;
        const uint32_t r32=(uint32_t)r;
        d->alu=(d->a&~UINT64_C(0xFFFFFFFF))|r32;d->z=r32==0;d->s=(int32_t)r32<0;d->c=(int)((r>>32)&1u);
        if(aluop==4)d->v|=(int)((~(al^pl)&(al^r32))>>31);else d->v|=(int)(((al^pl)&(al^r32))>>31);
        break;
    }
    case 6: {
        const uint64_t r=d->a+d->p;
        d->alu=r&DSIM_MASK48;d->z=(r&DSIM_MASK48)==0;d->s=(int)((r>>47)&1u);d->c=(int)((r>>48)&1u);break;
    }
    case 8: d->c=(int)(al&1u);d->alu=(d->a&~UINT64_C(0xFFFFFFFF))|(uint32_t)((int32_t)al>>1);break;
    case 9: d->c=(int)(al&1u);d->alu=(d->a&~UINT64_C(0xFFFFFFFF))|((al>>1)|(al<<31));break;
    case 10: d->c=(int)(al>>31);d->alu=(d->a&~UINT64_C(0xFFFFFFFF))|(al<<1);break;
    case 11: d->c=(int)(al>>31);d->alu=(d->a&~UINT64_C(0xFFFFFFFF))|((al<<1)|(al>>31));break;
    case 15: d->c=(int)((al>>24)&1u);d->alu=(d->a&~UINT64_C(0xFFFFFFFF))|((al<<8)|(al>>24));break;
    default: d->fault=14;return;
    }
    if(aluop>=8u){const uint32_t r=(uint32_t)d->alu;d->z=r==0;d->s=(int32_t)r<0;}
    /* X bus */
    if((xop&3u)==2u)d->p=dsim_sext48((int64_t)d->rx*d->ry);
    if(xop>=3u) {
        const uint32_t v=dsim_read(d,xs);if(xs<8u)read_banks|=1u<<(xs&3u);
        if((xop&3u)==3u)d->p=dsim_sext48((int32_t)v);
        if(xop&4u)d->rx=(int32_t)v;
    }
    /* Y bus */
    if((yop&3u)==1u)d->a=0;
    else if((yop&3u)==2u)d->a=d->alu;
    if(yop>=3u) {
        const uint32_t v=dsim_read(d,ys);if(ys<8u)read_banks|=1u<<(ys&3u);
        if((yop&3u)==3u)d->a=dsim_sext48((int32_t)v);
        if(yop&4u)d->ry=(int32_t)v;
    }
    /* D1 bus */
    if(d1op==1u || d1op==3u) {
        uint32_t v;
        if(d1op==1u)v=(uint32_t)(int32_t)(int8_t)(uint8_t)imm;
        else {v=dsim_read(d,imm&15u);if((imm&15u)<8u)read_banks|=1u<<(imm&3u);}
        if(dst<4u && (read_banks&(1u<<dst)))d->fault=15;
        else if(dst==4u && (xop&4u))d->fault=16;
        else if(dst==5u && (xop&2u))d->fault=17;
        else dsim_write(d,dst,v,0);
    } else if(d1op==2u)d->fault=18;
    for(uint32_t n=0;n<4u;++n){if(d->inc[n])d->ct[n]=(d->ct[n]+1u)&63u;d->inc[n]=0;}
}

static void dsim_step(dsim_t *d) {
    const uint32_t w=d->next;
    if(d->dma && !--d->dma_left)dsim_dma_finish(d);
    d->next=d->prog[d->pc&255u];
    d->pc=(d->pc+1u)&255u;
    ++d->cycles;
    switch(w>>30) {
    case 0: dsim_operation(d,w);break;
    case 1: d->fault=19;break;
    case 2: {
        const uint32_t dst=(w>>26)&15u;
        uint32_t v;
        if(w&(1u<<25)){if(!dsim_cond(d,(w>>19)&63u))break;v=(uint32_t)((int32_t)(w<<13)>>13);}
        else v=(uint32_t)((int32_t)(w<<7)>>7);
        if(dst==12u)d->fault=25;   /* MVI to PC: not used by any kernel */
        else if(dst<=7u || dst==10u)dsim_write(d,dst,v,1);
        else d->fault=20;
        break;
    }
    case 3:
        switch((w>>28)&3u) {
        case 0: {   /* DMA */
            if(d->dma){d->fault=21;break;}
            const uint32_t bank=(w>>8)&7u;
            if(bank>3u){d->fault=22;break;}
            uint32_t count=w&255u;
            if(w&(1u<<13)){const uint32_t n=w&3u;dsim_bank_use(d,n);count=d->ram[n][d->ct[n]];
                if(w&4u)d->ct[n]=(d->ct[n]+1u)&63u;}
            const uint32_t add=(w>>15)&7u;
            d->dma_to_d0=(int)((w>>12)&1u);
            d->dma_add=d->dma_to_d0?((1u<<add)&~1u):((1u<<(add&2u))&~1u);
            d->dma_bank=(int)bank;d->dma_count=count;d->dma_left=d->dma_delay;d->dma=1;
            if(!count)d->dma=0;
            break;
        }
        case 1:     /* JMP */
            if(!(w&(1u<<25)) || dsim_cond(d,(w>>19)&63u))d->pc=w&255u;
            break;
        case 2:     /* BTM, LPS */
            if(w&(1u<<27)){d->fault=23;break;}
            if(d->lop){d->pc=d->top;}
            d->lop=(d->lop-1u)&0xFFFu;
            break;
        default:    /* END, ENDI */
            d->running=0;break;
        }
        break;
    }
}

/* Start at pc; run until END, a fault or max_steps. Returns 0 on END. */
static int dsim_run(dsim_t *d,uint32_t pc,uint64_t max_steps) {
    d->pc=pc&255u;d->next=0;d->running=1;d->fault=0;d->cycles=0;
    while(d->running && !d->fault) {
        if(d->cycles>=max_steps)return 100;
        dsim_step(d);
    }
    if(!d->fault && d->dma)d->fault=24;  /* ended with a DMA in flight */
    return d->fault;
}

#endif
