/* SCU DSP access and the affine job (dsp-1): the committed kernel words run
 * on a reference DSP simulator (dsp_sim.h) against the CPU form, with guard
 * words, timing counts and mutation controls; plus argument contracts and
 * the portable path.
 *
 * Copyright (c) 2026 SH2ZAM Contributors. MIT License.
 * Inspired by SH4ZAM by Falco Girgis and SH4ZAM Contributors.
 */
#include "sh2zam/dsp_experimental.h"
#include "dsp_sim.h"
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static uint64_t checks;
#define CHECK(e) do { ++checks; if(!(e)) { fprintf(stderr,"FAIL %s:%d: %s\n",__FILE__,__LINE__,#e); exit(EXIT_FAILURE); } } while(0)
static uint32_t rng = UINT32_C(0x9e3779b9);
static uint32_t rnd(void) { rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5; return rng; }

/* Simulated work RAM: D0 byte address 0x06000000 + 4 * word. */
#define MEM_WORDS 8192u
#define MEM_BASE UINT32_C(0x06000000)
#define JOB_AT 16u
#define IN_AT 64u
#define GUARD 0xA5A5A5A5u
static uint32_t mem[MEM_WORDS];

typedef struct { int fault; uint64_t cycles; uint32_t mismatches; int guards_ok; } run_t;

/* Kind of data: 0 full 32-bit random, 1 view-like (Q1.14 rows, int16 positions),
 * 2 extremes. */
static int32_t value(unsigned kind,int coefficient) {
    static const int32_t edge[]={0,1,-1,INT32_MAX,INT32_MIN,32767,-32768,16384,-16384,0x10000};
    switch(kind) {
    case 0: return (int32_t)rnd();
    case 1: return coefficient?(int32_t)(rnd()%32769u)-16384:(int32_t)(int16_t)rnd();
    default: return edge[rnd()%10u];
    }
}

/* Lays out job, input (padded) and output (padded) with guards in mem,
 * runs program on the simulator and compares every padded result with
 * sh2z_dsp_xform_cpu. */
static run_t run_job(const uint32_t *program,uint32_t words,uint32_t count,unsigned kind,uint32_t dma_delay) {
    run_t r={0,0,0,1};
    const uint32_t padded=sh2z_dsp_xform_padded(count),n=padded*3u;
    const uint32_t out_at=IN_AT+n+16u;
    CHECK(out_at+n+16u<=MEM_WORDS);
    for(uint32_t i=0;i<MEM_WORDS;++i)mem[i]=GUARD;
    int32_t m[3][4];
    for(unsigned i=0;i<3u;++i)for(unsigned j=0;j<4u;++j)m[i][j]=value(kind,j<3u);
    if(kind==1)for(unsigned i=0;i<3u;++i)m[i][3]=(int32_t)(rnd()%(1u<<24))-(1<<23);
    int32_t *in=malloc(n*4u),*want=malloc(n*4u),*dummy=malloc(n*4u);
    CHECK(in && want && dummy);
    for(uint32_t i=0;i<n;++i)in[i]=value(kind,0);
    sh2z_dsp_xform_t job;
    CHECK(sh2z_dsp_xform_prepare(&job,(const int32_t (*)[4])m,in,dummy,count));
    CHECK(job.blocks==padded/SH2Z_DSP_XFORM_BATCH && job.count==count);
    for(unsigned i=0;i<3u;++i)for(unsigned j=0;j<4u;++j)mem[JOB_AT+i*4u+j]=(uint32_t)job.m[i][j];
    mem[JOB_AT+12u]=(MEM_BASE>>2)+IN_AT;
    mem[JOB_AT+13u]=(MEM_BASE>>2)+out_at;
    mem[JOB_AT+14u]=job.blocks;
    mem[JOB_AT+15u]=job.count;
    for(uint32_t i=0;i<n;++i)mem[IN_AT+i]=(uint32_t)in[i];
    sh2z_dsp_xform_cpu((const int32_t (*)[4])m,in,want,count);

    dsim_t d;
    dsim_init(&d,mem,MEM_BASE,MEM_WORDS);
    d.dma_delay=dma_delay;
    memcpy(d.prog,program,words*4u);
    d.ram[SH2Z_DSP_XFORM_JOB_BANK][SH2Z_DSP_XFORM_JOB_INDEX]=(MEM_BASE>>2)+JOB_AT;
    r.fault=dsim_run(&d,0u,UINT64_C(5000000));
    r.cycles=d.cycles;
    for(uint32_t i=0;i<n;++i)if(mem[out_at+i]!=(uint32_t)want[i])++r.mismatches;
    for(uint32_t i=0;i<16u;++i)
        if(mem[out_at-16u+i]!=GUARD || mem[out_at+n+i]!=GUARD || mem[IN_AT+n+i]!=GUARD)r.guards_ok=0;
    for(uint32_t i=0;i<n;++i)if(mem[IN_AT+i]!=(uint32_t)in[i])r.guards_ok=0;
    free(in);free(want);free(dummy);
    return r;
}

static void test_kernel_exact(void) {
    static const uint32_t counts[]={1,2,19,20,21,39,40,41,64,68,100,133,256,300,1000};
    uint64_t vectors=0;
    for(unsigned kind=0;kind<3u;++kind)for(unsigned c=0;c<sizeof(counts)/sizeof(counts[0]);++c)
        for(uint32_t delay=1;delay<=81u;delay+=40u) {
            const run_t r=run_job(sh2z_dsp_xform_program,sh2z_dsp_xform_program_words,counts[c],kind,delay);
            if(r.fault || r.mismatches || !r.guards_ok)
                fprintf(stderr,"count %u kind %u delay %u: fault %d, %u mismatches, guards %d\n",
                        (unsigned)counts[c],kind,(unsigned)delay,r.fault,(unsigned)r.mismatches,r.guards_ok);
            CHECK(!r.fault && !r.mismatches && r.guards_ok);
            vectors+=sh2z_dsp_xform_padded(counts[c]);
        }
    for(unsigned rep=0;rep<200u;++rep) {
        const uint32_t count=1u+rnd()%400u;
        const run_t r=run_job(sh2z_dsp_xform_program,sh2z_dsp_xform_program_words,count,rep%3u,1u+rnd()%80u);
        CHECK(!r.fault && !r.mismatches && r.guards_ok);
        vectors+=sh2z_dsp_xform_padded(count);
    }
    printf("kernel: %" PRIu64 " vectors equal to the CPU form, guards intact\n",vectors);
}

/* Exact DSP cycles (instructions executed; DMA completing in one step). */
static void test_kernel_cycles(void) {
    const run_t one=run_job(sh2z_dsp_xform_program,sh2z_dsp_xform_program_words,20u,0u,1u);
    const run_t two=run_job(sh2z_dsp_xform_program,sh2z_dsp_xform_program_words,40u,0u,1u);
    const run_t many=run_job(sh2z_dsp_xform_program,sh2z_dsp_xform_program_words,320u,0u,1u);
    CHECK(!one.fault && !two.fault && !many.fault);
    const uint64_t block=two.cycles-one.cycles;
    CHECK(many.cycles==one.cycles+15u*block);
    printf("cycles: %u words; %" PRIu64 " for one block, %" PRIu64 " per further block (%.2f per vector)\n",
           (unsigned)sh2z_dsp_xform_program_words,one.cycles,block,(double)block/SH2Z_DSP_XFORM_BATCH);
    CHECK(block<=3u*(6u*SH2Z_DSP_XFORM_BATCH+4u)+40u);
}

/* Mutation controls: each altered kernel must fault, time out, differ from
 * the CPU form or break a guard. */
static uint32_t find(const uint32_t *p,uint32_t words,uint32_t want,uint32_t mask,unsigned nth) {
    for(uint32_t i=0;i<words;++i)if((p[i]&mask)==want && !nth--)return i;
    CHECK(0);
    return 0;
}

static void test_mutations(void) {
    const uint32_t words=sh2z_dsp_xform_program_words;
    struct { const char *what; uint32_t want,mask; unsigned nth; uint32_t flip,set; } m[]={
        {"pass 1 ADD -> SUB",          0x135D0000u,~0u,1,UINT32_C(1)<<26,0},
        {"no CLR A before a vector",   0x035B3209u,~0u,0,UINT32_C(1)<<17,0},
        {"pass 2 row base 8 -> 4",     0x11041C08u,~0u,0,0x0Cu,0},
        {"output DMA 60 -> 61 words",  0xC001123Cu,~0u,0,0x01u,0},
        {"input DMA 60 -> 59 words",   0xC001013Cu,~0u,0,0x07u,0},
        {"flush store dropped",        0x00003209u,~0u,0,0x00003209u,0},
        {"block loop NZ -> Z",         0xD2080000u,0xFFFFFF00u,0,UINT32_C(0x20)<<19,0},
        {"constant 1 -> 2",            0x8C000001u,~0u,0,0x03u,0},
        {"no wait for the input DMA",  0xD3400000u,0xFFFFFF00u,1,0,1},
    };
    unsigned caught=0;
    for(unsigned k=0;k<sizeof(m)/sizeof(m[0]);++k) {
        uint32_t p[256];
        memcpy(p,sh2z_dsp_xform_program,words*4u);
        const uint32_t at=find(p,words,m[k].want,m[k].mask,m[k].nth);
        if(m[k].set)p[at]=0u;else p[at]^=m[k].flip;
        const run_t r=run_job(p,words,64u,0u,64u);   /* a 60-word DMA takes tens of cycles */
        const int hit=r.fault || r.mismatches || !r.guards_ok;
        printf("control %-28s fault %3d, %4u mismatches, guards %s: %s\n",m[k].what,r.fault,
               (unsigned)r.mismatches,r.guards_ok?"ok":"broken",hit?"caught":"MISSED");
        caught+=(unsigned)hit;
    }
    CHECK(caught==sizeof(m)/sizeof(m[0]));
}

/* The simulator's own semantics on small programs with hand-derived results. */
static void test_simulator(void) {
    dsim_t d;
    /* Delay slot: JMP skips one word but executes the next. */
    static const uint32_t jump[]={0xD0000003u,0x00001501u /* MOV #1,PL */,0x00001407u /* MOV #7,RX: skipped */,
                                  0x00001402u /* MOV #2,RX */,0xF0000000u};
    dsim_init(&d,mem,MEM_BASE,MEM_WORDS);memcpy(d.prog,jump,sizeof(jump));
    CHECK(dsim_run(&d,0,100)==0 && d.rx==2 && (uint32_t)d.p==1u && d.cycles==5u);
    /* Multiply pipeline, ADD/AD2 and ALL/ALH: RX = 3, RY = 0x10000 * 5,
     * P = RX * RY next word; A = 0 + P; ALH = A >> 16 = 15. */
    static const uint32_t mac[]={
        0x8C000003u,                    /* MVI #3,MC3  (M3[0] = 3) */
        0x8C050000u,                    /* MVI #0x50000,MC3 (M3[1]) */
        0x00001F00u,                    /* MOV #0,CT3 */
        0x02700000u,                    /* MOV MC3,X */
        0x0009C000u,                    /* MOV MC3,Y */
        0x01020000u,                    /* MOV MUL,P | CLR A */
        0x18040000u,                    /* AD2 | MOV ALU,A */
        0x0000300Au | (2u<<8),          /* MOV ALH,MC2 */
        0x00003009u | (2u<<8),          /* MOV ALL,MC2 */
        0xF0000000u};
    dsim_init(&d,mem,MEM_BASE,MEM_WORDS);memcpy(d.prog,mac,sizeof(mac));
    CHECK(dsim_run(&d,0,100)==0);
    CHECK(d.ram[2][0]==15u && d.ram[2][1]==0xF0000u && d.ct[2]==2u && d.ct[3]==2u);
    /* BTM runs its body LOP + 1 times; the delay-slot word runs every time. */
    static const uint32_t loop[]={0xA8000004u /* MVI #4,LOP */,0x00001B03u /* MOV #3,TOP */,0x00001500u /* MOV #0,PL */,
                                  0x00001401u /* body: MOV #1,RX */,0xE0000000u /* BTM */,0x00001200u /* MOV #0,MC2 */,
                                  0xF0000000u};
    dsim_init(&d,mem,MEM_BASE,MEM_WORDS);memcpy(d.prog,loop,sizeof(loop));
    CHECK(dsim_run(&d,0,100)==0 && d.ct[2]==5u && d.lop==0xFFFu);
}

static void test_contracts(void) {
    int32_t m[3][4]={{1,2,3,4},{5,6,7,8},{9,10,11,12}};
    static int32_t in[60*3],out[60*3];
    sh2z_dsp_xform_t job;
    memset(&job,0x5A,sizeof(job));
    const sh2z_dsp_xform_t before=job;
    CHECK(!sh2z_dsp_xform_prepare(NULL,(const int32_t (*)[4])m,in,out,1));
    CHECK(!sh2z_dsp_xform_prepare(&job,NULL,in,out,1));
    CHECK(!sh2z_dsp_xform_prepare(&job,(const int32_t (*)[4])m,NULL,out,1));
    CHECK(!sh2z_dsp_xform_prepare(&job,(const int32_t (*)[4])m,in,NULL,1));
    CHECK(!sh2z_dsp_xform_prepare(&job,(const int32_t (*)[4])m,in,out,0));
    CHECK(!sh2z_dsp_xform_prepare(&job,(const int32_t (*)[4])m,in,out,SH2Z_DSP_XFORM_MAX+1u));
    CHECK(!sh2z_dsp_xform_prepare(&job,(const int32_t (*)[4])m,in,in+30,1));       /* overlap */
    CHECK(!sh2z_dsp_xform_prepare(&job,(const int32_t (*)[4])m,in,(int32_t *)((char *)out+2),1));
    CHECK(memcmp(&job,&before,sizeof(job))==0);
    CHECK(sh2z_dsp_xform_prepare(&job,(const int32_t (*)[4])m,in,in+60,20));         /* adjacent */
    CHECK(sh2z_dsp_xform_padded(1)==20u && sh2z_dsp_xform_padded(20)==20u && sh2z_dsp_xform_padded(21)==40u);
    CHECK(sh2z_dsp_xform_at(0,0)==0u && sh2z_dsp_xform_at(19,2)==59u && sh2z_dsp_xform_at(21,1)==81u);
    CHECK(sh2z_dsp_d0((const void *)(uintptr_t)UINT32_C(0x26001234))==UINT32_C(0x06001234)>>2);
    CHECK(sh2z_dsp_d0((const void *)(uintptr_t)UINT32_C(0x06001234))==UINT32_C(0x06001234)>>2);
    CHECK(sh2z_dsp_xform_program_words<=SH2Z_DSP_PROGRAM_WORDS);
    /* Portable: no DSP, the job runs on the CPU at start. */
    CHECK(!sh2z_dsp_load(sh2z_dsp_xform_program,1) && !sh2z_dsp_xform_load() && !sh2z_dsp_busy() && sh2z_dsp_wait(1));
    uint32_t w=0;
    CHECK(!sh2z_dsp_data_write(0,0,&w,1) && !sh2z_dsp_data_read(0,0,&w,1));
    for(unsigned i=0;i<60u*3u;++i)in[i]=(int32_t)rnd();
    int32_t want[60*3];
    CHECK(sh2z_dsp_xform_prepare(&job,(const int32_t (*)[4])m,in,out,41));
    sh2z_dsp_xform_start(&job);
    CHECK(sh2z_dsp_xform_finish(&job,1));
    sh2z_dsp_xform_cpu((const int32_t (*)[4])m,in,want,41);
    CHECK(memcmp(out,want,sizeof(want))==0);
    CHECK(out[sh2z_dsp_xform_at(5,1)]==(int32_t)((uint32_t)5*(uint32_t)in[15]+(uint32_t)6*(uint32_t)in[16]+
                                                  (uint32_t)7*(uint32_t)in[17]+8u));
}

/* ---- scale16 ---- */
static int32_t big_value(uint32_t limit_bits) {
    const uint32_t r=rnd();
    const int32_t v=(int32_t)(r>>(32u-limit_bits));
    return (rnd()&1u)?-v:v;
}

static run_t run_scale(const uint32_t *program,uint32_t words,const int32_t *a,uint32_t n,const int32_t *b,uint32_t m,uint32_t dma_delay) {
    run_t r={0,0,0,1};
    const uint32_t out_at=512u;
    for(uint32_t i=0;i<MEM_WORDS;++i)mem[i]=GUARD;
    static sh2z_dsp_scale16_t job;
    static int32_t dummy[256];
    CHECK(sh2z_dsp_scale16_prepare(&job,a,n,b,m,dummy));
    mem[JOB_AT+0u]=(MEM_BASE>>2)+JOB_AT+20u;
    mem[JOB_AT+1u]=(MEM_BASE>>2)+out_at;
    mem[JOB_AT+2u]=n;mem[JOB_AT+3u]=m;
    for(uint32_t i=0;i<16u;++i){mem[JOB_AT+4u+i]=(uint32_t)job.a[i];mem[JOB_AT+20u+i]=(uint32_t)job.b[i];}
    int32_t want[256];
    sh2z_dsp_scale16_cpu(a,n,b,m,want);
    dsim_t d;
    dsim_init(&d,mem,MEM_BASE,MEM_WORDS);
    d.dma_delay=dma_delay;
    memcpy(d.prog,program,words*4u);
    d.ram[SH2Z_DSP_SCALE16_JOB_BANK][SH2Z_DSP_SCALE16_JOB_INDEX]=(MEM_BASE>>2)+JOB_AT;
    r.fault=dsim_run(&d,0u,UINT64_C(1000000));
    r.cycles=d.cycles;
    for(uint32_t i=0;i<n*m;++i)if(mem[out_at+i]!=(uint32_t)want[i])++r.mismatches;
    for(uint32_t i=0;i<16u;++i)if(mem[out_at-16u+i]!=GUARD || mem[out_at+n*m+i]!=GUARD)r.guards_ok=0;
    return r;
}

static void test_scale16(void) {
    int32_t a[16],b[16];
    uint64_t products=0,refused=0;
    for(unsigned rep=0;rep<3000u;++rep) {
        const uint32_t n=1u+rnd()%16u,m=1u+rnd()%16u;
        const unsigned kind=rep%4u;
        for(uint32_t i=0;i<n;++i)a[i]=kind==0?(int32_t)rnd():kind==1?big_value(31):kind==2?(int32_t)(rnd()%201u)-100:big_value(30);
        for(uint32_t k=0;k<m;++k)b[k]=kind<=1?(int32_t)(rnd()%65537u):0;
        if(kind==3)for(uint32_t k=0;k<m;++k)b[k]=(rnd()&1u)?65536:-(int32_t)(rnd()%65537u);
        if(kind==2)for(uint32_t k=0;k<m;++k)b[k]=(int32_t)(rnd()>>1)-(int32_t)(rnd()>>1); /* small a, any b */
        sh2z_dsp_scale16_t job;
        int32_t out[256];
        if(!sh2z_dsp_scale16_prepare(&job,a,n,b,m,out)){++refused;continue;}   /* some |a b| >= 2^47 */
        const run_t r=run_scale(sh2z_dsp_scale16_program,sh2z_dsp_scale16_program_words,a,n,b,m,1u+rnd()%40u);
        if(r.fault || r.mismatches || !r.guards_ok)
            fprintf(stderr,"scale16 n %u m %u kind %u: fault %d, %u mismatches, guards %d\n",(unsigned)n,(unsigned)m,kind,r.fault,(unsigned)r.mismatches,r.guards_ok);
        CHECK(!r.fault && !r.mismatches && r.guards_ok);
        products+=n*m;
    }
    /* Edges of the 48-bit bound and of floor. */
    static const int32_t ea[]={INT32_MAX,INT32_MIN+1,1,-1,65535,-65535,0,65536};
    static const int32_t eb[]={65535,65536,-65536,1,-1,0,3,-3};
    const run_t r=run_scale(sh2z_dsp_scale16_program,sh2z_dsp_scale16_program_words,ea,8u,eb,8u,7u);
    CHECK(!r.fault && !r.mismatches && r.guards_ok);
    int32_t out[64];
    sh2z_dsp_scale16_cpu(ea,8u,eb,8u,out);
    CHECK(out[0*8+1]==INT32_MAX && out[3*8+4]==0 && out[3*8+1]==-1 && out[2*8+4]==-1 && out[4*8+0]==65534);
    CHECK(out[3*8+6]==-1 && out[2*8+7]==-1 && out[3*8+7]==0);
    const run_t full=run_scale(sh2z_dsp_scale16_program,sh2z_dsp_scale16_program_words,ea,8u,eb,8u,1u);
    const run_t row=run_scale(sh2z_dsp_scale16_program,sh2z_dsp_scale16_program_words,ea,1u,eb,8u,1u);
    printf("scale16: %u words; %" PRIu64 " products equal to the CPU form (%" PRIu64 " jobs refused by the 2^47 bound); %" PRIu64 " cycles for 8 x 8, %" PRIu64 " for 1 x 8\n",
           (unsigned)sh2z_dsp_scale16_program_words,products,refused,full.cycles,row.cycles);
    /* Contracts. */
    sh2z_dsp_scale16_t job;
    CHECK(!sh2z_dsp_scale16_prepare(&job,a,0,b,1,out) && !sh2z_dsp_scale16_prepare(&job,a,1,b,17,out));
    CHECK(!sh2z_dsp_scale16_prepare(&job,a,1,b,1,(int32_t *)((char *)out+1)) && !sh2z_dsp_scale16_prepare(NULL,a,1,b,1,out));
    a[0]=INT32_MIN;b[0]=65536;
    CHECK(!sh2z_dsp_scale16_prepare(&job,a,1,b,1,out));       /* |a b| = 2^47 */
    a[0]=INT32_MAX;
    CHECK(sh2z_dsp_scale16_prepare(&job,a,1,b,1,out));
    sh2z_dsp_scale16_start(&job);
    CHECK(sh2z_dsp_scale16_finish(&job,1) && out[0]==INT32_MAX && !sh2z_dsp_scale16_load());
    /* Mutation controls. */
    struct { const char *what; uint32_t want,mask; unsigned nth; uint32_t flip; } c[]={
        {"AD2 -> ADD (32-bit sum)",   0x18040000u,~0u,0,UINT32_C(2)<<26},
        {"store ALH -> ALL",          0x0000320Au,~0u,0,0x03u},
        {"no CLR A",                  0x010B4000u,~0u,0,UINT32_C(1)<<17},
        {"row loop NZ -> Z",          0xD2080000u,0xFFFFFF00u,0,UINT32_C(0x20)<<19},
        {"row count from M2 not M3",  0xC0013203u,~0u,0,0x01u},
        {"no wait for the b DMA",     0xD3400000u,0xFFFFFF00u,1,0},
    };
    unsigned caught=0;
    for(unsigned k=0;k<sizeof(c)/sizeof(c[0]);++k) {
        uint32_t p[256];
        memcpy(p,sh2z_dsp_scale16_program,sh2z_dsp_scale16_program_words*4u);
        uint32_t at=0;unsigned nth=c[k].nth;int found=0;
        for(uint32_t i=0;i<sh2z_dsp_scale16_program_words;++i)if((p[i]&c[k].mask)==c[k].want && !nth--){at=i;found=1;break;}
        CHECK(found);
        if(c[k].flip)p[at]^=c[k].flip;else p[at]=0u;
        for(uint32_t i=0;i<16u;++i){a[i]=big_value(31);b[i]=(int32_t)(rnd()%65537u);}
        const run_t rr=run_scale(p,sh2z_dsp_scale16_program_words,a,16u,b,16u,40u);
        const int hit=rr.fault || rr.mismatches || !rr.guards_ok;
        printf("control %-28s fault %3d, %4u mismatches, guards %s: %s\n",c[k].what,rr.fault,(unsigned)rr.mismatches,
               rr.guards_ok?"ok":"broken",hit?"caught":"MISSED");
        caught+=(unsigned)hit;
    }
    CHECK(caught==sizeof(c)/sizeof(c[0]));
}

int main(void) {
    test_scale16();
    test_simulator();
    test_contracts();
    test_kernel_exact();
    test_kernel_cycles();
    test_mutations();
    printf("dsp: %" PRIu64 " checks passed\n",checks);
    return 0;
}
