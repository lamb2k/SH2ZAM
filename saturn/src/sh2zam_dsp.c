/* SCU DSP machine access and the batched affine job (dsp_experimental.h,
 * contract tag dsp-1). No libc.
 *
 * Copyright (c) 2026 SH2ZAM Contributors. MIT License.
 * Inspired by SH4ZAM by Falco Girgis and SH4ZAM Contributors.
 */
#include "sh2zam/dsp_experimental.h"

const uint32_t sh2z_dsp_xform_program[]={
#include "sh2zam_dsp_xform.inc"
};
const uint32_t sh2z_dsp_xform_program_words=(uint32_t)(sizeof(sh2z_dsp_xform_program)/sizeof(sh2z_dsp_xform_program[0]));
const uint32_t sh2z_dsp_scale16_program[]={
#include "sh2zam_dsp_scale16.inc"
};
const uint32_t sh2z_dsp_scale16_program_words=(uint32_t)(sizeof(sh2z_dsp_scale16_program)/sizeof(sh2z_dsp_scale16_program[0]));

#if SH2Z_DSP_NATIVE
#define DSP_PORT(a) (*(volatile uint32_t *)(uintptr_t)(a))

bool sh2z_dsp_load(const uint32_t *program,uint32_t words) {
    if(words>SH2Z_DSP_PROGRAM_WORDS || (words && !program))return false;
    DSP_PORT(SH2Z_DSP_PPAF_ADDRESS)=0u;                       /* stop */
    DSP_PORT(SH2Z_DSP_PPAF_ADDRESS)=SH2Z_DSP_PPAF_LOAD_PC;    /* program address 0 */
    for(uint32_t i=0;i<words;++i)DSP_PORT(SH2Z_DSP_PPD_ADDRESS)=program[i];
    return true;
}

static bool data_range(uint32_t bank,uint32_t index,const void *words,uint32_t count) {
    return bank<4u && index<SH2Z_DSP_BANK_WORDS && count<=SH2Z_DSP_BANK_WORDS-index && (words || !count);
}

bool sh2z_dsp_data_write(uint32_t bank,uint32_t index,const uint32_t *words,uint32_t count) {
    if(!data_range(bank,index,words,count))return false;
    DSP_PORT(SH2Z_DSP_PDA_ADDRESS)=bank<<6|index;
    for(uint32_t i=0;i<count;++i)DSP_PORT(SH2Z_DSP_PDD_ADDRESS)=words[i];
    return true;
}

bool sh2z_dsp_data_read(uint32_t bank,uint32_t index,uint32_t *words,uint32_t count) {
    if(!data_range(bank,index,words,count))return false;
    DSP_PORT(SH2Z_DSP_PDA_ADDRESS)=bank<<6|index;
    for(uint32_t i=0;i<count;++i)words[i]=DSP_PORT(SH2Z_DSP_PDD_ADDRESS);
    return true;
}

void sh2z_dsp_start(uint32_t pc) {
    DSP_PORT(SH2Z_DSP_PPAF_ADDRESS)=SH2Z_DSP_PPAF_EXECUTE|SH2Z_DSP_PPAF_LOAD_PC|(pc&0xFFu);
}

bool sh2z_dsp_busy(void) {
    return (DSP_PORT(SH2Z_DSP_PPAF_ADDRESS)&SH2Z_DSP_PPAF_EXECUTE)!=0u;
}

bool sh2z_dsp_wait(uint32_t polls) {
    for(uint32_t n=0;!polls || n<polls;++n)if(!sh2z_dsp_busy())return true;
    return false;
}

void sh2z_dsp_purge(const void *p,uint32_t bytes) {
    /* SH-2 associative purge: a write to 0x40000000 | address invalidates
     * the line holding that address, if any (16-byte lines). */
    const uintptr_t first=(uintptr_t)p&~(uintptr_t)15u;
    const uintptr_t end=(uintptr_t)p+bytes;
    for(uintptr_t a=first;a<end;a+=16u)
        *(volatile uint32_t *)((a&UINT32_C(0x1FFFFFF0))|UINT32_C(0x40000000))=0u;
}

bool sh2z_dsp_xform_load(void) {
    return sh2z_dsp_load(sh2z_dsp_xform_program,sh2z_dsp_xform_program_words);
}

void sh2z_dsp_xform_start(const sh2z_dsp_xform_t *job) {
    const uint32_t address=sh2z_dsp_d0(job);
    (void)sh2z_dsp_data_write(SH2Z_DSP_XFORM_JOB_BANK,SH2Z_DSP_XFORM_JOB_INDEX,&address,1u);
    sh2z_dsp_start(0u);
}

bool sh2z_dsp_xform_finish(const sh2z_dsp_xform_t *job,uint32_t polls) {
    if(!sh2z_dsp_wait(polls))return false;
    sh2z_dsp_purge(job->out,job->blocks*(3u*SH2Z_DSP_XFORM_BATCH*4u));
    return true;
}

bool sh2z_dsp_scale16_load(void) {
    return sh2z_dsp_load(sh2z_dsp_scale16_program,sh2z_dsp_scale16_program_words);
}

void sh2z_dsp_scale16_start(const sh2z_dsp_scale16_t *job) {
    const uint32_t address=sh2z_dsp_d0(job);
    (void)sh2z_dsp_data_write(SH2Z_DSP_SCALE16_JOB_BANK,SH2Z_DSP_SCALE16_JOB_INDEX,&address,1u);
    sh2z_dsp_start(0u);
}

bool sh2z_dsp_scale16_finish(const sh2z_dsp_scale16_t *job,uint32_t polls) {
    if(!sh2z_dsp_wait(polls))return false;
    sh2z_dsp_purge(job->out,job->n*job->m*4u);
    return true;
}
#else
bool sh2z_dsp_load(const uint32_t *program,uint32_t words) { (void)program;(void)words;return false; }
bool sh2z_dsp_data_write(uint32_t bank,uint32_t index,const uint32_t *words,uint32_t count) {
    (void)bank;(void)index;(void)words;(void)count;return false;
}
bool sh2z_dsp_data_read(uint32_t bank,uint32_t index,uint32_t *words,uint32_t count) {
    (void)bank;(void)index;(void)words;(void)count;return false;
}
void sh2z_dsp_start(uint32_t pc) { (void)pc; }
bool sh2z_dsp_busy(void) { return false; }
bool sh2z_dsp_wait(uint32_t polls) { (void)polls;return true; }
void sh2z_dsp_purge(const void *p,uint32_t bytes) { (void)p;(void)bytes; }
bool sh2z_dsp_xform_load(void) { return false; }

void sh2z_dsp_xform_start(const sh2z_dsp_xform_t *job) {
    sh2z_dsp_xform_cpu((const int32_t (*)[4])job->m,job->in,job->out,job->count);
}

bool sh2z_dsp_xform_finish(const sh2z_dsp_xform_t *job,uint32_t polls) { (void)job;(void)polls;return true; }
bool sh2z_dsp_scale16_load(void) { return false; }
void sh2z_dsp_scale16_start(const sh2z_dsp_scale16_t *job) {
    sh2z_dsp_scale16_cpu(job->a,job->n,job->b,job->m,job->out);
}
bool sh2z_dsp_scale16_finish(const sh2z_dsp_scale16_t *job,uint32_t polls) { (void)job;(void)polls;return true; }
#endif

bool sh2z_dsp_xform_prepare(sh2z_dsp_xform_t *job,const int32_t m[3][4],
    const int32_t *in,int32_t *out,uint32_t count) {
    if(!job || !m || !in || !out || !count || count>SH2Z_DSP_XFORM_MAX ||
       ((uintptr_t)in&3u) || ((uintptr_t)out&3u))return false;
    const uint32_t words=sh2z_dsp_xform_padded(count)*3u;
    const uintptr_t a=(uintptr_t)in,b=(uintptr_t)out,bytes=(uintptr_t)words*4u;
    if(a<b+bytes && b<a+bytes)return false;
    for(uint32_t r=0;r<3u;++r)for(uint32_t c=0;c<4u;++c)job->m[r][c]=m[r][c];
    job->in_d0=sh2z_dsp_d0(in);job->out_d0=sh2z_dsp_d0(out);
    job->blocks=words/(3u*SH2Z_DSP_XFORM_BATCH);job->count=count;
    job->in=in;job->out=out;
    return true;
}

void sh2z_dsp_xform_cpu(const int32_t m[3][4],const int32_t *in,int32_t *out,uint32_t count) {
    const uint32_t padded=sh2z_dsp_xform_padded(count);
    for(uint32_t k=0;k<padded;++k) {
        const uint32_t x=(uint32_t)in[3u*k],y=(uint32_t)in[3u*k+1u],z=(uint32_t)in[3u*k+2u];
        for(uint32_t r=0;r<3u;++r) {
            const uint32_t s=(uint32_t)m[r][0]*x+(uint32_t)m[r][1]*y+(uint32_t)m[r][2]*z+(uint32_t)m[r][3];
            out[sh2z_dsp_xform_at(k,r)]=(int32_t)s;
        }
    }
}

static uint64_t magnitude64(int64_t v) { return v<0?(uint64_t)0-(uint64_t)v:(uint64_t)v; }

bool sh2z_dsp_scale16_prepare(sh2z_dsp_scale16_t *job,const int32_t *a,uint32_t n,
    const int32_t *b,uint32_t m,int32_t *out) {
    if(!job || !a || !b || !out || !n || !m || n>SH2Z_DSP_SCALE16_MAX || m>SH2Z_DSP_SCALE16_MAX ||
       ((uintptr_t)out&3u))return false;
    const uintptr_t o=(uintptr_t)out,j=(uintptr_t)job,ob=(uintptr_t)n*m*4u;
    if(o<j+sizeof(*job) && j<o+ob)return false;
    uint64_t amax=0,bmax=0;
    for(uint32_t i=0;i<n;++i)if(magnitude64(a[i])>amax)amax=magnitude64(a[i]);
    for(uint32_t k=0;k<m;++k)if(magnitude64(b[k])>bmax)bmax=magnitude64(b[k]);
    if(amax && bmax>((UINT64_C(1)<<47)-1u)/amax)return false;
    job->n=n;job->m=m;job->out=out;
    for(uint32_t i=0;i<SH2Z_DSP_SCALE16_MAX;++i){job->a[i]=i<n?a[i]:0;job->b[i]=i<m?b[i]:0;}
    job->b_d0=sh2z_dsp_d0(job->b);job->out_d0=sh2z_dsp_d0(out);
    return true;
}

void sh2z_dsp_scale16_cpu(const int32_t *a,uint32_t n,const int32_t *b,uint32_t m,int32_t *out) {
    for(uint32_t i=0;i<n;++i)for(uint32_t k=0;k<m;++k) {
        const int64_t p=(int64_t)a[i]*b[k];
        const uint64_t q=p>=0?(uint64_t)p>>16:(((uint64_t)0-(uint64_t)p)+65535u)>>16;
        out[i*m+k]=p>=0?(int32_t)q:-(int32_t)q;
    }
}
