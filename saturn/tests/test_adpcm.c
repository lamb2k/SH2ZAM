/* IMA ADPCM: tables recomputed from the step table, then the fast decoder against
 * the reference on every (index, nibble, pred-boundary) state and on random streams
 * split at random points and alignments. MIT; SH2ZAM Contributors. */
#include "sh2zam/adpcm.h"
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static uint64_t checks;
#define CHECK(e) do { ++checks; if(!(e)) { fprintf(stderr,"FAIL %s:%d: %s\n",__FILE__,__LINE__,#e); exit(EXIT_FAILURE); } } while(0)
static uint32_t rng=UINT32_C(0x9e3779b9);
static uint32_t rnd(void) { rng^=rng<<13; rng^=rng>>17; rng^=rng<<5; return rng; }

static void tables(void) {
    for(int i=0;i<SH2Z_IMA_ROWS;i++) for(int n=0;n<8;n++) {
        const int s=sh2z_ima_step[i];
        int d=s>>3;
        if(n&4) d+=s;
        if(n&2) d+=s>>1;
        if(n&1) d+=s>>2;
        CHECK(sh2z_ima_diff[i*8+n]==d);
    }
}

/* every single-nibble transition from every index at pred values around both rails */
static void exhaustive_steps(void) {
    static const int32_t preds[]={-32768,-32767,-32000,-20000,-1,0,1,20000,32000,32766,32767};
    for(int idx=0;idx<SH2Z_IMA_ROWS;idx++) for(unsigned k=0;k<sizeof preds/sizeof *preds;k++)
        for(uint32_t nib=0;nib<16;nib++) for(uint32_t odd=0;odd<2;odd++) {
            const uint8_t src[1]={(uint8_t)(odd ? nib<<4 : nib)};
            sh2z_ima_state_t a={preds[k],idx},b=a;
            int16_t oa[1]={0x5555},ob[1]={0x5555};
            sh2z_ima_decode(&a,src,odd,oa,1);
            sh2z_ima_ref_decode(&b,src,odd,ob,1);
            CHECK(a.pred==b.pred && a.index==b.index && oa[0]==ob[0]);
        }
}

static void random_streams(unsigned iters) {
    static uint8_t src[600];
    static int16_t oa[1200+2],ob[1200+2];
    for(unsigned it=0;it<iters;it++) {
        const uint32_t mode=rnd()%4u;
        for(size_t i=0;i<sizeof src;i++) {
            uint32_t r=rnd();
            if(mode==1) r=(r&0x8888u) ? 0x77u : 0xFFu;           /* drive to the rails */
            else if(mode==2) r&=0x33u;                           /* small steps: low indexes */
            src[i]=(uint8_t)r;
        }
        const uint32_t start=rnd()%16u;
        const size_t total=(size_t)(rnd()%(uint32_t)(sizeof src*2u-start));
        sh2z_ima_state_t a={(int32_t)(rnd()%65536u)-32768,(int32_t)(rnd()%89u)},b=a;
        memset(oa,0x5A,sizeof oa); memset(ob,0x5A,sizeof ob);
        sh2z_ima_ref_decode(&b,src,start,ob,total);
        size_t done=0;
        while(done<total) {                    /* the fast path in pieces, ref in one go */
            size_t piece=(size_t)(rnd()%70u);
            if(piece>total-done) piece=total-done;
            sh2z_ima_decode(&a,src,start+(uint32_t)done,oa+done,piece);
            done+=piece;
        }
        CHECK(a.pred==b.pred && a.index==b.index);
        CHECK(memcmp(oa,ob,sizeof oa)==0);        /* includes the guard words past the end */
    }
}

int main(void) {
    tables();
    exhaustive_steps();
    const char *env=getenv("SH2Z_ADPCM_ITERS");
    random_streams(env ? (unsigned)strtoul(env,NULL,10) : 20000u);
    printf("adpcm: %" PRIu64 " checks passed\n",checks);
    return 0;
}
