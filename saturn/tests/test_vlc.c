/* Variable-length symbol decoding: random alphabets (synthetic, generated here), builder invariants, round trips
 * through an independent bit writer, a negative control and argument contracts. MIT; SH2ZAM Contributors. */
#include "sh2zam/vlc.h"
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static uint64_t checks;
#define CHECK(e) do { ++checks; if(!(e)) { fprintf(stderr,"FAIL %s:%d: %s\n",__FILE__,__LINE__,#e); exit(EXIT_FAILURE); } } while(0)
static uint32_t rng=UINT32_C(0x1badf00d);
static uint32_t rnd(void) { rng^=rng<<13; rng^=rng>>17; rng^=rng<<5; return rng; }

static void gen_freq(uint32_t f[256],unsigned kind) {
    memset(f,0,1024);
    switch(kind) {
    case 0: for(unsigned s=0;s<256;s++) f[s]=1+rnd()%1000u; break;                       /* near uniform */
    case 1: for(unsigned s=0;s<256;s++) f[s]=(uint32_t)(1000000.0/(1.0+s))+1u; break;       /* zipf */
    case 2: for(unsigned s=0;s<40;s++) f[(rnd()>>3)&255u]+=(uint32_t)(1u<<(rnd()%14u)); break;  /* few, wide range */
    case 3: for(unsigned s=0;s<256;s++) f[s]=s<30?(uint32_t)(1u<<(s/2u+1u)):1u; break;    /* long tail: deep codes */
    case 4: f[7]=5; f[200]=9; break;                                                     /* two symbols */
    case 5: f[99]=1234; break;                                                           /* one symbol */
    default: for(unsigned s=0;s<256;s++) f[s]=(rnd()%4u)?0u:1u+rnd()%50u; break;       /* sparse */
    }
    unsigned any=0; for(unsigned s=0;s<256;s++) any|=f[s]!=0; if(!any) f[0]=1;
}
static uint32_t pick(const uint32_t f[256],uint64_t total) {
    uint64_t x=((uint64_t)rnd()<<32|rnd())%total;
    for(unsigned s=0;s<256;s++) { if(x<f[s]) return s; x-=f[s]; }
    return 0;
}
typedef struct { uint8_t *b; size_t pos; uint32_t acc; unsigned nb; } writer_t;
static void put(writer_t *w,unsigned code,unsigned len) {       /* LSB first, 16-bit units stored low byte first */
    for(unsigned i=0;i<len;i++) {
        w->acc|=((code>>i)&1u)<<w->nb;
        if(++w->nb==16) { w->b[w->pos++]=(uint8_t)w->acc; w->b[w->pos++]=(uint8_t)(w->acc>>8); w->acc=0; w->nb=0; }
    }
}
static size_t finish(writer_t *w) {
    if(w->nb) { w->b[w->pos++]=(uint8_t)w->acc; w->b[w->pos++]=(uint8_t)(w->acc>>8); w->acc=0; w->nb=0; }
    memset(w->b+w->pos,0,8); return w->pos;
}
static unsigned prefix_free(const uint8_t *len,const uint16_t *code) {   /* independent check on the (len, code) pairs */
    for(unsigned a=0;a<256;a++) if(len[a]) for(unsigned b=0;b<256;b++) if(b!=a&&len[b]&&len[b]>=len[a]) {
        const unsigned m=(1u<<len[a])-1u; if((code[b]&m)==code[a]) return 0;
    }
    return 1;
}
static uint8_t msg[20032],dec[20032],bufw[60000];
static unsigned long long rounds; static int dumped;
static void dump_vector(const char *path,const sh2z_vlc8_t *t,const uint8_t *stream,size_t slen,const uint8_t *exp,size_t n) {   /* big endian table image + stream + expected symbols, for the SH-2 model driver */
    FILE *f=fopen(path,"wb"); if(!f) return; const size_t tlen=516u+2u*t->pool_len; uint8_t tb[516u+2u*SH2Z_VLC8_POOL_MAX]; memset(tb,0,sizeof tb);
    tb[0]=t->w; tb[1]=t->lmax; tb[2]=(uint8_t)(t->pool_len>>8); tb[3]=(uint8_t)t->pool_len; memcpy(tb+4,t->s,256); memcpy(tb+260,t->n,256);
    for(unsigned i=0;i<t->pool_len;i++) { tb[516+2*i]=(uint8_t)(t->pool[i]>>8); tb[517+2*i]=(uint8_t)t->pool[i]; }
    uint32_t h[3]={(uint32_t)n,(uint32_t)slen,(uint32_t)tlen}; for(int i=0;i<3;i++) { uint8_t b[4]={(uint8_t)(h[i]>>24),(uint8_t)(h[i]>>16),(uint8_t)(h[i]>>8),(uint8_t)h[i]}; fwrite(b,1,4,f); }
    fwrite(tb,1,tlen,f); fwrite(stream,1,slen,f); fwrite(exp,1,n,f); fclose(f);
}
static void round_trip(unsigned kind,unsigned w,unsigned lmax,size_t n) {
    static sh2z_vlc8_t t; uint32_t f[256]; uint8_t len[256]; uint16_t code[256]; uint64_t total=0; unsigned nsym=0;
    gen_freq(f,kind); for(unsigned s=0;s<256;s++) { total+=f[s]; nsym+=f[s]!=0; }
    if(nsym>(1u<<lmax)) { CHECK(!sh2z_vlc8_build(&t,f,w,lmax,len,code)); return; }
    const bool ok=sh2z_vlc8_build(&t,f,w,lmax,len,code);
    if(!ok) return;                                      /* pool overflow is a legal refusal; the next checks need a table */
    uint64_t kraft=0; for(unsigned s=0;s<256;s++) { CHECK((len[s]!=0)==(f[s]!=0)); CHECK(len[s]<=lmax); if(len[s]) kraft+=(uint64_t)1<<(16-len[s]); }
    CHECK(nsym==1?1:kraft==(uint64_t)1<<16);                                   /* complete code (one symbol: one bit) */
    CHECK(prefix_free(len,code));
    writer_t wr={bufw,0,0,0}; uint64_t bits=0;
    for(size_t i=0;i<n;i++) { const unsigned s=pick(f,total); msg[i]=(uint8_t)s; put(&wr,code[s],len[s]); bits+=len[s]; }
    const size_t bytes=finish(&wr);
    sh2z_bitr_t r; memset(dec,0xA5,sizeof dec); sh2z_bitr_init(&r,bufw); sh2z_vlc8_decode(&t,&r,dec,n);
    CHECK(!memcmp(dec,msg,n));
    { const char *dp=getenv("SH2Z_VLC_DUMP"); unsigned dk=1,dw=6,dl=12; { const char *dc=getenv("SH2Z_VLC_DUMP_CASE"); if(dc) sscanf(dc,"%u,%u,%u",&dk,&dw,&dl); } if(dp&&!dumped&&kind==dk&&w==dw&&lmax==dl&&n>=20000) { dump_vector(dp,&t,bufw,bytes,msg,n); dumped=1; } }
    CHECK(dec[n]==0xA5);                                                        /* nothing written beyond n */
    CHECK((size_t)(r.p-bufw)<=bytes+4u);                                       /* read at most 4 bytes beyond the stream */
    /* chunked decoding gives the same bytes as one call */
    sh2z_bitr_init(&r,bufw); memset(dec,0,n); for(size_t i=0;i<n;) { size_t k=1+rnd()%97u; if(i+k>n) k=n-i; sh2z_vlc8_decode(&t,&r,dec+i,k); i+=k; }
    CHECK(!memcmp(dec,msg,n));
    /* negative control: a flipped bit in the stream changes the output (several places, one may fall in padding bits) */
    if(n>=64&&bits>256) { unsigned caught=0; for(unsigned k=0;k<6&&!caught;k++) { const size_t at=(bytes/7u)*(k+1u); bufw[at]^=0x10; sh2z_bitr_init(&r,bufw); sh2z_vlc8_decode(&t,&r,dec,n); caught=memcmp(dec,msg,n)!=0; bufw[at]^=0x10; } CHECK(caught); }
    rounds++;
}
static void arguments(void) {
    static sh2z_vlc8_t t; uint32_t f[256]; memset(f,0,sizeof f); f[1]=3; f[2]=4;
    CHECK(sh2z_vlc8_build(&t,f,5,8,NULL,NULL));
    CHECK(!sh2z_vlc8_build(NULL,f,5,8,NULL,NULL)); CHECK(!sh2z_vlc8_build(&t,NULL,5,8,NULL,NULL));
    CHECK(!sh2z_vlc8_build(&t,f,0,8,NULL,NULL)); CHECK(!sh2z_vlc8_build(&t,f,9,16,NULL,NULL));
    CHECK(!sh2z_vlc8_build(&t,f,6,5,NULL,NULL)); CHECK(!sh2z_vlc8_build(&t,f,6,17,NULL,NULL));
    memset(f,0,sizeof f); CHECK(!sh2z_vlc8_build(&t,f,5,8,NULL,NULL));                 /* no symbol */
    for(unsigned s=0;s<256;s++) f[s]=1;
    CHECK(!sh2z_vlc8_build(&t,f,4,7,NULL,NULL));                                        /* 256 symbols need 8 bits */
    CHECK(sh2z_vlc8_build(&t,f,4,8,NULL,NULL));
}
int main(void) {
    arguments();
    for(unsigned kind=0;kind<7;kind++) for(unsigned w=1;w<=8;w++) for(unsigned lmax=w;lmax<=16;lmax+=(lmax<12?1u:2u)) for(unsigned rep=0;rep<2;rep++)
        round_trip(kind,w,lmax,(rep?20000u:1u+rnd()%300u));
    CHECK(rounds>300);
    printf("vlc: %" PRIu64 " checks, %llu round trips ok\n",checks,rounds);
    return 0;
}
