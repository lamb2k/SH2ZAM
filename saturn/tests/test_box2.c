/* Exact 2D integer box overlap oracle and buffer contracts. MIT; SH2ZAM Contributors.
 * The oracle uses int64 arithmetic on the mathematical definition of the closed
 * rectangles [min,max] / [c-h,c+h]; the library never widens. */
#include "sh2zam/box2.h"
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAXB 200u
static uint64_t checks;
#define CHECK(e) do { ++checks; if(!(e)) { fprintf(stderr,"FAIL %s:%d: %s\n",__FILE__,__LINE__,#e); exit(EXIT_FAILURE); } } while(0)
static uint32_t rng=UINT32_C(0x1b873593);
static uint32_t rnd(void) { rng^=rng<<13; rng^=rng>>17; rng^=rng<<5; return rng; }

static bool oracle_mm(const sh2z_ibox2_t *a,const sh2z_ibox2_t *b,int excl) {
    const int64_t ax0=a->min_x,ax1=a->max_x,ay0=a->min_y,ay1=a->max_y,bx0=b->min_x,bx1=b->max_x,by0=b->min_y,by1=b->max_y;
    if(excl) return ax0<bx1&&bx0<ax1&&ay0<by1&&by0<ay1;
    return ax0<=bx1&&bx0<=ax1&&ay0<=by1&&by0<=ay1;
}
static int64_t i64abs(int64_t v) { return v<0?-v:v; }
static bool oracle_c(const sh2z_ibox2c_t *a,const sh2z_ibox2c_t *b,int excl) {
    const int64_t dx=i64abs((int64_t)a->cx-b->cx),dy=i64abs((int64_t)a->cy-b->cy);
    const int64_t sx=(int64_t)a->hx+b->hx,sy=(int64_t)a->hy+b->hy;
    return excl ? dx<sx&&dy<sy : dx<=sx&&dy<=sy;
}
/* Different definition for valid boxes: centre form as the interval [c-h,c+h] in int64. */
static bool oracle_c_interval(const sh2z_ibox2c_t *a,const sh2z_ibox2c_t *b,int excl) {
    int64_t ax0=(int64_t)a->cx-a->hx,ax1=(int64_t)a->cx+a->hx,bx0=(int64_t)b->cx-b->hx,bx1=(int64_t)b->cx+b->hx;
    int64_t ay0=(int64_t)a->cy-a->hy,ay1=(int64_t)a->cy+a->hy,by0=(int64_t)b->cy-b->hy,by1=(int64_t)b->cy+b->hy;
    if(excl) return ax0<bx1&&bx0<ax1&&ay0<by1&&by0<ay1;
    return ax0<=bx1&&bx0<=ax1&&ay0<=by1&&by0<=ay1;
}

static int32_t coord(unsigned range) {
    switch(range) {
    case 0: return (int32_t)(rnd()%9u)-4;            /* tiny: many touching cases */
    case 1: return (int32_t)(rnd()%513u)-256;
    case 2: return (int32_t)(rnd()%65536u)-32768;    /* 16-bit world */
    case 3: { static const int32_t e[]={INT32_MIN,INT32_MIN+1,-1,0,1,INT32_MAX-1,INT32_MAX}; return e[rnd()%7u]; }
    default: return (int32_t)rnd();
    }
}
static void make_mm(sh2z_ibox2_t *b,unsigned range) {
    int32_t x0=coord(range),x1=coord(range),y0=coord(range),y1=coord(range);
    b->min_x=x0<x1?x0:x1; b->max_x=x0<x1?x1:x0; b->min_y=y0<y1?y0:y1; b->max_y=y0<y1?y1:y0;
}
static void make_c(sh2z_ibox2c_t *b,unsigned range) {
    b->cx=coord(range); b->cy=coord(range);
    uint32_t hx=(uint32_t)coord(range)&0x7FFFFFFFu,hy=(uint32_t)coord(range)&0x7FFFFFFFu;
    if(range<3) { hx%=300u; hy%=300u; if(range==0){hx%=4u;hy%=4u;} }
    b->hx=(int32_t)hx; b->hy=(int32_t)hy;
}
static size_t popcount_words(const uint32_t *w,size_t n) {
    size_t c=0; for(size_t i=0;i<n;i++) for(unsigned j=0;j<32;j++) c+=(w[i]>>j)&1u; return c;
}

static void check_row(const uint32_t *got,size_t words,const bool *want,size_t count) {
    for(size_t i=0;i<words*32u;i++) {
        const bool bit=(got[i>>5]>>(i&31u))&1u;
        if(i<count) CHECK(bit==want[i]); else CHECK(!bit);
    }
}

static void test_one_many(void) {
    static const size_t counts[]={0,1,2,31,32,33,63,64,65,95,96,97,127,128,129,MAXB};
    static sh2z_ibox2_t mm[MAXB],mq; static sh2z_ibox2c_t cc[MAXB],cq;
    static bool want[MAXB]; static uint32_t out[MAXB/32+4];
    for(unsigned iter=0;iter<4000;iter++) {
        const unsigned range=iter%5u; const size_t count=counts[(iter/5u)%(sizeof counts/sizeof counts[0])];
        for(int excl=0;excl<2;excl++) {
            const sh2z_box_edges_t e=excl?SH2Z_BOX_EXCLUSIVE:SH2Z_BOX_INCLUSIVE;
            const size_t words=sh2z_box_bitset_words(count);
            size_t hc=12345,expect=0;
            make_mm(&mq,range); for(size_t i=0;i<count;i++) { make_mm(&mm[i],range); if(rnd()%4u==0) mm[i]=mq; }
            for(size_t i=0;i<count;i++) { want[i]=oracle_mm(&mq,&mm[i],excl); expect+=want[i]; CHECK(sh2z_ibox2_overlap(&mq,&mm[i],e)==want[i]); }
            memset(out,0xA5,sizeof out);
            CHECK(sh2z_ibox2_overlap_one_many(out,words+1,&hc,&mq,mm,count,e));
            check_row(out,words,want,count); CHECK(hc==expect);
            CHECK(popcount_words(out,words)==expect);
            for(size_t i=words;i<MAXB/32+4;i++) CHECK(out[i]==UINT32_C(0xA5A5A5A5)); /* capacity beyond words untouched */
            CHECK(sh2z_ibox2_overlap_one_many(out,words,NULL,&mq,mm,count,e));
            { uint32_t u[MAXB/32+4]; memset(u,0x77,sizeof u);
              CHECK(sh2z_ibox2_overlap_one_many_unchecked(u,&mq,mm,count,e)==expect);
              for(size_t w=0;w<words;w++) CHECK(u[w]==out[w]);
              for(size_t w=words;w<MAXB/32+4;w++) CHECK(u[w]==UINT32_C(0x77777777)); }
            make_c(&cq,range); for(size_t i=0;i<count;i++) { make_c(&cc[i],range); if(rnd()%4u==0) cc[i]=cq; }
            expect=0;
            for(size_t i=0;i<count;i++) {
                want[i]=oracle_c(&cq,&cc[i],excl); expect+=want[i];
                CHECK(want[i]==oracle_c_interval(&cq,&cc[i],excl));
                CHECK(sh2z_ibox2c_overlap(&cq,&cc[i],e)==want[i]);
            }
            memset(out,0x5A,sizeof out);
            CHECK(sh2z_ibox2c_overlap_one_many(out,words,&hc,&cq,cc,count,e));
            check_row(out,words,want,count); CHECK(hc==expect);
            { uint32_t u[MAXB/32+4]; memset(u,0x77,sizeof u);
              CHECK(sh2z_ibox2c_overlap_one_many_unchecked(u,&cq,cc,count,e)==expect);
              for(size_t w=0;w<words;w++) CHECK(u[w]==out[w]); }
        }
    }
}
static void test_many_many(void) {
    static sh2z_ibox2_t a[40],b[MAXB]; static sh2z_ibox2c_t ac[40],bc[MAXB];
    static uint32_t bits[40*(MAXB/32+1)+2],row[MAXB/32+2];
    static const size_t bcounts[]={0,1,31,32,33,64,100,MAXB};
    for(unsigned iter=0;iter<400;iter++) {
        const size_t an=iter%41u,bn=bcounts[(iter/41u)%8u],rw=sh2z_box_bitset_words(bn);
        const unsigned range=iter%5u; const int excl=(int)(iter&1u); const sh2z_box_edges_t e=excl?SH2Z_BOX_EXCLUSIVE:SH2Z_BOX_INCLUSIVE;
        for(size_t i=0;i<an;i++) { make_mm(&a[i],range); make_c(&ac[i],range); }
        for(size_t i=0;i<bn;i++) { make_mm(&b[i],range); make_c(&bc[i],range); }
        size_t pc=777,expect=0;
        memset(bits,0xCC,sizeof bits);
        CHECK(sh2z_ibox2_overlap_many_many(bits,an*rw,&pc,a,an,b,bn,e));
        for(size_t r=0;r<an;r++) {
            CHECK(sh2z_ibox2_overlap_one_many(row,rw,NULL,&a[r],b,bn,e));
            for(size_t w=0;w<rw;w++) CHECK(bits[r*rw+w]==row[w]);
            for(size_t j=0;j<bn;j++) expect+=oracle_mm(&a[r],&b[j],excl);
        }
        CHECK(pc==expect);
        for(size_t i=an*rw;i<sizeof bits/sizeof bits[0];i++) CHECK(bits[i]==UINT32_C(0xCCCCCCCC));
        memset(bits,0xCC,sizeof bits); expect=0;
        CHECK(sh2z_ibox2c_overlap_many_many(bits,an*rw,&pc,ac,an,bc,bn,e));
        for(size_t r=0;r<an;r++) for(size_t j=0;j<bn;j++) {
            const bool bit=(bits[r*rw+(j>>5)]>>(j&31u))&1u; CHECK(bit==oracle_c(&ac[r],&bc[j],excl)); expect+=bit;
        }
        CHECK(pc==expect);
    }
}
static void test_semantics(void) {
    const sh2z_ibox2_t a={0,0,10,10},touch={10,0,20,10},corner={10,10,20,20},apart={11,0,20,10},inside={2,2,3,3},point={5,5,5,5};
    CHECK(sh2z_ibox2_overlap(&a,&touch,SH2Z_BOX_INCLUSIVE)&&!sh2z_ibox2_overlap(&a,&touch,SH2Z_BOX_EXCLUSIVE));
    CHECK(sh2z_ibox2_overlap(&a,&corner,SH2Z_BOX_INCLUSIVE)&&!sh2z_ibox2_overlap(&a,&corner,SH2Z_BOX_EXCLUSIVE));
    CHECK(!sh2z_ibox2_overlap(&a,&apart,SH2Z_BOX_INCLUSIVE)&&!sh2z_ibox2_overlap(&a,&apart,SH2Z_BOX_EXCLUSIVE));
    CHECK(sh2z_ibox2_overlap(&a,&inside,SH2Z_BOX_EXCLUSIVE)&&sh2z_ibox2_overlap(&a,&point,SH2Z_BOX_EXCLUSIVE));
    CHECK(sh2z_ibox2_overlap(&point,&point,SH2Z_BOX_INCLUSIVE)&&!sh2z_ibox2_overlap(&point,&point,SH2Z_BOX_EXCLUSIVE));
    const sh2z_ibox2c_t c1={0,0,5,5},c2={10,0,5,5},c3={11,0,5,5},big1={INT32_MIN,0,0,0},big2={INT32_MAX,0,INT32_MAX,0};
    CHECK(sh2z_ibox2c_overlap(&c1,&c2,SH2Z_BOX_INCLUSIVE)&&!sh2z_ibox2c_overlap(&c1,&c2,SH2Z_BOX_EXCLUSIVE));
    CHECK(!sh2z_ibox2c_overlap(&c1,&c3,SH2Z_BOX_INCLUSIVE));
    /* distance 2^32-1 == half-size sum 2^31-1 + 0 is not reachable; distance exceeds sum */
    CHECK(!sh2z_ibox2c_overlap(&big1,&big2,SH2Z_BOX_INCLUSIVE));
    const sh2z_ibox2c_t e1={INT32_MIN,0,INT32_MAX,0},e2={INT32_MAX,0,INT32_MAX,0}; /* dist 2^32-1, sum 2^32-2 */
    CHECK(!sh2z_ibox2c_overlap(&e1,&e2,SH2Z_BOX_INCLUSIVE));
    const sh2z_ibox2c_t e3={INT32_MIN,0,INT32_MAX,0},e4={INT32_MAX-1,0,INT32_MAX,0}; /* dist 2^32-2 == sum */
    CHECK(sh2z_ibox2c_overlap(&e3,&e4,SH2Z_BOX_INCLUSIVE)&&!sh2z_ibox2c_overlap(&e3,&e4,SH2Z_BOX_EXCLUSIVE));
}
static void test_arguments(void) {
    static sh2z_ibox2_t boxes[70]; static uint32_t hits[8],aligned_guard[8]; sh2z_ibox2_t q={0,0,1,1};
    size_t hc=9;
    for(unsigned i=0;i<70;i++) boxes[i]=q;
    memset(hits,0x11,sizeof hits);
    CHECK(!sh2z_ibox2_overlap_one_many(hits,2,&hc,&q,boxes,70,SH2Z_BOX_INCLUSIVE)); /* needs 3 words */
    CHECK(hits[0]==0x11111111u&&hc==9);
    CHECK(!sh2z_ibox2_overlap_one_many(NULL,3,&hc,&q,boxes,70,SH2Z_BOX_INCLUSIVE));
    CHECK(!sh2z_ibox2_overlap_one_many(hits,3,&hc,&q,NULL,70,SH2Z_BOX_INCLUSIVE));
    CHECK(!sh2z_ibox2_overlap_one_many(hits,3,&hc,NULL,boxes,70,SH2Z_BOX_INCLUSIVE));
    CHECK(!sh2z_ibox2_overlap_one_many(hits,3,&hc,&q,boxes,70,(sh2z_box_edges_t)7));
    CHECK(!sh2z_ibox2_overlap_one_many(hits,3,&hc,&q,boxes,(size_t)INT32_MAX+2u,SH2Z_BOX_INCLUSIVE));
    CHECK(!sh2z_ibox2_overlap_one_many((uint32_t*)(void*)((char*)boxes),3,&hc,&q,boxes,70,SH2Z_BOX_INCLUSIVE)); /* output overlaps input */
    CHECK(!sh2z_ibox2_overlap_one_many(hits,3,(size_t*)(void*)&hits[2],&q,boxes,70,SH2Z_BOX_INCLUSIVE)); /* count overlaps output */
    CHECK(!sh2z_ibox2_overlap_one_many((uint32_t*)((char*)hits+1),3,NULL,&q,boxes,70,SH2Z_BOX_INCLUSIVE)); /* misaligned */
    CHECK(hits[0]==0x11111111u&&hc==9);
    CHECK(sh2z_ibox2_overlap_one_many(NULL,0,&hc,NULL,NULL,0,SH2Z_BOX_INCLUSIVE)==false); /* query is required */
    CHECK(sh2z_ibox2_overlap_one_many(NULL,0,&hc,&q,NULL,0,SH2Z_BOX_INCLUSIVE)&&hc==0);
    CHECK(sh2z_ibox2_overlap_many_many(NULL,0,&hc,NULL,0,NULL,0,SH2Z_BOX_INCLUSIVE)&&hc==0);
    CHECK(sh2z_ibox2_overlap_many_many(NULL,0,&hc,boxes,5,NULL,0,SH2Z_BOX_EXCLUSIVE)&&hc==0);
    CHECK(!sh2z_ibox2_overlap_many_many(aligned_guard,7,&hc,boxes,5,boxes,40,SH2Z_BOX_INCLUSIVE)); /* needs 10 words */
    CHECK(!sh2z_ibox2_overlap_many_many(aligned_guard,8,&hc,boxes,SIZE_MAX/2u,boxes,64,SH2Z_BOX_INCLUSIVE));
}
int main(void) {
    test_semantics();
    test_one_many();
    test_many_many();
    test_arguments();
    printf("box2: %" PRIu64 " checks passed\n",checks);
    return 0;
}
