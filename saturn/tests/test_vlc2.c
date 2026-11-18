/* vlc-2 host tests (vlc2_experimental.h) against the independent oracle in
 * vlc2_oracle.h: every table build for alphabets of up to 6 symbols and
 * every window, random large alphabets, round trips through exact-size heap
 * buffers at every start parity (so ASan sees any read past the bound),
 * maximum-length codes and deep escapes, random and truncated streams under
 * the once-per-stream bound, hostile tables through the check, argument
 * contracts and negative controls. Alphabets are synthetic, generated here.
 * The SH-2 assembly path is checked by the instruction model
 * (benchmarks/vlc2-model); here the portable path runs.
 * SH2Z_VLC2_SCALE (default 1) multiplies the randomised counts.
 * MIT; SH2ZAM Contributors. */
#include "vlc2_oracle.h"
#include "vlc_oracle.h"
#include <inttypes.h>
#include <stdio.h>

static uint64_t checks;
static void check(bool value,const char *what,unsigned line) {
    ++checks;
    if(!value) {
        fprintf(stderr,"vlc2 check %" PRIu64 " failed at line %u: %s\n",checks,line,what);
        exit(EXIT_FAILURE);
    }
}
#define CHECK(e) check((e),#e,__LINE__)
static uint32_t rng=UINT32_C(0x5eed2c0d);
static uint32_t rnd(void) {rng^=rng<<13;rng^=rng>>17;rng^=rng<<5;return rng;}
static unsigned below(unsigned n) {return n?rnd()%n:0u;}
static unsigned scale=1;

static uint64_t n_exhaustive,n_accepted,n_random_builds,n_random_ok,n_symbols,n_streams,n_truncated,n_random_streams,n_garbage,n_garbage_ok,n_offsets;

/* ---- builds against the oracle ---- */
enum {TMAX=36864};   /* larger than any table: 1020 + 2^15 */
static uint8_t want_t[TMAX],got_t[TMAX],tight[TMAX];
/* Build with the library and the oracle; compare everything. Returns size. */
static size_t build_both(const uint8_t *len,unsigned nsym,unsigned w,unsigned *code) {
    const size_t want=vt_table(want_t,sizeof want_t,len,nsym,w,code,0);
    uint16_t code_lib[256];
    const size_t guard=(want?want:256u)+16u;   /* bytes watched for stray writes */
    memset(got_t,0xA5,guard);
    const size_t size=sh2z_vlc2_table_size(len,nsym,w);
    CHECK(size==want);
    const size_t got=sh2z_vlc2_build(got_t,sizeof got_t,len,nsym,w,code_lib);
    CHECK(got==want);
    if(!want) {
        for(size_t i=0;i<guard;i++)if(got_t[i]!=0xA5){check(false,"refused build wrote",__LINE__);break;}
        return 0;
    }
    CHECK(!memcmp(got_t,want_t,want));
    for(size_t i=want;i<guard;i++)CHECK(got_t[i]==0xA5);
    for(unsigned s=0;s<nsym;s++)CHECK(code_lib[s]==(len[s]?code[s]:0u));
    unsigned longest=0;for(unsigned s=0;s<nsym;s++)if(len[s]>longest)longest=len[s];
    CHECK(sh2z_vlc2_check(got_t,got,w)==longest);
    CHECK(vt_check(got_t,got,w)==longest);
    /* exact capacity works, one byte less is refused without writing */
    CHECK(sh2z_vlc2_build(tight,got,len,nsym,w,NULL)==got);
    memset(tight,0x5A,guard);
    CHECK(sh2z_vlc2_build(tight,got-1,len,nsym,w,NULL)==0);
    for(size_t i=0;i<guard;i++)CHECK(tight[i]==0x5A);
    return got;
}

/* ---- round trips ---- */
/* Encode `count` symbols drawn from the coded ones at a random offset of an
 * exact-size heap buffer, decode with the library, compare with the oracle
 * symbol by symbol and the reader state at the end. */
static void round_trip(const uint8_t *t,size_t tsize,const uint8_t *len,const unsigned *code,unsigned nsym,unsigned w,unsigned count,bool odd_base) {
    uint8_t *heap_t=(uint8_t *)malloc(tsize);
    memcpy(heap_t,t,tsize);   /* exact-size table copy: ASan sees any read past it */
    unsigned coded[256],m=0;for(unsigned s=0;s<nsym;s++)if(len[s])coded[m++]=s;
    const size_t offset=below(9);
    uint8_t *tmp=(uint8_t *)calloc(offset+16*(size_t)count+64,1);
    for(size_t i=0;i<offset;i++)tmp[i]=(uint8_t)rnd();
    unsigned *syms=(unsigned *)malloc(sizeof(unsigned)*(count+1));
    unsigned long long at=8ull*offset;
    const bool single=vt_kind(len,nsym)==2;
    for(unsigned i=0;i<count;i++) {
        syms[i]=coded[below(m)];
        if(single){if(rnd()&1u)tmp[at/8]|=(uint8_t)(1u<<(at%8));at++;}
        else at=vt_put(tmp,at,code[syms[i]],len[syms[i]]);
    }
    /* An odd base (raw + 1) shifts every address by one: the bound is
     * counted from the aligned address raw. */
    const size_t base=odd_base?1u:0u,consumed=(size_t)(at-8ull*offset);
    const size_t reach=sh2z_vlc2_reach(offset+base,consumed);
    CHECK(reach==vt_reach(offset+base,consumed,0));
    const size_t sbytes=reach-base;
    CHECK(sbytes>=(at+7)/8);
    uint8_t *raw=(uint8_t *)malloc(reach),*s=raw+base;   /* exactly the bound */
    memcpy(s,tmp,sbytes);
    sh2z_vlc2_reader_t r;
    sh2z_vlc2_begin(&r,s,offset);
    const int odd=(int)(((uintptr_t)(s+offset))&1u);
    unsigned long long ref_at=8ull*offset;
    for(unsigned i=0;i<count;i++) {
        const unsigned got=sh2z_vlc2_decode_portable(&r,heap_t,w);
        const unsigned want=vt_decode(len,code,nsym,s,sbytes,&ref_at,0);
        CHECK(want==syms[i]);
        CHECK(got==want);
        if(i%17u==0u||i+1==count) {
            const vt_state_t st=vt_state(s,sbytes,offset,odd,ref_at-8ull*offset,0);
            CHECK(r.bits==st.bits&&r.cnt==st.cnt&&r.p==s+st.end);
            CHECK(sh2z_vlc2_bitpos(&r,s)==ref_at);
        }
    }
    CHECK(sh2z_vlc2_bitpos(&r,s)==at);
    CHECK((size_t)(r.p-s)<=sbytes);
    n_symbols+=count;n_streams++;
    free(raw);free(tmp);free(syms);free(heap_t);
}

/* ---- 1: every build for alphabets of up to 6 symbols ---- */
static void exhaustive(void) {
    uint8_t len[6];unsigned code[6];
    for(unsigned nsym=1;nsym<=6;nsym++) {
        unsigned total=1;for(unsigned i=0;i<nsym;i++)total*=8;
        for(unsigned v=0;v<total;v++) {
            unsigned x=v;for(unsigned i=0;i<nsym;i++){len[i]=(uint8_t)(x%8);x/=8;}
            const int kind=vt_kind(len,nsym);
            for(unsigned w=1;w<=7;w++) {
                const size_t size=build_both(len,nsym,w,code);
                n_exhaustive++;
                CHECK((size!=0)==(kind!=0));
                if(!size)continue;
                n_accepted++;
                /* a short random message, both base parities */
                if(w==1||w==3||w==7||v%7u==0u)round_trip(got_t,size,len,code,nsym,w,2u+below(24),(v&1u)!=0);
            }
        }
    }
}

/* ---- 2: random large alphabets (lengths from the vlc-1 oracle's rule) ---- */
static void lengths_from_freq(uint8_t *len,unsigned nsym,unsigned lmax) {
    uint32_t f[256];unsigned l[256];
    memset(f,0,sizeof f);
    const unsigned kind=below(4);
    for(unsigned s=0;s<nsym;s++) {
        if(rnd()%5u==0u)continue;
        switch(kind){
        case 0:f[s]=1u+below(1000);break;
        case 1:f[s]=1000000u/(1u+below(nsym))+1u;break;
        case 2:f[s]=1u<<below(20);break;
        default:f[s]=s<16u?(1u<<(15u-s)):1u;break;}
    }
    unsigned m=0;for(unsigned s=0;s<nsym;s++)m+=f[s]!=0;
    if(!m)f[0]=1;
    vo_huffman(f,l,0);vo_limit(f,l,lmax,0);
    for(unsigned s=0;s<nsym;s++)len[s]=(uint8_t)l[s];
}
static void random_builds(void) {
    uint8_t len[256];unsigned code[256];
    for(unsigned it=0;it<4000u*scale;it++) {
        const unsigned nsym=1u+below(256),lmax=1u+below(15),w=1u+below(7);
        unsigned m=0;
        lengths_from_freq(len,nsym,lmax);
        for(unsigned s=0;s<nsym;s++)m+=len[s]!=0;
        if(m>(1u<<lmax)){it--;continue;}
        const size_t size=build_both(len,nsym,w,code);
        n_random_builds++;
        if(size){n_random_ok++;round_trip(got_t,size,len,code,nsym,w,below(300),(it&1u)!=0);}
    }
}

/* ---- 3: maximum lengths and deep escapes ---- */
static void long_codes(void) {
    uint8_t len[256];unsigned code[256];
    /* lengths 1, 2, ..., 14, 15, 15: every length up to the limit */
    for(unsigned s=0;s<16;s++)len[s]=(uint8_t)(s<15?s+1:15);
    for(unsigned w=1;w<=7;w++){const size_t size=build_both(len,16,w,code);CHECK(size!=0);round_trip(got_t,size,len,code,16,w,400,w&1u);}
    /* 2^k symbols of length k, reversed order, for k = 1..8 */
    for(unsigned k=1;k<=8;k++)for(unsigned w=1;w<=7;w++) {
        const unsigned n=1u<<k;
        for(unsigned s=0;s<n;s++)len[s]=(uint8_t)k;
        const size_t size=build_both(len,n,w,code);
        if(size)round_trip(got_t,size,len,code,n,w,300,0);
    }
    /* 128 symbols of length 15 + the rest shorter: big groups; pool limits */
    for(unsigned it=0;it<200u*scale;it++) {
        const unsigned nsym=2u+below(255),w=1u+below(7);
        lengths_from_freq(len,nsym,15);
        const size_t size=build_both(len,nsym,w,code);
        if(size)round_trip(got_t,size,len,code,nsym,w,200,it&1u);
    }
}

/* ---- 4: random bytes and truncated streams under the bound ---- */
static void hostile_streams(void) {
    uint8_t len[256];unsigned code[256];
    for(unsigned it=0;it<3000u*scale;it++) {
        const unsigned nsym=1u+below(256),w=1u+below(7);
        lengths_from_freq(len,nsym,1u+below(15));
        const size_t size=build_both(len,nsym,w,code);
        if(!size){it--;continue;}
        const unsigned longest=sh2z_vlc2_check(got_t,size,w);
        uint8_t *heap_t=(uint8_t *)malloc(size);memcpy(heap_t,got_t,size);
        const unsigned count=below(200);
        const size_t offset=below(7);
        const size_t bytes=sh2z_vlc2_reach(offset,(size_t)count*longest);
        CHECK(bytes==vt_reach(offset,(unsigned long long)count*longest,0));
        uint8_t *s=(uint8_t *)malloc(bytes);   /* exactly the bound */
        if(it%2u) {   /* random bytes */
            for(size_t i=0;i<bytes;i++)s[i]=(uint8_t)rnd();
            n_random_streams++;
        } else {      /* a message cut short: zeros after the cut */
            memset(s,0,bytes);
            unsigned long long at=8ull*offset;
            unsigned coded[256],m=0;for(unsigned x=0;x<nsym;x++)if(len[x])coded[m++]=x;
            for(unsigned i=0;i<count&&at+15<8ull*bytes;i++){const unsigned x=coded[below(m)];if(vt_kind(len,nsym)==2)at++;else at=vt_put(s,at,code[x],len[x]);}
            const size_t cut=offset+below((unsigned)(bytes-offset)+1u);
            for(size_t i=cut;i<bytes;i++)s[i]=0;
            n_truncated++;
        }
        sh2z_vlc2_reader_t r;sh2z_vlc2_begin(&r,s,offset);
        unsigned long long ref=8ull*offset;
        for(unsigned i=0;i<count;i++) {
            const unsigned got=sh2z_vlc2_decode_portable(&r,heap_t,w);
            const unsigned want=vt_decode_table(heap_t,w,s,bytes,&ref,0);
            CHECK(got==want);
        }
        const vt_state_t st=vt_state(s,bytes,offset,(int)(offset&1u),ref-8ull*offset,0);
        CHECK(r.bits==st.bits&&r.cnt==st.cnt&&r.p==s+st.end);
        CHECK((size_t)(r.p-s)<=bytes);
        CHECK(sh2z_vlc2_bitpos(&r,s)==ref);
        n_symbols+=count;
        free(s);free(heap_t);
    }
}

/* ---- 5: garbage tables through the check ---- */
static void garbage_tables(void) {
    for(unsigned it=0;it<40000u*scale;it++) {
        const unsigned w=below(9);
        const size_t size=below(600);
        uint8_t *t=(uint8_t *)malloc(size?size:1);
        const unsigned style=below(3);
        for(size_t i=0;i<size;i++) {
            if(style==0)t[i]=(uint8_t)rnd();
            else if(style==1)t[i]=(uint8_t)(rnd()%2u?4u*(1u+below(15)):64u+4u*(1u+below(6)));
            else t[i]=(uint8_t)(i%2u?4u*(1u+below(15)):below((unsigned)(size/4u)+2u));
        }
        if(style==2&&w>=1&&w<=7&&size>=(2u<<w))   /* plausible first level */
            for(unsigned e=0;e<(1u<<w);e++){if(rnd()%3u){t[(1u<<w)+e]=(uint8_t)(4u*(1u+below(15)));}else{t[(1u<<w)+e]=(uint8_t)(64u+4u*(1u+below(3)));t[e]=(uint8_t)below((unsigned)(size/4u)+1u);}}
        const unsigned got=sh2z_vlc2_check(size?t:NULL,size,w);
        CHECK(got==(size?vt_check(t,size,w):0u));
        n_garbage++;
        if(got) {
            n_garbage_ok++;
            const unsigned count=below(100);
            const size_t offset=below(5),bytes=sh2z_vlc2_reach(offset,(size_t)count*got);
            uint8_t *s=(uint8_t *)malloc(bytes);for(size_t i=0;i<bytes;i++)s[i]=(uint8_t)rnd();
            sh2z_vlc2_reader_t r;sh2z_vlc2_begin(&r,s,offset);
            unsigned long long ref=8ull*offset;
            for(unsigned i=0;i<count;i++)CHECK(sh2z_vlc2_decode_portable(&r,t,w)==vt_decode_table(t,w,s,bytes,&ref,0));
            CHECK(sh2z_vlc2_bitpos(&r,s)==ref);
            free(s);
        }
        free(t);
    }
}

/* ---- 6: begin at every offset parity of an aligned and an odd buffer ---- */
static void offsets(void) {
    uint8_t len[4]={2,2,2,2};unsigned code[4];
    const size_t size=build_both(len,4,6,code);
    CHECK(size==128);
    for(unsigned base=0;base<2;base++)for(size_t offset=0;offset<9;offset++) {
        round_trip(got_t,size,len,code,4,6,50,base!=0);
        uint8_t buf[64];for(unsigned i=0;i<64;i++)buf[i]=(uint8_t)rnd();
        sh2z_vlc2_reader_t r;sh2z_vlc2_begin(&r,buf+base,offset);
        const int odd=(int)(((uintptr_t)(buf+base+offset))&1u);
        const vt_state_t st=vt_state(buf+base,63,offset,odd,0,0);
        CHECK(r.bits==st.bits&&r.cnt==st.cnt&&r.p==buf+base+st.end);
        CHECK(((uintptr_t)r.p&1u)==0u);
        CHECK(sh2z_vlc2_bitpos(&r,buf+base)==8u*offset);
        CHECK(r.k==sh2z_vlc2_consts);
        n_offsets++;
    }
}

/* ---- 7: argument contracts and the constant block ---- */
static void contracts(void) {
    uint8_t len[257],t[512];
    memset(len,0,sizeof len);len[0]=1;len[1]=1;
    CHECK(sh2z_vlc2_build(t,sizeof t,len,2,6,NULL)==128);
    CHECK(sh2z_vlc2_build(t,sizeof t,len,0,6,NULL)==0);
    CHECK(sh2z_vlc2_build(t,sizeof t,len,257,6,NULL)==0);
    CHECK(sh2z_vlc2_build(t,sizeof t,len,2,0,NULL)==0);
    CHECK(sh2z_vlc2_build(t,sizeof t,len,2,8,NULL)==0);
    CHECK(sh2z_vlc2_build(NULL,sizeof t,len,2,6,NULL)==0);
    CHECK(sh2z_vlc2_build(t,sizeof t,NULL,2,6,NULL)==0);
    CHECK(sh2z_vlc2_table_size(NULL,2,6)==0);
    len[1]=16;len[0]=0;CHECK(sh2z_vlc2_table_size(len,2,6)==0);   /* too long */
    len[0]=2;len[1]=0;CHECK(sh2z_vlc2_table_size(len,2,6)==0);    /* one symbol, not length 1 */
    len[0]=1;len[1]=2;CHECK(sh2z_vlc2_table_size(len,2,6)==0);    /* incomplete */
    len[0]=1;len[1]=1;len[2]=1;CHECK(sh2z_vlc2_table_size(len,3,6)==0);  /* over-subscribed */
    CHECK(sh2z_vlc2_check(NULL,128,6)==0);
    CHECK(sh2z_vlc2_check(t,127,6)==0);
    for(unsigned i=1;i<16;i++)CHECK(sh2z_vlc2_consts[i]==UINT32_C(1)<<(32-i));
    for(unsigned i=16;i<32;i++)CHECK(sh2z_vlc2_consts[i]==1u);
    for(unsigned i=32;i<48;i++)CHECK(sh2z_vlc2_consts[i]==(UINT32_C(1)<<(i-32))-1u);
    for(unsigned i=48;i<64;i++)CHECK(sh2z_vlc2_consts[i]==UINT32_C(1)<<(i-48));
    CHECK(sh2z_vlc2_consts[0]==0);
    CHECK(sh2z_vlc2_reach(0,0)==2&&sh2z_vlc2_reach(1,0)==4&&sh2z_vlc2_reach(0,1)==4&&sh2z_vlc2_reach(0,16)==4&&sh2z_vlc2_reach(0,17)==6);
}

/* ---- 8: negative controls ---- */
/* Each mutated rule must disagree with the library somewhere in a sweep. */
static bool mutant_caught(unsigned mut) {
    uint8_t len[256];unsigned code[256],code_m[256];
    static uint8_t ta[TMAX],tb[TMAX];
    uint32_t keep=rng;rng=UINT32_C(0x0dd5eed5)+mut;
    bool caught=false;
    for(unsigned it=0;it<400&&!caught;it++) {
        const unsigned nsym=it<40u?1u+below(3):1u+below(256),w=1u+below(7);
        if(it<40u){memset(len,0,sizeof len);if(nsym==1)len[0]=1;else lengths_from_freq(len,nsym,15);}
        else lengths_from_freq(len,nsym,1u+below(15));
        const size_t a=sh2z_vlc2_build(ta,sizeof ta,len,nsym,w,NULL);
        const size_t b=vt_table(tb,sizeof tb,len,nsym,w,code_m,mut);
        vt_codes(len,nsym,code,0);
        if(a!=b||(a&&memcmp(ta,tb,a))){caught=true;break;}
        if(!a)continue;
        /* a message encoded with the true codes, decoded both ways */
        const size_t offset=below(6);
        uint8_t s[2048];memset(s,0,sizeof s);
        unsigned long long at=8ull*offset;
        unsigned coded[256],m=0;for(unsigned x=0;x<nsym;x++)if(len[x])coded[m++]=x;
        unsigned msg[64];
        for(unsigned i=0;i<64;i++){msg[i]=coded[below(m)];if(vt_kind(len,nsym)==2)at++;else at=vt_put(s,at,code[msg[i]],len[msg[i]]);}
        sh2z_vlc2_reader_t r;sh2z_vlc2_begin(&r,s,offset);
        unsigned long long ref=8ull*offset,ref_t=8ull*offset;
        for(unsigned i=0;i<64&&!caught;i++) {
            const unsigned got=sh2z_vlc2_decode_portable(&r,ta,w);
            if(got!=vt_decode(len,code_m,nsym,s,sizeof s,&ref,mut))caught=true;
            if(got!=vt_decode_table(ta,w,s,sizeof s,&ref_t,mut))caught=true;
        }
        if(caught)break;
        const vt_state_t st=vt_state(s,sizeof s,offset,(int)(offset&1u),ref-8ull*offset,mut);
        if(r.bits!=st.bits||r.cnt!=st.cnt||r.p!=s+st.end)caught=true;
        if(sh2z_vlc2_reach(offset,(size_t)(ref-8ull*offset))!=vt_reach(offset,ref-8ull*offset,mut))caught=true;
        if((size_t)(r.p-s)>vt_reach(offset,ref-8ull*offset,mut))caught=true;
    }
    rng=keep;
    return caught;
}
static unsigned negative_controls(void) {
    static const unsigned muts[]={VT_MSB,VT_CANON_DESC,VT_GROUP_ASC,VT_REFILL_LE16,VT_REACH_SHORT,VT_ESC_SHIFT,VT_ODD_NOALIGN,VT_SINGLE_ZERO};
    unsigned n=0;
    CHECK(!mutant_caught(0));   /* the unmutated oracle agrees */
    for(unsigned i=0;i<sizeof muts/sizeof muts[0];i++){CHECK(mutant_caught(muts[i]));n++;}
    return n;
}

int main(void) {
    const char *env=getenv("SH2Z_VLC2_SCALE");
    if(env&&atoi(env)>0)scale=(unsigned)atoi(env);
    contracts();
    offsets();
    exhaustive();
    random_builds();
    long_codes();
    hostile_streams();
    garbage_tables();
    const unsigned controls=negative_controls();
    printf("vlc2: %" PRIu64 " exhaustive builds (%" PRIu64 " accepted), %" PRIu64 " random builds (%" PRIu64 " accepted), %" PRIu64 " round trips (%" PRIu64 " symbols), %" PRIu64 " random and %" PRIu64 " truncated bounded streams, %" PRIu64 " garbage tables (%" PRIu64 " accepted), %" PRIu64 " begin offsets, %u negative controls caught, %" PRIu64 " checks\n",
        n_exhaustive,n_accepted,n_random_builds,n_random_ok,n_streams,n_symbols,n_random_streams,n_truncated,n_garbage,n_garbage_ok,n_offsets,controls,checks);
    return 0;
}
