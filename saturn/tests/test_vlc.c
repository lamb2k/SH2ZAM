/* vlc-1 host tests (vlc_experimental.h) against the independent oracle in
 * vlc_oracle.h: builder sweep over every (w, lmax), optimality gap against
 * package-merge, build_lengths, round trips through exact-size heap buffers
 * (so ASan sees any read past a stream), truncated and random streams, random
 * and hostile tables, compatibility with the contributed builder, argument
 * contracts and negative controls. Alphabets are synthetic, generated here.
 * SH2Z_VLC_SCALE (default 1) multiplies the randomised counts.
 * MIT; SH2ZAM Contributors. */
#include "vlc_oracle.h"
#include <inttypes.h>
#include <stdio.h>

static uint64_t checks;
static void check(bool value,const char *what,unsigned line) {
    ++checks;
    if(!value) {
        fprintf(stderr,"vlc check %" PRIu64 " failed at line %u: %s\n",checks,line,what);
        exit(EXIT_FAILURE);
    }
}
#define CHECK(e) check((e),#e,__LINE__)
static uint32_t rng=UINT32_C(0x1badf00d);
static uint32_t rnd(void) {rng^=rng<<13;rng^=rng>>17;rng^=rng<<5;return rng;}
static unsigned below(unsigned n) {return n?rnd()%n:0u;}
static unsigned scale=1;

/* ---- alphabets ---------------------------------------------------------- */
enum {KINDS=11};
static const char *const kind_name[KINDS]={"uniform","zipf","fibonacci","equal","powers","single","two","huge+ones","random32","long tail","geometric"};
/* m symbols (1..256) at random positions with a distribution of the kind. */
static void gen_freq(uint32_t f[256],unsigned kind,unsigned m) {
    unsigned perm[256];
    for(unsigned i=0;i<256;i++)perm[i]=i;
    for(unsigned i=255;i>0;i--){const unsigned j=below(i+1);const unsigned x=perm[i];perm[i]=perm[j];perm[j]=x;}
    if(rnd()%4u==0u)for(unsigned i=0;i<256;i++)perm[i]=i;   /* sometimes in symbol order */
    memset(f,0,256*sizeof f[0]);
    if(kind==5)m=1;
    if(kind==6)m=2;
    uint64_t a=1,b=1;
    for(unsigned r=0;r<m;r++) {
        uint32_t v=1;
        switch(kind) {
        case 0:v=1u+below(1000);break;
        case 1:v=(uint32_t)(1000000u/(1u+r))+1u;break;
        case 2:v=(uint32_t)(a>UINT32_MAX?UINT32_MAX:a);{const uint64_t c=a+b;a=b;b=c;}break;   /* deepest Huffman trees */
        case 3:v=7;break;
        case 4:v=UINT32_C(1)<<(r%32u);break;
        case 5:v=1u+below(100000);break;
        case 6:v=1u+below(1u<<below(20));break;
        case 7:v=r?1u:UINT32_MAX;break;
        case 8:v=rnd()|1u;break;
        case 9:v=r<30u?(UINT32_C(1)<<(r/2u+1u)):1u;break;
        default:v=(uint32_t)(UINT32_C(1)<<(31u-(r*31u)/m));break;
        }
        f[perm[r]]=v;
    }
}
static unsigned rand_m(void) {
    switch(rnd()%6u){case 0:return 1u+below(4);case 1:return 256;case 2:return 255;case 3:return 1u+below(40);default:return 1u+below(256);}
}

/* ---- the contributed builder (commit f2f7c4c), kept verbatim apart from
 * names, for the compatibility check ---- */
static void contrib_huff_lengths(const uint32_t *freq,unsigned *len) {
    uint64_t wt[512]; int par[512], alive[512]; unsigned m = 0; int ids[256];
    for (unsigned s = 0; s < 256; s++) { len[s] = 0; if (freq[s]) { wt[m] = freq[s]; par[m] = -1; alive[m] = 1; ids[s] = (int)m; m++; } else ids[s] = -1; }
    if (m == 1) { for (unsigned s = 0; s < 256; s++) if (ids[s] >= 0) len[s] = 1; return; }
    unsigned total = m;
    for (unsigned k = 0; k + 1 < m; k++) {
        int a = -1, b = -1;
        for (unsigned i = 0; i < total; i++) if (alive[i]) { if (a < 0 || wt[i] < wt[(unsigned)a]) { b = a; a = (int)i; } else if (b < 0 || wt[i] < wt[(unsigned)b]) b = (int)i; }
        wt[total] = wt[(unsigned)a] + wt[(unsigned)b]; par[total] = -1; alive[total] = 1; alive[a] = alive[b] = 0; par[a] = par[b] = (int)total; total++;
    }
    for (unsigned s = 0; s < 256; s++) if (ids[s] >= 0) { unsigned d = 0; int x = ids[s]; while (par[x] >= 0) { x = par[x]; d++; } len[s] = d; }
}
static bool contrib_build(sh2z_vlc8_t *t,const uint32_t freq[256],unsigned w,unsigned lmax) {
    unsigned len[256], nsym = 0;
    if (w < 1 || w > 8 || lmax < w || lmax > 16) return false;
    for (unsigned s = 0; s < 256; s++) if (freq[s]) nsym++;
    if (!nsym || (nsym > ((unsigned)1u << lmax))) return false;
    contrib_huff_lengths(freq, len);
    {
        const uint32_t full = (uint32_t)1u << lmax; uint32_t kraft = 0;
        for (unsigned s = 0; s < 256; s++) if (len[s]) { if (len[s] > lmax) len[s] = lmax; kraft += full >> len[s]; }
        while (kraft > full) {
            int best = -1;
            for (unsigned s = 0; s < 256; s++) if (len[s] && len[s] < lmax && (best < 0 || len[s] > len[(unsigned)best] || (len[s] == len[(unsigned)best] && freq[s] < freq[(unsigned)best]))) best = (int)s;
            if (best < 0) return false;
            kraft -= full >> (len[(unsigned)best] + 1u); len[(unsigned)best]++;
        }
        for (int again = 1; again;) {
            int best = -1; again = 0;
            for (unsigned s = 0; s < 256; s++) if (len[s] > 1 && kraft + (full >> len[s]) <= full && (best < 0 || freq[s] > freq[(unsigned)best])) best = (int)s;
            if (best >= 0) { kraft += full >> len[(unsigned)best]; len[(unsigned)best]--; again = 1; }
        }
    }
    unsigned count[17] = {0}, first[17], codes[256];
    for (unsigned s = 0; s < 256; s++) count[len[s]]++;
    count[0] = 0; { unsigned c = 0; for (unsigned l = 1; l <= 16; l++) { c = (c + count[l - 1]) << 1; first[l] = c; } }
    unsigned next[17]; memcpy(next, first, sizeof next);
    for (unsigned s = 0; s < 256; s++) if (len[s]) {
        unsigned c = next[len[s]]++, rev = 0;
        for (unsigned b = 0; b < len[s]; b++) rev |= ((c >> b) & 1u) << (len[s] - 1u - b);
        codes[s] = rev;
    }
    memset(t, 0, sizeof *t);
    t->w = (uint8_t)w; t->lmax = (uint8_t)lmax; t->pool_len = 0;
    for (unsigned s = 0; s < 256; s++) if (len[s] && len[s] <= w) {
        for (unsigned e = codes[s]; e < ((unsigned)1u << w); e += (1u << len[s])) { t->s[e] = (uint8_t)s; t->n[e] = (uint8_t)(4u * (len[s] - 1u)); }
    }
    for (unsigned e = 0; e < ((unsigned)1u << w); e++) {
        unsigned xb = 0, used = 0;
        for (unsigned s = 0; s < 256; s++) if (len[s] > w && (codes[s] & (((unsigned)1u << w) - 1u)) == e) { used = 1; if (len[s] - w > xb) xb = len[s] - w; }
        if (!used) continue;
        if (xb > 7 || t->pool_len + (1u << xb) > SH2Z_VLC8_POOL_MAX || t->pool_len >= 4096u) return false;
        const unsigned off = t->pool_len; t->pool_len = (uint16_t)(t->pool_len + (1u << xb));
        t->s[e] = (uint8_t)(off & 255u); t->n[e] = (uint8_t)(0x80u | xb << 4 | (off >> 8));
        for (unsigned s = 0; s < 256; s++) if (len[s] > w && (codes[s] & (((unsigned)1u << w) - 1u)) == e) {
            const unsigned hi = codes[s] >> w, step = 1u << (len[s] - w);
            for (unsigned k = hi; k < (1u << xb); k += step) t->pool[off + k] = (uint16_t)(s << 8 | 4u * (len[s] - 1u));
        }
    }
    return true;
}

/* ---- builder sweep -------------------------------------------------------- */
static uint64_t builds,refusals,compat_tables,length_sets;
static uint64_t pm_limited,pm_optimal,pm_cases;static double pm_worst,pm_sum;static unsigned pm_worst_m,pm_worst_lmax,pm_kind,pm_worst_kind;
static sh2z_vlc8_t built,want,other;

static bool prefix_free(const unsigned len[256],const unsigned code[256]) {
    for(unsigned a=0;a<256;a++)if(len[a])for(unsigned b=0;b<256;b++)
        if(b!=a&&len[b]>=len[a]&&(code[b]&((1u<<len[a])-1u))==code[a])return false;
    return true;
}
/* One alphabet against every (w, lmax); returns the number of mismatches
 * with oracle mutation `mut` (0 = must be none, checked inline). */
static unsigned sweep_alphabet(const uint32_t f[256],unsigned mut) {
    unsigned mismatches=0,m=0;
    for(unsigned s=0;s<256;s++)m+=f[s]!=0;
    for(unsigned lmax=1;lmax<=16;lmax++) {
        unsigned olen[256],ocode[256];
        const bool fits=m<=(1u<<lmax);
        if(fits){vo_huffman(f,olen,mut);vo_limit(f,olen,lmax,mut);vo_canonical(olen,ocode,mut);}
        if(!mut&&fits) {   /* code properties, once per lmax */
            for(unsigned s=0;s<256;s++){CHECK((olen[s]!=0)==(f[s]!=0));CHECK(olen[s]<=lmax);}
            CHECK(vo_kraft16(olen)==(m==1?32768u:65536u));
            CHECK(prefix_free(olen,ocode));
            unsigned pm[256],huff[256];
            vo_package_merge(f,lmax,pm);vo_huffman(f,huff,0);
            const uint64_t c=vo_cost(f,olen),cp=vo_cost(f,pm);
            CHECK(c>=cp);
            CHECK(vo_kraft16(pm)<=65536u);
            unsigned deepest=0;for(unsigned s=0;s<256;s++)if(huff[s]>deepest)deepest=huff[s];
            if(deepest<=lmax)CHECK(c==vo_cost(f,huff)&&c==cp);
            else {
                ++pm_limited;pm_optimal+=c==cp;
                const double x=(double)(c-cp)/(double)cp;pm_sum+=x;if(x>pm_worst){pm_worst=x;pm_worst_m=m;pm_worst_lmax=lmax;pm_worst_kind=pm_kind;}
            }
            ++pm_cases;
        }
        for(unsigned w=1;w<=8&&w<=lmax;w++) {
            uint8_t len8[256];uint16_t code16[256];
            const bool ok=sh2z_vlc8_build(&built,f,w,lmax,len8,code16);
            const bool wok=fits&&vo_table(&want,olen,ocode,w,lmax,mut);
            if(mut){
                if(ok!=wok)mismatches++;
                else if(ok){bool same=vo_same_table(&built,&want);for(unsigned s=0;s<256;s++)same=same&&len8[s]==olen[s]&&code16[s]==ocode[s];mismatches+=!same;}
                continue;
            }
            ++builds;
            CHECK(ok==wok);
            if(!ok){++refusals;continue;}
            CHECK(vo_same_table(&built,&want));
            for(unsigned s=0;s<256;s++){CHECK(len8[s]==olen[s]);CHECK(code16[s]==ocode[s]);}
            CHECK(sh2z_vlc8_check(&built));CHECK(vo_check(&built));
            /* the same table from the reported lengths */
            uint16_t code_b[256];
            CHECK(sh2z_vlc8_build_lengths(&other,len8,w,lmax,code_b));
            CHECK(vo_same_table(&other,&built));
            CHECK(!memcmp(code_b,code16,sizeof code_b));
            /* the contributed builder makes the same table, except that a
             * one-symbol table now decodes both bit values to the symbol */
            CHECK(contrib_build(&other,f,w,lmax));
            if(m==1)for(unsigned e=1;e<(1u<<w);e+=2){CHECK(other.s[e]==0&&other.n[e]==0);other.s[e]=built.s[e];}
            CHECK(vo_same_table(&other,&built));
            ++compat_tables;
        }
    }
    return mismatches;
}
static void builder_sweep(void) {
    uint32_t f[256];
    for(unsigned kind=0;kind<KINDS;kind++)for(unsigned rep=0;rep<3u*scale;rep++) {
        gen_freq(f,kind,rep==0?256u:rep==1?1u+below(8):rand_m());
        pm_kind=kind;
        CHECK(sweep_alphabet(f,0)==0);
    }
    printf("vlc: builder sweep %" PRIu64 " (alphabet, w, lmax) tables equal to the oracle and to the contributed builder (%" PRIu64 " refusals agreed)\n",builds,refusals);
    printf("vlc: lengths vs package-merge: %" PRIu64 " cases, %" PRIu64 " needed limiting, %" PRIu64 " of those optimal, mean excess %.4f%%, worst %.4f%% (%s, %u symbols, lmax %u)\n",
        pm_cases,pm_limited,pm_optimal,pm_limited?100.0*pm_sum/(double)pm_limited:0.0,100.0*pm_worst,kind_name[pm_worst_kind],pm_worst_m,pm_worst_lmax);
}

/* ---- build_lengths ---------------------------------------------------------- */
/* A random complete code: split leaves until there are m (m >= 2), depth <= lmax. */
static void random_lengths(unsigned len[256],unsigned m,unsigned lmax) {
    unsigned depth[256],n=1;depth[0]=0;
    while(n<m) {
        unsigned tries=0,i;
        do i=below(n);while(depth[i]>=lmax&&++tries<1000);
        if(depth[i]>=lmax){for(i=0;i<n&&depth[i]>=lmax;i++){}if(i==n)break;}
        depth[i]++;depth[n++]=depth[i];
    }
    unsigned perm[256];for(unsigned i=0;i<256;i++)perm[i]=i;
    for(unsigned i=255;i>0;i--){const unsigned j=below(i+1);const unsigned x=perm[i];perm[i]=perm[j];perm[j]=x;}
    for(unsigned s=0;s<256;s++)len[s]=0;
    for(unsigned i=0;i<n;i++)len[perm[i]]=depth[i];
}
static void lengths_tests(void) {
    for(unsigned it=0;it<3000u*scale;it++) {
        const unsigned lmax=1u+below(16),w=1u+below(lmax<8u?lmax:8u);
        unsigned len[256],code[256];
        const unsigned cap=lmax>=8?256u:(1u<<lmax);
        random_lengths(len,2u+below(cap-1u),lmax);
        const unsigned kind=below(6);
        if(kind==1){unsigned s=below(256);len[s]=len[s]?0u:1u+below(lmax);}           /* incomplete or over-subscribed */
        if(kind==2){unsigned s;do s=below(256);while(!len[s]);len[s]=lmax+1u+below(4);} /* too long */
        if(kind==3){for(unsigned s=0;s<256;s++)len[s]=0;len[below(256)]=1u+below(2);}   /* one symbol */
        if(kind==4)for(unsigned s=0;s<256;s++)len[s]=0;                                  /* none */
        uint8_t len8[256];uint16_t code16[256];
        for(unsigned s=0;s<256;s++)len8[s]=(uint8_t)(len[s]>255u?255u:len[s]);
        const unsigned m=vo_count(len);
        bool valid=m>0;
        for(unsigned s=0;s<256;s++)valid=valid&&len[s]<=lmax;
        if(valid)valid=m==1?vo_kraft16(len)==32768u:vo_kraft16(len)==65536u;
        if(valid)vo_canonical(len,code,0);
        const bool wok=valid&&vo_table(&want,len,code,w,lmax,0);
        const bool ok=sh2z_vlc8_build_lengths(&built,len8,w,lmax,code16);
        CHECK(ok==wok);
        if(ok){CHECK(vo_same_table(&built,&want));for(unsigned s=0;s<256;s++)CHECK(code16[s]==code[s]);CHECK(sh2z_vlc8_check(&built));}
        ++length_sets;
    }
    printf("vlc: build_lengths %" PRIu64 " length sets (complete, incomplete, over-subscribed, too long, single, empty) as the oracle rules\n",length_sets);
}

/* ---- round trips, truncation, fuzz --------------------------------------- */
static uint64_t trips,symbols,bits_total,cuts,fuzz_streams,fuzz_symbols,chunked,random_tables,checked_tables,garbage_tables;
static uint8_t msg[40000],out[40000];

static uint8_t *exact_copy(const uint8_t *s,size_t units) {   /* exactly 2*units bytes, or NULL */
    if(!units)return NULL;
    uint8_t *p=(uint8_t *)malloc(2*units);
    if(!p){fprintf(stderr,"out of memory\n");exit(EXIT_FAILURE);}
    memcpy(p,s,2*units);
    return p;
}
/* Decode n symbols from a `units`-unit stream with both decoders (in one call
 * and, for the dispatching one, in random chunks) and compare everything with
 * the oracle's bit-at-a-time decode: symbols, reader state, return value.
 * Returns the number of mismatches when mut != 0. */
static unsigned decode_compare(const sh2z_vlc8_t *t,const unsigned len[256],const unsigned code[256],bool by_table,
                               const uint8_t *src,size_t units,size_t n,unsigned mut) {
    static uint8_t ref[40000];
    unsigned mismatches=0;
    uint8_t *s=exact_copy(src,units);
    uint8_t *dst=(uint8_t *)malloc(n?n:1u);
    const uint64_t used=by_table?vo_decode_table(t,s,units,0,ref,n):vo_decode(len,code,s,units,0,ref,n,mut);
    const vo_state_t st=vo_state(s,units,used,mut);
    for(int k=0;k<3;k++) {
        sh2z_vlc_reader_t r;sh2z_vlc_reader_init(&r,s,units);
        bool ok=true;
        if(k==0)ok=sh2z_vlc8_decode(t,&r,n?dst:NULL,n);
        else if(k==1)ok=sh2z_vlc8_decode_portable(t,&r,n?dst:NULL,n);
        else{for(size_t i=0;i<n;){size_t c=below(rnd()%3u?40u:2u);if(c>n-i)c=n-i;ok=sh2z_vlc8_decode(t,&r,c?dst+i:NULL,c);i+=c;}if(!n)ok=sh2z_vlc8_decode(t,&r,NULL,0);++chunked;}
        const bool where=units?(size_t)(r.p-s)==st.p&&r.end==s+2*units:r.p==s&&r.end==s;
        const bool same=!memcmp(dst,ref,n)&&where&&r.bb==st.bb&&r.pad==st.pad&&ok==st.ok&&sh2z_vlc_reader_ok(&r)==ok;
        if(mut)mismatches+=!same;
        else{CHECK(!memcmp(dst,ref,n));CHECK(where);CHECK(r.bb==st.bb);CHECK(r.pad==st.pad);CHECK(ok==st.ok);CHECK(sh2z_vlc_reader_ok(&r)==ok);}
    }
    free(dst);free(s);
    return mismatches;
}
/* A message from the table's alphabet (weighted by f or uniform), encoded with
 * the oracle's codes into buf; returns the units and sets *bits. */
static size_t encode_message(uint8_t *buf,size_t cap,const uint32_t f[256],const unsigned len[256],const unsigned code[256],size_t n,bool uniform,uint64_t *bits) {
    unsigned alphabet[256],m=0;uint64_t total=0;
    for(unsigned s=0;s<256;s++)if(len[s]){alphabet[m++]=s;total+=f[s];}
    memset(buf,0,cap);
    uint64_t at=0;
    for(size_t i=0;i<n;i++) {
        unsigned s=alphabet[0];
        if(uniform||!total)s=alphabet[below(m)];
        else{uint64_t x=((uint64_t)rnd()<<32|rnd())%total;for(unsigned j=0;j<m;j++){if(x<f[alphabet[j]]){s=alphabet[j];break;}x-=f[alphabet[j]];}}
        msg[i]=(uint8_t)s;at=vo_put(buf,at,code[s],len[s]);
    }
    *bits=at;
    return (size_t)((at+15)/16);
}
static uint8_t streambuf[100000];
static void round_trips(void) {
    uint32_t f[256];
    for(unsigned it=0;it<1500u*scale;it++) {
        gen_freq(f,below(KINDS),rand_m());
        const unsigned lmax=1u+below(16),w=1u+below(lmax<8u?lmax:8u);
        uint8_t len8[256];uint16_t code16[256];
        if(!sh2z_vlc8_build(&built,f,w,lmax,len8,code16))continue;
        unsigned len[256],code[256];for(unsigned s=0;s<256;s++){len[s]=len8[s];code[s]=code16[s];}
        size_t n;
        switch(rnd()%5u){case 0:n=below(3);break;case 1:n=1u+below(16);break;case 2:n=20000;break;default:n=1u+below(2000);}
        uint64_t bits;
        const size_t units=encode_message(streambuf,sizeof streambuf,f,len,code,n,rnd()%3u==0u,&bits);
        CHECK(decode_compare(&built,len,code,false,streambuf,units,n,0)==0);
        /* the round trip itself: the oracle decode of an untruncated stream is the message */
        {sh2z_vlc_reader_t r;uint8_t *s=exact_copy(streambuf,units);sh2z_vlc_reader_init(&r,s,units);
         CHECK(sh2z_vlc8_decode(&built,&r,out,n));CHECK(!memcmp(out,msg,n));
         free(s);}
        ++trips;symbols+=n;bits_total+=bits;
        /* truncations: every short length for small streams, a sample otherwise */
        for(size_t u=0;u<units;u+=(units<40u?1u:1u+below((unsigned)(units/8u)))){CHECK(decode_compare(&built,len,code,false,streambuf,u,n,0)==0);++cuts;}
        /* decoding past the message: more symbols than were written */
        CHECK(decode_compare(&built,len,code,false,streambuf,units,n+below(64)+1u,0)==0);
    }
    printf("vlc: %" PRIu64 " round trips, %" PRIu64 " symbols, %" PRIu64 " bits exact in one call and in %" PRIu64 " chunked decodes; %" PRIu64 " truncated streams as the oracle\n",
        trips,symbols,bits_total,chunked,cuts);
}
static void fuzz(void) {
    uint32_t f[256];
    /* random bytes through built tables */
    for(unsigned it=0;it<4000u*scale;it++) {
        gen_freq(f,below(KINDS),rand_m());
        const unsigned lmax=1u+below(16),w=1u+below(lmax<8u?lmax:8u);
        uint8_t len8[256];uint16_t code16[256];
        if(!sh2z_vlc8_build(&built,f,w,lmax,len8,code16))continue;
        unsigned len[256],code[256];for(unsigned s=0;s<256;s++){len[s]=len8[s];code[s]=code16[s];}
        const size_t units=below(5)==0?0u:below(64);
        for(size_t i=0;i<2*units;i++)streambuf[i]=(uint8_t)(rnd()%4u?rnd():(rnd()%2u?0u:255u));
        const size_t n=below(600);
        CHECK(decode_compare(&built,len,code,false,streambuf,units,n,0)==0);
        ++fuzz_streams;fuzz_symbols+=n;
    }
    /* random tables in the table format that pass the check: any contents */
    for(unsigned it=0;it<4000u*scale;it++) {
        memset(&built,0,sizeof built);
        built.w=(uint8_t)(1u+below(8));built.pool_len=(uint16_t)below(SH2Z_VLC8_POOL_MAX+1u);
        for(unsigned i=0;i<SH2Z_VLC8_POOL_MAX;i++)built.pool[i]=(uint16_t)(below(256)<<8|4u*below(16));
        for(unsigned e=0;e<(1u<<built.w);e++) {
            const unsigned xb=below(8);
            if(rnd()%3u||built.pool_len<(1u<<xb)){built.s[e]=(uint8_t)rnd();built.n[e]=(uint8_t)(4u*below(16));continue;}
            const unsigned start=below(built.pool_len-(1u<<xb)+1u);
            built.s[e]=(uint8_t)(start&255u);built.n[e]=(uint8_t)(0x80u|xb<<4|start>>8);
        }
        CHECK(sh2z_vlc8_check(&built));CHECK(vo_check(&built));
        const size_t units=below(48),n=below(500);
        for(size_t i=0;i<2*units;i++)streambuf[i]=(uint8_t)rnd();
        CHECK(decode_compare(&built,NULL,NULL,true,streambuf,units,n,0)==0);
        ++random_tables;
    }
    /* garbage tables: the check agrees with the oracle's rule; accepted ones decode safely */
    for(unsigned it=0;it<20000u*scale;it++) {
        uint8_t *b=(uint8_t *)&built;
        for(size_t i=0;i<sizeof built;i++)b[i]=(uint8_t)rnd();
        if(rnd()%2u){built.w=(uint8_t)(1u+below(8));built.pool_len=(uint16_t)below(SH2Z_VLC8_POOL_MAX+1u);}
        if(rnd()%2u)for(unsigned e=0;e<256;e++)if(!(built.n[e]&0x80u))built.n[e]&=0x3cu;
        if(rnd()%2u)for(unsigned i=0;i<SH2Z_VLC8_POOL_MAX;i++)built.pool[i]&=0xff3cu;
        const bool ok=sh2z_vlc8_check(&built);
        CHECK(ok==vo_check(&built));
        ++garbage_tables;
        if(!ok)continue;
        ++checked_tables;
        const size_t units=below(32),n=below(300);
        for(size_t i=0;i<2*units;i++)streambuf[i]=(uint8_t)rnd();
        CHECK(decode_compare(&built,NULL,NULL,true,streambuf,units,n,0)==0);
    }
    printf("vlc: fuzz %" PRIu64 " random streams (%" PRIu64 " symbols) through built tables, %" PRIu64 " random valid-format tables, %" PRIu64 " garbage tables (%" PRIu64 " accepted, decoded in bounds)\n",
        fuzz_streams,fuzz_symbols,random_tables,garbage_tables,checked_tables);
}

/* ---- contracts ---------------------------------------------------------- */
static void contracts(void) {
    uint32_t f[256];uint8_t len8[256];
    memset(f,0,sizeof f);f[1]=3;f[2]=4;
    CHECK(sh2z_vlc8_build(&built,f,5,8,NULL,NULL));
    CHECK(!sh2z_vlc8_build(NULL,f,5,8,NULL,NULL));CHECK(!sh2z_vlc8_build(&built,NULL,5,8,NULL,NULL));
    CHECK(!sh2z_vlc8_build(&built,f,0,8,NULL,NULL));CHECK(!sh2z_vlc8_build(&built,f,9,16,NULL,NULL));
    CHECK(!sh2z_vlc8_build(&built,f,6,5,NULL,NULL));CHECK(!sh2z_vlc8_build(&built,f,6,17,NULL,NULL));
    memset(f,0,sizeof f);CHECK(!sh2z_vlc8_build(&built,f,5,8,NULL,NULL));           /* no symbol */
    for(unsigned s=0;s<256;s++)f[s]=1;
    CHECK(!sh2z_vlc8_build(&built,f,4,7,NULL,NULL));                                  /* 256 symbols need 8 bits */
    CHECK(sh2z_vlc8_build(&built,f,4,8,len8,NULL));
    for(unsigned s=0;s<256;s++)CHECK(len8[s]==8);
    CHECK(sh2z_vlc8_build(&built,f,8,8,NULL,NULL)&&built.pool_len==0);
    /* codes longer than w + 7 bits are refused: w = 1 needs 15 extra bits, and no table holds a 16-bit code */
    for(unsigned s=0;s<256;s++)f[s]=(uint32_t)(s<17u?(UINT32_C(1)<<(31u-s)):1u);
    CHECK(!sh2z_vlc8_build(&built,f,1,16,NULL,NULL));
    CHECK(!sh2z_vlc8_build(&built,f,8,16,NULL,NULL));                                /* 16 > 8 + 7 */
    CHECK(sh2z_vlc8_build(&built,f,8,15,len8,NULL));
    {unsigned deepest=0;for(unsigned s=0;s<256;s++)if(len8[s]>deepest)deepest=len8[s];CHECK(deepest==15);}
    /* one symbol: one bit, both bit values decode to it, overrun after 16 bits per unit */
    memset(f,0,sizeof f);f[99]=1234;
    CHECK(sh2z_vlc8_build(&built,f,3,3,len8,NULL)&&len8[99]==1);
    for(unsigned e=0;e<8;e++)CHECK(built.s[e]==99&&built.n[e]==0);
    {const uint8_t two[4]={0xff,0x5a,0x00,0x81};uint8_t o[33];sh2z_vlc_reader_t r;
     sh2z_vlc_reader_init(&r,two,2);CHECK(sh2z_vlc8_decode(&built,&r,o,32));
     for(unsigned i=0;i<32;i++)CHECK(o[i]==99);
     CHECK(!sh2z_vlc8_decode(&built,&r,o,1));CHECK(!sh2z_vlc_reader_ok(&r));
     CHECK(!sh2z_vlc8_decode(&built,&r,NULL,0));}
    /* an empty stream: ok until a bit is consumed */
    {sh2z_vlc_reader_t r;sh2z_vlc_reader_init(&r,NULL,0);
     CHECK(sh2z_vlc_reader_ok(&r)&&r.pad==1&&r.bb==0x10000u);
     CHECK(sh2z_vlc8_decode(&built,&r,NULL,0));
     uint8_t o;CHECK(!sh2z_vlc8_decode(&built,&r,&o,1));CHECK(o==99);}
    /* build_lengths argument rules */
    memset(len8,0,sizeof len8);len8[0]=1;len8[1]=1;
    CHECK(sh2z_vlc8_build_lengths(&built,len8,1,1,NULL));
    CHECK(!sh2z_vlc8_build_lengths(NULL,len8,1,1,NULL));CHECK(!sh2z_vlc8_build_lengths(&built,NULL,1,1,NULL));
    CHECK(!sh2z_vlc8_build_lengths(&built,len8,2,1,NULL));CHECK(!sh2z_vlc8_build_lengths(&built,len8,1,17,NULL));
    len8[1]=2;CHECK(!sh2z_vlc8_build_lengths(&built,len8,1,2,NULL));                 /* incomplete */
    len8[2]=2;CHECK(sh2z_vlc8_build_lengths(&built,len8,1,2,NULL));
    len8[3]=2;CHECK(!sh2z_vlc8_build_lengths(&built,len8,1,2,NULL));                 /* over-subscribed */
    memset(len8,0,sizeof len8);len8[7]=2;CHECK(!sh2z_vlc8_build_lengths(&built,len8,2,2,NULL));   /* one symbol, 2 bits */
    CHECK(!sh2z_vlc8_check(NULL));
    printf("vlc: argument and edge contracts ok\n");
}

/* ---- negative controls ----------------------------------------------------- */
static void negative_controls(void) {
    static const struct {unsigned mut;const char *name;int stream;} controls[]={
        {VO_TIE_HIGH,"Huffman ties to the higher id",0},
        {VO_LENGTHEN_FREQ,"length limit lengthens the most frequent symbol",0},
        {VO_SHORTEN_LEAST,"spare Kraft room goes to the least frequent symbol",0},
        {VO_CANON_DESC,"canonical order (length, -symbol)",0},
        {VO_MSB_STREAM,"bits most significant first within a unit",1},
        {VO_OVERRUN_EARLY,"overrun flagged when the last real bit is consumed",1},
        {VO_PAD_NOSAT,"pad count does not saturate",1},
        {VO_SINGLE_HALF,"one-symbol table decodes bit 1 as symbol 0",2},
    };
    unsigned caught=0;
    for(unsigned c=0;c<sizeof controls/sizeof controls[0];c++) {
        const unsigned mut=controls[c].mut;unsigned found=0;
        uint32_t saved=rng;rng=UINT32_C(0xc0ffee11)+c;
        for(unsigned it=0;it<200&&!found;it++) {
            uint32_t f[256];
            if(controls[c].stream==0){gen_freq(f,it%KINDS,it%3u?rand_m():256u);found+=sweep_alphabet(f,mut);continue;}
            gen_freq(f,controls[c].stream==2?5u:below(KINDS),rand_m());
            const unsigned lmax=1u+below(16),w=1u+below(lmax<8u?lmax:8u);
            uint8_t len8[256];uint16_t code16[256];
            if(!sh2z_vlc8_build(&built,f,w,lmax,len8,code16))continue;
            unsigned len[256],code[256];for(unsigned s=0;s<256;s++){len[s]=len8[s];code[s]=code16[s];}
            /* an encoded message decoded exactly, then a halved stream decoded three times as far */
            const size_t n=1u+below(300);uint64_t bits;
            const size_t units=encode_message(streambuf,sizeof streambuf,f,len,code,n,true,&bits);
            if(controls[c].stream==2)for(size_t i=0;i<2*units;i++)streambuf[i]=(uint8_t)rnd();
            found+=decode_compare(&built,len,code,false,streambuf,units,n,mut);
            found+=decode_compare(&built,len,code,false,streambuf,units/2u,3u*n,mut);
        }
        rng=saved;
        CHECK(found>0);
        printf("vlc: negative control caught: %s\n",controls[c].name);
        caught++;
    }
    printf("vlc: %u of %u negative controls caught\n",caught,(unsigned)(sizeof controls/sizeof controls[0]));
}

int main(void) {
    const char *e=getenv("SH2Z_VLC_SCALE");
    if(e&&atoi(e)>0)scale=(unsigned)atoi(e);
    contracts();
    builder_sweep();
    lengths_tests();
    round_trips();
    fuzz();
    negative_controls();
    printf("vlc: %" PRIu64 " checks passed (kinds:",checks);
    for(unsigned k=0;k<KINDS;k++)printf(" %s",kind_name[k]);
    printf(")\n");
    return 0;
}
