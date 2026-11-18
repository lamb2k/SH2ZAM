/* SPDX-License-Identifier: GPL-2.0-or-later
 * Executed-instruction checks and cycle model for the vlc-2 kernels
 * (vlc2_experimental.h). Runs the actual big-endian SH-2 object code in
 * Yabause's unmodified interpreter, like harness.c, whose loader and call
 * wrapper it reuses. Flat RAM, no caches or wait states: MODELLED cycles, not
 * Saturn timing; the optional placement model adds cache and region costs
 * beside them. Copyright (c) 2026 SH2ZAM Contributors. This optional model
 * harness is licensed separately from SH2ZAM's MIT core; see LICENSE.
 *
 * usage: vlc2_harness model.elf verify-scale > results.json
 * The ELF must contain bridge.o from benchmarks/vlc2-model and the library.
 * Every table, decoded symbol and reader state of every entry is checked
 * against the independent oracle in tests/vlc2_oracle.h, and every decoded
 * tile against the generator, before any timing is printed. Any read of
 * stream bytes at or past the bound sh2z_vlc2_reach gives, or of table bytes
 * past a checked table's size, stops the run. */
#define main sh2_model_main
#include "harness.c"
#undef main
#include "../../tests/vlc2_oracle.h"
#include "../../tests/vlc_oracle.h"

static void load_elf_symbols(const char *path) {
 FILE*f=fopen(path,"rb");if(!f){perror(path);exit(2);}
 fseek(f,0,SEEK_END);long length=ftell(f);rewind(f);
 unsigned char*b=malloc((size_t)length);if(!b||fread(b,1,(size_t)length,f)!=(size_t)length)exit(2);fclose(f);
 u32 shoff=be32(b+32);u16 shentsize=be16(b+46),shnum=be16(b+48);
 for(unsigned i=0;i<shnum;i++){
  const unsigned char*sh=b+shoff+(size_t)i*shentsize;
  if(be32(sh+4)!=2)continue; /* SHT_SYMTAB */
  u32 off=be32(sh+16),size=be32(sh+20),link=be32(sh+24);
  const unsigned char*str=b+be32(b+shoff+(size_t)link*shentsize+16);
  for(u32 p=off;p+16<=off+size;p+=16){
   const char*name=(const char*)str+be32(b+p);
   if(!*name||nsymbols>=2048)continue;
   if(strlen(name)>=sizeof symbols[0].name)continue;
   symbols[nsymbols].address=be32(b+p+4);strcpy(symbols[nsymbols++].name,name);
  }
 }
 free(b);
}
static struct cost call(const char *name,u32 a,u32 b,u32 c){char tmp[140];snprintf(tmp,sizeof tmp,"_%s",name);return invoke(tmp,a,b,c,0);}
static u32 symc(const char *name){char tmp[140];snprintf(tmp,sizeof tmp,"_%s",name);return sym(tmp);}
static void bad(const char *what,uint64_t it){fprintf(stderr,"vlc2 model mismatch: %s (case %"PRIu64")\n",what,it);exit(3);}

static uint32_t rng=UINT32_C(0x2c0de5a7);
static uint32_t rnd(void){rng^=rng<<13;rng^=rng>>17;rng^=rng<<5;return rng;}
static unsigned below(unsigned n){return n?rnd()%n:0u;}

/* ---- guarded reads: two forbidden ranges ---- */
static u32 g_lo[2],g_hi[2];static int guard_on;
static int guarded(u32 a,u32 n){if(!guard_on)return 0;for(int i=0;i<2;i++)if(a<g_hi[i]&&a+n>g_lo[i])return 1;return 0;}
static u8 g_rb(SH2_struct*sh,u32 a){if(guarded(a,1))fail("Read past a stream bound or a table",a);return rb(sh,a);}
static u16 g_rw(SH2_struct*sh,u32 a){if(guarded(a,2))fail("Read past a stream bound or a table",a);return rw(sh,a);}
static u32 g_rl(SH2_struct*sh,u32 a){if(guarded(a,4))fail("Read past a stream bound or a table",a);return rl(sh,a);}
static void guard_set(u32 slo,u32 shi,u32 tlo,u32 thi){g_lo[0]=slo;g_hi[0]=shi;g_lo[1]=tlo;g_hi[1]=thi;guard_on=1;}

enum {TMAX=36864,VSTREAM=16384,OUT_MAX=4608,ARENA=24576,STREAM_MAX=32768,NTILES=128};
static void put_bytes(const char *s,u32 off,const uint8_t *b,size_t n){const u32 a=symc(s)+off;for(size_t i=0;i<n;i++)wb(&cpu,a+(u32)i,b[i]);}
static void get_bytes(const char *s,u32 off,uint8_t *b,size_t n){const u32 a=symc(s)+off;for(size_t i=0;i<n;i++)b[i]=rb(&cpu,a+(u32)i);}

/* ---- lengths ---- */
/* Lengths from frequencies by the vlc-1 oracle's rule (input generation only). */
static void lengths_of(const uint32_t f[256],unsigned lmax,uint8_t *len){
 unsigned l[256];vo_huffman(f,l,0);vo_limit(f,l,lmax,0);
 for(unsigned s=0;s<256;s++)len[s]=(uint8_t)l[s];
}
static void random_freq(uint32_t f[256],unsigned nsym){
 memset(f,0,1024);const unsigned kind=below(4);
 for(unsigned s=0;s<nsym;s++){
  if(rnd()%5u==0u)continue;
  switch(kind){case 0:f[s]=1u+below(1000);break;case 1:f[s]=1000000u/(1u+below(nsym))+1u;break;
  case 2:f[s]=1u<<below(20);break;default:f[s]=s<16u?(1u<<(15u-s)):1u;break;}
 }
 unsigned m=0;for(unsigned s=0;s<nsym;s++)m+=f[s]!=0;if(!m)f[0]=1;
}

/* ---- verification ---- */
static uint64_t v_builds,v_build_refusals,v_streams,v_symbols,v_hostile,v_garbage,v_garbage_ok,v_mixed,v_odd;
static const char *const asm_dec[8]={0,"v2_asm_w1","v2_asm_w2","v2_asm_w3","v2_asm_w4","v2_asm_w5","v2_asm_w6","v2_asm_w7"};
static const char *const c_dec[8]={0,"v2_c_w1","v2_c_w2","v2_c_w3","v2_c_w4","v2_c_w5","v2_c_w6","v2_c_w7"};
static uint8_t tb[TMAX],sbuf[VSTREAM+8];

static void build_one(const uint8_t *len,unsigned nsym,unsigned w,uint64_t it,unsigned *code,size_t *size){
 put_bytes("lens",0,len,nsym);
 *size=vt_table(tb,TMAX,len,nsym,w,code,0);
 const u32 got=call("v2_build",nsym,w,0).result;
 if(got!=(u32)*size)bad("v2_build size",it);
 v_builds++;
 if(!*size){v_build_refusals++;return;}
 const u32 ta=symc("tbuf"),ca=symc("codes");
 for(size_t i=0;i<*size;i++)if(rb(&cpu,ta+(u32)i)!=tb[i])bad("v2_build table",it);
 for(unsigned s=0;s<nsym;s++)if(rw(&cpu,ca+2*s)!=(len[s]?code[s]:0u))bad("v2_build codes",it);
 unsigned longest=0;for(unsigned s=0;s<nsym;s++)if(len[s]>longest)longest=len[s];
 if(call("v2_check",(u32)*size,w,0).result!=longest)bad("v2_check of a built table",it);
}
/* Decode n symbols from sbuf (written at vstream + base) with both
 * decoders; the guard forbids stream reads at or past the bound and table
 * reads past size; compare symbols, state and bit position with the oracle.
 * table: 0 = the code (len, code), 1 = interpret the table. */
static void decode_both(const uint8_t *len,const unsigned *code,unsigned nsym,unsigned w,size_t size,size_t offset,unsigned base,size_t n,size_t bound_bits,uint64_t it,int interpret,const uint16_t *mixed){
 static uint8_t ref[OUT_MAX];
 const size_t bound=sh2z_vlc2_reach(offset+base,bound_bits);   /* from the aligned vstream */
 if(bound>VSTREAM)bad("stream too long",it);
 put_bytes("vstream",0,sbuf,bound);
 const uint8_t *s=sbuf+base;const size_t sbytes=bound-base;
 unsigned long long at=8ull*offset;
 for(size_t i=0;i<n;i++){
  if(mixed){const uint8_t *t=tb+mixed[i];ref[i]=(uint8_t)vt_decode_table(t,6,s,sbytes,&at,0);}
  else ref[i]=(uint8_t)(interpret?vt_decode_table(tb,w,s,sbytes,&at,0):vt_decode(len,code,nsym,s,sbytes,&at,0));
 }
 const int odd=(int)((offset+base)&1u);
 const vt_state_t st=vt_state(s,sbytes,offset,odd,at-8ull*offset,0);
 const u32 va=symc("vstream"),oa=symc("out"),ra=symc("rd"),ta=symc("tbuf");
 for(int k=0;k<2;k++){
  for(unsigned i=0;i<32;i++)wb(&cpu,oa+(u32)n+i,0xA5);
  guard_set(va+(u32)bound,va+VSTREAM+8,ta+(u32)size,ta+TMAX);
  const char *fn=mixed?(k?"v2_mixed_c":"v2_mixed_asm"):(k?c_dec[w]:asm_dec[w]);
  struct cost c=call(fn,(u32)offset,(u32)n,base);
  guard_on=0;
  if(c.result!=(u32)(at-0))bad(k?"portable bit position":"asm bit position",it);
  for(size_t i=0;i<n;i++)if(rb(&cpu,oa+(u32)i)!=ref[i])bad(k?"portable symbols":"asm symbols",it);
  for(unsigned i=0;i<32;i++)if(rb(&cpu,oa+(u32)n+i)!=0xA5)bad("write past out[n]",it);
  if(rl(&cpu,ra)!=st.bits||rl(&cpu,ra+4)!=st.cnt||rl(&cpu,ra+8)!=va+base+(u32)st.end||rl(&cpu,ra+12)!=symc("sh2z_vlc2_consts"))bad(k?"portable reader state":"asm reader state",it);
 }
 v_symbols+=n;v_streams++;if((offset+base)&1u)v_odd++;
}
static void verify(uint64_t scale){
 uint8_t len[256];unsigned code[256];
 /* every length vector of up to 4 symbols with lengths 0..4, every w */
 for(unsigned nsym=1;nsym<=4;nsym++){unsigned total=1;for(unsigned i=0;i<nsym;i++)total*=5;
  for(unsigned v=0;v<total;v++){unsigned x=v;for(unsigned i=0;i<nsym;i++){len[i]=(uint8_t)(x%5);x/=5;}
   for(unsigned w=1;w<=7;w++){size_t size;build_one(len,nsym,w,v,code,&size);
    if(size&&v%3u==0u){const size_t n=1u+below(40),offset=below(6);const unsigned base=below(2);
     memset(sbuf,0,sizeof sbuf);for(size_t i=0;i<offset+base;i++)sbuf[i]=(uint8_t)rnd();
     unsigned long long at=8ull*offset;unsigned coded[8],m=0;for(unsigned s=0;s<nsym;s++)if(len[s])coded[m++]=s;
     for(size_t i=0;i<n;i++){const unsigned s=coded[below(m)];if(vt_kind(len,nsym)==2){if(rnd()&1u)sbuf[base+at/8]|=(uint8_t)(1u<<(at%8));at++;}else at=vt_put(sbuf+base,at,code[s],len[s]);}
     decode_both(len,code,nsym,w,size,offset,base,n,(size_t)(at-8ull*offset),v,0,NULL);}}}}
 /* random alphabets: build, encode, decode (also past the message end) */
 for(uint64_t it=0;it<scale;it++){
  uint32_t f[256];const unsigned nsym=1u+below(256),lmax=1u+below(15),w=1u+below(7);
  random_freq(f,nsym);unsigned m=0;for(unsigned s=0;s<nsym;s++)m+=f[s]!=0;
  if(m>(1u<<lmax)){it--;continue;}
  lengths_of(f,lmax,len);size_t size;build_one(len,nsym,w,it,code,&size);
  if(!size)continue;
  unsigned longest=0;for(unsigned s=0;s<nsym;s++)if(len[s]>longest)longest=len[s];
  const size_t n=it%8u==0u?2048u:below(400),offset=below(9);const unsigned base=below(2);
  memset(sbuf,0,sizeof sbuf);for(size_t i=0;i<offset+base;i++)sbuf[i]=(uint8_t)rnd();
  unsigned long long at=8ull*offset;unsigned coded[256];m=0;for(unsigned s=0;s<nsym;s++)if(len[s])coded[m++]=s;
  for(size_t i=0;i<n;i++){const unsigned s=coded[below(m)];if(vt_kind(len,nsym)==2){if(rnd()&1u)sbuf[base+at/8]|=(uint8_t)(1u<<(at%8));at++;}else at=vt_put(sbuf+base,at,code[s],len[s]);}
  decode_both(len,code,nsym,w,size,offset,base,n,(size_t)(at-8ull*offset),it,0,NULL);
  /* the same message decoded 1..40 symbols past its end: bounded by count * longest */
  const size_t extra=1u+below(40);
  decode_both(len,code,nsym,w,size,offset,base,n+extra,(size_t)(at-8ull*offset)+extra*longest,it,0,NULL);
  /* random bytes under the once-per-stream bound */
  const size_t nr=below(500);
  for(size_t i=0;i<sizeof sbuf;i++)sbuf[i]=(uint8_t)rnd();
  decode_both(len,code,nsym,w,size,offset,base,nr,nr*longest,it,1,NULL);v_hostile++;
 }
 /* random tables in the format that pass the check (any contents) */
 for(uint64_t it=0;it<scale*4u;it++){
  const unsigned w=1u+below(7);const size_t size=(2u<<w)+below(400);
  for(size_t i=0;i<size;i++)tb[i]=(uint8_t)(i%2u?4u*(1u+below(15)):below((unsigned)(size/4u)+2u));
  for(unsigned e=0;e<(1u<<w);e++){if(rnd()%3u)tb[(1u<<w)+e]=(uint8_t)(4u*(1u+below(15)));else{tb[(1u<<w)+e]=(uint8_t)(64u+4u*(1u+below(3)));tb[e]=(uint8_t)below((unsigned)(size/4u)+1u);}}
  if(rnd()%4u==0u)tb[below((unsigned)size)]=(uint8_t)rnd();   /* sometimes a bad byte */
  put_bytes("tbuf",0,tb,size);
  const unsigned want=vt_check(tb,size,w);
  if(call("v2_check",(u32)size,w,0).result!=want)bad("v2_check",it);
  v_garbage++;
  if(!want)continue;
  v_garbage_ok++;
  const size_t n=below(300),offset=below(7);const unsigned base=below(2);
  for(size_t i=0;i<sizeof sbuf;i++)sbuf[i]=(uint8_t)rnd();
  decode_both(NULL,NULL,0,w,size,offset,base,n,n*want,it,1,NULL);
 }
 /* the table changes every symbol: 32 tables in one arena, w = 6 */
 for(uint64_t it=0;it<scale/4u+1u;it++){
  static uint8_t lens32[32][256];static unsigned codes32[32][256];static uint16_t off[32];static uint16_t mixed[OUT_MAX];
  size_t at_t=0;
  for(unsigned k=0;k<32;k++){
   uint32_t f[256];size_t size;
   do{random_freq(f,2u+below(255));lengths_of(f,12,lens32[k]);size=vt_table(tb+at_t,TMAX-at_t,lens32[k],256,6,codes32[k],0);}while(!size);
   off[k]=(uint16_t)at_t;at_t+=size;if(at_t>TMAX-2048)break;
  }
  put_bytes("tbuf",0,tb,at_t);
  const size_t n=1u+below(OUT_MAX-1u),offset=below(9);const unsigned base=below(2);
  memset(sbuf,0,sizeof sbuf);unsigned long long at=8ull*offset;
  const u32 ma=symc("mixed_off");
  for(size_t i=0;i<n;i++){
   const unsigned k=below(32);mixed[i]=off[k];ww(&cpu,ma+2*(u32)i,off[k]);
   unsigned s;do s=below(256);while(!lens32[k][s]);
   at=vt_put(sbuf+base,at,codes32[k][s],lens32[k][s]);
   if(at>8ull*(VSTREAM-64))bad("mixed stream too long",it);
  }
  decode_both(NULL,NULL,0,6,at_t,offset,base,n,(size_t)(at-8ull*offset),it,0,mixed);v_mixed++;
 }
}

/* ---- the synthetic tile codec (bridge.c describes the format) ---- */
struct mix {const char *name,*description;unsigned kind_w[4];};
static const struct mix mixes[]={
 {"shape","about 16 row, 4 dictionary and 50 pair symbols per tile",{300,180,125,350}},
 {"dense","about 16 row, 4 dictionary and 85 pair symbols per tile (about 105 symbols)",{100,115,125,500}},
};
struct codec {
 uint32_t rowf[16][16],dictf[256],pairf[32][256];
 uint8_t rowl[16][256],dictl[256],pairl[32][256];
 unsigned rowc[16][256],dictc[256],pairc[32][256];
 uint8_t arena[ARENA];uint16_t rowmap[16],ctxmap[256],dict_off;uint8_t klass16[256],ctx_table[256];uint8_t dict[256][4];
 size_t arena_used,hot_bytes;
};
static struct codec cd;
/* A table's frequencies, a code limited to 12 bits, its vlc-2 table in the
 * arena (refused tables are drawn again). */
static void zipf(uint32_t f[256],unsigned m,double s){
 unsigned perm[256];for(unsigned i=0;i<256;i++)perm[i]=i;
 for(unsigned i=255;i>0;i--){unsigned j=below(i+1),x=perm[i];perm[i]=perm[j];perm[j]=x;}
 memset(f,0,1024);for(unsigned r=0;r<m;r++)f[perm[r]]=(uint32_t)(1048576.0/pow(1.0+r,s))+1u;
}
static size_t place_table(const uint8_t *len,unsigned w,unsigned *code){
 const size_t size=vt_table(cd.arena+cd.arena_used,ARENA-cd.arena_used,len,256,w,code,0);
 if(!size)return SIZE_MAX;
 const size_t at=cd.arena_used;cd.arena_used+=size;return at;
}
static void make_codec(const struct mix *mx){
 memset(&cd,0,sizeof cd);
 rng=UINT32_C(0x7a11e5ed);
 /* row tables: P(row symbol | previous) from the mix's half-row kind
  * weights, with a repeat of the previous row symbol three times as likely */
 for(unsigned p=0;p<16;p++)for(unsigned r=0;r<16;r++)cd.rowf[p][r]=mx->kind_w[r&3u]*mx->kind_w[r>>2]*(r==p?3u:1u)+1u;
 zipf(cd.dictf,256,1.0);
 for(unsigned k=0;k<32;k++)zipf(cd.pairf[k],64u+below(193),1.05+0.5*(double)below(1000)/1000.0);
 for(unsigned p=0;p<16;p++){uint32_t f[256];memset(f,0,sizeof f);for(unsigned r=0;r<16;r++)f[r]=cd.rowf[p][r];
  lengths_of(f,12,cd.rowl[p]);const size_t at=place_table(cd.rowl[p],6,cd.rowc[p]);if(at==SIZE_MAX)bad("row table",p);cd.rowmap[p]=(uint16_t)at;}
 uint16_t pair_at[32];
 for(unsigned k=0;k<32;k++){
  size_t at;
  for(;;){lengths_of(cd.pairf[k],12,cd.pairl[k]);at=place_table(cd.pairl[k],6,cd.pairc[k]);if(at!=SIZE_MAX)break;zipf(cd.pairf[k],64u+below(193),1.3);}
  pair_at[k]=(uint16_t)at;
 }
 for(unsigned c=0;c<256;c++){cd.ctx_table[c]=(uint8_t)below(32);cd.ctxmap[c]=pair_at[cd.ctx_table[c]];}
 lengths_of(cd.dictf,12,cd.dictl);{const size_t at=place_table(cd.dictl,7,cd.dictc);if(at==SIZE_MAX)bad("dictionary table",0);cd.dict_off=(uint16_t)at;}
 for(unsigned b=0;b<256;b++)cd.klass16[b]=(uint8_t)(16u*below(16));
 for(unsigned d=0;d<256;d++)for(unsigned j=0;j<4;j++)cd.dict[d][j]=(uint8_t)rnd();
 /* hot part: first levels (16 row, 32 pair, the dictionary's), maps, class table, dictionary */
 cd.hot_bytes=16u*128u+32u*128u+256u+32u+512u+256u+1024u;
}
static unsigned sample(const uint32_t *f,unsigned n){
 uint64_t total=0;for(unsigned s=0;s<n;s++)total+=f[s];
 uint64_t x=((uint64_t)rnd()<<32|rnd())%total;unsigned s=0;while(x>=f[s]){x-=f[s];s++;}return s;
}
struct tile {uint8_t px[128];size_t off,poff;unsigned long long bits;unsigned rows,dicts,pairs,escapes,nsyms;uint8_t syms[256];};
static struct tile tiles[NTILES];
static uint8_t st_even[STREAM_MAX],st_packed[STREAM_MAX];
/* Encode one tile into both streams, choosing symbols by the tables'
 * frequencies in decode order with the decoder's contexts. */
static unsigned long long emit(uint8_t *s,unsigned long long at,unsigned code,unsigned l){return vt_put(s,at,code,l);}
static void make_tiles(void){
 size_t even=0,pk=0;
 for(unsigned i=0;i<NTILES;i++){
  struct tile *t=&tiles[i];memset(t,0,sizeof *t);
  t->off=even;t->poff=pk;
  unsigned long long ae=8ull*even,ap=8ull*pk;
  uint8_t px[136];memset(px,0,sizeof px);uint8_t *o=px+8;
  unsigned prev=0;
#define PUT(code,l) do{ae=emit(st_even,ae,code,l);ap=emit(st_packed,ap,code,l);t->bits+=l;}while(0)
#define SYM(v) (t->syms[t->nsyms++]=(uint8_t)(v))
  for(unsigned row=0;row<16;row++,o+=8){
   const unsigned rs=sample(cd.rowf[prev],16);PUT(cd.rowc[prev][rs],cd.rowl[prev][rs]);SYM(rs);t->rows++;if(cd.rowl[prev][rs]>6)t->escapes++;
   prev=rs;unsigned left=0;
   for(unsigned h=0;h<2;h++){
    uint8_t *q=o+4*h;const unsigned kind=(rs>>(2*h))&3u;
    if(kind==0){memset(q,0,4);left=0;}
    else if(kind==1){memcpy(q,q-8,4);left=q[3]&15u;}
    else if(kind==2){const unsigned d=sample(cd.dictf,256);PUT(cd.dictc[d],cd.dictl[d]);SYM(d);t->dicts++;if(cd.dictl[d]>7)t->escapes++;memcpy(q,cd.dict[d],4);left=q[3]&15u;}
    else for(unsigned j=0;j<4;j++){
     const unsigned k=cd.ctx_table[cd.klass16[(q-8)[j]]|left];
     const unsigned v=sample(cd.pairf[k],256);PUT(cd.pairc[k][v],cd.pairl[k][v]);SYM(v);t->pairs++;if(cd.pairl[k][v]>6)t->escapes++;
     q[j]=(uint8_t)v;left=v&15u;}
   }
  }
#undef PUT
#undef SYM
  memcpy(t->px,px+8,128);
  even=(size_t)((ae+15)/16)*2u;pk=(size_t)((ap+7)/8);
  if(even+64>STREAM_MAX)bad("tile stream too long",i);
 }
}
static void put_v1(u32 a,const sh2z_vlc8_t *t);
static void put_codec(void){
 put_bytes("arena",0,cd.arena,cd.arena_used);
 for(unsigned p=0;p<16;p++)ww(&cpu,symc("rowmap")+2*p,cd.rowmap[p]);
 /* rotated by 128: entry 128 + (int8_t)x holds x's value */
 for(unsigned c=0;c<256;c++){ww(&cpu,symc("map_rot")+2*((c+128u)&255u),cd.ctxmap[c]);wb(&cpu,symc("v1_ctx")+c,cd.ctx_table[c]);}
 ww(&cpu,symc("dict_off"),cd.dict_off);
 for(unsigned b=0;b<256;b++)wb(&cpu,symc("klass_rot")+((b+128u)&255u),cd.klass16[b]);
 for(unsigned d=0;d<256;d++)put_bytes("dict",4*d,cd.dict[d],4);
 put_bytes("stream",0,st_even,STREAM_MAX);put_bytes("packed",0,st_packed,STREAM_MAX);
 for(unsigned i=0;i<NTILES;i++){wl(&cpu,symc("tile_off")+4*i,(u32)tiles[i].off);wl(&cpu,symc("packed_off")+4*i,(u32)tiles[i].poff);}
 for(unsigned i=0;i<8;i++)wb(&cpu,symc("tile_words")+i,0);
 /* vlc-1 tables of the same codes */
 static sh2z_vlc8_t t1;unsigned l1[256],c1[256];
 const u32 row=symc("v1_row"),pair=symc("v1_pair"),dict=symc("v1_dict");
#define PUT1(addr,lens,codes,w) do{for(unsigned s=0;s<256;s++)l1[s]=(lens)[s];vo_canonical(l1,c1,0);\
  for(unsigned s=0;s<256;s++)if(l1[s]&&c1[s]!=(codes)[s])bad("vlc-1 and vlc-2 codes differ",s);\
  if(!vo_table(&t1,l1,c1,w,12,0))bad("vlc-1 table",0);put_v1(addr,&t1);}while(0)
 for(unsigned p=0;p<16;p++)PUT1(row+(u32)sizeof(sh2z_vlc8_t)*p,cd.rowl[p],cd.rowc[p],6);
 for(unsigned k=0;k<32;k++)PUT1(pair+(u32)sizeof(sh2z_vlc8_t)*k,cd.pairl[k],cd.pairc[k],6);
 PUT1(dict,cd.dictl,cd.dictc,7);
#undef PUT1
}
static void put_v1(u32 a,const sh2z_vlc8_t *t){
 wb(&cpu,a,t->w);wb(&cpu,a+1,t->lmax);ww(&cpu,a+2,t->pool_len);
 for(unsigned i=0;i<256;i++){wb(&cpu,a+4+i,t->s[i]);wb(&cpu,a+260+i,t->n[i]);}
 for(unsigned i=0;i<SH2Z_VLC8_POOL_MAX;i++)ww(&cpu,a+516+2*i,t->pool[i]);
}

/* ---- cost ---- */
static const struct place_decl place_v2[]={{"_arena",PC_TABLES},{"_rowmap",PC_TABLES},{"_map_rot",PC_TABLES},{"_dict_off",PC_TABLES},{"_klass_rot",PC_TABLES},{"_dict",PC_TABLES},{"_tile_words",PC_OUTPUT},{NULL,0}};
static const struct place_decl place_v1[]={{"_v1_row",PC_TABLES},{"_v1_dict",PC_TABLES},{"_v1_pair",PC_TABLES},{"_v1_ctx",PC_TABLES},{"_klass_rot",PC_TABLES},{"_dict",PC_TABLES},{"_tile_words",PC_OUTPUT},{NULL,0}};
struct variant {const char *name,*fn,*description;const struct place_decl *decl;int guard,packed;};
static const struct variant variants[]={
 {"v2","tile_v2","sh2z_vlc2_decode (inline SH-2 assembly), tiles at even offsets",place_v2,1,0},
 {"v2Packed","tile_v2_packed","sh2z_vlc2_decode, byte-packed tiles (odd and even offsets)",place_v2,1,1},
 {"v2Portable","tile_v2c","sh2z_vlc2_decode_portable compiled for SH-2",place_v2,1,0},
 {"marker","tile_mk","the same caller with a marker-bit reader variant (bridge only)",place_v2,1,0},
 {"vlc1","tile_v1","one vlc-1 sh2z_vlc8_decode call per symbol, vlc-1 tables",place_v1,0,0},
 {"glue","tile_glue","the same caller with the symbols read from a list: the caller's own cost, no decoding",place_v2,0,0},
};
#define NV (sizeof variants/sizeof variants[0])
static int cmp_u(const void*a,const void*b){unsigned x=*(const unsigned*)a,y=*(const unsigned*)b;return x<y?-1:x>y;}
struct stats {double mean,median;unsigned min,max;double instr;};
static struct stats summarize(unsigned *c,size_t n,uint64_t instr){
 qsort(c,n,sizeof *c,cmp_u);double sum=0;for(size_t i=0;i<n;i++)sum+=c[i];
 return (struct stats){sum/(double)n,n&1?c[n/2]:(c[n/2-1]+c[n/2])/2.0,c[0],c[n-1],(double)instr/(double)n};
}
static void print_stats(struct stats s){printf("{\"meanCycles\":%.2f,\"medianCycles\":%.1f,\"minCycles\":%u,\"maxCycles\":%u,\"meanInstructions\":%.2f}",s.mean,s.median,s.min,s.max,s.instr);}
static uint64_t v_tiles;
static void verify_tiles(void){
 const u32 sa=symc("stream"),pa=symc("packed"),tw=symc("tile_words");
 for(unsigned i=0;i<NTILES;i++)for(size_t v=0;v<NV;v++){
  const struct tile *t=&tiles[i];
  for(unsigned b=0;b<128;b++)wb(&cpu,tw+8+b,0x5A);
  if(variants[v].guard){const u32 base=variants[v].packed?pa:sa;const size_t off=variants[v].packed?t->poff:t->off;
   guard_set(base+(u32)sh2z_vlc2_reach(off,(size_t)t->bits),base+STREAM_MAX,0,0);}
  put_bytes("symlist",0,t->syms,t->nsyms);
  struct cost c=call(variants[v].fn,i,0,0);guard_on=0;
  for(unsigned b=0;b<128;b++)if(rb(&cpu,tw+8+b)!=t->px[b])bad(variants[v].fn,i);
  const unsigned long long want_pos=8ull*(variants[v].packed?t->poff:t->off)+t->bits;
  if(!strcmp(variants[v].name,"glue")&&c.result!=t->nsyms)bad("glue symbol count",i);
  if((!strcmp(variants[v].name,"v2")||!strcmp(variants[v].name,"v2Packed")||!strcmp(variants[v].name,"v2Portable"))&&c.result!=(u32)want_pos)bad("tile bit position",i);
  v_tiles++;
 }
}
static void cost_tiles(const struct mix *mx,bool *first){
 static unsigned cyc[NV][NTILES];uint64_t ins[NV];memset(ins,0,sizeof ins);
 for(size_t v=0;v<NV;v++)for(unsigned i=0;i<NTILES;i++){
  put_bytes("symlist",0,tiles[i].syms,tiles[i].nsyms);
  char label[80];snprintf(label,sizeof label,"vlc2.%s.%s",mx->name,variants[v].name);place_tag(label,variants[v].decl);
  struct cost c=call(variants[v].fn,i,0,0);cyc[v][i]=c.cycles;ins[v]+=c.instructions;
 }
 double rows=0,dicts=0,pairs=0,esc=0,bits=0,syms;
 for(unsigned i=0;i<NTILES;i++){rows+=tiles[i].rows;dicts+=tiles[i].dicts;pairs+=tiles[i].pairs;esc+=tiles[i].escapes;bits+=(double)tiles[i].bits;}
 rows/=NTILES;dicts/=NTILES;pairs/=NTILES;esc/=NTILES;bits/=NTILES;syms=rows+dicts+pairs;
 if(!*first)putchar(',');*first=false;
 printf("\"%s\":{\"description\":\"%s\",\"tiles\":%u,\"symbolsPerTile\":%.2f,\"rowSymbolsPerTile\":%.2f,\"dictionarySymbolsPerTile\":%.2f,\"pairSymbolsPerTile\":%.2f,\"escapesPerTile\":%.2f,\"bitsPerTile\":%.1f,\"arenaBytes\":%zu,\"hotBytes\":%zu,\"variants\":{",
  mx->name,mx->description,NTILES,syms,rows,dicts,pairs,esc,bits,cd.arena_used,cd.hot_bytes);
 for(size_t v=0;v<NV;v++){
  struct stats s=summarize(cyc[v],NTILES,ins[v]);
  printf("%s\"%s\":{\"description\":\"%s\",\"perTile\":",v?",":"",variants[v].name,variants[v].description);print_stats(s);
  printf(",\"cyclesPerSymbol\":%.2f}",s.mean/syms);
 }
 printf("}}");
}
/* ---- symbol loops over one table ---- */
static void cost_loops(bool *first,const char *name,const char *description,const uint32_t f[256],unsigned w){
 uint8_t len[256];unsigned code[256];
 lengths_of(f,12,len);const size_t size=vt_table(tb,TMAX,len,256,w,code,0);if(!size)bad("loop table",0);
 put_bytes("tbuf",0,tb,size);
 {unsigned l1[256],c1[256];static sh2z_vlc8_t t1;for(unsigned s=0;s<256;s++)l1[s]=len[s];vo_canonical(l1,c1,0);if(!vo_table(&t1,l1,c1,w,12,0))bad("loop vlc-1 table",0);put_v1(symc("v1_dict"),&t1);}
 enum {N=4096,CORPORA=8};
 static const char *const fns[5]={"loop_v2","loop_v2c","loop_mk","loop_v1","loop_v1_each"};
 static const char *const names[5]={"v2","v2Portable","marker","vlc1Batch","vlc1PerSymbol"};
 static unsigned cyc[5][2][CORPORA];uint64_t ins[5][2];memset(ins,0,sizeof ins);
 double bits=0,esc=0;
 for(unsigned c=0;c<CORPORA;c++){
  memset(sbuf,0,sizeof sbuf);unsigned long long at=0;static uint8_t ref[N];
  for(unsigned i=0;i<N;i++){const unsigned s=sample(f,256);ref[i]=(uint8_t)s;at=vt_put(sbuf,at,code[s],len[s]);if(len[s]>w)esc++;}
  bits+=(double)at;put_bytes("vstream",0,sbuf,VSTREAM);
  for(int k=0;k<5;k++)for(int z=0;z<2;z++){
   const unsigned n=z?N:1u;
   struct cost r=call(fns[k],n,0,0);
   for(unsigned i=0;i<n;i++)if(rb(&cpu,symc("out")+i)!=ref[i])bad(fns[k],c);
   cyc[k][z][c]=r.cycles;ins[k][z]+=r.instructions;
  }
 }
 if(!*first)putchar(',');*first=false;
 const double bps=bits/(CORPORA*(double)N);
 printf("\"%s\":{\"description\":\"%s\",\"w\":%u,\"lmax\":12,\"symbols\":%u,\"corpora\":%u,\"meanBitsPerSymbol\":%.3f,\"escapeFraction\":%.4f,\"decoders\":{",name,description,w,N,CORPORA,bps,esc/(CORPORA*(double)N));
 for(int k=0;k<5;k++){
  struct stats one=summarize(cyc[k][0],CORPORA,ins[k][0]),many=summarize(cyc[k][1],CORPORA,ins[k][1]);
  printf("%s\"%s\":{\"cyclesPerSymbol\":%.2f,\"instructionsPerSymbol\":%.2f,\"call1\":",k?",":"",names[k],(many.mean-one.mean)/(N-1),(many.instr-one.instr)/(N-1));
  print_stats(one);printf(",\"call%u\":",N);print_stats(many);printf("}");
 }
 printf("}}");
}

int main(int argc,char**argv){
 if(argc<3){fprintf(stderr,"usage: vlc2_harness model.elf verify-scale\n");return 2;}
 const uint64_t scale=strtoull(argv[2],0,10);
 load_elf_symbols(argv[1]);load_elf(argv[1]);place_init(argv[1]);
 cpu.model=SHMT_SH2;cpu.MappedMemoryReadByte=g_rb;cpu.MappedMemoryReadWord=g_rw;cpu.MappedMemoryReadLong=g_rl;cpu.MappedMemoryWriteByte=wb;cpu.MappedMemoryWriteWord=ww;cpu.MappedMemoryWriteLong=wl;
 for(unsigned i=0;i<65536;i++)cpu.opcodes[i]=decode(SHMT_SH2,(u16)i);for(unsigned i=0;i<256;i++)cpu.fetchlist[i]=fetch;
 verify(scale);
 /* tiles: verify every variant on every tile of both mixes, then time */
 for(size_t m=0;m<sizeof mixes/sizeof mixes[0];m++){make_codec(&mixes[m]);memset(st_even,0,sizeof st_even);memset(st_packed,0,sizeof st_packed);make_tiles();put_codec();verify_tiles();}
 printf("{\"schema\":1,\"kind\":\"Yabause isolated SH2 interpreter flat-RAM model, not Saturn timing\",");
 printf("\"verified\":{\"builds\":%"PRIu64",\"buildRefusals\":%"PRIu64",\"streams\":%"PRIu64",\"oddStartStreams\":%"PRIu64",\"symbolsPerDecoder\":%"PRIu64",\"randomByteStreams\":%"PRIu64",\"formatTables\":%"PRIu64",\"formatTablesAccepted\":%"PRIu64",\"mixedTableStreams\":%"PRIu64",\"tileDecodes\":%"PRIu64",\"entries\":\"v2_build tables and codes, v2_check, the inline-assembly and portable decoders at every w (symbols, reader state, bit position) against tests/vlc2_oracle.h; every tile variant against the generator; no stream read at or past sh2z_vlc2_reach and no read past a checked table\"},",
  v_builds,v_build_refusals,v_streams,v_odd,v_symbols,v_hostile,v_garbage,v_garbage_ok,v_mixed,v_tiles);
 bool first=true;printf("\"tiles\":{");
 for(size_t m=0;m<sizeof mixes/sizeof mixes[0];m++){make_codec(&mixes[m]);memset(st_even,0,sizeof st_even);memset(st_packed,0,sizeof st_packed);make_tiles();put_codec();cost_tiles(&mixes[m],&first);}
 printf("},\"loops\":{");first=true;
 {uint32_t f[256];rng=UINT32_C(0x10095eed);
  zipf(f,256,1.3);cost_loops(&first,"pairLike","256 symbols, Zipf s = 1.3, w = 6",f,6);
  memset(f,0,sizeof f);for(unsigned s=0;s<16;s++)f[s]=1u<<(15u-s);cost_loops(&first,"geometric16","16 symbols, each half as frequent as the last, w = 6",f,6);}
 printf("}}\n");
 return 0;
}
