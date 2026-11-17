/* SH-2 bridge for the executed-instruction vlc model. MIT; Copyright (c) 2026
 * SH2ZAM Contributors. Every entry point is called directly by the model
 * harness with the SH-2 calling convention; buffers are written by the
 * harness before each call; nothing here uses libc.
 *
 * Decoders, all over the same table and stream:
 *   fast_decode      sh2z_vlc8_decode (the hand-written SH-2 kernel;
 *                    fast_decode_odd: an odd stream address, portable path)
 *   portable_decode  sh2z_vlc8_decode_portable (the library's C path)
 *   plain_decode     straightforward C: a 32-bit buffer with a bit count,
 *                    byte-pair refill, variable shifts
 * Builders, same frequencies and parameters:
 *   fast_build       sh2z_vlc8_build
 *   plain_build      the builder as contributed (commit f2f7c4c): O(n^2)
 *                    Huffman, full scans in the length limit and per prefix
 *   fast_lengths     sh2z_vlc8_build_lengths from code lengths */
#include "sh2zam/vlc_experimental.h"

#define NOINLINE __attribute__((noinline))
#define STREAM_MAX 49152u
#define OUT_MAX 20480u

sh2z_vlc8_t table,plain_table;
sh2z_vlc_reader_t reader;
uint32_t freq[256];
uint8_t lengths[256];
uint8_t stream[STREAM_MAX] __attribute__((aligned(4)));
uint8_t out[OUT_MAX+32u];

NOINLINE uint32_t fast_decode(uint32_t units,uint32_t n) {
    sh2z_vlc_reader_init(&reader,stream,units);
    return sh2z_vlc8_decode(&table,&reader,out,n);
}
/* An odd stream address: the dispatcher must take the portable path. */
NOINLINE uint32_t fast_decode_odd(uint32_t units,uint32_t n) {
    sh2z_vlc_reader_init(&reader,stream+1,units);
    return sh2z_vlc8_decode(&table,&reader,out,n);
}
NOINLINE uint32_t portable_decode(uint32_t units,uint32_t n) {
    sh2z_vlc_reader_init(&reader,stream,units);
    return sh2z_vlc8_decode_portable(&table,&reader,out,n);
}
NOINLINE uint32_t plain_decode(uint32_t units,uint32_t n) {
    const uint8_t *p=stream,*const end=stream+2u*units;
    const unsigned w=table.w;
    const uint32_t mask=(1u<<w)-1u;
    uint32_t acc=0,missing=0;
    unsigned count=0;
    for(uint32_t i=0;i<n;i++) {
        while(count<=16u) {
            uint32_t unit=0;
            if(p!=end){unit=(uint32_t)p[0]|(uint32_t)p[1]<<8;p+=2;}
            else missing+=16u;
            acc|=unit<<count;
            count+=16u;
        }
        unsigned field=table.n[acc&mask],sym=table.s[acc&mask];
        if(field>=0x80u) {
            const unsigned v=table.pool[((field&15u)<<8|sym)+((acc>>w)&((1u<<((field>>4)&7u))-1u))];
            sym=v>>8;field=v&255u;
        }
        out[i]=(uint8_t)sym;
        const unsigned len=(field>>2)+1u;
        acc>>=len;count-=len;
    }
    return count>=missing;
}

NOINLINE uint32_t fast_build(uint32_t w,uint32_t lmax) {
    return sh2z_vlc8_build(&table,freq,w,lmax,0,0);
}
NOINLINE uint32_t fast_lengths(uint32_t w,uint32_t lmax) {
    return sh2z_vlc8_build_lengths(&table,lengths,w,lmax,0);
}

/* ---- the contributed builder, loops in place of memset/memcpy ---- */
static void plain_huff_lengths(const uint32_t *f,unsigned *len) {
    uint64_t wt[512]; int par[512], alive[512]; unsigned m = 0; int ids[256];
    for (unsigned s = 0; s < 256; s++) { len[s] = 0; if (f[s]) { wt[m] = f[s]; par[m] = -1; alive[m] = 1; ids[s] = (int)m; m++; } else ids[s] = -1; }
    if (m == 1) { for (unsigned s = 0; s < 256; s++) if (ids[s] >= 0) len[s] = 1; return; }
    unsigned total = m;
    for (unsigned k = 0; k + 1 < m; k++) {
        int a = -1, b = -1;
        for (unsigned i = 0; i < total; i++) if (alive[i]) { if (a < 0 || wt[i] < wt[(unsigned)a]) { b = a; a = (int)i; } else if (b < 0 || wt[i] < wt[(unsigned)b]) b = (int)i; }
        wt[total] = wt[(unsigned)a] + wt[(unsigned)b]; par[total] = -1; alive[total] = 1; alive[a] = alive[b] = 0; par[a] = par[b] = (int)total; total++;
    }
    for (unsigned s = 0; s < 256; s++) if (ids[s] >= 0) { unsigned d = 0; int x = ids[s]; while (par[x] >= 0) { x = par[x]; d++; } len[s] = d; }
}
NOINLINE uint32_t plain_build(uint32_t w,uint32_t lmax) {
    sh2z_vlc8_t *t = &plain_table;
    unsigned len[256], nsym = 0;
    if (w < 1 || w > 8 || lmax < w || lmax > 16) return 0;
    for (unsigned s = 0; s < 256; s++) if (freq[s]) nsym++;
    if (!nsym || (nsym > ((unsigned)1u << lmax))) return 0;
    plain_huff_lengths(freq, len);
    {
        const uint32_t full = (uint32_t)1u << lmax; uint32_t kraft = 0;
        for (unsigned s = 0; s < 256; s++) if (len[s]) { if (len[s] > lmax) len[s] = lmax; kraft += full >> len[s]; }
        while (kraft > full) {
            int best = -1;
            for (unsigned s = 0; s < 256; s++) if (len[s] && len[s] < lmax && (best < 0 || len[s] > len[(unsigned)best] || (len[s] == len[(unsigned)best] && freq[s] < freq[(unsigned)best]))) best = (int)s;
            if (best < 0) return 0;
            kraft -= full >> (len[(unsigned)best] + 1u); len[(unsigned)best]++;
        }
        for (int again = 1; again;) {
            int best = -1; again = 0;
            for (unsigned s = 0; s < 256; s++) if (len[s] > 1 && kraft + (full >> len[s]) <= full && (best < 0 || freq[s] > freq[(unsigned)best])) best = (int)s;
            if (best >= 0) { kraft += full >> len[(unsigned)best]; len[(unsigned)best]--; again = 1; }
        }
    }
    unsigned count[17], first[17], codes[256];
    for (unsigned l = 0; l < 17; l++) count[l] = 0;
    for (unsigned s = 0; s < 256; s++) count[len[s]]++;
    count[0] = 0; { unsigned c = 0; for (unsigned l = 1; l <= 16; l++) { c = (c + count[l - 1]) << 1; first[l] = c; } }
    unsigned next[17]; for (unsigned l = 0; l < 17; l++) next[l] = first[l];
    for (unsigned s = 0; s < 256; s++) if (len[s]) {
        unsigned c = next[len[s]]++, rev = 0;
        for (unsigned b = 0; b < len[s]; b++) rev |= ((c >> b) & 1u) << (len[s] - 1u - b);
        codes[s] = rev;
    }
    t->w = (uint8_t)w; t->lmax = (uint8_t)lmax; t->pool_len = 0;
    for (unsigned e = 0; e < 256; e++) { t->s[e] = 0; t->n[e] = 0; }
    for (unsigned s = 0; s < 256; s++) if (len[s] && len[s] <= w) {
        for (unsigned e = codes[s]; e < ((unsigned)1u << w); e += (1u << len[s])) { t->s[e] = (uint8_t)s; t->n[e] = (uint8_t)(4u * (len[s] - 1u)); }
    }
    for (unsigned e = 0; e < ((unsigned)1u << w); e++) {
        unsigned xb = 0, used = 0;
        for (unsigned s = 0; s < 256; s++) if (len[s] > w && (codes[s] & (((unsigned)1u << w) - 1u)) == e) { used = 1; if (len[s] - w > xb) xb = len[s] - w; }
        if (!used) continue;
        if (xb > 7 || t->pool_len + (1u << xb) > SH2Z_VLC8_POOL_MAX || t->pool_len >= 4096u) return 0;
        const unsigned off = t->pool_len; t->pool_len = (uint16_t)(t->pool_len + (1u << xb));
        t->s[e] = (uint8_t)(off & 255u); t->n[e] = (uint8_t)(0x80u | xb << 4 | (off >> 8));
        for (unsigned s = 0; s < 256; s++) if (len[s] > w && (codes[s] & (((unsigned)1u << w) - 1u)) == e) {
            const unsigned hi = codes[s] >> w, step = 1u << (len[s] - w);
            for (unsigned k = hi; k < (1u << xb); k += step) t->pool[off + k] = (uint16_t)(s << 8 | 4u * (len[s] - 1u));
        }
    }
    return 1;
}
