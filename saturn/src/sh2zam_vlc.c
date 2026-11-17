/* Variable-length symbol decoding for byte alphabets. MIT; Copyright (c) 2026 SH2ZAM Contributors. */
#include "sh2zam/vlc.h"
#include <string.h>

void sh2z_bitr_init(sh2z_bitr_t *r, const uint8_t *p) {
    r->bb = (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)1u << 16;
    r->p = p + 2;
    /* 16 valid bits: marker at bit 16; the first refill happens when fewer than 16 remain */
}

static void refill(sh2z_bitr_t *r) {            /* bb < 0x10000: marker at bit nb <= 15, append 16 bits */
    uint32_t bb = r->bb, nb = 0;
    while (bb >> (nb + 1)) nb++;
    bb = (bb & ~((uint32_t)1u << nb)) | ((uint32_t)r->p[0] | (uint32_t)r->p[1] << 8) << nb | (uint32_t)1u << (nb + 16);
    r->bb = bb; r->p += 2;
}

void sh2z_vlc8_decode(const sh2z_vlc8_t *t, sh2z_bitr_t *r, uint8_t *dst, size_t n) {
    const uint32_t mask = ((uint32_t)1u << t->w) - 1u;
    uint32_t bb = r->bb;
    for (size_t i = 0; i < n; i++) {
        const uint32_t e = bb & mask; const unsigned nv = t->n[e]; unsigned len; uint8_t sym;
        if (nv < 0x80u) { sym = t->s[e]; len = nv / 4u + 1u; }
        else {
            const unsigned xb = (nv >> 4) & 7u; const unsigned v = t->pool[((nv & 15u) << 8 | t->s[e]) + ((bb >> t->w) & (((uint32_t)1u << xb) - 1u))];
            sym = (uint8_t)(v >> 8); len = (v & 255u) / 4u + 1u;
        }
        dst[i] = sym; bb >>= len;
        if (bb < 0x10000u) { r->bb = bb; refill(r); bb = r->bb; }
    }
    r->bb = bb;
}

/* ---- builder ---- */
static void huff_lengths(const uint32_t *freq, unsigned *len) {    /* plain Huffman, O(n^2) on at most 256 leaves */
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

bool sh2z_vlc8_build(sh2z_vlc8_t *t, const uint32_t freq[256], unsigned w, unsigned lmax, uint8_t len_out[256], uint16_t code_out[256]) {
    unsigned len[256], nsym = 0;
    if (!t || !freq || w < 1 || w > 8 || lmax < w || lmax > 16) return false;
    for (unsigned s = 0; s < 256; s++) if (freq[s]) nsym++;
    if (!nsym || (nsym > ((unsigned)1u << lmax))) return false;
    huff_lengths(freq, len);
    {   /* limit to lmax: clamp, then restore the Kraft sum */
        const uint32_t full = (uint32_t)1u << lmax; uint32_t kraft = 0;
        for (unsigned s = 0; s < 256; s++) if (len[s]) { if (len[s] > lmax) len[s] = lmax; kraft += full >> len[s]; }
        while (kraft > full) {      /* lengthen the rarest symbol that is shorter than lmax, longest first (cheapest in Kraft terms per bit lost) */
            int best = -1;
            for (unsigned s = 0; s < 256; s++) if (len[s] && len[s] < lmax && (best < 0 || len[s] > len[(unsigned)best] || (len[s] == len[(unsigned)best] && freq[s] < freq[(unsigned)best]))) best = (int)s;
            if (best < 0) return false;
            kraft -= full >> (len[(unsigned)best] + 1u); len[(unsigned)best]++;
        }
        for (int again = 1; again;) {   /* spend leftover Kraft room: shorten the most frequent symbol that still fits */
            int best = -1; again = 0;
            for (unsigned s = 0; s < 256; s++) if (len[s] > 1 && kraft + (full >> len[s]) <= full && (best < 0 || freq[s] > freq[(unsigned)best])) best = (int)s;
            if (best >= 0) { kraft += full >> len[(unsigned)best]; len[(unsigned)best]--; again = 1; }
        }
    }
    /* canonical codes (MSB-first definition), then bit reversed for the LSB-first stream */
    unsigned count[17] = {0}, first[17], codes[256];
    for (unsigned s = 0; s < 256; s++) count[len[s]]++;
    count[0] = 0; { unsigned c = 0; for (unsigned l = 1; l <= 16; l++) { c = (c + count[l - 1]) << 1; first[l] = c; } }
    unsigned next[17]; memcpy(next, first, sizeof next);
    for (unsigned s = 0; s < 256; s++) if (len[s]) {
        unsigned c = next[len[s]]++, rev = 0;
        for (unsigned b = 0; b < len[s]; b++) rev |= ((c >> b) & 1u) << (len[s] - 1u - b);
        codes[s] = rev;
    }
    t->w = (uint8_t)w; t->lmax = (uint8_t)lmax; t->pool_len = 0; memset(t->s, 0, sizeof t->s); memset(t->n, 0, sizeof t->n);
    for (unsigned s = 0; s < 256; s++) {
        if (len_out) len_out[s] = (uint8_t)len[s];
        if (code_out) code_out[s] = len[s] ? (uint16_t)codes[s] : 0;
    }
    /* first level: codes of at most w bits fill every index that has the code as its low bits */
    for (unsigned s = 0; s < 256; s++) if (len[s] && len[s] <= w) {
        for (unsigned e = codes[s]; e < ((unsigned)1u << w); e += (1u << len[s])) { t->s[e] = (uint8_t)s; t->n[e] = (uint8_t)(4u * (len[s] - 1u)); }
    }
    /* longer codes: one group per w-bit prefix, 2^xb entries where xb = longest length with that prefix - w */
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
