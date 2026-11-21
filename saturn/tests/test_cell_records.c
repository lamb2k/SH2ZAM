/* Delta coded sprite records (contract tag cell-records-1) checks.
 * Every form against an independent per-cell oracle, chained calls, buffer and
 * argument contracts, and the negative controls (every planted oracle mutant
 * must disagree with the library).
 *
 * Copyright (c) 2026 SH2ZAM Contributors. MIT License.
 */
#include "sh2zam/cell_records_experimental.h"
#include "cell_records_oracle.h"
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAXC 72u
static uint64_t checks;
#define CHECK(e) do { ++checks; if(!(e)) { fprintf(stderr,"FAIL %s:%d: %s\n",__FILE__,__LINE__,#e); exit(EXIT_FAILURE); } } while(0)
static uint32_t rng=UINT32_C(0x2f6e2b1d);
static uint32_t rnd(void) { rng^=rng<<13; rng^=rng>>17; rng^=rng<<5; return rng; }

/* guard words around every output, over a 4-byte aligned arena */
enum { GUARD = 16 };
static uint32_t outbuf[(GUARD + MAXC * 8 + GUARD) / 4 + 2];
static uint8_t recbuf[MAXC * 4 + 8] __attribute__((aligned(4)));
static uint8_t offbuf[MAXC * 4 + 8] __attribute__((aligned(4)));

static const unsigned forms[] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 };

static void fill_params(sh2z_cellrec_params_t *p, unsigned form, unsigned kind) {
    p->form = form;
    p->pos_mask = kind == 0 ? UINT32_C(0x03ff03ff) : rnd();
    p->pos_or = kind == 0 ? (rnd() & UINT32_C(0xfc00fc00)) : rnd();
    p->mirror_bias = rnd();
    p->attr_keep = kind == 1 ? UINT32_C(0xffffffe0) : (kind == 0 ? UINT32_C(0xffffffe0) : rnd());
    p->attr_or = kind == 0 ? (rnd() & 0x1fu) : rnd();
}

/* the library on a prepared case against the oracle (mutation 0) or a mutant */
static int run_case(unsigned form, size_t n, unsigned rec_skew, unsigned kind, unsigned mut, unsigned trusted) {
    sh2z_cellrec_params_t p;
    sh2z_cellrec_pos_t pos, pos2;
    uint8_t *out = (uint8_t *)outbuf + GUARD, *exp = (uint8_t *)malloc(MAXC * 8 + 8);
    uint8_t *rec = recbuf + rec_skew;           /* 0 or 2: the record stream may be halfword aligned */
    uint32_t ex, ey;
    int ok = 1;
    fill_params(&p, form, kind);
    for(size_t i = 0; i < MAXC * 4; i++) { recbuf[i + 0] = (uint8_t)rnd(); offbuf[i] = (uint8_t)rnd(); }
    if(kind != 2) for(size_t i = 0; i < n * 2 && (form & 1u) == 0; i++) offbuf[i] = (uint8_t)((int)(rnd() % 40u) - 20);
    pos.x = rnd(); pos.y = rnd();
    pos2 = pos;
    ex = pos.x; ey = pos.y;
    memset(outbuf, 0xA5, sizeof outbuf);
    memset(exp, 0xA5, MAXC * 8 + 8);
    cro_expand(&ex, &ey, form, p.pos_mask, p.pos_or, p.mirror_bias, p.attr_keep, p.attr_or, rec, offbuf, exp, n, mut);
    if(trusted) sh2z_cellrec_expand_trusted(&pos, &p, rec, offbuf, out, n);
    else if(!sh2z_cellrec_expand(&pos, &p, rec, offbuf, out, n)) ok = 0;
    if(ok && memcmp(out, exp, n * 8u)) ok = 0;
    if(ok && (pos.x != (ex & 0xffffu) || pos.y != (ey & 0xffffu))) ok = 0;
    /* nothing outside the n entries is touched */
    for(size_t i = 0; ok && i < GUARD; i++) if(((uint8_t *)outbuf)[i] != 0xA5) ok = 0;
    for(size_t i = n * 8u + GUARD; ok && i < sizeof outbuf; i++) if(((uint8_t *)outbuf)[i] != 0xA5) ok = 0;
    (void)pos2;
    free(exp);
    return ok;
}

static void exact(void) {
    static const size_t counts[] = { 0, 1, 2, 3, 4, 5, 7, 8, 16, 17, 40, 41, MAXC };
    for(unsigned it = 0; it < 24000u; it++) {
        const unsigned form = forms[it % 12u];
        const size_t n = counts[(it / 12u) % 13u];
        CHECK(run_case(form, n, (it >> 3) & 2u, it % 3u, 0, it & 1u));
    }
}

/* two calls in a row equal one call over the whole list */
static void chained(void) {
    for(unsigned it = 0; it < 3000u; it++) {
        sh2z_cellrec_params_t p; sh2z_cellrec_pos_t a, b;
        const unsigned form = forms[it % 12u]; const size_t n = 2u + rnd() % 60u, k = 1u + rnd() % (n - 1u);
        uint8_t o1[MAXC * 8] __attribute__((aligned(4))), o2[MAXC * 8] __attribute__((aligned(4)));
        const size_t ob = (form & 1u) ? 4u : 2u;
        fill_params(&p, form, it % 3u);
        for(size_t i = 0; i < MAXC * 4; i++) { recbuf[i] = (uint8_t)rnd(); offbuf[i] = (uint8_t)rnd(); }
        a.x = b.x = rnd(); a.y = b.y = rnd();
        CHECK(sh2z_cellrec_expand(&a, &p, recbuf, offbuf, o1, n));
        CHECK(sh2z_cellrec_expand(&b, &p, recbuf, offbuf, o2, k));
        CHECK(sh2z_cellrec_expand(&b, &p, recbuf + 4u * k, offbuf + ob * k, o2 + 8u * k, n - k));
        CHECK(!memcmp(o1, o2, n * 8u) && a.x == b.x && a.y == b.y);
    }
}

/* every mutant must be caught: it disagrees with the library on the corpus */
static void mutants(void) {
    static const unsigned muts[] = { CRO_MUT_FLIP_BIT, CRO_MUT_WIDTH_SHIFT, CRO_MUT_MIRROR_SIGN, CRO_MUT_BIAS_IGNORED, CRO_MUT_ADD_CARRY,
        CRO_MUT_REPLACE_ORDER, CRO_MUT_S8_UNSIGNED, CRO_MUT_MASK_BEFORE_OR, CRO_MUT_Y_SIGN, CRO_MUT_STATE_X };
    for(unsigned m = 0; m < sizeof muts / sizeof *muts; m++) {
        unsigned caught = 0;
        for(unsigned it = 0; it < 6000u && !caught; it++) caught = !run_case(forms[it % 12u], 4u + it % 30u, 0, it % 3u, muts[m], 0);
        printf("mutation control %u (oracle bit %u): %s\n", m, muts[m], caught ? "caught" : "MISSED");
        CHECK(caught);
    }
}

static void arguments(void) {
    sh2z_cellrec_params_t p; sh2z_cellrec_pos_t pos = { 5, 6 };
    uint8_t out[64] __attribute__((aligned(4)));
    uint8_t rec[16] __attribute__((aligned(4))) = { 0 }, off[16] __attribute__((aligned(4))) = { 0 };
    fill_params(&p, 0, 0);
    memset(out, 0x77, sizeof out);
    CHECK(sh2z_cellrec_expand(&pos, &p, rec, off, out, 0) && out[0] == 0x77 && pos.x == 5 && pos.y == 6);   /* count 0: nothing, even with null streams */
    CHECK(sh2z_cellrec_expand(&pos, &p, NULL, NULL, NULL, 0));
    CHECK(sh2z_cellrec_expand(&pos, &p, rec, off, out, 4));
    memset(out, 0x77, sizeof out);
    {
        const uint32_t px = pos.x, py = pos.y;
        CHECK(!sh2z_cellrec_expand(NULL, &p, rec, off, out, 1));
        CHECK(!sh2z_cellrec_expand(&pos, NULL, rec, off, out, 1));
        CHECK(!sh2z_cellrec_expand(&pos, &p, NULL, off, out, 1));
        CHECK(!sh2z_cellrec_expand(&pos, &p, rec, NULL, out, 1));
        CHECK(!sh2z_cellrec_expand(&pos, &p, rec, off, NULL, 1));
        CHECK(!sh2z_cellrec_expand(&pos, &p, rec + 1, off, out, 1));         /* odd records */
        CHECK(!sh2z_cellrec_expand(&pos, &p, rec, off + 1, out, 1));         /* odd offsets */
        CHECK(!sh2z_cellrec_expand(&pos, &p, rec, off, out + 2, 1));         /* entries not 4-byte aligned */
        CHECK(!sh2z_cellrec_expand(&pos, &p, rec, off, out, SH2Z_CELLREC_MAX_COUNT + 1u));
        p.form = 16u; CHECK(!sh2z_cellrec_expand(&pos, &p, rec, off, out, 1));
        p.form = SH2Z_CELLREC_ATTR_REPLACE | SH2Z_CELLREC_ATTR_ADD_BYTE; CHECK(!sh2z_cellrec_expand(&pos, &p, rec, off, out, 1));
        p.form = 0;
        CHECK(!sh2z_cellrec_expand(&pos, &p, rec, off, rec, 1));               /* entries over the records */
        CHECK(!sh2z_cellrec_expand(&pos, &p, rec, off, off, 1));
        CHECK(!sh2z_cellrec_expand((sh2z_cellrec_pos_t *)out, &p, rec, off, out, 1));
        for(size_t i = 0; i < sizeof out; i++) CHECK(out[i] == 0x77);        /* every refusal left the output alone */
        CHECK(pos.x == px && pos.y == py);
    }
    /* an in-range case after refusals still works */
    p.form = 0; CHECK(sh2z_cellrec_expand(&pos, &p, rec, off, out, 1));
}

/* the count limit alone: the streams and the entries are separate buffers big enough for MAX_COUNT + 1 cells,
 * so no overlap or alignment check can refuse the call; only the count can */
static void count_limit(void) {
    static uint8_t rec[4u * (SH2Z_CELLREC_MAX_COUNT + 1u) + 8u] __attribute__((aligned(4)));
    static uint8_t off[2u * (SH2Z_CELLREC_MAX_COUNT + 1u) + 8u] __attribute__((aligned(4)));
    static uint8_t out[8u * (SH2Z_CELLREC_MAX_COUNT + 1u) + 8u] __attribute__((aligned(4)));
    sh2z_cellrec_params_t p; sh2z_cellrec_pos_t pos = { 5, 6 };
    fill_params(&p, 0, 0);
    memset(out, 0x77, sizeof out);
    CHECK(!sh2z_cellrec_expand(&pos, &p, rec, off, out, SH2Z_CELLREC_MAX_COUNT + 1u));
    for(size_t i = 0; i < sizeof out; i++) CHECK(out[i] == 0x77);          /* the refusal left the output and the position alone */
    CHECK(pos.x == 5u && pos.y == 6u);
    CHECK(sh2z_cellrec_expand(&pos, &p, rec, off, out, SH2Z_CELLREC_MAX_COUNT));   /* the same buffers at the limit are accepted */
    CHECK(out[8u * SH2Z_CELLREC_MAX_COUNT] == 0x77);                       /* and nothing is written past the last entry */
}

/* an exact worked case: three cells, byte offsets, replaced palette bits, mirrored */
static void worked(void) {
    static const uint8_t rec[12] __attribute__((aligned(4))) = { 0x12, 0x34, 0x03, 0x41, 0x12, 0x35, 0x13, 0xc2, 0x12, 0x36, 0x00, 0x03 };
    static const uint8_t off[6] __attribute__((aligned(2))) = { 0x10, 0x00, 0xf0, 0x08, 0x00, 0xfc };
    uint8_t out[24] __attribute__((aligned(4)));
    sh2z_cellrec_params_t p = { SH2Z_CELLREC_MIRROR_X | SH2Z_CELLREC_ATTR_REPLACE, UINT32_C(0x03ff03ff), UINT32_C(0x80008000), 3, UINT32_C(0xffffffe0), 7 };
    sh2z_cellrec_pos_t pos = { 100, 50 };
    CHECK(sh2z_cellrec_expand(&pos, &p, rec, off, out, 3));
    /* cell 0: attr 0x0341 -> keep high, (0x0341 & 0xffe0) | 7 = 0x0347, ^0x20 = 0x0367; width (0x0367 >> 4) & 0xf0 = 0x30;
       x = 100 - 16 = 84; entry x = 84 - 48 - 3 = 33; y = 50 + 0 = 50 */
    CHECK(out[4] == 0x12 && out[5] == 0x34 && out[6] == 0x03 && out[7] == 0x67);
    CHECK(out[0] == 0x80 && out[1] == 33 && out[2] == 0x80 && out[3] == 50);
    CHECK(pos.x == 100u && pos.y == 54u);                                  /* x: 100 - 16, + 16, + 0; y: 50 + 0 + 8 - 4 */
}

int main(void) {
    exact();
    chained();
    arguments();
    count_limit();
    worked();
    mutants();
    printf("cell_records_exact: %" PRIu64 " checks passed\n", checks);
    return 0;
}
