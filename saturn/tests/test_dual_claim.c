/* Two-CPU try-claim (dual-claim-1): every interleaving of the two sides'
 * memory operations against the contract (at most one owner; exactly one
 * when both claim), the real functions in each sequential order, and
 * negative controls (each broken protocol must allow two owners).
 *
 * Copyright (c) 2026 SH2ZAM Contributors. MIT License.
 * Inspired by SH4ZAM by Falco Girgis and SH4ZAM Contributors.
 */
#include "sh2zam/dual_claim_experimental.h"
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>

static uint64_t checks;
#define CHECK(e) do { ++checks; if(!(e)) { fprintf(stderr,"FAIL %s:%d: %s\n",__FILE__,__LINE__,#e); exit(EXIT_FAILURE); } } while(0)

/* Step model: each side is a program counter over single memory operations,
 * as sh2zam_dual_claim.c performs them in program order. */
enum { BUG_NONE, BUG_A_NO_WAIT, BUG_B_IGNORES_A, BUG_A_LOAD_FIRST, BUG_B_LOAD_FIRST };
typedef struct { uint32_t want_a, want_b, decide; } mem_t;
typedef struct { int pc, done, owns; uint32_t seen; } side_t;

/* One operation of A; returns 0 when blocked (spinning on decide). */
static int step_a(mem_t *m, side_t *a, uint32_t tag, int bug) {
    switch (a->pc) {
    case 0:
        if (bug == BUG_A_LOAD_FIRST) { a->seen = m->want_b; a->pc = 10; return 1; }
        m->want_a = tag; a->pc = 1; return 1;
    case 10: m->want_a = tag; a->pc = 11; return 1; /* reordered: decide on the early load */
    case 11:
        if (a->seen != tag || bug == BUG_A_NO_WAIT) { a->done = 1; a->owns = 1; return 1; }
        a->pc = 2; return 1;
    case 1:
        if (m->want_b != tag || bug == BUG_A_NO_WAIT) { a->done = 1; a->owns = 1; return 1; }
        a->pc = 2; return 1;
    case 2:
        if ((m->decide >> 1) != tag) return 0;
        a->done = 1; a->owns = (m->decide & 1u) == 0u; return 1;
    }
    return 0;
}
static int step_b(mem_t *m, side_t *b, uint32_t tag, int bug) {
    switch (b->pc) {
    case 0:
        if (bug == BUG_B_LOAD_FIRST) { b->seen = m->want_a; b->pc = 10; return 1; }
        m->want_b = tag; b->pc = 1; return 1;
    case 10: m->want_b = tag; b->pc = 11; return 1;
    case 11: b->owns = b->seen != tag; m->decide = (tag << 1) | (uint32_t)b->owns; b->done = 1; return 1;
    case 1:
        b->owns = bug == BUG_B_IGNORES_A ? 1 : m->want_a != tag; b->pc = 2; return 1;
    case 2: m->decide = (tag << 1) | (uint32_t)b->owns; b->done = 1; return 1;
    }
    return 0;
}

/* Depth-first over every interleaving; counts outcomes. */
static uint64_t paths, two_owners, no_owner, stuck;
static void explore(mem_t m, side_t a, side_t b, uint32_t tag, int bug) {
    if (a.done && b.done) {
        ++paths;
        if (a.owns && b.owns) ++two_owners;
        if (!a.owns && !b.owns) ++no_owner;
        return;
    }
    int moved = 0;
    if (!a.done) { mem_t m2 = m; side_t a2 = a; if (step_a(&m2, &a2, tag, bug)) { moved = 1; explore(m2, a2, b, tag, bug); } }
    if (!b.done) { mem_t m2 = m; side_t b2 = b; if (step_b(&m2, &b2, tag, bug)) { moved = 1; explore(m2, a, b2, tag, bug); } }
    if (!moved) ++stuck;
}

static void run_model(int bug, uint64_t *two, uint64_t *none, uint64_t *blocked) {
    paths = two_owners = no_owner = stuck = 0;
    /* Start states: a fresh record, and records left by older jobs in each
     * outcome (the tag words and the decision of tag - 1). */
    static const uint32_t tags[3] = { 1u, 2u, SH2Z_CLAIM_TAG_MAX };
    for (unsigned t = 0; t < 3u; ++t) {
        const uint32_t tag = tags[t], old = tag - 1u;
        for (unsigned s = 0; s < 8u; ++s) {
            mem_t m = { (s & 1u) ? old : 0u, (s & 2u) ? old : 0u, (s & 4u) ? (old << 1) | 1u : old << 1 };
            side_t a = { 0, 0, 0, 0 }, b = { 0, 0, 0, 0 };
            explore(m, a, b, tag, bug);
        }
    }
    *two = two_owners; *none = no_owner; *blocked = stuck;
}

static void test_interleavings(void) {
    uint64_t two, none, blocked;
    run_model(BUG_NONE, &two, &none, &blocked);
    CHECK(paths > 0u && two == 0u && none == 0u && blocked == 0u);
    printf("dual-claim-1: %" PRIu64 " interleavings, one owner each\n", paths);
    /* Controls: each broken protocol must allow two owners. */
    run_model(BUG_A_NO_WAIT, &two, &none, &blocked);  CHECK(two > 0u);
    run_model(BUG_B_IGNORES_A, &two, &none, &blocked); CHECK(two > 0u);
    run_model(BUG_A_LOAD_FIRST, &two, &none, &blocked); CHECK(two > 0u);
    run_model(BUG_B_LOAD_FIRST, &two, &none, &blocked); CHECK(two > 0u);
}

/* The real functions, in each sequential order and over many tags. */
static void test_sequential(void) {
    sh2z_claim_t c;
    sh2z_claim_init(&c);
    CHECK(c.want_a == 0u && c.want_b == 0u && c.decide == 0u);
    for (uint32_t tag = 1u; tag < 2000u; ++tag) {
        switch (tag % 4u) {
        case 0: CHECK(sh2z_claim_a(&c, tag)); CHECK(!sh2z_claim_b(&c, tag)); CHECK(c.decide == tag << 1); break;
        case 1: CHECK(sh2z_claim_b(&c, tag)); CHECK(!sh2z_claim_a(&c, tag)); CHECK(c.decide == ((tag << 1) | 1u)); break;
        case 2: sh2z_claim_b_decline(&c, tag); CHECK(sh2z_claim_a(&c, tag)); break;
        case 3: CHECK(sh2z_claim_b(&c, tag)); break; /* A does not claim this job */
        }
    }
    /* A on a tag no one else claims owns it at once, whatever the old words. */
    c.want_b = 7u; c.decide = (7u << 1) | 1u;
    CHECK(sh2z_claim_a(&c, 8u));
    c.want_a = SH2Z_CLAIM_TAG_MAX - 1u;
    CHECK(sh2z_claim_b(&c, SH2Z_CLAIM_TAG_MAX) && c.decide == ((SH2Z_CLAIM_TAG_MAX << 1) | 1u));
}

int main(void) {
    test_interleavings();
    test_sequential();
    printf("dual-claim-1: %" PRIu64 " checks passed\n", checks);
    return EXIT_SUCCESS;
}
