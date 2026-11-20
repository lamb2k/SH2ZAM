/* Two-CPU try-claim (dual_claim_experimental.h, contract tag dual-claim-1).
 * Portable C over volatile words, no libc, no hardware access.
 *
 * Copyright (c) 2026 SH2ZAM Contributors. MIT License.
 * Inspired by SH4ZAM by Falco Girgis and SH4ZAM Contributors.
 */
#include "sh2zam/dual_claim_experimental.h"

void sh2z_claim_init(sh2z_claim_t *c) {
    c->want_a = 0u; c->want_b = 0u; c->decide = 0u;
}

bool sh2z_claim_a(sh2z_claim_t *c, uint32_t tag) {
    c->want_a = tag;
    if (c->want_b != tag) return true; /* B has not started: it will see want_a and back off */
    uint32_t d;
    while (((d = c->decide) >> 1) != tag) {}
    return (d & 1u) == 0u;
}

bool sh2z_claim_b(sh2z_claim_t *c, uint32_t tag) {
    c->want_b = tag;
    if (c->want_a == tag) { c->decide = tag << 1; return false; }
    c->decide = (tag << 1) | 1u;
    return true;
}

void sh2z_claim_b_decline(sh2z_claim_t *c, uint32_t tag) {
    c->decide = tag << 1;
}
