/* Experimental body contact (contract tag body-contact-1).
 *
 * Copyright (c) 2026 SH2ZAM Contributors. MIT License.
 * Inspired by SH4ZAM by Falco Girgis and SH4ZAM Contributors.
 */
#include "sh2zam/body_contact_experimental.h"
#include "sh2zam/divu_experimental.h"
#include "sh2zam/unit30_experimental.h"

#define BC_ONE30 INT32_C(0x40000000)

static int64_t bc_r64(int64_t x, unsigned s) {
    const int64_t h = (int64_t)1 << (s - 1u);
    return x >= 0 ? (x + h) >> s : -((-x + h) >> s);
}

static int32_t bc_sat(int64_t x) {
    if(x > INT32_MAX) return INT32_MAX;
    if(x < INT32_MIN) return INT32_MIN;
    return (int32_t)x;
}

/* a (Q16) times b (Q16). */
static int32_t bc_m16(int32_t a, int32_t b) { return bc_sat(bc_r64((int64_t)a * b, 16)); }
/* a (any) times u (Q30). */
static int32_t bc_mu(int32_t a, int32_t u) { return bc_sat(bc_r64((int64_t)a * u, 30)); }

/* n * 2^s / d, rounded to nearest, d > 0. */
static int32_t bc_divq(int64_t n, int32_t d, unsigned s) {
    const int64_t num = n * ((int64_t)1 << s);
    const int64_t h = d / 2;
    return sh2z_divu_div64(num >= 0 ? num + h : num - h, d, NULL);
}

static int32_t bc_abs(int32_t x) { return x < 0 ? -x : x; }

/* Weight of a body in this pair. */
static uint32_t bc_weight(const sh2z_bc_body_t *b, const sh2z_bc_rules_t *r) {
    uint32_t w = b->strength;
    if(b->flags & SH2Z_BC_WEAK) w = (uint32_t)(((uint64_t)w * (uint32_t)r->weak_share + 0x8000u) >> 16);
    return w ? w : 1u;
}

/* Adds k * u (u Q30) to a Q16 vector. */
static void bc_add(int32_t o[3], int32_t k, const int32_t u[3]) {
    int i;
    for(i = 0; i < 3; i++) o[i] = bc_sat((int64_t)o[i] + bc_mu(k, u[i]));
}

uint32_t sh2z_bc_pair(sh2z_bc_body_t *a, sh2z_bc_body_t *b, uint32_t ia, uint32_t ib, bool touching_before,
                      const sh2z_bc_rules_t *r) {
    int64_t dx[3], d2 = 0, closing = 0;
    int32_t dv[3], n[3], d, share_a, share_b, len;
    uint32_t wa, wb, ev = SH2Z_BC_EV_TOUCH;
    const bool atk_a = (a->flags & SH2Z_BC_ATTACK) != 0, atk_b = (b->flags & SH2Z_BC_ATTACK) != 0;
    int i;

    for(i = 0; i < 3; i++) {
        dx[i] = (int64_t)b->pos[i] - a->pos[i];
        if(dx[i] > r->reach || dx[i] < -r->reach) return 0u;
    }
    for(i = 0; i < 3; i++) d2 += dx[i] * dx[i];
    if(d2 > (int64_t)r->reach * r->reach) return 0u;
    d = (int32_t)sh2z_isqrt64_q12((uint64_t)d2);   /* Q32 -> Q16 */
    for(i = 0; i < 3; i++) {
        dv[i] = bc_sat((int64_t)b->vel[i] - a->vel[i]);
        closing += dx[i] * dv[i];
    }
    if(d < r->near) {
        if(!sh2z_vec3_normalize_u30(n, dv)) n[0] = n[1] = n[2] = 0;
    } else {
        int32_t dxs[3];
        for(i = 0; i < 3; i++) dxs[i] = (int32_t)dx[i];   /* |dx| <= reach */
        if(!sh2z_vec3_normalize_u30(n, dxs)) n[0] = n[1] = n[2] = 0;
    }
    wa = bc_weight(a, r);
    wb = bc_weight(b, r);
    share_a = bc_divq((int64_t)wa, (int32_t)((((uint64_t)wa + wb) + 1u) >> 1), 29); /* wa / (wa + wb), Q30 */
    if(share_a > BC_ONE30) share_a = BC_ONE30;
    share_b = BC_ONE30 - share_a;

    if(!touching_before) {
        int64_t p = 0;
        int32_t impact;
        for(i = 0; i < 3; i++) p += (int64_t)n[i] * dv[i];
        impact = bc_m16(bc_abs(bc_sat(bc_r64(p, 30))), r->impact_gain);
        if(atk_a == atk_b) {
            a->damage = bc_sat((int64_t)a->damage + bc_mu(impact, share_b));
            b->damage = bc_sat((int64_t)b->damage + bc_mu(impact, share_a));
        } else if(atk_b) {
            a->damage = bc_sat((int64_t)a->damage + ((b->flags & SH2Z_BC_HEAVY) ? bc_m16(impact, r->heavy_gain) : impact));
        } else {
            b->damage = bc_sat((int64_t)b->damage + ((a->flags & SH2Z_BC_HEAVY) ? bc_m16(impact, r->heavy_gain) : impact));
        }
        /* Two credited contacts on one tick: the lower index keeps it, whatever the walk order. */
        if((a->flags & SH2Z_BC_CREDIT) && !(b->since == 0u && b->toucher >= 0 && (uint32_t)b->toucher < ia)) {
            b->toucher = (int16_t)ia; b->since = 0u;
        }
        if((b->flags & SH2Z_BC_CREDIT) && !(a->since == 0u && a->toucher >= 0 && (uint32_t)a->toucher < ib)) {
            a->toucher = (int16_t)ib; a->since = 0u;
        }
        if(a->first < 255u) a->first++;
        if(b->first < 255u) b->first++;
        ev |= SH2Z_BC_EV_FIRST;
    }

    if(closing <= 0) {
        if(d < r->near) {
            len = -r->reach;
        } else {
            len = r->reach - d;
            if((atk_a || atk_b) && len < r->attack_min) len = r->attack_min;
        }
        if(atk_a != atk_b) {
            if(atk_a) { bc_add(b->push, bc_m16(len, r->attack_push), n); b->hit = 1u; }
            else { bc_add(a->push, -bc_m16(len, r->attack_push), n); a->hit = 1u; }
            ev |= SH2Z_BC_EV_HIT;
        } else {
            bc_add(a->push, -bc_mu(len, share_b), n);
            bc_add(b->push, bc_mu(len, share_a), n);
        }
        ev |= SH2Z_BC_EV_PUSH;
    }
    return ev;
}

void sh2z_bc_field_init(sh2z_bc_field_t *f, sh2z_bc_body_t *body, uint32_t *last, uint32_t count) {
    uint32_t i, pairs = count > 1u ? count * (count - 1u) / 2u : 0u;
    f->body = body;
    f->last = last;
    f->count = count;
    f->tick = 0u;
    f->contacts = 0u;
    f->tested = 0u;
    for(i = 0; i < pairs; i++) last[i] = 0u;
    for(i = 0; i < count; i++) {
        sh2z_bc_body_t *b = &body[i];
        b->push[0] = b->push[1] = b->push[2] = 0;
        b->damage = 0;
        b->hit = 0u;
        b->first = 0u;
        b->toucher = -1;
        b->since = UINT32_MAX;
    }
}

void sh2z_bc_field_begin(sh2z_bc_field_t *f) {
    uint32_t i;
    f->tick++;
    for(i = 0; i < f->count; i++) {
        sh2z_bc_body_t *b = &f->body[i];
        b->push[0] = b->push[1] = b->push[2] = 0;
        b->damage = 0;
        b->hit = 0u;
        b->first = 0u;
        if(b->since != UINT32_MAX) b->since++;
    }
}

uint32_t sh2z_bc_field_resolve(sh2z_bc_field_t *f, const uint8_t *order, const int32_t *along, int32_t length,
                               const sh2z_bc_rules_t *r) {
    const uint32_t n = f->count;
    const int32_t half = length / 2;
    uint32_t s, k, touching = 0u, tested = 0u;
    for(s = 0u; s < n; s++) {
        const uint32_t i = order[s];
        if(!(f->body[i].flags & SH2Z_BC_ACTIVE)) continue;
        for(k = 1u; k < n; k++) {
            const uint32_t j = order[s + k < n ? s + k : s + k - n];
            int32_t gap = along[i] - along[j];
            uint32_t lo, hi, idx, ev;
            if(gap > half) gap -= length;
            else if(gap <= -half) gap += length;
            if(gap > r->reach_long) break;
            /* Each pair once: from the body ahead (ties: before the wrap). */
            if(gap < 0 || (gap == 0 && s + k >= n)) continue;
            if(!(f->body[j].flags & SH2Z_BC_ACTIVE)) continue;
            lo = i < j ? i : j;
            hi = i < j ? j : i;
            idx = hi * (hi - 1u) / 2u + lo;
            tested++;
            ev = sh2z_bc_pair(&f->body[lo], &f->body[hi], lo, hi, f->last[idx] != 0u && f->last[idx] + 1u == f->tick, r);
            if(ev & SH2Z_BC_EV_TOUCH) {
                f->last[idx] = f->tick;
                touching++;
                if(ev & SH2Z_BC_EV_FIRST) f->contacts++;
            }
        }
    }
    f->tested = tested;
    return touching;
}

int32_t sh2z_bc_credit(const sh2z_bc_field_t *f, uint32_t victim, const sh2z_bc_rules_t *r) {
    const sh2z_bc_body_t *b = &f->body[victim];
    if(b->toucher < 0 || b->since > r->credit_window) return -1;
    return b->toucher;
}
