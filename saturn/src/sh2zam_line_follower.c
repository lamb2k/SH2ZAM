/* Experimental racing-line follower (contract tag line-follower-1).
 *
 * Copyright (c) 2026 SH2ZAM Contributors. MIT License.
 * Inspired by SH4ZAM by Falco Girgis and SH4ZAM Contributors.
 */
#include "sh2zam/line_follower_experimental.h"

#if defined(__GNUC__)
#define LF_INLINE static inline __attribute__((always_inline))
#else
#define LF_INLINE static inline
#endif

LF_INLINE int32_t lf_clamp(int32_t v, int32_t lo, int32_t hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}
LF_INLINE int32_t lf_abs(int32_t v) { return v < 0 ? -v : v; }
/* (a * gain) >> 16 with a 64-bit product (DMULS on SH-2), rounding toward -inf. */
LF_INLINE int32_t lf_mul_q16(int32_t a, int32_t gain) {
    return (int32_t)(((int64_t)a * gain) >> 16);
}

uint32_t sh2z_line_node_inv_span(int32_t span) {
    if(span < 2 || span > 65536) return 0u;
    /* floor(2^32 / span): offline only, so a 64-bit divide is fine. */
    return (uint32_t)((UINT64_C(1) << 32) / (uint64_t)(uint32_t)span);
}

uint32_t sh2z_line_locate(const sh2z_line_t *line, int32_t dist, uint32_t hint) {
    const sh2z_line_node_t *n = line->nodes;
    uint32_t h = hint < line->count ? hint : 0u;
    if(dist < n[h].start) {
        while(h > 0u && n[h].start > dist) --h;
    } else {
        while(h + 1u < line->count && n[h + 1u].start <= dist) ++h;
    }
    return h;
}

void sh2z_line_sample(const sh2z_line_t *line, uint32_t node, int32_t dist, sh2z_line_sample_t *out) {
    const sh2z_line_node_t *a = &line->nodes[node];
    const sh2z_line_node_t *b = node + 1u < line->count ? a + 1 : &line->nodes[0];
    int32_t d = dist - a->start;
    if(d < 0) d = 0;
    /* frac in Q16: d * floor(2^32/span) >> 16, below 65536 for d < span. */
    int32_t f = (int32_t)(((uint64_t)(uint32_t)d * a->inv_span) >> 16);
    if(f > 65535) f = 65535;
    const int32_t la = (int32_t)a->lateral, lb = (int32_t)b->lateral;
    const int32_t ca = (int32_t)a->speed_cap, cb = (int32_t)b->speed_cap;
    const int32_t wa = (int32_t)a->half_width, wb = (int32_t)b->half_width;
    out->lateral = la * 256 + (int32_t)(((int64_t)(lb - la) * f) >> 8);
    out->speed_cap = ca * 16 + (int32_t)(((int64_t)(cb - ca) * f) >> 12);
    out->half_width = wa * 256 + (int32_t)(((int64_t)(wb - wa) * f) >> 8);
    out->flags = a->flags;
}

void sh2z_track_advance(const sh2z_line_t *line, sh2z_track_state_t *s, uint32_t steps) {
    int32_t d = s->dist + s->speed * (int32_t)steps;
    while(d >= line->length) { d -= line->length; ++s->lap; }
    while(d < 0) { d += line->length; --s->lap; }
    s->dist = d;
    s->node = sh2z_line_locate(line, d, s->node);
}

void sh2z_follow_step(const sh2z_line_t *line, const sh2z_follow_params_t *params, sh2z_track_state_t *s,
                      int32_t target_speed, int32_t target_lateral, uint32_t steps) {
    sh2z_line_sample_t at;
    if(steps < 1u) steps = 1u;
    if(steps > 64u) steps = 64u;
    const int32_t n = (int32_t)steps;
    sh2z_line_sample(line, s->node, s->dist, &at);
    /* Speed: toward min(target, cap) with n-scaled acceleration and braking. */
    int32_t want = target_speed < at.speed_cap ? target_speed : at.speed_cap;
    if(want < 0) want = 0;
    if(want > s->speed) {
        const int32_t up = s->speed + params->accel * n;
        s->speed = up < want ? up : want;
    } else {
        const int32_t down = s->speed - params->brake * n;
        s->speed = down > want ? down : want;
    }
    /* Lateral: proportional steering, rate-limited, kept inside the half width. */
    int32_t v = lf_mul_q16(target_lateral - s->lateral, params->steer_gain);
    v = lf_clamp(v, -params->steer_max, params->steer_max);
    int32_t limit = at.half_width - params->margin;
    if(limit < 0) limit = 0;
    int32_t lat = s->lateral + v * n;
    if(lat > limit) { lat = limit; v = 0; }
    else if(lat < -limit) { lat = -limit; v = 0; }
    s->lateral = lat;
    s->lat_vel = v;
    sh2z_track_advance(line, s, steps);
}

int32_t sh2z_pace_target(int32_t base, int32_t expected_total, int32_t actual_total, int32_t gain_q16,
                         int32_t lo, int32_t hi) {
    const int64_t t = (int64_t)base + (((int64_t)expected_total - actual_total) * gain_q16 >> 16);
    if(t < lo) return lo;
    if(t > hi) return hi;
    return (int32_t)t;
}

int32_t sh2z_follow_avoid(const sh2z_line_t *line, const sh2z_track_state_t *self, const sh2z_track_state_t *ahead,
                          int32_t own_target, int32_t reach_long, int32_t reach_lat, int32_t offset) {
    const int32_t gap = sh2z_track_near_gap(line, ahead, self);
    if(gap <= 0 || gap >= reach_long) return own_target;
    if(lf_abs(ahead->lateral - own_target) >= reach_lat) return own_target;
    return own_target <= ahead->lateral ? ahead->lateral - offset : ahead->lateral + offset;
}

uint32_t sh2z_lod_period(int32_t gap, const int32_t near[3]) {
    const int32_t g = lf_abs(gap);
    if(g < near[0]) return 1u;
    if(g < near[1]) return 2u;
    if(g < near[2]) return 4u;
    return 8u;
}

void sh2z_rank_sort(uint8_t *order, const sh2z_line_t *line, const sh2z_track_state_t *states, uint32_t count) {
    for(uint32_t i = 1u; i < count; ++i) {
        const uint8_t k = order[i];
        const int32_t kt = sh2z_track_total(line, &states[k]);
        uint32_t j = i;
        while(j > 0u) {
            const uint8_t p = order[j - 1u];
            const int32_t pt = sh2z_track_total(line, &states[p]);
            if(pt > kt || (pt == kt && p < k)) break;
            order[j] = p;
            --j;
        }
        order[j] = k;
    }
}

void sh2z_order_by_dist(uint8_t *order, const sh2z_track_state_t *states, uint32_t count) {
    for(uint32_t i = 1u; i < count; ++i) {
        const uint8_t k = order[i];
        const int32_t kd = states[k].dist;
        uint32_t j = i;
        while(j > 0u) {
            const uint8_t p = order[j - 1u];
            const int32_t pd = states[p].dist;
            if(pd > kd || (pd == kd && p < k)) break;
            order[j] = p;
            --j;
        }
        order[j] = k;
    }
}

/* Pushes a (ahead) and b (behind) apart when they touch. */
LF_INLINE uint32_t lf_contact_pair(const sh2z_line_t *line, const sh2z_contact_params_t *p,
                                   sh2z_track_state_t *a, sh2z_track_state_t *b, int32_t gap, int move_a) {
    if(gap < 0 || gap >= p->reach_long) return 0u;
    const int32_t dl = a->lateral - b->lateral;
    const int32_t adl = lf_abs(dl);
    if(adl >= p->reach_lat) return 0u;
    const int32_t overlap = p->reach_lat - adl;
    const int32_t push = move_a ? (overlap + 1) >> 1 : overlap;
    sh2z_line_sample_t at;
    /* Side: a goes to its side of b; on equal laterals a goes right. */
    const int32_t dir = dl >= 0 ? 1 : -1;
    if(move_a) {
        sh2z_line_sample(line, a->node, a->dist, &at);
        a->lateral = lf_clamp(a->lateral + dir * push, -at.half_width, at.half_width);
    }
    sh2z_line_sample(line, b->node, b->dist, &at);
    b->lateral = lf_clamp(b->lateral - dir * (move_a ? overlap - push : push), -at.half_width, at.half_width);
    /* Bump: the trailing agent cannot keep much more speed than the leader. */
    const int32_t keep = a->speed + lf_mul_q16(b->speed - a->speed, p->bump_keep);
    if(b->speed > keep) b->speed = keep;
    return 1u;
}

uint32_t sh2z_contact_resolve(const sh2z_line_t *line, const sh2z_contact_params_t *params,
                              sh2z_track_state_t *states, const uint8_t *order, uint32_t count) {
    uint32_t contacts = 0u;
    if(count < 2u) return 0u;
    for(uint32_t i = 0u; i < count; ++i) {
        sh2z_track_state_t *a = &states[order[i]];
        /* Scan agents behind a (wrapping) while they are within reach. */
        for(uint32_t k = 1u; k < count; ++k) {
            const uint32_t j = i + k < count ? i + k : i + k - count;
            sh2z_track_state_t *b = &states[order[j]];
            const int32_t gap = sh2z_track_near_gap(line, a, b);
            if(gap < 0 || gap >= params->reach_long) break;
            contacts += lf_contact_pair(line, params, a, b, gap, 1);
        }
    }
    return contacts;
}

uint32_t sh2z_contact_external(const sh2z_line_t *line, const sh2z_contact_params_t *params,
                               sh2z_track_state_t *states, uint32_t count, const sh2z_track_state_t *body) {
    uint32_t contacts = 0u;
    sh2z_track_state_t ghost;
    for(uint32_t i = 0u; i < count; ++i) {
        sh2z_track_state_t *s = &states[i];
        const int32_t gap = sh2z_track_near_gap(line, body, s);
        if(gap >= 0) {
            /* Body ahead of the agent: only the agent (trailing) moves. */
            ghost = *body;
            contacts += lf_contact_pair(line, params, &ghost, s, gap, 0);
        } else {
            /* Agent ahead: push the agent away from the body, no speed change. */
            if(-gap >= params->reach_long) continue;
            const int32_t dl = s->lateral - body->lateral;
            const int32_t adl = lf_abs(dl);
            if(adl >= params->reach_lat) continue;
            sh2z_line_sample_t at;
            sh2z_line_sample(line, s->node, s->dist, &at);
            const int32_t dir = dl >= 0 ? 1 : -1;
            s->lateral = lf_clamp(s->lateral + dir * (params->reach_lat - adl), -at.half_width, at.half_width);
            ++contacts;
        }
    }
    return contacts;
}
