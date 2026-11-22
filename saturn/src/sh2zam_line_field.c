/* Experimental line field (contract tags line-field-1 to line-field-5,
 * line-field-7 to line-field-10, line-field-12 to line-field-14).
 *
 * Copyright (c) 2026 SH2ZAM Contributors. MIT License.
 * Inspired by SH4ZAM by Falco Girgis and SH4ZAM Contributors.
 */
#include "sh2zam/line_field_experimental.h"

#if defined(__GNUC__)
#define FLD_INLINE static inline __attribute__((always_inline))
#define FLD_NOINLINE __attribute__((noinline)) static
#else
#define FLD_INLINE static inline
#define FLD_NOINLINE static
#endif

#define FLD_TOUCH_COOL 30u   /* ticks a contact incident lasts for energy accounting */

FLD_INLINE int32_t fld_abs(int32_t v) { return v < 0 ? -v : v; }
FLD_INLINE int32_t fld_clamp(int32_t v, int32_t lo, int32_t hi) { return v < lo ? lo : (v > hi ? hi : v); }
FLD_INLINE int32_t fld_mul_q16(int32_t a, int32_t g) { return (int32_t)(((int64_t)a * g) >> 16); }
FLD_INLINE uint32_t fld_rand(sh2z_field_t *f) {
    uint32_t x = f->rng;
    x ^= x << 13; x ^= x >> 17; x ^= x << 5;
    f->rng = x;
    return x;
}
/* In the way of others: a retired vehicle leaves contact and avoidance at once. */
FLD_INLINE int fld_active(const sh2z_vehicle_t *v) { return !(v->flags & (SH2Z_VEH_RETIRED | SH2Z_VEH_STOPPED | SH2Z_VEH_FALLING)); }
/* line-field-5: node flags against a mask pair (a zero mask matches nothing). */
FLD_INLINE int fld_match(uint16_t flags, uint16_t mask, uint16_t value) { return mask && (flags & mask) == value; }

/* line-field-4. A tube node: the surface's wrap mask matches its flags. */
FLD_INLINE int fld_tube(const sh2z_field_params_t *p, uint16_t flags) {
    const sh2z_field_surface_t *s = p->surface;
    return s && s->wrap_mask && (flags & s->wrap_mask) == s->wrap_value;
}
/* Round a tube of radius half_width: the circumference (411775 = 2 pi in Q16, rounded). */
FLD_INLINE int32_t fld_round(int32_t half_width) { return fld_mul_q16(half_width, 411775); }
/* A lateral (or a lateral difference) wrapped into [-c/2, c - c/2) for a circumference c > 0. */
FLD_INLINE int32_t fld_wrap(int32_t lat, int32_t c) {
    const int32_t h = c >> 1;
    while(lat >= c - h) lat -= c;
    while(lat < -h) lat += c;
    return lat;
}
/* line-field-7: the last node starting at or before dist (else 0), by halving. */
uint32_t sh2z_line_locate_far(const sh2z_line_t *line, int32_t dist) {
    const sh2z_line_node_t *n = line->nodes;
    uint32_t lo = 0u, hi = line->count;
    while(hi - lo > 1u) {
        const uint32_t mid = (lo + hi) >> 1;
        if(n[mid].start <= dist) lo = mid;
        else hi = mid;
    }
    return lo;
}
/* The zone holding a line state when its step starts, or 0. Zones are sorted by start. */
static uint32_t fld_zone(const sh2z_field_surface_t *s, const sh2z_track_state_t *t) {
    for(uint32_t k = 0u; k < s->zone_count; ++k) {
        const sh2z_field_zone_t *z = &s->zones[k];
        if(t->dist < z->start) break;
        if(t->dist < z->end && t->lateral >= z->lat_lo && t->lateral <= z->lat_hi) return z->kind;
    }
    return 0u;
}

void sh2z_field_grid_slot(const sh2z_line_t *line, uint32_t p, int32_t first_back, int32_t row_step,
                          int32_t column_half, uint32_t columns, sh2z_track_state_t *out) {
    if(columns < 1u) columns = 1u;
    if(columns > 4u) columns = 4u;
    /* p % columns without a divide: columns is 1..4. */
    uint32_t col = columns == 4u ? (p & 3u) : columns == 2u ? (p & 1u) : columns == 1u ? 0u : p;
    if(columns == 3u) while(col >= 3u) col -= 3u;
    int32_t d = line->length - first_back - (int32_t)p * row_step;
    int32_t lap = -1;
    while(d < 0) { d += line->length; --lap; }
    while(d >= line->length) { d -= line->length; ++lap; }
    out->dist = d;
    out->lap = lap;
    out->lateral = ((int32_t)columns - 1 - 2 * (int32_t)col) * column_half;
    out->speed = 0;
    out->lat_vel = 0;
    out->node = sh2z_line_locate(line, d, 0u);
}

void sh2z_field_init(sh2z_field_t *f, const sh2z_line_t *line, const sh2z_field_params_t *params,
                     sh2z_vehicle_t *veh, sh2z_field_brain_t *brain, uint8_t *order, uint8_t *slot,
                     uint8_t *rank_order, uint32_t count, uint32_t seed) {
    f->line = line; f->params = params; f->veh = veh; f->brain = brain;
    f->order = order; f->slot = slot; f->rank_order = rank_order; f->count = count;
    f->tick = 0u; f->start_tick = 0u; f->rng = seed ? seed : 1u; f->focus = -1; f->started = 0u;
    f->finished = 0u; f->retired = 0u; f->contacts = 0u; f->rank_changes = 0u; f->steps = 0u;
    for(uint32_t i = 0u; i < count; ++i) {
        sh2z_field_brain_t *b = &brain[i];
        order[i] = (uint8_t)i; slot[i] = (uint8_t)i; rank_order[i] = (uint8_t)i;
        veh[i].id = (uint8_t)i; veh[i].rank = (uint8_t)i;
        veh[i].kick_lat = 0; veh[i].kick_speed = 0; veh[i].boost_ticks = 0;
        b->expect = 0; b->target_lat = veh[i].t.lateral; b->target_speed = 0; b->last_tick = 0u;
        for(int k = 0; k < 4; ++k) b->lap_tick[k] = -1;
        b->finish_tick = -1; b->retire_tick = -1; b->laps_seen = (int16_t)veh[i].t.lap;
        b->pass_ticks = 0u; b->pass_dir = 0; b->attack_ticks = 0u; b->attack_target = 0u; b->finish_seq = 0u; b->touch_cool = 0u;
        b->fall_tick = -1; b->air_drift = 0; b->roll = 0; b->ko_by = -1; b->ko_credit = -1; b->ko_age = 0xffffu;
    }
    f->falls = 0u; f->kos = 0u;
}

void sh2z_field_start(sh2z_field_t *f) {
    f->started = 1u;
    f->start_tick = f->tick;
    for(uint32_t i = 0u; i < f->count; ++i) {
        f->brain[i].expect = sh2z_track_total(f->line, &f->veh[i].t);
        f->brain[i].last_tick = f->tick;
        f->brain[i].laps_seen = (int16_t)f->veh[i].t.lap;
    }
}

/* Race key for ranking: finishers first in finishing order, then by total
 * distance, retired vehicles last. Totals lie inside (-2^29, 2^29). */
FLD_INLINE int32_t fld_rank_key(const sh2z_field_t *f, uint32_t i) {
    const sh2z_vehicle_t *v = &f->veh[i];
    if(v->flags & SH2Z_VEH_FINISHED) return INT32_MAX - (int32_t)f->brain[i].finish_seq;
    /* A vehicle on a level-of-detail period is ranked where it is by now. */
    const int32_t stale = (v->flags & SH2Z_VEH_EXTERNAL) ? 0 : (int32_t)(f->tick - f->brain[i].last_tick);
    const int32_t total = sh2z_track_total(f->line, &v->t) + v->t.speed * stale;
    if(v->flags & SH2Z_VEH_RETIRED) return total - (INT32_C(1) << 30);
    return total;
}

static uint32_t fld_rank(sh2z_field_t *f) {
    uint8_t *o = f->rank_order;
    uint32_t moved = 0u;
    /* line-field-7: the key of o[i - 1] carries over from the previous pass (keys do not change while sorting). */
    int32_t last = f->count ? fld_rank_key(f, o[0]) : 0;
    for(uint32_t i = 1u; i < f->count; ++i) {
        const uint8_t k = o[i];
        const int32_t kk = fld_rank_key(f, k);
        int32_t next = kk;   /* the key that ends at o[i] */
        uint32_t j = i;
        while(j > 0u) {
            const uint8_t p = o[j - 1u];
            const int32_t pk = j == i ? last : fld_rank_key(f, p);
            /* Between running vehicles a place changes hands only on a clear lead
             * (no flicker side by side); finishing order is exact. */
            if((f->veh[p].flags | f->veh[k].flags) & SH2Z_VEH_FINISHED) {
                if(pk > kk || (pk == kk && p < k)) break;
            } else if((int64_t)pk + f->params->rank_lead >= kk) break;
            if(j == i) next = pk;
            o[j] = p;
            --j;
        }
        if(j != i) { o[j] = k; moved = 1u; }
        last = next;
    }
    for(uint32_t i = 0u; i < f->count; ++i) f->veh[o[i]].rank = (uint8_t)i;
    return moved;
}

/* Nearest active vehicle ahead of i in physical order, or -1. */
static int32_t fld_ahead(const sh2z_field_t *f, uint32_t i) {
    uint32_t s = f->slot[i];
    for(uint32_t k = 1u; k < f->count; ++k) {
        s = s ? s - 1u : f->count - 1u;
        const uint32_t j = f->order[s];
        if(fld_active(&f->veh[j])) return (int32_t)j;
    }
    return -1;
}

/* ---- line-field-12: pace ranks ---- */
/* Finish target of ranked agent i from its rank and distance covered; its pace and expectation follow. */
static void fld_retarget(sh2z_field_t *f, uint32_t i) {
    const sh2z_field_ranks_t *rk = f->params->ranks;
    sh2z_vehicle_t *v = &f->veh[i];
    sh2z_field_brain_t *b = &f->brain[i];
    if(b->pace_rank == 0u || b->pace_rank >= 254u || b->laps_seen < 1 || b->lap_tick[0] < 0 || rk->step_count == 0u) return;
    if(v->flags & (SH2Z_VEH_EXTERNAL | SH2Z_VEH_RETIRED | SH2Z_VEH_FINISHED)) return;
    const int32_t whole = (int32_t)f->params->laps * f->line->length;
    int32_t left_dist = whole - sh2z_track_total(f->line, &v->t);
    if(left_dist < 0) left_dist = 0;
    const uint32_t r = b->pace_rank - 1u < rk->step_count ? b->pace_rank - 1u : rk->step_count - 1u;
    const int32_t finish = rk->base - (int32_t)(((int64_t)rk->share * left_dist) / whole) + rk->step[r];
    const int32_t left = finish - b->lap_tick[0];
    if(left <= 0) return;
    b->finish_ticks = finish;
    b->pace = ((int32_t)f->params->laps - 1) * f->line->length / left;
    b->expect = f->line->length + ((int32_t)(f->tick - f->start_tick) - b->lap_tick[0]) * b->pace;
}

/* Renumber the ranked agents 1.. in the order (rank, larger distance covered first, lower index). */
static void fld_renumber(sh2z_field_t *f) {
    uint8_t idx[256];
    uint32_t n = 0u;
    for(uint32_t i = 0u; i < f->count; ++i) {
        const sh2z_field_brain_t *b = &f->brain[i];
        if(b->pace_rank >= 1u && b->pace_rank < 254u && !(f->veh[i].flags & SH2Z_VEH_EXTERNAL)) idx[n++] = (uint8_t)i;
    }
    for(uint32_t a = 1u; a < n; ++a) {   /* insertion sort, stable */
        const uint8_t x = idx[a];
        const int32_t xt = sh2z_track_total(f->line, &f->veh[x].t);
        uint32_t k = a;
        while(k > 0u) {
            const uint8_t y = idx[k - 1u];
            const int32_t yt = sh2z_track_total(f->line, &f->veh[y].t);
            const int before = f->brain[x].pace_rank < f->brain[y].pace_rank ||
                               (f->brain[x].pace_rank == f->brain[y].pace_rank && xt > yt);
            if(!before) break;
            idx[k] = y; --k;
        }
        idx[k] = x;
    }
    for(uint32_t a = 0u; a < n; ++a) {
        sh2z_field_brain_t *b = &f->brain[idx[a]];
        const uint8_t nr = (uint8_t)(a + 1u);
        if(b->pace_rank != nr) { b->pace_rank = nr; fld_retarget(f, idx[a]); }
    }
}

/* A chaser joins the ranking at the rank of the nearest ranked agent. */
static void fld_release(sh2z_field_t *f, uint32_t i) {
    sh2z_field_brain_t *b = &f->brain[i];
    const int32_t mine = sh2z_track_total(f->line, &f->veh[i].t);
    int32_t best = 0x7fffffff;
    uint8_t rank = 1u;
    for(uint32_t j = 0u; j < f->count; ++j) {
        const sh2z_field_brain_t *c = &f->brain[j];
        if(j == i || c->pace_rank < 1u || c->pace_rank >= 254u || (f->veh[j].flags & SH2Z_VEH_EXTERNAL)) continue;
        int32_t d = sh2z_track_total(f->line, &f->veh[j].t) - mine;
        if(d < 0) d = -d;
        if(d < best) { best = d; rank = c->pace_rank; }
    }
    b->pace_rank = rank;
    fld_renumber(f);
    fld_retarget(f, i);
}

FLD_INLINE void fld_think(sh2z_field_t *f, uint32_t i, uint32_t due, const sh2z_line_sample_t *at,
                          const sh2z_field_behave_t *bh) {
    const sh2z_field_params_t *p = f->params;
    sh2z_vehicle_t *v = &f->veh[i];
    sh2z_field_brain_t *b = &f->brain[i];
    const int32_t total = sh2z_track_total(f->line, &v->t);
    const int32_t base_lat = at->lateral + b->lane;

    if(v->flags & SH2Z_VEH_RETIRED) { b->target_speed = 0; b->target_lat = v->t.lateral; return; }
    if(v->flags & SH2Z_VEH_FINISHED) { b->target_speed = p->cruise; b->target_lat = base_lat; return; }
    /* line-field-4: the final dash. */
    const int dash = p->dash_from > 0 && v->t.lap == (int32_t)p->laps - 1 && v->t.dist >= p->dash_from;

    /* ---- pace ---- */
    int32_t want;
    const int32_t cap = fld_mul_q16(b->pace, p->pace_max);
    int32_t behind = b->expect - total;
    int chasing = b->chase >= 0 && (uint32_t)b->chase < f->count &&
                        !(f->veh[b->chase].flags & (SH2Z_VEH_RETIRED | SH2Z_VEH_FINISHED));
    int released = 0;
    if(p->ranks && chasing && b->pace_rank == 0u && b->laps_seen >= 1 && p->ranks->release_dist > 0 &&
       v->t.lap == (int32_t)p->laps - 1 && v->t.dist > p->ranks->release_dist) {
        /* line-field-12: a chaser in the last lap past the release distance joins the ranking. */
        b->chase = -1;
        chasing = 0;
        fld_release(f, i);
        released = 1;
    }
    if(!chasing && b->chase >= 0) {
        /* The chased vehicle is gone: carry on at our own pace from here. */
        b->chase = -1;
        b->expect = total;
        behind = 0;
        if(p->ranks && b->pace_rank == 0u && b->laps_seen >= 1) { fld_release(f, i); released = 1; }
    }
    if(released) behind = b->expect - total;
    const int after = p->ranks && p->ranks->after_speed > 0 && f->focus >= 0 && (uint32_t)f->focus < f->count &&
                      (f->veh[f->focus].flags & SH2Z_VEH_FINISHED) && !(f->veh[f->focus].flags & SH2Z_VEH_RETIRED);
    if(after) {
        /* line-field-12: the focus vehicle has finished: no paces and no chases, every agent drives on at full throttle. */
        want = p->ranks->after_speed;
        if(want > v->top_speed) want = v->top_speed;
        behind = 0;
    } else if(dash) {
        /* At least dash_pace of its pace, whatever its expectation. */
        want = sh2z_pace_target(b->pace, b->expect, total, p->pace_gain, b->pace >> 1, cap);
        const int32_t floor = fld_mul_q16(b->pace, p->dash_pace);
        if(want < floor) want = floor;
        if(want > v->top_speed) want = v->top_speed;
    } else if(chasing) {
        const sh2z_vehicle_t *c = &f->veh[b->chase];
        behind = sh2z_track_total(f->line, &c->t) + (int32_t)b->chase_lead * 256 - total;
        want = c->t.speed + fld_mul_q16(behind, p->chase_gain);
        if(want > v->top_speed) want = v->top_speed;
        if(b->first_lap_cap > 0 && v->t.lap < 1 && want > b->first_lap_cap) want = b->first_lap_cap; /* line-field-3 */
        if(want < (b->pace >> 1)) want = b->pace >> 1;
    } else {
        want = sh2z_pace_target(b->pace, b->expect, total, p->pace_gain, b->pace >> 1, cap);
        if(want > v->top_speed) want = v->top_speed;
    }
    /* ---- boost ---- */
    if(!after && !(v->flags & SH2Z_VEH_BOOST) && v->t.lap >= (int32_t)p->boost_from_lap && (dash || behind > p->boost_behind) &&
       at->speed_cap >= p->boost_min_cap &&
       (int64_t)v->energy * 65536 > (int64_t)v->max_energy * p->boost_min_share && v->energy > p->boost_energy) {
        v->flags |= SH2Z_VEH_BOOST;
        v->boost_ticks = p->boost_ticks;
        v->energy -= p->boost_energy;
    }
    if(v->flags & SH2Z_VEH_BOOST) {
        want += p->boost_speed;
        v->boost_ticks = (int16_t)(v->boost_ticks - (int16_t)due);
        if(v->boost_ticks <= 0) { v->boost_ticks = 0; v->flags &= (uint16_t)~SH2Z_VEH_BOOST; }
    }
    b->target_speed = want;

    /* ---- lateral ---- */
    int32_t lat = base_lat;
    const int32_t a = fld_ahead(f, i);
    if(b->attack_ticks) {
        const sh2z_vehicle_t *t = &f->veh[b->attack_target];
        b->attack_ticks = (uint8_t)(b->attack_ticks > due ? b->attack_ticks - due : 0u);
        if(fld_active(t) && fld_abs(sh2z_track_near_gap(f->line, &t->t, &v->t)) < p->attack_long) lat = t->t.lateral;
        else b->attack_ticks = 0u;
    } else if(b->pass_ticks) {
        b->pass_ticks = (uint8_t)(b->pass_ticks > due ? b->pass_ticks - due : 0u);
        lat = b->target_lat;                       /* hold the chosen side */
        if(a >= 0) {
            const sh2z_vehicle_t *av = &f->veh[a];
            const int32_t gap = sh2z_track_near_gap(f->line, &av->t, &v->t);
            if(gap > 0 && gap < p->avoid_long) lat = av->t.lateral + (int32_t)b->pass_dir * p->avoid_offset;
        }
        if(!b->pass_ticks) b->pass_dir = 0;
    } else {
        if(a >= 0) {
            const sh2z_vehicle_t *av = &f->veh[a];
            const int32_t gap = sh2z_track_near_gap(f->line, &av->t, &v->t);
            const int32_t closing = v->t.speed - av->t.speed;
            /* Look ahead far enough to clear sideways before reaching the vehicle. */
            const int32_t reach = p->avoid_long + (closing > 0 ? closing * p->avoid_ticks : 0);
            if(gap > 0 && gap < reach && closing > -64 &&
               fld_abs(av->t.lateral - base_lat) < p->avoid_lat) {
                /* Take the side with more room; a tie is a coin flip. */
                const int32_t room_right = at->half_width - av->t.lateral;
                const int32_t room_left = at->half_width + av->t.lateral;
                int8_t dir = room_right > room_left ? 1 : (room_right < room_left ? -1 : ((fld_rand(f) & 1u) ? 1 : -1));
                b->pass_dir = dir;
                b->pass_ticks = (uint8_t)fld_clamp(p->pass_hold, 1, 255);
                lat = av->t.lateral + (int32_t)dir * p->avoid_offset;
            }
        }
        /* line-field-9: no blocking hold. A vehicle does not hold its target
         * for a faster one close behind; the source avoidance reads the
         * vehicle behind only for a side attack. */
        /* Side attack, on the chased vehicle only and only while alongside it. */
        if(b->aggression && chasing) {
            const sh2z_vehicle_t *c = &f->veh[b->chase];
            if(fld_active(c) && fld_abs(sh2z_track_near_gap(f->line, &c->t, &v->t)) < p->attack_long &&
               fld_abs(c->t.lateral - v->t.lateral) < p->attack_lat && (fld_rand(f) & 0xffu) < (uint32_t)b->aggression) {
                b->attack_ticks = 24u;
                b->attack_target = (uint8_t)b->chase;
                lat = c->t.lateral;
            }
        }
    }
    /* line-field-9: no hold against cutting across a vehicle alongside. The
     * target stands as chosen; contact settles any overlap. */
    b->target_lat = lat;
    /* line-field-5: tube roll decisions. */
    if(bh && fld_tube(p, at->flags) && at->half_width > 0) {
        int exit = 0;
        if(fld_match(at->flags, bh->exit_mask, bh->exit_value)) {
            int32_t d = v->t.dist + bh->roll_exit_ahead;
            while(d >= f->line->length) d -= f->line->length;
            const uint32_t n = sh2z_line_locate_far(f->line, d);
            exit = !fld_tube(p, f->line->nodes[n].flags);
        }
        /* A decision stands for 4 ticks; a level-of-detail step for its own ticks. */
        const int32_t per = (int32_t)(due >> 2) > 1 ? (int32_t)(due >> 2) : 1;
        if(exit) b->roll = 0;
        else if((int32_t)(fld_rand(f) & 0xffffu) < bh->roll_chance * per) {
            if((int32_t)(fld_rand(f) & 0xffffu) < bh->roll_latch) b->roll = -1;
            else {
                const int32_t span = bh->roll_span > 0 ? bh->roll_span : 1;
                b->roll = (int16_t)((bh->roll_min > 0 ? bh->roll_min : 1) + (int32_t)(((fld_rand(f) & 0xffffu) * (uint32_t)span) >> 16));
            }
        } else if(b->roll > 0) b->roll = (int16_t)(b->roll > per ? b->roll - per : 0);
    }
}

/* Energy charged to a vehicle, scaled by its armor; retirement at zero. */
FLD_INLINE void fld_damage(sh2z_field_t *f, uint32_t i, int32_t amount, const sh2z_field_behave_t *bh) {
    sh2z_vehicle_t *v = &f->veh[i];
    if(amount <= 0 || (v->flags & (SH2Z_VEH_RETIRED | SH2Z_VEH_FINISHED | SH2Z_VEH_FALLING))) return;
    v->energy -= (int32_t)(((int64_t)amount * v->armor) >> 8);
    if(v->energy <= 0 && bh && bh->ko_near > 0 && f->focus >= 0 && (uint32_t)f->focus < f->count && (int32_t)i != f->focus &&
       fld_abs(sh2z_track_near_gap(f->line, &v->t, &f->veh[f->focus].t)) > bh->ko_near) {
        v->energy = bh->ko_floor > 0 ? bh->ko_floor : 1;   /* line-field-5: far from the focus, energy cannot retire */
        return;
    }
    if(v->energy <= 0) {
        v->energy = 0;
        if(f->params->ranks) f->brain[i].pace_rank = 254u;
        v->flags |= SH2Z_VEH_RETIRED;
        v->flags &= (uint16_t)~SH2Z_VEH_BOOST;
        f->brain[i].retire_tick = (int32_t)(f->tick - f->start_tick);
        ++f->retired;
    }
}

FLD_INLINE int fld_current(const sh2z_field_t *f, uint32_t i) {
    return (f->veh[i].flags & SH2Z_VEH_EXTERNAL) || f->brain[i].last_tick == f->tick;
}

/* line-field-7: sh2z_follow_step's speed and distance alone, for an agent whose lateral the step integrates itself
 * (the follower's lateral would be overwritten), from the step's own sample of the same position. steps >= 1. */
FLD_INLINE void fld_follow_speed(const sh2z_line_t *line, const sh2z_follow_params_t *fp, sh2z_track_state_t *s,
                                 const sh2z_line_sample_t *at, int32_t target_speed, uint32_t steps) {
    if(steps > 64u) steps = 64u;
    const int32_t n = (int32_t)steps;
    int32_t want = target_speed < at->speed_cap ? target_speed : at->speed_cap;
    if(want < 0) want = 0;
    if(want > s->speed) {
        const int32_t up = s->speed + fp->accel * n;
        s->speed = up < want ? up : want;
    } else {
        const int32_t down = s->speed - fp->brake * n;
        s->speed = down > want ? down : want;
    }
    sh2z_track_advance(line, s, steps);
}

/* Pushed into the line's edge: scrub speed and charge energy. */
FLD_INLINE void fld_wall(sh2z_field_t *f, uint32_t i, const sh2z_field_behave_t *bh, int32_t into) {
    sh2z_vehicle_t *v = &f->veh[i];
    v->flags |= SH2Z_VEH_WALL;
    if(bh && bh->wall_impact > 0) {
        /* line-field-12: an impact costs by the speed it arrives at; resting on the edge costs nothing. */
        if(into <= 0) return;
        if(into > v->grip) into = v->grip;   /* no agent moves sideways faster than it steers: a push's overlap is not a speed */
        v->t.speed = fld_mul_q16(v->t.speed, f->params->wall_keep);
        fld_damage(f, i, fld_mul_q16(into, bh->wall_impact), bh);
        return;
    }
    v->t.speed = fld_mul_q16(v->t.speed, f->params->wall_keep);
    fld_damage(f, i, f->params->wall_energy, bh);
}

/* line-field-10: the contact passes of one step share three things, because a contact moves a vehicle sideways and
 * changes its speed and energy, never its distance or node. (1) The line sample under a vehicle (flags and half
 * width) is taken once a step, not once a pair and a push. (2) The pairs that lie within the contact reach along
 * the line, in scan order, are the same in every pass while no vehicle retires, so the passes after the first walk
 * the first pass's list in place of rescanning the order. (3) If a vehicle retires during a pass (the only change
 * to the scan's rules: fld_active), the next pass scans in full again. The result is the line-field-9 step's.
 *
 * line-field-13, the same results with less work again. (4) The rows of the order are read once into a packed
 * array (the distance of each, or -1 for a vehicle out of contact), and when no two neighbouring rows are in reach of
 * each other, no pair is (a pair in reach has every vehicle between them in reach of its neighbours), so the
 * passes do nothing. Otherwise the first pass lists the pairs from the packed rows, and the pairs are then looked at in
 * list order, as the scan met them. (5) A pair that does not overlap sideways changes nothing, and whether it
 * does depends only on the two laterals (distance, node and width stand still), which only a push changes. A pair
 * found not overlapping is skipped by the later passes until a push has moved one of its two vehicles, and a pair
 * that does not overlap is rejected before the push code is entered. (6) What the line sample gives the contact
 * phase, a tube's circumference and the half width, is taken without the sample's speed cap and lateral. */
#define FLD_PAIR_MAX 192u    /* list capacity; a longer list is not kept and the passes scan in full */
#define FLD_NO_STAMP 0xffffu /* a list entry that overlapped when last looked at (a move count sum stays far below this) */
#define FLD_VEH_MAX 256u     /* the field's vehicle limit (255) rounded up: per vehicle arrays need no bound test */

typedef struct {
    int32_t half_width[FLD_VEH_MAX];  /* of the line under the vehicle; -1: not taken (yet) */
    int32_t wrap[FLD_VEH_MAX];    /* circumference of the tube under the vehicle (0: not a tube, or no width); -1: not taken */
    uint16_t moved[FLD_VEH_MAX];  /* pushes that have involved the vehicle in this contact phase */
} fld_cs_t;

/* The half width of sh2z_line_sample (and so what the line sample under a vehicle gives the contact phase) at
 * (node, dist), without the sample's speed cap and lateral. */
FLD_INLINE int32_t fld_half_width(const sh2z_line_t *line, uint32_t node, int32_t dist) {
    const sh2z_line_node_t *a = &line->nodes[node];
    const sh2z_line_node_t *b = node + 1u < line->count ? a + 1 : &line->nodes[0];
    int32_t d = dist - a->start;
    if(d < 0) d = 0;
    int32_t fr = (int32_t)(((uint64_t)(uint32_t)d * a->inv_span) >> 16);
    if(fr > 65535) fr = 65535;
    const int32_t wa = (int32_t)a->half_width, wb = (int32_t)b->half_width;
    return wa * 256 + (int32_t)(((int64_t)(wb - wa) * fr) >> 8);
}

/* The tube under vehicle i (distance and node do not change during the passes): the circumference round it (0: the node
 * is not a tube, or has no width). Returns it; the half width is taken where a push needs it. */
FLD_NOINLINE int32_t fld_cs_fill(const sh2z_field_t *f, fld_cs_t *cs, uint32_t i) {
    const sh2z_vehicle_t *v = &f->veh[i];
    int32_t wrap = 0, hw = -1;
    if(fld_tube(f->params, f->line->nodes[v->t.node].flags)) {
        hw = fld_half_width(f->line, v->t.node, v->t.dist);
        if(hw > 0) wrap = fld_round(hw);
    }
    cs->half_width[i] = hw;
    cs->wrap[i] = wrap;
    return wrap;
}
/* The half width under vehicle i (a vehicle whose node is not a tube takes it here, when a push needs the line's edge). */
FLD_INLINE int32_t fld_cs_half_width(const sh2z_field_t *f, fld_cs_t *cs, uint32_t i) {
    if(cs->wrap[i] < 0) fld_cs_fill(f, cs, i);
    if(cs->half_width[i] < 0) cs->half_width[i] = fld_half_width(f->line, f->veh[i].t.node, f->veh[i].t.dist);
    return cs->half_width[i];
}

/* A pair in reach along the line (gap >= 0 by distance, a ahead of b) whose lateral offset dl (wrapped round a tube) is
 * inside the contact reach: push apart by the overlap. */
FLD_NOINLINE void fld_touch(sh2z_field_t *f, const sh2z_field_behave_t *bh, fld_cs_t *cs, uint32_t ia, uint32_t ib,
                            int32_t gap, int32_t dl) {
    const sh2z_field_params_t *p = f->params;
    sh2z_vehicle_t *a = &f->veh[ia], *b = &f->veh[ib];   /* a ahead of b */
    const int32_t adl = fld_abs(dl);
    const int32_t overlap = p->contact_lat - adl;
    const int32_t dir = dl >= 0 ? 1 : -1;
    const int a_ext = (a->flags & SH2Z_VEH_EXTERNAL) != 0, b_ext = (b->flags & SH2Z_VEH_EXTERNAL) != 0;
    const int32_t half = (overlap + 1) >> 1;
    /* Push apart by the overlap. Each takes half; what one cannot take
     * (the line's edge, or an external vehicle's own dynamics) goes to
     * the other. An external vehicle receives its share as a kick. */
    int32_t move_a = half, move_b = overlap - half;
    for(int pass = 0; pass < 2; ++pass) {
        int32_t left_a = 0, left_b = 0;
        if(move_a) {
            if(a_ext) a->kick_lat += dir * move_a;
            else {
                if(cs->wrap[ia] < 0) fld_cs_fill(f, cs, ia);
                const int32_t nl = a->t.lateral + dir * move_a, wr = cs->wrap[ia];
                if(wr) a->t.lateral = fld_wrap(nl, wr);
                else {
                    const int32_t hw = fld_cs_half_width(f, cs, ia);
                    a->t.lateral = fld_clamp(nl, -hw, hw);
                    left_a = fld_abs(nl - a->t.lateral);
                    if(left_a && !pass) fld_wall(f, ia, bh, left_a);
                }
            }
        }
        if(move_b) {
            if(b_ext) b->kick_lat -= dir * move_b;
            else {
                if(cs->wrap[ib] < 0) fld_cs_fill(f, cs, ib);
                const int32_t nl = b->t.lateral - dir * move_b, wr = cs->wrap[ib];
                if(wr) b->t.lateral = fld_wrap(nl, wr);
                else {
                    const int32_t hw = fld_cs_half_width(f, cs, ib);
                    b->t.lateral = fld_clamp(nl, -hw, hw);
                    left_b = fld_abs(nl - b->t.lateral);
                    if(left_b && !pass) fld_wall(f, ib, bh, left_b);
                }
            }
        }
        if(!left_a && !left_b) break;
        move_a = left_b; move_b = left_a;   /* hand each remainder across, once */
    }
    /* Bump: the trailing vehicle keeps only part of its speed advantage. */
    const int32_t closing = b->t.speed - a->t.speed;
    if(closing > 0) {
        const int32_t keep = a->t.speed + fld_mul_q16(closing, p->bump_keep);
        if(b_ext) b->kick_speed += keep - b->t.speed;
        else b->t.speed = keep;
    }
    /* Energy: both pay for the hit; an attacker's victim pays extra. */
    /* Energy is charged per incident (the impact), not while both keep touching. */
    sh2z_field_brain_t *ba = &f->brain[ia], *bb = &f->brain[ib];
    const int fresh = !(ba->touch_cool && bb->touch_cool);
    a->flags |= SH2Z_VEH_TOUCH; b->flags |= SH2Z_VEH_TOUCH;
    ba->touch_cool = bb->touch_cool = FLD_TOUCH_COOL;
    /* line-field-9: no post-contact room hold. The contact is a push and
     * an impact; neither vehicle is set to hold a side afterwards. */
    if(fresh && bh) {   /* line-field-5: a contact incident with an external vehicle starts a knock-out age */
        if(a_ext && !b_ext) { bb->ko_by = (int16_t)ia; bb->ko_age = 1u; }
        if(b_ext && !a_ext) { ba->ko_by = (int16_t)ib; ba->ko_age = 1u; }
    }
    const int32_t hit = fresh ? p->side_energy + (closing > 0 ? fld_mul_q16(closing, p->hit_energy) : 0) : 0;
    const int a_attacks = !a_ext && ba->attack_ticks && ba->attack_target == ib;
    const int b_attacks = !b_ext && bb->attack_ticks && bb->attack_target == ia;
    if(fresh && bh && bh->contact_norm > 0) {
        /* line-field-5: the impact is the closing speed along the line between the two (longitudinal gap and
         * lateral offset; the length by max + min / 2), shared: contact_norm of it each, and the whole
         * 2 x contact_norm to an attacker's victim (the attacker pays nothing). */
        const int32_t ax = fld_abs(gap), ay = adl, len = (ax > ay ? ax + (ay >> 1) : ay + (ax >> 1));
        const int64_t dot = (int64_t)closing * gap + (int64_t)(b->t.lat_vel - a->t.lat_vel) * dl;
        const int32_t norm = len > 0 ? (int32_t)((dot < 0 ? -dot : dot) / len) : fld_abs(closing);
        const int32_t share = fld_mul_q16(norm, bh->contact_norm);
        fld_damage(f, ia, a_attacks ? 0 : (b_attacks ? share * 2 : share), bh);
        fld_damage(f, ib, b_attacks ? 0 : (a_attacks ? share * 2 : share), bh);
    } else if(fresh) {
        fld_damage(f, ia, a_attacks ? 0 : hit + (b_attacks ? p->attack_energy : 0), bh);
        fld_damage(f, ib, b_attacks ? 0 : hit + (a_attacks ? p->attack_energy : 0), bh);
    }
    /* An attack that lands is over. */
    if(a_attacks) ba->attack_ticks = 0u;
    if(b_attacks) bb->attack_ticks = 0u;
    f->contacts += (uint32_t)fresh;
}

/* One pair in reach along the line (gap >= 0, a ahead of b). Returns 1 when the pair overlaps sideways and was pushed,
 * else 0 with nothing changed. line-field-4: across a tube's seam the lateral offset wraps round the tube. */
FLD_INLINE uint32_t fld_pair(sh2z_field_t *f, const sh2z_field_behave_t *bh, fld_cs_t *cs, int32_t reach_lat,
                             uint32_t ia, uint32_t ib, int32_t gap) {
    int32_t dl = f->veh[ia].t.lateral - f->veh[ib].t.lateral;
    int32_t wr = cs->wrap[ia];
    if(wr < 0) wr = fld_cs_fill(f, cs, ia);
    if(wr) dl = fld_wrap(dl, wr);
    if(fld_abs(dl) >= reach_lat) return 0u;
    fld_touch(f, bh, cs, ia, ib, gap, dl);
    ++cs->moved[ia]; ++cs->moved[ib];
    return 1u;
}

/* What a scan reads of each row of the order, packed: the distance of the vehicle in the row, or -1 when it takes no
 * part (not active, or not current this tick: only between vehicles that are both current this tick, since a skipped
 * vehicle sits behind its true place and must not end the scan). Distances do not change in the contact phase; whether
 * a vehicle takes part changes only when one retires (ret is the retired count the rows were read at). */
typedef struct {
    uint32_t ret;
    uint32_t any;   /* some two neighbouring rows that take part are in reach of each other along the line */
    int32_t d[FLD_VEH_MAX];
} fld_rows_t;

static void fld_rows_read(const sh2z_field_t *f, fld_rows_t *r) {
    const int32_t length = f->line->length, half = length >> 1, reach = f->params->contact_long;
    int32_t first = -1, prev = -1;
    uint32_t any = 0u;
    r->ret = f->retired;
    for(uint32_t s = 0u; s < f->count; ++s) {
        const uint32_t i = f->order[s];
        const int32_t d = (fld_active(&f->veh[i]) && fld_current(f, i)) ? f->veh[i].t.dist : -1;
        r->d[s] = d;
        if(d < 0) continue;
        if(prev >= 0) {   /* sh2z_track_near_gap of the neighbours, in the scan's own terms */
            int32_t gap = prev - d;
            if(gap > half) gap -= length;
            else if(gap <= -half) gap += length;
            any |= (uint32_t)(gap < reach && gap > -reach);
        } else first = d;
        prev = d;
    }
    if(prev >= 0 && prev != first) {   /* the last row and the first, across the line's start */
        int32_t gap = prev - first;
        if(gap > half) gap -= length;
        else if(gap <= -half) gap += length;
        any |= (uint32_t)(gap < reach && gap > -reach);
    }
    r->any = any;
}

/* The pairs of one full scan that lie within the contact reach along the line, in scan order: the vehicle ahead and
 * the vehicle behind, the gap, the scan position (row s, column k) each was found at, and the move counts of its two
 * vehicles when it was last looked at and found not overlapping (FLD_NO_STAMP: it overlapped). */
typedef struct {
    int32_t gap[FLD_PAIR_MAX];
    uint16_t stamp[FLD_PAIR_MAX];
    uint8_t ia[FLD_PAIR_MAX], ib[FLD_PAIR_MAX], s[FLD_PAIR_MAX], k[FLD_PAIR_MAX];
    uint32_t n;
} fld_pl_t;

/* The pairs a full scan of the packed rows meets, in scan order (nothing is changed). Returns 0, or 1 when there
 * are more than FLD_PAIR_MAX of them (then the list is not kept). */
static uint32_t fld_list(const sh2z_field_t *f, fld_pl_t *pl, const fld_rows_t *r) {
    const int32_t length = f->line->length, half = length >> 1, reach = f->params->contact_long;
    const uint32_t count = f->count;
    const uint8_t *order = f->order;
    uint32_t n = 0u;
    for(uint32_t s = 0u; s < count; ++s) {
        const int32_t d0 = r->d[s];
        if(d0 < 0) continue;
        uint32_t row = s;
        for(uint32_t k = 1u; k < count; ++k) {
            if(++row == count) row = 0u;
            const int32_t d1 = r->d[row];
            if(d1 < 0) continue;
            /* sh2z_track_near_gap */
            int32_t gap = d0 - d1;
            if(gap > half) gap -= length;
            else if(gap <= -half) gap += length;
            if(gap >= reach) break;
            /* The order is by distance along the line: where it wraps past the line's start, the vehicle later in
             * the order can be the one ahead, so the gap's sign decides. */
            if(gap < 0) {
                if(gap <= -reach) continue;
                if(n == FLD_PAIR_MAX) return 1u;
                pl->ia[n] = order[row]; pl->ib[n] = order[s]; pl->gap[n] = -gap;
            } else {
                if(n == FLD_PAIR_MAX) return 1u;
                pl->ia[n] = order[s]; pl->ib[n] = order[row]; pl->gap[n] = gap;
            }
            pl->s[n] = (uint8_t)s; pl->k[n] = (uint8_t)k;
            ++n;
        }
    }
    pl->n = n;
    return 0u;
}

/* A scan of the order from row s0, column k0 that looks at each pair as it meets it (a pass cut short by a retirement
 * resumes at the next column of its row, whose first vehicle was checked when the row began; a pile too long for the
 * list is scanned this way in full). Returns the pairs pushed. */
static uint32_t fld_scan(sh2z_field_t *f, const sh2z_field_behave_t *bh, fld_cs_t *cs, fld_rows_t *r, uint32_t s0, uint32_t k0) {
    const sh2z_field_params_t *p = f->params;
    const int32_t length = f->line->length, half = length >> 1, reach = p->contact_long, reach_lat = p->contact_lat;
    const uint32_t count = f->count;
    uint32_t pushed = 0u;
    if(r->ret != f->retired) fld_rows_read(f, r);
    for(uint32_t s = s0; s < count; ++s) {
        const uint32_t kfirst = s == s0 ? k0 : 1u;
        if(kfirst == 1u && r->d[s] < 0) continue;
        const uint32_t i0 = f->order[s];
        const int32_t d0 = f->veh[i0].t.dist;   /* the row's own distance: a vehicle that retired in its row still pairs on */
        for(uint32_t k = kfirst; k < count; ++k) {
            const uint32_t row = s + k < count ? s + k : s + k - count;
            const int32_t d1 = r->d[row];
            if(d1 < 0) continue;
            int32_t gap = d0 - d1;   /* sh2z_track_near_gap */
            if(gap > half) gap -= length;
            else if(gap <= -half) gap += length;
            if(gap >= reach) break;
            if(gap < 0 && gap <= -reach) continue;
            const uint32_t i1 = f->order[row];
            pushed += gap < 0 ? fld_pair(f, bh, cs, reach_lat, i1, i0, -gap) : fld_pair(f, bh, cs, reach_lat, i0, i1, gap);
            if(r->ret != f->retired) fld_rows_read(f, r);   /* a retirement changes who takes part in the rest of the scan */
        }
    }
    return pushed;
}

/* The listed pairs, in list order: each is looked at unless it did not overlap when last looked at and neither of its
 * vehicles has been pushed since. If a vehicle retires, the scan resumes in full from the pair it was at. */
static uint32_t fld_walk(sh2z_field_t *f, const sh2z_field_behave_t *bh, fld_cs_t *cs, fld_pl_t *pl, fld_rows_t *r,
                         uint32_t ret_list) {
    const int32_t reach_lat = f->params->contact_lat;
    uint32_t pushed = 0u;
    for(uint32_t n = 0u; n < pl->n; ++n) {
        const uint32_t ia = pl->ia[n], ib = pl->ib[n];
        if(pl->stamp[n] == (uint16_t)(cs->moved[ia] + cs->moved[ib])) continue;   /* not pushed since it did not overlap */
        const uint32_t hit = fld_pair(f, bh, cs, reach_lat, ia, ib, pl->gap[n]);
        pushed += hit;
        pl->stamp[n] = hit ? FLD_NO_STAMP : (uint16_t)(cs->moved[ia] + cs->moved[ib]);
        if(f->retired != ret_list) { pushed += fld_scan(f, bh, cs, r, pl->s[n], (uint32_t)pl->k[n] + 1u); break; }
    }
    return pushed;
}

/* The contact passes of a step: at most three, ending after the first that pushes no more than one pair
 * (pile-ups of three or more settle over the passes; energy is per incident, so nothing is charged twice). */
FLD_INLINE void fld_contact_phase(sh2z_field_t *f, const sh2z_field_behave_t *bh) {
    const sh2z_field_params_t *p = f->params;
    /* No reach across: no pair can overlap (|dl| >= 0 >= contact_lat), so the walk would change nothing. */
    if(p->contact_lat <= 0) return;
    fld_cs_t cs;
    fld_pl_t pl;
    fld_rows_t rows;
    fld_rows_read(f, &rows);
    if(!rows.any) return;   /* no two neighbours in reach: no pair in reach */
    for(uint32_t i = 0u; i < f->count; ++i) { cs.wrap[i] = -1; cs.moved[i] = 0u; }
    uint32_t list_ok = 0u, ret_list = 0u, full = 0u;
    for(int pass = 0; pass < 3; ++pass) {
        uint32_t pushed;
        if(list_ok && f->retired == ret_list) {
            /* Nobody has retired since the list was scanned, so the scan would meet these pairs and no others. A
             * retirement during this pass changes which vehicles the rest of the scan meets: it resumes in full. */
            pushed = fld_walk(f, bh, &cs, &pl, &rows, ret_list);
        } else {
            ret_list = f->retired;
            if(!full) full = fld_list(f, &pl, &rows);
            if(full) { pushed = fld_scan(f, bh, &cs, &rows, 0u, 1u); list_ok = 0u; }
            else {
                for(uint32_t n = 0u; n < pl.n; ++n) pl.stamp[n] = FLD_NO_STAMP;
                pushed = fld_walk(f, bh, &cs, &pl, &rows, ret_list);
                list_ok = 1u;
            }
        }
        if(pushed <= 1u) break;
    }
}

static void fld_laps(sh2z_field_t *f, uint32_t i) {
    sh2z_vehicle_t *v = &f->veh[i];
    sh2z_field_brain_t *b = &f->brain[i];
    const int32_t lap = v->t.lap;
    while((int32_t)b->laps_seen < lap) {
        b->laps_seen = (int16_t)(b->laps_seen + 1);
        const int32_t done = (int32_t)b->laps_seen;          /* laps completed */
        if(done >= 1 && done <= 4) b->lap_tick[done - 1] = (int32_t)(f->tick - f->start_tick);
        if(f->params->ranks && b->pace_rank >= 1u && b->pace_rank < 254u && done >= 1 && done < (int32_t)f->params->laps) {
            fld_retarget(f, i);   /* line-field-12 */
        } else if(done == 1 && b->finish_ticks > 0 && f->params->laps > 1u && !(v->flags & (SH2Z_VEH_EXTERNAL | SH2Z_VEH_RETIRED))) {
            /* line-field-2: cover the remaining laps evenly by the finish target, expecting one lap now. */
            const int32_t left = b->finish_ticks - b->lap_tick[0];
            if(left > 0) {
                b->pace = ((int32_t)f->params->laps - 1) * f->line->length / left;
                b->expect = f->line->length;
            }
        }
        if(done == (int32_t)f->params->laps && !(v->flags & (SH2Z_VEH_FINISHED | SH2Z_VEH_RETIRED))) {
            v->flags |= SH2Z_VEH_FINISHED;
            v->flags &= (uint16_t)~SH2Z_VEH_BOOST;
            b->finish_tick = (int32_t)(f->tick - f->start_tick);
            b->finish_seq = (uint8_t)f->finished;
            ++f->finished;
        }
    }
    if(lap < (int32_t)b->laps_seen) b->laps_seen = (int16_t)lap;   /* an external vehicle driven backwards */
}

/* Physical order over the vehicle array (largest dist first, ties by index) and each vehicle's slot in it. The
 * order is a strict total order of the positions, so the result does not depend on the order it starts from. */
FLD_INLINE void fld_order(sh2z_field_t *f) {
    for(uint32_t i = 1u; i < f->count; ++i) {
        const uint8_t k = f->order[i];
        const int32_t kd = f->veh[k].t.dist;
        uint32_t j = i;
        while(j > 0u) {
            const uint8_t q = f->order[j - 1u];
            const int32_t qd = f->veh[q].t.dist;
            if(qd > kd || (qd == kd && q < k)) break;
            f->order[j] = q;
            --j;
        }
        f->order[j] = k;
    }
    for(uint32_t s = 0u; s < f->count; ++s) f->slot[f->order[s]] = (uint8_t)s;
}

/* The step, with the behaviour block as a parameter (line-field-7). Each entry point below inlines it once: with
 * bh a constant NULL every line-field-5 branch folds away, so the NULL step is line-field-4's code, and the
 * behaviour step carries none of the NULL tests. */
/* line-field-14: an agent in the air on a run too long for its reach leaves the line. The run is the contiguous nodes that
 * match the air mask, from the agent's own back to its start (at most FLD_AIR_SCAN nodes, never across the line's start);
 * the agent must be on one. Reach is speed * gap_reach_ticks in the line's own units (Q8, speed a tick). The agent leaves once it has
 * covered the reach from the run's start. It is then still inside the run, so the run is longer than the reach; a run exactly
 * as long as the reach is cleared, the agent is past its end by the time it has covered it. Stateless. */
#define FLD_AIR_SCAN 64u
FLD_INLINE int fld_air_node(const sh2z_field_behave_t *bh, uint16_t flags) { return fld_match(flags, bh->air_mask, bh->air_value); }
FLD_INLINE int fld_gap_short(const sh2z_line_t *line, const sh2z_field_behave_t *bh, const sh2z_vehicle_t *v) {
    if(bh->gap_reach_ticks <= 0 || v->t.speed <= 0 || !fld_air_node(bh, line->nodes[v->t.node].flags)) return 0;
    uint32_t a = v->t.node;
    for(uint32_t k = 0u; a > 0u && k < FLD_AIR_SCAN && fld_air_node(bh, line->nodes[a - 1u].flags); ++k) --a;
    return (int64_t)(v->t.dist - line->nodes[a].start) >= (int64_t)v->t.speed * bh->gap_reach_ticks;
}

FLD_INLINE void fld_step(sh2z_field_t *f, const sh2z_field_behave_t *bh) {
    const sh2z_field_params_t *p = f->params;
    const sh2z_line_t *line = f->line;
    ++f->tick;
    fld_order(f);
    if(!f->started) return;

    const sh2z_vehicle_t *focus = (f->focus >= 0 && (uint32_t)f->focus < f->count) ? &f->veh[f->focus] : (const sh2z_vehicle_t *)0;
    for(uint32_t i = 0u; i < f->count; ++i) {
        sh2z_vehicle_t *v = &f->veh[i];
        sh2z_field_brain_t *b = &f->brain[i];
        /* line-field-7: the lap bookkeeping only when the lap moved (fld_laps does nothing otherwise). */
        if(v->flags & SH2Z_VEH_EXTERNAL) { if(v->t.lap != (int32_t)b->laps_seen) fld_laps(f, i); continue; }
        if(v->flags & SH2Z_VEH_STOPPED) continue;
        if(bh && (v->flags & SH2Z_VEH_FALLING)) {   /* line-field-5: off the line, every tick, until it retires */
            b->last_tick = f->tick;
            v->t.speed = fld_mul_q16(v->t.speed, 61604);   /* 0.94 a tick */
            v->t.lateral += v->t.lat_vel;
            if((int32_t)(f->tick - f->start_tick) - b->fall_tick >= (bh->fall_ticks > 0 ? bh->fall_ticks : 1)) {
                v->energy = 0;
                v->flags = (uint16_t)((v->flags | SH2Z_VEH_RETIRED | SH2Z_VEH_STOPPED) & ~(SH2Z_VEH_BOOST | SH2Z_VEH_FALLING));
                v->t.speed = 0;
                b->retire_tick = (int32_t)(f->tick - f->start_tick);
                if(p->ranks) b->pace_rank = 254u;
                ++f->retired;
            }
            ++f->steps;
            continue;
        }
        const uint32_t period = focus ? sh2z_lod_period(sh2z_track_near_gap(line, &v->t, &focus->t), p->lod_near) : 1u;
        /* Vehicles on the same stretch of the line (512 units) share a phase, so
         * neighbours stay in step with each other; stretches are staggered. */
        uint32_t due = f->tick - b->last_tick;
        if(period > 1u && ((f->tick + ((uint32_t)v->t.dist >> 17)) & (period - 1u)) != 0u && due < period * 2u) due = 0u;
        if(!due) continue;
        b->last_tick = f->tick;
        b->expect += b->pace * (int32_t)due;
        if(bh && !(v->flags & (SH2Z_VEH_RETIRED | SH2Z_VEH_FINISHED))) {   /* line-field-5: knock-out age */
            const uint32_t age = (uint32_t)b->ko_age + due;
            b->ko_age = (uint16_t)(age > 0xffffu ? 0xffffu : age);
        }
        sh2z_line_sample_t at;
        sh2z_line_sample(line, v->t.node, v->t.dist, &at);
        /* line-field-4: the zone under the agent as its step starts. */
        const sh2z_field_surface_t *sf = p->surface;
        const uint32_t zone = (sf && sf->zone_count) ? fld_zone(sf, &v->t) : 0u;
        if(zone && !(v->flags & (SH2Z_VEH_RETIRED | SH2Z_VEH_FINISHED))) {
            if(zone == SH2Z_FIELD_ZONE_BOOST) {
                v->flags |= SH2Z_VEH_BOOST;
                if(v->boost_ticks < sf->plate_ticks) v->boost_ticks = (int16_t)fld_clamp(sf->plate_ticks, 0, INT16_MAX);
            } else if(zone == SH2Z_FIELD_ZONE_RECHARGE) {
                v->energy += sf->recharge * (int32_t)due;
                if(v->energy > v->max_energy) v->energy = v->max_energy;
            }
        }
        /* Decisions every 4th tick per vehicle (staggered), and on every level-of-detail step. */
        if(due > 1u || ((i + f->tick) & 3u) == 0u || (v->flags & (SH2Z_VEH_RETIRED | SH2Z_VEH_WALL))) fld_think(f, i, due > 1u ? due : 4u, &at, bh);
        sh2z_follow_params_t fp;
        fp.accel = v->accel; fp.brake = (v->flags & SH2Z_VEH_RETIRED) ? v->brake * 2 : v->brake;
        fp.steer_gain = p->steer_gain; fp.steer_max = v->grip; fp.margin = b->attack_ticks ? 0 : p->margin;
        int32_t target = b->target_speed;
        const int32_t top = v->top_speed + ((v->flags & SH2Z_VEH_BOOST) ? p->boost_speed : 0);
        if(target > top) target = top;
        if(zone == SH2Z_FIELD_ZONE_SLIP) fp.steer_gain = fld_mul_q16(fp.steer_gain, sf->slip_steer);
        else if(zone == SH2Z_FIELD_ZONE_ROUGH && !(v->flags & SH2Z_VEH_BOOST)) target = fld_mul_q16(target, sf->rough_keep);
        if(p->ranks && !(v->flags & (SH2Z_VEH_RETIRED | SH2Z_VEH_FINISHED | SH2Z_VEH_EXTERNAL))) {
            /* line-field-12: a racer whose target holds it below its speed only coasts (the throttle is off), and no target
             * is below the floor. A road cap that holds it is a braking limit, as before. */
            const sh2z_field_ranks_t *rk = p->ranks;
            if(rk->floor > 0 && target < rk->floor) target = rk->floor;
            if(rk->coast > 0 && target < v->t.speed && target < at.speed_cap) {
                const int32_t slow = fld_mul_q16(v->t.speed, rk->coast);
                fp.brake = slow > 1 ? slow : 1;
            }
        }
        /* line-field-4: in a tube there is no edge, and the short way round is taken. */
        const int tube = fld_tube(p, at.flags) && at.half_width > 0;
        int32_t target_lat = b->target_lat;
        if(tube) {
            target_lat = v->t.lateral + fld_wrap(target_lat - v->t.lateral, fld_round(at.half_width));
            fp.margin = -(INT32_C(1) << 28);
        }
        v->flags &= (uint16_t)~SH2Z_VEH_WALL;
        /* line-field-5: roll and hold round a tube, the air, lateral inertia. */
        int air = 0, own_lateral = 0;
        if(bh && !(v->flags & (SH2Z_VEH_RETIRED | SH2Z_VEH_FINISHED))) {
            if(tube && !b->attack_ticks && !b->pass_ticks) {
                const int hold = fld_match(at.flags, bh->hold_mask, bh->hold_value);
                if(b->roll && (hold || !bh->roll_well)) target_lat = v->t.lateral + fld_mul_q16(v->t.speed, bh->roll_lead);
                else if(b->roll) {   /* a tube with a pull back to its line: the roll settles roll_well of the radius round */
                    const int32_t off = fld_mul_q16(at.half_width, bh->roll_well);
                    target_lat = v->t.lateral + fld_wrap(target_lat + (bh->roll_lead < 0 ? -off : off) - v->t.lateral, fld_round(at.half_width));
                }
                else if(hold) target_lat = v->t.lateral + fld_mul_q16(v->t.speed, bh->hold_lead);
            }
            air = fld_match(at.flags, bh->air_mask, bh->air_value) && !tube;
            if(air && !(v->flags & SH2Z_VEH_AIR)) {
                const int32_t spread = bh->air_spread;
                b->air_drift = spread > 0 ? (int32_t)(((int64_t)(fld_rand(f) & 0xffffu) * (2 * spread + 1)) >> 16) - spread : 0;
            }
            v->flags = (uint16_t)(air ? (v->flags | SH2Z_VEH_AIR) : (v->flags & ~SH2Z_VEH_AIR));
            own_lateral = air || bh->lat_accel > 0;
        }
        if(own_lateral) {
            /* The follower moves speed and distance only; the lateral is integrated here. */
            const int32_t lat0 = v->t.lateral, vel0 = v->t.lat_vel, n = (int32_t)(due < 64u ? due : 64u);
            if(!tube) {
                const int32_t lim = at.half_width - p->margin > 0 ? at.half_width - p->margin : 0;
                target_lat = fld_clamp(target_lat, -lim, lim);
            }
            int32_t vel = vel0;
            if(!air) {
                int32_t want = fld_clamp(fld_mul_q16(target_lat - lat0, fp.steer_gain), -fp.steer_max, fp.steer_max);
                vel += fld_clamp(want - vel0, -bh->lat_accel * n, bh->lat_accel * n);
            }
            fld_follow_speed(line, &fp, &v->t, &at, target, due);
            v->t.lateral = lat0 + vel * n + (air ? fld_mul_q16(v->t.speed, b->air_drift) * n : 0);
            v->t.lat_vel = vel;
        } else sh2z_follow_step(line, &fp, &v->t, target, target_lat, due);
        /* On the edge: scrub speed and charge energy. */
        {   /* The line may narrow under the vehicle: stay inside at the new position. */
            sh2z_line_sample_t now;
            sh2z_line_sample(line, v->t.node, v->t.dist, &now);
            const int now_tube = fld_tube(p, now.flags) && now.half_width > 0;
            /* line-field-5: in the air beyond an open edge, the agent leaves the line. On a drop node it also leaves
             * the line when it comes out of a tube beyond the open edge (launched, launch_chance), or, on a drop tube, when it is
             * round beyond drop_lat of the radius and loses its hold (drop_chance per step). */
            int leaves = 0;
            if(bh && !(v->flags & (SH2Z_VEH_RETIRED | SH2Z_VEH_FINISHED))) {
                const int open = !now_tube && fld_match(now.flags, bh->open_mask, bh->open_value) &&
                                 fld_abs(v->t.lateral) > now.half_width + bh->fall_margin;
                if(air) leaves = open || fld_gap_short(line, bh, v);
                else if(fld_match(now.flags, bh->drop_mask, bh->drop_value)) {
                    if(tube && !now_tube) leaves = open && (bh->launch_chance >= 65536 || (int32_t)(fld_rand(f) & 0xffffu) < bh->launch_chance);
                    else if(now_tube && bh->drop_chance > 0 && fld_match(now.flags, bh->hold_mask, bh->hold_value) &&
                            fld_abs(fld_wrap(v->t.lateral, fld_round(now.half_width))) > fld_mul_q16(now.half_width, bh->drop_lat))
                        leaves = (int32_t)(fld_rand(f) & 0xffffu) < bh->drop_chance;
                }
            }
            if(leaves) {
                v->flags = (uint16_t)((v->flags | SH2Z_VEH_FALLING) & ~(SH2Z_VEH_BOOST | SH2Z_VEH_AIR));
                b->fall_tick = (int32_t)(f->tick - f->start_tick);
                ++f->falls;
                if(bh->ko_window > 0 && b->ko_by >= 0 && (uint32_t)b->ko_by < f->count && (int32_t)b->ko_age <= bh->ko_window &&
                   !(f->veh[b->ko_by].flags & (SH2Z_VEH_FINISHED | SH2Z_VEH_RETIRED))) {
                    b->ko_credit = b->ko_by;
                    ++f->kos;
                }
                ++f->steps;
                continue;
            }
            if(now_tube) v->t.lateral = fld_wrap(v->t.lateral, fld_round(now.half_width));
            else if(tube) v->t.lateral = fld_clamp(v->t.lateral, -now.half_width, now.half_width);   /* out of a tube */
            else if(air) { }   /* line-field-5: in the air, above any wall; it lands (or falls) later */
            else if(v->t.lateral > now.half_width) { const int32_t into = v->t.lateral - now.half_width; v->t.lateral = now.half_width; fld_wall(f, i, bh, into); if(own_lateral) v->t.lat_vel = -(v->t.lat_vel >> 1); }
            else if(v->t.lateral < -now.half_width) { const int32_t into = -now.half_width - v->t.lateral; v->t.lateral = -now.half_width; fld_wall(f, i, bh, into); if(own_lateral) v->t.lat_vel = -(v->t.lat_vel >> 1); }
            else if(fld_abs(v->t.lateral) >= at.half_width - 1 && at.half_width > 0) fld_wall(f, i, bh, 0);
        }
        if((v->flags & SH2Z_VEH_RETIRED) && v->t.speed == 0) v->flags |= SH2Z_VEH_STOPPED;
        if(v->t.lap != (int32_t)b->laps_seen) fld_laps(f, i);
        ++f->steps;
    }
    /* Touch history: TOUCH is this tick only; the cooldown spans an incident. */
    for(uint32_t i = 0u; i < f->count; ++i) {
        f->veh[i].flags &= (uint16_t)~SH2Z_VEH_TOUCH;
        if(f->brain[i].touch_cool) --f->brain[i].touch_cool;
    }
    /* line-field-8: the order again, from this step's positions. A contact scan ends at the first vehicle out of
     * reach, so a vehicle that passed another during the step must not be left behind it in the order. */
    fld_order(f);
    /* Further passes settle pile-ups of three or more (energy is per incident,
     * so nothing is charged twice). At most three passes. */
    fld_contact_phase(f, bh);
    f->rank_changes += fld_rank(f);
}

void sh2z_field_step_plain(sh2z_field_t *f) {
    fld_step(f, (const sh2z_field_behave_t *)0);
}

void sh2z_field_step_behave(sh2z_field_t *f) {
    fld_step(f, f->params->behave);
}

void sh2z_field_step(sh2z_field_t *f) {
    if(f->params->behave) sh2z_field_step_behave(f);
    else sh2z_field_step_plain(f);
}

FLD_INLINE uint32_t fld_mix(uint32_t h, uint32_t x) { return (h ^ x) * UINT32_C(16777619); }
#define FLD_MIX(x) (h = fld_mix(h, (uint32_t)(x)))

uint32_t sh2z_field_hash(const sh2z_field_t *f) {
    uint32_t h = UINT32_C(2166136261);
    FLD_MIX(f->tick); FLD_MIX(f->rng); FLD_MIX(f->finished); FLD_MIX(f->retired); FLD_MIX(f->contacts); FLD_MIX(f->rank_changes);
    for(uint32_t i = 0u; i < f->count; ++i) {
        const sh2z_vehicle_t *v = &f->veh[i];
        const sh2z_field_brain_t *b = &f->brain[i];
        FLD_MIX(v->t.dist); FLD_MIX(v->t.lateral); FLD_MIX(v->t.speed); FLD_MIX(v->t.lat_vel); FLD_MIX(v->t.lap);
        FLD_MIX(v->energy); FLD_MIX(v->flags); FLD_MIX(v->boost_ticks); FLD_MIX(v->rank); FLD_MIX(v->kick_lat); FLD_MIX(v->kick_speed);
        FLD_MIX(b->expect); FLD_MIX(b->target_lat); FLD_MIX(b->target_speed); FLD_MIX(b->finish_tick); FLD_MIX(b->retire_tick);
        FLD_MIX(b->pass_ticks); FLD_MIX(b->attack_ticks); FLD_MIX(f->rank_order[i]);
        if(b->finish_ticks) FLD_MIX(b->finish_ticks); /* line-field-1 states hash as before */
        if(b->first_lap_cap) FLD_MIX(b->first_lap_cap);
        if(f->params->ranks) FLD_MIX(b->pace_rank);
        if(f->params->behave) { FLD_MIX(b->fall_tick); FLD_MIX(b->air_drift); FLD_MIX(b->roll); FLD_MIX(b->ko_by); FLD_MIX(b->ko_credit); FLD_MIX(b->ko_age); }
    }
    if(f->params->behave) { FLD_MIX(f->falls); FLD_MIX(f->kos); }
    return h;
}
