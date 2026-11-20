/* Experimental line field (contract tags line-field-1 to line-field-5).
 *
 * Copyright (c) 2026 SH2ZAM Contributors. MIT License.
 * Inspired by SH4ZAM by Falco Girgis and SH4ZAM Contributors.
 */
#include "sh2zam/line_field_experimental.h"

#if defined(__GNUC__)
#define FLD_INLINE static inline __attribute__((always_inline))
#else
#define FLD_INLINE static inline
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
    for(uint32_t i = 1u; i < f->count; ++i) {
        const uint8_t k = o[i];
        const int32_t kk = fld_rank_key(f, k);
        uint32_t j = i;
        while(j > 0u) {
            const uint8_t p = o[j - 1u];
            const int32_t pk = fld_rank_key(f, p);
            /* Between running vehicles a place changes hands only on a clear lead
             * (no flicker side by side); finishing order is exact. */
            if((f->veh[p].flags | f->veh[k].flags) & SH2Z_VEH_FINISHED) {
                if(pk > kk || (pk == kk && p < k)) break;
            } else if((int64_t)pk + f->params->rank_lead >= kk) break;
            o[j] = p;
            --j;
        }
        if(j != i) { o[j] = k; moved = 1u; }
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
static int32_t fld_behind(const sh2z_field_t *f, uint32_t i) {
    uint32_t s = f->slot[i];
    for(uint32_t k = 1u; k < f->count; ++k) {
        s = s + 1u < f->count ? s + 1u : 0u;
        const uint32_t j = f->order[s];
        if(fld_active(&f->veh[j])) return (int32_t)j;
    }
    return -1;
}

static void fld_think(sh2z_field_t *f, uint32_t i, uint32_t due, const sh2z_line_sample_t *at) {
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
    const int chasing = b->chase >= 0 && (uint32_t)b->chase < f->count &&
                        !(f->veh[b->chase].flags & (SH2Z_VEH_RETIRED | SH2Z_VEH_FINISHED));
    if(!chasing && b->chase >= 0) {
        /* The chased vehicle is gone: carry on at our own pace from here. */
        b->chase = -1;
        b->expect = total;
        behind = 0;
    }
    if(dash) {
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
    if(!(v->flags & SH2Z_VEH_BOOST) && v->t.lap >= (int32_t)p->boost_from_lap && (dash || behind > p->boost_behind) &&
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
    const int32_t r = fld_behind(f, i);
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
        /* Blocking limit: with a faster vehicle close behind on our line, hold
         * the current target instead of moving across it. */
        if(r >= 0 && !b->pass_dir) {
            const sh2z_vehicle_t *rv = &f->veh[r];
            const int32_t gap = sh2z_track_near_gap(f->line, &v->t, &rv->t);
            if(gap > 0 && gap < p->avoid_long && rv->t.speed > v->t.speed &&
               fld_abs(rv->t.lateral - v->t.lateral) < p->avoid_lat) lat = b->target_lat;
        }
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
    /* Never cut across a vehicle alongside: hold the current lateral while one
     * sits between here and the target. */
    if(!b->attack_ticks) {
        for(int side = 0; side < 2; ++side) {
            const int32_t n = side ? r : a;
            if(n < 0) continue;
            const sh2z_vehicle_t *nv = &f->veh[n];
            if(fld_abs(sh2z_track_near_gap(f->line, &nv->t, &v->t)) >= p->contact_long * 2) continue;
            const int32_t lo = (v->t.lateral < lat ? v->t.lateral : lat) - p->contact_lat;
            const int32_t hi = (v->t.lateral < lat ? lat : v->t.lateral) + p->contact_lat;
            if(nv->t.lateral > lo && nv->t.lateral < hi && fld_abs(nv->t.lateral - v->t.lateral) >= p->contact_lat) {
                lat = v->t.lateral;
                break;
            }
        }
    }
    b->target_lat = lat;
    /* line-field-5: tube roll decisions. */
    const sh2z_field_behave_t *bh = p->behave;
    if(bh && fld_tube(p, at->flags) && at->half_width > 0) {
        int exit = 0;
        if(fld_match(at->flags, bh->exit_mask, bh->exit_value)) {
            int32_t d = v->t.dist + bh->roll_exit_ahead;
            while(d >= f->line->length) d -= f->line->length;
            const uint32_t n = sh2z_line_locate(f->line, d, v->t.node);
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
static void fld_damage(sh2z_field_t *f, uint32_t i, int32_t amount) {
    sh2z_vehicle_t *v = &f->veh[i];
    if(amount <= 0 || (v->flags & (SH2Z_VEH_RETIRED | SH2Z_VEH_FINISHED | SH2Z_VEH_FALLING))) return;
    v->energy -= (int32_t)(((int64_t)amount * v->armor) >> 8);
    const sh2z_field_behave_t *bh = f->params->behave;
    if(v->energy <= 0 && bh && bh->ko_near > 0 && f->focus >= 0 && (uint32_t)f->focus < f->count && (int32_t)i != f->focus &&
       fld_abs(sh2z_track_near_gap(f->line, &v->t, &f->veh[f->focus].t)) > bh->ko_near) {
        v->energy = bh->ko_floor > 0 ? bh->ko_floor : 1;   /* line-field-5: far from the focus, energy cannot retire */
        return;
    }
    if(v->energy <= 0) {
        v->energy = 0;
        v->flags |= SH2Z_VEH_RETIRED;
        v->flags &= (uint16_t)~SH2Z_VEH_BOOST;
        f->brain[i].retire_tick = (int32_t)(f->tick - f->start_tick);
        ++f->retired;
    }
}

FLD_INLINE int fld_current(const sh2z_field_t *f, uint32_t i) {
    return (f->veh[i].flags & SH2Z_VEH_EXTERNAL) || f->brain[i].last_tick == f->tick;
}

/* Pushed into the line's edge: scrub speed and charge energy. */
static void fld_wall(sh2z_field_t *f, uint32_t i) {
    sh2z_vehicle_t *v = &f->veh[i];
    v->flags |= SH2Z_VEH_WALL;
    v->t.speed = fld_mul_q16(v->t.speed, f->params->wall_keep);
    fld_damage(f, i, f->params->wall_energy);
}

/* One pass over every pair in reach; returns the number of pairs pushed. */
static uint32_t fld_contacts(sh2z_field_t *f) {
    const sh2z_field_params_t *p = f->params;
    const sh2z_line_t *line = f->line;
    sh2z_line_sample_t at;
    uint32_t pushed = 0u;
    for(uint32_t s = 0u; s < f->count; ++s) {
        const uint32_t i0 = f->order[s];
        if(!fld_active(&f->veh[i0]) || !fld_current(f, i0)) continue;
        for(uint32_t k = 1u; k < f->count; ++k) {
            const uint32_t i1 = f->order[s + k < f->count ? s + k : s + k - f->count];
            if(!fld_active(&f->veh[i1])) continue;
            /* Only between vehicles that are both current this tick (a skipped
             * vehicle sits behind its true place and must not end the scan). */
            if(!fld_current(f, i1)) continue;
            int32_t gap = sh2z_track_near_gap(line, &f->veh[i0].t, &f->veh[i1].t);
            if(gap >= p->contact_long) break;
            /* The order dates from the start of the tick: a pair may have swapped. */
            uint32_t ia = i0, ib = i1;
            if(gap < 0) {
                if(gap <= -p->contact_long) continue;
                ia = i1; ib = i0; gap = -gap;
            }
            sh2z_vehicle_t *a = &f->veh[ia], *b = &f->veh[ib];   /* a ahead of b */
            int32_t dl = a->t.lateral - b->t.lateral;
            if(p->surface && p->surface->wrap_mask) {   /* line-field-4: across a tube's seam */
                sh2z_line_sample(line, a->t.node, a->t.dist, &at);
                if(fld_tube(p, at.flags) && at.half_width > 0) dl = fld_wrap(dl, fld_round(at.half_width));
            }
            const int32_t adl = fld_abs(dl);
            if(adl >= p->contact_lat) continue;
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
                        sh2z_line_sample(line, a->t.node, a->t.dist, &at);
                        const int32_t nl = a->t.lateral + dir * move_a;
                        if(fld_tube(p, at.flags) && at.half_width > 0) a->t.lateral = fld_wrap(nl, fld_round(at.half_width));
                        else {
                            a->t.lateral = fld_clamp(nl, -at.half_width, at.half_width);
                            left_a = fld_abs(nl - a->t.lateral);
                            if(left_a && !pass) fld_wall(f, ia);
                        }
                    }
                }
                if(move_b) {
                    if(b_ext) b->kick_lat -= dir * move_b;
                    else {
                        sh2z_line_sample(line, b->t.node, b->t.dist, &at);
                        const int32_t nl = b->t.lateral - dir * move_b;
                        if(fld_tube(p, at.flags) && at.half_width > 0) b->t.lateral = fld_wrap(nl, fld_round(at.half_width));
                        else {
                            b->t.lateral = fld_clamp(nl, -at.half_width, at.half_width);
                            left_b = fld_abs(nl - b->t.lateral);
                            if(left_b && !pass) fld_wall(f, ib);
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
            const int fresh = !(f->brain[ia].touch_cool && f->brain[ib].touch_cool);
            a->flags |= SH2Z_VEH_TOUCH; b->flags |= SH2Z_VEH_TOUCH;
            f->brain[ia].touch_cool = f->brain[ib].touch_cool = FLD_TOUCH_COOL;
            /* Give each other room: both hold the side they were pushed to. */
            if(!a_ext && !f->brain[ia].attack_ticks) {
                f->brain[ia].pass_dir = (int8_t)dir; f->brain[ia].pass_ticks = (uint8_t)fld_clamp(p->pass_hold, 1, 255);
                f->brain[ia].target_lat = b->t.lateral + dir * p->avoid_offset;
            }
            if(!b_ext && !f->brain[ib].attack_ticks) {
                f->brain[ib].pass_dir = (int8_t)-dir; f->brain[ib].pass_ticks = (uint8_t)fld_clamp(p->pass_hold, 1, 255);
                f->brain[ib].target_lat = a->t.lateral - dir * p->avoid_offset;
            }
            if(fresh && p->behave) {   /* line-field-5: a contact incident with an external vehicle starts a knock-out age */
                if(a_ext && !b_ext) { f->brain[ib].ko_by = (int16_t)ia; f->brain[ib].ko_age = 1u; }
                if(b_ext && !a_ext) { f->brain[ia].ko_by = (int16_t)ib; f->brain[ia].ko_age = 1u; }
            }
            const int32_t hit = fresh ? p->side_energy + (closing > 0 ? fld_mul_q16(closing, p->hit_energy) : 0) : 0;
            const int a_attacks = !a_ext && f->brain[ia].attack_ticks && f->brain[ia].attack_target == ib;
            const int b_attacks = !b_ext && f->brain[ib].attack_ticks && f->brain[ib].attack_target == ia;
            if(fresh && p->behave && p->behave->contact_norm > 0) {
                /* line-field-5: the impact is the closing speed along the line between the two (longitudinal gap and
                 * lateral offset; the length by max + min / 2), shared: contact_norm of it each, and the whole
                 * 2 x contact_norm to an attacker's victim (the attacker pays nothing). */
                const int32_t ax = fld_abs(gap), ay = adl, len = (ax > ay ? ax + (ay >> 1) : ay + (ax >> 1));
                const int64_t dot = (int64_t)closing * gap + (int64_t)(b->t.lat_vel - a->t.lat_vel) * dl;
                const int32_t norm = len > 0 ? (int32_t)((dot < 0 ? -dot : dot) / len) : fld_abs(closing);
                const int32_t share = fld_mul_q16(norm, p->behave->contact_norm);
                fld_damage(f, ia, a_attacks ? 0 : (b_attacks ? share * 2 : share));
                fld_damage(f, ib, b_attacks ? 0 : (a_attacks ? share * 2 : share));
            } else if(fresh) {
                fld_damage(f, ia, a_attacks ? 0 : hit + (b_attacks ? p->attack_energy : 0));
                fld_damage(f, ib, b_attacks ? 0 : hit + (a_attacks ? p->attack_energy : 0));
            }
            /* An attack that lands is over. */
            if(a_attacks) f->brain[ia].attack_ticks = 0u;
            if(b_attacks) f->brain[ib].attack_ticks = 0u;
            f->contacts += (uint32_t)fresh;
            ++pushed;
        }
    }
    return pushed;
}

static void fld_laps(sh2z_field_t *f, uint32_t i) {
    sh2z_vehicle_t *v = &f->veh[i];
    sh2z_field_brain_t *b = &f->brain[i];
    const int32_t lap = v->t.lap;
    while((int32_t)b->laps_seen < lap) {
        b->laps_seen = (int16_t)(b->laps_seen + 1);
        const int32_t done = (int32_t)b->laps_seen;          /* laps completed */
        if(done >= 1 && done <= 4) b->lap_tick[done - 1] = (int32_t)(f->tick - f->start_tick);
        if(done == 1 && b->finish_ticks > 0 && f->params->laps > 1u && !(v->flags & (SH2Z_VEH_EXTERNAL | SH2Z_VEH_RETIRED))) {
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

void sh2z_field_step(sh2z_field_t *f) {
    const sh2z_field_params_t *p = f->params;
    const sh2z_line_t *line = f->line;
    ++f->tick;
    /* Physical order over the vehicle array (largest dist first, ties by index). */
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
    if(!f->started) return;

    const sh2z_vehicle_t *focus = (f->focus >= 0 && (uint32_t)f->focus < f->count) ? &f->veh[f->focus] : (const sh2z_vehicle_t *)0;
    for(uint32_t i = 0u; i < f->count; ++i) {
        sh2z_vehicle_t *v = &f->veh[i];
        sh2z_field_brain_t *b = &f->brain[i];
        if(v->flags & SH2Z_VEH_EXTERNAL) { fld_laps(f, i); continue; }
        if(v->flags & SH2Z_VEH_STOPPED) continue;
        if(v->flags & SH2Z_VEH_FALLING) {   /* line-field-5: off the line, every tick, until it retires */
            const sh2z_field_behave_t *bh = p->behave;
            b->last_tick = f->tick;
            v->t.speed = fld_mul_q16(v->t.speed, 61604);   /* 0.94 a tick */
            v->t.lateral += v->t.lat_vel;
            if((int32_t)(f->tick - f->start_tick) - b->fall_tick >= (bh->fall_ticks > 0 ? bh->fall_ticks : 1)) {
                v->energy = 0;
                v->flags = (uint16_t)((v->flags | SH2Z_VEH_RETIRED | SH2Z_VEH_STOPPED) & ~(SH2Z_VEH_BOOST | SH2Z_VEH_FALLING));
                v->t.speed = 0;
                b->retire_tick = (int32_t)(f->tick - f->start_tick);
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
        const sh2z_field_behave_t *bh = p->behave;
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
        if(due > 1u || ((i + f->tick) & 3u) == 0u || (v->flags & (SH2Z_VEH_RETIRED | SH2Z_VEH_WALL))) fld_think(f, i, due > 1u ? due : 4u, &at);
        sh2z_follow_params_t fp;
        fp.accel = v->accel; fp.brake = (v->flags & SH2Z_VEH_RETIRED) ? v->brake * 2 : v->brake;
        fp.steer_gain = p->steer_gain; fp.steer_max = v->grip; fp.margin = b->attack_ticks ? 0 : p->margin;
        int32_t target = b->target_speed;
        const int32_t top = v->top_speed + ((v->flags & SH2Z_VEH_BOOST) ? p->boost_speed : 0);
        if(target > top) target = top;
        if(zone == SH2Z_FIELD_ZONE_SLIP) fp.steer_gain = fld_mul_q16(fp.steer_gain, sf->slip_steer);
        else if(zone == SH2Z_FIELD_ZONE_ROUGH && !(v->flags & SH2Z_VEH_BOOST)) target = fld_mul_q16(target, sf->rough_keep);
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
            fp.steer_gain = 0; fp.margin = -(INT32_C(1) << 28);
            sh2z_follow_step(line, &fp, &v->t, target, lat0, due);
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
                if(air) leaves = open;
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
            else if(v->t.lateral > now.half_width) { v->t.lateral = now.half_width; fld_wall(f, i); if(own_lateral) v->t.lat_vel = -(v->t.lat_vel >> 1); }
            else if(v->t.lateral < -now.half_width) { v->t.lateral = -now.half_width; fld_wall(f, i); if(own_lateral) v->t.lat_vel = -(v->t.lat_vel >> 1); }
            else if(fld_abs(v->t.lateral) >= at.half_width - 1 && at.half_width > 0) fld_wall(f, i);
        }
        if((v->flags & SH2Z_VEH_RETIRED) && v->t.speed == 0) v->flags |= SH2Z_VEH_STOPPED;
        fld_laps(f, i);
        ++f->steps;
    }
    /* Touch history: TOUCH is this tick only; the cooldown spans an incident. */
    for(uint32_t i = 0u; i < f->count; ++i) {
        f->veh[i].flags &= (uint16_t)~SH2Z_VEH_TOUCH;
        if(f->brain[i].touch_cool) --f->brain[i].touch_cool;
    }
    /* Further passes settle pile-ups of three or more (energy is per incident,
     * so nothing is charged twice). At most three passes. */
    for(int pass = 0; pass < 3; ++pass) if(fld_contacts(f) <= 1u) break;
    f->rank_changes += fld_rank(f);
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
        if(f->params->behave) { FLD_MIX(b->fall_tick); FLD_MIX(b->air_drift); FLD_MIX(b->roll); FLD_MIX(b->ko_by); FLD_MIX(b->ko_credit); FLD_MIX(b->ko_age); }
    }
    if(f->params->behave) { FLD_MIX(f->falls); FLD_MIX(f->kos); }
    return h;
}
