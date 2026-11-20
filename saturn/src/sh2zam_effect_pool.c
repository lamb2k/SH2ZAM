/* Experimental effect pool (contract tag effect-pool-1).
 *
 * Copyright (c) 2026 SH2ZAM Contributors. MIT License.
 * Inspired by SH4ZAM by Falco Girgis and SH4ZAM Contributors.
 */
#include "sh2zam/effect_pool_experimental.h"
#include "sh2zam/divu_experimental.h"
#include "sh2zam/trig12_experimental.h"

static int32_t ep_sat(int64_t x) {
    if(x > INT32_MAX) return INT32_MAX;
    if(x < INT32_MIN) return INT32_MIN;
    return (int32_t)x;
}

/* a (Q16) times b (Q16), rounded to nearest (halves away from zero). */
static int32_t ep_m16(int32_t a, int32_t b) {
    const int64_t x = (int64_t)a * b;
    return ep_sat(x >= 0 ? (x + 0x8000) >> 16 : -((-x + 0x8000) >> 16));
}

bool sh2z_ep_init(sh2z_ep_pool_t *p, sh2z_ep_particle_t *items, uint32_t capacity, uint32_t life,
                  int32_t growth, int32_t damp) {
    uint32_t i;
    if(!p || !items || capacity == 0u || capacity > 256u || (capacity & (capacity - 1u)) != 0u) return false;
    if(life < 2u || life > 65535u) return false;
    p->items = items;
    p->mask = capacity - 1u;
    p->head = 0u;
    p->count = 0u;
    p->life = life;
    p->growth = growth;
    p->damp = damp;
    p->accel[0] = p->accel[1] = p->accel[2] = 0;
    for(i = 0; i < capacity; i++) {
        sh2z_ep_particle_t *e = &items[i];
        e->pos[0] = e->pos[1] = e->pos[2] = 0;
        e->vel[0] = e->vel[1] = e->vel[2] = 0;
        e->scale = 0;
        e->timer = 0u;
        e->user = 0u;
        e->seed = 0u;
    }
    return true;
}

void sh2z_ep_clear(sh2z_ep_pool_t *p) {
    uint32_t i;
    if(!p) return;
    for(i = 0; i <= p->mask; i++) p->items[i].timer = 0u;
    p->count = 0u;
}

int32_t sh2z_ep_spawn(sh2z_ep_pool_t *p, const int32_t pos[3], const int32_t vel[3], int32_t scale,
                      uint8_t user, uint8_t seed) {
    sh2z_ep_particle_t *e;
    uint32_t slot;
    int i;
    if(!p || !pos || !vel || p->count > p->mask) return -1;
    slot = p->head;
    e = &p->items[slot];
    for(i = 0; i < 3; i++) {
        e->vel[i] = vel[i];
        e->pos[i] = ep_sat((int64_t)pos[i] + vel[i]);
    }
    e->scale = scale;
    e->timer = 0u;
    e->user = user;
    e->seed = seed;
    p->head = (slot + 1u) & p->mask;
    p->count++;
    return (int32_t)slot;
}

/* a + b, saturated (32-bit, no 64-bit arithmetic). */
static int32_t ep_add(int32_t a, int32_t b) {
    const int32_t r = (int32_t)((uint32_t)a + (uint32_t)b);
    if(((a ^ r) & (b ^ r)) < 0) return a < 0 ? INT32_MIN : INT32_MAX;
    return r;
}

void sh2z_ep_step(sh2z_ep_pool_t *p) {
    uint32_t k, n;
    bool plain;
    if(!p) return;
    n = p->count;
    /* damp 1.0 leaves the velocity unchanged (ep_m16(v, 65536) == v); acceleration 0 adds nothing */
    plain = p->damp == 65536 && p->accel[0] == 0 && p->accel[1] == 0 && p->accel[2] == 0;
    for(k = 0; k < n; k++) {
        sh2z_ep_particle_t *e = &p->items[(p->head - 1u - k) & p->mask];
        uint32_t t = (uint32_t)e->timer + 1u;
        int i;
        if(t >= p->life) t = 0u;
        e->timer = (uint16_t)t;
        if(t == 0u) {
            p->count--;
            continue;
        }
        e->scale = ep_m16(e->scale, p->growth);
        if(plain) {
            for(i = 0; i < 3; i++) e->pos[i] = ep_add(e->pos[i], e->vel[i]);
            continue;
        }
        for(i = 0; i < 3; i++) {
            const int32_t v = p->damp == 65536 ? e->vel[i] : ep_m16(e->vel[i], p->damp);
            e->vel[i] = ep_add(v, p->accel[i]);
            e->pos[i] = ep_add(e->pos[i], e->vel[i]);
        }
    }
}

/* floor(r . d / 2^16) for a Q1.14 row and a Q16 vector (exact 64-bit sum). */
static int32_t ep_dot16(const int16_t r[3], const int32_t d[3]) {
    const int64_t s = (int64_t)r[0] * d[0] + (int64_t)r[1] * d[1] + (int64_t)r[2] * d[2];
    return ep_sat(s >> 16);
}

bool sh2z_ep_project(const sh2z_ep_view_t *v, const int32_t pos[3], sh2z_ep_point_t *out) {
    int32_t d[3], tx, ty, tz, qx, qy;
    int64_t ax, ay, gx, gy;
    if(!v || !pos || !out) return false;
    d[0] = (int32_t)((uint32_t)pos[0] - (uint32_t)v->eye[0]);
    d[1] = (int32_t)((uint32_t)pos[1] - (uint32_t)v->eye[1]);
    d[2] = (int32_t)((uint32_t)pos[2] - (uint32_t)v->eye[2]);
    tz = ep_dot16(v->r[2], d);
    if(tz <= v->near_z || tz <= 0) return false;
    tx = ep_dot16(v->r[0], d);
    ty = ep_dot16(v->r[1], d);
    qx = sh2z_divu_div64((int64_t)v->focal_x * tx, tz, NULL);
    qy = sh2z_divu_div64((int64_t)v->focal_y * ty, tz, NULL);
    ax = (int64_t)v->center_x * 65536 + qx;
    ay = (int64_t)v->center_y * 65536 - qy;
    gx = (int64_t)v->guard * 65536;
    gy = gx;
    if(ax < -gx || ax >= (int64_t)v->width * 65536 + gx) return false;
    if(ay < -gy || ay >= (int64_t)v->height * 65536 + gy) return false;
    out->x = (int32_t)ax;
    out->y = (int32_t)ay;
    out->depth = tz;
    return true;
}

int32_t sh2z_ep_pixels(const sh2z_ep_view_t *v, int32_t length, int32_t depth) {
    if(!v || depth <= 0 || length <= 0) return 0;
    return sh2z_divu_div64((int64_t)v->focal_y * (length >> 2), depth, NULL);
}

int32_t sh2z_ep_ratio(const sh2z_ep_view_t *v) {
    if(!v || v->focal_x <= 0) return 0;
    return sh2z_divu_div64((int64_t)v->focal_y * 65536, v->focal_x, NULL);
}

int32_t sh2z_ep_depth(const sh2z_ep_view_t *v, const int32_t pos[3]) {
    int32_t d[3];
    if(!v || !pos) return 0;
    d[0] = (int32_t)((uint32_t)pos[0] - (uint32_t)v->eye[0]);
    d[1] = (int32_t)((uint32_t)pos[1] - (uint32_t)v->eye[1]);
    d[2] = (int32_t)((uint32_t)pos[2] - (uint32_t)v->eye[2]);
    return ep_dot16(v->r[2], d);
}

bool sh2z_ep_spot(const sh2z_ep_view_t *v, int32_t ratio, const int32_t pos[3], sh2z_ep_spot_t *out) {
    int32_t d[3], tz, s, sy;
    int64_t x, y;
    if(!v || !pos || !out) return false;
    d[0] = (int32_t)((uint32_t)pos[0] - (uint32_t)v->eye[0]);
    d[1] = (int32_t)((uint32_t)pos[1] - (uint32_t)v->eye[1]);
    d[2] = (int32_t)((uint32_t)pos[2] - (uint32_t)v->eye[2]);
    tz = ep_dot16(v->r[2], d);
    if(tz <= v->near_z || tz <= 0) return false;
    s = sh2z_divu_div64((int64_t)v->focal_x * 65536, tz, NULL);
    sy = (int32_t)(((int64_t)s * ratio) >> 16);
    x = (int64_t)v->center_x + (((int64_t)ep_dot16(v->r[0], d) * s) >> 32);
    y = (int64_t)v->center_y + ((-(int64_t)ep_dot16(v->r[1], d) * sy) >> 32);
    if(x < -(int64_t)v->guard || x >= (int64_t)v->width + v->guard) return false;
    if(y < -(int64_t)v->guard || y >= (int64_t)v->height + v->guard) return false;
    out->x = (int32_t)x;
    out->y = (int32_t)y;
    out->depth = tz;
    out->scale = sy;
    return true;
}

int32_t sh2z_ep_spot_pixels(const sh2z_ep_spot_t *s, int32_t length) {
    if(!s || length <= 0 || s->scale <= 0) return 0;
    return (int32_t)(((int64_t)(length >> 2) * s->scale) >> 32);
}

static bool ep_in(int32_t c) { return c >= -1024 && c <= 1023; }

static void ep_words(uint16_t cmd[16], uint16_t ctrl, uint16_t pmod, uint16_t colr, uint16_t srca, uint16_t size,
                     const int16_t xy[4][2], uint16_t grda) {
    int i;
    cmd[0] = ctrl;
    cmd[1] = 0u;
    cmd[2] = pmod;
    cmd[3] = colr;
    cmd[4] = srca;
    cmd[5] = size;
    for(i = 0; i < 4; i++) {
        cmd[6 + 2 * i] = (uint16_t)xy[i][0];
        cmd[7 + 2 * i] = (uint16_t)xy[i][1];
    }
    cmd[14] = grda;
    cmd[15] = 0u;
}

static bool ep_texture_ok(uint32_t vram, uint32_t w, uint32_t h, uint32_t colr) {
    if((vram & 7u) || (colr & 31u) || vram > 0x7ffffu || colr > 0x7ffe0u) return false;
    if(w < 8u || w > 504u || (w & 7u) || h < 1u || h > 255u) return false;
    return true;
}

bool sh2z_ep_emit_quad(uint16_t cmd[16], const int16_t xy[4][2], uint32_t vram, uint32_t w, uint32_t h,
                       uint32_t colr, uint16_t pmod) {
    int i;
    if(!cmd || !xy || !ep_texture_ok(vram, w, h, colr)) return false;
    for(i = 0; i < 4; i++)
        if(!ep_in(xy[i][0]) || !ep_in(xy[i][1])) return false;
    ep_words(cmd, (uint16_t)SH2Z_EP_DISTORTED, pmod, (uint16_t)(colr >> 3), (uint16_t)(vram >> 3),
             (uint16_t)(((w >> 3) << 8) | h), xy, 0u);
    return true;
}

bool sh2z_ep_emit_diamond(uint16_t cmd[16], int32_t x, int32_t y, int32_t r, uint32_t vram, uint32_t w,
                          uint32_t h, uint32_t colr, uint16_t pmod) {
    int16_t xy[4][2];
    if(r < 1 || x - r < -1024 || x + r > 1023 || y - r < -1024 || y + r > 1023) return false;
    xy[0][0] = (int16_t)(x + r); xy[0][1] = (int16_t)y;
    xy[1][0] = (int16_t)x;       xy[1][1] = (int16_t)(y + r);
    xy[2][0] = (int16_t)(x - r); xy[2][1] = (int16_t)y;
    xy[3][0] = (int16_t)x;       xy[3][1] = (int16_t)(y - r);
    return sh2z_ep_emit_quad(cmd, (const int16_t (*)[2])xy, vram, w, h, colr, pmod);
}

bool sh2z_ep_emit_shape(uint16_t cmd[16], uint32_t code, const int16_t xy[4][2], uint16_t colour,
                        uint16_t pmod, uint32_t gouraud) {
    int i, n;
    if(!cmd || !xy || (code != SH2Z_EP_POLYGON && code != SH2Z_EP_LINE)) return false;
    if((gouraud & 7u) || gouraud > 0x7fff8u) return false;
    n = code == SH2Z_EP_LINE ? 2 : 4;
    for(i = 0; i < n; i++)
        if(!ep_in(xy[i][0]) || !ep_in(xy[i][1])) return false;
    ep_words(cmd, (uint16_t)code, pmod, colour, 0u, 0u, xy, (uint16_t)(gouraud >> 3));
    if(n == 2) cmd[10] = cmd[11] = cmd[12] = cmd[13] = 0u;
    return true;
}

/* a (Q1.14) times b (Q2.30), rounded to nearest, back to Q1.14. */
static int32_t ep_m30(int32_t a, int32_t b) {
    const int64_t x = (int64_t)a * b;
    return ep_sat(x >= 0 ? (x + (INT64_C(1) << 29)) >> 30 : -((-x + (INT64_C(1) << 29)) >> 30));
}

static int16_t ep_s16(int32_t v) {
    if(v > INT16_MAX) return INT16_MAX;
    if(v < INT16_MIN) return INT16_MIN;
    return (int16_t)v;
}

void sh2z_ep_spin_rows(int16_t fwd[3], int16_t side[3], uint32_t angle12) {
    const int32_t s = sh2z_sin12_u30(angle12), c = sh2z_cos12_u30(angle12);
    int i;
    if(!fwd || !side) return;
    for(i = 0; i < 3; i++) {
        const int32_t f = fwd[i], d = side[i];
        fwd[i] = ep_s16(ep_m30(f, c) + ep_m30(d, s));
        side[i] = ep_s16(ep_m30(d, c) - ep_m30(f, s));
    }
}

void sh2z_ep_sort(uint8_t *order, const int32_t *depth, uint32_t n) {
    uint32_t i;
    if(!order || !depth || n > 256u) return;
    for(i = 0; i < n; i++) {
        uint32_t j = i;
        while(j > 0u && depth[order[j - 1u]] < depth[i]) {
            order[j] = order[j - 1u];
            j--;
        }
        order[j] = (uint8_t)i;
    }
}
