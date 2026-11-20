/* Effect pool (contract tag effect-pool-1) checks.
 * An independent double-precision transcription of the contract judges the
 * pool step over long random runs, the projection and pixel size on random
 * points, the spin of a basis and the far-to-near order; the ring's spawn,
 * refusal and oldest-first ending are checked on scripted runs; every
 * emitted VDP1 command is decoded word by word. Every rule has a control: the
 * oracle with that rule changed must disagree with the library.
 *
 * Copyright (c) 2026 SH2ZAM Contributors. MIT License.
 * Inspired by SH4ZAM by Falco Girgis and SH4ZAM Contributors.
 */
#include "sh2zam/effect_pool_experimental.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static unsigned long long checks;
#define CHECK(e) do { ++checks; if(!(e)) { fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #e); exit(EXIT_FAILURE); } } while(0)
#define Q16(x) ((int32_t)lround((x) * 65536.0))

static uint32_t rng = UINT32_C(0x9e3779b9);
static uint32_t rnd(void) { rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5; return rng; }
static int32_t rnd_range(int32_t lo, int32_t hi) { return (int32_t)((int64_t)lo + (int64_t)(rnd() % (uint32_t)((int64_t)hi - lo + 1))); }

/* ---- Oracle: the step in doubles ----------------------------------------- */

typedef struct { double pos[3], vel[3], scale; unsigned timer; int live; } opart_t;
typedef struct { double growth, damp, accel[3]; unsigned life; int vel_first; } orules_t;

static void ostep(opart_t *o, const orules_t *r) {
    int i;
    if(!o->live) return;
    o->timer = (o->timer + 1u) % r->life;
    if(o->timer == 0u) { o->live = 0; return; }
    o->scale *= r->growth;
    for(i = 0; i < 3; i++) {
        if(r->vel_first) {
            o->vel[i] = o->vel[i] * r->damp + r->accel[i];
            o->pos[i] += o->vel[i];
        } else {
            o->pos[i] += o->vel[i];
            o->vel[i] = o->vel[i] * r->damp + r->accel[i];
        }
    }
}

/* Largest error (Q16 LSB) between the library particle and the oracle. */
static double perr(const sh2z_ep_particle_t *e, const opart_t *o) {
    double w = fabs(e->scale - o->scale * 65536.0);
    int i;
    for(i = 0; i < 3; i++) {
        const double a = fabs(e->pos[i] - o->pos[i] * 65536.0), b = fabs(e->vel[i] - o->vel[i] * 65536.0);
        if(a > w) w = a;
        if(b > w) w = b;
    }
    return w;
}

/* One random run of a pool against the oracle; returns the worst error and
 * whether the oracle's life and liveness agreed every tick. */
static double run_pool(const orules_t *orl, uint32_t cap, uint32_t ticks, int *agree) {
    static sh2z_ep_particle_t items[256];
    static opart_t oracle[256];
    sh2z_ep_pool_t p;
    double worst = 0.0;
    uint32_t t, k;
    CHECK(sh2z_ep_init(&p, items, cap, 16u, Q16(1.05), 65536));
    p.life = 16u;
    p.growth = Q16(1.05);
    p.damp = Q16(0.98);
    p.accel[0] = 0; p.accel[1] = Q16(-0.4); p.accel[2] = Q16(0.05);
    memset(oracle, 0, sizeof(oracle));
    *agree = 1;
    for(t = 0; t < ticks; t++) {
        if((rnd() & 3u) == 0u) {
            int32_t pos[3], vel[3];
            int32_t sc = rnd_range(Q16(2.0), Q16(80.0));
            int i;
            for(i = 0; i < 3; i++) { pos[i] = rnd_range(-Q16(20000.0), Q16(20000.0)); vel[i] = rnd_range(-Q16(20.0), Q16(20.0)); }
            {
                const uint32_t before = p.count;
                const int32_t slot = sh2z_ep_spawn(&p, pos, vel, sc, 7u, 9u);
                if(before == cap) CHECK(slot < 0 && p.count == cap);
                else {
                    opart_t *o = &oracle[slot];
                    CHECK(slot >= 0 && p.count == before + 1u);
                    o->live = 1; o->timer = 0u; o->scale = sc / 65536.0;
                    for(i = 0; i < 3; i++) { o->vel[i] = vel[i] / 65536.0; o->pos[i] = (pos[i] + vel[i]) / 65536.0; }
                }
            }
        }
        sh2z_ep_step(&p);
        for(k = 0; k < cap; k++) ostep(&oracle[k], orl);
        /* Live set: the oracle's live particles are exactly the count newest slots. */
        {
            uint32_t live = 0;
            for(k = 0; k < cap; k++) live += (uint32_t)oracle[k].live;
            if(live != p.count) *agree = 0;
            for(k = 0; k < p.count; k++) {
                const uint32_t slot = (p.head - 1u - k) & p.mask;
                const sh2z_ep_particle_t *e = sh2z_ep_at(&p, k);
                if(!oracle[slot].live || e->timer != oracle[slot].timer) { *agree = 0; continue; }
                {
                    const double w = perr(e, &oracle[slot]);
                    if(w > worst) worst = w;
                }
            }
        }
    }
    return worst;
}

/* The contract's rule on the pool's own Q16 constants. */
static orules_t base_rules(void) {
    orules_t o;
    o.growth = Q16(1.05) / 65536.0; o.damp = Q16(0.98) / 65536.0;
    o.accel[0] = 0.0; o.accel[1] = Q16(-0.4) / 65536.0; o.accel[2] = Q16(0.05) / 65536.0;
    o.life = 16u; o.vel_first = 1;
    return o;
}

/* The bound: each tick rounds the scale and the velocity once (at most half
 * an LSB each, damped); a position sums the velocity's rounding over at most
 * 15 ticks: 0.5 * (1 + 2 + ... + 15) = 60 LSB. */
#define STEP_BOUND 64.0

static void test_pool(void) {
    const orules_t o = base_rules();
    int agree;
    uint32_t caps[] = { 1u, 8u, 32u, 128u };
    double worst = 0.0;
    unsigned i;
    for(i = 0; i < 4; i++) {
        const double w = run_pool(&o, caps[i], 4000u, &agree);
        CHECK(agree);
        CHECK(w <= STEP_BOUND);
        if(w > worst) worst = w;
    }
    printf("effect_pool: pool step worst %.2f Q16 LSB (bound %.0f)\n", worst, STEP_BOUND);
    /* Controls: each changed rule must disagree. */
    {
        orules_t c = o; c.growth = 1.02;
        CHECK(run_pool(&c, 32u, 2000u, &agree) > STEP_BOUND);
        c = o; c.damp = 1.0;
        CHECK(run_pool(&c, 32u, 2000u, &agree) > STEP_BOUND);
        c = o; c.accel[1] = 0.4;
        CHECK(run_pool(&c, 32u, 2000u, &agree) > STEP_BOUND);
        c = o; c.vel_first = 0;
        CHECK(run_pool(&c, 32u, 2000u, &agree) > STEP_BOUND);
        c = o; c.life = 15u;
        (void)run_pool(&c, 32u, 2000u, &agree);
        CHECK(!agree);
    }
    /* Scripted ring: refusal when full, oldest first, count newest live. */
    {
        sh2z_ep_particle_t items[4];
        sh2z_ep_pool_t p;
        const int32_t z[3] = { 0, 0, 0 };
        uint32_t t;
        CHECK(!sh2z_ep_init(&p, items, 3u, 16u, 65536, 65536));
        CHECK(!sh2z_ep_init(&p, items, 4u, 1u, 65536, 65536));
        CHECK(sh2z_ep_init(&p, items, 4u, 5u, 65536, 65536));
        CHECK(sh2z_ep_spawn(&p, z, z, 1, 1u, 0u) == 0);
        sh2z_ep_step(&p);
        CHECK(sh2z_ep_spawn(&p, z, z, 1, 2u, 0u) == 1);
        CHECK(sh2z_ep_spawn(&p, z, z, 1, 3u, 0u) == 2);
        CHECK(sh2z_ep_spawn(&p, z, z, 1, 4u, 0u) == 3);
        CHECK(sh2z_ep_spawn(&p, z, z, 1, 5u, 0u) == -1);
        CHECK(p.count == 4u && sh2z_ep_at(&p, 0)->user == 4u && sh2z_ep_at(&p, 3)->user == 1u);
        for(t = 0; t < 3u; t++) sh2z_ep_step(&p);
        /* The first lived 4 ticks: one more ends it. */
        CHECK(p.count == 4u);
        sh2z_ep_step(&p);
        CHECK(p.count == 3u && sh2z_ep_at(&p, 2)->user == 2u && items[0].timer == 0u);
        CHECK(sh2z_ep_spawn(&p, z, z, 1, 6u, 0u) == 0);
        sh2z_ep_step(&p);
        CHECK(p.count == 1u && sh2z_ep_at(&p, 0)->user == 6u);
        sh2z_ep_clear(&p);
        CHECK(p.count == 0u);
    }
}

/* ---- Projection ----------------------------------------------------------- */

static void make_view(sh2z_ep_view_t *v, double yaw, double pitch) {
    const double cy = cos(yaw), sy = sin(yaw), cp = cos(pitch), sp = sin(pitch);
    const double fwd[3] = { sy * cp, -sp, cy * cp }, right[3] = { cy, 0.0, -sy };
    double up[3];
    int i;
    up[0] = right[1] * fwd[2] - right[2] * fwd[1];
    up[1] = right[2] * fwd[0] - right[0] * fwd[2];
    up[2] = right[0] * fwd[1] - right[1] * fwd[0];
    for(i = 0; i < 3; i++) {
        v->r[0][i] = (int16_t)lround(right[i] * 16384.0);
        v->r[1][i] = (int16_t)lround(up[i] * 16384.0);
        v->r[2][i] = (int16_t)lround(fwd[i] * 16384.0);
    }
    v->eye[0] = Q16(120.5); v->eye[1] = Q16(-40.25); v->eye[2] = Q16(3000.0);
    v->focal_x = Q16(277.0); v->focal_y = Q16(240.0);
    v->center_x = 160; v->center_y = 112;
    v->near_z = 4 * 16384;
    v->width = 320; v->height = 224; v->guard = 64;
}

static int oproject(const sh2z_ep_view_t *v, const int32_t pos[3], int sign_x, double *x, double *y, double *z) {
    double d[3], t[3];
    int i, k;
    for(i = 0; i < 3; i++) d[i] = (double)((int32_t)((uint32_t)pos[i] - (uint32_t)v->eye[i]));
    for(i = 0; i < 3; i++) { t[i] = 0.0; for(k = 0; k < 3; k++) t[i] += v->r[i][k] * d[k]; t[i] = floor(t[i] / 65536.0); }
    if(t[2] <= v->near_z || t[2] <= 0.0) return 0;
    *x = v->center_x * 65536.0 + sign_x * trunc(v->focal_x * t[0] / t[2]);
    *y = v->center_y * 65536.0 - trunc(v->focal_y * t[1] / t[2]);
    *z = t[2];
    if(*x < -v->guard * 65536.0 || *x >= (v->width + v->guard) * 65536.0) return 0;
    if(*y < -v->guard * 65536.0 || *y >= (v->height + v->guard) * 65536.0) return 0;
    return 1;
}

static void test_project(void) {
    sh2z_ep_view_t v;
    unsigned i, shown = 0, mism_sign = 0;
    for(i = 0; i < 200000u; i++) {
        int32_t pos[3];
        sh2z_ep_point_t pt;
        double x, y, z;
        int ok, ook;
        if(i % 1000u == 0u) {
            const double yaw = (rnd() % 6283u) / 1000.0;
            const double pitch = ((rnd() % 600u) - 300.0) / 1000.0;
            make_view(&v, yaw, pitch);
        }
        pos[0] = v.eye[0] + rnd_range(-Q16(3000.0), Q16(3000.0));
        pos[1] = v.eye[1] + rnd_range(-Q16(800.0), Q16(800.0));
        pos[2] = v.eye[2] + rnd_range(-Q16(3000.0), Q16(3000.0));
        ok = sh2z_ep_project(&v, pos, &pt);
        ook = oproject(&v, pos, 1, &x, &y, &z);
        CHECK(ok == ook);
        if(!ok) continue;
        ++shown;
        CHECK(pt.x == (int32_t)x && pt.y == (int32_t)y && pt.depth == (int32_t)z);
        {
            const int32_t len = rnd_range(1, Q16(200.0));
            const double want = trunc(v.focal_y * floor(len / 4.0) / z);
            CHECK(sh2z_ep_pixels(&v, len, pt.depth) == (int32_t)want);
        }
        /* Control: a mirrored x must disagree off the centre line. */
        if(oproject(&v, pos, -1, &x, &y, &z) && (int32_t)x != pt.x) ++mism_sign;
    }
    CHECK(shown > 20000u);
    CHECK(mism_sign > shown / 2u);
    CHECK(sh2z_ep_pixels(&v, 0, 100) == 0 && sh2z_ep_pixels(&v, 100, 0) == 0);
    printf("effect_pool: projection %u points shown, exact; mirror control %u\n", shown, mism_sign);
}

/* ---- Commands ------------------------------------------------------------- */

static void test_emit(void) {
    uint16_t c[16];
    int16_t xy[4][2];
    memset(c, 0xAB, sizeof(c));
    CHECK(sh2z_ep_emit_diamond(c, 100, 50, 7, 0x40000u, 32u, 32u, 0x7e020u, 0x88u));
    CHECK(c[0] == 2u && c[1] == 0u && c[2] == 0x88u && c[3] == (0x7e020u >> 3) && c[4] == (0x40000u >> 3));
    CHECK(c[5] == ((4u << 8) | 32u));
    /* A (+r, 0), B (0, +r), C (-r, 0), D (0, -r). */
    CHECK((int16_t)c[6] == 107 && (int16_t)c[7] == 50 && (int16_t)c[8] == 100 && (int16_t)c[9] == 57);
    CHECK((int16_t)c[10] == 93 && (int16_t)c[11] == 50 && (int16_t)c[12] == 100 && (int16_t)c[13] == 43);
    CHECK(c[14] == 0u && c[15] == 0u);
    memset(c, 0xCD, sizeof(c));
    CHECK(!sh2z_ep_emit_diamond(c, 1020, 0, 7, 0x40000u, 32u, 32u, 0x7e020u, 0u));
    CHECK(c[0] == 0xCDCDu);
    CHECK(!sh2z_ep_emit_diamond(c, 0, 0, 0, 0x40000u, 32u, 32u, 0x7e020u, 0u));
    CHECK(!sh2z_ep_emit_diamond(c, 0, 0, 4, 0x40004u, 32u, 32u, 0x7e020u, 0u));
    CHECK(!sh2z_ep_emit_diamond(c, 0, 0, 4, 0x40000u, 12u, 32u, 0x7e020u, 0u));
    CHECK(!sh2z_ep_emit_diamond(c, 0, 0, 4, 0x40000u, 32u, 32u, 0x7e028u, 0u));
    CHECK(!sh2z_ep_emit_diamond(c, 0, 0, 4, 0x40000u, 32u, 256u, 0x7e020u, 0u));
    xy[0][0] = -1024; xy[0][1] = 1023; xy[1][0] = 5; xy[1][1] = -6; xy[2][0] = 7; xy[2][1] = 8; xy[3][0] = 9; xy[3][1] = 10;
    CHECK(sh2z_ep_emit_quad(c, (const int16_t (*)[2])xy, 0x100u, 16u, 16u, 0x7e000u, 0x1234u));
    CHECK((int16_t)c[6] == -1024 && (int16_t)c[7] == 1023 && (int16_t)c[13] == 10 && c[5] == ((2u << 8) | 16u));
    xy[2][0] = 1024;
    CHECK(!sh2z_ep_emit_quad(c, (const int16_t (*)[2])xy, 0x100u, 16u, 16u, 0x7e000u, 0u));
    xy[2][0] = 7;
    CHECK(sh2z_ep_emit_shape(c, SH2Z_EP_POLYGON, (const int16_t (*)[2])xy, 0x801Fu, 0x0C0u, 0x7f000u));
    CHECK(c[0] == 4u && c[3] == 0x801Fu && c[4] == 0u && c[5] == 0u && c[14] == (0x7f000u >> 3) && (int16_t)c[12] == 9);
    xy[3][0] = 2000; /* ignored for a line */
    CHECK(sh2z_ep_emit_shape(c, SH2Z_EP_LINE, (const int16_t (*)[2])xy, 0x83E0u, 0u, 0u));
    CHECK(c[0] == 6u && c[10] == 0u && c[13] == 0u && (int16_t)c[8] == 5);
    CHECK(!sh2z_ep_emit_shape(c, SH2Z_EP_POLYGON, (const int16_t (*)[2])xy, 0x83E0u, 0u, 0u));
    CHECK(!sh2z_ep_emit_shape(c, 2u, (const int16_t (*)[2])xy, 0x83E0u, 0u, 0u));
    CHECK(!sh2z_ep_emit_shape(c, SH2Z_EP_LINE, (const int16_t (*)[2])xy, 0x83E0u, 0u, 4u));
}

/* ---- Spin and order ------------------------------------------------------- */

static void test_spin(void) {
    unsigned i, controls = 0;
    double worst = 0.0;
    for(i = 0; i < 100000u; i++) {
        const double a = (rnd() % 6283u) / 1000.0;
        const double b = (rnd() % 3141u) / 1000.0 - 1.57;
        int16_t f[3], s[3];
        double fo[3], so[3];
        const uint32_t ang = rnd() & 4095u;
        const double t = ang * (2.0 * 3.14159265358979323846 / 4096.0);
        int k;
        fo[0] = cos(b) * sin(a); fo[1] = sin(b); fo[2] = cos(b) * cos(a);
        so[0] = cos(a); so[1] = 0.0; so[2] = -sin(a);
        for(k = 0; k < 3; k++) { f[k] = (int16_t)lround(fo[k] * 16384.0); s[k] = (int16_t)lround(so[k] * 16384.0); }
        {
            double fw[3], sw[3], fwx[3];
            for(k = 0; k < 3; k++) {
                fw[k] = f[k] * cos(t) + s[k] * sin(t);
                sw[k] = s[k] * cos(t) - f[k] * sin(t);
                fwx[k] = f[k] * cos(t) - s[k] * sin(t);
            }
            sh2z_ep_spin_rows(f, s, ang);
            for(k = 0; k < 3; k++) {
                const double e1 = fabs(f[k] - fw[k]), e2 = fabs(s[k] - sw[k]);
                if(e1 > worst) worst = e1;
                if(e2 > worst) worst = e2;
                if(fabs(f[k] - fwx[k]) > 2.0) ++controls;
            }
        }
    }
    CHECK(worst <= 1.5);
    CHECK(controls > 50000u);
    printf("effect_pool: spin worst %.3f Q14 LSB; reversed-turn control %u\n", worst, controls);
}

static void test_sort(void) {
    unsigned trial;
    for(trial = 0; trial < 2000u; trial++) {
        int32_t depth[256];
        uint8_t order[256], want[256];
        const uint32_t n = (uint32_t)rnd_range(0, 256);
        uint32_t i, j, w = 0;
        for(i = 0; i < n; i++) depth[i] = rnd_range(0, 40);
        sh2z_ep_sort(order, depth, n);
        /* Reference: stable selection by depth from the largest down. */
        for(j = 41u; j-- > 0u;)
            for(i = 0; i < n; i++)
                if(depth[i] == (int32_t)j) want[w++] = (uint8_t)i;
        CHECK(w == n);
        CHECK(n == 0u || memcmp(order, want, n) == 0);
    }
}

int main(void) {
    test_pool();
    test_project();
    test_emit();
    test_spin();
    test_sort();
    printf("effect_pool: %llu checks passed\n", checks);
    return EXIT_SUCCESS;
}
