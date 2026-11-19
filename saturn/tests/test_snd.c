/* Sound events and CD audio (snd-1): oracle and contract tests.
 *
 * The queue and the slot allocator are driven by long random scripts and
 * compared step by step with independent reference models. Every slot write
 * is decoded back from the register array and checked against the request.
 * The CD player runs against a model CD block that answers commands after a
 * random delay and counts any command written while it was busy. Each rule
 * has a control: the same comparison against a reference with that one rule
 * broken must fail.
 *
 * Copyright (c) 2026 SH2ZAM Contributors. MIT License.
 * Inspired by SH4ZAM by Falco Girgis and SH4ZAM Contributors.
 */
#include "sh2zam/snd_experimental.h"
#include <math.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static uint64_t checks;
#define CHECK(e) do { ++checks; if (!(e)) { fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #e); exit(EXIT_FAILURE); } } while (0)
static uint32_t rng = UINT32_C(0x2545f491);
static uint32_t rnd(void) { rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5; return rng; }

/* ---- A. Queue against a reference ---- */
enum { BREAK_NONE, BREAK_AFTER_CLOSE, BREAK_EPOCH, BREAK_CLOSE_DROPS, BREAK_LIFO };

typedef struct { uint16_t id[64]; int16_t arg[64]; uint32_t n, open, epoch, rejected, dropped; } ref_queue_t;

static int ref_post(ref_queue_t *r, uint16_t id, int16_t arg, uint32_t epoch, int broken) {
    const int open = r->open || broken == BREAK_AFTER_CLOSE;
    const int same = epoch == r->epoch || broken == BREAK_EPOCH;
    if (!open || !same) { ++r->rejected; return 0; }
    if (r->n >= SH2Z_SND_QUEUE_MAX) { ++r->dropped; return 0; }
    r->id[r->n] = id; r->arg[r->n] = arg; ++r->n;
    return 1;
}

typedef struct { uint16_t id[64]; int16_t arg[64]; uint32_t n; } got_t;
static void collect(const sh2z_snd_event_t *e, void *ctx) {
    got_t *g = ctx; g->id[g->n] = e->id; g->arg[g->n] = e->arg; ++g->n;
}

/* Returns 1 when the library and the reference agree over the whole script. */
static int queue_script(uint32_t seed, int broken) {
    rng = seed;
    sh2z_snd_queue_t q; ref_queue_t r; memset(&r, 0, sizeof(r));
    sh2z_snd_queue_init(&q);
    uint32_t epoch = 1u;
    for (uint32_t step = 0; step < 400u; ++step) {
        const uint32_t op = rnd() % 10u;
        if (op == 0u) { ++epoch; sh2z_snd_queue_open(&q, epoch); r.open = 1u; r.epoch = epoch; }
        else if (op == 1u) {
            sh2z_snd_queue_close(&q); r.open = 0u;
            if (broken == BREAK_CLOSE_DROPS) r.n = 0u;
        } else if (op < 8u) {
            const uint32_t e = (rnd() & 3u) ? epoch : epoch - 1u - (rnd() & 1u);
            const uint16_t id = (uint16_t)(rnd() & 0xFFFFu); const int16_t arg = (int16_t)(rnd() & 0x7FFFu);
            if ((int)sh2z_snd_queue_post(&q, id, arg, e) != ref_post(&r, id, arg, e, broken)) return 0;
        } else {
            got_t g; g.n = 0u;
            const uint32_t n = sh2z_snd_queue_drain(&q, collect, &g);
            if (n != r.n || g.n != r.n) return 0;
            for (uint32_t i = 0; i < n; ++i) {
                const uint32_t k = broken == BREAK_LIFO ? n - 1u - i : i;
                if (g.id[i] != r.id[k] || g.arg[i] != r.arg[k]) return 0;
            }
            r.n = 0u;
        }
        if (q.rejected != r.rejected || q.dropped != r.dropped || q.count != r.n) return 0;
    }
    return 1;
}

static void test_queue(void) {
    sh2z_snd_queue_t q; got_t g; g.n = 0u;
    sh2z_snd_queue_init(&q);
    CHECK(!sh2z_snd_queue_post(&q, 1u, 0, 0u) && q.rejected == 1u);      /* closed at start */
    sh2z_snd_queue_open(&q, 5u);
    CHECK(sh2z_snd_queue_post(&q, 7u, -3, 5u));
    CHECK(!sh2z_snd_queue_post(&q, 8u, 0, 4u) && q.rejected == 2u);     /* another epoch */
    sh2z_snd_queue_close(&q);                                             /* the transition begins */
    CHECK(!sh2z_snd_queue_post(&q, 9u, 0, 5u) && q.rejected == 3u);     /* posted after leave */
    CHECK(sh2z_snd_queue_drain(&q, collect, &g) == 1u && g.n == 1u && g.id[0] == 7u && g.arg[0] == -3);
    CHECK(sh2z_snd_queue_drain(&q, collect, &g) == 0u && g.n == 1u);      /* exactly once */
    sh2z_snd_queue_open(&q, 6u);
    for (uint32_t i = 0; i < SH2Z_SND_QUEUE_MAX; ++i) CHECK(sh2z_snd_queue_post(&q, (uint16_t)i, 0, 6u));
    CHECK(!sh2z_snd_queue_post(&q, 99u, 0, 6u) && q.dropped == 1u);
    for (uint32_t s = 1; s <= 3000u; ++s) CHECK(queue_script(s * 2654435761u + 1u, BREAK_NONE));
    /* controls: one rule broken in the reference must disagree */
    for (int b = BREAK_AFTER_CLOSE; b <= BREAK_LIFO; ++b) {
        int disagree = 0;
        for (uint32_t s = 1; s <= 200u && !disagree; ++s) disagree = !queue_script(s * 2654435761u + 1u, b);
        CHECK(disagree);
    }
}

/* ---- B. Voices: register decode and allocator reference ---- */
static uint16_t slots[SH2Z_SND_SLOTS * SH2Z_SND_SLOT_STRIDE];
#define R(slot, byte) slots[(slot) * SH2Z_SND_SLOT_STRIDE + ((byte) >> 1)]

static double rate_of(int oct, uint32_t fns) { return 44100.0 * pow(2.0, oct) * (1.0 + fns / 1024.0); }

static void check_slot(uint32_t s, const sh2z_snd_clip_t *c, uint8_t level, int8_t pan) {
    const uint16_t w0 = R(s, 0x00);
    CHECK((w0 & 0x0800u) != 0u);                                           /* keyed on */
    CHECK(((uint32_t)(w0 & 0xFu) << 16 | R(s, 0x02)) == (c->address & 0xFFFFFu));
    CHECK(((w0 >> 5) & 3u) == (c->loop ? 1u : 0u));
    CHECK(((w0 >> 4) & 1u) == c->pcm8);
    CHECK(R(s, 0x06) == c->loop_end && R(s, 0x04) == (c->loop ? c->loop_start : 0u));
    const int oct = (int)((R(s, 0x10) >> 11) & 0xFu) - (((R(s, 0x10) >> 14) & 1u) ? 16 : 0);
    CHECK(oct == c->oct && (R(s, 0x10) & 0x3FFu) == c->fns);
    const uint32_t dipan = (R(s, 0x16) >> 8) & 0x1Fu;
    const int got_pan = (dipan & 0x10u) ? -(int)(dipan & 0xFu) : (int)(dipan & 0xFu);
    CHECK(got_pan == (pan < -15 ? -15 : pan > 15 ? 15 : pan));
    CHECK((R(s, 0x16) >> 13) == 7u);
    /* level: TL within one step of the exact attenuation */
    const uint32_t tl = R(s, 0x0C) & 0xFFu;
    if (level == 0u) CHECK(tl == 255u);
    else {
        const double want = c->tl + 20.0 * log10(255.0 / level) / 0.375;
        CHECK(fabs((double)tl - (want > 255.0 ? 255.0 : want)) <= 1.0);
    }
}

enum { VBREAK_NONE, VBREAK_STEAL_HIGHER, VBREAK_YOUNGEST, VBREAK_NO_FREE_FIRST };
typedef struct { int clip; uint32_t left, started; uint8_t prio; } ref_voice_t;

static int ref_pick(ref_voice_t *v, uint32_t first, uint32_t count, uint8_t prio, int broken) {
    int pick = -1;
    if (broken != VBREAK_NO_FREE_FIRST)
        for (uint32_t s = first; s < first + count; ++s) if (v[s].clip < 0) return (int)s;
    for (uint32_t s = first; s < first + count; ++s) {
        if (v[s].clip < 0) continue;                                        /* only reached when broken */
        if (v[s].prio > prio && broken != VBREAK_STEAL_HIGHER) continue;
        if (pick < 0) { pick = (int)s; continue; }
        const int older = broken == VBREAK_YOUNGEST ? v[s].started > v[pick].started : v[s].started < v[pick].started;
        if (v[s].prio < v[pick].prio || (v[s].prio == v[pick].prio && older)) pick = (int)s;
    }
    if (pick < 0)
        for (uint32_t s = first; s < first + count; ++s) if (v[s].clip < 0) return (int)s;
    return pick;
}

static sh2z_snd_clip_t clips[8];

static int voice_script(uint32_t seed, int broken) {
    rng = seed;
    sh2z_snd_t snd; ref_voice_t ref[SH2Z_SND_SLOTS]; uint32_t plays = 0u;
    sh2z_snd_init(&snd, slots);
    for (uint32_t s = 0; s < SH2Z_SND_SLOTS; ++s) { ref[s].clip = -1; ref[s].left = 0u; ref[s].prio = 0u; ref[s].started = 0u; }
    for (uint32_t step = 0; step < 600u; ++step) {
        const uint32_t op = rnd() % 8u;
        if (op < 5u) {
            const int ci = (int)(rnd() % 8u);
            const uint32_t first = rnd() % 8u, count = 1u + rnd() % 6u;
            const uint8_t prio = (uint8_t)(rnd() % 4u), level = (uint8_t)(rnd() & 0xFFu);
            const int8_t pan = (int8_t)((int)(rnd() % 41u) - 20);
            const int got = (int)sh2z_snd_play(&snd, &clips[ci], first, count, prio, level, pan);
            const int want = ref_pick(ref, first, count, prio, broken);
            if (got != want) return 0;
            if (got >= 0) {
                check_slot((uint32_t)got, &clips[ci], level, pan);
                ref[got].clip = ci; ref[got].prio = prio; ref[got].started = ++plays;
                ref[got].left = clips[ci].loop ? 0u : (clips[ci].frames ? clips[ci].frames : 1u);
            }
        } else if (op == 5u) {
            const uint32_t s = rnd() % SH2Z_SND_SLOTS;
            sh2z_snd_stop(&snd, s); ref[s].clip = -1;
            if (R(s, 0x00) & 0x0800u) return 0;                            /* keyed off */
        } else {
            sh2z_snd_frame(&snd);
            for (uint32_t s = 0; s < SH2Z_SND_SLOTS; ++s)
                if (ref[s].clip >= 0 && !clips[ref[s].clip].loop && ref[s].left && --ref[s].left == 0u) ref[s].clip = -1;
        }
        for (uint32_t s = 0; s < SH2Z_SND_SLOTS; ++s)
            if ((snd.voice[s].clip != NULL) != (ref[s].clip >= 0) ||
                (ref[s].clip >= 0 && snd.voice[s].clip != &clips[ref[s].clip])) return 0;
    }
    return 1;
}

static void test_voices(void) {
    for (uint32_t i = 0; i < 8u; ++i) {
        clips[i].address = 0x20u + i * 0x9F31u;
        clips[i].loop = (uint8_t)(i & 1u);
        clips[i].loop_start = (uint16_t)(i * 37u);
        clips[i].loop_end = (uint16_t)(4000u + i * 911u);
        clips[i].oct = (int8_t)((int)i - 4);
        clips[i].fns = (uint16_t)(i * 131u);
        clips[i].tl = (uint8_t)(i * 9u);
        clips[i].pcm8 = (uint8_t)(i < 6u);
        clips[i].frames = (uint16_t)(1u + i * 3u);
    }
    for (uint32_t s = 1; s <= 2000u; ++s) CHECK(voice_script(s * 0x9E3779B1u + 7u, VBREAK_NONE));
    for (int b = VBREAK_STEAL_HIGHER; b <= VBREAK_NO_FREE_FIRST; ++b) {
        int disagree = 0;
        for (uint32_t s = 1; s <= 200u && !disagree; ++s) disagree = !voice_script(s * 0x9E3779B1u + 7u, b);
        CHECK(disagree);
    }
    /* out-of-range ranges are refused, not written */
    sh2z_snd_t snd; sh2z_snd_init(&snd, slots);
    CHECK(sh2z_snd_play(&snd, &clips[0], 31u, 2u, 0u, 255u, 0) == -1 && snd.refused == 1u);
    CHECK(sh2z_snd_play(&snd, &clips[0], 32u, 1u, 0u, 255u, 0) == -1);
    /* every level: TL within one step of the exact value */
    for (uint32_t l = 0; l < 256u; ++l) {
        const uint8_t tl = sh2z_snd_tl(&clips[0], (uint8_t)l);
        if (l == 0u) CHECK(tl == 255u);
        else CHECK(fabs(tl - 20.0 * log10(255.0 / l) / 0.375) <= 1.0);
    }
    /* pitch scale: the new rate within one FNS step of the exact one */
    for (uint32_t i = 0; i < 20000u; ++i) {
        sh2z_snd_clip_t c = clips[i & 7u];
        c.oct = (int8_t)((int)(rnd() % 7u) - 4);
        c.fns = (uint16_t)(rnd() & 0x3FFu);
        const uint32_t ratio = 0x4000u + rnd() % 0x3C000u;              /* 0.25x .. 4x */
        int8_t o; uint16_t f;
        const double want = rate_of(c.oct, c.fns) * ratio / 65536.0;
        const int ok = sh2z_snd_pitch_scale(&c, ratio, &o, &f);
        const double lo = 44100.0 / 256.0, hi = 44100.0 * 256.0;
        if (want < lo * 1.001 || want > hi * 0.999) continue;
        CHECK(ok);
        const double got = rate_of(o, f);
        CHECK(fabs(got - want) <= rate_of(o, 1u) - rate_of(o, 0u) + 1e-9);
    }
}

/* ---- C. CD player against a model CD block ---- */
static uint16_t cdregs[32];
static uint16_t scsp[SH2Z_SND_SLOTS * SH2Z_SND_SLOT_STRIDE];
typedef struct { uint32_t busy, status, play_left, violations, cmds; uint16_t log[16][4]; } cdblock_t;

/* One frame of the model block: finishes a command, advances play. Called before the player's frame. */
static void cd_model(cdblock_t *m) {
    if (!(cdregs[4] & 1u) && m->busy == 0u) {                              /* a command was written */
        uint16_t c[4] = { cdregs[12], cdregs[14], cdregs[16], cdregs[18] };
        if (m->cmds < 16u) memcpy(m->log[m->cmds], c, sizeof(c));
        ++m->cmds;
        m->busy = 1u + rnd() % 4u;
        if ((c[0] >> 8) == 0x10u) { m->status = 0x04u; m->play_left = c[3] & 0x3FFFu; }
        else if ((c[0] >> 8) == 0x11u) m->status = 0x01u;
    }
    if (m->busy && --m->busy == 0u) {
        cdregs[4] |= 1u;
        if (m->status == 0x04u) m->status = 0x03u;
    }
    if (m->status == 0x03u && m->play_left && --m->play_left == 0u) m->status = 0x01u;
    if (cdregs[4] & 1u) cdregs[12] = (uint16_t)(m->status << 8);
}

/* A player that ignores CMOK: the control for the busy-block rule. */
static void bad_send(sh2z_cdda_t *cd) {
    if (!cd->want) return;
    cdregs[4] = (uint16_t)~1u;
    cdregs[12] = cd->pending[0]; cdregs[14] = cd->pending[1]; cdregs[16] = cd->pending[2]; cdregs[18] = cd->pending[3];
    cd->want = 0u;
}

static void test_cdda(void) {
    uint32_t toc[102];
    for (uint32_t i = 0; i < 102u; ++i) toc[i] = 0xFFFFFFFFu;
    toc[0] = 0x41000096u; toc[1] = 0x01000505u;
    CHECK(sh2z_cdda_track_fad(toc, 1u) == 0x96u && sh2z_cdda_track_fad(toc, 2u) == 0x505u);
    CHECK(sh2z_cdda_track_fad(toc, 3u) == 0u && sh2z_cdda_track_fad(toc, 0u) == 0u && sh2z_cdda_track_fad(toc, 100u) == 0u);

    for (uint32_t run = 0; run < 500u; ++run) {
        memset(cdregs, 0, sizeof(cdregs)); cdregs[4] = 1u;
        cdblock_t m; memset(&m, 0, sizeof(m)); m.status = 0x02u;
        sh2z_cdda_t cd; sh2z_cdda_init(&cd, cdregs, scsp);
        CHECK(((scsp[16 * SH2Z_SND_SLOT_STRIDE + 0xB] >> 5) & 7u) == 7u);
        const uint32_t start = 1000u + rnd() % 5000u, n = 20u + rnd() % 60u, loop = start + 5u + rnd() % 10u, ln = 10u + rnd() % 30u;
        cdregs[4] = (uint16_t)((rnd() & 1u) ? 1u : 0u);                    /* sometimes busy at the start */
        if (!(cdregs[4] & 1u)) m.busy = 2u;
        sh2z_cdda_play(&cd, start, n, loop, ln);
        uint32_t frame = 0u;
        while (cd.state != SH2Z_CDDA_LOOPING && frame < 400u) { cd_model(&m); sh2z_cdda_frame(&cd); ++frame; }
        CHECK(cd.state == SH2Z_CDDA_LOOPING && cd.loop_backs == 1u && m.cmds == 2u);
        CHECK(m.log[0][0] == (0x1080u | (start >> 16)) && m.log[0][1] == (uint16_t)start && m.log[0][2] == 0x0080u && m.log[0][3] == n);
        CHECK(m.log[1][1] == (uint16_t)loop && m.log[1][2] == 0x0F80u && m.log[1][3] == ln);
        /* fade over F frames: levels never rise, 0 by the end, then a pause and quiet */
        const uint32_t f = rnd() % 40u;
        sh2z_cdda_fade(&cd, f);
        uint32_t prev = 7u, k = 0u;
        while (!sh2z_cdda_quiet(&cd) && k < 400u) {
            cd_model(&m); sh2z_cdda_frame(&cd); ++k;
            const uint32_t lvl = (scsp[16 * SH2Z_SND_SLOT_STRIDE + 0xB] >> 5) & 7u;
            CHECK(lvl <= prev); prev = lvl;
            if (k >= f) CHECK(lvl == 0u);
        }
        CHECK(sh2z_cdda_quiet(&cd) && m.cmds == 3u && (m.log[2][0] >> 8) == 0x11u);
        CHECK(k <= f + 20u);
    }
    /* control: writing commands without waiting for CMOK is seen by the model as a lost command */
    memset(cdregs, 0, sizeof(cdregs));
    cdblock_t m; memset(&m, 0, sizeof(m)); m.busy = 5u;
    sh2z_cdda_t cd; sh2z_cdda_init(&cd, cdregs, scsp);
    cd.want = 0u; cd.state = SH2Z_CDDA_IDLE;
    sh2z_cdda_play(&cd, 100u, 10u, 0u, 0u);                                /* the library waits: nothing written */
    CHECK(cd.want == 1u && cd.commands == 0u && cd.busy_frames == 1u);
    bad_send(&cd);
    cd_model(&m);
    CHECK(m.cmds == 0u);                                                   /* the busy block never took it */
}

int main(void) {
    test_queue();
    test_voices();
    test_cdda();
    printf("snd-1: %" PRIu64 " checks passed\n", checks);
    return 0;
}
