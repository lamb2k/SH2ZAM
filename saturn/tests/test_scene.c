/* Scene sequencing (scene-1): oracle and contract tests.
 *
 * The sequencer is driven by long random scripts (requests of every rank,
 * retirement confirmations at random times, preloads of random length,
 * enters that fail) and its callback trace and fade levels are compared,
 * frame by frame, with an independent reference model written as a plain
 * timeline. Each rule has a control: the same comparison against a
 * reference with that one rule broken must fail. Arenas and press edges are
 * checked against reference models in the same way.
 *
 * Copyright (c) 2026 SH2ZAM Contributors. MIT License.
 * Inspired by SH4ZAM by Falco Girgis and SH4ZAM Contributors.
 */
#include "sh2zam/scene_experimental.h"
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static uint64_t checks;
#define CHECK(e) do { ++checks; if (!(e)) { fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #e); exit(EXIT_FAILURE); } } while (0)
static uint32_t rng = UINT32_C(0x9e3779b9);
static uint32_t rnd(void) { rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5; return rng; }

/* ---- Arena ---- */
static void test_arena(void) {
    static uint8_t buffer[4096];
    sh2z_arena_t a;
    sh2z_arena_init(&a, buffer + 3, 1000u);
    /* worked example */
    uint8_t *p = sh2z_arena_alloc(&a, 10u, 1u);
    CHECK(p == buffer + 3 && a.used == 10u);
    uint8_t *q = sh2z_arena_alloc(&a, 4u, 16u);
    CHECK(((uintptr_t)q & 15u) == 0u && q >= p + 10 && q < p + 10 + 16);
    CHECK(sh2z_arena_alloc(&a, 1u, 3u) == NULL && a.refused == 1u);
    CHECK(sh2z_arena_alloc(&a, 1u, 0u) == NULL && a.refused == 2u);
    CHECK(sh2z_arena_alloc(&a, 1u, 8192u) == NULL && a.refused == 3u);
    CHECK(sh2z_arena_alloc(&a, 2000u, 1u) == NULL && a.refused == 4u);
    const uint32_t used = a.used;
    CHECK(sh2z_arena_alloc(&a, 1000u - used, 1u) != NULL && a.used == 1000u);
    CHECK(sh2z_arena_alloc(&a, 1u, 1u) == NULL);
    CHECK(sh2z_arena_alloc(&a, 0u, 1u) != NULL);
    sh2z_arena_reset(&a);
    CHECK(a.used == 0u && a.high == 1000u);
    /* random against a reference */
    for (int run = 0; run < 2000; ++run) {
        const uint32_t off = rnd() % 64u, size = rnd() % 3000u;
        sh2z_arena_init(&a, buffer + off, size);
        uintptr_t ref_used = 0, ref_high = 0;
        uint32_t ref_refused = 0;
        for (int k = 0; k < 64; ++k) {
            if (rnd() % 16u == 0u) { sh2z_arena_reset(&a); ref_used = 0; continue; }
            const uint32_t align = (rnd() % 8u == 0u) ? rnd() % 5000u : 1u << (rnd() % 13u);
            const uint32_t bytes = rnd() % 400u;
            void *r = sh2z_arena_alloc(&a, bytes, align);
            const bool ok_align = align && align <= 4096u && (align & (align - 1u)) == 0u;
            const uintptr_t at = (uintptr_t)(buffer + off) + ref_used;
            const uintptr_t al = ok_align ? (at + align - 1u) / align * align : 0u;
            const bool fits = ok_align && al - (uintptr_t)(buffer + off) + bytes <= size;
            if (!fits) { ++ref_refused; CHECK(r == NULL); }
            else {
                CHECK((uintptr_t)r == al);
                ref_used = al - (uintptr_t)(buffer + off) + bytes;
                if (ref_used > ref_high) ref_high = ref_used;
            }
            CHECK(a.used == ref_used && a.high == ref_high && a.refused == ref_refused);
        }
    }
    /* overflow: a range ending at the top of the address space */
    sh2z_arena_init(&a, (void *)(UINTPTR_MAX - 15u), 16u);
    CHECK(sh2z_arena_alloc(&a, 4u, 4096u) == NULL);
    CHECK(sh2z_arena_alloc(&a, UINT32_MAX, 1u) == NULL);
}

/* ---- Sequencer: trace ---- */
enum { EV_PRELOAD = 1, EV_ENTER, EV_TICK, EV_LEAVE, EV_RELEASE };
typedef struct { uint32_t ev[8], n; uint32_t level; } frame_trace_t;

typedef struct {
    frame_trace_t *t;
    uint32_t preload_len[8], preload_done[8];
    uint32_t fail_enter_at;   /* epoch-free counter: the n-th enter call fails (0 = never) */
    uint32_t enters;
} user_t;

static void rec(user_t *u, uint32_t ev, uint32_t scene) { if (u->t->n < 8u) u->t->ev[u->t->n++] = ev << 8 | scene; }
static bool cb_preload(void *p, uint32_t s) {
    user_t *u = p; rec(u, EV_PRELOAD, s);
    if (++u->preload_done[s] >= u->preload_len[s]) { u->preload_done[s] = 0u; return true; }
    return false;
}
static bool cb_enter(void *p, uint32_t s) { user_t *u = p; rec(u, EV_ENTER, s); return ++u->enters != u->fail_enter_at; }
static void cb_tick(void *p, uint32_t s) { rec(p, EV_TICK, s); }
static void cb_leave(void *p, uint32_t s) { rec(p, EV_LEAVE, s); }
static void cb_release(void *p, uint32_t s) { rec(p, EV_RELEASE, s); }

/* ---- Reference model: a timeline ---- */
enum { BREAK_NONE, BREAK_CONFIRM, BREAK_PREEMPT, BREAK_RANK, BREAK_FADE_TURN, BREAK_PRELOAD, BREAK_COUNT };
static const char *const break_name[BREAK_COUNT] = {
    "none", "retire waits for confirmation", "preempt rank", "higher rank replaces", "fade turns around",
    "preload before enter"
};

typedef struct {
    int brk;
    uint32_t count, fade, retire_min, preempt;
    uint32_t mode;          /* 0 shown, 1 out, 2 retire, 3 preload, 4 fault */
    uint32_t cur, dest, pend, pend_rank;
    uint32_t vis;           /* ENTER/RUN: fade-in frames done (fade = fully shown); OUT: fade-out frames done */
    uint32_t retire_n, confirmed;
    user_t *u;
    const uint32_t *preload_len;
} ref_t;

static uint32_t ref_fade_level_in(const ref_t *m) {
    return m->vis >= m->fade ? 0u : SH2Z_SCENE_FADE_MAX - m->vis * SH2Z_SCENE_FADE_MAX / m->fade;
}
static uint32_t ref_fade_level_out(const ref_t *m) {
    return m->vis >= m->fade ? SH2Z_SCENE_FADE_MAX : m->vis * SH2Z_SCENE_FADE_MAX / m->fade;
}

static bool ref_request(ref_t *m, uint32_t target, uint32_t rank) {
    if (m->mode == 4u || target >= m->count) return false;
    if (m->mode == 0u) {
        if (m->pend != SH2Z_SCENE_NONE && rank <= m->pend_rank && m->brk != BREAK_RANK) return false;
        if (m->pend != SH2Z_SCENE_NONE && m->brk == BREAK_RANK) return false;
        m->pend = target; m->pend_rank = rank; return true;
    }
    if ((rank < m->preempt && m->brk != BREAK_PREEMPT) || rank <= m->pend_rank) return false;
    m->pend = target; m->pend_rank = rank;
    if (m->dest != target) m->dest = target;
    return true;
}

typedef struct { uint32_t level; uint32_t ev[8], n; } ref_frame_t;
static void rpush(ref_frame_t *f, uint32_t ev, uint32_t s) { if (f->n < 8u) f->ev[f->n++] = ev << 8 | s; }

static uint32_t ref_preload_progress[8];

static void ref_frame(ref_t *m, ref_frame_t *f, uint32_t *enters_ref, uint32_t fail_at) {
    f->n = 0;
    if (m->mode == 0u) {
        if (m->pend != SH2Z_SCENE_NONE) {
            const uint32_t from = ref_fade_level_in(m);
            m->dest = m->pend;
            rpush(f, EV_LEAVE, m->cur);
            m->mode = 1u;
            m->vis = m->brk == BREAK_FADE_TURN ? 0u : from * m->fade / SH2Z_SCENE_FADE_MAX;
            goto out;
        }
        rpush(f, EV_TICK, m->cur);
        ++m->vis;
        f->level = ref_fade_level_in(m);
        return;
    }
    if (m->mode == 1u) {
    out:
        ++m->vis;
        f->level = ref_fade_level_out(m);
        if (m->vis >= m->fade) { m->mode = 2u; m->retire_n = 0u; m->confirmed = 0u; }
        return;
    }
    if (m->mode == 2u) {
        ++m->retire_n;
        f->level = SH2Z_SCENE_FADE_MAX;
        const bool ok = (m->confirmed || m->brk == BREAK_CONFIRM) && m->retire_n >= m->retire_min;
        if (!ok) return;
        rpush(f, EV_RELEASE, m->cur);
        m->mode = 3u;
    }
    if (m->mode == 3u) {
        f->level = SH2Z_SCENE_FADE_MAX;
        const uint32_t t = m->dest;
        bool done = true;
        if (m->preload_len[t] && m->brk != BREAK_PRELOAD) {
            rpush(f, EV_PRELOAD, t);
            done = ++ref_preload_progress[t] >= m->preload_len[t];
            if (done) ref_preload_progress[t] = 0u;
        }
        if (done) {
            m->pend = SH2Z_SCENE_NONE; m->pend_rank = 0u;
            m->cur = t; m->dest = SH2Z_SCENE_NONE;
            rpush(f, EV_ENTER, t);
            if (++*enters_ref == fail_at) { m->mode = 4u; return; }
            m->mode = 0u; m->vis = 0u;
        }
        return;
    }
    f->level = SH2Z_SCENE_FADE_MAX;
}

/* One scripted run; returns true if the reference matched every frame. */
static bool run_script(uint32_t seed, int brk, bool must_match) {
    rng = seed;
    const uint32_t count = 2u + rnd() % 6u;
    const uint32_t fade = 1u + rnd() % 12u;
    const uint32_t retire_min = rnd() % 4u;
    const uint32_t preempt = 2u + rnd() % 2u;
    static frame_trace_t trace;
    user_t u; memset(&u, 0, sizeof(u)); u.t = &trace;
    for (uint32_t i = 0; i < 8u; ++i) u.preload_len[i] = (rnd() % 3u == 0u) ? 0u : rnd() % 5u + 1u;
    u.fail_enter_at = (rnd() % 8u == 0u) ? 2u + rnd() % 20u : 0u;
    sh2z_scene_t table[8];
    for (uint32_t i = 0; i < count; ++i) {
        table[i].preload = u.preload_len[i] ? cb_preload : NULL;
        table[i].enter = cb_enter; table[i].tick = cb_tick; table[i].leave = cb_leave; table[i].release = cb_release;
        table[i].arenas = NULL; table[i].arena_count = 0u;
    }
    const uint32_t first = rnd() % count;
    sh2z_scene_seq_t seq;
    trace.n = 0;
    const bool started = sh2z_scene_init(&seq, table, count, first, fade, retire_min, preempt, &u);
    ref_t m; memset(&m, 0, sizeof(m));
    m.brk = brk; m.count = count; m.fade = fade; m.retire_min = retire_min; m.preempt = preempt;
    m.cur = first; m.dest = SH2Z_SCENE_NONE; m.pend = SH2Z_SCENE_NONE; m.u = &u; m.preload_len = u.preload_len;
    memset(ref_preload_progress, 0, sizeof(ref_preload_progress));
    uint32_t enters_ref = 1u;
    m.mode = (u.fail_enter_at == 1u) ? 4u : 0u;
    if (must_match) CHECK(started == (m.mode == 0u) && trace.n == 1u && trace.ev[0] == (EV_ENTER << 8 | first));
    bool match = true;
    for (int frame = 0; frame < 600 && match; ++frame) {
        /* random caller actions before the frame */
        const uint32_t r = rnd() % 100u;
        const uint32_t requests = r < 12u ? 1u + rnd() % 3u : 0u; /* sometimes several in one frame */
        for (uint32_t q = 0; q < requests; ++q) {
            const uint32_t target = rnd() % (count + 1u), rank = rnd() % 4u;
            const bool a = sh2z_scene_request(&seq, target, rank), b = ref_request(&m, target, rank);
            if (a != b) match = false;
        }
        if (rnd() % 3u == 0u) { sh2z_scene_retired(&seq); if (m.mode == 2u) m.confirmed = 1u; }
        trace.n = 0;
        const uint32_t lv = sh2z_scene_frame(&seq);
        ref_frame_t f; memset(&f, 0, sizeof(f));
        ref_frame(&m, &f, &enters_ref, u.fail_enter_at);
        if (lv != f.level || trace.n != f.n || memcmp(trace.ev, f.ev, f.n * 4u) != 0) match = false;
        static const uint32_t phase_of_mode[5] = { 9u, SH2Z_SCENE_LEAVE, SH2Z_SCENE_RETIRE, SH2Z_SCENE_PRELOAD, SH2Z_SCENE_FAULT };
        if (m.mode == 0u) { if (seq.phase != SH2Z_SCENE_ENTER && seq.phase != SH2Z_SCENE_RUN) match = false; }
        else if (seq.phase != phase_of_mode[m.mode]) match = false;
        if (seq.phase <= SH2Z_SCENE_RUN && seq.current != m.cur) match = false;
        if (must_match && !match) {
            fprintf(stderr, "seed %08" PRIx32 " frame %d: level %" PRIu32 "/%" PRIu32 " events %" PRIu32 "/%" PRIu32 " phase %" PRIu32 " mode %" PRIu32 "\n",
                    seed, frame, lv, f.level, trace.n, f.n, seq.phase, m.mode);
        }
        ++checks;
    }
    if (must_match) CHECK(match);
    return match;
}

static void test_sequencer_worked(void) {
    /* Scene 0 -> 1 with fade 4, retire min 2, no preload: exact levels and events. */
    static frame_trace_t trace;
    user_t u; memset(&u, 0, sizeof(u)); u.t = &trace;
    sh2z_scene_t table[2];
    for (int i = 0; i < 2; ++i) { table[i].preload = NULL; table[i].enter = cb_enter; table[i].tick = cb_tick;
        table[i].leave = cb_leave; table[i].release = cb_release; table[i].arenas = NULL; table[i].arena_count = 0u; }
    sh2z_scene_seq_t seq;
    CHECK(sh2z_scene_init(&seq, table, 2u, 0u, 4u, 2u, 3u, &u));
    CHECK(!sh2z_scene_init(&seq, table, 2u, 2u, 4u, 2u, 3u, &u));
    CHECK(!sh2z_scene_init(&seq, table, 2u, 0u, 0u, 2u, 3u, &u));
    CHECK(!sh2z_scene_init(&seq, table, 2u, 0u, SH2Z_SCENE_FADE_MAX + 1u, 2u, 3u, &u));
    CHECK(sh2z_scene_init(&seq, table, 2u, 0u, 4u, 2u, 3u, &u));
    const uint32_t in[4] = { 24u, 16u, 8u, 0u };
    for (int i = 0; i < 4; ++i) { trace.n = 0; CHECK(sh2z_scene_frame(&seq) == in[i]); CHECK(trace.n == 1u && trace.ev[0] == (EV_TICK << 8)); }
    CHECK(seq.phase == SH2Z_SCENE_RUN);
    CHECK(sh2z_scene_request(&seq, 1u, 1u));
    CHECK(!sh2z_scene_request(&seq, 0u, 1u));   /* equal rank does not replace */
    const uint32_t out[4] = { 8u, 16u, 24u, 32u };
    for (int i = 0; i < 4; ++i) { trace.n = 0; CHECK(sh2z_scene_frame(&seq) == out[i]); CHECK(trace.n == (i == 0 ? 1u : 0u)); }
    CHECK(seq.phase == SH2Z_SCENE_RETIRE);
    CHECK(!sh2z_scene_request(&seq, 0u, 2u));   /* below preempt */
    /* No confirmation: RETIRE holds however long. */
    for (int i = 0; i < 50; ++i) { trace.n = 0; CHECK(sh2z_scene_frame(&seq) == SH2Z_SCENE_FADE_MAX && trace.n == 0u); }
    sh2z_scene_retired(&seq);
    trace.n = 0;
    CHECK(sh2z_scene_frame(&seq) == SH2Z_SCENE_FADE_MAX);
    CHECK(trace.n == 2u && trace.ev[0] == (EV_RELEASE << 8 | 0u) && trace.ev[1] == (EV_ENTER << 8 | 1u));
    CHECK(seq.current == 1u && seq.phase == SH2Z_SCENE_ENTER && seq.epoch == 2u && seq.switches == 1u);
    /* A confirmation before RETIRE does not count. */
    for (int i = 0; i < 4; ++i) (void)sh2z_scene_frame(&seq);
    sh2z_scene_retired(&seq);
    CHECK(sh2z_scene_request(&seq, 0u, 1u));
    for (int i = 0; i < 4; ++i) (void)sh2z_scene_frame(&seq);
    CHECK(seq.phase == SH2Z_SCENE_RETIRE && seq.retired == 0u);
    /* A preempting request redirects the transition without restarting it. */
    CHECK(sh2z_scene_request(&seq, 1u, 3u) && seq.redirects == 1u && seq.next == 1u);
    sh2z_scene_retired(&seq);
    (void)sh2z_scene_frame(&seq); (void)sh2z_scene_frame(&seq);
    CHECK(seq.current == 1u);
}

/* ---- Press edges ---- */
static void test_input(void) {
    sh2z_scene_input_t in;
    sh2z_scene_input_init(&in, 0xF0u, 20u, 6u);
    CHECK(sh2z_scene_input_update(&in, 0x01u) == 0x01u);
    CHECK(sh2z_scene_input_update(&in, 0x01u) == 0u);          /* same reading, next step: no second edge */
    sh2z_scene_input_consume(&in);
    CHECK(sh2z_scene_input_update(&in, 0x03u) == 0x02u);       /* latched A gives nothing; B is new */
    CHECK(sh2z_scene_input_update(&in, 0x00u) == 0u);
    CHECK(sh2z_scene_input_update(&in, 0x01u) == 0x01u);       /* released, so A presses again */
    /* repeat: delay 20 then every 6 */
    sh2z_scene_input_init(&in, 0xF0u, 20u, 6u);
    uint32_t reps[80]; for (int i = 0; i < 80; ++i) { sh2z_scene_input_update(&in, 0x10u); reps[i] = in.repeat; }
    for (int i = 0; i < 80; ++i) {
        const bool want = i == 0 || (i >= 20 && (i - 20) % 6 == 0);
        CHECK((reps[i] == 0x10u) == want && (reps[i] == 0u) == !want);
    }
    /* random against a reference */
    for (int run = 0; run < 3000; ++run) {
        const uint32_t mask = rnd() & 0xFFu, delay = rnd() % 5u, rate = rnd() % 4u;
        sh2z_scene_input_init(&in, mask, delay, rate);
        const uint32_t d = delay ? delay : 1u, r = rate ? rate : 1u;
        uint32_t prev = 0, lat = 0, hold = 0;
        for (int k = 0; k < 200; ++k) {
            const uint32_t held = (rnd() % 4u == 0u) ? (rnd() & 0xFFu) : prev;
            lat &= held;
            const uint32_t e = held & ~prev & ~lat;
            const uint32_t dir = held & mask & ~lat, before = prev & mask & ~lat;
            hold = (dir == 0u || dir != before) ? 0u : hold + 1u;
            uint32_t rep = e;
            if (hold >= d && (hold - d) % r == 0u) rep |= dir;
            CHECK(sh2z_scene_input_update(&in, held) == e && in.repeat == rep && in.latched == lat);
            prev = held;
            if (rnd() % 10u == 0u) { sh2z_scene_input_consume(&in); lat |= held; hold = 0u; }
        }
    }
}

int main(void) {
    test_arena();
    test_sequencer_worked();
    test_input();
    for (uint32_t s = 1; s <= 4000u; ++s) (void)run_script(s * 2654435761u, BREAK_NONE, true);
    for (int b = 1; b < BREAK_COUNT; ++b) {
        bool caught = false;
        for (uint32_t s = 1; s <= 4000u && !caught; ++s) caught = !run_script(s * 2654435761u, b, false);
        if (!caught) { fprintf(stderr, "control not caught: %s\n", break_name[b]); return EXIT_FAILURE; }
        printf("control caught: %s\n", break_name[b]);
    }
    printf("scene_exact: %" PRIu64 " checks\n", checks);
    return 0;
}
