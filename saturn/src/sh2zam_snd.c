/* Sound events to SCSP slots, and CD audio (snd_experimental.h, contract tag
 * snd-1). Portable C over caller-named register bases; no libc, no waits.
 *
 * SCSP slot registers used (16-bit words, slot stride 0x20 bytes):
 *   0x00 KYONEX(12) KYONB(11) LPCTL(6-5) PCM8B(4) SA[19:16](3-0)
 *   0x02 SA[15:0]   0x04 LSA   0x06 LEA
 *   0x08 AR=31, EGHOLD        0x0A RR=31
 *   0x0C TL(7-0)    0x10 OCT(14-11) FNS(9-0)
 *   0x16 DISDL(15-13) DIPAN(12-8) EFSDL(7-5) EFPAN(4-0)
 * CD block: HIRQ +0x08 (CMOK bit 0), CR1..CR4 +0x18, +0x1C, +0x20, +0x24.
 *
 * Copyright (c) 2026 SH2ZAM Contributors. MIT License.
 * Inspired by SH4ZAM by Falco Girgis and SH4ZAM Contributors.
 */
#include "sh2zam/snd_experimental.h"

/* ---- A. Event queue ---- */

void sh2z_snd_queue_init(sh2z_snd_queue_t *q) {
    q->head = 0u;
    q->count = 0u;
    q->epoch = 0u;
    q->open = 0u;
    q->rejected = 0u;
    q->dropped = 0u;
    q->delivered = 0u;
}

void sh2z_snd_queue_open(sh2z_snd_queue_t *q, uint32_t epoch) {
    q->epoch = epoch;
    q->open = 1u;
}

void sh2z_snd_queue_close(sh2z_snd_queue_t *q) {
    q->open = 0u;
}

bool sh2z_snd_queue_post(sh2z_snd_queue_t *q, uint16_t id, int16_t arg, uint32_t epoch) {
    if (!q->open || epoch != q->epoch) { ++q->rejected; return false; }
    if (q->count >= SH2Z_SND_QUEUE_MAX) { ++q->dropped; return false; }
    sh2z_snd_event_t *e = &q->events[(q->head + q->count) & (SH2Z_SND_QUEUE_MAX - 1u)];
    e->id = id;
    e->arg = arg;
    e->epoch = epoch;
    ++q->count;
    return true;
}

uint32_t sh2z_snd_queue_drain(sh2z_snd_queue_t *q,
                              void (*deliver)(const sh2z_snd_event_t *event, void *ctx), void *ctx) {
    /* Events posted by `deliver` itself wait for the next drain. */
    const uint32_t n = q->count;
    for (uint32_t i = 0; i < n; ++i) {
        const sh2z_snd_event_t e = q->events[q->head];
        q->head = (q->head + 1u) & (SH2Z_SND_QUEUE_MAX - 1u);
        --q->count;
        ++q->delivered;
        deliver(&e, ctx);
    }
    return n;
}

_Static_assert((SH2Z_SND_QUEUE_MAX & (SH2Z_SND_QUEUE_MAX - 1u)) == 0u, "the queue index wraps by mask");

/* ---- B. Voices ---- */

#define REG(s, slot, byte) ((s)[(slot) * SH2Z_SND_SLOT_STRIDE + ((byte) >> 1)])
#define KYONEX 0x1000u
#define KYONB 0x0800u
#define LOOP_NORMAL 0x0020u
#define PCM8B 0x0010u

/* 20 log10(255 / v) / 0.375 for v = 129.5, 133.5, ... 253.5 (inside each step of 4): every level
 * 1..255 lands within 0.91 TL steps of its exact attenuation (tests/test_snd.c). */
static const uint8_t tl_fraction[32] = { 16, 15, 14, 14, 13, 12, 12, 11, 11, 10, 9, 9, 8, 8, 7, 7,
                                         6, 6, 5, 5, 5, 4, 4, 3, 3, 2, 2, 2, 1, 1, 1, 0 };

uint8_t sh2z_snd_tl(const sh2z_snd_clip_t *clip, uint8_t level) {
    if (level == 0u) return 255u;
    uint32_t v = level, halvings = 0u;
    while (v < 128u) { v <<= 1; ++halvings; }
    const uint32_t tl = (uint32_t)clip->tl + 16u * halvings + tl_fraction[(v - 128u) >> 2];
    return (uint8_t)(tl > 255u ? 255u : tl);
}

static uint16_t pan_bits(int8_t pan) {
    if (pan < 0) return (uint16_t)(0x10u | (uint32_t)(pan < -15 ? 15 : -pan));
    return (uint16_t)(pan > 15 ? 15 : pan);
}

static uint16_t pitch_word(int8_t oct, uint16_t fns) {
    return (uint16_t)((((uint32_t)oct & 0xFu) << 11) | (fns & 0x3FFu));
}

static void key_execute(sh2z_snd_t *snd, uint32_t slot) {
    REG(snd->slots, slot, 0x00) = (uint16_t)(REG(snd->slots, slot, 0x00) | KYONEX);
}

void sh2z_snd_init(sh2z_snd_t *snd, volatile uint16_t *slots) {
    snd->slots = slots;
    snd->plays = 0u;
    snd->refused = 0u;
    snd->stolen = 0u;
    for (uint32_t s = 0; s < SH2Z_SND_SLOTS; ++s) {
        for (uint32_t r = 0; r < 0x18u; r += 2u) REG(slots, s, r) = 0u;
        snd->voice[s].clip = NULL;
        snd->voice[s].frames_left = 0u;
        snd->voice[s].started = 0u;
        snd->voice[s].priority = 0u;
    }
    key_execute(snd, 0u);
}

int32_t sh2z_snd_play(sh2z_snd_t *snd, const sh2z_snd_clip_t *clip, uint32_t first_slot, uint32_t slot_count,
                      uint8_t priority, uint8_t level, int8_t pan) {
    if (first_slot >= SH2Z_SND_SLOTS || slot_count == 0u || slot_count > SH2Z_SND_SLOTS - first_slot) {
        ++snd->refused;
        return -1;
    }
    int32_t pick = -1;
    for (uint32_t s = first_slot; s < first_slot + slot_count; ++s)
        if (!snd->voice[s].clip) { pick = (int32_t)s; break; }
    if (pick < 0) {
        /* Lowest priority not above the new one; the oldest among equals. */
        for (uint32_t s = first_slot; s < first_slot + slot_count; ++s) {
            const sh2z_snd_voice_t *v = &snd->voice[s];
            if (v->priority > priority) continue;
            if (pick < 0) { pick = (int32_t)s; continue; }
            const sh2z_snd_voice_t *b = &snd->voice[pick];
            if (v->priority < b->priority || (v->priority == b->priority && v->started < b->started))
                pick = (int32_t)s;
        }
        if (pick < 0) { ++snd->refused; return -1; }
        ++snd->stolen;
    }
    const uint32_t s = (uint32_t)pick;
    volatile uint16_t *r = snd->slots;
    /* Key off first so a stolen slot restarts from its new address. */
    REG(r, s, 0x00) = (uint16_t)(REG(r, s, 0x00) & ~KYONB);
    key_execute(snd, s);
    REG(r, s, 0x02) = (uint16_t)clip->address;
    REG(r, s, 0x04) = clip->loop ? clip->loop_start : 0u;
    REG(r, s, 0x06) = clip->loop_end;
    REG(r, s, 0x08) = 0x003Fu;                                   /* AR = 31, EGHOLD: full level at once */
    REG(r, s, 0x0A) = 0x001Fu;                                   /* RR = 31: stop at once on key off */
    REG(r, s, 0x0C) = sh2z_snd_tl(clip, level);
    REG(r, s, 0x0E) = 0u;
    REG(r, s, 0x10) = pitch_word(clip->oct, clip->fns);
    REG(r, s, 0x12) = 0u;
    REG(r, s, 0x14) = 0u;
    REG(r, s, 0x16) = (uint16_t)(7u << 13 | (uint32_t)pan_bits(pan) << 8);
    REG(r, s, 0x00) = (uint16_t)(KYONB | (clip->loop ? LOOP_NORMAL : 0u) | (clip->pcm8 ? PCM8B : 0u) |
                                 ((clip->address >> 16) & 0xFu));
    key_execute(snd, s);
    sh2z_snd_voice_t *v = &snd->voice[s];
    v->clip = clip;
    v->frames_left = clip->loop ? 0u : (clip->frames ? clip->frames : 1u);
    v->started = ++snd->plays;
    v->priority = priority;
    return pick;
}

void sh2z_snd_set_pitch(sh2z_snd_t *snd, uint32_t slot, int8_t oct, uint16_t fns) {
    if (slot >= SH2Z_SND_SLOTS || !snd->voice[slot].clip) return;
    REG(snd->slots, slot, 0x10) = pitch_word(oct, fns);
}

void sh2z_snd_set_level(sh2z_snd_t *snd, uint32_t slot, uint8_t level) {
    if (slot >= SH2Z_SND_SLOTS || !snd->voice[slot].clip) return;
    REG(snd->slots, slot, 0x0C) = sh2z_snd_tl(snd->voice[slot].clip, level);
}

void sh2z_snd_stop(sh2z_snd_t *snd, uint32_t slot) {
    if (slot >= SH2Z_SND_SLOTS) return;
    REG(snd->slots, slot, 0x00) = (uint16_t)(REG(snd->slots, slot, 0x00) & ~KYONB);
    key_execute(snd, slot);
    snd->voice[slot].clip = NULL;
    snd->voice[slot].frames_left = 0u;
}

void sh2z_snd_frame(sh2z_snd_t *snd) {
    for (uint32_t s = 0; s < SH2Z_SND_SLOTS; ++s) {
        sh2z_snd_voice_t *v = &snd->voice[s];
        if (v->clip && !v->clip->loop && v->frames_left && --v->frames_left == 0u) sh2z_snd_stop(snd, s);
    }
}

bool sh2z_snd_pitch_scale(const sh2z_snd_clip_t *clip, uint32_t ratio_16_16, int8_t *oct, uint16_t *fns) {
    if (ratio_16_16 < 0x100u || ratio_16_16 >= (1u << 24)) return false;
    /* p = (1024 + fns) * ratio, exact, in 2^-26 units; the mantissa is p >> e in [1024, 2048),
     * rounded once at the end. */
    const uint64_t p = (uint64_t)(1024u + (clip->fns & 0x3FFu)) * ratio_16_16;
    uint32_t e = 16u;
    int32_t o = clip->oct;
    while ((p >> e) >= 2048u) { ++e; ++o; }
    while ((p >> e) < 1024u) { --e; --o; }
    uint32_t m = (uint32_t)((p + ((uint64_t)1u << (e - 1u))) >> e);
    if (m >= 2048u) { m >>= 1; ++o; }
    if (o < -8 || o > 7) return false;
    *oct = (int8_t)o;
    *fns = (uint16_t)(m - 1024u);
    return true;
}

/* ---- C. CD audio ---- */

#define CD_HIRQ 4u
#define CD_CR1 12u
#define CD_CR2 14u
#define CD_CR3 16u
#define CD_CR4 18u
#define CMOK 0x0001u
#define FAD 0x800000u
#define STATUS_PAUSE 0x01u
#define STATUS_STANDBY 0x02u
#define STATUS_PLAY 0x03u
#define REPEAT_FOREVER 0x0Fu

static void cd_level(sh2z_cdda_t *cd, uint32_t level) {
    cd->level = level;
    REG(cd->slots, 16u, 0x16) = (uint16_t)(level << 5 | 0x1Fu);   /* CD left input, hard left */
    REG(cd->slots, 17u, 0x16) = (uint16_t)(level << 5 | 0x0Fu);   /* CD right input, hard right */
}

static void cd_command(sh2z_cdda_t *cd, uint16_t cr1, uint16_t cr2, uint16_t cr3, uint16_t cr4) {
    cd->pending[0] = cr1; cd->pending[1] = cr2; cd->pending[2] = cr3; cd->pending[3] = cr4;
    cd->want = 1u;
}

static void play_command(sh2z_cdda_t *cd, uint32_t fad, uint32_t sectors, uint32_t mode) {
    const uint32_t start = FAD | fad, end = FAD | sectors;
    cd_command(cd, (uint16_t)(0x1000u | ((start >> 16) & 0xFFu)), (uint16_t)start,
               (uint16_t)(mode << 8 | ((end >> 16) & 0xFFu)), (uint16_t)end);
}

static void cd_send(sh2z_cdda_t *cd) {
    if (!cd->want) return;
    if (!(cd->cd[CD_HIRQ] & CMOK)) { ++cd->busy_frames; return; }
    cd->cd[CD_HIRQ] = (uint16_t)~CMOK;
    cd->cd[CD_CR1] = cd->pending[0];
    cd->cd[CD_CR2] = cd->pending[1];
    cd->cd[CD_CR3] = cd->pending[2];
    cd->cd[CD_CR4] = cd->pending[3];
    cd->want = 0u;
    ++cd->commands;
}

void sh2z_cdda_init(sh2z_cdda_t *cd, volatile uint16_t *cd_regs, volatile uint16_t *slots) {
    cd->cd = cd_regs;
    cd->slots = slots;
    cd->want = 0u;
    cd->state = SH2Z_CDDA_IDLE;
    cd->loop_fad = 0u;
    cd->loop_sectors = 0u;
    cd->fade_frames = 0u;
    cd->fade_left = 0u;
    cd->commands = 0u;
    cd->busy_frames = 0u;
    cd->loop_backs = 0u;
    cd->last_status = 0u;
    cd_level(cd, SH2Z_CDDA_LEVEL_MAX);
}

uint32_t sh2z_cdda_track_fad(const uint32_t *toc, uint32_t track) {
    if (track < 1u || track > 99u) return 0u;
    const uint32_t e = toc[track - 1u];
    return e == 0xFFFFFFFFu ? 0u : (e & 0xFFFFFFu);
}

void sh2z_cdda_play(sh2z_cdda_t *cd, uint32_t start_fad, uint32_t sectors, uint32_t loop_fad, uint32_t loop_sectors) {
    cd->loop_fad = loop_fad;
    cd->loop_sectors = loop_sectors;
    cd->fade_left = 0u;
    cd_level(cd, SH2Z_CDDA_LEVEL_MAX);
    play_command(cd, start_fad, sectors, 0u);
    cd->state = SH2Z_CDDA_START_SENT;
    cd_send(cd);
}

void sh2z_cdda_fade(sh2z_cdda_t *cd, uint32_t frames) {
    if (cd->state == SH2Z_CDDA_IDLE || cd->state == SH2Z_CDDA_PAUSE_SENT) return;
    cd->want = 0u;                                               /* a waiting play is dropped */
    cd->fade_frames = frames;
    cd->fade_left = frames;
    cd->state = SH2Z_CDDA_FADING;
    if (!frames) cd_level(cd, 0u);
}

bool sh2z_cdda_quiet(const sh2z_cdda_t *cd) {
    return cd->state == SH2Z_CDDA_IDLE;
}

void sh2z_cdda_frame(sh2z_cdda_t *cd) {
    if (cd->state == SH2Z_CDDA_FADING) {
        if (cd->fade_left) {
            --cd->fade_left;
            /* level = 7 * left / frames, by comparison: no division */
            uint32_t level = SH2Z_CDDA_LEVEL_MAX;
            while (level && level * cd->fade_frames > SH2Z_CDDA_LEVEL_MAX * cd->fade_left) --level;
            if (level != cd->level) cd_level(cd, level);
        }
        if (!cd->fade_left) {
            if (cd->level) cd_level(cd, 0u);
            cd_command(cd, 0x11FFu, 0xFFFFu, 0u, 0u);             /* Seek to the pause position */
            cd->state = SH2Z_CDDA_PAUSE_SENT;
        }
    }
    cd_send(cd);
    if (cd->want || !(cd->cd[CD_HIRQ] & CMOK)) return;
    const uint32_t status = ((uint32_t)cd->cd[CD_CR1] >> 8) & 0x0Fu;
    cd->last_status = status;
    switch (cd->state) {
    case SH2Z_CDDA_START_SENT:
        if (status == STATUS_PLAY) cd->state = SH2Z_CDDA_PLAYING;
        break;
    case SH2Z_CDDA_LOOP_SENT:
        if (status == STATUS_PLAY) cd->state = SH2Z_CDDA_LOOPING;
        break;
    case SH2Z_CDDA_PLAYING:
        if (status == STATUS_PAUSE || status == STATUS_STANDBY) {
            if (!cd->loop_sectors) { cd->state = SH2Z_CDDA_IDLE; break; }
            play_command(cd, cd->loop_fad, cd->loop_sectors, REPEAT_FOREVER);
            ++cd->loop_backs;
            cd->state = SH2Z_CDDA_LOOP_SENT;
            cd_send(cd);
        }
        break;
    case SH2Z_CDDA_PAUSE_SENT:
        if (status == STATUS_PAUSE || status == STATUS_STANDBY) cd->state = SH2Z_CDDA_IDLE;
        break;
    default:
        break;
    }
}
