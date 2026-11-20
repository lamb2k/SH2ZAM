/* Experimental sound driver for the SH-2: sound events to SCSP slot writes,
 * with no 68000 program, and a CD audio player for the CD block. Not
 * selected by any default dispatch.
 *
 * Contract tag: snd-1. The caller names the register bases (the SCSP slot
 * registers, the SCSP common register, the CD block command registers), so
 * a host test can pass plain arrays. Every function is exact and
 * deterministic; none waits.
 *
 * A. Event queue (sh2z_snd_queue_t). Posts carry the poster's epoch. A queue
 *    is open for one epoch at a time: posts stamped with another epoch, or
 *    posted after sh2z_snd_queue_close(), are refused and counted
 *    (`rejected`). Posts accepted before the close are kept and delivered by
 *    the next drain. A full queue refuses and counts (`dropped`).
 *    sh2z_snd_queue_drain() delivers every queued event exactly once, oldest
 *    first, to a caller function; call it once per frame.
 * B. Voices (sh2z_snd_t). A clip is a PCM sample in sound RAM (8- or 16-bit,
 *    one-shot or looping) with its own pitch (OCT/FNS) and level (TL).
 *    sh2z_snd_play() picks a slot inside a caller range: a free slot first,
 *    otherwise the occupied slot with the lowest priority not above the new
 *    one, the oldest among equals. If none qualifies the play is refused
 *    (`refused`). It writes the slot's registers and keys it on.
 *    sh2z_snd_set_pitch() and sh2z_snd_set_level() change a playing slot;
 *    sh2z_snd_stop() keys it off. sh2z_snd_frame() ages one-shots by one
 *    frame and frees the slots whose clip has ended.
 * C. CD audio (sh2z_cdda_t). Plays a range of a disc track once, then a loop
 *    range forever (the loop restarts at a sector whose music matches the
 *    track end), from raw Play Disc commands written only when the CD block
 *    reports CMOK; a busy block gets the command at a later
 *    sh2z_cdda_frame(). sh2z_cdda_fade() ramps the CD input level (8 steps)
 *    to silence over a number of frames, then pauses the drive.
 *    sh2z_cdda_track_fad() reads a track's start from a Get TOC transfer.
 *
 * No function uses floating point, division, global state, libc or
 * allocation.
 *
 * Copyright (c) 2026 SH2ZAM Contributors. MIT License.
 * Inspired by SH4ZAM by Falco Girgis and SH4ZAM Contributors.
 */
#ifndef SH2ZAM_SND_EXPERIMENTAL_H
#define SH2ZAM_SND_EXPERIMENTAL_H

#include "sh2zam.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Contract tag snd-1 --------------------------------------------------- */

/* ---- A. Event queue ---- */
#define SH2Z_SND_QUEUE_MAX 32u

typedef struct {
    uint16_t id;     /* caller's event number */
    int16_t arg;     /* caller's argument (pan, a racer index ...) */
    uint32_t epoch;  /* the poster's epoch */
} sh2z_snd_event_t;

typedef struct {
    sh2z_snd_event_t events[SH2Z_SND_QUEUE_MAX];
    uint32_t head;      /* index of the oldest queued event */
    uint32_t count;     /* events queued */
    uint32_t epoch;     /* the epoch accepted while open */
    uint32_t open;      /* 1 between sh2z_snd_queue_open and sh2z_snd_queue_close */
    uint32_t rejected;  /* posts from another epoch or after the close */
    uint32_t dropped;   /* posts refused because the queue was full */
    uint32_t delivered; /* events handed to a drain function */
} sh2z_snd_queue_t;

/* Empties the queue and its counters; closed. */
void sh2z_snd_queue_init(sh2z_snd_queue_t *q);
/* Accepts posts stamped with `epoch` from now on. Queued events are kept. */
void sh2z_snd_queue_open(sh2z_snd_queue_t *q, uint32_t epoch);
/* Refuses every post from now on. Queued events are kept for the next drain. */
void sh2z_snd_queue_close(sh2z_snd_queue_t *q);
/* True when the event was queued. */
bool sh2z_snd_queue_post(sh2z_snd_queue_t *q, uint16_t id, int16_t arg, uint32_t epoch);
/* Hands every queued event, oldest first, to `deliver`; returns how many. */
uint32_t sh2z_snd_queue_drain(sh2z_snd_queue_t *q,
                              void (*deliver)(const sh2z_snd_event_t *event, void *ctx), void *ctx);

/* ---- B. Voices ---- */
#define SH2Z_SND_SLOTS 32u
#define SH2Z_SND_SLOT_STRIDE 0x10u      /* 16-bit registers per slot (0x20 bytes) */
#define SH2Z_SND_PAN_CENTRE 0           /* pan: -15 (left) .. 0 .. +15 (right) */

typedef struct {
    uint32_t address;      /* byte address in sound RAM */
    uint16_t loop_start;   /* samples */
    uint16_t loop_end;     /* samples; the clip's length when one-shot */
    int8_t oct;            /* pitch at its own rate: 44,100 * 2^oct * (1 + fns / 1024) Hz */
    uint16_t fns;          /* 0..1023 */
    uint8_t tl;            /* attenuation at full level, 0.375 dB steps */
    uint8_t loop;          /* 1: loops from loop_start to loop_end */
    uint8_t pcm8;          /* 1: signed 8-bit samples; 0: 16-bit */
    uint16_t frames;       /* one-shot length in 60 Hz frames (rounded up); unused when looping */
} sh2z_snd_clip_t;

typedef struct {
    const sh2z_snd_clip_t *clip; /* NULL: free */
    uint32_t frames_left;        /* one-shots: frames until the clip ends */
    uint32_t started;            /* play counter value at key on (age) */
    uint8_t priority;
} sh2z_snd_voice_t;

typedef struct {
    volatile uint16_t *slots;    /* SCSP slot registers (0x25B00000) */
    uint32_t mix_level;          /* direct send level of every play, 0..7 (DISDL, 6 dB steps); init sets 7 */
    sh2z_snd_voice_t voice[SH2Z_SND_SLOTS];
    uint32_t plays;              /* key-ons written */
    uint32_t refused;            /* plays with no slot to take */
    uint32_t stolen;             /* plays that took an occupied slot */
} sh2z_snd_t;

/* Takes the slot register base; clears every slot's registers and keys all off. */
void sh2z_snd_init(sh2z_snd_t *snd, volatile uint16_t *slots);
/* Returns the slot used, or -1 if refused. The clip plays at its own pitch; level: 0 (silent) .. 255
 * (the clip's own level); pan: -15 (left) .. 15 (right). */
int32_t sh2z_snd_play(sh2z_snd_t *snd, const sh2z_snd_clip_t *clip, uint32_t first_slot, uint32_t slot_count,
                      uint8_t priority, uint8_t level, int8_t pan);
/* Pitch of a playing slot, as OCT/FNS (engine pitch from speed). */
void sh2z_snd_set_pitch(sh2z_snd_t *snd, uint32_t slot, int8_t oct, uint16_t fns);
/* Level of a playing slot: 0 (silent) .. 255 (the clip's own level). */
void sh2z_snd_set_level(sh2z_snd_t *snd, uint32_t slot, uint8_t level);
/* Keys a slot off and frees it. */
void sh2z_snd_stop(sh2z_snd_t *snd, uint32_t slot);
/* One 60 Hz frame: frees one-shot slots whose clip has ended. */
void sh2z_snd_frame(sh2z_snd_t *snd);
/* The TL written for a clip at `level` (exposed for tests and callers that mix by hand). */
uint8_t sh2z_snd_tl(const sh2z_snd_clip_t *clip, uint8_t level);
/* OCT/FNS for a pitch ratio of the clip's own rate, as a 16.16 multiplier (0x10000 = unchanged,
 * 0x100 .. 0xFFFFFF accepted): the rate is within one FNS step of the exact one. Returns the OCT in
 * *oct and the FNS in *fns; false when the ratio or the result is out of range. */
bool sh2z_snd_pitch_scale(const sh2z_snd_clip_t *clip, uint32_t ratio_16_16, int8_t *oct, uint16_t *fns);

/* ---- C. CD audio ---- */
#define SH2Z_CDDA_LEVEL_MAX 7u

enum {
    SH2Z_CDDA_IDLE = 0,
    SH2Z_CDDA_START_SENT,
    SH2Z_CDDA_PLAYING,
    SH2Z_CDDA_LOOP_SENT,
    SH2Z_CDDA_LOOPING,
    SH2Z_CDDA_FADING,
    SH2Z_CDDA_PAUSE_SENT
};

typedef struct {
    volatile uint16_t *cd;       /* CD block registers (0x25890000): HIRQ at +0x08, CR1..CR4 at +0x18..+0x24 */
    volatile uint16_t *slots;    /* SCSP slot registers: the CD input level is slots 16 and 17, register 0x16 */
    uint16_t pending[4];         /* command waiting for CMOK */
    uint32_t want;               /* 1: pending is unsent */
    uint32_t state;
    uint32_t loop_fad, loop_sectors;
    uint32_t level;              /* current CD input level, 0..7 */
    uint32_t level_max;          /* the level plays start at (EFSDL, 6 dB steps); init sets 7 */
    uint32_t fade_frames, fade_left;
    uint32_t commands;           /* commands written */
    uint32_t busy_frames;        /* frames a command waited for CMOK */
    uint32_t loop_backs;         /* loop ranges started */
    uint32_t last_status;        /* periodic CD status code */
    uint32_t last_fad;           /* periodic pickup position */
} sh2z_cdda_t;

/* Takes the register bases and sets the CD input to full level. */
void sh2z_cdda_init(sh2z_cdda_t *cd, volatile uint16_t *cd_regs, volatile uint16_t *slots);
/* Start of track `track` (1..99) from a Get TOC transfer of 102 32-bit words; 0 if absent. */
uint32_t sh2z_cdda_track_fad(const uint32_t *toc, uint32_t track);
/* Plays sectors [start_fad, start_fad + sectors) once; then, if loop_sectors is not 0,
 * [loop_fad, loop_fad + loop_sectors) forever. */
void sh2z_cdda_play(sh2z_cdda_t *cd, uint32_t start_fad, uint32_t sectors, uint32_t loop_fad, uint32_t loop_sectors);
/* Ramps the CD input to silence over `frames` frames (0: at once), then pauses the drive. */
void sh2z_cdda_fade(sh2z_cdda_t *cd, uint32_t frames);
/* Once per frame: sends a waiting command when the block allows, follows the drive's status. */
void sh2z_cdda_frame(sh2z_cdda_t *cd);
/* True once a fade has paused the drive (or nothing is playing): the disc is free for reads. */
bool sh2z_cdda_quiet(const sh2z_cdda_t *cd);

#ifdef __cplusplus
}
#endif

#endif /* SH2ZAM_SND_EXPERIMENTAL_H */
