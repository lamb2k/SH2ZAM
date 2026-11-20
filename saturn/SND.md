# Sound events and CD audio (snd-1, experimental)

`sh2zam/snd_experimental.h` drives the Saturn's sound from the SH-2 with no
68000 program: sound events go into a queue, a once-per-frame drain turns them
into SCSP slot writes, and CD audio plays from the disc through the SCSP's CD
input. The caller names the register bases, so host tests use plain arrays.

## Why no 68000 program

- **Sample playback is register writes.** A slot plays a PCM clip from sound
  RAM once its address, loop, pitch and level are written and it is keyed on.
- **The 68000 can't reach main RAM or the cartridge.** Clips are copied into
  sound RAM by the SH-2, so the SH-2 already touches every sample.
- **Event rates are low.** A few events per frame cost tens of register writes.
- **So the 68000 is parked:** the caller writes a one-instruction idle loop at
  its reset vector and releases it, and sound RAM above that is all clip space.

## Contract

- **A. Queue** (`sh2z_snd_queue_t`):
  - **Epochs.** Posts carry the poster's epoch. The queue accepts one epoch at
    a time, between `sh2z_snd_queue_open` and `sh2z_snd_queue_close`.
  - **Refusals.** A post from another epoch, or after the close, is refused and
    counted (`rejected`). A full queue refuses and counts (`dropped`).
  - **Delivery.** `sh2z_snd_queue_drain` delivers each queued event exactly
    once, oldest first, including events accepted before a close. So a confirm
    sound posted on the press that starts a transition plays once, and a sound
    posted by a scene after it left never plays.
- **B. Voices** (`sh2z_snd_t`, `sh2z_snd_clip_t`):
  - **Clip.** A clip is 8- or 16-bit PCM in sound RAM, one-shot or looping,
    with its own pitch (OCT/FNS) and level (TL).
  - **Slot choice.** `sh2z_snd_play` takes a slot from a caller range: a free
    one first; otherwise the occupied slot with the lowest priority not above
    the new one, the oldest among equals; otherwise it refuses.
  - **Writes.** It writes address, loop, envelope (instant attack and
    release), TL, pitch and direct pan, and keys the slot on.
  - **Level.** Level 0..255 maps to TL within 0.91 steps (0.34 dB) of the
    exact attenuation.
  - **Pitch.** `sh2z_snd_pitch_scale` gives OCT/FNS for a 16.16 ratio of the
    clip's own rate, within one FNS step of the exact rate. An engine's pitch
    follows its speed through this and `sh2z_snd_set_pitch`.
  - **End of a one-shot.** `sh2z_snd_frame` counts one-shots down by their
    length in frames and frees their slots.
- **C. CD audio** (`sh2z_cdda_t`):
  - **Play.** `sh2z_cdda_play` plays a sector range once, then a loop range
    with repeat forever. The drive returns to a sector whose music matches the
    track end, so the join is continuous.
  - **Commands.** They are raw Play Disc writes, made only while the CD block
    reports CMOK. `sh2z_cdda_frame` retries a waiting command and follows the
    drive's periodic status. Nothing waits.
  - **Fade.** `sh2z_cdda_fade` ramps the CD input level (8 steps, EFSDL of
    slots 16 and 17) to silence over N frames, then pauses the drive.
    `sh2z_cdda_quiet` reports when the disc is free for data reads.
  - **TOC.** `sh2z_cdda_track_fad` reads a track's start from a Get TOC
    transfer.

## Tests (`tests/test_snd.c`)

- **Queue and slot allocator.** Each is checked against an independent
  reference over thousands of random scripts.
- **Slot writes.** Every write is decoded back from the register array and
  compared with the request: address, loop, format, pitch, pan, key and TL
  bound.
- **CD player.** It runs against a model CD block that takes commands after a
  random delay and loses any command written while it is busy.
- **Controls.** Each rule has one: a reference with the rule broken must
  disagree. The rules are posts after close, other epochs, events dropped at
  close, delivery order, stealing a higher priority, the youngest among
  equals, ignoring free slots first, and writing without CMOK.
