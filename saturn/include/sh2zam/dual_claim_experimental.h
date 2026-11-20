/* Experimental two-CPU try-claim: exactly one of two processors takes an
 * optional job, without an atomic instruction. Not selected by any default
 * dispatch.
 *
 * Contract tag: dual-claim-1. For each job tag, at most one side's claim
 * returns true, and when both sides claim the tag, exactly one does. A side
 * that does not want the job declines (side B) or simply does not claim
 * (side A); the other side's claim then returns true.
 *
 * Roles. Side A may wait: when it sees that B has started a claim on the same
 * tag, it spins until B has published its decision, which B does within a few
 * instructions of starting (B never waits). On the Saturn, A is the master
 * SH-2 and B the slave: the master reaches its own copy of the job late in its
 * frame and the slave takes it if it is idle first.
 *
 * Protocol (try-lock for two parties, tie to A):
 *   A: want_a = tag; if want_b != tag: A owns.
 *      else wait for decide to name tag; A owns iff B backed off.
 *   B: want_b = tag; if want_a == tag: decide = backed off, B does not own.
 *      else decide = owns, B owns.
 * If both read the other's word as unset, the later of the two stores would
 * have been seen by the later load, so that cannot happen; if A sees B's
 * word, B's decision tells A which case it is.
 *
 * Memory. The record must be read and written by both processors in program
 * order and without a stale copy: on the Saturn, place it where both CPUs
 * access it through the cache-through window (libyaul's .uncached section),
 * since the SH-2 caches do not snoop each other. Every access here is
 * volatile. The job's own data needs the same care, by the caller (a cache
 * purge or cache-through access after the owner reports completion).
 *
 * Tags: nonzero, below 2^31, and a new tag for every job (for example a frame
 * counter); a tag word left from an older job never matches a new one, so the
 * record needs no reset between jobs. No libc, no hardware access, no
 * allocation, no floating point. Counters are the caller's.
 *
 * Copyright (c) 2026 SH2ZAM Contributors. MIT License.
 * Inspired by SH4ZAM by Falco Girgis and SH4ZAM Contributors.
 */
#ifndef SH2ZAM_DUAL_CLAIM_EXPERIMENTAL_H
#define SH2ZAM_DUAL_CLAIM_EXPERIMENTAL_H

#include "sh2zam.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Contract tag dual-claim-1 ------------------------------------------- */

/** Largest job tag. */
#define SH2Z_CLAIM_TAG_MAX UINT32_C(0x7fffffff)

typedef struct {
    volatile uint32_t want_a; /**< written by A only: the tag A claims */
    volatile uint32_t want_b; /**< written by B only: the tag B claims */
    volatile uint32_t decide; /**< written by B only: tag * 2 + 1 B owns, tag * 2 B backed off */
} sh2z_claim_t;

/** All words zero (tag 0 is never used). */
void sh2z_claim_init(sh2z_claim_t *c);
/** Side A claims job `tag`; true when A owns it. May spin for B's decision. */
bool sh2z_claim_a(sh2z_claim_t *c, uint32_t tag);
/** Side B claims job `tag`; true when B owns it. Never waits. */
bool sh2z_claim_b(sh2z_claim_t *c, uint32_t tag);
/** Side B declines job `tag` (for example its deadline has passed): A's claim
 * on the tag then returns true without waiting, whatever the order. */
void sh2z_claim_b_decline(sh2z_claim_t *c, uint32_t tag);

#ifdef __cplusplus
}
#endif

#endif
