/* Experimental explicit access to the SH-2 on-chip division unit (DIVU).
 * Not used by any default dispatch.
 *
 * Each SH-2 has its own divider, memory mapped at 0xFFFFFF00, which divides a
 * signed 64-bit dividend by a signed 32-bit divisor in about 39 cycles while
 * the CPU keeps executing. SH2ZAM never touches it implicitly: only the calls
 * in this header, and functions whose names end in _divu, use it.
 *
 * Caller promise, per CPU: between sh2z_divu_start64 and sh2z_divu_finish no
 * other code on that CPU uses the divider, unless every interrupt handler that
 * uses it brackets its own use with sh2z_divu_save / sh2z_divu_restore.
 * The library leaves DVCR (and its overflow-interrupt enable) untouched; keep
 * OVFIE clear, as it is at reset.
 *
 * Results never depend on the hardware's overflow or divide-by-zero
 * behaviour. A software check runs first; only quotients that provably fit
 * are sent to the divider. Everything else is answered in software:
 *
 *     result   = sat32(trunc(n / d))   for d != 0
 *              = n >= 0 ? INT32_MAX : INT32_MIN   for d == 0
 *     overflow = (d == 0 || |n| >= |d| * 2^31)
 *
 * so an exact -2^31 quotient is returned exactly but still reports overflow,
 * because it is the one value whose hardware handling is not established.
 * Portable builds (anything that is not SH-2) compute the same results in C.
 *
 * Hardware facts assumed, from the SH7604 hardware manual's division-unit
 * chapter, and not yet confirmed on a console: the register addresses below;
 * writing DVDNTL starts a 64/32 division of DVDNTH:DVDNTL by DVSR; the
 * quotient truncates toward zero; reading DVDNTL or DVDNTH before the
 * division completes holds the CPU until it does. The SH-2 model harness
 * implements the same assumptions on its side; it is not evidence for them.
 *
 * Copyright (c) 2026 SH2ZAM Contributors. MIT License.
 * Inspired by SH4ZAM by Falco Girgis and SH4ZAM Contributors.
 */
#ifndef SH2ZAM_DIVU_EXPERIMENTAL_H
#define SH2ZAM_DIVU_EXPERIMENTAL_H

#include "sh2zam.h"

#ifdef __cplusplus
extern "C" {
#endif

#if defined(__sh__) && defined(__SH2__) && (!defined(SH2Z_FORCE_PORTABLE) || !SH2Z_FORCE_PORTABLE)
#define SH2Z_DIVU_NATIVE 1
#else
#define SH2Z_DIVU_NATIVE 0
#endif

#define SH2Z_DIVU_DVSR_ADDRESS   UINT32_C(0xFFFFFF00)
#define SH2Z_DIVU_DVDNTH_ADDRESS UINT32_C(0xFFFFFF10)
#define SH2Z_DIVU_DVDNTL_ADDRESS UINT32_C(0xFFFFFF14)

/** One division in flight. Caller owned; holds the software answer when the
 *  divider is not used, so no global state exists. */
typedef struct {
    int32_t quotient;   /* valid when in_software != 0 */
    uint8_t in_software;
    uint8_t overflow;
    uint8_t reserved[2];
} sh2z_divu_t;

/** Divider registers saved by an interrupt handler. */
typedef struct { uint32_t dvsr,dvdnth,dvdntl; } sh2z_divu_state_t;

/** True when the quotient of hi:lo / divisor provably fits the divider:
 *  divisor != 0 and |n| < |divisor| * 2^31. Exact; no division. */
static inline bool sh2z_divu_fits(int32_t hi,uint32_t lo,int32_t divisor) {
    if(divisor==0)return false;
    /* |n| as two 32-bit words (no 64-bit shifts, which SH-2 GCC may call
     * libgcc for); n = -2^63 gives mh = 2^31. */
    uint32_t mh=(uint32_t)hi,ml=lo;
    if(hi<0) {ml=(uint32_t)0-lo;mh=~mh+(lo==0u?1u:0u);}
    const uint32_t d=divisor<0?(uint32_t)0-(uint32_t)divisor:(uint32_t)divisor;
    if(mh>=UINT32_C(0x80000000))return false;     /* |n| >> 31 >= 2^32 > |d| */
    return ((mh<<1)|(ml>>31))<d;                  /* (|n| >> 31) < |d| */
}

/** The software answer for a division that does not fit (see above). */
static inline int32_t sh2z_divu_saturated(int32_t hi,int32_t divisor) {
    return ((hi<0)!=(divisor<0) && divisor!=0)?INT32_MIN:
        (divisor==0 && hi<0)?INT32_MIN:INT32_MAX;
}

#if !SH2Z_DIVU_NATIVE
/* Portable quotient of a fitting division: trunc(n / d) from magnitudes. */
static inline int32_t sh2z_divu_portable_quotient(int32_t hi,uint32_t lo,int32_t divisor) {
    const uint64_t raw=((uint64_t)(uint32_t)hi<<32)|lo;
    const uint64_t a=hi<0?(uint64_t)0-raw:raw;
    const uint64_t d=divisor<0?(uint64_t)0-(uint64_t)(int64_t)divisor:(uint64_t)divisor;
    const uint32_t q=(uint32_t)(a/d);              /* < 2^31 because it fits */
    return ((hi<0)!=(divisor<0))?-(int32_t)q:(int32_t)q;
}
#endif

/** Start hi:lo / divisor. Uses the hardware only if sh2z_divu_fits. */
static inline void sh2z_divu_start64(sh2z_divu_t *t,int32_t hi,uint32_t lo,int32_t divisor) {
    if(!sh2z_divu_fits(hi,lo,divisor)) {
        t->quotient=sh2z_divu_saturated(hi,divisor);
        t->in_software=1;t->overflow=1;
        return;
    }
    t->overflow=0;
#if SH2Z_DIVU_NATIVE
    t->in_software=0;
    *(volatile uint32_t *)SH2Z_DIVU_DVSR_ADDRESS=(uint32_t)divisor;
    *(volatile uint32_t *)SH2Z_DIVU_DVDNTH_ADDRESS=(uint32_t)hi;
    *(volatile uint32_t *)SH2Z_DIVU_DVDNTL_ADDRESS=lo;     /* starts the division */
#else
    t->in_software=1;
    t->quotient=sh2z_divu_portable_quotient(hi,lo,divisor);
#endif
}

/** Collect the quotient started by t. overflow may be null. */
static inline int32_t sh2z_divu_finish(const sh2z_divu_t *t,bool *overflow) {
    if(overflow)*overflow=t->overflow!=0;
#if SH2Z_DIVU_NATIVE
    if(!t->in_software)return (int32_t)*(volatile uint32_t *)SH2Z_DIVU_DVDNTL_ADDRESS;
#endif
    return t->quotient;
}

/** For interrupt handlers that use the divider: call save first, restore
 *  last. Covers an interruption at any point of start64/finish, because
 *  restore leaves DVSR, DVDNTH and DVDNTL exactly as saved: it re-creates
 *  the saved DVDNTL by dividing it by one, then rewrites DVSR and DVDNTH.
 *  Costs two divider latencies; relies on the read-holds assumption above.
 *  No-ops in portable builds. */
static inline void sh2z_divu_save(sh2z_divu_state_t *s) {
#if SH2Z_DIVU_NATIVE
    s->dvdntl=*(volatile uint32_t *)SH2Z_DIVU_DVDNTL_ADDRESS;  /* waits for a running division */
    s->dvdnth=*(volatile uint32_t *)SH2Z_DIVU_DVDNTH_ADDRESS;
    s->dvsr=*(volatile uint32_t *)SH2Z_DIVU_DVSR_ADDRESS;
#else
    s->dvsr=s->dvdnth=s->dvdntl=0;
#endif
}
static inline void sh2z_divu_restore(const sh2z_divu_state_t *s) {
#if SH2Z_DIVU_NATIVE
    *(volatile uint32_t *)SH2Z_DIVU_DVSR_ADDRESS=1;
    *(volatile uint32_t *)SH2Z_DIVU_DVDNTH_ADDRESS=(s->dvdntl&UINT32_C(0x80000000))?UINT32_C(0xFFFFFFFF):0;
    *(volatile uint32_t *)SH2Z_DIVU_DVDNTL_ADDRESS=s->dvdntl;
    (void)*(volatile uint32_t *)SH2Z_DIVU_DVDNTL_ADDRESS;     /* wait for it */
    *(volatile uint32_t *)SH2Z_DIVU_DVSR_ADDRESS=s->dvsr;
    *(volatile uint32_t *)SH2Z_DIVU_DVDNTH_ADDRESS=s->dvdnth;
#else
    (void)s;
#endif
}

/** Blocking signed 64/32 division with the result rule above. overflow may
 *  be null. On SH-2 this uses the divider between its own start and finish. */
int32_t sh2z_divu_div64(int64_t n,int32_t divisor,bool *overflow);

/* ---- Contract tag trig12-1: inline fixed-point quotients ----
 *
 * sh2z_divu_q(num, den, f) is the quotient num * 2^f / den: with num and den
 * in the same format, the ratio in a format with f fraction bits (Q16.16 for
 * f = 16, Q2.30 for f = 30). It is the division above with the dividend
 * num * 2^f, so the result rule is the same, written for these operands:
 *
 *     result   = trunc(num * 2^f / den)          when |num| * 2^f < |den| * 2^31
 *              = num, den of different signs ? INT32_MIN : INT32_MAX
 *                                                 otherwise, den != 0
 *              = num >= 0 ? INT32_MAX : INT32_MIN when den == 0 (0 / 0 too)
 *     overflow = !(den != 0 && |num| * 2^f < |den| * 2^31)
 *
 * Rounding is toward zero on both signs (-7 / 2 is -3, not -4). An exact
 * quotient of -2^31 is returned exactly and flagged, as above. num = 0 with
 * den != 0 gives 0. 0 <= f <= 31; pass a constant f, because the SH-2 has no
 * variable shift (a variable f compiles to libgcc shift calls).
 * sh2z_divu_q16 equals sh2z_div for every input except 0 / 0, which sh2z_div
 * answers with 0; sh2z_recip_q16 equals sh2z_recip for every input.
 *
 * The fit test is |num| >> (31 - f) < |den| as unsigned 32-bit values (|den|
 * of zero never passes), which is exact and costs a few instructions. On
 * SH-2, sh2z_divu_q_start writes the divider and returns; sh2z_divu_finish
 * collects the quotient, so up to ~39 cycles of independent work can run in
 * between. The blocking forms wait. The caller promise at the top of this
 * header applies between start and finish. */

/** Start num * 2^frac_bits / den; collect it with sh2z_divu_finish. */
static inline void sh2z_divu_q_start(sh2z_divu_t *t,int32_t num,int32_t den,unsigned frac_bits) {
    const uint32_t an=num<0?(uint32_t)0-(uint32_t)num:(uint32_t)num;
    const uint32_t ad=den<0?(uint32_t)0-(uint32_t)den:(uint32_t)den;
    if((an>>(31u-frac_bits))>=ad) {
        t->quotient=sh2z_divu_saturated(num,den);
        t->in_software=1;t->overflow=1;
        return;
    }
    /* num * 2^f as hi:lo (two's complement), without signed shifts. */
    const uint64_t n=(uint64_t)(int64_t)num<<frac_bits;
    const uint32_t hi=(uint32_t)(n>>32),lo=(uint32_t)n;
    t->overflow=0;
#if SH2Z_DIVU_NATIVE
    t->in_software=0;
    *(volatile uint32_t *)SH2Z_DIVU_DVSR_ADDRESS=(uint32_t)den;
    *(volatile uint32_t *)SH2Z_DIVU_DVDNTH_ADDRESS=hi;
    *(volatile uint32_t *)SH2Z_DIVU_DVDNTL_ADDRESS=lo;     /* starts the division */
#else
    t->in_software=1;
    t->quotient=sh2z_divu_portable_quotient((int32_t)hi,lo,den);
#endif
}

/** Blocking num * 2^frac_bits / den (rule above). */
static inline int32_t sh2z_divu_q(int32_t num,int32_t den,unsigned frac_bits) {
    sh2z_divu_t t;
    sh2z_divu_q_start(&t,num,den,frac_bits);
    return sh2z_divu_finish(&t,NULL);
}
/** Ratio of two same-format values in Q16.16: num * 2^16 / den. */
static inline int32_t sh2z_divu_q16(int32_t num,int32_t den) {return sh2z_divu_q(num,den,16u);}
/** Ratio of two same-format values in Q2.30: num * 2^30 / den. */
static inline int32_t sh2z_divu_q30(int32_t num,int32_t den) {return sh2z_divu_q(num,den,30u);}
/** 1 / x for a Q16.16 x, in Q16.16: 2^32 / den. */
static inline int32_t sh2z_recip_q16(int32_t den) {return sh2z_divu_q(INT32_C(1)<<16,den,16u);}
/** 1 / x for a Q2.30 x, in Q2.30: 2^60 / den. Fits (no saturation) only for
 *  |den| > 2^29, that is |x| > 0.5. */
static inline int32_t sh2z_recip_q30(int32_t den) {return sh2z_divu_q(INT32_C(1)<<30,den,30u);}

#ifdef __cplusplus
}
#endif
#endif
