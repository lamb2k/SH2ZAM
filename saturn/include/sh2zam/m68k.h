/* 68000-exact integer arithmetic for code lifted from or modelled on the Motorola
 * 68000: MULS.W, MULU.W, DIVS.W, DIVU.W and the condition codes they define.
 * Game-neutral: no title, data or
 * hardware layout is assumed. Not part of SH4ZAM; there is no SH-4 path.
 *
 * Contract (all four operations)
 *  - Operands and results use the 68000's register views. dn is the full 32-bit
 *    data register; src is the 16-bit source operand (the caller passes the
 *    low word of whatever it read). Only the low word of dn is used by the
 *    multiplies; the divides use all 32 bits as the dividend.
 *  - The return value is the 68000 condition code nibble produced by the
 *    instruction: SH2Z_M68K_N | _Z | _V | _C. X is never touched by these
 *    instructions and is not reported. dn is written only when the instruction
 *    would write its destination.
 *  - Multiplies: dn = low16(dn) * src (signed or unsigned 16x16 -> 32). N is
 *    bit 31 of the result, Z is result == 0, V = C = 0.
 *  - Divides: quotient truncates toward zero; DIVS remainder takes the sign of
 *    the dividend (a zero remainder is +0). On success dn = remainder << 16 |
 *    quotient (both 16 bits), N is quotient bit 15, Z is quotient == 0,
 *    V = C = 0.
 *  - Overflow (quotient does not fit 16 bits: unsigned > 0xFFFF; signed outside
 *    -32768..32767): dn is unchanged and V is set. The 68000 manual lists N and
 *    Z as undefined after an overflow; this library reports N = 1, Z = 0, C = 0.
 *    That choice is deterministic and identical on every path here, but it is
 *    not checked against silicon: callers that need real-hardware N/Z after an
 *    overflow must verify them separately.
 *  - Division by zero: dn unchanged, SH2Z_M68K_DIV_ZERO is the only bit
 *    returned. Raising the 68000's divide-by-zero exception is the caller's
 *    decision; the library has no exception model.
 *  Every function is reentrant, allocation-free and uses no global state.
 *
 * Paths: on SH-2 (-m2 -mb) the multiplies compile to MULS.W/MULU.W + STS MACL,
 * which clobbers MACL (the compiler knows this); an ISR that uses MACL must
 * preserve it per your SDK ABI, as for the rest of the library. The divides use
 * the CPU's DIV0U/DIV1 steps (16 steps, no 64-bit libgcc divide), not the
 * memory-mapped on-chip divider: that divider is unqualified on hardware here,
 * its register and interrupt rules are caller-owned, and the project avoids
 * implicit divider access (see saturn/HARDWARE.md). The sequence changes SR.T,
 * SR.Q and SR.M (all compiler-visible: T is clobbered, Q/M are only live
 * inside the single asm block and are reset by DIV0U on entry); SR.S is not
 * changed. Elsewhere (or with SH2Z_FORCE_PORTABLE=1) plain C division is used.
 * sh2z_m68k_ref_* are always plain C, written independently of the fast path,
 * for tests and as the specification.
 *
 * Copyright (c) 2026 SH2ZAM Contributors. MIT License.
 * Inspired by SH4ZAM by Falco Girgis and SH4ZAM Contributors.
 */
#ifndef SH2ZAM_M68K_H
#define SH2ZAM_M68K_H
#include "sh2zam.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SH2Z_M68K_C 0x01u
#define SH2Z_M68K_V 0x02u
#define SH2Z_M68K_Z 0x04u
#define SH2Z_M68K_N 0x08u
#define SH2Z_M68K_DIV_ZERO 0x10u /**< Not a 68000 flag: the divisor was zero. */

#if defined(__sh__) && defined(__SH2__) && (!defined(SH2Z_FORCE_PORTABLE) || !SH2Z_FORCE_PORTABLE)
#define SH2Z_M68K_NATIVE 1
#else
#define SH2Z_M68K_NATIVE 0
#endif

/* Implementation detail. Unsigned 32/16 divide: requires 1 <= divisor <= 0xFFFF
 * and (dividend >> 16) < divisor, so the quotient fits 16 bits. Returns the
 * quotient. Native: the SH-1/SH-2 manual's unsigned 32/16 DIV1 sequence. */
static inline uint32_t sh2z_m68k_udiv32_16(uint32_t dividend,uint32_t divisor) {
#if SH2Z_M68K_NATIVE
    uint32_t shifted=divisor<<16;
#define SH2Z_M68K_STEP4 "div1 %[d], %[n]\n\tdiv1 %[d], %[n]\n\tdiv1 %[d], %[n]\n\tdiv1 %[d], %[n]\n\t"
    __asm__("div0u\n\t" SH2Z_M68K_STEP4 SH2Z_M68K_STEP4 SH2Z_M68K_STEP4 SH2Z_M68K_STEP4
            "rotcl %[n]\n\textu.w %[n], %[n]"
        : [n] "+r"(dividend) : [d] "r"(shifted) : "t");
#undef SH2Z_M68K_STEP4
    return dividend;
#else
    return dividend/divisor;
#endif
}

/* N/Z of a 32-bit or 16-bit result (N and Z are mutually exclusive). Written as
 * branches: GCC otherwise turns the sign test into a libgcc arithmetic-shift call. */
static inline uint32_t sh2z_m68k_nz32(uint32_t r) {
    if((int32_t)r<0) return SH2Z_M68K_N;
    if(r==0u) return SH2Z_M68K_Z;
    return 0u;
}
static inline uint32_t sh2z_m68k_nz16(uint32_t r) {
    if(r&0x8000u) return SH2Z_M68K_N;
    if(!(r&0xFFFFu)) return SH2Z_M68K_Z;
    return 0u;
}
/** MULU.W: dn = (dn & 0xFFFF) * src, unsigned. Returns N/Z (V = C = 0). */
static inline uint32_t sh2z_m68k_mulu_w(uint32_t *dn,uint16_t src) {
    const uint32_t r=(uint32_t)(uint16_t)*dn*(uint32_t)src;
    *dn=r;
    return sh2z_m68k_nz32(r);
}
/** MULS.W: dn = (int16)dn * (int16)src, signed. Returns N/Z (V = C = 0). */
static inline uint32_t sh2z_m68k_muls_w(uint32_t *dn,uint16_t src) {
    const int32_t r=(int32_t)(int16_t)(uint16_t)*dn*(int32_t)(int16_t)src;
    *dn=(uint32_t)r;
    return sh2z_m68k_nz32((uint32_t)r);
}
/** DIVU.W: unsigned dn / src; see the file contract. */
static inline uint32_t sh2z_m68k_divu_w(uint32_t *dn,uint16_t src) {
    const uint32_t n=*dn;
    if(!src) return SH2Z_M68K_DIV_ZERO;
    if((n>>16)>=src) return SH2Z_M68K_V|SH2Z_M68K_N;
    const uint16_t q=(uint16_t)sh2z_m68k_udiv32_16(n,src);
    const uint16_t r=(uint16_t)(n-(uint32_t)q*src);
    *dn=((uint32_t)r<<16)|q;
    return sh2z_m68k_nz16(q);
}
/** DIVS.W: signed dn / (int16)src; see the file contract. */
static inline uint32_t sh2z_m68k_divs_w(uint32_t *dn,uint16_t src) {
    const int32_t n=(int32_t)*dn,d=(int16_t)src;
    if(!d) return SH2Z_M68K_DIV_ZERO;
    const uint32_t un=n<0 ? 0u-(uint32_t)n : (uint32_t)n;
    const uint32_t ud=d<0 ? 0u-(uint32_t)d : (uint32_t)d;
    const bool negative=(n<0)!=(d<0);
    if((un>>16)>=ud) return SH2Z_M68K_V|SH2Z_M68K_N;
    const uint32_t q=sh2z_m68k_udiv32_16(un,ud);
    if(q>(negative ? 0x8000u : 0x7FFFu)) return SH2Z_M68K_V|SH2Z_M68K_N;
    const uint32_t r=un-q*ud;
    const uint16_t sq=(uint16_t)(negative ? 0u-q : q),sr=(uint16_t)(n<0 ? 0u-r : r);
    *dn=((uint32_t)sr<<16)|sq;
    return sh2z_m68k_nz16(sq);
}

/* Portable specification of the same four instructions: straightforward C with
 * the libc-style division operators, no shared code with the fast path above.
 * Same contract and same return value. */
uint32_t sh2z_m68k_ref_mulu_w(uint32_t *dn,uint16_t src);
uint32_t sh2z_m68k_ref_muls_w(uint32_t *dn,uint16_t src);
uint32_t sh2z_m68k_ref_divu_w(uint32_t *dn,uint16_t src);
uint32_t sh2z_m68k_ref_divs_w(uint32_t *dn,uint16_t src);

#ifdef __cplusplus
}
#endif
#endif
