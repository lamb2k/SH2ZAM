/* Internal arithmetic helpers. MIT; Copyright (c) 2026 SH2ZAM Contributors. */
#ifndef SH2ZAM_INTERNAL_H
#define SH2ZAM_INTERNAL_H
#include "sh2zam/sh2zam.h"
#include <limits.h>
#if defined(__sh__) && defined(__SH2__) && (!defined(SH2Z_FORCE_PORTABLE) || !SH2Z_FORCE_PORTABLE)
#define SH2Z_NATIVE_MUL 1
#else
#define SH2Z_NATIVE_MUL 0
#endif
#if defined(SH2Z_REQUIRE_SH2) && SH2Z_REQUIRE_SH2 && (!defined(__sh__) || !defined(__SH2__))
#error "SH2Z_REQUIRE_SH2 needs an SH-2 compiler target (-m2)"
#endif
#if defined(__sh__) && defined(__BYTE_ORDER__) && __BYTE_ORDER__ != __ORDER_BIG_ENDIAN__
#error "Saturn requires big endian (-mb)"
#endif
static inline sh2z_fx_t sh2zi_clamp(int64_t v) {
    return v>INT32_MAX ? INT32_MAX : (v<INT32_MIN ? INT32_MIN : (sh2z_fx_t)v);
}
static inline bool sh2zi_fits(int64_t v) { return v>=INT32_MIN && v<=INT32_MAX; }
static inline uint32_t sh2zi_magnitude(int32_t v) {
    return v<0 ? 0u-(uint32_t)v : (uint32_t)v;
}
static inline int64_t sh2zi_product(int32_t a,int32_t b) {
#if SH2Z_NATIVE_MUL
    int32_t hi; uint32_t lo;
    /* Single assembly block: compiler sees both clobbered MAC registers.
     * Application ISRs must preserve MACH/MACL when they use them. */
    __asm__ volatile("dmuls.l %[a], %[b]\n\tsts mach, %[hi]\n\tsts macl, %[lo]"
        : [hi] "=&r"(hi),[lo] "=&r"(lo) : [a] "r"(a),[b] "r"(b) : "mach","macl");
    return (int64_t)hi*INT64_C(4294967296)+(int64_t)lo;
#else
    return (int64_t)a*b;
#endif
}
/* Well-defined for INT64_MIN, without signed shifts or negating INT64_MIN. */
static inline int64_t sh2zi_floor16(int64_t v) {
    return v>=0 ? v/65536 : -1-((-1-v)/65536);
}
/* Four signed raw products could overflow int64_t BEFORE Q16 extraction.
 * Split each into floor quotient + positive remainder; sum both separately.
 * Maximum quotient sum is 2^48, so cancellation remains exact without int128.
 * minus is a bit mask of terms to subtract; offset is already Q16.16. */
static inline int64_t sh2zi_dot(const int32_t *a,const int32_t *b,unsigned n,unsigned minus,int32_t offset) {
    int64_t sum=offset; uint32_t rem=0;
    for(unsigned i=0;i<n;i++) {
        int64_t p=sh2zi_product(a[i],b[i]);
        if(minus & (1u<<i)) p=-p;
        int64_t q=sh2zi_floor16(p);
        sum+=q; rem+=(uint32_t)(p-q*65536);
    }
    return sum+(rem>>16);
}
static inline uint64_t sh2zi_isqrt(uint64_t n) {
    uint64_t result=0,bit=UINT64_C(1)<<62;
    while(bit>n) bit>>=2;
    while(bit) {
        if(n>=result+bit) { n-=result+bit; result=(result>>1)+bit; }
        else result>>=1;
        bit>>=2;
    }
    return result;
}
#endif
