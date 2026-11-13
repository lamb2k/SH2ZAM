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
/* Extract the signed Q16 quotient with no signed shift, divide, or negation
 * of INT64_MIN. Both unsigned values converted below fit in int64_t. */
static inline int64_t sh2zi_floor16(int64_t v) {
    uint64_t bits=(uint64_t)v;
    return (int64_t)(bits>>16)-(int64_t)(bits>>63)*INT64_C(281474976710656);
}
/* Four signed raw products could overflow int64_t BEFORE Q16 extraction.
 * If sum(abs(a)) fits uint32_t, the absolute product sum is at most
 * (2^32-1)*2^31 < INT64_MAX. That common case can accumulate directly.
 * Otherwise split products into floor quotients + positive remainders.
 * The maximum quotient sum is 2^48, so cancellation stays exact without int128.
 * minus is a bit mask of terms to subtract; offset is already Q16.16. */
static inline int64_t sh2zi_dot(const int32_t *a,const int32_t *b,unsigned n,unsigned minus,int32_t offset) {
    uint32_t bound=0;
    unsigned i=0;
    for(;i<n;i++) {
        uint32_t magnitude=sh2zi_magnitude(a[i]);
        if(magnitude>UINT32_MAX-bound) break;
        bound+=magnitude;
    }
    if(i==n) {
        int64_t products=0;
        for(i=0;i<n;i++) {
            int64_t p=sh2zi_product(a[i],b[i]);
            products+=(minus & (1u<<i)) ? -p : p;
        }
        return sh2zi_floor16(products)+offset;
    }
    int64_t sum=offset; uint32_t rem=0;
    for(i=0;i<n;i++) {
        int64_t p=sh2zi_product(a[i],b[i]);
        if(minus & (1u<<i)) p=-p;
        int64_t q=sh2zi_floor16(p);
        sum+=q; rem+=(uint32_t)(p-q*65536);
    }
    return sum+(rem>>16);
}
static inline uint32_t sh2zi_isqrt32(uint32_t n) {
    uint32_t result=0,bit=UINT32_C(1)<<30;
    while(bit>n) bit>>=2;
    while(bit) {
        if(n>=result+bit) { n-=result+bit; result=(result>>1)+bit; }
        else result>>=1;
        bit>>=2;
    }
    return result;
}
/* Continue a 32-bit prefix square root with the sixteen low-word digits.
 * No 64-bit arithmetic remains inside either digit loop. The final digit
 * alone needs a high-word comparison; its remainder is not needed. */
static inline uint64_t sh2zi_isqrt(uint64_t n) {
    uint32_t hi=(uint32_t)(n>>32),lo=(uint32_t)n;
    if(!hi) return sh2zi_isqrt32(lo);
    uint32_t root=sh2zi_isqrt32(hi);
    uint32_t rem=hi-root*root;
    for(unsigned i=0;i<15;i++) {
        uint32_t carry=rem>>30;
        rem=(rem<<2)|(lo>>30); lo<<=2;
        uint32_t trial=(root<<2)|1u;
        root<<=1;
        if(carry || rem>=trial) { rem-=trial; root|=1u; }
    }
    uint32_t rem_hi=rem>>30;
    rem=(rem<<2)|(lo>>30);
    uint32_t trial_hi=root>>30,trial=(root<<2)|1u;
    root<<=1;
    if(rem_hi>trial_hi || (rem_hi==trial_hi && rem>=trial)) root|=1u;
    return root;
}
/* Exact sixteen-bit fractional quotient for value <= denominator <= 2^31.
 * Each doubled remainder fits uint32_t, including denominator == 2^31. */
static inline uint32_t sh2zi_fraction16(uint32_t value,uint32_t denominator) {
    if(value==denominator) return 65536;
    if(!value) return 0;
    uint32_t result=0;
    for(unsigned i=0;i<16;i++) {
        value<<=1; result<<=1;
        if(value>=denominator) { value-=denominator; result|=1u; }
    }
    return result;
}
/* A saturating signed 64/32 quotient needs at most 31 quotient bits. Reject
 * saturation before division, then use only 32-bit remainders. This avoids
 * the general 64/64 division helper on a 32-bit CPU. */
static inline int32_t sh2zi_div64_32(int64_t numerator,int32_t denominator) {
    if(!denominator) return numerator>0 ? INT32_MAX : (numerator<0 ? INT32_MIN : 0);
    bool negative=(numerator<0)!=(denominator<0);
    uint64_t magnitude=numerator<0 ? UINT64_C(0)-(uint64_t)numerator : (uint64_t)numerator;
    uint32_t divisor=sh2zi_magnitude(denominator);
    if(magnitude>=((uint64_t)divisor<<31)) return negative ? INT32_MIN : INT32_MAX;
    uint32_t hi=(uint32_t)(magnitude>>32),quotient=(uint32_t)magnitude;
    if(!hi) quotient/=divisor;
    else {
        uint32_t remainder=(hi<<1)|(quotient>>31);
        quotient<<=1;
        for(unsigned i=0;i<31;i++) {
            remainder=(remainder<<1)|(quotient>>31); quotient<<=1;
            if(remainder>=divisor) { remainder-=divisor; quotient|=1u; }
        }
    }
    return negative ? -(int32_t)quotient : (int32_t)quotient;
}
#endif
