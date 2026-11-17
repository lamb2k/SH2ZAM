/* Portable 68000 arithmetic specification. MIT; Copyright (c) 2026 SH2ZAM Contributors. */
#include "sh2zam/m68k.h"

uint32_t sh2z_m68k_ref_mulu_w(uint32_t *dn,uint16_t src) {
    uint64_t r=(uint64_t)(*dn&0xFFFFu)*src;
    *dn=(uint32_t)r;
    return ((r&0x80000000u)?SH2Z_M68K_N:0u)|(r==0?SH2Z_M68K_Z:0u);
}
uint32_t sh2z_m68k_ref_muls_w(uint32_t *dn,uint16_t src) {
    int32_t a=(*dn&0x8000u)?(int32_t)(*dn&0xFFFFu)-65536:(int32_t)(*dn&0xFFFFu);
    int32_t b=(src&0x8000u)?(int32_t)src-65536:(int32_t)src;
    int64_t r=(int64_t)a*b;
    *dn=(uint32_t)((uint64_t)r&0xFFFFFFFFu);
    return (r<0?SH2Z_M68K_N:0u)|(r==0?SH2Z_M68K_Z:0u);
}
uint32_t sh2z_m68k_ref_divu_w(uint32_t *dn,uint16_t src) {
    if(src==0) return SH2Z_M68K_DIV_ZERO;
    uint32_t q=*dn/src,r=*dn%src;
    if(q>0xFFFFu) return SH2Z_M68K_V|SH2Z_M68K_N;
    *dn=(r<<16)|q;
    return ((q&0x8000u)?SH2Z_M68K_N:0u)|(q==0?SH2Z_M68K_Z:0u);
}
uint32_t sh2z_m68k_ref_divs_w(uint32_t *dn,uint16_t src) {
    int64_t n=(int32_t)*dn;
    int64_t d=(src&0x8000u)?(int64_t)src-65536:(int64_t)src;
    if(d==0) return SH2Z_M68K_DIV_ZERO;
    int64_t q=n/d,r=n%d; /* C99: truncates toward zero, r takes n's sign */
    if(q>32767||q<-32768) return SH2Z_M68K_V|SH2Z_M68K_N;
    *dn=(((uint32_t)(int32_t)r&0xFFFFu)<<16)|((uint32_t)(int32_t)q&0xFFFFu);
    return ((q<0)?SH2Z_M68K_N:0u)|(q==0?SH2Z_M68K_Z:0u);
}
