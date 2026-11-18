/* Fused fade lookup and 5:5:5 colour per vertex batch with prepared fog and
 * colour handles (contract tag shade-2, shade_experimental.h), in C and, for
 * SH-2, in hand-written assembly with identical results.
 * MIT; SH2ZAM Contributors. */
#include "sh2zam/shade_experimental.h"
#include "sh2zam/cdiv_experimental.h"
#include "sh2zam_sections.h"

#if defined(__sh__) && defined(__SH2__) && (!defined(SH2Z_FORCE_PORTABLE) || !SH2Z_FORCE_PORTABLE)
#define SH2ZI_FG_NATIVE 1
#else
#define SH2ZI_FG_NATIVE 0
#endif

#define NB SH2Z_SHADE_FOGP_BUCKETS

static bool overlaps(const void *a,size_t a_bytes,const void *b,size_t b_bytes) {
    const uintptr_t x=(uintptr_t)a,y=(uintptr_t)b;
    return x<y+b_bytes && y<x+a_bytes;
}

/* ---- prepared fog ---- */

bool sh2z_shade_fogp_prepare(sh2z_shade_fogp_t *out,const sh2z_shade_fog_t *fog) {
    if(!out || !fog || fog->depth[0] || overlaps(out,sizeof(*out),fog,sizeof(*fog)))return false;
    const uint32_t *d=fog->depth;
    for(unsigned i=1;i<256;++i)if(d[i]<d[i-1u])return false;
    const uint32_t top=d[255]>UINT32_C(0x7FFFFFFF)?UINT32_C(0x7FFFFFFF):d[255];
    /* top >> shift < NB, as top >> 12 < step = 2^shift (no variable shift). */
    unsigned shift=1;
    uint32_t step=2,scale=UINT32_C(0x80000000);
    while((top>>12)>=step){++shift;step<<=1;scale>>=1;}   /* shift <= 19 */
    for(unsigned i=0;i<256;++i)out->depth[i]=d[i];
    /* bucket[b] = max i with depth[i] <= b << shift, in one merge pass. */
    uint32_t z=0;
    unsigned i=0;
    for(unsigned b=0;b<=NB;++b,z+=step) {
        while(i<255u && d[i+1u]<=z)++i;
        out->bucket[b]=(uint8_t)i;
    }
    out->bucket[NB+1]=out->bucket[NB+2]=out->bucket[NB+3]=0;
    out->scale=scale;
    out->shift=(uint8_t)shift;
    out->reserved[0]=out->reserved[1]=out->reserved[2]=0;
    return true;
}

/* ---- colour ramps ---- */

static bool thresholds_ok(const uint16_t threshold[2]) {
    return !threshold || (threshold[0]<=threshold[1] && threshold[1]<=256u);
}
static void ramp_common(sh2z_shade_ramp_t *out,const uint16_t threshold[2],unsigned form) {
    out->threshold[0]=(uint16_t)(threshold?threshold[0]:256u);
    out->threshold[1]=(uint16_t)(threshold?threshold[1]:256u);
    out->form=(uint8_t)form;
}

bool sh2z_shade_ramp_offset(sh2z_shade_ramp_t *out,const uint8_t live[3],const uint8_t base[3],
    const uint8_t level[3],const uint16_t threshold[2]) {
    if(!out || !live || !base || !level || !thresholds_ok(threshold))return false;
    for(unsigned k=0;k<3;++k)if(live[k]>127u || base[k]>127u)return false;
    for(unsigned f=0;f<256;++f)out->mix[f]=0;
    for(unsigned k=0;k<3;++k) {
        out->live[k]=(int16_t)(257*live[k]);
        out->base[k]=(int16_t)(257*base[k]);
        out->level[k]=level[k];
    }
    ramp_common(out,threshold,SH2Z_SHADE_RAMP_OFFSET);
    return true;
}

bool sh2z_shade_ramp_mix(sh2z_shade_ramp_t *out,const uint8_t near_rgb[3],const uint8_t far_rgb[3],
    const uint16_t threshold[2]) {
    if(!out || !near_rgb || !far_rgb || !thresholds_ok(threshold))return false;
    const uint32_t n0=near_rgb[0],n1=near_rgb[1],n2=near_rgb[2],f0=far_rgb[0],f1=far_rgb[1],f2=far_rgb[2];
    for(uint32_t f=0;f<256u;++f) {
        const uint32_t g=255u-f;
        const uint32_t v0=sh2z_cdiv65025_u32((n0*g+f0*f)*31u+32512u);
        const uint32_t v1=sh2z_cdiv65025_u32((n1*g+f1*f)*31u+32512u);
        const uint32_t v2=sh2z_cdiv65025_u32((n2*g+f2*f)*31u+32512u);
        out->mix[f]=(uint16_t)(0x8000u|v0|(v1<<5)|(v2<<10));
    }
    for(unsigned k=0;k<3;++k){out->live[k]=out->base[k]=0;out->level[k]=0;}
    ramp_common(out,threshold,SH2Z_SHADE_RAMP_MIX);
    return true;
}

/* ---- the OFFSET channel table ----
 *
 * x = fade * slope - bias, |x| <= 255 * 127 + 255 * 127 = 64770 in the ramp's
 * domain. For channel k the kernel forms
 *     X = 257 * x + 128 + 65536 * (OFF + SEG * k)        (0 < X < 2^31)
 * as fade * (257 * slope) + C (one 16 x 16 multiply; |257 * slope| <= 32639)
 * and reads sh2zi_offset_clamp[X >> 16]. With x = 255 q + r (0 <= r <= 254),
 * 257 x = 65536 q - q + 257 r, so (X >> 16) - OFF - SEG * k is exactly q while
 * 0 <= 257 r - q + 128 < 65536, that is for -130 < q <= 128, and within one of
 * q for the other q (|q| <= 254). Entry j of segment k is the packed channel
 * for q = j - OFF: clamp(16 + q + [q < 0], 0, 31) << 5k, with bit 15 in
 * segment 0. 16 + q + [q < 0] equals 16 + trunc(x / 255) except when x is a
 * negative multiple of 255 (then it is one more); for |q| > 128 both clamp to
 * the same end. tests/test_shade.c checks every x. */
#define OFF 256
#define SEG 512
#define CH(j) ((j)<OFF?((j)<OFF-17?0u:(unsigned)((j)-OFF+17)):((j)>OFF+15?31u:(unsigned)((j)-OFF+16)))
#define E0(j) (0x8000u|CH(j))
#define E1(j) (CH(j)<<5)
#define E2(j) (CH(j)<<10)
#define R4(E,j) E(j),E((j)+1),E((j)+2),E((j)+3)
#define R16(E,j) R4(E,j),R4(E,(j)+4),R4(E,(j)+8),R4(E,(j)+12)
#define R64(E,j) R16(E,j),R16(E,(j)+16),R16(E,(j)+32),R16(E,(j)+48)
#define R256(E,j) R64(E,j),R64(E,(j)+64),R64(E,(j)+128),R64(E,(j)+192)
#define R512(E) R256(E,0),R256(E,256)
/* Global (internal name) so the assembly below can address it. */
extern const uint16_t sh2zi_offset_clamp[3*SEG];
const uint16_t sh2zi_offset_clamp[3*SEG] SH2ZI_RODATA(sh2zi_offset_clamp)={R512(E0),R512(E1),R512(E2)};

/* ---- fused kernel, C ---- */

#if !SH2ZI_FG_NATIVE
static unsigned search(const uint32_t depth[256],uint32_t z,unsigned lo,unsigned hi) {
    /* max i in [lo, hi] with depth[i] <= z, given depth[lo] <= z. */
    while(lo<hi) {
        const unsigned mid=(lo+hi+1u)>>1;
        if(depth[mid]<=z)lo=mid;else hi=mid-1u;
    }
    return lo;
}
static inline unsigned fade_of(const sh2z_shade_fogp_t *fog,uint32_t scale,int32_t v) {
    /* b = u >> shift as the high word of u * 2^(32 - shift). A negative z
     * reads as u >= 2^31, beyond the table (shift <= 19), and has the fade
     * of depth 0; a positive one is beyond it only when it is past every
     * reachable threshold. */
    const uint32_t b=(uint32_t)(((uint64_t)(uint32_t)v*scale)>>32);
    if(b>=NB)return v<0?fog->bucket[0]:255u;
    const unsigned lo=fog->bucket[b],hi=fog->bucket[b+1u];
    return lo==hi?lo:search(fog->depth,(uint32_t)v,lo,hi);
}

/* Depth i: strided from z, or at byte offset z_stride of refs[i]. */
static inline int32_t depth_at(const void *z,size_t z_stride,bool refs,size_t i) {
    const char *p=refs?(const char *)((const void *const *)z)[i]+z_stride:(const char *)z+i*z_stride;
    return *(const int32_t *)(const void *)p;
}
static int fog_gouraud_c(uint16_t *out,const void *z,size_t z_stride,bool refs,size_t count,
    const sh2z_shade_fogp_t *fog,const sh2z_shade_ramp_t *ramp,const uint8_t mean[3]) {
    const uint32_t scale=fog->scale;
    uint32_t sum=0;
    if(ramp->form==SH2Z_SHADE_RAMP_MIX) {
        const uint16_t *mix=ramp->mix;
        for(size_t i=0;i<count;++i) {
            const unsigned f=fade_of(fog,scale,depth_at(z,z_stride,refs,i));
            sum+=f;out[i]=mix[f];
        }
    } else {
        for(size_t i=0;i<count;++i) {
            const unsigned f=fade_of(fog,scale,depth_at(z,z_stride,refs,i));
            sum+=f;out[i]=(uint16_t)f;
        }
    }
    const uint32_t n=(uint32_t)count;
    const int variant=(sum>=ramp->threshold[0]*n)+(sum>=ramp->threshold[1]*n);
    if(ramp->form==SH2Z_SHADE_RAMP_OFFSET) {
        const int32_t level=ramp->level[variant];
        int32_t s[3],c[3];
        for(unsigned k=0;k<3;++k) {
            const int32_t m=257*(int32_t)mean[k];
            s[k]=ramp->live[k]-m;                                         /* 257 * slope */
            c[k]=128+65536*(OFF+SEG*(int32_t)k)-level*(ramp->base[k]-m);  /* - 257 * bias */
        }
        for(size_t i=0;i<count;++i) {
            const int32_t f=out[i];
            const uint32_t x0=(uint32_t)(f*s[0]+c[0]),x1=(uint32_t)(f*s[1]+c[1]),x2=(uint32_t)(f*s[2]+c[2]);
            out[i]=(uint16_t)(sh2zi_offset_clamp[x0>>16]|sh2zi_offset_clamp[x1>>16]|sh2zi_offset_clamp[x2>>16]);
        }
    }
    return variant;
}
#else
/* ---- fused kernel, SH-2 ----
 *
 * Both public functions in full, argument checks included. The same two
 * passes as the C: fades (or MIX words) into out with their sum, the
 * variant, then for OFFSET the three channel words per fade. count == 4 with
 * the OFFSET form (a road piece) runs unrolled with the four fades in
 * registers. Per depth: one load (three for the refs form), DMULU.L for the
 * bucket, two byte loads and one compare; the bounded search of a bucket
 * holding thresholds (rare) is an out-of-line subroutine. Per OFFSET
 * colour: three MULS.W, each followed by an add, SHLR16, a doubling and one
 * table load. The body is assembled once per depth form (strided S, refs R),
 * each in its own section with its own copy of the exits and the search.
 * Callee-saved r8-r14, MACH and MACL are saved and restored (MACH also
 * carries the variant through the unrolled colours); PR is pushed only
 * around the search call; GBR is not touched. */
__asm__(
/* Depth i into r1 and its bucket product into MACH: r5 the depth pointer
 * (strided) or the next ref (refs), r6 the stride or the z offset, r9 scale. */
"    .macro SH2ZI_FG_LD ref\n"
"    .if \\ref\n"
"    mov.l   @r5+,r1\n"
"    add     r6,r1\n"
"    mov.l   @r1,r1\n"
"    dmulu.l r1,r9\n"
"    .else\n"
"    mov.l   @r5,r1\n"
"    dmulu.l r1,r9\n"
"    add     r6,r5\n"
"    .endif\n"
"    .endm\n"
/* One depth of the counted loop: r10 the bucket count, r11 bucket, r12
 * bucket + 1, r13 sum, r4 out, r7 count left; mix=1 stores mix[fade] (r14 =
 * mix words) instead of the fade. */
"    .macro SH2ZI_FG_P1 tag, mix, ref\n"
".Lfg_p1\\tag:\n"
"    SH2ZI_FG_LD \\ref\n"
"    sts     mach,r0\n"             /* bucket */
"    cmp/hs  r10,r0\n"
"    bt      .Lfg_far\\tag\n"
"    mov.b   @(r0,r11),r2\n"         /* lo */
"    mov.b   @(r0,r12),r3\n"         /* hi */
"    cmp/eq  r2,r3\n"
"    bf      .Lfg_srch\\tag\n"
"    extu.b  r2,r2\n"
".Lfg_got\\tag:\n"
"    add     r2,r13\n"
"    .if \\mix\n"
"    mov     r2,r0\n"
"    add     r0,r0\n"
"    mov.w   @(r0,r14),r2\n"
"    .endif\n"
"    mov.w   r2,@r4\n"
"    dt      r7\n"
"    bf/s    .Lfg_p1\\tag\n"
"    add     #2,r4\n"
"    bra     .Lfg_end\\tag\n"
"    nop\n"
".Lfg_far\\tag:\n"                   /* negative z: fade(0); past the table: 255 */
"    cmp/pz  r1\n"
"    bt/s    .Lfg_f1\\tag\n"
"    mov     #-1,r2\n"
"    mov.b   @r11,r2\n"
".Lfg_f1\\tag:\n"
"    bra     .Lfg_got\\tag\n"
"    extu.b  r2,r2\n"
".Lfg_srch\\tag:\n"
"    sts.l   pr,@-r15\n"
"    bsr     .Lfg_search\\ref\n"
"    nop\n"
"    lds.l   @r15+,pr\n"
"    bra     .Lfg_got\\tag\n"
"    nop\n"
".Lfg_end\\tag:\n"
"    .endm\n"
/* One depth of the unrolled path into register dst (not r0-r3). */
"    .macro SH2ZI_FG_Q tag, dst, ref\n"
"    SH2ZI_FG_LD \\ref\n"
"    sts     mach,r0\n"
"    cmp/hs  r10,r0\n"
"    bt      .Lfq_far\\tag\n"
"    mov.b   @(r0,r11),r2\n"
"    mov.b   @(r0,r12),r3\n"
"    cmp/eq  r2,r3\n"
"    bf      .Lfq_srch\\tag\n"
"    extu.b  r2,\\dst\n"
".Lfq_got\\tag:\n"
"    .endm\n"
/* Its out-of-line cases, placed within branch range after the lookups. */
"    .macro SH2ZI_FG_QS tag, dst, ref\n"
".Lfq_far\\tag:\n"
"    cmp/pz  r1\n"
"    bt/s    .Lfq_f1\\tag\n"
"    mov     #-1,r2\n"
"    mov.b   @r11,r2\n"
".Lfq_f1\\tag:\n"
"    bra     .Lfq_got\\tag\n"
"    extu.b  r2,\\dst\n"
".Lfq_srch\\tag:\n"
"    sts.l   pr,@-r15\n"
"    bsr     .Lfg_search\\ref\n"
"    nop\n"
"    lds.l   @r15+,pr\n"
"    bra     .Lfq_got\\tag\n"
"    mov     r2,\\dst\n"
"    .endm\n"
/* One OFFSET colour from fade f, stored at out + disp (r4): S0 r6, C0 r5,
 * S1 r9, C1 r10, S2 r11, C2 r12, table r3. */
"    .macro SH2ZI_FG_C f, disp\n"
"    muls.w  \\f,r6\n"
"    sts     macl,r0\n"
"    add     r5,r0\n"
"    shlr16  r0\n"
"    add     r0,r0\n"
"    mov.w   @(r0,r3),r1\n"
"    muls.w  \\f,r9\n"
"    sts     macl,r0\n"
"    add     r10,r0\n"
"    shlr16  r0\n"
"    add     r0,r0\n"
"    mov.w   @(r0,r3),r2\n"
"    muls.w  \\f,r11\n"
"    or      r2,r1\n"
"    sts     macl,r0\n"
"    add     r12,r0\n"
"    shlr16  r0\n"
"    add     r0,r0\n"
"    mov.w   @(r0,r3),r0\n"
"    or      r1,r0\n"
"    mov.w   r0,@(\\disp,r4)\n"
"    .endm\n"
/* Channel constants into S and C: m = mean[k] (r2 = mean), M = 257 m,
 * S = 257 live - M, C = K - level * (257 base - M); r3 ramp, r1 level. */
"    .macro SH2ZI_FG_K k, s, c, kconst\n"
"    mov.b   @(\\k,r2),r0\n"
"    mov     r0,\\c\n"
"    shll8   \\c\n"
"    add     r0,\\c\n"
"    mov.w   @(8+2*\\k,r3),r0\n"
"    sub     \\c,r0\n"
"    mov     r0,\\s\n"
"    mov.w   @(14+2*\\k,r3),r0\n"
"    sub     \\c,r0\n"
"    muls.w  r0,r1\n"
"    mov.l   \\kconst,\\c\n"
"    sts     macl,r0\n"
"    sub     r0,\\c\n"
"    .endm\n"
/* Argument checks (r4 out, r5 z or refs, r6 z_stride or z_offset, r7 count;
 * @r15 fog, @(4,r15) ramp, @(8,r15) mean); falls through when accepted. */
"    .macro SH2ZI_FG_CHECK sfx\n"
"    tst     r7,r7\n"
"    bt/s    .Lfg_zero\\sfx\n"
"    mov     #0,r0\n"
"    mov.l   @r15,r1\n"
"    tst     r4,r4\n"
"    bt      .Lfg_bad\\sfx\n"
"    tst     r5,r5\n"
"    bt      .Lfg_bad\\sfx\n"
"    tst     r1,r1\n"
"    bt      .Lfg_bad\\sfx\n"
"    mov.l   @(4,r15),r2\n"
"    tst     r2,r2\n"
"    bt      .Lfg_bad\\sfx\n"
"    mov     r7,r0\n"
"    or      r6,r0\n"
"    shlr16  r0\n"
"    tst     r0,r0\n"
"    bf      .Lfg_bad\\sfx\n"          /* count or stride >= 65536 */
"    mov     r5,r0\n"
"    or      r6,r0\n"
"    tst     #3,r0\n"
"    bf      .Lfg_bad\\sfx\n"
"    mov     r4,r0\n"
"    tst     #1,r0\n"
"    bf      .Lfg_bad\\sfx\n"
"    mov.b   @r2,r0\n"
"    cmp/eq  #2,r0\n"
"    bt      .Lfg_pass\\sfx\n"
"    cmp/eq  #1,r0\n"
"    bf      .Lfg_bad\\sfx\n"
"    mov.l   @(8,r15),r3\n"
"    tst     r3,r3\n"
"    bt      .Lfg_bad\\sfx\n"
"    mov.b   @r3,r1\n"
"    mov.b   @(1,r3),r0\n"
"    or      r0,r1\n"
"    mov.b   @(2,r3),r0\n"
"    or      r1,r0\n"
"    tst     #128,r0\n"
"    bf      .Lfg_bad\\sfx\n"
".Lfg_pass\\sfx:\n"
"    .endm\n"
/* Everything after the checks, for one depth form. */
"    .macro SH2ZI_FG_BODY sfx, ref\n"
"    mov.l   r8,@-r15\n"
"    mov.l   r9,@-r15\n"
"    mov.l   r10,@-r15\n"
"    mov.l   r11,@-r15\n"
"    mov.l   r12,@-r15\n"
"    mov.l   r13,@-r15\n"
"    mov.l   r14,@-r15\n"
"    sts.l   mach,@-r15\n"
"    sts.l   macl,@-r15\n"             /* fog @(36,r15), ramp @(40,r15), mean @(44,r15) */
"    mov.l   @(36,r15),r14\n"
"    mov.l   @r14,r9\n"                /* scale */
"    mov.l   .Lfg_bk\\sfx,r11\n"
"    add     r14,r11\n"                /* bucket */
"    mov     r11,r12\n"
"    add     #1,r12\n"                 /* bucket + 1 */
"    mov     #16,r10\n"
"    shll8   r10\n"                    /* 4096 buckets */
"    mov.l   @(40,r15),r14\n"          /* ramp */
"    mov.b   @r14,r0\n"
"    cmp/eq  #2,r0\n"
"    bt/s    .Lfg_tmix\\sfx\n"
"    mov     r7,r8\n"                  /* count */
"    mov     r7,r0\n"
"    cmp/eq  #4,r0\n"
"    bt/s    .Lfq_go\\sfx\n"
"    mov     #0,r13\n"                 /* sum */
"    bra     .Lfg_gen\\sfx\n"
"    nop\n"
".Lfg_tmix\\sfx:\n"
"    bra     .Lfg_mix\\sfx\n"
"    mov     #0,r13\n"
/* count == 4, OFFSET: the fades stay in r7, r8, r13, r14 */
".Lfq_go\\sfx:\n"
"    SH2ZI_FG_Q 0\\sfx, r7, \\ref\n"
"    SH2ZI_FG_Q 1\\sfx, r8, \\ref\n"
"    SH2ZI_FG_Q 2\\sfx, r13, \\ref\n"
"    SH2ZI_FG_Q 3\\sfx, r14, \\ref\n"
"    bra     .Lfq_sum\\sfx\n"
"    mov     r7,r1\n"
"    SH2ZI_FG_QS 0\\sfx, r7, \\ref\n"
"    SH2ZI_FG_QS 1\\sfx, r8, \\ref\n"
"    SH2ZI_FG_QS 2\\sfx, r13, \\ref\n"
"    SH2ZI_FG_QS 3\\sfx, r14, \\ref\n"
".Lfq_sum\\sfx:\n"
"    add     r8,r1\n"
"    add     r13,r1\n"
"    add     r14,r1\n"                 /* sum */
"    mov.l   @(40,r15),r3\n"           /* ramp */
"    mov.w   @(4,r3),r0\n"
"    shll2   r0\n"                     /* 4 threshold[0] */
"    cmp/hs  r0,r1\n"
"    movt    r2\n"
"    mov.w   @(6,r3),r0\n"
"    shll2   r0\n"
"    cmp/hs  r0,r1\n"
"    movt    r0\n"
"    add     r0,r2\n"                  /* variant */
"    mov     r2,r0\n"
"    add     #1,r0\n"
"    mov.b   @(r0,r3),r1\n"
"    lds     r2,mach\n"                /* variant kept in MACH (MULS.W writes MACL only) */
"    extu.b  r1,r1\n"                  /* level[variant] */
"    mov.l   @(44,r15),r2\n"           /* mean */
"    SH2ZI_FG_K 0, r6, r5, .Lfq_k0\\sfx\n"
"    SH2ZI_FG_K 1, r9, r10, .Lfq_k1\\sfx\n"
"    SH2ZI_FG_K 2, r11, r12, .Lfq_k2\\sfx\n"
"    mov.l   .Lfq_tab\\sfx,r3\n"
"    SH2ZI_FG_C r7, 0\n"
"    SH2ZI_FG_C r8, 2\n"
"    SH2ZI_FG_C r13, 4\n"
"    SH2ZI_FG_C r14, 6\n"
"    bra     .Lfg_ret\\sfx\n"
"    sts     mach,r0\n"
"    .align 2\n"
".Lfg_bk\\sfx:\n"
"    .long   1032\n"
".Lfq_k0\\sfx:\n"
"    .long   128+65536*256\n"
".Lfq_k1\\sfx:\n"
"    .long   128+65536*(256+512)\n"
".Lfq_k2\\sfx:\n"
"    .long   128+65536*(256+1024)\n"
".Lfq_tab\\sfx:\n"
"    .long   _sh2zi_offset_clamp\n"
/* MIX, any count: mix[fade] straight into out */
".Lfg_mix\\sfx:\n"
"    add     #20,r14\n"                /* mix words */
"    SH2ZI_FG_P1 mx\\sfx, 1, \\ref\n"
"    bra     .Lfg_var\\sfx\n"
"    nop\n"
/* OFFSET, count != 4: fades into out, then the colours over them */
".Lfg_gen\\sfx:\n"
"    SH2ZI_FG_P1 st\\sfx, 0, \\ref\n"
".Lfg_var\\sfx:\n"
"    mov.l   @(40,r15),r3\n"           /* ramp */
"    mov.w   @(4,r3),r0\n"
"    mulu.w  r0,r8\n"                  /* threshold[0] * count */
"    mov.w   @(6,r3),r0\n"
"    sts     macl,r1\n"
"    mulu.w  r0,r8\n"                  /* threshold[1] * count */
"    cmp/hs  r1,r13\n"
"    movt    r2\n"
"    sts     macl,r1\n"
"    cmp/hs  r1,r13\n"
"    movt    r1\n"
"    add     r1,r2\n"                  /* variant */
"    mov.b   @r3,r0\n"
"    cmp/eq  #2,r0\n"
"    bt/s    .Lfg_ret\\sfx\n"
"    mov     r2,r0\n"
"    mov     r8,r5\n"                  /* count */
"    mov     r8,r0\n"
"    add     r0,r0\n"
"    sub     r0,r4\n"                  /* out */
"    mov     r2,r0\n"
"    add     #1,r0\n"
"    mov.b   @(r0,r3),r1\n"
"    extu.b  r1,r1\n"                  /* level[variant] */
"    mov.l   @(44,r15),r6\n"           /* mean */
"    mov.b   @r6,r0\n"
"    mov     r0,r7\n"
"    shll8   r7\n"
"    add     r0,r7\n"                  /* M = 257 m */
"    mov.w   @(8,r3),r0\n"
"    sub     r7,r0\n"
"    mov     r0,r8\n"                  /* S0 = 257 (live - m) */
"    mov.w   @(14,r3),r0\n"
"    sub     r7,r0\n"
"    muls.w  r0,r1\n"
"    mov.l   .Lfg_k0\\sfx,r9\n"
"    sts     macl,r0\n"
"    sub     r0,r9\n"                  /* C0 = K0 - level * 257 (base - m) */
"    mov.b   @(1,r6),r0\n"
"    mov     r0,r7\n"
"    shll8   r7\n"
"    add     r0,r7\n"
"    mov.w   @(10,r3),r0\n"
"    sub     r7,r0\n"
"    mov     r0,r10\n"                 /* S1 */
"    mov.w   @(16,r3),r0\n"
"    sub     r7,r0\n"
"    muls.w  r0,r1\n"
"    mov.l   .Lfg_k1\\sfx,r11\n"
"    sts     macl,r0\n"
"    sub     r0,r11\n"                 /* C1 */
"    mov.b   @(2,r6),r0\n"
"    mov     r0,r7\n"
"    shll8   r7\n"
"    add     r0,r7\n"
"    mov.w   @(12,r3),r0\n"
"    sub     r7,r0\n"
"    mov     r0,r12\n"                 /* S2 */
"    mov.w   @(18,r3),r0\n"
"    sub     r7,r0\n"
"    muls.w  r0,r1\n"
"    mov.l   .Lfg_k2\\sfx,r13\n"
"    sts     macl,r0\n"
"    sub     r0,r13\n"                 /* C2 */
"    mov.l   .Lfg_tab\\sfx,r14\n"
"    mov     r5,r7\n"
"    mov     r2,r5\n"                  /* variant */
".Lfg_p2\\sfx:\n"
"    mov.w   @r4,r1\n"                 /* fade */
"    muls.w  r1,r8\n"
"    sts     macl,r0\n"
"    add     r9,r0\n"
"    shlr16  r0\n"
"    add     r0,r0\n"
"    mov.w   @(r0,r14),r2\n"
"    muls.w  r1,r10\n"
"    sts     macl,r0\n"
"    add     r11,r0\n"
"    shlr16  r0\n"
"    add     r0,r0\n"
"    mov.w   @(r0,r14),r3\n"
"    muls.w  r1,r12\n"
"    or      r3,r2\n"
"    sts     macl,r0\n"
"    add     r13,r0\n"
"    shlr16  r0\n"
"    add     r0,r0\n"
"    mov.w   @(r0,r14),r3\n"
"    or      r3,r2\n"
"    mov.w   r2,@r4\n"
"    dt      r7\n"
"    bf/s    .Lfg_p2\\sfx\n"
"    add     #2,r4\n"
"    mov     r5,r0\n"
".Lfg_ret\\sfx:\n"
"    lds.l   @r15+,macl\n"
"    lds.l   @r15+,mach\n"
"    mov.l   @r15+,r14\n"
"    mov.l   @r15+,r13\n"
"    mov.l   @r15+,r12\n"
"    mov.l   @r15+,r11\n"
"    mov.l   @r15+,r10\n"
"    mov.l   @r15+,r9\n"
"    rts\n"
"    mov.l   @r15+,r8\n"
"    .align 2\n"
".Lfg_k0\\sfx:\n"
"    .long   128+65536*256\n"
".Lfg_k1\\sfx:\n"
"    .long   128+65536*(256+512)\n"
".Lfg_k2\\sfx:\n"
"    .long   128+65536*(256+1024)\n"
".Lfg_tab\\sfx:\n"
"    .long   _sh2zi_offset_clamp\n"
"    .endm\n"
/* The exits and the bounded search are instantiated in each entry's own
 * section (an SH-2 branch cannot reach another section), so that each entry
 * links on its own: a caller of one form does not link the other. */
"    .macro SH2ZI_FG_EXITS sfx\n"
".Lfg_bad\\sfx:\n"
"    rts\n"
"    mov     #-1,r0\n"
".Lfg_zero\\sfx:\n"
"    rts\n"
"    nop\n"
"    .endm\n"
/* Bounded search of one bucket: r1 = u, r2 = lo, r3 = hi (sign-extended
 * bytes), r11 = bucket (depth = bucket - 1024). Returns r2 = the largest i in
 * [lo, hi] with depth[i] <= u; clobbers r0 and r3, keeps every other register. */
"    .macro SH2ZI_FG_SEARCH ref\n"
".Lfg_search\\ref:\n"
"    mov.l   r9,@-r15\n"
"    mov.l   r10,@-r15\n"
"    mov.w   .Lfg_c1024\\ref,r9\n"
"    mov     r11,r10\n"
"    sub     r9,r10\n"
"    extu.b  r2,r2\n"
"    extu.b  r3,r3\n"
".Lfg_s1\\ref:\n"
"    cmp/hs  r3,r2\n"
"    bt      .Lfg_s9\\ref\n"
"    mov     r2,r0\n"
"    add     r3,r0\n"
"    add     #1,r0\n"
"    shlr    r0\n"                     /* mid */
"    mov     r0,r9\n"
"    shll2   r0\n"
"    mov.l   @(r0,r10),r0\n"
"    cmp/hs  r0,r1\n"
"    bf      .Lfg_s2\\ref\n"
"    bra     .Lfg_s1\\ref\n"
"    mov     r9,r2\n"                  /* lo = mid */
".Lfg_s2\\ref:\n"
"    mov     r9,r3\n"
"    bra     .Lfg_s1\\ref\n"
"    add     #-1,r3\n"                 /* hi = mid - 1 */
".Lfg_s9\\ref:\n"
"    mov.l   @r15+,r10\n"
"    rts\n"
"    mov.l   @r15+,r9\n"
".Lfg_c1024\\ref:\n"
"    .word   1024\n"
"    .align 2\n"
"    .endm\n"
"    .pushsection .text.sh2z_shade_fog_gouraud_refs,\"ax\",@progbits\n"
"    .align 2\n"
"    SH2ZI_FG_EXITS R\n"
"    .global _sh2z_shade_fog_gouraud_refs\n"
"    .type _sh2z_shade_fog_gouraud_refs,@function\n"
"_sh2z_shade_fog_gouraud_refs:\n"
"    SH2ZI_FG_CHECK R\n"
"    bra     .Lfg_bodyR\n"
"    nop\n"
".Lfg_bodyR:\n"
"    SH2ZI_FG_BODY R, 1\n"
"    SH2ZI_FG_SEARCH 1\n"
"    .size _sh2z_shade_fog_gouraud_refs,.-_sh2z_shade_fog_gouraud_refs\n"
"    .popsection\n"
"    .pushsection .text.sh2z_shade_fog_gouraud,\"ax\",@progbits\n"
"    .align 2\n"
"    SH2ZI_FG_EXITS S\n"
"    .global _sh2z_shade_fog_gouraud\n"
"    .type _sh2z_shade_fog_gouraud,@function\n"
"_sh2z_shade_fog_gouraud:\n"
"    SH2ZI_FG_CHECK S\n"
"    SH2ZI_FG_BODY S, 0\n"
"    SH2ZI_FG_SEARCH 0\n"
"    .size _sh2z_shade_fog_gouraud,.-_sh2z_shade_fog_gouraud\n"
"    .popsection\n"
"    .purgem SH2ZI_FG_LD\n"
"    .purgem SH2ZI_FG_P1\n"
"    .purgem SH2ZI_FG_Q\n"
"    .purgem SH2ZI_FG_QS\n"
"    .purgem SH2ZI_FG_C\n"
"    .purgem SH2ZI_FG_K\n"
"    .purgem SH2ZI_FG_CHECK\n"
"    .purgem SH2ZI_FG_BODY\n"
"    .purgem SH2ZI_FG_EXITS\n"
"    .purgem SH2ZI_FG_SEARCH\n"
);
#endif

#if !SH2ZI_FG_NATIVE
static int fog_gouraud_args(uint16_t *out,const void *z,size_t z_stride,size_t count,
    const sh2z_shade_fogp_t *fog,const sh2z_shade_ramp_t *ramp,const uint8_t mean[3]) {
    if(!out || !z || !fog || !ramp || count>65535u || z_stride>65532u ||
       (((uintptr_t)z|z_stride)&3u) || ((uintptr_t)out&1u))return -1;
    const unsigned form=ramp->form;
    if(form==SH2Z_SHADE_RAMP_OFFSET) {
        if(!mean || ((mean[0]|mean[1]|mean[2])&0x80u))return -1;
    } else if(form!=SH2Z_SHADE_RAMP_MIX)return -1;
    return 0;
}
int sh2z_shade_fog_gouraud(uint16_t *out,const void *z,size_t z_stride,size_t count,
    const sh2z_shade_fogp_t *fog,const sh2z_shade_ramp_t *ramp,const uint8_t mean[3]) {
    if(!count)return 0;
    if(fog_gouraud_args(out,z,z_stride,count,fog,ramp,mean))return -1;
    return fog_gouraud_c(out,z,z_stride,false,count,fog,ramp,mean);
}
int sh2z_shade_fog_gouraud_refs(uint16_t *out,const void *const *refs,size_t z_offset,size_t count,
    const sh2z_shade_fogp_t *fog,const sh2z_shade_ramp_t *ramp,const uint8_t mean[3]) {
    if(!count)return 0;
    if(fog_gouraud_args(out,refs,z_offset,count,fog,ramp,mean))return -1;
    return fog_gouraud_c(out,refs,z_offset,true,count,fog,ramp,mean);
}
#endif
