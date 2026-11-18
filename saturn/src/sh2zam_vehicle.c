/* Vehicle kernels (contract tag vehicle-kernels-1): screen-space facing of
 * projected quads, lighting of referenced normals only, and the approximate
 * Gouraud packing. A separate file, so that the shade-1 functions compile
 * exactly as before. MIT; SH2ZAM Contributors. */
#include "sh2zam/shade_experimental.h"

#if defined(__sh__) && defined(__SH2__) && (!defined(SH2Z_FORCE_PORTABLE) || !SH2Z_FORCE_PORTABLE)
#define SH2ZI_VEHICLE_NATIVE 1
#if !defined(__BYTE_ORDER__) || __BYTE_ORDER__ != __ORDER_BIG_ENDIAN__
#error "the SH-2 vehicle kernels read big-endian words"
#endif
#else
#define SH2ZI_VEHICLE_NATIVE 0
#endif

static bool overlaps(const void *a,size_t a_bytes,const void *b,size_t b_bytes) {
    const uintptr_t x=(uintptr_t)a,y=(uintptr_t)b;
    return x<y+b_bytes && y<x+a_bytes;
}
/* Bytes spanned by count records of size bytes at stride; 0 on overflow
 * (count > 0 here). No division for counts and strides below 2^16. */
static size_t span(size_t count,size_t stride,size_t size) {
    const size_t n=count-1u;
    if(!((n|stride)>>16))return n*stride<=SIZE_MAX-size?n*stride+size:0;
    if(n>(SIZE_MAX-size)/stride)return 0;
    return n*stride+size;
}
static bool span_ok(const void *p,size_t bytes) {
    return bytes && (uintptr_t)p<=UINTPTR_MAX-bytes;
}

/* ---- screen-space facing ------------------------------------------------- */

static bool corner_simple(const sh2z_view_vertex_t *a) {
    return !(a->outcode&(SH2Z_OUT_NEAR|SH2Z_OUT_FAR)) &&
        a->sx>=-SH2Z_SHADE_FACING_LIMIT && a->sx<=SH2Z_SHADE_FACING_LIMIT &&
        a->sy>=-SH2Z_SHADE_FACING_LIMIT && a->sy<=SH2Z_SHADE_FACING_LIMIT;
}

#if SH2ZI_VEHICLE_NATIVE
typedef struct {
    const sh2z_view_vertex_t *vertices;
    const uint8_t (*corners)[4];
    uint16_t *kept;
    int32_t sign;
} facing_frame_t;
uint32_t sh2zi_facing_slow(uint32_t id,const facing_frame_t *frame);
/* Fast case (the clip rectangle inside the limits), sign +1 and sign -1.
 *
 * Per face: one long load gives the four corner indices; each corner's
 * sx:sy word W = sx << 16 | (sy & 0xFFFF) and its swap S = sy << 16 |
 * (sx & 0xFFFF) are loaded and formed; on the diagonals 1-3 and 0-2 the low
 * halves of W3 - W1, S3 - S1, W2 - W0 and S2 - S0 are y3 - y1, x3 - x1,
 * y2 - y0 and x2 - x0 modulo 2^16, exact as int16 because every coordinate
 * of a quad with zero outcodes is inside the clip rectangle, so inside
 * +/-1023. MULS.W reads exactly those halves:
 *     area = (x2 - x0)(y3 - y1) - (x3 - x1)(y2 - y0)
 * which is the sum of the two triangle areas of the definition (both are
 * the shoelace sum of the quad). A non-zero OR of the outcodes sends the
 * face to sh2zi_facing_slow (the definition in C). Registers: r8 faces,
 * r9 count, r10 kept, r11 vertices + 12, r12 vertices + 16, r13 corners,
 * r14 = 20; the frame pointer is kept on the stack for the slow call. */
#define FACING_ENTRY(tag,a,b) \
"    .pushsection .text.sh2zi_facing_" tag ",\"ax\",@progbits\n" \
"    .align 2\n" \
"    .global _sh2zi_facing_" tag "\n" \
"    .type _sh2zi_facing_" tag ",@function\n" \
"_sh2zi_facing_" tag ":\n" \
"    mov.l   r8,@-r15\n" \
"    mov.l   r9,@-r15\n" \
"    mov.l   r10,@-r15\n" \
"    mov.l   r11,@-r15\n" \
"    mov.l   r12,@-r15\n" \
"    mov.l   r13,@-r15\n" \
"    mov.l   r14,@-r15\n" \
"    sts.l   mach,@-r15\n" \
"    sts.l   macl,@-r15\n" \
"    sts.l   pr,@-r15\n" \
"    mov.l   r6,@-r15\n" \
"    mov     r4,r8\n" \
"    mov     r5,r9\n" \
"    mov.l   @r6,r11\n" \
"    mov     r11,r12\n" \
"    add     #12,r11\n" \
"    add     #16,r12\n" \
"    mov.l   @(4,r6),r13\n" \
"    mov.l   @(8,r6),r10\n" \
"    mov     #20,r14\n" \
"1:  mov.w   @r8,r0\n" \
"    extu.w  r0,r0\n" \
"    shll2   r0\n" \
"    mov.l   @(r0,r13),r1\n"      /* c0 c1 c2 c3 */ \
"    extu.b  r1,r0\n" \
"    mulu.w  r0,r14\n" \
"    sts     macl,r0\n" \
"    mov.l   @(r0,r11),r2\n"      /* W3 */ \
"    mov.b   @(r0,r12),r7\n"      /* outcodes ORed into r7 */ \
"    swap.w  r2,r3\n"             /* S3 */ \
"    swap.w  r1,r0\n" \
"    extu.b  r0,r0\n"             /* c1 */ \
"    mulu.w  r0,r14\n" \
"    sts     macl,r0\n" \
"    mov.l   @(r0,r11),r4\n"      /* W1 */ \
"    mov.b   @(r0,r12),r0\n" \
"    or      r0,r7\n" \
"    swap.w  r4,r5\n"             /* S1 */ \
"    sub     r4,r2\n"             /* low half y3 - y1 */ \
"    sub     r5,r3\n"             /* low half x3 - x1 */ \
"    swap.b  r1,r0\n" \
"    extu.b  r0,r0\n"             /* c2 */ \
"    mulu.w  r0,r14\n" \
"    sts     macl,r0\n" \
"    mov.l   @(r0,r11),r4\n"      /* W2 */ \
"    mov.b   @(r0,r12),r0\n" \
"    or      r0,r7\n" \
"    swap.w  r4,r5\n"             /* S2 */ \
"    shlr16  r1\n" \
"    shlr8   r1\n"                /* c0 */ \
"    mulu.w  r1,r14\n" \
"    sts     macl,r0\n" \
"    mov.l   @(r0,r11),r6\n"      /* W0 */ \
"    mov.b   @(r0,r12),r0\n" \
"    or      r0,r7\n" \
"    swap.w  r6,r1\n"             /* S0 */ \
"    sub     r6,r4\n"             /* low half y2 - y0 */ \
"    sub     r1,r5\n"             /* low half x2 - x0 */ \
"    tst     r7,r7\n" \
"    bf      4f\n" \
"    muls.w  r5,r2\n" \
"    sts     macl,r0\n"           /* (x2 - x0)(y3 - y1) */ \
"    muls.w  r3,r4\n" \
"    sts     macl,r1\n"           /* (x3 - x1)(y2 - y0) */ \
"    cmp/gt  " a "," b "\n"       /* sign * area > 0 */ \
"    bf      3f\n" \
"2:  mov.w   @r8,r0\n" \
"    mov.w   r0,@r10\n" \
"    add     #2,r10\n" \
"3:  dt      r9\n" \
"    bf/s    1b\n" \
"    add     #2,r8\n" \
"    mov.l   @r15+,r6\n" \
"    mov.l   @(8,r6),r1\n" \
"    mov     r10,r0\n" \
"    sub     r1,r0\n" \
"    shlr    r0\n"                /* kept count */ \
"    lds.l   @r15+,pr\n" \
"    lds.l   @r15+,macl\n" \
"    lds.l   @r15+,mach\n" \
"    mov.l   @r15+,r14\n" \
"    mov.l   @r15+,r13\n" \
"    mov.l   @r15+,r12\n" \
"    mov.l   @r15+,r11\n" \
"    mov.l   @r15+,r10\n" \
"    mov.l   @r15+,r9\n" \
"    rts\n" \
"    mov.l   @r15+,r8\n" \
"4:  mov.w   @r8,r4\n" \
"    extu.w  r4,r4\n" \
"    mov.l   5f,r0\n" \
"    jsr     @r0\n" \
"    mov.l   @r15,r5\n" \
"    tst     r0,r0\n" \
"    bt      3b\n" \
"    bra     2b\n" \
"    nop\n" \
"    .align 2\n" \
"5:  .long   _sh2zi_facing_slow\n" \
"    .size _sh2zi_facing_" tag ",.-_sh2zi_facing_" tag "\n" \
"    .popsection\n"
__asm__(
FACING_ENTRY("pos","r1","r0")
FACING_ENTRY("neg","r0","r1")
);
uint32_t sh2zi_facing_pos(const uint16_t *faces,size_t count,const facing_frame_t *frame);
uint32_t sh2zi_facing_neg(const uint16_t *faces,size_t count,const facing_frame_t *frame);
#endif

/* Portable path, and the SH-2 path's slow case (a corner outside the clip
 * rectangle): the definition written out. */
static bool facing_keep(const uint8_t c[4],const sh2z_view_vertex_t *vertices,int32_t sign) {
    const sh2z_view_vertex_t *a=vertices+c[0],*b=vertices+c[1],*d=vertices+c[2],*e=vertices+c[3];
    if(a->outcode&b->outcode&d->outcode&e->outcode)return false;
    if(!(corner_simple(a) && corner_simple(b) && corner_simple(d) && corner_simple(e)))return true;
    /* |coordinates| <= 1023: every product and sum is far inside int32. */
    const int32_t x0=a->sx,y0=a->sy;
    const int32_t area=(b->sx-x0)*(d->sy-y0)-(b->sy-y0)*(d->sx-x0)+
        (d->sx-x0)*(e->sy-y0)-(d->sy-y0)*(e->sx-x0);
    return sign*area>0;
}

bool sh2z_shade_screen_facing(uint16_t *kept,size_t *kept_count,const uint16_t *faces,
    size_t count,const uint8_t (*corners)[4],const sh2z_view_vertex_t *vertices,
    const sh2z_view_t *view,int32_t sign) {
    if(!kept_count)return false;
    *kept_count=0;
    if(!count)return true;
    if(!kept || !faces || !corners || !vertices || !view || (sign!=1 && sign!=-1) ||
       count>SIZE_MAX/sizeof(*kept))return false;
    const size_t bytes=count*sizeof(*kept);
    if(!span_ok(kept,bytes) || !span_ok(faces,bytes) ||
       (kept!=faces && overlaps(kept,bytes,faces,bytes)))return false;
#if SH2ZI_VEHICLE_NATIVE
    /* A quad whose four outcodes are zero lies inside the clip rectangle;
     * when that rectangle is inside the coordinate limits, such a quad is
     * simple without testing its coordinates. */
    const int16_t *clip=view->clip;
    if(clip[0]>=-SH2Z_SHADE_FACING_LIMIT && clip[2]<=SH2Z_SHADE_FACING_LIMIT &&
       clip[1]>=-SH2Z_SHADE_FACING_LIMIT && clip[3]<=SH2Z_SHADE_FACING_LIMIT &&
       !((uintptr_t)corners&3u) && !((uintptr_t)vertices&3u) && !((uintptr_t)faces&1u) &&
       !((uintptr_t)kept&1u) && count<=UINT32_MAX) {
        facing_frame_t frame;
        frame.vertices=vertices;frame.corners=corners;frame.kept=kept;frame.sign=sign;
        *kept_count=sign>0?sh2zi_facing_pos(faces,count,&frame):sh2zi_facing_neg(faces,count,&frame);
        return true;
    }
#endif
    size_t n=0;
    for(size_t i=0;i<count;++i) {
        const uint16_t id=faces[i];
        if(facing_keep(corners[id],vertices,sign))kept[n++]=id;
    }
    *kept_count=n;
    return true;
}

#if SH2ZI_VEHICLE_NATIVE
/* Slow case of the assembly loop (a non-zero outcode): 1 to keep face id. */
uint32_t sh2zi_facing_slow(uint32_t id,const facing_frame_t *frame) {
    return facing_keep(frame->corners[id],frame->vertices,frame->sign);
}
#endif

/* ---- lighting of referenced normals --------------------------------------- */

/* d * c / (127 * 16384) for 0 <= d <= 127 * 16384, from the prepared
 * ceil(c * 2^29 / 127): exact (shade-1's proof). */
static uint32_t light_scale(uint32_t d,uint32_t k) {
    return (uint32_t)(((uint64_t)d*k)>>43);
}
static void light_one(uint8_t *out,const uint8_t *n,const sh2z_shade_light_t *light,const uint8_t lut[256]) {
    const int32_t x=(int8_t)n[0],y=(int8_t)n[1],z=(int8_t)n[2];
    const int32_t diffuse=x*light->light[0]+y*light->light[1]+z*light->light[2];
    const int32_t reflected=x*light->reflection[0]+y*light->reflection[1]+z*light->reflection[2];
    uint32_t v0=light->ambient[0],v1=light->ambient[1],v2=light->ambient[2];
    if(diffuse>0) {
        const uint32_t d=diffuse>127*16384?127u*16384u:(uint32_t)diffuse;
        v0+=light_scale(d,light->scale[0]);v1+=light_scale(d,light->scale[1]);v2+=light_scale(d,light->scale[2]);
    }
    out[0]=(uint8_t)(v0>255u?255u:v0);
    out[1]=(uint8_t)(v1>255u?255u:v1);
    out[2]=(uint8_t)(v2>255u?255u:v2);
    int32_t index=reflected>=0?(int32_t)((uint32_t)reflected>>14):-(int32_t)((0u-(uint32_t)reflected)>>14);
    if(index< -128)index=-128;
    if(index>127)index=127;
    out[3]=lut[index+128];
}

#if SH2ZI_VEHICLE_NATIVE
/* SH-2 path for normal_stride == out_stride == 4, a 4-byte aligned output and
 * a prepared light whose six components fit int16 (MULS.W reads 16 bits).
 * The channel is light_normals' d * c / (127 * 16384) through
 * K = ceil(c * 2^28 / 127) = ceil(scale / 2): (d * K) >> 42 is exact for
 * 0 <= d <= D = 127 * 16384 because d * K / 2^42 exceeds d * c / D by less
 * than D / 2^42 < 1 / D (D^2 < 2^42), the smallest distance from a
 * non-integer d * c / D to the next integer; DMULU.L, then MACH >> 10
 * (tests/test_vehicle.c checks every d and every c). The reflection index
 * trunc(r / 16384) + 128 is (r + 2^21 + (r < 0 ? 16383 : 0)) >> 14 for the
 * in-range values; any other value of that shift is outside 0..255 and is
 * clamped to the table's end by the sign. A normal with d <= 0 stores the
 * ambient word and the reflection byte with one long store. */
typedef struct {
    uint8_t *out;
    int32_t lx,ly,lz;
    int32_t rx,ry,rz;        /* +16: the per-normal block */
    uint32_t k0,k1,k2;
    uint32_t a0,a1,a2;
    uint32_t ambient;        /* a0 << 24 | a1 << 16 | a2 << 8 */
    const uint8_t *lut;
    uint32_t bias;           /* 2^21 */
    uint32_t cap;            /* 127 * 16384 */
    uint32_t c255;
} light_frame_t;
_Static_assert(offsetof(light_frame_t,rx)==16,"light block");
_Static_assert(offsetof(light_frame_t,c255)==68,"light block end");
#define LIGHT_CHANNEL(k,a,step,sat) \
"    mov.l   @(" k ",r6),r2\n" \
"    dmulu.l r12,r2\n" \
"    mov.l   @(" a ",r6),r3\n" \
"    sts     mach,r2\n" \
"    shlr8   r2\n" \
"    shlr2   r2\n" \
"    add     r3,r2\n" \
sat \
"    mov.b   r2,@(r0,r8)\n" \
"    add     #" step ",r0\n"
#define LIGHT_SAT \
"    cmp/hi  r14,r2\n" \
"    subc    r3,r3\n" \
"    or      r3,r2\n"
#define LIGHT_ENTRY(tag,sat,satload) \
"    .pushsection .text.sh2zi_light_" tag ",\"ax\",@progbits\n" \
"    .align 2\n" \
"    .global _sh2zi_light_" tag "\n" \
"    .type _sh2zi_light_" tag ",@function\n" \
"_sh2zi_light_" tag ":\n" \
"    mov.l   r8,@-r15\n" \
"    mov.l   r9,@-r15\n" \
"    mov.l   r10,@-r15\n" \
"    mov.l   r11,@-r15\n" \
"    mov.l   r12,@-r15\n" \
"    mov.l   r13,@-r15\n" \
"    mov.l   r14,@-r15\n" \
"    sts.l   mach,@-r15\n" \
"    sts.l   macl,@-r15\n" \
"    mov.l   @r6,r8\n" \
"    mov.l   @(4,r6),r9\n" \
"    mov.l   @(8,r6),r10\n" \
"    mov.l   @(12,r6),r11\n" \
"    add     #16,r6\n" \
"1:  mov.b   @r4+,r0\n" \
"    extu.b  r0,r0\n" \
"    shll2   r0\n" \
"    mov.b   @(r0,r7),r1\n" \
"    add     #1,r0\n" \
"    mov.b   @(r0,r7),r2\n" \
"    add     #1,r0\n" \
"    mov.b   @(r0,r7),r3\n"        /* r0 = 4i + 2 */ \
"    muls.w  r1,r9\n" \
"    sts     macl,r12\n" \
"    muls.w  r2,r10\n" \
"    sts     macl,r13\n" \
"    muls.w  r3,r11\n" \
"    add     r13,r12\n" \
"    sts     macl,r13\n" \
"    add     r13,r12\n"            /* diffuse */ \
"    mov.l   @r6,r13\n" \
"    muls.w  r1,r13\n" \
"    mov.l   @(4,r6),r14\n" \
"    sts     macl,r1\n" \
"    muls.w  r2,r14\n" \
"    mov.l   @(8,r6),r13\n" \
"    sts     macl,r2\n" \
"    muls.w  r3,r13\n" \
"    add     r2,r1\n" \
"    sts     macl,r3\n" \
"    add     r3,r1\n"              /* reflected */ \
"    mov     r1,r2\n" \
"    shll    r2\n" \
"    subc    r2,r2\n" \
"    shlr16  r2\n" \
"    shlr2   r2\n" \
"    add     r2,r1\n" \
"    mov.l   @(44,r6),r2\n" \
"    add     r2,r1\n" \
"    shll2   r1\n" \
"    shlr16  r1\n"                 /* table index if <= 255 */ \
"    mov.l   @(52,r6),r2\n" \
"    cmp/hi  r2,r1\n" \
"    bt      9f\n" \
"8:  mov.l   @(40,r6),r2\n" \
"    add     r2,r1\n" \
"    cmp/pl  r12\n" \
"    bf/s    7f\n" \
"    mov.b   @r1,r1\n"             /* reflection byte */ \
"    mov.l   @(48,r6),r2\n" \
"    cmp/hi  r2,r12\n" \
"    bt      10f\n" \
"6:\n" \
satload \
LIGHT_CHANNEL("20","32","-1",sat) \
LIGHT_CHANNEL("16","28","-1",sat) \
LIGHT_CHANNEL("12","24","3",sat) \
"    mov.b   r1,@(r0,r8)\n" \
"    dt      r5\n" \
"    bf      1b\n" \
"    bra     5f\n" \
"    nop\n" \
"7:  extu.b  r1,r1\n" \
"    mov.l   @(36,r6),r2\n" \
"    or      r1,r2\n" \
"    add     #-2,r0\n" \
"    mov.l   r2,@(r0,r8)\n" \
"    dt      r5\n" \
"    bf      1b\n" \
"5:  lds.l   @r15+,macl\n" \
"    lds.l   @r15+,mach\n" \
"    mov.l   @r15+,r14\n" \
"    mov.l   @r15+,r13\n" \
"    mov.l   @r15+,r12\n" \
"    mov.l   @r15+,r11\n" \
"    mov.l   @r15+,r10\n" \
"    mov.l   @r15+,r9\n" \
"    rts\n" \
"    mov.l   @r15+,r8\n" \
"9:  shll16  r1\n"                 /* bit 15 set: below -128 */ \
"    cmp/pz  r1\n" \
"    movt    r1\n" \
"    neg     r1,r1\n" \
"    bra     8b\n" \
"    extu.b  r1,r1\n" \
"10: bra     6b\n" \
"    mov     r2,r12\n" \
"    .size _sh2zi_light_" tag ",.-_sh2zi_light_" tag "\n" \
"    .popsection\n"
__asm__(
LIGHT_ENTRY("run","","")
LIGHT_ENTRY("run_sat",LIGHT_SAT,"    mov.l   @(52,r6),r14\n")
);
void sh2zi_light_run(const uint8_t *index,size_t count,const light_frame_t *frame,const int8_t *normals);
void sh2zi_light_run_sat(const uint8_t *index,size_t count,const light_frame_t *frame,const int8_t *normals);
#endif

bool sh2z_shade_light_normals_indexed(uint8_t *out,size_t out_stride,const int8_t *normals,
    size_t normal_stride,size_t count,const uint8_t *index,size_t index_count,
    const sh2z_shade_light_t *light,const uint8_t reflection_lut[256]) {
    if(!index_count)return true;
    if(!count || !out || !normals || !index || !light || !reflection_lut || normal_stride<3 || out_stride<4)return false;
    const size_t out_bytes=span(count,out_stride,4),in_bytes=span(count,normal_stride,3);
    if(!span_ok(out,out_bytes) || !span_ok(normals,in_bytes) || !span_ok(index,index_count) ||
       overlaps(out,out_bytes,normals,in_bytes) || overlaps(out,out_bytes,light,sizeof(*light)) ||
       overlaps(out,out_bytes,reflection_lut,256) || overlaps(out,out_bytes,index,index_count))return false;
#if SH2ZI_VEHICLE_NATIVE
    if(normal_stride==4 && out_stride==4 && !((uintptr_t)out&3u)) {
        bool small=true,saturating=false;
        for(unsigned k=0;k<3;++k) {
            if(light->light[k]< -32767 || light->light[k]>32767 ||
               light->reflection[k]< -32767 || light->reflection[k]>32767)small=false;
            if((uint32_t)light->ambient[k]+light->color[k]>255u)saturating=true;
        }
        if(small) {
            light_frame_t f;
            f.out=out;f.lx=light->light[0];f.ly=light->light[1];f.lz=light->light[2];
            f.rx=light->reflection[0];f.ry=light->reflection[1];f.rz=light->reflection[2];
            f.k0=(light->scale[0]>>1)+(light->scale[0]&1u);
            f.k1=(light->scale[1]>>1)+(light->scale[1]&1u);
            f.k2=(light->scale[2]>>1)+(light->scale[2]&1u);
            f.a0=light->ambient[0];f.a1=light->ambient[1];f.a2=light->ambient[2];
            f.ambient=(f.a0<<24)|(f.a1<<16)|(f.a2<<8);
            f.lut=reflection_lut;f.bias=UINT32_C(1)<<21;f.cap=127u*16384u;f.c255=255u;
            if(saturating)sh2zi_light_run_sat(index,index_count,&f,normals);
            else sh2zi_light_run(index,index_count,&f,normals);
            return true;
        }
    }
#endif
    const uint8_t *n=(const uint8_t *)normals;
    for(size_t k=0;k<index_count;++k) {
        const size_t i=index[k];
        light_one(out+i*out_stride,n+i*normal_stride,light,reflection_lut);
    }
    return true;
}

/* ---- approximate Gouraud --------------------------------------------------- */

static uint32_t approx_channel(uint32_t b,uint32_t l,uint32_t f,uint32_t t) {
    const uint32_t m=(b*l+128u)>>8;
    const uint32_t lit=(m*t+f*(255u-t)+128u)>>8;
    return 16u+(lit>>3)-(b>>4);      /* 1 .. 47 */
}

#if SH2ZI_VEHICLE_NATIVE
/* SH-2 path for light_stride == 4 (4-byte aligned records). Per channel,
 * with x = b * l (MULU.W) and F = 128 - 256 * f:
 *     y = x + F;  low half of (y >> 8, logical) = m - f   (as int16: y >> 8
 *     taken logically keeps floor(y / 256) modulo 2^16, and |m - f| <= 255)
 *     S = (m - f) * t + C,  C = 255 * f + 128 + 16 * 2048   (MULS.W)
 *     v = (S >> 11) - (b >> 4)        = 16 + (lit >> 3) - (b >> 4)
 * since S - 32768 = m * t + f * (255 - t) + 128 >= 0. v > 31 takes a stub
 * that stores 31 and counts the saturation in the frame. */
typedef struct {
    const uint8_t *base;
    const uint8_t *light;
    uint32_t base_stride;
    int32_t g0,g1,g2;      /* 128 - 256 * f[k] */
    uint32_t c0,c1,c2;     /* 255 * f[k] + 128 + 32768 */
    uint32_t saturated;
} gouraud_frame_t;
_Static_assert(offsetof(gouraud_frame_t,saturated)==36,"gouraud frame");
#define GOURAUD_CHANNEL(n,lget,g,c) \
"    mov.b   @r1+,r3\n" \
"    extu.b  r3,r3\n"              /* b */ \
lget \
"    mulu.w  r3,r0\n" \
"    mov     r3,r13\n" \
"    shlr2   r13\n" \
"    shlr2   r13\n"                /* b >> 4 */ \
"    mov.l   @(" g ",r6),r3\n" \
"    sts     macl,r0\n" \
"    add     r3,r0\n" \
"    shlr8   r0\n"                 /* low half: m - f */ \
"    muls.w  r0,r14\n" \
"    mov.l   @(" c ",r6),r3\n" \
"    sts     macl,r0\n" \
"    add     r3,r0\n" \
"    shlr8   r0\n" \
"    shlr2   r0\n" \
"    shlr    r0\n" \
"    sub     r13,r0\n"             /* v, 1 .. 47 */ \
"    cmp/gt  r12,r0\n" \
"    bt      " n "f\n" \
n "0:\n"
#define GOURAUD_SAT(n) \
n ":  mov.l   @(36,r6),r3\n" \
"    add     #1,r3\n" \
"    mov.l   r3,@(36,r6)\n" \
"    bra     " n "0b\n" \
"    mov     r12,r0\n"
__asm__(
"    .pushsection .text.sh2zi_gouraud_run,\"ax\",@progbits\n"
"    .align 2\n"
"    .global _sh2zi_gouraud_run\n"
"    .type _sh2zi_gouraud_run,@function\n"
"_sh2zi_gouraud_run:\n"
"    mov.l   r8,@-r15\n"
"    mov.l   r9,@-r15\n"
"    mov.l   r10,@-r15\n"
"    mov.l   r11,@-r15\n"
"    mov.l   r12,@-r15\n"
"    mov.l   r13,@-r15\n"
"    mov.l   r14,@-r15\n"
"    sts.l   mach,@-r15\n"
"    sts.l   macl,@-r15\n"
"    mov.l   @r6,r9\n"             /* bases */
"    mov.l   @(4,r6),r8\n"         /* light records */
"    mov.l   @(8,r6),r11\n"        /* base stride */
"    mov     #31,r12\n"
"1:  mov.l   @r4+,r1\n"            /* base << 16 | light */
"    extu.w  r1,r0\n"
"    shll2   r0\n"
"    mov.l   @(r0,r8),r2\n"        /* l0 l1 l2 l3 */
"    shlr16  r1\n"
"    mulu.w  r1,r11\n"
"    not     r2,r14\n"
"    extu.b  r14,r14\n"            /* t = 255 - l3 */
"    sts     macl,r1\n"
"    add     r9,r1\n"              /* base record */
GOURAUD_CHANNEL("21",
"    mov     r2,r0\n"
"    shlr16  r0\n"
"    shlr8   r0\n","12","24")
"    mov     r0,r10\n"
GOURAUD_CHANNEL("22",
"    swap.w  r2,r0\n"
"    extu.b  r0,r0\n","16","28")
"    shll2   r0\n"
"    shll2   r0\n"
"    shll    r0\n"
"    or      r0,r10\n"
GOURAUD_CHANNEL("23",
"    swap.b  r2,r0\n"
"    extu.b  r0,r0\n","20","32")
"    shll8   r0\n"
"    shll2   r0\n"
"    or      r0,r10\n"
"    mov.w   r10,@r7\n"
"    dt      r5\n"
"    bf/s    1b\n"
"    add     #2,r7\n"
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
GOURAUD_SAT("21")
GOURAUD_SAT("22")
GOURAUD_SAT("23")
"    .size _sh2zi_gouraud_run,.-_sh2zi_gouraud_run\n"
"    .popsection\n"
);
void sh2zi_gouraud_run(const sh2z_shade_pair_t *pairs,size_t count,gouraud_frame_t *frame,uint16_t *out);
#endif

bool sh2z_shade_gouraud_approx(uint16_t *out,uint32_t *saturated,const sh2z_shade_pair_t *pairs,
    size_t count,const uint8_t *base,size_t base_stride,const uint8_t *light,
    size_t light_stride,const uint8_t fog[3]) {
    if(!count){if(saturated)*saturated=0;return true;}
    if(!out || !pairs || !base || !light || !fog || base_stride<3 || light_stride<4 ||
       count>SIZE_MAX/sizeof(*pairs))return false;
    const size_t out_bytes=count*sizeof(*out);
    if(!span_ok(out,out_bytes) || !span_ok(pairs,count*sizeof(*pairs)) ||
       overlaps(out,out_bytes,pairs,count*sizeof(*pairs)) || overlaps(out,out_bytes,fog,3))return false;
    uint32_t clamped=0;
#if SH2ZI_VEHICLE_NATIVE
    if(light_stride==4 && !((uintptr_t)light&3u) && base_stride<=0xFFFFu && !((uintptr_t)out&1u) &&
       !((uintptr_t)pairs&3u)) {
        gouraud_frame_t f;
        f.base=base;f.light=light;f.base_stride=(uint32_t)base_stride;
        f.g0=128-256*(int32_t)fog[0];f.g1=128-256*(int32_t)fog[1];f.g2=128-256*(int32_t)fog[2];
        f.c0=255u*fog[0]+128u+32768u;f.c1=255u*fog[1]+128u+32768u;f.c2=255u*fog[2]+128u+32768u;
        f.saturated=0;
        sh2zi_gouraud_run(pairs,count,&f,out);
        if(saturated)*saturated=f.saturated;
        return true;
    }
#endif
    const uint32_t f0=fog[0],f1=fog[1],f2=fog[2];
    for(size_t i=0;i<count;++i) {
        const uint8_t *b=base+(size_t)pairs[i].base*base_stride;
        const uint8_t *l=light+(size_t)pairs[i].light*light_stride;
        const uint32_t t=255u-l[3];
        uint32_t v0=approx_channel(b[0],l[0],f0,t),v1=approx_channel(b[1],l[1],f1,t),v2=approx_channel(b[2],l[2],f2,t);
        if(v0>31u){v0=31;++clamped;}
        if(v1>31u){v1=31;++clamped;}
        if(v2>31u){v2=31;++clamped;}
        out[i]=(uint16_t)(v0|(v1<<5)|(v2<<10));
    }
    if(saturated)*saturated=clamped;
    return true;
}
