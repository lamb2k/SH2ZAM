/* Exact 2D integer box overlap batches. MIT; Copyright (c) 2026 SH2ZAM Contributors. */
#include "sh2zam/box2.h"

/* Always inlined: size and alignment are compile-time constants at every call,
 * which turns the overflow test and the alignment test into shifts and masks
 * (a call with runtime values pulled in libgcc's 32-bit divide on SH-2). */
#if defined(__GNUC__)
#define BOX_INLINE static inline __attribute__((always_inline))
#else
#define BOX_INLINE static inline
#endif
BOX_INLINE bool box_buffer(const void *p,size_t count,size_t size,size_t alignment) {
    if(count>(size_t)INT32_MAX+1u) return false;
    if(count>SIZE_MAX/size) return false;
    if(!count) return true;
    if(!p||((uintptr_t)p&(alignment-1u))) return false;
    const size_t bytes=count*size;
    return (uintptr_t)p<=UINTPTR_MAX-(bytes-1);
}
BOX_INLINE bool box_overlap(const void *a,size_t an,const void *b,size_t bn) {
    if(!an||!bn) return false;
    const uintptr_t aa=(uintptr_t)a,bb=(uintptr_t)b;
    return aa>=bb?aa-bb<bn:bb-aa<an;
}
/* The optional count output must be aligned for size_t and disjoint from the
 * result words and the inputs. Checked only when the caller asks for a count. */
BOX_INLINE bool box_count_ok(const size_t *n,const void *out,size_t out_bytes,
                         const void *a,size_t a_bytes,const void *b,size_t b_bytes) {
    return box_buffer(n,1,sizeof *n,_Alignof(size_t))&&!box_overlap(n,sizeof *n,out,out_bytes)&&
           !box_overlap(n,sizeof *n,a,a_bytes)&&!box_overlap(n,sizeof *n,b,b_bytes);
}

/* Predicates for the batch loops: each is the documented formula. */
#define BOX_MM_INC(q,b) ((q)->min_x<=(b)->max_x&&(b)->min_x<=(q)->max_x&&(q)->min_y<=(b)->max_y&&(b)->min_y<=(q)->max_y)
#define BOX_MM_EXC(q,b) ((q)->min_x<(b)->max_x&&(b)->min_x<(q)->max_x&&(q)->min_y<(b)->max_y&&(b)->min_y<(q)->max_y)
static inline uint32_t box_absdiff(int32_t a,int32_t b) {
    return a>=b ? (uint32_t)a-(uint32_t)b : (uint32_t)b-(uint32_t)a;
}
#define BOX_C_INC(q,b) (box_absdiff((q)->cx,(b)->cx)<=(uint32_t)(q)->hx+(uint32_t)(b)->hx&&box_absdiff((q)->cy,(b)->cy)<=(uint32_t)(q)->hy+(uint32_t)(b)->hy)
#define BOX_C_EXC(q,b) (box_absdiff((q)->cx,(b)->cx)<(uint32_t)(q)->hx+(uint32_t)(b)->hx&&box_absdiff((q)->cy,(b)->cy)<(uint32_t)(q)->hy+(uint32_t)(b)->hy)

#if defined(__GNUC__)&&!defined(__clang__)&&__GNUC__>=8
#define BOX_UNROLL4 _Pragma("GCC unroll 4")
#else
#define BOX_UNROLL4
#endif
/* One bitset row; returns the number of set bits. The result bit is a mask that
 * doubles per box (an add, not a variable shift, which SH-2 lacks), so full and
 * tail words use the same loop. Counting inside the loop costs one add per hit. */
#define BOX_ROW(NAME,TYPE,TEST) \
static size_t NAME(uint32_t *out,const TYPE *q,const TYPE *b,size_t count) { \
    size_t hits=0; \
    for(size_t w=count>>5;w;w--) { \
        uint32_t acc=0,bit=1; \
        BOX_UNROLL4 for(unsigned j=0;j<32u;j++,b++,bit+=bit) { if(TEST(q,b)) { acc|=bit; hits++; } } \
        *out++=acc; \
    } \
    const unsigned rest=(unsigned)(count&31u); \
    if(rest) { \
        uint32_t acc=0,bit=1; \
        BOX_UNROLL4 for(unsigned j=0;j<rest;j++,b++,bit+=bit) { if(TEST(q,b)) { acc|=bit; hits++; } } \
        *out=acc; \
    } \
    return hits; \
}
BOX_ROW(row_mm_inc,sh2z_ibox2_t,BOX_MM_INC)
BOX_ROW(row_mm_exc,sh2z_ibox2_t,BOX_MM_EXC)
BOX_ROW(row_c_inc,sh2z_ibox2c_t,BOX_C_INC)
BOX_ROW(row_c_exc,sh2z_ibox2c_t,BOX_C_EXC)

#define BOX_API(ONE,MANY,UNCHECKED,TYPE,ROW_INC,ROW_EXC) \
size_t UNCHECKED(uint32_t *hits,const TYPE *query,const TYPE *boxes,size_t count,sh2z_box_edges_t edges) { \
    if(!count) return 0; \
    return edges==SH2Z_BOX_EXCLUSIVE ? ROW_EXC(hits,query,boxes,count) : ROW_INC(hits,query,boxes,count); \
} \
bool ONE(uint32_t *hits,size_t hit_word_capacity,size_t *hit_count,const TYPE *query,const TYPE *boxes,size_t count,sh2z_box_edges_t edges) { \
    const size_t words=sh2z_box_bitset_words(count),hit_bytes=words*sizeof *hits,box_bytes=count*sizeof *boxes; \
    if(!box_buffer(boxes,count,sizeof *boxes,_Alignof(TYPE))||!box_buffer(hits,words,sizeof *hits,_Alignof(uint32_t))|| \
       hit_word_capacity<words||!box_buffer(query,1,sizeof *query,_Alignof(TYPE))|| \
       (edges!=SH2Z_BOX_INCLUSIVE&&edges!=SH2Z_BOX_EXCLUSIVE)|| \
       box_overlap(hits,hit_bytes,boxes,box_bytes)||box_overlap(hits,hit_bytes,query,sizeof *query)) return false; \
    if(hit_count&&!box_count_ok(hit_count,hits,hit_bytes,boxes,box_bytes,query,sizeof *query)) return false; \
    const size_t n=!count ? 0u : edges==SH2Z_BOX_EXCLUSIVE ? ROW_EXC(hits,query,boxes,count) : ROW_INC(hits,query,boxes,count); \
    if(hit_count) *hit_count=n; \
    return true; \
} \
bool MANY(uint32_t *bits,size_t bit_word_capacity,size_t *pair_count,const TYPE *a,size_t a_count,const TYPE *b,size_t b_count,sh2z_box_edges_t edges) { \
    const size_t row_words=sh2z_box_bitset_words(b_count); \
    if(row_words&&a_count>SIZE_MAX/row_words) return false; \
    const size_t words=a_count*row_words,bit_bytes=words*sizeof *bits,a_bytes=a_count*sizeof *a,b_bytes=b_count*sizeof *b; \
    if(!box_buffer(a,a_count,sizeof *a,_Alignof(TYPE))||!box_buffer(b,b_count,sizeof *b,_Alignof(TYPE))|| \
       !box_buffer(bits,words,sizeof *bits,_Alignof(uint32_t))||bit_word_capacity<words|| \
       (edges!=SH2Z_BOX_INCLUSIVE&&edges!=SH2Z_BOX_EXCLUSIVE)|| \
       box_overlap(bits,bit_bytes,a,a_bytes)||box_overlap(bits,bit_bytes,b,b_bytes)) return false; \
    if(pair_count&&!box_count_ok(pair_count,bits,bit_bytes,a,a_bytes,b,b_bytes)) return false; \
    size_t n=0; \
    if(words) for(size_t r=0;r<a_count;r++) \
        n+=edges==SH2Z_BOX_EXCLUSIVE ? ROW_EXC(bits+r*row_words,a+r,b,b_count) : ROW_INC(bits+r*row_words,a+r,b,b_count); \
    if(pair_count) *pair_count=n; \
    return true; \
}
BOX_API(sh2z_ibox2_overlap_one_many,sh2z_ibox2_overlap_many_many,sh2z_ibox2_overlap_one_many_unchecked,sh2z_ibox2_t,row_mm_inc,row_mm_exc)
BOX_API(sh2z_ibox2c_overlap_one_many,sh2z_ibox2c_overlap_many_many,sh2z_ibox2c_overlap_one_many_unchecked,sh2z_ibox2c_t,row_c_inc,row_c_exc)
