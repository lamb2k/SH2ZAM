/* Private candidates, included after bridge storage. No public library API.
 * MIT; Copyright (c) 2026 SH2ZAM Contributors. */
static inline v128_t extract16(v128_t low,v128_t high) {
    /* Select bits 16..47 from each signed 64-bit lane. No i64 vector shift,
     * division or i64x2.mul; the prepared bound proves the final int32 fits. */
    return wasm_i8x16_shuffle(low,high,
        2,3,4,5,10,11,12,13,18,19,20,21,26,27,28,29);
}
static inline v128_t wide_row(v128_t x,v128_t y,v128_t z,
    v128_t a,v128_t b,v128_t c,v128_t t) {
    const v128_t low=wasm_i64x2_add(wasm_i64x2_add(
        wasm_i64x2_extmul_low_i32x4(x,a),wasm_i64x2_extmul_low_i32x4(y,b)),
        wasm_i64x2_extmul_low_i32x4(z,c));
    const v128_t high=wasm_i64x2_add(wasm_i64x2_add(
        wasm_i64x2_extmul_high_i32x4(x,a),wasm_i64x2_extmul_high_i32x4(y,b)),
        wasm_i64x2_extmul_high_i32x4(z,c));
    return wasm_i32x4_add(extract16(low,high),t);
}
static inline void store_aos(sh2z_vec3_t *out,v128_t x,v128_t y,v128_t z) {
    const v128_t a=wasm_i32x4_shuffle(x,y,0,4,1,5);
    const v128_t b=wasm_i32x4_shuffle(x,y,2,6,3,7);
    const v128_t c=wasm_i32x4_shuffle(z,b,1,4,5,2);
    wasm_v128_store((char *)out,wasm_i32x4_shuffle(a,z,0,1,4,2));
    wasm_v128_store((char *)out+16,wasm_i32x4_shuffle(a,c,3,4,5,6));
    wasm_v128_store((char *)out+32,wasm_i32x4_shuffle(z,b,2,6,7,3));
}
static inline void finish_tail(sh2z_vec3_t *out,unsigned i) {
    (void)sh2z_transform_plan_execute_range(out+i,&plan,i,count-i);
}
#define MATRIX_SPLATS \
 const v128_t a=wasm_i32x4_splat(fixed_matrix.m[0][0]),b=wasm_i32x4_splat(fixed_matrix.m[0][1]),c=wasm_i32x4_splat(fixed_matrix.m[0][2]),d=wasm_i32x4_splat(fixed_matrix.m[0][3]); \
 const v128_t e=wasm_i32x4_splat(fixed_matrix.m[1][0]),f=wasm_i32x4_splat(fixed_matrix.m[1][1]),g=wasm_i32x4_splat(fixed_matrix.m[1][2]),h=wasm_i32x4_splat(fixed_matrix.m[1][3]); \
 const v128_t j=wasm_i32x4_splat(fixed_matrix.m[2][0]),k=wasm_i32x4_splat(fixed_matrix.m[2][1]),l=wasm_i32x4_splat(fixed_matrix.m[2][2]),m=wasm_i32x4_splat(fixed_matrix.m[2][3]);

API("wide") void wide(void) {
    if(!plan.fast_path){(void)sh2z_transform_plan_execute(wide_output,&plan);return;}
    MATRIX_SPLATS
    unsigned i=0;
    for(;i+4<=count;i+=4) {
        const v128_t p=wasm_v128_load(input+i);
        const v128_t q=wasm_v128_load((const char *)(input+i)+16);
        const v128_t r=wasm_v128_load((const char *)(input+i)+32);
        const v128_t tx=wasm_i32x4_shuffle(p,q,0,3,6,0);
        const v128_t ty=wasm_i32x4_shuffle(p,q,1,4,7,1);
        const v128_t tz=wasm_i32x4_shuffle(p,q,2,5,2,5);
        const v128_t x=wasm_i32x4_shuffle(tx,r,0,1,2,5);
        const v128_t y=wasm_i32x4_shuffle(ty,r,0,1,2,6);
        const v128_t z=wasm_i32x4_shuffle(tz,r,0,1,4,7);
        store_aos(wide_output+i,wide_row(x,y,z,a,b,c,d),
            wide_row(x,y,z,e,f,g,h),wide_row(x,y,z,j,k,l,m));
    }
    finish_tail(wide_output,i);
}
API("packed") void packed(void) {
    if(!plan.fast_path){(void)sh2z_transform_plan_execute(packed_output,&plan);return;}
    MATRIX_SPLATS
    unsigned i=0;
    for(;i+4<=count;i+=4) {
        const v128_t x=wasm_v128_load(packed_input+3*i);
        const v128_t y=wasm_v128_load(packed_input+3*i+4);
        const v128_t z=wasm_v128_load(packed_input+3*i+8);
        store_aos(packed_output+i,wide_row(x,y,z,a,b,c,d),
            wide_row(x,y,z,e,f,g,h),wide_row(x,y,z,j,k,l,m));
    }
    finish_tail(packed_output,i);
}
#undef MATRIX_SPLATS

API("rows") void rows(void) {
    if(!plan.fast_path){(void)sh2z_transform_plan_execute(rows_output,&plan);return;}
    const v128_t a=wasm_i32x4_make(fixed_matrix.m[0][0],fixed_matrix.m[1][0],fixed_matrix.m[2][0],0);
    const v128_t b=wasm_i32x4_make(fixed_matrix.m[0][1],fixed_matrix.m[1][1],fixed_matrix.m[2][1],0);
    const v128_t c=wasm_i32x4_make(fixed_matrix.m[0][2],fixed_matrix.m[1][2],fixed_matrix.m[2][2],0);
    const v128_t t=wasm_i32x4_make(fixed_matrix.m[0][3],fixed_matrix.m[1][3],fixed_matrix.m[2][3],0);
    for(unsigned i=0;i<count;i++) {
        const v128_t x=wasm_i32x4_splat(input[i].x);
        const v128_t y=wasm_i32x4_splat(input[i].y);
        const v128_t z=wasm_i32x4_splat(input[i].z);
        const v128_t result=wide_row(x,y,z,a,b,c,t);
        wasm_v128_store64_lane(rows_output+i,result,0);
        rows_output[i].z=wasm_i32x4_extract_lane(result,2);
    }
}

/* Write m=mh*65536+ms and x=xh*65536+xs, with signed 16-bit halves.
 * The exact row is:
 *   sum(ms*xh + mh*xs) + 65536*sum(mh*xh) + floor(sum(ms*xs)/65536) + t.
 * WASM i32 additions/multiplications wrap, and intermediate integer terms may
 * wrap; the eligible final result fits int32 and their low bits remain exact.
 * The fractional pair must not overflow signed int32 before its floor. */
API("split_ok") int split_ok(void) {
    if(!plan.fast_path||!split_input_fits)return 0;
    for(unsigned r=0;r<3;r++) {
        for(unsigned c=0;c<3;c++) {
            const int32_t raw=fixed_matrix.m[r][c],low=low_signed16(raw);
            const int64_t high=((int64_t)raw-low)/65536;
            if(high<INT16_MIN||high>INT16_MAX)return 0;
        }
        /* The only overflow of two signed16 products is (-32768)^2 twice. */
        if(low_signed16(fixed_matrix.m[r][0])==INT16_MIN&&
           low_signed16(fixed_matrix.m[r][1])==INT16_MIN)return 0;
    }
    return 1;
}
API("narrow_ok") int narrow_ok(void) {
    if(!split_ok())return 0;
    for(unsigned r=0;r<3;r++) {
        uint32_t bound=0;
        for(unsigned c=0;c<3;c++) {
            const int32_t low=low_signed16(fixed_matrix.m[r][c]);
            bound+=(uint32_t)(low<0?-low:low);
        }
        if(bound>65535)return 0;
    }
    return 1;
}
static inline v128_t dot_pair(v128_t a,v128_t b,v128_t xy,v128_t z) {
    return wasm_i32x4_add(wasm_i32x4_dot_i16x8(a,xy),
                         wasm_i32x4_dot_i16x8(b,z));
}
static inline __attribute__((always_inline)) v128_t split_row(v128_t lowxy,v128_t lowz,
    v128_t highxy,v128_t highz,const int32_t *row,bool narrow) {
    const int32_t as=low_signed16(row[0]),bs=low_signed16(row[1]);
    const int32_t cs=low_signed16(row[2]);
    const int16_t ah=(int16_t)(((int64_t)row[0]-as)/65536);
    const int16_t bh=(int16_t)(((int64_t)row[1]-bs)/65536);
    const int16_t ch=(int16_t)(((int64_t)row[2]-cs)/65536);
    const v128_t lo=wasm_i16x8_make((int16_t)as,(int16_t)bs,(int16_t)as,(int16_t)bs,(int16_t)as,(int16_t)bs,(int16_t)as,(int16_t)bs);
    const v128_t lz=wasm_i16x8_make((int16_t)cs,0,(int16_t)cs,0,(int16_t)cs,0,(int16_t)cs,0);
    const v128_t hi=wasm_i16x8_make(ah,bh,ah,bh,ah,bh,ah,bh);
    const v128_t hz=wasm_i16x8_make(ch,0,ch,0,ch,0,ch,0);
    const v128_t p=wasm_i32x4_dot_i16x8(lo,lowxy);
    const v128_t q=wasm_i32x4_dot_i16x8(lz,lowz);
    v128_t fraction;
    if(narrow) {
        /* sum(abs(ms))<=65535 makes this raw residual sum fit int32. */
        fraction=wasm_i32x4_shr(wasm_i32x4_add(p,q),16);
    } else {
        const v128_t mask=wasm_i32x4_splat(65535);
        const v128_t carry=wasm_u32x4_shr(wasm_i32x4_add(
            wasm_v128_and(p,mask),wasm_v128_and(q,mask)),16);
        fraction=wasm_i32x4_add(wasm_i32x4_add(
            wasm_i32x4_shr(p,16),wasm_i32x4_shr(q,16)),carry);
    }
    const v128_t cross=wasm_i32x4_add(dot_pair(lo,lz,highxy,highz),
                                     dot_pair(hi,hz,lowxy,lowz));
    const v128_t integer=wasm_i32x4_shl(dot_pair(hi,hz,highxy,highz),16);
    return wasm_i32x4_add(wasm_i32x4_add(cross,integer),
        wasm_i32x4_add(fraction,wasm_i32x4_splat(row[3])));
}
static inline __attribute__((always_inline)) void
split_kernel(sh2z_vec3_t *restrict out,bool narrow) {
    unsigned i=0;
    for(;i+4<=count;i+=4) {
        const v128_t lowxy=wasm_v128_load(split_input+8*i);
        const v128_t lowz=wasm_v128_load(split_input+8*i+8);
        const v128_t highxy=wasm_v128_load(split_input+8*i+16);
        const v128_t highz=wasm_v128_load(split_input+8*i+24);
        store_aos(out+i,
            split_row(lowxy,lowz,highxy,highz,fixed_matrix.m[0],narrow),
            split_row(lowxy,lowz,highxy,highz,fixed_matrix.m[1],narrow),
            split_row(lowxy,lowz,highxy,highz,fixed_matrix.m[2],narrow));
    }
    finish_tail(out,i);
}
API("split") void split(void) {
    if(!split_ok()){(void)sh2z_transform_plan_execute(split_output,&plan);return;}
    split_kernel(split_output,false);
}
API("narrow") void narrow(void) {
    if(!split_ok()){(void)sh2z_transform_plan_execute(narrow_output,&plan);return;}
    if(narrow_ok())split_kernel(narrow_output,true);
    else split_kernel(narrow_output,false);
}
